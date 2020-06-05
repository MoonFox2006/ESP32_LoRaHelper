#include <esp_log.h>
#include <LoRa.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include "LoRaHelper.h"

struct __attribute__((__packed__)) lora_helper_t {
  TaskHandle_t lora_task;
  EventGroupHandle_t lora_flags;
  SemaphoreHandle_t ack_semaphore;
  portMUX_TYPE lora_mux;
  portMUX_TYPE packet_mux;
  lora_event_handler_t event_handler;
  lora_packet_t received_packet;
  uint16_t send_num;
  uint16_t ack_num;
  bool nack;
};

struct __attribute__((__packed__)) packet_header_t {
  uint16_t signature : 12;
  enum packettype_t { PACKET_NACK, PACKET_ACK, PACKET_DATA, PACKET_DATARELIABLE } type : 4;
  uint16_t num;
};

enum taskflags_t : uint8_t { FLAG_DESTROYED = 1, FLAG_STOPPED = 2 };

static const char TAG[] = "LoRa";

static const uint8_t LORA_TASK_PRIORITY = 1;

static const uint16_t PACKET_SIGNATURE = 0x0AAA;
static const uint32_t ACK_TIMEOUT = 100; // 100 ms.

static bool lora_send_packet(lora_handle_t handle, packet_header_t::packettype_t type, uint16_t num, const uint8_t *data, uint8_t data_len) {
  if (handle) {
    portENTER_CRITICAL(&handle->lora_mux);
    if (LoRa.beginPacket()) {
      packet_header_t header;

      header.signature = PACKET_SIGNATURE;
      header.type = type;
      header.num = num;
      if (LoRa.write((uint8_t*)&header, sizeof(header)) == sizeof(header)) {
        if (data && data_len) {
          if (LoRa.write(data, data_len) != data_len) {
            portEXIT_CRITICAL(&handle->lora_mux);
            ESP_LOGE(TAG, "Send packet error!");
            return false;
          }
        }
        if (LoRa.endPacket()) {
          portEXIT_CRITICAL(&handle->lora_mux);
          return true;
        }
      }
    }
    portEXIT_CRITICAL(&handle->lora_mux);
    ESP_LOGE(TAG, "Send packet error!");
  }
  return false;
}

static void lora_task(void *pvParam) {
  while (! (xEventGroupWaitBits(((lora_helper_t*)pvParam)->lora_flags, FLAG_STOPPED, pdFALSE, pdTRUE, pdMS_TO_TICKS(1)) & FLAG_STOPPED)) {
    int16_t packetSize;

    portENTER_CRITICAL(&((lora_helper_t*)pvParam)->lora_mux);
    packetSize = LoRa.parsePacket();
    if (packetSize) {
      if (packetSize >= sizeof(packet_header_t)) {
        packet_header_t header;

        if ((LoRa.readBytes((uint8_t*)&header, sizeof(header)) == sizeof(header)) && (header.signature == PACKET_SIGNATURE) && (header.type <= packet_header_t::PACKET_DATARELIABLE)) {
          if (header.type <= packet_header_t::PACKET_ACK) {
            ((lora_helper_t*)pvParam)->ack_num = header.num;
            ((lora_helper_t*)pvParam)->nack = header.type == packet_header_t::PACKET_NACK;
            xSemaphoreGive(((lora_helper_t*)pvParam)->ack_semaphore);
          } else { // DATA
            portENTER_CRITICAL(&((lora_helper_t*)pvParam)->packet_mux);
            ((lora_helper_t*)pvParam)->received_packet.rssi = LoRa.packetRssi();
            ((lora_helper_t*)pvParam)->received_packet.snr = LoRa.packetSnr();
            ((lora_helper_t*)pvParam)->received_packet.packet_num = header.num;
            ((lora_helper_t*)pvParam)->received_packet.nack = false;
            ((lora_helper_t*)pvParam)->received_packet.data_len = packetSize - sizeof(packet_header_t);
            if (((lora_helper_t*)pvParam)->received_packet.data) {
              vPortFree(((lora_helper_t*)pvParam)->received_packet.data);
              ((lora_helper_t*)pvParam)->received_packet.data = NULL;
            }
            if (((lora_helper_t*)pvParam)->received_packet.data_len) {
              ((lora_helper_t*)pvParam)->received_packet.data = (uint8_t*)pvPortMalloc(((lora_helper_t*)pvParam)->received_packet.data_len);
              if (((lora_helper_t*)pvParam)->received_packet.data) {
                if (LoRa.readBytes(((lora_helper_t*)pvParam)->received_packet.data, ((lora_helper_t*)pvParam)->received_packet.data_len) != ((lora_helper_t*)pvParam)->received_packet.data_len) {
                  vPortFree(((lora_helper_t*)pvParam)->received_packet.data);
                  ((lora_helper_t*)pvParam)->received_packet.data = NULL;
                  ESP_LOGE(TAG, "Read packet error!");
                }
              } else
                ESP_LOGE(TAG, "Memory allocation error!");
            }
            if (((lora_helper_t*)pvParam)->event_handler) {
              lora_event_t event;

              event.client = (lora_helper_t*)pvParam;
              event.event_id = LORA_EVENT_DATA;
              event.packet = &((lora_helper_t*)pvParam)->received_packet;
              ((lora_helper_t*)pvParam)->event_handler(&event);
            }
            if (header.type == packet_header_t::PACKET_DATARELIABLE) {
              lora_send_packet((lora_helper_t*)pvParam, ((lora_helper_t*)pvParam)->received_packet.nack ? packet_header_t::PACKET_NACK : packet_header_t::PACKET_ACK, header.num, NULL, 0);
            }
            portEXIT_CRITICAL(&((lora_helper_t*)pvParam)->packet_mux);
          }
        } else
          ESP_LOGW(TAG, "Wrong packet received!");
      } else
        ESP_LOGW(TAG, "Incomplete packet received!");
      // Skip remains of packet
      while (LoRa.available()) {
        (void)LoRa.read();
      }
    }
    portEXIT_CRITICAL(&((lora_helper_t*)pvParam)->lora_mux);
  }
  ((lora_helper_t*)pvParam)->lora_task = NULL;
  xEventGroupSetBits(((lora_helper_t*)pvParam)->lora_flags, FLAG_DESTROYED);
  vTaskDelete(NULL);
}

lora_handle_t lora_init(const lora_config_t *config) {
  lora_handle_t result = (lora_helper_t*)pvPortMalloc(sizeof(lora_helper_t));

  if (result) {
    result->lora_task = NULL;
    result->lora_flags = xEventGroupCreate();
    if (! result->lora_flags) {
      vPortFree(result);
      ESP_LOGE(TAG, "Flags creation error!");
      return NULL;
    }
    result->ack_semaphore = xSemaphoreCreateBinary();
    if (! result->ack_semaphore) {
      vEventGroupDelete(result->lora_flags);
      vPortFree(result);
      ESP_LOGE(TAG, "Acknowledge semaphore creation error!");
      return NULL;
    }
    result->lora_mux = portMUX_INITIALIZER_UNLOCKED;
    result->packet_mux = portMUX_INITIALIZER_UNLOCKED;
    result->event_handler = config->event_handler;
    LoRa.setPins(config->pin_ss, config->pin_rst, config->pin_dio0);
    if (! LoRa.begin(config->frequency)) {
      vSemaphoreDelete(result->ack_semaphore);
      vEventGroupDelete(result->lora_flags);
      vPortFree(result);
      ESP_LOGE(TAG, "LoRa.begin() error!");
      return NULL;
    }
    LoRa.setSignalBandwidth(config->bandwidth);
    LoRa.setSpreadingFactor(config->spreading_factor);
    LoRa.setCodingRate4(config->coding_rate4);
    LoRa.setTxPower(config->tx_power);
    LoRa.setSyncWord(config->sync_word);
    if (config->enable_crc)
      LoRa.enableCrc();
//    LoRa.receive();
    if (result->event_handler) {
      lora_event_t event;

      event.client = result;
      event.event_id = LORA_EVENT_INIT;
      event.packet = NULL;
      result->event_handler(&event);
    }
  } else
    ESP_LOGE(TAG, "Memory allocation error!");
  return result;
}

void lora_deinit(lora_handle_t handle) {
  if (handle) {
    lora_stop(handle);
    if (handle->event_handler) {
      lora_event_t event;

      event.client = handle;
      event.event_id = LORA_EVENT_DEINIT;
      event.packet = NULL;
      handle->event_handler(&event);
    }
    vSemaphoreDelete(handle->ack_semaphore);
    vEventGroupDelete(handle->lora_flags);
    vPortFree(handle);
    LoRa.end();
  }
}

bool lora_start(lora_handle_t handle) {
  if (handle && (! handle->lora_task)) {
    handle->send_num = 0;
    handle->ack_num = 0;
    xEventGroupClearBits(handle->lora_flags, FLAG_DESTROYED | FLAG_STOPPED);
    if (xTaskCreate(&lora_task, "LoRa", 4096, handle, LORA_TASK_PRIORITY, &handle->lora_task) == pdPASS) {
      portENTER_CRITICAL(&handle->packet_mux);
      memset(&handle->received_packet, 0, sizeof(handle->received_packet));
      portEXIT_CRITICAL(&handle->packet_mux);
      if (handle->event_handler) {
        lora_event_t event;

        event.client = handle;
        event.event_id = LORA_EVENT_START;
        event.packet = NULL;
        handle->event_handler(&event);
      }
      return true;
    } else
      ESP_LOGE(TAG, "Task creation error!");
  }
  return false;
}

bool lora_stop(lora_handle_t handle) {
  if (handle && handle->lora_task) {
    if (handle->event_handler) {
      lora_event_t event;

      event.client = handle;
      event.event_id = LORA_EVENT_STOP;
      event.packet = NULL;
      handle->event_handler(&event);
    }
    xEventGroupSetBits(handle->lora_flags, FLAG_STOPPED);
    xEventGroupWaitBits(handle->lora_flags, FLAG_DESTROYED, pdFALSE, pdTRUE, portMAX_DELAY);
    portENTER_CRITICAL(&handle->packet_mux);
    if (handle->received_packet.data) {
      vPortFree(handle->received_packet.data);
      handle->received_packet.data = NULL;
    }
    portEXIT_CRITICAL(&handle->packet_mux);
    return true;
  }
  return false;
}

bool lora_send(lora_handle_t handle, const uint8_t *data, uint8_t data_len) {
  if (handle && handle->lora_task) {
    if (++handle->send_num == 0)
      handle->send_num = 1;
    return lora_send_packet(handle, packet_header_t::PACKET_DATA, handle->send_num, data, data_len);
  }
  return false;
}

bool lora_send_reliable(lora_handle_t handle, const uint8_t *data, uint8_t data_len, uint8_t repeat) {
  if (handle && handle->lora_task) {
    if (++handle->send_num == 0)
      handle->send_num = 1;
    do {
      if (lora_send_packet(handle, packet_header_t::PACKET_DATARELIABLE, handle->send_num, data, data_len)) {
        if ((xSemaphoreTake(handle->ack_semaphore, pdMS_TO_TICKS(ACK_TIMEOUT)) == pdTRUE) && (handle->ack_num == handle->send_num))
          return (! handle->nack);
      }
    } while (repeat--);
  }
  return false;
}

bool lora_read(lora_handle_t handle, uint8_t *data, uint8_t &data_len, uint16_t *packet_num, bool *nack) {
  if (handle && handle->lora_task) {
    portENTER_CRITICAL(&handle->packet_mux);
    if (data) {
      if (handle->received_packet.data_len < data_len)
        data_len = handle->received_packet.data_len;
      if (data_len && handle->received_packet.data)
        memcpy(data, handle->received_packet.data, data_len);
    } else {
      data_len = handle->received_packet.data_len;
    }
    if (packet_num)
      *packet_num = handle->received_packet.packet_num;
    if (*nack)
      *nack = handle->received_packet.nack;
    portEXIT_CRITICAL(&handle->packet_mux);
    return (data_len > 0);
  }
  return false;
}
