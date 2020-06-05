#include <Arduino.h>
#include <SPI.h>
#include "LoRaHelper.h"

const uint8_t LORA_SCK_PIN = 5;
const uint8_t LORA_MOSI_PIN = 27;
const uint8_t LORA_MISO_PIN = 19;
const uint8_t LORA_CS_PIN = 18;
const uint8_t LORA_RST_PIN = 14;

static void loraEvent(lora_event_t *event);

const lora_config_t config = DEFAULT_LORA_CONFIG(LORA_CS_PIN, LORA_RST_PIN, loraEvent, 868000000);

lora_handle_t lora;

static void halt(const char *msg) {
  Serial.println(msg);
  Serial.println("System halted!");
  Serial.flush();
  digitalWrite(LED_BUILTIN, LOW);
  esp_deep_sleep_start();
}

static void restart(const char *msg) {
  Serial.println(msg);
  Serial.println("System restarting...");
  Serial.flush();
  digitalWrite(LED_BUILTIN, LOW);
  esp_restart();
}

static void loraEvent(lora_event_t *event) {
  digitalWrite(LED_BUILTIN, HIGH);
  switch (event->event_id) {
    case LORA_EVENT_INIT:
      Serial.println("LoRa initialized");
      break;
    case LORA_EVENT_DEINIT:
      Serial.println("LoRa deinitialized");
      break;
    case LORA_EVENT_START:
      Serial.println("LoRa started");
      break;
    case LORA_EVENT_STOP:
      Serial.println("LoRa stoped");
      break;
    case LORA_EVENT_DATA:
      Serial.printf("LoRa data packet #%u with RSSI %d, SNR %0.2f and length %u byte(s) received\r\n",
        event->packet->packet_num, event->packet->rssi, event->packet->snr, event->packet->data_len);
      for (uint8_t i = 0; i < event->packet->data_len; ++i) {
        if (i)
          Serial.print(' ');
        Serial.printf("%02X", event->packet->data[i]);
      }
      Serial.println();
      break;
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  Serial.begin(115200);
  Serial.println();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  SPI.begin(LORA_SCK_PIN, LORA_MISO_PIN, LORA_MOSI_PIN, LORA_CS_PIN); // LoRa on SPI
  lora = lora_init(&config);
  if (! lora)
    halt("LoRa init fail!");
  if (! lora_start(lora))
    halt("LoRa start fail!");
  Serial.println("LoRa ready");
}

void loop() {
  const uint32_t SEND_PERIOD = 5000; // 5 sec.

  vTaskDelay(pdMS_TO_TICKS(SEND_PERIOD));
  {
    const uint8_t MAX_ERRORS = 5;

    static uint8_t errors = 0;

    uint32_t time = millis();

    digitalWrite(LED_BUILTIN, HIGH);
    if (! lora_send_reliable(lora, (uint8_t*)&time, sizeof(time))) {
      Serial.println("Error sending uptime!");
      if (++errors >= MAX_ERRORS) {
        Serial.println("Too many send errors!");
        lora_deinit(lora);
        lora = lora_init(&config);
        if (! lora)
          restart("LoRa init fail!");
        if (! lora_start(lora))
          restart("LoRa start fail!");
        Serial.println("LoRa ready");
        errors = 0;
      }
    } else
      errors = 0;
    digitalWrite(LED_BUILTIN, LOW);
  }
  Serial.printf("Free heap size is %u bytes\r\n", esp_get_free_heap_size());
}
