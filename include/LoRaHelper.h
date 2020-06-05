#ifndef __LORAHELPER_H
#define __LORAHELPER_H

#include <inttypes.h>

enum lora_event_id_t : uint8_t { LORA_EVENT_INIT, LORA_EVENT_DEINIT, LORA_EVENT_START, LORA_EVENT_STOP, LORA_EVENT_DATA };

typedef struct lora_helper_t *lora_handle_t;

struct __attribute__((__packed__)) lora_packet_t {
  int16_t rssi;
  float snr;
  uint16_t packet_num;
  bool nack;
  uint8_t data_len;
  uint8_t *data;
};

struct __attribute__((__packed__)) lora_event_t {
  lora_handle_t client;
  lora_event_id_t event_id;
  lora_packet_t *packet;
};

typedef void (*lora_event_handler_t)(lora_event_t *event);

struct __attribute__((__packed__)) lora_config_t {
  int8_t pin_ss;
  int8_t pin_rst;
  int8_t pin_dio0;
  lora_event_handler_t event_handler;
  uint32_t frequency;
  uint32_t bandwidth;
  uint8_t spreading_factor;
  uint8_t coding_rate4;
  uint8_t tx_power;
  uint8_t sync_word;
  bool enable_crc;
};

#define DEFAULT_LORA_CONFIG(ss, rst, evt_handler, freq) { .pin_ss = (ss), .pin_rst = (rst), .pin_dio0 = -1, \
  .event_handler = evt_handler, .frequency = (freq), .bandwidth = 125000, .spreading_factor = 7, .coding_rate4 = 5, \
  .tx_power = 17, .sync_word = 0x55, .enable_crc = true }

lora_handle_t lora_init(const lora_config_t *config);
void lora_deinit(lora_handle_t handle);

bool lora_start(lora_handle_t handle);
bool lora_stop(lora_handle_t handle);

bool lora_send(lora_handle_t handle, const uint8_t *data, uint8_t data_len);
bool lora_send_reliable(lora_handle_t handle, const uint8_t *data, uint8_t data_len, uint8_t repeat = 2);

bool lora_read(lora_handle_t handle, uint8_t *data, uint8_t &data_len, uint16_t *packet_num = NULL, bool *nack = NULL);

#endif
