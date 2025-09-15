#pragma once
#ifndef GNSS_RECEIVER_H
#define GNSS_RECEIVER_H

#include "esp_err.h"

#include "driver/uart.h"
#include "soc/gpio_num.h"

#include "gnss_receiver_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t
gnss_receiver_init(const uart_config_t *uart_cfg, uart_port_t uart_port, gpio_num_t rx_pin, gpio_num_t tx_pin,
                 gnss_receiver_handle *out_receiver);
esp_err_t
gnss_receiver_del(gnss_receiver_handle receiver);

esp_err_t
gnss_receiver_start(gnss_receiver_handle receiver);
esp_err_t
gnss_receiver_stop(gnss_receiver_handle receiver);

esp_err_t
gnss_receiver_write_sentence(gnss_receiver_handle receiver, const struct gnss_receiver_sentence sentence);

esp_err_t
gnss_receiver_register_event_handler(gnss_receiver_handle receiver, gnss_receiver_event_handler handler);

esp_err_t
gnss_receiver_update_parse_sentences(gnss_receiver_handle receiver, enum minmea_sentence_id *parse_sentences,
                                   uint8_t parse_sentences_len);

#ifdef __cplusplus
}
#endif
#endif