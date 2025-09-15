#pragma once
#ifndef SNTP_CLIENT_H
#define SNTP_CLIENT_H

#include "stdint.h"

#include "esp_err.h"

#include "sntp_client_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t
sntp_client_init(struct sntp_client_config *sntp_cfg, sntp_client_handle *out_client);
esp_err_t
sntp_client_del(sntp_client_handle sntp_client);

esp_err_t
sntp_client_start(sntp_client_handle sntp_client);
esp_err_t
sntp_client_stop(sntp_client_handle sntp_client);

enum sntp_client_sync_state
sntp_client_get_state(sntp_client_handle sntp_client);

esp_err_t
sntp_client_push_server(sntp_client_handle sntp_client, const char *server);
esp_err_t
sntp_client_pop_server(sntp_client_handle sntp_client);

esp_err_t
sntp_client_sync_wait(sntp_client_handle sntp_client, uint32_t time_to_wait_ms);

uint64_t
sntp_client_get_boot_posix_time_us(sntp_client_handle sntp_client);

#ifdef __cplusplus
}
#endif

#endif