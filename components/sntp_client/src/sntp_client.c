#include "esp_check.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_timer.h"
#include "lwip/apps/sntp.h"

#include "private/sntp_client_private.h"
#include "sntp_client.h"
#include "sntp_client_defs.h"

struct sntp_client_instance {
  enum sntp_client_sync_state state;
  uint8_t last_add_server_idx;
  const char *sntp_servers[SNTP_CLIENT_MAX_SNTP_SERVERS]; // Not freed
  uint64_t boot_time_posix_us; /* The esp_timer_get_time API provides time in us since boot, some sensors record timestamp in time
                                  since boot. But it may be useful to convert the time since boot to a UTC posix timestamp, this
                                  variable can be used to accomplish that. */
};

static const char *TAG = "sntp_client";

// Cant use the callback, since it will overwrite the esp_netif internal one, which may break sync_wait
static void
sntp_client_time_callback(struct timeval *tv);

static uint64_t
calculate_boot_posix_time_us();

static esp_err_t
push_server(struct sntp_client_instance *sntp_instance, const char *server);

static esp_err_t
pop_server(struct sntp_client_instance *sntp_instance);

static esp_err_t
sntp_stop_api(void *ctx);

static void
sntp_client_cleanup(sntp_client_handle client);

esp_err_t
sntp_client_init(struct sntp_client_config *sntp_cfg, sntp_client_handle *out_client) {
  esp_err_t ret = ESP_OK;

  struct sntp_client_instance *client = NULL;

  ESP_GOTO_ON_FALSE(sntp_cfg, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");

  client = calloc(1, sizeof(struct sntp_client_instance));
  ESP_GOTO_ON_FALSE(client, ESP_ERR_NO_MEM, err, TAG, "failed to create sntp_client_instance");

  ESP_GOTO_ON_FALSE(client->state == SNTP_CLIENT_STATE_INVALID, ESP_ERR_INVALID_STATE, err, TAG, "invalid client state");

  esp_sntp_config_t esp_sntp_cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG(sntp_cfg->server);
  esp_sntp_cfg.start = false;
  ESP_GOTO_ON_ERROR(esp_netif_sntp_init(&esp_sntp_cfg), err, TAG, "failed to initialize esp_netif_sntp");
  client->last_add_server_idx = 0;

  ESP_GOTO_ON_ERROR(push_server(client, sntp_cfg->server), err, TAG, "failed adding an sntp server");
  client->state = SNTP_CLIENT_STATE_INITIALIZED;

  *out_client = client;

  return ESP_OK;
err:
  esp_netif_sntp_deinit();
  sntp_client_cleanup(client);
  return ret;
}
esp_err_t
sntp_client_del(sntp_client_handle sntp_client) {
  esp_err_t ret = ESP_OK;

  ESP_RETURN_ON_FALSE(sntp_client->state != SNTP_CLIENT_STATE_INVALID, ESP_ERR_INVALID_STATE, TAG, "invalid client state");

  esp_netif_sntp_deinit();
  sntp_client_cleanup(sntp_client);

  return ret;
}

esp_err_t
sntp_client_start(sntp_client_handle sntp_client) {
  esp_err_t ret = ESP_OK;

  ESP_RETURN_ON_FALSE(sntp_client, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_FALSE(sntp_client->state == SNTP_CLIENT_STATE_INITIALIZED, ESP_ERR_INVALID_STATE, TAG, "invalid client state");
  ESP_RETURN_ON_ERROR(esp_netif_sntp_start(), TAG, "failed to start esp_netif_sntp");

  sntp_client->state = SNTP_CLIENT_STATE_IN_PROGRESS;

  return ret;
}
esp_err_t
sntp_client_stop(sntp_client_handle sntp_client) {
  esp_err_t ret = ESP_OK;

  ESP_RETURN_ON_FALSE(sntp_client, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_ERROR(esp_netif_tcpip_exec(sntp_stop_api, NULL), TAG, "failed to stop esp_netif_sntp");
  sntp_client->state = SNTP_CLIENT_STATE_INITIALIZED;

  return ret;
}

enum sntp_client_sync_state
sntp_client_get_state(sntp_client_handle sntp_client) {
  if (NULL == sntp_client)
    return SNTP_CLIENT_STATE_INVALID;

  return sntp_client->state;
}

esp_err_t
sntp_client_push_server(sntp_client_handle sntp_client, const char *server) {
  ESP_RETURN_ON_FALSE(sntp_client, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  return push_server(sntp_client, server);
}
esp_err_t
sntp_client_pop_server(sntp_client_handle sntp_client) {
  ESP_RETURN_ON_FALSE(sntp_client, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  return pop_server(sntp_client);
}

esp_err_t
sntp_client_sync_wait(sntp_client_handle sntp_client, uint32_t time_to_wait_ms) {
  esp_err_t ret = ESP_OK;
  ESP_RETURN_ON_FALSE(sntp_client, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  ESP_RETURN_ON_ERROR(esp_netif_sntp_sync_wait(pdMS_TO_TICKS(time_to_wait_ms)), TAG, "failed to sync sntp");
  sntp_client->state = SNTP_CLIENT_STATE_SYNCHRONIZED;

  sntp_client->boot_time_posix_us = calculate_boot_posix_time_us();

  return ret;
}

uint64_t
sntp_client_get_boot_posix_time_us(sntp_client_handle sntp_client) {
  if (NULL == sntp_client)
    return 0;

  return sntp_client->boot_time_posix_us;
}

static uint64_t
calculate_boot_posix_time_us() {
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  uint64_t sntp_synced_utc_us = (uint64_t)tv_now.tv_sec * 1000000L + (uint64_t)tv_now.tv_usec;
  uint64_t boot_time_us = esp_timer_get_time();

  return (sntp_synced_utc_us - boot_time_us);
}

static void
sntp_client_time_callback(struct timeval *tv) {
}

static esp_err_t
push_server(struct sntp_client_instance *sntp_client_instance, const char *server) {
  ESP_RETURN_ON_FALSE(server && sntp_client_instance, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_RETURN_ON_FALSE(sntp_client_instance->last_add_server_idx < SNTP_CLIENT_MAX_SNTP_SERVERS, ESP_FAIL, TAG,
                      "failed to push a server, max number servers reached");
  sntp_client_instance->sntp_servers[sntp_client_instance->last_add_server_idx++] = server;

  return ESP_OK;
}

static esp_err_t
pop_server(struct sntp_client_instance *sntp_client_instance) {
  ESP_RETURN_ON_FALSE(sntp_client_instance, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
  ESP_RETURN_ON_FALSE(sntp_client_instance->last_add_server_idx > 0, ESP_FAIL, TAG, "failed to pop a server, invalid index");
  sntp_client_instance->sntp_servers[--sntp_client_instance->last_add_server_idx] = NULL;

  return ESP_OK;
}

static esp_err_t
sntp_stop_api(void *ctx) {
  sntp_stop();
  return ESP_OK;
}

static void
sntp_client_cleanup(sntp_client_handle client) {
  if (NULL == client)
    return;

  free(client);
}
