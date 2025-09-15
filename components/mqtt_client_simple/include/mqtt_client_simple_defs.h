#pragma once
#ifndef MQTT_CLIENT_SIMPLE_DEFS_H
#define MQTT_CLIENT_SIMPLE_DEFS_H

#include "esp_err.h"

#include "mqtt_client.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_ERR_MQTT_CLIENT_SIMPLE_BASE 0x7C00U

#define ESP_ERR_MQTT_CLIENT_SIMPLE_INIT_FAIL      (ESP_ERR_MQTT_CLIENT_SIMPLE_BASE + 1U)
#define ESP_ERR_MQTT_CLIENT_SIMPLE_NOT_INIT       (ESP_ERR_MQTT_CLIENT_SIMPLE_BASE + 2U)
#define ESP_ERR_INVALID_EVENT_HANDLER             (ESP_ERR_MQTT_CLIENT_SIMPLE_BASE + 3U)
#define ESP_ERR_MQTT_PUBLISH_FAILED               (ESP_ERR_MQTT_CLIENT_SIMPLE_BASE + 4U)
#define ESP_ERR_MQTT_ENQUEUE_FAILED               (ESP_ERR_MQTT_CLIENT_SIMPLE_BASE + 5U)
#define ESP_ERR_MQTT_SUBSCRIBE_FAILED             (ESP_ERR_MQTT_CLIENT_SIMPLE_BASE + 6U)
#define ESP_ERR_MQTT_UNSUBSCRIBE_FAILED           (ESP_ERR_MQTT_CLIENT_SIMPLE_BASE + 7U)
#define ESP_ERR_MQTT_CLIENT_SIMPLE_INVALID_STATE  (ESP_ERR_MQTT_CLIENT_SIMPLE_BASE + 8U)
#define ESP_ERR_MQTT_CLIENT_SIMPLE_CONNECT_FAILED (ESP_ERR_MQTT_CLIENT_SIMPLE_BASE + 9U)

#define ESP_ERR_MQTT_CLIENT_SIMPLE_END 0x7D00U

typedef struct mqtt_client_simple_instance *mqtt_client_simple_handle;

typedef esp_err_t (*mqtt_event_handler_t)(mqtt_client_simple_handle mqtt_client_simple, int32_t event_id, esp_mqtt_event_handle_t event_handle);

enum mqtt_client_simple_state {
  MQTT_CLIENT_SIMPLE_STATE_CONNECTION_TIMEOUT = -4,
  MQTT_CLIENT_SIMPLE_STATE_CONNECTION_LOST,
  MQTT_CLIENT_SIMPLE_STATE_CONNECT_FAILED,
  MQTT_CLIENT_SIMPLE_STATE_DISCONNECTED,
  MQTT_CLIENT_SIMPLE_STATE_CONNECTED,
  MQTT_CLIENT_SIMPLE_STATE_CONNECT_BAD_PROTOCOL,
  MQTT_CLIENT_SIMPLE_STATE_CONNECT_BAD_CLIENT_ID,
  MQTT_CLIENT_SIMPLE_STATE_CONNECT_UNAVAILABLE,
  MQTT_CLIENT_SIMPLE_STATE_CONNECT_BAD_CREDENTIALS,
  MQTT_CLIENT_SIMPLE_STATE_CONNECT_UNAUTHORIZED,
  MQTT_CLIENT_SIMPLE_STATE_CONNECT_TRANSPORT_ERROR,
};

enum mqtt_client_simple_event_type {
  MQTT_CLIENT_SIMPLE_ERROR_EVENT,
  MQTT_CLIENT_SIMPLE_DATA_EVENT,
  MQTT_CLIENT_SIMPLE_CONNECTION_EVENT,
  MQTT_CLIENT_SIMPLE_PUBLISH_EVENT,
  MQTT_CLIENT_SIMPLE_SUBSCRIBE_EVENT,
  MQTT_CLIENT_SIMPLE_DELETED_EVENT,
  MQTT_CLIENT_SIMPLE_CUSTOM_EVENT
  // Custom events
};

/**
 * Broker address
 *
 *  - uri have precedence over other fields
 *  - If uri isn't set at least hostname, transport and port should.
 */
struct mqtt_address_config {
  const char *uri;
  const char *hostname;
  esp_mqtt_transport_t transport;
  const char *path;
  uint32_t port;
};

/**
 * Broker identity verification
 *
 * If fields are not set broker's identity isn't verified. it's recommended
 * to set the options in this struct for security reasons.
 */
struct mqtt_verification_config {
  bool use_global_ca_store;
  esp_err_t (*crt_bundle_attach)(void *conf);
  const char *certificate;
  size_t certificate_len;
  const struct psk_key_hint *psk_hint_key;
  bool skip_cert_common_name_check;
  const char **alpn_protos;
  const char *common_name;
};

/**
 * Client related credentials for authentication.
 */
struct mqtt_credentials_config {
  const char *username;
  const char *client_id;
  bool set_null_client_id;
};

/**
 * Client authentication
 *
 * Fields related to client authentication by broker
 *
 * For mutual authentication using TLS, user could select certificate and key,
 * secure element or digital signature peripheral if available.
 *
 */
struct mqtt_auth_config {
  const char *password;
  const char *certificate;
  size_t certificate_len;
  const char *key;
  size_t key_len;
  const char *key_password;
  int key_password_len;
  bool use_secure_element;
  void *ds_data;
};

/**
 * Last Will and Testament message configuration.
 */
struct mqtt_last_will_config {
  const char *topic;
  const char *msg;
  int msg_len;
  int qos;
  int retain;
};

/**
 * *MQTT* Session related configuration
 */
struct mqtt_session_config {
  bool disable_clean_session;
  int keepalive;
  bool disable_keepalive;
  esp_mqtt_protocol_ver_t protocol_ver;
  int message_retransmit_timeout;
};

/**
 * Network related configuration
 */
struct mqtt_network_config {
  int reconnect_timeout_ms;
  int timeout_ms;
  int refresh_connection_after_ms;
  bool disable_auto_reconnect;
  esp_transport_handle_t transport;
  struct ifreq *if_name;
};

/**
 * Client task configuration
 */
struct mqtt_task_config {
  int priority;
  int stack_size;
};

/**
 * Client buffer size configuration
 *
 * Client have two buffers for input and output respectivelly.
 */
struct mqtt_buffer_config {
  int size;
  int out_size;
};

/**
 * Client outbox configuration options.
 */
struct mqtt_outbox_config {
  uint64_t limit;
};

#ifdef __cplusplus
}
#endif
#endif
