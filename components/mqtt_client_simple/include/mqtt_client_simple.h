#pragma once
#ifndef MQTT_CLIENT_SIMPLE_H
#define MQTT_CLIENT_SIMPLE_H

#include "mqtt_client_simple_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t
mqtt_client_simple_init(mqtt_client_simple_handle *out_client); // Initial configuration?
esp_err_t
mqtt_client_simple_del(mqtt_client_simple_handle client);

/**
 * Connect with preconfigured connection information if available
 */
esp_err_t
mqtt_client_simple_connect(mqtt_client_simple_handle client, uint32_t timeout_ms);
esp_err_t
mqtt_client_simple_connect_with_cred(mqtt_client_simple_handle client, const char *username, const char *password, uint32_t timeout_ms);
esp_err_t
mqtt_client_simple_connect_with_uri(mqtt_client_simple_handle client, const char *uri, uint32_t timeout_ms);

esp_err_t mqtt_client_simple_reconnect(mqtt_client_simple_handle client, uint32_t timeout_ms);

esp_err_t
mqtt_client_simple_disconnect(mqtt_client_simple_handle client);

esp_err_t
mqtt_client_simple_publish(mqtt_client_simple_handle client, const char *topic, const char *payload, uint32_t payload_len, uint8_t qos, bool retain);
esp_err_t
mqtt_client_simple_enqueue(mqtt_client_simple_handle client, const char *topic, const char *payload, uint8_t qos, bool retain);

esp_err_t
mqtt_client_simple_subscribe(mqtt_client_simple_handle client, const char *topic, uint8_t qos);
esp_err_t
mqtt_client_simple_unsubscribe(mqtt_client_simple_handle client, const char *topic);

esp_err_t
mqtt_client_simple_set_config(mqtt_client_simple_handle client, const esp_mqtt_client_config_t *config);

esp_err_t
mqtt_client_simple_set_address_cfg(mqtt_client_simple_handle client, const struct mqtt_address_config *addr_cfg);
esp_err_t
mqtt_client_simple_set_verification_cfg(mqtt_client_simple_handle client, const struct mqtt_verification_config *verif_cfg);
esp_err_t
mqtt_client_simple_set_credentials_cfg(mqtt_client_simple_handle client, const struct mqtt_credentials_config *cred_cfg);
esp_err_t
mqtt_client_simple_set_authentication_cfg(mqtt_client_simple_handle client, const struct mqtt_auth_config *auth_cfg);
esp_err_t
mqtt_client_simple_set_session_cfg(mqtt_client_simple_handle client, const struct mqtt_session_config *session_cfg);
esp_err_t
mqtt_client_simple_set_last_will_cfg(mqtt_client_simple_handle client, const struct mqtt_last_will_config *lwt_cfg);
esp_err_t
mqtt_client_simple_set_network_cfg(mqtt_client_simple_handle client, const struct mqtt_network_config *net_cfg);
esp_err_t
mqtt_client_simple_set_task_cfg(mqtt_client_simple_handle client, const struct mqtt_task_config *task_cfg);
esp_err_t
mqtt_client_simple_set_buffer_cfg(mqtt_client_simple_handle client, const struct mqtt_buffer_config *buffer_cfg);
esp_err_t
mqtt_client_simple_set_outbox_cfg(mqtt_client_simple_handle client, const struct mqtt_outbox_config *outbox_cfg);

esp_err_t
mqtt_client_simple_register_event_handler(mqtt_client_simple_handle client, enum mqtt_client_simple_event_type event, mqtt_event_handler_t event_handler);
esp_err_t
mqtt_client_simple_unregister_event_handler(mqtt_client_simple_handle client, enum mqtt_client_simple_event_type event);

enum mqtt_client_simple_state
mqtt_client_simple_get_status(mqtt_client_simple_handle client);

#ifdef __cplusplus
}
#endif
#endif