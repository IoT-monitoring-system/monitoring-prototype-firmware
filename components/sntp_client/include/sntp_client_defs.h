#pragma once
#ifndef SNTP_CLIENT_DEFS_H
#define SNTP_CLIENT_DEFS_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*sntp_client_time_cb_t)(struct timeval *tv);

enum sntp_client_sync_type {
  SNTP_CLIENT_SYNC_IMMED,
  SNTP_CLIENT_SYNC_SMOOTH,
};

enum sntp_client_sync_state {
  SNTP_CLIENT_STATE_INVALID,
  SNTP_CLIENT_STATE_INITIALIZED,
  SNTP_CLIENT_STATE_IN_PROGRESS,
  SNTP_CLIENT_STATE_SYNCHRONIZED,
};

struct sntp_client_config {
  // enum sntp_module_sync_type sync_type;
  // sntp_module_time_cb_t time_sync_cb;
  const char *server;
};

typedef struct sntp_client_instance *sntp_client_handle;

#ifdef __cplusplus
}
#endif

#endif