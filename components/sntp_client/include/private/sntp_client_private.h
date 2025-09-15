#pragma once
#ifndef SNTP_CLIENT_PRIVATE_H
#define SNTP_CLIENT_PRIVATE_H

#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SNTP_CLIENT_MAX_SNTP_SERVERS CONFIG_LWIP_SNTP_MAX_SERVERS

#ifdef __cplusplus
}
#endif

#endif