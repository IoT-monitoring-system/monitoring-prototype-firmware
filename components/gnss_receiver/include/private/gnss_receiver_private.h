#pragma once
#ifndef GNSS_RECEIVER_PRIVATE_H
#define GNSS_RECEIVER_PRIVATE_H

#ifdef __cplusplus
extern "C" {
#endif

#define UART_RX_BUF_SIZE ((2048U))

#define GNSS_RECEIVER_NMEA_TASK_STACK_SIZE 3144U
#define GNSS_RECEIVER_NMEA_TASK_PRIORITY   15U
#define GNSS_RECEIVER_NMEA_TASK_CORE       1U
#define GNSS_RECEIVER_NMEA_TASK_PERIOD_MS  500U

#ifdef __cplusplus
}
#endif
#endif