#pragma once
#ifndef GNSS_POSITION_MODULE_CONFIG_H
#define GNSS_POSITION_MODULE_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#define TASK_QUEUE_SEND_TIMEOUT_MS 10000U

#define TASK_GNSS_SAMPLING_REPORT_PERIOD_MS 5000U
#define TASK_GNSS_SAMPLING_QUEUE_SIZE       10U

#ifdef __cplusplus
}
#endif
#endif