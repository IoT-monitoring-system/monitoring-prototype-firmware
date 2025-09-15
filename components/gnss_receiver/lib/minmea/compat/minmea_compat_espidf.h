#ifndef MINMEA_COMPAT_H_
#define MINMEA_COMPAT_H_

#if defined(ESP_PLATFORM)
#include <time.h>
#endif /* __TI_ARM__ */

#define timegm mktime

#endif /* MINMEA_COMPAT_H */

/* vim: set ts=4 sw=4 et: */
