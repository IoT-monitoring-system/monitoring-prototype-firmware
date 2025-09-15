#pragma once
#ifndef GNSS_RECEIVER_DEFS_H
#define GNSS_RECEIVER_DEFS_H

#include "minmea/minmea.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct gnss_receiver_instance *gnss_receiver_handle;

struct gnss_receiver_sentence {
  enum minmea_sentence_id sentence_id;
  union {
    struct minmea_sentence_gbs gbs;
    struct minmea_sentence_rmc rmc;
    struct minmea_sentence_gga gga;
    struct minmea_sentence_gll gll;
    struct minmea_sentence_gst gst;
    struct minmea_sentence_gsa gsa;
    struct minmea_sentence_gsv gsv;
    struct minmea_sentence_vtg vtg;
    struct minmea_sentence_zda zda;
  } sentence;
};

typedef void (*gnss_receiver_event_handler)(struct gnss_receiver_sentence);

#ifdef __cplusplus
}
#endif
#endif