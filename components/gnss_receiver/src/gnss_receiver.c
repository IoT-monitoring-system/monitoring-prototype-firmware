#include "esp_check.h"
#include "memory.h"

#include "gnss_receiver.h"
#include "private/gnss_receiver_private.h"

static char *TAG = "gnss_receiver";

struct gnss_receiver_instance {
  TaskHandle_t nmea_uart_task;

  char rx_buf[UART_RX_BUF_SIZE];
  uint16_t rx_len;

  enum minmea_sentence_id parse_sentences[MINMEA_SENTENCE_MAX];
  uint16_t parse_sentences_len;

  gnss_receiver_event_handler event_handler;

  uart_config_t uart_cfg;
  uart_port_t uart_port;
  gpio_num_t rx_pin;
  gpio_num_t tx_pin;
};

static struct gnss_receiver_instance gnss_receiver_global;

static void
nmea_read_uart(uart_port_t uart_port, char *buf, uint16_t buf_size, uint32_t timeout_ms, enum minmea_sentence_id *parse_sentences,
               uint8_t parse_sentences_len, gnss_receiver_event_handler event_handler);

static void
handle_nmea_sentence(const char *sentence, uint16_t len, enum minmea_sentence_id *parse_sentences, uint8_t parse_sentences_len,
                     gnss_receiver_event_handler event_handler);

static void
nmea_uart_task(void *arg);

esp_err_t
gnss_receiver_init(const uart_config_t *uart_cfg, uart_port_t uart_port, gpio_num_t rx_pin, gpio_num_t tx_pin,
                 gnss_receiver_handle *out_receiver) {
  esp_err_t ret = ESP_OK;

  struct gnss_receiver_instance *receiver = &gnss_receiver_global;
  ESP_GOTO_ON_FALSE(out_receiver && uart_cfg && uart_port < UART_NUM_MAX && rx_pin < GPIO_NUM_MAX && tx_pin < GPIO_NUM_MAX,
                    ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");

  ESP_GOTO_ON_ERROR(uart_param_config(uart_port, uart_cfg), err, TAG, "failed to configure UART");
  ESP_GOTO_ON_ERROR(uart_set_pin(uart_port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), err, TAG,
                    "failed to set UART pin");
  ESP_GOTO_ON_ERROR(uart_driver_install(uart_port, UART_RX_BUF_SIZE, 0, 0, NULL, 0), err, TAG, "failed to install UART driver");

  ESP_GOTO_ON_FALSE(xTaskCreatePinnedToCore(nmea_uart_task, "nmea uart tsk", GNSS_RECEIVER_NMEA_TASK_STACK_SIZE, receiver,
                                            GNSS_RECEIVER_NMEA_TASK_PRIORITY, &receiver->nmea_uart_task,
                                            GNSS_RECEIVER_NMEA_TASK_CORE) == pdPASS,
                    ESP_FAIL, err, TAG, "failed to create NMEA uart task");

  gnss_receiver_global.rx_pin = rx_pin;
  gnss_receiver_global.tx_pin = tx_pin;
  gnss_receiver_global.uart_port = uart_port;
  gnss_receiver_global.uart_cfg = *uart_cfg;

  *out_receiver = receiver;
  return ESP_OK;
err:
  if (receiver) {
    memset(receiver->rx_buf, 0, sizeof(receiver->rx_buf));
    vTaskDelete(receiver->nmea_uart_task);
    receiver->rx_len = 0U;
  }
  return ret;
}
esp_err_t
gnss_receiver_del(gnss_receiver_handle receiver) {
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t
gnss_receiver_start(gnss_receiver_handle receiver) {
  ESP_RETURN_ON_FALSE(receiver, ESP_ERR_INVALID_ARG, TAG, "invalid argument in gnss_receiver_start");
  ESP_RETURN_ON_FALSE(receiver->nmea_uart_task, ESP_ERR_INVALID_STATE, TAG, "invalid state, invalid queue handle");

  vTaskResume(receiver->nmea_uart_task);

  return ESP_OK;
}
esp_err_t
gnss_receiver_stop(gnss_receiver_handle receiver) {
  ESP_RETURN_ON_FALSE(receiver, ESP_ERR_INVALID_ARG, TAG, "invalid argument in gnss_receiver_start");
  ESP_RETURN_ON_FALSE(receiver->nmea_uart_task, ESP_ERR_INVALID_STATE, TAG, "invalid state, invalid queue handle");

  vTaskSuspend(receiver->nmea_uart_task);

  return ESP_OK;
}

esp_err_t
gnss_receiver_write_sentence(gnss_receiver_handle receiver, const struct gnss_receiver_sentence sentence) {
  return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t
gnss_receiver_register_event_handler(gnss_receiver_handle receiver, gnss_receiver_event_handler handler) {
  ESP_RETURN_ON_FALSE(receiver && handler, ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  receiver->event_handler = handler;

  return ESP_OK;
}

esp_err_t
gnss_receiver_update_parse_sentences(gnss_receiver_handle receiver, enum minmea_sentence_id *parse_sentences,
                                   uint8_t parse_sentences_len) {

  ESP_RETURN_ON_FALSE(receiver && parse_sentences && parse_sentences_len && parse_sentences_len < MINMEA_SENTENCE_MAX,
                      ESP_ERR_INVALID_ARG, TAG, "invalid argument");

  receiver->parse_sentences_len = parse_sentences_len;
  for (uint8_t i = 0; i < parse_sentences_len; ++i) {
    ESP_RETURN_ON_FALSE(parse_sentences[i] > 0, ESP_ERR_INVALID_ARG, TAG, "invalid NMEA sentence requested");
    receiver->parse_sentences[i] = parse_sentences[i];
    ESP_LOGD(TAG, "Added %d sentence to parsed sentences", receiver->parse_sentences[i]);
  }

  return ESP_OK;
}

static void
nmea_read_uart(uart_port_t uart_port, char *buf, uint16_t buf_size, uint32_t timeout_ms, enum minmea_sentence_id *parse_sentences,
               uint8_t parse_sentences_len, gnss_receiver_event_handler event_handler) {
  int read_bytes = uart_read_bytes(uart_port, (uint8_t *)buf, buf_size - 1, pdMS_TO_TICKS(timeout_ms));

  if (read_bytes <= 0) {
    return;
  }
  buf[read_bytes] = '\0';

  char *search_ptr = buf;
  uint16_t bytes_left = read_bytes;

  while (bytes_left > 0) {
    char *start = memchr(search_ptr, '$', bytes_left);
    if (!start)
      break;

    char parse_buf[MINMEA_MAX_SENTENCE_LENGTH];
    char *lf_pos = memccpy(parse_buf, start, '\n', sizeof(parse_buf) - 1);

    if (!lf_pos) {
      break;
    }

    uint16_t sentence_len = lf_pos - parse_buf;
    parse_buf[sentence_len] = '\0';

    handle_nmea_sentence(parse_buf, sentence_len, parse_sentences, parse_sentences_len, event_handler);

    uint16_t processed = (start - search_ptr) + sentence_len;
    search_ptr += processed;
    bytes_left -= processed;
  }

  if (bytes_left > 0 && search_ptr != buf) {
    memmove(buf, search_ptr, bytes_left);
  }
}

static void
handle_nmea_sentence(const char *sentence, uint16_t len, enum minmea_sentence_id *parse_sentences, uint8_t parse_sentences_len,
                     gnss_receiver_event_handler event_handler) {
  ESP_RETURN_VOID_ON_FALSE(sentence && len, TAG, "invalid sentence in handle_nmea_sentence");
  ESP_RETURN_VOID_ON_FALSE(parse_sentences && parse_sentences_len, TAG, "invalid parse_sentences in handle_nmea_sentence");
  ESP_RETURN_VOID_ON_FALSE(event_handler, TAG, "invalid event_handler in handle_nmea_sentence");

  enum minmea_sentence_id sentence_id = minmea_sentence_id(sentence, true);
  if (!(sentence_id > 0)) {
    ESP_LOGE(TAG, "invalid sentence");
    return;
  }

  struct gnss_receiver_sentence sentence_event = {
      .sentence_id = sentence_id,
  };

  for (uint8_t i = 0; i < parse_sentences_len; ++i) {
    if (parse_sentences[i] != sentence_id)
      continue;
    // if (!(parse_sentences[i] > 0) || !(parse_sentences[i] < MINMEA_SENTENCE_MAX))
    //   continue;

    switch (parse_sentences[i]) {
    case MINMEA_SENTENCE_RMC: {
      struct minmea_sentence_rmc frame;
      if (!minmea_parse_rmc(&frame, sentence)) {
        goto invalid;
      }

      sentence_event.sentence.rmc = frame;

      if (event_handler)
        event_handler(sentence_event);
    } break;

    case MINMEA_SENTENCE_GBS: {
      struct minmea_sentence_gbs frame;
      if (!minmea_parse_gbs(&frame, sentence)) {
        goto invalid;
      }

      sentence_event.sentence.gbs = frame;

      if (event_handler)
        event_handler(sentence_event);
    } break;

    case MINMEA_SENTENCE_GGA: {
      struct minmea_sentence_gga frame;
      if (!minmea_parse_gga(&frame, sentence)) {
        goto invalid;
      }

      sentence_event.sentence.gga = frame;

      if (event_handler)
        event_handler(sentence_event);
    } break;

    case MINMEA_SENTENCE_GST: {
      struct minmea_sentence_gst frame;
      if (!minmea_parse_gst(&frame, sentence)) {
        goto invalid;
      }

      sentence_event.sentence.gst = frame;

      if (event_handler)
        event_handler(sentence_event);
    } break;

    case MINMEA_SENTENCE_GSV: {
      struct minmea_sentence_gsv frame;
      if (!minmea_parse_gsv(&frame, sentence)) {
        goto invalid;
      }

      sentence_event.sentence.gsv = frame;

      if (event_handler)
        event_handler(sentence_event);
    } break;

    case MINMEA_SENTENCE_GSA: {
      struct minmea_sentence_gsa frame;
      if (!minmea_parse_gsa(&frame, sentence)) {
        goto invalid;
      }
      sentence_event.sentence.gsa = frame;

      if (event_handler)
        event_handler(sentence_event);
    } break;

    case MINMEA_SENTENCE_GLL: {
      struct minmea_sentence_gll frame;
      if (!minmea_parse_gll(&frame, sentence)) {
        goto invalid;
      }

      sentence_event.sentence.gll = frame;

      if (event_handler)
        event_handler(sentence_event);
    } break;

    case MINMEA_SENTENCE_VTG: {
      struct minmea_sentence_vtg frame;
      if (!minmea_parse_vtg(&frame, sentence)) {
        goto invalid;
      }

      sentence_event.sentence.vtg = frame;

      if (event_handler)
        event_handler(sentence_event);
    } break;

    case MINMEA_SENTENCE_ZDA: {
      struct minmea_sentence_zda frame;
      if (!minmea_parse_zda(&frame, sentence)) {
        goto invalid;
      }

      sentence_event.sentence.zda = frame;

      if (event_handler)
        event_handler(sentence_event);
    } break;

    case MINMEA_INVALID: {
    invalid:
      ESP_LOGE(TAG, "Invalid NMEA sentence");
    } break;

    default: {
      ESP_LOGE(TAG, "Unexpected NMEA sentence");
    } break;
    }
  }
}

static void
nmea_uart_task(void *arg) {
  vTaskSuspend(NULL);

  ESP_RETURN_VOID_ON_FALSE(arg, TAG, "invalid argument in nmea uart task");

  struct gnss_receiver_instance *receiver = (struct gnss_receiver_instance *)arg;
  ESP_RETURN_VOID_ON_FALSE(receiver, TAG, "invald gnss receiver instance");

  for (;;) {
    nmea_read_uart(receiver->uart_port, receiver->rx_buf, UART_RX_BUF_SIZE, 60U, receiver->parse_sentences, receiver->parse_sentences_len,
                   receiver->event_handler);
    vTaskDelay(pdMS_TO_TICKS(GNSS_RECEIVER_NMEA_TASK_PERIOD_MS));
  }
}
