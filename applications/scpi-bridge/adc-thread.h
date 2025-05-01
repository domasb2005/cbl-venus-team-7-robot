#ifndef ADC_THREAD_H
#define ADC_THREAD_H
#include <glib.h>
#include <stdint.h>

#include "driver_ads1115.h"
/***
 * ADS
 */
typedef enum {
  ADS1115_GET_SAMPLE,
  ADS1115_SET_CHANNEL,
  ADS1115_GET_CHANNEL,
  ADS1115_SET_RATE,
  ADS1115_GET_RATE,
  ADS1115_SET_RANGE,
  ADS1115_GET_RANGE,
  ADS1115_SET_SAMPLING,
  ADS1115_ADD_SAMPLING_CHANNEL,
  ADS1115_REMOVE_SAMPLING_CHANNEL,
  ADS1115_CLEAR_SAMPLING_CHANNEL,
  ADS1115_QUIT
} ADS1115_Cmds;

typedef struct {
  int32_t channel;
  int32_t range;
} SampleChannelConfig;
typedef struct {
  GThread *thread;
  /** Commands */
  GAsyncQueue *input;
  /** Result samples. */
  GAsyncQueue *results;
  GAsyncQueue *samples;

  /** Wait samples */
  double wait_samples; 
  /** timeout */
  guint64 timeout;

  /** timer */
  GTimer *sample_timer;
  SampleChannelConfig *channels;
  int num_channels;
  int rate;

  ads1115_handle_t ads_handle;
} ADS1115_Thread;

typedef struct ads1115_MSG {
  ADS1115_Cmds cmd;
  int arg1;
  int arg2;
  int16_t raw;
  float volt;
} ads1115_MSG;

typedef struct ads1115_sample {
  double timestamp;
  // could be uint8_t but prefer alignment.
  uint32_t channel;
  float volt;
} ads1115_sample;

gpointer ads1115_thread(gpointer data);
void ads1115_handling_thread_init(ADS1115_Thread *t, int addr, uint32_t timeout);
void ads1115_handling_thread_destroy(ADS1115_Thread *t);
#endif // ADC_THREAD_H
