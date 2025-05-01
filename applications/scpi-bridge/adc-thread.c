#include <adc-thread.h>
#include <libpynq.h>
#include <stdio.h>

extern int log_level;
// TODO move

ADS1115_Thread ads1115t = {
    NULL,
};
// Handle used by driver.
ads1115_handle_t ads_handle = {
    .inited = 0,
};
uint8_t ads_readreg(uint8_t addr, uint8_t reg, uint8_t *d, uint16_t len) {
  return iic_read_register(IIC0, addr, reg, d, len);
}
uint8_t ads_writereg(uint8_t addr, uint8_t reg, uint8_t *d, uint16_t len) {
  return iic_write_register(IIC0, addr, reg, d, len);
}
uint8_t ads_dummy(void) { return 0; }

void ads_myprint(const char *fmt, ...) {
  va_list arg_list;
  va_start(arg_list, fmt);
  vfprintf(stdout, fmt, arg_list);
  va_end(arg_list);
}

gpointer ads1115_thread(gpointer data G_GNUC_UNUSED) {

  int do_samples = 0;
  GTimer *intv_timer = g_timer_new();
  double now = g_timer_elapsed(intv_timer, NULL);
  double next = now + ads1115t.timeout / 1.0e6;
  uint64_t index = 0;
  while (1) {
    ads1115_MSG *msg = NULL;
    now = g_timer_elapsed(intv_timer, NULL);
    double diff = next - now;
    guint to = ads1115t.timeout;
    //    if (diff > 0) {
    //      to = diff * 1e6;
    //    }
    index++;
    msg = g_async_queue_timeout_pop(ads1115t.input, to);
    now = g_timer_elapsed(intv_timer, NULL);
    //    next = now + ads1115t.timeout / 1.0e6;
    if (msg) {
      switch (msg->cmd) {
      case ADS1115_GET_SAMPLE: {
        int16_t raw;
        float v;
        ads1115_continuous_read(&(ads1115t.ads_handle), &raw, &v);
        msg->raw = raw;
        msg->volt = v;
        g_async_queue_push(ads1115t.results, msg);
        msg = NULL;
        break;
      }
      case ADS1115_SET_CHANNEL: {
        int32_t channel = msg->arg1;
        ads1115_set_channel(&(ads1115t.ads_handle), channel);
        break;
      }
      case ADS1115_GET_CHANNEL: {
        ads1115_channel_t channel = 0;
        ads1115_get_channel(&(ads1115t.ads_handle), &channel);
        msg->arg1 = channel;
        g_async_queue_push(ads1115t.results, msg);
        msg = NULL;
        break;
      }
      case ADS1115_SET_RATE: {
        int32_t rate = msg->arg1;
        ads1115_set_rate(&(ads1115t.ads_handle), rate);
        switch (rate) {
        case ADS1115_RATE_8SPS:
          ads1115t.rate = 8;
          break;
        case ADS1115_RATE_16SPS:
          ads1115t.rate = 16;
          break;
        case ADS1115_RATE_32SPS:
          ads1115t.rate = 32;
          break;
        case ADS1115_RATE_64SPS:
          ads1115t.rate = 64;
          break;
        case ADS1115_RATE_250SPS:
          ads1115t.rate = 250;
          break;
        case ADS1115_RATE_475SPS:
          ads1115t.rate = 475;
          break;
        case ADS1115_RATE_860SPS:
          ads1115t.rate = 860;
          break;
        default:
          ads1115t.rate = 128;
          break;
        }
        break;
      }
      case ADS1115_GET_RATE: {
        ads1115_rate_t rate = 0;
        ads1115_get_rate(&(ads1115t.ads_handle), &rate);
        msg->arg1 = rate;
        g_async_queue_push(ads1115t.results, msg);
        msg = NULL;
        break;
      }
      case ADS1115_SET_RANGE: {
        int32_t range = msg->arg1;
        ads1115_set_range(&(ads1115t.ads_handle), range);
        break;
      }
      case ADS1115_GET_RANGE: {
        ads1115_range_t range = 0;
        ads1115_get_range(&(ads1115t.ads_handle), &range);
        msg->arg1 = range;
        g_async_queue_push(ads1115t.results, msg);
        msg = NULL;
        break;
      }
      case ADS1115_SET_SAMPLING: {
        do_samples = msg->arg1;
        ads1115t.timeout = msg->arg2;
        if (do_samples) {
          if (ads1115t.sample_timer == NULL) {
            ads1115t.sample_timer = g_timer_new();
          } else {
            g_timer_reset(ads1115t.sample_timer);
          }
        }
        break;
      }
      case ADS1115_ADD_SAMPLING_CHANNEL: {
        ads1115t.channels =
            g_realloc(ads1115t.channels, (ads1115t.num_channels + 2) *
                                             sizeof(SampleChannelConfig));
        ads1115t.channels[ads1115t.num_channels].channel = msg->arg1;
        ads1115t.channels[ads1115t.num_channels].range = msg->arg2;
        ads1115t.num_channels++;
        ads1115t.channels[ads1115t.num_channels].channel = -1;
        ads1115t.channels[ads1115t.num_channels].range = -1;
        break;
      }
      case ADS1115_CLEAR_SAMPLING_CHANNEL: {
        g_free(ads1115t.channels);
        ads1115t.channels = NULL;
        ads1115t.num_channels = 0;
        break;
      }
      case ADS1115_QUIT: {
        // Quit thread.
        g_thread_exit(EXIT_SUCCESS);
        return EXIT_SUCCESS;
      }
      default:
        break;
      }
      g_free(msg);
    } else {
      // handle sampling when enabled.
      if (do_samples) {
        // TODO implement a queue of commands to process.
        int16_t raw;
        float v;
        g_async_queue_lock(ads1115t.samples);
        // max buffer 10k entries.
        while (g_async_queue_length_unlocked(ads1115t.samples) > 10000) {
          gpointer data = g_async_queue_pop_unlocked(ads1115t.samples);
          g_slice_free(ads1115_sample, data);
        }

        uint32_t index = 0;
        uint32_t channel = -1;
        do {
          if (ads1115t.channels == NULL) {
            channel = -1;
          } else {
            channel = ads1115t.channels[index].channel;
            ads1115_set_channel(&(ads1115t.ads_handle), channel);
            ads1115_set_range(&(ads1115t.ads_handle),
                              ads1115t.channels[index].range);
            g_usleep(ads1115t.wait_samples / ads1115t.rate);
          }
          ads1115_continuous_read(&(ads1115t.ads_handle), &raw, &v);
          ads1115_sample *s = g_slice_alloc(sizeof(ads1115_sample));
          s->channel = channel;
          s->timestamp = g_timer_elapsed(ads1115t.sample_timer, NULL);
          s->volt = v;
          g_async_queue_push_unlocked(ads1115t.samples, s);
          index++;
        } while (ads1115t.channels != NULL &&
                 ads1115t.channels[index].channel >= 0);

        g_async_queue_unlock(ads1115t.samples);
      }
    }
  }
}

/**
 * Initialize the threat that handles the ads1115.
 */
void ads1115_handling_thread_init(ADS1115_Thread *t, int addr, uint32_t timeout) {
  ads1115t.ads_handle.iic_addr = addr;
  ads1115t.ads_handle.iic_init = ads_dummy;
  ads1115t.ads_handle.iic_deinit = ads_dummy;
  ads1115t.ads_handle.iic_read = ads_readreg;
  ads1115t.ads_handle.iic_write = ads_writereg;
  ads1115t.ads_handle.delay_ms = (void (*)(uint32_t))sleep_msec;
  ads1115t.ads_handle.debug_print = ads_myprint;
  ads1115_init(&(ads1115t.ads_handle));
  // Start reading with default settings.
  ads1115_start_continuous_read(&(ads1115t.ads_handle));
  if (log_level) {
    printf("DEBUG: %s: initialized ads1115 at address %02X\n", __FUNCTION__,
           addr);
  }
  t->input = g_async_queue_new_full(g_free);
  t->results = g_async_queue_new_full(g_free);
  t->samples = g_async_queue_new();
  t->timeout = 1000000;
  t->wait_samples = timeout*1e6;
  t->rate = 128;
  t->thread = g_thread_new("ads1115", ads1115_thread, NULL);
  if (log_level) {
    printf("DEBUG: %s: initialized sampling thread.\n", __FUNCTION__);
  }
}

void ads1115_handling_thread_destroy(ADS1115_Thread *t) {
  if (t->thread == NULL) {
    return;
  }
  if (log_level) {
    printf("Teardown ADS1115 sampling thread.\n");
  }
  ads1115_MSG *msg = g_malloc0(sizeof(*msg));
  msg->cmd = ADS1115_QUIT;
  g_async_queue_push(t->input, msg);
  printf("join thread\n");
  g_thread_join(t->thread);
  if (log_level) {
    printf("ADS1115 Thread gone\n");
  }
  g_async_queue_unref(t->input);
  g_async_queue_unref(t->results);
  ads1115_sample *s;
  g_async_queue_lock(t->samples);
  while ((s = g_async_queue_try_pop_unlocked(t->samples))) {
    g_slice_free(ads1115_sample, s);
  }
  g_async_queue_unlock(t->samples);
  g_async_queue_unref(t->samples);
  memset(t, 0, sizeof(ADS1115_Thread));
}
