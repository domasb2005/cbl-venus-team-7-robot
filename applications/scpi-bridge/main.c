#define _GNU_SOURCE
#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <fcntl.h> //open
#include <log.h>
#include <sched.h> // scheduler.
#include <scpi/scpi.h>
#include <signal.h>
#include <stdarg.h>
#include <stdlib.h>
#include <sys/mman.h> //mmap
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h> //close
#include <unistd.h>

// GLib support
#include <glib-unix.h>
#include <glib.h>

// must have empty line here to avoid indent reodering the #includes!
#include <libpynq.h>

#include <audio.h>

#include <netinet/tcp.h>
#define TCP_KA_IDLE 2
#define TCP_KA_INTV 2
#define TCP_KA_COUNT 2

// must have empty line here to avoid indent reodering the #includes!
#include "scpi_names.h"
#define SCPI_IDN1 "MANUFACTURE"
#define SCPI_IDN2 "PYNQRYB V1.1"
#define SCPI_IDN3 NULL
#define SCPI_IDN4 "03-23"

#define SCPI_INPUT_BUFFER_LENGTH 4096
#define SCPI_ERROR_QUEUE_SIZE 64

#define MAX(a, b) (((a) > (b)) ? (a) : (b))

#include "adc-thread.h"
#include "display-SCPI.h"

extern ADS1115_Thread ads1115t;

scpi_t scpi_context;
char scpi_input_buffer[SCPI_INPUT_BUFFER_LENGTH];
scpi_error_t scpi_error_queue_data[SCPI_ERROR_QUEUE_SIZE];

int no_stdin = 0;
int log_level = 0;

typedef struct IntervalAE {
  uint32_t length;
  uint32_t real_length;
  double *diffs;
  struct timespec last_tp;
} IntervalAE;

typedef struct CounterState {
  bool enabled;
  InterruptEdge edge;
  uint64_t count;
  struct timespec first_tp;
  struct timespec last_tp;
  bool log_intervals;
  IntervalAE logs;
} CounterState;

CounterState counter_states[64] = {
    {.enabled = false,
     .edge = RISING_EDGE,
     .count = 0,
     .first_tp = {0, 0},
     .last_tp = {0, 0},
     .log_intervals = false,
     .logs = {0, 0, NULL, {0, 0}}},
};

static scpi_result_t SCPI_IdnQ(scpi_t *context) {
  SCPI_ResultText(context, SCPI_IDN2);
  return SCPI_RES_OK;
}

/**
 * Switchbox
 */

/**
 * Query the state of the current mapping!
 *
 * :SWITCHBOX:MAP? {PIN}
 *
 */
static scpi_result_t SCPI_SwitchBox_PinQ(scpi_t *context) {
  printf("switchbox pin q\n");
  int32_t param = 0;
  if (!SCPI_ParamChoice(context, pin_name_scpi, &param, TRUE)) {
    return SCPI_RES_ERR;
  }
  int32_t target = switchbox_get_pin(param);

  const char *name;
  SCPI_ChoiceToName(pin_name_scpi, param, &name);
  const char *name2;
  SCPI_ChoiceToName(switchbox_name_scpi, target, &name2);
  if (log_level) {
    printf("DEBUG: %s: %s(%d) --> %s(%d)\n", __FUNCTION__, name, param, name2,
           target);
  }
  SCPI_ResultMnemonic(context, name2);
  return SCPI_RES_OK;
}

/**
 * Set the state of the current mapping!
 *
 * :SWITCHBOX:MAP {PIN} {OUTPUT}
 *
 */
static scpi_result_t SCPI_SwitchBox_Pin(scpi_t *context) {
  int32_t target = 0;
  int32_t pin = 0;
  if (!SCPI_ParamChoice(context, pin_name_scpi, &pin, TRUE)) {
    return SCPI_RES_ERR;
  }
  if (!SCPI_ParamChoice(context, switchbox_name_scpi, &target, TRUE)) {
    return SCPI_RES_ERR;
  }
  const char *pinn;
  SCPI_ChoiceToName(pin_name_scpi, pin, &pinn);
  const char *targetn;
  SCPI_ChoiceToName(switchbox_name_scpi, target, &targetn);
  switchbox_set_pin(pin, target);
  if (log_level) {
    printf("DEBUG: %s: %s(%d), %s(%d)\n", __FUNCTION__, pinn, pin, targetn,
           target);
  }
  return SCPI_RES_OK;
}

/**
 * Reset the switchbox to initial state. (everything GPIO)
 *
 * :SWITCHBOX:RST
 */
static scpi_result_t SCPI_SwitchBox_Rst(scpi_t *constext
                                        __attribute__((unused))) {
  for (int i = 0; i < IO_NUM_PINS ; i++) {
    if (i == LCD_MOSI || i == LCD_SCLK || i == LCD_CS || i == LCD_DC || i == LCD_RESET || i == LCD_BL) {
      continue;
    }
    switchbox_set_pin(i, 0);
  }
  if (log_level) {
    printf("DEBUG: %s\n", __FUNCTION__);
  }
  return SCPI_RES_OK;
}

/**
 * GPIO
 *
 * :GPIO:VALUE {PIN} {VALUE}
 *
 */
static scpi_result_t SCPI_GPIO_LEVEL(scpi_t *context) {
  int32_t pin = 0;
  int32_t value = 0;
  // Get the pin name.
  if (!SCPI_ParamChoice(context, pin_name_scpi, &pin, TRUE)) {
    return SCPI_RES_ERR;
  }
  // Get value.
  if (!SCPI_ParamChoice(context, gpio_level_name_scpi, &value, TRUE)) {
    return SCPI_RES_ERR;
  }
  gpio_set_level(pin, value);
  char const *pinn = NULL, *valuen = NULL;
  SCPI_ChoiceToName(pin_name_scpi, pin, &pinn);
  SCPI_ChoiceToName(gpio_level_name_scpi, value, &valuen);
  if (log_level) {
    printf("DEBUG: %s: %s(%d) %s(%d)\n", __FUNCTION__, pinn, pin, valuen,
           value);
  }

  return SCPI_RES_OK;
}
/**
 * :GPIO:VALUE? {PIN}
 */
static scpi_result_t SCPI_GPIO_LEVELQ(scpi_t *context) {
  int32_t pin = 0;
  if (!SCPI_ParamChoice(context, pin_name_scpi, &pin, TRUE)) {
    return SCPI_RES_ERR;
  }
  int level = GPIO_LEVEL_LOW;
  level = gpio_get_level(pin);
  char const *pinn = NULL, *valuen = NULL;
  SCPI_ChoiceToName(pin_name_scpi, pin, &pinn);
  SCPI_ChoiceToName(gpio_level_name_scpi, level, &valuen);
  if (log_level) {
    printf("DEBUG: %s: %s(%d) --> %s(%d)\n", __FUNCTION__, pinn, pin, valuen,
           level);
  }

  SCPI_ResultMnemonic(context, valuen);
  return SCPI_RES_OK;
}
/**
 * :GPIO:DIR {PIN} {DIRECTION}
 */
static scpi_result_t SCPI_GPIO_DIR(scpi_t *context) {
  int32_t pin = 0;
  int32_t dir = 0;
  if (!SCPI_ParamChoice(context, pin_name_scpi, &pin, TRUE)) {
    return SCPI_RES_ERR;
  }
  if (!SCPI_ParamChoice(context, gpio_dir_name_scpi, &dir, TRUE)) {
    return SCPI_RES_ERR;
  }
  const char *pinn;
  SCPI_ChoiceToName(pin_name_scpi, pin, &pinn);
  const char *direction_name;
  SCPI_ChoiceToName(gpio_dir_name_scpi, dir, &direction_name);
  if (log_level) {
    printf("DEBUG: %s: %s(%d) %s(%d)\n", __FUNCTION__, pinn, pin,
           direction_name, dir);
  }
  gpio_set_direction(pin, dir);
  return SCPI_RES_OK;
}
/**
 * :GPIO:DIR? {PIN}
 */
static scpi_result_t SCPI_GPIO_DIRQ(scpi_t *context) {
  int32_t pin = 0;
  if (!SCPI_ParamChoice(context, pin_name_scpi, &pin, TRUE)) {
    return SCPI_RES_ERR;
  }
  char const *pinn = NULL, *valuen = NULL;
  SCPI_ChoiceToName(pin_name_scpi, pin, &pinn);
  int32_t dir = GPIO_DIR_INPUT;
  dir = gpio_get_direction(pin);
  SCPI_ChoiceToName(gpio_dir_name_scpi, dir, &valuen);
  if (log_level) {
    printf("DEBUG: %s: %s(%d) --> %s(%d)\n", __FUNCTION__, pinn, pin, valuen,
           dir);
  }

  SCPI_ResultMnemonic(context, valuen);
  return SCPI_RES_OK;
}

/**
 * Reset the GPIO module. All pins as input, all output set to LOW
 *
 * :GPIO:RST
 */
scpi_result_t SCPI_GPIO_Rst(scpi_t *context __attribute__((unused))) {
  for (int i = 0; i < IO_NUM_PINS ; i++) {
    if (i == LCD_MOSI || i == LCD_SCLK || i == LCD_CS || i == LCD_DC || i == LCD_RESET || i == LCD_BL) {
      continue;
    }
    gpio_reset_pin(i);
  }
 if (log_level) {
    printf("DEBUG: %s\n", __FUNCTION__);
  }
  return SCPI_RES_OK;
}

/**
 * Enable counter on pin.
 *
 * :COUNTER:ENABLE {pin}
 */
scpi_result_t SCPI_COUNTER_Enable(scpi_t *context) {
  int32_t pin = 0;
  if (!SCPI_ParamChoice(context, pin_name_scpi, &pin, TRUE)) {
    return SCPI_RES_ERR;
  }
  gpio_enable_interrupt(pin);
  counter_states[pin].enabled = true;
  // Reset the state
  counter_states[pin].count = 0;
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &(now));
  counter_states[pin].first_tp = now;

  char const *pinn = NULL;
  SCPI_ChoiceToName(pin_name_scpi, pin, &pinn);
  if (log_level) {
    printf("DEBUG: %s: %s(%d) \n", __FUNCTION__, pinn, pin);
  }

  return SCPI_RES_OK;
}
/**
 * Disable counter on pin.
 *
 * :COUNTER:DISABLE {pin}
 */
scpi_result_t SCPI_COUNTER_Disable(scpi_t *context) {
  int32_t pin = 0;
  if (!SCPI_ParamChoice(context, pin_name_scpi, &pin, TRUE)) {
    return SCPI_RES_ERR;
  }
  gpio_disable_interrupt(pin);
  counter_states[pin].enabled = false;

  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &(now));
  counter_states[pin].last_tp = now;

  char const *pinn = NULL;
  SCPI_ChoiceToName(pin_name_scpi, pin, &pinn);
  if (log_level) {
    printf("DEBUG: %s: %s(%d) \n", __FUNCTION__, pinn, pin);
  }

  return SCPI_RES_OK;
}
/**
 * Disable counter on pin.
 *
 * :COUNTER:Edge {pin} {edge}
 */
scpi_result_t SCPI_COUNTER_Edge(scpi_t *context) {
  int32_t pin = 0;
  if (!SCPI_ParamChoice(context, pin_name_scpi, &pin, TRUE)) {
    return SCPI_RES_ERR;
  }
  int32_t edget = BOTH_EDGE;
  if (!SCPI_ParamChoice(context, counter_edge_scpi, &edget, TRUE)) {
    return SCPI_RES_ERR;
  }
  counter_states[pin].edge = edget;

  char const *pinn = NULL;
  char const *edgen = NULL;
  SCPI_ChoiceToName(pin_name_scpi, pin, &pinn);
  SCPI_ChoiceToName(counter_edge_scpi, edget, &edgen);
  if (log_level) {
    printf("DEBUG: %s: %s(%d) -> %s\n", __FUNCTION__, pinn, pin, edgen);
  }

  return SCPI_RES_OK;
}

/**
 * Enable interval log on pin.
 * This clears previous samples if called while enabled.
 *
 * :COUNTER:INTERVAL:Enable {pin}
 */
scpi_result_t SCPI_COUNTER_INTERVAL_Enable(scpi_t *context) {
  int32_t pin = 0;
  if (!SCPI_ParamChoice(context, pin_name_scpi, &pin, TRUE)) {
    return SCPI_RES_ERR;
  }
  counter_states[pin].log_intervals = true;
  // Reset last sample point.
  counter_states[pin].logs.last_tp = (struct timespec){0, 0};
  counter_states[pin].logs.length = 0;

  char const *pinn = NULL;
  SCPI_ChoiceToName(pin_name_scpi, pin, &pinn);
  if (log_level) {
    printf("DEBUG: %s: %s(%d) \n", __FUNCTION__, pinn, pin);
  }

  return SCPI_RES_OK;
}
/**
 * Disable interval log on pin.
 *
 * :COUNTER:INTERVAL:Disable {pin}
 */
scpi_result_t SCPI_COUNTER_INTERVAL_Disable(scpi_t *context) {
  int32_t pin = 0;
  if (!SCPI_ParamChoice(context, pin_name_scpi, &pin, TRUE)) {
    return SCPI_RES_ERR;
  }
  gpio_enable_interrupt(pin);
  counter_states[pin].log_intervals = false;

  char const *pinn = NULL;
  SCPI_ChoiceToName(pin_name_scpi, pin, &pinn);
  if (log_level) {
    printf("DEBUG: %s: %s(%d) \n", __FUNCTION__, pinn, pin);
  }

  return SCPI_RES_OK;
}
static inline void timespec_diff(struct timespec *a, struct timespec *b,
                                 struct timespec *result) {
  result->tv_sec = a->tv_sec - b->tv_sec;
  result->tv_nsec = a->tv_nsec - b->tv_nsec;
  if (result->tv_nsec < 0) {
    --result->tv_sec;
    result->tv_nsec += 1000000000L;
  }
}
/**
 * Reset counter on pin.
 *
 * :COUNTER:RESET {pin}
 */
scpi_result_t SCPI_COUNTER_Reset(scpi_t *context) {
  int32_t pin = 0;
  if (!SCPI_ParamChoice(context, pin_name_scpi, &pin, TRUE)) {
    return SCPI_RES_ERR;
  }

  counter_states[pin].last_tp = (struct timespec){0, 0};
  uint32_t valuen = counter_states[pin].count = 0;
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &(now));
  counter_states[pin].first_tp = now;

  char const *pinn = NULL;
  SCPI_ChoiceToName(pin_name_scpi, pin, &pinn);
  if (log_level) {
    printf("DEBUG: %s: %s(%d) --> %08X\n", __FUNCTION__, pinn, pin, valuen);
  }

  return SCPI_RES_OK;
}
/**
 * Query the counter time since enable.
 *
 * :COUNTER? {pin}
 *
 * Return count, duration
 *
 * Duration is since enable to disabled (or now if still running) in seconds
 */
scpi_result_t SCPI_COUNTER_Query(scpi_t *context) {
  int32_t pin = 0;
  if (!SCPI_ParamChoice(context, pin_name_scpi, &pin, TRUE)) {
    return SCPI_RES_ERR;
  }

  double time = 0;
  if (counter_states[pin].first_tp.tv_sec > 0) {
    struct timespec now;
    if (counter_states[pin].enabled) {
      clock_gettime(CLOCK_MONOTONIC, &(now));
    } else {
      now = counter_states[pin].last_tp;
    }
    struct timespec diff;
    timespec_diff(&now, &(counter_states[pin].first_tp), &diff);
    time = diff.tv_sec + diff.tv_nsec / 1e9;
  }
  uint32_t valuen = counter_states[pin].count;
  char const *pinn = NULL;
  SCPI_ChoiceToName(pin_name_scpi, pin, &pinn);
  if (log_level) {
    printf("DEBUG: %s: %s(%d) --> %f\n", __FUNCTION__, pinn, pin, time);
  }

  SCPI_ResultUInt32(context, (uint32_t)valuen);
  SCPI_ResultDouble(context, time);
  return SCPI_RES_OK;
}

/**
 * Display message on the PYNQ display
 * 
 * :Message? {String}
 * 
 */

scpi_result_t SCPI_DISPLAY_message(scpi_t *context) {
  char  buffer[100];
  size_t copy_len;
  if (!SCPI_ParamCopyText(context, buffer, sizeof (buffer), &copy_len, FALSE)) {
      buffer[0] = '\0';
  }
  //printf("TEXT: ***%s***\r\n", buffer);
  displayMessage((uint8_t *) buffer);
  return SCPI_RES_OK;
}


/**
 * Query the counter interval state.
 *
 * :COUNTER:INTERVAL? {pin}
 */
scpi_result_t SCPI_COUNTER_INTERVAL_Query(scpi_t *context) {
  int32_t pin = 0;
  if (!SCPI_ParamChoice(context, pin_name_scpi, &pin, TRUE)) {
    return SCPI_RES_ERR;
  }

  uint32_t valuen = counter_states[pin].logs.length;
  char const *pinn = NULL;
  SCPI_ChoiceToName(pin_name_scpi, pin, &pinn);
  if (log_level) {
    printf("DEBUG: %s: %s(%d) --> items %08X\n", __FUNCTION__, pinn, pin,
           valuen);
  }

  SCPI_ResultArrayDouble(context, counter_states[pin].logs.diffs,
                         counter_states[pin].logs.length, SCPI_FORMAT_ASCII);
  counter_states[pin].logs.length = 0;
  return SCPI_RES_OK;
}
/**
 * PWM#:PERIOD {period}
 */
scpi_result_t SCPI_PWM_period(scpi_t *context __attribute__((unused))) {
  int cmd[1] = {0};
  if (!SCPI_CommandNumbers(context, cmd, 1, 0)) {
    return SCPI_RES_ERR;
  }
  if (cmd[0] < 0 || cmd[0] >= NUM_PWMS) {
    return SCPI_RES_ERR;
  }
  uint32_t period = 0;
  if (!SCPI_ParamUInt32(context, &period, TRUE)) {
    return SCPI_RES_ERR;
  }
  pwm_set_period(cmd[0], period);
  if (log_level) {
    printf("DEBUG: %s: PWM%d --> %u\n", __FUNCTION__, cmd[0], period);
  }
  return SCPI_RES_OK;
}
/**
 * PWM#:DUTY {period}
 */
scpi_result_t SCPI_PWM_duty(scpi_t *context __attribute__((unused))) {
  int cmd[1] = {0};
  if (!SCPI_CommandNumbers(context, cmd, 1, 0)) {
    return SCPI_RES_ERR;
  }
  if (cmd[0] < 0 || cmd[0] >= NUM_PWMS) {
    return SCPI_RES_ERR;
  }
  uint32_t duty = 0;
  if (!SCPI_ParamUInt32(context, &duty, TRUE)) {
    return SCPI_RES_ERR;
  }
  pwm_set_duty_cycle(cmd[0], duty);
  if (log_level) {
    printf("DEBUG: %s: PWM%d --> %u\n", __FUNCTION__, cmd[0], duty);
  }
  return SCPI_RES_OK;
}
/**
 * PWM#:PERIOD?
 */
scpi_result_t SCPI_PWM_periodq(scpi_t *context) {
  int cmd[1] = {0};
  if (!SCPI_CommandNumbers(context, cmd, 1, 0)) {
    return SCPI_RES_ERR;
  }
  if (cmd[0] < 0 || cmd[0] >= NUM_PWMS) {
    return SCPI_RES_ERR;
  }
  uint32_t period = pwm_get_period(cmd[0]);
  SCPI_ResultUInt32(context, period);
  if (log_level) {
    printf("DEBUG: %s: PWM%d --> %u\n", __FUNCTION__, cmd[0], period);
  }
  return SCPI_RES_OK;
}

/**
 * PWM#:DUTY?
 */
scpi_result_t SCPI_PWM_dutyq(scpi_t *context) {
  int cmd[1] = {0};
  if (!SCPI_CommandNumbers(context, cmd, 1, 0)) {
    return SCPI_RES_ERR;
  }
  if (cmd[0] < 0 || cmd[0] >= NUM_PWMS) {
    return SCPI_RES_ERR;
  }
  uint32_t duty_cycle = pwm_get_duty_cycle(cmd[0]);
  SCPI_ResultUInt32(context, duty_cycle);
  if (log_level) {
    printf("DEBUG: %s: PWM%d --> %u\n", __FUNCTION__, cmd[0], duty_cycle);
  }
  return SCPI_RES_OK;
}
/**
 * PWM#:STEPS?
 */
scpi_result_t SCPI_PWM_stepsq(scpi_t *context) {
  int cmd[1] = {0};
  if (!SCPI_CommandNumbers(context, cmd, 1, 0)) {
    return SCPI_RES_ERR;
  }
  if (cmd[0] < 0 || cmd[0] >= NUM_PWMS) {
    return SCPI_RES_ERR;
  }
  uint32_t duty_cycle = pwm_get_steps(cmd[0]);
  SCPI_ResultUInt32(context, duty_cycle);
  if (log_level) {
    printf("DEBUG: %s: PWM%d --> %u\n", __FUNCTION__, cmd[0], duty_cycle);
  }
  return SCPI_RES_OK;
}
/**
 * PWM#:STEPS {CNT}
 */
scpi_result_t SCPI_PWM_steps(scpi_t *context) {
  int cmd[1] = {0};
  if (!SCPI_CommandNumbers(context, cmd, 1, 0)) {
    return SCPI_RES_ERR;
  }
  if (cmd[0] < 0 || cmd[0] >= NUM_PWMS) {
    return SCPI_RES_ERR;
  }
  uint32_t steps = 0;
  if (!SCPI_ParamUInt32(context, &steps, TRUE)) {
    return SCPI_RES_ERR;
  }
  pwm_set_steps(cmd[0], steps);
  if (log_level) {
    printf("DEBUG: %s: PWM%d --> %u\n", __FUNCTION__, cmd[0], steps);
  }
  return SCPI_RES_OK;
}

/**
 * ADC:READ? {pin}
 */
scpi_result_t SCPI_ADC_readq(scpi_t *context) {
  int32_t pin = 0;
  if (!SCPI_ParamChoice(context, adc_channel_name_scpi, &pin, TRUE)) {
    return SCPI_RES_ERR;
  }

  double val = adc_read_channel(pin);
  SCPI_ResultDouble(context, val);

  return SCPI_RES_OK;
}
/**
 * ADC:RAW? {pin}
 */
scpi_result_t SCPI_ADC_rawq(scpi_t *context) {
  int32_t pin = 0;
  if (!SCPI_ParamChoice(context, adc_channel_name_scpi, &pin, TRUE)) {
    return SCPI_RES_ERR;
  }

  uint32_t val = adc_read_channel_raw(pin);
  SCPI_ResultUInt32(context, val);

  return SCPI_RES_OK;
}

/**
 * :UART#:SEND
 */
scpi_result_t SCPI_UART_send(scpi_t *context) {
  int cmd[1] = {0};
  if (!SCPI_CommandNumbers(context, cmd, 1, 0)) {
    return SCPI_RES_ERR;
  }
  if (cmd[0] < 0 || cmd[0] > 1) {
    return SCPI_RES_ERR;
  }
  const char *str = NULL;
  size_t len = 0;
  if (!SCPI_ParamCharacters(context, &str, &len, TRUE)) {
    return SCPI_RES_ERR;
  }
  if (log_level) {
    printf("DEBUG: %s: UART%d --> %.*s (%u)\n", __FUNCTION__, cmd[0], len, str,
           len);
  }
  for (size_t i = 0; i < len; i++) {
    if (!uart_has_space(cmd[0])) {
      fprintf(stderr, "ERROR: No space to send character: %u (%c)", i, str[i]);
      return SCPI_RES_ERR;
    }
    if (log_level) {
      printf("DEBUG: %s: UART%d --> %.*s(%u)\n", __FUNCTION__, cmd[0], len, str,
             i);
    }
    uart_send(cmd[0], (uint8_t)str[i]);
  }
  return SCPI_RES_OK;
}
/**
 * :UART#:RECV?
 */
scpi_result_t SCPI_UART_recv(scpi_t *context) {
  int cmd[1] = {0};
  if (!SCPI_CommandNumbers(context, cmd, 1, 0)) {
    return SCPI_RES_ERR;
  }
  if (cmd[0] < 0 || cmd[0] > 1) {
    return SCPI_RES_ERR;
  }
  if (!uart_has_data(cmd[0])) {
    return SCPI_RES_ERR;
  }
  uint8_t c = uart_recv(cmd[0]);
  char buf[4] = {
      0,
  };
  size_t len = snprintf(buf, 4, "%c", c);
  SCPI_ResultText(context, buf);

  if (log_level) {
    printf("DEBUG: %s: UART%d --> %02X |%c| (%u)\n", __FUNCTION__, cmd[0], c, c,
           len);
  }
  return SCPI_RES_OK;
}
/**
 * :UART#:SSpace?
 */
scpi_result_t SCPI_UART_send_space(scpi_t *context) {
  int cmd[1] = {0};
  if (!SCPI_CommandNumbers(context, cmd, 1, 0)) {
    return SCPI_RES_ERR;
  }
  if (cmd[0] < 0 || cmd[0] > 1) {
    return SCPI_RES_ERR;
  }
  bool val = uart_has_space(cmd[0]);
  SCPI_ResultMnemonic(context, val ? "TRUE" : "FALSE");
  if (log_level) {
    printf("DEBUG: %s: UART%d --> %s\n", __FUNCTION__, cmd[0],
           val ? "TRUE" : "FALSE");
  }
  return SCPI_RES_OK;
}
/**
 * :UART#:RData?
 */
scpi_result_t SCPI_UART_recv_data(scpi_t *context) {
  int cmd[1] = {0};
  if (!SCPI_CommandNumbers(context, cmd, 1, 0)) {
    return SCPI_RES_ERR;
  }
  if (cmd[0] < 0 || cmd[0] > 1) {
    return SCPI_RES_ERR;
  }
  bool val = uart_has_data(cmd[0]);
  SCPI_ResultMnemonic(context, val ? "TRUE" : "FALSE");
  if (log_level) {
    printf("DEBUG: %s: UART%d --> %s\n", __FUNCTION__, cmd[0],
           val ? "TRUE" : "FALSE");
  }
  return SCPI_RES_OK;
}
scpi_result_t SCPI_UART_reset(scpi_t *context) {
  int cmd[1] = {0};
  if (!SCPI_CommandNumbers(context, cmd, 1, 0)) {
    return SCPI_RES_ERR;
  }
  if (cmd[0] < 0 || cmd[0] > 1) {
    return SCPI_RES_ERR;
  }
  uart_reset_fifos(cmd[0]);
  if (log_level) {
    printf("DEBUG: %s: UART%d\n", __FUNCTION__, cmd[0]);
  }
  return SCPI_RES_OK;
}

/***********************************************************
 * ADS
 */

static void SCPI_ADS1115_destroy(void) {
  ads1115_handling_thread_destroy(&ads1115t);
}

scpi_result_t SCPI_ADS1115_init(scpi_t *context) {
  uint32_t cmd = 0;
  if (!SCPI_ParamUInt32(context, &cmd, TRUE)) {
    return SCPI_RES_ERR;
  }
  if (!(cmd == 0b1001000 || cmd == 0b1001001 || cmd == 0b1001010 ||
        cmd == 0b1001011)) {
    fprintf(stderr, "ERROR: ADS1115 invalid slave address: %02X\n", cmd);
    return SCPI_RES_ERR;
  }
  if (log_level) {
    printf("DEBUG: %s: initialize ads1115 at address %02X %02X\n", __FUNCTION__,
           cmd, 0b1001000);
  }
  uint32_t timeout = 4;
  if ( !SCPI_ParamUInt32(context, &timeout, FALSE) ){
	  timeout = 4;
  }
  if ( log_level){
	  printf("DEBUG: %s: using timeout %u\n", __FUNCTION__, timeout);
  }
  if (ads1115t.thread != NULL) {
    fprintf(stderr, "WARNING: ADS1115 is already initialized, rebuilding.\n");
    SCPI_ADS1115_destroy();
    // return SCPI_RES_ERR;
  }
  ads1115_handling_thread_init(&ads1115t, cmd, timeout);
  return SCPI_RES_OK;
}
scpi_result_t SCPI_ADS1115_set_channel(scpi_t *context) {
  int32_t channel = 0;
  if (!SCPI_ParamChoice(context, ads1115_channel_scpi, &channel, TRUE)) {
    fprintf(stderr, "ERROR: %s: Invalid channel specified.\n", __FUNCTION__);
    return SCPI_RES_ERR;
  }
  // Check of initialization.
  if (ads1115t.thread == NULL) {
    fprintf(stderr, "ERROR: ADS1115 is not initialized.\n");
    return SCPI_RES_ERR;
  }
  ads1115_MSG *msg = g_malloc0(sizeof(*msg));
  msg->cmd = ADS1115_SET_CHANNEL;
  msg->arg1 = channel;
  g_async_queue_push(ads1115t.input, msg);
  const char *channel_str;
  SCPI_ChoiceToName(ads1115_channel_scpi, channel, &channel_str);
  if (log_level) {
    printf("DEBUG: %s: Set ADS1115 channel: %s (%i)\n", __FUNCTION__,
           channel_str, channel);
  }

  return SCPI_RES_OK;
}
scpi_result_t SCPI_ADS1115_get_channel(scpi_t *context) {
  ads1115_channel_t channel = 0;
  // Check of initialization.
  if (ads1115t.thread == NULL) {
    fprintf(stderr, "ERROR: ADS1115 is not initialized.\n");
    return SCPI_RES_ERR;
  }
  const char *channel_str;
  ads1115_MSG *msg = g_malloc0(sizeof(*msg));
  msg->cmd = ADS1115_GET_CHANNEL;
  g_async_queue_push(ads1115t.input, msg);
  msg = NULL;
  msg = g_async_queue_pop(ads1115t.results);
  printf("%d %d\n", ADS1115_GET_CHANNEL, msg->cmd);
  g_assert(msg->cmd == ADS1115_GET_CHANNEL);
  channel = msg->arg1;
  SCPI_ChoiceToName(ads1115_channel_scpi, channel, &channel_str);
  SCPI_ResultMnemonic(context, channel_str);
  if (log_level) {
    printf("DEBUG: %s: Get ADS1115 channel: %s (%i)\n", __FUNCTION__,
           channel_str, channel);
  }
  return SCPI_RES_OK;
}

scpi_result_t SCPI_ADS1115_set_rate(scpi_t *context) {
  int32_t rate = 0;
  if (!SCPI_ParamChoice(context, ads1115_rate_scpi, &rate, TRUE)) {
    fprintf(stderr, "ERROR: %s: Invalid rate specified.\n", __FUNCTION__);
    return SCPI_RES_ERR;
  }
  // Check of initialization.
  if (ads1115t.thread == NULL) {
    fprintf(stderr, "ERROR: ADS1115 is not initialized.\n");
    return SCPI_RES_ERR;
  }
  ads1115_MSG *msg = g_malloc0(sizeof(*msg));
  msg->cmd = ADS1115_SET_RATE;
  msg->arg1 = rate;
  g_async_queue_push(ads1115t.input, msg);
  const char *rate_str;
  SCPI_ChoiceToName(pin_name_scpi, rate, &rate_str);
  if (log_level) {
    printf("DEBUG: %s: Set ADS1115 rate: %s (%i)\n", __FUNCTION__, rate_str,
           rate);
  }

  return SCPI_RES_OK;
}
scpi_result_t SCPI_ADS1115_get_rate(scpi_t *context) {
  ads1115_rate_t rate = 0;
  // Check of initialization.
  if (ads1115t.thread == NULL) {
    fprintf(stderr, "ERROR: ADS1115 is not initialized.\n");
    return SCPI_RES_ERR;
  }
  const char *rate_str;
  ads1115_MSG *msg = g_malloc0(sizeof(*msg));
  msg->cmd = ADS1115_GET_RATE;
  g_async_queue_push(ads1115t.input, msg);
  msg = NULL;
  msg = g_async_queue_pop(ads1115t.results);
  g_assert(msg->cmd == ADS1115_GET_RATE);
  rate = msg->arg1;
  SCPI_ChoiceToName(ads1115_rate_scpi, rate, &rate_str);
  SCPI_ResultMnemonic(context, rate_str);
  if (log_level) {
    printf("DEBUG: %s: Get ADS1115 rate: %s (%i)\n", __FUNCTION__, rate_str,
           rate);
  }
  return SCPI_RES_OK;
}

scpi_result_t SCPI_ADS1115_set_range(scpi_t *context) {
  int32_t range = 0;
  if (!SCPI_ParamChoice(context, ads1115_range_scpi, &range, TRUE)) {
    fprintf(stderr, "ERROR: %s: Invalid range specified.\n", __FUNCTION__);
    return SCPI_RES_ERR;
  }
  // Check of initialization.
  if (ads1115t.thread == NULL) {
    fprintf(stderr, "ERROR: ADS1115 is not initialized.\n");
    return SCPI_RES_ERR;
  }
  const char *range_str;
  SCPI_ChoiceToName(ads1115_range_scpi, range, &range_str);

  ads1115_MSG *msg = g_malloc0(sizeof(*msg));
  msg->cmd = ADS1115_SET_RANGE;
  msg->arg1 = range;
  g_async_queue_push(ads1115t.input, msg);
  if (log_level) {
    printf("DEBUG: %s: Set ADS1115 range: %s (%i)\n", __FUNCTION__, range_str,
           range);
  }

  return SCPI_RES_OK;
}
scpi_result_t SCPI_ADS1115_get_range(scpi_t *context) {
  ads1115_range_t range = 0;
  // Check of initialization.
  if (ads1115t.thread == NULL) {
    fprintf(stderr, "ERROR: ADS1115 is not initialized.\n");
    return SCPI_RES_ERR;
  }
  const char *range_str;
  ads1115_MSG *msg = g_malloc0(sizeof(*msg));
  msg->cmd = ADS1115_GET_RANGE;
  g_async_queue_push(ads1115t.input, msg);
  msg = NULL;
  msg = g_async_queue_pop(ads1115t.results);
  printf("%d %d\n", ADS1115_GET_RANGE, msg->cmd);
  g_assert(msg->cmd == ADS1115_GET_RANGE);
  range = msg->arg1;
  SCPI_ChoiceToName(ads1115_range_scpi, range, &range_str);
  SCPI_ResultMnemonic(context, range_str);
  if (log_level) {
    printf("DEBUG: %s: Get ADS1115 range: %s (%i)\n", __FUNCTION__, range_str,
           range);
  }
  return SCPI_RES_OK;
}

scpi_result_t SCPI_ADS1115_get_voltage(scpi_t *context) {
  // Check of initialization.
  if (ads1115t.thread == NULL) {
    fprintf(stderr, "ERROR: ADS1115 is not initialized.\n");
    return SCPI_RES_ERR;
  }
  float v;
  ads1115_MSG *msg = g_malloc0(sizeof(*msg));
  msg->cmd = ADS1115_GET_SAMPLE;
  g_async_queue_push(ads1115t.input, msg);
  msg = g_async_queue_pop(ads1115t.results);
  v = msg->volt;
  g_free(msg);
  SCPI_ResultDouble(context, v);
  if (log_level) {
    printf("DEBUG: %s: Get ADS1115 voltage: %.4f\n", __FUNCTION__, v);
  }
  return SCPI_RES_OK;
}
scpi_result_t SCPI_ADS1115_get_raw(scpi_t *context) {
  // Check of initialization.
  if (ads1115t.thread == NULL) {
    fprintf(stderr, "ERROR: ADS1115 is not initialized.\n");
    return SCPI_RES_ERR;
  }
  int16_t raw;
  ads1115_MSG *msg = g_malloc0(sizeof(*msg));
  msg->cmd = ADS1115_GET_SAMPLE;
  g_async_queue_push(ads1115t.input, msg);
  msg = g_async_queue_pop(ads1115t.results);
  raw = msg->raw;
  g_free(msg);
  SCPI_ResultInt(context, raw);
  if (log_level) {
    printf("DEBUG: %s: Get ADS1115 raw: %hd\n", __FUNCTION__, raw);
  }
  return SCPI_RES_OK;
}
scpi_result_t SCPI_ADS1115_run(scpi_t *context) {
  // Check of initialization.
  if (ads1115t.thread == NULL) {
    fprintf(stderr, "ERROR: ADS1115 is not initialized.\n");
    return SCPI_RES_ERR;
  }
  uint32_t timeout = 0;
  if (!SCPI_ParamUInt32(context, &timeout, TRUE)) {
    fprintf(stderr, "ERROR: %s: Invalid timeout specified.\n", __FUNCTION__);
    return SCPI_RES_ERR;
  }
  ads1115_MSG *msg = g_malloc0(sizeof(*msg));
  msg->cmd = ADS1115_SET_SAMPLING;
  msg->arg1 = 1;
  msg->arg2 = timeout;
  g_async_queue_push(ads1115t.input, msg);

  return SCPI_RES_OK;
}
scpi_result_t SCPI_ADS1115_stop(scpi_t *context) {
  // Check of initialization.
  if (ads1115t.thread == NULL) {
    fprintf(stderr, "ERROR: ADS1115 is not initialized.\n");
    return SCPI_RES_ERR;
  }
  ads1115_MSG *msg = g_malloc0(sizeof(*msg));
  msg->cmd = ADS1115_SET_SAMPLING;
  msg->arg1 = 0;
  msg->arg2 = 1000000;
  g_async_queue_push(ads1115t.input, msg);

  return SCPI_RES_OK;
}

scpi_result_t SCPI_ADS1115_samples(scpi_t *context) {
  // Check of initialization.
  if (ads1115t.thread == NULL) {
    fprintf(stderr, "ERROR: ADS1115 is not initialized.\n");
    return SCPI_RES_ERR;
  }
  ads1115_sample *data = NULL;
  // To avoid we reading as fast as we push, limit to what is currently in
  // buffer.
  gint count = g_async_queue_length(ads1115t.samples);
  while (count && (data = g_async_queue_try_pop(ads1115t.samples))) {
    SCPI_ResultDouble(context, data->timestamp);
    SCPI_ResultUInt32(context, data->channel);
    SCPI_ResultDouble(context, data->volt);
    g_slice_free(ads1115_sample, data);
    count--;
  }

  return SCPI_RES_OK;
}
scpi_result_t SCPI_ADS1115_add_channel(scpi_t *context) {
  int32_t channel = 0;
  int32_t range = 0;
  if (!SCPI_ParamChoice(context, ads1115_channel_scpi, &channel, TRUE)) {
    fprintf(stderr, "ERROR: %s: Invalid channel specified.\n", __FUNCTION__);
    return SCPI_RES_ERR;
  }
  if (!SCPI_ParamChoice(context, ads1115_range_scpi, &range, TRUE)) {
    fprintf(stderr, "ERROR: %s: Invalid range specified.\n", __FUNCTION__);
    return SCPI_RES_ERR;
  }
  // Check of initialization.
  if (ads1115t.thread == NULL) {
    fprintf(stderr, "ERROR: ADS1115 is not initialized.\n");
    return SCPI_RES_ERR;
  }
  ads1115_MSG *msg = g_malloc0(sizeof(*msg));
  msg->cmd = ADS1115_ADD_SAMPLING_CHANNEL;
  msg->arg1 = channel;
  msg->arg2 = range;
  g_async_queue_push(ads1115t.input, msg);
  const char *channel_str;
  SCPI_ChoiceToName(ads1115_channel_scpi, channel, &channel_str);
  if (log_level) {
    printf("DEBUG: %s: Add ADS1115 sampling channel: %s (%i)\n", __FUNCTION__,
           channel_str, channel);
  }

  return SCPI_RES_OK;
}
scpi_result_t SCPI_ADS1115_clear_channel(scpi_t *context) {
  // Check of initialization.
  if (ads1115t.thread == NULL) {
    fprintf(stderr, "ERROR: ADS1115 is not initialized.\n");
    return SCPI_RES_ERR;
  }
  ads1115_MSG *msg = g_malloc0(sizeof(*msg));
  msg->cmd = ADS1115_CLEAR_SAMPLING_CHANNEL;
  g_async_queue_push(ads1115t.input, msg);

  return SCPI_RES_OK;
}

scpi_result_t SCPI_FRESP_Ping(scpi_t *context) {
  uint32_t freq = 0;
  if (!SCPI_ParamUInt32(context, &freq, TRUE)) {
    return SCPI_RES_ERR;
  }
  uint32_t nperiods = 0;
  if (!SCPI_ParamUInt32(context, &nperiods, TRUE)) {
    return SCPI_RES_ERR;
  }
  // audio_record_response_start();
  uint32_t nsamples = 0;
  printf("Frequency: %u, periods: %u\n", freq, nperiods);
  int32_t *result = audio_record_response(freq, nperiods, 100, &nsamples);
  if (nsamples > 0) {
    SCPI_ResultArrayInt32(context, result, nsamples, SCPI_FORMAT_ASCII);
  }
  //  free(res);
  printf("done with result.\r\n");

  return SCPI_RES_OK;
}

/**
 * array to keep track of previous sample points.
 * Used to calculate roll-over.
 */
uint32_t last_count[NUM_PULSECOUNTERS - 1] = {
    0,
};

/** Pulse counter*/
scpi_result_t SCPI_PULSE_COUNTER_Query(scpi_t *context) {
  int cmd[1] = {0};
  if (!SCPI_CommandNumbers(context, cmd, 1, 0)) {
    return SCPI_RES_ERR;
  }
  if (cmd[0] >= NUM_PULSECOUNTERS) {
    return SCPI_RES_ERR;
  }
  uint32_t ts = 0;
  uint32_t count = pulsecounter_get_count(cmd[0], &ts);
  uint32_t diff = 0;

  if (ts < last_count[cmd[0]]) {
    diff = ts + (UINT32_MAX - last_count[cmd[0]]);
  } else {
    diff = ts - last_count[cmd[0]];
  }
  last_count[cmd[0]] = ts;

  SCPI_ResultUInt32(context, count);
  SCPI_ResultUInt32(context, diff);

  if (log_level) {
    printf("DEBUG: %s: query count of pulsecounter%u: %u, %u (%u)\n",
           __FUNCTION__, cmd[0], count, ts, diff);
  }
  return SCPI_RES_OK;
}
scpi_result_t SCPI_PULSE_COUNTER_Reset(scpi_t *context) {
  int cmd[1] = {0};
  if (!SCPI_CommandNumbers(context, cmd, 1, 0)) {
    return SCPI_RES_ERR;
  }
  if (cmd[0] >= NUM_PULSECOUNTERS) {
    return SCPI_RES_ERR;
  }
  pulsecounter_reset_count(cmd[0]);
  if (log_level) {
    printf("DEBUG: %s: reset count of pulsecounter%u\n", __FUNCTION__, cmd[0]);
  }
  return SCPI_RES_OK;
}

scpi_result_t SCPI_PULSE_COUNTER_Edge_Query(scpi_t *context) {
  int cmd[1] = {0};
  if (!SCPI_CommandNumbers(context, cmd, 1, 0)) {
    return SCPI_RES_ERR;
  }
  if (cmd[0] >= NUM_PULSECOUNTERS) {
    return SCPI_RES_ERR;
  }
  uint32_t edge = pulsecounter_get_edge(cmd[0]);
  const char *edge_str;
  SCPI_ChoiceToName(gpio_level_name_scpi, edge, &edge_str);

  SCPI_ResultMnemonic(context, edge_str);
  if (log_level) {
    printf("DEBUG: %s: query edge of pulsecounter%u -> %s\n", __FUNCTION__,
           cmd[0], edge_str);
  }
  return SCPI_RES_OK;
}
scpi_result_t SCPI_PULSE_COUNTER_Edge(scpi_t *context) {
  int cmd[1] = {0};
  if (!SCPI_CommandNumbers(context, cmd, 1, 0)) {
    return SCPI_RES_ERR;
  }
  if (cmd[0] >= NUM_PULSECOUNTERS) {
    return SCPI_RES_ERR;
  }
  int32_t edge = GPIO_LEVEL_HIGH;
  // Get value.
  if (!SCPI_ParamChoice(context, gpio_level_name_scpi, &edge, TRUE)) {
    return SCPI_RES_ERR;
  }

  pulsecounter_set_edge(cmd[0], edge);

  const char *edge_str;
  SCPI_ChoiceToName(gpio_level_name_scpi, edge, &edge_str);
  if (log_level) {
    printf("DEBUG: %s: set edge of pulsecounter%u -> %s\n", __FUNCTION__,
           cmd[0], edge_str);
  }
  return SCPI_RES_OK;
}

scpi_result_t SCPI_TIME_Set(scpi_t *context) {
    char buffer[17]; // Adjusted buffer size to accommodate date and time
    size_t copy_len;
    if (!SCPI_ParamCopyText(context, buffer, sizeof(buffer), &copy_len, FALSE)) {
        buffer[0] = '\0';
    }
    const char *new_date = buffer;
    
    // Check if the input string length is at least 12 characters
    if (strlen(new_date) < 14) {
        // Print error message or handle invalid input appropriately
        return SCPI_RES_ERR;
    }

    // Extract year, month, day, hour, and minute from the input string
    char year[5], month[3], day[3], hour[3], minute[3], second[3];
    strncpy(year, new_date, 4);
    year[4] = '\0';
    strncpy(month, new_date + 4, 2);
    month[2] = '\0';
    strncpy(day, new_date + 6, 2);
    day[2] = '\0';
    strncpy(hour, new_date + 8, 2);
    hour[2] = '\0';
    strncpy(minute, new_date + 10, 2);
    minute[2] = '\0';
    strncpy(second, new_date + 12, 2);
    second[2] = '\0';
    
    // Construct the date and time string in the format expected by the date command
    char command[100];
    snprintf(command, sizeof(command), "date %s%s%s%s%s.%s", month, day, hour, minute, year, second);
    
    // Set the system date and time using the constructed command
    system(command);
    
    return SCPI_RES_OK;
}

scpi_result_t SCPI_shutdown(scpi_t *context) {
    // Construct the command string for system shutdown
    displayDestroy();
    const char *command = "shutdown -h now";
    // Shutdown the system using the constructed command
    system(command);
    
    return SCPI_RES_OK;
}



/**
 * SCPI Commands list
 */
scpi_command_t scpi_commands[] = {
    {.pattern = "*IDN?", .callback = SCPI_IdnQ},
    {.pattern = "*RST", .callback = SCPI_CoreRst},
    {.pattern = ":SWITCHBOX:RST", .callback = SCPI_SwitchBox_Rst},
    {.pattern = ":SWITCHBOX:MAP?", .callback = SCPI_SwitchBox_PinQ},
    {.pattern = ":SWITCHBOX:MAP", .callback = SCPI_SwitchBox_Pin},
    {.pattern = ":GPIO:RST", .callback = SCPI_GPIO_Rst},
    {.pattern = ":GPIO:LEVEL", .callback = SCPI_GPIO_LEVEL},
    {.pattern = ":GPIO:LEVEL?", .callback = SCPI_GPIO_LEVELQ},
    {.pattern = ":GPIO:DIR", .callback = SCPI_GPIO_DIR},
    {.pattern = ":GPIO:DIR?", .callback = SCPI_GPIO_DIRQ},
    {.pattern = ":PWM#:PERIOD", .callback = SCPI_PWM_period},
    {.pattern = ":PWM#:DUTY", .callback = SCPI_PWM_duty},
    {.pattern = ":PWM#:PERIOD?", .callback = SCPI_PWM_periodq},
    {.pattern = ":PWM#:DUTY?", .callback = SCPI_PWM_dutyq},
    {.pattern = ":PWM#:STEPS", .callback = SCPI_PWM_steps},
    {.pattern = ":PWM#:STEPS?", .callback = SCPI_PWM_stepsq},
    {.pattern = ":ADC:READ?", .callback = SCPI_ADC_readq},
    {.pattern = ":ADC:RAW?", .callback = SCPI_ADC_rawq},
    {.pattern = ":UART#:SEND", .callback = SCPI_UART_send},
    {.pattern = ":UART#:RECV?", .callback = SCPI_UART_recv},
    {.pattern = ":UART#:SSpace?", .callback = SCPI_UART_send_space},
    {.pattern = ":UART#:RData?", .callback = SCPI_UART_recv_data},
    {.pattern = ":UART#:RST", .callback = SCPI_UART_reset},
    {.pattern = ":COUNTER:Enable", .callback = SCPI_COUNTER_Enable},
    {.pattern = ":COUNTER:Disable", .callback = SCPI_COUNTER_Disable},
    {.pattern = ":COUNTER:Edge", .callback = SCPI_COUNTER_Edge},
    {.pattern = ":COUNTER:RST", .callback = SCPI_COUNTER_Reset},
    {.pattern = ":COUNTER:INTERVAL:Enable",
     .callback = SCPI_COUNTER_INTERVAL_Enable},
    {.pattern = ":COUNTER:INTERVAL:Disable",
     .callback = SCPI_COUNTER_INTERVAL_Disable},
    {.pattern = ":COUNTER:INTERVAL?", .callback = SCPI_COUNTER_INTERVAL_Query},
    {.pattern = ":COUNTER?", .callback = SCPI_COUNTER_Query},
    // Display
    {.pattern = "DISPLAY:Message", .callback = SCPI_DISPLAY_message},
    // Pulse counter
    {.pattern = ":PULSECOUNTER#:Reset", .callback = SCPI_PULSE_COUNTER_Reset},
    {.pattern = ":PULSECOUNTER#:Count?", .callback = SCPI_PULSE_COUNTER_Query},
    {.pattern = ":PULSECOUNTER#:Edge?",
     .callback = SCPI_PULSE_COUNTER_Edge_Query},
    {.pattern = ":PULSECOUNTER#:Edge", .callback = SCPI_PULSE_COUNTER_Edge},
    // ADS1115
    {.pattern = ":ADS1115:INIT", .callback = SCPI_ADS1115_init},
    {.pattern = ":ADS1115:CHannel", .callback = SCPI_ADS1115_set_channel},
    {.pattern = ":ADS1115:CHannel?", .callback = SCPI_ADS1115_get_channel},
    {.pattern = ":ADS1115:RATE", .callback = SCPI_ADS1115_set_rate},
    {.pattern = ":ADS1115:RATE?", .callback = SCPI_ADS1115_get_rate},
    {.pattern = ":ADS1115:RANGE", .callback = SCPI_ADS1115_set_range},
    {.pattern = ":ADS1115:RANGE?", .callback = SCPI_ADS1115_get_range},
    {.pattern = ":ADS1115:Voltread?", .callback = SCPI_ADS1115_get_voltage},
    {.pattern = ":ADS1115:Rawread?", .callback = SCPI_ADS1115_get_raw},
    {.pattern = ":ADS1115:RUn", .callback = SCPI_ADS1115_run},
    {.pattern = ":ADS1115:STop", .callback = SCPI_ADS1115_stop},
    {.pattern = ":ADS1115:Samples?", .callback = SCPI_ADS1115_samples},
    {.pattern = ":ADS1115:ADDChannel", .callback = SCPI_ADS1115_add_channel},
    {.pattern = ":ADS1115:CLEARChannel",
     .callback = SCPI_ADS1115_clear_channel},
    // Testing response.
    {.pattern = ":FRESPONSE:Ping?", .callback = SCPI_FRESP_Ping},
    {.pattern = ":SYSTEM:Timeset", .callback = SCPI_TIME_Set},
    {.pattern = ":SYSTEM:Shutdown", .callback = SCPI_shutdown},
    SCPI_CMD_LIST_END};

int client_sock = -1;
/**
 * To be replaced.
 */
size_t myWrite(scpi_t *context, const char *data, size_t len) {
  (void)context;
  if (client_sock != -1) {
    size_t b = 0;
    while (b < len) {
      ssize_t a = send(client_sock, &data[b], len - b, MSG_NOSIGNAL);
      if (a < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          // it would block so we wait.
          fprintf(stderr, "Writing failed with warning: %s, retrying\n",
                  strerror(errno));
          usleep(10);
        } else {
          fprintf(stderr, "Writing failed with error: %s\n", strerror(errno));
          return -1;
        }

      } else if (a == 0) {
        fprintf(stderr, "Writing returned 0. confused");
      } else {
        b += a;
      }
    }
  }
  if (log_level > 1) {
    return fwrite(data, 1, len, stdout);
  }
  return len;
}
int SCPI_Error(scpi_t *context, int_fast16_t err) {
  (void)context;
  char const *data = SCPI_ErrorTranslate(err);
  size_t const len = strlen(data);
  if (client_sock != -1) {
    size_t b = 0;
    while (b < len) {
      ssize_t a = send(client_sock, &data[b], len - b, MSG_NOSIGNAL);
      if (a < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          // it would block so we wait.
          fprintf(stderr, "Writing failed with error: %s, retrying\n",
                  strerror(errno));
          usleep(10);
        } else {
          fprintf(stderr, "Writing failed with error: %s\n", strerror(errno));
          return -1;
        }

      } else if (a == 0) {
        fprintf(stderr, "Writing returned 0. confused");
      } else {
        b += a;
      }
    }
    send(client_sock, "\n", 1, MSG_NOSIGNAL);
  }
  printf("**ERROR: %d, \"%s\"\n", err, SCPI_ErrorTranslate(err));
  return 0;
}
scpi_result_t myReset(scpi_t *context) {
  fprintf(stderr, "**RESET\n");
  return SCPI_RES_OK;
}

static void reset_connection(void) {
  if (log_level) {
    printf("-Reset of connection requested.\n");
    printf("-Clearing remaining buffer.\n");
  }
  SCPI_Input(&scpi_context, NULL, 0);
  ads1115_handling_thread_destroy(&ads1115t);
}

scpi_interface_t scpi_interface = {
    .write = myWrite,
    .error = SCPI_Error,
    .reset = myReset,
};

#define DROP_SIZE 256
static void log_add(IntervalAE *iae, struct timespec diff) {
  if (iae->length >= iae->real_length) {
    iae->real_length += 128;
    iae->diffs = realloc(iae->diffs, sizeof(double) * (iae->real_length + 2));
  }
  // if buffer full, we drop 128 from the start to avoid indeffinate grow.
  if (iae->length > (2048 + DROP_SIZE)) {
    // Should be ok, as it uses neon instructions.
    memmove(iae->diffs, &(iae->diffs[DROP_SIZE]), iae->length);
    iae->length -= DROP_SIZE;
  }
  double d = diff.tv_sec + diff.tv_nsec / 1e9;
  iae->diffs[iae->length] = d;

  iae->length++;
}

static gboolean update_display_status() {
  if (client_sock >= 0) {
    displaySCPIStatus(CONNECTED);
    printf(".\n");
    ssize_t a = send(client_sock, NULL, 0, MSG_NOSIGNAL);
    if (a < 0) {
      close(client_sock);
      client_sock = -1;
      displaySCPIStatus(DISCONNECTED);
    }
  } else {
      displaySCPIStatus(DISCONNECTED);
  }
    return G_SOURCE_CONTINUE;
}

static void process_interrupt_read(int fd) {
  uint32_t interrupt = 0;
  (void)read(fd, &interrupt, 4);
  uint64_t bits = gpio_get_interrupt();

  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &(now));
  for (uint_fast8_t i = 0; i < 64; i++) {
    if (bits & 1) {
      if (counter_states[i].enabled) {
        int val = gpio_get_level(i);
        if ((val == GPIO_LEVEL_LOW) &&
            (counter_states[i].edge == RISING_EDGE)) {
          break;
        } else if ((val == GPIO_LEVEL_HIGH) &&
                   (counter_states[i].edge == FALLING_EDGE)) {
          break;
        }
        if (counter_states[i].log_intervals) {
          if (counter_states[i].logs.last_tp.tv_sec > 0) {
            struct timespec diff;
            timespec_diff(&now, &(counter_states[i].logs.last_tp), &diff);
            log_add(&(counter_states[i].logs), diff);
          }
          counter_states[i].logs.last_tp = now;
        }
        counter_states[i].last_tp = now;
        counter_states[i].count++;
      }
    }
    bits >>= 1;
  }

  /**
   * Acknowledge
   */
  gpio_ack_interrupt();
  uint32_t m = 1;
  write(fd, &m, 4);
}

static gboolean interrupt_callback(gint fd, GIOCondition condition,
                                   gpointer user_data __attribute__((unused))) {

  if (condition == G_IO_IN) {
    process_interrupt_read(fd);
  }
  return G_SOURCE_CONTINUE;
}

static gboolean client_interrupt_callback(gint fd, GIOCondition condition,
                                          gpointer user_data
                                          __attribute__((unused))) {
  if (condition == G_IO_IN) {
    size_t tok_size = 256;
    char client_message[tok_size + 1];
    // Receive a message from client
    ssize_t read_size = recv(fd, client_message, tok_size, MSG_NOSIGNAL);
    if (read_size == 0) {
      printf("-Connection closed\n");
      close(client_sock);
      client_sock = -1;
      reset_connection();
      // remove this watch.
      return G_SOURCE_REMOVE;
    } else if (read_size < 0) {
      printf("-Connection errored: %s\n", strerror(errno));
      close(client_sock);
      client_sock = -1;
      reset_connection();
      // remove this watch.
      return G_SOURCE_REMOVE;
    }
    // Should not be needed, but does not hurt.
    client_message[read_size] = '\0';
    SCPI_Input(&scpi_context, client_message, read_size);
  } else if (condition == G_IO_ERR) {
    printf("** Got error state on socket.\n");
  }
  return G_SOURCE_CONTINUE;
}
static gboolean socket_interrupt_callback(gint socket_desc,
                                          GIOCondition condition,
                                          gpointer user_data
                                          __attribute__((unused))) {

  if (condition == G_IO_IN && client_sock < 0) {
    struct sockaddr_in6 client;

    printf("-Handling incoming connections.\n");
    socklen_t c = sizeof(struct sockaddr_in);
    printf("- Accepting socket\n");
    client_sock = accept4(socket_desc, (struct sockaddr *)&client,
                          (socklen_t *)&c, SOCK_NONBLOCK);
    if (client_sock < 0) {
      fprintf(stderr, "* accept failed: %s\n", strerror(errno));
      return G_SOURCE_CONTINUE;
    }

    if (client.sin6_family == AF_INET6 &&
        !IN6_IS_ADDR_V4MAPPED(&client.sin6_addr)) {
      printf("Client is v6\n");
    } else {
      printf("Client is v4\n");
    }
    g_unix_fd_add(client_sock, G_IO_IN | G_IO_ERR, client_interrupt_callback,
                  NULL);
  } else if (condition == G_IO_ERR) {
    printf("** Got error state on socket.\n");
  }
  return G_SOURCE_CONTINUE;
}

/**
 * Keyboard handling.
 */
gboolean keyboard_interrupt_callback(gint socket_desc __attribute__((unused)),
                                     GIOCondition condition,
                                     gpointer user_data) {
  GMainLoop *mloop = (GMainLoop *)user_data;
  if (condition == G_IO_IN) {
    printf("* User requested quit.\n");
    g_main_loop_quit(mloop);
    return G_SOURCE_REMOVE;
  }
  return G_SOURCE_CONTINUE;
}

static int listen_and_response_to_tcp_socket(GMainLoop *mloop,
                                             int port_number) {
  /**
   * Setup interrupt_fd for interrupts.
   */

  int interrupt_fd = gpio_interrupt_init();

  // Add watch for interrupt handling.
  g_unix_fd_add(interrupt_fd, G_IO_IN, interrupt_callback, NULL);

  /**
   * Set socket to use re-use. This way you can directly restart it again, and
   * do not have to wait for the kernel to release the port.
   */
  int socket_desc = socket(AF_INET6, SOCK_STREAM | SOCK_NONBLOCK, 0);
  if (socket_desc == -1) {
    fprintf(stderr, "Failed to create socket: %s\n", strerror(errno));
    return EXIT_FAILURE;
  }
  int reuse = 1;
  if (setsockopt(socket_desc, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse,
                 sizeof(reuse)) < 0) {
    perror("setsockopt(SO_REUSEADDR) failed");
  }

#ifdef SO_REUSEPORT
  if (setsockopt(socket_desc, SOL_SOCKET, SO_REUSEPORT, (const char *)&reuse,
                 sizeof(reuse)) < 0) {
    perror("setsockopt(SO_REUSEPORT) failed");
  }
#endif
#ifdef SO_NOSIGPIPE
  {
    int set = 1;
    printf("+INFO: Set so_nosigpipe\r\n");
    if (setsockopt(socket_desc, SOL_SOCKET, SO_NOSIGPIPE, (const char *)&set,
                   sizeof(set)) < 0) {
      perror("setsockopt(SO_NOSIGPIPE) failed");
    }
  }
#endif

  printf("+Socket created\n");
  {
    int optval;
    socklen_t optlen = sizeof(optval);
    /* Check the status for the keepalive option */
    if (getsockopt(socket_desc, SOL_SOCKET, SO_KEEPALIVE, &optval, &optlen) <
        0) {
      perror("getsockopt()");
    }
    printf("+INFO: SO_KEEPALIVE is %s\n", (optval ? "ON" : "OFF"));

    /* Set the option active */
    optval = 1;
    optlen = sizeof(optval);
    if (setsockopt(socket_desc, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) <
        0) {
      perror("setsockopt()");
    }
    printf("+INFO: SO_KEEPALIVE set on socket\n");

    /* Check the status again */
    if (getsockopt(socket_desc, SOL_SOCKET, SO_KEEPALIVE, &optval, &optlen) <
        0) {
      perror("getsockopt()");
    }
    printf("+INFO: SO_KEEPALIVE is %s\n", (optval ? "ON" : "OFF"));
#ifdef TCP_KEEPCNT
    optval = TCP_KA_COUNT;
    printf("+INFO: set tcp keep alive count to %d\n", optval);
    if (setsockopt(socket_desc, SOL_TCP, TCP_KEEPCNT, &optval, sizeof(optval)) <
        0) {

      perror("setsockopt(TCP_KEEPCNT)");
    }
#endif

#ifdef TCP_KEEPIDLE
    optval = TCP_KA_IDLE;
    printf("+INFO: set tcp idle cnt to %d\n", optval);
    if (setsockopt(socket_desc, SOL_TCP, TCP_KEEPIDLE, &optval,
                   sizeof(optval)) < 0) {
      perror("setsockopt(TCP_KEEPIDLE)");
    }
#endif

#ifdef TCP_KEEPINTVL
    optval = TCP_KA_INTV;
    printf("+INFO: set tcp keep alive interval cnt to %d\n", optval);
    if (setsockopt(socket_desc, SOL_TCP, TCP_KEEPINTVL, &optval,
                   sizeof(optval)) < 0) {
      perror("setsockopt(TCP_KEEPINTVL)");
    }
#endif
  }
  // Prepare the sockaddr_in structure
  struct sockaddr_in6 server;
  server.sin6_family = AF_INET6;
  server.sin6_addr = in6addr_any;
  server.sin6_port = htons(port_number);

  if (bind(socket_desc, (struct sockaddr *)&server, sizeof(server)) < 0) {
    fprintf(stderr, "Failed to bind socket: %s\n", strerror(errno));
    close(socket_desc);
    return EXIT_FAILURE;
  }
  printf("+bind on port: %d done\n", port_number);

  /**
   * Listen for client, accept 1 connection.
   */
  listen(socket_desc, 1);
  printf("+ Listen done\n");
  g_unix_fd_add(socket_desc, G_IO_IN | G_IO_ERR, socket_interrupt_callback,
                NULL);
  if (!no_stdin) {
    g_unix_fd_add(STDIN_FILENO, G_IO_IN, keyboard_interrupt_callback,
                  (void *)mloop);
  }
  // Run the main loop.
  g_main_loop_run(mloop);
  printf("-Closing socket\n");
  close(socket_desc);
  return EXIT_SUCCESS;
}
static void do_interactive(void) {
  char smbuffer[10];
  while (1) {
    if (NULL == fgets(smbuffer, 10, stdin)) {
      break;
    }
    SCPI_Input(&scpi_context, smbuffer, strlen(smbuffer));
  }
}

int main(int argc __attribute__((unused)),
         char **argv __attribute__((unused))) {
  int port_number = 11008;
  int interactive = 0;

  // Set line buffering on the output, should help with logging.
  setlinebuf(stdout);
  setlinebuf(stderr);

  struct sched_param param = {
      0,
  };
  param.sched_priority = sched_get_priority_max(SCHED_RR);
  if (sched_setscheduler(0, SCHED_RR, &param) != 0) {
    fprintf(stderr, "Failed to set realtime priority: %s\r\n", strerror(errno));
  }
  /**
   * Input argument parsing.
   */
  int c;
  opterr = 0;
  while ((c = getopt(argc, argv, "dxihp:")) != -1) {
    switch (c) {
    case 'p':
      port_number = strtoul(optarg, NULL, 10);
      break;
    case 'd':
      log_level++;
      ;
      break;
    case 'x':
      no_stdin = 1;
      break;
    case 'i':
      interactive = 1;
      break;
    default:
    case 'h':
      fprintf(stdout, "-i: Run in interactive mode instead of listening on "
                      "tcp/ip port.\n");
      fprintf(stdout, "-p {port}: Listen on port {port} for incoming "
                      "connection (default 11008).\n");
      return EXIT_SUCCESS;
    case '?':
      if (optopt == 'p') {
        fprintf(stderr, "Option -%c requires an argument.\n", optopt);
      } else if (isprint(optopt)) {
        fprintf(stderr, "Unknown option `-%c'.\n", optopt);
      } else {
        fprintf(stderr, "Unknown option character `\\x%x'.\n", optopt);
      }
      return EXIT_FAILURE;
    }
  }
  //  signal(SIGPIPE, SIG_IGN);
  /**
   * Initialize the hardware we support.
   */
  pynq_init();
  for (uint8_t i = 0; i < NUM_PWMS; i++) {
    pwm_init(i, 0x4000);
  }
  uart_init(UART0);
  uart_init(UART1);
  adc_init();
  iic_init(IIC0);
  pulsecounter_init(PULSECOUNTER0);
  pulsecounter_init(PULSECOUNTER1);
  audio_init();
  audio_select_input(LINE_IN);
  audio_record_response_start();
  initialiseDisplay();
  
  SCPI_Init(&scpi_context, scpi_commands, &scpi_interface, scpi_units_def,
            SCPI_IDN1, SCPI_IDN2, SCPI_IDN3, SCPI_IDN4, scpi_input_buffer,
            SCPI_INPUT_BUFFER_LENGTH, scpi_error_queue_data,
            SCPI_ERROR_QUEUE_SIZE);
  /* printf("%.*s %s\r\n",  3, "asdadasdasdasdas", "b"); */
  if (interactive) {
    printf("SCPI Interactive\n");
    do_interactive();
  } else {
    printf("SCPI TCP/IP bridge\n");
    GMainLoop *mloop = g_main_loop_new(NULL, FALSE);
    g_timeout_add_seconds(1, update_display_status, NULL); 
    listen_and_response_to_tcp_socket(mloop, port_number);
    g_main_loop_unref(mloop);
  }
  printf("+Closing\r\n");
  /**
   * De-construct the hardware library.
   */
  displayDestroy();
  pulsecounter_destroy(PULSECOUNTER1);
  pulsecounter_destroy(PULSECOUNTER0);
  iic_destroy(IIC0);
  for (uint8_t i = 0; i < NUM_PWMS; i++) {
    pwm_destroy(i);
  }
  SCPI_ADS1115_destroy();
  adc_destroy();
  uart_destroy(UART0);
  uart_destroy(UART1);
  pynq_destroy();
  for (uint_fast8_t i = 0; i < 64; i++) {
    if (counter_states[i].logs.diffs != NULL) {
      free(counter_states[i].logs.diffs);
    }
  }
  return EXIT_SUCCESS;
}
