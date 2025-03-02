#include "esphome/core/helpers.h"
#ifdef USE_ESP_IDF
#include "driver/timer.h"
#endif

#include "dali_bus.h"

// For all variants of the timer, we have a frequency of 312500Hz, meaning
// each tick takes 3.2us. 750 ticks is therefore 2400us, or the correct
// minimum stop bit time. (We could in theory use a lower frequency, but
// "divide by 256" is the best we can do on ESP8266 and that's this.)
#define STOP_BIT_TICKS 750

namespace esphome {
namespace dali_bus {

#if defined(ESP8266)

void DALIBusComponent::setup_timer() {
  static DALIInterrupt *arg = &this->store_;
  timer1_attachInterrupt([] { DALIInterrupt::timer_intr(arg); });
}

void DALIInterrupt::start_stop_bit_timer(void) {
  timer1_enable(TIM_DIV256, TIM_EDGE, TIM_SINGLE);
  timer1_write(STOP_BIT_TICKS);
}

void DALIInterrupt::stop_stop_bit_timer(void) { timer1_disable(); }

#elif defined(USE_ESP32_FRAMEWORK_ARDUINO)

void DALIBusComponent::setup_timer() {
  // When ESPhome starts using IDF 5.1 and we specify a frequency rather than a
  // divider, frequency 312500 is correct here
  this->store_.timer = timerBegin(0, 256, true);
  timerStop(this->store_.timer);
  static DALIInterrupt *arg = &this->store_;
  timerAttachInterrupt(
      this->store_.timer, [] { DALIInterrupt::timer_intr(arg); }, false);
}

void DALIInterrupt::start_stop_bit_timer(void) {
  timerAlarmWrite(this->timer, STOP_BIT_TICKS, false);
  timerRestart(this->timer);
  timerStart(this->timer);
}

void DALIInterrupt::stop_stop_bit_timer(void) { timerStop(this->timer); }

#elif defined(USE_ESP_IDF)

void DALIBusComponent::setup_timer() {
  timer_config_t config = {
      .alarm_en = TIMER_ALARM_DIS,
      .counter_en = TIMER_PAUSE,
      .intr_type = TIMER_INTR_LEVEL,
      .counter_dir = TIMER_COUNT_UP,
      .auto_reload = TIMER_AUTORELOAD_DIS,
      .divider = 256,
  };
  timer_init(TIMER_GROUP_0, TIMER_0, &config);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
  timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, DALIInterrupt::timer_intr_bool, &this->store_, 0);
}

void DALIInterrupt::start_stop_bit_timer(void) {
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, STOP_BIT_TICKS);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
  timer_start(TIMER_GROUP_0, TIMER_0);
}

void DALIInterrupt::stop_stop_bit_timer(void) { timer_pause(TIMER_GROUP_0, TIMER_0); }

bool IRAM_ATTR HOT DALIInterrupt::timer_intr_bool(void *d) {
  timer_intr((DALIInterrupt *) d);
  return false;
}

#else

#error "Not a supported platform"

#endif

}  // namespace dali_bus
}  // namespace esphome
