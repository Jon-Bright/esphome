#include "esphome/core/helpers.h"
#ifdef USE_ESP_IDF
#include "driver/timer.h"
#endif

#include "dali_bus.h"

namespace esphome {
namespace dali_bus {

// For all variants of the timer, we have a frequency of 312500Hz, meaning
// each tick takes 3.2us. 750 ticks is therefore 2400us, or the correct
// minimum stop bit time. (We could in theory use a lower frequency, but
// "divide by 256" is the best we can do on ESP8266 and that's this.)
static const uint32_t STOP_BIT_TICKS = 750;

// 3.2*130 = 416us, exactly the nominal half-bit time.
static const uint32_t HALF_BIT_TICKS = 130;

// For all of the variants below: we _could_ use the alarm functionality to
// trigger alarms at just the times we want them. ESP32's timer functions,
// however, don't allow us to touch timer functions while we're in a timer
// interrupt. This would prevent us from triggering a new half-bit timer in
// response to a new half-bit timer.  We therefore trigger a timer every
// 10*3.2us=32us, whether we need it or not, which counts down a counter. If
// the counter reaches zero, we do stuff. This works because 10 divides evenly
// into both numbers above. If those numbers are changed, this greatest
// common factor should also be changed.
static const uint32_t GCF_TICKS = 10;

#if defined(ESP8266)

DALIInterrupt *DALIInterrupt::instance;
void IRAM_ATTR HOT DALIInterrupt::timer_intr_void() { DALIInterrupt::timer_intr(DALIInterrupt::instance); }

void DALIBusComponent::setup_timer_() {
  InterruptLock lock;
  // Ideally, we'd use this lambda-based version to avoid having a static
  // variable. The problem with doing this is that we can't mark the lambda
  // as IRAM_ATTR, which ESP8266 really needs (as in, for me, as soon as WiFi
  // starts, it crashes if it's not IRAM_ATTR). As such, we use the version
  // with an instance variable and a static function, which we can mark
  // appropriately.
  // static DALIInterrupt *arg = &this->store_;
  // timer1_attachInterrupt([] { DALIInterrupt::timer_intr(arg); });
  DALIInterrupt::instance = &this->store_;
  timer1_attachInterrupt(DALIInterrupt::timer_intr_void);
  timer1_enable(TIM_DIV256, TIM_EDGE, TIM_LOOP);
  timer1_write(GCF_TICKS);
}

#elif defined(USE_ESP32_FRAMEWORK_ARDUINO)

void DALIBusComponent::setup_timer_() {
  InterruptLock lock;
  // When ESPhome starts using IDF 5.1 and we specify a frequency rather than a
  // divider, frequency 312500 is correct here
  this->store_.timer = timerBegin(0, 256, true);
  timerStop(this->store_.timer);
  static DALIInterrupt *arg = &this->store_;
  timerAttachInterrupt(
      this->store_.timer, [] { DALIInterrupt::timer_intr(arg); }, false);
  timerAlarmWrite(this->store_.timer, GCF_TICKS, true);
  timerAlarmEnable(this->store_.timer);
  timerStart(this->store_.timer);
}

#elif defined(USE_ESP_IDF)

void DALIBusComponent::setup_timer_() {
  InterruptLock lock;
  timer_config_t config = {
      .alarm_en = TIMER_ALARM_DIS,
      .counter_en = TIMER_PAUSE,
      .intr_type = TIMER_INTR_LEVEL,
      .counter_dir = TIMER_COUNT_UP,
      .auto_reload = TIMER_AUTORELOAD_EN,
      .divider = 256,
  };
  timer_init(TIMER_GROUP_0, TIMER_0, &config);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
  timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, DALIInterrupt::timer_intr_bool, &this->store_, 0);
  timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
  timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, GCF_TICKS);
  timer_set_alarm(TIMER_GROUP_0, TIMER_0, TIMER_ALARM_EN);
  timer_start(TIMER_GROUP_0, TIMER_0);
}

bool IRAM_ATTR HOT DALIInterrupt::timer_intr_bool(void *d) {
  timer_intr((DALIInterrupt *) d);
  return false;
}

#else

#error "Not a supported platform"

#endif

void DALIInterrupt::start_stop_bit_timer(void) { this->timer_cnt = STOP_BIT_TICKS / GCF_TICKS; }

void DALIInterrupt::start_half_bit_timer(void) { this->timer_cnt = HALF_BIT_TICKS / GCF_TICKS; }

void DALIInterrupt::stop_stop_bit_timer(void) { this->timer_cnt = 0; }

}  // namespace dali_bus
}  // namespace esphome
