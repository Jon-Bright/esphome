#include "esphome/core/gpio.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include "dali_bus.h"

namespace esphome {
namespace dali_bus {

static const char *const TAG = "dali_bus";

void DALIBusComponent::setup() {
  this->out_pin_->pin_mode(gpio::FLAG_OUTPUT);
  this->out_pin_->setup();

  this->in_pin_->pin_mode(gpio::FLAG_INPUT);
  this->in_pin_->setup();
  this->store_.in_pin = this->in_pin_->to_isr();

  this->in_pin_->attach_interrupt(DALIInterrupt::gpio_intr, &this->store_, gpio::INTERRUPT_ANY_EDGE);
}

// DALI's "low" state is "the two bus wires are shorted together". This appears to us as the pin reading high.
// Conversely, DALI's "high" state is "no short" and this appears as the pin reading low.
// We receive an interrupt any time the level changes.
void IRAM_ATTR HOT DALIInterrupt::gpio_intr(DALIInterrupt *d) {
  if (d->in_pin.digital_read() == 0) {
    d->dali_high();
  } else {
    d->dali_low();
  }
}

void IRAM_ATTR HOT DALIInterrupt::timer_intr(DALIInterrupt *d) {
  // When the timer interval triggers, we've finished receiving bits - a stop bit has been seen
  d->dali_idle();
}

// The times below are ~63us more generous than the standard.  With electronics featuring a
// relatively slow zener diode, these times have proven reliable. If other people have circuits
// that respond differently, we could consider making this configurable.
#define DALI_HB_MIN 303  // half-bit
#define DALI_HB_MAX 530
#define DALI_2HB_MIN 636  // 2 half-bits
#define DALI_2HB_MAX 1030
#define DALI_HB_NOM 416  // Nominal

DALITime IRAM_ATTR DALIInterrupt::get_bit_time(void) {
  unsigned long diff;
  if (this->last_dali_high > this->last_dali_low) {
    diff = this->last_dali_high - this->last_dali_low;
  } else {
    diff = this->last_dali_low - this->last_dali_high;
  }
  if (diff < DALI_HB_MIN) {
    return tiTooShort;
  }
  if (diff < DALI_HB_MAX) {
    return tiHalfBit;
  }
  if (diff < DALI_2HB_MIN) {
    return tiInvalid;
  }
  if (diff < DALI_2HB_MAX) {
    return ti2HalfBits;
  }
  return tiTooLong;
}

void IRAM_ATTR HOT DALIInterrupt::dali_high() {}

void IRAM_ATTR HOT DALIInterrupt::dali_low() {}

void IRAM_ATTR HOT DALIInterrupt::dali_idle() {}

void DALIBusComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "dali_bus:");
  LOG_PIN("  DALI out: ", this->out_pin_);
  LOG_PIN("  DALI in: ", this->in_pin_);
  ESP_LOGCONFIG(TAG, "  Scan: ", YESNO(this->scan_));
}

}  // namespace dali_bus
}  // namespace esphome
