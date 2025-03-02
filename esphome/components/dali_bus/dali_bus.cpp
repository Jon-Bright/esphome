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

void IRAM_ATTR HOT DALIInterrupt::received_bit(bool bit) {
  this->rcvd_bits++;
  this->rcvd_val <<= 1;
  if (bit) {
    this->rcvd_val |= 1;
  }
}

void IRAM_ATTR HOT DALIInterrupt::dali_high() {
  this->last_dali_high = micros();
  if (this->state == stSending) {
    // We're sending - so hopefully, we're receiving what we're sending and can ignore it
    return;
  }
  DALITime bitTime = get_bit_time();
  if (this->state == stStartBitH1) {
    // We were in the first half of the start bit, we expect a half-bit timing.
    if (bitTime == tiHalfBit) {
      this->state = stStartBitH2;
    } else {
      // Incorrect bit timing
      ESP_LOGD(TAG, "Wrong start 1H bit time: %d", bitTime);
      this->state = stIdle;
    }
  } else if (this->state == stFirstHalf) {
    // We were in the first half of a normal bit, which implies that the previous state
    // was an edge change to DALI low at the start of a one bit. We should expect timing
    // for a half-bit.
    if (bitTime == tiHalfBit) {
      // Yep, this was the first half of a one. Now second half. Stop bit might follow.
      this->state = stSecondHalf;
      this->start_stop_bit_timer();
    } else {
      // Incorrect bit timing
      ESP_LOGD(TAG, "Wrong data 1H 1-bit time: %d", bitTime);
      this->state = stIdle;
    }
  } else if (this->state == stSecondHalf) {
    // We were in the second half of a normal bit. This implies the last edge was a change to
    // DALI low in the middle of a zero. This is _either_ the start of a zero after a half-bit
    // of delay, _or_ it's the midpoint of a one after two half-bits of delay.
    if (bitTime == tiHalfBit) {
      // OK, it was the second half of a zero. We're back in first half of a zero, or a stop bit.
      this->received_bit(false);
      this->state = stFirstHalf;
      this->start_stop_bit_timer();
    } else if (bitTime == ti2HalfBits) {
      // It was the second half of a zero and the first half of a one.  Remain in second half.
      // It might nevertheless be a stop bit.
      this->received_bit(false);
      this->start_stop_bit_timer();
    } else {
      // Incorrect bit timing
      ESP_LOGD(TAG, "Wrong data 2H zero bit time: %d", bitTime);
      this->state = stIdle;
    }
  }
}

void IRAM_ATTR HOT DALIInterrupt::dali_low() {
  this->last_dali_low = micros();
  if (this->state == stSending) {
    // We're sending - so hopefully, we're receiving what we're sending and can ignore it
    return;
  }
  // If a stop bit timer's still running: this isn't a stop bit, stop the timer.
  this->stop_stop_bit_timer();

  DALITime bitTime = get_bit_time();
  if (this->state == stIdle) {
    // We were idle, so this is the start of a start bit
    this->state = stStartBitH1;
    this->rcvd_bits = 0;
    this->rcvd_val = 0;
  } else if (this->state == stStartBitH2) {
    // We were in the second half of a start bit, so this is _either_ the start of a one after
    // a half-bit of delay, or the second half of a zero after two half-bits of delay.
    if (bitTime == tiHalfBit) {
      // It's a one, first half starts now
      this->state = stFirstHalf;
    } else if (bitTime == ti2HalfBits) {
      // It's a zero, second half starts now
      this->state = stSecondHalf;
    } else {
      // Incorrect bit timing
      ESP_LOGD(TAG, "Wrong start 2H bit time: %d", bitTime);
      this->state = stIdle;
    }
  } else if (this->state == stFirstHalf) {
    // We were in the first half of a normal bit, which implies that the previous state
    // was an edge change to DALI high at the start of a zero bit. We should expect timing
    // for a half-bit.
    if (bitTime == tiHalfBit) {
      // Yep, this was the first half of a zero. Now second half. (Stop bit can't follow without
      // an adge change back to high.)
      this->state = stSecondHalf;
    } else {
      // Incorrect bit timing
      ESP_LOGD(TAG, "Wrong data 1H 0-bit time: %d", bitTime);
      this->state = stIdle;
    }
  } else if (this->state == stSecondHalf) {
    // We were in the second half of a normal bit. This implies the last edge was a change to
    // DALI high in the middle of a one. This is _either_ the start of a one after a half-bit
    // of delay, _or_ it's the midpoint of a zero after two half-bits of delay.
    if (bitTime == tiHalfBit) {
      // OK, it was the second half of a one. We're back in first half of a one.
      this->received_bit(true);
      this->state = stFirstHalf;
    } else if (bitTime == ti2HalfBits) {
      // It was the second half of a one and the first half of a zero.  Remain in second half.
      this->received_bit(true);
    } else {
      // Incorrect bit timing
      ESP_LOGD(TAG, "Wrong data 2H one bit time: %d", bitTime);
      this->state = stIdle;
    }
  }
}

void IRAM_ATTR HOT DALIInterrupt::dali_idle() {
  if (this->state == stSecondHalf) {
    // We were in the second half of a normal bit. This implies the last edge was a change to
    // DALI high in the middle of a one. Add that last bit and we're ready.
    this->received_bit(true);
    this->state = stFrameReady;
    ESP_LOGD(TAG, "Frame ready, %d bits", this->rcvd_bits);
  } else if (this->state == stFirstHalf) {
    // We saw the line go high after a zero and assumed the first half of another zero, but
    // it turned out to be a stop bit.
    this->state = stFrameReady;
    ESP_LOGD(TAG, "Frame ready, %d bits", this->rcvd_bits);
  } else {
    // Incorrect bit timing
    ESP_LOGD(TAG, "Unexpected stop in state %d", this->state);
    this->state = stIdle;
  }
}

void DALIBusComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "dali_bus:");
  LOG_PIN("  DALI out: ", this->out_pin_);
  LOG_PIN("  DALI in: ", this->in_pin_);
  ESP_LOGCONFIG(TAG, "  Scan: ", YESNO(this->scan_));
}

}  // namespace dali_bus
}  // namespace esphome
