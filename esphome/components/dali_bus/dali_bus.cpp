#include "esphome/core/gpio.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include "dali_bus.h"

namespace esphome {
namespace dali_bus {

static const char *const TAG = "dali_bus";

// These are all "special" addresses. They're outside the range of normal short addresses
// and are (largely) used for sending commands with data.  Essentially, for those commands,
// the address is the opcode and the opcode byte is used for data.
const DALIAddr ADDR_BROADCAST = (DALIAddr) 0xFF;

const DALIAddr ADDR_TERMINATE = (DALIAddr) 0xa1;
const DALIAddr ADDR_DTR0 = (DALIAddr) 0xa3;
const DALIAddr ADDR_INITIALISE = (DALIAddr) 0xa5;
const DALIAddr ADDR_RANDOMISE = (DALIAddr) 0xa7;
const DALIAddr ADDR_COMPARE = (DALIAddr) 0xa9;
const DALIAddr ADDR_WITHDRAW = (DALIAddr) 0xab;
const DALIAddr ADDR_PING = (DALIAddr) 0xad;

const DALIAddr ADDR_SEARCH_ADDR_H = (DALIAddr) 0xb1;
const DALIAddr ADDR_SEARCH_ADDR_M = (DALIAddr) 0xb3;
const DALIAddr ADDR_SEARCH_ADDR_L = (DALIAddr) 0xb5;
const DALIAddr ADDR_PROGRAM_SHORT_ADDR = (DALIAddr) 0xb7;
const DALIAddr ADDR_VERIFY_SHORT_ADDR = (DALIAddr) 0xb9;
const DALIAddr ADDR_QUERY_SHORT_ADDR = (DALIAddr) 0xbb;

const DALIAddr ADDR_ENABLE_DEVICE_TYPE = (DALIAddr) 0xc1;
const DALIAddr ADDR_DTR1 = (DALIAddr) 0xc3;
const DALIAddr ADDR_DTR2 = (DALIAddr) 0xc5;
const DALIAddr ADDR_WRITE_MEM_LOC = (DALIAddr) 0xc7;
const DALIAddr ADDR_WRITE_MEM_LOC_NO_REPLY = (DALIAddr) 0xc7;

void DALIBusComponent::setup() {
  this->out_pin_->pin_mode(gpio::FLAG_OUTPUT);
  this->out_pin_->setup();
  this->store_.out_pin = this->out_pin_->to_isr();
  this->out_pin_->digital_write(false);  // DALI high, i.e. not shorted/idle

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
  if (d->recv_state == rsSending) {
    // We're sending. The timer for the prior half-bit expired, we should send the next half-bit (if any).
    d->send_next_half_bit();
  } else {
    // When the timer interval triggers, we've finished receiving bits - a stop bit has been seen
    d->dali_idle();
  }
}

// The times below are ~63us more generous than the standard.  With electronics featuring a
// relatively slow zener diode, these times have proven reliable. If other people have circuits
// that respond differently, we could consider making this configurable. HB_NOM matches the standard.
static const uint32_t DALI_HB_MIN = 303;  // half-bit
static const uint32_t DALI_HB_MAX = 530;
static const uint32_t DALI_2HB_MIN = 636;  // 2 half-bits
static const uint32_t DALI_2HB_MAX = 1030;
static const uint32_t DALI_HB_NOM = 416;  // Nominal

DALITime IRAM_ATTR DALIInterrupt::get_bit_time(void) {
  uint32_t diff;
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
  this->rcvd_bits = this->rcvd_bits + 1;
  this->rcvd_val = this->rcvd_val << 1;
  if (bit) {
    this->rcvd_val = this->rcvd_val | 1;
  }
}

void IRAM_ATTR HOT DALIInterrupt::dali_high() {
  this->last_dali_high = micros();
  if (this->recv_state == rsSending) {
    // We're sending - so hopefully, we're receiving what we're sending and can ignore it
    return;
  }
  DALITime bitTime = get_bit_time();
  if (this->recv_state == rsStartBitH1) {
    // We were in the first half of the start bit, we expect a half-bit timing.
    if (bitTime == tiHalfBit) {
      this->recv_state = rsStartBitH2;
    } else {
      // Incorrect bit timing
      ESP_LOGD(TAG, "Wrong start 1H bit time: %d", bitTime);
      this->recv_state = rsIdle;
    }
  } else if (this->recv_state == rsFirstHalf) {
    // We were in the first half of a normal bit, which implies that the previous state
    // was an edge change to DALI low at the start of a one bit. We should expect timing
    // for a half-bit.
    if (bitTime == tiHalfBit) {
      // Yep, this was the first half of a one. Now second half. Stop bit might follow.
      this->recv_state = rsSecondHalf;
      this->start_stop_bit_timer();
    } else {
      // Incorrect bit timing
      ESP_LOGD(TAG, "Wrong data 1H 1-bit time: %d", bitTime);
      this->recv_state = rsIdle;
    }
  } else if (this->recv_state == rsSecondHalf) {
    // We were in the second half of a normal bit. This implies the last edge was a change to
    // DALI low in the middle of a zero. This is _either_ the start of a zero after a half-bit
    // of delay, _or_ it's the midpoint of a one after two half-bits of delay.
    if (bitTime == tiHalfBit) {
      // OK, it was the second half of a zero. We're back in first half of a zero, or a stop bit.
      this->received_bit(false);
      this->recv_state = rsFirstHalf;
      this->start_stop_bit_timer();
    } else if (bitTime == ti2HalfBits) {
      // It was the second half of a zero and the first half of a one.  Remain in second half.
      // It might nevertheless be a stop bit.
      this->received_bit(false);
      this->start_stop_bit_timer();
    } else {
      // Incorrect bit timing
      ESP_LOGD(TAG, "Wrong data 2H zero bit time: %d", bitTime);
      this->recv_state = rsIdle;
    }
  }
}

void IRAM_ATTR HOT DALIInterrupt::dali_low() {
  this->last_dali_low = micros();
  if (this->recv_state == rsSending) {
    // We're sending - so hopefully, we're receiving what we're sending and can ignore it
    return;
  }
  // If a stop bit timer's still running: this isn't a stop bit, stop the timer.
  this->stop_stop_bit_timer();

  DALITime bitTime = get_bit_time();
  if (this->recv_state == rsIdle) {
    // We were idle, so this is the start of a start bit
    this->recv_state = rsStartBitH1;
    this->rcvd_bits = 0;
    this->rcvd_val = 0;
  } else if (this->recv_state == rsStartBitH2) {
    // We were in the second half of a start bit, so this is _either_ the start of a one after
    // a half-bit of delay, or the second half of a zero after two half-bits of delay.
    if (bitTime == tiHalfBit) {
      // It's a one, first half starts now
      this->recv_state = rsFirstHalf;
    } else if (bitTime == ti2HalfBits) {
      // It's a zero, second half starts now
      this->recv_state = rsSecondHalf;
    } else {
      // Incorrect bit timing
      ESP_LOGD(TAG, "Wrong start 2H bit time: %d", bitTime);
      this->recv_state = rsIdle;
    }
  } else if (this->recv_state == rsFirstHalf) {
    // We were in the first half of a normal bit, which implies that the previous state
    // was an edge change to DALI high at the start of a zero bit. We should expect timing
    // for a half-bit.
    if (bitTime == tiHalfBit) {
      // Yep, this was the first half of a zero. Now second half. (Stop bit can't follow without
      // an adge change back to high.)
      this->recv_state = rsSecondHalf;
    } else {
      // Incorrect bit timing
      ESP_LOGD(TAG, "Wrong data 1H 0-bit time: %d", bitTime);
      this->recv_state = rsIdle;
    }
  } else if (this->recv_state == rsSecondHalf) {
    // We were in the second half of a normal bit. This implies the last edge was a change to
    // DALI high in the middle of a one. This is _either_ the start of a one after a half-bit
    // of delay, _or_ it's the midpoint of a zero after two half-bits of delay.
    if (bitTime == tiHalfBit) {
      // OK, it was the second half of a one. We're back in first half of a one.
      this->received_bit(true);
      this->recv_state = rsFirstHalf;
    } else if (bitTime == ti2HalfBits) {
      // It was the second half of a one and the first half of a zero.  Remain in second half.
      this->received_bit(true);
    } else {
      // Incorrect bit timing
      ESP_LOGD(TAG, "Wrong data 2H one bit time: %d", bitTime);
      this->recv_state = rsIdle;
    }
  }
}

void IRAM_ATTR HOT DALIInterrupt::dali_idle() {
  if (this->recv_state == rsSecondHalf) {
    // We were in the second half of a normal bit. This implies the last edge was a change to
    // DALI high in the middle of a one. Add that last bit and we're ready.
    this->received_bit(true);
    this->recv_state = rsFrameReady;
    ESP_LOGD(TAG, "Frame ready, %d bits", this->rcvd_bits);
  } else if (this->recv_state == rsFirstHalf) {
    // We saw the line go high after a zero and assumed the first half of another zero, but
    // it turned out to be a stop bit.
    this->recv_state = rsFrameReady;
    ESP_LOGD(TAG, "Frame ready, %d bits", this->rcvd_bits);
  } else {
    // Incorrect bit timing
    ESP_LOGD(TAG, "Unexpected stop in state %d", this->recv_state);
    this->recv_state = rsIdle;
  }
}

inline void IRAM_ATTR HOT DALIInterrupt::set_dali_high() { this->out_pin.digital_write(false); }

inline void IRAM_ATTR HOT DALIInterrupt::set_dali_low() { this->out_pin.digital_write(true); }

void IRAM_ATTR HOT DALIInterrupt::send_next_half_bit() {
  // First, check if we collided with another sender on the bus. We can't see collisions if we'd
  // shorted the bus (DALI low), but if we were just sending a DALI high half-bit, then the last low
  // time should be what it was when we started that half-bit. If it isn't, someone else has shorted
  // the bus. If they've done that, then (a) we should stop sending and (b) that's presumably the
  // low at the start of their start bit and we should set the state accordingly.
  if (this->low_time_at_start_of_high != 0 && this->last_dali_low != this->low_time_at_start_of_high) {
    // Yep, we've collided
    this->send_state = ssFailed;
    this->recv_state = rsStartBitH1;
    return;
  }

  // OK, no collision, time for the next half-bit
  if (this->send_state == ssStartBit) {
    // We've just sent the first half of our start bit. Switch to DALI high, move on to data bits.
    set_dali_high();
    this->low_time_at_start_of_high = micros();
    this->send_state = ssDataBits;
    this->start_half_bit_timer();
  } else if (this->send_state == ssDataBits) {
    // We should send the next half bit
    if (this->send_half_bits == 0) {
      // ...but there's nothing more to send! Send a stop bit.
      set_dali_high();
      this->low_time_at_start_of_high = micros();
      this->send_state = ssStopBit;
      this->start_stop_bit_timer();
    } else {
      // ...and there's more to send.
      if ((this->send_val & 1) == 1) {
        set_dali_high();
        this->low_time_at_start_of_high = micros();
      } else {
        set_dali_low();
        this->low_time_at_start_of_high = 0;
      }
      this->send_val = this->send_val >> 1;
      this->send_half_bits = this->send_half_bits - 1;
      this->start_half_bit_timer();
    }
  } else if (this->send_state == ssStopBit) {
    // We've successfully waited out the stop bit, our work here is done.
    this->send_state = ssSuccess;
    this->recv_state = rsIdle;
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
