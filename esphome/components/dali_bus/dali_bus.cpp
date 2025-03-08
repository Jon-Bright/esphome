#include "esphome/core/gpio.h"
#include "esphome/core/hal.h"
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

void DALIBusComponent::wait_then_send_(struct SendMsg m) {
  uint32_t wait_us = 12000 + 1000 * m.pri;
  uint32_t now = micros();
  this->send_state_ = smsAwaitSend1;
  if (this->send_state_ == smsDone && now - this->store_.last_dali_low >= wait_us) {
    // We're not sending and we've already waited long enough, send now
    this->sending_ = m;
    this->send_forward_message_(m.addr, m.msg);
  } else {
    // Queue up for loop to send
    this->msg_queue_.push_back(m);
  }
}

void DALIBusComponent::send_forward_message_(DALIAddr addr, DALIMsg msg) {
  // We don't check the state before setting stSending. Whatever was happening before,
  // we should have waited for our priority (a bunch of ms) and nothing happened in that time.
  // We're OK to just overwrite a previous state. (This will also allow us to recover a few
  // odd states.)
  uint32_t to_send = 0;
  uint32_t set_bit = 1;
  uint8_t check_bit = 0x80;
  while (check_bit != 0) {
    if ((addr & check_bit) == 0) {
      to_send |= set_bit;
      set_bit <<= 2;
    } else {
      set_bit <<= 1;
      to_send |= set_bit;
      set_bit <<= 1;
    }
    check_bit >>= 1;
  }
  check_bit = 0x80;
  while (check_bit != 0) {
    if ((msg & check_bit) == 0) {
      to_send |= set_bit;
      set_bit <<= 2;
    } else {
      set_bit <<= 1;
      to_send |= set_bit;
      set_bit <<= 1;
    }
    check_bit >>= 1;
  }
  this->store_.begin_send(to_send, 32);
}

void DALIBusComponent::process_sent_message_() {
  if (this->store_.send_state == ssFailed) {
    this->store_.send_state = ssNone;
    if (this->sending_.callback) {
      this->sending_.callback(false, 0);
    }
    return;
  }
  if (this->store_.send_state != ssSuccess)
    return;
  this->store_.send_state = ssNone;
  if (this->send_state_ == smsAwaitSend1) {
    if ((this->sending_.msg >= 32 && this->sending_.msg <= 129) || this->sending_.addr == ADDR_INITIALISE ||
        this->sending_.addr == ADDR_RANDOMISE) {
      // This needs sending a second time
      this->sending_.pri = priTxn;
      this->send_state_ = smsAwaitSend2;
      this->msg_queue_.push_front(this->sending_);
    } else if (this->sending_.msg >= msgQueryStatus && this->sending_.msg <= msgReadMemoryLoc) {
      // We need to wait for a reply
      this->send_state_ = smsAwaitBackFrame;
    } else {
      // No resend, no back frame, done
      this->send_state_ = smsDone;
      if (this->sending_.callback) {
        this->sending_.callback(true, 0);
      }
    }
  } else if (this->send_state_ == smsAwaitSend2) {
    // None of the repeated messages have a backframe.
    this->send_state_ = smsDone;
    if (this->sending_.callback) {
      this->sending_.callback(true, 0);
    }
  }
}

void DALIBusComponent::send_message_if_ready_() {
  if (this->msg_queue_.empty()) {
    return;
  }
  struct SendMsg front = this->msg_queue_.front();
  uint32_t now = micros();
  if (now - this->store_.last_dali_low >= front.wait_us) {
    // Message is ready to send
    this->msg_queue_.pop_front();
    this->sending_ = front;
    this->send_forward_message_(front.addr, front.msg);
    // We don't need to loop through other queued messages - the fact that we just started
    // sending one means by definition that any others can't be ready to send.
  }
}

void DALIBusComponent::loop() {
  this->store_.log_any_recv_errors();
  this->process_sent_message_();
  this->send_message_if_ready_();
}

void DALIBusComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "dali_bus:");
  LOG_PIN("  DALI out: ", this->out_pin_);
  LOG_PIN("  DALI in: ", this->in_pin_);
  ESP_LOGCONFIG(TAG, "  Scan: ", YESNO(this->scan_));
}

}  // namespace dali_bus
}  // namespace esphome
