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
const DALIAddr ADDR_INITIALISE = (DALIAddr) 0xa5;  // Send twice
const DALIAddr ADDR_RANDOMISE = (DALIAddr) 0xa7;   // Send twice
const DALIAddr ADDR_COMPARE = (DALIAddr) 0xa9;     // Back frame
const DALIAddr ADDR_WITHDRAW = (DALIAddr) 0xab;
const DALIAddr ADDR_PING = (DALIAddr) 0xad;

const DALIAddr ADDR_SEARCH_ADDR_H = (DALIAddr) 0xb1;
const DALIAddr ADDR_SEARCH_ADDR_M = (DALIAddr) 0xb3;
const DALIAddr ADDR_SEARCH_ADDR_L = (DALIAddr) 0xb5;
const DALIAddr ADDR_PROGRAM_SHORT_ADDR = (DALIAddr) 0xb7;
const DALIAddr ADDR_VERIFY_SHORT_ADDR = (DALIAddr) 0xb9;  // Back frame
const DALIAddr ADDR_QUERY_SHORT_ADDR = (DALIAddr) 0xbb;   // Back frame

const DALIAddr ADDR_ENABLE_DEVICE_TYPE = (DALIAddr) 0xc1;
const DALIAddr ADDR_DTR1 = (DALIAddr) 0xc3;
const DALIAddr ADDR_DTR2 = (DALIAddr) 0xc5;
const DALIAddr ADDR_WRITE_MEM_LOC = (DALIAddr) 0xc7;  // Back frame
const DALIAddr ADDR_WRITE_MEM_LOC_NO_REPLY = (DALIAddr) 0xc7;

void DALIBusComponent::setup() {
  this->out_pin_->pin_mode(gpio::FLAG_OUTPUT);
  this->out_pin_->setup();
  this->store_.out_pin = this->out_pin_->to_isr();
  this->store_.out_shorted_state = this->out_inverted_;
  this->out_pin_->digital_write(!this->store_.out_shorted_state);  // DALI high, i.e. not shorted/idle

  this->in_pin_->pin_mode(gpio::FLAG_INPUT);
  this->in_pin_->setup();
  this->store_.in_pin = this->in_pin_->to_isr();
  this->store_.in_shorted_state = this->in_inverted_;

  this->in_pin_->attach_interrupt(DALIInterrupt::gpio_intr, &this->store_, gpio::INTERRUPT_ANY_EDGE);
  this->setup_timer_();

  if (this->scan_) {
    ESP_LOGD(TAG, "Beginning scan with reset");
    this->addr_state_ = asReset;
    this->addr_cb_ = std::bind(&DALIBusComponent::addressing_cb_, this, std::placeholders::_1, std::placeholders::_2);
    // Wait 5s from now before sending the reset. (Since we're at startup, there should be no clock-wrapping.)
    this->reset_time_ = micros() + this->scan_delay_ * 1000 * 1000;
  } else {
    this->addr_state_ = asInactive;
    this->reset_time_ = 0;
  }
}

void DALIBusComponent::wait_then_send_(struct SendMsg m) {
  uint32_t now = micros();
  m.wait_us = 12000 + 1000 * m.pri;
  if (this->send_state_ == smsDone && now - this->store_.last_dali_low >= m.wait_us) {
    // We're not sending and we've already waited long enough, send now
    this->sending_ = m;
    this->send_state_ = smsAwaitSend1;
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
    ESP_LOGD(TAG, "Sending message failed. Was sending addr %02X, msg %02X, ltasoh %08X, ldl %08X", this->sending_.addr,
             this->sending_.msg, this->store_.low_time_at_start_of_high, this->store_.last_dali_low);
    this->store_.send_state = ssNone;
    if (this->sending_.callback) {
      this->sending_.callback(crSendFailed, 0);
    }
    return;
  }
  if (this->store_.send_state != ssSuccess)
    return;
  this->store_.send_state = ssNone;
  if (this->send_state_ == smsAwaitSend1) {
    if (this->sending_.send_twice) {
      // This needs sending a second time
      this->sending_.pri = priTxn;
      this->send_state_ = smsAwaitSend2;
      this->msg_queue_.push_front(this->sending_);
    } else if (this->sending_.expect_back_frame) {
      // We need to wait for a reply
      this->send_state_ = smsAwaitBackFrame;
      this->back_frame_wait_start_ = micros();
    } else {
      // No resend, no back frame, done
      this->send_state_ = smsDone;
      if (this->sending_.callback) {
        this->sending_.callback(crSuccess, 0);
      }
    }
  } else if (this->send_state_ == smsAwaitSend2) {
    // None of the repeated messages have a backframe.
    this->send_state_ = smsDone;
    if (this->sending_.callback) {
      this->sending_.callback(crSuccess, 0);
    }
  }
}

void DALIBusComponent::process_back_frames_() {
  if (this->send_state_ != smsAwaitBackFrame) {
    // Not awaiting a back frame, nothing to do
    return;
  }
  if (this->store_.recv_state == rsFrameReady) {
    // Yay, a frame!
    DALICallbackResult cr;
    if (this->store_.rcvd_bits == 8) {
      cr = crGoodBackFrame;
    } else {
      cr = crWrongLength;
    }
    if (this->sending_.callback) {
      this->sending_.callback(cr, this->store_.rcvd_val);
    }
    this->send_state_ = smsDone;
  } else if (this->store_.recv_state == rsError) {
    // We detected an error with the frame
    this->sending_.callback(crTimingError, 0);
    this->store_.recv_state = rsIdle;
    this->send_state_ = smsDone;
  } else {
    // No frame yet.  Check whether we timed out waiting.
    uint32_t now = micros();
    // 20ms == 10.5ms max settle time, plus 1 start bit + 8 data bits at 1ms/bit, rounded up
    const uint32_t back_frame_timeout = 20 * 1000;
    if (now - this->back_frame_wait_start_ >= back_frame_timeout) {
      // We're done waiting, error out
      if (this->sending_.callback) {
        this->sending_.callback(crNoBackFrame, 0);
      }
      this->send_state_ = smsDone;
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
    if (this->send_state_ == smsDone) {
      this->send_state_ = smsAwaitSend1;
    }
    this->send_forward_message_(front.addr, front.msg);
    // We don't need to loop through other queued messages - the fact that we just started
    // sending one means by definition that any others can't be ready to send.
  }
}

// This callback implements (most of) the state machine for assigning short addresses to lamps on
// the bus. The full procedure is:
//  1. Broadcast a reset. This clears any previously assigned short address and likely turns the
//     lamps on. Lamps are only required to react 300ms after this reset. We're supposed to wait
//     350ms before sending commands.
//  2. Wait 350ms.
//  3. Broadcast an "off" message, for two reasons: (a) this avoids potentially quite bright
//     lamps remaining on for the duration of addressing, which could be quite a while if there
//     are many devices on the bus. (b) it's an early visual sign that the controller is
//     successfully controlling the bus.
//  4. Broadcast the "initialise" command, which enables the commands which are to follow.
//  5. Broadcast the "randomise" command, which tells the lamps to pick random 24-bit "long"
//     addresses. This random address setting is to be completed by the lamps within 100ms.
//  6. Wait for those 100ms. (This is implemented in the main loop, as we're not waiting on a
//     callback.) Set the next short address to 0.
//  7. Set min=0, max=fffffe, being the minimum and maximum permitted long address.
//  8. Start finding devices. Take the midpoint of the current min/max range. Send the high byte
//     of that midpoint.
//  9. Send the middle byte of the midpoint.
// 10. Send the low byte of the midpoint.
// 11. Send a compare message. This asks any lamp with a long address <= the search address sent
//     in steps 8-10 to reply. This may lead to multiple replies (because multiple lamps are in
//     the queried address segment). All of the replying lamps should be sending 0xFF as their
//     reply, but slight timing differences between the lamps may mean that their otherwise
//     identical replies appear to us as mistimed frames.
// 12. Receive a backward frame, or no frame. Because of the issue mentioned above, we _accept_
//     mistimed frames here. The three cases we care about are:
//     a) "failed to send" (an error, stop), dealt with prior to the main switch(),
//     b) "no reply" (there's no lamp below mid), and
//     c) "any kind of reply, including a broken one" (there's at least one lamp below mid)
//     If we get a reply and min==max, then we've found a lamp's long address - jump to step 13.
//     If we get a reply with min!=max, then there's at least one lamp in the bottom half of the
//     searched space. Set max to the midpoint and go back to step 8, meaning we'll search the
//     bottom half. (It's not a problem if there's a lamp in the top half _too_, we'll get it
//     later, see step 16 below.)
//     If we _don't_ get a reply, set min to mid+1 (the compare command is inclusive of mid) and
//     again go back to step 8. (If this adjustment results in mid>max, stop - there is no lamp
//     to be found, stop.)
// 13. We found a lamp! Program it with the next short address.
// 14. Verify its short address. We don't accept broken back frames here - only one lamp should
//     be replying. Increment the next short address by one.
// 15. Withdraw the lamp we found from consideration. It won't reply to future compare messages
//     even if it would otherwise match.
// 16. Go back to step 7 above (resetting min and max and starting a new search to find another
//     device).
//
// Steps 8-12 represent a binary search for lamp long addresses. You can see a worked example of
// what the search looks like in addr_example.txt. Notably it always ends with an unsuccessful
// search (when all lamps have been identified).

void DALIBusComponent::addressing_cb_(DALICallbackResult cr, uint8_t reply) {
  if (cr == crSendFailed ||
      (this->addr_state_ != asCompare && this->addr_state_ != asVerifyShortAddr && cr != crSuccess)) {
    ESP_LOGW(TAG, "Failed Readdressing, state %u, min %u, max %u, short %u", this->addr_state_, this->addr_min_,
             this->addr_max_, this->addr_short_);
    this->terminate_addressing_(false);
    return;
  }
  SendMsg m;
  m.expect_back_frame = false;
  m.send_twice = false;
  m.callback = this->addr_cb_;

  switch (this->addr_state_) {
    case asInactive:
      // Should not happen
      ESP_LOGE(TAG, "addressing_cb_ called while addressing inactive");
      break;
    case asReset:
      // Step 2: We sent the reset. Start waiting for 350ms. We'll be called back from
      // process_addr_wait_ (as we're not waiting for a message callback).
      this->addr_wait_start_ = micros();
      this->addr_state_ = asResetWait;
      break;
    case asResetWait:
      // Step 3
      m.pri = priTxn;
      m.addr = ADDR_BROADCAST;
      m.msg = msgOff;
      m.send_twice = true;
      this->addr_state_ = asLampOff;
      this->wait_then_send_(m);
      break;
    case asLampOff:
      // Step 4
      m.pri = priTxn;
      m.addr = ADDR_INITIALISE;
      m.msg = (DALIMsg) 0;
      m.send_twice = true;
      this->addr_state_ = asInitialise;
      this->wait_then_send_(m);
      break;
    case asInitialise:
      // Step 5
      m.pri = priTxn;
      m.addr = ADDR_RANDOMISE;
      m.msg = (DALIMsg) 0;
      m.send_twice = true;
      this->addr_state_ = asRandomise;
      this->wait_then_send_(m);
      break;
    case asRandomise:
      // Step 6
      // No message to send here, just setting the state will cause the main loop to call us
      // back when the 100ms wait is finished.
      this->addr_state_ = asRandomiseWait;
      break;
    case asRandomiseWait:
    case asResetParams:
      // Step 7
      this->addr_min_ = 0;
      this->addr_max_ = 0xfffffe;
      // Fallthrough
    case asReadySend:
      // Step 8
      this->addr_mid_ = (this->addr_min_ + this->addr_max_) / 2;
      ESP_LOGD(TAG, "Searching with min %06X, max %06X, mid %06X", this->addr_min_, this->addr_max_, this->addr_min_);
      m.pri = priTxn;
      m.addr = ADDR_SEARCH_ADDR_H;
      m.msg = (DALIMsg) ((this->addr_mid_ >> 16) & 0xFF);
      this->addr_state_ = asSearchAddrH;
      this->wait_then_send_(m);
      break;
    case asSearchAddrH:
      // Step 9
      m.pri = priTxn;
      m.addr = ADDR_SEARCH_ADDR_M;
      m.msg = (DALIMsg) ((this->addr_mid_ >> 8) & 0xFF);
      this->addr_state_ = asSearchAddrM;
      this->wait_then_send_(m);
      break;
    case asSearchAddrM:
      // Step 10
      m.pri = priTxn;
      m.addr = ADDR_SEARCH_ADDR_L;
      m.msg = (DALIMsg) (this->addr_mid_ & 0xFF);
      this->addr_state_ = asSearchAddrL;
      this->wait_then_send_(m);
      break;
    case asSearchAddrL:
      // Step 11
      m.pri = priTxn;
      m.addr = ADDR_COMPARE;
      m.msg = (DALIMsg) 0;
      m.expect_back_frame = true;
      this->addr_state_ = asCompare;
      this->wait_then_send_(m);
      break;
    case asCompare:
      // Step 12
      // "Failed to send" is dealt with above
      if (cr == crSuccess) {
        // This shouldn't happen - asCompare should get a good back frame, one of the accepted
        // errors below or no back frame
        ESP_LOGE(TAG, "crSuccess on asCompare, min %u, max %u, short %u", this->addr_min_, this->addr_max_,
                 this->addr_short_);
        this->terminate_addressing_(false);
        return;
      } else if (cr == crWrongLength) {
        ESP_LOGW(TAG, "Wrong length on asCompare, min %u, max %u, short %u, bits %d", this->addr_min_, this->addr_max_,
                 this->addr_short_, this->store_.rcvd_bits);
      } else if (cr == crTimingError) {
        ESP_LOGW(TAG, "Wrong length on asCompare, min %u, max %u, short %u, bits %d", this->addr_min_, this->addr_max_,
                 this->addr_short_, this->store_.rcvd_bits);
      }
      if (cr == crNoBackFrame) {  // There are three "got a frame" codes, but this one is unique
        // Case 12b
        this->addr_min_ = this->addr_mid_ + 1;
        if (this->addr_min_ > this->addr_max_) {
          // We've finished our search, no more lamps
          ESP_LOGI(TAG, "No more lamps found");
          this->terminate_addressing_(true);
        } else {
          // Ready to search again
          this->addr_state_ = asReadySend;
          // Call ourselves to kick that off
          this->addressing_cb_(crSuccess, (DALIAddr) 0);
        }
      } else {
        // Case 12c
        if (this->addr_min_ == this->addr_max_) {
          // Step 13
          // We found a lamp! Program its short address.
          // Theoretically, we could be a bit more strict about replies here. It'd be a bit
          // weird if two lamps replied with min==max. But, YOLO. It's probably fine.
          ESP_LOGI(TAG, "Programming long address %06X with short address %u", this->addr_min_, this->addr_short_);
          m.pri = priTxn;
          m.addr = ADDR_PROGRAM_SHORT_ADDR;
          m.msg = (DALIMsg) ((this->addr_short_ << 1) | 1);
          this->addr_state_ = asProgramShortAddr;
          this->wait_then_send_(m);
        } else {
          this->addr_max_ = this->addr_mid_;
          // Ready to search again
          this->addr_state_ = asReadySend;
          // Call ourselves to kick that off
          this->addressing_cb_(crSuccess, (DALIAddr) 0);
        }
      }
      break;
    case asProgramShortAddr:
      // Step 14
      m.pri = priTxn;
      m.addr = ADDR_VERIFY_SHORT_ADDR;
      m.msg = (DALIMsg) ((this->addr_short_ << 1) | 1);
      m.expect_back_frame = true;
      this->addr_state_ = asVerifyShortAddr;
      this->wait_then_send_(m);

      this->addr_short_ = (DALIMsg) (this->addr_short_ + 1);
      break;
    case asVerifyShortAddr:
      // Step 15
      m.pri = priTxn;
      m.addr = ADDR_WITHDRAW;
      m.msg = (DALIMsg) 0;
      this->addr_state_ = asWithdraw;
      this->wait_then_send_(m);
      break;
    case asWithdraw:
      // Ready to reset min/max and start an entirely new search
      this->addr_state_ = asResetParams;
      // Call ourselves to kick that off
      this->addressing_cb_(crSuccess, (DALIAddr) 0);
      break;
  }
}

void DALIBusComponent::terminate_addressing_(bool success) {
  if (this->addr_state_ >= asInitialise) {
    ESP_LOGI(TAG, "Terminating addressing");
    // We sent initialise, so we should terminate to get back out of config mode
    SendMsg m;
    m.pri = priAuto;
    m.addr = ADDR_TERMINATE;
    m.msg = (DALIMsg) 0;
    m.expect_back_frame = false;
    // We don't set a callback here - terminate works or it doesn't, we did our best
    this->addr_state_ = asInactive;
    this->wait_then_send_(m);
  } else {
    this->addr_state_ = asInactive;
  }
}

void DALIBusComponent::process_addr_wait_() {
  if (this->addr_state_ == asResetWait) {
    uint32_t now = micros();
    const uint32_t reset_wait = 350 * 1000;  // 350ms
    if (now - this->addr_wait_start_ >= reset_wait) {
      this->addressing_cb_(crSuccess, 0);
    }
  } else if (this->addr_state_ == asRandomiseWait) {
    uint32_t now = micros();
    const uint32_t randomise_wait = 100 * 1000;  // 100ms
    if (now - this->addr_wait_start_ >= randomise_wait) {
      this->addressing_cb_(crSuccess, 0);
    }
  }
}

void DALIBusComponent::loop() {
  if (this->reset_time_ != 0 && this->reset_time_ < micros()) {
    // We're in scan mode, kick off addressing
    ESP_LOGI(TAG, "Starting addressing");
    this->reset_time_ = 0;
    this->send_reset(ADDR_BROADCAST, this->addr_cb_);
  }
  this->store_.log_any_recv_errors();
  this->process_sent_message_();
  this->process_back_frames_();
  this->process_addr_wait_();
  this->send_message_if_ready_();
}

void DALIBusComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "dali_bus:");
  LOG_PIN("  DALI out: ", this->out_pin_);
  LOG_PIN("  DALI in: ", this->in_pin_);
  ESP_LOGCONFIG(TAG, "  Scan: %s", YESNO(this->scan_));
}

void DALIBusComponent::send_reset(DALIAddr addr, msg_callback_t cb) {
  SendMsg m{
    pri: priConfig,
    addr: (DALIAddr) ((addr << 1) | 1),
    msg: msgReset,
    callback: cb,
  };
  this->wait_then_send_(m);
}

void DALIBusComponent::send_lamp_off(DALIAddr addr, msg_callback_t cb) {
  if (this->addr_state_ != asInactive) {
    return;
  }
  ESP_LOGD(TAG, "send_lamp_off addr %02X", addr);
  SendMsg m{
    pri: priUser,
    addr: (DALIAddr) ((addr << 1) | 1),
    msg: msgOff,
    expect_back_frame: false,
    send_twice: true,
    callback: cb,
  };
  this->wait_then_send_(m);
}

void DALIBusComponent::send_dapc(DALIAddr addr, uint8_t level, msg_callback_t cb) {
  if (this->addr_state_ != asInactive) {
    return;
  }
  ESP_LOGD(TAG, "send_dapc addr %02X, level %02X", addr, level);
  SendMsg m{
    pri: priUser,
    // This is the one time where we _don't_ set the bottom bit - that's how DAPC commands are identified
    addr: (DALIAddr) (addr << 1),
    msg: (DALIMsg) level,
    expect_back_frame: false,
    send_twice: false,
    callback: cb,
  };
  this->wait_then_send_(m);
}

// This command is not addressed to a particular light. However, it will only take effect on
// any lights where memory writing is enabled, which is done by sending "enable write memory" below,
// which _is_ addressed.
void DALIBusComponent::send_dtr0(uint8_t dtr0, msg_callback_t cb) {
  if (this->addr_state_ != asInactive) {
    return;
  }
  ESP_LOGD(TAG, "send_dtr0 val %02X", dtr0);
  SendMsg m{
    pri: priUser,
    addr: ADDR_DTR0,
    msg: (DALIMsg) dtr0,
    expect_back_frame: false,
    send_twice: false,
    callback: cb,
  };
  this->wait_then_send_(m);
}

void DALIBusComponent::send_enable_write_memory(DALIAddr addr, msg_callback_t cb) {
  if (this->addr_state_ != asInactive) {
    return;
  }
  ESP_LOGD(TAG, "send_enable_write_memory addr %02X", addr);
  SendMsg m{
    pri: priUser,
    addr: (DALIAddr) ((addr << 1) | 1),
    msg: msgEnableWriteMemory,
    expect_back_frame: false,
    send_twice: true,
    callback: cb,
  };
  this->wait_then_send_(m);
}

void DALIBusComponent::send_set_fade_time(DALIAddr addr, msg_callback_t cb) {
  if (this->addr_state_ != asInactive) {
    return;
  }
  ESP_LOGD(TAG, "send_set_fade_time addr %02X", addr);
  SendMsg m{
    pri: priUser,
    addr: (DALIAddr) ((addr << 1) | 1),
    msg: msgSetFadeTime,
    expect_back_frame: false,
    send_twice: true,
    callback: cb,
  };
  this->wait_then_send_(m);
}

void DALIBusComponent::send_query_max_level(DALIAddr addr, msg_callback_t cb) {
  if (this->addr_state_ != asInactive) {
    return;
  }
  ESP_LOGD(TAG, "send_query_max_level addr %02X", addr);
  SendMsg m{
    pri: priUser,
    addr: (DALIAddr) ((addr << 1) | 1),
    msg: msgQueryMaxLevel,
    expect_back_frame: true,
    send_twice: false,
    callback: cb,
  };
  this->wait_then_send_(m);
}

void DALIBusComponent::send_query_min_level(DALIAddr addr, msg_callback_t cb) {
  if (this->addr_state_ != asInactive) {
    return;
  }
  ESP_LOGD(TAG, "send_query_min_level addr %02X", addr);
  SendMsg m{
    pri: priUser,
    addr: (DALIAddr) ((addr << 1) | 1),
    msg: msgQueryMinLevel,
    expect_back_frame: true,
    send_twice: false,
    callback: cb,
  };
  this->wait_then_send_(m);
}

}  // namespace dali_bus
}  // namespace esphome
