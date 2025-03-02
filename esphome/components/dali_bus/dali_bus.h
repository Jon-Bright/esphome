#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"

#ifdef USE_ESP32_FRAMEWORK_ARDUINO
#include <esp32-hal-timer.h>
#endif

namespace esphome {
namespace dali_bus {

// DALI uses Manchester encoding, meaning every bit has two halves.
// Frames start with a start bit, which is always low-high
// Data bits follow. A zero is encoded as high-low, a one is encoded as low-high.
// There may be 8, 16, or 24 data bits.
// Frames end with a stop bit, which is the bus being idle for >=2450us.
// Half-bits should be between 366.7us and 466.7us, typically 416.7us.
// Double half-bits (when a zero follows a one or vice-versa) should be between 733.3us and 933.3us, typically 833.3us.

// DALIRecvState represents where in a received bitstream we currently are
enum DALIRecvState : uint8_t {
  rsIdle,        // Nothing is happening on the bus
  rsSending,     // We're sending on the bus (we should ignore anything we receive, it's us)
  rsStartBitH1,  // We're receiving the first half of a start bit (DALI low)
  rsStartBitH2,  // We're receiving the second half of a start bit (DALI high)
  rsFirstHalf,   // We're receiving the first half of a normal data bit
  rsSecondHalf,  // We're receiving the second half of a normal data bit
  rsFrameReady,  // We've seen a stop bit, so our data frame is ready
};

// DALITime represents what the time between two edges on the input pin can validly represent
enum DALITime : uint8_t {
  // The time between edges on the bus was:
  tiTooShort,   // too short to represent even half a bit
  tiHalfBit,    // valid for half a bit
  tiInvalid,    // too long to be half a bit but not long enough to be two half-bits
  ti2HalfBits,  // valid for two half-bits
  tiTooLong,    // too long to represent even two half-bits
};

// DALISendState is the state of an ongoing send
enum DALISendState : uint8_t {
  ssStartBit,
  ssDataBits,
  ssStopBit,
  ssSuccess,
  ssFailed,
};

struct DALIInterrupt {
  ISRInternalGPIOPin in_pin;
  ISRInternalGPIOPin out_pin;
#ifdef USE_ESP32_FRAMEWORK_ARDUINO
  hw_timer_t *timer;
#endif

  volatile uint32_t last_dali_high{0};
  volatile uint32_t last_dali_low{0};
  volatile DALIRecvState recv_state{rsIdle};

  volatile uint8_t rcvd_bits{0};
  volatile uint32_t rcvd_val{0};

  volatile uint8_t send_half_bits{0};
  volatile uint32_t send_val{0};
  volatile uint32_t low_time_at_start_of_high{0};
  volatile DALISendState send_state{ssSuccess};

  static void gpio_intr(DALIInterrupt *d);
  static void timer_intr(DALIInterrupt *d);
#ifdef USE_ESP_IDF
  static bool timer_intr_bool(void *d);
#endif
  void received_bit(bool bit);
  void dali_high();
  void dali_low();
  void dali_idle();
  void send_next_half_bit();
  void start_stop_bit_timer(void);
  void start_half_bit_timer(void);
  void stop_stop_bit_timer(void);

  DALITime get_bit_time(void);
};

class DALIBusComponent : public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::BUS; }

  void set_scan(bool scan) { scan_ = scan; }
  void set_dali_out_pin(InternalGPIOPin *out_pin) { out_pin_ = out_pin; }
  void set_dali_in_pin(InternalGPIOPin *in_pin) { in_pin_ = in_pin; }

 protected:
  InternalGPIOPin *out_pin_;
  InternalGPIOPin *in_pin_;
  bool scan_;

 private:
  void setup_timer();
  DALIInterrupt store_;
};

}  // namespace dali_bus
}  // namespace esphome
