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

// DaliState represents where in a bitstream we currently are
enum DaliState : uint8_t {
  stIdle,        // Nothing is happening on the bus
  stSending,     // We're sending on the bus (we should ignore anything we receive, it's us)
  stStartBitH1,  // We're receiving the first half of a start bit (DALI low)
  stStartBitH2,  // We're receiving the second half of a start bit (DALI high)
  stFirstHalf,   // We're receiving the first half of a normal data bit
  stSecondHalf,  // We're receiving the second half of a normal data bit
  stFrameReady,  // We've seen a stop bit, so our data frame is ready
};

// DaliTime represents what the time between two edges on the input pin can validly represent
enum DaliTime {
  // The time between edges on the bus was:
  tiTooShort,   // too short to represent even half a bit
  tiHalfBit,    // valid for half a bit
  tiInvalid,    // too long to be half a bit but not long enough to be two half-bits
  ti2HalfBits,  // valid for two half-bits
  tiTooLong,    // too long to represent even two half-bits
};

struct DALIInterrupt {
  ISRInternalGPIOPin in_pin;
#ifdef USE_ESP32_FRAMEWORK_ARDUINO
  hw_timer_t *timer;
#endif

  volatile uint32_t last_dali_high{0};
  volatile uint32_t last_dali_low{0};
  volatile DaliState state{stIdle};

  static void gpio_intr(DALIInterrupt *d);
  static void timer_intr(DALIInterrupt *d);
#ifdef USE_ESP_IDF
  static bool timer_intr_bool(void *d);
#endif
  void dali_high();
  void dali_low();
  void dali_idle();
  void start_stop_bit_timer(void);
  void stop_stop_bit_timer(void);
};

class DALIBusComponent : public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::BUS; }

  void set_scan(bool scan) { scan_ = scan; }
  void set_dali_out_pin(GPIOPin *out_pin) { out_pin_ = out_pin; }
  void set_dali_in_pin(InternalGPIOPin *in_pin) { in_pin_ = in_pin; }

 protected:
  GPIOPin *out_pin_;
  InternalGPIOPin *in_pin_;
  bool scan_;

 private:
  void setup_timer();
  DALIInterrupt store_;
};

}  // namespace dali_bus
}  // namespace esphome
