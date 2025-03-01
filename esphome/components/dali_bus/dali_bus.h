#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"

namespace esphome {
namespace dali_bus {

struct DALIInterrupt {
  ISRInternalGPIOPin in_pin;
  static void gpio_intr(DALIInterrupt *d);
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
  DALIInterrupt store_;
};

}  // namespace dali_bus
}  // namespace esphome
