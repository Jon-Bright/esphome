#pragma once

#include "esphome/components/light/light_output.h"

namespace esphome {
namespace dali_light {

class DALILight : public Component, public light::LightOutput {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  light::LightTraits get_traits() override;
  void setup_state(light::LightState *state) override;
  void write_state(light::LightState *state) override;

  void set_dali_bus(dali_bus::DALIBusComponent *bus);
  void set_light_id(uint8_t id);

 protected:
  dali_bus::DALIBusComponent *bus_;
  dali_bus::DALIAddr light_id_;
  light::LightState *state_{nullptr};
  uint8_t min_level_{0};
  uint8_t max_level_{254};
};

}  // namespace dali_light
}  // namespace esphome
