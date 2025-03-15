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

  void set_dali_bus(esphome::dali_bus::DALIBusComponent *bus);
  void set_light_id(uint8_t id);

 protected:
  esphome::dali_bus::DALIBusComponent *bus_;
  uint8_t light_id_;
};

}  // namespace dali_light
}  // namespace esphome
