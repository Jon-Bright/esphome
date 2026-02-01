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
  std::unique_ptr<light::LightTransformer> create_default_transition() override;
  void setup_state(light::LightState *state) override;
  void write_state(light::LightState *state) override;

  void set_dali_bus(dali_bus::DALIBusComponent *bus);
  void set_light_id(uint8_t id);

  void set_fade_time(uint8_t ft, dali_bus::msg_callback_t cb);

 protected:
  void enable_write_memory_fade_cb_(dali_bus::DALICallbackResult cr, uint8_t reply);
  void dtr0_fade_cb_(dali_bus::DALICallbackResult cr, uint8_t reply);
  void set_fade_time_cb_(dali_bus::DALICallbackResult cr, uint8_t reply);
  void query_max_level_cb_(dali_bus::DALICallbackResult cr, uint8_t reply);
  void query_min_level_cb_(dali_bus::DALICallbackResult cr, uint8_t reply);

  dali_bus::DALIBusComponent *bus_;
  dali_bus::DALIAddr light_id_;
  light::LightState *state_{nullptr};
  uint8_t min_level_{0};
  uint8_t max_level_{254};
  uint8_t fade_time_{0};
  uint8_t sending_fade_time_{0};
  dali_bus::msg_callback_t fade_cb_{nullptr};
  bool levels_query_started_{false};
  bool levels_query_done_{false};
};

}  // namespace dali_light
}  // namespace esphome
