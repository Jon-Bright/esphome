#pragma once

namespace esphome {
namespace dali_light {

class DALILight : public Component {
 public:
  void setup() override;
  void loop() override;

  void set_dali_bus(esphome::dali_bus::DALIBusComponent *bus);
  void set_light_id(int id);

 protected:
  esphome::dali_bus::DALIBusComponent *bus_;
  int light_id_;
};

}  // namespace dali_light
}  // namespace esphome
