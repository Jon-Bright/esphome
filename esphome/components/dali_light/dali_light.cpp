#include "esphome/core/component.h"
#include "esphome/components/dali_bus/dali_bus.h"
#include "dali_light.h"

namespace esphome {
namespace dali_light {

void DALILight::setup() {}

void DALILight::loop() {}

void DALILight::set_dali_bus(esphome::dali_bus::DALIBusComponent *bus) { this->bus_ = bus; }

void DALILight::set_light_id(uint8_t id) { this->light_id_ = id; }

}  // namespace dali_light
}  // namespace esphome
