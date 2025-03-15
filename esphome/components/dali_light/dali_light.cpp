#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/dali_bus/dali_bus.h"
#include "dali_light.h"

namespace esphome {
namespace dali_light {

static const char *const TAG = "dali_light";

void DALILight::setup() {}

void DALILight::loop() {}

void DALILight::set_dali_bus(esphome::dali_bus::DALIBusComponent *bus) { this->bus_ = bus; }

void DALILight::set_light_id(uint8_t id) { this->light_id_ = id; }

void DALILight::dump_config() {
  ESP_LOGCONFIG(TAG, "DALI Light:");
  ESP_LOGCONFIG(TAG, "   Light ID %u", this->light_id_);
}

}  // namespace dali_light
}  // namespace esphome
