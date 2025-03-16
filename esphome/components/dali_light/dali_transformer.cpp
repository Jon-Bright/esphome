#include "esphome/core/log.h"
#include "esphome/components/dali_bus/dali_bus.h"
#include "dali_transformer.h"

namespace esphome {
namespace dali_light {

static const char *const TAG = "dali_light";

void DALITransitionTransformer::start() {
  auto cb = std::bind(&DALITransitionTransformer::ready_cb_, this, std::placeholders::_1, std::placeholders::_2);
  this->light_->set_fade_time(this->fade_time_from_ms_(this->length_), cb);
}

void DALITransitionTransformer::ready_cb_(dali_bus::DALICallbackResult cr, uint8_t reply) {
  if (cr != dali_bus::crSuccess) {
    ESP_LOGE(TAG, "Failed setting fade time, cr %u", cr);
    return;
  }
  this->ready_ = true;
}

optional<light::LightColorValues> DALITransitionTransformer::apply() {
  // ready_ will be set true when we're good to set values because fade time
  // is set correctly. Once the values have been set once, we don't need to
  // set them again, so we set it back to false.
  if (!this->ready_) {
    return {};
  }
  this->ready_ = false;
  return this->get_target_values();
}

// There are 15 "standard" fade times ranging from 0.7s to 90.5s, represented
// by values 1-15.
// The formula to calculate the value for us to send is DTR0=log2(ft^2/250000).
// But: there's 15 of them, let's not torture ourselves with logarithms.
// This table is the halfway points between the different fade times.
static const uint32_t thresholds[] = {
    854, 1207, 1707, 2414, 3414, 4828, 6828, 9657, 13657, 19314, 27314, 38627, 54627, 77255,
};

uint8_t DALITransitionTransformer::fade_time_from_ms_(uint32_t ms) {
  for (int i = 0; i < 14; i++) {
    if (ms < thresholds[i]) {
      return i + 1;
    }
  }
  return 15;
}

}  // namespace dali_light
}  // namespace esphome
