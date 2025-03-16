#pragma once

#include "esphome/components/light/light_transformer.h"
#include "dali_light.h"

namespace esphome {
namespace dali_light {

// DALI controllers implement their own fading, including their own dimming curve.
// If we send individual fade steps (as would be the default), we'll just produce
// a result that's less smooth and therefore less visually pleasing. To avoid that,
// we implement our own transition which sets the fade time on the controller and
// sends the fade right at the start, relying on the controller to implement the
// intermediate steps.
class DALITransitionTransformer : public light::LightTransformer {
 public:
  DALITransitionTransformer(DALILight *light) { this->light_ = light; }
  void start() override;
  optional<light::LightColorValues> apply() override;

 protected:
  DALILight *light_;
  bool ready_{false};

  void ready_cb_(dali_bus::DALICallbackResult cr, uint8_t reply);
  uint8_t fade_time_from_ms_(uint32_t ms);
};

}  // namespace dali_light
}  // namespace esphome
