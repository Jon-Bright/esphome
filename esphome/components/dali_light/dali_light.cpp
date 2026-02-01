#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/dali_bus/dali_bus.h"
#include "dali_light.h"
#include "dali_transformer.h"

namespace esphome {
namespace dali_light {

static const char *const TAG = "dali_light";

void DALILight::setup() {}

void DALILight::loop() {
  if (!this->levels_query_started_ && this->bus_ != nullptr && this->bus_->is_ready()) {
    this->levels_query_started_ = true;
    this->bus_->send_query_max_level(this->light_id_, std::bind(&DALILight::query_max_level_cb_, this,
                                                                std::placeholders::_1, std::placeholders::_2));
  }
}

light::LightTraits DALILight::get_traits() {
  auto traits = light::LightTraits();
  traits.set_supported_color_modes({light::ColorMode::BRIGHTNESS});
  return traits;
}

std::unique_ptr<light::LightTransformer> DALILight::create_default_transition() {
  return make_unique<DALITransitionTransformer>(this);
}

void DALILight::setup_state(light::LightState *state) { this->state_ = state; }

void DALILight::write_state(light::LightState *state) {
  float brightness = 0.0f;
  state->current_values_as_brightness(&brightness);
  if (!state->current_values.is_on()) {
    this->bus_->send_lamp_off(this->light_id_, nullptr);
    return;
  }
  uint8_t level = (this->max_level_ - this->min_level_) * brightness + this->min_level_;
  this->bus_->send_dapc(this->light_id_, level, nullptr);
}

void DALILight::set_fade_time(uint8_t ft, dali_bus::msg_callback_t cb) {
  if (ft == this->fade_time_) {
    if (cb) {
      cb(dali_bus::crSuccess, 0);
    }
    return;
  }
  this->sending_fade_time_ = ft;
  this->fade_cb_ = cb;
  this->bus_->send_enable_write_memory(this->light_id_, std::bind(&DALILight::enable_write_memory_fade_cb_, this,
                                                                  std::placeholders::_1, std::placeholders::_2));
}

void DALILight::enable_write_memory_fade_cb_(dali_bus::DALICallbackResult cr, uint8_t reply) {
  if (cr != dali_bus::crSuccess) {
    ESP_LOGE(TAG, "Failed enabling write memory, cr %u", cr);
    if (this->fade_cb_) {
      this->fade_cb_(cr, 0);
    }
    this->fade_cb_ = nullptr;
    return;
  }
  this->bus_->send_dtr0(this->sending_fade_time_,
                        std::bind(&DALILight::dtr0_fade_cb_, this, std::placeholders::_1, std::placeholders::_2));
}

void DALILight::dtr0_fade_cb_(dali_bus::DALICallbackResult cr, uint8_t reply) {
  if (cr != dali_bus::crSuccess) {
    ESP_LOGE(TAG, "Failed setting DTR0, cr %u", cr);
    if (this->fade_cb_) {
      this->fade_cb_(cr, 0);
    }
    this->fade_cb_ = nullptr;
    return;
  }
  this->bus_->send_set_fade_time(
      this->light_id_, std::bind(&DALILight::set_fade_time_cb_, this, std::placeholders::_1, std::placeholders::_2));
}

void DALILight::set_fade_time_cb_(dali_bus::DALICallbackResult cr, uint8_t reply) {
  if (cr != dali_bus::crSuccess) {
    ESP_LOGE(TAG, "Failed setting fade time, cr %u", cr);
  } else {
    this->fade_time_ = this->sending_fade_time_;
  }
  if (this->fade_cb_) {
    this->fade_cb_(cr, reply);
    this->fade_cb_ = nullptr;
  }
}

void DALILight::query_max_level_cb_(dali_bus::DALICallbackResult cr, uint8_t reply) {
  if (cr != dali_bus::crGoodBackFrame) {
    ESP_LOGE(TAG, "Failed querying max level, cr %u", cr);
  } else {
    this->max_level_ = reply;
    this->bus_->send_query_min_level(this->light_id_, std::bind(&DALILight::query_min_level_cb_, this,
                                                                std::placeholders::_1, std::placeholders::_2));
  }
}

void DALILight::query_min_level_cb_(dali_bus::DALICallbackResult cr, uint8_t reply) {
  if (cr != dali_bus::crGoodBackFrame) {
    ESP_LOGE(TAG, "Failed querying min level, cr %u", cr);
  } else {
    this->min_level_ = reply;
    ESP_LOGD(TAG, "Light %d, queried levels, min %d, max %d", this->light_id_, this->min_level_, this->max_level_);
    this->levels_query_done_ = true;
  }
}

void DALILight::set_dali_bus(dali_bus::DALIBusComponent *bus) { this->bus_ = bus; }

void DALILight::set_light_id(uint8_t id) { this->light_id_ = (dali_bus::DALIAddr) id; }

void DALILight::dump_config() {
  ESP_LOGCONFIG(TAG, "DALI Light:");
  ESP_LOGCONFIG(TAG, "   Light ID %u", this->light_id_);
}

}  // namespace dali_light
}  // namespace esphome
