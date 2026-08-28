#include "workflow_climate.h"

#include <algorithm>
#include <new>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

using namespace esphome;

static const char *const TAG = "workflow_climate";

void WorkflowClimate::set_inner_climate(climate::Climate *inner) {
  this->inner_ = inner;
  this->traits_ = inner->get_traits();
  inner->add_on_state_callback([this](climate::Climate &value) { this->on_inner_state_(value); });
  inner->add_on_control_callback([this](climate::ClimateCall &call) { this->on_inner_control_(call); });
}

void WorkflowClimate::set_temperature_sensor(sensor::Sensor *sensor) {
  this->temperature_sensor_ = sensor;
  sensor->add_on_state_callback([this](float value) {
    this->external_temperature_ = value;
    this->has_external_temperature_ = std::isfinite(value);
    if (this->initialized_) {
      this->current_temperature = this->has_external_temperature_ ? value : this->inner_->current_temperature;
      this->publish_state();
    }
  });
}

bool WorkflowClimate::inner_mode_supported_(climate::ClimateMode mode) const {
  return this->traits_.supports_mode(mode);
}

bool WorkflowClimate::mode_uses_target_(climate::ClimateMode mode) const {
  return mode != climate::CLIMATE_MODE_OFF && mode != climate::CLIMATE_MODE_FAN_ONLY;
}

bool WorkflowClimate::inner_state_valid_() const {
  if (this->inner_ == nullptr || !this->inner_mode_supported_(this->inner_->mode)) return false;
  if (this->traits_.has_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE) &&
      !std::isfinite(this->inner_->current_temperature))
    return false;
  return !this->mode_uses_target_(this->inner_->mode) || std::isfinite(this->inner_->target_temperature);
}

float WorkflowClimate::effective_min_temperature_() const {
  float value = this->inner_->get_traits().get_visual_min_temperature();
#ifdef USE_CLIMATE_VISUAL_OVERRIDES
  if (std::isfinite(this->visual_min_temperature_override_))
    value = std::max(value, this->visual_min_temperature_override_);
#endif
  return value;
}

float WorkflowClimate::effective_max_temperature_() const {
  float value = this->inner_->get_traits().get_visual_max_temperature();
#ifdef USE_CLIMATE_VISUAL_OVERRIDES
  if (std::isfinite(this->visual_max_temperature_override_))
    value = std::min(value, this->visual_max_temperature_override_);
#endif
  return value;
}

void WorkflowClimate::load_targets_() {
  const float midpoint = (this->effective_min_temperature_() + this->effective_max_temperature_()) * 0.5f;
  this->stored_targets_.version = TARGETS_VERSION;
  for (float &value : this->stored_targets_.values) value = midpoint;
  StoredTargets loaded{};
  if (this->targets_preference_.load(&loaded) && loaded.version == TARGETS_VERSION) {
    this->stored_targets_ = loaded;
    for (float &value : this->stored_targets_.values)
      if (!std::isfinite(value)) value = midpoint;
  }
}

void WorkflowClimate::save_targets_() { this->targets_preference_.save(&this->stored_targets_); }

float WorkflowClimate::target_for_mode_(climate::ClimateMode mode) const {
  return this->stored_targets_.values[static_cast<uint8_t>(mode)];
}

void WorkflowClimate::set_target_for_mode_(climate::ClimateMode mode, float value, bool persist) {
  value = std::max(this->effective_min_temperature_(), std::min(this->effective_max_temperature_(), value));
  this->stored_targets_.values[static_cast<uint8_t>(mode)] = value;
  if (persist) this->save_targets_();
}

void WorkflowClimate::setup() {
  if (this->inner_ == nullptr) {
    ESP_LOGE(TAG, "No inner climate configured");
    this->mark_failed();
    return;
  }
  const auto traits = this->inner_->get_traits();
  this->traits_ = traits;
  this->set_supported_custom_fan_modes(traits.get_supported_custom_fan_modes());
  this->set_supported_custom_presets(traits.get_supported_custom_presets());
  if (traits.has_feature_flags(climate::CLIMATE_SUPPORTS_TWO_POINT_TARGET_TEMPERATURE |
                               climate::CLIMATE_REQUIRES_TWO_POINT_TARGET_TEMPERATURE)) {
    ESP_LOGE(TAG, "Two-point target climates are not supported");
    this->mark_failed();
    return;
  }
  if (!traits.supports_mode(climate::CLIMATE_MODE_HEAT) &&
      !traits.supports_mode(climate::CLIMATE_MODE_COOL)) {
    ESP_LOGE(TAG, "Inner climate must support HEAT or COOL");
    this->mark_failed();
    return;
  }
  this->compatible_ = true;

  this->targets_preference_ = global_preferences->make_preference<StoredTargets>(
      this->get_object_id_hash() ^ 0x57464C57UL);
  this->load_targets_();
  this->restored_state_ = this->restore_state_();
  this->last_power_change_ = millis();

  if (this->pid_enabled_) {
    this->pid_ = new (std::nothrow) AdaptivePID(
        this->kp_, this->ki_, this->kd_, this->effective_min_temperature_(), this->effective_max_temperature_());
    if (this->pid_ == nullptr) {
      this->mark_failed();
      return;
    }
    this->pid_->set_learning_rates(0.02f, 0.005f, 0.003f);
    this->pid_->set_plant_sensitivity(1.0f);
    this->pid_->set_adapt_interval_ms(15000);
    this->pid_->enable_adaptation(true);
  }

  if (this->temperature_sensor_ != nullptr && this->temperature_sensor_->has_state()) {
    this->external_temperature_ = this->temperature_sensor_->state;
    this->has_external_temperature_ = std::isfinite(this->external_temperature_);
  }
  if (this->inner_state_valid_()) this->initialize_from_inner_();
}

void WorkflowClimate::initialize_from_inner_() {
  if (!this->compatible_ || this->initialized_) return;
  this->initialized_ = true;
  this->copy_inner_state_(true);
  if (this->restored_state_.has_value()) this->apply_restored_state_();
  else {
    this->set_target_for_mode_(this->mode, this->target_for_mode_(this->mode), false);
    this->target_temperature = this->target_for_mode_(this->mode);
    this->save_targets_();
    this->publish_state();
  }
}

void WorkflowClimate::apply_restored_state_() {
  const auto &restore = *this->restored_state_;
  climate::ClimateMode restored_mode = restore.mode;
  if (!this->inner_mode_supported_(restored_mode)) {
    ESP_LOGW(TAG, "Restored mode is no longer supported; starting OFF");
    restored_mode = climate::CLIMATE_MODE_OFF;
  }
  this->mode = restored_mode;
  this->target_temperature = this->target_for_mode_(restored_mode);
  if (restore.uses_custom_fan_mode) {
    if (restore.custom_fan_mode < this->traits_.get_supported_custom_fan_modes().size())
      this->set_custom_fan_mode_(this->traits_.get_supported_custom_fan_modes()[restore.custom_fan_mode]);
  } else {
    this->set_fan_mode_(restore.fan_mode);
  }
  this->swing_mode = restore.swing_mode;
  if (restore.uses_custom_preset) {
    if (restore.custom_preset < this->traits_.get_supported_custom_presets().size())
      this->set_custom_preset_(this->traits_.get_supported_custom_presets()[restore.custom_preset]);
  } else {
    this->set_preset_(restore.preset);
  }
  if (this->traits_.has_feature_flags(climate::CLIMATE_SUPPORTS_TARGET_HUMIDITY))
    this->target_humidity = restore.target_humidity;

  auto call = this->inner_->make_call();
  call.set_mode(this->mode);
  if (this->mode_uses_target_(this->mode) && std::isfinite(this->target_temperature))
    call.set_target_temperature(this->target_temperature);
  if (this->has_custom_fan_mode()) {
    call.set_fan_mode(this->get_custom_fan_mode().c_str());
    this->pending_fan_ = {true, true, climate::CLIMATE_FAN_AUTO,
                          this->get_custom_fan_mode().str(), millis() + this->command_ack_timeout_};
  } else if (this->fan_mode.has_value()) {
    call.set_fan_mode(*this->fan_mode);
    this->pending_fan_ = {true, false, *this->fan_mode, "", millis() + this->command_ack_timeout_};
  }
  if (this->traits_.supports_swing_mode(this->swing_mode)) {
    call.set_swing_mode(this->swing_mode);
    this->pending_swing_ = {true, this->swing_mode, millis() + this->command_ack_timeout_};
  }
  if (this->has_custom_preset()) {
    call.set_preset(this->get_custom_preset().c_str());
    this->pending_preset_ = {true, true, climate::CLIMATE_PRESET_NONE,
                             this->get_custom_preset().str(), millis() + this->command_ack_timeout_};
  } else if (this->preset.has_value()) {
    call.set_preset(*this->preset);
    this->pending_preset_ = {true, false, *this->preset, "", millis() + this->command_ack_timeout_};
  }
  if (this->traits_.has_feature_flags(climate::CLIMATE_SUPPORTS_TARGET_HUMIDITY) &&
      std::isfinite(this->target_humidity)) {
    call.set_target_humidity(this->target_humidity);
    this->pending_humidity_ = {true, this->target_humidity, millis() + this->command_ack_timeout_};
  }
  this->pending_mode_ = {true, this->mode, millis() + this->command_ack_timeout_};
  if (this->mode_uses_target_(this->mode) && std::isfinite(this->target_temperature))
    this->pending_target_ = {true, this->target_temperature, millis() + this->command_ack_timeout_};
  this->perform_inner_call_(call);
  this->publish_state();
}

void WorkflowClimate::copy_inner_state_(bool allow_mode_change) {
  this->action = this->inner_->action;
  this->current_temperature = this->has_external_temperature_ ? this->external_temperature_ : this->inner_->current_temperature;
  this->current_humidity = this->inner_->current_humidity;
  this->target_humidity = this->inner_->target_humidity;
  if (this->inner_->has_custom_fan_mode())
    this->set_custom_fan_mode_(this->inner_->get_custom_fan_mode());
  else if (this->inner_->fan_mode.has_value())
    this->set_fan_mode_(*this->inner_->fan_mode);
  else {
    this->fan_mode.reset();
    this->clear_custom_fan_mode_();
  }
  this->swing_mode = this->inner_->swing_mode;
  if (this->inner_->has_custom_preset())
    this->set_custom_preset_(this->inner_->get_custom_preset());
  else if (this->inner_->preset.has_value())
    this->set_preset_(*this->inner_->preset);
  else {
    this->preset.reset();
    this->clear_custom_preset_();
  }
  if (allow_mode_change) {
    this->mode = this->inner_->mode;
    this->target_temperature = this->target_for_mode_(this->mode);
  }
}

void WorkflowClimate::on_inner_state_(climate::Climate &inner) {
  if (!this->compatible_) return;
  if (!this->initialized_) {
    if (this->inner_state_valid_()) this->initialize_from_inner_();
    return;
  }
  bool suppress_mode = this->pending_mode_.active || this->mode_retry_required_;
  if (this->pending_mode_.active && inner.mode == this->pending_mode_.expected) {
    this->pending_mode_.active = false;
    this->mode_retry_required_ = false;
  }
  if (this->workflow_power_off_ && inner.mode == climate::CLIMATE_MODE_OFF) suppress_mode = true;
  if (this->pending_target_.active && std::isfinite(inner.target_temperature) &&
      std::fabs(inner.target_temperature - this->pending_target_.expected) < 0.01f)
    this->pending_target_.active = false;

  const bool suppress_fan = this->pending_fan_.active;
  if (this->pending_fan_.active) {
    const bool matches = this->pending_fan_.custom
        ? inner.has_custom_fan_mode() && inner.get_custom_fan_mode().str() == this->pending_fan_.expected_custom
        : inner.fan_mode.has_value() && *inner.fan_mode == this->pending_fan_.expected;
    if (matches) this->pending_fan_.active = false;
  }
  const bool suppress_swing = this->pending_swing_.active;
  if (this->pending_swing_.active && inner.swing_mode == this->pending_swing_.expected)
    this->pending_swing_.active = false;
  const bool suppress_preset = this->pending_preset_.active;
  if (this->pending_preset_.active) {
    const bool matches = this->pending_preset_.custom
        ? inner.has_custom_preset() && inner.get_custom_preset().str() == this->pending_preset_.expected_custom
        : inner.preset.has_value() && *inner.preset == this->pending_preset_.expected;
    if (matches) this->pending_preset_.active = false;
  }
  const bool suppress_humidity = this->pending_humidity_.active;
  if (this->pending_humidity_.active && std::isfinite(inner.target_humidity) &&
      std::fabs(inner.target_humidity - this->pending_humidity_.expected) < 0.01f)
    this->pending_humidity_.active = false;

  const bool saved_custom_fan = this->has_custom_fan_mode();
  const std::string saved_custom_fan_value = this->get_custom_fan_mode().str();
  const auto saved_fan = this->fan_mode;
  const auto saved_swing = this->swing_mode;
  const bool saved_custom_preset = this->has_custom_preset();
  const std::string saved_custom_preset_value = this->get_custom_preset().str();
  const auto saved_preset = this->preset;
  const float saved_humidity = this->target_humidity;

  this->copy_inner_state_(!suppress_mode);
  if (suppress_fan) {
    if (saved_custom_fan) this->set_custom_fan_mode_(saved_custom_fan_value.c_str());
    else if (saved_fan.has_value()) this->set_fan_mode_(*saved_fan);
  }
  if (suppress_swing) this->swing_mode = saved_swing;
  if (suppress_preset) {
    if (saved_custom_preset) this->set_custom_preset_(saved_custom_preset_value.c_str());
    else if (saved_preset.has_value()) this->set_preset_(*saved_preset);
  }
  if (suppress_humidity) this->target_humidity = saved_humidity;
  this->publish_state();
}

void WorkflowClimate::on_inner_control_(climate::ClimateCall &call) {
  if (!this->compatible_ || this->internal_inner_call_ || !this->initialized_) return;
  bool changed = false;
  if (call.get_mode().has_value()) {
    this->pending_mode_.active = false;
    this->mode_retry_required_ = false;
    this->workflow_power_off_ = false;
    this->mode = *call.get_mode();
    this->target_temperature = this->target_for_mode_(this->mode);
    changed = true;
  }
  if (call.get_target_temperature().has_value()) this->pending_target_.active = false;
  if (call.has_custom_fan_mode()) {
    this->pending_fan_.active = false;
    this->set_custom_fan_mode_(call.get_custom_fan_mode());
    changed = true;
  } else if (call.get_fan_mode().has_value()) {
    this->pending_fan_.active = false;
    this->set_fan_mode_(*call.get_fan_mode());
    changed = true;
  }
  if (call.get_swing_mode().has_value()) {
    this->pending_swing_.active = false;
    this->swing_mode = *call.get_swing_mode();
    changed = true;
  }
  if (call.has_custom_preset()) {
    this->pending_preset_.active = false;
    this->set_custom_preset_(call.get_custom_preset());
    changed = true;
  } else if (call.get_preset().has_value()) {
    this->pending_preset_.active = false;
    this->set_preset_(*call.get_preset());
    changed = true;
  }
  if (call.get_target_humidity().has_value()) {
    this->pending_humidity_.active = false;
    this->target_humidity = *call.get_target_humidity();
    changed = true;
  }
  if (changed) this->publish_state();
}

void WorkflowClimate::perform_inner_call_(climate::ClimateCall &call) {
  this->internal_inner_call_ = true;
  call.perform();
  this->internal_inner_call_ = false;
}

void WorkflowClimate::control(const climate::ClimateCall &call) {
  if (!this->initialized_) return;
  const bool has_mode = call.get_mode().has_value();
  const bool has_target = call.get_target_temperature().has_value();
  if (has_mode) {
    this->mode = *call.get_mode();
    this->workflow_power_off_ = false;
    this->mode_retry_required_ = false;
    if (!has_target) this->target_temperature = this->target_for_mode_(this->mode);
  }
  if (has_target) {
    this->target_temperature = *call.get_target_temperature();
    this->set_target_for_mode_(this->mode, this->target_temperature);
  }
  if (call.has_custom_fan_mode()) this->set_custom_fan_mode_(call.get_custom_fan_mode());
  else if (call.get_fan_mode().has_value()) this->set_fan_mode_(*call.get_fan_mode());
  if (call.get_swing_mode().has_value()) this->swing_mode = *call.get_swing_mode();
  if (call.has_custom_preset()) this->set_custom_preset_(call.get_custom_preset());
  else if (call.get_preset().has_value()) this->set_preset_(*call.get_preset());
  if (call.get_target_humidity().has_value()) this->target_humidity = *call.get_target_humidity();

  auto inner_call = this->inner_->make_call();
  if (has_mode) {
    inner_call.set_mode(this->mode);
    this->pending_mode_ = {true, this->mode, millis() + this->command_ack_timeout_};
  }
  if (has_target || (has_mode && this->mode_uses_target_(this->mode) && std::isfinite(this->target_temperature))) {
    inner_call.set_target_temperature(this->target_temperature);
    this->pending_target_ = {true, this->target_temperature, millis() + this->command_ack_timeout_};
  }
  if (call.has_custom_fan_mode()) {
    inner_call.set_fan_mode(call.get_custom_fan_mode().c_str());
    this->pending_fan_ = {true, true, climate::CLIMATE_FAN_AUTO, call.get_custom_fan_mode().str(),
                          millis() + this->command_ack_timeout_};
  } else if (call.get_fan_mode().has_value()) {
    inner_call.set_fan_mode(*call.get_fan_mode());
    this->pending_fan_ = {true, false, *call.get_fan_mode(), "", millis() + this->command_ack_timeout_};
  }
  if (call.get_swing_mode().has_value()) {
    inner_call.set_swing_mode(*call.get_swing_mode());
    this->pending_swing_ = {true, *call.get_swing_mode(), millis() + this->command_ack_timeout_};
  }
  if (call.has_custom_preset()) {
    inner_call.set_preset(call.get_custom_preset().c_str());
    this->pending_preset_ = {true, true, climate::CLIMATE_PRESET_NONE, call.get_custom_preset().str(),
                             millis() + this->command_ack_timeout_};
  } else if (call.get_preset().has_value()) {
    inner_call.set_preset(*call.get_preset());
    this->pending_preset_ = {true, false, *call.get_preset(), "", millis() + this->command_ack_timeout_};
  }
  if (call.get_target_humidity().has_value()) {
    inner_call.set_target_humidity(*call.get_target_humidity());
    this->pending_humidity_ = {true, *call.get_target_humidity(), millis() + this->command_ack_timeout_};
  }
  this->perform_inner_call_(inner_call);
  this->publish_state();
}

bool WorkflowClimate::is_workflow_mode_() const {
  return this->mode == climate::CLIMATE_MODE_HEAT || this->mode == climate::CLIMATE_MODE_COOL;
}
bool WorkflowClimate::cycle_change_allowed_() const {
  return static_cast<uint32_t>(millis() - this->last_power_change_) >= this->min_cycle_duration_;
}

void WorkflowClimate::request_workflow_mode_(climate::ClimateMode mode) {
  if (this->pending_mode_.active) return;
  auto call = this->inner_->make_call();
  call.set_mode(mode);
  if (this->mode_uses_target_(mode) && std::isfinite(this->target_temperature))
    call.set_target_temperature(this->target_temperature);
  this->pending_mode_ = {true, mode, millis() + this->command_ack_timeout_};
  this->workflow_power_off_ = mode == climate::CLIMATE_MODE_OFF;
  this->last_power_change_ = millis();
  this->perform_inner_call_(call);
}

void WorkflowClimate::request_workflow_target_(float target) {
  if (this->pending_target_.active) return;
  auto call = this->inner_->make_call();
  call.set_target_temperature(target);
  this->pending_target_ = {true, target, millis() + this->command_ack_timeout_};
  this->perform_inner_call_(call);
}

void WorkflowClimate::run_hysteresis_(float current, float target) {
  if (!this->hysteresis_enabled_ || this->pending_mode_.active) return;
  const bool physically_on = this->inner_->mode != climate::CLIMATE_MODE_OFF;
  const float delta = this->mode == climate::CLIMATE_MODE_HEAT ? current - target : target - current;
  if (physically_on && delta > this->hysteresis_off_ && this->cycle_change_allowed_())
    this->request_workflow_mode_(climate::CLIMATE_MODE_OFF);
  else if (!physically_on && -delta > this->hysteresis_on_ && this->cycle_change_allowed_())
    this->request_workflow_mode_(this->mode);
}

float WorkflowClimate::quantize_target_(float value) const {
  const float step = this->inner_->get_traits().get_visual_target_temperature_step();
  if (step > 0.0f) value = std::round(value / step) * step;
  return std::max(this->effective_min_temperature_(), std::min(this->effective_max_temperature_(), value));
}

void WorkflowClimate::run_pid_(float current, float target) {
  if (!this->pid_enabled_ || this->pid_ == nullptr || this->workflow_power_off_) return;
  const bool heating = this->mode == climate::CLIMATE_MODE_HEAT;
  if (!this->pid_target_initialized_ || this->pid_heating_ != heating ||
      std::fabs(this->pid_->get_target() - target) > 0.01f) {
    this->pid_->set_target(target, heating);
    this->pid_target_initialized_ = true;
    this->pid_heating_ = heating;
  }
  const float min_adjustment = heating ? this->max_adjustment_under_ : this->max_adjustment_over_;
  const float max_adjustment = heating ? this->max_adjustment_over_ : this->max_adjustment_under_;
  const float minimum = std::max(this->effective_min_temperature_(), target - min_adjustment);
  const float maximum = std::min(this->effective_max_temperature_(), target + max_adjustment);
  float corrected = this->pid_->update(current, millis(), true);
  corrected = this->quantize_target_(std::max(minimum, std::min(maximum, corrected)));
  const float step = this->inner_->get_traits().get_visual_target_temperature_step();
  if (step > 0.0f) {
    const float quantized_minimum = std::ceil((minimum - 0.0001f) / step) * step;
    const float quantized_maximum = std::floor((maximum + 0.0001f) / step) * step;
    if (quantized_minimum <= quantized_maximum)
      corrected = std::max(quantized_minimum, std::min(quantized_maximum, corrected));
  } else {
    corrected = std::max(minimum, std::min(maximum, corrected));
  }
  if (this->pid_setpoint_sensor_ != nullptr) this->pid_setpoint_sensor_->publish_state(corrected);
  if (!std::isfinite(this->inner_->target_temperature) ||
      std::fabs(this->inner_->target_temperature - corrected) > 0.01f)
    this->request_workflow_target_(corrected);
}

void WorkflowClimate::run_workflows_() {
  if (!this->initialized_ || !this->is_workflow_mode_()) return;
  if (this->pending_mode_.active || this->mode_retry_required_) return;
  const float current = this->has_external_temperature_ ? this->external_temperature_ : this->inner_->current_temperature;
  const float target = this->target_temperature;
  if (!std::isfinite(current) || !std::isfinite(target)) return;
  this->run_hysteresis_(current, target);
  this->run_pid_(current, target);
  if (!this->pid_enabled_ && !this->workflow_power_off_ && this->inner_->mode != climate::CLIMATE_MODE_OFF &&
      (!std::isfinite(this->inner_->target_temperature) ||
       std::fabs(this->inner_->target_temperature - target) > 0.01f))
    this->request_workflow_target_(target);
}

void WorkflowClimate::check_ack_timeouts_() {
  const uint32_t now = millis();
  if (this->pending_mode_.active && static_cast<int32_t>(now - this->pending_mode_.deadline) >= 0) {
    ESP_LOGW(TAG, "Inner mode command was not acknowledged; retaining outer intent");
    this->pending_mode_.active = false;
    this->mode_retry_required_ = true;
  }
  if (this->pending_target_.active && static_cast<int32_t>(now - this->pending_target_.deadline) >= 0) {
    ESP_LOGW(TAG, "Inner target command was not acknowledged; retaining logical target");
    this->pending_target_.active = false;
  }
  if (this->pending_fan_.active && static_cast<int32_t>(now - this->pending_fan_.deadline) >= 0) {
    ESP_LOGW(TAG, "Inner fan command was not acknowledged");
    this->pending_fan_.active = false;
  }
  if (this->pending_swing_.active && static_cast<int32_t>(now - this->pending_swing_.deadline) >= 0) {
    ESP_LOGW(TAG, "Inner swing command was not acknowledged");
    this->pending_swing_.active = false;
  }
  if (this->pending_preset_.active && static_cast<int32_t>(now - this->pending_preset_.deadline) >= 0) {
    ESP_LOGW(TAG, "Inner preset command was not acknowledged");
    this->pending_preset_.active = false;
  }
  if (this->pending_humidity_.active && static_cast<int32_t>(now - this->pending_humidity_.deadline) >= 0) {
    ESP_LOGW(TAG, "Inner humidity command was not acknowledged");
    this->pending_humidity_.active = false;
  }
}

void WorkflowClimate::loop() {
  this->check_ack_timeouts_();
  const uint32_t now = millis();
  if (this->initialized_ && this->mode_retry_required_ && !this->pending_mode_.active &&
      this->cycle_change_allowed_())
    this->request_workflow_mode_(this->mode);
  if (static_cast<uint32_t>(now - this->last_workflow_run_) >= this->update_interval_) {
    this->last_workflow_run_ = now;
    this->run_workflows_();
  }
}

void WorkflowClimate::dump_config() {
  LOG_CLIMATE("", "Workflow Climate", this);
  ESP_LOGCONFIG(TAG, "  PID: %s", YESNO(this->pid_enabled_));
  ESP_LOGCONFIG(TAG, "  Hysteresis: %s", YESNO(this->hysteresis_enabled_));
  ESP_LOGCONFIG(TAG, "  Update interval: %u ms", this->update_interval_);
  ESP_LOGCONFIG(TAG, "  Command acknowledgement timeout: %u ms", this->command_ack_timeout_);
}
