#include "climate_device_adapter.h"

#include <algorithm>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include "floats.h"

namespace esphome {
namespace workflow_climate {

static const char *const TAG = "workflow_climate.device";

void ClimateDeviceAdapter::set_limits(float min_temp, float max_temp, float step) {
  this->min_temp_ = min_temp;
  this->max_temp_ = max_temp;
  this->step_ = step;
}

void ClimateDeviceAdapter::begin(bool inner_on, uint32_t now) {
  this->tracker_.adopt(inner_on);
  // As before the split: no workflow power change during the first cycle after boot.
  this->last_power_change_ms_ = now;
}

float ClimateDeviceAdapter::clamp_target(float value) const {
  return devicestate::clamp(value, this->min_temp_, this->max_temp_);
}

float ClimateDeviceAdapter::quantize_target(float value) const {
  if (this->step_ <= 0.0f) {
    return this->clamp_target(value);
  }
  // Keep the result on the device's step grid without leaving the allowed range.
  const float lowest = std::ceil((this->min_temp_ - 0.0001f) / this->step_) * this->step_;
  const float highest = std::floor((this->max_temp_ + 0.0001f) / this->step_) * this->step_;
  if (lowest > highest) {
    return this->clamp_target(value);
  }
  value = std::round(value / this->step_) * this->step_;
  return std::max(lowest, std::min(highest, value));
}

bool ClimateDeviceAdapter::device_power_on() const {
  return this->inner_->mode != climate::CLIMATE_MODE_OFF;
}

bool ClimateDeviceAdapter::device_operating() const {
  return this->inner_->action != climate::CLIMATE_ACTION_OFF &&
         this->inner_->action != climate::CLIMATE_ACTION_IDLE;
}

workflow::PowerEvent ClimateDeviceAdapter::observe(bool outer_on, uint32_t now) {
  return this->tracker_.observe(this->device_power_on(), outer_on, now);
}

devicestate::DeviceStatus ClimateDeviceAdapter::getDeviceStatus() {
  devicestate::DeviceStatus status{};
  status.operating = this->device_operating();
  status.currentTemperature = this->inner_->current_temperature;
  return status;
}

devicestate::DeviceState ClimateDeviceAdapter::getDeviceState() {
  devicestate::DeviceState state{};
  // Inside the settle window the inner climate's report is not trustworthy yet.
  state.active = this->tracker_.settling(millis()) ? this->tracker_.commanded_on() : this->device_power_on();
  switch (this->logical_mode_) {
    case climate::CLIMATE_MODE_HEAT:
      state.mode = devicestate::DeviceMode_Heat;
      break;
    case climate::CLIMATE_MODE_COOL:
      state.mode = devicestate::DeviceMode_Cool;
      break;
    default:
      state.mode = devicestate::DeviceMode_Other;
      break;
  }
  state.targetTemperature = this->inner_->target_temperature;
  return state;
}

bool ClimateDeviceAdapter::throttled_(uint32_t now) const {
  return static_cast<uint32_t>(now - this->last_power_change_ms_) < this->min_cycle_ms_;
}

void ClimateDeviceAdapter::note_target_sent(float quantized, uint32_t now) {
  this->last_sent_target_ = quantized;
  this->last_target_sent_ms_ = now;
  this->has_sent_target_ = true;
  this->has_staged_target_ = false;
}

void ClimateDeviceAdapter::send_on_(uint32_t now) {
  auto call = this->inner_->make_call();
  call.set_mode(this->logical_mode_);
  // Mode and setpoint go out together so the unit never starts on a stale setpoint.
  const float base = std::isfinite(this->corrected_) ? this->corrected_ : this->target_;
  if (std::isfinite(base)) {
    const float quantized = this->quantize_target(base);
    call.set_target_temperature(quantized);
    this->note_target_sent(quantized, now);
  }
  this->tracker_.command(true, now);
  call.perform();
}

void ClimateDeviceAdapter::send_off_(uint32_t now) {
  auto call = this->inner_->make_call();
  call.set_mode(climate::CLIMATE_MODE_OFF);
  this->tracker_.command(false, now);
  call.perform();
}

void ClimateDeviceAdapter::resend_power(uint32_t now) {
  if (this->tracker_.commanded_on()) {
    this->send_on_(now);
  } else {
    this->send_off_(now);
  }
}

bool ClimateDeviceAdapter::internalTurnOn() {
  const uint32_t now = millis();
  if (this->throttled_(now)) {
    ESP_LOGD(TAG, "Throttling internal turn on");
    return false;
  }
  // Start from the user's target; the PID step corrects it from there.
  this->corrected_ = this->target_;
  this->send_on_(now);
  this->last_power_change_ms_ = now;
  return true;
}

bool ClimateDeviceAdapter::internalTurnOff() {
  const uint32_t now = millis();
  if (this->throttled_(now)) {
    ESP_LOGD(TAG, "Throttling internal turn off");
    return false;
  }
  this->send_off_(now);
  this->last_power_change_ms_ = now;
  return true;
}

float ClimateDeviceAdapter::quantize_within(float value, float min_allowed, float max_allowed) const {
  float quantized = this->quantize_target(value);
  if (this->step_ <= 0.0f) {
    return quantized;
  }
  // Rounding to the nearest step may overshoot a limit that is not itself on
  // the step grid (e.g. target + 0.25 on a 0.5 grid). Step back inside.
  if (quantized > max_allowed + 0.0001f && quantized - this->step_ >= min_allowed - 0.0001f) {
    quantized -= this->step_;
  } else if (quantized < min_allowed - 0.0001f && quantized + this->step_ <= max_allowed + 0.0001f) {
    quantized += this->step_;
  }
  return quantized;
}

bool ClimateDeviceAdapter::internalSetCorrectedTemperature(const float value, const float minAllowed,
                                                           const float maxAllowed) {
  const uint32_t now = millis();
  const float clamped = this->clamp_target(devicestate::clamp(value, minAllowed, maxAllowed));
  const float quantized = this->quantize_within(clamped, minAllowed, maxAllowed);

  // What the device is believed to be set to. Right after a command the inner
  // climate's report may still be stale, and a device that rounds differently
  // would never match, so the last value sent stands in for a while.
  float device_target = this->inner_->target_temperature;
  const bool recently_sent = this->has_sent_target_ &&
                             static_cast<uint32_t>(now - this->last_target_sent_ms_) < this->min_cycle_ms_;
  if (!std::isfinite(device_target) || this->tracker_.settling(now) ||
      (recently_sent && devicestate::same_float(quantized, this->last_sent_target_, 0.01f))) {
    if (this->has_sent_target_) {
      device_target = this->last_sent_target_;
    }
  }

  if (devicestate::same_float(this->corrected_, clamped, 0.01f) &&
      devicestate::same_float(quantized, device_target, 0.01f)) {
    return false;
  }

  ESP_LOGD(TAG, "Corrected setpoint %.2f -> %.2f (device step %.2f, device at %.2f, user target %.2f)",
           this->corrected_, clamped, quantized, device_target, this->target_);
  this->corrected_ = clamped;
  this->staged_target_ = quantized;
  this->has_staged_target_ = true;
  return true;
}

void ClimateDeviceAdapter::commit() {
  if (!this->has_staged_target_) {
    return;
  }
  auto call = this->inner_->make_call();
  call.set_target_temperature(this->staged_target_);
  this->note_target_sent(this->staged_target_, millis());
  call.perform();
}

void ClimateDeviceAdapter::sync_target() {
  if (!this->tracker_.commanded_on() || !std::isfinite(this->target_)) {
    return;
  }
  if (this->internalSetCorrectedTemperature(this->target_, this->min_temp_, this->max_temp_)) {
    this->commit();
  }
}

}  // namespace workflow_climate
}  // namespace esphome
