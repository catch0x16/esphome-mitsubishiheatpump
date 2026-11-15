#pragma once

#include <cmath>
#include <cstdint>

#include "esphome/components/climate/climate.h"

#include "controller_types.h"
#include "power_tracker.h"

namespace esphome {
namespace workflow_climate {

// Presents any single-target climate::Climate to the controller steps as an
// IDeviceStateManager. Owns everything about the *device-facing* state: the
// commanded power, the corrected (PID) setpoint, and the throttling.
class ClimateDeviceAdapter : public devicestate::IDeviceStateManager {
 public:
  void set_inner(climate::Climate *inner) { this->inner_ = inner; }
  void set_limits(float min_temp, float max_temp, float step);
  void set_min_cycle_ms(uint32_t value) { this->min_cycle_ms_ = value; }
  void set_settle_ms(uint32_t value) { this->tracker_.set_settle_ms(value); }

  // Call once the inner climate has valid state.
  void begin(bool inner_on, uint32_t now);

  // The user-facing (logical) mode and target, owned by the wrapper.
  void set_logical(climate::ClimateMode mode, float target) {
    this->logical_mode_ = mode;
    this->target_ = target;
  }

  float clamp_target(float value) const;
  float quantize_target(float value) const;
  // As quantize_target, but the result never leaves [min_allowed, max_allowed]:
  // a value that would round past a limit is rounded the other way.
  float quantize_within(float value, float min_allowed, float max_allowed) const;

  // The wrapper sent a mode to the inner climate on the user's behalf.
  void note_power_command(bool on, uint32_t now) { this->tracker_.command(on, now); }
  // The wrapper sent a setpoint alongside a mode change.
  void note_target_sent(float quantized, uint32_t now);
  // Adopt the inner climate's power state as the commanded one.
  void adopt_power(bool on) { this->tracker_.adopt(on); }

  workflow::PowerEvent observe(bool outer_on, uint32_t now);
  bool settling(uint32_t now) const { return this->tracker_.settling(now); }
  uint32_t last_power_command_ms() const { return this->tracker_.last_command_ms(); }

  // Send the commanded power state to the inner climate again.
  void resend_power(uint32_t now);
  // Keep the device setpoint on the user's target when no PID is configured.
  void sync_target();

  float corrected_target() const { return this->corrected_; }
  bool device_power_on() const;
  bool device_operating() const;

  // IDeviceStateManager
  devicestate::DeviceStatus getDeviceStatus() override;
  devicestate::DeviceState getDeviceState() override;
  float getTargetTemperature() override { return this->target_; }
  bool getOffsetDirection() override { return this->logical_mode_ == climate::CLIMATE_MODE_HEAT; }
  void commit() override;
  bool internalTurnOn() override;
  bool internalTurnOff() override;
  bool isInternalPowerOn() override { return this->tracker_.commanded_on(); }
  bool internalSetCorrectedTemperature(const float value, const float minAllowed, const float maxAllowed) override;

 protected:
  bool throttled_(uint32_t now) const;
  void send_on_(uint32_t now);
  void send_off_(uint32_t now);

  climate::Climate *inner_{nullptr};
  workflow::PowerTracker tracker_;

  climate::ClimateMode logical_mode_{climate::CLIMATE_MODE_OFF};
  float target_{NAN};

  float min_temp_{NAN};
  float max_temp_{NAN};
  float step_{0.0f};

  // Last setpoint requested by the controller, before rounding to the device step.
  float corrected_{NAN};
  float staged_target_{NAN};
  bool has_staged_target_{false};
  float last_sent_target_{NAN};
  uint32_t last_target_sent_ms_{0};
  bool has_sent_target_{false};

  uint32_t min_cycle_ms_{60000};
  uint32_t last_power_change_ms_{0};
};

}  // namespace workflow_climate
}  // namespace esphome
