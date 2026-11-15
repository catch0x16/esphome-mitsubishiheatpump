#pragma once

#include <cmath>
#include <cstdint>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

#include "climate_device_adapter.h"
#include "controller_types.h"
#include "pid_workflowstep.h"

namespace esphome {
namespace workflow_climate {

// User-facing climate that wraps another ("inner") climate and runs hysteresis
// and PID workflows on top of it.
//
// The outer climate owns the mode and target the user asked for. In HEAT and
// COOL the inner climate is driven by the workflows: it may be switched off
// while the outer climate still reads HEAT, and its setpoint is the PID's
// corrected value rather than the user's target. In every other mode, and for
// fan, swing, presets and humidity, the outer climate is a plain pass-through.
class WorkflowClimate : public Component, public climate::Climate {
 public:
  void set_inner_climate(climate::Climate *inner);
  void set_temperature_sensor(sensor::Sensor *sensor);
  void set_ready_sensor(binary_sensor::BinarySensor *sensor) { this->ready_sensor_ = sensor; }

  void set_internal_power_sensor(binary_sensor::BinarySensor *sensor) { this->internal_power_sensor_ = sensor; }
  void set_device_power_sensor(binary_sensor::BinarySensor *sensor) { this->device_power_sensor_ = sensor; }
  void set_operating_sensor(binary_sensor::BinarySensor *sensor) { this->operating_sensor_ = sensor; }
  void set_device_current_temperature_sensor(sensor::Sensor *sensor) {
    this->device_current_temperature_sensor_ = sensor;
  }
  void set_device_set_point_sensor(sensor::Sensor *sensor) { this->device_set_point_sensor_ = sensor; }
  void set_pid_set_point_sensor(sensor::Sensor *sensor) { this->pid_set_point_sensor_ = sensor; }
  void set_pid_kp_sensor(sensor::Sensor *sensor) { this->pid_kp_sensor_ = sensor; }
  void set_pid_ki_sensor(sensor::Sensor *sensor) { this->pid_ki_sensor_ = sensor; }
  void set_response_rate_sensor(sensor::Sensor *sensor) { this->response_rate_sensor_ = sensor; }

  void set_update_interval(uint32_t value) { this->update_interval_ = value; }
  void set_settle_time(uint32_t value) { this->settle_ms_ = value; }

  void set_pid_enabled(bool value) { this->pid_enabled_ = value; }
  void set_kp(float value) { this->kp_ = value; }
  void set_ki(float value) { this->ki_ = value; }
  void set_kd(float value) { this->kd_ = value; }
  void set_max_adjustment_under(float value) { this->max_adjustment_under_ = value; }
  void set_max_adjustment_over(float value) { this->max_adjustment_over_ = value; }
  void set_pid_adaptive(bool value) { this->tuning_.adaptive = value; }
  void set_pid_response_time(uint32_t ms) { this->tuning_.response_time_s = ms / 1000.0f; }
  void set_pid_dead_time(uint32_t ms) { this->tuning_.dead_time_s = ms / 1000.0f; }
  void set_pid_full_output_gap(float value) { this->tuning_.full_output_gap = value; }
  void set_pid_kp_min(float value) { this->tuning_.kp_min = value; }
  void set_pid_kp_max(float value) { this->tuning_.kp_max = value; }

  void set_hysteresis_enabled(bool value) { this->hysteresis_enabled_ = value; }
  void set_hysteresis_on(float value) { this->hysteresis_on_ = value; }
  void set_hysteresis_off(float value) { this->hysteresis_off_ = value; }
  void set_min_cycle_duration(uint32_t value) { this->min_cycle_ms_ = value; }

  // After the inner climate. An inner climate that sets up even later is
  // picked up when it first reports state.
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI - 1.0f; }
  void setup() override;
  void loop() override;
  void dump_config() override;

 protected:
  static constexpr uint32_t TARGETS_VERSION = 1;
  struct StoredTargets {
    uint32_t version;
    float heat;
    float cool;
  };

  // Learned response rates, degrees per second. Saved so tuning survives a reboot.
  static constexpr uint32_t TUNING_VERSION = 1;
  struct StoredTuning {
    uint32_t version;
    float heat_rate;
    float cool_rate;
  };
  // At most one flash write per this interval.
  static constexpr uint32_t TUNING_SAVE_INTERVAL_MS = 60 * 60 * 1000;

  climate::ClimateTraits traits() override { return this->traits_; }
  void control(const climate::ClimateCall &call) override;

  void refresh_traits_();
  bool inner_ready_() const;
  void initialize_from_inner_();
  void on_inner_state_();
  bool copy_passthrough_state_();
  void handle_power_event_(workflow::PowerEvent event, uint32_t now);
  void adopt_inner_mode_(const char *reason);
  void run_workflows_();
  void publish_diagnostics_();
  void warn_if_off_step_(const char *name, float value);
  void load_tuning_();
  void save_tuning_if_changed_(uint32_t now);

  bool is_workflow_mode_(climate::ClimateMode mode) const {
    return mode == climate::CLIMATE_MODE_HEAT || mode == climate::CLIMATE_MODE_COOL;
  }
  bool is_workflow_mode_() const { return this->is_workflow_mode_(this->mode); }
  bool mode_uses_target_(climate::ClimateMode mode) const {
    return mode != climate::CLIMATE_MODE_OFF && mode != climate::CLIMATE_MODE_FAN_ONLY;
  }
  float room_temperature_() const;
  float effective_min_temperature_() const;
  float effective_max_temperature_() const;

  float stored_target_(climate::ClimateMode mode) const;
  void store_target_(climate::ClimateMode mode, float value);
  void load_targets_();

  climate::Climate *inner_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  binary_sensor::BinarySensor *ready_sensor_{nullptr};

  binary_sensor::BinarySensor *internal_power_sensor_{nullptr};
  binary_sensor::BinarySensor *device_power_sensor_{nullptr};
  binary_sensor::BinarySensor *operating_sensor_{nullptr};
  sensor::Sensor *device_current_temperature_sensor_{nullptr};
  sensor::Sensor *device_set_point_sensor_{nullptr};
  sensor::Sensor *pid_set_point_sensor_{nullptr};
  sensor::Sensor *pid_kp_sensor_{nullptr};
  sensor::Sensor *pid_ki_sensor_{nullptr};
  sensor::Sensor *response_rate_sensor_{nullptr};

  climate::ClimateTraits traits_;
  ClimateDeviceAdapter device_;
  devicestate::WorkflowStep *hysteresis_step_{nullptr};
  workflow::pid::PidWorkflowStep *pid_step_{nullptr};
  workflow::pid::TuningConfig tuning_{};
  ESPPreferenceObject tuning_preference_;
  StoredTuning stored_tuning_{};
  uint32_t last_tuning_save_{0};

  optional<climate::ClimateDeviceRestoreState> restored_state_;
  ESPPreferenceObject targets_preference_;
  StoredTargets stored_targets_{};
  bool targets_loaded_{false};

  bool compatible_{false};
  bool initialized_{false};
  bool in_control_{false};

  bool pid_enabled_{false};
  bool hysteresis_enabled_{false};
  float kp_{0.0f};
  float ki_{0.0f};
  float kd_{0.0f};
  float max_adjustment_under_{2.0f};
  float max_adjustment_over_{2.0f};
  float hysteresis_on_{0.25f};
  float hysteresis_off_{0.25f};

  uint32_t update_interval_{2000};
  uint32_t settle_ms_{9000};
  uint32_t min_cycle_ms_{60000};
  uint32_t last_workflow_run_{0};
};

}  // namespace workflow_climate
}  // namespace esphome
