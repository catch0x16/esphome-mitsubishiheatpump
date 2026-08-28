#pragma once

#include <cmath>

#include "adaptive_pid.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"

class WorkflowClimate : public esphome::Component, public esphome::climate::Climate {
 public:
  void set_inner_climate(esphome::climate::Climate *inner);
  void set_temperature_sensor(esphome::sensor::Sensor *sensor);
  void set_pid_setpoint_sensor(esphome::sensor::Sensor *sensor) { this->pid_setpoint_sensor_ = sensor; }

  void set_update_interval(uint32_t value) { this->update_interval_ = value; }
  void set_command_ack_timeout(uint32_t value) { this->command_ack_timeout_ = value; }
  void set_pid_enabled(bool value) { this->pid_enabled_ = value; }
  void set_kp(float value) { this->kp_ = value; }
  void set_ki(float value) { this->ki_ = value; }
  void set_kd(float value) { this->kd_ = value; }
  void set_max_adjustment_under(float value) { this->max_adjustment_under_ = value; }
  void set_max_adjustment_over(float value) { this->max_adjustment_over_ = value; }
  void set_hysteresis_enabled(bool value) { this->hysteresis_enabled_ = value; }
  void set_hysteresis_on(float value) { this->hysteresis_on_ = value; }
  void set_hysteresis_off(float value) { this->hysteresis_off_ = value; }
  void set_min_cycle_duration(uint32_t value) { this->min_cycle_duration_ = value; }

  float get_setup_priority() const override { return esphome::setup_priority::AFTER_WIFI - 1.0f; }
  void setup() override;
  void loop() override;
  void dump_config() override;
  void control(const esphome::climate::ClimateCall &call) override;
  esphome::climate::ClimateTraits traits() override { return this->traits_; }

 protected:
  static constexpr uint32_t TARGETS_VERSION = 1;
  struct StoredTargets {
    uint32_t version;
    float values[7];
  };
  struct PendingMode {
    bool active{false};
    esphome::climate::ClimateMode expected{esphome::climate::CLIMATE_MODE_OFF};
    uint32_t deadline{0};
  };
  struct PendingTarget {
    bool active{false};
    float expected{NAN};
    uint32_t deadline{0};
  };
  struct PendingFan {
    bool active{false};
    bool custom{false};
    esphome::climate::ClimateFanMode expected{esphome::climate::CLIMATE_FAN_AUTO};
    std::string expected_custom;
    uint32_t deadline{0};
  };
  struct PendingSwing {
    bool active{false};
    esphome::climate::ClimateSwingMode expected{esphome::climate::CLIMATE_SWING_OFF};
    uint32_t deadline{0};
  };
  struct PendingPreset {
    bool active{false};
    bool custom{false};
    esphome::climate::ClimatePreset expected{esphome::climate::CLIMATE_PRESET_NONE};
    std::string expected_custom;
    uint32_t deadline{0};
  };
  struct PendingHumidity {
    bool active{false};
    float expected{NAN};
    uint32_t deadline{0};
  };

  void initialize_from_inner_();
  void on_inner_state_(esphome::climate::Climate &inner);
  void on_inner_control_(esphome::climate::ClimateCall &call);
  void copy_inner_state_(bool allow_mode_change);
  void apply_restored_state_();
  void run_workflows_();
  void run_hysteresis_(float current, float target);
  void run_pid_(float current, float target);
  void request_workflow_mode_(esphome::climate::ClimateMode mode);
  void request_workflow_target_(float target);
  void check_ack_timeouts_();
  bool cycle_change_allowed_() const;
  bool is_workflow_mode_() const;
  bool inner_state_valid_() const;
  bool mode_uses_target_(esphome::climate::ClimateMode mode) const;
  bool inner_mode_supported_(esphome::climate::ClimateMode mode) const;
  void perform_inner_call_(esphome::climate::ClimateCall &call);

  float target_for_mode_(esphome::climate::ClimateMode mode) const;
  void set_target_for_mode_(esphome::climate::ClimateMode mode, float value, bool persist = true);
  void load_targets_();
  void save_targets_();
  float effective_min_temperature_() const;
  float effective_max_temperature_() const;
  float quantize_target_(float value) const;

  esphome::climate::Climate *inner_{nullptr};
  esphome::sensor::Sensor *temperature_sensor_{nullptr};
  esphome::sensor::Sensor *pid_setpoint_sensor_{nullptr};
  esphome::climate::ClimateTraits traits_;
  esphome::optional<esphome::climate::ClimateDeviceRestoreState> restored_state_;
  esphome::ESPPreferenceObject targets_preference_;
  StoredTargets stored_targets_{};

  AdaptivePID *pid_{nullptr};
  bool pid_enabled_{false};
  bool pid_target_initialized_{false};
  bool pid_heating_{true};
  bool hysteresis_enabled_{false};
  bool compatible_{false};
  bool initialized_{false};
  bool internal_inner_call_{false};
  bool workflow_power_off_{false};
  bool mode_retry_required_{false};
  bool has_external_temperature_{false};
  float external_temperature_{NAN};

  float kp_{0.0f};
  float ki_{0.0f};
  float kd_{0.0f};
  float max_adjustment_under_{2.0f};
  float max_adjustment_over_{2.0f};
  float hysteresis_on_{0.25f};
  float hysteresis_off_{0.25f};
  uint32_t update_interval_{2000};
  uint32_t command_ack_timeout_{30000};
  uint32_t min_cycle_duration_{60000};
  uint32_t last_workflow_run_{0};
  uint32_t last_power_change_{0};
  PendingMode pending_mode_{};
  PendingTarget pending_target_{};
  PendingFan pending_fan_{};
  PendingSwing pending_swing_{};
  PendingPreset pending_preset_{};
  PendingHumidity pending_humidity_{};
};
