#pragma once

#include "cycle_management.h"
#include "devicestatemanager.h"

#include "esphome.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/select/select.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"

namespace devicestate {
class CN105ControlFlow;
}

static const char *const MITSUBISHI_CN105_VERSION = "5.0.0";
static const uint32_t MITSUBISHI_CN105_POLL_INTERVAL_DEFAULT = 500;
static const float MITSUBISHI_CN105_TARGET_TEMPERATURE_STEP = 0.5f;
static const float MITSUBISHI_CN105_CURRENT_TEMPERATURE_STEP = 0.1f;

class MitsubishiCN105Climate : public esphome::Component,
                              public esphome::climate::Climate,
                              public esphome::uart::UARTDevice {
 public:
  explicit MitsubishiCN105Climate(esphome::uart::UARTComponent *uart,
                                  uint32_t poll_interval = MITSUBISHI_CN105_POLL_INTERVAL_DEFAULT);

  float get_setup_priority() const override { return esphome::setup_priority::AFTER_WIFI; }
  void setup() override;
  void loop() override;
  void dump_config() override;
  void control(const esphome::climate::ClimateCall &call) override;
  esphome::climate::ClimateTraits traits() override;
  esphome::climate::ClimateTraits &config_traits() { return this->traits_; }

  uint32_t get_update_interval() const { return this->update_interval_; }
  void set_update_interval(uint32_t update_interval);
  void set_remote_temp_timeout(uint32_t timeout);
  void set_debounce_delay(uint32_t delay);
  void set_remote_temperature(float temperature);
  void set_remote_temperature_sensor(esphome::sensor::Sensor *sensor);

  void set_vertical_vane_select(esphome::select::Select *select);
  void set_horizontal_vane_select(esphome::select::Select *select);

  void set_device_state_connected_sensor(esphome::binary_sensor::BinarySensor *sensor) { this->device_state_connected_ = sensor; }
  void set_device_state_active_sensor(esphome::binary_sensor::BinarySensor *sensor) { this->device_state_active_ = sensor; }
  void set_device_status_operating_sensor(esphome::binary_sensor::BinarySensor *sensor) { this->device_status_operating_ = sensor; }
  void set_device_status_current_temperature_sensor(esphome::sensor::Sensor *sensor) { this->device_status_current_temperature_ = sensor; }
  void set_device_status_outside_temperature_sensor(esphome::sensor::Sensor *sensor) { this->device_status_outside_temperature_ = sensor; }
  void set_device_status_compressor_frequency_sensor(esphome::sensor::Sensor *sensor) { this->device_status_compressor_frequency_ = sensor; }
  void set_device_status_input_power_sensor(esphome::sensor::Sensor *sensor) { this->device_status_input_power_ = sensor; }
  void set_device_status_kwh_sensor(esphome::sensor::Sensor *sensor) { this->device_status_kwh_ = sensor; }
  void set_device_status_runtime_hours_sensor(esphome::sensor::Sensor *sensor) { this->device_status_runtime_hours_ = sensor; }
  void set_device_set_point_sensor(esphome::sensor::Sensor *sensor) { this->device_set_point_ = sensor; }

 protected:
  void terminate_cycle_();
  void update_device_();
  void control_delegate_(const esphome::climate::ClimateCall &call);
  void update_swing_horizontal_(const std::string &swing);
  void update_swing_vertical_(const std::string &swing);
  void on_horizontal_swing_change_(const std::string &swing);
  void on_vertical_swing_change_(const std::string &swing);

  esphome::climate::ClimateTraits traits_;
  esphome::uart::UARTComponent *uart_;
  cycleManagement loop_cycle_{};
  uint32_t update_interval_{MITSUBISHI_CN105_POLL_INTERVAL_DEFAULT};
  uint32_t debounce_delay_{100};
  uint32_t remote_temp_timeout_{UINT32_MAX};
  float min_temp_{ESPMHP_MIN_TEMPERATURE};
  float max_temp_{ESPMHP_MAX_TEMPERATURE};

  devicestate::DeviceStateManager *device_manager_{nullptr};
  devicestate::CN105ControlFlow *control_flow_{nullptr};
  devicestate::CN105State *cn105_state_{nullptr};

  esphome::select::Select *vertical_vane_select_{nullptr};
  esphome::select::Select *horizontal_vane_select_{nullptr};
  esphome::sensor::Sensor *remote_temperature_sensor_{nullptr};
  std::string vertical_swing_state_{"auto"};
  std::string horizontal_swing_state_{"auto"};

  esphome::binary_sensor::BinarySensor *device_state_connected_{nullptr};
  esphome::binary_sensor::BinarySensor *device_state_active_{nullptr};
  esphome::binary_sensor::BinarySensor *device_status_operating_{nullptr};
  esphome::sensor::Sensor *device_status_current_temperature_{nullptr};
  esphome::sensor::Sensor *device_status_outside_temperature_{nullptr};
  esphome::sensor::Sensor *device_status_compressor_frequency_{nullptr};
  esphome::sensor::Sensor *device_status_input_power_{nullptr};
  esphome::sensor::Sensor *device_status_kwh_{nullptr};
  esphome::sensor::Sensor *device_status_runtime_hours_{nullptr};
  esphome::sensor::Sensor *device_set_point_{nullptr};
};
