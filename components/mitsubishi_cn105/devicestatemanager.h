#pragma once

#define USE_CALLBACKS

#include <cstdint>
#include "devicestate_types.h"

#include "cn105_state.h"

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"

namespace devicestate {

  class DeviceStateManager {
    private:
      CN105State* hpState;

      float minTemp;
      float maxTemp;

      esphome::binary_sensor::BinarySensor* device_state_active;
      esphome::sensor::Sensor* device_set_point;
      esphome::binary_sensor::BinarySensor* device_status_operating;
      esphome::sensor::Sensor* device_status_current_temperature;
      esphome::sensor::Sensor* device_status_outside_temperature;
      esphome::sensor::Sensor* device_status_compressor_frequency;
      esphome::sensor::Sensor* device_status_input_power;
      esphome::sensor::Sensor* device_status_kwh;
      esphome::sensor::Sensor* device_status_runtime_hours;

      bool settingsInitialized = false;
      DeviceState deviceState{};

      bool statusInitialized = false;
      DeviceStatus deviceStatus{};

      void hpSettingsChanged();
      void hpStatusChanged();

      void dump_state();
      void log_heatpump_settings(heatpumpSettings& currentSettings);
      void log_heatpump_status(heatpumpStatus& currentStatus);
      static void log_packet(uint8_t* packet, unsigned int length, char* packetDirection);

    public:
      DeviceStateManager(
        CN105State* hpState,
        const float minTemp,
        const float maxTemp,
        esphome::binary_sensor::BinarySensor* device_state_active,
        esphome::sensor::Sensor* device_set_point,
        esphome::binary_sensor::BinarySensor* device_status_operating,
        esphome::sensor::Sensor* device_status_current_temperature,
        esphome::sensor::Sensor* device_status_outside_temperature,
        esphome::sensor::Sensor* device_status_compressor_frequency,
        esphome::sensor::Sensor* device_status_input_power,
        esphome::sensor::Sensor* device_status_kwh,
        esphome::sensor::Sensor* device_status_runtime_hours
      );

      DeviceStatus getDeviceStatus();
      DeviceState getDeviceState();

      bool isInitialized();
      void reset();

      void update();
      void commit();

      void setCool();
      void setHeat();
      void setDry();
      void setAuto();
      void setFan();

      bool setVerticalSwingMode(VerticalSwingMode mode);
      bool setHorizontalSwingMode(HorizontalSwingMode mode);
      bool setFanMode(FanMode mode);

      void turnOn(DeviceMode mode);
      void turnOff();

      float getCurrentTemperature();

      float getTargetTemperature();
      void setTargetTemperature(const float target);

      void publish();
  };
}
