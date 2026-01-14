#define USE_CALLBACKS

#include <cstdint>
#include "devicestate_types.h"

#include "HeatPump.h"
#include "cn105_state.h"

#include "cycle_management.h"

#include "io_device.h"

#include "esphome.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"

#ifndef DEVICESTATE_H
#define DEVICESTATE_H

#include <chrono>

namespace devicestate {

  class DeviceStateManager : public IDeviceStateManager {
    private:
      CN105State* hpState;

      float minTemp;
      float maxTemp;

      int disconnected;

      esphome::binary_sensor::BinarySensor* internal_power_on;
      esphome::binary_sensor::BinarySensor* device_state_connected;
      esphome::binary_sensor::BinarySensor* device_state_active;
      esphome::sensor::Sensor* device_set_point;
      esphome::binary_sensor::BinarySensor* device_status_operating;
      esphome::sensor::Sensor* device_status_current_temperature;
      esphome::sensor::Sensor* device_status_compressor_frequency;
      esphome::sensor::Sensor* device_status_input_power;
      esphome::sensor::Sensor* device_status_kwh;
      esphome::sensor::Sensor* device_status_runtime_hours;
      esphome::sensor::Sensor* pid_set_point_correction;

      uint32_t lastInternalPowerUpdate = esphome::millis();
      bool internalPowerOn;

      float targetTemperature = -1;
      float correctedTargetTemperature = -1;

      bool settingsInitialized;
      DeviceState deviceState;

      bool statusInitialized;
      DeviceStatus deviceStatus;

      bool connect();

      void hpSettingsChanged();
      void hpStatusChanged(heatpumpStatus currentStatus);

      bool shouldThrottle(uint32_t end);

      void dump_state();
      void log_heatpump_settings(heatpumpSettings currentSettings);
      void log_heatpump_status(heatpumpStatus currentStatus);
      static void log_packet(byte* packet, unsigned int length, char* packetDirection);

      float getRoundedTemp(float value);

      bool getOffsetDirection(const DeviceState* deviceState);

    public:
      DeviceStateManager(
        IIODevice* io_device,
        CN105State* hpState,
        const float minTemp,
        const float maxTemp,
        esphome::binary_sensor::BinarySensor* internal_power_on,
        esphome::binary_sensor::BinarySensor* device_state_connected,
        esphome::binary_sensor::BinarySensor* device_state_active,
        esphome::sensor::Sensor* device_set_point,
        esphome::binary_sensor::BinarySensor* device_status_operating,
        esphome::sensor::Sensor* device_status_current_temperature,
        esphome::sensor::Sensor* device_status_compressor_frequency,
        esphome::sensor::Sensor* device_status_input_power,
        esphome::sensor::Sensor* device_status_kwh,
        esphome::sensor::Sensor* device_status_runtime_hours,
        esphome::sensor::Sensor* pid_set_point_correction
      );

      HeatPump* hp;

      DeviceStatus getDeviceStatus();
      DeviceState getDeviceState();

      bool isInitialized();
      bool initialize();

      void update(cycleManagement& loopCycle);

      void setCool();
      void setHeat();
      void setDry();
      void setAuto();
      void setFan();

      bool setVerticalSwingMode(VerticalSwingMode mode);
      bool setVerticalSwingMode(VerticalSwingMode mode, bool commit);
      bool setHorizontalSwingMode(HorizontalSwingMode mode);
      bool setHorizontalSwingMode(HorizontalSwingMode mode, bool commit);
      bool setFanMode(FanMode mode);
      bool setFanMode(FanMode mode, bool commit);

      void turnOn(DeviceMode mode);
      void turnOff();

      float getCurrentTemperature();

      float getTargetTemperature() override;
      void setTargetTemperature(const float target);

      void setRemoteTemperature(const float current);
      void setAggressiveRemoteTemperatureRounding(const bool value);

      bool internalTurnOn() override;
      bool internalTurnOff() override;
      bool isInternalPowerOn() override;

      float getCorrectedTargetTemperature();
      bool internalSetCorrectedTemperature(const float value);

      bool getOffsetDirection() override;

      bool commit() override;

      void publish();
  };
}
#endif