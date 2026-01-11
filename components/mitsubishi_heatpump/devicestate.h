#define USE_CALLBACKS

#include <cstdint>
#include "HeatPump.h"
#include "esphome.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"

#ifndef DEVICESTATE_H
#define DEVICESTATE_H

#include <chrono>

namespace devicestate {

  class DeviceStateManager;

  class WorkflowStep {
    public:
        WorkflowStep(){}
    
        virtual void run(const float currentTemperature, DeviceStateManager* deviceManager) = 0;
  };

  enum DeviceMode {
    DeviceMode_Heat,
    DeviceMode_Cool,
    DeviceMode_Dry,
    DeviceMode_Fan,
    DeviceMode_Auto,
    DeviceMode_Unknown
  };
  DeviceMode toDeviceMode(heatpumpSettings *currentSettings);
  const char* deviceModeToString(DeviceMode mode);

  enum FanMode {
    FanMode_Auto,
    FanMode_Quiet,
    FanMode_Low,
    FanMode_Medium,
    FanMode_Middle,
    FanMode_High
  };
  FanMode toFanMode(heatpumpSettings *currentSettings);
  const char* fanModeToString(FanMode mode);

  enum SwingMode {
    SwingMode_Both,
    SwingMode_Vertical,
    SwingMode_Horizontal,
    SwingMode_Off
  };
  SwingMode toSwingMode(heatpumpSettings *currentSettings);

  enum VerticalSwingMode {
    VerticalSwingMode_Swing,
    VerticalSwingMode_Auto,
    VerticalSwingMode_Up,
    VerticalSwingMode_UpCenter,
    VerticalSwingMode_Center,
    VerticalSwingMode_DownCenter,
    VerticalSwingMode_Down,
    VerticalSwingMode_Off
  };
  VerticalSwingMode toVerticalSwingMode(heatpumpSettings *currentSettings);
  const char* verticalSwingModeToString(VerticalSwingMode mode);

  enum HorizontalSwingMode {
    HorizontalSwingMode_Swing,
    HorizontalSwingMode_Auto,
    HorizontalSwingMode_Left,
    HorizontalSwingMode_LeftCenter,
    HorizontalSwingMode_Center,
    HorizontalSwingMode_RightCenter,
    HorizontalSwingMode_Right,
    HorizontalSwingMode_Off
  };
  HorizontalSwingMode toHorizontalSwingMode(heatpumpSettings *currentSettings);
  const char* horizontalSwingModeToString(HorizontalSwingMode mode);

  struct DeviceStatus {
    bool operating;
    float currentTemperature;
    float compressorFrequency;
    float inputPower;
    float kWh;
    float runtimeHours;
  };
  bool deviceStatusEqual(const DeviceStatus& left, const DeviceStatus& right);
  DeviceStatus toDeviceState(heatpumpStatus *currentStatus);

  struct DeviceState {
    bool active;
    DeviceMode mode;
    FanMode fanMode;
    SwingMode swingMode;
    VerticalSwingMode verticalSwingMode;
    HorizontalSwingMode horizontalSwingMode;
    float targetTemperature;
  };
  bool deviceStateEqual(const DeviceState& left, const DeviceState& right);
  DeviceState toDeviceState(heatpumpSettings *currentSettings);

  struct ConnectionMetadata {
    esphome::uart::UARTComponent* hardwareSerial;
  };

  class DeviceStateManager {
    private:
      ConnectionMetadata connectionMetadata;
      HeatPump* hp;

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

    public:
      DeviceStateManager(
        ConnectionMetadata connectionMetadata,
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

      DeviceStatus getDeviceStatus();
      DeviceState getDeviceState();

      bool isInitialized();
      bool initialize();

      void update();

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

      float getTargetTemperature();
      void setTargetTemperature(const float target);

      void setRemoteTemperature(const float current);
      void setAggressiveRemoteTemperatureRounding(const bool value);

      bool internalTurnOn();
      bool internalTurnOff();
      bool isInternalPowerOn();

      float getCorrectedTargetTemperature();
      bool internalSetCorrectedTemperature(const float value);

      bool getOffsetDirection();
      bool getOffsetDirection(const DeviceState* deviceState);

      bool commit();

      void publish();
  };
}
#endif