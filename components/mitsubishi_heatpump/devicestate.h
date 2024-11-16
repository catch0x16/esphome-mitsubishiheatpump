#define USE_CALLBACKS

#include "HeatPump.h"
#include "esphome.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"

#ifndef DEVICESTATE_H
#define DEVICESTATE_H

#include <chrono>

#include "pidcontroller.h"

namespace devicestate {

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
    int compressorFrequency;
  };
  bool deviceStatusEqual(DeviceStatus left, DeviceStatus right);
  DeviceStatus toDeviceState(heatpumpStatus *currentStatus);

  struct DeviceState {
    bool active;
    DeviceMode mode;
    FanMode fanMode;
    SwingMode swingMode;
    VerticalSwingMode verticalSwingMode;
    HorizontalSwingMode horizontalSwingMode;
    float targetTemperature;
    //bool connected;
  };
  bool deviceStateEqual(DeviceState left, DeviceState right);
  DeviceState toDeviceState(heatpumpSettings *currentSettings);

  struct ConnectionMetadata {
    HardwareSerial *hardwareSerial;
    int baud;
    int rxPin;
    int txPin;
  };

  class DeviceStateManager {
    private:
      ConnectionMetadata connectionMetadata;
      float minTemp;
      float maxTemp;

      float maxAdjustmentUnder;
      float maxAdjustmentOver;
      float hysterisisUnderOff;
      float hysterisisOverOn;

      int disconnected;

      esphome::binary_sensor::BinarySensor* internal_power_on;
      esphome::binary_sensor::BinarySensor* device_state_connected;
      esphome::binary_sensor::BinarySensor* device_state_active;
      esphome::sensor::Sensor* device_set_point;
      esphome::sensor::Sensor* device_state_last_updated;
      esphome::binary_sensor::BinarySensor* device_status_operating;
      esphome::sensor::Sensor* device_status_current_temperature;
      esphome::sensor::Sensor* device_status_compressor_frequency;
      esphome::sensor::Sensor* device_status_last_updated;
      esphome::sensor::Sensor* pid_set_point_correction;

      // HeatPump object using the underlying Arduino library.
      HeatPump* hp;

      //uint32_t lastRunWorkflows = esphome::millis();

      uint32_t lastInternalPowerUpdate = esphome::millis();
      bool internalPowerOn;

      float targetTemperature = -1;
      float correctedTargetTemperature = -1;

      bool settingsInitialized;
      DeviceState deviceState;
      int deviceStateLastUpdated;

      bool statusInitialized;
      DeviceStatus deviceStatus;
      int deviceStatusLastUpdated;

      PIDController *pidController;

      bool connect();

      void hpSettingsChanged();
      void hpStatusChanged(heatpumpStatus currentStatus);

      bool shouldThrottle(uint32_t end);
      bool internalTurnOn();
      bool internalTurnOff();
      void internalSetTargetTemperature(const float value);
      void internalSetCorrectedTemperature(const float value);
      void ensurePIDTarget();

      void runHysteresisWorkflow(const float currentTemperature);
      void runPIDControllerWorkflow(const float currentTemperature);

      void dump_state();
      void log_heatpump_settings(heatpumpSettings currentSettings);
      static void log_packet(byte* packet, unsigned int length, char* packetDirection);

    public:
      DeviceStateManager(
        ConnectionMetadata connectionMetadata,
        const uint32_t updateInterval,
        const float minTemp,
        const float maxTemp,
        const float p,
        const float i,
        const float d,
        const float maxAdjustmentUnder,
        const float maxAdjustmentOver,
        const float hysterisisUnderOff,
        const float hysterisisOverOn,
        esphome::binary_sensor::BinarySensor* internal_power_on,
        esphome::binary_sensor::BinarySensor* device_state_connected,
        esphome::binary_sensor::BinarySensor* device_state_active,
        esphome::sensor::Sensor* device_set_point,
        esphome::sensor::Sensor* device_state_last_updated,
        esphome::binary_sensor::BinarySensor* device_status_operating,
        esphome::sensor::Sensor* device_status_current_temperature,
        esphome::sensor::Sensor* device_status_compressor_frequency,
        esphome::sensor::Sensor* device_status_last_updated,
        esphome::sensor::Sensor* pid_set_point_correction
      );

      DeviceStatus getDeviceStatus();
      DeviceState getDeviceState();

      bool isInitialized();
      bool initialize();

      void update();
      bool isInternalPowerOn();

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

      bool commit();

      void turnOn(DeviceMode mode);
      void turnOff();

      float getCurrentTemperature();

      float getTargetTemperature();
      void setTargetTemperature(const float target);

      void setRemoteTemperature(const float current);

      void runWorkflows(const float currentTemperature);
  };
}
#endif