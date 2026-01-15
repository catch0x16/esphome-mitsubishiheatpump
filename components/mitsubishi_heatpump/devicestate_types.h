#pragma once

#include "cn105_types.h"

using namespace devicestate;

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
    float compressorFrequency;
    float inputPower;
    float kWh;
    float runtimeHours;
  };
  bool deviceStatusEqual(const DeviceStatus& left, const DeviceStatus& right);
  DeviceStatus toDeviceStatus(heatpumpStatus *currentStatus);

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

  class IDeviceStateManager {
    public:
        virtual DeviceStatus getDeviceStatus() = 0;
        virtual DeviceState getDeviceState() = 0;

        virtual float getTargetTemperature() = 0;
        virtual bool getOffsetDirection() = 0;

        virtual void commit() = 0;
        virtual bool internalTurnOn() = 0;
        virtual bool internalTurnOff() = 0;
        virtual bool isInternalPowerOn() = 0;

        virtual bool internalSetCorrectedTemperature(const float value) = 0;

        virtual ~IDeviceStateManager() = default;    // Virtual destructor for safety
  };

  class WorkflowStep {
    public:
        WorkflowStep(){}
    
        virtual void run(const float currentTemperature, IDeviceStateManager* deviceManager) = 0;
  };

}
