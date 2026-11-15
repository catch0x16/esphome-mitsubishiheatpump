#pragma once

// The controller's view of the device it drives. Nothing here depends on a
// particular climate implementation.
namespace devicestate {

  enum DeviceMode {
    DeviceMode_Heat,
    DeviceMode_Cool,
    DeviceMode_Other
  };

  inline const char* deviceModeToString(DeviceMode mode) {
    switch (mode) {
      case DeviceMode_Heat: return "HEAT";
      case DeviceMode_Cool: return "COOL";
      default: return "OTHER";
    }
  }

  struct DeviceStatus {
    bool operating;
    float currentTemperature;
  };

  struct DeviceState {
    bool active;
    DeviceMode mode;
    float targetTemperature;
  };

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

        // Stage a corrected setpoint. It is rounded to the device's step, and
        // the rounded value stays inside [minAllowed, maxAllowed].
        virtual bool internalSetCorrectedTemperature(const float value, const float minAllowed, const float maxAllowed) = 0;

        virtual ~IDeviceStateManager() = default;
  };

  class WorkflowStep {
    public:
        WorkflowStep(){}
        virtual ~WorkflowStep() = default;

        virtual void run(const float currentTemperature, IDeviceStateManager* deviceManager) = 0;
  };

}
