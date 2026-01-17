#include "devicestate_types.h"
#include "floats.h"

#include "esphome.h"

using namespace devicestate;

namespace devicestate {

    static const char* TAG = "DeviceStateTypes"; // Logging tag

    bool deviceStatusEqual(const DeviceStatus& left, const DeviceStatus& right) {
        return left.operating == right.operating &&
            devicestate::same_float(left.currentTemperature, right.currentTemperature, 0.01f) &&
            devicestate::same_float(left.compressorFrequency, right.compressorFrequency, 0.01f) &&
            devicestate::same_float(left.inputPower, right.inputPower, 0.01f) &&
            devicestate::same_float(left.kWh, right.kWh, 0.01f) &&
            devicestate::same_float(left.runtimeHours, right.runtimeHours, 0.01f);
    }

    bool deviceStateEqual(const DeviceState& left, const DeviceState& right) {
        return left.active == right.active &&
            left.mode == right.mode &&
            left.fanMode == right.fanMode &&
            left.swingMode == right.swingMode &&
            left.verticalSwingMode == right.verticalSwingMode &&
            left.horizontalSwingMode == right.horizontalSwingMode &&
            devicestate::same_float(left.targetTemperature, right.targetTemperature, 0.01f);
    }

    bool isDeviceActive(heatpumpSettings& currentSettings) {
        if (currentSettings.power == nullptr) return false;
        return strcmp(currentSettings.power, "ON") == 0;
    }

    SwingMode toSwingMode(heatpumpSettings& currentSettings) {
        if (currentSettings.vane == nullptr || currentSettings.wideVane == nullptr) {
            return SwingMode::SwingMode_Off;
        }
        if (strcmp(currentSettings.vane, "SWING") == 0 &&
                strcmp(currentSettings.wideVane, "SWING") == 0) {
            return SwingMode::SwingMode_Both;
        } else if (strcmp(currentSettings.vane, "SWING") == 0) {
            return SwingMode::SwingMode_Vertical;
        } else if (strcmp(currentSettings.wideVane, "SWING") == 0) {
            return SwingMode::SwingMode_Horizontal;
        } else {
            return SwingMode::SwingMode_Off;
        }
    }

    VerticalSwingMode toVerticalSwingMode(heatpumpSettings& currentSettings) {
        if (currentSettings.vane == nullptr) {
            return VerticalSwingMode::VerticalSwingMode_Off;
        }
        if (strcmp(currentSettings.vane, "SWING") == 0) {
            return VerticalSwingMode::VerticalSwingMode_Swing;
        } else if (strcmp(currentSettings.vane, "AUTO") == 0) {
            return VerticalSwingMode::VerticalSwingMode_Auto;
        } else if (strcmp(currentSettings.vane, "↑↑") == 0) {
            return VerticalSwingMode::VerticalSwingMode_Up;
        } else if (strcmp(currentSettings.vane, "↑") == 0) {
            return VerticalSwingMode::VerticalSwingMode_UpCenter;
        } else if (strcmp(currentSettings.vane, "—") == 0) {
            return VerticalSwingMode::VerticalSwingMode_Center;
        } else if (strcmp(currentSettings.vane, "↓") == 0) {
            return VerticalSwingMode::VerticalSwingMode_DownCenter;
        } else if (strcmp(currentSettings.vane, "↓↓") == 0) {
            return VerticalSwingMode::VerticalSwingMode_Down;
        } else {
            return VerticalSwingMode::VerticalSwingMode_Off;
        }
    }

    const char* verticalSwingModeToString(VerticalSwingMode mode) {
        /* ******** HANDLE MITSUBISHI VANE CHANGES ********
        * const char* VANE_MAP[7]        = {"AUTO", "1", "2", "3", "4", "5", "SWING"};
        */
        switch(mode) {
            case VerticalSwingMode::VerticalSwingMode_Swing:
                return "SWING";
            case VerticalSwingMode::VerticalSwingMode_Auto:
                return "AUTO";
            case VerticalSwingMode::VerticalSwingMode_Up:
                return "1";
            case VerticalSwingMode::VerticalSwingMode_UpCenter:
                return "2";
            case VerticalSwingMode::VerticalSwingMode_Center:
                return "3";
            case VerticalSwingMode::VerticalSwingMode_DownCenter:
                return "4";
            case VerticalSwingMode::VerticalSwingMode_Down:
                return "5";
            default:
                ESP_LOGW(TAG, "Invalid vertical vane position %d", mode);
                return "UNKNOWN";
        }
    }

    HorizontalSwingMode toHorizontalSwingMode(heatpumpSettings& currentSettings) {
        if (currentSettings.wideVane == nullptr) {
            return HorizontalSwingMode::HorizontalSwingMode_Off;
        }
        if (strcmp(currentSettings.wideVane, "SWING") == 0) {
            return HorizontalSwingMode::HorizontalSwingMode_Swing;
        } else if (strcmp(currentSettings.wideVane, "<>") == 0) {
            return HorizontalSwingMode::HorizontalSwingMode_Auto;
        } else if (strcmp(currentSettings.wideVane, "<<") == 0) {
            return HorizontalSwingMode::HorizontalSwingMode_Left;
        } else if (strcmp(currentSettings.wideVane, "<") == 0) {
            return HorizontalSwingMode::HorizontalSwingMode_LeftCenter;
        } else if (strcmp(currentSettings.wideVane, "|") == 0) {
            return HorizontalSwingMode::HorizontalSwingMode_Center;
        } else if (strcmp(currentSettings.wideVane, ">") == 0) {
            return HorizontalSwingMode::HorizontalSwingMode_RightCenter;
        } else if (strcmp(currentSettings.wideVane, ">>") == 0) {
            return HorizontalSwingMode::HorizontalSwingMode_Right;
        } else {
            return HorizontalSwingMode::HorizontalSwingMode_Off;
        }
    }

    const char* horizontalSwingModeToString(HorizontalSwingMode mode) {
        switch(mode) {
            case HorizontalSwingMode::HorizontalSwingMode_Swing:
                return "SWING";
            case HorizontalSwingMode::HorizontalSwingMode_Auto:
                return "<>";
            case HorizontalSwingMode::HorizontalSwingMode_Left:
                return "<<";
            case HorizontalSwingMode::HorizontalSwingMode_LeftCenter:
                return "<";
            case HorizontalSwingMode::HorizontalSwingMode_Center:
                return "|";
            case HorizontalSwingMode::HorizontalSwingMode_RightCenter:
                return ">";
            case HorizontalSwingMode::HorizontalSwingMode_Right:
                return ">>";
            default:
                ESP_LOGW(TAG, "Invalid horizontal vane position %d", mode);
                return "UNKNOWN";
        }
    }

    FanMode toFanMode(heatpumpSettings& currentSettings) {
        /*
        * ******* HANDLE FAN CHANGES ********
        *
        * const char* FAN_MAP[6]         = {"AUTO", "QUIET", "1", "2", "3", "4"};
        */
        if (currentSettings.fan == nullptr) {
            return FanMode::FanMode_Auto;
        }
        if (strcmp(currentSettings.fan, "AUTO") == 0) {
            return FanMode::FanMode_Auto;
        } else if (strcmp(currentSettings.fan, "QUIET") == 0) {
            return FanMode::FanMode_Quiet;
        } else if (strcmp(currentSettings.fan, "1") == 0) {
            return FanMode::FanMode_Low;
        } else if (strcmp(currentSettings.fan, "2") == 0) {
            return FanMode::FanMode_Medium;
        } else if (strcmp(currentSettings.fan, "3") == 0) {
            return FanMode::FanMode_Middle;
        } else if (strcmp(currentSettings.fan, "4") == 0) {
            return FanMode::FanMode_High;
        } else { //case "AUTO" or default:
            return FanMode::FanMode_Auto;
        }
    }

    const char* fanModeToString(FanMode mode) {
        switch(mode) {
            case FanMode::FanMode_Auto:
                return "AUTO";
            case FanMode::FanMode_Quiet:
                return "QUIET";
            case FanMode::FanMode_Low:
                return "1";
            case FanMode::FanMode_Medium:
                return "2";
            case FanMode::FanMode_Middle:
                return "3";
            case FanMode::FanMode_High:
                return "4";
            default:
                ESP_LOGW(TAG, "Invalid fan mode %d", mode);
                return "UNKNOWN";
        }
    }

    const char* swingModeToString(SwingMode swingMode) {
        switch (swingMode) {
            case SwingMode::SwingMode_Both:
                return "Both";
            case SwingMode::SwingMode_Horizontal:
                return "Horizontal";
            case SwingMode::SwingMode_Vertical:
                return "Vertical";
            case SwingMode::SwingMode_Off:
                return "Off";
            default:
                ESP_LOGW(TAG, "Invalid swing mode %d", swingMode);
                return "UNKNOWN";
        }
    }

    DeviceMode toDeviceMode(heatpumpSettings& currentSettings) {
        if (currentSettings.mode == nullptr) {
            return DeviceMode::DeviceMode_Fan;
        }
        if (strcmp(currentSettings.mode, "HEAT") == 0) {
            return DeviceMode::DeviceMode_Heat;
        } else if (strcmp(currentSettings.mode, "DRY") == 0) {
            return DeviceMode::DeviceMode_Dry;
        } else if (strcmp(currentSettings.mode, "COOL") == 0) {
            return DeviceMode::DeviceMode_Cool;
        } else if (strcmp(currentSettings.mode, "FAN") == 0) {
            return DeviceMode::DeviceMode_Fan;
        } else if (strcmp(currentSettings.mode, "AUTO") == 0) {
            return DeviceMode::DeviceMode_Auto;
        } else {
            ESP_LOGW(TAG, "Invalid device mode %s", currentSettings.mode);
            return DeviceMode::DeviceMode_Fan;
        }
    }

    // Function to convert a Color enum value to its string 
    // representation 
    const char* deviceModeToString(DeviceMode mode) 
    { 
        switch (mode) {
            case DeviceMode::DeviceMode_Heat:
            return "HEAT";
            case DeviceMode::DeviceMode_Cool:
            return "COOL";
            case DeviceMode::DeviceMode_Dry:
            return "DRY";
            case DeviceMode::DeviceMode_Fan:
            return "FAN";
            case DeviceMode::DeviceMode_Auto:
            return "AUTO";
            default:
            ESP_LOGW(TAG, "Invalid device mode %d", mode);
            return "AUTO";
        }
    }

    DeviceStatus toDeviceStatus(heatpumpStatus& currentStatus) {
        DeviceStatus deviceStatus;
        deviceStatus.currentTemperature = currentStatus.roomTemperature;
        deviceStatus.outsideTemperature = currentStatus.outsideAirTemperature;
        deviceStatus.operating = currentStatus.operating;
        deviceStatus.compressorFrequency = currentStatus.compressorFrequency;
        deviceStatus.inputPower = currentStatus.inputPower;
        deviceStatus.kWh = currentStatus.kWh;
        deviceStatus.runtimeHours = currentStatus.runtimeHours;
        return deviceStatus;
    }

    DeviceState toDeviceState(heatpumpSettings& currentSettings) {
        DeviceState deviceState;
        deviceState.active = isDeviceActive(currentSettings);
        deviceState.mode = toDeviceMode(currentSettings);
        deviceState.fanMode = toFanMode(currentSettings);
        deviceState.swingMode = toSwingMode(currentSettings);
        deviceState.verticalSwingMode = toVerticalSwingMode(currentSettings);
        deviceState.horizontalSwingMode = toHorizontalSwingMode(currentSettings);
        deviceState.targetTemperature = currentSettings.temperature;
        return deviceState;
    }

    void log_device_state(const DeviceState& state) {
        ESP_LOGI(TAG, "Device State");
        ESP_LOGI(TAG, "  active: %s", TRUEFALSE(state.active));
        ESP_LOGI(TAG, "  mode: %s", devicestate::deviceModeToString(state.mode));
        ESP_LOGI(TAG, "  swingMode: %s", devicestate::swingModeToString(state.swingMode));
        ESP_LOGI(TAG, "  fanMode: %s", devicestate::fanModeToString(state.fanMode));
        ESP_LOGI(TAG, "  horizontalSwingMode: %s", devicestate::horizontalSwingModeToString(state.horizontalSwingMode));
        ESP_LOGI(TAG, "  verticalSwingMode: %s", devicestate::verticalSwingModeToString(state.verticalSwingMode));
        ESP_LOGI(TAG, "  targetTemperature: %f", state.targetTemperature);
    }

    DeviceState& DeviceState::operator=(const DeviceState& other) {
        if (this != &other) {
            active = other.active;
            fanMode = other.fanMode;
            horizontalSwingMode = other.horizontalSwingMode;
            mode = other.mode;
            swingMode = other.swingMode;
            targetTemperature = other.targetTemperature;
            verticalSwingMode = other.verticalSwingMode;
        }
        return *this;
    }

}