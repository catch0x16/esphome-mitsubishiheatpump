#include "devicestate.h"

#include "espmhp.h"
using namespace esphome;

#include "floats.h"
#include <math.h>

namespace devicestate {

    static const char* TAG = "DeviceStateManager"; // Logging tag

    bool deviceStatusEqual(DeviceStatus left, DeviceStatus right) {
        return left.operating == right.operating &&
            devicestate::same_float(left.currentTemperature, right.currentTemperature, 0.01f) &&
            devicestate::same_float(left.compressorFrequency, right.compressorFrequency, 0.01f) &&
            devicestate::same_float(left.inputPower, right.inputPower, 0.01f) &&
            devicestate::same_float(left.kWh, right.kWh, 0.01f) &&
            devicestate::same_float(left.runtimeHours, right.runtimeHours, 0.01f);
    }

    bool deviceStateEqual(DeviceState left, DeviceState right) {
        return left.active == right.active &&
            left.mode == right.mode &&
            left.fanMode == right.fanMode &&
            left.swingMode == right.swingMode &&
            left.verticalSwingMode == right.verticalSwingMode &&
            left.horizontalSwingMode == right.horizontalSwingMode &&
            devicestate::same_float(left.targetTemperature, right.targetTemperature, 0.01f);
    }

    bool isDeviceActive(heatpumpSettings *currentSettings) {
        return strcmp(currentSettings->power, "ON") == 0;
    }

    SwingMode toSwingMode(heatpumpSettings *currentSettings) {
        if (strcmp(currentSettings->vane, "SWING") == 0 &&
                strcmp(currentSettings->wideVane, "SWING") == 0) {
            return SwingMode::SwingMode_Both;
        } else if (strcmp(currentSettings->vane, "SWING") == 0) {
            return SwingMode::SwingMode_Vertical;
        } else if (strcmp(currentSettings->wideVane, "SWING") == 0) {
            return SwingMode::SwingMode_Horizontal;
        } else {
            return SwingMode::SwingMode_Off;
        }
    }

    VerticalSwingMode toVerticalSwingMode(heatpumpSettings *currentSettings) {
        if (strcmp(currentSettings->vane, "SWING") == 0) {
            return VerticalSwingMode::VerticalSwingMode_Swing;
        } else if (strcmp(currentSettings->vane, "AUTO") == 0) {
            return VerticalSwingMode::VerticalSwingMode_Auto;
        } else if (strcmp(currentSettings->vane, "1") == 0) {
            return VerticalSwingMode::VerticalSwingMode_Up;
        } else if (strcmp(currentSettings->vane, "2") == 0) {
            return VerticalSwingMode::VerticalSwingMode_UpCenter;
        } else if (strcmp(currentSettings->vane, "3") == 0) {
            return VerticalSwingMode::VerticalSwingMode_Center;
        } else if (strcmp(currentSettings->vane, "4") == 0) {
            return VerticalSwingMode::VerticalSwingMode_DownCenter;
        } else if (strcmp(currentSettings->vane, "5") == 0) {
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

    HorizontalSwingMode toHorizontalSwingMode(heatpumpSettings *currentSettings) {
        if (strcmp(currentSettings->wideVane, "SWING") == 0) {
            return HorizontalSwingMode::HorizontalSwingMode_Swing;
        } else if (strcmp(currentSettings->wideVane, "<>") == 0) {
            return HorizontalSwingMode::HorizontalSwingMode_Auto;
        } else if (strcmp(currentSettings->wideVane, "<<") == 0) {
            return HorizontalSwingMode::HorizontalSwingMode_Left;
        } else if (strcmp(currentSettings->wideVane, "<") == 0) {
            return HorizontalSwingMode::HorizontalSwingMode_LeftCenter;
        } else if (strcmp(currentSettings->wideVane, "|") == 0) {
            return HorizontalSwingMode::HorizontalSwingMode_Center;
        } else if (strcmp(currentSettings->wideVane, ">") == 0) {
            return HorizontalSwingMode::HorizontalSwingMode_RightCenter;
        } else if (strcmp(currentSettings->wideVane, ">>") == 0) {
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

    FanMode toFanMode(heatpumpSettings *currentSettings) {
        /*
        * ******* HANDLE FAN CHANGES ********
        *
        * const char* FAN_MAP[6]         = {"AUTO", "QUIET", "1", "2", "3", "4"};
        */
        if (strcmp(currentSettings->fan, "QUIET") == 0) {
            return FanMode::FanMode_Quiet;
        } else if (strcmp(currentSettings->fan, "1") == 0) {
            return FanMode::FanMode_Low;
        } else if (strcmp(currentSettings->fan, "2") == 0) {
            return FanMode::FanMode_Medium;
        } else if (strcmp(currentSettings->fan, "3") == 0) {
            return FanMode::FanMode_Middle;
        } else if (strcmp(currentSettings->fan, "4") == 0) {
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

    DeviceMode toDeviceMode(heatpumpSettings *currentSettings) {
        if (strcmp(currentSettings->mode, "HEAT") == 0) {
            return DeviceMode::DeviceMode_Heat;
        } else if (strcmp(currentSettings->mode, "DRY") == 0) {
            return DeviceMode::DeviceMode_Dry;
        } else if (strcmp(currentSettings->mode, "COOL") == 0) {
        return DeviceMode::DeviceMode_Cool;
        } else if (strcmp(currentSettings->mode, "FAN") == 0) {
            return DeviceMode::DeviceMode_Fan;
        } else if (strcmp(currentSettings->mode, "AUTO") == 0) {
            return DeviceMode::DeviceMode_Auto;
        } else {
            ESP_LOGW(TAG, "Invalid device mode %s", currentSettings->mode);
            return DeviceMode::DeviceMode_Unknown;
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
            return "FAN";
        }
    }

    DeviceStatus toDeviceStatus(heatpumpStatus *currentStatus) {
        DeviceStatus deviceStatus;
        deviceStatus.currentTemperature = currentStatus->roomTemperature;
        deviceStatus.operating = currentStatus->operating;
        deviceStatus.compressorFrequency = currentStatus->compressorFrequency;
        deviceStatus.inputPower = currentStatus->inputPower;
        deviceStatus.kWh = currentStatus->kWh;
        deviceStatus.runtimeHours = currentStatus->runtimeHours;
        return deviceStatus;
    }

    DeviceState toDeviceState(heatpumpSettings *currentSettings) {
        /*
        * ************ HANDLE POWER AND MODE CHANGES ***********
        * https://github.com/geoffdavis/HeatPump/blob/stream/src/HeatPump.h#L125
        * const char* POWER_MAP[2]       = {"OFF", "ON"};
        * const char* MODE_MAP[5]        = {"HEAT", "DRY", "COOL", "FAN", "AUTO"};
        */
        DeviceState deviceState;
        deviceState.active = isDeviceActive(currentSettings);
        deviceState.mode = toDeviceMode(currentSettings);
        deviceState.fanMode = toFanMode(currentSettings);
        deviceState.swingMode = toSwingMode(currentSettings);
        deviceState.verticalSwingMode = toVerticalSwingMode(currentSettings);
        deviceState.horizontalSwingMode = toHorizontalSwingMode(currentSettings);
        deviceState.targetTemperature = currentSettings->temperature;
        return deviceState;
    }

    DeviceStateManager::DeviceStateManager(
      ConnectionMetadata connectionMetadata,
      const float minTemp,
      const float maxTemp,
      const float offsetAdjustment,
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
    ) {
        this->connectionMetadata = connectionMetadata;

        this->minTemp = minTemp;
        this->maxTemp = maxTemp;
        this->offsetAdjustment = offsetAdjustment;

        this->internal_power_on = internal_power_on;
        this->device_state_connected = device_state_connected;
        this->device_state_active = device_state_active;
        this->device_set_point = device_set_point;
        this->device_status_operating = device_status_operating;
        this->device_status_current_temperature = device_status_current_temperature;
        this->device_status_compressor_frequency = device_status_compressor_frequency;
        this->device_status_input_power = device_status_input_power;
        this->device_status_kwh = device_status_kwh;
        this->device_status_runtime_hours = device_status_runtime_hours;
        this->pid_set_point_correction = pid_set_point_correction;

        this->disconnected = 0;

        ESP_LOGCONFIG(TAG, "Initializing new HeatPump object.");
        this->hp = new HeatPump();
        this->hp->enableExternalUpdate();

        #ifdef USE_CALLBACKS
            hp->setSettingsChangedCallback(
                    [this]() {
                        ESP_LOGW(TAG, "Callback hpSettingsChanged");
                        this->hpSettingsChanged();
                    }
            );

            hp->setStatusChangedCallback(
                    [this](heatpumpStatus currentStatus) {
                        ESP_LOGW(TAG, "Callback hpStatusChanged");
                        this->hpStatusChanged(currentStatus);
                    }
            );

            hp->setPacketCallback(this->log_packet);
        #endif
    }

    void DeviceStateManager::hpSettingsChanged() {
        heatpumpSettings currentSettings = hp->getSettings();
        ESP_LOGI(TAG, "Heatpump Settings Changed:");
        this->log_heatpump_settings(currentSettings);

        if (currentSettings.power == NULL) {
            /*
            * We should always get a valid pointer here once the HeatPump
            * component fully initializes. If HeatPump hasn't read the settings
            * from the unit yet (hp->connect() doesn't do this, sadly), we'll need
            * to punt on the update. Likely not an issue when run in callback
            * mode, but that isn't working right yet.
            */
            ESP_LOGW(TAG, "Waiting for HeatPump to read the settings the first time.");
            esphome::delay(10);
            return;
        }

        const DeviceState deviceState = devicestate::toDeviceState(&currentSettings);
        if (this->settingsInitialized) {
            if (this->internalPowerOn != deviceState.active) {
                ESP_LOGI(TAG, "Device active on change: deviceState.active={%s} internalPowerOn={%s}", YESNO(deviceState.active), YESNO(this->internalPowerOn));
            }
        } else {
            if (this->internalPowerOn != deviceState.active) {
                ESP_LOGW(TAG, "Initializing internalPowerOn state from %s to %s", ONOFF(this->internalPowerOn), ONOFF(deviceState.active));
                this->internalPowerOn = deviceState.active;
            }
            ESP_LOGW(TAG, "Initializing targetTemperature state from %f to %f", this->targetTemperature, deviceState.targetTemperature);
            this->targetTemperature = deviceState.targetTemperature;
            this->settingsInitialized = true;
        }
        this->deviceState = deviceState;

        this->device_state_connected->publish_state(this->hp->isConnected());
        this->internal_power_on->publish_state(this->internalPowerOn);
        this->device_state_active->publish_state(this->deviceState.active);
        this->device_set_point->publish_state(this->deviceState.targetTemperature);
        this->pid_set_point_correction->publish_state(this->correctedTargetTemperature);
    }

    /**
     * Report changes in the current temperature sensed by the HeatPump.
     */
    void DeviceStateManager::hpStatusChanged(heatpumpStatus currentStatus) {
        if (!this->statusInitialized) {
            this->statusInitialized = true;
            ESP_LOGW(TAG, "HeatPump status initialized.");
        }

        const DeviceStatus newDeviceStatus = devicestate::toDeviceStatus(&currentStatus);
        if (devicestate::deviceStatusEqual(this->deviceStatus, newDeviceStatus)) {
            ESP_LOGW(TAG, "Skipping status change as device status unchanged.");
            return;
        }

        ESP_LOGI(TAG, "HeatPump device status updated.");
        this->deviceStatus = newDeviceStatus;

        this->device_status_operating->publish_state(this->deviceStatus.operating);
        this->device_status_current_temperature->publish_state(this->deviceStatus.currentTemperature);
        this->device_status_compressor_frequency->publish_state(this->deviceStatus.compressorFrequency);
        this->device_status_input_power->publish_state(this->deviceStatus.inputPower);
        this->device_status_kwh->publish_state(this->deviceStatus.kWh);
        this->device_status_runtime_hours->publish_state(this->deviceStatus.runtimeHours);
    }

    void DeviceStateManager::log_packet(byte* packet, unsigned int length, char* packetDirection) {
        String packetHex;
        char textBuf[15];

        for (int i = 0; i < length; i++) {
            memset(textBuf, 0, 15);
            sprintf(textBuf, "%02X ", packet[i]);
            packetHex += textBuf;
        }
        
        if (strcmp(packetDirection, "packetRecv") == 0) {
            const char* packetName = HeatPump::lookupRecvPacketName(packet);
            ESP_LOGD(TAG, "PKT: [%s] (%s) %s", packetDirection, packetName, packetHex.c_str());
        } else {
            const char* packetName = HeatPump::lookupSendPacketName(packet);
            ESP_LOGD(TAG, "PKT: [%s] (%s) %s", packetDirection, packetName, packetHex.c_str());
        }
    }

    bool DeviceStateManager::isInitialized() {
        return this->settingsInitialized && this->statusInitialized;
    }

    bool DeviceStateManager::connect() {
        if (this->hp->connect(
                this->connectionMetadata.hardwareSerial,
                this->connectionMetadata.baud,
                this->connectionMetadata.rxPin,
                this->connectionMetadata.txPin)) {
            ESP_LOGW(TAG, "Connect succeeded");
            this->hp->sync();
            return true;
        }
        ESP_LOGW(TAG, "Connect failed");
        return false;
    }

    bool DeviceStateManager::initialize() {
        return this->connect();
    }

    DeviceStatus DeviceStateManager::getDeviceStatus() {
        return this->deviceStatus;
    }

    DeviceState DeviceStateManager::getDeviceState() {
        return this->deviceState;
    }

    void DeviceStateManager::update() {
        //this->dump_config();
        this->hp->sync();
    #ifndef USE_CALLBACKS
        this->hpSettingsChanged();
        heatpumpStatus currentStatus = hp->getStatus();
        this->hpStatusChanged(currentStatus);
    #endif

        if (!this->isInitialized()) {
            return;
        }

        DeviceState deviceState = this->getDeviceState();
        if (!this->hp->isConnected()) {
            this->disconnected += 1;
            ESP_LOGW(TAG, "Device not connected: %d", this->disconnected);
            if (this->disconnected >= 500) {
                this->connect();
                this->disconnected = 0;
            }
        } else {
            this->disconnected = 0;
        }
    }

    bool DeviceStateManager::isInternalPowerOn() {
        return this->internalPowerOn;
    }

    void DeviceStateManager::setCool() {
        this->turnOn(DeviceMode::DeviceMode_Cool);
    }

    void DeviceStateManager::setHeat() {
        this->turnOn(DeviceMode::DeviceMode_Heat);
    }

    void DeviceStateManager::setDry() {
        this->turnOn(DeviceMode::DeviceMode_Dry);
    }

    void DeviceStateManager::setAuto() {
        this->turnOn(DeviceMode::DeviceMode_Auto);
    }

    void DeviceStateManager::setFan() {
        this->turnOn(DeviceMode::DeviceMode_Fan);
    }

    void DeviceStateManager::turnOn(DeviceMode mode) {
        const char* deviceMode = deviceModeToString(mode);

        this->hp->setModeSetting(deviceMode);
        this->hp->setPowerSetting("ON");
        this->internalPowerOn = true;
        this->internal_power_on->publish_state(this->internalPowerOn);
    }

    void DeviceStateManager::turnOff() {
        this->hp->setPowerSetting("OFF");
        this->internalPowerOn = false;
        this->internal_power_on->publish_state(this->internalPowerOn);
    }

    bool DeviceStateManager::shouldThrottle(uint32_t end) {
        const int durationInMilliseconds = end - this->lastInternalPowerUpdate;
        const int durationInSeconds = durationInMilliseconds / 1000;
        const int remaining = 60 - durationInSeconds; 
        return remaining > 0;
    }

    bool DeviceStateManager::internalTurnOn() {
        if (!this->isInitialized()) {
            ESP_LOGW(TAG, "Cannot change internal power on until initialized");
            return false;
        }

        const uint32_t end = esphome::millis();
        if (this->shouldThrottle(end)) {
            ESP_LOGD(TAG, "Throttling internal turn on: %i seconds remaining", remaining);
            return false;
        }

        const char* deviceMode = deviceModeToString(this->deviceState.mode);
        this->hp->setModeSetting(deviceMode);
        this->hp->setPowerSetting("ON");
        this->internalSetCorrectedTemperature(this->getTargetTemperature());
        if (this->commit()) {
            this->lastInternalPowerUpdate = end;
            this->internalPowerOn = true;
            this->internal_power_on->publish_state(this->internalPowerOn);
            ESP_LOGW(TAG, "Performed internal turn on!");
            return true;
        } else {
            ESP_LOGW(TAG, "Failed to perform internal turn on!");
            return false;
        }
    }

    bool DeviceStateManager::internalTurnOff() {
        if (!this->isInitialized()) {
            ESP_LOGW(TAG, "Cannot change internal power off until initialized");
            return false;
        }

        const uint32_t end = esphome::millis();
        if (this->shouldThrottle(end)) {
            ESP_LOGD(TAG, "Throttling internal turn off: %i seconds remaining", remaining);
            return false;
        }

        this->hp->setPowerSetting("OFF");
        if (this->commit()) {
            this->lastInternalPowerUpdate = end;
            this->internalPowerOn = false;
            this->internal_power_on->publish_state(this->internalPowerOn);
            ESP_LOGW(TAG, "Performed internal turn off!");
            return true;
        } else {
            ESP_LOGW(TAG, "Failed to perform internal turn off!");
            return false;
        }
    }

    bool DeviceStateManager::setFanMode(FanMode mode) {
        return this->setFanMode(mode, true);
    }

    bool DeviceStateManager::setFanMode(FanMode mode, bool commit) {
        const char* newMode = fanModeToString(mode);
        if (mode == this->deviceState.fanMode) {
            const char* oldMode = fanModeToString(this->deviceState.fanMode);
            ESP_LOGW(TAG, "Did not update fan mode due to value: %s (%s)", newMode, oldMode);
            return false;
        }

        if (strcmp(newMode,"UNKNOWN") == 0) {
            ESP_LOGW(TAG, "Did not update fan mode due to invalid value: %s", newMode);
            return false;
        }

        this->hp->setFanSpeed(newMode);
        if (!commit) {
            return true;
        }

        if (this->commit()) {
            ESP_LOGW(TAG, "Performed set fan mode!");
            return true;
        } else {
            ESP_LOGW(TAG, "Failed to perform set fan mode!");
            return false;
        }
    }

    bool DeviceStateManager::setVerticalSwingMode(VerticalSwingMode mode) {
        return this->setVerticalSwingMode(mode, true);
    }

    bool DeviceStateManager::setVerticalSwingMode(VerticalSwingMode mode, bool commit) {
        const char* newMode = verticalSwingModeToString(mode);
        if (mode == this->deviceState.verticalSwingMode) {
            const char* oldMode = verticalSwingModeToString(this->deviceState.verticalSwingMode);
            ESP_LOGW(TAG, "Did not update vertical swing mode due to value: %s (%s)", newMode, oldMode);
            return false;
        }

        if (strcmp(newMode,"UNKNOWN") == 0) {
            ESP_LOGW(TAG, "Did not update vertical swing mode due to invalid value: %s", newMode);
            return false;
        }

        this->hp->setVaneSetting(newMode);
        if (!commit) {
            return true;
        }

        if (this->commit()) {
            ESP_LOGW(TAG, "Performed set vertical swing mode!");
            return true;
        } else {
            ESP_LOGW(TAG, "Failed to perform set vertical swing mode!");
            return false;
        }
    }

    bool DeviceStateManager::setHorizontalSwingMode(HorizontalSwingMode mode) {
        return this->setHorizontalSwingMode(mode, true);
    }

    bool DeviceStateManager::setHorizontalSwingMode(HorizontalSwingMode mode, bool commit) {
        const char* newMode = horizontalSwingModeToString(mode);
        if (mode == this->deviceState.horizontalSwingMode) {
            const char* oldMode = horizontalSwingModeToString(this->deviceState.horizontalSwingMode);
            ESP_LOGW(TAG, "Did not update horizontal swing mode due to value: %s (%s)", newMode, oldMode);
            return false;
        }

        if (strcmp(newMode,"UNKNOWN") == 0) {
            ESP_LOGW(TAG, "Did not update horizontal swing mode due to invalid value: %s", newMode);
            return false;
        }

        this->hp->setWideVaneSetting(newMode);
        if (!commit) {
            return true;
        }

        if (this->commit()) {
            ESP_LOGW(TAG, "Performed set horizontal swing mode!");
            return true;
        } else {
            ESP_LOGW(TAG, "Failed to perform set horizontal swing mode!");
            return false;
        }
    }

    float DeviceStateManager::getCurrentTemperature() {
        return this->deviceStatus.currentTemperature;
    }

    float DeviceStateManager::getTargetTemperature() {
        return this->targetTemperature;
    }

    float DeviceStateManager::getCorrectedTargetTemperature() {
        return this->correctedTargetTemperature;
    }

    bool DeviceStateManager::getOffsetDirection() {
        const DeviceState deviceState = this->getDeviceState();
        return this->getOffsetDirection(&deviceState);
    }

    bool DeviceStateManager::getOffsetDirection(const DeviceState* deviceState) {
        switch(deviceState->mode) {
            case devicestate::DeviceMode::DeviceMode_Heat: {
                return true;
            }
            case devicestate::DeviceMode::DeviceMode_Cool: {
                return false;
            }
            default: {
                ESP_LOGE(TAG, "Unexpected state for offset direction");
                return false;
            }
        }
    }

    float DeviceStateManager::getRoundedTemp(float value) {
        value = value * 2;
        value = round(value);
        return value / 2;
    }

    bool DeviceStateManager::internalSetCorrectedTemperature(const float value) {
        const DeviceState deviceState = this->getDeviceState();
        const bool direction = this->getOffsetDirection(&deviceState);
        const float correctionOffset = direction ? this->offsetAdjustment : -this->offsetAdjustment;
        const float setPointCorrectionOffset = value - correctionOffset;

        const float adjustedCorrectedTemperature = devicestate::clamp(setPointCorrectionOffset, this->minTemp, this->maxTemp);
        const float roundedAdjustedCorrectedTemperature = this->getRoundedTemp(adjustedCorrectedTemperature);
        if (devicestate::same_float(this->correctedTargetTemperature, adjustedCorrectedTemperature, 0.01f) &&
            devicestate::same_float(roundedAdjustedCorrectedTemperature, deviceState.targetTemperature, 0.01f)) {
            ESP_LOGW(TAG, "internalSetCorrectedTemperature: Adjustment unchanged: oldCorrection={%f} newCorrection={%f} roundedNewCorrection={%f} deviceTarget={%f} componentTarget={%f}", this->correctedTargetTemperature, adjustedCorrectedTemperature, roundedAdjustedCorrectedTemperature, deviceState.targetTemperature, this->getTargetTemperature());
            return false;
        }

        const float oldCorrectedTargetTemperature = this->correctedTargetTemperature;
        this->correctedTargetTemperature = adjustedCorrectedTemperature;
        this->hp->setTemperature(this->correctedTargetTemperature);

        ESP_LOGW(TAG, "internalSetCorrectedTemperature: Adjusted corrected temperature: oldCorrection={%f} newCorrection={%f} roundedNewCorrection={%f} deviceTarget={%f} componentTarget={%f}", oldCorrectedTargetTemperature, adjustedCorrectedTemperature, roundedAdjustedCorrectedTemperature, deviceState.targetTemperature, this->getTargetTemperature());
        this->pid_set_point_correction->publish_state(this->correctedTargetTemperature);
        return true;
    }

    void DeviceStateManager::setTargetTemperature(const float value) {
        const float adjustedTargetTemperature = devicestate::clamp(value, this->minTemp, this->maxTemp);

        const float oldTargetTemperature = this->targetTemperature;
        this->targetTemperature = adjustedTargetTemperature;
        ESP_LOGW(TAG, "setTargetTemperature: Device target temp changing from %f to %f", oldTargetTemperature, this->targetTemperature);
        this->hp->setTemperature(this->targetTemperature);
    }

    void DeviceStateManager::setRemoteTemperature(const float current) {
        this->hp->setRemoteTemperature(current);
    }

    bool DeviceStateManager::commit() {
        return this->hp->update();
    }

    void DeviceStateManager::log_heatpump_settings(heatpumpSettings currentSettings) {
        ESP_LOGI(TAG, "  power: %s", currentSettings.power);
        ESP_LOGI(TAG, "  mode: %s", currentSettings.mode);
        ESP_LOGI(TAG, "  temperature: %f", currentSettings.temperature);
        ESP_LOGI(TAG, "  fan: %s", currentSettings.fan);
        ESP_LOGI(TAG, "  vane: %s", currentSettings.vane);
        ESP_LOGI(TAG, "  wideVane: %s", currentSettings.wideVane);
        ESP_LOGI(TAG, "  connected: %s", TRUEFALSE(currentSettings.connected));
    }

    void DeviceStateManager::dump_state() {
        ESP_LOGI(TAG, "Internal State");
        ESP_LOGI(TAG, "  powerOn: %s", TRUEFALSE(this->isInternalPowerOn()));
        ESP_LOGI(TAG, "  connected: %s", TRUEFALSE(this->hp->isConnected()));
        /*
        struct DeviceState {
            bool active;
            DeviceMode mode;
            float targetTemperature;
        };
        */
        ESP_LOGI(TAG, "Device State");
        ESP_LOGI(TAG, "  active: %s", TRUEFALSE(this->deviceState.active));
        ESP_LOGI(TAG, "  mode: %s", devicestate::deviceModeToString(this->deviceState.mode));
        ESP_LOGI(TAG, "  targetTemperature: %f", this->deviceState.targetTemperature);

        /*
        struct DeviceStatus {
            bool operating;
            float currentTemperature;
            int compressorFrequency;
        };
        */
        ESP_LOGI(TAG, "Heatpump Status");
        ESP_LOGI(TAG, "  roomTemperature: %.2f", this->deviceStatus.currentTemperature);
        ESP_LOGI(TAG, "  operating: %s", TRUEFALSE(this->deviceStatus.operating));
        ESP_LOGI(TAG, "  compressorFrequency: %f", this->deviceStatus.compressorFrequency);
        ESP_LOGI(TAG, "  inputPower: %f", this->deviceStatus.inputPower);
        ESP_LOGI(TAG, "  kWh: %f", this->deviceStatus.kWh);
        ESP_LOGI(TAG, "  runtimeHours: %f", this->deviceStatus.runtimeHours);

        ESP_LOGI(TAG, "Heatpump Settings");
        heatpumpSettings currentSettings = this->hp->getSettings();
        this->log_heatpump_settings(currentSettings);
    }

}