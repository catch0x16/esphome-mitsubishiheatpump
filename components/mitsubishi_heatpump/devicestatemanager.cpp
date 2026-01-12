#include "devicestatemanager.h"

#include "espmhp.h"
using namespace esphome;

#include "floats.h"
#include <cmath>
#include <cstring>
#include <string>

#include "io_device.h"
#include "uart_io_device.h"

namespace devicestate {

    static const char* TAG = "DeviceStateManager"; // Logging tag

    DeviceStateManager::DeviceStateManager(
      ConnectionMetadata connectionMetadata,
      RequestScheduler scheduler,
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
    ) : scheduler{scheduler} {
        this->connectionMetadata = connectionMetadata;

        this->minTemp = minTemp;
        this->maxTemp = maxTemp;

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

        IIODevice* io_device = new UARTIODevice(
            connectionMetadata.hardwareSerial
        );
        this->hp = new HeatPump(io_device, scheduler);

        #ifdef USE_CALLBACKS
            std::function<void()> settingsChanged = [this]() {
                this->hpSettingsChanged();
            };

            hp->setSettingsChangedCallback(settingsChanged);

            std::function<void(heatpumpStatus)> statusChanged = [this](heatpumpStatus currentStatus) {
                this->hpStatusChanged(currentStatus);
            };

            hp->setStatusChangedCallback(statusChanged);

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
            ESP_LOGW(TAG, "Waiting for HeatPump to read the settings the first time (currentSettings.power == NULL).");
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
        ESP_LOGI(TAG, "HeatPump device state updated.");

        ESP_LOGW(TAG, "Callback hpStatusChanged");
    }

    /**
     * Report changes in the current temperature sensed by the HeatPump.
     */
    void DeviceStateManager::hpStatusChanged(heatpumpStatus currentStatus) {
        ESP_LOGW(TAG, "Callback hpStatusChanged starting");
        if (!this->statusInitialized) {
            this->statusInitialized = true;
            ESP_LOGW(TAG, "HeatPump status initialized.");
        }

        this->log_heatpump_status(currentStatus);
        const DeviceStatus newDeviceStatus = devicestate::toDeviceStatus(&currentStatus);
        if (!devicestate::deviceStatusEqual(this->deviceStatus, newDeviceStatus)) {
            this->deviceStatus = newDeviceStatus;
            ESP_LOGI(TAG, "HeatPump device status updated.");
        }

        ESP_LOGW(TAG, "Callback hpStatusChanged completed");
    }

    void DeviceStateManager::log_packet(byte* packet, unsigned int length, char* packetDirection) {
        //String packetHex;
        std::string packetHex;
        packetHex.reserve(length * 3 + 1); // "FF " par octet
        char textBuf[15];

        for (int i = 0; i < length; i++) {
            memset(textBuf, 0, 15);
            sprintf(textBuf, "%02X ", packet[i]);
            packetHex += textBuf;
        }
    }

    bool DeviceStateManager::isInitialized() {
        return this->settingsInitialized && this->statusInitialized;
    }

    bool DeviceStateManager::connect() {
        if (this->hp->connect()) {
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
    }

    void DeviceStateManager::turnOff() {
        this->hp->setPowerSetting("OFF");
        this->internalPowerOn = false;
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
            ESP_LOGD(TAG, "Throttling internal turn on");
            return false;
        }

        const char* deviceMode = deviceModeToString(this->deviceState.mode);
        this->hp->setModeSetting(deviceMode);
        this->hp->setPowerSetting("ON");
        this->internalSetCorrectedTemperature(this->getTargetTemperature());
        if (this->commit()) {
            this->lastInternalPowerUpdate = end;
            this->internalPowerOn = true;
            ESP_LOGW(TAG, "Performed internal turn on!");
            return true;
        } else {
            ESP_LOGW(TAG, "Failed to perform internal turn on!");
            return false;
        }
    }

    bool DeviceStateManager::internalTurnOff() {
        ESP_LOGW(TAG, "Turning off");
        if (!this->isInitialized()) {
            ESP_LOGW(TAG, "Cannot change internal power off until initialized");
            return false;
        }

        ESP_LOGW(TAG, "Check throttle");
        const uint32_t end = esphome::millis();
        if (this->shouldThrottle(end)) {
            ESP_LOGW(TAG, "Throttling internal turn off");
            return false;
        }

        ESP_LOGW(TAG, "Set power OFF");
        this->hp->setPowerSetting("OFF");
        ESP_LOGW(TAG, "Commit change");
        if (this->commit()) {
            ESP_LOGW(TAG, "Change committed.");
            this->lastInternalPowerUpdate = end;
            this->internalPowerOn = false;
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

    bool DeviceStateManager::internalSetCorrectedTemperature(const float setPointCorrection) {
        const DeviceState deviceState = this->getDeviceState();
        const bool direction = this->getOffsetDirection(&deviceState);

        const float adjustedCorrectedTemperature = devicestate::clamp(setPointCorrection, this->minTemp, this->maxTemp);
        const float roundedAdjustedCorrectedTemperature = this->getRoundedTemp(adjustedCorrectedTemperature);
        if (devicestate::same_float(this->correctedTargetTemperature, adjustedCorrectedTemperature, 0.01f) &&
            devicestate::same_float(roundedAdjustedCorrectedTemperature, deviceState.targetTemperature, 0.01f)) {
            ESP_LOGD(TAG, "internalSetCorrectedTemperature: Adjustment unchanged: oldCorrection={%f} newCorrection={%f} roundedNewCorrection={%f} deviceTarget={%f} componentTarget={%f}", this->correctedTargetTemperature, adjustedCorrectedTemperature, roundedAdjustedCorrectedTemperature, deviceState.targetTemperature, this->getTargetTemperature());
            return false;
        }

        const float oldCorrectedTargetTemperature = this->correctedTargetTemperature;
        this->correctedTargetTemperature = adjustedCorrectedTemperature;
        this->hp->setTemperature(this->correctedTargetTemperature);

        ESP_LOGW(TAG, "internalSetCorrectedTemperature: Adjusted corrected temperature: oldCorrection={%f} newCorrection={%f} roundedNewCorrection={%f} deviceTarget={%f} componentTarget={%f}", oldCorrectedTargetTemperature, adjustedCorrectedTemperature, roundedAdjustedCorrectedTemperature, deviceState.targetTemperature, this->getTargetTemperature());
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
        const float normalizedCurrent = this->getOffsetDirection()
            ? std::ceil(current * 2.0) / 2.0
            : std::floor(current * 2.0) / 2.0;

        this->hp->setRemoteTemperature(normalizedCurrent);
    }

    bool DeviceStateManager::commit() {
        ESP_LOGW(TAG, "DeviceStateManager attempting commit");
        const bool result = this->hp->update();
        ESP_LOGW(TAG, "DeviceStateManager completed commit: %s", TRUEFALSE(result));
        return result;
    }

    void DeviceStateManager::log_heatpump_settings(heatpumpSettings currentSettings) {
        ESP_LOGD(TAG, "  power: %s", currentSettings.power);
        ESP_LOGD(TAG, "  mode: %s", currentSettings.mode);
        ESP_LOGD(TAG, "  temperature: %f", currentSettings.temperature);
        ESP_LOGD(TAG, "  fan: %s", currentSettings.fan);
        ESP_LOGD(TAG, "  vane: %s", currentSettings.vane);
        ESP_LOGD(TAG, "  wideVane: %s", currentSettings.wideVane);
        ESP_LOGD(TAG, "  connected: %s", TRUEFALSE(currentSettings.connected));
    }

    void DeviceStateManager::log_heatpump_status(heatpumpStatus currentStatus) {
        ESP_LOGD(TAG, "  roomTemperature: %f", currentStatus.roomTemperature);
        ESP_LOGD(TAG, "  compressorFrequency: %f", currentStatus.compressorFrequency);
        ESP_LOGD(TAG, "  inputPower: %f", currentStatus.inputPower);
        ESP_LOGD(TAG, "  kWh: %f", currentStatus.kWh);
        ESP_LOGD(TAG, "  operating: %s", TRUEFALSE(currentStatus.operating));
    }

    void DeviceStateManager::dump_state() {
        ESP_LOGI(TAG, "Internal State");
        ESP_LOGI(TAG, "  powerOn: %s", TRUEFALSE(this->isInternalPowerOn()));
        ESP_LOGI(TAG, "  connected: %s", TRUEFALSE(this->hp->isConnected()));

        ESP_LOGI(TAG, "Device State");
        ESP_LOGI(TAG, "  active: %s", TRUEFALSE(this->deviceState.active));
        ESP_LOGI(TAG, "  mode: %s", devicestate::deviceModeToString(this->deviceState.mode));
        ESP_LOGI(TAG, "  targetTemperature: %f", this->deviceState.targetTemperature);

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

    void DeviceStateManager::publish() {
        // Publish device status
        this->device_status_operating->publish_state(this->deviceStatus.operating);
        this->device_status_current_temperature->publish_state(this->deviceStatus.currentTemperature);
        this->device_status_compressor_frequency->publish_state(this->deviceStatus.compressorFrequency);
        this->device_status_input_power->publish_state(this->deviceStatus.inputPower);
        this->device_status_kwh->publish_state(this->deviceStatus.kWh);
        this->device_status_runtime_hours->publish_state(this->deviceStatus.runtimeHours);

        // Public device state
        this->device_state_connected->publish_state(this->hp->isConnected());
        this->internal_power_on->publish_state(this->internalPowerOn);
        this->device_state_active->publish_state(this->deviceState.active);
        this->device_set_point->publish_state(this->deviceState.targetTemperature);
        this->pid_set_point_correction->publish_state(this->correctedTargetTemperature);
    }

}