#include "devicestatemanager.h"

#include "mitsubishi_cn105.h"
using namespace esphome;

#include "floats.h"
#include <cmath>
#include <cstring>
#include <string>

#define SAFE_STR(s) ((s) ? (s) : "null")

namespace devicestate {

    static const char* TAG = "DeviceStateManager"; // Logging tag

    DeviceStateManager::DeviceStateManager(
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
    ) {
        this->hpState = hpState;

        this->minTemp = minTemp;
        this->maxTemp = maxTemp;

        this->device_state_active = device_state_active;
        this->device_set_point = device_set_point;
        this->device_status_operating = device_status_operating;
        this->device_status_current_temperature = device_status_current_temperature;
        this->device_status_outside_temperature = device_status_outside_temperature;
        this->device_status_compressor_frequency = device_status_compressor_frequency;
        this->device_status_input_power = device_status_input_power;
        this->device_status_kwh = device_status_kwh;
        this->device_status_runtime_hours = device_status_runtime_hours;

        ESP_LOGCONFIG(TAG, "Initializing new HeatPump object.");
    }

    void DeviceStateManager::hpSettingsChanged() {
        if (!this->hpState->isSettingsInitialized()){
            return;
        }

        heatpumpSettings currentSettings = this->hpState->getCurrentSettings();
        ESP_LOGD(TAG, "Heatpump Settings Changed:");
        this->log_heatpump_settings(currentSettings);

        const DeviceState deviceState = devicestate::toDeviceState(currentSettings);
        if (!this->settingsInitialized) {
            this->deviceState = deviceState;
            this->settingsInitialized = true;
            return;
        }

        if (deviceStateEqual(this->deviceState, deviceState)) {
            return;
        }

        this->deviceState = deviceState;
        ESP_LOGW(TAG, "Callback hpStatusChanged");
    }

    /**
     * Report changes in the current temperature sensed by the HeatPump.
     */
    void DeviceStateManager::hpStatusChanged() {
        if (!this->hpState->isStatusInitialized()) {
            return;
        }
        
        ESP_LOGD(TAG, "Callback hpStatusChanged starting");

        heatpumpStatus currentStatus = this->hpState->getCurrentStatus();
        this->log_heatpump_status(currentStatus);
        const DeviceStatus newDeviceStatus = devicestate::toDeviceStatus(currentStatus);

        if (!this->statusInitialized) {
            this->deviceStatus = newDeviceStatus;
            this->statusInitialized = true;
            ESP_LOGW(TAG, "HeatPump status initialized.");
            ESP_LOGI(TAG, "HeatPump device status updated.");
            return;
        }

        if (devicestate::deviceStatusEqual(this->deviceStatus, newDeviceStatus)) {
            return;
        }

        this->deviceStatus = newDeviceStatus;
        ESP_LOGI(TAG, "HeatPump device status updated.");
        ESP_LOGD(TAG, "Callback hpStatusChanged completed");
    }

    bool DeviceStateManager::isInitialized() {
        return this->settingsInitialized && this->statusInitialized;
    }

    void DeviceStateManager::reset() {
        this->settingsInitialized = false;
        this->statusInitialized = false;
    }

    DeviceStatus DeviceStateManager::getDeviceStatus() {
        return this->deviceStatus;
    }

    DeviceState DeviceStateManager::getDeviceState() {
        return this->deviceState;
    }

    void DeviceStateManager::update() {
        //ESP_LOGE(TAG, "Updating...");
        //log_device_state(this->deviceState);
        this->hpSettingsChanged();
        //log_device_state(this->deviceState);

        this->hpStatusChanged();
        //ESP_LOGE(TAG, "Updated...");
    }

    void DeviceStateManager::commit() {
        this->hpState->onSettingsChanged();
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

        this->hpState->setModeSetting(deviceMode);
        this->hpState->setPowerSetting("ON");
    }

    void DeviceStateManager::turnOff() {
        this->hpState->setPowerSetting("OFF");
    }

    bool DeviceStateManager::setFanMode(FanMode mode) {
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

        this->hpState->setFanSpeed(newMode);
        this->commit();
        return true;
    }

    bool DeviceStateManager::setVerticalSwingMode(VerticalSwingMode mode) {
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

        this->hpState->setVaneSetting(newMode);
        this->commit();
        return true;
    }

    bool DeviceStateManager::setHorizontalSwingMode(HorizontalSwingMode mode) {
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

        this->hpState->setWideVaneSetting(newMode);
        this->commit();
        return true;
    }

    float DeviceStateManager::getCurrentTemperature() {
        return this->deviceStatus.currentTemperature;
    }

    float DeviceStateManager::getTargetTemperature() {
        return this->deviceState.targetTemperature;
    }

    void DeviceStateManager::setTargetTemperature(const float value) {
        const float adjustedTargetTemperature = devicestate::clamp(value, this->minTemp, this->maxTemp);
        this->hpState->setTemperature(adjustedTargetTemperature);
    }

    void DeviceStateManager::log_heatpump_settings(heatpumpSettings& currentSettings) {
        ESP_LOGD(TAG, "  power: %s", SAFE_STR(currentSettings.power));
        ESP_LOGD(TAG, "  mode: %s", SAFE_STR(currentSettings.mode));
        ESP_LOGD(TAG, "  temperature: %f", currentSettings.temperature);
        ESP_LOGD(TAG, "  fan: %s", SAFE_STR(currentSettings.fan));
        ESP_LOGD(TAG, "  vane: %s", SAFE_STR(currentSettings.vane));
        ESP_LOGD(TAG, "  wideVane: %s", SAFE_STR(currentSettings.wideVane));
        ESP_LOGD(TAG, "  connected: %s", TRUEFALSE(currentSettings.connected));
    }

    void DeviceStateManager::log_heatpump_status(heatpumpStatus& currentStatus) {
        ESP_LOGD(TAG, "  roomTemperature: %f", currentStatus.roomTemperature);
        ESP_LOGD(TAG, "  outsideAirTemperature: %f", currentStatus.outsideAirTemperature);
        ESP_LOGD(TAG, "  compressorFrequency: %f", currentStatus.compressorFrequency);
        ESP_LOGD(TAG, "  inputPower: %f", currentStatus.inputPower);
        ESP_LOGD(TAG, "  kWh: %f", currentStatus.kWh);
        ESP_LOGD(TAG, "  operating: %s", TRUEFALSE(currentStatus.operating));
    }

    void DeviceStateManager::dump_state() {
        ESP_LOGI(TAG, "Internal State");
        log_device_state(this->deviceState);

        ESP_LOGI(TAG, "Heatpump Status");
        ESP_LOGI(TAG, "  roomTemperature: %.2f", this->deviceStatus.currentTemperature);
        ESP_LOGI(TAG, "  outsideTemperature: %.2f", this->deviceStatus.outsideTemperature);
        ESP_LOGI(TAG, "  operating: %s", TRUEFALSE(this->deviceStatus.operating));
        ESP_LOGI(TAG, "  compressorFrequency: %f", this->deviceStatus.compressorFrequency);
        ESP_LOGI(TAG, "  inputPower: %f", this->deviceStatus.inputPower);
        ESP_LOGI(TAG, "  kWh: %f", this->deviceStatus.kWh);
        ESP_LOGI(TAG, "  runtimeHours: %f", this->deviceStatus.runtimeHours);

        ESP_LOGI(TAG, "Heatpump Settings");
        heatpumpSettings currentSettings = this->hpState->getCurrentSettings();
        this->log_heatpump_settings(currentSettings);
    }

    void DeviceStateManager::publish() {
        if (!this->isInitialized()) {
            ESP_LOGD(TAG, "Skipping state publication until settings and status are initialized.");
            return;
        }

        // Publish device status (with null checks)
        if (this->device_status_operating) {
            this->device_status_operating->publish_state(this->deviceStatus.operating);
        }
        if (this->device_status_current_temperature) {
            this->device_status_current_temperature->publish_state(this->deviceStatus.currentTemperature);
        }
        if (this->device_status_outside_temperature) {
            this->device_status_outside_temperature->publish_state(this->deviceStatus.outsideTemperature);
        }
        if (this->device_status_compressor_frequency) {
            this->device_status_compressor_frequency->publish_state(this->deviceStatus.compressorFrequency);
        }
        if (this->device_status_input_power) {
            this->device_status_input_power->publish_state(this->deviceStatus.inputPower);
        }
        if (this->device_status_kwh) {
            this->device_status_kwh->publish_state(this->deviceStatus.kWh);
        }
        if (this->device_status_runtime_hours) {
            this->device_status_runtime_hours->publish_state(this->deviceStatus.runtimeHours);
        }

        // Publish device state (with null checks)
        if (this->device_state_active) {
            this->device_state_active->publish_state(this->deviceState.active);
        }
        if (this->device_set_point) {
            this->device_set_point->publish_state(this->deviceState.targetTemperature);
        }
    }

}
