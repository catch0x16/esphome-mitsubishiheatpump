#include "Globals.h"
#include "cn105_state.h"
#include "cn105_protocol.h"
#include "cn105_logging.h"

using namespace devicestate;

namespace devicestate {

    static const char* TAG = "CN105State"; // Logging tag

    CN105State::CN105State() {
        this->functions = heatpumpFunctions();

        this->wantedSettings.resetSettings();
        this->wantedRunStates.resetSettings();

        this->wideVaneAdj = false;
        this->tempMode = false;
    }

    bool CN105State::isUpdated() {
        return wantedSettings.hasChanged || wantedRunStates.hasChanged;
    }

    void CN105State::setWideVaneAdj(bool value) {
        this->wideVaneAdj = value;
    }

    bool CN105State::shouldWideVaneAdj() {
        return wideVaneAdj;
    }

    bool CN105State::getTempMode() {
        return tempMode;
    }

    void CN105State::setTempMode(bool value) {
        tempMode = value;
    }

    heatpumpSettings& CN105State::getCurrentSettings() {
        return currentSettings;
    }

    void CN105State::setCurrentSettings(heatpumpSettings currentSettings) {
        this->currentSettings = currentSettings;
    }

    void CN105State::resetCurrentSettings() {
        this->currentSettings.resetSettings();
    }

    heatpumpStatus& CN105State::getCurrentStatus() {
        return currentStatus;
    }

    heatpumpFunctions& CN105State::getFunctions() {
        return functions;
    }

    wantedHeatpumpSettings& CN105State::getWantedSettings() {
        return wantedSettings;
    }

    void CN105State::resetWantedSettings() {
        this->wantedSettings.resetSettings();
    }

    heatpumpRunStates& CN105State::getCurrentRunStates() {
        return currentRunStates;
    }

    wantedHeatpumpRunStates& CN105State::getWantedRunStates() {
        return wantedRunStates;
    }

    void CN105State::resetCurrentRunStates() {
        this->wantedRunStates.resetSettings();
    }

    const char* CN105State::getModeSetting() {
        if (this->wantedSettings.mode) {
            return this->wantedSettings.mode;
        } else {
            return this->currentSettings.mode;
        }
    }

    const char* CN105State::getPowerSetting() {
        if (this->wantedSettings.power) {
            return this->wantedSettings.power;
        } else {
            return this->currentSettings.power;
        }
    }

     bool CN105State::getPowerSettingBool() {
        return this->getPowerSetting() == POWER_MAP[1];
    }

    const char* CN105State::getVaneSetting() {
        if (this->wantedSettings.vane) {
            return this->wantedSettings.vane;
        } else {
            return this->currentSettings.vane;
        }
    }

    const char* CN105State::getWideVaneSetting() {
        if (this->wantedSettings.wideVane) {
            if (strcmp(this->wantedSettings.wideVane, lookupByteMapValue(WIDEVANE_MAP, WIDEVANE, 8, 0x80 & 0x0F)) == 0 && !this->currentSettings.iSee) {
                this->wantedSettings.wideVane = this->currentSettings.wideVane;
            }
            return this->wantedSettings.wideVane;
        } else {
            return this->currentSettings.wideVane;
        }
    }

    const char* CN105State::getFanSpeedSetting() {
        if (this->wantedSettings.fan) {
            return this->wantedSettings.fan;
        } else {
            return this->currentSettings.fan;
        }
    }

    float CN105State::getTemperatureSetting() {
        if (this->wantedSettings.temperature != -1.0) {
            return this->wantedSettings.temperature;
        } else {
            return this->currentSettings.temperature;
        }
    }

    const char* CN105State::getAirflowControlSetting() {
        if (this->wantedRunStates.airflow_control) {
            return this->wantedRunStates.airflow_control;
        } else {
            return this->currentRunStates.airflow_control;
        }
    }

    bool CN105State::getAirPurifierRunState() {
        if (this->wantedRunStates.air_purifier != this->currentRunStates.air_purifier) {
            return this->wantedRunStates.air_purifier;
        } else {
            return this->currentRunStates.air_purifier;
        }
    }

    bool CN105State::getNightModeRunState() {
        if (this->wantedRunStates.night_mode != this->currentRunStates.night_mode) {
            return this->wantedRunStates.night_mode;
        } else {
            return this->currentRunStates.night_mode;
        }
    }

    bool CN105State::getCirculatorRunState() {
        if (this->wantedRunStates.circulator != this->currentRunStates.circulator) {
            return this->wantedRunStates.circulator;
        } else {
            return this->currentRunStates.circulator;
        }
    }

    void CN105State::setModeSetting(const char* setting) {
        int index = lookupByteMapIndex(MODE_MAP, 5, setting);
        if (index > -1) {
            wantedSettings.mode = MODE_MAP[index];
        } else {
            wantedSettings.mode = MODE_MAP[0];
        }
        this->onSettingsChanged();
    }

    void CN105State::setPowerSetting(bool setting) {
        wantedSettings.power = lookupByteMapIndex(POWER_MAP, 2, POWER_MAP[setting ? 1 : 0]) > -1 ? POWER_MAP[setting ? 1 : 0] : POWER_MAP[0];
        this->onSettingsChanged();
    }

    void CN105State::setPowerSetting(const char* setting) {
        int index = lookupByteMapIndex(POWER_MAP, 2, setting);
        if (index > -1) {
            wantedSettings.power = POWER_MAP[index];
        } else {
            wantedSettings.power = POWER_MAP[0];
        }
        this->onSettingsChanged();
    }

    void CN105State::setFanSpeed(const char* setting) {
        int index = lookupByteMapIndex(FAN_MAP, 6, setting);
        if (index > -1) {
            wantedSettings.fan = FAN_MAP[index];
        } else {
            wantedSettings.fan = FAN_MAP[0];
        }
        this->onSettingsChanged();
    }

    void CN105State::setVaneSetting(const char* setting) {
        int index = lookupByteMapIndex(VANE_MAP, 7, setting);
        if (index > -1) {
            wantedSettings.vane = VANE_MAP[index];
        } else {
            wantedSettings.vane = VANE_MAP[0];
        }
        this->onSettingsChanged();
    }

    void CN105State::setWideVaneSetting(const char* setting) {
        int index = lookupByteMapIndex(WIDEVANE_MAP, 8, setting);
        if (index > -1) {
            wantedSettings.wideVane = WIDEVANE_MAP[index];
        } else {
            wantedSettings.wideVane = WIDEVANE_MAP[0];
        }
        this->onSettingsChanged();
    }

    void CN105State::setAirflowControlSetting(const char* setting) {
        int index = lookupByteMapIndex(AIRFLOW_CONTROL_MAP, 3, setting);
        if (index > -1) {
            wantedRunStates.airflow_control = AIRFLOW_CONTROL_MAP[index];
        } else {
            wantedRunStates.airflow_control = AIRFLOW_CONTROL_MAP[0];
        }
    }

    void CN105State::setTemperature(float setting) {
        float temperature;
        if(!this->getTempMode()){
            temperature = lookupByteMapIndex(TEMP_MAP, 16, (int)(setting + 0.5)) > -1 ? setting : TEMP_MAP[0];
        }
        else {
            setting = setting * 2;
            setting = round(setting);
            setting = setting / 2;
            temperature = setting < 10 ? 10 : (setting > 31 ? 31 : setting);
        }
        wantedSettings.temperature = temperature;

        this->onSettingsChanged();
    }

    void CN105State::setRoomTemperature(float value) {
        currentStatus.roomTemperature = value;
    }

    void CN105State::setRuntimeHours(float value) {
        currentStatus.runtimeHours = value;
    }

    heatpumpTimers CN105State::getTimers() {
        return currentStatus.timers;
    }

    void CN105State::setTimers(heatpumpTimers value) {
        currentStatus.timers = value;
    }

    void CN105State::setOperating(bool value) {
        currentStatus.operating = value;
    }

    void CN105State::setCompressorFrequency(float value) {
        currentStatus.compressorFrequency = value;
    }

    void CN105State::setInputPower(float value) {
        currentStatus.inputPower = value;
    }

    void CN105State::setKWh(float value) {
        currentStatus.kWh = value;
    }

    void CN105State::onSettingsChanged() {
        ESP_LOGW(TAG, "Setting onSettingsChanged triggered");
        wantedSettings.hasChanged = true;
        wantedSettings.hasBeenSent = false;
        wantedSettings.lastChange = CUSTOM_MILLIS;
    }

    void CN105State::updateCurrentSettings(heatpumpSettings& settings) {
        if (settings != this->currentSettings) {
            ESP_LOGI(LOG_SETTINGS_TAG, "Settings changed, updating HA states");
            debugSettings("current", this->currentSettings);
            debugSettings("received", settings);
            debugSettings("wanted", this->wantedSettings);
            //this->debugClimate("climate");
            //this->publishStateToHA(settings);
        }

        if ((this->wantedSettings.mode == nullptr) && (this->wantedSettings.power == nullptr)) {        // to prevent overwriting a user demand
            if (this->hasChanged(currentSettings.power, settings.power, "power") ||
                   this->hasChanged(currentSettings.mode, settings.mode, "mode")) {           // mode or power change ?
                ESP_LOGI(TAG, "power or mode changed");
                currentSettings.power = settings.power;
                currentSettings.mode = settings.mode;
            }
        }

        //this->updateAction();       // update action info on HA climate component

        if (this->wantedSettings.fan == nullptr) {  // to prevent overwriting a user demand
            if (this->hasChanged(currentSettings.fan, settings.fan, "fan")) { // fan setting change ?
                ESP_LOGI(TAG, "fan setting changed");
                currentSettings.fan = settings.fan;
            }
        }

        if (this->wantedSettings.vane == nullptr) { // to prevent overwriting a user demand
            if (this->hasChanged(currentSettings.vane, settings.vane, "vane")) {    // widevane setting change ?
                ESP_LOGI(TAG, "vane setting changed");
                currentSettings.vane = settings.vane;
            }
        }

        if (this->wantedSettings.wideVane == nullptr) { // to prevent overwriting a user demand
            if (this->hasChanged(currentSettings.wideVane, settings.wideVane, "wideVane")) {    // widevane setting change ?
                ESP_LOGI(TAG, "vane setting changed");
                currentSettings.wideVane = settings.wideVane;
            }
        }

        // HA Temp
        // Ignorer temporairement une consigne entrante si une consigne utilisateur est en cours
        bool hasPendingUserTemp = (this->wantedSettings.temperature != -1.0f) && (this->wantedSettings.hasChanged) && (!this->wantedSettings.hasBeenSent);
        //uint32_t graceWindowMs = this->get_update_interval() + DEFER_SCHEDULE_UPDATE_LOOP_DELAY;
        //bool graceAfterSend = (this->wantedSettings.hasBeenSent) && ((CUSTOM_MILLIS - this->wantedSettings.lastChange) < graceWindowMs);
        if (!hasPendingUserTemp) {// && !graceAfterSend
            if (this->wantedSettings.temperature == -1) { // to prevent overwriting a user demand
                //this->updateTargetTemperaturesFromSettings(settings.temperature);
                this->currentSettings.temperature = settings.temperature;
            }
        } else {
            ESP_LOGD(TAG, "Ignoring incoming setpoint due to pending user change or grace window");
        }

        ESP_LOGW(TAG, "Wrapping-up updateCurrentSettings");

        this->currentSettings.iSee = settings.iSee;

        this->currentSettings.connected = true;
        ESP_LOGW(TAG, "Completed running updateCurrentSettings");

        // publish to HA
        //this->publish_state();
    }

    void CN105State::updateCurrentStatus(heatpumpStatus& status) {
        if (status != this->currentStatus) {
            debugStatus("received", status);
            debugStatus("current", currentStatus);

            this->currentStatus.operating = status.operating;
            this->currentStatus.compressorFrequency = status.compressorFrequency;
            this->currentStatus.inputPower = status.inputPower;
            this->currentStatus.kWh = status.kWh;
            this->currentStatus.runtimeHours = status.runtimeHours;
            this->currentStatus.roomTemperature = status.roomTemperature;
            this->currentStatus.outsideAirTemperature = status.outsideAirTemperature;
            //this->setCurrentTemperature(this->currentStatus.roomTemperature);

            //this->updateAction();       // update action info on HA climate component
            //this->publish_state();
        } // else no change
    }


    bool CN105State::hasChanged(const char* before, const char* now, const char* field, bool checkNotNull) {
        if (now == NULL) {
            if (checkNotNull) {
                ESP_LOGE(TAG, "CAUTION: expected value in hasChanged() function for %s, got NULL", field);
            } else {
                ESP_LOGD(TAG, "No value in hasChanged() function for %s", field);
            }
            return false;
        }
        return ((before == NULL) || (strcmp(before, now) != 0));
    }
}
