#include "cn105_state.h"
#include "cn105_protocol.h"

using namespace devicestate;

namespace devicestate {

    CN105State::CN105State() {
        this->wantedSettings.resetSettings();
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
    }

    void CN105State::setPowerSetting(const char* setting) {
        int index = lookupByteMapIndex(POWER_MAP, 2, setting);
        if (index > -1) {
            wantedSettings.power = POWER_MAP[index];
        } else {
            wantedSettings.power = POWER_MAP[0];
        }
    }

    void CN105State::setFanSpeed(const char* setting) {
        int index = lookupByteMapIndex(FAN_MAP, 6, setting);
        if (index > -1) {
            wantedSettings.fan = FAN_MAP[index];
        } else {
            wantedSettings.fan = FAN_MAP[0];
        }
    }

    void CN105State::setVaneSetting(const char* setting) {
        int index = lookupByteMapIndex(VANE_MAP, 7, setting);
        if (index > -1) {
            wantedSettings.vane = VANE_MAP[index];
        } else {
            wantedSettings.vane = VANE_MAP[0];
        }
    }

    void CN105State::setWideVaneSetting(const char* setting) {
        int index = lookupByteMapIndex(WIDEVANE_MAP, 8, setting);
        if (index > -1) {
            wantedSettings.wideVane = WIDEVANE_MAP[index];
        } else {
            wantedSettings.wideVane = WIDEVANE_MAP[0];
        }
    }

    void CN105State::setAirflowControlSetting(const char* setting) {
        int index = lookupByteMapIndex(AIRFLOW_CONTROL_MAP, 3, setting);
        if (index > -1) {
            wantedRunStates.airflow_control = AIRFLOW_CONTROL_MAP[index];
        } else {
            wantedRunStates.airflow_control = AIRFLOW_CONTROL_MAP[0];
        }
    }

}
