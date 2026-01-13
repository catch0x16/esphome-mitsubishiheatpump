#pragma once

#include "cn105_types.h"
#include "cn105_utils.h"
#include "heatpumpFunctions.h"

using namespace devicestate;

namespace devicestate {

    class CN105State {
        private:
            heatpumpSettings currentSettings{};
            wantedHeatpumpSettings wantedSettings{};

            // initialise to all off, then it will update shortly after connect;
            heatpumpStatus currentStatus{ 0, 0, false, {TIMER_MODE_MAP[0], 0, 0, 0, 0}, 0, 0, 0, 0 };

            heatpumpFunctions functions;

            heatpumpRunStates currentRunStates{};
            wantedHeatpumpRunStates wantedRunStates{};

            bool wideVaneAdj;
            bool tempMode;

        public:
            CN105State();

            CN105State(
                heatpumpSettings currentSettings, wantedHeatpumpSettings wantedSettings,
                heatpumpRunStates currentRunStates, wantedHeatpumpRunStates wantedRunStates,
                bool wideVaneAdj, bool tempMode);

            heatpumpSettings getCurrentSettings();
            void setCurrentSettings(heatpumpSettings currentSettings);
            wantedHeatpumpSettings getWantedSettings();

            heatpumpStatus getCurrentStatus();

            heatpumpFunctions getFunctions();

            heatpumpRunStates getCurrentRunStates();
            wantedHeatpumpRunStates getWantedRunStates();

            bool isUpdated();

            void setWideVaneAdj(bool value);
            bool shouldWideVaneAdj();

            bool getTempMode();
            void setTempMode(bool value);

            const char* getModeSetting();
            const char* getPowerSetting();
            bool getPowerSettingBool();

            const char* getVaneSetting();
            const char* getWideVaneSetting();
            const char* getAirflowControlSetting();
            const char* getFanSpeedSetting();

            float getTemperatureSetting();
            bool getAirPurifierRunState();
            bool getNightModeRunState();
            bool getCirculatorRunState();

            void setModeSetting(const char* setting);
            void setPowerSetting(bool setting);
            void setPowerSetting(const char* setting);
            void setVaneSetting(const char* setting);
            void setWideVaneSetting(const char* setting);
            void setAirflowControlSetting(const char* setting);
            void setFanSpeed(const char* setting);
            void setTemperatureSetting(float setting);

            void setRoomTemperature(float value);
            void setRuntimeHours(float value);

            heatpumpTimers getTimers();
            void setTimers(heatpumpTimers value);

            void setOperating(bool value);
            void setCompressorFrequency(float value);
            void setInputPower(float value);
            void setKWh(float value);
    };

}
