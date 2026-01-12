#pragma once

#include "cn105_types.h"
#include "cn105_utils.h"

using namespace devicestate;

namespace devicestate {

    class CN105State {
        private:
            heatpumpSettings currentSettings{};
            wantedHeatpumpSettings wantedSettings{};

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

            wantedHeatpumpSettings getWantedSettings();
            wantedHeatpumpRunStates getWantedRunStates();

            bool isUpdated();

            void setWideVaneAdj(bool value);
            bool shouldWideVaneAdj();

            bool getTempMode();

            const char* getModeSetting();
            const char* getPowerSetting();
            const char* getVaneSetting();
            const char* getWideVaneSetting();
            const char* getAirflowControlSetting();
            const char* getFanSpeedSetting();

            float getTemperatureSetting();
            bool getAirPurifierRunState();
            bool getNightModeRunState();
            bool getCirculatorRunState();

            void setModeSetting(const char* setting);
            void setPowerSetting(const char* setting);
            void setVaneSetting(const char* setting);
            void setWideVaneSetting(const char* setting);
            void setAirflowControlSetting(const char* setting);
            void setFanSpeed(const char* setting);
    };

}
