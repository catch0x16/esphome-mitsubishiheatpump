#pragma once

#include "cn105_types.h"

using namespace devicestate;

namespace devicestate {

    class CN105State {
        private:
            heatpumpSettings currentSettings{};
            wantedHeatpumpSettings wantedSettings{};

            heatpumpRunStates currentRunStates{};
            wantedHeatpumpRunStates wantedRunStates{};

        public:
            CN105State();

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
