#pragma once

#include "cn105_types.h"

using namespace devicestate;

namespace devicestate {

    void hpPacketDebug(uint8_t* packet, unsigned int length, const char* packetDirection);
    void debugSettings(const char* settingName, heatpumpSettings& settings);
    void debugStatus(const char* statusName, heatpumpStatus status);

}