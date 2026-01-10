#pragma once

#include "cn105_types.h"
#include <functional>

namespace devicestate {

    #define ON_CONNECT_CALLBACK_SIGNATURE std::function<void()> onConnectCallback
    #define SETTINGS_CHANGED_CALLBACK_SIGNATURE std::function<void()> settingsChangedCallback
    #define STATUS_CHANGED_CALLBACK_SIGNATURE std::function<void(heatpumpStatus newStatus)> statusChangedCallback
    #define PACKET_CALLBACK_SIGNATURE std::function<void(byte* packet, unsigned int length, char* packetDirection)> packetCallback

    class IProtocol {
    public:
        virtual bool connect() = 0;
        virtual bool isConnected() = 0;

        // settings
        virtual heatpumpSettings getSettings() = 0;

        // status
        virtual heatpumpStatus getStatus() = 0;

        // Write
        virtual bool update() = 0;

        // Read / flush
        virtual void sync(byte packetType = PACKET_TYPE_DEFAULT) = 0;

        virtual float getTemperature() = 0;
        virtual void setTemperature(float setting) = 0;
        virtual void setRemoteTemperature(float setting) = 0;

        virtual void setSettingsChangedCallback(SETTINGS_CHANGED_CALLBACK_SIGNATURE) = 0;
        virtual void setStatusChangedCallback(STATUS_CHANGED_CALLBACK_SIGNATURE) = 0;

        virtual void setPowerSetting(const char* setting) = 0;
        virtual void setModeSetting(const char* setting) = 0;
        virtual void setFanSpeed(const char* setting) = 0;
        virtual void setVaneSetting(const char* setting) = 0;
        virtual void setWideVaneSetting(const char* setting) = 0;

        virtual ~IProtocol() = default;    // Virtual destructor for safety
    };

}