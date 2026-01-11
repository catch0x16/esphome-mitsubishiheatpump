#pragma once

#include "Globals.h"

#include "protocol.h"
#include "io_device.h"

namespace devicestate {

    class CN105Protocol : public IProtocol {
    private:
        devicestate::IIODevice* io_device_;

        unsigned long lastResponseMs;
        unsigned long lastSend;
        unsigned long lastConnectRqTimeMs;
        unsigned long lastReconnectTimeMs;

        bool isUARTConnected_ = false;
        bool isHeatpumpConnected_ = false;

        void setupUART();

    public:
        CN105Protocol(devicestate::IIODevice* io_device);

        bool connect() override;
        bool isConnected() override;
    };

}
