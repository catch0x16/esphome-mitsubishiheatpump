#pragma once

#include "cn105_types.h"
#include <functional>

#include "io_device.h"

using namespace devicestate;

namespace devicestate {

    class CN105Connection {
        public:
            using TimeoutCallback = std::function<void(const std::string&, uint32_t, std::function<void()>)>;

            CN105Connection(IIODevice* io_device, TimeoutCallback timeoutCallback);

            bool isConnected();
            void writePacket(uint8_t* packet, int length, bool checkIsActive = true);
            bool readNextPacket(uint8_t* packet);

        private:
            IIODevice* io_device_;

            TimeoutCallback timeoutCallback_;

            bool foundStart = false;
            int bytesRead = 0;
            int dataLength = 0;
            uint8_t command = 0;

            bool isHeatpumpConnected_ = false;
            bool isUARTConnected_ = false;
            bool firstRun = false;

            bool conn_bootstrap_started_ = false;
            bool conn_timeout_armed_ = false;

            uint8_t pending_packet_[PACKET_LEN] = {};
            int pending_packet_len_ = 0;
            bool pending_check_is_active_ = true;
            bool has_pending_packet_ = false;

            unsigned long lastSend;
            unsigned long lastConnectRqTimeMs;
            unsigned long lastReconnectTimeMs;
            unsigned long lastResponseMs;

            void initBytePointer();
            bool checkSum();
            void setupUART();
            //void force_low_level_uart_reinit();
            void sendFirstConnectionPacket();

            void try_write_pending_packet();

            int parse(uint8_t inputData, uint8_t* storedInputData);
    };

}