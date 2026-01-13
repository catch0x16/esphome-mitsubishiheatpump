#pragma once

#include "cn105_types.h"
#include <functional>

#include "io_device.h"

using namespace devicestate;

namespace devicestate {

    class CN105Connection {
        public:
            using TimeoutCallback = std::function<void(const std::string&, uint32_t, std::function<void()>)>;
            using CommandCallback = std::function<void(uint8_t command)>;

            CN105Connection(IIODevice* io_device, TimeoutCallback timeoutCallback, int update_interval_);

            bool isConnected();
            void writePacket(uint8_t* packet, int length, bool checkIsActive = true);
            bool processInput(CommandCallback commandCallback);

        private:
            IIODevice* io_device_;
            int update_interval_;

            TimeoutCallback timeoutCallback_;

            uint8_t storedInputData[MAX_DATA_BYTES]; // multi-byte data
            uint8_t* data;

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
            void checkHeader(uint8_t inputData);
            void setupUART();
            void disconnectUART();
            void reconnectUART();
            //void force_low_level_uart_reinit();
            void sendFirstConnectionPacket();

            bool isHeatpumpConnectionActive();

            void try_write_pending_packet();

            void updateSuccess();
            void processCommand(CommandCallback commandCallback);
            void processDataPacket(CommandCallback commandCallback);
            void parse(uint8_t inputData, CommandCallback commandCallback);
    };

}