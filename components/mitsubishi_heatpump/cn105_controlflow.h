#pragma once

#include "io_device.h"

#include "cn105_types.h"
#include "cn105_state.h"

#include "cycle_management.h"
#include "request_scheduler.h"

using namespace devicestate;

namespace devicestate {

    class CN105ControlFlow {
        private:
            IIODevice* io_device_;
            RequestScheduler scheduler_;
            bool foundStart = false;
            int bytesRead = 0;
            int dataLength = 0;
            uint8_t command = 0;

            uint8_t storedInputData[MAX_DATA_BYTES]; // multi-byte data

        public:
            CN105ControlFlow(
                IIODevice* io_device,
                RequestScheduler::TimeoutCallback timeoutCallback,
                RequestScheduler::TerminateCallback terminateCallback
            );

            void buildAndSendRequestPacket(int packetType);
            void buildAndSendInfoPacket(uint8_t code);

            void checkHeader(uint8_t inputData);
            void initBytePointer();
            void parse(uint8_t inputData);
            bool processInput();
    };

}
