#pragma once

#include "io_device.h"

#include "cn105_types.h"
#include "cn105_state.h"
#include "cn105_connection.h"
#include "cn105_protocol.h"

#include "cycle_management.h"
#include "request_scheduler.h"

using namespace devicestate;

namespace devicestate {

    class CN105ControlFlow {
        private:
            CN105Connection* connection_;
            CN105State* hpState_;
            RequestScheduler scheduler_;
            CN105Protocol hpProtocol;

            bool processInput(CN105State& hpState);
            void buildAndSendInfoPacket(uint8_t code);
            void buildAndSendRequestsInfoPackets(cycleManagement& loopCycle);
            void buildAndSendRequestPacket(int packetType);
            

        public:
            CN105ControlFlow(
                CN105Connection* connection,
                CN105State* hpState,
                RequestScheduler::TimeoutCallback timeoutCallback,
                RequestScheduler::TerminateCallback terminateCallback
            );

            void loop(cycleManagement& loopCycle);
    };

}
