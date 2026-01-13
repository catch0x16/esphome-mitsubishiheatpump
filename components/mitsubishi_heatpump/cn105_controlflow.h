#pragma once

#include "io_device.h"

#include "cn105_types.h"
#include "cn105_state.h"
#include "cn105_connection.h"

#include "cycle_management.h"
#include "request_scheduler.h"

using namespace devicestate;

namespace devicestate {

    class CN105ControlFlow {
        private:
            CN105Connection* connection_;
            RequestScheduler scheduler_;

            bool processInput(CN105State& hpState);

        public:
            CN105ControlFlow(
                CN105Connection* connection,
                RequestScheduler::TimeoutCallback timeoutCallback,
                RequestScheduler::TerminateCallback terminateCallback
            );

            void buildAndSendRequestPacket(int packetType);
            void buildAndSendInfoPacket(uint8_t code);

            void loop(cycleManagement& loopCycle, CN105State& hpState);
    };

}
