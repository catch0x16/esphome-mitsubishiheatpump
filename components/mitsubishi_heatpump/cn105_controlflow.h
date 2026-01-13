#pragma once

#include "cn105_types.h"
#include "cn105_state.h"

#include "request_scheduler.h"

using namespace devicestate;

namespace devicestate {

    class CN105ControlFlow {
        private:
            RequestScheduler scheduler_;

        public:
            CN105ControlFlow(
                RequestScheduler::TimeoutCallback timeoutCallback,
                RequestScheduler::TerminateCallback terminateCallback
            );

            void buildAndSendRequestPacket(int packetType);
            void buildAndSendInfoPacket(uint8_t code);

            void loop(CN105State& hpState);
    };

}
