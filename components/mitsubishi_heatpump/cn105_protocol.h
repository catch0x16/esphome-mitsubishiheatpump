#pragma once

#include "cn105_types.h"
#include "cn105_utils.h"
#include "cn105_state.h"

using namespace devicestate;

namespace devicestate {

    uint8_t checkSum(uint8_t bytes[], int len);

    class CN105Protocol {
        private:
            CN105State state;

        public:
            // Write Protocol
            void prepareSetPacket(uint8_t* packet, int length);
            void createInfoPacket(uint8_t* packet, uint8_t code);
            void createPacket(uint8_t* packet, CN105State& hpState);
            // Write Protocol
    };

}
