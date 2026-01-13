#pragma once

#include "cn105_types.h"
#include "cn105_utils.h"
#include "cn105_state.h"

using namespace devicestate;

namespace devicestate {

    uint8_t checkSum(uint8_t bytes[], int len);

    class CN105Protocol {
        public:
            // Write Protocol
            void prepareSetPacket(uint8_t* packet, int length);
            void createInfoPacket(uint8_t* packet, uint8_t code);
            void prepareInfoPacket(uint8_t* packet, int length);
            void createPacket(uint8_t* packet, CN105State& hpState);
            // Write Protocol

            // Read Protocol
            void parseSettings0x02(uint8_t* packet, CN105State& hpState);
            void parseStatus0x03(uint8_t* packet, CN105State& hpState);
            void parseTimers0x05(uint8_t* packet, CN105State& hpState);
            void parseStatus0x06(uint8_t* packet, CN105State& hpState);
            void parseFunctions0x20(uint8_t* packet, CN105State& hpState);
            void parseFunctions0x22(uint8_t* packet, CN105State& hpState);
            // Read Protocol

    };

}
