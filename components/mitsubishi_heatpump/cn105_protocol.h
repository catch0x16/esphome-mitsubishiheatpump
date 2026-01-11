#pragma once

#include "cn105_types.h"

using namespace devicestate;

namespace devicestate {

    class CN105Protocol {
        public:
            void prepareSetPacket(uint8_t* packet, int length);
    };

}
