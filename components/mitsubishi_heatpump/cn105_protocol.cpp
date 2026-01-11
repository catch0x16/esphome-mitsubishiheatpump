#include "cn105_protocol.h"

#include "esphome.h"

using namespace devicestate;

namespace devicestate {

    void CN105Protocol::prepareSetPacket(uint8_t* packet, int length) {
        ESP_LOGV(TAG, "preparing Set packet...");
        memset(packet, 0, length * sizeof(uint8_t));

        for (int i = 0; i < HEADER_LEN && i < length; i++) {
            packet[i] = HEADER[i];
        }
    }

}
