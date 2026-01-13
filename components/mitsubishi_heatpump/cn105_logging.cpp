#include "cn105_logging.h"

#include "esphome.h"

namespace devicestate {

    void hpPacketDebug(uint8_t* packet, unsigned int length, const char* packetDirection) {
        // Construire la chaîne de sortie de façon sûre et performante
        std::string output;
        output.reserve(length * 3 + 1); // "FF " par octet

        char byteBuf[4];
        for (unsigned int i = 0; i < length; i++) {
            // Toujours borné à 3 caractères + NUL
            int written = snprintf(byteBuf, sizeof(byteBuf), "%02X ", packet[i]);
            if (written > 0) {
                output.append(byteBuf, static_cast<size_t>(written));
            }
        }

        char outputForSensor[15];
        // Tronquer proprement pour la publication éventuelle sur un capteur
        strncpy(outputForSensor, output.c_str(), sizeof(outputForSensor) - 1);
        outputForSensor[sizeof(outputForSensor) - 1] = '\0';

        /*if (strcasecmp(packetDirection, "WRITE") == 0) {
            this->last_sent_packet_sensor->publish_state(outputForSensor);
        }*/

        ESP_LOGD(packetDirection, "%s", output.c_str());
    }

}