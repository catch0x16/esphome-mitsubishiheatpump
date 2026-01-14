#include "cn105_logging.h"

#include "cn105_utils.h"
#include <cmath>

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

    void debugSettings(const char* settingName, heatpumpSettings& settings) {
        ESP_LOGD(LOG_SETTINGS_TAG, "[%s]-> [power: %s, target °C: %.1f, mode: %s, fan: %s, vane: %s, wvane: %s]",
            getIfNotNull(settingName, "unnamed"),
            getIfNotNull(settings.power, "-"),
            settings.temperature,
            getIfNotNull(settings.mode, "-"),
            getIfNotNull(settings.fan, "-"),
            getIfNotNull(settings.vane, "-"),
            getIfNotNull(settings.wideVane, "-")
        );
    }

    void debugStatus(const char* statusName, heatpumpStatus status) {
        // Déclarez un buffer (tableau de char) pour la conversion float -> string
        // 6 caractères suffisent pour "-99.9\0"
        static char outside_temp_buffer[6];

        ESP_LOGI(LOG_STATUS_TAG, "[%s]-> [room C°: %.1f, outside C°: %s, operating: %s, compressor freq: %.1f Hz]",
            statusName,
            status.roomTemperature,
            // Utilisation de snprintf dans l'expression ternaire
            std::isnan(status.outsideAirTemperature)
                ? "N/A"
                : (snprintf(outside_temp_buffer, sizeof(outside_temp_buffer), "%.1f", status.outsideAirTemperature) > 0 ? outside_temp_buffer : "ERR"),
            status.operating ? "YES" : "NO ",
            status.compressorFrequency);
    }


}