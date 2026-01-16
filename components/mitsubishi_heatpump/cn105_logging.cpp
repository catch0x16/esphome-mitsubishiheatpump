#include "cn105_logging.h"

#include "cn105_utils.h"
#include <cmath>

#include "esphome.h"

namespace devicestate {

    void hpPacketDebug(uint8_t* packet, unsigned int length, const char* packetDirection) {
        // Build the output string safely and efficiently
        std::string output;
        output.reserve(length * 3 + 1); // "FF " per byte

        char byteBuf[4];
        for (unsigned int i = 0; i < length; i++) {
            // Always bounded to 3 characters + NUL
            int written = snprintf(byteBuf, sizeof(byteBuf), "%02X ", packet[i]);
            if (written > 0) {
                output.append(byteBuf, static_cast<size_t>(written));
            }
        }

        char outputForSensor[15];
        // Properly truncate for eventual publication to a sensor
        strncpy(outputForSensor, output.c_str(), sizeof(outputForSensor) - 1);
        outputForSensor[sizeof(outputForSensor) - 1] = '\0';

        /*if (strcasecmp(packetDirection, "WRITE") == 0) {
            this->last_sent_packet_sensor->publish_state(outputForSensor);
        }*/

        ESP_LOGD(packetDirection, "%s", output.c_str());
    }

    void debugSettings(const char* settingName, heatpumpSettings& settings) {
        ESP_LOGI(LOG_SETTINGS_TAG, "[%s]-> [power: %s, target °C: %.1f, mode: %s, fan: %s, vane: %s, wvane: %s]",
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
        // Declare a buffer (char array) for float -> string conversion
        // 6 characters are enough for "-99.9\0"
        static char outside_temp_buffer[6];

        ESP_LOGI(LOG_STATUS_TAG, "[%s]-> [room C°: %.1f, outside C°: %s, operating: %s, compressor freq: %.1f Hz]",
            statusName,
            status.roomTemperature,
            // Using snprintf in the ternary expression
            std::isnan(status.outsideAirTemperature)
                ? "N/A"
                : (snprintf(outside_temp_buffer, sizeof(outside_temp_buffer), "%.1f", status.outsideAirTemperature) > 0 ? outside_temp_buffer : "ERR"),
            status.operating ? "YES" : "NO ",
            status.compressorFrequency);
    }


}