#include "cn105_protocol.h"

#include "esphome.h"

using namespace devicestate;

namespace devicestate {

    static const char* TAG = "CN105Protocol"; // Logging tag

    uint8_t checkSum(uint8_t bytes[], int len) {
        uint8_t sum = 0;
        for (int i = 0; i < len; i++) {
            sum += bytes[i];
        }
        return (0xfc - sum) & 0xff;
    }

    int lookupByteMapIndex(const int valuesMap[], int len, int lookupValue, const char* debugInfo) {
        for (int i = 0; i < len; i++) {
            if (valuesMap[i] == lookupValue) {
                return i;
            }
        }
        ESP_LOGW("lookup", "%s caution value %d not found, returning -1", debugInfo, lookupValue);
        return -1;
    }

    int lookupByteMapIndex(const char* valuesMap[], int len, const char* lookupValue, const char* debugInfo) {
        for (int i = 0; i < len; i++) {
            if (strcasecmp(valuesMap[i], lookupValue) == 0) {
                return i;
            }
        }
        ESP_LOGW("lookup", "%s caution value %s not found, returning -1", debugInfo, lookupValue);
        return -1;
    }

    const char* lookupByteMapValue(const char* valuesMap[], const uint8_t byteMap[], int len, uint8_t byteValue, const char* debugInfo, const char* defaultValue) {
        for (int i = 0; i < len; i++) {
            if (byteMap[i] == byteValue) {
                return valuesMap[i];
            }
        }

        if (defaultValue != nullptr) {
            return defaultValue;
        } else {
            ESP_LOGW("lookup", "%s caution: value %d not found, returning value at index 0", debugInfo, byteValue);
            return valuesMap[0];
        }

    }

    int lookupByteMapValue(const int valuesMap[], const uint8_t byteMap[], int len, uint8_t byteValue, const char* debugInfo) {
        for (int i = 0; i < len; i++) {
            if (byteMap[i] == byteValue) {
                return valuesMap[i];
            }
        }
        ESP_LOGW("lookup", "%s caution: value %d not found, returning value at index 0", debugInfo, byteValue);
        return valuesMap[0];
    }

    // Write Protocol

    void CN105Protocol::prepareSetPacket(uint8_t* packet, int length) {
        ESP_LOGV(TAG, "preparing Set packet...");
        memset(packet, 0, length * sizeof(uint8_t));

        for (int i = 0; i < HEADER_LEN && i < length; i++) {
            packet[i] = HEADER[i];
        }
    }

    void CN105Protocol::createInfoPacket(uint8_t* packet, uint8_t code) {
        ESP_LOGD(TAG, "creating Info packet");
        // add the header to the packet
        for (int i = 0; i < INFOHEADER_LEN; i++) {
            packet[i] = INFOHEADER[i];
        }

        // directly set requested info code (0x02, 0x03, 0x06, 0x09, 0x42, ...)
        packet[5] = code;

        // pad the packet out
        for (int i = 0; i < 15; i++) {
            packet[i + 6] = 0x00;
        }

        // add the checksum
        uint8_t chkSum = checkSum(packet, 21);
        packet[21] = chkSum;
    }

    // Write Protocol

}
