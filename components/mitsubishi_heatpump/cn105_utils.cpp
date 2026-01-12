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

}
