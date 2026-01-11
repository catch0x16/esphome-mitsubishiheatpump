#pragma once

#include "cn105_types.h"

using namespace devicestate;

namespace devicestate {

    class CN105Protocol {
        public:
            // Can these eventually move to private?
            uint8_t checkSum(uint8_t bytes[], int len);
            const char* lookupByteMapValue(const char* valuesMap[], const uint8_t byteMap[], int len, uint8_t byteValue, const char* debugInfo = "", const char* defaultValue = nullptr);
            int lookupByteMapValue(const int valuesMap[], const uint8_t byteMap[], int len, uint8_t byteValue, const char* debugInfo = "");
            int lookupByteMapIndex(const char* valuesMap[], int len, const char* lookupValue, const char* debugInfo = "");
            int lookupByteMapIndex(const int valuesMap[], int len, int lookupValue, const char* debugInfo = "");
            // Can these eventually move to private?

            // Write Protocol
            void prepareSetPacket(uint8_t* packet, int length);
            // Write Protocol
    };

}
