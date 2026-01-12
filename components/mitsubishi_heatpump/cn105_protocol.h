#pragma once

#include "cn105_types.h"

using namespace devicestate;

namespace devicestate {

    uint8_t checkSum(uint8_t bytes[], int len);

    const char* lookupByteMapValue(const char* valuesMap[], const uint8_t byteMap[], int len, uint8_t byteValue, const char* debugInfo = "", const char* defaultValue = nullptr);
    int lookupByteMapValue(const int valuesMap[], const uint8_t byteMap[], int len, uint8_t byteValue, const char* debugInfo = "");
    int lookupByteMapIndex(const char* valuesMap[], int len, const char* lookupValue, const char* debugInfo = "");
    int lookupByteMapIndex(const int valuesMap[], int len, int lookupValue, const char* debugInfo = "");

    class CN105Protocol {
        public:
            // Write Protocol
            void prepareSetPacket(uint8_t* packet, int length);
            void createInfoPacket(uint8_t* packet, uint8_t code);
            void createPacket(uint8_t* packet);
            // Write Protocol
    };

}
