#pragma once
#include <cstdint>

namespace devicestate {

    class IIODevice {
    public:
        virtual bool begin() = 0;
        virtual void write(uint8_t) = 0;
        virtual int available(void) = 0;
        virtual bool read(uint8_t*) = 0;
        virtual ~IIODevice() = default;    // Virtual destructor for safety
    };

}