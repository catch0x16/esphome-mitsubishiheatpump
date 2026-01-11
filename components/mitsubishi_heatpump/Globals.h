#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#endif

#include <esphome.h>

#define CUSTOM_MILLIS esphome::millis()
#define CUSTOM_DELAY(x) esphome::delay(x)