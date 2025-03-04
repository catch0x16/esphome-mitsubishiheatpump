#include <cmath>

#ifndef FLOATSDS_H
#define FLOATSDS_H

namespace devicestate {

    __attribute__((unused))static float roundToDecimals(const float value, const int n) {
        const int decimals = std::pow(10, n);
        return std::ceil(value * decimals) / decimals;
    }

    __attribute__((unused))static bool same_float(const float a, const float b, const float epsilon = 0.001f) {
        if (std::isnan(a) || std::isnan(b)) {
            return false;
        }
        return std::fabs(a - b) <= ( (std::fabs(a) > std::fabs(b) ? std::fabs(b) : std::fabs(a)) * epsilon);
    }

    __attribute__((unused))static float clamp(const float value, const float min, const float max) {
        if (value < min) {
            return min;
        } else if (value > max) {
            return max;
        }
        return value;
    }

}

#endif