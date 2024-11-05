#include <cmath>

#ifndef FLOATSDS_H
#define FLOATSDS_H

namespace devicestate {

    static bool same_float(const float a, const float b, const float epsilon) {
        if (std::isnan(a) || std::isnan(b)) {
            return false;
        }
        return std::fabs(a - b) <= ( (std::fabs(a) > std::fabs(b) ? std::fabs(b) : std::fabs(a)) * epsilon);
    }

    static float clamp(const float value, const float min, const float max) {
        if (value < min) {
            return min;
        } else if (value > max) {
            return max;
        }
        return value;
    }

}

#endif