#include <cmath>
#include <vector>

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

    __attribute__((unused))static double calculateDelta(double y1, double y2) {
        return y2 - y1;
    }

    class CircularBuffer {
        private:
            int capacity;
            int count;
            int head;
            int tail;

            float* items;

        public:
            CircularBuffer(int capacity) {
                this->capacity = capacity;
                this->count = 0;
                this->items = new float[capacity];
                this->head = 0;
                this->tail = 0;
            }

            bool offer(float next) {
                if (this->count >= this->capacity + 1) {
                    return false;
                }

                this->items[this->tail] = next;
                this->tail = (this->tail + 1) % this->capacity;
                this->count++;
                return true;
            }

            bool pop() {
                if (this->count == 0) {
                    return false;
                }

                this->head = (this->head + 1) % this->capacity;
                this->count--;
                return true;
            }
    };

    class MovingSlopeAverage {
        private:
            int window;
            float deltaSum;

            std::vector<float> data;

        public:
            MovingSlopeAverage(const int window) {
                this->window = window;
                this->deltaSum = 0;

                this->data = {};
            }

            float getAverage() {
                const int n = this->data.size();
                if (n < 2) {
                    return 0;
                }
                return this->deltaSum / n;
            }

            float increment(float next) {
                const int n = this->data.size();

                if (n == window) {
                    double lastDelta = calculateDelta(this->data[0], this->data[1]);
                    deltaSum -= lastDelta;
                    this->data.erase(this->data.begin());
                }

                double delta = calculateDelta(this->data[n - 1], next);
                this->deltaSum += delta;
                this->data.push_back(next);
                return delta;
            }
    };

}

#endif