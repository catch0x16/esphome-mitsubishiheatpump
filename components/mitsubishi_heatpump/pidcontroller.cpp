#include "pidcontroller.h"

#include <cmath>
#include <optional>
#include <stdexcept>
#include <iostream>

using namespace std;

float roundToDecimals(const float value, const int n) {
    const int decimals = std::pow(10, n);
    return std::ceil(value * decimals) / decimals;
}

PIDController::PIDController(
    const float p,
    const float i,
    const float d,
    const int sampleTime,
    const float target,
    const float outputMin,
    const float outputMax
) {
    this->sampleTime = sampleTime;
    this->setTunings(p, i, d);
    this->setOutputLimits(outputMin, outputMax);
    this->setTarget(target);
}

float PIDController::update(const float input) {
    if (!this->lastInput.has_value()) {
        this->lastInput = input;
    }

    // 1 = 21 - 20
    const float error = this->target - input;
    //std::cout<<this->target - input<<" = "<<this->target<<" - "<<input<<"\n";

    // 0 = 20 - 20
    const float inputDelta = input - this->lastInput.value();
    //std::cout<<input - this->lastInput.value()<<" = "<<input<<" - "<<this->lastInput.value()<<"\n";

    const float originalOutputSum = this->outputSum;
    // 0.0006 = (0.0006 * 1)
    this->outputSum += (this->ki * error);
    //std::cout<<this->outputSum<<" = "<<originalOutputSum<<" + ("<<this->ki<<" * "<<error<<")\n";
    this->outputSum = this->applyOutputLimits(this->outputSum);

    // 0.1 = 0.1 * 1
    float output = this->kp * error;

    // 0.1006 = 0.1 + 0.0006 - 0.0 * 0
    output += this->outputSum - this->kd * inputDelta;

    // 0.1006
    this->output = this->applyOutputLimits(output);

    // Update state variables
    this->lastInput = input;
    return this->output;
}

void PIDController::setTarget(const float target) {
    this->target = target;
    this->resetState();
}

void PIDController::setOutputLimits(const float min, const float max) {
    if (min > max) {
        // throw new std::invalid_argument("Minimum output limit cannot be bigger than maximum");
        return;
    }

    this->outputMin = min;
    this->outputMax = max;

    // Apply to current state
    this->output = this->applyOutputLimits(this->output);
    this->outputSum = this->applyOutputLimits(this->outputSum);
}

void PIDController::setTunings(const float p, const float i, const float d) {
    if (p < 0 || i < 0 || d < 0) {
        //throw new std::invalid_argument("PID values cannot be below 0");
        return;
    }

    this->p = p;
    this->i = i;
    this->d = d;

    // Calculate internal pid representations
    const float sampleTimeSec = roundToDecimals(this->sampleTime / 1000.0, 6);
    this->kp = p;
    this->ki = i * sampleTimeSec;
    this->kd = d / sampleTimeSec;
}

float PIDController::getOutputMin() {
    return this->outputMin;
}

float PIDController::getOutputMax() {
    return this->outputMax;
}

float PIDController::getTarget() {
    return this->target;
}

void PIDController::resetState() {
    this->outputSum = this->target; // Start adjusting from target
    this->lastInput.reset();
}

float PIDController::applyOutputLimits(const float output) {
    if (output > this->outputMax) {
        return this->outputMax;
    }
    if (output < this->outputMin) {
        return this->outputMin;
    }
    return output;
}