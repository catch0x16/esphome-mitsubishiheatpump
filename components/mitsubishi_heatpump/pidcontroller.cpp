#include "pidcontroller.h"

#include <cmath>
#include <optional>
#include <stdexcept>

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

    if (this->outputSum != this->outputSum) {
        this->outputSum = 0.0f;
    }

    const float error = this->target - input;
    const float inputDelta = input - this->lastInput.value();

    this->outputSum += (this->ki * error);
    this->outputSum = this->applyOutputLimits(this->outputSum);


    float output = this->kp * error;

    output += this->outputSum - this->kd * inputDelta;

    this->output = this->applyOutputLimits(output);

    // Update state variables
    this->lastInput = input;
    return this->output;
}

void PIDController::setTarget(const float target) {
    this->target = target;
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
    this->outputSum = 0.0f;
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