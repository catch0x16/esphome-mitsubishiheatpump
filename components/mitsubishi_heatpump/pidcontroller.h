// p, i, d, sampleTime, target, outputMin, outputMax, proportionalOnMeasurement = false

#ifndef PIDC_H
#define PIDC_H

#include <optional>

float roundToDecimals(const float value, const int n);

/*
    Based on: @mariusrumpf/pid-controller
*/
class PIDController {
    private:
        float p;
        float i;
        float d;

        float kp;
        float ki;
        float kd;
        float outputMin;
        float outputMax;

        float output;
        float outputSum = 0.0f;
        float target;
        std::optional<float> lastInput;
        int sampleTime;

        float applyOutputLimits(const float output);

    public:
        PIDController(
            const float p,
            const float i,
            const float d,
            const int sampleTime,
            const float target,
            const float outputMin,
            const float outputMax
        );

        float getOutputMin();
        float getOutputMax();

        float getTarget();
        void setTarget(const float target);
        float update(const float input);
        void resetState();
        void setSampleTime(const int sampleTime);
        void setTunings(const float p, const float i, const float d);
        void setOutputLimits(const float outputMin, const float outputMax);
};

#endif