#ifndef PIDC_H
#define PIDC_H

#include <optional>
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
        float maxAdjustmentOver;
        float maxAdjustmentUnder;
        float adjustedMin;
        float adjustedMax;

        float output;
        float outputSum = 0.0f;
        float target;
        std::optional<float> lastInput;
        int sampleTime;

        float applyOutputLimits(const float output);
        void adjustOutputLimits(const bool direction);
        void setTunings(const float p, const float i, const float d);

    public:
        PIDController(
            const float p,
            const float i,
            const float d,
            const int sampleTime,
            const float outputMin,
            const float outputMax,
            const float maxAdjustmentOver,
            const float maxAdjustmentUnder
        );

        float getOutputMin();
        float getOutputMax();
        float getAdjustedMin();
        float getAdjustedMax();

        float getTarget();
        void setTarget(const float target, const bool direction);
        float update(const float input);
        void resetState();
};

#endif
