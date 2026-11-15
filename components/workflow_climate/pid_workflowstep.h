#ifndef PID_WORKFLOWSTEP_H
#define PID_WORKFLOWSTEP_H

#include <cstdint>

#include "controller_types.h"

#include "adaptive_pid.h"
#include "response_rate_estimator.h"

namespace workflow {

    namespace pid {
        
        class PidWorkflowStep : public devicestate::WorkflowStep {
        private:
            AdaptivePID *adaptivePID;
            ResponseRateEstimator estimator;
            TuningConfig tuning;

            float minTemp;
            float maxTemp;
            float maxAdjustmentOver;
            float maxAdjustmentUnder;
            float initialKi;
            bool heating{true};

            bool ensurePIDTarget(devicestate::IDeviceStateManager* deviceManager);
            void applyTunedGains(bool heating);
        
        public:
            PidWorkflowStep(
                const uint32_t updateInterval,
                const float minTemp,
                const float maxTemp,
                const float p,
                const float i,
                const float d,
                const float maxAdjustmentUnder,
                const float maxAdjustmentOver,
                const TuningConfig &tuning
            );
        
            void run(const float currentTemperature, devicestate::IDeviceStateManager* deviceManager) override;

            // Gains currently in use.
            float kp() const { return this->adaptivePID->kp(); }
            float ki() const { return this->adaptivePID->ki(); }
            float kd() const { return this->adaptivePID->kd(); }

            // Learned response rate, degrees per second, for heating or cooling.
            bool isAdaptive() const { return this->tuning.adaptive; }
            float responseRate(bool heating) const { return this->estimator.rate(heating); }
            float responseRate() const { return this->estimator.rate(this->heating); }
            uint32_t measurements() const { return this->estimator.measurements(); }
            // Restore a previously learned rate (e.g. from flash).
            void restoreResponseRate(bool heating, float rate);
        };

    }

}

#endif
