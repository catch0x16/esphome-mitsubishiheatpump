#ifndef PID_WORKFLOWSTEP_H
#define PID_WORKFLOWSTEP_H

#include "esphome.h"
#include <array>

#include "devicestate_types.h"
using namespace devicestate;

#include "adaptive_pid.h"

namespace workflow {

    namespace pid {
        
        class PidWorkflowStep : public WorkflowStep {
        private:
            AdaptivePID *adaptivePID;

            float minTemp;
            float maxTemp;
            float maxAdjustmentOver;
            float maxAdjustmentUnder;
            // Kp, Ki, Kd, P term, I term, D term (same order as climate.py).
            std::array<esphome::sensor::Sensor*, 6> diagnostic_sensors_{};

            void publish_diagnostics(float kp, float ki, float kd);

            bool ensurePIDTarget(devicestate::IDeviceStateManager* deviceManager);
        
        public:
            PidWorkflowStep(
                const uint32_t updateInterval,
                const float minTemp,
                const float maxTemp,
                const float p,
                const float i,
                const float d,
                const float maxAdjustmentUnder,
                const float maxAdjustmentOver
            );
        
            void set_diagnostic_sensors(const std::array<esphome::sensor::Sensor*, 6>& sensors) {
                diagnostic_sensors_ = sensors;
            }

            void run(const float currentTemperature, devicestate::IDeviceStateManager* deviceManager) override {
                run(currentTemperature, deviceManager, true);
            }

            void run(const float currentTemperature, devicestate::IDeviceStateManager* deviceManager,
                     bool enabled);
        };

    }

}

#endif
