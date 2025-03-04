#include "esphome.h"

#include "devicestate.h"
using namespace devicestate;

#ifndef PID_WORKFLOWSTEP_H
#define PID_WORKFLOWSTEP_H

namespace workflow {

    namespace pid {
        
        class PidWorkflowStep : public WorkflowStep {
        private:
            PIDController *pidController;
            bool resetRequired;

            bool ensurePIDTarget(devicestate::DeviceStateManager* deviceManager);
        
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
        
            void run(const float currentTemperature, devicestate::DeviceStateManager* deviceManager);
        };

    }

}

#endif
