#ifndef PID_WORKFLOWSTEP_H
#define PID_WORKFLOWSTEP_H

#include "esphome.h"

#include "devicestate.h"
using namespace devicestate;

#include "adaptive_pidcontroller.h"

namespace workflow {

    namespace pid {
        
        class PidWorkflowStep : public WorkflowStep {
        private:
            AdaptivePIDController *adaptivePIDController;

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
