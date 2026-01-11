#ifndef PID_WORKFLOWSTEP_H
#define PID_WORKFLOWSTEP_H

#include "esphome.h"

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
        
            void run(const float currentTemperature, devicestate::IDeviceStateManager* deviceManager);
        };

    }

}

#endif
