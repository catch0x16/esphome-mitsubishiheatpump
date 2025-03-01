#include "esphome.h"

#include "devicestate.h"
using namespace devicestate;

#ifndef PID_WORKFLOWSTEP_H
#define PID_WORKFLOWSTEP_H

namespace workflow {

    namespace pid {
        
        class PidWorkflowStep : public WorkflowStep {
        private:
            float maxAdjustmentUnder;
            esphome::sensor::Sensor* pid_set_point_correction;
    
            PIDController *pidController;

            bool getOffsetDirection(const DeviceState* deviceState);
            void ensurePIDTarget(devicestate::DeviceStateManager* deviceManager, const float direction);
        
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
                esphome::sensor::Sensor* pid_set_point_correction
            );
        
            void run(const float currentTemperature, devicestate::DeviceStateManager* deviceManager);
        };

    }

}

#endif
