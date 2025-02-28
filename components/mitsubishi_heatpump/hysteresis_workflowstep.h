#include "esphome.h"

#include "devicestate.h"
using namespace devicestate;

#ifndef HYSTERESIS_WORKFLOWSTEP_H
#define HYSTERESIS_WORKFLOWSTEP_H

namespace workflow {

    namespace hysteresis {

        struct HysteresisResult {
            float currentTemperature;
            float targetTemperature;
            float delta;
            bool active;
            std::string label;
            bool shouldRun;
        };
        
        class HysteresisWorkflowStep : public WorkflowStep {
        private:
            float hysterisisOverOn;
            float hysterisisUnderOff;
        
            void executeHysteresisWorkflowStep(HysteresisResult* result, devicestate::DeviceStateManager* deviceManager);
            HysteresisResult getHysteresisResult(const float currentTemperature, devicestate::DeviceStateManager* deviceManager);
        
        public:
            HysteresisWorkflowStep(
                const float hysterisisOverOn,
                const float hysterisisUnderOff
            );
        
            void run(const float currentTemperature, devicestate::DeviceStateManager* deviceManager);
        };

    }

}

#endif
