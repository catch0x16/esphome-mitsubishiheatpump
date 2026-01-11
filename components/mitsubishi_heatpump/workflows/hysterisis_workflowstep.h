#include "esphome.h"

#include "../devicestate.h"
using namespace devicestate;

#ifndef HYSTERISIS_WORKFLOWSTEP_H
#define HYSTERISIS_WORKFLOWSTEP_H

namespace workflow {

    namespace hysterisis {

        struct HysterisisResult {
            float currentTemperature;
            float targetTemperature;
            float delta;
            bool active;
            std::string label;
            bool shouldRun;
        };
        
        class HysterisisWorkflowStep : public WorkflowStep {
        private:
            float hysterisisOn;
            float hysterisisOff;
        
            void executeHysterisisWorkflowStep(HysterisisResult* result, devicestate::DeviceStateManager* deviceManager);
            HysterisisResult getHysterisisResult(const float currentTemperature, devicestate::DeviceStateManager* deviceManager);
        
        public:
            HysterisisWorkflowStep(
                const float hysterisisOn,
                const float hysterisisOff
            );
        
            void run(const float currentTemperature, devicestate::DeviceStateManager* deviceManager);
        };

    }

}

#endif
