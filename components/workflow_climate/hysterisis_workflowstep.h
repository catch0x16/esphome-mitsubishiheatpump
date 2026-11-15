#include <string>

#include "controller_types.h"

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
        
        class HysterisisWorkflowStep : public devicestate::WorkflowStep {
        private:
            float hysterisisOn;
            float hysterisisOff;
        
            void executeHysterisisWorkflowStep(HysterisisResult* result, devicestate::IDeviceStateManager* deviceManager);
            HysterisisResult getHysterisisResult(const float currentTemperature, devicestate::IDeviceStateManager* deviceManager);
        
        public:
            HysterisisWorkflowStep(
                const float hysterisisOn,
                const float hysterisisOff
            );
        
            void run(const float currentTemperature, devicestate::IDeviceStateManager* deviceManager) override;
        };

    }

}

#endif
