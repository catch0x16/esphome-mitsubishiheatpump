#ifndef PID_WORKFLOWSTEP_H
#define PID_WORKFLOWSTEP_H

#include "esphome.h"

#include "devicestate_types.h"
using namespace devicestate;

#include "adaptive_pid.h"
#include "pid_gain_learner.h"

namespace workflow {

    namespace pid {
        
        class PidWorkflowStep : public WorkflowStep {
        private:
            AdaptivePID *adaptivePID;

            float minTemp;
            float maxTemp;
            float maxAdjustmentOver;
            float maxAdjustmentUnder;

            PidGainLearner *learner_ = nullptr;
            PidGainLearner::Sample learning_sample_{};
            bool learning_heating_ = true, learning_mode_selected_ = true;
            PidGainLearner::Gains configured_{};
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
        
            void attach_learner(PidGainLearner *learner, bool heating);
            void set_learning_sample(const PidGainLearner::Sample &sample, bool selected) {
                if (selected != learning_mode_selected_) {
                    adaptivePID->reset_history();
                    if (learner_) learner_->pause();
                }
                learning_mode_selected_ = selected;
                learning_sample_ = sample;
            }
            PidGainLearner::Gains active_gains() const {
                return {{adaptivePID->kp(), adaptivePID->ki(), adaptivePID->kd()}};
            }
            void pause_learning();
            void run(const float currentTemperature, devicestate::IDeviceStateManager* deviceManager);
        };

    }

}

#endif
