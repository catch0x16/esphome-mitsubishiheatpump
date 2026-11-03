#ifndef PID_WORKFLOWSTEP_H
#define PID_WORKFLOWSTEP_H

#include "esphome.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include <array>
#include <cmath>

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
            std::array<esphome::sensor::Sensor*, 11> diagnostic_sensors_{};
            esphome::text_sensor::TextSensor* tuning_state_sensor_{nullptr};

            void publish_diagnostics();

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
                const float maxAdjustmentOver,
                const bool autotuneEnabled = true
            );
        
            void set_diagnostic_sensors(const std::array<esphome::sensor::Sensor*, 11>& sensors) {
                diagnostic_sensors_ = sensors;
            }
            void set_tuning_state_sensor(esphome::text_sensor::TextSensor* sensor) {
                tuning_state_sensor_ = sensor;
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
