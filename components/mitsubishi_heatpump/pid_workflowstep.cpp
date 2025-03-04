#include "pid_workflowstep.h"

#include "esphome.h"
using namespace esphome;

#include "devicestate.h"
using namespace devicestate;

namespace workflow {

    namespace pid {

        static const char* TAG = "PidWorkflowStep"; // Logging tag

        PidWorkflowStep::PidWorkflowStep(
            const uint32_t updateInterval,
            const float minTemp,
            const float maxTemp,
            const float p,
            const float i,
            const float d,
            const float maxAdjustmentUnder,
            const float maxAdjustmentOver
        ) {
            this->pidController = new PIDController(
                p,
                i,
                d,
                updateInterval,
                minTemp,
                maxTemp,
                maxAdjustmentOver,
                maxAdjustmentUnder
            );
            this->resetRequired = true;
        }

        bool PidWorkflowStep::ensurePIDTarget(devicestate::DeviceStateManager* deviceManager) {
            if (devicestate::same_float(deviceManager->getTargetTemperature(), this->pidController->getTarget(), 0.01f)) {
                return false;
            }

            ESP_LOGI(TAG, "PID target temp changing from %f to %f", this->pidController->getTarget(), deviceManager->getTargetTemperature());
            this->pidController->setTarget(deviceManager->getTargetTemperature(), deviceManager->getOffsetDirection());
            return true;
        }

        void PidWorkflowStep::run(const float currentTemperature, devicestate::DeviceStateManager* deviceManager) {
            ESP_LOGV(TAG, "PIDController update current: %.2f", currentTemperature);

            const bool updatedPidTarget = this->ensurePIDTarget(deviceManager);

            // if pid target is not updated and internal power is not on
            if (updatedPidTarget || deviceManager->isInternalPowerOn()) {
                if (this->resetRequired) {
                    this->resetRequired = false;
                    this->pidController->resetState();
                }

                const float setPointCorrection = this->pidController->update(currentTemperature);
                if (deviceManager->internalSetCorrectedTemperature(setPointCorrection)) {
                    if (!deviceManager->commit()) {
                        ESP_LOGE(TAG, "PidWorkflowStep failed to update corrected temperature");
                    }
                }
            } else {
                this->resetRequired = true;
            }
        }

    }
}