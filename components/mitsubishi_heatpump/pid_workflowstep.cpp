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

            this->adaptivePIDController = new AdaptivePIDController(
                p,
                i,
                d,
                minTemp,
                maxTemp,
                maxAdjustmentOver,
                maxAdjustmentUnder
            );
        }

        bool PidWorkflowStep::ensurePIDTarget(devicestate::DeviceStateManager* deviceManager) {
            if (devicestate::same_float(deviceManager->getTargetTemperature(), this->pidController->getTarget(), 0.01f)) {
                return false;
            }

            ESP_LOGW(TAG, "PID target temp changing from %f to %f", this->pidController->getTarget(), deviceManager->getTargetTemperature());
            this->pidController->setTarget(deviceManager->getTargetTemperature(), deviceManager->getOffsetDirection());

            ESP_LOGW(TAG, "Adaptive PID target temp changing from %f to %f", this->adaptivePIDController->getTarget(), deviceManager->getTargetTemperature());
            this->adaptivePIDController->setTarget(deviceManager->getTargetTemperature(), deviceManager->getOffsetDirection());
            return true;
        }

        void PidWorkflowStep::run(const float currentTemperature, devicestate::DeviceStateManager* deviceManager) {
            const bool updatedPidTarget = this->ensurePIDTarget(deviceManager);

            unsigned long now = esphome::millis();
            const float setPointCorrectionAdaptive =
                this->adaptivePIDController->update(currentTemperature, now, deviceManager->isInternalPowerOn());
            ESP_LOGI(TAG, "PidWorkflowStep setPointCorrectionAdaptive={%.2f} target={%.2f} adjustedMin={%.2f} adjustedMax={%.2f} powerOn={%s} kp={%.3f} ki={%.3f} kd={%.3f}",
                setPointCorrectionAdaptive, this->pidController->getTarget(), this->adaptivePIDController->getAdjustedMin(), this->adaptivePIDController->getAdjustedMax(), YESNO(deviceManager->isInternalPowerOn()),
                this->adaptivePIDController->get_kp(), this->adaptivePIDController->get_ki(), this->adaptivePIDController->get_kd());

            if (updatedPidTarget || deviceManager->isInternalPowerOn()) {
                const float setPointCorrection = this->pidController->update(currentTemperature);
                ESP_LOGI(TAG, "PidWorkflowStep setPointCorrection={%.2f} target={%.2f} adjustedMin={%.2f} adjustedMax={%.2f} powerOn={%s}",
                    setPointCorrection, this->pidController->getTarget(), this->pidController->getAdjustedMin(), this->pidController->getAdjustedMax(), YESNO(deviceManager->isInternalPowerOn()));
                if (deviceManager->internalSetCorrectedTemperature(setPointCorrection)) {
                    if (!deviceManager->commit()) {
                        ESP_LOGE(TAG, "PidWorkflowStep failed to update corrected temperature");
                    }
                }
            }
        }

    }
}