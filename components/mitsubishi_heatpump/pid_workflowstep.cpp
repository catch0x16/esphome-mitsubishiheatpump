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
            this->minTemp = minTemp;
            this->maxTemp = maxTemp;
            this->maxAdjustmentUnder = maxAdjustmentUnder;
            this->maxAdjustmentOver = maxAdjustmentOver;

            this->pidController = new PIDController(
                p,
                i,
                d,
                updateInterval,
                minTemp,
                maxTemp
            );

            this->adaptivePIDController = new AdaptivePIDController(
                p,
                i,
                d,
                minTemp,
                maxTemp
            );

            this->adaptivePIDSimple = new AdaptivePIDSimple(
                p,
                i,
                d,
                minTemp,
                maxTemp
            );
            this->adaptivePIDSimple->set_learning_rates(0.02f, 0.005f, 0.003f);
            this->adaptivePIDSimple->set_plant_sensitivity(1.0f);
            this->adaptivePIDSimple->set_adapt_interval_ms(15000); // adapt every 15s
            this->adaptivePIDSimple->enable_adaptation(true);
        }

        bool PidWorkflowStep::ensurePIDTarget(devicestate::DeviceStateManager* deviceManager) {
            if (devicestate::same_float(deviceManager->getTargetTemperature(), this->adaptivePIDController->getTarget(), 0.01f)) {
                return false;
            }

            ESP_LOGW(TAG, "Standard PID target temp changing from %f to %f", this->adaptivePIDController->getTarget(), deviceManager->getTargetTemperature());
            this->pidController->setTarget(deviceManager->getTargetTemperature(), deviceManager->getOffsetDirection());

            ESP_LOGW(TAG, "Adaptive PID target temp changing from %f to %f", this->adaptivePIDController->getTarget(), deviceManager->getTargetTemperature());
            this->adaptivePIDController->setTarget(deviceManager->getTargetTemperature(), deviceManager->getOffsetDirection());

            ESP_LOGW(TAG, "Simple PID target temp changing from %f to %f", this->adaptivePIDController->getTarget(), deviceManager->getTargetTemperature());
            this->adaptivePIDSimple->set_target(deviceManager->getTargetTemperature(), deviceManager->getOffsetDirection());
            return true;
        }

        void PidWorkflowStep::run(const float currentTemperature, devicestate::DeviceStateManager* deviceManager) {
            const bool updatedPidTarget = this->ensurePIDTarget(deviceManager);

            // if pid target is not updated and internal power is not on
            if (updatedPidTarget || deviceManager->isInternalPowerOn()) {
                const float adjustMinOffset = deviceManager->getOffsetDirection()
                    ? this->maxAdjustmentUnder
                    : this->maxAdjustmentOver;
                const float adjustMaxOffset = deviceManager->getOffsetDirection()
                    ? this->maxAdjustmentOver
                    : this->maxAdjustmentUnder;
                const float adjustedMinTemp = devicestate::clamp(deviceManager->getTargetTemperature() - adjustMinOffset, this->minTemp, this->maxTemp);
                const float adjustedMaxTemp = devicestate::clamp(deviceManager->getTargetTemperature() + adjustMaxOffset, this->minTemp, this->maxTemp);

                ESP_LOGI(TAG, "PidWorkflowStep (Device) target={%.2f} adjustedMin={%.2f} adjustedMax={%.2f} powerOn={%s}",
                    this->adaptivePIDController->getTarget(), adjustedMinTemp, adjustedMaxTemp, YESNO(deviceManager->isInternalPowerOn()));

                unsigned long now = esphome::millis();
                const float setPointCorrectionAdaptive =
                    this->adaptivePIDController->update(currentTemperature, now, deviceManager->isInternalPowerOn());
                const float adjustedSetPointCorrectionAdaptive = devicestate::clamp(setPointCorrectionAdaptive, adjustedMinTemp, adjustedMaxTemp);
                ESP_LOGI(TAG, "PidWorkflowStep [Adaptive] setPointCorrection={%.2f} adjustedSetPoint={%.2f} kp={%.3f} ki={%.3f} kd={%.3f}",
                    setPointCorrectionAdaptive, adjustedSetPointCorrectionAdaptive,
                    this->adaptivePIDController->get_kp(), this->adaptivePIDController->get_ki(), this->adaptivePIDController->get_kd());

                const float setPointCorrectionSimple =
                    this->adaptivePIDSimple->update(currentTemperature, now, deviceManager->isInternalPowerOn());
                const float adjustedSetPointCorrectionSimple = devicestate::clamp(setPointCorrectionSimple, adjustedMinTemp, adjustedMaxTemp);
                ESP_LOGI(TAG, "PidWorkflowStep [Simple] setPointCorrection={%.2f} adjustedSetPoint={%.2f} kp={%.3f} ki={%.3f} kd={%.3f}",
                    setPointCorrectionSimple, adjustedSetPointCorrectionSimple,
                    this->adaptivePIDSimple->kp(), this->adaptivePIDSimple->ki(), this->adaptivePIDSimple->kd());

                const float setPointCorrection = this->pidController->update(currentTemperature);
                const float adjustedSetPointCorrection = devicestate::clamp(setPointCorrection, adjustedMinTemp, adjustedMaxTemp);
                ESP_LOGI(TAG, "PidWorkflowStep [Standard] setPointCorrection={%.2f} adjustedSetPoint={%.2f} kp={%.3f} ki={%.3f} kd={%.3f}",
                    setPointCorrection, adjustedSetPointCorrection,
                    this->pidController->getKp(), this->pidController->getKi(), this->pidController->getKd());

                if (deviceManager->internalSetCorrectedTemperature(adjustedSetPointCorrectionSimple)) {
                    if (!deviceManager->commit()) {
                        ESP_LOGE(TAG, "PidWorkflowStep failed to update corrected temperature");
                    }
                }
            }
        }

    }
}