#include "pid_workflowstep.h"

#include "esphome.h"
using namespace esphome;

#include "devicestatemanager.h"
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

            this->adaptivePID = new AdaptivePID(
                p,
                i,
                d,
                minTemp,
                maxTemp
            );
            this->adaptivePID->set_learning_rates(0.02f, 0.005f, 0.003f);
            this->adaptivePID->set_plant_sensitivity(1.0f);
            this->adaptivePID->set_adapt_interval_ms(15000); // adapt every 15s
            this->adaptivePID->enable_adaptation(true);
        }

        bool PidWorkflowStep::ensurePIDTarget(devicestate::IDeviceStateManager* deviceManager) {
            if (deviceManager == nullptr || this->adaptivePID == nullptr) {
                ESP_LOGW(TAG, "ensurePIDTarget: null parameter");
                return false;
            }
            if (devicestate::same_float(deviceManager->getTargetTemperature(), this->adaptivePID->get_target(), 0.01f)) {
                return false;
            }

            ESP_LOGW(TAG, "Simple PID target temp changing from %f to %f", this->adaptivePID->get_target(), deviceManager->getTargetTemperature());
            this->adaptivePID->set_target(deviceManager->getTargetTemperature(), deviceManager->getOffsetDirection());
            return true;
        }

        void PidWorkflowStep::run(const float currentTemperature, devicestate::IDeviceStateManager* deviceManager) {
            if (deviceManager == nullptr) {
                ESP_LOGW(TAG, "run: deviceManager is null");
                return;
            }
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
                    this->adaptivePID->get_target(), adjustedMinTemp, adjustedMaxTemp, YESNO(deviceManager->isInternalPowerOn()));

                unsigned long now = esphome::millis();

                const float setPointCorrectionSimple =
                    this->adaptivePID->update(currentTemperature, now, deviceManager->isInternalPowerOn());
                const float adjustedSetPointCorrectionSimple = devicestate::clamp(setPointCorrectionSimple, adjustedMinTemp, adjustedMaxTemp);
                ESP_LOGI(TAG, "PidWorkflowStep [Simple] setPointCorrection={%.2f} adjustedSetPoint={%.2f} kp={%.3f} ki={%.3f} kd={%.3f}",
                    setPointCorrectionSimple, adjustedSetPointCorrectionSimple,
                    this->adaptivePID->kp(), this->adaptivePID->ki(), this->adaptivePID->kd());

                if (deviceManager->internalSetCorrectedTemperature(adjustedSetPointCorrectionSimple)) {
                    deviceManager->commit();
                }
            }
        }

    }
}