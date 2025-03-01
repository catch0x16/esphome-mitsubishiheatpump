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
            const float maxAdjustmentOver,
            esphome::sensor::Sensor* pid_set_point_correction
        ) {
            this->pid_set_point_correction = pid_set_point_correction;

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
        }

        bool PidWorkflowStep::getOffsetDirection(const DeviceState* deviceState) {
            switch(deviceState->mode) {
                case devicestate::DeviceMode::DeviceMode_Heat: {
                    return true;
                }
                case devicestate::DeviceMode::DeviceMode_Cool: {
                    return false;
                }
                default: {
                    ESP_LOGE(TAG, "Unexpected state for offset direction");
                    return false;
                }
            }
        }

        void PidWorkflowStep::ensurePIDTarget(devicestate::DeviceStateManager* deviceManager, const float direction) {
            if (devicestate::same_float(deviceManager->getTargetTemperature(), this->pidController->getTarget(), 0.01f)) {
                return;
            }

            ESP_LOGI(TAG, "PID target temp changing from %f to %f", this->pidController->getTarget(), deviceManager->getTargetTemperature());
            this->pidController->setTarget(deviceManager->getTargetTemperature(), direction);
        }

        void PidWorkflowStep::run(const float currentTemperature, devicestate::DeviceStateManager* deviceManager) {
            ESP_LOGV(TAG, "PIDController update current: %.2f", currentTemperature);

            const DeviceStatus deviceStatus = deviceManager->getDeviceStatus();
            const DeviceState deviceState = deviceManager->getDeviceState();
            const bool direction = this->getOffsetDirection(&deviceState);

            this->ensurePIDTarget(deviceManager, direction);
            
            const float setPointCorrection = this->pidController->update(currentTemperature);
            const float correctionOffset = direction ? this->maxAdjustmentUnder : -this->maxAdjustmentUnder;
            const float setPointCorrectionOffset = setPointCorrection - correctionOffset;
            
            const float oldCorrectedTargetTemperature = deviceManager->getCorrectedTargetTemperature();
            if (!devicestate::same_float(setPointCorrection, oldCorrectedTargetTemperature)) {
                if (deviceManager->internalSetCorrectedTemperature(setPointCorrectionOffset)) {
                    ESP_LOGW(TAG, "Adjusted setpoint: oldCorrection={%f} newCorrection={%f} current={%f} deviceCurrent={%f} deviceTarget={%f} componentTarget={%f}", oldCorrectedTargetTemperature, setPointCorrectionOffset, currentTemperature, deviceStatus.currentTemperature, deviceState.targetTemperature, deviceManager->getTargetTemperature());
                    if (!deviceManager->commit()) {
                        ESP_LOGE(TAG, "Failed to update device state");
                    }
                }
            } else {
                ESP_LOGD(TAG, "Skipping setpoint adjustment: oldCorrection={%f} newCorrection={%f} current={%f} deviceCurrent={%f} deviceTarget={%f} componentTarget={%f}", oldCorrectedTargetTemperature, setPointCorrectionOffset, currentTemperature, deviceStatus.currentTemperature, deviceState.targetTemperature, deviceManager->getTargetTemperature());
            }

            ESP_LOGV(TAG, "PIDController set point target: %.2f", deviceManager->getTargetTemperature());
            ESP_LOGV(TAG, "PIDController set point correction: %.2f", deviceManager->getCorrectedTargetTemperature());
            this->pid_set_point_correction->publish_state(deviceManager->getCorrectedTargetTemperature());
        }

    }
}