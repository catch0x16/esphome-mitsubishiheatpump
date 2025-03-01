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
            const float offsetAdjustment,
            esphome::sensor::Sensor* pid_set_point_correction
        ) {
            this->offsetAdjustment = offsetAdjustment;
            this->pid_set_point_correction = pid_set_point_correction;

            this->pidController = new PIDController(
                p,
                i,
                d,
                updateInterval,
                (maxTemp + minTemp) / 2.0, // Set target to mid point of min/max
                minTemp,
                maxTemp,
                maxAdjustmentOver + this->offsetAdjustment,
                maxAdjustmentUnder -  - this->offsetAdjustment
            );
        }

        float PidWorkflowStep::getOffsetCorrection(const DeviceState* deviceState) {
            switch(deviceState->mode) {
                case devicestate::DeviceMode::DeviceMode_Heat: {
                    // Heating: 70 - 0.25 = 68.0;
                    return this->offsetAdjustment;
                }
                case devicestate::DeviceMode::DeviceMode_Cool: {
                    // Cooling: 70 + 0.25 = 72.0;
                    return -this->offsetAdjustment;
                }
                default:
                    return 0.0;
            }
        }

        void PidWorkflowStep::ensurePIDTarget(devicestate::DeviceStateManager* deviceManager) {
            if (devicestate::same_float(deviceManager->getTargetTemperature(), this->pidController->getTarget(), 0.01f)) {
                return;
            }

            ESP_LOGI(TAG, "PID target temp changing from %f to %f", this->pidController->getTarget(), deviceManager->getTargetTemperature());
            this->pidController->setTarget(deviceManager->getTargetTemperature());
        }

        void PidWorkflowStep::run(const float currentTemperature, devicestate::DeviceStateManager* deviceManager) {
            ESP_LOGV(TAG, "PIDController update current: %.2f", currentTemperature);
            this->ensurePIDTarget(deviceManager);
            
            const float setPointCorrection = this->pidController->update(currentTemperature);
            const DeviceState deviceState = deviceManager->getDeviceState();
            const float correctionOffset = this->getOffsetCorrection(&deviceState);
            const float setPointCorrectionOffset = setPointCorrection - correctionOffset;
            
            const DeviceStatus deviceStatus = deviceManager->getDeviceStatus();
            const float oldCorrectedTargetTemperature = deviceManager->getCorrectedTargetTemperature();
            if (!devicestate::same_float(setPointCorrectionOffset, oldCorrectedTargetTemperature)) {
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