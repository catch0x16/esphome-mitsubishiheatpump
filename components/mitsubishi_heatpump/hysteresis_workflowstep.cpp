#include "esphome.h"
using namespace esphome;

#include "devicestate.h"
using namespace devicestate;

namespace workflow {

    namespace hysteresis {

        static const char* TAG = "HysteresisWorkflowStep"; // Logging tag

        HysteresisWorkflowStep::HysteresisWorkflowStep(
            const float hysterisisOverOn,
            const float hysterisisUnderOff
        ) {
            this->hysterisisOverOn = hysterisisOverOn;
            this->hysterisisUnderOff = hysterisisUnderOff;
        }
        
        void HysteresisWorkflowStep::executeHysteresisWorkflowStep(
                HysteresisResult* result, devicestate::DeviceStateManager* deviceManager) {
            if (!result->active) {
                if (-result->delta > this->hysterisisOverOn) {
                    ESP_LOGI(TAG, "Turn on while %s: delta={%f} current={%f} targetTemperature={%f}", result->label.c_str(), result->delta, result->currentTemperature, result->targetTemperature);
                    deviceManager->internalTurnOn();
                }
                return;
            }
        
            if (result->delta > this->hysterisisUnderOff) {
                ESP_LOGI(TAG, "Turn off while %s: delta={%f} current={%f} targetTemperature={%f}", result->label.c_str(), result->delta, result->currentTemperature, result->targetTemperature);
                deviceManager->internalTurnOff();
                return;
            }
        }
        
        HysteresisResult HysteresisWorkflowStep::getHysteresisResult(
                const float currentTemperature, devicestate::DeviceStateManager* deviceManager) {
            HysteresisResult result;
        
            const float targetTemperature = deviceManager->getTargetTemperature();
        
            const DeviceState deviceState = deviceManager->getDeviceState();
            result.currentTemperature = currentTemperature;
            result.targetTemperature = targetTemperature;
            result.active = deviceState.active;
        
            switch(deviceState.mode) {
                case devicestate::DeviceMode::DeviceMode_Heat: {
                    const float delta = currentTemperature - targetTemperature;
                    result.shouldRun = true;
                    result.delta = delta;
                    result.label = "heating";
                    break;
                }
                case devicestate::DeviceMode::DeviceMode_Cool: {
                    const float delta = targetTemperature - currentTemperature;
                    result.shouldRun = true;
                    result.active = deviceState.active;
                    result.label = "cooling";
                    break;
                }
                default: {
                    ESP_LOGI(workflow::hysteresis::TAG, "Doing nothing in current mode (%s): current={%f} targetTemperature={%f}", 
                        devicestate::deviceModeToString(deviceState.mode), currentTemperature, targetTemperature);
                    result.shouldRun = false;
                }
            }
        
            return result;
        }
        
        void HysteresisWorkflowStep::run(const float currentTemperature, devicestate::DeviceStateManager* deviceManager) {
            HysteresisResult result = this->getHysteresisResult(currentTemperature, deviceManager);
            if (result.shouldRun) {
                this->executeHysteresisWorkflowStep(&result, deviceManager);
            }
        }

    }

}
