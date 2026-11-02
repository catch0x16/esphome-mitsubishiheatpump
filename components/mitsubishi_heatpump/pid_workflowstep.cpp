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
            const float maxAdjustmentOver,
            const bool autotuneEnabled
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
            this->adaptivePID->enable_adaptation(autotuneEnabled);
        }

        bool PidWorkflowStep::ensurePIDTarget(devicestate::IDeviceStateManager* deviceManager) {
            if (deviceManager == nullptr || this->adaptivePID == nullptr) {
                ESP_LOGW(TAG, "ensurePIDTarget: null parameter");
                return false;
            }
            const auto mode = deviceManager->getDeviceState().mode;
            const bool heating = mode == DeviceMode::DeviceMode_Heat;
            if (devicestate::same_float(deviceManager->getTargetTemperature(), this->adaptivePID->get_target(), 0.01f) &&
                heating == this->adaptivePID->is_heating()) {
                return false;
            }

            ESP_LOGW(TAG, "Simple PID target temp changing from %f to %f", this->adaptivePID->get_target(), deviceManager->getTargetTemperature());
            this->adaptivePID->set_target(deviceManager->getTargetTemperature(), heating);
            return true;
        }

        void PidWorkflowStep::publish_diagnostics(float kp, float ki, float kd) {
            const auto& tuner = adaptivePID->tuner();
            const float values[] = {kp, ki, kd, adaptivePID->p_term(), adaptivePID->i_term(), adaptivePID->d_term(),
                tuner.baseline_score(), tuner.trial_score(), static_cast<float>(tuner.accepted()),
                static_cast<float>(tuner.rejected()), static_cast<float>(tuner.discarded())};
            for (size_t index = 0; index < diagnostic_sensors_.size(); ++index) {
                if (diagnostic_sensors_[index] != nullptr) {
                    diagnostic_sensors_[index]->publish_state(values[index]);
                }
            }
            if (tuning_state_sensor_ != nullptr) tuning_state_sensor_->publish_state(tuner.status());
        }

        void PidWorkflowStep::run(const float currentTemperature, devicestate::IDeviceStateManager* deviceManager,
                                  bool enabled) {
            if (deviceManager == nullptr) {
                ESP_LOGW(TAG, "run: deviceManager is null");
                return;
            }
            const bool updatedPidTarget = this->ensurePIDTarget(deviceManager);
            const auto mode = deviceManager->getDeviceState().mode;
            const bool conditioning = mode == DeviceMode::DeviceMode_Heat || mode == DeviceMode::DeviceMode_Cool;
            const bool heating = mode == DeviceMode::DeviceMode_Heat;
            const float adjustMinOffset = heating ? this->maxAdjustmentUnder : this->maxAdjustmentOver;
            const float adjustMaxOffset = heating ? this->maxAdjustmentOver : this->maxAdjustmentUnder;
            const float adjustedMinTemp = devicestate::clamp(deviceManager->getTargetTemperature() - adjustMinOffset, this->minTemp, this->maxTemp);
            const float adjustedMaxTemp = devicestate::clamp(deviceManager->getTargetTemperature() + adjustMaxOffset, this->minTemp, this->maxTemp);
            this->adaptivePID->set_output_limits(adjustedMinTemp, adjustedMaxTemp);

            // Snapshot the gains used for this output: adaptation runs afterward
            // and its new gains take effect on the next update.
            const float kp = adaptivePID->kp();
            const float ki = adaptivePID->ki();
            const float kd = adaptivePID->kd();
            const bool powerOn = enabled && deviceManager->isInternalPowerOn();
            const float setPointCorrectionSimple = this->adaptivePID->update(
                currentTemperature, esphome::millis(), powerOn, enabled && conditioning);
            this->publish_diagnostics(kp, ki, kd);

            // Keep PID timing/reset state current while disabled, without sending
            // a new unit setpoint on each disabled update.
            if (enabled && std::isfinite(currentTemperature) && (updatedPidTarget || powerOn)) {

                ESP_LOGI(TAG, "PidWorkflowStep (Device) target={%.2f} adjustedMin={%.2f} adjustedMax={%.2f} powerOn={%s}",
                    this->adaptivePID->get_target(), adjustedMinTemp, adjustedMaxTemp, YESNO(deviceManager->isInternalPowerOn()));

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
