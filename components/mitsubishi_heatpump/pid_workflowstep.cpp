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
            configured_ = {{p, i, d}};
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
            if (devicestate::same_float(deviceManager->getTargetTemperature(), this->adaptivePID->get_target(), 0.01f) &&
                deviceManager->getOffsetDirection() == this->adaptivePID->heating()) {
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

            if (learner_) {
                const bool direction = deviceManager->getOffsetDirection();
                auto g = (direction == learning_heating_ && learning_mode_selected_) ? learner_->gains() : configured_;
                adaptivePID->set_gains(g[0], g[1], g[2]);
                const float low = devicestate::clamp(deviceManager->getTargetTemperature() -
                    (direction ? maxAdjustmentUnder : maxAdjustmentOver), minTemp, maxTemp);
                const float high = devicestate::clamp(deviceManager->getTargetTemperature() +
                    (direction ? maxAdjustmentOver : maxAdjustmentUnder), minTemp, maxTemp);
                const uint32_t now = esphome::millis();
                adaptivePID->update(currentTemperature, now, deviceManager->isInternalPowerOn());
                auto a = learner_->accepted_gains(), b = learner_->candidate_gains();
                auto command = [&](const PidGainLearner::Gains &x) {
                    return std::round(devicestate::clamp(adaptivePID->preview(x[0],x[1],x[2]), low, high)*2.f)/2.f;
                };
                auto sample = learning_sample_;
                sample.temperature = currentTemperature;
                sample.target = deviceManager->getTargetTemperature();
                sample.heating = direction;
                sample.power = deviceManager->isInternalPowerOn();
                sample.operating = deviceManager->getDeviceStatus().operating;
                sample.unit_setpoint = deviceManager->getDeviceState().targetTemperature;
                sample.command_a = command(a); sample.command_b = command(b);
                sample.enabled &= direction == learning_heating_ && learning_mode_selected_;
                learner_->observe(sample, now);
                if (learner_->testing_candidate()) {
                    const auto accepted = learner_->accepted_gains();
                    const auto trial = learner_->gains();
                    if (std::fabs(command(accepted)-command(trial)) > learner_->command_limit()+.001f)
                        learner_->abort("inconclusive_command_bound");
                }
                g = (direction == learning_heating_ && learning_mode_selected_) ? learner_->gains() : configured_;
                adaptivePID->set_gains(g[0],g[1],g[2]);
                const float setpoint = devicestate::clamp(adaptivePID->preview(g[0],g[1],g[2]), low, high);
                if (sample.power || updatedPidTarget) {
                    if (deviceManager->internalSetCorrectedTemperature(setpoint)) deviceManager->commit();
                }
                return;
            }
            // Legacy path remains unchanged for installations without gain_learning.
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
void workflow::pid::PidWorkflowStep::attach_learner(PidGainLearner *learner, bool heating, PidGainLearner::Gains configured) {
    configured_ = configured;
    learner_ = learner;
    learning_heating_ = heating;
    adaptivePID->enable_adaptation(false);
    adaptivePID->set_bounds(0, 250, 0, 10, 0, 7200);
    const auto gains = learner ? learner->accepted_gains() : configured_;
    adaptivePID->set_gains(gains[0],gains[1],gains[2]);
    adaptivePID->set_derivative_filter_seconds(30.f);
    adaptivePID->reset_history();
}

void workflow::pid::PidWorkflowStep::pause_learning() {
    if (learner_) learner_->pause();
    adaptivePID->reset_history();
}
