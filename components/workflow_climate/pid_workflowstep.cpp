#include "pid_workflowstep.h"

#include <new>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"
using namespace esphome;

#include "floats.h"
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
            const TuningConfig &tuning
        ) {
            this->minTemp = minTemp;
            this->maxTemp = maxTemp;
            this->maxAdjustmentUnder = maxAdjustmentUnder;
            this->maxAdjustmentOver = maxAdjustmentOver;

            this->tuning = tuning;
            this->initialKi = i;
            this->adaptivePID = new AdaptivePID(p, i, d);

            if (this->tuning.adaptive) {
                // A configured kp is the starting point: it implies a response
                // rate, which measurements then refine. Without one the
                // estimator's default rate is used.
                if (p > 0.0f) {
                    const float rate = rate_from_kp(p, this->tuning);
                    this->estimator.set_rate(true, rate);
                    this->estimator.set_rate(false, rate);
                }
                this->applyTunedGains(true);
            }
        }

        void PidWorkflowStep::applyTunedGains(bool heating) {
            const Gains gains = gains_from_rate(this->estimator.rate(heating), this->tuning);
            // A configured ki is kept until the first measurement or restored rate.
            const bool keepConfiguredKi = this->initialKi > 0.0f && this->estimator.measurements() == 0;
            this->adaptivePID->set_gains(gains.kp, keepConfiguredKi ? this->initialKi : gains.ki);
        }

        void PidWorkflowStep::restoreResponseRate(bool heating, float rate) {
            this->estimator.set_rate(heating, rate);
            this->initialKi = 0.0f;
            this->applyTunedGains(this->heating);
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

            if (this->tuning.adaptive) {
                // Learn how fast the room responds while the device is actively
                // heating or cooling, and retune from it.
                const bool nowHeating = deviceManager->getOffsetDirection();
                const bool operating = deviceManager->isInternalPowerOn() && deviceManager->getDeviceStatus().operating;
                const bool measured = this->estimator.update(currentTemperature, operating, nowHeating, esphome::millis());
                if (measured || nowHeating != this->heating) {
                    this->heating = nowHeating;
                    this->applyTunedGains(nowHeating);
                    if (measured) {
                        ESP_LOGI(TAG, "Tuned from a measured %s rate of %.2f C/h: kp: %.3f  ki: %.5f  kd: %.3f",
                            nowHeating ? "heating" : "cooling", this->estimator.rate(nowHeating) * 3600.0f,
                            this->adaptivePID->kp(), this->adaptivePID->ki(), this->adaptivePID->kd());
                    }
                }
            }

            // if pid target is not updated and internal power is not on
            if (updatedPidTarget || deviceManager->isInternalPowerOn()) {
                const float adjustMinOffset = deviceManager->getOffsetDirection()
                    ? this->maxAdjustmentUnder
                    : this->maxAdjustmentOver;
                const float adjustMaxOffset = deviceManager->getOffsetDirection()
                    ? this->maxAdjustmentOver
                    : this->maxAdjustmentUnder;
                const float target = deviceManager->getTargetTemperature();
                const float adjustedMinTemp = devicestate::clamp(target - adjustMinOffset, this->minTemp, this->maxTemp);
                const float adjustedMaxTemp = devicestate::clamp(target + adjustMaxOffset, this->minTemp, this->maxTemp);

                ESP_LOGI(TAG, "PidWorkflowStep (Device) target={%.2f} adjustedMin={%.2f} adjustedMax={%.2f} powerOn={%s}",
                    this->adaptivePID->get_target(), adjustedMinTemp, adjustedMaxTemp, YESNO(deviceManager->isInternalPowerOn()));

                unsigned long now = esphome::millis();

                // The PID knows the range it may use, so its integral stops
                // accumulating once the setpoint is pinned at either end.
                this->adaptivePID->set_output_limits(adjustedMinTemp - target, adjustedMaxTemp - target);
                const float setPointCorrectionSimple =
                    this->adaptivePID->update(currentTemperature, now, deviceManager->isInternalPowerOn());
                const float adjustedSetPointCorrectionSimple = devicestate::clamp(setPointCorrectionSimple, adjustedMinTemp, adjustedMaxTemp);
                ESP_LOGI(TAG, "PidWorkflowStep [Simple] setPointCorrection={%.2f} adjustedSetPoint={%.2f} kp={%.3f} ki={%.4f} kd={%.3f}",
                    setPointCorrectionSimple, adjustedSetPointCorrectionSimple,
                    this->adaptivePID->kp(), this->adaptivePID->ki(), this->adaptivePID->kd());

                if (deviceManager->internalSetCorrectedTemperature(
                        adjustedSetPointCorrectionSimple, adjustedMinTemp, adjustedMaxTemp)) {
                    deviceManager->commit();
                }
            }
        }

    }
}