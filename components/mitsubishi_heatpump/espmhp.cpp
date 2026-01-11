/**
 * espmhp.cpp
 *
 * Implementation of esphome-mitsubishiheatpump
 *
 * Author: Geoff Davis <geoff@geoffdavis.com>
 * Author: Phil Genera @pgenera on Github.
 * Author: Barry Loong @loongyh on GitHub.
 * Author: @am-io on Github.
 * Author: @nao-pon on Github.
 * Author: Simon Knopp @sijk on Github
 * Author: Paul Murphy @donutsoft on GitHub
 * Last Updated: 2023-04-22
 * License: BSD
 *
 * Requirements:
 * - https://github.com/SwiCago/HeatPump
 * - ESPHome 1.18.0 or greater
 */

#include "espmhp.h"
using namespace esphome;

#include "devicestatemanager.h"
using namespace devicestate;

#include "hysterisis_workflowstep.h"
using namespace workflow::hysterisis;

#include "pid_workflowstep.h"
using namespace workflow::pid;

#include "floats.h"

static const char* TAG = "MitsubishiHeatPump"; // Logging tag

/**
 * Create a new MitsubishiHeatPump object
 *
 * Args:
 *   hw_serial: pointer to an Arduino HardwareSerial instance
 *   poll_interval: polling interval in milliseconds
 */
MitsubishiHeatPump::MitsubishiHeatPump(
        uart::UARTComponent* hw_serial,
        uint32_t poll_interval
) : hw_serial_{hw_serial},
    scheduler_(
            // send_callback: envoie un paquet via buildAndSendInfoPacket
            [this](uint8_t code) {
                ESP_LOGI(TAG, "scheduled code: %d", code);
                //this->buildAndSendInfoPacket(code);
            },
            // timeout_callback: utilise set_timeout de Component
            [this](const std::string& name, uint32_t timeout_ms, std::function<void()> callback) {
                //this->set_timeout(name.c_str(), timeout_ms, std::move(callback));
                ESP_LOGI(TAG, "timeout set: %s for %d ms", name.c_str(), timeout_ms);
            },
            // terminate_callback: termine le cycle
            [this]() {
                ESP_LOGI(TAG, "terminate cycle");
                this->terminateCycle();
            }
        ) {

    this->traits_.add_feature_flags(
        climate::CLIMATE_SUPPORTS_ACTION |
        climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE
    );
    this->traits_.set_visual_min_temperature(ESPMHP_MIN_TEMPERATURE);
    this->traits_.set_visual_max_temperature(ESPMHP_MAX_TEMPERATURE);
    this->traits_.set_visual_target_temperature_step(ESPMHP_TARGET_TEMPERATURE_STEP);
    this->traits_.set_visual_current_temperature_step(ESPMHP_CURRENT_TEMPERATURE_STEP);

    this->loopCycle.init();

    // Assume a succesful connection was made to the ESPHome controller on
    // launch.
    this->ping();
}

uint32_t MitsubishiHeatPump::get_update_interval() const { return this->update_interval_; }
void MitsubishiHeatPump::set_update_interval(uint32_t update_interval) {
    //ESP_LOGD(TAG, "Setting update interval to %lu", update_interval);
    log_debug_uint32(TAG, "Setting update interval to ", update_interval);

    this->update_interval_ = update_interval;
    //this->autoUpdate = (update_interval != 0);
}

void MitsubishiHeatPump::terminateCycle() {
    //if (this->shouldSendExternalTemperature_) {
        // We will receive ACK packet for this.
        // Sending WantedSettings must be delayed in this case (lastSend timestamp updated).        
    //    ESP_LOGD(LOG_REMOTE_TEMP, "Sending remote temperature...");
    //    this->sendRemoteTemperature();
    //}

    this->loopCycle.cycleEnded();

    //if (this->hp_uptime_connection_sensor_ != nullptr) {
        // if the uptime connection sensor is configured
        // we trigger  manual update at the end of a cycle.
    //    this->hp_uptime_connection_sensor_->update();
    //}

    //this->nbCompleteCycles_++;
}

void MitsubishiHeatPump::banner() {
    ESP_LOGI(TAG, "ESPHome MitsubishiHeatPump version %s", ESPMHP_VERSION);
}

/**
 * @brief Executes the main loop for the CN105Climate component.
 * This function is called repeatedly in the main program loop.
 */
void MitsubishiHeatPump::loop() {
    // Bootstrap connexion CN105 (UART + CONNECT) depuis loop()
    //this->maybe_start_connection_();

    // Tant que la connexion n'a pas réussi, on ne lance AUCUN cycle/écriture (sinon ça court-circuite le délai).
    // On continue quand même à lire/processer l'input afin de détecter le 0x7A/0x7B (connection success).
    //const bool can_talk_to_hp = this->isHeatpumpConnected_;

    //if (this->processInput()) {
        // if we don't get any input: no read op
    //    return;
    //}

    //if (!can_talk_to_hp) {
    //    return;
    //}

//    if ((this->wantedSettings.hasChanged) && (!this->loopCycle.isCycleRunning())) {
//        this->checkPendingWantedSettings();
//    } else if ((this->wantedRunStates.hasChanged) && (!this->loopCycle.isCycleRunning())) {
//        this->checkPendingWantedRunStates();
//    } else {
    if (this->loopCycle.isCycleRunning()) {                         // if we are  running an update cycle
        this->loopCycle.checkTimeout(this->update_interval_);
    } else { // we are not running a cycle
        if (this->loopCycle.hasUpdateIntervalPassed(this->get_update_interval())) {
            this->loopCycle.cycleStarted();
            //this->buildAndSendRequestsInfoPackets();            // initiate an update cycle with this->cycleStarted();
            //this->update();
            this->scheduler_.send_next_after(0x00); // This will call terminate cycle when complete
        }
    }
    //}
}

void MitsubishiHeatPump::update() {
    // This will be called every "update_interval" milliseconds.
    this->dsm->update();
    if (!this->dsm->isInitialized()) {
        ESP_LOGW(TAG, "Waiting for HeatPump to read the settings the first time.");
        return;
    }

    this->updateDevice();

    this->enforce_remote_temperature_sensor_timeout();
    this->run_workflows();

    this->dsm->publish();
}

void MitsubishiHeatPump::set_baud_rate(int baud) {
    this->baud_ = baud;
}

void MitsubishiHeatPump::set_rx_pin(int rx_pin) {
    this->rx_pin_ = rx_pin;
}

void MitsubishiHeatPump::set_tx_pin(int tx_pin) {
    this->tx_pin_ = tx_pin;
}

/**
 * Get our supported traits.
 *
 * Note:
 * Many of the following traits are only available in the 1.5.0 dev train of
 * ESPHome, particularly the Dry operation mode, and several of the fan modes.
 *
 * Returns:
 *   This class' supported climate::ClimateTraits.
 */
climate::ClimateTraits MitsubishiHeatPump::traits() {
    return traits_;
}

/**
 * Modify our supported traits.
 *
 * Returns:
 *   A reference to this class' supported climate::ClimateTraits.
 */
climate::ClimateTraits& MitsubishiHeatPump::config_traits() {
    return traits_;
}

void MitsubishiHeatPump::update_swing_horizontal(const std::string &swing) {
    this->horizontal_swing_state_ = swing;

    if (this->horizontal_vane_select_ != nullptr &&
        this->horizontal_vane_select_->current_option() != this->horizontal_swing_state_) {
        this->horizontal_vane_select_->publish_state(
            this->horizontal_swing_state_);  // Set current horizontal swing
                                             // position
    }
}

void MitsubishiHeatPump::update_swing_vertical(const std::string &swing) {
    this->vertical_swing_state_ = swing;

    if (this->vertical_vane_select_ != nullptr &&
        this->vertical_vane_select_->current_option() != this->vertical_swing_state_) {
        this->vertical_vane_select_->publish_state(
            this->vertical_swing_state_);  // Set current vertical swing position
    }
}

void MitsubishiHeatPump::set_vertical_vane_select(
    select::Select *vertical_vane_select) {
    this->vertical_vane_select_ = vertical_vane_select;
    this->vertical_vane_select_->add_on_state_callback(
        [this](const std::string &value, size_t index) {
            if (value == this->vertical_swing_state_) return;
            this->on_vertical_swing_change(value);
        });
}

void MitsubishiHeatPump::set_horizontal_vane_select(
    select::Select *horizontal_vane_select) {
      this->horizontal_vane_select_ = horizontal_vane_select;
      this->horizontal_vane_select_->add_on_state_callback(
          [this](const std::string &value, size_t index) {
              if (value == this->horizontal_swing_state_) return;
              this->on_horizontal_swing_change(value);
          });
}

void MitsubishiHeatPump::on_vertical_swing_change(const std::string &swing) {
    ESP_LOGD(TAG, "Setting vertical swing position");
    bool updated = false;

    if (swing == "swing") {
        updated = this->dsm->setVerticalSwingMode(devicestate::VerticalSwingMode::VerticalSwingMode_Swing);
    } else if (swing == "auto") {
        updated = this->dsm->setVerticalSwingMode(devicestate::VerticalSwingMode::VerticalSwingMode_Auto);
    } else if (swing == "up") {
        updated = this->dsm->setVerticalSwingMode(devicestate::VerticalSwingMode::VerticalSwingMode_Up);
    } else if (swing == "up_center") {
        updated = this->dsm->setVerticalSwingMode(devicestate::VerticalSwingMode::VerticalSwingMode_UpCenter);
    } else if (swing == "center") {
        updated = this->dsm->setVerticalSwingMode(devicestate::VerticalSwingMode::VerticalSwingMode_Center);
    } else if (swing == "down_center") {
        updated = this->dsm->setVerticalSwingMode(devicestate::VerticalSwingMode::VerticalSwingMode_DownCenter);
    } else if (swing == "down") {
        updated = this->dsm->setVerticalSwingMode(devicestate::VerticalSwingMode::VerticalSwingMode_Down);
    } else {
        ESP_LOGW(TAG, "Invalid vertical vane position %s", swing);
    }

    ESP_LOGD(TAG, "Vertical vane - Was HeatPump updated? %s", YESNO(updated));
}

void MitsubishiHeatPump::on_horizontal_swing_change(const std::string &swing) {
    ESP_LOGD(TAG, "Setting horizontal swing position");
    bool updated = false;

    if (swing == "swing") {
        updated = this->dsm->setHorizontalSwingMode(devicestate::HorizontalSwingMode::HorizontalSwingMode_Swing);
    } else if (swing == "auto") {
        updated = this->dsm->setHorizontalSwingMode(devicestate::HorizontalSwingMode::HorizontalSwingMode_Auto);
    } else if (swing == "left") {
        updated = this->dsm->setHorizontalSwingMode(devicestate::HorizontalSwingMode::HorizontalSwingMode_Left);
    } else if (swing == "left_center") {
        updated = this->dsm->setHorizontalSwingMode(devicestate::HorizontalSwingMode::HorizontalSwingMode_LeftCenter);
    } else if (swing == "center") {
        updated = this->dsm->setHorizontalSwingMode(devicestate::HorizontalSwingMode::HorizontalSwingMode_Center);
    } else if (swing == "right_center") {
        updated = this->dsm->setHorizontalSwingMode(devicestate::HorizontalSwingMode::HorizontalSwingMode_RightCenter);
    } else if (swing == "right") {
        updated = this->dsm->setHorizontalSwingMode(devicestate::HorizontalSwingMode::HorizontalSwingMode_Right);
    } else {
        ESP_LOGW(TAG, "Invalid horizontal vane position %s", swing);
    }

    ESP_LOGD(TAG, "Horizontal vane - Was HeatPump updated? %s", YESNO(updated));
 }

/**
 * Implement control of a MitsubishiHeatPump.
 *
 * Maps HomeAssistant/ESPHome modes to Mitsubishi modes.
 */
void MitsubishiHeatPump::control(const climate::ClimateCall &call) {
    ESP_LOGW(TAG, "Control called.");

    ESP_LOGW(TAG, "Has target temp: %s", TRUEFALSE(call.get_target_temperature().has_value()));
    ESP_LOGW(TAG, "Has target temp low: %s", TRUEFALSE(call.get_target_temperature_low().has_value()));
    ESP_LOGW(TAG, "Has target temp high: %s", TRUEFALSE(call.get_target_temperature_high().has_value()));

    bool updated = false;
    bool has_mode = call.get_mode().has_value();
    bool has_temp = call.get_target_temperature().has_value();
    if (has_mode){
        this->mode = *call.get_mode();
    }

    if (last_remote_temperature_sensor_update_.has_value()) {
        // Some remote temperature sensors will only issue updates when a change
        // in temperature occurs. 

        // Assume a case where the idle sensor timeout is 12hrs and operating 
        // timeout is 1hr. If the user changes the HP setpoint after 1.5hrs, the
        // machine will switch to operating mode, the remote temperature 
        // reading will expire and the HP will revert to it's internal 
        // temperature sensor.

        // This change ensures that if the user changes the machine setpoint,
        // the remote sensor has an opportunity to issue an update to reflect
        // the new change in temperature.
        last_remote_temperature_sensor_update_ =
            std::chrono::steady_clock::now();
    }

    switch (this->mode) {
        case climate::CLIMATE_MODE_COOL:
            this->dsm->setCool();

            if (has_mode){
                if (cool_setpoint.has_value() && !has_temp) {
                    ESP_LOGW(TAG, "Using cool setpoint: HA %.2f", cool_setpoint.value());
                    this->update_setpoint(cool_setpoint.value());
                }
                this->action = climate::CLIMATE_ACTION_IDLE;
                updated = true;
            }
            break;
        case climate::CLIMATE_MODE_HEAT:
            this->dsm->setHeat();

            if (has_mode){
                if (heat_setpoint.has_value() && !has_temp) {
                    ESP_LOGW(TAG, "Using heat setpoint: HA %.2f", heat_setpoint.value());
                    this->update_setpoint(heat_setpoint.value());
                }
                this->action = climate::CLIMATE_ACTION_IDLE;
                updated = true;
            }
            break;
        case climate::CLIMATE_MODE_DRY:
            this->dsm->setDry();

            if (has_mode){
                this->action = climate::CLIMATE_ACTION_DRYING;
                updated = true;
            }
            break;
        case climate::CLIMATE_MODE_HEAT_COOL:
            this->dsm->setAuto();

            if (has_mode){
                if (auto_setpoint.has_value() && !has_temp) {
                    ESP_LOGW(TAG, "Using auto setpoint: HA %.2f", auto_setpoint.value());
                    this->update_setpoint(auto_setpoint.value());
                }
                this->action = climate::CLIMATE_ACTION_IDLE;
            }
            updated = true;
            break;
        case climate::CLIMATE_MODE_FAN_ONLY:
            this->dsm->setFan();

            if (has_mode){
                this->action = climate::CLIMATE_ACTION_FAN;
                updated = true;
            }
            break;
        case climate::CLIMATE_MODE_OFF:
        default:
            if (has_mode){
                this->dsm->turnOff();
                this->action = climate::CLIMATE_ACTION_OFF;
                updated = true;
            }
            break;
    }

    if (has_temp){
        ESP_LOGW(
            "control", "Sending target temp: %.1f",
            *call.get_target_temperature()
        );
        this->update_setpoint(*call.get_target_temperature());
        updated = true;
    }

    if (call.get_fan_mode().has_value()) {
        ESP_LOGV("control", "Requested fan mode is %s",
                 climate::climate_fan_mode_to_string(*call.get_fan_mode()));
        this->fan_mode = *call.get_fan_mode();

        switch(*call.get_fan_mode()) {
            case climate::CLIMATE_FAN_OFF:
                this->dsm->turnOff();
                updated = true;
                break;
            case climate::CLIMATE_FAN_DIFFUSE:
                this->dsm->setFanMode(devicestate::FanMode::FanMode_Quiet, false);
                updated = true;
                break;
            case climate::CLIMATE_FAN_LOW:
                this->dsm->setFanMode(devicestate::FanMode::FanMode_Low, false);
                updated = true;
                break;
            case climate::CLIMATE_FAN_MEDIUM:
                this->dsm->setFanMode(devicestate::FanMode::FanMode_Medium, false);
                updated = true;
                break;
            case climate::CLIMATE_FAN_MIDDLE:
                this->dsm->setFanMode(devicestate::FanMode::FanMode_Middle, false);
                updated = true;
                break;
            case climate::CLIMATE_FAN_HIGH:
                this->dsm->setFanMode(devicestate::FanMode::FanMode_High, false);
                updated = true;
                break;
            case climate::CLIMATE_FAN_ON:
            case climate::CLIMATE_FAN_AUTO:
            default:
                this->dsm->setFanMode(devicestate::FanMode::FanMode_Auto, false);
                updated = true;
                break;
        }
    }

    ESP_LOGV(TAG, "in the swing mode stage");
    if (call.get_swing_mode().has_value()) {
        ESP_LOGV(TAG, "control - requested swing mode is %s",
                climate::climate_swing_mode_to_string(*call.get_swing_mode()));

        this->swing_mode = *call.get_swing_mode();
        switch(*call.get_swing_mode()) {
            case climate::CLIMATE_SWING_OFF:
                this->dsm->setVerticalSwingMode(devicestate::VerticalSwingMode::VerticalSwingMode_Auto, false);
                this->dsm->setHorizontalSwingMode(devicestate::HorizontalSwingMode::HorizontalSwingMode_Center, false);
                updated = true;
                break;
            case climate::CLIMATE_SWING_VERTICAL:
                this->dsm->setVerticalSwingMode(devicestate::VerticalSwingMode::VerticalSwingMode_Swing, false);
                this->dsm->setHorizontalSwingMode(devicestate::HorizontalSwingMode::HorizontalSwingMode_Center, false);
                updated = true;
                break;
            case climate::CLIMATE_SWING_HORIZONTAL:
                this->dsm->setVerticalSwingMode(devicestate::VerticalSwingMode::VerticalSwingMode_Center, false);
                this->dsm->setHorizontalSwingMode(devicestate::HorizontalSwingMode::HorizontalSwingMode_Swing, false);
                updated = true;
                break;
            case climate::CLIMATE_SWING_BOTH:
                this->dsm->setVerticalSwingMode(devicestate::VerticalSwingMode::VerticalSwingMode_Swing, false);
                this->dsm->setHorizontalSwingMode(devicestate::HorizontalSwingMode::HorizontalSwingMode_Swing, false);
                updated = true;
                break;
            default:
                ESP_LOGW(TAG, "control - received unsupported swing mode request.");

        }
    }
    ESP_LOGD(TAG, "control - Was HeatPump updated? %s", YESNO(updated));

    // send the update back to esphome:
    this->publish_state();

    if (updated) {
        // and the heat pump:
        if (!this->dsm->commit()) {
            ESP_LOGW(TAG, "Failed to update device state");
        }
    }
}

void MitsubishiHeatPump::updateDevice() {
    const DeviceState deviceState = this->dsm->getDeviceState();
    const DeviceStatus deviceStatus = this->dsm->getDeviceStatus();
    if (devicestate::deviceStateEqual(this->lastDeviceState, deviceState) &&
            devicestate::deviceStatusEqual(this->lastDeviceStatus, deviceStatus) &&
            !this->remote_temperature_updated) {
        ESP_LOGD(TAG, "Skipping updateDevice due to no change");
        return;
    }

    ESP_LOGI(TAG, "Running updateDevice...");
    this->remote_temperature_updated = false;
    this->lastDeviceState = deviceState;
    this->lastDeviceStatus = deviceStatus;

    if (this->remote_temperature > 0) {
        ESP_LOGV(TAG, "Current: Using remote temperature %.2f", this->remote_temperature);
        this->current_temperature = this->remote_temperature;
    } else {
        ESP_LOGV(TAG, "Current: Using device temperature %.2f", deviceStatus.currentTemperature);
        this->current_temperature = deviceStatus.currentTemperature;
    }
    ESP_LOGI(TAG, "Current device temperature: %.2f", this->current_temperature);
    this->operating_ = deviceStatus.operating;

    // We cannot use the internal state of the device initialize component state
    if (this->isComponentActive()) {
        switch (deviceState.mode) {
            case DeviceMode::DeviceMode_Heat:
                this->mode = climate::CLIMATE_MODE_HEAT;
                if (!this->isInitialized && heat_setpoint.has_value()) {
                    ESP_LOGW(TAG, "Initializing target temp with Heat Setpoint: %.2f", heat_setpoint.value());
                    this->update_setpoint(heat_setpoint.value());
                    this->isInitialized = true;
                } else {
                    if (!devicestate::same_float(heat_setpoint.value(), this->dsm->getTargetTemperature(), 0.01f)) {
                        ESP_LOGW(TAG, "Heat Setpoint diff: HA %.2f / HP %.2f", heat_setpoint.value(), this->dsm->getTargetTemperature());
                        heat_setpoint = this->dsm->getTargetTemperature();
                        save(this->dsm->getTargetTemperature(), heat_storage);
                    }
                }

                if (deviceStatus.operating) {
                    this->action = climate::CLIMATE_ACTION_HEATING;
                } else {
                    if (this->dsm->isInternalPowerOn()) {
                        this->action = climate::CLIMATE_ACTION_IDLE;
                    } else {
                        this->action = climate::CLIMATE_ACTION_OFF;
                    }
                }
                break;
            case DeviceMode::DeviceMode_Dry:
                this->mode = climate::CLIMATE_MODE_DRY;
                if (deviceStatus.operating) {
                    this->action = climate::CLIMATE_ACTION_DRYING;
                } else {
                    if (this->dsm->isInternalPowerOn()) {
                        this->action = climate::CLIMATE_ACTION_IDLE;
                    } else {
                        this->action = climate::CLIMATE_ACTION_OFF;
                    }
                }
                break;
            case DeviceMode::DeviceMode_Cool:
                this->mode = climate::CLIMATE_MODE_COOL;
                if (!this->isInitialized && cool_setpoint.has_value()) {
                    ESP_LOGW(TAG, "Initializing target temp with Cool Setpoint: %.2f", cool_setpoint.value());
                    this->update_setpoint(cool_setpoint.value());
                    this->isInitialized = true;
                } else {
                    if (!devicestate::same_float(cool_setpoint.value(), this->dsm->getTargetTemperature(), 0.01f)) {
                        ESP_LOGW(TAG, "Cool Setpoint diff: HA %.2f / HP %.2f", cool_setpoint.value(), this->dsm->getTargetTemperature());
                        cool_setpoint = this->dsm->getTargetTemperature();
                        save(this->dsm->getTargetTemperature(), cool_storage);
                    }
                }

                if (deviceStatus.operating) {
                    this->action = climate::CLIMATE_ACTION_COOLING;
                } else {
                    if (this->dsm->isInternalPowerOn()) {
                        this->action = climate::CLIMATE_ACTION_IDLE;
                    } else {
                        this->action = climate::CLIMATE_ACTION_OFF;
                    }
                }
                break;
            case DeviceMode::DeviceMode_Fan:
                this->mode = climate::CLIMATE_MODE_FAN_ONLY;
                this->action = climate::CLIMATE_ACTION_FAN;
                break;
            case DeviceMode::DeviceMode_Auto:
                if (!this->isInitialized && auto_setpoint.has_value()) {
                    ESP_LOGW(TAG, "Initializing target temp with Auto Setpoint: %.2f", auto_setpoint.value());
                    this->update_setpoint(auto_setpoint.value());
                    this->isInitialized = true;
                } else {
                    if (!devicestate::same_float(auto_setpoint.value(), this->dsm->getTargetTemperature(), 0.01f)) {
                        ESP_LOGW(TAG, "Auto Setpoint diff: HA %.2f / HP %.2f", auto_setpoint.value(), this->dsm->getTargetTemperature());
                        auto_setpoint = this->dsm->getTargetTemperature();
                        save(this->dsm->getTargetTemperature(), auto_storage);
                    }
                }

                if (deviceStatus.operating) {
                    if (this->current_temperature > this->target_temperature) {
                        this->action = climate::CLIMATE_ACTION_COOLING;
                    } else if (this->current_temperature < this->target_temperature) {
                        this->action = climate::CLIMATE_ACTION_HEATING;
                    }
                } else {
                    if (this->dsm->isInternalPowerOn()) {
                        this->action = climate::CLIMATE_ACTION_IDLE;
                    } else {
                        this->action = climate::CLIMATE_ACTION_OFF;
                    }
                }

                break;
            default:
                ESP_LOGW(
                        TAG,
                        "Unknown climate mode value %s received from HeatPump",
                        devicestate::deviceModeToString(deviceState.mode)
                );
                if (this->dsm->isInternalPowerOn()) {
                    this->action = climate::CLIMATE_ACTION_IDLE;
                } else {
                    this->action = climate::CLIMATE_ACTION_OFF;
                }
        }
    } else {
        this->mode = climate::CLIMATE_MODE_OFF;
        this->action = climate::CLIMATE_ACTION_OFF;
    }

    ESP_LOGD(TAG, "Climate mode is: %d", this->mode);
    switch (deviceState.fanMode) {
        case FanMode::FanMode_Quiet:
            this->fan_mode = climate::CLIMATE_FAN_DIFFUSE;
        case FanMode::FanMode_Low:
            this->fan_mode = climate::CLIMATE_FAN_LOW;
        case FanMode::FanMode_Medium:
            this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
        case FanMode::FanMode_Middle:
            this->fan_mode = climate::CLIMATE_FAN_MIDDLE;
        case FanMode::FanMode_High:
            this->fan_mode = climate::CLIMATE_FAN_HIGH;
        default:
            this->fan_mode = climate::CLIMATE_FAN_AUTO;
    }
    ESP_LOGD(TAG, "Fan mode is: %d", this->fan_mode.value_or(-1));

    switch (deviceState.swingMode) {
        case SwingMode::SwingMode_Both:
            this->swing_mode = climate::CLIMATE_SWING_BOTH;
        case SwingMode::SwingMode_Vertical:
            this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
        case SwingMode::SwingMode_Horizontal:
            this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
        case SwingMode::SwingMode_Off:
            this->swing_mode = climate::CLIMATE_SWING_OFF;
    }
    ESP_LOGD(TAG, "Swing mode is: %d", this->swing_mode);

    switch(deviceState.verticalSwingMode) {
        case VerticalSwingMode::VerticalSwingMode_Swing:
            this->update_swing_vertical("swing");
        case VerticalSwingMode::VerticalSwingMode_Auto:
            this->update_swing_vertical("auto");
        case VerticalSwingMode::VerticalSwingMode_Up:
            this->update_swing_vertical("up");
        case VerticalSwingMode::VerticalSwingMode_UpCenter:
            this->update_swing_vertical("up_center");
        case VerticalSwingMode::VerticalSwingMode_Center:
            this->update_swing_vertical("center");
        case VerticalSwingMode::VerticalSwingMode_DownCenter:
            this->update_swing_vertical("down_center");
        case VerticalSwingMode::VerticalSwingMode_Down:
            this->update_swing_vertical("down");
        default:
            break;
    }
    ESP_LOGD(TAG, "Vertical vane mode is: %s", verticalSwingModeToString(deviceState.verticalSwingMode));

    switch(deviceState.horizontalSwingMode) {
        case HorizontalSwingMode::HorizontalSwingMode_Swing:
            this->update_swing_horizontal("swing");
        case HorizontalSwingMode::HorizontalSwingMode_Auto:
            this->update_swing_horizontal("auto");
        case HorizontalSwingMode::HorizontalSwingMode_Left:
            this->update_swing_horizontal("left");
        case HorizontalSwingMode::HorizontalSwingMode_LeftCenter:
            this->update_swing_horizontal("left_center");
        case HorizontalSwingMode::HorizontalSwingMode_Center:
            this->update_swing_horizontal("center");
        case HorizontalSwingMode::HorizontalSwingMode_RightCenter:
            this->update_swing_horizontal("right_center");
        case HorizontalSwingMode::HorizontalSwingMode_Right:
            this->update_swing_horizontal("right");
        default:
            break;
    }
    ESP_LOGD(TAG, "Horizontal vane mode is: %s", horizontalSwingModeToString(deviceState.horizontalSwingMode));

    this->target_temperature = this->dsm->getTargetTemperature();

    /*
     * ******** Publish state back to ESPHome. ********
     */
    this->publish_state();
}

void MitsubishiHeatPump::set_remote_temperature(float temp) {
    ESP_LOGI(TAG, "Setting remote temp: %.2f", temp);
    if (temp > 0) {
        last_remote_temperature_sensor_update_ = 
            std::chrono::steady_clock::now();
    } else {
        last_remote_temperature_sensor_update_.reset();
    }

    this->remote_temperature_updated =
        !devicestate::same_float(temp, this->remote_temperature, 0.001f);
    if (this->remote_temperature_updated) {
        this->remote_temperature = temp;
    }
    this->dsm->setRemoteTemperature(temp);
    this->publish_state();
}

void MitsubishiHeatPump::ping() {
    ESP_LOGD(TAG, "Ping request received");
    last_ping_request_ = std::chrono::steady_clock::now();
}

void MitsubishiHeatPump::set_remote_operating_timeout_minutes(int minutes) {
    ESP_LOGD(TAG, "Setting remote operating timeout time: %d minutes", minutes);
    remote_operating_timeout_ = std::chrono::minutes(minutes);
}

void MitsubishiHeatPump::set_remote_idle_timeout_minutes(int minutes) {
    ESP_LOGD(TAG, "Setting remote idle timeout time: %d minutes", minutes);
    remote_idle_timeout_ = std::chrono::minutes(minutes);
}

void MitsubishiHeatPump::set_remote_ping_timeout_minutes(int minutes) {
    ESP_LOGD(TAG, "Setting remote ping timeout time: %d minutes", minutes);
    remote_ping_timeout_ = std::chrono::minutes(minutes);
}

void MitsubishiHeatPump::enforce_remote_temperature_sensor_timeout() {
    // Handle ping timeouts.
    if (remote_ping_timeout_.has_value() && last_ping_request_.has_value()) {
        auto time_since_last_ping =
            std::chrono::steady_clock::now() - last_ping_request_.value();
        if(time_since_last_ping > remote_ping_timeout_.value()) {
            ESP_LOGW(TAG, "Ping timeout.");
            this->set_remote_temperature(0);
            last_ping_request_.reset();
            return;
        }
    }

    // Handle set_remote_temperature timeouts.
    auto remote_set_temperature_timeout =
        this->operating_ ? remote_operating_timeout_ : remote_idle_timeout_;
    if (remote_set_temperature_timeout.has_value() &&
            last_remote_temperature_sensor_update_.has_value()) {
        auto time_since_last_temperature_update =
            std::chrono::steady_clock::now() - last_remote_temperature_sensor_update_.value();
        if (time_since_last_temperature_update > remote_set_temperature_timeout.value()) {
            ESP_LOGW(TAG, "Set remote temperature timeout, operating=%d", this->operating_);
            this->set_remote_temperature(0);
            return;
        }
    }
}

void MitsubishiHeatPump::setup() {
    // This will be called by App.setup()
    this->banner();
    ESP_LOGCONFIG(TAG, "Setting up UART...");

    devicestate::ConnectionMetadata connectionMetadata;
    connectionMetadata.hardwareSerial = this->get_hw_serial_();

    this->min_temp = ESPMHP_MIN_TEMPERATURE;
    if (this->visual_min_temperature_override_.has_value()) {
        this->min_temp = this->visual_min_temperature_override_.value();
    }
    this->max_temp = ESPMHP_MAX_TEMPERATURE;
    if (this->visual_max_temperature_override_.has_value()) {
        this->max_temp = this->visual_max_temperature_override_.value();
    }

    this->hysterisisWorkflowStep = new HysterisisWorkflowStep(
        this->hysterisisOn_,
        this->hysterisisOff_
    );

    this->pidWorkflowStep = new PidWorkflowStep(
        this->get_update_interval(),
        this->min_temp,
        this->max_temp,
        this->kp_,
        this->ki_,
        this->kd_,
        this->maxAdjustmentUnder_,
        this->maxAdjustmentOver_
    );

    ESP_LOGCONFIG(TAG, "Initializing new HeatPump object.");
    this->dsm = new devicestate::DeviceStateManager(
        connectionMetadata,
        this->min_temp,
        this->max_temp,
        this->internal_power_on,
        this->device_state_connected,
        this->device_state_active,
        this->device_set_point,
        this->device_status_operating,
        this->device_status_current_temperature,
        this->device_status_compressor_frequency,
        this->device_status_input_power,
        this->device_status_kwh,
        this->device_status_runtime_hours,
        this->pid_set_point_correction
    );

    ESP_LOGCONFIG(TAG, "Calling dsm->initialize()");
    this->dsm->initialize();

    // create various setpoint persistence:
    cool_storage = global_preferences->make_preference<uint8_t>(this->get_object_id_hash() + 1);
    heat_storage = global_preferences->make_preference<uint8_t>(this->get_object_id_hash() + 2);
    auto_storage = global_preferences->make_preference<uint8_t>(this->get_object_id_hash() + 3);

    const float defaultMidTemp = (this->max_temp + this->min_temp) / 2.0;
    // load values from storage:
    cool_setpoint = esphome::make_optional(load(cool_storage).value_or(defaultMidTemp));
    heat_setpoint = esphome::make_optional(load(heat_storage).value_or(defaultMidTemp));
    auto_setpoint = esphome::make_optional(load(auto_storage).value_or(defaultMidTemp));

    auto restore = this->restore_state_();
    if (restore.has_value()) {
        restore->apply(this);
    } else {
        this->current_temperature = NAN;
        this->target_temperature = NAN;
        this->fan_mode = climate::CLIMATE_FAN_OFF;
        this->swing_mode = climate::CLIMATE_SWING_OFF;
        this->vertical_swing_state_ = "auto";
        this->horizontal_swing_state_ = "auto";
    }

    this->dump_config();
}

/**
 * The ESP only has a few bytes of rtc storage, so instead
 * of storing floats directly, we'll store the number of
 * TEMPERATURE_STEPs from MIN_TEMPERATURE.
 **/
void MitsubishiHeatPump::save(float value, ESPPreferenceObject& storage) {
    uint8_t steps = (value - ESPMHP_MIN_TEMPERATURE) / ESPMHP_CURRENT_TEMPERATURE_STEP;
    storage.save(&steps);
}

esphome::optional<float> MitsubishiHeatPump::load(ESPPreferenceObject& storage) {
    uint8_t steps = 0;
    if (!storage.load(&steps)) {
        return {};
    }
    return ESPMHP_MIN_TEMPERATURE + (steps * ESPMHP_CURRENT_TEMPERATURE_STEP);
}

void MitsubishiHeatPump::dump_config() {
    this->banner();
    ESP_LOGI(TAG, "  Supports HEAT: %s", YESNO(true));
    ESP_LOGI(TAG, "  Supports COOL: %s", YESNO(true));
    ESP_LOGI(TAG, "  Supports AWAY mode: %s", YESNO(false));
    ESP_LOGI(TAG, "  Min temp: %.1f", this->min_temp);
    ESP_LOGI(TAG, "  Max temp: %.1f", this->max_temp);
    ESP_LOGI(TAG, "  Saved heat: %.1f", heat_setpoint.value_or(-1));
    ESP_LOGI(TAG, "  Saved cool: %.1f", cool_setpoint.value_or(-1));
    ESP_LOGI(TAG, "  Saved auto: %.1f", auto_setpoint.value_or(-1));
    ESP_LOGI(TAG, "  Update interval: %d", this->get_update_interval());
}

void MitsubishiHeatPump::dump_state() {
    LOG_CLIMATE("", "MitsubishiHeatPump Climate", this);
    ESP_LOGI(TAG, "HELLO");
}

bool MitsubishiHeatPump::isComponentActive() {
    return this->mode != climate::CLIMATE_MODE_OFF;
}

void MitsubishiHeatPump::update_setpoint(const float value) {
    const float oldTargetTemperature = this->target_temperature;
    this->dsm->setTargetTemperature(value);
    this->target_temperature = this->dsm->getTargetTemperature();
    ESP_LOGI(TAG, "Target temp changed from %f to %f", oldTargetTemperature, this->target_temperature);

    const DeviceState deviceState = this->dsm->getDeviceState();
    switch (deviceState.mode) {
        case DeviceMode::DeviceMode_Heat:
            save(value, heat_storage);
            ESP_LOGW(TAG, "Saved new heat setting: %.1f", value);
            break;
        case DeviceMode::DeviceMode_Cool:
            save(value, cool_storage);
            ESP_LOGW(TAG, "Saved new cool setting: %.1f", value);
            break;
        case DeviceMode::DeviceMode_Auto:
            save(value, auto_storage);
            ESP_LOGW(TAG, "Saved new auto setting: %.1f", value);
            break;
        default:
            ESP_LOGW(TAG, "Didn't save temperature: %.1f", value);
    }
}

void MitsubishiHeatPump::run_workflows() {
    if (!this->isComponentActive()) {
        ESP_LOGW(TAG, "Skipping run workflow due to inactive state.");
        return;
    }

    ESP_LOGI(TAG, "Run workflows - currentTemperature: %.2f", this->current_temperature);
    this->hysterisisWorkflowStep->run(this->current_temperature, this->dsm);
    this->pidWorkflowStep->run(this->current_temperature, this->dsm);
}