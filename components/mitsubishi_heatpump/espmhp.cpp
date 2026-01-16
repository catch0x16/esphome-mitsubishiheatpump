#include "espmhp.h"
using namespace esphome;

#include "devicestatemanager.h"
#include "cn105_connection.h"
#include "cn105_state.h"
#include "io_device.h"
#include "uart_io_device.h"
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
) : hw_serial_{hw_serial} {

    this->traits_.add_feature_flags(
        climate::CLIMATE_SUPPORTS_ACTION |
        climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE
    );
    this->traits_.set_visual_min_temperature(ESPMHP_MIN_TEMPERATURE);
    this->traits_.set_visual_max_temperature(ESPMHP_MAX_TEMPERATURE);
    this->traits_.set_visual_target_temperature_step(ESPMHP_TARGET_TEMPERATURE_STEP);
    this->traits_.set_visual_current_temperature_step(ESPMHP_CURRENT_TEMPERATURE_STEP);

    this->loopCycle.init();
}

uint32_t MitsubishiHeatPump::get_update_interval() const { return this->update_interval_; }
void MitsubishiHeatPump::set_update_interval(uint32_t update_interval) {
    log_debug_uint32(TAG, "Setting update interval to ", update_interval);

    this->update_interval_ = update_interval;
    this->loopCycle.setUpdateInterval(this->update_interval_);
}

void MitsubishiHeatPump::terminateCycle() {
    ESP_LOGW(TAG, "Terminate cycle start");
    this->hpControlFlow_->completeCycle();

    this->dsm->update();
    if (this->dsm->isInitialized()) {
        this->updateDevice();

        this->run_workflows();
        this->dsm->publish();
    } else {
        ESP_LOGW(TAG, "DeviceStateManager not yet initialized.");
    }

    this->loopCycle.cycleEnded();
    ESP_LOGW(TAG, "Terminate cycle complete");
}

void MitsubishiHeatPump::banner() {
    ESP_LOGI(TAG, "ESPHome MitsubishiHeatPump version %s", ESPMHP_VERSION);
}

/**
 * @brief Executes the main loop for the MitsubishiHeatPump component.
 * This function is called repeatedly in the main program loop.
 */
void MitsubishiHeatPump::loop() {
    this->hpControlFlow_->loop(loopCycle);
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

 void MitsubishiHeatPump::controlDelegate(const climate::ClimateCall &call) {
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
        ESP_LOGW("control", "Requested fan mode is %s",
                 climate::climate_fan_mode_to_string(*call.get_fan_mode()));
        this->fan_mode = *call.get_fan_mode();

        switch(*call.get_fan_mode()) {
            case climate::CLIMATE_FAN_OFF:
                this->dsm->turnOff();
                updated = true;
                break;
            case climate::CLIMATE_FAN_DIFFUSE:
                this->dsm->setFanMode(devicestate::FanMode::FanMode_Quiet);
                updated = true;
                break;
            case climate::CLIMATE_FAN_LOW:
                this->dsm->setFanMode(devicestate::FanMode::FanMode_Low);
                updated = true;
                break;
            case climate::CLIMATE_FAN_MEDIUM:
                this->dsm->setFanMode(devicestate::FanMode::FanMode_Medium);
                updated = true;
                break;
            case climate::CLIMATE_FAN_MIDDLE:
                this->dsm->setFanMode(devicestate::FanMode::FanMode_Middle);
                updated = true;
                break;
            case climate::CLIMATE_FAN_HIGH:
                this->dsm->setFanMode(devicestate::FanMode::FanMode_High);
                updated = true;
                break;
            case climate::CLIMATE_FAN_ON:
            case climate::CLIMATE_FAN_AUTO:
            default:
                this->dsm->setFanMode(devicestate::FanMode::FanMode_Auto);
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
                this->dsm->setVerticalSwingMode(devicestate::VerticalSwingMode::VerticalSwingMode_Auto);
                this->dsm->setHorizontalSwingMode(devicestate::HorizontalSwingMode::HorizontalSwingMode_Center);
                updated = true;
                break;
            case climate::CLIMATE_SWING_VERTICAL:
                this->dsm->setVerticalSwingMode(devicestate::VerticalSwingMode::VerticalSwingMode_Swing);
                this->dsm->setHorizontalSwingMode(devicestate::HorizontalSwingMode::HorizontalSwingMode_Center);
                updated = true;
                break;
            case climate::CLIMATE_SWING_HORIZONTAL:
                this->dsm->setVerticalSwingMode(devicestate::VerticalSwingMode::VerticalSwingMode_Center);
                this->dsm->setHorizontalSwingMode(devicestate::HorizontalSwingMode::HorizontalSwingMode_Swing);
                updated = true;
                break;
            case climate::CLIMATE_SWING_BOTH:
                this->dsm->setVerticalSwingMode(devicestate::VerticalSwingMode::VerticalSwingMode_Swing);
                this->dsm->setHorizontalSwingMode(devicestate::HorizontalSwingMode::HorizontalSwingMode_Swing);
                updated = true;
                break;
            default:
                ESP_LOGW(TAG, "control - received unsupported swing mode request.");

        }
    }
    ESP_LOGD(TAG, "control - Was HeatPump updated? %s", YESNO(updated));
    if (updated) {
        this->dsm->commit();
    }

    // send the update back to esphome:
    this->publish_state();
 }

/**
 * Implement control of a MitsubishiHeatPump.
 *
 * Maps HomeAssistant/ESPHome modes to Mitsubishi modes.
 */
void MitsubishiHeatPump::control(const climate::ClimateCall &call) {
    auto callback = [this, &call]() {
        this->controlDelegate(call);
    };

    this->hpControlFlow_->acquireWantedSettingsLock(callback);
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

                ESP_LOGW(TAG, "Setting action...");
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
            break;
        case FanMode::FanMode_Low:
            this->fan_mode = climate::CLIMATE_FAN_LOW;
            break;
        case FanMode::FanMode_Medium:
            this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
            break;
        case FanMode::FanMode_Middle:
            this->fan_mode = climate::CLIMATE_FAN_MIDDLE;
            break;
        case FanMode::FanMode_High:
            this->fan_mode = climate::CLIMATE_FAN_HIGH;
            break;
        default:
            this->fan_mode = climate::CLIMATE_FAN_AUTO;
            break;
    }
    ESP_LOGD(TAG, "Fan mode is: %d", this->fan_mode.value_or(-1));

    switch (deviceState.swingMode) {
        case SwingMode::SwingMode_Both:
            this->swing_mode = climate::CLIMATE_SWING_BOTH;
            break;
        case SwingMode::SwingMode_Vertical:
            this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
            break;
        case SwingMode::SwingMode_Horizontal:
            this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
            break;
        case SwingMode::SwingMode_Off:
            this->swing_mode = climate::CLIMATE_SWING_OFF;
            break;
    }
    ESP_LOGD(TAG, "Swing mode is: %d", this->swing_mode);

    switch(deviceState.verticalSwingMode) {
        case VerticalSwingMode::VerticalSwingMode_Swing:
            this->update_swing_vertical("swing");
            break;
        case VerticalSwingMode::VerticalSwingMode_Auto:
            this->update_swing_vertical("auto");
            break;
        case VerticalSwingMode::VerticalSwingMode_Up:
            this->update_swing_vertical("up");
            break;
        case VerticalSwingMode::VerticalSwingMode_UpCenter:
            this->update_swing_vertical("up_center");
            break;
        case VerticalSwingMode::VerticalSwingMode_Center:
            this->update_swing_vertical("center");
            break;
        case VerticalSwingMode::VerticalSwingMode_DownCenter:
            this->update_swing_vertical("down_center");
            break;
        case VerticalSwingMode::VerticalSwingMode_Down:
            this->update_swing_vertical("down");
            break;
        default:
            break;
    }
    ESP_LOGD(TAG, "Vertical vane mode is: %s", verticalSwingModeToString(deviceState.verticalSwingMode));

    switch(deviceState.horizontalSwingMode) {
        case HorizontalSwingMode::HorizontalSwingMode_Swing:
            this->update_swing_horizontal("swing");
            break;
        case HorizontalSwingMode::HorizontalSwingMode_Auto:
            this->update_swing_horizontal("auto");
            break;
        case HorizontalSwingMode::HorizontalSwingMode_Left:
            this->update_swing_horizontal("left");
            break;
        case HorizontalSwingMode::HorizontalSwingMode_LeftCenter:
            this->update_swing_horizontal("left_center");
            break;
        case HorizontalSwingMode::HorizontalSwingMode_Center:
            this->update_swing_horizontal("center");
            break;
        case HorizontalSwingMode::HorizontalSwingMode_RightCenter:
            this->update_swing_horizontal("right_center");
            break;
        case HorizontalSwingMode::HorizontalSwingMode_Right:
            this->update_swing_horizontal("right");
            break;
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

    this->hpControlFlow_->setRemoteTemperature(temp);
    this->publish_state();
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

void MitsubishiHeatPump::setup() {
    // This will be called by App.setup()
    this->banner();
    ESP_LOGCONFIG(TAG, "Setting up UART...");

    this->min_temp = ESPMHP_MIN_TEMPERATURE;
    if (this->visual_min_temperature_override_.has_value()) {
        this->min_temp = this->visual_min_temperature_override_.value();
    }
    this->max_temp = ESPMHP_MAX_TEMPERATURE;
    if (this->visual_max_temperature_override_.has_value()) {
        this->max_temp = this->visual_max_temperature_override_.value();
    }

    this->hysterisisWorkflowStep = new (std::nothrow) HysterisisWorkflowStep(
        this->hysterisisOn_,
        this->hysterisisOff_
    );
    if (this->hysterisisWorkflowStep == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate HysterisisWorkflowStep");
        this->mark_failed();
        return;
    }

    this->pidWorkflowStep = new (std::nothrow) PidWorkflowStep(
        this->get_update_interval(),
        this->min_temp,
        this->max_temp,
        this->kp_,
        this->ki_,
        this->kd_,
        this->maxAdjustmentUnder_,
        this->maxAdjustmentOver_
    );
    if (this->pidWorkflowStep == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate PidWorkflowStep");
        this->mark_failed();
        return;
    }

    IIODevice* io_device = new (std::nothrow) UARTIODevice(
        this->get_hw_serial_()
    );
    if (io_device == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate UARTIODevice");
        this->mark_failed();
        return;
    }

    auto timeoutCallback = [this](const std::string& name, uint32_t timeout_ms, std::function<void()> callback) {
        this->set_timeout(name.c_str(), timeout_ms, std::move(callback));
    };

    auto retryCallback = [this](const std::string& name, uint32_t initial_wait_time, uint8_t max_attempts, std::function<esphome::RetryResult(uint8_t)> callback) {
        this->set_retry(name.c_str(), initial_wait_time, max_attempts, std::move(callback), 1.2f);
    };

    auto terminateCallback = [this]() {
        this->terminateCycle();
    };

    this->hpState_ = new (std::nothrow) CN105State();
    if (this->hpState_ == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate CN105State");
        this->mark_failed();
        return;
    }
    auto loopCycle = this->loopCycle;
    auto connectedCallback = [this, &loopCycle](bool state) {
        if (state) {
            // let's say that the last complete cycle was over now
            loopCycle.lastCompleteCycleMs = CUSTOM_MILLIS;
            this->hpState_->resetCurrentSettings();
            this->hpState_->resetCurrentRunStates();
            //this->hpState_->getCurrentSettings().resetSettings();
            //this->hpState_->getCurrentRunStates().resetSettings();
            ESP_LOGI(TAG, "Reset current settings..");
        }
        this->device_state_connected->publish_state(state);
    };

    CN105Connection* hpConnection = new (std::nothrow) CN105Connection(
        io_device,
        timeoutCallback,
        connectedCallback,
        this->get_update_interval()
    );
    if (hpConnection == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate CN105Connection");
        this->mark_failed();
        return;
    }

    this->hpControlFlow_ = new (std::nothrow) CN105ControlFlow(
        hpConnection,
        this->hpState_,
        timeoutCallback,
        terminateCallback,
        retryCallback
    );
    if (this->hpControlFlow_ == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate CN105ControlFlow");
        this->mark_failed();
        return;
    }

    this->hpState_->getWantedSettings().resetSettings();
    this->hpState_->getWantedSettings().resetSettings();

    ESP_LOGCONFIG(TAG, "Initializing new HeatPump object.");
    this->dsm = new (std::nothrow) devicestate::DeviceStateManager(
        io_device,
        this->hpState_,
        this->min_temp,
        this->max_temp,
        this->internal_power_on,
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
    if (this->dsm == nullptr) {
        ESP_LOGE(TAG, "Failed to allocate DeviceStateManager");
        this->mark_failed();
        return;
    }

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

    this->hpControlFlow_->registerInfoRequests();
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