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

#include "devicestate.h"
using namespace devicestate;

/**
 * Create a new MitsubishiHeatPump object
 *
 * Args:
 *   hw_serial: pointer to an Arduino HardwareSerial instance
 *   poll_interval: polling interval in milliseconds
 */
MitsubishiHeatPump::MitsubishiHeatPump(
        HardwareSerial* hw_serial,
        uint32_t poll_interval
) :
    PollingComponent{poll_interval}, // member initializers list
    hw_serial_{hw_serial}
{
    internal_power_on = new esphome::binary_sensor::BinarySensor();
    device_state_active = new esphome::binary_sensor::BinarySensor();
    device_state_initialized = new esphome::binary_sensor::BinarySensor();
    device_status_operating = new esphome::binary_sensor::BinarySensor();
    pid_set_point_correction = new esphome::sensor::Sensor();

    this->traits_.set_supports_action(true);
    this->traits_.set_supports_current_temperature(true);
    this->traits_.set_supports_two_point_target_temperature(false);
    this->traits_.set_visual_min_temperature(ESPMHP_MIN_TEMPERATURE);
    this->traits_.set_visual_max_temperature(ESPMHP_MAX_TEMPERATURE);
    this->traits_.set_visual_target_temperature_step(ESPMHP_TARGET_TEMPERATURE_STEP);
    this->traits_.set_visual_current_temperature_step(ESPMHP_CURRENT_TEMPERATURE_STEP);

    // Assume a succesful connection was made to the ESPHome controller on
    // launch.
    this->ping();
}

bool MitsubishiHeatPump::verify_serial() {
    if (!this->get_hw_serial_()) {
        ESP_LOGCONFIG(
                TAG,
                "No HardwareSerial was provided. "
                "Software serial ports are unsupported by this component."
        );
        return false;
    }

#ifdef USE_LOGGER
    if (this->get_hw_serial_() == logger::global_logger->get_hw_serial()) {
        ESP_LOGW(TAG, "  You're using the same serial port for logging"
                " and the MitsubishiHeatPump component. Please disable"
                " logging over the serial port by setting"
                " logger:baud_rate to 0.");
        return false;
    }
#endif
    // unless something went wrong, assume we have a valid serial configuration
    return true;
}

void MitsubishiHeatPump::banner() {
    ESP_LOGI(TAG, "ESPHome MitsubishiHeatPump version %s",
            ESPMHP_VERSION);
}

void MitsubishiHeatPump::update() {
    // This will be called every "update_interval" milliseconds.
    //this->dump_config();
    this->hp->sync();
#ifndef USE_CALLBACKS
    this->hpSettingsChanged();
    heatpumpStatus currentStatus = hp->getStatus();
    this->hpStatusChanged(currentStatus);
#endif
    this->enforce_remote_temperature_sensor_timeout();
    this->run_workflows();
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
        this->horizontal_vane_select_->state != this->horizontal_swing_state_) {
        this->horizontal_vane_select_->publish_state(
            this->horizontal_swing_state_);  // Set current horizontal swing
                                             // position
    }
}

void MitsubishiHeatPump::update_swing_vertical(const std::string &swing) {
    this->vertical_swing_state_ = swing;

    if (this->vertical_vane_select_ != nullptr &&
        this->vertical_vane_select_->state != this->vertical_swing_state_) {
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
        hp->setVaneSetting("SWING");
        updated = true;
    } else if (swing == "auto") {
        hp->setVaneSetting("AUTO");
        updated = true;
    } else if (swing == "up") {
        hp->setVaneSetting("1");
        updated = true;
    } else if (swing == "up_center") {
        hp->setVaneSetting("2");
        updated = true;
    } else if (swing == "center") {
        hp->setVaneSetting("3");
        updated = true;
    } else if (swing == "down_center") {
        hp->setVaneSetting("4");
        updated = true;
    } else if (swing == "down") {
        hp->setVaneSetting("5");
        updated = true;
    } else {
        ESP_LOGW(TAG, "Invalid vertical vane position %s", swing);
    }

    ESP_LOGD(TAG, "Vertical vane - Was HeatPump updated? %s", YESNO(updated));

    // and the heat pump:
    hp->update();
}

void MitsubishiHeatPump::on_horizontal_swing_change(const std::string &swing) {
    ESP_LOGD(TAG, "Setting horizontal swing position");
    bool updated = false;

    if (swing == "swing") {
        hp->setWideVaneSetting("SWING");
        updated = true;
    } else if (swing == "auto") {
        hp->setWideVaneSetting("<>");
        updated = true;
    } else if (swing == "left") {
        hp->setWideVaneSetting("<<");
        updated = true;
    } else if (swing == "left_center") {
        hp->setWideVaneSetting("<");
        updated = true;
    } else if (swing == "center") {
        hp->setWideVaneSetting("|");
        updated = true;
    } else if (swing == "right_center") {
        hp->setWideVaneSetting(">");
        updated = true;
    } else if (swing == "right") {
        hp->setWideVaneSetting(">>");
        updated = true;
    } else {
        ESP_LOGW(TAG, "Invalid horizontal vane position %s", swing);
    }

    ESP_LOGD(TAG, "Horizontal vane - Was HeatPump updated? %s", YESNO(updated));

    // and the heat pump:
    hp->update();
 }

/**
 * Implement control of a MitsubishiHeatPump.
 *
 * Maps HomeAssistant/ESPHome modes to Mitsubishi modes.
 */
void MitsubishiHeatPump::control(const climate::ClimateCall &call) {
    ESP_LOGV(TAG, "Control called.");

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
            hp->setModeSetting("COOL");
            hp->setPowerSetting("ON");
            this->internalPowerOn = true;
            internal_power_on->publish_state(this->internalPowerOn);

            if (has_mode){
                if (cool_setpoint.has_value() && !has_temp) {
                    hp->setTemperature(cool_setpoint.value());
                    this->update_setpoint(cool_setpoint.value());
                }
                this->action = climate::CLIMATE_ACTION_IDLE;
                updated = true;
            }
            break;
        case climate::CLIMATE_MODE_HEAT:
            hp->setModeSetting("HEAT");
            hp->setPowerSetting("ON");
            this->internalPowerOn = true;
            internal_power_on->publish_state(this->internalPowerOn);

            if (has_mode){
                if (heat_setpoint.has_value() && !has_temp) {
                    hp->setTemperature(heat_setpoint.value());
                    this->update_setpoint(heat_setpoint.value());
                }
                this->action = climate::CLIMATE_ACTION_IDLE;
                updated = true;
            }
            break;
        case climate::CLIMATE_MODE_DRY:
            hp->setModeSetting("DRY");
            hp->setPowerSetting("ON");
            this->internalPowerOn = true;
            internal_power_on->publish_state(this->internalPowerOn);

            if (has_mode){
                this->action = climate::CLIMATE_ACTION_DRYING;
                updated = true;
            }
            break;
        case climate::CLIMATE_MODE_HEAT_COOL:
            hp->setModeSetting("AUTO");
            hp->setPowerSetting("ON");
            this->internalPowerOn = true;
            internal_power_on->publish_state(this->internalPowerOn);

            if (has_mode){
                if (auto_setpoint.has_value() && !has_temp) {
                    hp->setTemperature(auto_setpoint.value());
                    this->update_setpoint(auto_setpoint.value());
                }
                this->action = climate::CLIMATE_ACTION_IDLE;
            }
            updated = true;
            break;
        case climate::CLIMATE_MODE_FAN_ONLY:
            hp->setModeSetting("FAN");
            hp->setPowerSetting("ON");
            this->internalPowerOn = true;
            internal_power_on->publish_state(this->internalPowerOn);

            if (has_mode){
                this->action = climate::CLIMATE_ACTION_FAN;
                updated = true;
            }
            break;
        case climate::CLIMATE_MODE_OFF:
        default:
            if (has_mode){
                hp->setPowerSetting("OFF");
                this->internalPowerOn = false;
                internal_power_on->publish_state(this->internalPowerOn);
                this->action = climate::CLIMATE_ACTION_OFF;
                updated = true;
            }
            break;
    }

    if (has_temp){
        ESP_LOGV(
            "control", "Sending target temp: %.1f",
            *call.get_target_temperature()
        );
        hp->setTemperature(*call.get_target_temperature());
        this->update_setpoint(*call.get_target_temperature());
        updated = true;
    }

    //const char* FAN_MAP[6]         = {"AUTO", "QUIET", "1", "2", "3", "4"};
    if (call.get_fan_mode().has_value()) {
        ESP_LOGV("control", "Requested fan mode is %s",
                 climate::climate_fan_mode_to_string(*call.get_fan_mode()));
        this->fan_mode = *call.get_fan_mode();
        switch(*call.get_fan_mode()) {
            case climate::CLIMATE_FAN_OFF:
                hp->setPowerSetting("OFF");
                updated = true;
                break;
            case climate::CLIMATE_FAN_DIFFUSE:
                hp->setFanSpeed("QUIET");
                updated = true;
                break;
            case climate::CLIMATE_FAN_LOW:
                hp->setFanSpeed("1");
                updated = true;
                break;
            case climate::CLIMATE_FAN_MEDIUM:
                hp->setFanSpeed("2");
                updated = true;
                break;
            case climate::CLIMATE_FAN_MIDDLE:
                hp->setFanSpeed("3");
                updated = true;
                break;
            case climate::CLIMATE_FAN_HIGH:
                hp->setFanSpeed("4");
                updated = true;
                break;
            case climate::CLIMATE_FAN_ON:
            case climate::CLIMATE_FAN_AUTO:
            default:
                hp->setFanSpeed("AUTO");
                updated = true;
                break;
        }
    }


    ESP_LOGV(TAG, "in the swing mode stage");
    //const char* VANE_MAP[7]        = {"AUTO", "1", "2", "3", "4", "5", "SWING"};
    if (call.get_swing_mode().has_value()) {
        ESP_LOGV(TAG, "control - requested swing mode is %s",
                climate::climate_swing_mode_to_string(*call.get_swing_mode()));

        this->swing_mode = *call.get_swing_mode();
        switch(*call.get_swing_mode()) {
            case climate::CLIMATE_SWING_OFF:
                hp->setVaneSetting("AUTO");
                hp->setWideVaneSetting("|");
                updated = true;
                break;
            case climate::CLIMATE_SWING_VERTICAL:
                hp->setVaneSetting("SWING");
                hp->setWideVaneSetting("|");
                updated = true;
                break;
            case climate::CLIMATE_SWING_HORIZONTAL:
                hp->setVaneSetting("3");
                hp->setWideVaneSetting("SWING");
                updated = true;
                break;
            case climate::CLIMATE_SWING_BOTH:
                hp->setVaneSetting("SWING");
                hp->setWideVaneSetting("SWING");
                updated = true;
                break;
            default:
                ESP_LOGW(TAG, "control - received unsupported swing mode request.");

        }
    }
    ESP_LOGD(TAG, "control - Was HeatPump updated? %s", YESNO(updated));

    // send the update back to esphome:
    this->publish_state();
    // and the heat pump:
    hp->update();
}

void MitsubishiHeatPump::hpSettingsChanged() {
    this->settingsUpdated = true;

    heatpumpSettings currentSettings = hp->getSettings();
    if (currentSettings.power == NULL) {
        /*
         * We should always get a valid pointer here once the HeatPump
         * component fully initializes. If HeatPump hasn't read the settings
         * from the unit yet (hp->connect() doesn't do this, sadly), we'll need
         * to punt on the update. Likely not an issue when run in callback
         * mode, but that isn't working right yet.
         */
        ESP_LOGW(TAG, "Waiting for HeatPump to read the settings the first time.");
        esphome::delay(10);
        return;
    }

    /*
     * ************ HANDLE POWER AND MODE CHANGES ***********
     * https://github.com/geoffdavis/HeatPump/blob/stream/src/HeatPump.h#L125
     * const char* POWER_MAP[2]       = {"OFF", "ON"};
     * const char* MODE_MAP[5]        = {"HEAT", "DRY", "COOL", "FAN", "AUTO"};
     */
    const DeviceState deviceState = devicestate::toDeviceState(&currentSettings);
    device_state_active->publish_state(deviceState.active);
    if (this->initializedState) {
        if (this->internalPowerOn != deviceState.active) {
            ESP_LOGI(TAG, "Device active on change: deviceState.active={%s} internalPowerOn={%s}", YESNO(deviceState.active), YESNO(this->internalPowerOn));
        }
    } else {
        if (this->internalPowerOn != deviceState.active) {
            ESP_LOGW(TAG, "Initializing internalPowerOn state from %s to %s", ONOFF(this->internalPowerOn), ONOFF(deviceState.active));
            this->internalPowerOn = deviceState.active;
        }
        internal_power_on->publish_state(this->internalPowerOn);

        this->initializedState = true;
        device_state_initialized->publish_state(this->initializedState);
    }
    

    // We cannot use the internal state of the device initialize component state
    if (this->isComponentActive()) {
        switch (deviceState.mode) {
            case DeviceMode::Heat:
                this->mode = climate::CLIMATE_MODE_HEAT;
                if (heat_setpoint != currentSettings.temperature) {
                    heat_setpoint = currentSettings.temperature;
                    save(currentSettings.temperature, heat_storage);
                }

                if (deviceState.active) {
                    this->action = climate::CLIMATE_ACTION_IDLE;
                } else {
                    this->action = climate::CLIMATE_ACTION_OFF;
                }
                break;
            case DeviceMode::Dry:
                this->mode = climate::CLIMATE_MODE_DRY;
                this->action = climate::CLIMATE_ACTION_DRYING;
                break;
            case DeviceMode::Cool:
                this->mode = climate::CLIMATE_MODE_COOL;
                if (cool_setpoint != currentSettings.temperature) {
                    cool_setpoint = currentSettings.temperature;
                    save(currentSettings.temperature, cool_storage);
                }

                if (deviceState.active) {
                    this->action = climate::CLIMATE_ACTION_IDLE;
                } else {
                    this->action = climate::CLIMATE_ACTION_OFF;
                }
                break;
            case DeviceMode::Fan:
                this->mode = climate::CLIMATE_MODE_FAN_ONLY;
                this->action = climate::CLIMATE_ACTION_FAN;
                break;
            case DeviceMode::Auto:
                this->mode = climate::CLIMATE_MODE_HEAT_COOL;
                if (auto_setpoint != currentSettings.temperature) {
                    auto_setpoint = currentSettings.temperature;
                    save(currentSettings.temperature, auto_storage);
                }

                if (deviceState.active) {
                    this->action = climate::CLIMATE_ACTION_IDLE;
                } else {
                    this->action = climate::CLIMATE_ACTION_OFF;
                }
                break;
            default:
                ESP_LOGW(
                        TAG,
                        "Unknown climate mode value %s received from HeatPump",
                        currentSettings.mode
                );
        }
    } else {
        this->mode = climate::CLIMATE_MODE_OFF;
        this->action = climate::CLIMATE_ACTION_OFF;
    }

    ESP_LOGI(TAG, "Climate mode is: %i", this->mode);

    /*
     * ******* HANDLE FAN CHANGES ********
     *
     * const char* FAN_MAP[6]         = {"AUTO", "QUIET", "1", "2", "3", "4"};
     */
    if (strcmp(currentSettings.fan, "QUIET") == 0) {
        this->fan_mode = climate::CLIMATE_FAN_DIFFUSE;
    } else if (strcmp(currentSettings.fan, "1") == 0) {
            this->fan_mode = climate::CLIMATE_FAN_LOW;
    } else if (strcmp(currentSettings.fan, "2") == 0) {
            this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
    } else if (strcmp(currentSettings.fan, "3") == 0) {
            this->fan_mode = climate::CLIMATE_FAN_MIDDLE;
    } else if (strcmp(currentSettings.fan, "4") == 0) {
            this->fan_mode = climate::CLIMATE_FAN_HIGH;
    } else { //case "AUTO" or default:
        this->fan_mode = climate::CLIMATE_FAN_AUTO;
    }
    ESP_LOGI(TAG, "Fan mode is: %i", this->fan_mode.value_or(-1));

    /* ******** HANDLE MITSUBISHI VANE CHANGES ********
     * const char* VANE_MAP[7]        = {"AUTO", "1", "2", "3", "4", "5", "SWING"};
     */
    if (strcmp(currentSettings.vane, "SWING") == 0 &&
        strcmp(currentSettings.wideVane, "SWING") == 0) {
        this->swing_mode = climate::CLIMATE_SWING_BOTH;
    } else if (strcmp(currentSettings.vane, "SWING") == 0) {
        this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
    } else if (strcmp(currentSettings.wideVane, "SWING") == 0) {
        this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
    } else {
        this->swing_mode = climate::CLIMATE_SWING_OFF;
    }
    ESP_LOGI(TAG, "Swing mode is: %i", this->swing_mode);
    if (strcmp(currentSettings.vane, "SWING") == 0) {
        this->update_swing_vertical("swing");
    } else if (strcmp(currentSettings.vane, "AUTO") == 0) {
        this->update_swing_vertical("auto");
    } else if (strcmp(currentSettings.vane, "1") == 0) {
        this->update_swing_vertical("up");
    } else if (strcmp(currentSettings.vane, "2") == 0) {
        this->update_swing_vertical("up_center");
    } else if (strcmp(currentSettings.vane, "3") == 0) {
        this->update_swing_vertical("center");
    } else if (strcmp(currentSettings.vane, "4") == 0) {
        this->update_swing_vertical("down_center");
    } else if (strcmp(currentSettings.vane, "5") == 0) {
        this->update_swing_vertical("down");
    }

    ESP_LOGI(TAG, "Vertical vane mode is: %s", currentSettings.vane);

    if (strcmp(currentSettings.wideVane, "SWING") == 0) {
        this->update_swing_horizontal("swing");
    } else if (strcmp(currentSettings.wideVane, "<>") == 0) {
        this->update_swing_horizontal("auto");
    } else if (strcmp(currentSettings.wideVane, "<<") == 0) {
        this->update_swing_horizontal("left");
    } else if (strcmp(currentSettings.wideVane, "<") == 0) {
        this->update_swing_horizontal("left_center");
    } else if (strcmp(currentSettings.wideVane, "|") == 0) {
        this->update_swing_horizontal("center");
    } else if (strcmp(currentSettings.wideVane, ">") == 0) {
        this->update_swing_horizontal("right_center");
    } else if (strcmp(currentSettings.wideVane, ">>") == 0) {
        this->update_swing_horizontal("right");
    }

    ESP_LOGI(TAG, "Horizontal vane mode is: %s", currentSettings.wideVane);



    /*
     * ******** HANDLE TARGET TEMPERATURE CHANGES ********
     */
    this->update_setpoint(currentSettings.temperature);
    ESP_LOGI(TAG, "Target temp is: %f", this->target_temperature);

    /*
     * ******** Publish state back to ESPHome. ********
     */
    this->publish_state();
}

/**
 * Report changes in the current temperature sensed by the HeatPump.
 */
void MitsubishiHeatPump::hpStatusChanged(heatpumpStatus currentStatus) {
    this->statusUpdated = true;
    device_status_operating->publish_state(currentStatus.operating);

    this->current_temperature = currentStatus.roomTemperature;
    switch (this->mode) {
        case climate::CLIMATE_MODE_HEAT:
            if (currentStatus.operating) {
                this->action = climate::CLIMATE_ACTION_HEATING;
            }
            else {
                if (this->internalPowerOn) {
                    this->action = climate::CLIMATE_ACTION_IDLE;
                } else {
                    this->action = climate::CLIMATE_ACTION_OFF;
                }
            }
            break;
        case climate::CLIMATE_MODE_COOL:
            if (currentStatus.operating) {
                this->action = climate::CLIMATE_ACTION_COOLING;
            }
            else {
                if (this->internalPowerOn) {
                    this->action = climate::CLIMATE_ACTION_IDLE;
                } else {
                    this->action = climate::CLIMATE_ACTION_OFF;
                }
            }
            break;
        case climate::CLIMATE_MODE_HEAT_COOL:
            if (this->internalPowerOn) {
                this->action = climate::CLIMATE_ACTION_IDLE;
            } else {
                this->action = climate::CLIMATE_ACTION_OFF;
            }

            if (currentStatus.operating) {
              if (this->current_temperature > this->target_temperature) {
                  this->action = climate::CLIMATE_ACTION_COOLING;
              } else if (this->current_temperature < this->target_temperature) {
                  this->action = climate::CLIMATE_ACTION_HEATING;
              }
            }
            break;
        case climate::CLIMATE_MODE_DRY:
            if (currentStatus.operating) {
                this->action = climate::CLIMATE_ACTION_DRYING;
            }
            else {
                if (this->internalPowerOn) {
                    this->action = climate::CLIMATE_ACTION_IDLE;
                } else {
                    this->action = climate::CLIMATE_ACTION_OFF;
                }
            }
            break;
        case climate::CLIMATE_MODE_FAN_ONLY:
            this->action = climate::CLIMATE_ACTION_FAN;
            break;
        default:
            if (this->internalPowerOn) {
                this->action = climate::CLIMATE_ACTION_IDLE;
            } else {
                this->action = climate::CLIMATE_ACTION_OFF;
            }
    }

    this->operating_ = currentStatus.operating;

    this->publish_state();
}

void MitsubishiHeatPump::set_remote_temperature(float temp) {
    ESP_LOGD(TAG, "Setting remote temp: %.1f", temp);
    if (temp > 0) {
        last_remote_temperature_sensor_update_ = 
            std::chrono::steady_clock::now();
    } else {
        last_remote_temperature_sensor_update_.reset();
    }

    this->hp->setRemoteTemperature(temp);
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

    if (!this->verify_serial()) {
        this->mark_failed();
        return;
    }

    ESP_LOGCONFIG(TAG, "Initializing new HeatPump object.");
    this->hp = new HeatPump();

#ifdef USE_CALLBACKS
    hp->setSettingsChangedCallback(
            [this]() {
                this->hpSettingsChanged();
            }
    );

    hp->setStatusChangedCallback(
            [this](heatpumpStatus currentStatus) {
                this->hpStatusChanged(currentStatus);
            }
    );

    hp->setPacketCallback(this->log_packet);    
#endif

    ESP_LOGCONFIG(
            TAG,
            "hw_serial(%p) is &Serial(%p)? %s",
            this->get_hw_serial_(),
            &Serial,
            YESNO((void *)this->get_hw_serial_() == (void *)&Serial)
    );

    ESP_LOGCONFIG(TAG, "Calling hp->connect(%p)", this->get_hw_serial_());
    if (hp->connect(this->get_hw_serial_(), this->baud_, this->rx_pin_, this->tx_pin_)) {
        hp->sync();
    }
    else {
        ESP_LOGCONFIG(
                TAG,
                "Connection to HeatPump failed."
                " Marking MitsubishiHeatPump component as failed."
        );
        this->mark_failed();
    }

    float min_temp = ESPMHP_MIN_TEMPERATURE;
    if (this->visual_min_temperature_override_.has_value()) {
        min_temp = this->visual_min_temperature_override_.value();
    }
    float max_temp = ESPMHP_MAX_TEMPERATURE;
    if (this->visual_max_temperature_override_.has_value()) {
        max_temp = this->visual_max_temperature_override_.value();
    }

    this->pidController = new PIDController(
        p,
        i,
        d,
        this->get_update_interval(),
        0.0,
        min_temp,
        max_temp
    );

    // create various setpoint persistence:
    cool_storage = global_preferences->make_preference<uint8_t>(this->get_object_id_hash() + 1);
    heat_storage = global_preferences->make_preference<uint8_t>(this->get_object_id_hash() + 2);
    auto_storage = global_preferences->make_preference<uint8_t>(this->get_object_id_hash() + 3);

    // load values from storage:
    cool_setpoint = load(cool_storage);
    heat_setpoint = load(heat_storage);
    auto_setpoint = load(auto_storage);

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

    internal_power_on->publish_state(this->internalPowerOn);
    device_state_initialized->publish_state(this->initializedState);

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

optional<float> MitsubishiHeatPump::load(ESPPreferenceObject& storage) {
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
    ESP_LOGI(TAG, "  Saved heat: %.1f", heat_setpoint.value_or(-1));
    ESP_LOGI(TAG, "  Saved cool: %.1f", cool_setpoint.value_or(-1));
    ESP_LOGI(TAG, "  Saved auto: %.1f", auto_setpoint.value_or(-1));
}

void MitsubishiHeatPump::dump_state() {
    LOG_CLIMATE("", "MitsubishiHeatPump Climate", this);
    ESP_LOGI(TAG, "HELLO");
}

void MitsubishiHeatPump::log_packet(byte* packet, unsigned int length, char* packetDirection) {
    String packetHex;
    char textBuf[15];

    for (int i = 0; i < length; i++) {
        memset(textBuf, 0, 15);
        sprintf(textBuf, "%02X ", packet[i]);
        packetHex += textBuf;
    }
    
    ESP_LOGV(TAG, "PKT: [%s] %s", packetDirection, packetHex.c_str());
}

bool MitsubishiHeatPump::isComponentActive() {
    return this->mode != climate::CLIMATE_MODE_OFF;
}

void MitsubishiHeatPump::dump_heat_pump_details(const devicestate::DeviceState& deviceState) {
    ESP_LOGI(TAG, "Internal State");
    ESP_LOGI(TAG, "  active: %s", TRUEFALSE(this->internalPowerOn));
    /*
    struct DeviceState {
        bool active;
        DeviceMode mode;
        float targetTemperature;
    };
    */
    ESP_LOGI(TAG, "Device State");
    ESP_LOGI(TAG, "  active: %s", TRUEFALSE(deviceState.active));
    ESP_LOGI(TAG, "  mode: unknown"); // , deviceState.mode
    ESP_LOGI(TAG, "  targetTemperature: %f", deviceState.targetTemperature);
    /*
    struct heatpumpStatus {
        float roomTemperature;
        bool operating; // if true, the heatpump is operating to reach the desired temperature
        heatpumpTimers timers;
        int compressorFrequency;
    };
    */
    ESP_LOGI(TAG, "Heatpump Status: %s", YESNO(this->statusUpdated));
    if (this->statusUpdated) {
        heatpumpStatus currentStatus = hp->getStatus();
        ESP_LOGI(TAG, "  roomTemperature: %f", currentStatus.roomTemperature);
        ESP_LOGI(TAG, "  operating: %s", TRUEFALSE(currentStatus.operating));
        ESP_LOGI(TAG, "  compressorFrequency: %f", currentStatus.compressorFrequency);
    }

    /*
    struct heatpumpSettings {
        const char* power;
        const char* mode;
        float temperature;
        const char* fan;
        const char* vane; //vertical vane, up/down
        const char* wideVane; //horizontal vane, left/right
        bool iSee;   //iSee sensor, at the moment can only detect it, not set it
        bool connected;
    };
    */
    
    ESP_LOGI(TAG, "Heatpump Settings: %s", YESNO(this->settingsUpdated));
    if (this->settingsUpdated) {
        heatpumpSettings currentSettings = hp->getSettings();
        ESP_LOGI(TAG, "  power: %s", currentSettings.power);
        ESP_LOGI(TAG, "  mode: %s", currentSettings.mode);
        ESP_LOGI(TAG, "  temperature: %f", currentSettings.temperature);
        ESP_LOGI(TAG, "  fan: %s", currentSettings.fan);
        ESP_LOGI(TAG, "  vane: %s", currentSettings.vane);
        ESP_LOGI(TAG, "  wideVane: %s", currentSettings.wideVane);
        ESP_LOGI(TAG, "  connected: %s", TRUEFALSE(currentSettings.connected));
    }

    /*
    ESP_LOGI(TAG, "Current temperature (state):  %f", this->current_temperature);
    ESP_LOGI(TAG, "Target temperature (state):  %f", this->target_temperature);
    */
}

bool MitsubishiHeatPump::same_float(const float left, const float right) {
    return fabs(left - right) <= 0.001;
}

void MitsubishiHeatPump::ensure_pid_target() {
    if (!this->same_float(this->target_temperature, this->pidController->getTarget())) {
        ESP_LOGI(TAG, "PID Target temp changing from %f to %f", this->pidController->getTarget(), this->target_temperature);
        this->pidController->setTarget(this->target_temperature);
    }
}

void MitsubishiHeatPump::update_setpoint(const float value) {
    if (this->same_float(this->target_temperature, value)) {
        ESP_LOGD(TAG, "Target temp unchanged: current={%f} updated={%f}", this->target_temperature, value);
        this->ensure_pid_target();
        return;
    }

    ESP_LOGI(TAG, "Target temp changing from %f to %f", this->target_temperature, value);
    this->target_temperature = value;

    ESP_LOGI(TAG, "PID Target temp changing from %f to %f", this->pidController->getTarget(), value);
    this->pidController->setTarget(this->target_temperature);
}

void MitsubishiHeatPump::internalTurnOn() {
    const uint32_t end = esphome::millis();
    const int durationInMilliseconds = end - this->lastInternalPowerUpdate;
    const int durationInSeconds = durationInMilliseconds / 1000;
    const int remaining = 60 - durationInSeconds; 
    if (remaining > 0) {
        ESP_LOGD(TAG, "Throttling internal turn on: %i seconds remaining", remaining);
        return;
    }

    hp->setPowerSetting("ON");
    if (hp->update()) {
        this->lastInternalPowerUpdate = end;
        this->internalPowerOn = true;
        internal_power_on->publish_state(this->internalPowerOn);
        ESP_LOGW(TAG, "Performed internal turn on!");
    } else {
        ESP_LOGW(TAG, "Failed to perform internal turn on!");
    }
}

void MitsubishiHeatPump::internalTurnOff() {
    const uint32_t end = esphome::millis();
    const int durationInMilliseconds = end - this->lastInternalPowerUpdate;
    const int durationInSeconds = durationInMilliseconds / 1000;
    const int remaining = 60 - durationInSeconds; 
    if (remaining > 0) {
        ESP_LOGD(TAG, "Throttling internal turn off: %i seconds remaining", remaining);
        return;
    }

    hp->setPowerSetting("OFF");
    if (hp->update()) {
        this->lastInternalPowerUpdate = end;
        this->internalPowerOn = false;
        internal_power_on->publish_state(this->internalPowerOn);
        ESP_LOGW(TAG, "Performed internal turn off!");
    } else {
        ESP_LOGW(TAG, "Failed to perform internal turn off!");
    }
}

void MitsubishiHeatPump::run_workflows() {
    if (!this->statusUpdated || !this->settingsUpdated) {
        ESP_LOGD(TAG, "System not ready, skipping run_workflows statusUpdated={%s}, settingsUpdated={%s}", YESNO(statusUpdated), YESNO(settingsUpdated));
        return;
    }

    ESP_LOGD(TAG, "Run workflows...");

    if (!this->isComponentActive()) {
        ESP_LOGW(TAG, "Skipping run workflow due to inactive state.");
        return;
    }

    ESP_LOGD(TAG, "Check PID Target: %f", this->pidController->getTarget());
    this->ensure_pid_target();
    if (this->pidController->getTarget() == 0) {
        ESP_LOGW(TAG, "Skipping run workflow due pid target 0.");
        return;
    }

    const float setPointCorrection = this->pidController->update(this->current_temperature);
    pid_set_point_correction->publish_state(setPointCorrection);
    ESP_LOGI(TAG, "PIDController set point correction: %.1f", setPointCorrection);

    heatpumpSettings currentSettings = hp->getSettings();
    const DeviceState deviceState = devicestate::toDeviceState(&currentSettings);
    device_state_active->publish_state(deviceState.active);
    ESP_LOGI(TAG, "Device active on workflow: deviceState.active={%s} internalPowerOn={%s}", YESNO(deviceState.active), YESNO(this->internalPowerOn));
    if (deviceState.active != this->internalPowerOn) {
        this->dump_heat_pump_details(deviceState);
    }
    switch(this->action) {
        case climate::CLIMATE_ACTION_HEATING: {
            if (!deviceState.active) {
                return;
            }

            if (this->current_temperature - setPointCorrection > hysterisisUnderOff ||
                    this->same_float(setPointCorrection, this->pidController->getOutputMin())) {
                ESP_LOGI(TAG, "Turn off heating!");
                this->internalTurnOff();
                return;
            }

            //this.updateHeatingSetpoint(setPointCorrection);
            break;
        }
        case climate::CLIMATE_ACTION_COOLING: {
            if (!deviceState.active) {
                return;
            }

            if (setPointCorrection - this->current_temperature > hysterisisUnderOff ||
                    this->same_float(this->pidController->getOutputMax(), setPointCorrection)) {
                ESP_LOGI(TAG, "Turn off cooling!");
                this->internalTurnOff();
                return;
            }

            //this.updateCoolingSetpoint(setPointCorrection);
            break;
        }
        case climate::CLIMATE_ACTION_IDLE: {
            if (!deviceState.active) {
                return;
            }

            if (this->mode == climate::CLIMATE_MODE_HEAT) {
                if (this->current_temperature - setPointCorrection > hysterisisUnderOff ||
                        this->same_float(setPointCorrection, this->pidController->getOutputMin())) {
                    ESP_LOGI(TAG, "Turn off while idling heat!");
                    this->internalTurnOff();
                    return;
                }
            } else if (this->mode == climate::CLIMATE_MODE_COOL) {
                if (setPointCorrection - this->current_temperature > hysterisisUnderOff ||
                        this->same_float(this->pidController->getOutputMax(), setPointCorrection)) {
                    ESP_LOGI(TAG, "Turn off while idling cool!");
                    this->internalTurnOff();
                    return;
                }
            }

            //this.updateCoolingSetpoint(setPointCorrection);
            break;
        }
        default: {
            if (deviceState.active) {
                return;
            }

            if (this->mode == climate::CLIMATE_MODE_HEAT) {
                const float delta = this->current_temperature - setPointCorrection;
                ESP_LOGI(TAG, "Device off on heat: delta={%f} current={%f} setPointCorrection={%f}", delta, this->current_temperature, setPointCorrection);
                if (delta > 0) {
                    return;
                }

                ESP_LOGI(TAG, "Turning on Workflow heat");
                this->internalTurnOn();
            } else if (this->mode == climate::CLIMATE_MODE_COOL) {
                const float delta = setPointCorrection - this->current_temperature;
                ESP_LOGI(TAG, "Device off on cool: delta={%f} current={%f} setPointCorrection={%f}", delta, this->current_temperature, setPointCorrection);
                if (delta > 0) {
                    return;
                }

                ESP_LOGI(TAG, "Turning on Workflow cool");
                this->internalTurnOn();
            } else {
                ESP_LOGI(TAG, "Device off on other: current={%f} setPointCorrection={%f}", this->current_temperature, setPointCorrection);
            }
            break;
        }
    }
}