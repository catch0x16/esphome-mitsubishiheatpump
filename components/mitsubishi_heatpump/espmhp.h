/**
 * espmhp.h
 *
 * Header file for esphome-mitsubishiheatpump
 *
 * Author: Geoff Davis <geoff@geoffdavis.com>
 * Author: Phil Genera @pgenera on Github.
 * Author: @nao-pon on Github
 * Author: Simon Knopp @sijk on Github
 * Last Updated: 2021-06-23
 * License: BSD
 *
 * Requirements:
 * - https://github.com/SwiCago/HeatPump
 * - ESPHome 1.19.1 or greater
 */

#include "esphome.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/select/select.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/preferences.h"
#include "esphome/components/uart/uart.h"

#include <chrono>

#include "devicestate.h"
using namespace devicestate;

#include "HeatPump.h"
#include "cycle_management.h"
#include "request_scheduler.h"
#include "logging.h"

#ifndef ESPMHP_H
#define ESPMHP_H

static const char* ESPMHP_VERSION = "2.5.0";

/* If polling interval is greater than 9 seconds, the HeatPump
library reconnects, but doesn't then follow up with our data request.*/
static const uint32_t ESPMHP_POLL_INTERVAL_DEFAULT = 500; // in milliseconds,
                                                           // 0 < X <= 9000

static const float   ESPMHP_TARGET_TEMPERATURE_STEP = 0.5; // temperature setting step,
                                                    // in degrees C
static const float   ESPMHP_CURRENT_TEMPERATURE_STEP = 0.1; // temperature setting step,
                                                    // in degrees C

class MitsubishiHeatPump : public esphome::Component, public esphome::climate::Climate, public esphome::uart::UARTDevice {
    public:

        /**
         * Create a new MitsubishiHeatPump object
         *
         * Args:
         *   hw_serial: pointer to a UART instance
         *   poll_interval: polling interval in milliseconds
         */
        MitsubishiHeatPump(
            esphome::uart::UARTComponent* hw_serial,
            uint32_t poll_interval=ESPMHP_POLL_INTERVAL_DEFAULT
        );

        esphome::binary_sensor::BinarySensor* internal_power_on;
        esphome::binary_sensor::BinarySensor* device_state_connected;
        esphome::binary_sensor::BinarySensor* device_state_active;
        esphome::binary_sensor::BinarySensor* device_status_operating;
        esphome::sensor::Sensor* device_status_current_temperature;
        esphome::sensor::Sensor* device_status_compressor_frequency;
        esphome::sensor::Sensor* device_status_input_power;
        esphome::sensor::Sensor* device_status_kwh;
        esphome::sensor::Sensor* device_status_runtime_hours;
        esphome::sensor::Sensor* pid_set_point_correction;
        esphome::sensor::Sensor* device_set_point;

        // Print a banner with library information.
        void banner();

        // Set the baud rate. Must be called before setup() to have any effect.
        void set_baud_rate(int);

        // Set the RX pin. Must be called before setup() to have any effect.
        void set_rx_pin(int);

        // Set the TX pin. Must be called before setup() to have any effect.
        void set_tx_pin(int);

        void set_kp(float kp) { this->kp_ = kp; }
        void set_ki(float ki) { this->ki_ = ki; }
        void set_kd(float kd) { this->kd_ = kd; }
        void set_max_adjustment_under(float maxAdjustmentUnder) { this->maxAdjustmentUnder_ = maxAdjustmentUnder; }
        void set_max_adjustment_over(float maxAdjustmentOver) { this->maxAdjustmentOver_ = maxAdjustmentOver; }
        void set_hysterisis_off(float hysterisisOff) { this->hysterisisOff_ = hysterisisOff; }
        void set_hysterisis_on(float hysterisisOn) { this->hysterisisOn_ = hysterisisOn; }

        uint32_t get_update_interval() const;
        void set_update_interval(uint32_t update_interval);

        // handle a change in device;
        void updateDevice();

        // Set up the component, initializing the HeatPump object.
        void setup() override;

        // This is called every poll_interval.
        void loop() override;

        // This is called every poll_interval.
        void update();

        // Configure the climate object with traits that we support.
        esphome::climate::ClimateTraits traits() override;

        // Get a mutable reference to the traits that we support.
        esphome::climate::ClimateTraits& config_traits();

        // print the current configuration
        void dump_config() override;

        // Debugging function to print the object's state.
        void dump_state();

        // Handle a request from the user to change settings.
        void control(const esphome::climate::ClimateCall &call) override;

        // Use the temperature from an external sensor. Use
        // set_remote_temp(0) to switch back to the internal sensor.
        void set_remote_temperature(float);

        void set_vertical_vane_select(esphome::select::Select *vertical_vane_select);
        void set_horizontal_vane_select(esphome::select::Select *horizontal_vane_select);

        // Used to validate that a connection is present between the controller
        // and this heatpump.
        void ping();

        // Number of minutes before the heatpump reverts back to the internal
        // temperature sensor if the machine is currently operating.
        void set_remote_operating_timeout_minutes(int);

        // Number of minutes before the heatpump reverts back to the internal
        // temperature sensor if the machine is currently idle.
        void set_remote_idle_timeout_minutes(int);

        // Number of minutes before the heatpump reverts back to the internal
        // temperature sensor if a ping isn't received from the controller.
        void set_remote_ping_timeout_minutes(int);

        void set_internal_power_on_sensor(esphome::binary_sensor::BinarySensor* internal_power_on) {
            this->internal_power_on = internal_power_on;
        }

        void set_device_state_connected_sensor(esphome::binary_sensor::BinarySensor* device_state_connected) {
            this->device_state_connected = device_state_connected;
        }

        void set_device_state_active_sensor(esphome::binary_sensor::BinarySensor* device_state_active) {
            this->device_state_active = device_state_active;
        }

        void set_device_status_operating_sensor(esphome::binary_sensor::BinarySensor* device_status_operating) {
            this->device_status_operating = device_status_operating;
        }

        void set_device_status_current_temperature_sensor(esphome::sensor::Sensor* device_status_current_temperature) {
            this->device_status_current_temperature = device_status_current_temperature;
        }

        void set_device_status_compressor_frequency_sensor(esphome::sensor::Sensor* device_status_compressor_frequency) {
            this->device_status_compressor_frequency = device_status_compressor_frequency;
        }

        void set_device_status_input_power_sensor(esphome::sensor::Sensor* device_status_input_power) {
            this->device_status_input_power = device_status_input_power;
        }

        void set_device_status_kwh_sensor(esphome::sensor::Sensor* device_status_kwh) {
            this->device_status_kwh = device_status_kwh;
        }

        void set_device_status_runtime_hours_sensor(esphome::sensor::Sensor* device_status_runtime_hours) {
            this->device_status_runtime_hours = device_status_runtime_hours;
        }

        void set_pid_set_point_correction_sensor(esphome::sensor::Sensor* pid_set_point_correction) {
            this->pid_set_point_correction = pid_set_point_correction;
        }

        void set_device_set_point_sensor(esphome::sensor::Sensor* device_set_point) {
            this->device_set_point = device_set_point;
        }

    protected:
        // HeatPump object using the underlying Arduino library.
        devicestate::DeviceStateManager* dsm{nullptr};

        WorkflowStep* hysterisisWorkflowStep;
        WorkflowStep* pidWorkflowStep;

        // The ClimateTraits supported by this HeatPump.
        esphome::climate::ClimateTraits traits_;

        // Vane position
        void update_swing_horizontal(const std::string &swing);
        void update_swing_vertical(const std::string &swing);
        std::string vertical_swing_state_;
        std::string horizontal_swing_state_;

        // Allow the HeatPump class to use get_hw_serial_
        friend class HeatPump;

        //Accessor method for the HardwareSerial pointer
        esphome::uart::UARTComponent* get_hw_serial_() {
            return this->hw_serial_;
        }

        // various prefs to save mode-specific temperatures, akin to how the IR
        // remote works.
        esphome::ESPPreferenceObject cool_storage;
        esphome::ESPPreferenceObject heat_storage;
        esphome::ESPPreferenceObject auto_storage;

        esphome::optional<float> cool_setpoint;
        esphome::optional<float> heat_setpoint;
        esphome::optional<float> auto_setpoint;

        static void save(float value, esphome::ESPPreferenceObject& storage);
        static esphome::optional<float> load(esphome::ESPPreferenceObject& storage);

        esphome::select::Select *vertical_vane_select_ =
            nullptr;  // Select to store manual position of vertical swing
        esphome::select::Select *horizontal_vane_select_ =
            nullptr;  // Select to store manual position of horizontal swing

        // When received command to change the vane positions
        void on_horizontal_swing_change(const std::string &swing);
        void on_vertical_swing_change(const std::string &swing);

    private:
        cycleManagement loopCycle{};
        uint32_t update_interval_;

         RequestScheduler* scheduler_;
         void terminateCycle();

        /// The current temperature of the climate device, as reported from the integration.
        float remote_temperature{NAN};
        bool remote_temperature_updated{false};

        void enforce_remote_temperature_sensor_timeout();

        // Retrieve the HardwareSerial pointer from friend and subclasses.
        esphome::uart::UARTComponent* hw_serial_;
        int baud_ = 0;
        int rx_pin_ = -1;
        int tx_pin_ = -1;

        bool operating_ = false;

        float kp_;
        float ki_;
        float kd_;
        float maxAdjustmentUnder_;
        float maxAdjustmentOver_;
        float hysterisisOff_;
        float hysterisisOn_;

        bool isInitialized = false;

        std::optional<std::chrono::duration<long long, std::ratio<60>>> remote_operating_timeout_;
        std::optional<std::chrono::duration<long long, std::ratio<60>>> remote_idle_timeout_;
        std::optional<std::chrono::duration<long long, std::ratio<60>>> remote_ping_timeout_;
        std::optional<std::chrono::time_point<std::chrono::steady_clock>> last_remote_temperature_sensor_update_;
        std::optional<std::chrono::time_point<std::chrono::steady_clock>> last_ping_request_;

        void update_setpoint(float value);

        void run_workflows();

        bool isComponentActive();

        devicestate::DeviceState lastDeviceState;
        devicestate::DeviceStatus lastDeviceStatus;

        float min_temp;
        float max_temp;
};

#endif
