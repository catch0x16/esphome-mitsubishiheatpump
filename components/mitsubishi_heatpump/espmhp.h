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

#include "esphome/components/pid/pid_controller.h"

#include <chrono>

#include "devicestate.h"

#include "HeatPump.h"

#ifndef ESPMHP_H
#define ESPMHP_H

static const char* TAG = "MitsubishiHeatPump"; // Logging tag

static const char* ESPMHP_VERSION = "2.5.0";

/* If polling interval is greater than 9 seconds, the HeatPump
library reconnects, but doesn't then follow up with our data request.*/
static const uint32_t ESPMHP_POLL_INTERVAL_DEFAULT = 500; // in milliseconds,
                                                           // 0 < X <= 9000
static const uint8_t ESPMHP_MIN_TEMPERATURE = 16; // degrees C,
                                                  // defined by hardware
static const uint8_t ESPMHP_MAX_TEMPERATURE = 31; // degrees C,
                                                  //defined by hardware
static const float   ESPMHP_TARGET_TEMPERATURE_STEP = 0.5; // temperature setting step,
                                                    // in degrees C
static const float   ESPMHP_CURRENT_TEMPERATURE_STEP = 0.1; // temperature setting step,
                                                    // in degrees C

static const float hysterisisUnderOff = 0.25; // in degrees C
static const float hysterisisOverOn = 0.25; // in degrees C

//P=4.09 I=0.058 D=3.38
static const float p = 4.0;
static const float i = 0.02;
static const float d = 0.1;

class MitsubishiHeatPump : public esphome::PollingComponent, public esphome::climate::Climate {
    public:

        /**
         * Create a new MitsubishiHeatPump object
         *
         * Args:
         *   hw_serial: pointer to an Arduino HardwareSerial instance
         *   poll_interval: polling interval in milliseconds
         */
        MitsubishiHeatPump(
            HardwareSerial* hw_serial,
            uint32_t poll_interval=ESPMHP_POLL_INTERVAL_DEFAULT
        );

        esphome::binary_sensor::BinarySensor* internal_power_on;
        esphome::binary_sensor::BinarySensor* device_state_connected;
        esphome::binary_sensor::BinarySensor* device_state_active;
        esphome::sensor::Sensor* device_state_last_updated;
        esphome::binary_sensor::BinarySensor* device_status_operating;
        esphome::sensor::Sensor* device_status_current_temperature;
        esphome::sensor::Sensor* device_status_compressor_frequency;
        esphome::sensor::Sensor* device_status_last_updated;
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

        // handle a change in device;
        void updateDevice();

        // Set up the component, initializing the HeatPump object.
        void setup() override;

        // This is called every poll_interval.
        void update() override;

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

    protected:
        // HeatPump object using the underlying Arduino library.
        devicestate::DeviceStateManager* dsm;

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
        HardwareSerial* get_hw_serial_() {
            return this->hw_serial_;
        }

        //Print a warning message if we're using the sole hardware UART on an
        //ESP8266 or UART0 on ESP32, or if no serial was provided
        bool verify_serial();

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
        /// The current temperature of the climate device, as reported from the integration.
        float remote_temperature{NAN};
        bool remote_temperature_updated{false};

        void enforce_remote_temperature_sensor_timeout();

        // Retrieve the HardwareSerial pointer from friend and subclasses.
        HardwareSerial *hw_serial_;
        int baud_ = 0;

        int rx_pin_ = -1;
        int tx_pin_ = -1;
        bool operating_ = false;

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

        PIDController *pidController;
        void ensure_pid_target();
};

#endif
