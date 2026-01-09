// Auto generated code by esphome
// ========== AUTO GENERATED INCLUDE BLOCK BEGIN ===========
#include "esphome.h"
using namespace esphome;
using std::isnan;
using std::min;
using std::max;
using namespace sensor;
using namespace climate;
using namespace select;
using namespace binary_sensor;
logger::Logger *logger_logger_id;
using namespace uart;
uart::IDFUARTComponent *hp_uart;
esp32::ESP32InternalGPIOPin *esp32_esp32internalgpiopin_id;
esp32::ESP32InternalGPIOPin *esp32_esp32internalgpiopin_id_2;
MitsubishiHeatPump *mitsubishiheatpump_id;
binary_sensor::BinarySensor *internal_power_on;
binary_sensor::BinarySensor *device_state_connected;
binary_sensor::BinarySensor *device_state_active;
binary_sensor::BinarySensor *device_status_operating;
sensor::Sensor *device_status_current_temperature;
sensor::Sensor *device_status_compressor_frequency;
sensor::Sensor *device_status_input_power;
sensor::Sensor *device_status_kwh;
sensor::Sensor *device_status_runtime_hours;
sensor::Sensor *pid_set_point_correction;
sensor::Sensor *device_set_point;
preferences::IntervalSyncer *preferences_intervalsyncer_id;
// ========== AUTO GENERATED INCLUDE BLOCK END ==========="

void setup() {
  // ========== AUTO GENERATED CODE BEGIN ===========
  // esphome:
  //   name: test-device
  //   min_version: 2025.12.5
  //   build_path: build/test-device
  //   friendly_name: ''
  //   platformio_options: {}
  //   environment_variables: {}
  //   includes: []
  //   includes_c: []
  //   libraries: []
  //   name_add_mac_suffix: false
  //   debug_scheduler: false
  //   areas: []
  //   devices: []
  App.pre_setup("test-device", "", "", __DATE__ ", " __TIME__, false);
  // sensor:
  // climate:
  // select:
  // binary_sensor:
  // logger:
  //   id: logger_logger_id
  //   baud_rate: 115200
  //   tx_buffer_size: 512
  //   deassert_rts_dtr: false
  //   task_log_buffer_size: 768
  //   hardware_uart: UART0
  //   level: DEBUG
  //   logs: {}
  //   runtime_tag_levels: false
  logger_logger_id = new logger::Logger(115200, 512);
  logger_logger_id->create_pthread_key();
  logger_logger_id->init_log_buffer(768);
  logger_logger_id->set_log_level(ESPHOME_LOG_LEVEL_DEBUG);
  logger_logger_id->set_uart_selection(logger::UART_SELECTION_UART0);
  logger_logger_id->pre_setup();
  logger_logger_id->set_component_source(LOG_STR("logger"));
  App.register_component(logger_logger_id);
  // esp32:
  //   board: esp32dev
  //   framework:
  //     type: esp-idf
  //     version: 5.5.1
  //     sdkconfig_options: {}
  //     log_level: ERROR
  //     advanced:
  //       compiler_optimization: SIZE
  //       enable_idf_experimental_features: false
  //       enable_lwip_assert: true
  //       ignore_efuse_custom_mac: false
  //       ignore_efuse_mac_crc: false
  //       enable_lwip_dhcp_server: false
  //       enable_lwip_mdns_queries: true
  //       enable_lwip_bridge_interface: false
  //       enable_lwip_tcpip_core_locking: true
  //       enable_lwip_check_thread_safety: true
  //       disable_libc_locks_in_iram: true
  //       disable_vfs_support_termios: true
  //       disable_vfs_support_select: true
  //       disable_vfs_support_dir: true
  //       freertos_in_iram: false
  //       ringbuf_in_iram: false
  //       execute_from_psram: false
  //       loop_task_stack_size: 8192
  //     components: []
  //     platform_version: https:github.com/pioarduino/platform-espressif32/releases/download/55.03.31-2/platform-espressif32.zip
  //     source: pioarduino/framework-espidf@https:github.com/pioarduino/esp-idf/releases/download/v5.5.1/esp-idf-v5.5.1.tar.xz
  //   flash_size: 4MB
  //   variant: ESP32
  //   cpu_frequency: 160MHZ
  // uart:
  //   id: hp_uart
  //   tx_pin:
  //     number: 17
  //     mode:
  //       output: true
  //       input: false
  //       open_drain: false
  //       pullup: false
  //       pulldown: false
  //     id: esp32_esp32internalgpiopin_id
  //     inverted: false
  //     ignore_pin_validation_error: false
  //     ignore_strapping_warning: false
  //     drive_strength: 20.0
  //   rx_pin:
  //     number: 16
  //     mode:
  //       input: true
  //       output: false
  //       open_drain: false
  //       pullup: false
  //       pulldown: false
  //     id: esp32_esp32internalgpiopin_id_2
  //     inverted: false
  //     ignore_pin_validation_error: false
  //     ignore_strapping_warning: false
  //     drive_strength: 20.0
  //   baud_rate: 2400
  //   rx_buffer_size: 256
  //   rx_timeout: 2
  //   stop_bits: 1
  //   data_bits: 8
  //   parity: NONE
  hp_uart = new uart::IDFUARTComponent();
  hp_uart->set_component_source(LOG_STR("uart"));
  App.register_component(hp_uart);
  hp_uart->set_baud_rate(2400);
  esp32_esp32internalgpiopin_id = new esp32::ESP32InternalGPIOPin();
  esp32_esp32internalgpiopin_id->set_pin(::GPIO_NUM_17);
  esp32_esp32internalgpiopin_id->set_drive_strength(::GPIO_DRIVE_CAP_2);
  esp32_esp32internalgpiopin_id->set_flags(gpio::Flags::FLAG_OUTPUT);
  hp_uart->set_tx_pin(esp32_esp32internalgpiopin_id);
  esp32_esp32internalgpiopin_id_2 = new esp32::ESP32InternalGPIOPin();
  esp32_esp32internalgpiopin_id_2->set_pin(::GPIO_NUM_16);
  esp32_esp32internalgpiopin_id_2->set_drive_strength(::GPIO_DRIVE_CAP_2);
  esp32_esp32internalgpiopin_id_2->set_flags(gpio::Flags::FLAG_INPUT);
  hp_uart->set_rx_pin(esp32_esp32internalgpiopin_id_2);
  hp_uart->set_rx_buffer_size(256);
  hp_uart->set_rx_full_threshold(1);
  hp_uart->set_rx_timeout(2);
  hp_uart->set_stop_bits(1);
  hp_uart->set_data_bits(8);
  hp_uart->set_parity(uart::UART_CONFIG_PARITY_NONE);
  // external_components:
  //   - source:
  //       path: /Users/admin/projects/esphome-mitsubishiheatpump/components
  //       type: local
  //     refresh: 1d
  //     components: all
  // climate.mitsubishi_heatpump:
  //   platform: mitsubishi_heatpump
  //   name: Test Heatpump
  //   uart_id: hp_uart
  //   control_parameters:
  //     kp: 0.0
  //     ki: 0.0
  //     kd: 0.0
  //     maxAdjustmentUnder: 2.0
  //     maxAdjustmentOver: 2.0
  //     hysterisisOff: 0.25
  //     hysterisisOn: 0.25
  //   disabled_by_default: false
  //   visual: {}
  //   id: mitsubishiheatpump_id
  //   update_interval: 2000ms
  //   supports:
  //     mode:
  //       - HEAT_COOL
  //       - COOL
  //       - HEAT
  //       - DRY
  //       - FAN_ONLY
  //     fan_mode:
  //       - AUTO
  //       - DIFFUSE
  //       - LOW
  //       - MEDIUM
  //       - MIDDLE
  //       - HIGH
  //     swing_mode:
  //       - 'OFF'
  //       - VERTICAL
  mitsubishiheatpump_id = new MitsubishiHeatPump(hp_uart);
  hp_uart->set_data_bits(8);
  hp_uart->set_parity(uart::UART_CONFIG_PARITY_EVEN);
  hp_uart->set_stop_bits(1);
  mitsubishiheatpump_id->config_traits().add_supported_mode(climate::CLIMATE_MODE_HEAT_COOL);
  mitsubishiheatpump_id->config_traits().add_supported_mode(climate::CLIMATE_MODE_COOL);
  mitsubishiheatpump_id->config_traits().add_supported_mode(climate::CLIMATE_MODE_HEAT);
  mitsubishiheatpump_id->config_traits().add_supported_mode(climate::CLIMATE_MODE_DRY);
  mitsubishiheatpump_id->config_traits().add_supported_mode(climate::CLIMATE_MODE_FAN_ONLY);
  mitsubishiheatpump_id->config_traits().add_supported_fan_mode(climate::CLIMATE_FAN_AUTO);
  mitsubishiheatpump_id->config_traits().add_supported_fan_mode(climate::CLIMATE_FAN_DIFFUSE);
  mitsubishiheatpump_id->config_traits().add_supported_fan_mode(climate::CLIMATE_FAN_LOW);
  mitsubishiheatpump_id->config_traits().add_supported_fan_mode(climate::CLIMATE_FAN_MEDIUM);
  mitsubishiheatpump_id->config_traits().add_supported_fan_mode(climate::CLIMATE_FAN_MIDDLE);
  mitsubishiheatpump_id->config_traits().add_supported_fan_mode(climate::CLIMATE_FAN_HIGH);
  mitsubishiheatpump_id->config_traits().add_supported_swing_mode(climate::CLIMATE_SWING_OFF);
  mitsubishiheatpump_id->config_traits().add_supported_swing_mode(climate::CLIMATE_SWING_VERTICAL);
  internal_power_on = new binary_sensor::BinarySensor();
  App.register_binary_sensor(internal_power_on);
  internal_power_on->set_name_and_object_id("Internal power on", "internal_power_on");
  internal_power_on->set_internal(false);
  internal_power_on->set_entity_category(::ENTITY_CATEGORY_DIAGNOSTIC);
  internal_power_on->set_trigger_on_initial_state(false);
  mitsubishiheatpump_id->set_internal_power_on_sensor(internal_power_on);
  device_state_connected = new binary_sensor::BinarySensor();
  App.register_binary_sensor(device_state_connected);
  device_state_connected->set_name_and_object_id("Device state connected", "device_state_connected");
  device_state_connected->set_internal(false);
  device_state_connected->set_entity_category(::ENTITY_CATEGORY_DIAGNOSTIC);
  device_state_connected->set_trigger_on_initial_state(false);
  mitsubishiheatpump_id->set_device_state_connected_sensor(device_state_connected);
  device_state_active = new binary_sensor::BinarySensor();
  App.register_binary_sensor(device_state_active);
  device_state_active->set_name_and_object_id("Device state active", "device_state_active");
  device_state_active->set_internal(false);
  device_state_active->set_entity_category(::ENTITY_CATEGORY_DIAGNOSTIC);
  device_state_active->set_trigger_on_initial_state(false);
  mitsubishiheatpump_id->set_device_state_active_sensor(device_state_active);
  device_status_operating = new binary_sensor::BinarySensor();
  App.register_binary_sensor(device_status_operating);
  device_status_operating->set_name_and_object_id("Device status operating", "device_status_operating");
  device_status_operating->set_internal(false);
  device_status_operating->set_entity_category(::ENTITY_CATEGORY_DIAGNOSTIC);
  device_status_operating->set_trigger_on_initial_state(false);
  mitsubishiheatpump_id->set_device_status_operating_sensor(device_status_operating);
  device_status_current_temperature = new sensor::Sensor();
  App.register_sensor(device_status_current_temperature);
  device_status_current_temperature->set_name_and_object_id("Device current temperature", "device_current_temperature");
  device_status_current_temperature->set_internal(false);
  device_status_current_temperature->set_entity_category(::ENTITY_CATEGORY_DIAGNOSTIC);
  device_status_current_temperature->set_device_class("temperature");
  device_status_current_temperature->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
  device_status_current_temperature->set_unit_of_measurement("\302\260C");
  device_status_current_temperature->set_accuracy_decimals(1);
  mitsubishiheatpump_id->set_device_status_current_temperature_sensor(device_status_current_temperature);
  device_status_compressor_frequency = new sensor::Sensor();
  App.register_sensor(device_status_compressor_frequency);
  device_status_compressor_frequency->set_name_and_object_id("Device status compressor frequency", "device_status_compressor_frequency");
  device_status_compressor_frequency->set_internal(false);
  device_status_compressor_frequency->set_entity_category(::ENTITY_CATEGORY_DIAGNOSTIC);
  device_status_compressor_frequency->set_device_class("frequency");
  device_status_compressor_frequency->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
  device_status_compressor_frequency->set_unit_of_measurement("Hz");
  device_status_compressor_frequency->set_accuracy_decimals(1);
  mitsubishiheatpump_id->set_device_status_compressor_frequency_sensor(device_status_compressor_frequency);
  device_status_input_power = new sensor::Sensor();
  App.register_sensor(device_status_input_power);
  device_status_input_power->set_name_and_object_id("Device status input power", "device_status_input_power");
  device_status_input_power->set_internal(false);
  device_status_input_power->set_entity_category(::ENTITY_CATEGORY_DIAGNOSTIC);
  device_status_input_power->set_device_class("power");
  device_status_input_power->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
  device_status_input_power->set_unit_of_measurement("W");
  device_status_input_power->set_accuracy_decimals(0);
  mitsubishiheatpump_id->set_device_status_input_power_sensor(device_status_input_power);
  device_status_kwh = new sensor::Sensor();
  App.register_sensor(device_status_kwh);
  device_status_kwh->set_name_and_object_id("Device status kWh", "device_status_kwh");
  device_status_kwh->set_internal(false);
  device_status_kwh->set_entity_category(::ENTITY_CATEGORY_DIAGNOSTIC);
  device_status_kwh->set_device_class("energy");
  device_status_kwh->set_state_class(sensor::STATE_CLASS_TOTAL_INCREASING);
  device_status_kwh->set_unit_of_measurement("kWh");
  device_status_kwh->set_accuracy_decimals(1);
  mitsubishiheatpump_id->set_device_status_kwh_sensor(device_status_kwh);
  device_status_runtime_hours = new sensor::Sensor();
  App.register_sensor(device_status_runtime_hours);
  device_status_runtime_hours->set_name_and_object_id("Device status runtime hours", "device_status_runtime_hours");
  device_status_runtime_hours->set_internal(false);
  device_status_runtime_hours->set_entity_category(::ENTITY_CATEGORY_DIAGNOSTIC);
  device_status_runtime_hours->set_device_class("duration");
  device_status_runtime_hours->set_state_class(sensor::STATE_CLASS_TOTAL_INCREASING);
  device_status_runtime_hours->set_unit_of_measurement("h");
  device_status_runtime_hours->set_accuracy_decimals(2);
  mitsubishiheatpump_id->set_device_status_runtime_hours_sensor(device_status_runtime_hours);
  pid_set_point_correction = new sensor::Sensor();
  App.register_sensor(pid_set_point_correction);
  pid_set_point_correction->set_name_and_object_id("PID Set Point Correction", "pid_set_point_correction");
  pid_set_point_correction->set_internal(false);
  pid_set_point_correction->set_entity_category(::ENTITY_CATEGORY_DIAGNOSTIC);
  pid_set_point_correction->set_device_class("temperature");
  pid_set_point_correction->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
  pid_set_point_correction->set_unit_of_measurement("\302\260C");
  pid_set_point_correction->set_accuracy_decimals(1);
  mitsubishiheatpump_id->set_pid_set_point_correction_sensor(pid_set_point_correction);
  device_set_point = new sensor::Sensor();
  App.register_sensor(device_set_point);
  device_set_point->set_name_and_object_id("Device Set Point", "device_set_point");
  device_set_point->set_internal(false);
  device_set_point->set_entity_category(::ENTITY_CATEGORY_DIAGNOSTIC);
  device_set_point->set_device_class("temperature");
  device_set_point->set_state_class(sensor::STATE_CLASS_MEASUREMENT);
  device_set_point->set_unit_of_measurement("\302\260C");
  device_set_point->set_accuracy_decimals(1);
  mitsubishiheatpump_id->set_device_set_point_sensor(device_set_point);
  mitsubishiheatpump_id->set_update_interval(2000);
  mitsubishiheatpump_id->set_component_source(LOG_STR("esphome.coroutine"));
  App.register_component(mitsubishiheatpump_id);
  App.register_climate(mitsubishiheatpump_id);
  mitsubishiheatpump_id->set_name_and_object_id("Test Heatpump", "test_heatpump");
  mitsubishiheatpump_id->set_kp(0.0f);
  mitsubishiheatpump_id->set_ki(0.0f);
  mitsubishiheatpump_id->set_kd(0.0f);
  mitsubishiheatpump_id->set_max_adjustment_under(2.0f);
  mitsubishiheatpump_id->set_max_adjustment_over(2.0f);
  mitsubishiheatpump_id->set_hysterisis_off(0.25f);
  mitsubishiheatpump_id->set_hysterisis_on(0.25f);
  // preferences:
  //   id: preferences_intervalsyncer_id
  //   flash_write_interval: 60s
  preferences_intervalsyncer_id = new preferences::IntervalSyncer();
  preferences_intervalsyncer_id->set_write_interval(60000);
  preferences_intervalsyncer_id->set_component_source(LOG_STR("preferences"));
  App.register_component(preferences_intervalsyncer_id);
  // socket:
  //   implementation: bsd_sockets
  // =========== AUTO GENERATED CODE END ============
  App.setup();
}

void loop() {
  App.loop();
}
