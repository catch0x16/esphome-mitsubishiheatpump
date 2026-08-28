#include "mitsubishi_cn105.h"

#include <memory>
#include <new>

#include "cn105_connection.h"
#include "cn105_controlflow.h"
#include "cn105_state.h"
#include "floats.h"
#include "uart_io_device.h"

using namespace esphome;
using namespace devicestate;

static const char *const TAG = "mitsubishi_cn105";

MitsubishiCN105Climate::MitsubishiCN105Climate(uart::UARTComponent *uart, uint32_t poll_interval)
    : uart_(uart), update_interval_(poll_interval) {
  this->traits_.add_feature_flags(climate::CLIMATE_SUPPORTS_ACTION |
                                  climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
  this->traits_.set_visual_min_temperature(ESPMHP_MIN_TEMPERATURE);
  this->traits_.set_visual_max_temperature(ESPMHP_MAX_TEMPERATURE);
  this->traits_.set_visual_target_temperature_step(MITSUBISHI_CN105_TARGET_TEMPERATURE_STEP);
  this->traits_.set_visual_current_temperature_step(MITSUBISHI_CN105_CURRENT_TEMPERATURE_STEP);
  this->loop_cycle_.init();
}

void MitsubishiCN105Climate::set_update_interval(uint32_t value) {
  this->update_interval_ = value;
  this->loop_cycle_.setUpdateInterval(value);
}
void MitsubishiCN105Climate::set_remote_temp_timeout(uint32_t value) {
  this->remote_temp_timeout_ = value;
  if (this->control_flow_ != nullptr) {
    this->control_flow_->set_remote_temp_timeout(value);
    this->control_flow_->pingExternalTemperature();
  }
}
void MitsubishiCN105Climate::set_debounce_delay(uint32_t value) {
  this->debounce_delay_ = value;
  if (this->control_flow_ != nullptr) this->control_flow_->set_debounce_delay(value);
}

climate::ClimateTraits MitsubishiCN105Climate::traits() { return this->traits_; }
void MitsubishiCN105Climate::loop() { this->control_flow_->loop(this->loop_cycle_); }

void MitsubishiCN105Climate::terminate_cycle_() {
  this->control_flow_->completeCycle();
  this->device_manager_->update();
  if (this->device_manager_->isInitialized()) {
    this->update_device_();
    this->device_manager_->publish();
  }
  this->loop_cycle_.cycleEnded();
}

void MitsubishiCN105Climate::set_remote_temperature_sensor(sensor::Sensor *sensor) {
  this->remote_temperature_sensor_ = sensor;
  sensor->add_on_state_callback([this](float value) { this->set_remote_temperature(value); });
}
void MitsubishiCN105Climate::set_remote_temperature(float value) {
  if (this->control_flow_ != nullptr) this->control_flow_->setRemoteTemperature(value);
}

void MitsubishiCN105Climate::set_vertical_vane_select(select::Select *value) {
  this->vertical_vane_select_ = value;
  value->add_on_state_callback([this](size_t index) {
    auto option = this->vertical_vane_select_->at(index).value();
    if (option != this->vertical_swing_state_) this->on_vertical_swing_change_(option);
  });
}
void MitsubishiCN105Climate::set_horizontal_vane_select(select::Select *value) {
  this->horizontal_vane_select_ = value;
  value->add_on_state_callback([this](size_t index) {
    auto option = this->horizontal_vane_select_->at(index).value();
    if (option != this->horizontal_swing_state_) this->on_horizontal_swing_change_(option);
  });
}
void MitsubishiCN105Climate::update_swing_vertical_(const std::string &value) {
  this->vertical_swing_state_ = value;
  if (this->vertical_vane_select_ != nullptr && this->vertical_vane_select_->current_option() != value)
    this->vertical_vane_select_->publish_state(value);
}
void MitsubishiCN105Climate::update_swing_horizontal_(const std::string &value) {
  this->horizontal_swing_state_ = value;
  if (this->horizontal_vane_select_ != nullptr && this->horizontal_vane_select_->current_option() != value)
    this->horizontal_vane_select_->publish_state(value);
}

void MitsubishiCN105Climate::on_vertical_swing_change_(const std::string &value) {
  if (value == "swing") this->device_manager_->setVerticalSwingMode(VerticalSwingMode_Swing);
  else if (value == "auto") this->device_manager_->setVerticalSwingMode(VerticalSwingMode_Auto);
  else if (value == "up") this->device_manager_->setVerticalSwingMode(VerticalSwingMode_Up);
  else if (value == "up_center") this->device_manager_->setVerticalSwingMode(VerticalSwingMode_UpCenter);
  else if (value == "center") this->device_manager_->setVerticalSwingMode(VerticalSwingMode_Center);
  else if (value == "down_center") this->device_manager_->setVerticalSwingMode(VerticalSwingMode_DownCenter);
  else if (value == "down") this->device_manager_->setVerticalSwingMode(VerticalSwingMode_Down);
}
void MitsubishiCN105Climate::on_horizontal_swing_change_(const std::string &value) {
  if (value == "swing") this->device_manager_->setHorizontalSwingMode(HorizontalSwingMode_Swing);
  else if (value == "auto") this->device_manager_->setHorizontalSwingMode(HorizontalSwingMode_Auto);
  else if (value == "left") this->device_manager_->setHorizontalSwingMode(HorizontalSwingMode_Left);
  else if (value == "left_center") this->device_manager_->setHorizontalSwingMode(HorizontalSwingMode_LeftCenter);
  else if (value == "center") this->device_manager_->setHorizontalSwingMode(HorizontalSwingMode_Center);
  else if (value == "right_center") this->device_manager_->setHorizontalSwingMode(HorizontalSwingMode_RightCenter);
  else if (value == "right") this->device_manager_->setHorizontalSwingMode(HorizontalSwingMode_Right);
}

void MitsubishiCN105Climate::control(const climate::ClimateCall &call) {
  auto callback = [this, call]() { this->control_delegate_(call); };
  this->control_flow_->acquireWantedSettingsLock(callback);
}

void MitsubishiCN105Climate::control_delegate_(const climate::ClimateCall &call) {
  bool updated = false;
  if (call.get_mode().has_value()) {
    switch (*call.get_mode()) {
      case climate::CLIMATE_MODE_COOL: this->device_manager_->setCool(); break;
      case climate::CLIMATE_MODE_HEAT: this->device_manager_->setHeat(); break;
      case climate::CLIMATE_MODE_DRY: this->device_manager_->setDry(); break;
      case climate::CLIMATE_MODE_AUTO:
      case climate::CLIMATE_MODE_HEAT_COOL: this->device_manager_->setAuto(); break;
      case climate::CLIMATE_MODE_FAN_ONLY: this->device_manager_->setFan(); break;
      default: this->device_manager_->turnOff(); break;
    }
    updated = true;
  }
  if (call.get_target_temperature().has_value()) {
    this->device_manager_->setTargetTemperature(*call.get_target_temperature());
    updated = true;
  }
  if (call.get_fan_mode().has_value()) {
    switch (*call.get_fan_mode()) {
      case climate::CLIMATE_FAN_DIFFUSE:
      case climate::CLIMATE_FAN_QUIET: this->device_manager_->setFanMode(FanMode_Quiet); break;
      case climate::CLIMATE_FAN_LOW: this->device_manager_->setFanMode(FanMode_Low); break;
      case climate::CLIMATE_FAN_MEDIUM: this->device_manager_->setFanMode(FanMode_Medium); break;
      case climate::CLIMATE_FAN_MIDDLE: this->device_manager_->setFanMode(FanMode_Middle); break;
      case climate::CLIMATE_FAN_HIGH: this->device_manager_->setFanMode(FanMode_High); break;
      default: this->device_manager_->setFanMode(FanMode_Auto); break;
    }
    updated = true;
  }
  if (call.get_swing_mode().has_value()) {
    switch (*call.get_swing_mode()) {
      case climate::CLIMATE_SWING_VERTICAL:
        this->device_manager_->setVerticalSwingMode(VerticalSwingMode_Swing);
        this->device_manager_->setHorizontalSwingMode(HorizontalSwingMode_Center);
        break;
      case climate::CLIMATE_SWING_HORIZONTAL:
        this->device_manager_->setVerticalSwingMode(VerticalSwingMode_Center);
        this->device_manager_->setHorizontalSwingMode(HorizontalSwingMode_Swing);
        break;
      case climate::CLIMATE_SWING_BOTH:
        this->device_manager_->setVerticalSwingMode(VerticalSwingMode_Swing);
        this->device_manager_->setHorizontalSwingMode(HorizontalSwingMode_Swing);
        break;
      default:
        this->device_manager_->setVerticalSwingMode(VerticalSwingMode_Auto);
        this->device_manager_->setHorizontalSwingMode(HorizontalSwingMode_Center);
        break;
    }
    updated = true;
  }
  if (updated) this->device_manager_->commit();
}

void MitsubishiCN105Climate::update_device_() {
  const DeviceState state = this->device_manager_->getDeviceState();
  const DeviceStatus status = this->device_manager_->getDeviceStatus();
  this->current_temperature = status.currentTemperature;
  this->target_temperature = state.targetTemperature;

  if (!state.active) {
    this->mode = climate::CLIMATE_MODE_OFF;
    this->action = climate::CLIMATE_ACTION_OFF;
  } else {
    switch (state.mode) {
      case DeviceMode_Heat:
        this->mode = climate::CLIMATE_MODE_HEAT;
        this->action = status.operating ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_IDLE;
        break;
      case DeviceMode_Cool:
        this->mode = climate::CLIMATE_MODE_COOL;
        this->action = status.operating ? climate::CLIMATE_ACTION_COOLING : climate::CLIMATE_ACTION_IDLE;
        break;
      case DeviceMode_Dry:
        this->mode = climate::CLIMATE_MODE_DRY;
        this->action = status.operating ? climate::CLIMATE_ACTION_DRYING : climate::CLIMATE_ACTION_IDLE;
        break;
      case DeviceMode_Fan:
        this->mode = climate::CLIMATE_MODE_FAN_ONLY;
        this->action = climate::CLIMATE_ACTION_FAN;
        break;
      case DeviceMode_Auto:
        this->mode = climate::CLIMATE_MODE_HEAT_COOL;
        this->action = !status.operating ? climate::CLIMATE_ACTION_IDLE
                          : (this->current_temperature > this->target_temperature
                                 ? climate::CLIMATE_ACTION_COOLING
                                 : climate::CLIMATE_ACTION_HEATING);
        break;
      default: break;
    }
  }

  switch (state.fanMode) {
    case FanMode_Quiet: this->fan_mode = climate::CLIMATE_FAN_QUIET; break;
    case FanMode_Low: this->fan_mode = climate::CLIMATE_FAN_LOW; break;
    case FanMode_Medium: this->fan_mode = climate::CLIMATE_FAN_MEDIUM; break;
    case FanMode_Middle: this->fan_mode = climate::CLIMATE_FAN_MIDDLE; break;
    case FanMode_High: this->fan_mode = climate::CLIMATE_FAN_HIGH; break;
    default: this->fan_mode = climate::CLIMATE_FAN_AUTO; break;
  }
  switch (state.swingMode) {
    case SwingMode_Both: this->swing_mode = climate::CLIMATE_SWING_BOTH; break;
    case SwingMode_Vertical: this->swing_mode = climate::CLIMATE_SWING_VERTICAL; break;
    case SwingMode_Horizontal: this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL; break;
    default: this->swing_mode = climate::CLIMATE_SWING_OFF; break;
  }
  switch (state.verticalSwingMode) {
    case VerticalSwingMode_Swing: this->update_swing_vertical_("swing"); break;
    case VerticalSwingMode_Auto: this->update_swing_vertical_("auto"); break;
    case VerticalSwingMode_Up: this->update_swing_vertical_("up"); break;
    case VerticalSwingMode_UpCenter: this->update_swing_vertical_("up_center"); break;
    case VerticalSwingMode_Center: this->update_swing_vertical_("center"); break;
    case VerticalSwingMode_DownCenter: this->update_swing_vertical_("down_center"); break;
    case VerticalSwingMode_Down: this->update_swing_vertical_("down"); break;
    default: break;
  }
  switch (state.horizontalSwingMode) {
    case HorizontalSwingMode_Swing: this->update_swing_horizontal_("swing"); break;
    case HorizontalSwingMode_Auto: this->update_swing_horizontal_("auto"); break;
    case HorizontalSwingMode_Left: this->update_swing_horizontal_("left"); break;
    case HorizontalSwingMode_LeftCenter: this->update_swing_horizontal_("left_center"); break;
    case HorizontalSwingMode_Center: this->update_swing_horizontal_("center"); break;
    case HorizontalSwingMode_RightCenter: this->update_swing_horizontal_("right_center"); break;
    case HorizontalSwingMode_Right: this->update_swing_horizontal_("right"); break;
    default: break;
  }
  this->publish_state();
}

void MitsubishiCN105Climate::setup() {
  this->min_temp_ = ESPMHP_MIN_TEMPERATURE;
  this->max_temp_ = ESPMHP_MAX_TEMPERATURE;
#ifdef USE_CLIMATE_VISUAL_OVERRIDES
  if (this->visual_min_temperature_override_ > this->min_temp_) this->min_temp_ = this->visual_min_temperature_override_;
  if (this->visual_max_temperature_override_ < this->max_temp_) this->max_temp_ = this->visual_max_temperature_override_;
#endif
  auto *io = new (std::nothrow) UARTIODevice(this->uart_);
  if (io == nullptr) { this->mark_failed(); return; }
  auto timeout = [this](const std::string &name, uint32_t delay, std::function<void()> callback) {
    this->set_timeout(name.c_str(), delay, std::move(callback));
  };
  auto retry = [this](const std::string &name, uint32_t delay, uint8_t attempts,
                      std::function<RetryResult(uint8_t)> callback) {
    struct State { std::function<RetryResult(uint8_t)> callback; uint8_t remaining; uint32_t delay; };
    auto state = std::make_shared<State>(State{std::move(callback), attempts, delay});
    auto handler = std::make_shared<std::function<void()>>();
    *handler = [this, name, state, handler]() {
      if (state->remaining == 0) return;
      auto result = state->callback(--state->remaining);
      if (result == RetryResult::DONE || state->remaining == 0) return;
      this->set_timeout(name.c_str(), state->delay, [handler]() { (*handler)(); });
      state->delay = static_cast<uint32_t>(state->delay * 1.2f);
    };
    this->set_timeout(name.c_str(), 0, [handler]() { (*handler)(); });
  };

  this->cn105_state_ = new (std::nothrow) CN105State();
  auto connected = [this](bool value) {
    if (this->device_manager_ != nullptr) this->device_manager_->reset();
    if (value && this->cn105_state_ != nullptr) {
      this->loop_cycle_.lastCompleteCycleMs = CUSTOM_MILLIS;
      this->cn105_state_->resetCurrentSettings();
      this->cn105_state_->resetCurrentRunStates();
    }
    if (this->device_state_connected_ != nullptr) this->device_state_connected_->publish_state(value);
  };
  auto *connection = this->cn105_state_ == nullptr ? nullptr
      : new (std::nothrow) CN105Connection(io, timeout, connected, this->update_interval_);
  this->control_flow_ = connection == nullptr ? nullptr
      : new (std::nothrow) CN105ControlFlow(connection, this->cn105_state_, timeout,
            [this]() { this->terminate_cycle_(); }, retry, this->debounce_delay_, this->remote_temp_timeout_);
  this->device_manager_ = this->cn105_state_ == nullptr ? nullptr
      : new (std::nothrow) DeviceStateManager(this->cn105_state_, this->min_temp_, this->max_temp_,
            this->device_state_active_, this->device_set_point_, this->device_status_operating_,
            this->device_status_current_temperature_, this->device_status_outside_temperature_,
            this->device_status_compressor_frequency_, this->device_status_input_power_,
            this->device_status_kwh_, this->device_status_runtime_hours_);
  if (this->control_flow_ == nullptr || this->device_manager_ == nullptr) { this->mark_failed(); return; }
  this->cn105_state_->resetWantedSettings();
  this->control_flow_->registerInfoRequests();
  if (this->remote_temperature_sensor_ != nullptr && this->remote_temperature_sensor_->has_state())
    this->set_remote_temperature(this->remote_temperature_sensor_->state);
}

void MitsubishiCN105Climate::dump_config() {
  LOG_CLIMATE("", "Mitsubishi CN105 Climate", this);
  ESP_LOGCONFIG(TAG, "  Version: %s", MITSUBISHI_CN105_VERSION);
  ESP_LOGCONFIG(TAG, "  Update interval: %u ms", this->update_interval_);
}
