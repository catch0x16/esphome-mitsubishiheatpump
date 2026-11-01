#include "adaptive_pid.h"

#include <algorithm>
#include <cmath>

AdaptivePID::AdaptivePID(float kp, float ki, float kd, float output_min, float output_max)
    : _kp(kp), _ki(ki), _kd(kd), _output_min(output_min), _output_max(output_max),
      _command_min(output_min), _command_max(output_max),
      _target((output_min + output_max) / 2), _last_applied_setpoint(_target),
      _tuner({{kp, ki, kd}}, (output_max - output_min) * 0.5f) {}

void AdaptivePID::set_target(float target, bool heating) {
    if (target == _target && heating == _heating) return;
    _target = target;
    _heating = heating;
    _integral = 0;
    _filtered_derivative = 0;
    _was_power_on = false;
    _tuner.reset_context();
    const auto gains = _tuner.gains();
    _kp = gains[0]; _ki = gains[1]; _kd = gains[2];
}

void AdaptivePID::set_bounds(float kp_min, float kp_max, float ki_min, float ki_max,
                             float kd_min, float kd_max) {
    _tuner.set_bounds({{kp_min, ki_min, kd_min}}, {{kp_max, ki_max, kd_max}});
    const auto gains = _tuner.gains();
    _kp = gains[0]; _ki = gains[1]; _kd = gains[2];
}

void AdaptivePID::set_deadband(float db) { _deadband = std::fabs(db); }

void AdaptivePID::set_output_limits(float minimum, float maximum) {
    _command_min = std::max(_output_min, std::min(_output_max, minimum));
    _command_max = std::max(_command_min, std::min(_output_max, maximum));
}

void AdaptivePID::log_state(char* buf, size_t buflen) const {
    std::snprintf(buf, buflen, "kp=%.4f ki=%.6f kd=%.4f tuning=%s accepted=%u rejected=%u",
                  _kp, _ki, _kd, _tuner.status(),
                  static_cast<unsigned>(_tuner.accepted()), static_cast<unsigned>(_tuner.rejected()));
}

void AdaptivePID::observe_tuning(float temperature, uint32_t now_ms, bool power_on, bool enabled) {
    _tuner.observe(temperature, _target, _heating, power_on, enabled && _adapt_enabled,
                   now_ms, _last_applied_setpoint);
    const auto gains = _tuner.gains();
    _kp = gains[0]; _ki = gains[1]; _kd = gains[2];
}

float AdaptivePID::update(float current_temp, uint32_t now_ms, bool system_power_on,
                          bool climate_enabled) {
    if (!std::isfinite(current_temp) || !system_power_on) {
        _integral = _filtered_derivative = 0;
        _p_term = _i_term = _d_term = 0;
        _last_update_ms = now_ms;
        _prev_temp = current_temp;
        _was_power_on = false;
        observe_tuning(current_temp, now_ms, false, climate_enabled);
        return _target;
    }
    uint32_t dt_ms = now_ms - _last_update_ms;
    if (!_was_power_on || dt_ms > 30000) {
        dt_ms = 0;
        _prev_temp = current_temp;
        _integral = _filtered_derivative = 0;
    }
    const float dt = dt_ms / 1000.0f;
    float error = _target - current_temp;
    if (std::fabs(error) < _deadband) {
        error = 0;
        _integral *= 0.9f;
    }
    // Preserve the existing accumulator cap independently of gain search bounds.
    _integral = std::max(-1.0f, std::min(1.0f, _integral + error * dt));

    // Derivative on measurement avoids target-change kick. A 30-second filter
    // spreads discrete sensor updates over time rather than amplifying a single
    // update as if the entire change happened in one UART polling interval.
    if (dt > 0) {
        const float derivative = -(current_temp - _prev_temp) / dt;
        _filtered_derivative += dt / (30.0f + dt) * (derivative - _filtered_derivative);
    }
    _p_term = _kp * error;
    _i_term = _ki * _integral;
    _d_term = _kd * _filtered_derivative;
    const float output = convert_output_to_setpoint(_p_term + _i_term + _d_term);
    _last_applied_setpoint = std::round(output * 2.0f) / 2.0f;
    _prev_temp = current_temp;
    _last_update_ms = now_ms;
    _was_power_on = true;
    // New gains apply on the next calculation; diagnostics snapshot gains before
    // calling update so the reported gains and contributions remain consistent.
    observe_tuning(current_temp, now_ms, true, climate_enabled);
    return output;
}

float AdaptivePID::convert_output_to_setpoint(float output) const {
    const float offset = std::tanh(output / 100.0f) * (_output_max - _output_min) * 0.5f;
    return std::max(_command_min, std::min(_command_max, _target + offset));
}
