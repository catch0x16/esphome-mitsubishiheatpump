#include "adaptive_pid.h"

// -----------------------------
// Constructor & defaults
// -----------------------------
AdaptivePID::AdaptivePID(float kp_init, float ki_init, float kd_init)
    : _kp(kp_init), _ki(ki_init), _kd(kd_init),
      _integral(0.0f), _prev_error(0.0f),
      _last_update_ms(0),
      _has_state(false),
      _output_min_offset(-2.0f), _output_max_offset(2.0f),
      _target(0.0f), _heating(true),
      _deadband(0.0f)
{
}

// -----------------------------
// Public configuration
// -----------------------------
void AdaptivePID::set_target(float target, bool heating) {
    _target = target;
    _heating = heating;
    // Reset internal integrator on target change to avoid windup
    _integral = 0.0f;
    // The error jumps with the target; do not differentiate across the jump.
    _has_state = false;
}

float AdaptivePID::get_target() const {
    return this->_target;
}

void AdaptivePID::set_output_limits(float min_offset, float max_offset) {
    _output_min_offset = std::min(min_offset, 0.0f);
    _output_max_offset = std::max(max_offset, 0.0f);
    clamp_integral();
}

void AdaptivePID::set_gains(float kp, float ki) {
    if (!std::isfinite(kp) || !std::isfinite(ki) || kp < 0.0f || ki < 0.0f) {
        return;
    }
    // Keep ki * integral unchanged so the output does not step.
    if (ki > 0.0f && _ki > 0.0f) {
        _integral *= _ki / ki;
    } else {
        _integral = 0.0f;
    }
    _kp = kp;
    _ki = ki;
    clamp_integral();
}

void AdaptivePID::set_deadband(float db) {
    _deadband = std::abs(db);
}

void AdaptivePID::log_state(char *buf, size_t buflen) const {
    // Small, safe formatted snapshot
    std::snprintf(buf, buflen,
                  "kp=%.4f ki=%.6f kd=%.4f integral=%.3f", _kp, _ki, _kd, _integral);
}

// -----------------------------
// Core update loop
// -----------------------------
float AdaptivePID::update(float current_temp, uint32_t current_time_ms, bool system_power_on) {
    // Error (target - measured)
    float error = _target - current_temp;

    // If system is off, reset integrator (anti-windup) and return neutral setpoint.
    // The timestamp keeps moving so the time spent off is not integrated on resume.
    if (!system_power_on) {
        _integral = 0.0f;
        _prev_error = error;
        _last_update_ms = current_time_ms;
        _has_state = true;
        return _target;
    }

    // First update, a changed target, or a long gap: start from here with no
    // elapsed time, so nothing is integrated or differentiated across the gap.
    uint32_t dt_ms = current_time_ms - _last_update_ms;
    if (!_has_state || dt_ms > MAX_UPDATE_GAP_MS) {
        _prev_error = error;
        dt_ms = 0;
        _has_state = true;
    }
    const float dt = dt_ms / 1000.0f;

    // Apply deadband (small zone of no action)
    if (std::fabs(error) < _deadband) {
        error = 0.0f;
        // gentle decay to integral to avoid windup in deadband
        _integral *= 0.9f;
    }

    // Derivative
    const float derivative = (dt > 0.0f) ? (error - _prev_error) / dt : 0.0f;

    const float p = _kp * error;
    const float d = _kd * derivative;

    // Integral with anti-windup: while the output is pinned at a limit, error
    // that would push it further past that limit is not accumulated.
    const float candidate_integral = _integral + error * dt;
    const float candidate_output = p + _ki * candidate_integral + d;
    const bool winding_up = (candidate_output > _output_max_offset && error > 0.0f) ||
                            (candidate_output < _output_min_offset && error < 0.0f);
    if (!winding_up) {
        _integral = candidate_integral;
    }
    clamp_integral();

    // offset = Kp*e + Ki*integral + Kd*derivative, limited to the allowed range
    float offset = p + _ki * _integral + d;
    if (offset < _output_min_offset) offset = _output_min_offset;
    if (offset > _output_max_offset) offset = _output_max_offset;

    // Update state
    _prev_error = error;
    _last_update_ms = current_time_ms;

    return _target + offset;
}

// -----------------------------
// Helpers
// -----------------------------
void AdaptivePID::clamp_integral() {
    // The integral term on its own never asks for more than the output range.
    if (_ki <= 0.0f) {
        return;
    }
    const float lo = _output_min_offset / _ki;
    const float hi = _output_max_offset / _ki;
    if (_integral < lo) _integral = lo;
    if (_integral > hi) _integral = hi;
}
