#include "adaptive_pid.h"

// -----------------------------
// Constructor & defaults
// -----------------------------
AdaptivePID::AdaptivePID(float kp_init, float ki_init, float kd_init,
                                     float output_min, float output_max)
    : _kp(kp_init), _ki(ki_init), _kd(kd_init),
      _kp_min(0.0f), _kp_max(100.0f),
      _ki_min(0.0f), _ki_max(10.0f),
      _kd_min(0.0f), _kd_max(50.0f),
      _integral(0.0f), _prev_error(0.0f), _prev_temp(0.0f),
      _last_update_ms(0), _last_adapt_ms(0),
      _lr_kp(0.02f), _lr_ki(0.005f), _lr_kd(0.005f),
      _plant_sensitivity(1.0f),
      _adapt_interval_ms(15000), // adapt every 15 seconds by default
      _max_relative_change(0.10f), // 10% per adapt step
      _grad_smooth_alpha(0.08f),   // smoothing alpha (8%) for gradient deltas
      _smoothed_dkp(0.0f), _smoothed_dki(0.0f), _smoothed_dkd(0.0f),
      _adapt_enabled(true),
      _output_min(output_min), _output_max(output_max),
      _target((output_min + output_max) / 2.0f), _heating(true),
      _deadband(0.0f)
{
    // Ensure sane bounds relative to initial gains
    _kp_min = std::max(0.0f, _kp * 0.01f);
    _kp_max = std::max(_kp_min + 1e-6f, _kp * 100.0f + 1.0f);
}

// -----------------------------
// Public configuration
// -----------------------------
void AdaptivePID::set_target(float target, bool heating) {
    _target = target;
    _heating = heating;
    // Reset internal integrator on target change to avoid windup
    _integral = 0.0f;
}

float AdaptivePID::get_target() {
    return this->_target;
}

void AdaptivePID::set_learning_rates(float lr_kp, float lr_ki, float lr_kd) {
    _lr_kp = lr_kp;
    _lr_ki = lr_ki;
    _lr_kd = lr_kd;
}

void AdaptivePID::set_plant_sensitivity(float sens) {
    _plant_sensitivity = std::fabs(sens) + 1e-9f; // keep positive
}

void AdaptivePID::set_bounds(float kp_min, float kp_max, float ki_min, float ki_max, float kd_min, float kd_max) {
    _kp_min = kp_min; _kp_max = kp_max;
    _ki_min = ki_min; _ki_max = ki_max;
    _kd_min = kd_min; _kd_max = kd_max;
}

void AdaptivePID::set_adapt_interval_ms(uint32_t interval_ms) {
    _adapt_interval_ms = std::max<uint32_t>(1000u, interval_ms);
}

void AdaptivePID::set_max_relative_change(float frac) {
    _max_relative_change = std::max(0.001f, frac);
}

void AdaptivePID::set_deadband(float db) {
    _deadband = std::abs(db);
}

void AdaptivePID::log_state(char *buf, size_t buflen) const {
    // Small, safe formatted snapshot
    std::snprintf(buf, buflen,
                  "kp=%.4f ki=%.6f kd=%.4f adapt_ms=%u lr=(%.4f,%.4f,%.4f)",
                  _kp, _ki, _kd, (unsigned)_adapt_interval_ms, _lr_kp, _lr_ki, _lr_kd);
}

// -----------------------------
// Core update loop
// -----------------------------
float AdaptivePID::update(float current_temp, uint32_t current_time_ms, bool system_power_on) {
    // Guard against zero-time initial calls: initialize last_update
    if (_last_update_ms == 0) {
        _last_update_ms = current_time_ms;
        _last_adapt_ms = current_time_ms;
        _prev_temp = current_temp;
        _prev_error = _target - current_temp;
    }

    // If system is off, reset integrator (anti-windup) and return neutral setpoint
    if (!system_power_on) {
        _integral = 0.0f;
        // Neutral behavior: return target to avoid aggressive moves
        return _target;
    }

    // dt for PID calculation (seconds)
    uint32_t dt_ms = current_time_ms - _last_update_ms;
    if (dt_ms == 0) dt_ms = 1;
    float dt = dt_ms / 1000.0f;

    // Error (target - measured)
    float error = _target - current_temp;

    // Apply deadband (small zone of no action)
    if (std::fabs(error) < _deadband) {
        error = 0.0f;
        // gentle decay to integral to avoid windup in deadband
        _integral *= 0.9f;
    }

    // Integral update with clamping (anti-windup)
    _integral += error * dt;
    // clamp integral to keep i-term reasonable (bound based on ki_max if available)
    float i_cap = (_ki_max > 0.0f) ? (10.0f / _ki_max) : 1e6f; // heuristic cap
    if (std::fabs(_integral) > i_cap) _integral = ( _integral > 0 ? i_cap : -i_cap );

    // Derivative
    float derivative = (dt > 0.0f) ? (error - _prev_error) / dt : 0.0f;

    // Compute control output u = Kp*e + Ki*integral + Kd*derivative
    float control_output = compute_pid_output(error, derivative);

    // Convert control output into a setpoint (bounded)
    float adjusted_setpoint = convert_output_to_setpoint(control_output, current_temp);

    // Possibly run adaptation if enough time passed
    if (_adapt_enabled && (uint32_t)(current_time_ms - _last_adapt_ms) >= _adapt_interval_ms) {
        run_gradient_adaptation(error, _integral, derivative, _adapt_interval_ms);
        _last_adapt_ms = current_time_ms;
    }

    // Update state
    _prev_error = error;
    _prev_temp = current_temp;
    _last_update_ms = current_time_ms;

    return adjusted_setpoint;
}

// -----------------------------
// Helpers
// -----------------------------
float AdaptivePID::compute_pid_output(float error, float derivative) {
    // Basic PID output composition
    float p = _kp * error;
    float i = _ki * _integral;
    float d = _kd * derivative;
    float out = p + i + d;
    // No additional clamping here; convert_output_to_setpoint will clamp setpoint
    return out;
}

float AdaptivePID::convert_output_to_setpoint(float control_output, float current_temp) const {
    // Map control output to a setpoint range around target; here control_output is unbounded,
    // but we'll project it to a ±span and clamp to output limits.
    // Choose a mapping span: 100 units of control_output maps to full range by default.
    const float map_scale = 100.0f;
    float frac = std::tanh(control_output / map_scale); // squashes large values safely between -1 and 1

    float half_span = (_output_max - _output_min) * 0.5f;
    float center = _target;
    float setpoint_offset = frac * half_span;

    float sp = _heating ? (center + setpoint_offset) : (center - setpoint_offset);
    // Clamp to output bounds
    if (sp < _output_min) sp = _output_min;
    if (sp > _output_max) sp = _output_max;
    return sp;
}

float AdaptivePID::apply_safe_update(float currentK, float deltaK, float Kmin, float Kmax) const {
    if (!std::isfinite(deltaK)) return currentK;
    // Cap by relative fraction of absolute currentK magnitude
    float cap = std::max(1e-6f, std::fabs(currentK) * _max_relative_change);
    if (deltaK > cap) deltaK = cap;
    if (deltaK < -cap) deltaK = -cap;
    float newK = currentK + deltaK;
    if (newK < Kmin) newK = Kmin;
    if (newK > Kmax) newK = Kmax;
    return newK;
}

// -----------------------------
// Gradient adaptation (surrogate gradient, simplified & smoothed)
// -----------------------------
void AdaptivePID::run_gradient_adaptation(float error, float integral, float derivative, uint32_t dt_ms) {
    // Use simple surrogate gradient: raw_dK = plant_sensitivity * error * phi
    // where phi is controller regressor: [e, integral, derivative]
    // We negate expected sign because d(error)/d(control) is negative for negative feedback.
    // So we add dK = + lr * plant_sensitivity * error * phi.
    // (Users can tune learning rates and plant_sensitivity.)
    float phi_p = error;
    float phi_i = integral;
    float phi_d = derivative;

    // raw deltas (per adapt interval)
    float raw_dkp = _plant_sensitivity * error * phi_p;
    float raw_dki = _plant_sensitivity * error * phi_i;
    float raw_dkd = _plant_sensitivity * error * phi_d;

    // scale by configured learning-rate multipliers and by normalized time (ms -> seconds)
    float dt_s = std::max(0.001f, dt_ms / 1000.0f);
    float delta_kp = _lr_kp * raw_dkp * dt_s;
    float delta_ki = _lr_ki * raw_dki * dt_s;
    float delta_kd = _lr_kd * raw_dkd * dt_s;

    // Smooth deltas (fixed alpha) to damp noisy estimates
    _smoothed_dkp += _grad_smooth_alpha * (delta_kp - _smoothed_dkp);
    _smoothed_dki += _grad_smooth_alpha * (delta_ki - _smoothed_dki);
    _smoothed_dkd += _grad_smooth_alpha * (delta_kd - _smoothed_dkd);

    // Apply safe updates with caps and bounds
    _kp = apply_safe_update(_kp, _smoothed_dkp, _kp_min, _kp_max);
    _ki = apply_safe_update(_ki, _smoothed_dki, _ki_min, _ki_max);
    _kd = apply_safe_update(_kd, _smoothed_dkd, _kd_min, _kd_max);

    // Ensure gains remain finite and sane
    if (!std::isfinite(_kp)) _kp = (_kp_min + _kp_max) * 0.5f;
    if (!std::isfinite(_ki)) _ki = (_ki_min + _ki_max) * 0.5f;
    if (!std::isfinite(_kd)) _kd = (_kd_min + _kd_max) * 0.5f;
}
