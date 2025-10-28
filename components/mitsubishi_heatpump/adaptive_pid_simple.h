#ifndef ADAPTIVE_PID_SIMPLE_H
#define ADAPTIVE_PID_SIMPLE_H

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <algorithm>

class AdaptivePIDSimple {
public:
    // Constructor: initialize with nominal PID gains and output range (e.g., 0..100)
    AdaptivePIDSimple(float kp_init, float ki_init, float kd_init,
                      float output_min = 0.0f, float output_max = 100.0f);

    // Main control API
    // - current_temp: measured process variable (e.g., temperature)
    // - current_time_ms: monotonic timestamp in milliseconds (e.g., millis())
    // - system_power_on: whether the controlled plant/system is enabled
    // Returns: adjusted setpoint (or neutral setpoint if system off)
    float update(float current_temp, uint32_t current_time_ms, bool system_power_on);

    // Configure target and direction (heating=true means setpoint increases with positive output)
    void set_target(float target, bool heating);

    // Configuration setters
    void set_learning_rates(float lr_kp, float lr_ki, float lr_kd); // per-adapt-step multipliers
    void set_plant_sensitivity(float sens);   // positive scalar (~how strongly control affects measurement)
    void set_bounds(float kp_min, float kp_max, float ki_min, float ki_max, float kd_min, float kd_max);
    void set_adapt_interval_ms(uint32_t interval_ms); // how often to run adaptation (default 15000 ms)
    void set_max_relative_change(float frac);         // e.g., 0.1 => 10% max change per adapt step
    void set_deadband(float db);                      // small deadband near target (units same as measured)

    // Logging: fills user-provided buffer (low-overhead)
    // Format example: "kp=1.234 ki=0.012 kd=0.045 mode=GRAD adapt_ms=15000"
    void log_state(char *buf, size_t buflen) const;

    // Accessors for current gains
    float kp() const { return _kp; }
    float ki() const { return _ki; }
    float kd() const { return _kd; }

    // Optional: quickly enable/disable adaptation (true = enabled)
    void enable_adaptation(bool enabled) { _adapt_enabled = enabled; }

private:
    // PID gains (mutable)
    float _kp, _ki, _kd;

    // Bounds for gains
    float _kp_min, _kp_max;
    float _ki_min, _ki_max;
    float _kd_min, _kd_max;

    // Internal state
    float _integral;
    float _prev_error;
    float _prev_temp;
    uint32_t _last_update_ms;
    uint32_t _last_adapt_ms;

    // Adaptation parameters
    float _lr_kp, _lr_ki, _lr_kd;    // learning rate multipliers (applied per adapt interval)
    float _plant_sensitivity;        // scalar (positive). sign assumed negative-feedback (handled in formula)
    uint32_t _adapt_interval_ms;     // adapt every N ms
    float _max_relative_change;      // fraction cap for per-step relative change
    float _grad_smooth_alpha;        // fixed smoothing alpha in (0,1)
    float _smoothed_dkp, _smoothed_dki, _smoothed_dkd;
    bool _adapt_enabled;

    // Output & behavior config
    float _output_min, _output_max;
    float _target;
    bool _heating;   // true: positive output raises setpoint above target (heating), false: cooling
    float _deadband;

    // Helpers
    float compute_pid_output(float error, float derivative);
    float convert_output_to_setpoint(float control_output, float current_temp) const;
    float apply_safe_update(float currentK, float deltaK, float Kmin, float Kmax) const;
    void run_gradient_adaptation(float error, float integral, float derivative, uint32_t dt_ms);
};

#endif // ADAPTIVE_PID_SIMPLE_H
