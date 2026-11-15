#ifndef ADAPTIVE_PID_SIMPLE_H
#define ADAPTIVE_PID_SIMPLE_H

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <algorithm>

// PID controller whose output is a setpoint offset, in degrees, around a target.
//
//   offset = kp * error + ki * integral(error) + kd * d(error)/dt
//   error  = target - measured
//
// So kp is "degrees of setpoint per degree of error", ki is the same per second
// of accumulated error, and kd is degrees per (degree per second). The offset is
// limited to [output_min_offset, output_max_offset] and the returned setpoint is
// target + offset. The gains can be replaced while running (see set_gains); what
// chooses them lives outside this class.
class AdaptivePID {
public:
    AdaptivePID(float kp_init, float ki_init, float kd_init);

    // Main control API
    // - current_temp: measured process variable
    // - current_time_ms: monotonic timestamp in milliseconds (e.g., millis())
    // - system_power_on: whether the controlled device is enabled
    // Returns: adjusted setpoint (the target itself while the device is off)
    float update(float current_temp, uint32_t current_time_ms, bool system_power_on);

    // Configure target and direction. Resets the integral and the derivative state.
    void set_target(float target, bool heating);
    float get_target() const;
    bool heating() const { return _heating; }

    // Allowed setpoint offset around the target: min_offset <= 0 <= max_offset.
    void set_output_limits(float min_offset, float max_offset);

    // Replace kp and ki while running. The integral is rescaled so the integral
    // term, and with it the output, does not jump.
    void set_gains(float kp, float ki);

    void set_deadband(float db);                      // small deadband near target (units same as measured)

    // Logging: fills user-provided buffer (low-overhead)
    void log_state(char *buf, size_t buflen) const;

    // Accessors for current gains and state
    float kp() const { return _kp; }
    float ki() const { return _ki; }
    float kd() const { return _kd; }
    float integral() const { return _integral; }

    // A gap between updates longer than this is treated as a restart: nothing is
    // integrated or differentiated across it.
    static constexpr uint32_t MAX_UPDATE_GAP_MS = 30000;

private:
    // PID gains (mutable)
    float _kp, _ki, _kd;

    // Internal state
    float _integral;
    float _prev_error;
    uint32_t _last_update_ms;
    bool _has_state;        // _prev_error and _last_update_ms are valid

    // Output & behavior config
    float _output_min_offset, _output_max_offset;
    float _target;
    bool _heating;   // true: heating, false: cooling. Informational: the error sign already handles both.
    float _deadband;

    // Helpers
    void clamp_integral();
};

#endif // ADAPTIVE_PID_SIMPLE_H
