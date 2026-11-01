#ifndef ADAPTIVE_PID_SIMPLE_H
#define ADAPTIVE_PID_SIMPLE_H

#include <cstdint>
#include <cstdio>
#include "pid_autotuner.h"

class AdaptivePID {
public:
    AdaptivePID(float kp, float ki, float kd, float output_min = 0, float output_max = 100);
    float update(float current_temp, uint32_t current_time_ms, bool system_power_on,
                 bool climate_enabled = true);
    void set_target(float target, bool heating);
    float get_target() const { return _target; }
    bool is_heating() const { return _heating; }
    void set_bounds(float kp_min, float kp_max, float ki_min, float ki_max, float kd_min, float kd_max);
    void set_deadband(float db);
    void set_output_limits(float minimum, float maximum);
    void enable_adaptation(bool enabled) { _adapt_enabled = enabled; }
    void log_state(char* buf, size_t buflen) const;

    float kp() const { return _kp; }
    float ki() const { return _ki; }
    float kd() const { return _kd; }
    float p_term() const { return _p_term; }
    float i_term() const { return _i_term; }
    float d_term() const { return _d_term; }
    const PidAutoTuner& tuner() const { return _tuner; }

private:
    float _kp, _ki, _kd;
    float _integral = 0, _prev_temp = 0, _filtered_derivative = 0;
    uint32_t _last_update_ms = 0;
    bool _was_power_on = false, _adapt_enabled = true;
    float _p_term = 0, _i_term = 0, _d_term = 0;
    float _output_min, _output_max, _command_min, _command_max;
    float _target, _last_applied_setpoint;
    bool _heating = true;
    float _deadband = 0;
    PidAutoTuner _tuner;

    float convert_output_to_setpoint(float output) const;
    void observe_tuning(float temperature, uint32_t now_ms, bool power_on, bool enabled);
};

#endif
