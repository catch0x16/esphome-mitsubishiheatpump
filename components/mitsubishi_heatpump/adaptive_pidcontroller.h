#ifndef APIDC_H
#define APIDC_H

/**
 * Adaptive PID Controller for ESPHome HVAC Temperature Regulation
 * 
 * This controller automatically adjusts PID parameters based on system response
 * to achieve stable temperature control without overshoot.
 */

#include <cmath>
#include <algorithm>

#include "floats.h"

class AdaptivePIDController {
private:
    float target;
    bool direction;

    // PID Parameters
    float kp;              // Proportional gain
    float ki;              // Integral gain
    float kd;              // Derivative gain

    float sampleTime;      // Sample time in seconds
    
    // PID State variables
    float integral;        // Accumulated integral error
    float prev_error;      // Previous error for derivative calculation
    float prev_temp;       // Previous temperature reading
    
    // Output limits (clamping bounds)
    float outputMin;      // Lower bound (for cooling)
    float outputMax;      // Upper bound (for heating)

    // Output limits (clamping bounds)
    float maxAdjustmentUnder;      // Lower bound for relative adjustment
    float maxAdjustmentOver;       // Upper bound for relative adjustment

    float adjustedMin;
    float adjustedMax;

    // Adaptive tuning variables
    float error_history[10];  // Recent error samples for adaptation
    int history_index;        // Current position in history array
    float adaptation_rate;    // How quickly to adjust parameters

    // Anti-windup and stability
    float integral_max;    // Maximum integral to prevent windup

    // Timing
    unsigned long last_update_time;  // Timestamp of last update (milliseconds)

    bool system_active;       // Current system state (on/off)
    unsigned long system_off_time;  // When system was turned off
    unsigned long system_on_time;   // When system was turned on

    int overshoot_count;      // Track how many times system was disabled

public:
    /**
     * Constructor - Initialize the PID controller with conservative defaults
     * These values work well for room temperature control (slow, stable response)
     */
    AdaptivePIDController(
            const float p,
            const float i,
            const float d,
            const float outputMin,
            const float outputMax,
            const float maxAdjustmentOver,
            const float maxAdjustmentUnder
    ) {
        // Start with conservative PID values for room temperature
        //kp = 5.0f;           // Proportional: moderate response
        //ki = 0.1f;           // Integral: slow to avoid overshoot
        //kd = 1.0f;           // Derivative: dampen oscillations
        this->kp = p;
        this->ki = i;
        this->kd = d;

        // Initialize state
        this->integral = 0.0f;
        this->prev_error = 0.0f;
        this->prev_temp = 0.0f;
        
        // Set reasonable output limits (0-100% typical for HVAC)
        this->outputMin = outputMin;
        this->outputMax = outputMax;

        this->maxAdjustmentUnder = maxAdjustmentUnder;
        this->maxAdjustmentOver = maxAdjustmentOver;
        
        // Adaptive tuning settings
        this->history_index = 0;
        this->adaptation_rate = 0.05f;  // Gradual adaptation
        for (int i = 0; i < 10; i++) {
            this->error_history[i] = 0.0f;
        }
        
        // Anti-windup and stability
        this->integral_max = 20.0f;     // Limit integral accumulation
        
        this->last_update_time = 0;
        this->system_active = true;     // Start with system enabled
        this->system_off_time = 0;
        this->system_on_time = 0;
        this->overshoot_count = 0;      // Track system disable events

        this->setTarget((outputMax + outputMin) / 2.0, false);
    }

    float getTarget() {
        return this->target;
    }

    void adjustOutputLimits(const bool direction) {
        const float adjustMinOffset = direction ? this->maxAdjustmentUnder : this->maxAdjustmentOver;
        const float adjustMaxOffset = direction ? this->maxAdjustmentOver : this->maxAdjustmentUnder;

        this->adjustedMin = devicestate::clamp(this->target - adjustMinOffset, this->outputMin, this->outputMax);
        this->adjustedMax = devicestate::clamp(this->target + adjustMaxOffset, this->outputMin, this->outputMax);
    }

    void setTarget(const float target, const bool direction) {
        this->target = target;
        this->direction = direction;
        this->adjustOutputLimits(direction);
        this->resetState();
    }

    float convert_output_to_setpoint(float control_output, float current_temp) {
        float setpoint = this->direction
            ? this->target + (control_output / 100.0f) * (this->adjustedMax - this->target)
            : this->target - (control_output / 100.0f) * (this->target - this->adjustedMin);
        return std::max(this->adjustedMin, std::min(setpoint, this->adjustedMax));
    }

    void set_system_state(bool system_active, float current_temp, unsigned long current_time) {
        if (this->system_active == system_active) {
            return;
        }

        if (system_active) {
            this->system_on_time = current_time;
            // Reset PID state for clean restart
            this->prev_error = this->target - current_temp;
            this->integral = 0.0f;
        } else {
            this->system_off_time = current_time;
            // Reset integral to prevent it from continuing to accumulate
            this->integral = 0.0f;
        }
        this->system_active = system_active;
    }

    /**
     * Main update function - Call this repeatedly with current measurements
     * 
     * @param current_temp   Current measured temperature
     * @param current_time   Current timestamp in milliseconds
     * @return               Control output value (clamped to bounds)
     */
    float update(float current_temp, unsigned long current_time, bool isInternalPowerOn) {
        // Assume if isInternalPowerOn == false, then hysteresis triggered
        if (this->system_active != isInternalPowerOn) {
            if (isInternalPowerOn) {
                this->notify_system_disabled(current_time);
            } else {
                this->notify_system_enabled(current_temp, current_time);
            }
            this->system_active = isInternalPowerOn;
        }
        // If system is off due to hysteresis, return target temp (no action)
        if (!this->system_active) {
            // Don't update error history during off periods
            // This prevents the adaptive tuner from trying to compensate for forced shutdowns
            
            return this->target;  // System off - send neutral setpoint
        }

        // Calculate time delta in seconds
        float dt = (last_update_time == 0) ? 0.1f : (current_time - last_update_time) / 1000.0f;
        last_update_time = current_time;
        
        // Ensure reasonable dt bounds
        dt = std::max(0.01f, std::min(dt, 1.0f));
        
        // Calculate error (how far we are from target)
        float error = this->target - current_temp;

        // Store error in history for adaptive tuning
        error_history[history_index] = error;
        history_index = (history_index + 1) % 10;
        
        // PROPORTIONAL TERM
        // Direct response to current error - larger error = larger correction
        float p_term = kp * error;
        
        // INTEGRAL TERM
        // Accumulates error over time - eliminates steady-state offset
        integral += error * dt;
        // Anti-windup: limit integral to prevent overshooting
        integral = std::max(-integral_max, std::min(integral, integral_max));
        float i_term = ki * integral;
        
        // DERIVATIVE TERM
        // Responds to rate of change - dampens oscillations
        float derivative = (dt > 0.0f) ? (error - prev_error) / dt : 0.0f;
        float d_term = kd * derivative;
        
        // Calculate total PID output
        float output = p_term + i_term + d_term;
        
        // Clamp the output to adjusted min/max
        float adjustedSetpoint = convert_output_to_setpoint(output, current_temp);

        // Adaptive parameter adjustment
        adapt_parameters(error, current_temp, dt);
        
        // Update state for next iteration
        prev_error = error;
        prev_temp = current_temp;
        
        return adjustedSetpoint;
    }

    /**
     * Notify controller that system has been disabled (e.g., by external hysteresis logic)
     * Call this when your external logic determines the system should turn off
     * 
     * @param current_time   Timestamp when system was disabled
     */
    void notify_system_disabled(unsigned long current_time) {
        if (system_active) {  // Only process if transitioning from ON to OFF
            system_active = false;
            system_off_time = current_time;
            overshoot_count++;
            
            // LEARN FROM OVERSHOOT: Reduce proportional gain to be less aggressive
            float reduction_factor = 0.95f;  // 5% reduction per disable event
            kp *= reduction_factor;
            kp = std::max(kp, 2.0f);  // Don't go below minimum
            
            // Also reduce integral cap to prevent future windup
            integral_max *= 0.98f;
            integral_max = std::max(integral_max, 10.0f);
            
            // Reset integral to prevent windup
            integral = 0.0f;
        }
    }
    
    /**
     * Notify controller that system has been enabled (e.g., by external hysteresis logic)
     * Call this when your external logic determines the system should turn on
     * 
     * @param current_time   Timestamp when system was enabled
     * @param current_error  Current temperature error (target - current)
     */
    void notify_system_enabled(float current_temp, unsigned long current_time) {
        if (!system_active) {  // Only process if transitioning from OFF to ON
            system_active = true;
            system_on_time = current_time;
            
            // Reset PID state for clean restart
            prev_error = this->target - current_temp;
            integral = 0.0f;
        }
    }

    /**
     * Adaptive parameter tuning - automatically adjusts PID gains
     * based on system response to improve performance over time
     * 
     * MODIFIED for hysteresis: Adapts more conservatively to avoid
     * confusing forced shutdowns with control problems
     */
    void adapt_parameters(float current_error, float current_temp, float dt) {
        // After hysteresis turns system back on, allow stabilization period
        // before making adaptations (system was just restarted with zeroed integral)
        unsigned long time_since_on = last_update_time - system_on_time;
        
        if (time_since_on < 300000) {  // 5 minutes stabilization after restart
            return;  // Don't adapt immediately after turning back on
        }
        
        // Calculate metrics from error history
        float error_variance = calculate_variance();
        float error_mean = calculate_mean();
        
        // Calculate temperature rate of change
        float temp_rate = (dt > 0.0f) ? (current_temp - prev_temp) / dt : 0.0f;
        
        // ADAPTATION RULES (modified for hysteresis):
        
        // 1. If system oscillates (high variance), reduce proportional gain
        //    BUT: Be more conservative - hysteresis causes natural cycling
        if (error_variance > 2.0f) {  // Higher threshold than before (was 1.0)
            kp *= (1.0f - adaptation_rate * 0.5f);  // Slower adaptation
            kp = std::max(kp, 1.0f);
        }
        
        // 2. If steady-state error exists, increase integral gain cautiously
        //    Lower cap to reduce risk of overshoot that triggers hysteresis
        if (std::abs(error_mean) > 0.5f && error_variance < 0.5f) {
            ki *= (1.0f + adaptation_rate * 0.5f);  // Slower adaptation
            ki = std::min(ki, 0.5f);  // Lower cap than non-hysteresis mode (was 1.0)
        }
        
        // 3. If temperature changes too rapidly, increase derivative gain
        //    This helps dampen oscillations caused by hysteresis cycling
        if (std::abs(temp_rate) > 0.5f) {
            kd *= (1.0f + adaptation_rate);
            kd = std::min(kd, 5.0f);
        }
        
        // 4. If system is stable, slightly reduce integral
        if (error_variance < 0.2f && std::abs(current_error) < 0.5f) {
            ki *= (1.0f - adaptation_rate * 0.5f);
            ki = std::max(ki, 0.05f);
        }
    }
    
    /**
     * Calculate variance of recent errors - measures oscillation
     */
    float calculate_variance() {
        float mean = calculate_mean();
        float sum_squared_diff = 0.0f;
        
        for (int i = 0; i < 10; i++) {
            float diff = error_history[i] - mean;
            sum_squared_diff += diff * diff;
        }
        
        return sum_squared_diff / 10.0f;
    }
    
    /**
     * Calculate mean of recent errors - measures steady-state offset
     */
    float calculate_mean() {
        float sum = 0.0f;
        for (int i = 0; i < 10; i++) {
            sum += error_history[i];
        }
        return sum / 10.0f;
    }

    /**
     * Get current system state (useful for monitoring)
     */
    bool is_system_active() const {
        return system_active;
    }

    /**
     * Get overshoot/disable statistics (for monitoring/debugging)
     */
    int get_disable_count() const {
        return overshoot_count;
    }
    
    float get_time_since_last_disable() const {
        if (system_off_time == 0 || last_update_time == 0) return -1.0f;
        return (last_update_time - system_off_time) / 1000.0f;
    }
    /**
     * Get time since last state change (in seconds)
     */
    float get_time_since_state_change() const {
        if (last_update_time == 0) return 0.0f;
        unsigned long state_change_time = system_active ? system_on_time : system_off_time;
        return (last_update_time - state_change_time) / 1000.0f;
    }

    /**
     * Reset controller state (call when mode changes or on startup)
     */
    void resetState() {
        integral = 0.0f;
        prev_error = 0.0f;
        prev_temp = 0.0f;
        for (int i = 0; i < 10; i++) {
            error_history[i] = 0.0f;
        }
        system_active = true;
        system_off_time = 0;
        system_on_time = 0;
        overshoot_count = 0;
        last_update_time = 0;
    }
    
    /**
     * Get current PID parameters (for monitoring/debugging)
     */
    float get_kp() const { return kp; }
    float get_ki() const { return ki; }
    float get_kd() const { return kd; }

    float getAdjustedMin() const { return this->adjustedMin; }
    float getAdjustedMax() const { return this->adjustedMax; }
};

#endif