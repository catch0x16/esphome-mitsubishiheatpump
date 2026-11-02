#ifndef PID_AUTOTUNER_H
#define PID_AUTOTUNER_H

#include <array>
#include <cstdint>

// A bounded, on-device coordinate search. This tests the existing controller;
// it does not switch the heat pump on/off or inject relay-test oscillations.
class PidAutoTuner {
public:
    using Gains = std::array<float, 3>;  // Kp, Ki, Kd
    enum class Stage { DISABLED, BASELINE, TRIAL, REFERENCE, CONFIRM, HOLD };
    struct Config {
        uint32_t window_ms = 3600000;
        uint32_t settling_ms = 600000;
        uint32_t minimum_active_ms = 900000;
        uint32_t minimum_cycling_active_ms = 300000;
        uint32_t hold_ms = 21600000;
        float improvement = 0.05f;
    };

    explicit PidAutoTuner(Gains initial, float output_half_span = 3.75f);
    void configure(const Config& config);
    void set_bounds(Gains minimum, Gains maximum);
    void reset_context() { restart(); }
    void observe(float temperature, float target, bool heating, bool power_on,
                 bool enabled, uint32_t now_ms, float applied_setpoint);
    Gains gains() const { return gains_; }
    Stage stage() const { return stage_; }
    const char* status() const;
    float baseline_score() const;
    float trial_score() const;
    uint32_t accepted() const { return accepted_; }
    uint32_t rejected() const { return rejected_; }
    uint32_t discarded() const { return discarded_; }

private:
    struct Window {
        uint32_t elapsed_ms = 0, active_ms = 0;
        float cost = 0, peak_over = 0, start_error = 0, movement = 0;
        uint32_t shutdowns = 0, complete_cycles = 0;
        float score() const;
    };
    Config config_;
    Gains gains_, best_, candidate_;
    Gains minimum_{{0, 0, 0}}, maximum_{{126, 10, 7200}};
    Stage stage_ = Stage::DISABLED;
    Window window_, baseline_, trial_, reference_;
    uint32_t last_ms_ = 0, settling_left_ms_ = 0, hold_start_ms_ = 0;
    uint32_t accepted_ = 0, rejected_ = 0, discarded_ = 0;
    bool cycle_started_ = false;
    bool have_sample_ = false, last_power_ = false, have_command_ = false;
    float last_temperature_ = 0, target_ = 0, last_command_ = 0;
    bool heating_ = true;
    unsigned probe_ = 0;  // Ki down/up, Kp down/up, Kd down/up
    float probe_output_ = 0;
    std::array<float, 3> step_scale_{{1, 1, 1}};

    void begin(Stage stage);
    void restart(bool reset_search = true);
    void discard_window();
    void next_candidate(uint32_t now_ms);
    void finish(uint32_t now_ms);
    bool comparable(const Window& a, const Window& b) const;
    bool improved(const Window& candidate, const Window& baseline) const;
};

#endif
