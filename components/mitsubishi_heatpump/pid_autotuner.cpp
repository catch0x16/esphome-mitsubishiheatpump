#include "pid_autotuner.h"

#include <algorithm>
#include <cmath>
#include <limits>

PidAutoTuner::PidAutoTuner(Gains initial, float output_half_span)
    : gains_(initial), best_(initial), candidate_(initial), initial_(initial) {
    maximum_[0] = std::max(100.0f, initial[0] * 100.0f + 1.0f);
    // A configured starting gain must remain representable.
    for (unsigned i = 0; i < 3; ++i) maximum_[i] = std::max(maximum_[i], initial[i]);
    // Probe enough raw output to produce a quarter-degree correction near the
    // center of the existing tanh mapping. Tiny gain changes disappear in the
    // unit's half-degree command quantization and cannot be evaluated.
    const float fraction = std::min(0.5f, 0.25f / std::max(0.01f, output_half_span));
    probe_output_ = 100.0f * std::atanh(fraction);
}

void PidAutoTuner::configure(const Config& config) {
    config_ = config;
    config_.window_ms = std::max<uint32_t>(1, config_.window_ms);
    config_.minimum_active_ms = std::min(config_.minimum_active_ms, config_.window_ms);
    config_.minimum_cycling_active_ms = std::min(config_.minimum_cycling_active_ms, config_.window_ms);
    config_.improvement = std::max(0.01f, std::min(0.5f, config_.improvement));
    restart();
}

void PidAutoTuner::set_bounds(Gains minimum, Gains maximum) {
    for (unsigned i = 0; i < 3; ++i) {
        if (!std::isfinite(minimum[i]) || !std::isfinite(maximum[i])) return;
        if (minimum[i] < 0 || maximum[i] < minimum[i]) return;
    }
    minimum_ = minimum;
    maximum_ = maximum;
    for (unsigned i = 0; i < 3; ++i)
        best_[i] = std::max(minimum_[i], std::min(maximum_[i], best_[i]));
    restart();
}

float PidAutoTuner::Window::score() const {
    if (elapsed_ms == 0) return std::numeric_limits<float>::quiet_NaN();
    const float hours = elapsed_ms / 3600000.0f;
    // Error is integrated in seconds. Overshoot has a stronger penalty than
    // unmet demand; peaks and shutdowns cannot disappear into a good average.
    return cost / (elapsed_ms / 1000.0f) + 2.0f * peak_over +
           0.1f * shutdowns / hours + 0.01f * movement / hours;
}

float PidAutoTuner::baseline_score() const { return baseline_.score(); }
float PidAutoTuner::trial_score() const { return trial_.score(); }

const char* PidAutoTuner::status() const {
    switch (stage_) {
        case Stage::DISABLED: return "disabled";
        case Stage::BASELINE: return "baseline";
        case Stage::REFERENCE: return "rechecking_baseline";
        case Stage::CONFIRM: return "confirming_trial";
        case Stage::HOLD: return ready_to_save_ ? "ready_to_save" : "holding_best";
        case Stage::TRIAL: {
            static const char* names[] = {"testing_ki_down", "testing_ki_up", "testing_kp_down",
                                          "testing_kp_up", "testing_kd_down", "testing_kd_up"};
            return names[probe_];
        }
    }
    return "disabled";
}

void PidAutoTuner::begin(Stage stage) {
    stage_ = stage;
    window_ = Window{};
    settling_left_ms_ = config_.settling_ms;
    have_command_ = false;
    cycle_started_ = false;
    ready_to_save_ = false;
}

void PidAutoTuner::restart(bool reset_search) {
    gains_ = best_;
    baseline_ = trial_ = reference_ = Window{};
    if (reset_search) {
        probe_ = 0;
        evaluated_probes_ = 0;
    }
    begin(Stage::BASELINE);
    have_sample_ = false;
}

void PidAutoTuner::discard_window() {
    ++discarded_;
    if (stage_ == Stage::TRIAL || stage_ == Stage::REFERENCE || stage_ == Stage::CONFIRM)
        ++probe_;
    // Inconclusive data must restore the accepted gains without repeatedly
    // sending the search back to its first Ki-down candidate.
    restart(false);
}

bool PidAutoTuner::comparable(const Window& a, const Window& b) const {
    return std::fabs(a.start_error - b.start_error) <= 0.5f;
}

bool PidAutoTuner::improved(const Window& candidate, const Window& baseline) const {
    return comparable(candidate, baseline) &&
           candidate.score() < baseline.score() * (1.0f - config_.improvement) &&
           candidate.peak_over <= baseline.peak_over + 0.1f;
}

void PidAutoTuner::next_candidate(uint32_t now_ms) {
    static const unsigned coordinates[] = {1, 1, 0, 0, 2, 2};
    for (; probe_ < 6; ++probe_) {
        candidate_ = best_;
        const unsigned coordinate = coordinates[probe_];
        // Observable probes at 0.5°C error, a saturated integral, or a filtered
        // 0.1°C/min trend. Refine the absolute floors after successful trials.
        const float floors[] = {probe_output_ / 0.5f, probe_output_, probe_output_ * 600.0f};
        const float step = std::max(std::fabs(best_[coordinate]) * 0.1f,
                                    floors[coordinate] * step_scale_[coordinate]);
        const float delta = (probe_ % 2 == 0 ? -step : step);
        candidate_[coordinate] = std::max(minimum_[coordinate],
            std::min(maximum_[coordinate], best_[coordinate] + delta));
        if (std::fabs(candidate_[coordinate] - best_[coordinate]) > 1e-7f) {
            gains_ = candidate_;
            trial_ = Window{};
            begin(Stage::TRIAL);
            return;
        }
        // There is no candidate to test in this direction at the gain bound.
        evaluated_probes_ |= static_cast<uint8_t>(1u << probe_);
    }
    gains_ = best_;
    begin(Stage::HOLD);
    bool differs_from_yaml = false;
    for (unsigned i = 0; i < 3; ++i) {
        const float tolerance = 1e-6f * std::max(1.0f, std::fabs(initial_[i]));
        differs_from_yaml |= std::fabs(best_[i] - initial_[i]) > tolerance;
    }
    // A discarded direction is not proof that no further improvement exists.
    ready_to_save_ = evaluated_probes_ == 0x3f && accepted_ > 0 && differs_from_yaml;
    hold_start_ms_ = now_ms;
}

void PidAutoTuner::finish(uint32_t now_ms) {
    const bool continuous_evidence = window_.active_ms >= config_.minimum_active_ms;
    const bool cycling_evidence = window_.active_ms >= config_.minimum_cycling_active_ms &&
                                  window_.complete_cycles >= 2;
    if (!continuous_evidence && !cycling_evidence) {
        // Low duty cycles are informative too: require two entire on/off runs
        // and five minutes of actual enabled time, rather than passive drift.
        discard_window();
        return;
    }
    switch (stage_) {
        case Stage::BASELINE:
            baseline_ = window_;
            next_candidate(now_ms);
            break;
        case Stage::TRIAL:
            trial_ = window_;
            gains_ = best_;
            if (improved(trial_, baseline_)) {
                begin(Stage::REFERENCE);
            } else {
                ++rejected_;
                if (comparable(trial_, baseline_))
                    evaluated_probes_ |= static_cast<uint8_t>(1u << probe_);
                ++probe_;
                // Refresh baseline between candidates instead of comparing
                // tonight's trial with yesterday's conditions.
                begin(Stage::BASELINE);
            }
            break;
        case Stage::REFERENCE: {
            reference_ = window_;
            const float drift = std::fabs(reference_.score() - baseline_.score());
            const float allowed = std::max(0.05f, baseline_.score() * 0.2f);
            if (!comparable(reference_, baseline_) || drift > allowed ||
                !improved(trial_, reference_)) {
                discard_window();  // Inconclusive, not evidence for accepting a gain.
            } else {
                gains_ = candidate_;
                begin(Stage::CONFIRM);
            }
            break;
        }
        case Stage::CONFIRM:
            if (improved(window_, baseline_) && improved(window_, reference_) &&
                comparable(window_, trial_) &&
                std::fabs(window_.score() - trial_.score()) <= std::max(0.05f, trial_.score() * 0.2f)) {
                best_ = candidate_;
                ++accepted_;
                const unsigned coordinates[] = {1, 1, 0, 0, 2, 2};
                const unsigned coordinate = coordinates[probe_];
                step_scale_[coordinate] = std::max(0.125f, step_scale_[coordinate] * 0.5f);
                probe_ = 0;  // Revisit every coordinate around the new best.
                evaluated_probes_ = 0;
            } else {
                ++rejected_;
                if (comparable(window_, baseline_) && comparable(window_, reference_) &&
                    comparable(window_, trial_) &&
                    std::fabs(window_.score() - trial_.score()) <= std::max(0.05f, trial_.score() * 0.2f))
                    evaluated_probes_ |= static_cast<uint8_t>(1u << probe_);
                ++probe_;
            }
            gains_ = best_;
            begin(Stage::BASELINE);
            break;
        default: break;
    }
}

void PidAutoTuner::observe(float temperature, float target, bool heating, bool power_on,
                          bool enabled, uint32_t now_ms, float applied_setpoint) {
    const bool valid = std::isfinite(temperature) && std::isfinite(target) &&
                       std::isfinite(applied_setpoint) && temperature >= 1 && temperature <= 40;
    if (!enabled || !valid) {
        restart();
        stage_ = enabled ? Stage::BASELINE : Stage::DISABLED;
        return;
    }
    if (stage_ == Stage::DISABLED || target != target_ || heating != heating_) {
        restart();
        target_ = target;
        heating_ = heating;
    }
    if (!have_sample_) {
        have_sample_ = true;
        last_ms_ = now_ms;
        last_temperature_ = temperature;
        last_power_ = power_on;
        return;
    }
    const uint32_t dt_ms = now_ms - last_ms_;
    const bool was_power_on = last_power_;
    const float jump = std::fabs(temperature - last_temperature_);
    last_ms_ = now_ms;
    last_temperature_ = temperature;
    last_power_ = power_on;
    if (dt_ms == 0) return;
    if (dt_ms > 30000 || jump > 1.0f) {
        discard_window();
        return;
    }
    if (stage_ == Stage::HOLD) {
        if (now_ms - hold_start_ms_ >= config_.hold_ms) restart();
        return;
    }
    const float error = heating ? target - temperature : temperature - target;
    const float over = std::max(0.0f, -error);
    const bool testing = stage_ == Stage::TRIAL || stage_ == Stage::CONFIRM;
    if (testing && (over > std::max(1.25f, baseline_.peak_over + 0.25f) || error > 3.0f)) {
        gains_ = best_;
        ++rejected_;
        ++probe_;
        begin(Stage::BASELINE);
        return;
    }
    // Settling excludes average-error scoring, not comfort peaks. A trial
    // must not hide an early overshoot by waiting until it has subsided.
    window_.peak_over = std::max(window_.peak_over, over);
    if (settling_left_ms_ > 0) {
        settling_left_ms_ -= std::min(dt_ms, settling_left_ms_);
        return;
    }
    if (window_.elapsed_ms == 0) window_.start_error = error;
    const uint32_t measured_ms = std::min(dt_ms, config_.window_ms - window_.elapsed_ms);
    window_.elapsed_ms += measured_ms;
    // Attribute an interval only when the enable state stayed on throughout.
    if (power_on && was_power_on) window_.active_ms += measured_ms;
    window_.cost += (std::max(0.0f, error) + 3.0f * over) * measured_ms / 1000.0f;
    if (power_on && !was_power_on) cycle_started_ = true;
    if (!power_on && was_power_on) {
        ++window_.shutdowns;
        if (cycle_started_) ++window_.complete_cycles;
        cycle_started_ = false;
    }
    if (power_on && have_command_) window_.movement += std::fabs(applied_setpoint - last_command_);
    if (power_on) {
        last_command_ = applied_setpoint;
        have_command_ = true;
    }
    if (window_.elapsed_ms >= config_.window_ms) finish(now_ms);
}
