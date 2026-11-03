#include "pid_autotuner.h"
#include "adaptive_pid.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>

using Stage = PidAutoTuner::Stage;

namespace {
struct Fixture {
    PidAutoTuner tuner{{{20, 2, 0}}};
    uint32_t now = 0;
    Fixture() {
        PidAutoTuner::Config config;
        config.window_ms = 10000;
        config.settling_ms = 0;
        config.minimum_active_ms = 5000;
        config.hold_ms = 10000;
        tuner.configure(config);
        sample(19.5f);
    }
    void sample(float temperature, bool power = true, bool enabled = true, float target = 20) {
        now += 1000;
        tuner.observe(temperature, target, true, power, enabled, now, 20);
    }
    void window(float temperature, bool power = true, float target = 20) {
        for (unsigned i = 0; i < 10; ++i) sample(temperature, power, true, target);
    }
};

void test_accept_requires_baseline_recheck_and_confirmation() {
    Fixture f;
    f.window(19.5f);
    assert(f.tuner.stage() == Stage::TRIAL);
    assert(f.tuner.gains()[1] < 2);
    f.window(19.6f);  // 20% lower unmet demand.
    assert(f.tuner.stage() == Stage::REFERENCE);
    assert(f.tuner.gains()[1] == 2);  // Original gains are restored for A/B/A/B.
    assert(f.tuner.accepted() == 0);
    f.window(19.5f);
    assert(f.tuner.stage() == Stage::CONFIRM);
    f.window(19.6f);
    assert(f.tuner.accepted() == 1);
    assert(f.tuner.gains()[1] < 2);
}

void test_worse_trial_rolls_back_and_searches_both_directions() {
    Fixture f;
    f.window(19.5f);
    f.window(19.4f);
    assert(f.tuner.rejected() == 1);
    assert(f.tuner.gains()[1] == 2);
    f.window(19.5f);
    assert(f.tuner.stage() == Stage::TRIAL);
    assert(f.tuner.gains()[1] > 2);
}

void test_all_coordinates_zero_derivative_and_periodic_retest() {
    Fixture f;
    std::set<std::string> tested;
    for (unsigned i = 0; i < 200 && f.tuner.stage() != Stage::HOLD; ++i) {
        if (f.tuner.stage() == Stage::TRIAL) tested.insert(f.tuner.status());
        f.sample(19.5f);  // No benefit: reject every trial.
    }
    assert(tested.count("testing_ki_down"));
    assert(tested.count("testing_ki_up"));
    assert(tested.count("testing_kp_down"));
    assert(tested.count("testing_kp_up"));
    assert(tested.count("testing_kd_up"));  // Can explore D starting from zero.
    assert(f.tuner.stage() == Stage::HOLD);
    assert(f.tuner.accepted() == 0);
    assert(f.tuner.gains()[2] == 0);
    for (unsigned i = 0; i < 11; ++i) f.sample(19.5f);
    assert(f.tuner.stage() == Stage::BASELINE);

    // With nonzero D, the downward derivative direction is also exercised.
    PidAutoTuner tuner{{{0, 0, 100}}};
    PidAutoTuner::Config config;
    config.window_ms = 1000; config.settling_ms = 0; config.minimum_active_ms = 0;
    tuner.configure(config);
    bool d_down = false;
    for (uint32_t t = 0; t < 30000; t += 1000) {
        tuner.observe(19.5f, 20, true, true, true, t, 20);
        d_down |= std::strcmp(tuner.status(), "testing_kd_down") == 0;
    }
    assert(d_down);
}

void test_bad_data_target_change_and_disable_abort_trial() {
    Fixture f;
    f.window(19.5f);
    f.sample(NAN);
    assert(f.tuner.gains()[1] == 2);
    assert(f.tuner.accepted() == 0);
    f.sample(19.5f);
    f.window(19.5f);
    assert(f.tuner.stage() == Stage::TRIAL);
    f.sample(19.5f, true, true, 21);
    assert(f.tuner.stage() == Stage::BASELINE);
    assert(f.tuner.gains()[1] == 2);
    f.window(20.5f, true, 21);
    assert(f.tuner.stage() == Stage::TRIAL);
    f.sample(20.5f, true, false, 21);
    assert(f.tuner.stage() == Stage::DISABLED);
    assert(f.tuner.gains()[1] == 2);
}

void test_emergency_rollback_and_insufficient_activity() {
    Fixture f;
    f.window(19.5f);
    f.sample(20.4f); f.sample(21.3f);
    assert(f.tuner.stage() == Stage::BASELINE);
    assert(f.tuner.rejected() == 1);
    assert(f.tuner.gains()[1] == 2);

    Fixture idle;
    idle.window(19.5f, false);
    assert(idle.tuner.stage() == Stage::BASELINE);
    assert(idle.tuner.accepted() == 0);
}

void test_weather_drift_and_failed_confirmation_are_not_accepted() {
    Fixture f;
    f.window(19.5f); f.window(19.6f); f.window(19.7f);
    assert(f.tuner.stage() == Stage::BASELINE);
    assert(f.tuner.accepted() == 0);
    assert(f.tuner.gains()[1] == 2);

    Fixture g;
    g.window(19.5f); g.window(19.6f); g.window(19.5f); g.window(19.4f);
    assert(g.tuner.stage() == Stage::BASELINE);
    assert(g.tuner.accepted() == 0);
    assert(g.tuner.gains()[1] == 2);
}

void test_gap_rollover_cooling_and_gain_bounds() {
    Fixture f;
    f.window(19.5f);
    f.now += 60000; f.sample(19.5f);
    assert(f.tuner.stage() == Stage::BASELINE);
    assert(f.tuner.gains()[1] == 2);

    PidAutoTuner tuner{{{20, 2, 0}}};
    PidAutoTuner::Config config;
    config.window_ms = 4000; config.settling_ms = 0; config.minimum_active_ms = 0;
    tuner.configure(config);
    tuner.set_bounds({{20, 2, 0}}, {{20, 2, 0}});
    for (uint32_t i = 0; i <= 4; ++i)
        tuner.observe(20.5f, 20, false, true, true, UINT32_MAX - 1999 + i * 1000, 20);
    assert(tuner.stage() == Stage::HOLD);
    // Cooling above target is unmet demand, not overshoot: score is 0.5.
    assert(std::fabs(tuner.baseline_score() - 0.5f) < 0.001f);
    assert(tuner.gains()[0] == 20);
}

void test_settling_is_not_scored() {
    PidAutoTuner tuner{{{20, 2, 0}}};
    PidAutoTuner::Config config;
    config.window_ms = 4000; config.settling_ms = 2000; config.minimum_active_ms = 0;
    tuner.configure(config);
    for (uint32_t t = 0; t <= 2000; t += 1000)
        tuner.observe(19, 20, true, true, true, t, 20);
    for (uint32_t t = 3000; t <= 6000; t += 1000)
        tuner.observe(19.5f, 20, true, true, true, t, 20);
    assert(std::fabs(tuner.baseline_score() - 0.5f) < 0.001f);
}

void test_small_and_zero_gains_receive_observable_probes() {
    PidAutoTuner tuner{{{1.25f, 0.015f, 0}}};
    PidAutoTuner::Config config;
    config.window_ms = 1000; config.settling_ms = 0; config.minimum_active_ms = 0;
    tuner.configure(config);
    bool ki = false, kp = false, kd = false;
    for (uint32_t t = 0; t < 20000; t += 1000) {
        tuner.observe(19.5f, 20, true, true, true, t, 20);
        const auto gains = tuner.gains();
        if (std::strcmp(tuner.status(), "testing_ki_up") == 0) {
            const float correction = 3.75f * std::tanh((gains[1] - 0.015f) / 100);
            assert(correction > 0.24f);
            ki = true;
        }
        if (std::strcmp(tuner.status(), "testing_kp_up") == 0) {
            assert(gains[0] > 10);
            kp = true;
        }
        if (std::strcmp(tuner.status(), "testing_kd_up") == 0) {
            assert(gains[2] > 1000 && gains[2] <= 7200);
            kd = true;
        }
    }
    assert(ki && kp && kd);
}

void test_settling_peak_is_retained_and_uncomparable_trial_not_accepted() {
    PidAutoTuner tuner{{{20, 2, 0}}};
    PidAutoTuner::Config config;
    config.window_ms = 2000; config.settling_ms = 2000; config.minimum_active_ms = 0;
    tuner.configure(config);
    tuner.observe(20.5f, 20, true, true, true, 0, 20);
    tuner.observe(20.5f, 20, true, true, true, 1000, 20);
    tuner.observe(20.5f, 20, true, true, true, 2000, 20);
    tuner.observe(20, 20, true, true, true, 3000, 20);
    tuner.observe(20, 20, true, true, true, 4000, 20);
    assert(tuner.baseline_score() >= 1);  // Early 0.5°C peak is not hidden.

    Fixture f;
    f.window(19.0f);
    f.window(19.8f);  // Better score but a different starting condition.
    assert(f.tuner.accepted() == 0);
    assert(f.tuner.stage() == Stage::BASELINE);
}

void test_closed_loop_simulation_preserves_limits_and_finite_state() {
    AdaptivePID pid(20, 2, 0, 18, 25.5f);
    pid.set_target(20.278f, true);
    pid.set_output_limits(18, 20.528f);
    float room = 20.0f, heat = 0;
    bool on = true;
    // Three simulated days, including hysteresis, thermal inertia, changing
    // heat loss, and daily target changes. This checks behavior, not field gains.
    for (uint32_t t = 0; t < 3 * 86400000u; t += 2000) {
        const bool night = (t / 3600000u) % 24 >= 12;
        const float target = night ? 20.278f : 19.444f;
        pid.set_target(target, true);
        pid.set_output_limits(18, target + 0.25f);
        if (on && room > target + 0.75f) on = false;
        if (!on && room < target - 0.25f) on = true;
        const float command = pid.update(room, t, on);
        assert(std::isfinite(command));
        assert(command >= 18 && command <= target + 0.2501f);
        const float demand = on ? std::max(0.0f, std::min(1.0f, 0.3f + (command - room))) : 0;
        heat += (demand - heat) * 2 / 120;
        room += 2 * ((18 - room) / 7200 + heat * 0.002f);
        assert(std::isfinite(room));
        assert(std::isfinite(pid.kp()) && std::isfinite(pid.ki()) && std::isfinite(pid.kd()));
    }
    std::printf("Three-day simulation: accepted=%u rejected=%u gains=(%.3f, %.3f, %.3f)\n",
        static_cast<unsigned>(pid.tuner().accepted()), static_cast<unsigned>(pid.tuner().rejected()),
        pid.kp(), pid.ki(), pid.kd());
}

void test_low_duty_cycles_complete_trials() {
    PidAutoTuner tuner{{{1.25f, 0.015f, 0}}};
    tuner.observe(19.5f, 20, true, false, true, 0, 20);
    // Reproduce the field pattern: 70-minute phases (10 settling + 60 scoring)
    // with two full cycles totaling 13 minutes of enabled heating per phase.
    const uint32_t phase_ms = 70 * 60000;
    for (uint32_t t = 2000; t <= 3 * phase_ms; t += 2000) {
        const uint32_t minute = (t % phase_ms) / 60000;
        const bool power = (minute >= 20 && minute < 27) || (minute >= 50 && minute < 56);
        tuner.observe(19.5f, 20, true, power, true, t, 20);
        if (t == phase_ms) {
            assert(tuner.stage() == Stage::TRIAL);
            assert(tuner.gains()[1] == 0);
        }
        if (t == 2 * phase_ms) {
            assert(tuner.stage() == Stage::BASELINE);
            assert(tuner.rejected() == 1);  // Scored, rather than discarded.
        }
    }
    assert(std::strcmp(tuner.status(), "testing_ki_up") == 0);
    assert(tuner.discarded() == 0);
}

void test_inconclusive_trial_does_not_repeat_first_probe() {
    Fixture f;
    f.window(19.5f);  // Baseline qualifies, Ki-down starts.
    f.window(19.5f, false);  // Candidate has insufficient activity.
    assert(f.tuner.discarded() == 1);
    assert(f.tuner.accepted() == 0 && f.tuner.rejected() == 0);
    assert(f.tuner.gains()[1] == 2);
    f.sample(19.5f);
    f.window(19.5f);
    assert(std::strcmp(f.tuner.status(), "testing_ki_up") == 0);
}

void test_idle_or_incomplete_cycles_do_not_qualify() {
    PidAutoTuner tuner{{{20, 2, 0}}};
    PidAutoTuner::Config config;
    config.window_ms = 10000;
    config.settling_ms = 5000;
    config.minimum_active_ms = 9000;
    config.minimum_cycling_active_ms = 1000;
    tuner.configure(config);
    // First on/off cycle starts in settling, so it cannot count as a complete
    // scored cycle. The single full scored cycle is not sufficient evidence.
    for (uint32_t t = 0; t <= 15000; t += 1000) {
        const bool power = (t >= 1000 && t < 6000) || (t >= 8000 && t < 10000);
        tuner.observe(19.5f, 20, true, power, true, t, 20);
    }
    assert(tuner.stage() == Stage::BASELINE);
    assert(tuner.discarded() == 1);

    // Two brief pulses also must not bypass the minimum enabled-time guard.
    PidAutoTuner pulses{{{20, 2, 0}}};
    config.settling_ms = 0;
    config.minimum_cycling_active_ms = 5000;
    pulses.configure(config);
    for (uint32_t t = 0; t <= 10000; t += 1000) {
        const bool power = (t >= 1000 && t < 3000) || (t >= 5000 && t < 7000);
        pulses.observe(19.5f, 20, true, power, true, t, 20);
    }
    assert(pulses.stage() == Stage::BASELINE);
    assert(pulses.discarded() == 1);
}

void accept_lower_ki(Fixture& f) {
    f.window(19.5f);
    f.window(19.6f);
    f.window(19.5f);
    f.window(19.6f);
    assert(f.tuner.accepted() == 1);
}

void finish_search_without_improvement(Fixture& f) {
    for (unsigned i = 0; i < 300 && f.tuner.stage() != Stage::HOLD; ++i)
        f.sample(19.5f);
    assert(f.tuner.stage() == Stage::HOLD);
}

void test_accepted_gains_do_not_expose_trials() {
    Fixture f;
    f.window(19.5f);
    assert(f.tuner.gains()[1] == 0);  // Active trial.
    assert(f.tuner.accepted_gains()[1] == 2);  // YAML/accepted value stays visible.
    f.window(19.6f); f.window(19.5f);
    assert(f.tuner.stage() == Stage::CONFIRM);
    assert(f.tuner.accepted_gains()[1] == 2);
    f.window(19.6f);
    assert(f.tuner.accepted_gains()[1] == 0);
    assert(std::strcmp(f.tuner.status(), "ready_to_save") != 0);
    f.window(19.5f);
    assert(f.tuner.gains()[1] > 0);
    assert(f.tuner.accepted_gains()[1] == 0);
}

void test_ready_to_save_requires_changed_gains_and_complete_search() {
    Fixture unchanged;
    finish_search_without_improvement(unchanged);
    assert(std::strcmp(unchanged.tuner.status(), "holding_best") == 0);

    Fixture improved;
    accept_lower_ki(improved);
    finish_search_without_improvement(improved);
    assert(std::strcmp(improved.tuner.status(), "ready_to_save") == 0);
    const auto saved = improved.tuner.accepted_gains();
    assert(saved[0] == 20 && saved[1] == 0 && saved[2] == 0);
    improved.sample(19.5f);
    assert(std::strcmp(improved.tuner.status(), "ready_to_save") == 0);
    // A target change invalidates search completeness, but keeps accepted gains.
    improved.sample(19.5f, true, true, 21);
    assert(improved.tuner.stage() == Stage::BASELINE);
    assert(improved.tuner.accepted_gains() == saved);
    assert(std::strcmp(improved.tuner.status(), "ready_to_save") != 0);

    Fixture saved_yaml;
    accept_lower_ki(saved_yaml);
    saved_yaml.tuner.set_bounds({{20, 2, 0}}, {{20, 2, 0}});
    // Returning to the YAML values must not prompt the user to save them again.
    saved_yaml.sample(19.5f);
    saved_yaml.window(19.5f);
    assert(saved_yaml.tuner.stage() == Stage::HOLD);
    assert(std::strcmp(saved_yaml.tuner.status(), "holding_best") == 0);
}

void test_inconclusive_search_does_not_claim_ready_to_save() {
    Fixture f;
    accept_lower_ki(f);
    f.window(19.5f);  // First remaining candidate is Ki-up.
    assert(f.tuner.stage() == Stage::TRIAL);
    f.window(19.5f, false);  // Insufficient activity: direction was not evaluated.
    f.sample(19.5f);
    finish_search_without_improvement(f);
    assert(f.tuner.discarded() > 0);
    assert(std::strcmp(f.tuner.status(), "holding_best") == 0);
}
}  // namespace

int main() {
    test_accept_requires_baseline_recheck_and_confirmation();
    test_worse_trial_rolls_back_and_searches_both_directions();
    test_all_coordinates_zero_derivative_and_periodic_retest();
    test_bad_data_target_change_and_disable_abort_trial();
    test_emergency_rollback_and_insufficient_activity();
    test_weather_drift_and_failed_confirmation_are_not_accepted();
    test_gap_rollover_cooling_and_gain_bounds();
    test_settling_is_not_scored();
    test_small_and_zero_gains_receive_observable_probes();
    test_settling_peak_is_retained_and_uncomparable_trial_not_accepted();
    test_closed_loop_simulation_preserves_limits_and_finite_state();
    test_low_duty_cycles_complete_trials();
    test_inconclusive_trial_does_not_repeat_first_probe();
    test_idle_or_incomplete_cycles_do_not_qualify();
    test_accepted_gains_do_not_expose_trials();
    test_ready_to_save_requires_changed_gains_and_complete_search();
    test_inconclusive_search_does_not_claim_ready_to_save();
    std::puts("Automatic PID tuner tests passed");
}
