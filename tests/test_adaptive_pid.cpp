#include "adaptive_pid.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>

namespace {
void near(float actual, float expected, float tolerance = 0.0001f) {
    assert(std::fabs(actual - expected) < tolerance);
}

void test_contributions_match_output() {
    AdaptivePID pid(2.0f, 0.5f, 3.0f, 18.0f, 25.5f);
    pid.enable_adaptation(false);
    pid.set_target(20.0f, true);
    pid.update(19.0f, 1000, true);
    const float output = pid.update(19.2f, 1200, true);
    near(pid.p_term(), 1.6f);
    near(pid.i_term(), 0.08f);
    near(pid.d_term(), -3.0f);
    near(output, 20.0f + 3.75f * std::tanh(
        (pid.p_term() + pid.i_term() + pid.d_term()) / 100.0f));
}

void test_restart_excludes_disabled_time_and_error_change() {
    AdaptivePID pid(2.0f, 0.5f, 3.0f);
    pid.enable_adaptation(false);
    pid.set_target(20.0f, true);
    pid.update(19.0f, 1000, true);
    pid.update(19.0f, 1500, true);
    near(pid.i_term(), 0.25f);

    // One off notification must also handle a long gap before the next call.
    near(pid.update(19.0f, 2000, false), 20.0f);
    near(pid.p_term(), 0.0f);
    near(pid.i_term(), 0.0f);
    near(pid.d_term(), 0.0f);
    pid.update(19.8f, 3602000, true);
    near(pid.p_term(), 0.4f);
    near(pid.i_term(), 0.0f);
    near(pid.d_term(), 0.0f);
    pid.update(19.8f, 3602500, true);
    near(pid.i_term(), 0.05f);
    near(pid.d_term(), 0.0f);
}

void test_disabled_updates_preserve_gains_and_restart_adaptation_clock() {
    AdaptivePID pid(2.0f, 0.5f, 3.0f);
    pid.set_target(20.0f, true);
    pid.update(19.5f, 1000, true);
    pid.update(19.5f, 16000, true);
    assert(pid.kp() > 2.0f);
    const float kp = pid.kp(), ki = pid.ki(), kd = pid.kd();
    pid.update(19.0f, 17000, false);
    pid.update(18.0f, 3600000, false);
    near(pid.kp(), kp);
    near(pid.ki(), ki);
    near(pid.kd(), kd);

    pid.update(19.5f, 7200000, true);
    near(pid.kp(), kp);  // No adaptation triggered by the disabled interval.
    near(pid.i_term(), 0.0f);
    near(pid.d_term(), 0.0f);
    pid.update(19.5f, 7214999, true);
    near(pid.kp(), kp);
    pid.update(19.5f, 7215000, true);
    assert(pid.kp() > kp);
}

void test_zero_timestamp_and_clock_rollover() {
    AdaptivePID pid(1.0f, 0.5f, 0.0f);
    pid.enable_adaptation(false);
    pid.set_target(20.0f, true);
    pid.update(19.5f, 0, true);
    pid.update(19.5f, 500, true);
    near(pid.i_term(), 0.125f);

    pid.update(19.5f, UINT32_MAX - 999, false);
    pid.update(19.5f, UINT32_MAX - 499, true);
    near(pid.i_term(), 0.0f);
    pid.update(19.5f, 500, true);
    near(pid.i_term(), 0.25f);
}
}  // namespace

int main() {
    test_contributions_match_output();
    test_restart_excludes_disabled_time_and_error_change();
    test_disabled_updates_preserve_gains_and_restart_adaptation_clock();
    test_zero_timestamp_and_clock_rollover();
    std::puts("Adaptive PID regression tests passed");
}
