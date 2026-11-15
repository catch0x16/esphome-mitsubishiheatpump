// Native test of the online response-rate estimator and the tuning rule:
//   c++ -std=c++17 -Wall -Wextra -Icomponents/workflow_climate -o /tmp/test_response_rate_estimator \
//     tests/test_response_rate_estimator.cpp && /tmp/test_response_rate_estimator
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "response_rate_estimator.h"

using workflow::pid::Gains;
using workflow::pid::ResponseRateEstimator;
using workflow::pid::TuningConfig;

static int failures = 0;

static void check(bool ok, const char *what) {
  std::printf("%s  %s\n", ok ? "ok  " : "FAIL", what);
  if (!ok) failures++;
}

static bool near(float a, float b, float tolerance) { return std::fabs(a - b) <= tolerance; }

static const float PER_HOUR = 1.0f / 3600.0f;

// Run the device for `minutes` at `rate_per_hour`, sampling every 2 s.
static uint32_t run(ResponseRateEstimator &estimator, uint32_t now, float &temperature, float rate_per_hour,
                    int minutes, bool operating, bool heating) {
  for (int i = 0; i < minutes * 30; i++) {
    now += 2000;
    temperature += rate_per_hour * PER_HOUR * 2.0f;
    estimator.update(temperature, operating, heating, now);
  }
  return now;
}

int main() {
  TuningConfig config;

  {
    // 3 C/h with the defaults (15 minute time scale, 0.75 gap) gives kp 1.0.
    const Gains gains = workflow::pid::gains_from_rate(3.0f * PER_HOUR, config);
    check(near(gains.kp, 1.0f, 0.001f), "rule: 3 C/h gives kp 1.0");
    check(near(gains.ki, 1.0f / 3600.0f, 0.00001f), "rule: ki = kp / (4 * 15 min)");
    check(near(workflow::pid::rate_from_kp(gains.kp, config), 3.0f * PER_HOUR, 0.00001f), "rule: kp maps back to the rate");
    check(near(workflow::pid::gains_from_rate(100.0f * PER_HOUR, config).kp, config.kp_min, 0.001f), "rule: kp lower bound");
    check(near(workflow::pid::gains_from_rate(0.01f * PER_HOUR, config).kp, config.kp_max, 0.001f), "rule: kp upper bound");
  }
  {
    // Repeated 40 minute heating runs at 3 C/h converge on 3 C/h from the default.
    ResponseRateEstimator estimator;
    float temperature = 20.0f;
    uint32_t now = 1000;
    for (int cycle = 0; cycle < 25; cycle++) {
      now = run(estimator, now, temperature, 3.0f, 40, true, true);
      now = run(estimator, now, temperature, -0.3f, 60, false, true);
    }
    check(near(estimator.rate(true) * 3600.0f, 3.0f, 0.05f), "estimator: converges on the heating rate");
    check(near(estimator.rate(false) * 3600.0f, 2.0f, 0.001f), "estimator: cooling rate untouched by heating runs");
  }
  {
    // A single measurement moves the estimate only part of the way.
    ResponseRateEstimator estimator;
    estimator.set_rate(true, 2.0f * PER_HOUR);
    float temperature = 20.0f;
    run(estimator, 1000, temperature, 4.0f, 31, true, true);
    check(estimator.measurements() == 1, "estimator: one measurement per 30 minutes of operation");
    check(near(estimator.rate(true) * 3600.0f, 2.4f, 0.05f), "estimator: one measurement moves 20% of the way");
  }
  {
    // Short bursts add up: only the time spent operating is measured.
    ResponseRateEstimator estimator;
    float temperature = 20.0f;
    uint32_t now = 1000;
    for (int cycle = 0; cycle < 200; cycle++) {
      now = run(estimator, now, temperature, 4.0f, 4, true, true);
      now = run(estimator, now, temperature, -1.0f, 12, false, true);
    }
    check(estimator.measurements() >= 20, "estimator: 4 minute bursts accumulate into measurements");
    check(near(estimator.rate(true) * 3600.0f, 4.0f, 0.1f), "estimator: bursts converge on the operating rate");
  }
  {
    // Time with no samples (or no valid temperature) is not counted.
    ResponseRateEstimator estimator;
    float temperature = 20.0f;
    uint32_t now = run(estimator, 1000, temperature, 3.0f, 10, true, true);
    now += 3600000;
    temperature += 5.0f;
    run(estimator, now, temperature, 3.0f, 25, true, true);
    check(estimator.measurements() == 1 && near(estimator.rate(true) * 3600.0f, 2.2f, 0.05f),
          "estimator: a gap between samples is skipped");
  }
  {
    // Cooling learns from falling temperature, separately from heating.
    ResponseRateEstimator estimator;
    float temperature = 26.0f;
    uint32_t now = 1000;
    for (int cycle = 0; cycle < 25; cycle++) {
      now = run(estimator, now, temperature, -1.5f, 45, true, false);
      now = run(estimator, now, temperature, 0.5f, 30, false, false);
    }
    check(near(estimator.rate(false) * 3600.0f, 1.5f, 0.05f), "estimator: converges on the cooling rate");
    check(near(estimator.rate(true) * 3600.0f, 2.0f, 0.001f), "estimator: heating rate untouched by cooling runs");
  }
  {
    // A room that loses ground while the device runs is held at the minimum rate.
    ResponseRateEstimator estimator;
    float temperature = 20.0f;
    uint32_t now = 1000;
    for (int cycle = 0; cycle < 40; cycle++) {
      now = run(estimator, now, temperature, -1.0f, 40, true, true);
      now = run(estimator, now, temperature, 0.0f, 5, false, true);
    }
    check(near(estimator.rate(true) * 3600.0f, 0.2f, 0.01f), "estimator: bounded below");
  }

  if (failures != 0) {
    std::printf("%d failure(s)\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("all passed\n");
  return EXIT_SUCCESS;
}
