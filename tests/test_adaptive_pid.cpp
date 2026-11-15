// Native test of the PID controller. No ESPHome needed:
//   c++ -std=c++17 -Wall -Wextra -Icomponents/workflow_climate -o /tmp/test_adaptive_pid \
//     tests/test_adaptive_pid.cpp components/workflow_climate/adaptive_pid.cpp && /tmp/test_adaptive_pid
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "adaptive_pid.h"

static int failures = 0;

static void check(bool ok, const char *what) {
  std::printf("%s  %s\n", ok ? "ok  " : "FAIL", what);
  if (!ok) failures++;
}

static bool near(float a, float b, float tolerance = 0.001f) { return std::fabs(a - b) <= tolerance; }

static AdaptivePID make(float kp, float ki, float kd) {
  AdaptivePID pid(kp, ki, kd);
  pid.set_output_limits(-2.0f, 2.0f);
  pid.set_target(21.0f, true);
  return pid;
}

int main() {
  {
    // Gains are in degrees of setpoint per degree of error.
    AdaptivePID pid = make(1.5f, 0.0f, 0.0f);
    check(near(pid.update(20.5f, 1000, true), 21.75f), "P: 0.5 below target with kp 1.5 raises the setpoint 0.75");
    check(near(pid.update(21.5f, 3000, true), 20.25f), "P: 0.5 above target lowers the setpoint 0.75");
  }
  {
    // Cooling uses the same sign: a warm room lowers the setpoint.
    AdaptivePID pid = make(1.0f, 0.0f, 0.0f);
    pid.set_target(24.0f, false);
    check(near(pid.update(25.0f, 1000, true), 23.0f), "P: cooling, 1 above target lowers the setpoint 1");
  }
  {
    AdaptivePID pid = make(0.0f, 0.01f, 0.0f);
    float out = 0.0f;
    for (uint32_t t = 1000; t <= 61000; t += 2000) out = pid.update(20.5f, t, true);
    // 0.5 error for 60 s at ki 0.01 -> 0.3
    check(near(out, 21.3f, 0.01f), "I: accumulates error over time");
  }
  {
    // Pinned at the limit, the integral stops growing, so the output leaves the
    // limit as soon as the error reverses.
    AdaptivePID pid = make(1.0f, 0.01f, 0.0f);
    float out = 0.0f;
    uint32_t t = 1000;
    for (int i = 0; i < 1800; i++, t += 2000) out = pid.update(18.0f, t, true);
    check(near(out, 23.0f), "limit: output is held at target + max offset");
    check(pid.integral() * pid.ki() <= 2.0f + 0.001f, "anti-windup: integral term never exceeds the output range");
    out = pid.update(21.5f, t, true);
    check(out < 23.0f, "anti-windup: output leaves the limit when the error reverses");
  }
  {
    // Time spent off is not integrated when the device comes back.
    AdaptivePID pid = make(0.0f, 0.01f, 0.0f);
    pid.update(20.5f, 1000, true);
    for (uint32_t t = 3000; t <= 603000; t += 2000) pid.update(20.5f, t, false);
    const float out = pid.update(20.5f, 605000, true);
    check(near(out, 21.01f, 0.005f), "resume: only the last interval is integrated after 10 minutes off");
  }
  {
    // A gap with no updates at all (e.g. another mode) is not integrated either.
    AdaptivePID pid = make(0.0f, 0.01f, 0.0f);
    pid.update(20.5f, 1000, true);
    const float out = pid.update(20.5f, 3601000, true);
    check(near(out, 21.0f), "gap: an hour without updates adds nothing");
  }
  {
    // Derivative responds to a changing error but not to a target change.
    AdaptivePID pid = make(0.0f, 0.0f, 10.0f);
    pid.update(20.0f, 1000, true);
    check(near(pid.update(20.1f, 3000, true), 20.5f), "D: room rising 0.05/s with kd 10 lowers the setpoint 0.5");
    pid.set_target(22.0f, true);
    check(near(pid.update(20.1f, 5000, true), 22.0f), "D: no kick on a target change");
  }
  {
    AdaptivePID pid = make(1.0f, 0.01f, 0.0f);
    for (uint32_t t = 1000; t <= 61000; t += 2000) pid.update(20.5f, t, true);
    check(near(pid.update(20.5f, 63000, false), 21.0f), "off: returns the target");
    check(near(pid.integral(), 0.0f), "off: integral is reset");
  }
  {
    AdaptivePID pid = make(100.0f, 0.0f, 0.0f);
    pid.set_output_limits(-3.0f, 0.5f);
    check(near(pid.update(15.0f, 1000, true), 21.5f), "limits: asymmetric upper limit");
    check(near(pid.update(27.0f, 3000, true), 18.0f), "limits: asymmetric lower limit");
  }

  {
    // Changing the gains while running does not step the output.
    AdaptivePID pid = make(1.0f, 0.001f, 0.0f);
    float before = 0.0f;
    for (uint32_t t = 1000; t <= 601000; t += 2000) before = pid.update(20.8f, t, true);
    pid.set_gains(1.0f, 0.002f);
    const float after = pid.update(20.8f, 603000, true);
    check(std::fabs(after - before) < 0.01f, "set_gains: integral term carries over without a step");
    pid.set_gains(2.0f, 0.002f);
    check(near(pid.kp(), 2.0f) && near(pid.ki(), 0.002f, 0.00001f), "set_gains: gains are replaced");
  }

  if (failures != 0) {
    std::printf("%d failure(s)\n", failures);
    return EXIT_FAILURE;
  }
  std::printf("all passed\n");
  return EXIT_SUCCESS;
}
