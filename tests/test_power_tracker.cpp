// Native test for the power tracking rules. No ESPHome needed:
//   c++ -std=c++17 -Wall -Wextra -o /tmp/test_power_tracker tests/test_power_tracker.cpp && /tmp/test_power_tracker
#include <cstdio>
#include <cstdlib>

#include "../components/workflow_climate/power_tracker.h"

using workflow::PowerEvent;
using workflow::PowerTracker;

static int failures = 0;

#define CHECK(condition) \
    do { \
        if (!(condition)) { \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
            failures++; \
        } \
    } while (0)

static const uint32_t SETTLE = 9000;

static PowerTracker make(bool on) {
    PowerTracker tracker;
    tracker.set_settle_ms(SETTLE);
    tracker.adopt(on);
    return tracker;
}

// An inner climate that publishes the requested state at once and then briefly
// reverts on a stale device read must not disturb anything.
static void optimistic_echo_then_stale_revert() {
    PowerTracker t = make(true);
    t.command(false, 1000);
    CHECK(t.observe(false, true, 1001) == PowerEvent::NONE);  // optimistic echo
    CHECK(!t.confirmed());                                    // an echo is not an acknowledgement
    CHECK(t.observe(true, true, 3000) == PowerEvent::NONE);   // stale revert inside the window
    CHECK(!t.commanded_on());
    CHECK(t.observe(false, true, 1000 + SETTLE) == PowerEvent::CONFIRMED);
    CHECK(t.confirmed());
    CHECK(t.observe(false, true, 60000) == PowerEvent::NONE);
}

static void lost_on_is_retried_not_treated_as_user_off() {
    PowerTracker t = make(false);
    t.command(true, 1000);
    CHECK(t.observe(false, true, 1000 + SETTLE) == PowerEvent::LOST_COMMAND);
    CHECK(t.commanded_on());
    CHECK(t.observe(false, true, 30000) == PowerEvent::LOST_COMMAND);
    t.command(true, 61000);  // the retry
    CHECK(t.observe(false, true, 62000) == PowerEvent::NONE);
    CHECK(t.observe(true, true, 61000 + SETTLE) == PowerEvent::CONFIRMED);
}

static void confirmed_on_then_off_is_external_off() {
    PowerTracker t = make(false);
    t.command(true, 1000);
    CHECK(t.observe(true, true, 1000 + SETTLE) == PowerEvent::CONFIRMED);
    CHECK(t.observe(false, true, 50000) == PowerEvent::EXTERNAL_OFF);
    CHECK(!t.commanded_on());
    CHECK(t.confirmed());
    // The outer climate is now off as well: nothing further to report.
    CHECK(t.observe(false, false, 51000) == PowerEvent::NONE);
}

static void workflow_off_then_device_on_resyncs() {
    PowerTracker t = make(true);
    t.command(false, 1000);  // hysteresis switched the device off
    CHECK(t.observe(false, true, 1000 + SETTLE) == PowerEvent::CONFIRMED);
    CHECK(t.observe(true, true, 40000) == PowerEvent::UNEXPECTED_ON);
    CHECK(t.commanded_on());
    CHECK(t.confirmed());
    CHECK(t.observe(true, true, 41000) == PowerEvent::NONE);
}

// Same handling when the workflow's OFF never reached the device.
static void workflow_off_never_applied_resyncs() {
    PowerTracker t = make(true);
    t.command(false, 1000);
    CHECK(t.observe(true, true, 1000 + SETTLE) == PowerEvent::UNEXPECTED_ON);
    CHECK(t.commanded_on());
}

static void outer_off_then_device_on_is_external_on() {
    PowerTracker t = make(true);
    t.command(false, 1000);  // the user switched the outer climate off
    CHECK(t.observe(false, false, 1000 + SETTLE) == PowerEvent::CONFIRMED);
    CHECK(t.observe(true, false, 90000) == PowerEvent::EXTERNAL_ON);
    CHECK(t.commanded_on());
    CHECK(t.confirmed());
}

static void lost_user_off_is_retried() {
    PowerTracker t = make(true);
    t.command(false, 1000);
    CHECK(t.observe(true, false, 1000 + SETTLE) == PowerEvent::LOST_COMMAND);
    CHECK(!t.commanded_on());
}

static void settle_window_survives_millis_rollover() {
    PowerTracker t = make(true);
    const uint32_t start = 0xFFFFFF00u;
    t.command(false, start);
    CHECK(t.settling(start + 500));        // wrapped past zero
    CHECK(t.observe(true, true, start + 500) == PowerEvent::NONE);
    CHECK(!t.settling(start + SETTLE));
    CHECK(t.observe(false, true, start + SETTLE) == PowerEvent::CONFIRMED);
}

int main() {
    optimistic_echo_then_stale_revert();
    lost_on_is_retried_not_treated_as_user_off();
    confirmed_on_then_off_is_external_off();
    workflow_off_then_device_on_resyncs();
    workflow_off_never_applied_resyncs();
    outer_off_then_device_on_is_external_on();
    lost_user_off_is_retried();
    settle_window_survives_millis_rollover();
    if (failures == 0) {
        std::printf("power tracker: all tests passed\n");
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
