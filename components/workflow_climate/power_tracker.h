#pragma once

#include <cstdint>

// Tracks the power state the wrapper has commanded on the inner climate versus
// what the inner climate reports. No ESPHome dependencies, so it can be unit
// tested natively (tests/test_power_tracker.cpp).
//
// A generic climate gives no command acknowledgement: it may publish the
// requested state before anything reaches the device, and a stale device read
// can briefly publish the old state afterwards. So a report is only trusted
// once a settle window has elapsed since the last power command, and an
// unrequested change only counts as external if the commanded state had been
// seen on the device first.
namespace workflow {

enum class PowerEvent : uint8_t {
    NONE,
    CONFIRMED,      // device reached the commanded state
    EXTERNAL_OFF,   // device was confirmed on, then turned off by someone else
    EXTERNAL_ON,    // device was confirmed off, then turned on while the outer climate is off
    UNEXPECTED_ON,  // device is on although the workflow commanded off and the outer climate is on
    LOST_COMMAND,   // device never reached the commanded state
};

class PowerTracker {
 public:
    void set_settle_ms(uint32_t settle_ms) { this->settle_ms_ = settle_ms; }

    // Adopt an observed state as both commanded and confirmed.
    void adopt(bool on) {
        this->commanded_on_ = on;
        this->confirmed_ = true;
        this->settling_ = false;
    }

    // The wrapper sent a power command to the inner climate.
    void command(bool on, uint32_t now) {
        this->commanded_on_ = on;
        this->confirmed_ = false;
        this->settling_ = true;
        this->last_command_ms_ = now;
    }

    bool commanded_on() const { return this->commanded_on_; }
    bool confirmed() const { return this->confirmed_; }
    uint32_t last_command_ms() const { return this->last_command_ms_; }

    bool settling(uint32_t now) const {
        return this->settling_ && static_cast<uint32_t>(now - this->last_command_ms_) < this->settle_ms_;
    }

    // inner_on: the inner climate currently reports a mode other than OFF.
    // outer_on: the user-facing climate is in a mode other than OFF.
    PowerEvent observe(bool inner_on, bool outer_on, uint32_t now) {
        if (this->settling(now)) {
            return PowerEvent::NONE;
        }
        this->settling_ = false;

        if (inner_on == this->commanded_on_) {
            if (this->confirmed_) {
                return PowerEvent::NONE;
            }
            this->confirmed_ = true;
            return PowerEvent::CONFIRMED;
        }

        if (this->commanded_on_) {
            // Commanded on, device reports off.
            if (!this->confirmed_) {
                return PowerEvent::LOST_COMMAND;
            }
            this->commanded_on_ = false;
            return PowerEvent::EXTERNAL_OFF;
        }

        // Commanded off, device reports on.
        if (outer_on) {
            // The workflow switched the device off and it is running anyway.
            // Resync and let the next workflow run decide what to do.
            this->commanded_on_ = true;
            this->confirmed_ = true;
            return PowerEvent::UNEXPECTED_ON;
        }
        if (!this->confirmed_) {
            return PowerEvent::LOST_COMMAND;
        }
        this->commanded_on_ = true;
        return PowerEvent::EXTERNAL_ON;
    }

 private:
    uint32_t settle_ms_{9000};
    uint32_t last_command_ms_{0};
    bool commanded_on_{false};
    bool confirmed_{false};
    bool settling_{false};
};

}  // namespace workflow
