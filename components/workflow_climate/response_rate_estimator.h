#pragma once

#include <cmath>
#include <cstdint>

// Online estimate of how fast the room responds while the device is actively
// heating or cooling, and the PI gains that follow from it. No ESPHome
// dependency, so it can be tested and replayed natively.
//
// Model: while the device is operating, the room moves toward the target at a
// roughly constant rate (degrees per second), measured as the temperature
// change accumulated over operating time. That makes the room an
// integrating process as seen by the PID, whose input is a setpoint offset:
// the device reaches full output once its setpoint is `full_output_gap`
// degrees past the room temperature, so
//
//   process gain k' = rate / full_output_gap        [1/s]
//
// and the SIMC tuning rule for an integrating process gives
//
//   kp = 1 / (k' * tc)        ki = kp / (4 * tc)        tc = response_time + dead_time
namespace workflow {
namespace pid {

struct TuningConfig {
  bool adaptive{true};
  float response_time_s{600.0f};
  float dead_time_s{300.0f};
  float full_output_gap{0.75f};
  float kp_min{0.25f};
  float kp_max{4.0f};
};

struct Gains {
  float kp;
  float ki;
};

inline Gains gains_from_rate(float rate, const TuningConfig &config) {
  const float tc = config.response_time_s + config.dead_time_s;
  float kp = config.full_output_gap / (rate * tc);
  if (!(kp >= config.kp_min)) kp = config.kp_min;  // also catches NaN
  if (kp > config.kp_max) kp = config.kp_max;
  return Gains{kp, kp / (4.0f * tc)};
}

inline float rate_from_kp(float kp, const TuningConfig &config) {
  return config.full_output_gap / (kp * (config.response_time_s + config.dead_time_s));
}

class ResponseRateEstimator {
 public:
  // Operating time is accumulated across runs, however short, and one
  // measurement is taken per MEASURE_MS of it.
  static constexpr uint32_t MEASURE_MS = 30 * 60 * 1000;
  // Two samples further apart than this are not joined into one interval.
  static constexpr uint32_t MAX_SAMPLE_GAP_MS = 5 * 60 * 1000;
  // Each measurement moves the estimate this fraction of the way.
  static constexpr float SMOOTHING = 0.2f;
  // 0.2 and 10 degrees per hour.
  static constexpr float MIN_RATE = 0.2f / 3600.0f;
  static constexpr float MAX_RATE = 10.0f / 3600.0f;

  static float clamp_rate(float rate) {
    if (!(rate >= MIN_RATE)) return MIN_RATE;
    return rate > MAX_RATE ? MAX_RATE : rate;
  }

  void set_rate(bool heating, float rate) { (heating ? this->heat_rate_ : this->cool_rate_) = clamp_rate(rate); }
  float rate(bool heating) const { return heating ? this->heat_rate_ : this->cool_rate_; }
  uint32_t measurements() const { return this->measurements_; }

  // Feed one sample. `operating` is true while the device is actively heating
  // or cooling. Returns true when a measurement updated the estimate.
  bool update(float temperature, bool operating, bool heating, uint32_t now_ms) {
    const bool usable = std::isfinite(temperature) && operating;
    if (usable && heating != this->heating_) {
      // Mode changed: what was accumulated belongs to the other mode.
      this->accumulated_ms_ = 0;
      this->accumulated_change_ = 0.0f;
      this->has_previous_ = false;
      this->heating_ = heating;
    }
    bool updated = false;
    if (usable && this->has_previous_) {
      const uint32_t elapsed_ms = now_ms - this->previous_ms_;
      if (elapsed_ms <= MAX_SAMPLE_GAP_MS) {
        this->accumulated_ms_ += elapsed_ms;
        this->accumulated_change_ += temperature - this->previous_temperature_;
        if (this->accumulated_ms_ >= MEASURE_MS) {
          const float change = heating ? this->accumulated_change_ : -this->accumulated_change_;
          const float measured = clamp_rate(change / (this->accumulated_ms_ / 1000.0f));
          float &rate = heating ? this->heat_rate_ : this->cool_rate_;
          rate += SMOOTHING * (measured - rate);
          this->measurements_++;
          this->accumulated_ms_ = 0;
          this->accumulated_change_ = 0.0f;
          updated = true;
        }
      }
    }
    this->has_previous_ = usable;
    this->previous_ms_ = now_ms;
    this->previous_temperature_ = temperature;
    return updated;
  }

 private:
  float heat_rate_{2.0f / 3600.0f};
  float cool_rate_{2.0f / 3600.0f};
  uint32_t measurements_{0};

  bool heating_{true};
  bool has_previous_{false};
  uint32_t previous_ms_{0};
  float previous_temperature_{0.0f};
  uint32_t accumulated_ms_{0};
  float accumulated_change_{0.0f};
};

}  // namespace pid
}  // namespace workflow
