#pragma once

#include <array>
#include <cstdint>

// Experimental forward-learning supervisor. No ESPHome dependencies, power
// switching, temperature forecasting, or claims of global optimality.
class PidGainLearner {
 public:
  using Gains = std::array<float, 3>;
  struct Config {
    uint32_t period_ms = 7200000, settling_ms = 1800000, hold_ms = 86400000;
    unsigned discovery_pairs = 4, confirmation_pairs = 24, max_pair_attempts = 48;
    float step = .10f, wider_step = .50f;
    float improvement = .05f, guard_relative = .05f, guard_absolute = .02f;
    float peak_guard = .10f, max_command_difference = .5f;
    float outdoor_pair_difference = 3.f, context_pair_difference = .25f;
    uint32_t observable_ms = 120000;
    Gains minimum{{0, 0, 0}}, maximum{{250, 10, 7200}};
  };
  struct Sample {
    float temperature = 0, target = 0, outdoor = 0, fusion_weight = 100;
    float other_duty = 0, other_0 = 0, other_1 = 0, unit_setpoint = 0, command_a = 0, command_b = 0;
    bool power = false, operating = false, external_heat = false;
    bool valid = false, enabled = false;
    // positive means past target in the conditioning direction.
    bool heating = true;
    float hysteresis_off = .75f;
  };
  struct Outcome {
    double seconds = 0, over = 0, absolute = 0, recovery = 0;
    double outdoor = 0, fireplace = 0, others = 0, other_0 = 0, other_1 = 0, fusion = 0, fusion_mean = 0;
    double observable_seconds = 0, operating_seconds = 0;
    float peak = 0, start_error = 0;
    unsigned shutdowns = 0;
  };
  enum class Stage { IDLE, DISCOVERY, SAVE_BUDGET, CONFIRMATION,
                     SAVE_PROMOTION, HOLD, SAVE_RESET, SAVE_ROLLBACK };
  struct Evidence {
    Outcome reference{}, candidate{};
    unsigned pairs = 0, wins = 0, losses = 0, attempts = 0;
    double p = 1, alpha = 0;
  };
  struct Record {
    uint32_t magic = 0, fingerprint = 0, revision = 0;
    uint32_t confirmations = 0, proposals = 0, rng_state = 1;
    Gains configured{}, accepted{}, previous{};
    Evidence evidence{};
  };

  PidGainLearner(Gains initial, const Config &config, uint32_t fingerprint,
                 uint32_t seed = 1, float output_half_span = 3.75f);
  void observe(const Sample &sample, uint32_t now);
  void pause();
  void abort(const char *reason);
  void reset();
  void rollback();
  bool restore(const Record &record);
  Record record() const;
  bool needs_save() const;
  void saved(bool success, uint32_t now);

  Gains gains() const { return testing_candidate() ? candidate_ : accepted_; }
  Gains accepted_gains() const { return accepted_; }
  Gains candidate_gains() const { return candidate_; }
  Stage stage() const { return stage_; }
  bool testing_candidate() const;
  bool confirming() const { return stage_ == Stage::CONFIRMATION; }
  const Evidence &evidence() const { return evidence_; }
  const Evidence &last_evidence() const { return last_evidence_; }
  const Evidence &verified_evidence() const { return verified_evidence_; }
  const Outcome &last_period() const { return last_period_; }
  Gains last_reference_gains() const { return last_reference_gains_; }
  Gains last_candidate_gains() const { return last_candidate_gains_; }
  uint32_t completed_periods() const { return completed_periods_; }
  bool last_period_candidate() const { return last_period_candidate_; }
  bool last_pair_confirmation() const { return last_pair_confirmation_; }
  const char *result() const { return result_; }
  uint32_t revision() const { return revision_; }
  uint32_t proposals() const { return proposals_; }
  unsigned period_number() const { return period_number_; }
  float progress() const;
  float command_limit() const { return config_.max_command_difference; }
  static double sign_probability(unsigned wins, unsigned losses);
  static bool finite_gains(const Gains &g);

 private:
  Config config_;
  Gains initial_, accepted_, previous_, candidate_, last_reference_gains_{}, last_candidate_gains_{};
  float probe_output_ = 6.68f;
  uint32_t fingerprint_, rng_, revision_ = 0, confirmations_ = 0, proposals_ = 0;
  Stage stage_ = Stage::IDLE;
  Evidence evidence_{}, last_evidence_{}, verified_evidence_{};
  Outcome period_{}, reference_{}, trial_{}, last_period_{};
  uint32_t completed_periods_ = 0;
  bool last_period_candidate_ = false, last_pair_confirmation_ = false;
  Sample last_{};
  bool have_sample_ = false, second_ = false, candidate_first_ = false;
  bool suspended_ = false;
  uint32_t last_ms_ = 0, elapsed_ms_ = 0, hold_start_ = 0, hold_duration_ = 0;
  unsigned period_number_ = 0;
  const char *result_ = "collecting";
  uint32_t random();
  bool bounded(const Gains &g) const;
  void start_candidate();
  void start_pair();
  void start_period();
  void finish_period(uint32_t now);
  void finish_pair(uint32_t now);
  bool comparable(const Outcome &a, const Outcome &b) const;
  bool guards(const Evidence &e) const;
  bool improved(const Outcome &a, const Outcome &b) const;
  static void add(Outcome &a, const Outcome &b);
};
