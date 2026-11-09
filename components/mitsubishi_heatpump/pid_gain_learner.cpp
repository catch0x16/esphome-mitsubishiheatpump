#include "pid_gain_learner.h"
#include <algorithm>
#include <cmath>

namespace {
constexpr uint32_t MAGIC = 0x50474c01;  // schema 1
float clamp(float x, float low, float high) { return std::max(low, std::min(x, high)); }
double mean(double value, double seconds) { return seconds > 0 ? value / seconds : 0; }
bool same(const PidGainLearner::Gains &a, const PidGainLearner::Gains &b) {
  for (unsigned i = 0; i < 3; ++i)
    if (std::fabs(a[i]-b[i]) > 1e-6f*std::max(1.f, std::fabs(b[i]))) return false;
  return true;
}
}

PidGainLearner::PidGainLearner(Gains initial, const Config &config,
                             uint32_t fingerprint, uint32_t seed, float output_half_span)
    : config_(config), initial_(initial), accepted_(initial), previous_(initial),
      candidate_(initial), fingerprint_(fingerprint), rng_(seed ? seed : 1) {
  probe_output_ = 100.f*std::atanh(std::min(.5f,.25f/std::max(.01f,output_half_span)));
}

bool PidGainLearner::finite_gains(const Gains &g) {
  for (float x : g) if (!std::isfinite(x) || x < 0) return false;
  return true;
}
bool PidGainLearner::bounded(const Gains &g) const {
  if (!finite_gains(g)) return false;
  for (unsigned i = 0; i < 3; ++i)
    if (g[i] < config_.minimum[i] || g[i] > config_.maximum[i]) return false;
  return true;
}
uint32_t PidGainLearner::random() {
  rng_ ^= rng_ << 13; rng_ ^= rng_ >> 17; rng_ ^= rng_ << 5;
  return rng_;
}
bool PidGainLearner::testing_candidate() const {
  return !suspended_ && (stage_ == Stage::DISCOVERY || stage_ == Stage::CONFIRMATION) &&
         (second_ ? !candidate_first_ : candidate_first_);
}
void PidGainLearner::start_period() {
  period_ = Outcome{};
  elapsed_ms_ = 0;
  have_sample_ = false;
  ++period_number_;
}
void PidGainLearner::start_pair() {
  second_ = false;
  candidate_first_ = (random() & 1u) != 0;
  reference_ = trial_ = Outcome{};
  start_period();
}
void PidGainLearner::start_candidate() {
  stage_ = Stage::DISCOVERY;
  evidence_ = Evidence{};
  // Six local directions, four wider coordinated directions, then two wider
  // directions per coordinate. Zero-valued terms can be explored too.
  static const unsigned coordinates[] = {0,0,1,1,2,2,0,0,0,0,0,0,1,1,2,2,0,0};
  unsigned probe = proposals_++ % 18;
  unsigned coordinate = coordinates[probe];
  candidate_ = accepted_;
  const float scale = probe < 6 ? config_.step : config_.wider_step;
  const float sign = probe % 2 == 0 ? -1.f : 1.f;
  // Nonzero exploration seeds are engineering defaults, not identified gains.
  const float seeds[] = {1.f, .01f, 30.f};
  for (unsigned i = 0; i < 3 && probe < 16; ++i) {
    const bool all = probe == 6 || probe == 7;
    const bool mixed = probe == 8 || probe == 9;
    if (mixed ? i == 2 : (!all && i != coordinate)) continue;
    float step = std::max(std::fabs(accepted_[i]), seeds[i]) * scale;
    candidate_[i] = clamp(accepted_[i] + sign*(mixed && i == 1 ? -step : step), config_.minimum[i], config_.maximum[i]);
  }
  // Every other wider-coordinate sweep explores a command-scale displacement.
  // Runtime command-difference checks prevent applying an excessive jump.
  if (probe >= 10 && probe < 16 && (proposals_/18) % 2 == 0) {
    const float observable_steps[] = {probe_output_/.5f, probe_output_, probe_output_*600.f};
    candidate_[coordinate] = clamp(accepted_[coordinate] + sign*observable_steps[coordinate],
                                   config_.minimum[coordinate], config_.maximum[coordinate]);
  }
  if (probe == 16) candidate_ = initial_;
  if (probe == 17) candidate_ = previous_;
  result_ = same(candidate_, accepted_) ? "candidate_unchanged" : "discovery";
  if (same(candidate_, accepted_)) {
    stage_ = Stage::HOLD;
    hold_start_ = last_ms_;
    hold_duration_ = config_.settling_ms;
    have_sample_ = false;
  } else start_pair();
}
void PidGainLearner::pause() {
  // Lose only the incomplete pair, preserving complete-pair evidence. Resume
  // with a fresh random order, and never leave trial gains active while paused.
  if (!suspended_ && (stage_ == Stage::DISCOVERY || stage_ == Stage::CONFIRMATION)) start_pair();
  suspended_ = true;
  have_sample_ = false;
}
void PidGainLearner::abort(const char *reason) {
  last_evidence_ = evidence_;
  result_ = reason;
  stage_ = Stage::HOLD;
  hold_start_ = last_ms_;
  hold_duration_ = config_.settling_ms;
  have_sample_ = false;
}
void PidGainLearner::reset() { stage_ = Stage::SAVE_RESET; have_sample_ = false; }
void PidGainLearner::rollback() {
  if (revision_ && !same(previous_, accepted_)) {
    stage_ = Stage::SAVE_ROLLBACK;
    have_sample_ = false;
  }
}

void PidGainLearner::observe(const Sample &s, uint32_t now) {
  if (!s.enabled || !s.valid || !std::isfinite(s.temperature) ||
      !std::isfinite(s.target) || !std::isfinite(s.outdoor) ||
      !std::isfinite(s.other_0) || !std::isfinite(s.other_1) ||
      !std::isfinite(s.fusion_weight) || !std::isfinite(s.other_duty) ||
      !std::isfinite(s.unit_setpoint) || !std::isfinite(s.command_a) ||
      !std::isfinite(s.command_b)) {
    pause(); last_ms_ = now; return;
  }
  suspended_ = false;
  if (stage_ == Stage::IDLE) start_candidate();
  if (stage_ == Stage::HOLD) {
    if (uint32_t(now-hold_start_) >= hold_duration_) start_candidate();
    else { last_ms_ = now; return; }
  }
  if (needs_save()) { last_ms_ = now; return; }
  if (std::fabs(s.command_b-s.command_a) > config_.max_command_difference+.001f) {
    abort("inconclusive_command_bound"); last_ms_ = now; return;
  }
  const float over = (s.heating ? 1.f : -1.f)*(s.temperature-s.target);
  if (testing_candidate() && over > s.hysteresis_off + config_.peak_guard) {
    abort("early_rollback_overshoot"); last_ms_ = now; return;
  }
  if (!have_sample_) {
    period_.start_error = std::fabs(s.temperature-s.target);
    have_sample_ = true; last_ = s; last_ms_ = now; return;
  }
  uint32_t dt_ms = now-last_ms_;
  last_ms_ = now;
  if (dt_ms > 30000 || s.heating != last_.heating) {
    pause(); return;
  }
  uint32_t previous_elapsed = elapsed_ms_;
  elapsed_ms_ += dt_ms;
  if (elapsed_ms_ > config_.settling_ms) {
    const uint32_t end_ms = config_.settling_ms + config_.period_ms;
    const uint32_t scored_ms = std::min(elapsed_ms_, end_ms) -
                               std::max(previous_elapsed, config_.settling_ms);
    const double dt = scored_ms/1000.;
    // Derive exposure from integer elapsed time, rather than accumulating
    // floating-point intervals: equal event counts must have equal denominators.
    period_.fireplace_context = s.fireplace_context;
    period_.fusion_context = s.fusion_context;
    period_.others_context = s.others_context;
    period_.seconds = (std::min(elapsed_ms_, end_ms)-config_.settling_ms)/1000.;
    period_.over += std::max(0.f, over)*dt;
    period_.absolute += std::fabs(over)*dt;
    period_.recovery += std::max(0.f, -over-.5f)*dt;
    period_.peak = std::max(period_.peak, over);
    period_.outdoor += s.outdoor*dt;
    period_.fireplace += (s.external_heat ? 1 : 0)*dt;
    period_.others += s.other_duty*dt;
    period_.other_0 += s.other_0*dt; period_.other_1 += s.other_1*dt;
    period_.fusion_mean += s.fusion_weight*dt;
    period_.fusion += std::fabs(s.fusion_weight-last_.fusion_weight);
    period_.operating_seconds += (s.operating ? 1 : 0)*dt;
    if (s.power && std::fabs(s.command_a-s.command_b) >= .49f &&
        std::fabs(s.unit_setpoint-(testing_candidate() ? s.command_b : s.command_a)) < .01f)
      period_.observable_seconds += dt;
    // Target changes remain in error/recovery scoring but must not be counted
    // as evidence of an overshoot-driven hysteresis shutdown.
    if (last_.power && !s.power && std::fabs(s.target-last_.target) < .01f &&
        over >= s.hysteresis_off-.05f && previous_elapsed >= config_.settling_ms)
      ++period_.shutdowns;
  }
  last_ = s;
  if (elapsed_ms_ >= config_.settling_ms + config_.period_ms) finish_period(now);
}

void PidGainLearner::add(Outcome &a, const Outcome &b) {
  a.seconds += b.seconds; a.over += b.over; a.absolute += b.absolute;
  a.recovery += b.recovery; a.outdoor += b.outdoor; a.fireplace += b.fireplace;
  a.others += b.others; a.other_0 += b.other_0; a.other_1 += b.other_1;
  a.fusion += b.fusion; a.fusion_mean += b.fusion_mean;
  a.observable_seconds += b.observable_seconds; a.operating_seconds += b.operating_seconds;
  a.fireplace_context = b.fireplace_context; a.fusion_context = b.fusion_context; a.others_context = b.others_context;
  a.shutdowns += b.shutdowns; a.peak = std::max(a.peak, b.peak);
}
bool PidGainLearner::comparable(const Outcome &a, const Outcome &b) const {
  return a.seconds > 0 && b.seconds > 0 &&
    std::fabs(mean(a.outdoor,a.seconds)-mean(b.outdoor,b.seconds)) <= config_.outdoor_pair_difference &&
    a.fireplace_context == b.fireplace_context && a.fusion_context == b.fusion_context &&
    a.others_context == b.others_context &&
    (!a.fireplace_context || std::fabs(mean(a.fireplace,a.seconds)-mean(b.fireplace,b.seconds)) <= config_.context_pair_difference) &&
    (!a.others_context ||
      (std::fabs(mean(a.other_0,a.seconds)-mean(b.other_0,b.seconds)) <= config_.context_pair_difference &&
       std::fabs(mean(a.other_1,a.seconds)-mean(b.other_1,b.seconds)) <= config_.context_pair_difference)) &&
    (!a.fusion_context ||
      (std::fabs(mean(a.fusion_mean,a.seconds)-mean(b.fusion_mean,b.seconds)) <= 5.f &&
       std::fabs(a.fusion/a.seconds-b.fusion/b.seconds) <= .01)) &&
    a.observable_seconds*1000 >= config_.observable_ms && b.observable_seconds*1000 >= config_.observable_ms;
}
bool PidGainLearner::improved(const Outcome &a, const Outcome &b) const {
  // Predeclared lexicographic ordering, with symmetric meaningful-effect
  // thresholds. Symmetry matters for the paired directional sign test.
  const double av[] = {mean(a.shutdowns,a.seconds), mean(a.over,a.seconds),
                       mean(a.recovery,a.seconds), mean(a.absolute,a.seconds)};
  const double bv[] = {mean(b.shutdowns,b.seconds), mean(b.over,b.seconds),
                       mean(b.recovery,b.seconds), mean(b.absolute,b.seconds)};
  for (unsigned i = 0; i < 4; ++i) {
    double threshold = std::max(i == 0 ? 1e-12 : .01,
                               std::max(av[i],bv[i])*config_.improvement);
    if (std::fabs(av[i]-bv[i]) >= threshold) return bv[i] < av[i];
  }
  return false;
}
bool PidGainLearner::guards(const Evidence &e) const {
  const auto &a = e.reference; const auto &b = e.candidate;
  return b.peak <= a.peak+config_.peak_guard &&
    mean(b.shutdowns,b.seconds) <= mean(a.shutdowns,a.seconds) &&
    mean(b.absolute,b.seconds) <= mean(a.absolute,a.seconds)*(1+config_.guard_relative)+config_.guard_absolute &&
    mean(b.recovery,b.seconds) <= mean(a.recovery,a.seconds)*(1+config_.guard_relative)+config_.guard_absolute;
}
void PidGainLearner::finish_period(uint32_t now) {
  last_period_ = period_; last_period_candidate_ = testing_candidate();
  last_reference_gains_ = accepted_; last_candidate_gains_ = candidate_;
  ++completed_periods_;
  if (testing_candidate()) trial_ = period_; else reference_ = period_;
  if (!second_) { second_ = true; start_period(); }
  else finish_pair(now);
}
double PidGainLearner::sign_probability(unsigned wins, unsigned losses) {
  unsigned n = wins+losses;
  if (!n) return 1;
  double term = std::pow(.5, n), total = 0;
  for (unsigned k = 0; k <= n; ++k) {
    if (k >= wins) total += term;
    if (k < n) term *= double(n-k)/double(k+1);
  }
  return std::min(1., total);
}
void PidGainLearner::finish_pair(uint32_t now) {
  last_pair_confirmation_ = stage_ == Stage::CONFIRMATION;
  ++evidence_.attempts;
  if (reference_.observable_seconds == 0 && trial_.observable_seconds == 0) {
    abort("inconclusive_unobservable_command"); hold_start_ = now; return;
  }
  if (comparable(reference_, trial_)) {
    ++evidence_.pairs;
    add(evidence_.reference, reference_); add(evidence_.candidate, trial_);
    if (improved(reference_,trial_)) ++evidence_.wins;
    else if (improved(trial_,reference_)) ++evidence_.losses;
    result_ = stage_ == Stage::CONFIRMATION ? "confirming" : "discovering";
  } else result_ = "inconclusive_context_or_command";
  evidence_.p = sign_probability(evidence_.wins,evidence_.losses);
  last_evidence_ = evidence_;
  const unsigned required = stage_ == Stage::DISCOVERY ? config_.discovery_pairs : config_.confirmation_pairs;
  if (evidence_.pairs < required && evidence_.attempts < config_.max_pair_attempts) {
    start_pair(); return;
  }
  evidence_.p = sign_probability(evidence_.wins,evidence_.losses);
  last_evidence_ = evidence_;
  if (evidence_.pairs < required) { abort("inconclusive_evidence"); hold_start_ = now; return; }
  if (stage_ == Stage::DISCOVERY) {
    if (guards(evidence_) && improved(evidence_.reference,evidence_.candidate) &&
        evidence_.wins >= (3*config_.discovery_pairs+3)/4) {
      // Reserve and persist an alpha allocation BEFORE consuming confirmation
      // data. Reboots cannot reset the multiple-comparison budget for free.
      if (confirmations_ == UINT32_MAX) { abort("verification_budget_exhausted"); hold_start_ = now; return; }
      const double next = double(confirmations_)+1;
      if (std::pow(.5,config_.confirmation_pairs) > .05/(next*(next+1))) {
        abort("verification_budget_exhausted"); hold_start_ = now; return;
      }
      ++confirmations_;
      stage_ = Stage::SAVE_BUDGET;
      result_ = "confirmation_pending_save";
    } else { abort("not_promising"); hold_start_ = now; }
  } else {
    evidence_.alpha = .05/(double(confirmations_)*(double(confirmations_)+1));
    last_evidence_ = evidence_;
    if (guards(evidence_) && improved(evidence_.reference,evidence_.candidate) &&
        evidence_.p <= evidence_.alpha) {
      stage_ = Stage::SAVE_PROMOTION; result_ = "verified_pending_save";
    } else { abort(guards(evidence_) ? "inconclusive_confirmation" : "rejected_guardrail"); hold_start_ = now; }
  }
  have_sample_ = false;
}
bool PidGainLearner::needs_save() const {
  return stage_ == Stage::SAVE_BUDGET || stage_ == Stage::SAVE_PROMOTION ||
         stage_ == Stage::SAVE_RESET || stage_ == Stage::SAVE_ROLLBACK;
}
PidGainLearner::Record PidGainLearner::record() const {
  Record r{};
  r.magic = MAGIC; r.fingerprint = fingerprint_; r.configured = initial_;
  r.accepted = accepted_; r.previous = previous_; r.revision = revision_;
  r.confirmations = confirmations_; r.proposals = proposals_; r.rng_state = rng_;
  r.evidence = verified_evidence_;
  if (stage_ == Stage::SAVE_PROMOTION) {
    r.previous = accepted_; r.accepted = candidate_; ++r.revision;
    r.evidence = evidence_;
  } else if (stage_ == Stage::SAVE_RESET) {
    r.previous = accepted_; r.accepted = initial_; ++r.revision;
    r.evidence = Evidence{};
  } else if (stage_ == Stage::SAVE_ROLLBACK) {
    r.previous = accepted_; r.accepted = previous_; ++r.revision;
    r.evidence = Evidence{};
  }
  return r;
}
bool PidGainLearner::restore(const Record &r) {
  if (!std::isfinite(r.evidence.p) || !std::isfinite(r.evidence.alpha) ||
      r.evidence.p < 0 || r.evidence.p > 1 || r.evidence.alpha < 0 || r.evidence.alpha > .05 ||
      r.evidence.wins+r.evidence.losses > r.evidence.pairs ||
      (r.evidence.pairs && (r.evidence.pairs != config_.confirmation_pairs ||
       r.evidence.p > r.evidence.alpha || !r.revision))) return false;

  if (r.magic != MAGIC || r.fingerprint != fingerprint_ || !bounded(r.accepted) ||
      !bounded(r.previous) || !finite_gains(r.configured) ||
      (!same(r.configured,initial_) && !same(r.accepted,initial_))) return false;
  accepted_ = r.accepted; previous_ = r.previous; revision_ = r.revision;
  confirmations_ = r.confirmations; proposals_ = r.proposals; rng_ = r.rng_state ? r.rng_state : 1;
  verified_evidence_ = r.evidence;
  stage_ = Stage::IDLE; result_ = r.evidence.pairs ? "restored_verified" : "restored_baseline"; have_sample_ = false;
  return true;
}
void PidGainLearner::saved(bool success, uint32_t now) {
  if (!needs_save()) return;
  if (!success) { result_ = "save_failed_retrying"; return; }
  if (stage_ == Stage::SAVE_BUDGET) {
    // Discovery evidence is never counted toward confirmation.
    evidence_ = Evidence{};
    stage_ = Stage::CONFIRMATION; result_ = "confirming"; start_pair();
  } else {
    Record r = record();
    accepted_ = r.accepted; previous_ = r.previous; revision_ = r.revision;
    verified_evidence_ = r.evidence;
    result_ = stage_ == Stage::SAVE_PROMOTION ? "verified_saved_copy_to_yaml" :
              stage_ == Stage::SAVE_RESET ? "reset_to_yaml_saved" : "rollback_saved";
    stage_ = Stage::HOLD; hold_start_ = now; hold_duration_ = config_.hold_ms; have_sample_ = false;
  }
}
float PidGainLearner::progress() const {
  const float required = confirming() ? config_.confirmation_pairs : config_.discovery_pairs;
  return std::min(1.f, evidence_.pairs/required);
}

PidGainProfiles::PidGainProfiles(PidGainLearner::Gains heat, PidGainLearner::Gains cool,
    const PidGainLearner::Config &config, uint32_t fingerprint, uint32_t seed, float span)
    : learners_{{PidGainLearner(heat, config, fingerprint ^ 0x48454154u, seed, span),
                 PidGainLearner(cool, config, fingerprint ^ 0x434f4f4cu, seed ^ 0x434f4f4cu, span)}} {}

PidGainLearner *PidGainProfiles::select(Mode mode) {
  const int next = mode == Mode::HEAT ? 0 : mode == Mode::COOL ? 1 : -1;
  if (next != selected_) {
    if (selected_ >= 0) learners_[selected_].pause();
    selected_ = next;
  }
  return selected_ < 0 ? nullptr : &learners_[selected_];
}
