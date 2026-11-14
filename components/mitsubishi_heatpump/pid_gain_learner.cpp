#include "pid_gain_learner.h"
#include <algorithm>
#include <cmath>

namespace {
constexpr uint32_t MAGIC = 0x50474c03;  // schema 3: whole-response and matched-context evidence
float clamp(float x, float low, float high) { return std::max(low, std::min(x, high)); }
double mean(double value, double seconds) { return seconds > 0 ? value / seconds : 0; }
bool same(const PidGainLearner::Gains &a, const PidGainLearner::Gains &b) {
  for (unsigned i = 0; i < 3; ++i)
    if (std::fabs(a[i]-b[i]) > 1e-6f*std::max(1.f, std::fabs(b[i]))) return false;
  return true;
}
}

PidGainLearner::PidGainLearner(Gains initial, const Config &config,
                             uint32_t fingerprint, uint32_t seed)
    : config_(config), initial_(initial), accepted_(initial), previous_(initial),
      candidate_(initial), fingerprint_(fingerprint), rng_(seed ? seed : 1) {}

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
  return !suspended_ && !awaiting_period_ && (stage_ == Stage::DISCOVERY || stage_ == Stage::CONFIRMATION) &&
         (second_ ? !candidate_first_ : candidate_first_);
}
void PidGainLearner::start_period() {
  period_ = Outcome{};
  awaiting_period_ = true;
  waiting_clock_started_ = false;
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
bool PidGainLearner::size_candidate(const PidCommandState &state, unsigned direction, Gains &candidate) const {
  const float baseline = state.command(accepted_);
  // Eligibility thresholds describe experimental excitation, not comfort limits:
  // error >= 0.1 C, integral >= 0.1 C*s, slope >= 0.1 C/min.
  const float informative[] = {.1f, .1f, 1.f/600.f};
  if (direction >= 16) {
    candidate = direction == 16 ? initial_ : previous_;
    for (unsigned i=0;i<3;++i)
      if (candidate[i] != accepted_[i] && std::fabs(state.terms[i]) < informative[i]) return false;
    const float difference = std::fabs(state.command(candidate)-baseline);
    return bounded(candidate) && difference >= .49f && difference <= .501f;
  }
  const unsigned coordinate = direction < 6 ? direction/2 : direction < 10 ? 0 : (direction-10)/2;
  const bool all = direction == 6 || direction == 7;
  const bool mixed = direction == 8 || direction == 9;
  const float sign = direction%2 == 0 ? -1.f : 1.f;
  const float seeds[] = {1.f, .01f, 30.f};
  Gains delta{{0,0,0}};
  float maximum_scale = 1e30f;
  for (unsigned i=0;i<3;++i) {
    if (mixed ? i == 2 : (!all && i != coordinate)) continue;
    if (std::fabs(state.terms[i]) < informative[i]) return false;
    delta[i] = sign*(mixed && i == 1 ? -1.f : 1.f)*std::max(std::fabs(accepted_[i]),seeds[i]);
    const float distance = delta[i] > 0 ? config_.maximum[i]-accepted_[i] : config_.minimum[i]-accepted_[i];
    maximum_scale = std::min(maximum_scale, distance/delta[i]);
  }
  auto at = [&](float scale) {
    Gains g;
    for (unsigned i=0;i<3;++i) g[i] = clamp(accepted_[i]+scale*delta[i],config_.minimum[i],config_.maximum[i]);
    return g;
  };
  if (maximum_scale <= 0 || state.command(at(maximum_scale)) == baseline) return false;
  // Bracket and bisect the first real packet change, including tanh, raw limits
  // and 0.5 C rounding. No trial time is spent searching for this boundary.
  const float seed_scale = direction < 6 ? config_.step : config_.wider_step;
  float hi = std::min(maximum_scale,seed_scale), lo = 0;
  while (hi < maximum_scale && state.command(at(hi)) == baseline)
    hi = std::min(maximum_scale,hi*2);
  for (unsigned i=0;i<32;++i) {
    const float middle = (lo+hi)*.5f;
    if (state.command(at(middle)) == baseline) lo=middle; else hi=middle;
  }
  candidate=at(hi);
  // Occasional wider alternatives can explore farther within the same single
  // command step. Never enlarge the permitted command change to force a test.
  if (direction >= 6) {
    const auto wider=at(std::min(maximum_scale,config_.wider_step));
    if (std::fabs(state.command(wider)-baseline) == .5f) candidate=wider;
  }
  const float difference=std::fabs(state.command(candidate)-baseline);
  return bounded(candidate) && difference >= .49f && difference <= .501f &&
         difference <= config_.max_command_difference+.001f;
}

bool PidGainLearner::start_candidate(const PidCommandState &state) {
  // Search every direction at most once per sample. Unsuitable directions are
  // deferred without entering discovery, starting a timer, or consuming alpha.
  for (unsigned n=0;n<18;++n) {
    const unsigned direction=next_direction_;
    next_direction_=(next_direction_+1)%18;
    Gains candidate;
    if (!size_candidate(state,direction,candidate)) continue;
    candidate_=candidate;
    ++proposals_;
    stage_=Stage::DISCOVERY;
    evidence_=Evidence{};
    pending_={}; phase_observations_=phase_observable_=0;
    result_="discovery";
    start_pair();
    return true;
  }
  result_="waiting_for_informative_command";
  return false;
}
void PidGainLearner::pause() {
  // Drop only the unfinished observation. Completed observations and matched
  // evidence survive a pause; retry the already-randomized arm assignment.
  if (!suspended_ && (stage_ == Stage::DISCOVERY || stage_ == Stage::CONFIRMATION)) start_period();
  context_initialized_ = false;
  suspended_ = true;
  awaiting_period_ = true;
  have_sample_ = false;
}
void PidGainLearner::abort(const char *reason) {
  last_evidence_ = evidence_;
  pending_={};
  result_ = reason;
  stage_ = Stage::HOLD;
  hold_start_ = last_ms_;
  hold_duration_ = config_.settling_ms;
  have_sample_ = false;
}
void PidGainLearner::reset() { pending_={}; stage_ = Stage::SAVE_RESET; have_sample_ = false; }
void PidGainLearner::rollback() {
  if (revision_ && !same(previous_, accepted_)) {
    pending_={};
    stage_ = Stage::SAVE_ROLLBACK;
    have_sample_ = false;
  }
}

unsigned PidGainLearner::pending_observations() const {
  unsigned count=0;
  for (const auto &item:pending_) if (item.used) ++count;
  return count;
}

bool PidGainLearner::update_context(const Sample &s, uint32_t now) {
  if (!context_initialized_ || uint32_t(now-context_last_ms_) > 30000 ||
      context_heating_ != s.heating) {
    context_initialized_=true; context_since_=fireplace_since_=now;
    context_last_ms_=now; context_temperature_=s.temperature; context_trend_=0;
    context_heating_=s.heating; context_fireplace_=s.external_heat; fireplace_transition_seen_=false;
  } else {
    const uint32_t dt=now-context_last_ms_;
    if (dt) {
      const float rate=(s.temperature-context_temperature_)*60000.f/dt;
      const float alpha=float(dt)/(config_.context_ms+float(dt));
      context_trend_+=alpha*(rate-context_trend_);
    }
    if (s.external_heat != context_fireplace_) {
      context_fireplace_=s.external_heat; fireplace_since_=now; fireplace_transition_seen_=true;
    }
    context_last_ms_=now; context_temperature_=s.temperature;
  }
  // Zero history is used only by portable mechanism fixtures. Production uses
  // a five-minute trend independent of PID resets, power and target changes.
  if (!config_.context_ms) context_trend_=-s.command_state.terms[2]*60;
  return uint32_t(now-context_since_) >= config_.context_ms &&
      (!s.fireplace_context || uint32_t(now-fireplace_since_) >= config_.context_ms);
}

PidGainLearner::Outcome PidGainLearner::starting_context(const Sample &s, uint32_t now) const {
  Outcome out;
  out.start_error=(s.heating?1.f:-1.f)*(s.temperature-s.target);
  out.start_trend=context_trend_; out.start_target=out.end_target=s.target;
  out.start_outdoor=s.outdoor; out.heating=s.heating;
  out.fireplace_context=s.fireplace_context; out.start_fireplace=s.external_heat;
  out.fusion_context=s.fusion_context; out.others_context=s.others_context;
  // Treat older fireplace states as established; retain recent transition age.
  out.fireplace_stable_ms=std::min(uint32_t(now-fireplace_since_),config_.context_ms*6);
  out.fireplace_recent_transition=fireplace_transition_seen_ &&
      uint32_t(now-fireplace_since_)<config_.context_ms*6;
  return out;
}

bool PidGainLearner::matching_start(const Outcome &a, const Outcome &b) const {
  return a.heating==b.heating && a.fireplace_context==b.fireplace_context &&
    a.fusion_context==b.fusion_context && a.others_context==b.others_context &&
    std::fabs(a.start_error-b.start_error) <= config_.start_error_difference &&
    std::fabs(a.start_trend-b.start_trend) <= config_.start_trend_difference &&
    std::fabs(a.start_target-b.start_target) <= .1f &&
    std::fabs(a.start_outdoor-b.start_outdoor) <= config_.outdoor_pair_difference &&
    (!a.fireplace_context || (a.start_fireplace==b.start_fireplace &&
      a.fireplace_recent_transition==b.fireplace_recent_transition &&
      (!a.fireplace_recent_transition ||
       std::fabs(double(a.fireplace_stable_ms)-b.fireplace_stable_ms) <= config_.context_ms)));
}

bool PidGainLearner::matching_pending_start(const Outcome &start) const {
  const bool candidate_arm=second_?!candidate_first_:candidate_first_;
  bool have_opposite=false;
  for (const auto &item:pending_) {
    if (!item.used || item.candidate==candidate_arm) continue;
    have_opposite=true;
    if (matching_start(start,item.outcome)) return true;
  }
  return !have_opposite;
}

void PidGainLearner::expire_pending(uint32_t now) {
  for (auto &item:pending_)
    if (item.used && uint32_t(now-item.finished_ms)>config_.retention_ms) item.used=false;
}

void PidGainLearner::advance_period() {
  if (!second_) { second_=true; start_period(); }
  else start_pair();
}

void PidGainLearner::observe(const Sample &input, uint32_t now) {
  const Sample &s=input;
  if (!s.enabled || !s.valid || !std::isfinite(s.temperature) ||
      !std::isfinite(s.target) || !std::isfinite(s.outdoor) ||
      !std::isfinite(s.other_0) || !std::isfinite(s.other_1) ||
      !std::isfinite(s.fusion_weight) || !std::isfinite(s.other_duty) ||
      !std::isfinite(s.unit_setpoint) || !s.command_state.valid()) {
    pause(); last_ms_ = now; return;
  }
  const bool context_ready=update_context(s,now);
  expire_pending(now);
  if (needs_save()) { last_ms_ = now; return; }
  if (stage_ == Stage::HOLD && uint32_t(now-hold_start_) < hold_duration_) {
    last_ms_=now; return;
  }
  if (stage_ == Stage::IDLE || stage_ == Stage::HOLD) {
    if (!s.power) { result_="waiting_for_enabled_unit"; last_ms_=now; return; }
    if (!context_ready) { result_="waiting_for_recent_context"; last_ms_=now; return; }
    if (!start_candidate(s.command_state)) { last_ms_=now; return; }
  }
  const float command_a=s.command_state.command(accepted_);
  const float command_b=s.command_state.command(candidate_);
  if (!awaiting_period_ && std::fabs(command_b-command_a) > config_.max_command_difference+.001f) {
    abort("inconclusive_command_bound"); last_ms_ = now; return;
  }
  const float over = (s.heating ? 1.f : -1.f)*(s.temperature-s.target);
  if (awaiting_period_) {
    // A fixed candidate can lose excitation between arms. Defer it, but do not
    // let an unstartable comparison monopolize the tuner indefinitely.
    if (!waiting_clock_started_) { waiting_since_=now; waiting_clock_started_=true; }
    if (!pending_observations() && uint32_t(now-waiting_since_) >= config_.period_ms) {
      abort("inconclusive_start_timeout"); last_ms_=now; return;
    }
    if (!s.power) { result_="waiting_for_enabled_unit"; last_ms_=now; return; }
    if (std::fabs(command_b-command_a) > config_.max_command_difference+.001f) {
      result_="waiting_for_command_bound"; last_ms_=now; return;
    }
    if (std::fabs(command_b-command_a) < .49f) {
      result_="waiting_for_command_separation"; last_ms_=now; return;
    }
    if (over > s.hysteresis_off+config_.peak_guard) {
      result_="waiting_for_recovery"; last_ms_=now; return;
    }
    if (!context_ready) { result_="waiting_for_recent_context"; last_ms_=now; return; }
    if (!matching_pending_start(starting_context(s,now))) {
      result_="waiting_for_matching_context"; last_ms_=now; return;
    }
    awaiting_period_=false;
    suspended_=false;
    result_=stage_==Stage::CONFIRMATION ? "confirming" : "discovery";
  }
  if (testing_candidate() && over > s.hysteresis_off + config_.peak_guard) {
    abort("early_rollback_overshoot"); last_ms_ = now; return;
  }
  if (!have_sample_) {
    period_=starting_context(s,now);
    have_sample_ = true; last_ = s; last_ms_ = now; return;
  }
  uint32_t dt_ms = now-last_ms_;
  last_ms_ = now;
  if (dt_ms > 30000 || s.heating != last_.heating) {
    pause(); return;
  }
  uint32_t previous_elapsed = elapsed_ms_;
  elapsed_ms_ += dt_ms;
  {
    const uint32_t end_ms=config_.settling_ms+config_.period_ms;
    const uint32_t scored_ms=std::min(elapsed_ms_,end_ms)-previous_elapsed;
    const double dt=scored_ms/1000.;
    // Score the entire fixed window, including its initial response. Derive
    // exposure from integer time so equal event counts have equal denominators.
    period_.seconds=std::min(elapsed_ms_,end_ms)/1000.;
    period_.target+=s.target*dt;
    period_.target_movement+=std::fabs(s.target-last_.target);
    period_.end_target=s.target;
    period_.fireplace_changed|=s.fireplace_context!=period_.fireplace_context ||
      (s.fireplace_context && s.external_heat!=period_.start_fireplace);
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
    if (s.power && std::fabs(command_a-command_b) >= .49f &&
        std::fabs(s.unit_setpoint-(testing_candidate() ? command_b : command_a)) < .01f)
      period_.observable_seconds += dt;
    // Target changes remain in error/recovery scoring but must not be counted
    // as evidence of an overshoot-driven hysteresis shutdown.
    if (last_.power && !s.power && std::fabs(s.target-last_.target) < .01f &&
        over >= s.hysteresis_off-.05f)
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
  a.target += b.target;
  a.shutdowns += b.shutdowns; a.peak = std::max(a.peak, b.peak);
}
bool PidGainLearner::comparable(const Outcome &a, const Outcome &b) const {
  return a.seconds > 0 && b.seconds > 0 && matching_start(a,b) &&
    !a.fireplace_changed && !b.fireplace_changed &&
    std::fabs(mean(a.target,a.seconds)-mean(b.target,b.seconds)) <= .1f &&
    std::fabs(a.target_movement-b.target_movement) <= .1f &&
    std::fabs(a.end_target-b.end_target) <= .1f &&
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
  last_pair_confirmation_=stage_==Stage::CONFIRMATION;
  ++phase_observations_;
  evidence_.attempts=(phase_observations_+1)/2;
  expire_pending(now);
  // Qualification depends on context and actual command exposure, never on the
  // improvement score. Oldest compatible counterpart wins, used exactly once.
  const bool exposed=period_.observable_seconds*1000 >= config_.observable_ms;
  if (exposed) ++phase_observable_;
  const bool qualified=!period_.fireplace_changed && exposed;
  Pending *match=nullptr;
  if (qualified) {
    for (auto &item:pending_) {
      if (!item.used || item.candidate==last_period_candidate_ || !comparable(period_,item.outcome)) continue;
      if (!match || uint32_t(now-item.finished_ms)>uint32_t(now-match->finished_ms)) match=&item;
    }
  }
  if (match) {
    reference_=last_period_candidate_?match->outcome:period_;
    trial_=last_period_candidate_?period_:match->outcome;
    match->used=false;
    finish_pair(now);
  } else {
    if (qualified) {
      Pending *slot=nullptr;
      for (auto &item:pending_) {
        if (!item.used) { slot=&item; break; }
        if (!slot || uint32_t(now-item.finished_ms)>uint32_t(now-slot->finished_ms)) slot=&item;
      }
      slot->outcome=period_;slot->candidate=last_period_candidate_;
      slot->finished_ms=now;slot->used=true;
      result_="retained_for_matching_context";
    } else result_=period_.fireplace_changed?"inconclusive_fireplace_transition":"inconclusive_unobservable_command";
    last_evidence_=evidence_;
  }
  if (stage_==Stage::DISCOVERY || stage_==Stage::CONFIRMATION) {
    const unsigned required=stage_==Stage::DISCOVERY?config_.discovery_pairs:config_.confirmation_pairs;
    if (phase_observations_>=2 && !phase_observable_) {
      abort("inconclusive_unobservable_command");hold_start_=now;
    } else if (evidence_.pairs<required && phase_observations_>=2*config_.max_pair_attempts) {
      abort("inconclusive_evidence");hold_start_=now;
    } else advance_period();
  }
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
  ++evidence_.pairs;
  add(evidence_.reference,reference_);add(evidence_.candidate,trial_);
  if (improved(reference_,trial_)) ++evidence_.wins;
  else if (improved(trial_,reference_)) ++evidence_.losses;
  result_=stage_==Stage::CONFIRMATION?"confirming":"discovering";
  evidence_.p = sign_probability(evidence_.wins,evidence_.losses);
  last_evidence_ = evidence_;
  const unsigned required = stage_ == Stage::DISCOVERY ? config_.discovery_pairs : config_.confirmation_pairs;
  if (evidence_.pairs < required) {
    return;
  }
  evidence_.p = sign_probability(evidence_.wins,evidence_.losses);
  last_evidence_ = evidence_;
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
  r.confirmations = confirmations_; r.proposals = proposals_; r.rng_state = rng_; r.next_direction = next_direction_;
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

  if (r.next_direction >= 18 || r.magic != MAGIC || r.fingerprint != fingerprint_ || !bounded(r.accepted) ||
      !bounded(r.previous) || !finite_gains(r.configured) ||
      (!same(r.configured,initial_) && !same(r.accepted,initial_))) return false;
  accepted_ = r.accepted; previous_ = r.previous; revision_ = r.revision;
  confirmations_ = r.confirmations; proposals_ = r.proposals; next_direction_ = r.next_direction; rng_ = r.rng_state ? r.rng_state : 1;
  verified_evidence_ = r.evidence;
  pending_={}; context_initialized_=false;
  stage_ = Stage::IDLE; result_ = r.evidence.pairs ? "restored_verified" : "restored_baseline"; have_sample_ = false;
  return true;
}
void PidGainLearner::saved(bool success, uint32_t now) {
  if (!needs_save()) return;
  if (!success) { result_ = "save_failed_retrying"; return; }
  if (stage_ == Stage::SAVE_BUDGET) {
    // Discovery evidence is never counted toward confirmation.
    evidence_ = Evidence{}; pending_={}; phase_observations_=phase_observable_=0;
    stage_ = Stage::CONFIRMATION; result_ = "confirming"; start_pair();
  } else {
    Record r = record();
    pending_={};
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
    const PidGainLearner::Config &config, uint32_t fingerprint, uint32_t seed)
    : learners_{{PidGainLearner(heat, config, fingerprint ^ 0x48454154u, seed),
                 PidGainLearner(cool, config, fingerprint ^ 0x434f4f4cu, seed ^ 0x434f4f4cu)}} {}

PidGainLearner *PidGainProfiles::select(Mode mode) {
  const int next = mode == Mode::HEAT ? 0 : mode == Mode::COOL ? 1 : -1;
  if (next != selected_) {
    if (selected_ >= 0) learners_[selected_].pause();
    selected_ = next;
  }
  return selected_ < 0 ? nullptr : &learners_[selected_];
}
