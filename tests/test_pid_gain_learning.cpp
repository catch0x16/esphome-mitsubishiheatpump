#include "pid_gain_learner.h"
#include "adaptive_pid.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <set>

using Learner = PidGainLearner;
using Stage = Learner::Stage;

// Deterministic command excitation for testing evidence/persistence separately
// from the plant. Physical controller state is exercised by closed-loop tests.
PidCommandState evidence_command_state() {
  PidCommandState state;
  state.target=20;state.output_min=18;state.output_max=25.5;
  state.low=18;state.high=22;
  state.terms={{-.4f,-.5f,-.003f}};
  return state;
}


struct Fixture {
  static Learner::Config config() {
    Learner::Config c;
    c.period_ms = 1000; c.settling_ms = 200; c.hold_ms = 1000;
    c.observable_ms = 100;
    c.context_ms = 0; // Isolate evidence mechanics; dedicated tests exercise real history.
    return c;
  }
  Learner tuner;
  uint32_t now = 1000;
  bool benefit = true, mismatch = false, no_command = false, bad_recovery = false;
  bool save_budget = true;
  Fixture() : tuner({{20, 2, 30}}, config(), 123, 9876) {}
  Learner::Sample frame() {
    Learner::Sample s;
    bool b = tuner.testing_candidate();
    s.valid = s.enabled = s.power = s.operating = true;
    s.fireplace_context=s.fusion_context=s.others_context=true;
    s.target = 20;
    s.temperature = 20 + ((b && benefit) ? .15f : .4f);
    if (b && bad_recovery) s.temperature = 18;
    s.command_state=evidence_command_state();
    if (no_command) s.command_state.low=s.command_state.high=20;
    s.unit_setpoint=s.command_state.command(b?tuner.candidate_gains():tuner.accepted_gains());
    s.outdoor = mismatch && b ? 18 : 10;
    s.fusion_weight = 75;
    return s;
  }
  void tick() {
    now += 100;
    tuner.observe(frame(), now);
    if (save_budget && tuner.stage() == Stage::SAVE_BUDGET) tuner.saved(true,now);
  }
  void until(Stage stage) {
    for (unsigned i=0;i<30000 && tuner.stage()!=stage;++i) tick();
    assert(tuner.stage()==stage);
  }
};

void confirmation_and_persistence() {
  Fixture f;
  auto initial = f.tuner.accepted_gains();
  f.until(Stage::CONFIRMATION);
  assert(f.tuner.accepted_gains() == initial);
  assert(f.tuner.evidence().pairs == 0);  // no discovery leakage
  assert(f.tuner.pending_observations()==0);
  f.until(Stage::SAVE_PROMOTION);
  assert(f.tuner.evidence().pairs == 24);
  assert(f.tuner.evidence().p <= f.tuner.evidence().alpha);
  assert(f.tuner.accepted_gains() == initial);
  auto record = f.tuner.record();
  assert(record.accepted[0] < initial[0]);
  f.tuner.saved(false,f.now);
  assert(f.tuner.accepted_gains()==initial);
  assert(std::strcmp(f.tuner.result(),"save_failed_retrying")==0);
  f.tuner.saved(true,f.now);
  assert(f.tuner.accepted_gains() == record.accepted);
  assert(f.tuner.revision()==1);
  Learner restored(initial, Fixture::config(),123);
  assert(restored.restore(record));
  assert(restored.accepted_gains()==record.accepted);
  assert(restored.record().evidence.pairs==24);
  assert(restored.record().next_direction==record.next_direction);
  auto invalid_direction=record;invalid_direction.next_direction=18;
  assert(!restored.restore(invalid_direction));
  // Copying accepted values into YAML keeps compatible history.
  Learner copied(record.accepted,Fixture::config(),123);
  assert(copied.restore(record));
  Learner manual({{21,2,30}},Fixture::config(),123);
  assert(!manual.restore(record));
  Learner changed(initial,Fixture::config(),124);
  assert(!changed.restore(record));
  record.accepted[0] = std::numeric_limits<float>::quiet_NaN();
  assert(!restored.restore(record));
}

void confirmation_can_fail_without_promoting() {
  Fixture f;
  auto initial=f.tuner.accepted_gains();
  f.until(Stage::CONFIRMATION);
  f.benefit = false;
  f.until(Stage::HOLD);
  assert(f.tuner.accepted_gains()==initial);
  assert(f.tuner.revision()==0);
  assert(std::strcmp(f.tuner.result(),"inconclusive_confirmation")==0);
}

void guards_and_context() {
  Fixture f;
  f.bad_recovery=true;
  f.until(Stage::HOLD);
  assert(f.tuner.revision()==0);
  Fixture g;
  g.mismatch=true;
  g.until(Stage::HOLD);
  assert(g.tuner.evidence().pairs==0);
  assert(g.tuner.revision()==0);
  Fixture h;
  h.no_command=true;
  for (unsigned i=0;i<100;++i) h.tick();
  assert(h.tuner.stage()==Stage::IDLE);
  assert(h.tuner.proposals()==0);
  assert(h.tuner.evidence().pairs==0);
  assert(h.tuner.revision()==0);
}

void missing_data_and_gaps_restore_accepted() {
  Fixture f;
  for (int i=0;i<200 && !f.tuner.testing_candidate();++i) f.tick();
  assert(f.tuner.testing_candidate());
  auto s=f.frame();s.valid=false;
  f.tuner.observe(s,f.now+=100);
  assert(f.tuner.gains()==f.tuner.accepted_gains());
  assert(!f.tuner.testing_candidate());
  s.valid=true;
  f.tuner.observe(s,f.now+=100);
  f.tuner.observe(s,f.now+=60000);
  assert(!f.tuner.testing_candidate());
}

void saved_budget_cannot_be_reused_after_reboot() {
  Fixture f;
  f.save_budget=false;
  f.until(Stage::SAVE_BUDGET);
  auto r=f.tuner.record();
  assert(r.confirmations==1);
  assert(r.accepted==f.tuner.accepted_gains());
  Learner rebooted(r.configured,Fixture::config(),123);
  assert(rebooted.restore(r));
  // A trial is never restored; only the old accepted set is in this record.
  assert(rebooted.gains()==r.accepted);
  assert(rebooted.record().confirmations==1);
  rebooted.reset();
  auto reset=rebooted.record();
  assert(reset.confirmations==1);  // manual reset does not replenish alpha
}

void gains_search_both_directions_and_wider() {
  Fixture f;
  f.benefit=false;
  std::set<unsigned> proposals;
  bool p_up=false,p_down=false,i_up=false,i_down=false,d_up=false,d_down=false,wider=false;
  auto a=f.tuner.accepted_gains();
  for (unsigned i=0;i<30000 && f.tuner.proposals()<15;++i) {
    f.tick();
    auto b=f.tuner.candidate_gains();
    proposals.insert(f.tuner.proposals());
    p_up|=b[0]>a[0];p_down|=b[0]<a[0];
    i_up|=b[1]>a[1];i_down|=b[1]<a[1];
    d_up|=b[2]>a[2];d_down|=b[2]<a[2];
    wider|=std::fabs(b[0]-a[0])>a[0]*.25;
  }
  assert(p_up&&p_down&&i_up&&i_down&&d_up&&d_down&&wider);
}

void confirmation_rejects_single_lucky_period() {
  Fixture f;
  f.until(Stage::CONFIRMATION);
  auto initial=f.tuner.accepted_gains();
  for(unsigned i=0;i<20000 && f.tuner.stage()!=Stage::HOLD;++i) {
    f.benefit=f.tuner.evidence().pairs<1;
    f.tick();
  }
  assert(f.tuner.stage()==Stage::HOLD);
  assert(f.tuner.accepted_gains()==initial);
}

void reset_and_rollback_only_commit_on_save() {
  Fixture f;
  auto initial=f.tuner.accepted_gains();
  f.until(Stage::SAVE_PROMOTION);f.tuner.saved(true,f.now);
  auto accepted=f.tuner.accepted_gains();
  f.tuner.rollback();
  assert(f.tuner.accepted_gains()==accepted);
  assert(f.tuner.record().accepted==initial);
  f.tuner.saved(true,f.now);
  assert(f.tuner.accepted_gains()==initial);
  f.tuner.reset();f.tuner.saved(true,f.now);
  assert(f.tuner.accepted_gains()==initial);
}

void pid_math_and_restart() {
  AdaptivePID pid(20,.5,30,18,25.5);
  pid.enable_adaptation(false);
  pid.set_derivative_filter_seconds(30);
  pid.set_target(20,true);
  pid.update(19,0,true);
  pid.update(19,500,true);
  auto output=pid.preview(20,.5,30);
  assert(std::fabs(output-(20+3.75*std::tanh((20+.25)/100)))<1e-5);
  pid.update(19,600,false);
  pid.update(19.9f,3600000,true);
  assert(std::fabs(pid.preview(0,.5,30)-20)<1e-5);
  pid.set_gains(10,.5,30);
  assert(pid.kp()==10);
  pid.update(NAN,3601000,true);
  assert(pid.preview(0,.5,30)==20);
  pid.set_target(21,true);pid.update(19,3602000,true);
  assert(pid.preview(0,.5,30)==21);
  assert(pid.preview(0,0,0)==21);
  pid.update(19.3f,3603000,true);
  assert(pid.preview(0,0,30)<21);
  assert(pid.preview(0,0,30)>20.9f);
}

void clock_rollover_and_command_bound() {
  Fixture f;
  f.now=UINT32_MAX-500;
  for(int i=0;i<100;++i)f.tick();
  assert(f.tuner.evidence().attempts>0);
  auto s=f.frame();
  s.command_state.target=50;s.command_state.output_max=100;s.command_state.high=100;
  s.command_state.terms={{-1,0,0}};
  f.tuner.observe(s,f.now+=100);
  assert(f.tuner.gains()==f.tuner.accepted_gains());
  assert(std::strcmp(f.tuner.result(),"inconclusive_command_bound")==0);
}


void synthetic_closed_loop(bool heating, float lag=180, float loss=.0005f, float fireplace_rate=.001f) {
  auto config=Fixture::config();
  config.period_ms=600000; config.settling_ms=300000; config.hold_ms=3600000;
  config.observable_ms=20000;config.context_ms=300000;
  Learner learner({{20,.015f,0}},config,789,12345);
  AdaptivePID pid(20,.015f,0,18,25.5f);
  pid.enable_adaptation(false);
  pid.set_bounds(0,250,0,10,0,7200);
  pid.set_derivative_filter_seconds(30);
  double temperature=heating ? 19.5 : 24.5, heat_rate=0;
  bool power=true;
  float command=heating ? 20.f : 24.f, previous_target=-1;
  unsigned shutdowns=0, applied_trials=0, observable_trials=0;
  uint32_t now=1000;
  for(unsigned step=0;step<4*86400/2;++step,now+=2000) {
    const float hour=float(step*2%86400)/3600;
    const bool fire=hour>=18 && hour<20;
    const float outdoor=(heating ? 10.f : 32.f)+2*std::sin(step*2./86400*6.283185);
    const float target=(heating ? 20.f : 24.f)+(hour<6 ? (heating ? -.5f : .5f) : 0);
    // Ceiling exposure and a changing fusion weight are measurement context,
    // not additional delivered heat. The physical plant has its own heat lag.
    const float ceiling=temperature+(heating ? 1 : -1)*std::fabs(heat_rate)*40;
    const float weight=std::fabs(ceiling-temperature)>.5 ? 90.f : 75.f;
    const float fused=weight*.01f*temperature+(1-weight*.01f)*ceiling;
    const float over=(heating ? 1.f : -1.f)*(fused-target);
    if(power && over>.75f) { power=false; ++shutdowns; }
    if(!power && over<-.25f) power=true;
    if(target!=previous_target) { pid.set_target(target,heating); previous_target=target; }
    auto g=learner.gains();pid.set_gains(g[0],g[1],g[2]);
    pid.update(fused,now,power);
    const float low=std::max(18.f,target-(heating ? 2.f : .25f));
    const float high=std::min(25.5f,target+(heating ? .25f : 2.f));
    auto preview=[&](const Learner::Gains &v) {
      return std::round(std::max(low,std::min(high,pid.preview(v[0],v[1],v[2])))*2)/2;
    };
    Learner::Sample s;
    s.fireplace_context=s.fusion_context=s.others_context=true;
    s.valid=step%3000!=2999; s.enabled=true;s.heating=heating;
    s.temperature=fused;s.target=target;s.outdoor=outdoor;s.fusion_weight=weight;
    s.power=power;s.operating=power&&std::fabs(heat_rate)>.001;s.external_heat=fire;
    s.other_0=(hour>8 && hour<12);s.other_1=(hour>5&&hour<7);
    s.other_duty=(s.other_0+s.other_1)*.5f;
    s.command_state=pid.command_state(low,high);
    s.unit_setpoint=command;
    learner.observe(s,now);
    if(learner.testing_candidate() && std::fabs(preview(learner.accepted_gains())-preview(learner.gains()))>.501f)
      learner.abort("inconclusive_command_bound");
    g=learner.gains();pid.set_gains(g[0],g[1],g[2]);
    command=preview(g);
    if(learner.testing_candidate()) {
      ++applied_trials;
      float difference=std::fabs(command-preview(learner.accepted_gains()));
      assert(difference<=.501f);
      if(difference>.49f) ++observable_trials;
    }
    assert(command>=18 && command<=25.5);
    if(learner.needs_save()) {
      auto record=learner.record();
      Learner reboot(record.configured,config,789);
      assert(reboot.restore(record));
      learner.saved(true,now);
    }
    float remote=heating ? std::ceil(fused*2)/2 : std::floor(fused*2)/2;
    float demand=std::max(0.f,std::min(1.f,(heating?1.f:-1.f)*(command-remote)+.5f));
    double desired=(power ? (heating?1:-1)*.014*demand : 0);
    heat_rate+=(desired-heat_rate)*2/lag;
    temperature+=2*(heat_rate-loss*(temperature-outdoor)+(fire?fireplace_rate:0));
    assert(std::isfinite(temperature)&&temperature>10&&temperature<35);
  }
  assert(applied_trials>0);
  if (fireplace_rate>=.005f) assert(shutdowns>0);
  assert(learner.proposals()>1);
  if (!observable_trials) assert(learner.revision()==0);
  std::printf("Synthetic %s lag=%.0fs: %u proposals, %u trial samples, %u observable trial samples, %u shutdowns, %u verified revisions (not a field-performance claim)\n",
      heating?"heat":"cool",lag,learner.proposals(),applied_trials,observable_trials,shutdowns,learner.revision());
}

void exact_scoring_with_scheduler_jitter() {
  Fixture f;
  for (unsigned i=0;i<100 && !f.tuner.completed_periods();++i) {
    f.now+=170;
    f.tuner.observe(f.frame(),f.now);
  }
  assert(f.tuner.completed_periods()==1);
  assert(std::fabs(f.tuner.last_period().seconds-1.2)<1e-9);

  Fixture varied;
  const unsigned ticks[] = {137,181,97,203,53};
  unsigned completed = 0;
  for (unsigned i=0;i<200 && completed<4;++i) {
    varied.now+=ticks[i%5];
    varied.tuner.observe(varied.frame(),varied.now);
    if (varied.tuner.completed_periods()!=completed) {
      completed=varied.tuner.completed_periods();
      assert(varied.tuner.last_period().seconds==1.2);
    }
  }
  assert(completed==4);
}

void independent_heat_and_cool_profiles() {
  PidGainProfiles profiles({{20,2,30}},{{12,1,5}},Fixture::config(),123);
  auto *heat=profiles.select(PidGainProfiles::Mode::HEAT);
  assert(heat==&profiles.at(0));
  assert(heat->accepted_gains()[0]==20);
  auto *cool=profiles.select(PidGainProfiles::Mode::COOL);
  assert(cool==&profiles.at(1));
  assert(cool->accepted_gains()[0]==12);
  auto heat_record=heat->record();
  auto cool_record=cool->record();
  assert(heat_record.fingerprint!=cool_record.fingerprint);
  assert(!heat->restore(cool_record));
  assert(profiles.select(PidGainProfiles::Mode::OTHER)==nullptr);
  assert(profiles.selected_index()==-1);
  assert(profiles.select(PidGainProfiles::Mode::HEAT)==heat);
  assert(profiles.select(PidGainProfiles::Mode::HEAT)==heat);

  uint32_t now=1000;
  for (unsigned i=0;i<100 && !heat->testing_candidate();++i) {
    Learner::Sample sample;
    sample.valid=sample.enabled=sample.power=true;
    sample.temperature=20.2f;sample.target=20;sample.outdoor=10;
    sample.command_state=evidence_command_state();
    sample.unit_setpoint=sample.command_state.command(heat->gains());
    heat->observe(sample,now+=100);
  }
  assert(heat->testing_candidate());
  profiles.select(PidGainProfiles::Mode::COOL);
  assert(heat->gains()==heat->accepted_gains());
  assert(cool->gains()==cool->accepted_gains());
  profiles.select(PidGainProfiles::Mode::HEAT);
  assert(heat->gains()==heat->accepted_gains());
  for (unsigned i=0;i<20000 && heat->stage()!=Stage::SAVE_PROMOTION;++i) {
    Learner::Sample sample;
    const bool trial=heat->testing_candidate();
    sample.valid=sample.enabled=sample.power=sample.operating=true;
    sample.target=20;sample.temperature=trial?20.15f:20.4f;sample.outdoor=10;
    sample.command_state=evidence_command_state();
    sample.unit_setpoint=sample.command_state.command(heat->gains());
    heat->observe(sample,now+=100);
    if (heat->stage()==Stage::SAVE_BUDGET) heat->saved(true,now);
  }
  assert(heat->stage()==Stage::SAVE_PROMOTION);
  heat->saved(true,now);
  assert(heat->revision()==1);
  assert(heat->accepted_gains()[0]<20);
  profiles.select(PidGainProfiles::Mode::COOL);
  // Heating's promotion changes neither cooling's gains nor its evidence budget.
  assert(cool->record().confirmations==0);
  assert(cool->revision()==0);
  assert(cool->accepted_gains()[0]==12);
  assert(heat->restore(heat->record()));
  assert(!cool->restore(heat->record()));
  profiles.select(PidGainProfiles::Mode::HEAT);
  assert(heat->gains()==heat->accepted_gains());
}

void omitted_metadata_is_not_reported_as_measured_off() {
  Fixture f;
  auto s=f.frame();
  s.fireplace_context=s.fusion_context=s.others_context=false;
  for (unsigned i=0;i<100 && !f.tuner.completed_periods();++i) {
    s.unit_setpoint=s.command_state.command(f.tuner.gains());
    f.tuner.observe(s,f.now+=100);
  }
  assert(!f.tuner.last_period().fireplace_context);
  assert(!f.tuner.last_period().fusion_context);
  assert(!f.tuner.last_period().others_context);
}

Learner::Sample physical_sample(AdaptivePID &pid, const Learner &learner, float temperature,
                                float target, uint32_t now, bool heating=true, bool power=true) {
  pid.set_gains(learner.gains()[0],learner.gains()[1],learner.gains()[2]);
  if (pid.get_target()!=target || pid.heating()!=heating) pid.set_target(target,heating);
  pid.update(temperature,now,power);
  Learner::Sample sample;
  sample.valid=sample.enabled=true;sample.power=power;sample.heating=heating;
  sample.temperature=temperature;sample.target=target;sample.outdoor=10;
  const float low=std::max(18.f,target-(heating?2.f:.25f));
  const float high=std::min(25.5f,target+(heating?.25f:2.f));
  sample.command_state=pid.command_state(low,high);
  sample.unit_setpoint=sample.command_state.command(learner.gains());
  return sample;
}

void quantization_aware_candidates() {
  for (bool heating : {true,false}) {
    Learner learner({{1.25f,.015f,0}},Fixture::config(),42);
    AdaptivePID pid(1.25f,.015f,0,18,25.5);
    pid.enable_adaptation(false);
    const float target=heating?20.f:24.f;
    auto sample=physical_sample(pid,learner,target+(heating?-.5f:.5f),target,1000,heating);
    // This cooling case allows a lower native command; a 0.25 C raw lower
    // limit at a half-degree target cannot do so because ties round upward.
    if (!heating) sample.command_state.low=target-.5f;
    auto small=learner.accepted_gains();small[0]*=1.1f;
    assert(sample.command_state.command(small)==sample.command_state.command(learner.accepted_gains()));
    learner.observe(sample,1000);
    assert(learner.stage()==Stage::DISCOVERY);
    const auto candidate=learner.candidate_gains();
    assert(candidate[0]>10 && candidate[0]<50); // sized for a real command, not 10%.
    assert(candidate[1]==.015f && candidate[2]==0);
    assert(std::fabs(sample.command_state.command(candidate)-
                     sample.command_state.command(learner.accepted_gains()))==.5f);
    auto smaller=candidate;
    smaller[0]=1.25f+(candidate[0]-1.25f)*.9999f;
    assert(sample.command_state.command(smaller)==sample.command_state.command(learner.accepted_gains()));
    // A later state cannot resize a candidate under test.
    sample.command_state.terms[0]*=.9f;
    learner.observe(sample,1100);
    assert(learner.candidate_gains()==candidate);
  }
  // A downward experiment is sized just across the other rounding boundary.
  Learner down({{30,0,0}},Fixture::config(),43);
  AdaptivePID pid(30,0,0,18,25.5);pid.enable_adaptation(false);
  auto sample=physical_sample(pid,down,19.5,20,1000);
  down.observe(sample,1000);
  assert(down.candidate_gains()[0]<30);
  assert(std::fabs(sample.command_state.command(down.candidate_gains())-
                   sample.command_state.command(down.accepted_gains()))==.5f);
}

void defer_uninformative_and_disabled_conditions() {
  Learner learner({{1.25f,.015f,0}},Fixture::config(),42);
  AdaptivePID pid(1.25f,.015f,0,18,25.5);pid.enable_adaptation(false);
  auto sample=physical_sample(pid,learner,19.5,20,1000);
  sample.power=false;
  learner.observe(sample,1000);
  assert(learner.proposals()==0 && learner.period_number()==0);
  assert(std::strcmp(learner.result(),"waiting_for_enabled_unit")==0);
  sample.power=true;sample.command_state.terms={{0,0,0}};
  learner.observe(sample,100000);
  assert(learner.proposals()==0 && learner.period_number()==0);
  assert(std::strcmp(learner.result(),"waiting_for_informative_command")==0);
  // A nearly flat signal must not cause escalation of D to force a command.
  sample.command_state.terms={{0,0,.00001f}};
  learner.observe(sample,100100);
  assert(learner.proposals()==0 && learner.candidate_gains()[2]==0);
  // Neither raw clipping nor tight gain bounds may be overridden for excitation.
  sample.command_state.terms={{.5f,1,.01f}};
  sample.command_state.low=sample.command_state.high=20;
  learner.observe(sample,100200);
  assert(learner.proposals()==0);
  sample.command_state.low=18;sample.command_state.high=20.25f;
  sample.command_state.terms={{.5f,0,0}};
  learner.observe(sample,100300);
  assert(learner.proposals()==1);
  auto config=Fixture::config();config.maximum={{2,.02f,1}};
  Learner bounded({{1.25f,.015f,0}},config,42);
  bounded.observe(sample,100300);
  assert(bounded.proposals()==0);
  sample.command_state.terms[0]=NAN;
  learner.observe(sample,100400);
  assert(learner.gains()==learner.accepted_gains());
}

void informative_integral_and_derivative() {
  for (unsigned coordinate : {1u,2u}) {
    Learner learner({{1.25f,.015f,0}},Fixture::config(),42);
    auto sample=Fixture().frame();
    sample.command_state.terms={{0,0,0}};
    sample.command_state.terms[coordinate]=coordinate==1?1.f:-.01f;
    learner.observe(sample,1000);
    assert(learner.proposals()==1);
    auto a=learner.accepted_gains(), b=learner.candidate_gains();
    assert(b[coordinate]>a[coordinate]);
    for (unsigned i=0;i<3;++i) if (i!=coordinate) assert(a[i]==b[i]);
    assert(std::fabs(sample.command_state.command(a)-sample.command_state.command(b))==.5f);
  }
}

void periods_include_shutdown_and_coasting() {
  Fixture f;
  f.tick();
  const auto candidate=f.tuner.candidate_gains();
  f.tick();f.tick();f.tick(); // beyond settling
  auto sample=f.frame();
  sample.power=false;sample.temperature=20.8f;
  sample.command_state.terms={{0,0,0}}; // actual PID resets while off
  const unsigned periods=f.tuner.completed_periods();
  for (unsigned i=0;i<20 && f.tuner.completed_periods()==periods;++i)
    f.tuner.observe(sample,f.now+=100);
  assert(f.tuner.completed_periods()==periods+1);
  assert(f.tuner.last_period().seconds==1.2);
  assert(f.tuner.last_period().shutdowns==1);
  assert(f.tuner.candidate_gains()==candidate);
  f.tuner.observe(sample,f.now+=100);
  assert(!f.tuner.testing_candidate());
  assert(f.tuner.gains()==f.tuner.accepted_gains());
  const auto number=f.tuner.period_number();
  for (unsigned i=0;i<3;++i) f.tuner.observe(sample,f.now+=100);
  assert(f.tuner.period_number()==number);
  assert(f.tuner.completed_periods()==periods+1);
  assert(std::strcmp(f.tuner.result(),"waiting_for_enabled_unit")==0);
  // Resume the same fixed candidate only after both commands separate again.
  sample=f.frame();sample.power=true;
  f.tuner.observe(sample,f.now+=100);
  assert(f.tuner.candidate_gains()==candidate);
  assert(f.tuner.completed_periods()==periods+1);
}

void bounded_wait_and_actual_readback() {
  Fixture f;
  auto sample=f.frame();
  for (unsigned i=0;i<100 && f.tuner.stage()!=Stage::HOLD;++i) {
    sample=f.frame();
    sample.unit_setpoint=-100; // neither arm's command was actually read back
    f.tuner.observe(sample,f.now+=100);
  }
  assert(f.tuner.stage()==Stage::HOLD && f.tuner.revision()==0);
  assert(std::strcmp(f.tuner.result(),"inconclusive_unobservable_command")==0);
  Fixture waiting;
  waiting.tick();
  for (unsigned i=0;i<30 && waiting.tuner.completed_periods()==0;++i) waiting.tick();
  sample=waiting.frame();sample.power=false;
  for (unsigned i=0;i<30;++i) waiting.tuner.observe(sample,waiting.now+=100);
  assert(waiting.tuner.stage()==Stage::DISCOVERY);
  assert(waiting.tuner.pending_observations()==1);
  assert(waiting.tuner.revision()==0);
  assert(std::strcmp(waiting.tuner.result(),"waiting_for_enabled_unit")==0);

}

void command_snapshot_matches_existing_mapping() {
  AdaptivePID pid(1.25f,.015f,0,18,25.5);
  pid.enable_adaptation(false);pid.set_derivative_filter_seconds(30);
  pid.set_target(20,true);pid.update(19,1000,true);pid.update(19.3f,3000,true);
  const auto state=pid.command_state(18,20.25f);
  for (float p : {0.f,1.25f,20.f,250.f})
    for (float i : {0.f,.015f,10.f})
      for (float d : {0.f,30.f,7200.f}) {
        const float output=p*state.terms[0]+i*state.terms[1]+d*state.terms[2];
        const float expected=std::max(18.f,std::min(25.5f,20+3.75f*std::tanh(output/100)));
        assert(std::fabs(pid.preview(p,i,d)-expected)<1e-6);
        assert(state.command({{p,i,d}})==std::round(std::max(18.f,std::min(20.25f,expected))*2)/2);
      }
  // At a half-degree target, a cooling lower limit of -0.25 C rounds back
  // to that same target; the learner must defer rather than cross the limit.
  Learner limited({{1.25f,.015f,0}},Fixture::config(),42);
  auto sample=physical_sample(pid,limited,24.5,24,5000,false);
  limited.observe(sample,5000);
  assert(limited.proposals()==0);
  assert(limited.gains()==limited.accepted_gains());
}

template <typename Change>
void complete_observation(Fixture &f, Change change) {
  const unsigned before=f.tuner.completed_periods();
  for (unsigned frame=0;frame<100 && f.tuner.completed_periods()==before;++frame) {
    auto sample=f.frame();change(sample,frame);
    sample.unit_setpoint=sample.command_state.command(f.tuner.gains());
    f.tuner.observe(sample,f.now+=100);
  }
  assert(f.tuner.completed_periods()==before+1);
}
void complete_observation(Fixture &f) {
  complete_observation(f,[](Learner::Sample &,unsigned){});
}

void initial_response_is_scored() {
  Fixture f;
  complete_observation(f,[&](Learner::Sample &s,unsigned frame){
    // The old unscored interval contained all distinct command exposure and a
    // hysteresis shutdown. Both must now count in the full response.
    s.temperature=frame<2?20.4f:20.8f;
    if (frame>=2) { s.power=false;s.command_state.terms={{0,0,0}}; }
  });
  const auto &out=f.tuner.last_period();
  assert(out.seconds==1.2);
  assert(out.observable_seconds>=.1 && out.observable_seconds<.3);
  assert(out.shutdowns==1);
  assert(out.over>.8); // includes the entire early response and later coasting
}

void retain_until_fireplace_context_returns() {
  Fixture f;
  complete_observation(f);
  const auto fixed=f.tuner.candidate_gains();
  const auto count=f.tuner.completed_periods();
  assert(f.tuner.pending_observations()==1);
  for (unsigned i=0;i<30;++i) {
    auto sample=f.frame();sample.external_heat=true;
    f.tuner.observe(sample,f.now+=100);
  }
  assert(f.tuner.completed_periods()==count);
  assert(f.tuner.pending_observations()==1);
  assert(f.tuner.candidate_gains()==fixed);
  assert(f.tuner.gains()==f.tuner.accepted_gains());
  assert(std::strcmp(f.tuner.result(),"waiting_for_matching_context")==0);
  // An outage discards only the partial measurement, never the complete cache.
  auto invalid=f.frame();invalid.valid=false;f.tuner.observe(invalid,f.now+=100);
  assert(f.tuner.pending_observations()==1);
  for (unsigned i=0;i<4 && f.tuner.evidence().pairs==0;++i) complete_observation(f);
  assert(f.tuner.evidence().pairs==1);
  assert(f.tuner.candidate_gains()==fixed);
  assert(f.tuner.pending_observations()<=1);
  // The cached observation cannot be reused for a second comparison.
  assert(f.tuner.evidence().pairs*2<=f.tuner.completed_periods());
}

void mismatched_error_target_and_fireplace_transition() {
  Fixture f;complete_observation(f);
  auto sample=f.frame();sample.temperature=19.6f; // opposite sign, same absolute error
  f.tuner.observe(sample,f.now+=100);
  assert(std::strcmp(f.tuner.result(),"waiting_for_matching_context")==0);
  assert(f.tuner.pending_observations()==1);
  sample=f.frame();sample.target=20.5f;sample.temperature=20.9f;sample.command_state.target=20.5f;
  f.tuner.observe(sample,f.now+=100);
  assert(std::strcmp(f.tuner.result(),"waiting_for_matching_context")==0);
  assert(f.tuner.evidence().pairs==0);
  Fixture transition;
  complete_observation(transition,[](Learner::Sample &s,unsigned frame){s.external_heat=frame>=3 && frame<5;});
  assert(transition.tuner.last_period().fireplace_changed);
  assert(transition.tuner.pending_observations()==0);
  assert(transition.tuner.evidence().pairs==0);
  assert(std::strcmp(transition.tuner.result(),"inconclusive_fireplace_transition")==0);
}

void bounded_retention_and_context_only_pairing() {
  Fixture full;full.mismatch=true;
  for (unsigned i=0;i<500 && full.tuner.completed_periods()<12;++i) {
    full.tick();assert(full.tuner.pending_observations()<=8);
  }
  assert(full.tuner.pending_observations()==8);
  assert(full.tuner.evidence().pairs==0);
  assert(full.tuner.revision()==0);
  // Expiration is bounded even across the millis rollover.
  auto config=Fixture::config();config.retention_ms=2000;
  Learner expiring({{20,2,30}},config,42);
  uint32_t now=UINT32_MAX-1700;
  auto sample=Fixture().frame();
  for (unsigned i=0;i<30 && !expiring.completed_periods();++i) {
    sample.unit_setpoint=sample.command_state.command(expiring.gains());
    expiring.observe(sample,now+=100);
  }
  assert(expiring.pending_observations()==1);
  sample.external_heat=true;
  for (unsigned i=0;i<30 && expiring.stage()!=Stage::HOLD;++i) expiring.observe(sample,now+=100);
  assert(expiring.pending_observations()==0);
  assert(expiring.stage()==Stage::HOLD);
  assert(expiring.revision()==0);
  full.tuner.reset();assert(full.tuner.pending_observations()==0);
}

void recent_trend_matches_thermal_carryover() {
  auto config=Fixture::config();config.context_ms=1000;
  Learner learner({{20,2,30}},config,42);
  uint32_t now=1000;float temperature=20.2f;
  auto tick=[&](bool power,float rate,bool fireplace=false) {
    temperature+=rate/600.f; // rate in C/min, 100 ms samples
    auto sample=Fixture().frame();
    sample.temperature=temperature;sample.power=power;sample.external_heat=fireplace;
    sample.command_state.terms[2]=0; // trend must not depend on PID derivative history
    sample.unit_setpoint=sample.command_state.command(learner.gains());
    learner.observe(sample,now+=100);
  };
  // Synthetic residual warming: build history even with the unit off.
  for (unsigned i=0;i<30;++i) tick(false,.2f);
  for (unsigned i=0;i<30 && !learner.completed_periods();++i) tick(true,.2f);
  assert(learner.completed_periods()==1);
  assert(learner.last_period().start_trend>.15f);
  assert(learner.pending_observations()==1);
  const auto fixed=learner.candidate_gains();
  for (unsigned i=0;i<60;++i) tick(false,-.2f);
  tick(true,-.2f);
  assert(std::strcmp(learner.result(),"waiting_for_matching_context")==0);
  assert(learner.completed_periods()==1 && learner.pending_observations()==1);
  // Similar temperature error is insufficient while momentum is opposite.
  for (unsigned i=0;i<60;++i) tick(false,.2f);
  for (unsigned i=0;i<30 && learner.completed_periods()==1;++i) tick(true,.2f);
  assert(learner.completed_periods()==2 && learner.evidence().pairs==1);
  assert(learner.candidate_gains()==fixed);
}

void scheduled_targets_are_scored_and_matched() {
  Fixture f;
  auto schedule=[](Learner::Sample &s,unsigned frame) {
    if (frame>=5) { s.target=20.5f;s.command_state.target=20.5f;s.temperature+=.5f; }
  };
  complete_observation(f,schedule);
  assert(f.tuner.last_period().target_movement==.5f);
  assert(f.tuner.last_period().seconds==1.2);
  complete_observation(f,schedule);
  assert(f.tuner.evidence().pairs==1);
  Fixture mismatch;
  complete_observation(mismatch,schedule);
  complete_observation(mismatch); // identical start, different target trajectory
  assert(mismatch.tuner.evidence().pairs==0);
  assert(mismatch.tuner.pending_observations()==2);
}

void discovery_cache_is_not_confirmation_evidence() {
  Fixture f;f.save_budget=false;
  // A complete but unmatched target trajectory stays queued through discovery.
  complete_observation(f,[](Learner::Sample &s,unsigned frame){
    if (frame>=5) {s.target+=.5f;s.temperature+=.5f;s.command_state.target+=.5f;}
  });
  f.until(Stage::SAVE_BUDGET);
  assert(f.tuner.pending_observations()>0);
  f.tuner.saved(true,f.now);
  assert(f.tuner.stage()==Stage::CONFIRMATION);
  assert(f.tuner.pending_observations()==0);
  assert(f.tuner.evidence().pairs==0);
  assert(f.tuner.record().confirmations==1);
}

void fireplace_residual_context_requires_similar_history() {
  auto config=Fixture::config();config.context_ms=1000;
  Learner learner({{20,2,30}},config,42);
  uint32_t now=1000;
  auto tick=[&](bool power,bool fireplace) {
    auto sample=Fixture().frame();sample.power=power;sample.external_heat=fireplace;
    sample.temperature=20.2f;sample.command_state.terms[2]=0;
    sample.unit_setpoint=sample.command_state.command(learner.gains());
    learner.observe(sample,now+=100);
  };
  for (unsigned i=0;i<100;++i) tick(false,false);
  for (unsigned i=0;i<30 && !learner.completed_periods();++i) tick(true,false);
  assert(learner.pending_observations()==1);
  // Fireplace on then off. Identical temperature and current off state cannot
  // hide a recently observed transition from the matching rules.
  for (unsigned i=0;i<20;++i) tick(false,true);
  for (unsigned i=0;i<15;++i) tick(false,false);
  tick(true,false);
  assert(learner.completed_periods()==1);
  assert(std::strcmp(learner.result(),"waiting_for_matching_context")==0);
  for (unsigned i=0;i<60;++i) tick(false,false);
  for (unsigned i=0;i<30 && learner.completed_periods()==1;++i) tick(true,false);
  assert(learner.completed_periods()==2);
  assert(learner.evidence().pairs==1);
}

void oldest_context_match_does_not_select_the_best_score() {
  Fixture f;f.benefit=false;
  complete_observation(f);
  const bool side=f.tuner.last_period_candidate();
  const double first=f.tuner.last_period().over;
  // Retain a second, better-scoring observation of the same arm. Opposite-arm
  // placeholders deliberately have a fireplace transition and cannot pair.
  double second=0;
  for (unsigned i=0;i<4 && f.tuner.pending_observations()<2;++i) {
    complete_observation(f,[&](Learner::Sample &s,unsigned frame){
      if (frame && f.tuner.testing_candidate()!=side) s.external_heat=frame>=3;
      else if (frame) s.temperature=20.1f;
    });
    if (f.tuner.last_period_candidate()==side) second=f.tuner.last_period().over;
  }
  assert(second>0 && second<first);
  assert(f.tuner.pending_observations()==2);
  auto counterpart=[&](Learner::Sample &s,unsigned frame) {
    if (frame && f.tuner.testing_candidate()==side) s.external_heat=frame>=3;
  };
  for (unsigned i=0;i<4 && f.tuner.evidence().pairs<1;++i) complete_observation(f,counterpart);
  assert(f.tuner.evidence().pairs==1);
  const auto &e=f.tuner.evidence();
  assert((side?e.candidate.over:e.reference.over)==first);
  assert(f.tuner.pending_observations()==1);
  for (unsigned i=0;i<4 && f.tuner.evidence().pairs<2;++i) complete_observation(f,counterpart);
  assert(f.tuner.evidence().pairs==2);
  assert((side?e.candidate.over:e.reference.over)==first+second);
  assert(f.tuner.pending_observations()==0);
}

int main() {
  oldest_context_match_does_not_select_the_best_score();
  discovery_cache_is_not_confirmation_evidence();
  fireplace_residual_context_requires_similar_history();
  initial_response_is_scored();
  retain_until_fireplace_context_returns();
  mismatched_error_target_and_fireplace_transition();
  bounded_retention_and_context_only_pairing();
  recent_trend_matches_thermal_carryover();
  scheduled_targets_are_scored_and_matched();
  command_snapshot_matches_existing_mapping();
  quantization_aware_candidates();
  defer_uninformative_and_disabled_conditions();
  informative_integral_and_derivative();
  periods_include_shutdown_and_coasting();
  bounded_wait_and_actual_readback();
  independent_heat_and_cool_profiles();
  omitted_metadata_is_not_reported_as_measured_off();
  exact_scoring_with_scheduler_jitter();
  assert(Learner::sign_probability(0,0)==1);
  assert(std::fabs(Learner::sign_probability(10,2)-.019287109375)<1e-12);
  confirmation_and_persistence();
  confirmation_can_fail_without_promoting();
  guards_and_context();
  missing_data_and_gaps_restore_accepted();
  saved_budget_cannot_be_reused_after_reboot();
  gains_search_both_directions_and_wider();
  confirmation_rejects_single_lucky_period();
  reset_and_rollback_only_commit_on_save();
  pid_math_and_restart();
  clock_rollover_and_command_bound();
  synthetic_closed_loop(true);
  synthetic_closed_loop(false);
  synthetic_closed_loop(true,900,.0003f,.005f);
  std::puts("PID gain learning regression tests passed");
}
