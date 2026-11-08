# Bidirectional tuner learning requirements

Status: agreed requirements. An experimental implementation and its explicit defaults are documented in [gain learning](gain-learning.md); it has not been deployed or verified on the real HVAC system. The [control vision](control-vision.md) remains authoritative. This document records implementation requirements separately from that durable vision.

## Learning and exploration

- Learn gain changes in both directions, including below the configured starting gains. Gradual return toward configured gains is not a substitute for learning that a different value performs better.
- Measure forward during ordinary operation. Keep evidence separated by firmware version and controller configuration.
- Test nearby gain alternatives and occasionally challenge the current best with a wider, bounded alternative, including coordinated gain changes where justified.
- Retain existing hysteresis and command limits throughout trials. Initially run gain trials on one room while observing all three, because the heads share an outdoor unit.
- Describe the result as the best verified gains under observed conditions. Neither failure of nearby candidates nor a completed search establishes a global optimum.

## Confirmation before promotion

1. Discovery identifies a promising candidate B relative to the accepted gains A. B remains provisional.
2. Freeze B's gains for confirmation. Do not keep adapting a candidate while claiming to confirm it.
3. Compare A and B using new operation that was not used to select B. Alternate them across subsequent operating periods, randomizing order where practical and repeating comparisons across comparable conditions.
4. Account for thermal carryover after switching gains. Record weather, fireplace activity, fusion changes, other heads' operation, target changes, and operating mode when interpreting comparisons.
5. Evaluate shutdown frequency, overshoot magnitude and duration, temperature regulation, and recovery separately. A favorable combined score must not conceal unacceptable recovery or regulation.
6. Promote B only when its benefit repeats and the guardrails are satisfied. Retain A otherwise. Inconclusive evidence is not a rejection and must not be recorded as proof that no improvement exists.

New confirmation data do not automatically imply statistical independence. Repeated operating periods or cycles, rather than individual correlated temperature samples, are the units of evidence. A replay of recorded temperatures cannot confirm how the house would respond to gains that were never applied.

Zero shutdowns with both settings do not establish improvement. An acceptance rule must explicitly address rare shutdowns and evidence from other outcomes rather than treating the absence of observed events as proof of equal shutdown risk.

## Persistence and visibility

- Distinguish YAML-configured starting gains, temporary trial gains, and verified accepted gains.
- Automatically persist each verified improvement and restore verified gains after restart. Do not persist a trial as accepted merely because it was active at shutdown or restart.
- Retain the previous accepted set for rollback.
- Expose verified Kp, Ki, and Kd for copying into ESPHome YAML, with a compact status indicating a saved improvement and a revision identifier linking it to its evidence. Verified gains can be copied while exploration continues.
- Keep a record of tested gains, observed conditions, outcomes, and uncertainty. Failed persistence must not be represented as a successful durable save.

## Implementation choices and qualification

The experimental implementation specifies candidate generation, gain bounds,
comparison timing, guards, fixed-look confirmation, alpha spending, early rollback,
missing-context behavior, persistence compatibility, YAML precedence, and reset
behavior in [gain learning](gain-learning.md). These engineering defaults are
separate from the durable control vision and require field qualification.

Do not silently choose acceptance thresholds to make trials pass or advertise
bench tests as evidence of improved behavior on the real system.

## Research basis

[NIST's randomized block design guidance](https://www.itl.nist.gov/div898/handbook/pri/section3/pri332.htm) supports grouping comparable conditions and randomizing treatment order to reduce the influence of nuisance factors. Applying those principles here still requires explicit treatment of thermal carryover and uncontrolled conditions.

[Lequin et al., Iterative Feedback Tuning of PID parameters](https://perso.uclouvain.be/michel.gevers/PublisMig/P70.pdf) discusses local minima in iterative PID tuning. It motivates distinguishing a locally successful result from global optimality; this document does not claim that the proposed tuner implements that paper's algorithm.
