# PID controller and tuner vision

The outer PID adjusts HVAC demand so the unit can remain enabled while its onboard controller regulates room temperature. Maintain room temperature close to target while minimizing overshoot-driven hysteresis shutdowns and preserving effective recovery. Evaluate deviations by both their magnitude and duration; perfectly constant temperature is not required.

## Priorities

1. Respect hysteresis limits. Crossing the configured shutdown bounds must cause shutdown; keeping the unit enabled never overrides this requirement.
2. Minimize overshoot that leads to hysteresis shutdown. This is the primary tuning concern.
3. Preserve sufficient demand for effective recovery when room temperature is substantially away from target. Insufficient demand during recovery is the secondary tuning concern, in either heating or cooling.

Remaining enabled alone does not establish successful control. Sustained temperature error or ineffective recovery still warrants improvement. An enabled unit may reduce output or cycle its compressor under its onboard controller; continuous compressor operation is not an objective.

No numerical temperature tolerance or duration threshold is implied by this vision.

## Interpreting tuning evidence

The component adjusts the unit's setpoint to influence HVAC demand. Calculated setpoint corrections, actual setpoint readback, enabled state, and operating flags describe commands or state; they are not measurements of delivered heating or cooling power. Interpret changes in those signals with their limits and quantization in mind.

The reference installation has three indoor units sharing one outdoor unit: Primary Bedroom, Hailee's Bedroom, and Main Floor. Account for these conditions when interpreting behavior:

- **Temperature fusion:** The bedrooms control against a fusion of wall and ceiling temperatures. Changing fusion weights can move the reported temperature without equivalent movement in the raw temperatures. Retain the fused measurement as the control reference and inspect raw readings and weights when attributing changes to HVAC response.
- **Fireplace heat:** The Main Floor fireplace can warm the room independently of HVAC activity, including residual warming after shutdown. Distinguish this external heat from HVAC response while still evaluating how well the controller reduces demand. A fireplace-driven crossing of hysteresis bounds still requires shutdown.
- **Outdoor temperature:** The outdoor aggregate averages three inputs: Pirate Weather, the HVAC outdoor reading, and a combined reading from four Blink cameras. The aggregate and HVAC reading are correlated because the latter contributes to the former; they are not independent confirmation of outdoor conditions.
- **Shared outdoor unit:** Other indoor units' operation may affect the response available to a room. Consider their operating states when comparing tuning evidence.

## Using this vision

Evaluate proposed changes against these objectives, preserving successful existing behavior and identifying the expected benefit and tradeoffs. Use measured evidence to distinguish changes in HVAC response from external heat or changes in measurement.

Keep experimental gains, temporary findings, hypotheses, and implementation proposals separate from this durable vision. Label uncertain explanations as hypotheses rather than turning them into requirements. Changes to the agreed objectives should be made explicit rather than inferred from a particular experiment or scoring method.
