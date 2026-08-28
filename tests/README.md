# Test fixtures

The files in `fixtures/` exercise the supported integration shapes against ESPHome 2026.6.0:

- `standalone_cn105.yaml`: raw CN105 without a wrapper.
- `composed_cn105.yaml`: local sensor shared by CN105 and PID/hysteresis.
- `non_cn105_single_target.yaml`: generic demo climate, including pass-through modes and auxiliary traits.
- `pid_only.yaml` and `hysteresis_only.yaml`: each workflow independently.
- `two_point_rejected.yaml`: a two-point inner climate. Running the host binary must log `Two-point target climates are not supported` and mark only the wrapper failed.

Validate every schema with:

```bash
for fixture in tests/fixtures/*.yaml; do
  uv run --with esphome==2026.6.0 esphome config "$fixture"
done
```

The composed ESP-IDF build and generic host build provide compile-time coverage for the raw protocol path and climate-only wrapper path respectively. State-machine invariants are kept in `WorkflowClimate`: inner targets are never copied to the outer target, HEAT/COOL are the only workflow modes, pending fields suppress inner echo, and the compatibility guard precedes all inner calls.
