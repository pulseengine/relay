# Falcon GroundEffect — custom Gazebo realism plugin

A Gazebo Harmonic (`gz-sim8`) **System plugin** that applies a near-surface
thrust-augmentation *cushion* force, bringing the SimBackend `ground_effect`
model (`thrust ×= 1 + gain·e^(−alt/decay)`) into the gz SITL.

**Why custom?** The stock `gz-sim-wind-effects-system` over-authors disturbances
and diverges the falcon mission; the realism arc's effects (wind, drag,
ground-effect, …) live in the verified Rust `SimBackend`. This plugin is the
start of bringing the **same, controlled** realism models into gz so the two
agree — the first of the v1.32 gz-realism thread.

The force on the configured link is `F_z = gain · e^(−alt/decay) · reference_force`,
strongest at the surface and decaying with altitude — exactly the cushion that
floats a landing (the v1.24 limitation, solved in the cascade by v1.27's
velocity-landing).

## Build

```bash
# qt@5 is keg-only on Homebrew; gz-sim8's cmake config needs it on the path.
cmake -B build -DCMAKE_PREFIX_PATH=/opt/homebrew/opt/qt@5
cmake --build build
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build
```

## SDF usage (a model plugin)

```xml
<plugin filename="FalconGroundEffect" name="falcon::GroundEffect">
  <link_name>base_link</link_name>
  <gain>0.4</gain>
  <decay>0.25</decay>            <!-- e-folding height, m -->
  <reference_force>20.0</reference_force>  <!-- N, ≈ vehicle weight -->
  <verbose>false</verbose>
</plugin>
```

The realism world generator emits it as the `groundeffect` layer:
`worlds/gen-realism-world.py groundeffect out.sdf`.

## Verify (bench-only — gz is not in CI)

```bash
./run-ground-effect-test.sh
```

Runs an A/B headless: a light free box just above the surface is **held aloft**
by the cushion (`gain=0.4` → settles ~0.10 m) but **falls through** without it
(`gain=0` → −78 m in 4 s). PASS proves the force is real and altitude-decaying.
