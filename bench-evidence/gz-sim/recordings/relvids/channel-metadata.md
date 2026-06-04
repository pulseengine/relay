# Falcon release videos — channel metadata

## v1.16
**Title:** Falcon v1.16 — Wind Rejection | a formally-verified drone flight stack

The verified falcon flight core now rejects a steady wind. A falsification-driven fix: the P-D position loop left a >1.5 m offset under wind, so we added an anti-windup integral that cuts it to <0.6 m. Verified in Gazebo SITL with the Lean Lyapunov proof and the Kani-verified FSM and mixer still holding. IEKF to geometric SE(3) to ADRC to mixer, all no_std / no_alloc.

#drone #autopilot #formalverification #rust #robotics

---

## v1.17
**Title:** Falcon v1.17 — Aerodynamic Drag | verified drone flight stack

Quadratic v-squared aerodynamic drag, the force that matters during fast flight. The verified cascade tracks a far [4,0,-2] m setpoint to within 0.6 m through the drag, and the drag damps the vehicle. Verified in SITL.

#drone #autopilot #formalverification #rust #robotics

---

## v1.18
**Title:** Falcon v1.18 — IMU Bias-Instability | verified drone flight stack

A realistic IMU bias-instability: a stochastic random-walk gyro bias, a moving target. The verified Invariant-EKF re-tracks it continuously, holding tilt under 0.15 rad as the bias wanders to about 0.06 rad/s. Verified in SITL.

#drone #autopilot #estimation #formalverification #rust

---

## v1.19
**Title:** Falcon v1.19 — Noisy + Intermittent GNSS | verified drone flight stack

Realistic GNSS: metre-class noise plus recurring outages. A falsification: an over-trusting measurement variance made the filter diverge ~2400 m. The fix is honest filter tuning, matching the variance to the sensor, and it holds. Verified in SITL.

#drone #autopilot #gps #formalverification #rust

---

## v1.20
**Title:** Falcon v1.20 — Barometer Fusion | verified drone flight stack

An independent vertical source so altitude survives a GPS-vertical outage. The barometer is fed into the verified IEKF as a vertical anchor; through a 40 s GPS loss it holds altitude to ~1.3 m, beating dead-reckoning. Verified in SITL.

#drone #autopilot #sensorfusion #formalverification #rust

---

## v1.21
**Title:** Falcon v1.21 — Battery Drain Failsafe | verified drone flight stack

The low-battery failsafe now fires on a real depleting pack, not a set value: the charge drains with motor load and the terminal voltage sags, and the supervisor actuates RTL/Land on a genuine endurance limit. Verified in SITL.

#drone #autopilot #safety #formalverification #rust

---

## v1.22
**Title:** Falcon v1.22 — Air-Density Thrust Lapse | verified drone flight stack

Thrust falls with altitude. The vertical twin of the wind finding: the P-D altitude loop sags, so an opt-in altitude integral holds it, and beyond the margin there is an honest service ceiling. Verified in SITL.

#drone #autopilot #control #formalverification #rust

---

## v1.23
**Title:** Falcon v1.23 — Motor Dynamics | verified drone flight stack

First-order motor spin-up lag, the actuator lag the ADRC extended-state observer was built to absorb. With a 40 ms motor time constant the inner loop still recovers a tilted body to level. A robustness confirmation. Verified in SITL.

#drone #autopilot #control #formalverification #rust

---

## v1.24
**Title:** Falcon v1.24 — Ground Effect | verified drone flight stack

Rotor downwash off the surface augments thrust near the ground. It aids takeoff; on landing it cushions the vehicle into a documented float that a velocity-based touchdown controller will solve, an honest limitation, not a faked touchdown. Verified in SITL.

#drone #autopilot #aerodynamics #formalverification #rust

---

## v1.25
**Title:** Falcon v1.25 — Turbulence | verified drone flight stack

A Dryden-like colored gust spectrum, harder than white noise because the gusts persist. Under 2 m/s RMS turbulence the vehicle stays bounded within a few metres and does not diverge. Honest scope: bounded, not crisp. Verified in SITL.

#drone #autopilot #turbulence #formalverification #rust

---

## v1.26
**Title:** Falcon v1.26 — WASM Component Model, runs in wasmtime | verified drone flight stack

The keystone of the toolchain-to-hardware path. The verified IEKF to geometric to ADRC to mixer cascade builds as a WebAssembly Component Model component and runs in wasmtime, recovering a tilted body and flying to a setpoint. This is the portable hand-off artifact for the meld to loom to synth to gale integration onto silicon.

#webassembly #wasm #drone #autopilot #formalverification #rust

---

