//! The VERIFIED falcon flight core running bare-metal on an ARM Cortex-M
//! (STM32H743) — with the SIMULATION as the backend. This is the
//! "run on a board with the sim as the backend first" milestone, on an
//! emulated target: the exact same `no_std` `FlightCore` + `FlightBackend`
//! that flies in the SITL bench, here driving an emulated MCU. Swap the
//! `SimBackend` for a real-hardware `FlightBackend` (v1.11 seam) and the same
//! ELF flies a real drone.
//!
//! Output is semihosting (`hprintln!`) → the Renode/QEMU console. Run with
//! `renode/falcon-cortex-m.resc` (Renode is a bench tool, not in CI).

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};
use panic_halt as _;

use falcon_core::{FlightCore, SimBackend};

#[entry]
fn main() -> ! {
    hprintln!("falcon-cortex-m: verified flight core on STM32H743, sim backend");

    let dt = 0.002_f32;
    // start tilted ~23° (cos/sin of 0.4 rad, baked in — no host float deps)
    let (c, s) = (0.921_060_99_f32, 0.389_418_36_f32);
    let r0 = [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]];

    let mut backend = SimBackend::new(r0, dt);
    let mut core = FlightCore::new(0.5, 1.0 / dt);
    core.set_position([2.0, -1.5, -2.0]); // fly 2 m N, 1.5 m W, 2 m up

    // run the verified cascade on the MCU
    for _ in 0..30000 {
        core.step(&mut backend);
    }

    let p = backend.pos;
    let tilt = backend.tilt();
    hprintln!(
        "result: pos = [{}, {}, {}]  tilt = {} rad",
        p[0], p[1], p[2], tilt
    );

    // PASS/FAIL the on-target run via the semihosting exit code, so a harness
    // can assert it: reached the setpoint AND settled near level.
    let err = (p[0] - 2.0) * (p[0] - 2.0) + (p[1] + 1.5) * (p[1] + 1.5) + (p[2] + 2.0) * (p[2] + 2.0);
    if err < 0.25 && tilt < 0.15 {
        hprintln!("ON-TARGET PASS — verified core flew to setpoint on the MCU");
        debug::exit(debug::EXIT_SUCCESS);
    } else {
        hprintln!("ON-TARGET FAIL");
        debug::exit(debug::EXIT_FAILURE);
    }
    loop {}
}
