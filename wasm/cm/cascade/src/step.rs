//! Sync coverage sibling of the P3 stream cascade (#202).
//!
//! Exports a call-return `step` so the witness MC/DC harness can drive the SAME
//! verified engine branches the async-lift `monitor` stream export hides. Shares
//! the cascade wiring via `cascade_orch` — no second copy of the pipeline. Runs
//! identical engine code to the stream component; only the outer ABI differs
//! (sync call-return vs async stream), which is exactly the difference that
//! makes this one harness-drivable.

use cascade_orch::{Cascade, TickIn};
use falcon_cascade_step_bindings::exports::falcon::cascade_step::step_api::Guest;

struct Component;

static mut CASCADE: Option<Cascade> = None;

fn cascade() -> &'static mut Cascade {
    unsafe {
        if CASCADE.is_none() {
            CASCADE = Some(Cascade::new());
        }
        CASCADE.as_mut().unwrap()
    }
}

impl Guest for Component {
    fn step(
        ax: f32,
        ay: f32,
        az: f32,
        gx: f32,
        gy: f32,
        gz: f32,
        north: f32,
        east: f32,
        down: f32,
        yaw: f32,
    ) -> f32 {
        let o = cascade().tick(TickIn {
            ax,
            ay,
            az,
            gx,
            gy,
            gz,
            north,
            east,
            down,
            yaw,
        });
        o.m1 + o.m2 + o.m3 + o.m4
    }
}

falcon_cascade_step_bindings::export!(Component with_types_in falcon_cascade_step_bindings);
