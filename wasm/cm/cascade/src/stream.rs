// Falcon Control Cascade — COMPOSED P3 Stream Transformer component.
//
// stream<cascade-input{imu, target}> -> stream<motor-pwm>, running the full
// flight-control pipeline (iekf -> pos -> att -> rate -> mixer) per tick. The
// streamed counterpart of the sync falcon-cascade socket: one orchestrating
// component driving all FIVE verified engines directly (STREAM-P11, v1.73).
//
// wac composes function-call interfaces, not stream data-flow, so a streamed
// pipeline is one orchestrating component rather than a wac_plug of the five
// stream components. Single input stream of a combined record — no multi-stream
// zip. Each stage runs its actual verified engine; no inline copies.

use cascade_orch::{Cascade, TickIn};

use falcon_cascade_stream_composed_bindings::exports::pulseengine::falcon_cascade_stream::cascade_stream::{
    CascadeInput as WitInput, Guest, MotorPwm as WitPwm,
};

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
    /// One tick of the full cascade per input — drives the shared `cascade_orch`
    /// pipeline (estimate -> position -> attitude -> rate -> mix), identical to
    /// the sync `step` coverage sibling.
    async fn monitor(
        mut inputs: wit_bindgen::rt::async_support::StreamReader<WitInput>,
    ) -> wit_bindgen::rt::async_support::StreamReader<WitPwm> {
        let (mut writer, reader) =
            falcon_cascade_stream_composed_bindings::wit_stream::new::<WitPwm>();

        while let Some(inp) = inputs.next().await {
            let o = cascade().tick(TickIn {
                ax: inp.imu.ax,
                ay: inp.imu.ay,
                az: inp.imu.az,
                gx: inp.imu.gx,
                gy: inp.imu.gy,
                gz: inp.imu.gz,
                north: inp.target.north,
                east: inp.target.east,
                down: inp.target.down,
                yaw: inp.target.yaw,
            });
            let out = WitPwm { m1: o.m1, m2: o.m2, m3: o.m3, m4: o.m4 };
            let _ = writer.write(vec![out]).await;
        }

        reader
    }
}

falcon_cascade_stream_composed_bindings::export!(Component with_types_in falcon_cascade_stream_composed_bindings);
