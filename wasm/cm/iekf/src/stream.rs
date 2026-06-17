// Falcon IEKF Estimator — P3 Stream Transformer WASM component.
//
// Takes stream<imu-sample>, emits stream<vehicle-state>.
// Wraps the verified relay-iekf invariant EKF — the fourth control-cascade
// engine promoted to a P3 async stream (#168 cascade arc).
//
// Built as a rust_wasm_component_bindgen (wasi_version="p3"), the proven
// cFS-stream pattern, over the verified :relay-iekf Bazel library.

use relay_iekf::{Iekf, Imu as RImu};

use falcon_iekf_stream_bindings::exports::falcon::iekf_stream::iekf_stream::{
    Guest, ImuSample as WitImu, VehicleState as WitState,
};

struct Component;

// Stateful across the stream: the EKF state persists sample to sample.
static mut IEKF: Option<Iekf> = None;

fn iekf() -> &'static mut Iekf {
    unsafe {
        if IEKF.is_none() { IEKF = Some(Iekf::level()); }
        IEKF.as_mut().unwrap()
    }
}

impl Guest for Component {
    /// STREAM TRANSFORMER: reads IMU samples, runs the verified EKF (1 kHz
    /// SE2(3) propagate + gravity update) on each, writes the vehicle-state
    /// estimate to the output stream.
    async fn monitor(
        mut samples: wit_bindgen::rt::async_support::StreamReader<WitImu>,
    ) -> wit_bindgen::rt::async_support::StreamReader<WitState> {
        let (mut writer, reader) =
            falcon_iekf_stream_bindings::wit_stream::new::<WitState>();

        while let Some(imu) = samples.next().await {
            let accel = [imu.ax, imu.ay, imu.az];
            let gyro = [imu.gx, imu.gy, imu.gz];
            let f = iekf();
            f.propagate(RImu { gyro, accel }, 0.001);
            f.update_gravity(accel, 0.5);
            let st = f.state();
            let out = WitState {
                qw: st.q[0],
                qx: st.q[1],
                qy: st.q[2],
                qz: st.q[3],
                pos_n: st.p[0],
                pos_e: st.p[1],
                pos_d: st.p[2],
                vel_n: st.v[0],
                vel_e: st.v[1],
                vel_d: st.v[2],
                // Body rates are the gyro reading, passed through for the rate loop.
                wx: imu.gx,
                wy: imu.gy,
                wz: imu.gz,
                innovation: 0.0,
            };
            let _ = writer.write(vec![out]).await;
        }

        reader
    }
}

falcon_iekf_stream_bindings::export!(Component with_types_in falcon_iekf_stream_bindings);
