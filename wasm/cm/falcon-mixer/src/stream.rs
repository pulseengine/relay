// Falcon Quad Mixer — P3 Stream Transformer (shared cascade-stream types).
//
// stream<torque-setpoint> -> stream<motor-pwm> over the shared types
// (STREAM-P10). Drives the verified relay-mix-quad QuadMixer.

use relay_mix_quad::QuadMixer;

use falcon_mixer_stream_bindings::exports::pulseengine::falcon_cascade_stream::mixer_stream::{
    Guest, MotorPwm as WitPwm, TorqueSetpoint as WitTorque,
};

struct Component;

impl Guest for Component {
    async fn monitor(
        mut inputs: wit_bindgen::rt::async_support::StreamReader<WitTorque>,
    ) -> wit_bindgen::rt::async_support::StreamReader<WitPwm> {
        let (mut writer, reader) =
            falcon_mixer_stream_bindings::wit_stream::new::<WitPwm>();
        let mut mixer = QuadMixer::new();

        let work = async move {
        while let Some(t) = inputs.next().await {
            let m = mixer.mix([t.tx, t.ty, t.tz], t.thrust);
            let out = WitPwm { m1: m[0], m2: m[1], m3: m[2], m4: m[3] };
            let _ = writer.write(vec![out]).await;
        }
        };
        // Spawned, `monitor` returns the reader at once. Awaited inline, the
        // first write waits for a reader the caller only gets when `monitor`
        // returns: under wasmtime 51 the inline form produced 0 outputs and
        // hung (tests/p3-stream). spawn_local needs wit-bindgen's async-spawn,
        // which the @crates hub (rules_wasm_component) does not enable, so the
        // Bazel build keeps the inline form until it does.
        #[cfg(feature = "p3-spawn")]
        wit_bindgen::spawn_local(work);
        #[cfg(not(feature = "p3-spawn"))]
        work.await;

        reader
    }
}

falcon_mixer_stream_bindings::export!(Component with_types_in falcon_mixer_stream_bindings);
