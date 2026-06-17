// Falcon Quad Mixer — P3 Stream Transformer WASM component.
//
// Takes stream<torque-setpoint>, emits stream<motor-pwm>.
// Wraps the verified relay-mix-quad QuadMixer — the fifth and final
// control-cascade engine promoted to a P3 async stream (#168 cascade arc).
//
// Built as a rust_wasm_component_bindgen (wasi_version="p3"), the proven
// cFS-stream pattern, over the verified :relay-mix-quad Bazel library.

use relay_mix_quad::QuadMixer;

use falcon_mixer_stream_bindings::exports::falcon::mixer_stream::mixer_stream::{
    Guest, MotorPwm as WitPwm, TorqueSetpoint as WitTorque,
};

struct Component;

impl Guest for Component {
    /// STREAM TRANSFORMER: reads torque setpoints, mixes each to the four motor
    /// PWM commands, writes them to the output stream. The mixer is stateless.
    async fn monitor(
        mut inputs: wit_bindgen::rt::async_support::StreamReader<WitTorque>,
    ) -> wit_bindgen::rt::async_support::StreamReader<WitPwm> {
        let (mut writer, reader) =
            falcon_mixer_stream_bindings::wit_stream::new::<WitPwm>();
        let mut mixer = QuadMixer::new();

        while let Some(t) = inputs.next().await {
            let m = mixer.mix([t.tx, t.ty, t.tz], t.thrust);
            let out = WitPwm {
                m1: m[0],
                m2: m[1],
                m3: m[2],
                m4: m[3],
            };
            let _ = writer.write(vec![out]).await;
        }

        reader
    }
}

falcon_mixer_stream_bindings::export!(Component with_types_in falcon_mixer_stream_bindings);
