//! wasm/cm/rate/src/stream.rs under cargo: the Bazel build names its generated
//! bindings crate `falcon_rate_stream_bindings`; this module stands in for it.
mod falcon_rate_stream_bindings {
    wit_bindgen::generate!({
        path: "../../../wit/falcon-cascade-stream",
        world: "rate-stream-component",
        pub_export_macro: true,
        default_bindings_module: "crate::falcon_rate_stream_bindings",
        generate_all,
    });
}

include!("../../../../wasm/cm/rate/src/stream.rs");
