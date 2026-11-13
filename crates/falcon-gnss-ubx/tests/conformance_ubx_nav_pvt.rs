use falcon_gnss_ubx::{NavPvt, UbxParser};

include!("vectors/ubx_nav_pvt.rs");

fn check(v: &Pvt, got: &NavPvt, ctx: &str, idx: usize) {
    assert_eq!(got.lat_deg_e7, v.lat_deg_e7, "{} lat vec {}", ctx, idx);
    assert_eq!(got.lon_deg_e7, v.lon_deg_e7, "{} lon vec {}", ctx, idx);
    assert_eq!(got.height_mm, v.height_mm, "{} height vec {}", ctx, idx);
    assert_eq!(got.h_msl_mm, v.h_msl_mm, "{} hmsl vec {}", ctx, idx);
    assert_eq!(got.vel_n_mmps, v.vel_n_mmps, "{} velN vec {}", ctx, idx);
    assert_eq!(got.vel_e_mmps, v.vel_e_mmps, "{} velE vec {}", ctx, idx);
    assert_eq!(got.vel_d_mmps, v.vel_d_mmps, "{} velD vec {}", ctx, idx);
    assert_eq!(got.fix_type, v.fix_type, "{} fix_type vec {}", ctx, idx);
    assert_eq!(got.num_sv, v.num_sv, "{} num_sv vec {}", ctx, idx);
}

#[test]
fn conformance_pvt_single_frame() {
    assert!(!PVT_VECTORS.is_empty());
    for (i, v) in PVT_VECTORS.iter().enumerate() {
        let mut p = UbxParser::new();
        let mut outs = Vec::new();
        for (k, &b) in v.bytes.iter().enumerate() {
            if let Some(f) = p.push(b) {
                outs.push((k, f));
            }
        }
        assert_eq!(outs.len(), 1, "vector {} decoded count", i);
        assert_eq!(
            outs[0].0,
            v.bytes.len() - 1,
            "vector {} emitted at last byte",
            i
        );
        check(v, &outs[0].1, "single", i);
    }
}

#[test]
fn conformance_pvt_back_to_back_reused_parser() {
    let mut p = UbxParser::new();
    for (i, v) in PVT_VECTORS.iter().enumerate() {
        let mut outs = Vec::new();
        for &b in v.bytes {
            if let Some(f) = p.push(b) {
                outs.push(f);
            }
        }
        assert_eq!(outs.len(), 1, "vector {} decoded count (reused parser)", i);
        check(v, &outs[0], "reused", i);
    }
}

#[test]
fn conformance_streams_chunked() {
    assert!(!STREAMS.is_empty());
    for (si, s) in STREAMS.iter().enumerate() {
        let mut p = UbxParser::new();
        let mut outs = Vec::new();
        let mut pos = 0usize;
        let mut ci = 0usize;
        while pos < s.bytes.len() {
            let n = s.chunks[ci % s.chunks.len()].max(1);
            ci += 1;
            let end = (pos + n).min(s.bytes.len());
            for &b in &s.bytes[pos..end] {
                if let Some(f) = p.push(b) {
                    outs.push(f);
                }
            }
            pos = end;
        }
        assert_eq!(outs.len(), s.expect.len(), "stream {} output count", si);
        for (k, &ei) in s.expect.iter().enumerate() {
            check(&PVT_VECTORS[ei], &outs[k], "stream", si);
        }
    }
}
