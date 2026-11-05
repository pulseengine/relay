// PR-only: count filter re-designs and pass-through entries under straddle jitter.
extern crate std;
use std::prelude::v1::*;
use std::{vec, println};
use crate::{CascadePartition, SUPERVISOR_CORE_HOVER_THRUST};

fn rand2(lo: f32, hi: f32, seed: u64) -> Vec<f32> {
    let mut s = seed;
    (0..200000).map(|_| {
        s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        if ((s >> 33) % 2) == 0 { hi } else { lo }
    }).collect()
}

#[test]
fn probepr_straddle_counts() {
    let cases: Vec<(&str, Vec<f32>)> = vec![
        ("alt 7.4/7.6ms", vec![0.0074, 0.0076]),
        ("rand 7.4/7.6ms s1", rand2(0.0074, 0.0076, 1)),
        ("rand 7.2/7.8ms s1", rand2(0.0072, 0.0078, 1)),
        ("rand 7.0/8.0ms s3", rand2(0.0070, 0.0080, 3)),
        ("rand 6.5/7.5ms s3", rand2(0.0065, 0.0075, 3)),
        ("rand 6.0/9.0ms s4", rand2(0.0060, 0.0090, 4)),
        ("rand 6.6/6.9ms s7", rand2(0.0066, 0.0069, 7)),
        ("rand 3.3/3.5ms s8", rand2(0.0033, 0.0035, 8)),
    ];
    for (name, pat) in cases {
        let mut c = CascadePartition::new(SUPERVISOR_CORE_HOVER_THRUST, 1000.0);
        let mean = pat.iter().take(2600).sum::<f32>() / 2600.0f32.min(pat.len() as f32);
        let n = (20.0 / mean) as usize;
        let (mut entries, mut exits, mut prev_pass) = (0u32, 0u32, false);
        let (mut lo, mut hi) = (f32::INFINITY, 0.0f32);
        for k in 0..n {
            c.redesign_filters(pat[k % pat.len()]);
            let f = c.filter_design_hz();
            let pass = 60.0 >= relay_adrc::LPF_MAX_FC_FRAC * f;
            if k > 0 && pass && !prev_pass { entries += 1; }
            if k > 0 && !pass && prev_pass { exits += 1; }
            prev_pass = pass;
            if k > 16 { lo = lo.min(f); hi = hi.max(f); }
        }
        println!("PR|{name}|frames {n}|redesigns {}|pass-through entries {entries} exits {exits}|design range {lo:.2}-{hi:.2} Hz", c.filter_redesigns());
    }
}
