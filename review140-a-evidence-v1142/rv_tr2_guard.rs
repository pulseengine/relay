include!("/private/tmp/claude-501/-Users-r-git-pulseengine-relay/f207d8bc-6a92-4057-9e33-a06782c12f0c/scratchpad/rv_adv2.rs");
#[test]
fn trace2() {
    let out = std::env::var("RV_OUT").unwrap();
    let pat = std::env::var("TR_PAT").unwrap();
    let hz: f32 = std::env::var("TR_HZ").unwrap().parse().unwrap();
    let vib: f32 = std::env::var("TR_VIB").unwrap().parse().unwrap();
    let every: u64 = std::env::var("TR_EVERY").map(|v| v.parse().unwrap()).unwrap_or(25);
    let (mut b, mut sup) = rig(hz, &pat, 1.0);
    if let Some(i) = pat.rfind('s') { b.b.reseed(pat[i+1..].parse::<u32>().unwrap_or(1) + 1); }
    let mut f = std::fs::File::create(out).unwrap();
    let pre = (5.0 * hz) as usize;
    for _ in 0..pre { sup.step(&mut b); }
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    let mut loiter = false; let mut tick = 0u64; let mut done_after: i64 = -1;
    let mut t = 0.0f64;
    for _ in 0..(60.0 * hz) as usize {
        let dtc = b.cur();
        sup.step(&mut b); tick += 1; t += dtc as f64;
        if !loiter && sup.mode() == Mode::Loiter { loiter = true; b.b.rotor_vib = vib; writeln!(f, "LOITER t={t:.3}").unwrap(); }
        let fm = sup.core().failed_motor();
        if fm.is_some() && done_after < 0 { done_after = tick as i64 + (0.5 * hz) as i64; writeln!(f, "ISO t={t:.3} {:?}", fm).unwrap(); }
        let near = done_after >= 0;
        if tick % every == 0 || near {
            let d = sup.core().fdi_diag();
            let w = sup.core().last_omega_d();
            writeln!(f, "t={t:.3} mode={:?} tilt={:.4} z={:.2} dhz={:.1} fdi=({:.4},{:.4},{},[{:.3},{:.3},{:.3},{:.3}]) wd=[{:.3},{:.3},{:.3}] m=[{:.3},{:.3},{:.3},{:.3}]{}", sup.mode(), b.b.tilt(), b.b.pos[2], sup.core().designed_loop_hz(), d.0, d.1, d.2, d.3[0], d.3[1], d.3[2], d.3[3], w[0], w[1], w[2], b.last_m[0], b.last_m[1], b.last_m[2], b.last_m[3], format!(" g={} fhz={:.1}", sup.core().alternation_guard(), sup.core().filter_design_hz())).unwrap();
        }
        if done_after >= 0 && tick as i64 > done_after { break; }
    }
}
