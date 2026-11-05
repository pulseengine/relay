// PROBE (review140, uncommitted): does dt jitter > 2% rebuild (zero) the gyro LPF every tick?
use falcon_core::{FlightBackend, FlightCore, ImuSample, Pathology, SimBackend};
type Vec3 = [f32; 3];

struct J {
    inner: SimBackend,
    pat: Vec<f32>,
    i: usize,
}
impl FlightBackend for J {
    fn read_imu(&mut self) -> ImuSample {
        self.inner.read_imu()
    }
    fn read_position(&mut self) -> Option<Vec3> {
        self.inner.read_position()
    }
    fn read_mag(&mut self) -> Option<Vec3> {
        self.inner.read_mag()
    }
    fn read_heading(&mut self) -> Option<f32> {
        self.inner.read_heading()
    }
    fn write_motors(&mut self, m: &[f32]) {
        self.inner.write_motors(m);
        self.i += 1;
    }
    fn dt(&self) -> f32 {
        self.pat[self.i % self.pat.len()]
    }
    fn read_battery_v(&mut self) -> Option<f32> {
        self.inner.read_battery_v()
    }
    fn read_range(&mut self) -> Option<f32> {
        self.inner.read_range()
    }
    fn read_motor_rpm(&mut self) -> Option<[i32; 4]> {
        self.inner.read_motor_rpm()
    }
}

fn run(name: &str, pat: Vec<f32>) {
    runp(name, pat, 0.004);
}
fn runp(name: &str, pat: Vec<f32>, pdt: f32) {
    run2(name, pat.clone(), false, pdt);
    run2(&format!("{name} [noisy]"), pat, true, pdt);
}
fn run2(name: &str, pat: Vec<f32>, noisy: bool, pdt: f32) {
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = J {
        inner: if noisy { SimBackend::new(level, pdt).with_pathology(Pathology { gyro_white: 0.02, vibration: 0.5, gps_noise: 0.3, turbulence: 1.0, ..Pathology::default() }) } else { SimBackend::new(level, pdt) },
        pat,
        i: 0,
    };
    let mut core = FlightCore::new(0.5, 1.0 / pdt);
    core.set_pos_var(0.25);
    core.set_process_floor(0.30, 0.05);
    core.set_position([0.0, 0.0, -2.0]);
    b.inner.pos[0] = 1.0;
    let n = (30.0 / pdt) as usize; // 30 s
    let (mut acc, mut cnt, mut rebuilds, mut peak_w, mut peak_h) =
        (0.0f64, 0usize, 0usize, 0.0f32, 0.0f32);
    let mut last = core.designed_loop_hz();
    let mut mmin = 1.0f32;
    let mut mmax = 0.0f32;
    for k in 0..n {
        core.step(&mut b);
        let d = core.designed_loop_hz();
        if d != last {
            rebuilds += 1;
            last = d;
        }
        let w = b.inner.omega;
        if k > n / 3 {
            acc += (w[0] * w[0] + w[1] * w[1]) as f64;
            cnt += 1;
            peak_w = peak_w.max((w[0] * w[0] + w[1] * w[1]).sqrt());
            let h = (b.inner.pos[0].powi(2) + b.inner.pos[1].powi(2)).sqrt();
            peak_h = peak_h.max(h);
            let m = [core.max_motor()];
            for x in m {
                mmin = mmin.min(x);
                mmax = mmax.max(x);
            }
        }
        if !b.inner.pos[2].is_finite() {
            break;
        }
    }
    println!("{name}: failed_motor={:?}", core.failed_motor());
    println!(
        "{name}: rebuilds={rebuilds} rp_rate_rms={:.4} peak_rate={:.3} peak_h={:.3} alt={:.3} motors[{:.3},{:.3}]",
        (acc / cnt.max(1) as f64).sqrt(),
        peak_w,
        peak_h,
        -b.inner.pos[2],
        mmin,
        mmax
    );
}

#[test]
fn probe_jitter() {
    if std::env::var("KEEP").is_ok() { falcon_core::PROBE_KEEP.store(true, std::sync::atomic::Ordering::Relaxed); }
    runp("R400 fixed 2.5ms", vec![0.0025], 0.0025);
    runp("R400 1ms-tick 2/3ms", vec![0.002, 0.003], 0.0025);
    runp("R1k fixed", vec![0.001], 0.001);
    runp("R1k +-25us alt", vec![0.000975, 0.001025], 0.001);
    run("A fixed 4.0ms", vec![0.004]);
    run("B 4.0/4.1ms alt (2.5%)", vec![0.004, 0.0041]);
    run(
        "C ms-quantized 1-in-10",
        vec![0.004, 0.004, 0.004, 0.004, 0.005, 0.003, 0.004, 0.004, 0.004, 0.004],
    );
    run("D 4.0/4.6ms alt (15%)", vec![0.004, 0.0046]);
    run("E 1.9% jitter", vec![0.004, 0.004076]);
    // pseudo-random uniform +-3% (LCG), 4 ms nominal = +-120 us
    let mut s: u32 = 12345;
    let mut v = Vec::new();
    for _ in 0..997 {
        s = s.wrapping_mul(1664525).wrapping_add(1013904223);
        let u = (s >> 8) as f32 / (1u32 << 24) as f32; // [0,1)
        v.push(0.004 * (1.0 + 0.06 * (u - 0.5)));
    }
    run("F random +-3%", v.clone());
    let v2: Vec<f32> = v.iter().map(|x| 0.004 + (x - 0.004) / 3.0).collect();
    run("G random +-1%", v2);
    let v3: Vec<f32> = v.iter().map(|x| 0.004 + (x - 0.004) * 5.0 / 3.0).collect();
    run("H random +-5%", v3);
}

fn sup_run(name: &str, pat: Vec<f32>, pdt: f32) {
    use falcon_core::FlightSupervisor;
    use falcon_core::relay_fsm::Event;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut b = J { inner: SimBackend::new(level, pdt).with_pathology(Pathology { gyro_white: 0.02, vibration: 0.5, gps_noise: 0.3, turbulence: 1.0, ..Pathology::default() }), pat, i: 0 };
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    let r1 = sup.command(Event::Arm, true, true);
    let r2 = sup.command(Event::RequestTakeoff, true, true);
    let n = (30.0 / pdt) as usize;
    let mut peak_tilt = 0.0f32;
    let mut modes = Vec::new();
    for _ in 0..n {
        sup.step(&mut b);
        peak_tilt = peak_tilt.max(b.inner.r[2][2].clamp(-1.0, 1.0).acos());
        let m = sup.mode();
        if modes.last() != Some(&m) { modes.push(m); }
    }
    println!("SUP {name}: arm={r1:?} to={r2:?} modes={modes:?} peak_tilt={peak_tilt:.3} alt={:.2} flags={:#x}",
        -b.inner.pos[2], sup.failsafe_flags());
}

#[test]
fn probe_sup() {
    if std::env::var("KEEP").is_ok() { falcon_core::PROBE_KEEP.store(true, std::sync::atomic::Ordering::Relaxed); }
    sup_run("400Hz fixed", vec![0.0025], 0.0025);
    sup_run("400Hz 1ms-tick 2/3ms", vec![0.002, 0.003], 0.0025);
}
