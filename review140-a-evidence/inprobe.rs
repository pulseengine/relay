
#[cfg(test)]
mod review_probe_step {
    use super::*;
    const L: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    fn run(from: f32, to: f32, pre: u32, pert: bool) -> (u32, f32, f32, f32) {
        let mut b = SimBackend::new(L, 1.0 / from).with_pathology(Pathology {
            gyro_white: 0.02,
            vibration: 0.5,
            gps_noise: 0.3,
            turbulence: 1.0,
            ..Pathology::default()
        });
        b.pos = [0.0, 0.0, -2.0];
        let mut core = FlightCore::new(SUPERVISOR_CORE_HOVER_THRUST, SUPERVISOR_CORE_LOOP_HZ);
        core.set_position([0.0, 0.0, -2.0]);
        for _ in 0..pre {
            core.step(&mut b);
        }
        b.dt = 1.0 / to;
        if pert {
            b.omega = [1.5, -1.0, 0.0];
        }
        let mut lag = u32::MAX;
        let (mut pk_rate, mut pk_tilt) = (0.0f32, 0.0f32);
        let mut acc = 0.0f64;
        let n = (1.0 * to) as u32;
        for k in 0..n {
            core.step(&mut b);
            if lag == u32::MAX && (core.designed_loop_hz() - to).abs() <= 0.02 * to {
                lag = k + 1;
            }
            let w = b.omega;
            let r = (w[0] * w[0] + w[1] * w[1]).sqrt();
            pk_rate = pk_rate.max(r);
            pk_tilt = pk_tilt.max(b.tilt());
            acc += (r * r) as f64;
        }
        (lag, pk_rate, pk_tilt, (acc / n as f64).sqrt() as f32)
    }
    #[test]
    fn probe_rate_step() {
        for (from, to) in [(1000.0f32, 250.0f32), (250.0, 1000.0), (1000.0, 100.0)] {
            let base = if from == 1000.0 { 2000 } else { 500 };
            for pert in [false, true] {
                let mut worst = (0u32, 0.0f32, 0.0f32, 0.0f32);
                let mut best = (u32::MAX, f32::MAX, f32::MAX, f32::MAX);
                for ph in 0..16u32 {
                    let r = run(from, to, base + ph, pert);
                    worst = (worst.0.max(r.0), worst.1.max(r.1), worst.2.max(r.2), worst.3.max(r.3));
                    best = (best.0.min(r.0), best.1.min(r.1), best.2.min(r.2), best.3.min(r.3));
                }
                std::eprintln!("STEP {from}->{to} pert {pert}: lag frames best/worst {}/{} peak rate {:.3}/{:.3} peak tilt {:.3}/{:.3} rms {:.4}/{:.4}", best.0, worst.0, best.1, worst.1, best.2, worst.2, best.3, worst.3);
            }
        }
    }
}
