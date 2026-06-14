//! Secure inter-core comms demo — the v1.60 (relay-bus seam) + v1.59 (relay-sec
//! transport-agnostic) story, on screen, driven by the REAL components.
//!
//! Models jess's inter-core link (Pixhawk 6X-RT M7 -> M4): a producer wraps
//! command messages with relay-sec (`SecurityHeader || payload || Ascon-tag`)
//! and pushes the frames into a no_alloc `SpscRing` (the shared-memory mailbox,
//! relay-bus); the consumer pops FIFO and verifies. Nothing here is faked —
//! every byte on screen comes from `SecurityChannel::wrap`/`verify` and the
//! real ring, so this is a falsifiable demo, not an animation over a card.
//!
//! Run:  cargo run -p relay-sb --example intercore
//! Fast: RELAY_DEMO_FAST=1 cargo run -p relay-sb --example intercore   (no sleeps)

use relay_bus::{MessageTransport, SpscRing};
use relay_sec::frame::{SecurityChannel, VerifyError};
use std::io::Write;
use std::time::Duration;

const FRAME_CAP: usize = 64;
const RING_N: usize = 4;

/// A wire frame sitting in the inter-core mailbox.
#[derive(Clone, Copy)]
struct Frame {
    buf: [u8; FRAME_CAP],
    len: usize,
}

fn beat(ms: u64) {
    if std::env::var("RELAY_DEMO_FAST").is_err() {
        std::io::stdout().flush().ok();
        std::thread::sleep(Duration::from_millis(ms));
    }
}

// ANSI
const DIM: &str = "\x1b[2m";
const B: &str = "\x1b[1m";
const G: &str = "\x1b[32m";
const R: &str = "\x1b[31m";
const Y: &str = "\x1b[33m";
const C: &str = "\x1b[36m";
const X: &str = "\x1b[0m";

/// Render the ring as occupancy slots: filled vs free.
fn ring_view(used: usize) -> String {
    let mut s = String::from("mailbox [");
    for i in 0..RING_N {
        if i < used {
            s.push_str(&format!("{G}█{X}"));
        } else {
            s.push_str(&format!("{DIM}░{X}"));
        }
    }
    s.push_str(&format!("] {used}/{RING_N}"));
    s
}

fn hex(bytes: &[u8], max: usize) -> String {
    let n = bytes.len().min(max);
    let mut s = bytes[..n].iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join("");
    if bytes.len() > max {
        s.push('…');
    }
    s
}

fn main() {
    let key = [0x5Au8; 16];
    // M7 -> M4 direction is one security association (one SPI). The reverse
    // direction would be a distinct SPI (SEC-K15: cross-channel isolation).
    let mut m7 = SecurityChannel::new(0x4D37, 1, key); // sender on M7
    let mut m4 = SecurityChannel::new(0x4D37, 1, key); // receiver on M4
    let mut mailbox: SpscRing<Frame, RING_N> = SpscRing::new();

    println!("\n{B}{C}╔══════════════════════════════════════════════════════════╗{X}");
    println!("{B}{C}║   Relay — secure inter-core comms  (falcon-v1.60.0)      ║{X}");
    println!("{B}{C}╚══════════════════════════════════════════════════════════╝{X}");
    println!("{DIM}  M7 ──[ relay-sec wrap ]──▶ SpscRing mailbox ──▶[ verify ]──▶ M4{X}");
    println!("{DIM}  relay-bus seam (no_std/no_alloc) · relay-sec E2E (Ascon){X}\n");
    beat(1400);

    // ── Phase 1: secure transit ──────────────────────────────────────────
    println!("{B}1 · Secure transit — commands flow M7 ▶ M4, authenticated{X}");
    let cmds: [&[u8]; 3] = [b"ARM", b"TAKEOFF 5m", b"WAYPOINT 12"];
    for cmd in cmds {
        let mut buf = [0u8; FRAME_CAP];
        let n = m7.wrap(cmd, &mut buf).expect("wrap");
        mailbox.push(Frame { buf, len: n });
        println!(
            "   {C}M7 wrap{X} {:<12} → {DIM}{}{X}   {}",
            String::from_utf8_lossy(cmd),
            hex(&buf[..n], 14),
            ring_view(mailbox.len())
        );
        beat(750);
    }
    beat(500);
    while let Some(f) = mailbox.pop() {
        match m4.verify(&f.buf[..f.len]) {
            Ok(payload) => println!(
                "   {G}M4 verify ✓ AUTHENTIC{X} → {B}{}{X}   {}",
                String::from_utf8_lossy(payload),
                ring_view(mailbox.len())
            ),
            Err(e) => println!("   {R}M4 verify ✗ {e:?}{X}"),
        }
        beat(650);
    }
    println!();
    beat(700);

    // ── Phase 2: backpressure (the seam contract) ────────────────────────
    println!("{B}2 · Backpressure — a full mailbox REFUSES, never overwrites{X}");
    for i in 0..RING_N {
        let mut buf = [0u8; FRAME_CAP];
        let n = m7.wrap(format!("MSG{i}").as_bytes(), &mut buf).unwrap();
        let ok = mailbox.push(Frame { buf, len: n });
        println!("   push MSG{i} → {}   {}", if ok { format!("{G}accepted{X}") } else { format!("{R}refused{X}") }, ring_view(mailbox.len()));
        beat(450);
    }
    let mut buf = [0u8; FRAME_CAP];
    let n = m7.wrap(b"OVERFLOW", &mut buf).unwrap();
    let refused = !mailbox.push(Frame { buf, len: n });
    println!(
        "   push OVERFLOW → {Y}⊘ {}{X}   {}   {DIM}(MessageTransport::push → false){X}",
        if refused { "BACKPRESSURE: refused, head intact" } else { "??" },
        ring_view(mailbox.len())
    );
    while mailbox.pop().is_some() {} // drain
    println!();
    beat(900);

    // ── Phase 3: tamper + cross-channel rejection ────────────────────────
    println!("{B}3 · Tamper & cross-channel — inauthentic frames are rejected{X}");
    let mut buf = [0u8; FRAME_CAP];
    let n = m7.wrap(b"DISARM", &mut buf).unwrap();

    // (a) flip a payload bit -> BadMac
    let mut tampered = buf;
    tampered[11] ^= 0x40;
    print!("   tampered DISARM frame  → ");
    match m4.verify(&tampered[..n]) {
        Err(VerifyError::BadMac) => println!("{G}✗ REJECTED (BadMac){X}"),
        other => println!("{R}UNEXPECTED {other:?}{X}"),
    }
    beat(700);

    // (b) a frame minted on a DIFFERENT core direction (other SPI) -> UnknownSpi
    let mut other_core = SecurityChannel::new(0x4D47, 1, key); // reverse direction SA
    let mut buf2 = [0u8; FRAME_CAP];
    let n2 = other_core.wrap(b"DISARM", &mut buf2).unwrap();
    print!("   frame from other core  → ");
    match m4.verify(&buf2[..n2]) {
        Err(VerifyError::UnknownSpi) => println!("{G}✗ REJECTED (UnknownSpi · SEC-K15){X}"),
        other => println!("{R}UNEXPECTED {other:?}{X}"),
    }
    beat(900);

    println!("\n{DIM}  Verified: Kani BUS-K01/K02 (ring total + bounded, full refuses),{X}");
    println!("{DIM}            SEC-K15 (cross-channel isolation, all SPI pairs).{X}");
    println!("{B}{G}  One verified layer; the transport on top. ✓{X}\n");
}
