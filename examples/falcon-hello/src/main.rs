//! falcon-hello — v0.1 showcase example.
//!
//! Demonstrates the relay-mavlink codec and relay-ekf-stub in a
//! runnable MAVLink heartbeat exchange over UDP loopback.
//!
//!   falcon-hello --mode vehicle   # send heartbeats (the autopilot side)
//!   falcon-hello --mode gcs       # receive heartbeats (the GCS side)
//!
//! Run both at once (different terminals) and you see the vehicle's
//! heartbeats decoded by the GCS — the full encode → wire → decode
//! round trip working over real sockets.
//!
//! By default the vehicle binds to 127.0.0.1:14550 and the GCS
//! listens on the same port. (14550 is the conventional MAVLink
//! UDP port; QGroundControl listens here by default.) Override
//! with --bind and --remote.

use std::io::ErrorKind;
use std::net::{SocketAddr, UdpSocket};
use std::process::ExitCode;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use relay_ekf_stub::{EkfStub, Timestamp};
use relay_mavlink::{
    encode_frame, parse_frame, peek_message_id, CodecError, Frame, FrameHeader, Heartbeat,
    HEARTBEAT_CRC_EXTRA, HEARTBEAT_MSG_ID, HEARTBEAT_PAYLOAD_LEN, MAGIC_V2, MAX_FRAME_SIZE,
};

const DEFAULT_PORT: u16 = 14550;
const VEHICLE_SYSTEM_ID: u8 = 1;
const VEHICLE_COMPONENT_ID: u8 = 1;
// MAVLink convention: GCS announces with sysid=255, compid=0. v0.1 GCS
// only listens; v0.2 will emit a periodic GCS heartbeat to identify
// itself to the vehicle, which is when these get used.
#[allow(dead_code)]
const GCS_SYSTEM_ID: u8 = 255;
#[allow(dead_code)]
const GCS_COMPONENT_ID: u8 = 0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    Vehicle,
    Gcs,
}

#[derive(Debug)]
struct Args {
    mode: Mode,
    bind: SocketAddr,
    remote: SocketAddr,
    duration: Option<Duration>,
    rate_hz: u32,
}

impl Args {
    fn parse(argv: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut mode: Option<Mode> = None;
        let mut bind: Option<SocketAddr> = None;
        let mut remote: Option<SocketAddr> = None;
        let mut duration: Option<Duration> = None;
        let mut rate_hz: u32 = 1;
        let mut argv = argv.peekable();
        // skip program name
        argv.next();
        while let Some(arg) = argv.next() {
            match arg.as_str() {
                "--mode" => {
                    let v = argv.next().ok_or("--mode requires an argument")?;
                    mode = Some(match v.as_str() {
                        "vehicle" => Mode::Vehicle,
                        "gcs" => Mode::Gcs,
                        other => return Err(format!("unknown --mode {other}")),
                    });
                }
                "--bind" => {
                    let v = argv.next().ok_or("--bind requires an argument")?;
                    bind = Some(v.parse().map_err(|e| format!("--bind: {e}"))?);
                }
                "--remote" => {
                    let v = argv.next().ok_or("--remote requires an argument")?;
                    remote = Some(v.parse().map_err(|e| format!("--remote: {e}"))?);
                }
                "--duration" => {
                    let v = argv.next().ok_or("--duration requires seconds")?;
                    let secs: u64 = v.parse().map_err(|e| format!("--duration: {e}"))?;
                    duration = Some(Duration::from_secs(secs));
                }
                "--rate" => {
                    let v = argv.next().ok_or("--rate requires hz")?;
                    rate_hz = v.parse().map_err(|e| format!("--rate: {e}"))?;
                    if rate_hz == 0 {
                        return Err("--rate must be > 0".into());
                    }
                }
                "--help" | "-h" => {
                    print_help();
                    std::process::exit(0);
                }
                other => return Err(format!("unknown argument: {other}")),
            }
        }
        let mode = mode.ok_or("--mode is required (vehicle|gcs)")?;
        let (default_bind, default_remote) = match mode {
            Mode::Vehicle => (
                SocketAddr::from(([127, 0, 0, 1], DEFAULT_PORT + 1)),
                SocketAddr::from(([127, 0, 0, 1], DEFAULT_PORT)),
            ),
            Mode::Gcs => (
                SocketAddr::from(([127, 0, 0, 1], DEFAULT_PORT)),
                SocketAddr::from(([127, 0, 0, 1], DEFAULT_PORT + 1)),
            ),
        };
        Ok(Self {
            mode,
            bind: bind.unwrap_or(default_bind),
            remote: remote.unwrap_or(default_remote),
            duration,
            rate_hz,
        })
    }
}

fn print_help() {
    eprintln!(
        "falcon-hello — v0.1 MAVLink heartbeat exchange example\n\n\
         USAGE:\n  \
           falcon-hello --mode vehicle [--bind ADDR] [--remote ADDR] [--rate HZ] [--duration SECS]\n  \
           falcon-hello --mode gcs     [--bind ADDR] [--remote ADDR]            [--duration SECS]\n\n\
         By default vehicle binds 127.0.0.1:14551 and talks to 127.0.0.1:14550,\n\
         and gcs binds 127.0.0.1:14550. Run vehicle and gcs together to see the\n\
         heartbeat exchange over real UDP sockets.\n"
    );
}

fn main() -> ExitCode {
    let args = match Args::parse(std::env::args()) {
        Ok(a) => a,
        Err(e) => {
            eprintln!("error: {e}");
            print_help();
            return ExitCode::from(2);
        }
    };
    eprintln!(
        "falcon-hello mode={:?} bind={} remote={}",
        args.mode, args.bind, args.remote
    );
    let result = match args.mode {
        Mode::Vehicle => run_vehicle(&args),
        Mode::Gcs => run_gcs(&args),
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::from(1)
        }
    }
}

fn run_vehicle(args: &Args) -> Result<(), String> {
    let sock = UdpSocket::bind(args.bind).map_err(|e| format!("bind {}: {e}", args.bind))?;
    sock.set_read_timeout(Some(Duration::from_millis(100)))
        .map_err(|e| format!("set_read_timeout: {e}"))?;
    let interval = Duration::from_secs_f64(1.0 / args.rate_hz as f64);
    let mut ekf = EkfStub::new();
    let mut seq: u8 = 0;
    let mut sent: u64 = 0;
    let start = Instant::now();
    let mut next_send = Instant::now();
    let mut buf = [0u8; MAX_FRAME_SIZE];

    eprintln!("vehicle: emitting heartbeats at {} Hz → {}", args.rate_hz, args.remote);

    loop {
        if let Some(d) = args.duration {
            if start.elapsed() >= d {
                eprintln!("vehicle: duration elapsed, sent {sent} heartbeat(s)");
                return Ok(());
            }
        }
        let now = Instant::now();
        if now >= next_send {
            let ts = current_timestamp();
            let _state = ekf.tick(ts); // exercise the EKF stub
            let hb = Heartbeat::falcon_quad_standby();
            let header = FrameHeader {
                magic: MAGIC_V2,
                payload_len: HEARTBEAT_PAYLOAD_LEN as u8,
                incompat_flags: 0,
                compat_flags: 0,
                sequence: seq,
                system_id: VEHICLE_SYSTEM_ID,
                component_id: VEHICLE_COMPONENT_ID,
                message_id: HEARTBEAT_MSG_ID,
            };
            let payload = hb.encode_payload();
            let n = encode_frame(&header, &payload, HEARTBEAT_CRC_EXTRA, &mut buf)
                .map_err(|e| format!("encode_frame: {e:?}"))?;
            sock.send_to(&buf[..n], args.remote)
                .map_err(|e| format!("send_to {}: {e}", args.remote))?;
            seq = seq.wrapping_add(1);
            sent += 1;
            eprintln!(
                "vehicle: tx seq={} type={} status={} mavlink_v={} ({} bytes)",
                header.sequence, hb.mav_type, hb.system_status, hb.mavlink_version, n
            );
            next_send = now + interval;
        }
        // Also drain any incoming traffic (a GCS may also send heartbeats).
        let mut rx_buf = [0u8; MAX_FRAME_SIZE];
        match sock.recv_from(&mut rx_buf) {
            Ok((n, peer)) => {
                if let Err(e) = handle_inbound(&rx_buf[..n], peer) {
                    eprintln!("vehicle: ignoring inbound from {peer}: {e:?}");
                }
            }
            Err(e) if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::TimedOut => {}
            Err(e) => return Err(format!("recv_from: {e}")),
        }
    }
}

fn run_gcs(args: &Args) -> Result<(), String> {
    let sock = UdpSocket::bind(args.bind).map_err(|e| format!("bind {}: {e}", args.bind))?;
    sock.set_read_timeout(Some(Duration::from_millis(500)))
        .map_err(|e| format!("set_read_timeout: {e}"))?;
    eprintln!("gcs: listening on {}", args.bind);
    let start = Instant::now();
    let mut received: u64 = 0;
    let mut buf = [0u8; MAX_FRAME_SIZE];
    loop {
        if let Some(d) = args.duration {
            if start.elapsed() >= d {
                eprintln!("gcs: duration elapsed, received {received} heartbeat(s)");
                return Ok(());
            }
        }
        match sock.recv_from(&mut buf) {
            Ok((n, peer)) => match handle_inbound(&buf[..n], peer) {
                Ok(hb) => {
                    eprintln!(
                        "gcs: rx heartbeat from {} type={} autopilot={} status={} custom_mode={}",
                        peer, hb.mav_type, hb.autopilot, hb.system_status, hb.custom_mode
                    );
                    received += 1;
                }
                Err(e) => eprintln!("gcs: ignoring frame from {peer}: {e:?}"),
            },
            Err(e) if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::TimedOut => {}
            Err(e) => return Err(format!("recv_from: {e}")),
        }
    }
}

/// Decode an inbound MAVLink frame from a peer.
/// Returns the heartbeat if (a) we recognize the message id and
/// (b) the CRC validates. Other messages are accepted but skipped
/// (UnsupportedMessage), which is the right behavior for v0.1.
fn handle_inbound(buf: &[u8], _peer: SocketAddr) -> Result<Heartbeat, CodecError> {
    let msg_id = peek_message_id(buf)?;
    match msg_id {
        HEARTBEAT_MSG_ID => {
            let (frame, _consumed) = parse_frame(buf, HEARTBEAT_CRC_EXTRA)?;
            let Frame { payload, .. } = frame;
            Heartbeat::decode_payload(payload).ok_or(CodecError::BadPayloadLength)
        }
        _ => Err(CodecError::UnsupportedMessage),
    }
}

fn current_timestamp() -> Timestamp {
    let dur = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or(Duration::ZERO);
    Timestamp {
        seconds: dur.as_secs(),
        fraction: ((dur.subsec_nanos() as u64) * (1u64 << 32) / 1_000_000_000) as u32,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;

    /// Integration test: spin up a GCS listener and a vehicle sender
    /// against each other on real UDP sockets, verify at least one
    /// heartbeat exchange happens within a bounded time.
    #[test]
    fn vehicle_and_gcs_exchange_heartbeats_over_udp() {
        // Use ephemeral ports to avoid colliding with a running QGC.
        let gcs_bind: SocketAddr = "127.0.0.1:0".parse().unwrap();
        let vehicle_bind: SocketAddr = "127.0.0.1:0".parse().unwrap();
        let gcs_sock = UdpSocket::bind(gcs_bind).expect("gcs bind");
        let vehicle_sock = UdpSocket::bind(vehicle_bind).expect("vehicle bind");
        gcs_sock.set_read_timeout(Some(Duration::from_secs(2))).unwrap();

        let gcs_addr = gcs_sock.local_addr().unwrap();
        let vehicle_addr = vehicle_sock.local_addr().unwrap();

        // Vehicle thread: send 3 heartbeats to the gcs.
        let vehicle = thread::spawn(move || {
            let mut buf = [0u8; MAX_FRAME_SIZE];
            for seq in 0..3 {
                let hb = Heartbeat::falcon_quad_standby();
                let header = FrameHeader {
                    magic: MAGIC_V2,
                    payload_len: HEARTBEAT_PAYLOAD_LEN as u8,
                    incompat_flags: 0,
                    compat_flags: 0,
                    sequence: seq,
                    system_id: VEHICLE_SYSTEM_ID,
                    component_id: VEHICLE_COMPONENT_ID,
                    message_id: HEARTBEAT_MSG_ID,
                };
                let payload = hb.encode_payload();
                let n = encode_frame(&header, &payload, HEARTBEAT_CRC_EXTRA, &mut buf).unwrap();
                vehicle_sock.send_to(&buf[..n], gcs_addr).unwrap();
                thread::sleep(Duration::from_millis(20));
            }
        });

        // GCS: receive at least one heartbeat, decode, assert fields.
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let (n, peer) = gcs_sock.recv_from(&mut buf).expect("rx heartbeat");
        assert_eq!(peer, vehicle_addr);
        let hb = handle_inbound(&buf[..n], peer).expect("decode heartbeat");
        let expected = Heartbeat::falcon_quad_standby();
        assert_eq!(hb, expected);

        vehicle.join().unwrap();
    }

    #[test]
    fn handle_inbound_rejects_unsupported_message() {
        // A frame with a message-id we don't yet support (e.g. id 1).
        // Encode a fake "STATUS" type frame using HEARTBEAT structure
        // but with msg_id 1; handle_inbound should return UnsupportedMessage
        // (NOT panic, NOT misparse).
        let hb = Heartbeat::falcon_quad_standby();
        let header = FrameHeader {
            magic: MAGIC_V2,
            payload_len: HEARTBEAT_PAYLOAD_LEN as u8,
            incompat_flags: 0,
            compat_flags: 0,
            sequence: 0,
            system_id: 1,
            component_id: 1,
            message_id: 1, // SYS_STATUS, not implemented in v0.1
        };
        let payload = hb.encode_payload();
        let mut buf = [0u8; MAX_FRAME_SIZE];
        // Use HEARTBEAT_CRC_EXTRA so the frame is "well-formed" but
        // for a different msg-id; handle_inbound should bail at the
        // msg-id dispatch before reaching CRC.
        let n = encode_frame(&header, &payload, HEARTBEAT_CRC_EXTRA, &mut buf).unwrap();
        let peer: SocketAddr = "127.0.0.1:0".parse().unwrap();
        let err = handle_inbound(&buf[..n], peer).unwrap_err();
        assert_eq!(err, CodecError::UnsupportedMessage);
    }

    #[test]
    fn handle_inbound_propagates_bad_crc() {
        let hb = Heartbeat::falcon_quad_standby();
        let header = FrameHeader {
            magic: MAGIC_V2,
            payload_len: HEARTBEAT_PAYLOAD_LEN as u8,
            incompat_flags: 0,
            compat_flags: 0,
            sequence: 0,
            system_id: 1,
            component_id: 1,
            message_id: HEARTBEAT_MSG_ID,
        };
        let payload = hb.encode_payload();
        let mut buf = [0u8; MAX_FRAME_SIZE];
        let n = encode_frame(&header, &payload, HEARTBEAT_CRC_EXTRA, &mut buf).unwrap();
        // Corrupt CRC.
        buf[n - 1] ^= 0xAA;
        let peer: SocketAddr = "127.0.0.1:0".parse().unwrap();
        let err = handle_inbound(&buf[..n], peer).unwrap_err();
        assert_eq!(err, CodecError::BadCrc);
    }

    #[test]
    fn handle_inbound_truncated() {
        let peer: SocketAddr = "127.0.0.1:0".parse().unwrap();
        let err = handle_inbound(&[MAGIC_V2, 9], peer).unwrap_err();
        assert_eq!(err, CodecError::Truncated);
    }

    #[test]
    fn current_timestamp_is_monotone_within_a_run() {
        let t1 = current_timestamp();
        thread::sleep(Duration::from_millis(2));
        let t2 = current_timestamp();
        // Either seconds incremented, or fraction did.
        assert!(t2.seconds > t1.seconds
            || (t2.seconds == t1.seconds && t2.fraction > t1.fraction));
    }

    #[test]
    fn args_default_ports_for_vehicle_mode() {
        let argv = ["falcon-hello", "--mode", "vehicle"].iter().map(|s| s.to_string());
        let args = Args::parse(argv).expect("parse");
        assert_eq!(args.mode, Mode::Vehicle);
        assert_eq!(args.bind.port(), DEFAULT_PORT + 1);
        assert_eq!(args.remote.port(), DEFAULT_PORT);
    }

    #[test]
    fn args_default_ports_for_gcs_mode() {
        let argv = ["falcon-hello", "--mode", "gcs"].iter().map(|s| s.to_string());
        let args = Args::parse(argv).expect("parse");
        assert_eq!(args.mode, Mode::Gcs);
        assert_eq!(args.bind.port(), DEFAULT_PORT);
        assert_eq!(args.remote.port(), DEFAULT_PORT + 1);
    }

    #[test]
    fn args_rejects_unknown_mode() {
        let argv = ["falcon-hello", "--mode", "spy"].iter().map(|s| s.to_string());
        let err = Args::parse(argv).unwrap_err();
        assert!(err.contains("unknown --mode"));
    }

    #[test]
    fn args_rejects_missing_mode() {
        let argv = ["falcon-hello"].iter().map(|s| s.to_string());
        let err = Args::parse(argv).unwrap_err();
        assert!(err.contains("--mode is required"));
    }
}
