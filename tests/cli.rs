mod common;

use common::*;
use ft_kalman::utils::{distance, format_position};
use std::net::{SocketAddr, UdpSocket};
use std::process::{Command, Stdio};
use std::thread;
use std::time::Duration;

fn bin() -> Command
{
    Command::new(env!("CARGO_BIN_EXE_ft_kalman"))
}

#[test]
fn help_and_usage_errors()
{
    let out = bin().arg("--help").output().unwrap();
    assert!(out.status.success());
    assert!(String::from_utf8_lossy(&out.stdout).contains("USAGE"));

    for bad in [vec!["--nope"], vec!["-p"], vec!["-p", "abc"], vec!["-p", "0"]]
    {
        let out = bin().args(&bad).output().unwrap();
        assert_eq!(out.status.code(), Some(2), "{:?}", bad);
        assert!(!String::from_utf8_lossy(&out.stderr).contains("panicked"));
    }
}

#[test]
fn no_server_is_a_clean_failure()
{
    let port = UdpSocket::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port();
    let out = bin().args(["-p", &port.to_string()]).output().unwrap();

    assert_eq!(out.status.code(), Some(1));
    let err = String::from_utf8_lossy(&out.stderr);
    assert!(err.contains("ft_kalman:") && !err.contains("panicked"), "{}", err);
}

fn send(sock: &UdpSocket, to: SocketAddr, text: &str)
{
    sock.send_to(text.as_bytes(), to).unwrap();
}

/// Plays a synthetic 60 s trajectory like the real stream does; returns the max estimation error.
fn play(sock: UdpSocket, truth_debug: bool) -> f64
{
    let mut buf = [0u8; 256];
    let (_, client) = sock.recv_from(&mut buf).unwrap();
    sock.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
    let traj = trajectory(60.0, &DEFAULT_NOISE, 77);
    let mut worst: f64 = 0.0;

    send(&sock, client, "Trajectory Generation. . .\n");
    send(&sock, client, "Trajectory Generated!\nSending Info. . .\n");
    for (frame, truth) in &traj
    {
        let h = |t: f64| format!("[{:02}:{:02}:{:06.3}]", (t / 3600.0) as u32, ((t % 3600.0) / 60.0) as u32, t % 60.0);
        let t = frame.time;
        send(&sock, client, "MSG_START");
        if let Some(p) = frame.true_position
        {
            send(&sock, client, &format!("{}TRUE POSITION\n{}\n{}\n{}\n", h(t), p[0], p[1], p[2]));
            send(&sock, client, &format!("{}SPEED\n{}\n", h(t), frame.speed_kmh.unwrap()));
        }
        else if truth_debug
        {
            send(&sock, client, &format!("{}TRUE POSITION\n{}\n{}\n{}\n", h(t), truth[0], truth[1], truth[2]));
        }
        if let Some(p) = frame.gps
        {
            send(&sock, client, &format!("{}POSITION\n{}\n{}\n{}\n", h(t), p[0], p[1], p[2]));
        }
        let a = frame.acceleration.unwrap();
        send(&sock, client, &format!("{}ACCELERATION\n{}\n{}\n{}\n", h(t), a[0], a[1], a[2]));
        let d = frame.direction.unwrap();
        send(&sock, client, &format!("{}DIRECTION\n{}\n{}\n{}\n", h(t), d[0], d[1], d[2]));
        send(&sock, client, "MSG_END");

        let n = sock.recv(&mut buf).expect("no estimate received");
        let text = String::from_utf8_lossy(&buf[..n]).to_string();
        let v: Vec<f64> = text.split(' ').map(|x| x.parse().expect("bad number")).collect();
        assert_eq!(v.len(), 3, "format must be 'X Y Z': {}", text);
        assert_eq!(text, format_position([v[0], v[1], v[2]]));
        worst = worst.max(distance([v[0], v[1], v[2]], *truth));
    }
    send(&sock, client, "GOODBYE.");
    worst
}

#[test]
fn full_run_against_a_synthetic_stream()
{
    let sock = UdpSocket::bind("127.0.0.1:0").unwrap();
    let port = sock.local_addr().unwrap().port();
    let server = thread::spawn(move || play(sock, true));
    let html = std::env::temp_dir().join(format!("ft_kalman_test_{}.html", port));

    let out = bin()
        .args(["-p", &port.to_string(), "-m", "-d", "-v", html.to_str().unwrap()])
        .stdin(Stdio::null())
        .output()
        .unwrap();
    let stdout = String::from_utf8_lossy(&out.stdout);

    assert!(out.status.success(), "{} / {}", stdout, String::from_utf8_lossy(&out.stderr));
    assert!(server.join().unwrap() < 3.0);
    assert!(stdout.contains("Mean response time"), "{}", stdout);
    assert!(stdout.contains("Delta vs true position"), "{}", stdout);
    let page = std::fs::read_to_string(&html).unwrap();
    assert!(page.contains("<canvas") && !page.contains("__EST__"));
    let _ = std::fs::remove_file(&html);
}

#[test]
fn unwritable_visual_file_is_an_error_not_a_panic()
{
    let sock = UdpSocket::bind("127.0.0.1:0").unwrap();
    let port = sock.local_addr().unwrap().port();
    let server = thread::spawn(move || play(sock, false));
    let out = bin().args(["-p", &port.to_string(), "-v", "/nonexistent_dir/x.html"]).output().unwrap();

    assert_eq!(out.status.code(), Some(1));
    assert!(!String::from_utf8_lossy(&out.stderr).contains("panicked"));
    let _ = server.join();
}
