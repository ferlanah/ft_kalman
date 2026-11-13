//! End to end tests against the real imu-sensor-stream program of the subject
//! (skipped when it cannot be run on this machine).

use std::net::UdpSocket;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::thread;
use std::time::Duration;

fn sensor_path() -> Option<PathBuf>
{
    let name = match (std::env::consts::OS, std::env::consts::ARCH)
    {
        ("linux", "x86_64") => "imu-sensor-stream-linux",
        ("macos", _) => "imu-sensor-stream-macos",
        _ => return None,
    };
    let p = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(name);

    if !p.exists()
    {
        return None;
    }
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        let _ = std::fs::set_permissions(&p, std::fs::Permissions::from_mode(0o755));
    }
    Some(p)
}

fn free_port() -> u16
{
    UdpSocket::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port()
}

/// Start the stream; its output goes to a file (a pipe nobody reads would fill up and block it).
fn start_sensor(path: &PathBuf, extra: &[&str]) -> (Child, u16, PathBuf)
{
    let port = free_port();
    let log = std::env::temp_dir().join(format!("ft_kalman_sensor_{}_{}.log", std::process::id(), port));
    let file = std::fs::File::create(&log).expect("cannot create the log file");
    let child = Command::new(path)
        .args(extra)
        .args(["-p", &port.to_string()])
        .stdin(Stdio::null())
        .stdout(file.try_clone().unwrap())
        .stderr(file)
        .spawn()
        .expect("cannot start the sensor stream");

    thread::sleep(Duration::from_millis(400));
    (child, port, log)
}

struct Outcome
{
    code: Option<i32>,
    stdout: String,
    stderr: String,
    max_delta: f64,
}

/// Run ft_kalman against the real stream (started with `extra` options, --delta added).
fn scenario(extra: &[&str]) -> Option<Outcome>
{
    let path = sensor_path()?;
    let mut args = extra.to_vec();
    args.push("--delta");
    let (mut sensor, port, log) = start_sensor(&path, &args);
    let out = Command::new(env!("CARGO_BIN_EXE_ft_kalman")).args(["-p", &port.to_string(), "-m"]).output().unwrap();
    let _ = sensor.wait();
    let text = std::fs::read_to_string(&log).unwrap_or_default();
    let _ = std::fs::remove_file(&log);
    let mut max_delta: f64 = 0.0;

    for line in text.lines()
    {
        let v: Vec<f64> = line.split(',').filter_map(|p| p.split(':').nth(1)).filter_map(|n| n.trim().parse().ok()).collect();
        if v.len() == 3
        {
            max_delta = max_delta.max((v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt());
        }
    }
    Some(Outcome
    {
        code: out.status.code(),
        stdout: String::from_utf8_lossy(&out.stdout).to_string(),
        stderr: String::from_utf8_lossy(&out.stderr).to_string(),
        max_delta,
    })
}

fn check(o: &Outcome, limit: f64)
{
    assert_eq!(o.code, Some(0), "stderr: {} stdout: {}", o.stderr, o.stdout);
    assert!(o.max_delta > 0.0, "no delta received from the stream");
    assert!(o.max_delta < limit, "max delta {} m", o.max_delta);
    assert!(o.stdout.contains("Mean response time"));
}

#[test]
fn several_seeds()
{
    for seed in ["1", "42", "1234"]
    {
        if let Some(o) = scenario(&["-s", seed, "-d", "3"])
        {
            check(&o, 2.5);
        }
    }
}

#[test]
fn trajectory_longer_than_a_few_minutes()
{
    if let Some(o) = scenario(&["-s", "7", "-d", "15"])
    {
        check(&o, 2.5);
    }
}

#[test]
fn bonus_more_noise()
{
    for (noise, limit) in [("2", 3.5), ("3", 4.5)]
    {
        if let Some(o) = scenario(&["-s", "5", "-d", "5", "-n", noise])
        {
            check(&o, limit);
        }
    }
}

#[test]
fn entropy_seed_and_manual_sigmas()
{
    if let Some(o) = scenario(&["-e", "-d", "2", "-a", "0.005", "-g", "0.5"])
    {
        check(&o, 2.5);
    }
}

#[test]
fn debug_mode_without_gps_does_not_crash()
{
    // true information is given but no GPS: short run, the filter dead-reckons
    if let Some(path) = sensor_path()
    {
        let (mut sensor, port, log) = start_sensor(&path, &["-s", "3", "-d", "1", "--debug"]);
        let out = Command::new(env!("CARGO_BIN_EXE_ft_kalman")).args(["-p", &port.to_string(), "-d"]).output().unwrap();
        let _ = sensor.wait();
        let _ = std::fs::remove_file(&log);

        assert_eq!(out.status.code(), Some(0), "{}", String::from_utf8_lossy(&out.stderr));
        assert!(String::from_utf8_lossy(&out.stdout).contains("Delta vs true position"));
    }
}

#[test]
fn the_stream_disappearing_stops_the_filter_cleanly()
{
    if let Some(path) = sensor_path()
    {
        let (mut sensor, port, log) = start_sensor(&path, &["-s", "3", "-d", "5"]);
        let client = Command::new(env!("CARGO_BIN_EXE_ft_kalman")).args(["-p", &port.to_string()]).stdout(Stdio::piped()).stderr(Stdio::piped()).spawn().unwrap();

        thread::sleep(Duration::from_millis(500));
        let _ = sensor.kill();
        let _ = sensor.wait();
        let _ = std::fs::remove_file(&log);
        let out = client.wait_with_output().unwrap();

        assert_eq!(out.status.code(), Some(1));
        let err = String::from_utf8_lossy(&out.stderr);
        assert!(err.contains("ft_kalman:") && !err.contains("panicked"), "{}", err);
    }
}

#[test]
fn extreme_noise_never_panics()
{
    // far beyond what any filter can follow: the program may finish (0) or stop cleanly (1), never crash
    if let Some(o) = scenario(&["-s", "4", "-d", "3", "-n", "10"])
    {
        assert!(matches!(o.code, Some(0) | Some(1)), "code {:?} stderr: {}", o.code, o.stderr);
        assert!(!o.stderr.contains("panicked"), "{}", o.stderr);
    }
}

/// 90 minutes against the real stream (about 5 real minutes): `make test-long`.
#[test]
#[ignore]
fn ninety_minutes_default_noise()
{
    if let Some(o) = scenario(&["-s", "1", "-d", "90"])
    {
        check(&o, 5.0);
    }
}

#[test]
#[ignore]
fn ninety_minutes_triple_noise()
{
    if let Some(o) = scenario(&["-s", "3", "-d", "90", "-n", "3"])
    {
        check(&o, 5.0);
    }
}
