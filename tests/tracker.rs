mod common;

use common::*;
use ft_kalman::simulator::Frame;
use ft_kalman::tracker::Tracker;
use ft_kalman::utils::distance;

fn run(seconds: f64, noise: &Noise, seed: u64) -> (f64, f64, Tracker)
{
    let traj = trajectory(seconds, noise, seed);
    let mut tracker = Tracker::new(&traj[0].0).unwrap();
    let (mut sum, mut max) = (0.0, 0.0f64);

    for (frame, truth) in &traj
    {
        tracker.step(frame).unwrap();
        let d = distance(tracker.position(), *truth);
        sum += d;
        max = max.max(d);
    }
    (sum / traj.len() as f64, max, tracker)
}

#[test]
fn default_noise_is_accurate()
{
    for seed in 1..=3
    {
        let (mean, max, t) = run(600.0, &DEFAULT_NOISE, seed);

        assert!(mean < 0.8 && max < 2.5, "seed {} mean {} max {}", seed, mean, max);
        // the filter measured the noise of the sensors
        assert!((t.acceleration_sigma() - 0.01).abs() < 0.002, "{}", t.acceleration_sigma());
        assert!((t.gyroscope_sigma() - 0.01).abs() < 0.002, "{}", t.gyroscope_sigma());
        assert!((t.gps_sigma() - 1.0).abs() < 0.3, "{}", t.gps_sigma());
    }
}

#[test]
fn more_noise_is_handled()
{
    for k in [2.0, 3.0]
    {
        let (mean, max, t) = run(600.0, &scaled(k), 11);

        assert!(max < 5.0, "noise x{}: mean {} max {}", k, mean, max);
        assert!((t.gps_sigma() / k - 1.0).abs() < 0.35, "gps sigma {}", t.gps_sigma());
        assert!((t.acceleration_sigma() / (0.01 * k) - 1.0).abs() < 0.2, "acc sigma {}", t.acceleration_sigma());
    }
}

#[test]
fn long_run_keeps_the_covariance_healthy()
{
    // 90 minutes: no divergence, no NaN, uncertainty stays small
    let (mean, max, t) = run(5400.0, &DEFAULT_NOISE, 21);

    assert!(max < 3.0 && mean < 1.0, "mean {} max {}", mean, max);
    assert!(t.position_sigma().iter().all(|s| s.is_finite() && *s < 1.0));
}

#[test]
fn works_with_much_less_noise_than_default()
{
    let (_, max, _) = run(300.0, &Noise { acc: 1e-4, gyro: 1e-4, gps: 0.01 }, 5);
    assert!(max < 0.5, "{}", max);
    let (_, max, _) = run(120.0, &Noise { acc: 0.0, gyro: 0.0, gps: 0.0 }, 5);
    assert!(max < 0.5, "{}", max);
}

#[test]
fn bad_first_frames_are_errors()
{
    let good = trajectory(1.0, &DEFAULT_NOISE, 1)[0].0.clone();

    assert!(Tracker::new(&good).is_ok());
    assert!(Tracker::new(&Frame { true_position: None, ..good.clone() }).is_err());
    assert!(Tracker::new(&Frame { speed_kmh: None, ..good.clone() }).is_err());
    assert!(Tracker::new(&Frame { direction: None, ..good.clone() }).is_err());
    assert!(Tracker::new(&Frame { speed_kmh: Some(f64::NAN), ..good.clone() }).is_err());
    assert!(Tracker::new(&Frame { true_position: Some([f64::INFINITY, 0.0, 0.0]), ..good.clone() }).is_err());
    assert!(Tracker::new(&Frame { acceleration: None, ..good }).is_ok());
}

#[test]
fn odd_frames_do_not_break_the_filter()
{
    let traj = trajectory(30.0, &DEFAULT_NOISE, 4);
    let mut tracker = Tracker::new(&traj[0].0).unwrap();

    for (k, (frame, truth)) in traj.iter().enumerate()
    {
        tracker.step(frame).unwrap();
        tracker.step(frame).unwrap(); // duplicated frame: ignored
        if k % 7 == 0
        {
            tracker.step(&Frame { time: frame.time - 1.0, ..frame.clone() }).unwrap(); // out of order
        }
        if k % 5 == 0
        {
            // frame without accelerometer nor gyroscope data
            let mut partial = traj[(k + 1).min(traj.len() - 1)].0.clone();
            partial.acceleration = None;
            partial.direction = None;
            partial.gps = None;
            tracker.step(&partial).unwrap();
        }
        assert!(distance(tracker.position(), *truth) < 3.0);
    }
}

#[test]
fn a_lost_filter_stops_cleanly_instead_of_lying()
{
    // no GPS and sensors 100 times noisier: the position uncertainty explodes
    let mut noise = scaled(1.0);
    noise.acc = 1.0;
    noise.gyro = 0.3;
    let mut traj = trajectory(3000.0, &noise, 8);
    for (frame, _) in traj.iter_mut()
    {
        frame.gps = None;
    }
    let mut tracker = Tracker::new(&traj[0].0).unwrap();
    let stopped = traj.iter().any(|(frame, _)| tracker.step(frame).is_err());

    assert!(stopped, "the filter should refuse to continue");
}

#[test]
fn nan_measurements_never_reach_the_estimate()
{
    let traj = trajectory(5.0, &DEFAULT_NOISE, 2);
    let mut tracker = Tracker::new(&traj[0].0).unwrap();

    for (frame, _) in &traj
    {
        let _ = tracker.step(frame);
    }
    let mut poisoned = traj[10].0.clone();
    poisoned.time = 100.0;
    poisoned.acceleration = Some([f64::NAN, 0.0, 0.0]);
    assert!(tracker.step(&poisoned).is_err());
}
