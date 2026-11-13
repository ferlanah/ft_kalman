#![allow(dead_code)]
//! Synthetic sensors used by the tests: same integration scheme and same noise
//! structure as imu-sensor-stream, without needing the program.

use ft_kalman::noise::Gaussian;
use ft_kalman::simulator::Frame;
use ft_kalman::vector::forward_axis;

pub struct Noise
{
    pub acc: f64,
    pub gyro: f64,
    pub gps: f64,
}

/// Default noise of the stream, as measured on its output.
pub const DEFAULT_NOISE: Noise = Noise { acc: 0.01, gyro: 0.01, gps: 1.0 };

pub fn scaled(k: f64) -> Noise
{
    Noise { acc: DEFAULT_NOISE.acc * k, gyro: DEFAULT_NOISE.gyro, gps: DEFAULT_NOISE.gps * k }
}

/// Frames (with the true position next to each) of a trajectory with turns, climbs and speed changes.
pub fn trajectory(seconds: f64, noise: &Noise, seed: u64) -> Vec<(Frame, [f64; 3])>
{
    let dt = 0.01;
    let n = (seconds / dt) as usize;
    let mut rng = Gaussian::new(seed);
    let mut out = Vec::with_capacity(n);
    let mut p = [-6.0, -4.0, 0.5];
    let mut prev_v = [0.0; 3];
    let mut yaw = 0.0;

    for k in 0..n
    {
        let t = k as f64 * dt;
        yaw += 0.02 * (t / 70.0).sin() * dt;
        let pitch = 0.05 * (t / 45.0).sin();
        let speed = 16.0 + 2.0 * (t / 90.0).sin();
        let f = forward_axis([0.0, pitch, yaw]);
        let v = [f.x * speed, f.y * speed, f.z * speed];

        let mut frame = Frame { time: t, ..Frame::default() };
        let a_true = if k == 0 { [0.0; 3] } else { [(v[0] - prev_v[0]) / dt, (v[1] - prev_v[1]) / dt, (v[2] - prev_v[2]) / dt] };
        frame.acceleration = Some([
            a_true[0] + rng.sample_sigma(noise.acc),
            a_true[1] + rng.sample_sigma(noise.acc),
            a_true[2] + rng.sample_sigma(noise.acc),
        ]);
        frame.direction = Some([
            rng.sample_sigma(noise.gyro),
            pitch + rng.sample_sigma(noise.gyro),
            yaw + rng.sample_sigma(noise.gyro),
        ]);
        if k == 0
        {
            frame.true_position = Some(p);
            frame.speed_kmh = Some(speed * 3.6);
        }
        else if k % 300 == 0
        {
            frame.gps = Some([
                p[0] + rng.sample_sigma(noise.gps),
                p[1] + rng.sample_sigma(noise.gps),
                p[2] + rng.sample_sigma(noise.gps),
            ]);
        }
        out.push((frame, p));
        // same scheme as the stream: position moves with the velocity of the previous step
        for i in 0..3
        {
            p[i] += v[i] * dt;
        }
        prev_v = v;
    }
    out
}
