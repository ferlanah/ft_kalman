//! The vehicle tracker: a Kalman filter fusing the accelerometer (control input),
//! the GPS (position measurement) and the gyroscope (direction of the velocity).
//!
//! State: x = [px py pz vx vy vz]
//!   predict (every 10 ms): p += v dt, v += a dt               (a = accelerometer, XYZ frame)
//!   update  (every 3 s)  : GPS measures p
//!   update  (every 10 ms): the vehicle only moves along its longitudinal axis, so the velocity
//!                          has no component orthogonal to the measured direction.
//! The noise levels are measured on the fly, so the filter keeps working with more noise
//! than the default one.

use crate::kalman::KalmanFilter;
use crate::matrix::Matrix;
use crate::noise::{AdaptiveVariance, NoiseEstimator};
use crate::simulator::Frame;
use crate::utils::kmh_to_ms;
use crate::vector::{forward_axis, orthonormal_basis, Vector3};

/// Above this 1-sigma position uncertainty the estimate can no longer be trusted (5 m is the limit).
pub const MAX_POSITION_SIGMA: f64 = 5.0;

const PRIOR_ACC_SIGMA: f64 = 0.01;
const PRIOR_GYRO_SIGMA: f64 = 0.01;
const PRIOR_GPS_SIGMA: f64 = 1.0;
const MIN_SIGMA: f64 = 1e-5;
const MIN_SPEED: f64 = 0.05;
const MIN_GPS_SIGMA: f64 = 1e-3;
const INITIAL_POSITION_VAR: f64 = 1e-4;
const INITIAL_SPEED_VAR: f64 = 1e-6;

pub struct Estimate
{
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub position_sigma: [f64; 3],
}

pub struct Tracker
{
    kf: KalmanFilter,
    last_time: f64,
    last_acceleration: [f64; 3],
    acc_noise: [NoiseEstimator; 3],
    gyro_noise: [NoiseEstimator; 3],
    gps_noise: AdaptiveVariance,
}

fn make_f(dt: f64) -> Matrix<f64>
{
    let mut f = Matrix::<f64>::zeros(6, 6);

    for i in 0..6
    {
        f.data[i][i] = 1.0;
    }
    for i in 0..3
    {
        f.data[i][i + 3] = dt;
    }
    f
}

fn make_q(dt: f64, sigma_acc: f64) -> Matrix<f64>
{
    let mut q = Matrix::<f64>::zeros(6, 6);
    let var = sigma_acc * sigma_acc;

    // The stream integrates like this: p(k) = p(k-1) + v(k-1) dt, v(k) = v(k-1) + a(k) dt.
    // The noise of a(k) therefore only enters the velocity at step k (and the position afterwards,
    // through F).
    for i in 0..3
    {
        q.data[i + 3][i + 3] = var * dt * dt;
    }
    q
}

fn make_h_gps() -> Matrix<f64>
{
    let mut h = Matrix::<f64>::zeros(3, 6);

    for i in 0..3
    {
        h.data[i][i] = 1.0;
    }
    h
}

fn rms(values: [f64; 3]) -> f64
{
    ((values[0] * values[0] + values[1] * values[1] + values[2] * values[2]) / 3.0).sqrt()
}

impl Tracker
{
    /// Initialise from the first frame: true position, true speed (km/h), direction.
    pub fn new(first: &Frame) -> Result<Tracker, String>
    {
        let position = first.true_position.ok_or("First frame has no true position")?;
        let speed = kmh_to_ms(first.speed_kmh.ok_or("First frame has no speed")?);
        let direction = first.direction.ok_or("First frame has no direction")?;

        if !speed.is_finite() || position.iter().any(|v| !v.is_finite()) || direction.iter().any(|v| !v.is_finite())
        {
            return Err("First frame contains invalid values".to_string());
        }

        let f = forward_axis(direction);
        let v = f * speed;
        let (e1, e2) = orthonormal_basis(f);
        let sv = (speed.abs().max(1.0) * PRIOR_GYRO_SIGMA).powi(2);

        let mut p = Matrix::<f64>::zeros(6, 6);
        for i in 0..3
        {
            p.data[i][i] = INITIAL_POSITION_VAR;
        }
        let (fa, a, b) = (f.to_array(), e1.to_array(), e2.to_array());
        for i in 0..3
        {
            for j in 0..3
            {
                p.data[3 + i][3 + j] = sv * (a[i] * a[j] + b[i] * b[j]) + INITIAL_SPEED_VAR * fa[i] * fa[j];
            }
        }

        let x = Matrix::column(&[position[0], position[1], position[2], v.x, v.y, v.z]);
        let kf = KalmanFilter::new(
            x,
            p,
            make_f(0.01),
            make_h_gps(),
            make_q(0.01, PRIOR_ACC_SIGMA),
            Matrix::<f64>::zeros(3, 3),
        );

        let acc = || NoiseEstimator::new(PRIOR_ACC_SIGMA, 50.0, 3000.0, MIN_SIGMA, false);
        let gyro = || NoiseEstimator::new(PRIOR_GYRO_SIGMA, 50.0, 3000.0, MIN_SIGMA, true);
        let mut tracker = Tracker
        {
            kf,
            last_time: first.time,
            last_acceleration: first.acceleration.unwrap_or([0.0; 3]),
            acc_noise: [acc(), acc(), acc()],
            gyro_noise: [gyro(), gyro(), gyro()],
            gps_noise: AdaptiveVariance::new(PRIOR_GPS_SIGMA, 6.0, 150.0, MIN_GPS_SIGMA),
        };
        tracker.feed_noise(first.acceleration, first.direction);
        tracker.kf.check_finite()?;
        Ok(tracker)
    }

    fn feed_noise(&mut self, acceleration: Option<[f64; 3]>, direction: Option<[f64; 3]>)
    {
        if let Some(a) = acceleration
        {
            for i in 0..3
            {
                self.acc_noise[i].push(a[i]);
            }
        }
        if let Some(d) = direction
        {
            for i in 0..3
            {
                self.gyro_noise[i].push(d[i]);
            }
        }
    }

    pub fn acceleration_sigma(&self) -> f64
    {
        rms([self.acc_noise[0].sigma(), self.acc_noise[1].sigma(), self.acc_noise[2].sigma()])
    }

    pub fn gyroscope_sigma(&self) -> f64
    {
        rms([self.gyro_noise[0].sigma(), self.gyro_noise[1].sigma(), self.gyro_noise[2].sigma()])
    }

    pub fn gps_sigma(&self) -> f64
    {
        self.gps_noise.sigma()
    }

    pub fn velocity(&self) -> Vector3
    {
        Vector3::new(self.kf.x.data[3][0], self.kf.x.data[4][0], self.kf.x.data[5][0])
    }

    pub fn position(&self) -> [f64; 3]
    {
        [self.kf.x.data[0][0], self.kf.x.data[1][0], self.kf.x.data[2][0]]
    }

    pub fn position_sigma(&self) -> [f64; 3]
    {
        let s = |i: usize| self.kf.P.data[i][i].max(0.0).sqrt();

        [s(0), s(1), s(2)]
    }

    pub fn estimate(&self) -> Estimate
    {
        Estimate { position: self.position(), velocity: self.velocity().to_array(), position_sigma: self.position_sigma() }
    }

    /// Process one frame. An error means the filter cannot be trusted any more:
    /// the caller must stop sending estimates.
    pub fn step(&mut self, frame: &Frame) -> Result<(), String>
    {
        let dt = frame.time - self.last_time;

        if !frame.time.is_finite() || dt <= 0.0
        {
            // duplicated or out of order frame: nothing new to integrate
            return Ok(());
        }
        self.last_time = frame.time;
        self.feed_noise(frame.acceleration, frame.direction);

        let acc = match frame.acceleration
        {
            Some(a) => { self.last_acceleration = a; a }
            None => self.last_acceleration,
        };

        self.kf.F = make_f(dt);
        self.kf.Q = make_q(dt, self.acceleration_sigma());
        let bu = Matrix::column(&[0.0, 0.0, 0.0, dt * acc[0], dt * acc[1], dt * acc[2]]);
        self.kf.predict_with_control(&bu)?;

        if let Some(gps) = frame.gps
        {
            self.update_gps(gps)?;
        }
        if let Some(direction) = frame.direction
        {
            self.update_direction(direction)?;
        }

        self.kf.symmetrize()?;
        self.kf.check_finite()?;
        self.check_uncertainty()
    }

    fn check_uncertainty(&self) -> Result<(), String>
    {
        if (0..6).any(|i| self.kf.P.data[i][i] < 0.0)
        {
            return Err("Kalman filter diverged (negative variance)".to_string());
        }
        let sigma = self.position_sigma();

        if sigma.iter().any(|s| *s > MAX_POSITION_SIGMA)
        {
            return Err(format!("Position uncertainty ({:.2} m) is too high, estimate is not reliable", sigma.iter().cloned().fold(0.0, f64::max)));
        }
        Ok(())
    }

    fn update_gps(&mut self, gps: [f64; 3]) -> Result<(), String>
    {
        // measure the GPS noise from the innovation: E[y²] = P_pp + sigma²
        let x = self.position();
        for i in 0..3
        {
            let y = gps[i] - x[i];
            self.gps_noise.update(y * y - self.kf.P.data[i][i]);
        }

        let var = self.gps_noise.variance();
        let mut r = Matrix::<f64>::zeros(3, 3);
        for i in 0..3
        {
            r.data[i][i] = var;
        }
        let z = Matrix::column(&gps);
        self.kf.update_with(&z, &make_h_gps(), &r)
    }

    fn update_direction(&mut self, direction: [f64; 3]) -> Result<(), String>
    {
        let measured = forward_axis(direction);
        let velocity = self.velocity();
        let speed = velocity.norm();
        let estimated = match velocity.normalized()
        {
            Some(u) if speed >= MIN_SPEED => u,
            _ => return Ok(()), // not moving: the heading says nothing about the velocity
        };

        // Basis (e1, e2) orthogonal to the *estimated* direction of motion. The measured heading
        // seen in this basis is a pure angle error, so the correction is purely transverse:
        // z = speed * (e1.f, e2.f), measurement model z = B^T v + noise (B^T v = 0 for the estimate).
        let (e1, e2) = orthonormal_basis(estimated);
        let (a, b) = (e1.to_array(), e2.to_array());
        let mut h = Matrix::<f64>::zeros(2, 6);
        for i in 0..3
        {
            h.data[0][3 + i] = a[i];
            h.data[1][3 + i] = b[i];
        }
        let var = (speed * self.gyroscope_sigma()).powi(2).max(1e-12);
        let r = Matrix { data: vec![vec![var, 0.0], vec![0.0, var]] };
        let z = Matrix::column(&[speed * e1.dot(measured), speed * e2.dot(measured)]);

        self.kf.update_with(&z, &h, &r)?;

        // This update must only correct the heading of the velocity. Removing the orthogonal
        // part of a noisy velocity always shortens it a little (second order effect, which
        // adds up to a fake deceleration): give back the speed it had before.
        let after = self.velocity();
        if let Some(unit) = after.normalized()
        {
            let v = unit * speed;
            self.kf.x.data[3][0] = v.x;
            self.kf.x.data[4][0] = v.y;
            self.kf.x.data[5][0] = v.z;
        }
        Ok(())
    }
}
