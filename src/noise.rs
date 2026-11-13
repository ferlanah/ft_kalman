//! Noise tools: a gaussian generator (used to simulate sensors in the tests) and
//! estimators able to measure the noise level of a sensor while the filter runs.

use std::f64::consts::PI;

/// Deterministic gaussian white noise generator (splitmix64 + Box-Muller).
pub struct Gaussian
{
    state: u64,
    spare: Option<f64>,
}

impl Gaussian
{
    pub fn new(seed: u64) -> Self
    {
        Gaussian { state: seed.wrapping_add(0x9E37_79B9_7F4A_7C15), spare: None }
    }

    fn next_u64(&mut self) -> u64
    {
        self.state = self.state.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    /// Uniform number in ]0, 1[.
    pub fn uniform(&mut self) -> f64
    {
        ((self.next_u64() >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    /// Standard normal sample (mean 0, sigma 1).
    pub fn sample(&mut self) -> f64
    {
        if let Some(s) = self.spare.take()
        {
            return s;
        }
        let u1 = self.uniform();
        let u2 = self.uniform();
        let r = (-2.0 * u1.ln()).sqrt();

        self.spare = Some(r * (2.0 * PI * u2).sin());
        r * (2.0 * PI * u2).cos()
    }

    /// Normal sample with the given sigma.
    pub fn sample_sigma(&mut self, sigma: f64) -> f64
    {
        self.sample() * sigma
    }
}

/// Exponentially weighted variance estimate, starting from a prior.
/// The weight of the prior fades as samples come; afterwards the estimate follows
/// a sliding window of about `window` samples. Samples are winsorized (limited to
/// 5 sigma) so that a few outliers cannot ruin the estimate, while a sensor much
/// noisier than the prior is still caught geometrically fast.
pub struct AdaptiveVariance
{
    var: f64,
    count: f64,
    prior_weight: f64,
    window: f64,
    min_var: f64,
}

impl AdaptiveVariance
{
    pub fn new(prior_sigma: f64, prior_weight: f64, window: f64, min_sigma: f64) -> Self
    {
        let min_var = min_sigma * min_sigma;

        AdaptiveVariance
        {
            var: (prior_sigma * prior_sigma).max(min_var),
            count: 0.0,
            prior_weight: prior_weight.max(1.0),
            window: window.max(1.0),
            min_var,
        }
    }

    /// Add a squared error sample (may be slightly negative when a known part was subtracted).
    pub fn update(&mut self, squared: f64)
    {
        if !squared.is_finite()
        {
            return;
        }
        let clipped = squared.min(25.0 * self.var);
        let alpha = (1.0 / (self.prior_weight + self.count)).max(1.0 / self.window);

        self.var += alpha * (clipped - self.var);
        self.var = self.var.max(self.min_var);
        self.count += 1.0;
    }

    pub fn variance(&self) -> f64
    {
        self.var
    }

    pub fn sigma(&self) -> f64
    {
        self.var.sqrt()
    }
}

/// Measures the white noise of a slowly varying signal from the differences of
/// successive samples: var(x[k] - x[k-1]) = 2 sigma^2 when the true signal barely moves.
pub struct NoiseEstimator
{
    var: AdaptiveVariance,
    last: Option<f64>,
    angle: bool,
}

impl NoiseEstimator
{
    pub fn new(prior_sigma: f64, prior_weight: f64, window: f64, min_sigma: f64, angle: bool) -> Self
    {
        NoiseEstimator
        {
            var: AdaptiveVariance::new(prior_sigma, prior_weight, window, min_sigma),
            last: None,
            angle,
        }
    }

    pub fn push(&mut self, sample: f64)
    {
        if !sample.is_finite()
        {
            return;
        }
        if let Some(last) = self.last
        {
            let mut d = sample - last;

            if self.angle
            {
                d = (d + PI).rem_euclid(2.0 * PI) - PI;
            }
            self.var.update(d * d / 2.0);
        }
        self.last = Some(sample);
    }

    pub fn sigma(&self) -> f64
    {
        self.var.sigma()
    }
}
