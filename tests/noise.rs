use ft_kalman::noise::{AdaptiveVariance, Gaussian, NoiseEstimator};

#[test]
fn gaussian_has_the_right_statistics_and_is_reproducible()
{
    let mut g = Gaussian::new(1);
    let n = 200_000;
    let v: Vec<f64> = (0..n).map(|_| g.sample()).collect();
    let mean = v.iter().sum::<f64>() / n as f64;
    let var = v.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n as f64;

    assert!(mean.abs() < 0.01);
    assert!((var - 1.0).abs() < 0.02);
    assert!(v.iter().all(|x| x.is_finite()));
    let mut a = Gaussian::new(5);
    let mut b = Gaussian::new(5);
    assert_eq!(a.sample(), b.sample());
    let u = Gaussian::new(0).uniform();
    assert!(u > 0.0 && u < 1.0);
}

#[test]
fn estimator_measures_sigma_even_when_the_prior_is_wrong()
{
    for truth in [0.001, 0.01, 0.1, 1.0]
    {
        let mut g = Gaussian::new(3);
        let mut e = NoiseEstimator::new(0.01, 50.0, 3000.0, 1e-5, false);

        for k in 0..20_000
        {
            e.push(5.0 + (k as f64 * 0.001).sin() + g.sample_sigma(truth));
        }
        let ratio = e.sigma() / truth;
        assert!(ratio > 0.8 && ratio < 1.25, "truth {} estimated {}", truth, e.sigma());
    }
}

#[test]
fn estimator_ignores_angle_wrapping_and_bad_values()
{
    let mut e = NoiseEstimator::new(0.01, 50.0, 3000.0, 1e-5, true);
    let mut g = Gaussian::new(9);

    for _ in 0..5000
    {
        e.push(std::f64::consts::PI - 0.001 + g.sample_sigma(0.01)); // crosses +-pi regularly
        e.push(f64::NAN);
    }
    assert!(e.sigma() > 0.008 && e.sigma() < 0.013, "{}", e.sigma());
}

#[test]
fn adaptive_variance_is_bounded_below()
{
    let mut v = AdaptiveVariance::new(1.0, 5.0, 100.0, 0.1);

    for _ in 0..1000
    {
        v.update(-5.0);
        v.update(f64::NAN);
    }
    assert!((v.sigma() - 0.1).abs() < 1e-9);
}
