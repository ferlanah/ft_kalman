use ft_kalman::kalman::KalmanFilter;
use ft_kalman::matrix::Matrix;
use ft_kalman::noise::Gaussian;

fn m(rows: Vec<Vec<f64>>) -> Matrix<f64>
{
    Matrix { data: rows }
}

/// 1D constant velocity filter: x = [position, velocity], we measure the position.
fn filter_1d(dt: f64) -> KalmanFilter
{
    KalmanFilter::new(
        Matrix::column(&[0.0, 0.0]),
        m(vec![vec![100.0, 0.0], vec![0.0, 100.0]]),
        m(vec![vec![1.0, dt], vec![0.0, 1.0]]),
        m(vec![vec![1.0, 0.0]]),
        m(vec![vec![1e-6, 0.0], vec![0.0, 1e-6]]),
        m(vec![vec![0.25]]),
    )
}

#[test]
fn predict_moves_the_state()
{
    let mut kf = filter_1d(1.0);
    kf.x = Matrix::column(&[10.0, 2.0]);
    kf.predict().unwrap();

    assert_eq!(kf.x.data, vec![vec![12.0], vec![2.0]]);
    assert!(kf.P.data[0][0] > 100.0);
}

#[test]
fn predict_with_control_adds_the_input()
{
    let mut kf = filter_1d(1.0);
    kf.x = Matrix::column(&[10.0, 2.0]);
    kf.predict_with_control(&Matrix::column(&[0.5, 1.0])).unwrap();

    assert_eq!(kf.x.data, vec![vec![12.5], vec![3.0]]);
}

#[test]
fn converges_on_a_constant_velocity_target()
{
    let mut kf = filter_1d(1.0);
    let mut rng = Gaussian::new(7);

    for k in 1..=200
    {
        let truth = 3.0 * k as f64 + 5.0;
        kf.predict().unwrap();
        kf.update(&Matrix::column(&[truth + rng.sample_sigma(0.5)])).unwrap();
    }
    assert!((kf.x.data[0][0] - 605.0).abs() < 1.0);
    assert!((kf.x.data[1][0] - 3.0).abs() < 0.1);
    assert!(kf.P.data[0][0] < 0.25); // more certain than a single measurement
}

#[test]
fn update_with_leaves_h_and_r_untouched_and_keeps_p_symmetric()
{
    let mut kf = filter_1d(1.0);
    let h_before = kf.H.clone();
    let r_before = kf.R.clone();
    kf.update_with(&Matrix::column(&[1.0]), &m(vec![vec![0.0, 1.0]]), &m(vec![vec![0.1]])).unwrap();

    assert_eq!(kf.H, h_before);
    assert_eq!(kf.R, r_before);
    assert!((kf.P.data[0][1] - kf.P.data[1][0]).abs() < 1e-12);
}

#[test]
fn bad_dimensions_are_errors_not_panics()
{
    let mut kf = filter_1d(1.0);

    assert!(kf.update(&Matrix::column(&[1.0, 2.0])).is_err());
    assert!(kf.predict_with_control(&Matrix::column(&[1.0])).is_err());
    kf.R = m(vec![vec![0.0]]);
    kf.P = m(vec![vec![0.0, 0.0], vec![0.0, 0.0]]);
    assert!(kf.update(&Matrix::column(&[1.0])).is_err()); // singular innovation covariance
}

#[test]
fn non_finite_values_are_detected()
{
    let mut kf = filter_1d(1.0);

    assert!(kf.check_finite().is_ok());
    kf.x.data[0][0] = f64::NAN;
    assert!(kf.check_finite().is_err());
}

#[test]
fn symmetrize()
{
    let mut kf = filter_1d(1.0);
    kf.P = m(vec![vec![1.0, 2.0], vec![4.0, 1.0]]);
    kf.symmetrize().unwrap();

    assert_eq!(kf.P.data[0][1], 3.0);
    assert_eq!(kf.P.data[1][0], 3.0);
    kf.P = m(vec![vec![1.0, 2.0]]);
    assert!(kf.symmetrize().is_err());
}
