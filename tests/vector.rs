use ft_kalman::vector::*;

#[test]
fn vector_operations()
{
    let a = Vector3::new(1.0, 2.0, 3.0);
    let b = Vector3::new(4.0, 5.0, 6.0);

    assert_eq!(a + b, Vector3::new(5.0, 7.0, 9.0));
    assert_eq!(b - a, Vector3::new(3.0, 3.0, 3.0));
    assert_eq!(a * 2.0, Vector3::new(2.0, 4.0, 6.0));
    assert_eq!(a.dot(b), 32.0);
    assert_eq!(a.cross(b), Vector3::new(-3.0, 6.0, -3.0));
    assert!((Vector3::new(3.0, 4.0, 0.0).norm() - 5.0).abs() < 1e-12);
    assert!(Vector3::new(0.0, 0.0, 0.0).normalized().is_none());
    assert!(Vector3::new(f64::NAN, 0.0, 0.0).normalized().is_none());
    assert!((a.normalized().unwrap().norm() - 1.0).abs() < 1e-12);
    assert_eq!(Vector3::from_array([1.0, 2.0, 3.0]).to_array(), [1.0, 2.0, 3.0]);
}

#[test]
fn forward_axis_is_the_first_column_of_the_rotation()
{
    for e in [[0.0, 0.0, 0.0], [0.3, -0.2, 1.1], [-1.0, 0.7, -2.5], [0.0, 1.5, 3.0]]
    {
        let r = rotation_matrix(e).unwrap();
        let f = forward_axis(e);

        assert!((r.data[0][0] - f.x).abs() < 1e-12);
        assert!((r.data[1][0] - f.y).abs() < 1e-12);
        assert!((r.data[2][0] - f.z).abs() < 1e-12);
        assert!((f.norm() - 1.0).abs() < 1e-12);
    }
    let f = forward_axis([0.0, 0.0, std::f64::consts::FRAC_PI_2]);
    assert!(f.x.abs() < 1e-12 && (f.y - 1.0).abs() < 1e-12); // yaw 90°: forward is +Y
}

#[test]
fn basis_is_orthonormal_even_when_looking_up()
{
    for f in [Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0), Vector3::new(0.0, 0.0, -1.0), Vector3::new(0.6, 0.0, 0.8)]
    {
        let (a, b) = orthonormal_basis(f);

        assert!((a.norm() - 1.0).abs() < 1e-9 && (b.norm() - 1.0).abs() < 1e-9);
        assert!(a.dot(b).abs() < 1e-9 && a.dot(f).abs() < 1e-9 && b.dot(f).abs() < 1e-9);
    }
}
