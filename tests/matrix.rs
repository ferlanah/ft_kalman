use ft_kalman::matrix::Matrix;

fn m(rows: Vec<Vec<f64>>) -> Matrix<f64>
{
    Matrix { data: rows }
}

#[test]
fn add_works_and_does_not_modify_operands()
{
    let a = m(vec![vec![1.0, 2.0], vec![3.0, 4.0]]);
    let b = m(vec![vec![10.0, 20.0], vec![30.0, 40.0]]);
    let c = a.add(&b).unwrap();

    assert_eq!(c.data, vec![vec![11.0, 22.0], vec![33.0, 44.0]]);
    assert_eq!(a.data, vec![vec![1.0, 2.0], vec![3.0, 4.0]]);
}

#[test]
fn sub_works_and_does_not_modify_operands()
{
    let a = m(vec![vec![5.0, 5.0]]);
    let b = m(vec![vec![1.0, 2.0]]);

    assert_eq!(a.sub(&b).unwrap().data, vec![vec![4.0, 3.0]]);
    assert_eq!(a.data, vec![vec![5.0, 5.0]]);
}

#[test]
fn add_sub_reject_bad_shapes_without_panic()
{
    let a = m(vec![vec![1.0, 2.0]]);
    let rows = m(vec![vec![1.0, 2.0], vec![3.0, 4.0]]);
    let cols = m(vec![vec![1.0, 2.0, 3.0]]);
    let empty = m(vec![]);
    let ragged = m(vec![vec![1.0, 2.0], vec![3.0]]);

    for other in [&rows, &cols, &empty]
    {
        assert!(a.add(other).is_err());
        assert!(a.sub(other).is_err());
        assert!(other.add(&a).is_err());
        assert!(other.sub(&a).is_err());
    }
    assert!(ragged.add(&ragged).is_err());
    assert!(ragged.sub(&ragged).is_err());
}

#[test]
fn mul_works()
{
    let a = m(vec![vec![1.0, 2.0, 3.0], vec![4.0, 5.0, 6.0]]);
    let b = m(vec![vec![7.0, 8.0], vec![9.0, 10.0], vec![11.0, 12.0]]);

    assert_eq!(a.mul(&b).unwrap().data, vec![vec![58.0, 64.0], vec![139.0, 154.0]]);
}

#[test]
fn mul_rejects_bad_shapes_without_panic()
{
    let a = m(vec![vec![1.0, 2.0]]);
    let b = m(vec![vec![1.0, 2.0]]);

    assert!(a.mul(&b).is_err());
    assert!(m(vec![]).mul(&a).is_err());
    assert!(a.mul(&m(vec![])).is_err());
    assert!(m(vec![vec![1.0, 2.0], vec![3.0]]).mul(&a).is_err());
}

#[test]
fn transpose_works()
{
    let a = m(vec![vec![1.0, 2.0, 3.0], vec![4.0, 5.0, 6.0]]);

    assert_eq!(a.transpose().unwrap().data, vec![vec![1.0, 4.0], vec![2.0, 5.0], vec![3.0, 6.0]]);
    assert!(m(vec![vec![1.0], vec![1.0, 2.0]]).transpose().is_err());
    assert!(m(vec![]).transpose().unwrap().data.is_empty());
}

#[test]
fn identity_works()
{
    let a = m(vec![vec![9.0, 9.0, 9.0], vec![9.0, 9.0, 9.0], vec![9.0, 9.0, 9.0]]);

    assert_eq!(a.identity().data, vec![vec![1.0, 0.0, 0.0], vec![0.0, 1.0, 0.0], vec![0.0, 0.0, 1.0]]);
}

#[test]
fn inverse_works()
{
    let a = m(vec![vec![4.0, 7.0], vec![2.0, 6.0]]);
    let inv = a.inverse().unwrap();
    let prod = a.mul(&inv).unwrap();

    for i in 0..2
    {
        for j in 0..2
        {
            let expected = if i == j { 1.0 } else { 0.0 };
            assert!((prod.data[i][j] - expected).abs() < 1e-12);
        }
    }
}

#[test]
fn inverse_needs_a_row_swap()
{
    let a = m(vec![vec![0.0, 1.0], vec![1.0, 0.0]]);

    assert_eq!(a.inverse().unwrap().data, vec![vec![0.0, 1.0], vec![1.0, 0.0]]);
}

#[test]
fn inverse_errors_without_panic()
{
    assert!(m(vec![vec![1.0, 2.0], vec![2.0, 4.0]]).inverse().is_err()); // singular
    assert!(m(vec![vec![0.0]]).inverse().is_err());
    assert!(m(vec![]).inverse().is_err());
    assert!(m(vec![vec![1.0, 2.0, 3.0], vec![4.0, 5.0, 6.0]]).inverse().is_err()); // not square
    assert!(m(vec![vec![1.0, 2.0], vec![3.0]]).inverse().is_err());
}

#[test]
fn constructors()
{
    assert!(Matrix::<f64>::from_rows(vec![]).is_err());
    assert!(Matrix::<f64>::from_rows(vec![vec![]]).is_err());
    assert!(Matrix::<f64>::from_rows(vec![vec![1.0], vec![1.0, 2.0]]).is_err());
    let z = Matrix::<f64>::zeros(2, 3);
    assert_eq!((z.rows(), z.cols()), (2, 3));
    assert!(z.is_finite());
    assert!(!m(vec![vec![f64::NAN]]).is_finite());
    assert_eq!(Matrix::column(&[1.0, 2.0]).data, vec![vec![1.0], vec![2.0]]);
}
