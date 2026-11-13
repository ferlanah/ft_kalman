use crate::matrix::Matrix;
use std::ops::{Add, Mul, Sub};

/// Small 3D vector used for positions, velocities, accelerations.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Vector3
{
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3
{
    pub fn new(x: f64, y: f64, z: f64) -> Self
    {
        Vector3 { x, y, z }
    }

    pub fn from_array(a: [f64; 3]) -> Self
    {
        Vector3 { x: a[0], y: a[1], z: a[2] }
    }

    pub fn to_array(self) -> [f64; 3]
    {
        [self.x, self.y, self.z]
    }

    pub fn dot(self, o: Vector3) -> f64
    {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn cross(self, o: Vector3) -> Vector3
    {
        Vector3
        {
            x: self.y * o.z - self.z * o.y,
            y: self.z * o.x - self.x * o.z,
            z: self.x * o.y - self.y * o.x,
        }
    }

    pub fn norm(self) -> f64
    {
        self.dot(self).sqrt()
    }

    /// Unit vector, or None when the vector is null / not finite.
    pub fn normalized(self) -> Option<Vector3>
    {
        let n = self.norm();

        if n.is_finite() && n > 1e-12
        {
            Some(self * (1.0 / n))
        }
        else
        {
            None
        }
    }
}

impl Add for Vector3
{
    type Output = Vector3;

    fn add(self, o: Vector3) -> Vector3
    {
        Vector3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl Sub for Vector3
{
    type Output = Vector3;

    fn sub(self, o: Vector3) -> Vector3
    {
        Vector3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl Mul<f64> for Vector3
{
    type Output = Vector3;

    fn mul(self, k: f64) -> Vector3
    {
        Vector3::new(self.x * k, self.y * k, self.z * k)
    }
}

/// Rotation matrix vehicle frame -> XYZ frame for Euler angles (roll, pitch, yaw),
/// convention Rz(yaw) * Ry(pitch) * Rx(roll).
pub fn rotation_matrix(euler: [f64; 3]) -> Result<Matrix<f64>, String>
{
    let (sr, cr) = euler[0].sin_cos();
    let (sp, cp) = euler[1].sin_cos();
    let (sy, cy) = euler[2].sin_cos();

    let rz = Matrix { data: vec![vec![cy, -sy, 0.0], vec![sy, cy, 0.0], vec![0.0, 0.0, 1.0]] };
    let ry = Matrix { data: vec![vec![cp, 0.0, sp], vec![0.0, 1.0, 0.0], vec![-sp, 0.0, cp]] };
    let rx = Matrix { data: vec![vec![1.0, 0.0, 0.0], vec![0.0, cr, -sr], vec![0.0, sr, cr]] };

    rz.mul(&ry)?.mul(&rx)
}

/// Longitudinal (forward) axis of the vehicle expressed in the XYZ frame:
/// first column of the rotation matrix. Unit vector, the roll does not change it.
pub fn forward_axis(euler: [f64; 3]) -> Vector3
{
    let (sp, cp) = euler[1].sin_cos();
    let (sy, cy) = euler[2].sin_cos();

    Vector3::new(cy * cp, sy * cp, -sp)
}

/// Two unit vectors orthogonal to each other and to the unit vector `f`.
pub fn orthonormal_basis(f: Vector3) -> (Vector3, Vector3)
{
    let reference = if f.z.abs() < 0.9 { Vector3::new(0.0, 0.0, 1.0) } else { Vector3::new(1.0, 0.0, 0.0) };
    let e1 = reference.cross(f).normalized().unwrap_or(Vector3::new(1.0, 0.0, 0.0));
    let e2 = f.cross(e1);

    (e1, e2)
}
