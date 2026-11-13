use super::Matrix;
use std::ops::{Add, Mul};

impl<K> Matrix<K>
where
    K: Copy + Default + Add<Output = K> + Mul<Output = K>,
{
    pub fn mul(&self, mat: &Matrix<K>) -> Result<Matrix<K>, String> 
    {
        let row = self.data.len();
        if row == 0 {
            return Err("Left matrix has no rows".to_string());
        }

        let n = self.data[0].len();

        for r in &self.data {
            if r.len() != n {
                return Err("Inconsistent row lengths in left matrix".to_string());
            }
        }

        if mat.data.is_empty() {
            return Err("Right matrix has no rows".to_string());
        }

        let col = mat.data[0].len();

        for r in &mat.data {
            if r.len() != col {
                return Err("Inconsistent row lengths in right matrix".to_string());
            }
        }

        if mat.data.len() != n {
            return Err("Matrix dimensions incompatible for multiplication".to_string());
        }

        let mut result = vec![vec![K::default(); col]; row];

        for i in 0..row {
            for k in 0..col {
                let mut sum = K::default();

                for j in 0..n {
                    sum = sum + self.data[i][j] * mat.data[j][k];
                }

                result[i][k] = sum;
            }
        }

        Ok(Matrix { data: result })
    }
}