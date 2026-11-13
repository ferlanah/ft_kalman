use super::Matrix;
use std::ops::AddAssign;

impl<K> Matrix<K>
where
    K: AddAssign + Copy,
{
    pub fn add(&self, other: &Matrix<K>) -> Result<Matrix<K>, String>
    {
        let mut result = self.clone();
        let row = self.data.len();

        if row == 0 || other.data.is_empty()
        {
            return Err("Matrices must not be empty".to_string());
        }
        let col = self.data[0].len();

        if row != other.data.len()
        {
            return Err("Matrices must have the same number of rows".to_string());
        }
        if col != other.data[0].len()
        {
            return Err("Matrices must have the same number of columns".to_string());
        }
        if self.data.iter().any(|r| r.len() != col) || other.data.iter().any(|r| r.len() != col)
        {
            return Err("Inconsistent row lengths in matrix".to_string());
        }

        for i in 0..row
        {
            for j in 0..col
            {
                result.data[i][j] += other.data[i][j];
            }
        }
        Ok(result)
    }
}
