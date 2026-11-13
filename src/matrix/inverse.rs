use super::Matrix;
use super::one::One;
use super::zero::Zero;

use std::ops::{DivAssign, Mul, SubAssign};

impl<K> Matrix<K>
where
    K: Copy
        + Zero
        + One
        + PartialEq
        + DivAssign
        + SubAssign
        + Mul<Output = K>,
{

    fn build_mat(&self) -> Vec<Vec<K>> 
    {
        let n = self.data.len();

        let mut m = vec![vec![K::zero(); 2 * n]; n];

        for i in 0..n 
        {
            for j in 0..n 
            {
                m[i][j] = self.data[i][j];

                if i == j 
                {
                    m[i][j + n] = K::one();
                }
            }
        }

        m
    }

    fn gauss_jordan(&self) -> Result<Vec<Vec<K>>, String> 
    {
        let n = self.data.len();

        let mut m = self.build_mat();

        for i in 0..n 
        {
            let mut pivot = m[i][i];


            if pivot == K::zero() 
            {
                let mut found = false;

                for j in (i + 1)..n 
                {
                    if m[j][i] != K::zero() 
                    {
                        m.swap(i, j);
                        pivot = m[i][i];
                        found = true;
                        break;
                    }
                }

                if !found 
                {
                    return Err("Matrix is singular".to_string());
                }
            }

            for k in 0..2 * n 
            {
                m[i][k] /= pivot;
            }

            for j in 0..n 
            {
                if j != i 
                {
                    let lambda = m[j][i];

                    for k in 0..2 * n {
                        let delta = m[i][k] * lambda;
                        m[j][k] -= delta;
                    }
                }
            }
        }

        Ok(m)
    }

    fn extract_inverse(&self, m: Vec<Vec<K>>) -> Matrix<K> 
    {
        let n = self.data.len();

        let mut data = Vec::with_capacity(n);

        for i in 0..n 
        {
            let mut row = Vec::with_capacity(n);

            for j in 0..n 
            {
                row.push(m[i][j + n]);
            }

            data.push(row);
        }

        Matrix { data }
    }

    pub fn inverse(&self) -> Result<Matrix<K>, String> 
    {
        let n = self.data.len();


        if n == 0 
        {
            return Err("Matrix is empty".to_string());
        }

        for row in &self.data 
        {
            if row.len() != n 
            {
                return Err("Matrix must be square".to_string());
            }
        }

        let m = self.gauss_jordan()?;

        Ok(self.extract_inverse(m))
    }
}