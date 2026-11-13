use super::Matrix;
use super::one::One;


impl<K> Matrix<K>
    where
        K: Default +  Copy + One,
{
    pub fn identity(&self) -> Matrix<K>
    {
        let row = self.data.len();
        let mut result = vec![vec![K::default(); row]; row];
        for i in 0..row
        {
            result[i][i] = K::one();
        }
        Matrix{data: result, } 
    }
}