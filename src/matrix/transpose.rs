use super::Matrix;

impl<K> Matrix<K>
where
    K: Copy + Default,
{
    pub fn transpose(&self) -> Result<Matrix<K>, String> 
    {
        let row = self.data.len();

        if row == 0 
        {
            return Ok(Matrix { data: Vec::new() });
        }

        let col = self.data[0].len();

        for r in &self.data 
        {
            if r.len() != col 
            {
                return Err("Inconsistent row lengths in matrix".to_string());
            }
        }

        let mut result = vec![vec![K::default(); row]; col];

        for i in 0..row 
        {
            for j in 0..col 
            {
                result[j][i] = self.data[i][j];
            }
        }
        Ok(Matrix { data: result })
    }
}