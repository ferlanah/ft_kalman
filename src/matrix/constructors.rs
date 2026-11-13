use super::Matrix;

impl<K> Matrix<K>
where
    K: Clone + Default,
{
    /// Build a matrix from rows. Fails when empty or when the rows have different lengths.
    pub fn from_rows(rows: Vec<Vec<K>>) -> Result<Matrix<K>, String>
    {
        if rows.is_empty() || rows[0].is_empty()
        {
            return Err("Matrix must not be empty".to_string());
        }
        let col = rows[0].len();
        if rows.iter().any(|r| r.len() != col)
        {
            return Err("Inconsistent row lengths in matrix".to_string());
        }
        Ok(Matrix { data: rows })
    }

    /// Matrix of `rows` x `cols` filled with the default value (zero for numbers).
    pub fn zeros(rows: usize, cols: usize) -> Matrix<K>
    {
        Matrix { data: vec![vec![K::default(); cols]; rows] }
    }

    pub fn rows(&self) -> usize
    {
        self.data.len()
    }

    pub fn cols(&self) -> usize
    {
        if self.data.is_empty() { 0 } else { self.data[0].len() }
    }
}

impl Matrix<f64>
{
    /// Column vector from a slice.
    pub fn column(values: &[f64]) -> Matrix<f64>
    {
        Matrix { data: values.iter().map(|v| vec![*v]).collect() }
    }

    /// True when every coefficient is a finite number (no NaN / infinity).
    pub fn is_finite(&self) -> bool
    {
        self.data.iter().all(|r| r.iter().all(|v| v.is_finite()))
    }
}
