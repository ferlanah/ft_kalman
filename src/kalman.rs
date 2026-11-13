#![allow(non_snake_case)]

use crate::matrix::Matrix;


pub struct KalmanFilter
{
    pub x: Matrix<f64>,
    pub P: Matrix<f64>,

    pub F: Matrix<f64>,
    pub H: Matrix<f64>,

    pub Q: Matrix<f64>,
    pub R: Matrix<f64>,
}


impl KalmanFilter
{
    pub fn new
    (
        x: Matrix<f64>,
        P: Matrix<f64>,
        F: Matrix<f64>,
        H: Matrix<f64>,
        Q: Matrix<f64>,
        R: Matrix<f64>,
    ) -> Self 
    {
        KalmanFilter
        {
            x,
            P,
            F,
            H,
            Q,
            R,
        }
    }


    pub fn predict_state(&mut self) -> Result<(), String>
    {
        let result = self.F.mul(&self.x)?;

        self.x = result;

        Ok(())
    }


    pub fn predict_covariance(&mut self) -> Result<(), String>
    {
        let ft = self.F.transpose()?;
        let fp = self.F.mul(&self.P)?;
        let fpf = fp.mul(&ft)?;
        let result = fpf.add(&self.Q)?;

        self.P = result;

        Ok(())
    }


    pub fn predict(&mut self) -> Result<(), String>
    {
        self.predict_state()?;
        self.predict_covariance()?;

        Ok(())
    }


    pub fn innovation(&self, z: &Matrix<f64>) -> Result<Matrix<f64>, String>
    {
        let hx = self.H.mul(&self.x)?;
        let y = z.sub(&hx)?;

        Ok(y)
    }


    pub fn innovation_covariance(&self) -> Result<Matrix<f64>, String>
    {
        let ht = self.H.transpose()?;
        let hp = self.H.mul(&self.P)?;
        let hph = hp.mul(&ht)?;
        let s = hph.add(&self.R)?;

        Ok(s)
    }


    pub fn kalman_gain(&self) -> Result<Matrix<f64>, String>
    {
        let ht = self.H.transpose()?;
        let s = self.innovation_covariance()?;
        let s_inv = s.inverse()?;
        let pht = self.P.mul(&ht)?;
        let k = pht.mul(&s_inv)?;

        Ok(k)
    }


    pub fn update_state(&mut self, k: &Matrix<f64>, y: &Matrix<f64>,) -> Result<(), String>
    {
        let ky = k.mul(y)?;
        let result = self.x.add(&ky)?;
        self.x = result;

        Ok(())
    }


    pub fn update_covariance(&mut self, k: &Matrix<f64>) -> Result<(), String>
    {
        let kh = k.mul(&self.H)?;
        let identity = self.P.identity();
        let i_kh = identity.sub(&kh)?;
        let result = i_kh.mul(&self.P)?;
        self.P = result;

        Ok(())
    }


    pub fn update(&mut self, z: &Matrix<f64>) -> Result<(), String>
    {
        let y = self.innovation(z)?;
        let k = self.kalman_gain()?;
        self.update_state(&k, &y)?;
        self.update_covariance(&k)?;

        Ok(())
    }
    /// Prediction with a control input: x = F x + B u, P = F P Ft + Q.
    /// `bu` is the already computed column vector B u.
    pub fn predict_with_control(&mut self, bu: &Matrix<f64>) -> Result<(), String>
    {
        let fx = self.F.mul(&self.x)?;
        let result = fx.add(bu)?;

        self.x = result;
        self.predict_covariance()?;

        Ok(())
    }


    /// Update with an arbitrary measurement model (H, R) given for this call only.
    /// Uses the Joseph form of the covariance update, which keeps P symmetric and
    /// positive on very long runs. Filter's own H and R are left untouched.
    pub fn update_with(&mut self, z: &Matrix<f64>, h: &Matrix<f64>, r: &Matrix<f64>) -> Result<(), String>
    {
        let old_h = std::mem::replace(&mut self.H, h.clone());
        let old_r = std::mem::replace(&mut self.R, r.clone());
        let result = self.update_joseph(z);
        self.H = old_h;
        self.R = old_r;

        result
    }


    pub fn update_joseph(&mut self, z: &Matrix<f64>) -> Result<(), String>
    {
        let y = self.innovation(z)?;
        let k = self.kalman_gain()?;
        self.update_state(&k, &y)?;

        let kh = k.mul(&self.H)?;
        let i_kh = self.P.identity().sub(&kh)?;
        let left = i_kh.mul(&self.P)?.mul(&i_kh.transpose()?)?;
        let right = k.mul(&self.R)?.mul(&k.transpose()?)?;
        self.P = left.add(&right)?;

        Ok(())
    }


    /// Force P = (P + Pt) / 2 to remove the numerical asymmetry.
    pub fn symmetrize(&mut self) -> Result<(), String>
    {
        let n = self.P.data.len();

        if n == 0 || self.P.data.iter().any(|r| r.len() != n)
        {
            return Err("Covariance matrix must be square".to_string());
        }
        for i in 0..n
        {
            for j in (i + 1)..n
            {
                let m = 0.5 * (self.P.data[i][j] + self.P.data[j][i]);
                self.P.data[i][j] = m;
                self.P.data[j][i] = m;
            }
        }

        Ok(())
    }


    /// Error if the state or the covariance contains NaN / infinity.
    pub fn check_finite(&self) -> Result<(), String>
    {
        if !self.x.is_finite() || !self.P.is_finite()
        {
            return Err("Kalman filter diverged (non finite values)".to_string());
        }

        Ok(())
    }
}
