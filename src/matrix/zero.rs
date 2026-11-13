pub trait Zero
{
    fn zero() -> Self;
}

impl Zero for f64
{
    fn zero() -> Self
    {
        0.0
    }
}

impl Zero for i32
{
    fn zero() -> Self
    {
        0
    }
}

impl Zero for u64
{
    fn zero() -> Self
    {
        0
    }
}
