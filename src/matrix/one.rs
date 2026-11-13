pub trait One
{
    fn one() -> Self;
}

impl One for f64
{
    fn one() -> Self
    {
        1.0
    }
}

impl One for i32
{
    fn one() -> Self
    {
        1
    }
}

impl One for u64
{
    fn one() -> Self
    {
        1
    }
}