#[derive(Clone, Debug, PartialEq)]
pub struct Matrix<K>
{
    pub data: Vec<Vec<K>>,
}

pub mod add;
pub mod sub;
pub mod mul;
pub mod transpose;
pub mod inverse;
pub mod identity;
pub mod one;
pub mod zero;
pub mod constructors;
