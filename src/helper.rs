
// a struct that contains an enum of the variable(whether it is a scalar, vector or matrix) and,
// if it is a scalar, only one lower and upper bound
// if it is a vector, n lower and upper bounds
// if it is a matrix, n*m lower and upper bounds

pub struct Bounds {
    pub lower_bound: f64,
    pub upper_bound: f64,
}

pub enum Input<'a> {
    Scalar(&'a str, Bounds),
    Vector(&'a str, Vec<Bounds>),
    Matrix(&'a str, Vec<Vec<Bounds>>),
}

#[derive(Debug, Clone, PartialEq)]
pub enum VarType {
    Scalar,
    Vector,
    Matrix,
}






