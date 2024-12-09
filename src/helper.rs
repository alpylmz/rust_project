use std::sync::Mutex;

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

pub enum VarType {
    Scalar,
    Vector,
    Matrix,
}


// have a struct to hold all the variables, because we may need some checks on them, and keeping track of everything will be easier
// they should also contain the vartype, and maybe some other information that we may need
// because of that, I will just put the info on a struct so that I can easily expand it

pub struct VarInfo {
    pub name: String,
    pub vartype: VarType
}

lazy_static::lazy_static! {
    static ref all_variables: Mutex<Vec<VarInfo>> = Mutex::new(Vec::new());
}

pub fn add_variable(info: VarInfo) {
    let mut variables = all_variables.lock().unwrap();
    variables.push(info);
}




