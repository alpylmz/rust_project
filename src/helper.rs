// add lazy_static
// add mutex
use std::sync::Mutex;
use lazy_static::lazy_static;



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


// have a global variable for all var names
lazy_static! {
    pub static ref VAR_NAMES: Mutex<Vec<String>> = Mutex::new(Vec::new());
    // temp var counter
    pub static ref TEMP_VAR_COUNTER: Mutex<i32> = Mutex::new(0);

    pub static ref ALL_VARS: Mutex<Vec<(String, Vec<usize>)>> = Mutex::new(Vec::new());
}

// Old, used in not UNROLL case
pub fn add_var_name(name: &str) {
    VAR_NAMES.lock().unwrap().push(name.to_string());
}

// New, used in UNROLL case
pub fn add_var(name: &str, dims: Vec<usize>) {
    ALL_VARS.lock().unwrap().push((name.to_string(), dims));
}

// TODO: I will collect all the sizes of variables in a place, and use this function to get the size of a variable
pub fn get_size(name: &str) -> Vec<usize> {
    let all_vars = ALL_VARS.lock().unwrap();
    for (var_name, dims) in all_vars.iter() {
        if var_name == name {
            return dims.clone();
        }
    }
    panic!("Variable not found for name: {}, and list of variables: {:?}", name, all_vars);
}

pub fn search_var_name(name: &str) -> bool {
    let all_vars = ALL_VARS.lock().unwrap();
    for (var_name, _) in all_vars.iter() {
        if var_name == name {
            return true;
        }
    }
    false
}

////pub fn search_var_name(name: &str) -> bool {
////    VAR_NAMES.lock().unwrap().contains(&name.to_string())
////}



// get a new name based on the input name
pub fn get_new_name(name: &str) -> String {
    let mut counter = TEMP_VAR_COUNTER.lock().unwrap();
    *counter += 1;
    format!("r_{}_{}", counter, name)
}




