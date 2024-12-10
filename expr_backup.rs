use std::ops::{Add, Sub, Mul};
use std::sync::Mutex;

struct Scalar {
    data: f64,
}

struct Vector {
    data: Vec<AST>,
}

struct Matrix {
    data: Vec<Vec<AST>>,
}

#[derive(Clone)]
enum AST {
    Int(i64),
    Scalar(f64),
    Var(String),
    Vector(Vec<AST>),
    Matrix(Vec<Vec<AST>>),
    Add(Box<AST>, Box<AST>),
    Multiply(Box<AST>, Box<AST>),
    Sub(Box<AST>, Box<AST>),
    Uminus(Box<AST>),
    At(Box<AST>), // at for vectors
    AtM(Box<AST>, Box<AST>), // at for matrices
    Transpose(Box<AST>),
    Dot(Box<AST>, Box<AST>),
}

// in this stage, everything will be collected into a single AST, and then it will be evaluated if needed
// get a global variable for AST
// but, the global AST will be a vector of ASTs, because I don't want to deal with the functional style in main.rs
lazy_static::lazy_static! {
    static ref GLOBAL_AST: Mutex<Vec<AST>> = Mutex::new(Vec::new());
}

pub fn update_ast(ast: AST) {
    let mut global_ast = GLOBAL_AST.lock().unwrap();
    *global_ast = vec![ast];
}

pub fn get_ast() -> Vec<AST> {
    let global_ast = GLOBAL_AST.lock().unwrap();
    global_ast.clone()
}


// This function takes the global AST, and makes sure that the Vector and Matrices are simple, meaning that each element of both are either a scalar or a variable
pub fn simplify() -> Vec<AST>{
    let global_ast = get_ast();
    let mut new_ast = Vec::new();
    for ast in global_ast.iter(){
        match ast {
            AST::Vector(v) => {
                let mut new_vector = Vec::new();
                for element in v.iter(){
                    match element {
                        AST::Scalar(_) | AST::Var(_) => new_vector.push(element.clone()),
                        _ => panic!("Vector can only contain scalars or variables"),
                    }
                }
                new_ast.push(AST::Vector(new_vector));
            }
            AST::Matrix(m) => {
                let mut new_matrix = Vec::new();
                for row in m.iter(){
                    let mut new_row = Vec::new();
                    for element in row.iter(){
                        match element {
                            AST::Scalar(_) | AST::Var(_) => new_row.push(element.clone()),
                            _ => panic!("Matrix can only contain scalars or variables"),
                        }
                    }
                    new_matrix.push(new_row);
                }
                new_ast.push(AST::Matrix(new_matrix));
            }
            _ => new_ast.push(ast.clone()),
        }
    }
    new_ast.clone()
}



//////enum Value {
//////    Scalar(f64),
//////    Vector(Vector),
//////    Matrix(Matrix),
//////}
//////
//////
//////fn eval(ast: AST) -> Result<Value, &'static str> {
//////    match ast {
//////        AST::Scalar(n) => Ok(Value::Scalar(n)),
//////        AST::Add(a1, a2) => {
//////            let m1 = eval(*a1)?;
//////            let m2 = eval(*a2)?;
//////            m1 + m2
//////        }
//////        AST::Multiply(a1, a2) => {
//////            let m1 = eval(*a1)?;
//////            let m2 = eval(*a2)?;
//////            todo!()
//////        }
//////        AST::Transpose(a) => {
//////            let m = eval(*a)?;
//////            match m {
//////                Value::Matrix(m) => Ok(Value::Matrix(m.transpose())),
//////                _ => Err("Cannot transpose non-matrix"),
//////            }
//////        }
//////        AST::Dot(a1, a2) => {
//////            let m1 = eval(*a1)?;
//////            let m2 = eval(*a2)?;
//////            todo!()
//////        }
//////    }
//////}

fn main() {
    println!("Hello, world!");
}