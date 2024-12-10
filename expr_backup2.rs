use crate::config::{get_mode, Mode, TEMP_COUNTER};
use crate::helper::{VarType};


pub trait ExprImpl {
    fn with_name(&self, name: &str) -> Self;
    fn temp_name() -> String {
        let mut counter = TEMP_COUNTER.lock().unwrap();
        let name = format!("temp{}", *counter);
        *counter += 1;
        name
    }

    fn add(&self, other: &Self) -> Self;
    fn sub(&self, other: &Self) -> Self;
    fn mul(&self, other: &Self) -> Self;
    fn uminus(&self) -> Self;
    fn at(&self, index: usize) -> Literal;
    fn at_m(&self, row: usize, col: usize) -> Literal;
}

#[derive(Clone)]
pub enum Literal {
    Var(String, VarType),
    Scalar(f64)
}


pub struct Vector {
    pub value: Vec<Literal>,
    pub name: String
}

pub struct Matrix {
    pub value: Vec<Vec<Literal>>,
    pub name: String
}

impl ExprImpl for Vector {
    fn with_name(&self, name: &str) -> Self {
        let vector = Vector {
            value: self.value.clone(),
            name: name.to_string()
        };

        vector
    }

    // In vector addition, we will not execute something yet
    // Since everything is a literal, we only need to print everything as is
    fn add(&self, other: &Self) -> Self {
        
        Vector{
            value: vec![],
            name: Vector::temp_name()
        }
    }

    fn sub(&self, other: &Self) -> Self {

        Vector{
            value: vec![],
            name: Vector::temp_name()
        }
    }

    fn mul(&self, other: &Self) -> Self {
        
        Vector{
            value: vec![],
            name: Vector::temp_name()
        }
    }

    fn at(&self, index: usize) -> Literal {
        self.value[index].clone()
    }       

    fn at_m(&self, row: usize, col: usize) -> Literal {
        panic!("Not implemented yet")
    }

    fn uminus(&self) -> Self {
        Vector{
            value: vec![],
            name: Vector::temp_name()
        }
    }
}

impl ExprImpl for Matrix {
    fn with_name(&self, name: &str) -> Self {
        let matrix = Matrix {
            value: self.value.clone(),
            name: name.to_string()
        };

        matrix
    }

    fn add(&self, other: &Self) -> Self {
        
        Matrix{
            value: vec![],
            name: Matrix::temp_name()
        }
    }

    fn sub(&self, other: &Self) -> Self {

        Matrix{
            value: vec![],
            name: Matrix::temp_name()
        }
    }

    fn mul(&self, other: &Self) -> Self {
        
        Matrix{
            value: vec![],
            name: Matrix::temp_name()
        }
    }

    fn uminus(&self) -> Self {
        Matrix{
            value: vec![],
            name: Matrix::temp_name()
        }
    }

    fn at(&self, index: usize) -> Literal {
        panic!("Not implemented yet")
    }       

    fn at_m(&self, row: usize, col: usize) -> Literal {
        self.value[row][col].clone()
    }
}


