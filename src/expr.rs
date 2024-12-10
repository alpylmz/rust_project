use crate::config::{get_mode, Mode, TEMP_COUNTER};
use std::ops::{Add, Sub, Mul};

#[derive(Debug, Clone)]
pub enum Expr {
    InputScalar{name: String},
    InputVector{name: String},
    InputMatrix{name: String},
    Var{name: String},
    Scalar{name: String, value: f64},
    Vector{name: String, value: Vec<Expr>},
    Matrix{name: String, value: Vec<Vec<Expr>>},
}

pub trait ExprImpl {
    // only in new function I will attempt to print
    fn new(data: Expr, name: &str) -> Self;
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
}

impl Expr {
    pub fn new(data: Expr, name: &str) -> Self {
        match data {
            Expr::Scalar{name, value} => {
                // first print
                if let Mode::ANALYSIS = get_mode() {
                    println!("val {} = {}", name, value);
                }
                Expr::Scalar{name: name.to_string(), value: value}
            }
            Expr::Vector{name, value} => {
                // first, go through every element, and print if something is needed
                // when something is needed? If an element is a scalar or a variable, we can just print it
                // for now I can't see a case where I would need, therefore I'll skip this part
                if let Mode::ANALYSIS = get_mode() {
                    let elements = value.iter().map(|e| match e {
                        Expr::Scalar{name, value} => value.to_string(),
                        Expr::Var{name} => name.to_string(),
                        _ => panic!("Vector can only contain Scalar or Var"),
                    }).collect::<Vec<_>>().join(", ");
                    println!("val {} = Vector(List({}))", name, elements);
                }

                Expr::Vector{name: name.to_string(), value: value}
            }
            Expr::Matrix{name, value} => {
                // first, go through every element, and print if something is needed
                // when something is needed? If an element is a scalar or a variable, we can just print it
                // for now I can't see a case where I would need, therefore I'll skip this part
                if let Mode::ANALYSIS = get_mode() {
                    let elements = value.iter().map(|row| {
                        row.iter().map(|e| match e {
                            Expr::Scalar{name, value} => value.to_string(),
                            Expr::Var{name} => name.to_string(),
                            _ => panic!("Matrix can only contain Scalar or Var"),
                        }).collect::<Vec<_>>().join(", ")
                    }).collect::<Vec<_>>().join(", ");
                    // print it like:
                    ////val limi_rotation_0: Matrix = Matrix(List(
                    ////    List(rotation_matrix_0.at(0, 0), rotation_matrix_0.at(0, 1), 0.0),
                    ////    List(rotation_matrix_0.at(1, 0), rotation_matrix_0.at(1, 1), 0.0),
                    ////    List(0.0, 0.0, 1.0)
                    ////))
                    // print the first line
                    println!("val {} = Matrix(List(", name);
                    for (i, row) in value.iter().enumerate() {
                        // print one line
                        let row_str = row.iter().map(|e| match e {
                            Expr::Scalar{name, value} => value.to_string(),
                            Expr::Var{name} => name.to_string(),
                            _ => panic!("Matrix can only contain Scalar or Var"),
                        }).collect::<Vec<_>>().join(", ");
                        println!("    List({})", row_str);
                    }
                    // print the last line
                    println!("))");
                }

                Expr::Matrix{name: name.to_string(), value: value}
            }
            _ => panic!("Scalar can only be constructed from Scalar Expr"),
        }
    }
    fn temp_name() -> String {
        let mut counter = TEMP_COUNTER.lock().unwrap();
        let name = format!("temp{}", *counter);
        *counter += 1;
        name
    }
    fn scalar(value: f64) -> Self {
        Expr::Scalar{name: Self::temp_name(), value}
    }
    fn vector(value: Vec<Expr>) -> Self {
        Expr::Vector{name: Self::temp_name(), value}
    }
    fn matrix(value: Vec<Vec<Expr>>) -> Self {
        Expr::Matrix{name: Self::temp_name(), value}
    }

    pub fn get_name(&self) -> String {
        match self {
            Expr::InputScalar{name} => name.to_string(),
            Expr::InputVector{name} => name.to_string(),
            Expr::InputMatrix{name} => name.to_string(),
            Expr::Var{name} => name.to_string(),
            Expr::Scalar{name, value: _} => name.to_string(),
            Expr::Vector{name, value: _} => name.to_string(),
            Expr::Matrix{name, value: _} => name.to_string(),
        }
    }

    pub fn with_name(&self, name: &str) -> Self {
        println!("with_name called");
        match self {
            Expr::InputScalar{name: _} => Expr::Var{name: name.to_string()},
            Expr::InputVector{name: _} => Expr::Var{name: name.to_string()},
            Expr::InputMatrix{name: _} => Expr::Var{name: name.to_string()},
            Expr::Var{name: _} => Expr::Var{name: name.to_string()},
            Expr::Scalar{name, value} => Expr::Scalar{name: name.to_string(), value: *value},
            Expr::Vector{name, value} => Expr::Vector{name: name.to_string(), value: value.clone()},
            Expr::Matrix{name, value} => Expr::Matrix{name: name.to_string(), value: value.clone()},
        }
    }

    pub fn uminus(&self) -> Self {
        match self {
            Expr::Scalar{name: _, value} => Expr::scalar(-value),
            Expr::Vector{name: _, value} => {
                let value = value.iter().map(|e| e.uminus()).collect();
                Expr::vector(value)
            },
            Expr::Matrix{name: _, value} => {
                let value = value.iter().map(|row| row.iter().map(|e| e.uminus()).collect()).collect();
                Expr::matrix(value)
            },
            Expr::Var{name: name} => {
                // add "-" to the name for now, it should work
                Expr::Var{name: format!("-{}", name)}
            }
            _ => panic!("Unary minus not supported for different types"),
        }
    }

    fn add(&self, other: &Self) -> Self {
        match (self, other) {
            (Expr::Scalar{name: _, value: v1}, Expr::Scalar{name: _, value: v2}) => Expr::scalar(v1 + v2),
            (Expr::Vector{name: _, value: v1}, Expr::Vector{name: _, value: v2}) => {
                let value = v1.iter().zip(v2.iter()).map(|(a, b)| a.add(b)).collect();
                Expr::vector(value)
            },
            (Expr::Matrix{name: _, value: v1}, Expr::Matrix{name: _, value: v2}) => {
                let value = v1.iter().zip(v2.iter()).map(|(a, b)| a.iter().zip(b.iter()).map(|(a, b)| a.add(b)).collect()).collect();
                Expr::matrix(value)
            },
            _ => panic!("Addition not supported for different types"),
        }
    }

    fn sub(&self, other: &Self) -> Self {
        match (self, other) {
            (Expr::Scalar{name: _, value: v1}, Expr::Scalar{name: _, value: v2}) => Expr::scalar(v1 - v2),
            (Expr::Vector{name: _, value: v1}, Expr::Vector{name: _, value: v2}) => {
                let value = v1.iter().zip(v2.iter()).map(|(a, b)| a.sub(b)).collect();
                Expr::vector(value)
            },
            (Expr::Matrix{name: _, value: v1}, Expr::Matrix{name: _, value: v2}) => {
                let value = v1.iter().zip(v2.iter()).map(|(a, b)| a.iter().zip(b.iter()).map(|(a, b)| a.sub(b)).collect()).collect();
                Expr::matrix(value)
            },
            _ => panic!("Subtraction not supported for different types"),
        }
    }

    fn mul(&self, other: &Self) -> Self {
        println!("mul called");
        // if Analysis mode, print the multiplication
        if let Mode::ANALYSIS = get_mode() {
            println!("val {} = {} * {}", Self::temp_name(), self.get_name(), other.get_name());
        }
        match (self, other) {
            (Expr::Scalar{name: _, value: v1}, Expr::Scalar{name: _, value: v2}) => Expr::scalar(v1 * v2),
            (Expr::Vector{name: _, value: v1}, Expr::Vector{name: _, value: v2}) => {
                panic!("You need to explicitly use cross or dot product for vector multiplication");
            },
            (Expr::Matrix{name: _, value: v1}, Expr::Matrix{name: _, value: v2}) => {
                let value = v1.iter().zip(v2.iter()).map(|(a, b)| a.iter().zip(b.iter()).map(|(a, b)| a.mul(b)).collect()).collect();
                Expr::matrix(value)
            },
            (Expr::Var{name: name1}, Expr::Var{name: name2}) => Expr::Var{name: format!("{} * {}", name1, name2)},
            (a, b) => panic!("Multiplication not supported for different types, types are: {:?}, {:?}", a, b),
        }
    }

    fn cross(&self, other: &Self) -> Self {
        match (self, other) {
            (Expr::Vector{name: _, value: v1}, Expr::Vector{name: _, value: v2}) => {
                let value = vec![
                    v1[1].clone().mul(v2[2].clone()).sub(v1[2].clone().mul(v2[1].clone())),
                    v1[2].clone().mul(v2[0].clone()).sub(v1[0].clone().mul(v2[2].clone())),
                    v1[0].clone().mul(v2[1].clone()).sub(v1[1].clone().mul(v2[0].clone())),
                ];
                Expr::vector(value)
            },
            _ => panic!("Cross product only supported for vectors"),
        }
    }

    fn dot(&self, other: &Self) -> f64 {
        match (self, other) {
            (Expr::Vector{name: _, value: v1}, Expr::Vector{name: _, value: v2}) => {
                v1.iter().zip(v2.iter()).map(|(a, b)| a.mul(b)).map(|x| match x {
                    Expr::Scalar{name: _, value: v} => v,
                    _ => panic!("Dot product should return a scalar"),
                }).sum()
            },
            _ => panic!("Dot product only supported for vectors"),
        }
    }

    pub fn at(&self, index: usize) -> Self {
        match self {
            Expr::Vector{name: name, value: v} => {
                // if it is just a vector, I can use .at(index) to get the element, by putting that into a variable
                Expr::Var{name: format!("{}.at({})", name, index)}
            }
            _ => panic!("Indexing only supported for vectors"),
        }
    }

    pub fn at_m(&self, i: usize, j: usize) -> Self {
        match self {
            Expr::Matrix{name: _, value: v} => v[i][j].clone(),
            _ => panic!("Indexing only supported for matrices"),
        }
    }


}

impl Add for Expr {
    type Output = Expr;

    fn add(self, other: Self) -> Self {
        self.add(other)
    }
}

impl Add<&Expr> for &Expr {
    type Output = Expr;

    fn add(self, other: &Expr) -> Expr {
        self.clone().add(other.clone())
    }
}

impl Sub for Expr {
    type Output = Expr;

    fn sub(self, other: Self) -> Self {
        self.sub(other)
    }
}

impl Mul for Expr {
    type Output = Expr;

    fn mul(self, other: Self) -> Self {
        self.mul(other)
    }
}

// Implement Mul for &Expr
impl Mul<&Expr> for &Expr {
    type Output = Expr;

    fn mul(self, other: &Expr) -> Expr {
        self.mul(other)
    }
}



