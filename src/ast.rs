

use std::fmt;
use crate::helper::{add_var_name, get_new_name, search_var_name, VarType};
use std::ops::{Add, Mul, Neg, Sub};
use crate::config::UNROLL;



#[derive(Debug, Clone)]
pub enum ASTNode {
    Scalar(f64),
    VariableS{name: String}, // Scalar
    VariableV{name: String}, // Vector
    VariableM{name: String}, // Matrix
    Vector(Vec<ASTNode>),
    Matrix(Vec<Vec<ASTNode>>),
    Add(Box<ASTNode>, Box<ASTNode>),
    Sub(Box<ASTNode>, Box<ASTNode>),
    Mul(Box<ASTNode>, Box<ASTNode>),
    Cross(Box<ASTNode>, Box<ASTNode>),
    Transpose(Box<ASTNode>),
    AtVec(Box<ASTNode>, usize),
    AtMat(Box<ASTNode>, usize, usize),
    Neg(Box<ASTNode>),
}


impl From<f64> for ASTNode {
    fn from(value: f64) -> Self {
        ASTNode::Scalar(value)
    }
}
impl From<i32> for ASTNode {
    fn from(value: i32) -> Self {
        ASTNode::Scalar(value as f64)
    }
}

// Operator overloads
impl Add for ASTNode {
    type Output = ASTNode;
    fn add(self, rhs: ASTNode) -> ASTNode {
        ASTNode::Add(Box::new(self), Box::new(rhs))
    }
}

impl Sub for ASTNode {
    type Output = ASTNode;
    fn sub(self, rhs: ASTNode) -> ASTNode {
        ASTNode::Sub(Box::new(self), Box::new(rhs))
    }
}

impl Mul for ASTNode {
    type Output = ASTNode;
    fn mul(self, rhs: ASTNode) -> ASTNode {
        ASTNode::Mul(Box::new(self), Box::new(rhs))
    }
}
impl Neg for ASTNode {
    type Output = ASTNode;
    fn neg(self) -> ASTNode {
        ASTNode::Neg(Box::new(self))
    }
}

#[macro_export]
macro_rules! Scalar {
    ($item:expr) => {
        ASTNode::Scalar($item)
    };
}

#[macro_export]
macro_rules! Vector {
    ($($item:expr),*) => {
        {
            let vec_ast: Vec<ASTNode> = vec![$($item.into()),*];
            ASTNode::Vector(vec_ast)
        }
    };
}

#[macro_export]
macro_rules! Matrix {
    ( $( [ $($item:expr),* ] ),* ) => {
        {
            let mat_ast: Vec<Vec<ASTNode>> = vec![
                $(
                    {
                        let row_vec: Vec<ASTNode> = vec![$($item.into()),*];
                        row_vec
                    }
                ),*
            ];
            ASTNode::Matrix(mat_ast)
        }
    };
}

impl ASTNode {
    pub fn at_vec(self, i: usize) -> ASTNode {
        ASTNode::AtVec(Box::new(self), i)
    }

    pub fn cross(self, rhs: ASTNode) -> ASTNode {
        ASTNode::Cross(Box::new(self), Box::new(rhs))
    }

    pub fn transpose(self) -> ASTNode {
        ASTNode::Transpose(Box::new(self))
    }

    pub fn at_mat(self, r: usize, c: usize) -> ASTNode {
        ASTNode::AtMat(Box::new(self), r, c)
    }

    /// Infer the VarType from the node type.
    fn infer_type(&self) -> VarType {
        match self {
            ASTNode::Scalar(_) => VarType::Scalar,
            ASTNode::VariableS { .. } => VarType::Scalar,
            ASTNode::VariableV { .. } => VarType::Vector,
            ASTNode::VariableM { .. } => VarType::Matrix,
            ASTNode::Vector(_) => VarType::Vector,
            ASTNode::Matrix(_) => VarType::Matrix,
            ASTNode::Add(child1, child2) => {
                // Assume some type. For addition, it's often the same as the children.
                // Adjust this logic as needed. If you need type correctness, store it explicitly.
                let child1_type = child1.infer_type();
                let child2_type = child2.infer_type();
                if child1_type == child2_type {
                    child1_type
                } else if child1_type == VarType::Vector && child2_type == VarType::Scalar {
                    VarType::Vector
                } else if child1_type == VarType::Matrix && child2_type == VarType::Scalar {
                    VarType::Matrix
                } else {
                    println!("Addtype");
                    println!("child1_type: {:?}", child1_type);
                    println!("child2_type: {:?}", child2_type);
                    println!("child1: {:?}", child1);
                    println!("child2: {:?}", child2);
                    panic!("Addition of different types is not allowed in this context")
                }
            }
            ASTNode::Mul(child1, child2) => {
                // Multiplication is the same as addition
                let child1_type = child1.infer_type();
                let child2_type = child2.infer_type();
                // if we are multiplying a scalar with a vector, or a scalar with a matrix that is also valid
                // anything else is not
                if child1_type == child2_type {
                    child1_type
                } else if child1_type == VarType::Scalar && child2_type == VarType::Vector {
                    VarType::Vector
                } else if child1_type == VarType::Scalar && child2_type == VarType::Matrix {
                    VarType::Matrix
                } else if child1_type == VarType::Matrix && child2_type == VarType::Vector {
                    VarType::Vector
                } else {
                    println!("Multype");
                    println!("child1_type: {:?}", child1_type);
                    println!("child2_type: {:?}", child2_type);
                    println!("child1: {:?}", child1);
                    println!("child2: {:?}", child2);
                    panic!("Multiplication of different types is not allowed in this context")
                }
            }
            ASTNode::Sub(child1, child2) => {
                // Subtraction is the same as addition
                let child1_type = child1.infer_type();
                let child2_type = child2.infer_type();
                // if we are subtracting a scalar from a vector, or a scalar from a matrix that is also valid
                // anything else is not
                if child1_type == child2_type {
                    child1_type
                } else if child1_type == VarType::Vector && child2_type == VarType::Scalar {
                    VarType::Vector
                } else if child1_type == VarType::Matrix && child2_type == VarType::Scalar {
                    VarType::Matrix
                } else {
                    println!("Subtype");
                    println!("child1_type: {:?}", child1_type);
                    println!("child2_type: {:?}", child2_type);
                    println!("child1: {:?}", child1);
                    println!("child2: {:?}", child2);
                    panic!("Subtraction of different types is not allowed in this context")
                }
            }
            ASTNode::AtVec(_, _) => VarType::Scalar,
            ASTNode::AtMat(_, _, _) => VarType::Scalar,
            ASTNode::Neg(child) => child.infer_type(),
            ASTNode::Cross(_, _) => VarType::Vector,
            ASTNode::Transpose(_) => VarType::Matrix,
        }
    }

    /// Define this node with a given name, print the definition, and return a Variable node.
    pub fn define(&self, name: &str) -> ASTNode {
        let mut new_name = name.to_string();
        // check if name is already defined
        if search_var_name(name) == true {
            new_name = get_new_name(name);
        }

        // Print definition
            println!("val {} = {}", new_name, self);
            add_var_name(&new_name);
        

        // Return a variable node that references this name and its inferred type
        let var_type = self.infer_type();
        match var_type {
            VarType::Scalar => ASTNode::VariableS{name: new_name},
            VarType::Vector => ASTNode::VariableV{name: new_name},
            VarType::Matrix => ASTNode::VariableM{name: new_name},
        }
    }
}

fn is_scalar(node: &ASTNode) -> bool {
    match node {
        ASTNode::Scalar(_) => true,
        ASTNode::VariableS { .. } => true,
        ASTNode::AtVec(_, _) => true,
        ASTNode::AtMat(_, _, _) => true,
        _ => false,
    }
}


// TODO: I will collect all the sizes of variables in a place, and use this function to get the size of a variable
fn get_size(name: &str) -> usize {
    let mut size = 0;
    for c in name.chars() {
        if c.is_digit(10) {
            size = size * 10 + c.to_digit(10).unwrap() as usize;
        }
    }
    size
}


impl fmt::Display for ASTNode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ASTNode::Scalar(value) => write!(f, "{}", value),
            ASTNode::VariableS { name } => write!(f, "{}", name),
            ASTNode::VariableV { name } => write!(f, "{}", name),
            ASTNode::VariableM { name } => write!(f, "{}", name),
            ASTNode::Vector(nodes) => {
                // Print vectors in a single line
                let elements = nodes.iter()
                    .map(|n| n.to_string())
                    .collect::<Vec<_>>()
                    .join(", ");
                write!(f, "Vector(List({}))", elements)
            }
            ASTNode::Matrix(rows) => {
                // Pretty print the matrix in multiple lines
                writeln!(f, "Matrix(List(")?;
                for (i, row) in rows.iter().enumerate() {
                    // Join the row elements
                    let row_elems = row.iter()
                        .map(|n| n.to_string())
                        .collect::<Vec<_>>()
                        .join(", ");
                    if i < rows.len() - 1 {
                        // For all but the last row, print a trailing comma
                        writeln!(f, "            List({}),", row_elems)?;
                    } else {
                        // Last row, no trailing comma
                        writeln!(f, "            List({})", row_elems)?;
                    }
                }
                write!(f, "        ))")
            }
            ASTNode::Add(lhs, rhs) => write!(f, "({} + {})", lhs, rhs),
            ASTNode::Sub(lhs, rhs) => write!(f, "({} - {})", lhs, rhs),
            ASTNode::Mul(lhs, rhs) => {
                if is_scalar(lhs) || is_scalar(rhs) {
                    // if one of them is not scalar, we need to write it like non_scalar * scalar
                    // because of Scala and Daisy...
                    if is_scalar(lhs) {
                        write!(f, "{} * ({})", rhs, lhs)
                    } else {
                        write!(f, "{} * ({})", lhs, rhs)
                    }
                }
                else {
                    write!(f, "{}.x({})", lhs, rhs)
                }
            }
            ASTNode::AtVec(base, i) => write!(f, "{}.at({})", base, i),
            ASTNode::AtMat(base, r, c) => write!(f, "{}.at({}, {})", base, r, c),
            ASTNode::Neg(child) => write!(f, "(-{})", child),
            ASTNode::Cross(lhs, rhs) => write!(f, "{}.x({})", lhs, rhs),
            ASTNode::Transpose(child) => write!(f, "{}.transpose()", child),
        }
    }
}
