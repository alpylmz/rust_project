

use std::fmt;
use crate::helper::{VarType};
use std::ops::{Add, Mul, Neg, Sub};



#[derive(Debug, Clone)]
pub enum ASTNode {
    Scalar(f64),
    Variable { var_type: VarType, name: String },
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
            ASTNode::Variable { var_type, .. } => var_type.clone(),
            ASTNode::Vector(_) => VarType::Vector,
            ASTNode::Matrix(_) => VarType::Matrix,
            ASTNode::Add(_, _) => {
                // In a more robust system, you'd keep track of types carefully.
                // For now, assume additions result in the same type as their operands,
                // or default to Scalar if unknown.
                // This might need refinement depending on your system.
                VarType::Scalar
            }
            ASTNode::Mul(_, _) => {
                // Similarly, assume some type. For matrix multiplies often it's Matrix or Vector.
                // Adjust this logic as needed. If you need type correctness, store it explicitly.
                VarType::Matrix
            }
            ASTNode::Sub(_, _) => VarType::Scalar,
            ASTNode::AtVec(_, _) => VarType::Scalar,
            ASTNode::AtMat(_, _, _) => VarType::Scalar,
            ASTNode::Neg(child) => child.infer_type(),
            ASTNode::Cross(_, _) => VarType::Vector,
            ASTNode::Transpose(_) => VarType::Matrix,
        }
    }

    /// Define this node with a given name, print the definition, and return a Variable node.
    pub fn define(self, name: &str) -> ASTNode {
        // Print definition
        println!("val {} = {}", name, self);

        // Return a variable node that references this name and its inferred type
        let var_type = self.infer_type();
        ASTNode::Variable {
            var_type,
            name: name.to_string(),
        }
    }
}

fn is_scalar(node: &ASTNode) -> bool {
    match node {
        ASTNode::Scalar(_) => true,
        ASTNode::Variable { var_type, .. } => var_type == &VarType::Scalar,
        ASTNode::AtVec(_, _) => true,
        ASTNode::AtMat(_, _, _) => true,
        _ => false,
    }
}


impl fmt::Display for ASTNode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ASTNode::Scalar(value) => write!(f, "{}", value),
            ASTNode::Variable { name, .. } => write!(f, "{}", name),
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
                    write!(f, "{} * ({})", lhs, rhs)
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
