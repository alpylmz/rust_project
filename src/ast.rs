

use std::fmt;
use crate::helper::{add_var_name, get_new_name, search_var_name, VarType, add_var, get_size, ALL_VARS};
use std::ops::{Add, Mul, Neg, Sub, Div};
use crate::config::UNROLL;
use std::io::Write;



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
    Div(Box<ASTNode>, Box<ASTNode>),
    Cross(Box<ASTNode>, Box<ASTNode>),
    Dot(Box<ASTNode>, Box<ASTNode>),
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

impl Div for ASTNode {
    type Output = ASTNode;
    fn div(self, rhs: ASTNode) -> ASTNode {
        ASTNode::Div(Box::new(self), Box::new(rhs))
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

// add for references
impl<'a, 'b> Add<&'b ASTNode> for &'a ASTNode {
    type Output = ASTNode;
    fn add(self, rhs: &'b ASTNode) -> ASTNode {
        ASTNode::Add(Box::new(self.clone()), Box::new(rhs.clone()))
    }
}
// add for one ref
impl<'a> Add<ASTNode> for &'a ASTNode {
    type Output = ASTNode;
    fn add(self, rhs: ASTNode) -> ASTNode {
        ASTNode::Add(Box::new(self.clone()), Box::new(rhs))
    }
}
// add for the other ref
impl<'a> Add<&'a ASTNode> for ASTNode {
    type Output = ASTNode;
    fn add(self, rhs: &'a ASTNode) -> ASTNode {
        ASTNode::Add(Box::new(self), Box::new(rhs.clone()))
    }
}

// Now, implement Add for &mut ASTNode by reborrowing as &ASTNode.
impl<'a, 'b> Add<&'b ASTNode> for &'a mut ASTNode {
    type Output = ASTNode;

    fn add(self, rhs: &'b ASTNode) -> ASTNode {
        // Reborrow self as an immutable reference
        (&*self) + rhs
    }
}

// &ASTNode + ASTNode
impl<'a> Sub<ASTNode> for &'a ASTNode {
    type Output = ASTNode;
    fn sub(self, rhs: ASTNode) -> ASTNode {
        self.clone() - rhs
    }
}

// ASTNode + &ASTNode
impl<'b> Sub<&'b ASTNode> for ASTNode {
    type Output = ASTNode;
    fn sub(self, rhs: &'b ASTNode) -> ASTNode {
        self - rhs.clone()
    }
}

// &ASTNode + &ASTNode
impl<'a, 'b> Sub<&'b ASTNode> for &'a ASTNode {
    type Output = ASTNode;
    fn sub(self, rhs: &'b ASTNode) -> ASTNode {
        self.clone() - rhs.clone()
    }
}

// ========================================
// Multiplication implementations
// ========================================

// &ASTNode + ASTNode
impl<'a> Mul<ASTNode> for &'a ASTNode {
    type Output = ASTNode;
    fn mul(self, rhs: ASTNode) -> ASTNode {
        self.clone() * rhs
    }
}

// ASTNode + &ASTNode
impl<'b> Mul<&'b ASTNode> for ASTNode {
    type Output = ASTNode;
    fn mul(self, rhs: &'b ASTNode) -> ASTNode {
        self * rhs.clone()
    }
}

// &ASTNode + &ASTNode
impl<'a, 'b> Mul<&'b ASTNode> for &'a ASTNode {
    type Output = ASTNode;
    fn mul(self, rhs: &'b ASTNode) -> ASTNode {
        self.clone() * rhs.clone()
    }
}

// ========================================
// Division implementations
// ========================================

// &ASTNode + ASTNode
impl<'a> Div<ASTNode> for &'a ASTNode {
    type Output = ASTNode;
    fn div(self, rhs: ASTNode) -> ASTNode {
        self.clone() / rhs
    }
}

// ASTNode + &ASTNode
impl<'b> Div<&'b ASTNode> for ASTNode {
    type Output = ASTNode;
    fn div(self, rhs: &'b ASTNode) -> ASTNode {
        self / rhs.clone()
    }
}

// &ASTNode + &ASTNode
impl<'a, 'b> Div<&'b ASTNode> for &'a ASTNode {
    type Output = ASTNode;
    fn div(self, rhs: &'b ASTNode) -> ASTNode {
        self.clone() / rhs.clone()
    }
}

impl<'a> Neg for &'a ASTNode {
    type Output = ASTNode;
    fn neg(self) -> ASTNode {
        ASTNode::Neg(Box::new(self.clone()))
    }
}



impl ASTNode {
    pub fn at_vec(&self, i: usize) -> ASTNode {
        ASTNode::AtVec(Box::new(self.clone()), i)
    }

    pub fn cross(&self, rhs: &ASTNode) -> ASTNode {
        ASTNode::Cross(Box::new(self.clone()), Box::new(rhs.clone()))
    }

    pub fn dot(&self, rhs: &ASTNode) -> ASTNode {
        ASTNode::Dot(Box::new(self.clone()), Box::new(rhs.clone()))
    }

    pub fn transpose(&self) -> ASTNode {
        ASTNode::Transpose(Box::new(self.clone()))
    }

    pub fn at_mat(&self, r: usize, c: usize) -> ASTNode {
        ASTNode::AtMat(Box::new(self.clone()), r, c)
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
                    eprintln!("Multype");
                    eprintln!("child1_type: {:?}", child1_type);
                    eprintln!("child2_type: {:?}", child2_type);
                    eprintln!("child1: {:?}", child1);
                    eprintln!("child2: {:?}", child2);
                    // flush stdout
                    std::io::stdout().flush().unwrap();
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
            ASTNode::Div(child1, child2) => {
                // Division is the same as multiplication
                let child1_type = child1.infer_type();
                let child2_type = child2.infer_type();
                if child1_type == VarType::Scalar && child2_type == VarType::Scalar {
                    VarType::Scalar
                } else {
                    println!("Divtype");
                    println!("child1_type: {:?}", child1_type);
                    println!("child2_type: {:?}", child2_type);
                    println!("child1: {:?}", child1);
                    println!("child2: {:?}", child2);
                    panic!("Division of different types is not allowed in this context")
                }
            }
            ASTNode::AtVec(_, _) => VarType::Scalar,
            ASTNode::AtMat(_, _, _) => VarType::Scalar,
            ASTNode::Neg(child) => child.infer_type(),
            ASTNode::Cross(child1, child2) => {
                let child1_type = child1.infer_type();
                let child2_type = child2.infer_type();

                if child1_type == VarType::Vector && child2_type == VarType::Vector {
                    VarType::Vector
                } else if child1_type == VarType::Matrix && child2_type == VarType::Matrix {
                    VarType::Matrix
                } else if child1_type == VarType::Matrix && child2_type == VarType::Vector {
                    VarType::Vector
                } else {
                    panic!("Cross product of different types is not allowed in this context")
                }
            }
            ASTNode::Dot(child1, child2) => {
                let child1_type = child1.infer_type();
                let child2_type = child2.infer_type();
                if child1_type == VarType::Vector && child2_type == VarType::Vector {
                    VarType::Scalar
                } else {
                    panic!("Dot product of different types is not allowed in this context")
                }
            }
            ASTNode::Transpose(_) => VarType::Matrix,
        }
    }

    fn unroll_opr(&self, lhs: &Box<ASTNode>, rhs: &Box<ASTNode>, new_name: &str, opr: &str) {
        match opr {
            "+" | "-" => {
                if is_scalar(lhs) && is_scalar(rhs) {
                    println!("val {} = {} {} {}", new_name, lhs, opr, rhs);
                    add_var(&new_name, vec![1]);
                    add_var_name(&new_name);
                }
                else if is_scalar(lhs) && is_vector(rhs) {
                    let size = get_size(&rhs.to_string());
                    if size.len() != 1 {
                        panic!("Vector length should have 1 dimension, for variable {} it has {}", rhs, size.len());
                    }
                    add_var(&new_name, vec![size[0]]);
                    for i in 0..size[0] {
                        let new_name = format!("{}_{}", new_name, i);
                        add_var(&new_name, vec![1]);
                        println!("val {} = {} {} {}_{}", new_name, lhs, opr, rhs, i);
                        add_var_name(&new_name);
                    }
                }
                else if is_scalar(lhs) && is_matrix(rhs) {
                    let size = get_size(&rhs.to_string());
                    if size.len() != 2 {
                        panic!("Matrix should have 2 dimensions, for variable {} it has {}", rhs, size.len());
                    }
                    add_var(&new_name, vec![size[0], size[1]]);
                    for i in 0..size[0] {
                        for j in 0..size[1] {
                            let new_name = format!("{}_{}_{}", new_name, i, j);
                            add_var(&new_name, vec![1]);
                            println!("val {} = {} {} {}_{}_{}", new_name, lhs, opr, rhs, i, j);
                            add_var_name(&new_name);
                        }
                    }
                }
                else if is_vector(lhs) && is_scalar(rhs) {
                    let size = get_size(&lhs.to_string());
                    if size.len() != 1 {
                        panic!("Vector length should have 1 dimension, for variable {} it has {}", lhs, size.len());
                    }
                    add_var(&new_name, vec![size[0]]);
                    for i in 0..size[0] {
                        let new_name = format!("{}_{}", new_name, i);
                        add_var(&new_name, vec![1]);
                        println!("val {} = {}_{} {} {}", new_name, lhs, i, opr, rhs);
                        add_var_name(&new_name);
                    }
                }
                else if is_matrix(lhs) && is_scalar(rhs) {
                    let size = get_size(&lhs.to_string());
                    if size.len() != 2 {
                        panic!("Matrix should have 2 dimensions, for variable {} it has {}", lhs, size.len());
                    }
                    add_var(&new_name, vec![size[0], size[1]]);
                    for i in 0..size[0] {
                        for j in 0..size[1] {
                            let new_name = format!("{}_{}_{}", new_name, i, j);
                            add_var(&new_name, vec![1]);
                            println!("val {} = {}_{}_{} {} {}", new_name, lhs, i, j, opr, rhs);
                            add_var_name(&new_name);
                        }
                    }
                }
                else if is_vector(lhs) && is_vector(rhs) {
                    let size = get_size(&lhs.to_string());
                    let size2 = get_size(&rhs.to_string());
                    if size.len() != 1 {
                        panic!("Vector length should have 1 dimension, for variable {} it has {}", lhs, size.len());
                    }
                    if size2.len() != 1 {
                        let all_vars = ALL_VARS.lock().unwrap();
                        panic!("Vector length should have 1 dimension, for variable {} it has {} in context {:?}", rhs, size2.len(), all_vars);
                    }
                    if size != size2 {
                        let all_vars = ALL_VARS.lock().unwrap();
                        panic!("Vectors should have the same size, for variable {} it has {:#?} and for variable {} it has {:#?} in context {:?}", lhs, size, rhs, size2, all_vars);
                    }
                    add_var(&new_name, vec![size[0]]);
                    for i in 0..size[0] {
                        let new_name = format!("{}_{}", new_name, i);
                        add_var(&new_name, vec![1]);
                        println!("val {} = {}_{} {} {}_{}", new_name, lhs, i, opr, rhs, i);
                        add_var_name(&new_name);
                    }
                }
                else if is_matrix(lhs) && is_matrix(rhs) {
                    let size = get_size(&lhs.to_string());
                    let size2 = get_size(&rhs.to_string());
                    if size.len() != 2 {
                        panic!("Matrix should have 2 dimensions, for variable {} it has {}", lhs, size.len());
                    }
                    if size2.len() != 2 {
                        panic!("Matrix should have 2 dimensions, for variable {} it has {}", rhs, size2.len());
                    }
                    if size != size2 {
                        panic!("Matrices should have the same size, for variable {} it has {:#?} and for variable {} it has {:#?}", lhs, size, rhs, size2);
                    }
                    add_var(&new_name, vec![size[0], size[1]]);
                    for i in 0..size[0] {
                        for j in 0..size[1] {
                            let new_name = format!("{}_{}_{}", new_name, i, j);
                            add_var(&new_name, vec![1]);
                            println!("val {} = {}_{}_{} {} {}_{}_{}", new_name, lhs, i, j, opr, rhs, i, j);
                            add_var_name(&new_name);
                        }
                    }
                }
                else {
                    let all_vars = ALL_VARS.lock().unwrap();
                    eprintln!("lhs: {:?}", lhs);
                    eprintln!("rhs: {:?}", rhs);
                    eprintln!("lhs type: {:?}", lhs.get_type());
                    eprintln!("rhs type: {:?}", rhs.get_type());
                    panic!("Addition/Subtraction of different types, {} and {} is not allowed in this context {:?}", lhs, rhs, all_vars);
                }
            }
            "/" => {
                if is_scalar(lhs) && is_scalar(rhs) {
                    println!("val {} = {} {} {}", new_name, lhs, opr, rhs);
                    add_var(&new_name, vec![1]);
                    add_var_name(&new_name);
                }
                else{
                    let all_vars = ALL_VARS.lock().unwrap();
                    panic!("Division of {} and {} is not allowed in this context {:?}", lhs, rhs, all_vars);
                }
            }
            "*" => { // element-wise multiplication
                if is_scalar(lhs) && is_scalar(rhs) {
                    println!("val {} = {} * {}", new_name, lhs, rhs);
                    add_var(&new_name, vec![1]);
                    add_var_name(&new_name);
                }
                else if is_scalar(lhs) && is_vector(rhs) {
                    let size = get_size(&rhs.to_string());
                    if size.len() != 1 {
                        panic!("Vector length should have 1 dimension, for variable {} it has {}", rhs, size.len());
                    }
                    add_var(&new_name, vec![size[0]]);
                    for i in 0..size[0] {
                        let new_name = format!("{}_{}", new_name, i);
                        add_var(&new_name, vec![1]);
                        println!("val {} = {} * {}_{}", new_name, lhs, rhs, i);
                        add_var_name(&new_name);
                    }
                }
                else if is_scalar(lhs) && is_matrix(rhs) {
                    let size = get_size(&rhs.to_string());
                    if size.len() != 2 {
                        panic!("Matrix should have 2 dimensions, for variable {} it has {}", rhs, size.len());
                    }
                    add_var(&new_name, vec![size[0], size[1]]);
                    for i in 0..size[0] {
                        for j in 0..size[1] {
                            let new_name = format!("{}_{}_{}", new_name, i, j);
                            add_var(&new_name, vec![1]);
                            println!("val {} = {} * {}_{}_{}", new_name, lhs, rhs, i, j);
                            add_var_name(&new_name);
                        }
                    }
                }
                else if is_vector(lhs) && is_scalar(rhs) {
                    let size = get_size(&lhs.to_string());
                    if size.len() != 1 {
                        panic!("Vector length should have 1 dimension, for variable {} it has {}", lhs, size.len());
                    }
                    add_var(&new_name, vec![size[0]]);
                    for i in 0..size[0] {
                        let new_name = format!("{}_{}", new_name, i);
                        add_var(&new_name, vec![1]);
                        println!("val {} = {}_{} * {}", new_name, lhs, i, rhs);
                        add_var_name(&new_name);
                    }
                }
                else if is_matrix(lhs) && is_scalar(rhs) {
                    let size = get_size(&lhs.to_string());
                    if size.len() != 2 {
                        panic!("Matrix should have 2 dimensions, for variable {} it has {}", lhs, size.len());
                    }
                    add_var(&new_name, vec![size[0], size[1]]);
                    for i in 0..size[0] {
                        for j in 0..size[1] {
                            let new_name = format!("{}_{}_{}", new_name, i, j);
                            add_var(&new_name, vec![1]);
                            println!("val {} = {}_{}_{} * {}", new_name, lhs, i, j, rhs);
                            add_var_name(&new_name);
                        }
                    }
                }
                else if is_vector(lhs) && is_vector(rhs) {
                    let size = get_size(&lhs.to_string());
                    let size2 = get_size(&rhs.to_string());
                    if size.len() != 1 {
                        panic!("Vector length should have 1 dimension, for variable {} it has {}", lhs, size.len());
                    }
                    if size2.len() != 1 {
                        panic!("Vector length should have 1 dimension, for variable {} it has {}", rhs, size2.len());
                    }
                    if size != size2 {
                        panic!("Vectors should have the same size, for variable {} it has {:#?} and for variable {} it has {:#?}", lhs, size, rhs, size2);
                    }
                    add_var(&new_name, vec![size[0]]);
                    for i in 0..size[0] {
                        let new_name = format!("{}_{}", new_name, i);
                        add_var(&new_name, vec![1]);
                        println!("val {} = {}_{} * {}_{}", new_name, lhs, i, rhs, i);
                        add_var_name(&new_name);
                    }
                }
                else if is_matrix(lhs) && is_matrix(rhs) {
                    let size = get_size(&lhs.to_string());
                    let size2 = get_size(&rhs.to_string());
                    if size.len() != 2 {
                        panic!("Matrix should have 2 dimensions, for variable {} it has {}", lhs, size.len());
                    }
                    if size2.len() != 2 {
                        panic!("Matrix should have 2 dimensions, for variable {} it has {}", rhs, size2.len());
                    }
                    if size != size2 {
                        panic!("Matrices should have the same size, for variable {} it has {:#?} and for variable {} it has {:#?}", lhs, size, rhs, size2);
                    }
                    add_var(&new_name, vec![size[0], size[1]]);
                    for i in 0..size[0] {
                        for j in 0..size[1] {
                            let new_name = format!("{}_{}_{}", new_name, i, j);
                            add_var(&new_name, vec![1]);
                            println!("val {} = {}_{}_{} * {}_{}_{}", new_name, lhs, i, j, rhs, i, j);
                            add_var_name(&new_name);
                        }
                    }
                }
                else{
                    let all_vars = ALL_VARS.lock().unwrap();
                    panic!("Multiplication of {} and {} is not allowed in this context {:?}", lhs, rhs, all_vars);
                }
            }
            "x" => { // cross product or matrix multiplication
                if is_vector(lhs) && is_vector(rhs) {
                    // implement cross product in that case
                    let size = get_size(&lhs.to_string());
                    let size2 = get_size(&rhs.to_string());
                    if size.len() != 1 {
                        panic!("Vector length should have 1 dimension, for variable {} it has {}", lhs, size.len());
                    }
                    if size2.len() != 1 {
                        panic!("Vector length should have 1 dimension, for variable {} it has {}", rhs, size2.len());
                    }
                    if size[0] != 3 || size2[0] != 3 {
                        panic!("Cross product is only allowed for vectors of size 3");
                    }
                    add_var(&new_name, vec![3]);
                    let old_name = new_name;
                    let new_name = format!("{}_0", new_name);
                    // implement cross product manually
                    // lhs_1 * rhs_2 - lhs_2 * rhs_1
                    println!("val {} = {}_1 * {}_2 - {}_2 * {}_1", new_name, lhs, rhs, lhs, rhs);
                    add_var(&new_name, vec![1]);
                    add_var_name(&new_name);

                    let new_name = format!("{}_1", old_name);
                    // lhs_2 * rhs_0 - lhs_0 * rhs_2
                    println!("val {} = {}_2 * {}_0 - {}_0 * {}_2", new_name, lhs, rhs, lhs, rhs);
                    add_var(&new_name, vec![1]);
                    add_var_name(&new_name);

                    let new_name = format!("{}_2", old_name);
                    // lhs_0 * rhs_1 - lhs_1 * rhs_0
                    println!("val {} = {}_0 * {}_1 - {}_1 * {}_0", new_name, lhs, rhs, lhs, rhs);
                    add_var(&new_name, vec![1]);
                    add_var_name(&new_name);
                }
                else if is_matrix(lhs) && is_matrix(rhs) {
                    let size1 = get_size(&lhs.to_string());
                    let size2 = get_size(&rhs.to_string());
                    // matrix multiplication
                    let size1_row = size1[0];
                    let size1_col = size1[1];
                    let size2_row = size2[0];
                    let size2_col = size2[1];
                    if size1_col != size2_row {
                        panic!("Matrix multiplication is only allowed if the number of columns of the first matrix is equal to the number of rows of the second matrix");
                    }
                    add_var(&new_name, vec![size1_row, size2_col]);
                    for i in 0..size1_row {
                        for j in 0..size2_col {
                            let new_name = format!("{}_{}_{}", new_name, i, j);
                            let mut elements = Vec::new();
                            for k in 0..size1_col {
                                let element = format!("{}_{}_{}", lhs, i, k);
                                let element2 = format!("{}_{}_{}", rhs, k, j);
                                elements.push(format!("{} * {}", element, element2));
                            }
                            println!("val {} = {}", new_name, elements.join(" + "));
                            add_var(&new_name, vec![1]);
                            add_var_name(&new_name);
                        }
                    }
                }
                else if is_matrix(lhs) && is_vector(rhs) {
                    // In this case you can assume that the vector is a column vector
                    let size1 = get_size(&lhs.to_string());
                    let size2 = get_size(&rhs.to_string());
                    if size1[1] != size2[0] {
                        panic!("Matrix multiplication is only allowed if the number of columns of the first matrix is equal to the number of rows of the second matrix");
                    }
                    add_var(&new_name, vec![size1[0]]);
                    for i in 0..size1[0] {
                        let new_name = format!("{}_{}", new_name, i);
                        let mut elements = Vec::new();
                        for k in 0..size1[1] {
                            let element = format!("{}_{}_{}", lhs, i, k);
                            let element2 = format!("{}_{}", rhs, k);
                            elements.push(format!("{} * {}", element, element2));
                        }
                        println!("val {} = {}", new_name, elements.join(" + "));
                        add_var(&new_name, vec![1]);
                        add_var_name(&new_name);
                    }
                }
                else{
                    let all_vars = ALL_VARS.lock().unwrap();
                    panic!("Cross product or matrix multiplication of {}: {} and {}: {} is not allowed in this context {:?}", lhs, lhs.get_type(), rhs, rhs.get_type(), all_vars);
                }
            }
            _ => {
                panic!("Invalid operator");
            }
        }
    }

    fn unroll_print(&self, new_name: &str) {
        match self {
            ASTNode::Scalar(value) => {
                println!("val {}: Real = {}", new_name, value);
                add_var(&new_name, vec![1]);
                add_var_name(&new_name);
            }
            ASTNode::VariableS { name } => {
                println!("val {}: Real = {}", new_name, name);
                add_var(&new_name, vec![1]);
                add_var_name(&new_name);
            }
            ASTNode::VariableV { name } => {
                // get the size of the vector
                let size = get_size(name);
                // if size length is not 1, we have a type mismatch
                if size.len() != 1 {
                    panic!("Vector length should have 1 dimension, for variable {} it has {}", name, size.len());
                }
                add_var(&new_name, vec![size[0]]);
                for i in 0..size[0] {
                    let new_name = format!("{}_{}", new_name, i);
                    println!("val {}: Real = {}_{}", new_name, name, i);
                    add_var_name(&new_name);
                }
            }
            ASTNode::VariableM { name } => {
                // get the size of the matrix
                let size = get_size(name);
                // if size length is not 2, we have a type mismatch
                if size.len() != 2 {
                    panic!("Matrix should have 2 dimensions, for variable {} it has {}", name, size.len());
                }
                add_var(&new_name, vec![size[0], size[1]]);
                for i in 0..size[0] {
                    for j in 0..size[1] {
                        let new_name = format!("{}_{}_{}", new_name, i, j);
                        println!("val {}: Real = {}_{}_{}", new_name, name, i, j);
                        add_var_name(&new_name);
                    }
                }
            }
            ASTNode::Vector(nodes) => {
                let size = nodes.len();
                add_var(&new_name, vec![size]);
                for (i, node) in nodes.iter().enumerate() {
                    let new_name = format!("{}_{}", new_name, i);
                    add_var(&new_name, vec![1]);
                    println!("val {}: Real = {}", new_name, node);
                    add_var_name(&new_name);
                }
            }
            ASTNode::Matrix(rows) => {
                let size = rows.len();
                let size_col = rows[0].len();
                add_var(&new_name, vec![size, size_col]);
                for (i, row) in rows.iter().enumerate() {
                    for (j, node) in row.iter().enumerate() {
                        let new_name = format!("{}_{}_{}", new_name, i, j);
                        println!("val {}: Real = {}", new_name, node);
                        add_var_name(&new_name);
                    }
                }
            }
            ASTNode::Add(ref lhs, ref rhs) => {
                self.unroll_opr(lhs, rhs, new_name, "+");
            }
            ASTNode::Sub(lhs, rhs) => {
                self.unroll_opr(lhs, rhs, new_name, "-");
            }
            ASTNode::Mul(lhs, rhs) => {
                self.unroll_opr(lhs, rhs, new_name, "*");
            }
            ASTNode::Div(lhs, rhs) => {
                self.unroll_opr(lhs, rhs, new_name, "/");
            }
            ASTNode::AtVec(base, i) => {
                let new_name = format!("{}", new_name);
                add_var(&new_name, vec![1]);
                println!("val {}: Real = {}_{}", new_name, base, i);
                add_var_name(&new_name);
            }
            ASTNode::AtMat(base, r, c) => {
                let new_name = format!("{}_{}_{}", new_name, r, c);
                add_var(&new_name, vec![1]);
                println!("val {}: Real = {}_{}_{}", new_name, base, r, c);
                add_var_name(&new_name);
            }
            ASTNode::Neg(child) => {
                let new_name = format!("{}", new_name);
                let size = get_size(&child.to_string());
                add_var(&new_name, size);
                println!("val {}: Real = -{}", new_name, child);
                add_var_name(&new_name);
            }
            ASTNode::Cross(lhs, rhs) => {
                self.unroll_opr(lhs, rhs, new_name, "x");
            }
            ASTNode::Dot(lhs, rhs) => {
                let size = get_size(&lhs.to_string());
                let size2 = get_size(&rhs.to_string());
                if size.len() != 1 {
                    panic!("Vector length should have 1 dimension, for variable {} it has {}", lhs, size.len());
                }
                if size2.len() != 1 {
                    panic!("Vector length should have 1 dimension, for variable {} it has {}", rhs, size2.len());
                }
                if size != size2 {
                    panic!("Vectors should have the same size, for variable {} it has {:#?} and for variable {} it has {:#?}", lhs, size, rhs, size2);
                }
                add_var(&new_name, vec![1]);
                let mut elements = Vec::new();
                for i in 0..size[0] {
                    let element = format!("{}_{}", lhs, i);
                    let element2 = format!("{}_{}", rhs, i);
                    elements.push(format!("{} * {}", element, element2));
                }
                println!("val {}: Real = {}", new_name, elements.join(" + "));
                add_var_name(&new_name);
            }
            ASTNode::Transpose(child) => {
                let new_name = format!("{}", new_name);
                match &**child {
                    ASTNode::VariableM {name} => {
                        let size = get_size(name);
                        if size.len() != 2 {
                            panic!("Matrix should have 2 dimensions, for variable {} it has {}", name, size.len());
                        }
                        add_var(&new_name, vec![size[1], size[0]]);
                        for i in 0..size[0] {
                            for j in 0..size[1] {
                                let new_name = format!("{}_{}_{}", new_name, j, i);
                                println!("val {}: Real = {}_{}_{}", new_name, name, i, j);
                                add_var_name(&new_name);
                            }
                        }
                    }
                    _ => {
                        panic!("Transpose is only allowed for matrices");
                    }
                }
            }
        }
    }

    /// Define this node with a given name, print the definition, and return a Variable node.
    pub fn define(&self, name: &str) -> ASTNode {
        let mut new_name = name.to_string();
        // check if name is already defined
        if search_var_name(name) == true || new_name == "" {
            new_name = get_new_name(name);
        }

        // Print definition
        if !UNROLL{
            println!("val {} = {}", new_name, self);
        }
        else{
            self.unroll_print(&new_name);
        }

        // Return a variable node that references this name and its inferred type
        let var_type = self.infer_type();
        match var_type {
            VarType::Scalar => ASTNode::VariableS{name: new_name},
            VarType::Vector => ASTNode::VariableV{name: new_name},
            VarType::Matrix => ASTNode::VariableM{name: new_name},
        }
    }

    // Function to use for debugging purposes, just print the type of the node
    fn get_type(&self) -> String {
        match self {
            ASTNode::Scalar(_) => "Scalar".to_string(),
            ASTNode::VariableS { .. } => "VariableS".to_string(),
            ASTNode::VariableV { .. } => "VariableV".to_string(),
            ASTNode::VariableM { .. } => "VariableM".to_string(),
            ASTNode::Vector(_) => "Vector".to_string(),
            ASTNode::Matrix(_) => "Matrix".to_string(),
            ASTNode::Add(_, _) => "Add".to_string(),
            ASTNode::Sub(_, _) => "Sub".to_string(),
            ASTNode::Mul(_, _) => "Mul".to_string(),
            ASTNode::Div(_, _) => "Div".to_string(),
            ASTNode::AtVec(_, _) => "AtVec".to_string(),
            ASTNode::AtMat(_, _, _) => "AtMat".to_string(),
            ASTNode::Neg(_) => "Neg".to_string(),
            ASTNode::Cross(_, _) => "Cross".to_string(),
            ASTNode::Dot(_, _) => "Dot".to_string(),
            ASTNode::Transpose(_) => "Transpose".to_string(),
        }
    }

}

fn is_scalar(node: &ASTNode) -> bool {
    match node {
        ASTNode::Scalar(_) => true,
        ASTNode::VariableS { .. } => true,
        ASTNode::AtVec(_, _) => true,
        ASTNode::AtMat(_, _, _) => true,
        ASTNode::Neg(a) => is_scalar(a),
        ASTNode::Mul(lhs, rhs) => is_scalar(lhs) && is_scalar(rhs), // needs better handling, dot product etc.
        ASTNode::Add(lhs, rhs) => is_scalar(lhs) && is_scalar(rhs),
        ASTNode::Sub(lhs, rhs) => is_scalar(lhs) && is_scalar(rhs),
        ASTNode::Div(lhs, rhs) => is_scalar(lhs) && is_scalar(rhs),
        _ => false,
    }
}

fn is_vector(node: &ASTNode) -> bool {
    match node {
        ASTNode::Vector(_) => true,
        ASTNode::VariableV { .. } => true,
        ASTNode::Neg(a) => is_vector(a),
        ASTNode::Mul(lhs, rhs) => is_vector(lhs) || is_vector(rhs),
        ASTNode::Add(lhs, rhs) => is_vector(lhs) && is_vector(rhs),
        ASTNode::Sub(lhs, rhs) => is_vector(lhs) && is_vector(rhs),
        _ => false,
    }
}

fn is_matrix(node: &ASTNode) -> bool {
    match node {
        ASTNode::Matrix(_) => true,
        ASTNode::VariableM { .. } => true,
        ASTNode::Neg(a) => is_matrix(a),
        ASTNode::Mul(lhs, rhs) => is_matrix(lhs) || is_matrix(rhs),
        ASTNode::Add(lhs, rhs) => is_matrix(lhs) && is_matrix(rhs),
        ASTNode::Sub(lhs, rhs) => is_matrix(lhs) && is_matrix(rhs),
        ASTNode::Cross(lhs, rhs) => is_matrix(lhs) && is_matrix(rhs),
        _ => false,
    }
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
            ASTNode::Div(lhs, rhs) => write!(f, "({} / {})", lhs, rhs),
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
            ASTNode::AtVec(base, i) => {
                if UNROLL {
                    return write!(f, "{}_{}", base, i);
                }
                else{
                    write!(f, "{}.at({})", base, i)
                }
            }
            ASTNode::AtMat(base, r, c) => {
                if UNROLL {
                    return write!(f, "{}_{}_{}", base, r, c);
                }
                else{
                    write!(f, "{}.at({}, {})", base, r, c)
                }
            }
            ASTNode::Neg(child) => write!(f, "(-{})", child),
            ASTNode::Cross(lhs, rhs) => write!(f, "{}.x({})", lhs, rhs),
            ASTNode::Dot(lhs, rhs) => write!(f, "{}.dot({})", lhs, rhs),
            ASTNode::Transpose(child) => write!(f, "{}.transpose()", child),
        }
    }
}
