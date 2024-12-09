use crate::config::{get_mode, Mode, TEMP_COUNTER};
use std::ops::{Add, Sub, Mul};
use crate::Vector;



#[derive(Debug, Clone)]
pub struct Matrix {
    pub data: Vec<Vec<f64>>, // 2D vector for matrix data
    pub name: String,        // Name of the matrix
    pub expression: String,  // Expression representing the operation
}


impl Matrix {
    // Constructor for Matrix
    pub fn new(data: Vec<Vec<f64>>, name: &str) -> Self {
        let matrix = Matrix {
            data,
            name: name.to_string(),
            expression: name.to_string(), // Initial expression is the name itself
        };

        // Log initialization in ANALYSIS mode
        if let Mode::ANALYSIS = get_mode() {
            if !name.is_empty() {
                let elements = matrix
                    .data
                    .iter()
                    .map(|row| format!("List({})", row.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(", ")))
                    .collect::<Vec<_>>()
                    .join(", ");
                println!("val {} = Matrix(List({}))", name, elements);
            }
        }

        matrix
    }

    // Assign a new name to the matrix and log the operation
    pub fn with_name(mut self, name: &str) -> Self {
        if let Mode::ANALYSIS = get_mode() {
            if !self.expression.is_empty() && self.expression != self.name {
                println!("val {} = {}", name, self.expression);
            } else {
                // If it's already a temporary result, explicitly assign the name
                println!("val {} = {}", name, self.name);
            }
        }

        self.name = name.to_string();
        self.expression.clear(); // Clear the expression after logging
        self
    }

    // Generate a temporary name for intermediate results
    fn temp_name() -> String {
        let mut counter = TEMP_COUNTER.lock().unwrap();
        let name = format!("temp{}", *counter);
        *counter += 1;
        name
    }

    // Helper to create a new matrix with an updated expression
    fn create_result(data: Vec<Vec<f64>>, left: &Matrix, op: &str, right: &Matrix) -> Matrix {
        let temp_name = Matrix::temp_name();

        if let Mode::ANALYSIS = get_mode() {
            let left_expr = if left.expression.is_empty() {
                left.name.clone()
            } else {
                left.expression.clone()
            };

            let right_expr = if right.expression.is_empty() {
                right.name.clone()
            } else {
                right.expression.clone()
            };

            println!("val {} = {} {} {}", temp_name, left_expr, op, right_expr);
        }

        Matrix {
            data,
            name: temp_name.clone(), // Clone temp_name for the name field
            expression: temp_name,   // Use the original temp_name for the expression field
        }
    }

    // Perform addition of two matrices
    fn add_internal(&self, other: &Matrix) -> Vec<Vec<f64>> {
        self.data
            .iter()
            .zip(&other.data)
            .map(|(row1, row2)| row1.iter().zip(row2).map(|(a, b)| a + b).collect())
            .collect()
    }

    // Perform subtraction of two matrices
    fn subtract_internal(&self, other: &Matrix) -> Vec<Vec<f64>> {
        self.data
            .iter()
            .zip(&other.data)
            .map(|(row1, row2)| row1.iter().zip(row2).map(|(a, b)| a - b).collect())
            .collect()
    }

    // Matrix × Matrix multiplication
    pub fn mul_matrix(&self, other: &Matrix) -> Matrix {
        // Validate dimensions
        if self.data[0].len() != other.data.len() {
            panic!("Matrix dimensions do not align for multiplication.");
        }

        let result_data = self
            .data
            .iter()
            .map(|row| {
                (0..other.data[0].len())
                    .map(|j| {
                        row.iter()
                            .zip(other.data.iter().map(|r| r[j]))
                            .map(|(a, b)| a * b)
                            .sum()
                    })
                    .collect()
            })
            .collect();

        let temp_name = Matrix::temp_name();

        if let Mode::ANALYSIS = get_mode() {
            println!("val {} = {}.x({})", temp_name, self.name, other.name);
        }

        Matrix {
            data: result_data,
            name: temp_name.clone(),
            expression: temp_name,
        }
    }

    // Matrix × Vector multiplication (treat Vector as a 1-column matrix)
    pub fn mul_vector(&self, vector: &Vector) -> Vector {
        // Validate dimensions
        if self.data[0].len() != vector.data.len() {
            panic!("Matrix and Vector dimensions do not align for multiplication.");
        }

        let result_data = self
            .data
            .iter()
            .map(|row| {
                row.iter()
                    .zip(&vector.data)
                    .map(|(a, b)| a * b)
                    .sum()
            })
            .collect();

        let res_vector = Vector::new_tempname(result_data);

        if let Mode::ANALYSIS = get_mode() {
            println!("val {} = {}.x({})", res_vector.name, self.name, vector.name);
        }

        res_vector

    }
}

// Overload the `+` operator for matrix addition
impl<'a, 'b> Add<&'b Matrix> for &'a Matrix {
    type Output = Matrix;

    fn add(self, other: &'b Matrix) -> Matrix {
        let data = self.add_internal(other);
        Matrix::create_result(data, self, "+", other)
    }
}

// Overload the `-` operator for matrix subtraction
impl<'a, 'b> Sub<&'b Matrix> for &'a Matrix {
    type Output = Matrix;

    fn sub(self, other: &'b Matrix) -> Matrix {
        let data = self.subtract_internal(other);
        Matrix::create_result(data, self, "-", other)
    }
}

// Overload `*` operator for Matrix × Matrix
impl<'a, 'b> Mul<&'b Matrix> for &'a Matrix {
    type Output = Matrix;

    fn mul(self, other: &'b Matrix) -> Matrix {
        self.mul_matrix(other)
    }
}

// Overload `*` operator for Matrix × Vector
impl<'a, 'b> Mul<&'b Vector> for &'a Matrix {
    type Output = Vector;

    fn mul(self, vector: &'b Vector) -> Vector {
        self.mul_vector(vector)
    }
}




