use crate::config::{get_mode, Mode, TEMP_COUNTER};
use std::ops::{Add, Sub};

#[derive(Debug, Clone)]
pub struct Vector {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub name: String,        // Name of the vector
    pub expression: String,  // Expression representing the operation
}


impl Vector {
    // Constructor for Vector
    pub fn new(x: f64, y: f64, z: f64, name: &str) -> Self {
        let vector = Vector {
            x,
            y,
            z,
            name: name.to_string(),
            expression: name.to_string(), // Initial expression is the name itself
        };

        // Log initialization in ANALYSIS mode
        if let Mode::ANALYSIS = get_mode() {
            println!("val {} = Vector(List({}, {}, {}))", name, x, y, z);
        }

        vector
    }

    // Assign a new name to the vector and log the operation
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

    // Helper to create a new vector with an updated expression
    fn create_result(x: f64, y: f64, z: f64, left: &Vector, op: &str, right: &Vector) -> Vector {
        let temp_name = Vector::temp_name();

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

        Vector {
            x,
            y,
            z,
            name: temp_name.clone(), // Clone temp_name for the name field
            expression: temp_name,   // Use the original temp_name for the expression field
        }
    }
}

// Overload the `+` operator for vector addition
impl<'a, 'b> Add<&'b Vector> for &'a Vector {
    type Output = Vector;

    fn add(self, other: &'b Vector) -> Vector {
        let x = self.x + other.x;
        let y = self.y + other.y;
        let z = self.z + other.z;

        Vector::create_result(x, y, z, self, "+", other)
    }
}

// Overload the `-` operator for vector subtraction
impl<'a, 'b> Sub<&'b Vector> for &'a Vector {
    type Output = Vector;

    fn sub(self, other: &'b Vector) -> Vector {
        let x = self.x - other.x;
        let y = self.y - other.y;
        let z = self.z - other.z;

        Vector::create_result(x, y, z, self, "-", other)
    }
}

// Dot product of two vectors
impl Vector {
    pub fn dot(&self, other: &Vector, result_name: &str) -> f64 {
        let result = self.x * other.x + self.y * other.y + self.z * other.z;

        if let Mode::ANALYSIS = get_mode() {
            println!("val {} = {}.dot({})", result_name, self.name, other.name);
        }

        result
    }

    // Cross product of two vectors
    pub fn cross(&self, other: &Vector, result_name: &str) -> Vector {
        let x = self.y * other.z - self.z * other.y;
        let y = self.z * other.x - self.x * other.z;
        let z = self.x * other.y - self.y * other.x;

        let expression = format!("{}.x({})", self.name, other.name);

        if let Mode::ANALYSIS = get_mode() {
            println!("val {} = {}", result_name, expression);
        }

        Vector {
            x,
            y,
            z,
            name: result_name.to_string(),
            expression,
        }
    }
}
