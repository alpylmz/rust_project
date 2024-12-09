use crate::config::{get_mode, Mode, TEMP_COUNTER};
use std::ops::{Add, Sub};

#[derive(Debug, Clone)]
pub struct Vector {
    pub data: Vec<f64>,
    pub name: String,        // Name of the vector
    pub expression: String,  // Expression representing the operation
}


impl Vector {
    // Constructor for Vector
    pub fn new(data: Vec<f64>, name: &str) -> Self {
        let vector = Vector {
            data,
            name: name.to_string(),
            expression: name.to_string(), // Initial expression is the name itself
        };

        // Log initialization in ANALYSIS mode
        if let Mode::ANALYSIS = get_mode() {
            if !name.is_empty() {
                let elements = vector.data.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(", ");
                println!("val {} = Vector(List({}))", name, elements);
            }
            else{
                panic!("Name cannot be empty");
            }
        }

        vector
    }

    // One more constructer is needed, this will be used when the vector is input, in that case we will not print the initialization
    pub fn new_input(data: Vec<f64>, name: &str) -> Self {
        let vector = Vector {
            data,
            name: name.to_string(),
            expression: name.to_string(), // Initial expression is the name itself
        };

        vector
    }

    // this constructor will assign a temporary name to the vector
    pub fn new_tempname(data: Vec<f64>) -> Self{
        let vector = Vector {
            data,
            name: Vector::temp_name(),
            expression: Vector::temp_name(),
        };

        // Log initialization in ANALYSIS mode
        if let Mode::ANALYSIS = get_mode() {
            let elements = vector.data.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(", ");
            println!("val {} = Vector({})", vector.name, elements);
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
    fn create_result(data: Vec<f64>, name: &str, expression: &str) -> Vector {
        let vector = Vector {
            data,
            name: name.to_string(),
            expression: expression.to_string(),
        };

        // Log the operation in ANALYSIS mode
        if let Mode::ANALYSIS = get_mode() {
            let elements = vector.data.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(", ");
            println!("val {} = Vector({})", name, elements);
        }

        vector
    }
}

// Overload the `+` operator for vector addition
impl<'a, 'b> Add<&'b Vector> for &'a Vector {
    type Output = Vector;

    fn add(self, other: &'b Vector) -> Vector {
        // Perform element-wise addition
        // iterate over all data
        let data = self.data.iter().zip(other.data.iter()).map(|(a, b)| a + b).collect::<Vec<_>>();

        Vector::new(data, "temp")
    }
}

// Overload the `-` operator for vector subtraction
impl<'a, 'b> Sub<&'b Vector> for &'a Vector {
    type Output = Vector;

    fn sub(self, other: &'b Vector) -> Vector {
        // Perform element-wise subtraction
        // iterate over all data
        let data = self.data.iter().zip(other.data.iter()).map(|(a, b)| a - b).collect::<Vec<_>>();

        Vector::new(data, "temp")
    }
}

// Dot and cross product of two vectors
impl Vector {
    pub fn dot(&self, other: &Vector, result_name: &str) -> f64 {
        // if both vectors do not have the same length, or if the length is not 3, panic
        if self.data.len() != other.data.len() {
            panic!("Vectors must have the same length and be of length 3 to perform dot product.");
        }

        // Perform element-wise multiplication and sum the results
        let result = self.data.iter().zip(other.data.iter()).map(|(a, b)| a * b).sum();

        // Log the operation in ANALYSIS mode
        if let Mode::ANALYSIS = get_mode() {
            println!("val {} = {}.x({})", result_name, self.name, other.name);
        }

        result
    }

    // Cross product of two vectors
    pub fn cross(&self, other: &Vector, result_name: &str) -> Vector {
        // if both vectors do not have the same length, or if the length is not 3, panic
        if self.data.len() != other.data.len() || self.data.len() != 3 {
            panic!("Vectors must have the same length and be of length 3 to perform cross product.");
        }

        // Perform cross product
        let x = self.data[1] * other.data[2] - self.data[2] * other.data[1];
        let y = self.data[2] * other.data[0] - self.data[0] * other.data[2];
        let z = self.data[0] * other.data[1] - self.data[1] * other.data[0];

        // Log the operation in ANALYSIS mode
        if let Mode::ANALYSIS = get_mode() {
            println!("val {} = {}.x({})", result_name, self.name, other.name);
        }

        Vector::new(vec![x, y, z], result_name)
    }
}
