// this file will be similar to Vector and Matrix, but will only contain scalars
use crate::config::{get_mode, Mode, TEMP_COUNTER};
use std::ops::{Add, Sub, Mul};



#[derive(Debug, Clone)]
pub struct Scalar {
    pub value: f64,
    pub name: String,
    pub expression: String,
}

impl Scalar {
    // Constructor for Scalar
    pub fn new(value: f64, name: &str) -> Self {
        let scalar = Scalar {
            value,
            name: name.to_string(),
            expression: name.to_string(), // Initial expression is the name itself
        };

        // Log initialization in ANALYSIS mode
        if let Mode::ANALYSIS = get_mode() {
            println!("val {} = {}", name, value);
        }

        scalar
    }

    // Assign a new name to the scalar and log the operation
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
}

impl Add for Scalar {
    type Output = Scalar;

    fn add(self, other: Scalar) -> Scalar {
        let temp_name = Scalar::temp_name();
        let expression = format!("{} + {}", self.name, other.name);

        if let Mode::ANALYSIS = get_mode() {
            println!("val {} = {} + {}", temp_name, self.name, other.name);
        }

        Scalar {
            value: self.value + other.value,
            name: temp_name,
            expression,
        }
    }
}

impl Sub for Scalar {
    type Output = Scalar;

    fn sub(self, other: Scalar) -> Scalar {
        let temp_name = Scalar::temp_name();
        let expression = format!("{} - {}", self.name, other.name);

        if let Mode::ANALYSIS = get_mode() {
            println!("val {} = {} - {}", temp_name, self.name, other.name);
        }

        Scalar {
            value: self.value - other.value,
            name: temp_name,
            expression,
        }
    }
}

impl Mul for Scalar {
    type Output = Scalar;

    fn mul(self, other: Scalar) -> Scalar {
        let temp_name = Scalar::temp_name();
        let expression = format!("{} * {}", self.name, other.name);

        if let Mode::ANALYSIS = get_mode() {
            println!("val {} = {} * {}", temp_name, self.name, other.name);
        }

        Scalar {
            value: self.value * other.value,
            name: temp_name,
            expression,
        }
    }
}