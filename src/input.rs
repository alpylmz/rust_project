// The functions in the file should be called first, even if there is no input,
// for the functions here will also define the function and include the libraries
// This file will be no longer needed when we move away from daisy
//use crate::config::{get_mode, Mode, TEMP_COUNTER};
//use std::ops::{Add, Sub, Mul};
use crate::Vector;
use crate::Matrix;

// add the struct from helper.rs
use crate::helper::{Input, VarType, Bounds};




// set input func
// the only input will be a set of vectors, matrices, and scalars
// in the future I should merge these three with a trait, but now it can wait
pub fn set_funcs(
    name: &str,
    return_type: VarType, // this is terrible, but I will fix it later, I should use an enum when I merge all variables
    inputs: Vec<Input>,
) {
    // firstly print the initialization
    println!("import daisy.lang._\nimport Real._\nimport daisy.lang.Vector._\n\nobject RustOutput {{\n");

    // print the function name
    print!("\tdef {}(", name);

    // print the inputs
    for input in inputs.iter() {
        match input {
            Input::Scalar(name, _) => {
                print!("\n\t\t{}: Real,", name);
            }
            Input::Vector(name, _) => {
                print!("\n\t\t{}: Vector,", name);
            }
            Input::Matrix(name, _) => {
                print!("\n\t\t{}: Matrix,", name);
            }
        }
    }

    // now, delete one character from the end
    print!("\x08");
    

    // the rest is \n\t\t): return_type = {")
    match return_type {
        VarType::Scalar => {
            println!("): Real = {{");
        }
        VarType::Vector => {
            println!("): Vector = {{");
        }
        VarType::Matrix => {
            println!("): Matrix = {{");
        }
    }

    // then, we will start to write the bounds

    print!("\t\trequire(");

    // now, we will print the bounds
    for input in inputs.iter() {
        match input {
            Input::Scalar(name, bounds) => {
                println!("\n\t\t\t{} >= {} && {} <= {} && {}.size(1) &&", name, bounds.lower_bound, name, bounds.upper_bound, name);
                println!("\t\t\t{}.specV(Set(", name);
                println!("\t\t\t\t((0, 0), ({}, {})),", bounds.lower_bound, bounds.upper_bound);
                print!("\t\t\t)) &&");
            }
            Input::Vector(name, bounds) => {
                let size = bounds.len();
                let min_bound: f64 = bounds
                    .iter()
                    .map(|b| b.lower_bound)
                    .fold(f64::INFINITY, |a, b| a.min(b)); // Use `min` from `f64` directly
                let max_bound: f64 = bounds
                    .iter()
                    .map(|b| b.upper_bound)
                    .fold(f64::NEG_INFINITY, |a, b| a.max(b)); // Use `max` from `f64` directly

                println!("\n\t\t\t{} >= {} && {} <= {} && {}.size({}) &&", name, min_bound, name, max_bound, name, size);
                
                // print the bounds
                println!("\t\t\t{}.specV(Set(", name);
                for i in 0..size {
                    println!("\t\t\t\t(({}, {}), ({}, {})),", i, i, bounds[i].lower_bound, bounds[i].upper_bound);
                }
                print!("\t\t\t)) &&");
            }
            Input::Matrix(name, bounds) => {
                let size_col = bounds.len();
                let size_row = bounds[0].len();
                let min_bound: f64 = bounds
                    .iter()
                    .map(|b| b.iter().map(|b| b.lower_bound).fold(f64::INFINITY, |a, b| a.min(b)))
                    .fold(f64::INFINITY, |a, b| a.min(b)); // Use `min` from `f64` directly
                let max_bound: f64 = bounds
                    .iter()
                    .map(|b| b.iter().map(|b| b.upper_bound).fold(f64::NEG_INFINITY, |a, b| a.max(b)))
                    .fold(f64::NEG_INFINITY, |a, b| a.max(b)); // Use `max` from `f64` directly

                println!("\n\t\t\t{} >= {} && {} <= {} && {}.size({}, {}) &&", name, min_bound, name, max_bound, name, size_col, size_row);

                println!("\t\t\t{}.specV(Set(", name);
                
                // print the bounds
                for i in 0..size_col {
                    for j in 0..size_row {
                        println!("\t\t\t\t(({}, {}), ({}, {})),", i, j, bounds[i][j].lower_bound, bounds[i][j].upper_bound);
                    }
                }

                print!("\t\t\t)) &&");
            }
        }
    }

    // delete the last &&
    print!("\x08\x08");
    print!(")");



} 


