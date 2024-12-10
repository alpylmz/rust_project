// The functions in the file should be called first, even if there is no input,
// for the functions here will also define the function and include the libraries
// This file will be no longer needed when we move away from daisy
use crate::helper::{Input, VarType, Bounds};

pub fn set_funcs(
    name: &str,
    return_type: VarType,
    inputs: Vec<Input>,
) {
    // Print imports and object name
    println!("import daisy.lang._");
    println!("import Real._");
    println!("import daisy.lang.Vector._");
    println!();
    println!("object RustOutput {{\n");

    // Start building the function definition line
    let mut func_sig = format!("\tdef {}(", name);

    // Collect parameter strings
    let mut params = Vec::new();
    for input in &inputs {
        let param_str = match input {
            Input::Scalar(n, _) => format!("\n\t\t{}: Real", n),
            Input::Vector(n, _) => format!("\n\t\t{}: Vector", n),
            Input::Matrix(n, _) => format!("\n\t\t{}: Matrix", n),
        };
        params.push(param_str);
    }

    // Join parameters with commas, and append to the function signature
    if !params.is_empty() {
        func_sig.push_str(&params.join(","));
    }
    // Close the parameter list
    func_sig.push(')');

    // Add the return type
    let return_str = match return_type {
        VarType::Scalar => "Real",
        VarType::Vector => "Vector",
        VarType::Matrix => "Matrix",
    };
    func_sig.push_str(&format!(": {} = {{", return_str));
    println!("{}", func_sig);

    // Now we print the require(...) line with all constraints
    // We'll build the inside of the require(...) call
    let mut require_lines = Vec::new();

    for input in &inputs {
        match input {
            Input::Scalar(n, b) => {
                // Scalar bounds
                // condition: n >= lower && n <= upper && n.size(1)
                // specV: ((0,0), (lower,upper))
                require_lines.push(format!("{} >= {} && {} <= {} && {}.size(1) &&",
                    n, b.lower_bound, n, b.upper_bound, n));

                require_lines.push(format!("{}.specV(Set(\n\t\t\t\t((0, 0), ({}, {}))\n\t\t\t)) &&",
                    n, b.lower_bound, b.upper_bound));
            }
            Input::Vector(n, bounds) => {
                // Vector bounds
                let size = bounds.len();
                let min_bound = bounds.iter().map(|b| b.lower_bound).fold(f64::INFINITY, f64::min);
                let max_bound = bounds.iter().map(|b| b.upper_bound).fold(f64::NEG_INFINITY, f64::max);

                require_lines.push(format!("{} >= {} && {} <= {} && {}.size({}) &&",
                    n, min_bound, n, max_bound, n, size));

                // specV entries
                require_lines.push(format!("{}.specV(Set(", n));
                for i in 0..size {
                    let b = &bounds[i];
                    require_lines.push(format!("\t\t\t\t(({}, {}), ({}, {})),", i, i, b.lower_bound, b.upper_bound));
                }
                // Remove the trailing comma from last specV entry
                if let Some(last) = require_lines.last_mut() {
                    if last.ends_with(",") {
                        last.pop();
                    }
                }
                require_lines.push(")) &&".to_string());
            }
            Input::Matrix(n, bounds) => {
                // Matrix bounds
                let size_col = bounds.len();
                let size_row = if size_col > 0 { bounds[0].len() } else { 0 };

                let min_bound = bounds.iter().flat_map(|row| row.iter().map(|b| b.lower_bound))
                    .fold(f64::INFINITY, f64::min);
                let max_bound = bounds.iter().flat_map(|row| row.iter().map(|b| b.upper_bound))
                    .fold(f64::NEG_INFINITY, f64::max);

                require_lines.push(format!("{} >= {} && {} <= {} && {}.size({}, {}) &&",
                    n, min_bound, n, max_bound, n, size_col, size_row));

                require_lines.push(format!("{}.specV(Set(", n));
                for i in 0..size_col {
                    for j in 0..size_row {
                        let b = &bounds[i][j];
                        require_lines.push(format!("\t\t\t\t(({}, {}), ({}, {})),", i, j, b.lower_bound, b.upper_bound));
                    }
                }
                // Remove the trailing comma from the last entry
                if let Some(last) = require_lines.last_mut() {
                    if last.ends_with(",") {
                        last.pop();
                    }
                }
                require_lines.push(")) &&".to_string());
            }
        }
    }

    // Remove the last '&&' from require lines if present
    if let Some(last) = require_lines.last_mut() {
        if last.ends_with("&&") {
            let trimmed = last.trim_end_matches('&').trim();
            *last = trimmed.to_string();
        }
    }

    // Print require(...) block
    print!("\t\trequire(");
    if !require_lines.is_empty() {
        println!();
        for line in &require_lines {
            println!("\t\t\t{}", line);
        }
        print!("\t\t)");
    } else {
        // If no constraints, just close require immediately
        print!(")");
    }

    println!("\n\n");
}
