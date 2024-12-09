mod vector;
mod config;
mod matrix;
mod input;
mod scalar;
mod helper;

use input::set_funcs;
use vector::Vector;
use config::Mode;
use matrix::Matrix;
use scalar::Scalar;
use helper::{Bounds, Input, VarType};



// implement some random function
fn rand_func(x: &Vector) -> Vector {
    let temp1 = &(x + x);
    let temp2 = &(x - temp1);
    let temp3 = temp2 + temp2;
    temp3
}

/*
fn main() {
    // Set the mode from the configuration file
    config::set_mode(Mode::ANALYSIS);

    // Initialize Matrices
    let m1 = Matrix::new(vec![vec![2.0, 1.0], vec![1.0, 3.0]], "m1");
    let m2 = Matrix::new(vec![vec![1.0, 2.0], vec![3.0, 4.0]], "m2");
    let m3 = Matrix::new(vec![vec![0.5, 0.0], vec![0.0, 0.25]], "m3");

    // Nested Operations
    let _result = (&(&m2 + &m1) - &m3).with_name("resultm");

    // Initialize Vectors
    let v1 = Vector::new(1.0, 2.0, 3.0, "v1");
    let v2 = Vector::new(4.0, 5.0, 6.0, "v2");
    let v3 = Vector::new(7.0, 8.0, 9.0, "v3");

    // Nested Operations
    let _result = (&(&v1 + &v2) - &v3).with_name("resultv");

    // Trying function
    let _rresult = &(rand_func(&_result)).with_name("final_result");
    let _rrresult = &(rand_func(&_rresult)).with_name("final_result2");
}
*/



fn main() {
    // Set the mode to ANALYSIS
    config::set_mode(Mode::ANALYSIS);

    // get the function name in the runtime as an input
    let mut func_name = "rand_func";
    // read it
    let args = std::env::args().collect::<Vec<String>>();
    if args.len() > 1 {
        func_name = &args[1];
    }
    else {
        println!("No function name provided. Using default function name: {}", func_name);
    }

    let return_type = VarType::Scalar;

    // set inputs with empty vectors, matrices, and scalars
    set_funcs(func_name, return_type, vec![
        // Set a vector
        Input::Vector("cos_qpos", vec![
            Bounds{lower_bound: -1.0, upper_bound: 1.0},
            Bounds{lower_bound: -1.0, upper_bound: 1.0},
            Bounds{lower_bound: -1.0, upper_bound: 1.0},
            Bounds{lower_bound: -1.0, upper_bound: 1.0},
            Bounds{lower_bound: -1.0, upper_bound: 1.0},
            Bounds{lower_bound: -1.0, upper_bound: 1.0},
        ]),
        Input::Vector("sin_qpos", vec![
            Bounds{lower_bound: -1.0, upper_bound: 1.0},
            Bounds{lower_bound: -1.0, upper_bound: 1.0},
            Bounds{lower_bound: -1.0, upper_bound: 1.0},
            Bounds{lower_bound: -1.0, upper_bound: 1.0},
            Bounds{lower_bound: -1.0, upper_bound: 1.0},
            Bounds{lower_bound: -1.0, upper_bound: 1.0},
        ])
    ]);

    let limi_translations = vec![
        Vector::new(vec![0.0, 0.0, 0.333], "limi_translation_0"),
        Vector::new(vec![0.0, 0.0, 0.0], "limi_translation_1"),
        Vector::new(vec![0.0, -0.316, 0.0], "limi_translation_2"),
        Vector::new(vec![0.083, 0.0, 0.0], "limi_translation_3"),
        Vector::new(vec![-0.083, 0.384, 0.0], "limi_translation_4"),
        Vector::new(vec![0.0, 0.0, 0.0], "limi_translation_5"),
    ];

    // cos_qpos and sin_qpos will be vectors that hold variables
    

    // Number of joints
    let num_joints = 6;

    // Initialize variables to track the latest rotation and translation
    let mut omi_rotation: Option<Matrix> = None;
    let mut omi_translation: Option<Vector> = None;

    // Loop over each joint
    for i in 0..num_joints {
        let rotation_matrix = Matrix::new(
            vec![
                vec![cos_qpos(i), -sin_qpos(i), 0.0],
                vec![sin_qpos(i), cos_qpos(i), 0.0],
                vec![0.0, 0.0, 1.0],
            ],
            &format!("rotation_matrix_{}", i),
        );

        let limi_rotation = if i == 0 {
            rotation_matrix
        } else if i == 1 || i == 4 {
            Matrix::new(
                vec![
                    vec![rotation_matrix.data[0][0], rotation_matrix.data[0][1], 0.0],
                    vec![0.0, 0.0, 1.0],
                    vec![-rotation_matrix.data[1][0], -rotation_matrix.data[1][1], 0.0],
                ],
                &format!("limi_rotation_{}", i),
            )
        } else {
            Matrix::new(
                vec![
                    vec![rotation_matrix.data[0][0], rotation_matrix.data[0][1], 0.0],
                    vec![0.0, 0.0, -1.0],
                    vec![rotation_matrix.data[1][0], rotation_matrix.data[1][1], 0.0],
                ],
                &format!("limi_rotation_{}", i),
            )
        };

        let limi_translation = limi_translations[i].clone();

        if i == 0 {
            // First joint: Initialize
            omi_rotation = Some(limi_rotation.with_name(&format!("omi_rotation_{}", i)));
            omi_translation = Some(limi_translation.with_name(&format!("omi_translation_{}", i)));
        } else {
            // Subsequent joints: Update
            omi_rotation = Some((&omi_rotation.unwrap() * &limi_rotation).with_name(&format!("omi_rotation_{}", i)));
            omi_translation = Some(
                (&omi_translation.unwrap() + &(omi_rotation.as_ref().unwrap() * &limi_translation))
                    .with_name(&format!("omi_translation_{}", i)),
            );
        }
    }

    // print omi_rotation' and omi_translation
    println!("omi_rotation: {:?}", omi_rotation.unwrap());
}

