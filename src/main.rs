//mod vector;
mod config;
//mod matrix;
mod input;
//mod scalar;
mod helper;
mod expr;

use input::set_funcs;
use expr::{ExprImpl, Expr};
use config::Mode;
use helper::{Bounds, Input, VarType};



/*
// implement some random function
fn rand_func(x: &Vector) -> Vector {
    let temp1 = &(x + x);
    let temp2 = &(x - temp1);
    let temp3 = temp2 + temp2;
    temp3
}

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

    // I will not use new since these are just inputs
    let cos_qpos = Expr::Vector{name: "cos_qpos".to_string(), value: vec![
        Expr::Var{name: "cos_qpos_0".to_string()},
        Expr::Var{name: "cos_qpos_1".to_string()},
        Expr::Var{name: "cos_qpos_2".to_string()},
        Expr::Var{name: "cos_qpos_3".to_string()},
        Expr::Var{name: "cos_qpos_4".to_string()},
        Expr::Var{name: "cos_qpos_5".to_string()},
    ]};

    let sin_qpos = Expr::Vector{name: "sin_qpos".to_string(), value: vec![
        Expr::Var{name: "sin_qpos_0".to_string()},
        Expr::Var{name: "sin_qpos_1".to_string()},
        Expr::Var{name: "sin_qpos_2".to_string()},
        Expr::Var{name: "sin_qpos_3".to_string()},
        Expr::Var{name: "sin_qpos_4".to_string()},
        Expr::Var{name: "sin_qpos_5".to_string()},
    ]};

    let limi_translations = vec![
        Expr::new(Expr::Vector{name: "limi_translation_0".to_string(), value: vec![
            Expr::Scalar{name: "".to_string(), value: 0.0},
            Expr::Scalar{name: "".to_string(), value: 0.0},
            Expr::Scalar{name: "".to_string(), value: 0.333},
        ]}, "limi_translation_0"),
        Expr::new(Expr::Vector{name: "limi_translation_1".to_string(), value: vec![
            Expr::Scalar{name: "".to_string(), value: 0.0},
            Expr::Scalar{name: "".to_string(), value: 0.0},
            Expr::Scalar{name: "".to_string(), value: 0.0},
        ]}, "limi_translation_1"),
        Expr::new(Expr::Vector{name: "limi_translation_2".to_string(), value: vec![
            Expr::Scalar{name: "".to_string(), value: 0.0},
            Expr::Scalar{name: "".to_string(), value: -0.316},
            Expr::Scalar{name: "".to_string(), value: 0.0},
        ]}, "limi_translation_2"),
        Expr::new(Expr::Vector{name: "limi_translation_3".to_string(), value: vec![
            Expr::Scalar{name: "".to_string(), value: 0.083},
            Expr::Scalar{name: "".to_string(), value: 0.0},
            Expr::Scalar{name: "".to_string(), value: 0.0},
        ]}, "limi_translation_3"),
        Expr::new(Expr::Vector{name: "limi_translation_4".to_string(), value: vec![
            Expr::Scalar{name: "".to_string(), value: -0.083},
            Expr::Scalar{name: "".to_string(), value: 0.384},
            Expr::Scalar{name: "".to_string(), value: 0.0},
        ]}, "limi_translation_4"),
        Expr::new(Expr::Vector{name: "limi_translation_5".to_string(), value: vec![
            Expr::Scalar{name: "".to_string(), value: 0.0},
            Expr::Scalar{name: "".to_string(), value: 0.0},
            Expr::Scalar{name: "".to_string(), value: 0.0},
        ]}, "limi_translation_5"),
    ];

    
    

    // Number of joints


    // Number of joints
    let num_joints = 6;   

    let mut omi_rotation: Option<Expr> = None;
    let mut omi_translation: Option<Expr> = None;

    // Loop over each joint
    for i in 0..num_joints {
        // Rotation matrix, some values will be coming from cos_qpos and sin_qpos
        let rotation_matrix = Expr::new(Expr::Matrix{value: vec![
            vec![cos_qpos.at(i), Expr::uminus(&cos_qpos.at(i)), Expr::Scalar{name: "".to_string(), value: 0.0}],
            vec![cos_qpos.at(i), cos_qpos.at(i), Expr::Scalar{name: "".to_string(), value: 0.0}],
            vec![Expr::Scalar{name: "".to_string(), value: 0.0}, Expr::Scalar{name: "".to_string(), value: 0.0}, Expr::Scalar{name: "".to_string(), value: 1.0}],
        ], name: format!("rotation_matrix_{}", i)}, &format!("rotation_matrix_{}", i));

        let limi_rotation = if i == 0 {
            Expr::new(Expr::Matrix{name: format!("limi_rotation_{}", i), value: vec![
            vec![
                rotation_matrix.at_m(0, 0),
                rotation_matrix.at_m(0, 1),
                Expr::Scalar{name: "".to_string(), value: 0.0},
            ]
            ,
            vec![
                rotation_matrix.at_m(1, 0),
                rotation_matrix.at_m(1, 1),
                Expr::Scalar{name: "".to_string(), value: 0.0},
            ],
            vec![
                Expr::Scalar{name: "".to_string(), value: 0.0},
                Expr::Scalar{name: "".to_string(), value: 0.0},
                Expr::Scalar{name: "".to_string(), value: 1.0},
            ],
        ]}, &format!("limi_rotation_{}", i))
        } else if i == 1 || i == 4 {
            Expr::new(Expr::Matrix{name: format!("limi_rotation_{}", i), value: vec![
            vec![
                rotation_matrix.at_m(0, 0),
                rotation_matrix.at_m(0, 1),
                Expr::Scalar{name: "".to_string(), value: 0.0},
            ],
            vec![
                Expr::Scalar{name: "".to_string(), value: 0.0},
                Expr::Scalar{name: "".to_string(), value: 0.0},
                Expr::Scalar{name: "".to_string(), value: 1.0},
            ],
            vec![
                Expr::uminus(&rotation_matrix.at_m(1, 0)),
                Expr::uminus(&rotation_matrix.at_m(1, 1)),
                Expr::Scalar{name: "".to_string(), value: 0.0},
            ],
        ]}, &format!("limi_rotation_{}", i))
        } else {
            Expr::new(Expr::Matrix{name: format!("limi_rotation_{}", i), value: vec![
            vec![
                rotation_matrix.at_m(0, 0),
                rotation_matrix.at_m(0, 1),
                Expr::Scalar{name: "".to_string(), value: 0.0},
            ],
            vec![
                Expr::Scalar{name: "".to_string(), value: 0.0},
                Expr::Scalar{name: "".to_string(), value: 0.0},
                Expr::Scalar{name: "".to_string(), value: -1.0},
            ],
            vec![
                rotation_matrix.at_m(1, 0),
                rotation_matrix.at_m(1, 1),
                Expr::Scalar{name: "".to_string(), value: 0.0},
            ],
        ]}, &format!("limi_rotation_{}", i))
        };

        let limi_translation = &limi_translations[i];

        if i == 0 {
            // First joint: Initialize
            omi_rotation = Some(limi_rotation.with_name(&format!("omi_rotation_{}", i)));
            omi_translation = Some(limi_translation.with_name(&format!("omi_translation_{}", i)));
        } else {
            // Subsequent joints: Update
            println!("doing it");
            omi_rotation = Some((&omi_rotation.unwrap() * &limi_rotation).with_name(&format!("omi_rotation_{}", i)));
            println!("done");
            omi_translation = Some(
                (&omi_translation.unwrap() + &(omi_rotation.as_ref().unwrap() * &limi_translation))
                    .with_name(&format!("omi_translation_{}", i)),
            );
        }
    }

    // print omi_rotation' and omi_translation
    println!("omi_rotation: {:?}", omi_rotation.unwrap());
}

