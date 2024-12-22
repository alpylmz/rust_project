
use crate::input::set_funcs;
use crate::helper::{VarType, Input, Bounds};
use crate::ast::ASTNode;
use crate::{Vector, Matrix};



pub fn ex1() {
    let return_type = VarType::Scalar;

    // set inputs with empty vectors, matrices, and scalars
    set_funcs("rand_func", return_type, vec![
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

    let cos_qpos = ASTNode::VariableV {
        name: "cos_qpos".to_string(),
    };
    let sin_qpos = ASTNode::VariableV {
        name: "sin_qpos".to_string(),
    };

    let limi_translations = vec![
        Vector!(0.0, 0.0, 0.333).define("limi_translation_0"),
        Vector!(0.0, 0.0, 0.0).define("limi_translation_1"),
        Vector!(0.0, -0.316, 0.0).define("limi_translation_2"),
        Vector!(0.083, 0.0, 0.0).define("limi_translation_3"),
        Vector!(-0.083, 0.384, 0.0).define("limi_translation_4"),
        Vector!(0.0, 0.0, 0.0).define("limi_translation_5"),
    ];

    let mut omi_rotation: Option<ASTNode> = None;
    let mut omi_translation: Option<ASTNode> = None;

    let num_joints = 6;
    for i in 0..num_joints {
        let rotation_matrix = Matrix!(
            [cos_qpos.clone().at_vec(i), -sin_qpos.clone().at_vec(i), 0.0],
            [sin_qpos.clone().at_vec(i), cos_qpos.clone().at_vec(i), 0.0],
            [0.0, 0.0, 1.0]
        ).define(format!("rotation_matrix_{}", i).as_str());

        let limi_rotation = if i == 0 {
            Matrix!(
                [rotation_matrix.clone().at_mat(0, 0), rotation_matrix.clone().at_mat(0, 1), 0.0],
                [rotation_matrix.clone().at_mat(1, 0), rotation_matrix.clone().at_mat(1, 1), 0.0],
                [0.0, 0.0, 1.0]
            ).define(format!("limi_rotation_{}", i).as_str())
        } else if i == 1 || i == 4 {
            Matrix!(
                [rotation_matrix.clone().at_mat(0, 0), rotation_matrix.clone().at_mat(0, 1), 0.0],
                [0.0, 0.0, 1.0],
                [-rotation_matrix.clone().at_mat(1, 0), -rotation_matrix.clone().at_mat(1, 1), 0.0]
            ).define(format!("limi_rotation_{}", i).as_str())
        } else {
            Matrix!(
                [rotation_matrix.clone().at_mat(0, 0), rotation_matrix.clone().at_mat(0, 1), 0.0],
                [0.0, 0.0, -1.0],
                [rotation_matrix.clone().at_mat(1, 0), rotation_matrix.clone().at_mat(1, 1), 0.0]
            ).define(format!("limi_rotation_{}", i).as_str())
        };
        
        let limi_translation = &limi_translations[i];

        if i == 0 {
            omi_rotation = Some(rotation_matrix.clone().define(format!("oMi_rotation_{}", i).as_str()));
            omi_translation = Some(limi_translation.clone().define(format!("oMi_translation_{}", i).as_str()));
        }
        else {
            let last_omi_rotation = omi_rotation.clone().unwrap();
            let last_omi_translation = omi_translation.clone().unwrap();
            // omi_rotation = omi_rotation_past * limi_rotation
            omi_rotation = Some((last_omi_rotation.clone() * limi_rotation.clone()).define(format!("oMi_rotation_{}", i).as_str()));
            //let omi_rotation = (rotation_matrix.clone() * limi_rotation.clone()).define(format!("oMi_rotation_{}", i).as_str());
            omi_translation = Some((last_omi_translation + last_omi_rotation.clone() * limi_translation.clone()).define(format!("oMi_translation_{}", i).as_str()));
        }
    }
    
}


