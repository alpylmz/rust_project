
use core::panic;

use crate::input::set_funcs;
use crate::helper::{VarType, Input, Bounds};
use crate::ast::ASTNode;
use crate::{Vector, Matrix};


fn validate_vector(node: &ASTNode, name: &str) {
    match node {
        ASTNode::Variable { var_type, .. } if *var_type == VarType::Vector => (),
        ASTNode::Vector(_) => (),
        _ => panic!("{} should be a variable or a vector", name),
    }
}

// I strongly disagree with this function's name, but it is actInv in pinocchio,
// so I will leave it as is for now
fn act_inv(translation: ASTNode, rotation: ASTNode, linear: ASTNode, angular: ASTNode, joint_id: usize) -> (ASTNode, ASTNode) {
    let act_inv1 = translation.cross(angular.clone()).define(format!("actInv1_{}", joint_id).as_str());
    let act_inv2 = (linear.clone() - act_inv1).define(format!("actInv2_{}", joint_id).as_str());
    let act_inv3 = rotation.transpose().define(format!("actInv3_{}", joint_id).as_str());
    let act_inv4 = act_inv3.clone() * act_inv2.define(format!("actInv4_{}", joint_id).as_str());
    let new_linear = (linear + act_inv4).define(format!("new_linear_{}", joint_id).as_str());
    let act_inv5 = (act_inv3.clone() * angular.clone()).define(format!("actInv5_{}", joint_id).as_str());
    let new_angular = (angular + act_inv5).define(format!("new_angular_{}", joint_id).as_str());

    (new_linear, new_angular)
}

// calc limi_rotation from rotation matrix
fn calc_limi(rotation_matrix: ASTNode, joint_id: usize) -> ASTNode {
    if joint_id == 0 {
        Matrix!(
            [rotation_matrix.clone().at_mat(0, 0), rotation_matrix.clone().at_mat(0, 1), 0.0],
            [rotation_matrix.clone().at_mat(1, 0), rotation_matrix.clone().at_mat(1, 1), 0.0],
            [0.0, 0.0, 1.0]
        ).define(format!("limi_rotation_{}", joint_id).as_str())
    } else if joint_id == 1 || joint_id == 4 {
        Matrix!(
            [rotation_matrix.clone().at_mat(0, 0), rotation_matrix.clone().at_mat(0, 1), 0.0],
            [0.0, 0.0, 1.0],
            [-rotation_matrix.clone().at_mat(1, 0), -rotation_matrix.clone().at_mat(1, 1), 0.0]
        ).define(format!("limi_rotation_{}", joint_id).as_str())
    } else {
        Matrix!(
            [rotation_matrix.clone().at_mat(0, 0), rotation_matrix.clone().at_mat(0, 1), 0.0],
            [0.0, 0.0, -1.0],
            [rotation_matrix.clone().at_mat(1, 0), rotation_matrix.clone().at_mat(1, 1), 0.0]
        ).define(format!("limi_rotation_{}", joint_id).as_str())
    }
}

// I will have two functions here, one is to be called from main function for a rnea application, 
// and the other will work as a helper function for a bigger application


fn first_pass(
    qsin: ASTNode, 
    qcos: ASTNode, 
    a_gf: &ASTNode, 
    data_v: &ASTNode, 
    q: &ASTNode, 
    v: &ASTNode, 
    a: &ASTNode,
    parent_v: &ASTNode,
    limi_translations: &Vec<ASTNode>,
    mut limi_rotations: Vec<ASTNode>,
    joint_id: usize,
) -> (Vec<ASTNode>, ASTNode) {

    let rotation_matrix = Matrix!(
        [qcos.clone(), -qsin.clone(), 0.0],
        [qsin.clone(), qcos.clone(), 0.0],
        [0.0, 0.0, 1.0]
    ).define(format!("rotation_matrix_{}", joint_id).as_str());

    let limi_rotation = calc_limi(rotation_matrix.clone(), joint_id);

    limi_rotations.push(limi_rotation.clone()); // intentional mutation, we will need it later
    let limi_translation = limi_translations[joint_id].clone();

    // in revolute joints, data.v[i] = jdata.v() line just makes data.v = v
    // I will keep it as is

    let mut new_v_linear = Vector!(
        v.clone().at_vec(0),
        v.clone().at_vec(1),
        v.clone().at_vec(2)
    ).define(format!("v_linear_{}", joint_id).as_str());

    let mut new_v_angular = Vector!(
        v.clone().at_vec(3),
        v.clone().at_vec(4),
        v.clone().at_vec(5)
    ).define(format!("v_angular_{}", joint_id).as_str());

    let parent_v_linear = Vector!(
        parent_v.clone().at_vec(0),
        parent_v.clone().at_vec(1),
        parent_v.clone().at_vec(2)
    ).define(format!("parent_v_linear_{}", joint_id).as_str());

    let parent_v_angular = Vector!(
        parent_v.clone().at_vec(3),
        parent_v.clone().at_vec(4),
        parent_v.clone().at_vec(5)
    ).define(format!("parent_v_angular_{}", joint_id).as_str());

    // if parent > 0
    if joint_id > 1 {
        (new_v_linear, new_v_angular) = act_inv(limi_translation, limi_rotation, parent_v_linear, parent_v_angular, joint_id);
    }

    let new_v = Vector!(
        new_v_linear.clone().at_vec(0),
        new_v_linear.clone().at_vec(1),
        new_v_linear.clone().at_vec(2),
        new_v_angular.clone().at_vec(0),
        new_v_angular.clone().at_vec(1),
        new_v_angular.clone().at_vec(2)
    ).define(format!("new_v_{}", joint_id).as_str());


    (limi_rotations, new_v)
}


/// Both model.nq and model.nv are 6 for panda, the hand joints are excluded
/// \param[in] q The joint configuration vector (dim model.nq).
/// \param[in] v The joint velocity vector (dim model.nv).
/// \param[in] a The joint acceleration vector (dim model.nv).
/// jointPlacements are model.jointPlacements in pinocchio, it is a vector of SE3 objects
/// SE3 has a rotation and a translation element
pub fn rnea(qsin: ASTNode, qcos: ASTNode, q: ASTNode, v: ASTNode, a: ASTNode) {
    // check if q, v, and a are ASTNode::Variable, or ASTNode::Vector
    // if they are not, return an error
    // if it is a variable, check if the variable is a vector
    // if it is not, return an error
    validate_vector(&q, "q");
    validate_vector(&qsin, "qsin");
    validate_vector(&qcos, "qcos");
    validate_vector(&v, "v");
    validate_vector(&a, "a");

    // I will hardcode some part of the logic here, the parts about panda's model placement
    // I need to change it to actual jointPlacements and other structure in the future, by making them inputs
    let limi_translations = vec![
        Vector!(0.0, 0.0, 0.333).define("limi_translation_0"),
        Vector!(0.0, 0.0, 0.0).define("limi_translation_1"),
        Vector!(0.0, -0.316, 0.0).define("limi_translation_2"),
        Vector!(0.083, 0.0, 0.0).define("limi_translation_3"),
        Vector!(-0.083, 0.384, 0.0).define("limi_translation_4"),
        Vector!(0.0, 0.0, 0.0).define("limi_translation_5"),
    ];

    let mut limi_rotations: Vec<ASTNode> = vec![];


    // we also have a_gf, which is the vector of joint accelerations due to the gravity field
    // it is constant, so I will keep it constant here, too
    let a_gf = vec!(
        Vector!(0.0, 0.0, -9.81, 0.0, 0.0, 0.0).define("a_gf_0"),
        Vector!(0.0, 0.0, -9.81, 0.0, 0.0, 0.0).define("a_gf_1"),
        Vector!(0.0, 0.0, -9.81, 0.0, 0.0, 0.0).define("a_gf_2"),
        Vector!(0.0, 0.0, -9.81, 0.0, 0.0, 0.0).define("a_gf_3"),
        Vector!(0.0, 0.0, -9.81, 0.0, 0.0, 0.0).define("a_gf_4"),
        Vector!(0.0, 0.0, -9.81, 0.0, 0.0, 0.0).define("a_gf_5")
    );
    // we also have data.v, which v[0] is set to zero explicitly, and I assume the rest should also be zero
    // I will keep it constant here, too, but keep in mind 
    let data_v = vec!(
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("data_v_0"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("data_v_1"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("data_v_2"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("data_v_3"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("data_v_4"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("data_v_5")
    );

    let mut parent_v = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("parent_v");

    // first pass, it takes model.joints[i], data.joints[i], model, data, q, v, a
    for i in 0..6 {
        (limi_rotations, parent_v) = first_pass(
            qsin.clone().at_vec(i), // qsin and qcos will not change, therefore no reference needed
            qcos.clone().at_vec(i),
            &a_gf[i],
            &data_v[i],
            &q,
            &v,
            &a,
            &parent_v, // parent_v can be empty for the first joint, it will not affect execution
            &limi_translations,
            limi_rotations,
            i
        );
    }



}

