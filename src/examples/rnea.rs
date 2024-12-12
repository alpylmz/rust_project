
use core::panic;

use crate::helper::{VarType, Input, Bounds};
use crate::ast::ASTNode;
use crate::{Vector, Matrix, Scalar};


fn validate_vector(node: &ASTNode, name: &str) {
    match node {
        ASTNode::Variable { var_type, .. } if *var_type == VarType::Vector => (),
        ASTNode::Vector(_) => (),
        _ => panic!("{} should be a variable or a vector", name),
    }
}

// I strongly disagree with this function's name, but it is actInv in pinocchio,
// so I will leave it as is for now
fn act_inv(translation: &ASTNode, rotation: &ASTNode, linear: ASTNode, angular: ASTNode, joint_id: usize) -> (ASTNode, ASTNode) {
    let act_inv1 = translation.clone().cross(angular.clone()).define(format!("actInv1_{}", joint_id).as_str());
    let act_inv2 = (linear.clone() - act_inv1).define(format!("actInv2_{}", joint_id).as_str());
    let act_inv3 = rotation.clone().transpose().define(format!("actInv3_{}", joint_id).as_str());
    let act_inv4 = act_inv3.clone() * act_inv2.define(format!("actInv4_{}", joint_id).as_str());
    let new_linear = (linear + act_inv4).define(format!("new_linear_{}", joint_id).as_str());
    let act_inv5 = (act_inv3.clone() * angular.clone()).define(format!("actInv5_{}", joint_id).as_str());
    let new_angular = (angular + act_inv5).define(format!("new_angular_{}", joint_id).as_str());

    (new_linear, new_angular)
}
fn act_inv2(translation: &ASTNode, rotation: &ASTNode, linear: ASTNode, angular: ASTNode, joint_id: usize) -> (ASTNode, ASTNode) {
    let act_inv1 = translation.clone().cross(angular.clone()).define(format!("actInv1_2_{}", joint_id).as_str());
    let act_inv2 = (linear.clone() - act_inv1).define(format!("actInv2_2_{}", joint_id).as_str());
    let act_inv3 = rotation.clone().transpose().define(format!("actInv3_2_{}", joint_id).as_str());
    let act_inv4 = act_inv3.clone() * act_inv2.define(format!("actInv4_2_{}", joint_id).as_str());
    let new_linear = (linear + act_inv4).define(format!("new_linear_up2_2_{}", joint_id).as_str());
    let act_inv5 = (act_inv3.clone() * angular.clone()).define(format!("actInv5_2_{}", joint_id).as_str());
    let new_angular = (angular + act_inv5).define(format!("new_angular_up2_2_{}", joint_id).as_str());

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


// Linear and angular are the same, there should be a better way to write it
//vout_[0] = -s*vin[1]; vout_[1] = s*vin[0]; vout_[2] = 0.;
fn alpha_cross_linear(vin: ASTNode, s: ASTNode, joint_id: usize) -> ASTNode {
    let alpha_cross1 = (-(s.clone()) * vin.clone().at_vec(1)).define(format!("alpha_cross1_linear_{}", joint_id).as_str());
    let alpha_cross2 = (s * vin.clone().at_vec(0)).define(format!("alpha_cross2_linear_{}", joint_id).as_str());
    let alpha_cross = Vector!(
        alpha_cross1,
        alpha_cross2,
        0.0
    ).define(format!("alpha_cross_linear_{}", joint_id).as_str());

    alpha_cross
}

//vout_[0] = -s*vin[1]; vout_[1] = s*vin[0]; vout_[2] = 0.;
fn alpha_cross_angular(vin: ASTNode, s: ASTNode, joint_id: usize) -> ASTNode {
    let alpha_cross1 = (-(s.clone()) * vin.clone().at_vec(1)).define(format!("alpha_cross1_angular_{}", joint_id).as_str());
    let alpha_cross2 = (s * vin.clone().at_vec(0)).define(format!("alpha_cross2_angular_{}", joint_id).as_str());
    let alpha_cross = Vector!(
        alpha_cross1,
        alpha_cross2,
        0.0
    ).define(format!("alpha_cross_angular_{}", joint_id).as_str());

    alpha_cross
}


// in the function, S3 is inertia, vin is v.angular, vout is f.angular
// vout_[0] = S3.m_data(0) * vin[0] + S3.m_data(1) * vin[1] + S3.m_data(3) * vin[2];
// vout_[1] = S3.m_data(1) * vin[0] + S3.m_data(2) * vin[1] + S3.m_data(4) * vin[2];
// vout_[2] = S3.m_data(3) * vin[0] + S3.m_data(4) * vin[1] + S3.m_data(5) * vin[2];
fn rhs_mult(inertia: ASTNode, vin: ASTNode, joint_id: usize) -> ASTNode {
    let vout_0_0 = inertia.clone().at_mat(0, 0) * vin.clone().at_vec(0);
    let vout_0_1 = inertia.clone().at_mat(0, 1) * vin.clone().at_vec(1);
    let vout_0_2 = inertia.clone().at_mat(1, 1) * vin.clone().at_vec(2);

    let vout_1_0 = inertia.clone().at_mat(0, 1) * vin.clone().at_vec(0);
    let vout_1_1 = inertia.clone().at_mat(0, 2) * vin.clone().at_vec(1);
    let vout_1_2 = inertia.clone().at_mat(1, 2) * vin.clone().at_vec(2);

    let vout_2_0 = inertia.clone().at_mat(1, 1) * vin.clone().at_vec(0);
    let vout_2_1 = inertia.clone().at_mat(1, 2) * vin.clone().at_vec(1);
    let vout_2_2 = inertia.clone().at_mat(2, 2) * vin.clone().at_vec(2);

    let rhs_mult1_temp = (vout_0_0 + vout_0_1).define(format!("rhsMult1_temp_{}", joint_id).as_str());
    let rhs_mult1 = (rhs_mult1_temp + vout_0_2).define(format!("rhsMult1_{}", joint_id).as_str());

    let rhs_mult2_temp = (vout_1_0 + vout_1_1).define(format!("rhsMult2_temp_{}", joint_id).as_str());
    let rhs_mult2 = (rhs_mult2_temp + vout_1_2).define(format!("rhsMult2_{}", joint_id).as_str());

    let rhs_mult3_temp = (vout_2_0 + vout_2_1).define(format!("rhsMult3_temp_{}", joint_id).as_str());
    let rhs_mult3 = (rhs_mult3_temp + vout_2_2).define(format!("rhsMult3_{}", joint_id).as_str());

    Vector!(
        rhs_mult1,
        rhs_mult2,
        rhs_mult3
    ).define(format!("rhsMult_{}", joint_id).as_str())
}
fn rhs_mult2(inertia: ASTNode, vin: ASTNode, joint_id: usize) -> ASTNode {
    let vout_0_0 = inertia.clone().at_mat(0, 0) * vin.clone().at_vec(0);
    let vout_0_1 = inertia.clone().at_mat(0, 1) * vin.clone().at_vec(1);
    let vout_0_2 = inertia.clone().at_mat(1, 1) * vin.clone().at_vec(2);

    let vout_1_0 = inertia.clone().at_mat(0, 1) * vin.clone().at_vec(0);
    let vout_1_1 = inertia.clone().at_mat(0, 2) * vin.clone().at_vec(1);
    let vout_1_2 = inertia.clone().at_mat(1, 2) * vin.clone().at_vec(2);

    let vout_2_0 = inertia.clone().at_mat(1, 1) * vin.clone().at_vec(0);
    let vout_2_1 = inertia.clone().at_mat(1, 2) * vin.clone().at_vec(1);
    let vout_2_2 = inertia.clone().at_mat(2, 2) * vin.clone().at_vec(2);

    let rhs_mult1_temp = (vout_0_0 + vout_0_1).define(format!("rhsMult1_2_temp_{}", joint_id).as_str());
    let rhs_mult1 = (rhs_mult1_temp + vout_0_2).define(format!("rhsMult1_2_{}", joint_id).as_str());

    let rhs_mult2_temp = (vout_1_0 + vout_1_1).define(format!("rhsMult2_2_temp_{}", joint_id).as_str());
    let rhs_mult2 = (rhs_mult2_temp + vout_1_2).define(format!("rhsMult2_2_{}", joint_id).as_str());

    let rhs_mult3_temp = (vout_2_0 + vout_2_1).define(format!("rhsMult3_2_temp_{}", joint_id).as_str());
    let rhs_mult3 = (rhs_mult3_temp + vout_2_2).define(format!("rhsMult3_2_{}", joint_id).as_str());

    Vector!(
        rhs_mult1,
        rhs_mult2,
        rhs_mult3
    ).define(format!("rhsMult_2_{}", joint_id).as_str())
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
    parent_a_gf: &ASTNode,
    limi_translations: &Vec<ASTNode>,
    mut limi_rotations: Vec<ASTNode>,
    joint_id: usize,
    levers: &Vec<ASTNode>, // lever is from urdf: <origin rpy="0 0 0" xyz="0.003875 0.002081 -0.04762"/> xyz is lever
    masses: &ASTNode, // mass is from urdf: 
    inertias: &Vec<ASTNode>, // inertia is from urdf: <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
) -> (Vec<ASTNode>, ASTNode, ASTNode, ASTNode, ASTNode) {

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

    let parent_a_gf_linear = Vector!(
        parent_a_gf.clone().at_vec(0),
        parent_a_gf.clone().at_vec(1),
        parent_a_gf.clone().at_vec(2)
    ).define(format!("parent_a_gf_linear_{}", joint_id).as_str());

    let parent_a_gf_angular = Vector!(
        parent_a_gf.clone().at_vec(3),
        parent_a_gf.clone().at_vec(4),
        parent_a_gf.clone().at_vec(5)
    ).define(format!("parent_a_gf_angular_{}", joint_id).as_str());

    // if parent > 0
    if joint_id > 1 {
        //data.v[i] += data.liMi[i].actInv(data.v[parent]);
        (new_v_linear, new_v_angular) = act_inv(&limi_translation, &limi_rotation, parent_v_linear, parent_v_angular, joint_id);
    }

    let new_v = Vector!(
        new_v_linear.clone().at_vec(0),
        new_v_linear.clone().at_vec(1),
        new_v_linear.clone().at_vec(2),
        new_v_angular.clone().at_vec(0),
        new_v_angular.clone().at_vec(1),
        new_v_angular.clone().at_vec(2)
    ).define(format!("new_v_{}", joint_id).as_str());


    //data.a_gf[i] = jdata.c() + (data.v[i] ^ jdata.v());
    // ^ operator is actually implemented in pinocchio/include/pinocchio/spatial/cartesian-axis.hpp inline void CartesianAxis<2>::alphaCross
    // vout_[0] = -s*vin[1]; vout_[1] = s*vin[0]; vout_[2] = 0.;
    let alpha_cross_linear = alpha_cross_linear(new_v_linear.clone(), v.clone().at_vec(joint_id), joint_id);
    let alpha_cross_angular = alpha_cross_angular(new_v_angular.clone(), v.clone().at_vec(joint_id), joint_id);
    
    let new_a_gf = Vector!(
        alpha_cross_linear.clone().at_vec(0),
        alpha_cross_linear.clone().at_vec(1),
        alpha_cross_linear.clone().at_vec(2),
        alpha_cross_angular.clone().at_vec(0),
        alpha_cross_angular.clone().at_vec(1),
        alpha_cross_angular.clone().at_vec(2)
    ).define(format!("new_a_gf_{}", joint_id).as_str());

    // data.a_gf[i] += jdata.S() * jmodel.jointVelocitySelector(a);
    // jointVelocitySelector(a) is only a[joint_id]
    // I couldn't print out info about jdata.S() easily but it is ConstraintRevoluteTpl, and I believe the only thing this line does is
    // data.a_gf[i][5] = jmodel.jointVelocitySelector(a)

    let new_a_gf_up1 = a.clone().at_vec(joint_id).define(format!("new_a_gf_up1_{}", joint_id).as_str());

    let new_a_gf2 = Vector!(
        new_a_gf.clone().at_vec(0),
        new_a_gf.clone().at_vec(1),
        new_a_gf.clone().at_vec(2),
        new_a_gf.clone().at_vec(3),
        new_a_gf.clone().at_vec(4),
        new_a_gf_up1
    ).define(format!("new_a_gf2_{}", joint_id).as_str());


    // data.a_gf[i] += data.liMi[i].actInv(data.a_gf[parent]);
    let (new_a_gf_up2_linear, new_a_gf_up2_angular) = act_inv2(&limi_translation, &limi_rotation, parent_a_gf_linear, parent_a_gf_angular, joint_id);

    let new_a_gf_up3 = Vector!(
        new_a_gf_up2_linear.clone().at_vec(0),
        new_a_gf_up2_linear.clone().at_vec(1),
        new_a_gf_up2_linear.clone().at_vec(2),
        new_a_gf_up2_angular.clone().at_vec(0),
        new_a_gf_up2_angular.clone().at_vec(1),
        new_a_gf_up2_angular.clone().at_vec(2)
    ).define(format!("new_a_gf_up3_{}", joint_id).as_str());

    // this line updates spatial momenta
    // model.inertias[i].__mult__(data.v[i],data.h[i]);
    //let data_h = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define(format!("data_h_{}", joint_id).as_str());
    // data.v[i] is new_v at this point
    // firstly mass * (v.linear - lever.cross(v.angular))
    let h_linear_1 = levers[joint_id].clone().cross(new_v_angular.clone()).define(format!("h_linear_1_{}", joint_id).as_str());
    let h_linear_2 = (new_v_linear.clone() - h_linear_1).define(format!("h_linear_2_{}", joint_id).as_str());
    let h_linear = (masses.clone().at_vec(joint_id) * h_linear_2).define(format!("h_linear_{}", joint_id).as_str());

    // next line is Symmetric3::rhsMult(inertia(),v.angular(),f.angular());
    let h_angular = rhs_mult(inertias[joint_id].clone(), new_v_angular.clone(), joint_id).define(format!("h_angular_first_{}", joint_id).as_str());

    // next line is f.angular() += lever().cross(f.linear());
    let h_angular_1 = levers[joint_id].clone().cross(h_linear.clone()).define(format!("h_angular_1_{}", joint_id).as_str());
    let h_angular_2 = (h_angular.clone() + h_angular_1).define(format!("h_angular_{}", joint_id).as_str());


    // next line is model.inertias[i].__mult__(data.a_gf[i],data.f[i]);
    // firstly mass * (a_gf.linear - lever.cross(a_gf.angular))
    let f_linear_1 = levers[joint_id].clone().cross(new_a_gf_up2_angular.clone()).define(format!("f_linear_1_{}", joint_id).as_str());
    let f_linear_2 = (new_a_gf_up2_linear.clone() - f_linear_1).define(format!("f_linear_2_{}", joint_id).as_str());
    let f_linear_3 = (masses.clone().at_vec(joint_id) * f_linear_2).define(format!("f_linear_3_{}", joint_id).as_str());

    // next line is Symmetric3::rhsMult(inertia(),a_gf.angular(),f.angular());
    let f_angular = rhs_mult2(inertias[joint_id].clone(), new_a_gf_up2_angular.clone(), joint_id).define(format!("f_angular_first_{}", joint_id).as_str());

    // next line is f.angular() += lever().cross(f.linear());
    let f_angular_1 = levers[joint_id].clone().cross(f_linear_3.clone()).define(format!("f_angular_1_{}", joint_id).as_str());
    let f_angular_2 = (f_angular.clone() + f_angular_1).define(format!("f_angular_2_{}", joint_id).as_str());

    // the cross here is not the regular cross product since the vectors are 6D
    // it is implemented in pinocchio/include/pinocchio/spatial/motion-dense.hpp cross_impl, 
    // and it calls a motionAction, which is implemented in pinocchio/include/pinocchio/spatial/force-dense.hpp motionAction
    // final line is data.f[i] += data.v[i].cross(data.h[i]);
    /*
    void motionAction(const MotionDense<M1> & v, ForceDense<M2> & fout) const
    {
      std::cout << "ForceDense::motionAction" << std::endl;
      fout.linear().noalias() = v.angular().cross(linear());
      fout.angular().noalias() = v.angular().cross(angular())+v.linear().cross(linear());
    }
    */
    let f_linear_4_temp = (new_v_angular.clone().cross(h_linear.clone())).define(format!("f_linear_4_temp_{}", joint_id).as_str());
    let f_linear_4 = (f_linear_3.clone() + f_linear_4_temp).define(format!("f_linear_4_{}", joint_id).as_str());

    let f_angular_3_temp = (new_v_angular.clone().cross(h_angular.clone())).define(format!("f_angular_3_temp_{}", joint_id).as_str());
    let f_angular_3 = (f_angular_2.clone() + f_angular_3_temp).define(format!("f_angular_3_{}", joint_id).as_str());
    let f_angular_4_temp = (new_v_linear.clone().cross(h_linear.clone())).define(format!("f_angular_4_temp_{}", joint_id).as_str());
    let f_angular_4 = (f_angular_3.clone() + f_angular_4_temp).define(format!("f_angular_4_{}", joint_id).as_str());

    let h = Vector!(
        h_linear.clone().at_vec(0),
        h_linear.clone().at_vec(1),
        h_linear.clone().at_vec(2),
        h_angular_2.clone().at_vec(0),
        h_angular_2.clone().at_vec(1),
        h_angular_2.clone().at_vec(2)
    ).define(format!("h_rnea_firstpass{}", joint_id).as_str());

    let f = Vector!(
        f_linear_4.clone().at_vec(0),
        f_linear_4.clone().at_vec(1),
        f_linear_4.clone().at_vec(2),
        f_angular_4.clone().at_vec(0),
        f_angular_4.clone().at_vec(1),
        f_angular_4.clone().at_vec(2)
    ).define(format!("f_rnea_firstpass{}", joint_id).as_str());









    (limi_rotations, new_v, new_a_gf_up3, h, f)
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

    let levers = vec![
        Vector!(0.003875, 0.002081, -0.04762).define("lever_0"),
        Vector!(0.0, 0.0, 0.0).define("lever_1"),
        Vector!(0.0, 0.0, 0.0).define("lever_2"),
        Vector!(0.0, 0.0, 0.0).define("lever_3"),
        Vector!(0.0, 0.0, 0.0).define("lever_4"),
        Vector!(0.0, 0.0, 0.0).define("lever_5")
    ];

    let masses = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("masses");

    let inertias = vec![
        Matrix!(
            [0.0001, 0.0, 0.0],
            [0.0, 0.0001, 0.0],
            [0.0, 0.0, 0.0001]
        ).define("inertia_0"),
        Matrix!(
            [0.0001, 0.0, 0.0],
            [0.0, 0.0001, 0.0],
            [0.0, 0.0, 0.0001]
        ).define("inertia_1"),
        Matrix!(
            [0.0001, 0.0, 0.0],
            [0.0, 0.0001, 0.0],
            [0.0, 0.0, 0.0001]
        ).define("inertia_2"),
        Matrix!(
            [0.0001, 0.0, 0.0],
            [0.0, 0.0001, 0.0],
            [0.0, 0.0, 0.0001]
        ).define("inertia_3"),
        Matrix!(
            [0.0001, 0.0, 0.0],
            [0.0, 0.0001, 0.0],
            [0.0, 0.0, 0.0001]
        ).define("inertia_4"),
        Matrix!(
            [0.0001, 0.0, 0.0],
            [0.0, 0.0001, 0.0],
            [0.0, 0.0, 0.0001]
        ).define("inertia_5")
    ];

    let mut all_v: Vec<ASTNode> = vec![];
    let mut all_a_gf: Vec<ASTNode> = vec![];
    let mut all_h: Vec<ASTNode> = vec![];
    let mut all_f: Vec<ASTNode> = vec![];

    let mut parent_v = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("parent_v");
    let mut parent_a_gf = Vector!(0.0, 0.0, -9.81, 0.0, 0.0, 0.0).define("parent_a_gf");
    let mut new_v = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("new_v");

    let mut new_a_gf = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("new_a_gf");

    let mut new_h = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("new_h");

    let mut new_f = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("new_f");

    // first pass, it takes model.joints[i], data.joints[i], model, data, q, v, a
    for i in 0..6 {
        if i != 0 {
            parent_v = all_v[i - 1].clone();
            parent_a_gf = a_gf[i - 1].clone();
        }
        (limi_rotations, new_v, new_a_gf, new_h, new_f) = first_pass(
            qsin.clone().at_vec(i), // qsin and qcos will not change, therefore no reference needed
            qcos.clone().at_vec(i),
            &a_gf[i],
            &data_v[i],
            &q,
            &v,
            &a,
            &parent_v, // parent_v can be empty for the first joint, it will not affect execution
            &parent_a_gf, // parent_a_gf can be empty for the first joint, it will not affect execution
            &limi_translations,
            limi_rotations,
            i,
            &levers,
            &masses,
            &inertias
        );

        all_v.push(new_v.clone());
        all_a_gf.push(new_a_gf.clone());
        all_h.push(new_h.clone());
        all_f.push(new_f.clone());

    }



}

