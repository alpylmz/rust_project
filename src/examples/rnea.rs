
use core::panic;

//use crate::helper::{VarType, Input, Bounds};
use crate::ast::ASTNode;
use crate::{Vector, Matrix};


fn validate_vector(node: &ASTNode, name: &str) {
    match node {
        ASTNode::VariableV { name: _ } => (),
        ASTNode::Vector(_) => (),
        _ => panic!("{} should be a variable or a vector", name),
    }
}

// I strongly disagree with this function's name, but it is actInv in pinocchio,
// so I will leave it as is for now
fn act_inv(translation: &ASTNode, rotation: &ASTNode, linear: &ASTNode, angular: &ASTNode, linear_parent: ASTNode, angular_parent: ASTNode, joint_id: usize) -> (ASTNode, ASTNode) {
    let act_inv1 = translation.clone().cross(angular_parent.clone()).define(format!("actInv1_{}", joint_id).as_str());
    let act_inv2 = (linear_parent.clone() - act_inv1).define(format!("actInv2_{}", joint_id).as_str());
    let act_inv3 = rotation.clone().transpose().define(format!("actInv3_{}", joint_id).as_str());
    let act_inv4 = (act_inv3.clone().cross(act_inv2)).define(format!("actInv4_{}", joint_id).as_str());
    let new_linear = (linear.clone() + act_inv4).define(format!("act_inv_linear_{}", joint_id).as_str());
    let act_inv5 = (act_inv3.clone().cross(angular_parent.clone())).define(format!("actInv5_{}", joint_id).as_str());
    let new_angular = (angular.clone() + act_inv5).define(format!("act_inv_angular_{}", joint_id).as_str());

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
fn alpha_cross_linear(s: ASTNode, vin: ASTNode, joint_id: usize) -> ASTNode {
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
fn alpha_cross_angular(s: ASTNode, vin: ASTNode, joint_id: usize) -> ASTNode {
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
    let vout_0_0 = (inertia.clone().at_mat(0, 0) * vin.clone().at_vec(0)).define("");
    let vout_0_1 = (inertia.clone().at_mat(0, 1) * vin.clone().at_vec(1)).define("");
    let vout_0_2 = (inertia.clone().at_mat(0, 2) * vin.clone().at_vec(2)).define("");

    let vout_1_0 = (inertia.clone().at_mat(0, 1) * vin.clone().at_vec(0)).define("");
    let vout_1_1 = (inertia.clone().at_mat(1, 1) * vin.clone().at_vec(1)).define("");
    let vout_1_2 = (inertia.clone().at_mat(1, 2) * vin.clone().at_vec(2)).define("");

    let vout_2_0 = (inertia.clone().at_mat(0, 2) * vin.clone().at_vec(0)).define("");
    let vout_2_1 = (inertia.clone().at_mat(1, 2) * vin.clone().at_vec(1)).define("");
    let vout_2_2 = (inertia.clone().at_mat(2, 2) * vin.clone().at_vec(2)).define("");

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


// in C++, pinocchio/include/pinocchio/spatial/force-dense.hpp:
// f.linear().noalias() = m.rotation()*linear();
// f.angular().noalias() = m.rotation()*angular();
// f.angular() += m.translation().cross(f.linear());
fn act(
    rotation: ASTNode, 
    translation: ASTNode, 
    f: ASTNode, 
    joint_id: usize
) -> ASTNode {

    let f_linear = Vector!(
        f.clone().at_vec(0),
        f.clone().at_vec(1),
        f.clone().at_vec(2)
    ).define(format!("f_linear_{}", joint_id).as_str());
    let f_angular = Vector!(
        f.clone().at_vec(3),
        f.clone().at_vec(4),
        f.clone().at_vec(5)
    ).define(format!("f_angular_{}", joint_id).as_str());

    let new_f_linear = (rotation.clone().cross(f_linear.clone())).define(format!("new_f_linear_{}", joint_id).as_str());
    let new_f_angular = (rotation.clone().cross(f_angular.clone())).define(format!("new_f_angular_temp_{}", joint_id).as_str());

    let f_angular_cross = translation.clone().cross(new_f_linear.clone()).define(format!("f_angular_cross_{}", joint_id).as_str());

    let new_f_angular = (new_f_angular.clone() + f_angular_cross.clone()).define(format!("new_f_angular_{}", joint_id).as_str());

    let new_f = Vector!(
        new_f_linear.clone().at_vec(0),
        new_f_linear.clone().at_vec(1),
        new_f_linear.clone().at_vec(2),
        new_f_angular.clone().at_vec(0),
        new_f_angular.clone().at_vec(1),
        new_f_angular.clone().at_vec(2)
    ).define(format!("new_f_{}", joint_id).as_str());

    new_f
}




// I will have two functions here, one is to be called from main function for a rnea application, 
// and the other will work as a helper function for a bigger application
fn first_pass(
    qsin: ASTNode, 
    qcos: ASTNode, 
    data_v: &ASTNode, 
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
        data_v.clone().at_vec(0),
        data_v.clone().at_vec(1),
        data_v.clone().at_vec(2)
    ).define(format!("v_linear_{}", joint_id).as_str());

    let mut new_v_angular = Vector!(
        data_v.clone().at_vec(3),
        data_v.clone().at_vec(4),
        v.clone().at_vec(joint_id)
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
    if joint_id > 0 {
        //data.v[i] += data.liMi[i].actInv(data.v[parent]);
        (new_v_linear, new_v_angular) = act_inv(&limi_translation, &limi_rotation, &new_v_linear, &new_v_angular, parent_v_linear, parent_v_angular, joint_id);
    }


    //data.a_gf[i] = jdata.c() + (data.v[i] ^ jdata.v());
    // ^ operator is actually implemented in pinocchio/include/pinocchio/spatial/cartesian-axis.hpp inline void CartesianAxis<2>::alphaCross
    // vout_[0] = -s*vin[1]; vout_[1] = s*vin[0]; vout_[2] = 0.;
    let minus_m_w = (-(v.clone().at_vec(joint_id))).define(format!("minus_m_w_{}", joint_id).as_str());
    let alpha_cross_linear = alpha_cross_linear(minus_m_w.clone(), new_v_linear.clone(), joint_id);
    let alpha_cross_angular = alpha_cross_angular(minus_m_w.clone(), new_v_angular.clone(), joint_id);
    
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

    let new_a_gf_up1 = (a.clone().at_vec(joint_id) + new_a_gf.clone().at_vec(5)).define(format!("new_a_gf_up1_{}", joint_id).as_str());

    let new_a_gf2_linear = Vector!(
        new_a_gf.clone().at_vec(0),
        new_a_gf.clone().at_vec(1),
        new_a_gf.clone().at_vec(2)
    ).define(format!("new_a_gf2_linear_{}", joint_id).as_str());

    let new_a_gf2_angular = Vector!(
        new_a_gf.clone().at_vec(3),
        new_a_gf.clone().at_vec(4),
        new_a_gf_up1
    ).define(format!("new_a_gf2_angular_{}", joint_id).as_str());

    // data.a_gf[i] += data.liMi[i].actInv(data.a_gf[parent]);
    let (new_a_gf_up2_linear, new_a_gf_up2_angular) = act_inv(&limi_translation, &limi_rotation, &new_a_gf2_linear, &new_a_gf2_angular, parent_a_gf_linear, parent_a_gf_angular, joint_id);

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

    //////////////////
    // next line is Symmetric3::rhsMult(inertia(),a_gf.angular(),f.angular());
    let f_angular = rhs_mult(inertias[joint_id].clone(), new_a_gf_up2_angular.clone(), joint_id).define(format!("f_angular_first_{}", joint_id).as_str());

    // next line is f.angular() += lever().cross(f.linear());
    let f_angular_1 = levers[joint_id].clone().cross(f_linear_3.clone()).define(format!("f_angular_1_{}", joint_id).as_str());
    let f_angular_2 = (f_angular.clone() + f_angular_1).define(format!("f_angular_2_{}", joint_id).as_str());
    //////////////////////////

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



    let new_v = Vector!(
        new_v_linear.clone().at_vec(0),
        new_v_linear.clone().at_vec(1),
        new_v_linear.clone().at_vec(2),
        new_v_angular.clone().at_vec(0),
        new_v_angular.clone().at_vec(1),
        new_v_angular.clone().at_vec(2)
    ).define(format!("new_v_{}", joint_id).as_str());





    (limi_rotations, new_v, new_a_gf_up3, h, f)
}



fn sec_pass(
    mut all_f: Vec<ASTNode>,
    limi_rotations: Vec<ASTNode>,
    limi_translations: &Vec<ASTNode>,
    n_joints: usize
) -> (Vec<ASTNode>, ASTNode) {
    // jmodel.jointVelocitySelector(data.tau) = jdata.S().transpose()*data.f[i];
    // I again couldn't print out info about jdata.S(), 
    // but at least for panda it works like this:
    // before data.tau:        0
    //     0
    //     0
    //     0
    // 0.381501
    // 0.471101
    // data.f[i]: 
    //      linear = -39.5124  42.6572  22.4058
    //      angular = 4.46655 1.28701  5.7779
    // 
    // after data.tau:        0
    //     0
    //     0
    // 5.7779
    // 0.381501
    // 0.471101
    // in each iteration, we set data.tau[joint_id] = data.f[i].angular[2];
    // the behaviour of S() ConstraintTpl was similar in forward pass

    let mut data_taus: Vec<ASTNode> = vec![];

    for i in (0..n_joints).rev() {
        data_taus.push(all_f[i].clone().at_vec(5).define(format!("data_tau_temp_{}", i).as_str()));

        //if(parent>0) data.f[parent] += data.liMi[i].act(data.f[i]);
        if i > 0 {
            let new_data_f_parent_add = act(limi_rotations[i].clone(), limi_translations[i].clone(), all_f[i].clone(), i);
            let new_data_f_parent = (all_f[i-1].clone() + new_data_f_parent_add).define(format!("new_data_f_parent_{}", i).as_str());
            all_f[i-1] = new_data_f_parent;
        }
    }

    let data_tau = Vector!(
        data_taus[5].clone(),
        data_taus[4].clone(),
        data_taus[3].clone(),
        data_taus[2].clone(),
        data_taus[1].clone(),
        data_taus[0].clone()
    ).define("data_tau");

    (all_f, data_tau)
}

/// Both model.nq and model.nv are 6 for panda, the hand joints are excluded
/// \param[in] q The joint configuration vector (dim model.nq).
/// \param[in] v The joint velocity vector (dim model.nv).
/// \param[in] a The joint acceleration vector (dim model.nv).
/// jointPlacements are model.jointPlacements in pinocchio, it is a vector of SE3 objects
/// SE3 has a rotation and a translation element
pub fn rnea(qsin: ASTNode, qcos: ASTNode, v: ASTNode, a: ASTNode) -> ASTNode {
    // check if q, v, and a are ASTNode::Variable, or ASTNode::Vector
    // if they are not, return an error
    // if it is a variable, check if the variable is a vector
    // if it is not, return an error
    validate_vector(&qsin, "qsin");
    validate_vector(&qcos, "qcos");
    validate_vector(&v, "v");
    validate_vector(&a, "a");

    let n_joints = 6;

    // I will hardcode some part of the logic here, the parts about panda's model placement
    // I need to change it to actual jointPlacements and other structure in the future, by making them inputs
    let limi_translations = vec![
        Vector!(0.0, 0.0, 0.333).define("limi_translation_0"),
        Vector!(0.0, 0.0, 0.0).define("limi_translation_1"),
        Vector!(0.0, -0.316, 0.0).define("limi_translation_2"),
        Vector!(0.0825, 0.0, 0.0).define("limi_translation_3"),
        Vector!(-0.0825, 0.384, 0.0).define("limi_translation_4"),
        Vector!(0.0, 0.0, 0.0).define("limi_translation_5"),
    ];

    let mut limi_rotations: Vec<ASTNode> = vec![];


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
        Vector!(-0.003141, -0.02872, 0.003495).define("lever_1"),
        Vector!(0.027518, 0.039252, -0.066502).define("lever_2"),
        Vector!(-0.05317, 0.104419, 0.027454).define("lever_3"),
        Vector!(-0.011953, 0.041065, -0.038437).define("lever_4"),
        Vector!(0.060149, -0.014117, -0.010517).define("lever_5")
    ];

    let masses = Vector!(4.970684, 0.646926, 3.228604, 3.587895, 1.225946, 1.66656).define("masses");

    let inertias = vec![
        Matrix!(
            [0.70337, -0.000139, 0.006772],
            [-0.000139, 0.70661, 0.019169],
            [0.006772, 0.019169, 0.009117]
        ).define("inertia_0"),
        Matrix!(
            [0.007962, -0.003925, 0.010254],
            [-0.003925, 0.02811, 0.000704],
            [0.010254, 0.000704, 0.025995]
        ).define("inertia_1"),
        Matrix!(
            [0.037242, -0.004761, -0.011396],
            [-0.004761, 0.036155, -0.012805],
            [-0.011396, -0.012805, 0.01083]
        ).define("inertia_2"),
        Matrix!(
            [0.025853, 0.007796, -0.001332],
            [0.007796, 0.019552, 0.008641],
            [-0.001332, 0.008641, 0.028323]
        ).define("inertia_3"),
        Matrix!(
            [0.035549, -0.002117, -0.004037],
            [-0.002117, 0.029474, 0.000229],
            [-0.004037, 0.000229, 0.008627]
        ).define("inertia_4"),
        Matrix!(
            [0.001964, 0.000109, -0.001158],
            [0.000109, 0.004354, 0.000341],
            [-0.001158, 0.000341, 0.005433]
        ).define("inertia_5")
    ];

    let mut all_v: Vec<ASTNode> = vec![];
    let mut all_a_gf: Vec<ASTNode> = vec![];
    let mut all_h: Vec<ASTNode> = vec![];
    let mut all_f: Vec<ASTNode> = vec![];

    let parent_v = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("parent_v");
    let parent_a_gf = Vector!(0.0, 0.0, 9.81, 0.0, 0.0, 0.0).define("parent_a_gf");

    // Check how to define these variables clearly
    let mut new_v = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("new_v");
    let mut new_a_gf = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("new_a_gf");
    let mut new_h = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("new_h");
    let mut new_f = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("new_f");

    // first pass, it takes model.joints[i], data.joints[i], model, data, q, v, a
    for i in 0..n_joints {
        if i == 0 {
            (limi_rotations, new_v, new_a_gf, new_h, new_f) = first_pass(
                qsin.clone().at_vec(i), // qsin and qcos will not change, therefore no reference needed
                qcos.clone().at_vec(i),
                &data_v[i],
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
        }
        else{
            (limi_rotations, new_v, new_a_gf, new_h, new_f) = first_pass(
                qsin.clone().at_vec(i), // qsin and qcos will not change, therefore no reference needed
                qcos.clone().at_vec(i),
                &data_v[i],
                &v,
                &a,
                &all_v[i - 1].clone(),
                &all_a_gf[i - 1].clone(),
                &limi_translations,
                limi_rotations,
                i,
                &levers,
                &masses,
                &inertias
            );
        }

        all_v.push(new_v.clone());
        all_a_gf.push(new_a_gf.clone());
        all_h.push(new_h.clone());
        all_f.push(new_f.clone());

    }

    // sec_pass will do its own iteration
    let (new_f, taus) = sec_pass(all_f, limi_rotations, &limi_translations, n_joints);

    taus

}

