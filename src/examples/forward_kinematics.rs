use crate::ast::ASTNode;
use crate::{Scalar,Vector, Matrix};


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

// I strongly disagree with this function's name, but it is actInv in pinocchio,
// so I will leave it as is for now
fn act_motion_inv(
    translation: &ASTNode, 
    rotation: &ASTNode, 
    current: &ASTNode,
    parent: &ASTNode, 
    joint_id: usize) -> ASTNode {
    let linear_parent = Vector!(parent.at_vec(0), parent.at_vec(1), parent.at_vec(2)).define(format!("linear_parent_{}", joint_id).as_str());
    let angular_parent = Vector!(parent.at_vec(3), parent.at_vec(4), parent.at_vec(5)).define(format!("angular_parent_{}", joint_id).as_str());
    let linear = Vector!(current.at_vec(0), current.at_vec(1), current.at_vec(2)).define(format!("linear_{}", joint_id).as_str());
    let angular = Vector!(current.at_vec(3), current.at_vec(4), current.at_vec(5)).define(format!("angular_{}", joint_id).as_str());
    let act_inv1 = translation.cross(&angular_parent).define(format!("actInv1_{}", joint_id).as_str());
    let act_inv2 = (linear_parent - act_inv1).define(format!("actInv2_{}", joint_id).as_str());
    let act_inv3 = rotation.transpose().define(format!("actInv3_{}", joint_id).as_str());
    let act_inv4 = (act_inv3.cross(&act_inv2)).define(format!("actInv4_{}", joint_id).as_str());
    let new_linear = (linear + act_inv4).define(format!("act_inv_linear_{}", joint_id).as_str());
    let act_inv5 = (act_inv3.cross(&angular_parent)).define(format!("actInv5_{}", joint_id).as_str());
    let new_angular = (angular + act_inv5).define(format!("act_inv_angular_{}", joint_id).as_str());
    let act_inv_res = Vector!(
        new_linear.at_vec(0), new_linear.at_vec(1), new_linear.at_vec(2),
        new_angular.at_vec(0), new_angular.at_vec(1), new_angular.at_vec(2)
    ).define(format!("act_inv_res_{}", joint_id).as_str());

    act_inv_res
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

fn alpha_cross(s: &ASTNode, vin: &ASTNode, joint_id: usize) -> ASTNode {
    let vin_linear = Vector!(vin.clone().at_vec(0), vin.clone().at_vec(1), vin.clone().at_vec(2)).define(format!("vin_linear_{}", joint_id).as_str());
    let vin_angular = Vector!(vin.clone().at_vec(3), vin.clone().at_vec(4), vin.clone().at_vec(5)).define(format!("vin_angular_{}", joint_id).as_str());
    let alpha_cross_linear = alpha_cross_linear(s.clone(), vin_linear.clone(), joint_id);
    let alpha_cross_angular = alpha_cross_angular(s.clone(), vin_angular.clone(), joint_id);

    let alpha_cross_res = Vector!(
        alpha_cross_linear.at_vec(0), 
        alpha_cross_linear.at_vec(1), 
        alpha_cross_linear.at_vec(2), 
        alpha_cross_angular.at_vec(0), 
        alpha_cross_angular.at_vec(1), 
        alpha_cross_angular.at_vec(2)
    ).define(format!("alpha_cross_{}", joint_id).as_str());

    alpha_cross_res
}

// I strongly disagree with this function's name, but it is actInv in pinocchio,
// so I will leave it as is for now
fn act_inv(
    translation: &ASTNode, 
    rotation: &ASTNode,  
    parent: &ASTNode, 
    joint_id: usize
) -> ASTNode {
    let linear_parent = Vector!(parent.at_vec(0), parent.at_vec(1), parent.at_vec(2)).define(format!("linear_parent_{}", joint_id).as_str());
    let angular_parent = Vector!(parent.at_vec(3), parent.at_vec(4), parent.at_vec(5)).define(format!("angular_parent_{}", joint_id).as_str());
    let act_inv1 = translation.clone().cross(&angular_parent).define(format!("actInv1_{}", joint_id).as_str());
    let act_inv2 = (linear_parent.clone() - act_inv1).define(format!("actInv2_{}", joint_id).as_str());
    let act_inv3 = rotation.clone().transpose().define(format!("actInv3_{}", joint_id).as_str());
    let act_inv4 = (act_inv3.clone().cross(&act_inv2)).define(format!("actInv4_{}", joint_id).as_str());
    let act_inv5 = (act_inv3.clone().cross(&angular_parent)).define(format!("actInv5_{}", joint_id).as_str());

    let res = Vector!(
        act_inv4.at_vec(0), 
        act_inv4.at_vec(1), 
        act_inv4.at_vec(2), 
        act_inv5.at_vec(0), 
        act_inv5.at_vec(1), 
        act_inv5.at_vec(2)
    ).define(format!("act_inv_res_{}", joint_id).as_str());

    res
}
 


fn forward_kinematics_helper(
    // these three is the q, v, and a of each joint in joint space
    qsin: &ASTNode,
    qcos: &ASTNode,
    all_joint_v: &ASTNode, 
    all_joint_a: &ASTNode,

    // these three contain speed and acceleration of each joint
    all_v: &mut Vec<ASTNode>,
    all_a: &mut Vec<ASTNode>,

    limi_translations: &Vec<ASTNode>,
    limi_rotations: &mut Vec<ASTNode>,

    oMi_translations: &mut Vec<ASTNode>,
    oMi_rotations: &mut Vec<ASTNode>,

    joint_id: usize,
){

    let rotation_matrix = Matrix!(
        [qcos.clone(), -qsin.clone(), 0.0],
        [qsin.clone(), qcos.clone(), 0.0],
        [0.0, 0.0, 1.0]
    ).define(format!("rotation_matrix_{}", joint_id).as_str());

    let limi_rotation = calc_limi(rotation_matrix.clone(), joint_id);

    limi_rotations.push(limi_rotation.clone()); // intentional mutation, we will need it later
    let limi_translation = limi_translations[joint_id].clone();

    //if(parent>0)
    //{
    //  data.oMi[i] = data.oMi[parent] * data.liMi[i];
    //  data.v[i] += data.liMi[i].actInv(data.v[parent]);
    //}
    //else
    //  data.oMi[i] = data.liMi[i];
    match joint_id {
        0 => {
            oMi_rotations.push(limi_rotation.clone());
            oMi_translations.push(limi_translation.clone());
            all_v.push(
                Vector!(0.0, 0.0, 0.0, 0.0, 0.0, all_joint_v.at_vec(joint_id)).define(format!("final_v_{}", joint_id).as_str())
            )
        }
        _ => {
            // the multiplication between oMi and liMi is defined as:
            //{ return SE3Tpl(rot*m2.rotation()
            //    ,translation()+rotation()*m2.translation());}
            let omi_rotation_i = (oMi_rotations[joint_id - 1].cross(&limi_rotation)).define(format!("oMi_rotation_{}", joint_id).as_str());
            let omi_rotation_i = (oMi_rotations[joint_id - 1].cross(&limi_rotation)).define(format!("oMi_rotation_{}", joint_id).as_str());
            let omi_translation_to_add = (oMi_rotations[joint_id - 1].cross(&limi_translation)).define(format!("oMi_translation_to_add_{}", joint_id).as_str());
            let omi_translation_i = (&oMi_translations[joint_id - 1] + omi_translation_to_add).define(format!("oMi_translation_{}", joint_id).as_str());
            //oMis.push((omi_rotation_i.clone(), omi_translation_i.clone()));
            oMi_rotations.push(omi_rotation_i.clone().define(format!("final_omi_rotation_{}", joint_id).as_str()));
            oMi_translations.push(omi_translation_i.clone().define(format!("final_omi_translation_{}", joint_id).as_str()));
            let temp_v = Vector!(0.0, 0.0, 0.0, 0.0, 0.0, all_joint_v.at_vec(joint_id)).define(format!("temp_v_{}", joint_id).as_str());
            let new_v = act_motion_inv(
                &limi_translation, 
                &limi_rotation, 
                &temp_v,
                &all_v[joint_id - 1],
                joint_id
            );
            all_v.push(new_v.clone().define(format!("final_v_{}", joint_id).as_str()));
        }
    }

    // data.a[i]  = jdata.S() * jmodel.jointVelocitySelector(a) + jdata.c() + (data.v[i] ^ jdata.v()) ;
    let minus_m_w = (-all_joint_v.at_vec(joint_id)).define(format!("minus_m_w_{}", joint_id).as_str());
    let temp_a1 = alpha_cross(&minus_m_w, &all_v[joint_id], joint_id);

    // jdata.S() * jmodel.jointVelocitySelector(a);
    let temp_a2 = (all_joint_a.at_vec(joint_id) + temp_a1.at_vec(5)).define(format!("temp_a2_{}", joint_id).as_str());

    let temp_a3 = Vector!(
        temp_a1.at_vec(0), 
        temp_a1.at_vec(1), 
        temp_a1.at_vec(2), 
        temp_a1.at_vec(3), 
        temp_a1.at_vec(4), 
        temp_a2
    ).define(format!("temp_a3_{}", joint_id).as_str());


    // data.a[i] += data.liMi[i].actInv(data.a[parent]);
    match joint_id {
        0 => {
            all_a.push(temp_a3.clone().define(format!("final_a_{}", joint_id).as_str()));
        }
        _ => {
            let add_a = act_inv(
                &limi_translation, 
                &limi_rotation, 
                &all_a[joint_id - 1],
                joint_id
            );
            let new_a = (temp_a3 + add_a).define(format!("final_a_{}", joint_id).as_str());
            all_a.push(new_a.clone());
        }
    }




}






// Forward kinematics takes q, v, and a
pub fn forward_kinematics(
    qsin: &ASTNode, 
    qcos: &ASTNode, 
    v: &ASTNode, 
    a: &ASTNode,
){

    // skipped
    //jmodel.calc(jdata.derived(),q.derived(),v.derived());

    // skipped
    // data.v[i] = jdata.v();

    // data.liMi[i] = model.jointPlacements[i] * jdata.M();
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

    let n_joints = 6;

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
    let mut all_a: Vec<ASTNode> = vec![];
    let mut limi_rotations: Vec<ASTNode> = vec![];
    let mut oMi_translations: Vec<ASTNode> = vec![];
    let mut oMi_rotations: Vec<ASTNode> = vec![];

    
    for i in 0..n_joints {
        forward_kinematics_helper(
            &qsin.at_vec(i),
            &qcos.at_vec(i), 
            v, 
            a, 
            &mut all_v, 
            &mut all_a, 
            &limi_translations,
            &mut limi_rotations,
            &mut oMi_translations,
            &mut oMi_rotations,
            i
        );
    }


}










