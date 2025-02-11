
use core::panic;

//use crate::helper::{VarType, Input, Bounds};
use crate::ast::ASTNode;
use crate::{Scalar,Vector, Matrix};


fn validate_vector(node: &ASTNode, name: &str) {
    match node {
        ASTNode::VariableV { name: _ } => (),
        ASTNode::Vector(_) => (),
        _ => panic!("{} should be a variable or a vector", name),
    }
}


// calc limi_rotation from rotation matrix
fn calc_limi(rotation_matrix: ASTNode, joint_id: usize) -> ASTNode {
    if joint_id == 0 {
        Matrix!(
            [rotation_matrix.at_mat(0, 0), rotation_matrix.at_mat(0, 1), 0.0],
            [rotation_matrix.at_mat(1, 0), rotation_matrix.at_mat(1, 1), 0.0],
            [0.0, 0.0, 1.0]
        ).define(format!("limi_rotation_{}", joint_id).as_str())
    } else if joint_id == 1 || joint_id == 4 {
        Matrix!(
            [rotation_matrix.at_mat(0, 0), rotation_matrix.at_mat(0, 1), 0.0],
            [0.0, 0.0, 1.0],
            [-rotation_matrix.at_mat(1, 0), -rotation_matrix.at_mat(1, 1), 0.0]
        ).define(format!("limi_rotation_{}", joint_id).as_str())
    } else {
        Matrix!(
            [rotation_matrix.at_mat(0, 0), rotation_matrix.at_mat(0, 1), 0.0],
            [0.0, 0.0, -1.0],
            [rotation_matrix.at_mat(1, 0), rotation_matrix.at_mat(1, 1), 0.0]
        ).define(format!("limi_rotation_{}", joint_id).as_str())
    }
}


// Linear and angular are the same, there should be a better way to write it
//vout_[0] = -s*vin[1]; vout_[1] = s*vin[0]; vout_[2] = 0.;
fn alpha_cross_linear(s: &ASTNode, vin: &ASTNode, joint_id: usize) -> ASTNode {
    let alpha_cross1 = (-(s.clone()) * vin.at_vec(1)).define(format!("alpha_cross1_linear_{}", joint_id).as_str());
    let alpha_cross2 = (s * vin.at_vec(0)).define(format!("alpha_cross2_linear_{}", joint_id).as_str());
    let alpha_cross = Vector!(
        alpha_cross1,
        alpha_cross2,
        0.0
    ).define(format!("alpha_cross_linear_{}", joint_id).as_str());

    alpha_cross
}

//vout_[0] = -s*vin[1]; vout_[1] = s*vin[0]; vout_[2] = 0.;
fn alpha_cross_angular(s: &ASTNode, vin: &ASTNode, joint_id: usize) -> ASTNode {
    let alpha_cross1 = (-(s.clone()) * vin.at_vec(1)).define(format!("alpha_cross1_angular_{}", joint_id).as_str());
    let alpha_cross2 = (s * vin.at_vec(0)).define(format!("alpha_cross2_angular_{}", joint_id).as_str());
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
    let vout_0_0 = (inertia.at_mat(0, 0) * vin.at_vec(0)).define("");
    let vout_0_1 = (inertia.at_mat(0, 1) * vin.at_vec(1)).define("");
    let vout_0_2 = (inertia.at_mat(0, 2) * vin.at_vec(2)).define("");

    let vout_1_0 = (inertia.at_mat(0, 1) * vin.at_vec(0)).define("");
    let vout_1_1 = (inertia.at_mat(1, 1) * vin.at_vec(1)).define("");
    let vout_1_2 = (inertia.at_mat(1, 2) * vin.at_vec(2)).define("");

    let vout_2_0 = (inertia.at_mat(0, 2) * vin.at_vec(0)).define("");
    let vout_2_1 = (inertia.at_mat(1, 2) * vin.at_vec(1)).define("");
    let vout_2_2 = (inertia.at_mat(2, 2) * vin.at_vec(2)).define("");

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


// If I am not mistaken, skew(vec3d) gives
// 
//    M_(0,0) = Scalar(0);  M_(0,1) = -v[2];      M_(0,2) = v[1];
//    M_(1,0) = v[2];       M_(1,1) = Scalar(0);  M_(1,2) = -v[0];
//    M_(2,0) = -v[1];      M_(2,1) = v[0];       M_(2,2) = Scalar(0);

fn skew_vec3d(vec: &ASTNode) -> ASTNode {
    let res = Matrix!(
        [Scalar!(0.0), -(vec.at_vec(2)), vec.at_vec(1)],
        [vec.at_vec(2), Scalar!(0.0), -(vec.at_vec(0))],
        [-(vec.at_vec(1)), vec.at_vec(0), Scalar!(0.0)]
    ).define("skew_vec3d");

    res
}

//inline void skewSquare(const Eigen::MatrixBase<V1> & u,
//                       const Eigen::MatrixBase<V2> & v,
//                       const Eigen::MatrixBase<Matrix3> & C)
//{
//  C_.noalias() = v*u.transpose();
//  const Scalar udotv(u.dot(v));
//  C_.diagonal().array() -= udotv;
//}
fn skew_square(u: &ASTNode, v: &ASTNode) -> ASTNode{
    // u and v should be vectors, so I need to either translate them into matrices
    // or I need to implement it manually
    let c1 = Matrix!(
        [v.at_vec(0) * u.at_vec(0), v.at_vec(0) * u.at_vec(1), v.at_vec(0) * u.at_vec(2)],
        [v.at_vec(1) * u.at_vec(0), v.at_vec(1) * u.at_vec(1), v.at_vec(1) * u.at_vec(2)],
        [v.at_vec(2) * u.at_vec(0), v.at_vec(2) * u.at_vec(1), v.at_vec(2) * u.at_vec(2)] 
    ).define("skew_square_c1");

    // then I need to calculate the dot product of u and v
    // I guess there is no dot product, so its manual
    let udotv = (u.at_vec(0) * v.at_vec(0) + u.at_vec(1) * v.at_vec(1) + u.at_vec(2) * v.at_vec(2)).define("skew_square_udotv");

    // then I need to subtract the diagonal of c1 by udotv
    let c2 = Matrix!(
        [c1.at_mat(0, 0) - udotv.clone(), c1.at_mat(0, 1), c1.at_mat(0, 2)],
        [c1.at_mat(1, 0), c1.at_mat(1, 1) - udotv.clone(), c1.at_mat(1, 2)],
        [c1.at_mat(2, 0), c1.at_mat(2, 1), c1.at_mat(2, 2) - udotv.clone()]
    ).define("skew_square_c2");

    c2
}

// alphaskewsquare here: 
// operator Symmetric3Tpl () const 
// {
// const Scalar & x = v[0], & y = v[1], & z = v[2];
// return Symmetric3Tpl(-m*(y*y+z*z),
//                         m* x*y,-m*(x*x+z*z),
//                         m* x*z,m* y*z,-m*(x*x+y*y));
// }
fn alpha_skew_square(m: &ASTNode, v: &ASTNode) -> ASTNode {
    let x = v.at_vec(0);
    let y = v.at_vec(1);
    let z = v.at_vec(2);

    let alpha_skew_square = Matrix!(
        [-m.clone() * (y.clone() * y.clone() + z.clone() * z.clone()), m.clone() * x.clone() * y.clone(), m.clone() * x.clone() * z.clone()],
        [m.clone() * x.clone() * y.clone(), -m.clone() * (x.clone() * x.clone() + z.clone() * z.clone()), m.clone() * y.clone() * z.clone()],
        [m.clone() * x.clone() * z.clone(), m.clone() * y.clone() * z.clone(), -m.clone() * (x.clone() * x.clone() + y.clone() * y.clone())]
    ).define("alpha_skew_square");

    alpha_skew_square
}



//Matrix6 variation(const Motion & v) const
//    {
//      Matrix6 res;
//      const Motion mv(v*mass());
//      
//      res.template block<3,3>(LINEAR,ANGULAR) = -skew(mv.linear()) - skewSquare(mv.angular(),lever()) + skewSquare(lever(),mv.angular());
//      res.template block<3,3>(ANGULAR,LINEAR) = res.template block<3,3>(LINEAR,ANGULAR).transpose();
//      
////      res.template block<3,3>(LINEAR,LINEAR) = mv.linear()*c.transpose(); // use as temporary variable
////      res.template block<3,3>(ANGULAR,ANGULAR) = res.template block<3,3>(LINEAR,LINEAR) - res.template block<3,3>(LINEAR,LINEAR).transpose();
//      res.template block<3,3>(ANGULAR,ANGULAR) = -skewSquare(mv.linear(),lever()) - skewSquare(lever(),mv.linear());
//      
//      res.template block<3,3>(LINEAR,LINEAR) = (inertia() - AlphaSkewSquare(mass(),lever())).matrix();
//      
//      res.template block<3,3>(ANGULAR,ANGULAR) -= res.template block<3,3>(LINEAR,LINEAR) * skew(v.angular());
//      res.template block<3,3>(ANGULAR,ANGULAR) += cross(v.angular(),res.template block<3,3>(LINEAR,LINEAR));
//      
//      res.template block<3,3>(LINEAR,LINEAR).setZero();
//      return res;
//    }
// rotation is inertia(), and translation is lever()
fn inertia_variation(rotation: &ASTNode, translation: &ASTNode, linear: &ASTNode, angular: &ASTNode, mass: &ASTNode, joint_id: usize) -> ASTNode {
    let mv_linear = Vector!(
        linear.at_vec(0) * mass.clone(),
        linear.at_vec(1) * mass.clone(),
        linear.at_vec(2) * mass.clone()
    ).define("mv_linear");
    let mv_angular = Vector!(
        angular.at_vec(0) * mass.clone(),
        angular.at_vec(1) * mass.clone(),
        angular.at_vec(2) * mass.clone()
    ).define("mv_angular");

    // res.template block<3,3>(LINEAR,ANGULAR) = -skew(mv.linear()) - skewSquare(mv.angular(),lever()) + skewSquare(lever(),mv.angular());
    let skew_first_first = skew_vec3d(&mv_linear);
    let skew_first_second = skew_square(&mv_angular, translation);
    let skew_first_third = skew_square(translation, &mv_angular);
    let res_first_temp = (skew_first_third - skew_first_second).define("res_first_temp");
    let res_first = (res_first_temp - skew_first_first).define("res_first");

    //      res.template block<3,3>(ANGULAR,LINEAR) = res.template block<3,3>(LINEAR,ANGULAR).transpose();
    // passing this for now

    //      res.template block<3,3>(ANGULAR,ANGULAR) = -skewSquare(mv.linear(),lever()) - skewSquare(lever(),mv.linear());
    let skew_second_first = skew_square(&mv_linear, translation);
    let skew_second_second = skew_square(translation, &mv_linear);
    let skew_second_first_neg = Matrix!(
        [-skew_second_first.at_mat(0, 0), -skew_second_first.at_mat(0, 1), -skew_second_first.at_mat(0, 2)],
        [-skew_second_first.at_mat(1, 0), -skew_second_first.at_mat(1, 1), -skew_second_first.at_mat(1, 2)],
        [-skew_second_first.at_mat(2, 0), -skew_second_first.at_mat(2, 1), -skew_second_first.at_mat(2, 2)]
    ).define("skew_second_second_neg");
    let res_second = (skew_second_first_neg - skew_second_second).define("res_second");

    //      res.template block<3,3>(LINEAR,LINEAR) = (inertia() - AlphaSkewSquare(mass(),lever())).matrix();
    let res_third = (rotation.clone() - alpha_skew_square(mass, translation)).define("res_third");

    // res.template block<3,3>(ANGULAR,ANGULAR) -= res.template block<3,3>(LINEAR,LINEAR) * skew(v.angular());
    let res_fourth = (res_third.clone().cross(&skew_vec3d(&angular))).define("res_fourth");

    let res_fifth = (res_second - res_fourth).define("res_fifth");

    // res.template block<3,3>(ANGULAR,ANGULAR) += cross(v.angular(),res.template block<3,3>(LINEAR,LINEAR));
    // cross here applies the cross product onto the columns of M.
    let res_third_first_col = Vector!(
        res_third.at_mat(0, 0),
        res_third.at_mat(1, 0),
        res_third.at_mat(2, 0)
    ).define("res_third_first_col");
    let res_third_second_col = Vector!(
        res_third.at_mat(0, 1),
        res_third.at_mat(1, 1),
        res_third.at_mat(2, 1)
    ).define("res_third_second_col");
    let res_third_third_col = Vector!(
        res_third.at_mat(0, 2),
        res_third.at_mat(1, 2),
        res_third.at_mat(2, 2)
    ).define("res_third_third_col");
    let res_sixth_first_col  = (angular.cross(&res_third_first_col)).define("res_sixth_first_col");
    let res_sixth_second_col = (angular.cross(&res_third_second_col)).define("res_sixth_second_col");
    let res_sixth_third_col  = (angular.cross(&res_third_third_col)).define("res_sixth_third_col");

    let res_sixth = Matrix!(
        [res_fifth.at_mat(0,0) + res_sixth_first_col.at_vec(0), res_fifth.at_mat(0,1) + res_sixth_second_col.at_vec(0), res_fifth.at_mat(0,2) + res_sixth_third_col.at_vec(0)],
        [res_fifth.at_mat(1,0) + res_sixth_first_col.at_vec(1), res_fifth.at_mat(1,1) + res_sixth_second_col.at_vec(1), res_fifth.at_mat(1,2) + res_sixth_third_col.at_vec(1)],
        [res_fifth.at_mat(2,0) + res_sixth_first_col.at_vec(2), res_fifth.at_mat(2,1) + res_sixth_second_col.at_vec(2), res_fifth.at_mat(2,2) + res_sixth_third_col.at_vec(2)]
    ).define("res_sixth");

    // linear parts are 0
    // the third part of the matrix is res_sixth

    // the first and second parts are the same, res_first
    
    let res_inertia_variation = Matrix!(
        [Scalar!(0.0), Scalar!(0.0), Scalar!(0.0), res_first.at_mat(0, 0), res_first.at_mat(0, 1), res_first.at_mat(0, 2)],
        [Scalar!(0.0), Scalar!(0.0), Scalar!(0.0), res_first.at_mat(1, 0), res_first.at_mat(1, 1), res_first.at_mat(1, 2)],
        [Scalar!(0.0), Scalar!(0.0), Scalar!(0.0), res_first.at_mat(2, 0), res_first.at_mat(2, 1), res_first.at_mat(2, 2)],
        [res_first.at_mat(0, 0), res_first.at_mat(1, 0), res_first.at_mat(2, 0), res_sixth.at_mat(0, 0), res_sixth.at_mat(0, 1), res_sixth.at_mat(0, 2)],
        [res_first.at_mat(0, 1), res_first.at_mat(1, 1), res_first.at_mat(2, 1), res_sixth.at_mat(1, 0), res_sixth.at_mat(1, 1), res_sixth.at_mat(1, 2)],
        [res_first.at_mat(0, 2), res_first.at_mat(1, 2), res_first.at_mat(2, 2), res_sixth.at_mat(2, 0), res_sixth.at_mat(2, 1), res_sixth.at_mat(2, 2)]
    ).define(format!("inertia_variation_{}", joint_id).as_str());

    res_inertia_variation
}


// motionSet::motionAction
//void motionAction(const MotionDense<M1> & v, MotionDense<M2> & mout) const
//{
//    std::cout << "MotionDense::motionAction 1" << std::endl;
//    mout.linear() = v.linear().cross(angular())+v.angular().cross(linear());
//    mout.angular() = v.angular().cross(angular());
//}
fn motionAction(v_linear: &ASTNode, v_angular: &ASTNode, linear: &ASTNode, angular: &ASTNode, joint_id: usize) -> (ASTNode, ASTNode) {
    let mout_linear_cross_angular = (v_linear.clone().cross(angular)).define(format!("mout_linear_cross_angular_{}", joint_id).as_str());
    let mout_angular_cross_linear = (v_angular.clone().cross(linear)).define(format!("mout_angular_cross_linear_{}", joint_id).as_str());
    let mout_linear = (mout_linear_cross_angular + mout_angular_cross_linear).define(format!("mout_linear_{}", joint_id).as_str());
    let mout_angular = (v_angular.clone().cross(&angular)).define(format!("mout_angular_{}", joint_id).as_str());

    (mout_linear, mout_angular)
}


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
fn cross_6d(v1: &ASTNode, v2: &ASTNode) -> ASTNode {
    let v1_linear = Vector!(
        v1.at_vec(0),
        v1.at_vec(1),
        v1.at_vec(2)
    ).define("v1_linear");
    let v1_angular = Vector!(
        v1.at_vec(3),
        v1.at_vec(4),
        v1.at_vec(5)
    ).define("v1_angular");

    let v2_linear = Vector!(
        v2.at_vec(0),
        v2.at_vec(1),
        v2.at_vec(2)
    ).define("v2_linear");
    let v2_angular = Vector!(
        v2.at_vec(3),
        v2.at_vec(4),
        v2.at_vec(5)
    ).define("v2_angular");

    let res_linear = (v1_angular.clone().cross(&v2_linear)).define("res_linear");
    let res_angular_1 = (v1_angular.clone().cross(&v2_angular)).define("res_angular_1");
    let res_angular_2 = (v1_linear.clone().cross(&v2_linear)).define("res_angular_2");
    let res_angular = (res_angular_1 + res_angular_2).define("res_angular");

    let res = Vector!(
        res_linear.at_vec(0),
        res_linear.at_vec(1),
        res_linear.at_vec(2),
        res_angular.at_vec(0),
        res_angular.at_vec(1),
        res_angular.at_vec(2)
    ).define("cross_6d_res");

    res
}

//  ///
//  /// \brief Add skew matrix represented by a 3d vector to a given matrix,
//  ///        i.e. add the antisymmetric matrix representation of the cross product operator (\f$ [v]_{\times} x = v \times x \f$)
//  ///
//  /// \param[in]  v a vector of dimension 3.
//  /// \param[out] M the 3x3 matrix to which the skew matrix is added.
//  ///
//  template <typename Vector3Like, typename Matrix3Like>
//  inline void addSkew(const Eigen::MatrixBase<Vector3Like> & v,
//                      const Eigen::MatrixBase<Matrix3Like> & M)
//  {
//    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Vector3Like,3);
//    PINOCCHIO_ASSERT_MATRIX_SPECIFIC_SIZE(Matrix3Like,M,3,3);
//    
//    Matrix3Like & M_ = PINOCCHIO_EIGEN_CONST_CAST(Matrix3Like,M);
//    
//                          M_(0,1) -= v[2];      M_(0,2) += v[1];
//    M_(1,0) += v[2];                            M_(1,2) -= v[0];
//    M_(2,0) -= v[1];      M_(2,1) += v[0];                     ;
//  }
fn add_skew(m: &ASTNode, v: &ASTNode) -> ASTNode {
    Matrix!(
        [m.at_mat(0, 0), m.at_mat(0, 1) - v.at_vec(2), m.at_mat(0, 2) + v.at_vec(1)],
        [m.at_mat(1, 0) + v.at_vec(2), m.at_mat(1, 1), m.at_mat(1, 2) - v.at_vec(0)],
        [m.at_mat(2, 0) - v.at_vec(1), m.at_mat(2, 1) + v.at_vec(0), m.at_mat(2, 2)]
    ).define("add_skew")
}

/// template<typename ForceDerived, typename M6>
/// static void addForceCrossMatrix(const ForceDense<ForceDerived> & f,
///                                 const Eigen::MatrixBase<M6> & mout)
/// {
///   M6 & mout_ = PINOCCHIO_EIGEN_CONST_CAST(M6,mout);
///   addSkew(-f.linear(),mout_.template block<3,3>(ForceDerived::LINEAR,ForceDerived::ANGULAR));
///   addSkew(-f.linear(),mout_.template block<3,3>(ForceDerived::ANGULAR,ForceDerived::LINEAR));
///   addSkew(-f.angular(),mout_.template block<3,3>(ForceDerived::ANGULAR,ForceDerived::ANGULAR));
/// }
fn add_force_cross_matrix(f_linear: &ASTNode, f_angular: &ASTNode, mout: &ASTNode, joint_id: usize) -> ASTNode {
    // the linear linear part will stay the same
    let linear_angular = Matrix!(
        [mout.at_mat(0, 3), mout.at_mat(0, 4), mout.at_mat(0, 5)],
        [mout.at_mat(1, 3), mout.at_mat(1, 4), mout.at_mat(1, 5)],
        [mout.at_mat(2, 3), mout.at_mat(2, 4), mout.at_mat(2, 5)]
    ).define("add_force_cross_linear_angular");
    let angular_linear = Matrix!(
        [mout.at_mat(3, 0), mout.at_mat(3, 1), mout.at_mat(3, 2)],
        [mout.at_mat(4, 0), mout.at_mat(4, 1), mout.at_mat(4, 2)],
        [mout.at_mat(5, 0), mout.at_mat(5, 1), mout.at_mat(5, 2)]
    ).define("add_force_cross_angular_linear");
    let angular_angular = Matrix!(
        [mout.at_mat(3, 3), mout.at_mat(3, 4), mout.at_mat(3, 5)],
        [mout.at_mat(4, 3), mout.at_mat(4, 4), mout.at_mat(4, 5)],
        [mout.at_mat(5, 3), mout.at_mat(5, 4), mout.at_mat(5, 5)]
    ).define("add_force_cross_angular_angular");

    let minus_f_linear = Vector!(
        -f_linear.at_vec(0),
        -f_linear.at_vec(1),
        -f_linear.at_vec(2)
    ).define("minus_f_linear");
    let minus_f_angular = Vector!(
        -f_angular.at_vec(0),
        -f_angular.at_vec(1),
        -f_angular.at_vec(2)
    ).define("minus_f_angular");
    // addSkew(-f.linear(),mout_.template block<3,3>(ForceDerived::LINEAR,ForceDerived::ANGULAR));
    let res_linear_angular = add_skew(&linear_angular, &minus_f_linear).define("res_linear_angular");
    // addSkew(-f.linear(),mout_.template block<3,3>(ForceDerived::ANGULAR,ForceDerived::LINEAR));
    let res_angular_linear = add_skew(&angular_linear, &minus_f_linear).define("res_angular_linear");
    // addSkew(-f.angular(),mout_.template block<3,3>(ForceDerived::ANGULAR,ForceDerived::ANGULAR));
    let res_angular_angular = add_skew(&angular_angular, &minus_f_angular).define("res_angular_angular");

    Matrix!(
        [mout.at_mat(0, 0), mout.at_mat(0, 1), mout.at_mat(0, 2), res_linear_angular.at_mat(0, 0), res_linear_angular.at_mat(0, 1), res_linear_angular.at_mat(0, 2)],
        [mout.at_mat(1, 0), mout.at_mat(1, 1), mout.at_mat(1, 2), res_linear_angular.at_mat(1, 0), res_linear_angular.at_mat(1, 1), res_linear_angular.at_mat(1, 2)],
        [mout.at_mat(2, 0), mout.at_mat(2, 1), mout.at_mat(2, 2), res_linear_angular.at_mat(2, 0), res_linear_angular.at_mat(2, 1), res_linear_angular.at_mat(2, 2)],
        [res_angular_linear.at_mat(0, 0), res_angular_linear.at_mat(0, 1), res_angular_linear.at_mat(0, 2), res_angular_angular.at_mat(0, 0), res_angular_angular.at_mat(0, 1), res_angular_angular.at_mat(0, 2)],
        [res_angular_linear.at_mat(1, 0), res_angular_linear.at_mat(1, 1), res_angular_linear.at_mat(1, 2), res_angular_angular.at_mat(1, 0), res_angular_angular.at_mat(1, 1), res_angular_angular.at_mat(1, 2)],
        [res_angular_linear.at_mat(2, 0), res_angular_linear.at_mat(2, 1), res_angular_linear.at_mat(2, 2), res_angular_angular.at_mat(2, 0), res_angular_angular.at_mat(2, 1), res_angular_angular.at_mat(2, 2)]
    ).define(format!("add_force_cross_matrix_{joint_id}").as_str())
    

    
}








// it uses a variable "axis", and it is always 2 for some reason.
//template<typename S1, int O1>
//typename SE3GroupAction<ConstraintRevoluteTpl>::ReturnType
//se3Action(const SE3Tpl<S1,O1> & m) const
//{
//  typedef typename SE3GroupAction<ConstraintRevoluteTpl>::ReturnType ReturnType;
//  ReturnType res;
//  res.template segment<3>(LINEAR) = m.translation().cross(m.rotation().col(axis));
//  res.template segment<3>(ANGULAR) = m.rotation().col(axis);
//  std::cout << "res: " << res << std::endl;
//  return res;
//}
fn act_constraint(rotation: &ASTNode, translation: &ASTNode, joint_id: usize) -> (ASTNode, ASTNode) {
    let rotation_col = Vector!(
        rotation.at_mat(0, 2),
        rotation.at_mat(1, 2),
        rotation.at_mat(2, 2)
    ).define(format!("act_constraint_rotation_col_{}", joint_id).as_str());

    let linear = (translation.clone().cross(&rotation_col)).define(format!("act_constraint_linear_{}", joint_id).as_str());

    (linear, rotation_col)
}

// not to be confused with motionAction. 
//      v.angular().noalias() = m.rotation()*angular();
//      v.linear().noalias() = m.rotation()*linear() + m.translation().cross(v.angular());
fn act_motion1(
    rotation: &ASTNode, 
    translation: &ASTNode, 
    linear: &ASTNode, 
    angular: &ASTNode, 
    joint_id: usize
) -> ASTNode {

    let rotation_crosss_linear = (rotation.cross(linear)).define(format!("motion_act_linear_{}", joint_id).as_str());
    let rotation_cross_angular = (rotation.cross(angular)).define(format!("motion_act_angular_{}", joint_id).as_str());

    let res_angular = rotation_cross_angular;

    let cross = translation.cross(&res_angular).define(format!("motion_act_cross_{}", joint_id).as_str());
    let res_linear = (rotation_crosss_linear + cross).define(format!("motion_act_linear2_{}", joint_id).as_str());

    Vector!(
        res_linear.at_vec(0),
        res_linear.at_vec(1),
        res_linear.at_vec(2),
        res_angular.at_vec(0),
        res_angular.at_vec(1),
        res_angular.at_vec(2)
    ).define(format!("act_motion_res_{}", joint_id).as_str())
}

// not to be confused with motionAction. 
// in C++, pinocchio/include/pinocchio/spatial/motion-dense.hpp:
//      v.angular().noalias() = m.rotation()*angular();
//      v.linear().noalias() = m.rotation()*linear() + m.translation().cross(v.angular());
fn act_motion(
    rotation: &ASTNode, 
    translation: &ASTNode, 
    t: &ASTNode, 
    joint_id: usize
) -> ASTNode {

    let linear = Vector!(
        t.at_vec(0),
        t.at_vec(1),
        t.at_vec(2)
    ).define(format!("t_linear_{}", joint_id).as_str());
    let angular = Vector!(
        t.at_vec(3),
        t.at_vec(4),
        t.at_vec(5)
    ).define(format!("t_angular_{}", joint_id).as_str());

    let rotation_crosss_linear = (rotation.cross(&linear)).define(format!("motion_act_linear_{}", joint_id).as_str());
    let rotation_cross_angular = (rotation.cross(&angular)).define(format!("motion_act_angular_{}", joint_id).as_str());

    let res_angular = rotation_cross_angular.clone();

    let cross = translation.cross(&res_angular).define(format!("motion_act_cross_{}", joint_id).as_str());
    let res_linear = (rotation_crosss_linear + cross).define(format!("motion_act_linear2_{}", joint_id).as_str());

    Vector!(
        res_linear.at_vec(0),
        res_linear.at_vec(1),
        res_linear.at_vec(2),
        res_angular.at_vec(0),
        res_angular.at_vec(1),
        res_angular.at_vec(2)
    ).define(format!("act_motion_res_{}", joint_id).as_str())
}

// I strongly disagree with this function's name, but it is actInv in pinocchio,
// so I will leave it as is for now
fn act_motion_inv(translation: &ASTNode, rotation: &ASTNode, linear: ASTNode, angular: ASTNode, linear_parent: ASTNode, angular_parent: &ASTNode, joint_id: usize) -> (ASTNode, ASTNode) {
    let act_inv1 = translation.cross(angular_parent).define(format!("actInv1_{}", joint_id).as_str());
    let act_inv2 = (linear_parent - act_inv1).define(format!("actInv2_{}", joint_id).as_str());
    let act_inv3 = rotation.transpose().define(format!("actInv3_{}", joint_id).as_str());
    let act_inv4 = (act_inv3.cross(&act_inv2)).define(format!("actInv4_{}", joint_id).as_str());
    let new_linear = (linear + act_inv4).define(format!("act_inv_linear_{}", joint_id).as_str());
    let act_inv5 = (act_inv3.cross(angular_parent)).define(format!("actInv5_{}", joint_id).as_str());
    let new_angular = (angular + act_inv5).define(format!("act_inv_angular_{}", joint_id).as_str());

    (new_linear, new_angular)
}





// res(0,0) = m_data(0); res(0,1) = m_data(1); res(0,2) = m_data(3);
// res(1,0) = m_data(1); res(1,1) = m_data(2); res(1,2) = m_data(4);
// res(2,0) = m_data(3); res(2,1) = m_data(4); res(2,2) = m_data(5);
/////Matrix32  decomposeltI() const
/////    {
/////      Matrix32 L;
/////      L << 
/////      m_data(0) - m_data(5),    m_data(1),
/////      m_data(1),              m_data(2) - m_data(5),
/////      2*m_data(3),            m_data(4) + m_data(4);
/////      return L;
/////    }
/// S:   0.70337 -0.000139  0.006772
/// -0.000139   0.70661  0.019169
/// 0.006772  0.019169  0.009117
/// L:  0.694253 -0.000139
/// -0.000139  0.697493
/// 0.013544  0.038338
fn decompose_it_i(i: &ASTNode) -> ASTNode {
    let l0 = i.at_mat(0, 0) - i.at_mat(2, 2);
    let l1 = i.at_mat(0, 1);
    let l2 = i.at_mat(0, 1);
    let l3 = i.at_mat(1, 1) - i.at_mat(2, 2);
    let l4 = Scalar!(2.0) * i.at_mat(0, 2);
    let l5 = i.at_mat(1, 2) + i.at_mat(1, 2);
    let l = Matrix!(
        [l0, l1],
        [l2, l3],
        [l4, l5]
    ).define("decompose_it_i");

    l
}

// this is implemented in
// include/pinocchio/spatial/symmetric3.hpp, Symmetric3::rotate
//Symmetric3Tpl rotate(const Eigen::MatrixBase<D> & R) const
//{
//    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(D,3,3);
//    assert(isUnitary(R.transpose()*R) && "R is not a Unitary matrix");
//
//    Symmetric3Tpl Sres;
//
//    // 4 a
//    const Matrix32 L( decomposeltI() );
//
//    // Y = R' L   ===> (12 m + 8 a)
//    const Matrix2 Y( R.template block<2,3>(1,0) * L );
//
//    // Sres= Y R  ===> (16 m + 8a)
//    Sres.m_data(1) = Y(0,0)*R(0,0) + Y(0,1)*R(0,1);
//    Sres.m_data(2) = Y(0,0)*R(1,0) + Y(0,1)*R(1,1);
//    Sres.m_data(3) = Y(1,0)*R(0,0) + Y(1,1)*R(0,1);
//    Sres.m_data(4) = Y(1,0)*R(1,0) + Y(1,1)*R(1,1);
//    Sres.m_data(5) = Y(1,0)*R(2,0) + Y(1,1)*R(2,1);
//
//    // r=R' v ( 6m + 3a)
//    const Vector3 r(-R(0,0)*m_data(4) + R(0,1)*m_data(3),
//                    -R(1,0)*m_data(4) + R(1,1)*m_data(3),
//                    -R(2,0)*m_data(4) + R(2,1)*m_data(3));
//
//    // Sres_11 (3a)
//    Sres.m_data(0) = L(0,0) + L(1,1) - Sres.m_data(2) - Sres.m_data(5);
//
//    // Sres + D + (Ev)x ( 9a)
//    Sres.m_data(0) += m_data(5);
//    Sres.m_data(1) += r(2); Sres.m_data(2)+= m_data(5);
//    Sres.m_data(3) +=-r(1); Sres.m_data(4)+= r(0); Sres.m_data(5) += m_data(5);
//
//    return Sres;
//}
fn symmetric3_rotate(
    s: &ASTNode,
    r: &ASTNode
) -> ASTNode {
    let l = decompose_it_i(s);

    // const Matrix2 Y( R.template block<2,3>(1,0) * L );
    // R.template block<2,3>(1,0) takes the bottom two rows of R
    let bottom_r = Matrix!(
        [r.at_mat(1, 0), r.at_mat(1, 1), r.at_mat(1, 2)],
        [r.at_mat(2, 0), r.at_mat(2, 1), r.at_mat(2, 2)]
    ).define("rotate_bottom_r");

    let y = (bottom_r.cross(&l)).define("rotate_y");

    let sres_first_1 = (y.at_mat(0, 0) * r.at_mat(0, 0) + y.at_mat(0, 1) * r.at_mat(0, 1)).define("sres_first_1");
    let sres_first_2 = (y.at_mat(0, 0) * r.at_mat(1, 0) + y.at_mat(0, 1) * r.at_mat(1, 1)).define("sres_first_2");
    let sres_first_3 = (y.at_mat(1, 0) * r.at_mat(0, 0) + y.at_mat(1, 1) * r.at_mat(0, 1)).define("sres_first_3");
    let sres_first_4 = (y.at_mat(1, 0) * r.at_mat(1, 0) + y.at_mat(1, 1) * r.at_mat(1, 1)).define("sres_first_4");
    let sres_first_5 = (y.at_mat(1, 0) * r.at_mat(2, 0) + y.at_mat(1, 1) * r.at_mat(2, 1)).define("sres_first_5");

    //    const Vector3 r(-R(0,0)*m_data(4) + R(0,1)*m_data(3),
    //                    -R(1,0)*m_data(4) + R(1,1)*m_data(3),
    //                    -R(2,0)*m_data(4) + R(2,1)*m_data(3));
    let const_r = Vector!(
        -(r.at_mat(0, 0) * s.at_mat(1, 2)) + r.at_mat(0, 1) * s.at_mat(0, 2),
        -(r.at_mat(1, 0) * s.at_mat(1, 2)) + r.at_mat(1, 1) * s.at_mat(0, 2),
        -(r.at_mat(2, 0) * s.at_mat(1, 2)) + r.at_mat(2, 1) * s.at_mat(0, 2)
    ).define("const_r");

//    Sres.m_data(0) = L(0,0) + L(1,1) - Sres.m_data(2) - Sres.m_data(5);
//
//    // Sres + D + (Ev)x ( 9a)
//    Sres.m_data(0) += m_data(5);
//    Sres.m_data(1) += r(2); Sres.m_data(2)+= m_data(5);
//    Sres.m_data(3) +=-r(1); Sres.m_data(4)+= r(0); Sres.m_data(5) += m_data(5);

    // let sres_update_0 = l.at_mat(0, 0) + l.at_mat(1, 1) - sres_first_2.clone() - sres_first_5.clone();
    let sres_update_0_tmp1 = (l.at_mat(0, 0) + l.at_mat(1, 1)).define("sres_update_0_tmp1");
    let sres_update_0_tmp2 = (sres_first_2.clone() + sres_first_5.clone()).define("sres_update_0_tmp2");
    let sres_update_0 = (sres_update_0_tmp1 - sres_update_0_tmp2).define("sres_update_0");


    let sres_final_0 = sres_update_0 + s.at_mat(2, 2);
    let sres_final_1 = sres_first_1.clone() + const_r.at_vec(2);
    let sres_final_2 = sres_first_2.clone() + s.at_mat(2, 2);
    let sres_final_3 = sres_first_3.clone() - const_r.at_vec(1);
    let sres_final_4 = sres_first_4.clone() + const_r.at_vec(0);
    let sres_final_5 = sres_first_5.clone() + s.at_mat(2, 2);

    // res(0,0) = m_data(0); res(0,1) = m_data(1); res(0,2) = m_data(3);
    // res(1,0) = m_data(1); res(1,1) = m_data(2); res(1,2) = m_data(4);
    // res(2,0) = m_data(3); res(2,1) = m_data(4); res(2,2) = m_data(5);
    let sres = Matrix!(
        [sres_final_0, sres_final_1.clone(), sres_final_3.clone()],
        [sres_final_1, sres_final_2, sres_final_4.clone()],
        [sres_final_3, sres_final_4, sres_final_5]
    ).define("symmetric3_rotate");

    sres
}


// in oMi.act(model.inertias[i]),
// model.inertias[i].se3Action_impl(oMi) is called
//InertiaTpl se3Action_impl(const SE3 & M) const
//    {
//      /* The multiplication RIR' has a particular form that could be used, however it
//       * does not seems to be more efficient, see http://stackoverflow.m_comom/questions/
//       * 13215467/eigen-best-way-to-evaluate-asa-transpose-and-store-the-result-in-a-symmetric .*/
//       //std::cout << "InertiaTpl::se3Action_impl" << std::endl;
//       return InertiaTpl(mass(),
//                         M.translation()+M.rotation()*lever(),
//                         inertia().rotate(M.rotation()));
//     }
fn act_inertia(
    rotation: &ASTNode, // of oMi
    translation: &ASTNode, // of oMi
    lever: &ASTNode,
    inertia: &ASTNode,    
) -> (ASTNode, ASTNode) {
    let new_translation_temp = (rotation.cross(lever)).define("new_translation_temp");
    let new_translation = (translation + new_translation_temp).define("new_translation");
    let new_rotation = symmetric3_rotate(inertia, rotation);

    (new_translation, new_rotation)
}

    // inertia * vector
    //data.oh[i] = data.oYcrb[i] * ov;
    // mult implemented as this
    //f.linear().noalias() = mass()*(v.linear() - lever().cross(v.angular()));
    //Symmetric3::rhsMult(inertia(),v.angular(),f.angular());
    //f.angular() += lever().cross(f.linear());
fn inertia_vec_mult(
    inertia_mass: &ASTNode,
    inertia_lever: &ASTNode,
    inertia_inertia: &ASTNode,
    v: &ASTNode,
    joint_id: usize
) -> ASTNode{

    let v_linear = Vector!(
        v.at_vec(0),
        v.at_vec(1),
        v.at_vec(2)
    ).define(format!("inertia_vec_mult_v_linear_{}", joint_id).as_str());
    let v_angular = Vector!(
        v.at_vec(3),
        v.at_vec(4),
        v.at_vec(5)
    ).define(format!("inertia_vec_mult_v_angular_{}", joint_id).as_str());
    
    let data_oh_linear_temp1 = inertia_lever.cross(&v_angular).define(format!("inertia_vec_mult_v_linear_temp1_{}", joint_id).as_str());
    let data_oh_linear_temp2 = (v_linear.clone() - data_oh_linear_temp1).define(format!("inertia_vec_mult_v_linear_temp1_{}", joint_id).as_str());
    let data_oh_linear = (inertia_mass * data_oh_linear_temp2).define(format!("inertia_vec_mult_v_linear{}", joint_id).as_str());
    
    let data_oh_angular_temp = rhs_mult(inertia_inertia.clone(), v_angular.clone(), joint_id);
    let data_oh_angular_temp2 = inertia_lever.cross(&data_oh_linear).define(format!("inertia_vec_mult_v_angular_temp2_{}", joint_id).as_str());
    let data_oh_angular = (data_oh_angular_temp + data_oh_angular_temp2).define(format!("inertia_vec_mult_v_angular_{}", joint_id).as_str());
    
    let res = Vector!(
        data_oh_linear.at_vec(0),
        data_oh_linear.at_vec(1),
        data_oh_linear.at_vec(2),
        data_oh_angular.at_vec(0),
        data_oh_angular.at_vec(1),
        data_oh_angular.at_vec(2)
    ).define(format!("inertia_vec_mult_res_{}", joint_id).as_str());

    res
}
    
    
// I will have two functions here, one is to be called from main function for a rnea application, 
// and the other will work as a helper function for a bigger application
fn first_pass(
    qsin: ASTNode, 
    qcos: ASTNode, 
    data_v: &ASTNode, 
    v: &ASTNode, 
    a: &ASTNode,
    all_v: &mut Vec<ASTNode>,
    oMis: &mut Vec<(ASTNode, ASTNode)>, // the first is rotation, the second is translation
    limi_translations: &Vec<ASTNode>,
    limi_rotations: &mut Vec<ASTNode>,
    joint_id: usize,
    levers: &Vec<ASTNode>, // lever is from urdf: <origin rpy="0 0 0" xyz="0.003875 0.002081 -0.04762"/> xyz is lever
    masses: &Vec<ASTNode>, // mass is from urdf: 
    inertias: &Vec<ASTNode>, // inertia is from urdf: <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    all_of: &mut Vec<ASTNode>,
    all_oh: &mut Vec<ASTNode>,
    all_doycrb: &mut Vec<ASTNode>,
    all_ov: &mut Vec<ASTNode>,
    all_oa: &mut Vec<ASTNode>,
    all_oa_gf: &mut Vec<ASTNode>,
    all_a: &mut Vec<ASTNode>,
    all_oycrb: &mut Vec<(ASTNode, ASTNode)>,
    j_cols: &mut Vec<ASTNode>,
    dj_cols: &mut Vec<ASTNode>,
    dAdq_cols: &mut Vec<ASTNode>,
    dAdv_cols: &mut Vec<ASTNode>,
    dVdq_cols: &mut Vec<ASTNode>,
) {

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
        data_v.at_vec(0),
        data_v.at_vec(1),
        data_v.at_vec(2)
    ).define(format!("v_linear_{}", joint_id).as_str());

    let mut new_v_angular = Vector!(
        data_v.at_vec(3),
        data_v.at_vec(4),
        v.at_vec(joint_id)
    ).define(format!("v_angular_{}", joint_id).as_str());

    let parent_v = match joint_id {
        0 => Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("parent_v"),
        _ => all_v[joint_id - 1].clone()
    };

    let parent_v_linear = Vector!(
        parent_v.at_vec(0),
        parent_v.at_vec(1),
        parent_v.at_vec(2)
    ).define(format!("parent_v_linear_{}", joint_id).as_str());

    let parent_v_angular = Vector!(
        parent_v.at_vec(3),
        parent_v.at_vec(4),
        parent_v.at_vec(5)
    ).define(format!("parent_v_angular_{}", joint_id).as_str());

    let parent_a = match joint_id {
        0 => Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("parent_a"),
        _ => all_a[joint_id - 1].clone()
    };

    let parent_a_linear = Vector!(
        parent_a.at_vec(0),
        parent_a.at_vec(1),
        parent_a.at_vec(2)
    ).define(format!("parent_a_linear_{}", joint_id).as_str());

    let parent_a_angular = Vector!(
        parent_a.at_vec(3),
        parent_a.at_vec(4),
        parent_a.at_vec(5)
    ).define(format!("parent_a_angular_{}", joint_id).as_str());


    //if(parent > 0)
    //  {
    //    data.oMi[i] = data.oMi[parent] * data.liMi[i];
    //    data.v[i] += data.liMi[i].actInv(data.v[parent]);
    //  }
    //  else
    //    data.oMi[i] = data.liMi[i];
    match joint_id {
        0 => oMis.push((limi_rotation.clone(), limi_translation.clone())),
        _ => {
            // the multiplication between oMi and liMi is defined as:
            //{ return SE3Tpl(rot*m2.rotation()
            //    ,translation()+rotation()*m2.translation());}
            let omi_rotation_i = (oMis[joint_id - 1].0.cross(&limi_rotation)).define(format!("oMi_rotation_{}", joint_id).as_str());
            let omi_translation_to_add = (oMis[joint_id - 1].0.cross(&limi_translation)).define(format!("oMi_translation_to_add_{}", joint_id).as_str());
            let omi_translation_i = (&oMis[joint_id - 1].1 + omi_translation_to_add).define(format!("oMi_translation_{}", joint_id).as_str());
            oMis.push((omi_rotation_i.clone(), omi_translation_i.clone()));
            (new_v_linear, new_v_angular) = act_motion_inv(
                &limi_translation, 
                &limi_rotation, 
                new_v_linear, 
                new_v_angular, 
                parent_v_linear, 
                &parent_v_angular, 
                joint_id);
        }
    }
    // as the calculation of data.v[i] is done here, we can push it to all_v
    all_v.push(Vector!(
        new_v_linear.at_vec(0),
        new_v_linear.at_vec(1),
        new_v_linear.at_vec(2),
        new_v_angular.at_vec(0),
        new_v_angular.at_vec(1),
        new_v_angular.at_vec(2)
    ).define(format!("all_v_{}", joint_id).as_str()));


    //oMis.push(oMis[joint_id - 1].clone() * limi_rotation.clone());


    // The below portion is the same with RNEA, because in RNEA derivative it is like this:
    //data.a[i] = jdata.S() * jmodel.jointVelocitySelector(a) + jdata.c() + (data.v[i] ^ jdata.v());
    //data.a_gf[i] = jdata.c() + (data.v[i] ^ jdata.v());
    // ^ operator is actually implemented in pinocchio/include/pinocchio/spatial/cartesian-axis.hpp inline void CartesianAxis<2>::alphaCross
    // vout_[0] = -s*vin[1]; vout_[1] = s*vin[0]; vout_[2] = 0.;
    let minus_m_w = (-(v.at_vec(joint_id))).define(format!("minus_m_w_{}", joint_id).as_str());
    let alpha_cross_linear = alpha_cross_linear(&minus_m_w, &new_v_linear, joint_id);
    let alpha_cross_angular = alpha_cross_angular(&minus_m_w, &new_v_angular, joint_id);
    
    // data.a_gf[i] += jdata.S() * jmodel.jointVelocitySelector(a);
    // jointVelocitySelector(a) is only a[joint_id]
    // I couldn't print out info about jdata.S() easily but it is ConstraintRevoluteTpl, and I believe the only thing this line does is
    // data.a_gf[i][5] = jmodel.jointVelocitySelector(a)
    
    let mut new_data_a = Vector!(
        alpha_cross_linear.at_vec(0),
        alpha_cross_linear.at_vec(1),
        alpha_cross_linear.at_vec(2),
        alpha_cross_angular.at_vec(0),
        alpha_cross_angular.at_vec(1),
        a.at_vec(joint_id)
    ).define(format!("new_data_a_{}", joint_id).as_str());

    let temp_a_linear = Vector!(
        new_data_a.at_vec(0),
        new_data_a.at_vec(1),
        new_data_a.at_vec(2)
    ).define(format!("temp_a_linear_{}", joint_id).as_str());
    let temp_a_angular = Vector!(
        new_data_a.at_vec(3),
        new_data_a.at_vec(4),
        new_data_a.at_vec(5)
    ).define(format!("temp_a_angular_{}", joint_id).as_str());

    //if(parent > 0)
    //  {
    //    data.a[i] += data.liMi[i].actInv(data.a[parent]);
    //  }

    match joint_id {
        0 => (),
        _ => {
            let limi_actInv_a_parent = act_motion_inv(
                &limi_translation, &limi_rotation, temp_a_linear, temp_a_angular, parent_a_linear, &parent_a_angular, joint_id);
    
            new_data_a = Vector!(
                limi_actInv_a_parent.0.at_vec(0),
                limi_actInv_a_parent.0.at_vec(1),
                limi_actInv_a_parent.0.at_vec(2),
                limi_actInv_a_parent.1.at_vec(0),
                limi_actInv_a_parent.1.at_vec(1),
                limi_actInv_a_parent.1.at_vec(2)
            ).define(format!("a_final_{}", joint_id).as_str());
             
        }
    }

    all_a.push(new_data_a.clone());

    // all of these act functions are different than rnea.rs
    // data.oYcrb[i] = data.oinertias[i] = data.oMi[i].act(model.inertias[i]);
    let (data_oycrb_trans_i, data_oycrb_rot_i) = act_inertia(&oMis[joint_id].0, &oMis[joint_id].1, &levers[joint_id], &inertias[joint_id]);
    let data_oinertias_trans_i = data_oycrb_trans_i.clone().define(format!("data_oinertias_trans_{}", joint_id).as_str());
    let data_oinertias_rot_i = data_oycrb_rot_i.clone().define(format!("data_oinertias_rot_{}", joint_id).as_str());

    all_oycrb.push((data_oycrb_rot_i.clone(), data_oycrb_trans_i.clone()));

    // ov = data.oMi[i].act(data.v[i]);
    let ov = act_motion1(&oMis[joint_id].0, &oMis[joint_id].1, &new_v_linear, &new_v_angular, joint_id).define(format!("ov_{}", joint_id).as_str());
    // oa = data.oMi[i].act(data.a[i]);
    let oa = act_motion(&oMis[joint_id].0, &oMis[joint_id].1, &new_data_a, joint_id).define(format!("oa_{}", joint_id).as_str());

    // push ov
    all_ov.push(ov.clone());
    // push oa
    all_oa.push(oa.clone());

    //oa_gf = oa - model.gravity; // add gravity contribution
    let model_gravity = Vector!(0.0, 0.0, -9.81, 0.0, 0.0, 0.0).define("model_gravity");
    let oa_gf = (oa.clone() - model_gravity.clone()).define(format!("oa_gf_{}", joint_id).as_str());

    // push oa_gf
    all_oa_gf.push(oa_gf.clone());




    // inertia * vector
    //data.oh[i] = data.oYcrb[i] * ov;
    // mult implemented as this
    //f.linear().noalias() = mass()*(v.linear() - lever().cross(v.angular()));
    //Symmetric3::rhsMult(inertia(),v.angular(),f.angular());
    //f.angular() += lever().cross(f.linear());
    let ov_linear = Vector!(
        ov.at_vec(0),
        ov.at_vec(1),
        ov.at_vec(2)
    ).define(format!("ov_linear_{}", joint_id).as_str());
    let ov_angular = Vector!(
        ov.at_vec(3),
        ov.at_vec(4),
        ov.at_vec(5)
    ).define(format!("ov_angular_{}", joint_id).as_str());

    let data_oh_linear_temp1 = data_oycrb_trans_i.cross(&ov_angular).define(format!("data_oh_linear_temp1_{}", joint_id).as_str());
    let data_oh_linear_temp2 = (ov_linear.clone() - data_oh_linear_temp1).define(format!("data_oh_linear_temp2_{}", joint_id).as_str());
    let data_oh_linear = (&masses[joint_id] * data_oh_linear_temp2).define(format!("data_oh_linear_{}", joint_id).as_str());

    let data_oh_angular_temp = rhs_mult(data_oycrb_rot_i.clone(), ov_angular.clone(), joint_id);
    let data_oh_angular_temp2 = data_oycrb_trans_i.cross(&data_oh_linear).define(format!("data_oh_angular_temp2_{}", joint_id).as_str());
    let data_oh_angular = (data_oh_angular_temp + data_oh_angular_temp2).define(format!("data_oh_angular_{}", joint_id).as_str());
    
    // push to all_oh
    all_oh.push(Vector!(
        data_oh_linear.at_vec(0),
        data_oh_linear.at_vec(1),
        data_oh_linear.at_vec(2),
        data_oh_angular.at_vec(0),
        data_oh_angular.at_vec(1),
        data_oh_angular.at_vec(2)
    ).define(format!("all_oh_{}", joint_id).as_str()));

    //data.of[i] = data.oYcrb[i] * oa_gf + ov.cross(data.oh[i]);
    // cross is ov.cross(data.oh[i]), so v is ov, linear() is oh_linear and angular() is oh_angular
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
    let oa_gf_linear = Vector!(
        oa_gf.at_vec(0),
        oa_gf.at_vec(1),
        oa_gf.at_vec(2)
    ).define(format!("oa_gf_linear_{}", joint_id).as_str());
    let oa_gf_angular = Vector!(
        oa_gf.at_vec(3),
        oa_gf.at_vec(4),
        oa_gf.at_vec(5)
    ).define(format!("oa_gf_angular_{}", joint_id).as_str());

    let data_of_linear_temp1 = data_oycrb_trans_i.cross(&oa_gf_angular).define(format!("data_of_linear_temp1_{}", joint_id).as_str());
    let data_of_linear_temp2 = (oa_gf_linear.clone() - data_of_linear_temp1.clone()).define(format!("data_of_linear_temp2_{}", joint_id).as_str());
    let data_of_linear_temp3 = (&masses[joint_id] * data_of_linear_temp2.clone()).define(format!("data_of_linear_temp3_{}", joint_id).as_str());

    let data_of_angular_temp = rhs_mult(data_oycrb_rot_i.clone(), oa_gf_angular.clone(), joint_id);
    let data_of_angular_temp2 = data_oycrb_trans_i.cross(&data_of_linear_temp3).define(format!("data_of_angular_temp2_{}", joint_id).as_str());
    let data_of_angular_temp3 = (data_of_angular_temp.clone() + data_of_angular_temp2.clone()).define(format!("data_of_angular_temp3_{}", joint_id).as_str());

    // now ov.cross(data_oh[i])
    let data_of_linear_temp4 = ov_angular.cross(&data_oh_linear.clone()).define(format!("data_of_linear_temp4_{}", joint_id).as_str());

    let data_of_angular_temp4 = ov_angular.cross(&data_oh_angular).define(format!("data_of_angular_temp4_{}", joint_id).as_str());
    let data_of_angular_temp5 = ov_linear.cross(&data_oh_linear).define(format!("data_of_angular_temp5_{}", joint_id).as_str());
    let data_of_angular_temp6 = (data_of_angular_temp4.clone() + data_of_angular_temp5.clone()).define(format!("data_of_angular_temp6_{}", joint_id).as_str());

    let data_of_linear = (data_of_linear_temp3.clone() + data_of_linear_temp4.clone()).define(format!("data_of_linear_{}", joint_id).as_str());
    let data_of_angular = (data_of_angular_temp3.clone() + data_of_angular_temp6.clone()).define(format!("data_of_angular_{}", joint_id).as_str());
    // push to all_of
    all_of.push(Vector!(
        data_of_linear.at_vec(0),
        data_of_linear.at_vec(1),
        data_of_linear.at_vec(2),
        data_of_angular.at_vec(0),
        data_of_angular.at_vec(1),
        data_of_angular.at_vec(2)
    ).define(format!("all_of_{}", joint_id).as_str()));
    // Correct until here
    
    // J_cols = data.oMi[i].act(jdata.S());
    // S is ConstraintRevoluteTpl, and this function can be found in:
    // include/pinocchio/multibody/joint/joint-revolute.hpp, ConstraintRevoluteTpl::se3Action
    let (J_cols_linear, J_cols_angular) = act_constraint(&oMis[joint_id].0, &oMis[joint_id].1, joint_id);

    // push to j_cols
    j_cols.push(Vector!(
        J_cols_linear.at_vec(0),
        J_cols_linear.at_vec(1),
        J_cols_linear.at_vec(2),
        J_cols_angular.at_vec(0),
        J_cols_angular.at_vec(1),
        J_cols_angular.at_vec(2)
    ).define(format!("j_cols_{}", joint_id).as_str()));
    
    //motionSet::motionAction(ov,J_cols,dJ_cols);
    let (dJ_cols_linear, dJ_cols_angular) = motionAction(&ov_linear, &ov_angular, &J_cols_linear, &J_cols_angular, joint_id);

    dj_cols.push(Vector!(
        dJ_cols_linear.at_vec(0),
        dJ_cols_linear.at_vec(1),
        dJ_cols_linear.at_vec(2),
        dJ_cols_angular.at_vec(0),
        dJ_cols_angular.at_vec(1),
        dJ_cols_angular.at_vec(2)
    ).define(format!("dj_cols_{}", joint_id).as_str()));

    
    // joint_id is one more than what it actually should be, so in parent for oa_gfs I should check joint_id for parent
    let oa_gf_parent_linear = match joint_id {
        0 => Vector!(0.0, 0.0, 0.0).define(format!("oa_gf_parent_linear_{}", joint_id).as_str()),
        _ => Vector!(
            all_oa_gf[joint_id - 1].at_vec(0),
            all_oa_gf[joint_id - 1].at_vec(1),
            all_oa_gf[joint_id - 1].at_vec(2)
        ).define(format!("oa_gf_parent_linear_{}", joint_id).as_str())
    };
    let oa_gf_parent_angular = match joint_id {
        0 => Vector!(0.0, 0.0, 9.81).define(format!("oa_gf_parent_angular_{}", joint_id).as_str()), // first oa_gf is gravity, it is hardcoded in pinocchio
        _ => Vector!(
            all_oa_gf[joint_id - 1].at_vec(3),
            all_oa_gf[joint_id - 1].at_vec(4),
            all_oa_gf[joint_id - 1].at_vec(5)
        ).define(format!("oa_gf_parent_angular_{}", joint_id).as_str())
    };

    //motionSet::motionAction(data.oa_gf[parent],J_cols,dAdq_cols);
    let (mut dAdq_cols_linear, mut dAdq_cols_angular) = motionAction(&oa_gf_parent_linear, &oa_gf_parent_angular, &J_cols_linear, &J_cols_angular, joint_id);

    // dAdv_cols = dJ_cols;
    let mut dAdv_cols_linear = dJ_cols_linear.clone().define(format!("dAdv_cols_linear_{}", joint_id).as_str());
    let mut dAdv_cols_angular = dJ_cols_angular.clone().define(format!("dAdv_cols_angular_{}", joint_id).as_str());
    
    //if(parent > 0)
    //  {
    //    motionSet::motionAction(data.ov[parent],J_cols,dVdq_cols);
    //    motionSet::motionAction<ADDTO>(data.ov[parent],dVdq_cols,dAdq_cols);
    //    dAdv_cols.noalias() += dVdq_cols;
    //  }
    //  else
    //  {
    //    dVdq_cols.setZero();
    //  }

    let (dvdq_cols_linear, dvdq_cols_angular) = match joint_id {
        0 => (Vector!(0.0, 0.0, 0.0).define(format!("dvdq_cols_linear_{}", joint_id).as_str()), Vector!(0.0, 0.0, 0.0).define(format!("dvdq_cols_angular_{}", joint_id).as_str())),
        _ => {
            // joint_id is one more than what it actually should be, so in parent for oa_gfs I should check joint_id for parent
            let data_ov_parent_linear = Vector!(
                all_ov[joint_id - 1].at_vec(0),
                all_ov[joint_id - 1].at_vec(1),
                all_ov[joint_id - 1].at_vec(2)
            ).define(format!("data_ov_parent_linear_{}", joint_id).as_str());
            let data_ov_parent_angular = Vector!(
                all_ov[joint_id - 1].at_vec(3),
                all_ov[joint_id - 1].at_vec(4),
                all_ov[joint_id - 1].at_vec(5)
            ).define(format!("data_ov_parent_angular_{}", joint_id).as_str());
            motionAction(&data_ov_parent_linear, &data_ov_parent_angular, &J_cols_linear, &J_cols_angular, joint_id)
        }
    };

    //    motionSet::motionAction<ADDTO>(data.ov[parent],dVdq_cols,dAdq_cols);
    //    dAdv_cols.noalias() += dVdq_cols;
    match joint_id {
        0 => (),
        _ => {
            // joint_id is one more than what it actually should be, so in parent for oa_gfs I should check joint_id for parent
            let data_ov_parent_linear = Vector!(
                all_ov[joint_id - 1].at_vec(0),
                all_ov[joint_id - 1].at_vec(1),
                all_ov[joint_id - 1].at_vec(2)
            ).define(format!("data_ov_parent_linear_{}", joint_id).as_str());
            let data_ov_parent_angular = Vector!(
                all_ov[joint_id - 1].at_vec(3),
                all_ov[joint_id - 1].at_vec(4),
                all_ov[joint_id - 1].at_vec(5)
            ).define(format!("data_ov_parent_angular_{}", joint_id).as_str());
            let (dAdq_add_linear, dAdq_add_angular) = motionAction(&data_ov_parent_linear, &data_ov_parent_angular, &dvdq_cols_linear, &dvdq_cols_angular, joint_id);
            dAdq_cols_linear = (dAdq_cols_linear + dAdq_add_linear).define(format!("dAdq_cols_linear_{}", joint_id).as_str());
            dAdq_cols_angular = (dAdq_cols_angular + dAdq_add_angular).define(format!("dAdq_cols_angular_{}", joint_id).as_str());
            dAdv_cols_linear = (dAdv_cols_linear + &dvdq_cols_linear).define(format!("dAdv_cols_linear_updated_{}", joint_id).as_str());
            dAdv_cols_angular = (dAdv_cols_angular + &dvdq_cols_angular).define(format!("dAdv_cols_angular_updated_{}", joint_id).as_str());
        }
    };

    dAdq_cols.push(Vector!(
        dAdq_cols_linear.at_vec(0),
        dAdq_cols_linear.at_vec(1),
        dAdq_cols_linear.at_vec(2),
        dAdq_cols_angular.at_vec(0),
        dAdq_cols_angular.at_vec(1),
        dAdq_cols_angular.at_vec(2)
    ).define(format!("dAdq_cols_{}", joint_id).as_str()));

    dAdv_cols.push(Vector!(
        dAdv_cols_linear.at_vec(0),
        dAdv_cols_linear.at_vec(1),
        dAdv_cols_linear.at_vec(2),
        dAdv_cols_angular.at_vec(0),
        dAdv_cols_angular.at_vec(1),
        dAdv_cols_angular.at_vec(2)
    ).define(format!("dAdv_cols_{}", joint_id).as_str()));

    dVdq_cols.push(Vector!(
        dvdq_cols_linear.at_vec(0),
        dvdq_cols_linear.at_vec(1),
        dvdq_cols_linear.at_vec(2),
        dvdq_cols_angular.at_vec(0),
        dvdq_cols_angular.at_vec(1),
        dvdq_cols_angular.at_vec(2)
    ).define(format!("dVdq_cols_{}", joint_id).as_str()));

    
    //// computes variation of inertias
    //data.doYcrb[i] = data.oYcrb[i].variation(ov);
    //
    //addForceCrossMatrix(data.oh[i],data.doYcrb[i]);
    
    let data_doycrb_i = inertia_variation(&data_oycrb_rot_i.clone(), &data_oycrb_trans_i.clone(), &ov_linear, &ov_angular, &masses[joint_id], joint_id);
    
    let data_doycrb_i = add_force_cross_matrix(&data_oh_linear.clone(), &data_oh_angular.clone(), &data_doycrb_i.clone(), joint_id);

    all_doycrb.push(data_doycrb_i.clone());
}

// InertiaTpl __plus__(const InertiaTpl & Yb) const
// {
// /* Y_{a+b} = ( m_a+m_b,
// *             (m_a*c_a + m_b*c_b ) / (m_a + m_b),
// *             I_a + I_b - (m_a*m_b)/(m_a+m_b) * AB_x * AB_x )
// */
// 
// const Scalar eps = ::Eigen::NumTraits<Scalar>::epsilon();
// 
// const Scalar & mab = mass()+Yb.mass();
// const Scalar mab_inv = Scalar(1)/math::max((Scalar)(mass()+Yb.mass()),eps);
// const Vector3 & AB = (lever()-Yb.lever()).eval();
// return InertiaTpl(mab,
//                 (mass()*lever()+Yb.mass()*Yb.lever())*mab_inv,
//                 inertia()+Yb.inertia() - (mass()*Yb.mass()*mab_inv)* typename Symmetric3::SkewSquare(AB));
// }
fn add_inertia(
    lhs_mass: &ASTNode, lhs_lever: &ASTNode, lhs_inertia: &ASTNode,
    rhs_mass: &ASTNode, rhs_lever: &ASTNode, rhs_inertia: &ASTNode,
    joint_id: usize
) -> (ASTNode, ASTNode, ASTNode) {
    let eps = 1e-10;
    let new_mass = (lhs_mass + rhs_mass).define(format!("new_mass_{}", joint_id - 1).as_str());
    
    let mab_inv = (Scalar!(1.0) / &new_mass).define(format!("mab_inv_{}", joint_id - 1).as_str());

    let mass_times_lever_lhs = (lhs_mass * lhs_lever).define(format!("mass_times_lever_lhs_{}", joint_id - 1).as_str());
    let mass_times_lever_rhs = (rhs_mass * rhs_lever).define(format!("mass_times_lever_rhs_{}", joint_id - 1).as_str());
    let mass_times_lever_sum = (mass_times_lever_lhs + mass_times_lever_rhs).define(format!("mass_times_lever_sum_{}", joint_id - 1).as_str());
    let new_lever = (&mab_inv * mass_times_lever_sum).define(format!("new_lever_{}", joint_id - 1).as_str());

    let inertia_sum = (lhs_inertia + rhs_inertia).define("inertia_sum");
    let mass_times_mass = (lhs_mass * rhs_mass).define("mass_times_mass");
    let mass_times_mass_mab_inv = (mass_times_mass * &mab_inv).define("mass_times_mass_mab_inv");

    let ab = (lhs_lever - rhs_lever).define("ab"); // eval? // AB must be parent_lever - child_lever
    // this skew_square is implemented in pinocchio/include/pinocchio/spatial/symmetric3.hpp
    //    const Scalar & x = v[0], & y = v[1], & z = v[2];
    //    return Symmetric3Tpl( -y*y-z*z,
    //                         x*y    ,  -x*x-z*z,
    //                         x*z    ,   y*z    ,  -x*x-y*y );
    // I don't want to put this as a function, as there are other skewSquare functions that have different implementations
    let x = &ab.at_vec(0);
    let y = &ab.at_vec(1);
    let z = &ab.at_vec(2);
    let skew_square_ab = Matrix!(
        [-y*y-z*z, x*y, x*z],
        [x*y, -x*x-z*z, y*z],
        [x*z, y*z, -x*x-y*y]
    ).define("skew_square_ab");

    let new_inertia_temp = (mass_times_mass_mab_inv * skew_square_ab).define("new_inertia_temp");

    let new_inertia = (inertia_sum - new_inertia_temp).define(format!("new_inertia_{}", joint_id - 1).as_str());

    (new_mass, new_lever, new_inertia)
}

fn second_pass(
    j_cols_vec: &Vec<ASTNode>,
    dVdq_cols: &Vec<ASTNode>,
    dAdq_cols: &mut Vec<ASTNode>,
    dAdv_cols: &mut Vec<ASTNode>,
    dFdq_cols: &mut Vec<ASTNode>,
    dFdv_cols: &mut Vec<ASTNode>,
    dFda_cols: &mut Vec<ASTNode>,
    rnea_partial_dq_rows: &mut Vec<ASTNode>,
    rnea_partial_dv_rows: &mut Vec<ASTNode>,
    rnea_partial_da: &mut Vec<ASTNode>,
    of: &mut Vec<ASTNode>,
    taus: &mut Vec<ASTNode>,
    all_oycrb: &mut Vec<(ASTNode, ASTNode)>,
    all_doycrb: &mut Vec<ASTNode>,
    masses: &mut Vec<ASTNode>,
    levers: &mut Vec<ASTNode>,
    joint_id: usize,
) {

    let j_cols = j_cols_vec[joint_id].clone();
    //jmodel.jointVelocitySelector(data.tau).noalias() = J_cols.transpose()*data.of[i].toVector();
    let of_i = of[joint_id].clone();
    let of_parent = match joint_id {
        0 => Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        _ => of[joint_id - 1].clone()
    };
    let tau = j_cols.dot(&of_i).define(format!("tau_{}", joint_id).as_str());
    taus.push(tau.clone());

    //if(parent>0)
    //  {
    //    data.oYcrb[parent] += data.oYcrb[i];
    //    data.doYcrb[parent] += data.doYcrb[i];
    //    data.of[parent] += data.of[i];
    //  }
    match joint_id {
        0 => (),
        _ => {
            let oy = add_inertia(
                &masses[joint_id - 1], &all_oycrb[joint_id - 1].1, &all_oycrb[joint_id - 1].0,
                &masses[joint_id],     &all_oycrb[joint_id].1,     &all_oycrb[joint_id].0,
                joint_id
            );
            masses[joint_id - 1] = oy.0; // correct
            all_oycrb[joint_id - 1] = (oy.2, oy.1); // correct

            let new_doycrb = (all_doycrb[joint_id - 1].clone() + all_doycrb[joint_id].clone()).define(format!("new_doycrb_{}", joint_id - 1).as_str());
            all_doycrb[joint_id - 1] = new_doycrb;

            of[joint_id - 1] = (of_parent + &of_i).define(format!("of_parent_updated_{}", joint_id).as_str());
        }
    }

    let oycrb_i = all_oycrb[joint_id].clone(); // This is not updated yet, and it shouldn't be. Be careful!
    let doycrb_i = all_doycrb[joint_id].clone(); // This is not updated yet, and it shouldn't be. Be careful!

    // motionSet::inertiaAction(data.oYcrb[i],J_cols,dFda_cols);
    let dFda_cols_ = inertia_vec_mult(&masses[joint_id], &oycrb_i.1, &oycrb_i.0, &j_cols, joint_id);
    //dFda_cols.push(dFda_cols_.define(format!("dFda_cols_{}", joint_id).as_str()));
    dFda_cols[joint_id] = dFda_cols_;

    // construct new dFda to use, using all cols
    let dFda_temp = Matrix!(
        [dFda_cols[0].at_vec(0), dFda_cols[1].at_vec(0), dFda_cols[2].at_vec(0), dFda_cols[3].at_vec(0), dFda_cols[4].at_vec(0), dFda_cols[5].at_vec(0)],
        [dFda_cols[0].at_vec(1), dFda_cols[1].at_vec(1), dFda_cols[2].at_vec(1), dFda_cols[3].at_vec(1), dFda_cols[4].at_vec(1), dFda_cols[5].at_vec(1)],
        [dFda_cols[0].at_vec(2), dFda_cols[1].at_vec(2), dFda_cols[2].at_vec(2), dFda_cols[3].at_vec(2), dFda_cols[4].at_vec(2), dFda_cols[5].at_vec(2)],
        [dFda_cols[0].at_vec(3), dFda_cols[1].at_vec(3), dFda_cols[2].at_vec(3), dFda_cols[3].at_vec(3), dFda_cols[4].at_vec(3), dFda_cols[5].at_vec(3)],
        [dFda_cols[0].at_vec(4), dFda_cols[1].at_vec(4), dFda_cols[2].at_vec(4), dFda_cols[3].at_vec(4), dFda_cols[4].at_vec(4), dFda_cols[5].at_vec(4)],
        [dFda_cols[0].at_vec(5), dFda_cols[1].at_vec(5), dFda_cols[2].at_vec(5), dFda_cols[3].at_vec(5), dFda_cols[4].at_vec(5), dFda_cols[5].at_vec(5)]
    ).define(format!("dFda_temp_{}", joint_id).as_str());

    // rnea_partial_da_.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]).noalias()
    // = J_cols.transpose()*data.dFda.middleCols(jmodel.idx_v(),data.nvSubtree[i]);

    let j_cols_matrix = Matrix!(
        [j_cols.at_vec(0), j_cols.at_vec(1), j_cols.at_vec(2), j_cols.at_vec(3), j_cols.at_vec(4), j_cols.at_vec(5)]
    ).define(format!("j_cols_matrix_{}", joint_id).as_str());
    
    let rnea_partial_da_temp = j_cols_matrix.cross(&dFda_temp).define(format!("rnea_partial_da_temp_{}", joint_id).as_str());
    rnea_partial_da[joint_id] = rnea_partial_da_temp;

    // dFdv_cols.noalias() = data.doYcrb[i] * J_cols;
    let dFdv_cols_temp = doycrb_i.cross(&j_cols).define(format!("dFdv_cols_temp_{}", joint_id).as_str());
    // motionSet::inertiaAction<ADDTO>(data.oYcrb[i],dAdv_cols,dFdv_cols);
    let dFdv_cols_temp2 = inertia_vec_mult(&masses[joint_id], &oycrb_i.1, &oycrb_i.0, &dAdv_cols[joint_id], joint_id);
    let dFdv_cols_final = (dFdv_cols_temp + dFdv_cols_temp2).define(format!("dFdv_cols_temp3_{}", joint_id).as_str());
    dFdv_cols[joint_id] = dFdv_cols_final.clone();

    let dFdv_temp = Matrix!(
        [dFdv_cols[0].at_vec(0), dFdv_cols[1].at_vec(0), dFdv_cols[2].at_vec(0), dFdv_cols[3].at_vec(0), dFdv_cols[4].at_vec(0), dFdv_cols[5].at_vec(0)],
        [dFdv_cols[0].at_vec(1), dFdv_cols[1].at_vec(1), dFdv_cols[2].at_vec(1), dFdv_cols[3].at_vec(1), dFdv_cols[4].at_vec(1), dFdv_cols[5].at_vec(1)],
        [dFdv_cols[0].at_vec(2), dFdv_cols[1].at_vec(2), dFdv_cols[2].at_vec(2), dFdv_cols[3].at_vec(2), dFdv_cols[4].at_vec(2), dFdv_cols[5].at_vec(2)],
        [dFdv_cols[0].at_vec(3), dFdv_cols[1].at_vec(3), dFdv_cols[2].at_vec(3), dFdv_cols[3].at_vec(3), dFdv_cols[4].at_vec(3), dFdv_cols[5].at_vec(3)],
        [dFdv_cols[0].at_vec(4), dFdv_cols[1].at_vec(4), dFdv_cols[2].at_vec(4), dFdv_cols[3].at_vec(4), dFdv_cols[4].at_vec(4), dFdv_cols[5].at_vec(4)],
        [dFdv_cols[0].at_vec(5), dFdv_cols[1].at_vec(5), dFdv_cols[2].at_vec(5), dFdv_cols[3].at_vec(5), dFdv_cols[4].at_vec(5), dFdv_cols[5].at_vec(5)]
    ).define(format!("dFdv_temp_{}", joint_id).as_str());


    //rnea_partial_dv_.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]).noalias()
    //  = J_cols.transpose()*data.dFdv.middleCols(jmodel.idx_v(),data.nvSubtree[i]);

    let rnea_partial_dv_temp = j_cols_matrix.cross(&dFdv_temp).define(format!("rnea_partial_dv_temp_{}", joint_id).as_str());
    //rnea_partial_dv[joint_id] = rnea_partial_dv_temp;

    let mut rnea_partial_dv_temp_vec = vec![
        rnea_partial_dv_temp.at_mat(0, 0),
        rnea_partial_dv_temp.at_mat(0, 1),
        rnea_partial_dv_temp.at_mat(0, 2),
        rnea_partial_dv_temp.at_mat(0, 3),
        rnea_partial_dv_temp.at_mat(0, 4),
        rnea_partial_dv_temp.at_mat(0, 5)
    ];

    // // dtau/dq
    // if(parent>0)
    // {
    //   dFdq_cols.noalias() = data.doYcrb[i] * dVdq_cols;
    //   motionSet::inertiaAction<ADDTO>(data.oYcrb[i],dAdq_cols,dFdq_cols);
    //   std::cout << "dFdq_cols temp 1: \n" << dFdq_cols << std::endl;  
    // }
    // else{
    //   motionSet::inertiaAction(data.oYcrb[i],dAdq_cols,dFdq_cols);
    //   std::cout << "dFdq_cols temp 2: \n" << dFdq_cols << std::endl;
    // }
    dFdq_cols[joint_id] = match joint_id {
        // motionSet::inertiaAction(data.oYcrb[i],dAdq_cols,dFdq_cols);
        0 => inertia_vec_mult(&masses[joint_id], &all_oycrb[joint_id].1, &all_oycrb[joint_id].0, &dAdq_cols[joint_id], joint_id),
        ///   dFdq_cols.noalias() = data.doYcrb[i] * dVdq_cols;
        ///   motionSet::inertiaAction<ADDTO>(data.oYcrb[i],dAdq_cols,dFdq_cols);
        _ => {
            let dFdq_temp = doycrb_i.cross(&dVdq_cols[joint_id]).define(format!("dFdq_temp_{}", joint_id).as_str());
            let dFdq_temp2 = inertia_vec_mult(&masses[joint_id], &oycrb_i.1, &oycrb_i.0, &dAdq_cols[joint_id], joint_id);
            (dFdq_temp + dFdq_temp2).define(format!("dFdq_temp3_{}", joint_id).as_str())
        }
    };

    let dFdq_temp = Matrix!(
        [dFdq_cols[0].at_vec(0), dFdq_cols[1].at_vec(0), dFdq_cols[2].at_vec(0), dFdq_cols[3].at_vec(0), dFdq_cols[4].at_vec(0), dFdq_cols[5].at_vec(0)],
        [dFdq_cols[0].at_vec(1), dFdq_cols[1].at_vec(1), dFdq_cols[2].at_vec(1), dFdq_cols[3].at_vec(1), dFdq_cols[4].at_vec(1), dFdq_cols[5].at_vec(1)],
        [dFdq_cols[0].at_vec(2), dFdq_cols[1].at_vec(2), dFdq_cols[2].at_vec(2), dFdq_cols[3].at_vec(2), dFdq_cols[4].at_vec(2), dFdq_cols[5].at_vec(2)],
        [dFdq_cols[0].at_vec(3), dFdq_cols[1].at_vec(3), dFdq_cols[2].at_vec(3), dFdq_cols[3].at_vec(3), dFdq_cols[4].at_vec(3), dFdq_cols[5].at_vec(3)],
        [dFdq_cols[0].at_vec(4), dFdq_cols[1].at_vec(4), dFdq_cols[2].at_vec(4), dFdq_cols[3].at_vec(4), dFdq_cols[4].at_vec(4), dFdq_cols[5].at_vec(4)],
        [dFdq_cols[0].at_vec(5), dFdq_cols[1].at_vec(5), dFdq_cols[2].at_vec(5), dFdq_cols[3].at_vec(5), dFdq_cols[4].at_vec(5), dFdq_cols[5].at_vec(5)]
    ).define(format!("dFdq_temp_{}", joint_id).as_str());

    // rnea_partial_dq_.block(jmodel.idx_v(),jmodel.idx_v(),jmodel.nv(),data.nvSubtree[i]).noalias()
    // = J_cols.transpose()*data.dFdq.middleCols(jmodel.idx_v(),data.nvSubtree[i]); // TODO:
    let mut rnea_partial_dq_temp = j_cols_matrix.cross(&dFdq_temp).define(format!("rnea_partial_dq_temp_{}", joint_id).as_str());
    let mut rnea_partial_dq_temp_vec = vec![
        rnea_partial_dq_temp.at_mat(0, 0),
        rnea_partial_dq_temp.at_mat(0, 1),
        rnea_partial_dq_temp.at_mat(0, 2),
        rnea_partial_dq_temp.at_mat(0, 3),
        rnea_partial_dq_temp.at_mat(0, 4),
        rnea_partial_dq_temp.at_mat(0, 5)
    ];

    // motionSet::act<ADDTO>(J_cols,data.of[i],dFdq_cols);
    let J_cols_cross_of_i = cross_6d(&j_cols, &of_i).define(format!("J_cols_cross_of_i_{}", joint_id).as_str());
    dFdq_cols[joint_id] = (&dFdq_cols[joint_id] + J_cols_cross_of_i).define(format!("dFdq_cols_final_{}", joint_id).as_str());

    let dFdq_temp2 = Matrix!(
        [dFdq_cols[0].at_vec(0), dFdq_cols[1].at_vec(0), dFdq_cols[2].at_vec(0), dFdq_cols[3].at_vec(0), dFdq_cols[4].at_vec(0), dFdq_cols[5].at_vec(0)],
        [dFdq_cols[0].at_vec(1), dFdq_cols[1].at_vec(1), dFdq_cols[2].at_vec(1), dFdq_cols[3].at_vec(1), dFdq_cols[4].at_vec(1), dFdq_cols[5].at_vec(1)],
        [dFdq_cols[0].at_vec(2), dFdq_cols[1].at_vec(2), dFdq_cols[2].at_vec(2), dFdq_cols[3].at_vec(2), dFdq_cols[4].at_vec(2), dFdq_cols[5].at_vec(2)],
        [dFdq_cols[0].at_vec(3), dFdq_cols[1].at_vec(3), dFdq_cols[2].at_vec(3), dFdq_cols[3].at_vec(3), dFdq_cols[4].at_vec(3), dFdq_cols[5].at_vec(3)],
        [dFdq_cols[0].at_vec(4), dFdq_cols[1].at_vec(4), dFdq_cols[2].at_vec(4), dFdq_cols[3].at_vec(4), dFdq_cols[4].at_vec(4), dFdq_cols[5].at_vec(4)],
        [dFdq_cols[0].at_vec(5), dFdq_cols[1].at_vec(5), dFdq_cols[2].at_vec(5), dFdq_cols[3].at_vec(5), dFdq_cols[4].at_vec(5), dFdq_cols[5].at_vec(5)]
    ).define(format!("dFdq_temp2_{}", joint_id).as_str());

    // There should be a better way to implement this
    // there is a if(parent > 0), but since this is backward pass,
    // lhsInertiaMult(data.oYcrb[i],J_cols.transpose(),M6tmpR.topRows(jmodel.nv()));
    let M6tmpR = inertia_vec_mult(&masses[joint_id], &oycrb_i.1, &oycrb_i.0, &j_cols, joint_id).define(format!("M6tmpR_{}", joint_id).as_str());
    let M6tmpR2 = j_cols.cross(&doycrb_i).define(format!("M6tmpR2_{}", joint_id).as_str());

    //for(int j = data.parents_fromRow[(typename Model::Index)jmodel.idx_v()];j >= 0; j = data.parents_fromRow[(typename Model::Index)j])
    //{
    //  rnea_partial_dq_.middleRows(jmodel.idx_v(),jmodel.nv()).col(j).noalias()
    //  = M6tmpR.topRows(jmodel.nv()) * data.dAdq.col(j)
    //  + M6tmpR2.topRows(jmodel.nv()) * data.dVdq.col(j); // TODO:seems kinda correct, but I'll need to check after I update dAdq, too
    //  std::cout << "rnea_partial_dq_: " << rnea_partial_dq_ << std::endl;
    //}
    for j in (0..joint_id){
        let dAdq_cols = dAdq_cols[j].clone();
        let dVdq_cols = dVdq_cols[j].clone();
        let rnea_partial_dq_temp2 = M6tmpR.dot(&dAdq_cols).define(format!("rnea_partial_dq_temp2_{}_{}", joint_id, j).as_str());
        let rnea_partial_dq_temp3 = M6tmpR2.dot(&dVdq_cols).define(format!("rnea_partial_dq_temp3_{}_{}", joint_id, j).as_str());
        let rnea_partial_dq_temp4 = (rnea_partial_dq_temp2 + rnea_partial_dq_temp3).define(format!("rnea_partial_dq_temp4_{}_{}", joint_id, j).as_str());

        // now, apart from the element rnea_partial_dq_temp[j][joint_id], make everything zero, and that field should be rnea_partial_dq_temp4
        // then add this to rnea_partial_dq_temp
        //rnea_partial_dq_temp = rnea_partial_dq_temp.set_mat_at(j, joint_id, rnea_partial_dq_temp4.at_vec(0)).define(format!("rnea_partial_dq_temp5_{}_{}", joint_id, j).as_str());
        rnea_partial_dq_temp_vec[j] = rnea_partial_dq_temp4;           
    }
    
    rnea_partial_dq_rows[joint_id] = Vector!(
        rnea_partial_dq_temp_vec[0].clone(),
        rnea_partial_dq_temp_vec[1].clone(),
        rnea_partial_dq_temp_vec[2].clone(),
        rnea_partial_dq_temp_vec[3].clone(),
        rnea_partial_dq_temp_vec[4].clone(),
        rnea_partial_dq_temp_vec[5].clone()
    ).define(format!("rnea_partial_dq_rows_{}", joint_id).as_str());


    // now rnea_partial_dv
    //rnea_partial_dv_.middleRows(jmodel.idx_v(),jmodel.nv()).col(j).noalias()
    //= M6tmpR.topRows(jmodel.nv()) * data.dAdv.col(j)
    //+ M6tmpR2.topRows(jmodel.nv()) * data.J.col(j); // TODO:
    for j in (0..joint_id){
        let dAdv_cols = dAdv_cols[j].clone();
        let J_cols = j_cols_vec[j].clone();
        let rnea_partial_dv_temp2 = M6tmpR.dot(&dAdv_cols).define(format!("rnea_partial_dv_temp2_{}_{}", joint_id, j).as_str());
        let rnea_partial_dv_temp3 = M6tmpR2.dot(&J_cols).define(format!("rnea_partial_dv_temp3_{}_{}", joint_id, j).as_str());
        let rnea_partial_dv_temp4 = (rnea_partial_dv_temp2 + rnea_partial_dv_temp3).define(format!("rnea_partial_dv_temp4_{}_{}", joint_id, j).as_str());

        // now, apart from the element rnea_partial_dv_temp[j][joint_id], make everything zero, and that field should be rnea_partial_dv_temp4
        // then add this to rnea_partial_dv_temp
        //rnea_partial_dv_temp = rnea_partial_dv_temp.set_mat_at(j, joint_id, rnea_partial_dv_temp4.at_vec(0)).define(format!("rnea_partial_dv_temp5_{}_{}", joint_id, j).as_str());
        rnea_partial_dv_temp_vec[j] = rnea_partial_dv_temp4;           
    }

    rnea_partial_dv_rows[joint_id] = Vector!(
        rnea_partial_dv_temp_vec[0].clone(),
        rnea_partial_dv_temp_vec[1].clone(),
        rnea_partial_dv_temp_vec[2].clone(),
        rnea_partial_dv_temp_vec[3].clone(),
        rnea_partial_dv_temp_vec[4].clone(),
        rnea_partial_dv_temp_vec[5].clone()
    ).define(format!("rnea_partial_dv_rows_{}", joint_id).as_str());

    


    //for(Eigen::DenseIndex k =0; k < jmodel.nv(); ++k)
    //  {
    //    MotionRef<typename ColsBlock::ColXpr> m_in(J_cols.col(k));
    //    MotionRef<typename ColsBlock::ColXpr> m_out(dAdq_cols.col(k));
    //    m_out.linear() += model.gravity.linear().cross(m_in.angular());
    //  }
    // we need to update dAdq_cols
    //dAdq_cols(k) += model.gravity.linear().cross(J_cols.col(k).angular());
    // jmodel.nv() is 1 always for revolute joints
    let gravity_linear = Vector!(0.0, 0.0, -9.81).define("gravity_linear");
    let j_cols_angular = Vector!(
        j_cols.at_vec(3), 
        j_cols.at_vec(4), 
        j_cols.at_vec(5)
    ).define("j_cols_angular");
    let cross_product = gravity_linear.cross(&j_cols_angular).define(format!("cross_product_{}", joint_id).as_str());
    let dAdq_cols_joint_linear = Vector!(
        dAdq_cols[joint_id].at_vec(0), 
        dAdq_cols[joint_id].at_vec(1), 
        dAdq_cols[joint_id].at_vec(2)
    ).define("dAdq_cols_joint_linear");
    let dAdq_cols_update = (dAdq_cols_joint_linear + cross_product).define(format!("dAdq_cols_temp_updated_{}", joint_id).as_str());
    let dAdq_cols_updated = Vector!(
        dAdq_cols_update.at_vec(0),
        dAdq_cols_update.at_vec(1),
        dAdq_cols_update.at_vec(2),
        dAdq_cols[joint_id].at_vec(3),
        dAdq_cols[joint_id].at_vec(4),
        dAdq_cols[joint_id].at_vec(5)
    ).define(format!("dAdq_cols_updated_{}", joint_id).as_str());

    dAdq_cols[joint_id] = dAdq_cols_updated;




}



/// Both model.nq and model.nv are 6 for panda, the hand joints are excluded
/// \param[in] q The joint configuration vector (dim model.nq).
/// \param[in] v The joint velocity vector (dim model.nv).
/// \param[in] a The joint acceleration vector (dim model.nv).
/// jointPlacements are model.jointPlacements in pinocchio, it is a vector of SE3 objects
/// SE3 has a rotation and a translation element
pub fn rneaderivatives(qsin: ASTNode, qcos: ASTNode, v: ASTNode, a: ASTNode) {
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

    let mut levers = vec![
        Vector!(0.003875, 0.002081, -0.04762).define("lever_0"),
        Vector!(-0.003141, -0.02872, 0.003495).define("lever_1"),
        Vector!(0.027518, 0.039252, -0.066502).define("lever_2"),
        Vector!(-0.05317, 0.104419, 0.027454).define("lever_3"),
        Vector!(-0.011953, 0.041065, -0.038437).define("lever_4"),
        Vector!(0.060149, -0.014117, -0.010517).define("lever_5")
    ];

    //let masses = Vector!(4.970684, 0.646926, 3.228604, 3.587895, 1.225946, 1.66656).define("masses");
    let mut masses = vec![
        Scalar!(4.970684).define("mass_0"),
        Scalar!(0.646926).define("mass_1"),
        Scalar!(3.228604).define("mass_2"),
        Scalar!(3.587895).define("mass_3"),
        Scalar!(1.225946).define("mass_4"),
        Scalar!(1.66656).define("mass_5")
    ];

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
    let mut all_of: Vec<ASTNode> = vec![];
    let mut all_oh: Vec<ASTNode> = vec![];
    let mut all_doycrb: Vec<ASTNode> = vec![];
    let mut all_ov: Vec<ASTNode> = vec![];
    let mut all_oa: Vec<ASTNode> = vec![];
    let mut all_oa_gf: Vec<ASTNode> = vec![];
    let mut all_a: Vec<ASTNode> = vec![];
    let mut all_oycrb: Vec<(ASTNode, ASTNode)> = vec![];
    let mut j_cols: Vec<ASTNode> = vec![];
    let mut dj_cols: Vec<ASTNode> = vec![];
    let mut dAdq_cols: Vec<ASTNode> = vec![];
    let mut dAdv_cols: Vec<ASTNode> = vec![];
    let mut dVdq_cols: Vec<ASTNode> = vec![];
    let mut oMis: Vec<(ASTNode, ASTNode)> = Vec::new();

    // first pass, it takes model.joints[i], data.joints[i], model, data, q, v, a
    for i in 0..n_joints {
        first_pass(
            qsin.at_vec(i), // qsin and qcos will not change, therefore no reference needed
            qcos.at_vec(i),
            &data_v[i],
            &v,
            &a,
            &mut all_v,
            &mut oMis,
            &limi_translations,
            &mut limi_rotations,
            i,
            &levers,
            &masses,
            &inertias,
            &mut all_of,
            &mut all_oh,
            &mut all_doycrb,
            &mut all_ov,
            &mut all_oa,
            &mut all_oa_gf,
            &mut all_a,
            &mut all_oycrb,
            &mut j_cols,
            &mut dj_cols,
            &mut dAdq_cols,
            &mut dAdv_cols,
            &mut dVdq_cols
        );
    }

    let mut taus: Vec<ASTNode> = vec![];
    let mut dFdq_cols: Vec<ASTNode> = vec![];
    let mut dFdv_cols: Vec<ASTNode> = vec![];
    let mut dFda_cols: Vec<ASTNode> = vec![];
    let mut rnea_partial_dv: ASTNode = Matrix!( // need a better way to initialize these!
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ).define("rnea_partial_dv_initial");

    let mut dFda_cols: Vec<ASTNode> = vec![
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("dFda_cols_init_0"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("dFda_cols_init_1"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("dFda_cols_init_2"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("dFda_cols_init_3"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("dFda_cols_init_4"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("dFda_cols_init_5")
    ];
    let mut dFdv_cols: Vec<ASTNode> = vec![
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("dFdv_cols_init_0"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("dFdv_cols_init_1"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("dFdv_cols_init_2"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("dFdv_cols_init_3"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("dFdv_cols_init_4"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("dFdv_cols_init_5")
    ];
    let mut dFdq_cols: Vec<ASTNode> = vec![
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("dFdq_cols_init_0"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("dFdq_cols_init_1"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("dFdq_cols_init_2"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("dFdq_cols_init_3"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("dFdq_cols_init_4"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("dFdq_cols_init_5")
    ];
    let mut rnea_partial_da_cols: Vec<ASTNode> = vec![
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("rnea_partial_da_cols_init_0"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("rnea_partial_da_cols_init_1"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("rnea_partial_da_cols_init_2"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("rnea_partial_da_cols_init_3"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("rnea_partial_da_cols_init_4"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("rnea_partial_da_cols_init_5")
    ];
    let mut rnea_partial_dv_cols: Vec<ASTNode> = vec![
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("rnea_partial_dv_cols_init_0"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("rnea_partial_dv_cols_init_1"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("rnea_partial_dv_cols_init_2"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("rnea_partial_dv_cols_init_3"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("rnea_partial_dv_cols_init_4"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("rnea_partial_dv_cols_init_5")
    ];
    let mut rnea_partial_dq_cols: Vec<ASTNode> = vec![
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("rnea_partial_dq_cols_init_0"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("rnea_partial_dq_cols_init_1"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("rnea_partial_dq_cols_init_2"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("rnea_partial_dq_cols_init_3"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("rnea_partial_dq_cols_init_4"),
        Vector!(0.0, 0.0, 0.0, 0.0, 0.0, 0.0).define("rnea_partial_dq_cols_init_5")
    ];

    for i in (0..n_joints).rev(){
        second_pass(
            &j_cols,
            &dVdq_cols,
            &mut dAdq_cols,
            &mut dAdv_cols,
            &mut dFdq_cols,
            &mut dFdv_cols,
            &mut dFda_cols,
            &mut rnea_partial_dq_cols,
            &mut rnea_partial_dv_cols,
            &mut rnea_partial_da_cols,
            &mut all_of,
            &mut taus,
            &mut all_oycrb,
            &mut all_doycrb,
            &mut masses,
            &mut levers,
            i
        );
    }

    let dFda_final: ASTNode = Matrix!(
        [dFda_cols[0].at_vec(0), dFda_cols[1].at_vec(0), dFda_cols[2].at_vec(0), dFda_cols[3].at_vec(0), dFda_cols[4].at_vec(0), dFda_cols[5].at_vec(0)],
        [dFda_cols[0].at_vec(1), dFda_cols[1].at_vec(1), dFda_cols[2].at_vec(1), dFda_cols[3].at_vec(1), dFda_cols[4].at_vec(1), dFda_cols[5].at_vec(1)],
        [dFda_cols[0].at_vec(2), dFda_cols[1].at_vec(2), dFda_cols[2].at_vec(2), dFda_cols[3].at_vec(2), dFda_cols[4].at_vec(2), dFda_cols[5].at_vec(2)],
        [dFda_cols[0].at_vec(3), dFda_cols[1].at_vec(3), dFda_cols[2].at_vec(3), dFda_cols[3].at_vec(3), dFda_cols[4].at_vec(3), dFda_cols[5].at_vec(3)],
        [dFda_cols[0].at_vec(4), dFda_cols[1].at_vec(4), dFda_cols[2].at_vec(4), dFda_cols[3].at_vec(4), dFda_cols[4].at_vec(4), dFda_cols[5].at_vec(4)],
        [dFda_cols[0].at_vec(5), dFda_cols[1].at_vec(5), dFda_cols[2].at_vec(5), dFda_cols[3].at_vec(5), dFda_cols[4].at_vec(5), dFda_cols[5].at_vec(5)]
    ).define("dFda_final");

    let rnea_partial_da: ASTNode = Matrix!(
        [rnea_partial_da_cols[0].at_mat(0, 0), rnea_partial_da_cols[0].at_mat(0, 1), rnea_partial_da_cols[0].at_mat(0, 2), rnea_partial_da_cols[0].at_mat(0, 3), rnea_partial_da_cols[0].at_mat(0, 4), rnea_partial_da_cols[0].at_mat(0, 5)],
        [rnea_partial_da_cols[1].at_mat(0, 0), rnea_partial_da_cols[1].at_mat(0, 1), rnea_partial_da_cols[1].at_mat(0, 2), rnea_partial_da_cols[1].at_mat(0, 3), rnea_partial_da_cols[1].at_mat(0, 4), rnea_partial_da_cols[1].at_mat(0, 5)],
        [rnea_partial_da_cols[2].at_mat(0, 0), rnea_partial_da_cols[2].at_mat(0, 1), rnea_partial_da_cols[2].at_mat(0, 2), rnea_partial_da_cols[2].at_mat(0, 3), rnea_partial_da_cols[2].at_mat(0, 4), rnea_partial_da_cols[2].at_mat(0, 5)],
        [rnea_partial_da_cols[3].at_mat(0, 0), rnea_partial_da_cols[3].at_mat(0, 1), rnea_partial_da_cols[3].at_mat(0, 2), rnea_partial_da_cols[3].at_mat(0, 3), rnea_partial_da_cols[3].at_mat(0, 4), rnea_partial_da_cols[3].at_mat(0, 5)],
        [rnea_partial_da_cols[4].at_mat(0, 0), rnea_partial_da_cols[4].at_mat(0, 1), rnea_partial_da_cols[4].at_mat(0, 2), rnea_partial_da_cols[4].at_mat(0, 3), rnea_partial_da_cols[4].at_mat(0, 4), rnea_partial_da_cols[4].at_mat(0, 5)],
        [rnea_partial_da_cols[5].at_mat(0, 0), rnea_partial_da_cols[5].at_mat(0, 1), rnea_partial_da_cols[5].at_mat(0, 2), rnea_partial_da_cols[5].at_mat(0, 3), rnea_partial_da_cols[5].at_mat(0, 4), rnea_partial_da_cols[5].at_mat(0, 5)]
    ).define("rnea_partial_da_final");

    let dFdv_final: ASTNode = Matrix!(
        [dFdv_cols[0].at_vec(0), dFdv_cols[1].at_vec(0), dFdv_cols[2].at_vec(0), dFdv_cols[3].at_vec(0), dFdv_cols[4].at_vec(0), dFdv_cols[5].at_vec(0)],
        [dFdv_cols[0].at_vec(1), dFdv_cols[1].at_vec(1), dFdv_cols[2].at_vec(1), dFdv_cols[3].at_vec(1), dFdv_cols[4].at_vec(1), dFdv_cols[5].at_vec(1)],
        [dFdv_cols[0].at_vec(2), dFdv_cols[1].at_vec(2), dFdv_cols[2].at_vec(2), dFdv_cols[3].at_vec(2), dFdv_cols[4].at_vec(2), dFdv_cols[5].at_vec(2)],
        [dFdv_cols[0].at_vec(3), dFdv_cols[1].at_vec(3), dFdv_cols[2].at_vec(3), dFdv_cols[3].at_vec(3), dFdv_cols[4].at_vec(3), dFdv_cols[5].at_vec(3)],
        [dFdv_cols[0].at_vec(4), dFdv_cols[1].at_vec(4), dFdv_cols[2].at_vec(4), dFdv_cols[3].at_vec(4), dFdv_cols[4].at_vec(4), dFdv_cols[5].at_vec(4)],
        [dFdv_cols[0].at_vec(5), dFdv_cols[1].at_vec(5), dFdv_cols[2].at_vec(5), dFdv_cols[3].at_vec(5), dFdv_cols[4].at_vec(5), dFdv_cols[5].at_vec(5)]
    ).define("dFdv_final");

    let dFdq_final: ASTNode = Matrix!(
        [dFdq_cols[0].at_vec(0), dFdq_cols[1].at_vec(0), dFdq_cols[2].at_vec(0), dFdq_cols[3].at_vec(0), dFdq_cols[4].at_vec(0), dFdq_cols[5].at_vec(0)],
        [dFdq_cols[0].at_vec(1), dFdq_cols[1].at_vec(1), dFdq_cols[2].at_vec(1), dFdq_cols[3].at_vec(1), dFdq_cols[4].at_vec(1), dFdq_cols[5].at_vec(1)],
        [dFdq_cols[0].at_vec(2), dFdq_cols[1].at_vec(2), dFdq_cols[2].at_vec(2), dFdq_cols[3].at_vec(2), dFdq_cols[4].at_vec(2), dFdq_cols[5].at_vec(2)],
        [dFdq_cols[0].at_vec(3), dFdq_cols[1].at_vec(3), dFdq_cols[2].at_vec(3), dFdq_cols[3].at_vec(3), dFdq_cols[4].at_vec(3), dFdq_cols[5].at_vec(3)],
        [dFdq_cols[0].at_vec(4), dFdq_cols[1].at_vec(4), dFdq_cols[2].at_vec(4), dFdq_cols[3].at_vec(4), dFdq_cols[4].at_vec(4), dFdq_cols[5].at_vec(4)],
        [dFdq_cols[0].at_vec(5), dFdq_cols[1].at_vec(5), dFdq_cols[2].at_vec(5), dFdq_cols[3].at_vec(5), dFdq_cols[4].at_vec(5), dFdq_cols[5].at_vec(5)]
    ).define("dFdq_final");

    let rnea_partial_dv: ASTNode = Matrix!(
        [rnea_partial_dv_cols[0].at_vec(0), rnea_partial_dv_cols[1].at_vec(0), rnea_partial_dv_cols[2].at_vec(0), rnea_partial_dv_cols[3].at_vec(0), rnea_partial_dv_cols[4].at_vec(0), rnea_partial_dv_cols[5].at_vec(0)],
        [rnea_partial_dv_cols[0].at_vec(1), rnea_partial_dv_cols[1].at_vec(1), rnea_partial_dv_cols[2].at_vec(1), rnea_partial_dv_cols[3].at_vec(1), rnea_partial_dv_cols[4].at_vec(1), rnea_partial_dv_cols[5].at_vec(1)],
        [rnea_partial_dv_cols[0].at_vec(2), rnea_partial_dv_cols[1].at_vec(2), rnea_partial_dv_cols[2].at_vec(2), rnea_partial_dv_cols[3].at_vec(2), rnea_partial_dv_cols[4].at_vec(2), rnea_partial_dv_cols[5].at_vec(2)],
        [rnea_partial_dv_cols[0].at_vec(3), rnea_partial_dv_cols[1].at_vec(3), rnea_partial_dv_cols[2].at_vec(3), rnea_partial_dv_cols[3].at_vec(3), rnea_partial_dv_cols[4].at_vec(3), rnea_partial_dv_cols[5].at_vec(3)],
        [rnea_partial_dv_cols[0].at_vec(4), rnea_partial_dv_cols[1].at_vec(4), rnea_partial_dv_cols[2].at_vec(4), rnea_partial_dv_cols[3].at_vec(4), rnea_partial_dv_cols[4].at_vec(4), rnea_partial_dv_cols[5].at_vec(4)],
        [rnea_partial_dv_cols[0].at_vec(5), rnea_partial_dv_cols[1].at_vec(5), rnea_partial_dv_cols[2].at_vec(5), rnea_partial_dv_cols[3].at_vec(5), rnea_partial_dv_cols[4].at_vec(5), rnea_partial_dv_cols[5].at_vec(5)]
    ).define("rnea_partial_dv_final");

    let rnea_partial_dq: ASTNode = Matrix!(
        [rnea_partial_dq_cols[0].at_vec(0), rnea_partial_dq_cols[1].at_vec(0), rnea_partial_dq_cols[2].at_vec(0), rnea_partial_dq_cols[3].at_vec(0), rnea_partial_dq_cols[4].at_vec(0), rnea_partial_dq_cols[5].at_vec(0)],
        [rnea_partial_dq_cols[0].at_vec(1), rnea_partial_dq_cols[1].at_vec(1), rnea_partial_dq_cols[2].at_vec(1), rnea_partial_dq_cols[3].at_vec(1), rnea_partial_dq_cols[4].at_vec(1), rnea_partial_dq_cols[5].at_vec(1)],
        [rnea_partial_dq_cols[0].at_vec(2), rnea_partial_dq_cols[1].at_vec(2), rnea_partial_dq_cols[2].at_vec(2), rnea_partial_dq_cols[3].at_vec(2), rnea_partial_dq_cols[4].at_vec(2), rnea_partial_dq_cols[5].at_vec(2)],
        [rnea_partial_dq_cols[0].at_vec(3), rnea_partial_dq_cols[1].at_vec(3), rnea_partial_dq_cols[2].at_vec(3), rnea_partial_dq_cols[3].at_vec(3), rnea_partial_dq_cols[4].at_vec(3), rnea_partial_dq_cols[5].at_vec(3)],
        [rnea_partial_dq_cols[0].at_vec(4), rnea_partial_dq_cols[1].at_vec(4), rnea_partial_dq_cols[2].at_vec(4), rnea_partial_dq_cols[3].at_vec(4), rnea_partial_dq_cols[4].at_vec(4), rnea_partial_dq_cols[5].at_vec(4)],
        [rnea_partial_dq_cols[0].at_vec(5), rnea_partial_dq_cols[1].at_vec(5), rnea_partial_dq_cols[2].at_vec(5), rnea_partial_dq_cols[3].at_vec(5), rnea_partial_dq_cols[4].at_vec(5), rnea_partial_dq_cols[5].at_vec(5)]
    ).define("rnea_partial_dq_final");

    let dAdq_final: ASTNode = Matrix!(
        [dAdq_cols[0].at_vec(0), dAdq_cols[1].at_vec(0), dAdq_cols[2].at_vec(0), dAdq_cols[3].at_vec(0), dAdq_cols[4].at_vec(0), dAdq_cols[5].at_vec(0)],
        [dAdq_cols[0].at_vec(1), dAdq_cols[1].at_vec(1), dAdq_cols[2].at_vec(1), dAdq_cols[3].at_vec(1), dAdq_cols[4].at_vec(1), dAdq_cols[5].at_vec(1)],
        [dAdq_cols[0].at_vec(2), dAdq_cols[1].at_vec(2), dAdq_cols[2].at_vec(2), dAdq_cols[3].at_vec(2), dAdq_cols[4].at_vec(2), dAdq_cols[5].at_vec(2)],
        [dAdq_cols[0].at_vec(3), dAdq_cols[1].at_vec(3), dAdq_cols[2].at_vec(3), dAdq_cols[3].at_vec(3), dAdq_cols[4].at_vec(3), dAdq_cols[5].at_vec(3)],
        [dAdq_cols[0].at_vec(4), dAdq_cols[1].at_vec(4), dAdq_cols[2].at_vec(4), dAdq_cols[3].at_vec(4), dAdq_cols[4].at_vec(4), dAdq_cols[5].at_vec(4)],
        [dAdq_cols[0].at_vec(5), dAdq_cols[1].at_vec(5), dAdq_cols[2].at_vec(5), dAdq_cols[3].at_vec(5), dAdq_cols[4].at_vec(5), dAdq_cols[5].at_vec(5)]
    ).define("dAdq_final");


    

}

