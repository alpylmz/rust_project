use crate::ast::ASTNode;
use crate::{Scalar, Vector, Matrix};
use crate::rnea::rnea;
use crate::ASTNode::Scalar;


// this T is the number of timesteps, set in python script
const T_: i32 = 5;


// this function is in include/crocoddyl/core/optctrl/shooting.hxx
// The only thing it does is it calls running_models_[i].calc()
// then terminal_model_.calc()
// then cost += running_models_[i].cost
// then cost += terminal_model_.cost
// In our system here is the cost definitions:
////runningCostModel.addCost("gripperPose", goalTrackingCost, 1)
////terminalCostModel.addCost("gripperPose", goalTrackingCost, 1e3)
fn problem_calc(xs_sin: ASTNode, xs_cos: ASTNode, xs: ASTNode, us: ASTNode, xgoal: ASTNode) {
    
    // create all costs vector
    let mut all_costs = Vec::new();
    for i in 0..T_ {
        // the running model is the goal tracking cost
        // the call to calc for running model calls void IntegratedActionModelEulerTpl<Scalar>::calc(
        // the first important line in that function is:
        // control_->calc(d->control, Scalar(0.), u); // d->control->w = u;
        // so it sets the control to u, skipping
    
        // the next important line is:
        // differential_->calc(d->differential, x, d->control->w);, d->control->w is u
        // it calls void DifferentialActionModelFreeInvDynamicsTpl<Scalar>::calc(
        // firstly, run rnea
        // xs has 12 elements, first 6 is pos, next 6 is vel
        let v = Vector!(
            xs.clone().at_mat(i.try_into().unwrap(), 6),
            xs.clone().at_mat(i.try_into().unwrap(), 7),
            xs.clone().at_mat(i.try_into().unwrap(), 8),
            xs.clone().at_mat(i.try_into().unwrap(), 9),
            xs.clone().at_mat(i.try_into().unwrap(), 10),
            xs.clone().at_mat(i.try_into().unwrap(), 11)
        ).define("v");
        let x_sin = Vector!(
            xs_sin.clone().at_mat(i.try_into().unwrap(), 0),
            xs_sin.clone().at_mat(i.try_into().unwrap(), 1),
            xs_sin.clone().at_mat(i.try_into().unwrap(), 2),
            xs_sin.clone().at_mat(i.try_into().unwrap(), 3),
            xs_sin.clone().at_mat(i.try_into().unwrap(), 4),
            xs_sin.clone().at_mat(i.try_into().unwrap(), 5)
        ).define("x_sin");
        let x_cos = Vector!(
            xs_cos.clone().at_mat(i.try_into().unwrap(), 0),
            xs_cos.clone().at_mat(i.try_into().unwrap(), 1),
            xs_cos.clone().at_mat(i.try_into().unwrap(), 2),
            xs_cos.clone().at_mat(i.try_into().unwrap(), 3),
            xs_cos.clone().at_mat(i.try_into().unwrap(), 4),
            xs_cos.clone().at_mat(i.try_into().unwrap(), 5)
        ).define("x_cos");
        let u = Vector!(
            us.clone().at_mat(i.try_into().unwrap(), 0),
            us.clone().at_mat(i.try_into().unwrap(), 1),
            us.clone().at_mat(i.try_into().unwrap(), 2),
            us.clone().at_mat(i.try_into().unwrap(), 3),
            us.clone().at_mat(i.try_into().unwrap(), 4),
            us.clone().at_mat(i.try_into().unwrap(), 5)
        ).define("u");
        let x = Vector!(
            xs.clone().at_mat(i.try_into().unwrap(), 0),
            xs.clone().at_mat(i.try_into().unwrap(), 1),
            xs.clone().at_mat(i.try_into().unwrap(), 2),
            xs.clone().at_mat(i.try_into().unwrap(), 3),
            xs.clone().at_mat(i.try_into().unwrap(), 4),
            xs.clone().at_mat(i.try_into().unwrap(), 5)
        ).define("x");
        let taus = rnea(x_sin, x_cos, v, u);

        // then it calls pinocchio::updateGlobalPlacements(pinocchio_, d->pinocchio);
        // but it only updates the position in d->pinocchio, and I removed it and did not observe
        // any problems *for now*, so skipping.
        
        // NEXT LINES: 
        /////actuation_->commands(d->multibody.actuation, x, d->pinocchio.tau); // d->multibody.actuation->u = d->pinocchio.tau;
        /////d->multibody.joint->a = u;
        /////d->multibody.joint->tau = d->multibody.actuation->u;
        /////actuation_->calc(d->multibody.actuation, x, d->multibody.joint->tau); // d->multibody.actuation->tau = d->multibody.joint->tau;
        // I will skip the above lines since they are merely assigning values
        // If I will have business with multibody data, I need to check here
        

        /////costs_->calc(d->costs, x, u);
        /////d->cost = d->costs->cost;
        /////d->constraints->resize(this, d);
        /////constraints_->calc(d->constraints, x, u);

        // costs_calc is in void ResidualModelFramePlacementTpl<Scalar>::calc(
        // it is way too complicated, I believe I can just get the squared distance between current and goal
        // and return it as cost
        let cost1_vec = Vector!(
            x.clone().at_vec(0) - xgoal.clone().at_vec(0),
            x.clone().at_vec(1) - xgoal.clone().at_vec(1),
            x.clone().at_vec(2) - xgoal.clone().at_vec(2),
            x.clone().at_vec(3) - xgoal.clone().at_vec(3),
            x.clone().at_vec(4) - xgoal.clone().at_vec(4),
            x.clone().at_vec(5) - xgoal.clone().at_vec(5)
        ).define("cost1_vec");
        let cost1_vec2 = Vector!(
            cost1_vec.clone().at_vec(0) * cost1_vec.clone().at_vec(0),
            cost1_vec.clone().at_vec(1) * cost1_vec.clone().at_vec(1),
            cost1_vec.clone().at_vec(2) * cost1_vec.clone().at_vec(2),
            cost1_vec.clone().at_vec(3) * cost1_vec.clone().at_vec(3),
            cost1_vec.clone().at_vec(4) * cost1_vec.clone().at_vec(4),
            cost1_vec.clone().at_vec(5) * cost1_vec.clone().at_vec(5)
        ).define("cost1_vec2");

        // TODO: No define, this may cause problems
        let cost1 = cost1_vec2.clone().at_vec(0) + cost1_vec2.clone().at_vec(1) + cost1_vec2.clone().at_vec(2) + cost1_vec2.clone().at_vec(3) + cost1_vec2.clone().at_vec(4) + cost1_vec2.clone().at_vec(5)
        all_costs.push(cost1);
    }

    // TODO: we may need terminal cost somewhere here
    





}


// exists in SolverDDP::calcDiff
fn calcDiff(xs_sin: ASTNode, xs_cos: ASTNode, xs: ASTNode, us: ASTNode, xgoal: ASTNode) {
    // iter_ is always 0, so I will assume the same here
    // But it may change!
    //problem_->calc(xs_, us_);
    problem_calc(xs_sin, xs_cos, xs, us, xgoal);

    // TODO: cost_ = problem_->calcDiff(xs_, us_);


    // TODO: hfeas_ = computeEqualityFeasibility();

}

// u0 is assumed to be 0 in the beginning
pub fn solver(
    x0: ASTNode,
    xs_sin: ASTNode, // assume sin and cos operations will be done in another place
    xs_cos: ASTNode,
    us: ASTNode,
    xgoal: ASTNode,
) {

    // the first lines, setCandidate and the first loop, just sets all xs_ elements to x0,
    // and in the beginning u0 is always 0, so I will just assume the same here
    // But changing it should be fairly easy.
    // size of x0 is assumed to be 6
    // I don't know how to make it so that it would work for variable sizes(for now). 
    // It is mainly a programming issue
    // xs_ contains [state, control] pairs, both state and control has n dof dimensions
    let xs_ = Matrix!(
        [x0.clone().at_vec(0), x0.clone().at_vec(1), x0.clone().at_vec(2), x0.clone().at_vec(3), x0.clone().at_vec(4), x0.clone().at_vec(5), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [x0.clone().at_vec(0), x0.clone().at_vec(1), x0.clone().at_vec(2), x0.clone().at_vec(3), x0.clone().at_vec(4), x0.clone().at_vec(5), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [x0.clone().at_vec(0), x0.clone().at_vec(1), x0.clone().at_vec(2), x0.clone().at_vec(3), x0.clone().at_vec(4), x0.clone().at_vec(5), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [x0.clone().at_vec(0), x0.clone().at_vec(1), x0.clone().at_vec(2), x0.clone().at_vec(3), x0.clone().at_vec(4), x0.clone().at_vec(5), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [x0.clone().at_vec(0), x0.clone().at_vec(1), x0.clone().at_vec(2), x0.clone().at_vec(3), x0.clone().at_vec(4), x0.clone().at_vec(5), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [x0.clone().at_vec(0), x0.clone().at_vec(1), x0.clone().at_vec(2), x0.clone().at_vec(3), x0.clone().at_vec(4), x0.clone().at_vec(5), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ).define("xs_");

    // Regularization parameters, don't know how to use them yet
    let preg_ = Scalar!(0.0).define("preg_");
    let dreg_ = Scalar!(23498237.0).define("dreg_");

    // next lines are checking if zero_upsilon is true or not
    // it is always false, but then upsilon_ is set to 0 anyways
    let upsilon_ = Scalar!(0.0).define("upsilon_");

    // to escape recalcDiff checks, I will assume recalcDiff is always true
    // though this will increase the runtime, and it will make things more complicated in benchmark
    

    // new_maxiter is also set to 1, and at least for some moves it works
    // I guess if we use this over and over on small steps, it will work
    // but even if it won't, I will still test it in new_maxiter = 1 and see


    // Start computeDirection(called from intro.cpp, defined in ddp.cpp)
    /*
    void SolverDDP::computeDirection(const bool recalcDiff) {
        START_PROFILER("SolverDDP::computeDirection");
        if (recalcDiff) {
        // I assume recalcDiff is always true
        std::cout << "recalcDiff is true" << std::endl;
        calcDiff();
        }
        else{
        std::cout << "only backwardPass" << std::endl;
        }
        backwardPass();
        STOP_PROFILER("SolverDDP::computeDirection");
    }
    */
    
    // recalcDiff is assumed to be true always, so we are calling calcDiff()
    calcDiff(xs_sin, xs_cos, xs_, us, xgoal);





}



