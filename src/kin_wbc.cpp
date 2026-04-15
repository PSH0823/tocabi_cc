#include "kin_wbc.h"

using namespace TOCABI;

KinWBC::KinWBC(RobotData& rd, CbfManager& cbf_mgr) : rd_(rd), cbf_mgr_(cbf_mgr) { }

void KinWBC::setTaskHierarchy(const TaskMotionType& motion_mode_)
{
    motion_mode = motion_mode_;

    if(motion_mode == TaskMotionType::Walking)
    {
        task_hierarchy = {
            {{COM_id, TaskType::Position}, {Pelvis, TaskType::Orientation}},
            {{Left_Foot, TaskType::Position}, {Left_Foot, TaskType::Orientation}, {Right_Foot, TaskType::Position}, {Right_Foot, TaskType::Orientation}}
        };
    }
    else
    {
        task_hierarchy = {
            {{COM_id, TaskType::Position}, {Pelvis, TaskType::Orientation}},
            {{Left_Foot, TaskType::Position}, {Left_Foot, TaskType::Orientation}, {Right_Foot, TaskType::Position}, {Right_Foot, TaskType::Orientation}},
            {{Upper_Body, TaskType::Orientation}},
            {{Head, TaskType::Orientation}},
            {{Left_Hand, TaskType::Position}, {Left_Hand, TaskType::Orientation}, {Right_Hand, TaskType::Position}, {Right_Hand, TaskType::Orientation}}};
    }
}

void KinWBC::computeTaskSpaceKinematicWBC()
{
    //--- Initialization
    qdot_des.setZero();
    Eigen::MatrixVVd Ni = Eigen::MatrixVVd::Identity();

    //--- Nullspace-based Prioritized Task Execution
    for (const auto& task_group : task_hierarchy)
    {
        int m = 3 * task_group.size();
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(m, MODEL_DOF_VIRTUAL);
        Eigen::VectorXd de = Eigen::VectorXd::Zero(m);
     
        for (size_t i = 0; i < task_group.size(); ++i)
        {
            const auto& [idx, type] = task_group[i];

            if (type == TaskType::Position)
            {
                J.block(3 * i, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[idx].local_Jac_v;
                Eigen::Vector3d pos_err = rd_.link_[idx].x_traj - rd_.link_[idx].local_xpos;

                de.segment<3>(3 * i) = pos_err;
            }
            else if (type == TaskType::Orientation)
            {
                J.block(3 * i, 0, 3, MODEL_DOF_VIRTUAL) = rd_.link_[idx].local_Jac_w;
                Eigen::Vector3d ori_err = -DyrosMath::getPhi(rd_.link_[idx].local_rotm, rd_.link_[idx].r_traj);

                de.segment<3>(3 * i) = ori_err;
            }
            else
            {
                ROS_ERROR("Unknown TaskType");
                assert(type == TaskType::Position || type == TaskType::Orientation);
            }
        }

        Eigen::MatrixXd J_pre = J * Ni;
        Eigen::MatrixXd J_pinv = DyrosMath::pinv_SVD(J_pre);

        qdot_des += J_pinv * (de - J * qdot_des);
        Ni *= (Eigen::MatrixVVd::Identity() - J_pinv * J_pre);
    }

    if(motion_mode == TaskMotionType::Walking)
    {
        const int UPPERBODY_DOF = MODEL_DOF - 12;
        Eigen::MatrixXd J; J.setZero(UPPERBODY_DOF, MODEL_DOF_VIRTUAL);
        J.rightCols(UPPERBODY_DOF).setIdentity();

        Eigen::MatrixXd J_pre = J * Ni;
        Eigen::MatrixXd J_pinv = DyrosMath::pinv_SVD(J_pre);

        qdot_des += J_pinv * ((q_init_des.tail(UPPERBODY_DOF) - rd_.q_.tail(UPPERBODY_DOF)) - J * qdot_des);
    }

    if(cbf_mgr_.getCbfMode() == CbfType::Kin){
        qdot_des = safetyFilter();
    }

    rd_.q_desired_virtual = integrate(rd_.local_q_virtual_, qdot_des);
    rd_.q_desired = rd_.q_desired_virtual.segment(6, MODEL_DOF);

    rd_.q_ddot_desired_virtual = computeDesiredJointAcceleration(rd_.local_q_virtual_, rd_.local_q_dot_virtual_, rd_.q_desired_virtual, rd_.Kp_virtual_diag, rd_.Kd_virtual_diag);
}

Eigen::VectorQVQd KinWBC::integrate(const Eigen::VectorQVQd &q_current, const Eigen::VectorVQd &q_delta)
{
    Eigen::VectorQVQd q_integrated_; q_integrated_.setZero();

    //--- Floating-base Integration (Base position)
    q_integrated_.segment(0, 3) = q_current.segment(0, 3) + q_delta.segment(0,3); 

    //--- Floating-base Integration (Base orientation)
    Eigen::Quaterniond current_quat(q_current(39), q_current(3), q_current(4), q_current(5));
    Eigen::Quaterniond desired_quat;
    desired_quat = integrateQuatBodyExp(current_quat, q_delta.segment(3, 3), 1.0);
    q_integrated_(3)  = desired_quat.x();   
    q_integrated_(4)  = desired_quat.y();   
    q_integrated_(5)  = desired_quat.z();   
    q_integrated_(39) = desired_quat.w();   

    //--- Floating-base Integration (Base actuated joints)
    q_integrated_.segment(6, MODEL_DOF) = q_current.segment(6, MODEL_DOF) + q_delta.segment(6, MODEL_DOF);

    return(q_integrated_);
}

Eigen::VectorVQd KinWBC::computeDesiredJointAcceleration(const Eigen::VectorQVQd &q_current, const Eigen::VectorVQd &qdot_current, const Eigen::VectorQVQd &q_desired, const Eigen::MatrixVVd &Kp, const Eigen::MatrixVVd &Kd)
{
    Eigen::VectorVQd qddot_integrated_; qddot_integrated_.setZero();

    //--- Joint Acceleration Command (Base position)
    qddot_integrated_.segment(0, 3) = Kp.block(0, 0, 3, 3) * (q_desired.segment(0, 3) - q_current.segment(0, 3)) 
                                    + Kd.block(0, 0, 3, 3) * (Eigen::Vector3d::Zero() - qdot_current.segment(0, 3));

    //--- Joint Acceleration Command (Base orientation)
    Eigen::Matrix3d current_rotation = Eigen::Quaterniond(q_current(39), q_current(3), q_current(4), q_current(5)).toRotationMatrix();
    Eigen::Matrix3d desired_rotation = Eigen::Quaterniond(q_desired(39), q_desired(3), q_desired(4), q_desired(5)).toRotationMatrix();
    qddot_integrated_.segment(3, 3) = Kp.block(3, 3, 3, 3) * (-DyrosMath::getPhi(current_rotation, desired_rotation)) 
                                    + Kd.block(3, 3, 3, 3) * (Eigen::Vector3d::Zero() - qdot_current.segment(3, 3));

    //--- Joint Acceleration Command (Base actuated joints)
    qddot_integrated_.segment(6, MODEL_DOF) = Kp.block(6, 6, MODEL_DOF, MODEL_DOF) * (q_desired.segment(6, MODEL_DOF) - q_current.segment(6, MODEL_DOF)) 
                                            + Kd.block(6, 6, MODEL_DOF, MODEL_DOF) * (Eigen::VectorQd::Zero() - qdot_current.segment(6, MODEL_DOF));

    return (qddot_integrated_);
}

void KinWBC::setInitialConfiguration(const Eigen::VectorQd &q_init_des_)
{
    q_init_des.setZero();
    q_init_des = q_init_des_;
}

Eigen::VectorVQd KinWBC::safetyFilter()
{
    constraints_.clear();
    calcCostHess();
    calcCostGrad();
    calcEqualityConstraint();
    calcInequalityConstraint();

    total_num_state = constraints_.empty() ? 0 : constraints_[0].A.cols();

    int total_num_constraints_current = 0;
    for (const auto& c : constraints_) {total_num_constraints_current += c.A.rows();}

    if (total_num_constraints != total_num_constraints_current){
        num_constraints_changed = true;
        total_num_constraints = total_num_constraints_current;
    }

    if(num_constraints_changed)
    {
        //--- Initialization or reinitialization of QP
        QP_safety_filter.InitializeProblemSize(total_num_state, total_num_constraints);

        A_const   = Eigen::MatrixXd::Zero(total_num_constraints, total_num_state);
        lbA_const = Eigen::VectorXd::Zero(total_num_constraints);
        ubA_const = Eigen::VectorXd::Zero(total_num_constraints);

        qdot_safety.setZero();

        num_constraints_changed = false;
    }

    //--- Stack Constraints
    int row_idx  = 0;
    for (const auto& c : constraints_) {
        int rows = c.A.rows();
        A_const.block(row_idx, 0, rows, total_num_state) = c.A;
        lbA_const.segment(row_idx , rows)    = c.lbA;
        ubA_const.segment(row_idx , rows)    = c.ubA;
        row_idx  += rows;
    }

    checkGradHessSize();

    QP_safety_filter.EnableEqualityCondition(1e-8);
    QP_safety_filter.UpdateMinProblem(Hess, grad);
    QP_safety_filter.DeleteSubjectToAx();
    QP_safety_filter.UpdateSubjectToAx(A_const, lbA_const, ubA_const);

    bool qp_status = true;
    Eigen::VectorXd X_; X_.setZero(total_num_state);
    if(QP_safety_filter.SolveQPoases(500, X_, true))
    {
        qdot_safety = X_.segment(0, MODEL_DOF_VIRTUAL);
        qp_status = true;
    }
    else
    {
        //--- CONSTRAINTS VIOLATION CHECKER
        if(is_cannot_solve_qp_ == true)   
        {
            Eigen::VectorXd Ax = A_const * X_; 

            for (int i = 0; i < A_const.rows(); ++i)
            {
                double val = Ax(i);
                double l = lbA_const(i);
                double u = ubA_const(i);

                double eps = 1e-5;

                if (val < l - eps)
                {
                    std::cerr << "[Constraint Violation] Row " << i << ": " << val << " < lbA = " << l << std::endl;
                }
                else if (val > u + eps)
                {
                    std::cerr << "[Constraint Violation] Row " << i << ": " << val << " > ubA = " << u << std::endl;
                }
            }
            is_cannot_solve_qp_ = false;
        }

        qdot_safety.setZero();
        std::cout << "Kin WBC SolveQPoases ERROR: Unable to find a valid solution." << std::endl;
        qp_status = false;
    }

    return (qdot_safety);
}

void KinWBC::calcCostHess()
{
    Hess.setIdentity(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
}

void KinWBC::calcCostGrad()
{
    grad.setZero(MODEL_DOF_VIRTUAL);
    grad = (-1.0) * Hess * qdot_des;
}

void KinWBC::calcEqualityConstraint()
{
}

void KinWBC::calcInequalityConstraint()
{
    //--- (1) Joint Limit constraints
    Eigen::MatrixXd A_qpos; A_qpos.setZero(MODEL_DOF, MODEL_DOF_VIRTUAL);
    Eigen::VectorQd lbA_qpos; lbA_qpos.setZero(MODEL_DOF); 
    Eigen::VectorQd ubA_qpos; ubA_qpos.setZero(MODEL_DOF);
    
    cbf_mgr_.getJointLimitCbfConstraint(A_qpos.block(0, 6, MODEL_DOF, MODEL_DOF), lbA_qpos, ubA_qpos);

    constraints_.push_back({A_qpos, lbA_qpos, ubA_qpos}); 

    //--- (2) Workspace Boundary constraints
    const int num_workspace_cbf = cbf_mgr_.getNumWorkspaceBoundaryPairs();
    Eigen::MatrixXd A_workspace; A_workspace.setZero(num_workspace_cbf, MODEL_DOF_VIRTUAL);
    Eigen::VectorXd lbA_workspace; lbA_workspace.setZero(num_workspace_cbf); 
    Eigen::VectorXd ubA_workspace; ubA_workspace.setZero(num_workspace_cbf);
    
    cbf_mgr_.getWorkspaceBoundaryCbfConstraint(A_workspace, lbA_workspace, ubA_workspace);

    constraints_.push_back({A_workspace, lbA_workspace, ubA_workspace});

    //--- (3) Self-collision avoidance constraints
    const int num_self_collision_cbf = cbf_mgr_.getNumSelfCollisionPairs();
    Eigen::MatrixXd A_self_collision; A_self_collision.setZero(num_self_collision_cbf, MODEL_DOF_VIRTUAL);
    Eigen::VectorXd lbA_self_collision; lbA_self_collision.setZero(num_self_collision_cbf); 
    Eigen::VectorXd ubA_self_collision; ubA_self_collision.setZero(num_self_collision_cbf);

    cbf_mgr_.getSelfCollisionCbfConstraint(A_self_collision, lbA_self_collision, ubA_self_collision);

    constraints_.push_back({A_self_collision, lbA_self_collision, ubA_self_collision});

    //--- (4) Obstacle avoidance constraints
    const int num_obstacle_avoidance_cbf = cbf_mgr_.getNumObstacleAvoidancePairs();
    Eigen::MatrixXd A_obstacle_avoidance; A_obstacle_avoidance.setZero(num_obstacle_avoidance_cbf, MODEL_DOF_VIRTUAL);
    Eigen::VectorXd lbA_obstacle_avoidance; lbA_obstacle_avoidance.setZero(num_obstacle_avoidance_cbf); 
    Eigen::VectorXd ubA_obstacle_avoidance; ubA_obstacle_avoidance.setZero(num_obstacle_avoidance_cbf);

    cbf_mgr_.getObstacleAvoidanceCbfConstraint(A_obstacle_avoidance, lbA_obstacle_avoidance, ubA_obstacle_avoidance);
    
    constraints_.push_back({A_obstacle_avoidance, lbA_obstacle_avoidance, ubA_obstacle_avoidance});
} 

void KinWBC::checkGradHessSize()
{
    static bool is_gradhess_init_ = true;
    if(is_gradhess_init_ == true)
    {
        std::cout << "==============================================" << std::endl;
        std::cout << "===== KinWBC COST & CONSTRAINTS DIM INFO =====" << std::endl;
        std::cout << "==============================================" << std::endl;

        std::cout << "total_num_state: " << total_num_state << std::endl;
        std::cout << "total_num_constraints: " << total_num_constraints << std::endl;
        std::cout << std::endl;

        std::cout << "Hess size: " << Hess.rows() << " x " << Hess.cols() << std::endl;
        std::cout << "grad size: " << grad.size() << std::endl;
        std::cout << std::endl;

        std::cout << "A: " << A_const.rows() << " x " << A_const.cols() << std::endl;
        std::cout << "lbA size: " << lbA_const.size() << std::endl;
        std::cout << "ubA size: " << ubA_const.size() << std::endl;
        std::cout << std::endl;

        is_gradhess_init_ = false;
    }
}