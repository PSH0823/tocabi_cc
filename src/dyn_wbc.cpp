// dyn_wbc.cpp
#include "dyn_wbc.h"
#include <iostream>

using namespace Eigen;
using namespace qpOASES;

DynWBC::DynWBC(RobotData& rd, CbfManager& cbf_mgr) : rd_(rd), cbf_mgr_(cbf_mgr) { }

Eigen::VectorQd DynWBC::computeDynamicWBC()
{
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

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
        QP_Dyn_Wbc.InitializeProblemSize(total_num_state, total_num_constraints);

        A_const   = Eigen::MatrixXd::Zero(total_num_constraints, total_num_state);
        lbA_const = Eigen::VectorXd::Zero(total_num_constraints);
        ubA_const = Eigen::VectorXd::Zero(total_num_constraints);

        torque_prev.setZero();

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

    QP_Dyn_Wbc.EnableEqualityCondition(1e-8);
    QP_Dyn_Wbc.UpdateMinProblem(Hess, grad);
    QP_Dyn_Wbc.DeleteSubjectToAx();
    QP_Dyn_Wbc.UpdateSubjectToAx(A_const, lbA_const, ubA_const);

    bool qp_status = true;
    Eigen::VectorXd X_; X_.setZero(total_num_state);
    if(QP_Dyn_Wbc.SolveQPoases(2000, X_, true))
    {
        contact_wrench_qp  = X_.segment(0, contact_dim);
        qddot_qp           = X_.segment(contact_dim, MODEL_DOF_VIRTUAL);
        torque_qp          = X_.segment(contact_dim + MODEL_DOF_VIRTUAL, MODEL_DOF);
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

                double eps = 0.0;

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

        std::cout << "Dyn WBC SolveQPoases ERROR: Unable to find a valid solution." << std::endl;
        qp_status = false;
    }

    torque_prev = torque_qp;

    return(torque_qp);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
}

void DynWBC::updateRobotStates(const Eigen::MatrixVVd &M_, const Eigen::VectorVQd &G_, const Eigen::MatrixXd &J_C_)
{
    //--- Robot States
    M = M_;
    G = G_;
    J_C.setZero(12, MODEL_DOF_VIRTUAL);
    J_C = J_C_;
    J_C_T.setZero(MODEL_DOF_VIRTUAL, 12); 
    J_C_T = J_C.transpose();

    Sa_T.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF); Sa_T.bottomRows(MODEL_DOF).setIdentity();
    Sa.setZero(MODEL_DOF, MODEL_DOF_VIRTUAL);   Sa = Sa_T.transpose();
    Sf.setZero(6, MODEL_DOF_VIRTUAL);    Sf.leftCols(6).setIdentity();

    //--- Friction cone constraints (https://scaron.info/robotics/wrench-friction-cones.html)
    Eigen::MatrixXd U_fric_dsp; U_fric_dsp.setZero(34, 12);
    Eigen::MatrixXd U_fric_ssp; U_fric_ssp.setZero(17, 6);
    double X = foot_size  / 2.0; 
    double Y = foot_width / 2.0; 
    U_fric_ssp <<  0,  0,            -1,   0,   0,  0,
                  -1,  0,           -mu,   0,   0,  0,
                  +1,  0,           -mu,   0,   0,  0,
                   0, -1,           -mu,   0,   0,  0,
                   0, +1,           -mu,   0,   0,  0,
                   0,  0,            -Y,  -1,   0,  0,
                   0,  0,            -Y,  +1,   0,  0,
                   0,  0,            -X,   0,  -1,  0,
                   0,  0,            -X,   0,  +1,  0,
                  -Y, -X, -(X + Y) * mu, -mu, +mu, -1,
                  +Y, +X, -(X + Y) * mu, +mu, -mu, -1,
                  +Y, -X, -(X + Y) * mu, +mu, +mu, -1,
                  +Y, +X, -(X + Y) * mu, +mu, +mu, -1,
                  +Y, -X, -(X + Y) * mu, +mu, +mu, +1,
                  +Y, +X, -(X + Y) * mu, +mu, -mu, +1,
                  -Y, -X, -(X + Y) * mu, -mu, -mu, +1,
                  -Y, +X, -(X + Y) * mu, -mu, +mu, +1;
    U_fric_dsp.topLeftCorner(17, 6) = U_fric_ssp;
    U_fric_dsp.bottomRightCorner(17, 6) = U_fric_ssp;

    A_fric.setZero(34, contact_dim + MODEL_DOF_VIRTUAL + MODEL_DOF);
    lbA_fric.setZero(34);
    ubA_fric.setZero(34);

    A_fric.leftCols(contact_dim) = U_fric_dsp;
}


void DynWBC::updateControlCommands(const Eigen::Vector12d& contact_wrench_cmd_, const Eigen::VectorVQd &qddot_cmd_)
{
    contact_wrench_cmd = contact_wrench_cmd_;
    qddot_cmd = qddot_cmd_;
}

/////////////////////////////////////////////
//--- Quadratic Programming Formulation ---//
/////////////////////////////////////////////
void DynWBC::calcCostHess()
{
    Hess.setZero(contact_dim + MODEL_DOF_VIRTUAL + MODEL_DOF, contact_dim + MODEL_DOF_VIRTUAL + MODEL_DOF);
    
    unsigned int start_idx = 0;
    Hess.block(start_idx, start_idx, contact_dim, contact_dim) = W_cwr * Eigen::MatrixXd::Identity(contact_dim, contact_dim);
    start_idx += contact_dim; 
    
    Hess.block(start_idx, start_idx, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) = W_qddot * Eigen::MatrixVVd::Identity() + W_energy * M;
    start_idx += MODEL_DOF_VIRTUAL; 

    Hess.block(start_idx, start_idx, MODEL_DOF, MODEL_DOF) = W_torque * Eigen::MatrixQQd::Identity();
    start_idx += MODEL_DOF; 
}

void DynWBC::calcCostGrad()
{
    grad.setZero(contact_dim + MODEL_DOF_VIRTUAL + MODEL_DOF);
    unsigned int start_idx = 0;

    grad.segment(start_idx, contact_dim) -= W_cwr * contact_wrench_cmd;
    start_idx += contact_dim; 

    grad.segment(start_idx, MODEL_DOF_VIRTUAL) -= W_qddot * qddot_cmd;
    start_idx += MODEL_DOF_VIRTUAL; 

    grad.segment(start_idx, MODEL_DOF) -= W_torque * torque_prev;
    start_idx += MODEL_DOF; 
}

void DynWBC::calcEqualityConstraint()
{
    //--- (1) Floating base dynamics
    Eigen::MatrixXd A_wb; A_wb.setZero(MODEL_DOF_VIRTUAL, contact_dim + MODEL_DOF_VIRTUAL + MODEL_DOF);
    Eigen::VectorXd lbA_wb; lbA_wb.setZero(MODEL_DOF_VIRTUAL);
    Eigen::VectorXd ubA_wb; ubA_wb.setZero(MODEL_DOF_VIRTUAL);

    unsigned int start_idx = 0;
    A_wb.block(0, start_idx, MODEL_DOF_VIRTUAL, contact_dim) = J_C_T;
    start_idx += contact_dim;
    A_wb.block(0, start_idx, MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL) = (-1.0) * M;
    start_idx += MODEL_DOF_VIRTUAL;
    A_wb.block(0, start_idx, MODEL_DOF_VIRTUAL, MODEL_DOF) = Sa_T;
    start_idx += MODEL_DOF; 

    lbA_wb = G;
    ubA_wb = G;
    constraints_.push_back({A_wb, lbA_wb, ubA_wb});
}

void DynWBC::calcInequalityConstraint()
{
    //--- (2) Friction cone constraints
    constraints_.push_back({   
        A_fric,
        Eigen::VectorXd::Constant(A_fric.rows(), -std::numeric_limits<double>::infinity()),
        ubA_fric
    });

    if(cbf_mgr_.getCbfMode() == CbfType::Dyn)
    {
        //--- (3) Joint Limit constraints
        Eigen::MatrixXd A_qpos; A_qpos.setZero(MODEL_DOF, contact_dim + MODEL_DOF_VIRTUAL + MODEL_DOF);
        Eigen::VectorQd lbA_qpos; lbA_qpos.setZero(MODEL_DOF); 
        Eigen::VectorQd ubA_qpos; ubA_qpos.setZero(MODEL_DOF);
        
        cbf_mgr_.getJointLimitCbfConstraint(A_qpos.block(0, contact_dim + 6, MODEL_DOF, MODEL_DOF), lbA_qpos, ubA_qpos);

        constraints_.push_back({A_qpos, lbA_qpos, ubA_qpos}); 

        //--- (4) Workspace Boundary constraints
        const int num_workspace_cbf = cbf_mgr_.getNumWorkspaceBoundaryPairs();
        Eigen::MatrixXd A_workspace; A_workspace.setZero(num_workspace_cbf, contact_dim + MODEL_DOF_VIRTUAL + MODEL_DOF);
        Eigen::VectorXd lbA_workspace; lbA_workspace.setZero(num_workspace_cbf); 
        Eigen::VectorXd ubA_workspace; ubA_workspace.setZero(num_workspace_cbf);
        
        cbf_mgr_.getWorkspaceBoundaryCbfConstraint(A_workspace.block(0, contact_dim, num_workspace_cbf, MODEL_DOF_VIRTUAL), lbA_workspace, ubA_workspace);

        constraints_.push_back({A_workspace, lbA_workspace, ubA_workspace});


        //--- (5) Self-collision avoidance constraints
        const int num_self_collision_cbf = cbf_mgr_.getNumSelfCollisionPairs();
        Eigen::MatrixXd A_self_collision; A_self_collision.setZero(num_self_collision_cbf, contact_dim + MODEL_DOF_VIRTUAL + MODEL_DOF);
        Eigen::VectorXd lbA_self_collision; lbA_self_collision.setZero(num_self_collision_cbf); 
        Eigen::VectorXd ubA_self_collision; ubA_self_collision.setZero(num_self_collision_cbf);

        cbf_mgr_.getSelfCollisionCbfConstraint(A_self_collision.block(0, contact_dim, num_self_collision_cbf, MODEL_DOF_VIRTUAL), lbA_self_collision, ubA_self_collision);

        constraints_.push_back({A_self_collision, lbA_self_collision, ubA_self_collision});

        //--- (6) Obstacle avoidance constraints
        const int num_obstacle_avoidance_cbf = cbf_mgr_.getNumObstacleAvoidancePairs();
        Eigen::MatrixXd A_obstacle_avoidance; A_obstacle_avoidance.setZero(num_obstacle_avoidance_cbf, contact_dim + MODEL_DOF_VIRTUAL + MODEL_DOF);
        Eigen::VectorXd lbA_obstacle_avoidance; lbA_obstacle_avoidance.setZero(num_obstacle_avoidance_cbf); 
        Eigen::VectorXd ubA_obstacle_avoidance; ubA_obstacle_avoidance.setZero(num_obstacle_avoidance_cbf);

        cbf_mgr_.getObstacleAvoidanceCbfConstraint(A_obstacle_avoidance.block(0, contact_dim, num_obstacle_avoidance_cbf, MODEL_DOF_VIRTUAL), lbA_obstacle_avoidance, ubA_obstacle_avoidance);

        constraints_.push_back({A_obstacle_avoidance, lbA_obstacle_avoidance, ubA_obstacle_avoidance});
    }
}

void DynWBC::checkGradHessSize()
{
    if(is_gradhess_init_ == true)
    {
        std::cout << "==============================================" << std::endl;
        std::cout << "===== DynWBC COST & CONSTRAINTS DIM INFO =====" << std::endl;
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

//--- cbf
double DynWBC::getSignedDistanceFunction(LinkData &linkA_, LinkData &linkB_, Eigen::MatrixXd &J_AB, Eigen::MatrixXd &Jqdot_AB)
{   
    // Initialization
    double sd_AB = 0.0;

    sd_AB = (linkA_.local_xpos - linkB_.local_xpos).norm(); 

    Eigen::Vector3d normal_vector_btw_AB; normal_vector_btw_AB.setZero();
    normal_vector_btw_AB = (linkA_.local_xpos - linkB_.local_xpos) / (linkA_.local_xpos - linkB_.local_xpos).norm();

    J_AB.setZero(1, MODEL_DOF_VIRTUAL);
    J_AB = normal_vector_btw_AB.transpose() * (linkA_.local_Jac_v - linkB_.local_Jac_v);

    Jqdot_AB.setZero(1, 1);
    Jqdot_AB = normal_vector_btw_AB.transpose() * (linkA_.local_Jdotqdot.head(3) - linkB_.local_Jdotqdot.head(3));

    return sd_AB;
}


//--- Setter
void DynWBC::setFrictionCoefficient(const double& mu_)
{
    //--- Friction, Contact, Torque limit constraints
    mu = mu_;
}

void DynWBC::setFootDimension(const double& foot_size_, const double& foot_width_)
{
    foot_size = foot_size_; 
    foot_width = foot_width_; 
}

void DynWBC::setJointTrackingWeight(const double &W_qddot_)
{
    W_qddot = W_qddot_;
}

void DynWBC::setContactWrenchTrackingWeight(const double &W_cwr_)
{
    W_cwr = W_cwr_;
}

void DynWBC::setTorqueMinimizationWeight(const double &W_torque_)
{
    W_torque = W_torque_;
}

void DynWBC::setAccelEnergyMinimizationWeight(const double &W_energy_)
{
    W_energy = W_energy_;
}
