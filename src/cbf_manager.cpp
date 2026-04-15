#include "cbf_manager/cbf_manager.h"

using namespace TOCABI;

CbfManager::CbfManager(RobotData &rd, RigidBodyDynamics::Model &model) : rd_(rd), model_(model), col_mgr_(rd_, model_) {}
CbfManager::~CbfManager(){}

void CbfManager::callAvailableQueue()
{
    col_mgr_.callAvailableQueue();
}

void CbfManager::setCbfMode(const CbfType& cbf_mode_)
{
    cbf_mode = cbf_mode_;
}

void CbfManager::update()
{
    col_mgr_.pubBasetoHeadTransform();

    col_mgr_.updateObstacle();
    col_mgr_.updateRobotCollisionObjectsPose();

    col_mgr_.computeSelfColAvoidConstraintTerms(cbf_mode);
    col_mgr_.computeObstacleAvoidConstraintTerms(cbf_mode);

    computeJointLimitCbfConstraint();
    computeWorkspaceBoundaryCbfConstraint();
    computeSelfCollisionCbfConstraint();
    computeObstacleAvoidanceCbfConstraint();
}

void CbfManager::pubDataFromSlowToFast()
{
    joint_limit_constraints_container              = joint_limit_constraints;
    workspace_boundary_constraints_container       = workspace_boundary_constraints;
    self_collision_avoidance_constraints_container = self_collision_avoidance_constraints;
    obstacle_avoidance_constraints_container       = obstacle_avoidance_constraints;
}

void CbfManager::subDataFromSlowToFast()
{
    joint_limit_constraints_fast              = joint_limit_constraints_container;
    workspace_boundary_constraints_fast       = workspace_boundary_constraints_container;
    self_collision_avoidance_constraints_fast = self_collision_avoidance_constraints_container;
    obstacle_avoidance_constraints_fast       = obstacle_avoidance_constraints_container;
}

// =========================
// Joint limit CBF
// =========================
void CbfManager::setJointLimitCbfParameters(const double& alpha, const double& epsilon)
{
    alpha_joint_limit = alpha;
    epsilon_joint_limit = epsilon;
}

void CbfManager::setJointLimitBoundaries(const Eigen::VectorQd& q_pos_lb_, const Eigen::VectorQd& q_pos_ub_)
{
    q_pos_lb = q_pos_lb_;
    q_pos_ub = q_pos_ub_;
}

void CbfManager::computeJointLimitCbfConstraint()
{
    // Gradient
    joint_limit_constraints.A.setIdentity(MODEL_DOF, MODEL_DOF);

    // Lower and upper bounds
    if(cbf_mode == CbfType::Dyn)
    {
        joint_limit_constraints.lbA = (-1.0) * (alpha_joint_limit + alpha_joint_limit) * rd_.q_dot_ 
                                    + (alpha_joint_limit * alpha_joint_limit) * (q_pos_lb - rd_.q_) ;
                                    + (1.0 / epsilon_joint_limit) * Eigen::VectorQd::Ones();
        
        joint_limit_constraints.ubA = (-1.0) * (alpha_joint_limit + alpha_joint_limit) * rd_.q_dot_ 
                                    + (alpha_joint_limit * alpha_joint_limit) * (q_pos_ub - rd_.q_) ;
                                    - (1.0 / epsilon_joint_limit) * Eigen::VectorQd::Ones();
    }
    else if (cbf_mode == CbfType::Kin)
    {
        joint_limit_constraints.lbA = alpha_joint_limit * (q_pos_lb - rd_.q_) + (1.0 / epsilon_joint_limit) * Eigen::VectorQd::Ones();
        
        joint_limit_constraints.ubA = alpha_joint_limit * (q_pos_ub - rd_.q_) - (1.0 / epsilon_joint_limit) * Eigen::VectorQd::Ones();
    }

}

void CbfManager::getJointLimitCbfConstraint(Eigen::Ref<Eigen::MatrixXd> A, Eigen::VectorQd &lbA, Eigen::VectorQd &ubA) const
{
    if(cbf_mode == CbfType::Dyn)
    {
        // Gradient
        A = joint_limit_constraints_fast.A;

        // Lower and upper bounds
        lbA = joint_limit_constraints_fast.lbA;
        ubA = joint_limit_constraints_fast.ubA;
    }
    else if (cbf_mode == CbfType::Kin)
    {
        // Gradient
        A = joint_limit_constraints.A;

        // Lower and upper bounds
        lbA = joint_limit_constraints.lbA;
        ubA = joint_limit_constraints.ubA;
    }
}

// =========================
// Workspace Boundary CBF
// =========================
void CbfManager::setWorkspaceBoundaryCbfParameters(const double &alpha, const double &epsilon)
{   
    alpha_workspace  = alpha;
    epsilon_workspace = epsilon;
}

void CbfManager::setWorkspaceBoundaryPairs(const std::vector<WorkspaceBoundaryPair>& pairs)
{
    workspace_boundary_pairs_ = pairs;
}

void CbfManager::computeWorkspaceBoundaryCbfConstraint()
{
    const int m = static_cast<int>(workspace_boundary_pairs_.size());

    // Gradient
    workspace_boundary_constraints.A.setZero(m, MODEL_DOF_VIRTUAL);

    // Lower and upper bounds
    workspace_boundary_constraints.lbA.setZero(m);
    workspace_boundary_constraints.ubA.setZero(m);

    for (int i = 0; i < m; ++i)
    {
        const auto& pr = workspace_boundary_pairs_[i];

        const LinkData& linkA = rd_.link_[pr.idx_A];
        const LinkData& linkB = rd_.link_[pr.idx_B];

        Eigen::MatrixXd J_AB, Jqdot_AB;
        const double dist_AB = getSignedDistanceFunction(linkA, linkB, J_AB, Jqdot_AB);

        workspace_boundary_constraints.A.block(i, 0, 1, MODEL_DOF_VIRTUAL) = -J_AB;

        const double Jdotqdot = Jqdot_AB(0, 0);
        const double Jqdot    = (J_AB * rd_.local_q_dot_virtual_)(0, 0);

        double h = 0.0;
        if(cbf_mode == CbfType::Dyn)
        {
            h = - Jdotqdot
                - (alpha_workspace + alpha_workspace) * Jqdot
                + (alpha_workspace * alpha_workspace) * (pr.max_dist - dist_AB)
                - (1.0 / epsilon_workspace) * (J_AB * J_AB.transpose())(0,0);
        }
        else if(cbf_mode == CbfType::Kin)
        {
            h = (+1.0) * alpha_workspace * (pr.max_dist - dist_AB); // - (1.0 / epsilon_workspace) * (-J_AB).squaredNorm();
        }

        workspace_boundary_constraints.lbA(i) = (-1.0) * h;
        workspace_boundary_constraints.ubA(i) = 1e5;
    }
}

void CbfManager::getWorkspaceBoundaryCbfConstraint(Eigen::Ref<Eigen::MatrixXd> A, Eigen::VectorXd &lbA, Eigen::VectorXd &ubA) const
{
    if (cbf_mode == CbfType::Dyn)
    {
        // Gradient
        A = workspace_boundary_constraints_fast.A;

        // Lower and upper bounds
        lbA = workspace_boundary_constraints_fast.lbA;
        ubA = workspace_boundary_constraints_fast.ubA;
    }
    else if (cbf_mode == CbfType::Kin)
    {
        // Gradient
        A = workspace_boundary_constraints.A;

        // Lower and upper bounds
        lbA = workspace_boundary_constraints.lbA;
        ubA = workspace_boundary_constraints.ubA;
    }
}

int CbfManager::getNumWorkspaceBoundaryPairs() const
{
    return static_cast<int>(workspace_boundary_pairs_.size());
}

// ===============================
// Self-collision constraints CBF
// ===============================
void CbfManager::setSelfCollisionCbfParameters(const double &alpha, const double &epsilon)
{
    alpha_self_collision  = alpha;
    epsilon_self_collision = epsilon;
}

void CbfManager::computeSelfCollisionCbfConstraint()
{
    self_collision_avoidance_constraints.num_pairs = col_mgr_.self_collision_avoid_terms_.num_pairs;

    // Gradient    
    self_collision_avoidance_constraints.A.setZero(self_collision_avoidance_constraints.num_pairs, MODEL_DOF_VIRTUAL);

    self_collision_avoidance_constraints.A = col_mgr_.self_collision_avoid_terms_.J;

    self_collision_avoidance_constraints.lbA.setZero(self_collision_avoidance_constraints.num_pairs);
    self_collision_avoidance_constraints.ubA.setZero(self_collision_avoidance_constraints.num_pairs);

    // Lower and upper bounds
    if(cbf_mode == CbfType::Dyn)
    {
        self_collision_avoidance_constraints.lbA = - col_mgr_.self_collision_avoid_terms_.Jdotqdot
                                                   - (alpha_self_collision + alpha_self_collision) * (self_collision_avoidance_constraints.A * rd_.local_q_dot_virtual_) 
                                                   - (alpha_self_collision * alpha_self_collision) * (col_mgr_.self_collision_avoid_terms_.min_distances) 
                                                   + (1.0 / epsilon_self_collision) * self_collision_avoidance_constraints.A.rowwise().squaredNorm();
    }
    else if(cbf_mode == CbfType::Kin)
    {
        self_collision_avoidance_constraints.lbA =   (-1.0) * (alpha_self_collision) * (col_mgr_.self_collision_avoid_terms_.min_distances); 
                                                 //  + (1.0 / epsilon_self_collision) * self_collision_avoidance_constraints.A.rowwise().squaredNorm();
    }

    self_collision_avoidance_constraints.ubA.setConstant(1e5);
}

void CbfManager::getSelfCollisionCbfConstraint(Eigen::Ref<Eigen::MatrixXd> A, Eigen::VectorXd &lbA, Eigen::VectorXd &ubA) const
{
    // Gradient
    if (cbf_mode == CbfType::Dyn)
    {
        // Gradient
        A = self_collision_avoidance_constraints_fast.A;

        // Lower and upper bounds
        lbA = self_collision_avoidance_constraints_fast.lbA;
        ubA = self_collision_avoidance_constraints_fast.ubA;
    }
    else if (cbf_mode == CbfType::Kin)
    {
        // Gradient
        A = self_collision_avoidance_constraints.A;

        // Lower and upper bounds
        lbA = self_collision_avoidance_constraints.lbA;
        ubA = self_collision_avoidance_constraints.ubA;
    }
}

int CbfManager::getNumSelfCollisionPairs() const
{
    if (cbf_mode == CbfType::Dyn)
    {
        return self_collision_avoidance_constraints_fast.num_pairs;
    }
    else if (cbf_mode == CbfType::Kin)
    {
        return self_collision_avoidance_constraints.num_pairs;
    }

    return 0;
}

// ===============================
// Obstacle avoidance constraints CBF
// ===============================
void CbfManager::setObstacleAvoidanceCbfParameters(const double &alpha, const double &epsilon)
{
    alpha_obstacle_avoidance  = alpha;
    epsilon_obstacle_avoidance = epsilon;
}

void CbfManager::computeObstacleAvoidanceCbfConstraint()
{
    obstacle_avoidance_constraints.num_pairs = col_mgr_.obstacle_avoid_terms_.num_pairs;

    // Gradient
    obstacle_avoidance_constraints.A.setZero(obstacle_avoidance_constraints.num_pairs, MODEL_DOF_VIRTUAL);

    obstacle_avoidance_constraints.A = col_mgr_.obstacle_avoid_terms_.J;

    // Lower and upper bounds
    obstacle_avoidance_constraints.lbA.setZero(obstacle_avoidance_constraints.num_pairs);
    obstacle_avoidance_constraints.ubA.setZero(obstacle_avoidance_constraints.num_pairs);

    if(cbf_mode == CbfType::Dyn){
        obstacle_avoidance_constraints.lbA = - col_mgr_.obstacle_avoid_terms_.Jdotqdot
                                             - (alpha_obstacle_avoidance + alpha_obstacle_avoidance) * (obstacle_avoidance_constraints.A * rd_.local_q_dot_virtual_ - col_mgr_.obstacle_avoid_terms_.obs_vel_projections)
                                             - (alpha_obstacle_avoidance * alpha_obstacle_avoidance) * (col_mgr_.obstacle_avoid_terms_.min_distances)
                                             + (1.0 / epsilon_obstacle_avoidance) * obstacle_avoidance_constraints.A.rowwise().squaredNorm();
    }
    else if(cbf_mode == CbfType::Kin)
    {
        obstacle_avoidance_constraints.lbA =   (-1.0) * (alpha_obstacle_avoidance) * (col_mgr_.obstacle_avoid_terms_.min_distances)
                                             + col_mgr_.obstacle_avoid_terms_.obs_vel_projections;
                                            //  + (1.0 / epsilon_obstacle_avoidance) * obstacle_avoidance_constraints.A.rowwise().squaredNorm();
    }

    obstacle_avoidance_constraints.ubA.setConstant(1e5);
}

void CbfManager::getObstacleAvoidanceCbfConstraint(Eigen::Ref<Eigen::MatrixXd> A, Eigen::VectorXd &lbA, Eigen::VectorXd &ubA) const
{
    if (cbf_mode == CbfType::Dyn)
    {
        // Gradient
        A = obstacle_avoidance_constraints_fast.A;

        // Lower and upper bounds
        lbA = obstacle_avoidance_constraints_fast.lbA;
        ubA = obstacle_avoidance_constraints_fast.ubA;
    }
    else if (cbf_mode == CbfType::Kin)
    {
        // Gradient
        A = obstacle_avoidance_constraints.A;

        // Lower and upper bounds
        lbA = obstacle_avoidance_constraints.lbA;
        ubA = obstacle_avoidance_constraints.ubA;
    }
}

int CbfManager::getNumObstacleAvoidancePairs() const
{
    if (cbf_mode == CbfType::Dyn)
    {
        return obstacle_avoidance_constraints_fast.num_pairs;
    }
    else if (cbf_mode == CbfType::Kin)
    {
        return obstacle_avoidance_constraints.num_pairs;
    }

    return 0;
}

// ===============================
// Utils
// ===============================
double CbfManager::getSignedDistanceFunction(const LinkData& linkA_,
                                             const LinkData& linkB_,
                                             Eigen::MatrixXd& J_AB,
                                             Eigen::MatrixXd& Jqdot_AB) const
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