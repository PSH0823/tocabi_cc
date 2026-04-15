#pragma once

#include <eigen3/Eigen/Core>
#include <map>
#include <vector>
#include <string>
#include "wholebody_functions.h"
#include "cbf_manager/cbf_manager.h"
#include "utils.h"

class KinWBC {
public:
    KinWBC(RobotData& rd, CbfManager& cbf_mgr);
    void computeTaskSpaceKinematicWBC();
    void setTaskHierarchy(const TaskMotionType& motion_mode_);
    void setInitialConfiguration(const Eigen::VectorQd &q_init_des_);
    
private:
    RobotData &rd_;
    CbfManager &cbf_mgr_;

    std::vector<std::vector<TaskInfo>> task_hierarchy;
    TaskMotionType motion_mode;
    bool is_cannot_solve_qp_ = true;

    Eigen::VectorVQd safetyFilter();
    CQuadraticProgram QP_safety_filter;

        void calcCostHess();
        void calcCostGrad();
        void calcEqualityConstraint();
        void calcInequalityConstraint();
        void checkGradHessSize();
        double getSignedDistanceFunction(LinkData &linkA_, LinkData &linkB_, Eigen::MatrixXd &J_AB, Eigen::MatrixXd &Jqdot_AB);
    
    Eigen::MatrixXd Hess; 
    Eigen::VectorXd grad;
    Eigen::MatrixXd A_const;   
    Eigen::VectorXd lbA_const;
    Eigen::VectorXd ubA_const;
    std::vector<ConstraintMatrix> constraints_;
    int total_num_state = 0;
    int total_num_constraints = 0;
    bool num_constraints_changed = false;

    Eigen::VectorVQd qdot_des;
    Eigen::VectorVQd qdot_safety;

    Eigen::VectorQVQd integrate(const Eigen::VectorQVQd &q_current, const Eigen::VectorVQd &q_delta_);
    Eigen::VectorVQd computeDesiredJointAcceleration(const Eigen::VectorQVQd &q_current, const Eigen::VectorVQd &qdot_current, const Eigen::VectorQVQd &q_desired, const Eigen::MatrixVVd &Kp, const Eigen::MatrixVVd &Kd);

    Eigen::VectorQd q_init_des;

};