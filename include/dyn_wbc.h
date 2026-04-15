// dyn_wbc.h
#ifndef DYN_WBC_H
#define DYN_WBC_H

#include <Eigen/Dense>
#include <qpOASES.hpp>
#include "wholebody_functions.h"
#include "cbf_manager/cbf_manager.h"
#include "utils.h"
#include <iomanip>

class DynWBC
{
public:
    DynWBC(RobotData& rd, CbfManager& cbf_mgr);

    //--- QP WBC 

    Eigen::VectorQd computeDynamicWBC();
    void updateControlCommands(const Eigen::Vector12d &contact_wrench_cmd_, const Eigen::VectorVQd &qddot_cmd_);
    void updateRobotStates(const Eigen::MatrixVVd &M_, const Eigen::VectorVQd &G_, const Eigen::MatrixXd &J_C_);

    //--- Setter
    void setFrictionCoefficient(const double& mu_);
    void setFootDimension(const double& foot_size_, const double& foot_width_);
    void setJointTrackingWeight(const double& W_qddot_);
    void setContactWrenchTrackingWeight(const double& W_cwr_);
    void setTorqueMinimizationWeight(const double &W_torque_);
    void setAccelEnergyMinimizationWeight(const double& W_energy_);


private:
    RobotData &rd_;
    CbfManager& cbf_mgr_;

    CQuadraticProgram QP_Dyn_Wbc;
        void calcCostGrad();
        void calcCostHess();
        void calcEqualityConstraint();
        void calcInequalityConstraint();
        void checkGradHessSize();
        double getSignedDistanceFunction(LinkData &linkA_, LinkData &linkB_, Eigen::MatrixXd &J_AB, Eigen::MatrixXd &Jqdot_AB);

        Eigen::MatrixXd Hess;  // HESSIAN
        Eigen::VectorXd grad;  // GRADIENT
        Eigen::MatrixXd A_const;    
        Eigen::VectorXd lbA_const;  
        Eigen::VectorXd ubA_const;  
        std::vector<ConstraintMatrix> constraints_; // CONSTRAINTS
        int total_num_state = 0;
        int total_num_constraints = 0;
        bool num_constraints_changed = false;

    Eigen::VectorXd  contact_wrench_cmd;
    Eigen::VectorVQd qddot_cmd;
    Eigen::VectorQd  torque_prev;

    Eigen::VectorXd contact_wrench_qp; 
    Eigen::VectorXd qddot_qp; 
    Eigen::VectorXd torque_qp; 

    bool is_gradhess_init_ = true;
    bool is_cannot_solve_qp_ = true;

    double mu = 0.0;
    double foot_size = 0.0; 
    double foot_width = 0.0; 

    //--- Local eigen variables
    Eigen::MatrixVVd M; 
    Eigen::MatrixVVd M_inv; 
    Eigen::VectorVQd G; 
    Eigen::MatrixXd J_C;
    Eigen::MatrixVVd N_C; 
    Eigen::MatrixXd J_C_T;
    Eigen::MatrixXd Sa_T;
    Eigen::MatrixXd Sa;  
    Eigen::MatrixXd Sf;  

    Eigen::MatrixXd A_fric;
    Eigen::VectorXd lbA_fric;
    Eigen::VectorXd ubA_fric;

    double W_qddot = 1.0;
    double W_cwr = 1e-5;
    double W_torque = 1.0;
    double W_energy = 1.0;

    int contact_dim = 12;
    int contact_dim_prev = 12;

    bool local_LF_contact = false;
    bool local_RF_contact = false;
};

#endif  // DYN_WBC_H