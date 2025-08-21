#include "math_type_define.h"
#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"

class MPC
{
    public:
        MPC(double initMpcFreq, double initN);
        // ~MPC();

        void cartTableModel(double T, double h);
        void cartTableModelMPC(double T, double com_height);
        void ComTrajectoryGenerator(Eigen::VectorXd &zx_ref, Eigen::VectorXd &zy_ref, Eigen::VectorXd &zx_ref_wo_offset, Eigen::VectorXd &zy_ref_wo_offset, Eigen::Vector3d &x_hat, Eigen::Vector3d &y_hat, double comHeight);

        bool is_mpc_init_ = true;

        Eigen::Vector2d zmp_calc_mpc;

        Eigen::MatrixXd A_mpc, B_mpc, C_mpc;
        Eigen::MatrixXd P_ps_mpc, P_pu_mpc, P_vs_mpc, P_vu_mpc, P_zs_mpc, P_zu_mpc;
        Eigen::MatrixXd Q_mpc, R_mpc;
        
        Eigen::MatrixXd Hess_;
        Eigen::VectorXd grad_x;
        Eigen::VectorXd grad_y;
        Eigen::VectorXd lb_x;
        Eigen::VectorXd ub_x;
        Eigen::VectorXd lb_y;
        Eigen::VectorXd ub_y;

        // QPOASES
        CQuadraticProgram QP_mpc_x_;
        CQuadraticProgram QP_mpc_y_;
        Eigen::VectorXd U_x;
        Eigen::VectorXd U_y;
    private:
        const double mpc_freq;
        const double N;
};