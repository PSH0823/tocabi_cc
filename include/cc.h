// Must inlude while using Pinocchio in noetic
// to avoid compilation errors from differing Boost-variant sizes.
#include <pinocchio/fwd.hpp>

#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include <std_msgs/String.h>
#include "math_type_define.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include "tocabi_msgs/matrix_3_4.h"
#include <std_msgs/String.h>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <mpc.h>

// Pinocchio Headers
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/parsers/urdf.hpp>


class CustomController
{
public:
    CustomController(RobotData &rd);
    Eigen::VectorQd getControl();
    double getMpcFrequency() const;

    void computeSlow();
    void computeFast();
    void computeThread3();
    void computePlanner();
    void copyRobotData(RobotData &rd_l);
    void pubDataSlowToThread3();
    void subDataSlowToThread3();
    void pubDataThread3ToSlow();
    void subDataThread3ToSlow();

    RobotData &rd_;
    RobotData rd_cc_;

    ros::NodeHandle nh_cc_;
    ros::CallbackQueue queue_cc_;

    // ROBOT 
    RigidBodyDynamics::Model model_d_;  //updated by desired q
    RigidBodyDynamics::Model model_c_;  //updated by current q

    void getParameterYAML();

    // LOW-LEVEL CONTROL
    void setGains();
    void moveInitialPose();
    Eigen::VectorQd q_init_;
    Eigen::VectorQd q_ref_;
    Eigen::VectorQd q_prev_;
    Eigen::Vector12d q_leg_desired_;
    Eigen::VectorQd Kp;
    Eigen::MatrixQQd Kp_diag;
    Eigen::VectorQd Kd;
    Eigen::MatrixQQd Kd_diag;

    Eigen::VectorXd joint_limit_l_;
    Eigen::VectorXd joint_limit_h_;
    Eigen::VectorXd joint_vel_limit_l_;
    Eigen::VectorXd joint_vel_limit_h_;

    // BOOLEAN OPERATOR
    bool walking_enable_;
    bool is_mode_6_init = true;
    bool is_mode_7_init = true;
    bool is_joystick_mode = false;
    bool is_support_foot_change = false;    
    bool is_lfoot_support = false;
    bool is_rfoot_support = false;
    bool is_dsp1 = false;
    bool is_ssp  = false;
    bool is_dsp2 = false;
    bool is_preview_ctrl_init = true;
    bool is_mpc_ctrl_init = true;
    bool is_mpc_x_update = false;
    bool is_mpc_y_update = false;

    std::atomic<bool> atb_grav_update_{false};
    std::atomic<bool> atb_mpc_update_{false};
    std::atomic<bool> atb_mpc_x_update_{false};
    std::atomic<bool> atb_mpc_y_update_{false};

    // BIPED WALKING PARAMETER
    void walkingParameterSetting();
    double target_x_;
    double target_y_;
    double target_z_;
    double target_theta_ = 0.0;
    double step_length_x_;
    double step_length_y_;
    double com_height_;
    double foot_height_;
    int is_right_foot_swing_;

    double t_last_;
    double t_start_;
    double t_start_real_;
    double t_temp_;  
    double t_rest_init_;
    double t_rest_last_;
    double t_double1_;
    double t_double2_;

    double t_total_;
    double t_dsp1_;
    double t_ssp_;
    double t_dsp2_;

    int current_step_num_, current_step_container, current_step_thread3, current_step_checker;
    int total_step_num_;

    Eigen::VectorQd Gravity_fast_;
    Eigen::VectorQd Gravity_;
    Eigen::VectorQd Gravity_DSP_;
    Eigen::VectorQd Gravity_SSP_;

    // BIPED WALKING CONTROL
    void updateInitialState();
    void getRobotState();
    void walkingStateMachine();
    void calculateFootStepTotal();
    void supportToFloatPattern();
    void floatToSupportFootstep();
    void updateNextStepTime();
    void gravityCalculate();

    void getZmpTrajectory();
    void addZmpOffset();
    void zmpGenerator(const unsigned int norm_size, const unsigned planning_step_num);
    void onestepZmp(unsigned int current_step_number, Eigen::VectorXd &temp_px, Eigen::VectorXd &temp_py);
    void onestepZmp_wo_offset(unsigned int current_step_number, Eigen::VectorXd &temp_px, Eigen::VectorXd &temp_py, Eigen::VectorXd &temp_px_wo_offset, Eigen::VectorXd &temp_py_wo_offset);

    void getComTrajectory_offline();
    void comGenerator(const unsigned int norm_size, const unsigned planning_step_num);
    void onestepCom(unsigned int current_step_number, Eigen::VectorXd &temp_cx, Eigen::VectorXd &temp_cy);
    void getComTrajectory(); 
    void getComTrajectory_preview();   
    void previewcontroller(double dt, int NL, int tick, 
                           Eigen::Vector3d &x_k, Eigen::Vector3d &y_k, double &UX, double &UY,
                           const Eigen::MatrixXd &Gi, const Eigen::VectorXd &Gd, const Eigen::MatrixXd &Gx, 
                           const Eigen::MatrixXd &A,  const Eigen::VectorXd &B,  const Eigen::MatrixXd &C);
    void preview_Parameter(double dt, int NL, Eigen::MatrixXd& Gi, Eigen::VectorXd& Gd, Eigen::MatrixXd& Gx, Eigen::MatrixXd& A, Eigen::VectorXd& B, Eigen::MatrixXd& C);
    void getComTrajectory_mpc();
    void getCPTrajectory();
    void getFootTrajectory(); 
    void getFootTrajectory_Kajita();
    void getPelvTrajectory();
    void contactWrenchCalculator();
    void contactWrench_Kajita();
    void calculateZMP_wo_offset();
    void computeIkControl(const Eigen::Isometry3d &float_trunk_transform, const Eigen::Isometry3d &float_lleg_transform, const Eigen::Isometry3d &float_rleg_transform, Eigen::Vector12d &q_des);
    void circling_motion();
    
    Eigen::MatrixXd foot_step_;
    Eigen::MatrixXd foot_step_support_frame_;
    Eigen::MatrixXd foot_step_support_frame_offset_;

    Eigen::Isometry3d pelv_support_start_;
    Eigen::Isometry3d pelv_support_init_;
    Eigen::Vector2d del_zmp;
    Eigen::Vector2d cp_desired_;
    Eigen::Vector2d cp_measured_;
    Eigen::Vector2d cp_measured_LPF;
    Eigen::Vector2d cp_measured_thread_;
    Eigen::Vector2d cp_measured_mpc_;
    Eigen::Vector2d cp_reference_eos_;
    Eigen::Vector3d cp_float_current_;
    double T_d_cp_eos_;

    Eigen::Vector3d com_desired_;
    Eigen::Vector3d com_desired_b_;
    Eigen::Vector3d com_desired_b_b_;
    Eigen::Vector3d com_desired_dot_;
    Eigen::Vector3d com_desired_ddot_;

    Eigen::Vector3d com_support_init_;
    Eigen::Vector3d com_float_init_;
    Eigen::Vector3d com_float_current_;
    Eigen::Vector3d com_support_current_;
    Eigen::Vector3d com_support_current_dot_;
    Eigen::Vector3d com_support_current_LPF;
    Eigen::Vector3d com_float_current_LPF;
    Eigen::Vector3d com_support_current_prev;
    Eigen::Vector3d com_support_cp_;

    Eigen::Vector3d com_float_current_dot;
    Eigen::Vector3d com_float_current_dot_prev;
    Eigen::Vector3d com_float_current_dot_LPF;
    Eigen::Vector3d com_support_current_dot_LPF;
    Eigen::Vector3d com_reference_eos_;

    Eigen::Vector3d pelv_rpy_current_;
    Eigen::Vector3d rfoot_rpy_current_;
    Eigen::Vector3d lfoot_rpy_current_;
    Eigen::Isometry3d pelv_yaw_rot_current_from_global_;
    Eigen::Isometry3d rfoot_roll_rot_;
    Eigen::Isometry3d lfoot_roll_rot_;
    Eigen::Isometry3d rfoot_pitch_rot_;
    Eigen::Isometry3d lfoot_pitch_rot_;

    Eigen::Isometry3d pelv_float_current_;
    Eigen::Isometry3d lfoot_float_current_;
    Eigen::Isometry3d rfoot_float_current_;
    Eigen::Isometry3d pelv_float_init_;
    Eigen::Isometry3d lfoot_float_init_;
    Eigen::Isometry3d rfoot_float_init_;

    Eigen::Isometry3d pelv_trajectory_support_; //local frame
    Eigen::Isometry3d pelv_trajectory_support_fast_; //local frame
    Eigen::Isometry3d pelv_trajectory_support_slow_; //local frame
    
    Eigen::Isometry3d rfoot_trajectory_support_;  //local frame
    Eigen::Isometry3d lfoot_trajectory_support_;
    Eigen::Isometry3d lfoot_trajectory_support_fast_;
    Eigen::Isometry3d lfoot_trajectory_support_slow_;

    Eigen::Vector3d rfoot_trajectory_euler_support_;
    Eigen::Vector3d lfoot_trajectory_euler_support_;

    Eigen::Isometry3d pelv_trajectory_float_; //pelvis frame

    Eigen::Vector3d com_trajectory_float_;

    Eigen::Isometry3d lfoot_trajectory_float_;
    Eigen::Isometry3d lfoot_trajectory_float_fast_;
    Eigen::Isometry3d lfoot_trajectory_float_slow_;

    Eigen::Isometry3d rfoot_trajectory_float_;
    Eigen::Isometry3d rfoot_trajectory_float_fast_;
    Eigen::Isometry3d rfoot_trajectory_float_slow_;

    Eigen::Vector3d pelv_support_euler_init_;
    Eigen::Vector3d lfoot_support_euler_init_;
    Eigen::Vector3d rfoot_support_euler_init_;
    double wn = 0;

    double walking_end_flag = 0;
    
    Eigen::Isometry3d swingfoot_float_current_; 
    Eigen::Isometry3d supportfoot_float_current_; 

    Eigen::Isometry3d pelv_support_current_;
    Eigen::Isometry3d lfoot_support_current_;
    Eigen::Isometry3d rfoot_support_current_;

    Eigen::Isometry3d lfoot_support_init_;
    Eigen::Isometry3d rfoot_support_init_;
    
    Eigen::Vector6d supportfoot_support_init_offset_;
    Eigen::Vector6d supportfoot_float_init_;
    Eigen::Vector6d supportfoot_support_init_;
    Eigen::Vector6d swingfoot_float_init_;
    Eigen::Vector6d swingfoot_support_init_;
    
    Eigen::MatrixXd ref_com_;
    Eigen::MatrixXd ref_com_ddot_;

    Eigen::MatrixXd ref_zmp_;
    Eigen::MatrixXd ref_zmp_container;
    Eigen::MatrixXd ref_zmp_thread3;
    
    Eigen::MatrixXd ref_zmp_wo_offset_;
    Eigen::MatrixXd ref_zmp_wo_offset_container;
    Eigen::MatrixXd ref_zmp_wo_offset_thread3;

    int first_current_step_flag_ = 0;
    int first_current_step_number_ = 0;
    
    double Tau_L_x_error_ = 0;
    double Tau_L_x_error_pre_ = 0;
    double Tau_L_x_error_dot_ = 0;    

    double Tau_L_y_error_ = 0;
    double Tau_L_y_error_pre_ = 0;
    double Tau_L_y_error_dot_ = 0;    

    double Tau_R_x_error_ = 0;
    double Tau_R_x_error_pre_ = 0;
    double Tau_R_x_error_dot_ = 0;    

    double Tau_R_y_error_ = 0;
    double Tau_R_y_error_pre_ = 0;
    double Tau_R_y_error_dot_ = 0;    

    double F_F_error_ = 0;
    double F_F_error_pre_ = 0;
    double F_F_error_dot_ = 0;

    double F_L_ = 0;
    double F_L_b_ = 0;
    double F_R_ = 0;
    double F_R_b_ = 0;
    
    double Tau_all_x_ = 0;
    double Tau_all_y_ = 0;

    double Tau_R_x_ = 0;
    double Tau_R_x_b_ = 0;
    double Tau_L_x_ = 0;
    double Tau_L_x_b_ = 0;

    double Tau_R_y_ = 0;
    double Tau_R_y_b_ = 0;
    double Tau_L_y_ = 0;
    double Tau_L_y_b_ = 0;

    double P_angle_i = 0;
    double P_angle = 0;
    double P_angle_input_dot = 0;
    double P_angle_input = 0;
    double R_angle = 0;
    double R_angle_input_dot = 0;
    double R_angle_input = 0;
    double aa = 0; 
    double Y_angle_input = 0;

    Eigen::Vector6d l_ft_;
    Eigen::Vector6d l_ft_b_;
    Eigen::Vector6d r_ft_;
    Eigen::Vector6d r_ft_b_;
    Eigen::Vector6d l_ft_LPF;
    Eigen::Vector6d r_ft_LPF;
    Eigen::Vector6d l_ft_LPF_b_;
    Eigen::Vector6d r_ft_LPF_b_;
    Eigen::Vector2d zmp_measured_mj_;
    Eigen::Vector2d zmp_measured_LPF_;

    // PREVIEW CONTROL
    Eigen::Vector3d x_preview_;
    Eigen::Vector3d y_preview_;

    Eigen::Vector3d xs_preview_;
    Eigen::Vector3d ys_preview_;
    Eigen::Vector3d xd_preview_;
    Eigen::Vector3d yd_preview_; 

    Eigen::MatrixXd Gi_preview_;
    Eigen::MatrixXd Gx_preview_;
    Eigen::VectorXd Gd_preview_;
    Eigen::MatrixXd A_preview_;
    Eigen::VectorXd B_preview_;
    Eigen::MatrixXd C_preview_;
    double UX_preview_, UY_preview_;

    // MODEL PREDICTIVE CONTROL
    Eigen::Vector3d x_mpc_, x_mpc_prev, x_mpc_container, x_mpc_container2, x_mpc_thread3;
    Eigen::Vector3d y_mpc_, y_mpc_prev, y_mpc_container, y_mpc_container2, y_mpc_thread3;

    Eigen::VectorXd zx_ref;
    Eigen::VectorXd zx_ref_wo_offset;
    Eigen::VectorXd zy_ref;
    Eigen::VectorXd zy_ref_wo_offset;

    unsigned int mpc_interpol_cnt_x = 0;
    unsigned int mpc_interpol_cnt_y = 0;

    double del_t = 0.0005;
    double xi_preview_;
    double yi_preview_;
    double zc_preview_;
    double ZMP_X_REF_;
    double ZMP_Y_REF_;
    double ZMP_Y_REF_alpha_;
    double alpha_lpf_ = 0.0;

    // CONTACT WRENCH CONTROL
    Eigen::VectorQd swing_wrench_torque;
    Eigen::VectorQd contact_wrench_torque;
    Eigen::Vector6d rfoot_contact_wrench;
    Eigen::Vector6d lfoot_contact_wrench;

    double F_F_input_dot = 0;
    double F_F_input = 0;
    double z_ctrl_ = 0;

    double F_T_L_x_input = 0;
    double delta_phi_L_ = 0;
    double F_T_L_x_input_dot = 0;
    double F_T_R_x_input = 0;
    double delta_phi_R = 0;
    double F_T_R_x_input_dot = 0;  

    double F_T_L_y_input = 0;
    double delta_theta_L_ = 0;
    double F_T_L_y_input_dot = 0;
    double F_T_R_y_input = 0;
    double delta_theta_R_ = 0;
    double F_T_R_y_input_dot = 0;

    double kp_cp = 0.0;
    double zmp_offset_ = 0.0;

    /*******************************************
     * Pinocchio Related Methods and Variables *
     *******************************************/
    
     /**
      * @brief Compute the mass matrix of the robot with floating base
      * 
      * @param q_virtual_pin the joint configuration of the robot + the virtual joints of the floating base
      *                      (q[0:2]: base pos., q[3:6]: base ori.quaternion)
      * @return The mass matrix of the robot(including floating base)
      */
    Eigen::MatrixXd pinGetMassMatrix(
        const Eigen::VectorQVQd &q_virtual_pin
        );
    
    /**
     * @brief Compute the Jacobian matrix of a given frame of the robot with floating base
     * 
     * @param q_virtual_pin the joint configuration of the robot + the virtual joints of the floating base
     *                      (q[0:2]: base pos., q[3:6]: base ori.quaternion)
     * @param frame_id the id of TOCABI model in Pinocchio(see enum Pin)
     * 
     * @return The Jacobian matrix of the given frame(including floating base)
     */
    Eigen::MatrixXd pinGetFrameJacobian(
        const Eigen::VectorQVQd &q_virtual_pin,
        const int frame_id
        );
    
    /**
     * @brief Convert the joint configuration including floating base from RBDL format to Pinocchio format
     * 
     * @param q_virtual_rbdl the joint configuration of the robot + the virtual joints of the floating base in RBDL format
     * 
     * @return The joint configuration of the robot + the virtual joints of the floating base in Pinocchio format
     * 
     * @note q_virtual format of RBDL : [base pos.(x,y,z), base ori. quat(x,y,z), joint pos.(n), base ori. quat(w)]
     * 
     *       q_virtual format of Pinocchio : [base pos.(x,y,z), base ori. quat(x,y,z,w), joint pos.(n)]
     */
    Eigen::VectorQVQd convertQVirtualRBDLtoPin(
        const Eigen::VectorQVQd &q_virtual_rbdl
        );

    /** @brief Variables using Pinocchio library*/
    // model of the Robot
    pinocchio::Model pin_model_;
    // data of the Robot
    pinocchio::Data pin_data_;
    pinocchio::Data pin_data_wov;
    // mass matrix 
    Eigen::MatrixXd M_pin_;
    // Jacobian matrix
    Eigen::MatrixXd J_pin_pelv_;    // pelvis
    Eigen::MatrixXd J_pin_rfoot_;   // right foot
    Eigen::MatrixXd J_pin_lfoot_;   // left foot
    // joint positions(including floating base) of pinocchio
    Eigen::VectorQVQd q_virtual_pin_;
    
    // enum for the frame id of TOCABI model in Pinocchio
    enum FrameIdxPin
    { 
        PELVIS,
        LEFT_FOOT, 
        ROGHT_FOOT,
        LEFT_ELBOW, 
        LEFT_HAND,
        HEAD,
        RIGHT_ELBOW,
        RIGHT_HAND,
        FRAME_COUNT // total number of specific frames (used for frame_id_pin_ array sizing)
    };

    // array to store the frame ids of specific links
    std::array<int,FRAME_COUNT> frame_id_pin_;

private:
    Eigen::VectorQd ControlVal_;
    unsigned int walking_tick = 0;
    unsigned int walking_tick_container = 0;
    unsigned int walking_tick_thread3 = 0;

    unsigned int zmp_start_time_ = 0; 
    unsigned int zmp_start_time_container = 0; 
    unsigned int zmp_start_time_thread3 = 0; 

    unsigned int com_start_time_ = 0;

    unsigned int initial_tick_ = 0;
    const double hz_ = 2000.0;

    //const double mpc_freq = 100.0;
    const double mpc_freq = 50.0;
    //const double mpc_N  = 200.0;
    const double mpc_N  = 100.0;

    unsigned int initial_flag = 0;
};
