#pragma once
#include <Eigen/Dense>

#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include "utils.h"
#include <deque>
#include <iomanip>

class WalkingManager 
{
public:
WalkingManager(RobotData &rd);

void computeWalkingMotion();
void updateSupportPhaseIndicator();

//---Setter
void updateContactState(const bool &local_LF_contact_, const bool &local_RF_contact_);
void setControlFrequency(const double &hz);
void setCenterOfMassHeight(const double &com_height_);
// void setWalkingParameter(const double &step_length_, const double &foot_yaw_angle_, const double &foot_height_);
void setWalkingParameter(const double &step_length_, const double &step_lateral_, const double &foot_yaw_angle_, const double &foot_height_);

void setStepDuration(const double &step_duration_);
void setDspDuration(const double &dsp_duration_);
void setTransferDuration(const double &transfer_duration_);
void findPreviewParameter(double dt, int NL);   

//---Getter
double getPreviewStepNumber();
ContactIndicator getSupportPhaseIndicator();

void contactWrenchCalculator();
Eigen::Vector2d cp_desired_;
Eigen::Vector2d cp_measured_;
Eigen::Vector2d cp_error_;
Eigen::Vector2d cp_error_int_;

Eigen::Vector2d del_zmp;
double zmp_kp_ = 3.0;               // proportional gain
double zmp_ki_ = 0.5;               // integral gain

double zmp_x_ref = 0.0;
double zmp_y_ref = 0.0;
double wn = 0.0;

private:
RobotData &rd_;
double hz_ = 2000.0;

void updateStepTick();
void updateSupportInitialState();
void calcFootstepQueue();
void getZmpTrajectory();
void getFootTrajectory();
void getComTrajectory();
void updateFootPoseFromContactWrench();
void updateMomentControl(double &error,
                         double &error_pre,
                         double &input,
                         double measurement,
                         double reference,
                         double kp,
                         double kd,
                         double damping,
                         double hz)
{
    error_pre = error;
    error     = reference - measurement;
    double error_dot = (error - error_pre) * hz;

    double input_dot = kp * error + kd * error_dot - damping * input;
    input += input_dot / hz;
}

void footstepOptimizer();
CQuadraticProgram QP_stepping;

void mapSupportToBase();

Eigen::Vector2d footstep_des;
Eigen::Vector2d footstep_delta;

const int preview_idx = 4;
Eigen::Vector2d step_command;
std::deque<Eigen::Vector2d> step_queue;
std::deque<double> step_yaw_queue;
std::deque<Eigen::Vector2d> cp_end_queue;
double step_length    = 0.0;
double step_width     = 0.0;
double step_lateral     = 0.0;
double foot_yaw_angle = 0.0;
double foot_height = 0.0;

int step_tick = 0;
double step_time = 0.0;
int step_cnt = 0;
double step_duration = 0.0;
double dsp_duration = 0.0;
double transfer_duration = 0.0;

ContactIndicator support_phase_indicator_;

double com_height = 0.0;
double T = 0.0;

Eigen::MatrixXd A;
Eigen::VectorXd B;
Eigen::MatrixXd C;
Eigen::MatrixXd Gi;
Eigen::VectorXd Gd;
Eigen::MatrixXd Gx;
Eigen::VectorXd zmp_x_traj, zmp_y_traj;

Eigen::Vector3d com_x_dx_ddx;
Eigen::Vector3d com_y_dy_ddy;

Eigen::Vector6d lfoot_contact_wrench;
Eigen::Vector6d rfoot_contact_wrench;

double cf_error = 0.0, cf_error_pre = 0.0, cf_input = 0.0;
double cm_lfoot_roll_error = 0.0, cm_lfoot_roll_error_pre = 0.0, cm_lfoot_roll_input = 0.0;
double cm_rfoot_roll_error = 0.0, cm_rfoot_roll_error_pre = 0.0, cm_rfoot_roll_input = 0.0;
double cm_lfoot_pitch_error = 0.0, cm_lfoot_pitch_error_pre = 0.0, cm_lfoot_pitch_input = 0.0;
double cm_rfoot_pitch_error = 0.0, cm_rfoot_pitch_error_pre = 0.0, cm_rfoot_pitch_input = 0.0;

bool local_LF_contact = true;
bool local_RF_contact = true;

bool is_support_transition = false;
bool is_phase_indicator_transition = false;
bool is_footstep_update = false;
bool is_zmp_update = true;
bool is_preview_transition = false;
};