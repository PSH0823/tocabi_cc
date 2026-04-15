#pragma once
#include <string>
#include <Eigen/Dense>

#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include "utils.h"
#include "walking_manager.h"
#include "teleop_manager.h"

class TaskManager
{
public:
  TaskManager(RobotData &rd);

  void runTestMotion(const TaskMotionType& motion_mode);
  void mapBaseToSupport();

  //--- Class Setter
  void setControlFrequency(double &hz);
  void setTrajectoryDuration(double &traj_time_);
  void setPelvisDistance(double &pelv_dist_);
  void setHandDistance(double &hand_dist_);
  void setStepStride(double step_length_);
  void setStepLateral(double step_lateral_);
  void setStepYaw(double step_yaw_);
  void setFootHeight(double &foot_height_);

  void setStepDuration(double &step_duration_);
  void setDspDuration(double &dsp_duration_);

private:
  void movePelvHandPose();
  void moveTaichiMotion();
  void bipedalWalkingController();
  void teleOperationController();


private:
  RobotData &rd_;
  double hz_ = 2000.0;

  Eigen::Vector3d base_pos;
  Eigen::Matrix3d base_rot;
  ContactIndicator support_phase_indicator_ = ContactIndicator::DoubleSupport;

  double traj_time = 0.0;
  double pelv_dist = 0.0;
  double hand_dist = 0.0;
  double step_length = 0.0;
  double step_lateral = 0.0;
  double step_yaw = 0.0;
  double foot_height = 0.0;
  double step_duration = 0.0;
  double dsp_duration = 0.0;
};
