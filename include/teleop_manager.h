#pragma once
#include <Eigen/Dense>

#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include "utils.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <eigen_conversions/eigen_msg.h>

class TeleOperationManager 
{
public:
    TeleOperationManager(RobotData &rd);
    void callAvailableQueue();
    void setHumanParameterFromTrackers();
    void setRobotParameterFromModel();
    void motionRetargeting();
        void pelvisRetargeting();
        void comRetargeting();
        void handRetargeting();
        void footRetargeting();

private:
    RobotData &rd_;
    ros::NodeHandle nh_;
    ros::CallbackQueue queue_;

    ros::Subscriber tracker_pose_sub;

    void TrackerPoseCallback(const geometry_msgs::PoseArray::ConstPtr &msg);

private:
    Eigen::Isometry3d tracker_head_pose_raw_;
    Eigen::Isometry3d tracker_chest_pose_raw_;
    Eigen::Isometry3d tracker_pelv_pose_raw_;
    Eigen::Isometry3d tracker_lshoulder_pose_raw_;
    Eigen::Isometry3d tracker_lhand_pose_raw_;
    Eigen::Isometry3d tracker_rshoulder_pose_raw_;
    Eigen::Isometry3d tracker_rhand_pose_raw_;
    Eigen::Isometry3d tracker_lupperarm_pose_raw_;
    Eigen::Isometry3d tracker_rupperarm_pose_raw_;

    Eigen::Isometry3d robot_lshoulder_pose_;
    Eigen::Isometry3d robot_rshoulder_pose_;
    Eigen::Isometry3d robot_lhand_pose_target_;
    Eigen::Isometry3d robot_rhand_pose_target_;

    double dist_btw_chest_head = 0.0;
    double dist_btw_shoulder_head = 0.0;
    double dist_btw_shoulder_hand_human = 0.0;

    double pelvis_height = 0.0;
    double dist_btw_shoulder_hand_robot = 0.0;

    double ratio_arm = 0.0;

};