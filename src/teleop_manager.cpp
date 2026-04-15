#include "teleop_manager.h"

TeleOperationManager::TeleOperationManager(RobotData &rd) : rd_(rd)
{
    nh_.setCallbackQueue(&queue_);

    tracker_pose_sub = nh_.subscribe(
        "/tracker_pose",
        1,
        &TeleOperationManager::TrackerPoseCallback, 
        this,
        ros::TransportHints().tcpNoDelay(true)
    );
}

void TeleOperationManager::callAvailableQueue()
{
    queue_.callAvailable(ros::WallDuration());
}

void TeleOperationManager::setHumanParameterFromTrackers()
{
    dist_btw_chest_head = (tracker_head_pose_raw_.translation() - tracker_chest_pose_raw_.translation()).norm();
    dist_btw_shoulder_head = dist_btw_chest_head / 1.5;
    dist_btw_shoulder_hand_human = ((tracker_rshoulder_pose_raw_.translation() - tracker_rhand_pose_raw_.translation()).norm() + (tracker_lshoulder_pose_raw_.translation() - tracker_lhand_pose_raw_.translation()).norm()) / 2.0;

    std::cout << "==========================" << std::endl;
    std::cout << "dist_btw_chest_head : "    << dist_btw_chest_head << std::endl;
    std::cout << "dist_btw_shoulder_head : " << dist_btw_shoulder_head << std::endl;
    std::cout << "dist_btw_shoulder_hand_human : " << dist_btw_shoulder_hand_human << std::endl;
}

void TeleOperationManager::setRobotParameterFromModel()
{
    pelvis_height = rd_.link_[Pelvis].xpos(2);
    dist_btw_shoulder_hand_robot = ((rd_.link_[Left_Hand].xpos - rd_.link_[Left_Hand - 5].xpos).norm() + (rd_.link_[Right_Hand].xpos - rd_.link_[Right_Hand - 5].xpos).norm()) / 2.0;

    ratio_arm = dist_btw_shoulder_hand_robot / dist_btw_shoulder_hand_human;

    std::cout << "==========================" << std::endl;
    std::cout << "pelvis_height : "    << pelvis_height << std::endl;
    std::cout << "dist_btw_shoulder_hand_robot : " << dist_btw_shoulder_hand_robot << std::endl;
    std::cout << "ratio_arm : " << ratio_arm << std::endl;
}

void TeleOperationManager::TrackerPoseCallback(const geometry_msgs::PoseArray::ConstPtr &msg)
{
    // Get TF data w.r.t world frame
    tf::poseMsgToEigen(msg->poses[0],tracker_pelv_pose_raw_);

    tracker_pelv_pose_raw_.linear() = tracker_pelv_pose_raw_.linear() * DyrosMath::rotateWithZ(M_PI); // tracker is behind the chair

    tf::poseMsgToEigen(msg->poses[1],tracker_chest_pose_raw_);
    tf::poseMsgToEigen(msg->poses[2],tracker_lupperarm_pose_raw_);
    tf::poseMsgToEigen(msg->poses[3],tracker_lhand_pose_raw_);
    tf::poseMsgToEigen(msg->poses[4],tracker_rupperarm_pose_raw_);
    tf::poseMsgToEigen(msg->poses[5],tracker_rhand_pose_raw_);
    tf::poseMsgToEigen(msg->poses[6],tracker_head_pose_raw_);
}

void TeleOperationManager::motionRetargeting()
{
    pelvisRetargeting();
    comRetargeting();
    footRetargeting();
    handRetargeting();

    rd_.link_[Left_Hand].x_traj  = robot_lhand_pose_target_.translation();
    rd_.link_[Left_Hand].r_traj  = robot_lhand_pose_target_.linear();
    rd_.link_[Right_Hand].x_traj = robot_rhand_pose_target_.translation();
    rd_.link_[Right_Hand].r_traj = robot_rhand_pose_target_.linear();
}

void TeleOperationManager::pelvisRetargeting()
{
    // TODO
}

void TeleOperationManager::comRetargeting()
{
    // TODO
    rd_.link_[COM_id].x_traj = rd_.link_[COM_id].support_xpos_init;
    rd_.link_[COM_id].x_traj = rd_.link_[COM_id].x_traj - rd_.link_[Pelvis].support_xpos;
}

void TeleOperationManager::footRetargeting()
{
    // TODO
    rd_.link_[Left_Foot].x_traj  = rd_.link_[Left_Foot].support_xpos_init;
    rd_.link_[Right_Foot].x_traj = rd_.link_[Right_Foot].support_xpos_init;

    rd_.link_[Left_Foot].x_traj = rd_.link_[Left_Foot].x_traj - rd_.link_[Pelvis].support_xpos;
    rd_.link_[Right_Foot].x_traj = rd_.link_[Right_Foot].x_traj - rd_.link_[Pelvis].support_xpos;
}

void TeleOperationManager::handRetargeting()
{
    // Remap to World frame to Pelvis frame
    tracker_chest_pose_raw_     = DyrosMath::inverseIsometry3d(tracker_pelv_pose_raw_) * tracker_chest_pose_raw_; 
    tracker_lupperarm_pose_raw_ = DyrosMath::inverseIsometry3d(tracker_pelv_pose_raw_) * tracker_lupperarm_pose_raw_; 
    tracker_lhand_pose_raw_     = DyrosMath::inverseIsometry3d(tracker_pelv_pose_raw_) * tracker_lhand_pose_raw_; 
    tracker_rupperarm_pose_raw_ = DyrosMath::inverseIsometry3d(tracker_pelv_pose_raw_) * tracker_rupperarm_pose_raw_; 
    tracker_rhand_pose_raw_     = DyrosMath::inverseIsometry3d(tracker_pelv_pose_raw_) * tracker_rhand_pose_raw_; 
    tracker_head_pose_raw_      = DyrosMath::inverseIsometry3d(tracker_pelv_pose_raw_) * tracker_head_pose_raw_; 
    tracker_pelv_pose_raw_      = DyrosMath::inverseIsometry3d(tracker_pelv_pose_raw_) * tracker_pelv_pose_raw_; 

    // Shoulder Retargeting
    tracker_lshoulder_pose_raw_.translation()(0) = tracker_chest_pose_raw_.translation()(0);
    tracker_lshoulder_pose_raw_.translation()(1) = tracker_chest_pose_raw_.translation()(1) + dist_btw_shoulder_head;
    tracker_lshoulder_pose_raw_.translation()(2) = tracker_chest_pose_raw_.translation()(2) + (tracker_head_pose_raw_.translation()(2) - tracker_chest_pose_raw_.translation()(2)) / 2.0;

    tracker_rshoulder_pose_raw_.translation()(0) = tracker_chest_pose_raw_.translation()(0);
    tracker_rshoulder_pose_raw_.translation()(1) = tracker_chest_pose_raw_.translation()(1) - dist_btw_shoulder_head;
    tracker_rshoulder_pose_raw_.translation()(2) = tracker_chest_pose_raw_.translation()(2) + (tracker_head_pose_raw_.translation()(2) - tracker_chest_pose_raw_.translation()(2)) / 2.0;

    // Target Hand Pose
    robot_lshoulder_pose_.translation() = rd_.link_[Left_Hand - 5].local_xpos;
    robot_rshoulder_pose_.translation() = rd_.link_[Right_Hand - 5].local_xpos;
    robot_lshoulder_pose_.linear() = rd_.link_[Left_Hand - 5].local_rotm;
    robot_rshoulder_pose_.linear() = rd_.link_[Right_Hand - 5].local_rotm;

    robot_lhand_pose_target_.translation() = robot_lshoulder_pose_.translation() + ratio_arm * (tracker_lhand_pose_raw_.translation() - tracker_lshoulder_pose_raw_.translation());
    robot_rhand_pose_target_.translation() = robot_rshoulder_pose_.translation() + ratio_arm * (tracker_rhand_pose_raw_.translation() - tracker_rshoulder_pose_raw_.translation());
    robot_lhand_pose_target_.linear() = tracker_lhand_pose_raw_.linear();
    robot_rhand_pose_target_.linear() = tracker_rhand_pose_raw_.linear();
}
