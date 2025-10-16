#include "collision_manager.h"

using namespace TOCABI;

CollisionManager::CollisionManager(const RobotData &rd) : rd_cm_(rd)
{
    aruco_pose_pub_ = nh_cm_.advertise<geometry_msgs::PoseStamped>("/tocabi_cc/aruco_pose", 10);

    // initialize HPP-FCL collision objects for the robot links
    col_obj_robot_links_.resize(Col_Obj_Count);
    col_obj_robot_links_[Left_Hand_Col_ID] = assignSphereCollisionObject(rd_cm_.link_[Left_Hand].rotm, rd_cm_.link_[Left_Hand].xpos, 0.07);
    col_obj_robot_links_[Right_Hand_Col_ID] = assignSphereCollisionObject(rd_cm_.link_[Right_Hand].rotm, rd_cm_.link_[Right_Hand].xpos, 0.07);
    col_obj_robot_links_[Body_Col_ID] = assignBoxCollisionObject(rd_cm_.link_[Pelvis].rotm, rd_cm_.link_[Pelvis].xpos, 0.15, 0.15, 0.27);    
}

//=========================================== HPP-FCL ============================================//

std::shared_ptr<hpp::fcl::CollisionObject> CollisionManager::assignSphereCollisionObject(const Eigen::Matrix3d obj_rot, const Eigen::Vector3d obj_trans, const double radius)
{
    // Create geometry
    // The type of geometry should be shared pointer since it is required by CollisionObject
    auto sphere = std::make_shared<hpp::fcl::Sphere>(radius);

    // Build transform
    hpp::fcl::Transform3f obj_tf(obj_rot, obj_trans);

    // Create collision object with geometry + transform
    auto col_obj = std::make_shared<hpp::fcl::CollisionObject>(sphere, obj_tf);

    return col_obj;
}

std::shared_ptr<hpp::fcl::CollisionObject> CollisionManager::assignCapsuleCollisionObject(const Eigen::Matrix3d obj_rot, const Eigen::Vector3d obj_trans, const double radius, const double height)
{
    // Create geometry
    // The type of geometry should be shared pointer since it is required by CollisionObject
    auto capsule = std::make_shared<hpp::fcl::Capsule>(radius, height);

    // Build transform
    hpp::fcl::Transform3f obj_tf(obj_rot, obj_trans);

    // Create collision object with geometry + transform
    auto col_obj = std::make_shared<hpp::fcl::CollisionObject>(capsule, obj_tf);

    return col_obj;
}

std::shared_ptr<hpp::fcl::CollisionObject> CollisionManager::assignBoxCollisionObject(const Eigen::Matrix3d obj_rot, const Eigen::Vector3d obj_trans, const double size_x, const double size_y, const double size_z)
{
    // Create geometry
    // The type of geometry should be shared pointer since it is required by CollisionObject
    auto box = std::make_shared<hpp::fcl::Box>(size_x, size_y, size_z);

    // Build transform
    hpp::fcl::Transform3f obj_tf(obj_rot, obj_trans);

    // Create collision object with geometry + transform
    auto col_obj = std::make_shared<hpp::fcl::CollisionObject>(box, obj_tf);

    return col_obj;
}

void CollisionManager::updateRobotCollisionObjectsTransforms()
{
    col_obj_robot_links_[Left_Hand_Col_ID]->setTransform(hpp::fcl::Transform3f(rd_cm_.link_[Left_Hand].rotm, rd_cm_.link_[Left_Hand].xpos));
    col_obj_robot_links_[Right_Hand_Col_ID]->setTransform(hpp::fcl::Transform3f(rd_cm_.link_[Right_Hand].rotm, rd_cm_.link_[Right_Hand].xpos));
}

hpp::fcl::DistanceResult CollisionManager::getDistanceResultBetweenObjects(const int col_obj_idx1, const int col_obj_idx2, DistanceResultOption option)
{
    hpp::fcl::DistanceRequest col_request;
    col_request.enable_nearest_points = true;

    hpp::fcl::DistanceResult col_result;
    if(option == Links) {
        distance(col_obj_robot_links_[col_obj_idx1].get(), col_obj_robot_links_[col_obj_idx2].get(), col_request, col_result);
    }
    else if(option == Link2Env) {
        distance(col_obj_robot_links_[col_obj_idx1].get(), col_obj_obstacles_[col_obj_idx2].get(), col_request, col_result);
    }
    else {
        distance(col_obj_obstacles_[col_obj_idx1].get(), col_obj_obstacles_[col_obj_idx2].get(), col_request, col_result);
    }

    // For debugging/verification: print minimum distance and closest points
    // cout << "Minimum distance: " << col_result.min_distance << endl;
    // cout << "Closest point on object 1: " << col_result.nearest_points[0].transpose() << endl;
    // cout << "Closest point on object 2: " << col_result.nearest_points[1].transpose() << endl;

    return col_result;
}

//________________________________________________________________________________________________//

//================================== Communication with Camera ===================================//

void CollisionManager::pubBasetoHeadTransform()
{
    // ts_base_to_head stores the transform data between base_link and head_link
    base_to_head_tf_msg_.header.stamp = ros::Time::now();

    // frame id
    base_to_head_tf_msg_.header.frame_id = "base_link";
    base_to_head_tf_msg_.child_frame_id = "head_link";

    // translation
    base_to_head_tf_msg_.transform.translation.x = base_to_head_transform_.translation()(0);
    base_to_head_tf_msg_.transform.translation.y = base_to_head_transform_.translation()(1);
    base_to_head_tf_msg_.transform.translation.z = base_to_head_transform_.translation()(2);

    // rotation matrix -> quaternion
    Eigen::Quaterniond quat_head(base_to_head_transform_.rotation());
    base_to_head_tf_msg_.transform.rotation.x = quat_head.x();
    base_to_head_tf_msg_.transform.rotation.y = quat_head.y();
    base_to_head_tf_msg_.transform.rotation.z = quat_head.z();
    base_to_head_tf_msg_.transform.rotation.w = quat_head.w();

    // tf_broadcaster publishes the transform data
    tf_broadcaster_.sendTransform(base_to_head_tf_msg_);
}

Eigen::Isometry3d CollisionManager::getBasetoQRTransform()
{
    Eigen::Isometry3d base_to_qr_transform;

    try
    {
        // listener looks for the transform data between base_link and qr_code(object_frame)
        base_to_qr_tf_msg_ = tf_buffer_.lookupTransform("base_link", "object_frame", ros::Time(0));

        // store the translation data
        base_to_qr_transform.translation() << base_to_qr_tf_msg_.transform.translation.x,
                                              base_to_qr_tf_msg_.transform.translation.y,
                                              base_to_qr_tf_msg_.transform.translation.z;
        // store the rotation data (quaternion -> rotation matrix)
        Eigen::Quaterniond quat_qr(base_to_qr_tf_msg_.transform.rotation.w,
                                   base_to_qr_tf_msg_.transform.rotation.x,
                                   base_to_qr_tf_msg_.transform.rotation.y,
                                   base_to_qr_tf_msg_.transform.rotation.z);
        base_to_qr_transform.linear() = quat_qr.toRotationMatrix();
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("Failed to lookup transform form base_link to object_frame: %s", ex.what());
        base_to_qr_transform.setIdentity();
    }
    
    return base_to_qr_transform;
}

void CollisionManager::getBasetoHeadTransform(){
    global_to_base_rot_yaw_only_ = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rd_cm_.link_[Pelvis].rotm)(2));
    global_to_base_trans_ = rd_cm_.link_[Pelvis].xpos;

    base_to_head_transform_.linear() = global_to_base_rot_yaw_only_.transpose() * rd_cm_.link_[Head].rotm;
    base_to_head_transform_.translation() = global_to_base_rot_yaw_only_.transpose() * (rd_cm_.link_[Head].xpos - rd_cm_.link_[Pelvis].xpos);
}

//________________________________________________________________________________________________//

//====================================== Obstacle in MuJoCo ======================================//

void CollisionManager::pubQRObstaclePose(const int sim_tick, const double hz){
    // ts_QR stores the position and orientation data of the QR obstacle in MuJoCo environment
    aruco_pose_msg_.header.stamp = ros::Time::now();

    // translation
    aruco_pose_msg_.pose.position.x = 0.7;
    aruco_pose_msg_.pose.position.y = 0.5 + 0.2 * sin(M_PI * (sim_tick/hz));
    aruco_pose_msg_.pose.position.z = 1.5;

    // orientation (Quaternion)
    aruco_pose_msg_.pose.orientation.x = 0.5;
    aruco_pose_msg_.pose.orientation.y = -0.5;
    aruco_pose_msg_.pose.orientation.z = -0.5;
    aruco_pose_msg_.pose.orientation.w = 0.5;

    // tf_broadcaster publishes the transform data
    aruco_pose_pub_.publish(aruco_pose_msg_);
}

//________________________________________________________________________________________________//