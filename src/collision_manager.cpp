#include "cbf_manager/collision_manager.h"

using namespace TOCABI;

CollisionManager::CollisionManager(RobotData &rd, RigidBodyDynamics::Model &model) : rd_(rd), model_(model)
{
    nh_cm_.setCallbackQueue(&queue_cm_);

    // Initialize the vector of CollisionBody objects for all robot links
    initCollisionBody();

    // Initialize variables related to self-collision avoidance constraints
    self_collision_avoid_terms_.num_pairs = col_id_pairs_.size();
    self_collision_avoid_terms_.J.setZero(self_collision_avoid_terms_.num_pairs, MODEL_DOF_VIRTUAL);
    self_collision_avoid_terms_.Jdotqdot.setZero(self_collision_avoid_terms_.num_pairs);
    self_collision_avoid_terms_.min_distances.setZero(self_collision_avoid_terms_.num_pairs);

    base_to_head_pose_pub_ = nh_cm_.advertise<geometry_msgs::PoseStamped>("/tocabi_cc/base_to_head", 1);
    aruco_detect_sub_ = nh_cm_.subscribe("/camera/qr_pose", 1, &CollisionManager::BasetoQRTransformCallback, this);
    ros::param::get("/kalman_filter/tracking_duration", tracking_duration);
    ros::param::get("/kalman_filter/process_variance", process_var);
    ros::param::get("/kalman_filter/process_rate_variance", process_rate_var);
    ros::param::get("/kalman_filter/measurement_variance", measurement_var);

    dt = 1.0 / hz_;
    TrackedObstacle::setSamplingTime(dt);
    TrackedObstacle::setCounterSize(static_cast<int>(hz_* tracking_duration));
    TrackedObstacle::setCovariances(process_var, process_rate_var, measurement_var);
}

void CollisionManager::callAvailableQueue()
{
    queue_cm_.callAvailable(ros::WallDuration());
}

//====================================== Collision Related =======================================//

void CollisionManager::pubSelfCollisionStatus()
{
    for(unsigned int id = 0; id < Col_Obj_Count; id++){
        col_status_msg_.data[id] = static_cast<uint8_t>(cb_robot_[id].in_collision);
    }
    col_status_pub_.publish(col_status_msg_);

    // reset collision flags
    std::fill(collision_flags_.begin(), collision_flags_.end(), 0);
}

void CollisionManager::updateRobotCollisionObjectsPose()
{
    for(unsigned int i = Left_Pelvis_Col_ID; i < Col_Obj_Count; i++)
    {
        world_to_base_rotm_yaw_only_ = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm)(2)); 
        world_to_base_trans_ = rd_.link_[Pelvis].xpos;

        // cb_robot_[i].xpos = rd_.link_[cb_robot_[i].link_id].xpos + rd_.link_[cb_robot_[i].link_id].rotm * cb_robot_[i].link_xpos;
        // cb_robot_[i].rotm = rd_.link_[cb_robot_[i].link_id].rotm * cb_robot_[i].link_rotm;

        cb_robot_[i].local_xpos = rd_.link_[cb_robot_[i].link_id].local_xpos + rd_.link_[cb_robot_[i].link_id].local_rotm * cb_robot_[i].link_xpos;
        cb_robot_[i].local_rotm = rd_.link_[cb_robot_[i].link_id].local_rotm * cb_robot_[i].link_rotm;
    }
}


void CollisionManager::computeSelfColAvoidConstraintTerms(const CbfType cbf_mode)
{
    for(unsigned int i = 0; i < self_collision_avoid_terms_.num_pairs; i++)
    {
        CollisionObjectIdx id1 = col_id_pairs_[i].first;
        CollisionObjectIdx id2 = col_id_pairs_[i].second;

        Vector3d nearest_point1, nearest_point2;

        if(cb_robot_[id1].sphere && cb_robot_[id2].sphere){
            ColResultSphere2Sphere(id1, id2, self_collision_avoid_terms_.min_distances(i), nearest_point1, nearest_point2);
        }
        else if(cb_robot_[id1].sphere && cb_robot_[id2].capsule){
            ColResultSphere2Capsule(id1, id2, self_collision_avoid_terms_.min_distances(i), nearest_point1, nearest_point2);
        }
        else if(cb_robot_[id1].capsule && cb_robot_[id2].sphere){
            ColResultSphere2Capsule(id2, id1, self_collision_avoid_terms_.min_distances(i), nearest_point2, nearest_point1);
        }
        else if(cb_robot_[id1].capsule && cb_robot_[id2].capsule){
            ColResultCapsule2Capsule(id1, id2, self_collision_avoid_terms_.min_distances(i), nearest_point1, nearest_point2);
        }

        if (self_collision_avoid_terms_.min_distances(i) < 0.0){
            collision_flags_[id1] += 1;
            collision_flags_[id2] += 1;
        }

        computeSelfColAvoidConstraintTermsRow(id1, nearest_point1,
                                              id2, nearest_point2,
                                              DyrosMath::sign(self_collision_avoid_terms_.min_distances(i)),
                                              self_collision_avoid_terms_.J.row(i),
                                              self_collision_avoid_terms_.Jdotqdot(i),
                                              cbf_mode);
    }

    for(unsigned int id = 0; id < Col_Obj_Count; id++) cb_robot_[id].in_collision = collision_flags_[id] > 0;

}

void CollisionManager::computeObstacleAvoidConstraintTerms(const CbfType cbf_mode)
{
    std::vector<double> min_distances;
    std::vector<RowVectorXd> J_col_rows;
    std::vector<double> Jdotqdot_projections;
    std::vector<double> obstacle_speeds;

    if(!cb_obstacles_.empty()){
        for (unsigned int i = 0; i < cb_obstacles_.size(); i++)
        {
            for(unsigned int id = 0; id < Col_Obj_Count; id++)
            {
                double min_distance;

                Vector3d nearest_point_obj, nearest_point_obs;

                if(cb_robot_[id].sphere){
                    if(cb_obstacles_[i].sphere){
                        ColResultSphere2Sphere(id, i, min_distance, nearest_point_obj, nearest_point_obs, DistanceResultOption::Link2Env);
                    }
                    else if(cb_obstacles_[i].capsule){
                        ColResultSphere2Capsule(id, i, min_distance, nearest_point_obj, nearest_point_obs, DistanceResultOption::Link2Env, false);
                    }
                }
                else if(cb_robot_[id].capsule){
                    if(cb_obstacles_[i].sphere){
                        ColResultSphere2Capsule(i, id, min_distance, nearest_point_obs, nearest_point_obj, DistanceResultOption::Link2Env, true);
                    }
                    else if(cb_obstacles_[i].capsule){
                        ColResultCapsule2Capsule(id, i, min_distance, nearest_point_obj, nearest_point_obs, DistanceResultOption::Link2Env);
                    }
                }

                if((cb_obstacles_[i].sphere && min_distance <= 2 * cb_obstacles_[i].sphere->radius) || 
                   (cb_obstacles_[i].capsule && min_distance <= 2 * cb_obstacles_[i].capsule->radius)){
                    
                    double obs_vel_projection;
                    double Jdotqdot_projection;
                    J_col_rows.push_back(computeObstacleAvoidConstraintTermsRow(id, nearest_point_obj,
                                                                                i, nearest_point_obs,
                                                                                DyrosMath::sign(min_distance),
                                                                                Jdotqdot_projection,
                                                                                obs_vel_projection,
                                                                                cbf_mode));
                    min_distances.push_back(min_distance);
                    Jdotqdot_projections.push_back(Jdotqdot_projection);
                    obstacle_speeds.push_back(obs_vel_projection);
                }
            }
        }
    }

    obstacle_avoid_terms_.num_pairs = J_col_rows.size();

    obstacle_avoid_terms_.J.setZero(obstacle_avoid_terms_.num_pairs, MODEL_DOF_VIRTUAL);
    for(int i = 0; i < obstacle_avoid_terms_.num_pairs; i++) obstacle_avoid_terms_.J.row(i) = J_col_rows[i];
    if(min_distances.empty()){
        obstacle_avoid_terms_.min_distances.resize(0);
        obstacle_avoid_terms_.obs_vel_projections.resize(0);
        obstacle_avoid_terms_.Jdotqdot.resize(0);
    } else {
        obstacle_avoid_terms_.min_distances = Eigen::Map<VectorXd>(min_distances.data(), obstacle_avoid_terms_.num_pairs);
        obstacle_avoid_terms_.obs_vel_projections = Eigen::Map<VectorXd>(obstacle_speeds.data(), obstacle_avoid_terms_.num_pairs);
        obstacle_avoid_terms_.Jdotqdot = Eigen::Map<VectorXd>(Jdotqdot_projections.data(), obstacle_avoid_terms_.num_pairs);
    }
}

void CollisionManager::checkSelfCollision()
{
    for(unsigned int i = 0; i < self_collision_avoid_terms_.num_pairs; i++)
    {
        CollisionObjectIdx id1 = col_id_pairs_[i].first;
        CollisionObjectIdx id2 = col_id_pairs_[i].second;
        
        Vector3d nearest_point1, nearest_point2;

        if(cb_robot_[id1].sphere && cb_robot_[id2].sphere){
            ColResultSphere2Sphere(id1, id2, self_collision_avoid_terms_.min_distances(i), nearest_point1, nearest_point2);
        }
        else if(cb_robot_[id1].sphere && cb_robot_[id2].capsule){
            ColResultSphere2Capsule(id1, id2, self_collision_avoid_terms_.min_distances(i), nearest_point1, nearest_point2);
        }
        else if(cb_robot_[id1].capsule && cb_robot_[id2].sphere){
            ColResultSphere2Capsule(id2, id1, self_collision_avoid_terms_.min_distances(i), nearest_point2, nearest_point1);
        }
        else if(cb_robot_[id1].capsule && cb_robot_[id2].capsule){
            ColResultCapsule2Capsule(id1, id2, self_collision_avoid_terms_.min_distances(i), nearest_point1, nearest_point2);
        }
        
        if (self_collision_avoid_terms_.min_distances(i) < 0.0){
            collision_flags_[id1] += 1;
            collision_flags_[id2] += 1;
        }

        // if(id1 == Right_Pelvis_Col_ID && id2 == Right_ForeArm_Col_ID){
        //     cout << "nearest 1: " << (cb_robot_[id1].rotm.transpose() * (nearest_point1 - cb_robot_[id1].xpos)).transpose() << endl;
        //     cout << "nearest 2: " << (cb_robot_[id2].rotm.transpose() * (nearest_point2 - cb_robot_[id2].xpos)).transpose() << endl;
        //     cout << "min_distance: " << self_collision_avoid_terms_.min_distances(i) << endl;
        //     cout << "-------------------------------" << endl;
        // }
    }

    for(unsigned int id = 0; id < Col_Obj_Count; id++) cb_robot_[id].in_collision = collision_flags_[id] > 0;

}

void CollisionManager::initCollisionBody()
{
    cb_robot_.resize(Col_Obj_Count);

    // Left Pelvis
    cb_robot_[Left_Pelvis_Col_ID].link_name = "Pelvis_Link";
    cb_robot_[Left_Pelvis_Col_ID].link_id = TOCABI::Pelvis;
    cb_robot_[Left_Pelvis_Col_ID].link_xpos  = Eigen::Vector3d(0.05, 0.02, 0.1);
    cb_robot_[Left_Pelvis_Col_ID].link_rotm = Eigen::Matrix3d::Identity();
    cb_robot_[Left_Pelvis_Col_ID].capsule = std::make_shared<Capsule>(0.18, 0.05); // radius, height

    // Right Pelvis
    cb_robot_[Right_Pelvis_Col_ID].link_name = "Pelvis_Link";
    cb_robot_[Right_Pelvis_Col_ID].link_id = TOCABI::Pelvis;
    cb_robot_[Right_Pelvis_Col_ID].link_xpos  = Eigen::Vector3d(0.05, -0.02, 0.1);
    cb_robot_[Right_Pelvis_Col_ID].link_rotm = Eigen::Matrix3d::Identity();
    cb_robot_[Right_Pelvis_Col_ID].capsule = std::make_shared<Capsule>(0.18, 0.05); // radius, height

    // Left Upper Leg
    cb_robot_[Left_Upper_Leg_Col_ID].link_name = "L_Thigh_Link";
    cb_robot_[Left_Upper_Leg_Col_ID].link_id = TOCABI::Left_Thigh;
    cb_robot_[Left_Upper_Leg_Col_ID].link_xpos = Eigen::Vector3d(0.0, 0.0, -0.1323);
    cb_robot_[Left_Upper_Leg_Col_ID].link_rotm = Eigen::Matrix3d::Identity();
    cb_robot_[Left_Upper_Leg_Col_ID].capsule = std::make_shared<Capsule>(0.075, 0.34); // radius, height

    // Left Lower Leg
    cb_robot_[Left_Lower_Leg_Col_ID].link_name = "L_Knee_Link";
    cb_robot_[Left_Lower_Leg_Col_ID].link_id = TOCABI::Left_Knee;
    cb_robot_[Left_Lower_Leg_Col_ID].link_xpos = Eigen::Vector3d(0.0, 0.0, -0.23);
    cb_robot_[Left_Lower_Leg_Col_ID].link_rotm = Eigen::Matrix3d::Identity();
    cb_robot_[Left_Lower_Leg_Col_ID].capsule = std::make_shared<Capsule>(0.075, 0.3); // radius, height

    // Left Inner Foot
    cb_robot_[Left_Inner_Foot_Col_ID].link_name = "L_Foot_Link";
    cb_robot_[Left_Inner_Foot_Col_ID].link_id = TOCABI::Left_Foot;
    cb_robot_[Left_Inner_Foot_Col_ID].link_xpos = Eigen::Vector3d(0.03, -0.0455, -0.15);
    cb_robot_[Left_Inner_Foot_Col_ID].link_rotm = DyrosMath::rotateWithY(-M_PI / 2.0);
    cb_robot_[Left_Inner_Foot_Col_ID].capsule = std::make_shared<Capsule>(0.0432, 0.24); // radius, height

    // Left Outer Foot
    cb_robot_[Left_Outer_Foot_Col_ID].link_name = "L_Foot_Link";
    cb_robot_[Left_Outer_Foot_Col_ID].link_id = TOCABI::Left_Foot;
    cb_robot_[Left_Outer_Foot_Col_ID].link_xpos = Eigen::Vector3d(0.03, 0.0455, -0.15);
    cb_robot_[Left_Outer_Foot_Col_ID].link_rotm = DyrosMath::rotateWithY(-M_PI / 2.0);
    cb_robot_[Left_Outer_Foot_Col_ID].capsule = std::make_shared<Capsule>(0.0432, 0.24); // radius, height

    // Right Upper Leg
    cb_robot_[Right_Upper_Leg_Col_ID].link_name = "R_Thigh_Link";
    cb_robot_[Right_Upper_Leg_Col_ID].link_id = TOCABI::Right_Thigh;
    cb_robot_[Right_Upper_Leg_Col_ID].link_xpos = Eigen::Vector3d(0.0, 0.0, -0.1323);
    cb_robot_[Right_Upper_Leg_Col_ID].link_rotm = Eigen::Matrix3d::Identity();
    cb_robot_[Right_Upper_Leg_Col_ID].capsule = std::make_shared<Capsule>(0.075, 0.34); // radius, height

    // Right Lower Leg
    cb_robot_[Right_Lower_Leg_Col_ID].link_name = "R_Knee_Link";
    cb_robot_[Right_Lower_Leg_Col_ID].link_id = TOCABI::Right_Knee;
    cb_robot_[Right_Lower_Leg_Col_ID].link_xpos = Eigen::Vector3d(0.0, 0.0, -0.23);
    cb_robot_[Right_Lower_Leg_Col_ID].link_rotm = Eigen::Matrix3d::Identity();
    cb_robot_[Right_Lower_Leg_Col_ID].capsule = std::make_shared<Capsule>(0.075, 0.30); // radius, height

    // Right Inner Foot
    cb_robot_[Right_Inner_Foot_Col_ID].link_name = "R_Foot_Link";
    cb_robot_[Right_Inner_Foot_Col_ID].link_id = TOCABI::Right_Foot;
    cb_robot_[Right_Inner_Foot_Col_ID].link_xpos = Eigen::Vector3d(0.03, 0.0455, -0.15);
    cb_robot_[Right_Inner_Foot_Col_ID].link_rotm = DyrosMath::rotateWithY(-M_PI / 2.0);
    cb_robot_[Right_Inner_Foot_Col_ID].capsule = std::make_shared<Capsule>(0.0432, 0.24); // radius, height

    // Right Outer Foot
    cb_robot_[Right_Outer_Foot_Col_ID].link_name = "R_Foot_Link";
    cb_robot_[Right_Outer_Foot_Col_ID].link_id = TOCABI::Right_Foot;
    cb_robot_[Right_Outer_Foot_Col_ID].link_xpos = Eigen::Vector3d(0.03, -0.0455, -0.15);
    cb_robot_[Right_Outer_Foot_Col_ID].link_rotm = DyrosMath::rotateWithY(-M_PI / 2.0);
    cb_robot_[Right_Outer_Foot_Col_ID].capsule = std::make_shared<Capsule>(0.0432, 0.24); // radius, height

    // // Left Upper Body
    // cb_robot_[Left_Upper_Body_Col_ID].link_name = "Upperbody_Link";
    // cb_robot_[Left_Upper_Body_Col_ID].link_id = TOCABI::Upper_Body;
    // cb_robot_[Left_Upper_Body_Col_ID].link_xpos = Eigen::Vector3d(0.023, 0.154, 0.19);
    // cb_robot_[Left_Upper_Body_Col_ID].link_rotm = Eigen::Matrix3d::Identity();
    // cb_robot_[Left_Upper_Body_Col_ID].capsule = std::make_shared<Capsule>(0.06, 0.03); // radius, height

    // // Right Upper Body
    // cb_robot_[Right_Upper_Body_Col_ID].link_name = "Upperbody_Link";
    // cb_robot_[Right_Upper_Body_Col_ID].link_id = TOCABI::Upper_Body;
    // cb_robot_[Right_Upper_Body_Col_ID].link_xpos = Eigen::Vector3d(0.023, -0.154, 0.19);
    // cb_robot_[Right_Upper_Body_Col_ID].link_rotm = Eigen::Matrix3d::Identity();
    // cb_robot_[Right_Upper_Body_Col_ID].capsule = std::make_shared<Capsule>(0.06, 0.03); // radius, height

    // Left Upper Arm
    cb_robot_[Left_Upper_Arm_Col_ID].link_name = "L_Armlink_Link";
    cb_robot_[Left_Upper_Arm_Col_ID].link_id = TOCABI::Left_Upper_Arm;
    cb_robot_[Left_Upper_Arm_Col_ID].link_xpos = Eigen::Vector3d(0.005, 0.12, -0.01);
    cb_robot_[Left_Upper_Arm_Col_ID].link_rotm = DyrosMath::rotateWithX(75 * DEG2RAD);
    cb_robot_[Left_Upper_Arm_Col_ID].capsule = std::make_shared<Capsule>(0.062, 0.2); // radius, height

    // Left ForeArm
    cb_robot_[Left_ForeArm_Col_ID].link_name = "L_Forearm_Link";
    cb_robot_[Left_ForeArm_Col_ID].link_id = TOCABI::Left_Forearm;
    cb_robot_[Left_ForeArm_Col_ID].link_xpos = Eigen::Vector3d(0.0, 0.05, 0.0);
    cb_robot_[Left_ForeArm_Col_ID].link_rotm = DyrosMath::rotateWithX(M_PI / 2.0);
    cb_robot_[Left_ForeArm_Col_ID].capsule = std::make_shared<Capsule>(0.055, 0.2); // radius, height

    // Left Hand
    cb_robot_[Left_Hand_Col_ID].link_name = "L_Wrist2_Link";
    cb_robot_[Left_Hand_Col_ID].link_id = TOCABI::Left_Hand;
    cb_robot_[Left_Hand_Col_ID].link_xpos = Eigen::Vector3d(0.0, 0.0, 0.0);
    cb_robot_[Left_Hand_Col_ID].link_rotm = Eigen::Matrix3d::Identity();
    cb_robot_[Left_Hand_Col_ID].sphere = std::make_shared<Sphere>(0.06); // radius

    // Head
    cb_robot_[Head_Col_ID].link_name = "Head_Link";
    cb_robot_[Head_Col_ID].link_id = TOCABI::Head;
    cb_robot_[Head_Col_ID].link_xpos = Eigen::Vector3d(0.035, 0.0, 0.0735);
    cb_robot_[Head_Col_ID].link_rotm = Eigen::Matrix3d::Identity();
    cb_robot_[Head_Col_ID].sphere = std::make_shared<Sphere>(0.142); // radius

    // Right Upper Arm
    cb_robot_[Right_Upper_Arm_Col_ID].link_name = "R_Armlink_Link";
    cb_robot_[Right_Upper_Arm_Col_ID].link_id = TOCABI::Right_Upper_Arm;
    cb_robot_[Right_Upper_Arm_Col_ID].link_xpos = Eigen::Vector3d(0.005, -0.12, -0.01);
    cb_robot_[Right_Upper_Arm_Col_ID].link_rotm = DyrosMath::rotateWithX(-75 * DEG2RAD);
    cb_robot_[Right_Upper_Arm_Col_ID].capsule = std::make_shared<Capsule>(0.062, 0.2); // radius, height

    // Right ForeArm
    cb_robot_[Right_ForeArm_Col_ID].link_name = "R_Forearm_Link";
    cb_robot_[Right_ForeArm_Col_ID].link_id = TOCABI::Right_Forearm;
    cb_robot_[Right_ForeArm_Col_ID].link_xpos = Eigen::Vector3d(0.0, -0.05, 0.0);
    cb_robot_[Right_ForeArm_Col_ID].link_rotm = DyrosMath::rotateWithX(M_PI / 2.0);
    cb_robot_[Right_ForeArm_Col_ID].capsule = std::make_shared<Capsule>(0.055, 0.2); // radius, height

    // Right Hand
    cb_robot_[Right_Hand_Col_ID].link_name = "R_Wrist2_Link";
    cb_robot_[Right_Hand_Col_ID].link_id = TOCABI::Right_Hand;
    cb_robot_[Right_Hand_Col_ID].link_xpos = Eigen::Vector3d(0.0, 0.0, 0.0);
    cb_robot_[Right_Hand_Col_ID].link_rotm = Eigen::Matrix3d::Identity();
    cb_robot_[Right_Hand_Col_ID].sphere = std::make_shared<Sphere>(0.06); // radius
}

void CollisionManager::ColResultSphere2Sphere(const unsigned int obj_id1, 
                                              const unsigned int obj_id2, 
                                              double &min_distance, 
                                              Eigen::Vector3d &nearest_point1, 
                                              Eigen::Vector3d &nearest_point2, 
                                              DistanceResultOption option)
{
    // center point: center of sphere
    Vector3d center_point1, center_point2;

    if(option == Links){
        center_point1 = cb_robot_[obj_id1].local_xpos;
        center_point2 = cb_robot_[obj_id2].local_xpos;

        min_distance = (center_point1 - center_point2).norm() - (cb_robot_[obj_id1].sphere->radius + cb_robot_[obj_id2].sphere->radius);

        nearest_point1= center_point1 + (center_point2 - center_point1).normalized() * cb_robot_[obj_id1].sphere->radius;
        nearest_point2= center_point2 + (center_point1 - center_point2).normalized() * cb_robot_[obj_id2].sphere->radius;
    }
    else if(option == Link2Env){
        center_point1 = cb_robot_[obj_id1].local_xpos;
        center_point2 = cb_obstacles_[obj_id2].local_xpos;

        min_distance = (center_point1 - center_point2).norm() - (cb_robot_[obj_id1].sphere->radius + cb_obstacles_[obj_id2].sphere->radius);

        nearest_point1= center_point1 + (center_point2 - center_point1).normalized() * cb_robot_[obj_id1].sphere->radius;
        nearest_point2= center_point2 + (center_point1 - center_point2).normalized() * cb_obstacles_[obj_id2].sphere->radius;
    }
}

void CollisionManager::ColResultSphere2Capsule(const unsigned int obj_id1, 
                                               const unsigned int obj_id2, 
                                               double &min_distance, 
                                               Eigen::Vector3d &nearest_point1, 
                                               Eigen::Vector3d &nearest_point2, 
                                               DistanceResultOption option, 
                                               bool obs_is_sphere)
{
    // center_point1: center of sphere, center_point2: position of the nearest point on the centerline(i.e., the central axis) of capsule 
    Vector3d center_point1, center_point2;
    // p_capsule1, p_capsule2: endpoints of capsule centerline
    Vector3d p_capsule1, p_capsule2;

    if(option == Links){
        center_point1 =  cb_robot_[obj_id1].local_xpos;
        p_capsule1 = cb_robot_[obj_id2].local_xpos + cb_robot_[obj_id2].local_rotm * Eigen::Vector3d(0, 0, cb_robot_[obj_id2].capsule->half_length);
        p_capsule2 = cb_robot_[obj_id2].local_xpos + cb_robot_[obj_id2].local_rotm * Eigen::Vector3d(0, 0, -cb_robot_[obj_id2].capsule->half_length);
    }
    else if(option == Link2Env){
        if(obs_is_sphere){
            center_point1 =  cb_obstacles_[obj_id1].local_xpos;
            p_capsule1 = cb_robot_[obj_id2].local_xpos + cb_robot_[obj_id2].local_rotm * Eigen::Vector3d(0, 0, cb_robot_[obj_id2].capsule->half_length);
            p_capsule2 = cb_robot_[obj_id2].local_xpos + cb_robot_[obj_id2].local_rotm * Eigen::Vector3d(0, 0, -cb_robot_[obj_id2].capsule->half_length);
        }
        else{
            center_point1 =  cb_robot_[obj_id1].local_xpos;
            p_capsule1 = cb_obstacles_[obj_id2].local_xpos + cb_obstacles_[obj_id2].local_rotm * Eigen::Vector3d(0, 0, cb_obstacles_[obj_id2].capsule->half_length);
            p_capsule2 = cb_obstacles_[obj_id2].local_xpos + cb_obstacles_[obj_id2].local_rotm * Eigen::Vector3d(0, 0, -cb_obstacles_[obj_id2].capsule->half_length);
        }
    }

    Vector3d vec_capsule1_to_sphere = nearest_point1 - p_capsule1;
    Vector3d vec_capsule2_to_sphere = nearest_point1 - p_capsule2;
    Vector3d vec_capsule1_to_capsule2 = p_capsule2 - p_capsule1;

    // case 1: the sphere center lies to the left of the capsule’s centerline (before the first endpoint)
    if(vec_capsule1_to_sphere.dot(vec_capsule1_to_capsule2) <= 0){
        center_point2 = p_capsule1;
    }

    // case 2: the sphere center lies to the right of the capsule’s centerline (beyond the second endpoint)
    else if(vec_capsule2_to_sphere.dot(vec_capsule1_to_capsule2) >= 0){
        center_point2 = p_capsule2;
    }

    // case 3: the sphere center lies between the two endpoints of the capsule’s centerline
    else{
        center_point2 = p_capsule1 + (vec_capsule1_to_sphere.dot(vec_capsule1_to_capsule2) / vec_capsule1_to_capsule2.squaredNorm()) * vec_capsule1_to_capsule2;
    }

    if(option == Links){
        min_distance = (center_point1 - center_point2).norm() - (cb_robot_[obj_id1].sphere->radius + cb_robot_[obj_id2].capsule->radius);

        nearest_point1 = center_point1 + (center_point2 - center_point1).normalized() * cb_robot_[obj_id1].sphere->radius;
        nearest_point2 = center_point2 + (center_point1 - center_point2).normalized() * cb_robot_[obj_id2].capsule->radius;
    }
    else if(option == Link2Env){
        if(obs_is_sphere){
            min_distance = (center_point1 - center_point2).norm() - (cb_obstacles_[obj_id1].sphere->radius + cb_robot_[obj_id2].capsule->radius);

            nearest_point1 = center_point1 + (center_point2 - center_point1).normalized() * cb_obstacles_[obj_id1].sphere->radius;
            nearest_point2 = center_point2 + (center_point1 - center_point2).normalized() * cb_robot_[obj_id2].capsule->radius;
        }
        else{
            min_distance = (center_point1 - center_point2).norm() - (cb_robot_[obj_id1].sphere->radius + cb_obstacles_[obj_id2].capsule->radius);

            nearest_point1 = center_point1 + (center_point2 - center_point1).normalized() * cb_robot_[obj_id1].sphere->radius;
            nearest_point2 = center_point2 + (center_point1 - center_point2).normalized() * cb_obstacles_[obj_id2].capsule->radius;
        }
    }
}

void CollisionManager::ColResultCapsule2Capsule(const unsigned int obj_id1, 
                                                const unsigned int obj_id2, 
                                                double &min_distance, 
                                                Eigen::Vector3d &nearest_point1, 
                                                Eigen::Vector3d &nearest_point2, 
                                                DistanceResultOption option)
{
    // reference paper: Vladimir J. Lumelsky, "On fast computation of distance between line segments" (1985)
    // link: https://www.sciencedirect.com/science/article/pii/0020019085900328

    // center_point: position of the nearest point on the centerline(i.e., the central axis) of capsule
    Vector3d center_point1, center_point2;

    // pA, pB: endpoints of capsule 1 centerline
    // pC, pD: endpoints of capsule 2 centerline
    Vector3d pA, pB, pC, pD;

    if(option == Links){
        pA = cb_robot_[obj_id1].local_xpos + cb_robot_[obj_id1].local_rotm * Eigen::Vector3d(0, 0, cb_robot_[obj_id1].capsule->half_length);
        pB = cb_robot_[obj_id1].local_xpos + cb_robot_[obj_id1].local_rotm * Eigen::Vector3d(0, 0, -cb_robot_[obj_id1].capsule->half_length);
        pC = cb_robot_[obj_id2].local_xpos + cb_robot_[obj_id2].local_rotm * Eigen::Vector3d(0, 0, cb_robot_[obj_id2].capsule->half_length);
        pD = cb_robot_[obj_id2].local_xpos + cb_robot_[obj_id2].local_rotm * Eigen::Vector3d(0, 0, -cb_robot_[obj_id2].capsule->half_length);
    }
    else if(option == Link2Env){
        pA = cb_robot_[obj_id1].local_xpos + cb_robot_[obj_id1].local_rotm * Eigen::Vector3d(0, 0, cb_robot_[obj_id1].capsule->half_length);
        pB = cb_robot_[obj_id1].local_xpos + cb_robot_[obj_id1].local_rotm * Eigen::Vector3d(0, 0, -cb_robot_[obj_id1].capsule->half_length);
        pC = cb_obstacles_[obj_id2].local_xpos + cb_obstacles_[obj_id2].local_rotm * Eigen::Vector3d(0, 0, cb_obstacles_[obj_id2].capsule->half_length);
        pD = cb_obstacles_[obj_id2].local_xpos + cb_obstacles_[obj_id2].local_rotm * Eigen::Vector3d(0, 0, -cb_obstacles_[obj_id2].capsule->half_length);
    }

    // center_point1 = pA * (1-t) + pB * t (0 <= t <=1)
    // center_point2 = pC * (1-u) + pD * u (0 <= u <=1)
    double t, u;

    double R, S1, S2, D1, D2;

    // Step 1
    R = (pB - pA).dot(pD - pC);
    S1 = (pB - pA).dot(pC - pA);
    S2 = (pD - pC).dot(pC - pA);
    D1 = (pB - pA).squaredNorm();
    D2 = (pD - pC).squaredNorm();
    
    // parallel case(set t = 0 & directly go to Step 3)
    if((D1 * D2 - pow(R,2)) == 0){
        t = 0;
        // Step 3
        u = - S2 / D2;
        if(u < 0.0){
            u = 0.0;
            // Step 4                          
            t = DyrosMath::minmax_cut((u * R + S1) / D1,
                                      0.0,
                                      1.0
                                      );
        }
        else if (u > 1.0){
            u = 1.0;
            // Step 4                          
            t = DyrosMath::minmax_cut((u * R + S1) / D1,
                                      0.0,
                                      1.0
                                      );
        }
    }

    else{
        // Step 2
        t = DyrosMath::minmax_cut((S1 * D2 - S2 * R) / (D1 * D2 - pow(R,2)),
                                  0.0,
                                  1.0
                                  );
        // Step 3
        u = (t * R - S2) / D2;
        if(u < 0.0){
            u = 0.0;
            // Step 4
            t = DyrosMath::minmax_cut((u * R + S1) / D1,
                                      0.0,
                                      1.0
                                      );
        }
        else if (u > 1.0){
            u = 1.0;
            // Step 4                          
            t = DyrosMath::minmax_cut((u * R + S1) / D1,
                                      0.0,
                                      1.0
                                      );
        }
    }

    // Step 5
    center_point1 = pA * (1 - t) + pB * t;
    center_point2 = pC * (1 - u) + pD * u;

    if(option == Links){
        min_distance = (center_point1 - center_point2).norm() - (cb_robot_[obj_id1].capsule->radius + cb_robot_[obj_id2].capsule->radius);
        
        nearest_point1 = center_point1 + (center_point2 - center_point1).normalized() * cb_robot_[obj_id1].capsule->radius;
        nearest_point2 = center_point2 + (center_point1 - center_point2).normalized() * cb_robot_[obj_id2].capsule->radius;
    }
    else if(option == Link2Env){
        min_distance = (center_point1 - center_point2).norm() - (cb_robot_[obj_id1].capsule->radius + cb_obstacles_[obj_id2].capsule->radius);
        
        nearest_point1 = center_point1 + (center_point2 - center_point1).normalized() * cb_robot_[obj_id1].capsule->radius;
        nearest_point2 = center_point2 + (center_point1 - center_point2).normalized() * cb_obstacles_[obj_id2].capsule->radius;
    }
}

void CollisionManager::computeSelfColAvoidConstraintTermsRow(const unsigned int obj_id1, 
                                                             const Eigen::Vector3d nearest_point1, 
                                                             const unsigned int obj_id2, 
                                                             const Eigen::Vector3d nearest_point2, 
                                                             const int sign,
                                                             Eigen::Ref<Eigen::RowVectorXd, 0, Eigen::Stride<1, Eigen::Dynamic>> J_row,
                                                             double &Jdotqdot_projection,
                                                             const CbfType cbf_mode)
{
    // reference paper: C. Khazoom, D. Gonzalez-Diaz, Y. Ding and S. Kim, "Humanoid Self-Collision Avoidance Using Whole-Body Control with Control Barrier Functions" (Humanoids, 2022)
    // link: https://ieeexplore.ieee.org/abstract/document/10000235

    J_row.setZero();
    Jdotqdot_projection = 0.0;

    // unit vector connecting the two closest points between two collision objects
    Vector3d normal_vec = (nearest_point1 - nearest_point2) / (nearest_point1 - nearest_point2).norm();

    // linear parts of Jacobians from base to trans_joint_to_nearestcenter
    MatrixXd J_col_obj1, J_col_obj2;
    J_col_obj1.setZero(3,MODEL_DOF_VIRTUAL); J_col_obj2.setZero(3,MODEL_DOF_VIRTUAL);

    // first collision object
    // vector stores translation vector in link frame from the joint to the nearest point on the collision object's center (sphere: center or capsule: axis point)
    Vector3d link_xpos_joint_to_nearestcenter1 = rd_.link_[cb_robot_[obj_id1].link_id].local_rotm.transpose() * (nearest_point1 - rd_.link_[cb_robot_[obj_id1].link_id].local_xpos);

    // calcate linear part of Jacobian for the first collision object
    RigidBodyDynamics::CalcPointJacobian(rd_.model_, rd_.q_virtual_, rd_.link_[cb_robot_[obj_id1].link_id].id, link_xpos_joint_to_nearestcenter1, J_col_obj1, false);

    // second collision object
    // vector stores translation vector in link frame from the joint to the nearest point on the collision object's center (sphere: center or capsule: axis point)
    Vector3d link_xpos_joint_to_nearestcenter2 = rd_.link_[cb_robot_[obj_id2].link_id].local_rotm.transpose() * (nearest_point2 - rd_.link_[cb_robot_[obj_id2].link_id].local_xpos);

    // calcate linear part of Jacobian for the second collision object
    RigidBodyDynamics::CalcPointJacobian(rd_.model_, rd_.q_virtual_, rd_.link_[cb_robot_[obj_id2].link_id].id, link_xpos_joint_to_nearestcenter2, J_col_obj2, false);

    J_row = sign * normal_vec.transpose() * world_to_base_rotm_yaw_only_.transpose() * (J_col_obj1 - J_col_obj2);

    if (cbf_mode == CbfType::Dyn){
        // linear part of Jdot*qdot for both collision objects
        Vector3d Jdotqdot_col_obj1, Jdotqdot_col_obj2;
        Jdotqdot_col_obj1.setZero(); Jdotqdot_col_obj2.setZero();

        // calculate linear part of Jdot*qdot for both collision objects
        // a = Jdot*qdot + J*qddot -> here qddot is set to zero(Eigen::Vector3d::Zero()) vector
        Jdotqdot_col_obj1 = RigidBodyDynamics::CalcPointAcceleration(rd_.model_, rd_.q_virtual_, rd_.q_dot_virtual_, Eigen::Vector3d::Zero(), rd_.link_[cb_robot_[obj_id1].link_id].id, link_xpos_joint_to_nearestcenter1, false);
        Jdotqdot_col_obj2 = RigidBodyDynamics::CalcPointAcceleration(rd_.model_, rd_.q_virtual_, rd_.q_dot_virtual_, Eigen::Vector3d::Zero(), rd_.link_[cb_robot_[obj_id2].link_id].id, link_xpos_joint_to_nearestcenter2, false);

        Jdotqdot_projection = sign * normal_vec.transpose() * world_to_base_rotm_yaw_only_.transpose() * (Jdotqdot_col_obj1 - Jdotqdot_col_obj2);
    }
}

Eigen::RowVectorXd CollisionManager::computeObstacleAvoidConstraintTermsRow(const unsigned int obj_id, 
                                                                            const Eigen::Vector3d nearest_point_obj, 
                                                                            const unsigned int obs_id, 
                                                                            const Eigen::Vector3d nearest_point_obs, 
                                                                            const int sign,
                                                                            double &Jdotqdot_projection,
                                                                            double &obs_vel_projection,
                                                                            const CbfType cbf_mode)
{
    // Jacobian used for the obstacle avoidance constraint
    RowVectorXd J_row;

    J_row.setZero(MODEL_DOF_VIRTUAL);
    Jdotqdot_projection = 0.0;

    // unit vector connecting the two closest points between two collision objects
    Vector3d normal_vec = (nearest_point_obj - nearest_point_obs) / (nearest_point_obj - nearest_point_obs).norm();

    // linear part of Jacobian from base to trans_joint_to_nearestcenter
    MatrixXd J_col_obj;
    J_col_obj.setZero(3,MODEL_DOF_VIRTUAL);

    // vector stores translation vector in link frame from the joint to the nearest point on the collision object's center (sphere: center or capsule: axis point)
    Vector3d link_xpos_joint_to_nearestcenter = rd_.link_[cb_robot_[obj_id].link_id].local_rotm.transpose() * (nearest_point_obj - rd_.link_[cb_robot_[obj_id].link_id].local_xpos);

    // calcate linear part of Jacobian for the first collision object
    RigidBodyDynamics::CalcPointJacobian(rd_.model_, rd_.q_virtual_, rd_.link_[cb_robot_[obj_id].link_id].id, link_xpos_joint_to_nearestcenter, J_col_obj, false);

    J_row = sign * normal_vec.transpose() * world_to_base_rotm_yaw_only_.transpose() * (J_col_obj);
    obs_vel_projection = sign * normal_vec.transpose() * cb_obstacles_[0].local_v;

    if (cbf_mode == CbfType::Dyn){
        // linear part of Jdot*qdot for the collision object
        VectorXd Jdotqdot_col_obj;
        Jdotqdot_col_obj.setZero(MODEL_DOF_VIRTUAL);

        // calculate linear part of Jdot*qdot for the collision object
        // a = Jdot*qdot + J*qddot -> here qddot is set to zero(Eigen::Vector3d::Zero()) vector
        Jdotqdot_col_obj = RigidBodyDynamics::CalcPointAcceleration(rd_.model_, rd_.q_virtual_, rd_.q_dot_virtual_, Eigen::Vector3d::Zero(), rd_.link_[cb_robot_[obj_id].link_id].id, link_xpos_joint_to_nearestcenter, false);

        Jdotqdot_projection = sign * normal_vec.transpose() * world_to_base_rotm_yaw_only_.transpose() * (Jdotqdot_col_obj);
    }

    return J_row;
}

//________________________________________________________________________________________________//

//================================== Communication with Camera ===================================//

void CollisionManager::pubBasetoHeadTransform()
{
    // ts_base_to_head stores the transform data between base_link and head_link
    base_to_head_pose_msg_.header.stamp = ros::Time::now();

    // translation
    base_to_head_pose_msg_.pose.position.x = rd_.link_[Head].local_xpos(0);
    base_to_head_pose_msg_.pose.position.y = rd_.link_[Head].local_xpos(1);
    base_to_head_pose_msg_.pose.position.z = rd_.link_[Head].local_xpos(2);

    // rotation matrix -> quaternion
    Eigen::Quaterniond quat_head(rd_.link_[Head].local_rotm);
    base_to_head_pose_msg_.pose.orientation.x = quat_head.x();
    base_to_head_pose_msg_.pose.orientation.y = quat_head.y();
    base_to_head_pose_msg_.pose.orientation.z = quat_head.z();
    base_to_head_pose_msg_.pose.orientation.w = quat_head.w();

    // publishes the transform data
    base_to_head_pose_pub_.publish(base_to_head_pose_msg_);
}

void CollisionManager::BasetoQRTransformCallback(const geometry_msgs::PoseStamped &msg)
{
    std::lock_guard<std::mutex> lk(measurement_mutex_);
    
    base_to_qr_transform_.translation() << msg.pose.position.x,
                                           msg.pose.position.y,
                                           msg.pose.position.z;
    
    Eigen::Quaterniond quat_qr(msg.pose.orientation.w,
                               msg.pose.orientation.x,
                               msg.pose.orientation.y,
                               msg.pose.orientation.z);
    
    base_to_qr_transform_.linear() = quat_qr.toRotationMatrix();                           

    has_new_measure_ = true;
}

void CollisionManager::updateObstacle()
{
    // 장애물이 아직 하나밖에 없어서 0 인덱스로 바로 updateState 진행했슴다.
    // 후에는 장애물이 여러 개 추적될 때 새로운 장애물인지 판별하는 cost matrix
    if(!tracked_obstacles_.empty()) tracked_obstacles_[0].updateState();
    
    {
        std::lock_guard<std::mutex> lk(measurement_mutex_);
        if(has_new_measure_){
            if(tracked_obstacles_.empty()){
                TrackedObstacle to(base_to_qr_transform_.translation());
                tracked_obstacles_.push_back(to);
            }
            else{
                tracked_obstacles_[0].correctState(base_to_qr_transform_.translation());
                state_corrected = true;
            }

            has_new_measure_ = false;
        }
    }
    
    if(!tracked_obstacles_.empty()){
        if(tracked_obstacles_[0].hasFaded()){
            tracked_obstacles_.clear();
            cb_obstacles_.clear();
            state_corrected = false;
        }
        else{
            if(cb_obstacles_.empty()){
                CollisionBody cb;
                
                cb.local_xpos = tracked_obstacles_[0].getObstacle().pos_;
                // cb.local_xpos = base_to_qr_transform_.translation();

                // consider the size of the QR code attached obstacle 
                cb.local_xpos(0) += 0.02;

                cb.local_v = tracked_obstacles_[0].getObstacle().vel_;
                cb.sphere = std::make_shared<Sphere>(0.1149);

                cb_obstacles_.push_back(cb);
            }
            else{
                cb_obstacles_[0].local_xpos = tracked_obstacles_[0].getObstacle().pos_;
                // cb_obstacles_[0].local_xpos = base_to_qr_transform_.translation();

                // consider the size of the QR code attached obstacle 
                cb_obstacles_[0].local_xpos(0) += 0.02;

                cb_obstacles_[0].local_v = tracked_obstacles_[0].getObstacle().vel_;
            }
        }
    }
}

// ugly initialization of static members of tracked obstacles
int    TrackedObstacle::s_fade_counter_size_     = 0;
double TrackedObstacle::s_sampling_time_         = 100.0;
double TrackedObstacle::s_process_variance_      = 0.0;
double TrackedObstacle::s_process_rate_variance_ = 0.0;
double TrackedObstacle::s_measurement_variance_  = 0.0;