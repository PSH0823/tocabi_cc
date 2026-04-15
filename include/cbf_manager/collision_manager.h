/**
 * @file collision_manager.h
 * @brief CollisionManager class for managing robot and environment collision objects
 *        and integrating perception data into the control loop.
 *
 * This class handles:
 *  - Updating the robot's collision objects of the robot model based on joint configurations.
 *  - Representing and updating collision objects of surrounding environment obstacles
 *    detected via cameras or sensors.
 *  - Computing minimum distances, nearest points, and Jacobian-based constraint terms
 *    for self-collision avoidance and obstacle avoidance.
 *
 * This class is intended to be used inside the control loop so that collision objects
 * consistently reflect the current robot state and the perceived environment.
 */

#ifndef COLLISION_MANAGER_H
#define COLLISION_MANAGER_H

// ROS Headers
#include <std_msgs/UInt8MultiArray.h>           // For collision status of collision objects
#include <std_msgs/Bool.h>
#include <geometry_msgs/TransformStamped.h>     // For TF information of camera frame
#include <geometry_msgs/PoseStamped.h>          // For sending obstacle poses to MuJoCo

#include "math_type_define.h"
#include "tocabi_lib/robot_data.h"
#include "utils.h"
#include "cbf_manager/tracked_obstacle.h"

class CollisionManager
{
public:
    CollisionManager(RobotData &rd, RigidBodyDynamics::Model &model);

    void callAvailableQueue();

private:
    // robot data
    RobotData &rd_;

    // RBDL model of the Robot
    RigidBodyDynamics::Model &model_;

    // control frequency
    double hz_ = 2000;

    // ROS
    ros::NodeHandle nh_cm_;
    ros::CallbackQueue queue_cm_;
    // publisher
    ros::Publisher col_status_pub_;                         // send collision status of collision objects to MuJoCo
    ros::Publisher base_to_head_pose_pub_;                  // send base_to_head_pose_pub_ to camera
    ros::Publisher aruco_pose_pub_;                         // send aruco_pose_msg to MuJoCo
    // subscriber
    ros::Subscriber aruco_detect_sub_;                      // receive the pose of the QR code(ArUCo) obstacle
    // msg
    std_msgs::UInt8MultiArray col_status_msg_;              // contain collision status of collision objects
    geometry_msgs::TransformStamped base_to_head_tf_msg_;   // contain transform from base to head
    geometry_msgs::TransformStamped base_to_qr_tf_msg_;     // contain transform from base to qr object
    geometry_msgs::PoseStamped aruco_pose_msg_;             // contain pos.&ori. of the obstacle
    geometry_msgs::PoseStamped base_to_head_pose_msg_;      // contain transform from base to head

    //============================ Collision Related =============================//
public:
    // collision geometry shapes
    // sphere
    struct Sphere{
        double radius;

        // Sphere() = default;
        Sphere(double r) : radius(r) {}
    };

    // capsule
    struct Capsule{
        double radius;
        double half_length;

        // Capsule() = default;
        Capsule(double r, double lz) : radius(r), half_length(lz / 2) {}
    };

    // box
    struct Box{
        Vector3d half_side;

        // Box() = default;
        Box(double x, double y, double z) : half_side(x / 2, y / 2, z / 2) {}
    };

    /**
     * @brief Struct representing a single collision body,
     *        including its geometry, transform, and link association.
     */
    struct CollisionBody{
        // name the link where the collision object is attached
        std::string link_name;
        // link id of robot data(rd)
        int link_id;

        // collision status of each collision object (true: in collision, false: safe(no collision))
        bool in_collision = false; 

        // translation vector
        Eigen::Vector3d link_xpos;       // from link frame to collision object frame
        Eigen::Vector3d local_xpos;      // from floating base to collision object frame (used for obstacle)
        Eigen::Vector3d xpos;            // from world frame to collision object frame

        // velocity vector
        Eigen::Vector3d local_v;         // velocity of collision object in world frame

        // rotation matrix
        Eigen::Matrix3d link_rotm;       // from link frame to collision object frame
        Eigen::Matrix3d local_rotm;      // from floating base to collision object frame
        Eigen::Matrix3d rotm;            // from world frame to collision object frame
        
        // collision geometry shapes
        std::shared_ptr<Sphere> sphere = nullptr;     // valid if type == Sphere
        std::shared_ptr<Capsule> capsule = nullptr;   // valid if type == Capsule
        std::shared_ptr<Box> box = nullptr;           // valid if type == Box
    };

    // enum for the ID of collision objects for the robot links
    enum CollisionObjectIdx
    {
        Left_Pelvis_Col_ID,
        Right_Pelvis_Col_ID,
        Left_Upper_Leg_Col_ID,
        Left_Lower_Leg_Col_ID,
        Left_Inner_Foot_Col_ID,
        Left_Outer_Foot_Col_ID,
        Right_Upper_Leg_Col_ID,
        Right_Lower_Leg_Col_ID,
        Right_Inner_Foot_Col_ID,
        Right_Outer_Foot_Col_ID,
        // Left_Upper_Body_Col_ID,
        // Right_Upper_Body_Col_ID,
        Left_Upper_Arm_Col_ID,
        Left_ForeArm_Col_ID,
        Left_Hand_Col_ID,
        Head_Col_ID,
        Right_Upper_Arm_Col_ID,
        Right_ForeArm_Col_ID,
        Right_Hand_Col_ID,
        Col_Obj_Count  // total number of collision Objects in Robot (used for cb_robot_ vector sizing)
    };

    // options for choosing object pairs in getDistanceResultBetweenObjects()
    enum DistanceResultOption
    {
        Links,
        Link2Env,
        Envs
    };

    // collisionBody vectors of the robot
    std::vector<CollisionBody> cb_robot_;

    // collisionBody vectors of the obstacles in the environment
    std::vector<CollisionBody> cb_obstacles_;

    /**
     * @brief Struct to hold constraint terms for self-collision avoidance (between robot collision objects)
     */
    struct SelfColAvoidConstraintTerms
    {
        int num_pairs;                    // the number of collision pairs
        
        MatrixXd J;                       // Jacobian-based constraint matrix for all defined collision pairs         

        VectorXd Jdotqdot;                // Jacobian dot times q dot based vector for all defined collision pairs

        VectorXd min_distances;           // minimum distances between all defined collision pairs
    };

    /**
     * @brief Struct to hold constraint terms for obstacle avoidance (between robot and obstacles)
     */
    struct ObstacleAvoidConstraintTerms : public SelfColAvoidConstraintTerms
    {
        VectorXd obs_vel_projections;    // obstacle velocity projection to the normal vector that connects the nearest points
    };

    struct SelfColAvoidConstraintTerms self_collision_avoid_terms_;
    struct ObstacleAvoidConstraintTerms obstacle_avoid_terms_;

    /**
     * @brief Publishes the self-collision status of all collision objects of the robot
     *        to the MuJoCo simulation
     * 
     * @note This method constructs and sends a `std_msgs::UInt8MultiArray` message
     *       where each element represents the collision status of a corresponding collision object:
     * 
     *       - 0: safe (no collision)
     * 
     *       - 1: in collision
     *
     */
    void pubSelfCollisionStatus();

    /**
     * @brief Update the transformation matrices of the robot's collision objects
     * 
     * @note This method should be called after updating the robot's model and computing forward kinematics,
     *       so that the collision objects correctly reflect the current joint configuration
     */
    void updateRobotCollisionObjectsPose();

    /**
     * @brief Compute the Jacobian-based constraint matrix for self-collision avoidance
     *        and minimum distances between collision pairs
     * 
     * @param cbf_mode the type of control barrier function (None, Kin, Dyn)
     * 
     * @note This method calculates the constraint matrix for self-collision avoidance
     *       based on the predefined collision pairs specified in col_pair_ids_
     */
    void computeSelfColAvoidConstraintTerms(const CbfType cbf_mode);

    /**
     * @brief Compute the Jacobian-based constraint matrix for obstacle avoidance
     *        , minimum distances between collision pairs, and obstacle velocity projections
     * 
     * @param cbf_mode the type of control barrier function (None, Kin, Dyn)
     * 
     * @note This method calculates the constraint matrix for obstacle avoidance
     *       between the robot's collision objects and the environment obstacles tracked in cb_obstacles_
     */
    void computeObstacleAvoidConstraintTerms(const CbfType cbf_mode);

    /**
     * @brief Check for self-collisions among the robot's collision objects
     * 
     * @note This method is used to detect collisions for visualization purposes, particularly to highlight
     *       collisions in red when the self-collision avoidance constraint is not applied.
     */
    void checkSelfCollision();

private:
    // pairs of collision object IDs for self-collision checks
    std::vector<std::pair<CollisionObjectIdx, CollisionObjectIdx>> col_id_pairs_ =
    {
        {Left_Pelvis_Col_ID, Left_Upper_Arm_Col_ID},
        {Left_Pelvis_Col_ID, Left_ForeArm_Col_ID},
        {Left_Pelvis_Col_ID, Left_Hand_Col_ID},
        {Right_Pelvis_Col_ID, Right_Upper_Arm_Col_ID},
        {Right_Pelvis_Col_ID, Right_ForeArm_Col_ID},
        {Right_Pelvis_Col_ID, Right_Hand_Col_ID},
        {Left_Upper_Leg_Col_ID, Right_Upper_Leg_Col_ID},
        {Left_Upper_Leg_Col_ID, Right_Lower_Leg_Col_ID},
        {Left_Upper_Leg_Col_ID, Left_ForeArm_Col_ID},
        {Left_Upper_Leg_Col_ID, Left_Hand_Col_ID},
        {Left_Lower_Leg_Col_ID, Right_Upper_Leg_Col_ID},
        {Left_Lower_Leg_Col_ID, Right_Lower_Leg_Col_ID},
        {Left_Lower_Leg_Col_ID, Right_Inner_Foot_Col_ID},
        {Left_Inner_Foot_Col_ID, Right_Lower_Leg_Col_ID},
        {Left_Inner_Foot_Col_ID, Right_Inner_Foot_Col_ID},
        {Right_Upper_Leg_Col_ID, Right_ForeArm_Col_ID},
        {Right_Upper_Leg_Col_ID, Right_Hand_Col_ID},
        // {Left_Upper_Arm_Col_ID, Right_Hand_Col_ID},
        // {Left_Hand_Col_ID, Right_Upper_Arm_Col_ID},
        {Left_ForeArm_Col_ID, Right_ForeArm_Col_ID},
        {Left_ForeArm_Col_ID, Right_Hand_Col_ID},
        {Left_Hand_Col_ID, Right_ForeArm_Col_ID},
        {Left_Hand_Col_ID, Right_Hand_Col_ID},
        {Head_Col_ID, Left_ForeArm_Col_ID},
        {Head_Col_ID, Left_Hand_Col_ID}
    };

    // collision status flags for each collision object (0: safe, 0>=: in collision)
    std::vector<unsigned int> collision_flags_ = std::vector<unsigned int>(Col_Obj_Count, 0);

    /**
     * @brief Initialize the CollisionBody struct information of the robot
     * 
     * @note This method sets the predefined values of CollisionBody members
     */
    void initCollisionBody();
    
    /**
     * @brief Get the collision result (ColResult) between two sphere collision objects
     * 
     * @param obj_id1 index of the first collision object
     * @param obj_id2 index of the second collision object
     * @param min_distance minimum distance between the nearest points of the two objects
     * @param nearest_point1 the point on object 1 that is closest to object 2, expressed in the world frame
     * @param nearest_point2 the point on object 2 that is closest to object 1, expressed in the world frame
     * @param option  specifies which pair of objects to include in the collision result:
     * 
     *               - Links: DistanceResult between two robot links
     * 
     *               - Link2Env: DistanceResult between a robot link and an environment object
     * 
     *               - Envs: DistanceResult between two environment objects
     * 
     * @note - The collision result contains the minimum distance between the nearest points
     *         and the corresponding points on each object's surface
     * 
     *       - When the option is set to Link2Env, obj_id1 must correspond to a robot link,
     *         and obj_id2 must correspond to an environment obstacle.
     */                                                          
    void ColResultSphere2Sphere(const unsigned int obj_id1, 
                                const unsigned int obj_id2, 
                                double &min_distance,
                                Eigen::Vector3d &nearest_point1,
                                Eigen::Vector3d &nearest_point2,
                                DistanceResultOption option = Links);

    /**
     * @brief Get the collision result (ColResult) between sphere and capsule collision objects
     * 
     * @param obj_id1 index of the first collision object
     * @param obj_id2 index of the second collision object
     * @param min_distance minimum distance between the nearest points of the two objects
     * @param nearest_point1 the point on object 1 that is closest to object 2, expressed in the world frame
     * @param nearest_point2 the point on object 2 that is closest to object 1, expressed in the world frame
     * @param option  specifies which pair of objects to include in the collision result:
     * 
     *               - Links: DistanceResult between two robot links
     * 
     *               - Link2Env: DistanceResult between a robot link and an environment object
     * 
     *               - Envs: DistanceResult between two environment objects
     * @param obs_is_sphere indicates whether the obstacle is a sphere (true) or capsule (false)
     * 
     * @note - The sphere object must be assigned to obj_id1
     *      
     *       - The collision result contains the minimum distance between the nearest points
     *         and the corresponding points on each object's surface
     *       
     */                               
    void ColResultSphere2Capsule(const unsigned int obj_id1, 
                                 const unsigned int obj_id2,
                                 double &min_distance, 
                                 Eigen::Vector3d &nearest_point1,
                                 Eigen::Vector3d &nearest_point2,
                                 DistanceResultOption option = Links,
                                 bool obs_is_sphere = true);
    
    /**
     * @brief Get the collision result (ColResult) between two capsule collision objects
     * 
     * @param obj_id1 index of the first collision object
     * @param obj_id2 index of the second collision object
     * @param min_distance minimum distance between the nearest points of the two objects
     * @param nearest_point1 the point on object 1 that is closest to object 2, expressed in the world frame
     * @param nearest_point2 the point on object 2 that is closest to object 1, expressed in the world frame
     * @param option  specifies which pair of objects to include in the collision result:
     * 
     *               - Links: DistanceResult between two robot links
     * 
     *               - Link2Env: DistanceResult between a robot link and an environment object
     * 
     *               - Envs: DistanceResult between two environment objects
     * 
     * @note - The collision result contains the minimum distance between the nearest points
     *         and the corresponding points on each object's surface
     * 
     *       - When the option is set to Link2Env, obj_id1 must correspond to a robot link,
     *         and obj_id2 must correspond to an environment obstacle.
     */                                
    void ColResultCapsule2Capsule(const unsigned int obj_id1, 
                                  const unsigned int obj_id2, 
                                  double &min_distance, 
                                  Eigen::Vector3d &nearest_point1, 
                                  Eigen::Vector3d &nearest_point2,
                                  DistanceResultOption option = Links);

    /**
     * @brief Compute the row-wise terms (Jacobian-based row and Jdot*qdot projection) 
     *        for the self-collision avoidance constraint
     *
     * @param obj_id1 index of the first collision object
     * @param center_point1 the point on object 1 that is closest to object 2, expressed in the world frame
     * @param obj_id2 index of the second collision object
     * @param center_point2 the point on object 2 that is closest to object 1, expressed in the world frame
     * @param sign sign of the minimum distance (positive (objects separated): 1, negative (penetration): -1)
     * @param J_row the Jacobian-based constraint row for the single collision pair
     * @param Jdotqdot_projection the projection of Jdot*qdot term ((Jdot_1-Jdot_2)*qdot)
     *                            onto the normal vector between the two nearest points 
     * @param cbf_mode the type of control barrier function (None, Kin, Dyn)
     * 
     * @note RigidBodyDynamics::UpdateKinematics() must be called before using this method
     */                                 
    void computeSelfColAvoidConstraintTermsRow(const unsigned int obj_id1,
                                               const Eigen::Vector3d nearest_point1,
                                               const unsigned int obj_id2,
                                               const Eigen::Vector3d nearest_point2,
                                               const int sign,
                                               Eigen::Ref<Eigen::RowVectorXd, 0, Eigen::Stride<1, Eigen::Dynamic>> J_row,
                                               double &Jdotqdot_projection,
                                               const CbfType cbf_mode);
    
    /**
     * @brief Compute the row-wise terms (Jacobian-based row, Jdot*qdot projection,
     *        and obstacle velocity projection) for the obstacle avoidance constraint
     * 
     * @param obj_id index of the collision object in the robot
     * @param nearest_point_obj the point on the robot object that is closest to the obstacle, expressed in the world frame
     * @param obs_id index of the collision object in the obstacle
     * @param nearest_point_obs the point on the obstacle that is closest to the robot object, expressed in the world frame
     * @param sign sign of the minimum distance (positive (objects separated): 1, negative (penetration): -1)
     * @param Jdotqdot_projection the projection of Jdot*qdot of the robot onto the normal vector between the two nearest points
     * @param obs_vel_projection the projection of the obstacle velocity onto the normal vector between the two nearest points
     * @param cbf_mode the type of control barrier function (None, Kin, Dyn)
     * 
     * @return the Jacobian-based constraint row for the single collision pair
     * 
     * @note The Jacobian row is returned by value since the number of constraints is determined at runtime
     * 
     *       RigidBodyDynamics::UpdateKinematics() must be called before using this method
     */
    Eigen::RowVectorXd computeObstacleAvoidConstraintTermsRow(const unsigned int obj_id,
                                                              const Eigen::Vector3d nearest_point_obj,
                                                              const unsigned int obs_id,
                                                              const Eigen::Vector3d nearest_point_obs,
                                                              const int sign,
                                                              double &Jdotqdot_projection,
                                                              double &obs_vel_projection,
                                                              const CbfType cbf_mode);

    //____________________________________________________________________________//
                                                             
    //======================== Communication with Camera =========================//
public:
    // transformation matrices
    Eigen::Isometry3d base_to_qr_transform_;    // from base frame to object frame
    Eigen::Isometry3d base_to_head_transform_;  // from base frame to head frame

    // rotation matrix and translation vector from world frame to base frame
    Eigen::Matrix3d world_to_base_rotm_yaw_only_;
    Eigen::Vector3d world_to_base_trans_;

    // mutex for measurement update
    std::mutex measurement_mutex_;

    // flag to indicate new measurement arrival
    bool has_new_measure_ = false;
    
    // Kalman filter parameters
    double tracking_duration;                   // duration for tracking
    double process_var;                         // process variance
    double process_rate_var;                    // process rate variance (time derivative of process)
    double measurement_var;                     // measurement variance

    /**
     * @brief Publish the transformation matrix from base frame to head frame
     * 
     * @note This method uses ROS to publish the transformation matrix
     */
    void pubBasetoHeadTransform();

    /**
     * @brief Callback function to process the received pose of the QR code (ArUCo) obstacle
     * 
     * @param msg the received PoseStamped message containing the pose of the QR code(ArUCo) obstacle
     */
    void BasetoQRTransformCallback(const geometry_msgs::PoseStamped &msg);
    
    /**
     * @brief Update the obstacle information based on the latest measurement
     * 
     * @note This method updates the position and velocity of the tracked obstacles
     *       using a Kalman filter
     */
    void updateObstacle();

private:
    // vector of tracked obstacles
    std::vector<TrackedObstacle> tracked_obstacles_;

    // flag to indicate whether the state inside the Kalman filter has been corrected
    bool state_corrected = false;
    double dt;
};

#endif // COLLISION_MANAGER_H