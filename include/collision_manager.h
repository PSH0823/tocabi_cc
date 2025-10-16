/**
 * @file collision_manager.h
 * @brief CollisionManager class for integrating environment perception with HPP-FCL.
 * 
 * This class handles:
 *  - Updating the robot's collision objects based on joint configurations.
 *  - Representing and updating collision objects of the surrounding environment
 *    detected via camera or sensors.
 *  - Performing collision checks and computing minimum distances between
 *    robot links and environmental objects using HPP-FCL.
 *  - Broadcasting and accessing transformation matrices for robot frames
 *    and camera-detected objects using ROS tf2.
 *  - Sending position and orientation of QR code (ArUCo) attached obstacles
 *    to the MuJoCo simulation, allowing real-time update of obstacle positions.
 * 
 * It is intended to be used within the control loop to ensure that
 * collision objects always reflect the current robot and environment state.
 */

#include "math_type_define.h"
#include "tocabi_lib/robot_data.h"

// Collision Libarary(HPP-FCL) Headers
#include <hpp/fcl/shape/geometric_shapes.h>
#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/math/transform.h>
#include <hpp/fcl/collision_data.h>
#include <hpp/fcl/distance.h>
#include <hpp/fcl/collision.h>

// Headers for Sending TF information of camera frame
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/TransformStamped.h>

// Header for sending position and orientation of obstacles to MuJoCo simulation
#include <geometry_msgs/PoseStamped.h>

class CollisionManager
{
public:
    CollisionManager(const RobotData &rd);

    // robot data
    const RobotData& rd_cm_;

    //================================= HPP-FCL ==================================//

    // enum for the index of collision objects for the robot links
    enum CollisionObjectIdx
    {
        Left_Hand_Col_ID,
        Right_Hand_Col_ID,
        Body_Col_ID,
        Col_Obj_Count  // total number of collision Objects in Robot (used for col_obj_robot_links_ array sizing)
    };

    // Options for choosing object pairs in getDistanceResultBetweenObjects()
    enum DistanceResultOption
    {
        Links,
        Link2Env,
        Envs
    };

    // collision objects for the robot links
    vector<std::shared_ptr<hpp::fcl::CollisionObject>> col_obj_robot_links_;

    // collision objects for the environment
    vector<std::shared_ptr<hpp::fcl::CollisionObject>> col_obj_obstacles_;

    /**
     * @brief Assign a collision object as a sphere shape to the target object
     * 
     * @param obj_rot the rotation matrix of the coordinate of the collision object
     * @param obj_trans the translation of the coordinate of the collision object
     * @param radius the radius of the sphere
     * 
     * @return The assigned collision object as a sphere shape
     */
    std::shared_ptr<hpp::fcl::CollisionObject> assignSphereCollisionObject(const Eigen::Matrix3d obj_rot,
                                                                           const Eigen::Vector3d obj_trans,
                                                                           const double radius
                                                                           );
    
    /**
     * @brief Assign a collision object as a capsule shape to the target object
     * 
     * @param obj_rot the rotation matrix of the coordinate of the collision object
     * @param obj_trans the translation of the coordinate of the collision object
     * @param radius the radius of the capsule
     * @param height the height of the capsule
     * 
     * @return The assigned collision object as a capsule shape
     */
    std::shared_ptr<hpp::fcl::CollisionObject> assignCapsuleCollisionObject(const Eigen::Matrix3d obj_rot,
                                                                            const Eigen::Vector3d obj_trans,
                                                                            const double radius,
                                                                            const double height
                                                                            );
    
    /**
     * @brief Assign a collision object as a box shape to the target object
     * 
     * @param obj_rot the rotation matrix of the coordinate of the collision object
     * @param obj_trans the translation of the coordinate of the collision object
     * @param size_x the size of the box in x direction
     * @param size_y the size of the box in y direction
     * @param size_z the size of the box in z direction
     * 
     * @return The assigned collision object as a box shape
     */
    std::shared_ptr<hpp::fcl::CollisionObject> assignBoxCollisionObject(const Eigen::Matrix3d obj_rot,
                                                                        const Eigen::Vector3d obj_trans,
                                                                        const double size_x,
                                                                        const double size_y,
                                                                        const double size_z
                                                                        );

    /**
     * @brief Update the transformation matrices of the robot's collision objects
     * 
     * @note This method should be called after updating the robot's model and computing forward kinematics,
     *       so that the collision objects correctly reflect the current joint configuration
     */
    void updateRobotCollisionObjectsTransforms();

    /**
     * @brief Get the DistanceResult structure between two collision objects
     * 
     * @param col_obj1 the first collision object.
     * @param col_obj2 the second collision object.
     * @param option specifies which pair of objects to include in the returned DistanceResult:
     * 
     *               - Links: DistanceResult between two robot links
     * 
     *               - Link2Env: DistanceResult between a robot link and an environment object
     * 
     *               - Envs: DistanceResult between two environment objects
     * 
     * @return A DistanceResult structure containing the minimum distance and the closest points between the two objects
     * 
     * @note - minimum distance: <Name-of-DistanceResult>.min_distance
     * 
     *       - closest point on object 1: <Name-of-DistanceResult>.nearest_points[0]
     * 
     *       - closest point on object 2: <Name-of-DistanceResult>.nearest_points[1]
     */                                                                
    hpp::fcl::DistanceResult getDistanceResultBetweenObjects(const int col_obj_idx1,
                                                             const int col_obj_idx2,
                                                             DistanceResultOption option
                                                             );
    
    //____________________________________________________________________________//

    //======================== Communication with Camera =========================//

    // transformation matrices
    Eigen::Isometry3d base_to_qr_transform_;    // from base frame to object frame
    Eigen::Isometry3d base_to_head_transform_;  // from base frame to head frame
    //rotation matrix and translation vector from global frame to base frame
    Eigen::Matrix3d global_to_base_rot_yaw_only_;
    Eigen::Vector3d global_to_base_trans_;

    /**
     * @brief Publish the transformation matrix from base frame to head frame
     * 
     * @note This method uses ROS tf2 to broadcast the transformation matrix
     */
    void pubBasetoHeadTransform();

    /**
     * @brief Get the transformation matrix from base frame to QR code(ArUCo) frame
     * 
     * @return The transformation matrix from base frame to QR code(ArUCo) frame(object frame)
     * 
     * @note This method uses ROS tf2 to lookup the transformation matrix
     */
    Eigen::Isometry3d getBasetoQRTransform();

    /** This method is for checking aruco code detection
     * @brief Get the transformation matrix from base frame to head frame
     */
    void getBasetoHeadTransform();
    
    //____________________________________________________________________________//

    //============================ Obstacle in MuJoCo ============================//

    /**
     * @brief Publish the desired position and orientation of a QR code (ArUCo) obstacle
     *        in the MuJoCo simulation environment.
     * 
     * @param sim_tick the simulation tick at which to apply the new obstacle position.
     * @param hz control frequency
     */
    void pubQRObstaclePose(const int sim_tick, 
                           const double hz);

    //____________________________________________________________________________//


private:   
    // ROS
    ros::NodeHandle nh_cm_;
    
    // tf2_ros(coordinate transform communication)
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener{tf_buffer_};


    //======================== Communication with Camera =========================//
    
    geometry_msgs::TransformStamped base_to_head_tf_msg_;   // contation transform from base to head
    geometry_msgs::TransformStamped base_to_qr_tf_msg_;     // contain transform from base to qr object

    //____________________________________________________________________________//

    //============================ Obstacle in MuJoCo ============================//
    
    ros::Publisher aruco_pose_pub_;              // send aruco_pose_msg to MuJoCo

    geometry_msgs::PoseStamped aruco_pose_msg_;  // contain pos.&ori. of the obstacle

    //____________________________________________________________________________//

};