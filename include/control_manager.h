#pragma once
#include <string>
#include <Eigen/Dense>

#include "tocabi_lib/robot_data.h"
#include "wholebody_functions.h"
#include "utils.h"

class ControlManager
{
public:
    ControlManager(RobotData &rd, RigidBodyDynamics::Model &model);

    void update(bool &cm_init_save_trigger);
    void saveInitialState();

private:
    // Depedencies
    RobotData &rd_;
    RigidBodyDynamics::Model &model_;  

    void contactStateMachine();
    void mapGlobalToBase();
    void updateDynamics();
    void updateContact();
    
    Eigen::Vector3d base_pos; 
    Eigen::Matrix3d base_rot;
    bool local_LF_contact = false;
    bool local_RF_contact = false;
};