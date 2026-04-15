#include "control_manager.h"

using namespace TOCABI;

ControlManager::ControlManager(RobotData& rd, RigidBodyDynamics::Model& model) : rd_(rd), model_(model)
{
}

/*------------------------*/
/*--- PUBLIC FUNCTIONS ---*/
void ControlManager::update(bool &cm_init_save_trigger)
{
    contactStateMachine();
    mapGlobalToBase();
    updateDynamics();
    updateContact();

    if(cm_init_save_trigger)
    {           
        saveInitialState();

        std::cout << " " << std::endl;
        std::cout << "===== ControlManager Initial State Saved =====" << std::endl;
        std::cout << " " << std::endl;
        cm_init_save_trigger = false;
    }
}

/*-------------------------*/
/*--- PRIVATE FUNCTIONS ---*/
void ControlManager::contactStateMachine()
{
    //--- Contact Change Trigger
    if(rd_.is_left_contact_transition == true)
    {
        WBC::SetContact(rd_, true, false);
        
        rd_.is_left_contact_transition = false;
    }
    else if(rd_.is_right_contact_transition == true)
    {
        WBC::SetContact(rd_, false, true);

        rd_.is_right_contact_transition = false;
    }
    else if(rd_.is_double_contact_transition == true)
    {
        WBC::SetContact(rd_, true, true);

        rd_.is_double_contact_transition = false;
    }

    local_LF_contact = rd_.ee_[0].contact;
    local_RF_contact = rd_.ee_[1].contact;
}

void ControlManager::mapGlobalToBase()
{
    //--- Robot States
    base_pos = rd_.link_[Pelvis].xpos; 
    base_rot = DyrosMath::rotateWithZ(DyrosMath::rot2Euler(rd_.link_[Pelvis].rotm)(2)); 
    
    for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
    {
        //--- Base frame
        rd_.link_[idx].local_Jac_v             = base_rot.transpose() * rd_.link_[idx].Jac().topRows(3);  
        rd_.link_[idx].local_Jac_w             = base_rot.transpose() * rd_.link_[idx].Jac().bottomRows(3); 
        rd_.link_[idx].local_Jac.topRows(3)    =                        rd_.link_[idx].local_Jac_v;
        rd_.link_[idx].local_Jac.bottomRows(3) =                        rd_.link_[idx].local_Jac_w;

        rd_.link_[idx].local_xpos           = base_rot.transpose() * (rd_.link_[idx].xpos - base_pos);
        rd_.link_[idx].local_rotm           = base_rot.transpose() *  rd_.link_[idx].rotm;                             
        rd_.link_[idx].local_v              = base_rot.transpose() *  rd_.link_[idx].v;                               
        rd_.link_[idx].local_w              = base_rot.transpose() *  rd_.link_[idx].w;

        if(idx != COM_id){
            Eigen::Vector6d Jdotqdot; Jdotqdot.setZero();
            Jdotqdot = RigidBodyDynamics::CalcPointAcceleration6D(rd_.model_, rd_.q_virtual_, rd_.q_dot_virtual_, Eigen::VectorVQd::Zero(), idx, Eigen::Vector3d::Zero(), false);

            rd_.link_[idx].local_Jdotqdot.head(3) = base_rot.transpose() * Jdotqdot.tail(3);
            rd_.link_[idx].local_Jdotqdot.tail(3) = base_rot.transpose() * Jdotqdot.head(3);
        }
    }

    //--- Virtual Joints
    rd_.local_q_virtual_ = rd_.q_virtual_;
    rd_.local_q_dot_virtual_ = rd_.q_dot_virtual_;
    rd_.local_q_virtual_.segment(0,3) = rd_.link_[Pelvis].local_xpos;
    
    Quaterniond base_quat(rd_.link_[Pelvis].local_rotm);
    base_quat.normalize();
    
    rd_.local_q_virtual_(3)  = base_quat.x();
    rd_.local_q_virtual_(4)  = base_quat.y();
    rd_.local_q_virtual_(5)  = base_quat.z();
    rd_.local_q_virtual_(39) = base_quat.w();

    rd_.local_q_dot_virtual_.segment(0,3) = rd_.link_[Pelvis].local_v;
    rd_.local_q_dot_virtual_.segment(3,3) = rd_.link_[Pelvis].local_w;
}

void ControlManager::updateDynamics()
{
    //--- Dynamics
    rd_.local_A.setZero(); 
    rd_.local_G.setZero(); 
    rd_.local_A_inv.setZero();

    Eigen::MatrixXd A_temp_; A_temp_.setZero(MODEL_DOF_VIRTUAL, MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_, rd_.local_q_virtual_, A_temp_, true);
    rd_.local_A = A_temp_;
    rd_.local_A_inv = rd_.local_A.inverse();

    Eigen::VectorXd G_temp_; G_temp_.setZero(MODEL_DOF_VIRTUAL);
    RigidBodyDynamics::NonlinearEffects(model_, rd_.local_q_virtual_, rd_.local_q_dot_virtual_, G_temp_);
    rd_.local_G = G_temp_;
}

void ControlManager::updateContact()
{
    rd_.local_J_C.setZero(12, MODEL_DOF_VIRTUAL);

    rd_.local_J_C.block(0, 0, 3, MODEL_DOF_VIRTUAL) = base_rot.transpose() * rd_.ee_[0].jac_contact.cast<double>().topRows(3);
    rd_.local_J_C.block(3, 0, 3, MODEL_DOF_VIRTUAL) = base_rot.transpose() * rd_.ee_[0].jac_contact.cast<double>().bottomRows(3);
    rd_.local_J_C.block(6, 0, 3, MODEL_DOF_VIRTUAL) = base_rot.transpose() * rd_.ee_[1].jac_contact.cast<double>().topRows(3);
    rd_.local_J_C.block(9, 0, 3, MODEL_DOF_VIRTUAL) = base_rot.transpose() * rd_.ee_[1].jac_contact.cast<double>().bottomRows(3);
}

void ControlManager::saveInitialState()
{
    rd_.q_desired_virtual = rd_.local_q_virtual_;
    rd_.q_dot_desired_virtual.setZero();

    for (int idx = 0; idx < LINK_NUMBER + 1; idx++)
    {
        //--- Global frame
        rd_.link_[idx].rot_init = rd_.link_[idx].local_rotm;                             

        //--- Base frame
        rd_.link_[idx].local_xpos_init = rd_.link_[idx].local_xpos;
        rd_.link_[idx].local_rotm_init = rd_.link_[idx].local_rotm;                             
        rd_.link_[idx].local_v_init    = rd_.link_[idx].local_v;                               
        rd_.link_[idx].local_w_init    = rd_.link_[idx].local_w;
    }
}