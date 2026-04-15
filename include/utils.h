#pragma once
#include <Eigen/Dense>
#include "math_type_define.h"

enum class TaskType { Position, Orientation };

struct TaskInfo {
    int link_idx;
    TaskType type;
};

enum class ContactIndicator {
    DoubleSupport,
    LeftSingleSupport,
    RightSingleSupport
};

struct ConstraintMatrix {
    Eigen::MatrixXd A;
    Eigen::VectorXd lbA;
    Eigen::VectorXd ubA;
};

enum class TaskMotionType {
    None,
    PelvHand,
    Taichi,
    Walking,
    TeleOperation
};

enum class CbfType {
    None,
    Kin,
    Dyn
};


struct WorkspaceBoundaryPair {
    int idx_A; int idx_B; double max_dist{0.0};
};

Eigen::Vector3d AngvelToEulerRates(const Eigen::Vector3d& angVel, const Eigen::Vector3d& EulerAngle);
Eigen::Quaterniond integrateQuatBodyExp(const Eigen::Quaterniond &q, const Eigen::Vector3d &omega_body, double dt);
Eigen::Vector3d getOrientationError(const Eigen::Matrix3d& current_rotation, const Eigen::Matrix3d& desired_rotation);
Eigen::Matrix3d hat(const Eigen::Vector3d& v);
Eigen::Vector3d vee(const Eigen::Matrix3d& M);
Eigen::Matrix2d rotZaxis2d(double theta);
double cubicBezierPolynomial(double current_time, double start_time, double end_time, double p_init, double p_mid, double p_end);
double cubicDotBezierPolynomial(double current_time, double start_time, double end_time, double p_init, double p_mid, double p_end);
inline std::string contactIndicatorToString(const ContactIndicator& mode)
{
    switch (mode)
    {
        case ContactIndicator::DoubleSupport:      return "DoubleSupport";
        case ContactIndicator::LeftSingleSupport:  return "LeftSingleSupport";
        case ContactIndicator::RightSingleSupport: return "RightSingleSupport";
        default:                                   return "Unknown";
    }
}