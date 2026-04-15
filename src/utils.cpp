#include "utils.h"

Eigen::Vector3d AngvelToEulerRates(const Eigen::Vector3d &angVel, const Eigen::Vector3d &EulerAngle)
{

    double roll = EulerAngle(0);
    double pitch = EulerAngle(1);
    double yaw = EulerAngle(2);

    Eigen::Matrix3d T_inv;
    T_inv << cos(yaw) / cos(pitch), sin(yaw) / cos(pitch), 0,
        -sin(yaw), cos(yaw), 0,
        cos(yaw) * tan(pitch), tan(pitch) * sin(yaw), 1;

    return (T_inv * angVel); // Euler rates
}

Eigen::Vector3d getOrientationError(const Eigen::Matrix3d &current_rotation,
                                    const Eigen::Matrix3d &desired_rotation)
{
    //--- Rotation Difference
    Eigen::Matrix3d R_err;
    R_err.setZero();
    R_err = desired_rotation.transpose() * current_rotation;

    //--- Log Matrix
    double r11 = R_err(0, 0);
    double r22 = R_err(1, 1);
    double r33 = R_err(2, 2);
    double trace = r11 + r22 + r33;

    Eigen::Vector3d l;
    l << R_err(2, 1) - R_err(1, 2),
        R_err(0, 2) - R_err(2, 0),
        R_err(1, 0) - R_err(0, 1);

    double norm_l = l.norm();
    double theta = std::atan2(norm_l, trace - 1.0);

    Eigen::Vector3d phi;
    phi.setZero();
    const double epsilon = 1e-6;

    if (R_err.isIdentity() == true) // Case 1: R = I (no rotation)
    {
        phi.setZero();
    }
    else if (R_err.isDiagonal() == true) // Case 2: R is 180-degree flip (diagonal rotation matrix)
    {
        phi << (M_PI / 2.0) * (r11 + 1),
            (M_PI / 2.0) * (r22 + 1),
            (M_PI / 2.0) * (r33 + 1);
    }
    // Case 3: General case
    else
    {
        phi = theta * l / norm_l;
    }

    return phi;
}

Eigen::Matrix3d hat(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d m;
    m << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return m;
}

Eigen::Vector3d vee(const Eigen::Matrix3d &M)
{
    Eigen::Vector3d v;
    v << M(2, 1), M(0, 2), M(1, 0);
    return v;
}

Eigen::Matrix2d rotZaxis2d(double theta)
{
    Eigen::Matrix2d R;
    R << cos(theta), -sin(theta),
        sin(theta), cos(theta);
    return R;
}

double cubicBezierPolynomial(double current_time, double start_time, double end_time, double p_init, double p_mid, double p_end)
{
    double x_t;

    double P0 = p_init;
    double P1 = p_mid;
    double P2 = p_mid;
    double P3 = p_end;

    if (current_time < start_time)
    {
        x_t = P0;
    }
    else if (current_time > end_time)
    {
        x_t = P3;
    }
    else
    {
        double elapsed_time = current_time - start_time;
        double total_time = end_time - start_time;

        double t = elapsed_time / total_time;

        double coeff0 = P0;
        double coeff1 = (-3 * P0 + 3 * P1);
        double coeff2 = (3 * P0 - 6 * P1 + 3 * P2);
        double coeff3 = (-1 * P0 + 3 * P1 - 3 * P2 + P3);

        x_t = coeff0 + coeff1 * t + coeff2 * t * t + coeff3 * t * t * t;
    }

    return x_t;
}

double cubicDotBezierPolynomial(double current_time, double start_time, double end_time, double p_init, double p_mid, double p_end)
{
    double x_t;

    double P0 = p_init;
    double P1 = p_mid;
    double P2 = p_mid;
    double P3 = p_end;

    if (current_time < start_time)
    {
        x_t = 3 * P1 - 3 * P0;
    }
    else if (current_time > end_time)
    {
        x_t = 3 * P3 - 3 * P2;
    }
    else
    {
        double elapsed_time = current_time - start_time;
        double total_time = end_time - start_time;

        double t = elapsed_time / total_time;

        double coeff0 = (-3 * P0 + 3 * P1);
        double coeff1 = (6 * P0 - 12 * P1 + 6 * P2);
        double coeff2 = (9 * P1 - 3 * P0 - 9 * P2 + 3 * P3);

        x_t = coeff0 + coeff1 * t + coeff2 * t * t;
    }

    return x_t;
}

Eigen::Quaterniond integrateQuatBodyExp(const Eigen::Quaterniond &q, const Eigen::Vector3d &omega_body, double dt)
{
    const double norm_w = omega_body.norm();
    Eigen::Quaterniond dq;

    if (norm_w < 1e-12)
    {
        // small-angle approximation
        Eigen::Vector3d half = 0.5 * omega_body * dt;
        dq = Eigen::Quaterniond(1.0, half.x(), half.y(), half.z());
    }
    else
    {
        const double theta = norm_w * dt;
        const Eigen::Vector3d axis = omega_body / norm_w;
        const double half = 0.5 * theta;
        const double s = std::sin(half);
        dq = Eigen::Quaterniond(std::cos(half),
                                axis.x() * s,
                                axis.y() * s,
                                axis.z() * s);
    }

    Eigen::Quaterniond q_next = (q * dq).normalized();
    return q_next;
}