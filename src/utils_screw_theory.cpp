/**
 * This module calculates the Forward Kinematics of the PSM
 * of the da Vinci Research Kit Robot.
 */

#include "utils_screw_theory.hpp"
#include <cmath>

namespace psm_dyn
{

    Utils_Screw_Theory::Utils_Screw_Theory()
    {

    }


    bool Utils_Screw_Theory::axisangle2rot(const Eigen::Vector3d& in_omega,
                                           const double& in_theta,
                                           Eigen::Matrix3d& out_rot)
    {
        bool retVal = true;
        out_rot = Eigen::Matrix3d::Zero();

        Eigen::Matrix<double, 3, 3> omega_ss {};
        omega_ss << 0,          -in_omega(2),      in_omega(1),
                    in_omega(2),   0,              -in_omega(0),
                    -in_omega(1),  in_omega(0),       0;


        out_rot =   Eigen::Matrix<double, 3, 3>::Identity() +
                    sin(in_theta) * omega_ss                   +
                    (1-cos(in_theta)) * (omega_ss*omega_ss);

        return retVal;
    }


    bool Utils_Screw_Theory::twist_to_ht(const psm_dyn::Vector6d& in_screw_axis,
                                         const double& in_theta,
                                         Eigen::Matrix4d& out_ht)
    {
        bool retVal = true;
        out_ht = Eigen::Matrix4d::Zero();

        Eigen::Vector3d omega = in_screw_axis.head<3>();
        Eigen::Vector3d vel = in_screw_axis.tail<3>();
        Eigen::Matrix3d rot = Eigen::Matrix3d::Zero();
        axisangle2rot(omega, in_theta, rot);

        Eigen::Matrix<double, 3, 3> omega_ss {};
        omega_ss << 0,          -omega(2),      omega(1),
                    omega(2),   0,              -omega(0),
                   -omega(1),  omega(0),       0;
        Eigen::Matrix3d term1 = (1-cos(in_theta))*omega_ss;
        Eigen::Matrix3d term2 = (in_theta-sin(in_theta))*omega_ss*omega_ss;
        Eigen::Vector3d p = (Eigen::Matrix3d::Identity()*in_theta + term1 + term2)*vel;

        out_ht << rot,     p,
                  0, 0, 0, 1;

        return retVal;
    }
}       // namespace psm_dyn
