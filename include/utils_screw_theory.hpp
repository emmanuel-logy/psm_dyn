/**
 * This module contains the frequently used mathematical functions to
 * compute Kinematics and Dynamics of a given robot using screw axis thoery
 */

#ifndef PSM_UTILS_SCREW_THEORY_HPP
#define PSM_UTILS_SCREW_THEORY_HPP

#include "psm_defines.hpp"

namespace psm_dyn
{
    class Utils_Screw_Theory
    {
    public:
        Utils_Screw_Theory();

        bool axisangle2rot(const Eigen::Vector3d& in_omega,
                           const double& in_theta,
                           Eigen::Matrix3d& out_rot);

        bool twist_to_ht(const psm_dyn::Vector6d& in_screw_axis,
                         const double& in_theta,
                         Eigen::Matrix4d& out_ht);

    private:

    };

}    // namespace psm_dyn

#endif      // #ifndef PSM_UTILS_SCREW_THEORY_HPP
