/**
 * This module contains the frequently used mathematical functions to
 * compute Kinematics and Dynamics of a given robot
 */

#ifndef PSM_UTILS_HPP
#define PSM_UTILS_HPP

#include "psm_defines.hpp"
#include "geometry_msgs/Pose.h"

namespace psm_dyn
{
    class Utils
    {
    public:
        bool tdh(const double& in_theta,
                 const double& in_d,
                 const double& in_a,
                 const double& in_alpha,
                 const bool& in_modified,
                 Eigen::Matrix4d& out_ht);

        bool ht_to_pose(const Eigen::Matrix4d& in_ht,
                        psm_dyn::Vector7d& out_pose);

        bool eigenPose_to_rosPose(psm_dyn::Vector7d& in_eigenPose,
                                  geometry_msgs::Pose& out_rosPose);

    private:

    };

}    // namespace psm_dyn

#endif      // #ifndef PSM_FK_HPP
