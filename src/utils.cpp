/**
 * This module contains the frequently used mathematical functions to
 * compute Kinematics and Dynamics of a given robot
 */

#include "utils.hpp"

namespace psm_dyn
{

    bool Utils::tdh( const double& in_theta,
                     const double& in_d,
                     const double& in_a,
                     const double& in_alpha,
                     const bool& in_modified,
                     Eigen::Matrix4d& out_ht)
    {
        bool retVal = true;
        out_ht = Eigen::Matrix4d::Zero();

        if ( !in_modified )
        {
            out_ht <<   cos(in_theta),  -sin(in_theta)*cos(in_alpha),   sin(in_theta)*sin(in_alpha),    in_a*cos(in_theta),
                         sin(in_theta), cos(in_theta)*cos(in_alpha),    -cos(in_theta)*sin(in_alpha),   in_a*sin(in_theta),
                         0,             sin(in_alpha),                  cos(in_alpha),                  in_d,
                         0,             0,                              0,                              1;
        }
        else
        {
            out_ht <<   cos(in_theta),                  -sin(in_theta),                 0,                  in_a,
                        sin(in_theta)*cos(in_alpha),    cos(in_theta)*cos(in_alpha),   -sin(in_alpha),      -in_d*sin(in_alpha),
                        sin(in_theta)*sin(in_alpha),    cos(in_theta)*sin(in_alpha),   cos(in_alpha),       in_d*cos(in_alpha),
                        0,                              0,                              0,                  1;
         }

        return retVal;
    }


    bool Utils::ht_to_pose( const Eigen::Matrix4d& in_ht,
                            psm_dyn::Vector7d& out_pose)
    {
        bool retVal = true;
        out_pose = psm_dyn::Vector7d::Zero();

        Eigen::Matrix<double, 3, 3> rot_mat = in_ht.block<3,3>(0,0);    // Extract in_a block of matrix of size 3*3 from index [0,0] of T
        Eigen::Quaternion<double> quats(rot_mat);

        out_pose << in_ht(0,3),
                    in_ht(1,3),
                    in_ht(2,3),
                    quats.x(),
                    quats.y(),
                    quats.z(),
                    quats.w();

        return retVal;
    }


    bool Utils::eigenPose_to_rosPose(psm_dyn::Vector7d& in_eigenPose,
                                     geometry_msgs::Pose& out_rosPose)
    {
        bool retVal = true;
        out_rosPose = {};

        out_rosPose.position.x =  in_eigenPose(0);
        out_rosPose.position.y = in_eigenPose(1);
        out_rosPose.position.z = in_eigenPose(2);
        out_rosPose.orientation.x = in_eigenPose(3);
        out_rosPose.orientation.y = in_eigenPose(4);
        out_rosPose.orientation.z = in_eigenPose(5);
        out_rosPose.orientation.w = in_eigenPose(6);

        return retVal;
    }

}    // namespace psm_dyn
