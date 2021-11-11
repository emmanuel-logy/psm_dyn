/**
 * This module contains the frequently used mathematical functions to
 * compute Kinematics and Dynamics of a given robot
 */

#ifndef PSM_DEFINES_HPP
#define PSM_DEFINES_HPP

#include <iostream>
#include <eigen3/Eigen/Dense>


namespace psm_dyn
{
    // Some helpful constants
    const double pi = 3.14;
    const double degToRad = pi/180;
    const double radToDeg = 180/pi;

    // Some helpful typedefs
    constexpr int robot_max_dof = 12;
    using MatrixNd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::StorageOptions::ColMajor, robot_max_dof, robot_max_dof>;
    using VectorNd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::StorageOptions::ColMajor, robot_max_dof, 1>;
    using Vector7d = Eigen::Matrix<double, 7, 1>;
    using Vector6d = Eigen::Matrix<double, 6, 1>;


}    // namespace psm_dyn

#endif      // #ifndef PSM_DEFINES_HPP
