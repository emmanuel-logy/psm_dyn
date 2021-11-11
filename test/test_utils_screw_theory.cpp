/**
 * This module contains the unit test cases to validate the psm_fk class
 */

#include <catch2/catch.hpp>
#include "utils_screw_theory.hpp"
#include "psm_defines.hpp"
using namespace psm_dyn;

TEST_CASE( "Testing axisangle2rot", "[UTILS_SCREW_THEORY]" )
{
    Utils_Screw_Theory utils_st;

    SECTION( "No rotation" )
    {
        double theta = 0 * psm_dyn::degToRad;
        Eigen::Vector3d omega {0, 0, 1};

        Eigen::Matrix3d actual_ht = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d expected_ht = Eigen::Matrix3d::Identity();

        REQUIRE( utils_st.axisangle2rot(omega, theta, actual_ht) == true );
        REQUIRE( actual_ht == expected_ht );
    }

    SECTION( "Rotation around z-axis by 30deg" )
    {
        double theta = 30 * psm_dyn::degToRad;
        Eigen::Vector3d omega {0, 0, 1};

        Eigen::Matrix3d actual_ht = Eigen::Matrix3d::Zero();
        Eigen::Matrix3d expected_ht;
        expected_ht <<  cos(theta), -sin(theta), 0,
                        sin(theta),  cos(theta), 0,
                        0,           0,          1;

        REQUIRE( utils_st.axisangle2rot(omega, theta, actual_ht) == true );
        REQUIRE( actual_ht == expected_ht );
    }
}


TEST_CASE( "Testing twist_to_ht", "[UTILS_SCREW_THEORY]" )
{
    Utils_Screw_Theory utils_st;

    SECTION( "Pure Rotation around z-axis by 30deg" )
    {
        psm_dyn::Vector6d screw_axis = psm_dyn::Vector6d::Zero();
        screw_axis(2) = 1;  // screw_axis = [0 0 1 0 0 0]';'
        double theta = 30 * psm_dyn::degToRad;

        Eigen::Matrix4d actual_ht = Eigen::Matrix4d::Zero();
        Eigen::Matrix4d expected_ht;
        expected_ht <<  cos(theta), -sin(theta), 0, 0,
                        sin(theta),  cos(theta), 0, 0,
                        0,           0,          1, 0,
                        0,           0,          0, 1;

        REQUIRE( utils_st.twist_to_ht(screw_axis, theta, actual_ht) == true );
        REQUIRE( actual_ht == expected_ht );
    }
}
