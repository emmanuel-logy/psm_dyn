/**
 * This module contains the unit test cases to validate the psm_fk class
 */

#include <catch2/catch.hpp>
#include "psm_kinematics.hpp"


TEST_CASE( "Verifying FK of PSM", "[PSM_FK]" )
{
    psm_dyn::MatrixNd screw_axes = Eigen::Matrix<double,6,7>::Zero();
    screw_axes <<   0,     -1,     0,      0,     -1,     0,       0,
                    -1,    0,      0,      0,     0,      -1,      0,
                    0,     0,      0,      -1,    0,      0,       -1,
                    0,     0,      0,      0,     0,      0.065,   0,
                    0,     0,      0,      0,     0.0156, 0,       0,
                    0,     0,      -1,     0,     0,      0,       0;

    Eigen::Matrix4d home_config = Eigen::Matrix4d::Identity();
    psm_dyn::Robot_SA psm1(screw_axes, home_config);

    REQUIRE( psm1.dof() == 7 );

    SECTION( "computing FK at home config" )
    {
        psm_dyn::VectorNd joint_pos = psm_dyn::VectorNd::Zero(7);
        Eigen::Matrix4d actual_ht = Eigen::Matrix4d::Zero();
        // std::cout << actual_ht << std::endl;
        // Eigen::Matrix4d expected_ht <<  ;

        REQUIRE( psm1.compute_FK(joint_pos, actual_ht) == true );
        // REQUIRE( actual_ht == expected_ht );
    }

    SECTION( "computing FK at some random config" )
    {
        psm_dyn::VectorNd joint_pos = psm_dyn::VectorNd::Random(7);
        // std::cout << joint_pos << std::endl;
        // Eigen::Matrix4d actual_ht = Eigen::Matrix4d::Zero();
        // Eigen::Matrix4d expected_ht <<  ;

        // REQUIRE( psm1.computeFK(joint_pos, actual_ht) == true );
        // REQUIRE( actual_ht == expected_ht );
    }
}
