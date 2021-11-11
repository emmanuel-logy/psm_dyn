/**
 * This module calculates the Forward Kinematics of the PSM
 * of the da Vinci Research Kit Robot.
 */

#include "psm_kinematics.hpp"

namespace psm_dyn
{

    Robot_SA::Robot_SA(const psm_dyn::MatrixNd& in_screw_axes,
                       const Eigen::Matrix4d& in_home_config) : m_screw_axes(in_screw_axes),
                                                                m_home_config(in_home_config),
                                                                m_current_joint_pos(in_screw_axes.cols()),
                                                                m_dof(in_screw_axes.cols())
    {

    }


    int Robot_SA::dof()
    {
        return m_dof;
    }


    bool Robot_SA::compute_FK(const psm_dyn::VectorNd& in_joint_pos,
                              Eigen::Matrix4d& out_ht)
    {
        bool retVal = true;
        out_ht = {};

        this->m_current_joint_pos = in_joint_pos;

        out_ht = Eigen::Matrix4d::Identity();
        for (int i=0; i<in_joint_pos.rows(); ++i)
        {
            // out_ht = out_ht * twist_to_ht();
        }


        return retVal;
    }

}       // namespace psm_dyn

// int main(int argc, char **argv)
// {
//     psm_dyn::MatrixNd b = Eigen::Matrix<double, 6, 7>::Zero();
//     std::cout << b.rows() << " " << b.cols() << " " << b.size() << std::endl;
//     std::cout << b << std::endl;
//
//     return 0;
// }
