/**
 * This module calculates the Forward Kinematics of the PSM
 * of the da Vinci Research Kit Robot.
 */

 #ifndef PSM_FK_HPP
 #define PSM_FK_HPP

#include "psm_defines.hpp"


namespace psm_dyn
{
    class Robot_SA
    {
    public:
        Robot_SA(const psm_dyn::MatrixNd& in_screw_axes,
                 const Eigen::Matrix4d& in_home_config);

        int dof();

        bool compute_FK(const psm_dyn::VectorNd& in_joint_pos,
                        Eigen::Matrix4d& out_ht);

        bool simulate();

    private:
        psm_dyn::MatrixNd m_screw_axes;

        Eigen::Matrix4d m_home_config;

        psm_dyn::VectorNd m_current_joint_pos;

        int m_dof;

    };

}    // namespace psm_dyn

#endif      // #ifndef PSM_FK_HPP
