/**
 * This module communicates with the ambf python server using ROS framework
 */

 #ifndef PSM_ROS_CLIENT_HPP
 #define PSM_ROS_CLIENT_HPP

#include "psm_defines.hpp"
#include "ros/ros.h"
#include "psm_dyn/joint_pos.h"
#include <memory>

namespace psm_dyn
{
    class PSM_ROS_Client
    {
    public:
        PSM_ROS_Client();

        bool set_joint_pos(const psm_dyn::VectorNd& in_joint_pos);

    private:
        std::unique_ptr<ros::NodeHandle> m_node;

        ros::ServiceClient m_psm_ros_client;
    };

}    // namespace psm_dyn


#endif      // #ifndef PSM_ROS_CLIENT_HPP
