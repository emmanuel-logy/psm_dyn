/**
 * This module communicates with the ambf python server using ROS framework
 */
 
#include "psm_ros_client.hpp"

namespace psm_dyn
{

    PSM_ROS_Client::PSM_ROS_Client()
    {
        int argc = 0;
        char **argv = nullptr;

        ros::init(argc, argv, "psm_ros_client", ros::init_options::AnonymousName);
        m_node = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle);
        m_psm_ros_client = m_node->serviceClient<psm_dyn::joint_pos>("psm_ros_server");
    }

    bool PSM_ROS_Client::set_joint_pos(const psm_dyn::VectorNd& in_joint_pos)
    {
        bool retVal = true;

        psm_dyn::joint_pos srv;
        srv.request.j1 = -0.2;   // srv.request.j1 = in_joint_pos(0);
        srv.request.j2 = in_joint_pos(1);
        srv.request.j3 = in_joint_pos(2);
        srv.request.j4 = in_joint_pos(3);
        srv.request.j5 = in_joint_pos(4);
        srv.request.j6 = in_joint_pos(5);
        srv.request.j7 = in_joint_pos(6);

        std::cout << "Calling psm_client" << std::endl;
        m_psm_ros_client.call(srv);

        return retVal;
    }

}    // namespace psm_dyn
