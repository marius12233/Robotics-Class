/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   ik_client.cpp
 * Author:  Mario Cerino
 * Org.:    UNISA
 * Date:    Oct 8, 2020
 *
 * ROS service client that exploits a service to compute an inverse
 * kinematics solution.
 * 
 * -------------------------------------------------------------------
 */

#include <ros/ros.h>
#include <kinematics_service_msgs/GetInverseKinematicSolution.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinematics_service_client");
    ros::NodeHandle nh;
    
    ros::ServiceClient client = nh.serviceClient<kinematics_service_msgs::GetInverseKinematicSolution>("compute_inverse_kinematics");

    // Prepare the service request
    kinematics_service_msgs::GetInverseKinematicSolution ik_service;

    ik_service.request.end_effector_pose.position.x = 1.0;
    ik_service.request.end_effector_pose.position.y = 0.0;
    ik_service.request.end_effector_pose.position.z = 1.0;
    
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, 0.0);

    ik_service.request.end_effector_pose.orientation = tf2::toMsg(quaternion);

    if(!client.call(ik_service))
        ROS_ERROR("Could not compute an inverse kinematic solution");

    std::ostringstream output_msg;

    output_msg << "Computed inverse kinematic solution: [";

    int n_joints = ik_service.response.robot_state.joint_state.position.size();

    for(int i=0; i < n_joints; i++)
    {
        output_msg << ik_service.response.robot_state.joint_state.position[i];

        if(i != n_joints - 1)
            output_msg << ", ";
    }

    output_msg << "]";

    ROS_INFO_STREAM(output_msg.str());

    ros::shutdown();
    return 0;
}