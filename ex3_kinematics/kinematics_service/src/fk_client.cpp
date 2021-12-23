/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   fk_client.cpp
 * Author:  Mario Cerino
 * Org.:    UNISA
 * Date:    Nov 25, 2020
 *
 * ROS service client that exploits a custom service to compute a
 * forward kinematic solution and compares it with the one returned by
 * the /compute_fk service of the move_group node.
 * 
 * -------------------------------------------------------------------
 */

#include <ros/ros.h>
#include <kinematics_service_msgs/GetForwardKinematicSolution.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_msgs/GetPositionFK.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinematics_service_client");
    ros::NodeHandle nh;
    
    ros::ServiceClient client = nh.serviceClient<kinematics_service_msgs::GetForwardKinematicSolution>("compute_forward_kinematics");

    // Prepare the service request
    kinematics_service_msgs::GetForwardKinematicSolution fk_service;

    // Load the robot model
    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader = std::shared_ptr<const robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader("robot_description"));

    // Get the robot kinematic model
    robot_model::RobotModelConstPtr kinematic_model = robot_model_loader->getModel();

    // Create default robot state from kinematic model
    moveit::core::RobotState robot_state(kinematic_model);

    const std::vector<double> joint_positions = {0, 0.341793, 0.00991429, 0, -1.9225, -3.14159};

    // Get the planning group name from the parameter server
    std::string planning_group_name;
    
    if(!nh.getParam("planning_group_name", planning_group_name))
    {
        ROS_ERROR("'planning_group_name' is undefined on the parameter server");
        return false;
    }

    // Get the planning group
    const robot_state::JointModelGroup * joint_model_group = kinematic_model->getJointModelGroup(planning_group_name);

    robot_state.setJointGroupPositions(joint_model_group, joint_positions);

    moveit::core::robotStateToRobotStateMsg(robot_state, fk_service.request.robot_state);

    if(!client.call(fk_service))
        ROS_ERROR("Could not compute forward kinematics");

    // Convert orientation to RPY
    tf2::Quaternion quaternion;
    tf2::fromMsg(fk_service.response.end_effector_pose.orientation, quaternion);
    
    tf2::Matrix3x3 matrix(quaternion);
    tf2Scalar roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);

    // Print solution to stdout
    std::ostringstream output_msg;

    output_msg << "Computed forward kinematic solution with custom solver:"  << std::endl;
    output_msg << "Position (XYZ): ["; 
    output_msg << fk_service.response.end_effector_pose.position.x << ", ";
    output_msg << fk_service.response.end_effector_pose.position.y << ", ";
    output_msg << fk_service.response.end_effector_pose.position.z << "]" << std::endl;
    output_msg << "Orientation (RPY): [" << roll << ", " << pitch << ", " << yaw << "]";

    ROS_INFO_STREAM(output_msg.str());

    // Compare result with that of move_group's /compute_fk

    client = nh.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");

    std::vector<std::string> link_names = joint_model_group->getLinkModelNames();

    moveit_msgs::GetPositionFK move_group_fk_service;
    move_group_fk_service.request.header.frame_id = link_names[0];
    move_group_fk_service.request.fk_link_names.push_back(link_names.back());
    moveit::core::robotStateToRobotStateMsg(robot_state, move_group_fk_service.request.robot_state);

    if(!client.call(move_group_fk_service))
        ROS_ERROR("Could not compute forward kinematics");

    // Convert orientation to RPY
    tf2::fromMsg(move_group_fk_service.response.pose_stamped[0].pose.orientation, quaternion);
    
    tf2::Matrix3x3 rotation_matrix(quaternion);
    rotation_matrix.getRPY(roll, pitch, yaw);

    // Print solution to stdout
    std::ostringstream msg;

    msg << "Computed forward kinematic solution with move_group solver:"  << std::endl;
    msg << "Position (XYZ): ["; 
    msg << move_group_fk_service.response.pose_stamped[0].pose.position.x << ", ";
    msg << move_group_fk_service.response.pose_stamped[0].pose.position.y << ", ";
    msg << move_group_fk_service.response.pose_stamped[0].pose.position.z << "]" << std::endl;
    msg << "Orientation (RPY): [" << roll << ", " << pitch << ", " << yaw << "]";

    ROS_INFO_STREAM(msg.str());

    ros::shutdown();
    return 0;
}