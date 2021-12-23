/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   fk_server.cpp
 * Author:  Mario Cerino
 * Org.:    UNISA
 * Date:    Nov 25, 2020
 *
 * ROS service server that computes forward kinematics of a given
 * joint-space configuration, provided by a service client.
 * 
 * -------------------------------------------------------------------
 */

#include <ros/ros.h>
#include <kinematics_service_msgs/GetForwardKinematicSolution.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <eigen_conversions/eigen_msg.h>

bool computeForwardKinematicSolution(kinematics_service_msgs::GetForwardKinematicSolutionRequest & req, kinematics_service_msgs::GetForwardKinematicSolutionResponse & res);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinematics_service_server");
    ros::NodeHandle nh;
    
    ros::ServiceServer service_server = nh.advertiseService("compute_forward_kinematics", computeForwardKinematicSolution);

    ROS_INFO("Started forward kinematics service");

    ros::spin();

    ros::shutdown();
    return 0;
}

bool computeForwardKinematicSolution(kinematics_service_msgs::GetForwardKinematicSolutionRequest & req, kinematics_service_msgs::GetForwardKinematicSolutionResponse & res)
{
    // Load the robot model
    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader = std::shared_ptr<const robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader("robot_description"));

    // Get the robot kinematic model
    robot_model::RobotModelConstPtr kinematic_model = robot_model_loader->getModel();

    // Create default robot state from kinematic model
    moveit::core::RobotState robot_state(kinematic_model);

    // Convert robot state message to RobotState
    moveit::core::robotStateMsgToRobotState(req.robot_state, robot_state);

    robot_state.updateLinkTransforms();

    // Get the planning group name from the parameter server
    ros::NodeHandle nh;
    std::string planning_group_name;
    
    if(!nh.getParam("planning_group_name", planning_group_name))
    {
        ROS_ERROR("'planning_group_name' is undefined on the parameter server");
        return false;
    }

    // Get the planning group
    const robot_state::JointModelGroup * joint_model_group = kinematic_model->getJointModelGroup(planning_group_name);

    std::vector<std::string> link_names = joint_model_group->getLinkModelNames();

    // Compute FK and prepare the response message
    Eigen::Isometry3d forward_kinematics = robot_state.getGlobalLinkTransform(link_names.back());

    tf::poseEigenToMsg(forward_kinematics, res.end_effector_pose);

    return true;
}