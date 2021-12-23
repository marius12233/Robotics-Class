/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   ik_server.cpp
 * Author:  Mario Cerino
 * Org.:    UNISA
 * Date:    Oct 7, 2020
 *
 * ROS service server that computes one of the inverse kinematics
 * solutions related to a given end-effector pose.
 * 
 * -------------------------------------------------------------------
 */

#include <ros/ros.h>
#include <kinematics_service_msgs/GetInverseKinematicSolution.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

bool computeInverseKinematicSolution(kinematics_service_msgs::GetInverseKinematicSolutionRequest & req, kinematics_service_msgs::GetInverseKinematicSolutionResponse & res);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinematics_service_server");
    ros::NodeHandle nh;
    
    ros::ServiceServer service_server = nh.advertiseService("compute_inverse_kinematics", computeInverseKinematicSolution);

    ROS_INFO("Started inverse kinematics service");

    ros::spin();

    ros::shutdown();
    return 0;
}

bool computeInverseKinematicSolution(kinematics_service_msgs::GetInverseKinematicSolutionRequest & req, kinematics_service_msgs::GetInverseKinematicSolutionResponse & res)
{
    // Load the robot model
    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader = std::shared_ptr<const robot_model_loader::RobotModelLoader>(new robot_model_loader::RobotModelLoader("robot_description"));

    // Get the robot kinematic model
    robot_model::RobotModelConstPtr kinematic_model = robot_model_loader->getModel();

    moveit::core::RobotState robot_state(kinematic_model);

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

    robot_state.setFromIK(joint_model_group, req.end_effector_pose);

    moveit::core::robotStateToRobotStateMsg(robot_state, res.robot_state);

    return true;
}