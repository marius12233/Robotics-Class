/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   planner.cpp
 * Author:  Mario Cerino
 * Org.:    UNISA
 * Date:    Dec 9, 2020
 *
 * This is a trajectory planning node that loads a trajectory from
 * file in .traj format and uses the WorkspaceTrajectory class from
 * the moveit_dp_redundancy_resolution package to load it. Then, the
 * node performs time parametrization and verifies that joint limits
 * are respected.
 * 
 * -------------------------------------------------------------------
 */

#include <ros/package.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <angles/angles.h>
#include <moveit_dp_redundancy_resolution/workspace_trajectory.h>

void computeVectorsDifference(
    Eigen::VectorXd & diff,
    const Eigen::VectorXd & minuend,
    const Eigen::VectorXd & subtrahend,
    const moveit::core::JointModelGroup * jmg);

int main(int argc, char** argv)
{
    // Initializing the node and the move_group interface

    ros::init(argc, argv, "unisa_trajectory_planner");
    ros::NodeHandle node_handle;

    /*
     * The async spinner spawns a new thread in charge of calling callbacks
     * when needed. Uncomment the lines below if, as instance, you need to
     * ask for the robot state and expect an answer before the time expires.
     */

    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::string planning_group_name;

    if(!node_handle.getParam("planning_group_name", planning_group_name))
    {
        ROS_ERROR("Cannot retrieve parameter 'planning_group_name' from the parameter server");
    }

    std::string trajectory_filename;

    if(!node_handle.getParam("trajectory_filename", trajectory_filename))
    {
        ROS_ERROR("Cannot retrieve parameter 'trajectory_filename' from the parameter server");
    }

    // Load the robot model
    robot_model_loader::RobotModelLoaderConstPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));

    // Get the robot kinematic model
    robot_model::RobotModelConstPtr kinematic_model = robot_model_loader->getModel();

    // Get the planning group
    const robot_state::JointModelGroup * joint_model_group = kinematic_model->getJointModelGroup(planning_group_name);

    // Compose trajectory path
    std::string package_path = ros::package::getPath("unisa_trajectory_planner");
    std::string trajectory_file_path = package_path + "/data/" + trajectory_filename;

    // Load workspace trajectory and adjust duration
    moveit_dp_redundancy_resolution::WorkspaceTrajectory ws_trajectory("yz_circular", trajectory_file_path);
    moveit_dp_redundancy_resolution_msgs::WorkspaceTrajectory ws_trajectory_msg;
    ws_trajectory.getWorkspaceTrajectoryMsg(ws_trajectory_msg);

    ROS_INFO_STREAM("Trajectory duration is " << ws_trajectory.getDuration() << " s");

    // Publish workspace trajectory to be visualized in RViz
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("world");

    visual_tools.deleteAllMarkers();
    visual_tools.publishPath(ws_trajectory.getWaypoints(), rvt::LIME_GREEN, rvt::SMALL);
    visual_tools.trigger();

    // Create empty joint-space trajectory
    robot_trajectory::RobotTrajectory robot_trajectory(kinematic_model, joint_model_group);

    // Compute IK and verify limits
    double dt = 0;
    Eigen::VectorXd joint_positions_prev = Eigen::VectorXd::Zero(joint_model_group->getVariableCount());
    Eigen::VectorXd joint_positions_curr = Eigen::VectorXd::Zero(joint_model_group->getVariableCount());
    Eigen::VectorXd joint_velocities_prev = Eigen::VectorXd::Zero(joint_model_group->getVariableCount());
    Eigen::VectorXd joint_velocities_curr = Eigen::VectorXd::Zero(joint_model_group->getVariableCount());
    Eigen::VectorXd joint_accelerations = Eigen::VectorXd::Zero(joint_model_group->getVariableCount());

    for(int i=0; i < ws_trajectory_msg.timestamps.size(); i++)
    {
        moveit::core::RobotState robot_state(kinematic_model);

        if(!robot_state.setFromIK(joint_model_group, ws_trajectory_msg.waypoints[i]))
            ROS_WARN_STREAM("Could not compute IK solution for waypoint " << i);

        if(i > 0)
        {
            dt = ws_trajectory_msg.timestamps[i] - ws_trajectory_msg.timestamps[i-1];

            robot_state.copyJointGroupPositions(joint_model_group, joint_positions_curr);

            computeVectorsDifference(joint_velocities_curr, joint_positions_curr, joint_positions_prev, joint_model_group);
            joint_velocities_curr = joint_velocities_curr/dt;

            joint_accelerations = (joint_velocities_curr - joint_velocities_prev)/dt;

            robot_state.setJointGroupVelocities(joint_model_group, joint_velocities_curr);
            robot_state.setJointGroupAccelerations(joint_model_group, joint_accelerations);
        }

        robot_trajectory.addSuffixWayPoint(robot_state, dt);

        joint_positions_prev = joint_positions_curr;
        joint_velocities_prev = joint_velocities_curr;
    }

    moveit_msgs::RobotTrajectory robot_trajectory_msg;

    robot_trajectory.getRobotTrajectoryMsg(robot_trajectory_msg);

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to publish the joint space solution");

    // Prepare display trajectory message and publish it
    ros::Publisher display_path_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>(planning_pipeline::PlanningPipeline::DISPLAY_PATH_TOPIC, 10, true);

    moveit_msgs::DisplayTrajectory display_trajectory_msg;

    display_trajectory_msg.model_id = kinematic_model->getName();
    display_trajectory_msg.trajectory.resize(1, robot_trajectory_msg);
    robot_state::robotStateToRobotStateMsg(robot_trajectory.getFirstWayPoint(), display_trajectory_msg.trajectory_start);
    display_path_publisher.publish(display_trajectory_msg);

    // Publish joint space solution to plot in rqt_multiplot
    ros::Publisher plot_trajectory_publisher = node_handle.advertise<trajectory_msgs::JointTrajectoryPoint>("plot_planned_trajectory", 10000, true);

    ros::Duration sleep_time(0.05);

    for(int i=0; i < robot_trajectory_msg.joint_trajectory.points.size(); i++)
    {
        trajectory_msgs::JointTrajectoryPoint jtp = robot_trajectory_msg.joint_trajectory.points[i];

        plot_trajectory_publisher.publish(jtp);

        sleep_time.sleep();
    }

    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to terminate the planner");

    spinner.stop();
    ros::shutdown();
    exit(0);

}

void computeVectorsDifference(
    Eigen::VectorXd & diff,
    const Eigen::VectorXd & minuend,
    const Eigen::VectorXd & subtrahend,
    const moveit::core::JointModelGroup * jmg)
{
    for (int i = 0; i < minuend.size(); i++)
    {
        if (jmg->getActiveJointModels()[i]->getType() == robot_model::JointModel::REVOLUTE)
        {
            // Compute difference between revolute joints
            robot_model::VariableBounds bounds = jmg->getParentModel().getVariableBounds(jmg->getVariableNames()[i]);

            if(bounds.position_bounded_)
            {
                angles::shortest_angular_distance_with_limits(
                    angles::normalize_angle(subtrahend[i]), 
                    angles::normalize_angle(minuend[i]), 
                    angles::normalize_angle(bounds.min_position_), 
                    angles::normalize_angle(bounds.max_position_), 
                    diff[i]);
            }
            else
                diff[i] = angles::shortest_angular_distance(subtrahend[i], minuend[i]);
        }
        else if (jmg->getActiveJointModels()[i]->getType() == robot_model::JointModel::PLANAR)
        {
            ROS_ERROR("Planar joints are not currently supported.");
        }
        else
        {
            // ! Other joint model types are included here (PRISMATIC, FIXED, UNKNOWN, etc.)
            diff[i] = minuend[i] - subtrahend[i];
        }
    }
}