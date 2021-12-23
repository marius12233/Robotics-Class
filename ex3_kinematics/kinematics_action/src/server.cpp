/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   server.cpp
 * Author:  Mario Cerino
 * Org.:    UNISA
 * Date:    Oct 9, 2020
 *
 * ROS action server that computes all the inverse kinematic solutions
 * for a given end-effector pose.
 * 
 * -------------------------------------------------------------------
 */

#include <kinematics_action/inverse_kinematics_action.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinematics_action_server");
    
    kinematics_action::InverseKinematicsAction ik_action;

    ROS_INFO("Started inverse kinematics action server");

    ros::spin();

    ros::shutdown();
    return 0;
}