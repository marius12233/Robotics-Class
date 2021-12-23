/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   subscriber.cpp
 * Author:  Mario Cerino
 * Org.:    UNISA
 * Date:    Sep 30, 2020
 *
 * ROS Subscriber example.
 * 
 * -------------------------------------------------------------------
 */

#include <ros/ros.h>
#include <publisher_subscriber_msgs/JointPositions.h>

void messageReceived(const publisher_subscriber_msgs::JointPositions & msg);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber subscriber = nh.subscribe("joint_positions", 1, messageReceived);
    
    ros::spin();
    ros::shutdown();
    return 0;
}

void messageReceived(const publisher_subscriber_msgs::JointPositions & msg)
{
    ROS_INFO_STREAM(
        std::endl << "Joint positions: " << std::endl 
        << "1: " << msg.encoder_readings[0] << std::endl
        << "2: " << msg.encoder_readings[1] << std::endl
        << "3: " << msg.encoder_readings[2] << std::endl
        << "4: " << msg.encoder_readings[3] << std::endl
        << "5: " << msg.encoder_readings[4] << std::endl
        << "6: " << msg.encoder_readings[5] << std::endl);
}