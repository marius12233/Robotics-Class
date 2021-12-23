/* -------------------------------------------------------------------
 *
 * This module has been developed by the Automatic Control Group
 * of the University of Salerno, Italy.
 *
 * Title:   publisher.cpp
 * Author:  Mario Cerino
 * Org.:    UNISA
 * Date:    Sep 30, 2020
 *
 * ROS Publisher example.
 * 
 * -------------------------------------------------------------------
 */

#include <ros/ros.h>
#include <publisher_subscriber_msgs/JointPositions.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_publisher");
    ros::NodeHandle nh;
    ros::Publisher publisher = nh.advertise<publisher_subscriber_msgs::JointPositions>("joint_positions", 1);
    ros::Rate rate(1);

    publisher_subscriber_msgs::JointPositions joint_positions_msg;
    joint_positions_msg.encoder_readings.resize(6);
    joint_positions_msg.encoder_readings[0] = 0.0;
    joint_positions_msg.encoder_readings[1] = 0.0;
    joint_positions_msg.encoder_readings[2] = 0.0;
    joint_positions_msg.encoder_readings[3] = 0.0;
    joint_positions_msg.encoder_readings[4] = 0.0;
    joint_positions_msg.encoder_readings[5] = 0.0;

    while(ros::ok())
    {
        publisher.publish(joint_positions_msg);
        ros::spinOnce();
        rate.sleep();    

        joint_positions_msg.encoder_readings[0] += 0.01;
        joint_positions_msg.encoder_readings[1] += 0.01;
        joint_positions_msg.encoder_readings[2] += 0.01;
        joint_positions_msg.encoder_readings[3] += 0.01;
        joint_positions_msg.encoder_readings[4] += 0.01;
        joint_positions_msg.encoder_readings[5] += 0.01;   
    }

    ros::shutdown();
    return 0;
}