#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "std_srvs/Empty.h"
#include "turtle_move/MoveStraight.h"

bool moveStraight(turtle_move::MoveStraight::Request &req, turtle_move::MoveStraight::Response &res);

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "turtle_move_straight");

    ros::NodeHandle nh;

    // Create service server for moving straight
    ros::ServiceServer service = nh.advertiseService("/move_straight", moveStraight);
    ROS_INFO("Ready to move the turtle straight.");

    ros::spin();
    // ros::shutdown();
    return 0;
}

bool moveStraight(turtle_move::MoveStraight::Request &req, turtle_move::MoveStraight::Response &res)
{
    ros::NodeHandle nh;
    ros::Publisher velocity_publisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    geometry_msgs::Twist vel_msg; 

    // Setting the linear velocity in x direction
    if (req.isForward) {
        vel_msg.linear.x = req.speed;
    } else {
        vel_msg.linear.x = -req.speed;
    }
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    // Loop to move the turtle
    double t0 = ros::Time::now().toSec();
    double current_distance = 0.0;

    ros::Rate loop_rate(10);
    while (current_distance < req.distance) {
        velocity_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_distance = abs(req.speed * (t1 - t0));
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Stop the turtle after moving the required distance
    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);

    res.success = true;
    return true;
}