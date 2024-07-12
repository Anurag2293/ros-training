#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include "std_srvs/Empty.h"
#include "turtlesim/Pose.h"
#include "turtle_move/MoveTo.h"

class TurtleBot
{
public:
    ros::NodeHandle nh; 

    TurtleBot()
        : pose(turtlesim::Pose()), nh("~")  // Private node handle
    {
        // Publisher to topic "/turtle1/cmd_vel"
        vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
        // Subscriber to topic "/turtle1/pose"
        pose_sub = nh.subscribe("/turtle1/pose", 10, &TurtleBot::poseCallback, this);
    }

    void poseCallback(const turtlesim::Pose::ConstPtr& msg)
    {
        pose = *msg; 
    }

    double euclideanDistance(const turtlesim::Pose& goal_pose)
    {
        return std::sqrt(
            std::pow(goal_pose.x - pose.x, 2) +
            std::pow(goal_pose.y - pose.y, 2)
        );
    }

    double linearVel(const turtlesim::Pose& goal_pose, double constant = 1.5)
    {
        return constant * euclideanDistance(goal_pose);
    }

    double steeringAngle(const turtlesim::Pose& goal_pose)
    {
        return std::atan2(goal_pose.y - pose.y, goal_pose.x - pose.x);
    }

    double angularVel(const turtlesim::Pose& goal_pose, double constant = 6)
    {
        return constant * (steeringAngle(goal_pose) - pose.theta);
    }

    bool move2goal(turtle_move::MoveTo::Request &req, turtle_move::MoveTo::Response &res)
    {
        turtlesim::Pose goal_pose;
        goal_pose.x = req.x;
        goal_pose.y = req.y;

        double distance_tolerance;
        distance_tolerance = req.tolerance;

        ros::Rate loop_rate(10);
        geometry_msgs::Twist vel_msg;

        while (euclideanDistance(goal_pose) >= distance_tolerance)
        {
            // Linear velocity
            vel_msg.linear.x = linearVel(goal_pose);
            vel_msg.linear.y = 0;
            vel_msg.linear.z = 0;

            // Angular velocity
            vel_msg.angular.x = 0;
            vel_msg.angular.y = 0;
            vel_msg.angular.z = angularVel(goal_pose);

            vel_pub.publish(vel_msg);
            
            ros::spinOnce(); // Important!
            loop_rate.sleep();
        }

        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        vel_pub.publish(vel_msg); // Stop the robot

        res.success = true;
        return true;
    }

private:
    ros::Publisher vel_pub;
    ros::Subscriber pose_sub;
    turtlesim::Pose pose;
};


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "turtle_move_goal");
    ros::NodeHandle nhPublic;

    TurtleBot tb;
    ros::ServiceServer service = nhPublic.advertiseService("/move_to", &TurtleBot::move2goal, &tb);

    ROS_INFO("Ready to move the turtle to goal!");

    ros::spin();

    return 0;
}