#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

ros::Publisher twist_pub;

void twist_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    geometry_msgs::Twist twist;
    twist.linear.x = msg->linear.x;
    twist.linear.y = msg->linear.y;
    twist.linear.z = msg->linear.z;
    twist.angular.x = msg->angular.x;
    twist.angular.y = msg->angular.y;
    twist.angular.z = msg->angular.z;
    twist_pub.publish(twist);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_actuator");
    ros::NodeHandle nh;
    twist_pub = nh.advertise<geometry_msgs::Twist>("/twist_controller/command",1);
    ros::Subscriber sub = nh.subscribe("/twist/cmd",1, twist_cb);
    ros::spin();
}