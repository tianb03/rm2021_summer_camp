#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
int main(int argc,char **argv)
{
    ros::init(argc,argv,"vp");
    ros::NodeHandle n;
    ros::Publisher turtle;
    turtle = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);
    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        /* code for loop body */
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x=0.5;
        vel_msg.angular.z=0.2;

        turtle.publish(vel_msg);
        ROS_INFO("publish turtle x:%0.2f m/s,z:%0.2f rads",vel_msg.linear.x,vel_msg.angular.z);
        loop_rate.sleep();
    }
    
}