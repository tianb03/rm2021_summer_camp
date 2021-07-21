#include<ros/ros.h>
#include<tf/transform_listener.h>
#include<geometry_msgs/Twist.h>
#include<turtlesim/Spawn.h>
int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");
  ros::NodeHandle node;
  ros::service::waitForService("spawn");
  ros::ServiceClient add_turtle = 
    node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn srv;
  add_turtle.call(srv);
  add_turtle.call(srv);
  ros::Publisher turtle_vel2 = 
    node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
  ros::Publisher turtle_vel3 = 
    node.advertise<geometry_msgs::Twist>("turtle3/cmd_vel", 10);
  tf::TransformListener listener2, listener3;
  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform2, transform3;
    try{
      listener2.lookupTransform("/turtle2", "/turtle2_target",  
                               ros::Time(0), transform2);
      listener3.lookupTransform("/turtle3", "/turtle3_target",  
                               ros::Time(0), transform3);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    geometry_msgs::Twist vel_msg2, vel_msg3;
    vel_msg2.angular.z = 4.0 * atan2(transform2.getOrigin().y(),
                                transform2.getOrigin().x());
    vel_msg2.linear.x = 0.5 * sqrt(pow(transform2.getOrigin().x(), 2) +
                                pow(transform2.getOrigin().y(), 2));
    vel_msg3.angular.z = 4.0 * atan2(transform3.getOrigin().y(),
                                transform3.getOrigin().x());
    vel_msg3.linear.x = 0.5 * sqrt(pow(transform3.getOrigin().x(), 2) +
                                pow(transform3.getOrigin().y(), 2));
    turtle_vel2.publish(vel_msg2);
    turtle_vel3.publish(vel_msg3);
    rate.sleep();
  }
  return 0;
};
