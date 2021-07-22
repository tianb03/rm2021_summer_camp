#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<nav_msgs/Odometry.h>
#include<turtlesim/Pose.h>
double x, y, th, vx, vy, vth;
ros::Time current_time, last_time;
void Callback(const turtlesim::PoseConstPtr& msg){
	vx = msg -> x, vy = msg -> y, vth = msg -> theta;
}
int main(int argc, char** argv){
  ros::init(argc, argv, "odometry");
  ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/turtle1/pose", 10, &Callback);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/turtle1/odom", 50);
  tf::TransformBroadcaster odom_broadcaster;
  current_time = last_time = ros::Time::now();
  ros::Rate r(1.0);
  while(n.ok()){
    ros::spinOnce();
    current_time = ros::Time::now();
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;
    x += delta_x;
    y += delta_y;
    th += delta_th;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    odom_pub.publish(odom);
    last_time = current_time;
    r.sleep();
  }
	ros::spin();
	return 0;
 }
