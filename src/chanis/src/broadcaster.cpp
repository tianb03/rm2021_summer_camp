#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<turtlesim/Pose.h>
std::string turtle_name;
void poseCallback(const turtlesim::PoseConstPtr& msg){
  static tf::TransformBroadcaster br, br21, br31;
  tf::Transform transform, transform21, transform31;
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  transform21.setOrigin( tf::Vector3(-1.0, 1.0, 0.0) );
  transform31.setOrigin( tf::Vector3(-1.0, -1.0, 0.0) );
  tf::Quaternion q, q21, q31;
  q.setRPY(0, 0, msg->theta);
  q21.setRPY(0, 0, 0);
  q31.setRPY(0, 0, 0);
  transform.setRotation(q);
  transform21.setRotation(q21);
  transform31.setRotation(q31);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
  //if(turtle_name == "turtle1"){
  	br21.sendTransform(tf::StampedTransform(transform21, ros::Time::now(), "turtle1", "turtle2_target"));
  	br31.sendTransform(tf::StampedTransform(transform31, ros::Time::now(), "turtle1", "turtle3_target"));
	//}
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
  turtle_name = argv[1];
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);
  ros::spin();
  return 0;
};
