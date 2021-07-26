/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Eigen>
#include <chrono>
#include <tf/transform_broadcaster.h>
#include "utility.h"


namespace robomaster {


    class PIDPlanner {
    public:
        PIDPlanner(ros::NodeHandle &given_nh) : nh(given_nh), plan_(false) {

//    nh.param<double>("max_x_speed", max_x_speed_, 1.0);
//    nh.param<double>("max_y_speed", max_y_speed_, 1.0);
//    nh.param<double>("max_yaw_speed", max_yaw_speed_, 2.0);
//    nh.param<double>("p_x_coeff", p_x_coeff_, 1);
//    nh.param<double>("p_y_coeff", p_y_coeff_, 1);
//    nh.param<double>("p_yaw_coeff", p_yaw_coeff_, 1);
            nh.param<int>("plan_frequency", plan_freq_, 30);
            nh.param<double>("goal_dist_tolerance", goal_dist_tolerance_, 0.1);
            nh.param<double>("goal_angle_tolerance", goal_angle_tolerance_, 0.05);

            tf_listener_ = std::make_shared<tf::TransformListener>();
            cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/calib_vel", 1);

            goal_sub_ = nh.subscribe("/item_pose", 1, &PIDPlanner::GoalCallback, this);

            plan_timer_ = nh.createTimer(ros::Duration(1.0 / plan_freq_), &PIDPlanner::Plan, this);
        }

        ~PIDPlanner() = default;

        void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
            goal_ = *msg;
            plan_ = true;

        }

    private:

        void Plan(const ros::TimerEvent &event) {

            if (plan_) {
                tf::Transform tag2goal_transform, baselink2goal_transform, baselink2tag_transform;

                tag2goal_transform.setOrigin(tf::Vector3(0, 0, 0.26));
                tf::Quaternion q;
                q.setRPY(0, M_PI_2, M_PI_2);
                tag2goal_transform.setRotation(q);

                tf::Stamped<tf::Pose> object_in_baselink;
                double timeout = (ros::Time::now() - goal_.header.stamp).toSec();
                //std::cout<<"stamped time use: "<< timeout<<std::endl;
                if(timeout > 0.090){
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0;
                    cmd_vel.linear.y = 0;
                    cmd_vel.angular.z = 0;
                    cmd_vel_pub_.publish(cmd_vel);
                    ROS_INFO("Time Out! %f", timeout);
                    return;
                }
                //goal_.header.stamp = ros::Time::now();
                TransformPose(tf_listener_, "base_link", goal_, object_in_baselink);
                baselink2tag_transform.setOrigin(object_in_baselink.getOrigin());
                baselink2tag_transform.setRotation(object_in_baselink.getRotation());

                baselink2goal_transform = baselink2tag_transform * tag2goal_transform;

                auto goal2baselink_transform = baselink2goal_transform.inverse();
                tf::Matrix3x3 m(goal2baselink_transform.getRotation());

                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                if (hypot(goal2baselink_transform.getOrigin().getX(), goal2baselink_transform.getOrigin().getY()) <=
                    goal_dist_tolerance_
                    && std::abs(yaw) < goal_angle_tolerance_) {
                    plan_ = false;
                    geometry_msgs::Twist cmd_vel;
                    cmd_vel.linear.x = 0;
                    cmd_vel.linear.y = 0;
                    cmd_vel.angular.z = 0;
                    cmd_vel_pub_.publish(cmd_vel);
                    ROS_INFO("Planning Success!");
                    return;
                }

                auto dx = goal2baselink_transform.getOrigin().getX();
                auto dy = goal2baselink_transform.getOrigin().getY();
                auto dw = yaw;
                Eigen::Matrix2d trans_matrix;
                Eigen::Vector2d dxy_vector(dx, dy);
                Eigen::Vector2d vxy_vector;

                trans_matrix << std::cos(yaw), -std::sin(yaw),
                        std::sin(yaw), std::cos(yaw);

                vxy_vector = trans_matrix.inverse() * dxy_vector;
                auto vw = dw;
                auto k = std::hypot(vxy_vector(0), vxy_vector(1));
                geometry_msgs::Twist cmd_vel;
                cmd_vel.angular.z = -vw;
                cmd_vel.linear.x = -vxy_vector(0);
                cmd_vel.linear.y = -vxy_vector(1);
                cmd_vel_pub_.publish(cmd_vel);
                std::cout << "x: " << goal2baselink_transform.getOrigin().getX() << " y: "
                          << goal2baselink_transform.getOrigin().getY() << " yaw: " << yaw * 57.3 <<" vx: "
                          <<cmd_vel.linear.x<<" vy: "<<cmd_vel.linear.y<<" vyaw: "
                          <<cmd_vel.angular.z<< std::endl;

            }
        }

    private:

        ros::NodeHandle nh;
        std::shared_ptr<tf::TransformListener> tf_listener_;

        geometry_msgs::PoseStamped goal_;
        ros::Timer plan_timer_;

        ros::Subscriber goal_sub_;
        ros::Publisher cmd_vel_pub_;

        bool plan_;

        double max_x_speed_, max_y_speed_, max_yaw_speed_;
        double p_x_coeff_, p_y_coeff_, p_yaw_coeff_;
        double goal_dist_tolerance_, goal_angle_tolerance_;

        int plan_freq_;

    };
}
using namespace robomaster;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "pid_planner");
  ros::NodeHandle nh("~");
  PIDPlanner pid_planner(nh);
  ros::spin();
  return 0;
}

