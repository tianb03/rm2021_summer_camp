#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <cstdio>
#include <iostream>

using std::cin;

const int MAXN = 500 + 5, cnt = 462;
const double A = 0.1, B = 0.1, R = 0.05, dt = 0.01;
double enc1[MAXN], enc2[MAXN], enc3[MAXN], enc4[MAXN];
double x, y, th, vx, vy, vth;

struct My_type{
    double vx, vy, vth;
};

My_type solve(double w1, double w2, double w3, double w4){
    My_type res;
    res.vx = (w1 + w2 + w3 + w4) * R / 4.0;
    res.vy = (- w1 + w2 + w3 - w4) * R / 4.0;
    res.vth = (w1 - w2 - w3 + w4) * R / (A + B) / 4.0;
    return res;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "enc_solve");
    ros::NodeHandle node;
    ros::Publisher enc_pub = node.advertise<nav_msgs::Odometry>("/enc", 10);

    freopen("/home/chanis/rm2021_summer_camp/src/chanis/src/data.txt", "r", stdin);
    for(int i = 1; i <= cnt; i ++)
        cin >> enc1[i] >> enc2[i] >> enc3[i] >> enc4[i];

    ros::Rate r(100.0);
    for(int i = 1; i <= cnt; i ++){
        ros::spinOnce();

        My_type ans = solve(enc1[i], enc2[i], enc3[i], enc4[i]);
        vx = ans.vx, vy = ans.vy, vth = ans.vth;
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;
        x += delta_x, y+= delta_y, th += delta_th;

        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(th);
        nav_msgs::Odometry odom;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.header.stamp = ros::Time::now();
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = quat;
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;
        enc_pub.publish(odom);

        r.sleep();
    }
    ros::spin();

    return 0;
}
