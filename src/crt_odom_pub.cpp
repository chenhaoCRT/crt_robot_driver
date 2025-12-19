/*
crt_odom_pub.cpp
crt_driverBase节点实现文件
功能：
    发布里程计话题

作者：CRT
日期：2025-12-18 
*/

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "crt_driver/crt_encoder.h"
#include "crt_driver/crt_kinematics_model.h"





void do_recv_encoder_fromMCU(const crt_driver::crt_encoder::ConstPtr& msg)
{
    = msg->leftEncoder;
    = msg->leftEncoder;
}

int main(int argc, char **argv)
{
    /*初始化ros节点*/
    ros::init(argc, argv, "crt_pub_odom");
    ros::NodeHandle nh;
    /*odom发布者*/
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
    /*订阅编码器消息*/
    ros::Subscriber encoder_sub = nh.subscribe<crt_driver::crt_encoder>("/encoder_fromMCU", 10, do_recv_encoder_fromMCU);


    ros::Rate loop_rate(50);
    while (ros::OK()) {


        loop_rate.sleep();
    }


}   