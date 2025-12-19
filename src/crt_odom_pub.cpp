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



Kinematics_model RobotKinematics_model;

/*获取编码器数值并计算轮速度*/
void do_recv_encoder_fromMCU(const crt_driver::crt_encoder::ConstPtr& msg)
{
    /*获取编码器值*/
    RobotKinematics_model.LeftWheel.encoder_count = msg->leftEncoder;
    RobotKinematics_model.RightWheel.encoder_count = msg->rightEncoder;

    
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
    /*发布odom*/
    tf::TransformBroadcaster odom_broadcaster;

    /*记录时间*/
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = current_time;

    ros::Rate loop_rate(50);
    while (ros::ok()) {

        ros::spinOnce();  

        /*计算时间差*/
        current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        /*正运动学解算计算机器人轮速、线速度和角速度*/
        RobotKinematics_model.forward_kinematics();
        /*更新机器人位姿*/
        RobotKinematics_model.update_pose(dt);
        
        /*tf四元数*/
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(RobotKinematics_model.pose_theta);
        
        /*创建TransformStamped消息，通过 tf发布从“odom”到“base_link”的转换*/
        geometry_msgs::TransformStamped odom_trans;

        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = RobotKinematics_model.pose_x;
        odom_trans.transform.translation.y = RobotKinematics_model.pose_y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        /*创建并发布里程计消息*/
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        /*位置*/
        odom.pose.pose.position.x = RobotKinematics_model.pose_x;
        odom.pose.pose.position.y = RobotKinematics_model.pose_y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        /*速度*/
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = RobotKinematics_model.velocity_x_mps;
        odom.twist.twist.linear.y = RobotKinematics_model.velocity_y_mps;
        odom.twist.twist.angular.z = RobotKinematics_model.angularVelocity_radps;
        odom_pub.publish(odom);
        /*更新时间*/
        last_time = current_time;
    
        loop_rate.sleep();
    }


}   