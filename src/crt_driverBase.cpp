/*
crt_driverBase.cpp
crt_driverBase节点主程序
功能：
    创建ROS节点，初始化crt_serial_protocol类，读取串口数据并发布编码器数据
    订阅编码器速度并发送串口到底盘
作者：CRT
日期：2025-12-18
*/



#include "ros/ros.h"
#include "crt_driver/crt_serial_protocol.h"
#include "serial/serial.h"
#include "crt_driver/crt_encoder.h"



int LSpeed_toMCU = 0;
int RSpeed_toMCU = 0;

void do_recv_encoder(const crt_driver::crt_encoder::ConstPtr& msg)
{
    LSpeed_toMCU = msg->leftEncoder;
    RSpeed_toMCU = msg->rightEncoder;
}


int main(int argc, char **argv)
{
    /*初始化ROS节点*/
    ros::init(argc, argv, "crt_driverBase");
    /*初始化串口*/
    crt_serial_protocol serial_protocol("/dev/serial_stm32", 115200, 1000);
    

    ros::NodeHandle nh;
    
    /*发布底盘发出来的编码器数据*/
    ros::Publisher encoder_pub = nh.advertise<crt_driver::crt_encoder>("/encoder_fromMCU", 10);

    /*订阅控制底盘的编码器数据*/
    ros::Subscriber encoder_sub = nh.subscribe<crt_driver::crt_encoder>("/encoder_toMCU", 10, do_recv_encoder);


    
    /*循环读取串口数据并发布编码器数据*/
    crt_driver::crt_encoder encoder_msg;
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        /*读取串口*/
        if (serial_protocol.crt_serial_protocol_readSerial()) {
            encoder_msg.leftEncoder = serial_protocol.L_encoder;
            encoder_msg.rightEncoder = serial_protocol.R_encoder;
            encoder_pub.publish(encoder_msg);
            ros::spinOnce();

        }

        /*发送串口*/
        serial_protocol.crt_serial_protocol_sendMotorSpeed(LSpeed_toMCU, RSpeed_toMCU);        
        
        loop_rate.sleep();
    }   

    return 0;
}