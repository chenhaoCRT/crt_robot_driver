/*
crt_kinematics_model.cpp
功能：
    实现机器人运动学模型相关计算，差速式底盘

相关物理参数：
    电机减速比1:30, 编码器线数13线， 轮子直径38mm, 履带间距220mm
作者：CRT
日期：2025-12-19
*/

#include "crt_driver/crt_ kinematics_model.h"
#include <math.h>



/*通过编码器数值计算转速、线速度*/
void Wheel_speed::encoder_to_speed(void)
{
    /*计算转速*/
    this->rotationalSpeed_rps = (float)this->encoder_count / (float)(REDUCTION_GEAR_RATIO * PPR
                                    * ENCODER_COUNT_PERIOD * QUADRATURE_MULTIPLIER);
    
    /*计算线速度*/
    this->linearSpeed_mps = this->rotationalSpeed_rps * 2 * M_PI * WHEEL_RADIUS;
}


/*正运动学解算*/
 void Kinematics_model::forward_kinematics(void)
 {
    this->LeftWheel.encoder_to_speed();
    this->RightWheel.encoder_to_speed();
    this->linearVelocity_mps = (this->LeftWheel.linearSpeed_mps + this->RightWheel.linearSpeed_mps) / 2.0;
    this->angularVelocity_radps = (this->RightWheel.linearSpeed_mps - this->LeftWheel.linearSpeed_mps) / TRACK_WIDTH;
    this->velocity_x_mps = this->linearVelocity_mps * cos(this->pose_theta);
    this->velocity_y_mps = this->linearVelocity_mps * sin(this->pose_theta);
 }


 /*
 逆运动学解算
 参数：
    linearVelocity_mps: 目标机器人线速度，单位 m/s
    angularVelocity_radps: 目标机器人角速度，单位 rad/s
    leftWheelSpeed_mps: 输出左轮线速度，单位 m/s
    rightWheelSpeed_mps: 输出右轮线速度，单位 m/s   
 */
void Kinematics_model::inverse_kinematics(double linearVelocity_mps, double angularVelocity_radps, double &leftWheelSpeed_mps, double &rightWheelSpeed_mps)
{
    leftWheelSpeed_mps = linearVelocity_mps - (angularVelocity_radps * TRACK_WIDTH / 2.0);
    rightWheelSpeed_mps = linearVelocity_mps + (angularVelocity_radps * TRACK_WIDTH / 2.0);
}


/*机器人位姿更新*/
void Kinematics_model::update_pose(double dt)
{
    this->delta_x =  this->velocity_x_mps * dt;
    this->delta_y = this->velocity_y_mps * dt;
    this->delta_theta = this->angularVelocity_radps * dt;
    this->pose_x += this->delta_x;
    this->pose_y += this->delta_y;
    this->pose_theta += this->delta_theta;
}





