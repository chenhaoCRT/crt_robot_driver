/*
crt_kinematics_model.cpp
功能：
    实现机器人运动学模型相关计算，差速式底盘

相关物理参数：
    电机减速比1:30, 编码器线数13线， 轮子直径38mm, 履带间距220mm
作者：CRT
日期：2025-12-19
*/

#include "include/crt_driver/crt_ kinematics_model.h"

/*通过编码器数值计算转速、线速度*/
void Wheel_speed::encoder_to_speed(void)
{
    /*计算转速*/
    this->rotationalSpeed_rps = (float)this->encoder_count / (float)(REDUCTION_GEAR_RATIO * PPR
                                    * ENCODER_COUNT_PERIOD * QUADRATURE_MULTIPLIER);
    
    /*计算线速度*/
    this->linearSpeed_mps = this->rotationalSpeed_rps * 2 * M_PI * WHEEL_RADIUS;
}




