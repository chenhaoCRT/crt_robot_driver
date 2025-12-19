/*
crt_kinematics_model.h
功能：
    实现机器人运动学模型相关计算，差速式底盘

相关物理参数：
    电机减速比1:30, 编码器线数13线， 轮子直径38mm, 履带间距220mm
作者：CRT
日期：2025-12-19 
*/

#ifndef __CRT_KINEMATICS_MODEL_H__
#define __CRT_KINEMATICS_MODEL_H__


/*电机减速比*/
#define REDUCTION_GEAR_RATIO    30

/*编码器线数*/
#define PPR                     13
/*编码器计数周期，单位 s*/
#define ENCODER_COUNT_PERIOD   0.02
/*编码器倍频数*/
#define QUADRATURE_MULTIPLIER   4 

/*轮子半径，单位 m*/
#define WHEEL_RADIUS            0.019
/*轴距， 单位 m*/
#define TRACK_WIDTH             0.22

#define M_PI    3.141



/*每个轮子参数类*/
class Wheel_speed
{
    public:
        /*二十毫秒编码器数值*/
        int encoder_count;

        /*轮子转速，单位：转/s*/
        double rotationalSpeed_rps;

        /*轮子线速度，单位：m/s*/
        double linearSpeed_mps;

        /*构造函数*/
        Wheel_speed():encoder_count(0), rotationalSpeed_rps(0.0), linearSpeed_mps(0.0) {};

        /*通过编码器数值计算转速、线速度*/
        void encoder_to_speed(void);

};


class Kinematics_model
{
    public:
        /*左右轮*/
        Wheel_speed LeftWheel, RightWheel;

        /*线速度，单位 m/s*/
        double linearVelocity_mps; 

        /*x_y方向速度*/
        double velocity_x_mps;
        double velocity_y_mps;

        /*角速度，单位 rad/s*/  
        double angularVelocity_radps; 
        
        /*机器人位姿*/
        double pose_x;
        double pose_y;
        double pose_theta;

        double delta_x;
        double delta_y;
        double delta_theta;


        /*构造函数*/
        Kinematics_model():pose_x(0.0), pose_y(0.0), pose_theta(0.0) {};


        /*正运动学解算*/
        void forward_kinematics(void);


        /*位姿更新*/
        void update_pose(double dt);

        /*逆运动学解算*/
        void inverse_kinematics(double linearVelocity_mps, double angularVelocity_radps, double &leftWheelSpeed_mps, double &rightWheelSpeed_mps);



};

#endif /*__CRT_KINEMATICS_MODEL_H__*/
