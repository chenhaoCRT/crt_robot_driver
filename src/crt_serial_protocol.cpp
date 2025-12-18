/*
crt_serial_protocol.cpp
crt_serial_protocol类实现文件
功能：实现与小车底盘的CRT串口通信协议
作者：CRT
日期：2025-12-18
crt通信协议格式：
    0xAA帧头+1Byte源地址+1Byte目的地址+1Byte指令ID+1Byte数据长度+nByte数据内容+1Byte和校验+1Byte附加校验

    小车底盘地址:   0x01
    上位机地址:     0x02

    指令ID：
        0x01：控制左右电机速度，数据内容为左右电机速度各2Byte，共4Byte，单位rpm，正负表示方向
        0x02：设置PID参数，数据内容为Kp、Ki、Kd各2Byte浮点数，共6Byte, 参数都乘以1000后取整存储
    
        0xA1: 发送左右电机编码器计数值，数据内容为左右编码器计数值各2Byte，共4Byte
...

*/


#include "ros/ros.h"
#include "crt_driver/crt_serial_protocol.h"
#include  "serial/serial.h"
#include  "stdio.h"
#include <unistd.h>  



/*
crt_serial_protocol类构造函数
    参数：
        Port:串口端口号，如"/dev/ttyUSB0"
        Baudrate:串口波特率，如115200
        timeout:串口读写超时时间，单位ms
    功能：
        初始化串口参数并打开串口


 */
crt_serial_protocol::crt_serial_protocol(std::string Port, uint32_t Baudrate,  uint32_t timeout)
{
    L_encoder = 0;
    R_encoder = 0;


    /*初始化串口*/
    ser_driverBase.setPort(Port);
    ser_driverBase.setBaudrate(Baudrate);
    serial::Timeout to = serial::Timeout::simpleTimeout(timeout);
    ser_driverBase.setTimeout(to);
    ser_driverBase.open();


    /*判断是否打开成功*/
    if(ser_driverBase.isOpen()) {
        ROS_INFO("crt_serial_protocol:serial open success");
        ser_driverBase.flushInput();
        
    }
    else {
        ROS_INFO("crt_serial_protocol:serial open failed");
        exit(-1);
        
    }
    
}

/*
crt_serial_protocol类析构函数
    功能：关闭串口
*/
crt_serial_protocol::~crt_serial_protocol()
{
    if(ser_driverBase.isOpen()) {
        ser_driverBase.close();
    }
}




/*
crt_serial_protocol_receiveOneByte函数
    参数：
        byte:接收到的一个字节数据
    功能：
        按字节接收数据，并解析成完整帧，解析完成后调用crt_serial_protocol_anlyzeFrame函数处理数据
    返回值：
        bool:解析完成并处理成功返回true，否则返回false
*/
bool crt_serial_protocol::crt_serial_protocol_receiveOneByte(uint8_t byte)
{
    static uint16_t _recv_cnt = 0;
    static uint16_t _data_cnt = 0;
    static uint8_t _state = 0;
    

    /*帧头*/
    if (_state == 0 && byte == 0xAA) {
        _recv_cnt = 0;
        RxFrame.rawBytes[_recv_cnt++] = byte;
        _state++;
        
    }

    /*源地址*/
    else if (_state == 1) {
        RxFrame.rawBytes[_recv_cnt++] = byte;
        _state++;
    }

    /*目的地址*/
    else if (_state == 2) {
        RxFrame.rawBytes[_recv_cnt++] = byte;
        _state++;
    }

    /*指令ID*/
    else if (_state == 3) {
        RxFrame.rawBytes[_recv_cnt++] = byte;
        _state++;
    }

    /*数据长度*/
    else if (_state == 4) {
        RxFrame.rawBytes[_recv_cnt++] = byte;
        _state++;
    }

    /*数据*/
    else if (_state == 5) {
        RxFrame.rawBytes[_recv_cnt++] = byte;
        _data_cnt++;
        if (_data_cnt >= RxFrame.frame.data_length) {
            _state++;
            _data_cnt = 0;
        }
    }

    /*和校验*/
    else if (_state == 6) {
        RxFrame.frame.sc1 = byte;
        _state++;
    }

    /*附加校验*/
    else if (_state == 7) {
        RxFrame.frame.sc2 = byte;

        /*处理数据*/
        bool res = crt_serial_protocol_anlyzeFrame();
        _state = 0;
        _data_cnt = 0;
        return res;
    }

    else {
        _state = 0;
        _data_cnt = 0;
        return false;
    }
    return false;
}


/*
crt_serial_protocol_anlyzeFrame函数
    功能：
        解析完整帧数据，并根据指令ID处理数据
    返回值：
        bool:处理成功返回true，否则返回false
*/
bool crt_serial_protocol::crt_serial_protocol_anlyzeFrame(void)
{
    uint8_t check_sum1 = 0;
    uint8_t check_sum2 = 0;


    /*检查校验值*/
    for (int i = 0; i < (5 + RxFrame.frame.data_length); i++) {
        check_sum1 += RxFrame.rawBytes[i];
        check_sum2 += check_sum1;
    }
    //printf("HEAD: 0x%02X, SRC: 0x%02X, DEST: 0x%02X, CMD_ID: 0x%02X, LEN: %d\n", RxFrame.rawBytes[0], RxFrame.rawBytes[1], RxFrame.rawBytes[2], RxFrame.rawBytes[3], RxFrame.frame.data_length);
    if (check_sum1 != RxFrame.frame.sc1 || check_sum2 != RxFrame.frame.sc2) {
        return false;
    }

    if (RxFrame.frame.dest_addr != ADDR_HOST) {
        return false;
    }
    

    /*判断ID*/
    switch (RxFrame.frame.cmd_id) {
        /*收到编码器数据*/
        case CMD_ID_SEND_ENCODER_VALUES:
            L_encoder = (int16_t)(RxFrame.frame.data[0] << 8 | (RxFrame.frame.data[1]));
            R_encoder = (int16_t)(RxFrame.frame.data[2] << 8 | (RxFrame.frame.data[3]));
            return true;
            break;


        default:
            return false;
            break;
    }
    return false;
}



/*
crt_serial_protocol_readSerial函数
    功能：
        读取串口数据，并按字节调用crt_serial_protocol_receiveOneByte函数解析数据
    返回值：
        bool:解析完成并处理成功返回true，否则返回false
*/
bool crt_serial_protocol::crt_serial_protocol_readSerial(void)
{
    /*判断是否有数据可读*/
    int _cnt = ser_driverBase.available();
    if (_cnt != 0) {
        /*读取一串数据*/
        ser_driverBase.read(read_buf, _cnt);
        for (int i = 0; i < _cnt; i++) {
            
            /*逐一解析*/
            if (crt_serial_protocol_receiveOneByte(read_buf[i]))
                return true;
        }
    }
    return false;
}



/*
crt_serial_protocol_sendMotorSpeed函数
    参数：
        L_speed:左电机速度，正负表示方向
        R_speed:右电机速度，正负表示方向
    功能：
        发送控制左右电机速度指令帧
*/
void crt_serial_protocol::crt_serial_protocol_sendMotorSpeed(int16_t L_speed, int16_t R_speed)
{
    /*包头0xAA*/
    TxFrame.frame.frame_head = 0xAA;
    /*源地址上位机ID*/
    TxFrame.frame.src_addr = ADDR_HOST;
    /*目的地址MCU*/
    TxFrame.frame.dest_addr = ADDR_DRIVER;
    /*指令ID发送编码器速度*/
    TxFrame.frame.cmd_id = CMD_ID_SET_MOTOR_SPEED;
    /*数据长度4Byte*/
    TxFrame.frame.data_length = 4;
    /*数据内容左右电机速度各2Byte， 先高字节后低字节*/
    TxFrame.frame.data[0] = (uint8_t)((L_speed >> 8) & 0xFF);
    TxFrame.frame.data[1] = (uint8_t)(L_speed & 0xFF);
    TxFrame.frame.data[2] = (uint8_t)((R_speed >> 8) & 0xFF);
    TxFrame.frame.data[3] = (uint8_t)(R_speed & 0xFF);
    /*计算校验*/
    uint8_t check_sum1 = 0;
    uint8_t check_sum2 = 0;
    for (int i = 0; i < (5 + TxFrame.frame.data_length); i++) {
        check_sum1 += TxFrame.rawBytes[i];
        check_sum2 += check_sum1;
    }
    TxFrame.rawBytes[5 + TxFrame.frame.data_length] = check_sum1;
	TxFrame.rawBytes[6 + TxFrame.frame.data_length] = check_sum2;
    /*发送数据*/
    ser_driverBase.write(TxFrame.rawBytes, 7 + TxFrame.frame.data_length);
}

