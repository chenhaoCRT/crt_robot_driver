/*
crt_serial_protocol.h
crt_serial_protocol类头文件
功能：实现与小车底盘的CRT串口通信协议
作者：CRT
日期：2025-12-18
*/


#ifndef __CRT_SERIAL_PROTOCOL_H__
#define __CRT_SERIAL_PROTOCOL_H__

#include <stdint.h>
#include  "serial/serial.h"

/*MCU驱动地址*/
#define ADDR_DRIVER         0x01
/*上位机地址*/
#define ADDR_HOST           0x02

#define CMD_ID_SET_MOTOR_SPEED      0x01
#define CMD_ID_SET_PID_PARAMS       0x02

#define CMD_ID_SEND_ENCODER_VALUES  0xA1



typedef struct
{
    uint8_t frame_head;
    uint8_t src_addr;
    uint8_t dest_addr;
    uint8_t cmd_id;
    uint8_t data_length;
    uint8_t data[256];
    uint8_t sc1;
    uint8_t sc2;

} __attribute__ ((__packed__)) serial_protocol_frame_struct;




typedef union
{

    serial_protocol_frame_struct frame;
    uint8_t rawBytes[sizeof(serial_protocol_frame_struct)];

} __attribute__ ((__packed__)) serial_protocol_frame;



class crt_serial_protocol
{
    public:
        uint8_t read_buf[256];
        int16_t L_encoder, R_encoder;

        serial_protocol_frame RxFrame, TxFrame;

        serial::Serial ser_driverBase;
        crt_serial_protocol(std::string Port="/dev/serial_stm32", uint32_t Baudrate=115200,  uint32_t timeout=1000);
        ~crt_serial_protocol();

        bool crt_serial_protocol_receiveOneByte(uint8_t byte);
        bool crt_serial_protocol_anlyzeFrame(void);
        bool crt_serial_protocol_readSerial(void);

        void crt_serial_protocol_sendMotorSpeed(int16_t L_speed, int16_t R_speed);


};



#endif // __CRT_SERIAL_PROTOCOL_H__
