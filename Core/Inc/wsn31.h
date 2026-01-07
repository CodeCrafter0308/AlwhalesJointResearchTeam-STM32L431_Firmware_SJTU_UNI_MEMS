/*
 * wsn31.h
 *
 *  Created on: 2021年3月11日
 *      Author: Tao
 */

#ifndef __WSN31_H__
#define __WSN31_H__

#include "stm32l4xx_hal.h"
#include "main.h"

#define WSN31_DEVADDR                   0x0002
#define WSN31_WKMOD                     GPIOB_OUT(12)            //WSN31_WKMOD = 1, work mode; WSN31_WKMOD = 0, set mode;
#define WSN31_PORT_NUM                  1
#define WSN31_PORT                      USART1


//GPIO_PinState pinState_1_1, pinState_1_2, pinState_1_3, pinState_1_4;
//GPIO_PinState pinState_2_1, pinState_2_2, pinState_2_3, pinState_2_4;

typedef struct
{
    uint8_t netNum;         //number of the network
    uint16_t devAddr;        //address of the module, valid only if wlMod = 2
    uint8_t wkMod;          //work model 1~4, 1: broadcast mode, 2: point-to-point mode, 3: IO mode, 4: reserved
    uint8_t chan;              //information channel, 1~100
    uint8_t power;            //transmitted power, 1 (Min.)~8 (Max.)
    uint8_t baudrate;        //baud rate of the serial port, 1:1200 2:2400 3:4800 4:9600 5:19200 6:38400 7:57600 8:115200
    uint8_t porttime;        //interval time to judge 1 package data, 5~100 ms
} WSN31_ConfigStruct;

typedef struct
{
    /*---------------------------Work mode 0------------------------------*/
    uint8_t netNum;             //number of the network
    uint16_t devAddr;           //address of the module

    /**
     * work model 0~5 (default 0)
     * 0: 连续传输模式
     * 1: 单包传输模式
     * 2: 唤醒模式
     * 3: 省电模式
     * 4: 深度休眠模式
     * 5: 电平跟随模式 (少见，本协议未实现驱动)
     */
    uint8_t wkMod;              //work model 0~5 (default 0)
    uint8_t uart_baudrate;     //uart: baudrate - 0:1200 1:2400 2:4800 3:9600 4:19200 5:38400 6:57600 7:115200
    uint8_t uart_format;        //uart: format - 0: 8N1 1: 8E1 2: 8O1 3: 7E1 4: 7O1
    uint8_t wireless_channel;       //Wireless channel 0~199 信道（默认 0 信道）（建议使用 0-99 信道），注意是十进制数
    uint8_t wireless_power;            //transmitted power, 0 (Min.)~7 (Max.)

    /*---------------------------Work mode 1------------------------------*/
    uint8_t wireless_rate;            //Wireless rate  0:0.5K 1:1.2K 2:2.5K 3:5.0K 4:10K 5:25K 6:150K 7:240K

    /**
     * 支持 FEC 算法，只有 FEC 模式相同的模块才能通讯。
     * 使能 FEC 后串口一次发送数据不能 超过 150 个字节.
     */
    uint8_t isEnableFEC;            //Enable FEC ?  0: 关闭 FEC 1: 开启 FEC

    /**
     * 当使能定点发送时，串口发送数据时需要在数据前 端加两个字节的目的模块地址（e.g. 12 34 01 02 03）,
     * 只有对应模块地址(12 34)的模块才能收到 数据（01 02 03）。
     */
    uint8_t isPointToPoint;         //"Point to point" or "broadcast" mode ?  0: 广播发送 1: 定点发送

    /**
     * 当使能输出源地址时，模块收到数据后会在有效 数据前加上两个字节的源模块地址（e.g. 12 33 01 02 03）,
     * 即（01 02 03）这个数据是模块地址 为(12 33)的模块发送的。
     */
    uint8_t isOutputSourceAddr;     //Is output source address ?

    /**
     * 使能 RSSI 输出时，模块收到数据后会在有效数 据前加上一个字节的 RSSI 值（e.g. F8 01 02 03），
     * 即（F8）为当前模块接收的无线数据的 RSSI 值，该值越大说明信号越好。
     */
    uint8_t isOutputRSSI;                   //Is output RSSI ?    0: 不输出RSSI值 1: 输出RSSI值

    /*---------------------------Work mode 2------------------------------*/
    uint8_t wakeupTime;
    /*---------------------------Work mode 3------------------------------*/
    uint8_t wakeupPackage;
    /*---------------------------Work mode 4------------------------------*/
    uint8_t delaySleep;
} WSN32_ConfigStruct;

void WSN31_GeneCmd(uint8_t *cmd, WSN31_ConfigStruct *config);
void WSN31_Config(uint8_t *cmd, void (* msDelayCB)(uint16_t));
void WSN31_Init();

void WSN32_GeneCmd(uint8_t *cmd, WSN32_ConfigStruct *config);
void WSN32_Config(uint8_t *cmd, void (* msDelayCB)(uint16_t));
void WSN32_Init(uint8_t net_num, uint8_t wireless_channel, uint16_t dev_addr);
uint8_t GetDipSwitchValue(GPIO_PinState dipStates[], int size);
#endif /* SOURCE_ALWHALESLIB_HARDWARE_INC_WSN31_H_ */
