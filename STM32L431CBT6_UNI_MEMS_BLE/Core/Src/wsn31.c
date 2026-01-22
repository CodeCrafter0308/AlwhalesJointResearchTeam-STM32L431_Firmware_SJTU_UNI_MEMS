/*
 * wsn31.c
 *
 *  Created on: 2021年3月11日
 *      Author: Tao
 */

#include "wsn31.h"
//#include "usart_ext.h"
//#include "check.h"

#include "FreeRTOS.h"
#include "task.h"
#include "stm32l4xx_hal.h"
#include "usart.h"

/**
 * @brief Generate the configuration command of the WSN-31 module.
 * @param cmd uint8_t type array, with a fix length of 22
 * @param config Parameters of the WSN-31, encapsulated into a structure
 */

uint8_t GetDipSwitchValue(GPIO_PinState dipStates[], int size)
{
    uint8_t netAddr = 0;
    for (int i = 0; i < size; i++) {
        // 如果读出的是低电平，则设置相应的位为1
        if (dipStates[i] == GPIO_PIN_RESET) {
            netAddr |= (1 << i);
        }
        // 如果是高电平，由于netAddr已初始化为0，所以不需要操作
    }
    return netAddr;
}

//uint32_t *DIPSW_GpioArray[8] =
//{
//    DIP_SWITCH_1_1_ADDR,
//    DIP_SWITCH_1_2_ADDR,
//    DIP_SWITCH_1_3_ADDR,
//    DIP_SWITCH_1_4_ADDR,
//    DIP_SWITCH_2_1_ADDR,
//    DIP_SWITCH_2_2_ADDR,
//    DIP_SWITCH_2_3_ADDR,
//    DIP_SWITCH_2_4_ADDR,
//};

void WSN31_GeneCmd(uint8_t *cmd, WSN31_ConfigStruct *config)
{
    uint8_t i = 0;
    cmd[i++] = 0x55; // head of cmd
    cmd[i++] = 0xAA;
    cmd[i++] = 0x16; // data length = 22

    /// cmd type: 0x00: confirm type, 0x01: not confirm,
    ///  0x02: read parameter, 0x03: read parameter response
    ///  0x04: write parameter
    cmd[i++] = 0x04;

    cmd[i++] = config->netNum;                     // network number
    cmd[i++] = ((uint8_t *)(&config->devAddr))[1]; // device address - high byte
    cmd[i++] = ((uint8_t *)(&config->devAddr))[0]; // device address - low byte
    cmd[i++] = config->wkMod;                      // work mode

    cmd[i++] = 0x02; // default slave module
    cmd[i++] = 0x01; // do not output source address

    cmd[i++] = config->chan;
    cmd[i++] = config->power;
    cmd[i++] = config->baudrate;

    cmd[i++] = 1; // default data bits of serial port is 8
    cmd[i++] = 1; // default no check of serial port
    cmd[i++] = 1; // default stop bit is 1

    cmd[i++] = config->porttime;

    // reserved bytes
    cmd[i++] = 0x00;
    cmd[i++] = 0x00;
    cmd[i++] = 0x00;
    cmd[i++] = 0x00;

    // last byte of the command is checksum byte.
    cmd[i++] = CheckSum_8Bits_Gene(cmd, 22 - 1);
}

/**
 * @brief Initialization of the WSN31
 */
void WSN31_Init()
{
    WSN31_ConfigStruct wsn31_config =
        {
            .chan = 66,               // information channel is 66 (not hex)
            .devAddr = WSN31_DEVADDR, // device address is 0x0001
            .netNum = 0x66,           // Net number is 0x66
            .baudrate = 8,            // 8: 115200
            .porttime = 5,            // 5 ms
            .power = 5,               // Power 5
            .wkMod = 2,               // point-to-point mode
        };

    // Length of command of WSN31 is 22 (include sum check)
    uint8_t configCmd[22] = {0};

    WSN31_GeneCmd(configCmd, &wsn31_config);

    WSN31_Config(configCmd, vTaskDelay);
}

/**
 * @brief Generate the configuration command of the WSN-31 module.
 * @param cmd uint8_t type array, with a fix length of 22
 * @param config Parameters of the WSN-31, encapsulated into a structure
 */
void WSN32_GeneCmd(uint8_t *cmd, WSN32_ConfigStruct *config)
{
    uint8_t i = 0;
    cmd[i++] = 0x55; // head of cmd
    cmd[i++] = 0xAA;

    /// cmd type: 0x00: confirm type, 0x01: not confirm,
    ///  0x02: read parameter, 0x03: read parameter response
    ///  0x04: write parameter
    cmd[i++] = 0x04;

    cmd[i++] = config->netNum;                     // network number
    cmd[i++] = ((uint8_t *)(&config->devAddr))[1]; // device address - high byte
    cmd[i++] = ((uint8_t *)(&config->devAddr))[0]; // device address - low byte
    cmd[i++] = config->wkMod;                      // work mode

    /// parameters of work mode 0
    cmd[i++] = config->uart_baudrate;    // uart: baudrate - 0:1200 1:2400 2:4800 3:9600 4:19200 5:38400 6:57600 7:115200
    cmd[i++] = config->uart_format;      // uart: format - 0: 8N1 1: 8E1 2: 8O1 3: 7E1 4: 7O1
    cmd[i++] = config->wireless_channel; // Wireless channel 0-199 信道（默认 0 信道）（建议使用 0-99 信道），注意是十进制数
    cmd[i++] = config->wireless_power;   // Wireless power
    /// parameters of work mode 1
    cmd[i++] = config->wireless_rate;      // Wireless rate  0:0.5K 1:1.2K 2:2.5K 3:5.0K 4:10K 5:25K 6:150K 7:240K
    cmd[i++] = config->isEnableFEC;        // Enable FEC ?  0: 关闭 FEC 1: 开启 FEC
    cmd[i++] = config->isPointToPoint;     //"Point to point" or "broadcast" mode ?  0: 广播发送 1: 定点发送
    cmd[i++] = config->isOutputSourceAddr; // Is output source address ?
    cmd[i++] = config->isOutputRSSI;       // Is output RSSI ?    0: 不输出RSSI值 1: 输出RSSI值

    // reserved bytes
    for (; i < 21; i++)
    {
        cmd[i] = 0x00;
    }
}

void WSN32_Config(uint8_t *cmd, void (*msDelayCB)(uint16_t ms))
{
    //进入设置模式
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

    //设置串口参数
//    User_SetPortParam(WSN31_PORT_NUM, 9600, 8, 1, USART_Parity_No);
//    vTaskDelay(150);

    //发送参数设置命令
    HAL_UART_Transmit(&huart1, cmd, 21, 100);  // 100是超时时间，单位为毫秒
    vTaskDelay(100);

    //进入工作模式
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}

/**
 * @brief Initialization of the WSN32
 */
void WSN32_Init(uint8_t net_num, uint8_t wireless_channel, uint16_t dev_addr)
{
    WSN32_ConfigStruct wsn32_config = {
        .netNum = net_num, // 网络标志符
        .devAddr = dev_addr,
        .wkMod = 1,
        .uart_baudrate = 7, // 115200
        .wireless_rate = 7, // 240K
        .uart_format = 0,
        .wireless_channel = wireless_channel,
        .wireless_power = 7,
        .isEnableFEC = 0,
        .isPointToPoint = 1,
        .isOutputSourceAddr = 0,
        .isOutputRSSI = 0,
    };
    
    // Length of command of WSN32 is 21 (do not have sum check)
    uint8_t configCmd[21] = {0};
    WSN32_GeneCmd(configCmd, &wsn32_config);
    WSN32_Config(configCmd, vTaskDelay);
}
