/*
 * board.h
 *
 *  Created on: 2021年4月30日
 *      Author: Tao
 */

#ifndef SOURCE_USER_INC_BOARD_H_
#define SOURCE_USER_INC_BOARD_H_

#include "stm32l4xx_hal.h"
//#include "filter.h"
//#include "delay.h"
//#include "debug.h"
//
#include "gpio_ext_user.h"
//#include "usart_ext.h"
//#include "timer_ext.h"
//#include "timer_ext_user.h"
//#include "adc_ext_user.h"
//#include "irqhandler.h"

#define SYS_LED_ADDR                    GPIOA_OUT_ADDR(15)
#define SYS_LED                         GPIOB_OUT(13)

#define SYS_BUZZER_ADDR                 GPIOB_OUT_ADDR(0)
#define SYS_BUZZER                      GPIOB_OUT(0)

#define DIP_SWITCH_1_1_ADDR                GPIOB_IN_ADDR(3)
#define DIP_SWITCH_1_2_ADDR                GPIOB_IN_ADDR(4)
#define DIP_SWITCH_1_3_ADDR                GPIOB_IN_ADDR(5)
#define DIP_SWITCH_1_4_ADDR                GPIOB_IN_ADDR(6)

#define DIP_SWITCH_2_1_ADDR                GPIOA_IN_ADDR(7)
#define DIP_SWITCH_2_2_ADDR                GPIOA_IN_ADDR(6)
#define DIP_SWITCH_2_3_ADDR                GPIOA_IN_ADDR(5)
#define DIP_SWITCH_2_4_ADDR                GPIOA_IN_ADDR(4)

void Board_Config();
void Board_ConfigRCC();
void Board_ConfigNVIC();
void Board_ConfigPort();
void Board_ConfigTimer();
void Board_ConfigUSART();
void Board_ConfigADC();
void Board_ConfigIWDG(uint8_t IWDG_Prescaler ,uint16_t Reload);

#endif /* SOURCE_USER_INC_BOARD_H_ */
