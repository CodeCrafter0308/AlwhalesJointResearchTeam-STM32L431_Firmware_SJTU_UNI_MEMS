/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "adc.h"
#include "usart.h"
#include "stdio.h"
#include "stdlib.h"
#include <string.h>
#include "wsn31.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void GetChipID ( uint32_t *id )
{
   //小端模式
    id[0] = * ( int32_t * ) ( 0x1fff7590 );
    id[1] = * ( int32_t * ) ( 0x1fff7594 );
    id[2] = * ( int32_t * ) ( 0x1fff7598 );
}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
#define NUM_DIP_PINS   4
#define NUM_SENSORS 8  // 传感器的数量
float x_est[NUM_SENSORS] = {0};  // 估计状�?�初始化�?0
float p_est[NUM_SENSORS] = {1};  // 误差协方差初始化�?1
float q = 0.01;                  // 过程噪声
float r = 7;                    // 观测噪声
float k[NUM_SENSORS];            // 卡尔曼增�?
//int numBytes;    // 将保存sprintf返回的字符数
char JSON_Buffer[256];
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId DAQ_TaskHandle;
osThreadId LED_TaskHandle;
osThreadId PWM_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void GetChipID ( uint32_t *id );
void apply_moving_average_filter(float* input, float* output, int length);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Apptask_DAQ(void const * argument);
void Apptask_LED(void const * argument);
void Apptask_PWM(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}

/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of DAQ_Task */
  osThreadDef(DAQ_Task, Apptask_DAQ, osPriorityNormal, 0, 1024);
  DAQ_TaskHandle = osThreadCreate(osThread(DAQ_Task), NULL);

  /* definition and creation of LED_Task */
  osThreadDef(LED_Task, Apptask_LED, osPriorityNormal, 0, 128);
  LED_TaskHandle = osThreadCreate(osThread(LED_Task), NULL);

  /* definition and creation of PWM_Task */
  osThreadDef(PWM_Task, Apptask_PWM, osPriorityNormal, 0, 128);
  PWM_TaskHandle = osThreadCreate(osThread(PWM_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
  /**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Apptask_DAQ */
/**
* @brief Function implementing the DAQ_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Apptask_DAQ */
void Apptask_DAQ(void const * argument)
{
	/* USER CODE BEGIN Apptask_DAQ */

	    // ==========================================
	    // 1. 变量定义与初始化 (放在循环外，只执行一次)
	    // ==========================================

	    // 定义足够大的缓存区，防止 JSON 字符串溢出
	    char JSON_Buffer[512];
	    char id[32];
	    const char* NT = "";
	    int row = 0;
	    int col = 0;

	    uint32_t uid = HAL_GetUIDw0();
	    char uid_1[8];
	    sprintf(uid_1, "%08X", uid);
	    char name[24] = "TTM:REN-UNIMEMS-";
	    strcat(name, uid_1);
	  //  MX_USART3_UART_Init_115200();
	    HAL_Delay(2000);
	    HAL_UART_Transmit(&huart1, name, 24, 100);
	    HAL_Delay(1000);
	    HAL_UART_Transmit(&huart1,"TTM:BPS-115200", 14, 100);
	    HAL_Delay(1000);
	    MX_USART1_UART_Init_115200();
	    HAL_UART_Transmit(&huart1,"TTM:TPL-(+4)", 14, 100);
	    HAL_Delay(1000);

	    // 原始 ADC 数据缓存 (8个通道)
	    uint32_t ADC_Value[8] = {0};

	    // 电压值变量 (建议直接用数组 float ad[8] 会更简洁，这里保留你的风格)
	    double ad1, ad2, ad3, ad4, ad5, ad6, ad7, ad8;
	    double res1, res2, res3, res4, res5, res6, res7, res8;
	    double res[8];

	    // 等待系统稳定
	    vTaskDelay(500);

	    /* Infinite loop */
	    for(;;)
	    {
	        // ==========================================
	        // 2. ADC 采集 (轮询模式)
	        // ==========================================
	        for(int k = 0; k < 8; k++)
	        {
	            HAL_ADC_Start(&hadc1);
	            // 等待转换完成，超时时间 20ms
	            if (HAL_ADC_PollForConversion(&hadc1, 20) == HAL_OK)
	            {
	                 // 读取转换结果
	                 ADC_Value[k] = HAL_ADC_GetValue(&hadc1);
	            }
	            // 每次采完一个通道最好清除一下状态，防止数据错位
	            // (视具体 CubeMX 配置而定，通常 Poll 模式不需要手动 Stop，但为了稳妥可以不动)
	        }
	        HAL_ADC_Stop(&hadc1); // 采完一轮后停止

	        // ==========================================
	        // 3. 电压计算 (修复了数组下标越界)
	        // ==========================================
	        // 注意：数组下标是 0 到 7
	        ad1 = (double)(ADC_Value[0] & 0xFFF) * 3300.0f / 4096.0f;
	        ad2 = (double)(ADC_Value[1] & 0xFFF) * 3300.0f / 4096.0f;
	        ad3 = (double)(ADC_Value[2] & 0xFFF) * 3300.0f / 4096.0f;
	        ad4 = (double)(ADC_Value[3] & 0xFFF) * 3300.0f / 4096.0f;
	        ad5 = (double)(ADC_Value[4] & 0xFFF) * 3300.0f / 4096.0f;
	        ad6 = (double)(ADC_Value[5] & 0xFFF) * 3300.0f / 4096.0f;
	        ad7 = (double)(ADC_Value[6] & 0xFFF) * 3300.0f / 4096.0f;
	        ad8 = (double)(ADC_Value[7] & 0xFFF) * 3300.0f / 4096.0f;

//	        res[0] = 33000000.0f / ad1 - 10000.0f;
//	        res[1] = 33000000.0f / ad2 - 10000.0f;
//	        res[2] = 33000000.0f / ad3 - 10000.0f;
//	        res[3] = 33000000.0f / ad4 - 10000.0f;
//	        res[4] = 33000000.0f / ad5 - 10000.0f;
//	        res[5] = 33000000.0f / ad6 - 10000.0f;
//	        res[6] = 33000000.0f / ad7 - 10000.0f;
//	        res[7] = 33000000.0f / ad8 - 10000.0f;

	        res[0] = 33000.0f / ad1 - 10.0f;
	        res[1] = 33000.0f / ad2 - 10.0f;
	        res[2] = 33000.0f / ad3 - 10.0f;
	        res[3] = 33000.0f / ad4 - 10.0f;
	        res[4] = 33000.0f / ad5 - 10.0f;
	        res[5] = 33000.0f / ad6 - 10.0f;
	        res[6] = 33000.0f / ad7 - 10.0f;
	        res[7] = 33000.0f / ad8 - 10.0f;

	        // ==========================================
	        // 4. JSON 组包 (修复格式与换行)
	        // ==========================================
	        memset(JSON_Buffer, 0, sizeof(JSON_Buffer));

	        snprintf(JSON_Buffer, sizeof(JSON_Buffer),
	            "{\"ID\":\"%s\",\"NT\":\"%s\",\"ROW\":%d,\"COL\":%d,"
	            "\"VALS\":[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]}\r\n",
	            id,
	            NT,
	            row,
	            col,
	            res[0], res[1], res[2], res[3],
	            res[4], res[5], res[6], res[7]
	        );


	        // ==========================================
	        // 5. 串口发送
	        // ==========================================
	        HAL_UART_Transmit(&huart1, (uint8_t *)JSON_Buffer, strlen(JSON_Buffer), 0xFFFF);

	        // 延时 250ms 进入下一次循环
	        vTaskDelay(1000);
	    }
	  /* USER CODE END Apptask_DAQ */
}

/* USER CODE BEGIN Header_Apptask_LED */
/**
* @brief Function implementing the LED_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Apptask_LED */
void Apptask_LED(void const * argument)
{
  /* USER CODE BEGIN Apptask_LED */
  /* Infinite loop */
	while (1)
	{
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	  vTaskDelay(250);
	}
  /* USER CODE END Apptask_LED */
}

/* USER CODE BEGIN Header_Apptask_PWM */
/**
* @brief Function implementing the PWM_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Apptask_PWM */
void Apptask_PWM(void const * argument)
{
  /* USER CODE BEGIN Apptask_PWM */
  /* Infinite loop */
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Apptask_PWM */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
