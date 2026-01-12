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
#include "cJSON.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
QueueHandle_t JSON_QueueHandle; // 定义队列句柄
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
  JSON_QueueHandle = xQueueCreate(5, sizeof(char*));
  if (JSON_QueueHandle == NULL) {
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of DAQ_Task */
  osThreadDef(DAQ_Task, Apptask_DAQ, osPriorityNormal, 0, 2056);
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
    char id[32];
    const char* NT = "";
    uint32_t uid = HAL_GetUIDw0();
    snprintf(id, sizeof(id), "0001", (unsigned long)uid);

    uint32_t ADC_Value[8] = {0};
    double ad1 = 0.0, ad2 = 0.0, ad3 = 0.0, ad4 = 0.0, ad5 = 0.0, ad6 = 0.0, ad7 = 0.0, ad8 = 0.0;

    vTaskDelay(10);
    for(;;)
    {
		for(int k = 0; k < 3; k++)
		{
			HAL_ADC_Start(&hadc1);
			if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
				 ADC_Value[k] = HAL_ADC_GetValue(&hadc1);
			}
		}
		HAL_ADC_Stop(&hadc1);

		ad1 = (double)(ADC_Value[0] & 0xFFF) * 3300.0f / 4096.0f;
		ad2 = (double)(ADC_Value[1] & 0xFFF) * 3300.0f / 4096.0f;
		ad3 = (double)(ADC_Value[2] & 0xFFF) * 3300.0f / 4096.0f;
		ad4 = (double)(ADC_Value[3] & 0xFFF) * 3300.0f / 4096.0f;
		ad5 = (double)(ADC_Value[4] & 0xFFF) * 3300.0f / 4096.0f;
		ad6 = (double)(ADC_Value[5] & 0xFFF) * 3300.0f / 4096.0f;
		ad7 = (double)(ADC_Value[6] & 0xFFF) * 3300.0f / 4096.0f;
		ad8 = (double)(ADC_Value[7] & 0xFFF) * 3300.0f / 4096.0f;

		// --- 4. JSON 组包 ---
		cJSON *root = cJSON_CreateObject();

		cJSON_AddStringToObject(root, "ID", id);
		cJSON_AddStringToObject(root, "NT", NT);
		cJSON_AddNumberToObject(root, "Sen.1", ad1);
		cJSON_AddNumberToObject(root, "Sen.2", ad2);
		cJSON_AddNumberToObject(root, "Sen.3", ad3);
		cJSON_AddNumberToObject(root, "Sen.4", ad4);
		cJSON_AddNumberToObject(root, "Sen.5", ad5);
		cJSON_AddNumberToObject(root, "Sen.6", ad6);
		cJSON_AddNumberToObject(root, "Sen.7", ad7);
		cJSON_AddNumberToObject(root, "Sen.8", ad8);

		/* 生成 JSON 字符串 */
		char *json = cJSON_Print(root);
		HAL_UART_Transmit(&huart1, (uint8_t *)json, strlen(json), HAL_MAX_DELAY);
		cJSON_free(json);
		cJSON_Delete(root);

		vTaskDelay(250);
		/* 任务延时：每隔200毫秒进行下一轮完整扫描 */
		vTaskDelay(200);
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
  for(;;)
  {
	while (1)
	{
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
	  vTaskDelay(250);
	}
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
