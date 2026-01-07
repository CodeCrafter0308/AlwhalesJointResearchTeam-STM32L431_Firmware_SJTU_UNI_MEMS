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
#include "dma.h"
#include "usart.h"
#include "stdio.h"
#include "stdlib.h"
#include <string.h>
#include "wsn31.h"
#include "cJSON.h"
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

    // 1. 变量定义
    // 使用 static 确保卡尔曼滤波的状态在每次循环中保持记忆，不会被重置
    static float x_est[8] = {100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f, 100.0f}; // 初始值设为非0，防止第一次计算除零
    static float p_est[8] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f}; // 初始协方差
    static float k[8] = {0};

    // 卡尔曼参数 (假设这些是你的调试值)
    const float q = 0.001f; // 过程噪声
    const float r = 0.1f;   // 测量噪声

    uint32_t chipID[3];
    char UUID[9];
    const char* NT = "0000";

    // 协议头 (必须发送)
    uint8_t te[2] = {0x03, 0x96};

    uint16_t adc_buf[10] = {0};
    float res_buf[8] = {0};

    // 2. 硬件初始化
    GetChipID(chipID);
    sprintf(UUID, "%08X", chipID[2]);

    vTaskDelay(250);
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA((ADC_HandleTypeDef*)&hadc1, (uint32_t*)adc_buf, 8);

    // DIP 开关读取
    GPIO_PinState DIP_netAddr[NUM_DIP_PINS];
    GPIO_PinState DIP_sigChan[NUM_DIP_PINS];

    DIP_sigChan[0] = HAL_GPIO_ReadPin(DIP_1_1_GPIO_Port, DIP_1_1_Pin);
    DIP_sigChan[1] = HAL_GPIO_ReadPin(DIP_1_2_GPIO_Port, DIP_1_2_Pin);
    DIP_sigChan[2] = HAL_GPIO_ReadPin(DIP_1_3_GPIO_Port, DIP_1_3_Pin);
    DIP_sigChan[3] = HAL_GPIO_ReadPin(DIP_1_4_GPIO_Port, DIP_1_4_Pin);

    DIP_netAddr[0] = HAL_GPIO_ReadPin(DIP_2_1_GPIO_Port, DIP_2_1_Pin);
    DIP_netAddr[1] = HAL_GPIO_ReadPin(DIP_2_2_GPIO_Port, DIP_2_2_Pin);
    DIP_netAddr[2] = HAL_GPIO_ReadPin(DIP_2_3_GPIO_Port, DIP_2_3_Pin);
    DIP_netAddr[3] = HAL_GPIO_ReadPin(DIP_2_4_GPIO_Port, DIP_2_4_Pin);

    uint8_t netAddr = GetDipSwitchValue(DIP_netAddr, NUM_DIP_PINS);
    uint8_t sigChan = GetDipSwitchValue(DIP_sigChan, NUM_DIP_PINS);

    WSN32_Init(9, sigChan, netAddr);
    MX_USART1_UART_Init_115200();

    /* Infinite loop */
    for(;;)
    {
        // --- 3. 数据采集与处理 ---
        for (int i = 0; i < 8; i++) {
            float z_measured = (float)adc_buf[i] * 3300.0f / 4096.0f;

            // 卡尔曼滤波更新
            // x_est[i] = x_est[i]; // 这一行是多余的，已删除
            p_est[i] = p_est[i] + q;
            k[i] = p_est[i] / (p_est[i] + r);
            x_est[i] = x_est[i] + k[i] * (z_measured - x_est[i]);
            p_est[i] = (1.0f - k[i]) * p_est[i];

            // 计算电阻值 (防止除以零)
            // 只要 x_est 不为0 (初始值已设为100)，这里就不会算出 null
            if (x_est[i] > 0.0001f || x_est[i] < -0.0001f) {
                res_buf[i] = 33000.0f / x_est[i] - 10.0f;
            } else {
                res_buf[i] = 0.0f; // 默认值
            }
        }

        // --- 4. JSON 组包 ---
        cJSON *root = cJSON_CreateObject();
        if (root == NULL) {
            vTaskDelay(100); // 内存不足保护
            continue;
        }

        cJSON_AddStringToObject(root, "ID", UUID);
        cJSON_AddStringToObject(root, "NT", NT);

        char sen_name[8];
        for(int i=0; i<8; i++) {
            sprintf(sen_name, "Sen.%d", i+1);
            cJSON_AddNumberToObject(root, sen_name, res_buf[i]);
        }

        char *json_str = cJSON_Print(root);

        // --- 5. 数据发送 ---
        if (json_str != NULL) {
            // 先发送协议头 (Binary)
            HAL_UART_Transmit(&huart1, te, sizeof(te), 20);

            // 再发送 JSON 数据 (Text)
            /* USER CODE BEGIN 2 */
            // ... 其他初始化代码
            HAL_UART_Transmit(&huart1, (uint8_t *)json_str, strlen(json_str), 1000);

            // 释放内存
            cJSON_free(json_str);
        }

        // 释放 cJSON 对象内存
        cJSON_Delete(root);

        vTaskDelay(500);
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
