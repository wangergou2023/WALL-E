/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTaskMotorHandle;
osThreadId myTaskEncoderHandle;
osMutexId myMutexPrintfHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTaskMotor(void const * argument);
void StartTaskEncoder(void const * argument);

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
  /* Create the mutex(es) */
  /* definition and creation of myMutexPrintf */
  osMutexDef(myMutexPrintf);
  myMutexPrintfHandle = osMutexCreate(osMutex(myMutexPrintf));

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

  /* definition and creation of myTaskMotor */
  osThreadDef(myTaskMotor, StartTaskMotor, osPriorityIdle, 0, 128);
  myTaskMotorHandle = osThreadCreate(osThread(myTaskMotor), NULL);

  /* definition and creation of myTaskEncoder */
  osThreadDef(myTaskEncoder, StartTaskEncoder, osPriorityIdle, 0, 128);
  myTaskEncoderHandle = osThreadCreate(osThread(myTaskEncoder), NULL);

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
    HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
    printf("hello\r\n");
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTaskMotor */
/**
* @brief Function implementing the myTaskMotor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskMotor */
void StartTaskMotor(void const * argument)
{
  /* USER CODE BEGIN StartTaskMotor */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskMotor */
}

/* USER CODE BEGIN Header_StartTaskEncoder */
/**
* @brief Function implementing the myTaskEncoder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskEncoder */
void StartTaskEncoder(void const * argument)
{
  /* USER CODE BEGIN StartTaskEncoder */
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1 | TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1 | TIM_CHANNEL_2);
  /* Infinite loop */
  for(;;)
  {
    int Direction = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);   // 读取电机转动方向
    int CaptureNumber = (short)__HAL_TIM_GET_COUNTER(&htim2); // 读取编码�?
    __HAL_TIM_GET_COUNTER(&htim2) = 0;                        // 计数器重新置0
    printf("Direction1 is %d \r\n", Direction);
    printf("CaptureNumber1 is %d \r\n", CaptureNumber);
    int Direction2 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);   // 读取电机转动方向
    int CaptureNumber2 = (short)__HAL_TIM_GET_COUNTER(&htim3); // 读取编码�?
    __HAL_TIM_GET_COUNTER(&htim3) = 0;                        // 计数器重新置0
    printf("Direction2 is %d \r\n", Direction2);
    printf("CaptureNumber2 is %d \r\n", CaptureNumber2);
    HAL_Delay(500);
  }
  /* USER CODE END StartTaskEncoder */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

