/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
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
uint8_t buffer[10];
//QueueHandle_t dataQueue = NULL;
/* USER CODE END Variables */
/* Definitions for Moto_Task */
osThreadId_t Moto_TaskHandle;
const osThreadAttr_t Moto_Task_attributes = {
  .name = "Moto_Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 130 * 4
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 130 * 4
};
/* Definitions for Uart_ProcessTas */
osThreadId_t Uart_ProcessTasHandle;
const osThreadAttr_t Uart_ProcessTas_attributes = {
  .name = "Uart_ProcessTas",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 200 * 4
};
/* Definitions for dataQueue */
osMessageQueueId_t dataQueueHandle;
const osMessageQueueAttr_t dataQueue_attributes = {
  .name = "dataQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void AppMoto_Task(void *argument);
void StartTask02(void *argument);
void App_Uart_ProcessTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* Create the queue(s) */
  /* creation of dataQueue */
  dataQueueHandle = osMessageQueueNew (16, sizeof(buffer), &dataQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Moto_Task */
  Moto_TaskHandle = osThreadNew(AppMoto_Task, NULL, &Moto_Task_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of Uart_ProcessTas */
  Uart_ProcessTasHandle = osThreadNew(App_Uart_ProcessTask, NULL, &Uart_ProcessTas_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_AppMoto_Task */
/**
 * @brief  Function implementing the Moto_Task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_AppMoto_Task */
void AppMoto_Task(void *argument)
{
  /* USER CODE BEGIN AppMoto_Task */
  TMC5130_Init();
  HAL_GPIO_WritePin(TMC_ENN_GPIO_Port, TMC_ENN_Pin, GPIO_PIN_RESET);//使能tmc电机引脚
  TMC5130_Write(0xa7, 0x20000);//设置tmc电机速度
  /* Infinite loop */
  for (;;)
  {
    TMC5130_Write(0xa0, 1);//设置tmc电机方向向前
    vTaskDelay(1000);
    TMC5130_Write(0xa0, 2);
    vTaskDelay(1000);
  }
  /* USER CODE END AppMoto_Task */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for (;;)
  {
    vTaskDelay(50);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_App_Uart_ProcessTask */
/**
 * @brief Function implementing the Uart_ProcessTas thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_App_Uart_ProcessTask */
void App_Uart_ProcessTask(void *argument)
{
  /* USER CODE BEGIN App_Uart_ProcessTask */
  /* Infinite loop */
  for (;;)
  {
    vTaskDelay(100);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
    if (xQueueReceive(dataQueueHandle, buffer, portMAX_DELAY))//阻塞接受队列消息
    {
     printf("%s",buffer);
    }
     // 处理接收到的数据
      //processData(buffer);
      // unsigned short size=queue_find_cmd(buffer,CMD_MAX_SIZE);
      // processData(PCTRL_MSG msg, uint16_t size)
  }
  /* USER CODE END App_Uart_ProcessTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

