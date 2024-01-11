/* USER CODE BEGIN Header */
/**
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

uint8_t buffer[200];
extern TIM_HandleTypeDef htim14;
extern uint8_t HeatPWMVal;
extern TIM_HandleTypeDef htim16;

extern uint8_t EyeTmpRaw[2];
extern float EyeTmp;

extern PID_typedef HeatPID;

const char *str1 = "queue";
char HeatPWMVal_str[3];
/* USER CODE END Variables */
/* Definitions for Motor_Task */
osThreadId_t Motor_TaskHandle;
const osThreadAttr_t Motor_Task_attributes = {
    .name = "Motor_Task",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 128 * 4};
/* Definitions for HeatTask */
osThreadId_t HeatTaskHandle;
const osThreadAttr_t HeatTask_attributes = {
    .name = "HeatTask",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 128 * 4};
/* Definitions for Uart_ProcessTas */
osThreadId_t Uart_ProcessTasHandle;
const osThreadAttr_t Uart_ProcessTas_attributes = {
    .name = "Uart_ProcessTas",
    .priority = (osPriority_t)osPriorityNormal,
    .stack_size = 130 * 4};
/* Definitions for dataQueue */
osMessageQueueId_t dataQueueHandle;
const osMessageQueueAttr_t dataQueue_attributes = {
    .name = "dataQueue"};
/* Definitions for All_Event */
osEventFlagsId_t All_EventHandle;
const osEventFlagsAttr_t All_Event_attributes = {
    .name = "All_Event"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void AppMotor_Task(void *argument);
void APP_HeatTask(void *argument);
void App_Uart_ProcessTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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
  dataQueueHandle = osMessageQueueNew(3, sizeof(buffer), &dataQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Motor_Task */
  Motor_TaskHandle = osThreadNew(AppMotor_Task, NULL, &Motor_Task_attributes);

  /* creation of HeatTask */
  HeatTaskHandle = osThreadNew(APP_HeatTask, NULL, &HeatTask_attributes);

  /* creation of Uart_ProcessTas */
  Uart_ProcessTasHandle = osThreadNew(App_Uart_ProcessTask, NULL, &Uart_ProcessTas_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of All_Event */
  All_EventHandle = osEventFlagsNew(&All_Event_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */

  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_AppMotor_Task */
/**
 * @brief  Function implementing the Motor_Task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_AppMotor_Task */
void AppMotor_Task(void *argument)
{
  /* USER CODE BEGIN AppMotor_Task */
  TMC5130_Init();
  HAL_GPIO_WritePin(TMC_ENN_GPIO_Port, TMC_ENN_Pin, GPIO_PIN_RESET); // 使能tmc电机引脚
  TMC5130_Write(0xa7, 0x20000);
  for (;;)
  {
    TMC5130_Write(0xa0, 1); // 设置tmc电机方向向前
    vTaskDelay(1000);
    TMC5130_Write(0xa0, 2);
    vTaskDelay(1000);
  }

  /* USER CODE END AppMotor_Task */
}

/* USER CODE BEGIN Header_APP_HeatTask */
/**
 * @brief Function implementing the HeatTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_APP_HeatTask */
void APP_HeatTask(void *argument)
{
  /* USER CODE BEGIN APP_HeatTask */
  /* Infinite loop */
  EventBits_t Heat_Event_Bit;

  HeatPIDInit();
  TMP114_Init();
  for (;;)
  {
    Heat_Event_Bit = xEventGroupWaitBits(
        All_EventHandle, // Event group handle
        BIT_0,           // flag bits to wait for
        pdFALSE,         // clear these bits when the function responds
        pdTRUE,          // Whether to wait for all flag bits
        portMAX_DELAY    // Whether to wait indefinitely
    );
    // if ((Heat_Event_Bit & (BIT_0 | BIT_1)) == (BIT_0 | BIT_1)) {
    if ((Heat_Event_Bit & BIT_0) == BIT_0)
    {
      vTaskDelay(100);
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);

      TMP114_Read(0x00, EyeTmpRaw);    // obtain original value of the current temperature sensor by reading the iic
      EyeTmp = TmpRaw2Ture(EyeTmpRaw); // convert raw temperature data
      //printf("Temperature:%f\n", EyeTmp);
      HeatPWMVal = PID_realize(&HeatPID, EyeTmp); // Obtain PWM value through PID algorithm
      snprintf(HeatPWMVal_str, sizeof(HeatPWMVal_str), "%02X", HeatPWMVal);
      //printf("PWM:%s\n", HeatPWMVal_str);
      __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, HeatPWMVal); // enable timer comparison to generate PWM
      //ScreenUpdateTemperature(EyeTmp, 0x0302);                   // send data to the serial screen
    }
  }

  /* USER CODE END APP_HeatTask */
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
  // HAL_TIM_Base_Start(&htim16);
  // HAL_TIM_PWM_Start(&htim16, LED_TIM_CHANNEL);
  __HAL_TIM_ENABLE_DMA(&htim16, TIM_DMA_CC1);
  sendColor(0, 0, 25);
  UCS1903Show();
  for (;;)
  {
    // vTaskDelay(50);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    if (xQueueReceive(dataQueueHandle, buffer, 100)) // 阻塞接受队列消息
    {
      processData((PCTRL_MSG)buffer); // 处理接收到的数据
      
    }
  }
  /* USER CODE END App_Uart_ProcessTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
