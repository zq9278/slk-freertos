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

uart_data uart_rx_data;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim7;
extern uint8_t HeatPWMVal;
extern TIM_HandleTypeDef htim16;
uint8_t MotorCompareState = 0;

extern uint8_t EyeTmpRaw[2];
extern float EyeTmp;
extern int32_t ForceRawSet;

extern PID_typedef HeatPID;

const char *str1 = "queue";
char HeatPWMVal_str[3];
float temperature;
int Force;
int32_t ForceRawActual;
int32_t ForceRawOffset;
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
    .priority = (osPriority_t)osPriorityHigh,
    .stack_size = 130 * 4};
/* Definitions for dataQueue */
osMessageQueueId_t dataQueueHandle;
const osMessageQueueAttr_t dataQueue_attributes = {
    .name = "dataQueue"};
/* Definitions for Temperature_Queue */
osMessageQueueId_t Temperature_QueueHandle;
const osMessageQueueAttr_t Temperature_Queue_attributes = {
    .name = "Temperature_Queue"};
/* Definitions for Force_Queue */
osMessageQueueId_t Force_QueueHandle;
const osMessageQueueAttr_t Force_Queue_attributes = {
    .name = "Force_Queue"};
/* Definitions for Keep_Force_Timer */
osTimerId_t Keep_Force_TimerHandle;
const osTimerAttr_t Keep_Force_Timer_attributes = {
    .name = "Keep_Force_Timer"};
/* Definitions for Zero_Force_Timer */
osTimerId_t Zero_Force_TimerHandle;
const osTimerAttr_t Zero_Force_Timer_attributes = {
    .name = "Zero_Force_Timer"};
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
void Keep_Force(void *argument);
void Zero_Force(void *argument);

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

  /* Create the timer(s) */
  /* creation of Keep_Force_Timer */
  Keep_Force_TimerHandle = osTimerNew(Keep_Force, osTimerPeriodic, NULL, &Keep_Force_Timer_attributes);

  /* creation of Zero_Force_Timer */
  Zero_Force_TimerHandle = osTimerNew(Zero_Force, osTimerPeriodic, NULL, &Zero_Force_Timer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of dataQueue */
  dataQueueHandle = osMessageQueueNew(3, sizeof(uart_rx_data), &dataQueue_attributes);

  /* creation of Temperature_Queue */
  Temperature_QueueHandle = osMessageQueueNew(100, sizeof(float), &Temperature_Queue_attributes);

  /* creation of Force_Queue */
  Force_QueueHandle = osMessageQueueNew(10, sizeof(float), &Force_Queue_attributes);

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

  EventBits_t Motor_Event_Bit;

  TMC5130_Init();
  HX711_Init();

  MotorChecking();
  TMC5130_Write(0xa7, 0x10000);
  TMC5130_Write(0xa0, 1);
  // HAL_GPIO_WritePin(TMC_ENN_GPIO_Port, TMC_ENN_Pin, GPIO_PIN_RESET); // 使能tmc电机引脚
  for (;;)
  {
    Motor_Event_Bit = xEventGroupWaitBits(
        All_EventHandle,                     // Event group handle
        Motor_BIT_2 | Auto_BIT_3 | SW_BIT_1, // flag bits to wait for
        pdFALSE,                             // clear these bits when the function responds
        pdFALSE,                             // Whether to wait for all flag bits
        200                                  // Whether to wait indefinitely
                                             // portMAX_DELAY    // Whether to wait indefinitely
    );
    if ((((Motor_Event_Bit & Motor_BIT_2) != 0) || ((Motor_Event_Bit & Auto_BIT_3) != 0)) && ((Motor_Event_Bit & SW_BIT_1) == 0)) // 脉动或�?�自动事件发生，按钮事件没发生（预热模式�???????
    {
      // vTaskDelay(200);
      // printf("预电机模式\n");

      HAL_GPIO_WritePin(TMC_ENN_GPIO_Port, TMC_ENN_Pin, GPIO_PIN_RESET); // 使能tmc电机引脚
      ForceRawActual = HX711_Read();

      if ( (ForceRawActual-(ForceRawOffset + ForceRawSet))>32007)
      {
        HAL_TIM_Base_Start_IT(&htim7);
        // vTaskDelay(2000);
      }
        switch (MotorCompareState)
        {
        case 0:
          MotorCompare(ForceRawOffset + ForceRawSet, ForceRawActual);
          break;
        case 1:
          TMC5130_Write(0xa7, 0x6000);
          TMC5130_Write(0xa0, 2);
          break;

        default:
          break;
        
      }

      // uint8_t MotorCompareState=MotorCompare(ForceRawOffset+ForceRawSet,ForceRawActual);

      Force = (ForceRawActual - ForceRawOffset < 0) ? 0 : (ForceRawActual - ForceRawOffset);
      xQueueSend(Force_QueueHandle, &Force, NULL);
    }
    else if (((Motor_Event_Bit & (Motor_BIT_2 | SW_BIT_1)) == (Motor_BIT_2 | SW_BIT_1)) || (Motor_Event_Bit & (Auto_BIT_3 | SW_BIT_1)) == (Auto_BIT_3 | SW_BIT_1)) // 脉动或�?�自动事件发生，按钮事件发生（正式脉动模式�?
    {
      vTaskDelay(200);

      printf("正式脉动模式");
    }

    // TMC5130_Write(0xa0, 1); // 设置tmc电机方向向前
    // vTaskDelay(1000);
    // TMC5130_Write(0xa0, 2);
    // vTaskDelay(1000);
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

  for (;;)
  {
    Heat_Event_Bit = xEventGroupWaitBits(
        All_EventHandle,                    // Event group handle
        Heat_BIT_0 | Auto_BIT_3 | SW_BIT_1, // flag bits to wait for
        pdFALSE,                            // clear these bits when the function responds
        pdFALSE,                            // Whether to wait for all flag bits
        100                                 // Whether to wait indefinitely
                                            // portMAX_DELAY                      // Whether to wait indefinitely
    );
    // if ((Heat_Event_Bit & (BIT_0 | BIT_1)) == (BIT_0 | BIT_1)) {
    // if ( ((Heat_Event_Bit & Heat_BIT_0) != 0)&&((Heat_Event_Bit & SW_BIT_1) == 0))//加热事件发生，按钮事件没发生（预热模式）
    if ((((Heat_Event_Bit & Heat_BIT_0) || ((Heat_Event_Bit & Auto_BIT_3) != 0)) != 0) && ((Heat_Event_Bit & SW_BIT_1) == 0)) // 加热或自动事件发生，按钮事件没发生（预热模式�???????
    {
      // printf("预加热模式\n");
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
      vTaskDelay(100);
      TMP114_Read(0x00, EyeTmpRaw);    // obtain original value of the current temperature sensor by reading the iic
      EyeTmp = TmpRaw2Ture(EyeTmpRaw); // convert raw temperature data
      xQueueSend(Temperature_QueueHandle, &EyeTmp, NULL);
      // printf("Temperature:%f\n", EyeTmp);
      HeatPWMVal = PID_realize(&HeatPID, EyeTmp); // Obtain PWM value through PID algorithm
      // snprintf(HeatPWMVal_str, sizeof(HeatPWMVal_str), "%02X", HeatPWMVal);
      //  printf("PWM:%s\n", HeatPWMVal_str);
      __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, HeatPWMVal); // enable timer comparison to generate PWM
    }
    // else if( ((Heat_Event_Bit & Heat_BIT_0) != 0)&&((Heat_Event_Bit & SW_BIT_1) != 0))//加热事件发生，按钮事件发生（正式加热模式�????????
    else if (((Heat_Event_Bit & (Heat_BIT_0 | SW_BIT_1)) == (Heat_BIT_0 | SW_BIT_1)) || ((Heat_Event_Bit & (Auto_BIT_3 | SW_BIT_1)) == (Auto_BIT_3 | SW_BIT_1))) // 加热或�?�自动事件发生，按钮事件发生（正式加热模式�?
    {
      printf("正式加热模式");
      vTaskDelay(100);
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
  // sendColor(0, 0, 25);
  // UCS1903Show();
  EventBits_t Data_Event_Bit;
  float temperature_buffer[FILTER_SIZE] = {0};
  int buffer_index = 0;
  uint32_t Force_buffer[FILTER_SIZE] = {0};
  int buffer_index_Force = 0;

  for (;;)
  {

    Data_Event_Bit = xEventGroupWaitBits(
        All_EventHandle,                                  // Event group handle
        Heat_BIT_0 | Auto_BIT_3 | Motor_BIT_2 | SW_BIT_1, // flag bits to wait for
        pdFALSE,                                          // clear these bits when the function responds
        pdFALSE,                                          // Whether to wait for all flag bits
        100                                               // Whether to wait indefinitely
                                                          // portMAX_DELAY    // Whether to wait indefinitely
    );
    // vTaskDelay(100);

    if ((Data_Event_Bit & Heat_BIT_0) != 0)

    {                                                               // printf("打开热敷数据");
      if (xQueueReceive(Temperature_QueueHandle, &temperature, 10)) // 阻塞接受温度队列消息
      {
        temperature_buffer[buffer_index] = temperature;  // 取出队列的数�?????
        buffer_index = (buffer_index + 1) % FILTER_SIZE; // 累加到缓冲区
        if (buffer_index == 0)                           // 当达到预设平均滤波数量时，平均一�?????
        {
          float filtered_temperature = processFilter(temperature_buffer); // 计算均�??
          ScreenUpdateTemperature(filtered_temperature, 0x0302);          // 向屏幕发送数�????
          // HAL_UART_Transmit(&huart1, (uint8_t *)&filtered_temperature, sizeof(float), 0xFFFF);
        }
      }
    }
    if ((Data_Event_Bit & Motor_BIT_2) != 0)
    {                                                   // printf("打开压力数据");
      if (xQueueReceive(Force_QueueHandle, &Force, 10)) // 阻塞接受温度队列消息
      {
        Force_buffer[buffer_index_Force] = Force;                    // 取出队列的数�?????
        buffer_index_Force = (buffer_index_Force + 1) % FILTER_SIZE; // 累加到缓冲区
                                                                     // ScreenUpdateForce(Force, 0x0702);          // 向屏幕发送数�????
        if (buffer_index_Force == 0)                                 // 当达到预设平均滤波数量时，平均一�?????
        {
          uint32_t filtered_Force = processFilter_force(Force_buffer); // 计算均�??
          ScreenUpdateForce(filtered_Force, 0x0702);                   // 向屏幕发送数�????
        }
      }
    }
    if ((Data_Event_Bit & Auto_BIT_3) != 0)
    { // printf("打开自动数据");

      if (xQueueReceive(Temperature_QueueHandle, &temperature, 10)) // 阻塞接受温度队列消息
      {
        temperature_buffer[buffer_index] = temperature;  // 取出队列的数�?????
        buffer_index = (buffer_index + 1) % FILTER_SIZE; // 累加到缓冲区
        if (buffer_index == 0)                           // 当达到预设平均滤波数量时，平均一�?????
        {
          float filtered_temperature = processFilter(temperature_buffer);
          // printf("%f\n", filtered_temperature);
          ScreenUpdateTemperature(filtered_temperature, 0x0C03);
          // HAL_UART_Transmit(&huart1, (uint8_t *)&filtered_temperature, sizeof(float), 0xFFFF);
        }
      }

      if (xQueueReceive(Force_QueueHandle, &Force, 10)) // 阻塞接受压力队列消息
      {
        // Force_buffer[buffer_index_Force] = Force;        // 取出队列的数�?????
        // buffer_index_Force = (buffer_index_Force + 1) % FILTER_SIZE; // 累加到缓冲区
        // if (buffer_index_Force == 0)                           // 当达到预设平均滤波数量时，平均一�?????
        {
          // float filtered_Force = processFilter_force(Force_buffer); // 计算均�??
          // ScreenUpdateForce(filtered_Force, 0x0C04);          // 向屏幕发送数�????        // 向屏幕发送数�????
        }
      }
    }
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    if (xQueueReceive(dataQueueHandle, &uart_rx_data, 10)) // 阻塞接受队列消息
    {
      // HAL_UART_Transmit(&huart1, (uint8_t *)&(uart_rx_data.buffer), uart_rx_data.length, 0xFFFF);
      processData((PCTRL_MSG)uart_rx_data.buffer); // 处理接收到的数据
    }
  }
  /* USER CODE END App_Uart_ProcessTask */
}

/* Keep_Force function */
void Keep_Force(void *argument)
{
  /* USER CODE BEGIN Keep_Force */
  MotorCompare(ForceRawOffset + ForceRawSet, ForceRawActual);
  /* USER CODE END Keep_Force */
}

/* Zero_Force function */
void Zero_Force(void *argument)
{
  /* USER CODE BEGIN Zero_Force */
  TMC5130_Write(0xa7, 0x8000);
  TMC5130_Write(0xa0, 2);
  /* USER CODE END Zero_Force */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
