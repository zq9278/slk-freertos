#include "Screen.h"
#define CMD_HEAD 0XEE             // 帧头
#define CMD_TAIL 0XFFFCFFFF       // 帧尾
#define CMD_MAX_SIZE 64           // 帧尾
uint8_t cmd_buffer[CMD_MAX_SIZE]; // 指令缓存

extern EventGroupHandle_t All_EventHandle;
extern TIM_HandleTypeDef htim14;
extern DMA_HandleTypeDef hdma_usart1_tx;
const EventBits_t xBitsToSet = Heat_BIT_0;
const EventBits_t xBitsToSet1 = Motor_BIT_2;
uint8_t SendBuff[12];

void processData(PCTRL_MSG msg)
{
    // 解决大端小端问题。stm32是大端处理
    uint32_t cmd_head = msg->cmd_head; // 指令类型
    cmd_head = ((cmd_head & 0x000000FF) << 24) |
               ((cmd_head & 0x0000FF00) << 8) |
               ((cmd_head & 0x00FF0000) >> 8) |
               ((cmd_head & 0xFF000000) >> 24);
    uint16_t cmd_type = msg->cmd_type; // 指令类型
    cmd_type = ((cmd_type & 0x00FF) << 8) |
               ((cmd_type & 0xFF00) >> 8);
    uint8_t control_type = msg->control_type; // 指令类型
    uint16_t data = msg->data;                // 指令类型
    data = ((data & 0x00FF) << 8) |
           ((data & 0xFF00) >> 8);

    // // 为每个变量创建字符串缓冲区
    // char cmd_head_str[9];     // uint32_t 转换为16进制字符串需要8个字符 + '\0'
    // char cmd_type_str[5];     // uint16_t 转换为16进制字符串需要4个字符 + '\0'
    // char control_type_str[3]; // uint8_t 转换为16进制字符串需要2个字符 + '\0'
    // char data_str[5];         // uint16_t 转换为16进制字符串需要4个字符 + '\0'

    // // 将数值转换为16进制字符串
    // snprintf(cmd_head_str, sizeof(cmd_head_str), "%08X", cmd_head);
    // snprintf(cmd_type_str, sizeof(cmd_type_str), "%04X", cmd_type);
    // snprintf(control_type_str, sizeof(control_type_str), "%02X", control_type);
    // snprintf(data_str, sizeof(data_str), "%04X", data);

    // // 打印转换后的字符串
    // printf("cmd_head: %s, cmd_type: %s, control_type: %s, data: %s\n",
    //        cmd_head_str, cmd_type_str, control_type_str, data_str);

    switch (cmd_type)
    {
        /*热敷开始*/

    case 0x1041:
        HeatPIDInit();
        TMP114_Init();
        xEventGroupSetBits(All_EventHandle, xBitsToSet); // 设定热敷任务开启标志位
        HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);//enable pwm for heating film 
        break;

    /*热敷结束*/
    case 0x1030:
        xEventGroupClearBits(All_EventHandle, xBitsToSet); // 清除热敷任务开启标志位
        // 发送停止位
        break;

        /*脉动开始*/
        case 0x1005:
        	// ForceSet=ScreenCmdData/50;//设定压力
        	// ForceRawSet=ForceSet*HX711_SCALE_FACTOR;
        	// WorkMode = 0x02;
            xEventGroupSetBits(All_EventHandle, xBitsToSet1); // 设定脉动任务开启标志位
        	break;

        /*脉动结束*/
        case 0x1034:
        	// WorkMode=0;
        	// HAL_TIM_Base_Stop_IT(&htim6);
        	// Tim6Cnt=0;
        	// MotorState=0;
        	// MotorChecking();
            xEventGroupClearBits(All_EventHandle, xBitsToSet1); // 清除脉动任务开启标志位
        	break;

        /*自动模式开始*/
        case 0x1037:
        	// ForceSet=ScreenCmdData/50;//设定压力
        	// ForceRawSet=ForceSet*HX711_SCALE_FACTOR;
        	// WorkMode = 0x03;
            // xEventGroupSetBits(All_EventHandle, xBitsToSet); // 设定热敷任务开启标志位
            //  xEventGroupSetBits(All_EventHandle, Motor_BIT_2); // 设定脉动任务开启标志位
             xEventGroupSetBits(All_EventHandle, Auto_BIT_3); // 设定自动任务开启标志位
        	break;

        /*自动模式结束*/
        case 0x1038:
        	// WorkMode=0;
        	// HeatPower(OFF);
        	// HeatPWMVal=0;
        	// HAL_TIM_Base_Stop_IT(&htim6);
        	// Tim6Cnt=0;
        	// MotorState=0;
        	// MotorChecking();
            // xEventGroupClearBits(All_EventHandle, xBitsToSet); // 清除热敷任务开启标志位
            // xEventGroupClearBits(All_EventHandle, Motor_BIT_2); // 清除脉动任务开启标志位
            xEventGroupClearBits(All_EventHandle, Auto_BIT_3); // 清除脉动任务开启标志位
        	break;

    default:
        break;
    }
}

void ScreenUpdateTemperature(float value, uint16_t work_mode)
{
    uint16_t Tmpvalue;

    Tmpvalue = value + 0.5f;
    while (hdma_usart1_tx.State != HAL_DMA_STATE_READY)
        ;

    // SendBuff[4]=0x03;
    // SendBuff[6]=0x02;

    // SendBuff[4]=0x0C;
    // SendBuff[6]=0x03;
    SendBuff[4] = (work_mode >> 8) & 0xFF; // 高字节
    SendBuff[6] = work_mode & 0xFF;        // 低字节

    SendBuff[0] = 0xEE;
    SendBuff[1] = 0xB1;
    SendBuff[2] = 0x10;
    SendBuff[3] = 0x00;

    SendBuff[5] = 0x00;

    SendBuff[7] = Tmpvalue / 10 + '0';
    SendBuff[8] = Tmpvalue % 10 + '0';
    SendBuff[9] = 0xFF;
    SendBuff[10] = 0xFC;
    SendBuff[11] = 0xFF;
    SendBuff[12] = 0xFF;

    HAL_UART_Transmit_DMA(&huart1, SendBuff, 13);
}

void ScreenUpdateForce(uint32_t value,uint16_t work_mode)
{
	
	
	Forcevalue=(uint16_t)(value/HX711_SCALE_FACTOR_10*6);
	
	while(hdma_usart1_tx.State!=HAL_DMA_STATE_READY);
	// if(WorkMode==0x06)
	// {
	// 	SendBuff[4]=0x07;
	// 	SendBuff[6]=0x02;
	// }
	// else if(WorkMode==0x07)
	// {
	// 	SendBuff[4]=0x0C;
	// 	SendBuff[6]=0x04;
	// }
    SendBuff[4] = (work_mode >> 8) & 0xFF; // 高字节
    SendBuff[6] = work_mode & 0xFF;        // 低字节
	SendBuff[0]=0xEE;
	SendBuff[1]=0xB1;
	SendBuff[2]=0x10;
	SendBuff[3]=0x00;
	
	SendBuff[5]=0x00;
	
	SendBuff[7]=Forcevalue/100+'0';
	SendBuff[8]=Forcevalue/10%10+'0';
	SendBuff[9]=Forcevalue%10+'0';
	SendBuff[10]=0xFF;
	SendBuff[11]=0xFC;
	SendBuff[12]=0xFF;
	SendBuff[13]=0xFF;
	
	
	HAL_UART_Transmit_DMA(&huart1,SendBuff,14);	
}

void ScreenUpdateSOC(uint16_t value,uint8_t state)
{
	uint8_t SOCvalue;
	SOCvalue=value/20;
	if(SOCvalue==5)SOCvalue=4;
	if(state==1 || state==2)SOCvalue+=5;
	while(hdma_usart1_tx.State!=HAL_DMA_STATE_READY);
	SendBuff[0]=0xEE;
	SendBuff[1]=0xB1;
	SendBuff[2]=0x23;
	SendBuff[3]=0x00;
	SendBuff[4]=0x1E;
	SendBuff[5]=0x27;
	SendBuff[6]=0x15;
	SendBuff[7]=SOCvalue;
	SendBuff[8]=0xFF;
	SendBuff[9]=0xFC;
	SendBuff[10]=0xFF;
	SendBuff[11]=0xFF;
	
	HAL_UART_Transmit_DMA(&huart1,SendBuff,12);	
}

void ScreenWorkModeQuit(uint8_t workmodenumber)
{
	while(hdma_usart1_tx.State!=HAL_DMA_STATE_READY);
	SendBuff[0]=0xEE;
	SendBuff[1]=0xB1;
	SendBuff[2]=0x10;
	SendBuff[3]=0x00;
	// if(workmodenumber==0x05)SendBuff[4]=0x03;
	// if(workmodenumber==0x06)SendBuff[4]=0x07;
	// if(workmodenumber==0x07)SendBuff[4]=0x0C;
	SendBuff[4]=workmodenumber;
	SendBuff[5]=0x00;
	SendBuff[6]=0x07;
	SendBuff[7]=0x32;
	SendBuff[8]=0xFF;
	SendBuff[9]=0xFC;
	SendBuff[10]=0xFF;
	SendBuff[11]=0xFF;
	
	HAL_UART_Transmit_DMA(&huart1,SendBuff,12);	
}

void ScreenTimerStart(uint8_t workmodenumber)
{
	while(hdma_usart1_tx.State!=HAL_DMA_STATE_READY);
	SendBuff[0]=0xEE;
	SendBuff[1]=0xB1;
	SendBuff[2]=0x10;
	SendBuff[3]=0x00;
	// if(workmodenumber==0x05)SendBuff[4]=0x03;
	// if(workmodenumber==0x06)SendBuff[4]=0x07;
	// if(workmodenumber==0x07)SendBuff[4]=0x0C;
    	SendBuff[4]=workmodenumber;
	SendBuff[5]=0x00;
	SendBuff[6]=0x07;
	SendBuff[7]=0x31;
	SendBuff[8]=0xFF;
	SendBuff[9]=0xFC;
	SendBuff[10]=0xFF;
	SendBuff[11]=0xFF;
	
	HAL_UART_Transmit_DMA(&huart1,SendBuff,12);	
}
