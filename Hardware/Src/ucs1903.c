
#include "ucs1903.h"


extern TIM_HandleTypeDef htim16;


void sendBit(uint8_t bit){
    if(bit){
        // 发送1
        __HAL_TIM_SET_COMPARE(&htim16, LED_TIM_CHANNEL, 80); // 调整这个值来匹配UCS1903的时序
    } else {
        // 发送0
        __HAL_TIM_SET_COMPARE(&htim16, LED_TIM_CHANNEL, 20); // 调整这个值来匹配UCS1903的时序
    }
    //vTaskDelay(1);
    for(int i = 0; i < 160; i++){
        __NOP();
    }
}

void sendByte(uint8_t b){
    for(int i = 0; i < 8; i++){
        sendBit(b & 0x80);
        b <<= 1;
    }
}


void sendColor(uint8_t red, uint8_t green, uint8_t blue, uint8_t length){
    for(int i = 0; i < length; i++){
         sendByte(green);
    sendByte(red);
    sendByte(blue);
    //vTaskDelay(1);
    }
  
}

// int main(void){
//     HAL_Init();
//     MX_TIM3_Init(); // 初始化定时器和PWM

//     while(1){
//         sendColor(0, 0, 255); // 发送蓝色
//         HAL_Delay(1000); // 等待一段时间
//     }
// }