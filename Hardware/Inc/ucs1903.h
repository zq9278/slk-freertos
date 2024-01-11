#include "main.h"
//#define LED_TIM_HANDLE TIM16
#define LED_TIM_CHANNEL TIM_CHANNEL_1




void sendBit(uint8_t bit);

void sendByte(uint8_t b);

void sendColor(uint8_t red, uint8_t green, uint8_t blue, uint8_t length);
