#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h" // Include your HAL library header

//WS2812LED Library stm32 f407
//1. Set the timer frequency to 800kHz and enable DMA
//2. Copy WS2812LED.h to Core/Inc and WS2812LED.c to Core/Src
//3. copy >> int datasentflag=0 << in the main program;
//4. copy this function below >>
/*
 void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_2);
	datasentflag=1;
}
 */
//5. call Set_LED(led number, R, G, B) to set the color of each led
//6. call WS2812_Send(timer handler, channel, timer arr) to send the data


#define MAX_LED 11 //set the total LED
#define USE_BRIGHTNESS 0
#define PI 3.14159265

// Function prototypes
void Set_LED(int LEDnum, int Red, int Green, int Blue);
void Set_Brightness(int brightness);
void WS2812_Send(TIM_HandleTypeDef *htim, uint32_t channel, int timer_arr);
void Reset_LED(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_CONTROL_H */
