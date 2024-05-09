#include "WS2812LED.h"
#include <math.h>

// Define global variables
uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];
uint16_t pwmData[(24 * MAX_LED) + 50];

// Function definitions
void Set_LED(int LEDnum, int Red, int Green, int Blue) {
    LED_Data[LEDnum][0] = LEDnum;
    LED_Data[LEDnum][1] = Green;
    LED_Data[LEDnum][2] = Red;
    LED_Data[LEDnum][3] = Blue;
}

void Set_Brightness(int brightness) {
#if USE_BRIGHTNESS
    if (brightness > 45) brightness = 45;
    for (int i = 0; i < MAX_LED; i++) {
        LED_Mod[i][0] = LED_Data[i][0];
        for (int j = 1; j < 4; j++) {
            float angle = 90 - brightness;  // in degrees
            angle = angle * PI / 180;        // in rad
            LED_Mod[i][j] = (LED_Data[i][j]) / (tan(angle));
        }
    }
#endif
}

void WS2812_Send(TIM_HandleTypeDef *htim, uint32_t channel, int timer_arr) {
    uint32_t indx = 0;
    uint32_t color;
    float twothird = (2.0 / 3.0) * timer_arr;
    float onethird = (1.0 / 3.0) * timer_arr;

    for (int i = 0; i < MAX_LED; i++) {
#if USE_BRIGHTNESS
        color = ((LED_Mod[i][1] << 16) | (LED_Mod[i][2] << 8) | (LED_Mod[i][3]));
#else
        color = ((LED_Data[i][1] << 16) | (LED_Data[i][2] << 8) | (LED_Data[i][3]));
#endif

        for (int i = 23; i >= 0; i--) {
            if (color & (1 << i)) {
                pwmData[indx] = (uint16_t)twothird;  // 2/3 of arr
            } else {
                pwmData[indx] = (uint16_t)onethird;  // 1/3 of arr
            }
            indx++;
        }
    }

    for (int i = 0; i < 50; i++) {
        pwmData[indx] = 0;
        indx++;
    }

    HAL_TIM_PWM_Start_DMA(htim, channel, (uint32_t *)pwmData, indx);
}

void Reset_LED(void) {
    for (int i = 0; i < MAX_LED; i++) {
        LED_Data[i][0] = i;
        LED_Data[i][1] = 0;
        LED_Data[i][2] = 0;
        LED_Data[i][3] = 0;
    }
}
