#include "main.h"
#include "gpio.h"
#include "tim.h"

uint32_t count = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2) {
    count++;
    uint32_t arr_value =__HAL_TIM_GetAutoreload(&htim1) +1;
    uint32_t brightness = (__HAL_TIM_GetCompare(&htim1,TIM_CHANNEL_2)+1) % arr_value;
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,brightness);
    }
}