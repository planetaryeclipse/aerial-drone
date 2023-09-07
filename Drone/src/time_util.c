/*
 * time_util.c
 *
 *  Created on: Aug 28, 2023
 *      Author: samuel
 */

#include "tim.h"
#include "stm32f4xx_hal.h"
#include "time_util.h"

void delay_us(uint16_t us, TIM_HandleTypeDef* tim){
    __HAL_TIM_SET_COUNTER(tim, 0);
    while(__HAL_TIM_GET_COUNTER(tim) < us);
}

void delay_ms(uint16_t ms, TIM_HandleTypeDef* tim){
    for(int i = 0; i < ms; i++)
        delay_us(1000, tim);
}
