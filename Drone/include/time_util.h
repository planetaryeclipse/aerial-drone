/*
 * time_util.h
 *
 *  Created on: Aug 28, 2023
 *      Author: samuel
 */

#ifndef INCLUDE_TIME_UTIL_H_
#define INCLUDE_TIME_UTIL_H_

#include "tim.h"

void delay_us(uint16_t us, TIM_HandleTypeDef* tim);
void delay_ms(uint16_t ms, TIM_HandleTypeDef* tim);

#endif /* INCLUDE_TIME_UTIL_H_ */
