/*
 * usr_interrupts.c
 *
 *  Created on: Aug 28, 2023
 *      Author: samuel
 */

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_uart.h"

#include "printf/printf.h"
#include "radio.h"
#include "dist_sens.h"

#include "main.h"

#include "usart.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        RadioRxCpltCallback();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == RNG1_ECHO_Pin){
        Rng1EchoCallback();
    }
    else if (GPIO_Pin == RNG2_ECHO_Pin){
        Rng2EchoCallback();
    }
}
