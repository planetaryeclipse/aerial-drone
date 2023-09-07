/*
 * dist_sens.c
 *
 *  Created on: Aug 28, 2023
 *      Author: samuel
 */

#include "main.h"
#include "cmsis_os.h"

#include "dist_sens.h"
#include "stdbool.h"

#include "tim.h"
#include "time_util.h"

#include "sens_types.h"

#include "printf/printf.h"

#define SPEED_SOUND_GROUND 343

#define RNG1_QUEUE_PUT_TIMEOUT_MS 5
#define RNG2_QUEUE_PUT_TIMEOUT_MS 5

extern osMessageQueueId_t rng1ToLogHandle;
extern osMessageQueueId_t rng2ToLogHandle;

bool rng1_echo_started = false;
bool rng2_echo_started = false;

void ReadRngs(){
    for(;;){
        // Generates the triggers to cause a read, the echos are handled by interrupts

        osKernelLock(); // Prevents switching while generating trigger pulse

        // Generates pulses concurrently
        HAL_GPIO_WritePin(RNG1_TRIG_GPIO_Port, RNG1_TRIG_Pin, GPIO_PIN_SET);
        delay_us(10, &htim10);
        HAL_GPIO_WritePin(RNG1_TRIG_GPIO_Port, RNG1_TRIG_Pin, GPIO_PIN_RESET);

        HAL_GPIO_WritePin(RNG2_TRIG_GPIO_Port, RNG2_TRIG_Pin, GPIO_PIN_SET);
        delay_us(10, &htim11);
        HAL_GPIO_WritePin(RNG2_TRIG_GPIO_Port, RNG2_TRIG_Pin, GPIO_PIN_RESET);

        osKernelUnlock();

        osDelay(1000); // Measures at 10 Hz
    }

    osThreadTerminate(NULL);
}

void Rng1EchoCallback(){
    if (HAL_GPIO_ReadPin(RNG1_ECHO_GPIO_Port, RNG1_ECHO_Pin)){
        // printf("Rng1 echo rising edge!\n");
        __HAL_TIM_SET_COUNTER(&htim10, 0);
        rng1_echo_started = true;
    } else if (rng1_echo_started){
        // printf("Rng1 echo falling edge FOLLOWING rising!\n");
        // Triggers the echo only if the rising edge was previously detected
        rng1_echo_started = false;
        uint32_t echo_duration_us = __HAL_TIM_GET_COUNTER(&htim10);
        float calc_rng = ((float)echo_duration_us / 1e6) * SPEED_SOUND_GROUND / 2;

        printf("echo duration (us): %i\n", echo_duration_us);
        printf("Rng1: %.03f\n", calc_rng);

        sens_rng_t rng_reading;
        rng_reading.timestamp_ms = osKernelGetTickCount();
        rng_reading.rng = calc_rng;

        osMessageQueuePut(rng1ToLogHandle, &rng_reading, 0, 0);
    }
}

void Rng2EchoCallback(){
    if (HAL_GPIO_ReadPin(RNG2_ECHO_GPIO_Port, RNG2_ECHO_Pin)){
        __HAL_TIM_SET_COUNTER(&htim11, 0);
        rng2_echo_started = true;
    } else if (rng2_echo_started){
        rng2_echo_started = false;
        uint32_t echo_duration_us = __HAL_TIM_GET_COUNTER(&htim11);
        float calc_rng = ((float)echo_duration_us / 1e6) * SPEED_SOUND_GROUND / 2;

        printf("echo duration (us): %i\n", echo_duration_us);
        printf("Rng2: %.03f\n", calc_rng);

        sens_rng_t rng_reading;
        rng_reading.timestamp_ms = osKernelGetTickCount();
        rng_reading.rng = calc_rng;

        osMessageQueuePut(rng2ToLogHandle, &rng_reading, 0, 0);
    }
}
