/*
 * sensors.c
 *
 *  Created on: Jul 24, 2023
 *      Author: samuel
 */

#include "main.h"
#include "cmsis_os.h"

#include "printf/printf.h"
#include "time_util.h"

#include "sens_types.h"

extern osMessageQueueId_t pressrToLogHandle;

#define PRESSR_USE_40HZ 0
#define PRESSR_QUEUE_PUT_TIMEOUT_MS 5

/**
* @brief Reads from the HX710b-based pressure sensor
* @param argument: Not used
* @retval None
*/
void ReadPressureSensor(void *argument)
{
	// Sets the SCK pin low to start as this is what the sensor expects
	HAL_GPIO_WritePin(PRESSR_SCK_GPIO_Port, PRESSR_SCK_Pin, GPIO_PIN_RESET);

  /* Infinite loop 	*/
  for(;;)
  {
	/// Needs to wait until PRESSR_OUT is high before value can be read
	if (HAL_GPIO_ReadPin(PRESSR_OUT_GPIO_Port, PRESSR_OUT_Pin) == GPIO_PIN_SET){
		// osThreadYield(); // Ensures nonblocking behaviour
	    //osDelay(1); // Waits until the next clock tick
		osThreadYield();
	    continue;
	}

	osKernelLock(); // Prevents switching while reading sensor

	uint32_t raw_adc = 0;
#if PRESSR_USE_40HZ
	for(int i = 0; i < 27; i++)	{
#else
	for(int i = 0; i < 25; i++) {
#endif
		// The line PRESSR_SCK must be set high and then a slight delay is added to allow the ADC
		// to shift out the next highest bit on PRESSR_OUT. Only the first 24 bits must be stored
		// as the total 25-27 SCK pulses are used to select the data to output.

		HAL_GPIO_WritePin(PRESSR_SCK_GPIO_Port, PRESSR_SCK_Pin, GPIO_PIN_SET);
		delay_us(1, &htim9);

		if (i < 24){
			GPIO_PinState bit_val = HAL_GPIO_ReadPin(PRESSR_OUT_GPIO_Port, PRESSR_OUT_Pin);
			raw_adc |= (bit_val << (23-i));
		}

		HAL_GPIO_WritePin(PRESSR_SCK_GPIO_Port, PRESSR_SCK_Pin, GPIO_PIN_RESET);
		delay_us(1, &htim9);
	}
	osKernelUnlock();

	sens_pressr_t pressr_reading;
	pressr_reading.timestamp_ms = osKernelGetTickCount();
	pressr_reading.pressr_adc = raw_adc;
	// TODO: will need to compute the actual pressure using the curve fit from MATLAB
	pressr_reading.pressr = 0.0;

	// printf("Pressure Sensor ADC Input: %ld\n", raw_adc);
	osMessageQueuePut(pressrToLogHandle, &pressr_reading, 0, PRESSR_QUEUE_PUT_TIMEOUT_MS);

#if PRESSR_USE_40HZ
	osDelay(25);
#else
    osDelay(100); // 10 Hz update rate
#endif
	}

	osThreadTerminate(NULL);
}
