/*
 * open_logger.c
 *
 *  Created on: Aug 13, 2023
 *      Author: samuel
 */

#include "usart.h"
#include "main.h"
#include "cmsis_os.h"

#include "printf/printf.h"
//#include "stdio.h"
//#include "stdlib.h"
//#include "string.h"

#include "sens_types.h"

extern osMessageQueueId_t logPressrHandle;
extern UART_HandleTypeDef huart1;

#define SENS_QUEUE_TIMEOUT_MS 5

#define OPENLOGGER_WRITE_BUFFER_LEN 128
#define OPENLOGGER_WRITE_TIMEOUT_MS 20

void LoggerRxCpltCallback() {}
void LoggerTxCpltCallback() {}

/**
 * @brief Reads sensor values and writes to the SparkFun OpenLogger
 * @param argument: Not used
 * @retval None
 */
//void WriteLogger(void *argument){
//    // Sets the OPENLOGGER_DTR pin to enable to the OpenLogger before UART transmission
//    HAL_GPIO_WritePin(OPENLOGGER_DTR_GPIO_Port, OPENLOGGER_DTR_Pin, GPIO_PIN_SET);
//
//    osStatus_t queue_status = osOK;
//    char buf[OPENLOGGER_WRITE_BUFFER_LEN] = {0};
//
//    for(;;)
//    {
//        sens_pressr_t pressr_reading = {0};
//        queue_status = osMessageQueueGet(logPressrHandle, &pressr_reading, NULL, SENS_QUEUE_TIMEOUT_MS);
//        if (queue_status == osOK){
//            int len = snprintf_(buf, OPENLOGGER_WRITE_BUFFER_LEN, "sens=pressr,timestamp_ms=%ld,adc_val=%ld,pressr=%f\n",
//                    pressr_reading.timestamp_ms, pressr_reading.adc_val, pressr_reading.pressr);
//
//            // printf("Pressure sensor log: %s\n", buf);
//            HAL_UART_Transmit(&huart1, (uint8_t*)buf, (uint16_t)len, OPENLOGGER_WRITE_TIMEOUT_MS);
//        }
//
//
//        osDelay(50);
//    }
//
//    osThreadTerminate(NULL);
//}
