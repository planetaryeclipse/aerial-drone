/*
 * radio.c
 *
 *  Created on: Aug 15, 2023
 *      Author: samuel
 */

#include "usart.h"
#include "main.h"
#include "cmsis_os.h"

#include "string.h"
#include "printf/printf.h"

#include "comms_types.h"
#include "radio.h"


#include "stdbool.h"

extern UART_HandleTypeDef huart1;

#define RADIO_RECV_BUFFER_LEN 128
#define RADIO_SEND_BUFFER_LEN 128

uint8_t recv_data[RADIO_RECV_BUFFER_LEN] = {0};
uint8_t send_data[RADIO_SEND_BUFFER_LEN] = {0};

extern osMessageQueueId_t actCmdsHandle;

#define RECV_MSG_SIZE (4+sizeof(float)*9+2)

uint8_t msg_parse_buf[RECV_MSG_SIZE] = {0};
unsigned int pos_in_msg = 0;

void ParseRecvMsg(){
    // First validates the packet
    bool header_present = msg_parse_buf[0] == 'S' && msg_parse_buf[1] == 'C' && msg_parse_buf[2] == 'M' && msg_parse_buf[3];
    bool terminator_present = msg_parse_buf[RECV_MSG_SIZE-2] == '\r' && msg_parse_buf[RECV_MSG_SIZE-1] == '\n';

    if (!header_present || !terminator_present)
        return; // Packet is invalid

//    printf("Received command bytes: ");
//    for(int i = 0; i < RECV_MSG_SIZE; i++){
//        printf("%02x ", msg_parse_buf[i]);
//    }
//    printf("\n");

    float* pkt_data = (float*)(msg_parse_buf+4);

    actuator_t act_cmd;
    act_cmd.timestamp_ms = osKernelGetSysTimerCount();
    act_cmd.left_thruster_power = pkt_data[0];
    act_cmd.right_thruster_power = pkt_data[1];
    act_cmd.left_flap_ang = pkt_data[2];
    act_cmd.left_aileron_ang = pkt_data[3];
    act_cmd.right_flap_ang = pkt_data[4];
    act_cmd.right_aileron_ang = pkt_data[5];
    act_cmd.left_elevator_ang = pkt_data[6];
    act_cmd.right_elevator_ang = pkt_data[7];
    act_cmd.rudder_ang = pkt_data[8];

    // printf("Received command: lthrust=%.03f, rthrust=%.03f, left_flap=%.03f, left_aileron=%.03f, right_flap=%.03f, right_aileron=%.03f, left_elev=%f, right_elev=%.03f, rudder=%.03f\n", act_cmd.left_thruster_power, act_cmd.right_thruster_power, act_cmd.left_flap_ang, act_cmd.left_aileron_ang, act_cmd.right_flap_ang, act_cmd.right_aileron_ang, act_cmd.left_elevator_ang, act_cmd.right_elevator_ang, act_cmd.rudder_ang);

    osMessageQueuePut(actCmdsHandle, &act_cmd, 0, 0);
}

bool handling_radio_rx_callback = false;

void RadioRxCpltCallback(){
    // printf("In radio receive callback!\n");

    for(int i = 0;i < RADIO_RECV_BUFFER_LEN; i++){
        if (recv_data[i] == 'S' && pos_in_msg == 0){
            // Presumed start of message, start copying into message buffer if not already
            msg_parse_buf[pos_in_msg++] = recv_data[i];
            // printf("Found start of message at idx %i\n", i);
        } else if (recv_data[i] == '\n' && pos_in_msg > 0) {
            if (pos_in_msg == RECV_MSG_SIZE-1){
                // Proper size message detected
                msg_parse_buf[pos_in_msg++] = recv_data[i];
                // printf("Found end of message at idx %i\n", i);

                ParseRecvMsg();
                pos_in_msg = 0;
            } else{
                // Early end of message detected, throw out data
                pos_in_msg = 0;
                // printf("Early end of message detected at idx %i, throwing out data\n", i);
            }
        } else if (pos_in_msg > 0){
            // Only copy characters into buffer if the message has started to be copied
            // into the buffer already (given by pos_in_msg being greater than 0)
            if (pos_in_msg > 0 && pos_in_msg < RECV_MSG_SIZE){
                msg_parse_buf[pos_in_msg++] = recv_data[i];
            } else if (pos_in_msg == RECV_MSG_SIZE){
                // Reached the end of the expected message and it has not seemed to
                // terminate so clear the buffer and only read the next message
                pos_in_msg = 0;
                // printf("Received bytes have exceeded message size, throwing out data\n");
            }

        }
    }

    // Resets the receive buffer
    memset(recv_data, 0, RADIO_RECV_BUFFER_LEN);
}

void RadioTxCpltCallback() {}

/**
 * @brief Facilitates communication with the ground segment via the telemetry radio
 * @param argument: Not used
 * @retval None
 */
void RadioManager(void *argument){
    // Baud rate is 57600


    for(;;)   {
        HAL_UART_Receive_IT(&huart1, recv_data, RADIO_RECV_BUFFER_LEN); // RECV_MSG_SIZE);

        // Prepares a test message to send over the radio connection
        // Test
//        uint8_t test_buf[8] = {0};
//        HAL_UART_Receive(&huart1, test_buf, 8, 10);
//        for(int i = 0; i < 8; i++)
//            printf("%02x ", test_buf[i]);
//        printf("\n");

        // printf("Radio mngr thread runs silently!\n");

        osDelay(10);

        // osThreadYield();
    }

    osThreadTerminate(NULL);
}
