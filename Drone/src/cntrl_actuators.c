/*
 * cntrl_actuators.c
 *
 *  Created on: Aug 29, 2023
 *      Author: samuel
 */

#include "cmsis_os.h"
#include "stdbool.h"

#include "tim.h"

#include "printf/printf.h"

#include "comms_types.h"


// Servo limits and calibration

#define WING_SERVO_IDEAL_MIN_CCX 50
#define WING_SERVO_IDEAL_MAX_CCX 100
#define WING_SERVO_IDEAL_NEUTRAL_CCX (WING_SERVO_IDEAL_MIN_CCX + WING_SERVO_IDEAL_MAX_CCX)/2
#define WING_SERVO_IDEAL_RANGE_CCX (WING_SERVO_IDEAL_MAX_CCX - WING_SERVO_IDEAL_MIN_CCX)
#define WING_SERVO_MAX_ANG 45

#define TAIL_SERVO_IDEAL_MIN_CCX 25
#define TAIL_SERVO_IDEAL_MAX_CCX 125
#define TAIL_SERVO_IDEAL_NEUTRAL_CCX (TAIL_SERVO_IDEAL_MIN_CCX + TAIL_SERVO_IDEAL_MAX_CCX)/2
#define TAIL_SERVO_IDEAL_RANGE_CCX (TAIL_SERVO_IDEAL_MAX_CCX - TAIL_SERVO_IDEAL_MIN_CCX)
#define TAIL_SERVO_MAX_ANG 90

#define WING_SERVO_1_ZERO_BIAS_CCX -2
#define WING_SERVO_2_ZERO_BIAS_CCX -3
#define WING_SERVO_3_ZERO_BIAS_CCX 1
#define WING_SERVO_4_ZERO_BIAS_CCX 5

#define TAIL_SERVO_1_ZERO_BIAS_CCX 3
#define TAIL_SERVO_2_ZERO_BIAS_CCX -5
#define TAIL_SERVO_3_ZERO_BIAS_CCX 2

// Calibration modes

#define CALIBRATION_MODE 0
#define CALIBRATE_SERVO_LIMITS 0
#define CALIBRATE_ESC_LIMITS 0

// Control purposes

#define WING_SERVO_1_FLIPPED false
#define WING_SERVO_2_FLIPPED false
#define WING_SERVO_3_FLIPPED false
#define WING_SERVO_4_FLIPPED false

#define TAIL_SERVO_1_FLIPPED false
#define TAIL_SERVO_2_FLIPPED false
#define TAIL_SERVO_3_FLIPPED false

extern osMessageQueueId_t actCmdsHandle;

void ControlActuators(){

#if CALIBRATION_MODE && !CALIBRATE_SERVO_LIMITS
    // Allow modification of the zero angle bias to identify true 0 degrees

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, WING_SERVO_IDEAL_NEUTRAL_CCX + (WING_SERVO_1_ZERO_BIAS_CCX));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, WING_SERVO_IDEAL_NEUTRAL_CCX + (WING_SERVO_2_ZERO_BIAS_CCX));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, WING_SERVO_IDEAL_NEUTRAL_CCX + (WING_SERVO_3_ZERO_BIAS_CCX));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, WING_SERVO_IDEAL_NEUTRAL_CCX + (WING_SERVO_4_ZERO_BIAS_CCX));

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, TAIL_SERVO_IDEAL_NEUTRAL_CCX + (TAIL_SERVO_1_ZERO_BIAS_CCX));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, TAIL_SERVO_IDEAL_NEUTRAL_CCX + (TAIL_SERVO_2_ZERO_BIAS_CCX));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, TAIL_SERVO_IDEAL_NEUTRAL_CCX + (TAIL_SERVO_3_ZERO_BIAS_CCX));
#elif CALIBRATION_MODE && CALIBRATE_SERVO_LIMITS
    bool increasing = true;

    for(;;){
        uint8_t wing_servo_val, tail_servo_val;
        if (increasing){
            wing_servo_val = WING_SERVO_IDEAL_MAX_CCX;
            tail_servo_val = TAIL_SERVO_IDEAL_MAX_CCX;
            increasing = false;
        } else{
            wing_servo_val = WING_SERVO_IDEAL_MIN_CCX;
            tail_servo_val = TAIL_SERVO_IDEAL_MIN_CCX;
            increasing = true;
        }

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, wing_servo_val + (WING_SERVO_1_ZERO_BIAS_CCX));
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, wing_servo_val + (WING_SERVO_2_ZERO_BIAS_CCX));
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, wing_servo_val + (WING_SERVO_3_ZERO_BIAS_CCX));
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, wing_servo_val + (WING_SERVO_4_ZERO_BIAS_CCX));

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, tail_servo_val + (TAIL_SERVO_1_ZERO_BIAS_CCX));
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, tail_servo_val + (TAIL_SERVO_2_ZERO_BIAS_CCX));
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, tail_servo_val + (TAIL_SERVO_3_ZERO_BIAS_CCX));

        osDelay(1000);
    }
#else
    // Use standard control scheme
    for(;;){

        // Pulls all the available actuator commands so as to clear the queue
        actuator_t act_cmd;
        uint32_t num_waiting_cmds = osMessageQueueGetCount(actCmdsHandle);

        for(int i = 0; i < num_waiting_cmds; i++)
            osMessageQueueGet(actCmdsHandle, &act_cmd, 0, osWaitForever);

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (WING_SERVO_1_FLIPPED? -1 : 1) * (act_cmd.left_flap_ang/WING_SERVO_MAX_ANG * (WING_SERVO_IDEAL_RANGE_CCX/2)) + WING_SERVO_IDEAL_NEUTRAL_CCX + (WING_SERVO_1_ZERO_BIAS_CCX));
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (WING_SERVO_2_FLIPPED? -1 : 1) * (act_cmd.left_aileron_ang/WING_SERVO_MAX_ANG * (WING_SERVO_IDEAL_RANGE_CCX/2)) + WING_SERVO_IDEAL_NEUTRAL_CCX + (WING_SERVO_2_ZERO_BIAS_CCX));
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (WING_SERVO_3_FLIPPED? -1 : 1) * (act_cmd.right_flap_ang/WING_SERVO_MAX_ANG * (WING_SERVO_IDEAL_RANGE_CCX/2)) + WING_SERVO_IDEAL_NEUTRAL_CCX + (WING_SERVO_3_ZERO_BIAS_CCX));
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (WING_SERVO_4_FLIPPED? -1 : 1) * (act_cmd.right_aileron_ang/WING_SERVO_MAX_ANG * (WING_SERVO_IDEAL_RANGE_CCX/2)) + WING_SERVO_IDEAL_NEUTRAL_CCX + (WING_SERVO_4_ZERO_BIAS_CCX));

        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (TAIL_SERVO_1_FLIPPED? -1 : 1) * (act_cmd.left_elevator_ang/TAIL_SERVO_MAX_ANG * (TAIL_SERVO_IDEAL_RANGE_CCX/2)) + TAIL_SERVO_IDEAL_NEUTRAL_CCX + (TAIL_SERVO_1_ZERO_BIAS_CCX));
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (TAIL_SERVO_2_FLIPPED? -1 : 1) * (act_cmd.right_elevator_ang/TAIL_SERVO_MAX_ANG * (TAIL_SERVO_IDEAL_RANGE_CCX/2)) + TAIL_SERVO_IDEAL_NEUTRAL_CCX + (TAIL_SERVO_2_ZERO_BIAS_CCX));
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (TAIL_SERVO_3_FLIPPED? -1 : 1) * (act_cmd.rudder_ang/TAIL_SERVO_MAX_ANG * (TAIL_SERVO_IDEAL_RANGE_CCX/2)) + TAIL_SERVO_IDEAL_NEUTRAL_CCX + (TAIL_SERVO_3_ZERO_BIAS_CCX));




        //        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, wing_servo_val + (WING_SERVO_2_ZERO_BIAS_CCX));
//        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, wing_servo_val + (WING_SERVO_3_ZERO_BIAS_CCX));
//        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, wing_servo_val + (WING_SERVO_4_ZERO_BIAS_CCX));
//
//        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, tail_servo_val + (TAIL_SERVO_1_ZERO_BIAS_CCX));
//        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, tail_servo_val + (TAIL_SERVO_2_ZERO_BIAS_CCX));
//        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, tail_servo_val + (TAIL_SERVO_3_ZERO_BIAS_CCX));

        // osDelay(50); // Run at 20 Hz
    }
#endif

//    bool increasing = true;
//
//    for(;;){
//
//        uint8_t small_motor_val = 0;
//        uint8_t large_motor_val = 0;
//        if (increasing){
//            small_motor_val = small_motor_max;
//            large_motor_val = large_motor_max;
//            increasing = false;
//        } else{
//            small_motor_val = small_motor_min;
//            large_motor_val = large_motor_min;
//            increasing = true;
//        }
//
//        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, large_motor_val);
//        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, large_motor_val);
//        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, large_motor_val);
//        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, large_motor_val);
//
//        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, small_motor_val);
//        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, small_motor_val);
//        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, small_motor_val);
//
//        osDelay(1000);
//    }


    // For testing: note that the while pulse width band is not needed, all that is
    // important is having a pulse of 0 to 2 milliseconds which corresponds to 1 and
    // 2 ms precisely

//    bool increasing = true;
//    uint8_t val = 50; // 1 ms -> 0 degrees
//
//    // should be 100 -> 2 ms of 180 and 50 -> 1 ms for 0 degrees
//    // for some reason get full 180 degrees if 25 to 125 values are used
//
//    uint8_t max = 125; // 100;
//    uint8_t min = 25; // 50;
//    uint8_t step = 5;
//
//    val = min;
//
//    for(;;){
//        if(val == min)
//            val = max;
//        else if (val == max)
//            val = min;
//
//        printf("servo value: %i\n", val);
//
////        uint32_t counter_max = 10; // corresponds to 2 ms
////        uint32_t duty_cycle_val = (uint32_t)((float)duty_cycle * 0.01 * counter_max);
//        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, val);

//        printf("counter_max=%i, duty_cycle=%i, duty_cyc_val=%i\n", counter_max, duty_cycle, duty_cycle_val);
//
//        if (increasing){
//            duty_cycle++;
//            if (duty_cycle == 100)
//                increasing = false;
//        } else{
//            duty_cycle--;
//            if (duty_cycle == 0)
//                increasing = true;
//        }

//        osDelay(1000);
//    }

    osThreadTerminate(NULL);
}
