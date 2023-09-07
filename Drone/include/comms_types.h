/*
 * comms_types.h
 *
 *  Created on: Aug 27, 2023
 *      Author: samuel
 */

#ifndef COMMS_TYPES_H_
#define COMMS_TYPES_H_

#include "stdbool.h"

typedef struct {
    uint32_t timestamp_ms;

    // Thruster power (percentage of max power)
    float left_thruster_power, right_thruster_power;

    // Wing control surfaces (degrees)
    float left_flap_ang, left_aileron_ang;
    float right_flap_ang, right_aileron_ang;

    // Tail control surfaces (degrees)
    float left_elevator_ang, right_elevator_ang, rudder_ang;
} actuator_t;

typedef struct {
    unsigned long timestamp_ms;

    bool pressure_present;
    float pressure, airspeed;
    unsigned int pressure_adc;

    bool gps_present;
    float latitutde, longitude, altitude;

    bool imu_present;
    float world_accel[3];
    float world_ang_vel[3];
    float world_mag[3];

    float accel[3];
    float ang_vel[3];
    float mag[3];

    bool range_1_present;
    float range_1;

    bool range_2_present;
    float range_2;
} telemetry_t;

#endif /* COMMS_TYPES_H_ */
