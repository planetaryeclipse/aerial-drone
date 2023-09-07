/*
 * sensor_types.h
 *
 *  Created on: Aug 14, 2023
 *      Author: samuel
 */

#ifndef INC_DRONE_SENS_TYPES_H_
#define INC_DRONE_SENS_TYPES_H_

typedef struct {
    uint32_t timestamp_ms;

    float pressr;
    uint32_t pressr_adc;

    float calc_airspeed; // Computed property
} sens_pressr_t;

typedef struct {
    uint32_t timestamp_ms;

    float accel[3];
    float ang_vel[3];
    float mag[3];

    int16_t accel_adc[3];
    int16_t ang_vel_adc[3];
    int16_t mag_adc[3];

    // Computed property
    float world_accel[3];
    float world_ang_vel[3];
    float world_mag[3];
} sens_imu_t;

typedef struct {
    uint32_t timestamp_ms;
    float latitude, longitude, altitude;
} gps_t;

typedef struct {
    uint32_t timestamp_ms;
    float rng;
} sens_rng_t;

#endif /* INC_DRONE_SENS_TYPES_H_ */
