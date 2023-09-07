/*
 * imu_inertial.h
 *
 *  Created on: Aug 18, 2023
 *      Author: samuel
 */

#ifndef IMU_INERTIAL_H_
#define IMU_INERTIAL_H_

#include "stdbool.h"
#include "arm_math.h"

// I2C Addresses

#define SAO_DRIVEN_LOW 0

#if SAO_DRIVEN_LOW
#define INERTIAL_I2C_ADDR 0x6A
#define MAG_I2C_ADDR 0x1C
#else
#define INERTIAL_I2C_ADDR 0x6B
#define MAG_I2C_ADDR 0x1E
#endif

#define IMU_I2C_TIMEOUT_MS 30

// Inertial registers

#define INERTIAL_FUNC_CFG_ACCESS 0x01
#define INERTIAL_FIFO_CTRL1 0x06
#define INERTIAL_FIFO_CTRL2 0x07
#define INERTIAL_FIFO_CTRL3 0x08
#define INERTIAL_FIFO_CTRL4 0x09
#define INERTIAL_FIFO_CTRL5 0x0A
#define INERTIAL_ORIENT_CFG_G 0x0B
#define INERTIAL_INT1_CTRL 0x0D
#define INERTIAL_INT2_CTRL 0x0E
#define INERTIAL_WHO_AM_I 0x0F
#define INERTIAL_CTRL1_XL 0x10
#define INERTIAL_CTRL2_G 0x11
#define INERTIAL_CTRL3_C 0x12
#define INERTIAL_CTRL4_C 0x13
#define INERTIAL_CTRL5_C 0x14
#define INERTIAL_CTRL6_C 0x15
#define INERTIAL_CTRL7_G 0x16
#define INERTIAL_CTRL8_XL 0x17
#define INERTIAL_CTRL9_XL 0x18
#define INERTIAL_CTRL10_C 0x19
#define INERTIAL_WAKE_UP_SRC 0x1B
#define INERTIAL_TAP_SRC 0x1C
#define INERTIAL_D6D_SRC 0x1D
#define INERTIAL_STATUS_REG 0x1E
#define INERTIAL_OUT_TEMP_L 0x20
#define INERTIAL_OUT_TEMP_H 0x21
#define INERTIAL_OUTX_L_G 0x22
#define INERTIAL_OUTX_H_G 0x23
#define INERTIAL_OUTY_L_G 0x24
#define INERTIAL_OUTY_H_G 0x25
#define INERTIAL_OUTZ_L_G 0x26
#define INERTIAL_OUTZ_H_G 0x27
#define INERTIAL_OUTX_L_XL 0x28
#define INERTIAL_OUTX_H_XL 0x29
#define INERTIAL_OUTY_L_XL 0x2A
#define INERTIAL_OUTY_H_XL 0x2B
#define INERTIAL_OUTZ_L_XL 0x2C
#define INERTIAL_OUTZ_H_XL 0x2D
#define INERTIAL_FIFO_STATUS1 0x3A
#define INERTIAL_FIFO_STATUS2 0x3B
#define INERTIAL_FIFO_STATUS3 0x3C
#define INERTIAL_FIFO_STATUS4 0x3D
#define INERTIAL_FIFO_DATA_OUT_L 0x3E
#define INERTIAL_FIFO_DATA_OUT_H 0x3F
#define INERTIAL_TIMESTAMP0_REG 0x40
#define INERTIAL_TIMESTAMP1_REG 0x41
#define INERTIAL_TIMESTAMP2_REG 0x42
#define INERTIAL_STEP_TIMESTAMP_L 0x49
#define INERTIAL_STEP_TIMESTAMP_H 0x4A
#define INERTIAL_STEP_COUNTER_L 0x4B
#define INERTIAL_STEP_COUNTER_H 0x4C
#define INERTIAL_FUNC_SRC 0x53
#define INERTIAL_TAP_CFG 0x58
#define INERTIAL_TAP_THS_6D 0x59
#define INERTIAL_INT_DUR2 0x5A
#define INERTIAL_WAKE_UP_THS 0x5B
#define INERTIAL_WAKE_UP_DUR 0x5C
#define INERTIAL_FREE_FALL 0x5D
#define INERTIAL_MD1_CFG 0x5E
#define INERTIAL_MD2_CFG 0x5F



// Magnetometer Registers

#define MAG_WHO_AM_I 0x0F
#define MAG_CTRL_REG1 0x20
#define MAG_CTRL_REG2 0x21
#define MAG_CTRL_REG3 0x22
#define MAG_CTRL_REG4 0x23
#define MAG_CTRL_REG5 0x24
#define MAG_STATUS_REG 0x27
#define MAG_OUT_X_L 0x28
#define MAG_OUT_X_H 0x29
#define MAG_OUT_Y_L 0x2A
#define MAG_OUT_Y_H 0X2B
#define MAG_OUT_Z_L 0X2C
#define MAG_OUT_Z_H 0X2D
#define MAG_TEMP_OUT_L 0X2E
#define MAG_TEMP_OUT_H 0X2D
#define MAG_INT_CFG 0x30
#define MAG_INT_SRC 0x31
#define MAG_INT_THS_L 0x32
#define MAG_INT_THS_H 0x33

// Magnetometer

typedef enum {
    MAG_LOW_POWER = 0x00,
    MAG_MEDIUM_POWER = 0x01,
    MAG_HIGH_POWER = 0x02,
    MAG_ULTRA_HIGH_POWER = 0x03
} mag_axis_mode_t;

typedef enum {
    MAG_0_625_HZ = 0x00,
    MAG_1_25_HZ = 0x01,
    MAG_2_5_HZ = 0x02,
    MAG_5_HZ = 0x03,
    MAG_10_HZ = 0x04,
    MAG_20_HZ = 0x05,
    MAG_40_HZ = 0x06,
    MAG_80_HZ = 0x07
} mag_odr_t;

typedef enum {
    MAG_FS_4_GAUSS = 0x00,
    MAG_FS_8_GAUSS = 0x01,
    MAG_FS_12_GAUSS = 0x02,
    MAG_FS_16_GAUSS = 0x03
} mag_fs_rng_t;

typedef enum {
    MAG_CONTINUOUS = 0x00,
    MAG_SINGLE_CONVERSION = 0x01,
    MAG_POWER_DOWN = 0x02
} mag_sys_mode_t;

/*
 * The data output rate is ignored if the fast output data rate is enabled. In that
 * case the rate is determined by the operating mode.
 *
 * Low power -> 1000 Hz
 * Medium power -> 560 Hz
 * High power -> 300 Hz
 * Ultra high power -> 155 Hz
 */
typedef struct {
    bool enable_temp_sens;
    bool block_data_update;
    mag_sys_mode_t sys_mode;

    mag_axis_mode_t xy_om;
    mag_axis_mode_t z_om;

    mag_odr_t odr;
    bool fast_odr;

    mag_fs_rng_t fs_rng;
} mag_config_t;

// Inertial

//typedef struct {
//    bool ang_x_sgn;
//    bool ang_y_sgn;
//    bool ang_z_sgn;
//    enum {
//        ORIENT_XYZ = 0x00,
//        ORIENT_XZY = 0x01,
//        ORIENT_YXZ = 0x02,
//        ORIENT_YZX = 0X03,
//        ORIENT_ZXY = 0x04,
//        ORIENT_ZYX = 0x05
//    } orient_dir;
//} orient_cfg_t;

typedef enum {
    ACCEL_POWER_DOWN=0x00,
    ACCEL_13_HZ=0x01, // Low or high
    ACCEL_26_HZ=0X02, // Low or high
    ACCEL_52_HZ=0X03, // Low or high
    ACCEL_104_HZ=0X04, // Normal or high
    ACCEL_208_HZ=0X05, // Normal or high
    ACCEL_416_HZ=0X06, // Only high
    ACCEL_833_HZ=0X07, // Only high
    ACCEL_1_66K_HZ=0X08, // Only high
    ACCEL_3_33K_HZ=0x09, // Only high
    ACCEL_6_66K_HZ=0x0A // Only high
} accel_odr_t;

typedef enum {
    ACCEL_FS_2_G=0x00,
    ACCEL_FS_4_G=0x02,
    ACCEL_FS_8_G=0x03,
    ACCEL_FS_16_G=0x01,
} accel_fs_rng_t;

typedef enum {
    GYRO_POWER_DOWN=0x00,
    GYRO_13_HZ=0x01, // Low or high
    GYRO_26_HZ=0x02, // Low or high
    GYRO_52_HZ=0x03, // Low or high
    GYRO_104_HZ=0x04, // Normal or high
    GYRO_208_HZ=0x05, // Normal or high
    GYRO_416_HZ=0x06, // Only high
    GYRO_833_HZ=0x07, // Only high
    GYRO_1_66K_HZ=0x08 // Only high
} gyro_odr_t;

typedef enum {
    GYRO_FS_125_DPS=0x0F, // Arbitrary value as will set bit to override full-scale selection
    GYRO_FS_245_DPS=0x00,
    GYRO_FS_500_DPS=0x01,
    GYRO_FS_1000_DPS=0x02,
    GYRO_FS_2000_DPS=0x03
} gyro_fs_rng_t;

typedef struct {
    bool use_gyro_high_power;
    bool use_accel_high_power;

    bool block_data_update;

    // orient_cfg_t orient_cfg;

    accel_odr_t accel_odr;
    accel_fs_rng_t accel_fs_rng;

    gyro_odr_t gyro_odr;
    gyro_fs_rng_t gyro_fs_rng;
} inertial_cfg_t;

typedef float32_t imu_axis_t;

inline void verify_imu_comms();

inline bool set_mag_ctrl_regs(mag_config_t* cfg);
inline bool check_if_mag_avail();
inline bool read_mag_axes(imu_axis_t* axes, int16_t* adc_axes, mag_config_t *cfg);

inline bool set_inertial_ctrl_regs(inertial_cfg_t* cfg);
inline bool check_if_inertial_avail();
inline bool read_inertial_axes(imu_axis_t* accel_axes, imu_axis_t* gyro_axes, int16_t* accel_adc_axes, int16_t* gyro_adc_axes, inertial_cfg_t* cfg);

#endif /* IMU_INERTIAL_H_ */
