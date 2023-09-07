/*
 * imu.c
 *
 *  Created on: Aug 16, 2023
 *      Author: samuel
 */

#include "cmsis_os2.h"
#include "imu.h"
#include "main.h"
#include "stdbool.h"
#include "printf/printf.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_i2c.h"
#include "string.h"
#include "sys/_stdint.h"
#include "arm_math.h"

#define IMU_PWR_TOGGLE_WAIT_MS 25

extern I2C_HandleTypeDef hi2c1;

void verify_imu_comms() {
    HAL_StatusTypeDef inertial_status = HAL_I2C_IsDeviceReady(&hi2c1,
            INERTIAL_I2C_ADDR << 1, 1, IMU_I2C_TIMEOUT_MS);
    HAL_StatusTypeDef mag_status = HAL_I2C_IsDeviceReady(&hi2c1, MAG_I2C_ADDR,
            1, IMU_I2C_TIMEOUT_MS);
    if (inertial_status != HAL_OK || mag_status != HAL_OK) {
        printf("IMU is not ready, power cycling...\n");
        HAL_GPIO_WritePin(IMU_PWR_GPIO_Port, IMU_PWR_Pin, GPIO_PIN_RESET);
        HAL_Delay(IMU_PWR_TOGGLE_WAIT_MS);
        HAL_GPIO_WritePin(IMU_PWR_GPIO_Port, IMU_PWR_Pin, GPIO_PIN_SET);
        HAL_Delay(IMU_PWR_TOGGLE_WAIT_MS);
        printf("IMU power cycle complete!\n");
    }
}

bool set_mag_ctrl_regs(mag_config_t *cfg) {
    /*
     * CTRL_REG1
     * Sets temperature enable/disable of temperature sensor, operation mode of x and y axes, and the output
     * data rate of the sensor (also allows setup of the fast output data rate) and disables self test mode (bit 0)
     *
     * CTRL_REG2
     * Sets the full range limits on the gauss reading of each axis and disables reboot and soft reset
     *
     * CTRL_REG3
     * Disables low power mode, sets 3-wire SPI interface, and sets the system operating mode
     *
     * CTRL_REG4
     * Sets the operating mode of the z axis and sets little endian
     *
     * CTRL_REG5
     * Disables fast read (reading only high part of data out to increase reading efficiency) and sets the block
     * data update mode to only update the data out registers once the Msb and Lsb bits have been read
     */

    uint8_t buf[5];
    buf[0] = (cfg->enable_temp_sens << 7) | (cfg->xy_om << 5) | (cfg->odr << 2)
            | (cfg->fast_odr << 1);
    buf[1] = (cfg->fs_rng << 5);
    buf[2] = cfg->sys_mode;
    buf[3] = (cfg->z_om << 2);
    buf[4] = (cfg->block_data_update << 6);

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, MAG_I2C_ADDR << 1,
            MAG_CTRL_REG1, I2C_MEMADD_SIZE_8BIT, buf, 5, IMU_I2C_TIMEOUT_MS);
    return status == HAL_OK;
}

bool check_if_mag_avail() {
    uint8_t sens_status = 0;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, MAG_I2C_ADDR << 1,
            MAG_STATUS_REG, I2C_MEMADD_SIZE_8BIT, &sens_status, 1,
            IMU_I2C_TIMEOUT_MS);
    if (status == HAL_OK) {
        return (sens_status & 0x08) >> 3; // Checks ZYXDA (whether all axes have new data)
    }
    return false; // Default of no data available if read fails
}

bool read_mag_axes(imu_axis_t *axes, int16_t *adc_axes, mag_config_t *cfg) {
    uint8_t buf[6] = { 0 };
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, MAG_I2C_ADDR << 1,
            MAG_OUT_X_L, I2C_MEMADD_SIZE_8BIT, buf, 6, IMU_I2C_TIMEOUT_MS);
    if (status == HAL_OK) {
        uint16_t *adc_axes_bytes = (uint16_t*) adc_axes;
        adc_axes_bytes[0] = (buf[1] << 8) | buf[0]; // OUT_X_H, OUT_X_L
        adc_axes_bytes[1] = (buf[3] << 8) | buf[2]; // OUT_Y_H, OUT_Y_L
        adc_axes_bytes[2] = (buf[5] << 8) | buf[4]; // OUT_Z_H, OUT_Z_L

        imu_axis_t lsb_per_gauss;
        switch (cfg->fs_rng) {
            case MAG_FS_4_GAUSS:
                lsb_per_gauss = 6842;
                break;
            case MAG_FS_8_GAUSS:
                lsb_per_gauss = 3421;
                break;
            case MAG_FS_12_GAUSS:
                lsb_per_gauss = 2281;
                break;
            case MAG_FS_16_GAUSS:
                lsb_per_gauss = 1711;
                break;
            default:
                return false; // Unrecognized full-scale range
        }

        // Computes the float values
        axes[0] = adc_axes[0] / lsb_per_gauss;
        axes[1] = adc_axes[1] / lsb_per_gauss;
        axes[2] = adc_axes[2] / lsb_per_gauss;

        return true;
    }
    return false; // Failure to read
}

bool set_inertial_ctrl_regs(inertial_cfg_t *cfg) {
    /*
     * CTRL1_XL
     * Sets accelerometer output data rate/power mode selection, full-scale selection,
     * and defaults to an anti-aliasing filter with BW of 400 Hz (although this selection
     * will not be used as the bit XL_BW_SACL_ODR will be set to 0 thereby auto setting this value) 
     * 
     * CTRL2_G
     * Sets gyroscope output data rate/power mode selection and full-scale selection.
     * 
     * CTRL3_C
     * Sets block data update, keeps interfaces and memory resets set at default values. The value
     * of IF_INC is kept at 1 to ensure multi-byte access through I2C is allowed.
     * 
     * CTRL4_C
     * Keeps default values and ensures accelerometer bandwidth selection determined by the output
     * data rate of the accelerometer. The I2C interface is also not disabled.
     * 
     * CTRL5_C
     * Disables circular burst-mode (rounding) read from output registers. Angular rates and linear
     * acceleration sensor self-tests are disabled.
     * 
     * CTRL6_C
     * Sets the triggers of the gyroscope to default values and enables/disables the high-performance
     * mode of the accelerometer.
     * 
     * CTRL7_G
     * Enables/disables the high-performance mode of the gyroscope and disables gyroscope filters. Note
     * that the high-pass filter cutoff frequency is set to the default of 0.0081 Hz but is unused as the
     * filter is disabled. Also disables the rounding on STATUS_REG, FUNC_SRC, and WAKE_UP_SRC registers.
     */

    uint8_t buf[7];
    buf[0] = (cfg->accel_odr << 4) | (cfg->accel_fs_rng << 2);
    if (cfg->gyro_fs_rng == GYRO_FS_125_DPS) {
        // Sets the bit to force the gyroscope full-scale to 125 dps
        buf[1] = (cfg->gyro_odr << 4) | 0x02;
    } else {
        buf[1] = (cfg->gyro_odr << 4) | (cfg->gyro_fs_rng << 2);
    }
    buf[2] = (cfg->block_data_update << 6) | 0x04;
    buf[3] = 0x00;
    buf[4] = 0x00;
    buf[5] = cfg->use_accel_high_power << 4;
    buf[6] = cfg->use_gyro_high_power << 7;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c1, INERTIAL_I2C_ADDR << 1,
            INERTIAL_CTRL1_XL, I2C_MEMADD_SIZE_8BIT, buf, 7,
            IMU_I2C_TIMEOUT_MS);
    return status == HAL_OK;
}

bool check_if_inertial_avail() {
    uint8_t sens_status = 0;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, INERTIAL_I2C_ADDR << 1,
            INERTIAL_STATUS_REG, I2C_MEMADD_SIZE_8BIT, &sens_status, 1,
            IMU_I2C_TIMEOUT_MS);
    if (status == HAL_OK) {
        // Ensures that both gyroscope and accelerometer data is available
        return ((sens_status & 0x02) >> 1) & (sens_status & 0x01);
    }
    return false; // Default of no data available if read fails
}

bool read_inertial_axes(imu_axis_t *accel_axes, imu_axis_t *gyro_axes,
        int16_t *accel_adc_axes, int16_t *gyro_adc_axes, inertial_cfg_t *cfg) {
    uint8_t buf[12] = { 0 };
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, INERTIAL_I2C_ADDR << 1,
            INERTIAL_OUTX_L_G, I2C_MEMADD_SIZE_8BIT, buf, 12,
            IMU_I2C_TIMEOUT_MS);
    if (status == HAL_OK) {
        uint16_t *accel_adc_axes_bytes = (uint16_t*) accel_adc_axes;
        uint16_t *gyro_adc_axes_bytes = (uint16_t*) gyro_adc_axes;

        gyro_adc_axes_bytes[0] = (buf[1] << 8) | buf[0];
        gyro_adc_axes_bytes[1] = (buf[3] << 8) | buf[2];
        gyro_adc_axes_bytes[2] = (buf[5] << 8) | buf[4];

        accel_adc_axes_bytes[0] = (buf[7] << 8) | buf[6];
        accel_adc_axes_bytes[1] = (buf[9] << 8) | buf[8];
        accel_adc_axes_bytes[2] = (buf[11] << 8) | buf[10];

        imu_axis_t accel_mg_per_lsb;
        switch (cfg->accel_fs_rng) {
            case ACCEL_FS_2_G:
                accel_mg_per_lsb = 0.061;
                break;
            case ACCEL_FS_4_G:
                accel_mg_per_lsb = 0.122;
                break;
            case ACCEL_FS_8_G:
                accel_mg_per_lsb = 0.244;
                break;
            case ACCEL_FS_16_G:
                accel_mg_per_lsb = 0.488;
                break;
            default:
                return false; // Unrecognized full-scale range
        }

        imu_axis_t gyro_mdps_per_lsb;
        switch (cfg->gyro_fs_rng) {
            case GYRO_FS_125_DPS:
                gyro_mdps_per_lsb = 4.375;
                break;
            case GYRO_FS_245_DPS:
                gyro_mdps_per_lsb = 8.75;
                break;
            case GYRO_FS_500_DPS:
                gyro_mdps_per_lsb = 17.50;
                break;
            case GYRO_FS_1000_DPS:
                gyro_mdps_per_lsb = 35;
                break;
            case GYRO_FS_2000_DPS:
                gyro_mdps_per_lsb = 70;
                break;
            default:
                return false; // Unrecognized full-scale range
        }

        // Computes the angular velocity (dps)
        gyro_axes[0] = gyro_adc_axes[0] * 0.001 * gyro_mdps_per_lsb;
        gyro_axes[1] = gyro_adc_axes[1] * 0.001 * gyro_mdps_per_lsb;
        gyro_axes[2] = gyro_adc_axes[2] * 0.001 * gyro_mdps_per_lsb;

        // Computes the linear acceleration (g)
        accel_axes[0] = accel_adc_axes[0] * 0.001 * accel_mg_per_lsb;
        accel_axes[1] = accel_adc_axes[1] * 0.001 * accel_mg_per_lsb;
        accel_axes[2] = accel_adc_axes[2] * 0.001 * accel_mg_per_lsb;

        return true;
    }
    return false; // Failure to read
}

void ReadMag(void *argument){
    verify_imu_comms(); // Should trigger the power to the test LED

    for(;;){
        osDelay(1000);
    }

    osThreadTerminate(NULL);
}

/**
 * @brief Reads magnetometer sensor values from the IMU
 * @param argument: Not used
 * @retval None
 */
//void ReadMag(void *argument){
//    if(HAL_I2C_IsDeviceReady(&hi2c1, MAG_I2C_ADDR << 1, 1, IMU_I2C_TIMEOUT_MS) != osOK){
//        // Due to debugging sometimes the I2C bus between the master and slave is messed up in
//        // which case communication fails. This will fix the communication issues.
//
//        for(int i = 0; i < 9; i++){
//            HAL_GPIO_WritePin(IMU_I2C_SCL_GPIO_Port, IMU_I2C_SCL_Pin, GPIO_PIN_SET);
//            HAL_Delay(2);
//            HAL_GPIO_WritePin(IMU_I2C_SCL_GPIO_Port, IMU_I2C_SCL_Pin, GPIO_PIN_RESET);
//        }
//    }
//
//    mag_config_t cfg;
//    cfg.enable_temp_sens = false;
//    cfg.xy_om = MAG_MEDIUM_POWER;
//    cfg.z_om = MAG_MEDIUM_POWER;
//    cfg.odr = MAG_20_HZ;
//    cfg.fast_odr = false;
//    cfg.fs_rng = MAG_FS_16_GAUSS;
//    cfg.sys_mode = MAG_CONTINUOUS;
//    cfg.block_data_update = true;
//
//    if(set_mag_ctrl_regs(&cfg)){
//        printf("Successfully wrote magnnetometer config\n");
//    } else{
//        printf("Failed to write magnetometer config\n");
//    }
//
//    imu_axis_t mag_read[3] = {0};
//    int16_t raw_adc[3] = {0};
//
//    for(;;){
//        if (check_if_mag_avail()){
//           if(read_mag_axes(mag_read, raw_adc, &cfg)){
//               printf("Magnetometer value: %.03f %.03f %.03f\n", mag_read[0], mag_read[1], mag_read[2]);
//           } else{
//               printf("Magnetometer fail read\n");
//           }
//        } else{
//            // printf("Magnetometer not avail\n");
//        }
//
//        osThreadYield();
//        // osDelay(100);
//    }
//
//    osThreadTerminate(NULL);
//}

/**
 * @brief Reads accelerometer and gyroscope sensor values from the IMU
 * @param argument: Not used
 * @retval None
 */
//void ReadInertial(void *argument) {
//    inertial_cfg_t cfg;
//    cfg.use_accel_high_power = true;
//    cfg.use_gyro_high_power = true;
//    cfg.block_data_update = true;
//
//    cfg.accel_fs_rng = ACCEL_FS_4_G;
//    cfg.accel_odr = ACCEL_26_HZ;
//
//    cfg.gyro_fs_rng = GYRO_FS_125_DPS;
//    cfg.gyro_odr = GYRO_26_HZ;
//
//    if (set_inertial_ctrl_regs(&cfg)) {
//        printf("Successfully wrote accel and gyro config\n");
//    } else {
//        printf("Failed to write accel and gyro config\n");
//    }
//
//    imu_axis_t gyro_read[3] = { 0 };
//    imu_axis_t accel_read[3] = { 0 };
//
//    int16_t gyro_raw_adc[3] = { 0 };
//    int16_t accel_raw_adc[3] = { 0 };
//
//    for (;;) {
//        if (check_if_inertial_avail()) {
//            if (read_inertial_axes(accel_read, gyro_read, accel_raw_adc,
//                    gyro_raw_adc, &cfg)) {
//                // printf("Received inertial!!!\n");
//                printf("Angular rates: %.03f %.03f %.03f\n", gyro_read[0],
//                        gyro_read[1], gyro_read[2]);
//                printf("Linear accel: %.03f %.03f %.03f\n", accel_read[0],
//                        accel_read[1], accel_read[2]);
//            } else {
//                printf("Inertial fail read\n");
//            }
//        } else {
//            // printf("Intertial fail avail\n");
//        }
//
//        osThreadYield();
//        //osDelay(100);
//    }
//
//    osThreadTerminate(NULL);
//}
