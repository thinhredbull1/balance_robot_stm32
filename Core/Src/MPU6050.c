/*
 * mpu6050.c
 *
 *  Created on: Nov 13, 2019
 *      Author: Bulanov Konstantin
 *
 *  Contact information
 *  -------------------
 *
 * e-mail   :  leech001@gmail.com
 */

/*
 * |---------------------------------------------------------------------------------
 * | Copyright (C) Bulanov Konstantin,2021
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |
 * | Kalman filter algorithm used from https://github.com/TKJElectronics/KalmanFilter
 * |---------------------------------------------------------------------------------
 */

#include <math.h>
#include "MPU6050.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

// Setup MPU6050
#define MPU6050_ADDR 0xD0
const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;

// Gyro zero-rate bias in deg/s, measured by MPU6050_Calibrate()
static float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;

#define CALIB_SAMPLES      200    // ~1 s at 2 ms spacing
#define CALIB_MAX_RANGE    150    // raw LSB (~1.1 deg/s): larger spread = robot was moving
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
        return 0;
    }
    return check;
}

/*
 * Average the gyro while the robot is held still and store it as bias.
 * Returns 0 = OK, 1 = I2C error, 2 = robot moved during calibration (bias NOT updated).
 */
uint8_t MPU6050_Calibrate(I2C_HandleTypeDef *I2Cx)
{
    int32_t sum[3] = {0, 0, 0};
    int16_t mn[3] = {32767, 32767, 32767};
    int16_t mx[3] = {-32768, -32768, -32768};
    uint8_t d[6];

    for (int i = 0; i < CALIB_SAMPLES; i++)
    {
        if (HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, d, 6, i2c_timeout) != HAL_OK)
            return 1;
        for (int a = 0; a < 3; a++)
        {
            int16_t v = (int16_t)(d[2 * a] << 8 | d[2 * a + 1]);
            sum[a] += v;
            if (v < mn[a]) mn[a] = v;
            if (v > mx[a]) mx[a] = v;
        }
        HAL_Delay(2);
    }

    for (int a = 0; a < 3; a++)
        if ((mx[a] - mn[a]) > CALIB_MAX_RANGE)
            return 2;

    gyroBiasX = (float)sum[0] / CALIB_SAMPLES / 131.0f;
    gyroBiasY = (float)sum[1] / CALIB_SAMPLES / 131.0f;
    gyroBiasZ = (float)sum[2] / CALIB_SAMPLES / 131.0f;
    return 0;
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2];
    int16_t temp;

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}


void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14];
    int16_t temp;

    // Read MPU6050 data
    if (HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR,
                         ACCEL_XOUT_H_REG, 1,
                         Rec_Data, 14, i2c_timeout) != HAL_OK)
    {
        return;
    }

    // Raw data
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    // Convert accelerometer
    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0f;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0f;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;

    // Temperature
    DataStruct->Temperature = (float)temp / 340.0f + 36.53f;

    // Convert gyroscope (deg/s)
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0f - gyroBiasX;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0f - gyroBiasY;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0f - gyroBiasZ;

    // Calculate dt
    // Microsecond-accurate dt from the DWT cycle counter (enabled in main.c)
    static uint32_t lastCycles = 0;
    uint32_t currentCycles = DWT->CYCCNT;

    float dt = (float)(currentCycles - lastCycles) / (float)SystemCoreClock;
    lastCycles = currentCycles;

    // Avoid abnormal dt on first reading
    if (dt <= 0.0f || dt > 0.1f)
    {
        dt = 0.01f;
    }

    // Accelerometer angles
    float rollAcc = atan2f(DataStruct->Ay, DataStruct->Az) * RAD_TO_DEG;

    float pitchAcc = atan2f(-DataStruct->Ax,
                            sqrtf(DataStruct->Ay * DataStruct->Ay +
                                  DataStruct->Az * DataStruct->Az)) * RAD_TO_DEG;

    // Complementary filter
    const float alpha = 0.99f;

    static uint8_t firstRun = 1;
    static float pitch = 0.0f;
    static float roll = 0.0f;

    if (firstRun)
    {
        pitch = pitchAcc;
        roll = rollAcc;
        firstRun = 0;
    }
    else
    {
        // Pitch: Gyroscope Y
        pitch = alpha * (pitch + DataStruct->Gy * dt)
                + (1.0f - alpha) * pitchAcc;

        // Roll: Gyroscope X
        roll = alpha * (roll + DataStruct->Gx * dt)
               + (1.0f - alpha) * rollAcc;
    }

    // Save filtered angles
    DataStruct->AngleRoll = roll;
    DataStruct->AnglePitch = pitch;
}



