/**
  ******************************************************************************
  * @file   IMU.c
  * @author Sifli software development team
  * @brief   This file provides all the IMU firmware functions.
  *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <math.h>
#include "board.h"
#include "ICM20948.h"

#ifdef DOF10_IMU

#define DRV_DEBUG
#define LOG_TAG              "drv.imu"
#include <drv_log.h>

typedef struct imu_st_sensor_data_tag
{
    int16_t s16X;
    int16_t s16Y;
    int16_t s16Z;
} IMU_ST_SENSOR_DATA;


volatile float q0 = 1.0f;;
volatile float q1 = 0.0f;
volatile float q2 = 0.0f;
volatile float q3 = 0.0f;
IMU_ST_SENSOR_DATA gstMagOffset = {0, 0, 0};
/**
  * @brief  invSqrt
  * @param
  * @retval
  */

float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;

    long i = *(long *)&y;               //get bits for floating value
    i = 0x5f3759df - (i >> 1);          //gives initial guss you
    y = *(float *)&i;                   //convert bits back to float
    y = y * (1.5f - (halfx * y * y));   //newtop step, repeating increases accuracy

    return y;
}


#define Kp 4.50f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 1.0f    // integral gain governs rate of convergence of gyroscope biases

/**
  * @brief  Updata attitude and heading
  * @param  ax: accelerometer X
  * @param  ay: accelerometer Y
  * @param  az: accelerometer Z
  * @param  gx: gyroscopes X
  * @param  gy: gyroscopes Y
  * @param  gz: gyroscopes Z
  * @param  mx: magnetometer X
  * @param  my: magnetometer Y
  * @param  mz: magnetometer Z
  * @retval None
  */
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;
    float ex, ey, ez, halfT = 0.024f;

    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    norm = invSqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    norm = invSqrt(mx * mx + my * my + mz * mz);
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm;

    // compute reference direction of flux
    hx = 2 * mx * (0.5f - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2);
    hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5f - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1);
    hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5f - q1q1 - q2q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = hz;

    // estimated direction of gravity and flux (v and w)
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2 * bx * (0.5 - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
    wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
    wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5 - q1q1 - q2q2);

    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    if (ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
        exInt = exInt + ex * Ki * halfT;
        eyInt = eyInt + ey * Ki * halfT;
        ezInt = ezInt + ez * Ki * halfT;

        gx = gx + Kp * ex + exInt;
        gy = gy + Kp * ey + eyInt;
        gz = gz + Kp * ez + ezInt;
    }

    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;
}


/**
  * @brief  Get quaters
  * @param  None
  * @retval None
  */
void IMU_GetQuater(void)
{
    float MotionVal[9];
    int16_t s16Gyro[3], s16Accel[3], s16Magn[3];

    invmsICM20948AccelRead(&s16Accel[0], &s16Accel[1], &s16Accel[2]);
    invmsICM20948GyroRead(&s16Gyro[0], &s16Gyro[1], &s16Gyro[2]);
    invmsICM20948MagRead(&s16Magn[0], &s16Magn[1], &s16Magn[2]);

    MotionVal[0] = s16Gyro[0] / 32.8;
    MotionVal[1] = s16Gyro[1] / 32.8;
    MotionVal[2] = s16Gyro[2] / 32.8;
    MotionVal[3] = s16Accel[0];
    MotionVal[4] = s16Accel[1];
    MotionVal[5] = s16Accel[2];
    MotionVal[6] = s16Magn[0] - gstMagOffset.s16X;
    MotionVal[7] = s16Magn[1] - gstMagOffset.s16Y;
    MotionVal[8] = s16Magn[2] - gstMagOffset.s16Z;
    IMU_AHRSupdate((float)MotionVal[0] * 0.0175, (float)MotionVal[1] * 0.0175, (float)MotionVal[2] * 0.0175,
                   (float)MotionVal[3], (float)MotionVal[4], (float)MotionVal[5], (float)MotionVal[6], (float)MotionVal[7], MotionVal[8]);
}


/**
  * @brief  Get Yaw Pitch Roll
  * @param  None
  * @retval None
  */
void IMU_GetYawPitchRoll(float *Angles)
{
    IMU_GetQuater();
    Angles[1] = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3; // pitch
    Angles[2] = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3; // roll
    Angles[0] = atan2(-2 * q1 * q2 - 2 * q0 * q3, 2 * q2 * q2 + 2 * q3 * q3 - 1) * 57.3; // yaw
}


#endif  //DOF10_IMU

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
