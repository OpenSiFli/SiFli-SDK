/**
  ******************************************************************************
  * @file   compass_service.c
  * @author Sifli software development team
  * @brief compass service source.
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


#include <string.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <rtdef.h>
#include <board.h>
#include "data_service_subscriber.h"
#include "data_service_provider.h"
#include "sensor.h"
#include "sensor_memsic_mmc36x0kj.h"
#include "MemsicCompass.h"
#include "MemsicAlgo.h"
#include "compass_service.h"

#define MAG_SERVICE_NAME "MAG"


#define A_LAYOUT    0
#define M_LAYOUT    0


typedef struct
{
    uint8_t ref_count;
    /* mag service handle */
    uint8_t mag_handle;
    datas_handle_t service;

    /* default hard iron offset and magnetic field strength*/
    float mag_hmm[4];

    //TODO: need FIFO
    struct rt_sensor_data mag_raw_data;

    /* These variables are used to save the magnetic sensor calibrated data. */
    float cal_mag[3];

    compass_data_t output_data;

    compass_data_t last_output_data;

    float cali_mag_para[4];
    //ble_weather_srv_data_t data;
} compass_service_env_t;


static float comp_mag_smm[9] = {1.0f, 0.0f, 0.0f, \
                                0.0f, 1.0f, 0.0f, \
                                0.0f, 0.0f, 1.0f
                               };

static compass_service_env_t compass_service_env;


static int mag_service_callback(data_callback_arg_t *arg)
{
    compass_service_env_t *env = &compass_service_env;

    switch (arg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_RSP:
    {
        data_subscribe_rsp_t *rsp;
        rsp = (data_subscribe_rsp_t *)arg->data;
        RT_ASSERT(rsp);
        if (rsp->result < 0)
        {
            env->mag_handle = DATA_CLIENT_INVALID_HANDLE;
        }
        else
        {
            env->mag_handle = rsp->handle;
        }


        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_RSP:
    {
        env->mag_handle = -1;
        break;
    }
    case MSG_SERVICE_DATA_NTF_IND:
    {
        uint8_t *data;

        RT_ASSERT(arg->data_len > 0);
        RT_ASSERT(arg->data);

        data = rt_malloc(arg->data_len);
        memcpy(data, arg->data, arg->data_len);
        datas_data_ready(env->service, arg->data_len, data);
        break;
    }
    }
    return 0;
}

static int32_t compass_service_subscribe(datas_handle_t service)
{
    if (DATA_CLIENT_INVALID_HANDLE == compass_service_env.mag_handle && compass_service_env.ref_count == 0)
    {
        compass_service_env.mag_handle = datac_open();
        RT_ASSERT(DATA_CLIENT_INVALID_HANDLE != compass_service_env.mag_handle);
        datac_subscribe(compass_service_env.mag_handle, MAG_SERVICE_NAME, mag_service_callback, 0);
    }
    compass_service_env.ref_count++;
    return 0;
}

static int32_t compass_service_unsubscribe(datas_handle_t service)
{
    int32_t result = 0;
    compass_service_env.ref_count--;
    if ((DATA_CLIENT_INVALID_HANDLE != compass_service_env.mag_handle) && compass_service_env.ref_count == 0)
    {
        datac_close(compass_service_env.mag_handle);
    }
    return result;
}

static int32_t compass_service_config(datas_handle_t service, void *config)
{
    return 0;
}


/* Read saved calibration parameters. These para is saved in somewhere that can not disappear when power-off
 * ReadPara function should be create by customer.If customer can not implement this function, should comment it out
 * If there is no parameter saved, ReadPara() should return -1, else return 1;
 * If there is no para saved, pass the default value to the algorithm library
 */
static float sp[4] = {0.0f, 0.0f, 0.0f, 0.5f};
static int ReadPara(float *p)
{
    int i;

    /*
    .
    . Need to be implemented by user.
    .
    */

    for (i = 0; i < 4; i++)
    {
        p[i] = sp[i];
    }
    return 1;
}

/* Save calibration parameters in somewhere that can not lose after power-off.
 * When the system power on next time, need to read this parameters by the function ReadPara() for algorithm initial.
 * If save parameter successfully, return 1, else return -1;
 */
static int SavePara(float *p)
{
    int i;

    /*
    .
    . Need to be implemented by user.
    .
    */
    for (i = 0; i < 4; i++)
    {
        sp[i] = p[i] ;
    }

    return 1;
}

/* Convert the sensor coordinate to right-front-up coordinate system;
 */
static void acc_coord_raw_to_real(int layout, float *in, float *out)
{
    switch (layout)
    {
    case 0:
        out[0] =  in[0];
        out[1] =  in[1];
        out[2] =  in[2];
        break;
    case 1:
        out[0] = -in[1];
        out[1] =  in[0];
        out[2] =  in[2];
        break;
    case 2:
        out[0] = -in[0];
        out[1] = -in[1];
        out[2] =  in[2];
        break;
    case 3:
        out[0] =  in[1];
        out[1] = -in[0];
        out[2] =  in[2];
        break;
    case 4:
        out[0] =  in[1];
        out[1] =  in[0];
        out[2] = -in[2];
        break;
    case 5:
        out[0] = -in[0];
        out[1] =  in[1];
        out[2] = -in[2];
        break;
    case 6:
        out[0] = -in[1];
        out[1] = -in[0];
        out[2] = -in[2];
        break;
    case 7:
        out[0] =  in[0];
        out[1] = -in[1];
        out[2] = -in[2];
        break;
    default:
        out[0] =  in[0];
        out[1] =  in[1];
        out[2] =  in[2];
        break;
    }
}
/* Convert the sensor(MMC3630KJ) coordinate to right-front-up coordinate system;
 */
static void mag_coord_raw_to_real(int layout, float *in, float *out)
{
    switch (layout)
    {
    case 0:
        out[0] =  in[0];
        out[1] =  in[1];
        out[2] =  in[2];
        break;
    case 1:
        out[0] = -in[1];
        out[1] =  in[0];
        out[2] =  in[2];
        break;
    case 2:
        out[0] = -in[0];
        out[1] = -in[1];
        out[2] =  in[2];
        break;
    case 3:
        out[0] =  in[1];
        out[1] = -in[0];
        out[2] =  in[2];
        break;
    case 4:
        out[0] =  in[1];
        out[1] =  in[0];
        out[2] = -in[2];
        break;
    case 5:
        out[0] = -in[0];
        out[1] =  in[1];
        out[2] = -in[2];
        break;
    case 6:
        out[0] = -in[1];
        out[1] = -in[0];
        out[2] = -in[2];
        break;
    case 7:
        out[0] =  in[0];
        out[1] = -in[1];
        out[2] = -in[2];
        break;
    default:
        out[0] =  in[0];
        out[1] =  in[1];
        out[2] = -in[2];
        break;
    }
}


static void process_data(void)
{
    compass_service_env_t *env = &compass_service_env;

    float acc_raw_data[3] = {0.0f}; //accelerometer field vector, unit is g
    float acc_real_data[3] = {0.0f};
    float mag_raw_data[3] = {0.0f}; //accelerometer field vector, unit is g
    float mag_real_data[3] = {0.0f};
    /* This variable is used to save the calibrated orientation data. */
    float cali_ori[3] = {0.0f};

    int save_flag;

    /* Get the acc raw data, unit is g*/
    acc_raw_data[0] = 0.0;
    acc_raw_data[1] = 0.0;
    acc_raw_data[2] = 0.0;

    /* convert the coordinate system */
    acc_coord_raw_to_real(A_LAYOUT, acc_raw_data, acc_real_data);

    mag_raw_data[0] = (float)env->mag_raw_data.data.mag.x / 1000;
    mag_raw_data[1] = (float)env->mag_raw_data.data.mag.y / 1000;
    mag_raw_data[2] = (float)env->mag_raw_data.data.mag.z / 1000;

    /* Convert the coordinate system */
    mag_coord_raw_to_real(M_LAYOUT, mag_raw_data, mag_real_data);

    /* Below functions are algorithm interface.
     * input acc, mag data into algorithm
     * make sure that acc and mag XYZ data meet the right-hand(right-front-up) coordinate system
     */
    MainAlgorithmProcess(acc_real_data, mag_real_data, 1);

    /* Get calibrated mag data */
    GetCalMag(env->cal_mag);

    /* Get orientation vector */
    GetCalOri(acc_real_data, env->cal_mag, cali_ori);

    /* Get the fAzimuth Pitch Roll for the eCompass */
    env->output_data.azimuth = cali_ori[0];
    env->output_data.pitch = cali_ori[1];
    env->output_data.roll = cali_ori[2];

    /* Get the accuracy of the algorithm */
    env->output_data.accuracy = GetMagAccuracy();

    /* Get the SET Flag from algorithm */
    if (GetMagSaturation())
    {
        //MEMSIC_Magnetic_Sensor_SET(); //Do SET action
    }

#if 0
    /* Get corrected mag data */
    save_flag = GetCalPara(env->cali_mag_para);

    if (save_flag == 1)
    {
        /* Save the calpara buffer into the system file */
        /* This function should be create by customer.If customer can not implement this function, should comment it out */
        SavePara(env->cali_mag_para);
    }
#endif
}


static int32_t compass_service_msg_handler(datas_handle_t service, data_msg_t *msg)
{
    switch (msg->msg_id)
    {
    case MSG_SERVICE_SUBSCRIBE_REQ:
    {
        compass_service_subscribe(service);
        break;
    }
    case MSG_SERVICE_UNSUBSCRIBE_REQ:
    {
        compass_service_unsubscribe(service);
        break;
    }
    case MSG_SERVICE_CONFIG_REQ:
    {
        int32_t result;
        data_req_t *req = (data_req_t *)data_service_get_msg_body(msg);
        result = compass_service_config(service, &req->data[0]);
        datas_send_response(service, msg, result);
        break;
    }

    case MSG_SRV_COMPASS_CUR_VAL_GET_REQ:
    {
        compass_service_env_t *env = &compass_service_env;

        datas_send_response_data(service, msg, sizeof(env->output_data), (uint8_t *)&env->output_data);
        break;
    }
    case MSG_SERVICE_DATA_RDY_IND:
    {
        compass_service_env_t *env = &compass_service_env;
        data_rdy_ind_t *data_ind = (data_rdy_ind_t *)(data_service_get_msg_body(msg));

        RT_ASSERT(data_ind);
        RT_ASSERT(sizeof(env->mag_raw_data) == data_ind->len);

        memcpy(&env->mag_raw_data, data_ind->data, data_ind->len);
        rt_free(data_ind->data);

        process_data();
        break;
    }
    default:
    {
        RT_ASSERT(0);
    }
    }

    return 0;
}

static data_service_config_t compass_service_cb =
{
    .max_client_num = 5,
    .queue = RT_NULL,
    .msg_handler = compass_service_msg_handler,
};


static void init_algo(void)
{
    int i;

    /* This variable is used to save calibrated mag para that need to be saved in system file */
    float save_data[4] = {0.0f, 0.0f, 0.0f, 0.5f};

    /* Read saved calibration parameters. These para is saved in somewhere that can not disappear when power-off
     * ReadPara function should be create by customer.If customer can not implement this function, should comment it out
     * If there is no parameter saved, ReadPara() should return -1, else return 1;
     * If there is no para saved, pass the default value to the algorithm library
     */
    if (ReadPara(save_data) == 1)
    {
        for (i = 0; i < 4; i++)
        {
            compass_service_env.mag_hmm[i] = save_data[i];
        }
    }

    /* Initial the acc, mag, and calibrated parameters
     * if already saved the calibrated offset and radius last time, read it out and init the magOffset and magRadius
     */
    InitialAlgorithm(comp_mag_smm, save_data);
}

int compass_service_register(void)
{
    compass_service_env.mag_handle = DATA_CLIENT_INVALID_HANDLE;
    compass_service_env.service = datas_register("COMP", &compass_service_cb);
    RT_ASSERT(compass_service_env.service);

    init_algo();

    return 0;
}

INIT_COMPONENT_EXPORT(compass_service_register);


