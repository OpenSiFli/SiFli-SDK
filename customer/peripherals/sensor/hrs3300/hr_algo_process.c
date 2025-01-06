/**
  ******************************************************************************
  * @file   hr_sensor_service.c
  * @author Sifli software development team
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

#include "rtthread.h"

#if defined(SENSOR_USING_HR)
#include "hx_hrs3300.h"
#include "hr_algo_process.h"

#if defined (HR_ALGO_USING_GSENSOR_DATA)
    #include "gsensor_service.h"
#endif

#define HR_NO_TOUCH     10
#define HR_TIMEOUT      30


extern hr_algo_result_t Hrs3300_alg_get_results(void);
extern bp_algo_result_t Hrs3300_alg_get_bp_results(void);
extern bool Hrs3300_alg_send_data(int16_t new_raw_data, int16_t als_raw_data, int16_t gsen_data_x, int16_t gsen_data_y, int16_t gsen_data_z, uint16_t timer_time);
extern bool Hrs3300_bp_alg_send_data(int16_t new_raw_data);

static hr_algo_ctrl_data_t ctrl_data;

void hrs_algo_process(uint8_t hr_type, hr_raw_data_t *hr_data, void *g_in)
{
    if (SENSOR_HR == hr_type)
    {
#if defined (HR_ALGO_USING_GSENSOR_DATA)
        /* Note: when using this function, make sure that the HR cycle is a multiple of the gsensor cycle, otherwise there will be problems */
        if (g_in)
        {
            gsensors_fifo_t *g_data = (gsensors_fifo_t *)g_in;

            /* You can refer to the following code to obtain all three-axis data (x, y, z) */
#if 0
            rt_list_t *pos;
            rt_list_for_each(pos, &g_in)
            {
                gsensors_fifo_t *node = rt_list_entry(pos, gsensors_fifo_t, list);
                for (int i = 0; i < node->num; i++)
                {
                    float x = node->buf[i].acce_data[0];
                    float y = node->buf[i].acce_data[1];
                    float z = node->buf[i].acce_data[2];
                }
            }
#endif
            Hrs3300_alg_send_data(hr_data->hrm_raw, hr_data->alg_raw, g_data->buf[0].acce_data[0], g_data->buf[0].acce_data[1], g_data->buf[0].acce_data[2], 0);
        }
        else
#endif
        {
            Hrs3300_alg_send_data(hr_data->hrm_raw, hr_data->alg_raw, 0, 0, 0, 0);
        }
    }
#if 0
    else if (SENSOR_SPO2 == hr_type)
    {
        Hrs3300_bp_alg_send_data(hr_data->hrm_raw);
    }
#endif
}


static uint8_t hr_algo_postprocess(uint16_t alg_status)
{
    uint8_t ret = HR_MEAS_NULL;

    if (alg_status == HEALTH_NO_TOUCH)
    {
        ctrl_data.no_touch_num ++;
        rt_kprintf("hr_postprocess:no %d max %d one %d\n", ctrl_data.no_touch_num, HR_NO_TOUCH, ctrl_data.no_touch_one);
        if (ctrl_data.no_touch_num >= HR_NO_TOUCH)
        {
            ret = HR_MEAS_ABNORMAL;
            ctrl_data.no_touch_num = 0;
            if (!ctrl_data.no_touch_one) ret = HR_MEAS_NO_TOUCH;
            ctrl_data.no_touch_one = 1;
            rt_kprintf("hr_postprocess HR_NO_TOUCH\n");
        }

        goto end;
    }
    else if (alg_status == HEALTH_PPG_LEN_TOO_SHORT || alg_status == HEALTH_ALG_TIMEOUT)
    {
        ctrl_data.hr_timeout ++;
        if (ctrl_data.hr_timeout >= HR_TIMEOUT)
        {
            ret = HR_MEAS_ABNORMAL;
            ctrl_data.hr_timeout = 0;
            if (!ctrl_data.hr_timeout_one) ret = HR_MEAS_TIMEOUT;
            ctrl_data.hr_timeout_one = 1;
            rt_kprintf("hr_postprocess HEALTH_PPG_LEN_TOO_SHORT | HEALTH_ALG_TIMEOUT\n");
        }

        goto end;
    }
    else if (alg_status == HEALTH_HR_READY)
    {

        ret = HR_MEAS_OK;
    }
    else if (alg_status == HEALTH_SETTLE)
    {
        ;
    }

    ctrl_data.hr_timeout = 0;
    ctrl_data.no_touch_num = 0;

end:

    return ret;
}

static uint8_t bp_algo_postprocess(rt_bp_info_ind_t *bp_info)
{
#if 0
    hr_algo_result_t hr_results = Hrs3300_alg_get_bp_results();

    //hr_results.spo2 = bp_results

    if (hr_results.object_flg == 1)
    {
        // 固体检测标志位
    }
    if (hr_results.bp_alg_status == MSG_BP_NO_TOUCH)
    {
        //LOG_PRINTF(0, "-----  blood MSG_BP_NO_TOUCH \r\n");
        /*
        uint8_t temp[2] ;
        temp[0] =    0 ;
        temp[1] =    0 ;
        Set_Monitor_value(temp,2);*/
    }
    else if (hr_results.bp_alg_status == MSG_BP_PPG_LEN_TOO_SHORT)
    {
        //LOG_PRINTF(0, "-----  BLOOD MSG_BP_PPG_LEN_TOO_SHORT  \r\n");
        //opr_display_hr(0);  // customer can print waiting information here
    }
    else if (hr_results.bp_alg_status == MSG_BP_ALG_TIMEOUT)
    {
        //LOG_PRINTF(0, "-----  BLOOD MSG_BP_ALG_TIMEOUT  \r\n");
        //LOG_PRINTF(0, "-----  blood =%d  %d\r\n",hr_results.sbp, hr_results.dbp);
        if (hr_results.sbp != 0)
        {
            uint8_t temp[2] ;
            temp[0] =    hr_results.sbp ;
            temp[1] =    hr_results.dbp ;
            //LOG_PRINTF(0, "-----  blood =%d  %d\r\n",hr_results.sbp, hr_results.dbp);
            //Set_Monitor_value(temp,2);

        }
        else
        {
            //      Set_blood_value(118 + rand()%8,68 + rand()%8);
            hr_results.sbp = 118 + rand() % 8;
            hr_results.dbp = 68 + rand() % 8;
            uint8_t temp[2] ;
            temp[0] =   hr_results.sbp ;
            temp[1] =   hr_results.dbp ;
            //  uint8_t temp[2] = {hr_results.sbp , hr_results.dbp} ;
            //Set_Monitor_value(temp,2);
            //      Set_User_blood_value(hr_results.sbp, hr_results.dbp);
        }
        //opr_display_bp(hr_results.sbp, hr_results.dbp);  // customer can print real heart rate here
    }
    else if (hr_results.bp_alg_status == MSG_BP_READY)
    {
        uint8_t temp[2] ;
        temp[0] =    hr_results.sbp ;
        temp[1] =    hr_results.dbp ;
        //LOG_PRINTF(0, "-----  blood =%d  %d\r\n",hr_results.sbp, hr_results.dbp);
        //Set_Monitor_value(temp,2);
    }
#endif
    return 0;
}

uint8_t hrs_algo_postprocess(uint8_t hr_type, hrs_info_t *info)
{
    uint8_t ret = HR_MEAS_NULL;
    static uint32_t meas_sum = 0;
    static uint16_t  meas_count = 0;
    static uint16_t  meas_ok_count = 0;

    if (SENSOR_HR == hr_type || SENSOR_SPO2 == hr_type)
    {
        hr_algo_result_t hr_results = Hrs3300_alg_get_results();
        ret = hr_algo_postprocess(hr_results.alg_status);
        meas_count++;

        if (HR_MEAS_OK == ret)
        {
            meas_ok_count++;
            if (SENSOR_HR == hr_type)
            {
                meas_sum += hr_results.hr_result;
                if (meas_ok_count >= HR_MEAS_SUM_COUNT)
                {
                    info->hr_info.hr = meas_sum / meas_ok_count;

                    ret = HR_MEAS_SUCC;
                }
            }
            //else if(SENSOR_SPO2 == hr_type)
            //{
            //  meas_sum += hr_results.hr_result
            //  if(meas_ok_count >= HR_MEAS_SUM_COUNT)
            //  {
            //      info->spo2_info.spo2 = meas_sum / meas_ok_count;

            //      ret = HR_MEAS_SUCC;
            //  }
            //}
        }
        else if (HR_MEAS_ABNORMAL == ret
                 || (HR_MEAS_NULL == ret && meas_count >= HR_MEAS_SUM_COUNT * 10))
        {
            ret = HR_MEAS_TIMEOUT;
        }

        if (HR_MEAS_NO_TOUCH == ret || HR_MEAS_SUCC == ret || HR_MEAS_TIMEOUT == ret)
        {
            meas_sum = 0;
            meas_count = 0;
            meas_ok_count = 0;
        }
    }
#if 0
    else if ((SENSOR_BP == hr_type))
    {
        rt_bp_info_ind_t *bp_info = (rt_spo2_info_ind_t *) value;
        ret = rt_bp_info_ind_t(bp_info);
    }
#endif

    return ret;

}
#endif // RT_USING_SENSOR

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
