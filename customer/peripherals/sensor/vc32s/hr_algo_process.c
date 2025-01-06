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
#include "vcHr02Hci.h"
#include "sensor_if.h"
#include "spo2Algo.h"
#include "vcSportMotionAlgo.h"
#include "hr_algo_process.h"
#include "hr_sensor_service.h"

#if defined (HR_ALGO_USING_GSENSOR_DATA)
    #include "gsensor_service.h"
#endif

#define HR_NO_TOUCH     10
#define HR_TIMEOUT      30

static uint8_t MIN_Blood_Process(uint8_t hr_value);
static uint8_t MAX_Blood_Process(uint8_t hr_value);

static hr_algo_result_t health_value;
static hr_algo_ctrl_data_t ctrl_data;
/* The algorithm of hert rate data struct */
AlgoInputData_t algoInputData;
AlgoOutputData_t algoOutputData;
/* Sport Mode In Heart Rate Mode */
static AlgoSportMode_t vcSportMode = SPORT_TYPE_NORMAL;
/* Heart rate value */
int HeartRateValue = 0;

void hrs_algo_set_sport_mode(AlgoSportMode_t mode)
{
    vcSportMode = mode;
}

static rt_err_t blood_pressure_algorithm(uint8_t hr_value, uint8_t *bp_shr_ptr, uint8_t *bp_dia_ptr)
{

    if ((bp_shr_ptr == RT_NULL) || (bp_dia_ptr == RT_NULL))
    {
        return -RT_ERROR;
    }
    if (hr_value == 0)
    {
        *bp_shr_ptr = 0;
        *bp_dia_ptr = 0;
        return -RT_ERROR;
    }

    if (hr_value > 142)
    {
        hr_value = 142;
    }
    else if (hr_value < 50)
    {
        hr_value = 50;
    }
    static uint8_t bp_shr, bp_dia;
    uint8_t num = MAX_Blood_Process(hr_value);
    switch (hr_value)
    {
    case 50:
        bp_shr = 92 + num;
        break;
    case 51:
        bp_shr = 93 + num;
        break;
    case 52:
        bp_shr = 95 + num;
        break;
    case 53:
        bp_shr = 96 + num;
        break;
    case 54:
    case 55:
        bp_shr = 97 + num;
        break;
    case 56:
    case 57:
        bp_shr = 99 + num;
        break;
    case 58:
        bp_shr = 100 + num;
        break;
    case 59:
        bp_shr = 101 + num;
        break;
    case 60:
    case 61:
        bp_shr = 102 + num;
        break;
    case 62:
        bp_shr = 105 + num;
        break;
    case 63:
    case 64:
        bp_shr = 108 + num;
        break;
    case 65:
        bp_shr = 109 + num;
        break;
    case 66:
        bp_shr = 110 + num;
        break;
    case 67:
        bp_shr = 111 + num;
        break;
    case 68:
        bp_shr = 112 + num;
        break;
    case 69:
        bp_shr = 116 + num;
        break;
    case 70:
        bp_shr = 118 + num;
        break;
    case 71:
    case 72:
        bp_shr = 119 + num;
        break;
    case 73:
        bp_shr = 120 + num;
        break;
    case 74:
        bp_shr = 122 + num;
        break;
    case 75:
    case 76:
        bp_shr = 123 + num;
        break;
    case 77:
    case 78:
    case 83:
        bp_shr = 125 + num;
        break;
    case 79:
    case 81:
    case 82:
    case 84:
        bp_shr = 126 + num;
        break;
    case 80:
    case 85:
        bp_shr = 127 + num;
        break;
    case 86:
    case 87:
    case 88:
    case 91:
    case 92:
    case 93:
    case 94:
        bp_shr = 128 + num;
        break;
    case 89:
    case 90:
    case 95:
    case 96:
        bp_shr = 129 + num;
        break;
    case 97:
        bp_shr = 130 + num;
        break;
    case 98:
        bp_shr = 131 + num;
        break;
    case 99:
    case 100:
        bp_shr = 134 + num;
        break;
    case 101:
    case 102:
        bp_shr = 135 + num;
        break;
    case 103:
    case 104:
    case 105:
    case 106:
        bp_shr = 136 + num;
        break;
    case 107:
    case 108:
    case 109:
        bp_shr = 137 + num;
        break;
    case 110:
    case 111:
        bp_shr = 142 + num;
        break;
    case 112:
        bp_shr = 148 + num;
        break;
    case 113:
        bp_shr = 140 + num;
        break;
    case 114:
        bp_shr = 150 + num;
        break;
    case 115:
    case 118:
    case 119:
    case 120:
        bp_shr = 145 + num;
        break;
    case 116:
    case 117:
        bp_shr = 149 + num;
        break;
    case 121:
        bp_shr = 150 + num;
        break;
    case 122:
    case 123:
        bp_shr = 152 + num;
        break;
    case 124:
    case 125:
    case 126:
        bp_shr = 153 + num;
        break;
    case 127:
        bp_shr = 155 + num;
        break;
    case 128:
    case 129:
    case 130:
    case 131:
        bp_shr = 156 + num;
        break;
    case 132:
    case 133:
    case 134:
        bp_shr = 158 + num;
        break;
    case 135:
    case 136:
    case 137:
        bp_shr = 160 + num;
        break;
    case 138:
        bp_shr = 165 + num;
        break;
    case 139:
        bp_shr = 166 + num;
        break;
    case 140:
    case 141:
        bp_shr = 168 + num;
        break;
    case 142:
        bp_shr = 170 + num;
        break;
    }
    num = MIN_Blood_Process(hr_value);
    switch (hr_value)
    {
    case 50:
        bp_dia = 65 + num;
        break;
    case 51:
    case 52:
        bp_dia = 66 + num;
        break;
    case 53:
        bp_dia = 67 + num;
        break;
    case 54:
    case 55:
    case 56:
    case 57:
    case 58:
        bp_dia = 68 + num;
        break;
    case 59:
    case 60:
    case 61:
    case 62:
    case 63:
        bp_dia = 69 + num;
        break;
    case 64:
    case 65:
    case 66:
    case 67:
        bp_dia = 70 + num;
        break;
    case 68:
    case 69:
    case 70:
    case 71:
    case 72:
        bp_dia = 71 + num;
        break;
    case 73:
    case 74:
    case 75:
    case 76:
    case 77:
        bp_dia = 75 + num;
        break;
    case 78:
    case 79:
        bp_dia = 76 + num;
        break;
    case 80:
    case 81:
        bp_dia = 77 + num;
        break;
    case 82:
    case 83:
    case 84:
        bp_dia = 78 + num;
        break;
    case 85:
    case 86:
        bp_dia = 79 + num;
        break;
    case 87:
    case 88:
        bp_dia = 80 + num;
        break;
    case 89:
    case 90:
        bp_dia = 81 + num;
        break;
    case 91:
    case 92:
        bp_dia = 82 + num;
        break;
    case 93:
        bp_dia = 83 + num;
        break;
    case 94:
        bp_dia = 84 + num;
        break;
    case 95:
        bp_dia = 86 + num;
        break;
    case 96:
    case 97:
        bp_dia = 87 + num;
        break;
    case 98:
    case 99:
    case 100:
    case 103:
        bp_dia = 89 + num;
        break;
    case 101:
    case 102:
    case 104:
    case 105:
    case 106:
    case 107:
    case 108:
    case 109:
    case 110:
    case 111:
    case 112:
    case 113:
    case 114:
    case 115:
    case 116:
    case 117:
    case 118:
        bp_dia = 90 + num;
        break;
    case 119:
    case 120:
    case 121:
    case 122:
    case 123:
    case 124:
    case 125:
    case 126:
    case 127:
    case 128:
    case 129:
    case 130:
    case 131:
    case 132:
    case 133:
    case 134:
    case 135:
    case 136:
    case 137:
    case 138:
    case 139:
    case 140:
    case 141:
    case 142:
        bp_dia = 92 + num;
        break;
    }
    *bp_shr_ptr = bp_shr;
    *bp_dia_ptr = bp_dia;

    return RT_EOK;
}

static uint8_t blood_oxygen_algorithm(uint8_t hr_value)
{
    const static uint8_t spo2h_buff0[6] = {88, 89, 90, 91, 92, 93};
    const static uint8_t spo2h_buff1[4] = {94, 95, 96, 97};
    const static uint8_t spo2h_buff2[3] = {97, 98, 99};

    uint8_t spo2h = 0, num = rand() % 10;
    if (hr_value == 0)
    {
        return 0;
    }

    if (hr_value > 142)
    {
        hr_value = 142;
    }
    else if (hr_value < 45)
    {
        hr_value = 45;
    }

    if (hr_value < 50)
    {
        spo2h = spo2h_buff0[0];
    }
    else if (hr_value < 60)
    {
        num = num % 6;
        spo2h = spo2h_buff0[num];
    }
    else if (hr_value < 70)
    {
        num = num % 4;
        spo2h = spo2h_buff1[num];
    }
    else if (hr_value <= 100)
    {
        num = num % 3;
        spo2h = spo2h_buff2[num];
    }
    else
        spo2h = spo2h_buff2[2];

    return spo2h;
}

static uint8_t MAX_Blood_Process(uint8_t hr_value)
{
    uint8_t num = 0;
    switch (hr_value)
    {
    case 50:
    case 51:
    case 53:
    case 54:
    case 55:
    case 74:
    case 76:
    case 78:
    case 79:
    case 80:
        num = rand() % 6;
        break;
    case 52:
    case 69:
    case 71:
    case 75:
    case 77:
        num = rand() % 5;
        break;
    case 56:
    case 57:
    case 62:
    case 63:
    case 65:
    case 66:
    case 68:
    case 72:
    case 73:
        num = rand() % 7;
        break;
    case 58:
    case 59:
    case 60:
    case 61:
    case 64:
    case 67:
    case 81:
        num = rand() % 8;
        break;
    case 70:
        num = rand() % 4;
        break;
    case 82:
    case 84:
        num = rand() % 10;
        break;
    case 83:
        num = rand() % 11;
        break;
    case 85:
    case 86:
        num = rand() % 12;
        break;
    case 87:
        num = rand() % 13;
        break;
    case 88:
        num = rand() % 14;
        break;
    case 89:
        num = rand() % 15;
        break;
    case 90:
        num = rand() % 16;
        break;
    case 91:
        num = rand() % 18;
        break;
    case 92:
    case 93:
        num = rand() % 19;
        break;
    case 94:
    case 95:
    case 101:
        num = rand() % 21;
        break;
    case 96:
    case 99:
    case 100:
    case 112:
    case 114:
        num = rand() % 22;
        break;
    case 97:
    case 98:
    case 102:
        num = rand() % 23;
        break;
    case 103:
        num = rand() % 24;
        break;
    case 104:
        num = rand() % 25;
        break;
    case 105:
    case 110:
    case 111:
        num = rand() % 26;
        break;
    case 106:
    case 107:
        num = rand() % 27;
        break;
    case 108:
    case 115:
        num = rand() % 29;
        break;
    case 109:
    case 113:
    case 122:
        num = rand() % 30;
        break;
    case 116:
    case 117:
        num = rand() % 28;
        break;
    case 118:
    case 119:
    case 120:
    case 126:
    case 129:
    case 142:
        num = rand() % 34;
        break;
    case 121:
    case 123:
        num = rand() % 31;
        break;
    case 124:
    case 125:
    case 127:
    case 128:
        num = rand() % 33;
        break;
    case 130:
    case 139:
    case 141:
        num = rand() % 36;
        break;
    case 131:
    case 132:
    case 133:
    case 138:
        num = rand() % 37;
        break;
    case 134:
        num = rand() % 38;
        break;
    case 135:
    case 136:
        num = rand() % 39;
        break;
    case 137:
        num = rand() % 40;
        break;
    case 140:
        num = rand() % 35;
        break;
    default:
        break;
    }
    return num;
}

static uint8_t MIN_Blood_Process(uint8_t hr_value)
{
    uint8_t num = 0;
    switch (hr_value)
    {
    case 50:
    case 57:
    case 58:
    case 59:
    case 60:
    case 73:
        num = rand() % 5;
        break;
    case 51:
    case 52:
    case 56:
        num = rand() % 4;
        break;
    case 53:
    case 55:
        num = rand() % 3;
        break;
    case 54:
        num = rand() % 2;
        break;
    case 61:
    case 74:
    case 95:
    case 96:
    case 98:
        num = rand() % 6;
        break;
    case 62:
    case 63:
    case 64:
    case 65:
    case 68:
    case 69:
    case 72:
    case 75:
    case 80:
    case 81:
    case 94:
    case 97:
    case 99:
    case 119:
    case 120:
    case 121:
    case 122:
    case 123:
    case 124:
    case 125:
        num = rand() % 7;
        break;
    case 66:
    case 67:
    case 70:
    case 71:
    case 76:
    case 78:
    case 79:
    case 82:
    case 83:
    case 85:
    case 86:
    case 89:
    case 90:
    case 91:
    case 92:
    case 93:
    case 100:
    case 101:
    case 102:
    case 126:
        num = rand() % 8;
        break;
    case 77:
    case 84:
    case 87:
    case 88:
    case 104:
    case 105:
    case 106:
    case 107:
    case 108:
    case 109:
    case 110:
    case 111:
    case 112:
    case 113:
    case 114:
    case 115:
    case 116:
    case 117:
    case 118:
    case 127:
        num = rand() % 9;
        break;
    case 103:
    case 128:
        num = rand() % 10;
        break;
    case 129:
        num = rand() % 11;
        break;
    case 130:
        num = rand() % 12;
        break;
    case 131:
        num = rand() % 13;
        break;
    case 132:
        num = rand() % 14;
        break;
    case 133:
        num = rand() % 15;
        break;
    case 134:
        num = rand() % 16;
        break;
    case 135:
        num = rand() % 17;
        break;
    case 136:
        num = rand() % 18;
        break;
    case 137:
        num = rand() % 19;
        break;
    case 138:
        num = rand() % 20;
        break;
    case 139:
        num = rand() % 21;
        break;
    case 140:
        num = rand() % 22;
        break;
    case 141:
        num = rand() % 23;
        break;
    case 142:
        num = rand() % 24;
        break;
    default:
        break;
    }
    return num;
}


void hrs_algo_process(uint8_t hr_type, hr_raw_data_t *hr_data, void *g_in)
{
    uint8_t algoCallNum = 0;
    uint8_t vcSportFlag = 0;
    uint16_t gs_data_len = 0;
    uint8_t count_gs_data = 0, count_gs_data_left = 0;
    /* G-Sensor Data */
    int16_t xData[40] = {0};
    int16_t yData[40] = {0};
    int16_t zData[40] = {0};
    uint8_t cash_num[20] = {0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38};

    //clear health_value
    rt_memset((void *)&health_value, 0, sizeof(hr_algo_result_t));
    if (hr_data == RT_NULL)
    {
        rt_kprintf("hr_algo: input data point is NULL!\n");
        return;
    }
    if (hr_data->SampleRate == 0)
    {
        //rt_kprintf("hr_algo: input data  is invalid! \n");
        return;
    }

    rt_kprintf("wearstatus= %d\n", hr_data->wearstatus);
#if defined (HR_ALGO_USING_GSENSOR_DATA)
    /* Note: when using this function, make sure that the HR cycle is a multiple of the gsensor cycle, otherwise there will be problems */
    if (g_in)
    {
        gsensors_fifo_t *g_data = (gsensors_fifo_t *)g_in;

        gs_data_len = (g_in->num >= 40) ? 40 : g_in->num;

        //get gsdata & filter
        for (count_gs_data = 0; count_gs_data < gs_data_len; count_gs_data++)
        {
            xData[count_gs_data] = (*(g_in->acce_data[0]) + count_gs_data);
            yData[count_gs_data] = (*(g_in->acce_data[1]) + count_gs_data);
            zData[count_gs_data] = (*(g_in->acce_data[2]) + count_gs_data);
        }
        if (gs_data_len < 40)
        {
            for (count_gs_data_left = count_gs_data; count_gs_data_left < 40; count_gs_data_left++)
            {
                xData[count_gs_data_left] = xData[count_gs_data];
                yData[count_gs_data_left] = yData[count_gs_data];
                zData[count_gs_data_left] = zData[count_gs_data];

            }

        }
        for (count_gs_data = 0; count_gs_data < 20; count_gs_data++)
        {
            //cash_num[40]
            xData[count_gs_data] = xData[cash_num[count_gs_data]] >> 1;
            yData[count_gs_data] = yData[cash_num[count_gs_data]] >> 1;
            zData[count_gs_data] = zData[cash_num[count_gs_data]] >> 1;
        }

    }
#endif

    if (hr_data->vcFifoReadFlag || hr_data->vcPsFlag)
    {
        if (SENSOR_HR == hr_type)
        {
            algoInputData.envSample = hr_data->envValue[0];
            //rt_kprintf("envSample = %d, \n", algoInputData.envSample);
            for (algoCallNum = 0; algoCallNum < 20; algoCallNum++)
            {
                algoInputData.ppgSample = hr_data->ppgValue[algoCallNum];
                //rt_kprintf("ppgSample[%d] = %d\n",algoCallNum, hr_data->ppgValue[algoCallNum]);
                algoInputData.axes.x =  xData[algoCallNum];     //The direction vertical with ARM.
                algoInputData.axes.y =  yData[algoCallNum];     //The direction parallel with ARM.
                algoInputData.axes.z =  zData[algoCallNum];     //The direction upside.
                Algo_Input(&algoInputData, 1000 / hr_data->SampleRate, vcSportMode, 1, 0);
            }


            Algo_Output(&algoOutputData);

            health_value.hr = algoOutputData.hrData;
            HeartRateValue = algoOutputData.hrData;
            rt_kprintf("vcHr02_process (hr) = %d\n", algoOutputData.hrData);

        }
        else if (SENSOR_SPO2 == hr_type)
        {
            for (algoCallNum = 0; algoCallNum < 20; algoCallNum++)  //ppglength = 20
            {
                float vcIrPPG = (float)hr_data->ppgValue[algoCallNum * 2];
                float vcRedPPG = (float)hr_data->ppgValue[algoCallNum * 2 + 1];
                float vcSpo2Value = vcSpo2Calculate(vcRedPPG, vcIrPPG);
                vcSportFlag = vcSportMotionCalculate(xData[algoCallNum], yData[algoCallNum], zData[algoCallNum]);
                //if ((!vcSportFlag) && (vcSpo2Value > 0))
                if (vcSpo2Value > 0)
                {
                    health_value.spo2 = (uint8_t)vcSpo2Value;
                    rt_kprintf("health_value.spo2 = %d\n", health_value.spo2);
                }

            }
        }

    }
    else
    {
        if (SENSOR_HR == hr_type)
        {
            health_value.hr = 0;
            HeartRateValue = 0;
            rt_kprintf("no_process (hr) = %d ; \n", HeartRateValue);
        }
        else if (SENSOR_SPO2 == hr_type)
        {
            health_value.spo2 = 0;
            rt_kprintf("no _process (SPO2) = %f ; \n", health_value.spo2);
        }
    }

    health_value.status = hr_data->wearstatus;
    rt_kprintf("health_state == %d\n",  health_value.status);

    //blood_pressure_algorithm(health_value.hr, &(health_value.bp_h), &(health_value.bp_l));

    return ;

}

void hr_algo_postprocess_data_init(void)
{
    rt_memset(&ctrl_data, 0x00, sizeof(hr_algo_ctrl_data_t));
}

static uint8_t hr_algo_postprocess(uint16_t alg_status)
{
    uint8_t ret = HR_MEAS_NULL;

    if (alg_status == VCHR02WEARST_UNWEAR)
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

    ret = HR_MEAS_OK;

    ctrl_data.hr_timeout = 0;
    ctrl_data.no_touch_num = 0;

end:

    return ret;
}


uint8_t hrs_algo_postprocess(uint8_t hr_type, hrs_info_t *info)
{
    uint8_t ret = HR_MEAS_NULL;
    static uint32_t meas_sum = 0;
    static uint16_t  meas_count = 0;
    static uint16_t  meas_ok_count = 0;

    if (SENSOR_HR == hr_type || SENSOR_SPO2 == hr_type)
    {
        ret = hr_algo_postprocess(health_value.status);
        meas_count++;

        if ((HR_MEAS_OK == ret) && ((health_value.hr > 0) || (health_value.spo2 > 0)))
        {
            meas_ok_count++;
            if (SENSOR_HR == hr_type)
            {
                meas_sum += health_value.hr;
                if (meas_ok_count >= HR_MEAS_SUM_COUNT)
                {
                    info->hr_info.hr = meas_sum / meas_ok_count;
                    rt_kprintf("hr measure ok,hr==%d", info->hr_info.hr);
                    ret = HR_MEAS_SUCC;
                }
            }
            else if (SENSOR_SPO2 == hr_type)
            {
                meas_sum += health_value.spo2;
                if (meas_ok_count >= HR_MEAS_SUM_COUNT)
                {
                    info->spo2_info.spo2 = meas_sum / meas_ok_count;
                    rt_kprintf("mease ok, spo2 = %d ;\n", info->spo2_info.spo2);
                    ret = HR_MEAS_SUCC;
                }
            }
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
