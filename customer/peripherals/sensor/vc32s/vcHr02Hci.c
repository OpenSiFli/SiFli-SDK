
#include "vcHr02Hci.h"
#include <rtthread.h>
#include "drv_i2c.h"

#include "algo.h"
#include "spo2Algo.h"
#include "vcSportMotionAlgo.h"
#include "hr_algo_process.h"

#define DBG_TAG               "vcHr02Hci.c"
#define DBG_LVL               DBG_ERROR//DBG_WARNING//DBG_ERROR
#include <rtdbg.h>
#define Drive_Version  V_1_9_4

static uint32_t lastRtcValue = 0;
static uint8_t  ledCurrent[SLOTCHNU], ledMaxCurrent[SLOTCHNU] = {0x6f, 0x6f, 0x6f};
static uint8_t  pdRes[SLOTCHNU], pdResMax[SLOTCHNU] = {7, 7, 7};
static uint8_t  pdResSet[SLOTCHNU], ppgGain[SLOTCHNU];
static uint8_t  recordEnv[SLOTCHNU] = {0, 0, 0};
static uint16_t count_pdRes[SLOTCHNU], count_QaPdRes[SLOTCHNU];
static uint8_t  adjustFlag[2][64];
static uint8_t  slotCount, sampleIndex, totalSlot = 0, readIndex = 0, readCnt = 0;
static uint8_t  fifoWriteIndex = 0, fifoReadIndex = 0x80, fifoIntDiv = 0;
static uint8_t  enFifo, enSlot[SLOTCHNU];
static uint16_t oscCnt = 0, cntCalOsc = 0;
static uint32_t intDiffRtc[8] = {0}, intDiffAdj[8] = {0};
static uint8_t  oscDifSetting = 2;//oscDifSetting = 2，mean diffrtc <= 2
static bool slot0EnvIsExceedFlag, slot1EnvIsExceedFlag;
static uint8_t psBias = 0;
static bool psBiasReadInPdFlag = false;
extern int HeartRateValue;
static bool flag_first_getsample = 0;

/* Timer clock frequency
   Be used to adjust INT frequency */
//const uint32_t mcuOscData = 30000;
uint32_t mcuOscData = 32768;
/* Heart rate sample frequency */
const uint8_t vcHr02SampleRate = 25;

/* MCU I2C clock frequency */
const uint16_t mcuI2cClock = 400;

#if SportMotionEn
/* SportMotion */
int32_t gsensor_abs_xyz_sum_last_tick;
int32_t gsensor_abs_xyz_sum_current_tick;
int32_t gsensor_abs_xyz_diff_value;


/*
 * @brief
 *
 *
 **/
void green_led_off_state_gsensor_abs_sum_diff_init_func(void)
{
    gsensor_abs_xyz_sum_current_tick = 0;
    gsensor_abs_xyz_sum_last_tick = 0;
    gsensor_abs_xyz_diff_value = 0;
}


/*
 * @brief
 *
 *
 **/
void green_led_off_state_gsensor_abs_sum_diff_func(int32_t x_axis_value, int32_t y_axis_value, int32_t z_axis_value)
{
    int32_t x_axis_value_current = x_axis_value;
    int32_t y_axis_value_current = y_axis_value;
    int32_t z_axis_value_current = z_axis_value;

    gsensor_abs_xyz_sum_last_tick = gsensor_abs_xyz_sum_current_tick;
    if (x_axis_value < 0)
    {
        x_axis_value_current = -x_axis_value;//modify by Tatum
    }
    if (y_axis_value < 0)
    {
        y_axis_value_current = -y_axis_value;
    }
    if (z_axis_value < 0)
    {
        z_axis_value_current = -z_axis_value;
    }

    gsensor_abs_xyz_sum_current_tick = x_axis_value_current + y_axis_value_current + z_axis_value_current;
    if (gsensor_abs_xyz_sum_current_tick && gsensor_abs_xyz_sum_last_tick)
    {
        gsensor_abs_xyz_diff_value = gsensor_abs_xyz_sum_last_tick - gsensor_abs_xyz_sum_current_tick;
    }
    else
    {
        gsensor_abs_xyz_diff_value = 0;
    }
    if (gsensor_abs_xyz_diff_value < 0)
    {
        gsensor_abs_xyz_diff_value = -gsensor_abs_xyz_diff_value;
    }
}
#endif


/*
 * @brief  Get the count value of RTC or general timer of MCU.
 *
 *
 **/
static uint32_t vcHr02GetRtcCountFromMCU(void)
{
    /*------------------INSERT YOUR CODE HERE-----------------*/
    uint32_t tick_count;
    uint32_t freq;
    //uint32_t tick_count = rt_tick_get();
    //tick_count = HAL_GetTick();
    //tick_count = tick_count * 32768 / 1000;
    //rt_kprintf("tick_count = %d\n", tick_count);

    tick_count = HAL_GTIMER_READ();
    freq = HAL_LPTIM_GetFreq();
    if (mcuOscData == 20000)
        tick_count = tick_count * 20000 / 32768; //get 20000 freq tick
    //rt_kprintf("tick_count =%d,freq=%d",tick_count,freq);
    return tick_count;
}


/*
 * @brief
 *      Get the fifoIntDiv and slotNum Configuration
 *
 **/
static void vcHr02ConfigInit(vcHr02_t *pvcHr02)
{
    fifoIntDiv = (pvcHr02->regConfig[3] & 0x3f);
    enFifo = (fifoIntDiv != 0) ? 1 : 0;

    /* Get Slot0 Status */
    enSlot[0] = pvcHr02->regConfig[0] & 0x01;
    /* Get Slot1 Status */
    enSlot[1] = (pvcHr02->regConfig[0] & 0x02) >> 1;

    totalSlot = (pvcHr02->regConfig[0] & 0x03) == 3 ? 2 : ((pvcHr02->regConfig[0] & 0x03) == 0 ? 0 : 1);

    pvcHr02->adjustInfo[0].step = 4;
    pvcHr02->adjustInfo[0].directionLast = VCHR02ADJUSTDIRECTION_NULL;
    pvcHr02->adjustInfo[0].direction = VCHR02ADJUSTDIRECTION_NULL;

    pvcHr02->adjustInfo[1].step = 4;
    pvcHr02->adjustInfo[1].directionLast = VCHR02ADJUSTDIRECTION_NULL;
    pvcHr02->adjustInfo[1].direction = VCHR02ADJUSTDIRECTION_NULL;

    pvcHr02->wearStatus = VCHR02WEARST_ISWEAR;
    pvcHr02->vcPsFlag = 0;
    pvcHr02->vcFifoReadFlag = 0;

    pvcHr02->unWearCnt = VCUNWEAR_CNT;
    pvcHr02->isWearCnt = VCISWEAR_CNT;
}


/*
 * @brief
 *      Initialize vcHr02's register Settings
 *
 **/
vcHr02Ret_t vcHr02RegInit(vcHr02_t *pvcHr02)
{

    uint8_t i = 0;
    uint8_t regConfig[17] = {0x01, 0x3F, 0x8A, 0x40, 0x03, 0x1F, 0x00, 0x00, 0x80, 0x00, 0x57, 0x37, 0x07, 0x16, 0x56, 0x16, 0x00};
    vcHr02Ret_t ret = VCHR02RET_ISOK;



    regConfig[3] = regConfig[3] | (pvcHr02->vcSampleRate - 6);
    regConfig[6] = pvcHr02->vcSampleRate - 6;

#if  VCHR02ISVC31B
    regConfig[9] =  0xE0; //CUR = 80mA//write Hs equal to 1
    regConfig[12] =  0x67;
#elif  VCHR02ISVC32S
    regConfig[9] =  0x50; //CUR = 40mA
    regConfig[12] =  0x17;

#elif  VCHR02ISVC51B
    regConfig[9] =  0x5A; //CUR = 60mA
    regConfig[12] =  0x17; //PDRES = 0x17;
#elif  VCHR02ISVC52S
    regConfig[9] =   0x30; //CUR = 10mA
    regConfig[12] =  0x17; //PDRES = 0x17;
#elif  VCHR02ISVC52T
    regConfig[9] =  0x50; //modify by Tatum，Using our demo for black testing 2020.04.12
    regConfig[12] =  0x37; //PDRES = 0x37;

#else
    regConfig[9] = 0x20;  // CUR = 5mA
    regConfig[12] = 0x17; // PDRES = 0x17;

#endif
    switch (pvcHr02->workMode)
    {
    case VCWORK_MODE_HRWORK:
        regConfig[0] = 0x45;
        break;

    case VCWORK_MODE_SPO2WORK:
        regConfig[0] = 0x47;
        regConfig[1] = 0x2F;
        regConfig[6] = 0;
        regConfig[7] = 0x80;
        regConfig[13] = 0x96;
        regConfig[16] = 0x04; // OverSample
        break;

    case VCWORK_MODE_CROSSTALKTEST:
        regConfig[0] = 0x45;
        regConfig[7] = 0xEF;
        regConfig[10] = 0x77;
        break;

    case VCWORK_MODE_NOISEWORK:
        regConfig[0] = 0x41;
        regConfig[10] = 0x57;
        regConfig[13] = 0x16;
        regConfig[16] = 0x00;
        break;
    case VCWORK_MODE_LPDETECTION:
        regConfig[0]  = 0x44;
        regConfig[1] =  0x30;
        break;
    default:
        break;
    }
#if  VCHR02ISVC31B
    if (psBiasReadInPdFlag == false)
    {
        regConfig[0]  = 0x04;
    }
#endif
    for (i = 0; i < 17; i++)
    {
        pvcHr02->regConfig[i] = regConfig[i];
    }
    ret |= vcHr02WriteRegisters(VCREG11, pvcHr02->regConfig, 17);

    if (1)
    {
        uint8_t value[18] = {0};
        vcHr02ReadRegisters(VCREG11, value, 17);
        rt_kprintf("0x%x,0x%x,0x%x,0x%x\n", value[0], value[1], value[2], value[3]);
        rt_kprintf("0x%x,0x%x,0x%x,0x%x\n", value[4], value[5], value[6], value[7]);
        rt_kprintf("0x%x,0x%x,0x%x,0x%x\n", value[8], value[9], value[10], value[11]);
        rt_kprintf("0x%x,0x%x,0x%x,0x%x\n", value[12], value[13], value[14], value[15]);
        rt_kprintf("0x%x\n", value[16]);
    }

    pvcHr02->regConfig[0] = 0x80 | regConfig[0];
    ret |= vcHr02WriteRegisters(VCREG11, pvcHr02->regConfig, 1);

    vcHr02ConfigInit(pvcHr02);

    return ret;
}


/*
 * @brief:
 *
 **/
vcHr02Ret_t vcHr02StartSample(vcHr02_t *pvcHr02)
{
    vcHr02Ret_t ret = VCHR02RET_ISOK;

    ret |= vcHr02RegInit(pvcHr02);

    return ret;
}


/*
 * @brief
 *      shut down the logic channel
 *      workmode = sleepmode
 *
 **/
vcHr02Ret_t vcHr02StopSample(vcHr02_t *pvcHr02)
{
    /* Shut down all logic solt and goto sleep mode */
    uint8_t modeCtrl = 0x00;
    vcHr02Ret_t ret = VCHR02RET_ISOK;

    ret |= vcHr02WriteRegisters(VCREG11, (uint8_t *)&modeCtrl, VCREG11_SIZE);

    return ret;
}


/*
 * @brief
 * Function: Added adjust flag when interrupt happend.
 *            Provides it for algorithmic processing.
 *
 **/
static vcHr02Ret_t vcHr02AddAdjustFlag(uint8_t slotNum)
{
    vcHr02Ret_t ret = VCHR02RET_ISOK;

    /* If fifoIntDiv is 0,FIFO Function disable */
    if (fifoIntDiv == 0)
    {
        adjustFlag[slotNum][0] = 1;
    }
    else
    {
        ret |= vcHr02ReadRegisters(VCREG3, (uint8_t *)&fifoWriteIndex, VCREG3_SIZE);
        sampleIndex = (fifoWriteIndex - 0x80) / 2 / totalSlot;
        if (sampleIndex == 0)
        {
            sampleIndex = totalSlot == 1 ? 63 : 31;
        }
        else
        {
            sampleIndex = sampleIndex - 1;
        }
        adjustFlag[slotNum][sampleIndex] = 1;
    }
    return ret;
}


/*
* @brief
*      Read data from FIFO
*      If there is an adjustment, an adjustment flag is added
*
**/
static vcHr02Ret_t vcHr02OutPutPPGData(vcHr02_t *pvcHr02, uint8_t StartIndex, uint16_t StopIndex, uint8_t IndexFlag)
{
    uint16_t i = 0;
    uint8_t dataLength = 0;
    uint8_t sampleData[128];
    vcHr02Ret_t ret = VCHR02RET_ISOK;

    dataLength = StopIndex - StartIndex;
    ret |= vcHr02ReadRegisters(StartIndex, sampleData, dataLength);

    for (i = StartIndex; i < StopIndex; i = i + 2)
    {
        pvcHr02->sampleData.ppgValue[(i - StartIndex) / 2 + IndexFlag] = ((sampleData[(i - StartIndex)] << 8) | sampleData[(i - StartIndex) + 1]);
        slotCount = ((i - 0x80) % (totalSlot * 2)) / 2;
        sampleIndex = (i - 0x80) / (totalSlot * 2);
        if (adjustFlag[slotCount][sampleIndex] == 1)
        {
            pvcHr02->sampleData.ppgValue[(i - StartIndex) / 2 + IndexFlag] |= 0x1000;
            adjustFlag[slotCount][sampleIndex] = 0;
        }
    }
    return ret;
}


/*
 * @brief
 *      Clock Calibration
 *
 **/
uint16_t vcHr02CalculateOSCFreq(vcHr02_t *pvcHr02)
{
    uint8_t clkAdjCnt[4] = {0}, newFreDiv[2] = {0}, i = 0;
    uint32_t rtcCount[2];
    uint16_t adjCount[2], freDiv[2] = {0};
    uint16_t newDiv, f20kOsc = 0;

    bool diffCountIsOk = true;
    bool diffIsOK;

    /* Get McuRtcCount */
    rtcCount[0] = vcHr02GetRtcCountFromMCU();
    vcHr02ReadRegisters(VCREG7, clkAdjCnt, 4);
    /* Get McuRtcCount */
    rtcCount[1] = vcHr02GetRtcCountFromMCU();

    adjCount[0] = clkAdjCnt[0] << 8 | clkAdjCnt[1];
    adjCount[1] = clkAdjCnt[2] << 8 | clkAdjCnt[3];
    if (adjCount[1] < adjCount[0] || rtcCount[1] < rtcCount[0])
    {
        cntCalOsc = 0;
        return 0;
    }

    intDiffAdj[2 * cntCalOsc] = adjCount[0];
    intDiffAdj[2 * cntCalOsc + 1] = adjCount[1];

    intDiffRtc[2 * cntCalOsc] = rtcCount[0];
    intDiffRtc[2 * cntCalOsc + 1] = rtcCount[1];

    cntCalOsc += 1;

    if (cntCalOsc >= 3)
    {

        //rt_kprintf("adj[0]=%d,[1]=%d,[2]=%d,[3]=%d,[4]=%d,[5]=%d\n",intDiffAdj[0],intDiffAdj[1],intDiffAdj[2],intDiffAdj[3],intDiffAdj[4],intDiffAdj[5]);
        //rt_kprintf("rtc[0]=%d,[1]=%d,[2]=%d,[3]=%d,[4]=%d,[5]=%d\n",intDiffRtc[0],intDiffRtc[1],intDiffRtc[2],intDiffRtc[3],intDiffRtc[4],intDiffRtc[5]);
        cntCalOsc = 0;
        for (i = 0; i < 2; i++)
        {
            diffCountIsOk = (((intDiffRtc[2 * i + 2] - intDiffRtc[2 * i]) - (intDiffRtc[2 * i + 3] - intDiffRtc[2 * i + 1]) <= oscDifSetting) || ((intDiffRtc[2 * i + 3] - intDiffRtc[2 * i + 1]) - (intDiffRtc[2 * i + 2] - intDiffRtc[2 * i]) <= oscDifSetting)) && diffCountIsOk;
            diffCountIsOk = (((intDiffAdj[2 * i + 2] - intDiffAdj[2 * i]) - (intDiffAdj[2 * i + 3] - intDiffAdj[2 * i + 1]) <= 2) || ((intDiffAdj[2 * i + 3] - intDiffAdj[2 * i + 1]) - (intDiffAdj[2 * i + 2] - intDiffAdj[2 * i]) <= 2)) && diffCountIsOk;
        }
        //rt_kprintf("diffCountIsOk=%d\n",diffCountIsOk);
        //rt_kprintf("pvcHr02->mcuOscValue=%d\n",pvcHr02->mcuOscValue);

        if (diffCountIsOk)
        {
            for (i = 0; i < 2; i++)
            {
                freDiv[i] = ((uint32_t)(intDiffAdj[2 * i + 2] - intDiffAdj[2 * i])) * pvcHr02->mcuOscValue / pvcHr02->vcSampleRate / ((uint32_t)(intDiffRtc[2 * i + 2] - intDiffRtc[2 * i])) - 1;
                //rt_kprintf("freDiv[%d]=%d\n",i,freDiv[i]);
                if (freDiv[i] < (20000 / pvcHr02->vcSampleRate * 0.75f) || freDiv[i] > (20000 / pvcHr02->vcSampleRate * 1.25f))
                {
                    return 0;
                }
            }
            diffIsOK = (freDiv[1] - freDiv[0] <= 20) || (freDiv[0] - freDiv[1] <= 20);
            if (diffIsOK)
            {
                newDiv = (freDiv[1] + freDiv[0]) / 2;
                f20kOsc = (newDiv + 1) * pvcHr02->vcSampleRate;
                newFreDiv[0] = newDiv >> 8;
                newFreDiv[1] = newDiv;
                vcHr02WriteRegisters(VCREG15, newFreDiv, VCREG15_SIZE);
                pvcHr02->oscCheckFinishFlag = 1;
                oscCnt = 0;
            }
        }
        else
        {
            return 0;
        }
    }
    return f20kOsc;
}


/*
 * @brief
 *      Wear monitoring.
 *
 **/
static vcHr02Ret_t vcHr02WearStatusDetection(vcHr02_t *pvcHr02)
{
    uint8_t newSoltChNu = 0;
    uint8_t psDiv1Hz =  pvcHr02->regConfig[3] & 0x3f;
    uint8_t enPs = (pvcHr02->regConfig[1] | 0x10) & 0xf0;
    vcHr02Ret_t ret = VCHR02RET_ISOK;
    //LOG_W("psValue=%d\n",pvcHr02->sampleData.psValue);
    //LOG_W("ENV2=%d\n",pvcHr02->sampleData.envValue[2]);

    switch (pvcHr02->wearStatus)
    {
    /* Identify whether the wearing state needs to be switched,
    Wear status from unWear to isWear */
    case VCHR02WEARST_UNWEAR:
#if SportMotionEn
        if ((pvcHr02->sampleData.psValue >= VCPS_TH) && (pvcHr02->sampleData.envValue[2] < 3) && (gsensor_abs_xyz_diff_value > 20))
#else
        if ((pvcHr02->sampleData.psValue >= VCPS_TH) && (pvcHr02->sampleData.envValue[2] < 3))
#endif
        {
            /* 1 time */
            if (--pvcHr02->isWearCnt <= 0)
            {
                pvcHr02->wearStatus = VCHR02WEARST_ISWEAR;
                pvcHr02->unWearCnt = VCUNWEAR_CNT;
                pvcHr02->isWearCnt = VCISWEAR_CNT;

                /* Reset Reg to defult value */
                vcHr02SoftReset(pvcHr02);

                vcHr02RegInit(pvcHr02);
                ret |= VCHR02RET_UNWEARTOISWEAR;

                lastRtcValue = vcHr02GetRtcCountFromMCU();

            }
        }
        else
        {
            pvcHr02->isWearCnt = VCISWEAR_CNT;
        }
        break;

    case VCHR02WEARST_ISWEAR:
#if BiometricEn
#if (VCHR02ISVC32S || VCHR02ISVC52S || VCHR02ISVC9201)
        if ((pvcHr02->sampleData.psValue < VCPS_TH) || (pvcHr02->sampleData.envValue[2] > VCENV_TH) || (HeartRateValue == -1))
#else
        if ((pvcHr02->sampleData.psValue < VCPS_TH) || (pvcHr02->sampleData.envValue[2] > VCENV_TH) || (slot0EnvIsExceedFlag == true) || (slot1EnvIsExceedFlag == true) || (HeartRateValue == -1))
#endif
        {
            if (HeartRateValue == -1)
            {
                pvcHr02->unWearCnt = 1;
            }
            /* 3 times */
            if (--pvcHr02->unWearCnt <= 0)
            {
                HeartRateValue = 0;
                pvcHr02->wearStatus = VCHR02WEARST_UNWEAR;
                pvcHr02->unWearCnt = VCUNWEAR_CNT;
                pvcHr02->isWearCnt = VCISWEAR_CNT;
                if (pvcHr02->workMode == VCWORK_MODE_SPO2WORK)
                {
                    vcHr02WriteRegisters(VCREG16, (uint8_t *)&psDiv1Hz, VCREG16_SIZE);
                    vcHr02WriteRegisters(VCREG12, (uint8_t *)&enPs, VCREG12_SIZE);
                    pvcHr02->regConfig[1] |= 0x10;
                }
                /* Enable solt2,Disable solt0 and solt1 */
                newSoltChNu = (pvcHr02->regConfig[0] & 0xF8) | 0x04;

                pvcHr02->vcPsFlag = 1;
                ret |= vcHr02WriteRegisters(VCREG11, (uint8_t *)&newSoltChNu, VCREG11_SIZE);
                ret |= VCHR02RET_ISWEARTOUNWEAR;
#if SportMotionEn
                green_led_off_state_gsensor_abs_sum_diff_init_func(); //modify by Tatum
#endif
            }
        }
#else
#if (VCHR02ISVC32S || VCHR02ISVC52S || VCHR02ISVC9201)
        if ((pvcHr02->sampleData.psValue < VCPS_TH) || (pvcHr02->sampleData.envValue[2] > VCENV_TH))
#else
        if ((pvcHr02->sampleData.psValue < VCPS_TH) || (pvcHr02->sampleData.envValue[2] > VCENV_TH) || (slot0EnvIsExceedFlag == true) || (slot1EnvIsExceedFlag == true))
#endif
        {
            /* 3 times */
            if (--pvcHr02->unWearCnt <= 0)
            {
                pvcHr02->wearStatus = VCHR02WEARST_UNWEAR;
                pvcHr02->unWearCnt = VCUNWEAR_CNT;
                pvcHr02->isWearCnt = VCISWEAR_CNT;

                if (pvcHr02->workMode == VCWORK_MODE_SPO2WORK)
                {

                    vcHr02WriteRegisters(VCREG16, (uint8_t *)&psDiv1Hz, VCREG16_SIZE);
                    vcHr02WriteRegisters(VCREG12, (uint8_t *)&enPs, VCREG12_SIZE);
                    pvcHr02->regConfig[1] |= 0x10;
                }
                /* Enable solt2,Disable solt0 and solt1 */
                newSoltChNu = (pvcHr02->regConfig[0] & 0xF8) | 0x04;

                pvcHr02->vcPsFlag = 1;
                ret |= vcHr02WriteRegisters(VCREG11, (uint8_t *)&newSoltChNu, VCREG11_SIZE);
                ret |= VCHR02RET_ISWEARTOUNWEAR;
            }
        }
#endif
        else
        {
            pvcHr02->vcPsFlag = 0;
            pvcHr02->unWearCnt = VCUNWEAR_CNT;
        }
        break;
    }
    return ret;
}


/*
 * @brief
 *      FIFO Read.(Or GetValue)
 *
 **/
static  vcHr02Ret_t vcHr02ReadFifo(vcHr02_t *pvcHr02, uint8_t *len)
{
    uint32_t rtcValue = 0, diffValue = 0;
    uint8_t dateLength = 0;
    float cacheTime = 64.0f, res = 0.0f;
    vcHr02Ret_t ret = VCHR02RET_ISOK;

    ret |= vcHr02ReadRegisters(VCREG3, (uint8_t *)&fifoWriteIndex, VCREG3_SIZE);
    rtcValue = vcHr02GetRtcCountFromMCU();
    //rt_kprintf("rtcValue = %d;lastRtcValue = %d\n",rtcValue,lastRtcValue);
    if (flag_first_getsample)
    {
        lastRtcValue = rtcValue;
        flag_first_getsample = 0;
    }

    if (rtcValue >= lastRtcValue)
        diffValue = rtcValue - lastRtcValue;
    else
        diffValue = (0x1ffff - lastRtcValue) + rtcValue;
    //if ((rtcValue - lastRtcValue) > (((cacheTime / pvcHr02->vcSampleRate) * 0.8f * pvcHr02->mcuOscValue) / totalSlot))
    //res = (((cacheTime / pvcHr02->vcSampleRate) * 0.8f * pvcHr02->mcuOscValue) / totalSlot);
    if (diffValue > (((cacheTime / pvcHr02->vcSampleRate) * 0.8f * pvcHr02->mcuOscValue) / totalSlot))

        //rt_kprintf("diffValue = %d;res = %f\n",diffValue,res);
        //if(diffValue > res)
    {
        pvcHr02->vcFifoReadFlag = 0;
        fifoWriteIndex = 0;
        fifoReadIndex = 0x80;
        lastRtcValue = rtcValue;
        return VCHR02RET_FIFOISOVERFLOW;
    }

    if (fifoIntDiv != 0)
    {
        if (fifoWriteIndex > fifoReadIndex)
        {
            dateLength = (fifoWriteIndex - fifoReadIndex) / 2;
            vcHr02OutPutPPGData(pvcHr02, fifoReadIndex, fifoWriteIndex, 0);
        }
        else
        {
            dateLength = (fifoWriteIndex + 0x80 - fifoReadIndex) / 2;
            if (dateLength == 64)
            {
                dateLength = 0;
                pvcHr02->vcFifoReadFlag = 0;
                return VCHR02RET_ISERR;
            }
            vcHr02OutPutPPGData(pvcHr02, fifoReadIndex, 256, 0);
            if (fifoWriteIndex != 0x80)
            {
                vcHr02OutPutPPGData(pvcHr02, 0x80, fifoWriteIndex, (256 - fifoReadIndex) / 2);
            }
        }
        fifoReadIndex = fifoWriteIndex;
    }
    else
    {
        /* Not use FIFO, the sampleData is storage in 0x80 */
        dateLength = totalSlot;
        vcHr02OutPutPPGData(pvcHr02, fifoReadIndex, fifoReadIndex + totalSlot * 2, 0);
    }

    /* Return the array length for data processing */
    *len = dateLength;
    lastRtcValue = rtcValue;
    return ret;
}



/*
 * @brief
 *      Adjust PDRes Value and modify the PDResMax
 *
 **/
static vcHr02Ret_t vcHr02EnvAdjust(vcHr02_t *pvcHr02, uint8_t slotNum)
{
    vcHr02Ret_t ret = VCHR02RET_ISOK;

    if ((pvcHr02->sampleData.envValue[slotNum] > VCENV_TH) && enSlot[slotNum])
    {
        recordEnv[slotNum] = pvcHr02->sampleData.envValue[slotNum];
        if (pdRes[slotNum] != 0)
        {
            pdRes[slotNum] = pdRes[slotNum] - 1;
            pdResMax[slotNum] = pdRes[slotNum];

            /* Added logical adjustment flag bit */
            vcHr02AddAdjustFlag(slotNum);

            pvcHr02->regConfig[slotNum + 10] = (pdRes[slotNum] << 4) | pdResSet[slotNum];
            ret |= vcHr02WriteRegisters((VCREG20 + slotNum), (uint8_t *) & (pvcHr02->regConfig[slotNum + 10]), 1);
        }
        else
        {
            return VCHR02RET_ENVCURRENTISLARGE;
        }
    }
    if (pvcHr02->sampleData.envValue[2] > VCENV_TH)
    {
        return VCHR02RET_PSENVCURRENTISLARGE;
    }
    return ret;
}


/*
 * @brief
 *      Adjust ledCurrent Value and modify the ledCurrentMax value
 *
 **/
static vcHr02Ret_t vcHr02OvloadAdjust(vcHr02_t *pvcHr02, uint8_t slotNum)
{
    uint8_t overLoad = 0;
    uint8_t ovload[SLOTCHNU];
    vcHr02Ret_t ret = VCHR02RET_ISOK;

    ret |= vcHr02ReadRegisters(VCREG1, (uint8_t *)&overLoad, VCREG1_SIZE);

    /* Get Slot0,Slot1 and Slot2's Overload status */
    ovload[0] = overLoad & 0x01;
    ovload[1] = (overLoad & 0x02) >> 1;
    ovload[2] = (overLoad & 0x04) >> 2;

    /* Adjust led emitter and record max_led_emitter */
    if ((overLoad & (pvcHr02->regConfig[0] & 0x07)) != 0)
    {
        if (ovload[slotNum] && enSlot[slotNum])
        {
            if (ledCurrent[slotNum] != 0)
            {
                ledCurrent[slotNum] = ledCurrent[slotNum] - 1;
                ledMaxCurrent[slotNum] = ledCurrent[slotNum];
                vcHr02AddAdjustFlag(slotNum);
                pvcHr02->regConfig[slotNum + 7] = (ledCurrent[slotNum] | ppgGain[slotNum]);
                ret |= vcHr02WriteRegisters((VCREG17 + slotNum), (uint8_t *) & (pvcHr02->regConfig[slotNum + 7]), 1);
            }
            else
            {
                return VCHR02RET_SOLTISOVERLOAD;
            }
        }
        /* After the calibration of infrared channel parameters,
        the current and resistance will not be adjusted */
        if (ovload[2])
        {
            return VCHR02RET_PSISOVERLOAD;
        }
    }
    return ret;
}


/*
 * @brief
 *      Adjust ledCurrent Value and PDRes value
 *
 **/
static vcHr02Ret_t vcHr02PPgLedCurAndPdResAdjust(vcHr02_t *pvcHr02, uint8_t slotNum, uint8_t ledcur, uint8_t pdres)
{
    uint8_t  oldPdRes, oldLedCurrent;
    uint8_t  newPdRes, newLedCurrent;
    vcHr02Ret_t ret = VCHR02RET_ISOK;

    oldLedCurrent = ledCurrent[slotNum];
    newLedCurrent = oldLedCurrent;
    oldPdRes = pdRes[slotNum];
    newPdRes = oldPdRes;

    slot0EnvIsExceedFlag = false;
    slot1EnvIsExceedFlag = false;
    if ((oldLedCurrent == ledcur) && (pvcHr02->workMode != VCWORK_MODE_NOISEWORK))
    {
        if (oldPdRes == pdres)
        {
            newPdRes = oldPdRes ;
            slot0EnvIsExceedFlag = (slotNum == 0) ? true : false;
            slot1EnvIsExceedFlag = (slotNum == 1) ? true : false;
            return VCHR02RET_PPGCANNOTADJUSTABLE;
        }
        else
        {
            if (pvcHr02->adjustInfo[slotNum].direction == VCHR02ADJUSTDIRECTION_UP)
            {
                newPdRes = (oldPdRes >= 7) ? 7 : (oldPdRes + 1);
            }
            else
            {
                newPdRes = (oldPdRes < 1) ? 0 : (oldPdRes - 1);
            }
            newLedCurrent = oldLedCurrent;
            vcHr02AddAdjustFlag(slotNum);
        }
    }
    else
    {
        if (pvcHr02->adjustInfo[slotNum].directionLast == VCHR02ADJUSTDIRECTION_NULL)
        {
            pvcHr02->adjustInfo[slotNum].step *= 16;
        }
        else if (pvcHr02->adjustInfo[slotNum].direction == pvcHr02->adjustInfo[slotNum].directionLast)
        {
            if ((pvcHr02->adjustInfo[slotNum].step == 1) || (pvcHr02->adjustInfo[slotNum].step == 2))
            {
                pvcHr02->adjustInfo[slotNum].step = (pvcHr02->adjustInfo[slotNum].step + 1) * 16;
            }
            else
            {
                pvcHr02->adjustInfo[slotNum].step *= VCADJUST_INCREASE;
            }
        }
        else
        {
            pvcHr02->adjustInfo[slotNum].step *= VCADJUST_DECREASE;
        }

        pvcHr02->adjustInfo[slotNum].step = pvcHr02->adjustInfo[slotNum].step >> 4;

        pvcHr02->adjustInfo[slotNum].step = (pvcHr02->adjustInfo[slotNum].step <= VCADJUST_STEP_MIN) ? VCADJUST_STEP_MIN : pvcHr02->adjustInfo[slotNum].step;
        pvcHr02->adjustInfo[slotNum].step = (pvcHr02->adjustInfo[slotNum].step >= VCADJUST_STEP_MAX) ? VCADJUST_STEP_MAX : pvcHr02->adjustInfo[slotNum].step;

        if (pvcHr02->adjustInfo[slotNum].direction == VCHR02ADJUSTDIRECTION_UP)
        {
            newLedCurrent = ((oldLedCurrent + pvcHr02->adjustInfo[slotNum].step) > ledcur) ? ledcur : oldLedCurrent + pvcHr02->adjustInfo[slotNum].step;
        }
        else
        {
            newLedCurrent = (oldLedCurrent < pvcHr02->adjustInfo[slotNum].step) ? ledcur : oldLedCurrent - pvcHr02->adjustInfo[slotNum].step;
        }
        vcHr02AddAdjustFlag(slotNum);
        newPdRes = oldPdRes;
        pvcHr02->adjustInfo[slotNum].directionLast = pvcHr02->adjustInfo[slotNum].direction;
    }

    ledCurrent[slotNum] = newLedCurrent;
    pdRes[slotNum] = newPdRes;

    pvcHr02->regConfig[slotNum + 7] = ledCurrent[slotNum] | ppgGain[slotNum];
    ret |= vcHr02WriteRegisters((VCREG17 + slotNum), (uint8_t *) & (pvcHr02->regConfig[slotNum + 7]), 1);
    pvcHr02->regConfig[slotNum + 10] = (pdRes[slotNum] << 4) | pdResSet[slotNum];
    ret |= vcHr02WriteRegisters((VCREG20 + slotNum), (uint8_t *) & (pvcHr02->regConfig[slotNum + 10]), 1);

    return ret;
}


/*
 * @brief
 *      Get the PPG saturation direction
 *
 **/
static vcHr02Ret_t vcHr02PpgAdjust(vcHr02_t *pvcHr02, uint8_t slotNum)
{
    uint8_t  ppgDataAddress, ppgDataAddress_EnFifo;
    uint8_t  ppgDate[2];
    uint16_t ppgCurValue[2];
    uint8_t  IndexGap;
    vcHr02Ret_t ret = VCHR02RET_ISOK;

    /* Get fifoWriteIndex Value */
    ret |= vcHr02ReadRegisters(VCREG3, (uint8_t *)&fifoWriteIndex, VCREG3_SIZE);

    if (totalSlot == 1)
    {
        /* Determine whether the FIFO is overflowing */
        ppgDataAddress_EnFifo = fifoWriteIndex - totalSlot * 2;
        ppgDataAddress_EnFifo = ppgDataAddress_EnFifo < 0x80 ? ppgDataAddress_EnFifo + 0x80 : ppgDataAddress_EnFifo;
        ppgDataAddress = enFifo ? ppgDataAddress_EnFifo : 0x80;
    }
    else
    {
        /* Determine whether the FIFO is overflowing */
        ppgDataAddress_EnFifo = fifoWriteIndex - totalSlot * 2 + slotNum * 2;
        ppgDataAddress_EnFifo = ppgDataAddress_EnFifo < 0x80 ? ppgDataAddress_EnFifo + 0x80 : ppgDataAddress_EnFifo;
        ppgDataAddress = enFifo ? ppgDataAddress_EnFifo : 0x80 + slotNum * 2;
    }
    ret |= vcHr02ReadRegisters(ppgDataAddress, ppgDate, 2);
    ppgCurValue[slotNum] = (ppgDate[0] << 8) | ppgDate[1];

    if (fifoIntDiv != 0)
    {
        IndexGap = (fifoWriteIndex > readIndex) ? (fifoWriteIndex - readIndex) : (fifoWriteIndex - readIndex + 0x80);
        if (IndexGap != 2 * totalSlot)
        {
            pvcHr02->adjustInfo[slotNum].directionLast = VCHR02ADJUSTDIRECTION_NULL;
        }
        readCnt += 1;
        if (readCnt == totalSlot)
        {
            readCnt = 0;
            readIndex = fifoWriteIndex;
        }
    }
    /* PPG supersaturation */
    if (ppgCurValue[slotNum] > 4095 - VCPPG_TH * 32)
    {
        /* If the LED luminous current reaches the maximum current,
        it can only be adjusted by increasing the PD resistance */
        pvcHr02->adjustInfo[slotNum].direction = VCHR02ADJUSTDIRECTION_UP;
        ret |= vcHr02PPgLedCurAndPdResAdjust(pvcHr02, slotNum, ledMaxCurrent[slotNum], pdResMax[slotNum]);
    }
    /* Lower saturation */
    else if (ppgCurValue[slotNum] < VCPPG_TH * 32)
    {
        pvcHr02->adjustInfo[slotNum].direction = VCHR02ADJUSTDIRECTION_DOWN;
        ret |= vcHr02PPgLedCurAndPdResAdjust(pvcHr02, slotNum, 0, 0);
    }
    return ret;
}


/*
 * @brief
 *      Prejude the ENVvalue and modify the PD maximum resistance.
 *      Don't need to write to register
 *
 **/
static vcHr02Ret_t vcHr02AdjustPDResMax(vcHr02_t *pvcHr02, uint8_t slotNum)
{
    uint8_t minEnv[SLOTCHNU];
    vcHr02Ret_t ret = VCHR02RET_ISOK;

    if (pdResMax[slotNum] == 7)
    {
        return VCHR02RET_PDRESISMAX;
    }
    minEnv[slotNum] = ((recordEnv[slotNum] / 2) < VCENV_TH) ? (recordEnv[slotNum] / 2) : VCENV_TH;
    if (pvcHr02->sampleData.envValue[slotNum] < minEnv[slotNum])
    {
        if (pvcHr02->sampleData.envValue[slotNum] < 1)
        {
            count_pdRes[slotNum] = 0;
            count_QaPdRes[slotNum] = (count_QaPdRes[slotNum] == 30 * pvcHr02->vcSampleRate) ? 0 : count_QaPdRes[slotNum] + 1;
            pdResMax[slotNum] = (count_QaPdRes[slotNum] == 30 * pvcHr02->vcSampleRate) ? 7 : pdResMax[slotNum];
        }
        else
        {
            count_QaPdRes[slotNum] = 0;
            count_pdRes[slotNum] += 1;
            pdResMax[slotNum] = (count_pdRes[slotNum] == 120 * pvcHr02->vcSampleRate) ? pdResMax[slotNum] + 1 : pdResMax[slotNum];
        }
    }
    else
    {
        count_QaPdRes[slotNum] = 0;
        count_pdRes[slotNum] = 0;
    }
    return ret;
}


/*
 * @brief
 *      Before getsample value,prejude Status.
 *
 **/
static vcHr02Ret_t vcHr02ReadStatus(vcHr02_t *pvcHr02)
{
    uint8_t vcHr02SlotCnt = 0;
    vcHr02Ret_t ret = VCHR02RET_ISOK;

    /* Clear FifoReadFlag */
    pvcHr02->vcFifoReadFlag = 0;

    if (pvcHr02->vcHr02Status & VCREG1_CONFLICT)
    {
        return VCHR02RET_ISCONFLICT;
    }
    if (pvcHr02->vcHr02Status & VCREG1_INSAMPLE)
    {
        return VCHR02RET_ISINSAMPLE;
    }

#if (VCWEAR_STATUS_DETECTION_MODE != VCWEAR_STATUS_DETECTION_OFF)
    if (pvcHr02->regConfig[1] & 0x10)
    {
        if (pvcHr02->intReason & 0x10)
        {
            ret |= vcHr02WearStatusDetection(pvcHr02);
        }
    }
#endif

    if ((ret == VCHR02RET_UNWEARTOISWEAR) && (pvcHr02->wearStatus == VCHR02WEARST_ISWEAR))
    {
        //Set psflag to 1 when the state is switched (from unwear to iswear)
        //to prevent psflag and fifoflag from being 0 at the same time
        pvcHr02->vcPsFlag = 1;
    }
    if ((ret != VCHR02RET_UNWEARTOISWEAR) && (pvcHr02->wearStatus == VCHR02WEARST_ISWEAR))
    {
        //Clear the psflag that appears after the state switch (from unwear to iswear)
        pvcHr02->vcPsFlag = 0;
    }
    for (vcHr02SlotCnt = 0; vcHr02SlotCnt < totalSlot; vcHr02SlotCnt++)
    {
        slotCount = (totalSlot == 2) ? vcHr02SlotCnt : (enSlot[0] ? 0 : 1);
        vcHr02AdjustPDResMax(pvcHr02, slotCount);
        ledCurrent[slotCount] = pvcHr02->regConfig[7 + slotCount] & 0x7f;
        ppgGain[slotCount] = pvcHr02->regConfig[7 + slotCount] & 0x80;
        pdRes[slotCount] = (pvcHr02->regConfig[10 + slotCount] & 0x70) >> 4;
        pdResSet[slotCount] = pvcHr02->regConfig[10 + slotCount] & 0x8F;
        //rt_kprintf("945>intReason =0x%x\n",pvcHr02->intReason);

        if (pvcHr02->intReason & VCREG1_INTENV)
        {
            ret |= vcHr02EnvAdjust(pvcHr02, slotCount);
        }
        if (pvcHr02->intReason & VCREG1_INTOV)
        {
            ret |= vcHr02OvloadAdjust(pvcHr02, slotCount);
        }
        if (pvcHr02->intReason & VCREG1_INTPPG)
        {
            ret |= vcHr02PpgAdjust(pvcHr02, slotCount);
        }
        else
        {
            pvcHr02->adjustInfo[slotCount].directionLast = VCHR02ADJUSTDIRECTION_NULL;
            pvcHr02->adjustInfo[slotCount].direction = VCHR02ADJUSTDIRECTION_NULL;
        }
    }
    if (pvcHr02->intReason & VCREG1_INTFIFO) //中断原因寄存器
    {
        if (pvcHr02->workMode == VCWORK_MODE_SPO2WORK)
        {
#if (VCWEAR_STATUS_DETECTION_MODE != VCWEAR_STATUS_DETECTION_OFF)
            ret |= vcHr02WearStatusDetection(pvcHr02);
#endif
        }
        pvcHr02->vcFifoReadFlag = 1;
    }
    return ret;
}


/*
 * @brief
 *      Read Pre and EnvValue
 *
 **/
static vcHr02Ret_t vcHr02ReadReg(vcHr02_t *pvcHr02)
{
    uint8_t vcRegisterValue[6] = {0};
    vcHr02Ret_t ret = VCHR02RET_ISOK;

    ret |= vcHr02ReadRegisters(VCREG1, vcRegisterValue, 6);

    pvcHr02->vcHr02Status = vcRegisterValue[0];
    pvcHr02->intReason = vcRegisterValue[1];

    pvcHr02->sampleData.envValue[0] = vcRegisterValue[3] >> 4;
    pvcHr02->sampleData.preValue[0] = vcRegisterValue[3] & 0x0F;
    pvcHr02->sampleData.envValue[1] = vcRegisterValue[4] >> 4;
    pvcHr02->sampleData.preValue[1] = vcRegisterValue[4] & 0x0F;
    pvcHr02->sampleData.envValue[2] = vcRegisterValue[5] >> 4;
    pvcHr02->sampleData.psValue = vcRegisterValue[5] & 0x0F;

    ret |= vcHr02ReadRegisters(VCREG17, vcRegisterValue, 6);
    pvcHr02->sampleData.pdResValue[0] = (vcRegisterValue[3] >> 4) & 0x07;
    pvcHr02->sampleData.currentValue[0] = vcRegisterValue[0] & 0x7F;
    pvcHr02->sampleData.pdResValue[1] = (vcRegisterValue[4] >> 4) & 0x07;
    pvcHr02->sampleData.currentValue[1] = vcRegisterValue[1] & 0x7F;
    pvcHr02->sampleData.pdResValue[2] = (vcRegisterValue[5] >> 4) & 0x07;
    pvcHr02->sampleData.currentValue[2] = vcRegisterValue[2] & 0x7F;

#if VCHR02ISVC31B
    if ((!psBiasReadInPdFlag) && (pvcHr02->intReason & 0x10))
    {
        psBias = pvcHr02->sampleData.psValue / 2;
        psBiasReadInPdFlag = true;
        vcHr02SoftReset(pvcHr02);
        vcHr02RegInit(pvcHr02);

    }
    pvcHr02->sampleData.psValue = (pvcHr02->sampleData.psValue > psBias) ? (pvcHr02->sampleData.psValue - psBias) : 0;
#endif

    return ret;
}


/*
 * @brief
 *
 *
 **/
vcHr02Ret_t  vcHr02GetSampleValues(vcHr02_t *pvcHr02, uint8_t *DataLen)
{
    vcHr02Ret_t ret = VCHR02RET_ISOK;
    uint16_t ret1;

    int16_t secondCnt = pvcHr02->vcSampleRate * 180 / (fifoIntDiv + 1); //3min
    //int16_t secondCnt = pvcHr02->vcSampleRate*10/(fifoIntDiv+1);//10s

    vcHr02ReadReg(pvcHr02);

    switch (pvcHr02->workMode)
    {
    case VCWORK_MODE_HRWORK:
        ret |= vcHr02ReadStatus(pvcHr02);
        //rt_kprintf("vcHr02ReadStatus oscCheckFinishFlag=%d\n", pvcHr02->oscCheckFinishFlag);

        if (!pvcHr02->oscCheckFinishFlag)
        {
            ret1 = vcHr02CalculateOSCFreq(pvcHr02);
            rt_kprintf("<1034>f20kOsc =%d\n", ret1);

            // pvcHr02->vcFifoReadFlag = 0;
        }
        if (pvcHr02->vcFifoReadFlag == 1)
        {
            oscCnt += 1;
            if (oscCnt >= secondCnt)
            {
                /* Consider the 20k Adjust to three mintures called */
                ret1 = vcHr02CalculateOSCFreq(pvcHr02);
                rt_kprintf("<1045>f20kOsc =%d\n", ret1);
            }
            if (pvcHr02->oscCheckFinishFlag)
            {
#if (VCWEAR_READ_FIFO_MODE == VCWEAR_READ_FIFO_BY_INT)
                ret |= vcHr02ReadFifo(pvcHr02, DataLen);
                //rt_kprintf("vcHr02ReadFifo vcHr02Ret_t = %d\n", ret);
#endif
            }
        }
        break;

    case VCWORK_MODE_SPO2WORK:
        ret |= vcHr02ReadStatus(pvcHr02);
        if (!pvcHr02->oscCheckFinishFlag)
        {
            vcHr02CalculateOSCFreq(pvcHr02);
            // pvcHr02->vcFifoReadFlag = 0;
        }
        if (pvcHr02->vcFifoReadFlag == 1)
        {
            oscCnt += 1;
            if (oscCnt >= secondCnt)
            {
                /* Consider the 20k Adjust to three mintures called */
                vcHr02CalculateOSCFreq(pvcHr02);
            }
            if (pvcHr02->oscCheckFinishFlag)
            {
#if (VCWEAR_READ_FIFO_MODE == VCWEAR_READ_FIFO_BY_INT)
                ret |= vcHr02ReadFifo(pvcHr02, DataLen);
                //rt_kprintf("vcHr02ReadFifo vcHr02Ret_t = %d\n", ret);
#endif
            }

        }
        break;

    case VCWORK_MODE_CROSSTALKTEST:
        ledCurrent[0] = pvcHr02->regConfig[7] & 0x7f;
        if (pvcHr02->intReason & VCREG1_INTOV)
        {
            ret |= vcHr02OvloadAdjust(pvcHr02, 0);
        }
        pvcHr02->sampleData.maxLedCur = ledMaxCurrent[0];
        /* PASS: */
//          ((pvcHr02->sampleData.maxLedCur >= 100) && (pvcHr02->sampleData.preValue[0] <= 2)
//          /*Monitor ps Value*/
//           read pvcHr02->sampleData.psValue
        break;

    case VCWORK_MODE_NOISEWORK:
        ret |= vcHr02ReadStatus(pvcHr02);
#if (VCWEAR_READ_FIFO_MODE == VCWEAR_READ_FIFO_BY_INT)
        ret |= vcHr02ReadFifo(pvcHr02, DataLen);
#endif
        break;

    case VCWORK_MODE_LPDETECTION:
        if (!pvcHr02->oscCheckFinishFlag)
        {
            vcHr02CalculateOSCFreq(pvcHr02);
        }
        oscCnt += 1;
        if (oscCnt >= secondCnt)
        {
            /* Consider the 20k Adjust to five mintures called */
            vcHr02CalculateOSCFreq(pvcHr02);
        }
        ret |= vcHr02WearStatusDetection(pvcHr02);
        pvcHr02->vcPsFlag = 1;
        break;

    default:
        break;
    }
    //rt_kprintf("vcHr02GetSampleValues vcHr02Ret_t = %d\n", ret);
    return ret;
}


/*
 * @brief
 *
 *
 **/
vcHr02Ret_t  vcHr02TimeReadValues(vcHr02_t *pvcHr02, uint8_t *DataLen)
{
    vcHr02Ret_t ret = VCHR02RET_ISOK;

    /* Obtian the env and prevalue */
    vcHr02ReadReg(pvcHr02);

#if (VCWEAR_STATUS_DETECTION_MODE != VCWEAR_STATUS_DETECTION_OFF)
    ret |= vcHr02WearStatusDetection(pvcHr02);
#endif

    oscCnt += 1;
    if (oscCnt >= 180)
    {
        vcHr02CalculateOSCFreq(pvcHr02);
    }
    if (pvcHr02->oscCheckFinishFlag)
    {
        ret |= vcHr02ReadFifo(pvcHr02, DataLen);
    }
    return ret;
}


/*
 * @brief
 *      Soft reset to make all registor
 *      Write 8'h5A to 8'h3B cause soft reset.
 **/
vcHr02Ret_t vcHr02SoftReset(vcHr02_t *pvcHr02)
{
    uint8_t Reset = 0x5A;
    vcHr02Ret_t ret = VCHR02RET_ISOK;

    ret |= vcHr02WriteRegisters(VCREG10, (uint8_t *)&Reset, VCREG10_SIZE);
    oscCnt = 0;
    pvcHr02->oscCheckFinishFlag = 0;
    fifoWriteIndex = 0;
    fifoReadIndex = 0x80;
    return ret;
}

void vcHr02Init(vcHr02_t *pVcHr02)
{
    vcHr02StopSample(pVcHr02);
    vcHr02SoftReset(pVcHr02);
    vcSportMotionAlgoTerminate();

    if (HAL_PMU_LXT_DISABLED())
        mcuOscData = 10000; //RC10
    else
        mcuOscData = 20000;//32768;//32768; //32.678晶振

    rt_kprintf("mcuOscData=%d \n", mcuOscData);
    pVcHr02->vcSampleRate = vcHr02SampleRate;
    pVcHr02->mcuOscValue = mcuOscData;
    pVcHr02->mcuSclRate = mcuI2cClock;
    flag_first_getsample = 1;


    /* HeartRate Mode */
    if (pVcHr02->workMode == VCWORK_MODE_HRWORK)
    {
        Algo_Init();
    }
    /* Spo2 Mode */
    else if (pVcHr02->workMode == VCWORK_MODE_SPO2WORK)
    {
        vcSportMotionAlgoInit();
        vcSpo2AlgoInit();
    }
    /* Light Crosstalk Test Mode */
    else if (pVcHr02->workMode == VCWORK_MODE_LPDETECTION)
    {
        Algo_Init();
    }
    /* Light Crosstalk Test Mode */

    hr_algo_postprocess_data_init();

}


