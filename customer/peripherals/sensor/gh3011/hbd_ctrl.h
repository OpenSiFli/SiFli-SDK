/*
*File Name: hbd_ctrl.h
************************************************************************
*
* Copyright 2018 - 2020 All rights reserved. Goodix Inc.
*
* Description: GH30x sensor control library header.
* Note:
* Identifier abbreviation:  Wnd - Window
*                           Msg - Message
**********************************************************************/

#ifndef __HBD_CTRL_H__
#define __HBD_CTRL_H__

#ifndef __HBD_HB_ENABLE__
    #define __HBD_HB_ENABLE__ (1)
#endif
#ifndef __HBD_HRV_ENABLE__
    #define __HBD_HRV_ENABLE__ (1)
#endif
#ifndef __HBD_BP_ENABLE__
    #define __HBD_BP_ENABLE__ (1)
#endif
#ifndef __HBD_PFA_ENABLE__
    #define __HBD_PFA_ENABLE__ (1)
#endif
#ifndef __HBD_SPO2_ENABLE__
    #define __HBD_SPO2_ENABLE__ (1)
#endif
#ifndef __HBD_HSM_ENABLE__
    #define __HBD_HSM_ENABLE__ (1)
#endif

typedef unsigned char GU8;
typedef signed char GS8;
typedef unsigned short GU16;
typedef signed short GS16;
typedef long int GS32;
typedef unsigned long int GU32;
typedef float GF32;

/* Hbd functiional state enum. */
typedef enum
{
    HBD_FUNCTIONAL_STATE_DISABLE = 0,
    HBD_FUNCTIONAL_STATE_ENABLE = 1,
} EM_HBD_FUNCTIONAL_STATE;

/* Int status enum. */
enum EM_INT_STATUS
{
    INT_STATUS_CHIP_RESET = 0,
    INT_STATUS_NEW_DATA,
    INT_STATUS_FIFO_WATERMARK,
    INT_STATUS_FIFO_FULL,
    INT_STATUS_WEAR_DETECTED,
    INT_STATUS_UNWEAR_DETECTED,
    INT_STATUS_INVALID,
} ;

/* I2c Low Two bit select enum. */
enum EM_HBD_I2C_ID_SEL
{
    HBD_I2C_ID_SEL_1L0L = 0,
    HBD_I2C_ID_SEL_1L0H,
    HBD_I2C_ID_SEL_1H0L,
    HBD_I2C_ID_SEL_1H0H,
    HBD_I2C_ID_INVALID,
};

/* Led pd select enum. */
typedef enum
{
    HBD_LED_PD_SEL_INTERNAL = 0,
    HBD_LED_PD_SEL_EXTERNAL,
} EM_HBD_LED_PD_SEL;

/* Led logic channel map to hw led enum. */
typedef enum
{
    HBD_LED_LOGIC_CHANNEL_MAP_PHY012 = 0,
    HBD_LED_LOGIC_CHANNEL_MAP_PHY021,
    HBD_LED_LOGIC_CHANNEL_MAP_PHY102,
    HBD_LED_LOGIC_CHANNEL_MAP_PHY120,
    HBD_LED_LOGIC_CHANNEL_MAP_PHY201,
    HBD_LED_LOGIC_CHANNEL_MAP_PHY210,
} EM_HBD_LED_LOGIC_CHANNEL_MAP;

/* sample time enum. */
typedef enum
{
    HBD_SAMPLE_TIME_32CYCLES = 0,
    HBD_SAMPLE_TIME_64CYCLES,
    HBD_SAMPLE_TIME_128CYCLES,
    HBD_SAMPLE_TIME_256CYCLES,
    HBD_SAMPLE_TIME_512CYCLES,
    HBD_SAMPLE_TIME_1024CYCLES,
    HBD_SAMPLE_TIME_2048CYCLES,
} EM_HBD_SAMPLE_TIME;

/* tia gain enum. */
typedef enum
{
    HBD_TIA_GAIN_0 = 0,
    HBD_TIA_GAIN_1,
    HBD_TIA_GAIN_2,
    HBD_TIA_GAIN_3,
    HBD_TIA_GAIN_4,
    HBD_TIA_GAIN_5,
    HBD_TIA_GAIN_6,
    HBD_TIA_GAIN_7,
} EM_HBD_TIA_GAIN;

/* G-sensor sensitivity(counts/g) enum. */
typedef enum
{
    HBD_GSENSOR_SENSITIVITY_512_COUNTS_PER_G = 0,
    HBD_GSENSOR_SENSITIVITY_1024_COUNTS_PER_G,
    HBD_GSENSOR_SENSITIVITY_2048_COUNTS_PER_G,
    HBD_GSENSOR_SENSITIVITY_4096_COUNTS_PER_G,
    HBD_GSENSOR_SENSITIVITY_8192_COUNTS_PER_G,
} EM_HBD_GSENSOR_SENSITIVITY;

/* Hb config struct type. */
typedef struct
{
    EM_HBD_FUNCTIONAL_STATE emHbModeFifoEnable;     // HB fifo enable flag
    EM_HBD_FUNCTIONAL_STATE emHrvModeFifoEnable;    // HRV fifo enable flag
    EM_HBD_FUNCTIONAL_STATE emHsmModeFifoEnable;     // HB fifo enable flag
    EM_HBD_FUNCTIONAL_STATE emBpdModeFifoEnable;     // HB fifo enable flag
    EM_HBD_FUNCTIONAL_STATE emPfaModeFifoEnable;     // HB fifo enable flag
    EM_HBD_FUNCTIONAL_STATE emSpo2ModeFifoEnable;     // Spo2 fifo enable flag
} ST_HB_CONFIG_TYPE;

/* Adt config struct type. Notes: only logic channel0 & channel1 valid in adt mode. */
typedef struct
{
    EM_HBD_FUNCTIONAL_STATE emGINTEnable;                  // GINT enable flag
    EM_HBD_LED_LOGIC_CHANNEL_MAP emLedLogicChannelMap;     // Logic map
    EM_HBD_LED_PD_SEL emLogicChannel0PDSelect;             // Logic Channel0 PD select
    EM_HBD_LED_PD_SEL emLogicChannel1PDSelect;             // Logic Channel1 PD select
    GU8 uchLogicChannel0Current;                           // Logic Channel0 current cofig(1 step = 400uA)
    GU8 uchLogicChannel1Current;                           // Logic Channel1 current cofig(1 step = 400uA)
    EM_HBD_TIA_GAIN emLogicChannel0TiaGain;                // Logic Channel0 tia gain(valid value:0-7)
    EM_HBD_TIA_GAIN emLogicChannel1TiaGain;                // Logic Channel1 tia gain(valid value:0-7)
    EM_HBD_SAMPLE_TIME emSampleTime;                       // Sample time
} ST_ADT_CONFIG_TYPE;

/* Hbd init config struct type. */
typedef struct
{
    ST_HB_CONFIG_TYPE stHbInitConfig;
    ST_ADT_CONFIG_TYPE stAdtInitConfig;
} ST_HBD_INIT_CONFIG_TYPE;

/* Gsensor data struct type. */
typedef struct
{
    GS16 sXAxisVal; // X-Axis Value
    GS16 sYAxisVal; // Y-Axis Value
    GS16 sZAxisVal; // Z-Axis Value
} ST_GS_DATA_TYPE;

/* register struct. */
typedef struct
{
    GU16 usRegAddr;
    GU16 usRegData;
} ST_REGISTER;

/* autoled  channel struct. */
typedef struct
{
    GU32 unTrigerUpperThr;
    GU32 unTrigerLowerThr;

    GU32 unTargetUpperThr;
    GU32 unTargetLowerThr;
    GU32 unLightLeakLowerThr;
    EM_HBD_FUNCTIONAL_STATE emAutoledEnable;
} ST_AUTO_LED_CH_CONFIG;

/* Hbd function return code definitions list. */
#define HBD_RET_OK                          (0)                /**< There is no error */
#define HBD_RET_GENERIC_ERROR               (-1)               /**< A generic error happens */
#define HBD_RET_PARAMETER_ERROR             (-2)               /**< Parameter error */
#define HBD_RET_COMM_NOT_REGISTERED_ERROR   (-3)               /**< I2c/Spi communication interface not registered error */
#define HBD_RET_COMM_ERROR                  (-4)               /**< I2c/Spi Communication error */
#define HBD_RET_RESOURCE_ERROR              (-5)               /**< Resource full or not available error */
#define HBD_RET_NO_INITED_ERROR             (-6)               /**< No inited error */
#define HBD_RET_LED_CONFIG_ALL_OFF_ERROR    (-7)               /**< Led config all off error */

/******* hbd init default config ****************
 * stHbInitConfig = { HBD_FUNCTIONAL_STATE_ENABLE,          /// hb mode fifo enable
 *                    HBD_FUNCTIONAL_STATE_ENABLE           /// hrv mode fifo enable
 *                    HBD_FUNCTIONAL_STATE_ENABLE           /// hsm mode fifo enable
 *                    HBD_FUNCTIONAL_STATE_ENABLE           /// bpd mode fifo enable
 *                    HBD_FUNCTIONAL_STATE_ENABLE           /// pfa mode fifo enable
 *                  }
 * stAdtInitConfig = { HBD_FUNCTIONAL_STATE_DISABLE,        /// GINT disable
 *                     HBD_LED_LOGIC_CHANNEL_MAP_PHY012,    /// Logic map
 *                     HBD_LED_PD_SEL_INTERNAL,             /// Logic Channel0 PD select
 *                     HBD_LED_PD_SEL_INTERNAL,             /// Logic Channel1 PD select
 *                     0x19,                                /// Logic Channel0 current 10mA (1 step = 400uA)
 *                     0x19,                                /// Logic Channel1 current 10mA (1 step = 400uA)
 *                     HBD_TIA_GAIN_3,                      /// Logic Channel0 tia gain
 *                     HBD_TIA_GAIN_3,                      /// Logic Channel1 tia gain
 *                     HBD_SAMPLE_TIME_128CYCLES,           /// sample time
 *                   }
 */
#define HBD_INIT_CONFIG_DEFAULT_DEF(var) ST_HBD_INIT_CONFIG_TYPE var={\
                                                                      {HBD_FUNCTIONAL_STATE_ENABLE,\
                                                                      HBD_FUNCTIONAL_STATE_ENABLE,\
                                                                      HBD_FUNCTIONAL_STATE_ENABLE,\
                                                                      HBD_FUNCTIONAL_STATE_ENABLE,\
                                                                      HBD_FUNCTIONAL_STATE_ENABLE,\
                                                                      HBD_FUNCTIONAL_STATE_ENABLE},\
                                                                      {HBD_FUNCTIONAL_STATE_DISABLE,\
                                                                      HBD_LED_LOGIC_CHANNEL_MAP_PHY012, \
                                                                      HBD_LED_PD_SEL_INTERNAL, \
                                                                      HBD_LED_PD_SEL_INTERNAL,\
                                                                      0x19, 0x19, HBD_TIA_GAIN_3, HBD_TIA_GAIN_3, HBD_SAMPLE_TIME_128CYCLES}\
                                                                    }


/****************************************************************
* Description: set i2c operation function
* Input:  uchI2cIdLowTwoBitsSelect: i2c low two bits addr selected, see EM_HBD_I2C_ID_SEL
          pI2cWriteFunc: i2c write function pointer
          pI2cReadFunc: i2c read function pointer
* Return: HBD_RET_PARAMETER_ERROR: function pointer parameter set null error,
          HBD_RET_OK: register success
******************************************************************/
GS8 HBD_SetI2cRW(GU8 uchI2cIdLowTwoBitsSelect,
                 GU8(*pI2cWriteFunc)(GU8 uchDeviceId, const GU8 uchWriteBytesArr[], GU16 usWriteLen),
                 GU8(*pI2cReadFunc)(GU8 uchDeviceId, const GU8 uchCmddBytesArr[], GU16 usCmddLen, GU8 uchReadBytesArr[], GU16 usMaxReadLen));

/****************************************************************
* Description: set delay function callback
* Input:  pDelayUsFunc:delay function callback
* Return: None
******************************************************************/
void HBD_SetDelayUsCallback(void (*pDelayUsFunc)(GU16 usUsec));

/****************************************************************
* Description: load new GH30x config
* Input:    uchNewConfigArr: config array ,
            uchForceUpdateConfig: 1: need force update,0 :don't need,
* Return: HBD_RET_OK:load config success, HBD_RET_COMM_ERROR: load config error,
******************************************************************/
GS8 HBD_LoadNewConfig(GU8 uchNewConfigArr[], GU8 uchForceUpdateConfig);

/****************************************************************
* Description: load new GH30x reg config array
* Input:    stRegConfigArr: reg config array ,
            usRegConfigLen: reg config array len,
* Return: HBD_RET_OK:load config success, HBD_RET_COMM_ERROR: load config error,
******************************************************************/
GS8 HBD_LoadNewRegConfigArr(const ST_REGISTER stRegConfigArr[], GU16 usRegConfigLen);

/****************************************************************
* Description: Communication operation interface confirm.
* Output:  None
* Return: HBD_RET_OK=GH30x comunicate ok,
          HBD_RET_COMM_ERROR=GH30x comunicate error,
          HBD_RET_COMM_NOT_REGISTERED_ERROR=GH30x i2c/spi interface not registered,
******************************************************************/
GS8 HBD_CommunicationInterfaceConfirm(void);

/****************************************************************
* Description: simple init hbd configure parameters
* Input:    stHbInitConfigParam: Init Config struct ,see ST_HBD_INIT_CONFIG_TYPE,
* Return: HBD_RET_OK=success,
          HBD_RET_PARAMETER_ERROR=paramters error,
          HBD_RET_COMM_ERROR=GH30x comunicate error,
          HBD_RET_COMM_NOT_REGISTERED_ERROR=GH30x i2c interface not registered
******************************************************************/
GS8 HBD_SimpleInit(ST_HBD_INIT_CONFIG_TYPE *stHbdInitConfigParam);

#if __HBD_HB_ENABLE__
    /****************************************************************
    * Description: start hbd
    * Input:  None
    * Return: HBD_RET_OK=success,
    HBD_RET_NO_INITED_ERROR=not inited,
    ******************************************************************/
    GS8 HBD_HbDetectStart(void);
#endif

/****************************************************************
* Description: start Wear state confirm.
* Input:  None,
* Return: HBD_RET_OK=success,
          HBD_RET_NO_INITED_ERROR=not inited,
          HBD_RET_LED_CONFIG_ALL_OFF_ERROR=all Led disable error,
          HBD_RET_GENERIC_ERROR=don't need check wear state,
******************************************************************/
GS8 HBD_WearStateConfirmStart(void);

/****************************************************************
* Description: start in-ear ep wear detect, only use with hb detect
* Input:  None,
* Return: HBD_RET_OK=success,
          HBD_RET_NO_INITED_ERROR=not inited,
          HBD_RET_LED_CONFIG_ALL_OFF_ERROR=all Led disable error,
          HBD_RET_GENERIC_ERROR=wear detect function is disabled.
******************************************************************/
GS8 HBD_InearEpWearDetectStart(void);

/****************************************************************
* Description: recover in-ear ep detect, only use in autoled int
* Input:  None,
* Return: HBD_RET_OK=success,
          HBD_RET_NO_INITED_ERROR=not inited,
          HBD_RET_LED_CONFIG_ALL_OFF_ERROR=all Led disable error,
          HBD_RET_GENERIC_ERROR=wear detect function is disabled.
******************************************************************/
GS8 HBD_InearEpDetectRecover(void);

/****************************************************************
* Description: start wear detect, only use with hb detect
* Input:  None,
* Return: HBD_RET_OK=success,
          HBD_RET_NO_INITED_ERROR=not inited,
          HBD_RET_LED_CONFIG_ALL_OFF_ERROR=all Led disable error,
          HBD_RET_GENERIC_ERROR=wear detect function is disabled.
******************************************************************/
GS8 HBD_SoftWearDetectStart(void);

/****************************************************************
* Description: recover detect, only use in autoled int
* Input:  None,
* Return: HBD_RET_OK=success,
          HBD_RET_NO_INITED_ERROR=not inited,
          HBD_RET_LED_CONFIG_ALL_OFF_ERROR=all Led disable error,
          HBD_RET_GENERIC_ERROR=wear detect function is disabled.
******************************************************************/
GS8 HBD_SoftWearDetectRecover(void);

#if __HBD_HRV_ENABLE__
    /****************************************************************
    * Description: start hrv
    * Input:  None
    * Return: HBD_RET_OK=success,
    HBD_RET_NO_INITED_ERROR=not inited,
    ******************************************************************/
    GS8 HBD_HrvDetectStart(void);
#endif

#if __HBD_HSM_ENABLE__
    /****************************************************************
    * Description: start sleep monitor
    * Input:  None
    * Return: HBD_RET_OK=success,
    HBD_RET_NO_INITED_ERROR=not inited,
    ******************************************************************/
    GS8 HBD_HsmDetectStart(void);
#endif

#if __HBD_BP_ENABLE__
    /****************************************************************
    * Description: start blood pressure detect
    * Input:  None
    * Return: HBD_RET_OK=success,
    HBD_RET_LED_CONFIG_ALL_OFF_ERROR=all Led disable error,
    HBD_RET_NO_INITED_ERROR=not inited,
    ******************************************************************/
    GS8 HBD_BpdDetectStart(void);
#endif

#if __HBD_PFA_ENABLE__
    /****************************************************************
    * Description: start pfa detect
    * Input:  None
    * Return: HBD_RET_OK=success,
    HBD_RET_LED_CONFIG_ALL_OFF_ERROR=all Led disable error,
    HBD_RET_NO_INITED_ERROR=not inited,
    ******************************************************************/
    GS8 HBD_PfaDetectStart(void);
#endif

#if (__HBD_HB_ENABLE__ && __HBD_HRV_ENABLE__)
    /****************************************************************
    * Description: start hbd with hrv
    * Input:  None
    * Return: HBD_RET_OK=success,
    HBD_RET_LED_CONFIG_ALL_OFF_ERROR=all Led disable error,
    HBD_RET_NO_INITED_ERROR=not inited,
    ******************************************************************/
    GS8 HBD_HbWithHrvDetectStart(void);
#endif

/****************************************************************
* Description: stop hbd
* Input:  None
* Return: HBD_RET_OK=success,
          HBD_RET_NO_INITED_ERROR=fail:don't init success
******************************************************************/
GS8 HBD_Stop(void);

/****************************************************************
* Description: start adt wear detect
* Input:  None
* Return: HBD_RET_OK=success,
          HBD_RET_NO_INITED_ERROR=fail:don't init success
******************************************************************/
GS8 HBD_AdtWearDetectStart(void);

/****************************************************************
* Description: start adt wear detect continuous
* Input:  None
* Return: HBD_RET_OK=success,
          HBD_RET_NO_INITED_ERROR=fail:don't init success
******************************************************************/
GS8 HBD_AdtWearContinuousDetectStart(void);

/****************************************************************
* Description: check adt has started
* Input:  None
* Return: 1= has started,
          0= has not started
******************************************************************/
GU8 HBD_IsAdtWearDetectHasStarted(void);

/****************************************************************
* Description: get Int status
* Input:  None
* Return: Status, see EM_INT_STATUS
******************************************************************/
GU8 HBD_GetIntStatus(void);

#if __HBD_HB_ENABLE__
/****************************************************************
* Description: calculate hbd_value and wearing detect by newdata int.
* Input:  stGsAxisValue: gsensor data
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output:   puchHbValue:ptr of heartbeat value output
            puchWearingState:ptr of wearing state output
            puchWearingQuality: ptr of wearing quality output
            puchVoiceBroadcast: ptr of voice broadcast output
            pusRRvalue: ptr of RR value output
* Return: refresh flag, 1:heartbeat refresh, 2:wearingstate refresh,3: heartbeat&wearingstate refresh.
******************************************************************/
GU8 HBD_HbCalculateByNewdataInt(ST_GS_DATA_TYPE *stGsAxisValue, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity,
                                GU8 *puchHbValue, GU8 *puchWearingState, GU8 *puchWearingQuality, GU8 *puchVoiceBroadcast, GU16 *pusRRvalue);

/****************************************************************
* Description: calculate hbd_value and wearing detect by fifo int.
* Input:  stGsAxisValue: gsensor data buffer
             usGsDataNum: gsensor data count
             emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output:   puchHbValue:ptr of heartbeat value output
            puchWearingState:ptr of wearing state output
            puchWearingQuality: ptr of wearing quality output
            puchVoiceBroadcast: ptr of voice broadcast output
            pusRRvalue: ptr of RR value output
* Return: refresh flag, if heartbeat value have refresh that return 1.
******************************************************************/
GU8 HBD_HbCalculateByFifoInt(ST_GS_DATA_TYPE stGsAxisValue[], GU16 usGsDataNum, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity,
                             GU8 *puchHbValue, GU8 *puchWearingState, GU8 *puchWearingQuality, GU8 *puchVoiceBroadcast, GU16 *pusRRvalue);

/****************************************************************
* Description: calculate hbd_value and wearing detect by fifo int.
* Input:  stGsAxisValue: gsensor data buffer
             usGsDataNum: gsensor data count
             emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output:   puchHbValue:ptr of heartbeat value output
            puchWearingState:ptr of wearing state output
            puchWearingQuality: ptr of wearing quality output
            puchVoiceBroadcast: ptr of voice broadcast output
            pusRRvalue: ptr of RR value output
            nRawdataOut: ptr of rawdata array output (ppg1, ppg2, gs-x, gs-y, gs-z, index)
            pusRawdataOutLen：ptr of rawdata len output
* Return: refresh flag, if heartbeat value have refresh that return 1.
******************************************************************/
GU8 HBD_HbCalculateByFifoIntDebugOutputData(ST_GS_DATA_TYPE stGsAxisValue[], GU16 usGsDataNum, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity,
        GU8 *puchHbValue, GU8 *puchWearingState, GU8 *puchWearingQuality, GU8 *puchVoiceBroadcast, GU16 *pusRRvalue,
        GS32 nRawdataOut[][6], GU16 *pusRawdataOutLen);

/****************************************************************
* Description: calculate hbd_value and wearing detect by fifo int.
* Input:  stGsAxisValue: gsensor data buffer
             usGsDataNum: gsensor data count
             emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output:   puchHbValue:ptr of heartbeat value output
            puchAccuracyLevel: ptr of heartbeat accuracy level(1-3), max 3
            puchWearingState:ptr of wearing state output
            puchWearingQuality: ptr of wearing quality output
            puchVoiceBroadcast: ptr of voice broadcast output
            pusRRvalue: ptr of RR value output
* Return: refresh flag, if heartbeat value have refresh that return 1.
******************************************************************/
GU8 HBD_HbCalculateWithLvlByFifoInt(ST_GS_DATA_TYPE stGsAxisValue[], GU16 usGsDataNum, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity, GU8 *puchHbValue, GU8 *puchAccuracyLevel,
                                    GU8 *puchWearingState, GU8 *puchWearingQuality, GU8 *puchVoiceBroadcast, GU16 *pusRRvalue);

/****************************************************************
* Description: calculate hbd_value and wearing detect by fifo int.
* Input:  stGsAxisValue: gsensor data buffer
             usGsDataNum: gsensor data count
             emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output:   puchHbValue:ptr of heartbeat value output
            puchAccuracyLevel: ptr of heartbeat accuracy level(1-3), max 3
            puchWearingState:ptr of wearing state output
            puchWearingQuality: ptr of wearing quality output
            puchVoiceBroadcast: ptr of voice broadcast output
            pusRRvalue: ptr of RR value output
            nRawdataOut: ptr of rawdata array output (ppg1, ppg2, gs-x, gs-y, gs-z, index)
            pusRawdataOutLen：ptr of rawdata len output
* Return: refresh flag, if heartbeat value have refresh that return 1.
******************************************************************/
GU8 HBD_HbCalculateWithLvlByFifoIntDebugOutputData(ST_GS_DATA_TYPE stGsAxisValue[], GU16 usGsDataNum, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity,
        GU8 *puchHbValue, GU8 *puchAccuracyLevel, GU8 *puchWearingState, GU8 *puchWearingQuality, GU8 *puchVoiceBroadcast, GU16 *pusRRvalue,
        GS32 nRawdataOut[][6], GU16 *pusRawdataOutLen);
#endif

#if (__HBD_HB_ENABLE__ && __HBD_HRV_ENABLE__)
/****************************************************************
* Description: calculate hbd_value and wearing detect by newdata int. must started with HBD_HbWithHrvDetectStart
* Input:  stGsAxisValue: gsensor data
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output:   puchHbValue:ptr of heartbeat value output
            puchWearingState:ptr of wearing state output
            puchWearingQuality: ptr of wearing quality output
            puchVoiceBroadcast: ptr of voice broadcast output
            pusRRvalue: ptr of RR value output
            usRRvalueArr: ptr of RR value output
            puchRRvalueCnt: ptr of RR value count output
            puchHrvConfidentLvl: ptr of RR value confident
* Return: refresh flag, 1:heartbeat refresh, 2:wearingstate refresh,3: heartbeat&wearingstate refresh.
******************************************************************/
GU8 HBD_HbWithHrvCalculateByNewdataInt(ST_GS_DATA_TYPE *stGsAxisValue, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity,
                                       GU8 *puchHbValue, GU8 *puchWearingState, GU8 *puchWearingQuality, GU8 *puchVoiceBroadcast,
                                       GU16 usRRvalueArr[4], GU8 *puchRRvalueCnt, GU8 *puchHrvConfidentLvl);

/****************************************************************
* Description: calculate hbd_value,wearing detect and hrv by fifo int. must started with HBD_HbWithHrvDetectStart
* Input:  stGsAxisValue: gsensor data buffer
             usGsDataNum: gsensor data count
             emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output:   puchHbValue:ptr of heartbeat value output
            puchAccuracyLevel: ptr of heartbeat accuracy level(1-3), max 3
            puchWearingState:ptr of wearing state output
            puchWearingQuality: ptr of wearing quality output
            puchVoiceBroadcast: ptr of voice broadcast output
            usRRvalueArr: ptr of RR value output
            puchRRvalueCnt: ptr of RR value count output
            puchHrvConfidentLvl: ptr of RR value confident
            nRawdataOut: ptr of rawdata array output (ppg1, ppg2, gs-x, gs-y, gs-z, index)
            pusRawdataOutLen：ptr of rawdata len output
* Return: refresh flag, if heartbeat value have refresh that return 1.
******************************************************************/
GU8 HBD_HbWithHrvCalculateByFifoIntDbgData(ST_GS_DATA_TYPE stGsAxisValue[], GU16 usGsDataNum, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity,
        GU8 *puchHbValue, GU8 *puchAccuracyLevel, GU8 *puchWearingState, GU8 *puchWearingQuality, GU8 *puchVoiceBroadcast,
        GU16 usRRvalueArr[4], GU8 *puchRRvalueCnt, GU8 *puchHrvConfidentLvl,
        GS32 nRawdataOut[][6], GU16 *pusRawdataOutLen);
#endif

/****************************************************************
* Description: Wear state confirm.
* Output:   None,
* Return: wear state, 0: during wear state detect;
                      1: state output wear
                      2: state output unwaer
******************************************************************/
GU8 HBD_WearStateConfirm(void);

/****************************************************************
* Description: Wear state Detect.
* Output:   None,
* Return: wear state, 0: during wear state detect;
                      1: state output wear
                      2: state output unwaer
******************************************************************/
GU8 HBD_WearStateConfirmEx(void);

/****************************************************************
* Description: Wear state confirm by new int.
* Output:   None,
* Return: wear state, 0: during wear state detect;
                      1: state output wear
                      2: state output unwaer
******************************************************************/
GU8 HBD_WearStateConfirmByNewdataInt(void);

/****************************************************************
* Description: Wear state confirm by new int.
* Output:   None,
* Return: wear state, 0: during wear state detect;
                      1: state output wear
                      2: state output unwaer
******************************************************************/
GU8 HBD_WearStateConfirmByFifoInt(void);

/****************************************************************
* Description: enable wearing and setting direction array
* Input:    fDirectionArr: gsensor direction
* Return: None
******************************************************************/
void HBD_EnableWearing(GF32 fDirectionArr[3]);

/****************************************************************
* Description: Hb algorithm scenario config
* Input:
    uchScenario:
        0 : Default
        1 : Routine
        2 : Indoor running
        3 : Indoor walking
        4 : Stair activity

        5 : Outdoor running
        6 : Outdoor walking

        7 : Tranquillization
        8 : Rest
        9 : Short of breath

        10: Indoor cycling
        11: Outdoor cycling
        12: Bicycle Motocross
        13: High heart rate running

        14: Combination running of treadmill
        15: High intensity exercise combination
        16: Traditional strength training combination
        17: Step-on testing
        18: Ball game
        19: Aerobics

        others: fixed 0(default) and return HBD_RET_PARAMETER_ERROR

* Return: HBD_RET_OK=success,
          HBD_RET_PARAMETER_ERROR=paramters error,
******************************************************************/
GS8 HBD_HbAlgoScenarioConfig(GU8 uchScenario);

#if __HBD_HRV_ENABLE__
/****************************************************************
* Description: calculate hrv_value by newdata int.
* Input:  stGsAxisValue: gsensor data, if want get algo static state value, could set NULL.
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output: pusHrvRRValueArr:ptr of hrv RR value output array
* Return: refresh cnt, all count of RR value have refreshed.
******************************************************************/
GU8 HBD_HrvCalculateByNewdataInt(ST_GS_DATA_TYPE *stGsAxisValue, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity, GU16 *pusHrvRRValueArr);

/****************************************************************
* Description: calculate hrv_value by fifo int.
* Input:  stGsAxisValue: gsensor data buffer, if want get algo static state value, could set NULL.
          usGsDataNum: gsensor data count, if want get algo static state value, could set less thah 100.
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output: pusHrvRRValueArr:ptr of hrv RR value output array
* Return: refresh cnt, all count of RR value have refreshed.
******************************************************************/
GU8 HBD_HrvCalculateByFifoInt(ST_GS_DATA_TYPE stGsAxisValue[], GU16 usGsDataNum, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity, GU16 *pusHrvRRValueArr);

/****************************************************************
* Description: calculate hrv_value by fifo int.
* Input:  stGsAxisValue: gsensor data buffer, if want get algo static state value, could set NULL.
          usGsDataNum: gsensor data count, if want get algo static state value, could set less thah 100.
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output: pusHrvRRValueArr:ptr of hrv RR value output array
          nRawdataOut: ptr of rawdata array output (ppg1, ppg2, gs-x, gs-y, gs-z, index)
          pusRawdataOutLen：ptr of rawdata len output
* Return: refresh cnt, all count of RR value have refreshed.
******************************************************************/
GU8 HBD_HrvCalculateByFifoIntDbgRawdata(ST_GS_DATA_TYPE stGsAxisValue[], GU16 usGsDataNum, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity,
                                        GU16 *pusHrvRRValueArr, GS32 nRawdataOut[][6], GU16 *pusRawdataOutLen);

/****************************************************************
* Description: calculate hrv_value by newdata int.
* Input:  stGsAxisValue: gsensor data, if want get algo static state value, could set NULL.
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output: pusHrvRRValueArr:ptr of hrv RR value output array
          puchConfidentLvl:ptr of confident level, 0:no confidence, 1:confidence
* Return: refresh cnt, all count of RR value have refreshed.
******************************************************************/
GU8 HBD_HrvCalculateWithLvlByNewdataInt(ST_GS_DATA_TYPE *stGsAxisValue, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity, GU16 *pusHrvRRValueArr, GU8 *puchConfidentLvl);

/****************************************************************
* Description: calculate hrv_value by fifo int.
* Input:  stGsAxisValue: gsensor data buffer, if want get algo static state value, could set NULL.
          usGsDataNum: gsensor data count, if want get algo static state value, could set less thah 100.
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output: pusHrvRRValueArr:ptr of hrv RR value output array
          puchConfidentLvl:ptr of confident level, 0:no confidence, 1:confidence
* Return: refresh cnt, all count of RR value have refreshed.
******************************************************************/
GU8 HBD_HrvCalculateWithLvlByFifoInt(ST_GS_DATA_TYPE stGsAxisValue[], GU16 usGsDataNum, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity, GU16 *pusHrvRRValueArr, GU8 *puchConfidentLvl);

/****************************************************************
* Description: calculate hrv_value by fifo int.
* Input:  stGsAxisValue: gsensor data buffer, if want get algo static state value, could set NULL.
          usGsDataNum: gsensor data count, if want get algo static state value, could set less thah 100.
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output: pusHrvRRValueArr:ptr of hrv RR value output array
          puchConfidentLvl:ptr of confident level, 0:no confidence, 1:confidence
          nRawdataOut: ptr of rawdata array output (ppg1, ppg2, gs-x, gs-y, gs-z, index)
          pusRawdataOutLen：ptr of rawdata len output
* Return: refresh cnt, all count of RR value have refreshed.
******************************************************************/
GU8 HBD_HrvCalculateWithLvlByFifoIntDbgRawdata(ST_GS_DATA_TYPE stGsAxisValue[], GU16 usGsDataNum, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity,
        GU16 *pusHrvRRValueArr, GU8 *puchConfidentLvl, GS32 nRawdataOut[][6], GU16 *pusRawdataOutLen);
#endif

#if __HBD_HSM_ENABLE__
/****************************************************************
* Description: calculate sleep state by newdata int.
* Input:  stGsAxisValue: gsensor data, if want get algo static state value, could set NULL.
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output: puchSleepState:ptr of sleep state value output, 0:deep sleep, 1:light sleep, 2: REM, 3: sober
          puchHbValue:ptr of hb value output
          puchRespiratoryRate:ptr of respiratory rate value output
          psFallAsleepTime: ptr of fall asleep time, if time calc output val not refresh, return val less than 0.
          psWakeupTime: ptr of wake up time, if time calc output val not refresh, return val less than 0.
* Return: refresh flag, if Hsm sleep state value have refresh that return 1.
******************************************************************/
GU8 HBD_HsmCalculateByNewdataInt(ST_GS_DATA_TYPE *stGsAxisValue, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity,
                                 GU8 *puchSleepState, GU8 *puchHbValue, GU8 *puchRespiratoryRate, GU16 *psFallAsleepTime, GU16 *psWakeupTime);

/****************************************************************
* Description: calculate sleep state by fifo int.
* Input:  stGsAxisValue: gsensor data buffer, if want get algo static state value, could set NULL.
          usGsDataNum: gsensor data count, if want get algo static state value, could set less thah 25.
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output: puchSleepState:ptr of sleep state value output, 0:deep sleep, 1:light sleep, 2: REM, 3: sober
          puchHbValue:ptr of hb value output
          puchRespiratoryRate:ptr of respiratory rate value output
          psFallAsleepTime: ptr of fall asleep time, if time calc output val not refresh, return val less than 0.
          psWakeupTime: ptr of wake up time, if time calc output val not refresh, return val less than 0.
* Return: refresh flag, if Hsm sleep state value have refresh that return 1.
******************************************************************/
GU16 HBD_HsmCalculateByFifoInt(ST_GS_DATA_TYPE stGsAxisValue[], GU16 usGsDataNum, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity,
                               GU8 *puchSleepState, GU8 *puchHbValue, GU8 *puchRespiratoryRate, GU16 *psFallAsleepTime, GU16 *psWakeupTime);

/****************************************************************
* Description: calculate sleep state by fifo int.
* Input:  stGsAxisValue: gsensor data buffer, if want get algo static state value, could set NULL.
          usGsDataNum: gsensor data count, if want get algo static state value, could set less thah 25.
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output: puchSleepState:ptr of sleep state value output, 0:deep sleep, 1:light sleep, 2: REM, 3: sober
          puchHbValue:ptr of hb value output
          puchRespiratoryRate:ptr of respiratory rate value output
          nRawdataOut: ptr of rawdata array output (ppg1, ppg2, gs-x, gs-y, gs-z, index)
          pusRawdataOutLen：ptr of rawdata len output
          psFallAsleepTime: ptr of fall asleep time, if time calc output val not refresh, return val less than 0.
          psWakeupTime: ptr of wake up time, if time calc output val not refresh, return val less than 0.
* Return: refresh flag, if Hsm sleep state value have refresh that return 1.
******************************************************************/
GU16 HBD_HsmCalculateByFifoIntDbgRawdata(ST_GS_DATA_TYPE stGsAxisValue[], GU16 usGsDataNum, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity,
        GU8 *puchSleepState, GU8 *puchHbValue, GU8 *puchRespiratoryRate, GU16 *psFallAsleepTime, GU16 *psWakeupTime,
        GS32 nRawdataOut[][6], GU16 *pusRawdataOutLen);

/****************************************************************
* Description: calculate sleep state info by sleep state.
* Input:  pusSleepStateValueArr: sleep state array.
          nSleepStateValueLength: sleep state array length.
          usFallAsleepTime: asleep time.
          usWakeupTime: wake up time.
* Output: pnSleepStateInfo[0]:sleep start time, x minutes
          pnSleepStateInfo[1]:sleep stop time, x minutes
          pnSleepStateInfo[2]:percent of deep sleep
          pnSleepStateInfo[3]:percent of light sleep
          pnSleepStateInfo[4]:percent of REM
          pnSleepStateInfo[5]:percent of sober
          pnSleepStateInfo[6]:score of sleep quality
* Return: None.
******************************************************************/
void HBD_HsmGetSleepStateInfo(GS16 *pusSleepStateValueArr, GS32 nSleepStateValueLength, GU16 usFallAsleepTime,  GU16 usWakeupTime, GS32 *pnSleepStateInfo);
#endif

#if __HBD_BP_ENABLE__
/****************************************************************
* Description: calculate blood pressure by newdata int.
* Input:  stGsAxisValue: gsensor data, if want get algo static state value, could set NULL.
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output: pusSbpValue:ptr of spd value output
          pusDbpValue:ptr of dbp value output
* Return: refresh flag, if blood pressure value have refresh that return 1.
******************************************************************/
GU8 HBD_BpdCalculateByNewdataInt(ST_GS_DATA_TYPE *stGsAxisValue, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity,
                                 GU16 *pusSbpValue, GU16 *pusDbpValue);

/****************************************************************
* Description: calculate sleep state by fifo int.
* Input:  stGsAxisValue: gsensor data buffer, if want get algo static state value, could set NULL.
          usGsDataNum: gsensor data count, if want get algo static state value, could set less thah 25.
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output: pusSbpValue:ptr of sbp value output
          pusDbpValue:ptr of dbp value output
* Return: refresh flag, if Hsm sleep state value have refresh that return 1.
******************************************************************/
GU16 HBD_BpdCalculateByFifoInt(ST_GS_DATA_TYPE stGsAxisValue[], GU16 usGsDataNum, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity,
                               GU16 *pusSbpValue, GU16 *pusDbpValue);
#endif

#if __HBD_PFA_ENABLE__
/****************************************************************
* Description: calculate Pfa level by newdata int.
* Input:  stGsAxisValue: gsensor data, if want get algo static state value, could set NULL.
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output: puchPressureLevel:ptr of pressure level output
          puchFatigueLevel:ptr of fatigue level output
          puchBodyAge:ptr of body-age output
* Return: refresh flag, if pfa value have refresh that return 1.
******************************************************************/
GU8 HBD_PfaCalculateByNewdataInt(ST_GS_DATA_TYPE *stGsAxisValue, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity,
                                 GU8 *puchPressureLevel, GU8 *puchFatigueLevel, GU8 *puchBodyAge);

/****************************************************************
* Description: calculate sleep state by fifo int.
* Input:  stGsAxisValue: gsensor data buffer, if want get algo static state value, could set NULL.
          usGsDataNum: gsensor data count, if want get algo static state value, could set less thah 25.
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output: puchPressureLevel:ptr of pressure level output
          puchFatigueLevel:ptr of fatigue level output
          puchBodyAge:ptr of body-age output
* Return: refresh flag, if pfa value have refresh that return 1.
******************************************************************/
GU16 HBD_PfaCalculateByFifoInt(ST_GS_DATA_TYPE stGsAxisValue[], GU16 usGsDataNum, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity,
                               GU8 *puchPressureLevel, GU8 *puchFatigueLevel, GU8 *puchBodyAge);
#endif

/****************************************************************
* Description: calculate wear state by newdata int.
* Input:  stGsAxisValue: gsensor data, if want get algo static state value, could set NULL.
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output: puchWearState:ptr of wear state value output, 0:default, 1:wear, 2:unwear,
* Return: refresh flag, if wear state value have refresh that return 1.
******************************************************************/
GU8 HBD_InearEpWearDetectCalculateByNewdataInt(ST_GS_DATA_TYPE *stGsAxisValue, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity, GU8 *puchWearState);

/****************************************************************
* Description: calculate wear state by fifo int.
* Input:  stGsAxisValue: gsensor data buffer, if want get algo static state value, could set NULL.
          usGsDataNum: gsensor data count, if want get algo static state value, could set less thah 25.
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output: puchWearState:ptr of wear state value output, 0:default, 1:wear, 2:unwear,
* Return: refresh flag, if wear state value have refresh that return 1.
******************************************************************/
GU16 HBD_InearEpWearDetectCalculateByFifoInt(ST_GS_DATA_TYPE stGsAxisValue[], GU16 usGsDataNum, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity, GU8 *puchWearState);

/****************************************************************
* Description: calculate wear state by fifo int.
* Input:  stGsAxisValue: gsensor data buffer, if want get algo static state value, could set NULL.
          usGsDataNum: gsensor data count, if want get algo static state value, could set less thah 25.
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output: puchWearState:ptr of wear state value output, 0:default, 1:wear, 2:unwear,
          nRawdataOut: ptr of rawdata array output (ppg1, ppg2, gs-x, gs-y, gs-z, index)
          pusRawdataOutLen：ptr of rawdata len output
* Return: refresh flag, if wear state value have refresh that return 1.
******************************************************************/
GU16 HBD_InearEpWearDetectByFifoIntDbgRawdata(ST_GS_DATA_TYPE stGsAxisValue[], GU16 usGsDataNum, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity, GU8 *puchWearState, GS32 nRawdataOut[][6], GU16 *pusRawdataOutLen);

/****************************************************************
* Description: get fifo count has read
* Input:  None,
* Return: fifo count has read,
******************************************************************/
GU8 HBD_GetFifoCntHasRead(void);

/****************************************************************
* Description: Reset chip
* Input:  None
* Return: HBD_RET_OK=success,
          HBD_RET_COMM_NOT_REGISTERED_ERROR=GH30x i2c interface not registered
******************************************************************/
GS8 HBD_ChipReset(void);

/****************************************************************
* Description: set irq pluse width
* Input:  uchIrqPluseWidth: irq width(us) setting ,
                            if set 0: fixed 1us and return HBD_RET_PARAMETER_ERROR
* Return: HBD_RET_OK=success,
          HBD_RET_PARAMETER_ERROR=fail:parameter error
******************************************************************/
GS8 HBD_SetIrqPluseWidth(GU8 uchIrqPluseWidth);

/****************************************************************
* Description: change Hb config
* Input:  uchMode: 0:Hrv reconfig, else Hb reconfig
*         emFifoEnable: see EM_HBD_FUNCTIONAL_STATE
* Return: HBD_RET_OK=success,
          HBD_RET_PARAMETER_ERROR=fail:parameter error,
******************************************************************/
GS8 HBD_FifoConfig(GU8 uchMode, EM_HBD_FUNCTIONAL_STATE emFifoEnable);

/****************************************************************
* Description: set irq pluse width
* Input:  uchMode: 0:Hrv config,
                   1:Hb config
                   2:hsm config
                   3:bpd config
                   4:pfa config
            usFifoCnt: Fifo thr setting (if enable two channel led,thr will auto * 2)
* Return: HBD_RET_OK=success,
          HBD_RET_PARAMETER_ERROR=fail:parameter error, usFifoCnt not allow 0, uchMode > 4
******************************************************************/
GS8 HBD_SetFifoThrCnt(GU8 uchMode, GU16 usFifoCnt);

/****************************************************************
* Description: Hb algorithm output fast mode config
* Input:    emOutputFast: see EM_HBD_FUNCTIONAL_STATE
* Return: HBD_RET_OK=success,
******************************************************************/
GS8 HBD_HbAlgoOutputFastModeConfig(EM_HBD_FUNCTIONAL_STATE emOutputFast);

/****************************************************************
* Description: Hb algorithm output time config, only nonFast mode valid
* Input:    nOutputTime: valid 7~GS32_MAX
* Return: HBD_RET_OK=success,
******************************************************************/
GS8 HBD_HbAlgoOutputNonFastModeTimeConfig(GS32 nOutputTime);

/****************************************************************
* Description: get Hb algorithm output time
* Input:  None
* Return: output time,
******************************************************************/
GS32 HBD_GetHbAlgoOutputNonFastModeTime(void);

/****************************************************************
* Description: clear hb detect reference value
* Input:  None
* Return: HBD_RET_OK=success,
******************************************************************/
GS8 HBD_HbDetectClearReference(void);

/****************************************************************
* Description: config autoled channel
* Input:  stAutoLedChannelConfig: channal config
          uchChannel: channel index(1 or 2)
* Return: HBD_RET_OK=success,
          HBD_RET_PARAMETER_ERROR=paramters error,
******************************************************************/
GS8 HBD_AutoLedChannelConfig(ST_AUTO_LED_CH_CONFIG stAutoLedChannelConfig, GU8 uchChannel);

/****************************************************************
* Description: config autoled bg thr
* Input:  uchBgUpperThrConfig: bg upper thr config
          uchgLowerThrConfig: bg lower thr config
* Return: HBD_RET_OK=success
******************************************************************/
GS8 HBD_AutoLedBgThrConfig(GU8 uchBgUpperThrConfig, GU8 uchgLowerThrConfig);

/****************************************************************
* Description: config autoled
* Input:  stAutoLedCh1Config: channal 1 config
          stAutoLedCh2Config: channal 2 config
          uchBgUpperThrConfig: bg upper thr config
          uchgLowerThrConfig: bg lower thr config
* Return: HBD_RET_OK=success,
******************************************************************/
GS8 HBD_AutoLedConfig(ST_AUTO_LED_CH_CONFIG stAutoLedCh1Config, ST_AUTO_LED_CH_CONFIG stAutoLedCh2Config,
                      GU8 uchBgUpperThrConfig, GU8 uchgLowerThrConfig);

/****************************************************************
* Description: config autoled gain start val
* Input:  emUseConfigValEnable: set HBD_FUNCTIONAL_STATE_ENABLE, use uchGainVal,
                                else use default val;
          uchGainVal: gain val, only 0-7 valid,
* Return: HBD_RET_OK=success,
******************************************************************/
GS8 HBD_AutoLedGainModeConfig(EM_HBD_FUNCTIONAL_STATE emUseConfigValEnable, GU8 uchGainVal);

/****************************************************************
* Description: config autoled current thr max val
* Input:  uchAutoledCurrentMaxThrVal: current max thr val;
* Return: HBD_RET_OK=success,
******************************************************************/
GS8 HBD_AutoLedCurrentMaxThrConfig(GU8 uchAutoledCurrentMaxThrVal);

/****************************************************************
* Description: Get version
* Input:    None,
* Return: library Hbd ctrl version
******************************************************************/
GS8 *HBD_GetHbdVersion(void);

/****************************************************************
* Description: Get hba algorithm version
* Input:    None,
* Return: library hba algorithm version
******************************************************************/
GS8 *HBD_GetHbaVersion(void);

#if __HBD_SPO2_ENABLE__
/****************************************************************
* Description: Get spo2 algorithm version
* Input:    None,
* Return: library hba algorithm version
******************************************************************/
GU8 *HBD_GetSpo2Version(void);

/****************************************************************
* Description: start SpO2
* Input:  None
* Return: HBD_RET_OK=success,
          HBD_RET_LED_CONFIG_ALL_OFF_ERROR=all Led disable error,
          HBD_RET_NO_INITED_ERROR=not inited,
******************************************************************/
GS8 HBD_SpO2DetectStart(void);

/****************************************************************
* Description: calculate SpO2_value by fifo int.
* Input:  stGsAxisValue: gsensor data buffer, if want get algo static state value, could set NULL.
          usGsDataNum: gsensor data count, if want get algo static state value, could set less thah 100.
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output:   pusSpo2Value : Spo2 Value
            puchSpo2ConfidentLvl: Spo2 Confident Level
            puchHbValue  : HB Value
            puchHbConfidentLvl: HB Confident Level
            pusHrvRRVal1 : HRV RRI value 1
            pusHrvRRVal2 : HRV RRI value 2
            pusHrvRRVal3 : HRV RRI value 3
            pusHrvRRVal4 : HRV RRI value 4
            puchHrvConfidentLvl: HRV Confident Level
            puchHrvcnt : HRV valid cnt
            pusSpo2RVal :spo2 R value
            puchWearingState : Wearing State
* Return:  refresh flag, if spo2 value have refresh that return 1.
******************************************************************/
GU8 HBD_Spo2CalculateByFifoInt(ST_GS_DATA_TYPE stGsAxisValue[], GU16 usGsDataNum, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity, GU8 *pusSpo2Value, GU8 *puchSpo2ConfidentLvl, GU8 *puchHbValue, GU8 *puchHbConfidentLvl,
                               GU16 *pusHrvRRVal1, GU16 *pusHrvRRVal2, GU16 *pusHrvRRVal3, GU16 *pusHrvRRVal4, GU8 *puchHrvConfidentLvl, GU8 *puchHrvcnt, GU16 *pusSpo2RVal, GU8 *puchWearingState);
/****************************************************************
* Description: calculate SpO2_value by newdata int.
* Input:  stGsAxisValue: gsensor data, if want get algo static state value, could set NULL.
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output:   pusSpo2Value : Spo2 Value
            puchSpo2ConfidentLvl: Spo2 Confident Level
            puchHbValue  : HB Value
            puchHbConfidentLvl: HB Confident Level
            pusHrvRRVal1 : HRV RRI value 1
            pusHrvRRVal2 : HRV RRI value 2
            pusHrvRRVal3 : HRV RRI value 3
            pusHrvRRVal4 : HRV RRI value 4
            puchHrvConfidentLvl: HRV Confident Level
            puchHrvcnt : HRV valid cnt
            pusSpo2RVal :spo2 R value
            puchWearingState : Wearing State
* Return:  refresh flag, if spo2 value have refresh that return 1.
******************************************************************/
GU8 HBD_Spo2CalculateByNewdataInt(ST_GS_DATA_TYPE *stGsAxisValue, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity,  GU8 *pusSpo2Value, GU8 *puchSpo2ConfidentLvl, GU8 *puchHbValue, GU8 *puchHbConfidentLvl,
                                  GU16 *pusHrvRRVal1, GU16 *pusHrvRRVal2, GU16 *pusHrvRRVal3, GU16 *pusHrvRRVal4, GU8 *puchHrvConfidentLvl, GU8 *puchHrvcnt, GU16 *pusSpo2RVal, GU8 *puchWearingState);

/****************************************************************
* Description: calculate SpO2_value by fifo int.
* Input:  stGsAxisValue: gsensor data buffer, if want get algo static state value, could set NULL.
          usGsDataNum: gsensor data count, if want get algo static state value, could set less thah 100.
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output:   pusSpo2Value : Spo2 Value
            puchSpo2ConfidentLvl: Spo2 Confident Level
            puchHbValue  : HB Value
            puchHbConfidentLvl: HB Confident Level
            pusHrvRRVal1 : HRV RRI value 1
            pusHrvRRVal2 : HRV RRI value 2
            pusHrvRRVal3 : HRV RRI value 3
            pusHrvRRVal4 : HRV RRI value 4
            puchHrvConfidentLvl: HRV Confident Level
            puchHrvcnt : HRV valid cnt
            pusSpo2RVal :spo2 R value
            puchWearingState : Wearing State
* Return:  refresh flag, if spo2 value have refresh that return 1.
******************************************************************/
GU8 HBD_Spo2CalculateByFifoIntDbgRawdata(ST_GS_DATA_TYPE stGsAxisValue[], GU16 usGsDataNum, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity,
        GU8 *pusSpo2Value, GU8 *puchSpo2ConfidentLvl, GU8 *puchHbValue, GU8 *puchHbConfidentLvl,
        GU16 *pusHrvRRVal1, GU16 *pusHrvRRVal2, GU16 *pusHrvRRVal3, GU16 *pusHrvRRVal4, GU8 *puchHrvConfidentLvl,
        GU8 *puchHrvcnt, GU16 *pusSpo2RVal, GU8 *puchWearingState, GS32 nRawdataOut[][6], GU16 *pusRawdataOutLen);
#endif

#if __HBD_HB_ENABLE__
/****************************************************************
* Description: calculate hbd_value and wearing detect by newdata int.
* Input:  stGsAxisValue: gsensor data
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output:   puchHbValue:ptr of heartbeat value output
            puchWearingState:ptr of wearing state output
            puchWearingQuality: ptr of wearing quality output
            puchVoiceBroadcast: ptr of voice broadcast output
            pusRRvalue: ptr of RR value output
* Return: refresh flag, 1:heartbeat refresh, 2:wearingstate refresh,3: heartbeat&wearingstate refresh.
******************************************************************/
GU8 HBD_HbCalculateByNewdataIntDbg(ST_GS_DATA_TYPE *stGsAxisValue, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity,
                                   GU8 *puchHbValue, GU8 *puchWearingState, GU8 *puchWearingQuality, GU8 *puchVoiceBroadcast, GU16 *pusRRvalue);
#endif

#if __HBD_SPO2_ENABLE__
/****************************************************************
* Description: calculate SpO2_value by fifo int.
* Input:  stGsAxisValue: gsensor data buffer, if want get algo static state value, could set NULL.
          usGsDataNum: gsensor data count, if want get algo static state value, could set less thah 100.
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output:   pusSpo2Value : Spo2 Value
            puchSpo2ConfidentLvl: Spo2 Confident Level
            puchHbValue  : HB Value
            puchHbConfidentLvl: HB Confident Level
            pusHrvRRVal1 : HRV RRI value 1
            pusHrvRRVal2 : HRV RRI value 2
            pusHrvRRVal3 : HRV RRI value 3
            pusHrvRRVal4 : HRV RRI value 4
            puchHrvConfidentLvl: HRV Confident Level
            puchHrvcnt : HRV valid cnt
            pusSpo2RVal :spo2 R value
            puchWearingState : Wearing State
            nRawdataOut :
            pusRawdataOutLen :
            puchValidLvl :
* Return:  refresh flag, if spo2 value have refresh that return 1.
******************************************************************/
GU8 HBD_Spo2CalculateByFifoIntDbgRawdataInnerUse(ST_GS_DATA_TYPE stGsAxisValue[], GU16 usGsDataNum, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity,
        GU8 *pusSpo2Value, GU8 *puchSpo2ConfidentLvl, GU8 *puchHbValue, GU8 *puchHbConfidentLvl,
        GU16 *pusHrvRRVal1, GU16 *pusHrvRRVal2, GU16 *pusHrvRRVal3, GU16 *pusHrvRRVal4, GU8 *puchHrvConfidentLvl,
        GU8 *puchHrvcnt, GU16 *pusSpo2RVal, GU8 *puchWearingState, GS32 nRawdataOut[][6], GU16 *pusRawdataOutLen, GU8 *puchValidLvl);
#endif

/****************************************************************
* Description: config adt confrim
* Input:  usAdtConfirmGsThrVal : gsensor confirm thr
          uchAdtConfirmGsCalcThrCntMax: gsensor confirm thr cnt max
          uchAdtConfirmGsCalcThrCnt  : gsensor confirm thr cnt
* Return: None
******************************************************************/
void HBD_AdtConfirmConfig(GU16 usAdtConfirmGsThrVal, GU8 uchAdtConfirmGsCalcThrCntMax, GU8 uchAdtConfirmGsCalcThrCnt);

/****************************************************************
* Description: start adt confrim
* Input:  None
* Return: HBD_RET_OK=success,
          HBD_RET_LED_CONFIG_ALL_OFF_ERROR=all Led disable error,
          HBD_RET_NO_INITED_ERROR=not inited,
******************************************************************/
GS8 HBD_AdtConfirmStart(void);

/****************************************************************
* Description: adt confirm detect by newdata int.
* Input:  stGsAxisValue: gsensor data
          emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output:  None
* Return: wear flag, 0x00: calc ing 0x11:wear, 0x12:unwear
******************************************************************/
GU8 HBD_AdtConfirmCalculateByNewdataInt(ST_GS_DATA_TYPE *stGsAxisValue, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity);

/****************************************************************
* Description: adt confirm detect by fifo int.
* Input:  stGsAxisValue: gsensor data buffer
             usGsDataNum: gsensor data count
             emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output:   None
* Return: wear flag, 0x00: calc ing 0x11:wear, 0x12:unwear
******************************************************************/
GU8 HBD_AdtConfirmCalculateByFifoInt(ST_GS_DATA_TYPE stGsAxisValue[], GU16 usGsDataNum, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity);

/****************************************************************
* Description: adt confirm detect by fifo int.
* Input:  stGsAxisValue: gsensor data buffer
             usGsDataNum: gsensor data count
             emGsensorSensitivity: G-sensor sensitivity (counts/g), see EM_HBD_GSENSOR_SENSITIVITY
* Output:   nRawdataOut: ptr of rawdata array output (ppg1, ppg2, gs-x, gs-y, gs-z, index)
            pusRawdataOutLen：ptr of rawdata len output
* Return: wear flag, 0x00: calc ing 0x11:wear, 0x12:unwear
******************************************************************/
GU8 HBD_AdtConfirmCalculateByFifoIntDbgOutputData(ST_GS_DATA_TYPE stGsAxisValue[], GU16 usGsDataNum, EM_HBD_GSENSOR_SENSITIVITY emGsensorSensitivity,
        GS32 nRawdataOut[][6], GU16 *pusRawdataOutLen);


// void HBD_HbCalculateByFifoIntTestInit();

// GU8 HBD_HbCalculateByFifoIntTest();
#endif /* __HBD_CTRL_H__ */

/********END OF FILE********* (C) COPYRIGHT 2018 .********/
