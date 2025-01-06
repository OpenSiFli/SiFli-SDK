#ifndef PXIALG_H__
#define PXIALG_H__
#include "stdint.h"
#include "pxivar.h"

#define FW              1

#define FW_DEBUG  0
#if ( FW == 0 )
    #undef FW_DEBUG
    #define FW_DEBUG  0
#endif

//#define STRIIV

#if FW == 1
    #define PXIALG_API
#else
    #ifdef PXIALG_EXPORTS
        #define PXIALG_API      __declspec(dllexport)
    #else
        #define PXIALG_API      __declspec(dllimport)
    #endif
#endif
/*
typedef struct {
    uint8_t                     frame_count;
    uint32_t                    time;
    uint8_t                     touch_flag;
    uint32_t                    nf_ppg_channel;
    uint32_t                    nf_ppg_per_channel;
    int32_t                     *ppg_data;
    uint32_t                    nf_mems;
    int16_t                     *mems_data;
} pah8series_data_t;

typedef enum {
    FLAG_DATA_READY             = 0,
    FLAG_DATA_NOT_READY,
    FLAG_DATA_LOSS,
    FLAG_NO_TOUCH,
    FLAG_DATA_ERROR,
    FLAG_POOR_SIGNAL,
    FLAG_FIFO_ERROR,
    FLAG_TIMMING_ERROR,
    FLAG_MEMS_ERROR,
    FLAG_HR_UPDATE              = 16,
    FLAG_SG_UPDATE,
} PXI_STATUS_FLAG;

typedef enum {
    _quick_report_rms_gs_th     = 0,
    _hr_speed_mode,
    _support_cal_motion_idx_enhancement_enh,
    _rsv1,
    _set_switch_mems_fft_max_energy_th,
    _set_switch_mems_large_energy_num_th,
    _set_switch_mn_fft_energy_lo_thr,
    _set_switch_mn_fft_energy_hi_thr,

    _ppg_large_energy_num_th,
    _max_hr_idx_th,
    _min_hr_idx_th,
    _ppg_fft_total_peak_num_th,
    _mems_large_energy_num_th,
    _mems_fft_max_energy_th,
    _support_signal_grade_motion_detection_enh,
    _support_signal_grade_check_signal_lelvel_enh,
    _support_signal_grade_large_energy_num_enh,
    _rsv2,
    _rsv3,
    _set_reg_shake_count_th0,
} PXI_PARAM_SET;

typedef enum {
    _rms_gs                     = 0,
    _signal_level,
    _cal_motion_idx,
    _is_prediction_hr,
    _get_switch_mems_fft_max_energy_th,
    _get_switch_mems_large_energy_num_th,
    _get_switch_mn_fft_energy_lo_thr,
    _get_switch_mn_fft_energy_hi_thr,

    _hr_sample_count,
    _ppg_large_energy_num,
    _signal_grade_motion_frm,
    _ppg_fft_max_energy_idx,
    _ppg_fft_total_peak_num,
    _mems_large_energy_num,
    _mems_fft_max_energy,
    _grade,
    _signal_grade_hr_to_low_frm,
    _flag_signal_grade_ready_go,
    _motion_flag,
    _get_reg_shake_count_th0,
} PXI_PARAM_GET;

typedef enum {
    MemsLargeEnergyNumTh        = 0,
    MemsFftMaxEnergyTh,
    QuickReportRmsGsTh,
    SupportSigGradeMotionDetection,
    PpgLargeEnergyNumTh,
    SupportSigGradeLargeEnergyNumEnh,
    SupportSigGradeLargeEnergyNumTh,
    SigGradeThr,
    SupportSigGradeCheckSigLevel,
    FlagCheckNotStableFirstHR,
    MaxHrDefinition,
    MinHrDefinition,
} PXI_PARAM2;
*/

#if FW == 1
    /**
    * @brief When HRD and MEMS data are ready, call this function to do the algorithm processing
    *
    * @param[in] HRD_Data   Pointer to the buffer where HRD data (13 Bytes) is stored.
    * @param[in] MEMS_Data  Pointer to the buffer where MEMS data (3*sizeof(float) Bytes) is stored.
    *
    * @return Return one of the PXI_STATUS_FLAG types.
    */
    PXIALG_API int32_t PxiAlg_ProcessByTime(unsigned char *HRD_Data, float *MEMS_Data, float time);
    PXIALG_API int32_t PxiAlg_Process(unsigned char *HRD_Data, float *MEMS_Data);

    /**
    * @brief Call this function to get Heart Rate
    *
    * @param[out] hr   Pointer to a float variable where heart rate is stored
    *
    * @return None
    */
    PXIALG_API void PxiAlg_HrGet(float *hr);


    PXIALG_API void PxiAlg_FlagInitialSignalNoiseGet(uint8_t *flag);
    /**
    * @brief Call this function to feed Algorithm raw data(19 Bytes), and get Signal Grade and Heart Rate
    *
    * @param[in] pixart_19_byte_data   Pointer to a arrag where raw data is stored
    *  Format: 13 bytes PPG + G-sensor X_L[13] + X_H[14] + Y_L[15] + Y_H[16] + Z_L[17] + Z_H[18]
    * @param[out] grade    Pointer to a float variable where signal grade is stored.
    * @param[out] heart_rate   Pointer to a float variable where heart rate is stored
    *
    * @return status bit field, combinatoin of the list below
    * 1 << FLAG_DATA_READY,
    * 1 << FLAG_DATA_NOT_READY,
    * 1 << FLAG_DATA_LOSS,
    * 1 << FLAG_NO_TOUCH,
    * 1 << FLAG_DATA_ERROR,
    * 1 << FLAG_POOR_SIGNAL,
    * 1 << FLAG_FIFO_ERROR,
    * 1 << FLAG_TIMMING_ERROR,
    * 1 << FLAG_MEMS_ERROR,
    * 1 << FLAG_HR_UPDATE,
    * 1 << FLAG_SG_UPDATE,
    */
    PXIALG_API uint32_t PxiAlg_HRM(uint8_t *pixart_19_byte_data, float *grade, float *heart_rate);
#else
    /**
    * @brief When HRD, MEMS, and ECG data are ready, call this function to do the algorithm processing
    *
    * @param[in] HRD_Data   Pointer to the buffer where HRD data (13 Bytes) is stored.
    * @param[in] MEMS_Data  Pointer to the buffer where MEMS data (12 Bytes) is stored.
    * @param[in] ECG_Data   Pointer to the buffer where ECG data (4 Bytes) is stored.
    *
    * @return Return one of the PXI_STATUS_FLAG types.
    */
    PXIALG_API int32_t PxiAlg_Process(unsigned char *HRD_Data, float *MEMS_Data, float ECG_Data);

    /**
    * @brief When HRD, MEMS, ECG data, and time interval are ready, call this function to do the algorithm processing
    *
    * @param[in] HRD_Data   Pointer to the buffer where HRD data (13 Bytes) is stored.
    * @param[in] MEMS_Data  Pointer to the buffer where MEMS data (12 Bytes) is stored.
    * @param[in] ECG_Data   Pointer to the buffer where ECG data (4 Bytes) is stored.
    * @param[in] time       Time interval.
    *
    * @return Return one of the PXI_STATUS_FLAG types.
    */
    PXIALG_API int32_t PxiAlg_ProcessByTime(unsigned char *HRD_Data, float *MEMS_Data, float ECG_Data, float time);

    /**
    * @brief Call this function to get Heart Rate
    *
    * @param[out] hr   Pointer to a float variable where heart rate is stored
    *
    * @return None
    */
    PXIALG_API void PxiAlg_HrGet(float *hr);

    PXIALG_API void PxiAlg_FlagInitialSignalNoiseGet(uint8_t *flag);

    /**
    * @brief Call this     function to get output
    *
    * @param[out] output   Pointer to a SHr_motion_plot variable where output information is stored
    *
    * @return None
    */
    PXIALG_API int32_t PxiAlg_OutputGet(void *output);

    /**
    * @brief Call this     function to get a specific parameter
    *
    * @param[out] param    Pointer to the start address of the variable of parameter list
    * @param[in]  offset   Offset of the parameter variable
    * @param[in]  size     Type size of the parameter
    *
    * @return None
    */
    PXIALG_API void PxiAlg_ParamGet(uint8_t *param, int32_t offset, int32_t size);

    /**
    * @brief Call this     function to set a specific parameter
    *
    * @param[in] offset    Offset of the parameter variable
    * @param[in] size      Type size of the parameter
    * @param[in] param     Pointer to the start address of the variable of parameter list
    *
    * @return None
    */
    PXIALG_API void PxiAlg_ParamSet(int32_t offset, int32_t size, uint8_t *param);
#endif

/**
* @brief When HRD, MEMS, and Expo data are ready, call this function to do the algorithm processing
*
* @param[in] HRD_Data   Pointer to the buffer where HRD data (13 Bytes) is stored.
* @param[in] MEMS_Data  Pointer to the buffer where MEMS data (12 Bytes) is stored.
* @param[in] expo       Exposure Time.
*
* @return Return one of the PXI_STATUS_FLAG types.
*/
PXIALG_API int32_t PxiAlg_Process_Expo(unsigned char *HRD_Data, float *MEMS_Data, int expo);

/**
 * @brief Call this function to determine the version of the algorithm
 *
 *
 * @return Version of the algorithm
 */
PXIALG_API int32_t PxiAlg_Version(void);

/**
 * @brief Call this funtion to get Ready_Flag
 *
 * @return Return Ready_Flag
 */
PXIALG_API unsigned char PxiAlg_GetReadyFlag(void);

/**
 * @brief Call this funtion to get Motion_Flag
 *
 * @return Return Motion_Flag
 */
PXIALG_API unsigned char PxiAlg_GetMotionFlag(void);


/**
* @brief Call this function to get force flag
*
* @return force_flag
*/
PXIALG_API unsigned char PxiAlg_GetForceFlag(void);

/**
 * @brief Call this funtion to get Timeout_Flag
 *
 * @return Return Timeout_Flag
 */
PXIALG_API unsigned char PxiAlg_GetTimeoutFlag(void);

/**
 * @brief Call this function to set PPG buffer pointer and size
 *
 * @param[in] ppg_buffer   Pointer to a float buffer where the PPG data would be stored
 * @param[in] size   PPG buffer size
 *
 * @return None
 */
PXIALG_API void PxiAlg_SetPPGBuffer(float *ppg_buffer, int32_t size);

/**
 * @brief Call this function to set algorithm parameters
 *
 * @param[in] hr_param   Pointer to a "HR_Param" sturcture where the parameters are stored.
 *
 * @return None
 */
PXIALG_API void PxiAlg_SetParam(void *hr_param);

/**
 * @brief Call this function to set data
 *
 * @param[in] data   Pointer to an algorithm-dependent data
 *
 * @return None
 */
PXIALG_API void PxiAlg_SetData(void *data);


/**
 * @brief Call this function to notify algorithm the Output Data Rate of PPG Sensor
 *
 * @param[in] odr           The Output Data Rate of PPG Sensor. Only 10,20,40,160 and 200 ODR are supported.
 *
 * @return 1 for success. 0 for failure.
 */
PXIALG_API uint8_t PxiAlg_SetODR(int32_t odr);

/**
 * @brief Call this function to notify algorithm the Timeout Length of PPG Sensor
 *
 * @param[in] t         The Timeout Length of PPG Sensor. Unit: second.
 *
 * @return 1 for success. 0 for failure.
 */
PXIALG_API uint8_t PxiAlg_SetTimeoutLength(int32_t t);

/**
 * @brief Call this function to notify algorithm the Quality Threshold of PPG Sensor
 *
 * @param[in] th            The Quality Threshold of PPG Sensor. Range: 0.0 ~ 1.0
 *
 * @return 1 for success. 0 for failure.
 */
PXIALG_API uint8_t PxiAlg_SetQualityThreshold(float th);

/**
* @brief Call this function to notify algorithm the HR smooth weight of PPG Sensor
*
* @param[in] w          The weight. Range: 0.0 ~ 1.0
*
* @return 1 for success. 0 for failure.
*/
PXIALG_API uint8_t PxiAlg_SetHRSmoothWeight(float w);

/**
* @brief Call this function to notify algorithm the length of process window of PPG Sensor
*
* @param[in] t          The normal time window.
* @param[in] t_fast     The fast time window.
*
* @return 1 for success. 0 for failure.
*/
PXIALG_API uint8_t PxiAlg_SetProcWindow(float t, float t_fast);

/**
 * @brief Call this function to notify algorithm the number of PPG sample to skip
 *
 * @param[in] num           The number
 *
 * @return 1 for success. 0 for failure.
 */
PXIALG_API uint8_t PxiAlg_SetSkipSampleNum(int num);


/**
 * @brief Call this function to notify algorithm the MEMS Scale of Motion Sensor
 *
 * @param[in] scale         The MEMS Scale of Motion Sensor. 0: +-2G, 1: +-4G, 2: +-8G, and 3: +-16G
 *
 * @return 1 for success. 0 for failure.
 */
PXIALG_API uint8_t PxiAlg_SetMemsScale(int scale);
PXIALG_API uint8_t PxiAlg_SetMemsScaleBase(int scale, int th);





/**
 * @brief Call this function to notify algorithm the time length to force hr to update
 *
 * @param[in] t         The time length
 * @param[in] ub        The upper bound
 * @param[in] lb        The lower bound
 *
 * @return 1 for success. 0 for failure.
 */
PXIALG_API uint8_t PxiAlg_SetForceOutput(int t, int ub, int lb);



/**
 * @brief Call this function to notify algorithm force hr to output
 *
 * @param[in] t         The time length
 * @param[in] ub        The upper bound
 * @param[in] lb        The lower bound
 * @param[in] link_flag To link with normal output
 * @param[in] w         Smoothing weight between force and normal
 *
 * @return 1 for success. 0 for failure.
 */
PXIALG_API uint8_t PxiAlg_SetForceOutputLink(int t, int ub, int lb, int link_flag, float w);


/**
 * @brief Call this function to notify algorithm normal hr to output
 *
 * @param[in] ub            The upper bound
 * @param[in] lb            The lower bound
 * @param[in] static_flag   First normal output at static
 *
 * @return 1 for success. 0 for failure.
 */
PXIALG_API uint8_t PxiAlg_SetNormalOutput(int ub, int lb, int static_flag);


/**
 * @brief Call this function to notify algorithm the time to force hr to output
 *
 * @param[in] t         The duration
 * @param[in] bpm       The amplitude
 *
 * @return 1 for success. 0 for failure.
 */
PXIALG_API uint8_t PxiAlg_SetForceRipple(int t, int bpm);


/**
 * @brief Call this function to notify algorithm the time to force hr to output
 *
 * @param[in] ch        The ppg channel selection
 * @param[in] flag      The resample flag
 *
 * @return 1 for success. 0 for failure.
 */
PXIALG_API uint8_t PxiAlg_SetPackage(int ch, int flag);


/**
 * @brief Call this function to notify algorithm the time length to force hr to update
 *
 * @param[in] t         The time length
 *
 * @return 1 for success. 0 for failure.
 */
PXIALG_API uint8_t PxiAlg_SetForceUpdateTimeLength(int t);
/**
 * @brief Call this function to notify algorithm the time to force hr to output
 *
 * @param[in] t         The time length
 *
 * @return 1 for success. 0 for failure.
 */
PXIALG_API uint8_t PxiAlg_SetForceOutputTime(int t);
/**
 * @brief Call this function to notify algorithm the bound of hr
 *
 * @param[in] ub            The upper bound
 * @param[in] lb            The lower bound
 *
 * @return 1 for success. 0 for failure.
 */
PXIALG_API uint8_t PxiAlg_SetFullHRBound(int ub, int lb);
/**
 * @brief Call this function to notify algorithm the bound of first hr
 *
 * @param[in] ub            The upper bound
 * @param[in] lb            The lower bound
 * @param[in] t         The time length to keep this bound
 *
 * @return 1 for success. 0 for failure.
 */
PXIALG_API uint8_t PxiAlg_SetFirstHRBound(int ub, int lb, int t);



/**
 * @brief Call this function to notify algorithm the MEMS Energy Threshold of Motion Sensor
 *
 * @param[in] lower         The MEMS energy lower threshold
 * @param[in] upper         The MEMS energy upper threshold
 *
 * @return 1 for success. 0 for failure.
 */
PXIALG_API uint8_t PxiAlg_SetMemsEnergyThrd(int lower, int upper);

/**
 * @brief Call this function to set PPG Signal Grade Threshold
 *
 * @param[in] threshold     The PPG Signal Grade Threshold. Its value ranges from 0 to 100.
 *
 * @return 1 for success. 0 for failure.
 */
PXIALG_API uint8_t PxiAlg_SetSigGradeThrd(float thrd);

/**
 * @brief Call this function to set parameters of PPG Signal Grade
 *
 * @param[in] frm_thrd      The PPG Signal Grade Frame Threshold.
 * @param[in] rep_count     The PPG Signal Grade Repetitive Counter.
 *
 * @return 1 for success. 0 for failure.
 */
PXIALG_API uint8_t PxiAlg_SetSigGradeParam(int16_t frm_thrd, int16_t rep_count);

/**
 * @brief Call this function to enable or disable fast output mode
 *
 * @param[in] en        The flag of fast output mode.
 */

PXIALG_API void PxiAlg_EnableFastOutput(uint8_t en);

/**
 * @brief Call this function to enable or disable motion mode
 *
 * @param[in] en        The flag of motion mode.
 */
PXIALG_API void PxiAlg_EnableMotionMode(uint8_t en);

/**
 * @brief Call this function to enable or disable auto mode
 *
 * @param[in] en        The flag of auto mode.
 */
PXIALG_API void PxiAlg_EnableAutoMode(uint8_t en);

/**
 * @brief Call this function to enable or disable tracking mode after fast output
 *
 * @param[in] en        The flag of tracking mode.
 */
PXIALG_API void PxiAlg_EnableTrackingAfterFastOut(uint8_t en);

/**
 * @brief Call this function to enable or disable mems0 signal grade mode
 *
 * @param[in] en        1 : enable. 0 : disable.
 */
PXIALG_API void PxiAlg_EnableMEMS0SigGrade(uint8_t en);

//PXIALG_API void PxiAlg_EnableMEMS0SigGrade(uint8_t en, uint8_t idx);
//// idx = 0: PxiAlg_EnableFastOutput(uint8_t en);
//// idx = 1: PxiAlg_EnableMotionMode(uint8_t en);
//// idx = 2: PxiAlg_EnableAutoMode(uint8_t en);
//// idx = 3: PxiAlg_EnableTrackingAfterFastOut(uint8_t en);

/**
 * @brief Call this function to get PPG Signal Grade
 *
 * @param[out] grade    Pointer to a float variable where signal grade is stored.
 *
 * @return  Return 1 when Signal Grade is ready. Otherwise, return 0.
 */
PXIALG_API uint8_t PxiAlg_GetSigGrade(float *grade);

/**
 * @brief Call this function to get the alarm flag of fast output
 *
 * @param[out] flag     The alarm flag of fast output
 *
 * @return  Return 1 when the flag is set. Otherwise, return 0.
 */
PXIALG_API uint8_t PxiAlg_GetFastOutAlarmFlag(void);

/**
 * @brief Call this function to get the alarm flag of signal bad
 *
 * @param[out] flag     The alarm flag of fast output
 *
 * @return  Return 1 when the flag is set. Otherwise, return 0.
 */
PXIALG_API uint8_t PxiAlg_GetSignalBadAlarm(void);

/**
 * @brief Call this function to open algorithm
 */
#if FW == 1
    PXIALG_API uint8_t PxiAlg_Open_Mem(void *mem);
#else
    PXIALG_API void PxiAlg_Open(void);
#endif

/**
 * @brief Call this function to close algorithm
 */
PXIALG_API void PxiAlg_Close(void);

/**
 * @brief Call this function to initialize 200Hz low-pass filter
 */
PXIALG_API void PxiAlg_LPF200Hz_Init(void);

/**
 * @brief Call this function to process 200Hz low-pass filter
 */
PXIALG_API float PxiAlg_LPF200Hz_Process(float data);

/**
 * @brief Call this function to get PPG Signal Level
 *
 * @param[out] grade    Pointer to a float variable where signal level is stored.
 *
 * @return  Return 1 when Signal Level is ready. Otherwise, return 0.
 */
#ifndef REDUCE_MORE_CODE_SIZE
    PXIALG_API uint8_t PxiAlg_Get_Signal_Level(float *level);
#endif
/**
 * @brief Call this function to enable/disable Check_Not_Stable_First_HR
 *
 * @param[in] 1 : enable, 0: disable
 */
PXIALG_API void PxiAlg_Check_Not_Stable_First_HR(uint8_t check);

/**
 * @brief Call this function to set Signal_Grade_Large_Energy_Num_Th_Lo
 *
 * @param[in] Threshold.
 *
 */
PXIALG_API void PxiAlg_Set_Signal_Grade_Large_Energy_Num_Th_Lo(uint8_t th) ;

/**
 * @brief Call this function to Set SportMode
 *
 * @@param[in] en   0: disable 1: enable
 *
 * @return  Return 1 when Set Signal Grade Large Energy Num Threshold is ready. Otherwise, return 0.
 */
PXIALG_API void PxiAlg_SetSportMode(uint8_t en);

/**
 * @brief Call this function to Set Signal Grade Large Energy Num Threshold
 *
 * @@param[in] th   => threshold
 *
 * @return  Return 1 when Set Signal Grade Large Energy Num Threshold is ready. Otherwise, return 0.
 */
PXIALG_API uint8_t PxiAlg_Set_Signal_Grade_Large_Energy_Num_Th(uint8_t th);


/**
 * @brief Call this function to Set Algorithm Parameters
 *
 * @@param[in] index - index of parameters, param - parameter value
 *
 * @return  Return 1 when Set algorithm parameter suceeessfully. Otherwise, return 0.
 */
PXIALG_API uint8_t PxiAlg_Set_Parameter(uint32_t index, float param);

/**
 * @brief Call this function to Get Algorithm Parameters
 *
 * @@param[in] index - index of parameters, *param - pointer to parameter value
 *
 * @return  Return 1 when Set algorithm parameter suceeessfully. Otherwise, return 0.
 */
PXIALG_API uint8_t PxiAlg_Get_Parameter(uint32_t index, float *param);

/**
* @brief Call this function to Set Algorithm Parameters (2nd Version)
*
* @@param[in] index - index of parameters, param - parameter value
*
* @return  Return 1 when Set algorithm parameter suceeessfully. Otherwise, return 0.
*/
PXIALG_API uint8_t PxiAlg_Set_Parameter2(uint32_t index, float param);

/**
* @brief Call this function to Get Algorithm Parameters (2nd Version)
*
* @@param[in] index - index of parameters, *param - pointer to parameter value
*
* @return  Return 1 when Set algorithm parameter suceeessfully. Otherwise, return 0.
*/
PXIALG_API uint8_t PxiAlg_Get_Parameter2(uint32_t index, float *param);



PXIALG_API uint8_t PxiAlg_Set_Cal_Flag_HR_Trust_Energy_Ratio(float fValue);
PXIALG_API uint8_t PxiAlg_Set_Cal_Flag_HR_Trust_Large_Energy_Num_Th(uint16_t fValue);
PXIALG_API uint8_t PxiAlg_Get_Cal_Flag_HR_Trust_Energy_Ratio(float *fValue);
PXIALG_API uint8_t PxiAlg_Get_Cal_Flag_HR_Trust_Large_Energy_Num_Th(uint16_t *fValue);

PXIALG_API uint8_t PxiAlg_Get_Flag_HR_Trust(uint8_t *flag);

#if (ANDROID_JNI==1)
    PXIALG_API uint8_t PxiAlg_Get_ParameterName(uint32_t index, const char **param);
#endif

PXIALG_API uint8_t PxiAlg_DecodeProcess(uint32_t data, int touch_flag, float time_in);
PXIALG_API uint8_t  PxiAlg_Entrance_0(pah8series_data_t *pah8series_data);
PXIALG_API uint8_t  PxiAlg_Entrance(pah8series_data_t *pah8series_data);
PXIALG_API uint8_t  PxiAlg_EntranceD(pah8series_data_t *pah8series_data);
PXIALG_API uint32_t PxiAlg_Query_Mem_Size();
PXIALG_API uint8_t PxiAlg_GetHrUpdateFlag();

PXIALG_API void PxiAlg_SetData_Seg(pah8series_data_t *pah8series_data);
PXIALG_API uint8_t PxiAlg_Process_Seg(uint8_t *is_finished);
PXIALG_API uint8_t PxiAlg_ProcessD_Seg(uint8_t *is_finished);
#endif /* PXIALG_H__ */