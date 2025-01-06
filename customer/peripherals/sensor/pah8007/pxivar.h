#ifndef PXIVAR_H__
#define PXIVAR_H__

#include "stdint.h"

#ifndef pah8series_data_t
typedef struct
{
    uint8_t                     frame_count;
    uint32_t                    time;
    uint8_t                     touch_flag;
    uint32_t                    nf_ppg_channel;
    uint32_t                    nf_ppg_per_channel;
    int32_t                     *ppg_data;
    uint32_t                    nf_mems;
    int16_t                     *mems_data;
} pah8series_data_t;
#endif

typedef enum
{
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

typedef enum
{
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

typedef enum
{
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

typedef enum
{
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

#endif /* PXIVAR_H__ */