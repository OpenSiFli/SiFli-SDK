#ifndef _SP_ECCFGPARA_H
#define _SP_ECCFGPARA_H

#include "SpxEnhCom.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 麦克风前置增益 */
#define INPUT_GAIN_TALK (4.0f)

typedef struct
{
    short sys_delay;
    short sys_delayNb;
    short AECTap[1];
    float AECMu;
    float WeightFactor;
    float para0;
    float pAECFilterCoef[1][64];
    float pAECFilterCoefNb[1][64];
} g_Cfg_SpEc_TxAec;

typedef struct
{
    float gc_MIN_NonLinearGain;
    float gc_Tx_FdEQ_Tbl_16k[SPX_FrmLen / 2 + 1];
    float gc_Tx_FdEQ_Tbl_8k[SPX_FrmLen / 2 + 1];

#if AEC_NonLP_Enable == 1
    float gc_AecPfMinGain;
    float gc_AecPfRefPwrWeight;
    int   gc_AecPfUppBin;
    int   gc_AecPfLowBin;
#endif
    float gc_DnnPreProGain[81];
    float tx_Drc_Pregain;
    float tx_TgtLevelUpp;
    float tx_Drc_Att;
    float tx_Drc_Decay;
#if (DRC_Mode == 1)
    float tx_Drc_Ratio;
    float tx_TgtLevelSup;
#endif
} g_Cfg_SpEc_TxEnc;

typedef struct
{
    float rx_Drc_Pregain;
    float rx_TgtLevelUpp;
    float rx_Drc_Att;
    float rx_Drc_Decay;
#if (DRC_Mode == 1)
    float rx_Drc_Ratio;
    float rx_TgtLevelSup;
#endif
} g_Cfg_SpEc_Rx;

extern const g_Cfg_SpEc_TxAec gc_Para_TxAecInit;

extern const g_Cfg_SpEc_TxEnc gc_Para_TxEncCfg;

extern const g_Cfg_SpEc_Rx    gc_Para_RxCfg;

#ifdef __cplusplus
}
#endif

#endif
