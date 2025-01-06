#ifndef _SPXENHCOM_H_
#define _SPXENHCOM_H_

#ifdef __cplusplus
extern "C" {
#endif

#define DRC_Mode 1 //0:linear 1:db
//#define _DEBUG_OUTPUTREF_

#define MY_PI      3.1415926535897932f
#define MY_TWO_PI  (2*MY_PI)

#define SPX_Fs_16K 16000        //just support 16k data
#define SPX_Fs_8K 8000        //just support 8k data

/*data structure define*/
#if 1
#define SPX_FrmShf 240
#define SPX_FrmLen 512
#else
#define SPX_FrmShf 120
#define SPX_FrmLen 256
#endif
#define SPX_0dBLevel            32768


#define DMBT_MinPwr (SPX_0dBLevel*SPX_0dBLevel*1.0e-6f)//-60dB
// EPS Pre-Define
#define EPS_MinPwr (SPX_0dBLevel * SPX_0dBLevel * 1.0e-8f)  //-80dB
#define C_SOUND 340  //m/s

#define DMBT_FrmShf SPX_FrmShf
#define DMBT_FFTLEN SPX_FrmLen
#define DMBT_OLALEN (DMBT_FFTLEN - DMBT_FrmShf)

#define  EPS     (2.2204e-16f*SPX_0dBLevel*SPX_0dBLevel) // 2.3841e-7

#define SNDP_DDNSE_ENABLE 0
#define AEC_NonLP_Enable 1

#define ARM_DSP_CONFIG_TABLES
#define ARM_FFT_ALLOW_TABLES
#define ARM_TABLE_TWIDDLECOEF_F32_256
#define ARM_TABLE_BITREVIDX_FLT_256
#define ARM_TABLE_TWIDDLECOEF_F32_256
#define ARM_TABLE_TWIDDLECOEF_RFFT_F32_512
#define ARM_TABLE_TWIDDLECOEF_F32_128
#define ARM_TABLE_BITREVIDX_FLT_128
#define ARM_TABLE_TWIDDLECOEF_F32_128
#define ARM_TABLE_TWIDDLECOEF_RFFT_F32_256

#define ARM_FAST_ALLOW_TABLES
#define ARM_TABLE_SIN_F32

extern float PostGmin;
extern float PostGmax;

#ifdef __cplusplus
}
#endif

#endif
