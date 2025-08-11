
#ifndef __SOUND_FILTER_H__
#define __SOUND_FILTER_H__

#include "sdcommon.h"

#ifdef __cplusplus
extern "C" {
#endif


/** @defgroup Audio Filter library
 * @ingroup ENG
 */
/*@{*/

/* @{@name Define audio version*/
/** Use this to define version string */
/* 注意：如果结构体有修改，则必须修改中版本号 */
#define AUDIO_FILTER_VERSION_STRING        "AudioFilter Version V2.02.00"
/** @} */

// 最后一个兼容的小版本号
#define  AUDIO_FILTER_COMPATIBLE_MINOR       0

typedef enum
{
    _SD_FILTER_UNKNOWN = 0,
    _SD_FILTER_EQ,
    _SD_FILTER_WSOLA,
    _SD_FILTER_RESAMPLE,
    _SD_FILTER_3DSOUND,
    _SD_FILTER_NR,
    _SD_FILTER_AGC,                // obsolete
    _SD_FILTER_VOICECHANGE,        // pitch
    _SD_FILTER_PCMMIXER,
    _SD_FILTER_3DENHANCE,
    _SD_FILTER_MVBASS,
    _SD_FILTER_ASLC,
    _SD_FILTER_TONE_DETECTION,     // detection type. data out is detection results.
    _SD_FILTER_VOLUME_CONTROL,     // volctl
    _SD_FILTER_REECHO,
    _SD_FILTER_MDRC,
    _SD_FILTER_DEVOCAL,
    _SD_FILTER_PCMBUF,
    _SD_FILTER_VAD,                // obsolete
    _SD_FILTER_HOWLING_SUPPRESS,
    _SD_FILTER_AEC,                // accoustic echo cancellation. only used through Echo api
    _SD_FILTER_CRY_DETECT,         // detection type. data out is T_CRY_DETECT_EVENT
    _SD_FILTER_SOUND_LEVEL_DETECT, // detection type. data out is T_SOUND_LEVEL_DETECT_EVENT
    _SD_FILTER_AI_CRY_DETECT,      // detection type. data out is T_AI_CRY_DETECT_EVENT
    _SD_FILTER_DENC,               // d-enc, dual-mic environment noise cancellation
    _SD_FILTER_SSL,                // sound source localization. detection type. data out is T_SSL_EVENT
    _SD_FILTER_LINEARRAY_SSL,      // line array sound source localization. detection type. data out is T_SSL_EVENT
    _SD_FILTER_TONE_SRC,           // sample rate converter
    _SD_FILTER_RNNOISE,
    _SD_FILTER_AIDENOISE,          // AI denoise. user be notified with FILTER_EVENT_NNE_TASK event
    _SD_FILTER_HOWLING_SUPPRESSION2,
    _SD_FILTER_AI_KEYWORD_SPOTTING,  // detection type. data out is T_AI_KEYWORD_SPOTTING_EVENT
    _SD_FILTER_AI_KEYWORD_SPOTTING_ASR,  // detection type. data out is T_AI_KEYWORD_SPOTTING_ASR_EVENT
    _SD_FILTER_SRP_SSL,            // srp sound source localization. detection type. data out is T_SSL_EVENT
    _SD_FILTER_TYPE_MAX,
} T_AUDIO_FILTER_TYPE;

typedef enum _FILTER_EVENT
{
    FILTER_EVENT_BASE = 0x30000000,

    FILTER_EVENT_HOWLING_DETECTION   // param points to T_FILTER_HOWLING_INFO
    = (FILTER_EVENT_BASE | (_SD_FILTER_HOWLING_SUPPRESS << 16) | 0),
    FILTER_EVENT_NNE_TASK            // param points to T_FILTER_NNE_TASK_INFO
    = (FILTER_EVENT_BASE | ((_SD_FILTER_TYPE_MAX + 1)    << 16) | 0),
} T_FILTER_EVENT;

// Filter uses this NOTIFY CB function to send messages to user, in the form of event and its companion param.
// event: see T_FILTER_EVENT
// param: points to an event-specific structure, type of which is mentioned in the comments of T_FILTER_EVENT.
//        this structure is internal buffer. user must not use it outside of the callback.
// cb_notify_param: user specified it in T_AUDIO_FILTER_INPUT when calling _SD_Filter_Open.
// filterId: user assigned it in T_AUDIO_FILTER_INPUT when calling _SD_Filter_Open.
typedef T_VOID(*FILTER_CALLBACK_FUN_NOTIFY)(T_FILTER_EVENT event, T_pVOID param, T_HANDLE cb_notify_param, T_HANDLE filterId);

typedef struct
{
    // compatible with T_AUDIO_CB_FUNS
    MEDIALIB_CALLBACK_FUN_MALLOC                Malloc;
    MEDIALIB_CALLBACK_FUN_FREE                  Free;
    MEDIALIB_CALLBACK_FUN_PRINTF                printf;
    MEDIALIB_CALLBACK_FUN_FLUSH_DCACHE_RANGE    flushDCache;

    FILTER_CALLBACK_FUN_NOTIFY                  notify;
} T_AUDIO_FILTER_CB_FUNS;


// mode: synchronous(0) or asynchronous(1)
// return: result, 1: success, 0: fail
typedef T_S32(*FUN_NNE_TAST_START)(T_HANDLE task_handle, T_S32 mode);
// result: return value of FUN_NNE_TAST_START
typedef T_VOID(*FUN_NNE_TAST_COMPLETE)(T_HANDLE task_handle, T_S32 result);

typedef struct _FILTER_NNE_TASK_INFO
{
    FUN_NNE_TAST_START fun_start;
    FUN_NNE_TAST_COMPLETE fun_complete;
    T_HANDLE task_handle;
} T_FILTER_NNE_TASK_INFO;

typedef enum
{
    _SD_EQ_MODE_NORMAL = 0,
    _SD_EQ_MODE_CLASSIC,
    _SD_EQ_MODE_JAZZ,
    _SD_EQ_MODE_POP,
    _SD_EQ_MODE_ROCK,
    _SD_EQ_MODE_EXBASS,
    _SD_EQ_MODE_SOFT,
    _SD_EQ_USER_DEFINE,
} T_EQ_MODE;

//to define the filter type
typedef enum
{
    FILTER_TYPE_NO = 0,
    FILTER_TYPE_HPF,   // high-pass filter
    FILTER_TYPE_LPF,   // low-pass filter
    FILTER_TYPE_HSF,   // high-shelf filter
    FILTER_TYPE_LSF,   // low-shelf filter
    FILTER_TYPE_PF1    // peaking filter
} T_EQ_FILTER_TYPE;


typedef enum
{
    _SD_WSOLA_0_5 = 0,
    _SD_WSOLA_0_6,
    _SD_WSOLA_0_7,
    _SD_WSOLA_0_8,
    _SD_WSOLA_0_9,
    _SD_WSOLA_1_0,
    _SD_WSOLA_1_1,
    _SD_WSOLA_1_2,
    _SD_WSOLA_1_3,
    _SD_WSOLA_1_4,
    _SD_WSOLA_1_5,
    _SD_WSOLA_1_6,
    _SD_WSOLA_1_7,
    _SD_WSOLA_1_8,
    _SD_WSOLA_1_9,
    _SD_WSOLA_2_0
} T_WSOLA_TEMPO;

typedef enum
{
    _SD_WSOLA_ARITHMATIC_0 = 0, // 0:WSOLA, fast but tone bab
    _SD_WSOLA_ARITHMATIC_1      // 1:PJWSOLA, slow but tone well
} T_WSOLA_ARITHMATIC;


typedef enum
{
    RESAMPLE_ARITHMETIC_0 = 0,  // 0: 音质好，只能固定采样率之间采样
    RESAMPLE_ARITHMETIC_1       // 1: 音质差，可以任意采样率之间采样
} RESAMPLE_ARITHMETIC;

typedef enum
{
    PITCH_NORMAL = 0,
    PITCH_CHILD_VOICE,
    PITCH_MACHINE_VOICE,
    PITCH_ECHO_EFFECT,
    PITCH_ROBOT_VOICE,
    PITCH_RESERVE
} T_PITCH_MODES;

typedef enum
{
    VOLCTL_VOL_MUTIPLE = 0,
    VOLCTL_VOL_DB = 2,
} VOLCTL_VOL_MODE;

typedef struct
{
    T_S32 num;
    struct
    {
        T_S32 x; // x level. use the form of AK32Q15(x.xx) or SD_dBFS_TO_LEVEL(x.xx) to set it.
        T_S32 y; // y level. use the form of AK32Q15(x.xx) or SD_dBFS_TO_LEVEL(x.xx) to set it.
    } stone[10];
} T_FILTER_MILESTONE;

#define    _SD_MDRC_MAX_BANDS  4
typedef struct
{
    T_U8  bands;
    /*
     whether need to bypass total limiter
     0: do total limit;
     1: bypass total limit
    */
    T_U8  limiterBypass;
    /*
     whether does bandi Chorus to output
     0: bandi chorus
     1: bandi doff
    */
    T_U8  bandiDoff[_SD_MDRC_MAX_BANDS];
    /*
     whether does bandi's DRC bypass
     0: do bandi's DRC;
     1: bypass bandi's DRC
    */
    T_U8  bandiDrcBypass[_SD_MDRC_MAX_BANDS];

    /* set boundary freqs */
    T_U32 boundaryFreqs[_SD_MDRC_MAX_BANDS - 1];

    // define bands' drc para
    T_S32 bandsLookAheadTime;  //ms
    struct
    {
        struct
        {
            T_S32 x;
            T_S32 y;
        } stone[2];
        T_S32 gainAttackTime;  //ms
        T_S32 gainReleaseTime;  //ms
    } drcband[_SD_MDRC_MAX_BANDS];

    // define output limiter para
    struct
    {
        struct
        {
            T_S32 x;
            T_S32 y;
        } stone[2];
        T_S32 lookAheadTime;  //ms
        T_S32 gainAttackTime;  //ms
        T_S32 gainReleaseTime;  //ms
    } drctotal;
} T_FILTER_MDRC_PARA;


#define    _SD_EQ_MAX_BANDS    10
struct sd_param_eq
{
    T_U32 eqmode; // T_EQ_MODE

    /*
    设置总增益值(dB)，注意：preGain 赋值形式为 AK16Q10(x.xxx)
    */
    T_S16 preGain;      //-12 <= x.xxx <= 12

    // For User Presets
    T_U32 bands;      //1~10
    T_U32 bandfreqs[_SD_EQ_MAX_BANDS]; // Hz, 20 <= bandfreqs <= sample_rate/2/1.2
    /*
    设置每个频带的增益值(dB)，注意：bandgains 赋值形式为 AK16Q10(x.xxx)
    */
    T_S16 bandgains[_SD_EQ_MAX_BANDS];  // -32.0 < x.xxx < 32.0
    /*
    设置每个频带的Q值，注意：
    - bandQ 赋值形式为 AKU16Q10(x.xxx)
    - bandQ 如果设置为0，则采用库内部的默认值，x.xxx=  1.22
    - x.xxx < 采样率/(2*该频带的中心频点), 并且x.xxx值必须小于64.000
    */
    T_U16 bandQ[_SD_EQ_MAX_BANDS];     // q < sr/(2*f)
    T_U8  bandTypes[_SD_EQ_MAX_BANDS]; // T_EQ_FILTER_TYPE
    /*
    设置每个频带是否生效
    [in]  0: band is enabled
          1: band is skipped
    [out] 0: band is valid and takes effect
          1: band is bypassed because the band param is invalid
    */
    T_U8  bandBypass[_SD_EQ_MAX_BANDS];

    /*
    在调用"_SD_Filter_SetParam()"函数改变EQ参数的时候，模式间的平滑过渡参数
    */
    T_U8  smoothEna;   // 0-不做平滑处理； 1-做平滑处理
    T_U16 smoothTime;  //平滑时间(ms)。若设置0，则采用库内部默认值(256.0*1000/采样率)

    /*** for ffeq dc_remove ***/
    T_U8  dcRmEna;
    T_U32 dcfb;

    /*** for EQ aslc ***/
    T_U8  aslcEna;
    T_U16 aslcLevelMax;

    /*** hw specific params ***/
    T_U8  numFrameDescriptor; // frame descriptor 的个数
    T_U16 frameSize; // frame 大小，samples
};
// use SET_EQ_BAND macro to ease setting EQ params
#define _SET_EQ_BAND_ALL(eq_param, seq, freq_Hz, gain_dB, q, type, bypass) (\
            (eq_param)->bandfreqs[seq]  = (freq_Hz), \
            (eq_param)->bandgains[seq]  = AK16Q10(gain_dB), \
            (eq_param)->bandQ[seq]      = AKU16Q10(q), \
            (eq_param)->bandTypes[seq]  = (type), \
            (eq_param)->bandBypass[seq] = (bypass), \
            seq)
#define SET_EQ_BAND(eq_param, seq, freq_Hz, gain_dB, q, type) _SET_EQ_BAND_ALL(eq_param, seq, freq_Hz, gain_dB, q, type, 0)

struct sd_param_devocal
{
    T_U16 frameSize; // frame 大小，samples
    T_U16 bassFreq;  // 低频
    T_U16 trebleFreq;// 高频
    T_U16 strength; // 1~5, bigger is more cancelling
};
struct sd_param_wsola
{
    T_U32 tempo;            // T_WSOLA_TEMPO
    T_U32 arithmeticChoice; // T_WSOLA_ARITHMATIC
};
struct sd_param_3dsound
{
    T_U8 is3DSurround;
};

#define SD_VERSION_RESAMPLE   (sizeof(struct sd_param_resample))
struct sd_param_resample
{
    T_U32 outSampleRate; // 输出采样率

    //设置最大输入长度(bytes)，open时需要用作动态分配的依据。
    //后面具体调用重采样时，输入长度不能超过这个值
    T_U32 maxinputlen;

    T_U32 reSampleArithmetic; // RESAMPLE_ARITHMETIC
    T_U32 outChannel;
};
struct sd_param_agc
{
    T_U16 AGClevel;  // make sure AGClevel < 32767
    /* used in AGC_1 */
    T_U32  max_noise;
    T_U32  min_noise;
    /* used in AGC_2 */
    T_U8  noiseReduceDis;  // 是否屏蔽自带的降噪功能
    T_U8  agcDis;  // 是否屏蔽自带的AGC功能
    /*
    agcPostEna：在agcDis==0的情况下，设置是否真正的AGC2库里面做AGC：
    0：表示真正在库里面做agc，即filter_control出来的数据是已经做好agc的；
    1: 表示库里面只要计算agc的gain值，不需要真正做agc处理；真正的agc由外面的调用者后续处理
    */
    T_U8  agcPostEna;
    T_U16 maxGain;  // 最大放大倍数
    T_U16 minGain;  // 最小放大倍数
    T_U32 dc_freq;  // hz
    T_U32 nr_range; // 1~300,越低降噪效果越明显
};
struct sd_param_nr
{
    T_U32   nrEna;            // enable noise reduction
    T_U32   agcEna;           // enable AGC
    T_U32   agcLevel;         // agc's target level, 0: use default. use the form of AK32Q15(x.xx) or SD_dBFS_TO_LEVEL(x.xx) to set it.
    T_U16   maxGain;          // agc's max_gain, Q0
    T_U16   minGain;          // agc's min_gain. use the form of AK16Q10(x.xx) to set it.
    T_S16   noiseSuppressDb;  // attenuation of noise in dB (negative number), 0: use default
    T_S16   nearSensitivity;  // sensitivity of near-end speech [1:100], 0: use default
};
struct sd_param_howlingSuppress
{
    T_U32   hsEna;            // enable howling suppression
    T_S16   phprThr;          // PHPR detection threshold value
    T_S16   minphprThr;       // PHPR detection min threshold value
    T_S16   ipmpThr;          // IPMP detection threshold value
    T_S16   ptprThr;          // PTPR detection threshold value
    T_S16   minptprThr;       // PTPR detection min threshold value
    T_S16   pnprThr;          // PNPR detection threshold value
    T_S16   minpnprThr;       // PNPR detection min threshold value
    T_S16   nbanks;           // design filter bank
    T_S16   bandwidth;        // design filter bandwidth
    T_S16   depthDb;          // design filter depth(dB)
    T_U16   SF1;              // detection algorithm1:howl candidate suatain frames
    T_U16   SF2;              // detection algorithm2:howl candidate suatain frames
};
struct sd_param_howlingSuppression2
{
    T_U16   hsEna;            // enable howling suppression2
    T_S16   paprThr;          // PAPR detection threshold(dB), 0: use default
    T_S16   pnprThr;          // PNPR detection threshold(dB), 0: use default
    T_S16   phprThr;          // PHPR detection threshold(dB), 0: use default
    T_U16   ipmpThr;          // IPMP detection threshold, suggest to set 3, 0: use default
    T_U16   ipmpCanNumFrames; // IPMP candidate frames numbers, suggest to set 5, ipmpThr < ipmpCanNumFrames, larger ipmpCanNumFrames need more resource, 0: use default
    T_U16   durationMs;       // howling suppressing duration in ms, 0: use default
    T_S16   ptprThr;          // PTPR detection threshold(dB), 0, reserved
    T_S16   imsdThr;          // IMSD detection threshold(dB), 0, reserved
    T_U16   imsdHisNumFrames; // IMSD history frames numbers,  0, reserved, larger imsdHisNumFrames need more resource
};

typedef struct _FILTER_HOWLING_INFO
{
    T_U32 howlingFreq;          // howling frequency
    T_U8  trigger;              // trigger of howling event, 1: howling freq arises, 0: howling freq falls
    T_U8  nHowlingFreq;         // number of howling frequencies still active
} T_FILTER_HOWLING_INFO;

struct sd_param_pitch
{
    T_U32 pitchMode;  // T_PITCH_MODES
    /*
     只在 PITCH_CHILD_VOICE==pitchMode 时，pitchTempo参数生效。
     pitchTempo 参数的范围是[0-10], 0~4提高音调，5正常音调，6~10降低音调
    */
    T_U8      pitchTempo;
};
struct sd_param_reecho
{
    /*是否使能混响效果，1为使能，0为关闭*/
    T_S32 reechoEna;
    /*
    衰减因子，格式为 AK32Q10(0.xx)
    */
    T_S32 degree;      //0-无混响效果
    /*
    设置房间大小，建议设置0-300
    */
    T_U16 roomsize;    //0-采用默认值(71)。
    /*
    设置最长混响时间(ms)，即多长时间后混响消失。
    注意：这个设置的越长，需要的缓冲就越大，所以不建议设置太大。
          一般建议设置1000以内，如果内存足够，也可以设置大些。
          如果内存不够，则减小这个值。
    */
    T_U16 reechoTime;  //0-采用默认值(840)
    /*
    是否需要把原始主声音同时输出。
    0: 不输出原始主声音，输出的都是反射之后的声音；
    1：主声音和反射之后的声音一起输出。
    */
    T_U8  needMainBody; //0 or 1
};
struct sd_param_3DEnhance
{
    /*
    设置总增益值(dB)，
    注意：preGain 赋值形式为 AK16Q10(x.xxx)，
    限制 -12 <= x.xxx <= 12
    */
    T_S16 preGain;
    T_S16 cutOffFreq;
    /*
    设置3D深度，
    注意: depth赋值形式为 AK16Q10(x.xxx),
    限制 -1 < x.xxx < 1
    */
    T_S16 depth;
};
struct sd_param_mvBass
{
    /*
    设置总增益值(dB)，
    注意：preGain 赋值形式为 AK16Q10(x.xxx)，
    限制 -12 <= x.xxx <= 12
    */
    T_S16 preGain;
    T_S16 cutOffFreq;
    /*
    设置增强幅度(dB)，
    注意: bassGain 赋值形式为 AK16Q10(x.xxx),
    限制 0 < x.xxx < 12
    */
    T_S16 bassGain;
    /*** for MVBass's aslc ***/
    T_U8   aslcEna;
    T_U16  aslcLevelMax;
};
struct sd_param_aslc
{
    T_BOOL aslcEna;
    T_U16  aslcLimitLevel;  // 限幅的最大幅度, pcm value in y axis. eg, AKU16Q15(0.xx), SD_dBFS_TO_LEVEL(-x.xx)
    T_U16  aslcStartLevel;  // 限幅的起始幅度, pcm value in y axis. eg, AKU16Q15(0.xx), SD_dBFS_TO_LEVEL(-x.xx). 0: same as aslcLimitLevel
    T_S32  vol_dB;          // volume in dB, Q10, use the form of AK32Q10(x.xx) to set it.
    // -40 ~ 20 dB is recommended. less than -90dB actually mutes the signal.
    /*
    jointChannels:
       0: 各声道独立控制增益
       2: 两个声道采用统一的增益控制
    */
    T_U16  jointChannels;

    /*
     maxLenin：设置最大的输入pcm数据长度，字节为单位。
     为了在设置参数的时候，能预先按照最大长度分配好内存。
     解决数据长度发生改变时多次分配内存，导致内存分配失败的问题
    */
    T_U16  maxLenin;
    T_S32  lookAheadTime;   //ms
    T_S32  gainAttackTime;  //ms
    T_S32  gainReleaseTime; //ms

    int32_t aslcPreset; // 0: limiter, this is the basic type, not use milestone
    // 1: noise gate
    //   when milestone.num is 0, use default noise gate, aslcLimitLevel and vol_dB is applied
    //   others: milestone.num must be 5, use it to define gate threshold and limit, vol_dB is applied
    // 2: user defined by milestone
    T_FILTER_MILESTONE milestone;
};
struct sd_param_volumeControl
{
    /*
    set volume mode:
    VOLCTL_VOL_MUTIPLE: 音量值是 volume 的值，即外部传入音量倍数值
    VOLCTL_VOL_DB:      音量值是 voldb 的值， 即外部出入dB值
    */
    T_U16 setVolMode;

    /*
    设置音量倍数值, 赋值形式为 AKU16Q10(x.xx), x.xx=[0.00~7.99]表示倍数
    建议设置的音量值不要超过 1.00，因为超过可能会导致数据溢出，声音产生失真
    */
    T_U16 volume;

    /*
    设置音量dB, 赋值形式为 AK32Q10(x.xx), x.xx=[-60.00~8.00]
    建议设置的音量值不要超过0dB，因为超过可能会导致数据溢出，声音产生失真
    若 x.xx<=-79dB, 则输出无声；若 x.xx>8.0, 可能会导致输出噪音。
    */
    T_S32 voldb;

    /* 为了防止音量变换过程产生pipa音，对音量进行平滑处理，这里设置平滑的过渡时间 */
    T_U16 volSmoothTime;  //ms
};
struct sd_param_toneDetection
{
    T_U32 baseFreq;
};
struct sd_param_mdrc
{
    /*
     maxLenin：设置最大的输入pcm数据长度，字节为单位。
     为了在设置参数的时候，能预先按照最大长度分配好内存。
     解决数据长度发生改变时多次分配内存，导致内存分配失败的问题
    */
    T_U16  maxLenin;
};
struct sd_param_pcmbuf
{
    T_U32 InputSampleRate;  // sample rate of input pcm. 0: same as nominal m_SampleRate
    T_U32 OutputSampleRate; // sample rate of output pcm. 0: same as nominal m_SampleRate
    T_U32 bufSize;          // buffer size in bytes. 0: use default or not changed
    T_U16 frameSize;        // frame size in bytes. 0: use default or not changed

    // This bound defines a safe region of data size, out of which will
    // trigger aggressive pcm skipping.
    T_U32 upperBound;       // upper bound in bytes. 0: no pcm skipping
};

#define SD_VERSION_SOUND_LEVEL_DETECT   (sizeof(struct sd_param_soundLevelDetect) + sizeof(T_SOUND_LEVEL_DETECT_EVENT))
struct sd_param_soundLevelDetect
{
    T_S32   framelen;         // number of samples in a frame. 0: use default
    T_U32   flag;             // bit[15]: probe flag. When this bit is set, thresh in T_SOUND_LEVEL_DETECT_EVENT just
    //          matches the value for the current signal to be detected as active in the designated level.
    T_S32   thresh;           // threshold of signal amplitude. use the form of AK32Q15(x.xx) or SD_dBFS_TO_LEVEL(x.xx) to set it
    T_S32   level;            // sensitivity level. [0, 4], smaller is more sensitive.
};

typedef struct _SOUND_LEVEL_DETECT_EVENT
{
    T_S8  trigger;            // this event is triggered by 0: active status change, 1: probe
    T_S8  active;             // 1: voice is active, 0: voice is not active
    T_S32 probedThresh;       // when probe flag is set, this is a calibrated value. others: not defined
} T_SOUND_LEVEL_DETECT_EVENT;

#define SD_VERSION_CRY_DETECT   (sizeof(struct sd_param_cryDetect) + sizeof(T_CRY_DETECT_EVENT))
struct sd_param_cryDetect
{
    T_S32   framelen;         // number of samples in a frame. 0: use default
    T_S32   thresh;           // threshold of signal amplitude. use the form of AK32Q15(x.xx) or SD_dBFS_TO_LEVEL(x.xx) to set it
    T_S32   level;            // sensitivity level. [0, 4], smaller is more sensitive.
};

typedef struct _CRY_DETECT_EVENT
{
    T_S8  active;             // 1: voice is active, 0: voice is not active
} T_CRY_DETECT_EVENT;

#define SD_VERSION_AICRY_DETECT   (sizeof(struct sd_param_AICryDetect) + sizeof(T_AI_CRY_DETECT_EVENT))
struct sd_param_AICryDetect
{
    T_U16 frameSize;    // 帧长度     samples. 0: use default
    T_U16 frameShift;   // 帧步进长度 samples. 0: use default
    T_U16 fftSize;      // FFT 长度   samples. 0: use default
    T_U16 nBands;       // 滤波器组频带数 0: use default
    T_U16 nFrames;      // 帧的总数目   0: use default
    T_U32 lowFreq;      // 最低频率 0: use default
    T_U32 highFreq;     // 最高频率 0: use default

    T_CHR *modelName;   // 哭声检测模型名称 0: use default "model_crydet"
    T_S32 modelSize;    // 哭声检测模型的长度. must be 0

    T_U32 flag;         // bit[15]: probe flag. when this bit is set, output cry probability in T_AI_CRY_DETECT_EVENT
    T_S32 thresh;       // reserved, not used
    T_S32 level;        // 哭声检测的误报灵敏度等级,数值越大,误报越低. 数值范围 [0-5], 0: use default level 0
};

typedef struct _AI_CRY_DETECT_EVENT
{
    T_S8  trigger;            // this event is triggered by 0: active status change, 1: probe
    T_S8  active;             // 1: voice is active, 0: voice is not active
    T_S16 probability;        // when probe flag is set, this is the cry label softmax probability. others: not defined
} T_AI_CRY_DETECT_EVENT;


#define MAX_MIC_NUM 4
#define MAX_INTERF_NUM 4

// direction of the axis is reletive to the camera's perspective.
// x: to the right
// y: forwards
// z: upwards
typedef struct
{
    T_S32 x; // m. use the form of AK32Q15(x.xx)
    T_S32 y; // m. use the form of AK32Q15(x.xx)
    T_S32 z; // m. use the form of AK32Q15(x.xx)
} T_MIC_POSITION;

// The convention used:
// azimuth: zero is to the right from the camera's perspective, with positive
//          angles in radians counter-clockwise.
// elevation: zero is horizontal, with positive angles in radians upwards.
// radius: distance from the camera in meters.
typedef struct
{
    T_S32 azimuth;   // 方位角(rad) = this * pi/2. use the form of AK32Q15(x.xx)
    T_S32 elevation; // 仰角(rad) = this * pi/2. use the form of AK32Q15(x.xx)
    T_S32 radius;    // 距离(m). use the form of AK32Q15(x.xx)
} T_SOUND_SPHERICAL_DIRECTION;

typedef struct
{
    T_U32 numMic; // number of mic
    T_MIC_POSITION micsPos[MAX_MIC_NUM];
} T_ARRAY_GEOMETRY;

struct sd_param_denc
{
    T_U32 mode; // reserved for future use. should be 0
    T_ARRAY_GEOMETRY arrayGeometry;
    T_SOUND_SPHERICAL_DIRECTION targetSpherical;
    T_U32 numInterf; // number of interference
    T_SOUND_SPHERICAL_DIRECTION interfSpherical[MAX_INTERF_NUM];
};

#define SD_VERSION_SSL   (sizeof(struct sd_param_ssl) + sizeof(T_SSL_EVENT))

struct sd_param_ssl
{
    T_U16 frameSize; // frame size in bytes
    T_U16 soundSpeed; // speed of sound
    T_U32 sourcesNumbers; // number of sound sources
    T_ARRAY_GEOMETRY arrayGeometry;
};

typedef struct _SSL_EVENT
{
    T_U32 sourcesNumbers; // number of sound sources
    T_SOUND_SPHERICAL_DIRECTION soundSourceDirection[MAX_MIC_NUM - 1];
} T_SSL_EVENT;

#define SD_VERSION_LINEARRAY_SSL   (sizeof(struct sd_param_linearray_ssl) + sizeof(T_SSL_EVENT))
struct sd_param_linearray_ssl
{
    T_U16 frameSize;    // frame size in samples
    T_U16 soundSpeed;   // m. speed of sound
    T_U16 numMic;       // numbers of mics
    T_U16 micSpace;     // m. use the form of AK16Q15(x.xx), interval of mics
    // for sound level detect
    T_S32 thresh;       // threshold of signal amplitude. use the form of AK32Q15(x.xx) or SD_dBFS_TO_LEVEL(x.xx) to set it
    T_S32 level;        // sensitivity level. [0, 4], smaller is more sensitive.
};

#define MAX_SINGLE_FREQUENCY_NUM    (30)
#define SD_VERSION_SRP_SSL   (sizeof(struct sd_param_srp_ssl) + sizeof(T_SSL_EVENT))
typedef struct
{
    T_U16 numFreqHz;                        // number of single frequency(Hz)
    T_U32 freqHz[MAX_SINGLE_FREQUENCY_NUM]; // value of single frequency(Hz)
} T_SINGLE_FREQ_HZ;

struct sd_param_srp_ssl
{
    T_U16 dim;              // spatial dimension of sound source, values are 2 and 3
    T_U16 freqMode;         // 0: frequency range(Hz)  1: single frequency(Hz)  2: all frequency
    T_U16 frameSize;        // frame size in samples
    T_U16 soundSpeed;       // m. speed of sound
    T_U16 srcNums;          // numbers of sound source
    T_U16 numSnap;          // output the sound source localization results every numSnap frames
    // for frequency range(Hz)
    T_U32 minFreqHz;        // the minimum frequency(Hz)
    T_U32 maxFreqHz;        // the maximum frequency(Hz)
    // for single frequency(Hz)
    T_SINGLE_FREQ_HZ singleFreqHz;
    // microphone array information
    T_ARRAY_GEOMETRY arrayGeometry;
    // for sound level detect
    T_S32 thresh;       // threshold of signal amplitude. use the form of AK32Q15(x.xx) or SD_dBFS_TO_LEVEL(x.xx) to set it
    T_S32 level;        // sensitivity level. [0, 4], smaller is more sensitive.
};


struct sd_param_toneSRC
{
    T_S32 input_sr;         // sample rate of input pcm. 0: same as nominal m_SampleRate
    T_S32 output_sr;        // sample rate of output pcm
    T_U16 input_framelen;   // frame size, in samples of input pcm
};

struct sd_param_rnnoise
{
    T_U32   rnnoiseEna;            // enable rnn noise reduction
    T_S32   rnnoiseSampleRate;     // on which sample rate to do rnnoise. 0: same as input
};

#define SD_VERSION_AI_DENOISE   (sizeof(struct sd_param_AIDenoise) + sizeof(T_FILTER_NNE_TASK_INFO))
struct sd_param_AIDenoise
{
    T_U32  aidenoiseEna;    // enable AI denoise
    T_U16  sampleRate;

    T_U16  frameShift;      // 帧步进长度 samples. 0: use default
    T_U16  frameSize;       // 帧长度     samples. 0: use default
    T_U16  fftSize;         // FFT 长度   samples. 0: use default
    T_U16  numFrameDescriptor; // frame descriptor 的个数

    T_CHR *modelName;       // AI降噪模型名称 0: use default "model_aidenoise"
    T_S32  modelSize;       // AI降噪模型的长度. must be 0
};


#define SD_VERSION_AI_KEYWORD_SPOTTING (sizeof(struct sd_param_AIKeywordSpotting) + sizeof(T_AI_KEYWORD_SPOTTING_EVENT))
struct sd_param_AIKeywordSpotting
{
    T_U32  aikwsEna;        // enable AI keyword spotting
    T_U16  nKeywords;       // 关键词数量
    T_U16  sampleRate;

    T_U16  frameShift;      // 帧步进长度 samples. 0: use default
    T_U16  frameSize;       // 帧长度     samples. 0: use default
    T_U16  fftSize;         // FFT 长度   samples. 0: use default
    T_U16  nFrames;
    T_U16  nBands;

    T_CHR *modelName;       // AI关键词检测模型名称. 0: use default "model_aikws"
    T_S32  modelSize;       // AI关键词检测模型的长度. must be 0
};

typedef struct _AI_KEYWORD_SPOTTING_EVENT
{
    T_S16 keyword;            // keyword number
    T_S16 probability;        // when probe flag is set, this is the keyword label softmax probability. others: not defined
} T_AI_KEYWORD_SPOTTING_EVENT;


#define SD_VERSION_AI_KEYWORD_SPOTTING_ASR (sizeof(struct sd_param_AIKeywordSpottingASR) + sizeof(T_AI_KEYWORD_SPOTTING_ASR_EVENT))
struct sd_param_AIKeywordSpottingASR
{
    T_U32  aikwsEna;        // enable AI keyword spotting ASR
    T_U16  nKeywords;       // 关键词数量
    T_U16  sampleRate;

    T_U16  frameShift;      // 帧步进长度 samples. 0: use default
    T_U16  frameSize;       // 帧长度     samples. 0: use default
    T_U16  fftSize;         // FFT 长度   samples. 0: use default
    T_U16  nFrames;
    T_U16  nBands;

    T_CHR *modelName;       // AI关键词检测模型名称. 0: use default "model_aikwsasr"
    T_S32  modelSize;       // AI关键词检测模型的长度. must be 0
};

typedef struct _AI_KEYWORD_SPOTTING_ASR_EVENT
{
    T_S16 keyword;            // keyword number
    T_S16 probability;        // when probe flag is set, this is the keyword label softmax probability. others: not defined
} T_AI_KEYWORD_SPOTTING_ASR_EVENT;


typedef struct
{
    T_U32    m_Type;             //T_AUDIO_FILTER_TYPE
    T_U32    m_SampleRate;       //sample rate, sample per second
    T_U16    m_Channels;         //channel number
    T_U16    m_BitsPerSample;    //bits per sample
    T_U8     reserved;

    T_U32    m_PrivateSize;      // set to sizeof(sd_param_xxx)
    union
    {
        struct sd_param_eq                    m_eq;
        struct sd_param_devocal               m_devocal;
        struct sd_param_wsola                 m_wsola;
        struct sd_param_3dsound               m_3dsound;
        struct sd_param_resample              m_resample;
        struct sd_param_nr                    m_nr;
        struct sd_param_pitch                 m_pitch;
        struct sd_param_reecho                m_reecho;
        struct sd_param_3DEnhance             m_3DEnhance;
        struct sd_param_mvBass                m_mvBass;
        struct sd_param_aslc                  m_aslc;
        struct sd_param_volumeControl         m_volumeControl;
        struct sd_param_toneDetection         m_toneDetection;
        struct sd_param_mdrc                  m_mdrc;
        struct sd_param_pcmbuf                m_pcmBuf;
        struct sd_param_howlingSuppress       m_howlingSuppress;
        struct sd_param_soundLevelDetect      m_soundLevelDetect;
        struct sd_param_cryDetect             m_cryDetect;
        struct sd_param_denc                  m_denc;
        struct sd_param_AICryDetect           m_AICryDetect;
        struct sd_param_ssl                   m_ssl;
        struct sd_param_linearray_ssl         m_linearray_ssl;
        struct sd_param_toneSRC               m_toneSRC;
        struct sd_param_rnnoise               m_rnnoise;
        struct sd_param_AIDenoise             m_AIDenoise;
        struct sd_param_howlingSuppression2   m_howlingSuppression2;
        struct sd_param_AIKeywordSpotting     m_AIKeywordSpotting;
        struct sd_param_AIKeywordSpottingASR  m_AIKeywordSpottingASR;
        struct sd_param_srp_ssl               m_srp_ssl;
    } m_Private;
} T_AUDIO_FILTER_IN_INFO;

typedef struct
{
    const char              *strVersion; // set to AUDIO_FILTER_VERSION_STRING
    T_AUDIO_FILTER_CB_FUNS   cb_fun;
    T_HANDLE                 cb_notify_param;
    T_HANDLE                 filterId;
    T_AUDIO_FILTER_IN_INFO   m_info;

    const T_VOID            *ploginInfo;
} T_AUDIO_FILTER_INPUT;

typedef T_U64 T_AUDIO_FILTER_TS;

typedef struct
{
    T_VOID *buf_in;   // [in]  ptr to input buffer. filter will consume input data from here.
    // [out] ptr to unconsumed input data
    T_U32   len_in;   // [in]  size of input data in bytes
    // [out] size of unconsumed input data, in bytes
    T_U32   flag_in;  // bit[0]: end flag
    //   [in]  input data are the end of the stream.
    //   [out] cleared only if all input data are consumed.
    T_VOID *meta_in;  // [in]  user can specify a handle of metadata here. filter just passes this handle out to meta_out.

    T_VOID *buf_out;  // [in]  ptr to output buffer
    T_U32   len_out;  // [in]  size of output buffer in bytes
    T_VOID *meta_out; // [out] filter passes out a handle of metadata. see comment of meta_in.

    T_VOID *buf_in2;  // for mix pcm samples
    T_U32   len_in2;
} T_AUDIO_FILTER_BUF_STRC;

typedef struct
{
    T_AUDIO_FILTER_CB_FUNS cb;
    T_U32    m_Type;
} T_AUDIO_FILTER_LOG_INPUT;

//////////////////////////////////////////////////////////////////////////

/**
 * @brief    获取音效库版本信息.
 * @param    [in] T_VOID
 * @return   T_CHR *
 * @retval   返回库版本号字符串
 */
T_CHR *_SD_GetAudioFilterVersionInfo(T_VOID);

/**
 * @brief    检查头文件版本是否与库版本匹配
 * @param    [in] filter_input
 * @return   T_S32
 * @retval   T_TRUE or T_FALSE
 */
T_S32 _SD_CheckAudioFilterVersion(T_AUDIO_FILTER_INPUT *filter_input);

/**
 * @brief    设置音效库的 debug zones，选择打印信息的类型
 * @param    [in] bit-or of SD_ZONE_ID_XXX.
 *                Default is SD_DEFAULT_DEBUG_ZONES
 * @return   T_VOID
 */
T_VOID _SD_Filter_SetDebugZones(T_U32 debugZones);

/**
 * @brief    打开音效处理设备.
 * @param    [in] filter_input: 音效处理的输入结构
 * @return   T_VOID *
 * @retval   音效处理设备句柄
 *           AK_NULL: open 失败
 */
T_VOID *_SD_Filter_Open(T_AUDIO_FILTER_INPUT *filter_input);

/**
 * @brief    音效处理.
 * @param    [in]      audio_filter: 音效处理设备句柄
 * @param    [in][out] audio_filter_buf: 输入输出buffer结构
 * @return   T_S32
 * @retval   > 0: 音效库处理后的音频数据大小，以byte为单位
 *           <=0: T_SD_ERROR_CODE
 */
T_S32 _SD_Filter_Control(T_VOID *audio_filter, T_AUDIO_FILTER_BUF_STRC *audio_filter_buf);

/**
 * @brief    关闭音效处理设备.
 * @param    [in] audio_filter: 音效处理设备句柄
 * @return   T_S32
 * @retval   AK_TRUE :  关闭成功
 *           AK_FALSE:  关闭异常
 */
T_S32 _SD_Filter_Close(T_VOID *audio_filter);

T_S32 _SD_Filter_Reset(T_VOID *audio_filter);

const T_CHR *_SD_Filter_GetName(T_VOID *audio_filter);
// return T_AUDIO_FILTER_TYPE
T_U32 _SD_Filter_GetType(T_VOID *audio_filter);

/**
 * @brief    设置音效参数.
 *           如果m_SampleRate,m_BitsPerSample,m_Channels三个有1个为0,则不改变任何音效,返回AK_TRUE
 * @param    [in]      audio_filter: 音效处理设备句柄
 * @param    [in][out] info: 音效信息保存结构. 返回时，更新为实际生效的参数.
 * @return   T_S32
 * @retval   AK_TRUE :  设置成功
 *           AK_FALSE:  设置异常
 */
T_S32 _SD_Filter_SetParam(T_VOID *audio_filter, T_AUDIO_FILTER_IN_INFO *info);

/**
 * @brief    获取音效参数
 * @param    [in] audio_filter: 音效处理设备句柄
 * @param    [out] info: 音效信息保存结构
 * @return   T_S32
 * @retval   AK_TRUE :  获取成功
 *           AK_FALSE:  获取异常
 */
T_S32 _SD_Filter_GetParam(T_VOID *audio_filter, T_AUDIO_FILTER_IN_INFO *info);

/**
 * @brief    设置ASLC模块的静音检测参数之静音幅度阈值.
 * @param    [in] audio_filter: 音效处理设备句柄
 * @param    [in] silenceLevel: 静音阈值，当pcm幅值小于这个数值认为是静音
 * @return   T_S32
 * @retval   AK_TRUE :  设置成功
 * @retval   AK_FALSE:  设置异常
 */
T_S32 _SD_Filter_SetAslcSilenceLevel(T_VOID *audio_filter, T_U32 silenceLevel);

/**
 * @brief    设置ASLC模块的静音检测参数之连续静音时间阈值.
 * @param    [in] audio_filter: 音效处理设备句柄
 * @param    [in] silenceTime: 静音时间阈值，当连续静音时间超过这个阈值，认为真正进入静音状态
 * @return   T_S32
 * @retval   AK_TRUE :  设置成功
 *           AK_FALSE:  设置异常
 */
T_S32 _SD_Filter_SetAslcSilenceTime(T_VOID *audio_filter, T_U32 silenceTime);

/**
 * @brief    设置MDRC模块的限幅曲线.
 * @param    [in] audio_filter: 音效处理设备句柄
 * @param    [in] fmdrc: MDRC的参数
 * @return   T_S32
 * @retval   AK_TRUE :  设置成功
 *           AK_FALSE:  设置异常
 */
T_S32 _SD_Filter_SetMdrcPara(T_VOID *audio_filter, T_FILTER_MDRC_PARA *fmdrc);

/**
 * @brief    快速重采样
 * @param    [in]  audio_filter: 音效处理设备句柄
 * @param    [out] dstData: 输出的pcm数据
 * @param    [in]  srcData: 输入的pcm数据
 * @param    [in]  srcLen:  输入pcm数据的byte数
 * @return   T_S32
 * @retval   >=0 :  重采样后的输出pcm数据的byte数
 *           <0  :  重采样失败
 */
T_S32  _SD_Filter_Audio_Scale(T_VOID *audio_filter, T_S16 dstData[], T_S16 srcData[], T_U32 srcLen);


/**
* @brief     把EQ的频域参数转为时域参数.
* @param     [in] audio_filter: 音效处理设备句柄
* @param     [in] info: 音效信息保存结构
* @return    T_VOID *
* @retval    EQ库时域参数的指针
*            AK_NULL: 获取失败
*/
T_VOID *_SD_Filter_GetEqTimePara(T_VOID *audio_filter, T_AUDIO_FILTER_IN_INFO *info);

/**
* @brief     设置EQ时域参数
* @param     [in] audio_filter: 音效处理设备句柄
* @param     [in] peqTime: 时域参数的指针
* @return    T_S32
* @retval    AK_TRUE :  设置成功
*            AK_FALSE:  设置异常
*/
T_S32 _SD_Filter_SetEqTimePara(T_VOID *audio_filter, T_VOID *peqTime);

/**
* @brief     释放EQ时域参数占用的空间.
* @param     [in] audio_filter: 音效处理设备句柄
* @param     [in] peqTime: 时域参数的指针
* @return    T_S32
* @retval    AK_TRUE :  设置成功
*            AK_FALSE:  设置异常
*/
T_S32 _SD_Filter_DestoryEqTimePara(T_VOID *audio_filter, T_VOID *peqTime);


/**
 * @brief    设置模块的音量值. 对 VOLUME_CONTROL 和 ASLC 有效.
 * @param    [in] audio_filter: 音效处理设备句柄
 * @param    [in] volume: 目标音量值。
 *                音量倍数值, 赋值形式为 AKU16Q10(x.xx), x.xx=[0.00, 7.99]
 *                建议x.xx不要超过 1.00，否则可能导致数据溢出，声音产生失真
 * @return    T_S32
 * @retval    AK_TRUE :  设置成功
 *            AK_FALSE:  设置异常
 */
T_S32 _SD_Filter_SetVolume(T_VOID *audio_filter, T_U16 volume);

/**
 * @brief    设置模块的音量值. 对 VOLUME_CONTROL 和 ASLC 有效.
 * @param    [in] audio_filter: 音效处理设备句柄
 * @param    [in] vol_dB: 目标音量dB值。
 *                音量dB, 赋值形式为 AK32Q10(x.xx), x.xx=[-100.00, 8.00]
 *                建议设置的音量值不要超过 0dB，否则可能导致数据溢出，声音产生失真
 *                若 x.xx<=-79dB, 则输出无声；若 x.xxx>8.0, 可能会导致输出噪音。
 * @return   T_S32
 * @retval   AK_TRUE :  设置成功
 *           AK_FALSE:  设置异常
 */
T_S32 _SD_Filter_SetVolumeDB(T_VOID *audio_filter, T_S32 vol_dB);

/**
 * @brief    获取模块内部缓存的数据量(byte)，以输出格式计
 * @param    [in] audio_filter: 音效处理设备句柄
 * @return   T_S32
 * @retval   >=0: 获取到的大小
 * @retval   <0:  获取失败
 */
T_S32 _SD_Filter_GetDataSize(T_VOID *audio_filter);

/**
* @brief     设置 VOLUME_CONTROL 音量变化的平滑时间
* @param     [in] audio_filter: 音效处理设备句柄
* @param     [in] stime: 要设置的平滑时间，单位ms，指无声到0dB需要的时间。
* @return    T_S32
* @retval    AK_TRUE :  设置成功
*            AK_FALSE:  设置异常
*/
T_S32 _SD_Filter_Volctl_SetSmoothTime(T_VOID *audio_filter, T_U32 stime);

/**
* @brief     获取 VOLUME_CONTROL 当前正在生效的音量值
* @param     [in] audio_filter: 音效处理设备句柄
* @return    T_S32
* @retval    >=0:  获取到的音量倍数值, Q10格式
*            <0 :  获取失败
*/
T_S32 _SD_Filter_Volctl_GetCurVolume(T_VOID *audio_filter);

/**
* @brief     获取 PcmBuf 内部缓存的数据量(byte)
* @param     [in] audio_filter: 音效处理设备句柄
* @return    T_S32
* @retval    >=0:  获取到的大小, 单位byte
*            <0 :  获取失败
*/
#define _SD_Filter_PcmBuf_GetDataSize(audio_filter) _SD_Filter_GetDataSize(audio_filter)

/**
* @brief     获取 PcmBuf 内部缓存的空余空间(byte)
* @param     [in] audio_filter: 音效处理设备句柄
* @return    T_S32
* @retval    >=0:  获取到的大小, 单位byte
*            <0 :  获取失败
*/
T_S32 _SD_Filter_PcmBuf_GetFreeSize(T_VOID *audio_filter);


//////////////////////////////////////////////////////////////////////////
// module logins

const T_VOID *_SD_EQ_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_3DEnhance_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_3DSound_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_ASLC_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_mvBass_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_NR_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_VolCtl_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_WSOLA_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_pitch_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_Mixer_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_Reecho_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_Resample_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_toneDetection_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_MDRC_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_Devocal_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_PcmBuf_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_HowlingSuppress_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_HowlingSuppression2_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_AEC_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_CryDetect_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_SoundLevelDetect_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_Denc_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_AICryDetect_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_SSL_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_LineArraySSL_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_SrpSSL_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_toneSRC_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_Rnnoise_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_AIDenoise_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_AIKeywordSpotting_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);
const T_VOID *_SD_AIKeywordSpottingASR_login(T_AUDIO_FILTER_LOG_INPUT *plogInput);

//////////////////////////////////////////////////////////////////////////
// Echo API

typedef enum _ECHO_EVENT
{
    ECHO_EVENT_NONE = 0,
    ECHO_EVENT_VOICE_ACTIVE_CHANGE, // param points to T_ECHO_VAD_INFO
    ECHO_EVENT_VOICE_ACTIVE_PROBE,  // param points to T_ECHO_VAD_INFO
    ECHO_EVENT_HOWLING_DETECTION,   // param points to T_ECHO_HOWLING_INFO
    ECHO_EVENT_AIDENOISE_NNE_TASK,  // param points to T_FILTER_NNE_TASK_INFO
    ECHO_EVENT_MAX
} T_ECHO_EVENT;

typedef struct _ECHO_SOUND_ACTIVE
{
    T_U32 vadType;              // same as echo_param_vad.vadType
    T_S32 vadThresh;            // when vadType has a probe flag, this is a calibrated value.
    // in other cases, it is the same as echo_param_vad.vadTresh
    T_S32 vadLevel;             // same as echo_param_vad.vadLevel
    T_U32 trigger;              // trigger of status change, 1: voice is active, 0: voice is not active
    T_AUDIO_FILTER_TS ts;       // timestamp of the event

    T_pVOID userParam;          // user specified it in echo_param_vad
} T_ECHO_VAD_INFO;

typedef struct _ECHO_HOWLING_INFO
{
    T_U32 howlingFreq;          // howling frequency
    T_U8  trigger;              // trigger of howling event, 1: howling freq arises, 0: howling freq falls
    T_U8  nHowlingFreq;         // number of howling frequencies still active
    T_AUDIO_FILTER_TS ts;       // timestamp of the event

    T_U32 flags;                // b[0]: set 1 to disable internal global volume control strategy. default: 1
} T_ECHO_HOWLING_INFO;

// Echo uses this NOTIFY CB function to send messages to user, in the form of event and its companion param.
// event: see T_ECHO_EVENT
// param: points to an event-specific structure, type of which is mentioned in the comments of T_ECHO_EVENT.
//        this structure is internal buffer. user must not use it outside of the callback.
// cb_notify_param: user specified it in T_ECHO_IN_INFO when calling _SD_Echo_Open.
// pathId: user assigned it in T_ECHO_IN_INFO when calling _SD_Echo_Open.
typedef T_VOID(*ECHO_CALLBACK_FUN_NOTIFY)(T_ECHO_EVENT event, T_pVOID param, T_HANDLE cb_notify_param, T_U8 pathId);

typedef enum _ECHO_DUMP_ITEM_ID
{
    ECHO_ITEM_PCM               = 0x01000000,             // pcm's meta is of type (T_ECHO_PCM_META*)

    // near path
    ECHO_ITEM_ADC_STREAM        = ECHO_ITEM_PCM | 0x010,  // input to Echo's near path
    ECHO_ITEM_DAC_LOOPBACK      = ECHO_ITEM_PCM | 0x011,  // 2nd input to Echo's near path
    ECHO_ITEM_NEAR_DENC         = ECHO_ITEM_PCM | 0x015,  // after D-ENC
    ECHO_ITEM_NEAR_EQ           = ECHO_ITEM_PCM | 0x020,  // after near_EQ
    ECHO_ITEM_SYNCED_PAIR       = ECHO_ITEM_PCM | 0x030,  // synchronized inputs to AEC, stereo format. L: synced_far, R: synced_near
    ECHO_ITEM_AEC               = ECHO_ITEM_PCM | 0x040,  // after AEC
    ECHO_ITEM_NEAR_NR           = ECHO_ITEM_PCM | 0x050,  // after near_NR
    ECHO_ITEM_CORE_RESULT       = ECHO_ITEM_PCM | 0x060,  // after AGC
    ECHO_ITEM_NEAR_AIDEN        = ECHO_ITEM_PCM | 0x063,  // after near_AIDenoise
    ECHO_ITEM_NEAR_HS           = ECHO_ITEM_PCM | 0x065,  // after near_HowlingSuppress
    ECHO_ITEM_NEAR_VOLUME       = ECHO_ITEM_PCM | 0x070,  // after near_Volume
    ECHO_ITEM_RESULT_STREAM     = ECHO_ITEM_PCM | 0x080,  // output of Echo's near path

    // far path
    ECHO_ITEM_FAR_STREAM        = ECHO_ITEM_PCM | 0x110,  // input to Echo's far path
    ECHO_ITEM_FAR_EQ            = ECHO_ITEM_PCM | 0x120,  // after far_EQ
    ECHO_ITEM_FAR_NR            = ECHO_ITEM_PCM | 0x130,  // after far_NR
    ECHO_ITEM_FAR_HS            = ECHO_ITEM_PCM | 0x140,  // after far_HowlingSuppress
    ECHO_ITEM_FAR_VOLUME        = ECHO_ITEM_PCM | 0x150,  // after far_Volume
    ECHO_ITEM_DAC_STREAM        = ECHO_ITEM_PCM | 0x160,  // output of Echo's far path

    ECHO_ITEM_DEBUG_VARIABLE    = 0x02000000,             // variable's meta is of type T_CHR[], which is the variable's name string
} T_ECHO_DUMP_ITEM_ID;
typedef struct _ECHO_PCM_META
{
    T_S8  nChannels;
    T_S8  bitsPerSample;
    T_S8  byte_align;       // # bytes in a sampling cycle
    T_S32 sampleRate;
    T_AUDIO_FILTER_TS ts;   // timestamp of the first sample
} T_ECHO_PCM_META;

// Echo uses this DUMP CB function to send dumped data to user.
// item_id: see T_ECHO_DUMP_ITEM_ID
// size: # bytes of data
// data: buffer containing the dumped item
// meta: points to an item-specific structure, type of which is mentioned in the comments of T_ECHO_DUMP_ITEM_ID.
//       data and meta are all internal buffers. user must not modify it or use it outside of the callback.
// cb_dump_param: user specified it in T_ECHO_IN_INFO when calling _SD_Echo_Open.
// pathId: user assigned it in T_ECHO_IN_INFO when calling _SD_Echo_Open.
typedef T_VOID(*ECHO_CALLBACK_FUN_DUMP)(T_ECHO_DUMP_ITEM_ID item_id, T_S32 size, T_pCVOID data, T_pCVOID meta, T_HANDLE cb_dump_param, T_U8 pathId);

typedef struct
{
    // compatible with T_AUDIO_CB_FUNS
    MEDIALIB_CALLBACK_FUN_MALLOC                Malloc;
    MEDIALIB_CALLBACK_FUN_FREE                  Free;
    MEDIALIB_CALLBACK_FUN_PRINTF                Printf;
    MEDIALIB_CALLBACK_FUN_FLUSH_DCACHE_RANGE    flushDCache;

    ECHO_CALLBACK_FUN_NOTIFY                    notify;
    ECHO_CALLBACK_FUN_DUMP                      dump;
} T_ECHO_CB_FUNS;

struct echo_param_aec
{
    T_U32   aecEna;             // enable AEC
    T_U16   tail;               // tail size in samples
    T_S16   farDigiGain;        // digtal gain for dac_loopback, Q10, 0: use default. use the form of AK16Q10(x.xx) to set it.
    T_S16   nearDigiGain;       // digtal gain for adc_stream, Q10, 0: use default. use the form of AK16Q10(x.xx) to set it.
    //  it will apply to adc_stream even if aecEna is set to 0.
    T_S32   farLimit;           // max amplitude of dac_loopback, Q15, 0: use default. use the form of AK32Q15(x.xx) or SD_dBFS_TO_LEVEL(x.xx) to set it.
    //  note: it applies only to dac_loopback, not dac_stream. so it can not change far path volume.
    //  it should be identical with the param "vol_param.limit" set by _SD_Echo_SetFarVolumeParam.
    T_S32   farBreakdownThresh; // when dac_loopback's amplitude is bigger than this threshold, speaker or mic will breakdown, so the aec is doomed.
    //  Echo lib will make its effort to keep echo suppressed during these periods by deploying half-duplex method.
    //  Q15, 0: no breakdown. use the form of AK32Q15(x.xx) or SD_dBFS_TO_LEVEL(x.xx) to set it.
};

struct echo_param_nr
{
    T_U32   nrEna;            // enable noise reduction
    T_S16   noiseSuppressDb;  // attenuation of noise in dB (negative number), 0: use default
};

struct echo_param_vad
{
    T_U32   vadType;          // bit[3:0]: position to apply vad.
    //   1: after nr
    //   2: after agc
    //   3: on adc_stream
    //   others: not implemented
    // bit[7:4]: algorithm
    //   0: sound level detection
    //   1: baby screaming detection
    //   2: baby screaming detection (AI)
    // bit[11:8]: id
    // bit[15]: probe flag.
    //   Echo lib will notify an ECHO_EVENT_VOICE_ACTIVE_PROBE event, in which the output vadThresh just
    //     matches the value for the current signal to be detected as active in the designated vadLevel.
    T_S32   vadThresh;        // threshold of signal amplitude. use the form of AK32Q15(x.xx) or SD_dBFS_TO_LEVEL(x.xx) to set it
    T_S32   vadLevel;         // sensitivity level. [0, 4], smaller is more sensitive.
    T_pVOID userParam;        // user defined param, T_ECHO_VAD_INFO will send it back to user

    T_S8   *modelName;        // AI 检测模型文件名. 0: use default, for algorithm 2, see comments on sd_param_AICryDetect
};

// helper to compose vadType
#define SD_VAD_TYPE(pos, algorithm, id, probe_flag)     ((pos) | ((algorithm)<<4) | ((id)<<8) | ((probe_flag)?(1<<15):0))

struct echo_param_agc
{
    T_U32   agcEna;           // enable AGC
    T_U32   agcLevel;         // agc's target level, 0: use default. use the form of AK32Q15(x.xx) or SD_dBFS_TO_LEVEL(x.xx) to set it.
    T_U16   maxGain;          // agc's max_gain, Q0
    T_U16   minGain;          // agc's min_gain. use the form of AK16Q10(x.xx) to set it.
    T_S16   nearSensitivity;  // sensitivity of near-end speech [1:100], bigger is more sensitive. 0: use default
};

struct echo_param_volctrl
{
    T_S32 limit;   // max amplitude of samples, Q15, 0: use default. use the form of AK32Q15(x.xx) or SD_dBFS_TO_LEVEL(x.xx) to set it.
    T_S32 vol_dB;  // volume in dB, Q10, use the form of AK32Q10(x.xx) to set it.
    // -40 ~ 20 dB is recommended. less than -90dB actually mutes the signal.
};

struct echo_param_howlingSuppress
{
    T_U32   hsEna;            // enable hs
    T_U32   hsAlgorithm;      // 0 or 1: using howling suppression, 2: using howling suppression2, others: not defined

    T_S16   hsDurationMs;     // howling suppressing duration in ms, 0: use default
    T_S16   paprThr;          // 0: use default
    T_S16   pnprThr;          // 0: use default
    T_S16   phprThr;          // 0: use default
    T_S16   ptprThr;          // 0: use default
};

typedef enum
{
    ECHO_FLAG_HAVE_AEC       = (1 << 0),
    ECHO_FLAG_HAVE_NR        = (1 << 1),
    ECHO_FLAG_HAVE_EQ        = (1 << 2),
    ECHO_FLAG_HAVE_HS        = (1 << 3),
    ECHO_FLAG_HAVE_VOLUME    = (1 << 4),
    ECHO_FLAG_HAVE_SYNC      = (1 << 5),
    ECHO_FLAG_HAVE_MICARRAY  = (1 << 6),

    ECHO_FLAG_AI             = (0 << 15),
    ECHO_FLAG_AO             = (1 << 15),
} T_ECHO_MODE_FLAG;

#define ECHO_MODE_AI_NORMAL             /*0x0037*/(ECHO_FLAG_AI | ECHO_FLAG_HAVE_AEC | ECHO_FLAG_HAVE_NR | ECHO_FLAG_HAVE_EQ | ECHO_FLAG_HAVE_VOLUME | ECHO_FLAG_HAVE_SYNC)
#define ECHO_MODE_AI_MIC_ARRAY_PROCESS  /*0x0077*/(ECHO_FLAG_AI | ECHO_FLAG_HAVE_AEC | ECHO_FLAG_HAVE_NR | ECHO_FLAG_HAVE_EQ | ECHO_FLAG_HAVE_VOLUME | ECHO_FLAG_HAVE_SYNC | ECHO_FLAG_HAVE_MICARRAY)
#define ECHO_MODE_AI_GET_AEC_FRAME      /*0x0020*/(ECHO_FLAG_AI | ECHO_FLAG_HAVE_SYNC)
#define ECHO_MODE_AO_NORMAL             /*0x801e*/(ECHO_FLAG_AO | ECHO_FLAG_HAVE_NR | ECHO_FLAG_HAVE_EQ | ECHO_FLAG_HAVE_VOLUME | ECHO_FLAG_HAVE_HS)

typedef struct
{
    T_pCSTR          strVersion; // set to AUDIO_FILTER_VERSION_STRING
    T_ECHO_CB_FUNS   cb_fun;
    T_HANDLE         cb_notify_param;    // user defined param, cb_fun.notify will send it back to user
    T_HANDLE         cb_dump_param;      // user defined param, cb_fun.dump will send it back to user

    T_U8             m_pathId;           // user-assigned id for this path, cb_fun will send it back to user
    T_U16            m_mode;             // mode to select path topology. bit-or of T_ECHO_MODE_FLAG. user can use ECHO_MODE_XXX preset.
    T_U32            m_SampleRate;       // sample rate, nominal
    T_U16            m_Channels;         // channel number. NOTE: if HAVE_MICARRAY flag is set, m_Channels shall be 1
    T_U16            m_BitsPerSample;    // bits per sample
    T_U8             reserved;

    T_U32            m_hwSampleRate;     // actual sample rate of hardware, set different than m_SampleRate to enable sample rate compensation
    T_U32            m_jitterBufLen;     // size of jitter buf in bytes

    // params for near path only
    T_U16            m_syncedInputs;     // adc_stream & dac_loopback are (0)not synchronized or (1)guaranteed synced
    T_U16            m_syncBufLen;       // size of sync buf in bytes for input adc_stream & dac_loopback, set when m_syncedInputs is 0
    T_S32            m_syncCompensate;   // # samples to compensate for dac_loopback leading adc_stream
} T_ECHO_IN_INFO;


#define SD_ZONE_ID_ECHO_FAR_PATH     (1<<18) // 调试 Echo 的 far_path
#define SD_ZONE_ID_ECHO_NEAR_PATH    (1<<19) // 调试 Echo 的 near_path

/**
 * @brief    设置 Echo 库的 debug zones，选择打印信息的类型
 * @param    [in] bit-or of SD_ZONE_ID_XXX.
 *                Default is SD_DEFAULT_DEBUG_ZONES
 * @return   T_VOID
 */
T_VOID _SD_Echo_SetDebugZones(T_U32 debugZones);

#define SD_ECHO_DEFAULT_DEBUG_ZONES  (SD_DEFAULT_DEBUG_ZONES | SD_ZONE_ID_PARAM)

// return a path handle hEcho, or AK_NULL if open failed.
T_VOID *_SD_Echo_Open(T_ECHO_IN_INFO *echo_input);

// return AK_TRUE or AK_FALSE
T_S32 _SD_Echo_Close(T_VOID *hEcho);

// Tell the lib these two paths have a causal relationship according to echo generation.
// called whenever these two handles are changed
T_VOID _SD_Echo_Pair_Paths(T_VOID *hNearPath, T_VOID *hFarPath);

// 重置通道的所有状态，清除各子模块中的数据
// return AK_TRUE or AK_FALSE
T_S32 _SD_Echo_Reset(T_VOID *hEcho);

// 进入 flush 状态，path 内部的所有数据将通过后续 _SD_Echo_GetXxx 调用输出. 在所有数据输出之后，flush 状态自动解除，可以 fill 新数据。
// hUser: user-specified handle. reserved for future use.
//        user should set it to an non-zero value.
// return AK_TRUE or AK_FALSE
T_S32 _SD_Echo_Flush(T_VOID *hEcho, T_HANDLE hUser);

/**
 * @brief   向far通道填充数据
 * @param   [in] pbuf: 待拷贝的数据缓冲区
 * @param   [in] len: 待拷贝的数据长度(byte)
 * @param   [in] ts: time stamp (us) of the **first** sample in pbuf
 * @param   [in] atomic:
 *               1: 必须一次全部拷贝，如空间不足则不拷贝，返回0
 *               0: 如空间不足，则拷贝部分数据，返回实际拷贝的长度
 * @retval  实际填充的长度(byte)
 *          If pbuf is AK_NULL, return the maximun data size you can fill, regardless of len and atomic value.
 */
T_S32 _SD_Echo_FillFarStream(T_VOID *hEcho, T_U8 *pbuf, T_S32 len, T_AUDIO_FILTER_TS ts, T_S32 atomic);
/**
 * @brief   从far通道读取数据
 * @param   [out] pbuf: 待写入的数据缓冲区
 * @param   [in ] len: 待写入的数据长度(byte)
 * @param   [out] ts: time stamp (us) of the **first** sample in pbuf
 * @param   [in ] atomic:
 *               1: 必须一次读取长度为len的数据，如数据量不足则不读取，返回0
 *               0: 如数据量小于len，则读出所有现有的数据，返回实际读取的长度
 * @retval  实际读取的长度(byte)
 *          If pbuf is AK_NULL, return the maximun data size you can get, regardless of len and atomic value.
 */
T_S32 _SD_Echo_GetDacStream(T_VOID *hEcho, T_U8 *pbuf, T_S32 len, T_AUDIO_FILTER_TS *ts, T_S32 atomic);

/**
 * @brief   向near通道填充 adc_stream 数据
 * @param   [in] pbuf: 待拷贝的数据缓冲区
 * @param   [in] len: 待拷贝的数据长度(byte)
 * @param   [in] ts: time stamp (us) of the **last** sample in pbuf
 * @param   [in] atomic:
 *               1: 必须一次全部拷贝，如空间不足则不拷贝，返回0
 *               0: 如空间不足，则拷贝部分数据，返回实际拷贝的长度
 * @retval  实际填充的长度(byte, >=0)，或错误码 T_SD_ERROR_CODE (<0)
 *          If pbuf is AK_NULL, return the maximun data size you can fill, regardless of len and atomic value.
 */
T_S32 _SD_Echo_FillAdcStream(T_VOID *hEcho, T_U8 *pbuf, T_S32 len, T_AUDIO_FILTER_TS ts, T_S32 atomic);
/**
 * @brief   向near通道填充 dac_loopback 数据
 * @param   [in] pbuf: 待拷贝的数据缓冲区
 * @param   [in] len: 待拷贝的数据长度(byte)
 * @param   [in] ts: time stamp (us) of the **last** sample in pbuf
 * @param   [in] atomic:
 *               1: 必须一次全部拷贝，如空间不足则不拷贝，返回0
 *               0: 如空间不足，则拷贝部分数据，返回实际拷贝的长度
 * @retval  实际填充的长度(byte, >=0)，或错误码 T_SD_ERROR_CODE (<0)
 *          If pbuf is AK_NULL, return the maximun data size you can fill, regardless of len and atomic value.
 */
T_S32 _SD_Echo_FillDacLoopback(T_VOID *hEcho, T_U8 *pbuf, T_S32 len, T_AUDIO_FILTER_TS ts, T_S32 atomic);
/**
 * @brief   从near通道读取result数据
 * @param   [out] pbuf: 待写入的数据缓冲区
 * @param   [in ] len: 待写入的数据长度(byte)
 * @param   [out] ts: time stamp (us) of the **first** sample in pbuf
 * @param   [in] atomic:
 *               1: 必须一次读取长度为len的数据，如数据量不足则不读取，返回0
 *               0: 如数据量小于len，则读出所有现有的数据，返回实际读取的长度
 * @retval  实际读取的长度(byte, >=0)，或错误码 T_SD_ERROR_CODE (<0)
 *          If pbuf is AK_NULL, return the maximun data size you can get, regardless of len and atomic value.
 */
T_S32 _SD_Echo_GetResult(T_VOID *hEcho, T_U8 *pbuf, T_S32 len, T_AUDIO_FILTER_TS *ts, T_S32 atomic);


/**
 * @brief   获取子模块的参数数据
 * @param   [out] xxx_param: 子模块的参数结构体
 * @retval  AK_TRUE:  读取成功
 *          AK_FALSE: 读取失败，或模块没有加载
 */
T_S32 _SD_Echo_GetAecParam(T_VOID *hEcho, struct echo_param_aec *aec_params);
T_S32 _SD_Echo_GetNrParam(T_VOID *hEcho, struct echo_param_nr *nr_param);
T_S32 _SD_Echo_GetRnnoiseParam(T_VOID *hEcho, struct sd_param_rnnoise *rnnoise_param);
T_S32 _SD_Echo_GetAIDenoiseParam(T_VOID *hEcho, struct sd_param_AIDenoise *aidenoise_param);
T_S32 _SD_Echo_GetVadParam(T_VOID *hEcho, struct echo_param_vad *vad_param);
T_S32 _SD_Echo_GetAgcParam(T_VOID *hEcho, struct echo_param_agc *agc_param);
T_S32 _SD_Echo_GetNearEqParam(T_VOID *hEcho, struct sd_param_eq *eq_param);
T_S32 _SD_Echo_GetNearVolumeParam(T_VOID *hEcho, struct echo_param_volctrl *vol_param);
T_S32 _SD_Echo_GetNearHowlingSuppressParam(T_VOID *hEcho, struct echo_param_howlingSuppress *hs_param);
T_S32 _SD_Echo_GetNearJitterBufParam(T_VOID *hEcho, struct sd_param_pcmbuf *pcmbuf_param);
T_S32 _SD_Echo_GetDencParam(T_VOID *hEcho, struct sd_param_denc *denc_param);

T_S32 _SD_Echo_GetFarEqParam(T_VOID *hEcho, struct sd_param_eq *eq_param);
T_S32 _SD_Echo_GetFarNrParam(T_VOID *hEcho, struct echo_param_nr *nr_param);
T_S32 _SD_Echo_GetFarHowlingSuppressParam(T_VOID *hEcho, struct echo_param_howlingSuppress *hs_param);
T_S32 _SD_Echo_GetFarVolumeParam(T_VOID *hEcho, struct echo_param_volctrl *vol_param);
T_S32 _SD_Echo_GetFarJitterBufParam(T_VOID *hEcho, struct sd_param_pcmbuf *pcmbuf_param);

/**
 * @brief   加载并配置子模块的参数数据，或卸载该子模块
 * @param   [in] load_module: 加载(1) / 卸载(0) 该子模块
 * @param   [in] xxx_param: 子模块的参数结构体
 * @retval  AK_TRUE:  命令成功
 *          AK_FALSE: 命令失败
 */
T_S32 _SD_Echo_SetAecParam(T_VOID *hEcho, T_S32 load_module, struct echo_param_aec *aec_params);
T_S32 _SD_Echo_SetNrParam(T_VOID *hEcho, T_S32 load_module, struct echo_param_nr *nr_param);
T_S32 _SD_Echo_SetRnnoiseParam(T_VOID *hEcho, T_S32 load_module, struct sd_param_rnnoise *rnnoise_param);
T_S32 _SD_Echo_SetAIDenoiseParam(T_VOID *hEcho, T_S32 load_module, struct sd_param_AIDenoise *aidenoise_param);
T_S32 _SD_Echo_SetVadParam(T_VOID *hEcho, T_S32 load_module, struct echo_param_vad *vad_param);
T_S32 _SD_Echo_SetAgcParam(T_VOID *hEcho, T_S32 load_module, struct echo_param_agc *agc_param);
T_S32 _SD_Echo_SetNearEqParam(T_VOID *hEcho, T_S32 load_module, struct sd_param_eq *eq_param);
T_S32 _SD_Echo_SetNearVolumeParam(T_VOID *hEcho, T_S32 load_module, struct echo_param_volctrl *vol_param);
T_S32 _SD_Echo_SetNearHowlingSuppressParam(T_VOID *hEcho, T_S32 load_module, struct echo_param_howlingSuppress *hs_param);
T_S32 _SD_Echo_SetNearJitterBufParam(T_VOID *hEcho, T_S32 load_module, struct sd_param_pcmbuf *pcmbuf_param);
T_S32 _SD_Echo_SetDencParam(T_VOID *hEcho, T_S32 load_module, struct sd_param_denc *denc_param);

T_S32 _SD_Echo_SetFarEqParam(T_VOID *hEcho, T_S32 load_module, struct sd_param_eq *eq_param);
T_S32 _SD_Echo_SetFarNrParam(T_VOID *hEcho, T_S32 load_module, struct echo_param_nr *nr_param);
T_S32 _SD_Echo_SetFarHowlingSuppressParam(T_VOID *hEcho, T_S32 load_module, struct echo_param_howlingSuppress *hs_param);
T_S32 _SD_Echo_SetFarVolumeParam(T_VOID *hEcho, T_S32 load_module, struct echo_param_volctrl *vol_param);
T_S32 _SD_Echo_SetFarJitterBufParam(T_VOID *hEcho, T_S32 load_module, struct sd_param_pcmbuf *pcmbuf_param);


#include "sdParamFactory.h"

/**
 * @brief   配置 path 的参数
 * @retval  AK_TRUE:  命令成功
 *          AK_FALSE: 命令失败
 */
T_S32 _SD_Echo_SetFarPathParam(T_VOID *hEcho, T_pSD_PARAM_FACTORY param_factory);
T_S32 _SD_Echo_SetNearPathParam(T_VOID *hEcho, T_pSD_PARAM_FACTORY param_factory);

/**
 * @brief   获取 path 的参数字符串
 * @retval  a cmd line string consisting of all params in the path
 *          user shall use it immediately
 */
char *_SD_Echo_GetFarPathParamString(T_VOID *hEcho);
char *_SD_Echo_GetNearPathParamString(T_VOID *hEcho);

void _SD_Echo_PrintFarPathParamHelp(void);
void _SD_Echo_PrintNearPathParamHelp(void);

#ifdef __cplusplus
}
#endif

#endif
/* end of sdfilter.h */
/*@}*/
