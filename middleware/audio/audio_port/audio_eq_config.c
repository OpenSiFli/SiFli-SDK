#include <rtthread.h>

#ifdef SOC_BF0_HCPU
#include <string.h>
#include <stdlib.h>
#include <rtdevice.h>
#include "audioproc.h"
#include "audio_server.h"
#include "audio_mem.h"

#if PKG_USING_SOUNDPLUS
    #define BT_VOICE_EQ_ON_OFF      0  //hfp eq:  0 off, 1 on
#else
    #define BT_VOICE_EQ_ON_OFF      1  //hfp eq:  0 off, 1 on
#endif

#define BT_MUSIC_EQ_ON_OFF      1  //a2dp sink eq:  0 off, 1 on
#define OTHER_MUSIC_EQ_ON_OFF   1  //local music eq:  0 off, 1 on

uint8_t get_eq_config(audio_type_t type)
{
    if (type == AUDIO_TYPE_BT_VOICE)
    {
        return BT_VOICE_EQ_ON_OFF;
    }
    if (type == AUDIO_TYPE_BT_MUSIC)
    {
        return BT_MUSIC_EQ_ON_OFF;
    }
    return OTHER_MUSIC_EQ_ON_OFF;
}
#endif
