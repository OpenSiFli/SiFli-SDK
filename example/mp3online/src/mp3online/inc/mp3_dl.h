#ifndef __MP3_DL_H__
#define __MP3_DL_H__

typedef enum
{
    MP3_DL_STATE_IDLE,
    MP3_DL_STATE_INIT,
    MP3_DL_STATE_DLING,
} mp3_dl_state_t;
extern mp3_dl_state_t g_mp3_dl_state;



#endif