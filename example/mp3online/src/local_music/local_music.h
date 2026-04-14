
#ifndef __LOCAL_MUSIC_H__
#define __LOCAL_MUSIC_H__

#include <stdint.h>
#include "audio_server.h"
#include "audio_mp3ctrl.h"

typedef enum
{
    CMD_MP3_PALY = 0,  /* mp3 play */
    CMD_MP3_STOP,      /* mp3 stop */
    CMD_MP3_PAUSE,     /* mp3 pause */
    CMD_MP3_RESUME,    /* mp3 resume */
    CMD_MP3_MAX
} CMD_MP3_E;

typedef struct
{
    uint8_t cmd;  /* see enum CMD_MP3_E. */
    mp3_ioctl_cmd_param_t param;  /* mp3_ioctl_cmd_param_t */
    uint32_t loop;  /*loop times. 0 : play one time. 1 ~ n : play 2 ~ n+1 times. */
} mp3_ctrl_info_t;

void play_buff(const char *buff, int len);
void play_pause(void);
void play_resume(void);
void play_stop(void);
uint32_t play_get_total_seconds(void);
void play_ringbuff(int file_len);

#endif
