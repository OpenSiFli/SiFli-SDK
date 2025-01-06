#include <rtthread.h>
#ifdef SOC_BF0_HCPU
#include <string.h>
#include <stdlib.h>
#include "audioproc.h"
#include "ipc/ringbuffer.h"
#include "bf0_mbox_common.h"
#include "ipc/dataqueue.h"
#include "drivers/audio.h"
#include "dfs_file.h"
#include "dfs_posix.h"

#include "audio_mem.h"
#define DBG_TAG           "audio_3a"
#define DBG_LVL           AUDIO_DBG_LVL //LOG_LVL_WARNING
#include "log.h"
#include "audio_server.h"

#define CODEC_DATA_UNIT_LEN        (320)  //should same as audio_server.c
#define MIC_DELAY_REF_16K          378
#define MIC_DELAY_REF_8K           384

__WEAK void audio_tick_in(uint8_t type)
{
}
__WEAK void audio_tick_out(uint8_t type)
{
}
__WEAK void audio_time_print(void)
{
}
__WEAK void audio_uplink_time_print(void)
{
}
__WEAK void audio_dnlink_time_print(void)
{
}

void audio_3a_set_bypass(uint8_t is_bypass, uint8_t mic, uint8_t down)
{
}

void audio_3a_open(uint32_t samplerate)
{
}

void audio_3a_close()
{
}

void audio_3a_far_put(uint8_t *fifo, uint16_t fifo_size)
{
}

uint8_t audio_3a_dnlink_buf_is_full(uint8_t size)
{
    return 0;
}

void audio_3a_downlink(uint8_t *fifo, uint8_t size)
{
}

void audio_3a_uplink(uint8_t *fifo, uint16_t fifo_size, uint8_t is_mute, uint8_t is_pdm)
{
}

void audio_command_process(uint8_t *cmd_1)
{
}

#endif
