#ifndef AUDIOPROC_H
#define AUDIOPROC_H

#include "ipc/ringbuffer.h"

#define AUDIO_DBG_LVL           LOG_LVL_INFO


#define USING_2RX_WAKEUP        1 //wakeup aduio server when adc and pdm data coming all
#define DEBUG_FRAME_SYNC        0
typedef struct
{
    rt_tick_t dump_start;
    int       fd;
    uint8_t   dump_enable;
    uint8_t   dump_end;
    uint32_t  max_frame;
    uint32_t  cur_frame;
} audio_dump_ctrl_t;

typedef enum
{
    ADUMP_DOWNLINK = 0,
    ADUMP_DOWNLINK_AGC,
    ADUMP_AUDPRC,
    ADUMP_DC_OUT,
    ADUMP_RAMP_IN_OUT,
    ADUMP_AECM_INPUT1,
    ADUMP_AECM_INPUT2,
    ADUMP_AECM_OUT,
    ADUMP_ANS_OUT,
    ADUMP_AGC_OUT,
    ADUMP_RAMP_OUT_OUT,
    ADUMP_PDM_RX,
    ADUMP_NUM,
} audio_dump_type_t;
void audio_dump_start();
void audio_dump_stop();
void audio_dump_enable(uint8_t type);
void audio_dump_clear();
void audio_dump_data(audio_dump_type_t type, uint8_t *fifo, uint32_t size);

#ifdef BT_FINSH
    void msbc_open(uint32_t samplerate);
    void msbc_close(void);
    void msbc_encode_process(uint8_t *fifo, uint16_t fifo_size);
    uint8_t msbc_decode_process(uint8_t *fifo, uint8_t *output, uint8_t size);
    void bt_voice_open(uint32_t samplerate);
    void bt_voice_close(void);
    void bt_voice_downlink_process(uint8_t is_ready);
    void bt_voice_uplink_send(void);
#else
    #define msbc_open(samplerate)
    #define msbc_close()
    #define msbc_encode_process(fifo, fifo_size);
    #define msbc_decode_process(fifo,output,size) 0
    #define bt_voice_open(samplerate)
    #define bt_voice_close()
    #define bt_voice_downlink_process(is_ready)
    #define bt_voice_uplink_send()
#endif

void speaker_ring_put(uint8_t *fifo, uint16_t fifo_size);
void audio_3a_downlink(uint8_t *fifo, uint8_t size);
void audio_3a_save_pdm(uint8_t *fifo, uint16_t size);
void audio_3a_uplink2(uint8_t *audprc, uint8_t is_mute);
void audio_3a_uplink(uint8_t *fifo, uint16_t fifo_size, uint8_t is_mute, uint8_t is_pdm);
void audio_3a_open(uint32_t samplerate);
void audio_3a_close();
void audio_3a_far_put(uint8_t *fifo, uint16_t fifo_size);
void audio_3a_put_pdm(uint8_t *fifo, uint16_t fifo_size);
void audio_hardware_pa_init(void);
void audio_hardware_pa_start(uint32_t samplerate, uint32_t reserved);
void audio_hardware_pa_stop(void);
int is_audio_dump_enable();
int is_audio_dump_enable_type(audio_dump_type_t type);
#endif


