/**
  ******************************************************************************
  * @file   audio_bt_voice.c
  * @author Sifli software development team
  * @brief SIFLI bt audio path handler.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2020 - 2021,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifdef SOC_BF0_HCPU

#include "bts2_app_inc.h"
#include "audioproc.h"
#include "ipc/ringbuffer.h"
#include "bf0_mbox_common.h"
#include "ipc/dataqueue.h"
#include "drivers/audio.h"
#include "audio_msbc_plc.h"
#include "av_sbc_api.h"
#include "ipc/ringbuffer.h"
#include "bf0_mbox_common.h"

#include "audio_server.h"
#include "audio_cvsd.h"
#include "audio_filter.h"

#define DBG_TAG           "audio"
#define DBG_LVL           AUDIO_DBG_LVL
#include "log.h"

#ifndef SF32LB58X_3SCO
    #define BT_SCO_MAX_NUM    1
    #define BT_SCO_TX_HAS_HEADER 0
    #define SOFT_CVSD_ENCODE  0
#else
    #define BT_SCO_MAX_NUM    3
    #define BT_SCO_TX_HAS_HEADER 1
    #define SOFT_CVSD_ENCODE  1
#endif

#define AUDIO_FMT_PCM   0
#define AUDIO_FMT_MSBC  1

#define SYS_HL_BT_AUDIO_QUEUE    (6)

static ipc_queue_handle_t sys_hl_bt_audio_queue = IPC_QUEUE_INVALID_HANDLE;


#define HCPU_LCPU_SHARE_MEM_BASE_ADDR  LCPU_AUDIO_MEM_START_ADDR
//#define AUDIO_RINGBUFFER_LEN       ((LCPU_HCPU_AUDIO_MEM_SIZE - 0x50 - sizeof(struct hci_sync_con_cmp_evt))/8*4)  //cache data and synchionization
//#define  SCO_PARA_ADDR_BASE    (HCPU_LCPU_SHARE_MEM_BASE_ADDR + LCPU_HCPU_AUDIO_MEM_SIZE - sizeof(struct hci_sync_con_cmp_evt))


static struct rt_ringbuffer *pt_bt2speaker_rbf;
static struct rt_ringbuffer *pt_mic2bt_rbf;
static uint8_t *p_u8_tx_rbf_pool;
static uint8_t *p_u8_rx_rbf_pool;
static uint8_t *p_uplink_pool;
static struct rt_ringbuffer uplink_ring;

#define LCPU2HCPU_DATA_HEADER           4
#define AUDIO_BT_VOICE_BUFFER_LEN       248
#define AUDIO_BT_VOICE_PCM_IN_LEN       120
#define AUDIO_BT_VOICE_MSBC_IN_LEN      60
#define CVSD_MODE                       2
#define TRANS_MODE                      3
#define AUDIO_MSBC_BUFFER_LEN           240
#define AUDIO_BT_UPLINK_BUFFER_SIZE     (AUDIO_BT_VOICE_MSBC_IN_LEN * 48)

struct hci_sync_con_cmp_evt
{
    ///Status for command reception
    uint8_t status;
    ///Connection handle
    uint16_t    conhdl;
    ///BD address
    uint8_t  addr[6];;
    ///Link type
    uint8_t lk_type;
    ///Transmit interval
    uint8_t tx_int;
    ///Retransmission window
    uint8_t ret_win;
    ///rx packet length
    uint16_t rx_pkt_len;
    ///tx packet length
    uint16_t tx_pkt_len;
    ///Air mode
    uint8_t air_mode;

};
/*audio parameter,from LCPU temporary, general from HFP*/
static volatile struct hci_sync_con_cmp_evt *p_sco_para;

uint8_t *g_bt_rx_fifo;
uint8_t *g_bt_tx_fifo;



typedef struct audio_msbc_tag
{
    volatile uint8_t      send_enable;
    volatile uint8_t      state;
    uint8_t      sn_cnt;
    uint8_t      trans_flag;
    //struct PLC_State  *msbc_plc_state;
    LowcFE_c          *pcm_plc;
    uint32_t     total_packet;
    uint32_t     error_packet;
    uint32_t     decode_err;
    uint32_t     old_error_cnt;
} audio_msbc_t;

audio_msbc_t g_audio_msbc_env =
{
    .state  = 0,
    .sn_cnt = 0
};

#if SOFT_CVSD_ENCODE
typedef struct audio_cvsd_tag
{
    cvsd_t cvsd_e;//encode
    cvsd_t cvsd_d;//decode
    uint8_t *bit_buf;
    int16_t *inp_buf;
    int16_t *out_buf;
    int16_t *interpolate_buf;
    int16_t *decimate_buf;
    int16_t *inp_buf_shift;
    int16_t *out_buf_shift;
    int buf_size_FIR_assumpt;
    int out_len_interpolate;
    int out_len_interp_FIR_assumpt;
} audio_cvsd_t;

audio_cvsd_t g_audio_cvsd_env =
{
    .bit_buf  = NULL,
    .inp_buf  = NULL,
    .out_buf  = NULL,
    .interpolate_buf  = NULL,
    .decimate_buf  = NULL,
    .inp_buf_shift = NULL,
    .out_buf_shift = NULL,
};
#endif

/*input buffer data struct, not only data but also status*/
typedef struct lc_sco_data_tag
{
    /// length of the data
    uint8_t  length;
    uint8_t  packet_status;
    /// reserved for feature use
    uint16_t rsvd1;
    /// data pointer
    uint8_t  data[120];
} audio_sco_data_t;

const U8 indices0[] = { 0xad, 0x0, 0x0, 0xc5, 0x0, 0x0, 0x0, 0x0, 0x77, 0x6d,
                        0xb6, 0xdd, 0xdb, 0x6d, 0xb7, 0x76, 0xdb, 0x6d, 0xdd, 0xb6, 0xdb, 0x77, 0x6d,
                        0xb6, 0xdd, 0xdb, 0x6d, 0xb7, 0x76, 0xdb, 0x6d, 0xdd, 0xb6, 0xdb, 0x77, 0x6d,
                        0xb6, 0xdd, 0xdb, 0x6d, 0xb7, 0x76, 0xdb, 0x6d, 0xdd, 0xb6, 0xdb, 0x77, 0x6d,
                        0xb6, 0xdd, 0xdb, 0x6d, 0xb7, 0x76, 0xdb, 0x6c
                      };


static const uint8_t msbc_sn[4] = {0x08, 0x38, 0xC8, 0xF8};
static uint8_t *g_msbc_fifo;

void _hcpu_2_lcpu_ipc_audio_notify();

__WEAK void bt_rx_event_to_audio_server()
{


}

static inline uint8_t sco_interval_support()
{
    if ((p_sco_para->tx_int == 12) || (p_sco_para->tx_int == 6))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

static inline void bt_voice_notify_trans()
{
    audio_msbc_t *p_msbc_env = &g_audio_msbc_env;

    if (1 == p_msbc_env->trans_flag)
    {
#ifdef RT_USING_BT
        bt_notify_t args;
        args.event = BT_EVENT_TRANS_AUDIO_IND;
        args.args = RT_NULL;
        rt_bt_event_notify(&args);
#endif
        p_msbc_env->trans_flag = 2;
        LOG_I("URC trans audio");
    }
}

void bt_voice_trans_disable()
{
    audio_msbc_t *p_msbc_env = &g_audio_msbc_env;

    p_msbc_env->trans_flag = 0xFF;
}

static inline uint8_t input_size()
{
#if !SOFT_CVSD_ENCODE
    if (p_sco_para->air_mode == CVSD_MODE) // CVSD 8K
    {
        if ((p_sco_para->rx_pkt_len == 60) || (!sco_interval_support()))
        {
            return AUDIO_BT_VOICE_PCM_IN_LEN + LCPU2HCPU_DATA_HEADER;
        }
        else
        {
            return AUDIO_BT_VOICE_PCM_IN_LEN / 2 + LCPU2HCPU_DATA_HEADER;
        }
    }
    else
#endif
    {
        if ((p_sco_para->rx_pkt_len == AUDIO_BT_VOICE_MSBC_IN_LEN) || (!sco_interval_support())) //mSBC 7.5ms
        {
            return AUDIO_BT_VOICE_MSBC_IN_LEN + LCPU2HCPU_DATA_HEADER;
        }
        else
        {
            return (AUDIO_BT_VOICE_MSBC_IN_LEN / 2 + LCPU2HCPU_DATA_HEADER);
        }
    }
}


static inline uint8_t input_fmt()
{
    if (p_sco_para->air_mode == CVSD_MODE) // CVSD 8K
    {
        return AUDIO_FMT_PCM;
    }
    return AUDIO_FMT_MSBC;
}

static inline uint16_t input_samplerate()
{
    /* maybe called by HFP, parameter from HFP */
    if (p_sco_para->air_mode == CVSD_MODE) // CVSD 8K
    {
        return 8000;
    }
    return 16000;
}
void bt_voice_uplink_process(uint8_t *fifo, uint16_t size)
{
    if (g_audio_msbc_env.state == 0)
    {
        return;
    }

    if (rt_ringbuffer_space_len(&uplink_ring) < size)
    {
        LOG_I("msbc uplink full");
    }
    else
    {
        rt_ringbuffer_put(&uplink_ring, fifo, size);
    }
    if (g_audio_msbc_env.send_enable == 0 && rt_ringbuffer_data_len(&uplink_ring) >= rt_ringbuffer_get_size(&uplink_ring) / 2)
    {
        g_audio_msbc_env.send_enable = 1;
        LOG_I("uplink send enable");
    }
}

#if SOFT_CVSD_ENCODE
#define BT_CVSD_FRAME_LEN  60
static void *calloc_buffer(void *src, int size, int n)
{
    if ((src = calloc(n, sizeof(size))) == NULL)
    {
        rt_kprintf("Error in memory allocation!\n");
        //exit(1);
    }
    return src;
}
void bt_cvsd_init(void)
{
    int pow_M_L_factor = 3;

    g_audio_cvsd_env.buf_size_FIR_assumpt = BT_CVSD_FRAME_LEN + FIR_FILTER_LENGTH;

    g_audio_cvsd_env.out_len_interpolate = BT_CVSD_FRAME_LEN << pow_M_L_factor;
    g_audio_cvsd_env.out_len_interp_FIR_assumpt = g_audio_cvsd_env.out_len_interpolate + FIR_FILTER_LENGTH;

    g_audio_cvsd_env.bit_buf = (uint8_t *)calloc_buffer(g_audio_cvsd_env.bit_buf, sizeof(uint8_t), BT_CVSD_FRAME_LEN);

    g_audio_cvsd_env.inp_buf = (int16_t *)calloc_buffer(g_audio_cvsd_env.inp_buf, sizeof(int16_t), g_audio_cvsd_env.buf_size_FIR_assumpt);
    g_audio_cvsd_env.out_buf = (int16_t *)calloc_buffer(g_audio_cvsd_env.out_buf, sizeof(int16_t), g_audio_cvsd_env.out_len_interp_FIR_assumpt);
    g_audio_cvsd_env.interpolate_buf = (int16_t *)calloc_buffer(g_audio_cvsd_env.interpolate_buf, sizeof(int16_t), g_audio_cvsd_env.out_len_interpolate);
    g_audio_cvsd_env.decimate_buf = (int16_t *)calloc_buffer(g_audio_cvsd_env.decimate_buf, sizeof(int16_t), BT_CVSD_FRAME_LEN);
    g_audio_cvsd_env.inp_buf_shift = (int16_t *)(g_audio_cvsd_env.inp_buf + FIR_FILTER_LENGTH);
    g_audio_cvsd_env.out_buf_shift = (int16_t *)(g_audio_cvsd_env.out_buf + FIR_FILTER_LENGTH);

    if (cvsdInit(&g_audio_cvsd_env.cvsd_e))
    {
        rt_kprintf("incorrect initialization of CVSD!\n");
        //exit(1);
    }

    if (cvsdInit(&g_audio_cvsd_env.cvsd_d))
    {
        rt_kprintf("incorrect initialization of CVSD!\n");
        //exit(1);
    }
}
void bt_cvsd_deinit(void)
{
    free(g_audio_cvsd_env.bit_buf);
    free(g_audio_cvsd_env.inp_buf);
    free(g_audio_cvsd_env.out_buf);
    free(g_audio_cvsd_env.interpolate_buf);
    free(g_audio_cvsd_env.decimate_buf);

    g_audio_cvsd_env.bit_buf = NULL;
    g_audio_cvsd_env.inp_buf = NULL;
    g_audio_cvsd_env.out_buf = NULL;
    g_audio_cvsd_env.interpolate_buf = NULL;
    g_audio_cvsd_env.decimate_buf = NULL;
}
static const unsigned char table[256] =
{
    0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
    0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
    0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
    0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
    0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
    0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
    0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
    0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
    0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
    0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
    0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
    0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
    0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
    0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
    0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
    0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
    0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
    0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
    0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
    0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
    0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
    0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
    0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
    0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
    0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
    0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
    0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
    0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
    0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
    0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
    0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
    0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff,
};
unsigned char Reverse_byte(unsigned char c)
{
    return table[c];
}
#endif
#if !BT_SCO_TX_HAS_HEADER
void bt_voice_uplink_send()
{
    rt_size_t num;
    uint8_t fifo[120];
    if (g_audio_msbc_env.send_enable == 0 || g_audio_msbc_env.state == 0)
    {
        //LOG_I("uplink wait buffer half");
        return;
    }
    uint16_t size = input_size() - LCPU2HCPU_DATA_HEADER;


    if (rt_ringbuffer_data_len(&uplink_ring) < size)
    {
        LOG_I("uplink msbc buf empty");
        return;
    }

#if 0
    if (rt_ringbuffer_data_len(pt_mic2bt_rbf) == 0)
    {
        LOG_W("3a_w bt_voice_uplink is empty");

        for (int i = 0; i < 4; i++)
        {
            num = rt_ringbuffer_get(&uplink_ring, fifo, size);
            if (num == size)
            {
                num = rt_ringbuffer_put(pt_mic2bt_rbf, fifo, size);
                //LOG_W("3a_w uplink preput %d", num);
            }
            else
            {
                //LOG_W("3a_w uplink buffer read error1?");
                break;
            }
        }
    }
    else if (rt_ringbuffer_space_len(pt_mic2bt_rbf) < size)
    {
        //LOG_W("3a_w bt_voice_uplink is full!");
    }
    else
    {
        num = rt_ringbuffer_get(&uplink_ring, fifo, size);
        if (num == size)
        {
            num = rt_ringbuffer_put(pt_mic2bt_rbf, fifo, size);
            if (num != size)
            {
                //LOG_W("3a_w uplink buf fill error1=%d", num);
            }
        }
        else
        {
            //LOG_W("3a_w uplink buf empty error2=%d", num);
        }
    }
#else
    while (rt_ringbuffer_space_len(pt_mic2bt_rbf) > size)
    {
        if (rt_ringbuffer_data_len(&uplink_ring) > size)
        {
            num = rt_ringbuffer_get(&uplink_ring, fifo, size);
            if (num == size)
            {
                num = rt_ringbuffer_put(pt_mic2bt_rbf, fifo, size);
            }
            else
            {
                LOG_W("3a_w uplink buffer readerr");
                break;
            }
        }
        else
        {
            break;
        }
    }
#endif
}

static uint8_t d_drop_cnt = 0;
static uint8_t s_packet_cnt = 0;
static uint8_t size_decode = 0;
extern uint8_t audio_3a_dnlink_buf_is_full(uint8_t size);
void bt_voice_downlink_process(uint8_t is_ready)
{
    uint8_t need_algo;
    rt_uint32_t getnum;
    uint8_t size = input_size();
    uint8_t decode_len;
    if (rt_ringbuffer_data_len(pt_bt2speaker_rbf) < size)
    {
        LOG_I("3a_w pt_bt2speaker_rbf empty");
        return;
    }

    while (rt_ringbuffer_data_len(pt_bt2speaker_rbf) > size)
    {
        if (audio_3a_dnlink_buf_is_full(size_decode))
        {
            return;
        }

        getnum = rt_ringbuffer_get(pt_bt2speaker_rbf, &g_bt_rx_fifo[s_packet_cnt * size], size);
        //RT_ASSERT(getnum == size);
        if (getnum != size)
        {
            LOG_I("3a_w pt_bt2speaker_rbf get %d %d", getnum, size);
        }

        if (sco_interval_support())
        {
            if ((g_bt_rx_fifo[s_packet_cnt * size] != (size - LCPU2HCPU_DATA_HEADER)) ||
                    (g_bt_rx_fifo[s_packet_cnt * size + 1] > 3))
            {
                LOG_I("3a_w pt_bt2speaker_rbf flush,%d, 0x%x, %d, %d, %d, %d", rt_ringbuffer_data_len(pt_bt2speaker_rbf),
                      *((uint32_t *)g_bt_rx_fifo), size, input_fmt(), s_packet_cnt, p_sco_para->rx_pkt_len);
                s_packet_cnt = 0;
                while (rt_ringbuffer_data_len(pt_bt2speaker_rbf) > 0)
                {
                    rt_ringbuffer_get(pt_bt2speaker_rbf, &g_bt_rx_fifo[0], size);
                }
                return;
            }
        }
        else
        {
            g_bt_rx_fifo[1] = 2;
        }

        if (is_ready == 0)
        {
            d_drop_cnt++;
            if (d_drop_cnt == 1)
                LOG_W("3a_w downlink drop data");

        }
#if !SOFT_CVSD_ENCODE
        if (((input_fmt() == AUDIO_FMT_PCM) && (size == (AUDIO_BT_VOICE_PCM_IN_LEN / 2 + LCPU2HCPU_DATA_HEADER))) ||
                ((input_fmt() == AUDIO_FMT_MSBC) && (size == (AUDIO_BT_VOICE_MSBC_IN_LEN / 2 + LCPU2HCPU_DATA_HEADER))))
#else
        if (size == (AUDIO_BT_VOICE_MSBC_IN_LEN / 2 + LCPU2HCPU_DATA_HEADER))
#endif
        {
            if (s_packet_cnt == 0)
            {
                s_packet_cnt++;
                return;
            }
            else
            {
                s_packet_cnt = 0;
            }
        }
        audio_tick_in(AUDIO_MSBC_DECODE_TIME);
        decode_len = msbc_decode_process(g_bt_rx_fifo, g_bt_tx_fifo, size);
        audio_tick_out(AUDIO_MSBC_DECODE_TIME);

        need_algo = audio_server_bt_voice_ind(g_bt_tx_fifo, decode_len);

        if (decode_len > 0 && is_ready && need_algo)
        {
            bt_voice_notify_trans();
            audio_tick_in(AUDIO_DNAGC_TIME);
            audio_3a_downlink(g_bt_tx_fifo, decode_len);
            audio_tick_out(AUDIO_DNAGC_TIME);
            size_decode = decode_len;
        }
    }
}
void bt_voice_open(uint32_t samplerate)
{
    msbc_open(samplerate);
}
#else
typedef struct bt_sco_data_hdr_tag
{
    /// length of the data
    uint8_t  length;
    uint8_t  packet_status;
    /// reserved for feature use
    uint16_t scohdl;
} bt_sco_data_hdr;
uint8_t g_sco_path_sel = 1;
static uint8_t s_packet_cnt = 0;
static uint8_t size_decode = 0;
extern uint8_t audio_3a_dnlink_buf_is_full(uint8_t size);
void bt_voice_uplink_send()
{
    //move to bt_voice_downlink_process
}
uint8_t bt_voice_chk_datahdr(uint32_t hdr)
{
    uint8_t hdr_ok = 0;
    bt_sco_data_hdr *sco_hdr = (bt_sco_data_hdr *)&hdr;
    uint8_t scohdl_l = sco_hdr->scohdl & 0xF;
    uint8_t scohdl_m = (sco_hdr->scohdl >> 4) & 0xF;
    uint8_t scohdl_h = (sco_hdr->scohdl >> 8) & 0xFF;

    //only support 60byte data length
    if ((sco_hdr->length == 0x3c) && (scohdl_l <= 4) && (scohdl_m == 8) && (scohdl_h <= 4))
    {
        hdr_ok = 1;
    }

    return hdr_ok;
}
static uint8_t sco_path_sel;
void bt_voice_downlink_process(uint8_t is_ready)
{
    uint8_t need_algo;
    rt_uint32_t getnum, putnum;
    uint8_t size = 64;
    uint8_t decode_len;
    int8_t find_hdr = 0;
    uint32_t datahdr;
    static uint8_t other_sco_cnt = 0;


    while (rt_ringbuffer_data_len(pt_bt2speaker_rbf) > size)
    {
        if (audio_3a_dnlink_buf_is_full(size_decode))
        {
            return;
        }

        find_hdr = 0;
        while (find_hdr == 0)
        {
            getnum = rt_ringbuffer_get(pt_bt2speaker_rbf, (uint8_t *)&datahdr, 4);
            if (getnum == 4)
            {
                find_hdr = bt_voice_chk_datahdr(datahdr);
            }
            else
            {
                find_hdr = -1;
            }
        }

        if (find_hdr == 1)
        {
            uint8_t sco_path = datahdr >> 24;

            *(uint32_t *)g_bt_rx_fifo = datahdr;
            getnum = rt_ringbuffer_get(pt_bt2speaker_rbf, &g_bt_rx_fifo[4], 0x3c);//only support 60byte data length
            if (sco_path == sco_path_sel)
            {
                //slect one sco path, decode date and send to speaker
                //if decode other sco path, need multiple decoding environment(for cvsd, need multiple  g_audio_cvsd_env)
                //the PLC environment also need multiple for other sco path(p_msbc_env->pcm_plc)
                //the function of msbc_decode_process includes decoding and PLC, support msbc and cvsd
                decode_len = msbc_decode_process(g_bt_rx_fifo, g_bt_tx_fifo, 64);

                audio_3a_downlink(g_bt_tx_fifo, decode_len);//downlink webrtc algorithom
                size_decode = decode_len;

                //wait uplink webrtc algorithom processing data
                if (g_audio_msbc_env.send_enable == 1 && g_audio_msbc_env.state == 1)
                {
                    if ((rt_ringbuffer_data_len(&uplink_ring) > 60) && (rt_ringbuffer_space_len(pt_mic2bt_rbf) > 64))
                    {
                        getnum = rt_ringbuffer_get(&uplink_ring, &g_bt_tx_fifo[4], 60);
                        if (getnum == 60)
                        {
                            *(uint32_t *)g_bt_tx_fifo = datahdr;
                            putnum = rt_ringbuffer_put(pt_mic2bt_rbf, &g_bt_tx_fifo[0], 64);
                        }
                        else
                        {
                            LOG_W("3a_w uplink buffer readerr");
                        }
                    }
                }
                other_sco_cnt = 0;
            }
            else
            {
                //other sco path just loopback
                if (rt_ringbuffer_space_len(pt_mic2bt_rbf) > 64)
                {
                    putnum = rt_ringbuffer_put(pt_mic2bt_rbf, &g_bt_rx_fifo[0], 64);
                }

                if (sco_path_sel != 0)
                {
                    other_sco_cnt++;
                    if (other_sco_cnt == 20)
                    {
                        sco_path_sel = 0;// the slect sco is disconnect
                    }
                }

                if (sco_path_sel == 0)
                {
                    //just dicard mic data, send 0 to speaker
                    if (rt_ringbuffer_data_len(&uplink_ring) > 60)
                    {
                        rt_ringbuffer_get(&uplink_ring, &g_bt_tx_fifo[4], 60);
                        memset(g_bt_tx_fifo, 0, size_decode);
                        audio_3a_downlink(g_bt_tx_fifo, size_decode);
                    }
                }
            }
        }

    }
}
static void btsco_sel(uint8_t argc, char **argv)
{
    if (argc > 1)
    {
        uint8_t sel = strtol(argv[1], NULL, 0);
        if (sel <= 3)
        {
            g_sco_path_sel = sel;
        }
        else
        {
            rt_kprintf("input para err should less than 4\n");
        }
        rt_kprintf("sco sel:%d\n", g_sco_path_sel);
    }
}
#ifdef RT_USING_FINSH
    MSH_CMD_EXPORT(btsco_sel, slect which sco to speaker);
#endif // RT_USING_FINSH
void bt_voice_open(uint32_t samplerate)
{
    sco_path_sel = g_sco_path_sel;
    msbc_open(samplerate);
    bt_voice_trans_disable();
}

#endif


void bt_voice_close()
{
    _hcpu_2_lcpu_ipc_audio_notify();
    msbc_close();
}


int bt_audiopath_init(void)
{
    int ret;
    uint16_t ringbuffer_len;
    uint8_t sco_num = BT_SCO_MAX_NUM;
    ringbuffer_len = ((LCPU_HCPU_AUDIO_MEM_SIZE - 0x50 - sco_num * sizeof(struct hci_sync_con_cmp_evt)) / 8 * 4);

    pt_bt2speaker_rbf = (struct rt_ringbuffer *)HCPU_LCPU_SHARE_MEM_BASE_ADDR;
    pt_mic2bt_rbf = (struct rt_ringbuffer *)(HCPU_LCPU_SHARE_MEM_BASE_ADDR + 0x20);
    p_u8_tx_rbf_pool = (uint8_t *)(HCPU_LCPU_SHARE_MEM_BASE_ADDR + 0x50);
    p_u8_rx_rbf_pool = (uint8_t *)(HCPU_LCPU_SHARE_MEM_BASE_ADDR + 0x50 + ringbuffer_len);

    p_sco_para = (struct hci_sync_con_cmp_evt *)(HCPU_LCPU_SHARE_MEM_BASE_ADDR + LCPU_HCPU_AUDIO_MEM_SIZE - sizeof(struct hci_sync_con_cmp_evt) * sco_num);

    //input buffer
    rt_ringbuffer_init(pt_mic2bt_rbf, p_u8_rx_rbf_pool, ringbuffer_len);
    rt_ringbuffer_init(pt_bt2speaker_rbf, p_u8_tx_rbf_pool, ringbuffer_len);

    //output buffer
    g_bt_rx_fifo = (uint8_t *)audio_mem_malloc(AUDIO_BT_VOICE_BUFFER_LEN);
    RT_ASSERT(g_bt_rx_fifo);
    g_bt_tx_fifo = (uint8_t *)audio_mem_malloc(AUDIO_BT_VOICE_BUFFER_LEN);
    RT_ASSERT(g_bt_tx_fifo);
#if SOFT_CVSD_ENCODE
    {
        uint32_t is_soft_cvsd = 0x5A5AA5A5;
        HAL_LCPU_CONFIG_set(HAL_LCPU_CONFIG_SOFT_CVSD, &is_soft_cvsd, 4);
    }
#endif

    return 0;
}


void _hcpu_2_lcpu_ipc_audio_notify()
{
    LOG_I("_hcpu_2_lcpu_ipc_audio_notify");
    ipc_queue_write(sys_hl_bt_audio_queue, NULL, 0, 0);//stop notify
}




/*every bt packet trigger*/
int32_t _hl_bt_audio_queue_rx_ind(ipc_queue_handle_t handle, size_t size)
{
    LOG_D("_hl_bt_audio_queue_rx_ind");

    bt_rx_event_to_audio_server();

    return 0;
}

int bt_voice_init(void)
{
    ipc_queue_cfg_t q_cfg;

    q_cfg.qid = SYS_HL_BT_AUDIO_QUEUE;
    q_cfg.tx_buf_size = 0;
    q_cfg.tx_buf_addr = NULL;
    q_cfg.tx_buf_addr_alias = NULL;
    q_cfg.rx_buf_addr = NULL;
    q_cfg.rx_ind = _hl_bt_audio_queue_rx_ind;
    q_cfg.user_data = 0;
    g_msbc_fifo = (uint8_t *)audio_mem_malloc(AUDIO_MSBC_BUFFER_LEN);
    RT_ASSERT(g_msbc_fifo);
    sys_hl_bt_audio_queue = ipc_queue_init(&q_cfg);

    RT_ASSERT(IPC_QUEUE_INVALID_HANDLE != sys_hl_bt_audio_queue);

    ipc_queue_open(sys_hl_bt_audio_queue);

    bt_audiopath_init();
    return 0;
}


INIT_COMPONENT_EXPORT(bt_voice_init);


static uint8_t drop_cnt  = 0;
uint8_t msbc_decode_process(uint8_t *fifo, uint8_t *output, uint8_t size)
{
    uint8_t out_size = 120;
    audio_msbc_t *p_msbc_env = &g_audio_msbc_env;
    BTS2S_SBC_STREAM  pbss_t;

    audio_sco_data_t *p_sco_data = (audio_sco_data_t *)fifo;
    uint8_t *p_data = NULL;

    if (p_msbc_env->state == 0)
    {
        drop_cnt++;
        if (drop_cnt == 1)
        {
            LOG_I("msbc closed");
        }

        return 0;
    }

    LOG_D("msbc_proc:length:%d,packet_sta:%d,data:0x%x\n", p_sco_data->length, p_sco_data->packet_status, *(uint32_t *)&p_sco_data->data[0]);
    p_msbc_env->total_packet++;
#ifdef AUDIO_MSBC_STATIC_TIME
    g_msbc_test_cur = audio_get_curr_tick();
#endif
    if (input_fmt() == AUDIO_FMT_PCM)
    {
        uint8_t packet_status = p_sco_data->packet_status;
#if !SOFT_CVSD_ENCODE
        if (input_size() == 64) //3.75ms
        {
            audio_sco_data_t *p_sco_data_2 = (audio_sco_data_t *)(fifo + 64);
            RT_ASSERT((packet_status <= 3) && (p_sco_data_2->packet_status <= 3));
            if (0 != p_sco_data_2->packet_status)
            {
                packet_status = p_sco_data_2->packet_status;
            }
            memcpy(&p_sco_data->data[60], &p_sco_data_2->data[0], 60);
        }
#else
        if (input_size() == 34) //3.75ms
        {
            uint8_t packet_status_2 = *(fifo + 35);//for align, see (audio_sco_data_t *)
            RT_ASSERT((packet_status <= 3) && (packet_status_2 <= 3));
            if (0 != packet_status_2)
            {
                packet_status = packet_status_2;
            }
            memcpy(&p_sco_data->data[30], (fifo + 38), 30);

        }
#endif
        //only PLC process
        if (0 != packet_status)
        {
            g711plc_dofe(p_msbc_env->pcm_plc, (short *)(&p_sco_data->data[0]));
            p_msbc_env->error_packet++;
#ifdef AUDIO_MSBC_STATIC_TIME
            g_delta[6].delt_sum += audio_get_delta_tick_in_10us(g_msbc_test_cur);
            g_delta[6].times++;
#endif
        }
        else
        {
#if SOFT_CVSD_ENCODE
            //audio_dump_data_align_size(ADUMP_DOWNLINK, &p_sco_data->data[0], 60);
            memmove(g_audio_cvsd_env.out_buf, (int16_t *)(g_audio_cvsd_env.out_buf + g_audio_cvsd_env.out_len_interpolate), FIR_FILTER_LENGTH * sizeof(int16_t));
            //memcpy(g_audio_cvsd_env.bit_buf, &p_sco_data->data[0], BT_CVSD_FRAME_LEN);
            for (int i = 0; i < BT_CVSD_FRAME_LEN; i++)
            {
                p_sco_data->data[i] = Reverse_byte(p_sco_data->data[i]);
            }
            cvsdDecode(&(g_audio_cvsd_env.cvsd_d), (const uint8_t *)(&p_sco_data->data[0]), BT_CVSD_FRAME_LEN, (int16_t *)(g_audio_cvsd_env.out_buf_shift));
            //decimation_x8(g_audio_cvsd_env.out_buf, g_audio_cvsd_env.out_len_interp_FIR_assumpt, g_audio_cvsd_env.decimate_buf, BT_CVSD_FRAME_LEN);
            decimation_x8(g_audio_cvsd_env.out_buf, g_audio_cvsd_env.out_len_interp_FIR_assumpt, (int16_t *)&p_sco_data->data[0], BT_CVSD_FRAME_LEN);
            //audio_dump_data_align_size(ADUMP_DOWNLINK_AGC, &p_sco_data->data[0], 120);
#endif
            g711plc_addtohistory(p_msbc_env->pcm_plc, (short *)(&p_sco_data->data[0]));
#ifdef AUDIO_MSBC_STATIC_TIME
            g_delta[5].delt_sum += audio_get_delta_tick_in_10us(g_msbc_test_cur);
            g_delta[5].times++;
#endif
        }
        memcpy(output, &p_sco_data->data[0], 120);
        out_size = 120;
        LOG_D("msbc_proc:packet_sta:%d\n", p_sco_data->packet_status);
        if ((p_msbc_env->total_packet & 0xFF) == 0)
        {
            LOG_W("3a_w cvsd total packet:%d, rx err:%d\n", p_msbc_env->total_packet, p_msbc_env->error_packet);

            uint32_t *p_sco_sta = (uint32_t *)(HCPU_LCPU_SHARE_MEM_BASE_ADDR + 0x40);
            LOG_W("3a_w cvsd lcpu buf err:0x%x, buf full_empty:0x%x, rx cnt:0x%x\n", *p_sco_sta, *(p_sco_sta + 1), *(p_sco_sta + 2));

        }
    }
    else //msbc
    {
        //msbc_plc
#ifdef AUDIO_MEM_ALLOC
        U8 *msbc_dest_data_plc_in = audio_mem_calloc(1, 240);
        U8 *msbc_dest_data_in = audio_mem_calloc(1, 240);

#else
        U8 msbc_dest_data_plc_in[240];
        U8 msbc_dest_data_in[240];
        memset(msbc_dest_data_plc_in, 0x00, sizeof(msbc_dest_data_plc_in));
        memset(msbc_dest_data_in, 0x00, sizeof(msbc_dest_data_in));
#endif
        uint8_t packet_status = p_sco_data->packet_status;

        pbss_t.psrc = &p_sco_data->data[0];
        pbss_t.src_len = 60;
        pbss_t.dst_len = 240;
        pbss_t.pdst = msbc_dest_data_plc_in;

        if (input_size() == 34) //3.75ms
        {
            uint8_t packet_status_2 = *(fifo + 35);//for align, see (audio_sco_data_t *)
            RT_ASSERT((packet_status <= 3) && (packet_status_2 <= 3));
            if (0 != packet_status_2)
            {
                packet_status = packet_status_2;
            }
            memcpy(&p_sco_data->data[30], (fifo + 38), 30);

        }
        if (0 != packet_status)//error packet
        {
            RT_ASSERT(packet_status <= 3);
            p_msbc_env->error_packet++;
#if 0
            pbss_t.src_len = 57;
            pbss_t.psrc = (U8 *)&indices0[0];
            bts2_msbc_decode(&pbss_t);
            LOG_W("3a_w msbc decode error src_len_use=%d,dst_len_use=%d\n", pbss_t.src_len_used, pbss_t.dst_len_used);
            if (0 == pbss_t.dst_len_used)
            {
                pbss_t.src_len = 57;
                pbss_t.psrc = (U8 *)&indices0[0];
                bts2_msbc_decode(&pbss_t);
            }
#endif
#ifdef AUDIO_MSBC_STATIC_TIME
            g_delta[4].delt_sum += audio_get_delta_tick_in_10us(g_msbc_test_cur);
            g_delta[4].times++;
            g_msbc_test_cur = audio_get_curr_tick();
#endif
            //PLC_bad_frame(p_msbc_env->msbc_plc_state, (short *)msbc_dest_data_plc_in, (short *)output);
            g711plc_dofe(p_msbc_env->pcm_plc, (short *)msbc_dest_data_plc_in);
            pbss_t.dst_len_used = 240;


#ifdef AUDIO_MSBC_STATIC_TIME
            g_delta[6].delt_sum += audio_get_delta_tick_in_10us(g_msbc_test_cur);
            g_delta[6].times++;
#endif

        }
        else
        {
            bts2_msbc_decode(&pbss_t);
            if (240 != pbss_t.dst_len_used)
            {
                //LOG_I("msbc_decode err:src_len_use=%d,dst_len_use=%d\n", pbss_t.src_len_used, pbss_t.dst_len_used);
                g711plc_dofe(p_msbc_env->pcm_plc, (short *)msbc_dest_data_plc_in);
                pbss_t.dst_len_used = 240;
                p_msbc_env->decode_err++;
            }
            else
            {
#ifdef AUDIO_MSBC_STATIC_TIME
                g_delta[4].delt_sum += audio_get_delta_tick_in_10us(g_msbc_test_cur);
                g_delta[4].times++;
                g_msbc_test_cur = audio_get_curr_tick();
#endif
                //PLC_good_frame(p_msbc_env->msbc_plc_state, (short *)msbc_dest_data_plc_in, (short *)output);
                g711plc_addtohistory(p_msbc_env->pcm_plc, (short *)msbc_dest_data_plc_in);
#ifdef AUDIO_MSBC_STATIC_TIME
                g_delta[5].delt_sum += audio_get_delta_tick_in_10us(g_msbc_test_cur);
                g_delta[5].times++;
#endif
            }
        }
        memcpy(output, msbc_dest_data_plc_in, 240);
        if ((p_msbc_env->total_packet & 0xFF) == 0)
        {
            LOG_W("3a_w msbc total packet:%d, rx err:%d, decode err:%d\n", p_msbc_env->total_packet, p_msbc_env->error_packet, p_msbc_env->decode_err);

            uint32_t *p_sco_sta = (uint32_t *)(HCPU_LCPU_SHARE_MEM_BASE_ADDR + 0x40);
            LOG_W("3a_w msbc lcpu buf err:0x%x, buf full_empty:0x%x, rx cnt:0x%x\n", *p_sco_sta, *(p_sco_sta + 1), *(p_sco_sta + 2));

        }
        //LOG_D("msbc_proc:packet_sta:%d,src_len_use=%d,dst_len_use=%d\n", packet_status, pbss_t.src_len_used, pbss_t.dst_len_used);

        if (pbss_t.dst_len_used == 240)
        {
            out_size = 240;
        }
        else
        {
            LOG_I("msbc_decode error\n");
            out_size = 0;
        }
#ifdef AUDIO_MEM_ALLOC
        audio_mem_free(msbc_dest_data_plc_in);
        audio_mem_free(msbc_dest_data_in);
#endif
    }

    if ((p_msbc_env->total_packet & 0xFF) == 0)
    {
        if ((p_msbc_env->trans_flag == 0) && (p_msbc_env->error_packet - p_msbc_env->old_error_cnt) > 250)
        {
            p_msbc_env->trans_flag = 1;
        }
        p_msbc_env->old_error_cnt = p_msbc_env->error_packet;
    }

    return out_size;
}


void msbc_encode_process(uint8_t *fifo, uint16_t fifo_size)
{
    audio_msbc_t *p_msbc_env = &g_audio_msbc_env;
    BTS2S_SBC_STREAM  pbss_t;
    uint8_t *p_data = NULL;
    if (p_msbc_env->state == 0)
    {
        //LOG_I("msbc closed");
        return;
    }
#ifdef AUDIO_MSBC_STATIC_TIME
    g_msbc_test_cur = audio_get_curr_tick();
#endif
    if (240 == fifo_size) //msbc,  16000
    {
        //msbc_encode();
        p_data = g_msbc_fifo;
        memset(p_data, 0, 60);
        *p_data++ = 0x1;
        *p_data++ = msbc_sn[p_msbc_env->sn_cnt++];
        if (p_msbc_env->sn_cnt == 4)
        {
            p_msbc_env->sn_cnt = 0;
        }
        pbss_t.psrc = fifo;
        pbss_t.src_len = 240;
        pbss_t.dst_len = 57;
        pbss_t.pdst = p_data;
        bts2_msbc_encode(&pbss_t);
        if ((240 != pbss_t.src_len_used) || (pbss_t.dst_len_used != 57))
        {
            LOG_W("3a_w msbc encode src_len_use=%d,dst_len_use=%d\n", pbss_t.src_len_used, pbss_t.dst_len_used);
        }
        bt_voice_uplink_process(g_msbc_fifo, 60);
    }
    else//cvsd
    {
        //no process
#if !SOFT_CVSD_ENCODE
        bt_voice_uplink_process(fifo, 120);
#else
        //audio_dump_data_align_size(ADUMP_DOWNLINK, fifo, 120);

        memmove(g_audio_cvsd_env.inp_buf, (int16_t *)(g_audio_cvsd_env.inp_buf + BT_CVSD_FRAME_LEN), FIR_FILTER_LENGTH * sizeof(int16_t));
        memcpy(g_audio_cvsd_env.inp_buf_shift, fifo, 120);

        interpolation_x8(g_audio_cvsd_env.inp_buf, g_audio_cvsd_env.buf_size_FIR_assumpt, g_audio_cvsd_env.interpolate_buf, g_audio_cvsd_env.out_len_interpolate);
        //cvsdEncode(&cvsd_e, (const int16_t *)interpolate_buf, out_len_interpolate, (uint32_t *)bit_buf);
        cvsdEncode(&(g_audio_cvsd_env.cvsd_e), (const int16_t *)g_audio_cvsd_env.interpolate_buf, g_audio_cvsd_env.out_len_interpolate, (uint32_t *)fifo);

        for (int i = 0; i < BT_CVSD_FRAME_LEN; i++)
        {
            fifo[i] = Reverse_byte(fifo[i]);
        }

        //audio_dump_data_align_size(ADUMP_DOWNLINK_AGC, fifo, 60);

        bt_voice_uplink_process(fifo, 60);
#endif
    }
}
void msbc_open(uint32_t samplerate)
{
    audio_msbc_t *p_msbc_env = &g_audio_msbc_env;
    if (0 == p_msbc_env->state)
    {
        uint8_t defresize, enfresize;
        defresize = bts2_msbc_decode_cfg();
        enfresize = bts2_msbc_encode_cfg();
#if SOFT_CVSD_ENCODE
        bt_cvsd_init();
        LOG_I("BT soft cvsd enable\n");
#endif
        //p_msbc_env->msbc_plc_state = audio_mem_malloc(sizeof(struct PLC_State));
        p_uplink_pool  = audio_mem_calloc(1, AUDIO_BT_UPLINK_BUFFER_SIZE);
        RT_ASSERT(p_uplink_pool);
        rt_ringbuffer_init(&uplink_ring, p_uplink_pool, AUDIO_BT_UPLINK_BUFFER_SIZE);
        p_msbc_env->pcm_plc = audio_mem_malloc(sizeof(LowcFE_c));
        RT_ASSERT(p_msbc_env->pcm_plc);
        if (samplerate == 8000)
        {
            cvsd_g711plc_construct(p_msbc_env->pcm_plc);
        }
        else
        {
            msbc_g711plc_construct(p_msbc_env->pcm_plc);
        }

        LOG_I("msbc_open defresize=%d,enfresize=%d, rate=%d\n", defresize, enfresize, samplerate);
        p_msbc_env->state = 1;
        p_msbc_env->error_packet = 0;
        p_msbc_env->total_packet = 0;
        p_msbc_env->send_enable = 0;
        p_msbc_env->sn_cnt = 0;
        p_msbc_env->decode_err = 0;
        p_msbc_env->old_error_cnt = 0;
        p_msbc_env->trans_flag = 0;
        s_packet_cnt = 0;
    }
    LOG_I("msbc_open mode=%d,tx=%d, rx=%d\n", p_sco_para->air_mode, p_sco_para->tx_pkt_len, p_sco_para->rx_pkt_len);
}

void msbc_close()
{
    audio_msbc_t *p_msbc_env = &g_audio_msbc_env;
    if (p_msbc_env->state != 0)
    {
        p_msbc_env->state = 0;
        //audio_mem_free(p_msbc_env->msbc_plc_state);
        audio_mem_free(p_msbc_env->pcm_plc);
#if SOFT_CVSD_ENCODE
        bt_cvsd_deinit();
#endif
        bts2_msbc_encode_completed();
        bts2_msbc_decode_completed();
        audio_mem_free(p_uplink_pool);
        p_msbc_env->send_enable = 0;
        p_msbc_env->sn_cnt = 0;
        LOG_I("msbc packet: total %d rx error %d, decode error %d", p_msbc_env->total_packet, p_msbc_env->error_packet, p_msbc_env->decode_err);
    }
    LOG_I("msbc_close");
}

#endif
