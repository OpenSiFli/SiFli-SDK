/*
 * SPDX-FileCopyrightText: 2020-2021 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
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
    #define BT_SCO_TX_HAS_HEADER 0b
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




#define LCPU2HCPU_DATA_HEADER           4
#define AUDIO_BT_VOICE_BUFFER_LEN       248
#define AUDIO_BT_CVSD_PCM_LEN           120
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


typedef struct sco_data_tag
{
    uint8_t in_data[AUDIO_BT_VOICE_BUFFER_LEN];
    uint8_t out_data[AUDIO_BT_VOICE_BUFFER_LEN];
    uint8_t in_pos;
    uint8_t stat;
    uint16_t handle;
    uint8_t is_used;
    uint8_t hav_data;
    uint32_t     lost_cnt;
    uint32_t     total_packet;
    uint32_t     error_packet;
    uint32_t     decode_err;
} sco_data_t;

typedef struct sco_uplink_data_tag
{
    uint8_t tmp_data[AUDIO_BT_VOICE_BUFFER_LEN];
    uint8_t out_data[AUDIO_BT_VOICE_BUFFER_LEN];
    uint8_t hav_data[BT_SCO_MAX_NUM];
} sco_uplink_data_t;

typedef struct sco_ipc_tag
{
    struct hci_sync_con_cmp_evt *p_sco_para;
    struct rt_ringbuffer *pt_bt2speaker_rbf;
    struct rt_ringbuffer *pt_mic2bt_rbf;
    uint8_t *p_u8_tx_rbf_pool;
    uint8_t *p_u8_rx_rbf_pool;
} sco_ipc_t;

typedef struct bt_sco_data_hdr_tag
{
    /// length of the data
    uint8_t  length;
    uint8_t  packet_status;
    /// reserved for feature use
    uint16_t scohdl;
} bt_sco_data_hdr;

/********************************************************
  msbc decode not support multiple instances
  all esco should use same  samplerate
  only cvsd support 2/3 esco
********************************************************/
typedef struct bt_voice_tag
{
#if SOFT_CVSD_ENCODE
    audio_cvsd_t cvsd_env[BT_SCO_MAX_NUM];
#endif
    LowcFE_c     pcm_plc[BT_SCO_MAX_NUM];
    struct hci_sync_con_cmp_evt sco_para[BT_SCO_MAX_NUM];
    uint8_t *p_uplink_pool;
    struct rt_ringbuffer uplink_ring;
    sco_data_t dn_data[BT_SCO_MAX_NUM];
    sco_uplink_data_t up_data;
    bt_sco_data_hdr header[BT_SCO_MAX_NUM];
    sco_ipc_t  *sco_ipc;
    uint8_t sco_num;
    uint8_t air_mod;
    uint8_t state;
    uint8_t send_enable;
    uint8_t sn_cnt;
} bt_voice_t;

sco_ipc_t  g_sco_ipc;
bt_voice_t *pt_bt_voice;

void _hcpu_2_lcpu_ipc_audio_notify();

__WEAK void bt_rx_event_to_audio_server()
{


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
void bt_cvsd_init(uint8_t cnt)
{
    int pow_M_L_factor = 3;
    audio_cvsd_t *pt_cvsd = &(pt_bt_voice->cvsd_env[cnt]);

    pt_cvsd->buf_size_FIR_assumpt = BT_CVSD_FRAME_LEN + FIR_FILTER_LENGTH;

    pt_cvsd->out_len_interpolate = BT_CVSD_FRAME_LEN << pow_M_L_factor;
    pt_cvsd->out_len_interp_FIR_assumpt = pt_cvsd->out_len_interpolate + FIR_FILTER_LENGTH;

    pt_cvsd->bit_buf = (uint8_t *)calloc_buffer(pt_cvsd->bit_buf, sizeof(uint8_t), BT_CVSD_FRAME_LEN);

    pt_cvsd->inp_buf = (int16_t *)calloc_buffer(pt_cvsd->inp_buf, sizeof(int16_t), pt_cvsd->buf_size_FIR_assumpt);
    pt_cvsd->out_buf = (int16_t *)calloc_buffer(pt_cvsd->out_buf, sizeof(int16_t), pt_cvsd->out_len_interp_FIR_assumpt);
    pt_cvsd->interpolate_buf = (int16_t *)calloc_buffer(pt_cvsd->interpolate_buf, sizeof(int16_t), pt_cvsd->out_len_interpolate);
    pt_cvsd->decimate_buf = (int16_t *)calloc_buffer(pt_cvsd->decimate_buf, sizeof(int16_t), BT_CVSD_FRAME_LEN);
    pt_cvsd->inp_buf_shift = (int16_t *)(pt_cvsd->inp_buf + FIR_FILTER_LENGTH);
    pt_cvsd->out_buf_shift = (int16_t *)(pt_cvsd->out_buf + FIR_FILTER_LENGTH);

    if (cvsdInit(&pt_cvsd->cvsd_e))
    {
        rt_kprintf("incorrect initialization of CVSD!\n");
        //exit(1);
    }

    if (cvsdInit(&pt_cvsd->cvsd_d))
    {
        rt_kprintf("incorrect initialization of CVSD!\n");
        //exit(1);
    }
}
void bt_cvsd_deinit(uint8_t cnt)
{
    audio_cvsd_t *pt_cvsd = &(pt_bt_voice->cvsd_env[cnt]);

    free(pt_cvsd->bit_buf);
    free(pt_cvsd->inp_buf);
    free(pt_cvsd->out_buf);
    free(pt_cvsd->interpolate_buf);
    free(pt_cvsd->decimate_buf);

    pt_cvsd->bit_buf = NULL;
    pt_cvsd->inp_buf = NULL;
    pt_cvsd->out_buf = NULL;
    pt_cvsd->interpolate_buf = NULL;
    pt_cvsd->decimate_buf = NULL;
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

uint8_t bt_voice_cvsd_decode(uint8_t sco_idx)
{
    sco_data_t *pt_data = &pt_bt_voice->dn_data[sco_idx];
    audio_cvsd_t *pt_cvsd = &pt_bt_voice->cvsd_env[sco_idx];
    LowcFE_c *pt_plc = &pt_bt_voice->pcm_plc[sco_idx];

    pt_data->total_packet++;
    if (0 != pt_data->stat)
    {
        g711plc_dofe(pt_plc, (short *)(&pt_data->out_data[0]));
        pt_data->error_packet++;
    }
    else
    {
#if SOFT_CVSD_ENCODE
        //audio_dump_data_align_size(ADUMP_DOWNLINK, &p_sco_data->data[0], 60);
        memmove(pt_cvsd->out_buf, (int16_t *)(pt_cvsd->out_buf + pt_cvsd->out_len_interpolate), FIR_FILTER_LENGTH * sizeof(int16_t));
        //memcpy(pt_cvsd->bit_buf, &p_sco_data->data[0], BT_CVSD_FRAME_LEN);
        for (int i = 0; i < BT_CVSD_FRAME_LEN; i++)
        {
            pt_data->in_data[0] = Reverse_byte(pt_data->in_data[0]);
        }
        cvsdDecode(&(pt_cvsd->cvsd_d), (const uint8_t *)(&pt_data->in_data[0]), BT_CVSD_FRAME_LEN, (int16_t *)(pt_cvsd->out_buf_shift));

        decimation_x8(pt_cvsd->out_buf, pt_cvsd->out_len_interp_FIR_assumpt, (int16_t *)&pt_data->out_data[0], BT_CVSD_FRAME_LEN);

#else
        memcpy(&pt_data->out_data[0], (&pt_data->in_data[0]), AUDIO_BT_VOICE_PCM_IN_LEN);
#endif
        g711plc_addtohistory(pt_plc, (short *)(&pt_data->out_data[0]));
    }

    if ((pt_data->total_packet & 0xFF) == 0)
    {
        LOG_W("3a_w cvsd sco:%d, total packet:%d, rx err:%d\n", sco_idx, pt_data->total_packet, pt_data->error_packet);

        uint32_t *p_sco_sta = (uint32_t *)(HCPU_LCPU_SHARE_MEM_BASE_ADDR + 0x40);
        LOG_W("3a_w cvsd lcpu buf err:0x%x, buf full_empty:0x%x, rx cnt:0x%x\n", *p_sco_sta, *(p_sco_sta + 1), *(p_sco_sta + 2));
    }

    return 0;
}

uint8_t bt_voice_msbc_decode(uint8_t sco_idx)
{
    BTS2S_SBC_STREAM  pbss_t;

    sco_data_t *pt_data = &pt_bt_voice->dn_data[sco_idx];
    LowcFE_c *pt_plc = &pt_bt_voice->pcm_plc[sco_idx];

    pt_data->total_packet++;
    if (0 != pt_data->stat)
    {
        g711plc_dofe(pt_plc, (short *)(&pt_data->out_data[0]));
        pt_data->error_packet++;
    }
    else
    {
        pbss_t.psrc = &pt_data->in_data[0];
        pbss_t.src_len = 60;
        pbss_t.dst_len = 240;
        pbss_t.pdst = &pt_data->out_data[0];

        bts2_msbc_decode_ext(&pbss_t, sco_idx);
        if (240 != pbss_t.dst_len_used)
        {
            g711plc_dofe(pt_plc, (short *)(&pt_data->out_data[0]));
            pt_data->decode_err++;
        }
        else
        {
            g711plc_addtohistory(pt_plc, (short *)(&pt_data->out_data[0]));
        }
    }

    if ((pt_data->total_packet & 0xFF) == 0)
    {
        LOG_W("3a_w msbc sco:%d, total packet:%d, rx err:%d, decode err:%d\n", sco_idx, pt_data->total_packet, pt_data->error_packet, pt_data->decode_err);

        uint32_t *p_sco_sta = (uint32_t *)(HCPU_LCPU_SHARE_MEM_BASE_ADDR + 0x40);
        LOG_W("3a_w msbc lcpu buf err:0x%x, buf full_empty:0x%x, rx cnt:0x%x\n", *p_sco_sta, *(p_sco_sta + 1), *(p_sco_sta + 2));
    }

    return 0;
}

void bt_voice_uplink_process(uint8_t *fifo, uint16_t size)
{
    if (pt_bt_voice->state == 0)
    {
        return;
    }

    if (rt_ringbuffer_space_len(&pt_bt_voice->uplink_ring) < size)
    {
        LOG_I("msbc uplink full");
    }
    else
    {
        rt_ringbuffer_put(&pt_bt_voice->uplink_ring, fifo, size);
    }
    if (pt_bt_voice->send_enable == 0 && rt_ringbuffer_data_len(&pt_bt_voice->uplink_ring) >= rt_ringbuffer_get_size(&pt_bt_voice->uplink_ring) / 2)
    {
        pt_bt_voice->send_enable = 1;
        LOG_I("uplink send enable");
    }
}

uint8_t bt_voice_msbc_encode(uint8_t *fifo, uint16_t fifo_size)
{
    uint8_t *p_data = NULL;
    BTS2S_SBC_STREAM  pbss_t;
    uint8_t msbc_sn[4] = {0x08, 0x38, 0xC8, 0xF8};

    RT_ASSERT(pt_bt_voice->air_mod == TRANS_MODE);

    p_data = &(pt_bt_voice->up_data.tmp_data[0]);
    memset(p_data, 0, 60);
    *p_data++ = 0x1;
    *p_data++ = msbc_sn[pt_bt_voice->sn_cnt++];
    if (pt_bt_voice->sn_cnt == 4)
    {
        pt_bt_voice->sn_cnt = 0;
    }
    pbss_t.psrc = fifo;
    pbss_t.src_len = 240;
    pbss_t.dst_len = 57;
    pbss_t.pdst = p_data;
    bts2_msbc_encode_ext(&pbss_t, 0);
    if ((240 != pbss_t.src_len_used) || (pbss_t.dst_len_used != 57))
    {
        LOG_W("3a_w msbc encode err src_len_use=%d,dst_len_use=%d\n", pbss_t.src_len_used, pbss_t.dst_len_used);
    }
    bt_voice_uplink_process(&(pt_bt_voice->up_data.tmp_data[0]), 60);

    return 0;
}

uint8_t bt_voice_cvsd_encode(uint8_t *fifo, uint16_t fifo_size)
{
    audio_cvsd_t *pt_cvsd = &pt_bt_voice->cvsd_env[0];

    RT_ASSERT(pt_bt_voice->air_mod == CVSD_MODE);
    //no process
#if !SOFT_CVSD_ENCODE
    bt_voice_uplink_process(fifo, 120);
#else

    memmove(pt_cvsd->inp_buf, (int16_t *)(pt_cvsd->inp_buf + BT_CVSD_FRAME_LEN), FIR_FILTER_LENGTH * sizeof(int16_t));
    memcpy(pt_cvsd->inp_buf_shift, fifo, 120);

    interpolation_x8(pt_cvsd->inp_buf, pt_cvsd->buf_size_FIR_assumpt, pt_cvsd->interpolate_buf, pt_cvsd->out_len_interpolate);
    cvsdEncode(&(pt_cvsd->cvsd_e), (const int16_t *)pt_cvsd->interpolate_buf, pt_cvsd->out_len_interpolate, (uint32_t *)fifo);

    for (int i = 0; i < BT_CVSD_FRAME_LEN; i++)
    {
        fifo[i] = Reverse_byte(fifo[i]);
    }

    bt_voice_uplink_process(fifo, 60);
#endif
    return 0;
}

void msbc_encode_process(uint8_t *fifo, uint16_t fifo_size)
{
    if (pt_bt_voice->state == 0)
    {
        //LOG_I("msbc closed");
        return;
    }

    if (240 == fifo_size) //msbc,  16000
    {
        bt_voice_msbc_encode(fifo, fifo_size);
    }
    else//cvsd
    {
        bt_voice_cvsd_encode(fifo, fifo_size);
    }
}

uint8_t uplink_data_send(uint8_t idx, uint32_t datahdr)
{
    uint8_t i;
    rt_uint32_t getnum, putnum;
    sco_uplink_data_t *pt_data = &pt_bt_voice->up_data;
    //wait uplink webrtc algorithom processing data
    RT_ASSERT((SOFT_CVSD_ENCODE == 1) && (BT_SCO_TX_HAS_HEADER == 1));
    if (pt_bt_voice->send_enable == 1 && pt_bt_voice->state == 1)
    {
        if (pt_data->hav_data[idx] == 1)
        {
            *(uint32_t *)(&pt_data->out_data[0]) = datahdr;
            putnum = rt_ringbuffer_put(g_sco_ipc.pt_mic2bt_rbf, &pt_data->out_data[0], 64);
            pt_data->hav_data[idx] = 0;
        }
        else
        {
            if ((rt_ringbuffer_data_len(&pt_bt_voice->uplink_ring) > 60) && (rt_ringbuffer_space_len(g_sco_ipc.pt_mic2bt_rbf) > 64))
            {
                getnum = rt_ringbuffer_get(&pt_bt_voice->uplink_ring, &pt_data->out_data[4], 60);
                if (getnum == 60)
                {
                    *(uint32_t *)(&pt_data->out_data[0]) = datahdr;
                    putnum = rt_ringbuffer_put(g_sco_ipc.pt_mic2bt_rbf, &pt_data->out_data[0], 64);
                    for (i = 0; i < BT_SCO_MAX_NUM; i++)
                    {
                        pt_data->hav_data[i] = 1;
                    }

                    pt_data->hav_data[idx] = 0;
                }
                else
                {
                    LOG_W("3a_w uplink buffer readerr");
                }
            }
        }
    }

    return 0;
}

uint8_t mix_process()
{
    uint8_t i, need_algo;
    uint8_t sco_num = 0;
    sco_data_t *pt_sco[BT_SCO_MAX_NUM];
    int16_t    *p_data[BT_SCO_MAX_NUM];
    uint16_t data_len;

    if (pt_bt_voice->air_mod == CVSD_MODE)
    {
        data_len = AUDIO_BT_CVSD_PCM_LEN;
    }
    else
    {
        data_len = AUDIO_MSBC_BUFFER_LEN;
    }

    for (i = 0; i < BT_SCO_MAX_NUM; i++)
    {
        if ((pt_bt_voice->dn_data[i].is_used) && ((pt_bt_voice->dn_data[i].hav_data)))
        {
            pt_sco[sco_num] = &pt_bt_voice->dn_data[i];
            p_data[sco_num] = (int16_t *)&pt_bt_voice->dn_data[i].out_data;
            sco_num++;
            pt_bt_voice->dn_data[i].hav_data = 0;
        }
    }

    if (2 == sco_num)
    {
        for (i = 0; i < data_len; i++)
        {
            p_data[0][i] = (p_data[0][i] >> 1) + (p_data[1][i] >> 1);
        }
    }
    else if (3 == sco_num)
    {
        for (i = 0; i < data_len; i++)
        {
            p_data[0][i] = (p_data[0][i] >> 2) + (p_data[1][i] >> 2)  + (p_data[2][i] >> 2);
        }
    }

    need_algo = audio_server_bt_voice_ind((uint8_t *)p_data[0], data_len);

    if (need_algo)
    {
        audio_3a_downlink((uint8_t *)p_data[0], data_len);
    }

    return 0;
}

extern uint8_t audio_3a_dnlink_buf_is_full(uint8_t size);
void bt_voice_uplink_send()
{
    //audio_server interface
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
uint8_t bt_sco_get_idx_from_hdl(uint16_t hdl)
{
    uint8_t sco_path = (hdl >> 8) - 1;

    return sco_path;
}
void bt_sco_get_param(uint8_t idx)
{
    struct hci_sync_con_cmp_evt *pt_sco;

    pt_sco = &(pt_bt_voice->sco_para[idx]);
    pt_bt_voice->sco_num++;
    memcpy(pt_sco, (g_sco_ipc.p_sco_para + idx), sizeof(struct hci_sync_con_cmp_evt));

    if (pt_sco->air_mode == TRANS_MODE)
    {
        RT_ASSERT(pt_bt_voice->sco_num == 1);
    }

    RT_ASSERT(pt_sco->air_mode == pt_bt_voice->air_mod);

    LOG_I("sco open num=%d, mode=%d,tx=%d, rx=%d\n", pt_bt_voice->sco_num, pt_sco->air_mode, pt_sco->tx_pkt_len, pt_sco->rx_pkt_len);

}

void bt_voice_downlink_process(uint8_t is_ready)
{
    uint8_t need_algo;
    rt_uint32_t getnum, putnum;
    uint8_t size = 64;
    uint8_t decode_len;
    int8_t find_hdr = 0;
    uint32_t datahdr;

    if ((pt_bt_voice) && (pt_bt_voice->state))
    {
        if (pt_bt_voice->air_mod == TRANS_MODE)
        {
            decode_len = AUDIO_MSBC_BUFFER_LEN;
        }
        else
        {
            decode_len = AUDIO_BT_CVSD_PCM_LEN;
        }
    }
    else
    {

        return;
    }
    while (rt_ringbuffer_data_len(g_sco_ipc.pt_bt2speaker_rbf) > size)
    {
        if (audio_3a_dnlink_buf_is_full(decode_len))
        {
            return;
        }

        find_hdr = 0;
        while (find_hdr == 0)
        {
            getnum = rt_ringbuffer_get(g_sco_ipc.pt_bt2speaker_rbf, (uint8_t *)&datahdr, 4);
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
            bt_sco_data_hdr *sco_hdr = (bt_sco_data_hdr *)&datahdr;
            uint8_t sco_path = bt_sco_get_idx_from_hdl(sco_hdr->scohdl);
            sco_data_t *pt_dn_data = &pt_bt_voice->dn_data[sco_path];

            pt_bt_voice->header[sco_path] = *sco_hdr;
            pt_dn_data->stat = sco_hdr->packet_status;
            pt_dn_data->handle = sco_hdr->scohdl;

            if (pt_dn_data->is_used == 0)
            {
                bt_sco_get_param(sco_path);
                pt_dn_data->is_used = 1;
            }

            if (pt_dn_data->hav_data == 1)
            {
                mix_process();
            }

            RT_ASSERT(sco_hdr->length == 0x3c);
            getnum = rt_ringbuffer_get(g_sco_ipc.pt_bt2speaker_rbf, &pt_dn_data->in_data[0], sco_hdr->length);//only support 60byte data length


            if (getnum == sco_hdr->length)
            {
                if (pt_bt_voice->air_mod == CVSD_MODE)
                {
                    bt_voice_cvsd_decode(sco_path);
                }
                else
                {
                    bt_voice_msbc_decode(sco_path);
                }

                RT_ASSERT(0 == pt_dn_data->hav_data);
                pt_dn_data->hav_data = 1;

                uplink_data_send(sco_path, datahdr);
            }
            else
            {
                LOG_I("pt_bt2speaker_rbf get err\n");
            }
        }
        else
        {
            LOG_I("pt_bt2speaker_rbf headr err\n");
        }

    }
}

void bt_voice_open(uint32_t samplerate)
{
    uint8_t i;

    if (NULL == pt_bt_voice)
    {
        uint8_t defresize, enfresize, i;
        pt_bt_voice = audio_mem_calloc(1, sizeof(bt_voice_t));
        RT_ASSERT(pt_bt_voice);
        pt_bt_voice->sco_ipc = &g_sco_ipc;

        defresize = bts2_msbc_decode_cfg_ext(0);
        enfresize = bts2_msbc_encode_cfg_ext(0);

        for (i = 0; i < BT_SCO_MAX_NUM; i++)
        {
#if SOFT_CVSD_ENCODE
            bt_cvsd_init(i);
#endif
            if (samplerate == 8000)
            {
                cvsd_g711plc_construct(&(pt_bt_voice->pcm_plc[i]));
                pt_bt_voice->air_mod = CVSD_MODE;
            }
            else
            {
                msbc_g711plc_construct(&(pt_bt_voice->pcm_plc[i]));
                pt_bt_voice->air_mod = TRANS_MODE;
            }
        }
        pt_bt_voice->p_uplink_pool  = audio_mem_calloc(1, AUDIO_BT_UPLINK_BUFFER_SIZE);
        RT_ASSERT(pt_bt_voice->p_uplink_pool);
        rt_ringbuffer_init(&(pt_bt_voice->uplink_ring), pt_bt_voice->p_uplink_pool, AUDIO_BT_UPLINK_BUFFER_SIZE);
    }

    LOG_I("bt_voice_open mode=%d,state=%d\n", pt_bt_voice->air_mod, pt_bt_voice->state);


    if (0 == pt_bt_voice->state)
    {
        pt_bt_voice->state = 1;
    }
}

void bt_voice_close()
{
    uint8_t i;

    if (pt_bt_voice)
    {
        _hcpu_2_lcpu_ipc_audio_notify();

        for (i = 0; i < BT_SCO_MAX_NUM; i++)
        {
#if SOFT_CVSD_ENCODE
            bt_cvsd_deinit(i);
#endif
        }
        audio_mem_free(pt_bt_voice->p_uplink_pool);
        bts2_msbc_encode_completed_ext(0);
        bts2_msbc_decode_completed_ext(0);
        audio_mem_free(pt_bt_voice);
        pt_bt_voice = NULL;
    }
}

int bt_audiopath_init(void)
{
    int ret;
    uint16_t ringbuffer_len;
    uint8_t sco_num = BT_SCO_MAX_NUM;
    ringbuffer_len = ((LCPU_HCPU_AUDIO_MEM_SIZE - 0x50 - sco_num * sizeof(struct hci_sync_con_cmp_evt)) / 8 * 4);

    g_sco_ipc.pt_bt2speaker_rbf = (struct rt_ringbuffer *)HCPU_LCPU_SHARE_MEM_BASE_ADDR;
    g_sco_ipc.pt_mic2bt_rbf = (struct rt_ringbuffer *)(HCPU_LCPU_SHARE_MEM_BASE_ADDR + 0x20);
    g_sco_ipc.p_u8_tx_rbf_pool = (uint8_t *)(HCPU_LCPU_SHARE_MEM_BASE_ADDR + 0x50);
    g_sco_ipc.p_u8_rx_rbf_pool = (uint8_t *)(HCPU_LCPU_SHARE_MEM_BASE_ADDR + 0x50 + ringbuffer_len);

    g_sco_ipc.p_sco_para = (struct hci_sync_con_cmp_evt *)(HCPU_LCPU_SHARE_MEM_BASE_ADDR + LCPU_HCPU_AUDIO_MEM_SIZE - sizeof(struct hci_sync_con_cmp_evt) * sco_num);

    //input buffer
    rt_ringbuffer_init(g_sco_ipc.pt_mic2bt_rbf, g_sco_ipc.p_u8_rx_rbf_pool, ringbuffer_len);
    rt_ringbuffer_init(g_sco_ipc.pt_bt2speaker_rbf, g_sco_ipc.p_u8_tx_rbf_pool, ringbuffer_len);

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
    q_cfg.tx_buf_addr = (uint32_t)NULL;
    q_cfg.tx_buf_addr_alias = (uint32_t)NULL;
    q_cfg.rx_buf_addr = (uint32_t)NULL;
    q_cfg.rx_ind = _hl_bt_audio_queue_rx_ind;
    q_cfg.user_data = 0;

    sys_hl_bt_audio_queue = ipc_queue_init(&q_cfg);

    RT_ASSERT(IPC_QUEUE_INVALID_HANDLE != sys_hl_bt_audio_queue);

    ipc_queue_open(sys_hl_bt_audio_queue);

    bt_audiopath_init();
    return 0;
}

INIT_COMPONENT_EXPORT(bt_voice_init);


#endif
