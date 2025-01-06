/**
  ******************************************************************************
  * @file   bts2_app_av.h
  * @author Sifli software development team
  * @brief SIFLI BT a2dp app common header file.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2023 - 2023,  Sifli Technology
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

#ifndef __BTS2_APP_AV_H
#define __BTS2_APP_AV_H

#include "rtconfig.h"

#ifdef CFG_AV

#ifdef AUDIO
    #include "audio_server.h"
#endif

#ifdef CFG_AV_SNK
    #define MAX_NUM_LOCAL_SNK_SEIDS 2
#else
    #define MAX_NUM_LOCAL_SNK_SEIDS 0
#endif

#ifdef CFG_AV_SRC
    #define MAX_NUM_LOCAL_SRC_SEIDS 2
#else
    #define MAX_NUM_LOCAL_SRC_SEIDS 0
#endif

#ifdef PKG_USING_AAC_DECODER_LIBFAAD
    #define CFG_AV_AAC
#endif


#define MAX_NUM_LOCAL_SEIDS (MAX_NUM_LOCAL_SNK_SEIDS + MAX_NUM_LOCAL_SRC_SEIDS)

#define ASSIGN_TLABEL ((U8)(inst->tlabel++ % 16))

#define MAX_ACTS  2
#define MAX_CONNS 2

#define INITIATOR            (1)
#define ACPTOR               (0)

#define MAX_NUM_RMT_SEIDS    (10)

#define SBC_MEDIA_CODEC_SC_SIZE  (8)
#define SBC_IE_SAMPLE_FREQ_MASK  ((U8)0xF0)
#define SBC_IE_CHNL_MODE_MASK    ((U8)0x0F)
#define SBC_IE_BLOCK_MASK        ((U8)0xF0)
#define SBC_IE_SUBBAND_MASK      ((U8)0x0C)
#define SBC_IE_ALLOC_METHOD_MASK ((U8)0x03)


#define AAC_MEDIA_CODEC_SC_SIZE  (10)
#define AV_DELAY_REPORT_SC_SIZE (2)
#define AV_MEDIA_TRASPORT_SIZE (2)
#define AV_CONTENT_PROTECTION_SIZE (4)


#ifndef MAX_BUFF_SIZE
    #define MAX_BUFF_SIZE      (24 * 1024)
    #define MAX_AUDIO_LEN  MAX_BUFF_SIZE/2
#endif

#define BUF_CNT 4
#define BUF_LEN     (1024*18)
#define AV_PLAY_BUF_SIZE    (24 * 1024)
//#define DECODE_BUF_LEN  (1024*4)
#define DECODE_BUF_LEN  (5120)

#define BT_MUSIC_SAMPLERATE 44100

typedef enum
{
    avidle,
    avconned,
    avconned_open,
    avconned_streaming,
    avdisced
} avseid_st;


typedef enum
{
    BQB_BI_03_C = 0,
    BQB_BI_08_C,
    BQB_BI_10_C,
    BQB_BI_11_C,
    BQB_BI_12_C,
    BQB_BI_13_C,
    BQB_BI_14_C,
    BQB_BI_15_C,
    BQB_BI_16_C,
    BQB_BI_20_C,
    A2DP_SRC_AS_BV_01_I,
    A2DP_SRC_AS_BI_01_I,
    A2DP_SRC_AVP_BI_10_C,
    A2DP_SRC_AVP_BI_11_C,
    A2DP_SRC_AVP_BI_12_C,
    A2DP_SRC_AVP_BI_14_C,
    A2DP_SRC_AVP_BI_16_C,
    A2DP_SRC_AVP_BI_17_C,
    A2DP_SRC_AVP_BI_18_C,
    A2DP_SRC_AVP_BI_20_C,
    A2DP_SRC_AVP_BI_21_C,
    A2DP_SRC_AVP_BI_24_C,
    A2DP_SRC_AVP_BI_25_C,
    A2DP_SRC_AVP_BI_26_C,
    A2DP_SRC_AVP_BI_30_C,
    A2DP_SRC_AVP_BI_31_C,
    A2DP_SRC_AVP_BI_32_C,
    A2DP_SRC_AVP_BI_33_C,
    A2DP_SRC_CC_BV_09_I,
    A2DP_SRC_CC_BV_10_I,
    A2DP_SRC_REL_BV_01_I,
    A2DP_SRC_REL_BV_02_I,
    A2DP_SRC_SET_BV_01_I,
    A2DP_SRC_SET_BV_02_I,
    A2DP_SRC_SET_BV_03_I,
    A2DP_SRC_SET_BV_04_I,
    A2DP_SRC_SET_BV_05_I,
    A2DP_SRC_SET_BV_06_I,
    A2DP_SRC_SUS_BV_01_I,
    A2DP_SRC_SUS_BV_02_I,
    BQB_TEST_RESET = 0xff
} BQB_TEST;


#if defined(CFG_AV_SRC) && defined(AUDIO_USING_MANAGER)
typedef enum
{
    AVSRC_AUDIO_SER_IDLE,
    AVSRC_AUDIO_SER_OPEN,
} avsrc_audio_ser_st;
#endif // defined(CFG_AV_SRC) && defined(AUDIO_USING_MANAGER)

typedef struct list_hdr
{
    /// Pointer to next list_hdr
    struct list_hdr *next;
} list_hdr_t;

typedef struct
{
    U8 is_enbd;
    BTS2S_AV_SEID_INFO local_seid;
} local_seid_info_t;


#ifdef CFG_AV_SNK
/// struct of a list
typedef struct play_list
{
    /// pointer to first element of the list
    list_hdr_t *first;
    /// pointer to the last element
    list_hdr_t *last;
    uint32_t  cnt;
    uint32_t  cnt_th;
    uint32_t  full_num;
    uint32_t  empty_num;
    uint32_t  total_num;
} play_list_t;

typedef struct
{
    list_hdr_t hdr;
    U16  len;
} play_data_t;
#endif // CFG_AV_SRC

typedef struct
{
    U16 serv_cap_len;
    U8  *serv_cap_data; /* only bmalloc if not error */
} bts2_rmt_capa_t;

typedef struct
{
    BTS2E_SBC_CHNL_MODE chnl_mode;
    BTS2E_SBC_ALLOC_METHOD alloc_method;
    U16 sample_freq;
    U16 frmsize;
    U8  blocks;
    U8  subbands;
    U8  min_bitpool;
    U8  max_bitpool;
    U8  bit_pool;
    U8  chnls;
    U16 frms_per_payload;
    U16 bytes_per_encoding;
    U32 samples_per_l2c_frm;
    U32 bytes_to_rd;
    U32 bytes_per_sample;
} bts2_sbc_cfg;


typedef struct
{
    bts2_sbc_cfg act_cfg;
    uint8_t local_seid_idx;
    U8 rmt_seid[MAX_NUM_RMT_SEIDS];
    bts2_rmt_capa_t rmt_capa[MAX_NUM_RMT_SEIDS];
    U16 conn_id;
    U16 cfg;  /* AV_AUDIO_SNK , AV_AUDIO_SRC*/
    avseid_st st;
    U8 role; /* INITIATOR, ACPTOR */
    U8 forcefully_suspended;
    U8 in_use;
    U8 stream_hdl;
    U8 rmt_seid_idx;
    U8 is_suspend_cfg;
    U8 is_start_cfg;
    BTS2S_BD_ADDR av_rmt_addr;
} bts2_av_conn;

#ifdef CFG_AV_SRC
typedef struct
{
    U32 m_sec_per_pkt;
    U32 u_sec_per_pkt;
    U32 u_sec_per_pkt_sum;
    S32 m_sec_time_4_next_pkt;
    U32 stream_frm_time_begin;
    U32 stream_frm_time_end;
#ifdef AUDIO_USING_MANAGER
    audio_device_input_callback input_cb;
    avsrc_audio_ser_st audio_state;
    U32 tid;
#endif
} bts2s_avsrc_inst_data;
#endif //CFG_AV_SRC

#ifdef CFG_AV_SNK
typedef struct
{
    //char play_buf[AV_PLAY_BUF_SIZE];
    short *audio_ptr;
    U32 m_sec_per_pkt;
    U32 m_sec_time_4_next_pkt;
    U32 stream_frm_time_begin;
    U32 stream_frm_time_end;

    int play_wr_idx;
    int play_rd_idx;
    //DWORD pcm_playback_thrd;

    U8 can_play;
    U8 filter_prompt_enable;
    U8 reveive_start;
    U8 slience_filter_enable;
    U8 slience_count;

    //HWAVEOUT play_handle;
    //HANDLE play_event;
    rt_sem_t buf_sem;
    U8  play_state;
    U8  codec;
    play_list_t playlist;
    U8  *decode_buf;
    U16 decode_buf_len;
#if defined(AUDIO_USING_MANAGER) && defined(AUDIO_BT_AUDIO)
    audio_client_t audio_client;
#endif
    play_data_t *pt_curdata;
} bts2s_avsnk_inst_data;
#endif  // CFG_AV_SNK


typedef struct
{
    bts2_av_conn con[MAX_CONNS];
    U32 time_stamp;
    U16 max_frm_size;
    U16 seq_no;
    U8  tlabel;
    U8  con_idx;
    BOOL close_pending;
    BOOL suspend_pending;
    local_seid_info_t local_seid_info[MAX_NUM_LOCAL_SEIDS];
    U16 que_id;

#ifdef CFG_AV_SNK
    bts2s_avsnk_inst_data snk_data;
#endif

#ifdef CFG_AV_SRC
    bts2s_avsrc_inst_data src_data;
#endif

} bts2s_av_inst_data;

#ifdef CFG_AV_SRC
    #include "bts2_app_av_src.h"
#endif

#ifdef CFG_AV_SNK
    #include "bts2_app_av_snk.h"
#endif


#ifdef CFG_AV_SNK
    extern U8 bt_avsnk_prepare_sbc(bts2s_av_inst_data *inst, U8 con_idx);
    extern U16 bt_avsnk_calculate_decode_buffer_size(bts2s_av_inst_data *inst, U8 con_idx);
    extern void bt_avsnk_hdl_disc_handler(bts2s_av_inst_data *inst, uint8_t con_idx);
    extern int8_t bt_avsnk_hdl_start_cfm(bts2s_av_inst_data *inst, uint8_t con_idx);
    extern void bt_avsnk_close_handler(bts2s_av_inst_data *inst, uint8_t con_idx);
    extern uint8_t bt_avsnk_hdl_start_ind(bts2s_av_inst_data *inst, BTS2S_AV_START_IND *msg, uint8_t con_idx);
    extern void bt_avsnk_hdl_suspend_ind(bts2s_av_inst_data *inst, uint8_t con_idx);
    extern void bt_avsnk_hdl_streamdata_ind(bts2s_av_inst_data *inst, uint8_t con_idx, BTS2S_AV_STREAM_DATA_IND *msg);
#endif

#ifdef CFG_AV_SRC
    extern U8 bt_avsrc_prepare_sbc(bts2s_av_inst_data *inst, U8 con_idx);
    extern void bt_avsrc_hdl_disc_handler(bts2s_av_inst_data *inst, uint8_t con_idx);
    extern int8_t bt_avsrc_hdl_start_cfm(bts2s_av_inst_data *inst, uint8_t con_idx);
    extern uint8_t bt_avsrc_hdl_start_ind(bts2s_av_inst_data *inst, BTS2S_AV_START_IND *msg, uint8_t con_idx);
    extern int8_t bt_avsrc_hdl_streaming_start(bts2s_av_inst_data *inst, uint8_t con_idx);
    extern void bt_avsrc_hdl_suspend_ind(bts2s_av_inst_data *inst, uint8_t con_idx);
    extern void bt_avsrc_hdl_abort_ind(bts2s_av_inst_data *inst, uint8_t con_idx);
    extern void bt_avrc_close_handler(bts2s_av_inst_data *inst, uint8_t con_idx);
    extern BOOL bt_avrc_check_stream_state(void);
#endif

#ifdef AUDIO_USING_MANAGER
    extern void bt_av_start_stream_by_audio_server(uint8_t con_idx);
    extern void bt_av_suspend_stream_by_audio_server(uint8_t con_idx);
#endif

#ifdef CFG_AV_SNK
    void bt_av_snk_open(void);
    void bt_av_snk_close(void);
    void bt_av_unregister_sdp(U16 local_role);
    void bt_av_register_sdp(U16 local_role);
#endif

extern void bt_av_disconnect(uint8_t con_idx);
extern void bt_av_suspend_stream(uint8_t con_idx);
extern void bt_av_open_stream(void);
extern void bt_av_start_stream(uint8_t con_idx);
extern void bt_av_release_stream(uint8_t con_idx);
extern void bt_av_abort_stream(uint8_t con_idx);
extern void bt_av_get_cfg(uint8_t con_idx);
extern void bt_av_init(bts2_app_stru *bts2_app_data);
U8 bt_av_conn_check(void);
extern void bt_av_rel(void);
extern bts2s_av_inst_data *bt_av_get_inst_data(void);
extern void bt_av_msg_handler(bts2_app_stru *bts2_app_data);
extern void bt_av_conn(BTS2S_BD_ADDR *bd_addr, uint8_t peer_role);
void bt_av_set_can_play(void);
U8 bt_av_get_receive_a2dp_start(void);
void bt_av_set_slience_filter_enable(U8 enable);
U8 bt_av_get_slience_filter_enable(void);
U8 bt_av_get_a2dp_stream_state(void);
void bt_av_hdl_set_bqb_test(U8 value);
void bt_av_hdl_reset_bqb_test(void);
void bt_av_set_filter_prompt_enable(U8 enable);
U8 bt_av_get_filter_prompt_enable(void);

#else

#define bt_av_disconnect(con_idx)
#define bt_av_suspend_stream(con_idx)
#define bt_av_open_stream()
#define bt_av_start_stream(con_idx)
#define bt_av_release_stream(con_idx)
#define bt_av_abort_stream(con_idx)
#define bt_av_get_cfg(con_idx)
#define bt_av_init(bts2_app_data)
#define bt_av_conn_check() 0
#define bt_av_rel()
#define bt_av_get_inst_data() NULL
#define bt_av_msg_handler(bts2_app_data)
#define bt_av_conn(bd_addr,peer_role)
#define bt_av_set_can_play()
#define bt_av_get_receive_a2dp_start() 0
#define bt_av_set_slience_filter_enable(enable)
#define bt_av_get_slience_filter_enable() 0
#define bt_av_get_a2dp_stream_state() 0
#define bt_av_hdl_set_bqb_test(value)
#define bt_av_hdl_reset_bqb_test()
#define bt_av_set_filter_prompt_enable(enable)
#define bt_av_get_filter_prompt_enable() 0

#endif //defined(CFG_AV)

#endif //__BTS2_APP_AV_H

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
