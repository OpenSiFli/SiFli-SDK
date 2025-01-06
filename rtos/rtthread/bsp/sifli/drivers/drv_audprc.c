/**
  ******************************************************************************
  * @file   drv_audprc.c
  * @author Sifli software development team
  * @brief   Audio Process driver adaption layer
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
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

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <string.h>
#include "board.h"
#include "drv_config.h"
#include "stdlib.h"
#ifdef BSP_ENABLE_AUD_PRC
    #include "drv_audprc.h"
#endif

#if RT_USING_DFS
    #include "dfs_file.h"
    #include "dfs_posix.h"
#endif
__WEAK uint8_t get_factory_mode_status(void)
{
    return 0;
}

#define AUDPRC_MIN_VOLUME       (-18)
#define AUDCODEC_MIN_VOLUME     (-36)
#define AUDCODEC_MAX_VOLUME     (54)
#define EQ_DEBUG_FILE_PATH      "/dyn/eq_debug.bin"
#define EQ_SYSTEM_FILE_PATH     "/eq.bin"


static int g_pdm_volume = 8 + 48; // 5 * 0.5db = 2.5db,  0.5db unit


/*code produced by eq tools start*/

/*
   note if want to change eq, g_eq_value[] must define as
   uint32_t g_eq_value[50] =
   50 is big enouth
*/
static const int8_t g_eq_version[20] = "default_bypass";
static const char g_eq_code_ver[] = "1.0.9";
int8_t g_adc_volume = 0;
int8_t g_tel_max_vol = -2;
int8_t g_tel_max_vol_level = 0;
int8_t g_tel_vol_level[16] = {-36, -34, -32, -30, -28, -26, -24, -22, -20, -17, -14, -11, -10, -8, -6, -4};

int8_t g_music_max_vol = -2;
int8_t g_music_max_vol_level = 0;
int8_t g_music_vol_level[16] = {-55, -34, -32, -30, -28, -26, -24, -22, -20, -17, -14, -11, -10, -8, -6, -4};

uint8_t  g_music_state = 10;
uint8_t  g_voice_state = 10;

uint32_t  g_music_eqValue[50] =
{
    0x00200000, 0x0, 0x0, 0x0, 0x0,
    0x00200000, 0x0, 0x0, 0x0, 0x0,
    0x00200000, 0x0, 0x0, 0x0, 0x0,
    0x00200000, 0x0, 0x0, 0x0, 0x0,
    0x00200000, 0x0, 0x0, 0x0, 0x0,
    0x00200000, 0x0, 0x0, 0x0, 0x0,
    0x00200000, 0x0, 0x0, 0x0, 0x0,
    0x00200000, 0x0, 0x0, 0x0, 0x0,
    0x00200000, 0x0, 0x0, 0x0, 0x0,
    0x00200000, 0x0, 0x0, 0x0, 0x0,
};

uint32_t g_voice_eqValue[50] =
{
    0x00200000, 0x0, 0x0, 0x0, 0x0,
    0x00200000, 0x0, 0x0, 0x0, 0x0,
    0x00200000, 0x0, 0x0, 0x0, 0x0,
    0x00200000, 0x0, 0x0, 0x0, 0x0,
    0x00200000, 0x0, 0x0, 0x0, 0x0,
    0x00200000, 0x0, 0x0, 0x0, 0x0,
    0x00200000, 0x0, 0x0, 0x0, 0x0,
    0x00200000, 0x0, 0x0, 0x0, 0x0,
    0x00200000, 0x0, 0x0, 0x0, 0x0,
    0x00200000, 0x0, 0x0, 0x0, 0x0,
};

/****** BASE POINT INFO
30            Band EQ       0.00          0.75
60            Band EQ       0.00          0.75
120           Band EQ       0.00          0.75
240           Band EQ       0.00          0.75
480           Band EQ       0.00          0.75
1000          Band EQ       0.00          0.75
2000          Band EQ       0.00          0.75
4000          Band EQ       0.00          0.75
8000          Band EQ       0.00          0.75
16000         Band EQ       0.00          0.75
******/


/*code produced by eq tools start*/


typedef struct
{
    uint8_t version[20];
    uint32_t music_eqValue[50];
    uint32_t voice_eqValue[50];
    uint8_t music_eqNum;
    uint8_t voice_eqNum;
    int8_t  dGain;
    int8_t voicMaxValue;
    int8_t voicMaxLevel;
    int8_t musicMaxValue;
    int8_t musicMaxLevel;
    int8_t reserved;
    int8_t voiceLevelValue[16];
    int8_t musicLevelValue[16];
} eqbin_info_t;


static uint8_t g_eq_debuging = 0;
static uint8_t g_eq_default_volume = 15;
static uint8_t g_eq_is_bt_music = 0;
static uint16_t g_eq_samplerate = 0;

#if defined (SYS_HEAP_IN_PSRAM)
    #undef calloc
    #undef free
    extern void *app_sram_calloc(rt_size_t count, rt_size_t size);
    extern void *app_sram_free(void *ptr);
    #define  calloc(c, s)   app_sram_calloc(c, s)
    #define  free(p)        app_sram_free(p)
#endif

static int load_eq(void)
{
#if RT_USING_DFS
    int fd = open(EQ_DEBUG_FILE_PATH, O_RDONLY | O_BINARY);
    if (fd < 0)
        fd = open(EQ_SYSTEM_FILE_PATH, O_RDONLY | O_BINARY);
    if (fd < 0)
        return 0;

    eqbin_info_t *bin = (eqbin_info_t *)rt_malloc(sizeof(eqbin_info_t));
    RT_ASSERT(bin);
    int ret = read(fd, bin, sizeof(eqbin_info_t));
    if (ret != sizeof(eqbin_info_t))
    {
        rt_kprintf("buildin eq config:\r\n");
        goto Exit;
    }
    g_adc_volume = bin->dGain;
    g_music_state = bin->music_eqNum;
    g_voice_state = bin->voice_eqNum;
    memcpy(g_music_eqValue, bin->music_eqValue, sizeof(uint32_t) * 50);
    memcpy(g_voice_eqValue, bin->voice_eqValue, sizeof(uint32_t) * 50);
    g_music_max_vol = bin->musicMaxValue;
    g_music_max_vol_level = bin->musicMaxLevel;
    memcpy(g_music_vol_level, bin->musicLevelValue, sizeof(g_music_vol_level));
    g_tel_max_vol = bin->voicMaxValue;
    g_tel_max_vol_level = bin->voicMaxLevel;
    memcpy(g_tel_vol_level, bin->voiceLevelValue, sizeof(g_tel_vol_level));
    rt_kprintf("load eq config:\r\n");
Exit:
    close(fd);
    rt_free(bin);
    rt_kprintf("g_adc_volume=%d\r\n", g_adc_volume);
    rt_kprintf("g_music_state=%d\r\n", g_music_state);
    rt_kprintf("g_voice_state=%d\r\n", g_voice_state);
    rt_kprintf("g_music_max_vol=%d\r\n", g_music_max_vol);
    rt_kprintf("g_music_max_vol_level=%d\r\n", g_music_max_vol_level);
    for (int i = 0; i < 16; i++)
    {
        rt_kprintf("music level %d=%d\r\n", i, g_music_vol_level[i]);
    }
    rt_kprintf("g_tel_max_vol=%d\r\n", g_tel_max_vol);
    rt_kprintf("g_tel_max_vol_level=%d\r\n", g_tel_max_vol_level);
    for (int i = 0; i < 16; i++)
    {
        rt_kprintf("voice level %d=%d\r\n", i, g_tel_vol_level[i]);
    }

#endif
    return 0;
}

INIT_PRE_APP_EXPORT(load_eq);

static void save_eq()
{
#if RT_USING_DFS
    int fd = open(EQ_DEBUG_FILE_PATH, O_RDWR | O_CREAT | O_TRUNC | O_BINARY);
    RT_ASSERT(fd >= 0);
    eqbin_info_t *bin = (eqbin_info_t *)rt_malloc(sizeof(eqbin_info_t));
    RT_ASSERT(bin);
    bin->dGain = g_adc_volume;
    bin->music_eqNum = g_music_state;
    bin->voice_eqNum = g_voice_state;
    memcpy(bin->version, g_eq_version, 20);
    bin->reserved = 'A';
    memcpy(bin->music_eqValue, g_music_eqValue, sizeof(uint32_t) * 50);
    memcpy(bin->voice_eqValue, g_voice_eqValue, sizeof(uint32_t) * 50);
    bin->musicMaxValue = g_music_max_vol;
    bin->musicMaxLevel = g_music_max_vol_level;
    memcpy(bin->musicLevelValue, g_music_vol_level, sizeof(g_music_vol_level));
    bin->voicMaxValue = g_tel_max_vol;
    bin->voicMaxLevel = g_tel_max_vol_level;
    memcpy(bin->voiceLevelValue, g_tel_vol_level, sizeof(g_tel_vol_level));
    int ret = write(fd, bin, sizeof(eqbin_info_t));
    RT_ASSERT(ret ==  sizeof(eqbin_info_t));
    close(fd);
    rt_free(bin);
#endif
    return;
}

uint8_t eq_is_working()
{
    return g_eq_debuging;
}

int8_t get_mic_volume()
{

    if (get_factory_mode_status())
    {
        rt_kprintf("get_mic_volume = factory -4\r\n");
        return -4;
    }
    else
    {
        rt_kprintf("get_mic_volume = %ddb\r\n", g_adc_volume);
        return g_adc_volume;
    }
}
int eq_get_default_volumex2()
{
    if (g_eq_is_bt_music)
        return eq_get_music_volumex2(g_eq_default_volume);

    return eq_get_tel_volumex2(g_eq_default_volume);
}

int eq_get_music_volumex2(uint8_t level)
{
    int vol = g_music_vol_level[level & 0x0F];
    if (vol  < (AUDPRC_MIN_VOLUME + AUDCODEC_MIN_VOLUME))
        return MUTE_UNDER_MIN_VOLUME;
    if (vol * 2 > g_music_max_vol)
        return g_music_max_vol;
    return vol * 2;
}

int eq_get_tel_volumex2(uint8_t level)
{
    int vol = g_tel_vol_level[level & 0x0F];
    if (vol  < (AUDPRC_MIN_VOLUME + AUDCODEC_MIN_VOLUME))
        return MUTE_UNDER_MIN_VOLUME;
    if (vol * 2 > g_tel_max_vol)
        return g_tel_max_vol;
    return vol * 2;
}

int8_t eq_get_decrease_level(int is_tel, int volumex2)
{
    if (is_tel)
    {
        if (volumex2 == g_tel_max_vol)
            return g_tel_max_vol_level;
    }
    else if (volumex2 == g_music_max_vol)
        return g_music_max_vol_level;

    return 0;
}

void eq_get_version(uint8_t version[20])
{
#if RT_USING_DFS
    int ret = 0;
    int fd = open(EQ_DEBUG_FILE_PATH, O_RDONLY | O_BINARY);
    if (fd < 0)
        fd = open(EQ_SYSTEM_FILE_PATH, O_RDONLY | O_BINARY);

    if (fd < 0)
    {
        memcpy(version, g_eq_version, 20);
        return;
    }
    if (read(fd, version, 20) != 20)
    {
        memcpy(version, g_eq_version, 20);
    }
    close(fd);
#endif
}

#ifdef FPGA
static int bf0_enable_pll(uint32_t freq, uint8_t type)
{
    return 0;
}
static void bf0_disable_pll()
{

}
static void set_pll_state(uint8_t state)
{

}

#endif

#if defined(BSP_ENABLE_AUD_PRC) ||defined(_SIFLI_DOXYGEN_)

#define LOG_TAG              "drv.audprc"
//#define DBG_LEVEL                DBG_LOG
#include "drv_log.h"

extern HAL_StatusTypeDef HAL_AUDPRC_Config_ADCPath_Volume(AUDPRC_HandleTypeDef *haprc, int channel, int volume);

#define AUDPRC_DMA_RBF_NUM  16
struct bf0_audio_prc
{
    struct rt_audio_device audio_device;    /*!< parent  audio device registerd to OS*/

    AUDPRC_HandleTypeDef audprc;
    uint32_t slot_valid;
    uint8_t *queue_buf[HAL_AUDPRC_INSTANC_CNT];
    uint8_t tx_instanc;
    uint8_t rx_instanc;
    bool    tx_rbf_enable;
    bool    rx_rbf_enable;
    uint8_t rbf_tx_pool[AUDPRC_DMA_RBF_NUM];
    uint8_t rbf_rx_pool[AUDPRC_DMA_RBF_NUM];
    struct rt_ringbuffer *rbf_tx_instanc;
    struct rt_ringbuffer *rbf_rx_instanc;
    uint8_t eq_opened;
};

typedef struct _src_table_
{
    uint16_t fsin;
    uint16_t fsout;
    uint8_t hbf1_en;    // x2 or /2
    uint8_t hbf1_mode;  // 0 for hbf1 x2, 1 for hbf1 /2
    uint8_t hbf2_en;
    uint8_t hbf2_mode;
    uint8_t hbf3_en;
    uint8_t hbf3_mode;
    uint8_t sinc_en;        // source/dst not 2^ multi
    uint32_t sinc_ratio;    // source after hbf(1&2&3) / dst, only 31 bits valid
} audprc_src_table_t;

static struct bf0_audio_prc h_aud_prc;
const audprc_src_table_t src_table[] =
{
    {8000,  11025, 0, 0, 0, 0, 0, 0, 1, 0x2e709c60},
    {8000,  12000, 0, 0, 0, 0, 0, 0, 1, 0x2aaaaaa8},
    {8000,  16000, 1, 0, 0, 0, 0, 0, 0, 0},
    {8000,  22050, 1, 0, 0, 0, 0, 0, 1, 0x2e706d80},
    {8000,  24000, 1, 0, 1, 0, 0, 0, 1, 0x55553700},
    {8000,  32000, 1, 0, 1, 0, 0, 0, 0, 0},
    {8000,  44100, 1, 0, 1, 0, 0, 0, 1, 0x2e702f00},
    {8000,  48000, 1, 0, 1, 0, 0, 0, 1, 0x2aaa5d00},
    {11025,   8000, 0, 0, 0, 0, 0, 0, 1, 0x583313d9},
    {11025, 12000, 0, 0, 0, 0, 0, 0, 1, 0x3accb7e6},
    {11025, 16000, 0, 0, 0, 0, 0, 0, 1, 0x2c197464},
    {11025, 22050, 1, 0, 0, 0, 0, 0, 0, 0},
    {11025, 24000, 1, 0, 0, 0, 0, 0, 1, 0x3accb7e6},
    {11025, 32000, 1, 0, 0, 0, 0, 0, 1, 0x2c197464},
    {11025, 44100, 1, 0, 1, 0, 0, 0, 0, 0},
    {11025, 48000, 1, 0, 1, 0, 0, 0, 1, 0x3acc61c4},
    {12000,   8000, 1, 1, 0, 0, 0, 0, 1, 0x2fffeef0},
    {12000, 11025, 0, 0, 0, 0, 0, 0, 1, 0x45a8d320},
    {12000, 16000, 0, 0, 0, 0, 0, 0, 1, 0x2fffd780},
    {12000, 22050, 1, 0, 0, 0, 0, 0, 1, 0x45a8a440},
    {12000, 24000, 1, 0, 0, 0, 0, 0, 0, 0},
    {12000, 32000, 1, 0, 0, 0, 0, 0, 1, 0x2fffd780},
    {12000, 44100, 1, 0, 1, 0, 0, 0, 1, 0x45a84680},
    {12000, 48000, 1, 0, 1, 0, 0, 0, 0, 0},
    {16000,   8000, 1, 1, 0, 0, 0, 0, 0, 0},
    {16000, 11025, 0, 0, 0, 0, 0, 0, 1, 0x5ce11980},
    {16000, 12000, 0, 0, 0, 0, 0, 0, 1, 0x55553700},
    {16000, 22050, 0, 0, 0, 0, 0, 0, 1, 0x2e706d80},
    {16000, 24000, 0, 0, 0, 0, 0, 0, 1, 0x2aaa9b80},
    {16000, 32000, 1, 0, 0, 0, 0, 0, 0, 0},
    {16000, 44100, 1, 0, 0, 0, 0, 0, 1, 0x2e702f00},
    {16000, 48000, 1, 0, 0, 0, 0, 0, 1, 0x2aaa5d00},
    {22050,   8000, 1, 1, 0, 0, 0, 0, 1, 0x583313d9},
    {22050, 11025, 1, 1, 0, 0, 0, 0, 0, 0},
    {22050, 12000, 1, 1, 0, 0, 0, 0, 1, 0x3accb7e6},
    {22050, 16000, 0, 0, 0, 0, 0, 0, 1, 0x5832e8c8},
    {22050, 24000, 0, 0, 0, 0, 0, 0, 1, 0x3accb7e6},
    {22050, 32000, 0, 0, 0, 0, 0, 0, 1, 0x2c197464},
    {22050, 44100, 1, 0, 0, 0, 0, 0, 0, 0},
    {22050, 48000, 1, 0, 0, 0, 0, 0, 1, 0x3acc61c4},
    {24000,   8000, 1, 1, 0, 0, 0, 0, 1, 0x5fffdde0},
    {24000, 11025, 1, 1, 0, 0, 0, 0, 1, 0x45a8d320},
    {24000, 12000, 1, 1, 0, 0, 0, 0, 0, 0},
    {24000, 16000, 1, 1, 0, 0, 0, 0, 1, 0x2fffd780},
    {24000, 22050, 0, 0, 0, 0, 0, 0, 1, 0x45a8a440},
    {24000, 32000, 0, 0, 0, 0, 0, 0, 1, 0x2fffd780},
    {24000, 44100, 1, 0, 0, 0, 0, 0, 1, 0x45a84680},
    {24000, 48000, 1, 0, 0, 0, 0, 0, 0, 0},
    {32000,   8000, 1, 1, 1, 1, 0, 0, 0, 0},
    {32000, 11025, 1, 1, 0, 0, 0, 0, 1, 0x5ce11980},
    {32000, 12000, 1, 1, 0, 0, 0, 0, 1, 0x55553700},
    {32000, 16000, 1, 1, 0, 0, 0, 0, 0, 0},
    {32000, 22050, 1, 1, 0, 0, 0, 0, 1, 0x2e706d80},
    {32000, 24000, 0, 0, 0, 0, 0, 0, 1, 0x55553700},
    {32000, 44100, 0, 0, 0, 0, 0, 0, 1, 0x2e702f00},
    {32000, 48000, 0, 0, 0, 0, 0, 0, 1, 0x2aaa5d00},
    {44100,   8000, 1, 1, 1, 1, 0, 0, 1, 0x583313d9},
    {44100, 11025, 1, 1, 1, 1, 0, 0, 0, 0},
    {44100, 12000, 1, 1, 1, 1, 0, 0, 1, 0x3accb7e6},
    {44100, 16000, 1, 1, 0, 0, 0, 0, 1, 0x5832e8c8},
    {44100, 22050, 1, 1, 0, 0, 0, 0, 0, 0},
    {44100, 24000, 1, 1, 0, 0, 0, 0, 1, 0x3accb7e6},
    {44100, 32000, 0, 0, 0, 0, 0, 0, 1, 0x5832e8c8},
    {44100, 48000, 0, 0, 0, 0, 0, 0, 1, 0x3acc61c4},
    {48000,   8000, 1, 1, 1, 1, 1, 1, 1, 0x2fffeef0},
    {48000, 11025, 1, 1, 1, 1, 0, 0, 1, 0x45a8d320},
    {48000, 12000, 1, 1, 1, 1, 0, 0, 0, 0},
    {48000, 16000, 1, 1, 0, 0, 0, 0, 1, 0x5fffaf00},
    {48000, 22050, 1, 1, 0, 0, 0, 0, 1, 0x45a8a440},
    {48000, 24000, 1, 1, 0, 0, 0, 0, 0, 0},
    {48000, 32000, 1, 1, 0, 0, 0, 0, 1, 0x2fffd780},
    {48000, 44100, 0, 0, 0, 0, 0, 0, 1, 0x45a84680},
};


const AUDPRC_CLK_CONFIG_TYPE   audprc_clk_cfg_tb[9] =
{
#if ALL_CLK_USING_PLL
    {48000, 1,  1000},
    {32000, 1,  1500},
    {24000, 1,  2000},
    {16000, 1,  3000},
    {12000, 1,  4000},
    { 8000, 1,  6000},
#else
    {48000, 0,  1000},
    {32000, 0,  1500},
    {24000, 0,  2000},
    {16000, 0,  3000},
    {12000, 0,  4000},
    { 8000, 0,  6000},
#endif
    {44100, 1,  1000},
    {22050, 1,  2000},
    {11025, 1,  4000},
};
static uint8_t g_rx_stop;
static inline int get_audprc_adc_volume();
AUDPRC_HandleTypeDef *get_audprc_handle()
{
    return &h_aud_prc.audprc;
}
static void ADPRC_DUMP_REG()
{
    volatile uint32_t *ptr = (volatile uint32_t *)AUDPRC_BASE;
    int i;
    for (i = 0; i < 40; i++)
    {
        LOG_RAW("0x%08x ", *ptr);
        ptr++;
        if ((i & 3) == 3)
            LOG_RAW("\r\n");
    }
}

void bf0_audprc_stop()
{
    AUDPRC_HandleTypeDef *haprc = get_audprc_handle();
    __HAL_AUDPRC_DISABLE(haprc);
    haprc->Init.adc_cfg.rx2tx_loopback = 0;
    __HAL_AUDPRC_SRESET_START(haprc);
    __HAL_AUDPRC_SRESET_STOP(haprc);
}

#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
void AUDPRC_TX0_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    //LOG_I("AUDPRC_TX0_DMA_IRQHandler");

    HAL_DMA_IRQHandler(h_aud_prc.audprc.hdma[0]);

    /* leave interrupt */
    rt_interrupt_leave();
}

void AUDPRC_TX1_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    //LOG_I("AUDPRC_TX1_DMA_IRQHandler");

    HAL_DMA_IRQHandler(h_aud_prc.audprc.hdma[1]);

    /* leave interrupt */
    rt_interrupt_leave();
}

void AUDPRC_TX2_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    //LOG_I("AUDPRC_TX2_DMA_IRQHandler");

    HAL_DMA_IRQHandler(h_aud_prc.audprc.hdma[2]);

    /* leave interrupt */
    rt_interrupt_leave();
}

void AUDPRC_TX3_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    //LOG_I("AUDPRC_TX3_DMA_IRQHandler");

    HAL_DMA_IRQHandler(h_aud_prc.audprc.hdma[3]);

    /* leave interrupt */
    rt_interrupt_leave();
}

void AUDPRC_RX0_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    //LOG_I("AUDPRC_RX0_DMA_IRQHandler");

    HAL_DMA_IRQHandler(h_aud_prc.audprc.hdma[4]);

    /* leave interrupt */
    rt_interrupt_leave();
}

void AUDPRC_RX1_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    //LOG_I("AUDPRC_RX1_DMA_IRQHandler");

    HAL_DMA_IRQHandler(h_aud_prc.audprc.hdma[5]);

    /* leave interrupt */
    rt_interrupt_leave();
}

void AUDPRC_TX_OUT0_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    //LOG_I("AUDPRC_TX_OUT0_DMA_IRQHandler");

    HAL_DMA_IRQHandler(h_aud_prc.audprc.hdma[6]);

    /* leave interrupt */
    rt_interrupt_leave();
}

void AUDPRC_TX_OUT1_DMA_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    //LOG_I("AUDPRC_TX_OUT1_DMA_IRQHandler");

    HAL_DMA_IRQHandler(h_aud_prc.audprc.hdma[7]);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */

static void bf0_adc_dac_path_cfg_init(AUDPRC_HandleTypeDef *haudprc)
{
    // adc path configure
    haudprc->Init.adc_cfg.src_hbf3_mode = 0;
    haudprc->Init.adc_cfg.src_hbf3_en = 0;
    haudprc->Init.adc_cfg.src_hbf2_mode = 0;
    haudprc->Init.adc_cfg.src_hbf2_en = 0;
    haudprc->Init.adc_cfg.src_hbf1_mode = 0;
    haudprc->Init.adc_cfg.src_hbf1_en = 0;
    haudprc->Init.adc_cfg.src_ch_en = 0;
    haudprc->Init.adc_cfg.rx2tx_loopback = 0;
    haudprc->Init.adc_cfg.data_swap = 0;
    haudprc->Init.adc_cfg.src_sel = 0;
    haudprc->Init.adc_cfg.vol_l = get_audprc_adc_volume();
    haudprc->Init.adc_cfg.vol_r = get_audprc_adc_volume();
    haudprc->Init.adc_cfg.src_sinc_en = 0;
    haudprc->Init.adc_cfg.sinc_ratio = 0;

    // dac path configure
    haudprc->Init.dac_cfg.dst_sel = 0;
    haudprc->Init.dac_cfg.mixrsrc1 = 5;
    haudprc->Init.dac_cfg.mixrsrc0 = 1;
    haudprc->Init.dac_cfg.mixlsrc1 = 5;
    haudprc->Init.dac_cfg.mixlsrc0 = 0;
#ifdef  APP_BSP_TEST
#ifdef SF32LB52X
    haudprc->Init.dac_cfg.vol_r = 0;
    haudprc->Init.dac_cfg.vol_l = 0;
#else
    haudprc->Init.dac_cfg.vol_r = 0;
    haudprc->Init.dac_cfg.vol_l = 0;//-18~13
#endif
#else
    haudprc->Init.dac_cfg.vol_r = 0;//36;
    haudprc->Init.dac_cfg.vol_l = 0;//36;
#endif
    haudprc->Init.dac_cfg.src_hbf3_mode = 0;
    haudprc->Init.dac_cfg.src_hbf3_en = 0;
    haudprc->Init.dac_cfg.src_hbf2_mode = 0;
    haudprc->Init.dac_cfg.src_hbf2_en = 0;
    haudprc->Init.dac_cfg.src_hbf1_mode = 0;
    haudprc->Init.dac_cfg.src_hbf1_en = 0;
    haudprc->Init.dac_cfg.src_ch_en = 0;
    haudprc->Init.dac_cfg.eq_clr = 0;
    haudprc->Init.dac_cfg.eq_stage = 1;
    haudprc->Init.dac_cfg.eq_ch_en = 0;
    haudprc->Init.dac_cfg.muxrsrc1 = 5;
    haudprc->Init.dac_cfg.muxrsrc0 = 1;
    haudprc->Init.dac_cfg.muxlsrc1 = 5;
    haudprc->Init.dac_cfg.muxlsrc0 = 0;
    haudprc->Init.dac_cfg.src_sinc_en = 0;
    haudprc->Init.dac_cfg.sinc_ratio = 0;
}
static rt_err_t bf0_audprc_src(struct rt_audio_device *audio, uint16_t source, uint16_t dest, uint8_t input)
{
    int i;
    struct bf0_audio_prc *audprc = (struct bf0_audio_prc *) audio;
    AUDPRC_HandleTypeDef *haudprc = (AUDPRC_HandleTypeDef *) & (audprc->audprc);
    int length = sizeof(src_table) / (sizeof(audprc_src_table_t));

    for (i = 0; i < 9; i++)
    {
        if (dest == audprc_clk_cfg_tb[i].samplerate)
        {
            haudprc->Init.adc_div = audprc_clk_cfg_tb[i].clk_div;
            haudprc->Init.dac_div = audprc_clk_cfg_tb[i].clk_div;
            haudprc->Init.clk_sel = audprc_clk_cfg_tb[i].clk_src_sel;
            break;
        }
    }
    if (haudprc->Init.clk_sel == 0)
    {
        __HAL_AUDPRC_CLK_XTAL(haudprc);
    }
    else
    {
        __HAL_AUDPRC_CLK_PLL(haudprc);
    }
    __HAL_AUDPRC_STB_DIV_CLK(haudprc, haudprc->Init.adc_div, haudprc->Init.dac_div);
    if (source == dest)
    {
        bf0_adc_dac_path_cfg_init(haudprc);
        if (input)  // adc
        {
            haudprc->Init.adc_cfg.src_ch_en = 0;
            HAL_AUDPRC_Config_ADCPath(haudprc, &(haudprc->Init.adc_cfg));
        }
        else // dac
        {
            haudprc->Init.dac_cfg.src_ch_en = 0;
            HAL_AUDPRC_Config_DACPath(haudprc, &(haudprc->Init.dac_cfg));
        }
        return RT_EOK;
    }

    for (i = 0; i < length; i++)
    {
        if ((source == src_table[i].fsin) && (dest == src_table[i].fsout))   // found  valid table
        {
            LOG_I("table %d meet request\n", i);
            break;
        }
    }
    if (i >= length) // not found valid table
        return RT_ERROR;

    if (input)  // adc
    {
        haudprc->Init.adc_cfg.src_hbf3_mode = src_table[i].hbf3_mode;
        haudprc->Init.adc_cfg.src_hbf3_en = src_table[i].hbf3_en;
        haudprc->Init.adc_cfg.src_hbf2_mode = src_table[i].hbf2_mode;
        haudprc->Init.adc_cfg.src_hbf2_en = src_table[i].hbf2_en;
        haudprc->Init.adc_cfg.src_hbf1_mode = src_table[i].hbf1_mode;
        haudprc->Init.adc_cfg.src_hbf1_en = src_table[i].hbf1_en;
        haudprc->Init.adc_cfg.src_sinc_en = src_table[i].sinc_en;
        haudprc->Init.adc_cfg.sinc_ratio = src_table[i].sinc_ratio;
        haudprc->Init.adc_cfg.src_ch_en = 3;
        HAL_AUDPRC_Config_ADCPath(haudprc, &(haudprc->Init.adc_cfg));
    }
    else // dac
    {
        haudprc->Init.dac_cfg.src_hbf3_mode = src_table[i].hbf3_mode;
        haudprc->Init.dac_cfg.src_hbf3_en = src_table[i].hbf3_en;
        haudprc->Init.dac_cfg.src_hbf2_mode = src_table[i].hbf2_mode;
        haudprc->Init.dac_cfg.src_hbf2_en = src_table[i].hbf2_en;
        haudprc->Init.dac_cfg.src_hbf1_mode = src_table[i].hbf1_mode;
        haudprc->Init.dac_cfg.src_hbf1_en = src_table[i].hbf1_en;
        haudprc->Init.dac_cfg.src_sinc_en = src_table[i].sinc_en;
        haudprc->Init.dac_cfg.sinc_ratio = src_table[i].sinc_ratio;
        haudprc->Init.dac_cfg.src_ch_en = 3;
        HAL_AUDPRC_Config_DACPath(haudprc, &(haudprc->Init.dac_cfg));
    }

    return RT_EOK;
}

static rt_err_t bf0_audio_getcaps(struct rt_audio_device *audio, struct rt_audio_caps *caps)
{
    rt_err_t result = RT_EOK;

    return result;
}

void print_value(const char *str, uint32_t val)
{
    rt_kprintf("%s=0x%08x\r\n", str, val);
}
static rt_err_t bf0_audio_configure(struct rt_audio_device *audio, struct rt_audio_caps *caps)
{
    rt_err_t result = RT_EOK;
    struct bf0_audio_prc *audprc = (struct bf0_audio_prc *) audio;
    AUDPRC_HandleTypeDef *haudprc = (AUDPRC_HandleTypeDef *) & (audprc->audprc);

    if (audio == NULL || caps == NULL)
        return RT_ERROR;

    switch (caps->main_type)
    {
    case AUDIO_TYPE_INPUT:
    {
        AUDPRC_ChnlCfgTypeDef cfg;
        switch (caps->sub_type) // sub type as channel index
        {
        case 0: // rx ch0
        {
#ifdef BSP_AUDPRC_RX0_DMA
            if (haudprc->buf[HAL_AUDPRC_RX_CH0] == NULL)
            {
                haudprc->buf[HAL_AUDPRC_RX_CH0] = calloc(1, haudprc->bufRxSize);
                RT_ASSERT(haudprc->buf[HAL_AUDPRC_RX_CH0]);
                if (haudprc->buf[HAL_AUDPRC_RX_CH0] == NULL)
                    return RT_ERROR_MEMFAULT;
            }
            else
            {
                memset(haudprc->buf[HAL_AUDPRC_RX_CH0], 0, haudprc->bufRxSize);
            }
            cfg.dma_mask = 0;
            cfg.en = 1;
            cfg.format = caps->udata.config.samplefmt == 16 ? 0 : 1;
            if (cfg.format == 0) // only 16 bit support stereo
                cfg.mode = caps->udata.config.channels == 1 ? 0 : 1;
            else
                cfg.mode = 0;
            //rt_kprintf("mic adc gain=%d\r\n", get_mic_volume());
            HAL_AUDPRC_Config_ADCPath_Volume(haudprc, 0, get_mic_volume());
            HAL_AUDPRC_Config_ADCPath_Volume(haudprc, 1, get_mic_volume());
            memcpy((void *)&haudprc->cfg, (void *)&cfg, sizeof(AUDPRC_ChnlCfgTypeDef));
            audprc->rx_instanc = HAL_AUDPRC_RX_CH0;
#endif
        }
        break;

        case 1:              // rx ch0
        {
#ifdef BSP_AUDPRC_RX1_DMA
            if (haudprc->buf[HAL_AUDPRC_RX_CH1] == NULL)
            {
                haudprc->buf[HAL_AUDPRC_RX_CH1] = calloc(1, haudprc->bufRxSize);
                RT_ASSERT(haudprc->buf[HAL_AUDPRC_RX_CH1]);
                if (haudprc->buf[HAL_AUDPRC_RX_CH1] == NULL)
                    return RT_ERROR_MEMFAULT;
            }
            else
            {
                memset(haudprc->buf[HAL_AUDPRC_RX_CH1], 0, haudprc->bufRxSize);
            }
            cfg.dma_mask = 0;
            cfg.en = 1;
            cfg.format = caps->udata.config.samplefmt == 16 ? 0 : 1;
            if (cfg.format == 0) // only 16 bit support stereo
                cfg.mode = caps->udata.config.channels == 1 ? 0 : 1;
            else
                cfg.mode = 0;

            HAL_AUDPRC_Config_ADCPath_Volume(haudprc, 0, get_mic_volume());
            HAL_AUDPRC_Config_ADCPath_Volume(haudprc, 1, get_mic_volume());
            memcpy((void *)&haudprc->cfg1, (void *)&cfg, sizeof(AUDPRC_ChnlCfgTypeDef));
            audprc->rx_instanc = HAL_AUDPRC_RX_CH1;
#endif
        }
        break;

        case 2:         // tx out ch0
        {
#ifdef BSP_AUDPRC_TX_OUT0_DMA
            if (haudprc->buf[HAL_AUDPRC_TX_OUT_CH0] == NULL)
            {
                haudprc->buf[HAL_AUDPRC_TX_OUT_CH0] = calloc(1, haudprc->bufRxSize);
                RT_ASSERT(haudprc->buf[HAL_AUDPRC_TX_OUT_CH0]);
                if (haudprc->buf[HAL_AUDPRC_TX_OUT_CH0] == NULL)
                    return RT_ERROR_MEMFAULT;
            }
            else
            {
                memset(haudprc->buf[HAL_AUDPRC_TX_OUT_CH0], 0, haudprc->bufRxSize);
            }
            cfg.dma_mask = 0;
            cfg.en = 1;
            cfg.format = caps->udata.config.samplefmt == 16 ? 0 : 1;
            if (cfg.format == 0) // only 16 bit support stereo
                cfg.mode = caps->udata.config.channels == 1 ? 0 : 1;
            else
                cfg.mode = 0;
            HAL_AUDPRC_Config_OutChanel(haudprc, 0, &cfg);
            audprc->rx_instanc = HAL_AUDPRC_TX_OUT_CH0;
#endif
        }
        break;
        case 3:              // tx out ch1
        {
#ifdef BSP_AUDPRC_TX_OUT1_DMA
            if (haudprc->buf[HAL_AUDPRC_TX_OUT_CH1] == NULL)
            {
                haudprc->buf[HAL_AUDPRC_TX_OUT_CH1] = calloc(1, haudprc->bufRxSize);
                RT_ASSERT(haudprc->buf[HAL_AUDPRC_TX_OUT_CH1]);
                if (haudprc->buf[HAL_AUDPRC_TX_OUT_CH1] == NULL)
                    return RT_ERROR_MEMFAULT;
            }
            else
            {
                memset(haudprc->buf[HAL_AUDPRC_TX_OUT_CH1], 0, haudprc->bufRxSize);
            }
            cfg.dma_mask = 0;
            cfg.en = 1;
            cfg.format = caps->udata.config.samplefmt == 16 ? 0 : 1;
            if (cfg.format == 0) // only 16 bit support stereo
                cfg.mode = caps->udata.config.channels == 1 ? 0 : 1;
            else
                cfg.mode = 0;
            HAL_AUDPRC_Config_OutChanel(haudprc, 1, &cfg);
            audprc->rx_instanc = HAL_AUDPRC_TX_OUT_CH1;
#endif
        }
        break;

        default:
        {
            result = -RT_ERROR;
        }
        break;
        }
    }
    break;
    case AUDIO_TYPE_OUTPUT:
    {
        AUDPRC_ChnlCfgTypeDef cfg;
        switch (caps->sub_type) // sub type as channel index
        {
        case 0:     // tx ch0
        {
#ifdef BSP_AUDPRC_TX0_DMA
            if (haudprc->buf[HAL_AUDPRC_TX_CH0])
            {
                free(haudprc->buf[HAL_AUDPRC_TX_CH0]);
            }

            haudprc->buf[HAL_AUDPRC_TX_CH0] = calloc(1, haudprc->bufTxSize);
            RT_ASSERT(haudprc->buf[HAL_AUDPRC_TX_CH0]);
            if (haudprc->buf[HAL_AUDPRC_TX_CH0] == NULL)
                return RT_ERROR_MEMFAULT;

            audprc->queue_buf[HAL_AUDPRC_TX_CH0] = haudprc->buf[HAL_AUDPRC_TX_CH0];
            cfg.dma_mask = 0;
            cfg.en = 1;
            cfg.format = caps->udata.config.samplefmt == 16 ? 0 : 1;
            if (cfg.format == 0) // only 16 bit support stereo
                cfg.mode = caps->udata.config.channels == 1 ? 0 : 1;
            else
                cfg.mode = 0;
            g_eq_samplerate = caps->udata.config.samplerate;
            HAL_AUDPRC_Config_TChanel(haudprc, 0, &cfg);
            audprc->tx_instanc = HAL_AUDPRC_TX_CH0;
#endif
        }
        break;

        case 1:               // tx ch1
        {
#ifdef BSP_AUDPRC_TX1_DMA
            if (haudprc->buf[HAL_AUDPRC_TX_CH1])
            {
                free(haudprc->buf[HAL_AUDPRC_TX_CH1]);
            }

            haudprc->buf[HAL_AUDPRC_TX_CH1] = calloc(1, haudprc->bufTxSize);
            RT_ASSERT(haudprc->buf[HAL_AUDPRC_TX_CH1]);
            if (haudprc->buf[HAL_AUDPRC_TX_CH1] == NULL)
                return RT_ERROR_MEMFAULT;

            audprc->queue_buf[HAL_AUDPRC_TX_CH1] = haudprc->buf[HAL_AUDPRC_TX_CH1];
            cfg.dma_mask = 0;
            cfg.en = 1;
            cfg.format = caps->udata.config.samplefmt == 16 ? 0 : 1;
            if (cfg.format == 0) // only 16 bit support stereo
                cfg.mode = caps->udata.config.channels == 1 ? 0 : 1;
            else
                cfg.mode = 0;
            g_eq_samplerate = caps->udata.config.samplerate;
            HAL_AUDPRC_Config_TChanel(haudprc, 1, &cfg);
            audprc->tx_instanc = HAL_AUDPRC_TX_CH1;
#endif

        }
        break;

        case 2:              // tx ch2
        {
#ifdef BSP_AUDPRC_TX2_DMA
            if (haudprc->buf[HAL_AUDPRC_TX_CH2])
            {
                free(haudprc->buf[HAL_AUDPRC_TX_CH2]);
            }

            haudprc->buf[HAL_AUDPRC_TX_CH2] = calloc(1, haudprc->bufTxSize);
            RT_ASSERT(haudprc->buf[HAL_AUDPRC_TX_CH2]);
            if (haudprc->buf[HAL_AUDPRC_TX_CH2] == NULL)
                return RT_ERROR_MEMFAULT;

            audprc->queue_buf[HAL_AUDPRC_TX_CH2] = haudprc->buf[HAL_AUDPRC_TX_CH2];
            cfg.dma_mask = 0;
            cfg.en = 1;
            cfg.format = caps->udata.config.samplefmt == 16 ? 0 : 1;
            if (cfg.format == 0) // only 16 bit support stereo
                cfg.mode = caps->udata.config.channels == 1 ? 0 : 1;
            else
                cfg.mode = 0;
            g_eq_samplerate = caps->udata.config.samplerate;
            HAL_AUDPRC_Config_TChanel(haudprc, 2, &cfg);
            audprc->tx_instanc = HAL_AUDPRC_TX_CH2;
#endif

        }
        break;

        case 3:              // tx ch3
        {
#ifdef BSP_AUDPRC_TX3_DMA
            if (haudprc->buf[HAL_AUDPRC_TX_CH3])
            {
                free(haudprc->buf[HAL_AUDPRC_TX_CH3]);
            }

            haudprc->buf[HAL_AUDPRC_TX_CH3] = calloc(1, haudprc->bufTxSize);
            RT_ASSERT(haudprc->buf[HAL_AUDPRC_TX_CH3]);
            if (haudprc->buf[HAL_AUDPRC_TX_CH3] == NULL)
                return RT_ERROR_MEMFAULT;

            audprc->queue_buf[HAL_AUDPRC_TX_CH3] = haudprc->buf[HAL_AUDPRC_TX_CH3];
            cfg.dma_mask = 0;
            cfg.en = 1;
            cfg.format = caps->udata.config.samplefmt == 16 ? 0 : 1;
            if (cfg.format == 0) // only 16 bit support stereo
                cfg.mode = caps->udata.config.channels == 1 ? 0 : 1;
            else
                cfg.mode = 0;
            g_eq_samplerate = caps->udata.config.samplerate;
            HAL_AUDPRC_Config_TChanel(haudprc, 3, &cfg);
            audprc->tx_instanc = HAL_AUDPRC_TX_CH3;
#endif
        }
        break;

        default:
        {
            result = -RT_ERROR;
        }
        break;
        }
    }
    break;
    case AUDIO_TYPE_MIXER:
    {
        if (caps->sub_type == 1)
        {
            haudprc->Init.dac_cfg.mixrsrc1 = 3; // fixed tx ch3
            haudprc->Init.dac_cfg.mixrsrc0 = 1; // fixed tx ch1
            haudprc->Init.dac_cfg.mixlsrc1 = 2; // fixed tx ch2
            haudprc->Init.dac_cfg.mixlsrc0 = 0; // fixed tx ch0
        }
        else if (caps->sub_type == 2)
        {
            haudprc->Init.dac_cfg.mixrsrc1 = 5; // mute
            haudprc->Init.dac_cfg.mixrsrc0 = 1; // fixed tx ch1
            haudprc->Init.dac_cfg.mixlsrc1 = 5; // mute
            haudprc->Init.dac_cfg.mixlsrc0 = 0; // fixed tx ch0
        }
        else
        {
            haudprc->Init.dac_cfg.mixrsrc1 = (caps->udata.value >> 12) & 0xF;
            haudprc->Init.dac_cfg.mixrsrc0 = (caps->udata.value >> 8) & 0xF;
            haudprc->Init.dac_cfg.mixlsrc1 = (caps->udata.value >> 4) & 0xF;
            haudprc->Init.dac_cfg.mixlsrc0 = caps->udata.value & 0xF;
        }
        HAL_AUDPRC_Config_DACPath(haudprc, &(haudprc->Init.dac_cfg));
        break;
    }
    case AUDIO_TYPE_SELECTOR:       // mux
    {
        if (caps->sub_type == 1)
        {
            haudprc->Init.dac_cfg.muxrsrc1 = 3; // fixed rx right
            haudprc->Init.dac_cfg.muxrsrc0 = 1; // fixed tx right
            haudprc->Init.dac_cfg.muxlsrc1 = 2; // fixed rx left
            haudprc->Init.dac_cfg.muxlsrc0 = 0; // fixed tx left
        }
        else if (caps->sub_type == 2)
        {
            haudprc->Init.dac_cfg.muxrsrc1 = 5; // fixed mute
            haudprc->Init.dac_cfg.muxrsrc0 = 1; // fixed tx right
            haudprc->Init.dac_cfg.muxlsrc1 = 5; // fixed mute
            haudprc->Init.dac_cfg.muxlsrc0 = 0; // fixed tx left
        }
        else
        {
            haudprc->Init.dac_cfg.muxrsrc1 = (caps->udata.value >> 12) & 0xF;
            haudprc->Init.dac_cfg.muxrsrc0 = (caps->udata.value >> 8) & 0xF;
            haudprc->Init.dac_cfg.muxlsrc1 = (caps->udata.value >> 4) & 0xF;
            haudprc->Init.dac_cfg.muxlsrc0 = caps->udata.value & 0xF;
        }
        HAL_AUDPRC_Config_DACPath(haudprc, &(haudprc->Init.dac_cfg));
        break;
    }
    case AUDIO_TYPE_LOOPBACK:
    {
        haudprc->Init.adc_cfg.rx2tx_loopback = 1;
        HAL_AUDPRC_Config_ADCPath(haudprc, &(haudprc->Init.adc_cfg));
        break;
    }
    default:
        result = -RT_ERROR;
        break;
    }

    return result;

}

static rt_err_t bf0_audio_init(struct rt_audio_device *audio)
{
    struct bf0_audio_prc *audprc = (struct bf0_audio_prc *) audio;
    AUDPRC_HandleTypeDef *haudprc = (AUDPRC_HandleTypeDef *) & (audprc->audprc);

    HAL_RCC_EnableModule(RCC_MOD_AUDPRC);

    // init dma handle and request, other parameters configure in HAL driver
#ifdef BSP_AUDPRC_TX0_DMA
    haudprc->hdma[HAL_AUDPRC_TX_CH0] = malloc(sizeof(DMA_HandleTypeDef));

    if (NULL == haudprc->hdma[HAL_AUDPRC_TX_CH0])
    {
        return RT_ENOMEM;
    }
    memset(haudprc->hdma[HAL_AUDPRC_TX_CH0], 0, sizeof(DMA_HandleTypeDef));

    haudprc->hdma[HAL_AUDPRC_TX_CH0]->Instance                 = AUDPRC_TX0_DMA_INSTANCE;
    haudprc->hdma[HAL_AUDPRC_TX_CH0]->Init.Request             = AUDPRC_TX0_DMA_REQUEST;
#endif

#ifdef BSP_AUDPRC_TX1_DMA
    haudprc->hdma[HAL_AUDPRC_TX_CH1] = malloc(sizeof(DMA_HandleTypeDef));

    if (NULL == haudprc->hdma[HAL_AUDPRC_TX_CH1])
    {
        return RT_ENOMEM;
    }
    memset(haudprc->hdma[HAL_AUDPRC_TX_CH1], 0, sizeof(DMA_HandleTypeDef));

    haudprc->hdma[HAL_AUDPRC_TX_CH1]->Instance                 = AUDPRC_TX1_DMA_INSTANCE;
    haudprc->hdma[HAL_AUDPRC_TX_CH1]->Init.Request             = AUDPRC_TX1_DMA_REQUEST;
#endif

#ifdef BSP_AUDPRC_TX2_DMA
    haudprc->hdma[HAL_AUDPRC_TX_CH2] = malloc(sizeof(DMA_HandleTypeDef));

    if (NULL == haudprc->hdma[HAL_AUDPRC_TX_CH2])
    {
        return RT_ENOMEM;
    }
    memset(haudprc->hdma[HAL_AUDPRC_TX_CH2], 0, sizeof(DMA_HandleTypeDef));

    haudprc->hdma[HAL_AUDPRC_TX_CH2]->Instance                 = AUDPRC_TX2_DMA_INSTANCE;
    haudprc->hdma[HAL_AUDPRC_TX_CH2]->Init.Request             = AUDPRC_TX2_DMA_REQUEST;
#endif

#ifdef BSP_AUDPRC_TX3_DMA
    haudprc->hdma[HAL_AUDPRC_TX_CH3] = malloc(sizeof(DMA_HandleTypeDef));

    if (NULL == haudprc->hdma[HAL_AUDPRC_TX_CH3])
    {
        return RT_ENOMEM;
    }
    memset(haudprc->hdma[HAL_AUDPRC_TX_CH3], 0, sizeof(DMA_HandleTypeDef));

    haudprc->hdma[HAL_AUDPRC_TX_CH3]->Instance                 = AUDPRC_TX3_DMA_INSTANCE;
    haudprc->hdma[HAL_AUDPRC_TX_CH3]->Init.Request             = AUDPRC_TX3_DMA_REQUEST;
#endif

#ifdef BSP_AUDPRC_RX0_DMA
    haudprc->hdma[HAL_AUDPRC_RX_CH0] = malloc(sizeof(DMA_HandleTypeDef));

    if (NULL == haudprc->hdma[HAL_AUDPRC_RX_CH0])
    {
        return RT_ENOMEM;
    }
    memset(haudprc->hdma[HAL_AUDPRC_RX_CH0], 0, sizeof(DMA_HandleTypeDef));

    haudprc->hdma[HAL_AUDPRC_RX_CH0]->Instance                 = AUDPRC_RX0_DMA_INSTANCE;
    haudprc->hdma[HAL_AUDPRC_RX_CH0]->Init.Request             = AUDPRC_RX0_DMA_REQUEST;
#endif

#ifdef BSP_AUDPRC_RX1_DMA
    haudprc->hdma[HAL_AUDPRC_RX_CH1] = malloc(sizeof(DMA_HandleTypeDef));

    if (NULL == haudprc->hdma[HAL_AUDPRC_RX_CH1])
    {
        return RT_ENOMEM;
    }
    memset(haudprc->hdma[HAL_AUDPRC_RX_CH1], 0, sizeof(DMA_HandleTypeDef));

    haudprc->hdma[HAL_AUDPRC_RX_CH1]->Instance                 = AUDPRC_RX1_DMA_INSTANCE;
    haudprc->hdma[HAL_AUDPRC_RX_CH1]->Init.Request             = AUDPRC_RX1_DMA_REQUEST;
#endif

#ifdef BSP_AUDPRC_TX_OUT0_DMA
    haudprc->hdma[HAL_AUDPRC_TX_OUT_CH0] = malloc(sizeof(DMA_HandleTypeDef));

    if (NULL == haudprc->hdma[HAL_AUDPRC_TX_OUT_CH0])
    {
        return RT_ENOMEM;
    }
    memset(haudprc->hdma[HAL_AUDPRC_TX_OUT_CH0], 0, sizeof(DMA_HandleTypeDef));

    haudprc->hdma[HAL_AUDPRC_TX_OUT_CH0]->Instance                 = AUDPRC_TX_OUT0_DMA_INSTANCE;
    haudprc->hdma[HAL_AUDPRC_TX_OUT_CH0]->Init.Request             = AUDPRC_TX_OUT0_DMA_REQUEST;
#endif

#ifdef BSP_AUDPRC_TX_OUT1_DMA
    haudprc->hdma[HAL_AUDPRC_TX_OUT_CH1] = malloc(sizeof(DMA_HandleTypeDef));

    if (NULL == haudprc->hdma[HAL_AUDPRC_TX_OUT_CH1])
    {
        return RT_ENOMEM;
    }
    memset(haudprc->hdma[HAL_AUDPRC_TX_OUT_CH1], 0, sizeof(DMA_HandleTypeDef));

    haudprc->hdma[HAL_AUDPRC_TX_OUT_CH1]->Instance                 = AUDPRC_TX_OUT1_DMA_INSTANCE;
    haudprc->hdma[HAL_AUDPRC_TX_OUT_CH1]->Init.Request             = AUDPRC_TX_OUT1_DMA_REQUEST;
#endif

    // set clock
#ifndef ASIC
    haudprc->Init.clk_div = 1;
#else
    haudprc->Init.clk_div = 3;
#endif
    haudprc->Init.adc_div = 1;
    haudprc->Init.dac_div = 1;

    haudprc->bufTxSize = CFG_AUDIO_RECORD_PIPE_SIZE * 2;
    haudprc->bufRxSize = CFG_AUDIO_RECORD_PIPE_SIZE * 2;

    audprc->slot_valid = 0;
    int i;
    for (i = 0; i < HAL_AUDPRC_INSTANC_CNT; i++)
    {
        audprc->queue_buf[i] = NULL;
    }
    audprc->rbf_tx_instanc = malloc(sizeof(struct rt_ringbuffer));
    audprc->rbf_rx_instanc = malloc(sizeof(struct rt_ringbuffer));
    rt_ringbuffer_init(audprc->rbf_tx_instanc, audprc->rbf_tx_pool, AUDPRC_DMA_RBF_NUM);
    rt_ringbuffer_init(audprc->rbf_rx_instanc, audprc->rbf_rx_pool, AUDPRC_DMA_RBF_NUM);

    bf0_adc_dac_path_cfg_init(haudprc);
#if 0
    // adc path configure
    haudprc->Init.adc_cfg.src_hbf3_mode = 0;
    haudprc->Init.adc_cfg.src_hbf3_en = 0;
    haudprc->Init.adc_cfg.src_hbf2_mode = 0;
    haudprc->Init.adc_cfg.src_hbf2_en = 0;
    haudprc->Init.adc_cfg.src_hbf1_mode = 0;
    haudprc->Init.adc_cfg.src_hbf1_en = 0;
    haudprc->Init.adc_cfg.src_ch_en = 0;
    haudprc->Init.adc_cfg.rx2tx_loopback = 0;
    haudprc->Init.adc_cfg.data_swap = 0;
    haudprc->Init.adc_cfg.src_sel = 0;
    haudprc->Init.adc_cfg.vol_l = 36;
    haudprc->Init.adc_cfg.vol_r = 36;
    haudprc->Init.adc_cfg.src_sinc_en = 0;
    haudprc->Init.adc_cfg.sinc_ratio = 0;

    // dac path configure
    haudprc->Init.dac_cfg.dst_sel = 0;
    haudprc->Init.dac_cfg.mixrsrc1 = 5;
    haudprc->Init.dac_cfg.mixrsrc0 = 1;
    haudprc->Init.dac_cfg.mixlsrc1 = 5;
    haudprc->Init.dac_cfg.mixlsrc0 = 0;
    haudprc->Init.dac_cfg.vol_r = 36;
    haudprc->Init.dac_cfg.vol_l = 36;
    haudprc->Init.dac_cfg.src_hbf3_mode = 0;
    haudprc->Init.dac_cfg.src_hbf3_en = 0;
    haudprc->Init.dac_cfg.src_hbf2_mode = 0;
    haudprc->Init.dac_cfg.src_hbf2_en = 0;
    haudprc->Init.dac_cfg.src_hbf1_mode = 0;
    haudprc->Init.dac_cfg.src_hbf1_en = 0;
    haudprc->Init.dac_cfg.src_ch_en = 0;
    haudprc->Init.dac_cfg.eq_clr = 0;
    haudprc->Init.dac_cfg.eq_stage = 1;
    haudprc->Init.dac_cfg.eq_ch_en = 0;
    haudprc->Init.dac_cfg.muxrsrc1 = 5;
    haudprc->Init.dac_cfg.muxrsrc0 = 1;
    haudprc->Init.dac_cfg.muxlsrc1 = 5;
    haudprc->Init.dac_cfg.muxlsrc0 = 0;
    haudprc->Init.dac_cfg.src_sinc_en = 0;
    haudprc->Init.dac_cfg.sinc_ratio = 0;
#endif
    int res = HAL_AUDPRC_Init(haudprc);

    LOG_I("init 00 ADC_PATH_CFG0 0x%x\n",  haudprc->Instance->ADC_PATH_CFG0);

    LOG_I("HAL_AUDPRC_Init res %d\n", res);

    return RT_EOK;
}

static rt_err_t bf0_audio_shutdown(struct rt_audio_device *audio)
{
    struct bf0_audio_prc *audprc = (struct bf0_audio_prc *) audio;
    AUDPRC_HandleTypeDef *haudprc = (AUDPRC_HandleTypeDef *) & (audprc->audprc);

#ifdef BSP_AUDPRC_TX0_DMA
    if (haudprc->buf[HAL_AUDPRC_TX_CH0] != NULL)
    {
        free(haudprc->buf[HAL_AUDPRC_TX_CH0]);
        haudprc->buf[HAL_AUDPRC_TX_CH0] = NULL;
    }
#endif

#ifdef BSP_AUDPRC_TX1_DMA
    if (haudprc->buf[HAL_AUDPRC_TX_CH1] != NULL)
    {
        free(haudprc->buf[HAL_AUDPRC_TX_CH1]);
        haudprc->buf[HAL_AUDPRC_TX_CH1] = NULL;
    }
#endif
#ifdef BSP_AUDPRC_TX2_DMA
    if (haudprc->buf[HAL_AUDPRC_TX_CH2] != NULL)
    {
        free(haudprc->buf[HAL_AUDPRC_TX_CH2]);
        haudprc->buf[HAL_AUDPRC_TX_CH2] = NULL;
    }
#endif

#ifdef BSP_AUDPRC_TX3_DMA
    if (haudprc->buf[HAL_AUDPRC_TX_CH3] != NULL)
    {
        free(haudprc->buf[HAL_AUDPRC_TX_CH3]);
        haudprc->buf[HAL_AUDPRC_TX_CH3] = NULL;
    }
#endif

#ifdef BSP_AUDPRC_RX0_DMA
    if (haudprc->buf[HAL_AUDPRC_RX_CH0] != NULL)
    {
        free(haudprc->buf[HAL_AUDPRC_RX_CH0]);
        haudprc->buf[HAL_AUDPRC_RX_CH0] = NULL;
    }
#endif

#ifdef BSP_AUDPRC_RX1_DMA
    if (haudprc->buf[HAL_AUDPRC_RX_CH1] != NULL)
    {
        free(haudprc->buf[HAL_AUDPRC_RX_CH1]);
        haudprc->buf[HAL_AUDPRC_RX_CH1] = NULL;
    }
#endif

#ifdef BSP_AUDPRC_TX_OUT0_DMA
    if (haudprc->buf[HAL_AUDPRC_TX_OUT_CH0] != NULL)
    {
        free(haudprc->buf[HAL_AUDPRC_TX_OUT_CH0]);
        haudprc->buf[HAL_AUDPRC_TX_OUT_CH0] = NULL;
    }
#endif

#ifdef BSP_AUDPRC_TX_OUT1_DMA
    if (haudprc->buf[HAL_AUDPRC_TX_OUT_CH1] != NULL)
    {
        free(haudprc->buf[HAL_AUDPRC_TX_OUT_CH1]);
        haudprc->buf[HAL_AUDPRC_TX_OUT_CH1] = NULL;
    }
#endif

    return RT_EOK;
}

/**
  * @brief  Start audio device for recording/playback.
  * @param[in]  audio: audio device handle.
  * @param[in]  stream: lower 8 bit for playback/record, high 8 bit for slot number.
  * @retval RT_EOK if success, otherwise -RT_ERROR
  */
extern  void  config_eq_para(AUDPRC_HandleTypeDef *haprc);
static rt_err_t bf0_audio_start(struct rt_audio_device *audio, int stream)
{
    struct bf0_audio_prc *audprc = (struct bf0_audio_prc *) audio;
    AUDPRC_HandleTypeDef *haudprc = (AUDPRC_HandleTypeDef *) & (audprc->audprc);
    HAL_StatusTypeDef res = HAL_OK;
    uint8_t rx_dma_num = 0;
    uint8_t tx_dma_num = 0;

    if (((stream & 0xff) == AUDIO_STREAM_REPLAY) || ((stream & 0xff) == AUDIO_STREAM_RXandTX))
    {
        if (!audprc->eq_opened)
        {
            config_eq_para(haudprc);
            audprc->eq_opened = 1;
        }
    }

    if (((stream & 0xff) == AUDIO_STREAM_RECORD) || ((stream & 0xff) == AUDIO_STREAM_RXandTX))
    {
        g_rx_stop = 0;
#ifdef AUDPRC_RX0_DMA_INSTANCE
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDPRC_RX_CH0) << 8)))
        {
            HAL_AUDPRC_Config_RChanel(haudprc, 0, &haudprc->cfg);
            res = HAL_AUDPRC_Receive_DMA(haudprc, haudprc->buf[HAL_AUDPRC_RX_CH0], haudprc->bufRxSize, HAL_AUDPRC_RX_CH0);
#ifndef PKG_USING_MIC_BIAS_AS_GPIO_ONLY
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_EnableIRQ(AUDPRC_RX0_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
#endif
            rx_dma_num++;
            audprc->rx_instanc = HAL_AUDPRC_RX_CH0;
            haudprc->channel_ref |= (1 << HAL_AUDPRC_RX_CH0);
        }
#endif
#ifdef AUDPRC_RX1_DMA_INSTANCE
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDPRC_RX_CH1) << 8)))
        {
            HAL_AUDPRC_Config_RChanel(haudprc, 1, &haudprc->cfg1);
            res = HAL_AUDPRC_Receive_DMA(haudprc, haudprc->buf[HAL_AUDPRC_RX_CH1], haudprc->bufRxSize, HAL_AUDPRC_RX_CH1);
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_EnableIRQ(AUDPRC_RX1_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            rx_dma_num++;
            audprc->rx_instanc = HAL_AUDPRC_RX_CH1;
            haudprc->channel_ref |= (1 << HAL_AUDPRC_RX_CH1);
        }
#endif
#ifdef AUDPRC_TX_OUT0_DMA_INSTANCE
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDPRC_TX_OUT_CH0) << 8)))
        {
            res = HAL_AUDPRC_Receive_DMA(haudprc, haudprc->buf[HAL_AUDPRC_TX_OUT_CH0], haudprc->bufRxSize, HAL_AUDPRC_TX_OUT_CH0);
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_EnableIRQ(AUDPRC_TX_OUT0_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            rx_dma_num++;
            audprc->rx_instanc = HAL_AUDPRC_TX_OUT_CH0;
            haudprc->channel_ref |= (1 << HAL_AUDPRC_TX_OUT_CH0);
        }
#endif
#ifdef AUDPRC_TX_OUT1_DMA_INSTANCE
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDPRC_TX_OUT_CH1) << 8)))
        {
            res = HAL_AUDPRC_Receive_DMA(haudprc, haudprc->buf[HAL_AUDPRC_TX_OUT_CH1], haudprc->bufRxSize, HAL_AUDPRC_TX_OUT_CH1);
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_EnableIRQ(AUDPRC_TX_OUT1_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            rx_dma_num++;
            audprc->rx_instanc = HAL_AUDPRC_TX_OUT_CH1;
            haudprc->channel_ref |= (1 << HAL_AUDPRC_TX_OUT_CH1);
        }
#endif

    }

    if (((stream & 0xff) == AUDIO_STREAM_REPLAY) || ((stream & 0xff) == AUDIO_STREAM_RXandTX))
    {
#ifdef AUDPRC_TX0_DMA_INSTANCE
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDPRC_TX_CH0) << 8)))
        {
            LOG_I("-----tx dma size = %d", haudprc->bufTxSize / 2);
            res = HAL_AUDPRC_Transmit_DMA(haudprc, haudprc->buf[HAL_AUDPRC_TX_CH0], haudprc->bufTxSize, HAL_AUDPRC_TX_CH0);
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_EnableIRQ(AUDPRC_TX0_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            tx_dma_num++;
            audprc->tx_instanc = HAL_AUDPRC_TX_CH0;
            haudprc->channel_ref |= (1 << HAL_AUDPRC_TX_CH0);
        }
#endif
#ifdef AUDPRC_TX1_DMA_INSTANCE
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDPRC_TX_CH1) << 8)))
        {
            res = HAL_AUDPRC_Transmit_DMA(haudprc, haudprc->buf[HAL_AUDPRC_TX_CH1], haudprc->bufTxSize, HAL_AUDPRC_TX_CH1);
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_EnableIRQ(AUDPRC_TX1_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            tx_dma_num++;
            audprc->tx_instanc = HAL_AUDPRC_TX_CH1;
            haudprc->channel_ref |= (1 << HAL_AUDPRC_TX_CH1);
        }
#endif
#ifdef AUDPRC_TX2_DMA_INSTANCE
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDPRC_TX_CH2) << 8)))
        {
            res = HAL_AUDPRC_Transmit_DMA(haudprc, haudprc->buf[HAL_AUDPRC_TX_CH2], haudprc->bufTxSize, HAL_AUDPRC_TX_CH2);
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_EnableIRQ(AUDPRC_TX2_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            tx_dma_num++;
            audprc->tx_instanc = HAL_AUDPRC_TX_CH2;
            haudprc->channel_ref |= (1 << HAL_AUDPRC_TX_CH2);
        }
#endif
#ifdef AUDPRC_TX3_DMA_INSTANCE
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDPRC_TX_CH3) << 8)))
        {
            res = HAL_AUDPRC_Transmit_DMA(haudprc, haudprc->buf[HAL_AUDPRC_TX_CH3], haudprc->bufTxSize, HAL_AUDPRC_TX_CH3);
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_EnableIRQ(AUDPRC_TX3_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            tx_dma_num++;
            audprc->tx_instanc = HAL_AUDPRC_TX_CH3;
            haudprc->channel_ref |= (1 << HAL_AUDPRC_TX_CH3);
        }
#endif

    }

    if (tx_dma_num > 1)
    {
        audprc->tx_rbf_enable = true;
        rt_ringbuffer_reset(audprc->rbf_tx_instanc);
    }
    else
    {
        audprc->tx_rbf_enable = false;
    }

    if (rx_dma_num > 1)
    {
        audprc->rx_rbf_enable = true;
        rt_ringbuffer_reset(audprc->rbf_rx_instanc);
    }
    else
    {
        audprc->rx_rbf_enable = false;
    }

    /* enable AUDPRC at last*/
    __HAL_AUDPRC_ENABLE(haudprc);

    return RT_EOK;
}

/**
  * @brief  Stop audio device for recording/playback.
  * @param[in]  audio: audio device handle.
  * @param[in]  stream: lower 8 bit for playback/record, high 8 bit for slot number.
  * @retval RT_EOK if success, otherwise -RT_ERROR
  */
static rt_err_t bf0_audio_stop(struct rt_audio_device *audio, int stream)
{
    struct bf0_audio_prc *audprc = (struct bf0_audio_prc *) audio;
    AUDPRC_HandleTypeDef *haudprc = (AUDPRC_HandleTypeDef *) & (audprc->audprc);
    rt_err_t ret = RT_EOK;
    if (((stream & 0xff) == AUDIO_STREAM_RECORD) || ((stream & 0xff) == AUDIO_STREAM_RXandTX))
    {
        g_rx_stop = 1;
    }
    if ((stream == AUDIO_STREAM_REPLAY || stream == AUDIO_STREAM_RECORD) && !haudprc->channel_ref)
    {
        return RT_EOK;
    }

    LOG_I("bf0_audio_stop 0x%x\n", stream);

    if (((stream & 0xff) == AUDIO_STREAM_REPLAY) || ((stream & 0xff) == AUDIO_STREAM_RXandTX))
    {
        if (audprc->eq_opened)
        {
            audprc->eq_opened = 0;
        }
    }
    if (((stream & 0xff) == AUDIO_STREAM_RECORD) || ((stream & 0xff) == AUDIO_STREAM_RXandTX))  // rx
    {
#ifdef AUDPRC_RX0_DMA_INSTANCE
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDPRC_RX_CH0) << 8)))
        {
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_DisableIRQ(AUDPRC_RX0_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            ret = HAL_AUDPRC_DMAStop(haudprc, HAL_AUDPRC_RX_CH0);
            haudprc->channel_ref &= ~(1 << HAL_AUDPRC_RX_CH0);
        }
#endif
#ifdef AUDPRC_RX1_DMA_INSTANCE
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDPRC_RX_CH1) << 8)))
        {
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_DisableIRQ(AUDPRC_RX1_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            ret = HAL_AUDPRC_DMAStop(haudprc, HAL_AUDPRC_RX_CH1);
            haudprc->channel_ref &= ~(1 << HAL_AUDPRC_RX_CH1);
        }
#endif
#ifdef AUDPRC_TX_OUT0_DMA_INSTANCE
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDPRC_TX_OUT_CH0) << 8)))
        {
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_DisableIRQ(AUDPRC_TX_OUT0_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            ret = HAL_AUDPRC_DMAStop(haudprc, HAL_AUDPRC_TX_OUT_CH0);
            haudprc->channel_ref &= ~(1 << HAL_AUDPRC_TX_OUT_CH0);
        }
#endif
#ifdef AUDPRC_TX_OUT1_DMA_INSTANCE
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDPRC_TX_OUT_CH1) << 8)))
        {
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_DisableIRQ(AUDPRC_TX_OUT1_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            ret = HAL_AUDPRC_DMAStop(haudprc, HAL_AUDPRC_TX_OUT_CH1);
            haudprc->channel_ref &= ~(1 << HAL_AUDPRC_TX_OUT_CH1);
        }
#endif

    }

    if (((stream & 0xff) == AUDIO_STREAM_REPLAY) || ((stream & 0xff) == AUDIO_STREAM_RXandTX)) //tx
    {
#ifdef AUDPRC_TX0_DMA_INSTANCE
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDPRC_TX_CH0) << 8)))
        {
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_DisableIRQ(AUDPRC_TX0_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            ret = HAL_AUDPRC_DMAStop(haudprc, HAL_AUDPRC_TX_CH0);
            haudprc->channel_ref &= ~(1 << HAL_AUDPRC_TX_CH0);
            if (haudprc->buf[HAL_AUDPRC_TX_CH0])
            {
                free(haudprc->buf[HAL_AUDPRC_TX_CH0]);
                haudprc->buf[HAL_AUDPRC_TX_CH0] = NULL;
            }
        }
#endif
#ifdef AUDPRC_TX1_DMA_INSTANCE
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDPRC_TX_CH1) << 8)))
        {
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_DisableIRQ(AUDPRC_TX1_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            ret = HAL_AUDPRC_DMAStop(haudprc, HAL_AUDPRC_TX_CH1);
            haudprc->channel_ref &= ~(1 << HAL_AUDPRC_TX_CH1);
            if (haudprc->buf[HAL_AUDPRC_TX_CH1])
            {
                free(haudprc->buf[HAL_AUDPRC_TX_CH1]);
                haudprc->buf[HAL_AUDPRC_TX_CH1] = NULL;
            }
        }
#endif
#ifdef AUDPRC_TX2_DMA_INSTANCE
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDPRC_TX_CH2) << 8)))
        {
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_DisableIRQ(AUDPRC_TX2_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            ret = HAL_AUDPRC_DMAStop(haudprc, HAL_AUDPRC_TX_CH2);
            haudprc->channel_ref &= ~(1 << HAL_AUDPRC_TX_CH2);
            if (haudprc->buf[HAL_AUDPRC_TX_CH2])
            {
                free(haudprc->buf[HAL_AUDPRC_TX_CH2]);
                haudprc->buf[HAL_AUDPRC_TX_CH2] = NULL;
            }
        }
#endif
#ifdef AUDPRC_TX3_DMA_INSTANCE
        if (0 != ((stream & 0xff00) & ((1 << HAL_AUDPRC_TX_CH3) << 8)))
        {
#ifndef DMA_SUPPORT_DYN_CHANNEL_ALLOC
            HAL_NVIC_DisableIRQ(AUDPRC_TX3_DMA_IRQ);
#endif /* !DMA_SUPPORT_DYN_CHANNEL_ALLOC */
            ret = HAL_AUDPRC_DMAStop(haudprc, HAL_AUDPRC_TX_CH3);
            haudprc->channel_ref &= ~(1 << HAL_AUDPRC_TX_CH3);
            if (haudprc->buf[HAL_AUDPRC_TX_CH3])
            {
                free(haudprc->buf[HAL_AUDPRC_TX_CH3]);
                haudprc->buf[HAL_AUDPRC_TX_CH3] = NULL;
            }
        }
#endif

    }

    uint16_t dac_mask, adc_mask;
    dac_mask = (1 << HAL_AUDPRC_TX_CH3) | (1 << HAL_AUDPRC_TX_CH2) | (1 << HAL_AUDPRC_TX_CH1) | (1 << HAL_AUDPRC_TX_CH0)
               | (1 << HAL_AUDPRC_TX_OUT_CH0) | (1 << HAL_AUDPRC_TX_OUT_CH1);
    adc_mask = (1 << HAL_AUDPRC_RX_CH0) | (1 << HAL_AUDPRC_RX_CH1);
    if (haudprc->channel_ref == 0)
    {
        LOG_I("audprc close adc/dac");
        __HAL_AUDPRC_DISABLE(haudprc);
        HAL_AUDPRC_Clear_All_Channel(haudprc);
        haudprc->Init.adc_cfg.rx2tx_loopback = 0;
        __HAL_AUDPRC_SRESET_START(haudprc);
        __HAL_AUDPRC_SRESET_STOP(haudprc);
    }
    else if ((haudprc->channel_ref & dac_mask) == 0)
    {
        LOG_I("audprc close dac");
        HAL_AUDPRC_Clear_Dac_Channel(haudprc);
        haudprc->Init.adc_cfg.rx2tx_loopback = 0;
    }
    else if ((haudprc->channel_ref & adc_mask) == 0)
    {
        LOG_I("audprc close adc");
        HAL_AUDPRC_Clear_Adc_Channel(haudprc);
        haudprc->Init.adc_cfg.rx2tx_loopback = 0;
    }
    else
    {
        LOG_I("audprc channel_ref=%d", haudprc->channel_ref);
    }
    for (int i = 0; i < HAL_AUDPRC_INSTANC_CNT; i++)
    {
        haudprc->State[i] = HAL_AUDPRC_STATE_READY;
    }

    LOG_I("bf0_audio_stop 0x%x done\n", stream);

    return ret;
}

static rt_err_t bf0_audio_suspend(struct rt_audio_device *audio, int stream)
{
    rt_err_t ret = RT_ERROR;

    return ret;
}

static rt_err_t bf0_audio_resume(struct rt_audio_device *audio, int stream)
{
    rt_err_t ret = RT_ERROR;

    return ret;
}


static rt_err_t bf0_audio_control(struct rt_audio_device *audio, int cmd, void *args)
{
    struct bf0_audio_prc *audprc = (struct bf0_audio_prc *) audio;
    AUDPRC_HandleTypeDef *haudprc = (AUDPRC_HandleTypeDef *) & (audprc->audprc);
    rt_err_t result = RT_EOK;

    switch (cmd)
    {
    case AUDIO_CTL_SET_TX_DMA_SIZE:
    {
        uint32_t dma_size = (uint32_t)args;
        haudprc->bufTxSize = dma_size * 2;
        break;
    }
    case AUDIO_CTL_SETOUTPUT:
    {
        uint32_t intf = (uint32_t)args;
        haudprc->dest_sel = intf;
        switch (intf)
        {
        case AUDPRC_TX_TO_CODEC:
            __HAL_AUDPRC_DAC_DST_CODEC(haudprc);
            break;
        case AUDPRC_TX_TO_I2S:
            __HAL_AUDPRC_DAC_DST_I2S(haudprc);
            break;
        case AUDPRC_TX_TO_MEM:
            __HAL_AUDPRC_DAC_DST_MEM(haudprc);
            break;
        default:
            LOG_E("AUDPRC DAC dest invalid %d\n", intf);
            break;
        }
        LOG_D("AUDPRC set dest %d\n", intf);
        break;
    }
    case AUDIO_CTL_SETINPUT:
    {
        uint32_t intf = (uint32_t)args;
        switch (intf)
        {
        case AUDPRC_RX_FROM_CODEC:
            __HAL_AUDPRC_ADC_SRC_CODEC(haudprc);
            break;
        case AUDPRC_RX_FROM_I2S:
            __HAL_AUDPRC_ADC_SRC_I2S(haudprc);
            break;
        default:
            LOG_E("AUDPRC ADC source invalid %d\n", intf);
            break;
        }
        LOG_D("AUDPRC set source %d\n", intf);
        break;
    }
    case AUDIO_CTL_OUTPUTSRC:
    {
        struct rt_audio_sr_convert *cfg = (struct rt_audio_sr_convert *)args;
        result = bf0_audprc_src(audio, cfg->source_sr, cfg->dest_sr, 0);
        break;
    }
    case AUDIO_CTL_INPUTSRC:
    {
        struct rt_audio_sr_convert *cfg = (struct rt_audio_sr_convert *)args;
        result = bf0_audprc_src(audio, cfg->source_sr, cfg->dest_sr, 1);
        break;
    }
    case RT_DEVICE_CTRL_SUSPEND:
    {
        struct bf0_audio_prc *audprc = (struct bf0_audio_prc *) audio;
        AUDPRC_HandleTypeDef *haudprc = (AUDPRC_HandleTypeDef *) & (audprc->audprc);
        HAL_AUDPRC_DeInit(haudprc);
        set_pll_state(0);
        break;
    }
    case RT_DEVICE_CTRL_RESUME:
    {
        struct bf0_audio_prc *audprc = (struct bf0_audio_prc *) audio;
        AUDPRC_HandleTypeDef *haudprc = (AUDPRC_HandleTypeDef *) & (audprc->audprc);
        HAL_RCC_EnableModule(RCC_MOD_AUDPRC);
        HAL_AUDPRC_Init(haudprc);
        break;
    }
    default:
        result = -RT_ERROR;
        break;
    }

    return result;
}

static rt_size_t bf0_audio_trans(struct rt_audio_device *audio, const void *writeBuf, void *readBuf, rt_size_t size)
{
    struct bf0_audio_prc *audprc = (struct bf0_audio_prc *) audio;
    AUDPRC_HandleTypeDef *haudprc = (AUDPRC_HandleTypeDef *) & (audprc->audprc);
    uint8_t tx_ins = audprc->tx_instanc;


    if (writeBuf != NULL)
    {
        RT_ASSERT(tx_ins < HAL_AUDPRC_RX_CH0);
        RT_ASSERT(size <= haudprc->bufTxSize);
//#ifdef AUDPRC_TX0_DMA_INSTANCE
        if (audprc->queue_buf[tx_ins] != NULL) //HAL_AUDPRC_TX_CH0
        {
            memcpy(audprc->queue_buf[tx_ins], writeBuf, size);
            if (audprc->tx_rbf_enable)
            {
                audprc->tx_instanc = 0xFF;
            }
        }
        else
        {
            RT_ASSERT(false);
        }
//#endif
    }

    if (readBuf != NULL)
    {
#ifdef AUDPRC_RX0_DMA_INSTANCE
        //HAL_AUDPRC_Receive_DMA(haudprc, haudprc->buf[HAL_AUDPRC_RX_CH0], haudprc->bufSize, HAL_AUDPRC_RX_CH0);
        //HAL_NVIC_EnableIRQ(AUDPRC_RX0_DMA_IRQ);
#endif
    }

    return size;
}


static const struct rt_audio_ops       _g_audio_ops =
{
    .getcaps    = bf0_audio_getcaps,
    .configure  = bf0_audio_configure,

    .init       = bf0_audio_init,
    .shutdown   = bf0_audio_shutdown,
    .start      = bf0_audio_start,
    .stop       = bf0_audio_stop,
    .suspend    = bf0_audio_suspend,
    .resume     = bf0_audio_resume,
    .control    = bf0_audio_control,
    .transmit   = bf0_audio_trans,
};


/**
* @brief  Audio Process devices initialization
*/
int rt_bf0_audio_prc_init(void)
{
    int result;

    memset(&h_aud_prc, 0, sizeof(h_aud_prc));

    h_aud_prc.audprc.Instance = hwp_audprc;
    h_aud_prc.audio_device.ops = (struct rt_audio_ops *)&_g_audio_ops;

    result = rt_audio_register((struct rt_audio_device *)&h_aud_prc,
                               "audprc", RT_DEVICE_FLAG_RDWR, NULL);

    rt_device_init((rt_device_t)(&h_aud_prc.audio_device));
    return result;
}

INIT_DEVICE_EXPORT(rt_bf0_audio_prc_init);

void HAL_AUDPRC_TxCpltCallback(AUDPRC_HandleTypeDef *haprc, int cid)
{
    struct bf0_audio_prc *haudio = rt_container_of(haprc, struct bf0_audio_prc, audprc);
    struct rt_audio_device *audio = &(haudio->audio_device);

    if (audio != NULL)
    {
        if (AUDPRC_TX_TO_MEM == haprc->dest_sel)
        {
            haudio->queue_buf[cid] = (rt_uint8_t *)((uint32_t)(haprc->buf[cid]));
            rt_audio_tx_complete(audio, haudio->queue_buf[cid]);
        }
        else
        {
            haudio->queue_buf[cid] = (rt_uint8_t *)((uint32_t)(haprc->buf[cid]) + haprc->bufTxSize / 2);
            rt_audio_tx_complete(audio, haudio->queue_buf[cid]);
        }
        if (haudio->tx_rbf_enable)
        {
            rt_size_t putsize;
            uint8_t putdata;
            putdata = (1 << 4) | cid;
            putsize = rt_ringbuffer_put(haudio->rbf_tx_instanc, &putdata, 1);
            RT_ASSERT(putsize == 1);
        }
    }
}

void HAL_AUDPRC_TxHalfCpltCallback(AUDPRC_HandleTypeDef *haprc, int cid)
{
    struct bf0_audio_prc *haudio = rt_container_of(haprc, struct bf0_audio_prc, audprc);
    struct rt_audio_device *audio = &(haudio->audio_device);

    if (audio != NULL)
    {
        haudio->queue_buf[cid] = haprc->buf[cid];
        rt_audio_tx_complete(audio, haudio->queue_buf[cid]);
        if (haudio->tx_rbf_enable)
        {
            rt_size_t putsize;
            uint8_t putdata;
            putdata = cid;
            putsize = rt_ringbuffer_put(haudio->rbf_tx_instanc, &putdata, 1);
            RT_ASSERT(putsize == 1);
        }
    }
}

void HAL_AUDPRC_RxCpltCallback(AUDPRC_HandleTypeDef *haprc, int cid)
{
    struct bf0_audio_prc *haudio = rt_container_of(haprc, struct bf0_audio_prc, audprc);
    struct rt_audio_device *audio = &(haudio->audio_device);
    if (g_rx_stop)
    {
        return;
    }

    if (audio != NULL)
    {
        if (AUDPRC_TX_TO_MEM == haprc->dest_sel)
        {
            haudio->queue_buf[cid] = (rt_uint8_t *)((uint32_t)(haprc->buf[cid]));
            rt_audio_rx_done(audio, haudio->queue_buf[cid], haprc->bufRxSize);
        }
        else
        {
            haudio->queue_buf[cid] = (rt_uint8_t *)((uint32_t)(haprc->buf[cid]) + haprc->bufRxSize / 2);
            rt_audio_rx_done(audio, haudio->queue_buf[cid], haprc->bufRxSize / 2);
        }
        if (haudio->rx_rbf_enable)
        {
            rt_size_t putsize;
            uint8_t putdata;
            putdata = (1 << 4) | cid;
            putsize = rt_ringbuffer_put(haudio->rbf_rx_instanc, &putdata, 1);
            RT_ASSERT(putsize == 1);
        }
    }
}
void HAL_AUDPRC_RxHalfCpltCallback(AUDPRC_HandleTypeDef *haprc, int cid)
{
    struct bf0_audio_prc *haudio = rt_container_of(haprc, struct bf0_audio_prc, audprc);
    struct rt_audio_device *audio = &(haudio->audio_device);
    if (g_rx_stop)
    {
        return;
    }

    if (audio != NULL)
    {
        haudio->queue_buf[cid] = haprc->buf[cid];
        rt_audio_rx_done(audio, haudio->queue_buf[cid], haprc->bufRxSize / 2);
        if (haudio->rx_rbf_enable)
        {
            rt_size_t putsize;
            uint8_t putdata;
            putdata = cid;
            putsize = rt_ringbuffer_put(haudio->rbf_rx_instanc, &putdata, 1);
            RT_ASSERT(putsize == 1);
        }
    }
}

uint8_t bf0_audprc_get_tx_rbf_en()
{
    return h_aud_prc.tx_rbf_enable;
}

uint8_t bf0_audprc_get_tx_channel()
{

    if (h_aud_prc.tx_rbf_enable)
    {
        rt_size_t getsize;
        uint8_t getdata;
        if (rt_ringbuffer_data_len(h_aud_prc.rbf_tx_instanc))
        {
            getsize = rt_ringbuffer_get(h_aud_prc.rbf_tx_instanc, &getdata, 1);
            RT_ASSERT(getsize == 1);
            h_aud_prc.tx_instanc = getdata & 0xF;
        }
        else
        {
            h_aud_prc.tx_instanc = 0xFF;
        }
    }

    return h_aud_prc.tx_instanc;
}

void bf0_audprc_set_tx_channel(uint8_t chan)
{
    RT_ASSERT(chan < HAL_AUDPRC_RX_CH0);
    h_aud_prc.tx_instanc = chan;
}


uint8_t bf0_audprc_get_rx_channel()
{

    if (h_aud_prc.rx_rbf_enable)
    {
        rt_size_t getsize;
        uint8_t getdata;
        getsize = rt_ringbuffer_get(h_aud_prc.rbf_rx_instanc, &getdata, 1);
        RT_ASSERT(getsize == 1);
        h_aud_prc.rx_instanc = getdata & 0xF;
    }

    return h_aud_prc.rx_instanc;
}

void bf0_audprc_set_rx_channel(uint8_t chan)
{
    RT_ASSERT((chan >= HAL_AUDPRC_RX_CH0) && (chan < HAL_AUDPRC_INSTANC_CNT));
    h_aud_prc.rx_instanc = chan;
}

void bf0_audprc_device_write(rt_device_t dev, rt_off_t    pos, const void *buffer, rt_size_t   size) /*para is same to rt_device_write*/
//(struct rt_audio_device *audio, const void *writeBuf, void *readBuf, rt_size_t size)
{
    struct rt_audio_device *audio = (struct rt_audio_device *)dev;

    bf0_audio_trans(audio, buffer, NULL, size);
}

//tc_drv_audprc.c used
void bf0_audprc_dma_restart(uint16_t chann_used)
{
    uint32_t txentry, rxentry, bufsize;
    AUDPRC_HandleTypeDef *haudprc = &(h_aud_prc.audprc);
    /* Change DMA peripheral state */
    haudprc->hdma[chann_used]->State = HAL_DMA_STATE_BUSY;
    __HAL_DMA_ENABLE_IT(haudprc->hdma[chann_used], (DMA_IT_TC | DMA_IT_TE));
    bufsize = haudprc->bufTxSize;
    if (chann_used == HAL_AUDPRC_RX_CH0 || chann_used == HAL_AUDPRC_RX_CH1)
    {
        bufsize = haudprc->bufRxSize;
    }
    __HAL_DMA_SET_COUNTER(haudprc->hdma[chann_used], (bufsize >> 2));

    //LOG_I("audprc channel %d dma restart!\n", chann_used);
}

extern AUDPRC_HandleTypeDef *get_audprc_handle();
#if BSP_ENABLE_AUD_CODEC
extern AUDCODEC_HandleTypeDef *get_audcodec_handle();
#else
AUDCODEC_HandleTypeDef *get_audcodec_handle()
{
    RT_ASSERT(0);
    return NULL;
}
#endif

//void HAL_DBG_printf(const char *fmt, ...);
__WEAK int audio_server_is_speaker_working()
{
    return 0;
}


static uint8_t g_eq_enable = 1;

void  config_eq_para(AUDPRC_HandleTypeDef *haprc)
{
    uint32_t *pointer;
    uint32_t *eq_value = RT_NULL;
    uint8_t eq_state = 0;
#if !AUDPRC_EQ_ENABLE
    return;
#endif

    if (g_eq_samplerate == 44100)
    {
        eq_value = g_music_eqValue;
        eq_state = g_music_state;
        LOG_I("samplerate = %d, select music eq\n", g_eq_samplerate);
    }
    else if (g_eq_samplerate == 16000)
    {
        eq_value = g_voice_eqValue;
        eq_state = g_voice_state;
        LOG_I("samplerate = %d, select voice eq\n", g_eq_samplerate);
    }
    else
    {
        LOG_I("samplerate:%d unsupport eq\n", g_eq_samplerate);
        return;
    }

    if (g_eq_enable == 0 || eq_state == 0)
    {
        return;
    }

    MODIFY_REG(haprc->Instance->CFG, AUDPRC_CFG_AUTO_GATE_EN_Msk,
               MAKE_REG_VAL(0, AUDPRC_CFG_AUTO_GATE_EN_Msk, AUDPRC_CFG_AUTO_GATE_EN_Pos));

    MODIFY_REG(haprc->Instance->DAC_PATH_CFG1, AUDPRC_DAC_PATH_CFG1_EQ_CLR_Msk,
               MAKE_REG_VAL(1, AUDPRC_DAC_PATH_CFG1_EQ_CLR_Msk, AUDPRC_DAC_PATH_CFG1_EQ_CLR_Pos));
    while (!(haprc->Instance->DAC_PATH_CFG1 & AUDPRC_DAC_PATH_CFG1_EQ_CLR_DONE_Msk));
    MODIFY_REG(haprc->Instance->DAC_PATH_CFG1, AUDPRC_DAC_PATH_CFG1_EQ_CLR_Msk,
               MAKE_REG_VAL(0, AUDPRC_DAC_PATH_CFG1_EQ_CLR_Msk, AUDPRC_DAC_PATH_CFG1_EQ_CLR_Pos));
    MODIFY_REG(haprc->Instance->DAC_PATH_CFG1, AUDPRC_DAC_PATH_CFG1_EQ_STAGE_Msk,
               MAKE_REG_VAL(eq_state, AUDPRC_DAC_PATH_CFG1_EQ_STAGE_Msk, AUDPRC_DAC_PATH_CFG1_EQ_STAGE_Pos));
    MODIFY_REG(haprc->Instance->DAC_PATH_CFG1, AUDPRC_DAC_PATH_CFG1_EQ_CH_EN_Msk,
               MAKE_REG_VAL(3, AUDPRC_DAC_PATH_CFG1_EQ_CH_EN_Msk, AUDPRC_DAC_PATH_CFG1_EQ_CH_EN_Pos));


    pointer = (uint32_t *) & (haprc->Instance->DAC_EQ_CFG0);
    for (int i = 0; i < eq_state * 5; i++)
    {
        *pointer = eq_value[i];
        pointer++;
    }

    MODIFY_REG(haprc->Instance->CFG, AUDPRC_CFG_AUTO_GATE_EN_Msk,
               MAKE_REG_VAL(1, AUDPRC_CFG_AUTO_GATE_EN_Msk, AUDPRC_CFG_AUTO_GATE_EN_Pos));
}

void bf0_audprc_eq_enable_offline(uint8_t is_enable)
{
    g_eq_enable = is_enable;
    LOG_I("set g_eq_enable=%d", g_eq_enable);
}
void eq_debug_reconfig_eq()
{
    //clear eq
    AUDPRC_HandleTypeDef *haprc = get_audprc_handle();
    AUDCODEC_HandleTypeDef *hacodec = get_audcodec_handle();

    return;//ID1003296

#if 0
    if (!audio_server_is_speaker_working())
    {
        return;
    }
    //HAL_DBG_printf("haprc=0x%x DAC_PATH_CFG1=0x%x", haprc->Instance, &haprc->Instance->DAC_PATH_CFG1);
    //HAL_DBG_printf("codec hp=0x%x", hacodec->Instance_hp);

    MODIFY_REG(haprc->Instance->DAC_PATH_CFG1, AUDPRC_DAC_PATH_CFG1_EQ_CLR_Msk,
               MAKE_REG_VAL(1, AUDPRC_DAC_PATH_CFG1_EQ_CLR_Msk, AUDPRC_DAC_PATH_CFG1_EQ_CLR_Pos));

    while (!(haprc->Instance->DAC_PATH_CFG1 & AUDPRC_DAC_PATH_CFG1_EQ_CLR_DONE_Msk));


    //HAL_DBG_printf("done set");

    MODIFY_REG(haprc->Instance->DAC_PATH_CFG1, AUDPRC_DAC_PATH_CFG1_EQ_CLR_Msk,
               MAKE_REG_VAL(0, AUDPRC_DAC_PATH_CFG1_EQ_CLR_Msk, AUDPRC_DAC_PATH_CFG1_EQ_CLR_Pos));

    //disable ramp


    //HAL_DBG_printf("clear ramp");
#ifdef SF32LB52X
    MODIFY_REG(hacodec->Instance->DAC_CH0_CFG_EXT, AUDCODEC_DAC_CH0_CFG_EXT_RAMP_EN_Msk,
               MAKE_REG_VAL(0, AUDCODEC_DAC_CH0_CFG_EXT_RAMP_EN_Msk, AUDCODEC_DAC_CH0_CFG_EXT_RAMP_EN_Pos));
    MODIFY_REG(hacodec->Instance->DAC_CH1_CFG_EXT, AUDCODEC_DAC_CH1_CFG_EXT_RAMP_EN_Msk,
               MAKE_REG_VAL(0, AUDCODEC_DAC_CH1_CFG_EXT_RAMP_EN_Msk, AUDCODEC_DAC_CH1_CFG_EXT_RAMP_EN_Pos));
#else
    MODIFY_REG(hacodec->Instance_hp->DAC_CH0_CFG_EXT, AUDCODEC_HP_DAC_CH0_CFG_EXT_RAMP_EN_Msk,
               MAKE_REG_VAL(0, AUDCODEC_HP_DAC_CH0_CFG_EXT_RAMP_EN_Msk, AUDCODEC_HP_DAC_CH0_CFG_EXT_RAMP_EN_Pos));
    MODIFY_REG(hacodec->Instance_hp->DAC_CH1_CFG_EXT, AUDCODEC_HP_DAC_CH1_CFG_EXT_RAMP_EN_Msk,
               MAKE_REG_VAL(0, AUDCODEC_HP_DAC_CH1_CFG_EXT_RAMP_EN_Msk, AUDCODEC_HP_DAC_CH1_CFG_EXT_RAMP_EN_Pos));
#endif
    //HAL_DBG_printf("finish");
#endif

}


static inline int get_audprc_adc_volume()
{
    return g_adc_volume;
}


void hal_audprc_set_dac_volume(int audprc_volume)
{
    extern __HAL_ROM_USED HAL_StatusTypeDef HAL_AUDPRC_Config_DACPath_Volume(AUDPRC_HandleTypeDef * haprc, int channel, int volume);
    HAL_AUDPRC_Config_DACPath_Volume(get_audprc_handle(), 0, audprc_volume);
    HAL_AUDPRC_Config_DACPath_Volume(get_audprc_handle(), 1, audprc_volume);
}

#ifdef RT_USING_FINSH

static inline int set_music_state(char *value)
{
    g_music_state = atoi(value);
    LOG_I("g_music_state=%d", g_music_state);
    if (g_music_state > 10)
    {
        return -1;
    }
    return 0;
}

static inline int set_music_eq(int argc, char *argv[])
{
    char str[3] = {0};
    int index = strtoul(argv[2], 0, 10);
    if (argc < 4)
    {
        LOG_I("input para num %d < 4", argc);
        return -1;
    }

    if (strlen(argv[3]) != 40)
    {
        LOG_I("argv[2] len error");
        return -1;
    }

    if ((index >= 10) || (index < 0))
    {
        LOG_I("argv[2] %d not in 0-9", argv[2]);
        return -1;
    }

    uint8_t *pData = (uint8_t *)&g_music_eqValue[index * 5];
    for (int m = 0; m < 20; m++)
    {
        str[0] = argv[3][m * 2];
        str[1] = argv[3][m * 2 + 1];

        pData[m] = strtoul(str, 0, 16);
    }
    eq_debug_reconfig_eq();
    return 0;
}

static inline int get_music_eq(char *value)
{
    int index = strtoul(value, 0, 10);
    LOG_I("value:0x%08x,0x%08x,0x%08x,0x%08x,0x%08x",
          g_music_eqValue[index * 5],
          g_music_eqValue[index * 5 + 1],
          g_music_eqValue[index * 5 + 2],
          g_music_eqValue[index * 5 + 3],
          g_music_eqValue[index * 5 + 4]);
    return 0;
}

static inline int set_voice_state(char *value)
{
    g_voice_state = atoi(value);
    LOG_I("g_voice_state=%d", g_voice_state);
    if (g_voice_state > 10)
    {
        return -1;
    }
    return 0;
}

static inline int set_voice_eq(int argc, char *argv[])
{
    char str[3] = {0};
    int index = strtoul(argv[2], 0, 10);
    if (argc < 4)
    {
        LOG_I("input para num %d < 4", argc);
        return -1;
    }

    if (strlen(argv[3]) != 40)
    {
        LOG_I("argv[2] len error");
        return -1;
    }

    if ((index >= 10) || (index < 0))
    {
        LOG_I("argv[2] %d not in 0-9", argv[2]);
        return -1;
    }

    uint8_t *pData = (uint8_t *)&g_voice_eqValue[index * 5];
    for (int m = 0; m < 20; m++)
    {
        str[0] = argv[3][m * 2];
        str[1] = argv[3][m * 2 + 1];

        pData[m] = strtoul(str, 0, 16);
    }
    eq_debug_reconfig_eq();

    return 0;
}

static inline int get_voice_eq(char *value)
{
    int index = strtoul(value, 0, 10);

    LOG_I("value:0x%08x,0x%08x,0x%08x,0x%08x,0x%08x",
          g_voice_eqValue[index * 5],
          g_voice_eqValue[index * 5 + 1],
          g_voice_eqValue[index * 5 + 2],
          g_voice_eqValue[index * 5 + 3],
          g_voice_eqValue[index * 5 + 4]);

    return 0;
}

static inline int get_tel_adc()
{
    LOG_I("aprc_debug_vol %d\n", g_adc_volume);
    return 0;
}

static inline int set_tel_adc(char *value)
{
    g_adc_volume = atoi(value);
    LOG_I("aprc_debug_vol_w success");

    get_audprc_handle()->Init.adc_cfg.vol_l = g_adc_volume;
    get_audprc_handle()->Init.adc_cfg.vol_r = g_adc_volume;
    HAL_AUDPRC_Config_ADCPath_Volume(get_audprc_handle(), 0, g_adc_volume);
    HAL_AUDPRC_Config_ADCPath_Volume(get_audprc_handle(), 1, g_adc_volume);
    return 0;
}

static inline int get_tel_dac()
{
    LOG_I("aprc_debug_vol %d\n", g_tel_max_vol);
    return 0;
}

static inline int set_tel_dac(char *value)
{
    g_tel_max_vol = atoi(value);
    if (g_tel_max_vol < 2 * AUDCODEC_MIN_VOLUME || g_tel_max_vol > 2 * AUDCODEC_MAX_VOLUME)
    {
        return -1;
    }
    LOG_I("aprc_debug_vol_w success");
    return 0;
}
static inline int set_tel_max_dac_level(char *value)
{
    g_tel_max_vol_level = atoi(value);
    if (g_tel_max_vol_level < 0 || g_tel_max_vol_level > 6)
        return -1;
    LOG_I("aprc_debug_vol_w success");
    return 0;
}

static inline int get_tel_max_dac_level()
{
    LOG_I("aprc_debug_vol %d\n", g_tel_max_vol_level);
    return 0;
}
static inline int get_eq_version()
{
    LOG_I("aprc_debug_version %s\n", g_eq_code_ver);
    return 0;
}
static inline int get_music_dac()
{
    LOG_I("aprc_debug_vol %d\n", g_music_max_vol);
    return 0;
}
static inline int set_music_dac_max(char *value)
{
    int volume;
    g_music_max_vol = atoi(value);
    if (g_music_max_vol < 2 * AUDCODEC_MIN_VOLUME || g_music_max_vol > 2 * AUDCODEC_MAX_VOLUME)
        return -1;
    LOG_I("aprc_debug_vol_w success");
    return 0;
}
static inline int set_music_max_dac_level(char *value)
{
    g_music_max_vol_level = atoi(value);
    if (g_music_max_vol_level < 0 || g_music_max_vol_level > 6)
        return -1;

    LOG_I("aprc_debug_vol_w success");
    return 0;
}

static inline int get_music_max_dac_level()
{
    LOG_I("aprc_debug_vol %d\n", g_music_max_vol_level);
    return 0;
}

static inline int get_music_level_volume(char *index)
{
    int i = atoi(index);
    if (i < 0 || i > 15)
    {
        return -1;
    }
    LOG_I("aprc_debug_vol %d\n", g_music_vol_level[i]);
    return 0;
}

static inline int set_music_level_volume(char *index, char *value)
{
    int i = atoi(index);

    if (i < 0 || i > 15)
    {
        return -1;
    }
    g_music_vol_level[i] = atoi(value);
    LOG_I("aprc_debug_vol_w success");
    return 0;
}

static inline int set_music_default_volume(char *index, char *value)
{
    int i = atoi(index);
    g_eq_is_bt_music = 1;
    if (i < 0 || i > 15)
    {
        return -1;
    }
    g_eq_default_volume = (uint8_t)i;
    g_music_vol_level[i] = atoi(value);
    LOG_I("aprc_debug_vol_w success");
    return 0;
}

static inline int set_tel_default_volume(char *index, char *value)
{
    int i = atoi(index);
    g_eq_is_bt_music = 0;
    if (i < 0 || i > 15)
    {
        return -1;
    }
    g_eq_default_volume = (uint8_t)i;
    g_tel_vol_level[i] = atoi(value);
    LOG_I("aprc_debug_vol_w success");
    return 0;
}

static inline int get_tel_level_volume(char *index)
{
    int i = atoi(index);
    if (i < 0 || i > 15)
    {
        return -1;
    }
    LOG_I("aprc_debug_vol %d\n", g_tel_vol_level[i]);
    return 0;
}

static inline int set_tel_level_volume(char *index, char *value)
{
    int i = atoi(index);
    if (i < 0 || i > 15)
    {
        return -1;
    }
    g_tel_vol_level[i] = atoi(value);
    LOG_I("music volume[%d]=%d", i, g_tel_vol_level[i]);
    LOG_I("aprc_debug_vol_w success");
    return 0;
}

static inline int set_eq_start()
{
#ifdef RT_USING_PM
    if (g_eq_debuging == 0)
    {
        rt_pm_request(PM_SLEEP_MODE_IDLE);
    }
#endif
    g_eq_debuging = 1;
    return 0;
}

static inline int set_eq_end()
{
    save_eq();
#ifdef RT_USING_PM
    if (g_eq_debuging == 1)
    {
        rt_pm_release(PM_SLEEP_MODE_IDLE);
    }
#endif
    g_eq_debuging = 0;

    return 0;
}

#if 1//!PKG_USING_3MICS_WITHOUT_ADC
int mic_gain(int argc, char **argv)
{
    if (argc < 2)
    {
        rt_kprintf("mic gain=%ddb\r\n", get_mic_volume());
        rt_kprintf("mic gain reg=0x%08x\r\n", get_audprc_handle()->Instance->ADC_PATH_CFG0);
        return 0;
    }
    g_adc_volume = atoi(argv[1]);
    rt_kprintf("set mic gain to %ddb\r\n", g_adc_volume);
    get_audprc_handle()->Init.adc_cfg.vol_l = g_adc_volume;
    get_audprc_handle()->Init.adc_cfg.vol_r = g_adc_volume;
    HAL_AUDPRC_Config_ADCPath_Volume(get_audprc_handle(), 0, g_adc_volume);
    HAL_AUDPRC_Config_ADCPath_Volume(get_audprc_handle(), 1, g_adc_volume);
    rt_kprintf("ADC_PATH_CFG0=0x%x\r\n", get_audprc_handle()->Instance->ADC_PATH_CFG0);
    return 0;
}

MSH_CMD_EXPORT_ALIAS(mic_gain, mic_gain, mic_gain);
#endif

int Set_mic_gain(int8_t value)
{
    g_adc_volume = value;
    rt_kprintf("set mic gain to %ddb\r\n", g_adc_volume);
    get_audprc_handle()->Init.adc_cfg.vol_l = g_adc_volume;
    get_audprc_handle()->Init.adc_cfg.vol_r = g_adc_volume;
    HAL_AUDPRC_Config_ADCPath_Volume(get_audprc_handle(), 0, g_adc_volume);
    HAL_AUDPRC_Config_ADCPath_Volume(get_audprc_handle(), 1, g_adc_volume);
    rt_kprintf("ADC_PATH_CFG0=0x%x\r\n", get_audprc_handle()->Instance->ADC_PATH_CFG0);
    return 0;
}



#if AUDIO_USING_PDM

extern void set_pdm_gain_to_register(int val);

int pdm_gain(int argc, char **argv)
{
    int val;
    if (argc < 2)
    {
        uint32_t base = PDM1_BASE + 0x18;
        uint32_t *p = (uint32_t *)base;
        rt_kprintf("pdm gain=%d * 0.5db=%fdb\r\n", g_pdm_volume, (float)g_pdm_volume / 2.0f);
        rt_kprintf("pdm gain reg=0x%08x\r\n", *p);
        return 0;
    }
    val = atoi(argv[1]);
    if (val < -30)
    {
        val = -30;
    }
    else if (val > 90)
    {
        val = 90;
    }
    set_pdm_gain_to_register(val);
    g_pdm_volume = val;
    rt_kprintf("set pdm gain to %d * 0.5db=%fdb, not update to reg\r\n", g_pdm_volume, (float)g_pdm_volume / 2.0f);
    return 0;
}
MSH_CMD_EXPORT_ALIAS(pdm_gain, pdm_gain, pdm_gain);


int Set_pdm_gain(int8_t value)
{
    int val;

    val = value;
    if (val < -30)
    {
        val = -30;
    }
    else if (val > 90)
    {
        val = 90;
    }
    set_pdm_gain_to_register(val);
    g_pdm_volume = val;
    rt_kprintf("set pdm gain to %d * 0.5db=%fdb, not update to reg\r\n", g_pdm_volume, (float)g_pdm_volume / 2.0f);
    return 0;
}
#endif

int aprc_debug(int argc, char *argv[])
{
    int ret = -1;

    if (argc < 3)
    {
        LOG_I("input para num %d < 3", argc);
        goto Exit;
    }

    if (argv[1][0] == 's') //aprc_debug s xxx
    {
        if (argv[1][1] == 'm')
            ret = set_music_state(argv[2]);
        else if (argv[1][1] == 'v')
            ret = set_voice_state(argv[2]);
    }
    else if (argc == 4 && argv[1][0] == 'w') //aprc_debug w index yyy
    {
        if (argv[1][1] == 'm')
            ret = set_music_eq(argc, argv);
        else if (argv[1][1] == 'v')
            ret = set_voice_eq(argc, argv);
    }
    else if (argv[1][0] == 'r') //aprc_debug r xxx
    {
        if (argv[1][1] == 'm')
            ret = get_music_eq(argv[2]);
        else if (argv[1][1] == 'v')
            ret = get_voice_eq(argv[2]);
    }
    else if (argv[1][0] == 't')
    {
        if (argc == 5 && argv[2][0] == 'm' && argv[3][0] == 'w') //aprc_debug t mlevel w xxx
            ret = set_tel_max_dac_level(argv[4]);
        else if (argc == 4  && argv[2][0] == 'm' && argv[3][0] == 'r')
            ret = get_tel_max_dac_level(); //aprc_debug m mlevel r
        else if (argc == 5 && argv[2][0] == 'l' && argv[3][0] == 'r')
            ret = get_tel_level_volume(argv[4]);
        else if (argc == 5 && argv[2][0] == 'l')
            ret = set_tel_level_volume(argv[3], argv[4]);
        else if (argc == 5 && argv[2][0] == 'd')
            ret = set_tel_default_volume(argv[3], argv[4]);

    }
    else if (argv[1][0] == 'e')
    {
        if (argv[2][0] == 'v') //aprc_debug eq v
            ret = get_eq_version();
        else if (argv[2][0] == 's') //aprc_debug eq start
            ret = set_eq_start();
        else if (argv[2][0] == 'e') //aprc_debug eq end
            ret = set_eq_end();
    }
    else if (argv[1][0] == 'v' && argv[2][0] == 'r') //aprc_debug vol r
    {
        ret = get_tel_dac();
    }
    else if (argc > 3 && argv[1][0] == 'v' && argv[2][0] == 'w') //aprc_debug vol w xxx
    {
        ret = set_tel_dac(argv[3]);
    }
    else if (argv[1][0] == 'a' && argv[2][0] == 'r') //aprc_debug adc r
    {
        ret = get_tel_adc();
    }
    else if (argc > 3 && argv[1][0] == 'a' && argv[2][0] == 'w') //aprc_debug adc w xxx
    {
        ret = set_tel_adc(argv[3]);
    }
    else if (argv[1][0] == 'm')
    {
        if (argc == 3 && argv[2][0] == 'r') //aprc_debug mvol r
            ret = get_music_dac();
        else if (argc == 4 && argv[2][0] == 'w') //aprc_debug mvol w xxx
            ret = set_music_dac_max(argv[3]);
        else if (argc == 4 && argv[2][0] == 'm' && argv[3][0] == 'r')
            ret = get_music_max_dac_level(); //aprc_debug m mlevel r
        else if (argc == 5 && argv[2][0] == 'm' && argv[3][0] == 'w')
            ret = set_music_max_dac_level(argv[4]);  //aprc_debug m mlevel w xxx
        else if (argc == 5 && argv[2][0] == 'l' && argv[3][0] == 'r')
            ret = get_music_level_volume(argv[4]);
        else if (argc == 5 && argv[2][0] == 'l')
            ret = set_music_level_volume(argv[3], argv[4]);
        else if (argc == 5 && argv[2][0] == 'd')
            ret = set_music_default_volume(argv[3], argv[4]);
    }

Exit:
    if (ret < 0)
    {
        LOG_I("aprc_rw fail");
    }
    else
    {
        LOG_I("aprc_rw success");
    }

    return ret;
}
MSH_CMD_EXPORT(aprc_debug, aprc_debug r / w idx value);


int8_t bf0_audprc_get_adc_vol(void)
{
    return g_adc_volume;
}

void bf0_audprc_set_adc_vol(int8_t vol)
{
    g_adc_volume = vol;
}


int8_t bf0_audprc_get_max_call_dac_vol(void)
{
    return g_tel_max_vol;
}

void bf0_audprc_set_max_call_dac_vol(int8_t vol)
{
    g_tel_max_vol = vol;
}


int8_t bf0_audprc_get_max_call_dac_vol_level(void)
{
    return g_tel_max_vol_level;
}

void bf0_audprc_set_max_call_dac_vol_level(int8_t lvl)
{
    g_tel_max_vol_level = lvl;
}

int32_t bf0_audprc_get_call_dac_vol_level(uint8_t idx, int8_t *lvl)
{
    if (idx > 15 || lvl == NULL)
        return -1;
    *lvl = g_tel_vol_level[idx];
    return 0;
}

int32_t bf0_audprc_set_call_dac_vol_level(uint8_t idx, int8_t lvl)
{
    if (idx > 15)
        return -1;
    g_tel_vol_level[idx] = lvl;
    return 0;
}


int32_t bf0_audprc_set_default_call_dac_vol_level(uint8_t idx, int8_t lvl)
{
    g_eq_is_bt_music = 0;
    if (idx < 0 || idx > 15)
    {
        return -1;
    }
    g_eq_default_volume = (uint8_t)idx;
    g_tel_vol_level[idx] = lvl;
    return 0;
}


int8_t bf0_audprc_get_max_music_dac_vol(void)
{
    return g_music_max_vol;
}

void bf0_audprc_set_max_music_dac_vol(int8_t vol)
{
    g_music_max_vol = vol;
}

int8_t bf0_audprc_get_max_music_dac_vol_level(void)
{
    return g_music_max_vol_level;
}

void bf0_audprc_set_max_music_dac_vol_level(int8_t lvl)
{
    g_music_max_vol_level = lvl;
}

int32_t bf0_audprc_get_music_dac_vol_level(uint8_t idx, int8_t *lvl)
{
    if (idx > 15 || lvl == NULL)
        return -1;
    *lvl = g_music_vol_level[idx];
    return 0;
}

int32_t bf0_audprc_set_music_dac_vol_level(uint8_t idx, int8_t lvl)
{
    if (idx > 15)
        return -1;

    g_music_vol_level[idx] = lvl;
    return 0;
}

int32_t bf0_audprc_set_default_music_dac_vol_level(uint8_t idx, int8_t lvl)
{
    g_eq_is_bt_music = 1;
    if (idx > 15)
    {
        return -1;
    }
    g_eq_default_volume = (uint8_t)idx;
    g_music_vol_level[idx] = lvl;
    return 0;
}

void bf0_audprc_get_eq_code_ver(char *ver, uint8_t len)
{
    if (len >= sizeof(g_eq_code_ver))
        memcpy(ver, g_eq_code_ver, sizeof(g_eq_code_ver));
}

int32_t bf0_audprc_set_music_eq(uint8_t idx, uint8_t *data, uint8_t len)
{
    if (len != 20)
    {
        return -1;
    }

    if (idx >= 10)
    {
        return -2;
    }

    uint8_t *pData = (uint8_t *)&g_music_eqValue[idx * 5];
    memcpy(pData, data, len);
    eq_debug_reconfig_eq();
    return 0;

}

int32_t bf0_audprc_get_music_eq(uint8_t idx, uint8_t *data, uint8_t len)
{
    if (data == NULL || len != 20)
        return -1;

    memcpy(data, (uint8_t *)&g_music_eqValue[idx * 5], len);
    return 0;
}

int32_t bf0_audprc_set_voice_eq(uint8_t idx, uint8_t *data, uint8_t len)
{
    if (len != 20)
    {
        return -1;
    }

    if (idx >= 10)
    {
        return -2;
    }

    uint8_t *pData = (uint8_t *)&g_voice_eqValue[idx * 5];
    memcpy(pData, data, len);
    eq_debug_reconfig_eq();
    return 0;

}

int32_t bf0_audprc_get_voice_eq(uint8_t idx, uint8_t *data, uint8_t len)
{
    if (data == NULL || len != 20)
        return -1;

    memcpy(data, (uint8_t *)&g_voice_eqValue[idx * 5], len);
    return 0;
}


int32_t bf0_audprc_set_music_state(uint8_t state)
{
    if (state > 10)
        return -1;

    g_music_state = state;
    return 0;
}

int32_t bf0_audprc_set_voice_state(uint8_t state)
{
    if (state > 10)
        return -1;

    g_voice_state = state;
    return 0;
}


int32_t bf0_audprc_start_eq(uint8_t is_start)
{
    if (is_start)
        set_eq_start();
    else
        set_eq_end();
    return 0;
}


#endif


//#define DRV_TEST
#if defined(DRV_TEST) || defined (APP_BSP_TEST)

#include "string.h"
#include "drv_flash.h"


/****
 * if define use flash, i2s rx data save to flash and tx load from flash, use jlink to check memory
 * if node define use flash, default use psram memory, save/load with psram, use jlink to savebin/loadbin
**/
#define SAVE_WAVE_TO_FLASH
#define AUD_SAVE_MEM_BASE           (0x1c180000)

#define AUD_LOAD_MEM_BASE           (0x1c180000)

#define AUDIO_BUF_SIZE 1024
#define AUD_TEST_FLEN       (0x7E000)

static rt_device_t dev_i2s;
static rt_device_t dev_aprc;
static uint8_t g_pipe_data[AUDIO_BUF_SIZE];
static uint8_t tx_pipe_data[AUDIO_BUF_SIZE];

static rt_thread_t rx_tid;
static rt_thread_t tx_tid;
static rt_event_t g_rx_ev;
static rt_event_t g_tx_ev;
static uint8_t *buf_flag = NULL;

static char *mic_buf = (char *)AUD_SAVE_MEM_BASE;
static char *audio_mem_buf = (char *)AUD_LOAD_MEM_BASE;
static int aud_flen = 0;

static int aptest_save(char *dst, uint8_t *src, uint32_t size)
{
    uint32_t addr = (uint32_t)dst;
    int res = 0;
#ifdef SAVE_WAVE_TO_FLASH
    // for flash , use flash write
    res = rt_flash_write(addr, src, size);
#else
    // for psram , use memcpy
    memcpy(dst, src, size);
    res = size;
#endif
    return res;
}

static void aptest_init_buf(char *buf, int data, uint32_t size)
{
    uint32_t addr = (uint32_t)buf;
#ifdef SAVE_WAVE_TO_FLASH
    // for flash, use erase
    rt_flash_erase(addr, size);
#else
    // for psram
    memset(buf, data, size);
#endif
    return;
}

typedef struct
{
    uint8_t riff[4];
    uint32_t lenth;
    uint8_t wave[4];
    uint8_t fmt[4];
    uint32_t size1;
    uint16_t fmt_tag;
    uint16_t channel;
    uint32_t sampleRate;
    uint32_t bytePerSec;
    uint16_t blockAlign;
    uint16_t bitPerSample;
    uint8_t data[4];
    uint32_t size2;
} AUD_WAV_HDR_T;

static void aptest_fill_header(uint32_t sr)
{
    AUD_WAV_HDR_T hdr;

    hdr.riff[0] = 'R';
    hdr.riff[1] = 'I';
    hdr.riff[2] = 'F';
    hdr.riff[3] = 'F';
    hdr.lenth = AUD_TEST_FLEN + 36;
    hdr.wave[0] = 'W';
    hdr.wave[1] = 'A';
    hdr.wave[2] = 'V';
    hdr.wave[3] = 'E';
    hdr.fmt[0] = 'f';
    hdr.fmt[1] = 'm';
    hdr.fmt[2] = 't';
    hdr.fmt[3] = ' ';
    hdr.size1 = 16;
    hdr.fmt_tag = 1;
    hdr.channel = 1;
    hdr.sampleRate = sr;
    //hdr.bytePerSec = 32000;
    hdr.blockAlign = 2;
    hdr.bitPerSample = 16;
    hdr.bytePerSec = hdr.sampleRate * hdr.channel * hdr.bitPerSample / 8;
    hdr.data[0] = 'd';
    hdr.data[1] = 'a';
    hdr.data[2] = 't';
    hdr.data[3] = 'a';
    hdr.size2 = AUD_TEST_FLEN; //;    // record data lenght
    LOG_I("save wav hdr sampel rate %d\n", hdr.sampleRate);

    aptest_init_buf(mic_buf, 0, 0x80000);
    aptest_save(mic_buf, (uint8_t *)&hdr, 44);
    aud_flen = 0;
    mic_buf += 44;
}

void bf0_ap_tx_entry(void *param)
{
    rt_uint32_t evt, size, cnt;
    struct bf0_audio_prc *haprc = (struct bf0_audio_prc *)param;

    cnt = 0;
    // read header if wave file include wav header
    memcpy(tx_pipe_data, audio_mem_buf + cnt, 44);
    cnt += 44;

    // read data
    memcpy(tx_pipe_data, audio_mem_buf + cnt, AUDIO_BUF_SIZE);
    cnt += AUDIO_BUF_SIZE;

    // for test only use tx ch0
    //memcpy(haprc->audprc.buf[HAL_AUDPRC_TX_CH0], tx_pipe_data, AUDIO_BUF_SIZE);
    rt_device_write(dev_aprc, 0, tx_pipe_data, AUDIO_BUF_SIZE);
    size = AUDIO_BUF_SIZE / 2;

    g_tx_ev = rt_event_create("audio_tx_evt", RT_IPC_FLAG_FIFO);
    rt_kprintf("bf0_audio_tx_entry started, wait event\n");
    while (1)
    {
        size = AUD_TEST_FLEN - cnt > (AUDIO_BUF_SIZE / 2) ? (AUDIO_BUF_SIZE / 2) : AUD_TEST_FLEN - cnt;
        // read data
        memcpy(tx_pipe_data, audio_mem_buf + cnt, size);
        cnt += size;
        if (cnt >= AUD_TEST_FLEN)
            cnt = 44; // 0; // to data header

        // wait dma done to fill to buffer
        rt_event_recv(g_tx_ev, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        if (buf_flag != NULL)
        {
            //memcpy(buf_flag, tx_pipe_data, size);
            rt_device_write(dev_aprc, 0, tx_pipe_data, size);
        }
        LOG_I("filled %d\n", cnt);
    }
}


void bf0_a_rx_entry(void *param)
{
    rt_uint32_t evt;
    int size;

    g_rx_ev = rt_event_create("audio_evt", RT_IPC_FLAG_FIFO);
    rt_kprintf("bf0_audio_rx_entry started, wait event\n");
    while (1)
    {
        rt_event_recv(g_rx_ev, 1, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
        while (1)
        {
            rt_size_t len;
            len = rt_device_read(dev_aprc, 0, g_pipe_data, AUDIO_BUF_SIZE);
            LOG_I("Got Audio size=%d\n", len);
            if (len != AUDIO_BUF_SIZE / 2)
            {
                //LOG_I("Got Audio size=%d\n", len);
                //LOG_HEX("g_pipe_data", 10, g_pipe_data, len > 16 ? 16 : len);
            }

            if (aud_flen + len <= AUD_TEST_FLEN)
            {
                aptest_save(mic_buf, g_pipe_data, len);
                mic_buf += len;
                aud_flen += len;
            }

            if (len == AUDIO_BUF_SIZE)
                break;
        }
    }
}

static rt_err_t ap_rx_ind(rt_device_t dev, rt_size_t size)
{
    LOG_I("audio_rx_ind %d\n", size);
    rt_event_send(g_rx_ev, 1);
    return RT_EOK;
}

rt_err_t ap_tx_done(rt_device_t dev, void *buffer)
{
    LOG_I("audio_tx_done \n");
    buf_flag = (uint8_t *)buffer;
    rt_event_send(g_tx_ev, 1);
    return RT_EOK;
}


/**
* @brief  Audio commands.
* This function provide 'audio' command to shell(FINSH) .
* The commands supported:
*   - audio open

      Open microphone and speaker device

    - audio config [sample rate]

      Configure microphone catpure sample rate

    - audio start rx

      Audio start capture, it will start \ref bf0_audio_rx_entry thread.

    - audio start tx

      Audio start replay, it will start \ref bf0_audio_tx_entry thread.

    - audio stop rx

      Audio stop capture

   - audio stop tx

      Audio stop replay
* @retval RT_EOK
*/
int cmd_aprc(int argc, char *argv[])
{
    if (argc > 1)
    {
        if (strcmp(argv[1], "open") == 0)
        {
            if (dev_aprc == NULL)
            {
                dev_aprc = rt_device_find("audprc");
                if (dev_aprc)
                {
                    rt_device_open(dev_aprc, RT_DEVICE_FLAG_RDWR);
                    LOG_I("audio prc opened\n");
                }
                else
                {
                    LOG_E("Could not find audio prc device\n");
                    return -RT_ERROR;
                }
            }
            if (dev_i2s == NULL)
            {
                dev_i2s = rt_device_find("i2s2");
                if (dev_i2s)
                {
                    rt_device_open(dev_i2s, RT_DEVICE_FLAG_RDWR);
                    LOG_I("I2S opened\n");
                }
                else
                {
                    LOG_E("Could not find i2s device\n");
                    return -RT_ERROR;
                }
            }
        }
        if (strcmp(argv[1], "config") == 0)
        {
            if (dev_aprc)
            {
                uint32_t inf = 1;
                struct rt_audio_caps caps;
                if (strcmp(argv[2], "rx") == 0)
                {
                    // set i2s rx external interface
                    inf = 1;
                    rt_device_control(dev_i2s, AUDIO_CTL_SETINPUT, (void *)inf);
#if 0
                    // configure i2s sample rate
                    caps.main_type = AUDIO_TYPE_INPUT;
                    caps.sub_type = AUDIO_DSP_SAMPLERATE;
                    caps.udata.value = 16000;
                    rt_device_control(dev_i2s, AUDIO_CTL_CONFIGURE, &caps);

                    // configure i2s track, for i2s external interface, it should alway be stereo?
                    caps.main_type = AUDIO_TYPE_INPUT;
                    caps.sub_type = AUDIO_DSP_CHANNELS;
                    caps.udata.value = 2;
                    rt_device_control(dev_i2s, AUDIO_CTL_CONFIGURE, &caps);
#endif
                    caps.main_type = AUDIO_TYPE_INPUT;      // for I2S2, configure RX will configure RX+TX
                    caps.sub_type = AUDIO_DSP_PARAM;
                    caps.udata.config.channels   = 2;
                    caps.udata.config.samplerate = 16000; //8000, 16000
                    caps.udata.config.samplefmt = 16; //16 or 32
                    rt_device_control(dev_i2s, AUDIO_CTL_CONFIGURE, &caps);
                    LOG_I("Config audio parameter: channel %d, samplerate %d, bitwidth %d\n", caps.udata.config.channels,
                          caps.udata.config.samplerate, caps.udata.config.samplefmt);

                    // set aud prc source from external interface
                    inf = AUDPRC_RX_FROM_I2S;
                    rt_device_control(dev_aprc, AUDIO_CTL_SETINPUT, (void *)inf);

                    caps.main_type = AUDIO_TYPE_INPUT;
                    caps.sub_type = 0;  // for rx channel 0
                    //caps.udata.value = AUDPRC_RX_FROM_I2S;
                    caps.udata.config.channels = 1;
                    caps.udata.config.samplefmt = 16;
                    caps.udata.config.samplerate = 16000;
                    rt_device_control(dev_aprc, AUDIO_CTL_CONFIGURE, &caps);

                    //inf = AUDPRC_RX_FROM_I2S;
                    //rt_device_control(dev_aprc, AUDIO_CTL_SETINPUT, (void *)inf);
                }
                else if (strcmp(argv[2], "tx") == 0)
                {
                    // configure i2s interface
                    inf = 1;
                    rt_device_control(dev_i2s, AUDIO_CTL_SETOUTPUT, (void *)inf);

                    // configure i2s sample rate
                    caps.main_type = AUDIO_TYPE_OUTPUT;
                    caps.sub_type = AUDIO_DSP_SAMPLERATE;
                    caps.udata.value = 16000;
                    rt_device_control(dev_i2s, AUDIO_CTL_CONFIGURE, &caps);

                    // configure i2s track, for i2s external interface, it should alway be stereo?
                    caps.main_type = AUDIO_TYPE_OUTPUT;
                    caps.sub_type = AUDIO_DSP_CHANNELS;
                    caps.udata.value = 2;
                    rt_device_control(dev_i2s, AUDIO_CTL_CONFIGURE, &caps);

                    // configure audprc interface
                    inf = AUDPRC_TX_TO_I2S;
                    rt_device_control(dev_aprc, AUDIO_CTL_SETOUTPUT, (void *)inf);

                    // configure data source format
                    caps.main_type = AUDIO_TYPE_OUTPUT;
                    caps.sub_type = 0;  // tx channel 0
                    //caps.udata.value = AUDPRC_TX_TO_I2S;
                    caps.udata.config.channels = 1;
                    caps.udata.config.samplefmt = 16;
                    caps.udata.config.samplerate = 16000;
                    // tx channel use ch0 fix for test
                    rt_device_control(dev_aprc, AUDIO_CTL_CONFIGURE, &caps);
                }
                else if (strcmp(argv[2], "src") == 0)
                {
                    int src_sr, dst_sr, chn;
                    struct rt_audio_sr_convert cfg;
                    src_sr = 16000; // same with pcm data
                    dst_sr = 48000; // output to i2s
                    chn = 2;    // same to pcm data

                    cfg.channel = chn;
                    cfg.source_sr = src_sr;
                    cfg.dest_sr = dst_sr;

                    // configure i2s interface
                    inf = 1;
                    rt_device_control(dev_i2s, AUDIO_CTL_SETOUTPUT, (void *)inf);

                    // configure i2s sample rate
                    caps.main_type = AUDIO_TYPE_OUTPUT;
                    caps.sub_type = AUDIO_DSP_SAMPLERATE;
                    caps.udata.value = dst_sr;
                    rt_device_control(dev_i2s, AUDIO_CTL_CONFIGURE, &caps);

                    // configure i2s track, for i2s external interface, it should alway be stereo?
                    caps.main_type = AUDIO_TYPE_OUTPUT;
                    caps.sub_type = AUDIO_DSP_CHANNELS;
                    caps.udata.value = chn;
                    rt_device_control(dev_i2s, AUDIO_CTL_CONFIGURE, &caps);

                    // configure audprc interface
                    inf = AUDPRC_TX_TO_I2S;
                    rt_device_control(dev_aprc, AUDIO_CTL_SETOUTPUT, (void *)inf);

                    // config src
                    //bf0_audprc_src((struct rt_audio_device *)dev_aprc, src_sr, dst_sr, 0);
                    rt_device_control(dev_aprc, AUDIO_CTL_OUTPUTSRC, (void *)(&cfg));

                    // configure data source format
                    caps.main_type = AUDIO_TYPE_OUTPUT;
                    caps.sub_type = 0;  // tx channel 0
                    //caps.udata.value = AUDPRC_TX_TO_I2S;
                    caps.udata.config.channels = chn;
                    caps.udata.config.samplefmt = 16;
                    caps.udata.config.samplerate = 16000;
                    // tx channel use ch0 fix for test
                    rt_device_control(dev_aprc, AUDIO_CTL_CONFIGURE, &caps);
                }
                else
                {
                    LOG_E("Error parameters\n");
                    return RT_ERROR;
                }

                LOG_I("Config AUD PRC with I2S interface %d\n", inf);
            }
        }
        if (strcmp(argv[1], "start") == 0)
        {
            if (dev_aprc)
            {
                int stream = 0;
                if (strcmp(argv[2], "tx") == 0)
                {
                    stream = AUDIO_STREAM_REPLAY;
                    // start replay thread
                    tx_tid = rt_thread_create("tx_th", bf0_ap_tx_entry, dev_aprc, 1024, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
                    if (tx_tid == NULL)
                    {
                        LOG_E("Create tx thread fail\n");
                        return RT_ERROR;
                    }
                    rt_thread_startup(tx_tid);
                    rt_device_set_tx_complete(dev_aprc, ap_tx_done);
                    rt_device_control(dev_i2s, AUDIO_CTL_START, &stream);
                    stream |= ((1 << HAL_AUDPRC_TX_CH0) << 8);
                    rt_device_control(dev_aprc, AUDIO_CTL_START, &stream);
                }
                if (strcmp(argv[2], "rx") == 0)
                {
                    stream = AUDIO_STREAM_RECORD;
                    struct rt_audio_caps caps;
                    caps.main_type = AUDIO_TYPE_INPUT;
                    caps.sub_type = AUDIO_DSP_SAMPLERATE;
                    rt_device_control(dev_i2s, AUDIO_CTL_GETCAPS, &caps);
                    aptest_fill_header(caps.udata.value);   //
                    // start record thread
                    rx_tid = rt_thread_create("aud_th", bf0_a_rx_entry, dev_aprc, 1024, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);
                    if (rx_tid == NULL)
                    {
                        LOG_E("Create rx thread fail\n");
                        return RT_ERROR;
                    }
                    rt_thread_startup(rx_tid);
                    rt_device_set_rx_indicate(dev_aprc, ap_rx_ind);
                    rt_device_control(dev_i2s, AUDIO_CTL_START, &stream);
                    stream |= ((1 << HAL_AUDPRC_RX_CH0) << 8);
                    rt_device_control(dev_aprc, AUDIO_CTL_START, &stream);
                }
            }
        }
        if (strcmp(argv[1], "stop") == 0)
        {
            if (dev_aprc)
            {
                int stream = 0;
                if (strcmp(argv[2], "tx") == 0)
                {
                    stream = AUDIO_STREAM_REPLAY;
                    rt_device_control(dev_aprc, AUDIO_CTL_STOP, &stream);
                    rt_device_control(dev_i2s, AUDIO_CTL_STOP, &stream);
                }
                if (strcmp(argv[2], "rx") == 0)
                {
                    stream = AUDIO_STREAM_RECORD;
                    rt_device_control(dev_aprc, AUDIO_CTL_STOP, &stream);
                    rt_device_control(dev_i2s, AUDIO_CTL_STOP, &stream);
                }
            }
        }
        if (strcmp(argv[1], "reg") == 0)
        {
            ADPRC_DUMP_REG();
        }
    }
    return RT_EOK;
}
FINSH_FUNCTION_EXPORT_ALIAS(cmd_aprc, __cmd_aprc, Test audioPrc driver);
MSH_CMD_EXPORT(cmd_aprc, audio_prc);

#endif  //DRV_TEST

#endif  //BSP_ENABLE_AUD_PRC
