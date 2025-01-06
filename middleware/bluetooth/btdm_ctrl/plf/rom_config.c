/**
  ******************************************************************************
  * @file   rom_config.c
  * @author Sifli software development team
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



#include <rtconfig.h>
#include <stdint.h>
#include <string.h>
#ifdef BSP_USING_PC_SIMULATOR
    #define __USED
#else
    #include <cmsis_compiler.h>
#endif // !1
#include "board.h"
#include "h4tl.h"
#include "rwip.h"
#include "rom_config.h"
#if (EMB_PRESENT)
    #include "em_map.h"
#endif



typedef struct
{
    const struct bt_eif_api *port_if[H4TL_NB_CHANNEL_MAX];
} ble_port_config_t;


static ble_port_config_t g_port_config;
static ble_mem_config_t g_mem_config;

static bluetooth_act_configt_t *g_act_config;

uint16_t g_rom_em_offset[ROM_EM_END];

__USED rom_config_t g_rom_config =
{
    .controller_enable_bit = 3,
    .lld_prog_delay = 2,
    .lld_prog_delay_min = 2,
    .default_sleep_mode = 0,
    .default_sleep_enabled = 1,
    .default_xtal_enabled = 1,
    .default_rc_cycle = 8,
    .max_sleep_time = 327680,
};


#if (EMB_PRESENT)
__USED const static rom_em_default_attr_t g_rom_em_attr[ROM_EM_END] =
{
    {EM_BLE_OFFSET, 1},  {REG_EM_BLE_CS_SIZE, EM_BLE_CS_NB_MAX}, {REG_EM_BLE_WPAL_SIZE, BLE_WHITELIST_MAX_CONFIG}, {REG_EM_BLE_RAL_SIZE, BLE_RESOL_ADDR_LIST_MAX_CONFIG},
    {REG_EM_BLE_RX_DESC_SIZE, EM_BLE_RX_DESC_NB}, {REG_EM_BLE_TX_DESC_SIZE, EM_BLE_TX_DESC_NB}, {EM_BLE_LLCPTXBUF_SIZE, EM_BLE_LLCPTXBUF_NB},
    {EM_BLE_ADVEXTHDRTXBUF_SIZE, EM_BLE_ADVEXTHDRTXBUF_NB}, {EM_BLE_ADVDATATXBUF_SIZE, EM_BLE_ADVDATATXBUF_NB}, {EM_BLE_AUXCONNECTREQTXBUF_SIZE, EM_BLE_AUXCONNECTREQTXBUF_NB},
    {EM_BLE_DATARXBUF_SIZE, EM_BLE_DATARXBUF_NB}, {EM_BLE_ACLTXBUF_SIZE, EM_BLE_ACLTXBUF_NB}, {EM_BLE_ISO_HOP_SEQ_SIZE, EM_BLE_ISO_HOP_SEQ_NB},
    {EM_BLE_ISO_DESC_SIZE, EM_BLE_ISO_DESC_NB}, {EM_BLE_ISO_BUF_SIZE, EM_BLE_ISO_BUF_NB}, {0, 0}, {0, 0}, {0, 0},
    {0, 0}, // BLE end
    {REG_EM_BT_CS_SIZE, EM_BT_CS_NB}, {REG_EM_BT_RXDESC_SIZE, EM_BT_RXDESC_NB}, {REG_EM_BT_TXDESC_SIZE, EM_BT_TXDESC_NB}, {EM_BT_LMPRXBUF_SIZE, EM_BT_LMPRXBUF_NB},
    {EM_BT_LMPTXBUF_SIZE, EM_BT_LMPTXBUF_NB}, {EM_BT_ISCANFHSTXBUF_SIZE, 1}, {EM_BT_PAGEFHSTXBUF_SIZE, 1}, {EM_BT_EIRTXBUF_SIZE, 1}, {EM_BT_LOCAL_SAM_SUBMAP_SIZE, 1},
    {RW_PEER_SAM_MAP_MAX_LEN, MAX_NB_ACTIVE_ACL}, {EM_BT_STPTXBUF_SIZE, 1}, {EM_BT_ACLRXBUF_SIZE, EM_BT_ACLRXBUF_NB}, {EM_BT_ACLTXBUF_SIZE, EM_BT_ACLTXBUF_NB},
    {EM_BT_AUDIOBUF_SIZE, EM_BT_AUDIOBUF_NB},
    {0, 0}, // BT_END
};
#endif //(EMB_PRESENT)


uint8_t rom_config_get_ble_controller_enabled(void)
{
    return ((rom_config_get_controller_enabled() & BLE_CONTROLLER_ENABLE_MASK) == BLE_CONTROLLER_ENABLE_MASK);
}

uint8_t rom_config_get_bt_controller_enabled(void)
{
    return ((rom_config_get_controller_enabled() & BT_CONTROLLER_ENABLE_MASK) == BT_CONTROLLER_ENABLE_MASK);
}



uint8_t rom_config_get_controller_enabled(void)
{
    return g_rom_config.controller_enable_bit;
}

uint8_t rom_config_get_lld_prog_delay(void)
{
    return g_rom_config.lld_prog_delay;
}

uint8_t rom_config_get_lld_prog_delay_min(void)
{
    return g_rom_config.lld_prog_delay_min;
}

uint8_t rom_config_get_default_sleep_mode(void)
{
    return g_rom_config.default_sleep_mode;
}

uint8_t rom_config_get_default_sleep_enabled(void)
{
    return g_rom_config.default_sleep_enabled;
}

uint8_t rom_config_get_default_xtal_enabled(void)
{
    return g_rom_config.default_xtal_enabled;
}

uint8_t rom_config_get_default_rc_cycle(void)
{
    return g_rom_config.default_rc_cycle;
}

uint32_t rom_config_get_max_sleep_time(void)
{
    return g_rom_config.max_sleep_time;
}

uint8_t rom_config_get_default_swprofiling(void)
{
    return g_rom_config.default_swprofiling_cfg;
}


__USED void rom_config_set_controller_enabled(uint8_t enabled_module)
{
    g_rom_config.controller_enable_bit = enabled_module;
}

__USED void rom_config_set_lld_prog_delay(uint8_t lld_prog_delay)
{
    g_rom_config.lld_prog_delay = lld_prog_delay;
}

__USED void rom_config_set_lld_prog_delay_min(uint8_t lld_prog_delay_min)
{
    g_rom_config.lld_prog_delay_min = lld_prog_delay_min;
}

__USED void rom_config_set_default_sleep_mode(uint8_t default_sleep_mode)
{
    g_rom_config.default_sleep_mode = default_sleep_mode;
}

__USED void rom_config_set_default_sleep_enabled(uint8_t default_sleep_enabled)
{
    g_rom_config.default_sleep_enabled = default_sleep_enabled;
}

__USED void rom_config_set_default_xtal_enabled(uint8_t default_xtal_enabled)
{
    g_rom_config.default_xtal_enabled = default_xtal_enabled;
}

__USED void rom_config_set_default_rc_cycle(uint8_t default_rc_cycle)
{
    g_rom_config.default_rc_cycle = default_rc_cycle;
}

__USED void rom_config_set_max_sleep_time(uint32_t max_sleep_time)
{
    g_rom_config.max_sleep_time = max_sleep_time;
}

__USED void rom_config_set_swprofiling(uint8_t default_swprofiling)
{
    g_rom_config.default_swprofiling_cfg = default_swprofiling;
}

__USED void rom_config_set_tl(const struct bt_eif_api *tl, uint8_t idx)
{
    if (idx < H4TL_NB_CHANNEL)
        g_port_config.port_if[idx] = tl;
}

__USED const struct bt_eif_api *rom_config_get_tl(uint8_t idx)
{
    const struct bt_eif_api *tl = NULL;
    if (idx < H4TL_NB_CHANNEL)
    {
        tl = g_port_config.port_if[idx];
    }
    return tl;
}


__USED void ble_memory_config(ble_mem_config_t *config)
{
    g_mem_config = *config;
}

__USED ble_mem_config_t *ble_memory_config_get(void)
{
    return &g_mem_config;
}

__USED int8_t ble_max_nb_of_cp_get(void)
{
    return g_mem_config.max_nb_of_hci_completed;
}


__USED void rom_config_em_memory(uint8_t *buf, uint8_t num)
{
    if (num > ROM_EM_END)
        num = ROM_EM_END;
    memcpy((void *)g_rom_em_offset, (void *)buf, num * 2);
}


#if (EMB_PRESENT)
__USED const rom_em_default_attr_t *rom_config_get_default_attribute_4_em(rom_em_offset_t em_type)
{
    return &g_rom_em_attr[em_type];
}
#endif // (EMB_PRESENT)

__USED void rom_config_set_default_link_config(bluetooth_act_configt_t *cfg)
{
    g_act_config = cfg;
}

bluetooth_act_configt_t *rom_config_get_default_link_config(void)
{
    return g_act_config;
}

inline uint8_t rom_config_get_ble_max_act(void)
{
    return g_act_config->ble_max_act;
}


