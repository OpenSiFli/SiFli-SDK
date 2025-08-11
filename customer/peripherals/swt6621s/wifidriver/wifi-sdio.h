/** @file wifi-sdio.h
 *
 *  @brief WLAN on SDIO
 *
 *  Copyright 2008-2022 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef __WIFI_SDIO_H__
#define __WIFI_SDIO_H__

#include <wifi.h>
//#include "firmware_dnld.h"

#define wifi_io_e(...) wmlog_e("wifi_io", ##__VA_ARGS__)
#define wifi_io_w(...) wmlog_w("wifi_io", ##__VA_ARGS__)

#if CONFIG_WIFI_IO_DEBUG
    #define wifi_io_d(...) wmlog("wifi_io", ##__VA_ARGS__)
#else
    #define wifi_io_d(...)
#endif /* ! CONFIG_WIFI_IO_DEBUG */

#define wifi_io_info_e(...) wmlog_e("wpkt", ##__VA_ARGS__)
#define wifi_io_info_w(...) wmlog_w("wpkt", ##__VA_ARGS__)

#if CONFIG_WIFI_IO_INFO_DUMP
    #define wifi_io_info_d(...) wmlog("wpkt", ##__VA_ARGS__)
#else
    #define wifi_io_info_d(...)
#endif

#define WLAN_MAGIC_NUM (('W' << 0) | ('L' << 8) | ('F' << 16) | ('W' << 24))

#if CONFIG_RF_TEST_MODE
    /* sizeof(HostCmd_DS_COMMAND) in worst case is 2828 where HostCmd_DS_MFG_CMD_OTP_CAL_DATA_T size is 2820 */
    #define WIFI_FW_CMDBUF_SIZE 2832U
#else
    #define WIFI_FW_CMDBUF_SIZE 2100U
#endif

#define WIFI_RESP_WAIT_TIME 10

#define SDIO_INBUF_LEN (2048 * 2)

#define SDIO_OUTBUF_LEN 2048U

#if (SDIO_INBUF_LEN % MLAN_SDIO_BLOCK_SIZE)
    #error "Please keep buffer length aligned to SDIO block size"
#endif /* Sanity check */

#if (SDIO_OUTBUF_LEN % MLAN_SDIO_BLOCK_SIZE)
    #error "Please keep buffer length aligned to SDIO block size"
#endif /* Sanity check */

#define SDIO_PAYLOAD_SIZE 16

/*! @brief Data block count accessed in card */
#define DATA_BLOCK_COUNT (4U)
/*! @brief Data buffer size. */
#ifndef __ZEPHYR__
    #define DATA_BUFFER_SIZE (FSL_SDMMC_DEFAULT_BLOCK_SIZE * DATA_BLOCK_COUNT)
#else
    #define DATA_BUFFER_SIZE (SDMMC_DEFAULT_BLOCK_SIZE * DATA_BLOCK_COUNT)
#endif

/* Duplicated in wlan.c. keep in sync till we can be included directly */
typedef struct __nvram_backup_struct
{
    t_u32 ioport;
    t_u32 curr_wr_port;
    t_u32 curr_rd_port;
    t_u32 mp_end_port;
    t_u32 bss_num;
    t_u32 sta_mac_addr1;
    t_u32 sta_mac_addr2;
    t_u32 wifi_state;
} nvram_backup_t;

extern bool g_txrx_flag;
#ifdef WLAN_LOW_POWER_ENABLE
    extern bool low_power_mode;
#endif
extern bool cal_data_valid;
extern bool mac_addr_valid;

mlan_status sd_wifi_init(enum wlan_type type, const uint8_t *fw_start_addr, const size_t size);

#if (CONFIG_WIFI_IND_DNLD)
    mlan_status sd_wifi_reinit(enum wlan_type type, const uint8_t *fw_start_addr, const size_t size, uint8_t fw_reload);
#endif

mlan_status sd_wifi_post_init(enum wlan_type type);

void sd_wifi_reset_ports();

void sd_wifi_deinit(void);

/*
 * @internal
 *
 *
 */
int wlan_send_sdio_cmd(t_u8 *buf, t_u32 tx_blocks, t_u32 buflen);

/*
 * @internal
 *
 *
 */
int wifi_send_cmdbuffer(t_u32 tx_blocks, t_u32 len);

/*
 * @internal
 *
 *
 */
HostCmd_DS_COMMAND *wifi_get_command_buffer(void);
#if CONFIG_FW_VDLL
    int wifi_send_vdllcmdbuffer(t_u32 tx_blocks, t_u32 len);
    HostCmd_DS_COMMAND *wifi_get_vdllcommand_buffer(void);
    int wlan_send_sdio_vdllcmd(t_u8 *buf, t_u32 tx_blocks, t_u32 buflen);
#endif

mlan_status wlan_process_int_status(mlan_adapter *pmadapter);
mlan_status wlan_xmit_pkt(t_u8 *buffer, t_u32 txlen, t_u8 interface, t_u32 tx_control);
int raw_process_pkt_hdrs(void *pbuf, t_u32 payloadlen, t_u8 interface);
uint32_t wifi_get_device_value1(void);

#if CONFIG_WMM
    uint8_t *wifi_wmm_get_sdio_outbuf(uint32_t *outbuf_len, mlan_wmm_ac_e queue);
    mlan_status wlan_xmit_wmm_pkt(t_u8 interface, t_u32 txlen, t_u8 *tx_buf);
    mlan_status wlan_flush_wmm_pkt(t_u8 pkt_cnt);
    mlan_status wlan_xmit_bypass_pkt(t_u8 *buffer, t_u32 txlen, t_u8 interface);
    #if CONFIG_AMSDU_IN_AMPDU
        uint8_t *wifi_get_amsdu_outbuf(uint32_t offset);
        mlan_status wlan_xmit_wmm_amsdu_pkt(mlan_wmm_ac_e ac, t_u8 interface, t_u32 txlen, t_u8 *tx_buf, t_u8 amsdu_cnt);
    #endif
#endif

void sdio_enable_interrupt(void);

void sdio_disable_interrupt(void);

void process_pkt_hdrs(void *pbuf, t_u32 payloadlen, t_u8 interface, t_u8 tid, t_u32 tx_control);

#if CONFIG_WIFI_FW_DEBUG
    extern void wifi_dump_firmware_info();
    extern void wifi_sdio_reg_dbg();
#endif /* CONFIG_WIFI_FW_DEBUG */

#endif /* __WIFI_SDIO_H__ */
