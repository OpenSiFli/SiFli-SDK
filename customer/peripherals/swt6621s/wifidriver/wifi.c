/** @file wifi.c
 *
 *  @brief  This file provides WiFi Core API
 *
 *  Copyright 2008-2024 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <mlan_api.h>

#include <stdio.h>
#include <string.h>
#include <wifi.h>
#include <osa.h>

#include "wifi-internal.h"
#include <wm_net.h>
#if defined(RW610)
    #include "wifi-imu.h"
#else
    //#include <mlan_sdio_api.h>
    #include "wifi-sdio.h"
    //#include "mlan_sdio.h"
    // #include "sdio.h"
    // #include "firmware_dnld.h"

#endif
#ifdef RW610
    //#include "fsl_adapter_imu.h"
#endif

/* Always keep this include at the end of all include files */
#include <mlan_remap_mem_operations.h>

#if CONFIG_HEAP_DEBUG
    OSA_SEMAPHORE_HANDLE_DEFINE(os_mem_stat_sem);

    t_u32 valid_item_cnt = 0;
    wifi_os_mem_info wifi_os_mem_stat[OS_MEM_STAT_TABLE_SIZE];
#endif

#if CONFIG_CSI
#define MAX_CSI_LOCAL_BUF        80
#define CSI_LOCAL_BUF_ENTRY_SIZE 768
t_u8 csi_local_buff[MAX_CSI_LOCAL_BUF][CSI_LOCAL_BUF_ENTRY_SIZE] =
{
    0,
};

csi_local_buff_statu csi_buff_stat = {0, 0, 0};

int csi_event_cnt        = 0;
t_u64 csi_event_data_len = 0;
#endif

#if CONFIG_ECSA
    extern wifi_ecsa_status_control ecsa_status_control;
#endif

#if !CONFIG_WIFI_CORE_STACK_SIZE
    #define CONFIG_WIFI_CORE_STACK_SIZE (2048)
#endif

#define MAX_MCAST_LEN (MLAN_MAX_MULTICAST_LIST_SIZE * MLAN_MAC_ADDR_LENGTH)
#define MAX_WAIT_TIME 3000

#ifndef USB_SUPPORT_ENABLE
    #define _T(x) x
#endif

#if CONFIG_WMM
    #define BOARD_DATA_BUFFER_ALIGN_SIZE 32

    SDK_ALIGN(uint8_t outbuf_arr[MAX_WMM_BUF_NUM][OUTBUF_WMM_LEN], BOARD_DATA_BUFFER_ALIGN_SIZE);
#endif

#define RXPD_CHAN_MASK 0x3FE0
static int wifi_low_level_input(const uint8_t interface, const uint8_t *buffer, const uint16_t len);
/* Global variable wm_rand_seed */
uint32_t wm_rand_seed = 0xFFFFFFFFU;

#if CONFIG_WMM
    OSA_SEMAPHORE_HANDLE_DEFINE(txbuf_sem);
#endif

bool sta_ampdu_tx_enable         = true;
t_u8 sta_ampdu_tx_enable_per_tid = 0xFF;

bool sta_ampdu_rx_enable         = true;
t_u8 sta_ampdu_rx_enable_per_tid = 0xFF;

bool uap_ampdu_tx_enable         = true;
t_u8 uap_ampdu_tx_enable_per_tid = 0xFF;

bool uap_ampdu_rx_enable         = true;
t_u8 uap_ampdu_rx_enable_per_tid = 0xFF;

/* tx status: 0-RUNNING, 1-BLOCK */
t_u8 wifi_tx_status = WIFI_DATA_RUNNING;
/* tx data count blocked */
t_u8 wifi_tx_block_cnt = 0;
/* rx status: 0-RUNNING, 1-BLOCK */
t_u8 wifi_rx_status = WIFI_DATA_RUNNING;
/* rx data count blocked */
t_u8 wifi_rx_block_cnt = 0;

int retry_attempts;
wm_wifi_t wm_wifi;
static bool xfer_pending;
static bool scan_thread_in_process = false;

#if CONFIG_HOST_SLEEP
    OSA_SEMAPHORE_HANDLE_DEFINE(wakelock);
    int wakeup_by = 0;
#endif
#if CONFIG_WIFI_RECOVERY
    bool wifi_recovery_enable = false;
    t_u16 wifi_recovery_cnt   = 0;
#endif
bool wifi_shutdown_enable = false;

typedef enum __mlan_status
{
    MLAN_STATUS_FW_DNLD_SKIP = 1,
    MLAN_CARD_NOT_DETECTED   = 3,
    MLAN_STATUS_FW_DNLD_FAILED,
    MLAN_STATUS_FW_NOT_DETECTED,
    MLAN_STATUS_FW_NOT_READY,
    MLAN_CARD_CMD_TIMEOUT
} __mlan_status;
#ifndef RW610

    static void wifi_core_task(osa_task_param_t arg);

    /* OSA_TASKS: name, priority, instances, stackSz, useFloat */
    static OSA_TASK_DEFINE(wifi_core_task, 10, 1, CONFIG_WIFI_CORE_STACK_SIZE, 0);

#endif

#if !CONFIG_WIFI_SCAN_STACK_SIZE
    #define CONFIG_WIFI_SCAN_STACK_SIZE (4096)
#endif

static void wifi_scan_task(osa_task_param_t arg);

/* OSA_TASKS: name, priority, instances, stackSz, useFloat */
static OSA_TASK_DEFINE(wifi_scan_task, 10, 1, CONFIG_WIFI_SCAN_STACK_SIZE, 0);

#if !CONFIG_WIFI_DRIVER_STACK_SIZE
    #define CONFIG_WIFI_DRIVER_STACK_SIZE (2048)
#endif

static void wifi_drv_task(osa_task_param_t arg);

/* OSA_TASKS: name, priority, instances, stackSz, useFloat */
static OSA_TASK_DEFINE(wifi_drv_task, 10, 1, CONFIG_WIFI_DRIVER_STACK_SIZE, 0);

#if CONFIG_WMM

    #if !CONFIG_WIFI_DRV_TX_STACK_SIZE
        #define CONFIG_WIFI_DRV_TX_STACK_SIZE (2048)
    #endif

    static void wifi_drv_tx_task(osa_task_param_t arg);

    /* OSA_TASKS: name, priority, instances, stackSz, useFloat */
    #ifdef RW610
        static OSA_TASK_DEFINE(wifi_drv_tx_task, WLAN_TASK_PRI_NORMAL, 1, CONFIG_WIFI_DRV_TX_STACK_SIZE, 0);
    #else
        static OSA_TASK_DEFINE(wifi_drv_tx_task, WLAN_TASK_PRI_HIGH, 1, CONFIG_WIFI_DRV_TX_STACK_SIZE, 0);
    #endif
#endif

#if !CONFIG_WIFI_POWERSAVE_STACK_SIZE
    #define CONFIG_WIFI_POWERSAVE_STACK_SIZE (512)
#endif

static void wifi_powersave_task(osa_task_param_t arg);

/* OSA_TASKS: name, priority, instances, stackSz, useFloat */
static OSA_TASK_DEFINE(wifi_powersave_task, 10, 1, CONFIG_WIFI_POWERSAVE_STACK_SIZE, 0);

int wifi_set_mac_multicast_addr(const char *mlist, t_u32 num_of_addr);
int wrapper_get_wpa_ie_in_assoc(uint8_t *wpa_ie);

#if CONFIG_HOST_SLEEP
int wakelock_get(void)
{
    int ret = WM_SUCCESS;
#if CONFIG_POWER_MANAGER
    ret = OSA_SemaphorePost((osa_semaphore_handle_t)wakelock);
    if (ret != WM_SUCCESS)
        wifi_e("Failed to get wakelock");
#endif
    return ret;
}

int wakelock_put(void)
{
    int ret = WM_SUCCESS;
#if CONFIG_POWER_MANAGER
    ret = OSA_SemaphoreWait((osa_semaphore_handle_t)wakelock, 0);
    if (ret != WM_SUCCESS)
        wifi_e("Failed to put wakelock");
#endif
    return ret;
}

int wakelock_isheld(void)
{
#if CONFIG_POWER_MANAGER
    return OSA_MsgQAvailableMsgs((osa_msgq_handle_t)wakelock);
#else
    return 1;
#endif
}
#endif
extern void process_pkt_hdrs(void *pbuf, t_u32 payloadlen, t_u8 interface, t_u8 tid, t_u32 tx_control);

unsigned wifi_get_last_cmd_sent_ms(void)
{
    return wm_wifi.last_sent_cmd_msec;
}

uint32_t wifi_get_value1(void)
{
    return wifi_get_device_value1();
}

/* Wake up Wi-Fi card */
void wifi_wake_up_card(uint32_t *resp)
{
#if CONFIG_WIFI_PS_DEBUG
    wcmdr_d("Wakeup device...");
#endif

#ifndef RW610
    //(void)sdio_drv_creg_write(0x0, 1, 0x02, resp);
#else
    imu_wakeup_card();
#endif
}

/* When Wi-Fi card is in IEEE PS and sleeping
 * CMD or Data cannot be transmited.
 * The card must be woken up.
 * So data or command trasnfer is temporarily kept
 * in pending state. This function returns value
 * of pending flag true/false.
 */
bool wifi_get_xfer_pending(void)
{
    return xfer_pending;
}
/*
 * This function sets the flag value
 */
void wifi_set_xfer_pending(bool xfer_val)
{
    xfer_pending = xfer_val;
}

void wifi_update_last_cmd_sent_ms(void)
{
    wm_wifi.last_sent_cmd_msec = OSA_TimeGetMsec();
}

static int wifi_get_command_resp_sem(unsigned long wait)
{
    return OSA_SemaphoreWait((osa_semaphore_handle_t)wm_wifi.command_resp_sem, wait);
}

int wifi_put_command_resp_sem(void)
{
    return OSA_SemaphorePost((osa_semaphore_handle_t)wm_wifi.command_resp_sem);
}

#define WL_ID_WIFI_CMD "wifi_cmd"

int wifi_get_command_lock(void)
{
    osa_status_t status;

#if CONFIG_HOST_SLEEP
    wakelock_get();
#endif
    status = OSA_MutexLock((osa_mutex_handle_t)wm_wifi.command_lock, osaWaitForever_c);

    if (status != KOSA_StatusSuccess)
    {
        return -WM_FAIL;
    }

    return WM_SUCCESS;
}

int wifi_put_command_lock(void)
{
    osa_status_t status;

#if CONFIG_HOST_SLEEP
    wakelock_put();
#endif
    status = OSA_MutexUnlock((osa_mutex_handle_t)wm_wifi.command_lock);

    if (status != KOSA_StatusSuccess)
    {
        return -WM_FAIL;
    }

    return WM_SUCCESS;
}

static int wifi_get_mcastf_lock(void)
{
    osa_status_t status;

    status = OSA_MutexLock((osa_mutex_handle_t)wm_wifi.mcastf_mutex, osaWaitForever_c);

    if (status != KOSA_StatusSuccess)
    {
        return -WM_FAIL;
    }

    return WM_SUCCESS;
}

static int wifi_put_mcastf_lock(void)
{
    osa_status_t status;

    status = OSA_MutexUnlock((osa_mutex_handle_t)wm_wifi.mcastf_mutex);

    if (status != KOSA_StatusSuccess)
    {
        return -WM_FAIL;
    }

    return WM_SUCCESS;
}



#if CONFIG_FW_VDLL
int wifi_wait_for_vdllcmdresp(void *cmd_resp_priv)
{
    int ret                 = WM_SUCCESS;
    HostCmd_DS_COMMAND *cmd = wifi_get_vdllcommand_buffer();
#ifndef RW610
    t_u32 buf_len = MLAN_SDIO_BLOCK_SIZE;
    t_u32 tx_blocks;
#endif

#ifndef RW610
#if (CONFIG_WIFI_CMD_RESP_DEBUG) && (CONFIG_FW_VDLL_DEBUG)

    wcmdr_d("VDLL CMD --- : 0x%x Size: %d Seq: %x", cmd->command, cmd->size, cmd->seq_num);
#endif /* CONFIG_ENABLE_WARNING_LOGS || CONFIG_WIFI_CMD_RESP_DEBUG*/
#endif
    if (cmd->size > WIFI_FW_CMDBUF_SIZE)
    {
        /*
         * This is a error added to be flagged during
         * development cycle. It is not expected to
         * occur in production. The legacy code below
         * only sents out MLAN_SDIO_BLOCK_SIZE or 2 *
         * MLAN_SDIO_BLOCK_SIZE sized packet. If ever
         * in future greater packet size generated then
         * this error will help to localize the problem.
         */
        wifi_e("cmd size greater than WIFI_FW_CMDBUF_SIZE\r\n");
        return -WM_FAIL;
    }

#ifndef RW610
    tx_blocks = ((t_u32)cmd->size + MLAN_SDIO_BLOCK_SIZE - 1U) / MLAN_SDIO_BLOCK_SIZE;
#endif

#if defined(RW610)
    (void)wifi_send_cmdbuffer();
#else
    (void)wifi_send_vdllcmdbuffer(tx_blocks, buf_len);
#endif

    return ret;
}
#endif

#if (CONFIG_WIFI_IND_DNLD)
static int wifi_reinit(uint8_t fw_reload);
t_u8 wifi_rx_block_cnt;
t_u8 wifi_tx_block_cnt;

void wlan_process_hang(uint8_t fw_reload)
{
    int i, ret = WM_SUCCESS;

    if (mlan_adap->in_reset == true)
    {
        wifi_d("Already in process hanging");
        return;
    }

    wifi_d("Start to process hanging");

#if CONFIG_WIFI_IND_RESET
    wifi_ind_reset_start();
#endif

    /* Block TX data */
    wifi_set_tx_status(WIFI_DATA_BLOCK);
    /* Block RX data */
    wifi_set_rx_status(WIFI_DATA_BLOCK);

    if (is_split_scan_complete() == false)
    {
        wifi_user_scan_config_cleanup();
        (void)wifi_event_completion(WIFI_EVENT_SCAN_RESULT, WIFI_EVENT_REASON_FAILURE, NULL);
    }

    mlan_adap->in_reset = true;
    for (i = 0; i < (int)(MIN(MLAN_MAX_BSS_NUM, mlan_adap->priv_num)); i++)
    {
        if (mlan_adap->priv[i]->media_connected == MTRUE)
        {
            mlan_adap->priv[i]->media_connected = MFALSE;

            if (mlan_adap->priv[i]->bss_type == MLAN_BSS_TYPE_STA)
            {
            }
            else if (mlan_adap->priv[i]->bss_type == MLAN_BSS_TYPE_UAP)
            {
                mlan_adap->priv[i]->uap_bss_started = MFALSE;
            }
        }

        if (mlan_adap->priv[i])
        {
            wlan_clean_txrx(mlan_adap->priv[i]);
        }
    }

    (void)wifi_event_completion(WIFI_EVENT_FW_HANG, WIFI_EVENT_REASON_SUCCESS, NULL);

    ret = wifi_reinit(fw_reload);

    if (ret != WM_SUCCESS)
    {
        ASSERT(0);
    }

    /* Unblock TX data */
    wifi_set_tx_status(WIFI_DATA_RUNNING);
    /* Unblock RX data */
    wifi_set_rx_status(WIFI_DATA_RUNNING);
    mlan_adap->in_reset = false;
    wifi_tx_block_cnt   = 0;
    wifi_rx_block_cnt   = 0;

    (void)wifi_event_completion(WIFI_EVENT_FW_RESET, WIFI_EVENT_REASON_SUCCESS, NULL);

}
#endif
extern uint8_t ctrl_cmd_buf[WIFI_FW_CMDBUF_SIZE];
int wifi_wait_for_cmdresp(void *cmd_resp_priv)
{
    int ret;
    HostCmd_DS_COMMAND *cmd = wifi_get_command_buffer();
#ifndef RW610
    t_u32 buf_len = MLAN_SDIO_BLOCK_SIZE;
    t_u32 tx_blocks;
#endif
    mlan_private *pmpriv    = (mlan_private *)mlan_adap->priv[0];
    mlan_adapter *pmadapter = pmpriv->adapter;

#ifndef RW610
#if (CONFIG_ENABLE_WARNING_LOGS) || (CONFIG_WIFI_CMD_RESP_DEBUG)

#if !CONFIG_WIFI_PS_DEBUG
    if (cmd->command != HostCmd_CMD_802_11_PS_MODE_ENH)
    {
        wcmdr_d("CMD --- : 0x%x Size: %d Seq: %d", cmd->command, cmd->size, cmd->seq_num);
    }
#else
    wcmdr_d("CMD --- : 0x%x Size: %d Seq: %d", cmd->command, cmd->size, cmd->seq_num);
#endif

#endif /* CONFIG_ENABLE_WARNING_LOGS || CONFIG_WIFI_CMD_RESP_DEBUG*/
#endif

#if CONFIG_WIFI_IND_RESET
    /* IR is in progress so any CMD coming during progress should be ignored */
    if (wifi_ind_reset_in_progress() == true)
    {
        (void)wifi_put_command_lock();
        return WM_SUCCESS;
    }
#endif

#if CONFIG_FW_VDLL
    while (pmadapter->vdll_in_progress == MTRUE)
    {
        OSA_TimeDelay(50);
    }
#endif

    if (0)//(cmd->size > WIFI_FW_CMDBUF_SIZE)
    {
        /*
         * This is a error added to be flagged during
         * development cycle. It is not expected to
         * occur in production. The legacy code below
         * only sents out MLAN_SDIO_BLOCK_SIZE or 2 *
         * MLAN_SDIO_BLOCK_SIZE sized packet. If ever
         * in future greater packet size generated then
         * this error will help to localize the problem.
         */
        wifi_e("cmd size greater than WIFI_FW_CMDBUF_SIZE cmd->size=%d\r\n", cmd->size);
        RT_ASSERT(0);
        (void)wifi_put_command_lock();
        return -WM_FAIL;
    }

#if !CONFIG_UART_WIFI_BRIDGE
    ret = OSA_RWLockReadLock(&sleep_rwlock, MAX_WAIT_TIME);
    if (ret != WM_SUCCESS)
    {
#if CONFIG_WIFI_PS_DEBUG
        wifi_e("Failed to wakeup card");
#endif

        // wakelock_put(WL_ID_LL_OUTPUT);
        (void)wifi_put_command_lock();
#if CONFIG_WIFI_RECOVERY
        wifi_recovery_enable = true;
        return -WM_FAIL;
#else
        RT_ASSERT(0);
#endif
    }
#endif
#if CONFIG_WMM_UAPSD
    /*
     * No PS handshake between driver and FW for the uapsd case,
     * CMD should not wakeup FW, needs to wait to send till receiving PS_AWAKE Event from FW.
     */
    OSA_SemaphoreWait((osa_semaphore_handle_t)uapsd_sem, osaWaitForever_c);
#endif
    /*
     * This is the private pointer. Only the command response handler
     * function knows what it means or where it points to. It can be
     * NULL.
     */
    wm_wifi.cmd_resp_priv = cmd_resp_priv;

#if !CONFIG_UART_WIFI_BRIDGE
    /* put the sleep_rwlock after send command but not wait for the command response,
     * for sleep confirm command, sleep confirm response(in wifi_process_ps_enh_response())
     * would try to get the sleep_rwlock until get it,
     * so here put the sleep_rwlock as early as possible.
     */
    (void)OSA_RWLockReadUnlock(&sleep_rwlock);
#endif

    pmadapter->cmd_sent = MTRUE;

    /* Wait max 20 sec for the command response */
    ret = wifi_get_command_resp_sem(WIFI_COMMAND_RESPONSE_WAIT_MS);//等待sem信号
    if (0)//(ret != WM_SUCCESS)
    {
        pmadapter->cmd_sent = MFALSE;
    }

    if (cmd->command == HostCmd_CMD_FUNC_SHUTDOWN)
    {
        wifi_shutdown_enable = true;
    }

    wm_wifi.cmd_resp_priv = NULL;
#if CONFIG_WMM_UAPSD
    OSA_SemaphorePost((osa_semaphore_handle_t)uapsd_sem);
#endif
    wifi_set_xfer_pending(false);

    (void)wifi_put_command_lock();
    return ret;
}


int wifi_event_completion(enum wifi_event event, enum wifi_event_reason result, void *data)
{
    struct wifi_message msg;
    if (wm_wifi.wlc_mgr_event_queue == MNULL)
    {
        wifi_e("wlc_mgr_event_queue has not been created, event %d", event);
        return -WM_FAIL;
    }

    msg.data   = data;
    msg.reason = result;
    msg.event  = (uint16_t)event;
    if (OSA_MsgQPut((osa_msgq_handle_t)wm_wifi.wlc_mgr_event_queue, &msg) != KOSA_StatusSuccess)
    {
        wifi_e("Failed to send response on Queue, event %d", event);
        return -WM_FAIL;
    }
    return WM_SUCCESS;
}

static int cmp_mac_addr(uint8_t *mac_addr1, uint8_t *mac_addr2)
{
    int i = 0;

    if ((mac_addr1 == MNULL) || (mac_addr2 == MNULL))
    {
        return 1;
    }

    for (i = 0; i < MLAN_MAC_ADDR_LENGTH; i++)
    {
        if (mac_addr1[i] != mac_addr2[i])
        {
            return 1;
        }
    }
    return 0;
}

static int add_mcast_ip(uint8_t *mac_addr)
{
    mcast_filter *node_t, *new_node;
    (void)wifi_get_mcastf_lock();
    node_t = wm_wifi.start_list;
    if (wm_wifi.start_list == NULL)
    {
#if !CONFIG_MEM_POOLS
        new_node = OSA_MemoryAllocate(sizeof(mcast_filter));
#else
        new_node = (mcast_filter *)OSA_MemoryPoolAllocate(buf_32_MemoryPool);
#endif
        if (new_node == NULL)
        {
            (void)wifi_put_mcastf_lock();
            return -WM_FAIL;
        }
        (void)memcpy((void *)new_node->mac_addr, (const void *)mac_addr, MLAN_MAC_ADDR_LENGTH);
        new_node->next     = NULL;
        wm_wifi.start_list = new_node;
        (void)wifi_put_mcastf_lock();
        return WM_SUCCESS;
    }
    while (node_t->next != NULL && cmp_mac_addr(node_t->mac_addr, mac_addr))
    {
        node_t = node_t->next;
    }

    if (!cmp_mac_addr(node_t->mac_addr, mac_addr))
    {
        (void)wifi_put_mcastf_lock();
        return -WM_E_EXIST;
    }
#if !CONFIG_MEM_POOLS
    new_node = OSA_MemoryAllocate(sizeof(mcast_filter));
#else
    new_node = (mcast_filter *)OSA_MemoryPoolAllocate(buf_32_MemoryPool);
#endif
    if (new_node == NULL)
    {
        (void)wifi_put_mcastf_lock();
        return -WM_FAIL;
    }
    (void)memcpy((void *)new_node->mac_addr, (const void *)mac_addr, MLAN_MAC_ADDR_LENGTH);
    new_node->next = NULL;
    node_t->next   = new_node;
    (void)wifi_put_mcastf_lock();
    return WM_SUCCESS;
}

static int remove_mcast_ip(uint8_t *mac_addr)
{
    mcast_filter *curr_node, *prev_node;
    (void)wifi_get_mcastf_lock();
    if (wm_wifi.start_list == NULL)
    {
        (void)wifi_put_mcastf_lock();
        return -WM_FAIL;
    }
    curr_node = wm_wifi.start_list->next;
    prev_node = wm_wifi.start_list;
    if (curr_node == NULL && cmp_mac_addr(prev_node->mac_addr, mac_addr))
    {
#if !CONFIG_MEM_POOLS
        OSA_MemoryFree(prev_node);
#else
        OSA_MemoryPoolFree(buf_32_MemoryPool, prev_node);
#endif
        wm_wifi.start_list = NULL;
        (void)wifi_put_mcastf_lock();
        return WM_SUCCESS;
    }
    /* If search element is at first location */
    if (!cmp_mac_addr(prev_node->mac_addr, mac_addr))
    {
        wm_wifi.start_list = prev_node->next;
#if !CONFIG_MEM_POOLS
        OSA_MemoryFree(prev_node);
#else
        OSA_MemoryPoolFree(buf_32_MemoryPool, prev_node);
#endif
        (void)wifi_put_mcastf_lock();
        return WM_SUCCESS;
    }
    /* Find node in linked list */
    while (cmp_mac_addr(curr_node->mac_addr, mac_addr) && curr_node->next != NULL)
    {
        prev_node = curr_node;
        curr_node = curr_node->next;
    }
    if (!cmp_mac_addr(curr_node->mac_addr, mac_addr))
    {
        prev_node->next = curr_node->next;
#if !CONFIG_MEM_POOLS
        OSA_MemoryFree(curr_node);
#else
        OSA_MemoryPoolFree(buf_32_MemoryPool, curr_node);
#endif
        (void)wifi_put_mcastf_lock();
        return WM_SUCCESS;
    }
    (void)wifi_put_mcastf_lock();
    return -WM_FAIL;
}

static int make_filter_list(char *mlist, int maxlen)
{
    mcast_filter *node_t;
    int maddr_cnt = 0;
    (void)wifi_get_mcastf_lock();
    node_t = wm_wifi.start_list;
    while (node_t != NULL)
    {
        (void)memcpy((void *)mlist, (const void *)node_t->mac_addr, MLAN_MAC_ADDR_LENGTH);
        node_t = (struct mcast_filter *)node_t->next;
        mlist  = mlist + MLAN_MAC_ADDR_LENGTH;
        maddr_cnt++;
        if (maddr_cnt > (maxlen / 6U))
        {
            break;
        }
    }
    (void)wifi_put_mcastf_lock();
    return maddr_cnt;
}

void wifi_get_ipv4_multicast_mac(uint32_t ipaddr, uint8_t *mac_addr)
{
    int i = 0, j = 0;
    uint32_t mac_addr_r = 0x01005E;
    ipaddr              = ipaddr & 0x7FFFFFU;
    /* Generate Multicast Mapped Mac Address for IPv4
     * To get Multicast Mapped MAC address,
     * To calculate 6 byte Multicast-Mapped MAC Address.
     * 1) Fill higher 24-bits with IANA Multicast OUI (01-00-5E)
     * 2) Set 24th bit as Zero
     * 3) Fill lower 23-bits with from IP address (ignoring higher
     * 9bits).
     */
    for (i = 2; i >= 0; i--)
    {
        mac_addr[j] = (uint8_t)((char)(mac_addr_r >> 8 * i) & 0xFF);
        j++;
    }

    for (i = 2; i >= 0; i--)
    {
        mac_addr[j] = (uint8_t)((char)(ipaddr >> 8 * i) & 0xFF);
        j++;
    }
}

#if CONFIG_IPV6
void wifi_get_ipv6_multicast_mac(uint32_t ipaddr, uint8_t *mac_addr)
{
    int i = 0, j = 0;
    uint32_t mac_addr_r = 0x3333;
    /* Generate Multicast Mapped Mac Address for IPv6
     * To get Multicast Mapped MAC address,
     * To calculate 6 byte Multicast-Mapped MAC Address.
     * 1) Fill higher 16-bits with IANA Multicast OUI (33-33)
     * 2) Fill lower 24-bits with from IP address
     */
    for (i = 1; i >= 0; i--)
    {
        mac_addr[j] = (char)(mac_addr_r >> 8 * i) & 0xFF;
        j++;
    }

    for (i = 3; i >= 0; i--)
    {
        mac_addr[j] = (char)(ipaddr >> 8 * i) & 0xFF;
        j++;
    }
}
#endif /* CONFIG_IPV6 */

int wifi_add_mcast_filter(uint8_t *mac_addr)
{
    char mlist[MAX_MCAST_LEN] = {0};
    int len, ret;
    /* If MAC address is 00:11:22:33:44:55,
     * then pass mac_addr array in following format:
     * mac_addr[0] = 00
     * mac_addr[1] = 11
     * mac_addr[2] = 22
     * mac_addr[3] = 33
     * mac_addr[4] = 44
     * mac_addr[5] = 55
     */
    wifi_d("wifi_add_mcast_filter mac_addr: %02x:%02x:%02x:%02x:%02x:%02x\n", mac_addr[0], mac_addr[1], mac_addr[2],
           mac_addr[3], mac_addr[4], mac_addr[5]);
    (void)memset(&mlist, 0x00, MAX_MCAST_LEN);
    ret = add_mcast_ip(mac_addr);
    if (ret != WM_SUCCESS)
    {
        return ret;
    }
    len = make_filter_list(mlist, (int)MAX_MCAST_LEN);
    return wifi_set_mac_multicast_addr(mlist, (t_u32)len);
}

int wifi_remove_mcast_filter(uint8_t *mac_addr)
{
    char mlist[MAX_MCAST_LEN];
    int len, ret;
    /* If MAC address is 00:11:22:33:44:55,
     * then pass mac_addr array in following format:
     * mac_addr[0] = 00
     * mac_addr[1] = 11
     * mac_addr[2] = 22
     * mac_addr[3] = 33
     * mac_addr[4] = 44
     * mac_addr[5] = 55
     */

    (void)memset(&mlist, 0x00, MAX_MCAST_LEN);
    ret = remove_mcast_ip(mac_addr);
    if (ret != WM_SUCCESS)
    {
        return ret;
    }
    len = make_filter_list(mlist, (int)MAX_MCAST_LEN);
    ret = wifi_set_mac_multicast_addr(mlist, (uint32_t)len);
    return ret;
}

void wifi_remove_all_mcast_filter(uint8_t need_lock)
{
    mcast_filter *node = NULL;

    if (wm_wifi.start_list == NULL)
        return;

    if (need_lock)
        wifi_get_mcastf_lock();

    while (wm_wifi.start_list)
    {
        node               = wm_wifi.start_list;
        wm_wifi.start_list = node->next;
#if !CONFIG_MEM_POOLS
        OSA_MemoryFree(node);
#else
        OSA_MemoryPoolFree(buf_32_MemoryPool, node);
#endif
    }

    if (need_lock)
        wifi_put_mcastf_lock();
}

static struct wifi_scan_result2 common_desc;
int wifi_get_scan_result(unsigned int index, struct wifi_scan_result2 **desc)
{
    (void)memset(&common_desc, 0x00, sizeof(struct wifi_scan_result2));
    int rv =
        wrapper_bssdesc_first_set((int)index, common_desc.bssid, &common_desc.is_ibss_bit_set, &common_desc.ssid_len,
                                  common_desc.ssid, &common_desc.Channel, &common_desc.RSSI, &common_desc.beacon_period,
                                  &common_desc.dtim_period, &common_desc.WPA_WPA2_WEP, &common_desc.wpa_mcstCipher,
                                  &common_desc.wpa_ucstCipher, &common_desc.rsn_mcstCipher, &common_desc.rsn_ucstCipher,
                                  &common_desc.ap_mfpc, &common_desc.ap_mfpr, &common_desc.ap_pwe);
    if (rv != WM_SUCCESS)
    {
        wifi_e("wifi_get_scan_result failed");
        return rv;
    }

    /* Country info not populated */
    rv = wrapper_bssdesc_second_set((int)index, &common_desc.phtcap_ie_present, &common_desc.phtinfo_ie_present,
#if CONFIG_11AC
                                    & common_desc.pvhtcap_ie_present,
#endif
#if CONFIG_11AX
                                    & common_desc.phecap_ie_present,
#endif
                                    & common_desc.wmm_ie_present, &common_desc.band, &common_desc.wps_IE_exist,
                                    &common_desc.wps_session, &common_desc.wpa2_entp_IE_exist,
#if CONFIG_11R
                                    & common_desc.mdid,
#endif
#if CONFIG_11K
                                    & common_desc.neighbor_report_supported,
#endif
#if CONFIG_11V
                                    & common_desc.bss_transition_supported,
#endif
                                    & common_desc.trans_mode, common_desc.trans_bssid, &common_desc.trans_ssid_len,
                                    common_desc.trans_ssid
#if CONFIG_DRIVER_MBO
                                    ,
                                    &common_desc.mbo_assoc_disallowed
#endif
                                   );

    if (rv != WM_SUCCESS)
    {
        wifi_e("wifi_get_scan_result failed");
        return rv;
    }

    *desc = &common_desc;

    return WM_SUCCESS;
}

int wifi_register_event_queue(osa_msgq_handle_t event_queue)
{
    if (event_queue == MNULL)
    {
        return -WM_E_INVAL;
    }

    if (wm_wifi.wlc_mgr_event_queue != NULL)
    {
        return -WM_FAIL;
    }

    wm_wifi.wlc_mgr_event_queue = event_queue;
    return WM_SUCCESS;
}

int wifi_unregister_event_queue(osa_msgq_handle_t event_queue)
{
    if ((wm_wifi.wlc_mgr_event_queue == MNULL) || wm_wifi.wlc_mgr_event_queue != event_queue)
    {
        return -WM_FAIL;
    }

    wm_wifi.wlc_mgr_event_queue = NULL;
    return WM_SUCCESS;
}


int wifi_get_wpa_ie_in_assoc(uint8_t *wpa_ie)
{
    return wrapper_get_wpa_ie_in_assoc(wpa_ie);
}

#define WL_ID_WIFI_MAIN_LOOP "wifi_drv_task"
void wm_wifi_event(void *sg, int len, int type)
{
    struct wifi_mq event_msg;
    event_msg.len = len;
    event_msg.data = sg;
    event_msg.type = type;
    struct skw_msg *msg = (struct skw_msg *)(sg + 16);
    if (SKW_EVENT_CREDIT_UPDATE == msg->id)
    {
        //wifi_d("wm_wifi_event: SKW_EVENT_CREDIT_UPDATE\n");
        goto end;

    }
    else if (SKW_EVENT_BA_ACTION == msg->id)
    {
        // void skw_wifi_event_ba_action(void *data);
        // skw_wifi_event_ba_action(msg + 8);
        struct skw_ba_action *ba = (struct skw_ba_action *)(sg + 8 + 16);
        // wifi_d("%s %d ba->action=%x ba->tid=%x ba->status_code=%x,ba->win_size=0x%x\n", __func__, __LINE__,
        //            ba->action, ba->tid, ba->status_code, ba->win_size);
        goto end;

    }
    if (7 == type)
    {
        struct skw_rx_desc *rx_desc = (struct skw_rx_desc *)event_msg.data + 4;
        // wifi_d("%s %d retry_frame=%d ,sn=%d msdu_len=%d\n", __func__, __LINE__,rx_desc->retry_frame,
        //             rx_desc->sn,rx_desc->msdu_len);
        wifi_low_level_input(0/*STA MODE*/, event_msg.data + sizeof(struct skw_rx_desc),
                             event_msg.len - 18);
        goto end;
        return;
    }
    else if (6 == type)
    {
        struct skw_msg *msg = (struct skw_msg *)(sg + 16);
        //wifi_d("%s %d msg->type=%d\n", __func__, __LINE__,msg->type);
        if (msg->type == SKW_MSG_EVENT)//如果是event消息就响应
        {
            (void)wifi_handle_fw_event(msg);
        }
        else if (msg->type == SKW_MSG_CMD_ACK)//如果是cmd消息就响应
        {
            //msg 已经去掉了前面16字节的头部信息
            //(void)wifi_process_cmd_response(msg);
            wifi_update_last_cmd_sent_ms();
            (void)wifi_put_command_resp_sem();//告诉发送cmd完成了cmd指令 收到了ACK
        }
        else
        {
            /* Do Nothing */
        }
        goto end;
    }
    int ret = OSA_MsgQPut((osa_msgq_handle_t)wm_wifi.io_events, &event_msg);//到对端线程去释放
    if (ret != 0)
    {
        wifi_d("spi sig send queue failed\n");
    }
end:
    //rt_free(event_msg.data);
    return;
}
#include "skw_type_msg.h"
static uint8_t rx_wifi_data[2048] = {0};
static void wifi_drv_task(void *argv)//需要将该函数与希微的RX底层进行结合起来，并识别消息的类型，以及时给到反馈
{
    osa_status_t status;
    struct skw_msg *msg;
    struct wifi_mq event_msg;
    rt_memset((void *)&msg, 0, sizeof(struct skw_msg));

    /* Main Loop */
    while (true)
    {
        //wifi_d("%s %d\n", __func__, __LINE__);
        status = OSA_MsgQGet((osa_msgq_handle_t)wm_wifi.io_events, &event_msg, osaWaitForever_c);//收到sdio的中断队列消息
        //wifi_d("%s %d data=%p len=%d\n", __func__, __LINE__,event_msg.data,event_msg.len,event_msg.type);

        if (status == KOSA_StatusSuccess)
        {
            if (event_msg.type == 6)
            {
                char *buff = rt_malloc(event_msg.len);
                rt_memcpy(buff, event_msg.data + 4, event_msg.len);//去掉头部
                //rt_free(event_msg.data);
                msg = (struct skw_msg *)(buff + 12);
                //wifi_d("%s %d msg->type=%d\n", __func__, __LINE__,msg->type);
                if (msg->type == SKW_MSG_EVENT)//如果是event消息就响应
                {

                    (void)wifi_handle_fw_event(msg);
                    /*
                    * Free the buffer after the event is
                    * handled.
                    */
                    if (buff != NULL)
                    {
                        //wifi_free_eventbuf(msg.data);

                    }
                }
                else if (msg->type == SKW_MSG_CMD_ACK)//如果是cmd消息就响应
                {
                    //msg 已经去掉了前面16字节的头部信息
                    //(void)wifi_process_cmd_response(msg);
                    wifi_update_last_cmd_sent_ms();
                    (void)wifi_put_command_resp_sem();//告诉发送cmd完成了cmd指令 收到了ACK
                }
                else
                {
                    /* Do Nothing */
                }

                // wakelock_put(WL_ID_WIFI_MAIN_LOOP);
                rt_free(buff);
            }
            else if (event_msg.type == 7)
            {
                //char* buff = rt_malloc(event_msg.len);
                rt_memcpy(&rx_wifi_data, event_msg.data, event_msg.len + 4);
                struct skw_rx_desc *rx_desc = (struct skw_rx_desc *)&rx_wifi_data;
                wifi_d("%s %d sizeof(struct skw_rx_desc)=%d\n", __func__, __LINE__, sizeof(struct skw_rx_desc));
                wifi_low_level_input(0/*STA MODE*/, &rx_wifi_data[sizeof(struct skw_rx_desc)],
                                     event_msg.len - sizeof(struct skw_rx_desc));
                //rt_free(buff);
            }
        }
    }
}

#ifndef RW610
#define WL_ID_WIFI_CORE_INPUT "wifi_core_task"
/**
 * This function should be called when a packet is ready to be read
 * from the interface.
 */
static void wifi_core_task(void *argv)
{
    OSA_SR_ALLOC();
#ifndef __ZEPHYR__
    osa_event_flags_t flagsToWait = WIFI_EVENT_SDIO;
    osa_event_flags_t pSetFlags;
#endif

    for (;;)
    {
        OSA_ENTER_CRITICAL();

        /* Allow interrupt handler to deliver us a packet */
        g_txrx_flag = true;
        //      SDIOC_IntMask(SDIOC_INT_CDINT, UNMASK);
        //      SDIOC_IntSigMask(SDIOC_INT_CDINT, UNMASK);
#ifndef RW610
        //sdio_enable_interrupt();
#endif

        OSA_EXIT_CRITICAL();

#ifdef __ZEPHYR__
        (void)OSA_EventNotifyGet(osaWaitForever_c);
#else
        /* Wait till we receive a packet from SDIO */
        (void)OSA_EventWait((osa_event_handle_t)wm_wifi.wifi_event_Handle, flagsToWait, false, osaWaitForever_c,
                            &pSetFlags);

        if ((pSetFlags & WIFI_EVENT_SDIO) == 0U)
        {
            continue;
        }
#endif

        // wakelock_get(WL_ID_WIFI_CORE_INPUT);

#if defined(RW610)
        (void)wifi_imu_lock();
#else
        /* Protect the SDIO from other parallel activities */
        (void)wifi_sdio_lock();

        (void)wlan_process_int_status(mlan_adap);
#endif

#if defined(RW610)
        wifi_imu_unlock();
#else
        wifi_sdio_unlock();
#endif
        // wakelock_put(WL_ID_WIFI_CORE_INPUT);
    } /* for ;; */
}
#endif
void wifi_user_scan_config_cleanup(void)
{
    if (wm_wifi.g_user_scan_cfg != NULL)
    {
#if !CONFIG_MEM_POOLS
        OSA_MemoryFree((void *)wm_wifi.g_user_scan_cfg);
#else
        OSA_MemoryPoolFree(buf_768_MemoryPool, wm_wifi.g_user_scan_cfg);
#endif
        wm_wifi.g_user_scan_cfg = NULL;
    }
}

void wifi_scan_stop(void)
{
    wm_wifi.scan_stop = true;
    while (scan_thread_in_process)
    {
        /* wait for scan task done */
        OSA_TimeDelay(1000);
    }
    wm_wifi.scan_stop = false;
}

/**
 * This function should be called when scan command is ready
 *
 */
static void wifi_scan_task(void *argv)
{
    mlan_status rv;
#ifndef __ZEPHYR__
    osa_event_flags_t flagsToWait = WIFI_EVENT_SCAN;
    osa_event_flags_t pSetFlags;
#endif

    for (;;)
    {
#ifdef __ZEPHYR__
        (void)OSA_EventNotifyGet(osaWaitForever_c);
#else
        /* Wait till we receive scan command */
        (void)OSA_EventWait((osa_event_handle_t)wm_wifi.wifi_event_Handle, flagsToWait, false, osaWaitForever_c,
                            &pSetFlags);

        OSA_EventClear((osa_event_handle_t)wm_wifi.wifi_event_Handle, WIFI_EVENT_SCAN);

        if ((pSetFlags & WIFI_EVENT_SCAN) == 0U)
        {
            continue;
        }
#endif
        wifi_d("%s %d \n", __func__, __LINE__);
        if (wm_wifi.scan_stop == true)
        {
            wm_wifi.scan_stop = false;
            wifi_user_scan_config_cleanup();
            break;
        }

        scan_thread_in_process = true;
        if (wm_wifi.g_user_scan_cfg != NULL)
        {
#if CONFIG_WPA_SUPP
            (void)wifi_event_completion(WIFI_EVENT_SCAN_START, WIFI_EVENT_REASON_SUCCESS, NULL);
#endif
            wifi_d("%s %d \n", __func__, __LINE__);
            rv = wlan_scan_networks((mlan_private *)mlan_adap->priv[0], NULL, wm_wifi.g_user_scan_cfg);
            if (rv != MLAN_STATUS_SUCCESS)
            {
                wifi_d("%s %d \n", __func__, __LINE__);
                wifi_user_scan_config_cleanup();
                (void)wifi_event_completion(WIFI_EVENT_SCAN_RESULT, WIFI_EVENT_REASON_FAILURE, NULL);
            }
        }
        wifi_d("%s %d \n", __func__, __LINE__);
        scan_thread_in_process = false;
    } /* for ;; */
    wifi_d("%s %d \n", __func__, __LINE__);
    while (true)
    {
        OSA_TimeDelay(60000);
    }
    wifi_d("%s %d \n", __func__, __LINE__);
}

static void wifi_powersave_task(void *data)
{
    osa_status_t status;
    struct wifi_message msg;

    while (1)
    {
        status = OSA_MsgQGet((osa_msgq_handle_t)wm_wifi.powersave_queue, &msg, osaWaitForever_c);
        if (status == KOSA_StatusSuccess)
        {
            switch (msg.event)
            {
            case WIFI_EVENT_SLEEP:
                wifi_event_completion(WIFI_EVENT_SLEEP, WIFI_EVENT_REASON_SUCCESS, NULL);
                break;
            default:
                wifi_w("got unknown message: %d", msg.event);
                break;
            }
        }
    }
}

#if CONFIG_FW_VDLL
/**
 *  @brief This function flushes all data
 *
 *  @param context      Reorder context pointer
 *
 *  @return             N/A
 */
static t_void wlan_vdll_complete(osa_timer_arg_t tmr_handle)
{
    mlan_adap->vdll_in_progress = MFALSE;
}
#endif

static void wifi_core_deinit(void);


static int wifi_core_init(void)
{
    osa_status_t status;
    int ret;

    if (wm_wifi.wifi_core_init_done != 0U)
    {
        return WM_SUCCESS;
    }

    status = OSA_MutexCreate((osa_mutex_handle_t)wm_wifi.command_lock);
    if (status != KOSA_StatusSuccess)
    {
        wifi_e("Create command_lock failed");
        goto fail;
    }

    status = OSA_EventCreate((osa_event_handle_t)wm_wifi.wifi_event_Handle, 1);
    if (status != KOSA_StatusSuccess)
    {
        wifi_e("Create event handle failed");
        goto fail;
    }

    status = OSA_SemaphoreCreateBinary((osa_semaphore_handle_t)wm_wifi.command_resp_sem);
    if (status != KOSA_StatusSuccess)
    {
        wifi_e("Create command resp sem failed");
        goto fail;
    }
    OSA_SemaphorePost((osa_semaphore_handle_t)wm_wifi.command_resp_sem);
    status = OSA_MutexCreate((osa_mutex_handle_t)wm_wifi.mcastf_mutex);
    if (status != KOSA_StatusSuccess)
    {
        wifi_e("Create mcastf mutex failed");
        goto fail;
    }
    /*
     * Take the cmd resp lock immediately so that we can later block on
     * it.
     */
    (void)wifi_get_command_resp_sem(osaWaitForever_c);

    status = OSA_MsgQCreate((osa_msgq_handle_t)wm_wifi.io_events, MAX_EVENTS, sizeof(struct bus_message));
    if (status != KOSA_StatusSuccess)
    {
        wifi_e("Create io events queue failed");
        goto fail;
    }

    ret = bus_register_event_queue((osa_msgq_handle_t)wm_wifi.io_events);
    if (ret != WM_SUCCESS)
    {
        wifi_e("Register io events queue failed");
        goto fail;
    }

    status = OSA_TaskCreate((osa_task_handle_t)wm_wifi.wifi_drv_task_Handle, OSA_TASK(wifi_drv_task), NULL);
    if (status != KOSA_StatusSuccess)
    {
        wifi_e("Create wifi driver thread failed");
        goto fail;
    }

    ret = bus_register_data_input_function(&wifi_low_level_input);
    if (ret != WM_SUCCESS)
    {
        wifi_e("Register wifi low level input failed");
        goto fail;
    }

    status = OSA_TaskCreate((osa_task_handle_t)wm_wifi.wifi_scan_task_Handle, OSA_TASK(wifi_scan_task), NULL);
    if (status != KOSA_StatusSuccess)
    {
        wifi_e("Create wifi scan thread failed");
        goto fail;
    }
#ifndef RW610
    status = OSA_TaskCreate((osa_task_handle_t)wm_wifi.wifi_core_task_Handle, OSA_TASK(wifi_core_task), NULL);
    if (status != KOSA_StatusSuccess)
    {
        wifi_e("Create stack dispatcher thread failed");
        goto fail;
    }
#endif

    wm_wifi.wifi_core_init_done = 1;

#if CONFIG_WMM
    ret = wifi_wmm_buf_pool_init(&outbuf_arr[0][0]);
    if (ret != WM_SUCCESS)
    {
        wifi_e("Unable to init wmm buffer pool");
        goto fail;
    }

    ret = wifi_bypass_txq_init();
    if (ret != WM_SUCCESS)
    {
        wifi_e("Init bypass txq failed\r\n");
        goto fail;
    }

    status = OSA_SemaphoreCreateBinary((osa_semaphore_handle_t)txbuf_sem);
    if (status != KOSA_StatusSuccess)
    {
        wifi_e("Create txbuf sem failed");
        return ret;
    }

    /* Semaphore to protect wmm data parameters */
    status = OSA_SemaphoreCreateBinary((osa_semaphore_handle_t)wm_wifi.tx_data_sem);
    if (status != KOSA_StatusSuccess)
    {
        PRINTF("Create tx data sem failed");
        goto fail;
    }
    OSA_SemaphorePost((osa_semaphore_handle_t)wm_wifi.tx_data_sem);
#ifdef __ZEPHYR__
    status = OSA_MsgQCreate((osa_msgq_handle_t)wm_wifi.tx_data, MAX_EVENTS, sizeof(struct bus_message));
    if (status != KOSA_StatusSuccess)
    {
        wifi_e("Create tx_data queue failed");
        goto fail;
    }
#endif

    status = OSA_TaskCreate((osa_task_handle_t)wm_wifi.wifi_drv_tx_task_Handle, OSA_TASK(wifi_drv_tx_task), NULL);
    if (status != KOSA_StatusSuccess)
    {
        PRINTF("Create tx data thread failed");
        goto fail;
    }
#endif

    status = OSA_MsgQCreate((osa_msgq_handle_t)wm_wifi.powersave_queue, MAX_EVENTS, sizeof(struct bus_message));
    if (status != KOSA_StatusSuccess)
    {
        PRINTF("Create power save queue failed");
        goto fail;
    }

    status = OSA_TaskCreate((osa_task_handle_t)wm_wifi.wifi_powersave_task_Handle, OSA_TASK(wifi_powersave_task), NULL);
    if (status != KOSA_StatusSuccess)
    {
        PRINTF("Create power save thread failed");
        goto fail;
    }

#if CONFIG_CSI
    /* Semaphore to protect data parameters */
    status = OSA_SemaphoreCreateBinary((osa_semaphore_handle_t)csi_buff_stat.csi_data_sem);
    if (status != KOSA_StatusSuccess)
    {
        PRINTF("Create usb data sem failed");
        goto fail;
    }
    OSA_SemaphorePost((osa_semaphore_handle_t)csi_buff_stat.csi_data_sem);
#endif

#if CONFIG_ECSA
    /* Semaphore to wait ECSA complete */
    status = OSA_SemaphoreCreateBinary((osa_semaphore_handle_t)ecsa_status_control.ecsa_sem);
    if (status != KOSA_StatusSuccess)
    {
        PRINTF("Create ecsa sem failed");
        goto fail;
    }
#endif

#if CONFIG_FW_VDLL
    (void)mlan_adap->callbacks.moal_init_timer(mlan_adap->pmoal_handle, &mlan_adap->vdll_timer, wlan_vdll_complete,
            NULL);
#endif

    wm_wifi.wifi_core_init_done = 1;

#if defined(SD8801) || defined(RW610)
    wifi_uap_set_bandwidth(BANDWIDTH_20MHZ);
#else
    wifi_uap_set_bandwidth(BANDWIDTH_40MHZ);
#endif

    return WM_SUCCESS;

fail:

    wifi_core_deinit();

    return -WM_FAIL;
}

static void wifi_core_deinit(void)
{
    int i = 0;

    mlan_adap->in_reset = true;
    for (i = 0; i < (int)(MIN(MLAN_MAX_BSS_NUM, mlan_adap->priv_num)); i++)
    {
        if (mlan_adap->priv[i])
        {
            wlan_clean_txrx(mlan_adap->priv[i]);
            (void)OSA_MutexDestroy((osa_mutex_handle_t)mlan_adap->priv[i]->tx_ba_stream_tbl_lock);
            (void)OSA_SemaphoreDestroy((osa_semaphore_handle_t)mlan_adap->priv[i]->rx_reorder_tbl_lock);
#if CONFIG_WMM
            wlan_ralist_deinit_enh(mlan_adap->priv[i]);
#endif
        }
    }

    wm_wifi.wifi_core_init_done = 0;

    bus_deregister_event_queue();
    bus_deregister_data_input_funtion();

    (void)OSA_MsgQDestroy((osa_msgq_handle_t)wm_wifi.io_events);

    (void)OSA_MsgQDestroy((osa_msgq_handle_t)wm_wifi.powersave_queue);

#if CONFIG_WMM
#ifdef __ZEPHYR__
    (void)OSA_MsgQDestroy((osa_msgq_handle_t)wm_wifi.tx_data);
#endif
    wifi_wmm_buf_pool_deinit();
    wifi_bypass_txq_deinit();

    (void)OSA_SemaphoreDestroy((osa_semaphore_handle_t)txbuf_sem);
#endif

    wifi_remove_all_mcast_filter(0);

    (void)OSA_MutexDestroy((osa_mutex_handle_t)wm_wifi.mcastf_mutex);
    (void)OSA_SemaphoreDestroy((osa_semaphore_handle_t)wm_wifi.command_resp_sem);
#if CONFIG_WMM
    (void)OSA_SemaphoreDestroy((osa_semaphore_handle_t)wm_wifi.tx_data_sem);
#endif
    (void)OSA_MutexDestroy((osa_mutex_handle_t)wm_wifi.command_lock);
    (void)OSA_EventDestroy((osa_event_handle_t)wm_wifi.wifi_event_Handle);
#ifndef RW610
    (void)OSA_TaskDestroy((osa_task_handle_t)wm_wifi.wifi_drv_task_Handle);
    (void)OSA_TaskDestroy((osa_task_handle_t)wm_wifi.wifi_core_task_Handle);
    (void)OSA_TaskDestroy((osa_task_handle_t)wm_wifi.wifi_scan_task_Handle);
    (void)OSA_TaskDestroy((osa_task_handle_t)wm_wifi.wifi_powersave_task_Handle);
#if CONFIG_WMM
    (void)OSA_TaskDestroy((osa_task_handle_t)wm_wifi.wifi_drv_tx_task_Handle);
#endif
#else
    wm_wifi.cmd_resp_priv   = NULL;
    wm_wifi.cmd_resp_ioctl  = NULL;
    wm_wifi.cmd_resp_status = 0;
    memset(&wm_wifi, 0x00, sizeof(wm_wifi));
#endif
#if CONFIG_CSI
    (void)OSA_SemaphoreDestroy((osa_semaphore_handle_t)csi_buff_stat.csi_data_sem);
#endif
#if CONFIG_ECSA
    (void)OSA_SemaphoreDestroy((osa_semaphore_handle_t)ecsa_status_control.ecsa_sem);
#endif
#if CONFIG_FW_VDLL
    (void)mlan_adap->callbacks.moal_stop_timer(mlan_adap->pmoal_handle, mlan_adap->vdll_timer);
    (void)mlan_adap->callbacks.moal_free_timer(mlan_adap->pmoal_handle, &mlan_adap->vdll_timer);
#endif
}

int wifi_init(const uint8_t *fw_start_addr, const size_t size)
{
    int ret = WM_SUCCESS;
    if (wm_wifi.wifi_init_done != 0U)
    {
        return WM_SUCCESS;
    }

    (void)memset(&wm_wifi, 0, sizeof(wm_wifi_t));

    wm_wifi.fw_start_addr = fw_start_addr;
    wm_wifi.size          = size;
    wifi_d("%s %d\n", __func__, __LINE__);
    ret = (int)sd_wifi_init(WLAN_TYPE_NORMAL, fw_start_addr, size);

    wifi_d("wifi_init: %d %d\n", wm_wifi.fw_start_addr, wm_wifi.size);


    ret = wifi_core_init();
    if (ret != WM_SUCCESS)
    {
        wifi_e("wifi_core_init failed. status code %d", ret);
        return ret;
    }
    skw_sdio_io_init();

    if (ret == WM_SUCCESS)
    {
        wm_wifi.wifi_init_done = 1;
    }

    return ret;
}

#if (CONFIG_WIFI_IND_DNLD)
static int wifi_reinit(uint8_t fw_reload)
{
    int ret = WM_SUCCESS;

    ret = (int)sd_wifi_reinit(WLAN_TYPE_NORMAL, wm_wifi.fw_start_addr, wm_wifi.size, fw_reload);
#if CONFIG_WIFI_IND_RESET
    wifi_ind_reset_stop();
#endif
    if (ret != WM_SUCCESS)
    {
        if (ret != MLAN_STATUS_FW_DNLD_SKIP)
        {
            wifi_e("sd_wifi_reinit failed. status code %d", ret);
        }
        switch (ret)
        {
        case MLAN_CARD_CMD_TIMEOUT:
        case MLAN_CARD_NOT_DETECTED:
            ret = -WIFI_ERROR_CARD_NOT_DETECTED;
            break;
        case MLAN_STATUS_FW_DNLD_FAILED:
            ret = -WIFI_ERROR_FW_DNLD_FAILED;
            break;
        case MLAN_STATUS_FW_NOT_DETECTED:
            ret = -WIFI_ERROR_FW_NOT_DETECTED;
            break;
        case MLAN_STATUS_FW_NOT_READY:
            ret = -WIFI_ERROR_FW_NOT_READY;
            break;
        case MLAN_STATUS_FW_DNLD_SKIP:
            ret = WM_SUCCESS;
            break;
        default:
            PRINTM(MINFO, "Unexpected MLAN FW Status \n");
            ret = -WM_FAIL;
            break;
        }
    }
#ifndef RW610
    else
    {
        ret = (int)sd_wifi_post_init(WLAN_TYPE_NORMAL);
        if (ret != WM_SUCCESS)
        {
            wifi_e("sd_wifi_post_init failed. status code %d", ret);
            return ret;
        }
    }
#endif

    return ret;
}
#endif

#ifndef RW610
int wifi_init_fcc(const uint8_t *fw_start_addr, const size_t size)
{
    if (wm_wifi.wifi_init_done != 0U)
    {
        return WM_SUCCESS;
    }
    int ret = (int)sd_wifi_init(WLAN_TYPE_FCC_CERTIFICATION, fw_start_addr, size);
#if 0
    int ret = (int)sd_wifi_init(WLAN_TYPE_FCC_CERTIFICATION, fw_start_addr, size);
    if (ret != 0)
    {
        wifi_e("sd_wifi_init failed. status code %d", ret);
        switch (ret)
        {
        case MLAN_CARD_CMD_TIMEOUT:
        case MLAN_CARD_NOT_DETECTED:
            ret = -WIFI_ERROR_CARD_NOT_DETECTED;
            break;
        case MLAN_STATUS_FW_DNLD_FAILED:
            ret = -WIFI_ERROR_FW_DNLD_FAILED;
            break;
        case MLAN_STATUS_FW_NOT_DETECTED:
            ret = -WIFI_ERROR_FW_NOT_DETECTED;
            break;
        case MLAN_STATUS_FW_NOT_READY:
            ret = -WIFI_ERROR_FW_NOT_READY;
            break;
        default:
            wifi_d("sd_wifi_init unexpected MLAN Status %d", ret);
            break;
        }
        return ret;
    }
#endif
    int skw_sdio_io_init(void);
    skw_sdio_io_init();//希微wifi的初始化
    ret = wifi_core_init();
    if (ret != 0)
    {
        wifi_e("wifi_core_init failed. status code %d", ret);
    }

    if (ret == WM_SUCCESS)
    {
        wm_wifi.wifi_init_done = 1;
    }
#if 0//ndef RW610
    ret = (int)sd_wifi_post_init(WLAN_TYPE_FCC_CERTIFICATION);
    if (ret != WM_SUCCESS)
    {
        wifi_e("sd_wifi_post_init failed. status code %d", ret);
        return ret;
    }
#endif

    return ret;
}
#endif

void wifi_deinit(void)
{
    if (wm_wifi.wifi_init_done == 0U)
    {
        return;
    }

    wm_wifi.wifi_init_done = 0;

    wifi_core_deinit();

#if defined(RW610)
    imu_wifi_deinit();
#else
    sd_wifi_deinit();
#endif
}

#ifdef RW610
bool wifi_fw_is_hang(void)
{
    if (mlan_adap && mlan_adap->bus_ops.fw_is_hang)
        return mlan_adap->bus_ops.fw_is_hang();
    return false;
}

void wifi_destroy_wifidriver_tasks(void)
{
#if CONFIG_WMM
    (void)OSA_TaskDestroy((osa_task_handle_t)wm_wifi.wifi_drv_tx_task_Handle);
#endif
    (void)OSA_TaskDestroy((osa_task_handle_t)wm_wifi.wifi_drv_task_Handle);
#ifndef RW610
    (void)OSA_TaskDestroy((osa_task_handle_t)wm_wifi.wifi_core_task_Handle);
#endif
    (void)OSA_TaskDestroy((osa_task_handle_t)wm_wifi.wifi_scan_task_Handle);
    (void)OSA_TaskDestroy((osa_task_handle_t)wm_wifi.wifi_powersave_task_Handle);

    imu_uninstall_callback();
}
#endif
void wifi_set_tx_status(t_u8 status)
{
    wifi_tx_status = status;
}

void wifi_set_rx_status(t_u8 status)
{
    wifi_rx_status = status;
}

void wifi_set_packet_retry_count(const int count)
{
    retry_attempts = count;
}

void wifi_sta_ampdu_tx_enable(void)
{
    sta_ampdu_tx_enable = true;
}

void wifi_sta_ampdu_tx_disable(void)
{
    sta_ampdu_tx_enable = false;
}

void wifi_sta_ampdu_tx_enable_per_tid(t_u8 tid)
{
    sta_ampdu_tx_enable_per_tid = tid;
}

t_u8 wifi_sta_ampdu_tx_enable_per_tid_is_allowed(t_u8 tid)
{
    if ((sta_ampdu_tx_enable_per_tid >> tid) & 0x01)
        return MTRUE;
    else
        return MFALSE;
}

void wifi_sta_ampdu_rx_enable(void)
{
    sta_ampdu_rx_enable = true;
}

void wifi_sta_ampdu_rx_disable(void)
{
    sta_ampdu_rx_enable = false;
}

void wifi_sta_ampdu_rx_enable_per_tid(t_u8 tid)
{
    sta_ampdu_rx_enable_per_tid = tid;
}

t_u8 wifi_sta_ampdu_rx_enable_per_tid_is_allowed(t_u8 tid)
{
    if ((sta_ampdu_rx_enable_per_tid >> tid) & 0x01)
        return MTRUE;
    else
        return MFALSE;
}

void wifi_uap_ampdu_tx_enable(void)
{
    uap_ampdu_tx_enable = true;
}

void wifi_uap_ampdu_tx_disable(void)
{
    uap_ampdu_tx_enable = false;
}

void wifi_uap_ampdu_tx_enable_per_tid(t_u8 tid)
{
    uap_ampdu_tx_enable_per_tid = tid;
}

t_u8 wifi_uap_ampdu_tx_enable_per_tid_is_allowed(t_u8 tid)
{
    if ((uap_ampdu_tx_enable_per_tid >> tid) & 0x01)
        return MTRUE;
    else
        return MFALSE;
}

void wifi_uap_ampdu_rx_enable(void)
{
    uap_ampdu_rx_enable = true;
}

void wifi_uap_ampdu_rx_disable(void)
{
    uap_ampdu_rx_enable = false;
}

void wifi_uap_ampdu_rx_enable_per_tid(t_u8 tid)
{
    uap_ampdu_rx_enable_per_tid = tid;
}

t_u8 wifi_uap_ampdu_rx_enable_per_tid_is_allowed(t_u8 tid)
{
    if ((uap_ampdu_rx_enable_per_tid >> tid) & 0x01)
        return MTRUE;
    else
        return MFALSE;
}

#if ((FSL_USDHC_ENABLE_SCATTER_GATHER_TRANSFER) && FSL_USDHC_ENABLE_SCATTER_GATHER_TRANSFER > 0U)
int wifi_register_get_rxbuf_desc_callback(void *(*wifi_get_rxbuf_desc)(t_u16 rx_len))
{
    if (wm_wifi.wifi_get_rxbuf_desc != NULL)
    {
        return -WM_FAIL;
    }

    wm_wifi.wifi_get_rxbuf_desc = wifi_get_rxbuf_desc;

    return WM_SUCCESS;
}

void wifi_deregister_get_rxbuf_desc_callback(void)
{
    wm_wifi.wifi_get_rxbuf_desc = NULL;
}
#endif
//普通数据接收的回调函数
int wifi_register_data_input_callback(void (*data_input_callback)(const uint8_t interface,
                                      const uint8_t *buffer,
                                      const uint16_t len))
{
#if CONFIG_HEAP_DEBUG
    static bool mem_stat_sem_init = 0;
    int ret;
#endif
    if (wm_wifi.data_input_callback != NULL)
    {
        return -WM_FAIL;
    }

    wm_wifi.data_input_callback = data_input_callback;

#if CONFIG_HEAP_DEBUG
    /* Semaphore to protect os mem stat */
    if (!mem_stat_sem_init)
    {
        ret = OSA_SemaphoreCreateBinary((osa_semaphore_handle_t)os_mem_stat_sem);
        if (ret != WM_SUCCESS)
        {
            PRINTF("Create os mem stat sem failed");
            return -WM_FAIL;
        }
        OSA_SemaphorePost((osa_semaphore_handle_t)os_mem_stat_sem);
        mem_stat_sem_init = 1;
    }
#endif

    return WM_SUCCESS;
}

void wifi_deregister_data_input_callback(void)
{
    wm_wifi.data_input_callback = NULL;
}

int wifi_register_amsdu_data_input_callback(void (*amsdu_data_input_callback)(uint8_t interface,
        uint8_t *buffer,
        uint16_t len))
{
    if (wm_wifi.amsdu_data_input_callback != NULL)
    {
        return -WM_FAIL;
    }

    wm_wifi.amsdu_data_input_callback = amsdu_data_input_callback;

    return WM_SUCCESS;
}

void wifi_deregister_amsdu_data_input_callback(void)
{
    wm_wifi.amsdu_data_input_callback = NULL;
}

int wifi_register_deliver_packet_above_callback(void (*deliver_packet_above_callback)(void *rxpd,
        uint8_t interface,
        void *lwip_pbuf))
{
    if (wm_wifi.deliver_packet_above_callback != NULL)
    {
        return -WM_FAIL;
    }

    wm_wifi.deliver_packet_above_callback = deliver_packet_above_callback;

    return WM_SUCCESS;
}

void wifi_deregister_deliver_packet_above_callback(void)
{
    wm_wifi.deliver_packet_above_callback = NULL;
}

int wifi_register_wrapper_net_is_ip_or_ipv6_callback(bool (*wrapper_net_is_ip_or_ipv6_callback)(const t_u8 *buffer))
{
    if (wm_wifi.wrapper_net_is_ip_or_ipv6_callback != NULL)
    {
        return -WM_FAIL;
    }

    wm_wifi.wrapper_net_is_ip_or_ipv6_callback = wrapper_net_is_ip_or_ipv6_callback;

    return WM_SUCCESS;
}

void wifi_deregister_wrapper_net_is_ip_or_ipv6_callback(void)
{
    wm_wifi.wrapper_net_is_ip_or_ipv6_callback = NULL;
}

#if CONFIG_WPA_SUPP

void wpa_supp_handle_link_lost(mlan_private *priv)
{
    t_u8 broadcast_addr[ETH_ALEN]      = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    nxp_wifi_event_mlme_t *deauth_resp = &wm_wifi.mgmt_resp;
    IEEE80211_MGMT *mgmt               = (IEEE80211_MGMT *)deauth_resp->frame.frame;

    if (priv->bss_role == MLAN_BSS_ROLE_STA)
    {
        memset(mgmt, 0, sizeof(IEEE80211_MGMT));

        mgmt->frame_control            = SUBTYPE_DEAUTH;
        mgmt->duration                 = 0;
        mgmt->seq_ctrl                 = 0;
        mgmt->u.deauth_req.reason_code = WLAN_REASON_DEAUTH_LEAVING;

        memcpy((void *)mgmt->da, broadcast_addr, MLAN_MAC_ADDR_LENGTH);
        memcpy((void *)mgmt->sa, priv->curr_bss_params.bss_descriptor.mac_address, MLAN_MAC_ADDR_LENGTH);
        memcpy((void *)mgmt->bssid, priv->curr_bss_params.bss_descriptor.mac_address, MLAN_MAC_ADDR_LENGTH);

        deauth_resp->frame.frame_len = 26;

        priv->curr_bss_params.host_mlme = 0;
        priv->auth_flag                 = 0;

        if (wm_wifi.supp_if_callbk_fns->deauth_callbk_fn)
        {
            wm_wifi.supp_if_callbk_fns->deauth_callbk_fn(wm_wifi.if_priv, deauth_resp, deauth_resp->frame.frame_len);
        }
    }
}

/**
 *   @brief This function processes the 802.11 mgmt Frame
 *
 *   @param priv            A pointer to mlan_private
 *
 *   @param payload         A pointer to the received buffer
 *   @param payload_len     Length of the received buffer
 *   @param prx_pd          A pointer to RxPD
 *
 *   @return                MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static mlan_status wlan_process_802dot11_mgmt_pkt2(mlan_private *priv, t_u8 *payload, t_u32 payload_len, RxPD *prx_pd)
{
    // pmlan_adapter pmadapter = priv->adapter;
    // pmlan_callbacks pcb = &pmadapter->callbacks;
    mlan_status ret                   = MLAN_STATUS_SUCCESS;
    wlan_802_11_header *pieee_pkt_hdr = MNULL;
    t_u16 sub_type                    = 0;
    // t_u8 *event_buf = MNULL;
    // mlan_event *pevent = MNULL;
    t_u8 unicast     = 0;
    t_u8 broadcast[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

    struct skw_mgmt_hdr *skw_mgmt_hdr = (struct skw_mgmt_hdr *)prx_pd;
    //RT_ASSERT(0);
#if CONFIG_WIFI_EXTRA_DEBUG
    IEEE80211_MGMT *mgmt = MNULL;
#endif
    t_u8 band_config = skw_mgmt_hdr->band;//(prx_pd->rx_info & 0x3); /* Bit[1:0] 0: HALCHANBAND_BG, 1:HALCHANBAND_A, 2: HALCHANBAND_6E */
    t_u8 chan_num    = skw_mgmt_hdr->chan;//(prx_pd->rx_info & RXPD_CHAN_MASK) >>5; /* Bit[13: 5] Non zero channel number on which this packet is received */
    t_u8 category    = 0;
    t_u8 action_code = 0;
    wifi_d("%s %d band_config=%d chan_num=%d\n", __func__, __LINE__, band_config, chan_num);
#ifdef DOT1AS_SUPPORT
    struct timestamps tstamps;
#endif
    // t_u8 *sta_addr = NULL;
    // sta_node *sta_ptr = MNULL;
    // MrvlIETypes_MgmtFrameSet_t *tlv;
    // pmlan_buffer pmbuf;

    ENTER();
#if 0
    /* rx buffer read from data path, nothing with MAX_EVENT_SIZE */
    if (payload_len > (MAX_EVENT_SIZE - sizeof(mlan_event)))
    {
        wifi_d("Dropping large mgmt frame,len =%d", payload_len);
        LEAVE();
        return ret;
    }
#endif
    /* Check  packet type-subtype and compare with mgmt_passthru_mask
     * If event is needed to host, just eventify it */
    pieee_pkt_hdr = (wlan_802_11_header *)payload;
    sub_type      = IEEE80211_GET_FC_MGMT_FRAME_SUBTYPE(pieee_pkt_hdr->frm_ctl);//取高4位
    wifi_d("%s %d sub_type=%d\n", __func__, __LINE__, sub_type);
    if (0)//(((1 << sub_type) & priv->mgmt_frame_passthru_mask) == 0)
    {
        wifi_d("Dropping mgmt frame for subtype %d snr=%d.", sub_type, prx_pd->snr);
        LEAVE();
        return ret;
    }
    switch (sub_type)
    {
    case SUBTYPE_ASSOC_REQUEST:
    case SUBTYPE_REASSOC_REQUEST:
    case SUBTYPE_ASSOC_RESP:
        if (priv->uap_host_based)
        {
            if (!memcmp(pieee_pkt_hdr->addr3, priv->curr_addr, MLAN_MAC_ADDR_LENGTH))
            {
                wifi_d("wlan: HostMlme MICRO_AP_STA_ASSOC " MACSTR "", MAC2STR(pieee_pkt_hdr->addr2));

            }
            else
            {
                wifi_d("wlan: Drop MICRO_AP_STA_ASSOC " MACSTR " from unknown BSSID " MACSTR "\r\n",
                       MAC2STR(pieee_pkt_hdr->addr2), MAC2STR(pieee_pkt_hdr->addr3));
            }
        }
        unicast = MTRUE;
        break;
    case SUBTYPE_AUTH://11  0xb  AUTH的 req帧数据
        unicast = MTRUE;
        wifi_d("wlan: HostMlme Auth received from " MACSTR "\r\n", MAC2STR(pieee_pkt_hdr->addr2));
        wifi_d("%s %d sub_type=%d priv->bss_role=%d ,host_mlme=%d\n", __func__, __LINE__,
               sub_type, priv->bss_role, priv->curr_bss_params.host_mlme);
        if (priv->bss_role == MLAN_BSS_ROLE_STA)
        {
            if (priv->curr_bss_params.host_mlme)
            {
                if (priv->auth_flag & HOST_MLME_AUTH_PENDING)
                {
                    if (priv->auth_alg != WLAN_AUTH_SAE)
                    {
                        priv->auth_flag &= ~HOST_MLME_AUTH_PENDING;
                        priv->auth_flag |= HOST_MLME_AUTH_DONE;
                    }
                }
            }
        }
        wifi_d("%s %d sub_type=%d priv->auth_flag=%d\n", __func__, __LINE__,
               sub_type, priv->auth_flag);
        break;
    case SUBTYPE_PROBE_RESP:
        unicast = MTRUE;
        break;
    case SUBTYPE_DISASSOC:
    case SUBTYPE_DEAUTH:
        if (memcmp(pieee_pkt_hdr->addr1, broadcast, MLAN_MAC_ADDR_LENGTH))
            unicast = MTRUE;
        if (priv->uap_host_based)
        {
            if (!memcmp(pieee_pkt_hdr->addr3, priv->curr_addr, MLAN_MAC_ADDR_LENGTH))
            {
#if CONFIG_WIFI_EXTRA_DEBUG
                mgmt = (IEEE80211_MGMT *)payload;
#endif

                wifi_d("wlan: HostMlme Deauth Receive from " MACSTR " reason code: %d\r\n",
                       MAC2STR(pieee_pkt_hdr->addr2), mgmt->u.deauth_req.reason_code);

#if 0
                sta_addr = OSA_MemoryAllocate(MLAN_MAC_ADDR_LENGTH);
                if (sta_addr == MNULL)
                {
                    wifi_w("No mem. Cannot process MAC address from deauth");
                    LEAVE();
                    return ret;
                }

                (void)memcpy((void *)sta_addr, (const void *)pieee_pkt_hdr->addr2, MLAN_MAC_ADDR_LENGTH);
                if (wifi_event_completion(WIFI_EVENT_UAP_CLIENT_DEAUTH, WIFI_EVENT_REASON_SUCCESS, sta_addr) !=
                        WM_SUCCESS)
                {
                    /* If fail to send message on queue, free allocated memory ! */
                    OSA_MemoryFree((void *)sta_addr);
                }
#endif
            }
            else
            {
                LEAVE();
                return ret;
            }
        }
        if (priv->bss_role == MLAN_BSS_ROLE_STA)
        {
            /* check receiving broadcast deauth frame from other BSSID and drop the deauh frame */
            t_u8 zero_mac[MLAN_MAC_ADDR_LENGTH] = {0};
            if ((memcmp(pieee_pkt_hdr->addr3, (t_u8 *)priv->curr_bss_params.bss_descriptor.mac_address,
                        MLAN_MAC_ADDR_LENGTH) &&
                    memcmp(zero_mac, (t_u8 *)priv->curr_bss_params.bss_descriptor.mac_address,
                           MLAN_MAC_ADDR_LENGTH)) ||
                    memcmp(pieee_pkt_hdr->addr3, (t_u8 *)priv->curr_bss_params.attemp_bssid,
                           MLAN_MAC_ADDR_LENGTH))
            {
                wifi_d("Dropping Deauth frame from other bssid: type=%d " MACSTR "\r\n", sub_type,
                       MAC2STR(pieee_pkt_hdr->addr3));
                LEAVE();
                return ret;
            }
            wifi_d("wlan: HostMlme Disconnected: sub_type=%d\n", sub_type);
#if 0
            pmadapter->pending_disconnect_priv = priv;
            wlan_recv_event(
                priv, MLAN_EVENT_ID_DRV_DEFER_HANDLING,
                MNULL);
#endif
        }
        break;
    case SUBTYPE_ACTION:
        category    = *(payload + sizeof(wlan_802_11_header));
        action_code = *(payload + sizeof(wlan_802_11_header) + 1);
        /*wpa_supplicant only deals with those action frame below*/
        if (category != IEEE_MGMT_ACTION_CATEGORY_WMM_TSPEC && category != IEEE_MGMT_ACTION_CATEGORY_FST &&
                category != IEEE_MGMT_ACTION_CATEGORY_PUBLIC && category != IEEE_MGMT_ACTION_CATEGORY_RADIO_RSRC &&
                category != IEEE_MGMT_ACTION_CATEGORY_PROTECTED_DUAL && category != IEEE_MGMT_ACTION_CATEGORY_QOS &&
                category != IEEE_MGMT_ACTION_CATEGORY_FAST_BSS_TRANS &&
                category != IEEE_MGMT_ACTION_CATEGORY_SA_QUERY && category != IEEE_MGMT_ACTION_CATEGORY_AV_STREAMING &&
                category != IEEE_MGMT_ACTION_CATEGORY_WNM)
        {
            wifi_d("Drop action frame: category = %d, action_code=%d", category, action_code);
            LEAVE();
            return ret;
        }
        if (category == IEEE_MGMT_ACTION_CATEGORY_BLOCK_ACK)
        {
            wifi_d("Drop BLOCK ACK action frame: action_code=%d", action_code);
            LEAVE();
            return ret;
        }
        if ((category == IEEE_MGMT_ACTION_CATEGORY_PUBLIC) &&
                (action_code == BSS_20_40_COEX || action_code == FILS_DISCOVERY))
        {
            wifi_d("Drop unneed public action frame %d", action_code);
            LEAVE();
            return ret;
        }
#ifdef DOT1AS_SUPPORT
        if ((category == IEEE_MGMT_ACTION_CATEGORY_UNPROTECT_WNM) && (action_code == 0x1))
        {
            prx_pd->toa_tod_tstamps = wlan_le64_to_cpu(prx_pd->toa_tod_tstamps);
            tstamps.t3              = prx_pd->toa_tod_tstamps >> 32;
            tstamps.t2              = (t_u32)prx_pd->toa_tod_tstamps;
            tstamps.t2_err       = 0;
            tstamps.t3_err       = 0;
            tstamps.ingress_time = pcb->moal_do_div(pmadapter->host_bbu_clk_delta, 10);
            tstamps.ingress_time += tstamps.t2; // t2, t3 is 10ns
            // and delta is in 1
            // ns unit;
            PRINTM(MINFO, "T2: %d, T3: %d, ingress: %lu\n", tstamps.t2, tstamps.t3, tstamps.ingress_time);
        }
#endif
        if (memcmp(pieee_pkt_hdr->addr1, broadcast, MLAN_MAC_ADDR_LENGTH))
            unicast = MTRUE;
        break;
    default:
        break;
    }
    wifi_d("%s %d sub_type=%d unicast=%d\n", __func__, __LINE__, sub_type, unicast);
    if (unicast == MTRUE)
    {
        if (memcmp(pieee_pkt_hdr->addr1, priv->curr_addr/*本机地址*/, MLAN_MAC_ADDR_LENGTH))
        {
            /**/
            wifi_d("%s %d sub_type=%d priv->curr_addr=" MACSTR " pieee_pkt_hdr->addr1=" MACSTR "\n",
                   __func__, __LINE__, sub_type, MAC2STR(priv->curr_addr), MAC2STR(pieee_pkt_hdr->addr1));
            wifi_d("Dropping mgmt frame for others: type=%d " MACSTR "\r\n", sub_type, MAC2STR(pieee_pkt_hdr->addr1));
            LEAVE();
            RT_ASSERT(0);
            return ret;
        }
    }

    // memmove((uint8_t *)pieee_pkt_hdr + (sizeof(wlan_802_11_header)),
    //         (uint8_t *)pieee_pkt_hdr + (sizeof(wlan_802_11_header)), payload_len - sizeof(wlan_802_11_header));
    wifi_d("%s %d payload_len=%d\n", __func__, __LINE__, payload_len);
    //payload_len -= sizeof(wlan_802_11_header);
    wifi_d("%s %d payload_len=%d\n", __func__, __LINE__, payload_len);
    if (payload_len > sizeof(wm_wifi.mgmt_resp.frame.frame))
    {
        wifi_w("The payload length (%d) overs the max length(%d), dropping mgmt frame: type=%d", payload_len,
               sizeof(wm_wifi.mgmt_resp.frame.frame), sub_type);
        dump_hex(payload, 64);
        return MLAN_STATUS_FAILURE;
    }
    wifi_d("%s %d priv->bss_role=%d sub_type=%d payload_len=%d\n", __func__, __LINE__,
           priv->bss_role, sub_type, payload_len);
    if (priv->bss_role == MLAN_BSS_ROLE_STA)
    {
        if (sub_type == (t_u16)SUBTYPE_ASSOC_RESP)
        {
            int result = WIFI_EVENT_REASON_FAILURE;
            int rv = wlan_ret_802_11_associate(priv, (IEEEtypes_MgmtHdr_t *)pieee_pkt_hdr, NULL);
            wifi_d("%s %d rv=%d\n", __func__, __LINE__, rv);
#if !CONFIG_WPA_SUPP
            if (rv != MLAN_STATUS_SUCCESS)
            {
                (void)wifi_event_completion(WIFI_EVENT_ASSOCIATION, WIFI_EVENT_REASON_FAILURE, NULL);
                return -WM_FAIL;
            }
#endif
#if CONFIG_WPA_SUPP
            nxp_wifi_assoc_event_mlme_t *assoc_resp = &wm_wifi.assoc_resp;
            memset(assoc_resp, 0, sizeof(nxp_wifi_assoc_event_mlme_t));

            IEEEtypes_AssocRsp_t *passoc_rsp1 = (IEEEtypes_AssocRsp_t *)(void *)pieee_pkt_hdr + sizeof(wlan_802_11_header);

            if (passoc_rsp1->status_code == WLAN_STATUS_UNSPECIFIED_FAILURE)
            {
                result = WIFI_EVENT_REASON_FAILURE;
                wifi_d("%s %d passoc_rsp1->status_code=%d\n", __func__, __LINE__, passoc_rsp1->status_code);
                goto assoc_resp_ret;
            }

            // IEEEtypes_AssocRsp_t *passoc_rsp =
            //     (IEEEtypes_AssocRsp_t *)((t_u8 *)(&resp->params) + sizeof(IEEEtypes_MgmtHdr_t));

#if CONFIG_11R
            priv->ft_roam = MFALSE;
#endif
            assoc_resp->frame.frame_len = payload_len;
            if (assoc_resp->frame.frame_len > (int)sizeof(assoc_resp->frame.frame))
            {
                wifi_e("Assocate response payload length (%d) overs the max length(%d), dropping it",
                       assoc_resp->frame.frame_len, sizeof(assoc_resp->frame.frame));
                assoc_resp->frame.frame_len = 0;
                result                      = WIFI_EVENT_REASON_FAILURE;
                wifi_d("%s %d passoc_rsp1->status_code=%d\n", __func__, __LINE__, passoc_rsp1->status_code);
                goto assoc_resp_ret;
            }
            // coverity[overrun-buffer-arg:SUPPRESS]
            memcpy(assoc_resp->frame.frame, pieee_pkt_hdr, assoc_resp->frame.frame_len);

            if (priv->assoc_req_size && (priv->assoc_req_size <= (int)sizeof(assoc_resp->req_ie)))
            {
                assoc_resp->req_ie_len = priv->assoc_req_size;
                // coverity[overrun-buffer-arg:SUPPRESS]
                memcpy(assoc_resp->req_ie, priv->assoc_req_buf, assoc_resp->req_ie_len);
            }
            if (wm_wifi.supp_if_callbk_fns->assoc_resp_callbk_fn)
            {
                priv->auth_flag = 0;

                wm_wifi.supp_if_callbk_fns->assoc_resp_callbk_fn(wm_wifi.if_priv, assoc_resp,
                        sizeof(nxp_wifi_assoc_event_mlme_t));
            }
#else
            IEEEtypes_AssocRsp_t *passoc_rsp = (IEEEtypes_AssocRsp_t *)(void *)&resp->params;
#endif
            if (passoc_rsp1->status_code == 0U)
            {
                result = WIFI_EVENT_REASON_SUCCESS;
            }
            else
            {
#if CONFIG_WIFI_EXTRA_DEBUG
                wifi_w("Assoc failed: Status: %d\n\r", passoc_rsp->status_code);
                wifi_show_assoc_fail_reason(passoc_rsp->status_code);
#endif
                /* Since we have failed assoc attempt clear this */
                priv->media_connected = MFALSE;
            }
            /** Clear BSS blacklist if association is successful */
            if (result == WIFI_EVENT_REASON_SUCCESS)
            {
                (void)memset(&priv->adapter->blacklist_bss.bssids, 0x0,
                             sizeof(priv->adapter->blacklist_bss.bssids));
                priv->adapter->blacklist_bss.num_bssid = 0;
            }
            wifi_d("%s %d result=%d\n", __func__, __LINE__, result);
#if CONFIG_WPA_SUPP
assoc_resp_ret:
#endif
            (void)wifi_event_completion(WIFI_EVENT_ASSOCIATION, result, NULL);
        }
        if (sub_type == (t_u16)SUBTYPE_AUTH)
        {
            wifi_d("%s %d \n", __func__, __LINE__);
            nxp_wifi_event_mlme_t *auth_resp = &wm_wifi.mgmt_resp;

            if (payload_len <= sizeof(auth_resp->frame.frame))
            {
                memset(auth_resp, 0, sizeof(nxp_wifi_event_mlme_t));
                auth_resp->frame.frame_len = payload_len;
                memcpy((void *)auth_resp->frame.frame, (const void *)pieee_pkt_hdr, payload_len);
                //RT_ASSERT(0);
                if (wm_wifi.supp_if_callbk_fns->auth_resp_callbk_fn)
                {
                    wifi_d("%s %d payload_len=%d\n", __func__, __LINE__, payload_len);
                    wm_wifi.supp_if_callbk_fns->auth_resp_callbk_fn(wm_wifi.if_priv, auth_resp,
                            auth_resp->frame.frame_len);
                    wifi_d("%s %d \n", __func__, __LINE__);
                }//auth 的反馈
            }
            else
            {
                wifi_e("Insufficient frame buffer");
            }
        }

        if (sub_type == (t_u16)SUBTYPE_DEAUTH)
        {
            nxp_wifi_event_mlme_t *deauth_resp = &wm_wifi.mgmt_resp;

            wlan_abort_split_scan();
            wifi_user_scan_config_cleanup();

            if (payload_len <= (int)sizeof(deauth_resp->frame.frame))
            {
                memset(deauth_resp, 0, sizeof(nxp_wifi_event_mlme_t));
                deauth_resp->frame.frame_len = payload_len;
                memcpy((void *)deauth_resp->frame.frame, (const void *)pieee_pkt_hdr, deauth_resp->frame.frame_len);
                if (wm_wifi.supp_if_callbk_fns->deauth_callbk_fn)
                {
                    wm_wifi.supp_if_callbk_fns->deauth_callbk_fn(wm_wifi.if_priv, deauth_resp,
                            deauth_resp->frame.frame_len);
                }
            }
            else
            {
                wifi_e("Insufficient frame buffer");
            }
        }

        if (sub_type == (t_u16)SUBTYPE_DISASSOC)
        {
            nxp_wifi_event_mlme_t *disassoc_resp = &wm_wifi.mgmt_resp;

            wlan_abort_split_scan();
            wifi_user_scan_config_cleanup();

            if (payload_len <= (int)sizeof(disassoc_resp->frame.frame))
            {
                memset(disassoc_resp, 0, sizeof(nxp_wifi_event_mlme_t));
                disassoc_resp->frame.frame_len = payload_len;
                memcpy((void *)disassoc_resp->frame.frame, (const void *)pieee_pkt_hdr, disassoc_resp->frame.frame_len);
                if (wm_wifi.supp_if_callbk_fns->disassoc_callbk_fn)
                {
                    wm_wifi.supp_if_callbk_fns->disassoc_callbk_fn(wm_wifi.if_priv, disassoc_resp,
                            disassoc_resp->frame.frame_len);
                }
            }
            else
            {
                wifi_e("Insufficient frame buffer");
            }
        }

        if (sub_type == (t_u16)SUBTYPE_ACTION)
        {
            nxp_wifi_event_mlme_t *mgmt_rx = &wm_wifi.mgmt_rx;

            if (payload_len <= (int)sizeof(mgmt_rx->frame.frame))
            {
                memset(mgmt_rx, 0, sizeof(nxp_wifi_event_mlme_t));
                mgmt_rx->frame.frame_len = payload_len;
                memcpy((void *)mgmt_rx->frame.frame, (const void *)pieee_pkt_hdr, mgmt_rx->frame.frame_len);
                mgmt_rx->frame.freq = channel_to_frequency(chan_num, band_config);
                if (wm_wifi.supp_if_callbk_fns->mgmt_rx_callbk_fn)
                {
                    wm_wifi.supp_if_callbk_fns->mgmt_rx_callbk_fn(wm_wifi.if_priv, mgmt_rx, mgmt_rx->frame.frame_len);
                }
            }
            else
            {
                wifi_e("Insufficient frame buffer");
            }
        }
    }
    else if (priv->bss_role == MLAN_BSS_ROLE_UAP)
    {
        nxp_wifi_event_mlme_t *mgmt_rx = &wm_wifi.mgmt_rx;

        mgmt_rx->frame.frame_len = payload_len;

        if (mgmt_rx->frame.frame_len <= (int)sizeof(mgmt_rx->frame.frame))
        {
            memcpy((void *)mgmt_rx->frame.frame, (const void *)pieee_pkt_hdr, mgmt_rx->frame.frame_len);
            mgmt_rx->frame.freq = channel_to_frequency(chan_num, band_config);
            if (wm_wifi.supp_if_callbk_fns->mgmt_rx_callbk_fn)
            {
                wm_wifi.supp_if_callbk_fns->mgmt_rx_callbk_fn(wm_wifi.hapd_if_priv, mgmt_rx, mgmt_rx->frame.frame_len);
            }
        }
        else
        {
            wifi_e("Insufficient frame buffer");
        }
    }

    LEAVE();
    return MLAN_STATUS_SUCCESS;
}
/*
判断并处理接收到的数据包是否为 802.11 管理帧，
如果是，则将其解析并分发给 WPA Supplicant 或上层协议栈进行进一步处理。
*/
void wifi_is_wpa_supplicant_input(const uint8_t interface, const uint8_t *buffer, const uint16_t len)
{
    mlan_private *priv           = (mlan_private *)mlan_adap->priv[interface];
    RxPD *prx_pd                 = (RxPD *)(void *)((t_u8 *)buffer + INTF_HEADER_LEN);//去掉头部的4个字节
    wlan_mgmt_pkt *pmgmt_pkt_hdr = MNULL;

    /* Check if this is mgmt packet and needs to
     * forwarded to app as an event
     */
    pmgmt_pkt_hdr          = (wlan_mgmt_pkt *)((t_u8 *)prx_pd + prx_pd->rx_pkt_offset);
    pmgmt_pkt_hdr->frm_len = wlan_le16_to_cpu(pmgmt_pkt_hdr->frm_len);

    if ((pmgmt_pkt_hdr->wlan_header.frm_ctl & IEEE80211_FC_MGMT_FRAME_TYPE_MASK) == 0)
        wlan_process_802dot11_mgmt_pkt2(priv, (t_u8 *)&pmgmt_pkt_hdr->wlan_header,
                                        pmgmt_pkt_hdr->frm_len + sizeof(wlan_mgmt_pkt) - sizeof(pmgmt_pkt_hdr->frm_len),
                                        prx_pd);
}

void wifi_wpa_supplicant_eapol_input(const uint8_t interface,
                                     const uint8_t *src_addr,
                                     const uint8_t *buffer,
                                     const uint16_t len)
{
    nxp_wifi_event_eapol_mlme_t *eapol_rx = &wm_wifi.eapol_rx;

    memcpy((void *)eapol_rx->mac_addr, (const void *)src_addr, MLAN_MAC_ADDR_LENGTH);
    wifi_d("%s %d len=%d\n", __func__, __LINE__, len);
    eapol_rx->frame.frame_len = len;
    memcpy((void *)eapol_rx->frame.frame, (const void *)buffer, eapol_rx->frame.frame_len);
    if (wm_wifi.supp_if_callbk_fns->eapol_rx_callbk_fn)
    {
        wm_wifi.supp_if_callbk_fns->eapol_rx_callbk_fn(
            interface == MLAN_BSS_TYPE_STA ? wm_wifi.if_priv : wm_wifi.hapd_if_priv, eapol_rx,
            eapol_rx->frame.frame_len);
    }
}

#define RX_PKT_TYPE_OFFSET  5U
#define ETH_PROTO_EAPOL     0x888EU
#define WIFI_SIZEOF_ETH_HDR 14U
static t_u8 rfc1042_eth_hdr[MLAN_MAC_ADDR_LENGTH] = {0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00};
#endif

int wifi_auth_event_req(const uint8_t interface, const uint8_t *buffer, const uint16_t len)
{
    struct skw_mgmt_hdr *skw_mgmt_hdr = (struct skw_mgmt_hdr *)((t_u8 *)buffer + 8);

    mlan_private *priv           = (mlan_private *)mlan_adap->priv[interface];
    wlan_mgmt_pkt *pmgmt_pkt_hdr = MNULL;

    /* Check if this is mgmt packet and needs to
     * forwarded to app as an event
     */
    wifi_d("%s %d pmgmt_pkt_hdr->frm_len=%d\n", __func__, __LINE__, skw_mgmt_hdr->mgmt_len);
    if ((pmgmt_pkt_hdr->wlan_header.frm_ctl & IEEE80211_FC_MGMT_FRAME_TYPE_MASK) == 0)
        wlan_process_802dot11_mgmt_pkt2(priv, (t_u8 *)skw_mgmt_hdr->mgmt,
                                        skw_mgmt_hdr->mgmt_len,
                                        (RxPD *)skw_mgmt_hdr);
    return WM_SUCCESS;
}
int wifi_assoc_event_req(const uint8_t interface, const uint8_t *buffer, const uint16_t len)
{
    struct skw_mgmt_hdr *skw_mgmt_hdr = (struct skw_mgmt_hdr *)((t_u8 *)buffer + 8);

    mlan_private *priv           = (mlan_private *)mlan_adap->priv[interface];
    wlan_mgmt_pkt *pmgmt_pkt_hdr = MNULL;

    /* Check if this is mgmt packet and needs to
     * forwarded to app as an event
     */

    return WM_SUCCESS;
}
int wifi_low_level_input(const uint8_t interface, const uint8_t *buffer, const uint16_t len)
{
#if CONFIG_WPA_SUPP
    //RxPD *prx_pd  = (RxPD *)(void *)((t_u8 *)buffer + INTF_HEADER_LEN);
    eth_hdr *ethh = MNULL;
    t_u16 eth_proto;
    t_u8 offset = 0;
#if 0
    //已经交给其他地方进行处理
    if (*((t_u16 *)buffer + RX_PKT_TYPE_OFFSET) == PKT_TYPE_MGMT_FRAME)//判断是否为管理帧
    {
        wifi_is_wpa_supplicant_input(interface, buffer, len);
        return WM_SUCCESS;
    }
#endif
    ethh = (eth_hdr *)buffer;//((t_u8 *)prx_pd + prx_pd->rx_pkt_offset);

    eth_proto = mlan_ntohs(ethh->h_proto);
#if 0
    /*
    在 WiFi 驱动中，802.11 数据帧常用 LLC/SNAP 封装来兼容以太网协议。
    需要识别并跳过 LLC/SNAP 头，才能正确解析上层协议类型和数据内容。
    */
    if (memcmp((t_u8 *)prx_pd + prx_pd->rx_pkt_offset + WIFI_SIZEOF_ETH_HDR, rfc1042_eth_hdr,
               sizeof(rfc1042_eth_hdr)) == 0U)
    {
        eth_llc_hdr *ethllchdr = (eth_llc_hdr *)(void *)((t_u8 *)prx_pd + prx_pd->rx_pkt_offset + WIFI_SIZEOF_ETH_HDR);
        eth_proto              = mlan_ntohs(ethllchdr->type);
        offset                 = sizeof(eth_llc_hdr);
    }
#endif
    //wifi_d("%s %d eth_proto=0x%x wifi_rx_status=%d\n", __func__, __LINE__, eth_proto,wifi_rx_status);
    if (eth_proto == ETH_PROTO_EAPOL)//管理帧
    {
        wifi_wpa_supplicant_eapol_input(interface, ethh->src_addr, (uint8_t *)(ethh + 1) + offset, len);
        return WM_SUCCESS;
    }
#endif
    if (wifi_rx_status == WIFI_DATA_BLOCK)
    {
        wifi_rx_block_cnt++;
        return WM_SUCCESS;
    }
#if 0
    if (mlan_adap->ps_state == PS_STATE_SLEEP)
    {
        OSA_RWLockWriteUnlock(&sleep_rwlock);
        mlan_adap->ps_state = PS_STATE_AWAKE;
    }
#endif
    if (wm_wifi.data_input_callback != NULL)
    {
        wm_wifi.data_input_callback(interface, buffer, len);//发送数据到上层
        return WM_SUCCESS;
    }


    return -WM_FAIL;
}

#define ERR_INPROGRESS -5

#define WL_ID_LL_OUTPUT "wifi_low_level_output"

void wifi_tx_card_awake_lock(void)
{
    int ret;

    /* Write mutex is used to avoid the case that, during waitting for sleep confirm cmd response,
     * wifi_driver_tx task might be scheduled and send data to FW */
    (void)OSA_MutexLock((osa_mutex_handle_t)sleep_rwlock.write_mutex, osaWaitForever_c);
    ret = OSA_RWLockReadLock(&sleep_rwlock, MAX_WAIT_TIME);
    (void)OSA_MutexUnlock((osa_mutex_handle_t)sleep_rwlock.write_mutex);
    if (ret != WM_SUCCESS)
    {
#if CONFIG_WIFI_PS_DEBUG
        wifi_e("Failed to wakeup card for Tx");
#endif
#if CONFIG_WIFI_RECOVERY
        wifi_recovery_enable = true;
#else
        RT_ASSERT(0);
#endif
    }
}

void wifi_tx_card_awake_unlock(void)
{
    OSA_RWLockReadUnlock(&sleep_rwlock);
}

#if CONFIG_WMM

#define ETHER_TYPE_IP_01         0xc
#define ETHER_TYPE_IP_02         0xd
#define ETHER_TYPE_IPV4_VALUE_01 0x8
#define ETHER_TYPE_IPV4_VALUE_02 0x0
#define WMM_PACKET_TOS_IV4       0xf
#define PRIORITY_COMPENSATOR     0x20
#define UDP_IDENTIFIER_POS       0x11
#define UDP_IDENTIFIER_VAL       0xda

#define ETHER_TYPE_IPV6_VALUE_01 0x86
#define ETHER_TYPE_IPV6_VALUE_02 0xdd
#define WMM_PACKET_TOS_IPV6_01   0xe
#define WMM_PACKET_TOS_IPV6_02   0xf
#define TOS_MASK_IPV6            0x0ff0 /* 0000111111110000 */

void wifi_wmm_init()
{
    mlan_private *pmpriv    = (mlan_private *)mlan_adap->priv[0];
    mlan_adapter *pmadapter = pmpriv->adapter;
    mlan_status status      = MLAN_STATUS_SUCCESS;

    status =
        wlan_prepare_cmd(pmpriv, HostCmd_CMD_WMM_PARAM_CONFIG, HostCmd_ACT_GEN_SET, 0, MNULL, &pmadapter->ac_params);
    if (status != MLAN_STATUS_SUCCESS)
    {
        wifi_e("ERR: WMM wlan_prepare_cmd returned status=0x%x", status);
    }
}

/* Packet priority is 16th byte of payload.
 * Provided that the packet is IPV4 type
 * Since value comes between the range of 0-255, coversion is expected between 0-7 to map to TIDs.
 * */
t_u32 wifi_wmm_get_pkt_prio(void *buf, t_u8 *tid)
{
    bool ip_hdr = 0;

    if (buf == NULL)
        return -WM_FAIL;
    t_u8 *type_01 = net_stack_buffer_skip(buf, ETHER_TYPE_IP_01);
    t_u8 *type_02 = net_stack_buffer_skip(buf, ETHER_TYPE_IP_02);
    if (*type_01 == ETHER_TYPE_IPV4_VALUE_01 && *type_02 == ETHER_TYPE_IPV4_VALUE_02)
    {
        t_u8 *id = net_stack_buffer_skip(buf, WMM_PACKET_TOS_IV4);
        *tid     = *id / PRIORITY_COMPENSATOR;
        ip_hdr   = 1;
    }
    else if (*type_01 == ETHER_TYPE_IPV6_VALUE_01 && *type_02 == ETHER_TYPE_IPV6_VALUE_02)
    {
        t_u8 *tos1     = net_stack_buffer_skip(buf, WMM_PACKET_TOS_IPV6_01);
        t_u8 *tos2     = net_stack_buffer_skip(buf, WMM_PACKET_TOS_IPV6_02);
        t_u16 ipv6_tos = (*tos1 << 8) | (*tos2);
        *tid           = (t_u8)(((ipv6_tos & TOS_MASK_IPV6) >> 4) / PRIORITY_COMPENSATOR);
        ip_hdr         = 1;
    }
    if (ip_hdr)
    {
        switch (*tid)
        {
        case 0:
            return WMM_AC_BE;
        case 1:
        case 2:
            return WMM_AC_BK;
        case 3:
            return WMM_AC_BE;
        case 4:
        case 5:
            return WMM_AC_VI;
        case 6:
        case 7:
            return WMM_AC_VO;
        default:
            return WMM_AC_BE;
        }
    }
    else
        return WMM_AC_BE;
}

INLINE t_u8 wifi_wmm_get_packet_cnt(void)
{
    return (MAX_WMM_BUF_NUM - mlan_adap->outbuf_pool.free_cnt);
}

#if CONFIG_WIFI_TP_STAT
    t_u32 g_wifi_xmit_schedule_end = 0;
#endif

#if CONFIG_AMSDU_IN_AMPDU
/* aggregate one amsdu packet and xmit */
static mlan_status wifi_xmit_amsdu_pkts(mlan_private *priv, t_u8 ac, raListTbl *ralist)
{
    outbuf_t *buf                = MNULL;
    t_u32 max_amsdu_size         = MIN(priv->max_amsdu, priv->adapter->tx_buffer_size);
    t_u32 amsdu_offset           = sizeof(TxPD) + INTF_HEADER_LEN;
    t_u8 amsdu_cnt               = 0;
    t_u32 amsdu_buf_used_size    = 0;
    int amsdu_buf_available_size = max_amsdu_size - amsdu_buf_used_size;
    t_u32 amsdu_pkt_len          = 0;
    int pad_len                  = 0;
    int last_pad_len             = 0;
#if CONFIG_WIFI_TP_STAT
    t_u8 *buf_end = MNULL;
#endif

    while (ralist->total_pkts > 0)
    {
        mlan_adap->callbacks.moal_semaphore_get(mlan_adap->pmoal_handle, &ralist->buf_head.plock);
        buf = (outbuf_t *)util_peek_list(mlan_adap->pmoal_handle, &ralist->buf_head, MNULL, MNULL);
        ASSERT(buf != NULL);

#if CONFIG_WIFI_TP_STAT
        buf_end = &buf->data[0] + buf->tx_pd.tx_pkt_length;
        wifi_stat_tx_dequeue_start(buf_end, g_wifi_xmit_schedule_end);
#endif

        /* calculate amsdu buffer length */
        amsdu_buf_used_size += buf->tx_pd.tx_pkt_length + sizeof(TxPD) + INTF_HEADER_LEN;
        if (amsdu_cnt == 0)
        {
            /* First A-MSDU packet */
            amsdu_buf_available_size = max_amsdu_size - amsdu_buf_used_size - LLC_SNAP_LEN;
        }
        else
        {
            /* The following A-MSDU packets */
            amsdu_pkt_len            = amsdu_buf_used_size - sizeof(TxPD) - INTF_HEADER_LEN + LLC_SNAP_LEN;
            pad_len                  = ((amsdu_pkt_len & 3)) ? (4 - ((amsdu_pkt_len) & 3)) : 0;
            amsdu_buf_available_size = max_amsdu_size - amsdu_pkt_len - pad_len;
        }

        /* dequeue and store this buffer in amsdu buffer */
        if (amsdu_buf_available_size >= 0)
        {
            util_unlink_list(mlan_adap->pmoal_handle, &ralist->buf_head, &buf->entry, MNULL, MNULL);
            ralist->total_pkts--;
            mlan_adap->callbacks.moal_semaphore_put(mlan_adap->pmoal_handle, &ralist->buf_head.plock);

            amsdu_offset += wlan_11n_form_amsdu_pkt(wifi_get_amsdu_outbuf(amsdu_offset), &buf->data[0],
                                                    buf->tx_pd.tx_pkt_length, &last_pad_len);
            amsdu_cnt++;

#if CONFIG_WIFI_TP_STAT
            wifi_stat_tx_dequeue_end(buf_end);
#endif
            wifi_wmm_buf_put(buf);
            priv->wmm.pkts_queued[ac]--;
        }
        else
        {
            mlan_adap->callbacks.moal_semaphore_put(mlan_adap->pmoal_handle, &ralist->buf_head.plock);
        }

        /*
         * amsdu buffer room not enough, or last packet in this ra_list in AC queue,
         * add amsdu buffer to imu queue
         */
        if (amsdu_buf_available_size < 0 || ralist->total_pkts == 0)
        {
            return wlan_xmit_wmm_amsdu_pkt((mlan_wmm_ac_e)ac, priv->bss_index, amsdu_offset - last_pad_len,
                                           wifi_get_amsdu_outbuf(0), amsdu_cnt);
        }
    }
    return MLAN_STATUS_SUCCESS;
}
#endif

t_u8 wifi_txbuf_available()
{
#ifdef RW610
    return 1;//!HAL_ImuIsTxBufQueueEmpty(kIMU_LinkCpu1Cpu3);
#else
    return !!(mlan_adap->mp_wr_bitmap);
#endif
}

static inline t_u8 wifi_is_max_tx_cnt(t_u8 pkt_cnt)
{
#ifdef RW610
    return pkt_cnt >= 15 ? MTRUE : MFALSE;
#else
    return (pkt_cnt >= SDIO_PAYLOAD_SIZE) ? MTRUE : MFALSE;
#endif
}

/* dequeue and xmit one packet */
static mlan_status wifi_xmit_pkts(mlan_private *priv, t_u8 ac, raListTbl *ralist)
{
    mlan_status ret;
    outbuf_t *buf = MNULL;

    mlan_adap->callbacks.moal_semaphore_get(mlan_adap->pmoal_handle, &ralist->buf_head.plock);
    buf = (outbuf_t *)util_dequeue_list(mlan_adap->pmoal_handle, &ralist->buf_head, MNULL, MNULL);
    ralist->total_pkts--;
    mlan_adap->callbacks.moal_semaphore_put(mlan_adap->pmoal_handle, &ralist->buf_head.plock);
    ASSERT(buf != MNULL);

    /* TODO: this may go wrong for TxPD->tx_pkt_type 0xe5 */
    /* this will get card port lock and probably sleep */
#if CONFIG_TX_RX_ZERO_COPY
    ret = wlan_xmit_wmm_pkt(priv->bss_index, buf->tx_pd.tx_pkt_length + sizeof(TxPD) + INTF_HEADER_LEN, (t_u8 *)buf);
#else
    ret = wlan_xmit_wmm_pkt(priv->bss_index, buf->tx_pd.tx_pkt_length + sizeof(TxPD) + INTF_HEADER_LEN,
                            (t_u8 *)&buf->intf_header[0]);
#endif
    if (ret != MLAN_STATUS_SUCCESS)
    {
#ifdef RW610
        ASSERT(0);
#else
        mlan_adap->callbacks.moal_semaphore_get(mlan_adap->pmoal_handle, &ralist->buf_head.plock);
        util_enqueue_list_head(mlan_adap->pmoal_handle, &ralist->buf_head, &buf->entry, MNULL, MNULL);
        ralist->total_pkts++;
        mlan_adap->callbacks.moal_semaphore_put(mlan_adap->pmoal_handle, &ralist->buf_head.plock);
        return MLAN_STATUS_RESOURCE;
#endif
    }

    wifi_wmm_buf_put(buf);
    priv->wmm.pkts_queued[ac]--;

    return MLAN_STATUS_SUCCESS;
}

/*
 *  xmit all buffers under this ralist
 *  should be called inside wmm tid_tbl_ptr ra_list lock,
 *  return MLAN_STATUS_SUCESS to continue looping ralists,
 *  return MLAN_STATUS_RESOURCE to break looping ralists
 */
static mlan_status wifi_xmit_ralist_pkts(mlan_private *priv, t_u8 ac, raListTbl *ralist, t_u8 *pkt_cnt)
{
    mlan_status ret;

    if (ralist->tx_pause == MTRUE)
        return MLAN_STATUS_SUCCESS;

    while (ralist->total_pkts > 0)
    {
        if ((wifi_txbuf_available() == MFALSE) || (WIFI_DATA_RUNNING != wifi_tx_status))
            break;

#if CONFIG_AMSDU_IN_AMPDU
        if (wlan_is_amsdu_allowed(priv, priv->bss_index, ralist->total_pkts, ac))
            ret = wifi_xmit_amsdu_pkts(priv, ac, ralist);
        else
#endif
            ret = wifi_xmit_pkts(priv, ac, ralist);

        if (ret != MLAN_STATUS_SUCCESS)
            return ret;

        /*
         * in amsdu case,
         * multiple packets aggregated as one amsdu packet, are counted as one imu packet
         */
        (*pkt_cnt)++;
        if (wifi_is_max_tx_cnt(*pkt_cnt) == MTRUE)
        {
            wlan_flush_wmm_pkt(*pkt_cnt);
            *pkt_cnt = 0;
        }
    }
    return MLAN_STATUS_SUCCESS;
}

/*
 *  dequeue and xmit all buffers under ac queue
 *  loop each ac queue
 *  loop each ralist
 *  dequeue all buffers from buf_head list and xmit
 */
static int wifi_xmit_wmm_ac_pkts_enh(mlan_private *priv)
{
    int ac;
    mlan_status ret;
    t_u8 pkt_cnt       = 0;
    raListTbl *ralist  = MNULL;
    tid_tbl_t *tid_ptr = MNULL;

#if CONFIG_WIFI_TP_STAT
    g_wifi_xmit_schedule_end = OSA_GetTimestamp();
#endif

    for (ac = WMM_AC_VO; ac >= 0; ac--)
    {
        tid_ptr = &priv->wmm.tid_tbl_ptr[ac];

        mlan_adap->callbacks.moal_semaphore_get(mlan_adap->pmoal_handle, &tid_ptr->ra_list.plock);

        if (priv->wmm.pkts_queued[ac] == 0)
        {
            mlan_adap->callbacks.moal_semaphore_put(mlan_adap->pmoal_handle, &tid_ptr->ra_list.plock);
            continue;
        }

        ralist =
            (raListTbl *)util_peek_list(mlan_adap->pmoal_handle, (mlan_list_head *)&tid_ptr->ra_list, MNULL, MNULL);

        while (ralist && ralist != (raListTbl *)&tid_ptr->ra_list)
        {
            ret = wifi_xmit_ralist_pkts(priv, ac, ralist, &pkt_cnt);
            if (ret != MLAN_STATUS_SUCCESS)
            {
                mlan_adap->callbacks.moal_semaphore_put(mlan_adap->pmoal_handle, &tid_ptr->ra_list.plock);
                goto RET;
            }
            ralist = ralist->pnext;
        }
        mlan_adap->callbacks.moal_semaphore_put(mlan_adap->pmoal_handle, &tid_ptr->ra_list.plock);
    }

RET:
    wlan_flush_wmm_pkt(pkt_cnt);
    return WM_SUCCESS;
}

t_void wlan_process_bypass_txq(t_u8 interface)
{
    bypass_outbuf_t *buf;
    mlan_status status = MLAN_STATUS_SUCCESS;
    pmlan_private priv = mlan_adap->priv[interface];

    wifi_tx_card_awake_lock();
#ifndef RW610
    wifi_sdio_lock();
#endif

    while (!wlan_bypass_txq_empty(interface) && (wifi_txbuf_available() == MTRUE))
    {
        wlan_get_bypass_lock(interface);
        buf = (bypass_outbuf_t *)util_dequeue_list(mlan_adap->pmoal_handle, &priv->bypass_txq, MNULL, MNULL);
        priv->bypass_txq_cnt--;
        wlan_put_bypass_lock(interface);

        status = wlan_xmit_bypass_pkt((t_u8 *)&buf->intf_header[0],
                                      buf->tx_pd.tx_pkt_length + sizeof(TxPD) + INTF_HEADER_LEN, interface);

#if !CONFIG_MEM_POOLS
        OSA_MemoryFree(buf);
#else
        OSA_MemoryPoolFree(buf_1536_MemoryPool, buf);
#endif

        if (status != MLAN_STATUS_SUCCESS)
        {
            wifi_d("[%s] bypass xmit pkt failed \r\n", __func__);
        }
    }

#ifndef RW610
    wifi_sdio_unlock();
#endif
    wifi_tx_card_awake_unlock();
}

typedef enum _wifi_tx_event
{
    TX_TYPE_DATA = 10U,
    TX_TYPE_NULL_DATA,
    TX_TYPE_BYPASS_DATA,
} wifi_tx_event_t;

#ifdef __ZEPHYR__
static void notify_wifi_driver_tx_event(uint32_t events)
{
    struct bus_message msg;
    osa_status_t status;

    if (events & WIFI_EVENT_TX_DATA)
    {
        msg.event = MLAN_TYPE_DATA;
    }
    else if (events & WIFI_EVENT_TX_NULL_DATA)
    {
        msg.event = MLAN_TYPE_NULL_DATA;
    }
    else if (events & WIFI_EVENT_TX_BYPASS_DATA)
    {
        msg.event = MLAN_TYPE_BYPASS_DATA;
    }
    else
    {
        msg.event = MLAN_TYPE_DATA;
        wifi_w("unknown tx event");
    }

    msg.reason = (events & 1) ? MLAN_BSS_TYPE_STA : MLAN_BSS_TYPE_UAP;

    status = OSA_MsgQPut((osa_msgq_handle_t)wm_wifi.tx_data, &msg);
    if (status == KOSA_StatusSuccess)
    {
        if (msg.event == MLAN_TYPE_BYPASS_DATA && !OSA_IsISR())
        {
            OSA_TaskYield();
        }
    }
}
#endif

int send_wifi_driver_tx_data_event(t_u8 interface)
{
    osa_event_flags_t events;

    events = (1U << interface) | WIFI_EVENT_TX_DATA;

    if (1 != wm_wifi.wifi_core_init_done)
        return 0;

#ifdef __ZEPHYR__
    notify_wifi_driver_tx_event(events);
#else
    (void)OSA_EventSet((osa_event_handle_t)wm_wifi.wifi_event_Handle, events);
    if (!__get_IPSR())
    {
        OSA_TaskYield();
    }
#endif

    return 0;
}

int send_wifi_driver_tx_null_data_event(t_u8 interface)
{
    osa_event_flags_t events;

    events = (1U << interface) | WIFI_EVENT_TX_NULL_DATA;

#ifdef __ZEPHYR__
    notify_wifi_driver_tx_event(events);
#else
    (void)OSA_EventSet((osa_event_handle_t)wm_wifi.wifi_event_Handle, events);
    if (!__get_IPSR())
    {
        OSA_TaskYield();
    }
#endif

    return 0;
}

int send_wifi_driver_bypass_data_event(t_u8 interface)
{
    osa_event_flags_t events;

    events = (1U << interface) | WIFI_EVENT_TX_BYPASS_DATA;

#ifdef __ZEPHYR__
    notify_wifi_driver_tx_event(events);
#else
    (void)OSA_EventSet((osa_event_handle_t)wm_wifi.wifi_event_Handle, events);
    if (!__get_IPSR())
    {
        OSA_TaskYield();
    }
#endif

    return 0;
}

static void wifi_drv_tx_task(osa_task_param_t arg)
{
    mlan_private *pmpriv    = (mlan_private *)mlan_adap->priv[0];
    mlan_adapter *pmadapter = pmpriv->adapter;
    t_u8 i;
    t_u16 event    = 0;
    t_u8 interface = 0;
#ifdef __ZEPHYR__
    struct bus_message msg;
    osa_status_t status;
#else
    osa_event_flags_t flagsToWait =
        WIFI_EVENT_STA | WIFI_EVENT_UAP | WIFI_EVENT_TX_DATA | WIFI_EVENT_TX_NULL_DATA | WIFI_EVENT_TX_BYPASS_DATA;
    osa_event_flags_t pSetFlags;
#endif

    for (;;)
    {
        event     = 0;
        interface = 0;

#if CONFIG_ECSA
        /*
         * Reduce block tx check interval, try to make it sync with ECSA status.
         */
        while (true == get_ecsa_block_tx_flag())
        {
            OSA_TimeDelay(wm_wifi.beacon_period);
        }
#endif
#ifdef __ZEPHYR__
        status = OSA_MsgQGet((osa_msgq_handle_t)wm_wifi.tx_data, &msg, osaWaitForever_c);
        if (status == KOSA_StatusSuccess)
        {
            event     = msg.event;
            interface = msg.reason;
        }
#else
        (void)OSA_EventWait((osa_event_handle_t)wm_wifi.wifi_event_Handle, flagsToWait, false, osaWaitForever_c,
                            &pSetFlags);

        OSA_EventClear((osa_event_handle_t)wm_wifi.wifi_event_Handle, WIFI_EVENT_STA | WIFI_EVENT_UAP |
                       WIFI_EVENT_TX_DATA | WIFI_EVENT_TX_NULL_DATA |
                       WIFI_EVENT_TX_BYPASS_DATA);

        if ((pSetFlags & (WIFI_EVENT_STA | WIFI_EVENT_UAP | WIFI_EVENT_TX_DATA | WIFI_EVENT_TX_NULL_DATA |
                          WIFI_EVENT_TX_BYPASS_DATA)) == 0U)
        {
            continue;
        }

        if (pSetFlags & WIFI_EVENT_STA)
        {
            interface = MLAN_BSS_TYPE_STA;
        }

        if (pSetFlags & WIFI_EVENT_UAP)
        {
            interface = MLAN_BSS_TYPE_UAP;
        }

        if ((interface != MLAN_BSS_TYPE_STA) && (interface != MLAN_BSS_TYPE_UAP))
        {
            continue;
        }

        if (pSetFlags & WIFI_EVENT_TX_DATA)
        {
            event = MLAN_TYPE_DATA;
        }

        if (pSetFlags & WIFI_EVENT_TX_NULL_DATA)
        {
            event = MLAN_TYPE_NULL_DATA;
        }

        if (pSetFlags & WIFI_EVENT_TX_BYPASS_DATA)
        {
            event = MLAN_TYPE_BYPASS_DATA;
        }
#endif
        pmadapter = mlan_adap;

#if CONFIG_HOST_SLEEP
        wakelock_get();
#endif
        if (event == MLAN_TYPE_DATA || event == MLAN_TYPE_NULL_DATA || event == MLAN_TYPE_BYPASS_DATA)
        {
#if CONFIG_WMM_UAPSD
            while (pmadapter->pps_uapsd_mode && (pmadapter->tx_lock_flag == MTRUE))
            {
                OSA_TimeDelay(1);
            }
#endif
            for (i = 0; i < MLAN_MAX_BSS_NUM; i++)
            {
                if (!wlan_bypass_txq_empty(i))
                {
                    /*Give high priority to xmit bypass txqueue*/
                    wlan_process_bypass_txq(i);
                }
            }

            /* Send packet when the outbuf pool is not empty and not in block tx status*/
            if ((wifi_wmm_get_packet_cnt() > 0) && (WIFI_DATA_RUNNING == wifi_tx_status))
            {
                for (i = 0; i < MLAN_MAX_BSS_NUM; i++)
                {
                    pmpriv = pmadapter->priv[i];
                    if (!pmpriv->media_connected || pmpriv->tx_pause)
                    {
                        continue;
                    }

                    wifi_tx_card_awake_lock();
#ifndef RW610
                    wifi_sdio_lock();
#endif
                    wifi_xmit_wmm_ac_pkts_enh(pmpriv);
#ifndef RW610
                    wifi_sdio_unlock();
#endif
                    wifi_tx_card_awake_unlock();
                }
            }
#if CONFIG_WMM_UAPSD
            else
            {
                if (event == MLAN_TYPE_NULL_DATA)
                {
                    wifi_tx_card_awake_lock();
                    /* send null packet until the finish of CMD response processing */
                    OSA_SemaphoreWait((osa_semaphore_handle_t)uapsd_sem, osaWaitForever_c);
                    pmpriv = pmadapter->priv[interface];
                    if (pmadapter->pps_uapsd_mode && pmpriv->media_connected && pmadapter->gen_null_pkt)
                    {
                        if (wlan_send_null_packet(
                                    pmpriv, MRVDRV_TxPD_POWER_MGMT_NULL_PACKET | MRVDRV_TxPD_POWER_MGMT_LAST_PACKET) ==
                                MLAN_STATUS_SUCCESS)
                        {
                            pmadapter->tx_lock_flag = MTRUE;
                        }
                        else
                        {
                            OSA_SemaphorePost((osa_semaphore_handle_t)uapsd_sem);
                            pmadapter->tx_lock_flag = MFALSE;
                        }
                    }
                    else
                    {
                        wifi_d(
                            "No need to send null packet, pps_uapsd_mode: %d, media_connected: %d, gen_null_pkt: "
                            "%d",
                            pmadapter->pps_uapsd_mode, pmpriv->media_connected, pmadapter->gen_null_pkt);
                        OSA_SemaphorePost((osa_semaphore_handle_t)uapsd_sem);
                        pmadapter->tx_lock_flag = MFALSE;
                    }
                    wifi_tx_card_awake_unlock();
                }
            }
#endif
        }
#if CONFIG_HOST_SLEEP
        wakelock_put();
#endif
    }
}
#endif /* CONFIG_WMM */

#ifndef __ZEPHYR__
#if CONFIG_11AX
#if CONFIG_TCP_ACK_ENH
#define ETH_PROTO_IP     0x0800U
#define WIFI_IPPROTO_TCP 6

#define RATEID_VHT_MCS7_1SS_BW80 58
#define RATEID_VHT_MCS7_1SS_BW40 48
#define RATEID_VHT_MCS7_1SS_BW20 38

#define RATEID_VHT_MCS8_1SS_BW80 59
#define RATEID_VHT_MCS8_1SS_BW40 49
#define RATEID_VHT_MCS8_1SS_BW20 39

#define RATEID_VHT_MCS9_1SS_BW80 60
#define RATEID_VHT_MCS9_1SS_BW40 50
#define RATEID_VHT_MCS9_1SS_BW20 40

#define RATEID_HE_MCS9_1SS_BW80 94
#define RATEID_HE_MCS8_1SS_BW40 81
#define RATEID_HE_MCS4_1SS_BW40 77
#define RATEID_HE_MCS7_1SS_BW20 68

static int wlan_is_tcp_ack(mlan_private *priv, const t_u8 *pmbuf)
{
    eth_hdr *ethh = NULL;
    ip_hdr *iph   = NULL;
    tcp_hdr *tcph = NULL;

    ENTER();

    /** check the tcp packet */
#if CONFIG_TX_RX_ZERO_COPY
    ethh = (eth_hdr *)(((outbuf_t *)pmbuf)->eth_header);
#else
    ethh = (eth_hdr *)(pmbuf);
#endif
    if (mlan_ntohs(ethh->h_proto) != ETH_PROTO_IP)
    {
        LEAVE();
        return 0;
    }
#if CONFIG_TX_RX_ZERO_COPY
    iph = (ip_hdr *)(((outbuf_t *)pmbuf)->payload);
#else
    iph  = (ip_hdr *)((t_u8 *)ethh + sizeof(eth_hdr));
#endif
    if (iph->protocol != WIFI_IPPROTO_TCP)
    {
        LEAVE();
        return 0;
    }
#if CONFIG_TX_RX_ZERO_COPY
    tcph = (tcp_hdr *)(net_stack_buffer_skip(((outbuf_t *)pmbuf)->buffer, (uint16_t)(iph->ihl * 4)));
#else
    tcph = (tcp_hdr *)((t_u8 *)iph + iph->ihl * 4);
#endif

    if (*((t_u8 *)tcph + 13) == 0x10)
    {
        /* Only replace ACK */
        if (mlan_ntohs(iph->tot_len) > (iph->ihl + tcph->_hdrlen_rsvd_flags.doff) * 4)
        {
            /* Don't drop ACK with payload */
            /* TODO: should we delete previous TCP session */
            LEAVE();
            return 0;
        }
        return 1;
    }

    LEAVE();
    return 0;
}
#endif /** CONFIG_TCP_ACK_ENH */
#endif /** CONFIG_11AX*/
#endif

#if CONFIG_WMM

int wifi_add_to_bypassq(const t_u8 interface, void *pkt, t_u32 len)
{
    t_u32 pkt_len            = 0;
    t_u32 link_point_len     = 0;
    bypass_outbuf_t *poutbuf = NULL;
    t_u16 eth_type           = 0;
    t_u32 magic_cookie       = 0;

    eth_type = mlan_ntohs(*(t_u16 *)(net_stack_buffer_skip(pkt, MLAN_ETHER_PKT_TYPE_OFFSET)));

    if (len > MLAN_ETHER_PKT_DHCP_MAGIC_COOKIE_OFFSET)
    {
        magic_cookie = mlan_ntohl(*((t_u32 *)(net_stack_buffer_skip(pkt, MLAN_ETHER_PKT_DHCP_MAGIC_COOKIE_OFFSET))));
    }

    if ((eth_type == MLAN_ETHER_PKT_TYPE_EAPOL) || (eth_type == MLAN_ETHER_PKT_TYPE_ARP) ||
            (magic_cookie == MLAN_ETHER_PKT_DHCP_MAGIC_COOKIE))
    {
        /*Dword align*/
        pkt_len        = sizeof(TxPD) + INTF_HEADER_LEN;
        link_point_len = sizeof(mlan_linked_list);

#if !CONFIG_MEM_POOLS
        poutbuf = OSA_MemoryAllocate(link_point_len + pkt_len + len);
#else
        poutbuf = (bypass_outbuf_t *)OSA_MemoryPoolAllocate(buf_1536_MemoryPool);
#endif
        if (!poutbuf)
        {
            wuap_e("[%s] ERR:Cannot allocate buffer!\r\n", __func__);
            mlan_adap->priv[interface]->tx_overrun_cnt++;
            return -WM_FAIL;
        }

        (void)memset((t_u8 *)poutbuf, 0, link_point_len + pkt_len);

        (void)net_stack_buffer_copy_partial(pkt, (void *)((t_u8 *)poutbuf + link_point_len + pkt_len), (t_u16)len, 0);

        /* process packet headers with interface header and TxPD */
        process_pkt_hdrs((void *)((t_u8 *)poutbuf + link_point_len), pkt_len + len, interface, 0, 0);

        wlan_add_buf_bypass_txq((t_u8 *)poutbuf, interface);
        send_wifi_driver_bypass_data_event(interface);

        return WM_SUCCESS;
    }

    return -WM_FAIL;
}
#endif

int wifi_low_level_output(const t_u8 interface,
                          const t_u8 *sd_buffer,
                          const t_u16 len
#if CONFIG_WMM
    ,
    t_u8 pkt_prio,
    t_u8 tid
#endif
                         )
{
    int ret;
#if CONFIG_TX_RX_ZERO_COPY
    const t_u8 *buffer = ((outbuf_t *)sd_buffer)->eth_header;
#else
    const t_u8 *buffer = sd_buffer +
#if CONFIG_WMM
                         sizeof(mlan_linked_list) +
#endif
                         sizeof(TxPD) + INTF_HEADER_LEN;
#endif
#if !CONFIG_WMM
    int retry = retry_attempts;
    mlan_status i;
#endif

    /** Tx control */
    t_u32 tx_control = 0;
#ifndef __ZEPHYR__
    mlan_private *pmpriv = (mlan_private *)mlan_adap->priv[interface];
#endif
    w_pkt_d("Data TX: Kernel=>Driver, if %d, len %d", interface, len);

    // wakelock_get(WL_ID_LL_OUTPUT);
    /* Following condition is added to check if device is not connected and data packet is being transmitted */
#ifndef __ZEPHYR__
    if (0)//(pmpriv->media_connected == MFALSE)
    {
#if CONFIG_WMM
        wifi_wmm_buf_put((outbuf_t *)sd_buffer);
        wifi_wmm_drop_no_media(interface);
#endif
        return -WM_E_BUSY;
    }

#if CONFIG_11AX
#if CONFIG_TCP_ACK_ENH
    if ((interface == MLAN_BSS_TYPE_STA) && (pmpriv->enable_tcp_ack_enh == MTRUE) &&
            (pmpriv->curr_bss_params.bss_descriptor.phe_cap != NULL))
    {
#if CONFIG_TX_RX_ZERO_COPY
        ret = wlan_is_tcp_ack(pmpriv, sd_buffer);
#else
        ret = wlan_is_tcp_ack(pmpriv, buffer);
#endif
        if (ret)
        {
            if (pmpriv->curr_bss_params.bss_descriptor.curr_bandwidth == BW_80MHZ)
            {
                if (pmpriv->curr_bss_params.bss_descriptor.phe_cap != NULL)
                {
                    tx_control = (RATEID_HE_MCS9_1SS_BW80 << 16) | TXPD_TXRATE_ENABLE;
                }
                else if (pmpriv->curr_bss_params.bss_descriptor.pvht_cap != NULL)
                {
                    tx_control = (RATEID_VHT_MCS9_1SS_BW80 << 16) | TXPD_TXRATE_ENABLE;
                }
#if CONFIG_WMM
                /* Though TID is not used in case of TCP traffic,
                 * but making tid as voice traffic is a way to tell firmware to not to use
                 * MCS rates above 9 for TCP ack packets.
                 * This introduction is harmless for TCP traffic as TID is dont care
                 * */
                tid = WMM_HIGHEST_PRIORITY - 1;
#endif
            }
            else if (pmpriv->curr_bss_params.bss_descriptor.curr_bandwidth == BW_40MHZ)
            {
                if (pmpriv->curr_bss_params.bss_descriptor.phe_cap != NULL)
                {
                    tx_control = (RATEID_HE_MCS4_1SS_BW40 << 16) | TXPD_TXRATE_ENABLE;
                }
                else if (pmpriv->curr_bss_params.bss_descriptor.pvht_cap != NULL)
                {
                    tx_control = (RATEID_VHT_MCS8_1SS_BW40 << 16) | TXPD_TXRATE_ENABLE;
                }
#if CONFIG_WMM
                /* Though TID is not used in case of TCP traffic,
                 * but making tid as voice traffic is a way to tell firmware to not to use
                 * MCS rates above 9 for TCP ack packets.
                 * This introduction is harmless for TCP traffic as TID is dont care
                 * */
                tid = WMM_HIGHEST_PRIORITY - 1;
#endif
            }
            else if (pmpriv->curr_bss_params.bss_descriptor.curr_bandwidth == BW_20MHZ)
            {
                if (pmpriv->curr_bss_params.bss_descriptor.phe_cap != NULL)
                {
                    tx_control = (RATEID_HE_MCS7_1SS_BW20 << 16) | TXPD_TXRATE_ENABLE;
                }
                else if (pmpriv->curr_bss_params.bss_descriptor.pvht_cap != NULL)
                {
                    tx_control = (RATEID_VHT_MCS7_1SS_BW20 << 16) | TXPD_TXRATE_ENABLE;
                }
            }
        }
    }
    else if (interface == MLAN_BSS_TYPE_UAP)
    {
        ret = wlan_is_tcp_ack(pmpriv, buffer);
        if (ret)
        {
            if (wm_wifi.bandwidth == BANDWIDTH_80MHZ)
            {
                if (mlan_adap->usr_dot_11ax_enable == MTRUE)
                {
                    tx_control = (RATEID_HE_MCS9_1SS_BW80 << 16) | TXPD_TXRATE_ENABLE;
                }
                else if (mlan_adap->usr_dot_11ac_enable == MTRUE)
                {
                    tx_control = (RATEID_VHT_MCS9_1SS_BW80 << 16) | TXPD_TXRATE_ENABLE;
                }
            }
            else if (wm_wifi.bandwidth == BANDWIDTH_40MHZ)
            {
                if (mlan_adap->usr_dot_11ax_enable == MTRUE)
                {
                    tx_control = (RATEID_HE_MCS8_1SS_BW40 << 16) | TXPD_TXRATE_ENABLE;
                }
                else if (mlan_adap->usr_dot_11ac_enable == MTRUE)
                {
                    tx_control = (RATEID_VHT_MCS8_1SS_BW40 << 16) | TXPD_TXRATE_ENABLE;
                }
            }
            else if (wm_wifi.bandwidth == BANDWIDTH_20MHZ)
            {
                if (mlan_adap->usr_dot_11ax_enable == MTRUE)
                {
                    tx_control = (RATEID_HE_MCS7_1SS_BW20 << 16) | TXPD_TXRATE_ENABLE;
                }
                else if (mlan_adap->usr_dot_11ac_enable == MTRUE)
                {
                    tx_control = (RATEID_VHT_MCS7_1SS_BW20 << 16) | TXPD_TXRATE_ENABLE;
                }
            }
        }
    }
#endif /** CONFIG_TCP_ACK_ENH */
#endif /** CONFIG_11AX */
#endif

#if CONFIG_WMM
    /* process packet headers with interface header and TxPD */
    process_pkt_hdrs((void *)(sd_buffer + sizeof(mlan_linked_list)), len - sizeof(mlan_linked_list), interface, tid,
                     tx_control);

    /* add buffer to ra lists */
    if (wlan_wmm_add_buf_txqueue_enh(interface, sd_buffer, len, pkt_prio) != MLAN_STATUS_SUCCESS)
    {
        wifi_wmm_drop_no_media(interface);
        ret = -WM_E_BUSY;
        goto exit_fn;
    }

    send_wifi_driver_tx_data_event(interface);
#else
    //wifi_tx_card_awake_lock();
#if defined(RW610)
    wifi_imu_lock();
#else
    //(void)wifi_sdio_lock();
#endif

    while (true)
    {
        //i = wlan_xmit_pkt((t_u8 *)sd_buffer, len, interface, tx_control);
        int skw_send_ethdata(void *skb, int buffer_len);
        i = skw_send_ethdata((void *)sd_buffer, len);
#if defined(RW610)
        wifi_imu_unlock();
#else
        // wifi_sdio_unlock();
#endif
        if (i == MLAN_STATUS_SUCCESS)
        {
            break;
        }
        else
        {
            if (i == MLAN_STATUS_FAILURE)
            {
                ret = -WM_E_NOMEM;
                goto exit_fn;
            }
            else if (i == MLAN_STATUS_RESOURCE)
            {
                if (retry == 0)
                {
                    ret = -WM_E_BUSY;
                    goto exit_fn;
                }
                else
                {
                    retry--;
                    /* Allow the other thread to run and hence
                     * update the write bitmap so that pkt
                     * can be sent to FW */
                    OSA_TimeDelay(1);
#if defined(RW610)
                    wifi_imu_lock();
#else
                    (void)wifi_sdio_lock();
#endif
                    continue;
                }
            }
            else
            {
                /* Do Nothing */
            }
            break;
        } /* if (i != MLAN_STATUS_SUCCESS) */
    }     /* while(true) */

    //wifi_tx_card_awake_unlock();
#endif

    if (interface == BSS_TYPE_STA && sta_ampdu_tx_enable
#if CONFIG_WMM
            && wifi_sta_ampdu_tx_enable_per_tid_is_allowed(tid)
#endif
       )
    {
        if (wm_wifi.wrapper_net_is_ip_or_ipv6_callback(buffer))
        {
            (void)wrapper_wlan_sta_ampdu_enable(
#if CONFIG_WMM
                tid
#endif
            );
        }
    }

    if (interface == BSS_TYPE_UAP && uap_ampdu_tx_enable
#if CONFIG_WMM
            && wifi_uap_ampdu_tx_enable_per_tid_is_allowed(tid)
#endif
       )
    {
        if (wm_wifi.wrapper_net_is_ip_or_ipv6_callback(buffer))
        {
            (void)wrapper_wlan_uap_ampdu_enable((uint8_t *)buffer
#if CONFIG_WMM
                                                ,
                                                tid
#endif
                                               );
        }
    }

    ret = WM_SUCCESS;

exit_fn:

#if !CONFIG_WMM
    if (ret != WM_SUCCESS)
    {
        // wifi_tx_card_awake_unlock();
    }
#endif

    wifi_set_xfer_pending(false);
    // wakelock_put(WL_ID_LL_OUTPUT);

    return ret;
}
uint8_t out_putbuf[2048] __attribute__((aligned(32)));
uint8_t *wifi_get_outbuf(uint32_t *outbuf_len)
{
#if defined(RW610)
    return wifi_get_imu_outbuf(outbuf_len);
#else
    return (uint8_t *)out_putbuf;//wifi_get_sdio_outbuf(outbuf_len);
#endif
}

#if CONFIG_HEAP_DEBUG
static bool get_os_mem_stat_index(char const *func, t_u32 *index)
{
    int i     = 0;
    t_u32 len = strlen(func);

    len = (len > MAX_FUNC_SYMBOL_LEN - 1) ? (MAX_FUNC_SYMBOL_LEN - 1) : len;

    for (i = 0; i < valid_item_cnt; i++)
    {
        if (!strncmp(wifi_os_mem_stat[i].name, func, len))
        {
            // Find matched item
            *index = i;
            return true;
        }
    }

    if (valid_item_cnt >= OS_MEM_STAT_TABLE_SIZE)
    {
        (void)PRINTF("os_mem_stat table full\r\n");
        *index = OS_MEM_STAT_TABLE_SIZE - 1;
    }
    else
    {
        // Add a new item, increase valid_item_cnt
        *index = valid_item_cnt;
        valid_item_cnt++;
    }

    return false;
}

static int record_os_mem_item(t_u32 size, char const *func, t_u32 line_num, bool is_alloc)
{
    t_u32 index = 0;
    t_u32 len   = strlen(func);

    len = (len > MAX_FUNC_SYMBOL_LEN - 1) ? (MAX_FUNC_SYMBOL_LEN - 1) : len;

    // If don't get matched item, record stat in new item; else just increase alloc_cnt or free_cnt.
    if (false == get_os_mem_stat_index(func, &index))
    {
        wifi_os_mem_stat[index].line_num = line_num;

        if (true == is_alloc)
        {
            wifi_os_mem_stat[index].size = size;
        }

        memcpy(wifi_os_mem_stat[index].name, func, len);
    }

    return index;
}

void record_os_mem_alloc(t_u32 size, char const *func, t_u32 line_num)
{
    int index     = 0;
    bool is_alloc = true;

    OSA_SemaphoreWait((osa_semaphore_handle_t)os_mem_stat_sem, osaWaitForever_c);
    index = record_os_mem_item(size, func, line_num, is_alloc);
    wifi_os_mem_stat[index].alloc_cnt++;
    OSA_SemaphorePost((osa_semaphore_handle_t)os_mem_stat_sem);
}

void record_os_mem_free(char const *func, t_u32 line_num)
{
    int index     = 0;
    t_u32 size    = 0;
    bool is_alloc = false;

    OSA_SemaphoreWait((osa_semaphore_handle_t)os_mem_stat_sem, osaWaitForever_c);
    index = record_os_mem_item(size, func, line_num, is_alloc);
    wifi_os_mem_stat[index].free_cnt++;
    OSA_SemaphorePost((osa_semaphore_handle_t)os_mem_stat_sem);
}

void wifi_show_os_mem_stat()
{
    int index = 0;

    (void)PRINTF("os_mem_alloc_stat: \r\n");
    (void)PRINTF(
        "Func name                                                         line_num    size        alloc_cnt    "
        "free_cnt\r\n");

    for (index = 0; index < valid_item_cnt; index++)
    {
        (void)PRINTF("%-64s  %-10d  %-10d  %-10d   %-10d \r\n", wifi_os_mem_stat[index].name,
                     wifi_os_mem_stat[index].line_num, wifi_os_mem_stat[index].size, wifi_os_mem_stat[index].alloc_cnt,
                     wifi_os_mem_stat[index].free_cnt);
    }
}
#endif

/**
 * Frame Tx - Injecting Wireless frames from Host
 *
 * This function is used to Inject Wireless frames from application
 * directly.
 *
 * \param[in] interface Interface on which frame to be injected.
 * \param[in] buf Buffer holding 802.11 Wireless frame (Header + Data).
 * \param[in] len Length of the 802.11 Wireless frame.
 *
 * \return WM_SUCCESS on success or error code.
 *
 */
static int raw_low_level_output(const t_u8 interface, const t_u8 *buf, t_u32 len)
{
#if (CONFIG_WMM)
    t_u32 pkt_len            = 0;
    t_u32 link_point_len     = 0;
    bypass_outbuf_t *poutbuf = NULL;

    /*Dword align*/
    pkt_len        = sizeof(TxPD) + INTF_HEADER_LEN - 2;
    link_point_len = sizeof(mlan_linked_list);

#if !CONFIG_MEM_POOLS
    poutbuf = OSA_MemoryAllocate(link_point_len + pkt_len + len);
#else
    poutbuf = (bypass_outbuf_t *)OSA_MemoryPoolAllocate(buf_1536_MemoryPool);
#endif
    if (!poutbuf)
    {
        wuap_e("[%s] ERR:Cannot allocate buffer!\r\n", __func__);
        return -WM_FAIL;
    }

    (void)memset((t_u8 *)poutbuf, 0, link_point_len + pkt_len);

    (void)raw_process_pkt_hdrs((t_u8 *)poutbuf + link_point_len, pkt_len + len, interface);
    (void)memcpy((void *)((t_u8 *)poutbuf + link_point_len + pkt_len), (const void *)buf, (size_t)len);
    /* process packet headers with interface header and TxPD */
    process_pkt_hdrs((void *)((t_u8 *)poutbuf + link_point_len), pkt_len + len, interface, 0, 0);

    wlan_add_buf_bypass_txq((t_u8 *)poutbuf, interface);
    send_wifi_driver_bypass_data_event(interface);

    return WM_SUCCESS;
#else
    mlan_status i;
    t_u32 pkt_len       = 0;
    uint32_t outbuf_len = 0;
    uint8_t *poutbuf    = wifi_get_outbuf(&outbuf_len);

    pkt_len = sizeof(TxPD) + INTF_HEADER_LEN;

    wifi_tx_card_awake_lock();
#if defined(RW610)
    wifi_imu_lock();
#else
    (void)wifi_sdio_lock();
#endif

    (void)memset(poutbuf, 0, pkt_len);

    (void)raw_process_pkt_hdrs((t_u8 *)poutbuf, pkt_len + len - 2U, interface);
    (void)memcpy((void *)((t_u8 *)poutbuf + pkt_len - 2), (const void *)buf, (size_t)len);
    i = wlan_xmit_pkt(poutbuf, pkt_len + len - 2U, interface, 0);

#if defined(RW610)
    wifi_imu_unlock();
#else
    wifi_sdio_unlock();
#endif
    wifi_tx_card_awake_unlock();

    if (i == MLAN_STATUS_FAILURE)
    {
        return (int) - WM_FAIL;
    }

    wifi_set_xfer_pending(false);
    return WM_SUCCESS;

#endif
}

int wifi_inject_frame(const enum wlan_bss_type bss_type, const uint8_t *buff, const size_t len)
{
    return raw_low_level_output((t_u8)bss_type, buff, len);
}

#if CONFIG_WPS2
int wps_low_level_output(const uint8_t interface, const uint8_t *buf, const uint16_t len)
{
    mlan_status i;
    t_u32 pkt_len;
    uint32_t outbuf_len = 0;

    uint8_t *outbuf = wifi_get_outbuf(&outbuf_len);
    if (!outbuf)
        return (int) - WM_FAIL;

    pkt_len = sizeof(TxPD) + INTF_HEADER_LEN;
    if ((pkt_len + len) > outbuf_len)
    {
        return (int) - WM_FAIL;
    }

    wifi_tx_card_awake_lock();
#if defined(RW610)
    wifi_imu_lock();
#else
    wifi_sdio_lock();
#endif

    (void)memset(outbuf, 0x00, pkt_len);

    (void)memcpy((t_u8 *)outbuf + pkt_len, buf, len);

    i = wlan_xmit_pkt(outbuf, pkt_len + len, interface, 0);

#if defined(RW610)
    wifi_imu_unlock();
#else
    wifi_sdio_unlock();
#endif
    wifi_tx_card_awake_unlock();

    if (i == MLAN_STATUS_FAILURE)
    {
        return (int) - WM_FAIL;
    }

    return WM_SUCCESS;
}
#endif /* CONFIG_WPS2 */

int wifi_set_country_code(const char *alpha2)
{
    mlan_adapter *pmadapter             = (mlan_adapter *)mlan_adap;
    t_u8 country_code[COUNTRY_CODE_LEN] = {0};

#ifdef OTP_CHANINFO
    if (pmadapter->otp_region && pmadapter->otp_region->force_reg)
    {
        wifi_e("ForceRegionRule is set in the on-chip OTP memory");
        return -WM_FAIL;
    }
#endif

    (void)memcpy(country_code, alpha2, COUNTRY_CODE_LEN - 1);

    pmadapter->region_code = region_string_2_region_code(country_code);

    pmadapter->cfp_code_bg = pmadapter->region_code;
#if CONFIG_5GHz_SUPPORT
    pmadapter->cfp_code_a = pmadapter->region_code;
#endif

    if (wlan_set_regiontable(pmadapter->priv[1], pmadapter->region_code, pmadapter->config_bands))
    {
        wifi_e("%s set regiontable fail", __func__);
        return -WM_FAIL;
    }
    (void)memcpy(pmadapter->country_code, country_code, COUNTRY_CODE_LEN);

#if CONFIG_WPA_SUPP
    if (wm_wifi.supp_if_callbk_fns->chan_list_changed_callbk_fn)
    {
        wm_wifi.supp_if_callbk_fns->chan_list_changed_callbk_fn(wm_wifi.if_priv, alpha2);
    }
#endif

    return WM_SUCCESS;
}

int wifi_get_country_code(char *alpha2)
{
    (void)memcpy(alpha2, mlan_adap->country_code, COUNTRY_CODE_LEN - 1);

    return WM_SUCCESS;
}

int wifi_set_country_ie_ignore(uint8_t *ignore)
{
    mlan_adap->country_ie_ignore = *ignore;
    return WM_SUCCESS;
}

#if CONFIG_WPA_SUPP
int wifi_nxp_scan_res_num(void)
{
    mlan_private *pmpriv    = (mlan_private *)mlan_adap->priv[0];
    mlan_adapter *pmadapter = pmpriv->adapter;
    wifi_d("%s num_in_scan_table=%d\n", __func__, pmadapter->num_in_scan_table);
    return pmadapter->num_in_scan_table;
}

int wifi_nxp_scan_res_get2(t_u32 table_idx, nxp_wifi_event_new_scan_result_t *scan_res)
{
    mlan_private *pmpriv    = (mlan_private *)mlan_adap->priv[0];
    mlan_adapter *pmadapter = pmpriv->adapter;
    struct os_time t;
    BSSDescriptor_t *bss_new_entry;

    bss_new_entry = &pmadapter->pscan_table[table_idx];
    memcpy(scan_res->mac_addr, bss_new_entry->mac_address, sizeof(bss_new_entry->mac_address));
    scan_res->frequency  = channel_to_frequency(bss_new_entry->channel, (bss_new_entry->bss_band == BAND_A ? 1 : 0));
    scan_res->chan_width = bss_new_entry->curr_bandwidth;
    scan_res->beacon_interval = bss_new_entry->beacon_period;
    memcpy(&scan_res->capability, &bss_new_entry->cap_info, sizeof(unsigned short));
    memcpy(&scan_res->ies_tsf, bss_new_entry->time_stamp, sizeof(bss_new_entry->time_stamp));
    os_get_time(&t);
    scan_res->seen_ms_ago = t.sec * 1000 - bss_new_entry->scan_result_tsf / 1000000 * 1000;
    if (bss_new_entry->ies_len > 0)
    {
        scan_res->ies.ie     = bss_new_entry->ies;
        //bss_new_entry->ies   = NULL;//这里可能会造成内存泄漏
        scan_res->ies.ie_len = (t_u16)bss_new_entry->ies_len;
    }
    else
    {
        scan_res->ies.ie_len = (t_u16)0U;
    }

    scan_res->rssi = (t_u8) - (bss_new_entry->rssi);
#if CONFIG_SCAN_CHANNEL_GAP
    scan_res->noise = bss_new_entry->chan_noise;
#endif

    if ((pmpriv->media_connected == MTRUE) &&
            (memcmp(bss_new_entry->mac_address, (t_u8 *)&pmpriv->curr_bss_params.bss_descriptor.mac_address,
                    MLAN_MAC_ADDR_LENGTH) == 0U))

    {
        scan_res->status = 1;
    }
    return WM_SUCCESS;
}

void wifi_nxp_reset_scan_flag()
{
    mlan_adap->wpa_supp_scan_triggered = MFALSE;
}

int wifi_nxp_survey_res_get(void)
{
#if CONFIG_SCAN_CHANNEL_GAP
    mlan_private *pmpriv          = (mlan_private *)mlan_adap->priv[0];
    ChanStatistics_t *pchan_stats = NULL;
    mlan_scan_resp scan_resp;
    t_u32 idx;
    nxp_wifi_event_new_survey_result_t survey_res;
    bool more_res = true;
#endif

    ENTER();
    wifi_d("dump_survey");

#if CONFIG_SCAN_CHANNEL_GAP
    memset(&scan_resp, 0, sizeof(scan_resp));
    wifi_get_scan_table(pmpriv, &scan_resp);

    pchan_stats = (ChanStatistics_t *)scan_resp.pchan_stats;

    for (idx = 0; idx < scan_resp.num_in_chan_stats; idx++)
    {
        if (pchan_stats[idx].chan_num == 0)
        {
            if (wm_wifi.supp_if_callbk_fns->survey_res_callbk_fn)
            {
#if CONFIG_HOSTAPD
                if (wm_wifi.hostapd_op)
                {
                    wm_wifi.supp_if_callbk_fns->survey_res_callbk_fn(wm_wifi.hapd_if_priv, NULL, 0, false);
                }
                else
#endif
                {
                    wm_wifi.supp_if_callbk_fns->survey_res_callbk_fn(wm_wifi.if_priv, NULL, 0, false);
                }

                OSA_TimeDelay(50);
            }

            break;
        }

        memset(&survey_res, 0x00, sizeof(nxp_wifi_event_new_survey_result_t));

        survey_res.freq         = channel_to_frequency(pchan_stats[idx].chan_num, pchan_stats[idx].bandcfg.chanBand);
        survey_res.nf           = pchan_stats[idx].noise;
        survey_res.channel_time = pchan_stats[idx].cca_scan_duration;
        survey_res.channel_time_busy = pchan_stats[idx].cca_busy_duration;

        if (pchan_stats[idx + 1].chan_num == 0)
        {
            more_res = false;
        }
        if (wm_wifi.supp_if_callbk_fns->survey_res_callbk_fn)
        {
#if CONFIG_HOSTAPD
            if (wm_wifi.hostapd_op)
            {
                wm_wifi.supp_if_callbk_fns->survey_res_callbk_fn(wm_wifi.hapd_if_priv, &survey_res,
                        sizeof(nxp_wifi_event_new_survey_result_t), more_res);
            }
            else
#endif
            {
                wm_wifi.supp_if_callbk_fns->survey_res_callbk_fn(wm_wifi.if_priv, &survey_res,
                        sizeof(nxp_wifi_event_new_survey_result_t), more_res);
            }

            OSA_TimeDelay(50);
        }
    }
#else
    if (wm_wifi.supp_if_callbk_fns->survey_res_callbk_fn)
    {
#if CONFIG_HOSTAPD
        if (wm_wifi.hostapd_op)
        {
            wm_wifi.supp_if_callbk_fns->survey_res_callbk_fn(wm_wifi.hapd_if_priv, NULL, 0, false);
        }
        else
#endif
        {
            wm_wifi.supp_if_callbk_fns->survey_res_callbk_fn(wm_wifi.if_priv, NULL, 0, false);
        }

        OSA_TimeDelay(50);
    }
#endif
    return WM_SUCCESS;
}

#if CONFIG_WPA_SUPP_WPS
bool wifi_nxp_wps_session_enable(void)
{
    mlan_private *pmpriv = (mlan_private *)mlan_adap->priv[0];

    return pmpriv->wps.session_enable;
}
#endif


static int supp_low_level_output(const t_u8 interface, const t_u8 *buf, t_u32 len)
{
#if (CONFIG_WMM)
    t_u32 pkt_len            = 0;
    t_u32 link_point_len     = 0;
    bypass_outbuf_t *poutbuf = NULL;

    /*Dword align*/
    pkt_len        = sizeof(TxPD) + INTF_HEADER_LEN;
    link_point_len = sizeof(mlan_linked_list);

#if !CONFIG_MEM_POOLS
    poutbuf = OSA_MemoryAllocate(link_point_len + pkt_len + len);
#else
    poutbuf = (bypass_outbuf_t *)OSA_MemoryPoolAllocate(buf_1536_MemoryPool);
#endif
    if (!poutbuf)
    {
        wuap_e("[%s] ERR:Cannot allocate buffer!\r\n", __func__);
        return -WM_FAIL;
    }

    (void)memset((t_u8 *)poutbuf, 0, link_point_len + pkt_len);

    (void)memcpy((void *)((t_u8 *)poutbuf + link_point_len + pkt_len), (const void *)buf, (size_t)len);
    /* process packet headers with interface header and TxPD */
    process_pkt_hdrs((void *)((t_u8 *)poutbuf + link_point_len), pkt_len + len, interface, 0, 0);

    wlan_add_buf_bypass_txq((t_u8 *)poutbuf, interface);
    send_wifi_driver_bypass_data_event(interface);

    return WM_SUCCESS;
#else

#if 1
    mlan_status i;
    uint32_t pkt_len, outbuf_len;

    uint8_t *outbuf = wifi_get_outbuf(&outbuf_len);

    if (!outbuf)
    {
        return (int) - WM_FAIL;
    }

    pkt_len = len;//sizeof(TxPD) + INTF_HEADER_LEN;
    wifi_d("%s %d len=%d interface=%d\n", __func__, __LINE__, interface);
    if ((len + pkt_len) > outbuf_len)
    {
        return (int) - WM_FAIL;
    }

    //wifi_tx_card_awake_lock();
#if defined(RW610)
    wifi_imu_lock();
#else
    //wifi_sdio_lock();
#endif

    (void)memset(outbuf, 0x00, pkt_len);
    wifi_d("%s %d outbuf=%p len=%d\n", __func__, __LINE__, outbuf, len);
    (void)memcpy((t_u8 *)outbuf, buf, len);
    wifi_d("%s %d outbuf=%p len=%d\n", __func__, __LINE__, outbuf, len);
    // i = wlan_xmit_pkt(outbuf, pkt_len + len, interface, 0);
    wifi_low_level_output(0/*STA MODE*/, outbuf, len);

#if defined(RW610)
    wifi_imu_unlock();
#else
    //wifi_sdio_unlock();
#endif
    //wifi_tx_card_awake_unlock();

    // if (i == MLAN_STATUS_FAILURE)
    // {
    //     return (int)-WM_FAIL;
    // }
#endif
    wifi_set_xfer_pending(false);
    return (int)WM_SUCCESS;

#endif
}

int wifi_supp_inject_frame(const unsigned int bss_type, const uint8_t *buff, const size_t len)
{
    return supp_low_level_output((t_u8)bss_type, buff, len);
}

/**
 *  Alpha2 may has only 2 octets.
 *  Need to avoid accessing the third octet.
 *  If On-Chip OTP memory sets ForceRegion Rule, set country may return fail.
 *  Ignore it to not let it block AP setup.
 */
int wifi_nxp_set_country(unsigned int bss_type, const char *alpha2)
{
    (void)wifi_set_country_code(alpha2);
    return WM_SUCCESS;
}

/**
 *  Alpha2 may has only 2 octets.
 *  Need to avoid accessing the third octet.
 */
int wifi_nxp_get_country(unsigned int bss_type, char *alpha2)
{
    return wifi_get_country_code(alpha2);
}

int wifi_nxp_get_signal(unsigned int bss_type, nxp_wifi_signal_info_t *signal_params)
{
    wifi_rssi_info_t rssi_info;

    (void)wifi_send_rssi_info_cmd(&rssi_info);

    signal_params->current_signal    = rssi_info.bcn_rssi_last;
    signal_params->avg_signal        = rssi_info.data_rssi_avg;
    signal_params->avg_beacon_signal = rssi_info.bcn_rssi_avg;
    signal_params->current_noise     = rssi_info.bcn_nf_last;

    return WM_SUCCESS;
}

int wifi_nxp_send_mlme(unsigned int bss_type, int channel, unsigned int wait_time, const t_u8 *data, size_t data_len)
{
    mlan_private *pmpriv              = (mlan_private *)mlan_adap->priv[bss_type];
    wlan_mgmt_pkt *pmgmt_pkt_hdr      = MNULL;
    wlan_802_11_header *pieee_pkt_hdr = MNULL;
    t_u8 buf[1580];

    // dump_hex(data, data_len);
    memset(buf, 0x00, sizeof(buf));

    if ((bss_type == BSS_TYPE_STA) && (pmpriv->media_connected == MFALSE))
    {
        if (wait_time == 0)
        {
            wait_time = 1000;
        }
        wifi_remain_on_channel(true, channel, wait_time);
    }

    pmgmt_pkt_hdr = (wlan_mgmt_pkt *)&buf[0];

    pmgmt_pkt_hdr->frm_len = data_len + MLAN_MAC_ADDR_LENGTH;

    pieee_pkt_hdr = (wlan_802_11_header *)(void *)&pmgmt_pkt_hdr->wlan_header;

    memcpy(pieee_pkt_hdr, data, sizeof(wlan_802_11_header) - MLAN_MAC_ADDR_LENGTH);
    // coverity[overrun-local:SUPPRESS]
    memcpy(pieee_pkt_hdr + 1, data + sizeof(wlan_802_11_header) - MLAN_MAC_ADDR_LENGTH,
           data_len - (sizeof(wlan_802_11_header) - MLAN_MAC_ADDR_LENGTH));

    data_len = pmgmt_pkt_hdr->frm_len + 2U;

    return wifi_inject_frame((enum wlan_bss_type)bss_type, buf, data_len);
}
#else
int wifi_supp_inject_frame(const unsigned int bss_type, const uint8_t *buff, const size_t len)
{
    (void)bss_type;
    (void)buff;
    (void)len;

    return WM_SUCCESS;
}
#endif

bool wifi_is_remain_on_channel(void)
{
    return (mlan_adap->remain_on_channel ? true : false);
}

int wifi_remain_on_channel(const bool status, const uint8_t channel, const uint32_t duration)
{
    wifi_remain_on_channel_t roc;

    (void)memset(&roc, 0x00, sizeof(wifi_remain_on_channel_t));

    roc.remove = (uint16_t)!status;

    roc.channel = channel;

    roc.remain_period = duration;

#if CONFIG_5GHz_SUPPORT
    if (channel > 14)
    {
        roc.bandcfg |= 1;
    }
#endif
#if CONFIG_WMM
    if (true == status)
    {
        /* Block tx data before send remain on channel,
         * then get txbuf_sem, keep the next auth frame can get txbuf
         */
        wifi_set_tx_status(WIFI_DATA_BLOCK);

        if (wifi_txbuf_available() == MFALSE)
        {
            mlan_adap->wait_txbuf = true;
            OSA_SemaphoreWait((osa_semaphore_handle_t)txbuf_sem, osaWaitForever_c);
            mlan_adap->wait_txbuf = false;
        }
    }
    else if (false == status)
    {
        /* Restore tx when cancel remain on channel*/
        wifi_set_tx_status(WIFI_DATA_RUNNING);

        send_wifi_driver_tx_data_event(MLAN_BSS_TYPE_STA);
        send_wifi_driver_tx_data_event(MLAN_BSS_TYPE_UAP);
    }
#endif

    return wifi_send_remain_on_channel_cmd(MLAN_BSS_TYPE_STA, &roc);
}

#ifdef RW610
int wifi_imu_get_task_lock(void)
{
    return imu_get_task_lock();
}

int wifi_imu_put_task_lock(void)
{
    return imu_put_task_lock();
}
#endif

#if CONFIG_CSI

/* csi data recv user callback */
int (*csi_data_recv)(void *buffer, size_t len) = NULL;

int register_csi_user_callback(int (*csi_data_recv_callback)(void *buffer, size_t len))
{
    csi_data_recv = csi_data_recv_callback;

    return WM_SUCCESS;
}

int unregister_csi_user_callback(void)
{
    csi_data_recv = NULL;

    return WM_SUCCESS;
}

void process_csi_info_callback(void *data, size_t len)
{
    if (csi_data_recv != NULL)
    {
        csi_data_recv(data, len);
    }
}

void csi_local_buff_init()
{
    csi_event_cnt      = 0;
    csi_event_data_len = 0;

    csi_buff_stat.write_index    = 0;
    csi_buff_stat.read_index     = 0;
    csi_buff_stat.valid_data_cnt = 0;

    memset(csi_local_buff, 0x00, sizeof(csi_local_buff));
}

void csi_save_data_to_local_buff(void *data)
{
    OSA_SemaphoreWait((osa_semaphore_handle_t)csi_buff_stat.csi_data_sem, osaWaitForever_c);

    if (csi_buff_stat.valid_data_cnt >= MAX_CSI_LOCAL_BUF)
    {
        OSA_SemaphorePost((osa_semaphore_handle_t)csi_buff_stat.csi_data_sem);
        wifi_w("******csi_local_buff is full******\r\n");
        return;
    }

    memcpy(&csi_local_buff[csi_buff_stat.write_index][0], (t_u8 *)data, CSI_LOCAL_BUF_ENTRY_SIZE);

    csi_buff_stat.valid_data_cnt++;

    csi_buff_stat.write_index = (csi_buff_stat.write_index + 1) % MAX_CSI_LOCAL_BUF;

    OSA_SemaphorePost((osa_semaphore_handle_t)csi_buff_stat.csi_data_sem);
}

void csi_deliver_data_to_user()
{
    int i               = 0;
    t_u16 save_data_len = 0;

    OSA_SemaphoreWait((osa_semaphore_handle_t)csi_buff_stat.csi_data_sem, osaWaitForever_c);

    for (i = 0; (i < MAX_CSI_LOCAL_BUF) && (csi_buff_stat.valid_data_cnt > 0); i++)
    {
        pcsi_record_ds csi_record = (pcsi_record_ds)(&csi_local_buff[csi_buff_stat.read_index][0]);
        save_data_len             = (csi_record->Len & 0x1fff) * 4;
        save_data_len = (save_data_len >= CSI_LOCAL_BUF_ENTRY_SIZE) ? CSI_LOCAL_BUF_ENTRY_SIZE : save_data_len;

        process_csi_info_callback((t_u8 *)csi_record, save_data_len);

        csi_buff_stat.valid_data_cnt--;

        csi_buff_stat.read_index = (csi_buff_stat.read_index + 1) % MAX_CSI_LOCAL_BUF;
    }

    OSA_SemaphorePost((osa_semaphore_handle_t)csi_buff_stat.csi_data_sem);
}
#endif
