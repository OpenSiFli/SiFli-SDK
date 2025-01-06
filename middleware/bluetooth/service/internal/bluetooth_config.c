/**
  ******************************************************************************
  * @file   bluetooth_config.c
  * @author Sifli software development team
  * @brief SIFLI bluetooth stack config implementation.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2021 - 2022,  Sifli Technology
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


#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include "rtthread.h"
#include "rthw.h"
#include "os_adaptor.h"
#include "board.h"
#include "ble_stack.h"
#include "ipc_queue.h"

#include "log.h"
#include "bf0_ble_common.h"
#include "bluetooth_int.h"

#ifndef BSP_USING_PC_SIMULATOR
    #include "drv_flash.h"
    #include "bluetooth_hci_flash.h"
    #include "bf0_pm.h"
#endif

#if defined(SOC_SF32LB58X) || defined(SOC_SF32LB56X) || defined(SOC_SF32LB52X) ||defined(BSP_USING_PC_SIMULATOR)

void ble_memory_config(ble_mem_config_t *config);

#define IO_MB_CH      (0)
#ifdef BF0_LCPU
    #define TX_BUF_SIZE   LCPU2HCPU_MB_CH1_BUF_SIZE
    #define TX_BUF_ADDR   LCPU2HCPU_MB_CH1_BUF_START_ADDR
    #define TX_BUF_ADDR_ALIAS LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH1_BUF_START_ADDR);
    #define RX_BUF_ADDR   HCPU_ADDR_2_LCPU_ADDR(HCPU2LCPU_MB_CH1_BUF_START_ADDR);
#elif defined(BF0_HCPU)
    #define TX_BUF_SIZE   HCPU2LCPU_MB_CH1_BUF_SIZE
    #define TX_BUF_ADDR   HCPU2LCPU_MB_CH1_BUF_START_ADDR
    #define TX_BUF_ADDR_ALIAS HCPU_ADDR_2_LCPU_ADDR(HCPU2LCPU_MB_CH1_BUF_START_ADDR);
    #define RX_BUF_ADDR   LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH1_BUF_START_ADDR);

    #define RX_BUF_REV_B_ADDR   LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH1_BUF_REV_B_START_ADDR);
#elif !defined(BSP_USING_PC_SIMULATOR)
    #error "invalid MB config"
#endif

#define GLOBAL_INT_DISABLE();                                               \
do {                                                                        \
    uint32_t __old;                                                         \
    __old = rt_hw_interrupt_disable();                                  \

/** @brief Restore interrupts from the previous global disable.
 * @sa GLOBAL_INT_DISABLE
 */
#define GLOBAL_INT_RESTORE();                                               \
    rt_hw_interrupt_enable(__old);                                      \
} while(0)


#if defined(SOC_SF32LB58X)
    #ifdef SF32LB58X_3SCO
        #define NVDS_BUFF_START NVDS_BUF_START_ADDR
    #else
        #define NVDS_BUFF_START 0x204FFD00
    #endif
    #define NVDS_BUFF_SIZE 512
#elif defined(SOC_SF32LB56X)
    #define NVDS_BUFF_START 0x2041FD00
    #define NVDS_BUFF_SIZE 512
#elif defined(SOC_SF32LB52X) && !defined(DFU_OTA_MANAGER)
    #define NVDS_BUFF_START 0x2040FE00
    #define NVDS_BUFF_SIZE 512
#else
    #define NVDS_BUFF_START 0
    #define NVDS_BUFF_SIZE 0
#endif


enum bt_eif_status
{
    BT_EIF_STATUS_OK,
    BT_EIF_STATUS_ERROR,
};

typedef enum
{
    TL_PORT_UART0,
    TL_PORT_UART1,
    TL_PORT_MB,
    TL_PORT_IPC,
    TL_PORT_FLASH,
    TL_PORT_LOG,
    TL_PORT_IDLE,
    TL_PORT_MAX
} tl_port_t;



typedef void (*bt_eif_callback)(void *, uint8_t);

struct bt_eif_api
{
    void (*read)(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy);
    void (*write)(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy);
    void (*flow_on)(void);
    bool (*flow_off)(void);
};

struct uart_txrxchannel
{
    /// call back function pointer
    void (*callback)(void *, uint8_t);
    /// Dummy data pointer returned to callback when operation is over.
    void *dummy;
    uint8_t *buf;
    uint32_t data_size;
    uint32_t offset;
};


struct uart_env_tag
{
    rt_device_t device;
    /// tx channel
    struct uart_txrxchannel tx;
    /// rx channel
    struct uart_txrxchannel rx;
    /// error detect
    uint8_t errordetect;
    /// external wakeup
    bool ext_wakeup;
};


struct mbox_env_tag
{
    ipc_queue_handle_t ipc_port;
    // Adapt for uart
    struct uart_txrxchannel tx;
    struct uart_txrxchannel rx;
    rt_timer_t  tx_timer;
    uint8_t is_init;
};

struct tl_ipc_slist_t
{
    rt_slist_t list;
    uint32_t buf_len;
    uint8_t buf[0];
};


struct tl_ipc_env_tag
{
    struct uart_txrxchannel rx;
    rt_slist_t list;
    struct tl_ipc_slist_t *cur_list;
    uint32_t offset;
    tl_data_callback_t callback;
    uint8_t is_data_process;
};


typedef const struct bt_eif_api *(*tl_port_config_cb_t)(void);

typedef struct
{
    tl_port_config_cb_t callback;
} tl_port_select_t;

#ifdef BF0_HCPU
typedef struct
{
    uint8_t trans_en;
    ipc_queue_handle_t ipc_port;
    rt_device_t uart_port;
    rt_mailbox_t to_mb;
    rt_mailbox_t to_uart;
    rt_thread_t  f2mb_th;
    rt_thread_t  f2uart_th;
    rt_err_t (*saved_rx_indicate)(rt_device_t dev, rt_size_t size);
    rt_thread_t  btrssi_th;
    int32_t    rssi_acc;
    int32_t    rssi_cnt;
    int8_t     rssi_ave;
    uint8_t no_signal_state; // 1: ble rx start; 2: bt rx start 0:stop
    rt_event_t  evt;
    uint16_t   saved_open_flag;
    uint8_t   *pbuf2mb;
    uint8_t   *pbuf2api;
} hci_forward_env_t;

static hci_forward_env_t g_hci_forward_env =
{
    .trans_en = 0,
    .f2mb_th  = NULL,
    .f2uart_th = NULL,
    .btrssi_th = NULL,
    .no_signal_state = 0,
    .pbuf2mb = NULL,
    .pbuf2api = NULL,
};

static hci_forward_env_t *hci_forward_get_env(void);
static rt_err_t hci_trans_uart_rx_ind(rt_device_t dev, rt_size_t size);
static int32_t hci_trans_mb_rx_ind(ipc_queue_handle_t dev, size_t size);
#endif

void uart0_flow_on(void);
bool uart0_flow_off(void);
void uart0_read(uint8_t *bufptr, uint32_t size, void (*callback)(void *, uint8_t), void *dummy);
void uart0_write(uint8_t *bufptr, uint32_t size, void (*callback)(void *, uint8_t), void *dummy);

void uart1_flow_on(void);
bool uart1_flow_off(void);
void uart1_read(uint8_t *bufptr, uint32_t size, void (*callback)(void *, uint8_t), void *dummy);
void uart1_write(uint8_t *bufptr, uint32_t size, void (*callback)(void *, uint8_t), void *dummy);

void uart_flow_on(struct uart_env_tag       *env);
bool uart_flow_off(struct uart_env_tag       *env);
void uart_read(struct uart_env_tag      *env, uint8_t *bufptr, uint32_t size, void (*callback)(void *, uint8_t), void *dummy);
void uart_write(struct uart_env_tag      *env, uint8_t *bufptr, uint32_t size, void (*callback)(void *, uint8_t), void *dummy);
static rt_err_t uart_rx_ind(struct uart_env_tag      *env, rt_device_t dev, rt_size_t size);
static rt_err_t uart_tx_complete(struct uart_env_tag      *env, rt_device_t dev, void *buffer);

#ifdef USING_IPC_QUEUE
    static void mbox_read(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy);
    static void mbox_write(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy);
    static void mbox_flow_on(void);
    static bool mbox_flow_off(void);
#endif

static uint32_t tl_ipc_read_data(uint8_t *bufptr, uint32_t size);
static void tl_ipc_read(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy);
static void tl_ipc_write(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy);
static void tl_ipc_flow_on(void);
static bool tl_ipc_flow_off(void);

#if defined(BF0_HCPU) && defined(HCI_ON_FLASH)
    static void tl_flash_read(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy);
    static void tl_flash_write(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy);
    static void tl_flash_flow_on(void);
    static bool tl_flash_flow_off(void);
#endif // defined(BF0_HCPU) && defined(HCI_ON_FLASH)


#ifdef HCI_ON_LOG
    static void tl_log_read(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy);
    static void tl_log_write(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy);
    static void tl_log_flow_on(void);
    static bool tl_log_flow_off(void);
#endif // HCI_ON_LOG

static void tl_idle_read(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy);
static void tl_idle_write(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy);
static void tl_idle_flow_on(void);
static bool tl_idle_flow_off(void);


const struct bt_eif_api *port_config_uart0(void);
const struct bt_eif_api *port_config_uart1(void);
const struct bt_eif_api *port_config_mailbox(void);
const struct bt_eif_api *port_config_ipc(void);
const struct bt_eif_api *port_config_flash(void);
const struct bt_eif_api *port_config_log(void);
const struct bt_eif_api *port_config_idle(void);





// Creation of uart external interface api
__ROM_USED const struct bt_eif_api uart0_api =
{
    uart0_read,
    uart0_write,
    uart0_flow_on,
    uart0_flow_off,
};

__ROM_USED const struct bt_eif_api uart1_api =
{
    uart1_read,
    uart1_write,
    uart1_flow_on,
    uart1_flow_off,
};


__ROM_USED struct uart_env_tag uart0_env;
__ROM_USED struct uart_env_tag uart1_env;





#ifdef USING_IPC_QUEUE
__ROM_USED const struct bt_eif_api mbox_api =
{
    mbox_read,
    mbox_write,
    mbox_flow_on,
    mbox_flow_off,
};

__ROM_USED struct mbox_env_tag mbox_env =
{
    .tx_timer = NULL,
};
#endif //USING_IPC_QUEUE

__ROM_USED const struct bt_eif_api tl_ipc_api =
{
    tl_ipc_read,
    tl_ipc_write,
    tl_ipc_flow_on,
    tl_ipc_flow_off,
};
__ROM_USED struct tl_ipc_env_tag tl_ipc_env;


#if defined(BF0_HCPU) && defined(HCI_ON_FLASH)
const struct bt_eif_api tl_flash_api =
{
    tl_flash_read,
    tl_flash_write,
    tl_flash_flow_on,
    tl_flash_flow_off,
};
#endif

#ifdef HCI_ON_LOG
const struct bt_eif_api tl_log_api =
{
    tl_log_read,
    tl_log_write,
    tl_log_flow_on,
    tl_log_flow_off,
};
#endif // HCI_ON_LOG


const struct bt_eif_api tl_idle_api =
{
    tl_idle_read,
    tl_idle_write,
    tl_idle_flow_on,
    tl_idle_flow_off,
};


__ROM_USED const tl_port_select_t tl_port_api[TL_PORT_MAX] =
{
    port_config_uart0,
    port_config_uart1,
    port_config_mailbox,
    port_config_ipc,
    port_config_flash,
    port_config_log,
    port_config_idle,
};


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
#ifndef BSP_USING_PC_SIMULATOR

__WEAK void ble_port_config(ipc_queue_cfg_t *cfg, rt_device_t ble_uart0, rt_device_t ble_uart1)
{
    // Do nothing
}

__WEAK void ble_memory_config(ble_mem_config_t *config)
{
    // Do nothing
}

#endif


void uart0_flow_on(void)
{
    uart_flow_on(&uart0_env);
}

bool uart0_flow_off(void)
{
    return uart_flow_off(&uart0_env);
}


void uart0_read(uint8_t *bufptr, uint32_t size, void (*callback)(void *, uint8_t), void *dummy)
{
    uart_read(&uart0_env, bufptr, size, callback, dummy);
}

void uart0_write(uint8_t *bufptr, uint32_t size, void (*callback)(void *, uint8_t), void *dummy)
{
    uart_write(&uart0_env, bufptr, size, callback, dummy);
}

#ifdef UART_PORT0
static rt_err_t uart0_rx_ind(rt_device_t dev, rt_size_t size)
{
    return uart_rx_ind(&uart0_env, dev, size);
}

static rt_err_t uart0_tx_complete(rt_device_t dev, void *buffer)
{
    return uart_tx_complete(&uart0_env, dev, buffer);
}
#endif // UART_PORT0

void uart1_flow_on(void)
{
    uart_flow_on(&uart1_env);
}

bool uart1_flow_off(void)
{
    return uart_flow_off(&uart1_env);
}


void uart1_read(uint8_t *bufptr, uint32_t size, void (*callback)(void *, uint8_t), void *dummy)
{
    uart_read(&uart1_env, bufptr, size, callback, dummy);
}

void uart1_write(uint8_t *bufptr, uint32_t size, void (*callback)(void *, uint8_t), void *dummy)
{
    uart_write(&uart1_env, bufptr, size, callback, dummy);
}

#ifdef UART_PORT1
//static rt_err_t uart1_rx_ind(rt_device_t dev, rt_size_t size)
//{
//    return uart_rx_ind(&uart1_env, dev, size);
//}

static rt_err_t uart1_tx_complete(rt_device_t dev, void *buffer)
{
    return uart_tx_complete(&uart1_env, dev, buffer);
}
#endif // UART_PORT1

void uart_flow_on(struct uart_env_tag       *env)
{

}

bool uart_flow_off(struct uart_env_tag       *env)
{
    return true;
}


void uart_read(struct uart_env_tag      *env, uint8_t *bufptr, uint32_t size, void (*callback)(void *, uint8_t), void *dummy)
{
    rt_size_t len;
    bool succ;

    // Sanity check
    RT_ASSERT(env != NULL);
    RT_ASSERT(bufptr != NULL);
    RT_ASSERT(size != 0);
    RT_ASSERT(callback != NULL);
    // If device not existed, just notify send successful.
    if (env->device == NULL)
    {
        if (callback)
            callback(dummy, BT_EIF_STATUS_OK);
        return;
    }

    GLOBAL_INT_DISABLE();
    len = rt_device_read(env->device, 0, bufptr, size);

    if (len == size)
    {
        succ = true;
    }
    else if (len < size)
    {
        succ = false;
        env->rx.callback = callback;
        env->rx.dummy    = dummy;
        env->rx.buf = bufptr;
        env->rx.data_size = size - len;
        env->rx.offset = len;
    }
    else
    {
        RT_ASSERT(0);
    }

    GLOBAL_INT_RESTORE();

    if (succ && callback)
    {
        callback(dummy, BT_EIF_STATUS_OK);
    }

}

void uart_write(struct uart_env_tag *env, uint8_t *bufptr, uint32_t size, void (*callback)(void *, uint8_t), void *dummy)
{
    // Sanity check
    RT_ASSERT(env != NULL);
    //RT_ASSERT(bufptr != NULL);
    RT_ASSERT(size != 0);

    // If device not existed, just notify send successful.
    if (env->device == NULL)
    {
        if (callback)
            callback(dummy, BT_EIF_STATUS_OK);
        return;
    }

    //    RT_ASSERT(callback != NULL);
    env->tx.callback = callback;
    env->tx.dummy    = dummy;
    env->tx.buf = bufptr;
    env->tx.data_size = size;

    // Clear streaming mode to avoid serial auto added "\0A"
    rt_uint16_t old_flag = env->device->open_flag;
    env->device->open_flag &= ~RT_DEVICE_FLAG_STREAM;

    rt_device_write(env->device, 0, bufptr, size);
    env->device->open_flag = old_flag;

    if (0 == (env->device->open_flag & RT_DEVICE_FLAG_DMA_TX))
    {
        if (NULL != callback)
        {
            callback(env->tx.dummy, BT_EIF_STATUS_OK);
        }
    }
}


static rt_err_t uart_rx_ind(struct uart_env_tag *env, rt_device_t dev, rt_size_t size)
{
    RT_ASSERT(env->device == dev);
    rt_size_t len;

    if (NULL == env->rx.buf)
        return RT_EOK;

    if (env->rx.data_size <= size)
    {
        len = rt_device_read(dev, 0, env->rx.buf + env->rx.offset, env->rx.data_size);
        RT_ASSERT(len == env->rx.data_size);
        env->rx.buf = NULL;
        if (NULL != env->rx.callback)
        {
            env->rx.callback(env->rx.dummy, BT_EIF_STATUS_OK);
        }
    }
    else
    {
        len = rt_device_read(dev, 0, env->rx.buf + env->rx.offset, size);
        RT_ASSERT(len == size);
        env->rx.data_size -= size;
        env->rx.offset += size;
    }

    return RT_EOK;
}

static rt_err_t uart_tx_complete(struct uart_env_tag *env, rt_device_t dev, void *buffer)
{
    RT_ASSERT(env->device == dev);
    RT_ASSERT(env->tx.buf == buffer);

    if (NULL != env->tx.callback)
    {
        env->tx.callback(env->tx.dummy, BT_EIF_STATUS_OK);
        env->tx.callback = NULL;
    }

    return RT_EOK;

}

RT_WEAK void blebredr_rf_power_set(uint8_t type, int8_t txpwr)
{
    return;
}


#ifdef USING_IPC_QUEUE
#include "mem_map.h"

#if 0
// rw 0: read, 1:write
static const char *mbox_get_device(uint8_t rw)
{
    uint8_t core = rom_config_get_ble_service_working_core();
    if (rw == 0)
        return mbox_port[core].read_dev;
    return mbox_port[core].write_dev;
}
#endif
static void mbox_read(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy)
{
    rt_size_t len;
    bool succ;
    struct mbox_env_tag *env = &mbox_env;

    // Sanity check
    RT_ASSERT(bufptr != NULL);
    RT_ASSERT(size != 0);
    RT_ASSERT(callback != NULL);

#ifdef BF0_HCPU
    hci_forward_env_t *hci_env = hci_forward_get_env();

    if (hci_env->trans_en)
    {
        return;
    }
#endif
    GLOBAL_INT_DISABLE();
    len = ipc_queue_read(env->ipc_port, bufptr, size);

    if (len == size)
    {
        succ = true;
    }
    else if (len < size)
    {
        succ = false;
        env->rx.callback = callback;
        env->rx.dummy    = dummy;
        env->rx.buf = bufptr;
        env->rx.data_size = size - len;
        env->rx.offset = len;
    }
    else
    {
        RT_ASSERT(0);
    }

    GLOBAL_INT_RESTORE();

    if (succ && callback)
    {
        callback(dummy, BT_EIF_STATUS_OK);
    }

}
static void mbox_tx_timeout(void *para)
{
    struct mbox_env_tag *env = &mbox_env;
    rt_size_t size, written, offset = 0;

    size = env->tx.data_size - env->tx.offset;
    offset = env->tx.offset;
    written = ipc_queue_write(env->ipc_port, env->tx.buf + offset, size, 10);

    LOG_D("mbox timeout write %d %d\n", written, size);

    if (written < size)
    {
        env->tx.offset += written;

        if (env->tx_timer != NULL)
        {
            rt_timer_start(env->tx_timer);
        }

    }
    else
    {
        if (NULL != env->tx.callback)
        {
            env->tx.callback(env->tx.dummy, BT_EIF_STATUS_OK);
        }
    }
}
static void mbox_write(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy)
{
    struct mbox_env_tag *env = &mbox_env;
    // Sanity check
    RT_ASSERT(bufptr != NULL);
    RT_ASSERT(size != 0);
    rt_size_t written;
    //RT_ASSERT(callback != NULL);
    env->tx.callback = callback;
    env->tx.dummy    = dummy;
    env->tx.buf = bufptr;
    env->tx.data_size = size;

#ifdef BF0_HCPU
    hci_forward_env_t *hci_env = hci_forward_get_env();

    if (hci_env->trans_en)
    {
        if (NULL != callback)
        {
            callback(env->tx.dummy, BT_EIF_STATUS_OK);
        }
        return;
    }
#endif

    written = ipc_queue_write(env->ipc_port, bufptr, size, 10);

    if (written < size)
    {
        LOG_D("mbox start timer write %d %d\n", written, size);
        env->tx.offset = written;
        if (env->tx_timer == NULL)
        {
            env->tx_timer = rt_timer_create("bttx_t", mbox_tx_timeout, RT_NULL, 50, RT_TIMER_FLAG_ONE_SHOT | RT_TIMER_FLAG_SOFT_TIMER);
        }

        if (env->tx_timer != NULL)
        {
            rt_timer_start(env->tx_timer);
        }

    }
    else
    {
        if (NULL != callback)
        {
            callback(env->tx.dummy, BT_EIF_STATUS_OK);
        }
    }

}

static void mbox_flow_on(void)
{
}


static bool mbox_flow_off(void)
{
    return true;
}

static int32_t mbox_rx_ind(ipc_queue_handle_t handle, size_t size)
{
    struct mbox_env_tag *env = &mbox_env;
    RT_ASSERT(env->ipc_port == handle);
    rt_size_t len;
#ifdef BF0_HCPU
    hci_forward_env_t *hci_env = hci_forward_get_env();

    if (hci_env->trans_en)
    {
        return hci_trans_mb_rx_ind(handle, size);
    }
#endif

    if (NULL == env->rx.buf)
        return 0;

    if (env->rx.data_size <= size)
    {
        len = ipc_queue_read(handle, env->rx.buf + env->rx.offset, env->rx.data_size);
        RT_ASSERT(len == env->rx.data_size);
        //LOG_D("mbox r l(%d, %d),d(%d)\r\n", len, env->rx.offset, *(env->rx.buf));
        env->rx.buf = NULL;

        if (NULL != env->rx.callback)
        {
            env->rx.callback(env->rx.dummy, BT_EIF_STATUS_OK);
        }
    }
    else
    {
        len = ipc_queue_read(handle, env->rx.buf + env->rx.offset, size);
        RT_ASSERT(len == size);
        env->rx.data_size -= size;
        env->rx.offset += size;
    }

    return 0;

}

#ifdef BF0_HCPU

#define  LOCAL2_BT_RSSI_START               (1 << 8)
#define  LOCAL2_BLE_RSSI_START              (1 << 9)
#define  LOCAL2_BT_NO_SIGNAL_START          (1 << 10)
#define  LOCAL2_BT_NO_SIGNAL_ISR            (1 << 11)
#define  LOCAL2_BT_NO_SIGNAL_RET            (1 << 12)
#define  LOCAL2_BT_RSSI_ALL  ( \
                                LOCAL2_BT_RSSI_START| \
                                LOCAL2_BLE_RSSI_START| \
                                LOCAL2_BT_NO_SIGNAL_START| \
                                LOCAL2_BT_NO_SIGNAL_ISR| \
                                0 \
                                )


static hci_forward_env_t *hci_forward_get_env(void)
{
    return &g_hci_forward_env;
}
//static int8_t g_rssi_test[1000];
void bt_nosignal_evt_send(void)
{
    hci_forward_env_t *env = hci_forward_get_env();
    rt_event_send(env->evt, LOCAL2_BT_NO_SIGNAL_ISR);
}

#if defined(SOC_SF32LB58X)
    #define BT_RSSI_ADDR   0x500840D8
    #define BT_PKT_ADDR    0x500904DC
    #define BLE_PKT_ADDR   0x500908D8
#elif defined(SOC_SF32LB56X)
    #define BT_RSSI_ADDR   0x500840C8
    #define BT_PKT_ADDR    0x500904DC
    #define BLE_PKT_ADDR   0x500908D8
#elif defined(SOC_SF32LB52X)
    #define BT_RSSI_ADDR   0x400840D4
    #define BT_PKT_ADDR    0x400904DC
    #define BLE_PKT_ADDR   0x400908D8
#else
    #define BT_RSSI_ADDR   0x400840D4
    #define BT_PKT_ADDR    0x400904DC
    #define BLE_PKT_ADDR   0x400908D8
#endif
void bt_test_rssi_entry(void *param)
{
    rt_uint32_t  evt;
    hci_forward_env_t *env = hci_forward_get_env();
    uint32_t rx_pkt_old, rx_pkt_new;
    volatile uint32_t *pkt_addr = NULL;
    volatile uint32_t *rssi_addr = (volatile uint32_t *)BT_RSSI_ADDR;
    uint32_t agc_stat;
    int8_t rssi;
    uint8_t wait_exit = 0;
    rt_int32_t delay_time;

    while (1)
    {
        rt_event_recv(env->evt, LOCAL2_BT_RSSI_ALL, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);

        if (evt & LOCAL2_BT_NO_SIGNAL_ISR)
        {
            int8_t rssi = -128;
#ifdef SOC_SF32LB52X
            extern int8_t bt_nosignal_irq(void);
            rssi = bt_nosignal_irq();
#endif
            if (rssi != -128)
            {
                env->rssi_acc += rssi;
                env->rssi_cnt++;
            }
        }

        if (evt & (LOCAL2_BT_RSSI_START | LOCAL2_BLE_RSSI_START))
        {
            //LOG_I("rssi entry evt:%d, state:%d\n", evt, env->no_signal_state);
            rt_thread_mdelay(3);
            rx_pkt_old = 0;
            rx_pkt_new = 0;
            if (evt & LOCAL2_BLE_RSSI_START)
            {
                pkt_addr = (volatile uint32_t *)BLE_PKT_ADDR;
                delay_time = 5;//ms
            }
            if (evt & LOCAL2_BT_RSSI_START)
            {
                pkt_addr = (volatile uint32_t *)BT_PKT_ADDR;
                delay_time = 3;//ms
            }
            env->rssi_acc = 0;
            env->rssi_cnt = 0;
            while (1)
            {
                if (env->no_signal_state == 0)
                {
                    break;
                }
                rx_pkt_new = *pkt_addr;
                //LOG_I("new:%d, old:%d\n", rx_pkt_new, rx_pkt_old);
                if (rx_pkt_new > rx_pkt_old)
                {
                    wait_exit = 0;
                    rx_pkt_old = rx_pkt_new;
                    agc_stat = *rssi_addr;
                    rssi = (int8_t)((agc_stat >> 16) & 0xFF);

                    env->rssi_cnt++;
                    env->rssi_acc += (int32_t)rssi;
                }
                else if (rx_pkt_new < rx_pkt_old)
                {
                    env->rssi_acc = 0;
                    env->rssi_cnt = 0;
                    rx_pkt_old = rx_pkt_new;
                }
                rt_thread_mdelay(delay_time);

            }
            env->rssi_ave = env->rssi_acc / env->rssi_cnt;
            //LOG_D("rssi acc:%d, cnt:%d, ave:%d\n", env->rssi_acc, env->rssi_cnt, env->rssi_ave);
            //LOG_D("rxend rssi acc:%d, cnt:%d, ave:%d\n", env->rssi_acc, env->rssi_cnt, (env->rssi_ave - 6));
        }
    }
}

static uint8_t crystal_cali_set(int32_t freq_off)
{
    int16_t code_table[12] = {-23, -48, -73, -101, -130, -161, -293, -228, -266, -305, -348, -394 };
    int32_t cal_value = ((hwp_pmuc->HXT_CR1 & PMUC_HXT_CR1_CBANK_SEL_Msk) >> PMUC_HXT_CR1_CBANK_SEL_Pos);
    uint8_t idx;
    int32_t cal_check = 0;
    int res;

    if (freq_off > 120000 || freq_off < -120000)  //+- 120KHz
    {
        return 1;
    }

    if (freq_off > 0)
    {
        code_table[0] = 22;
        code_table[1] = 43;
        code_table[2] = 63;
        code_table[3] = 82;
        code_table[4] = 100;
        code_table[5] = 117;
        code_table[6] = 134;
        code_table[7] = 149;
        code_table[8] = 165;
        code_table[9] = 179;
        code_table[10] = 193;
        code_table[11] = 206;
    }
    else
    {
        freq_off *= -1;
    }

    idx = freq_off / 10000;

    if (idx > 0 && idx < 11)
    {
        cal_value += code_table[idx] + ((code_table[idx + 1] - code_table[idx]) * (freq_off % 10000)) / 10000;
    }
    else if (idx == 0)
    {
        cal_value += (code_table[idx] * (freq_off % 10000)) / 10000;
    }
    else if (idx == 11)
    {
        cal_value += code_table[idx];
    }

    if (cal_value >= 0 && cal_value < 0x3FF)
    {
        res = rt_flash_config_write(FACTORY_CFG_ID_CRYSTAL, (uint8_t *)&cal_value, sizeof(cal_value));
        if (res > 0)
        {
            res = rt_flash_config_read(FACTORY_CFG_ID_CRYSTAL, (uint8_t *)&cal_check, sizeof(cal_check));
            if (res > 0 && cal_check == cal_value)
            {
                hwp_pmuc->HXT_CR1 &= ~PMUC_HXT_CR1_CBANK_SEL_Msk;
                hwp_pmuc->HXT_CR1 |= cal_value << PMUC_HXT_CR1_CBANK_SEL_Pos;
                return 0;
            }
        }
    }

    return 1;
}

static uint8_t crystal_cali_reset(void)
{
    int32_t cal_value = ((hwp_pmuc->HXT_CR1 & PMUC_HXT_CR1_CBANK_SEL_Msk) >> PMUC_HXT_CR1_CBANK_SEL_Pos);
    int32_t cal_check;
    int res;

    if (cal_value == 0x1CA)
    {
        res = rt_flash_config_read(FACTORY_CFG_ID_CRYSTAL, (uint8_t *)&cal_check, sizeof(cal_check));
        if (res > 0 && cal_check == cal_value)
        {
            return 0;
        }
    }

    cal_value = 0x1CA;
    hwp_pmuc->HXT_CR1 &= ~PMUC_HXT_CR1_CBANK_SEL_Msk;
    hwp_pmuc->HXT_CR1 |= cal_value << PMUC_HXT_CR1_CBANK_SEL_Pos;

    res = rt_flash_config_write(FACTORY_CFG_ID_CRYSTAL, (uint8_t *)&cal_value, sizeof(cal_value));
    if (res > 0)
    {
        res = rt_flash_config_read(FACTORY_CFG_ID_CRYSTAL, (uint8_t *)&cal_check, sizeof(cal_check));
        if (res > 0 && cal_check == cal_value)
        {
            return 0;
        }
    }

    return 1;
}

static uint8_t loc_cmd_hdl(uint8_t *cmd, uint16_t len)
{
    hci_forward_env_t *env = hci_forward_get_env();
    uint8_t is_handle = 0;
    uint8_t pattern[3] = {0x08, 0xFF, 0xEE};
    uint8_t ret_len = 7;
    if (memcmp(pattern, cmd, 3) == 0)
    {
        is_handle = 1;
        uint8_t res = 1; // Not support
        do
        {
            if (cmd[3] == 0x02)
            {
                int8_t tx_pwr = cmd[4];
                extern void blebredr_rf_power_set(uint8_t type, int8_t txpwr);
                blebredr_rf_power_set(0, tx_pwr);
                res = 0;
            }
            else if (cmd[3] == 0x82)
            {
                int8_t tx_pwr = cmd[4];
                extern void blebredr_rf_power_set(uint8_t type, int8_t txpwr);
                blebredr_rf_power_set(1, tx_pwr);
                res = 0;
            }
            else if (cmd[3] == 0x05)
            {
                ret_len = 8;
                res = 0;
            }
            else if (cmd[3] == 0x06)
            {
                int32_t freq_off = ((int32_t)cmd[4] | ((int32_t)cmd[5] << 8) \
                                    | ((int32_t)cmd[6] << 16) | ((int32_t)cmd[7] << 24));

                res = crystal_cali_set(freq_off);
            }
            else if (cmd[3] == 0x07)
            {
                res = crystal_cali_reset();
            }
        }
        while (0);


        {
            uint8_t *ptr = malloc(ret_len);
            ptr[0] = 0x09;
            ptr[1] = 0xFF;
            ptr[2] = 0xEE;
            ptr[3] = 0x0;
            ptr[4] = cmd[3];
            ptr[5] = ret_len - 6;
            ptr[6] = res;
            if ((cmd[3] == 0x05) && (ret_len == 8))
            {
                if (env->rssi_cnt == 0)
                {
                    ptr[7] = (uint8_t)(0x80);
                }
                else
                {
                    ptr[7] = (uint8_t)(env->rssi_ave - 6);
                }
            }
            {
                void hci_write_result(uint8_t *bufptr, uint32_t size);
                hci_write_result(ptr, (uint32_t)ret_len);
            }
            bt_mem_free(ptr);
        }
    }


    return is_handle;
}

static uint8_t loc_cmd2_hdl(uint8_t *cmd, uint16_t len)
{
    hci_forward_env_t *env = hci_forward_get_env();
    uint8_t is_handle = 0;
    uint8_t pattern[3] = {0x06, 0xEE, 0xFF};
    uint8_t ipc_rvt = 0;
    uint8_t ble_rx_pattern[3] = {0x01, 0x33, 0x20};
    uint8_t ble_rx_stop_pattern[3] = {0x01, 0x1F, 0x20};
    uint8_t bt_rx_pattern[3] = {0x01, 0x70, 0xfc};
    uint8_t bt_rx_stop_pattern[3] = {0x01, 0x72, 0xfc};
    uint8_t retlen = 5;

    if (memcmp(ble_rx_pattern, cmd, 3) == 0)
    {
        env->no_signal_state = 1;
        if (env->btrssi_th == NULL)
        {
            env->btrssi_th = rt_thread_create("btrssi", bt_test_rssi_entry, NULL, 2048, 10, 10);
            rt_thread_startup(env->btrssi_th);
        }
        rt_event_send(env->evt, LOCAL2_BLE_RSSI_START);
    }
    if (memcmp(bt_rx_pattern, cmd, 3) == 0)
    {
        env->no_signal_state = 2;
        if (env->btrssi_th == NULL)
        {
            env->btrssi_th = rt_thread_create("btrssi", bt_test_rssi_entry, NULL, 2048, 10, 10);
            rt_thread_startup(env->btrssi_th);
        }
        rt_event_send(env->evt, LOCAL2_BT_RSSI_START);
    }
    if ((memcmp(ble_rx_stop_pattern, cmd, 3) == 0) || (memcmp(bt_rx_stop_pattern, cmd, 3) == 0))
    {
        env->no_signal_state = 0;
    }

    if (memcmp(pattern, cmd, 3) == 0)
    {
        is_handle = 1;
        uint8_t res = 4; // Not support
        do
        {
            if (cmd[3] == 0x80 && len == 4)
            {
                res = 0;
                ipc_rvt = 1;
            }
            if (cmd[3] == 0x02 && cmd[4] == 0x05) //
            {
                uint8_t channel = cmd[5];
                uint16_t pkt_len = ((uint16_t)cmd[7] << 8) | cmd[6];
                uint8_t pkt_payload = cmd[8];
                uint8_t pkt_type = cmd[9];
                if (env->btrssi_th == NULL)
                {
                    env->btrssi_th = rt_thread_create("btrssi", bt_test_rssi_entry, NULL, 2048, 10, 10);
                    rt_thread_startup(env->btrssi_th);
                }
                env->rssi_acc = 0;
                env->rssi_cnt = 0;
                //rt_event_send(env->evt, LOCAL2_BT_NO_SIGNAL_START);
#ifdef SOC_SF32LB52X
                extern void bt_nosignal_start(uint8_t channel, uint8_t pkt_type, uint8_t pkt_payload, uint16_t pkt_len);
                bt_nosignal_start(channel, pkt_type, pkt_payload, pkt_len);
                res = 0;
#else
                res = 1;
#endif
            }
            if (cmd[3] == 0x03 && cmd[4] == 0x00)
            {
                res = 0;
                retlen = 23;
            }
        }
        while (0);

        {
            uint8_t *ptr = malloc(retlen);
            ptr[0] = 0x07;
            ptr[1] = 0xEE;
            ptr[2] = 0xFF;
            ptr[3] = cmd[3];
            ptr[4] = res;
            if (retlen == 23)
            {
                ptr[5] = 17;
#ifdef SOC_SF32LB52X
                extern void bt_nosignal_stop(uint8_t *data);
                bt_nosignal_stop(&ptr[6]);
#endif
                ptr[22] = (uint8_t)(env->rssi_acc / env->rssi_cnt - 6);
            }
            {
                void hci_write_result(uint8_t *bufptr, uint32_t size);
                hci_write_result(ptr, (uint32_t)retlen);
            }
            bt_mem_free(ptr);
        }
    }

    if (ipc_rvt)
    {
        rt_thread_mdelay(100);
        extern void uart_ipc_path_revert(void);
        uart_ipc_path_revert();
    }

    return is_handle;

}

static rt_err_t hci_trans_uart_rx_ind(rt_device_t dev, rt_size_t size)
{
    hci_forward_env_t *env = hci_forward_get_env();
    rt_mb_send(env->to_mb, size);

    return 0;
}

static int32_t hci_trans_mb_rx_ind(ipc_queue_handle_t dev, size_t size)
{
    hci_forward_env_t *env = hci_forward_get_env();
    rt_mb_send(env->to_uart, size);

    return 0;
}
#define  UART_EXTRA_DATA_LEN  20
uint16_t data_complete_check(uint8_t *cmd, uint16_t len)
{
    hci_forward_env_t *env = hci_forward_get_env();
    uint16_t size;
    HAL_Delay(20);
    size = rt_device_read(env->uart_port, 0, (cmd + len), UART_EXTRA_DATA_LEN);
    return (size + len);
}

void hci_forward_to_mb_entry(void *param)
{
    hci_forward_env_t *env = hci_forward_get_env();
    env->to_mb = rt_mb_create("fwd2mb", 16, RT_IPC_FLAG_FIFO);
    env->evt = rt_event_create("cmd_evt", RT_IPC_FLAG_FIFO);
    rt_uint32_t size;
    uint8_t *ptr;
    int written, offset = 0;
    rt_size_t read_len;
    while (1)
    {
        rt_mb_recv(env->to_mb, &size, RT_WAITING_FOREVER);
        LOG_D("(TB)read size %d, mb ptr %x\r\n", size, env->ipc_port);
        if (!size)
            continue;
        ptr = bt_mem_alloc(size + UART_EXTRA_DATA_LEN);
        RT_ASSERT(ptr);

        // Read from uart
        if (env->uart_port)
        {
            read_len = rt_device_read(env->uart_port, 0, ptr, size);
            if (read_len == 0)
            {
                bt_mem_free(ptr);
                continue;
            }
            if (read_len < size)
                size = read_len;
            read_len = data_complete_check(ptr, size);
            size = read_len;
        }
        else if (env->pbuf2mb)
        {
            memcpy(ptr, env->pbuf2mb, size);
        }
        else
        {
            RT_ASSERT(0);
        }
        //rt_hexdump("hci_tob", 32, ptr, size);
        // Write to mailbox
        HAL_DBG_print_data((char *)ptr, 0, size);

        if ((loc_cmd_hdl(ptr, size) == 1) || (loc_cmd2_hdl(ptr, size) == 1))
        {
            bt_mem_free(ptr);
            continue;
        }

        if (IPC_QUEUE_INVALID_HANDLE != env->ipc_port)
        {
            LOG_D("Write to MB %d\n", size);
            written = ipc_queue_write(env->ipc_port, ptr, size, 10);
            while (written < size)
            {
                size -= written;
                offset += written;
                written = ipc_queue_write(env->ipc_port, ptr + offset, size, 10);
            }
            LOG_D("Written to MB %d\n", written);
        }

        offset = 0;
        bt_mem_free(ptr);
    }

}
extern uint8_t lcpu_reset_evt_handle(uint8_t *evt, uint16_t len);

void hci_forward_to_uart_entry(void *param)
{
    hci_forward_env_t *env = hci_forward_get_env();
    env->to_uart = rt_mb_create("fwd2uart", 16, RT_IPC_FLAG_FIFO);
    rt_uint32_t size;
    uint8_t *ptr;
    int written;
    rt_size_t read_len;
    static uint8_t bcore_init = 0;
    while (1)
    {
        rt_mb_recv(env->to_uart, &size, RT_WAITING_FOREVER);
        LOG_D("(TU)read size %d, uart status %x\r\n", size, env->uart_port);
        if (!size)
            continue;
        ptr = bt_mem_alloc(size);
        RT_ASSERT(ptr);

        // Read from mailbox
        read_len = ipc_queue_read(env->ipc_port, ptr, size);
        if (read_len < size)
            size = read_len;
        HAL_DBG_print_data((char *)ptr, 0, size);
        //rt_hexdump("hci_tou", 32, ptr, size);

        // Write to mailbox
#ifdef SOC_SF32LB52X
        if (lcpu_reset_evt_handle(ptr, size) == 0)
#endif
        {
            void hci_write_result(uint8_t *bufptr, uint32_t size);
            hci_write_result(ptr, size);
        }

        bt_mem_free(ptr);
    }

}

__WEAK void bt_rf_bqb_config(void)
{

}

void uart_ipc_path_change(void)
{
    hci_forward_env_t *env = hci_forward_get_env();

#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
    rt_pm_hw_device_start();

    HAL_HPAON_WakeCore(CORE_ID_LCPU);
    extern rt_err_t pm_scenario_start(pm_scenario_name_t scenario);
    pm_scenario_start(PM_SCENARIO_AUDIO);
    rt_thread_delay(500);
#ifdef SOC_SF32LB52X
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_PERI,
                             RCC_CLK_TICK_HXT48);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_TICK, RCC_CLK_TICK_HXT48);
    CLEAR_BIT(hwp_pmuc->HRC_CR, PMUC_HRC_CR_EN);
#endif
#endif

    env->ipc_port = mbox_env.ipc_port;
    env->uart_port = rt_console_get_device();;
    env->trans_en = 1;
    env->saved_rx_indicate = env->uart_port->rx_indicate;
    env->uart_port->rx_indicate = hci_trans_uart_rx_ind;
    env->saved_open_flag = env->uart_port->open_flag;
    env->uart_port->open_flag &= ~RT_DEVICE_FLAG_STREAM;

    log_pause(true);

    //__asm__("B .");

    // Forward data to MB
    if (env->f2mb_th == NULL)
    {
        env->f2mb_th = rt_thread_create("fwd2mb", hci_forward_to_mb_entry, NULL, 2048, 10, 10);
        rt_thread_startup(env->f2mb_th);
    }
    // Forward data to uart
    if (env->f2uart_th == NULL)
    {
        env->f2uart_th = rt_thread_create("fwd2uart", hci_forward_to_uart_entry, NULL, 4096, 10, 10);
        rt_thread_startup(env->f2uart_th);
    }
    // Avoid LCPU enter sleep mode to access its UART
    HAL_HPAON_WakeCore(CORE_ID_LCPU);

    rt_thread_mdelay(100);

    bt_rf_bqb_config();

    {
        uint8_t reset[4] = {0x01, 0x03, 0x0c, 0x00};
        ipc_queue_write(env->ipc_port, reset, 4, 10);
    }

}
void uart_ipc_path_revert(void)
{
    hci_forward_env_t *env = hci_forward_get_env();

    if (env->uart_port)
    {
        env->uart_port->rx_indicate = env->saved_rx_indicate;
        env->uart_port->open_flag = env->saved_open_flag;
    }
    env->trans_en = 0;
    log_pause(false);
    HAL_HPAON_CANCEL_LP_ACTIVE_REQUEST();
#ifdef RT_USING_PM
    rt_pm_hw_device_stop();
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif
}

void hci_ipc_path_change(void)
{
    hci_forward_env_t *env = hci_forward_get_env();

#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
#endif

    env->ipc_port = mbox_env.ipc_port;
    env->uart_port = NULL;
    env->trans_en = 1;
    env->saved_rx_indicate = NULL;

    //__asm__("B .");

    // Forward data to MB
    if (env->f2mb_th == NULL)
    {
        env->f2mb_th = rt_thread_create("fwd2mb", hci_forward_to_mb_entry, NULL, 2048, 10, 10);
        rt_thread_startup(env->f2mb_th);
    }
    // Forward data to uart
    if (env->f2uart_th == NULL)
    {
        env->f2uart_th = rt_thread_create("fwd2uart", hci_forward_to_uart_entry, NULL, 4096, 10, 10);
        rt_thread_startup(env->f2uart_th);
    }

    if (env->pbuf2mb == NULL)
    {
        env->pbuf2mb = bt_mem_alloc(30);
        RT_ASSERT(env->pbuf2mb);
    }

    if (env->pbuf2api == NULL)
    {

        env->pbuf2api = bt_mem_alloc(30);
        RT_ASSERT(env->pbuf2api);
    }
    // Avoid LCPU enter sleep mode to access its UART
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
#if 0
    rt_thread_mdelay(100);
    {
        uint8_t reset[4] = {0x01, 0x03, 0x0c, 0x00};
        ipc_queue_write(env->ipc_port, reset, 4, 10);
    }
#endif
}
void hci_write_result(uint8_t *bufptr, uint32_t size)
{
    hci_forward_env_t *env = hci_forward_get_env();

    if (env->uart_port)
    {
        int written = rt_device_write(env->uart_port, 0, bufptr, size);
        RT_ASSERT(size == written);
    }
    else if (env->pbuf2api)
    {
        memcpy(env->pbuf2api, bufptr, size);
        rt_event_send(env->evt, LOCAL2_BT_NO_SIGNAL_RET);
    }
}
int8_t bt_ns_rx_stop(bt_ns_test_new_rx_rslt_t *rst)
{
    rt_uint32_t  evt;
    uint8_t pattern[5] = {0x06, 0xEE, 0xFF, 0x03, 0x00};
    hci_forward_env_t *env = hci_forward_get_env();
    uint8_t *data = (uint8_t *)rst;

    memcpy(env->pbuf2mb, pattern, 5);
    rt_mb_send(env->to_mb, 5);

    rt_event_recv(env->evt, LOCAL2_BT_NO_SIGNAL_RET, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);

    for (int i = 0; i < 16; i++)
    {
        *(data + i) = *(env->pbuf2api + 6 + i);
    }
    rst->rssi = *(env->pbuf2api + 22);

    return 0;
}

int8_t bt_ns_rx_start(bt_ns_test_new_rx_para_t *rxpara, bt_ns_test_new_rx_rslt_t *rst, uint32_t delay)
{
    rt_uint32_t  evt;
    uint8_t pattern[5] = {0x06, 0xEE, 0xFF, 0x02, 0x05};
    hci_forward_env_t *env = hci_forward_get_env();

    hci_ipc_path_change();

    memcpy(env->pbuf2mb, pattern, 5);
    env->pbuf2mb[5] = rxpara->channel;
    env->pbuf2mb[6] = (uint8_t)(rxpara->pkt_len & 0xFF);
    env->pbuf2mb[7] = (uint8_t)((rxpara->pkt_len >> 8) & 0xFF);
    env->pbuf2mb[8] = rxpara->pkt_payload;
    env->pbuf2mb[9] = rxpara->pkt_type;
    rt_mb_send(env->to_mb, 10);

    rt_event_recv(env->evt, LOCAL2_BT_NO_SIGNAL_RET, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);

    HAL_Delay(delay);

    bt_ns_rx_stop(rst);

    return 0;

}

int8_t ble_ns_rx_stop(ble_ns_test_rx_rslt_t *rst)
{
    rt_uint32_t  evt;
    uint8_t pattern[4] = {0x01, 0x1f, 0x20, 0x00};
    uint8_t pattern2[4] = {0x08, 0xff, 0xee, 0x05};
    hci_forward_env_t *env = hci_forward_get_env();

    memcpy(env->pbuf2mb, pattern, 4);
    rt_mb_send(env->to_mb, 4);

    rt_event_recv(env->evt, LOCAL2_BT_NO_SIGNAL_RET, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);

    rst->total_pkt_num = ((uint16_t)(*(env->pbuf2api + 8)) << 8) + *(env->pbuf2api + 7);

    memcpy(env->pbuf2mb, pattern2, 4);
    rt_mb_send(env->to_mb, 4);

    rt_event_recv(env->evt, LOCAL2_BT_NO_SIGNAL_RET, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);
    rst->rssi = *(env->pbuf2api + 7);

    return 0;
}

int8_t ble_ns_rx_start(ble_ns_test_rx_t *rxpara, ble_ns_test_rx_rslt_t *rst, uint32_t delay)
{
    rt_uint32_t  evt;
    uint8_t pattern[4] = {0x01, 0x33, 0x20, 0x03};
    hci_forward_env_t *env = hci_forward_get_env();

    hci_ipc_path_change();

    memcpy(env->pbuf2mb, pattern, 4);
    env->pbuf2mb[4] = rxpara->channel;
    env->pbuf2mb[5] = rxpara->phy;
    env->pbuf2mb[6] = rxpara->modulation_idx;
    rt_mb_send(env->to_mb, 7);

    rt_event_recv(env->evt, LOCAL2_BT_NO_SIGNAL_RET, RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, RT_WAITING_FOREVER, &evt);

    HAL_Delay(delay);

    ble_ns_rx_stop(rst);

    return 0;

}

#endif
#endif

void bluetooth_stack_ipc_register_tx_cb(tl_data_callback_t callback)
{
    tl_ipc_env.callback = callback;
}

int bluetooth_stack_ipc_write_data(uint8_t *bufptr, uint32_t size)
{
    int ret = 0;
    struct tl_ipc_env_tag *env = &tl_ipc_env;
    struct tl_ipc_slist_t *s_list = (struct tl_ipc_slist_t *)bt_mem_alloc(sizeof(struct tl_ipc_env_tag) + size);
    RT_ASSERT(s_list);
    s_list->buf_len = size;
    memcpy(s_list->buf, bufptr, size);
    GLOBAL_INT_DISABLE();
    rt_slist_append(&env->list, &s_list->list);
    if (env->rx.buf != NULL)
    {
        // cur_list should be NULL in this case.
        RT_ASSERT(env->cur_list == NULL);
        if (tl_ipc_read_data(env->rx.buf, env->rx.data_size) == env->rx.data_size)
        {
            env->rx.buf = NULL;
            if (env->rx.callback)
                env->rx.callback(env->rx.dummy, BT_EIF_STATUS_OK);
        }
        else
            ret = -1;

    }
    GLOBAL_INT_RESTORE();
    return ret;
}

static uint32_t tl_ipc_read_data(uint8_t *bufptr, uint32_t size)
{
    struct tl_ipc_env_tag *env = &tl_ipc_env;
    uint32_t rd_len = 0;
    // Sanity check
    RT_ASSERT(bufptr != NULL);
    RT_ASSERT(size != 0);

    if (!env->cur_list)
    {
        GLOBAL_INT_DISABLE();
        env->cur_list = (struct tl_ipc_slist_t *)rt_slist_first(&env->list);
        rt_slist_remove((rt_slist_t *)env->cur_list, &env->list);
        GLOBAL_INT_RESTORE();
        env->offset = 0;
    }

    if (env->cur_list)
    {
        RT_ASSERT(env->offset + size > env->cur_list->buf_len);
        memcpy(bufptr, env->cur_list->buf + env->offset, size);
        env->offset += size;
        rd_len = size;
        if (env->offset + size == env->cur_list->buf_len)
        {
            bt_mem_free(env->cur_list);
            env->cur_list = NULL;
            env->offset = 0;
        }
    }
    return rd_len;
}

static void tl_ipc_read(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy)
{
    bool succ = false;
    struct tl_ipc_env_tag *env = &tl_ipc_env;
    // Sanity check
    RT_ASSERT(bufptr != NULL);
    RT_ASSERT(size != 0);
    RT_ASSERT(callback != NULL);

    // IPC will make sure store completed buffer for one data packet.
    if (tl_ipc_read_data(bufptr, size) == size)
        succ = true;
    else
    {
        GLOBAL_INT_DISABLE();
        env->rx.callback = callback;
        env->rx.dummy    = dummy;
        env->rx.buf = bufptr;
        env->rx.data_size = size;
        env->rx.offset = 0;
        GLOBAL_INT_RESTORE();
    }

    if (succ && callback)
    {
        callback(dummy, BT_EIF_STATUS_OK);
    }

}

static void tl_ipc_write(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy)
{
    struct tl_ipc_env_tag *env = &tl_ipc_env;
    // Sanity check
    RT_ASSERT(bufptr != NULL);
    RT_ASSERT(size != 0);

    if (env->callback)
        env->callback(bufptr, size);

    if (NULL != callback)
    {
        callback(dummy, BT_EIF_STATUS_OK);
    }

}


static void tl_ipc_flow_on(void)
{
}


static bool tl_ipc_flow_off(void)
{
    return true;
}


#if defined(BF0_HCPU) && defined(HCI_ON_FLASH)
static void tl_flash_read(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy)
{
    // DO nothing
}

static void tl_flash_write(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy)
{
#ifdef RT_USING_DFS
    bt_hci_write(bufptr, size);
#endif
    if (NULL != callback)
    {
        callback(dummy, BT_EIF_STATUS_OK);
    }

}


static void tl_flash_flow_on(void)
{
}


static bool tl_flash_flow_off(void)
{
    return true;
}
#endif // defined(BF0_HCPU) && defined(HCI_ON_FLASH)

#ifdef HCI_ON_LOG
static void tl_log_read(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy)
{
    // DO nothing
}

static void tl_log_write(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy)
{
#ifdef LOG_ON_CONSOLE
    //LOG_I("len %d", size);
    LOG_BIN_MIX(bufptr, size);
#else
    LOG_BIN(bufptr, size);
#endif
    if (NULL != callback)
    {
        callback(dummy, BT_EIF_STATUS_OK);
    }

}


static void tl_log_flow_on(void)
{
}


static bool tl_log_flow_off(void)
{
    return true;
}
#endif //HCI_ON_LOG



static void tl_idle_read(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy)
{
    // DO nothing
}

static void tl_idle_write(uint8_t *bufptr, uint32_t size, bt_eif_callback callback, void *dummy)
{
    // Do nothing
}


static void tl_idle_flow_on(void)
{
}


static bool tl_idle_flow_off(void)
{
    return true;
}

#if 0

    // Pre-defined
    #if defined(BLE_ACTIVITY_6)
        #define ROM_ENV_BUF_SIZE 4448
        #define ROM_MSG_BUF_SIZE 11408
        #define HCI_MAX_NB_OF_COMPLETED 5
    #elif defined(BLE_ACTIVITY_2)
        #define ROM_ENV_BUF_SIZE 1784
        #define ROM_MSG_BUF_SIZE 6780
        #define HCI_MAX_NB_OF_COMPLETED 0
    #else
        #error "Not Supported"
    #endif


    #define ROM_NT_BUF_SIZE 668


    #define IO_MB_CH      (0)
    #define TX_BUF_SIZE   LCPU2HCPU_MB_CH1_BUF_SIZE
    #define TX_BUF_ADDR   LCPU2HCPU_MB_CH1_BUF_START_ADDR
    #define TX_BUF_ADDR_ALIAS LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH1_BUF_START_ADDR);
    #define RX_BUF_ADDR   HCPU_ADDR_2_LCPU_ADDR(HCPU2LCPU_MB_CH1_BUF_START_ADDR);



    static uint8_t *rom_env_buf;
    static uint8_t *rom_att_buf;
    static uint8_t *rom_msg_buf;
    static uint8_t *rom_nt_buf;
    static uint8_t *rom_log_buf;
#endif // ROM_COMFIG_ENABLE




__ROM_USED const struct bt_eif_api *port_config_uart0(void)
{
#ifdef UART_PORT0
    rt_uint16_t oflag = RT_DEVICE_OFLAG_RDWR;
    rt_err_t result;

    uart0_env.device = rt_device_find(UART_PORT0_PORT);
    if (NULL != uart0_env.device)
    {
        if (uart0_env.device->flag & RT_DEVICE_FLAG_DMA_RX)
        {
            oflag |= RT_DEVICE_FLAG_DMA_RX;
        }
        else
        {
            oflag |= RT_DEVICE_FLAG_INT_RX;
        }

        if (uart0_env.device->flag & RT_DEVICE_FLAG_DMA_TX)
        {
            oflag |= RT_DEVICE_FLAG_DMA_TX;
        }
        else
        {
            oflag |= RT_DEVICE_FLAG_INT_TX;
        }

        result = rt_device_open(uart0_env.device, oflag);
        RT_ASSERT(RT_EOK == result);

        rt_device_set_rx_indicate(uart0_env.device, uart0_rx_ind);
        rt_device_set_tx_complete(uart0_env.device, uart0_tx_complete);
    }

    return &uart0_api;
#else
    return NULL;
#endif
}


const struct bt_eif_api *port_config_flash(void)
{
#if defined(BF0_HCPU) && defined(HCI_ON_FLASH)
#ifdef RT_USING_DFS
    bt_hci_init(BT_HCI_DEFAULT_FLUSH_SIZE, 1);
#endif
    return &tl_flash_api;
#endif
    return NULL;
}


const struct bt_eif_api *port_config_log(void)
{
#ifdef HCI_ON_LOG
    return &tl_log_api;
#endif
    return NULL;
}

const struct bt_eif_api *port_config_idle(void)
{
    return &tl_idle_api;
}


__ROM_USED const struct bt_eif_api *port_config_uart1(void)
{
#ifdef UART_PORT1
    rt_uint16_t oflag = RT_DEVICE_OFLAG_RDWR;
    rt_err_t result;
    uart1_env.device = rt_device_find(UART_PORT1_PORT);

    if (NULL != uart1_env.device)
    {
        if (uart1_env.device->flag & RT_DEVICE_FLAG_DMA_RX)
        {
            oflag |= RT_DEVICE_FLAG_DMA_RX;
        }
        else
        {
            oflag |= RT_DEVICE_FLAG_INT_RX;
        }

        if (uart1_env.device->flag & RT_DEVICE_FLAG_DMA_TX)
        {
            oflag |= RT_DEVICE_FLAG_DMA_TX;
        }
        else
        {
            oflag |= RT_DEVICE_FLAG_INT_TX;
        }

        result = rt_device_open(uart1_env.device, oflag);
        RT_ASSERT(RT_EOK == result);

        //rt_device_set_rx_indicate(uart1_env.device, uart1_rx_ind);
        rt_device_set_tx_complete(uart1_env.device, uart1_tx_complete);
    }

    return &uart1_api;
#endif
    return NULL;
}


__ROM_USED const struct bt_eif_api *port_config_mailbox(void)
{
#ifdef USING_IPC_QUEUE

    ipc_queue_cfg_t q_cfg;

    q_cfg.qid = IO_MB_CH;
    q_cfg.tx_buf_size = TX_BUF_SIZE;
    q_cfg.tx_buf_addr = TX_BUF_ADDR;
    q_cfg.tx_buf_addr_alias = TX_BUF_ADDR_ALIAS;
#ifndef SF32LB52X
    /* Config IPC queue. */
    q_cfg.rx_buf_addr = RX_BUF_ADDR;

#else // SF32LB52X
    uint8_t rev_id = __HAL_SYSCFG_GET_REVID();
    if (rev_id < HAL_CHIP_REV_ID_A4)
    {
#if !defined(SF32LB52X_REV_B)
        q_cfg.rx_buf_addr = RX_BUF_ADDR;
#endif // !defined(SF32LB52X_REV_B)
    }
    else
    {
#if (defined(SF32LB52X_REV_B) || defined(SF32LB52X_REV_AUTO)) && defined(BF0_HCPU)
        q_cfg.rx_buf_addr = RX_BUF_REV_B_ADDR;
#endif // (defined(SF32LB52X_REV_B) || defined(SF32LB52X_REV_AUTO)) && defined(BF0_HCPU)
    }
#endif // !SF32LB52X

    q_cfg.rx_ind = NULL;
    q_cfg.user_data = 0;

    if (q_cfg.rx_ind == NULL)
        q_cfg.rx_ind = mbox_rx_ind;

    if (mbox_env.is_init == 0)
    {
        mbox_env.ipc_port = ipc_queue_init(&q_cfg);
        RT_ASSERT(IPC_QUEUE_INVALID_HANDLE != mbox_env.ipc_port);
        if (0 == ipc_queue_open(mbox_env.ipc_port))
            RT_ASSERT(1);
        mbox_env.is_init = 1;
    }
    return &mbox_api;
#else
    return NULL;
#endif
}


__ROM_USED const struct bt_eif_api *port_config_ipc(void)
{
#ifdef IPC_PORT
    return &tl_ipc_api;
#else
    return NULL;
#endif
}

__ROM_USED tl_port_t rom_port_get(uint8_t idx)
{
    tl_port_t type;
    switch (idx)
    {
    case 0:
    {
#ifdef UART_PORT0
        type = TL_PORT_UART0;
#elif defined(MB_PORT)
        type = TL_PORT_MB;
#elif defined(IPC_PORT)
        type = TL_PORT_IPC;
#else
        RT_ASSERT(0);
#endif
    }
    break;
    case 1:
    {
#ifdef UART_PORT1
        type = TL_PORT_UART1;
#elif defined(HCI_ON_FLASH)
        type = TL_PORT_FLASH;
#elif defined(HCI_ON_LOG)
        type = TL_PORT_LOG;
#else
        type = TL_PORT_IDLE;
#endif
    }
    break;
    default:
    {
        RT_ASSERT(0)
    }
    }
    return type;
}

// Pre-defined
#if defined(BT_DUAL_FULL_MEM)
    #define ROM_ENV_BUF_SIZE 7520
    #define ROM_MSG_BUF_SIZE 12176
#elif defined(BT_DUAL_CTRL_MEM)
    #ifndef SOC_SF32LB52X
        #define ROM_ENV_BUF_SIZE 5064
        #define ROM_MSG_BUF_SIZE 9104
    #else
        #define ROM_ENV_BUF_SIZE 4096
        #define ROM_MSG_BUF_SIZE 8192
    #endif
#elif defined(BT_DUAL_HOST_MEM)
    #define ROM_ENV_BUF_SIZE 2468
    #define ROM_MSG_BUF_SIZE 3084
#elif defined(BSP_USING_PC_SIMULATOR)
    #define ROM_ENV_BUF_SIZE 7520
    #define ROM_MSG_BUF_SIZE 12176
#else
    #error "Not Supported"
#endif

#define HCI_MAX_NB_OF_COMPLETED 6
#define ROM_NT_BUF_SIZE 668
#define BLE_RX_NB 6
#define BT_RX_NB 4

#if (defined(SOC_SF32LB56X) || defined(SOC_SF32LB52X)) && defined(BF0_LCPU)
#if defined(SOC_SF32LB56X)
    #define MAX_EM_BUFFER (28*1024)
#elif defined(SOC_SF32LB52X)
    #define MAX_EM_BUFFER (24*1024)
#endif
#ifndef MAX_BT_ACL
    #define MAX_BT_ACL 2
#endif
#ifndef MAX_BT_SCO
    #define MAX_BT_SCO 1
#endif
#ifndef MAX_BLE_ACT
    #define MAX_BLE_ACT 6
#endif
#ifndef MAX_BLE_RAL
    #define MAX_BLE_RAL 3
#endif

#ifndef SOC_SF32LB52X
    #ifndef ZBT
        #define MAX_BLE_ISO 0
    #else
        #define MAX_BLE_ISO 2
        #undef MAX_BT_ACL
        #define MAX_BT_ACL 1
        #undef MAX_BT_SCO
        #define MAX_BT_SCO 0
    #endif
#else
    #define MAX_BLE_ISO 2
#endif

#define MAX_EXTRA_TX_BUF_CNT 2
#if MAX_BT_ACL > 2
    #error "BT ACL link exceed maximum number !"
#endif
#if MAX_BT_SCO > 1
    #error "BT SCO link exceed maximum number !"
#endif
#if MAX_BLE_ACL > 10
    #error "BT ACL link exceed maximum number !"
#endif
#if MAX_BLE_RAL > 6
    #error "BT RAL exceed maximum number !"
#endif

typedef struct
{
    uint8_t bt_max_acl;
    uint8_t bt_max_sco;
    uint8_t ble_max_act;
    uint8_t ble_max_ral;
    uint8_t ble_max_iso;
    uint8_t is_bt_on;
    uint8_t is_ble_on;
    uint8_t ble_rx_desc;
    uint8_t bt_rx_desc;
} bluetooth_act_configt_t;

#ifndef SOC_SF32LB52X
static const bluetooth_act_configt_t g_bluetooth_act_config =
{
    .bt_max_acl = MAX_BT_ACL,
    .bt_max_sco = MAX_BT_SCO,
    .ble_max_act = MAX_BLE_ACT,
    .ble_max_ral = MAX_BLE_RAL,
    .ble_max_iso = MAX_BLE_ISO,
    .ble_rx_desc = BLE_RX_NB,
    .bt_rx_desc = BT_RX_NB,
#ifdef STACK_BT_ON
    .is_bt_on = 1,
#else
    .is_bt_on = 0,
#endif
#ifdef STACK_BLE_ON
    .is_ble_on = 1,
#else
    is_ble_on = 0,
#endif
};
#else
__ROM_USED bluetooth_act_configt_t g_bluetooth_act_config;
#endif

extern const rom_em_default_attr_t *rom_config_get_default_attribute_4_em(rom_em_offset_t em_type);
#define ALIGN4_SIZE(val) (((val)+3)&~3)



#define ROM_MEM_CALC_BLE_START_TOTAL_SIZE(size)              (size)
#define ROM_MEM_CALC_BLE_CS_TOTAL_SIZE(size)                 (((g_bluetooth_act_config.ble_max_act + 4) * size) * g_bluetooth_act_config.is_ble_on)
#define ROM_MEM_CALC_BLE_WPAL_TOTAL_SIZE(size)               (((g_bluetooth_act_config.ble_max_act + 2) * size) * g_bluetooth_act_config.is_ble_on)
#define ROM_MEM_CALC_BLE_RAL_TOTAL_SIZE(size)                (((g_bluetooth_act_config.ble_max_ral) * size) * g_bluetooth_act_config.is_ble_on)
#define ROM_MEM_CALC_BLE_RX_DESC_TOTAL_SIZE(size)            (((g_bluetooth_act_config.ble_rx_desc) * size) * g_bluetooth_act_config.is_ble_on)
#define ROM_MEM_CALC_BLE_TX_DESC_TOTAL_SIZE(size)            ((ROM_MEM_TYPE_DEFAULT_COUNT(BLE_TX_DESC) * size) * g_bluetooth_act_config.is_ble_on) // resver some bytes
#define ROM_MEM_CALC_BLE_LLCPTXBUF_TOTAL_SIZE(size)          ((2*g_bluetooth_act_config.ble_max_act*size) * g_bluetooth_act_config.is_ble_on)
#define ROM_MEM_CALC_BLE_ADVEXTHDRTXBUF_TOTAL_SIZE(size)     ((g_bluetooth_act_config.ble_max_act*size) * g_bluetooth_act_config.is_ble_on)
#define ROM_MEM_CALC_BLE_ADVDATATXBUF_TOTAL_SIZE(size)       (((g_bluetooth_act_config.ble_max_act+1)/2*size) * g_bluetooth_act_config.is_ble_on)
#define ROM_MEM_CALC_BLE_AUXCONNECTREQTXBUF_TOTAL_SIZE(size) (size * g_bluetooth_act_config.is_ble_on)
#define ROM_MEM_CALC_BLE_DATARXBUF_TOTAL_SIZE(size)          (((g_bluetooth_act_config.ble_rx_desc + 2) * size) * g_bluetooth_act_config.is_ble_on)
#define ROM_MEM_CALC_BLE_ACLTXBUF_TOTAL_SIZE(size)           (((g_bluetooth_act_config.ble_max_act+2)*size) * g_bluetooth_act_config.is_ble_on)
#define ROM_MEM_CALC_BLE_ISO_HOP_SEQ_TOTAL_SIZE(size)        (((g_bluetooth_act_config.ble_max_iso * 2) * size) * g_bluetooth_act_config.is_ble_on)
#define ROM_MEM_CALC_BLE_ISO_DESC_TOTAL_SIZE(size)           (((g_bluetooth_act_config.ble_max_iso * 4 + g_bluetooth_act_config.ble_max_iso) * size) * g_bluetooth_act_config.is_ble_on)
#ifndef SOC_SF32LB52X
    #define ROM_MEM_CALC_BLE_ISO_BUF_TOTAL_SIZE(size)            (((g_bluetooth_act_config.ble_max_iso * 10) * size) * g_bluetooth_act_config.is_ble_on)
#else
    #define ROM_MEM_CALC_BLE_ISO_BUF_TOTAL_SIZE(size)            (((g_bluetooth_act_config.ble_max_iso * 5) * size) * g_bluetooth_act_config.is_ble_on)
#endif
#define ROM_MEM_CALC_BLE_RX_CTE_DESC_TOTAL_SIZE(size)        (0)
#define ROM_MEM_CALC_BLE_TX_ANTENNA_TOTAL_SIZE(size)         (0)
#define ROM_MEM_CALC_BLE_RX_ANTENNA_TOTAL_SIZE(size)         (0)
#define ROM_MEM_CALC_BLE_END_TOTAL_SIZE(size)                (0)

#define ROM_MEM_CALC_BT_CS_TOTAL_SIZE(size)                  (((g_bluetooth_act_config.bt_max_acl + 5) * size)  * g_bluetooth_act_config.is_bt_on)
#define ROM_MEM_CALC_BT_RXDESC_TOTAL_SIZE(size)              (((g_bluetooth_act_config.bt_rx_desc) * size)  * g_bluetooth_act_config.is_bt_on)
#define ROM_MEM_CALC_BT_TXDESC_TOTAL_SIZE(size)              (((g_bluetooth_act_config.bt_max_acl * 2 + 4) * size) * g_bluetooth_act_config.is_bt_on)
#define ROM_MEM_CALC_BT_LMPRXBUF_TOTAL_SIZE(size)            ((ROM_MEM_TYPE_DEFAULT_COUNT(BT_LMPRXBUF) * size)  * g_bluetooth_act_config.is_bt_on)
#define ROM_MEM_CALC_BT_LMPTXBUF_TOTAL_SIZE(size)            ((((g_bluetooth_act_config.bt_max_acl > 3) * (2 * g_bluetooth_act_config.bt_max_acl) + \
                                                               (g_bluetooth_act_config.bt_max_acl <= 3) * 7) * size) * g_bluetooth_act_config.is_bt_on)
#define ROM_MEM_CALC_BT_ISCANFHSTXBUF_TOTAL_SIZE(size)       (size  * g_bluetooth_act_config.is_bt_on)
#define ROM_MEM_CALC_BT_PAGEFHSTXBUF_TOTAL_SIZE(size)        (size  * g_bluetooth_act_config.is_bt_on)
#define ROM_MEM_CALC_BT_EIRTXBUF_TOTAL_SIZE(size)            (size  * g_bluetooth_act_config.is_bt_on)
#define ROM_MEM_CALC_BT_LOCAL_SAM_SUBMAP_TOTAL_SIZE(size)    (size  * g_bluetooth_act_config.is_bt_on)
#define ROM_MEM_CALC_BT_PEER_SAM_MAP_TOTAL_SIZE(size)        ((g_bluetooth_act_config.bt_max_acl * size)  * g_bluetooth_act_config.is_bt_on)
#define ROM_MEM_CALC_BT_STPTXBUF_TOTAL_SIZE(size)            (size  * g_bluetooth_act_config.is_bt_on)
#define ROM_MEM_CALC_BT_ACLRXBUF_TOTAL_SIZE(size)            (((g_bluetooth_act_config.bt_rx_desc+2) * size) * g_bluetooth_act_config.is_bt_on)
#define ROM_MEM_CALC_BT_ACLTXBUF_TOTAL_SIZE(size)            (((g_bluetooth_act_config.bt_max_acl + MAX_EXTRA_TX_BUF_CNT) * size)  * g_bluetooth_act_config.is_bt_on)
#define ROM_MEM_CALC_BT_AUDIOBUF_TOTAL_SIZE(size)            ((g_bluetooth_act_config.bt_max_sco * size) * g_bluetooth_act_config.is_bt_on)
#define ROM_MEM_CALC_BT_END_TOTAL_SIZE(size)                 (0)


#define ROM_MEM_CALC_SIZE(type) ROM_MEM_CALC_##type##_TOTAL_SIZE(ROM_MEM_TYPE_SIZE(type))
#define ROM_MEM_TYPE_SIZE(type) (rom_config_get_default_attribute_4_em(ROM_EM_##type##_OFFSET)->size)
#define ROM_MEM_TYPE_DEFAULT_COUNT(type) (rom_config_get_default_attribute_4_em(ROM_EM_##type##_OFFSET)->cnt)

__ROM_USED void em_config_customized(void)
{
    uint16_t *em_buf = bt_mem_alloc(ROM_EM_END * 2);
    uint16_t offset;
    uint32_t i = 0;

    // Speical handle for start offset
    *(em_buf + i++) = offset = ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BLE_START));
    *(em_buf + i++) = offset = ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BLE_START));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BLE_CS));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BLE_WPAL));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BLE_RAL));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BLE_RX_DESC));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BLE_TX_DESC));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BLE_LLCPTXBUF));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BLE_ADVEXTHDRTXBUF));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BLE_ADVDATATXBUF));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BLE_AUXCONNECTREQTXBUF));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BLE_DATARXBUF));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BLE_ACLTXBUF));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BLE_ISO_HOP_SEQ));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BLE_ISO_DESC));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BLE_ISO_BUF));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BLE_RX_CTE_DESC));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BLE_TX_ANTENNA));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BLE_RX_ANTENNA));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BLE_END));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BT_CS));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BT_RXDESC));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BT_TXDESC));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BT_LMPRXBUF));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BT_LMPTXBUF));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BT_ISCANFHSTXBUF));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BT_PAGEFHSTXBUF));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BT_EIRTXBUF));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BT_LOCAL_SAM_SUBMAP));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BT_PEER_SAM_MAP));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BT_STPTXBUF));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BT_ACLRXBUF));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BT_ACLTXBUF));
    *(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BT_AUDIOBUF));
    //*(em_buf + i++) = offset += ALIGN4_SIZE(ROM_MEM_CALC_SIZE(BT_END));

    RT_ASSERT(i == ROM_EM_END);
    //LOG_I("EM offset end %x", *(em_buf + i - 1));
    if (*(em_buf + i - 1) > MAX_EM_BUFFER)
        RT_ASSERT(0 && "EM buffer exceed maximum length !!!");
    //LOG_HEX("em_offset", 16, em_buf, ROM_EM_END * 2);

    extern void rom_config_em_memory(uint8_t *buf, uint8_t num);
    rom_config_em_memory((uint8_t *)em_buf, ROM_EM_END);
    bt_mem_free(em_buf);

}


__ROM_USED void em_config(void)
{
    uint16_t *em_buf = bt_mem_alloc(ROM_EM_END * 2);
    const rom_em_default_attr_t *attr;
    uint16_t offset;
    uint32_t i = 0;
    extern const rom_em_default_attr_t *rom_config_get_default_attribute_4_em(rom_em_offset_t em_type);
    // Speical handle for start offset
    attr = rom_config_get_default_attribute_4_em(0);
    *em_buf = offset = ALIGN4_SIZE(attr->size * attr->cnt);

    for (i = 1 ; i < ROM_EM_END; i++)
    {
        *(em_buf + i) = offset;
        attr = rom_config_get_default_attribute_4_em(i);
        offset += ALIGN4_SIZE(attr->size * attr->cnt);
    }
    //LOG_HEX("em_offset", 16, em_buf, ROM_EM_END * 2);

    extern void rom_config_em_memory(uint8_t *buf, uint8_t num);
    rom_config_em_memory((uint8_t *)em_buf, ROM_EM_END);
    bt_mem_free(em_buf);
}
#define BLE_CONTROLLER_ENABLE_MASK ((g_bluetooth_act_config.is_ble_on << 0) & 0x01)
#define BT_CONTROLLER_ENABLE_MASK ((g_bluetooth_act_config.is_bt_on << 1) & 0x02)
#endif //SOC_SF32LB56X && BF0_LCPU

#if defined(SOC_SF32LB52X) && defined(BF0_LCPU)
extern void rom_config_set_default_link_config(bluetooth_act_configt_t *cfg);
static void act_config(void)
{
    g_bluetooth_act_config.bt_max_acl = MAX_BT_ACL;
    g_bluetooth_act_config.bt_max_sco = MAX_BT_SCO;
    g_bluetooth_act_config.ble_max_act = MAX_BLE_ACT;
    g_bluetooth_act_config.ble_max_ral = MAX_BLE_RAL;
    g_bluetooth_act_config.ble_max_iso = MAX_BLE_ISO;
    g_bluetooth_act_config.ble_rx_desc = ROM_MEM_TYPE_DEFAULT_COUNT(BLE_RX_DESC) - 2;
    g_bluetooth_act_config.bt_rx_desc = ROM_MEM_TYPE_DEFAULT_COUNT(BT_RXDESC) - 2;
#ifdef STACK_BT_ON
    g_bluetooth_act_config.is_bt_on = 1;
#else
    g_bluetooth_act_config.is_bt_on = 0;
#endif
#ifdef STACK_BLE_ON
    g_bluetooth_act_config.is_ble_on = 1;
#else
    g_bluetooth_act_config.is_ble_on = 0;
#endif
    rom_config_set_default_link_config(&g_bluetooth_act_config);
}
#endif // SOC_SF32LB52X


__ROM_USED void mem_env_config(ble_mem_config_t *config, uint16_t env_buf_size, uint16_t att_buf_size,
                               uint16_t msg_buf_size, uint16_t nt_buf_size, uint16_t log_buf_size,
                               int8_t nb_of_comp, uint8_t *nvds_buf, uint16_t nvds_buf_size)
{
    if (env_buf_size)
    {
        config->env_buf_size = env_buf_size;
        config->env_buf = bt_mem_alloc(env_buf_size);
        RT_ASSERT(config->env_buf);
    }

    if (att_buf_size)
    {
        config->att_buf_size = att_buf_size;
        config->att_buf = bt_mem_alloc(att_buf_size);
        RT_ASSERT(config->att_buf);
    }

    if (msg_buf_size)
    {
        config->msg_buf_size = msg_buf_size;
        config->msg_buf = bt_mem_alloc(msg_buf_size);
        RT_ASSERT(config->msg_buf);
    }

    if (nt_buf_size)
    {
        config->nt_buf_size = nt_buf_size;
        config->nt_buf = bt_mem_alloc(nt_buf_size);
        RT_ASSERT(config->nt_buf);
    }

#if (defined(BF0_LCPU) || ((defined(SOC_SF32LB52X) || defined(SOC_SF32LB56X) || defined(SOC_SF32LB58X)) && defined(BF0_HCPU)))
    if (log_buf_size)
    {
        config->log_buf_size = log_buf_size;
        config->log_buf = bt_mem_alloc(log_buf_size);
        RT_ASSERT(config->log_buf);
    }
#endif

    config->max_nb_of_hci_completed = nb_of_comp;

#ifndef SOC_SF32LB56X
    config->nvds_buf = (uint8_t *)nvds_buf;
    config->nvds_buf_size = nvds_buf_size;
#else
//    config.nvds_buf = (uint8_t *)bt_mem_alloc(NVDS_BUFF_SIZE);
    config->nvds_buf = (uint8_t *)nvds_buf;
    config->nvds_buf_size = nvds_buf_size;
#endif
}

__ROM_USED void mem_config(void)
{

#if (defined(SOC_SF32LB56X) || defined(SOC_SF32LB52X)) && defined(BF0_LCPU)
    extern void rom_config_set_controller_enabled(uint8_t enabled_module);
    rom_config_set_controller_enabled(BT_CONTROLLER_ENABLE_MASK | BLE_CONTROLLER_ENABLE_MASK);
#endif

    ble_mem_config_t config = {0};
    mem_env_config(&config, ROM_ENV_BUF_SIZE, ROM_ATT_BUF_SIZE, ROM_MSG_BUF_SIZE, ROM_NT_BUF_SIZE,
#if ((defined(BF0_LCPU) && !defined(SOC_SF32LB52X)) || ((defined(SOC_SF32LB52X) || defined(SOC_SF32LB56X) || defined(SOC_SF32LB58X)) && defined(BF0_HCPU)))
                   ROM_LOG_SIZE,
#else
                   0,
#endif
                   HCI_MAX_NB_OF_COMPLETED, (uint8_t *)NVDS_BUFF_START, NVDS_BUFF_SIZE);

    ble_memory_config(&config);
#if (defined(SOC_SF32LB56X) || defined(SOC_SF32LB52X)) && defined(BF0_LCPU)
    em_config_customized();
//em_config();

#endif // SOC_SF32LB56X && BF0_LCPU

}


__ROM_USED void port_config(void)
{
#if defined(FPGA)||defined(BSP_USING_PC_SIMULATOR) || defined(SOC_SF32LB58X) || defined(SOC_SF32LB56X) || defined(SOC_SF32LB52X)

    const struct bt_eif_api *tl;
    extern  void rom_config_set_tl(const struct bt_eif_api * tl, uint8_t idx);
    // config idx 0 as data transfer function
    tl = tl_port_api[rom_port_get(0)].callback();
    RT_ASSERT(tl);
    rom_config_set_tl(tl, 0);

#if !defined(BSP_USING_PC_SIMULATOR) && ((!defined(BF0_HCPU) && !defined(SOC_SF32LB52X)) || \
    (defined(BF0_HCPU) && (defined(SOC_SF32LB52X) || defined(SOC_SF32LB56X) || defined(SOC_SF32LB58X))))
    // config idx 1 as trace function
    tl = tl_port_api[rom_port_get(1)].callback();
    RT_ASSERT(tl);
    rom_config_set_tl(tl, 1);
#endif
#endif
}

// SOC_SF32LB58X || BSP_USING_PC_SIMULATOR
#elif defined(SOC_SF32LB55X)

// Pre-defined
#if defined(BLE_ACTIVITY_6)
    #define ROM_ENV_BUF_SIZE 4448
    #define ROM_MSG_BUF_SIZE 11408
    #define HCI_MAX_NB_OF_COMPLETED 5
#elif defined(BLE_ACTIVITY_2)
    #define ROM_ENV_BUF_SIZE 1784
    #define ROM_MSG_BUF_SIZE 6780
    #define HCI_MAX_NB_OF_COMPLETED 0
#else
    #error "Not Supported"
#endif


#define ROM_NT_BUF_SIZE 668


#define IO_MB_CH      (0)
#define TX_BUF_SIZE   LCPU2HCPU_MB_CH1_BUF_SIZE
#define TX_BUF_ADDR   LCPU2HCPU_MB_CH1_BUF_START_ADDR
#define TX_BUF_ADDR_ALIAS LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH1_BUF_START_ADDR);
#define RX_BUF_ADDR   HCPU_ADDR_2_LCPU_ADDR(HCPU2LCPU_MB_CH1_BUF_START_ADDR);

#ifdef RT_USING_CONSOLE
    #define LOG_PORT_NAME RT_CONSOLE_DEVICE_NAME
#else
    #define LOG_PORT_NAME ROM_LOG_PORT
#endif


__WEAK void ble_port_config(ipc_queue_cfg_t *cfg, rt_device_t ble_uart0, rt_device_t ble_uart1)
{
    // Do nothing
}

__WEAK void ble_memory_config(ble_mem_config_t *config)
{
    // Do nothing
}


static void mem_config(void)
{
    ble_mem_config_t config;
    config.env_buf_size = ROM_ENV_BUF_SIZE;
    config.env_buf = bt_mem_alloc(ROM_ENV_BUF_SIZE);
    RT_ASSERT(config.env_buf);

    config.att_buf_size = ROM_ATT_BUF_SIZE;
    config.att_buf = bt_mem_alloc(ROM_ATT_BUF_SIZE);
    RT_ASSERT(config.att_buf);

    config.msg_buf_size = ROM_MSG_BUF_SIZE;
    config.msg_buf = bt_mem_alloc(ROM_MSG_BUF_SIZE);
    RT_ASSERT(config.msg_buf);

    config.nt_buf_size = ROM_NT_BUF_SIZE;
    config.nt_buf = bt_mem_alloc(ROM_NT_BUF_SIZE);
    RT_ASSERT(config.nt_buf);

    config.log_buf_size = ROM_LOG_SIZE;
    config.log_buf = bt_mem_alloc(ROM_LOG_SIZE);
    RT_ASSERT(config.log_buf || (config.log_buf_size == 0));

    config.max_nb_of_hci_completed = HCI_MAX_NB_OF_COMPLETED;

    ble_memory_config(&config);

}


static void port_config(void)
{
    ipc_queue_cfg_t q_cfg;
    rt_device_t uart0_handle;

    /* Config IPC queue. */
    q_cfg.qid = IO_MB_CH;
    q_cfg.tx_buf_size = TX_BUF_SIZE;
    q_cfg.tx_buf_addr = TX_BUF_ADDR;
    q_cfg.tx_buf_addr_alias = TX_BUF_ADDR_ALIAS;
    q_cfg.rx_buf_addr = RX_BUF_ADDR;
    q_cfg.rx_ind = NULL;
    q_cfg.user_data = 0;

    /* Config BLE uart0. */
    uart0_handle = rt_device_find(LOG_PORT_NAME);
    RT_ASSERT(uart0_handle != NULL);

    ble_port_config(&q_cfg, uart0_handle, NULL);
}

// Only for chip ID small than 3
static void port_config_ex(void)
{

    uint32_t i = 0;
    if (rt_strcmp("uart3", LOG_PORT_NAME) == 0)
        i = 3;
    else if (rt_strcmp("uart4", LOG_PORT_NAME) == 0)
        i = 4;
    else if (rt_strcmp("uart5", LOG_PORT_NAME) == 0)
        i = 5;
    // reuse for port config
    void rom_config_set_ble_service_working_core(uint8_t working_core);
    rom_config_set_ble_service_working_core(i);
}


#endif // SOC_SF32LB58X || BSP_USING_PC_SIMULATOR





static uint8_t rom_config_valid_check(void)
{
#if (defined(FPGA)||defined(BSP_USING_PC_SIMULATOR) || defined(SOC_SF32LB58X) || defined(SOC_SF32LB56X) || defined(SOC_SF32LB52X))
    return 1;
#elif defined(SOC_SF32LB55X)
    uint8_t rev;
    uint16_t len = sizeof(rev);
    HAL_StatusTypeDef ret = HAL_LCPU_CONFIG_get(HAL_LCPU_CONFIG_CHIP_REV, &rev, &len);
    if (ret == HAL_OK && rev == HAL_CHIP_REV_ID_A3)
    {
        return 1;
    }
#endif
    return 0;
}

#if ((defined(SOC_SF32LB52X) || defined(SOC_SF32LB56X) || defined(SOC_SF32LB58X)) && defined(BF0_HCPU))||defined(BSP_USING_PC_SIMULATOR)

typedef bool (*hostlib_get_trace_en_callback)(void);

hostlib_get_trace_en_callback g_hostlib_trace_en_callback = NULL;

void register_hostlib_trace_en_callback(hostlib_get_trace_en_callback fun)
{
    g_hostlib_trace_en_callback = fun;
}
extern void hostlib_config_set_trc_en(void);
#endif

void bluetooth_config(void)
{

#ifdef SOC_SF32LB55X
    port_config_ex();
#endif
    if (rom_config_valid_check() == 0)
        return;

#if defined(SOC_SF32LB52X) && defined(BF0_LCPU)
    act_config();
#endif
#if ((defined(SOC_SF32LB52X) || defined(SOC_SF32LB56X) || defined(SOC_SF32LB58X)) && defined(BF0_HCPU)||defined(BSP_USING_PC_SIMULATOR))
    if (g_hostlib_trace_en_callback && g_hostlib_trace_en_callback())
    {
        hostlib_config_set_trc_en();
    }
#endif

    mem_config();
    port_config();
}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

