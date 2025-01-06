/**
  ******************************************************************************
  * @file   bf0_sibles_util.c
  * @author Sifli software development team
  * @brief SIFLI BLE utility source.
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

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include <stdlib.h>
#include "os_adaptor.h"

#include "bf0_sibles_util.h"
#include "ble_stack.h"
#include "bf0_ble_hci.h"
#include "att.h"

#define LOG_TAG "sibles_utils"
#include "log.h"
#include "ipc_queue.h"
#include "mem_section.h"

#include "bf0_sibles_nvds.h"



#ifdef SOC_BF_Z0
    #ifdef SOC_BF0_HCPU
        #define IO_MB_CH      (8)
        #define TX_BUF_SIZE   HCPU2BCPU_MB_CH1_BUF_SIZE
        #define TX_BUF_ADDR   HCPU2BCPU_MB_CH1_BUF_START_ADDR
        #define TX_BUF_ADDR_ALIAS HCPU_ADDR_2_BCPU_ADDR(HCPU2BCPU_MB_CH1_BUF_START_ADDR);
        #define RX_BUF_ADDR   BCPU_ADDR_2_HCPU_ADDR(BCPU2HCPU_MB_CH1_BUF_START_ADDR);


    #elif defined(SOC_BF0_LCPU)
        #define IO_MB_CH      (8)
        #define TX_BUF_SIZE   LCPU2BCPU_MB_CH1_BUF_SIZE
        #define TX_BUF_ADDR   LCPU2BCPU_MB_CH1_BUF_START_ADDR
        #define TX_BUF_ADDR_ALIAS HCPU_ADDR_2_BCPU_ADDR(LCPU2BCPU_MB_CH1_BUF_START_ADDR);
        #define RX_BUF_ADDR   BCPU_ADDR_2_HCPU_ADDR(BCPU2HCPU_MB_CH1_BUF_START_ADDR);
    #else
        #error "wrong config"
    #endif
#else
    #define IO_MB_CH      (0)
    #define TX_BUF_SIZE   HCPU2LCPU_MB_CH1_BUF_SIZE
    #define TX_BUF_ADDR   HCPU2LCPU_MB_CH1_BUF_START_ADDR
    #define TX_BUF_ADDR_ALIAS HCPU_ADDR_2_LCPU_ADDR(HCPU2LCPU_MB_CH1_BUF_START_ADDR)
    #define RX_BUF_ADDR   LCPU_ADDR_2_HCPU_ADDR(LCPU2HCPU_MB_CH1_BUF_START_ADDR)
#endif

#define SIBLES_PROCESS_MAX_TICK 20

static OS_THREAD_DECLAR(g_sifli_tid);
static OS_SEM_DECLAR(g_sifli_sem);

#ifdef SIBLES_TRANSPARENT_ENABLE
    static rt_device_t output_dev_handle;
    static uint8_t is_trans_enable;
#endif //SIBLES_TRANSPARENT_ENABLE

#define SIFLI_MBOX_THREAD_STACK_SIZE  (4096)

#ifdef SOC_BF0_HCPU
    static ipc_queue_handle_t ipc_queue_handle = IPC_QUEUE_INVALID_HANDLE;
    static OS_MAILBOX_DECLAR(g_bf0_sible_mb);
    static OS_EVENT_DECLAR(evt_mailbox);
    static OS_SEM_DECLAR(g_sifli_sem2);
    L1_NON_RET_BSS_SECT_BEGIN(sifli_mbox_thread_stack)
    ALIGN(RT_ALIGN_SIZE)
    static uint8_t sifli_mbox_thread_stack[SIFLI_MBOX_THREAD_STACK_SIZE];
    L1_NON_RET_BSS_SECT_END
#endif /* SOC_BF0_HCPU */

#if defined(SOC_SF32LB56X)
    #define DATA_SVC_WAIT_RF_FUL_DELAY   3000
#else
    #define DATA_SVC_WAIT_RF_FUL_DELAY   2000
#endif

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if defined(SOC_SF32LB55X) && defined(SOC_BF0_HCPU)

static uint8_t sifli_msg_process(void);
static void sifli_msg_free(void *param_ptr);
static uint32_t sifli_tx_msg_process(uint8_t *data);


ipc_queue_handle_t sifli_get_mbox_read_dev(void)
{
    return ipc_queue_handle;
}

ipc_queue_handle_t sifli_get_mbox_write_dev(void)
{
    return ipc_queue_handle;
}

os_status_t sifli_mbox_send(uint8_t *data)
{
    return os_mailbox_put(g_bf0_sible_mb, (uint32_t)data);
}


#ifdef SIBLES_TRANSPARENT_ENABLE
rt_device_t sifli_transparent_get_output_dev(void)
{
    return output_dev_handle;
}
#endif //SIBLES_TRANSPARENT_ENABLE




void *sifli_msg_alloc(sifli_msg_id_t const id, sifli_task_id_t const dest_id,
                      sifli_task_id_t const src_id, uint16_t const param_len)
{
    sibles_msg_para_t *msg = (sibles_msg_para_t *) bt_mem_alloc(sizeof(sibles_msg_para_t) + param_len);
    void *param_ptr = NULL;

    BT_OOM_ASSERT(msg != NULL);
    msg->id        = id;
    msg->dest_id   = dest_id;
    msg->src_id    = src_id;
    msg->param_len = param_len;

    param_ptr = sifli_msg2param(msg);

    memset(param_ptr, 0, param_len);

    return param_ptr;
}

void sifli_msg_free(void *param_ptr)
{
    OS_ASSERT(param_ptr);
    sibles_msg_para_t *msg = sifli_param2msg(param_ptr);
    bt_mem_free(msg);
}

rt_err_t silfi_mbox_notify(uint32_t evt)
{
    return os_event_flags_set(evt_mailbox, evt);
}


void sifli_msg_send(void const *param_ptr)
{
    os_status_t ret = sifli_mbox_send((uint8_t *)param_ptr);
    RT_ASSERT(ret == RT_EOK);
    silfi_mbox_notify(SIFLI_TASK_TRAN_EVT);
}

static size_t sifli_msg_send_to_ipc_queue(void *buffer, size_t buffer_len)
{
    ipc_queue_handle_t dev = sifli_get_mbox_write_dev();
    size_t len = buffer_len;
    int written, offset = 0;
    uint32_t count = 0xFFFFF;
    while (len)
    {
        written = ipc_queue_write(dev, (uint8_t *)(buffer + offset), len, 10);
        len -= written;
        offset += written;
        count--;
        if (count == 0)
        {
            LOG_E("sifli send failed\r\n");
            break;
        }
    }
    return len;
}

static void sifli_msg_send_internal(void const *param_ptr)
{
    uint8_t type = AHI_TYPE;
    OS_ASSERT(param_ptr);
    sibles_msg_para_t *msg = sifli_param2msg(param_ptr);
    uint16_t len = sizeof(sibles_msg_para_t) + msg->param_len;
    size_t remain_len;

    LOG_I("tx:%x, %d\n", msg->id, msg->param_len);
//#ifdef SIBLE_DEBUG
    //rt_print_data((char *)msg->param, 0, msg->param_len);
    LOG_HEX("dump_tx", 16, (uint8_t *)msg, len);
//#endif
    // Write header
    remain_len = sifli_msg_send_to_ipc_queue(&type, sizeof(type));
    if (remain_len != 0)
    {
        LOG_E("Header write failed!");
        return;
    }
    // Write body
    sifli_msg_send_to_ipc_queue(msg, (size_t)len);
}

uint32_t sifli_tx_msg_process(uint8_t *data)
{
    sifli_msg_send_internal((void const *)data);
    sifli_msg_free(data);
    // return for handle send failed case.
    // Notify task
    return 1;
}

// Sifli RX handle

static int32_t sifli_mbox_hcpuind(ipc_queue_handle_t dev, size_t size)
{
    uint8_t *data;
    uint32_t i;

    if (size > MAX_MAIL_SIZE)
        LOG_E("too much data\n");
    LOG_I("Got Mailbox interupt\n");
    if (evt_mailbox)
        os_event_flags_set(evt_mailbox, SIFLI_TASK_RECV_EVT);
    return 0;
}

void sifli_AHI_process(void)
{
    size_t rd_buf_len, offset;
    ipc_queue_handle_t dev = sifli_get_mbox_read_dev();
    uint8_t *buf = sifli_get_mbox_buffer();
    rd_buf_len = ipc_queue_read(dev, buf, sizeof(sibles_msg_para_t));
    while (rd_buf_len < sizeof(sibles_msg_para_t))
    {
        offset = rd_buf_len;
        rd_buf_len = ipc_queue_read(dev, buf + offset, sizeof(sibles_msg_para_t) - offset);
        rd_buf_len += offset;
    }

    sibles_msg_para_t *header = (sibles_msg_para_t *)buf;
    // Not handle buffer large than MAX_MAIL_SIZE.
    RT_ASSERT(header->param_len <= MAX_MAIL_SIZE && "Buffer more than mailbox buffer");
    uint8_t *buf_ptr = (uint8_t *)(&header->param);
    rd_buf_len = ipc_queue_read(dev, buf_ptr, header->param_len);
    while (rd_buf_len < header->param_len)
    {
        offset = rd_buf_len;
        rd_buf_len = ipc_queue_read(dev, buf_ptr + offset, header->param_len - offset);
        rd_buf_len += offset;
    }

    LOG_D("got event(%x): %s, len=%d\n", header->id, msg_str(header->id), header->param_len);
    sifli_mbox_process(header, buf_ptr, header->param_len);
}



#ifdef SIBLES_TRANSPARENT_ENABLE


uint8_t sifli_msg_transparent_process(void)
{

    ipc_queue_handle_t ipc_dev = sifli_get_mbox_read_dev();
    rt_device_t uart_handle =  sifli_transparent_get_output_dev();

    rt_uint32_t size;
    uint8_t *ptr;
    int written;
    rt_size_t read_len;
    static uint8_t bcore_init = 0;
    // Just malloc max buffer
    size = LCPU2HCPU_MB_CH1_BUF_SIZE;
    ptr = bt_mem_alloc(size);
    BT_OOM_ASSERT(ptr);
    do
    {

        // Read from mailbox
        read_len = ipc_queue_read(ipc_dev, ptr, size);
        if (read_len < size)
            size = read_len;
        //HAL_DBG_print_data((char *)ptr, 0, size);
        LOG_HEX("read data", 16, ptr, size);

        // NVDS init handle
        if (bcore_init == 0)
        {
            uint8_t init_array_stack[] = {0x05, 0x01, 0x0F, 0x10, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x01};
            uint8_t init_rsp[] = {0x05, 0x02, 0x0F, 0x0F, 0x00, 0x10, 0x00, 0x08, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
#ifdef BSP_BLE_NVDS_SYNC
            uint8_t init_array_app[] = {0x05, 0x01, 0x0F, 0x10, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x02};
            uint8_t init_rsp_2[] = {0x05, 0x02, 0x0F, 0x0F, 0x00, 0x10, 0x00, 0x08, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00};
#endif
            uint8_t power_on_req[] = {0x05, 0xFF, 0x00, 0x10, 0x00, 0x47, 0x00}; // No need to check parameter
            uint8_t requester_set[] = {0x05, 0xB3, 0x0D, 0x0D, 0x00, 0x10, 0x00, 0x00, 0x00};
            if ((size == sizeof(init_array_stack)) && (memcmp(ptr, init_array_stack, size) == 0))
            {
                // Response bcore
#ifdef BSP_BLE_NVDS_SYNC
                uint16_t len;
                uint8_t ret = sifli_nvds_read(SIFLI_NVDS_TYPE_STACK, &len, ptr);
                if (ret == NVDS_OK && len)
                {
                    uint8_t ahi_header[] = {0x05, 0x02, 0x0F, 0x0F, 0x00, 0x10, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00};
                    uint8_t *ptr1 = bt_mem_alloc(len + sizeof(ahi_header) + 2);
                    RT_ASSERT(ptr1);
                    rt_memcpy(ptr1, ahi_header, sizeof(ahi_header));
                    *(ptr1 + sizeof(ahi_header)) = (uint8_t)(len & 0xFF);
                    *(ptr1 + sizeof(ahi_header) + 1) = (uint8_t)((len >> 8) & 0xFF);
                    *(ptr1 + 7) = (uint8_t)((len + 8) & 0xFF);
                    *(ptr1 + 8) = (uint8_t)(((len + 8) >> 8) & 0xFF);
                    rt_memcpy(ptr1 + sizeof(ahi_header) + 2, ptr, len);
                    LOG_HEX("nvds data init", 16, ptr1, len + sizeof(ahi_header) + 2);
                    ipc_queue_write(ipc_dev, ptr1, len + sizeof(ahi_header) + 2, 10);
                    bt_mem_free(ptr1);
                }
                else
#endif
                    ipc_queue_write(ipc_dev, init_rsp, sizeof(init_rsp), 10);
                break;
                // No need forward
            }
#ifdef BSP_BLE_NVDS_SYNC
            else if ((size == sizeof(init_array_app)) && (memcmp(ptr, init_array_app, size) == 0))
            {
                ipc_queue_write(ipc_dev, init_rsp_2, sizeof(init_rsp_2), 10);
                break;
            }
#endif
            if (memcmp(ptr, power_on_req, sizeof(power_on_req)) == 0)
            {
                // Response bcore
                ipc_queue_write(ipc_dev, requester_set, sizeof(requester_set), 10);
                // No need forward
                break;
            }
            //bcore_init = 1;
        }

        // Write to mailbox
        if (uart_handle)
        {
            LOG_HEX("write uart data", 16, ptr, size);
            written = rt_device_write(uart_handle, 0, ptr, size);
            RT_ASSERT(size == written);
        }
    }
    while (0);

    bt_mem_free(ptr);
    // Just read once
    return 0;
}


rt_size_t sifli_transparent_output_handler(void)
{
    rt_uint32_t size = 512;
    uint8_t *ptr;
    int written, offset = 0;
    rt_size_t read_len;
    rt_device_t uart_handle =  sifli_transparent_get_output_dev();
    ipc_queue_handle_t ipc_dev = sifli_get_mbox_read_dev();

    // read 512 byte till read empty
    ptr = bt_mem_alloc(size);
    BT_OOM_ASSERT(ptr);

    do
    {

        // Read from uart
        read_len = rt_device_read(uart_handle, 0, ptr, size);
        if (read_len == 0)
            break;

        if (read_len < size)
            size = read_len;

        // Write to mailbox
        LOG_HEX("uart_data", 16, (rt_uint8_t *)ptr, size);
        if (IPC_QUEUE_INVALID_HANDLE != ipc_dev)
        {
            written = ipc_queue_write(ipc_dev, ptr, size, 10);
            while (written < size)
            {
                size -= written;
                offset += written;
                written = ipc_queue_write(ipc_dev, ptr + offset, size, 10);
            }
        }
        offset = 0;
    }
    while (0);
    bt_mem_free(ptr);

    return read_len;
}



static rt_err_t ble_transparent_output_rx_ind(rt_device_t dev, rt_size_t size)
{
    if (evt_mailbox)
        os_event_flags_set(evt_mailbox, SIFLI_TASK_TRAN_OUTPUT_EVT);
    return 0;
}


static void sifli_transparent_output_init(void)
{
    rt_err_t result;
    rt_uint16_t oflag;

    output_dev_handle = rt_device_find(SIFLI_TRAN_OUTPUT_PORT);
    if (output_dev_handle)
    {
        //oflag = RT_DEVICE_OFLAG_RDWR;
        oflag = RT_DEVICE_OFLAG_RDWR;

        if (output_dev_handle->flag & RT_DEVICE_FLAG_DMA_RX)
        {
            oflag |= RT_DEVICE_FLAG_DMA_RX;
        }
        else
        {
            oflag |= RT_DEVICE_FLAG_INT_RX;
        }
#if 0
        if (env->uart_port->flag & RT_DEVICE_FLAG_DMA_TX)
        {
            oflag |= RT_DEVICE_FLAG_DMA_TX;
        }
        else
        {
            oflag |= RT_DEVICE_FLAG_INT_TX;
        }
#endif
        result = rt_device_open(output_dev_handle, oflag);
        RT_ASSERT(RT_EOK == result);

        rt_device_set_rx_indicate(output_dev_handle, ble_transparent_output_rx_ind);
    }

}


#endif




uint8_t sifli_msg_process(void)
{
    uint8_t type;
    ipc_queue_handle_t dev = sifli_get_mbox_read_dev();
    size_t len = ipc_queue_read(dev, &type, sizeof(type));

    if (len == 0)
    {
        return 0;
    }

    if (type == AHI_TYPE)
    {
        sifli_AHI_process();
    }
    else if (type == HCI_EVT_MSG_TYPE)
    {
        sifli_hci_process();
    }
    return 1;
}


int sifli_ipc_queue_init(void)
{
    ipc_queue_cfg_t q_cfg;
    int32_t ret;
    q_cfg.qid = IO_MB_CH;
    q_cfg.tx_buf_size = TX_BUF_SIZE;
    q_cfg.tx_buf_addr = TX_BUF_ADDR;
    q_cfg.tx_buf_addr_alias = TX_BUF_ADDR_ALIAS;
#if defined(BSP_CHIP_ID_COMPATIBLE) && defined(SF32LB55X)
    q_cfg.rx_buf_addr = RX_BUF_ADDR + ((__HAL_SYSCFG_GET_REVID() & HAL_CHIP_REV_ID_A3) >> 7) * (LPSYS_RAM_SIZE_A3 - LPSYS_RAM_SIZE);
#else
    q_cfg.rx_buf_addr = RX_BUF_ADDR;
#endif
    q_cfg.rx_ind = sifli_mbox_hcpuind;
    q_cfg.user_data = 0;

    ipc_queue_handle = ipc_queue_init(&q_cfg);
    OS_ASSERT(IPC_QUEUE_INVALID_HANDLE != ipc_queue_handle);
    ret = ipc_queue_open(ipc_queue_handle);
    OS_ASSERT(0 == ret);
    return 0;

}

static uint32_t sifli_msg_process_arbitration(rt_tick_t start_timer, rt_tick_t duration)
{
    rt_tick_t curr_timer = rt_tick_get();
    return curr_timer - start_timer >= duration ? 0 : 1;
}

void sifli_mbox_entry(void *param)
{
    uint32_t evt;
    uint32_t ptr;

    os_event_create(evt_mailbox);
    os_mailbox_create(g_bf0_sible_mb, 16, NULL);
#ifdef SIBLES_TRANSPARENT_ENABLE
    is_trans_enable = 1; //TODO : Change to NVDS read
    if (is_trans_enable)
        sifli_transparent_output_init();
#endif
    OS_ASSERT(evt_mailbox && g_bf0_sible_mb);
    ble_power_on();
    // Due to ipc queue init first, to avoid miss isr from stack, just read once in the first loop.
    os_event_flags_set(evt_mailbox, SIFLI_TASK_RECV_EVT);
    LOG_I("Turn on BCPU\n");
    while (1)
    {
        os_event_flags_wait(evt_mailbox, SIFLI_TASK_EVT_ALL, OS_EVENT_FLAG_WAIT_ANY | OS_EVENT_FLAG_CLEAR, OS_WAIT_FORVER, &evt);
        if (evt & SIFLI_TASK_RECV_EVT)
        {
            rt_tick_t start_timer = rt_tick_get();
            while (1)
            {
                uint8_t ret;
                // First handle TX because TX data is simple.
#ifdef SIBLES_TRANSPARENT_ENABLE
                if (is_trans_enable)
                    ret = sifli_msg_transparent_process();
                else
#endif
                    ret = sifli_msg_process();
                if (ret == 0)
                    break;

                // To avoid operation execute too much time, just switch to other evt and set evt again for this operation.
                if (sifli_msg_process_arbitration(start_timer, SIBLES_PROCESS_MAX_TICK) == 0)
                {
                    os_event_flags_set(evt_mailbox, SIFLI_TASK_RECV_EVT);
                    break;
                }
            }
        }
        if (evt & SIFLI_TASK_TRAN_EVT)
        {
            rt_tick_t start_timer = rt_tick_get();
            while (os_mailbox_get(g_bf0_sible_mb, &ptr, 0) == RT_EOK)
            {
                uint32_t ret = sifli_tx_msg_process((uint8_t *)ptr);
                if (ret == 0)
                    break;

                // To avoid operation execute too much time, just switch to other evt and set evt again for this operation.
                if (sifli_msg_process_arbitration(start_timer, SIBLES_PROCESS_MAX_TICK) == 0)
                {
                    os_event_flags_set(evt_mailbox, SIFLI_TASK_TRAN_EVT);
                    break;
                }
            }
        }
#ifdef SIBLES_TRANSPARENT_ENABLE
        if (evt & SIFLI_TASK_TRAN_OUTPUT_EVT)
        {
            rt_tick_t start_timer = rt_tick_get();
            while (1)
            {
                rt_size_t len = sifli_transparent_output_handler();
                if (len == 0)
                    break;

                // To avoid operation execute too much time, just switch to other evt and set evt again for this operation.
                if (sifli_msg_process_arbitration(start_timer, SIBLES_PROCESS_MAX_TICK) == 0)
                {
                    os_event_flags_set(evt_mailbox, SIFLI_TASK_TRAN_OUTPUT_EVT);
                    break;
                }
            }
        }
#endif
    }
}


void sifli_env_init(void)
{

    //g_sifli_sem = rt_sem_create("sibles_sem", 0, RT_IPC_FLAG_FIFO);
    os_sem_create(g_sifli_sem, 0);


    os_thread_create(g_sifli_tid, sifli_mbox_entry, NULL, sifli_mbox_thread_stack, sizeof(sifli_mbox_thread_stack), RT_MAIN_THREAD_PRIORITY + 2, 10);
}


INIT_ENV_EXPORT(sifli_ipc_queue_init);


#else //defined(SOC_SF32LB55X) && defined(SOC_BF0_HCPU)

int32_t ble_event_process(uint16_t const msgid, void const *param,
                          uint16_t const dest_id, uint16_t const src_id)
{
    sibles_msg_para_t *header = sifli_param2msg(param);
    sifli_mbox_process(header, (uint8_t *)param, header->param_len);
    return 0;
}

uint8_t volatile g_block_host;

#if defined(BLUETOOTH) && !defined(BSP_USING_PC_SIMULATOR) && !defined(SOC_SF32LB55X)

rt_err_t sifli_sem_take_ex(int32_t timeout)
{
    rt_err_t err = RT_EINVAL;
    if (g_sifli_sem2)
        err = os_sem_take(g_sifli_sem2, timeout);

    return err;
}

rt_err_t sifli_sem_release_ex(void)
{
    rt_err_t err = RT_EINVAL;
    if (g_sifli_sem2)
        err = os_sem_release(g_sifli_sem2);

    return err;
}


void sifli_mbox_entry(void *param)
{
    os_sem_create(g_sifli_sem2, 0);
    extern int bluetooth_init(void);
    ble_power_on();
    //while (!g_block_host)
#if defined(BSP_USING_DATA_SVC) && !defined(DATA_SVC_MBOX_THREAD_DISABLED)
    // only continue when LCPU is running
    while ((hwp_lpsys_aon->PMR & LPSYS_AON_PMR_CPUWAIT) != 0)
    {
        rt_thread_mdelay(500);
    }

    rt_err_t err = sifli_sem_take_ex(5000);
    if (err != RT_EOK)
        LOG_W("take sema failed %d", err);
    else
        rt_thread_mdelay(1000);

#else
    LOG_D("delay");
    rt_thread_mdelay(1000);
#endif

#if 0
#ifndef BSP_USING_DATA_SVC
    rt_thread_mdelay(1000);
#else
    rt_thread_mdelay(DATA_SVC_WAIT_RF_FUL_DELAY);
#endif
#endif
    bluetooth_init();
    os_sem_delete(g_sifli_sem2);
    g_sifli_sem2 = NULL;
}
#endif


void sifli_env_init(void)
{

#if defined(BLUETOOTH) && !defined(BSP_USING_PC_SIMULATOR) && !defined(SOC_SF32LB55X)

    os_sem_create(g_sifli_sem, 0);
    os_thread_create(g_sifli_tid, sifli_mbox_entry, NULL, NULL, SIFLI_MBOX_THREAD_STACK_SIZE, RT_THREAD_PRIORITY_HIGH - 1, 10);

#if 0
    //TODO Wait controller ready signal
    extern int bluetooth_init(void);
    ble_power_on();
    //while (!g_block_host)
#ifndef BSP_USING_DATA_SVC
    rt_thread_mdelay(1000);
#else
    rt_thread_mdelay(2000);
#endif
    bluetooth_init();
#endif
#endif
}



#endif // defined(SOC_SF32LB55X) && defined(SOC_BF0_HCPU)


sifli_task_id_t sifli_get_stack_id(void)
{
#if defined(SOC_SF32LB55X) && defined(SOC_BF0_HCPU)
    sifli_task_id_t id = TASK_ID_AHI;
#else
    sifli_task_id_t id = TASK_ID_AHI_INT;
#endif
    return id;
}

void sifli_sem_take(void)
{
    if (g_sifli_sem)
        os_sem_take(g_sifli_sem, RT_WAITING_FOREVER);
}

void sifli_sem_release(void)
{
    if (g_sifli_sem)
        os_sem_release(g_sifli_sem);
}

// ATT data handle
void sibles_attm_convert_to128(uint8_t *uuid128, uint8_t *uuid, uint8_t uuid_len)
{
    uint8_t auc_128UUIDBase[ATT_UUID_128_LEN] = ATT_BT_UUID_128;
    uint8_t cursor = 0;

    if ((uuid_len == ATT_UUID_32_LEN) || (uuid_len == ATT_UUID_16_LEN))
    {
        /* place the UUID on 12th to 15th location of UUID */
        cursor = 12;
    }
    else
    {
        /* we consider it's 128 bits UUID */
        uuid_len  = ATT_UUID_128_LEN;
    }

    /* place the UUID on 12th to 15th location of UUID */
    memcpy(&(auc_128UUIDBase[cursor]), uuid, uuid_len);

    /* update value */
    memcpy(&uuid128[0], &auc_128UUIDBase[0], ATT_UUID_128_LEN);
}


void sibles_covert_to_raw_data(void *src, void *dest, uint8_t data_len)
{
    uint8_t *temp = (uint8_t *)dest;
    if (data_len == 2)
    {
        temp[0] = *((uint16_t *)src) & 0xFF;
        temp[1] = (*((uint16_t *)src) & 0xFF00) >> 8;
    }
    else if (data_len == 4)
    {
        temp[0] = *((uint32_t *)src) & 0xFF;
        temp[1] = (*((uint32_t *)src) & 0xFF00) >> 8;
        temp[2] = (*((uint32_t *)src) & 0xFF0000) >> 16;
        temp[3] = (*((uint32_t *)src) & 0xFF000000) >> 24;
    }


}

#if defined(BLUETOOTH) &&!defined(BSP_USING_PC_SIMULATOR) && !defined(SOC_SF32LB55X)
static void sible(uint8_t argc, char **argv)
{
    char *value = NULL;

    if (argc > 1)
    {
        if (strcmp(argv[1], "go") == 0)
        {
            g_block_host = 1;
        }
    }
}
MSH_CMD_EXPORT(sible, Sifli BLE command);
#endif


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
