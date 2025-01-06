/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-09-04     armink       the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include "bf0_hal.h"
#include "drv_flash.h"
#include "section.h"
//#include <ulog.h>
#include <flashdb.h>

#if defined(SOC_BF0_HCPU) && defined(SAVE_ASSERT_CONTEXT_IN_FLASH)
#ifdef ULOG_BACKEND_USING_RAM
    #include "ram_be.h"
#endif /* ULOG_BACKEND_USING_RAM */

#if USING_METRICS_COLLECTOR
    #include "metrics_collector.h"
#endif

#if defined (FLASH_PART20_BASE_ADDR) && defined (FLASH_PART20_SIZE)
#define ASSERT_CONTEXT_BASE_ADDR
#define ASSERT_CONTEXT_LEN         FLASH_PART20_SIZE

#define CB_IS_IN_SRAM_RANGE(addr)    ((((addr) >= HPSYS_RAM0_BASE) && ((addr) < HPSYS_RAM_END)) ? true : false)

typedef struct
{
    uint16_t flag;
    uint16_t block_num;
    int32_t total_len;
} cb_backinfo_t;

typedef struct
{
    uint32_t addr;
    int32_t len;
} cb_block_header_t;


static uint8_t g_save_close = 0;

void open_assert()
{
    g_save_close = 0;

    rt_kprintf("enabel save assert context in addr:0x%x len:0x%x\n ", ASSERT_CONTEXT_BASE_ADDR, ASSERT_CONTEXT_LEN);
}
MSH_CMD_EXPORT_ALIAS(open_assert, open_assert, enable the assert flag);

void close_assert()
{
    g_save_close = 1;

    rt_kprintf("disable save assert context in addr:0x%x len:0x%x\n ", ASSERT_CONTEXT_BASE_ADDR, ASSERT_CONTEXT_LEN);
}
MSH_CMD_EXPORT_ALIAS(close_assert, close_assert, disable the assert flag);


void clear_assert()
{
    if (ASSERT_CONTEXT_BASE_ADDR == 0 || ASSERT_CONTEXT_LEN <= sizeof(cb_backinfo_t))
    {
        rt_kprintf("no flash space for assert context base:0x%x len:0x%x\n ", ASSERT_CONTEXT_BASE_ADDR, ASSERT_CONTEXT_LEN);
        return;
    }

    rt_flash_erase(ASSERT_CONTEXT_BASE_ADDR, ASSERT_CONTEXT_LEN);
}
MSH_CMD_EXPORT_ALIAS(clear_assert, clear_assert, clear the assert flag);

void is_assert()
{
    cb_backinfo_t *flag = (cb_backinfo_t *)ASSERT_CONTEXT_BASE_ADDR;

    if (ASSERT_CONTEXT_BASE_ADDR == 0 || ASSERT_CONTEXT_LEN <= sizeof(cb_backinfo_t))
    {
        rt_kprintf("no flash space for assert context base:0x%x len:0x%x\n ", ASSERT_CONTEXT_BASE_ADDR, ASSERT_CONTEXT_LEN);
        return;
    }

    rt_kprintf("assert flag: 0x%x base:0x%x size:0x%x\n ", flag->flag, ASSERT_CONTEXT_BASE_ADDR, ASSERT_CONTEXT_LEN);

    if ((flag->flag & 0xff00) == 0xaa00)
    {
        rt_kprintf("assert base:0x%x len:0x%x\n ", ASSERT_CONTEXT_BASE_ADDR, flag->total_len);
    }
    else
    {
        rt_kprintf("the device is not assert\n ");
    }
}
MSH_CMD_EXPORT_ALIAS(is_assert, is_assert, check if device assert);

void get_assert()
{
    cb_backinfo_t *flag = (cb_backinfo_t *)ASSERT_CONTEXT_BASE_ADDR;

    if (ASSERT_CONTEXT_BASE_ADDR == 0 || ASSERT_CONTEXT_LEN <= sizeof(cb_backinfo_t))
    {
        rt_kprintf("no flash space for assert context base:0x%x len:0x%x\n ", ASSERT_CONTEXT_BASE_ADDR, ASSERT_CONTEXT_LEN);
        return;
    }

    if ((flag->flag & 0xff00) == 0xaa00)
    {
        rt_kprintf("assert base:0x%x len:0x%x\n ", ASSERT_CONTEXT_BASE_ADDR, flag->total_len);

        rt_hexdump("ASSERT_INFO", 32, (uint8_t *)ASSERT_CONTEXT_BASE_ADDR, flag->total_len);
    }
    else
    {
        rt_kprintf("the device is not assert\n ");
    }
}
MSH_CMD_EXPORT_ALIAS(get_assert, get_assert, get assert info and clear it);

uint32_t ble_log_get_assert(uint32_t *addr)
{
    cb_backinfo_t *flag = (cb_backinfo_t *)ASSERT_CONTEXT_BASE_ADDR;

    if (ASSERT_CONTEXT_BASE_ADDR == 0 || ASSERT_CONTEXT_LEN <= sizeof(cb_backinfo_t))
    {
        rt_kprintf("no flash space for assert context base:0x%x len:0x%x\n ", ASSERT_CONTEXT_BASE_ADDR, ASSERT_CONTEXT_LEN);
        return 0;
    }

    if ((flag->flag & 0xff00) == 0xaa00)
    {
        rt_kprintf("assert base:0x%x len:0x%x\n ", ASSERT_CONTEXT_BASE_ADDR, flag->total_len);

        *addr = ASSERT_CONTEXT_BASE_ADDR;
        return flag->total_len;
    }
    else
    {
        rt_kprintf("the device is not assert\n ");
        return 0;
    }
}

#if ULOG_BACKEND_USING_TSDB
#define FDB_MAX_TSL_LEN 256
void ulog_tsdb_query2flash(fdb_tsl_cb func);
static uint32_t g_tsdb_off = 0;

static bool query_debug_cb(fdb_tsl_t tsl, void *arg)
{
    struct fdb_blob blob;
    uint32_t len = FDB_MAX_TSL_LEN;
    char str[FDB_MAX_TSL_LEN];
    fdb_tsdb_t db = arg;
    fdb_blob_read((fdb_db_t) db, fdb_tsl_to_blob(tsl, fdb_blob_make(&blob, str, FDB_MAX_TSL_LEN)));


    if (tsl->log_len < FDB_MAX_TSL_LEN)
    {
        len = tsl->log_len;
    }

    if (g_tsdb_off + len <= ASSERT_CONTEXT_LEN)
    {
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + g_tsdb_off, (uint8_t *)&str, len);
        g_tsdb_off += len;
    }

    //rt_kprintf("%s", str);

    return false;
}
#endif


uint32_t ble_get_tsdb_log(uint32_t *addr)
{
    cb_backinfo_t *flag = (cb_backinfo_t *)ASSERT_CONTEXT_BASE_ADDR;
#if ULOG_BACKEND_USING_TSDB
    if (ASSERT_CONTEXT_BASE_ADDR == 0 || ASSERT_CONTEXT_LEN <= sizeof(cb_backinfo_t))
    {
        rt_kprintf("no flash space for TSDB log, base:0x%x len:0x%x\n ", ASSERT_CONTEXT_BASE_ADDR, ASSERT_CONTEXT_LEN);
        return 0;
    }

    if ((flag->flag & 0xff00) == 0xaa00)
    {
        rt_kprintf("assert data here, save it and clear the mem first, base:0x%x len:0x%x\n ", ASSERT_CONTEXT_BASE_ADDR, flag->total_len);
        return 0;
    }
    else
    {
        rt_flash_erase(ASSERT_CONTEXT_BASE_ADDR, ASSERT_CONTEXT_LEN);
        g_tsdb_off = 0;
        ulog_tsdb_query2flash(query_debug_cb);

        rt_kprintf("save TSDB log, base:0x%x len:0x%x\n ", ASSERT_CONTEXT_BASE_ADDR, g_tsdb_off);
        *addr = ASSERT_CONTEXT_BASE_ADDR;
        return g_tsdb_off;
    }
#else
    rt_kprintf("ULOG_BACKEND_USING_TSDB not enable\n ");
    return 0;
#endif

}


#if USING_METRICS_COLLECTOR
static uint32_t g_mc_off = 0;

static bool dump_metrics_callback(void *data, uint32_t data_len, uint32_t time)
{
    uint32_t packet_hdr[4] = {time, 0, data_len, data_len};
    uint32_t total_len;

    total_len = sizeof(packet_hdr) + data_len;

    if (g_mc_off + total_len <= ASSERT_CONTEXT_LEN)
    {
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + g_mc_off, (const uint8_t *)&packet_hdr[0], sizeof(packet_hdr));
        g_mc_off += sizeof(packet_hdr);

        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + g_mc_off, data, data_len);
        g_mc_off += data_len;
    }

    return false;
}
#endif

uint32_t ble_get_tsdb_metrics(uint32_t *addr)
{
    cb_backinfo_t *flag = (cb_backinfo_t *)ASSERT_CONTEXT_BASE_ADDR;

#if USING_METRICS_COLLECTOR
    uint8_t fileheader[] = { 0xD4, 0xC3, 0xB2, 0xA1, 0x02, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0xa0, 0x00, 0x00, 0x00 };

    if (ASSERT_CONTEXT_BASE_ADDR == 0 || ASSERT_CONTEXT_LEN <= sizeof(cb_backinfo_t))
    {
        rt_kprintf("no flash space for METRICS log, base:0x%x len:0x%x\n ", ASSERT_CONTEXT_BASE_ADDR, ASSERT_CONTEXT_LEN);
        return 0;
    }

    if ((flag->flag & 0xff00) == 0xaa00)
    {
        rt_kprintf("assert data here, save it and clear the mem first, base:0x%x len:0x%x\n ", ASSERT_CONTEXT_BASE_ADDR, flag->total_len);
        return 0;
    }
    else
    {
        rt_flash_erase(ASSERT_CONTEXT_BASE_ADDR, ASSERT_CONTEXT_LEN);
        g_mc_off = 0;

        if (g_mc_off + sizeof(fileheader) <= ASSERT_CONTEXT_LEN)
        {
            rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + g_mc_off, fileheader, sizeof(fileheader));
            g_mc_off += sizeof(fileheader);
        }

        mc_read_raw_metrics(dump_metrics_callback);

        rt_kprintf("save METRICS log, base:0x%x len:0x%x\n ", ASSERT_CONTEXT_BASE_ADDR, g_mc_off);
        *addr = ASSERT_CONTEXT_BASE_ADDR;
        return g_tsdb_off;

    }
#else
    rt_kprintf("USING_METRICS_COLLECTOR not enable\n ");
    return 0;
#endif
}

uint32_t ble_wfile_get_mem(uint32_t *addr)
{
    cb_backinfo_t *flag = (cb_backinfo_t *)ASSERT_CONTEXT_BASE_ADDR;

    if (ASSERT_CONTEXT_BASE_ADDR == 0 || ASSERT_CONTEXT_LEN <= sizeof(cb_backinfo_t))
    {
        rt_kprintf("no flash space for assert context base:0x%x len:0x%x\n ", ASSERT_CONTEXT_BASE_ADDR, ASSERT_CONTEXT_LEN);
        return 0;
    }

    rt_kprintf("mem for ble wfile:0x%x len:0x%x\n ", ASSERT_CONTEXT_BASE_ADDR, ASSERT_CONTEXT_LEN);

    *addr = ASSERT_CONTEXT_BASE_ADDR;

    rt_flash_erase(ASSERT_CONTEXT_BASE_ADDR, ASSERT_CONTEXT_LEN);

    return ASSERT_CONTEXT_LEN;

}

uint32_t ble_wfile_to_mem(uint8_t *src, uint32_t addr, uint32_t len)
{
    if ((addr >= ASSERT_CONTEXT_BASE_ADDR) &&
            (addr + len <= ASSERT_CONTEXT_BASE_ADDR + ASSERT_CONTEXT_LEN))
    {
        return rt_flash_write(addr, (uint8_t *)src, len);
    }
    else
    {
        rt_kprintf("ble_wfile_to_mem error: 0x%x len:0x%x\n ", addr, len);
    }

    return 0;

}



void get_assert_context_mem(uint32_t *addr, uint32_t *len)
{
    *addr = ASSERT_CONTEXT_BASE_ADDR;
    *len = ASSERT_CONTEXT_LEN;
}

//-------------------------------------------------------------------------
#ifdef SAVE_IPREG_CONTEXT
#if 0 //def ULOG_BACKEND_USING_RAM
rt_err_t rt_register_backup(uint32_t addr, uint32_t len, cb_backinfo_t *backinfo)
{

    rt_err_t ret = RT_EOK;
    uint8_t *reg_buff = RT_NULL;
    rt_uint32_t size = 0;
    int32_t max_size = ASSERT_CONTEXT_LEN;
    cb_block_header_t block_header;

    block_header.addr = addr;
    block_header.len = len;

    reg_buff = (uint8_t *)ulog_ram_be_buf_get(&size);

    //no buff for use
    if (reg_buff == RT_NULL || size < len)
    {
        return ret;
    }

    if (block_header.len + sizeof(block_header) + backinfo->total_len > max_size)
    {
        block_header.len = max_size - (backinfo->total_len + sizeof(block_header));
        ret = RT_EFULL;
        backinfo->flag = 0xaa33;
    }
    if (block_header.len > 0)
    {
        memcpy((void *)reg_buff, (void *)block_header.addr, block_header.len);
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)&block_header, sizeof(block_header));
        backinfo->total_len += sizeof(block_header);
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, reg_buff, block_header.len);
        //rt_flash_write(start_addr + backinfo->total_len, (uint8_t *)block_header.addr, block_header.len);
        backinfo->total_len += block_header.len;
        backinfo->block_num++;
    }

    return ret;
}
#else
rt_err_t rt_register_backup(uint32_t addr, uint32_t len, cb_backinfo_t *backinfo)
{
    rt_err_t ret = RT_EOK;
    uint8_t reg_buff[64] = {0};
    uint16_t icnt = 0;
    int32_t max_size = ASSERT_CONTEXT_LEN;
    cb_block_header_t block_header;

    block_header.addr = addr;
    block_header.len = len;

    if (block_header.len + sizeof(block_header) + backinfo->total_len > max_size)
    {
        block_header.len = max_size - (backinfo->total_len + sizeof(block_header));
        ret = RT_EFULL;
        backinfo->flag = 0xaa33;
    }
    if (block_header.len > 0)
    {
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)&block_header, sizeof(block_header));
        backinfo->total_len += sizeof(block_header);

        for (icnt = 0; icnt < block_header.len / sizeof(reg_buff); icnt++)
        {
            memcpy((void *)reg_buff, (void *)(block_header.addr + icnt * sizeof(reg_buff)), sizeof(reg_buff));
            rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, reg_buff, sizeof(reg_buff));
            //rt_flash_write(start_addr + backinfo->total_len, (uint8_t *)block_header.addr, block_header.len);
            backinfo->total_len += sizeof(reg_buff);
        }

        if (block_header.len % sizeof(reg_buff))
        {
            memcpy((void *)reg_buff, (void *)(block_header.addr + icnt * sizeof(reg_buff)), block_header.len % sizeof(reg_buff));
            rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, reg_buff, block_header.len % sizeof(reg_buff));
            //rt_flash_write(start_addr + backinfo->total_len, (uint8_t *)block_header.addr, block_header.len);
            backinfo->total_len += block_header.len % sizeof(reg_buff);
        }

        backinfo->block_num++;
    }

    return ret;
}
#endif  //ULOG_BACKEND_USING_RAM

rt_err_t rt_register_backup_debug(cb_backinfo_t *backinfo)
{
    rt_err_t ret = RT_EOK;

//-----LCDC1_BASE
    ret = rt_register_backup(LCDC1_BASE, sizeof(LCD_IF_TypeDef), backinfo);
    if (ret == RT_EFULL)
    {
        return RT_EFULL;
    }
    //-----DSI_HOST_BASE
    ret = rt_register_backup(DSI_HOST_BASE, sizeof(DSI_HOST_TypeDef), backinfo);
    if (ret == RT_EFULL)
    {
        return RT_EFULL;
    }

    //-----DSI_PHY_BASE
    ret = rt_register_backup(DSI_PHY_BASE, sizeof(DSI_PHY_TypeDef), backinfo);
    if (ret == RT_EFULL)
    {
        return RT_EFULL;
    }

    //-----EPIC_BASE
    ret = rt_register_backup(EPIC_BASE, sizeof(EPIC_TypeDef), backinfo);
    if (ret == RT_EFULL)
    {
        return RT_EFULL;
    }

    //-----EZIP_BASE
    ret = rt_register_backup(EZIP_BASE, sizeof(EZIP_TypeDef), backinfo);
    if (ret == RT_EFULL)
    {
        return RT_EFULL;
    }

    //-----GPIO1_BASE
    ret = rt_register_backup(GPIO1_BASE, sizeof(GPIO1_TypeDef), backinfo);
    if (ret == RT_EFULL)
    {
        return RT_EFULL;
    }

    //-----GPIO2_BASE
    ret = rt_register_backup(GPIO2_BASE, sizeof(GPIO2_TypeDef), backinfo);
    if (ret == RT_EFULL)
    {
        return RT_EFULL;
    }

    //-----PINMUX1_BASE
    ret = rt_register_backup(PINMUX1_BASE, sizeof(HPSYS_PINMUX_TypeDef), backinfo);
    if (ret == RT_EFULL)
    {
        return RT_EFULL;
    }

    //-----PINMUX2_BASE
    ret = rt_register_backup(PINMUX2_BASE, sizeof(LPSYS_PINMUX_TypeDef), backinfo);
    if (ret == RT_EFULL)
    {
        return RT_EFULL;
    }

#ifdef SF32LB55X
    //-----BLE_RFC_BASE
    ret = rt_register_backup(BLE_RFC_BASE, sizeof(BLE_RF_DIG_TypeDef), backinfo);
    if (ret == RT_EFULL)
    {
        return RT_EFULL;
    }

    //-----BLE_MAC_BASE
    ret = rt_register_backup(BLE_MAC_BASE, sizeof(BLE_MAC_TypeDef), backinfo);
    if (ret == RT_EFULL)
    {
        return RT_EFULL;
    }
#else
    //-----BT_RFC_BASE
    ret = rt_register_backup(BT_RFC_REG_BASE, sizeof(BT_RFC_TypeDef), backinfo);
    if (ret == RT_EFULL)
    {
        return RT_EFULL;
    }

    //-----BT_MAC_BASE
    ret = rt_register_backup(BT_MAC_BASE, sizeof(BT_MAC_TypeDef), backinfo);
    if (ret == RT_EFULL)
    {
        return RT_EFULL;
    }

#endif /* SF32LB55X */


    return RT_EOK;
}

#endif  //SAVE_IPREG_CONTEXT

//-------------------------------------------------------------------------
#ifdef SAVE_HCPU_HEAP_CONTEXT

#define RT_MEM_BACKUP_OPT
#define MIN_SIZE 12
#define MIN_SIZE_ALIGNED     RT_ALIGN(MIN_SIZE, RT_ALIGN_SIZE)
#define SIZEOF_STRUCT_MEM    RT_ALIGN(sizeof(struct heap_mem), RT_ALIGN_SIZE)

struct heap_mem
{
    /* magic and used flag */
    rt_uint16_t magic;
    rt_uint16_t used;

    volatile rt_size_t next, prev;
    rt_uint32_t size;       /**< requested memory size excluding header*/
#ifdef RT_USING_MEMTRACE
#ifdef RT_MEM_RECORD_THREAD_NAME
    rt_uint8_t thread[4];   /**< thread name */
#endif  /* RT_MEM_RECORD_THREAD_NAME */
    rt_uint32_t ret_addr;
    rt_tick_t   tick;
#endif
};

extern rt_uint32_t rt_mem_base(void);
extern rt_uint32_t rt_mem_tail(void);

rt_err_t rt_mem_backup_debug(cb_backinfo_t *backinfo)
{
    rt_err_t ret = RT_EOK;
    cb_block_header_t block_header;
    int32_t max_size = ASSERT_CONTEXT_LEN;
    rt_uint8_t *heap_ptr = (rt_uint8_t *)rt_mem_base();
    struct heap_mem *heap_end = (struct heap_mem *)rt_mem_tail();

#ifndef RT_MEM_BACKUP_OPT
    block_header.addr = (uint32_t)heap_ptr;
    block_header.len = rt_mem_used_size();
    if (block_header.len + sizeof(block_header) + backinfo->total_len > max_size)
    {
        block_header.len = max_size - (backinfo->total_len + sizeof(block_header));
        ret = RT_EFULL;
    }

    if (block_header.len > 0)
    {
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)&block_header, sizeof(block_header));
        backinfo->total_len += sizeof(block_header);
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)block_header.addr, block_header.len);
        backinfo->total_len += block_header.len; //((block_header.len+3)/4)*4;
        backinfo->block_num++;
    }
    if (ret == RT_EFULL)
    {
        return RT_EFULL;
    }
#else
    struct heap_mem *block;

    block = (struct heap_mem *)heap_ptr;
    while (block < heap_end)
    {
        block_header.addr = (uint32_t)block;

        if (block->used)
        {
            /* copy header and data */
            block_header.len = (rt_uint32_t)block->next - ((rt_uint32_t)block - (rt_uint32_t)heap_ptr);
        }
        else
        {
            /* only copy header */
            block_header.len = SIZEOF_STRUCT_MEM;
        }

        if (block_header.len + sizeof(block_header) + backinfo->total_len > max_size)
        {
            block_header.len = max_size - (backinfo->total_len + sizeof(block_header));
            ret = RT_EFULL;
        }

        if (block_header.len > 0)
        {
            rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)&block_header, sizeof(block_header));
            backinfo->total_len += sizeof(block_header);
            rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)block_header.addr, block_header.len);
            backinfo->total_len += block_header.len; //((block_header.len+3)/4)*4;
            backinfo->block_num++;
        }
        if (ret == RT_EFULL)
        {
            return RT_EFULL;
        }
        block = (struct heap_mem *)&heap_ptr[block->next];
    }

#endif

    return RT_EOK;

}

//***********************
#define RT_MEMHEAP_BACKUP_OPT
#define RT_MEMHEAP_MAGIC        0x1ea01ea0
#define RT_MEMHEAP_MASK         0xfffffffe
#define RT_MEMHEAP_USED         0x01
#define RT_MEMHEAP_FREED        0x00
#define RT_MEMHEAP_IS_USED(i)   ((i)->magic & RT_MEMHEAP_USED)
#define RT_MEMHEAP_MINIALLOC    12
#define RT_MEMHEAP_SIZE         RT_ALIGN(sizeof(struct rt_memheap_item), RT_ALIGN_SIZE)
#define MEMITEM_SIZE(item)      ((rt_uint32_t)item->next - (rt_uint32_t)item - RT_MEMHEAP_SIZE)
#define RT_MEMHEAP_TAILER(memheap)   ((struct rt_memheap_item *)((rt_uint32_t)((memheap)->start_addr) + (memheap)->pool_size - RT_MEMHEAP_SIZE))

rt_err_t rt_memheap_backup_debug(struct rt_memheap *heap, cb_backinfo_t *backinfo)
{
    rt_err_t ret = RT_EOK;
    cb_block_header_t block_header;
    int32_t max_size = ASSERT_CONTEXT_LEN;

#ifndef RT_MEMHEAP_BACKUP_OPT
    block_header.addr = (uint32_t)heap->start_addr;
    block_header.len = rt_memheap_used_size(heap);
    if (block_header.len + sizeof(block_header) + backinfo->total_len > max_size)
    {
        block_header.len = max_size - (backinfo->total_len + sizeof(block_header));
        ret = RT_EFULL;
    }

    if (block_header.len > 0)
    {
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)&block_header, sizeof(block_header));
        backinfo->total_len += sizeof(block_header);
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)block_header.addr, block_header.len);
        backinfo->total_len += block_header.len; //((block_header.len+3)/4)*4;
        backinfo->block_num++;
    }
    if (ret == RT_EFULL)
    {
        return RT_EFULL;
    }
#else
    struct rt_memheap_item *next;

    next = heap->block_list;

    while (next < RT_MEMHEAP_TAILER(heap))
    {
        block_header.addr = (uint32_t)next;

        if (RT_MEMHEAP_IS_USED(next))
        {
            /* copy header and data */
            block_header.len = (rt_uint32_t)next->next - (rt_uint32_t)next;
        }
        else
        {
            /* only copy header */
            block_header.len = RT_MEMHEAP_SIZE;
        }
        if (block_header.len + sizeof(block_header) + backinfo->total_len > max_size)
        {
            block_header.len = max_size - (backinfo->total_len + sizeof(block_header));
            ret = RT_EFULL;
        }

        if (block_header.len > 0)
        {
            rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)&block_header, sizeof(block_header));
            backinfo->total_len += sizeof(block_header);
            rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)block_header.addr, block_header.len);
            backinfo->total_len += block_header.len; //((block_header.len+3)/4)*4;
            backinfo->block_num++;
        }
        if (ret == RT_EFULL)
        {
            return RT_EFULL;
        }
        next = next->next;
    }
#endif

    return RT_EOK;
}

rt_err_t rt_hcpu_heap_backup_debug(cb_backinfo_t *backinfo)
{
    rt_err_t ret = RT_EOK;
#ifdef RT_USING_SMALL_MEM
#ifndef RT_USING_MEMHEAP_AS_HEAP
    uint32_t addr;
    do
    {
        addr = rt_mem_base();
        if (!CB_IS_IN_SRAM_RANGE(addr))
        {
            break;
        }

        ret = rt_mem_backup_debug(backinfo);
        if (RT_EOK != ret)
        {
            ret = RT_EFULL;
            backinfo->flag = 0xaa66;
            return ret;
        }
    }
    while (0);
#endif
#endif

#ifdef RT_USING_MEMHEAP
    struct rt_object_information *info;
    struct rt_list_node *heap_list;
    struct rt_memheap *mh;
    struct rt_list_node *node;

    info = rt_object_get_information(RT_Object_Class_MemHeap);
    heap_list = &info->object_list;
    for (node = heap_list->next; node != heap_list; node = node->next)
    {
        mh = (struct rt_memheap *)rt_list_entry(node, struct rt_object, list);

        if (!CB_IS_IN_SRAM_RANGE((uint32_t)mh->start_addr))
        {
            continue;
        }

        ret = rt_memheap_backup_debug(mh, backinfo);
        if (RT_EOK != ret)
        {
            ret = RT_EFULL;
            backinfo->flag = 0xaa66;
            return ret;
        }
    }
#endif
    return ret;
}
#endif /* SAVE_HCPU_HEAP_CONTEXT */

//-------------------------------------------------------------------------
#ifdef SAVE_HCPU_STATIC_CONTEXT

EXEC_REGION_DEF(ER_ITCM$$RW);
EXEC_REGION_DEF(ER_ITCM$$ZI);
EXEC_REGION_DEF(RW_IRAM_RET$$RW);
EXEC_REGION_DEF(RW_IRAM_RET$$ZI);
EXEC_REGION_DEF(ER_ITCM$$RO);
EXEC_REGION_DEF(ER_IROM1_EX$$RO);
EXEC_REGION_DEF(RW_IRAM1);
EXEC_REGION_LOAD_SYM_DEF(ER_ITCM$$RO);
EXEC_REGION_LOAD_SYM_DEF(ER_IROM1_EX$$RO);
EXEC_REGION_DEF(RW_IRAM1);
#define STATIC_BACKUP_REGION_START_ADDR (EXEC_REGION_START_ADDR(RW_IRAM1))
#define STATIC_BACKUP_REGION_SIZE       ((uint32_t)HEAP_BEGIN - (uint32_t)STATIC_BACKUP_REGION_START_ADDR)

#define KVDB_SECT_NAME    kvdb
SECTION_DEF(KVDB_SECT_NAME, void);

#define STATIC_KVDB_SECTION_START    SECTION_START_ADDR(KVDB_SECT_NAME)
#define STATIC_KVDB_SECTION_END      SECTION_END_ADDR(KVDB_SECT_NAME)

rt_err_t rt_hcpu_static_backup_debug(cb_backinfo_t *backinfo)
{
    rt_err_t ret = RT_EOK;
    cb_block_header_t block_header;
    int32_t max_size = ASSERT_CONTEXT_LEN;
    uint8_t temp_buff[64] = {0};
    int icnt = 0;

    block_header.addr = (uint32_t)STATIC_BACKUP_REGION_START_ADDR;
    if ((uint32_t)STATIC_KVDB_SECTION_START >= (uint32_t)STATIC_BACKUP_REGION_START_ADDR)
    {
        block_header.len = (uint32_t)STATIC_KVDB_SECTION_START - (uint32_t)STATIC_BACKUP_REGION_START_ADDR;
    }
    else
    {
        block_header.len = STATIC_BACKUP_REGION_SIZE;
    }
    if (block_header.len + sizeof(block_header) + backinfo->total_len > max_size)
    {
        block_header.len = max_size - (backinfo->total_len + sizeof(block_header));
        ret = RT_EFULL;
        backinfo->flag = 0xaa00;
    }
    if (block_header.len > 0)
    {
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)&block_header, sizeof(block_header));
        backinfo->total_len += sizeof(block_header);
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)block_header.addr, block_header.len);
        backinfo->total_len += block_header.len; //((block_header.len+3)/4)*4;
        backinfo->block_num++;
    }
    if (ret == RT_EFULL)
    {
        return ret;
    }

    if (((uint32_t)STATIC_KVDB_SECTION_END > (uint32_t)STATIC_KVDB_SECTION_START)
            && ((uint32_t)STATIC_KVDB_SECTION_END < ((uint32_t)STATIC_BACKUP_REGION_START_ADDR + STATIC_BACKUP_REGION_SIZE))
            && ((uint32_t)STATIC_KVDB_SECTION_END > (uint32_t)STATIC_BACKUP_REGION_START_ADDR))
    {
        block_header.addr = (uint32_t)STATIC_KVDB_SECTION_END;
        block_header.len = ((uint32_t)STATIC_BACKUP_REGION_START_ADDR + STATIC_BACKUP_REGION_SIZE) - (uint32_t)STATIC_KVDB_SECTION_END;

        if (block_header.len + sizeof(block_header) + backinfo->total_len > max_size)
        {
            block_header.len = max_size - (backinfo->total_len + sizeof(block_header));
            ret = RT_EFULL;
            backinfo->flag = 0xaa00;
        }
        if (block_header.len > 0)
        {
            rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)&block_header, sizeof(block_header));
            backinfo->total_len += sizeof(block_header);
            rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)block_header.addr, block_header.len);
            backinfo->total_len += block_header.len; //((block_header.len+3)/4)*4;
            backinfo->block_num++;
        }
        if (ret == RT_EFULL)
        {
            return ret;
        }
        if (block_header.len + sizeof(block_header) + backinfo->total_len > max_size)
        {
            block_header.len = max_size - (backinfo->total_len + sizeof(block_header));
            ret = RT_EFULL;
            backinfo->flag = 0xaa00;
        }
        if (block_header.len > 0)
        {
            rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)&block_header, sizeof(block_header));
            backinfo->total_len += sizeof(block_header);
            rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)block_header.addr, block_header.len);
            backinfo->total_len += block_header.len; //((block_header.len+3)/4)*4;
            backinfo->block_num++;
        }
        if (ret == RT_EFULL)
        {
            return ret;
        }
    }

    block_header.addr = (uint32_t)EXEC_REGION_START_ADDR(ER_ITCM$$RW);
    block_header.len = (uint32_t)EXEC_REGION_END_ADDR(ER_ITCM$$ZI) - (uint32_t)EXEC_REGION_START_ADDR(ER_ITCM$$RW);
    if (block_header.len + sizeof(block_header) + backinfo->total_len > max_size)
    {
        block_header.len = max_size - (backinfo->total_len + sizeof(block_header));
        ret = RT_EFULL;
        backinfo->flag = 0xaa00;
    }
    if (block_header.len > 0)
    {
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)&block_header, sizeof(block_header));
        backinfo->total_len += sizeof(block_header);

        for (icnt = 0; icnt < block_header.len / sizeof(temp_buff); icnt++)
        {
            memcpy((void *)temp_buff, (void *)(block_header.addr + icnt * sizeof(temp_buff)), sizeof(temp_buff));
            rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, temp_buff, sizeof(temp_buff));
            //rt_flash_write(start_addr + backinfo->total_len, (uint8_t *)block_header.addr, block_header.len);
            backinfo->total_len += sizeof(temp_buff);
        }

        if (block_header.len % sizeof(temp_buff))
        {
            memcpy((void *)temp_buff, (void *)(block_header.addr + icnt * sizeof(temp_buff)), block_header.len % sizeof(temp_buff));
            rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, temp_buff, block_header.len % sizeof(temp_buff));
            //rt_flash_write(start_addr + backinfo->total_len, (uint8_t *)block_header.addr, block_header.len);
            backinfo->total_len += block_header.len % sizeof(temp_buff);
        }

        //rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)block_header.addr, block_header.len);
        //backinfo->total_len += block_header.len; //((block_header.len+3)/4)*4;
        backinfo->block_num++;
    }
    if (ret == RT_EFULL)
    {
        return ret;
    }

    block_header.addr = (uint32_t)EXEC_REGION_START_ADDR(RW_IRAM_RET$$RW);
    block_header.len = (uint32_t)EXEC_REGION_END_ADDR(RW_IRAM_RET$$ZI) - (uint32_t)EXEC_REGION_START_ADDR(RW_IRAM_RET$$RW);
    if (block_header.len + sizeof(block_header) + backinfo->total_len > max_size)
    {
        block_header.len = max_size - (backinfo->total_len + sizeof(block_header));
        ret = RT_EFULL;
        backinfo->flag = 0xaa00;
    }
    if (block_header.len > 0)
    {
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)&block_header, sizeof(block_header));
        backinfo->total_len += sizeof(block_header);

        for (icnt = 0; icnt < block_header.len / sizeof(temp_buff); icnt++)
        {
            memcpy((void *)temp_buff, (void *)(block_header.addr + icnt * sizeof(temp_buff)), sizeof(temp_buff));
            rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, temp_buff, sizeof(temp_buff));
            //rt_flash_write(start_addr + backinfo->total_len, (uint8_t *)block_header.addr, block_header.len);
            backinfo->total_len += sizeof(temp_buff);
        }

        if (block_header.len % sizeof(temp_buff))
        {
            memcpy((void *)temp_buff, (void *)(block_header.addr + icnt * sizeof(temp_buff)), block_header.len % sizeof(temp_buff));
            rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, temp_buff, block_header.len % sizeof(temp_buff));
            //rt_flash_write(start_addr + backinfo->total_len, (uint8_t *)block_header.addr, block_header.len);
            backinfo->total_len += block_header.len % sizeof(temp_buff);
        }

        //rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)block_header.addr, block_header.len);
        //backinfo->total_len += block_header.len; //((block_header.len+3)/4)*4;
        backinfo->block_num++;
    }
    if (ret == RT_EFULL)
    {
        return ret;
    }

    return RT_EOK;
}
#endif  //SAVE_HCPU_STATIC_CONTEXT

//-------------------------------------------------------------------------
#ifdef SAVE_HCPU_STACK_CONTEXT
rt_err_t rt_hcpu_stack_backup_debug(cb_backinfo_t *backinfo)
{
    struct rt_object_information *info;
    struct rt_thread *thread;
    struct rt_list_node *node;
    struct rt_list_node *thread_list;
    rt_uint32_t psp;
    int32_t max_size = ASSERT_CONTEXT_LEN;
    cb_block_header_t block_header;
    rt_err_t ret = RT_EOK;
    //struct rt_thread *curr_thread;

    //curr_thread = rt_thread_self();

    info = rt_object_get_information(RT_Object_Class_Thread);
    thread_list = &info->object_list;
    for (node = thread_list->next; node != thread_list; node = node->next)
    {
        thread = rt_list_entry(node, struct rt_thread, list);
#if 0
        if (thread == curr_thread)
        {
            /* don't save stack of current thread */
            continue;
        }
#endif

        if (!CB_IS_IN_SRAM_RANGE((uint32_t)thread->stack_addr))
        {
            continue;
        }

        psp = __get_PSP();
#if defined(ARCH_CPU_STACK_GROWS_UPWARD)
        block_header.addr = thread->stack_addr;

        if ((psp < (rt_uint32_t)thread->stack_addr) || (psp >= (rt_uint32_t)thread->stack_addr + thread->stack_size))
        {
            /* PSP is not current thread, stack top is saved by thread->sp,
               otherwise, use PSP as stack top */
            psp = (rt_uint32_t)thread->sp;
        }

        /* including 4 bytes pointed by SP */
        block_header.len = (rt_uint32_t)thread->sp - (rt_uint32_t)thread->stack_addr + 4;
#else

        if ((psp >= (rt_uint32_t)thread->stack_addr) && (psp < (rt_uint32_t)thread->stack_addr + thread->stack_size))
        {
            /* PSP is current thread, use the actual stack top */
            block_header.addr = (uint32_t)psp;
        }
        else
        {
            /* PSP is not current thread, stack top is saved in thread->sp */
            block_header.addr = (uint32_t)thread->sp;
            psp = (uint32_t)thread->sp;
        }

        /* including 4 bytes pointed by SP */
        block_header.len = psp - (rt_uint32_t)thread->stack_addr;
        block_header.len = thread->stack_size - block_header.len;
#endif

        if (block_header.len + sizeof(block_header) + backinfo->total_len > max_size)
        {
            block_header.len = max_size - (backinfo->total_len + sizeof(block_header));
            ret = RT_EFULL;
            backinfo->flag = 0xaa11;
        }

        if (block_header.len > 0)
        {
            rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)&block_header, sizeof(block_header));
            backinfo->total_len += sizeof(block_header);
            rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)block_header.addr, block_header.len);
            backinfo->total_len += block_header.len; //((block_header.len+3)/4)*4;
            backinfo->block_num++;
        }
        if (ret == RT_EFULL)
        {
            return ret;
        }
    }

    return RT_EOK;
}
#endif  //SAVE_HCPU_STACK_CONTEXT

//-------------------------------------------------------------------------
#ifdef ULOG_BACKEND_USING_RAM
rt_err_t rt_hcpu_log_backup_debug(cb_backinfo_t *backinfo)
{
    rt_err_t ret = RT_EOK;
    ulog_ram_be_buf_t *pLog = ulog_ram_be_buf_get(RT_NULL);
    cb_block_header_t block_header_loop = {0};
    cb_block_header_t block_header = {0};
    int32_t max_size = ASSERT_CONTEXT_LEN;

    if (pLog->wr_offset + sizeof(block_header) + backinfo->total_len > max_size)
    {
        block_header.len = max_size - (backinfo->total_len + sizeof(block_header));
        block_header.addr = (uint32_t)&pLog->buf[pLog->wr_offset - block_header.len];
        ret = RT_EFULL;
        backinfo->flag = 0xaa22;
    }
    else
    {
        if (pLog->full)
        {
            if (ULOG_RAM_BE_BUF_SIZE + sizeof(block_header) + backinfo->total_len > max_size)
            {
                block_header_loop.len = max_size - (backinfo->total_len + sizeof(block_header) + pLog->wr_offset);
                block_header_loop.addr = (uint32_t)&pLog->buf[ULOG_RAM_BE_BUF_SIZE - block_header_loop.len];
                ret = RT_EFULL;
                backinfo->flag = 0xaa22;
            }
            else
            {
                block_header_loop.len = ULOG_RAM_BE_BUF_SIZE - pLog->wr_offset;
                block_header_loop.addr = (uint32_t)&pLog->buf[pLog->wr_offset];
            }
        }

        block_header.addr = (uint32_t)&pLog->buf[0];
        block_header.len = pLog->wr_offset;

    }

    if (block_header.len > 0)
    {
        cb_block_header_t block_header_special = {0};
        block_header_special.addr = 0xffffffff;
        block_header_special.len = block_header_loop.len + block_header.len;
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)&block_header_special, sizeof(block_header));
        backinfo->total_len += sizeof(block_header);
        if (block_header_loop.len > 0)
        {
            rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)block_header_loop.addr, block_header_loop.len);
            backinfo->total_len += block_header_loop.len;
        }
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)block_header.addr, block_header.len);
        backinfo->total_len += block_header.len;
        backinfo->block_num++;
    }

    return ret;
}
#endif  //ULOG_BACKEND_USING_RAM

//-------------------------------------------------------------------------
#ifdef BSP_USING_PSRAM
rt_err_t rt_psram_backup_debug(cb_backinfo_t *backinfo)
{
    rt_err_t ret = RT_EOK;
    int32_t max_size = ASSERT_CONTEXT_LEN;
    cb_block_header_t block_header = {0};

    block_header.addr = PSRAM_BASE;
    block_header.len = 1024;

    if (block_header.len + sizeof(block_header) + backinfo->total_len > max_size)
    {
        block_header.len = max_size - (backinfo->total_len + sizeof(block_header));
        ret = RT_EFULL;
        backinfo->flag = 0xaa21;
    }
    if (block_header.len > 0)
    {
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)&block_header, sizeof(block_header));
        backinfo->total_len += sizeof(block_header);
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)block_header.addr, block_header.len);
        backinfo->total_len += block_header.len;
        backinfo->block_num++;
    }

    return ret;
}
#endif

//-------------------------------------------------------------------------
#ifdef SAVE_LCPU_STATIC_CONTEXT
#define _LPSYS_SRAM_SIZE_ (224*1024)
extern uint32_t lcpu_ramcode_len();
rt_err_t rt_lcpu_static_backup_debug(cb_backinfo_t *backinfo)
{
    rt_err_t ret = RT_EOK;
    int32_t max_size = ASSERT_CONTEXT_LEN;
    uint32_t len = lcpu_ramcode_len();
    cb_block_header_t block_header = {0};

    if (len <= LPSYS_ITCM_SIZE)
    {
        block_header.addr = LPSYS_SRAM_BASE;
        block_header.len = _LPSYS_SRAM_SIZE_;
    }
    else
    {
        block_header.addr = LPSYS_SRAM_BASE + (len - LPSYS_ITCM_SIZE);
        block_header.len = _LPSYS_SRAM_SIZE_ - (len - LPSYS_ITCM_SIZE);
    }

    if (block_header.len + sizeof(block_header) + backinfo->total_len > max_size)
    {
        block_header.len = max_size - (backinfo->total_len + sizeof(block_header));
        ret = RT_EFULL;
        backinfo->flag = 0xaa44;
    }

    if (block_header.len > 0)
    {
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)&block_header, sizeof(block_header));
        backinfo->total_len += sizeof(block_header);
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)block_header.addr, block_header.len);
        backinfo->total_len += block_header.len; //((block_header.len+3)/4)*4;
        backinfo->block_num++;
    }

    return ret;
}
#endif /* SAVE_LCPU_STATIC_CONTEXT */

//-------------------------------------------------------------------------
#ifdef SAVE_LCPU_DTCM_CONTEXT
rt_err_t rt_lcpu_dtcm_backup_debug(cb_backinfo_t *backinfo)
{
    rt_err_t ret = RT_EOK;
    int32_t max_size = ASSERT_CONTEXT_LEN;
    cb_block_header_t block_header = {0};

    block_header.addr = LCPU_DTCM_ADDR_2_HCPU_ADDR(LPSYS_DTCM_BASE);
    block_header.len = LPSYS_DTCM_SIZE;

    if (block_header.len + sizeof(block_header) + backinfo->total_len > max_size)
    {
        block_header.len = max_size - (backinfo->total_len + sizeof(block_header));
        ret = RT_EFULL;
        backinfo->flag = 0xaa55;
    }

    if (block_header.len > 0)
    {
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)&block_header, sizeof(block_header));
        backinfo->total_len += sizeof(block_header);
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR + backinfo->total_len, (uint8_t *)block_header.addr, block_header.len);
        backinfo->total_len += block_header.len; //((block_header.len+3)/4)*4;
        backinfo->block_num++;
    }

    return ret;
}
#endif /* SAVE_LCPU_DTCM_CONTEXT */

extern HAL_StatusTypeDef HAL_LCPU_ASSERT_INFO_get();
extern void rt_flash_enable_lock(uint8_t en);
rt_err_t save_assert_context_in_flash()
{
    rt_err_t ret = RT_EOK;
    int32_t max_size = ASSERT_CONTEXT_LEN;
    cb_backinfo_t backinfo;

    if (g_save_close == 1)
    {
        rt_kprintf("disable save assert context in addr:0x%x len:0x%x\n ", ASSERT_CONTEXT_BASE_ADDR, ASSERT_CONTEXT_LEN);
        return RT_ERROR;
    }

    if (ASSERT_CONTEXT_BASE_ADDR == 0 || ASSERT_CONTEXT_LEN <= sizeof(cb_backinfo_t))
    {
        rt_kprintf("no flash space for assert context base:0x%x len:0x%x\n ", ASSERT_CONTEXT_BASE_ADDR, ASSERT_CONTEXT_LEN);
        return RT_ERROR;
    }

    backinfo.flag = 0xaaaa;
    backinfo.block_num = 0;
    backinfo.total_len = sizeof(cb_backinfo_t);

    rt_flash_enable_lock(0);
    rt_flash_erase(ASSERT_CONTEXT_BASE_ADDR, ASSERT_CONTEXT_LEN);

#if defined(SAVE_LCPU_STATIC_CONTEXT) || defined(SAVE_LCPU_DTCM_CONTEXT)
    {
        int iCnt = 0;
        while (iCnt++ < 2000)
        {
            if (HAL_OK == HAL_LCPU_ASSERT_INFO_get())
            {
                //extern void HAL_LCPU_ASSERT_INFO_clear(void);
                //HAL_LCPU_ASSERT_INFO_clear();
                break;
            }
            HAL_Delay_us(1000);
        }
    }
#endif


#ifdef SAVE_HCPU_STATIC_CONTEXT
    ret = rt_hcpu_static_backup_debug(&backinfo);
    if (ret == RT_EFULL)
    {
        goto FUNC_END;
    }
#endif

#ifdef SAVE_HCPU_STACK_CONTEXT
    ret = rt_hcpu_stack_backup_debug(&backinfo);
    if (ret == RT_EFULL)
    {
        goto FUNC_END;
    }
#endif

#ifdef ULOG_BACKEND_USING_RAM
    ret = rt_hcpu_log_backup_debug(&backinfo);
    if (ret == RT_EFULL)
    {
        goto FUNC_END;
    }
#endif

#ifdef BSP_USING_PSRAM
    ret = rt_psram_backup_debug(&backinfo);
    if (ret == RT_EFULL)
    {
        goto FUNC_END;
    }
#endif

#ifdef SAVE_IPREG_CONTEXT
    ret = rt_register_backup_debug(&backinfo);
    if (ret == RT_EFULL)
    {
        goto FUNC_END;
    }
#endif

#ifdef SAVE_LCPU_STATIC_CONTEXT
    ret = rt_lcpu_static_backup_debug(&backinfo);
    if (ret == RT_EFULL)
    {
        goto FUNC_END;
    }
#endif

#ifdef SAVE_LCPU_DTCM_CONTEXT
    ret = rt_lcpu_dtcm_backup_debug(&backinfo);
    if (ret == RT_EFULL)
    {
        goto FUNC_END;
    }
#endif

#ifdef SAVE_HCPU_HEAP_CONTEXT
    ret = rt_hcpu_heap_backup_debug(&backinfo);
    if (ret == RT_EFULL)
    {
        goto FUNC_END;
    }
#endif


FUNC_END:
    if (backinfo.block_num != 0)
    {
        rt_flash_write(ASSERT_CONTEXT_BASE_ADDR, (uint8_t *)&backinfo, sizeof(backinfo));
    }

    return ret;
}
#else
void clear_assert()
{
}

uint32_t ble_get_tsdb_log(uint32_t *addr)
{
    return 0;
}

uint32_t ble_get_tsdb_metrics(uint32_t *addr)
{
    return 0;
}

uint32_t ble_log_get_assert(uint32_t *addr)
{
    return 0;
}

uint32_t ble_wfile_get_mem(uint32_t *addr)
{
    return 0;
}

uint32_t ble_wfile_to_mem(uint8_t *src, uint32_t addr, uint32_t len)
{
    return 0;
}

void get_assert_context_mem(uint32_t *addr, uint32_t *len)
{
}

rt_err_t save_assert_context_in_flash()
{
    return RT_EOK;
}

#endif

#endif //defined(SOC_BF0_HCPU) && defined(SAVE_ASSERT_CONTEXT_IN_FLASH)
