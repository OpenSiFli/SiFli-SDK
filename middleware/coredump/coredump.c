/*
 * SPDX-FileCopyrightText: 2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "board.h"
#include "section.h"
#ifdef ULOG_BACKEND_USING_RAM
    #include "ram_be.h"
#endif /* ULOG_BACKEND_USING_RAM */
#include "coredump.h"
#include "rtdbg.h"

#ifdef USING_MODULE_RECORD
    #define COREDUMP_RECORD_STEP(step) sifli_record_crash_save_process(step)
#else
    #define COREDUMP_RECORD_STEP(step)
#endif /* USING_MODULE_RECORD */

#if defined(__CC_ARM) || defined(__CLANG_ARM)
    extern int __stack_limit;
    extern int __initial_sp;
    #define COREDUMP_IRQ_STACK_START_ADDR    ((uint32_t)&__stack_limit)
    #define COREDUMP_IRQ_STACK_END_ADDR      ((uint32_t)&__initial_sp)
#else
    extern int __StackLimit;
    extern int __StackTop;
    #define COREDUMP_IRQ_STACK_START_ADDR    ((uint32_t)&__StackLimit)
    #define COREDUMP_IRQ_STACK_END_ADDR      ((uint32_t)&__StackTop)

#endif /* __CC_ARM || __CLANG_ARM */

#define MINLEN_LOG_SAVE  (16*1024)

#define COREDUMP_MINIDUMP_TAIL_MAGIC    0xF1EFEFEFUL
#define COREDUMP_RAMLOG_MINIMUM_MAGIC   0xEFEFEFEFUL
#define COREDUMP_RAMLOG_REMAINING_MAGIC 0xDFDFDFDFUL

/**
 * @brief Coredump block header structure
 */
typedef struct
{
    /** Address of the data block */
    uint32_t addr;
    /** Length of the data block in bytes */
    size_t len;
} coredump_block_header_t;

/**
 * @brief Coredump stack type enumeration
 */
typedef enum
{
    /** Stack in SRAM */
    COREDUMP_STACK_TYPE_SRAM = 0,
    /** Stack in PSRAM */
    COREDUMP_STACK_TYPE_PSRAM = 1,
    /** Current thread stack */
    COREDUMP_STACK_TYPE_CURR_THREAD = 2,
} coredump_stack_type_t;

/**
 * @brief Coredump state enumeration
 */
typedef enum
{
    /** Coredump is idle, ready to start */
    COREDUMP_STATE_IDLE = 0,
    /** Coredump is busy, processing in progress */
    COREDUMP_STATE_BUSY,
} coredump_state_t;

/**
 * @brief Coredump runtime information structure
 */
typedef struct
{
    /** Interrupt nesting level */
    uint8_t interrupt_nest;
    /** Main stack pointer value */
    uint32_t msp;
    /** Process stack pointer value */
    uint32_t psp;
    /** Global timer value */
    uint32_t gtimer;
} coredump_info_t;

/**
 * @brief Coredump context structure
 */
typedef struct
{
    /** Type of coredump (minimum or full) */
    coredump_type_t type;
    /** Current coredump state */
    coredump_state_t state;
    /** Pointer to fulldump backend */
    const coredump_backend_t *fulldump_backend;
    /** Pointer to minidump backend */
    const coredump_backend_t *minidump_backend;
    /** pointer to current working backend */
    const coredump_backend_t *curr_backend;
    /** Coredump enable/disable flag */
    uint8_t enabled_flag;
    /** Log position for hcpu log dump */
    int32_t log_pos;
} coredump_ctx_t;

/** Global coredump context instance */
coredump_ctx_t coredump_ctx;
/** Global coredump information instance */
coredump_info_t coredump_info;

#ifdef HPSYS_ITCM_BASE
    EXEC_REGION_DEF(ER_ITCM$$RW);
    EXEC_REGION_DEF(ER_ITCM$$ZI);
#endif /* HPSYS_ITCM_BASE */

EXEC_REGION_DEF(RW_IRAM1);
EXEC_REGION_DEF(RW_IRAM_RET$$RW);
EXEC_REGION_DEF(RW_IRAM_RET$$ZI);

extern uint32_t lcpu_ramcode_len(void);


// COREDUMP_MEMORY_REGION_REGISTER(static_data) =
// {
//     .start_addr = (uint32_t)EXEC_REGION_START_ADDR(RW_IRAM1),
//     .len = (uint32_t)HEAP_BEGIN - (uint32_t)EXEC_REGION_START_ADDR(RW_IRAM1)
// };

__WEAK const coredump_memory_region_t coredump_memory_region_list[] =
{
    {(uint32_t)EXEC_REGION_START_ADDR(RW_IRAM1), (uint32_t)HEAP_BEGIN},
#if defined(HPSYS_ITCM_BASE)
    {(uint32_t)EXEC_REGION_START_ADDR(ER_ITCM$$RW), (uint32_t)EXEC_REGION_END_ADDR(ER_ITCM$$ZI)},
#endif /* HPSYS_ITCM_BASE */
    {(uint32_t)EXEC_REGION_START_ADDR(RW_IRAM_RET$$RW), (uint32_t)EXEC_REGION_END_ADDR(RW_IRAM_RET$$ZI)},
    {0, 0}
};

__WEAK const coredump_register_region_t coredump_register_list[] =
{
    {LCDC1_BASE, sizeof(LCD_IF_TypeDef)},
#ifdef DSI_HOST_BASE
    {DSI_HOST_BASE, sizeof(DSI_HOST_TypeDef)},
    {DSI_PHY_BASE, sizeof(DSI_PHY_TypeDef)},
#endif /* DSI_HOST_BASE */
#ifdef EZIP_BASE
    {EZIP_BASE, sizeof(EZIP_TypeDef)},
#else
    {EZIP1_BASE, sizeof(EZIP_TypeDef)},
#endif /* EZIP_BASE */
    {EPIC_BASE, sizeof(EPIC_TypeDef)},
    {GPIO1_BASE, sizeof(GPIO1_TypeDef)},
    {GPIO2_BASE, sizeof(GPIO2_TypeDef)},
    {PINMUX1_BASE, sizeof(HPSYS_PINMUX_TypeDef)},
    {PINMUX2_BASE, sizeof(LPSYS_PINMUX_TypeDef)},
    {HPSYS_AON_BASE, sizeof(HPSYS_AON_TypeDef)},
    {LPSYS_AON_BASE, sizeof(LPSYS_AON_TypeDef)},
#ifdef BLE_RFC_BASE
    {BLE_RFC_BASE, sizeof(BLE_RF_DIG_TypeDef)},
    {BLE_MAC_BASE, sizeof(BLE_MAC_TypeDef)},
#else
    {BT_RFC_REG_BASE, sizeof(BT_RFC_TypeDef)},
    {BT_MAC_BASE, sizeof(BT_MAC_TypeDef)},
#endif /* BLE_RFC_BASE */
    {SysTick_BASE, sizeof(SysTick_Type)},
    {SCB_BASE, sizeof(SCB_Type)},
    {HPSYS_RCC_BASE, sizeof(HPSYS_RCC_TypeDef)},
#ifdef MPI1_BASE
    {MPI1_BASE, sizeof(MPI_TypeDef)},
    {MPI2_BASE, sizeof(MPI_TypeDef)},
#endif /* MPI1_BASE */
    {PMUC_BASE, sizeof(PMUC_TypeDef)},
    {HPSYS_CFG_BASE, sizeof(HPSYS_CFG_TypeDef)},
    {EXTDMA_BASE, sizeof(EXTDMA_TypeDef)},
    {NVIC_BASE, sizeof(NVIC_Type)},
    {0, 0}
};

static void coredump_sync(void)
{
    if (coredump_ctx.curr_backend && coredump_ctx.curr_backend->sync)
    {
        coredump_ctx.curr_backend->sync();
    }
}

static int32_t coredump_query(coredump_query_id_t id, void *arg)
{
    return coredump_ctx.curr_backend->query(id, arg);
}

static coredump_err_code_t coredump_start(void)
{
    return coredump_ctx.curr_backend->start();
}

static void coredump_end(void)
{
    coredump_ctx.curr_backend->end();
}

static coredump_err_code_t coredump_set_mode(coredump_type_t coredump_type)
{
    if (coredump_type == COREDUMP_TYPE_FULL)
    {
        coredump_ctx.curr_backend = coredump_ctx.fulldump_backend;
    }
    else
    {
#ifdef COREDUMP_MINIDUMP_ENABLED
        coredump_ctx.curr_backend = coredump_ctx.minidump_backend;
#else
        return COREDUMP_ERR_INVALID_PARAM;
#endif /* COREDUMP_MINIDUMP_ENABLED */
    }

    return coredump_ctx.curr_backend->set_mode(coredump_type);
}


size_t coredump_raw_write(uint8_t *buf, size_t len)
{
    return coredump_ctx.curr_backend->write(buf, len);
}

void coredump_block_prepare(uint32_t addr, size_t len, coredump_block_header_t *block_header)
{
    int32_t remain_size;

    block_header->addr = addr;
    block_header->len = len;
    remain_size = coredump_ctx.curr_backend->query(COREDUMP_QUERY_REMAIN_SIZE, NULL);
    if ((remain_size < 0) || ((size_t)remain_size <= sizeof(*block_header)))
    {
        block_header->len = 0;
        return;
    }

    if ((len + sizeof(*block_header)) > (size_t)remain_size)
    {
        block_header->len = (size_t)remain_size - sizeof(*block_header);
    }

}

size_t coredump_block_write(uint32_t addr, size_t len)
{
    coredump_block_header_t block_header;

    coredump_block_prepare(addr, len, &block_header);
    if (block_header.len > 0)
    {
        coredump_raw_write((uint8_t *)&block_header, sizeof(block_header));
        coredump_raw_write((uint8_t *)block_header.addr, block_header.len);
    }

    return block_header.len;
}

size_t coredump_block_write_copy(uint32_t addr, size_t len)
{
    coredump_block_header_t block_header;
    uint32_t i;
    uint8_t buf[64];

    coredump_block_prepare(addr, len, &block_header);
    if (block_header.len > 0)
    {
        coredump_raw_write((uint8_t *)&block_header, sizeof(block_header));

        for (i = 0; i < block_header.len / sizeof(buf); i++)
        {
            memcpy((void *)buf, (void *)(block_header.addr + i * sizeof(buf)), sizeof(buf));
            coredump_raw_write(buf, sizeof(buf));
        }

        if (block_header.len % sizeof(buf))
        {
            memcpy((void *)buf, (void *)(block_header.addr + i * sizeof(buf)), block_header.len % sizeof(buf));
            coredump_raw_write(buf, block_header.len % sizeof(buf));
        }
    }

    return block_header.len;
}

static rt_uint32_t coredump_dump_callback(rt_uint32_t addr, rt_uint32_t size, void *user_data, rt_uint32_t *actual_size)
{
    size_t dump_size;

    dump_size = coredump_block_write(addr, size);
    if (actual_size)
    {
        *actual_size = dump_size + sizeof(coredump_block_header_t);
    }

    return dump_size;
}

static void coredump_write_header(void)
{
#define HEADER_STR_SIZE 24
#define HEADER_FORMAT_STR "%04d_%02d_%02d %02d:%02d:%02d"
    char header_str[HEADER_STR_SIZE] = {0};
    time_t now;
    struct tm *p_tm;
    struct tm tm_tmp;
    const char *fmt_str;

    now = time(RT_NULL);
    p_tm = localtime(&now);
    if (!p_tm)
    {
        p_tm = &tm_tmp;
        memset(p_tm, 0, sizeof(tm_tmp));
    }

#if defined(SOC_SF32LB55X)
    fmt_str = "55X_" HEADER_FORMAT_STR;
#elif defined(SOC_SF32LB56X)
    fmt_str = "56X_" HEADER_FORMAT_STR;
#elif defined(SOC_SF32LB58X)
    fmt_str = "58X_" HEADER_FORMAT_STR;
#elif defined(SOC_SF32LB52X)
    fmt_str = "52X_" HEADER_FORMAT_STR;
#else
#error "Unknown SOC, please check!"
#endif

    snprintf(header_str, sizeof(header_str), fmt_str,
             p_tm->tm_year + 1900, p_tm->tm_mon + 1, p_tm->tm_mday, p_tm->tm_hour, p_tm->tm_min, p_tm->tm_sec);

    coredump_raw_write((uint8_t *)header_str, sizeof(header_str));

    return;
}

static rt_err_t coredump_write_tail(void)
{
    rt_err_t ret = RT_EOK;
    coredump_block_header_t block_header;
    int32_t remain_size;

    remain_size = coredump_query(COREDUMP_QUERY_REMAIN_SIZE, NULL);
    if ((remain_size < 0) || ((size_t)remain_size < sizeof(block_header)))
    {
        return RT_EFULL;
    }

    block_header.addr = COREDUMP_MINIDUMP_TAIL_MAGIC;
    block_header.len = (size_t)remain_size - sizeof(block_header);
    coredump_raw_write((uint8_t *)&block_header, sizeof(block_header));
    return RT_EOK;
}

static void coredump_dump_memory_region_list(void)
{
    const coredump_memory_region_t *region = &coredump_memory_region_list[0];
    size_t len;

    while (1)
    {
        if ((0 == region->start_addr) || (0 == region->end_addr))
        {
            break;
        }
        if (region->end_addr > region->start_addr)
        {
            len = region->end_addr - region->start_addr;
            if (coredump_block_write(region->start_addr, len) < len)
            {
                break;
            }
        }
        region++;
    }
}

#ifdef COREDUMP_LCPU_SRAM_DATA_ENABLED
static rt_err_t coredump_dump_lcpu_sram_data(void)
{
    uint32_t start_addr;
    size_t len;

    /* dump data in SRAM */
#if defined(SOC_SF32LB52X)
    start_addr = LCPU_ADDR_2_HCPU_ADDR(LPSYS_SRAM_BASE);
    len = LPSYS_SRAM_TOTAL_SIZE;
#else
    len = lcpu_ramcode_len();
    if (len <= LPSYS_ITCM_SIZE)
    {
        /* all code is in ITCM, data is available from the beginning of SRAM */
        start_addr = LCPU_ADDR_2_HCPU_ADDR(LPSYS_SRAM_BASE);
        len = LPSYS_SRAM_TOTAL_SIZE;
    }
    else
    {
        start_addr = LCPU_ADDR_2_HCPU_ADDR(LPSYS_SRAM_BASE) + (len - LPSYS_ITCM_SIZE);
        len = LPSYS_SRAM_TOTAL_SIZE - (len - LPSYS_ITCM_SIZE);
    }
#endif

    if (coredump_block_write(start_addr, len) < len)
    {
        return RT_EFULL;
    }

    return RT_EOK;
}
#endif /* COREDUMP_LCPU_SRAM_DATA_ENABLED */

#ifdef COREDUMP_LCPU_TCM_DATA_ENABLED
static rt_err_t coredump_dump_lcpu_tcm_data(void)
{
    uint32_t start_addr;
    size_t len;

#ifdef LPSYS_DTCM_BASE
    start_addr = LCPU_DTCM_ADDR_2_HCPU_ADDR(LPSYS_DTCM_BASE);
    len = LPSYS_DTCM_SIZE;

    if (coredump_block_write(start_addr, len) < len)
    {
        return RT_EFULL;
    }
#endif /* LPSYS_DTCM_BASE */

#ifdef LPSYS_ITCM_BASE
    len = lcpu_ramcode_len();

    if (len < LPSYS_ITCM_SIZE)
    {
        start_addr = LCPU_ITCM_ADDR_2_HCPU_ADDR(LPSYS_ITCM_BASE) + len;
        len = LPSYS_ITCM_SIZE - len;
    }
    else
    {
        /* all ITCM is used to store code, no data present */
        len = 0;
    }

    if ((len > 0) && (coredump_block_write(start_addr, len) < len))
    {
        return RT_EFULL;
    }
#endif /* LPSYS_ITCM_BASE */

    return RT_EOK;
}
#endif /* COREDUMP_LCPU_TCM_DATA_ENABLED */


#ifdef COREDUMP_REG_DATA_ENABLED

static rt_err_t coredump_dump_register_list(void)
{
    rt_err_t ret = RT_EOK;
    const coredump_register_region_t *region = &coredump_register_list[0];

    while (1)
    {
        if (0 == region->start_addr)
        {
            break;
        }
        if (region->len > 0)
        {
            if (coredump_block_write_copy(region->start_addr, region->len) < region->len)
            {
                ret = RT_EFULL;
                break;
            }
        }
        region++;
    }

    return ret;
}
#endif  /* COREDUMP_REG_DATA_ENABLED */

#ifdef COREDUMP_HCPU_HEAP_DATA_ENABLED
static rt_err_t coredump_dump_all_heaps(bool sram_heap)
{
    rt_err_t ret = RT_EOK;
#ifdef RT_USING_SMALL_MEM
#ifndef RT_USING_MEMHEAP_AS_HEAP
    uint32_t addr;
    do
    {
        addr = rt_mem_base();
        if (sram_heap && (!HCPU_IS_SRAM_ADDR(addr)))
        {
            break;
        }

        if ((!sram_heap) && HCPU_IS_SRAM_ADDR(addr))
        {
            break;
        }

        if (RT_EFULL == rt_mem_dump(coredump_dump_callback, NULL, NULL, NULL))
        {
            ret = RT_EFULL;
            return ret;
        }
    }
    while (0);
#endif /* RT_USING_MEMHEAP_AS_HEAP */
#endif /* RT_USING_SMALL_MEM */

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

        if (sram_heap && (!HCPU_IS_SRAM_ADDR((uint32_t)mh->start_addr)))
        {
            continue;
        }

        if ((!sram_heap) && HCPU_IS_SRAM_ADDR((uint32_t)mh->start_addr))
        {
            continue;
        }

        ret = rt_memheap_dump(mh, coredump_dump_callback, NULL, NULL, NULL);
        if (RT_EFULL == ret)
        {
            return ret;
        }
    }
#endif /* RT_USING_MEMHEAP */
    return ret;
}
#endif /* COREDUMP_HCPU_HEAP_DATA_ENABLED */

#if defined(COREDUMP_HCPU_STACK_DATA_ENABLED) || defined(COREDUMP_MINIDUMP_ENABLED)
static rt_err_t coredump_dump_irq_stack(void)
{
    uint32_t addr;
    size_t len;

    addr = COREDUMP_IRQ_STACK_START_ADDR;
    len = COREDUMP_IRQ_STACK_END_ADDR - COREDUMP_IRQ_STACK_START_ADDR;

    if (coredump_block_write(addr, len) < len)
    {
        return RT_EFULL;
    }
    coredump_sync();
    return RT_EOK;
}

static void coredump_dump_curr_thread_stack(void)
{
    struct rt_thread *curr_thread;

    curr_thread = rt_thread_self();
    if (curr_thread)
    {
        coredump_block_write((uint32_t)curr_thread->stack_addr, curr_thread->stack_size);
    }
}
#endif /* COREDUMP_HCPU_STACK_DATA_ENABLED || COREDUMP_MINIDUMP_ENABLED */

#ifdef COREDUMP_HCPU_STACK_DATA_ENABLED
static uint8_t is_psp_thread_range(rt_uint32_t psp, struct rt_thread *thread)
{
    if ((psp >= (rt_uint32_t)thread->stack_addr) &&
            (psp < (rt_uint32_t)thread->stack_addr + thread->stack_size))
    {
        return 1;
    }
    return 0;
}

static rt_err_t coredump_dump_thread_stack(coredump_stack_type_t stack_type)
{
    struct rt_object_information *info;
    struct rt_thread *thread;
    struct rt_list_node *node;
    struct rt_list_node *thread_list;
    rt_uint32_t psp;
    uint32_t start_addr;
    size_t len;
    rt_err_t ret = RT_EOK;
    struct rt_thread *curr_thread;

    curr_thread = rt_thread_self();

    info = rt_object_get_information(RT_Object_Class_Thread);
    thread_list = &info->object_list;
    psp = __get_PSP();
    for (node = thread_list->next; node != thread_list; node = node->next)
    {
        thread = rt_list_entry(node, struct rt_thread, list);

        if ((COREDUMP_STACK_TYPE_CURR_THREAD == stack_type)
                && ((thread != curr_thread) && !is_psp_thread_range(psp, thread)))
        {
            continue;
        }

        if ((COREDUMP_STACK_TYPE_PSRAM == stack_type) && HCPU_IS_SRAM_ADDR((uint32_t)thread->stack_addr))
        {
            continue;
        }

        if ((COREDUMP_STACK_TYPE_SRAM == stack_type) && !HCPU_IS_SRAM_ADDR((uint32_t)thread->stack_addr))
        {
            continue;
        }

        if (is_psp_thread_range(psp, thread) || (thread == rt_thread_self()))
        {
            /* PSP is current thread, use the actual stack top */
            start_addr = (uint32_t)thread->stack_addr ;
            len = thread->stack_size;
        }
        else
        {
            /* PSP is not current thread, stack top is saved in thread->sp */
            start_addr = (uint32_t)thread->sp;

            if ((uint32_t)thread->sp >= (uint32_t)thread->stack_addr)
            {
                /* including 4 bytes pointed by SP */
                len = (uint32_t)thread->sp - (rt_uint32_t)thread->stack_addr;
                len = thread->stack_size - len;
            }
            else
            {
                /* SP is overflowed, dump whole stack */
                start_addr = (uint32_t)thread->stack_addr;
                len = thread->stack_size;
            }
        }

        if (len > coredump_block_write(start_addr, len))
        {
            ret = RT_EFULL;
            return ret;
        }
    }

    coredump_sync();
    return RT_EOK;
}
#endif  //COREDUMP_HCPU_STACK_DATA_ENABLED

#ifdef ULOG_BACKEND_USING_RAM
static void coredump_dump_hcpu_log_minimum(void)
{
    ulog_ram_be_buf_t *log_buf = ulog_ram_be_buf_get(RT_NULL);
    coredump_block_header_t block_header_loop;
    coredump_block_header_t block_header;
    coredump_block_header_t block_header_special;
    int32_t remain_size;
    size_t wr_len = MINLEN_LOG_SAVE;

    remain_size = coredump_query(COREDUMP_QUERY_REMAIN_SIZE, NULL);
    if ((remain_size < 0) || ((size_t)remain_size <= sizeof(block_header)))
    {
        return;
    }
    if ((wr_len + sizeof(block_header)) >= (size_t)remain_size)
    {
        wr_len = (size_t)remain_size - sizeof(block_header);
    }

    block_header_loop.len = 0;
    if (log_buf->wr_offset > wr_len)
    {
        block_header.len = wr_len;
        block_header.addr = (uint32_t)&log_buf->buf[log_buf->wr_offset - block_header.len];
        coredump_ctx.log_pos = log_buf->wr_offset - block_header.len;
    }
    else
    {
        coredump_ctx.log_pos = -1;
        if (log_buf->full)
        {
            if (ULOG_RAM_BE_BUF_SIZE > wr_len)
            {
                block_header_loop.len = wr_len - log_buf->wr_offset;
                block_header_loop.addr = (uint32_t)&log_buf->buf[ULOG_RAM_BE_BUF_SIZE - block_header_loop.len];
                coredump_ctx.log_pos = ULOG_RAM_BE_BUF_SIZE - block_header_loop.len;
            }
            else
            {
                block_header_loop.len = ULOG_RAM_BE_BUF_SIZE - log_buf->wr_offset;
                block_header_loop.addr = (uint32_t)&log_buf->buf[log_buf->wr_offset];

            }
        }

        block_header.addr = (uint32_t)&log_buf->buf[0];
        block_header.len = log_buf->wr_offset;

    }

    if (block_header.len > 0)
    {
        block_header_special.addr = COREDUMP_RAMLOG_MINIMUM_MAGIC;
        block_header_special.len = block_header_loop.len + block_header.len;

        coredump_raw_write((uint8_t *)&block_header_special, sizeof(block_header_special));
        if (block_header_loop.len > 0)
        {
            coredump_raw_write((uint8_t *)block_header_loop.addr, block_header_loop.len);
        }
        coredump_raw_write((uint8_t *)block_header.addr, block_header.len);
    }
}

rt_err_t coredump_dump_hcpu_log_remaining(void)
{
    rt_err_t ret = RT_EOK;
    ulog_ram_be_buf_t *log_buf = ulog_ram_be_buf_get(RT_NULL);
    coredump_block_header_t block_header_loop;
    coredump_block_header_t block_header;
    coredump_block_header_t block_header_special;
    int32_t remain_size;

    if (coredump_ctx.log_pos < 0) //save over in step1
    {
        return ret;
    }

    remain_size = coredump_query(COREDUMP_QUERY_REMAIN_SIZE, NULL);
    if ((remain_size < 0) || ((size_t)remain_size <= sizeof(block_header)))
    {
        return RT_EFULL;
    }
    block_header_loop.len = 0;
    if (coredump_ctx.log_pos >= log_buf->wr_offset)
    {
        if ((size_t)coredump_ctx.log_pos - log_buf->wr_offset + sizeof(block_header) > (size_t)remain_size)
        {
            block_header.len = (size_t)remain_size - sizeof(block_header);
            block_header.addr = (uint32_t)&log_buf->buf[coredump_ctx.log_pos - block_header.len];
            ret = RT_EFULL;
        }
        else
        {
            block_header.len = (size_t)coredump_ctx.log_pos - log_buf->wr_offset;
            block_header.addr = (uint32_t)&log_buf->buf[log_buf->wr_offset];
        }
    }
    else
    {
        if ((size_t)coredump_ctx.log_pos + sizeof(block_header) > (size_t)remain_size)
        {
            block_header.len = (size_t)remain_size - sizeof(block_header);
            block_header.addr = (uint32_t)&log_buf->buf[coredump_ctx.log_pos - block_header.len];
            ret = RT_EFULL;
        }
        else
        {
            if (log_buf->full)
            {
                if (ULOG_RAM_BE_BUF_SIZE - (log_buf->wr_offset - coredump_ctx.log_pos) + sizeof(block_header) > (size_t)remain_size)
                {
                    block_header_loop.len = (size_t)remain_size - (sizeof(block_header) + (size_t)coredump_ctx.log_pos);
                    block_header_loop.addr = (uint32_t)&log_buf->buf[ULOG_RAM_BE_BUF_SIZE - block_header_loop.len];
                    ret = RT_EFULL;
                }
                else
                {
                    block_header_loop.len = ULOG_RAM_BE_BUF_SIZE - log_buf->wr_offset;
                    block_header_loop.addr = (uint32_t)&log_buf->buf[log_buf->wr_offset];

                }
            }

            block_header.addr = (uint32_t)&log_buf->buf[0];
            block_header.len = coredump_ctx.log_pos;
        }
    }

    if (block_header.len > 0)
    {

        block_header_special.addr = COREDUMP_RAMLOG_REMAINING_MAGIC;
        block_header_special.len = block_header_loop.len + block_header.len;
        coredump_raw_write((uint8_t *)&block_header_special, sizeof(block_header_special));

        if (block_header_loop.len > 0)
        {
            coredump_raw_write((uint8_t *)block_header_loop.addr, block_header_loop.len);
        }
        coredump_raw_write((uint8_t *)block_header.addr, block_header.len);
    }

    return ret;
}

#endif  /* ULOG_BACKEND_USING_RAM */

static void coredump_info_update(void)
{
    if (!coredump_info.msp)
    {
        coredump_info.msp = __get_MSP();
        coredump_info.psp = __get_PSP();
        coredump_info.gtimer = HAL_GTIMER_READ();
    }
}

static rt_err_t coredump_full(void)
{
    rt_err_t ret = RT_EOK;
    coredump_err_code_t err;

    if (coredump_ctx.enabled_flag == 0)
    {
        return RT_ERROR;
    }

    err = coredump_start();
    if (err != COREDUMP_ERR_NO)
    {
        return RT_ERROR;
    }
#if !defined(COREDUMP_MINIDUMP_ENABLED)
    coredump_write_header();
    coredump_sync();
    rt_coredump_info_dump(coredump_dump_callback);
    coredump_block_write((uint32_t)&coredump_info, sizeof(coredump_info));
    coredump_sync();
#endif /* !COREDUMP_MINIDUMP_ENABLED */

#ifdef COREDUMP_HCPU_STATIC_DATA_ENABLED
    COREDUMP_RECORD_STEP(RECORD_CRASH_SAVE_STATIC);
    coredump_dump_memory_region_list();
    coredump_sync();
#endif /* COREDUMP_HCPU_STATIC_DATA_ENABLED */

#ifdef COREDUMP_HCPU_STACK_DATA_ENABLED
    COREDUMP_RECORD_STEP(RECORD_CRASH_SAVE_STACK);
    ret = coredump_dump_thread_stack(COREDUMP_STACK_TYPE_SRAM);
    if (ret == RT_EFULL)
    {
        goto __EXIT;
    }

#if !defined(COREDUMP_MINIDUMP_ENABLED)
    ret = coredump_dump_irq_stack();
    if (ret == RT_EFULL)
    {
        goto __EXIT;
    }
#endif /* COREDUMP_MINIDUMP_ENABLED */
    coredump_sync();
#endif

#ifdef ULOG_BACKEND_USING_RAM
    COREDUMP_RECORD_STEP(RECORD_CRASH_SAVE_LOG);
#if !defined(COREDUMP_MINIDUMP_ENABLED)
    coredump_dump_hcpu_log_minimum();
#endif
    coredump_sync();
#endif /* ULOG_BACKEND_USING_RAM */

#ifdef COREDUMP_REG_DATA_ENABLED
    COREDUMP_RECORD_STEP(RECORD_CRASH_SAVE_REGISTER);
    ret = coredump_dump_register_list();
    if (ret == RT_EFULL)
    {
        goto __EXIT;
    }
    coredump_sync();
#endif /* COREDUMP_REG_DATA_ENABLED */

#if defined(COREDUMP_LCPU_SRAM_DATA_ENABLED) || defined(COREDUMP_LCPU_TCM_DATA_ENABLED)
    {
        uint32_t i = 0;
        /* wait until LCPU stop for data dump */
        while (i++ < 2000)
        {
            if (HAL_OK == HAL_LCPU_ASSERT_INFO_get())
            {
                break;
            }
            HAL_Delay_us_(1000);
        }
    }
#endif /* COREDUMP_LCPU_SRAM_DATA_ENABLED || COREDUMP_LCPU_TCM_DATA_ENABLED */

#ifdef COREDUMP_LCPU_SRAM_DATA_ENABLED
    COREDUMP_RECORD_STEP(RECORD_CRASH_SAVE_LCPU_STATIC);
    ret = coredump_dump_lcpu_sram_data();
    if (ret == RT_EFULL)
    {
        goto __EXIT;
    }
    coredump_sync();
#endif /* COREDUMP_LCPU_SRAM_DATA_ENABLED */

#ifdef COREDUMP_LCPU_TCM_DATA_ENABLED
    COREDUMP_RECORD_STEP(RECORD_CRASH_SAVE_LCPU_DTCM);
    ret = coredump_dump_lcpu_tcm_data();
    if (ret == RT_EFULL)
    {
        goto __EXIT;
    }
    coredump_sync();
#endif /* COREDUMP_LCPU_TCM_DATA_ENABLED */

#ifdef COREDUMP_HCPU_HEAP_DATA_ENABLED
    COREDUMP_RECORD_STEP(RECORD_CRASH_SAVE_HCPU_HEAP);
    ret = coredump_dump_all_heaps(true);
    if (ret == RT_EFULL)
    {
        goto __EXIT;
    }
    coredump_sync();
#endif /* COREDUMP_HCPU_HEAP_DATA_ENABLED */

#ifdef BSP_USING_PSRAM
#ifdef COREDUMP_HCPU_STACK_DATA_ENABLED
    COREDUMP_RECORD_STEP(RECORD_CRASH_SAVE_STACK_1);
    ret = coredump_dump_thread_stack(COREDUMP_STACK_TYPE_PSRAM);
    if (ret == RT_EFULL)
    {
        goto __EXIT;
    }
    coredump_sync();
#endif /* COREDUMP_HCPU_STACK_DATA_ENABLED */

#ifdef COREDUMP_HCPU_HEAP_DATA_ENABLED
    COREDUMP_RECORD_STEP(RECORD_CRASH_SAVE_HCPU_HEAP_1);
    ret = coredump_dump_all_heaps(false);
    if (ret == RT_EFULL)
    {
        goto __EXIT;
    }
    coredump_sync();
#endif /* COREDUMP_HCPU_HEAP_DATA_ENABLED */
#endif /* BSP_USING_PSRAM */

#ifdef ULOG_BACKEND_USING_RAM

#if defined(USING_MODULE_RECORD)
    sifli_record_crash_save_process(RECORD_CRASH_SAVE_LOG_1);
#endif

    ret = coredump_dump_hcpu_log_remaining();
    if (ret == RT_EFULL)
    {
        goto __EXIT;
    }
    coredump_sync();
#endif /* ULOG_BACKEND_USING_RAM */

__EXIT:

    coredump_end();

    return ret;
}

void coredump(void)
{
    coredump_err_code_t r;
    if (COREDUMP_STATE_IDLE != coredump_ctx.state)
    {
        COREDUMP_RECORD_STEP(RECORD_CRASH_SAVE_ERR);
        goto __EXIT;
    }

    coredump_ctx.type = COREDUMP_TYPE_FULL;
    coredump_ctx.state = COREDUMP_STATE_BUSY;
    r = coredump_set_mode(COREDUMP_TYPE_FULL);
    if (r != COREDUMP_ERR_NO)
    {
        COREDUMP_RECORD_STEP(RECORD_CRASH_SAVE_ERR);
        goto __EXIT;
    }
    coredump_info_update();

#if defined(USING_MODULE_RECORD)
    sifli_record_crash_status(1);
    sifli_record_crash_save_process(RECORD_CRASH_SAVE_START);
#endif
    log_pause(1);

#if defined(SOC_SF32LB52X)
    HAL_HPAON_WakeCore(CORE_ID_LCPU);
#endif
    if (rt_interrupt_get_nest())
    {
        coredump_info.interrupt_nest = rt_interrupt_get_nest();
        rt_interrupt_nest_clear();
    }

    coredump_full();

    HAL_LCPU_ASSERT_INFO_clear();

#ifdef USING_COREDUMP_EXT
    /* call user-defined core dump function */
    coredump_ext();
#endif /* USING_COREDUMP_EXT */

#ifdef MC_BACKEND_USING_FILE
    mc_close();
#endif

    log_pause(0);

    COREDUMP_RECORD_STEP(RECORD_CRASH_SAVE_END);

__EXIT:
    coredump_ctx.state = COREDUMP_STATE_IDLE;

#ifdef RT_USING_WDT
    if (rt_hw_watchdog_get_status())
    {
        drv_reboot();
    }
#endif /* RT_USING_WDT */
}

#ifdef COREDUMP_MINIDUMP_ENABLED
void coredump_minimum(void)
{
    coredump_err_code_t r;

    if (COREDUMP_STATE_IDLE != coredump_ctx.state)
    {
        COREDUMP_RECORD_STEP(RECORD_CRASH_SAVE_ERR);
        return;
    }

    coredump_ctx.type = COREDUMP_TYPE_MINIMUM;
    coredump_ctx.state = COREDUMP_STATE_BUSY;
    r = coredump_set_mode(COREDUMP_TYPE_MINIMUM);
    if (r != COREDUMP_ERR_NO)
    {
        COREDUMP_RECORD_STEP(RECORD_CRASH_SAVE_ERR);
        coredump_ctx.state = COREDUMP_STATE_IDLE;
        return;
    }
    coredump_info_update();

    r = coredump_start();
    if (r != COREDUMP_ERR_NO)
    {
        COREDUMP_RECORD_STEP(RECORD_CRASH_SAVE_ERR);
        coredump_ctx.state = COREDUMP_STATE_IDLE;
        return;
    }

    coredump_write_header();

    rt_coredump_info_dump(coredump_dump_callback);

    coredump_block_write((uint32_t)&coredump_info, sizeof(coredump_info));

    coredump_dump_irq_stack();

#ifdef ULOG_BACKEND_USING_RAM
    coredump_dump_hcpu_log_minimum();
#endif /* ULOG_BACKEND_USING_RAM */

    coredump_dump_curr_thread_stack();

    coredump_write_tail();

    coredump_end();

    coredump_ctx.state = COREDUMP_STATE_IDLE;
}
#endif /* COREDUMP_MINIDUMP_ENABLED */

static int coredump_config(int argc, char **argv)
{
    uint32_t addr;
    size_t max_size;

    if (argc >= 2)
    {
        if (argv[1][0] == '0')
        {
            coredump_ctx.enabled_flag = 0;
        }
        else if (argv[1][0] == '1')
        {
            coredump_ctx.enabled_flag = 1;
        }
        else if (argv[1][0] == '2')
        {
            addr = (uint32_t)coredump_query(COREDUMP_QUERY_PATH, NULL);
            max_size = (size_t)coredump_query(COREDUMP_QUERY_MAX_SIZE, NULL);
            rt_kprintf("g_save_assert:%d g_assertmem_addr:0x%x g_default_context.write_max_size:%u\n",
                       coredump_ctx.enabled_flag, addr, max_size);
        }
    }

    return 0;
}
MSH_CMD_EXPORT(coredump_config, coredump_config 1 enable or coredump_config 0 disable or coredump_config 2 get flag);

int coredump_set_onoff(int flag)
{
    if (flag == 2)
    {
        return coredump_ctx.enabled_flag;
    }
    else
    {
        coredump_ctx.enabled_flag = flag;
    }

    return 0;
}

int coredump_clear(void)
{
    RT_ASSERT(coredump_ctx.curr_backend);

    coredump_ctx.curr_backend->clear();

    return 0;
}

int coredump_get_type(void)
{
    if (coredump_ctx.curr_backend == &coredump_backend_partition)
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

coredump_err_code_t coredump_get_data(coredump_data_t *data)
{
    int32_t r;

    RT_ASSERT(data);

#ifdef COREDUMP_BACKEND_FILE
    data->is_file = true;
#else
    data->is_file = false;
#endif /* COREDUMP_BACKEND_FILE */

#ifdef COREDUMP_MINIDUMP_ENABLED
    r = coredump_set_mode(COREDUMP_TYPE_MINIMUM);
    RT_ASSERT(COREDUMP_ERR_NO == r);
    r = coredump_query(COREDUMP_QUERY_DATA_PRESENT, NULL);
    if (r > 0)
    {
        data->minidump_size = r;
        r = coredump_query(COREDUMP_QUERY_PATH, NULL);
        RT_ASSERT(r);
        data->minidump_addr = (uint32_t)r;
    }
    else
    {
        data->minidump_addr = (uint32_t)NULL;
        data->minidump_size = 0;
    }
#else
    data->minidump_addr = (uint32_t)NULL;
    data->minidump_size = 0;
#endif /* COREDUMP_MINIDUMP_ENABLED */


    r = coredump_set_mode(COREDUMP_TYPE_FULL);
    RT_ASSERT(COREDUMP_ERR_NO == r);
#ifndef COREDUMP_MINIDUMP_ENABLED
    r = coredump_query(COREDUMP_QUERY_DATA_PRESENT, NULL);

#else
    r = coredump_query(COREDUMP_QUERY_MAX_SIZE, NULL);
#endif
    if (r > 0)
    {
        data->fulldump_size = r;
        r = coredump_query(COREDUMP_QUERY_PATH, NULL);
        RT_ASSERT(r);
        data->fulldump_addr = (uint32_t)r;
    }
    else
    {
        data->fulldump_addr = (uint32_t)NULL;
        data->fulldump_size = 0;
    }

    return COREDUMP_ERR_NO;
}

size_t coredump_read_dump(uint32_t offset, uint8_t *buf, size_t len)
{
    const coredump_backend_t *backend = coredump_ctx.curr_backend;

    RT_ASSERT(backend);

    if (!backend->read)
    {
        return 0;
    }

    return backend->read(offset, buf, len);
}


size_t coredump_read_minidump(uint32_t offset, uint8_t *buf, size_t len)
{
#ifdef COREDUMP_MINIDUMP_ENABLED
    const coredump_backend_t *backend = coredump_ctx.minidump_backend;
    coredump_err_code_t err;
    size_t rd_size;

    RT_ASSERT(backend && backend->set_mode && backend->read);

    err = backend->set_mode(COREDUMP_TYPE_MINIMUM);
    RT_ASSERT(COREDUMP_ERR_NO == err);

    rd_size = backend->read(offset, buf, len);

    if (backend == coredump_ctx.fulldump_backend)
    {
        err = backend->set_mode(COREDUMP_TYPE_FULL);
        RT_ASSERT(COREDUMP_ERR_NO == err);
    }

    return rd_size;

#else
    return 0;
#endif /* COREDUMP_MINIDUMP_ENABLED */


}


static int coredump_init(void)
{
    coredump_err_code_t r;

#ifdef COREDUMP_MINIDUMP_ENABLED
    coredump_ctx.minidump_backend = &coredump_backend_partition;
    r = coredump_ctx.minidump_backend->init(COREDUMP_TYPE_MINIMUM);
    if (COREDUMP_ERR_NO != r)
    {
        rt_kprintf("Coredump backend init failed for minimum dump, err code: %d\n", r);
        RT_ASSERT(0);
    }
#endif /* COREDUMP_MINIDUMP_ENABLED */

#ifdef COREDUMP_BACKEND_PARTITION
    coredump_ctx.fulldump_backend = &coredump_backend_partition;
#elif defined(COREDUMP_BACKEND_FILE)
    coredump_ctx.fulldump_backend = &coredump_backend_file;
#else
#error "Please select a backend for core dump"
#endif /* COREDUMP_BACKEND_PARTITION */

    r = coredump_ctx.fulldump_backend->init(COREDUMP_TYPE_FULL);
    if (COREDUMP_ERR_NO != r)
    {
        rt_kprintf("Coredump backend init failed for full dump, err code: %d\n", r);
        RT_ASSERT(0);
    }

    coredump_set_mode(COREDUMP_TYPE_FULL);

    coredump_ctx.enabled_flag = 1;

    return 0;
}
INIT_PRE_APP_EXPORT(coredump_init);