/**
 ****************************************************************************************
 *
 * @file dbg_trc_mem.c
 *
 * @brief Trace memory
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup TRACER
 * @{
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"  // SW configuration

#if (TRACER_PRESENT)
#include <string.h>         // string definitions
#include "dbg_trc.h"        // tracer definitions
#include "dbg_trc_int.h"    // tracer internal definitions
#include "rwip.h"
#include "co_utils.h"
#include "arch.h"            // architectural definition
#include "rom_config.h"

/*
 * DEFINES
 ****************************************************************************************
 */
#define TRACE_MEM_SIZE                  4096

/// Patterns used to mark memory blocks as valid or invalid
__STATIC const uint8_t valid_pattern      = 0xAA; //10101010
__STATIC const uint8_t invalid_pattern    = 0x55; //01010101

typedef uint16_t trc_size_t;

#define TRACE_BLOCK_SIZE(trace_len) \
        sizeof(valid_pattern)       +\
        sizeof(trc_size_t)          +\
        trace_len

#define TRCLEN2PAYLEN(trc_len)      \
        trc_len                     -\
        CHANNEL_ID_LEN              -\
        TRC_MSG_HDR_LEN

/*
 * STRUCTURES DEFINITIONS
 ****************************************************************************************
 */

///Tracer Environment context structure
struct dbg_trc_mem_env_tag
{
    /// trace memory
    uint8_t *trace_memory;

    /// Read pointer
    uint8_t *read_ptr;

    /// Write pointer
    uint8_t *write_ptr;

    /// Sequence number
    uint16_t  seq_number;

    /// Log buffer size
    uint16_t size;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

///Tracer environment structure external global variable declaration
__STATIC struct dbg_trc_mem_env_tag dbg_trc_mem_env;

/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Checks whether a trace can be written in memory
 *****************************************************************************************
 */
__STATIC bool dbg_trc_mem_writing_allowed(uint16_t trace_len)
{
    uint8_t *data_ptr = dbg_trc_mem_env.write_ptr;
    bool writing_flag = false;
    if (*data_ptr == invalid_pattern)
    {
        //check if the trace fits in the remaining space of the memory
        if (data_ptr + TRACE_BLOCK_SIZE(trace_len) + sizeof(invalid_pattern) >= dbg_trc_mem_env.trace_memory + dbg_trc_mem_env.size)
        {
            //check if the read pointer is at the end of the trace memory
            if (dbg_trc_mem_env.read_ptr == data_ptr)
            {
                //move the read pointer at the beginning of the trace memory
                dbg_trc_mem_env.read_ptr = dbg_trc_mem_env.trace_memory;
            }

            data_ptr = dbg_trc_mem_env.trace_memory;
        }

        // writing pointer points to an invalid location and trace fits in memory
        if (*data_ptr == invalid_pattern && data_ptr + TRACE_BLOCK_SIZE(trace_len) + sizeof(invalid_pattern) < dbg_trc_mem_env.trace_memory + dbg_trc_mem_env.size)
        {
            // the memory is empty
            if (dbg_trc_mem_env.read_ptr == data_ptr)
            {
                writing_flag = true;
            }
            else if (data_ptr < dbg_trc_mem_env.read_ptr)
            {
                // it is possible to write the new trace without overwriting valid memory locations
                if (data_ptr + TRACE_BLOCK_SIZE(trace_len) + sizeof(invalid_pattern) < dbg_trc_mem_env.read_ptr)
                {
                    writing_flag = true;
                }
            }
            else
            {
                // trace fits in memory
                writing_flag = true;
            }
        }
    }
    if (writing_flag == true)
    {
        //update the write pointer
        dbg_trc_mem_env.write_ptr = data_ptr;
    }
    return writing_flag;
}

/*
 * MODULES INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void dbg_trc_mem_init()
{
    memset(&dbg_trc_mem_env, 0, sizeof(dbg_trc_mem_env));
    ble_mem_config_t *p_cfg = ble_memory_config_get();
    dbg_trc_mem_env.trace_memory = p_cfg->log_buf;
    dbg_trc_mem_env.size = p_cfg->log_buf_size;

    //set write pointer to the beginning of the memory
    dbg_trc_mem_env.write_ptr = dbg_trc_mem_env.trace_memory;
    //set read pointer to the beginning of the memory
    dbg_trc_mem_env.read_ptr = dbg_trc_mem_env.trace_memory;
    //initialize number of allocated trace messages
    dbg_trc_mem_env.seq_number = 0;
    //write the invalid pattern in the first byte
    dbg_trc_mem_env.trace_memory[0] = invalid_pattern;
}

uint8_t *dbg_trc_mem_alloc(uint16_t trace_len)
{
    uint8_t *buf = NULL;
    static uint8_t log_lost_num = 0;

    GLOBAL_INT_DISABLE();
    //check if the trace can be written at the location pointed by the write pointer
    if (dbg_trc_mem_writing_allowed(trace_len))
    {
        //Get the trace to write
        buf = dbg_trc_mem_env.write_ptr;

        //Update the validness
        *buf++  =   valid_pattern;
        //Update the block size
        co_write16p(buf, trace_len);
        buf     +=  2;
        //leave space for channel ID to be added next
        buf     +=  CHANNEL_ID_LEN;
        //write the opcode
        *buf++  =   TRACE;
        //write the trace length
        co_write16p(buf, TRCLEN2PAYLEN(trace_len));
        buf     +=  2;
        //write the space for the sequence number
        dbg_trc_mem_env.seq_number++;
        co_write16p(buf, dbg_trc_mem_env.seq_number);
        buf     +=  2;
        //write the timestamp
        co_write32p(buf,  rwip_time_get().hs);
        buf     +=  3;
        *buf    = *buf | (log_lost_num << 4);
        log_lost_num = 0;
        buf     +=  1;
        //Update the write pointer
        dbg_trc_mem_env.write_ptr += TRACE_BLOCK_SIZE(trace_len);
        *dbg_trc_mem_env.write_ptr = invalid_pattern;
    }
    else
    {
        if (15 != log_lost_num)
        {
            log_lost_num++;
        }
    }
    GLOBAL_INT_RESTORE();
    return buf;
}

uint8_t *dbg_trc_mem_read()
{
    uint8_t *data_ptr = NULL;
    GLOBAL_INT_DISABLE();
    ASSERT_INFO((dbg_trc_mem_env.read_ptr >= dbg_trc_mem_env.trace_memory) && (dbg_trc_mem_env.read_ptr <  dbg_trc_mem_env.trace_memory + dbg_trc_mem_env.size), dbg_trc_mem_env.read_ptr, dbg_trc_mem_env.trace_memory);

    if (*(dbg_trc_mem_env.read_ptr) == valid_pattern)
    {
        //valid reading
        data_ptr = dbg_trc_mem_env.read_ptr + sizeof(valid_pattern);
    }
    GLOBAL_INT_RESTORE();
    return data_ptr;
}

void dbg_trc_mem_dealloc()
{
    GLOBAL_INT_DISABLE();
    uint8_t *data_ptr = NULL;
    if (*(dbg_trc_mem_env.read_ptr) == invalid_pattern)
    {
        ASSERT_ERR(0);
    }
    //Update the validness of the trace to deallocate
    *(dbg_trc_mem_env.read_ptr) = invalid_pattern;
    data_ptr = dbg_trc_mem_env.read_ptr;

    //Update the read pointer
    data_ptr += TRACE_BLOCK_SIZE(co_read16p(dbg_trc_mem_env.read_ptr + sizeof(invalid_pattern)));

    //check if the read pointer is at the end of the trace memory
    if ((*data_ptr == invalid_pattern) && (data_ptr != dbg_trc_mem_env.write_ptr))
    {
        data_ptr = dbg_trc_mem_env.trace_memory;
    }
    dbg_trc_mem_env.read_ptr = data_ptr;
    GLOBAL_INT_RESTORE();
}

#endif /*(TRACER_PRESENT)*/
/// @} TRACER
