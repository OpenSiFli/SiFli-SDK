/**
 ****************************************************************************************
 *
 * @file arch_main.c
 *
 * @brief Main loop of the application.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */


/*
 * INCLUDES
 ****************************************************************************************
 */

#include "rtconfig.h"

#include "rwip_config.h" // RW SW configuration

#include "arch.h"      // architectural platform definitions
#include <stdlib.h>    // standard lib functions
#include <stddef.h>    // standard definitions
#include <stdint.h>    // standard integer definition
#include <stdbool.h>   // boolean definition
#include "rwip.h"      // RW SW initialization
#include "dbg_trc.h"
#include "rom_config.h"


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (PLF_DEBUG)

/// Variable to enable infinite loop on assert
volatile int dbg_assert_block = 1;


void rw_assert_err(const char *condition, const char *file, int line)
{
    TRC_REQ_SW_ASS_ERR(file, line, 0, 0);

    // Trigger assert message
    rwip_assert(file, line, 0, 0, ASSERT_TYPE_ERROR);

    // Let time for the message transfer
    for (int i = 0; i < 2000; i++)
    {
        dbg_assert_block = 1;
    };

    //asrt_line_set(line);
    //asrt_addr_setf((uint32_t)file);
    //asrt_trigg_setf(1);

    //GLOBAL_INT_STOP();

    while (dbg_assert_block);
}

void rw_assert_param(int param0, int param1, const char *file, int line)
{
    TRC_REQ_SW_ASS_ERR(file, line, param0, param1);

    // Trigger assert message
    rwip_assert(file, line, param0, param1, ASSERT_TYPE_ERROR);
    // Let time for the message transfer
    for (int i = 0; i < 2000; i++)
    {
        dbg_assert_block = 1;
    };

    //asrt_line_set(line);
    //asrt_addr_setf((uint32_t)file);
    //asrt_params_setf(1);
    //asrt_param_1_setf(param0);
    //asrt_param_2_setf(param1);
    //asrt_params_setf(1);
    //asrt_trigg_setf(1);

    //GLOBAL_INT_STOP();
    while (dbg_assert_block);
}

void rw_assert_warn(int param0, int param1, const char *file, int line)
{
    TRC_REQ_SW_ASS_WARN(file, line, param0, param1);

    // Trigger assert message
    rwip_assert(file, line, param0, param1, ASSERT_TYPE_WARNING);

    //asrt_line_set(line);
    //asrt_addr_setf((uint32_t)file);
    //asrt_params_setf(0);
    //asrt_warn_setf(1);

    DUMP_DATA(&param0, sizeof(int));
    DUMP_DATA(&param1, sizeof(int));
}

void dump_data(uint8_t *data, uint16_t length)
{
    //asrt_param_1_setf(length);
    //asrt_params_setf(1);
    //asrt_addr_setf((uint32_t)data);
    //asrt_warn_setf(1);
}


void dump_hci(uint8_t type, uint8_t direction, uint8_t *p_data, uint16_t length, uint8_t *p_hdr_data, uint16_t hdr_length)
{
#if 0
    asrt_dump_ext_cfg_set(hdr_length);
    asrt_dump_ext_data_addr_setf((uint32_t)p_hdr_data);
    asrt_dump_data_addr_setf((uint32_t)p_data);
    asrt_dump_cfg_pack(/*log*/    0,
                                  /*hci*/    1,
                                  /*ext*/ ((hdr_length > 0) ? 1 : 0),
                                  /*packed*/ 1,
                                  /*level*/  0,
                                  /*dir*/ ((direction == 1) ? 1 : 0),
                                  /*type*/   type,
                                  /*length*/ length);
#endif
}


void dump_upk_hci(uint8_t evttype, uint8_t direction, uint16_t code, uint8_t *p_data, uint16_t length)
{
#if 0
    asrt_dump_ext_cfg_set(code);
    asrt_dump_data_addr_setf((uint32_t)p_data);
    asrt_dump_cfg_pack(/*log*/    0,
                                  /*hci*/    1,
                                  /*ext*/    0,
                                  /*packed*/ 0,
                                  /*level*/  0,
                                  /*dir*/ ((direction == 1) ? 1 : 0),
                                  /*type*/   evttype,
                                  /*length*/ length);
#endif
}

void dump_str(int log_level, const char *p_format, ...)
{
#if 0
    va_list argptr;
    va_start(argptr, p_format);
    PRINT_STR(log_level, p_format, argptr);
    va_end(argptr);
#endif
}

void print_str(uint8_t level, uint8_t **pp_format)
{
#if 0
    asrt_text_addr_setf((uint32_t) *pp_format);
    asrt_dump_data_addr_setf((uint32_t) pp_format);
    asrt_dump_cfg_pack(/*log*/    1,
                                  /*hci*/    0,
                                  /*ext*/    0,
                                  /*packed*/ 1,
                                  /*level*/  level & 0x7,
                                  /*dir*/    0,
                                  /*type*/   0,
                                  /*length*/ 0);
#endif
}

#endif //PLF_DEBUG

uint16_t get_stack_usage(void)
{
    //TODO;
    return 0;
}

void platform_reset(uint32_t error)
{
    while (1);
}

void platform_assert(void)
{
    RT_ASSERT(0);
}

/*
 * MAIN FUNCTION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief RW main function.
 *
 * This function is called right after the booting process has completed.
 *
 * @return status   exit status
 ****************************************************************************************
 */
void rw_main(void)
{

}


/// @} DRIVERS
