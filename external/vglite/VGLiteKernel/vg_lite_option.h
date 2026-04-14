/****************************************************************************
*
*    The MIT License (MIT)
*
*    Copyright (c) 2014 - 2022 Vivante Corporation
*
*    Permission is hereby granted, free of charge, to any person obtaining a
*    copy of this software and associated documentation files (the "Software"),
*    to deal in the Software without restriction, including without limitation
*    the rights to use, copy, modify, merge, publish, distribute, sublicense,
*    and/or sell copies of the Software, and to permit persons to whom the
*    Software is furnished to do so, subject to the following conditions:
*
*    The above copyright notice and this permission notice shall be included in
*    all copies or substantial portions of the Software.
*
*    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
*    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
*    DEALINGS IN THE SOFTWARE.
*
*****************************************************************************/


#ifndef _vg_lite_option_h_
#define _vg_lite_option_h_

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Set gcdVG_ENABLE_WRITEBUFFER to 1 for Linux Write Combined memory access
 * Set gcdVG_ENABLE_WRITEBUFFER to 0 for Linux Uncacheable memory access
*/
#ifndef gcdVG_ENABLE_WRITEBUFFER
    #define gcdVG_ENABLE_WRITEBUFFER                     1
#endif

/*
 * Backup state command, only support Linux and RTOS
*/
#ifndef gcdVG_ENABLE_BACKUP_COMMAND
    #define gcdVG_ENABLE_BACKUP_COMMAND                  0
#endif

/*
 * Backup command buffer by user
*/
#ifndef gcdVG_ENABLE_COMMAND_BUFFER_CACHE
    #define gcdVG_ENABLE_COMMAND_BUFFER_CACHE            0
#endif

/*
 * Power management, only support Linux and RTOS
*/
#ifndef gcdVG_ENABLE_POWER_MANAGEMENT
    #define gcdVG_ENABLE_POWER_MANAGEMENT                0
#endif

/*
 * when set to 1, vg_lite_hal_trace can use to print message
*/
#ifndef gcdVG_ENABLE_DEBUG
    #define gcdVG_ENABLE_DEBUG                           1
#endif

/*
 * when set to 1, dump last submit command from kernel
*/
#ifndef gcdVG_ENABLE_DUMP_COMMAND
    #define gcdVG_ENABLE_DUMP_COMMAND                    0
#endif

/*
 * when gpu hang, set 1 to open gpu reset function
*/
#ifndef gcdVG_ENABLE_GPU_RESET
    #define gcdVG_ENABLE_GPU_RESET                       0
#endif

/*
 * Set 1 to open gpu auto clock gating feature
*/
#ifndef gcdVG_ENABLE_AUTO_CLOCK_GATING
    #define gcdVG_ENABLE_AUTO_CLOCK_GATING               0
#endif

/*
 * Set 1 to open dump debug register
*/
#ifndef gcdVG_DUMP_DEBUG_REGISTER
    #define gcdVG_DUMP_DEBUG_REGISTER                    0
#endif

/*
 * For zephyr system
 * Set gcdVG_ENABLE_DELAY_RESUME to 1 to open delay resume feature.
*/
#ifndef gcdVG_ENABLE_DELAY_RESUME
    #define gcdVG_ENABLE_DELAY_RESUME                    0
#endif

/*
 * Set 1 to record GPU hardware running time.
*/
#ifndef gcdVG_RECORD_HARDWARE_RUNNING_TIME
    #define gcdVG_RECORD_HARDWARE_RUNNING_TIME           0
#endif

#ifdef __cplusplus
}
#endif

#endif
