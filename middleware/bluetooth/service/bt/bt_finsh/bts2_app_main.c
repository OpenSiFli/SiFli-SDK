/**
  ******************************************************************************
  * @file   bts2_app_main.c
  * @author Sifli software development team
  ******************************************************************************
*/
/*
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
#include "rtthread.h"
#include "bts2_app_inc.h"

#ifdef PKG_USING_AAC_DECODER_LIBFAAD
    #define BTS_TASK_SIZE (50*1024)
#else
    #define BTS_TASK_SIZE 0xC00  // (3072)
#endif

#include "log.h"


#ifdef CFG_BT_DBG
void bts2_app_dbg_msg(U16 task_src, U16 task_dest, U16 msg_cls, U16 msg_type)
{
#if 0
    char *task_name_src, *task_name_dest;
    task_name_src = bts2_get_task_name(task_src);
    task_name_dest = bts2_get_task_name(task_dest);
    DBG_TRACE(("task %s send message %d %d to task %s\r\n", task_name_src, msg_cls, msg_type, task_name_dest));
#endif
}
#endif

#if 0
/*----------------------------------------------------------------------------*
 * PAR:
 *      U8 dir
 *          1 tx
 *          0 rx
 *      U8 type
 *          1 command
 *          2 acl
 *          3 sco
 *          4 event
 *      U16 len
 *      U8* data
 *      BTS2S_BD_ADDR* Bluetooth device address, only used for ACL data
 *----------------------------------------------------------------------------*/

void bts2_data_info_cb(U8 dir, U8 type, U16 len, const U8 *data, BTS2S_BD_ADDR *bd)
{
    switch (type)
    {
    case 1:
        /* HCI command */
        break;

    case 2:
        /* ACL data send out */
        if (dir == 1)
        {
            /* add DATA OUT length number with the specified bd addr */
            ;
        }
        /* ACL data received */
        else
        {
            /* add DATA IN length number with the specified bd addr */
            ;
        }
        break;

    case 3:
        /* SCO data */
        break;

    case 4:
        /* HCI event */
        break;

    default:
        break;

        /* log data out in release mode, something like DBG_DATA() */
    }
}
#endif

// #ifdef CFG_BT_DBG
//     U16 bts2g_dbg_set;
// #endif

extern U16 bts2g_dbg_set;
static uint8_t g_is_created;
typedef void (fn_type_init)(void **);
typedef void (fn_type_rel)(void **);
typedef void (fn_type_hdl)(void **);

void bts2_app_reg_cbk(fn_type_init *init, fn_type_rel *rel, fn_type_hdl *hdl);

static void bts_thread_entry(void *parameter)
{
    extern void bts2_run(void);
    void bts2_task_rel(void);

    bts2_run();
    bts2_task_rel();
    g_is_created = 0;
}

static int bts_run_init()
{
    rt_thread_t tid;

    tid = rt_thread_create("bts",
                           bts_thread_entry, RT_NULL,
                           BTS_TASK_SIZE, RT_THREAD_PRIORITY_HIGH, RT_THREAD_TICK_DEFAULT);

    if (tid != RT_NULL)
        rt_thread_startup(tid);

    return 0;
}

void bts2_main(void)
{
    BOOL r = TRUE;
    if (g_is_created)
        return;

    g_is_created = 1;
    /*BTS2 key functions*/
    bts2_app_reg_cbk((void *)app_fn_init, (void *)app_fn_rel, (void *)app_fn_hdl_ext);
    bts2_init((U8 *)"bts2_sm.db");
#ifdef CFG_BT_DBG
    bts2g_dbg_set |=
        DBG_FUNC_ON
        //| DBG_WARNING_ON
        //| DBG_ERROR_ON
        //| DBG_BREAK_ON
        //| DBG_TRACE_ON
        //| DBG_MSG_ON
        // DBG_DATA_ON
        ;
#endif
#ifdef CFG_BT_DBG
    // bts2_msg_cbk_reg(bts2_app_dbg_msg);
#endif

#if 0
    gap_reg_bts2_data_info_callback(bts2_data_info_cb);
#endif

#if defined(CFG_TL_USB)
    r = usb_drv_start(NULL);
#endif

#ifdef CFG_MS
    bt_close_kb_thread();
#endif
    bts_run_init();
}
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
