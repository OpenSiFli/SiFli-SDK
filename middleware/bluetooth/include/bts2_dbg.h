/**
  ******************************************************************************
  * @file   bts2_dbg.h
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

#ifndef _BTS2_DBG_H_
#define _BTS2_DBG_H_
#include "stdint.h"
#include "rtthread.h"
#include "bf0_ble_common.h"
#define BT_TAG_D 0
#define BT_TAG_I 1
#define BT_TAG_W 2
#define BT_TAG_E 3
#define BT_TAG_V 4


#ifdef CFG_BT_DBG

#ifdef CFG_MS
    #include <stdlib.h>
    #ifdef CFG_DBG_MEM_LEAK
        #define _CRTDBG_MAP_ALLOC
        #include <crtdbg.h>

        /*----------------------------------------------------------------------------*
        *
        * DESCRIPTION:
        *      Dump out the memory leak information of BTS2.
        *
        * INPUT:
        *      void
        *
        * OUTPUT:
        *      void.
        *
        * NOTE:
        *      windows function.
        *
        *----------------------------------------------------------------------------*/
        void mem_leak_dump(void);

    #else

    #endif
#endif

#define SAVE_MEM_PATH       "bts2_mem.log"
#define DBG_PRINT           dbg_print  /* only be used in bts2_dbg.c    */

extern U16 bts2g_dbg_set;

#define DBG_FUNC_ON         0x0001     /* Enable Message trace          */
#define DBG_TRACE_ON        0x0002     /* Enable trace                  */
#define DBG_MSG_ON          0x0004     /* Enable Message trace          */
#define DBG_DATA_ON         0x0008     /* Enable tx/rx packet           */
#define DBG_WARNING_ON      0x0010     /* Display warning messages      */
#define DBG_ERROR_ON        0x0020     /* Display error messages        */
#define DBG_BREAK_ON        0x0040     /* Enable break                  */

#define DBG_FUNC(F)     {static const char __FUNC__[] = F;}

#define DBG_TRACE(T)    {if (bts2g_dbg_set & DBG_TRACE_ON) DBG_PRINT T;}

#define DBG_MSG(A,B,C,D)

#define DBG_DATA(A,B,C) {if (bts2g_dbg_set & DBG_DATA_ON) dbg_trace_data(A,B,C);}

#define DBG_WARNING(W)  {if (bts2g_dbg_set & DBG_WARNING_ON) \
                            {DBG_PRINT W; \
                             DBG_PRINT("Warning at line %d in file %s\r\n",__LINE__,__FILE__);}}

#define DBG_ERROR(E)    {if (bts2g_dbg_set & DBG_ERROR_ON) \
                            {DBG_PRINT E; \
                             DBG_PRINT("Error at line %d in file %s\r\n",__LINE__,__FILE__);}}

#define DBG_BREAK(B)    {if (bts2g_dbg_set & DBG_BREAK_ON) \
                            {DBG_PRINT B; \
                             DBG_PRINT("BTS2 stop at line %d file %s\r\n",__LINE__,__FILE__);\
                             /*exit(0);*/}}

#define BT_DBG_D(...) bt_log_output(BT_TAG_D, __VA_ARGS__)
#define BT_DBG_I(...) bt_log_output(BT_TAG_I, __VA_ARGS__)
#define BT_DBG_W(...) bt_log_output(BT_TAG_W, __VA_ARGS__)
#define BT_DBG_E(...) bt_log_output(BT_TAG_E, __VA_ARGS__)

//#define BT_ASSERT(expr) RT_ASSERT(expr)
//#define BT_OOM_ASSERT(expr) RT_ASSERT(expr)

typedef void (*bt_fsm_hook_function)(const uint8_t *string, uint8_t state, uint8_t evt);
void bt_log_output(uint8_t tag, char *fmt, ...);
void bt_fsm_hook_set(bt_fsm_hook_function hook);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      The dbg_print function formats and prints a series of characters and
 *      values.
 *
 * INPUT:
 *      char *fmt: Format control.
 *      ...: Optional arguments.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      This is a system independent functions, if in windows system we output the
 *      information data to VC out put window, if in other embedded system, it's
 *      better to output in UART or something like that.
 *
 *----------------------------------------------------------------------------*/
void dbg_print(char *fmt, ...);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Trace out the HCI Data which include HCI command, event and  ACL data.
 *
 * INPUT:
 *      U8 *data: characters string.
 *      int len: the length of data.
 *      char * indicator: a hint of print string.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void dbg_trace_data(U8 *data, U16 len, char *indicator);

#else //!CFG_BT_DBG

#define DBG_FUNC(F)
#define DBG_TRACE(T)
#define DBG_DATA(A,B,C)
#define DBG_MSG(A,B,C,D)
#define DBG_WARNING(W)
#define DBG_ERROR(E)
#define DBG_BREAK(B)
#ifdef CFG_BT_DBG_EXT
    #define BT_DBG_D(...) bt_log_output(BT_TAG_D, __VA_ARGS__)
    #define BT_DBG_I(...) bt_log_output(BT_TAG_I, __VA_ARGS__)
    #define BT_DBG_W(...) bt_log_output(BT_TAG_W, __VA_ARGS__)
    #define BT_DBG_E(...) bt_log_output(BT_TAG_E, __VA_ARGS__)
#else // !CFG_BT_DBG_EXT
    #define BT_DBG_D(...)
    #define BT_DBG_I(...)
    #define BT_DBG_W(...)
    #define BT_DBG_E(...)
#endif

//#define BT_ASSERT(expr) RT_ASSERT(expr)
//#define BT_OOM_ASSERT(expr) RT_ASSERT(expr)
typedef void (*bt_fsm_hook_function)(const uint8_t *string, uint8_t state, uint8_t evt);
void bt_log_output(uint8_t tag, char *fmt, ...);
void bt_fsm_hook_set(bt_fsm_hook_function hook);

#endif

#ifdef CFG_BT_DBG_MEM2

    // #define DBG_IN(F)               {DBG_FUNC(F);  dbg_stk_in(F);}
    // #define DBG_OUT()               dbg_stk_out();
    // #define DBG_STK_START(FLAG)     dbg_stk_start(FLAG);
    // #define DBG_STK_STOP()          dbg_stk_stop();
    // #define DBG_STK_RESET()         dbg_stk_reset();
    // #define DBG_STK_TASK(IDX)       dbg_stk_task(IDX);
    //#define DBG_IN(F)  {DBG_FUNC(F);  DBG_PRINT("%s\n",F);}
    #define DBG_IN(F)
    #define DBG_OUT()
    #define DBG_STK_START(FLAG)
    #define DBG_STK_STOP()
    #define DBG_STK_RESET()
    #define DBG_STK_TASK(IDX)
#else
    #ifdef CFG_BT_DBG
        #define DBG_IN(F) {DBG_FUNC(F);  DBG_PRINT("%s\n",F);}
    #elif defined(CFG_BT_DBG_EXT) && defined(BSP_USING_PC_SIMULATOR) // CFG_BT_DBG
        #define DBG_IN(...) bt_log_output(BT_TAG_V,__VA_ARGS__);
    #else
        #define DBG_IN(F)
    #endif // CFG_BT_DBG
    #define DBG_OUT()
    #define DBG_STK_START(FLAG)
    #define DBG_STK_STOP()
    #define DBG_STK_RESET()
    #define DBG_STK_TASK(IDX)

#endif

typedef struct BTS2S_STK_INFO_TAG
{
    U32 base;           /* mark the stack location when program setup       */
    U32 maxstk;
    U32 *pmaxstk;       /* point a buffer stored max stack every queue      */
    S8 func[512];       /* current max call graph                           */
    S8 cur_func[512];   /* current call graph                               */
    S8 **pf;            /* point a buffer stored max call graph every queue */
    U8 flag;
    U8 count;
} BTS2S_STK_INFO;

#ifdef CFG_BT_DBG_MEM2
    /*----------------------------------------------------------------------------*
    *
    * DESCRIPTION:
    *      Initialize variables, save the original stack address.
    *
    * INPUT:
    *      U8 flag.
    *
    * OUTPUT:
    *      void.
    *
    * NOTE:
    *      none.
    *
    *----------------------------------------------------------------------------*/
    void dbg_stk_start(U8 flag);

    /*----------------------------------------------------------------------------*
    *
    * DESCRIPTION:
    *      Print the used stack information, call it when program exit.
    *
    * INPUT:
    *      void.
    *
    * OUTPUT:
    *      void.
    *
    * NOTE:
    *      none.
    *
    *----------------------------------------------------------------------------*/
    void dbg_stk_stop();

    /*----------------------------------------------------------------------------*
    *
    * DESCRIPTION:
    *      Push function name into memory, if the stack peak value is big than saved
    *      value, note it.
    *
    * INPUT:
    *      char *func: function name.
    *
    * OUTPUT:
    *      current used stack peak value.
    *
    * NOTE:
    *      none.
    *
    *----------------------------------------------------------------------------*/
    U32 dbg_stk_in(const char *func);

    /*----------------------------------------------------------------------------*
    *
    * DESCRIPTION:
    *      Delete function name from memory when this function close, make buffer
    *      depict function call stack in time.
    *
    * INPUT:
    *      none.
    *
    * OUTPUT:
    *      void.
    *
    * NOTE:
    *      none.
    *
    *----------------------------------------------------------------------------*/
    void dbg_stk_out();

    /*----------------------------------------------------------------------------*
    *
    * DESCRIPTION:
    *      Flush statistic value for saved next task information.
    *
    * INPUT:
    *
    *
    * OUTPUT:
    *      void.
    *
    * NOTE:
    *      none.
    *
    *----------------------------------------------------------------------------*/
    void dbg_stk_reset();

    /*----------------------------------------------------------------------------*
    *
    * DESCRIPTION:
    *      Save one task stack information when a task loop end.
    *
    * INPUT:
    *      U16 id: task number.
    *
    * OUTPUT:
    *      void.
    *
    * NOTE:
    *      none.
    *
    *----------------------------------------------------------------------------*/
    void dbg_stk_task(U16 id);

#endif

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
