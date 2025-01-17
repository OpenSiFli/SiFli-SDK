/**
  ******************************************************************************
  * @file   bts2_task.h
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

#ifndef _BTS2_TASK_H_
#define _BTS2_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif

#define BTS2_TASK_MAX_NAME_LEN       (16)
#define BTS2_MSG_MAX_NUM             (0xFF)
#define BTS2_TIME_EV_MAX_NUM         (0xFF)

typedef void (fn_type_init)(void **);
typedef void (fn_type_rel)(void **);
typedef void (fn_type_hdl)(void **);

typedef struct
{
    fn_type_init *fn_init;
    fn_type_rel *fn_rel;
    fn_type_hdl *fn_hdl;
#if defined(CFG_BT_DBG)
    char task_name[BTS2_TASK_MAX_NAME_LEN];
#endif
} BTS2S_TASK_TYPE;

typedef struct
{
    void                *inst_data;
    BTS2S_MSG_FIFO_TYPE *msg_list;
    BTS2S_MSG_FIFO_TYPE *msg_list_tail;
    BTS2S_MSG_FIFO_TYPE *preempt_msg;
} BTS2S_TASK_INFO_TYPE;

typedef struct
{
    U16 cur_task_id;                    /* current task id of BTS2T_ID          */
    U16 pending_msg_num;                /* num of pending message               */
    U16 timed_ev_num;                   /* num of timed event                   */
    U16 interrupt_set;                  /* interrupt set                        */
    U32 unique_id;                      /* next unique id                       */
    U32 cur_time_ev_id;                 /* current timed event unique id        */
    BOOL bexit;                         /* flag of BTS2 stop                    */
#ifdef CFG_BT_DBG
    void (*fn_msg_cbk)(U16 task_src, U16 task_dest, U16 msg_cls, U16 msg_type);
    /* call back function of message put    */
#endif
    BTS2S_TIME_EV_TYPE *timed_ev_list;   /* list of timed events                 */
    BTS2S_TASK_INFO_TYPE *task_info;     /* tasks information data               */
} BTS2S_TASK_INST;

typedef enum
{
    BTS2T_HCI_CMD = 0,
    BTS2T_HCI_EVT,
    BTS2T_HCI_CONN,
    BTS2T_L2C,
    BTS2T_L2C_HCI,
    BTS2T_RFC,/*5*/
    BTS2T_RFC_L2C,
    BTS2T_RFC_HCI,
    BTS2T_SDP_L2C,
    BTS2T_SDP,
    BTS2T_GAP,/*10*/
    BTS2T_SC,

#ifdef BSP_BTS2_SDAP
    BTS2T_SDAP,/*12*/
#endif

#ifdef CFG_BPP_SRV
    BTS2T_BPP_SRV,
#endif

#ifdef CFG_SPP_CLT
    BTS2T_SPP_CLT,
#endif

#ifdef CFG_SPP_SRV
    BTS2T_SPP_SRV,
#endif

#ifdef CFG_HFP_HF
    BTS2T_HFP_HF,
#endif

#ifdef CFG_HFP_AG
    BTS2T_HFP_AG,
#endif

#ifdef CFG_FTP_CLT
    BTS2T_FTP_CLT,
#endif

#ifdef CFG_FTP_SRV
    BTS2T_FTP_SRV,
#endif

#ifdef CFG_OPP_CLT
    BTS2T_OPP_CLT,
#endif

#ifdef CFG_OPP_SRV
    BTS2T_OPP_SRV,
#endif

#ifdef CFG_AV
    BTS2T_AV,
#endif

#ifdef CFG_AVRCP
    BTS2T_AVRCP,
#endif

#ifdef CFG_HID
    BTS2T_HID,
#endif

#ifdef CFG_HID_HOST
    BTS2T_HID_HOST,
#endif

#ifdef CFG_BIP_CLT
    BTS2T_BIP_CLT,
#endif

#ifdef CFG_BIP_SRV
    BTS2T_BIP_SRV,
#endif

#ifdef CFG_BPP_CLT
    BTS2T_BPP_CLT,
#endif

#ifdef CFG_DUN_CLT
    BTS2T_DUN_CLT,
#endif
#ifdef CFG_DUN_SRV
    BTS2T_DUN_SRV,
#endif

#ifdef CFG_FAX_CLT
    BTS2T_FAX_CLT,
#endif
#ifdef CFG_FAX_SRV
    BTS2T_FAX_SRV,
#endif

#ifdef CFG_PAN
    BTS2T_BNEP,
    BTS2T_PAN,
#endif
#ifdef CFG_HCRP_SRV
    BTS2T_HCRP_SRV,
#endif
#ifdef CFG_SYNC_SRV
    BTS2T_SYNC_SRV,
#endif

#ifdef CFG_SAP_CLT
    BTS2T_SAP_CLT,
#endif

#ifdef CFG_SAP_SRV
    BTS2T_SAP_SRV,
#endif

#ifdef CFG_PBAP_CLT
    BTS2T_PBAP_CLT,
#endif

#ifdef CFG_PBAP_SRV
    BTS2T_PBAP_SRV,
#endif

#ifdef CFG_BR_GATT_SRV
    BTS2T_GATT_OVER_BR,
#endif

#ifdef CFG_BT_DIS
    BTS2T_BT_DIS,
#endif

#ifdef CFG_BT_L2CAP_PROFILE
    BTS2T_BT_L2CAP_PROFILE,
#endif

    BTS2T_APP,
//#ifdef ENABLE_SOLUTON_BT_INTERFACE
//   BTS2T_BT_SIFLI_SOLUTION,
//#endif

    BTS2T_NUM
} BTS2T_ID;

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Register a interrupt service routine to BTS2 task message looping.
 *      Once the ISR registered and related vector be set, the isr_fn will be
 *      called.
 *
 * INPUT:
 *      U8 vector: interrupt vector id from 1 - 3.
 *      void (* isr_fn)(void): pointer of ISR.
 *
 * OUTPUT:
 *      TRUE if successful, otherwise FALSE.
 *
 * NOTE:
 *      The ISR can be registered again which will result in replace the old one.
 *
 *----------------------------------------------------------------------------*/
BOOL bts2_interrupt_service_routine_reg(U8 vector, void (* isr_fn)(void));

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Request the interrupt with the specified vector.
 *
 * INPUT:
 *      U8 vector: interrupt vector id from 1 - 8.
 *
 * OUTPUT:
 *      void
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
void bts2_interrupt_req(U8 vector);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Register the BTS2 application handle functions.
 *
 * INPUT:
 *      void *init: used to initialize resources before BTS2 running.
 *      void *rel:  used to release resources before BTS2 stop.
 *      void *hdl:  the main handler function used to process all messages
 *                  received from BTS2.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      void.
 *
 *----------------------------------------------------------------------------*/
void bts2_app_reg_cbk(fn_type_init *init, fn_type_rel *rel, fn_type_hdl *hdl);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     Initialize the BTS2 task and message looping unit.
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
void bts2_task_init(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      This function used to make the BTS2 task and message looping mechanism
 *      begin to running.
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
void bts2_task_run(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     This function used to call task release function handler before BTS2 stop.
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
void bts2_task_rel(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     This function used to stop the BTS2 task and message looping.
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
void bts2_task_stop(void);



// #ifdef CFG_BT_DBG
#if 0
/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Register a BTS2 message call back function which used to trace out
 *      task_src send message xxxx to task_dest.
 *
 * INPUT:
 *      void (*fn_msg_cbk)(U16 task_src, U16 task_dest, U16 msg_cls, U16 msg_type)
 *      prototype of call back function.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      In the call back function, bts2_get_task_name function can be used.
 *
 *----------------------------------------------------------------------------*/
void bts2_msg_cbk_reg(void (*fn_msg_cbk)(U16 task_src, U16 task_dest, U16 msg_cls, U16 msg_type));

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Get the task name string.
 *
 * INPUT:
 *      U16 id: task id.
 *
 * OUTPUT:
 *      the string of task name.
 *
 * NOTE:
 *      none.
 *
 *----------------------------------------------------------------------------*/
char *bts2_get_task_name(U16 id);

#endif

#ifdef __cplusplus
}
#endif

#endif

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
