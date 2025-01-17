/**
  ******************************************************************************
  * @file   bts2_fsm.h
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

#ifndef _BTS2_FSM_H_
#define _BTS2_FSM_H_

/* event value used to exit from FSM */
#define BTS2_FSM_EV_FINISH           0xFF    /* max event number should within 255 */
#define BTS2_FSM_IDX_NULL            0xFF    /* max function index should within 255 */

/* result code of handler function */
#define BTS2_FSM_FAIL                0
#define BTS2_FSM_SUCC                1

/* prototype of handler function of FSM,  */
typedef U8(*BTS2_FSM_FUNC)(void *p1, void *p2, U8 *pst);

/* BTS2 FSM Instance definition */
typedef struct
{
    const BTS2_FSM_FUNC *func;
    const U8 *matrix;
#ifdef CONFIG_DBG
    const U8 *fsm_name;   /* string of FSM name */
#endif
    const U8 max_st;
    const U8 max_ev;
} BTS2S_FSM_INST;

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     The engine function for BTS2 full State Machine which will keep running
 *     until until there are no more events.
 *
 * INPUT:
 *     const BTS2S_FSM_INST *inst
 *     U8 *pst
 *     U8 ev
 *     void *p1
 *     void *p2
 *
 * OUTPUT:
 *     BTS2_FSM_SUCC if successful, otherwise BTS2_FSM_FAIL.
 *
 * NOTE:
 *     the instant function should have *pst = new_state code sentence.
 *----------------------------------------------------------------------------*/
U8 bts2_fsm(const BTS2S_FSM_INST *inst, U8 *pst, U8 ev, void *p1, void *p2);

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
