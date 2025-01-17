/**
  ******************************************************************************
  * @file   bts2_api.h
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

#ifndef _BTS2_API_H_
#define _BTS2_API_H_
#ifdef __cplusplus
extern "C" {
#endif
#define BTS2_VER_LEN     25

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      BTS2 initialize related resources to prepare to run.
 *
 * INPUT:
 *      BTS2S_DB_FILE_PATH path: DB file path.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      True if successful, otherwise failed.
 *
 *----------------------------------------------------------------------------*/
U8 bts2_init(BTS2S_DB_FILE_PATH path);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Start the BTS2 task message loop running.
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
void bts2_run(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Let the BTS2 task message loop begin to run each task release function
 *      to release related resources.
 *
 * INPUT:
 *      void.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      True if successful, otherwise failed.
 *
 *----------------------------------------------------------------------------*/
BOOL bts2_rel(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Let the BTS2 task message loop running exit.
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
void bts2_stop(void);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      Get the BTS2 version.
 *
 * INPUT:
 *      char *ver: The char string used to get the BTS2 version back.
 *
 * OUTPUT:
 *      void.
 *
 * NOTE:
 *      The input string length should not be less than BTS2_VER_LEN.
 *
 *----------------------------------------------------------------------------*/
void bts2_get_ver(char *ver);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *     This function used to get app task id.
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
U16 bts2_task_get_app_task_id();

U16 bts2_task_get_pan_task_id();



U16 bts2_get_l2cap_default_mtu_size(void);

#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
