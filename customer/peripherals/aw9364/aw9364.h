/**
  ******************************************************************************
  * @file   aw9364.h
  * @author Sifli software development team
  ******************************************************************************
*/
/**
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

#ifndef __AW9364_H
#define __AW9364_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* macro ------------------------------------------------------------------*/
//larger than 20 us, set to 50
#define AW9364_READYTIME_TON               (50)

// larger than 0.5us ,set to 50
#define AW9364_PULSE_THI                   (50)

// larger than 0.5us and less than 500us, set to 50
#define AW9364_PULSE_TLO                   (50)

// larger than 2.5ms
#define AW9364_SHUTDONW_TSHDN              (3000)


#define AW9364_LIGHT_MAX_LEVEL             (16)

/* type ------------------------------------------------------------------*/

/* function ------------------------------------------------------------------*/

/**
 * @brief open backlight
 * @return 0 if success.
 */
int sif_aw9364_open();

/**
 * @brief close backlight
 * @return 0 if success.
 */
int sif_aw9364_close();

/**
 * @brief set backlight
 * @param[in] backlight, from 0 ~ 16, 0 is closed, 16 max.
 * @return 0 if success.
 */
int sif_aw9364_set_backlight(uint8_t backlight);


#ifdef __cplusplus
}
#endif

#endif /* __AW9364_H */


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
