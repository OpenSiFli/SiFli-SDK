/**
  ******************************************************************************
  * @file   message_layer.c
  * @author Sifli software development team
  * @brief     Motion Library - Message Layer
 *              Holds Low Occurance messages
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

#include "message_layer.h"
#include "log.h"

struct message_holder_t
{
    long message;
};

static struct message_holder_t mh;

/** Sets a message.
* @param[in] set The flags to set.
* @param[in] clear Before setting anything this will clear these messages,
*                  which is useful for mutually exclusive messages such
*                  a motion or no motion message.
* @param[in] level Level of the messages. It starts at 0, and may increase
*            in the future to allow more messages if the bit storage runs out.
*/
void inv_set_message(long set, long clear, int level)
{
    if (level == 0)
    {
        mh.message &= ~clear;
        mh.message |= set;
    }
}

/** Returns Message Flags for Level 0 Messages.
* Levels are to allow expansion of more messages in the future.
* @param[in] clear If set, will clear the message. Typically this will be set
*  for one reader, so that you don't get the same message over and over.
* @return bit field to corresponding message.
*/
long inv_get_message_level_0(int clear)
{
    long msg;
    msg = mh.message;
    if (clear)
    {
        mh.message = 0;
    }
    return msg;
}

/**
 * @}
 */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
