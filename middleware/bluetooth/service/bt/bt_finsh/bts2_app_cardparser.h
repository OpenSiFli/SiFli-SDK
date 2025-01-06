/**
  ******************************************************************************
  * @file   bts2_app_cardparser.h
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

#ifndef CARD_PARSER_H
#define CARD_PARSER_H

#ifdef __cplusplus
extern "C"
{
#endif

#define CARD_PARSER_VER_MAJOR 1
#define CARD_PARSER_VER_MINOR 0
#define CARD_PARSER_VER "1.0"

typedef char CARD_Char;
typedef void *CARD_Parser;

/* version */
const char *CARD_ParserVersion();

/* setup & parsing */
CARD_Parser CARD_ParserCreate(CARD_Char *encoding);
void CARD_ParserFree(CARD_Parser p);
int CARD_Parse(CARD_Parser p, const char *s, int len, int isFinal);

/* user data */
void CARD_SetUserData(CARD_Parser p, void *userData);
void *CARD_GetUserData(CARD_Parser p);

/* handlers */
/** \typedef typedef void (*CARD_PropHandler)(void *userData, const CARD_Char *propname, const CARD_Char **params);
    \param userData user data specified for the card parser (CARD_SetUserData())
    \param propname name of property for this event
    \param params null terminated list of param name/value pairs
            - params[0] = param name
            - params[1] = param value (can be null)
            - params[n] = 0
    \return nothing
    \brief Called when a property & its params is fully parsed
*/
typedef void (*CARD_PropHandler)(void *userData, const CARD_Char *propname, const CARD_Char **params);

/** \typedef typedef void (*CARD_DataHandler)(void *userData, const CARD_Char *data, int len);
    \param userData user data specified for the card parser (CARD_SetUserData())
    \param data pointer to decoded data for a parameter (data = NULL = eod)
    \param len length of data (len = 0 indicates eod)
    \return nothing
    \brief Called when a data chunk for a property is decoded
*/
typedef void (*CARD_DataHandler)(void *userData, const CARD_Char *data, int len);

/* Set Handlers */
void CARD_SetPropHandler(CARD_Parser p, CARD_PropHandler cardProp);
void CARD_SetDataHandler(CARD_Parser p, CARD_DataHandler cardData);

#ifdef __cplusplus
}
#endif


#endif/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
