/**
  ******************************************************************************
  * @file   bts2_app_cardparser.c
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

#include "bts2_app_inc.h"
#include "bts2_app_cardparser.h"
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

#ifndef TRUE
    #define TRUE -1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

/*--------------------------------------
implementation
*/

/* version */
const char *CARD_ParserVersion()
{
    return CARD_PARSER_VER;
};

typedef enum {cp_PropName, cp_ParamBN, cp_ParamIN, cp_ValueBV, cp_ValueIV, cp_PropData} TCARDParserState;
typedef enum {cp_EncodingNone, cp_EncodingQP, cp_EncodingBase64, cp_Encoding8Bit} TCARDParserEncoding;

typedef struct
{
    /* folding stuff */
    int startOfLine;
    int folding;

    /* inital buffer */
    char *lineBuf;
    int lbSize;
    int lbLen;

    /* line parsing state */
    TCARDParserState state;
    int startOfData;
    CARD_Char **params;
    int nParams;
    int valueEscape;
    TCARDParserEncoding encoding;
    int     decoding;
    char    decodeBuf[4];
    int     dbLen;


    /* user data (duh!) */
    void *userData;

    /* handlers */
    CARD_PropHandler    cardProp;
    CARD_DataHandler    cardData;
} VP;

void vcard_clear_line(VP *vp)
{
    vp->state = cp_PropName;
    vp->lbLen = 0;
    if (vp->lbSize > 1024)
    {
        bfree(vp->lineBuf);
        vp->lineBuf = NULL;
        vp->lbSize = 0;
    };
    vp->startOfData = TRUE;
    vp->nParams = 0;
    vp->valueEscape = FALSE;
    vp->encoding = cp_EncodingNone;
    vp->decoding = FALSE;
    memset(vp->decodeBuf, 0, sizeof(vp->decodeBuf));
    vp->dbLen = 0;
}

/** \fn CARD_Parser CARD_ParserCreate(CARD_Char *encoding)
    \param encoding encoding of the card, currentl not used
    \return CARD_Parser handle or NULL on error
    \retval NULL An error ocurred
    \brief Creates a card parser handle.
    Allocates a handle for a card parser. Free with CARD_ParserFree()
*/
CARD_Parser CARD_ParserCreate(CARD_Char *encoding)
{
    VP *vp = (VP *) bmalloc(sizeof(VP));
    if (! vp)
        return NULL;

    memset(vp, 0, sizeof(VP));

    /* folding stuff */
    vp->startOfLine = TRUE;
    vp->folding = FALSE;

    /* init line stuff */
    vcard_clear_line(vp);

    return vp;
}

/** \fn void CARD_ParserFree(CARD_Parser p)
    \param p Card parser object
    \return nothing
    \brief Deallocates the card parser object.
    Frees the memory associated with a card parser object created by CARD_ParserCreate()
*/
void CARD_ParserFree(CARD_Parser p)
{
    VP *vp = (VP *) p;
    bfree(vp->lineBuf);
    bfree(vp->params);
    bfree(p);
}

int vcard_add_char_to_line(VP *vp, char c)
{
    /* expand buf if neccessary */
    if (vp->lbLen >= vp->lbSize)
    {
        vp->lbSize = vp->lbLen + 128;
        vp->lineBuf = (char *) realloc(vp->lineBuf, vp->lbSize);
        if (vp->lineBuf == NULL)
        {
            /* memory failure, crash & burn */
            vp->lbSize = 0;
            vp->lbLen = 0;
            return FALSE;
        }
    }

    /* place in linebuf */
    vp->lineBuf[vp->lbLen] = c;
    vp->lbLen++;

    return TRUE;
}

int card_add_param(VP *vp, int paramOff)
{
    vp->params = (CARD_Char **) realloc(vp->params, sizeof(CARD_Char *) * (vp->nParams + 1) * 2);

    if (! vp->params)
        return FALSE;

    vp->params[vp->nParams * 2] = (CARD_Char *) paramOff;
    vp->params[vp->nParams * 2 + 1] = NULL;
    vp->nParams++;

    return TRUE;
}

int card_set_paramvalue(VP *vp, int valueOff)
{
    if (! vp->params)
        return FALSE;

    vp->params[(vp->nParams - 1) * 2 + 1] = (CARD_Char *) valueOff;

    return TRUE;
}

int vcard_end_prop(VP *vp)
{
    char *b;
    const CARD_Char *propName;
    int i;
    CARD_Char *encoding;

    if (vp->cardProp)
    {
        /* null terminate the line buf */
        if (! vcard_add_char_to_line(vp, 0))
            return FALSE;

        /* propname */
        b = vp->lineBuf;
        propName = b;

        /* fix up params & values */
        for (i = 0; i < vp->nParams; i++)
        {
            vp->params[i * 2] = vp->lineBuf + (int) vp->params[i * 2];
            if (vp->params[i * 2 + 1])
                vp->params[i * 2 + 1] = vp->lineBuf + (int) vp->params[i * 2 + 1];

            /* check for encodings, needlessly complicated by solo value string
               dumb spec choice */
            encoding = vp->params[i * 2];
            if (bstricmp(encoding, "ENCODING") == 0)
                encoding = vp->params[i * 2 + 1];

            if (encoding)
            {
                if (bstricmp(encoding, "QUOTED-PRINTABLE") == 0)
                    vp->encoding = cp_EncodingQP;
                else if (bstricmp(encoding, "BASE64") == 0)
                    vp->encoding = cp_EncodingBase64;
                else if (bstricmp(encoding, "8BIT") == 0)
                    vp->encoding = cp_Encoding8Bit;
            }
        }

        /* null terminate */
        card_add_param(vp, 0);

        /* do callback */
        vp->cardProp(vp->userData, propName, (const CARD_Char **) vp->params);
    }

    vp->lbLen = 0;
    vp->state = cp_PropData;

    return TRUE;
}

/*  Base64 Chars
    ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/
    Folling func translates a base64 char to its ordinal val
*/

unsigned char Base64ToOrd(char ch)
{
    if (ch >= 'A' && ch <= 'Z')
        return ch - 'A';

    else if (ch >= 'a' && ch <= 'z')
        return 26 + ch - 'a';

    else if (ch >= '0' && ch <= '9')
        return 52 + ch - '0';

    else if (ch == '+')
        return 62;

    else if (ch == '/')
        return 63;

    return 0xFF;
}

int vcard_process_char(VP *vp, char c)
{
    // for qp decoding
    int x;
    // for base64 decoding
    unsigned char ch;
    unsigned char u0;
    unsigned char u1;
    unsigned char u2;
    unsigned char u3;

    switch (vp->state)
    {
    /* --------------------------------------- */
    /* Property Names */
    case cp_PropName:
        /* looking for eof propName delim */
        switch (c)
        {
        case ':':
            vcard_end_prop(vp);
            return TRUE;

        case ';':
            /* we have parameters - null terminate name */
            vp->state = cp_ParamBN;
            return vcard_add_char_to_line(vp, 0);

        case '\r':
            return TRUE;

        case '\n':
            /* throw it all away */
            vcard_clear_line(vp);
            return TRUE;

        default:
            /* just add */
            if (! isspace(c))
                return vcard_add_char_to_line(vp, c);
            else
                return TRUE;
        }
        break;
    /* --------------------------------------- */
    /* Param Names */
    case cp_ParamBN: /* before name */
        switch (c)
        {
        case ':':
            /* error */
            vcard_clear_line(vp);
            return FALSE;

        default:
            if (isspace(c))
                return TRUE; /* ignore */

            /* start name - store as offset into lineBuf, as lineBuf may move when reallocated */
            card_add_param(vp, vp->lbLen);

            /* now in name */
            vp->state = cp_ParamIN;
            return vcard_add_char_to_line(vp, c);
        }
        break;

    case cp_ParamIN: /* in name */
        switch (c)
        {
        case '=':
            /* eof param name, value follows */
            vp->state = cp_ValueBV;
            return vcard_add_char_to_line(vp, 0); /* terminate param name */

        case ';':
            /* eof param, start new param */
            vp->state = cp_ParamBN;
            return vcard_add_char_to_line(vp, 0); /* terminate param name */

        case ':':
            /* eof param, into data */
            vcard_end_prop(vp);
            return TRUE;

        default:
            if (! isspace(c))
                return vcard_add_char_to_line(vp, c);
            else
                return TRUE;
        }
        break;

    /* ----------------------------------------------- */
    /* Param Values */
    case cp_ValueBV: /* before value */
        switch (c)
        {
        case ';':
        case ':':
            /* error ? can we have empty values (ie. "name=;" or "name=:") */
            vcard_clear_line(vp);
            return FALSE;

        default:
            if (isspace(c))
                return TRUE; /* ignore */

            /* start value - store as offset into lineBuf, as lineBuf may move */
            card_set_paramvalue(vp, vp->lbLen);

            /* now in Value */
            vp->state = cp_ValueIV;
            return vcard_add_char_to_line(vp, c);
        }
        break;

    case cp_ValueIV: /* in value */
        switch (c)
        {
        case ';':
            if (vp->valueEscape)
            {
                /* just add to value */
                vp->valueEscape = FALSE;
                return vcard_add_char_to_line(vp, c);
            };

            /* eof value, start new param */
            vp->state = cp_ParamBN;
            return vcard_add_char_to_line(vp, 0); /* terminate value */

        case ':':
            /* eof value, into data */
            vcard_end_prop(vp);
            return TRUE;

        default:
            if (! isspace(c))
            {
                if (c == '\\')
                {
                    if (vp->valueEscape)
                        vp->valueEscape = FALSE; /* unescaping now */
                    else
                    {
                        vp->valueEscape = TRUE; /* escaping now */
                        return TRUE;
                    };
                };
                return vcard_add_char_to_line(vp, c);
            }
            else
                return TRUE;
        }
        break;

    /* ----------------------------------------------- */
    /* Property Data */
    case cp_PropData:
        switch (c)
        {
        case '\r':
            return TRUE;

        case '\n':
            /* terminate data */
            if (vp->cardData)
            {
                /* check for improperly terminated base64 encoding */
                if (vp->encoding == cp_EncodingBase64)
                {
                    if (vp->dbLen > 0)
                    {
                        while (vp->dbLen != 0)
                            vcard_process_char(vp, '=');
                    };
                };

                /* report any data */
                if (vp->lbLen > 0)
                    vp->cardData(vp->userData, vp->lineBuf, vp->lbLen);

                /* terminate data */
                vp->cardData(vp->userData, NULL, 0);
            };

            vcard_clear_line(vp);
            return TRUE;

        default:
            if (vp->startOfData && isspace(c))
                return TRUE;
            else
                vp->startOfData = FALSE;

            /* only store data if we bother reporting it */
            if (vp->cardData)
            {
                switch (vp->encoding)
                {
                case cp_EncodingBase64:
                    if (vp->dbLen < 4)
                    {
                        // add to decode buf
                        // may have trailing ws on lines, so ignore
                        if (! isspace(c))
                        {
                            vp->decodeBuf[vp->dbLen] = c;
                            vp->dbLen++;
                        };
                    };
                    if (vp->dbLen < 4)
                        return TRUE;

                    // we have a quartet
                    u0 = Base64ToOrd(vp->decodeBuf[0]);
                    u1 = Base64ToOrd(vp->decodeBuf[1]);
                    u2 = Base64ToOrd(vp->decodeBuf[2]);
                    u3 = Base64ToOrd(vp->decodeBuf[3]);

                    if (u0 == 0xFF || u1 == 0xFF)
                        return FALSE;
                    ch = ((u0 << 2) | (u1 >> 4));
                    vcard_add_char_to_line(vp, ch);

                    if (vp->decodeBuf[2] != '=')
                    {
                        if (u1 == 0xFF || u2 == 0xFF)
                            return FALSE;
                        ch = (((u1 & 0xF) << 4) | (u2 >> 2));
                        vcard_add_char_to_line(vp, ch);

                        if (vp->decodeBuf[3] != '=')
                        {
                            if (u2 == 0xFF || u3 == 0xFF)
                                return FALSE;
                            ch = (((u2 & 0x3) << 6) | u3);
                            vcard_add_char_to_line(vp, ch);
                        }
                    }

                    memset(vp->decodeBuf, 0, sizeof(vp->decodeBuf));
                    vp->dbLen = 0;
                    return TRUE;

                case cp_EncodingQP:
                    if (vp->decoding)
                    {
                        if (vp->decodeBuf[0] == 0)
                        {
                            vp->decodeBuf[0] = c;
                            return TRUE;
                        }
                        else
                        {
                            /* decoding
                               finished */
                            x = 0;
                            vp->decodeBuf[1] = c;
                            vp->decodeBuf[2] = 0;
                            sscanf(vp->decodeBuf, "%X", &x);
                            c = x;
                            vp->decoding = FALSE;
                            break; /* drops down to add char */
                        }
                    }
                    else if (c == '=')
                    {
                        vp->decoding = TRUE;
                        memset(vp->decodeBuf, 0, sizeof(vp->decodeBuf));
                        vp->dbLen = 0;
                        return TRUE;
                    }
                    break;
                default:
                    break;
                }
                vcard_add_char_to_line(vp, c);
            }
            return TRUE;
        }
    }
    return TRUE;
}

/** \fn int CARD_Parse(CARD_Parser p, const char *s, int len, int isFinal)
    \param p Card parser object
    \param s pointer to binary data, can be null, does not need to be null terminated
    \param len length of data
    \param isFinal wether this is the final chunk of data
    \return success/failure
    \retval 0 a parse error occurred
    \retval non-zero success
    \brief Parses a block of memory.
    Parses the block of memory, the memory does not need to be zero-terminated. isFinal should be set to
    TRUE (non-zero) for the final call so that internal structures can be flushed to the output.
    \note \code CARD_Parse(p, NULL, 0, TRUE);\endcode is an acceptable way of terminating input.
*/
int CARD_Parse(CARD_Parser p, const char *s, int len, int isFinal)
{
    int i;
    char c;
    VP *vp = (VP *) p;

    /* add to lineBuf */
    for (i = 0; i < len; i++)
    {
        c = s[i];

        /* check start of line whitespace/folding */
        if (vp->startOfLine)
        {
            if (c == '\n')
            {
                vp->startOfLine = TRUE;
                vp->folding = FALSE;
                continue; // empty line
            }

            if (! vp->folding && isspace(c))
            {
                /* we are folding, carry on */
                vp->folding = TRUE;
                continue;
            }

            /* no longer start of line */
            vp->startOfLine = FALSE;

            /* either are folding or can terminate last line */
            if (vp->folding)
            {
                /* cancel any qp encoding as it does not continue across folding breaks */
                if (vp->encoding == cp_EncodingQP)
                    vp->decoding = FALSE;
                vcard_process_char(vp, ' ');
            }
            else
                vcard_process_char(vp, '\n');
        }

        if (c == '\n')
        {
            vp->startOfLine = TRUE;
            vp->folding = FALSE;
        }
        else
        {
            /* add to lineBuf */
            if (! vcard_process_char(vp, c))
                return 0;
        }
    }

    if (isFinal)
    {
        /* terminate  */
        vcard_process_char(vp, '\n');
    }

    return TRUE;
}

/* user data */
/** \fn void CARD_SetUserData(CARD_Parser p, void *userData)
    \param p Card parser object
    \param userData value to be set
    \return nothing
    \brief sets the user data value for the callbacks (::CARD_PropHandler, ::CARD_DataHandler)
    This value will be passed to the specified callbacks
*/
void CARD_SetUserData(CARD_Parser p, void *userData)
{
    VP *vp = (VP *) p;
    vp->userData = userData;
}

/** \fn void *CARD_GetUserData(CARD_Parser p)
    \param p Card parser object
    \return current useData value
    \brief returns the user data value for the callbacks (::CARD_PropHandler, ::CARD_DataHandler)
*/
void *CARD_GetUserData(CARD_Parser p)
{
    VP *vp = (VP *) p;
    return vp->userData;
}

/** \fn void CARD_SetPropHandler(CARD_Parser p, CARD_PropHandler cardProp)
    \param p Card parser object
    \param cardProp function pointer for prop event handler
    \return nothing
    \brief Set the callback for vcard properties
*/
void CARD_SetPropHandler(CARD_Parser p, CARD_PropHandler cardProp)
{
    VP *vp = (VP *) p;
    vp->cardProp = cardProp;
}

/** \fn void CARD_SetDataHandler(CARD_Parser p, CARD_DataHandler cardData)
    \param p Card parser object
    \param cardData function pointer for data event handler
    \return nothing
    \brief Set the callback for vcard data
*/
void CARD_SetDataHandler(CARD_Parser p, CARD_DataHandler cardData)
{
    VP *vp = (VP *) p;
    vp->cardData = cardData;
}
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
