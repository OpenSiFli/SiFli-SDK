/**
  ******************************************************************************
  * @file   bts2_app_sys.c
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

#ifdef CFG_MS
    #include "windows.h"
#endif

U32 bt_multibyte_to_widechar(const U32 CodePage,
                             const U32 dwFlags,
                             void     *lpMultiByteStr, //S8
                             U32       cbMultiByte,
                             void     *lpWideCharStr,  //S16
                             U32       cchWideChar)
{
#ifdef CFG_MS
    return MultiByteToWideChar(CodePage,
                               dwFlags,
                               (LPCSTR)lpMultiByteStr,
                               cbMultiByte,
                               (LPWSTR)lpWideCharStr,
                               cchWideChar);
#endif
    return 0;
}

U32 bt_widechar_to_multibyte(const U32 CodePage,
                             const U32 dwFlags,
                             void     *lpWideCharStr, //LPCWSTR S16
                             U32       cchWideChar,
                             void     *lpMultiByteStr,//S8 LPCSTR
                             U32       cbMultiByte,
                             void     *lpDefaultChar, //S8 LPCSTR
                             void     *lpUsedDefaultChar)
{
#ifdef CFG_MS
    return  WideCharToMultiByte(CodePage,
                                dwFlags,
                                (LPCWSTR)lpWideCharStr,
                                cchWideChar,
                                (LPSTR) lpMultiByteStr,
                                cbMultiByte,
                                (LPCSTR)lpDefaultChar,
                                (LPBOOL)lpUsedDefaultChar);
#endif
    return 0;
}

U32 bt_get_file_size(void *stream)
{
    U32 curpos;
    U32 length = 0;

    curpos = ftell((FILE *)stream);
    fseek((FILE *)stream, 0L, SEEK_END);
    length = ftell((FILE *)stream);
    fseek((FILE *)stream, curpos, SEEK_SET);

    return length;
}
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
