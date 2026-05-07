/*
 ******************************************************************************
 * @file   app_module.h
 * @author Sifli software development team
 ******************************************************************************
 */
/*
 * @attention
 * Copyright (c) 2019 - 2024,  Sifli Technology
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

/*
 * SDK multilingual migration note:
 * The original solution-side app_module.h depends on the complete solution
 * resource-path macro system such as APP_LANG_PATH and APP_CFG_PATH.
 * The SDK does not provide that full path framework yet, so only the
 * multilingual-related subset is enabled for now. The original path mapping is
 * preserved below in a disabled block for later migration.
 */

#ifndef __APP_MODULE_H_
#define __APP_MODULE_H_

#include "app_lang.h"

#if 0
/*
 * Original solution-side multilingual path definitions:
 * #define LANG_INSTALLER_PATH APP_LANG_INSTALLER_PATH
 * #define OTA_FILE_LIST_PATH  APP_CFG_PATH
 * #define APP_LANG_INSTALLER_PATH APP_LANG_PATH "installer/"
 */
#define LANG_INSTALLER_PATH                         APP_LANG_INSTALLER_PATH
#define OTA_FILE_LIST_PATH                          APP_CFG_PATH
#define APP_LANG_INSTALLER_PATH                     APP_LANG_PATH               "installer/"
#endif

#ifndef APP_LANG_INSTALLER_PATH
    #define APP_LANG_INSTALLER_PATH "/ex/resource/lang/installer"
#endif

#ifndef LANG_INSTALLER_PATH
    #define LANG_INSTALLER_PATH APP_LANG_INSTALLER_PATH
#endif

#ifndef MAX_LANG_NAME_LEN
    #define MAX_LANG_NAME_LEN 32
#endif

#ifndef GUI_APP_NAME_MAX_LEN
    #define GUI_APP_NAME_MAX_LEN 32
#endif

#endif /* __APP_MODULE_H_ */
