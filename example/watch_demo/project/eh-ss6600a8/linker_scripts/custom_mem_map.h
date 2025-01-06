

/**
  ******************************************************************************
  * @file   custom_mem_map.h
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
#ifndef __CUSTOM_MEM_MAP_H__
#define __CUSTOM_MEM_MAP_H__
#define NOR_FLASH1_DEV_NAME             "flash1"
#define NOR_FLASH2_DEV_NAME             "flash2"

#define _FLASH1_SIZE_                   (BSP_QSPI1_MEM_SIZE * 1024 *1024)
#define _FLASH2_SIZE                    (FLASH2_SIZE)


/**
    the following marcro is flash1 layout size.
*/
#define _DFU_MNG_CODE_SIZE              ( 256 * 1024)
#define _CODE_SIZE_                     (1536 * 1024)
#define _FS_ROOT_SIZE                   ( 384 * 1024)
#define _NVM_DFU_SIZE_                  (  16 * 1024)
#define _NVM_BLE_SIZE_                  (  16 * 1024)
#define _NVM_MSG_SIZE_                  (  96 * 1024)
#define _NVM_APP_SIZE_                  ( 256 * 1024)

/**
    the following marcro is flash2 layout size.
*/
#define _IMG_SIZE_                      (6 * 512 * 1024)
#define _FONT_SIZE_                     (17 * 512 * 1024)
#define _FS_EX_SIZE_                    (FLASH2_SIZE - _IMG_SIZE_ - _FONT_SIZE_ - _DFU_DOWNLOAD_BUFFER_SIZE_-_ULOG_SIZE_) //reserved 3.1MB

#ifdef ULOG_BACKEND_USING_TSDB
    #define _ULOG_SIZE_                     ( 1 * 512 * 1024)
#else
    #define _ULOG_SIZE_                     0
#endif
#define _DFU_DOWNLOAD_BUFFER_SIZE_      ( 900 * 1024)


/**
    the following is the falsh1 layout.
*/

/**
    0x0000..0x02fff : ftab
    0x3000..0x20000 : reserved
*/
//dfu_mng_code: dfu upgrade code.
#define FLASH_PART0_NAME                "dfu_mng_code"
#define FLASH_PART0_DEVICE              NOR_FLASH1_DEV_NAME
#define FLASH_PART0_OFFSET              (0x20000)
#define FLASH_PART0_BASE_ADDR           (FLASH_BASE_ADDR + FLASH_PART0_OFFSET)
#define FLASH_PART0_SIZE                _DFU_MNG_CODE_SIZE

//solution_code: solution code size, including HCPU + LCPU
#define FLASH_PART1_NAME                "solution_code"
#define FLASH_PART1_DEVICE              NOR_FLASH1_DEV_NAME
#ifdef BSP_USING_DFU_COMPRESS
    #define FLASH_PART1_OFFSET              FLASH_PART0_OFFSET + _DFU_MNG_CODE_SIZE
#else
    #define FLASH_PART1_OFFSET              FLASH_PART0_OFFSET
#endif

#define FLASH_PART1_BASE_ADDR           (FLASH_BASE_ADDR + FLASH_PART1_OFFSET)
#define FLASH_PART1_SIZE                _CODE_SIZE_

//root: for file system. to store installed watchface as well as multi-language
#define FLASH_PART2_NAME                "root"
#define FLASH_PART2_DEVICE              NOR_FLASH1_DEV_NAME
#define FLASH_PART2_OFFSET              (FLASH_PART1_OFFSET + FLASH_PART1_SIZE)
#define FLASH_PART2_BASE_ADDR           (FLASH_BASE_ADDR + FLASH_PART2_OFFSET)
#define FLASH_PART2_SIZE                _FS_ROOT_SIZE

//msg: to store app message & notification
#define FLASH_PART3_NAME                "msg"
#define FLASH_PART3_DEVICE              NOR_FLASH1_DEV_NAME
#define FLASH_PART3_OFFSET              (FLASH_PART2_OFFSET + FLASH_PART2_SIZE)
#define FLASH_PART3_BASE_ADDR           (FLASH_BASE_ADDR + FLASH_PART3_OFFSET)
#define FLASH_PART3_SIZE                _NVM_MSG_SIZE_

//app: to store app history & setting & alarm...
#define FLASH_PART4_NAME                "app"
#define FLASH_PART4_DEVICE              NOR_FLASH1_DEV_NAME
#define FLASH_PART4_OFFSET              (FLASH_PART3_OFFSET + FLASH_PART3_SIZE)
#define FLASH_PART4_BASE_ADDR           (FLASH_BASE_ADDR + FLASH_PART4_OFFSET)
#define FLASH_PART4_SIZE                _NVM_APP_SIZE_

//dfu: to store dfu nvm
#define FLASH_PART5_NAME                "dfu"
#define FLASH_PART5_DEVICE              NOR_FLASH1_DEV_NAME
#define FLASH_PART5_OFFSET              (FLASH_PART4_OFFSET + FLASH_PART4_SIZE)
#define FLASH_PART5_BASE_ADDR           (FLASH_BASE_ADDR + FLASH_PART5_OFFSET)
#define FLASH_PART5_SIZE                _NVM_DFU_SIZE_

//ble: to store ble nvm
#define FLASH_PART6_NAME                "ble"
#define FLASH_PART6_DEVICE              NOR_FLASH1_DEV_NAME
#define FLASH_PART6_OFFSET              (FLASH_PART5_OFFSET + FLASH_PART5_SIZE)
#define FLASH_PART6_BASE_ADDR           (FLASH_BASE_ADDR + FLASH_PART6_OFFSET)
#define FLASH_PART6_SIZE                _NVM_BLE_SIZE_

/**
    the following is the falsh2 layout. note: the item of flash 2 can merge into flash 1.
*/

//app_img: to store application images.
#define FLASH_PART10_NAME               "app_img"
#define FLASH_PART10_DEVICE             NOR_FLASH2_DEV_NAME
#define FLASH_PART10_OFFSET             (0x0)
#define FLASH_PART10_BASE_ADDR          (FLASH2_BASE_ADDR + FLASH_PART10_OFFSET)
#define FLASH_PART10_SIZE               _IMG_SIZE_

//app_img: to store application font.
#define FLASH_PART11_NAME               "font"
#define FLASH_PART11_DEVICE
#define FLASH_PART11_OFFSET             (FLASH_PART10_OFFSET + FLASH_PART10_SIZE)
#define FLASH_PART11_BASE_ADDR          (FLASH2_BASE_ADDR + FLASH_PART11_OFFSET)
#define FLASH_PART11_SIZE               _FONT_SIZE_


//ex: for file system. to store packs which will be installed and resouces.
#define FLASH_PART12_NAME               "ex"
#define FLASH_PART12_DEVICE             NOR_FLASH2_DEV_NAME
#define FLASH_PART12_OFFSET             (FLASH_PART11_OFFSET + FLASH_PART11_SIZE)
#define FLASH_PART12_BASE_ADDR          (FLASH2_BASE_ADDR + FLASH_PART12_OFFSET)
#define FLASH_PART12_SIZE               _FS_EX_SIZE_


//ts_area: for logout to flash
#define FLASH_PART19_NAME               "ts_area"
#define FLASH_PART19_DEVICE             NOR_FLASH2_DEV_NAME
#define FLASH_PART19_OFFSET             (_FLASH2_SIZE - _DFU_DOWNLOAD_BUFFER_SIZE_ - _ULOG_SIZE_)
#define FLASH_PART19_BASE_ADDR          (FLASH2_BASE_ADDR + FLASH_PART19_OFFSET)
#define FLASH_PART19_SIZE               _ULOG_SIZE_

//dfu_dl: for file system. to store packs which will be installed and resouces.
#define FLASH_PART20_NAME               "dfu_download_buffer"
#define FLASH_PART20_DEVICE             NOR_FLASH2_DEV_NAME
#define FLASH_PART20_OFFSET             (_FLASH2_SIZE - _DFU_DOWNLOAD_BUFFER_SIZE_)
#define FLASH_PART20_BASE_ADDR          (FLASH2_BASE_ADDR + FLASH_PART20_OFFSET)
#define FLASH_PART20_SIZE               _DFU_DOWNLOAD_BUFFER_SIZE_


/**
    the following defined for factory test.
*/
#define FLASH_IMG_BASE_ADDR             FLASH_PART10_BASE_ADDR
#define FLASH_FONT_BASE_ADDR            FLASH_PART11_BASE_ADDR
#define FLASH_OTA_BASE_ADDR             FLASH_PART20_BASE_ADDR


/**
    the following is FAL PART declaration.
*/

#ifdef ULOG_BACKEND_USING_TSDB
#define FAL_PART_TABLE                                                  \
        {                                                               \
        FAL_PART_DEF(2),                                                \
        FAL_PART_DEF(3),                                                \
        FAL_PART_DEF(4),                                                \
        FAL_PART_DEF(5),                                                \
        FAL_PART_DEF(6),                                                \
        FAL_PART_DEF(12),                                               \
        FAL_PART_DEF(19)                                                \
        }

#else

#define FAL_PART_TABLE                                                  \
        {                                                               \
        FAL_PART_DEF(2),                                                \
        FAL_PART_DEF(3),                                                \
        FAL_PART_DEF(4),                                                \
        FAL_PART_DEF(5),                                                \
        FAL_PART_DEF(6),                                                \
        FAL_PART_DEF(12)                                                \
        }

#endif

/**
    the following will be used in sct file.
*/

#undef  DFU_FLASH_CODE_START_ADDR
#undef  DFU_FLASH_CODE_SIZE
#undef  DFU_FLASH_CODE_END_ADDR
#define DFU_FLASH_CODE_START_ADDR       FLASH_PART_BASE_ADDR(0)
#define DFU_FLASH_CODE_SIZE             FLASH_PART_SIZE(0)
#define DFU_FLASH_CODE_END_ADDR         (END_ADDR(DFU_FLASH_CODE_START_ADDR, DFU_FLASH_CODE_SIZE))


#undef  HCPU_FLASH_CODE_START_ADDR
#undef  HCPU_FLASH_CODE_SIZE
#undef  HCPU_FLASH_CODE_END_ADDR
#define HCPU_FLASH_CODE_START_ADDR      FLASH_PART_BASE_ADDR(1) /* 0x10020000 or 0x10060000 if dfu enabled. */
#define HCPU_FLASH_CODE_SIZE            FLASH_PART_SIZE(1)
#define HCPU_FLASH_CODE_END_ADDR        (END_ADDR(HCPU_FLASH_CODE_START_ADDR, HCPU_FLASH_CODE_SIZE))  /* 0x100FFFFF */

#undef  HCPU_FLASH2_IMG_START_ADDR
#undef  HCPU_FLASH2_IMG_SIZE
#undef  HCPU_FLASH2_IMG_END_ADDR
#define HCPU_FLASH2_IMG_START_ADDR      FLASH_PART_BASE_ADDR(10)     /* 0x18000000 */
#define HCPU_FLASH2_IMG_SIZE            FLASH_PART_SIZE(10)
#define HCPU_FLASH2_IMG_END_ADDR        (END_ADDR(HCPU_FLASH2_IMG_START_ADDR, HCPU_FLASH2_IMG_SIZE))  /*  0x105FFFFF */

#undef  HCPU_FLASH2_FONT_START_ADDR
#undef  HCPU_FLASH2_FONT_SIZE
#undef  HCPU_FLASH2_FONT_END_ADDR
#define HCPU_FLASH2_FONT_START_ADDR     FLASH_PART_BASE_ADDR(11) /* 0x18400000 */
#define HCPU_FLASH2_FONT_SIZE           FLASH_PART_SIZE(11)
#define HCPU_FLASH2_FONT_END_ADDR       (END_ADDR(HCPU_FLASH2_FONT_START_ADDR, HCPU_FLASH2_FONT_SIZE))

#undef  DFU_RES_FLASH_CODE_START_ADDR
#undef  DFU_RES_FLASH_CODE_END_ADDR
#undef  DFU_RES_FLASH_CODE_SIZE
#define DFU_RES_FLASH_CODE_START_ADDR   FLASH_PART_BASE_ADDR(20)    /* 0x19F1F000 in 32M flash or 0x18F1F000 */
#define DFU_RES_FLASH_CODE_SIZE         FLASH_PART_SIZE(20)
#define DFU_RES_FLASH_CODE_END_ADDR     (END_ADDR(DFU_RES_FLASH_CODE_START_ADDR, DFU_RES_FLASH_CODE_SIZE))


#endif /* __CUSTOM_MEM_MAP_H__ */






/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
