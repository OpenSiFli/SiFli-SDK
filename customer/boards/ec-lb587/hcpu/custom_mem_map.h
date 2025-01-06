/**
  ******************************************************************************
  * @file   mem_map.h
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


#ifndef __CUSTOM_MEM_MAP__
#define __CUSTOM_MEM_MAP__

#ifdef USING_PARTITION_TABLE
    #include "ptab.h"
#endif /* USING_PARTITION_TABLE */

#define FAL_PART_TABLE \
{ \
    {FAL_PART_MAGIC_WORD,       "dfu",      NOR_FLASH3_DEV_NAME,    KVDB_DFU_REGION_OFFSET,   KVDB_DFU_REGION_SIZE, 0}, \
    {FAL_PART_MAGIC_WORD,       "ble",      NOR_FLASH3_DEV_NAME,    KVDB_BLE_REGION_OFFSET,   KVDB_BLE_REGION_SIZE, 0}, \
}
#ifdef SF32LB58X_3SCO
    #undef  LPSYS_RAM_SIZE
    #define LPSYS_RAM_SIZE      (512*1024)
    /*Following are copy from \example\watch_demo\project\watch_l_micro_58\linker_scripts\custom_mem_map.h in 52x_rom_a4*/

    #undef LPSYS_ROM_BASE
    #undef LPSYS_ROM_SIZE
    #define LPSYS_ROM_BASE          LPSYS_ITCM_BASE
    //#define LPSYS_ROM_SIZE          0x60000
    #define LPSYS_ROM_SIZE          0x80000


    #undef LCPU_RAM_DATA_START_ADDR
    #undef LCPU_RAM_DATA_SIZE
    #define LCPU_RAM_DATA_START_ADDR        LCPU_ROM_RAM_START_ADDR
    #define LCPU_RAM_DATA_SIZE              (LCPU_ROM_RAM_SIZE+LCPU_ROM_HEAP_SIZE)

    // Start from 0x20400000
    #undef  LCPU_ROM_RAM_SIZE
    #undef  KE_ENV_BUF_SIZE
    #undef  LCPU_PATCH_BUF_SIZE

    //#define LCPU_ROM_RAM_SIZE                                  (0x1C00)
    #define LCPU_ROM_RAM_SIZE                                  (0x2C00)
    #define LCPU_ROM_HEAP_SIZE                                 (0x1000)
    #define KE_ENV_BUF_SIZE                                    (0x5400)
    #define LCPU_PATCH_BUF_SIZE                                (0x0)

    // Start from 0x204F0000
    #undef  EM_BUF_SIZE
    #undef  LCPU_AUDIO_MEM_SIZE
    #undef  LCPU_HCPU_AUDIO_MEM_SIZE
    #undef  KE_MSG_BUF_SIZE
    #undef  NVDS_BUF_SIZE
    #define EM_BUF_SIZE                                        (0xB000)
    #define LCPU_AUDIO_MEM_SIZE                                (0x400)
    #define KE_MSG_BUF_SIZE                                    (0x2800)
    #define NVDS_BUF_SIZE                                      (0x200)
    #define LCPU_HCPU_AUDIO_MEM_SIZE                           (LCPU_AUDIO_MEM_SIZE)
    #define KE_LOG_BUF_SIZE                                    (0x1000)

    #undef  LCPU_ROM_RAM_START_ADDR
    #undef  KE_ENV_BUF_START_ADDR
    #undef  LCPU_PATCH_BUF_START_ADDR
    #define LCPU_ROM_RAM_START_ADDR                            (0x20480000)
    #define KE_ENV_BUF_START_ADDR                              (LCPU_ROM_RAM_START_ADDR+LCPU_RAM_DATA_SIZE)
    #define LCPU_PATCH_BUF_START_ADDR                          (KE_ENV_BUF_START_ADDR+KE_ENV_BUF_SIZE)

    #undef  KE_ENV_BUF_OFFSET
    #undef  LCPU_PATCH_BUF_OFFSET
    #define KE_ENV_BUF_OFFSET                                  (KE_ENV_BUF_START_ADDR-LPSYS_SRAM_BASE)
    #define LCPU_PATCH_BUF_OFFSET                              (LCPU_PATCH_BUF_START_ADDR-LPSYS_SRAM_BASE)

    #undef  EM_BUF_START_ADDR
    #undef  LCPU_AUDIO_MEM_START_ADDR
    #undef  KE_MSG_BUF_START_ADDR
    #undef  NVDS_BUF_START_ADDR
    #undef  LCPU2HCPU_MB_CH1_BUF_START_ADDR
    #undef  LCPU2HCPU_MB_CH2_BUF_START_ADDR
    #define EM_BUF_START_ADDR                                  (0x204F0000)
    #define LCPU_AUDIO_MEM_START_ADDR                          (EM_BUF_START_ADDR + EM_BUF_SIZE)
    #define KE_MSG_BUF_START_ADDR                              (LCPU_AUDIO_MEM_START_ADDR+LCPU_AUDIO_MEM_SIZE)
    #define NVDS_BUF_START_ADDR                                (KE_MSG_BUF_START_ADDR+KE_MSG_BUF_SIZE)
    #define LCPU2HCPU_MB_CH1_BUF_START_ADDR                    (NVDS_BUF_START_ADDR+NVDS_BUF_SIZE)
    #define LCPU2HCPU_MB_CH2_BUF_START_ADDR                    (LCPU2HCPU_MB_CH1_BUF_START_ADDR + LCPU2HCPU_MB_CH1_BUF_SIZE)
    #define KE_LOG_BUF_START_ADDR                              (LCPU2HCPU_MB_CH2_BUF_START_ADDR + LCPU2HCPU_MB_CH2_BUF_SIZE)

    #undef  EM_BUF_OFFSET
    #undef  LCPU_AUDIO_MEM_OFFSET
    #undef  KE_MSG_BUF_OFFSET
    #undef  NVDS_BUF_OFFSET
    #define EM_BUF_OFFSET                                      (EM_BUF_START_ADDR-LPSYS_SRAM_BASE)
    #define LCPU_AUDIO_MEM_OFFSET                              (LCPU_AUDIO_MEM_START_ADDR-LPSYS_SRAM_BASE)
    #define KE_MSG_BUF_OFFSET                                  (KE_MSG_BUF_START_ADDR-LPSYS_SRAM_BASE)
    #define NVDS_BUF_OFFSET                                    (NVDS_BUF_START_ADDR-LPSYS_SRAM_BASE)

    // LCPU_HCPU_AUDIO_RAM
    #define LCPU_AUDIO_MEM_START_ADDR_H  (LCPU_AUDIO_MEM_START_ADDR)
    #define LCPU_AUDIO_MEM_END_ADDR      (END_ADDR(LCPU_AUDIO_MEM_START_ADDR + LCPU_HCPU_AUDIO_MEM_SIZE))

#elif defined(SF32LB52X_58)
    /*Following are copy from \example\watch_demo\project\watch_l_micro_58\linker_scripts\custom_mem_map.h in 52x_rom_a4*/

    #undef LPSYS_ROM_BASE
    #undef LPSYS_ROM_SIZE
    #define LPSYS_ROM_BASE          LPSYS_ITCM_BASE
    #define LPSYS_ROM_SIZE          0x60000
    //#define LPSYS_ROM_SIZE          0x80000


    #undef LCPU_RAM_DATA_START_ADDR
    #undef LCPU_RAM_DATA_SIZE
    #define LCPU_RAM_DATA_START_ADDR        LCPU_ROM_RAM_START_ADDR
    #define LCPU_RAM_DATA_SIZE              (LCPU_ROM_RAM_SIZE+LCPU_ROM_HEAP_SIZE)

    // Start from 0x20400000
    #undef  LCPU_ROM_RAM_SIZE
    #undef  KE_ENV_BUF_SIZE
    #undef  LCPU_PATCH_BUF_SIZE

    //#define LCPU_ROM_RAM_SIZE                                  (0x1C00)
    #define LCPU_ROM_RAM_SIZE                                  (0x2C00)
    #define LCPU_ROM_HEAP_SIZE                                 (0x0C00)
    #define KE_ENV_BUF_SIZE                                    (0x2400)
    #define LCPU_PATCH_BUF_SIZE                                (0x3000)

    // Start from 0x204F0000
    #undef  EM_BUF_SIZE
    #undef  LCPU_AUDIO_MEM_SIZE
    #undef  LCPU_HCPU_AUDIO_MEM_SIZE
    #undef  KE_MSG_BUF_SIZE
    #undef  NVDS_BUF_SIZE
    #define EM_BUF_SIZE                                        (0x6000)
    #define LCPU_AUDIO_MEM_SIZE                                (0x400)
    #define KE_MSG_BUF_SIZE                                    (0x1800)
    #define NVDS_BUF_SIZE                                      (0x200)
    #define LCPU_HCPU_AUDIO_MEM_SIZE                           (LCPU_AUDIO_MEM_SIZE)

    #undef  LCPU_ROM_RAM_START_ADDR
    #undef  KE_ENV_BUF_START_ADDR
    #undef  LCPU_PATCH_BUF_START_ADDR
    #define LCPU_ROM_RAM_START_ADDR                            (0x20480000)
    #define KE_ENV_BUF_START_ADDR                              (LCPU_ROM_RAM_START_ADDR+LCPU_RAM_DATA_SIZE)
    #define LCPU_PATCH_BUF_START_ADDR                          (KE_ENV_BUF_START_ADDR+KE_ENV_BUF_SIZE)

    #undef  KE_ENV_BUF_OFFSET
    #undef  LCPU_PATCH_BUF_OFFSET
    #define KE_ENV_BUF_OFFSET                                  (KE_ENV_BUF_START_ADDR-LPSYS_SRAM_BASE)
    #define LCPU_PATCH_BUF_OFFSET                              (LCPU_PATCH_BUF_START_ADDR-LPSYS_SRAM_BASE)

    #undef  EM_BUF_START_ADDR
    #undef  LCPU_AUDIO_MEM_START_ADDR
    #undef  KE_MSG_BUF_START_ADDR
    #undef  NVDS_BUF_START_ADDR
    #undef  LCPU2HCPU_MB_CH1_BUF_START_ADDR
    #undef  LCPU2HCPU_MB_CH2_BUF_START_ADDR
    #define EM_BUF_START_ADDR                                  (0x204F0000)
    #define LCPU_AUDIO_MEM_START_ADDR                          (EM_BUF_START_ADDR + EM_BUF_SIZE)
    #define KE_MSG_BUF_START_ADDR                              (LCPU_AUDIO_MEM_START_ADDR+LCPU_AUDIO_MEM_SIZE)
    #define NVDS_BUF_START_ADDR                                (KE_MSG_BUF_START_ADDR+KE_MSG_BUF_SIZE)
    #define LCPU2HCPU_MB_CH1_BUF_START_ADDR                    (NVDS_BUF_START_ADDR+NVDS_BUF_SIZE)
    #define LCPU2HCPU_MB_CH2_BUF_START_ADDR                    (LCPU2HCPU_MB_CH1_BUF_START_ADDR + LCPU2HCPU_MB_CH1_BUF_SIZE)

    #undef  EM_BUF_OFFSET
    #undef  LCPU_AUDIO_MEM_OFFSET
    #undef  KE_MSG_BUF_OFFSET
    #undef  NVDS_BUF_OFFSET
    #define EM_BUF_OFFSET                                      (EM_BUF_START_ADDR-LPSYS_SRAM_BASE)
    #define LCPU_AUDIO_MEM_OFFSET                              (LCPU_AUDIO_MEM_START_ADDR-LPSYS_SRAM_BASE)
    #define KE_MSG_BUF_OFFSET                                  (KE_MSG_BUF_START_ADDR-LPSYS_SRAM_BASE)
    #define NVDS_BUF_OFFSET                                    (NVDS_BUF_START_ADDR-LPSYS_SRAM_BASE)

    // LCPU_HCPU_AUDIO_RAM
    #define LCPU_AUDIO_MEM_START_ADDR_H  (LCPU_AUDIO_MEM_START_ADDR)
    #define LCPU_AUDIO_MEM_END_ADDR      (END_ADDR(LCPU_AUDIO_MEM_START_ADDR + LCPU_HCPU_AUDIO_MEM_SIZE))

#else
    #undef LCPU2HCPU_MB_CH1_BUF_START_ADDR
    #define LCPU2HCPU_MB_CH1_BUF_START_ADDR      (0x204C3C00)
#endif
#endif  /* __MEM_MAP__ */





/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
