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

#ifndef __MEM_MAP__
    #define __MEM_MAP__

    //#define FLASH_XIP
    #define END_ADDR(start, size)  ((start) + (size) - 1)

    //======================================= Memory resources  =======================================

    //================== HPSYS ==================

    // Size
    #define HPSYS_ROM_SIZE      (64*1024)
    #define HPSYS_RETM_SIZE     (128*1024)
    #define HPSYS_DTCM_SIZE     (128*1024)
    #define HPSYS_RAM0_SIZE     (128*1024)
    #define HPSYS_RAM1_SIZE     (128*1024)
    #define HPSYS_RAM2_SIZE     (256*1024)
    #define HPSYS_RAM3_SIZE     (288*1024)

    // Total 800KBytes
    #define HPSYS_RAM_SIZE      (HPSYS_RAM0_SIZE+HPSYS_RAM1_SIZE+HPSYS_RAM2_SIZE+HPSYS_RAM3_SIZE)


    // Address
    #define HPSYS_ROM_BASE      (0x00000000)
    /* RAM0 is DTCM and retention RAM */
    #define HPSYS_RAM0_BASE     (0x20000000)
    #define HPSYS_RETM_BASE     HPSYS_RAM0_BASE
    #define HPSYS_RAM1_BASE     (0x20020000)
    #define HPSYS_RAM2_BASE     (0x20040000)
    #define HPSYS_RAM3_BASE     (0x20080000)

    #define HPSYS_RAM_END       END_ADDR(HPSYS_RAM0_BASE,HPSYS_RAM_SIZE)
    #define HPSYS_RETM_END      (END_ADDR(HPSYS_RETM_BASE, HPSYS_RETM_SIZE))


    // Mailbox
    #define HPSYS_MBOX_BUF_SIZE (2*512)
    #define HPSYS_MBOX_BUF_ADDR (HPSYS_RAM_END+1-HPSYS_MBOX_BUF_SIZE)

    #define HCPU2LCPU_MB_CH2_BUF_START_ADDR  (HPSYS_MBOX_BUF_ADDR)               /* 0x2007FC00 */
    #define HCPU2LCPU_MB_CH2_BUF_SIZE        (512)
    #define HCPU2LCPU_MB_CH2_BUF_END_ADDR    (END_ADDR(HCPU2LCPU_MB_CH2_BUF_START_ADDR, HCPU2LCPU_MB_CH2_BUF_SIZE))

    #define HCPU2LCPU_MB_CH1_BUF_START_ADDR  (HCPU2LCPU_MB_CH2_BUF_END_ADDR+1)   /* 0x2007FE00 */
    #define HCPU2LCPU_MB_CH1_BUF_SIZE        (512)
    #define HCPU2LCPU_MB_CH1_BUF_END_ADDR    (END_ADDR(HCPU2LCPU_MB_CH1_BUF_START_ADDR, HCPU2LCPU_MB_CH1_BUF_SIZE))

    //================== LPSYS ==================
    // Size
    #define LPSYS_ROM_SIZE      (384*1024)
    #define LPSYS_ITCM_SIZE     (16*1024)
    #define LPSYS_DTCM_SIZE     (16*1024)
    #define LPSYS_RAM_SIZE      (96*1024)
    #define LPSYS_EM_SIZE       (32*1024)
    #define LCPU_PATCH_RAM_SIZE              (1 * 1024)
    #define LCPU_HCPU_AUDIO_MEM_SIZE         (1 * 1024)


    // Address, TODO: LPSYS has more SRAM
    #define LPSYS_ROM_BASE      (0x00000000)
    #define LPSYS_ITCM_BASE     (0x003FC000)
    #define LPSYS_DTCM_BASE     (0x203FC000)
    #define LPSYS_SRAM_BASE     (0x20400000)
    #define LPSYS_RAM_BASE      (0x203FC000)
    #define LPSYS_ROM_RAM_BASE  (LPSYS_RAM_BASE + LPSYS_RAM_SIZE + LCPU_PATCH_RAM_SIZE)  //TODO  0x20414400
    #define LPSYS_RAM_END       END_ADDR(LPSYS_RAM_BASE, LPSYS_RAM_SIZE)
    #define LPSYS_RAM_CBUS_BASE (LPSYS_ITCM_BASE)
    #define LPSYS_RAM_CBUS_END  END_ADDR(LPSYS_RAM_CBUS_BASE, LPSYS_RAM_SIZE)
    #define LPSYS_PATCH_BASE    (0x1C004000)  //TODO
    #define LPSYS_PATCH_END     END_ADDR(LPSYS_PATCH_BASE, LCPU_PATCH_TOTAL_SIZE)

    // For LCPU has enough RAM space
    #define LPSYS_PATCH_RAM_BASE    (0x20410000)  //TODO
    #define LPSYS_PATCH_RAM_END     END_ADDR(LPSYS_PATCH_RAM_BASE, LCPU_PATCH_TOTAL_SIZE)


    #define LPSYS_EM_BASE       (0x20418000)
    #define LPSYS_EM_END        END_ADDR(LPSYS_EM_BASE, LPSYS_EM_SIZE)  /* 0x20437FFF */


    // Mailbox
    #define LPSYS_MBOX_BUF_SIZE (2*512)
    #define LCPU2HCPU_MB_CH1_BUF_START_ADDR  (LPSYS_RAM_END - LPSYS_MBOX_BUF_SIZE + 1)  /* 0x20413C00 */
    #define LCPU2HCPU_MB_CH1_BUF_SIZE        (512)
    #define LCPU2HCPU_MB_CH1_BUF_END_ADDR    (END_ADDR(LCPU2HCPU_MB_CH1_BUF_START_ADDR, LCPU2HCPU_MB_CH1_BUF_SIZE))

    #define LCPU2HCPU_MB_CH2_BUF_START_ADDR  (LCPU2HCPU_MB_CH1_BUF_END_ADDR + 1)        /* 0x20413E00 */
    #define LCPU2HCPU_MB_CH2_BUF_SIZE        (512)
    #define LCPU2HCPU_MB_CH2_BUF_END_ADDR    (END_ADDR(LCPU2HCPU_MB_CH2_BUF_START_ADDR, LCPU2HCPU_MB_CH2_BUF_SIZE))

    //================== QSPI Memory ==================

    #define QSPI1_MEM_BASE   (0x10000000)
    #define QSPI2_MEM_BASE   (0x10800000)
    #define QSPI3_MEM_BASE   (0x14000000)
    #define QSPI5_MEM_BASE   (0x1C000000)

    #define MPI1_MEM_BASE   QSPI1_MEM_BASE
    #define MPI2_MEM_BASE   QSPI2_MEM_BASE
    #define MPI3_MEM_BASE   QSPI3_MEM_BASE
    #define MPI5_MEM_BASE   QSPI5_MEM_BASE

    #define HPSYS_MPI_MEM_CBUS_2_SBUS_OFFSET   (0x50000000)

    #define QSPI1_MAX_SIZE      (0x400000)
    #define QSPI2_MAX_SIZE      (0x3c00000)
    #define QSPI3_MAX_SIZE      (0x8000000)
    #define QSPI4_MAX_SIZE      (0x4000000)
    #define QSPI5_MAX_SIZE      (0x4000000)


    // Size
    #define FLASH_TABLE_SIZE            (20*1024)
    #define FLASH_CAL_TABLE_SIZE        (8*1024)
    #define FLASH_BOOT_PATCH_SIZE       (64*1024)

    #define FLASH_BASE_ADDR             (QSPI5_MEM_BASE)
    #define FLASH_TABLE_START_ADDR      (FLASH_BASE_ADDR)
    #define FLASH_TABLE_END_ADDR        (END_ADDR(FLASH_TABLE_START_ADDR, FLASH_TABLE_SIZE))
    #define FLASH_CAL_TABLE_START_ADDR  (FLASH_TABLE_END_ADDR+1)
    #define FLASH_BOOT_PATCH_START_ADDR (0x1C010000)
    #define FLASH_BOOT_PATCH_END_ADDR   (END_ADDR(FLASH_BOOT_PATCH_START_ADDR, FLASH_BOOT_PATCH_SIZE)) /* 0x1C01FFFF */
    //================== Bootloader ==================
    #define FLASH_BOOT_LOADER_START_ADDR (FLASH_BOOT_PATCH_END_ADDR + 1)   /* 0x1C020000 */
    #define FLASH_BOOT_LOADER_SIZE       (128*1024)
    #define FLASH_BOOT_LOADER_END_ADDR   (END_ADDR(FLASH_BOOT_LOADER_START_ADDR, FLASH_BOOT_LOADER_SIZE))

    //================== Flash 1 ==================
    #define FLASH_USER_CODE_START_ADDR   (QSPI1_MEM_BASE)   /* 0x10000000 */
    #define FLASH1_BASE_ADDR            (QSPI1_MEM_BASE)
    #ifdef BSP_QSPI1_MEM_SIZE
        #define FLASH1_SIZE                 (BSP_QSPI1_MEM_SIZE*1024*1024)
    #else
        #define FLASH1_SIZE                 (0)
    #endif

    //================== Flash 2 ==================
    #define FLASH2_BASE_ADDR            (QSPI2_MEM_BASE)
    #ifdef BSP_QSPI2_MEM_SIZE
        #define FLASH2_SIZE                 (BSP_QSPI2_MEM_SIZE*1024*1024)
    #else
        #define FLASH2_SIZE                 (0)
    #endif

    //================== Flash 3 ==================
    #define FLASH3_BASE_ADDR            (QSPI3_MEM_BASE)
    #ifdef BSP_QSPI3_MEM_SIZE
        #define FLASH3_SIZE                 (BSP_QSPI3_MEM_SIZE*1024*1024)
    #else
        #define FLASH3_SIZE                 (0)
    #endif

    //================== Flash 4 ==================
    #define FLASH4_BASE_ADDR            (QSPI4_MEM_BASE)
    #ifdef BSP_QSPI4_MEM_SIZE
        #define FLASH4_SIZE                 (BSP_QSPI4_MEM_SIZE*1024*1024)
    #else
        #define FLASH4_SIZE                 (0)
    #endif

    //================== Flash 5 ==================
    #define FLASH5_BASE_ADDR            (QSPI5_MEM_BASE)
    #ifdef BSP_QSPI5_MEM_SIZE
        #define FLASH5_SIZE                 (BSP_QSPI5_MEM_SIZE*1024*1024)
    #else
        #define FLASH5_SIZE                 (0)
    #endif

    //================== MPI-PSRAM  ==================
    // NOTE : set first psram as psram base and not include QSPI PSRAM (add it ?)
    #if defined (BSP_MPI1_MODE_3) || defined (BSP_MPI1_MODE_4) || defined (BSP_MPI1_MODE_2)|| defined (BSP_MPI1_MODE_5)|| defined (BSP_MPI1_MODE_6)
        #define PSRAM_SIZE                  (BSP_QSPI1_MEM_SIZE*1024*1024)
        #define PSRAM_BASE                  (0x60000000)
    #elif defined (BSP_MPI2_MODE_3)  || defined (BSP_MPI2_MODE_4) || defined (BSP_MPI2_MODE_2)|| defined (BSP_MPI2_MODE_5)|| defined (BSP_MPI2_MODE_6)
        #define PSRAM_SIZE                  (BSP_QSPI2_MEM_SIZE*1024*1024)
        #define PSRAM_BASE                  (0x60800000)
    #elif defined(BSP_MPI3_MODE_3) || defined (BSP_MPI3_MODE_4)
        #define PSRAM_SIZE                  (BSP_QSPI3_MEM_SIZE*1024*1024)
        #define PSRAM_BASE                  (0x64000000)
    #else   // Not define PSRAM, use a default value or assert?
        #define PSRAM_SIZE                  (0)
        #define PSRAM_BASE                  (0x60000000)
    #endif  //(BSP_MPI_MODE_3 || BSP_MPI_MODE_4)

    //================== QSPI-PSRAM  ==================
    #define PSRAM2_BASE_ADDR            (QSPI2_MEM_BASE)
    #ifdef BSP_QSPI2_MEM_SIZE
        #define PSRAM2_SIZE                 (BSP_QSPI2_MEM_SIZE*1024*1024)
    #else
        #define PSRAM2_SIZE                 (0)
    #endif

    #define PSRAM3_BASE_ADDR            (QSPI3_MEM_BASE)
    #ifdef BSP_QSPI3_MEM_SIZE
        #define PSRAM3_SIZE                 (BSP_QSPI3_MEM_SIZE*1024*1024)
    #else
        #define PSRAM3_SIZE                 (0)
    #endif

    #define PSRAM4_BASE_ADDR            (QSPI4_MEM_BASE)
    #ifdef BSP_QSPI4_MEM_SIZE
        #define PSRAM4_SIZE                 (BSP_QSPI4_MEM_SIZE*1024*1024)
    #else
        #define PSRAM4_SIZE                 (0)
    #endif
    #define HPSYS_RAM_IN_ITCM(addr)         (0)


    //======================================= Code mapping =======================================


    //================= Boot loader ===============
    // Size
    #define BOOTLOADER_CODE_SIZE         (64*1024)
    #define BOOTLOADER_RAM_DATA_SIZE     (64*1024) //reserved 4 byte for LCPU_BOOT_ADDR
    #define BOOTLOADER_PATCH_CODE_SIZE   (64*1024) // Bootloader patch code in RAM size
    #define BOOTLOADER_PATCH_DATA_SIZE   (64*1024)

    // Address
    #define BOOTLOADER_CODE_START_ADDR          (HPSYS_ROM_BASE)                        // Bootloader in ROM start from 0
    #define BOOTLOADER_CODE_END_ADDR            (END_ADDR(BOOTLOADER_CODE_START_ADDR, BOOTLOADER_CODE_SIZE))
    #define BOOTLOADER_RAM_DATA_START_ADDR      (HPSYS_RAM1_BASE)              // 0x20020000
    #define BOOTLOADER_RAM_DATA_END_ADDR        (END_ADDR(BOOTLOADER_RAM_DATA_START_ADDR, BOOTLOADER_RAM_DATA_SIZE))

    // Bootloader Patch
    #define BOOTLOADER_PATCH_CODE_ADDR          (BOOTLOADER_RAM_DATA_END_ADDR + 1)      //0x20010000, Bootloader patch code in RAM start from 2nd 64k bytes of RAM
    #define BOOTLOADER_PATCH_CODE_END_ADDR      (END_ADDR(BOOTLOADER_PATCH_CODE_ADDR, BOOTLOADER_PATCH_CODE_SIZE))
    #define BOOTLOADER_PATCH_DATA_ADDR          (BOOTLOADER_PATCH_CODE_END_ADDR + 1)    //0x20020000, Bootloader patch data in RAM start from 3th 64k bytes of RAM
    #define BOOTLOADER_PATCH_DATA_END_ADDR      (END_ADDR(BOOTLOADER_PATCH_DATA_ADDR, BOOTLOADER_PATCH_DATA_SIZE))

    #if BOOTLOADER_PATCH_DATA_END_ADDR >= (HPSYS_RAM0_BASE+HPSYS_RAM_SIZE)
        #error "bootloader ram overflow"
    #endif


    //================= HP subsys ROM =================
    // Size
    #define HCPU_CODE_SIZE                  (HPSYS_ROM_SIZE)
    #define HCPU_RO_DATA_SIZE               (16*1024)
    #define HCPU_RAM_DATA_SIZE              (HPSYS_RAM_SIZE - HCPU_RO_DATA_SIZE - HPSYS_MBOX_BUF_SIZE - HPSYS_RETM_SIZE)
    #define HCPU_CODE_START_ADDR            0 //(BOOTLOADER_CODE_END_ADDR+1)
    #define HCPU_CODE_END_ADDR              (END_ADDR(HCPU_CODE_START_ADDR, HCPU_CODE_SIZE))
    #define HCPU_RAM_DATA_START_ADDR        (HPSYS_RAM1_BASE)         /* 0x20020000 */
    #define HCPU_RAM_DATA_END_ADDR          (END_ADDR(HCPU_RAM_DATA_START_ADDR, HCPU_RAM_DATA_SIZE))
    #define HCPU_RO_DATA_START_ADDR         (HCPU_RAM_DATA_END_ADDR+1)
    #define HCPU_RO_DATA_END_ADDR           (END_ADDR(HCPU_RO_DATA_START_ADDR, HCPU_RO_DATA_SIZE))
    #define HCPU_LCPU_CODE_START_ADDR       (LCPU_ITCM_ADDR_2_HCPU_ADDR(LPSYS_ITCM_BASE))

    //================= HP subsys Flash1 =================
    #ifdef BSP_USING_DFU_COMPRESS
        // DFU Size
        #define DFU_FLASH_CODE_SIZE             (256*1024)
        #define DFU_RES_FLASH_CODE_SIZE         (640*1024)
        #define HCPU_FLASH_CODE_SIZE            (896*1024)

        // DFU Address
        #define DFU_FLASH_CODE_START_ADDR       FLASH_USER_CODE_START_ADDR
        #define DFU_FLASH_CODE_END_ADDR         (END_ADDR(DFU_FLASH_CODE_START_ADDR, DFU_FLASH_CODE_SIZE))  /* 0x1005FFFF */

        #define DFU_RES_FLASH_CODE_START_ADDR   (DFU_FLASH_CODE_END_ADDR + 1)  /* 0x10060000 */
        #define DFU_RES_FLASH_CODE_END_ADDR     (END_ADDR(DFU_RES_FLASH_CODE_START_ADDR, DFU_RES_FLASH_CODE_SIZE))  /* 0x100FFFFF */

        #define HCPU_FLASH_CODE_START_ADDR      (DFU_RES_FLASH_CODE_END_ADDR + 1)  /* 0x10100000 */
        #define HCPU_FLASH_CODE_END_ADDR        (END_ADDR(HCPU_FLASH_CODE_START_ADDR, HCPU_FLASH_CODE_SIZE))  /* 0x101DFFFF */

    #else
        // Size
        #define HCPU_FLASH_CODE_SIZE            (1024*1024)

        // Address
        #define HCPU_FLASH_CODE_START_ADDR      FLASH_USER_CODE_START_ADDR  /* 0x10020000 */
        #define HCPU_FLASH_CODE_END_ADDR        (END_ADDR(HCPU_FLASH_CODE_START_ADDR, HCPU_FLASH_CODE_SIZE))  /* 0x100FFFFF */
    #endif

    #define PSRAM_DATA_START_ADDR               (0x60200000)
    #define PSRAM_DATA_SIZE                     (2*1024*1024)
    #if PSRAM_DATA_START_ADDR <= HCPU_FLASH_CODE_END_ADDR
        #error "wrong config"
    #endif


    // Size
    #define HCPU_FLASH_IMG_SIZE             (4096*1024)
    #define HCPU_FLASH_FONT_SIZE            (4096*3*1024)

    // Address
    #define HCPU_FLASH_IMG_START_ADDR       (0x10100000)   /* 0x10100000 */
    #define HCPU_FLASH_IMG_END_ADDR         (END_ADDR(HCPU_FLASH_IMG_START_ADDR, HCPU_FLASH_IMG_SIZE))  /*  0x105FFFFF */
    #define HCPU_FLASH_FONT_START_ADDR      (HCPU_FLASH_IMG_END_ADDR + 1) /* 0x10600000 */
    #define HCPU_FLASH_FONT_END_ADDR        (END_ADDR(HCPU_FLASH_FONT_START_ADDR, HCPU_FLASH_FONT_SIZE))

    //================= HP subsys Flash2 =================
    // Size
    #define HCPU_FLASH2_IMG_SIZE            (4096*1024)
    #define HCPU_FLASH2_FONT_SIZE           (4096*3*1024)
    #define HCPU_FLASH2_IMG_UPGRADE_SIZE    (HCPU_FLASH_IMG_SIZE/4)
    #define HCPU_FLASH2_FONT_UPGRADE_SIZE   (HCPU_FLASH2_FONT_SIZE/4)

    // Address
    #define HCPU_FLASH2_IMG_START_ADDR              (FLASH2_BASE_ADDR)  /* 0x64000000 */
    #define HCPU_FLASH2_IMG_END_ADDR                (END_ADDR(HCPU_FLASH2_IMG_START_ADDR, HCPU_FLASH2_IMG_SIZE))  /*  0x643FFFFF */
    #define HCPU_FLASH2_FONT_START_ADDR             (HCPU_FLASH2_IMG_END_ADDR + 1)  /* 0x64400000 */
    #define HCPU_FLASH2_FONT_END_ADDR               (END_ADDR(HCPU_FLASH2_FONT_START_ADDR, HCPU_FLASH2_FONT_SIZE))
    #define HCPU_FLASH2_IMG_UPGRADE_START_ADDR      (HCPU_FLASH2_FONT_END_ADDR + 1)
    #define HCPU_FLASH2_IMG_UPGRADE_END_ADDR        (END_ADDR(HCPU_FLASH2_IMG_UPGRADE_START_ADDR, HCPU_FLASH2_IMG_UPGRADE_SIZE))
    #define HCPU_FLASH2_FONT_UPGRADE_START_ADDR     (HCPU_FLASH2_IMG_UPGRADE_END_ADDR + 1)
    #define HCPU_FLASH2_FONT_UPGRADE_END_ADDR       (END_ADDR(HCPU_FLASH2_FONT_UPGRADE_START_ADDR, HCPU_FLASH2_FONT_UPGRADE_SIZE))

    //================= LP subsys ======================


    // Size
    #define LCPU_ROM_CODE_SIZE               (LPSYS_ROM_SIZE)
    #define LCPU_RAM_CODE_SIZE               (60 * 1024)
    #define LCPU_MBOX_SIZE                   (LPSYS_MBOX_BUF_SIZE)
    #define LCPU_ROM_RAM_SIZE                (12 * 1024)
    #define LCPU_PATCH_TOTAL_SIZE            (64 * 1024)
    #define LCPU_PATCH_RECORD_SIZE           (256)
    #define LCPU_RAM_DATA_SIZE               (LPSYS_RAM_SIZE - LCPU_MBOX_SIZE - LCPU_RAM_CODE_SIZE)
    #define LCPU_FLASH_CODE_SIZE             (512*1024)
    #define LCPU_PATCH_RAM_EXTRA_SIZE              (512)


    /***************** LPSYS RAM MEM MAP   **********************
    LPSYS_RAM       PATCH_RAM       ROM_RAM         LCPU_HCPU_AUDIO_RAM         EM
    SIZE         96*1024        1*1024          12*1024         3*1024                  32*1024
    LPSYS_RAM_SIZE  LCPU_PATCH_RAM_SIZE LCPU_ROM_RAM_SIZE  LCPU_HCPU_AUDIO_MEM_SIZE
    start_addr  0x203FC000      0x20414000      0x20414400    0x20417400        0x20418000
    */

    // Address in C-Bus
    #define LCPU_ROM_CODE_START_ADDR     (LPSYS_ROM_BASE)
    #define LCPU_RAM_CODE_START_ADDR     (LPSYS_RAM_CBUS_BASE)
    #define LCPU_RAM_CODE_END_ADDR       (END_ADDR(LCPU_RAM_CODE_START_ADDR, LCPU_RAM_CODE_SIZE))
    //#define LCPU_ROM_RAM_START_ADDR      (LPSYS_RAM_END + 1)
    #define LCPU_ROM_RAM_START_ADDR      (LPSYS_ROM_RAM_BASE)
    #define LCPU_ROM_RAM_END_ADDR        (END_ADDR(LCPU_ROM_RAM_START_ADDR + LCPU_ROM_RAM_SIZE))
    #define LCPU_PATCH_START_ADDR        (LPSYS_PATCH_BASE) //(LPSYS_PATCH_BASE-0x20000000)
    #define LCPU_PATCH_START_ADDR_S      (LPSYS_PATCH_BASE)

    #define LCPU_PATCH_ROM_RAM_START_ADDR        (LPSYS_PATCH_RAM_BASE-0x20000000) //(LPSYS_PATCH_BASE-0x20000000)
    #define LCPU_PATCH_ROM_RAM_START_ADDR_S      (LPSYS_PATCH_RAM_BASE)

    #define LCPU_PATCH_END_ADDR          (END_ADDR(LCPU_PATCH_START_ADDR, LCPU_PATCH_TOTAL_SIZE))
    #define LCPU_PATCH_RAM_START_ADDR_S  (LPSYS_RAM_BASE + LPSYS_RAM_SIZE)
    #define LCPU_PATCH_RAM_START_ADDR    (LCPU_PATCH_RAM_START_ADDR_S)
    #define LCPU_PATCH_RAM_END_ADDR      (END_ADDR(LCPU_PATCH_RAM_START_ADDR + LCPU_PATCH_RAM_SIZE))
    #define LCPU_AUDIO_MEM_START_ADDR    (0x2041F800)
    #define LCPU_AUDIO_MEM_START_ADDR_H  (LCPU_AUDIO_MEM_START_ADDR)
    #define LCPU_AUDIO_MEM_END_ADDR      (END_ADDR(LCPU_AUDIO_MEM_START_ADDR + LCPU_HCPU_AUDIO_MEM_SIZE))
    #define LCPU_PATCH_RAM_EXTRA_START_ADDR         (0x417200)
    #define LCPU_PATCH_RAM_EXTRA_START_ADDR_S       (0x20417200)


    //Address in S-Bus
    #define LCPU_RAM_CODE_START_ADDR_S   (LPSYS_RAM_BASE)
    #define LCPU_RAM_DATA_START_ADDR     (LCPU_RAM_CODE_START_ADDR_S + LCPU_RAM_CODE_SIZE)
    #define LCPU_RAM_DATA_END_ADDR       (END_ADDR(LCPU_RAM_DATA_START_ADDR, LCPU_RAM_DATA_SIZE))
    #define LCPU_PATCH_RECORD_ADDR       (LCPU_ROM_RAM_START_ADDR+LCPU_ROM_RAM_SIZE)

    #define LCPU_FLASH_CODE_START_ADDR      (FLASH_BOOT_LOADER_END_ADDR + 1)  /* 0x1C040000 */
    #define LCPU_FLASH_CODE_END_ADDR        (END_ADDR(LCPU_FLASH_CODE_START_ADDR, LCPU_FLASH_CODE_SIZE))  /* 0x12040000 */


    #define FLASH_PART_NAME(id)       FLASH_PART##id##_NAME
    #define FLASH_PART_DEVICE(id)     FLASH_PART##id##_DEVICE
    #define FLASH_PART_BASE_ADDR(id)  FLASH_PART##id##_BASE_ADDR
    #define FLASH_PART_OFFSET(id)     FLASH_PART##id##_OFFSET
    #define FLASH_PART_SIZE(id)       FLASH_PART##id##_SIZE


    #if (LCPU2BCPU_MB_CH1_BUF_END_ADDR - LPSYS_RAM_BASE) >= LPSYS_RAM_SIZE
        #error "LCPU RAM overflow"
    #endif

    //======================================= Customize =======================================
    #ifdef CUSTOM_MEM_MAP
        #ifdef SOLUTION_WATCH
            #include "flash_map.h"
        #else
            #include "custom_mem_map.h"
        #endif
    #endif /* CUSTOM_MEM_MAP */

#endif  /* __MEM_MAP__ */

/**
    @brief  Factory configuration saved on flash
*/
#define SYSCFG_FACTORY_ADDRESS  (FLASH_TABLE_START_ADDR + 0xE000)
#define AUTO_FLASH_MAC_ADDRESS  (FLASH_TABLE_START_ADDR + 0xE000)
#define SYSCFG_FACTORY_SIZE     0x2000      /*!< Max configuration size*/


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
