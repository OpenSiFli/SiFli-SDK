/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __MEM_MAP__
#define __MEM_MAP__

//#define FLASH_XIP
#define END_ADDR(start, size)  ((start) + (size) - 1)

//======================================= Memory resources  =======================================

//================== HPSYS ==================

// Size
#define HPSYS_ROM_SIZE      (64*1024)
#define HPSYS_RAM0_SIZE     (128*1024)      // DTCM
#define HPSYS_RETM_SIZE     (512*1024)
#define HPSYS_DTCM_SIZE     (128*1024)
#define HPSYS_RAM1_SIZE     (128*1024)
#define HPSYS_RAM2_SIZE     (256*1024)

// Total 512KBytes
#define HPSYS_RAM_SIZE      (HPSYS_RAM0_SIZE+HPSYS_RAM1_SIZE+HPSYS_RAM2_SIZE)

// Address
#define HPSYS_ROM_BASE      (0x00000000)
/* RAM0 is DTCM and retention RAM */
#define HPSYS_RAM0_BASE     (0x20000000)
#define HPSYS_RETM_BASE     HPSYS_RAM0_BASE
#define HPSYS_RAM1_BASE     (0x20020000)
#define HPSYS_RAM2_BASE     (0x20040000)

#define HPSYS_RAM_END       END_ADDR(HPSYS_RAM0_BASE,HPSYS_RAM_SIZE)

// Mailbox
#define HPSYS_MBOX_BUF_SIZE (4*512)
#define HPSYS_MBOX_BUF_ADDR (HPSYS_RAM_END+1-HPSYS_MBOX_BUF_SIZE)


#define HCPU2ACPU_IPC_QUEUE_BUF_START_ADDR  (HPSYS_MBOX_BUF_ADDR)               /* 0x2007F800 */
#define HCPU2ACPU_IPC_QUEUE_BUF_SIZE        (512)
#define HCPU2ACPU_IPC_QUEUE_BUF_END_ADDR    (END_ADDR(HCPU2ACPU_IPC_QUEUE_BUF_START_ADDR, HCPU2ACPU_IPC_QUEUE_BUF_SIZE))

#define ACPU2HCPU_IPC_QUEUE_BUF_START_ADDR  (HCPU2ACPU_IPC_QUEUE_BUF_END_ADDR+1)   /* 0x2007FA00 */
#define ACPU2HCPU_IPC_QUEUE_BUF_SIZE        (512)
#define ACPU2HCPU_IPC_QUEUE_BUF_END_ADDR    (END_ADDR(ACPU2HCPU_IPC_QUEUE_BUF_START_ADDR, ACPU2HCPU_IPC_QUEUE_BUF_SIZE))

#define HCPU2LCPU_MB_CH2_BUF_START_ADDR  (ACPU2HCPU_IPC_QUEUE_BUF_END_ADDR+1)    /* 0x2007FC00 */
#define HCPU2LCPU_MB_CH2_BUF_SIZE        (512)
#define HCPU2LCPU_MB_CH2_BUF_END_ADDR    (END_ADDR(HCPU2LCPU_MB_CH2_BUF_START_ADDR, HCPU2LCPU_MB_CH2_BUF_SIZE))

#define HCPU2LCPU_MB_CH1_BUF_START_ADDR  (HCPU2LCPU_MB_CH2_BUF_END_ADDR+1)   /* 0x2007FE00 */
#define HCPU2LCPU_MB_CH1_BUF_SIZE        (512)
#define HCPU2LCPU_MB_CH1_BUF_END_ADDR    (END_ADDR(HCPU2LCPU_MB_CH1_BUF_START_ADDR, HCPU2LCPU_MB_CH1_BUF_SIZE))

//================== LPSYS ==================
// Size
#define LPSYS_ROM_SIZE      (384*1024)
#define LPSYS_SRAM0_SIZE    (32*1024)
#define LPSYS_SRAM1_SIZE    (32*1024)
/** LPSYS SRAM size in total (SRAM0+SRAM1) */
#define LPSYS_SRAM_TOTAL_SIZE (LPSYS_SRAM0_SIZE+LPSYS_SRAM1_SIZE)
/** LPSYS EM size, space in SRAM1 excluding space used by software */
#define LPSYS_EM_SIZE       (24*1024)   // RAM 1
/** available LCPU RAM size from SW perspective */
#define LPSYS_RAM_SIZE      (24*1024)

// Address, TODO: LPSYS has more SRAM
#define LPSYS_ROM_BASE      (0x00000000)
#define LPSYS_RAM_BASE      (0x20400000)
#define LPSYS_SRAM_BASE     (LPSYS_RAM_BASE)
#define LPSYS_RAM_END       END_ADDR(LPSYS_RAM_BASE, LPSYS_RAM_SIZE)

#define LPSYS_RAM_CBUS_BASE (0x00400000)
#define LPSYS_RAM_CBUS_END  END_ADDR(LPSYS_RAM_CBUS_BASE, LPSYS_RAM_SIZE)
#define LPSYS_EM_BASE       (0x20408000)
#define LPSYS_EM_END        END_ADDR(LPSYS_EM_BASE, LPSYS_EM_SIZE)  /* 0x2040FFFF */

// Mailbox
#define LPSYS_MBOX_BUF_SIZE (2*512)

#define LCPU2HCPU_MB_CH2_BUF_START_ADDR  (LCPU2HCPU_MB_CH1_BUF_END_ADDR + 1)        /* 0x20405E00 */
#define LCPU2HCPU_MB_CH2_BUF_SIZE        (512)
#define LCPU2HCPU_MB_CH2_BUF_END_ADDR    (END_ADDR(LCPU2HCPU_MB_CH2_BUF_START_ADDR, LCPU2HCPU_MB_CH2_BUF_SIZE))

//================== QSPI Memory ==================

#define QSPI1_MEM_BASE   (0x10000000)
#define QSPI2_MEM_BASE   (0x12000000)
#define QSPI3_MEM_BASE   (0x14000000)

#define MPI1_MEM_BASE   QSPI1_MEM_BASE
#define MPI2_MEM_BASE   QSPI2_MEM_BASE
#define MPI3_MEM_BASE   QSPI3_MEM_BASE

#define HPSYS_MPI_MEM_CBUS_2_SBUS_OFFSET   (0x50000000)

#define QSPI1_MAX_SIZE      (0x2000000)
#define QSPI2_MAX_SIZE      (0x2000000)
#define QSPI3_MAX_SIZE      (0xC000000) // S-BUS max size is 0x3C000000

//================== SDMMC Memory Card ==================
#define SDMMC1_MEM_BASE     (0x64000000)
#define SDMMC2_MEM_BASE     (0xA0000000)


// Size
#define FLASH_TABLE_SIZE            (20*1024)
#define FLASH_CAL_TABLE_SIZE        (8*1024)
#define FLASH_BOOT_PATCH_SIZE       (64*1024)

#define FLASH_BASE_ADDR             (QSPI1_MEM_BASE)
#define FLASH_TABLE_START_ADDR      (FLASH_BASE_ADDR)
#define FLASH_TABLE_END_ADDR        (END_ADDR(FLASH_TABLE_START_ADDR, FLASH_TABLE_SIZE))
#define FLASH_CAL_TABLE_START_ADDR  (FLASH_TABLE_END_ADDR+1)
#define FLASH_BOOT_PATCH_START_ADDR (0x10010000)
#define FLASH_BOOT_PATCH_END_ADDR   (END_ADDR(FLASH_BOOT_PATCH_START_ADDR, FLASH_BOOT_PATCH_SIZE)) /* 0x1001FFFF */
//================== Bootloader ==================
#define FLASH_BOOT_LOADER_START_ADDR (FLASH_BOOT_PATCH_END_ADDR + 1)   /* 0x10020000 */
#define FLASH_BOOT_LOADER_SIZE       (128*1024)
#define FLASH_BOOT_LOADER_END_ADDR   (END_ADDR(FLASH_BOOT_LOADER_START_ADDR, FLASH_BOOT_LOADER_SIZE))

//================== Flash 1 ==================
#define FLASH_USER_CODE_START_ADDR   (FLASH_BOOT_LOADER_END_ADDR+1)   /* 0x10040000 */

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

//================== MPI-PSRAM  ==================
// NOTE : set first psram as psram base and not include QSPI PSRAM (add it ?)
#ifdef BSP_USING_PSRAM1
    #define PSRAM_SIZE                  (BSP_QSPI1_MEM_SIZE*1024*1024)
    #define PSRAM_BASE                  (0x60000000)
#else   // Not define PSRAM, use a default value or assert?
    #define PSRAM_SIZE                  (0)
    #define PSRAM_BASE                  (0x60000000)
#endif  // only mpi1 can use psram for 57x

//================== QSPI-PSRAM  ==================
#define PSRAM2_BASE_ADDR            (QSPI2_MEM_BASE)
#ifdef BSP_QSPI2_MEM_SIZE
    #define PSRAM2_SIZE                 (BSP_QSPI2_MEM_SIZE*1024*1024)
#else
    #define PSRAM2_SIZE                 (0)
#endif
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
#define BOOTLOADER_RAM_DATA_START_ADDR      (HPSYS_RAM0_BASE)              // 0x20000000
#define BOOTLOADER_RAM_DATA_END_ADDR        (END_ADDR(BOOTLOADER_RAM_DATA_START_ADDR, BOOTLOADER_RAM_DATA_SIZE))

// Bootloader Patch
#define BOOTLOADER_PATCH_CODE_ADDR          (0x20050000)      // Bootloader patch code
#if BOOTLOADER_PATCH_CODE_ADDR <= BOOTLOADER_RAM_DATA_END_ADDR
    #error "bootloader patch code overlapped with bootloader ram data"
#endif
#define BOOTLOADER_PATCH_CODE_END_ADDR      (END_ADDR(BOOTLOADER_PATCH_CODE_ADDR, BOOTLOADER_PATCH_CODE_SIZE))
#define BOOTLOADER_PATCH_DATA_ADDR          (BOOTLOADER_PATCH_CODE_END_ADDR + 1)    //0x20020000, Bootloader patch data in RAM start from 3th 64k bytes of RAM
#define BOOTLOADER_PATCH_DATA_END_ADDR      (END_ADDR(BOOTLOADER_PATCH_DATA_ADDR, BOOTLOADER_PATCH_DATA_SIZE))

#if BOOTLOADER_PATCH_DATA_END_ADDR >= (HPSYS_RAM0_BASE+HPSYS_RAM_SIZE)
    #error "bootloader ram overflow"
#endif

//================= HP subsys ROM =================
// Size
#define HCPU_CODE_SIZE                  (HPSYS_ROM_SIZE)
#define HCPU_CUSTOM_CONFIG_SIZE         (256)
#define HCPU_RO_DATA_SIZE               (0*1024)
#define HCPU_RAM_DATA_SIZE              (HPSYS_RAM_SIZE - HCPU_RO_DATA_SIZE - HPSYS_MBOX_BUF_SIZE - HCPU_CUSTOM_CONFIG_SIZE)
#define HCPU_CODE_START_ADDR            0 //(BOOTLOADER_CODE_END_ADDR+1)
#define HCPU_CODE_END_ADDR              (END_ADDR(HCPU_CODE_START_ADDR, HCPU_CODE_SIZE))
#define HCPU_RAM_DATA_START_ADDR        (HPSYS_RAM0_BASE)         /* 0x20000000 */
#define HCPU_RAM_DATA_END_ADDR          (END_ADDR(HCPU_RAM_DATA_START_ADDR, HCPU_RAM_DATA_SIZE))
#define HCPU_RO_DATA_START_ADDR         (HCPU_RAM_DATA_END_ADDR+1)
#define HCPU_RO_DATA_END_ADDR           (END_ADDR(HCPU_RO_DATA_START_ADDR, HCPU_RO_DATA_SIZE))

#define HCPU_CUSTOM_CONFIG_START_ADDR   (HCPU_RO_DATA_END_ADDR+1)  /*0x2007fb04*/
#define HCPU_CUSTOM_CONFIG_END_ADDR     (END_ADDR(HCPU_CUSTOM_CONFIG_START_ADDR, HCPU_CUSTOM_CONFIG_SIZE))

#define HCPU_LCPU_CODE_START_ADDR       (LCPU_RAM_CODE_START_ADDR_S)

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
#define HCPU_FLASH2_IMG_START_ADDR              (FLASH2_BASE_ADDR)  /* 0x12000000 */
#define HCPU_FLASH2_IMG_END_ADDR                (END_ADDR(HCPU_FLASH2_IMG_START_ADDR, HCPU_FLASH2_IMG_SIZE))  /*  0x123FFFFF */
#define HCPU_FLASH2_FONT_START_ADDR             (HCPU_FLASH2_IMG_END_ADDR + 1)  /* 0x12400000 */
#define HCPU_FLASH2_FONT_END_ADDR               (END_ADDR(HCPU_FLASH2_FONT_START_ADDR, HCPU_FLASH2_FONT_SIZE))
#define HCPU_FLASH2_IMG_UPGRADE_START_ADDR      (HCPU_FLASH2_FONT_END_ADDR + 1)
#define HCPU_FLASH2_IMG_UPGRADE_END_ADDR        (END_ADDR(HCPU_FLASH2_IMG_UPGRADE_START_ADDR, HCPU_FLASH2_IMG_UPGRADE_SIZE))
#define HCPU_FLASH2_FONT_UPGRADE_START_ADDR     (HCPU_FLASH2_IMG_UPGRADE_END_ADDR + 1)
#define HCPU_FLASH2_FONT_UPGRADE_END_ADDR       (END_ADDR(HCPU_FLASH2_FONT_UPGRADE_START_ADDR, HCPU_FLASH2_FONT_UPGRADE_SIZE))

//================= LP subsys ======================


// Size
#define LCPU_ROM_CODE_SIZE               (LPSYS_ROM_SIZE)


#define LCPU_RAM_CODE_SIZE               (6 * 1024)
#define LCPU_PATCH_TOTAL_SIZE            (8 * 1024)
#define LCPU_PATCH_RECORD_SIZE           (256)
#define LCPU_HCPU_AUDIO_MEM_SIZE         (1 * 1024)
#define LCPU_MBOX_SIZE                   (LPSYS_MBOX_BUF_SIZE)
#define LCPU_RAM_DATA_SIZE               (LPSYS_RAM_SIZE - LCPU_MBOX_SIZE - LCPU_RAM_CODE_SIZE)


/***************** LPSYS RAM MEM MAP   **********************
LPSYS_RAM                 PATCH                         EM       LCPU_HCPU_AUDIO_RAM        ROM_RAM
SIZE        24*1024                 8*1024             24*1024         1*1024               7*1024
4k_ram_code heap mailbox     patch_record  patch
start_addr  0x0x20400000           0x20406000          0x20408000      0x2040E000           0x2040E400
*/

// Address in C-Bus
#define LCPU_ROM_CODE_START_ADDR     (LPSYS_ROM_BASE)
#define LCPU_RAM_CODE_START_ADDR     (LPSYS_RAM_CBUS_BASE)


#define LCPU_ROM_RAM_START_ADDR                            ((LPSYS_SRAM_BASE) + (LCPU_ROM_RAM_OFFSET))            /* 0x20400000 */
#define LCPU_ROM_RAM_SIZE                                  (0x00001C00)
#define LCPU_ROM_RAM_OFFSET                                (0x00000000)
#define LCPU_ROM_RAM_END_ADDR        (END_ADDR(LCPU_ROM_RAM_START_ADDR,LCPU_ROM_RAM_SIZE))

#define LCPU_ROM_CONFIG_START_ADDR                         (((LPSYS_SRAM_BASE) + (LCPU_ROM_CONFIG_OFFSET)))      /* 0x204013800 */
#define LCPU_ROM_CONFIG_SIZE                               (0x00000100)
#define LCPU_ROM_CONFIG_OFFSET                             (0x00013800)

#define LCPU2HCPU_MB_CH1_BUF_START_ADDR                    ((LPSYS_SRAM_BASE) + (LCPU2HCPU_MB_CH1_BUF_OFFSET))   /* 0x20413A00 */
#define LCPU2HCPU_MB_CH1_BUF_SIZE                          (0x00000200)
#define LCPU2HCPU_MB_CH1_BUF_OFFSET                        (0x00013A00)
#define LCPU2HCPU_MB_CH1_BUF_END_ADDR                      (END_ADDR(LCPU2HCPU_MB_CH1_BUF_START_ADDR, LCPU2HCPU_MB_CH1_BUF_SIZE))

#define NVDS_BUF_START_ADDR                                ((LPSYS_SRAM_BASE) + (NVDS_BUF_OFFSET))               /* 0x20413600 */
#define NVDS_BUF_SIZE                                      (0x00000200)
#define NVDS_BUF_OFFSET                                    (0x00013600)

#define KE_LOG_BUF_START_ADDR                              ((LPSYS_SRAM_BASE) + (KE_LOG_BUF_OFFSET))             /* 0x20402800 */
#define KE_LOG_BUF_SIZE                                    (0x00000000)
#define KE_LOG_BUF_OFFSET                                  (0x00002800)

#define KE_ENV_BUF_START_ADDR                              ((LPSYS_SRAM_BASE) + (KE_ENV_BUF_OFFSET))             /* 0x20402800 */
#define KE_ENV_BUF_SIZE                                    (0x00002400)
#define KE_ENV_BUF_OFFSET                                  (0x00002800)

#define KE_MSG_BUF_START_ADDR                              ((LPSYS_SRAM_BASE) + (KE_MSG_BUF_OFFSET))             /* 0x20404C00 */
#define KE_MSG_BUF_SIZE                                    (0x00003400)
#define KE_MSG_BUF_OFFSET                                  (0x00004C00)

#define LCPU_PATCH_BUF_START_ADDR                          ((LPSYS_SRAM_BASE) + (LCPU_PATCH_BUF_OFFSET))         /* 0x20410000 */
#define LCPU_PATCH_BUF_SIZE                                (0x00003600)
#define LCPU_PATCH_BUF_OFFSET                              (0x00010000)

#define EM_BUF_START_ADDR                                  ((LPSYS_SRAM_BASE) + (EM_BUF_OFFSET))                 /* 0x20408000 */
#define EM_BUF_SIZE                                        (0x00008000)
#define EM_BUF_OFFSET                                      (0x00008000)

#define LCPU_AUDIO_MEM_START_ADDR                          ((LPSYS_SRAM_BASE) + (LCPU_AUDIO_MEM_OFFSET))         /* 0x20413C00 */
#define LCPU_AUDIO_MEM_SIZE                                (0x00000400)
#define LCPU_AUDIO_MEM_OFFSET                              (0x00013C00)
#define LCPU_AUDIO_MEM_END_ADDR                            (END_ADDR(LCPU_AUDIO_MEM_START_ADDR, LCPU_AUDIO_MEM_SIZE))

#define LCPU_RF_CONFIG_START_ADDR                          ((LPSYS_SRAM_BASE) + (LCPU_RF_CONFIG_OFFSET))         /* 0x20413900 */
#define LCPU_RF_CONFIG_SIZE                                (0x00000100)
#define LCPU_RF_CONFIG_OFFSET                              (0x00013900)


#define LCPU_RAM_CODE_START_ADDR_S   (LPSYS_RAM_BASE)
#define LCPU_RAM_DATA_START_ADDR     (LCPU_RAM_CODE_START_ADDR_S + LCPU_RAM_CODE_SIZE)
#define LCPU_RAM_DATA_END_ADDR       (END_ADDR(LCPU_RAM_DATA_START_ADDR, LCPU_RAM_DATA_SIZE))


// PATCH
#define LCPU_PATCH_START_ADDR_S      (0x20406000)
#define LCPU_PATCH_START_ADDR        (LCPU_PATCH_START_ADDR_S-0x20000000)
#define LCPU_PATCH_END_ADDR          (END_ADDR(LCPU_PATCH_START_ADDR, LCPU_PATCH_TOTAL_SIZE))
#define LCPU_PATCH_RECORD_ADDR       (LCPU_PATCH_START_ADDR_S+LCPU_PATCH_TOTAL_SIZE-LCPU_PATCH_RECORD_SIZE)


/*******************************************************************************************
 * @defgroup EFUSE_BITMAP_Definition EFUSE Bitmap Definition
 * @brief  EFUSE Bitmap Definition
 *
 * _OFFSET: offset in bit across all banks, i.e. 0~255 is the first bank, 256~511 is the second bank
 * _SIZE: size in bit
 *
 * @{
 *******************************************************************************************/
/** UID offset  */
#define EFUSE_UID_OFFSET        0
/** UID size  */
#define EFUSE_UID_SIZE          128
#define EFUSE_UID_BYTE_SIZE     (EFUSE_UID_SIZE >> 3)
/** SSEN offset */
#define EFUSE_SSEN_OFFSET      224
/** SSEN size */
#define EFUSE_SSEN_SIZE        2
/** Package ID offset */
#define EFUSE_PKGID_OFFSET      228
/** Package ID size  */
#define EFUSE_PKGID_SIZE        8
#define EFUSE_SWDDIS_OFFSET     236
#define EFUSE_SWDDIS_SIZE       2
#define EFUSE_SECEN_OFFSET      238
#define EFUSE_SECEN_SIZE        2
#define EFUSE_IDSEL_OFFSET      240
#define EFUSE_IDSEL_SIZE        2
#define EFUSE_PINRST_OFFSET     242
#define EFUSE_PINRST_SIZE       2
#define EFUSE_BANK1_RDDIS_OFFSET      244
#define EFUSE_BANK1_RDDIS_SIZE        2
#define EFUSE_BANK1_PGMDIS_OFFSET     246
#define EFUSE_BANK1_PGMDIS_SIZE       2
#define EFUSE_BANK2_RDDIS_OFFSET      248
#define EFUSE_BANK2_RDDIS_SIZE        2
#define EFUSE_BANK2_PGMDIS_OFFSET     250
#define EFUSE_BANK2_PGMDIS_SIZE       2

#define EFUSE_ROOTKEY_OFFSET          512
#define EFUSE_ROOTKEY_SIZE            256
#define EFUSE_ROOTKEY_BYTE_SIZE       (EFUSE_ROOTKEY_SIZE >> 3)

/**
 * @}
 */

/*******************************************************************************************
 * @defgroup SSEN_Definition SSEN Definition
 * @brief  SSEN Definition
 * @{
 *******************************************************************************************/
/**  SSEN_LDO18 Position */
#define SSEN_LDO18_Pos        (0U)
/**  SSEN_LDO18 Mask */
#define SSEN_LDO18_Msk        (0x1UL << SSEN_LDO18_Pos)
/**  SSEN_VDD33_LDO2 Position */
#define SSEN_VDD33_LDO2_Pos   (1U)
/**  SSEN_VDD33_LDO2 Mask */
#define SSEN_VDD33_LDO2_Msk   (0x1UL << SSEN_VDD33_LDO2_Pos)

/**
 * @}
 */

/*******************************************************************************************
 * @defgroup PKGID_Definition Package ID Definition
 * @brief  Package ID Definition
 * @{
 *******************************************************************************************/
/**  Boot Device Position */
#define PKGID_BOOT_DEVICE_Pos        (0U)
/**  Boot Device Mask */
#define PKGID_BOOT_DEVICE_Msk        (0x3UL << PKGID_BOOT_DEVICE_Pos)
/**  LDO18 Enable Position */
#define PKGID_LDO18_EN_Pos           (2U)
/**  LDO18 Enable Mask */
#define PKGID_LDO18_EN_Msk           (0x1UL << PKGID_LDO18_EN_Pos)
/**  LDO33 Enable Position */
#define PKGID_LDO33_EN_Pos           (3U)
/**  LDO33 Enable Mask */
#define PKGID_LDO33_EN_Msk           (0x1UL << PKGID_LDO33_EN_Pos)
/**  MPI1 PSRAM Type Position */
#define PKGID_MPI1_PSRAM_TYPE_Pos    (4U)
/**  MPI1 PSRAM Type Mask */
#define PKGID_MPI1_PSRAM_TYPE_Msk    (0x3UL << PKGID_MPI1_PSRAM_TYPE_Pos)
/**  MPI2 PSRAM Type Position */
#define PKGID_MPI2_PSRAM_TYPE_Pos    (6U)
/**  MPI2 PSRAM Type Mask */
#define PKGID_MPI2_PSRAM_TYPE_Msk    (0x3UL << PKGID_MPI2_PSRAM_TYPE_Pos)


/**
 * @}
 */


/*******************************************************************************************
 * @defgroup PKGID_BOOT_DEVICE_Definition Package ID Boot Device Definition
 * @brief  Package ID Boot Device Definition
 * @{
 *******************************************************************************************/
/** Boot Device is internal SB NOR Flash connected to MPI2 */
#define PKGID_BOOT_DEVICE_MPI2                0
/** Boot Device is internal SA NOR Flash type 1 connected to MPI1 */
#define PKGID_BOOT_DEVICE_MPI1_TYPE1          1
/** Boot Device is internal SA NOR Flash type 2 connected to MPI1 */
#define PKGID_BOOT_DEVICE_MPI1_TYPE2          2
/** Boot Device is external storage */
#define PKGID_BOOT_DEVICE_EXT                 3

/**
 * @}
 */


//======================================= Customize =======================================
#define FLASH_PART_NAME(id)       FLASH_PART##id##_NAME
#define FLASH_PART_DEVICE(id)     FLASH_PART##id##_DEVICE
#define FLASH_PART_BASE_ADDR(id)  FLASH_PART##id##_BASE_ADDR
#define FLASH_PART_OFFSET(id)     FLASH_PART##id##_OFFSET
#define FLASH_PART_SIZE(id)       FLASH_PART##id##_SIZE

/**
@brief  Factory configuration saved on flash
*/
#define SYSCFG_FACTORY_ADDRESS  (FLASH_TABLE_START_ADDR + 0xE000)
#define AUTO_FLASH_MAC_ADDRESS  (FLASH_TABLE_START_ADDR + 0xE000)
#define SYSCFG_FACTORY_SIZE     0x2000      /*!< Max configuration size*/
#define NMI_SEC_CODE_SIZE           (0x8C00)
#define NMI_SEC_SHARE_SIZE          (0x400)

#define NMI_SEC_CODE_START_ADDR      (PSRAM_BASE + PSRAM_SIZE - NMI_SEC_CODE_SIZE - NMI_SEC_SHARE_SIZE)
#define NMI_SEC_SHARE_START_ADDR     (PSRAM_BASE + PSRAM_SIZE - NMI_SEC_SHARE_SIZE)

#ifdef CUSTOM_MEM_MAP
    #ifdef SOLUTION_WATCH
        #include "flash_map.h"
    #else
        #ifdef __has_include
            #if __has_include("custom_mem_map.h")
                #include "custom_mem_map.h"
            #else
                #include "ptab.h"
            #endif
        #else
            #include "custom_mem_map.h"
        #endif
    #endif
#endif /* CUSTOM_MEM_MAP */

#define HPSYS_RAM_IN_ITCM(addr) 0
#endif  /* __MEM_MAP__ */
