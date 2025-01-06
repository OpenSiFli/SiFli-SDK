#ifndef __PTAB__H__
#define __PTAB__H__




/* flash2 */
#undef  FLASH_TABLE_START_ADDR
#define FLASH_TABLE_START_ADDR                             (0x62000000)
#undef  FLASH_TABLE_SIZE
#define FLASH_TABLE_SIZE                                   (0x00008000)
#undef  FLASH_TABLE_OFFSET
#define FLASH_TABLE_OFFSET                                 (0x00000000)
#undef  _START_ADDR
#define _START_ADDR                                        (0x62040000)
#undef  _SIZE
#define _SIZE                                              (0x00600000)
#undef  _OFFSET
#define _OFFSET                                            (0x00040000)
#undef  FS_REGION_START_ADDR
#define FS_REGION_START_ADDR                               (0x63040000)
#undef  FS_REGION_SIZE
#define FS_REGION_SIZE                                     (0x00800000)
#undef  FS_REGION_OFFSET
#define FS_REGION_OFFSET                                   (0x01040000)
#undef  FS_ROOT_START_ADDR
#define FS_ROOT_START_ADDR                                 (0x63040000)
#undef  FS_ROOT_SIZE
#define FS_ROOT_SIZE                                       (0x00800000)
#undef  FS_ROOT_OFFSET
#define FS_ROOT_OFFSET                                     (0x01040000)
#undef  FLASH_PART2_START_ADDR
#define FLASH_PART2_START_ADDR                             (0x63840000)
#undef  FLASH_PART2_SIZE
#define FLASH_PART2_SIZE                                   (0x00080000)
#undef  FLASH_PART2_OFFSET
#define FLASH_PART2_OFFSET                                 (0x01840000)
#undef  FLASH_PART12_START_ADDR
#define FLASH_PART12_START_ADDR                            (0x638C0000)
#undef  FLASH_PART12_SIZE
#define FLASH_PART12_SIZE                                  (0x00080000)
#undef  FLASH_PART12_OFFSET
#define FLASH_PART12_OFFSET                                (0x018C0000)


/* psram1 */
#undef  PSRAM_DATA_START_ADDR
#define PSRAM_DATA_START_ADDR                              (0x60400000)
#undef  PSRAM_DATA_SIZE
#define PSRAM_DATA_SIZE                                    (0x00400000)
#undef  PSRAM_DATA_OFFSET
#define PSRAM_DATA_OFFSET                                  (0x00400000)


/* psram1_cbus */
#undef  HCPU_FLASH_CODE_START_ADDR
#define HCPU_FLASH_CODE_START_ADDR                         (0x10000000)
#undef  HCPU_FLASH_CODE_SIZE
#define HCPU_FLASH_CODE_SIZE                               (0x00400000)
#undef  HCPU_FLASH_CODE_OFFSET
#define HCPU_FLASH_CODE_OFFSET                             (0x00000000)


/* hpsys_ram */
#undef  HCPU_RAM_DATA_START_ADDR
#define HCPU_RAM_DATA_START_ADDR                           (0x20000000)
#undef  HCPU_RAM_DATA_SIZE
#define HCPU_RAM_DATA_SIZE                                 (0x00052C00)
#undef  HCPU_RAM_DATA_OFFSET
#define HCPU_RAM_DATA_OFFSET                               (0x00000000)
#undef  NMI_SEC_CODE_START_ADDR
#define NMI_SEC_CODE_START_ADDR                            (0x20052C00)
#undef  NMI_SEC_CODE_SIZE
#define NMI_SEC_CODE_SIZE                                  (0x00018F80)
#undef  NMI_SEC_CODE_OFFSET
#define NMI_SEC_CODE_OFFSET                                (0x00052C00)
#undef  NMI_SEC_SHARE_START_ADDR
#define NMI_SEC_SHARE_START_ADDR                           (0x2006BB80)
#undef  NMI_SEC_SHARE_SIZE
#define NMI_SEC_SHARE_SIZE                                 (0x00000080)
#undef  NMI_SEC_SHARE_OFFSET
#define NMI_SEC_SHARE_OFFSET                               (0x0006BB80)
#undef  FLASH_BOOT_LOADER_START_ADDR
#define FLASH_BOOT_LOADER_START_ADDR                       (0x20020000)
#undef  FLASH_BOOT_LOADER_SIZE
#define FLASH_BOOT_LOADER_SIZE                             (0x00010000)
#undef  FLASH_BOOT_LOADER_OFFSET
#define FLASH_BOOT_LOADER_OFFSET                           (0x00020000)
#undef  BOOTLOADER_RAM_DATA_START_ADDR
#define BOOTLOADER_RAM_DATA_START_ADDR                     (0x20030000)
#undef  BOOTLOADER_RAM_DATA_SIZE
#define BOOTLOADER_RAM_DATA_SIZE                           (0x00010000)
#undef  BOOTLOADER_RAM_DATA_OFFSET
#define BOOTLOADER_RAM_DATA_OFFSET                         (0x00030000)
#undef  HCPU_RO_DATA_START_ADDR
#define HCPU_RO_DATA_START_ADDR                            (0x2006BC00)
#undef  HCPU_RO_DATA_SIZE
#define HCPU_RO_DATA_SIZE                                  (0x00014000)
#undef  HCPU_RO_DATA_OFFSET
#define HCPU_RO_DATA_OFFSET                                (0x0006BC00)
#undef  HPSYS_MBOX_START_ADDR
#define HPSYS_MBOX_START_ADDR                              (0x2007FC00)
#undef  HPSYS_MBOX_SIZE
#define HPSYS_MBOX_SIZE                                    (0x00000400)
#undef  HPSYS_MBOX_OFFSET
#define HPSYS_MBOX_OFFSET                                  (0x0007FC00)
#undef  HCPU2LCPU_MB_CH2_BUF_START_ADDR
#define HCPU2LCPU_MB_CH2_BUF_START_ADDR                    (0x2007FC00)
#undef  HCPU2LCPU_MB_CH2_BUF_SIZE
#define HCPU2LCPU_MB_CH2_BUF_SIZE                          (0x00000200)
#undef  HCPU2LCPU_MB_CH2_BUF_OFFSET
#define HCPU2LCPU_MB_CH2_BUF_OFFSET                        (0x0007FC00)
#undef  HCPU2LCPU_MB_CH1_BUF_START_ADDR
#define HCPU2LCPU_MB_CH1_BUF_START_ADDR                    (0x2007FE00)
#undef  HCPU2LCPU_MB_CH1_BUF_SIZE
#define HCPU2LCPU_MB_CH1_BUF_SIZE                          (0x00000200)
#undef  HCPU2LCPU_MB_CH1_BUF_OFFSET
#define HCPU2LCPU_MB_CH1_BUF_OFFSET                        (0x0007FE00)


/* lcpu_ram */
#undef  LCPU_RAM_CODE_START_ADDR
#define LCPU_RAM_CODE_START_ADDR                           (0x20400000)
#undef  LCPU_RAM_CODE_SIZE
#define LCPU_RAM_CODE_SIZE                                 (0x00001800)
#undef  LCPU_RAM_CODE_OFFSET
#define LCPU_RAM_CODE_OFFSET                               (0x00000000)



#endif
