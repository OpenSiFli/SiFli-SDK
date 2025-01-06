#ifndef __PTAB__H__
#define __PTAB__H__




/* flash5 */
#undef  FLASH_BOOT_LOADER_START_ADDR
#define FLASH_BOOT_LOADER_START_ADDR                       (0x1C080000)
#undef  FLASH_BOOT_LOADER_SIZE
#define FLASH_BOOT_LOADER_SIZE                             (0x00018000)
#undef  LCPU_FLASH_CODE_START_ADDR
#define LCPU_FLASH_CODE_START_ADDR                         (0x1C020000)
#undef  LCPU_FLASH_CODE_SIZE
#define LCPU_FLASH_CODE_SIZE                               (0x00080000)


/* psram1 */
#undef  PSRAM_DATA_START_ADDR
#define PSRAM_DATA_START_ADDR                              (0x60200000)
#undef  PSRAM_DATA_SIZE
#define PSRAM_DATA_SIZE                                    (0x00200000)
#undef  ACPU_CODE_REGION2_START_ADDR
#define ACPU_CODE_REGION2_START_ADDR                       (0x60400000)
#undef  ACPU_CODE_REGION2_SIZE
#define ACPU_CODE_REGION2_SIZE                             (0x00020000)


/* psram1_cbus */
#undef  HCPU_FLASH_CODE_START_ADDR
#define HCPU_FLASH_CODE_START_ADDR                         (0x10000000)
#undef  HCPU_FLASH_CODE_SIZE
#define HCPU_FLASH_CODE_SIZE                               (0x001C2000)


/* flash4 */
#undef  HCPU_FLASH_CODE_LOAD_REGION_START_ADDR
#define HCPU_FLASH_CODE_LOAD_REGION_START_ADDR             (0x18000000)
#undef  HCPU_FLASH_CODE_LOAD_REGION_SIZE
#define HCPU_FLASH_CODE_LOAD_REGION_SIZE                   (0x00200000)
#undef  FS_REGION_START_ADDR
#define FS_REGION_START_ADDR                               (0x18200000)
#undef  FS_REGION_SIZE
#define FS_REGION_SIZE                                     (0x00100000)
#undef  ACPU_CODE_LOAD_REGION1_START_ADDR
#define ACPU_CODE_LOAD_REGION1_START_ADDR                  (0x18300000)
#undef  ACPU_CODE_LOAD_REGION1_SIZE
#define ACPU_CODE_LOAD_REGION1_SIZE                        (0x00010000)
#undef  ACPU_CODE_LOAD_REGION2_START_ADDR
#define ACPU_CODE_LOAD_REGION2_START_ADDR                  (0x18420000)
#undef  ACPU_CODE_LOAD_REGION2_SIZE
#define ACPU_CODE_LOAD_REGION2_SIZE                        (0x00020000)


/* hpsys_ram */
#undef  HCPU_RAM_DATA_START_ADDR
#define HCPU_RAM_DATA_START_ADDR                           (0x20000000)
#undef  HCPU_RAM_DATA_SIZE
#define HCPU_RAM_DATA_SIZE                                 (0x0006BC00)
#undef  HCPU_RO_DATA_START_ADDR
#define HCPU_RO_DATA_START_ADDR                            (0x2006BC00)
#undef  HCPU_RO_DATA_SIZE
#define HCPU_RO_DATA_SIZE                                  (0x00014000)
#undef  HPSYS_MBOX_START_ADDR
#define HPSYS_MBOX_START_ADDR                              (0x2007FC00)
#undef  HPSYS_MBOX_SIZE
#define HPSYS_MBOX_SIZE                                    (0x00000400)
#undef  HCPU2LCPU_MB_CH2_BUF_START_ADDR
#define HCPU2LCPU_MB_CH2_BUF_START_ADDR                    (0x2007FC00)
#undef  HCPU2LCPU_MB_CH2_BUF_SIZE
#define HCPU2LCPU_MB_CH2_BUF_SIZE                          (0x00000200)
#undef  HCPU2LCPU_MB_CH1_BUF_START_ADDR
#define HCPU2LCPU_MB_CH1_BUF_START_ADDR                    (0x2007FE00)
#undef  HCPU2LCPU_MB_CH1_BUF_SIZE
#define HCPU2LCPU_MB_CH1_BUF_SIZE                          (0x00000200)
#undef  ACPU_CODE_REGION1_SBUS_START_ADDR
#define ACPU_CODE_REGION1_SBUS_START_ADDR                  (0x20200000)
#undef  ACPU_CODE_REGION1_SBUS_SIZE
#define ACPU_CODE_REGION1_SBUS_SIZE                        (0x00020000)


/* acpu_cbus */
#undef  ACPU_CODE_REGION1_START_ADDR
#define ACPU_CODE_REGION1_START_ADDR                       (0x00000000)
#undef  ACPU_CODE_REGION1_SIZE
#define ACPU_CODE_REGION1_SIZE                             (0x00020000)



#endif
