
#ifndef _SIF_FLASH_CMD_TABLE_H__
#define _SIF_FLASH_CMD_TABLE_H__

#ifdef SF32LB55X
    #include "bf0_hal_qspi_ex.h"
#else
    #include "bf0_hal_mpi_ex.h"
#endif


#if defined(CFG_BOOTLOADER) || defined(JLINK) || defined(KEIL)
    #define FT_CONST               /* For bootloader, need compress to reduce code size */
#else
    #define FT_CONST const
#endif


#define DTR_SUPPORT_FLAG        (1<<0)
#define PLANE_SELEC_FLAG        (1<<1)
#define BIG_PAGE_FLAG           (1<<2)
#define BIG_BLK_FLAG            (1<<3)

#define NAND_ECC_START_POS              (4)
#define NAND_ECC_FULL_RESERVED            (0xF<<NAND_ECC_START_POS)
typedef enum __NAND_ECC_STATUS_MODE_
{
    BIT2_IN_C0_T1 = 0,   // int reg c0, bit 4 and bit 5, b00 no error; b01 1-bit been corrected; others more than 1 bit and can not correct
    BIT2_IN_C0_T2 = 1, // 2 bits in c0, bit 4 and 5, b00 no error; b01/b03 has error but corrected; b02 has error can not recorrected, and may have ext infor in other reg
    BIT3_IN_C0_T1 = 2, // 3 bits in c0, bit 4,5,6; 0 no err; 1,3,5 has err but corrected, 2 more than 8 bite err no correct, others reseved
    BIT3_IN_C0_T2 = 3, // 3 bits in c0, bit 4,5,6; 0 no err; 7 has err but can not corrected, others has error but corrected
    BIT4_IN_C0_T1 = 4,   // 4 bits in c0, 0 no error; xx10b err can not corrected; others other corrected
    BIT4_IN_C0_T2 = 5,   // 4 bits in c0, 0 no error;  error bit = bit value, max to 8; error when larger than 8
    BIT2_IN_C0_T3 = 6   // 2 bits in c0, bit 4 and bit 5, b00 no error; b01 1~2 bit corrected; b10 3~6 bit corrected; b11 not corrected
} NAND_ECC_MODE_T;

typedef struct FLASH_FULL_CHIP_ID
{
    uint8_t manufacture_id;
    uint8_t memory_type;
    uint8_t memory_density;// union 16 bits as device ID for NAND
    uint8_t ext_flags;    // bit 0: dtr support flag for NOR, set to 0 as reserved for NAND
    // bit 1: plane select flag for NAND, set to 0 as reserved for NOR
    // bit 2: for NAND big nand page, 0 for default 2048, 1 for 4096, set to 0 as reserved for NOR
    // bit 3: for NAND big nand blk, 0 for 64 pages, 1 for 128 pages, set to 0 as reserved for NOR
    // bit 4~7: for NAND ECC status mode as NAND_ECC_MODE_T, set to 0 as reserved for NOR
    uint32_t mem_size;  // flash size with bytes
} FLASH_RDID_TYPE_T;


const SPI_FLASH_FACT_CFG_T *spi_flash_get_cmd_by_id(uint8_t fid, uint8_t did, uint8_t type);
int spi_flash_get_size_by_id(uint8_t fid, uint8_t did, uint8_t type);
int spi_flash_is_support_dtr(uint8_t fid, uint8_t did, uint8_t type);

const SPI_FLASH_FACT_CFG_T *spi_nand_get_cmd_by_id(uint8_t fid, uint8_t did, uint8_t type);
const SPI_FLASH_FACT_CFG_T *spi_nand_get_default_ctable(void);
int spi_nand_get_size_by_id(uint8_t fid, uint8_t did, uint8_t type);
int spi_nand_get_plane_select_flag(uint8_t fid, uint8_t did, uint8_t type);
int spi_nand_get_big_page_flag(uint8_t fid, uint8_t did, uint8_t type);
int spi_nand_get_ecc_mode(uint8_t fid, uint8_t did, uint8_t type);

extern FT_CONST FLASH_RDID_TYPE_T *FT_CONST flash_cmd_id_pool[];
extern FT_CONST SPI_FLASH_FACT_CFG_T flash_cmd_table_list[];

extern FT_CONST SPI_FLASH_FACT_CFG_T nand_cmd_table_list[];
extern FT_CONST FLASH_RDID_TYPE_T   *FT_CONST nand_cmd_id_pool[];

#if defined(JLINK) || defined(KEIL)
    void spi_nor_table_init(void);
    void spi_nand_table_init(void);
#endif /* JLINK || KEIL */

#endif  // _SIF_FLASH_CMD_TABLE_H__
