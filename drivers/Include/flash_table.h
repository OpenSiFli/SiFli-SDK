/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

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

#ifdef __cplusplus
extern "C" {
#endif

#define EXT_FLAGS_DTR_SUPPORT_FLAG_Pos    (0U)
#define EXT_FLAGS_DTR_SUPPORT_FLAG_Msk    (0x1U << EXT_FLAGS_DTR_SUPPORT_FLAG_Pos)
#define DTR_SUPPORT_FLAG                  EXT_FLAGS_DTR_SUPPORT_FLAG_Msk
#define EXT_FLAGS_OTP_BASE_TYPE_Pos       (1U)
#define EXT_FLAGS_OTP_BASE_TYPE_Msk       (0x3U << EXT_FLAGS_OTP_BASE_TYPE_Pos)
#define EXT_FLAGS_OTP_BASE_TYPE_0         (0x0U << EXT_FLAGS_OTP_BASE_TYPE_Pos)
#define EXT_FLAGS_OTP_BASE_TYPE_FFC000    (0x1U << EXT_FLAGS_OTP_BASE_TYPE_Pos)
#define PLANE_SELEC_FLAG        (1<<1)
#define BIG_PAGE_FLAG           (1<<2)
#define BIG_BLK_FLAG            (1<<3)

#define NAND_ECC_START_POS              (4)
#define NAND_ECC_FULL_RESERVED            (0xF<<NAND_ECC_START_POS)
/** ecc status mode
 * if ext_cfg is present in handle, the ecc_mode is only used to define ecc status register bit width, i.e. how many bits represent ecc status,
 * the meaning is defined by ext_cfg.ecc_err_mask
 */
typedef enum __NAND_ECC_STATUS_MODE_
{
    BIT2_IN_C0_T1 = 0,   // int reg c0, bit 4 and bit 5, b00 no error; b01 1-bit been corrected; others more than 1 bit and can not correct
    BIT2_IN_C0_T2 = 1,   // 2 bits in c0, bit 4 and 5, b00 no error; b01/b03 has error but corrected; b02 has error can not recorrected, and may have ext infor in other reg
    BIT3_IN_C0_T1 = 2,   // 3 bits in c0, bit 4,5,6; 0 no err; 1,3,5 has err but corrected, 2 more than 8 bite err no correct, others reseved
    BIT3_IN_C0_T2 = 3,   // 3 bits in c0, bit 4,5,6; 0 no err; 7 has err but can not corrected, others has error but corrected
    BIT4_IN_C0_T1 = 4,   // 4 bits in c0, 0 no error; xx10b err can not corrected; others other corrected
    BIT4_IN_C0_T2 = 5,   // 4 bits in c0, 0 no error;  error bit = bit value, max to 8; error when larger than 8
    BIT2_IN_C0_T3 = 6,   // 2 bits in c0, bit 4 and bit 5, b00 no error; b01 1~2 bit corrected; b10 3~6 bit corrected; b11 not corrected
    BIT2_IN_C0_T4 = 7    // 2 bits in c0, bit 4 and bit 5, b00 no error; b01 1~2 bit corrected; b10 3~4 bit corrected; b11 5~6 bit corrected
} NAND_ECC_MODE_T;

typedef struct FLASH_FULL_CHIP_ID
{
    uint8_t manufacture_id;
    /** for NAND, it's the first byte of device id */
    uint8_t memory_type;
    /** for NAND, it's the second byte of device id
     * for some NAND, device id size is only one byte, the second byte is simply a repeat of manufacture_id
     */
    uint8_t memory_density; // union 16 bits as device ID for NAND
    uint8_t ext_flags;      // bit 0: dtr support flag for NOR, set to 0 as reserved for NAND
    // bit 1: plane select flag for NAND, set to 0 as reserved for NOR
    // bit 2: for NAND big nand page, 0 for default 2048, 1 for 4096, set to 0 as reserved for NOR
    // bit 3: for NAND big nand blk, 0 for 64 pages, 1 for 128 pages, set to 0 as reserved for NOR
    // bit 4~7: for NAND ECC status mode as NAND_ECC_MODE_T, set to 0 as reserved for NOR
    uint32_t mem_size;  // flash size with bytes
} FLASH_RDID_TYPE_T;

typedef enum
{
    NAND_TYPE0 = 0,  // normal type, base on winbond w25n01gw, with NON-BUF, NO QE, EB with 4 dummy
    NAND_TYPE1,      // based on XT26G01D, BUF, QE, EB, EB with 2 dummy
    NAND_TYPE2,      // based on ds35x1gaxxx, BUF , QE, NO EB
    NAND_TYPE3,      // based on tc58cyg0s3hraij, BUF, NO QE, NO EB
    NAND_TYPE4,      // based on FM25LS01, BUF, NO QE, EB   with 4 dummy
    NAND_TYPE5,      // based on GD5F1GM7RE, BUF, QE, EB, EB with 4 dummy
    NAND_CMD_TABLE_CNT
} NAND_CMD_TABLE_ID_T;

typedef enum
{
    NOR_TYPE0 = 0,  // normal type 0, DTR, NO CMD_WRSR2, Max 128Mb, as default command table
    NOR_TYPE1,      // type 1, WRSR2 to write status register 2(QE), Max 128Mb
    NOR_TYPE2,      // type 2, 256Mb, DTR, 4 bytes address command diff with 3 bytes, OTP support 4-B mode
    NOR_TYPE3,      // type 3, 256Mb , NO DTR , 4 bytes command same to 3 bytes, only timing changed, OTP 3-B only
    NOR_TYPE4,      // type 4, 256Mb, NO DTR, 4B ADDR command diff with 3B addr , OTP support 4-B mode
    NOR_TYPE5,      // type 5, 256Mb, NO DTR, MXIC flash have too many diff with others
    NOR_CMD_TABLE_CNT
} FLASH_CMD_TABLE_ID_T;

const SPI_FLASH_FACT_CFG_T *spi_flash_get_cmd_by_id(uint8_t fid, uint8_t did, uint8_t type);
int spi_flash_get_size_by_id(uint8_t fid, uint8_t did, uint8_t type);
int spi_flash_is_support_dtr(uint8_t fid, uint8_t did, uint8_t type);
/**
 * @brief Get the OTP base address for the specified NOR flash.
 *
 * Looks up the flash entry by manufacturer ID, density ID and memory type,
 * and returns the OTP base address saved in ext_flags.
     *
 * @param[in] fid    manufacture id
 * @param[in] did    low 8bit of device id, density id for NOR,
 * @param[in] mtype  high 8bit of device id, memory type of NOR
 * @return OTP base address
 */
uint32_t spi_flash_get_otp_base(uint8_t fid, uint8_t did, uint8_t mtype);

const SPI_FLASH_FACT_CFG_T *spi_nand_get_cmd_by_id(uint8_t fid, uint8_t did, uint8_t type);
const SPI_FLASH_FACT_CFG_T *spi_nand_get_default_ctable(void);
int spi_nand_get_size_by_id(uint8_t fid, uint8_t did, uint8_t type);
int spi_nand_get_plane_select_flag(uint8_t fid, uint8_t did, uint8_t type);
int spi_nand_get_big_page_flag(uint8_t fid, uint8_t did, uint8_t type);
int spi_nand_get_ecc_mode(uint8_t fid, uint8_t did, uint8_t type);

/**
 * @brief Get NAND extended configuration by flash chipid
 *
 * It's a weak function that can be overridden by user.
 * Default implementation returns NULL, which means no extended configuration for NAND.
 *
 * @param[in] fid    manufacture id
 * @param[in] did    low 8bit of device id
 *                   for 8bit device id, did is repeat of manufacture id, mtype is used as the 8bit device id
 * @param[in] mtype  high 8bit of device id
 *
 * @return pointer to nand_ext_cfg_t structure
 *
 */
const nand_ext_cfg_t *spi_nand_get_ext_cfg_by_id(uint8_t fid, uint8_t did, uint8_t mtype);

/**
 * @brief Get flash type and config according to flash chipid
 *
 * It's a weak function that can be overridden by user to add user-defined flash
 *
 * @param[in] fid    manufacture id
 * @param[in] did    low 8bit of device id, density id for NOR,
 *                   for 8bit device id, did is repeat of manufacture id, mtype is used as the 8bit device id
 * @param[in] mtype   high 8bit of device id, memory type of NOR
 * @param[out] cmd_tbl_type  nand cmd table type
 *
 * @return pointer to FLASH_RDID_TYPE_T structure
 *
 */
const FLASH_RDID_TYPE_T *spi_nand_get_user_flash_cfg(uint8_t fid, uint8_t did, uint8_t mtype, NAND_CMD_TABLE_ID_T *cmd_tbl_type);

/**
 * @brief Get NOR extended configuration by flash chipid
 *
 * It's a weak function that can be overridden by user.
 * Default implementation returns NULL, which means no extended configuration for NOR.
 *
 * @param[in] fid    manufacture id
 * @param[in] did    low 8bit of device id, density id for NOR,
 * @param[in] mtype   high 8bit of device id, memory type of NOR
 *
 * @return pointer to nor_ext_cfg_t structure
 *
 */
const nor_ext_cfg_t *spi_nor_get_ext_cfg_by_id(uint8_t fid, uint8_t did, uint8_t mtype);

/**
 * @brief Get flash type and config according to flash chipid
 *
 * It's a weak function that can be overridden by user to add user-defined flash
 *
 * @param[in] fid    manufacture id
 * @param[in] did    low 8bit of device id, density id for NOR,
 * @param[in] mtype   high 8bit of device id, memory type of NOR
 * @param[out] cmd_tbl_type  nor cmd table type
 *
 * @return pointer to FLASH_RDID_TYPE_T structure
 *
 */
const FLASH_RDID_TYPE_T *spi_nor_get_user_flash_cfg(uint8_t fid, uint8_t did, uint8_t mtype, FLASH_CMD_TABLE_ID_T *cmd_tbl_type);

extern FT_CONST FLASH_RDID_TYPE_T *FT_CONST flash_cmd_id_pool[];
extern FT_CONST SPI_FLASH_FACT_CFG_T flash_cmd_table_list[];

extern FT_CONST SPI_FLASH_FACT_CFG_T nand_cmd_table_list[];
extern FT_CONST FLASH_RDID_TYPE_T   *FT_CONST nand_cmd_id_pool[];

#if defined(JLINK) || defined(KEIL)
void spi_nor_table_init(void);
void spi_nand_table_init(void);
#endif /* JLINK || KEIL */

#ifdef __cplusplus
}
#endif
#endif  // _SIF_FLASH_CMD_TABLE_H__