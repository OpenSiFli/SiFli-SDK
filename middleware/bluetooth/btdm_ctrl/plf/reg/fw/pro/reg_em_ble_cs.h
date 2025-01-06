#ifndef _REG_EM_BLE_CS_H_
#define _REG_EM_BLE_CS_H_

#include <stdint.h>
#include "_reg_em_ble_cs.h"
#include "compiler.h"
#include "arch.h"
#include "em_map.h"
#include "reg_access.h"

#define REG_EM_BLE_CS_COUNT 74

#define REG_EM_BLE_CS_DECODING_MASK 0x000000FF

#define REG_EM_BLE_CS_ADDR_GET(idx) (EM_BLE_CS_OFFSET + (idx) * REG_EM_BLE_CS_SIZE)

/**
 * @brief CNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     10             TXBSY_EN   0
 *     09             RXBSY_EN   0
 *     08              DNABORT   0
 *  04:00               FORMAT   0x0
 * </pre>
 */
#define EM_BLE_CNTL_ADDR   (0x204F0000 + EM_BLE_CS_OFFSET)
#define EM_BLE_CNTL_INDEX  0x00000000
#define EM_BLE_CNTL_RESET  0x00000000

__INLINE uint16_t em_ble_cntl_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_CNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_cntl_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_CNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_TXBSY_EN_BIT    ((uint16_t)0x00000400)
#define EM_BLE_TXBSY_EN_POS    10
#define EM_BLE_RXBSY_EN_BIT    ((uint16_t)0x00000200)
#define EM_BLE_RXBSY_EN_POS    9
#define EM_BLE_DNABORT_BIT     ((uint16_t)0x00000100)
#define EM_BLE_DNABORT_POS     8
#define EM_BLE_FORMAT_MASK     ((uint16_t)0x0000001F)
#define EM_BLE_FORMAT_LSB      0
#define EM_BLE_FORMAT_WIDTH    ((uint16_t)0x00000005)

#define EM_BLE_TXBSY_EN_RST    0x0
#define EM_BLE_RXBSY_EN_RST    0x0
#define EM_BLE_DNABORT_RST     0x0
#define EM_BLE_FORMAT_RST      0x0

__INLINE void em_ble_cntl_pack(int elt_idx, uint8_t txbsyen, uint8_t rxbsyen, uint8_t dnabort, uint8_t format)
{
    ASSERT_ERR((((uint16_t)txbsyen << 10) & ~((uint16_t)0x00000400)) == 0);
    ASSERT_ERR((((uint16_t)rxbsyen << 9) & ~((uint16_t)0x00000200)) == 0);
    ASSERT_ERR((((uint16_t)dnabort << 8) & ~((uint16_t)0x00000100)) == 0);
    ASSERT_ERR((((uint16_t)format << 0) & ~((uint16_t)0x0000001F)) == 0);
    EM_BLE_WR(EM_BLE_CNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, ((uint16_t)txbsyen << 10) | ((uint16_t)rxbsyen << 9) | ((uint16_t)dnabort << 8) | ((uint16_t)format << 0));
}

__INLINE void em_ble_cntl_unpack(int elt_idx, uint8_t *txbsyen, uint8_t *rxbsyen, uint8_t *dnabort, uint8_t *format)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_CNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);

    *txbsyen = (localVal & ((uint16_t)0x00000400)) >> 10;
    *rxbsyen = (localVal & ((uint16_t)0x00000200)) >> 9;
    *dnabort = (localVal & ((uint16_t)0x00000100)) >> 8;
    *format = (localVal & ((uint16_t)0x0000001F)) >> 0;
}

__INLINE uint8_t em_ble_cntl_txbsy_en_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_CNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000400)) >> 10);
}

__INLINE void em_ble_cntl_txbsy_en_setf(int elt_idx, uint8_t txbsyen)
{
    ASSERT_ERR((((uint16_t)txbsyen << 10) & ~((uint16_t)0x00000400)) == 0);
    EM_BLE_WR(EM_BLE_CNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_CNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000400)) | ((uint16_t)txbsyen << 10));
}

__INLINE uint8_t em_ble_cntl_rxbsy_en_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_CNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000200)) >> 9);
}

__INLINE void em_ble_cntl_rxbsy_en_setf(int elt_idx, uint8_t rxbsyen)
{
    ASSERT_ERR((((uint16_t)rxbsyen << 9) & ~((uint16_t)0x00000200)) == 0);
    EM_BLE_WR(EM_BLE_CNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_CNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000200)) | ((uint16_t)rxbsyen << 9));
}

__INLINE uint8_t em_ble_cntl_dnabort_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_CNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000100)) >> 8);
}

__INLINE void em_ble_cntl_dnabort_setf(int elt_idx, uint8_t dnabort)
{
    ASSERT_ERR((((uint16_t)dnabort << 8) & ~((uint16_t)0x00000100)) == 0);
    EM_BLE_WR(EM_BLE_CNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_CNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000100)) | ((uint16_t)dnabort << 8));
}

__INLINE uint8_t em_ble_cntl_format_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_CNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000001F)) >> 0);
}

__INLINE void em_ble_cntl_format_setf(int elt_idx, uint8_t format)
{
    ASSERT_ERR((((uint16_t)format << 0) & ~((uint16_t)0x0000001F)) == 0);
    EM_BLE_WR(EM_BLE_CNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_CNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x0000001F)) | ((uint16_t)format << 0));
}

/**
 * @brief LINKCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:14           HP_LP_MODE   0x0
 *  12:08              LINKLBL   0x0
 *     06                  SAS   0
 *     05        NULLRXLLIDFLT   0
 *     04             MIC_MODE   0
 *     03           CRYPT_MODE   0
 *     02           TXCRYPT_EN   0
 *     01           RXCRYPT_EN   0
 *     00            PRIV_NPUB   0
 * </pre>
 */
#define EM_BLE_LINKCNTL_ADDR   (0x204F0002 + EM_BLE_CS_OFFSET)
#define EM_BLE_LINKCNTL_INDEX  0x00000001
#define EM_BLE_LINKCNTL_RESET  0x00000000

__INLINE uint16_t em_ble_linkcntl_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_linkcntl_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_HP_LP_MODE_MASK      ((uint16_t)0x0000C000)
#define EM_BLE_HP_LP_MODE_LSB       14
#define EM_BLE_HP_LP_MODE_WIDTH     ((uint16_t)0x00000002)
#define EM_BLE_LINKLBL_MASK         ((uint16_t)0x00001F00)
#define EM_BLE_LINKLBL_LSB          8
#define EM_BLE_LINKLBL_WIDTH        ((uint16_t)0x00000005)
#define EM_BLE_SAS_BIT              ((uint16_t)0x00000040)
#define EM_BLE_SAS_POS              6
#define EM_BLE_NULLRXLLIDFLT_BIT    ((uint16_t)0x00000020)
#define EM_BLE_NULLRXLLIDFLT_POS    5
#define EM_BLE_MIC_MODE_BIT         ((uint16_t)0x00000010)
#define EM_BLE_MIC_MODE_POS         4
#define EM_BLE_CRYPT_MODE_BIT       ((uint16_t)0x00000008)
#define EM_BLE_CRYPT_MODE_POS       3
#define EM_BLE_TXCRYPT_EN_BIT       ((uint16_t)0x00000004)
#define EM_BLE_TXCRYPT_EN_POS       2
#define EM_BLE_RXCRYPT_EN_BIT       ((uint16_t)0x00000002)
#define EM_BLE_RXCRYPT_EN_POS       1
#define EM_BLE_PRIV_NPUB_BIT        ((uint16_t)0x00000001)
#define EM_BLE_PRIV_NPUB_POS        0

#define EM_BLE_HP_LP_MODE_RST       0x0
#define EM_BLE_LINKLBL_RST          0x0
#define EM_BLE_SAS_RST              0x0
#define EM_BLE_NULLRXLLIDFLT_RST    0x0
#define EM_BLE_MIC_MODE_RST         0x0
#define EM_BLE_CRYPT_MODE_RST       0x0
#define EM_BLE_TXCRYPT_EN_RST       0x0
#define EM_BLE_RXCRYPT_EN_RST       0x0
#define EM_BLE_PRIV_NPUB_RST        0x0

__INLINE void em_ble_linkcntl_pack(int elt_idx, uint8_t hplpmode, uint8_t linklbl, uint8_t sas, uint8_t nullrxllidflt, uint8_t micmode, uint8_t cryptmode, uint8_t txcrypten, uint8_t rxcrypten, uint8_t privnpub)
{
    ASSERT_ERR((((uint16_t)hplpmode << 14) & ~((uint16_t)0x0000C000)) == 0);
    ASSERT_ERR((((uint16_t)linklbl << 8) & ~((uint16_t)0x00001F00)) == 0);
    ASSERT_ERR((((uint16_t)sas << 6) & ~((uint16_t)0x00000040)) == 0);
    ASSERT_ERR((((uint16_t)nullrxllidflt << 5) & ~((uint16_t)0x00000020)) == 0);
    ASSERT_ERR((((uint16_t)micmode << 4) & ~((uint16_t)0x00000010)) == 0);
    ASSERT_ERR((((uint16_t)cryptmode << 3) & ~((uint16_t)0x00000008)) == 0);
    ASSERT_ERR((((uint16_t)txcrypten << 2) & ~((uint16_t)0x00000004)) == 0);
    ASSERT_ERR((((uint16_t)rxcrypten << 1) & ~((uint16_t)0x00000002)) == 0);
    ASSERT_ERR((((uint16_t)privnpub << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BLE_WR(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, ((uint16_t)hplpmode << 14) | ((uint16_t)linklbl << 8) | ((uint16_t)sas << 6) | ((uint16_t)nullrxllidflt << 5) | ((uint16_t)micmode << 4) | ((uint16_t)cryptmode << 3) | ((uint16_t)txcrypten << 2) | ((uint16_t)rxcrypten << 1) | ((uint16_t)privnpub << 0));
}

__INLINE void em_ble_linkcntl_unpack(int elt_idx, uint8_t *hplpmode, uint8_t *linklbl, uint8_t *sas, uint8_t *nullrxllidflt, uint8_t *micmode, uint8_t *cryptmode, uint8_t *txcrypten, uint8_t *rxcrypten, uint8_t *privnpub)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);

    *hplpmode = (localVal & ((uint16_t)0x0000C000)) >> 14;
    *linklbl = (localVal & ((uint16_t)0x00001F00)) >> 8;
    *sas = (localVal & ((uint16_t)0x00000040)) >> 6;
    *nullrxllidflt = (localVal & ((uint16_t)0x00000020)) >> 5;
    *micmode = (localVal & ((uint16_t)0x00000010)) >> 4;
    *cryptmode = (localVal & ((uint16_t)0x00000008)) >> 3;
    *txcrypten = (localVal & ((uint16_t)0x00000004)) >> 2;
    *rxcrypten = (localVal & ((uint16_t)0x00000002)) >> 1;
    *privnpub = (localVal & ((uint16_t)0x00000001)) >> 0;
}

__INLINE uint8_t em_ble_linkcntl_hp_lp_mode_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000C000)) >> 14);
}

__INLINE void em_ble_linkcntl_hp_lp_mode_setf(int elt_idx, uint8_t hplpmode)
{
    ASSERT_ERR((((uint16_t)hplpmode << 14) & ~((uint16_t)0x0000C000)) == 0);
    EM_BLE_WR(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x0000C000)) | ((uint16_t)hplpmode << 14));
}

__INLINE uint8_t em_ble_linkcntl_linklbl_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00001F00)) >> 8);
}

__INLINE void em_ble_linkcntl_linklbl_setf(int elt_idx, uint8_t linklbl)
{
    ASSERT_ERR((((uint16_t)linklbl << 8) & ~((uint16_t)0x00001F00)) == 0);
    EM_BLE_WR(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00001F00)) | ((uint16_t)linklbl << 8));
}

__INLINE uint8_t em_ble_linkcntl_sas_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000040)) >> 6);
}

__INLINE void em_ble_linkcntl_sas_setf(int elt_idx, uint8_t sas)
{
    ASSERT_ERR((((uint16_t)sas << 6) & ~((uint16_t)0x00000040)) == 0);
    EM_BLE_WR(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000040)) | ((uint16_t)sas << 6));
}

__INLINE uint8_t em_ble_linkcntl_nullrxllidflt_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000020)) >> 5);
}

__INLINE void em_ble_linkcntl_nullrxllidflt_setf(int elt_idx, uint8_t nullrxllidflt)
{
    ASSERT_ERR((((uint16_t)nullrxllidflt << 5) & ~((uint16_t)0x00000020)) == 0);
    EM_BLE_WR(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000020)) | ((uint16_t)nullrxllidflt << 5));
}

__INLINE uint8_t em_ble_linkcntl_mic_mode_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000010)) >> 4);
}

__INLINE void em_ble_linkcntl_mic_mode_setf(int elt_idx, uint8_t micmode)
{
    ASSERT_ERR((((uint16_t)micmode << 4) & ~((uint16_t)0x00000010)) == 0);
    EM_BLE_WR(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000010)) | ((uint16_t)micmode << 4));
}

__INLINE uint8_t em_ble_linkcntl_crypt_mode_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000008)) >> 3);
}

__INLINE void em_ble_linkcntl_crypt_mode_setf(int elt_idx, uint8_t cryptmode)
{
    ASSERT_ERR((((uint16_t)cryptmode << 3) & ~((uint16_t)0x00000008)) == 0);
    EM_BLE_WR(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000008)) | ((uint16_t)cryptmode << 3));
}

__INLINE uint8_t em_ble_linkcntl_txcrypt_en_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000004)) >> 2);
}

__INLINE void em_ble_linkcntl_txcrypt_en_setf(int elt_idx, uint8_t txcrypten)
{
    ASSERT_ERR((((uint16_t)txcrypten << 2) & ~((uint16_t)0x00000004)) == 0);
    EM_BLE_WR(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000004)) | ((uint16_t)txcrypten << 2));
}

__INLINE uint8_t em_ble_linkcntl_rxcrypt_en_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000002)) >> 1);
}

__INLINE void em_ble_linkcntl_rxcrypt_en_setf(int elt_idx, uint8_t rxcrypten)
{
    ASSERT_ERR((((uint16_t)rxcrypten << 1) & ~((uint16_t)0x00000002)) == 0);
    EM_BLE_WR(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000002)) | ((uint16_t)rxcrypten << 1));
}

__INLINE uint8_t em_ble_linkcntl_priv_npub_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000001)) >> 0);
}

__INLINE void em_ble_linkcntl_priv_npub_setf(int elt_idx, uint8_t privnpub)
{
    ASSERT_ERR((((uint16_t)privnpub << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BLE_WR(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_LINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000001)) | ((uint16_t)privnpub << 0));
}

/**
 * @brief ISOLINKCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:11           STREAM_LBL   0x0
 *  10:08            GROUP_LBL   0x0
 *     03          ISOSYNCMODE   0
 *     02            ISOSYNCEN   0
 *  01:00              ISOTYPE   0x0
 * </pre>
 */
#define EM_BLE_ISOLINKCNTL_ADDR   (0x204F0004 + EM_BLE_CS_OFFSET)
#define EM_BLE_ISOLINKCNTL_INDEX  0x00000002
#define EM_BLE_ISOLINKCNTL_RESET  0x00000000

__INLINE uint16_t em_ble_isolinkcntl_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_ISOLINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_isolinkcntl_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_ISOLINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_STREAM_LBL_MASK    ((uint16_t)0x0000F800)
#define EM_BLE_STREAM_LBL_LSB     11
#define EM_BLE_STREAM_LBL_WIDTH   ((uint16_t)0x00000005)
#define EM_BLE_GROUP_LBL_MASK     ((uint16_t)0x00000700)
#define EM_BLE_GROUP_LBL_LSB      8
#define EM_BLE_GROUP_LBL_WIDTH    ((uint16_t)0x00000003)
#define EM_BLE_ISOSYNCMODE_BIT    ((uint16_t)0x00000008)
#define EM_BLE_ISOSYNCMODE_POS    3
#define EM_BLE_ISOSYNCEN_BIT      ((uint16_t)0x00000004)
#define EM_BLE_ISOSYNCEN_POS      2
#define EM_BLE_ISOTYPE_MASK       ((uint16_t)0x00000003)
#define EM_BLE_ISOTYPE_LSB        0
#define EM_BLE_ISOTYPE_WIDTH      ((uint16_t)0x00000002)

#define EM_BLE_STREAM_LBL_RST     0x0
#define EM_BLE_GROUP_LBL_RST      0x0
#define EM_BLE_ISOSYNCMODE_RST    0x0
#define EM_BLE_ISOSYNCEN_RST      0x0
#define EM_BLE_ISOTYPE_RST        0x0

__INLINE void em_ble_isolinkcntl_pack(int elt_idx, uint8_t streamlbl, uint8_t grouplbl, uint8_t isosyncmode, uint8_t isosyncen, uint8_t isotype)
{
    ASSERT_ERR((((uint16_t)streamlbl << 11) & ~((uint16_t)0x0000F800)) == 0);
    ASSERT_ERR((((uint16_t)grouplbl << 8) & ~((uint16_t)0x00000700)) == 0);
    ASSERT_ERR((((uint16_t)isosyncmode << 3) & ~((uint16_t)0x00000008)) == 0);
    ASSERT_ERR((((uint16_t)isosyncen << 2) & ~((uint16_t)0x00000004)) == 0);
    ASSERT_ERR((((uint16_t)isotype << 0) & ~((uint16_t)0x00000003)) == 0);
    EM_BLE_WR(EM_BLE_ISOLINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, ((uint16_t)streamlbl << 11) | ((uint16_t)grouplbl << 8) | ((uint16_t)isosyncmode << 3) | ((uint16_t)isosyncen << 2) | ((uint16_t)isotype << 0));
}

__INLINE void em_ble_isolinkcntl_unpack(int elt_idx, uint8_t *streamlbl, uint8_t *grouplbl, uint8_t *isosyncmode, uint8_t *isosyncen, uint8_t *isotype)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOLINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);

    *streamlbl = (localVal & ((uint16_t)0x0000F800)) >> 11;
    *grouplbl = (localVal & ((uint16_t)0x00000700)) >> 8;
    *isosyncmode = (localVal & ((uint16_t)0x00000008)) >> 3;
    *isosyncen = (localVal & ((uint16_t)0x00000004)) >> 2;
    *isotype = (localVal & ((uint16_t)0x00000003)) >> 0;
}

__INLINE uint8_t em_ble_isolinkcntl_stream_lbl_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOLINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000F800)) >> 11);
}

__INLINE void em_ble_isolinkcntl_stream_lbl_setf(int elt_idx, uint8_t streamlbl)
{
    ASSERT_ERR((((uint16_t)streamlbl << 11) & ~((uint16_t)0x0000F800)) == 0);
    EM_BLE_WR(EM_BLE_ISOLINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_ISOLINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x0000F800)) | ((uint16_t)streamlbl << 11));
}

__INLINE uint8_t em_ble_isolinkcntl_group_lbl_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOLINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000700)) >> 8);
}

__INLINE void em_ble_isolinkcntl_group_lbl_setf(int elt_idx, uint8_t grouplbl)
{
    ASSERT_ERR((((uint16_t)grouplbl << 8) & ~((uint16_t)0x00000700)) == 0);
    EM_BLE_WR(EM_BLE_ISOLINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_ISOLINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000700)) | ((uint16_t)grouplbl << 8));
}

__INLINE uint8_t em_ble_isolinkcntl_isosyncmode_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOLINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000008)) >> 3);
}

__INLINE void em_ble_isolinkcntl_isosyncmode_setf(int elt_idx, uint8_t isosyncmode)
{
    ASSERT_ERR((((uint16_t)isosyncmode << 3) & ~((uint16_t)0x00000008)) == 0);
    EM_BLE_WR(EM_BLE_ISOLINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_ISOLINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000008)) | ((uint16_t)isosyncmode << 3));
}

__INLINE uint8_t em_ble_isolinkcntl_isosyncen_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOLINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000004)) >> 2);
}

__INLINE void em_ble_isolinkcntl_isosyncen_setf(int elt_idx, uint8_t isosyncen)
{
    ASSERT_ERR((((uint16_t)isosyncen << 2) & ~((uint16_t)0x00000004)) == 0);
    EM_BLE_WR(EM_BLE_ISOLINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_ISOLINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000004)) | ((uint16_t)isosyncen << 2));
}

__INLINE uint8_t em_ble_isolinkcntl_isotype_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOLINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000003)) >> 0);
}

__INLINE void em_ble_isolinkcntl_isotype_setf(int elt_idx, uint8_t isotype)
{
    ASSERT_ERR((((uint16_t)isotype << 0) & ~((uint16_t)0x00000003)) == 0);
    EM_BLE_WR(EM_BLE_ISOLINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_ISOLINKCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000003)) | ((uint16_t)isotype << 0));
}

/**
 * @brief THRCNTL_RATECNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:12                RXTHR   0x0
 *  11:08                TXTHR   0x0
 *  05:04             AUX_RATE   0x0
 *  03:02               RXRATE   0x0
 *  01:00               TXRATE   0x0
 * </pre>
 */
#define EM_BLE_THRCNTL_RATECNTL_ADDR   (0x204F0006 + EM_BLE_CS_OFFSET)
#define EM_BLE_THRCNTL_RATECNTL_INDEX  0x00000003
#define EM_BLE_THRCNTL_RATECNTL_RESET  0x00000000

__INLINE uint16_t em_ble_thrcntl_ratecntl_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_THRCNTL_RATECNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_thrcntl_ratecntl_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_THRCNTL_RATECNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_RXTHR_MASK      ((uint16_t)0x0000F000)
#define EM_BLE_RXTHR_LSB       12
#define EM_BLE_RXTHR_WIDTH     ((uint16_t)0x00000004)
#define EM_BLE_TXTHR_MASK      ((uint16_t)0x00000F00)
#define EM_BLE_TXTHR_LSB       8
#define EM_BLE_TXTHR_WIDTH     ((uint16_t)0x00000004)
#define EM_BLE_AUX_RATE_MASK   ((uint16_t)0x00000030)
#define EM_BLE_AUX_RATE_LSB    4
#define EM_BLE_AUX_RATE_WIDTH  ((uint16_t)0x00000002)
#define EM_BLE_RXRATE_MASK     ((uint16_t)0x0000000C)
#define EM_BLE_RXRATE_LSB      2
#define EM_BLE_RXRATE_WIDTH    ((uint16_t)0x00000002)
#define EM_BLE_TXRATE_MASK     ((uint16_t)0x00000003)
#define EM_BLE_TXRATE_LSB      0
#define EM_BLE_TXRATE_WIDTH    ((uint16_t)0x00000002)

#define EM_BLE_RXTHR_RST       0x0
#define EM_BLE_TXTHR_RST       0x0
#define EM_BLE_AUX_RATE_RST    0x0
#define EM_BLE_RXRATE_RST      0x0
#define EM_BLE_TXRATE_RST      0x0

__INLINE void em_ble_thrcntl_ratecntl_pack(int elt_idx, uint8_t rxthr, uint8_t txthr, uint8_t auxrate, uint8_t rxrate, uint8_t txrate)
{
    ASSERT_ERR((((uint16_t)rxthr << 12) & ~((uint16_t)0x0000F000)) == 0);
    ASSERT_ERR((((uint16_t)txthr << 8) & ~((uint16_t)0x00000F00)) == 0);
    ASSERT_ERR((((uint16_t)auxrate << 4) & ~((uint16_t)0x00000030)) == 0);
    ASSERT_ERR((((uint16_t)rxrate << 2) & ~((uint16_t)0x0000000C)) == 0);
    ASSERT_ERR((((uint16_t)txrate << 0) & ~((uint16_t)0x00000003)) == 0);
    EM_BLE_WR(EM_BLE_THRCNTL_RATECNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, ((uint16_t)rxthr << 12) | ((uint16_t)txthr << 8) | ((uint16_t)auxrate << 4) | ((uint16_t)rxrate << 2) | ((uint16_t)txrate << 0));
}

__INLINE void em_ble_thrcntl_ratecntl_unpack(int elt_idx, uint8_t *rxthr, uint8_t *txthr, uint8_t *auxrate, uint8_t *rxrate, uint8_t *txrate)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_THRCNTL_RATECNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);

    *rxthr = (localVal & ((uint16_t)0x0000F000)) >> 12;
    *txthr = (localVal & ((uint16_t)0x00000F00)) >> 8;
    *auxrate = (localVal & ((uint16_t)0x00000030)) >> 4;
    *rxrate = (localVal & ((uint16_t)0x0000000C)) >> 2;
    *txrate = (localVal & ((uint16_t)0x00000003)) >> 0;
}

__INLINE uint8_t em_ble_thrcntl_ratecntl_rxthr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_THRCNTL_RATECNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000F000)) >> 12);
}

__INLINE void em_ble_thrcntl_ratecntl_rxthr_setf(int elt_idx, uint8_t rxthr)
{
    ASSERT_ERR((((uint16_t)rxthr << 12) & ~((uint16_t)0x0000F000)) == 0);
    EM_BLE_WR(EM_BLE_THRCNTL_RATECNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_THRCNTL_RATECNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x0000F000)) | ((uint16_t)rxthr << 12));
}

__INLINE uint8_t em_ble_thrcntl_ratecntl_txthr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_THRCNTL_RATECNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000F00)) >> 8);
}

__INLINE void em_ble_thrcntl_ratecntl_txthr_setf(int elt_idx, uint8_t txthr)
{
    ASSERT_ERR((((uint16_t)txthr << 8) & ~((uint16_t)0x00000F00)) == 0);
    EM_BLE_WR(EM_BLE_THRCNTL_RATECNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_THRCNTL_RATECNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000F00)) | ((uint16_t)txthr << 8));
}

__INLINE uint8_t em_ble_thrcntl_ratecntl_aux_rate_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_THRCNTL_RATECNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000030)) >> 4);
}

__INLINE void em_ble_thrcntl_ratecntl_aux_rate_setf(int elt_idx, uint8_t auxrate)
{
    ASSERT_ERR((((uint16_t)auxrate << 4) & ~((uint16_t)0x00000030)) == 0);
    EM_BLE_WR(EM_BLE_THRCNTL_RATECNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_THRCNTL_RATECNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000030)) | ((uint16_t)auxrate << 4));
}

__INLINE uint8_t em_ble_thrcntl_ratecntl_rxrate_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_THRCNTL_RATECNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000000C)) >> 2);
}

__INLINE void em_ble_thrcntl_ratecntl_rxrate_setf(int elt_idx, uint8_t rxrate)
{
    ASSERT_ERR((((uint16_t)rxrate << 2) & ~((uint16_t)0x0000000C)) == 0);
    EM_BLE_WR(EM_BLE_THRCNTL_RATECNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_THRCNTL_RATECNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x0000000C)) | ((uint16_t)rxrate << 2));
}

__INLINE uint8_t em_ble_thrcntl_ratecntl_txrate_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_THRCNTL_RATECNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000003)) >> 0);
}

__INLINE void em_ble_thrcntl_ratecntl_txrate_setf(int elt_idx, uint8_t txrate)
{
    ASSERT_ERR((((uint16_t)txrate << 0) & ~((uint16_t)0x00000003)) == 0);
    EM_BLE_WR(EM_BLE_THRCNTL_RATECNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_THRCNTL_RATECNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000003)) | ((uint16_t)txrate << 0));
}

/**
 * @brief LEBDADDR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00               BDADDR   0x0
 * </pre>
 */
#define EM_BLE_LEBDADDR_ADDR   (0x204F0008 + EM_BLE_CS_OFFSET)
#define EM_BLE_LEBDADDR_INDEX  0x00000004
#define EM_BLE_LEBDADDR_RESET  0x00000000
#define EM_BLE_LEBDADDR_COUNT  3

__INLINE uint16_t em_ble_lebdaddr_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 2);
    return EM_BLE_RD(EM_BLE_LEBDADDR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
}

__INLINE void em_ble_lebdaddr_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 2);
    EM_BLE_WR(EM_BLE_LEBDADDR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BLE_BDADDR_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_BDADDR_LSB    0
#define EM_BLE_BDADDR_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_BDADDR_RST    0x0

__INLINE uint16_t em_ble_lebdaddr_bdaddr_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 2);
    uint16_t localVal = EM_BLE_RD(EM_BLE_LEBDADDR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_lebdaddr_bdaddr_setf(int elt_idx, int reg_idx, uint16_t bdaddr)
{
    ASSERT_ERR(reg_idx <= 2);
    ASSERT_ERR((((uint16_t)bdaddr << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_LEBDADDR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, (uint16_t)bdaddr << 0);
}

/**
 * @brief SYNCWL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00            SYNCWORDL   0x0
 * </pre>
 */
#define EM_BLE_SYNCWL_ADDR   (0x204F000E + EM_BLE_CS_OFFSET)
#define EM_BLE_SYNCWL_INDEX  0x00000007
#define EM_BLE_SYNCWL_RESET  0x00000000

__INLINE uint16_t em_ble_syncwl_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_SYNCWL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_syncwl_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_SYNCWL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_SYNCWORDL_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_SYNCWORDL_LSB    0
#define EM_BLE_SYNCWORDL_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_SYNCWORDL_RST    0x0

__INLINE uint16_t em_ble_syncwl_syncwordl_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_SYNCWL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_syncwl_syncwordl_setf(int elt_idx, uint16_t syncwordl)
{
    ASSERT_ERR((((uint16_t)syncwordl << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_SYNCWL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)syncwordl << 0);
}

/**
 * @brief SYNCWH register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00            SYNCWORDH   0x0
 * </pre>
 */
#define EM_BLE_SYNCWH_ADDR   (0x204F0010 + EM_BLE_CS_OFFSET)
#define EM_BLE_SYNCWH_INDEX  0x00000008
#define EM_BLE_SYNCWH_RESET  0x00000000

__INLINE uint16_t em_ble_syncwh_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_SYNCWH_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_syncwh_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_SYNCWH_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_SYNCWORDH_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_SYNCWORDH_LSB    0
#define EM_BLE_SYNCWORDH_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_SYNCWORDH_RST    0x0

__INLINE uint16_t em_ble_syncwh_syncwordh_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_SYNCWH_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_syncwh_syncwordh_setf(int elt_idx, uint16_t syncwordh)
{
    ASSERT_ERR((((uint16_t)syncwordh << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_SYNCWH_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)syncwordh << 0);
}

/**
 * @brief CRCINIT0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00             CRCINIT0   0x0
 * </pre>
 */
#define EM_BLE_CRCINIT0_ADDR   (0x204F0012 + EM_BLE_CS_OFFSET)
#define EM_BLE_CRCINIT0_INDEX  0x00000009
#define EM_BLE_CRCINIT0_RESET  0x00000000

__INLINE uint16_t em_ble_crcinit0_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_CRCINIT0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_crcinit0_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_CRCINIT0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_CRCINIT0_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_CRCINIT0_LSB    0
#define EM_BLE_CRCINIT0_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_CRCINIT0_RST    0x0

__INLINE uint16_t em_ble_crcinit0_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_CRCINIT0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_crcinit0_setf(int elt_idx, uint16_t crcinit0)
{
    ASSERT_ERR((((uint16_t)crcinit0 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_CRCINIT0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)crcinit0 << 0);
}

/**
 * @brief CRCINIT1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:08          RXMAXCTEBUF   0x0
 *  07:00             CRCINIT1   0x0
 * </pre>
 */
#define EM_BLE_CRCINIT1_ADDR   (0x204F0014 + EM_BLE_CS_OFFSET)
#define EM_BLE_CRCINIT1_INDEX  0x0000000A
#define EM_BLE_CRCINIT1_RESET  0x00000000

__INLINE uint16_t em_ble_crcinit1_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_CRCINIT1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_crcinit1_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_CRCINIT1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_RXMAXCTEBUF_MASK   ((uint16_t)0x0000FF00)
#define EM_BLE_RXMAXCTEBUF_LSB    8
#define EM_BLE_RXMAXCTEBUF_WIDTH  ((uint16_t)0x00000008)
#define EM_BLE_CRCINIT1_MASK      ((uint16_t)0x000000FF)
#define EM_BLE_CRCINIT1_LSB       0
#define EM_BLE_CRCINIT1_WIDTH     ((uint16_t)0x00000008)

#define EM_BLE_RXMAXCTEBUF_RST    0x0
#define EM_BLE_CRCINIT1_RST       0x0

__INLINE void em_ble_crcinit1_pack(int elt_idx, uint8_t rxmaxctebuf, uint8_t crcinit1)
{
    ASSERT_ERR((((uint16_t)rxmaxctebuf << 8) & ~((uint16_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint16_t)crcinit1 << 0) & ~((uint16_t)0x000000FF)) == 0);
    EM_BLE_WR(EM_BLE_CRCINIT1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, ((uint16_t)rxmaxctebuf << 8) | ((uint16_t)crcinit1 << 0));
}

__INLINE void em_ble_crcinit1_unpack(int elt_idx, uint8_t *rxmaxctebuf, uint8_t *crcinit1)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_CRCINIT1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);

    *rxmaxctebuf = (localVal & ((uint16_t)0x0000FF00)) >> 8;
    *crcinit1 = (localVal & ((uint16_t)0x000000FF)) >> 0;
}

__INLINE uint8_t em_ble_crcinit1_rxmaxctebuf_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_CRCINIT1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000FF00)) >> 8);
}

__INLINE void em_ble_crcinit1_rxmaxctebuf_setf(int elt_idx, uint8_t rxmaxctebuf)
{
    ASSERT_ERR((((uint16_t)rxmaxctebuf << 8) & ~((uint16_t)0x0000FF00)) == 0);
    EM_BLE_WR(EM_BLE_CRCINIT1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_CRCINIT1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x0000FF00)) | ((uint16_t)rxmaxctebuf << 8));
}

__INLINE uint8_t em_ble_crcinit1_crcinit1_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_CRCINIT1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x000000FF)) >> 0);
}

__INLINE void em_ble_crcinit1_crcinit1_setf(int elt_idx, uint8_t crcinit1)
{
    ASSERT_ERR((((uint16_t)crcinit1 << 0) & ~((uint16_t)0x000000FF)) == 0);
    EM_BLE_WR(EM_BLE_CRCINIT1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_CRCINIT1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x000000FF)) | ((uint16_t)crcinit1 << 0));
}

/**
 * @brief FILTPOL_RALCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:08        FILTER_POLICY   0x0
 *     04         RAL_RESOL_EN   0
 *     03       PERADV_FILT_EN   0
 *     02        LOCAL_RPA_SEL   0
 *     01             RAL_MODE   0
 *     00               RAL_EN   0
 * </pre>
 */
#define EM_BLE_FILTPOL_RALCNTL_ADDR   (0x204F0016 + EM_BLE_CS_OFFSET)
#define EM_BLE_FILTPOL_RALCNTL_INDEX  0x0000000B
#define EM_BLE_FILTPOL_RALCNTL_RESET  0x00000000

__INLINE uint16_t em_ble_filtpol_ralcntl_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_filtpol_ralcntl_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_FILTER_POLICY_MASK    ((uint16_t)0x0000FF00)
#define EM_BLE_FILTER_POLICY_LSB     8
#define EM_BLE_FILTER_POLICY_WIDTH   ((uint16_t)0x00000008)
#define EM_BLE_RAL_RESOL_EN_BIT      ((uint16_t)0x00000010)
#define EM_BLE_RAL_RESOL_EN_POS      4
#define EM_BLE_PERADV_FILT_EN_BIT    ((uint16_t)0x00000008)
#define EM_BLE_PERADV_FILT_EN_POS    3
#define EM_BLE_LOCAL_RPA_SEL_BIT     ((uint16_t)0x00000004)
#define EM_BLE_LOCAL_RPA_SEL_POS     2
#define EM_BLE_RAL_MODE_BIT          ((uint16_t)0x00000002)
#define EM_BLE_RAL_MODE_POS          1
#define EM_BLE_RAL_EN_BIT            ((uint16_t)0x00000001)
#define EM_BLE_RAL_EN_POS            0

#define EM_BLE_FILTER_POLICY_RST     0x0
#define EM_BLE_RAL_RESOL_EN_RST      0x0
#define EM_BLE_PERADV_FILT_EN_RST    0x0
#define EM_BLE_LOCAL_RPA_SEL_RST     0x0
#define EM_BLE_RAL_MODE_RST          0x0
#define EM_BLE_RAL_EN_RST            0x0

__INLINE void em_ble_filtpol_ralcntl_pack(int elt_idx, uint8_t filterpolicy, uint8_t ralresolen, uint8_t peradvfilten, uint8_t localrpasel, uint8_t ralmode, uint8_t ralen)
{
    ASSERT_ERR((((uint16_t)filterpolicy << 8) & ~((uint16_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint16_t)ralresolen << 4) & ~((uint16_t)0x00000010)) == 0);
    ASSERT_ERR((((uint16_t)peradvfilten << 3) & ~((uint16_t)0x00000008)) == 0);
    ASSERT_ERR((((uint16_t)localrpasel << 2) & ~((uint16_t)0x00000004)) == 0);
    ASSERT_ERR((((uint16_t)ralmode << 1) & ~((uint16_t)0x00000002)) == 0);
    ASSERT_ERR((((uint16_t)ralen << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BLE_WR(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, ((uint16_t)filterpolicy << 8) | ((uint16_t)ralresolen << 4) | ((uint16_t)peradvfilten << 3) | ((uint16_t)localrpasel << 2) | ((uint16_t)ralmode << 1) | ((uint16_t)ralen << 0));
}

__INLINE void em_ble_filtpol_ralcntl_unpack(int elt_idx, uint8_t *filterpolicy, uint8_t *ralresolen, uint8_t *peradvfilten, uint8_t *localrpasel, uint8_t *ralmode, uint8_t *ralen)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);

    *filterpolicy = (localVal & ((uint16_t)0x0000FF00)) >> 8;
    *ralresolen = (localVal & ((uint16_t)0x00000010)) >> 4;
    *peradvfilten = (localVal & ((uint16_t)0x00000008)) >> 3;
    *localrpasel = (localVal & ((uint16_t)0x00000004)) >> 2;
    *ralmode = (localVal & ((uint16_t)0x00000002)) >> 1;
    *ralen = (localVal & ((uint16_t)0x00000001)) >> 0;
}

__INLINE uint8_t em_ble_filtpol_ralcntl_filter_policy_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000FF00)) >> 8);
}

__INLINE void em_ble_filtpol_ralcntl_filter_policy_setf(int elt_idx, uint8_t filterpolicy)
{
    ASSERT_ERR((((uint16_t)filterpolicy << 8) & ~((uint16_t)0x0000FF00)) == 0);
    EM_BLE_WR(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x0000FF00)) | ((uint16_t)filterpolicy << 8));
}

__INLINE uint8_t em_ble_filtpol_ralcntl_ral_resol_en_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000010)) >> 4);
}

__INLINE void em_ble_filtpol_ralcntl_ral_resol_en_setf(int elt_idx, uint8_t ralresolen)
{
    ASSERT_ERR((((uint16_t)ralresolen << 4) & ~((uint16_t)0x00000010)) == 0);
    EM_BLE_WR(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000010)) | ((uint16_t)ralresolen << 4));
}

__INLINE uint8_t em_ble_filtpol_ralcntl_peradv_filt_en_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000008)) >> 3);
}

__INLINE void em_ble_filtpol_ralcntl_peradv_filt_en_setf(int elt_idx, uint8_t peradvfilten)
{
    ASSERT_ERR((((uint16_t)peradvfilten << 3) & ~((uint16_t)0x00000008)) == 0);
    EM_BLE_WR(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000008)) | ((uint16_t)peradvfilten << 3));
}

__INLINE uint8_t em_ble_filtpol_ralcntl_local_rpa_sel_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000004)) >> 2);
}

__INLINE void em_ble_filtpol_ralcntl_local_rpa_sel_setf(int elt_idx, uint8_t localrpasel)
{
    ASSERT_ERR((((uint16_t)localrpasel << 2) & ~((uint16_t)0x00000004)) == 0);
    EM_BLE_WR(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000004)) | ((uint16_t)localrpasel << 2));
}

__INLINE uint8_t em_ble_filtpol_ralcntl_ral_mode_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000002)) >> 1);
}

__INLINE void em_ble_filtpol_ralcntl_ral_mode_setf(int elt_idx, uint8_t ralmode)
{
    ASSERT_ERR((((uint16_t)ralmode << 1) & ~((uint16_t)0x00000002)) == 0);
    EM_BLE_WR(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000002)) | ((uint16_t)ralmode << 1));
}

__INLINE uint8_t em_ble_filtpol_ralcntl_ral_en_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000001)) >> 0);
}

__INLINE void em_ble_filtpol_ralcntl_ral_en_setf(int elt_idx, uint8_t ralen)
{
    ASSERT_ERR((((uint16_t)ralen << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BLE_WR(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_FILTPOL_RALCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000001)) | ((uint16_t)ralen << 0));
}

/**
 * @brief HOPCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15                FH_EN   0
 *  14:13             HOP_MODE   0x0
 *  12:08              HOP_INT   0x0
 *  05:00               CH_IDX   0x0
 * </pre>
 */
#define EM_BLE_HOPCNTL_ADDR   (0x204F0018 + EM_BLE_CS_OFFSET)
#define EM_BLE_HOPCNTL_INDEX  0x0000000C
#define EM_BLE_HOPCNTL_RESET  0x00000000

__INLINE uint16_t em_ble_hopcntl_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_HOPCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_hopcntl_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_HOPCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_FH_EN_BIT       ((uint16_t)0x00008000)
#define EM_BLE_FH_EN_POS       15
#define EM_BLE_HOP_MODE_MASK   ((uint16_t)0x00006000)
#define EM_BLE_HOP_MODE_LSB    13
#define EM_BLE_HOP_MODE_WIDTH  ((uint16_t)0x00000002)
#define EM_BLE_HOP_INT_MASK    ((uint16_t)0x00001F00)
#define EM_BLE_HOP_INT_LSB     8
#define EM_BLE_HOP_INT_WIDTH   ((uint16_t)0x00000005)
#define EM_BLE_CH_IDX_MASK     ((uint16_t)0x0000003F)
#define EM_BLE_CH_IDX_LSB      0
#define EM_BLE_CH_IDX_WIDTH    ((uint16_t)0x00000006)

#define EM_BLE_FH_EN_RST       0x0
#define EM_BLE_HOP_MODE_RST    0x0
#define EM_BLE_HOP_INT_RST     0x0
#define EM_BLE_CH_IDX_RST      0x0

__INLINE void em_ble_hopcntl_pack(int elt_idx, uint8_t fhen, uint8_t hopmode, uint8_t hopint, uint8_t chidx)
{
    ASSERT_ERR((((uint16_t)fhen << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)hopmode << 13) & ~((uint16_t)0x00006000)) == 0);
    ASSERT_ERR((((uint16_t)hopint << 8) & ~((uint16_t)0x00001F00)) == 0);
    ASSERT_ERR((((uint16_t)chidx << 0) & ~((uint16_t)0x0000003F)) == 0);
    EM_BLE_WR(EM_BLE_HOPCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, ((uint16_t)fhen << 15) | ((uint16_t)hopmode << 13) | ((uint16_t)hopint << 8) | ((uint16_t)chidx << 0));
}

__INLINE void em_ble_hopcntl_unpack(int elt_idx, uint8_t *fhen, uint8_t *hopmode, uint8_t *hopint, uint8_t *chidx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_HOPCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);

    *fhen = (localVal & ((uint16_t)0x00008000)) >> 15;
    *hopmode = (localVal & ((uint16_t)0x00006000)) >> 13;
    *hopint = (localVal & ((uint16_t)0x00001F00)) >> 8;
    *chidx = (localVal & ((uint16_t)0x0000003F)) >> 0;
}

__INLINE uint8_t em_ble_hopcntl_fh_en_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_HOPCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_ble_hopcntl_fh_en_setf(int elt_idx, uint8_t fhen)
{
    ASSERT_ERR((((uint16_t)fhen << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BLE_WR(EM_BLE_HOPCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_HOPCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)fhen << 15));
}

__INLINE uint8_t em_ble_hopcntl_hop_mode_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_HOPCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00006000)) >> 13);
}

__INLINE void em_ble_hopcntl_hop_mode_setf(int elt_idx, uint8_t hopmode)
{
    ASSERT_ERR((((uint16_t)hopmode << 13) & ~((uint16_t)0x00006000)) == 0);
    EM_BLE_WR(EM_BLE_HOPCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_HOPCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00006000)) | ((uint16_t)hopmode << 13));
}

__INLINE uint8_t em_ble_hopcntl_hop_int_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_HOPCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00001F00)) >> 8);
}

__INLINE void em_ble_hopcntl_hop_int_setf(int elt_idx, uint8_t hopint)
{
    ASSERT_ERR((((uint16_t)hopint << 8) & ~((uint16_t)0x00001F00)) == 0);
    EM_BLE_WR(EM_BLE_HOPCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_HOPCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00001F00)) | ((uint16_t)hopint << 8));
}

__INLINE uint8_t em_ble_hopcntl_ch_idx_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_HOPCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000003F)) >> 0);
}

__INLINE void em_ble_hopcntl_ch_idx_setf(int elt_idx, uint8_t chidx)
{
    ASSERT_ERR((((uint16_t)chidx << 0) & ~((uint16_t)0x0000003F)) == 0);
    EM_BLE_WR(EM_BLE_HOPCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_HOPCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x0000003F)) | ((uint16_t)chidx << 0));
}

/**
 * @brief TXRXCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15          RXBUFF_FULL   0
 *     14            LASTEMPTY   0
 *     13                   SN   0
 *     12                 NESN   0
 *     11           RXBFMICERR   0
 *     10            RXMAFSERR   0
 *     09          ENDS_ON_SAC   0
 *     08            EXT_PA_EN   0
 *  07:00                TXPWR   0x0
 * </pre>
 */
#define EM_BLE_TXRXCNTL_ADDR   (0x204F001A + EM_BLE_CS_OFFSET)
#define EM_BLE_TXRXCNTL_INDEX  0x0000000D
#define EM_BLE_TXRXCNTL_RESET  0x00000000

__INLINE uint16_t em_ble_txrxcntl_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_txrxcntl_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_RXBUFF_FULL_BIT    ((uint16_t)0x00008000)
#define EM_BLE_RXBUFF_FULL_POS    15
#define EM_BLE_LASTEMPTY_BIT      ((uint16_t)0x00004000)
#define EM_BLE_LASTEMPTY_POS      14
#define EM_BLE_SN_BIT             ((uint16_t)0x00002000)
#define EM_BLE_SN_POS             13
#define EM_BLE_NESN_BIT           ((uint16_t)0x00001000)
#define EM_BLE_NESN_POS           12
#define EM_BLE_RXBFMICERR_BIT     ((uint16_t)0x00000800)
#define EM_BLE_RXBFMICERR_POS     11
#define EM_BLE_RXMAFSERR_BIT      ((uint16_t)0x00000400)
#define EM_BLE_RXMAFSERR_POS      10
#define EM_BLE_ENDS_ON_SAC_BIT    ((uint16_t)0x00000200)
#define EM_BLE_ENDS_ON_SAC_POS    9
#define EM_BLE_EXT_PA_EN_BIT      ((uint16_t)0x00000100)
#define EM_BLE_EXT_PA_EN_POS      8
#define EM_BLE_TXPWR_MASK         ((uint16_t)0x000000FF)
#define EM_BLE_TXPWR_LSB          0
#define EM_BLE_TXPWR_WIDTH        ((uint16_t)0x00000008)

#define EM_BLE_RXBUFF_FULL_RST    0x0
#define EM_BLE_LASTEMPTY_RST      0x0
#define EM_BLE_SN_RST             0x0
#define EM_BLE_NESN_RST           0x0
#define EM_BLE_RXBFMICERR_RST     0x0
#define EM_BLE_RXMAFSERR_RST      0x0
#define EM_BLE_ENDS_ON_SAC_RST    0x0
#define EM_BLE_EXT_PA_EN_RST      0x0
#define EM_BLE_TXPWR_RST          0x0

__INLINE void em_ble_txrxcntl_pack(int elt_idx, uint8_t rxbufffull, uint8_t lastempty, uint8_t sn, uint8_t nesn, uint8_t rxbfmicerr, uint8_t rxmafserr, uint8_t endsonsac, uint8_t extpaen, uint8_t txpwr)
{
    ASSERT_ERR((((uint16_t)rxbufffull << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)lastempty << 14) & ~((uint16_t)0x00004000)) == 0);
    ASSERT_ERR((((uint16_t)sn << 13) & ~((uint16_t)0x00002000)) == 0);
    ASSERT_ERR((((uint16_t)nesn << 12) & ~((uint16_t)0x00001000)) == 0);
    ASSERT_ERR((((uint16_t)rxbfmicerr << 11) & ~((uint16_t)0x00000800)) == 0);
    ASSERT_ERR((((uint16_t)rxmafserr << 10) & ~((uint16_t)0x00000400)) == 0);
    ASSERT_ERR((((uint16_t)endsonsac << 9) & ~((uint16_t)0x00000200)) == 0);
    ASSERT_ERR((((uint16_t)extpaen << 8) & ~((uint16_t)0x00000100)) == 0);
    ASSERT_ERR((((uint16_t)txpwr << 0) & ~((uint16_t)0x000000FF)) == 0);
    EM_BLE_WR(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, ((uint16_t)rxbufffull << 15) | ((uint16_t)lastempty << 14) | ((uint16_t)sn << 13) | ((uint16_t)nesn << 12) | ((uint16_t)rxbfmicerr << 11) | ((uint16_t)rxmafserr << 10) | ((uint16_t)endsonsac << 9) | ((uint16_t)extpaen << 8) | ((uint16_t)txpwr << 0));
}

__INLINE void em_ble_txrxcntl_unpack(int elt_idx, uint8_t *rxbufffull, uint8_t *lastempty, uint8_t *sn, uint8_t *nesn, uint8_t *rxbfmicerr, uint8_t *rxmafserr, uint8_t *endsonsac, uint8_t *extpaen, uint8_t *txpwr)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);

    *rxbufffull = (localVal & ((uint16_t)0x00008000)) >> 15;
    *lastempty = (localVal & ((uint16_t)0x00004000)) >> 14;
    *sn = (localVal & ((uint16_t)0x00002000)) >> 13;
    *nesn = (localVal & ((uint16_t)0x00001000)) >> 12;
    *rxbfmicerr = (localVal & ((uint16_t)0x00000800)) >> 11;
    *rxmafserr = (localVal & ((uint16_t)0x00000400)) >> 10;
    *endsonsac = (localVal & ((uint16_t)0x00000200)) >> 9;
    *extpaen = (localVal & ((uint16_t)0x00000100)) >> 8;
    *txpwr = (localVal & ((uint16_t)0x000000FF)) >> 0;
}

__INLINE uint8_t em_ble_txrxcntl_rxbuff_full_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_ble_txrxcntl_rxbuff_full_setf(int elt_idx, uint8_t rxbufffull)
{
    ASSERT_ERR((((uint16_t)rxbufffull << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BLE_WR(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)rxbufffull << 15));
}

__INLINE uint8_t em_ble_txrxcntl_lastempty_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00004000)) >> 14);
}

__INLINE void em_ble_txrxcntl_lastempty_setf(int elt_idx, uint8_t lastempty)
{
    ASSERT_ERR((((uint16_t)lastempty << 14) & ~((uint16_t)0x00004000)) == 0);
    EM_BLE_WR(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00004000)) | ((uint16_t)lastempty << 14));
}

__INLINE uint8_t em_ble_txrxcntl_sn_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00002000)) >> 13);
}

__INLINE void em_ble_txrxcntl_sn_setf(int elt_idx, uint8_t sn)
{
    ASSERT_ERR((((uint16_t)sn << 13) & ~((uint16_t)0x00002000)) == 0);
    EM_BLE_WR(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00002000)) | ((uint16_t)sn << 13));
}

__INLINE uint8_t em_ble_txrxcntl_nesn_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00001000)) >> 12);
}

__INLINE void em_ble_txrxcntl_nesn_setf(int elt_idx, uint8_t nesn)
{
    ASSERT_ERR((((uint16_t)nesn << 12) & ~((uint16_t)0x00001000)) == 0);
    EM_BLE_WR(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00001000)) | ((uint16_t)nesn << 12));
}

__INLINE uint8_t em_ble_txrxcntl_rxbfmicerr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000800)) >> 11);
}

__INLINE void em_ble_txrxcntl_rxbfmicerr_setf(int elt_idx, uint8_t rxbfmicerr)
{
    ASSERT_ERR((((uint16_t)rxbfmicerr << 11) & ~((uint16_t)0x00000800)) == 0);
    EM_BLE_WR(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000800)) | ((uint16_t)rxbfmicerr << 11));
}

__INLINE uint8_t em_ble_txrxcntl_rxmafserr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000400)) >> 10);
}

__INLINE void em_ble_txrxcntl_rxmafserr_setf(int elt_idx, uint8_t rxmafserr)
{
    ASSERT_ERR((((uint16_t)rxmafserr << 10) & ~((uint16_t)0x00000400)) == 0);
    EM_BLE_WR(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000400)) | ((uint16_t)rxmafserr << 10));
}

__INLINE uint8_t em_ble_txrxcntl_ends_on_sac_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000200)) >> 9);
}

__INLINE void em_ble_txrxcntl_ends_on_sac_setf(int elt_idx, uint8_t endsonsac)
{
    ASSERT_ERR((((uint16_t)endsonsac << 9) & ~((uint16_t)0x00000200)) == 0);
    EM_BLE_WR(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000200)) | ((uint16_t)endsonsac << 9));
}

__INLINE uint8_t em_ble_txrxcntl_ext_pa_en_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000100)) >> 8);
}

__INLINE void em_ble_txrxcntl_ext_pa_en_setf(int elt_idx, uint8_t extpaen)
{
    ASSERT_ERR((((uint16_t)extpaen << 8) & ~((uint16_t)0x00000100)) == 0);
    EM_BLE_WR(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000100)) | ((uint16_t)extpaen << 8));
}

__INLINE uint8_t em_ble_txrxcntl_txpwr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x000000FF)) >> 0);
}

__INLINE void em_ble_txrxcntl_txpwr_setf(int elt_idx, uint8_t txpwr)
{
    ASSERT_ERR((((uint16_t)txpwr << 0) & ~((uint16_t)0x000000FF)) == 0);
    EM_BLE_WR(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_TXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x000000FF)) | ((uint16_t)txpwr << 0));
}

/**
 * @brief RXDFCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     08              DFRSPEN   0
 *  07:06             DFSWCNTL   0x0
 *  05:04           DFSAMPCNTL   0x0
 *  03:02               DFTYPE   0x0
 *     01           DFFILTEREN   0
 *     00                 DFEN   0
 * </pre>
 */
#define EM_BLE_RXDFCNTL_ADDR   (0x204F001C + EM_BLE_CS_OFFSET)
#define EM_BLE_RXDFCNTL_INDEX  0x0000000E
#define EM_BLE_RXDFCNTL_RESET  0x00000000

__INLINE uint16_t em_ble_rxdfcntl_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_rxdfcntl_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_DFRSPEN_BIT       ((uint16_t)0x00000100)
#define EM_BLE_DFRSPEN_POS       8
#define EM_BLE_DFSWCNTL_MASK     ((uint16_t)0x000000C0)
#define EM_BLE_DFSWCNTL_LSB      6
#define EM_BLE_DFSWCNTL_WIDTH    ((uint16_t)0x00000002)
#define EM_BLE_DFSAMPCNTL_MASK   ((uint16_t)0x00000030)
#define EM_BLE_DFSAMPCNTL_LSB    4
#define EM_BLE_DFSAMPCNTL_WIDTH  ((uint16_t)0x00000002)
#define EM_BLE_DFTYPE_MASK       ((uint16_t)0x0000000C)
#define EM_BLE_DFTYPE_LSB        2
#define EM_BLE_DFTYPE_WIDTH      ((uint16_t)0x00000002)
#define EM_BLE_DFFILTEREN_BIT    ((uint16_t)0x00000002)
#define EM_BLE_DFFILTEREN_POS    1
#define EM_BLE_DFEN_BIT          ((uint16_t)0x00000001)
#define EM_BLE_DFEN_POS          0

#define EM_BLE_DFRSPEN_RST       0x0
#define EM_BLE_DFSWCNTL_RST      0x0
#define EM_BLE_DFSAMPCNTL_RST    0x0
#define EM_BLE_DFTYPE_RST        0x0
#define EM_BLE_DFFILTEREN_RST    0x0
#define EM_BLE_DFEN_RST          0x0

__INLINE void em_ble_rxdfcntl_pack(int elt_idx, uint8_t dfrspen, uint8_t dfswcntl, uint8_t dfsampcntl, uint8_t dftype, uint8_t dffilteren, uint8_t dfen)
{
    ASSERT_ERR((((uint16_t)dfrspen << 8) & ~((uint16_t)0x00000100)) == 0);
    ASSERT_ERR((((uint16_t)dfswcntl << 6) & ~((uint16_t)0x000000C0)) == 0);
    ASSERT_ERR((((uint16_t)dfsampcntl << 4) & ~((uint16_t)0x00000030)) == 0);
    ASSERT_ERR((((uint16_t)dftype << 2) & ~((uint16_t)0x0000000C)) == 0);
    ASSERT_ERR((((uint16_t)dffilteren << 1) & ~((uint16_t)0x00000002)) == 0);
    ASSERT_ERR((((uint16_t)dfen << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BLE_WR(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, ((uint16_t)dfrspen << 8) | ((uint16_t)dfswcntl << 6) | ((uint16_t)dfsampcntl << 4) | ((uint16_t)dftype << 2) | ((uint16_t)dffilteren << 1) | ((uint16_t)dfen << 0));
}

__INLINE void em_ble_rxdfcntl_unpack(int elt_idx, uint8_t *dfrspen, uint8_t *dfswcntl, uint8_t *dfsampcntl, uint8_t *dftype, uint8_t *dffilteren, uint8_t *dfen)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);

    *dfrspen = (localVal & ((uint16_t)0x00000100)) >> 8;
    *dfswcntl = (localVal & ((uint16_t)0x000000C0)) >> 6;
    *dfsampcntl = (localVal & ((uint16_t)0x00000030)) >> 4;
    *dftype = (localVal & ((uint16_t)0x0000000C)) >> 2;
    *dffilteren = (localVal & ((uint16_t)0x00000002)) >> 1;
    *dfen = (localVal & ((uint16_t)0x00000001)) >> 0;
}

__INLINE uint8_t em_ble_rxdfcntl_dfrspen_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000100)) >> 8);
}

__INLINE void em_ble_rxdfcntl_dfrspen_setf(int elt_idx, uint8_t dfrspen)
{
    ASSERT_ERR((((uint16_t)dfrspen << 8) & ~((uint16_t)0x00000100)) == 0);
    EM_BLE_WR(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000100)) | ((uint16_t)dfrspen << 8));
}

__INLINE uint8_t em_ble_rxdfcntl_dfswcntl_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x000000C0)) >> 6);
}

__INLINE void em_ble_rxdfcntl_dfswcntl_setf(int elt_idx, uint8_t dfswcntl)
{
    ASSERT_ERR((((uint16_t)dfswcntl << 6) & ~((uint16_t)0x000000C0)) == 0);
    EM_BLE_WR(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x000000C0)) | ((uint16_t)dfswcntl << 6));
}

__INLINE uint8_t em_ble_rxdfcntl_dfsampcntl_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000030)) >> 4);
}

__INLINE void em_ble_rxdfcntl_dfsampcntl_setf(int elt_idx, uint8_t dfsampcntl)
{
    ASSERT_ERR((((uint16_t)dfsampcntl << 4) & ~((uint16_t)0x00000030)) == 0);
    EM_BLE_WR(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000030)) | ((uint16_t)dfsampcntl << 4));
}

__INLINE uint8_t em_ble_rxdfcntl_dftype_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000000C)) >> 2);
}

__INLINE void em_ble_rxdfcntl_dftype_setf(int elt_idx, uint8_t dftype)
{
    ASSERT_ERR((((uint16_t)dftype << 2) & ~((uint16_t)0x0000000C)) == 0);
    EM_BLE_WR(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x0000000C)) | ((uint16_t)dftype << 2));
}

__INLINE uint8_t em_ble_rxdfcntl_dffilteren_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000002)) >> 1);
}

__INLINE void em_ble_rxdfcntl_dffilteren_setf(int elt_idx, uint8_t dffilteren)
{
    ASSERT_ERR((((uint16_t)dffilteren << 1) & ~((uint16_t)0x00000002)) == 0);
    EM_BLE_WR(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000002)) | ((uint16_t)dffilteren << 1));
}

__INLINE uint8_t em_ble_rxdfcntl_dfen_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000001)) >> 0);
}

__INLINE void em_ble_rxdfcntl_dfen_setf(int elt_idx, uint8_t dfen)
{
    ASSERT_ERR((((uint16_t)dfen << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BLE_WR(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_RXDFCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000001)) | ((uint16_t)dfen << 0));
}

/**
 * @brief RXWINCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15               RXWIDE   0
 *  14:00              RXWINSZ   0x0
 * </pre>
 */
#define EM_BLE_RXWINCNTL_ADDR   (0x204F001E + EM_BLE_CS_OFFSET)
#define EM_BLE_RXWINCNTL_INDEX  0x0000000F
#define EM_BLE_RXWINCNTL_RESET  0x00000000

__INLINE uint16_t em_ble_rxwincntl_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXWINCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_rxwincntl_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXWINCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_RXWIDE_BIT     ((uint16_t)0x00008000)
#define EM_BLE_RXWIDE_POS     15
#define EM_BLE_RXWINSZ_MASK   ((uint16_t)0x00007FFF)
#define EM_BLE_RXWINSZ_LSB    0
#define EM_BLE_RXWINSZ_WIDTH  ((uint16_t)0x0000000F)

#define EM_BLE_RXWIDE_RST     0x0
#define EM_BLE_RXWINSZ_RST    0x0

__INLINE void em_ble_rxwincntl_pack(int elt_idx, uint8_t rxwide, uint16_t rxwinsz)
{
    ASSERT_ERR((((uint16_t)rxwide << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)rxwinsz << 0) & ~((uint16_t)0x00007FFF)) == 0);
    EM_BLE_WR(EM_BLE_RXWINCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, ((uint16_t)rxwide << 15) | ((uint16_t)rxwinsz << 0));
}

__INLINE void em_ble_rxwincntl_unpack(int elt_idx, uint8_t *rxwide, uint16_t *rxwinsz)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXWINCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);

    *rxwide = (localVal & ((uint16_t)0x00008000)) >> 15;
    *rxwinsz = (localVal & ((uint16_t)0x00007FFF)) >> 0;
}

__INLINE uint8_t em_ble_rxwincntl_rxwide_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXWINCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_ble_rxwincntl_rxwide_setf(int elt_idx, uint8_t rxwide)
{
    ASSERT_ERR((((uint16_t)rxwide << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BLE_WR(EM_BLE_RXWINCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_RXWINCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)rxwide << 15));
}

__INLINE uint16_t em_ble_rxwincntl_rxwinsz_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXWINCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00007FFF)) >> 0);
}

__INLINE void em_ble_rxwincntl_rxwinsz_setf(int elt_idx, uint16_t rxwinsz)
{
    ASSERT_ERR((((uint16_t)rxwinsz << 0) & ~((uint16_t)0x00007FFF)) == 0);
    EM_BLE_WR(EM_BLE_RXWINCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_RXWINCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00007FFF)) | ((uint16_t)rxwinsz << 0));
}

/**
 * @brief ISOTXDESCPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  13:00         ISOTXDESCPTR   0x0
 * </pre>
 */
#define EM_BLE_ISOTXDESCPTR_ADDR   (0x204F0020 + EM_BLE_CS_OFFSET)
#define EM_BLE_ISOTXDESCPTR_INDEX  0x00000010
#define EM_BLE_ISOTXDESCPTR_RESET  0x00000000

__INLINE uint16_t em_ble_isotxdescptr_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_ISOTXDESCPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_isotxdescptr_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_ISOTXDESCPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_ISOTXDESCPTR_MASK   ((uint16_t)0x00003FFF)
#define EM_BLE_ISOTXDESCPTR_LSB    0
#define EM_BLE_ISOTXDESCPTR_WIDTH  ((uint16_t)0x0000000E)

#define EM_BLE_ISOTXDESCPTR_RST    0x0

__INLINE uint16_t em_ble_isotxdescptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOTXDESCPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x00003FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_isotxdescptr_setf(int elt_idx, uint16_t isotxdescptr)
{
    ASSERT_ERR((((uint16_t)isotxdescptr << 0) & ~((uint16_t)0x00003FFF)) == 0);
    EM_BLE_WR(EM_BLE_ISOTXDESCPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)isotxdescptr << 0);
}

/**
 * @brief ISORXDESCPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  13:00         ISORXDESCPTR   0x0
 * </pre>
 */
#define EM_BLE_ISORXDESCPTR_ADDR   (0x204F0022 + EM_BLE_CS_OFFSET)
#define EM_BLE_ISORXDESCPTR_INDEX  0x00000011
#define EM_BLE_ISORXDESCPTR_RESET  0x00000000

__INLINE uint16_t em_ble_isorxdescptr_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_ISORXDESCPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_isorxdescptr_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_ISORXDESCPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_ISORXDESCPTR_MASK   ((uint16_t)0x00003FFF)
#define EM_BLE_ISORXDESCPTR_LSB    0
#define EM_BLE_ISORXDESCPTR_WIDTH  ((uint16_t)0x0000000E)

#define EM_BLE_ISORXDESCPTR_RST    0x0

__INLINE uint16_t em_ble_isorxdescptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISORXDESCPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x00003FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_isorxdescptr_setf(int elt_idx, uint16_t isorxdescptr)
{
    ASSERT_ERR((((uint16_t)isorxdescptr << 0) & ~((uint16_t)0x00003FFF)) == 0);
    EM_BLE_WR(EM_BLE_ISORXDESCPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)isorxdescptr << 0);
}

/**
 * @brief ACLTXDESCPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  13:00         ACLTXDESCPTR   0x0
 * </pre>
 */
#define EM_BLE_ACLTXDESCPTR_ADDR   (0x204F0024 + EM_BLE_CS_OFFSET)
#define EM_BLE_ACLTXDESCPTR_INDEX  0x00000012
#define EM_BLE_ACLTXDESCPTR_RESET  0x00000000

__INLINE uint16_t em_ble_acltxdescptr_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_ACLTXDESCPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_acltxdescptr_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_ACLTXDESCPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_ACLTXDESCPTR_MASK   ((uint16_t)0x00003FFF)
#define EM_BLE_ACLTXDESCPTR_LSB    0
#define EM_BLE_ACLTXDESCPTR_WIDTH  ((uint16_t)0x0000000E)

#define EM_BLE_ACLTXDESCPTR_RST    0x0

__INLINE uint16_t em_ble_acltxdescptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ACLTXDESCPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x00003FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_acltxdescptr_setf(int elt_idx, uint16_t acltxdescptr)
{
    ASSERT_ERR((((uint16_t)acltxdescptr << 0) & ~((uint16_t)0x00003FFF)) == 0);
    EM_BLE_WR(EM_BLE_ACLTXDESCPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)acltxdescptr << 0);
}

/**
 * @brief RXDFANTPATTCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15                DFRSP   0
 *  12:08         MAX_SAMP_CTE   0x0
 *  06:00   RX_ANT_PATT_LENGTH   0x0
 * </pre>
 */
#define EM_BLE_RXDFANTPATTCNTL_ADDR   (0x204F0026 + EM_BLE_CS_OFFSET)
#define EM_BLE_RXDFANTPATTCNTL_INDEX  0x00000013
#define EM_BLE_RXDFANTPATTCNTL_RESET  0x00000000

__INLINE uint16_t em_ble_rxdfantpattcntl_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXDFANTPATTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_rxdfantpattcntl_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXDFANTPATTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_DFRSP_BIT                 ((uint16_t)0x00008000)
#define EM_BLE_DFRSP_POS                 15
#define EM_BLE_MAX_SAMP_CTE_MASK         ((uint16_t)0x00001F00)
#define EM_BLE_MAX_SAMP_CTE_LSB          8
#define EM_BLE_MAX_SAMP_CTE_WIDTH        ((uint16_t)0x00000005)
#define EM_BLE_RX_ANT_PATT_LENGTH_MASK   ((uint16_t)0x0000007F)
#define EM_BLE_RX_ANT_PATT_LENGTH_LSB    0
#define EM_BLE_RX_ANT_PATT_LENGTH_WIDTH  ((uint16_t)0x00000007)

#define EM_BLE_DFRSP_RST                 0x0
#define EM_BLE_MAX_SAMP_CTE_RST          0x0
#define EM_BLE_RX_ANT_PATT_LENGTH_RST    0x0

__INLINE void em_ble_rxdfantpattcntl_pack(int elt_idx, uint8_t dfrsp, uint8_t maxsampcte, uint8_t rxantpattlength)
{
    ASSERT_ERR((((uint16_t)dfrsp << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)maxsampcte << 8) & ~((uint16_t)0x00001F00)) == 0);
    ASSERT_ERR((((uint16_t)rxantpattlength << 0) & ~((uint16_t)0x0000007F)) == 0);
    EM_BLE_WR(EM_BLE_RXDFANTPATTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, ((uint16_t)dfrsp << 15) | ((uint16_t)maxsampcte << 8) | ((uint16_t)rxantpattlength << 0));
}

__INLINE void em_ble_rxdfantpattcntl_unpack(int elt_idx, uint8_t *dfrsp, uint8_t *maxsampcte, uint8_t *rxantpattlength)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXDFANTPATTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);

    *dfrsp = (localVal & ((uint16_t)0x00008000)) >> 15;
    *maxsampcte = (localVal & ((uint16_t)0x00001F00)) >> 8;
    *rxantpattlength = (localVal & ((uint16_t)0x0000007F)) >> 0;
}

__INLINE uint8_t em_ble_rxdfantpattcntl_dfrsp_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXDFANTPATTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_ble_rxdfantpattcntl_dfrsp_setf(int elt_idx, uint8_t dfrsp)
{
    ASSERT_ERR((((uint16_t)dfrsp << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BLE_WR(EM_BLE_RXDFANTPATTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_RXDFANTPATTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)dfrsp << 15));
}

__INLINE uint8_t em_ble_rxdfantpattcntl_max_samp_cte_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXDFANTPATTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00001F00)) >> 8);
}

__INLINE void em_ble_rxdfantpattcntl_max_samp_cte_setf(int elt_idx, uint8_t maxsampcte)
{
    ASSERT_ERR((((uint16_t)maxsampcte << 8) & ~((uint16_t)0x00001F00)) == 0);
    EM_BLE_WR(EM_BLE_RXDFANTPATTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_RXDFANTPATTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00001F00)) | ((uint16_t)maxsampcte << 8));
}

__INLINE uint8_t em_ble_rxdfantpattcntl_rx_ant_patt_length_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXDFANTPATTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000007F)) >> 0);
}

__INLINE void em_ble_rxdfantpattcntl_rx_ant_patt_length_setf(int elt_idx, uint8_t rxantpattlength)
{
    ASSERT_ERR((((uint16_t)rxantpattlength << 0) & ~((uint16_t)0x0000007F)) == 0);
    EM_BLE_WR(EM_BLE_RXDFANTPATTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_RXDFANTPATTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x0000007F)) | ((uint16_t)rxantpattlength << 0));
}

/**
 * @brief RXDFANTSWPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  13:00    RX_ANTENNA_ID_PTR   0x0
 * </pre>
 */
#define EM_BLE_RXDFANTSWPTR_ADDR   (0x204F0028 + EM_BLE_CS_OFFSET)
#define EM_BLE_RXDFANTSWPTR_INDEX  0x00000014
#define EM_BLE_RXDFANTSWPTR_RESET  0x00000000

__INLINE uint16_t em_ble_rxdfantswptr_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXDFANTSWPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_rxdfantswptr_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXDFANTSWPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_RX_ANTENNA_ID_PTR_MASK   ((uint16_t)0x00003FFF)
#define EM_BLE_RX_ANTENNA_ID_PTR_LSB    0
#define EM_BLE_RX_ANTENNA_ID_PTR_WIDTH  ((uint16_t)0x0000000E)

#define EM_BLE_RX_ANTENNA_ID_PTR_RST    0x0

__INLINE uint16_t em_ble_rxdfantswptr_rx_antenna_id_ptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXDFANTSWPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x00003FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_rxdfantswptr_rx_antenna_id_ptr_setf(int elt_idx, uint16_t rxantennaidptr)
{
    ASSERT_ERR((((uint16_t)rxantennaidptr << 0) & ~((uint16_t)0x00003FFF)) == 0);
    EM_BLE_WR(EM_BLE_RXDFANTSWPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)rxantennaidptr << 0);
}

/**
 * @brief TXDFANTPATTCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  06:00   TX_ANT_PATT_LENGTH   0x0
 * </pre>
 */
#define EM_BLE_TXDFANTPATTCNTL_ADDR   (0x204F002A + EM_BLE_CS_OFFSET)
#define EM_BLE_TXDFANTPATTCNTL_INDEX  0x00000015
#define EM_BLE_TXDFANTPATTCNTL_RESET  0x00000000

__INLINE uint16_t em_ble_txdfantpattcntl_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_TXDFANTPATTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_txdfantpattcntl_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_TXDFANTPATTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_TX_ANT_PATT_LENGTH_MASK   ((uint16_t)0x0000007F)
#define EM_BLE_TX_ANT_PATT_LENGTH_LSB    0
#define EM_BLE_TX_ANT_PATT_LENGTH_WIDTH  ((uint16_t)0x00000007)

#define EM_BLE_TX_ANT_PATT_LENGTH_RST    0x0

__INLINE uint8_t em_ble_txdfantpattcntl_tx_ant_patt_length_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXDFANTPATTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000007F)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_txdfantpattcntl_tx_ant_patt_length_setf(int elt_idx, uint8_t txantpattlength)
{
    ASSERT_ERR((((uint16_t)txantpattlength << 0) & ~((uint16_t)0x0000007F)) == 0);
    EM_BLE_WR(EM_BLE_TXDFANTPATTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)txantpattlength << 0);
}

/**
 * @brief TXDFANTSWPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  13:00    TX_ANTENNA_ID_PTR   0x0
 * </pre>
 */
#define EM_BLE_TXDFANTSWPTR_ADDR   (0x204F002C + EM_BLE_CS_OFFSET)
#define EM_BLE_TXDFANTSWPTR_INDEX  0x00000016
#define EM_BLE_TXDFANTSWPTR_RESET  0x00000000

__INLINE uint16_t em_ble_txdfantswptr_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_TXDFANTSWPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_txdfantswptr_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_TXDFANTSWPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_TX_ANTENNA_ID_PTR_MASK   ((uint16_t)0x00003FFF)
#define EM_BLE_TX_ANTENNA_ID_PTR_LSB    0
#define EM_BLE_TX_ANTENNA_ID_PTR_WIDTH  ((uint16_t)0x0000000E)

#define EM_BLE_TX_ANTENNA_ID_PTR_RST    0x0

__INLINE uint16_t em_ble_txdfantswptr_tx_antenna_id_ptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXDFANTSWPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x00003FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_txdfantswptr_tx_antenna_id_ptr_setf(int elt_idx, uint16_t txantennaidptr)
{
    ASSERT_ERR((((uint16_t)txantennaidptr << 0) & ~((uint16_t)0x00003FFF)) == 0);
    EM_BLE_WR(EM_BLE_TXDFANTSWPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)txantennaidptr << 0);
}

/**
 * @brief WINOFFSET register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00            WINOFFSET   0x0
 * </pre>
 */
#define EM_BLE_WINOFFSET_ADDR   (0x204F002E + EM_BLE_CS_OFFSET)
#define EM_BLE_WINOFFSET_INDEX  0x00000017
#define EM_BLE_WINOFFSET_RESET  0x00000000

__INLINE uint16_t em_ble_winoffset_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_WINOFFSET_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_winoffset_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_WINOFFSET_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_WINOFFSET_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_WINOFFSET_LSB    0
#define EM_BLE_WINOFFSET_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_WINOFFSET_RST    0x0

__INLINE uint16_t em_ble_winoffset_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_WINOFFSET_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_winoffset_setf(int elt_idx, uint16_t winoffset)
{
    ASSERT_ERR((((uint16_t)winoffset << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_WINOFFSET_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)winoffset << 0);
}

/**
 * @brief MINEVTIME register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00            MINEVTIME   0x0
 * </pre>
 */
#define EM_BLE_MINEVTIME_ADDR   (0x204F002E + EM_BLE_CS_OFFSET)
#define EM_BLE_MINEVTIME_INDEX  0x00000017
#define EM_BLE_MINEVTIME_RESET  0x00000000

__INLINE uint16_t em_ble_minevtime_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_MINEVTIME_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_minevtime_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_MINEVTIME_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_MINEVTIME_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_MINEVTIME_LSB    0
#define EM_BLE_MINEVTIME_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_MINEVTIME_RST    0x0

__INLINE uint16_t em_ble_minevtime_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_MINEVTIME_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_minevtime_setf(int elt_idx, uint16_t minevtime)
{
    ASSERT_ERR((((uint16_t)minevtime << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_MINEVTIME_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)minevtime << 0);
}

/**
 * @brief MAXEVTIME register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00            MAXEVTIME   0x0
 * </pre>
 */
#define EM_BLE_MAXEVTIME_ADDR   (0x204F0030 + EM_BLE_CS_OFFSET)
#define EM_BLE_MAXEVTIME_INDEX  0x00000018
#define EM_BLE_MAXEVTIME_RESET  0x00000000

__INLINE uint16_t em_ble_maxevtime_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_MAXEVTIME_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_maxevtime_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_MAXEVTIME_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_MAXEVTIME_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_MAXEVTIME_LSB    0
#define EM_BLE_MAXEVTIME_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_MAXEVTIME_RST    0x0

__INLINE uint16_t em_ble_maxevtime_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_MAXEVTIME_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_maxevtime_setf(int elt_idx, uint16_t maxevtime)
{
    ASSERT_ERR((((uint16_t)maxevtime << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_MAXEVTIME_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)maxevtime << 0);
}

/**
 * @brief CONNINTERVAL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00         CONNINTERVAL   0x0
 * </pre>
 */
#define EM_BLE_CONNINTERVAL_ADDR   (0x204F0032 + EM_BLE_CS_OFFSET)
#define EM_BLE_CONNINTERVAL_INDEX  0x00000019
#define EM_BLE_CONNINTERVAL_RESET  0x00000000

__INLINE uint16_t em_ble_conninterval_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_CONNINTERVAL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_conninterval_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_CONNINTERVAL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_CONNINTERVAL_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_CONNINTERVAL_LSB    0
#define EM_BLE_CONNINTERVAL_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_CONNINTERVAL_RST    0x0

__INLINE uint16_t em_ble_conninterval_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_CONNINTERVAL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_conninterval_setf(int elt_idx, uint16_t conninterval)
{
    ASSERT_ERR((((uint16_t)conninterval << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_CONNINTERVAL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)conninterval << 0);
}

/**
 * @brief CHMAP0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00             LLCHMAP0   0xFFFF
 * </pre>
 */
#define EM_BLE_CHMAP0_ADDR   (0x204F0032 + EM_BLE_CS_OFFSET)
#define EM_BLE_CHMAP0_INDEX  0x00000019
#define EM_BLE_CHMAP0_RESET  0x0000FFFF

__INLINE uint16_t em_ble_chmap0_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_CHMAP0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_chmap0_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_CHMAP0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_LLCHMAP0_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_LLCHMAP0_LSB    0
#define EM_BLE_LLCHMAP0_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_LLCHMAP0_RST    0xFFFF

__INLINE uint16_t em_ble_chmap0_llchmap0_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_CHMAP0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_chmap0_llchmap0_setf(int elt_idx, uint16_t llchmap0)
{
    ASSERT_ERR((((uint16_t)llchmap0 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_CHMAP0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)llchmap0 << 0);
}

/**
 * @brief CHMAP1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00             LLCHMAP1   0xFFFF
 * </pre>
 */
#define EM_BLE_CHMAP1_ADDR   (0x204F0034 + EM_BLE_CS_OFFSET)
#define EM_BLE_CHMAP1_INDEX  0x0000001A
#define EM_BLE_CHMAP1_RESET  0x0000FFFF

__INLINE uint16_t em_ble_chmap1_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_CHMAP1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_chmap1_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_CHMAP1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_LLCHMAP1_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_LLCHMAP1_LSB    0
#define EM_BLE_LLCHMAP1_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_LLCHMAP1_RST    0xFFFF

__INLINE uint16_t em_ble_chmap1_llchmap1_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_CHMAP1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_chmap1_llchmap1_setf(int elt_idx, uint16_t llchmap1)
{
    ASSERT_ERR((((uint16_t)llchmap1 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_CHMAP1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)llchmap1 << 0);
}

/**
 * @brief CHMAP2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:10               CH_AUX   0x0
 *  07:05             ADVCHMAP   0x7
 *  04:00             LLCHMAP2   0x1F
 * </pre>
 */
#define EM_BLE_CHMAP2_ADDR   (0x204F0036 + EM_BLE_CS_OFFSET)
#define EM_BLE_CHMAP2_INDEX  0x0000001B
#define EM_BLE_CHMAP2_RESET  0x000000FF

__INLINE uint16_t em_ble_chmap2_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_CHMAP2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_chmap2_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_CHMAP2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_CH_AUX_MASK     ((uint16_t)0x0000FC00)
#define EM_BLE_CH_AUX_LSB      10
#define EM_BLE_CH_AUX_WIDTH    ((uint16_t)0x00000006)
#define EM_BLE_ADVCHMAP_MASK   ((uint16_t)0x000000E0)
#define EM_BLE_ADVCHMAP_LSB    5
#define EM_BLE_ADVCHMAP_WIDTH  ((uint16_t)0x00000003)
#define EM_BLE_LLCHMAP2_MASK   ((uint16_t)0x0000001F)
#define EM_BLE_LLCHMAP2_LSB    0
#define EM_BLE_LLCHMAP2_WIDTH  ((uint16_t)0x00000005)

#define EM_BLE_CH_AUX_RST      0x0
#define EM_BLE_ADVCHMAP_RST    0x7
#define EM_BLE_LLCHMAP2_RST    0x1F

__INLINE void em_ble_chmap2_pack(int elt_idx, uint8_t chaux, uint8_t advchmap, uint8_t llchmap2)
{
    ASSERT_ERR((((uint16_t)chaux << 10) & ~((uint16_t)0x0000FC00)) == 0);
    ASSERT_ERR((((uint16_t)advchmap << 5) & ~((uint16_t)0x000000E0)) == 0);
    ASSERT_ERR((((uint16_t)llchmap2 << 0) & ~((uint16_t)0x0000001F)) == 0);
    EM_BLE_WR(EM_BLE_CHMAP2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, ((uint16_t)chaux << 10) | ((uint16_t)advchmap << 5) | ((uint16_t)llchmap2 << 0));
}

__INLINE void em_ble_chmap2_unpack(int elt_idx, uint8_t *chaux, uint8_t *advchmap, uint8_t *llchmap2)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_CHMAP2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);

    *chaux = (localVal & ((uint16_t)0x0000FC00)) >> 10;
    *advchmap = (localVal & ((uint16_t)0x000000E0)) >> 5;
    *llchmap2 = (localVal & ((uint16_t)0x0000001F)) >> 0;
}

__INLINE uint8_t em_ble_chmap2_ch_aux_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_CHMAP2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000FC00)) >> 10);
}

__INLINE void em_ble_chmap2_ch_aux_setf(int elt_idx, uint8_t chaux)
{
    ASSERT_ERR((((uint16_t)chaux << 10) & ~((uint16_t)0x0000FC00)) == 0);
    EM_BLE_WR(EM_BLE_CHMAP2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_CHMAP2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x0000FC00)) | ((uint16_t)chaux << 10));
}

__INLINE uint8_t em_ble_chmap2_advchmap_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_CHMAP2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x000000E0)) >> 5);
}

__INLINE void em_ble_chmap2_advchmap_setf(int elt_idx, uint8_t advchmap)
{
    ASSERT_ERR((((uint16_t)advchmap << 5) & ~((uint16_t)0x000000E0)) == 0);
    EM_BLE_WR(EM_BLE_CHMAP2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_CHMAP2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x000000E0)) | ((uint16_t)advchmap << 5));
}

__INLINE uint8_t em_ble_chmap2_llchmap2_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_CHMAP2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000001F)) >> 0);
}

__INLINE void em_ble_chmap2_llchmap2_setf(int elt_idx, uint8_t llchmap2)
{
    ASSERT_ERR((((uint16_t)llchmap2 << 0) & ~((uint16_t)0x0000001F)) == 0);
    EM_BLE_WR(EM_BLE_CHMAP2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_CHMAP2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x0000001F)) | ((uint16_t)llchmap2 << 0));
}

/**
 * @brief RXMAXAUXCHAIN register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:11          MAXRXCHDESC   0x0
 *  10:00          MAXRXCHBYTE   0x0
 * </pre>
 */
#define EM_BLE_RXMAXAUXCHAIN_ADDR   (0x204F0038 + EM_BLE_CS_OFFSET)
#define EM_BLE_RXMAXAUXCHAIN_INDEX  0x0000001C
#define EM_BLE_RXMAXAUXCHAIN_RESET  0x00000000

__INLINE uint16_t em_ble_rxmaxauxchain_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXMAXAUXCHAIN_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_rxmaxauxchain_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXMAXAUXCHAIN_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_MAXRXCHDESC_MASK   ((uint16_t)0x0000F800)
#define EM_BLE_MAXRXCHDESC_LSB    11
#define EM_BLE_MAXRXCHDESC_WIDTH  ((uint16_t)0x00000005)
#define EM_BLE_MAXRXCHBYTE_MASK   ((uint16_t)0x000007FF)
#define EM_BLE_MAXRXCHBYTE_LSB    0
#define EM_BLE_MAXRXCHBYTE_WIDTH  ((uint16_t)0x0000000B)

#define EM_BLE_MAXRXCHDESC_RST    0x0
#define EM_BLE_MAXRXCHBYTE_RST    0x0

__INLINE void em_ble_rxmaxauxchain_pack(int elt_idx, uint8_t maxrxchdesc, uint16_t maxrxchbyte)
{
    ASSERT_ERR((((uint16_t)maxrxchdesc << 11) & ~((uint16_t)0x0000F800)) == 0);
    ASSERT_ERR((((uint16_t)maxrxchbyte << 0) & ~((uint16_t)0x000007FF)) == 0);
    EM_BLE_WR(EM_BLE_RXMAXAUXCHAIN_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, ((uint16_t)maxrxchdesc << 11) | ((uint16_t)maxrxchbyte << 0));
}

__INLINE void em_ble_rxmaxauxchain_unpack(int elt_idx, uint8_t *maxrxchdesc, uint16_t *maxrxchbyte)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXMAXAUXCHAIN_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);

    *maxrxchdesc = (localVal & ((uint16_t)0x0000F800)) >> 11;
    *maxrxchbyte = (localVal & ((uint16_t)0x000007FF)) >> 0;
}

__INLINE uint8_t em_ble_rxmaxauxchain_maxrxchdesc_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXMAXAUXCHAIN_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000F800)) >> 11);
}

__INLINE void em_ble_rxmaxauxchain_maxrxchdesc_setf(int elt_idx, uint8_t maxrxchdesc)
{
    ASSERT_ERR((((uint16_t)maxrxchdesc << 11) & ~((uint16_t)0x0000F800)) == 0);
    EM_BLE_WR(EM_BLE_RXMAXAUXCHAIN_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_RXMAXAUXCHAIN_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x0000F800)) | ((uint16_t)maxrxchdesc << 11));
}

__INLINE uint16_t em_ble_rxmaxauxchain_maxrxchbyte_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXMAXAUXCHAIN_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x000007FF)) >> 0);
}

__INLINE void em_ble_rxmaxauxchain_maxrxchbyte_setf(int elt_idx, uint16_t maxrxchbyte)
{
    ASSERT_ERR((((uint16_t)maxrxchbyte << 0) & ~((uint16_t)0x000007FF)) == 0);
    EM_BLE_WR(EM_BLE_RXMAXAUXCHAIN_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_RXMAXAUXCHAIN_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x000007FF)) | ((uint16_t)maxrxchbyte << 0));
}

/**
 * @brief RXMAXBUF register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:08          ISORXMAXBUF   0x0
 *  07:00          ACLRXMAXBUF   0x0
 * </pre>
 */
#define EM_BLE_RXMAXBUF_ADDR   (0x204F0038 + EM_BLE_CS_OFFSET)
#define EM_BLE_RXMAXBUF_INDEX  0x0000001C
#define EM_BLE_RXMAXBUF_RESET  0x00000000

__INLINE uint16_t em_ble_rxmaxbuf_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXMAXBUF_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_rxmaxbuf_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXMAXBUF_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_ISORXMAXBUF_MASK   ((uint16_t)0x0000FF00)
#define EM_BLE_ISORXMAXBUF_LSB    8
#define EM_BLE_ISORXMAXBUF_WIDTH  ((uint16_t)0x00000008)
#define EM_BLE_ACLRXMAXBUF_MASK   ((uint16_t)0x000000FF)
#define EM_BLE_ACLRXMAXBUF_LSB    0
#define EM_BLE_ACLRXMAXBUF_WIDTH  ((uint16_t)0x00000008)

#define EM_BLE_ISORXMAXBUF_RST    0x0
#define EM_BLE_ACLRXMAXBUF_RST    0x0

__INLINE void em_ble_rxmaxbuf_pack(int elt_idx, uint8_t isorxmaxbuf, uint8_t aclrxmaxbuf)
{
    ASSERT_ERR((((uint16_t)isorxmaxbuf << 8) & ~((uint16_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint16_t)aclrxmaxbuf << 0) & ~((uint16_t)0x000000FF)) == 0);
    EM_BLE_WR(EM_BLE_RXMAXBUF_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, ((uint16_t)isorxmaxbuf << 8) | ((uint16_t)aclrxmaxbuf << 0));
}

__INLINE void em_ble_rxmaxbuf_unpack(int elt_idx, uint8_t *isorxmaxbuf, uint8_t *aclrxmaxbuf)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXMAXBUF_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);

    *isorxmaxbuf = (localVal & ((uint16_t)0x0000FF00)) >> 8;
    *aclrxmaxbuf = (localVal & ((uint16_t)0x000000FF)) >> 0;
}

__INLINE uint8_t em_ble_rxmaxbuf_isorxmaxbuf_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXMAXBUF_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000FF00)) >> 8);
}

__INLINE void em_ble_rxmaxbuf_isorxmaxbuf_setf(int elt_idx, uint8_t isorxmaxbuf)
{
    ASSERT_ERR((((uint16_t)isorxmaxbuf << 8) & ~((uint16_t)0x0000FF00)) == 0);
    EM_BLE_WR(EM_BLE_RXMAXBUF_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_RXMAXBUF_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x0000FF00)) | ((uint16_t)isorxmaxbuf << 8));
}

__INLINE uint8_t em_ble_rxmaxbuf_aclrxmaxbuf_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXMAXBUF_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x000000FF)) >> 0);
}

__INLINE void em_ble_rxmaxbuf_aclrxmaxbuf_setf(int elt_idx, uint8_t aclrxmaxbuf)
{
    ASSERT_ERR((((uint16_t)aclrxmaxbuf << 0) & ~((uint16_t)0x000000FF)) == 0);
    EM_BLE_WR(EM_BLE_RXMAXBUF_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_RXMAXBUF_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x000000FF)) | ((uint16_t)aclrxmaxbuf << 0));
}

/**
 * @brief RXMAXTIME register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  12:00            RXMAXTIME   0x0
 * </pre>
 */
#define EM_BLE_RXMAXTIME_ADDR   (0x204F003A + EM_BLE_CS_OFFSET)
#define EM_BLE_RXMAXTIME_INDEX  0x0000001D
#define EM_BLE_RXMAXTIME_RESET  0x00000000

__INLINE uint16_t em_ble_rxmaxtime_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXMAXTIME_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_rxmaxtime_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXMAXTIME_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_RXMAXTIME_MASK   ((uint16_t)0x00001FFF)
#define EM_BLE_RXMAXTIME_LSB    0
#define EM_BLE_RXMAXTIME_WIDTH  ((uint16_t)0x0000000D)

#define EM_BLE_RXMAXTIME_RST    0x0

__INLINE uint16_t em_ble_rxmaxtime_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXMAXTIME_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x00001FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_rxmaxtime_setf(int elt_idx, uint16_t rxmaxtime)
{
    ASSERT_ERR((((uint16_t)rxmaxtime << 0) & ~((uint16_t)0x00001FFF)) == 0);
    EM_BLE_WR(EM_BLE_RXMAXTIME_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)rxmaxtime << 0);
}

/**
 * @brief SK register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00                   SK   0x0
 * </pre>
 */
#define EM_BLE_SK_ADDR   (0x204F003C + EM_BLE_CS_OFFSET)
#define EM_BLE_SK_INDEX  0x0000001E
#define EM_BLE_SK_RESET  0x00000000
#define EM_BLE_SK_COUNT  8

__INLINE uint16_t em_ble_sk_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 7);
    return EM_BLE_RD(EM_BLE_SK_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
}

__INLINE void em_ble_sk_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 7);
    EM_BLE_WR(EM_BLE_SK_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BLE_SK_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_SK_LSB    0
#define EM_BLE_SK_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_SK_RST    0x0

__INLINE uint16_t em_ble_sk_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 7);
    uint16_t localVal = EM_BLE_RD(EM_BLE_SK_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_sk_setf(int elt_idx, int reg_idx, uint16_t sk)
{
    ASSERT_ERR(reg_idx <= 7);
    ASSERT_ERR((((uint16_t)sk << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_SK_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, (uint16_t)sk << 0);
}

/**
 * @brief ADV_BD_ADDR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00          ADV_BD_ADDR   0x0
 * </pre>
 */
#define EM_BLE_ADV_BD_ADDR_ADDR   (0x204F003C + EM_BLE_CS_OFFSET)
#define EM_BLE_ADV_BD_ADDR_INDEX  0x0000001E
#define EM_BLE_ADV_BD_ADDR_RESET  0x00000000
#define EM_BLE_ADV_BD_ADDR_COUNT  3

__INLINE uint16_t em_ble_adv_bd_addr_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 2);
    return EM_BLE_RD(EM_BLE_ADV_BD_ADDR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
}

__INLINE void em_ble_adv_bd_addr_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 2);
    EM_BLE_WR(EM_BLE_ADV_BD_ADDR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BLE_ADV_BD_ADDR_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_ADV_BD_ADDR_LSB    0
#define EM_BLE_ADV_BD_ADDR_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_ADV_BD_ADDR_RST    0x0

__INLINE uint16_t em_ble_adv_bd_addr_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 2);
    uint16_t localVal = EM_BLE_RD(EM_BLE_ADV_BD_ADDR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_adv_bd_addr_setf(int elt_idx, int reg_idx, uint16_t advbdaddr)
{
    ASSERT_ERR(reg_idx <= 2);
    ASSERT_ERR((((uint16_t)advbdaddr << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_ADV_BD_ADDR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, (uint16_t)advbdaddr << 0);
}

/**
 * @brief PEER_RALPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  13:00          PEER_RALPTR   0x0
 * </pre>
 */
#define EM_BLE_PEER_RALPTR_ADDR   (0x204F003C + EM_BLE_CS_OFFSET)
#define EM_BLE_PEER_RALPTR_INDEX  0x0000001E
#define EM_BLE_PEER_RALPTR_RESET  0x00000000

__INLINE uint16_t em_ble_peer_ralptr_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_PEER_RALPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_peer_ralptr_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_PEER_RALPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_PEER_RALPTR_MASK   ((uint16_t)0x00003FFF)
#define EM_BLE_PEER_RALPTR_LSB    0
#define EM_BLE_PEER_RALPTR_WIDTH  ((uint16_t)0x0000000E)

#define EM_BLE_PEER_RALPTR_RST    0x0

__INLINE uint16_t em_ble_peer_ralptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_PEER_RALPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x00003FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_peer_ralptr_setf(int elt_idx, uint16_t peerralptr)
{
    ASSERT_ERR((((uint16_t)peerralptr << 0) & ~((uint16_t)0x00003FFF)) == 0);
    EM_BLE_WR(EM_BLE_PEER_RALPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)peerralptr << 0);
}

/**
 * @brief ADV_BD_ADDR_TYPE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     00     ADV_BD_ADDR_TYPE   0
 * </pre>
 */
#define EM_BLE_ADV_BD_ADDR_TYPE_ADDR   (0x204F0042 + EM_BLE_CS_OFFSET)
#define EM_BLE_ADV_BD_ADDR_TYPE_INDEX  0x00000021
#define EM_BLE_ADV_BD_ADDR_TYPE_RESET  0x00000000

__INLINE uint16_t em_ble_adv_bd_addr_type_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_ADV_BD_ADDR_TYPE_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_adv_bd_addr_type_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_ADV_BD_ADDR_TYPE_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_ADV_BD_ADDR_TYPE_BIT    ((uint16_t)0x00000001)
#define EM_BLE_ADV_BD_ADDR_TYPE_POS    0

#define EM_BLE_ADV_BD_ADDR_TYPE_RST    0x0

__INLINE uint8_t em_ble_adv_bd_addr_type_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ADV_BD_ADDR_TYPE_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x00000001)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_adv_bd_addr_type_setf(int elt_idx, uint8_t advbdaddrtype)
{
    ASSERT_ERR((((uint16_t)advbdaddrtype << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BLE_WR(EM_BLE_ADV_BD_ADDR_TYPE_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)advbdaddrtype << 0);
}

/**
 * @brief AUXTXDESCPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  13:00         AUXTXDESCPTR   0x0
 * </pre>
 */
#define EM_BLE_AUXTXDESCPTR_ADDR   (0x204F0044 + EM_BLE_CS_OFFSET)
#define EM_BLE_AUXTXDESCPTR_INDEX  0x00000022
#define EM_BLE_AUXTXDESCPTR_RESET  0x00000000

__INLINE uint16_t em_ble_auxtxdescptr_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_AUXTXDESCPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_auxtxdescptr_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_AUXTXDESCPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_AUXTXDESCPTR_MASK   ((uint16_t)0x00003FFF)
#define EM_BLE_AUXTXDESCPTR_LSB    0
#define EM_BLE_AUXTXDESCPTR_WIDTH  ((uint16_t)0x0000000E)

#define EM_BLE_AUXTXDESCPTR_RST    0x0

__INLINE uint16_t em_ble_auxtxdescptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_AUXTXDESCPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x00003FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_auxtxdescptr_setf(int elt_idx, uint16_t auxtxdescptr)
{
    ASSERT_ERR((((uint16_t)auxtxdescptr << 0) & ~((uint16_t)0x00003FFF)) == 0);
    EM_BLE_WR(EM_BLE_AUXTXDESCPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)auxtxdescptr << 0);
}

/**
 * @brief IV register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00                   IV   0x0
 * </pre>
 */
#define EM_BLE_IV_ADDR   (0x204F004C + EM_BLE_CS_OFFSET)
#define EM_BLE_IV_INDEX  0x00000026
#define EM_BLE_IV_RESET  0x00000000
#define EM_BLE_IV_COUNT  4

__INLINE uint16_t em_ble_iv_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 3);
    return EM_BLE_RD(EM_BLE_IV_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
}

__INLINE void em_ble_iv_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 3);
    EM_BLE_WR(EM_BLE_IV_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BLE_IV_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_IV_LSB    0
#define EM_BLE_IV_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_IV_RST    0x0

__INLINE uint16_t em_ble_iv_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 3);
    uint16_t localVal = EM_BLE_RD(EM_BLE_IV_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_iv_setf(int elt_idx, int reg_idx, uint16_t iv)
{
    ASSERT_ERR(reg_idx <= 3);
    ASSERT_ERR((((uint16_t)iv << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_IV_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, (uint16_t)iv << 0);
}

/**
 * @brief WINOFFSET_2M register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00         WINOFFSET_2M   0x0
 * </pre>
 */
#define EM_BLE_WINOFFSET_2M_ADDR   (0x204F0046 + EM_BLE_CS_OFFSET)
#define EM_BLE_WINOFFSET_2M_INDEX  0x00000023
#define EM_BLE_WINOFFSET_2M_RESET  0x00000000

__INLINE uint16_t em_ble_winoffset_2m_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_WINOFFSET_2M_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_winoffset_2m_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_WINOFFSET_2M_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_WINOFFSET_2M_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_WINOFFSET_2M_LSB    0
#define EM_BLE_WINOFFSET_2M_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_WINOFFSET_2M_RST    0x0

__INLINE uint16_t em_ble_winoffset_2m_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_WINOFFSET_2M_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_winoffset_2m_setf(int elt_idx, uint16_t winoffset2m)
{
    ASSERT_ERR((((uint16_t)winoffset2m << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_WINOFFSET_2M_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)winoffset2m << 0);
}

/**
 * @brief CONNINTERVAL_2M register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00      CONNINTERVAL_2M   0x0
 * </pre>
 */
#define EM_BLE_CONNINTERVAL_2M_ADDR   (0x204F0048 + EM_BLE_CS_OFFSET)
#define EM_BLE_CONNINTERVAL_2M_INDEX  0x00000024
#define EM_BLE_CONNINTERVAL_2M_RESET  0x00000000

__INLINE uint16_t em_ble_conninterval_2m_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_CONNINTERVAL_2M_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_conninterval_2m_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_CONNINTERVAL_2M_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_CONNINTERVAL_2M_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_CONNINTERVAL_2M_LSB    0
#define EM_BLE_CONNINTERVAL_2M_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_CONNINTERVAL_2M_RST    0x0

__INLINE uint16_t em_ble_conninterval_2m_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_CONNINTERVAL_2M_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_conninterval_2m_setf(int elt_idx, uint16_t conninterval2m)
{
    ASSERT_ERR((((uint16_t)conninterval2m << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_CONNINTERVAL_2M_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)conninterval2m << 0);
}

/**
 * @brief WINOFFSET_LR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00         WINOFFSET_LR   0x0
 * </pre>
 */
#define EM_BLE_WINOFFSET_LR_ADDR   (0x204F004A + EM_BLE_CS_OFFSET)
#define EM_BLE_WINOFFSET_LR_INDEX  0x00000025
#define EM_BLE_WINOFFSET_LR_RESET  0x00000000

__INLINE uint16_t em_ble_winoffset_lr_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_WINOFFSET_LR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_winoffset_lr_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_WINOFFSET_LR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_WINOFFSET_LR_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_WINOFFSET_LR_LSB    0
#define EM_BLE_WINOFFSET_LR_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_WINOFFSET_LR_RST    0x0

__INLINE uint16_t em_ble_winoffset_lr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_WINOFFSET_LR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_winoffset_lr_setf(int elt_idx, uint16_t winoffsetlr)
{
    ASSERT_ERR((((uint16_t)winoffsetlr << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_WINOFFSET_LR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)winoffsetlr << 0);
}

/**
 * @brief CONNINTERVAL_LR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00      CONNINTERVAL_LR   0x0
 * </pre>
 */
#define EM_BLE_CONNINTERVAL_LR_ADDR   (0x204F004C + EM_BLE_CS_OFFSET)
#define EM_BLE_CONNINTERVAL_LR_INDEX  0x00000026
#define EM_BLE_CONNINTERVAL_LR_RESET  0x00000000

__INLINE uint16_t em_ble_conninterval_lr_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_CONNINTERVAL_LR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_conninterval_lr_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_CONNINTERVAL_LR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_CONNINTERVAL_LR_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_CONNINTERVAL_LR_LSB    0
#define EM_BLE_CONNINTERVAL_LR_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_CONNINTERVAL_LR_RST    0x0

__INLINE uint16_t em_ble_conninterval_lr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_CONNINTERVAL_LR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_conninterval_lr_setf(int elt_idx, uint16_t connintervallr)
{
    ASSERT_ERR((((uint16_t)connintervallr << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_CONNINTERVAL_LR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)connintervallr << 0);
}

/**
 * @brief TXWINOFFSET register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00          TXWINOFFSET   0x0
 * </pre>
 */
#define EM_BLE_TXWINOFFSET_ADDR   (0x204F0054 + EM_BLE_CS_OFFSET)
#define EM_BLE_TXWINOFFSET_INDEX  0x0000002A
#define EM_BLE_TXWINOFFSET_RESET  0x00000000

__INLINE uint16_t em_ble_txwinoffset_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_TXWINOFFSET_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_txwinoffset_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_TXWINOFFSET_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_TXWINOFFSET_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_TXWINOFFSET_LSB    0
#define EM_BLE_TXWINOFFSET_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_TXWINOFFSET_RST    0x0

__INLINE uint16_t em_ble_txwinoffset_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXWINOFFSET_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_txwinoffset_setf(int elt_idx, uint16_t txwinoffset)
{
    ASSERT_ERR((((uint16_t)txwinoffset << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_TXWINOFFSET_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)txwinoffset << 0);
}

/**
 * @brief TXCCMPKTCNT0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00         TXCCMPKTCNT0   0x0
 * </pre>
 */
#define EM_BLE_TXCCMPKTCNT0_ADDR   (0x204F0054 + EM_BLE_CS_OFFSET)
#define EM_BLE_TXCCMPKTCNT0_INDEX  0x0000002A
#define EM_BLE_TXCCMPKTCNT0_RESET  0x00000000

__INLINE uint16_t em_ble_txccmpktcnt0_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_TXCCMPKTCNT0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_txccmpktcnt0_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_TXCCMPKTCNT0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_TXCCMPKTCNT0_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_TXCCMPKTCNT0_LSB    0
#define EM_BLE_TXCCMPKTCNT0_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_TXCCMPKTCNT0_RST    0x0

__INLINE uint16_t em_ble_txccmpktcnt0_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXCCMPKTCNT0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_txccmpktcnt0_setf(int elt_idx, uint16_t txccmpktcnt0)
{
    ASSERT_ERR((((uint16_t)txccmpktcnt0 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_TXCCMPKTCNT0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)txccmpktcnt0 << 0);
}

/**
 * @brief EXTADVSTAT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     07             PREV_CTE   0
 *     06             PREV_PAM   0
 *     05             PREV_LAM   0
 *  04:03        PREV_ADV_MODE   0x0
 *  02:00    PREV_ADV_PKT_TYPE   0x0
 * </pre>
 */
#define EM_BLE_EXTADVSTAT_ADDR   (0x204F0056 + EM_BLE_CS_OFFSET)
#define EM_BLE_EXTADVSTAT_INDEX  0x0000002B
#define EM_BLE_EXTADVSTAT_RESET  0x00000000

__INLINE uint16_t em_ble_extadvstat_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_EXTADVSTAT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_extadvstat_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_EXTADVSTAT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_PREV_CTE_BIT             ((uint16_t)0x00000080)
#define EM_BLE_PREV_CTE_POS             7
#define EM_BLE_PREV_PAM_BIT             ((uint16_t)0x00000040)
#define EM_BLE_PREV_PAM_POS             6
#define EM_BLE_PREV_LAM_BIT             ((uint16_t)0x00000020)
#define EM_BLE_PREV_LAM_POS             5
#define EM_BLE_PREV_ADV_MODE_MASK       ((uint16_t)0x00000018)
#define EM_BLE_PREV_ADV_MODE_LSB        3
#define EM_BLE_PREV_ADV_MODE_WIDTH      ((uint16_t)0x00000002)
#define EM_BLE_PREV_ADV_PKT_TYPE_MASK   ((uint16_t)0x00000007)
#define EM_BLE_PREV_ADV_PKT_TYPE_LSB    0
#define EM_BLE_PREV_ADV_PKT_TYPE_WIDTH  ((uint16_t)0x00000003)

#define EM_BLE_PREV_CTE_RST             0x0
#define EM_BLE_PREV_PAM_RST             0x0
#define EM_BLE_PREV_LAM_RST             0x0
#define EM_BLE_PREV_ADV_MODE_RST        0x0
#define EM_BLE_PREV_ADV_PKT_TYPE_RST    0x0

__INLINE void em_ble_extadvstat_pack(int elt_idx, uint8_t prevcte, uint8_t prevpam, uint8_t prevlam, uint8_t prevadvmode, uint8_t prevadvpkttype)
{
    ASSERT_ERR((((uint16_t)prevcte << 7) & ~((uint16_t)0x00000080)) == 0);
    ASSERT_ERR((((uint16_t)prevpam << 6) & ~((uint16_t)0x00000040)) == 0);
    ASSERT_ERR((((uint16_t)prevlam << 5) & ~((uint16_t)0x00000020)) == 0);
    ASSERT_ERR((((uint16_t)prevadvmode << 3) & ~((uint16_t)0x00000018)) == 0);
    ASSERT_ERR((((uint16_t)prevadvpkttype << 0) & ~((uint16_t)0x00000007)) == 0);
    EM_BLE_WR(EM_BLE_EXTADVSTAT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, ((uint16_t)prevcte << 7) | ((uint16_t)prevpam << 6) | ((uint16_t)prevlam << 5) | ((uint16_t)prevadvmode << 3) | ((uint16_t)prevadvpkttype << 0));
}

__INLINE void em_ble_extadvstat_unpack(int elt_idx, uint8_t *prevcte, uint8_t *prevpam, uint8_t *prevlam, uint8_t *prevadvmode, uint8_t *prevadvpkttype)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_EXTADVSTAT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);

    *prevcte = (localVal & ((uint16_t)0x00000080)) >> 7;
    *prevpam = (localVal & ((uint16_t)0x00000040)) >> 6;
    *prevlam = (localVal & ((uint16_t)0x00000020)) >> 5;
    *prevadvmode = (localVal & ((uint16_t)0x00000018)) >> 3;
    *prevadvpkttype = (localVal & ((uint16_t)0x00000007)) >> 0;
}

__INLINE uint8_t em_ble_extadvstat_prev_cte_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_EXTADVSTAT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000080)) >> 7);
}

__INLINE void em_ble_extadvstat_prev_cte_setf(int elt_idx, uint8_t prevcte)
{
    ASSERT_ERR((((uint16_t)prevcte << 7) & ~((uint16_t)0x00000080)) == 0);
    EM_BLE_WR(EM_BLE_EXTADVSTAT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_EXTADVSTAT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000080)) | ((uint16_t)prevcte << 7));
}

__INLINE uint8_t em_ble_extadvstat_prev_pam_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_EXTADVSTAT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000040)) >> 6);
}

__INLINE void em_ble_extadvstat_prev_pam_setf(int elt_idx, uint8_t prevpam)
{
    ASSERT_ERR((((uint16_t)prevpam << 6) & ~((uint16_t)0x00000040)) == 0);
    EM_BLE_WR(EM_BLE_EXTADVSTAT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_EXTADVSTAT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000040)) | ((uint16_t)prevpam << 6));
}

__INLINE uint8_t em_ble_extadvstat_prev_lam_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_EXTADVSTAT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000020)) >> 5);
}

__INLINE void em_ble_extadvstat_prev_lam_setf(int elt_idx, uint8_t prevlam)
{
    ASSERT_ERR((((uint16_t)prevlam << 5) & ~((uint16_t)0x00000020)) == 0);
    EM_BLE_WR(EM_BLE_EXTADVSTAT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_EXTADVSTAT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000020)) | ((uint16_t)prevlam << 5));
}

__INLINE uint8_t em_ble_extadvstat_prev_adv_mode_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_EXTADVSTAT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000018)) >> 3);
}

__INLINE void em_ble_extadvstat_prev_adv_mode_setf(int elt_idx, uint8_t prevadvmode)
{
    ASSERT_ERR((((uint16_t)prevadvmode << 3) & ~((uint16_t)0x00000018)) == 0);
    EM_BLE_WR(EM_BLE_EXTADVSTAT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_EXTADVSTAT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000018)) | ((uint16_t)prevadvmode << 3));
}

__INLINE uint8_t em_ble_extadvstat_prev_adv_pkt_type_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_EXTADVSTAT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000007)) >> 0);
}

__INLINE void em_ble_extadvstat_prev_adv_pkt_type_setf(int elt_idx, uint8_t prevadvpkttype)
{
    ASSERT_ERR((((uint16_t)prevadvpkttype << 0) & ~((uint16_t)0x00000007)) == 0);
    EM_BLE_WR(EM_BLE_EXTADVSTAT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_EXTADVSTAT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000007)) | ((uint16_t)prevadvpkttype << 0));
}

/**
 * @brief TXCCMPKTCNT1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00         TXCCMPKTCNT1   0x0
 * </pre>
 */
#define EM_BLE_TXCCMPKTCNT1_ADDR   (0x204F0056 + EM_BLE_CS_OFFSET)
#define EM_BLE_TXCCMPKTCNT1_INDEX  0x0000002B
#define EM_BLE_TXCCMPKTCNT1_RESET  0x00000000

__INLINE uint16_t em_ble_txccmpktcnt1_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_TXCCMPKTCNT1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_txccmpktcnt1_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_TXCCMPKTCNT1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_TXCCMPKTCNT1_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_TXCCMPKTCNT1_LSB    0
#define EM_BLE_TXCCMPKTCNT1_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_TXCCMPKTCNT1_RST    0x0

__INLINE uint16_t em_ble_txccmpktcnt1_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXCCMPKTCNT1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_txccmpktcnt1_setf(int elt_idx, uint16_t txccmpktcnt1)
{
    ASSERT_ERR((((uint16_t)txccmpktcnt1 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_TXCCMPKTCNT1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)txccmpktcnt1 << 0);
}

/**
 * @brief TXCCMPKTCNT2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  06:00         TXCCMPKTCNT2   0x0
 * </pre>
 */
#define EM_BLE_TXCCMPKTCNT2_ADDR   (0x204F0058 + EM_BLE_CS_OFFSET)
#define EM_BLE_TXCCMPKTCNT2_INDEX  0x0000002C
#define EM_BLE_TXCCMPKTCNT2_RESET  0x00000000

__INLINE uint16_t em_ble_txccmpktcnt2_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_TXCCMPKTCNT2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_txccmpktcnt2_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_TXCCMPKTCNT2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_TXCCMPKTCNT2_MASK   ((uint16_t)0x0000007F)
#define EM_BLE_TXCCMPKTCNT2_LSB    0
#define EM_BLE_TXCCMPKTCNT2_WIDTH  ((uint16_t)0x00000007)

#define EM_BLE_TXCCMPKTCNT2_RST    0x0

__INLINE uint8_t em_ble_txccmpktcnt2_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXCCMPKTCNT2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000007F)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_txccmpktcnt2_setf(int elt_idx, uint8_t txccmpktcnt2)
{
    ASSERT_ERR((((uint16_t)txccmpktcnt2 << 0) & ~((uint16_t)0x0000007F)) == 0);
    EM_BLE_WR(EM_BLE_TXCCMPKTCNT2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)txccmpktcnt2 << 0);
}

/**
 * @brief RXCCMPKTCNT0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00         RXCCMPKTCNT0   0x0
 * </pre>
 */
#define EM_BLE_RXCCMPKTCNT0_ADDR   (0x204F005A + EM_BLE_CS_OFFSET)
#define EM_BLE_RXCCMPKTCNT0_INDEX  0x0000002D
#define EM_BLE_RXCCMPKTCNT0_RESET  0x00000000

__INLINE uint16_t em_ble_rxccmpktcnt0_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXCCMPKTCNT0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_rxccmpktcnt0_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXCCMPKTCNT0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_RXCCMPKTCNT0_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_RXCCMPKTCNT0_LSB    0
#define EM_BLE_RXCCMPKTCNT0_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_RXCCMPKTCNT0_RST    0x0

__INLINE uint16_t em_ble_rxccmpktcnt0_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXCCMPKTCNT0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_rxccmpktcnt0_setf(int elt_idx, uint16_t rxccmpktcnt0)
{
    ASSERT_ERR((((uint16_t)rxccmpktcnt0 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_RXCCMPKTCNT0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)rxccmpktcnt0 << 0);
}

/**
 * @brief RXCCMPKTCNT1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00         RXCCMPKTCNT1   0x0
 * </pre>
 */
#define EM_BLE_RXCCMPKTCNT1_ADDR   (0x204F005C + EM_BLE_CS_OFFSET)
#define EM_BLE_RXCCMPKTCNT1_INDEX  0x0000002E
#define EM_BLE_RXCCMPKTCNT1_RESET  0x00000000

__INLINE uint16_t em_ble_rxccmpktcnt1_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXCCMPKTCNT1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_rxccmpktcnt1_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXCCMPKTCNT1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_RXCCMPKTCNT1_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_RXCCMPKTCNT1_LSB    0
#define EM_BLE_RXCCMPKTCNT1_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_RXCCMPKTCNT1_RST    0x0

__INLINE uint16_t em_ble_rxccmpktcnt1_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXCCMPKTCNT1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_rxccmpktcnt1_setf(int elt_idx, uint16_t rxccmpktcnt1)
{
    ASSERT_ERR((((uint16_t)rxccmpktcnt1 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_RXCCMPKTCNT1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)rxccmpktcnt1 << 0);
}

/**
 * @brief RXCCMPKTCNT2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  06:00         RXCCMPKTCNT2   0x0
 * </pre>
 */
#define EM_BLE_RXCCMPKTCNT2_ADDR   (0x204F005E + EM_BLE_CS_OFFSET)
#define EM_BLE_RXCCMPKTCNT2_INDEX  0x0000002F
#define EM_BLE_RXCCMPKTCNT2_RESET  0x00000000

__INLINE uint16_t em_ble_rxccmpktcnt2_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXCCMPKTCNT2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_rxccmpktcnt2_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXCCMPKTCNT2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_RXCCMPKTCNT2_MASK   ((uint16_t)0x0000007F)
#define EM_BLE_RXCCMPKTCNT2_LSB    0
#define EM_BLE_RXCCMPKTCNT2_WIDTH  ((uint16_t)0x00000007)

#define EM_BLE_RXCCMPKTCNT2_RST    0x0

__INLINE uint8_t em_ble_rxccmpktcnt2_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXCCMPKTCNT2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000007F)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_rxccmpktcnt2_setf(int elt_idx, uint8_t rxccmpktcnt2)
{
    ASSERT_ERR((((uint16_t)rxccmpktcnt2 << 0) & ~((uint16_t)0x0000007F)) == 0);
    EM_BLE_WR(EM_BLE_RXCCMPKTCNT2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)rxccmpktcnt2 << 0);
}

/**
 * @brief EVTCNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00               EVTCNT   0x0
 * </pre>
 */
#define EM_BLE_EVTCNT_ADDR   (0x204F0060 + EM_BLE_CS_OFFSET)
#define EM_BLE_EVTCNT_INDEX  0x00000030
#define EM_BLE_EVTCNT_RESET  0x00000000

__INLINE uint16_t em_ble_evtcnt_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_EVTCNT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_evtcnt_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_EVTCNT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_EVTCNT_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_EVTCNT_LSB    0
#define EM_BLE_EVTCNT_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_EVTCNT_RST    0x0

__INLINE uint16_t em_ble_evtcnt_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_EVTCNT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_evtcnt_setf(int elt_idx, uint16_t evtcnt)
{
    ASSERT_ERR((((uint16_t)evtcnt << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_EVTCNT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)evtcnt << 0);
}

/**
 * @brief EVTCNT_OFFSET0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00       EVTCNT_OFFSET0   0x0
 * </pre>
 */
#define EM_BLE_EVTCNT_OFFSET0_ADDR   (0x204F0062 + EM_BLE_CS_OFFSET)
#define EM_BLE_EVTCNT_OFFSET0_INDEX  0x00000031
#define EM_BLE_EVTCNT_OFFSET0_RESET  0x00000000

__INLINE uint16_t em_ble_evtcnt_offset0_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_EVTCNT_OFFSET0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_evtcnt_offset0_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_EVTCNT_OFFSET0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_EVTCNT_OFFSET0_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_EVTCNT_OFFSET0_LSB    0
#define EM_BLE_EVTCNT_OFFSET0_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_EVTCNT_OFFSET0_RST    0x0

__INLINE uint16_t em_ble_evtcnt_offset0_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_EVTCNT_OFFSET0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_evtcnt_offset0_setf(int elt_idx, uint16_t evtcntoffset0)
{
    ASSERT_ERR((((uint16_t)evtcntoffset0 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_EVTCNT_OFFSET0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)evtcntoffset0 << 0);
}

/**
 * @brief NSE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  07:00                  NSE   0x0
 * </pre>
 */
#define EM_BLE_NSE_ADDR   (0x204F0062 + EM_BLE_CS_OFFSET)
#define EM_BLE_NSE_INDEX  0x00000031
#define EM_BLE_NSE_RESET  0x00000000

__INLINE uint16_t em_ble_nse_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_NSE_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_nse_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_NSE_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_NSE_MASK   ((uint16_t)0x000000FF)
#define EM_BLE_NSE_LSB    0
#define EM_BLE_NSE_WIDTH  ((uint16_t)0x00000008)

#define EM_BLE_NSE_RST    0x0

__INLINE uint8_t em_ble_nse_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_NSE_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x000000FF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_nse_setf(int elt_idx, uint8_t nse)
{
    ASSERT_ERR((((uint16_t)nse << 0) & ~((uint16_t)0x000000FF)) == 0);
    EM_BLE_WR(EM_BLE_NSE_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)nse << 0);
}

/**
 * @brief EVTCNT_OFFSET1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00       EVTCNT_OFFSET1   0x0
 * </pre>
 */
#define EM_BLE_EVTCNT_OFFSET1_ADDR   (0x204F0064 + EM_BLE_CS_OFFSET)
#define EM_BLE_EVTCNT_OFFSET1_INDEX  0x00000032
#define EM_BLE_EVTCNT_OFFSET1_RESET  0x00000000

__INLINE uint16_t em_ble_evtcnt_offset1_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_EVTCNT_OFFSET1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_evtcnt_offset1_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_EVTCNT_OFFSET1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_EVTCNT_OFFSET1_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_EVTCNT_OFFSET1_LSB    0
#define EM_BLE_EVTCNT_OFFSET1_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_EVTCNT_OFFSET1_RST    0x0

__INLINE uint16_t em_ble_evtcnt_offset1_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_EVTCNT_OFFSET1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_evtcnt_offset1_setf(int elt_idx, uint16_t evtcntoffset1)
{
    ASSERT_ERR((((uint16_t)evtcntoffset1 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_EVTCNT_OFFSET1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)evtcntoffset1 << 0);
}

/**
 * @brief EVTCNT_OFFSET2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  06:00       EVTCNT_OFFSET2   0x0
 * </pre>
 */
#define EM_BLE_EVTCNT_OFFSET2_ADDR   (0x204F0066 + EM_BLE_CS_OFFSET)
#define EM_BLE_EVTCNT_OFFSET2_INDEX  0x00000033
#define EM_BLE_EVTCNT_OFFSET2_RESET  0x00000000

__INLINE uint16_t em_ble_evtcnt_offset2_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_EVTCNT_OFFSET2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_evtcnt_offset2_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_EVTCNT_OFFSET2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_EVTCNT_OFFSET2_MASK   ((uint16_t)0x0000007F)
#define EM_BLE_EVTCNT_OFFSET2_LSB    0
#define EM_BLE_EVTCNT_OFFSET2_WIDTH  ((uint16_t)0x00000007)

#define EM_BLE_EVTCNT_OFFSET2_RST    0x0

__INLINE uint8_t em_ble_evtcnt_offset2_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_EVTCNT_OFFSET2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000007F)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_evtcnt_offset2_setf(int elt_idx, uint8_t evtcntoffset2)
{
    ASSERT_ERR((((uint16_t)evtcntoffset2 << 0) & ~((uint16_t)0x0000007F)) == 0);
    EM_BLE_WR(EM_BLE_EVTCNT_OFFSET2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)evtcntoffset2 << 0);
}

/**
 * @brief ISOEVTCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:08             FLUSHCNT   0x0
 *  07:00            SUBEVTCNT   0x0
 * </pre>
 */
#define EM_BLE_ISOEVTCNTL_ADDR   (0x204F0068 + EM_BLE_CS_OFFSET)
#define EM_BLE_ISOEVTCNTL_INDEX  0x00000034
#define EM_BLE_ISOEVTCNTL_RESET  0x00000000

__INLINE uint16_t em_ble_isoevtcntl_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_ISOEVTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_isoevtcntl_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_ISOEVTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_FLUSHCNT_MASK    ((uint16_t)0x0000FF00)
#define EM_BLE_FLUSHCNT_LSB     8
#define EM_BLE_FLUSHCNT_WIDTH   ((uint16_t)0x00000008)
#define EM_BLE_SUBEVTCNT_MASK   ((uint16_t)0x000000FF)
#define EM_BLE_SUBEVTCNT_LSB    0
#define EM_BLE_SUBEVTCNT_WIDTH  ((uint16_t)0x00000008)

#define EM_BLE_FLUSHCNT_RST     0x0
#define EM_BLE_SUBEVTCNT_RST    0x0

__INLINE void em_ble_isoevtcntl_pack(int elt_idx, uint8_t flushcnt, uint8_t subevtcnt)
{
    ASSERT_ERR((((uint16_t)flushcnt << 8) & ~((uint16_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint16_t)subevtcnt << 0) & ~((uint16_t)0x000000FF)) == 0);
    EM_BLE_WR(EM_BLE_ISOEVTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, ((uint16_t)flushcnt << 8) | ((uint16_t)subevtcnt << 0));
}

__INLINE void em_ble_isoevtcntl_unpack(int elt_idx, uint8_t *flushcnt, uint8_t *subevtcnt)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOEVTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);

    *flushcnt = (localVal & ((uint16_t)0x0000FF00)) >> 8;
    *subevtcnt = (localVal & ((uint16_t)0x000000FF)) >> 0;
}

__INLINE uint8_t em_ble_isoevtcntl_flushcnt_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOEVTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000FF00)) >> 8);
}

__INLINE void em_ble_isoevtcntl_flushcnt_setf(int elt_idx, uint8_t flushcnt)
{
    ASSERT_ERR((((uint16_t)flushcnt << 8) & ~((uint16_t)0x0000FF00)) == 0);
    EM_BLE_WR(EM_BLE_ISOEVTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_ISOEVTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x0000FF00)) | ((uint16_t)flushcnt << 8));
}

__INLINE uint8_t em_ble_isoevtcntl_subevtcnt_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOEVTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x000000FF)) >> 0);
}

__INLINE void em_ble_isoevtcntl_subevtcnt_setf(int elt_idx, uint8_t subevtcnt)
{
    ASSERT_ERR((((uint16_t)subevtcnt << 0) & ~((uint16_t)0x000000FF)) == 0);
    EM_BLE_WR(EM_BLE_ISOEVTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_ISOEVTCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x000000FF)) | ((uint16_t)subevtcnt << 0));
}

/**
 * @brief ISOTXRXCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15       ISORXBUFF_FULL   0
 *     13               ISOCIE   0
 *     12                ISOSN   0
 *     11              ISONESN   0
 *     09              ISORETX   0
 *     08              ISORSVD   0
 *     03           ISOWAITACK   0
 *     02           ISOLASTCIE   0
 *     01          ISOLASTNESN   0
 *     00            ISOLASTSN   0
 * </pre>
 */
#define EM_BLE_ISOTXRXCNTL_ADDR   (0x204F006A + EM_BLE_CS_OFFSET)
#define EM_BLE_ISOTXRXCNTL_INDEX  0x00000035
#define EM_BLE_ISOTXRXCNTL_RESET  0x00000000

__INLINE uint16_t em_ble_isotxrxcntl_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_isotxrxcntl_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_ISORXBUFF_FULL_BIT    ((uint16_t)0x00008000)
#define EM_BLE_ISORXBUFF_FULL_POS    15
#define EM_BLE_ISOCIE_BIT            ((uint16_t)0x00002000)
#define EM_BLE_ISOCIE_POS            13
#define EM_BLE_ISOSN_BIT             ((uint16_t)0x00001000)
#define EM_BLE_ISOSN_POS             12
#define EM_BLE_ISONESN_BIT           ((uint16_t)0x00000800)
#define EM_BLE_ISONESN_POS           11
#define EM_BLE_ISORETX_BIT           ((uint16_t)0x00000200)
#define EM_BLE_ISORETX_POS           9
#define EM_BLE_ISORSVD_BIT           ((uint16_t)0x00000100)
#define EM_BLE_ISORSVD_POS           8
#define EM_BLE_ISOWAITACK_BIT        ((uint16_t)0x00000008)
#define EM_BLE_ISOWAITACK_POS        3
#define EM_BLE_ISOLASTCIE_BIT        ((uint16_t)0x00000004)
#define EM_BLE_ISOLASTCIE_POS        2
#define EM_BLE_ISOLASTNESN_BIT       ((uint16_t)0x00000002)
#define EM_BLE_ISOLASTNESN_POS       1
#define EM_BLE_ISOLASTSN_BIT         ((uint16_t)0x00000001)
#define EM_BLE_ISOLASTSN_POS         0

#define EM_BLE_ISORXBUFF_FULL_RST    0x0
#define EM_BLE_ISOCIE_RST            0x0
#define EM_BLE_ISOSN_RST             0x0
#define EM_BLE_ISONESN_RST           0x0
#define EM_BLE_ISORETX_RST           0x0
#define EM_BLE_ISORSVD_RST           0x0
#define EM_BLE_ISOWAITACK_RST        0x0
#define EM_BLE_ISOLASTCIE_RST        0x0
#define EM_BLE_ISOLASTNESN_RST       0x0
#define EM_BLE_ISOLASTSN_RST         0x0

__INLINE void em_ble_isotxrxcntl_pack(int elt_idx, uint8_t isorxbufffull, uint8_t isocie, uint8_t isosn, uint8_t isonesn, uint8_t isoretx, uint8_t isorsvd, uint8_t isowaitack, uint8_t isolastcie, uint8_t isolastnesn, uint8_t isolastsn)
{
    ASSERT_ERR((((uint16_t)isorxbufffull << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)isocie << 13) & ~((uint16_t)0x00002000)) == 0);
    ASSERT_ERR((((uint16_t)isosn << 12) & ~((uint16_t)0x00001000)) == 0);
    ASSERT_ERR((((uint16_t)isonesn << 11) & ~((uint16_t)0x00000800)) == 0);
    ASSERT_ERR((((uint16_t)isoretx << 9) & ~((uint16_t)0x00000200)) == 0);
    ASSERT_ERR((((uint16_t)isorsvd << 8) & ~((uint16_t)0x00000100)) == 0);
    ASSERT_ERR((((uint16_t)isowaitack << 3) & ~((uint16_t)0x00000008)) == 0);
    ASSERT_ERR((((uint16_t)isolastcie << 2) & ~((uint16_t)0x00000004)) == 0);
    ASSERT_ERR((((uint16_t)isolastnesn << 1) & ~((uint16_t)0x00000002)) == 0);
    ASSERT_ERR((((uint16_t)isolastsn << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BLE_WR(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, ((uint16_t)isorxbufffull << 15) | ((uint16_t)isocie << 13) | ((uint16_t)isosn << 12) | ((uint16_t)isonesn << 11) | ((uint16_t)isoretx << 9) | ((uint16_t)isorsvd << 8) | ((uint16_t)isowaitack << 3) | ((uint16_t)isolastcie << 2) | ((uint16_t)isolastnesn << 1) | ((uint16_t)isolastsn << 0));
}

__INLINE void em_ble_isotxrxcntl_unpack(int elt_idx, uint8_t *isorxbufffull, uint8_t *isocie, uint8_t *isosn, uint8_t *isonesn, uint8_t *isoretx, uint8_t *isorsvd, uint8_t *isowaitack, uint8_t *isolastcie, uint8_t *isolastnesn, uint8_t *isolastsn)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);

    *isorxbufffull = (localVal & ((uint16_t)0x00008000)) >> 15;
    *isocie = (localVal & ((uint16_t)0x00002000)) >> 13;
    *isosn = (localVal & ((uint16_t)0x00001000)) >> 12;
    *isonesn = (localVal & ((uint16_t)0x00000800)) >> 11;
    *isoretx = (localVal & ((uint16_t)0x00000200)) >> 9;
    *isorsvd = (localVal & ((uint16_t)0x00000100)) >> 8;
    *isowaitack = (localVal & ((uint16_t)0x00000008)) >> 3;
    *isolastcie = (localVal & ((uint16_t)0x00000004)) >> 2;
    *isolastnesn = (localVal & ((uint16_t)0x00000002)) >> 1;
    *isolastsn = (localVal & ((uint16_t)0x00000001)) >> 0;
}

__INLINE uint8_t em_ble_isotxrxcntl_isorxbuff_full_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_ble_isotxrxcntl_isorxbuff_full_setf(int elt_idx, uint8_t isorxbufffull)
{
    ASSERT_ERR((((uint16_t)isorxbufffull << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BLE_WR(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)isorxbufffull << 15));
}

__INLINE uint8_t em_ble_isotxrxcntl_isocie_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00002000)) >> 13);
}

__INLINE void em_ble_isotxrxcntl_isocie_setf(int elt_idx, uint8_t isocie)
{
    ASSERT_ERR((((uint16_t)isocie << 13) & ~((uint16_t)0x00002000)) == 0);
    EM_BLE_WR(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00002000)) | ((uint16_t)isocie << 13));
}

__INLINE uint8_t em_ble_isotxrxcntl_isosn_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00001000)) >> 12);
}

__INLINE void em_ble_isotxrxcntl_isosn_setf(int elt_idx, uint8_t isosn)
{
    ASSERT_ERR((((uint16_t)isosn << 12) & ~((uint16_t)0x00001000)) == 0);
    EM_BLE_WR(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00001000)) | ((uint16_t)isosn << 12));
}

__INLINE uint8_t em_ble_isotxrxcntl_isonesn_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000800)) >> 11);
}

__INLINE void em_ble_isotxrxcntl_isonesn_setf(int elt_idx, uint8_t isonesn)
{
    ASSERT_ERR((((uint16_t)isonesn << 11) & ~((uint16_t)0x00000800)) == 0);
    EM_BLE_WR(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000800)) | ((uint16_t)isonesn << 11));
}

__INLINE uint8_t em_ble_isotxrxcntl_isoretx_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000200)) >> 9);
}

__INLINE void em_ble_isotxrxcntl_isoretx_setf(int elt_idx, uint8_t isoretx)
{
    ASSERT_ERR((((uint16_t)isoretx << 9) & ~((uint16_t)0x00000200)) == 0);
    EM_BLE_WR(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000200)) | ((uint16_t)isoretx << 9));
}

__INLINE uint8_t em_ble_isotxrxcntl_isorsvd_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000100)) >> 8);
}

__INLINE void em_ble_isotxrxcntl_isorsvd_setf(int elt_idx, uint8_t isorsvd)
{
    ASSERT_ERR((((uint16_t)isorsvd << 8) & ~((uint16_t)0x00000100)) == 0);
    EM_BLE_WR(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000100)) | ((uint16_t)isorsvd << 8));
}

__INLINE uint8_t em_ble_isotxrxcntl_isowaitack_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000008)) >> 3);
}

__INLINE void em_ble_isotxrxcntl_isowaitack_setf(int elt_idx, uint8_t isowaitack)
{
    ASSERT_ERR((((uint16_t)isowaitack << 3) & ~((uint16_t)0x00000008)) == 0);
    EM_BLE_WR(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000008)) | ((uint16_t)isowaitack << 3));
}

__INLINE uint8_t em_ble_isotxrxcntl_isolastcie_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000004)) >> 2);
}

__INLINE void em_ble_isotxrxcntl_isolastcie_setf(int elt_idx, uint8_t isolastcie)
{
    ASSERT_ERR((((uint16_t)isolastcie << 2) & ~((uint16_t)0x00000004)) == 0);
    EM_BLE_WR(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000004)) | ((uint16_t)isolastcie << 2));
}

__INLINE uint8_t em_ble_isotxrxcntl_isolastnesn_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000002)) >> 1);
}

__INLINE void em_ble_isotxrxcntl_isolastnesn_setf(int elt_idx, uint8_t isolastnesn)
{
    ASSERT_ERR((((uint16_t)isolastnesn << 1) & ~((uint16_t)0x00000002)) == 0);
    EM_BLE_WR(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000002)) | ((uint16_t)isolastnesn << 1));
}

__INLINE uint8_t em_ble_isotxrxcntl_isolastsn_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x00000001)) >> 0);
}

__INLINE void em_ble_isotxrxcntl_isolastsn_setf(int elt_idx, uint8_t isolastsn)
{
    ASSERT_ERR((((uint16_t)isolastsn << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BLE_WR(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (EM_BLE_RD(EM_BLE_ISOTXRXCNTL_ADDR + elt_idx * REG_EM_BLE_CS_SIZE) & ~((uint16_t)0x00000001)) | ((uint16_t)isolastsn << 0));
}

/**
 * @brief TXRXDESCCNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:08         ACLRXDESCCNT   0x0
 *  07:00         ACLTXDESCCNT   0x0
 * </pre>
 */
#define EM_BLE_TXRXDESCCNT_ADDR   (0x204F006C + EM_BLE_CS_OFFSET)
#define EM_BLE_TXRXDESCCNT_INDEX  0x00000036
#define EM_BLE_TXRXDESCCNT_RESET  0x00000000

__INLINE uint16_t em_ble_txrxdesccnt_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_TXRXDESCCNT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

// field definitions
#define EM_BLE_ACLRXDESCCNT_MASK   ((uint16_t)0x0000FF00)
#define EM_BLE_ACLRXDESCCNT_LSB    8
#define EM_BLE_ACLRXDESCCNT_WIDTH  ((uint16_t)0x00000008)
#define EM_BLE_ACLTXDESCCNT_MASK   ((uint16_t)0x000000FF)
#define EM_BLE_ACLTXDESCCNT_LSB    0
#define EM_BLE_ACLTXDESCCNT_WIDTH  ((uint16_t)0x00000008)

#define EM_BLE_ACLRXDESCCNT_RST    0x0
#define EM_BLE_ACLTXDESCCNT_RST    0x0

__INLINE void em_ble_txrxdesccnt_unpack(int elt_idx, uint8_t *aclrxdesccnt, uint8_t *acltxdesccnt)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXRXDESCCNT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);

    *aclrxdesccnt = (localVal & ((uint16_t)0x0000FF00)) >> 8;
    *acltxdesccnt = (localVal & ((uint16_t)0x000000FF)) >> 0;
}

__INLINE uint8_t em_ble_txrxdesccnt_aclrxdesccnt_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXRXDESCCNT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000FF00)) >> 8);
}

__INLINE uint8_t em_ble_txrxdesccnt_acltxdesccnt_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_TXRXDESCCNT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x000000FF)) >> 0);
}

/**
 * @brief ISOTXRXPKTCNT register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:08         ISORXPKTCNTL   0x0
 *  07:00         ISOTXPKTCNTL   0x0
 * </pre>
 */
#define EM_BLE_ISOTXRXPKTCNT_ADDR   (0x204F006E + EM_BLE_CS_OFFSET)
#define EM_BLE_ISOTXRXPKTCNT_INDEX  0x00000037
#define EM_BLE_ISOTXRXPKTCNT_RESET  0x00000000

__INLINE uint16_t em_ble_isotxrxpktcnt_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_ISOTXRXPKTCNT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

// field definitions
#define EM_BLE_ISORXPKTCNTL_MASK   ((uint16_t)0x0000FF00)
#define EM_BLE_ISORXPKTCNTL_LSB    8
#define EM_BLE_ISORXPKTCNTL_WIDTH  ((uint16_t)0x00000008)
#define EM_BLE_ISOTXPKTCNTL_MASK   ((uint16_t)0x000000FF)
#define EM_BLE_ISOTXPKTCNTL_LSB    0
#define EM_BLE_ISOTXPKTCNTL_WIDTH  ((uint16_t)0x00000008)

#define EM_BLE_ISORXPKTCNTL_RST    0x0
#define EM_BLE_ISOTXPKTCNTL_RST    0x0

__INLINE void em_ble_isotxrxpktcnt_unpack(int elt_idx, uint8_t *isorxpktcntl, uint8_t *isotxpktcntl)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOTXRXPKTCNT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);

    *isorxpktcntl = (localVal & ((uint16_t)0x0000FF00)) >> 8;
    *isotxpktcntl = (localVal & ((uint16_t)0x000000FF)) >> 0;
}

__INLINE uint8_t em_ble_isotxrxpktcnt_isorxpktcntl_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOTXRXPKTCNT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x0000FF00)) >> 8);
}

__INLINE uint8_t em_ble_isotxrxpktcnt_isotxpktcntl_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_ISOTXRXPKTCNT_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    return ((localVal & ((uint16_t)0x000000FF)) >> 0);
}

/**
 * @brief COUNTER0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00             COUNTER0   0x0
 * </pre>
 */
#define EM_BLE_COUNTER0_ADDR   (0x204F0070 + EM_BLE_CS_OFFSET)
#define EM_BLE_COUNTER0_INDEX  0x00000038
#define EM_BLE_COUNTER0_RESET  0x00000000
#define EM_BLE_COUNTER0_COUNT  2

__INLINE uint16_t em_ble_counter0_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 1);
    return EM_BLE_RD(EM_BLE_COUNTER0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
}

__INLINE void em_ble_counter0_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 1);
    EM_BLE_WR(EM_BLE_COUNTER0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BLE_COUNTER0_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_COUNTER0_LSB    0
#define EM_BLE_COUNTER0_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_COUNTER0_RST    0x0

__INLINE uint16_t em_ble_counter0_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 1);
    uint16_t localVal = EM_BLE_RD(EM_BLE_COUNTER0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_counter0_setf(int elt_idx, int reg_idx, uint16_t counter0)
{
    ASSERT_ERR(reg_idx <= 1);
    ASSERT_ERR((((uint16_t)counter0 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_COUNTER0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, (uint16_t)counter0 << 0);
}

/**
 * @brief COUNTER1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00             COUNTER1   0x0
 * </pre>
 */
#define EM_BLE_COUNTER1_ADDR   (0x204F0074 + EM_BLE_CS_OFFSET)
#define EM_BLE_COUNTER1_INDEX  0x0000003A
#define EM_BLE_COUNTER1_RESET  0x00000000
#define EM_BLE_COUNTER1_COUNT  2

__INLINE uint16_t em_ble_counter1_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 1);
    return EM_BLE_RD(EM_BLE_COUNTER1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
}

__INLINE void em_ble_counter1_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 1);
    EM_BLE_WR(EM_BLE_COUNTER1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BLE_COUNTER1_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_COUNTER1_LSB    0
#define EM_BLE_COUNTER1_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_COUNTER1_RST    0x0

__INLINE uint16_t em_ble_counter1_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 1);
    uint16_t localVal = EM_BLE_RD(EM_BLE_COUNTER1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_counter1_setf(int elt_idx, int reg_idx, uint16_t counter1)
{
    ASSERT_ERR(reg_idx <= 1);
    ASSERT_ERR((((uint16_t)counter1 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_COUNTER1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, (uint16_t)counter1 << 0);
}

/**
 * @brief COUNTER2 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00             COUNTER2   0x0
 * </pre>
 */
#define EM_BLE_COUNTER2_ADDR   (0x204F0078 + EM_BLE_CS_OFFSET)
#define EM_BLE_COUNTER2_INDEX  0x0000003C
#define EM_BLE_COUNTER2_RESET  0x00000000
#define EM_BLE_COUNTER2_COUNT  2

__INLINE uint16_t em_ble_counter2_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 1);
    return EM_BLE_RD(EM_BLE_COUNTER2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
}

__INLINE void em_ble_counter2_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 1);
    EM_BLE_WR(EM_BLE_COUNTER2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BLE_COUNTER2_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_COUNTER2_LSB    0
#define EM_BLE_COUNTER2_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_COUNTER2_RST    0x0

__INLINE uint16_t em_ble_counter2_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 1);
    uint16_t localVal = EM_BLE_RD(EM_BLE_COUNTER2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_counter2_setf(int elt_idx, int reg_idx, uint16_t counter2)
{
    ASSERT_ERR(reg_idx <= 1);
    ASSERT_ERR((((uint16_t)counter2 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_COUNTER2_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, (uint16_t)counter2 << 0);
}

/**
 * @brief COUNTER3 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00             COUNTER3   0x0
 * </pre>
 */
#define EM_BLE_COUNTER3_ADDR   (0x204F007C + EM_BLE_CS_OFFSET)
#define EM_BLE_COUNTER3_INDEX  0x0000003E
#define EM_BLE_COUNTER3_RESET  0x00000000
#define EM_BLE_COUNTER3_COUNT  2

__INLINE uint16_t em_ble_counter3_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 1);
    return EM_BLE_RD(EM_BLE_COUNTER3_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
}

__INLINE void em_ble_counter3_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 1);
    EM_BLE_WR(EM_BLE_COUNTER3_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BLE_COUNTER3_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_COUNTER3_LSB    0
#define EM_BLE_COUNTER3_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_COUNTER3_RST    0x0

__INLINE uint16_t em_ble_counter3_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 1);
    uint16_t localVal = EM_BLE_RD(EM_BLE_COUNTER3_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_counter3_setf(int elt_idx, int reg_idx, uint16_t counter3)
{
    ASSERT_ERR(reg_idx <= 1);
    ASSERT_ERR((((uint16_t)counter3 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_COUNTER3_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, (uint16_t)counter3 << 0);
}

/**
 * @brief COUNTER4 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00             COUNTER4   0x0
 * </pre>
 */
#define EM_BLE_COUNTER4_ADDR   (0x204F0080 + EM_BLE_CS_OFFSET)
#define EM_BLE_COUNTER4_INDEX  0x00000040
#define EM_BLE_COUNTER4_RESET  0x00000000
#define EM_BLE_COUNTER4_COUNT  2

__INLINE uint16_t em_ble_counter4_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 1);
    return EM_BLE_RD(EM_BLE_COUNTER4_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
}

__INLINE void em_ble_counter4_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 1);
    EM_BLE_WR(EM_BLE_COUNTER4_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BLE_COUNTER4_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_COUNTER4_LSB    0
#define EM_BLE_COUNTER4_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_COUNTER4_RST    0x0

__INLINE uint16_t em_ble_counter4_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 1);
    uint16_t localVal = EM_BLE_RD(EM_BLE_COUNTER4_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_counter4_setf(int elt_idx, int reg_idx, uint16_t counter4)
{
    ASSERT_ERR(reg_idx <= 1);
    ASSERT_ERR((((uint16_t)counter4 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_COUNTER4_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, (uint16_t)counter4 << 0);
}

/**
 * @brief COUNTER5 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00             COUNTER5   0x0
 * </pre>
 */
#define EM_BLE_COUNTER5_ADDR   (0x204F0084 + EM_BLE_CS_OFFSET)
#define EM_BLE_COUNTER5_INDEX  0x00000042
#define EM_BLE_COUNTER5_RESET  0x00000000
#define EM_BLE_COUNTER5_COUNT  2

__INLINE uint16_t em_ble_counter5_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 1);
    return EM_BLE_RD(EM_BLE_COUNTER5_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
}

__INLINE void em_ble_counter5_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 1);
    EM_BLE_WR(EM_BLE_COUNTER5_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BLE_COUNTER5_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_COUNTER5_LSB    0
#define EM_BLE_COUNTER5_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_COUNTER5_RST    0x0

__INLINE uint16_t em_ble_counter5_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 1);
    uint16_t localVal = EM_BLE_RD(EM_BLE_COUNTER5_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_counter5_setf(int elt_idx, int reg_idx, uint16_t counter5)
{
    ASSERT_ERR(reg_idx <= 1);
    ASSERT_ERR((((uint16_t)counter5 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_COUNTER5_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, (uint16_t)counter5 << 0);
}

/**
 * @brief COUNTER6 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00             COUNTER6   0x0
 * </pre>
 */
#define EM_BLE_COUNTER6_ADDR   (0x204F0088 + EM_BLE_CS_OFFSET)
#define EM_BLE_COUNTER6_INDEX  0x00000044
#define EM_BLE_COUNTER6_RESET  0x00000000
#define EM_BLE_COUNTER6_COUNT  2

__INLINE uint16_t em_ble_counter6_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 1);
    return EM_BLE_RD(EM_BLE_COUNTER6_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
}

__INLINE void em_ble_counter6_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 1);
    EM_BLE_WR(EM_BLE_COUNTER6_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BLE_COUNTER6_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_COUNTER6_LSB    0
#define EM_BLE_COUNTER6_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_COUNTER6_RST    0x0

__INLINE uint16_t em_ble_counter6_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 1);
    uint16_t localVal = EM_BLE_RD(EM_BLE_COUNTER6_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_counter6_setf(int elt_idx, int reg_idx, uint16_t counter6)
{
    ASSERT_ERR(reg_idx <= 1);
    ASSERT_ERR((((uint16_t)counter6 << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_COUNTER6_ADDR + elt_idx * REG_EM_BLE_CS_SIZE + reg_idx * 2, (uint16_t)counter6 << 0);
}

/**
 * @brief HOPCS2CNTL0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00              PRNRETX   0x0
 * </pre>
 */
#define EM_BLE_HOPCS2CNTL0_ADDR   (0x204F008C + EM_BLE_CS_OFFSET)
#define EM_BLE_HOPCS2CNTL0_INDEX  0x00000046
#define EM_BLE_HOPCS2CNTL0_RESET  0x00000000

__INLINE uint16_t em_ble_hopcs2cntl0_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_HOPCS2CNTL0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_hopcs2cntl0_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_HOPCS2CNTL0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_PRNRETX_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_PRNRETX_LSB    0
#define EM_BLE_PRNRETX_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_PRNRETX_RST    0x0

__INLINE uint16_t em_ble_hopcs2cntl0_prnretx_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_HOPCS2CNTL0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_hopcs2cntl0_prnretx_setf(int elt_idx, uint16_t prnretx)
{
    ASSERT_ERR((((uint16_t)prnretx << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_HOPCS2CNTL0_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)prnretx << 0);
}

/**
 * @brief HOPCS2CNTL1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00           RETXMAPIDX   0x0
 * </pre>
 */
#define EM_BLE_HOPCS2CNTL1_ADDR   (0x204F008E + EM_BLE_CS_OFFSET)
#define EM_BLE_HOPCS2CNTL1_INDEX  0x00000047
#define EM_BLE_HOPCS2CNTL1_RESET  0x00000000

__INLINE uint16_t em_ble_hopcs2cntl1_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_HOPCS2CNTL1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_hopcs2cntl1_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_HOPCS2CNTL1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_RETXMAPIDX_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_RETXMAPIDX_LSB    0
#define EM_BLE_RETXMAPIDX_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_RETXMAPIDX_RST    0x0

__INLINE uint16_t em_ble_hopcs2cntl1_retxmapidx_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_HOPCS2CNTL1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_hopcs2cntl1_retxmapidx_setf(int elt_idx, uint16_t retxmapidx)
{
    ASSERT_ERR((((uint16_t)retxmapidx << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_HOPCS2CNTL1_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)retxmapidx << 0);
}

/**
 * @brief HOPPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  13:00          HOP_SEQ_PTR   0x0
 * </pre>
 */
#define EM_BLE_HOPPTR_ADDR   (0x204F0090 + EM_BLE_CS_OFFSET)
#define EM_BLE_HOPPTR_INDEX  0x00000048
#define EM_BLE_HOPPTR_RESET  0x00000000

__INLINE uint16_t em_ble_hopptr_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_HOPPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
}

__INLINE void em_ble_hopptr_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_HOPPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, value);
}

// field definitions
#define EM_BLE_HOP_SEQ_PTR_MASK   ((uint16_t)0x00003FFF)
#define EM_BLE_HOP_SEQ_PTR_LSB    0
#define EM_BLE_HOP_SEQ_PTR_WIDTH  ((uint16_t)0x0000000E)

#define EM_BLE_HOP_SEQ_PTR_RST    0x0

__INLINE uint16_t em_ble_hopptr_hop_seq_ptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_HOPPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x00003FFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_hopptr_hop_seq_ptr_setf(int elt_idx, uint16_t hopseqptr)
{
    ASSERT_ERR((((uint16_t)hopseqptr << 0) & ~((uint16_t)0x00003FFF)) == 0);
    EM_BLE_WR(EM_BLE_HOPPTR_ADDR + elt_idx * REG_EM_BLE_CS_SIZE, (uint16_t)hopseqptr << 0);
}


#endif // _REG_EM_BLE_CS_H_

