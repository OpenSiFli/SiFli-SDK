#ifndef _REG_EM_BLE_RX_DESC_H_
#define _REG_EM_BLE_RX_DESC_H_

#include <stdint.h>
#include "_reg_em_ble_rx_desc.h"
#include "compiler.h"
#include "arch.h"
#include "em_map.h"
#include "reg_access.h"

#define REG_EM_BLE_RX_DESC_COUNT 10

#define REG_EM_BLE_RX_DESC_DECODING_MASK 0x0000001F

#define REG_EM_BLE_RX_DESC_ADDR_GET(idx) (EM_BLE_RX_DESC_OFFSET + (idx) * REG_EM_BLE_RX_DESC_SIZE)

/**
 * @brief RXCNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15               RXDONE   0
 *  14:00            RXNEXTPTR   0x0
 * </pre>
 */
#define EM_BLE_RXCNTL_ADDR   (0x40110000 + EM_BLE_RX_DESC_OFFSET)
#define EM_BLE_RXCNTL_INDEX  0x00000000
#define EM_BLE_RXCNTL_RESET  0x00000000

__INLINE uint16_t em_ble_rxcntl_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXCNTL_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
}

__INLINE void em_ble_rxcntl_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXCNTL_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_RXDONE_BIT       ((uint16_t)0x00008000)
#define EM_BLE_RXDONE_POS       15
#define EM_BLE_RXNEXTPTR_MASK   ((uint16_t)0x00007FFF)
#define EM_BLE_RXNEXTPTR_LSB    0
#define EM_BLE_RXNEXTPTR_WIDTH  ((uint16_t)0x0000000F)

#define EM_BLE_RXDONE_RST       0x0
#define EM_BLE_RXNEXTPTR_RST    0x0

__INLINE void em_ble_rxcntl_pack(int elt_idx, uint8_t rxdone, uint16_t rxnextptr)
{
    ASSERT_ERR((((uint16_t)rxdone << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)rxnextptr << 0) & ~((uint16_t)0x00007FFF)) == 0);
    EM_BLE_WR(EM_BLE_RXCNTL_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, ((uint16_t)rxdone << 15) | ((uint16_t)rxnextptr << 0));
}

__INLINE void em_ble_rxcntl_unpack(int elt_idx, uint8_t *rxdone, uint16_t *rxnextptr)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXCNTL_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);

    *rxdone = (localVal & ((uint16_t)0x00008000)) >> 15;
    *rxnextptr = (localVal & ((uint16_t)0x00007FFF)) >> 0;
}

__INLINE uint8_t em_ble_rxcntl_rxdone_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXCNTL_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_ble_rxcntl_rxdone_setf(int elt_idx, uint8_t rxdone)
{
    ASSERT_ERR((((uint16_t)rxdone << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BLE_WR(EM_BLE_RXCNTL_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXCNTL_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)rxdone << 15));
}

__INLINE uint16_t em_ble_rxcntl_rxnextptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXCNTL_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00007FFF)) >> 0);
}

__INLINE void em_ble_rxcntl_rxnextptr_setf(int elt_idx, uint16_t rxnextptr)
{
    ASSERT_ERR((((uint16_t)rxnextptr << 0) & ~((uint16_t)0x00007FFF)) == 0);
    EM_BLE_WR(EM_BLE_RXCNTL_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXCNTL_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00007FFF)) | ((uint16_t)rxnextptr << 0));
}

/**
 * @brief RXSTATCE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     07             NESN_ERR   0
 *     06               SN_ERR   0
 *     05             LLID_ERR   0
 *     04              MIC_ERR   0
 *     03              CRC_ERR   0
 *     02              LEN_ERR   0
 *     01           RXTIME_ERR   0
 *     00             SYNC_ERR   0
 * </pre>
 */
#define EM_BLE_RXSTATCE_ADDR   (0x40110002 + EM_BLE_RX_DESC_OFFSET)
#define EM_BLE_RXSTATCE_INDEX  0x00000001
#define EM_BLE_RXSTATCE_RESET  0x00000000

__INLINE uint16_t em_ble_rxstatce_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
}

__INLINE void em_ble_rxstatce_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_NESN_ERR_BIT      ((uint16_t)0x00000080)
#define EM_BLE_NESN_ERR_POS      7
#define EM_BLE_SN_ERR_BIT        ((uint16_t)0x00000040)
#define EM_BLE_SN_ERR_POS        6
#define EM_BLE_LLID_ERR_BIT      ((uint16_t)0x00000020)
#define EM_BLE_LLID_ERR_POS      5
#define EM_BLE_MIC_ERR_BIT       ((uint16_t)0x00000010)
#define EM_BLE_MIC_ERR_POS       4
#define EM_BLE_CRC_ERR_BIT       ((uint16_t)0x00000008)
#define EM_BLE_CRC_ERR_POS       3
#define EM_BLE_LEN_ERR_BIT       ((uint16_t)0x00000004)
#define EM_BLE_LEN_ERR_POS       2
#define EM_BLE_RXTIME_ERR_BIT    ((uint16_t)0x00000002)
#define EM_BLE_RXTIME_ERR_POS    1
#define EM_BLE_SYNC_ERR_BIT      ((uint16_t)0x00000001)
#define EM_BLE_SYNC_ERR_POS      0

#define EM_BLE_NESN_ERR_RST      0x0
#define EM_BLE_SN_ERR_RST        0x0
#define EM_BLE_LLID_ERR_RST      0x0
#define EM_BLE_MIC_ERR_RST       0x0
#define EM_BLE_CRC_ERR_RST       0x0
#define EM_BLE_LEN_ERR_RST       0x0
#define EM_BLE_RXTIME_ERR_RST    0x0
#define EM_BLE_SYNC_ERR_RST      0x0

__INLINE void em_ble_rxstatce_pack(int elt_idx, uint8_t nesnerr, uint8_t snerr, uint8_t lliderr, uint8_t micerr, uint8_t crcerr, uint8_t lenerr, uint8_t rxtimeerr, uint8_t syncerr)
{
    ASSERT_ERR((((uint16_t)nesnerr << 7) & ~((uint16_t)0x00000080)) == 0);
    ASSERT_ERR((((uint16_t)snerr << 6) & ~((uint16_t)0x00000040)) == 0);
    ASSERT_ERR((((uint16_t)lliderr << 5) & ~((uint16_t)0x00000020)) == 0);
    ASSERT_ERR((((uint16_t)micerr << 4) & ~((uint16_t)0x00000010)) == 0);
    ASSERT_ERR((((uint16_t)crcerr << 3) & ~((uint16_t)0x00000008)) == 0);
    ASSERT_ERR((((uint16_t)lenerr << 2) & ~((uint16_t)0x00000004)) == 0);
    ASSERT_ERR((((uint16_t)rxtimeerr << 1) & ~((uint16_t)0x00000002)) == 0);
    ASSERT_ERR((((uint16_t)syncerr << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, ((uint16_t)nesnerr << 7) | ((uint16_t)snerr << 6) | ((uint16_t)lliderr << 5) | ((uint16_t)micerr << 4) | ((uint16_t)crcerr << 3) | ((uint16_t)lenerr << 2) | ((uint16_t)rxtimeerr << 1) | ((uint16_t)syncerr << 0));
}

__INLINE void em_ble_rxstatce_unpack(int elt_idx, uint8_t *nesnerr, uint8_t *snerr, uint8_t *lliderr, uint8_t *micerr, uint8_t *crcerr, uint8_t *lenerr, uint8_t *rxtimeerr, uint8_t *syncerr)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);

    *nesnerr = (localVal & ((uint16_t)0x00000080)) >> 7;
    *snerr = (localVal & ((uint16_t)0x00000040)) >> 6;
    *lliderr = (localVal & ((uint16_t)0x00000020)) >> 5;
    *micerr = (localVal & ((uint16_t)0x00000010)) >> 4;
    *crcerr = (localVal & ((uint16_t)0x00000008)) >> 3;
    *lenerr = (localVal & ((uint16_t)0x00000004)) >> 2;
    *rxtimeerr = (localVal & ((uint16_t)0x00000002)) >> 1;
    *syncerr = (localVal & ((uint16_t)0x00000001)) >> 0;
}

__INLINE uint8_t em_ble_rxstatce_nesn_err_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000080)) >> 7);
}

__INLINE void em_ble_rxstatce_nesn_err_setf(int elt_idx, uint8_t nesnerr)
{
    ASSERT_ERR((((uint16_t)nesnerr << 7) & ~((uint16_t)0x00000080)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000080)) | ((uint16_t)nesnerr << 7));
}

__INLINE uint8_t em_ble_rxstatce_sn_err_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000040)) >> 6);
}

__INLINE void em_ble_rxstatce_sn_err_setf(int elt_idx, uint8_t snerr)
{
    ASSERT_ERR((((uint16_t)snerr << 6) & ~((uint16_t)0x00000040)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000040)) | ((uint16_t)snerr << 6));
}

__INLINE uint8_t em_ble_rxstatce_llid_err_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000020)) >> 5);
}

__INLINE void em_ble_rxstatce_llid_err_setf(int elt_idx, uint8_t lliderr)
{
    ASSERT_ERR((((uint16_t)lliderr << 5) & ~((uint16_t)0x00000020)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000020)) | ((uint16_t)lliderr << 5));
}

__INLINE uint8_t em_ble_rxstatce_mic_err_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000010)) >> 4);
}

__INLINE void em_ble_rxstatce_mic_err_setf(int elt_idx, uint8_t micerr)
{
    ASSERT_ERR((((uint16_t)micerr << 4) & ~((uint16_t)0x00000010)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000010)) | ((uint16_t)micerr << 4));
}

__INLINE uint8_t em_ble_rxstatce_crc_err_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000008)) >> 3);
}

__INLINE void em_ble_rxstatce_crc_err_setf(int elt_idx, uint8_t crcerr)
{
    ASSERT_ERR((((uint16_t)crcerr << 3) & ~((uint16_t)0x00000008)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000008)) | ((uint16_t)crcerr << 3));
}

__INLINE uint8_t em_ble_rxstatce_len_err_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000004)) >> 2);
}

__INLINE void em_ble_rxstatce_len_err_setf(int elt_idx, uint8_t lenerr)
{
    ASSERT_ERR((((uint16_t)lenerr << 2) & ~((uint16_t)0x00000004)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000004)) | ((uint16_t)lenerr << 2));
}

__INLINE uint8_t em_ble_rxstatce_rxtime_err_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000002)) >> 1);
}

__INLINE void em_ble_rxstatce_rxtime_err_setf(int elt_idx, uint8_t rxtimeerr)
{
    ASSERT_ERR((((uint16_t)rxtimeerr << 1) & ~((uint16_t)0x00000002)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000002)) | ((uint16_t)rxtimeerr << 1));
}

__INLINE uint8_t em_ble_rxstatce_sync_err_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000001)) >> 0);
}

__INLINE void em_ble_rxstatce_sync_err_setf(int elt_idx, uint8_t syncerr)
{
    ASSERT_ERR((((uint16_t)syncerr << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXSTATCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000001)) | ((uint16_t)syncerr << 0));
}

/**
 * @brief RXSTATADV register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     12     DEV_FILTERING_OK   0
 *     11               IN_WHL   0
 *     10          IN_PERADVAL   0
 *     09           FOLLOW_ADI   0
 *     08            ADI_MATCH   0
 *     07       PEER_ADD_MATCH   0
 *     06         BDADDR_MATCH   0
 *     05             TYPE_ERR   0
 *     04             PRIV_ERR   0
 *     03              CRC_ERR   0
 *     02              LEN_ERR   0
 *     01           RXTIME_ERR   0
 *     00             SYNC_ERR   0
 * </pre>
 */
#define EM_BLE_RXSTATADV_ADDR   (0x40110002 + EM_BLE_RX_DESC_OFFSET)
#define EM_BLE_RXSTATADV_INDEX  0x00000001
#define EM_BLE_RXSTATADV_RESET  0x00000000

__INLINE uint16_t em_ble_rxstatadv_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
}

__INLINE void em_ble_rxstatadv_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_DEV_FILTERING_OK_BIT    ((uint16_t)0x00001000)
#define EM_BLE_DEV_FILTERING_OK_POS    12
#define EM_BLE_IN_WHL_BIT              ((uint16_t)0x00000800)
#define EM_BLE_IN_WHL_POS              11
#define EM_BLE_IN_PERADVAL_BIT         ((uint16_t)0x00000400)
#define EM_BLE_IN_PERADVAL_POS         10
#define EM_BLE_FOLLOW_ADI_BIT          ((uint16_t)0x00000200)
#define EM_BLE_FOLLOW_ADI_POS          9
#define EM_BLE_ADI_MATCH_BIT           ((uint16_t)0x00000100)
#define EM_BLE_ADI_MATCH_POS           8
#define EM_BLE_PEER_ADD_MATCH_BIT      ((uint16_t)0x00000080)
#define EM_BLE_PEER_ADD_MATCH_POS      7
#define EM_BLE_BDADDR_MATCH_BIT        ((uint16_t)0x00000040)
#define EM_BLE_BDADDR_MATCH_POS        6
#define EM_BLE_TYPE_ERR_BIT            ((uint16_t)0x00000020)
#define EM_BLE_TYPE_ERR_POS            5
#define EM_BLE_PRIV_ERR_BIT            ((uint16_t)0x00000010)
#define EM_BLE_PRIV_ERR_POS            4
#define EM_BLE_CRC_ERR_BIT             ((uint16_t)0x00000008)
#define EM_BLE_CRC_ERR_POS             3
#define EM_BLE_LEN_ERR_BIT             ((uint16_t)0x00000004)
#define EM_BLE_LEN_ERR_POS             2
#define EM_BLE_RXTIME_ERR_BIT          ((uint16_t)0x00000002)
#define EM_BLE_RXTIME_ERR_POS          1
#define EM_BLE_SYNC_ERR_BIT            ((uint16_t)0x00000001)
#define EM_BLE_SYNC_ERR_POS            0

#define EM_BLE_DEV_FILTERING_OK_RST    0x0
#define EM_BLE_IN_WHL_RST              0x0
#define EM_BLE_IN_PERADVAL_RST         0x0
#define EM_BLE_FOLLOW_ADI_RST          0x0
#define EM_BLE_ADI_MATCH_RST           0x0
#define EM_BLE_PEER_ADD_MATCH_RST      0x0
#define EM_BLE_BDADDR_MATCH_RST        0x0
#define EM_BLE_TYPE_ERR_RST            0x0
#define EM_BLE_PRIV_ERR_RST            0x0
#define EM_BLE_CRC_ERR_RST             0x0
#define EM_BLE_LEN_ERR_RST             0x0
#define EM_BLE_RXTIME_ERR_RST          0x0
#define EM_BLE_SYNC_ERR_RST            0x0

__INLINE void em_ble_rxstatadv_pack(int elt_idx, uint8_t devfilteringok, uint8_t inwhl, uint8_t inperadval, uint8_t followadi, uint8_t adimatch, uint8_t peeraddmatch, uint8_t bdaddrmatch, uint8_t typeerr, uint8_t priverr, uint8_t crcerr, uint8_t lenerr, uint8_t rxtimeerr, uint8_t syncerr)
{
    ASSERT_ERR((((uint16_t)devfilteringok << 12) & ~((uint16_t)0x00001000)) == 0);
    ASSERT_ERR((((uint16_t)inwhl << 11) & ~((uint16_t)0x00000800)) == 0);
    ASSERT_ERR((((uint16_t)inperadval << 10) & ~((uint16_t)0x00000400)) == 0);
    ASSERT_ERR((((uint16_t)followadi << 9) & ~((uint16_t)0x00000200)) == 0);
    ASSERT_ERR((((uint16_t)adimatch << 8) & ~((uint16_t)0x00000100)) == 0);
    ASSERT_ERR((((uint16_t)peeraddmatch << 7) & ~((uint16_t)0x00000080)) == 0);
    ASSERT_ERR((((uint16_t)bdaddrmatch << 6) & ~((uint16_t)0x00000040)) == 0);
    ASSERT_ERR((((uint16_t)typeerr << 5) & ~((uint16_t)0x00000020)) == 0);
    ASSERT_ERR((((uint16_t)priverr << 4) & ~((uint16_t)0x00000010)) == 0);
    ASSERT_ERR((((uint16_t)crcerr << 3) & ~((uint16_t)0x00000008)) == 0);
    ASSERT_ERR((((uint16_t)lenerr << 2) & ~((uint16_t)0x00000004)) == 0);
    ASSERT_ERR((((uint16_t)rxtimeerr << 1) & ~((uint16_t)0x00000002)) == 0);
    ASSERT_ERR((((uint16_t)syncerr << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, ((uint16_t)devfilteringok << 12) | ((uint16_t)inwhl << 11) | ((uint16_t)inperadval << 10) | ((uint16_t)followadi << 9) | ((uint16_t)adimatch << 8) | ((uint16_t)peeraddmatch << 7) | ((uint16_t)bdaddrmatch << 6) | ((uint16_t)typeerr << 5) | ((uint16_t)priverr << 4) | ((uint16_t)crcerr << 3) | ((uint16_t)lenerr << 2) | ((uint16_t)rxtimeerr << 1) | ((uint16_t)syncerr << 0));
}

__INLINE void em_ble_rxstatadv_unpack(int elt_idx, uint8_t *devfilteringok, uint8_t *inwhl, uint8_t *inperadval, uint8_t *followadi, uint8_t *adimatch, uint8_t *peeraddmatch, uint8_t *bdaddrmatch, uint8_t *typeerr, uint8_t *priverr, uint8_t *crcerr, uint8_t *lenerr, uint8_t *rxtimeerr, uint8_t *syncerr)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);

    *devfilteringok = (localVal & ((uint16_t)0x00001000)) >> 12;
    *inwhl = (localVal & ((uint16_t)0x00000800)) >> 11;
    *inperadval = (localVal & ((uint16_t)0x00000400)) >> 10;
    *followadi = (localVal & ((uint16_t)0x00000200)) >> 9;
    *adimatch = (localVal & ((uint16_t)0x00000100)) >> 8;
    *peeraddmatch = (localVal & ((uint16_t)0x00000080)) >> 7;
    *bdaddrmatch = (localVal & ((uint16_t)0x00000040)) >> 6;
    *typeerr = (localVal & ((uint16_t)0x00000020)) >> 5;
    *priverr = (localVal & ((uint16_t)0x00000010)) >> 4;
    *crcerr = (localVal & ((uint16_t)0x00000008)) >> 3;
    *lenerr = (localVal & ((uint16_t)0x00000004)) >> 2;
    *rxtimeerr = (localVal & ((uint16_t)0x00000002)) >> 1;
    *syncerr = (localVal & ((uint16_t)0x00000001)) >> 0;
}

__INLINE uint8_t em_ble_rxstatadv_dev_filtering_ok_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00001000)) >> 12);
}

__INLINE void em_ble_rxstatadv_dev_filtering_ok_setf(int elt_idx, uint8_t devfilteringok)
{
    ASSERT_ERR((((uint16_t)devfilteringok << 12) & ~((uint16_t)0x00001000)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00001000)) | ((uint16_t)devfilteringok << 12));
}

__INLINE uint8_t em_ble_rxstatadv_in_whl_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000800)) >> 11);
}

__INLINE void em_ble_rxstatadv_in_whl_setf(int elt_idx, uint8_t inwhl)
{
    ASSERT_ERR((((uint16_t)inwhl << 11) & ~((uint16_t)0x00000800)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000800)) | ((uint16_t)inwhl << 11));
}

__INLINE uint8_t em_ble_rxstatadv_in_peradval_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000400)) >> 10);
}

__INLINE void em_ble_rxstatadv_in_peradval_setf(int elt_idx, uint8_t inperadval)
{
    ASSERT_ERR((((uint16_t)inperadval << 10) & ~((uint16_t)0x00000400)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000400)) | ((uint16_t)inperadval << 10));
}

__INLINE uint8_t em_ble_rxstatadv_follow_adi_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000200)) >> 9);
}

__INLINE void em_ble_rxstatadv_follow_adi_setf(int elt_idx, uint8_t followadi)
{
    ASSERT_ERR((((uint16_t)followadi << 9) & ~((uint16_t)0x00000200)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000200)) | ((uint16_t)followadi << 9));
}

__INLINE uint8_t em_ble_rxstatadv_adi_match_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000100)) >> 8);
}

__INLINE void em_ble_rxstatadv_adi_match_setf(int elt_idx, uint8_t adimatch)
{
    ASSERT_ERR((((uint16_t)adimatch << 8) & ~((uint16_t)0x00000100)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000100)) | ((uint16_t)adimatch << 8));
}

__INLINE uint8_t em_ble_rxstatadv_peer_add_match_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000080)) >> 7);
}

__INLINE void em_ble_rxstatadv_peer_add_match_setf(int elt_idx, uint8_t peeraddmatch)
{
    ASSERT_ERR((((uint16_t)peeraddmatch << 7) & ~((uint16_t)0x00000080)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000080)) | ((uint16_t)peeraddmatch << 7));
}

__INLINE uint8_t em_ble_rxstatadv_bdaddr_match_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000040)) >> 6);
}

__INLINE void em_ble_rxstatadv_bdaddr_match_setf(int elt_idx, uint8_t bdaddrmatch)
{
    ASSERT_ERR((((uint16_t)bdaddrmatch << 6) & ~((uint16_t)0x00000040)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000040)) | ((uint16_t)bdaddrmatch << 6));
}

__INLINE uint8_t em_ble_rxstatadv_type_err_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000020)) >> 5);
}

__INLINE void em_ble_rxstatadv_type_err_setf(int elt_idx, uint8_t typeerr)
{
    ASSERT_ERR((((uint16_t)typeerr << 5) & ~((uint16_t)0x00000020)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000020)) | ((uint16_t)typeerr << 5));
}

__INLINE uint8_t em_ble_rxstatadv_priv_err_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000010)) >> 4);
}

__INLINE void em_ble_rxstatadv_priv_err_setf(int elt_idx, uint8_t priverr)
{
    ASSERT_ERR((((uint16_t)priverr << 4) & ~((uint16_t)0x00000010)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000010)) | ((uint16_t)priverr << 4));
}

__INLINE uint8_t em_ble_rxstatadv_crc_err_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000008)) >> 3);
}

__INLINE void em_ble_rxstatadv_crc_err_setf(int elt_idx, uint8_t crcerr)
{
    ASSERT_ERR((((uint16_t)crcerr << 3) & ~((uint16_t)0x00000008)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000008)) | ((uint16_t)crcerr << 3));
}

__INLINE uint8_t em_ble_rxstatadv_len_err_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000004)) >> 2);
}

__INLINE void em_ble_rxstatadv_len_err_setf(int elt_idx, uint8_t lenerr)
{
    ASSERT_ERR((((uint16_t)lenerr << 2) & ~((uint16_t)0x00000004)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000004)) | ((uint16_t)lenerr << 2));
}

__INLINE uint8_t em_ble_rxstatadv_rxtime_err_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000002)) >> 1);
}

__INLINE void em_ble_rxstatadv_rxtime_err_setf(int elt_idx, uint8_t rxtimeerr)
{
    ASSERT_ERR((((uint16_t)rxtimeerr << 1) & ~((uint16_t)0x00000002)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000002)) | ((uint16_t)rxtimeerr << 1));
}

__INLINE uint8_t em_ble_rxstatadv_sync_err_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000001)) >> 0);
}

__INLINE void em_ble_rxstatadv_sync_err_setf(int elt_idx, uint8_t syncerr)
{
    ASSERT_ERR((((uint16_t)syncerr << 0) & ~((uint16_t)0x00000001)) == 0);
    EM_BLE_WR(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXSTATADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000001)) | ((uint16_t)syncerr << 0));
}

/**
 * @brief RXPHCE register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:08                RXLEN   0x0
 *  07:05             RXACLRFU   0x0
 *     04                 RXMD   0
 *     03                 RXSN   0
 *     02               RXNESN   0
 *  01:00               RXLLID   0x0
 * </pre>
 */
#define EM_BLE_RXPHCE_ADDR   (0x40110004 + EM_BLE_RX_DESC_OFFSET)
#define EM_BLE_RXPHCE_INDEX  0x00000002
#define EM_BLE_RXPHCE_RESET  0x00000000

__INLINE uint16_t em_ble_rxphce_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
}

__INLINE void em_ble_rxphce_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_RXLEN_MASK      ((uint16_t)0x0000FF00)
#define EM_BLE_RXLEN_LSB       8
#define EM_BLE_RXLEN_WIDTH     ((uint16_t)0x00000008)
#define EM_BLE_RXACLRFU_MASK   ((uint16_t)0x000000E0)
#define EM_BLE_RXACLRFU_LSB    5
#define EM_BLE_RXACLRFU_WIDTH  ((uint16_t)0x00000003)
#define EM_BLE_RXMD_BIT        ((uint16_t)0x00000010)
#define EM_BLE_RXMD_POS        4
#define EM_BLE_RXSN_BIT        ((uint16_t)0x00000008)
#define EM_BLE_RXSN_POS        3
#define EM_BLE_RXNESN_BIT      ((uint16_t)0x00000004)
#define EM_BLE_RXNESN_POS      2
#define EM_BLE_RXLLID_MASK     ((uint16_t)0x00000003)
#define EM_BLE_RXLLID_LSB      0
#define EM_BLE_RXLLID_WIDTH    ((uint16_t)0x00000002)

#define EM_BLE_RXLEN_RST       0x0
#define EM_BLE_RXACLRFU_RST    0x0
#define EM_BLE_RXMD_RST        0x0
#define EM_BLE_RXSN_RST        0x0
#define EM_BLE_RXNESN_RST      0x0
#define EM_BLE_RXLLID_RST      0x0

__INLINE void em_ble_rxphce_pack(int elt_idx, uint8_t rxlen, uint8_t rxaclrfu, uint8_t rxmd, uint8_t rxsn, uint8_t rxnesn, uint8_t rxllid)
{
    ASSERT_ERR((((uint16_t)rxlen << 8) & ~((uint16_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint16_t)rxaclrfu << 5) & ~((uint16_t)0x000000E0)) == 0);
    ASSERT_ERR((((uint16_t)rxmd << 4) & ~((uint16_t)0x00000010)) == 0);
    ASSERT_ERR((((uint16_t)rxsn << 3) & ~((uint16_t)0x00000008)) == 0);
    ASSERT_ERR((((uint16_t)rxnesn << 2) & ~((uint16_t)0x00000004)) == 0);
    ASSERT_ERR((((uint16_t)rxllid << 0) & ~((uint16_t)0x00000003)) == 0);
    EM_BLE_WR(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, ((uint16_t)rxlen << 8) | ((uint16_t)rxaclrfu << 5) | ((uint16_t)rxmd << 4) | ((uint16_t)rxsn << 3) | ((uint16_t)rxnesn << 2) | ((uint16_t)rxllid << 0));
}

__INLINE void em_ble_rxphce_unpack(int elt_idx, uint8_t *rxlen, uint8_t *rxaclrfu, uint8_t *rxmd, uint8_t *rxsn, uint8_t *rxnesn, uint8_t *rxllid)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);

    *rxlen = (localVal & ((uint16_t)0x0000FF00)) >> 8;
    *rxaclrfu = (localVal & ((uint16_t)0x000000E0)) >> 5;
    *rxmd = (localVal & ((uint16_t)0x00000010)) >> 4;
    *rxsn = (localVal & ((uint16_t)0x00000008)) >> 3;
    *rxnesn = (localVal & ((uint16_t)0x00000004)) >> 2;
    *rxllid = (localVal & ((uint16_t)0x00000003)) >> 0;
}

__INLINE uint8_t em_ble_rxphce_rxlen_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x0000FF00)) >> 8);
}

__INLINE void em_ble_rxphce_rxlen_setf(int elt_idx, uint8_t rxlen)
{
    ASSERT_ERR((((uint16_t)rxlen << 8) & ~((uint16_t)0x0000FF00)) == 0);
    EM_BLE_WR(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x0000FF00)) | ((uint16_t)rxlen << 8));
}

__INLINE uint8_t em_ble_rxphce_rxaclrfu_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x000000E0)) >> 5);
}

__INLINE void em_ble_rxphce_rxaclrfu_setf(int elt_idx, uint8_t rxaclrfu)
{
    ASSERT_ERR((((uint16_t)rxaclrfu << 5) & ~((uint16_t)0x000000E0)) == 0);
    EM_BLE_WR(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x000000E0)) | ((uint16_t)rxaclrfu << 5));
}

__INLINE uint8_t em_ble_rxphce_rxmd_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000010)) >> 4);
}

__INLINE void em_ble_rxphce_rxmd_setf(int elt_idx, uint8_t rxmd)
{
    ASSERT_ERR((((uint16_t)rxmd << 4) & ~((uint16_t)0x00000010)) == 0);
    EM_BLE_WR(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000010)) | ((uint16_t)rxmd << 4));
}

__INLINE uint8_t em_ble_rxphce_rxsn_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000008)) >> 3);
}

__INLINE void em_ble_rxphce_rxsn_setf(int elt_idx, uint8_t rxsn)
{
    ASSERT_ERR((((uint16_t)rxsn << 3) & ~((uint16_t)0x00000008)) == 0);
    EM_BLE_WR(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000008)) | ((uint16_t)rxsn << 3));
}

__INLINE uint8_t em_ble_rxphce_rxnesn_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000004)) >> 2);
}

__INLINE void em_ble_rxphce_rxnesn_setf(int elt_idx, uint8_t rxnesn)
{
    ASSERT_ERR((((uint16_t)rxnesn << 2) & ~((uint16_t)0x00000004)) == 0);
    EM_BLE_WR(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000004)) | ((uint16_t)rxnesn << 2));
}

__INLINE uint8_t em_ble_rxphce_rxllid_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000003)) >> 0);
}

__INLINE void em_ble_rxphce_rxllid_setf(int elt_idx, uint8_t rxllid)
{
    ASSERT_ERR((((uint16_t)rxllid << 0) & ~((uint16_t)0x00000003)) == 0);
    EM_BLE_WR(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXPHCE_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000003)) | ((uint16_t)rxllid << 0));
}

/**
 * @brief RXPHADV register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:08             RXADVLEN   0x0
 *     07              RXRXADD   0
 *     06              RXTXADD   0
 *     05             RXCHSEL2   0
 *     04             RXADVRFU   0
 *  03:00               RXTYPE   0x0
 * </pre>
 */
#define EM_BLE_RXPHADV_ADDR   (0x40110004 + EM_BLE_RX_DESC_OFFSET)
#define EM_BLE_RXPHADV_INDEX  0x00000002
#define EM_BLE_RXPHADV_RESET  0x00000000

__INLINE uint16_t em_ble_rxphadv_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
}

__INLINE void em_ble_rxphadv_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_RXADVLEN_MASK   ((uint16_t)0x0000FF00)
#define EM_BLE_RXADVLEN_LSB    8
#define EM_BLE_RXADVLEN_WIDTH  ((uint16_t)0x00000008)
#define EM_BLE_RXRXADD_BIT     ((uint16_t)0x00000080)
#define EM_BLE_RXRXADD_POS     7
#define EM_BLE_RXTXADD_BIT     ((uint16_t)0x00000040)
#define EM_BLE_RXTXADD_POS     6
#define EM_BLE_RXCHSEL2_BIT    ((uint16_t)0x00000020)
#define EM_BLE_RXCHSEL2_POS    5
#define EM_BLE_RXADVRFU_BIT    ((uint16_t)0x00000010)
#define EM_BLE_RXADVRFU_POS    4
#define EM_BLE_RXTYPE_MASK     ((uint16_t)0x0000000F)
#define EM_BLE_RXTYPE_LSB      0
#define EM_BLE_RXTYPE_WIDTH    ((uint16_t)0x00000004)

#define EM_BLE_RXADVLEN_RST    0x0
#define EM_BLE_RXRXADD_RST     0x0
#define EM_BLE_RXTXADD_RST     0x0
#define EM_BLE_RXCHSEL2_RST    0x0
#define EM_BLE_RXADVRFU_RST    0x0
#define EM_BLE_RXTYPE_RST      0x0

__INLINE void em_ble_rxphadv_pack(int elt_idx, uint8_t rxadvlen, uint8_t rxrxadd, uint8_t rxtxadd, uint8_t rxchsel2, uint8_t rxadvrfu, uint8_t rxtype)
{
    ASSERT_ERR((((uint16_t)rxadvlen << 8) & ~((uint16_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint16_t)rxrxadd << 7) & ~((uint16_t)0x00000080)) == 0);
    ASSERT_ERR((((uint16_t)rxtxadd << 6) & ~((uint16_t)0x00000040)) == 0);
    ASSERT_ERR((((uint16_t)rxchsel2 << 5) & ~((uint16_t)0x00000020)) == 0);
    ASSERT_ERR((((uint16_t)rxadvrfu << 4) & ~((uint16_t)0x00000010)) == 0);
    ASSERT_ERR((((uint16_t)rxtype << 0) & ~((uint16_t)0x0000000F)) == 0);
    EM_BLE_WR(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, ((uint16_t)rxadvlen << 8) | ((uint16_t)rxrxadd << 7) | ((uint16_t)rxtxadd << 6) | ((uint16_t)rxchsel2 << 5) | ((uint16_t)rxadvrfu << 4) | ((uint16_t)rxtype << 0));
}

__INLINE void em_ble_rxphadv_unpack(int elt_idx, uint8_t *rxadvlen, uint8_t *rxrxadd, uint8_t *rxtxadd, uint8_t *rxchsel2, uint8_t *rxadvrfu, uint8_t *rxtype)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);

    *rxadvlen = (localVal & ((uint16_t)0x0000FF00)) >> 8;
    *rxrxadd = (localVal & ((uint16_t)0x00000080)) >> 7;
    *rxtxadd = (localVal & ((uint16_t)0x00000040)) >> 6;
    *rxchsel2 = (localVal & ((uint16_t)0x00000020)) >> 5;
    *rxadvrfu = (localVal & ((uint16_t)0x00000010)) >> 4;
    *rxtype = (localVal & ((uint16_t)0x0000000F)) >> 0;
}

__INLINE uint8_t em_ble_rxphadv_rxadvlen_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x0000FF00)) >> 8);
}

__INLINE void em_ble_rxphadv_rxadvlen_setf(int elt_idx, uint8_t rxadvlen)
{
    ASSERT_ERR((((uint16_t)rxadvlen << 8) & ~((uint16_t)0x0000FF00)) == 0);
    EM_BLE_WR(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x0000FF00)) | ((uint16_t)rxadvlen << 8));
}

__INLINE uint8_t em_ble_rxphadv_rxrxadd_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000080)) >> 7);
}

__INLINE void em_ble_rxphadv_rxrxadd_setf(int elt_idx, uint8_t rxrxadd)
{
    ASSERT_ERR((((uint16_t)rxrxadd << 7) & ~((uint16_t)0x00000080)) == 0);
    EM_BLE_WR(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000080)) | ((uint16_t)rxrxadd << 7));
}

__INLINE uint8_t em_ble_rxphadv_rxtxadd_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000040)) >> 6);
}

__INLINE void em_ble_rxphadv_rxtxadd_setf(int elt_idx, uint8_t rxtxadd)
{
    ASSERT_ERR((((uint16_t)rxtxadd << 6) & ~((uint16_t)0x00000040)) == 0);
    EM_BLE_WR(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000040)) | ((uint16_t)rxtxadd << 6));
}

__INLINE uint8_t em_ble_rxphadv_rxchsel2_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000020)) >> 5);
}

__INLINE void em_ble_rxphadv_rxchsel2_setf(int elt_idx, uint8_t rxchsel2)
{
    ASSERT_ERR((((uint16_t)rxchsel2 << 5) & ~((uint16_t)0x00000020)) == 0);
    EM_BLE_WR(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000020)) | ((uint16_t)rxchsel2 << 5));
}

__INLINE uint8_t em_ble_rxphadv_rxadvrfu_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000010)) >> 4);
}

__INLINE void em_ble_rxphadv_rxadvrfu_setf(int elt_idx, uint8_t rxadvrfu)
{
    ASSERT_ERR((((uint16_t)rxadvrfu << 4) & ~((uint16_t)0x00000010)) == 0);
    EM_BLE_WR(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000010)) | ((uint16_t)rxadvrfu << 4));
}

__INLINE uint8_t em_ble_rxphadv_rxtype_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x0000000F)) >> 0);
}

__INLINE void em_ble_rxphadv_rxtype_setf(int elt_idx, uint8_t rxtype)
{
    ASSERT_ERR((((uint16_t)rxtype << 0) & ~((uint16_t)0x0000000F)) == 0);
    EM_BLE_WR(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXPHADV_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x0000000F)) | ((uint16_t)rxtype << 0));
}

/**
 * @brief RXCHASS register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:14                 RATE   0x0
 *  13:08          USED_CH_IDX   0x0
 *  07:00                 RSSI   0x0
 * </pre>
 */
#define EM_BLE_RXCHASS_ADDR   (0x40110006 + EM_BLE_RX_DESC_OFFSET)
#define EM_BLE_RXCHASS_INDEX  0x00000003
#define EM_BLE_RXCHASS_RESET  0x00000000

__INLINE uint16_t em_ble_rxchass_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXCHASS_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
}

__INLINE void em_ble_rxchass_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXCHASS_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_RATE_MASK          ((uint16_t)0x0000C000)
#define EM_BLE_RATE_LSB           14
#define EM_BLE_RATE_WIDTH         ((uint16_t)0x00000002)
#define EM_BLE_USED_CH_IDX_MASK   ((uint16_t)0x00003F00)
#define EM_BLE_USED_CH_IDX_LSB    8
#define EM_BLE_USED_CH_IDX_WIDTH  ((uint16_t)0x00000006)
#define EM_BLE_RSSI_MASK          ((uint16_t)0x000000FF)
#define EM_BLE_RSSI_LSB           0
#define EM_BLE_RSSI_WIDTH         ((uint16_t)0x00000008)

#define EM_BLE_RATE_RST           0x0
#define EM_BLE_USED_CH_IDX_RST    0x0
#define EM_BLE_RSSI_RST           0x0

__INLINE void em_ble_rxchass_pack(int elt_idx, uint8_t rate, uint8_t usedchidx, uint8_t rssi)
{
    ASSERT_ERR((((uint16_t)rate << 14) & ~((uint16_t)0x0000C000)) == 0);
    ASSERT_ERR((((uint16_t)usedchidx << 8) & ~((uint16_t)0x00003F00)) == 0);
    ASSERT_ERR((((uint16_t)rssi << 0) & ~((uint16_t)0x000000FF)) == 0);
    EM_BLE_WR(EM_BLE_RXCHASS_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, ((uint16_t)rate << 14) | ((uint16_t)usedchidx << 8) | ((uint16_t)rssi << 0));
}

__INLINE void em_ble_rxchass_unpack(int elt_idx, uint8_t *rate, uint8_t *usedchidx, uint8_t *rssi)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXCHASS_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);

    *rate = (localVal & ((uint16_t)0x0000C000)) >> 14;
    *usedchidx = (localVal & ((uint16_t)0x00003F00)) >> 8;
    *rssi = (localVal & ((uint16_t)0x000000FF)) >> 0;
}

__INLINE uint8_t em_ble_rxchass_rate_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXCHASS_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x0000C000)) >> 14);
}

__INLINE void em_ble_rxchass_rate_setf(int elt_idx, uint8_t rate)
{
    ASSERT_ERR((((uint16_t)rate << 14) & ~((uint16_t)0x0000C000)) == 0);
    EM_BLE_WR(EM_BLE_RXCHASS_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXCHASS_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x0000C000)) | ((uint16_t)rate << 14));
}

__INLINE uint8_t em_ble_rxchass_used_ch_idx_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXCHASS_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00003F00)) >> 8);
}

__INLINE void em_ble_rxchass_used_ch_idx_setf(int elt_idx, uint8_t usedchidx)
{
    ASSERT_ERR((((uint16_t)usedchidx << 8) & ~((uint16_t)0x00003F00)) == 0);
    EM_BLE_WR(EM_BLE_RXCHASS_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXCHASS_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00003F00)) | ((uint16_t)usedchidx << 8));
}

__INLINE uint8_t em_ble_rxchass_rssi_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXCHASS_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x000000FF)) >> 0);
}

__INLINE void em_ble_rxchass_rssi_setf(int elt_idx, uint8_t rssi)
{
    ASSERT_ERR((((uint16_t)rssi << 0) & ~((uint16_t)0x000000FF)) == 0);
    EM_BLE_WR(EM_BLE_RXCHASS_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXCHASS_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x000000FF)) | ((uint16_t)rssi << 0));
}

/**
 * @brief RXCLKNSYNC0 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00          CLKNRXSYNC0   0x0
 * </pre>
 */
#define EM_BLE_RXCLKNSYNC0_ADDR   (0x40110008 + EM_BLE_RX_DESC_OFFSET)
#define EM_BLE_RXCLKNSYNC0_INDEX  0x00000004
#define EM_BLE_RXCLKNSYNC0_RESET  0x00000000

__INLINE uint16_t em_ble_rxclknsync0_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXCLKNSYNC0_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
}

// field definitions
#define EM_BLE_CLKNRXSYNC0_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_CLKNRXSYNC0_LSB    0
#define EM_BLE_CLKNRXSYNC0_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_CLKNRXSYNC0_RST    0x0

__INLINE uint16_t em_ble_rxclknsync0_clknrxsync0_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXCLKNSYNC0_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief RXCLKNSYNC1 register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  11:00          CLKNRXSYNC1   0x0
 * </pre>
 */
#define EM_BLE_RXCLKNSYNC1_ADDR   (0x4011000A + EM_BLE_RX_DESC_OFFSET)
#define EM_BLE_RXCLKNSYNC1_INDEX  0x00000005
#define EM_BLE_RXCLKNSYNC1_RESET  0x00000000

__INLINE uint16_t em_ble_rxclknsync1_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXCLKNSYNC1_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
}

// field definitions
#define EM_BLE_CLKNRXSYNC1_MASK   ((uint16_t)0x00000FFF)
#define EM_BLE_CLKNRXSYNC1_LSB    0
#define EM_BLE_CLKNRXSYNC1_WIDTH  ((uint16_t)0x0000000C)

#define EM_BLE_CLKNRXSYNC1_RST    0x0

__INLINE uint16_t em_ble_rxclknsync1_clknrxsync1_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXCLKNSYNC1_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x00000FFF)) == 0);
    return (localVal >> 0);
}

/**
 * @brief RXFCNTSYNC register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:11            RXLINKLBL   0x0
 *  09:00           FCNTRXSYNC   0x0
 * </pre>
 */
#define EM_BLE_RXFCNTSYNC_ADDR   (0x4011000C + EM_BLE_RX_DESC_OFFSET)
#define EM_BLE_RXFCNTSYNC_INDEX  0x00000006
#define EM_BLE_RXFCNTSYNC_RESET  0x00000000

__INLINE uint16_t em_ble_rxfcntsync_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXFCNTSYNC_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
}

// field definitions
#define EM_BLE_RXLINKLBL_MASK    ((uint16_t)0x0000F800)
#define EM_BLE_RXLINKLBL_LSB     11
#define EM_BLE_RXLINKLBL_WIDTH   ((uint16_t)0x00000005)
#define EM_BLE_FCNTRXSYNC_MASK   ((uint16_t)0x000003FF)
#define EM_BLE_FCNTRXSYNC_LSB    0
#define EM_BLE_FCNTRXSYNC_WIDTH  ((uint16_t)0x0000000A)

#define EM_BLE_RXLINKLBL_RST     0x0
#define EM_BLE_FCNTRXSYNC_RST    0x0

__INLINE void em_ble_rxfcntsync_unpack(int elt_idx, uint8_t *rxlinklbl, uint16_t *fcntrxsync)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXFCNTSYNC_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);

    *rxlinklbl = (localVal & ((uint16_t)0x0000F800)) >> 11;
    *fcntrxsync = (localVal & ((uint16_t)0x000003FF)) >> 0;
}

__INLINE uint8_t em_ble_rxfcntsync_rxlinklbl_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXFCNTSYNC_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x0000F800)) >> 11);
}

__INLINE uint16_t em_ble_rxfcntsync_fcntrxsync_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXFCNTSYNC_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x000003FF)) >> 0);
}

/**
 * @brief RXRALPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00             RXRALPTR   0x0
 * </pre>
 */
#define EM_BLE_RXRALPTR_ADDR   (0x4011000E + EM_BLE_RX_DESC_OFFSET)
#define EM_BLE_RXRALPTR_INDEX  0x00000007
#define EM_BLE_RXRALPTR_RESET  0x00000000

__INLINE uint16_t em_ble_rxralptr_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXRALPTR_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
}

__INLINE void em_ble_rxralptr_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXRALPTR_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_RXRALPTR_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_RXRALPTR_LSB    0
#define EM_BLE_RXRALPTR_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_RXRALPTR_RST    0x0

__INLINE uint16_t em_ble_rxralptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXRALPTR_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_rxralptr_setf(int elt_idx, uint16_t rxralptr)
{
    ASSERT_ERR((((uint16_t)rxralptr << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_RXRALPTR_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (uint16_t)rxralptr << 0);
}

/**
 * @brief RXAEHEADER register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15               RXRSVD   0
 *     14                RXPOW   0
 *     13               RXSYNC   0
 *     12             RXAUXPTR   0
 *     11                RXADI   0
 *     10               RXSUPP   0
 *     09               RXTGTA   0
 *     08               RXADVA   0
 *  07:06             RXAEMODE   0x0
 *  05:00           RXAELENGTH   0x0
 * </pre>
 */
#define EM_BLE_RXAEHEADER_ADDR   (0x40110010 + EM_BLE_RX_DESC_OFFSET)
#define EM_BLE_RXAEHEADER_INDEX  0x00000008
#define EM_BLE_RXAEHEADER_RESET  0x00000000

__INLINE uint16_t em_ble_rxaeheader_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
}

__INLINE void em_ble_rxaeheader_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_RXRSVD_BIT        ((uint16_t)0x00008000)
#define EM_BLE_RXRSVD_POS        15
#define EM_BLE_RXPOW_BIT         ((uint16_t)0x00004000)
#define EM_BLE_RXPOW_POS         14
#define EM_BLE_RXSYNC_BIT        ((uint16_t)0x00002000)
#define EM_BLE_RXSYNC_POS        13
#define EM_BLE_RXAUXPTR_BIT      ((uint16_t)0x00001000)
#define EM_BLE_RXAUXPTR_POS      12
#define EM_BLE_RXADI_BIT         ((uint16_t)0x00000800)
#define EM_BLE_RXADI_POS         11
#define EM_BLE_RXSUPP_BIT        ((uint16_t)0x00000400)
#define EM_BLE_RXSUPP_POS        10
#define EM_BLE_RXTGTA_BIT        ((uint16_t)0x00000200)
#define EM_BLE_RXTGTA_POS        9
#define EM_BLE_RXADVA_BIT        ((uint16_t)0x00000100)
#define EM_BLE_RXADVA_POS        8
#define EM_BLE_RXAEMODE_MASK     ((uint16_t)0x000000C0)
#define EM_BLE_RXAEMODE_LSB      6
#define EM_BLE_RXAEMODE_WIDTH    ((uint16_t)0x00000002)
#define EM_BLE_RXAELENGTH_MASK   ((uint16_t)0x0000003F)
#define EM_BLE_RXAELENGTH_LSB    0
#define EM_BLE_RXAELENGTH_WIDTH  ((uint16_t)0x00000006)

#define EM_BLE_RXRSVD_RST        0x0
#define EM_BLE_RXPOW_RST         0x0
#define EM_BLE_RXSYNC_RST        0x0
#define EM_BLE_RXAUXPTR_RST      0x0
#define EM_BLE_RXADI_RST         0x0
#define EM_BLE_RXSUPP_RST        0x0
#define EM_BLE_RXTGTA_RST        0x0
#define EM_BLE_RXADVA_RST        0x0
#define EM_BLE_RXAEMODE_RST      0x0
#define EM_BLE_RXAELENGTH_RST    0x0

__INLINE void em_ble_rxaeheader_pack(int elt_idx, uint8_t rxrsvd, uint8_t rxpow, uint8_t rxsync, uint8_t rxauxptr, uint8_t rxadi, uint8_t rxsupp, uint8_t rxtgta, uint8_t rxadva, uint8_t rxaemode, uint8_t rxaelength)
{
    ASSERT_ERR((((uint16_t)rxrsvd << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)rxpow << 14) & ~((uint16_t)0x00004000)) == 0);
    ASSERT_ERR((((uint16_t)rxsync << 13) & ~((uint16_t)0x00002000)) == 0);
    ASSERT_ERR((((uint16_t)rxauxptr << 12) & ~((uint16_t)0x00001000)) == 0);
    ASSERT_ERR((((uint16_t)rxadi << 11) & ~((uint16_t)0x00000800)) == 0);
    ASSERT_ERR((((uint16_t)rxsupp << 10) & ~((uint16_t)0x00000400)) == 0);
    ASSERT_ERR((((uint16_t)rxtgta << 9) & ~((uint16_t)0x00000200)) == 0);
    ASSERT_ERR((((uint16_t)rxadva << 8) & ~((uint16_t)0x00000100)) == 0);
    ASSERT_ERR((((uint16_t)rxaemode << 6) & ~((uint16_t)0x000000C0)) == 0);
    ASSERT_ERR((((uint16_t)rxaelength << 0) & ~((uint16_t)0x0000003F)) == 0);
    EM_BLE_WR(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, ((uint16_t)rxrsvd << 15) | ((uint16_t)rxpow << 14) | ((uint16_t)rxsync << 13) | ((uint16_t)rxauxptr << 12) | ((uint16_t)rxadi << 11) | ((uint16_t)rxsupp << 10) | ((uint16_t)rxtgta << 9) | ((uint16_t)rxadva << 8) | ((uint16_t)rxaemode << 6) | ((uint16_t)rxaelength << 0));
}

__INLINE void em_ble_rxaeheader_unpack(int elt_idx, uint8_t *rxrsvd, uint8_t *rxpow, uint8_t *rxsync, uint8_t *rxauxptr, uint8_t *rxadi, uint8_t *rxsupp, uint8_t *rxtgta, uint8_t *rxadva, uint8_t *rxaemode, uint8_t *rxaelength)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);

    *rxrsvd = (localVal & ((uint16_t)0x00008000)) >> 15;
    *rxpow = (localVal & ((uint16_t)0x00004000)) >> 14;
    *rxsync = (localVal & ((uint16_t)0x00002000)) >> 13;
    *rxauxptr = (localVal & ((uint16_t)0x00001000)) >> 12;
    *rxadi = (localVal & ((uint16_t)0x00000800)) >> 11;
    *rxsupp = (localVal & ((uint16_t)0x00000400)) >> 10;
    *rxtgta = (localVal & ((uint16_t)0x00000200)) >> 9;
    *rxadva = (localVal & ((uint16_t)0x00000100)) >> 8;
    *rxaemode = (localVal & ((uint16_t)0x000000C0)) >> 6;
    *rxaelength = (localVal & ((uint16_t)0x0000003F)) >> 0;
}

__INLINE uint8_t em_ble_rxaeheader_rxrsvd_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_ble_rxaeheader_rxrsvd_setf(int elt_idx, uint8_t rxrsvd)
{
    ASSERT_ERR((((uint16_t)rxrsvd << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BLE_WR(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)rxrsvd << 15));
}

__INLINE uint8_t em_ble_rxaeheader_rxpow_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00004000)) >> 14);
}

__INLINE void em_ble_rxaeheader_rxpow_setf(int elt_idx, uint8_t rxpow)
{
    ASSERT_ERR((((uint16_t)rxpow << 14) & ~((uint16_t)0x00004000)) == 0);
    EM_BLE_WR(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00004000)) | ((uint16_t)rxpow << 14));
}

__INLINE uint8_t em_ble_rxaeheader_rxsync_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00002000)) >> 13);
}

__INLINE void em_ble_rxaeheader_rxsync_setf(int elt_idx, uint8_t rxsync)
{
    ASSERT_ERR((((uint16_t)rxsync << 13) & ~((uint16_t)0x00002000)) == 0);
    EM_BLE_WR(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00002000)) | ((uint16_t)rxsync << 13));
}

__INLINE uint8_t em_ble_rxaeheader_rxauxptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00001000)) >> 12);
}

__INLINE void em_ble_rxaeheader_rxauxptr_setf(int elt_idx, uint8_t rxauxptr)
{
    ASSERT_ERR((((uint16_t)rxauxptr << 12) & ~((uint16_t)0x00001000)) == 0);
    EM_BLE_WR(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00001000)) | ((uint16_t)rxauxptr << 12));
}

__INLINE uint8_t em_ble_rxaeheader_rxadi_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000800)) >> 11);
}

__INLINE void em_ble_rxaeheader_rxadi_setf(int elt_idx, uint8_t rxadi)
{
    ASSERT_ERR((((uint16_t)rxadi << 11) & ~((uint16_t)0x00000800)) == 0);
    EM_BLE_WR(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000800)) | ((uint16_t)rxadi << 11));
}

__INLINE uint8_t em_ble_rxaeheader_rxsupp_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000400)) >> 10);
}

__INLINE void em_ble_rxaeheader_rxsupp_setf(int elt_idx, uint8_t rxsupp)
{
    ASSERT_ERR((((uint16_t)rxsupp << 10) & ~((uint16_t)0x00000400)) == 0);
    EM_BLE_WR(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000400)) | ((uint16_t)rxsupp << 10));
}

__INLINE uint8_t em_ble_rxaeheader_rxtgta_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000200)) >> 9);
}

__INLINE void em_ble_rxaeheader_rxtgta_setf(int elt_idx, uint8_t rxtgta)
{
    ASSERT_ERR((((uint16_t)rxtgta << 9) & ~((uint16_t)0x00000200)) == 0);
    EM_BLE_WR(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000200)) | ((uint16_t)rxtgta << 9));
}

__INLINE uint8_t em_ble_rxaeheader_rxadva_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00000100)) >> 8);
}

__INLINE void em_ble_rxaeheader_rxadva_setf(int elt_idx, uint8_t rxadva)
{
    ASSERT_ERR((((uint16_t)rxadva << 8) & ~((uint16_t)0x00000100)) == 0);
    EM_BLE_WR(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x00000100)) | ((uint16_t)rxadva << 8));
}

__INLINE uint8_t em_ble_rxaeheader_rxaemode_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x000000C0)) >> 6);
}

__INLINE void em_ble_rxaeheader_rxaemode_setf(int elt_idx, uint8_t rxaemode)
{
    ASSERT_ERR((((uint16_t)rxaemode << 6) & ~((uint16_t)0x000000C0)) == 0);
    EM_BLE_WR(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x000000C0)) | ((uint16_t)rxaemode << 6));
}

__INLINE uint8_t em_ble_rxaeheader_rxaelength_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    return ((localVal & ((uint16_t)0x0000003F)) >> 0);
}

__INLINE void em_ble_rxaeheader_rxaelength_setf(int elt_idx, uint8_t rxaelength)
{
    ASSERT_ERR((((uint16_t)rxaelength << 0) & ~((uint16_t)0x0000003F)) == 0);
    EM_BLE_WR(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXAEHEADER_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE) & ~((uint16_t)0x0000003F)) | ((uint16_t)rxaelength << 0));
}

/**
 * @brief RXDATAPTR register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:00            RXDATAPTR   0x0
 * </pre>
 */
#define EM_BLE_RXDATAPTR_ADDR   (0x40110012 + EM_BLE_RX_DESC_OFFSET)
#define EM_BLE_RXDATAPTR_INDEX  0x00000009
#define EM_BLE_RXDATAPTR_RESET  0x00000000

__INLINE uint16_t em_ble_rxdataptr_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXDATAPTR_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
}

__INLINE void em_ble_rxdataptr_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXDATAPTR_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_RXDATAPTR_MASK   ((uint16_t)0x0000FFFF)
#define EM_BLE_RXDATAPTR_LSB    0
#define EM_BLE_RXDATAPTR_WIDTH  ((uint16_t)0x00000010)

#define EM_BLE_RXDATAPTR_RST    0x0

__INLINE uint16_t em_ble_rxdataptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXDATAPTR_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE);
    ASSERT_ERR((localVal & ~((uint16_t)0x0000FFFF)) == 0);
    return (localVal >> 0);
}

__INLINE void em_ble_rxdataptr_setf(int elt_idx, uint16_t rxdataptr)
{
    ASSERT_ERR((((uint16_t)rxdataptr << 0) & ~((uint16_t)0x0000FFFF)) == 0);
    EM_BLE_WR(EM_BLE_RXDATAPTR_ADDR + elt_idx * REG_EM_BLE_RX_DESC_SIZE, (uint16_t)rxdataptr << 0);
}


#endif // _REG_EM_BLE_RX_DESC_H_

