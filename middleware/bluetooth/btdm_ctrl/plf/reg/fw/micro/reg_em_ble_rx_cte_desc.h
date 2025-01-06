#ifndef _REG_EM_BLE_RX_CTE_DESC_H_
#define _REG_EM_BLE_RX_CTE_DESC_H_

#include <stdint.h>
#include "_reg_em_ble_rx_cte_desc.h"
#include "compiler.h"
#include "arch.h"
#include "em_map.h"
#include "reg_access.h"

#define REG_EM_BLE_RX_CTE_DESC_COUNT 3

#define REG_EM_BLE_RX_CTE_DESC_DECODING_MASK 0x00000007

#define REG_EM_BLE_RX_CTE_DESC_ADDR_GET(idx) (EM_BLE_RX_CTE_DESC_OFFSET + (idx) * REG_EM_BLE_RX_CTE_DESC_SIZE)

/**
 * @brief RXCTECNTL register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *     15               RXDONE   0
 *  13:00            RXNEXTPTR   0x0
 * </pre>
 */
#define EM_BLE_RXCTECNTL_ADDR   (0x20408000 + EM_BLE_RX_CTE_DESC_OFFSET)
#define EM_BLE_RXCTECNTL_INDEX  0x00000000
#define EM_BLE_RXCTECNTL_RESET  0x00000000

__INLINE uint16_t em_ble_rxctecntl_get(int elt_idx)
{
    return EM_BLE_RD(EM_BLE_RXCTECNTL_ADDR + elt_idx * REG_EM_BLE_RX_CTE_DESC_SIZE);
}

__INLINE void em_ble_rxctecntl_set(int elt_idx, uint16_t value)
{
    EM_BLE_WR(EM_BLE_RXCTECNTL_ADDR + elt_idx * REG_EM_BLE_RX_CTE_DESC_SIZE, value);
}

// field definitions
#define EM_BLE_RXDONE_BIT       ((uint16_t)0x00008000)
#define EM_BLE_RXDONE_POS       15
#define EM_BLE_RXNEXTPTR_MASK   ((uint16_t)0x00003FFF)
#define EM_BLE_RXNEXTPTR_LSB    0
#define EM_BLE_RXNEXTPTR_WIDTH  ((uint16_t)0x0000000E)

#define EM_BLE_RXDONE_RST       0x0
#define EM_BLE_RXNEXTPTR_RST    0x0

__INLINE void em_ble_rxctecntl_pack(int elt_idx, uint8_t rxdone, uint16_t rxnextptr)
{
    ASSERT_ERR((((uint16_t)rxdone << 15) & ~((uint16_t)0x00008000)) == 0);
    ASSERT_ERR((((uint16_t)rxnextptr << 0) & ~((uint16_t)0x00003FFF)) == 0);
    EM_BLE_WR(EM_BLE_RXCTECNTL_ADDR + elt_idx * REG_EM_BLE_RX_CTE_DESC_SIZE, ((uint16_t)rxdone << 15) | ((uint16_t)rxnextptr << 0));
}

__INLINE void em_ble_rxctecntl_unpack(int elt_idx, uint8_t *rxdone, uint16_t *rxnextptr)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXCTECNTL_ADDR + elt_idx * REG_EM_BLE_RX_CTE_DESC_SIZE);

    *rxdone = (localVal & ((uint16_t)0x00008000)) >> 15;
    *rxnextptr = (localVal & ((uint16_t)0x00003FFF)) >> 0;
}

__INLINE uint8_t em_ble_rxctecntl_rxdone_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXCTECNTL_ADDR + elt_idx * REG_EM_BLE_RX_CTE_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00008000)) >> 15);
}

__INLINE void em_ble_rxctecntl_rxdone_setf(int elt_idx, uint8_t rxdone)
{
    ASSERT_ERR((((uint16_t)rxdone << 15) & ~((uint16_t)0x00008000)) == 0);
    EM_BLE_WR(EM_BLE_RXCTECNTL_ADDR + elt_idx * REG_EM_BLE_RX_CTE_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXCTECNTL_ADDR + elt_idx * REG_EM_BLE_RX_CTE_DESC_SIZE) & ~((uint16_t)0x00008000)) | ((uint16_t)rxdone << 15));
}

__INLINE uint16_t em_ble_rxctecntl_rxnextptr_getf(int elt_idx)
{
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXCTECNTL_ADDR + elt_idx * REG_EM_BLE_RX_CTE_DESC_SIZE);
    return ((localVal & ((uint16_t)0x00003FFF)) >> 0);
}

__INLINE void em_ble_rxctecntl_rxnextptr_setf(int elt_idx, uint16_t rxnextptr)
{
    ASSERT_ERR((((uint16_t)rxnextptr << 0) & ~((uint16_t)0x00003FFF)) == 0);
    EM_BLE_WR(EM_BLE_RXCTECNTL_ADDR + elt_idx * REG_EM_BLE_RX_CTE_DESC_SIZE, (EM_BLE_RD(EM_BLE_RXCTECNTL_ADDR + elt_idx * REG_EM_BLE_RX_CTE_DESC_SIZE) & ~((uint16_t)0x00003FFF)) | ((uint16_t)rxnextptr << 0));
}

/**
 * @brief RXCTESAMPBUF register definition
 * <pre>
 *   Bits           Field Name   Reset Value
 *  -----   ------------------   -----------
 *  15:08                 RX_Q   0x0
 *  07:00                 RX_I   0x0
 * </pre>
 */
#define EM_BLE_RXCTESAMPBUF_ADDR   (0x20408004 + EM_BLE_RX_CTE_DESC_OFFSET)
#define EM_BLE_RXCTESAMPBUF_INDEX  0x00000002
#define EM_BLE_RXCTESAMPBUF_RESET  0x00000000
#define EM_BLE_RXCTESAMPBUF_COUNT  82

__INLINE uint16_t em_ble_rxctesampbuf_get(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 81);
    return EM_BLE_RD(EM_BLE_RXCTESAMPBUF_ADDR + elt_idx * REG_EM_BLE_RX_CTE_DESC_SIZE + reg_idx * 2);
}

__INLINE void em_ble_rxctesampbuf_set(int elt_idx, int reg_idx, uint16_t value)
{
    ASSERT_ERR(reg_idx <= 81);
    EM_BLE_WR(EM_BLE_RXCTESAMPBUF_ADDR + elt_idx * REG_EM_BLE_RX_CTE_DESC_SIZE + reg_idx * 2, value);
}

// field definitions
#define EM_BLE_RX_Q_MASK   ((uint16_t)0x0000FF00)
#define EM_BLE_RX_Q_LSB    8
#define EM_BLE_RX_Q_WIDTH  ((uint16_t)0x00000008)
#define EM_BLE_RX_I_MASK   ((uint16_t)0x000000FF)
#define EM_BLE_RX_I_LSB    0
#define EM_BLE_RX_I_WIDTH  ((uint16_t)0x00000008)

#define EM_BLE_RX_Q_RST    0x0
#define EM_BLE_RX_I_RST    0x0

__INLINE void em_ble_rxctesampbuf_pack(int elt_idx, int reg_idx, uint8_t rxq, uint8_t rxi)
{
    ASSERT_ERR(reg_idx <= 81);
    ASSERT_ERR((((uint16_t)rxq << 8) & ~((uint16_t)0x0000FF00)) == 0);
    ASSERT_ERR((((uint16_t)rxi << 0) & ~((uint16_t)0x000000FF)) == 0);
    EM_BLE_WR(EM_BLE_RXCTESAMPBUF_ADDR + elt_idx * REG_EM_BLE_RX_CTE_DESC_SIZE + reg_idx * 2, ((uint16_t)rxq << 8) | ((uint16_t)rxi << 0));
}

__INLINE void em_ble_rxctesampbuf_unpack(int elt_idx, int reg_idx, uint8_t *rxq, uint8_t *rxi)
{
    ASSERT_ERR(reg_idx <= 81);
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXCTESAMPBUF_ADDR + elt_idx * REG_EM_BLE_RX_CTE_DESC_SIZE + reg_idx * 2);

    *rxq = (localVal & ((uint16_t)0x0000FF00)) >> 8;
    *rxi = (localVal & ((uint16_t)0x000000FF)) >> 0;
}

__INLINE uint8_t em_ble_rxctesampbuf_rx_q_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 81);
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXCTESAMPBUF_ADDR + elt_idx * REG_EM_BLE_RX_CTE_DESC_SIZE + reg_idx * 2);
    return ((localVal & ((uint16_t)0x0000FF00)) >> 8);
}

__INLINE void em_ble_rxctesampbuf_rx_q_setf(int elt_idx, int reg_idx, uint8_t rxq)
{
    ASSERT_ERR(reg_idx <= 81);
    ASSERT_ERR((((uint16_t)rxq << 8) & ~((uint16_t)0x0000FF00)) == 0);
    EM_BLE_WR(EM_BLE_RXCTESAMPBUF_ADDR + elt_idx * REG_EM_BLE_RX_CTE_DESC_SIZE + reg_idx * 2, (EM_BLE_RD(EM_BLE_RXCTESAMPBUF_ADDR + elt_idx * REG_EM_BLE_RX_CTE_DESC_SIZE + reg_idx * 2) & ~((uint16_t)0x0000FF00)) | ((uint16_t)rxq << 8));
}

__INLINE uint8_t em_ble_rxctesampbuf_rx_i_getf(int elt_idx, int reg_idx)
{
    ASSERT_ERR(reg_idx <= 81);
    uint16_t localVal = EM_BLE_RD(EM_BLE_RXCTESAMPBUF_ADDR + elt_idx * REG_EM_BLE_RX_CTE_DESC_SIZE + reg_idx * 2);
    return ((localVal & ((uint16_t)0x000000FF)) >> 0);
}

__INLINE void em_ble_rxctesampbuf_rx_i_setf(int elt_idx, int reg_idx, uint8_t rxi)
{
    ASSERT_ERR(reg_idx <= 81);
    ASSERT_ERR((((uint16_t)rxi << 0) & ~((uint16_t)0x000000FF)) == 0);
    EM_BLE_WR(EM_BLE_RXCTESAMPBUF_ADDR + elt_idx * REG_EM_BLE_RX_CTE_DESC_SIZE + reg_idx * 2, (EM_BLE_RD(EM_BLE_RXCTESAMPBUF_ADDR + elt_idx * REG_EM_BLE_RX_CTE_DESC_SIZE + reg_idx * 2) & ~((uint16_t)0x000000FF)) | ((uint16_t)rxi << 0));
}


#endif // _REG_EM_BLE_RX_CTE_DESC_H_

