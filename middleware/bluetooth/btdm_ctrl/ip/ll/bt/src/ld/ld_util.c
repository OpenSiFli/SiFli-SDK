/**
****************************************************************************************
*
* @file ld_util.c
*
* @brief LD utilities source code
*
* Copyright (C) RivieraWaves 2009-2015
*
****************************************************************************************
*/

/**
 ****************************************************************************************
 * @addtogroup LD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "ld_util.h"            // link driver utilities
#include "ld_int.h"             // link driver internal data
#include "reg_access.h"         // EM accesses
#include "btdm_patch.h"

/*
 * DEFINES
 *****************************************************************************************
 */



/*
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */


/*
 * VARIABLE DEFINITION
 *****************************************************************************************
 */

/// G polynom
__STATIC const uint8_t ld_util_bch_gpolynom[LD_BCH_SIZE] =
{
    0x00, 0x00, 0x00, 0x05, 0x85, 0x71, 0x3D, 0xA9
};

/// Pseudo Random Noise polynom (P)
__STATIC const uint8_t ld_util_bch_ppolynom[LD_BCH_SIZE] =
{
    0x83, 0x84, 0x8D, 0x96, 0xBB, 0xCC, 0x54, 0xFC
};

__STATIC struct bt_ch_map ld_active_ch_map;

/// Table for synchronization train channels
const uint8_t ld_sync_train_channels[SYNC_TRAIN_CHANNEL_NB] =
{
    SYNC_TRAIN_CHANNEL_0,
    SYNC_TRAIN_CHANNEL_1,
    SYNC_TRAIN_CHANNEL_2,
};

/*
 * LOCAL FUNCTIONS DECLARATION
 *****************************************************************************************
 */


/*
 * LOCAL FUNCTIONS DEFINITION
 *****************************************************************************************
 */
/*
 ****************************************************************************************
 * @brief Execute Rest = Dividend modulo Divisor.
 *
 * @param[in] dividend   Input Dividend Poly [5]
 * @param[in] divisor    Input Divisor Poly [5]
 * @param[in] rest       Output Rest Vector [5]
 *
 ****************************************************************************************
 */
__STATIC void ld_util_bch_modulo(uint8_t *dividend, const uint8_t *divisor, uint8_t *rest)
{
    uint8_t clock;
    uint8_t carry;
    uint8_t index;
    uint8_t dividend_tmp[LD_BCH_SIZE];

    FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(LD_UTIL_PATCH_TYPE, LD_UTIL_BCH_MODULO_BIT, dividend, divisor, rest);

    // Copy dividend vector in a temporary vector
    // And reset rest vector
    for (index = 0 ; index < LD_BCH_SIZE ; index++)
    {
        dividend_tmp[index] = dividend[index];
        rest[index] = 0;
    }

    // Execute 64 times the LFSR process
    for (clock = 0 ; clock < 64 ; clock++)
    {
        // Store bit Rest(degree-1) in carry (assume degree of G is 32)
        carry = rest[3] & 0x02;

        // the rest is shifted of 1 bit Left
        // The MSB of rest if lost and the MSB of dividend is shifted in LSB
        rest[0] = (rest[0] << 1) | ((rest[1] & 0x80) >> 7);
        rest[1] = (rest[1] << 1) | ((rest[2] & 0x80) >> 7);
        rest[2] = (rest[2] << 1) | ((rest[3] & 0x80) >> 7);
        rest[3] = (rest[3] << 1) | ((rest[4] & 0x80) >> 7);
        rest[4] = (rest[4] << 1) | ((rest[5] & 0x80) >> 7);
        rest[5] = (rest[5] << 1) | ((rest[6] & 0x80) >> 7);
        rest[6] = (rest[6] << 1) | ((rest[7] & 0x80) >> 7);
        rest[7] = (rest[7] << 1) | ((dividend_tmp[0] & 0x80) >> 7);

        // the dividend_tmp is shifted of 1 bit Left (a 0 is shifted in LSB)
        dividend_tmp[0] = (dividend_tmp[0] << 1) | ((dividend_tmp[1] & 0x80) >> 7);
        dividend_tmp[1] = (dividend_tmp[1] << 1) | ((dividend_tmp[2] & 0x80) >> 7);
        dividend_tmp[2] = (dividend_tmp[2] << 1) | ((dividend_tmp[3] & 0x80) >> 7);
        dividend_tmp[3] = (dividend_tmp[3] << 1) | ((dividend_tmp[4] & 0x80) >> 7);
        dividend_tmp[4] = (dividend_tmp[4] << 1) | ((dividend_tmp[5] & 0x80) >> 7);
        dividend_tmp[5] = (dividend_tmp[5] << 1) | ((dividend_tmp[6] & 0x80) >> 7);
        dividend_tmp[6] = (dividend_tmp[6] << 1) | ((dividend_tmp[7] & 0x80) >> 7);
        dividend_tmp[7] = (dividend_tmp[7] << 1);

        // If bit carry value was 1
        if (carry != 0)
        {
            // rest = rest XOR Divisor
            for (index = 0 ; index < LD_BCH_SIZE ; index++)
            {
                rest[index] ^= divisor[index];
            }
        }
    }
}

/*
 * MODULE FUNCTIONS DEFINITION
 *****************************************************************************************
 */


/*
 * EXPORTED FUNCTIONS DEFINITION
 *****************************************************************************************
 */

void ld_util_fhs_unpk(uint16_t fhs_buf_ptr, uint8_t *bch_ptr, struct bd_addr *bd_addr, struct devclass *class_of_dev, uint8_t *lt_addr, uint32_t *clk_frm, uint8_t *page_scan_rep_mode)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_NO_RETURN(LD_UTIL_PATCH_TYPE, LD_UTIL_FHS_UNPK_BIT, 7, fhs_buf_ptr, bch_ptr, bd_addr, class_of_dev, lt_addr, clk_frm, page_scan_rep_mode);

    if (bch_ptr != NULL)
    {
        uint32_t val0 = em_rd32p(fhs_buf_ptr);
        uint32_t val1 = em_rd32p(fhs_buf_ptr + 4);
        // Extract BCH
        *(bch_ptr + 0) = (val0 >> 0   & 0xFF);
        *(bch_ptr + 1) = (val0 >> 8   & 0xFF);
        *(bch_ptr + 2) = (val0 >> 16  & 0xFF);
        *(bch_ptr + 3) = (val0 >> 24  & 0xFF);
        *(bch_ptr + 4) = (val1 >> 0   & 0x03);
    }

    if (bd_addr != NULL)
    {
        // Extract LAP, UAP, NAP fields
        uint32_t lap = em_rd32p(fhs_buf_ptr + (FHS_LAP_POS / 8)) >> (FHS_LAP_POS & 0x7);
        uint32_t uap = em_rd32p(fhs_buf_ptr + (FHS_UAP_POS / 8)) >> (FHS_UAP_POS & 0x7);
        uint32_t nap = em_rd32p(fhs_buf_ptr + (FHS_NAP_POS / 8)) >> (FHS_NAP_POS & 0x7);

        // Build corresponding BD address
        bd_addr->addr[0] = (lap >> 0 & 0xFF);
        bd_addr->addr[1] = (lap >> 8  & 0xFF);
        bd_addr->addr[2] = (lap >> 16  & 0xFF);
        bd_addr->addr[3] = (uap >> 0  & 0xFF);
        bd_addr->addr[4] = (nap >> 0  & 0xFF);
        bd_addr->addr[5] = (nap >> 8  & 0xFF);
    }

    if (class_of_dev != NULL)
    {
        // Extract class of device
        uint32_t class = (em_rd32p(fhs_buf_ptr + (FHS_CLASS_OF_DEV_POS / 8)) >> (FHS_CLASS_OF_DEV_POS & 0x7)) & ((1 << FHS_CLASS_OF_DEV_LEN) - 1);
        memcpy(class_of_dev, &class, sizeof(struct devclass));
    }

    if (lt_addr != NULL)
    {
        // Extract LT Address
        *lt_addr = (em_rd8p(fhs_buf_ptr + (FHS_LT_ADDR_POS / 8)) >> (FHS_LT_ADDR_POS & 0x7)) & ((1 << FHS_LT_ADDR_LEN) - 1);
    }

    if (clk_frm != NULL)
    {
        // Extract clock
        *clk_frm = (em_rd32p(fhs_buf_ptr + (FHS_CLK_POS / 8)) >> (FHS_CLK_POS & 0x7)) & ((1 << FHS_CLK_LEN) - 1);
    }

    if (page_scan_rep_mode != NULL)
    {
        // Extract Page Scan Repetition Mode
        *page_scan_rep_mode = (em_rd8p(fhs_buf_ptr + (FHS_SR_POS / 8)) >> (FHS_SR_POS & 0x7)) & ((1 << FHS_SR_LEN) - 1);
    }
}

void ld_util_bch_create(uint8_t *lap, uint8_t *bch)
{
    uint8_t vector[LD_BCH_SIZE];
    uint8_t vector2[LD_BCH_SIZE];
    uint8_t index;

    FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(LD_UTIL_PATCH_TYPE, LD_UTIL_BCH_CREATE_BIT, lap, bch);

    // Copy lap and Appended in bch
    if (lap[2] & 0x80)
    {
        vector[0] = 0x4C;                   // If a23 = 1
    }
    else
    {
        vector[0] = 0xB0;                   // If a23 = 0
    }

    vector[0] |= lap[2] >> 6;
    vector[1] = lap[2] << 2 | lap[1] >> 6;
    vector[2] = lap[1] << 2 | lap[0] >> 6;
    vector[3] = lap[0] << 2;

    // Xor Vector and PN (Vector contains only 30 significant bits)
    for (index = 0 ; index < 4; index++)
    {
        vector[index] ^= ld_util_bch_ppolynom[index];
    }

    // Reset 34 last bits
    vector[3] &= 0xFC;
    vector[4]  = 0;
    vector[5]  = 0;
    vector[6]  = 0;
    vector[7]  = 0;

    // Generate Parity bits Vector Modulo G
    ld_util_bch_modulo(vector, ld_util_bch_gpolynom, vector2);

    // Create CodeWord (concatenate Modulo result and Xored Vector)
    vector[3] |= vector2[3];
    vector[4]  = vector2[4];
    vector[5]  = vector2[5];
    vector[6]  = vector2[6];
    vector[7]  = vector2[7];

    // Xor codeWord and PN
    for (index = 0 ; index < 8; index++)
    {
        bch[7 - index] = vector[index] ^ ld_util_bch_ppolynom[index];
    }
}

void ld_util_fhs_pk(uint16_t fhs_buf_ptr, uint8_t *bch_ptr, uint8_t eir, uint8_t sr, struct bd_addr *bd_addr, struct devclass *class_of_dev, uint8_t lt_addr, uint8_t page_scan_mode)
{
    uint32_t val;
    uint8_t undefined = 0;
    uint8_t reserved = 2; // 10 in binary

    FUNC_PATCH_ENTRY_VARI_PARAM_NO_RETURN(LD_UTIL_PATCH_TYPE, LD_UTIL_FHS_PK_BIT, 8, fhs_buf_ptr, bch_ptr, eir, sr, bd_addr, class_of_dev, lt_addr, page_scan_mode);

    val = bch_ptr[0] | (bch_ptr[1] << 8) | (bch_ptr[2] << 16) | (bch_ptr[3] << 24);
    em_wr32p(fhs_buf_ptr, val);
    val = (bch_ptr[4] & 0x03) | (bd_addr->addr[0] << 2) | (bd_addr->addr[1] << 10) | (bd_addr->addr[2] << 18) | ((eir & 0x01) << 26) | ((undefined & 0x01) << 27) | ((sr & 0x03) << 28) | ((reserved & 0x03) << 30);
    em_wr32p(fhs_buf_ptr + 4, val);
    val = bd_addr->addr[3] | (bd_addr->addr[4] << 8) | (bd_addr->addr[5] << 16) | (class_of_dev->A[0] << 24);
    em_wr32p(fhs_buf_ptr + 8, val);
    val = class_of_dev->A[1] | (class_of_dev->A[2] << 8) | ((lt_addr & 0x07) << 16);
    em_wr32p(fhs_buf_ptr + 12, val);
    val = (page_scan_mode & 0x07) << 13;
    em_wr16p(fhs_buf_ptr + 16, (uint16_t) val);
}

void ld_util_active_master_afh_map_set(struct bt_ch_map *ch_map)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(LD_UTIL_PATCH_TYPE, LD_UTIL_ACTIVE_MASTER_AFH_MAP_SET_BIT, ch_map);

    ld_active_ch_map = *ch_map;
}

struct bt_ch_map *ld_util_active_master_afh_map_get(void)
{

    return &ld_active_ch_map;
}

void ld_util_stp_unpk(uint16_t buf_ptr, uint32_t *clk_hs, uint32_t *future_csb_inst, struct bt_ch_map *afh_ch_map, struct bd_addr *mst_bd_addr, uint16_t *csb_intv, uint8_t *csb_lt_addr, uint8_t *svc_data)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_NO_RETURN(LD_UTIL_PATCH_TYPE, LD_UTIL_STP_UNPK_BIT, 8, buf_ptr, clk_hs, future_csb_inst, afh_ch_map, mst_bd_addr, csb_intv, csb_lt_addr, svc_data);

    if (clk_hs != NULL)
    {
        // Clock
        *clk_hs = em_rd32p(buf_ptr + STP_CLK_POS);
    }

    if (future_csb_inst != NULL)
    {
        // Future Connectionless Slave Broadcast Instant
        *future_csb_inst = em_rd32p(buf_ptr + STP_FUT_CSB_INST_POS);
    }

    if (afh_ch_map != NULL)
    {
        // AFH Channel Map
        em_rd(&afh_ch_map->map[0], buf_ptr + STP_AFH_CH_MAP_POS, BT_CH_MAP_LEN);
    }

    if (mst_bd_addr != NULL)
    {
        // Master BD_ADDR
        em_rd(&mst_bd_addr->addr[0], buf_ptr + STP_MST_BD_ADDR_POS, BD_ADDR_LEN);
    }

    if (csb_intv != NULL)
    {
        // Connectionless Slave Broadcast Interval
        *csb_intv = em_rd16p(buf_ptr + STP_CSB_INTV_POS);
    }

    if (csb_lt_addr != NULL)
    {
        // Connectionless Slave Broadcast LT_ADDR
        *csb_lt_addr = em_rd8p(buf_ptr + STP_CSB_LT_ADDR_POS);
    }

    if (svc_data != NULL)
    {
        // Service Data
        *svc_data = em_rd8p(buf_ptr + STP_SVC_DATA_POS);
    }
}

void ld_util_stp_pk(uint16_t buf_ptr, uint32_t clk_hs, uint32_t future_csb_inst, struct bt_ch_map *afh_ch_map, struct bd_addr *mst_bd_addr, uint16_t csb_intv, uint8_t csb_lt_addr, uint8_t svc_data)
{
    FUNC_PATCH_ENTRY_VARI_PARAM_NO_RETURN(LD_UTIL_PATCH_TYPE, LD_UTIL_STP_PK_BIT, 8, buf_ptr, clk_hs, future_csb_inst, afh_ch_map, mst_bd_addr, csb_intv, csb_lt_addr, svc_data);
    // Clock
    em_wr32p(buf_ptr + STP_CLK_POS, clk_hs);

    // Future Connectionless Slave Broadcast Instant
    em_wr32p(buf_ptr + STP_FUT_CSB_INST_POS, future_csb_inst);

    // AFH Channel Map
    if (afh_ch_map != NULL)
    {
        em_wr(&afh_ch_map->map[0], buf_ptr + STP_AFH_CH_MAP_POS, BT_CH_MAP_LEN);
    }

    // Master BD_ADDR
    if (mst_bd_addr != NULL)
    {
        em_wr(&mst_bd_addr->addr[0], buf_ptr + STP_MST_BD_ADDR_POS, BD_ADDR_LEN);
    }

    // Connectionless Slave Broadcast Interval
    em_wr16p(buf_ptr + STP_CSB_INTV_POS, csb_intv);

    // Connectionless Slave Broadcast LT_ADDR
    em_wr8p(buf_ptr + STP_CSB_LT_ADDR_POS, csb_lt_addr);

    // Service Data
    em_wr8p(buf_ptr + STP_SVC_DATA_POS, svc_data);
}

///@} LD
