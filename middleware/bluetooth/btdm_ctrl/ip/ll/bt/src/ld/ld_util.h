/**
****************************************************************************************
*
* @file ld_util.h
*
* @brief LD Util API
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/

#ifndef LD_UTIL_H_
#define LD_UTIL_H_

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

#include "rwip_config.h"    // stack configuration

#include <stdint.h>         // integer

#include "co_bt.h"          // BT standard definitions

/*
 * CONSTANT DEFINITIONS
 ****************************************************************************************
 */

/// Undefined BT clock value
#define LD_CLOCK_UNDEF       0xFFFFFFFF

/**
 * Margin between 2 ACL frames (in us)
 *
 * The chosen value is based on the packets duration and the time needed by the system to close and start a frame.
 *
 *  Remaining guard band after Poll/Null : 499us
 *  Remaining guard band after max DM1 packet : 199us
 *
 *  Time used by system after frame (Power Down / CS+Desc update) : 50
 *  Time used by system after frame (Power Up / CS+Desc fetch) : 50
 */
#define LD_ACL_INTERFRAME_MARGIN     (120)

/**
 * Margin after eSCO frame (in us)
 */
#define LD_SCO_INTERFRAME_MARGIN     (80)

/**
 * Margin after inquiry frames (in us)
 *
 * The chosen value is based on last frame duration and the time needed by the system to close and start a frame.
 *
 *  Remaining guard band after last ID packet missed reception : 250us
 *
 *  Time used by system after frame (Power Down / CS+Desc update) : 50
 *  Time used by system after frame (Power Up / CS+Desc fetch) : 50
 */
#define LD_INQ_INTERFRAME_MARGIN     (200)
#define LD_PAGE_INTERFRAME_MARGIN    (200)

/**
 * Margin after scan frames (in us)
 *
 * The chosen value is based on last frame duration and the time needed by the anticipated prefetch abort mechanism
 * to abort the scan. The prefetch abort delay is configured to the register.
 *
 * Just consider half-slot for enabling interlacing acitivites with high bit offset
 */
#define LD_SCAN_INTERFRAME_MARGIN    (SLOT_SIZE - LD_ACL_INTERFRAME_MARGIN + 1)

/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */


/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

/*
 * VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Table for synchronization train channels
extern const uint8_t ld_sync_train_channels[SYNC_TRAIN_CHANNEL_NB];


/*
 * FUNCTIONS DECLARATION
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Unpack FHS packet
 *
 * @param[in]   fhs_buf_ptr          Pointer to the FHS payload buffer (in EM)
 * @param[out]  bch_ptr              Pointer to BCH
 * @param[out]  bd_addr              BD Address
 * @param[out]  class_dev            Class Of Device
 * @param[out]  lt_addr              LT Address
 * @param[out]  clk_frm              Clock (in 1.25ms BT frame)
 * @param[out]  page_scan_rep_mode   Page Scan Repetition Mode (R0, R1, R2)
 ****************************************************************************************
 */
void ld_util_fhs_unpk(uint16_t fhs_buf_ptr, uint8_t *bch_ptr, struct bd_addr *bd_addr, struct devclass *class_of_dev, uint8_t *lt_addr, uint32_t *clk_frm, uint8_t *page_scan_rep_mode);

/**
 ****************************************************************************************
 * @brief Generate a BCH code from LAP.
 *
 * The lap pointer must point on a table :
 *                   Msb Lsb
 *          lap[0] = a7..a0
 *          lap[1] = a15..a7
 *          lap[2] = a23..a16
 * The bch result is stored in :
 *                   Msb  Lsb
 *          bch[0] = a7..a0
 *          bch[1] = a15..a8
 *          bch[2] = a23..a16
 *          bch[3] = a31..a24
 *          bch[4] = a39..a32
 *          bch[5] = a47..a40
 *          bch[6] = a55..a48
 *          bch[7] = a63..a56
 *
 * @param[in]  lap    Input LAP Vector[3]
 * @param[out] bch    Output BCH [8]
 *
 ****************************************************************************************
 */
void ld_util_bch_create(uint8_t *lap, uint8_t *bch);

/**
 ****************************************************************************************
 * @brief Pack FHS packet
 *
 * @param[in]  fhs_buf_ptr          Pointer to the FHS payload buffer (in EM)
 * @param[in]  bch_ptr              Pointer to BCH
 * @param[in]  eir                  Value indicating whether EIR is available or not
 * @param[in]  sr                   Scan repetition
 * @param[in]  bd_addr              BD Address
 * @param[in]  class_of_dev         Class Of Device
 * @param[in]  lt_addr              LT Address
 * @param[in]  page_scan_mode       Page scan mode
 ****************************************************************************************
 */
void ld_util_fhs_pk(uint16_t fhs_buf_ptr, uint8_t *bch_ptr, uint8_t eir, uint8_t sr, struct bd_addr *bd_addr, struct devclass *class_of_dev, uint8_t lt_addr, uint8_t page_scan_mode);

/**
 ****************************************************************************************
 * @brief Set the active AFH Map
 *
 * @param[in]  ch_map             AFH map
 ****************************************************************************************
 */
void ld_util_active_master_afh_map_set(struct bt_ch_map *ch_map);

/**
 ****************************************************************************************
 * @brief Get the active AFH Map
 *
 * @param[out]  chnl_map             AFH map
 ****************************************************************************************
 */
struct bt_ch_map *ld_util_active_master_afh_map_get(void);

/**
 ****************************************************************************************
 * @brief Unpack sync train packet
 *
 * @param[in]   buf_ptr          Pointer to the STP payload buffer (in EM)
 * @param[out]  clk_hs           Current piconet clock (CLK) (in half slots)
 * @param[out]  future_csb_inst  Future Connectionless Slave Broadcast Instant
 * @param[out]  afh_ch_map       The current AFH_Channel_Map used for the PBD logical link
 * @param[out]  mst_bd_addr      Master BD Address
 * @param[out]  csb_intv         CSB interval (in slots)
 * @param[out]  csb_lt_addr      The LT_ADDR reserved for the CSB logical transport
 * @param[out]  svc_data         Defined by the service using the Connectionless Slave Broadcast feature
 ****************************************************************************************
 */
void ld_util_stp_unpk(uint16_t buf_ptr, uint32_t *clk_hs, uint32_t *future_csb_inst, struct bt_ch_map *afh_ch_map, struct bd_addr *mst_bd_addr, uint16_t *csb_intv, uint8_t *csb_lt_addr, uint8_t *svc_data);

/**
 ****************************************************************************************
 * @brief Pack sync train packet
 *
 * @param[in]   buf_ptr          Pointer to the STP payload buffer (in EM)
 * @param[out]  clk_hs           Current piconet clock (CLK) (in half slots)
 * @param[out]  future_csb_inst  Future Connectionless Slave Broadcast Instant (in slots)
 * @param[out]  afh_ch_map       The current AFH_Channel_Map used for the PBD logical link
 * @param[out]  mst_bd_addr      Master BD Address
 * @param[out]  csb_intv         CSB interval (in slots)
 * @param[out]  csb_lt_addr      The LT_ADDR reserved for the CSB logical transport
 * @param[out]  svc_data         Defined by the service using the Connectionless Slave Broadcast feature
 ****************************************************************************************
 */
void ld_util_stp_pk(uint16_t buf_ptr, uint32_t clk_hs, uint32_t future_csb_inst, struct bt_ch_map *afh_ch_map, struct bd_addr *mst_bd_addr, uint16_t csb_intv, uint8_t csb_lt_addr, uint8_t svc_data);


/// @} LD
#endif // LD_UTIL_H_
