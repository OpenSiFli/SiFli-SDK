/*****************************************************************************************
*
* @file rf_extrc.c
*
* @brief External Radio Controller initialization and specific functions
*
* Copyright (C) RivieraWaves 2009-2017
*
*
*****************************************************************************************/

/*****************************************************************************************
* @addtogroup RF_EXTRC
* @ingroup RF
* @brief External Radio Controller Driver
*
* This is the driver block for external radio controller
* @{
*****************************************************************************************/

/*****************************************************************************************
 * INCLUDE FILES
 ****************************************************************************************/

#include "rwip_config.h"        // RW SW configuration

#include <string.h>             // for memcpy
#include "co_utils.h"           // common utility definition
#include "co_math.h"            // common math functions
#include "rf.h"                 // RF interface
#include "plf.h"                // Platform functions
#include "rwip.h"               // for RF API structure definition
#include "em_map.h"             // exchange table

#include "reg_ipcore.h"        // DM core registers

#if (BLE_EMB_PRESENT)
    #include "reg_blecore.h"        // ble core registers
    #include "reg_em_ble_cs.h"      // control structure definitions
#endif //(BLE_EMB_PRESENT)

#if (BT_EMB_PRESENT)
    #include "reg_btcore.h"         // bt core registers
    #include "reg_em_bt_cs.h"       // control structure definitions
#endif //(BT_EMB_PRESENT)

/*****************************************************************************************
 * DEFINES
 ****************************************************************************************/

#define RF_GAIN_TBL_SIZE           (8)
#define RF_PWR_TBL_SIZE            (8)

#define RF_RSSI_20dB_THRHLD        -20
#define RF_RSSI_40dB_THRHLD        -40
#define RF_RSSI_45dB_THRHLD        -45
#define RF_RSSI_48dB_THRHLD        -48
#define RF_RSSI_55dB_THRHLD        -55
#define RF_RSSI_60dB_THRHLD        -60
#define RF_RSSI_70dB_THRHLD        -70

// TX max power (index)
#define RF_POWER_MAX                7
#define RF_POWER_MIN                1

// Tx power step size (in dBm)
#define RF_TX_PWR_STEP_SIZE (3)

/*****************************************************************************************
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************/

// Gain table
__STATIC const uint8_t RF_RX_GAIN_TBL[RF_GAIN_TBL_SIZE] =
{
    [0] = 43,
    [1] = 37,
    [2] = 31,
    [3] = 25,
    [4] = 19,
    [5] = 13,
    [6] = 7,
    [7] = 1
};

// Power table
__STATIC const int8_t RF_TX_PW_CONV_TBL[RF_PWR_TBL_SIZE] =
{
    [0] = -23,
    [1] = -20,
    [2] = -17,
    [3] = -14,
    [4] = -11,
    [5] = -8,
    [6] = -5,
    [7] = -2
};

/*****************************************************************************************
 * FUNCTION DEFINITIONS
 ****************************************************************************************/

/*****************************************************************************************
 * @brief Read access
 *
 * @param[in] addr    register address
 *
 * @return uint32_t value
 ****************************************************************************************/
__STATIC uint32_t rf_reg_rd(uint32_t addr)
{
    return 0;
}

/*****************************************************************************************
 * @brief Write access
 *
 * @param[in] addr    register address
 * @param[in] value   value to write
 ****************************************************************************************/
__STATIC void rf_reg_wr(uint32_t addr, uint32_t value)
{
    return;
}

/*****************************************************************************************
 * @brief Initialize frequency table in the exchange memory
 ****************************************************************************************/
__STATIC void rf_em_init(void)
{
    uint8_t idx = 0;
    uint8_t temp_freq_tbl[EM_RF_FREQ_TABLE_LEN];

#if (BT_EMB_PRESENT)
    // First half part of frequency table is for the even frequencies
    while (idx < (EM_RF_FREQ_TABLE_LEN / 2))
    {
        temp_freq_tbl[idx] = 2 * idx;
        idx++;
    }

    while (idx < EM_RF_FREQ_TABLE_LEN)
    {
        temp_freq_tbl[idx] = 2 * (idx - (EM_RF_FREQ_TABLE_LEN / 2)) + 1;
        idx++;
    }

    em_wr(&temp_freq_tbl[0], EM_FT_OFFSET, EM_RF_FREQ_TABLE_LEN);
#elif (BLE_EMB_PRESENT)
    while (idx < EM_RF_FREQ_TABLE_LEN)
    {
        temp_freq_tbl[idx] = 2 * idx;
        idx++;
    }

    em_wr(&temp_freq_tbl[0], EM_FT_OFFSET, EM_RF_FREQ_TABLE_LEN);
#endif //(BT_EMB_PRESENT/BLE_EMB_PRESENT)
}

/**
 *****************************************************************************************
 * @brief Convert RSSI to dBm
 *
 * @param[in] rssi_reg RSSI read from the HW registers
 *
 * @return The converted RSSI
 *****************************************************************************************
 */
__STATIC int8_t rf_rssi_convert(uint8_t rssi_reg)
{
    int8_t rssi_dbm;
    uint16_t power_modem;

    /* Get the RSSI value from the look up table and get its signed value
     * Get the 2-complements signed value on 8 bits */
    power_modem = ((rssi_reg & 0xF8) >> 3) * 2;
    rssi_dbm = power_modem - RF_RX_GAIN_TBL[rssi_reg & 0x07] - 64;

    return (rssi_dbm);
}

/**
 *****************************************************************************************
 * @brief Get the TX power as control structure TX power field from a value in dBm.
 *
 * @param[in] txpwr_dbm   TX power in dBm
 * @param[in] option      If TXPWR_CS_LOWER, return index equal to or lower than requested
 *                        If TXPWR_CS_HIGHER, return index equal to or higher than requested
 *                        If TXPWR_CS_NEAREST, return index nearest to the desired value
 *
 * @return The TX power value in dBm
 *
 *****************************************************************************************
 */
__STATIC uint8_t rf_txpwr_cs_get(int8_t txpwr_dbm, uint8_t option)
{
    ASSERT_ERR(option <= TXPWR_CS_NEAREST);

    uint8_t i;

    for (i = RF_POWER_MIN; i < RF_POWER_MAX; i++)
    {
        // Loop until we find a power higher than or equal to the requested one
        if (RF_TX_PW_CONV_TBL[i] >= txpwr_dbm)
            break;
    }

    if ((RF_TX_PW_CONV_TBL[i] > txpwr_dbm) && (i > RF_POWER_MIN))
    {
        if ((option == TXPWR_CS_LOWER)
                || ((option == TXPWR_CS_NEAREST) && (co_abs(txpwr_dbm - RF_TX_PW_CONV_TBL[i - 1]) < co_abs(txpwr_dbm - RF_TX_PW_CONV_TBL[i]))))
        {
            i--;
        }
    }

    return (RF_TX_PW_CONV_TBL[i]);
}

/**
 *****************************************************************************************
 * @brief Init RF sequence after reset.
 *****************************************************************************************
 */
__STATIC void rf_reset(void)
{
    return;
}

#if (BT_EMB_PRESENT)
/**
 *****************************************************************************************
 * @brief Decrease the TX power by one step
 *
 * @param[in] link_id Link ID for which the TX power has to be decreased
 *
 * @return true when minimum power is reached, false otherwise
 *****************************************************************************************
 */
__STATIC bool rf_txpwr_dec(uint8_t link_id)
{
    // Get current TX power value
    int8_t tx_pwr = (int8_t)em_bt_pwrcntl_txpwr_getf(EM_BT_CS_ACL_INDEX(link_id));

    // Check if value can be decreased
    if (tx_pwr > RF_TX_PW_CONV_TBL[RF_POWER_MIN])
    {
        // Decrease the TX power value
        em_bt_pwrcntl_txpwr_setf(EM_BT_CS_ACL_INDEX(link_id), tx_pwr - RF_TX_PWR_STEP_SIZE);
    }

    return !(tx_pwr > RF_TX_PW_CONV_TBL[RF_POWER_MIN]);
}

/**
 *****************************************************************************************
 * @brief Increase the TX power by one step
 *
 * @param[in] link_id Link ID for which the TX power has to be increased
 *
 * @return true when maximum power is reached, false otherwise
 *****************************************************************************************
 */
__STATIC bool rf_txpwr_inc(uint8_t link_id)
{
    // Get current TX power value
    int8_t tx_pwr = (int8_t)em_bt_pwrcntl_txpwr_getf(EM_BT_CS_ACL_INDEX(link_id));

    // Check if value can be increased
    if (tx_pwr < RF_TX_PW_CONV_TBL[RF_POWER_MAX])
    {
        // Increase the TX power value
        em_bt_pwrcntl_txpwr_setf(EM_BT_CS_ACL_INDEX(link_id), tx_pwr + RF_TX_PWR_STEP_SIZE);
    }

    return !(tx_pwr < RF_TX_PW_CONV_TBL[RF_POWER_MAX]);
}

/**
 ****************************************************************************************
 * @brief Set the TX power to max
 *
 * @param[in] link_id     Link Identifier
 ****************************************************************************************
 */
__STATIC void rf_txpwr_max_set(uint8_t link_id)
{
    // Set max TX power value
    em_bt_pwrcntl_txpwr_setf(EM_BT_CS_ACL_INDEX(link_id), RF_TX_PW_CONV_TBL[RF_POWER_MAX]);
}
#endif //(BT_EMB_PRESENT)

#if (BLE_EMB_PRESENT)
/**
 *****************************************************************************************
 * @brief Enable/disable force AGC mechanism
 *
 * @param[in]  True: Enable / False: disable
 *****************************************************************************************
 */
__STATIC void rf_force_agc_enable(bool en)
{
    return;
}
#endif //(BLE_EMB_PRESENT)

/**
 *****************************************************************************************
 * @brief Get TX power in dBm from the index in the control structure
 *
 * @param[in] txpwr_idx  Value of the TX power in the control structure
 * @param[in] modulation Modulation: 1 or 2 or 3 MBPS
 *
 * @return The TX power in dBm
 *****************************************************************************************
 */
__STATIC int8_t rf_txpwr_dbm_get(uint8_t txpwr_idx, uint8_t modulation)
{
    // power value is equivalent
    return txpwr_idx;
}

/**
 *****************************************************************************************
 * @brief Sleep function for the RF.
 *****************************************************************************************
 */
__STATIC void rf_sleep(void)
{
    ip_deepslcntl_set(ip_deepslcntl_get() |
                      IP_DEEP_SLEEP_ON_BIT |     // RW BT Core sleep
                      IP_RADIO_SLEEP_EN_BIT |    // Radio sleep
                      IP_OSC_SLEEP_EN_BIT);      // Oscillator sleep
}

/****************************************************************************************
 * RADIO FUNCTION INTERFACE
 ***************************************************************************************/
void rf_init(struct rwip_rf_api *api)
{
#if (BLE_EMB_PRESENT)
#if (!BT_EMB_PRESENT)
    // Indicate which PHYs are supported
    uint8_t phy_msk = 0;
#endif //(!BT_EMB_PRESENT)
#endif //(BLE_EMB_PRESENT)

    // ********************************************************
    // *       Initialize the RF driver API structure         *
    // ********************************************************

    api->reg_rd        = rf_reg_rd;
    api->reg_wr        = rf_reg_wr;
    api->txpwr_dbm_get = rf_txpwr_dbm_get;
    api->txpwr_min     = RF_TX_PW_CONV_TBL[RF_POWER_MIN];
    api->txpwr_max     = RF_TX_PW_CONV_TBL[RF_POWER_MAX];
    api->sleep         = rf_sleep;
    api->reset         = rf_reset;
    api->rssi_convert  = rf_rssi_convert;
    api->txpwr_cs_get  = rf_txpwr_cs_get;

#if (BLE_EMB_PRESENT)
    api->force_agc_enable = rf_force_agc_enable;
#endif //(BLE_EMB_PRESENT)

#if (BT_EMB_PRESENT)
    api->txpwr_dec       = rf_txpwr_dec;
    api->txpwr_inc       = rf_txpwr_inc;
    api->txpwr_max_set   = rf_txpwr_max_set;
#endif //(BT_EMB_PRESENT)
    api->rssi_interf_thr = RF_RSSI_70dB_THRHLD;
    api->rssi_high_thr   = RF_RSSI_40dB_THRHLD;
    api->rssi_low_thr    = RF_RSSI_60dB_THRHLD;

    // ********************************************************
    // *             Initialize Exchange Memory               *
    // ********************************************************

    rf_em_init();

    // ********************************************************
    // *          Initialize BLE/BT Core Registers            *
    // ********************************************************

    /* BLE RADIOCNTL0 */
    ip_radiocntl0_pack(/*uint16_t spiptr*/    0,
            /*uint8_t  spicfg*/   0,
            /*uint8_t  spifreq*/  0,
            /*uint8_t  spigo*/    0);

    /* BLE RADIOCNTL1 */
    ip_radiocntl1_pack(/*uint8_t  forceagcen*/       0,
            /*uint8_t  forceiq*/         0,
            /*uint8_t  rxdnsl*/          0,
            /*uint8_t  txdnsl*/          0,
            /*uint16_t forceagclength*/  0,
            /*uint8_t  syncpulsemode*/   0,
            /*uint8_t  syncpulsesrc*/    0,
            /*uint8_t  dpcorren*/        0,
            /*uint8_t  jefselect*/       1,
            /*uint8_t  xrfsel*/          2,
            /*uint8_t  subversion*/      0);

#if (BLE_EMB_PRESENT)
#if (!BT_EMB_PRESENT)

    // Check if 2M and Coded PHY must be set as supported
#if (BLE_PHY_2MBPS_SUPPORT)
    phy_msk |= (1 << 0);
#endif //(BLE_PHY_2MBPS_SUPPORT)
#if (BLE_PHY_CODED_SUPPORT)
    phy_msk |= (1 << 1);
#endif //(BLE_PHY_CODED_SUPPORT)

    /* BLE RADIOCNTL2 */
    ble_radiocntl2_pack(/*uint8_t  lrsynccompmode*/ 0x3,
            /*uint8_t  rxcitermbypass*/ 0x0,
            /*uint8_t  lrvtbflush*/     8,
            /*uint8_t  phymsk*/         phy_msk,
            /*uint8_t  lrsyncerr*/      0,
            /*uint8_t  syncerr*/        0,
            /*uint16_t freqtableptr*/   EM_FT_OFFSET >> 2);

    /* BLE RADIOCNTL3 */
    ble_radiocntl3_pack(/*uint8_t rxrate3cfg*/    0x3,
            /*uint8_t rxrate2cfg*/    0x2,
            /*uint8_t rxrate1cfg*/    0x1,
            /*uint8_t rxrate0cfg*/    0x0,
            /*uint8_t getrssidelay */ BLE_GETRSSIDELAY_RST,
            /*uint8_t rxsyncrouting*/ 0x0,
            /*uint8_t rxvalidbeh*/    0x0,
            /*uint8_t txrate3cfg*/    0x3,
            /*uint8_t txrate2cfg*/    0x2,
            /*uint8_t txrate1cfg*/    0x1,
            /*uint8_t txrate0cfg*/    0x0,
            /*uint8_t txvalidbeh*/    0x0);
#endif //(!BT_EMB_PRESENT)

    ble_radiocntl2_phymsk_setf(0x3); // mark that 2mbps and Coded phy are supported for TLM

#ifdef CFG_MODEM_LR
#define EXTRC_RXPATHDLY2            0x41
#define EXTRC_RXFLUSHPATHDLY2       0x11
#define EXTRC_RFRXTMDA2             1
#define EXTRC_TXPATHDLY2            2
#define EXTRC_RXFLUSHPATHDLY3       0x11
#define EXTRC_RFRXTMDA3             1
#define EXTRC_TXPATHDLY3            2
#else //(!CFG_MODEM_LR)
#define EXTRC_RXPATHDLY2            2
#define EXTRC_RXFLUSHPATHDLY2       0xF
#define EXTRC_RFRXTMDA2             2
#define EXTRC_TXPATHDLY2            1
#define EXTRC_RXFLUSHPATHDLY3       0xD
#define EXTRC_RFRXTMDA3             0
#define EXTRC_TXPATHDLY3            1
#endif //(!CFG_MODEM_LR)

#define EXTRC_RXPWRUP0              0x30
#define EXTRC_TXPWRUP0              0x31
#define EXTRC_RXPWRUP1              0x31
#define EXTRC_TXPWRUP1              0x31
#define EXTRC_RXPWRUP2              0x31
#define EXTRC_TXPWRUP2              0x31
#define EXTRC_TXPWRUP3              0x31
#define EXTRC_RFRXTMDA0             2
#define EXTRC_RXPATHDLY0            2
#define EXTRC_TXPATHDLY0            1
#define EXTRC_TXPATHDLY1            1

    /* BLE RADIOPWRUPDN0 */
    ble_radiopwrupdn0_pack(/*uint8_t syncposition0*/ 0,
            /*uint8_t rxpwrup0*/      EXTRC_RXPWRUP0,
            /*uint8_t txpwrdn0*/      0x07,
            /*uint8_t txpwrup0*/      EXTRC_TXPWRUP0);

    /* BLE RADIOPWRUPDN1 */
    ble_radiopwrupdn1_pack(/*uint8_t syncposition1*/ 0,
            /*uint8_t rxpwrup1*/      EXTRC_RXPWRUP1,
            /*uint8_t txpwrdn1*/      0x07,
            /*uint8_t txpwrup1*/      EXTRC_TXPWRUP1);

    /* BLE RADIOPWRUPDN2 */
    ble_radiopwrupdn2_pack(/*uint8_t syncposition2*/ 0,
            /*uint8_t rxpwrup2*/      EXTRC_RXPWRUP2,
            /*uint8_t txpwrdn2*/      0x07,
            /*uint8_t txpwrup2*/      EXTRC_TXPWRUP2);

    /* BLE RADIOPWRUPDN3 */
    ble_radiopwrupdn3_pack(/*uint8_t txpwrdn3*/      0x07,
            /*uint8_t txpwrup3*/      EXTRC_TXPWRUP3);


    /* BLE RADIOTXRXTIM0 */
    ble_radiotxrxtim0_pack(/*uint8_t rfrxtmda0*/   EXTRC_RFRXTMDA0,
            /*uint8_t rxpathdly0*/  EXTRC_RXPATHDLY0,
            /*uint8_t txpathdly0*/  EXTRC_TXPATHDLY0);

    /* BLE RADIOTXRXTIM1 */
    ble_radiotxrxtim1_pack(/*uint8_t rfrxtmda1*/   1,
            /*uint8_t rxpathdly1*/  1,
            /*uint8_t txpathdly1*/  EXTRC_TXPATHDLY1);

    /* BLE RADIOTXRXTIM2 */
    ble_radiotxrxtim2_pack(/*uint8_t rxflushpathdly2*/ EXTRC_RXFLUSHPATHDLY2,
            /*uint8_t rfrxtmda2*/       EXTRC_RFRXTMDA2,
            /*uint8_t rxpathdly2*/      EXTRC_RXPATHDLY2,
            /*uint8_t txpathdly2*/      EXTRC_TXPATHDLY2);

    /* BLE RADIOTXRXTIM3 */
    ble_radiotxrxtim3_pack(/*uint8_t rxflushpathdly3*/ EXTRC_RXFLUSHPATHDLY3,
            /*uint8_t rfrxtmda3*/       EXTRC_RFRXTMDA3,
            /*uint8_t txpathdly3*/      EXTRC_TXPATHDLY3);

#if (BLE_CON_CTE_REQ | BLE_CONLESS_CTE_RX)
    // Init the DF CNTL
    ble_dfcntl0_1us_pack(/*uint8_t rxsampstinst01us*/ 0x08, /*uint8_t rxswstinst01us*/ 0x18, /*uint8_t txswstinst01us*/ 0x19);
    ble_dfcntl0_2us_pack(/*uint8_t rxsampstinst02us*/ 0x08, /*uint8_t rxswstinst02us*/ 0x18, /*uint8_t txswstinst02us*/ 0x19);
    ble_dfcntl1_1us_pack(/*uint8_t rxsampstinst11us*/ 0x08, /*uint8_t rxswstinst11us*/ 0x18, /*uint8_t txswstinst11us*/ 0x19);
    ble_dfcntl1_2us_pack(/*uint8_t rxsampstinst12us*/ 0x08, /*uint8_t rxswstinst12us*/ 0x18, /*uint8_t txswstinst12us*/ 0x19);
    ble_dfantcntl_pack(/*uint8_t rxprimidcntlen*/ 1, /*uint8_t rxprimantid*/ 0, /*uint8_t txprimidcntlen*/ 1, /*uint8_t txprimantid*/ 0);
#endif // (BLE_CON_CTE_REQ | BLE_CONLESS_CTE_RX)

#endif //(BLE_EMB_PRESENT)

#if (BT_EMB_PRESENT)

#define EXTRC_RXGRD_TIMEOUT         18
#define EXTRC_TXPWRDNCT             0x02
#define EXTRC_RXPATHDLY             2
#define EXTRC_TXPATHDLY             1
#define EXTRC_SYNC_POSITION         0

    /* EDRCNTL */
    bt_rwbtcntl_nwinsize_setf(0);
    bt_edrcntl_rxgrd_timeout_setf(EXTRC_RXGRD_TIMEOUT);

    /* BT RADIOPWRUPDN */
    bt_radiopwrupdn_rxpwrupct_setf(0x31);
    bt_radiopwrupdn_txpwrdnct_setf(EXTRC_TXPWRDNCT);
    bt_radiopwrupdn_txpwrupct_setf(0x31);

    /* BT RADIOCNTL 2 */
    bt_radiocntl2_freqtable_ptr_setf((EM_FT_OFFSET >> 2));
    bt_radiocntl2_syncerr_setf(0x7);

    /* BT RADIOTXRXTIM */
    bt_radiotxrxtim_rxpathdly_setf(EXTRC_RXPATHDLY);
    bt_radiotxrxtim_txpathdly_setf(EXTRC_TXPATHDLY);
    bt_radiotxrxtim_sync_position_setf(EXTRC_SYNC_POSITION);

    /* BT RADIOCNTL 3*/
    bt_radiocntl3_pack(/*uint8_t rxrate2cfg*/    3,
            /*uint8_t rxrate1cfg*/    2,
            /*uint8_t rxrate0cfg*/    1,
            /*uint8_t getrssidelay*/  0,
            /*uint8_t rxserparif*/    0,
            /*uint8_t rxsyncrouting*/ 0,
            /*uint8_t rxvalidbeh*/    0,
            /*uint8_t txrate2cfg*/    3,
            /*uint8_t txrate1cfg*/    2,
            /*uint8_t txrate0cfg*/    1,
            /*uint8_t txserparif*/    0,
            /*uint8_t txvalidbeh*/    0);
#endif //(BT_EMB_PRESENT)

}

///@} RF_EXTRC
