/**
****************************************************************************************
*
* @file rf_au50xx.c
*
* @brief AU50XX radio initialization and specific functions
*
* Copyright (C) RivieraWaves 2009-2015
*
* $Rev: $
*
****************************************************************************************
*/

/**
****************************************************************************************
* @addtogroup RF_AU50XX
* @ingroup RF
* @brief AU50XX Radio Driver
*
* This is the driver block for AU50XX radio
* @{
****************************************************************************************
*/

/**
 *****************************************************************************************
 * INCLUDE FILES
 *****************************************************************************************
 */
#include <string.h>        // for memcpy
#include "co_utils.h"      // common utility definition
#include "co_math.h"       // common math functions
#include "co_endian.h"     // endian definitions
#include "rf.h"            // RF interface
#include "em_map.h"        // RF area mapping

#include "rwip.h"          // for RF API structure definition
#include "reg_ipcore.h"    // DM core registers
#include "reg_blecore.h"   // ble core registers
#include "reg_em_ble_cs.h" // control structure definitions

#include "plf.h"           // Platform register
#include "lcd.h"           // LCD interface

#include "dbg.h"           // LCD interface

// AU50XX register definitions and access functions
__STATIC uint32_t rf_au50xx_reg_rd(uint32_t addr);
__STATIC void rf_au50xx_reg_wr(uint32_t addr, uint32_t value);
#if !defined(RP_HWSIM_BYPASS)
    __STATIC void rf_au50xx_init_seq(void);
#endif //(RP_HWSIM_BYPASS)

#define REG_AU50XX_RD                rf_au50xx_reg_rd
#define REG_AU50XX_WR                rf_au50xx_reg_wr

/* The offset value given below is the offset to add to the frequency table index to
   get the value to be programmed in the radio for each channel                      */
#define AU50XX_FREQTAB_OFFSET          0   // Offset for AU50XX radio
/**
 ****************************************************************************************
 * DEFINES
 ****************************************************************************************
 **/

#define RF_EM_SPI_ADRESS              (EM_BASE_ADDR + EM_RF_SW_SPI_OFFSET)

#define AU50XX_SPIRD                   0x00
#define AU50XX_SPIWR                   0x80

#define AU50XX_MAX_BURST_SIZE          0x80

// TX max power
#define AU50XX_POWER_MAX               0x0C

#define AU50XX_SPIRD                   0x00
#define AU50XX_SPIWR                   0x80

#define AU50XX_RSSI_20dB_THRHLD        -20
#define AU50XX_RSSI_40dB_THRHLD        -40
#define AU50XX_RSSI_45dB_THRHLD        -45
#define AU50XX_RSSI_48dB_THRHLD        -48
#define AU50XX_RSSI_55dB_THRHLD        -55
#define AU50XX_RSSI_60dB_THRHLD        -60
#define AU50XX_RSSI_70dB_THRHLD        -70
/**
****************************************************************************************
* MACROS
*****************************************************************************************
*/

/// AU50XX EM Write Macro for HW driven SPI chained structures
#define RF_AU50XX_EM_BLE_WR(addr, val) \
    EM_BLE_WR((((uint32_t) (addr)) + REG_EM_BLE_CS_BASE_ADDR), (val))


// Max burst register
__STATIC uint8_t rf_au50xx_reg_buf[AU50XX_MAX_BURST_SIZE + 5]; // max burst size + buffer controls

/**
 ****************************************************************************************
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 **/

/* AU50XX dynamic register */
enum
{
    AU50XX_MODE_CFG_RX1   = 0x10,
    AU50XX_MODE_CFG_TX1   = 0x11,
    AU50XX_MODE_CFG_TXRX  = 0x14,
    RSSI                = 0x17,
    TIM_REC_CFG         = 0x39,
    DCOC_DBG            = 0x69,
};

/**
 ****************************************************************************************
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief SPI access
 ***************************************************************************************
 */
__STATIC void rf_au50xx_spi_tf(void)
{
    //launch SPI transfer
    ip_radiocntl0_spigo_setf(1);

    //wait for transfer to be completed
    while (!ip_radiocntl0_spicomp_getf());
}

/**
 ****************************************************************************************
 * @brief AU50XX specific read access
 *
 * @param[in] addr    register address
 *
 * @return uint32_t value
 *****************************************************************************************
 */
__STATIC uint32_t rf_au50xx_reg_rd(uint32_t addr)
{
    // Next Pointr to 0x0
    rf_au50xx_reg_buf[0] = (uint8_t)(0);  // Next Pointer set to 0x0000 to stop the SPI Chained access
    rf_au50xx_reg_buf[1] = (uint8_t)(0);

    //copy control and number of u32 to send
    rf_au50xx_reg_buf[2] = (uint8_t)(0);

    //copy address
    rf_au50xx_reg_buf[3] = (uint8_t)(addr & 0x00FF);

    // Data
    rf_au50xx_reg_buf[4] = 0x00;
    rf_au50xx_reg_buf[5] = 0x00;

    memcpy((void *)RF_EM_SPI_ADRESS, rf_au50xx_reg_buf, 6);

    //do the transfer
    rf_au50xx_spi_tf();

    return (uint32_t)((*((uint8_t *)(RF_EM_SPI_ADRESS + 4))) | ((*((uint8_t *)(RF_EM_SPI_ADRESS + 5))) << 8));
}

/**
 ****************************************************************************************
 * @brief AU50XX specific write access
 *
 * @param[in] addr    register address
 * @param[in] value   value to write
 *
 * @return uint32_t value
 ****************************************************************************************
 */
__STATIC void rf_au50xx_reg_wr(uint32_t addr, uint32_t value)
{
    rf_au50xx_reg_buf[0] = (uint8_t)(0);  // Next Pointer set to 0x0000 to stop the SPI Chained access
    rf_au50xx_reg_buf[1] = (uint8_t)(0);

    //inversion for EM reading by U8 on ATL SPI side
    //copy control and number of u32 to send
    rf_au50xx_reg_buf[2] = (uint8_t)(AU50XX_SPIWR);

    //copy address
    rf_au50xx_reg_buf[3] = (uint8_t)(addr & 0x00FF);

    // Byte to be written
    co_write32p(&rf_au50xx_reg_buf[4], value);

    memcpy((void *)RF_EM_SPI_ADRESS, rf_au50xx_reg_buf, 6);

    //do the transfer
    rf_au50xx_spi_tf();
}

__STATIC void rf_em_init(void)
{
    uint8_t idx = 0;
    uint8_t temp_freq_tbl[EM_RF_FREQ_TABLE_LEN];

    while (idx < EM_RF_FREQ_TABLE_LEN)
    {
        temp_freq_tbl[idx] = idx + AU50XX_FREQTAB_OFFSET;
        idx++;
    }
    em_wr(&temp_freq_tbl[0], EM_FT_OFFSET, EM_RF_FREQ_TABLE_LEN);
}

/**
 *****************************************************************************************
 * @brief Init RF sequence after reset.
 *****************************************************************************************
 */
__STATIC void rf_reset(void)
{
}

/***************************************************************************************
 * @brief Static function - Initialization sequence for Ripple radio
 ****************************************************************************************
 */
#if !defined(RP_HWSIM_BYPASS)
__STATIC void rf_au50xx_init_seq(void)
{
    /* ***********************/
    /* RF Clock Enable **/
    /* **********************/
    rf_au50xx_reg_wr(0xE1, 0x100B);
    rf_au50xx_reg_wr(0xE0, 0x3D57);

    /* *****************************/
    /* RF register initialization */
    /* ****************************/
    /* To increase the pulse width if rx_data_clk */
    rf_au50xx_reg_wr(0x3C, 0x0019);
    ASSERT_ERR(rf_au50xx_reg_rd(0x3C) == 0x0019);

    /* For PA ramp up of 2us */
    rf_au50xx_reg_wr(0x12, 0x0049);
    ASSERT_ERR(rf_au50xx_reg_rd(0x12) == 0x0049);

    /* To set LDO_ANT = 1.05v */
    rf_au50xx_reg_wr(0x13, 0x239A);
    ASSERT_ERR(rf_au50xx_reg_rd(0x13) == 0x239A);

    /* To set PA Bias to lower harmonics in TX */
    rf_au50xx_reg_wr(0xA6, 0x0031);
    ASSERT_ERR(rf_au50xx_reg_rd(0xA6) == 0x0031);

    /* To enable static current in FCAL to reduce drop in LDO voltage */
    rf_au50xx_reg_wr(0xAE, 0x0001);
    ASSERT_ERR(rf_au50xx_reg_rd(0xAE) == 0x0001);

    /* Setting for LR mode */
    rf_au50xx_reg_wr(0x1B, 0x095CA);
    rf_au50xx_reg_wr(0x1C, 0x71CC);
    ASSERT_ERR(rf_au50xx_reg_rd(0x1B) == 0x095CA);
    ASSERT_ERR(rf_au50xx_reg_rd(0x1C) == 0x71CC);

    /* Setting for filtering of RSSI readback */
    rf_au50xx_reg_wr(0x18, 0x0002);
    ASSERT_ERR(rf_au50xx_reg_rd(0x18) == 0x0002);

    /* RSSI_Stop = 4095 */
    rf_au50xx_reg_wr(0x19, 0x0FFF);
    ASSERT_ERR(rf_au50xx_reg_rd(0x19) == 0x0FFF);

    /* ALPHA_EXP = 3 */
    rf_au50xx_reg_wr(0x1A, 0x0003);
    ASSERT_ERR(rf_au50xx_reg_rd(0x1A) == 0x0003);

    /* Setting for blanking */
    rf_au50xx_reg_wr(0x3D, 0x0E20);
    ASSERT_ERR(rf_au50xx_reg_rd(0x3D) == 0x0E20);

    /* To set AGC Thresholds */
    rf_au50xx_reg_wr(0x25, 0x0709);
    rf_au50xx_reg_wr(0x27, 0x0F08);
    rf_au50xx_reg_wr(0x2A, 0x0709);
    rf_au50xx_reg_wr(0x2B, 0x078C);
    rf_au50xx_reg_wr(0x2C, 0x0F08);
    ASSERT_ERR(rf_au50xx_reg_rd(0x25) == 0x0709);
    ASSERT_ERR(rf_au50xx_reg_rd(0x27) == 0x0F08);
    ASSERT_ERR(rf_au50xx_reg_rd(0x2A) == 0x0709);
    ASSERT_ERR(rf_au50xx_reg_rd(0x2B) == 0x078C);
    ASSERT_ERR(rf_au50xx_reg_rd(0x2C) == 0x0F08);

    /* For Optimum preamble detection */
    rf_au50xx_reg_wr(0x3B, 0x07350);
    ASSERT_ERR(rf_au50xx_reg_rd(0x3B) == 0x07350);
}
#endif
/**
 ****************************************************************************************
 * @brief ISR to be called in BLE ISR routine when RF Interrupt occurs.
 *****************************************************************************************
 */
__STATIC void rf_force_agc_enable(bool en)
{
}

/**
 *****************************************************************************************
 * @brief Get TX power in dBm from the index in the control structure
 *
 * @param[in] txpwr_idx  Index of the TX power in the control structure
 * @param[in] modulation Modulation: 1 or 2 or 3 MBPS
 *
 * @return The TX power in dBm
 *
 *****************************************************************************************
 */
__STATIC int8_t rf_txpwr_dbm_get(uint8_t txpwr_idx, uint8_t modulation)
{
    // Power table should be provided
    return (0);
}

/**
 *****************************************************************************************
 * @brief Sleep function for AU50XX RF.
 *****************************************************************************************
 */
__STATIC void rf_sleep(void)
{
    ip_deepslcntl_set(ip_deepslcntl_get() |
                      IP_DEEP_SLEEP_ON_BIT |    // RW BLE Core sleep
                      IP_RADIO_SLEEP_EN_BIT |   // Radio sleep
                      IP_OSC_SLEEP_EN_BIT);     // Oscillator sleep
}

/**
 *****************************************************************************************
 * @brief Convert RSSI to dBm
 *
 * @param[in] rssi_reg RSSI read from the HW registers
 *
 * @return The converted RSSI
 *
 *****************************************************************************************
 */
__STATIC int8_t rf_rssi_convert(uint8_t rssi_reg)
{
    int8_t RssidBm = 0;
    RssidBm = ((rssi_reg) >> 1) - 118;
    return (RssidBm);
}

/**
 *****************************************************************************************
 * @brief Get the TX power as control structure TX power field from a value in dBm.
 *
 * @param[in] txpwr_dbm   TX power in dBm
 * @param[in] high        If true, return index equal to or higher than requested
 *                        If false, return index equal to or lower than requested
 *
 * @return The index of the TX power
 *
 *****************************************************************************************
 */
__STATIC uint8_t rf_txpwr_cs_get(int8_t txpwr_dbm, uint8_t option)
{
    return (AU50XX_POWER_MAX);
}

/**
 ****************************************************************************************
 * RADIO FUNCTION INTERFACE
 ****************************************************************************************
 **/
void rf_init(struct rwip_rf_api *api)
{
    uint16_t nextptr ;
    uint8_t length = PARAM_LEN_RSSI_THR;

    // Initialize the RF driver API structure
    api->reg_rd = rf_au50xx_reg_rd;
    api->reg_wr = rf_au50xx_reg_wr;
    api->txpwr_dbm_get = rf_txpwr_dbm_get;
    api->txpwr_min = AU50XX_POWER_MAX;
    api->txpwr_max = AU50XX_POWER_MAX;
    api->sleep = rf_sleep;
    api->reset = rf_reset;
    api->force_agc_enable = rf_force_agc_enable;
    api->rssi_convert = rf_rssi_convert;
    api->txpwr_cs_get = rf_txpwr_cs_get;

    /// Initialize Exchange Memory
    rf_em_init();

    // Initialize the RSSI thresholds (high, low, interference)
    // The upper threshold level is 20 dB above the lower threshold level to an accuracy of +-6 dB
    // These are 'real' signed values in dBm
    if (rwip_param.get(PARAM_ID_RSSI_HIGH_THR, &length, (uint8_t *)&api->rssi_high_thr) != PARAM_OK)
    {
        api->rssi_high_thr = (int8_t)AU50XX_RSSI_40dB_THRHLD;
    }
    if (rwip_param.get(PARAM_ID_RSSI_LOW_THR, &length, (uint8_t *)&api->rssi_low_thr) != PARAM_OK)
    {
        api->rssi_low_thr = (int8_t)AU50XX_RSSI_60dB_THRHLD;
    }
    if (rwip_param.get(PARAM_ID_RSSI_INTERF_THR, &length, (uint8_t *)&api->rssi_interf_thr) != PARAM_OK)
    {
        api->rssi_interf_thr = (int8_t)AU50XX_RSSI_70dB_THRHLD;
    }

    ip_radiocntl0_pack(/*uint16_t spiptr*/ (EM_RF_SW_SPI_OFFSET >> 2),
                                           /*uint8_t  spicfg*/   0,
                                           /*uint8_t  spifreq*/  1,
                                           /*uint8_t  spigo*/    0);

    /* BLE RADIOCNTL1 */
    ip_radiocntl1_pack(/*uint8_t  forceagcen*/       0,
            /*uint8_t  forceiq*/         0,
            /*uint8_t  rxdnsl*/          0,
            /*uint8_t  txdnsl*/          0,
            /*uint16_t forceagclength*/  0x0,
            /*uint8_t  syncpulsemode*/   1,
            /*uint8_t  syncpulsesrc*/    0,
            /*uint8_t  dpcorren*/        0,
            /*uint8_t  jefselect*/       1,
            /*uint8_t  xrfsel*/          0x06,
            /*uint8_t  subversion*/      0x0);

    /* BLE RADIOCNTL2 */
    ble_radiocntl2_pack(/*uint8_t  lrsynccompmode*/ 0x1,
            /*uint8_t  rxcitermbypass*/ 0x0,
            /*uint8_t  lrvtbflush*/     0x8,
            /*uint8_t  phymsk*/         0x3, // mark that Coded phy and 2Mbps are supported
            /*uint8_t  lrsyncerr*/      0,
            /*uint8_t  syncerr*/        0,
            /*uint16_t freqtableptr*/ (EM_FT_OFFSET >> 2));

    /* BLE RADIOCNTL3 */
    ble_radiocntl3_pack(/*uint8_t rxrate3cfg*/    0x2,
            /*uint8_t rxrate2cfg*/    0x2,
            /*uint8_t rxrate1cfg*/    0x1,
            /*uint8_t rxrate0cfg*/    0x0,
            /*uint8_t rxsyncrouting*/ 0x0,
            /*uint8_t getrssidelay*/  0x0,
            /*uint8_t rxvalidbeh*/    0x0,
            /*uint8_t txrate3cfg*/    0x0,
            /*uint8_t txrate2cfg*/    0x0,
            /*uint8_t txrate1cfg*/    0x1,
            /*uint8_t txrate0cfg*/    0x0,
            /*uint8_t txvalidbeh*/    0x3);

    /* BLE RADIOPWRUPDN0 */
    ble_radiopwrupdn0_pack(/*uint8_t syncposition0*/   0x0,
            /*uint8_t rxpwrup0*/        0x64,
            /*uint8_t txpwrdn0*/        0x04,
            /*uint8_t txpwrup0*/        0x75);

    ble_radiopwrupdn1_pack(/*uint8_t syncposition1*/   0x0,
            /*uint8_t rxpwrup1*/        0x64,
            /*uint8_t txpwrdn1*/        0x04,
            /*uint8_t txpwrup1*/        0x75);

    /* BLE RADIOPWRUPDN2 */
    ble_radiopwrupdn2_pack(/*uint8_t syncposition2*/   0x0,
            /*uint8_t rxpwrup2*/        0x64,
            /*uint8_t txpwrdn2*/        0x04,
            /*uint8_t txpwrup2*/        0x75);

    /* BLE RADIOPWRUPDN3 */
    ble_radiopwrupdn3_pack(/*uint8_t txpwrdn3*/        0x04,
            /*uint8_t txpwrup3*/        0x75);

    /* BLE RADIOTXRXTIM0 */
    ble_radiotxrxtim0_pack(/*uint8_t rfrxtmda0*/       0x13,
            /*uint8_t rxpathdly0*/      0x13,
            /*uint8_t txpathdly0*/      0x2);

    /* BLE RADIOTXRXTIM1 */
    ble_radiotxrxtim1_pack(/*uint8_t rfrxtmda1*/       0x9,
            /*uint8_t rxpathdly1*/      0x9,
            /*uint8_t txpathdly1*/      0x2);

    /* BLE RADIOTXRXTIM2 */
    ble_radiotxrxtim2_pack(/*uint8_t rxflushpathdly2*/ 0x12,
            /*uint8_t rfrxtmda2*/       0x4E,
            /*uint8_t rxpathdly2*/      0x38,
            /*uint8_t txpathdly2*/      0x2);

    /* BLE RADIOTXRXTIM3 */
    ble_radiotxrxtim3_pack(/*uint8_t rxflushpathdly3*/ 0x12,
            /*uint8_t rfrxtmda3*/       0x1E,
            /*uint8_t txpathdly3*/      0x2);




    /* *************************************************************************************** */
    /* Initialize HW SPI Chains Pointers  */
    /* *************************************************************************************** */
    uint16_t txonptr      = (EM_RF_HW_SPI_OFFSET);
#if !defined(RP_HWSIM_BYPASS)
    uint16_t txoffptr     = 0;
#else
    uint16_t txoffptr     = (EM_RF_HW_SPI_OFFSET + 0x20); // Needed for TB au50xx spi Tx en == off
#endif
    uint16_t rxonptr      = (EM_RF_HW_SPI_OFFSET + 0x20 + 0x10);
    uint16_t rxoffptr     = (EM_RF_HW_SPI_OFFSET + 0x20 + 0x10 + 0x30);
    uint16_t rssiptr      = (EM_RF_HW_SPI_OFFSET + 0x20 + 0x10 + 0x30 + 0x10);

    /* TxOn Sequence start pointer */
    ble_spiptrcntl0_txonptr_setf(txonptr >> 2);

    /* TxOff Sequence start pointer */
    ble_spiptrcntl0_txoffptr_setf(txoffptr >> 2);

    /* RxOn Sequence start pointer */
    ble_spiptrcntl1_rxonptr_setf(rxonptr >> 2);

    /* RxOff Sequence start pointer */
    ble_spiptrcntl1_rxoffptr_setf(rxoffptr >> 2);

    /* RSSI Sequence start pointer */
    ble_spiptrcntl2_rssiptr_setf(rssiptr >> 2);

    /* *************************************************************************************** */
    /* Initialize HW SPI Tx On Chained list  -> 1 structure of 16 bytes*/
    /* *************************************************************************************** */
    /*     TxON Sequence -> Write MODE_CFG_TX1
                   ->   Next Pointer = 0x1010
                   ->   0xc0 -> Write Access / SPI_SPLIT=1
                   ->   SPI Address = MODE_CFG_TX1=0x11
                   ->   LE1M    : 0x2320
                   ->   LE2M    : 0x0643
                   ->   LECoded : 0x2320
                   -> Write BURST_CFG_TXRX
                   ->   Next Pointer = 0x00
                   ->   0x80 -> Write Access / SPI_SPLIT=0
                   ->   SPI Address = BURST_CFG_TXRX=0x14
                   ->   write Data Channel Index and set TXEN*/
    nextptr = (uint16_t)((txonptr + 0xC) >> 2);
    RF_AU50XX_EM_BLE_WR(txonptr,    nextptr);
    RF_AU50XX_EM_BLE_WR(txonptr + 0x2, 0x11C0);
    RF_AU50XX_EM_BLE_WR(txonptr + 0x4, 0x2320);
    RF_AU50XX_EM_BLE_WR(txonptr + 0x6, 0x0643);
    RF_AU50XX_EM_BLE_WR(txonptr + 0x8, 0x2320);
    RF_AU50XX_EM_BLE_WR(txonptr + 0xC, 0x0000);
    RF_AU50XX_EM_BLE_WR(txonptr + 0xE, 0x1480);
    RF_AU50XX_EM_BLE_WR(txonptr + 0x10, 0x0040);




    /* *************************************************************************************** */
    /* Initialize HW SPI Tx Off Chained list  -> 1 structure of 8 bytes*/
    /* *************************************************************************************** */

    /*     TxOff Sequence -> Write BURST_CFG_TXRX
                 ->   Next Pointer = 0x00
                 ->   0x80 -> Write Access / SPI_SPLIT=0
                 ->   SPI Address = BURST_CFG_TXRX=0x14
                 ->   write Data Channel Index and Disable TXEN*/
    RF_AU50XX_EM_BLE_WR(txoffptr,     0x0000);
    RF_AU50XX_EM_BLE_WR(txoffptr + 0x2, 0x1480);
    RF_AU50XX_EM_BLE_WR(txoffptr + 0x4, 0x0000);
    RF_AU50XX_EM_BLE_WR(txoffptr + 0x6, 0x0000);

    /* *************************************************************************************** */
    /* Initialize HW SPI Rx On Chained list -> 4 structures of 10 bytes */
    /* *************************************************************************************** */
    /* Initialize HW SPI Rx On Chained list  */
    /*   RxON Sequence RxON Sequence
                  -> Write MODE_CFG_RX1
                       Next Pointer = 0x1010
                       0xc0 -> Write Access / SPI_SPLIT=1
                       SPI Address = MODE_CFG_RX1=0x10
                       LE1M    : 0x1016
                       LE2M    : 0x5004
                       LECoded : 0x1017
                  -> Write TIM_REC_CFG
                       Next Pointer = 0x1020
                       0xC0 -> Write Access / SPI_SPLIT=1
                       SPI Address = TIM_REC_CFG=0x39
                       LE1M    : 0x019B
                       LE2M    : 0x019B
                       LECoded : 0x019B
                  -> Write DCOC_DBG
                       Next Pointer = 0x1030
                       Write Access / SPI_SPLIT=1
                       SPI Address = DCOC_DBG=0x69
                       LE1M    : 0x15BB
                       LE2M    : 0x14FB
                       LECoded : 0x15BB
                   -> Write BURST_CFG_TXRX
                       Next Pointer = 0x0000
                       0x80 -> Write Access / SPI_SPLIT=1
                       SPI Address = BURST_CFG_TXRX=0x14
                       // Bytes to be written  => will be or-ed with RF Rx config by HW*/

    /* MODE_CFG_RX1 */
    nextptr = (uint16_t)((rxonptr + 0xC) >> 2);
    RF_AU50XX_EM_BLE_WR(rxonptr,     nextptr);
    RF_AU50XX_EM_BLE_WR(rxonptr + 0x2, 0x10c0);
    RF_AU50XX_EM_BLE_WR(rxonptr + 0x4, 0x1016);
    RF_AU50XX_EM_BLE_WR(rxonptr + 0x6, 0x5004);
    RF_AU50XX_EM_BLE_WR(rxonptr + 0x8, 0x1017);

    /* TIM_REC_CFG */
    nextptr = (uint16_t)((rxonptr + 0x18) >> 2);
    RF_AU50XX_EM_BLE_WR(rxonptr + 0xC,  nextptr);
    RF_AU50XX_EM_BLE_WR(rxonptr + 0xE, 0x39c0);
    RF_AU50XX_EM_BLE_WR(rxonptr + 0x10, 0x019B);
    RF_AU50XX_EM_BLE_WR(rxonptr + 0x12, 0x019B);
    RF_AU50XX_EM_BLE_WR(rxonptr + 0x14, 0x019B);

    /* DCOC_DBG */
    nextptr = (uint16_t)((rxonptr + 0x24) >> 2);
    RF_AU50XX_EM_BLE_WR(rxonptr + 0x18, nextptr);
    RF_AU50XX_EM_BLE_WR(rxonptr + 0x1A, 0x69c0);
    RF_AU50XX_EM_BLE_WR(rxonptr + 0x1C, 0x15BB);
    RF_AU50XX_EM_BLE_WR(rxonptr + 0x20, 0x14FB);
    RF_AU50XX_EM_BLE_WR(rxonptr + 0x12, 0x15BB);

    /* BURST_CFG_TXRX */
    RF_AU50XX_EM_BLE_WR(rxonptr + 0x24, 0x0000);
    RF_AU50XX_EM_BLE_WR(rxonptr + 0x26, 0x1480);
    RF_AU50XX_EM_BLE_WR(rxonptr + 0x28, 0x0080);


    /* *************************************************************************************** */
    /* Initialize HW SPI Rx Off -> 1 structure of 8 bytes*/
    /* *************************************************************************************** */
    /*     TxOff Sequence -> Write BURST_CFG_TXRX
                ->   Next Pointer = 0x00
                ->   0x80 -> Write Access / SPI_SPLIT=0
                ->   SPI Address = BURST_CFG_TXRX=0x14
                ->   write Data Channel Index and Disable RXEN*/
    RF_AU50XX_EM_BLE_WR(rxoffptr,     0x0000);
    RF_AU50XX_EM_BLE_WR(rxoffptr + 0x2, 0x1480);
    RF_AU50XX_EM_BLE_WR(rxoffptr + 0x4, 0x0000);
    RF_AU50XX_EM_BLE_WR(rxoffptr + 0x6, 0x0000);


    /* *************************************************************************************** */
    /* Initialize HW SPI RSSI Chained list  -> 1 structure of 6 bytes*/
    /* *************************************************************************************** */
    /*        RSSI Sequence -> Read RSSI
                ->  Next Pointer = 0x00
                ->  0x00 -> Read Access / SPI_SPLIT=0
                ->  SPI Address = RSSI=0x17
                ->  Byte to be write by HW  */
    RF_AU50XX_EM_BLE_WR(rssiptr,     0x0000);
    RF_AU50XX_EM_BLE_WR(rssiptr + 0x2, 0x1700);
    RF_AU50XX_EM_BLE_WR(rssiptr + 0x4, 0x0000);


#if !defined(RP_HWSIM_BYPASS)
    // AU50XX initialization sequence
    rf_au50xx_init_seq();
#endif //RP_HWSIM_BYPASS


};
///@} RF_AU50XX
