/**
****************************************************************************************
*
* @file rf_btipt.c
*
* @brief Atlas radio initialization and specific functions
*
* Copyright (C) RivieraWaves 2009-2015
*
* $Rev: $
*
****************************************************************************************
*/

/**
****************************************************************************************
* @addtogroup RF_ICYV2
* @ingroup RF
* @brief Atlas Radio Driver
*
* This is the driver block for BTIPT radio
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

#if (BLE_EMB_PRESENT)
    #include "reg_blecore.h"   // ble core registers
    #include "reg_em_ble_cs.h" // control structure definitions
#endif //BLE_EMB_PRESENT

#if (BT_EMB_PRESENT)
    #include "reg_btcore.h"    // bt core registers
    #include "reg_em_bt_cs.h"  // control structure definitions
#endif //BT_EMB_PRESENT

#include "plf.h"           // Platform register
#include "flash.h"         // Flash interface
#include "lcd.h"           // LCD interface

#include "dbg.h"           // LCD interface

// BTIPT register definitions and access functions
__STATIC uint32_t rf_btipt_reg_rd(uint32_t addr);
__STATIC void rf_btipt_reg_wr(uint32_t addr, uint32_t value);

/**
 ****************************************************************************************
 * DEFINES
 ****************************************************************************************
 **/

#define REG_BTIPT_RD                     rf_btipt_reg_rd
#define REG_BTIPT_WR                     rf_btipt_reg_wr

#define RF_EM_SPI_ADRESS                (EM_BASE_ADDR + EM_RF_SW_SPI_OFFSET)
/* The offset value given below is the offset to add to the frequency table index to
   get the value to be programmed in the radio for each channel                      */
#define BTIPT_FREQTAB_OFFSET             0   // Offset for BTIPT radio

#define RF_FLASH_BTIPT_PROG_MEM         (0x00D00000)
#define RF_FLASH_BTIPT_DATA_MEM         (0x00D40000)

#define RF_BTDM001T28ES1_PROG_MEM_PTR    0x4000
#define RF_BTDM001T28ES1_DATA_MEM_BUF    0x0000

#define RF_BTDM001G28ES1_PROG_MEM_PTR    0xF0000
#define RF_BTDM001G28ES1_DATA_MEM_BUF    0x18400

#define RF_BTDM001T28ES2_PROG_MEM_PTR    0xF0000
#define RF_BTDM001T28ES2_DATA_MEM_BUF    0x08400

#define RF_BTDM001T28ES3_PROG_MEM_PTR    0x0F000
#define RF_BTDM001T28ES3_DATA_MEM_BUF    0x06000

#define RF_BTDM001T28ES1_PROG_STEP       6
#define RF_BTDM001T28ES1_DATA_STEP       2
#define RF_BTDM001T28ES1_ADDRESS         2

#define RF_BTDM001G28ES1_PROG_STEP       14
#define RF_BTDM001G28ES1_DATA_STEP       2
#define RF_BTDM001G28ES1_ADDRESS         4

#define RF_BTDM001T28ES2_PROG_STEP       12
#define RF_BTDM001T28ES2_DATA_STEP       2
#define RF_BTDM001T28ES2_ADDRESS         4

#define RF_BTDM001T28ES3_PROG_STEP       12
#define RF_BTDM001T28ES3_DATA_STEP       2
#define RF_BTDM001T28ES3_ADDRESS         4

#define BTIPT_SPIRD                      0x00
#define BTIPT_SPIWR                      0x80

#define BTIPT_RSSI_20dB_THRHLD           -20
#define BTIPT_RSSI_40dB_THRHLD           -40
#define BTIPT_RSSI_45dB_THRHLD           -45
#define BTIPT_RSSI_48dB_THRHLD           -48
#define BTIPT_RSSI_55dB_THRHLD           -55
#define BTIPT_RSSI_60dB_THRHLD           -60
#define BTIPT_RSSI_70dB_THRHLD           -70

#define BTIPT_RSSI_INVALID               127

// TX max power
#define BTIPT_POWER_MIN                  0x00
#define BTIPT_POWER_MAX                  0x09
#define BTIPT_POWER_MSK                  0x3F

#define BTIPT_MAX_BURST_SIZE             32

#if defined(CFG_GAIA)
    #define BRAM_RF_DATA_OFFSET       (0x10000)
    #define BRAM_RF_PROG_OFFSET       (0x30000)
#endif // defined(CFG_GAIA)

enum btipt_chip_vers
{
    RF_BTIPT_BTDM001T28ES1,
    RF_BTIPT_BTDM001G28ES1,
    RF_BTIPT_BTDM001T28ES2,
    RF_BTIPT_BTDM001T28ES3
};

/**
 ****************************************************************************************
 * STRUCTURE DEFINITION
 *****************************************************************************************
 */

#if defined(CFG_GAIA)
// RF load BRAM format
typedef struct
{
    uint8_t addr[3];
    uint8_t ld_len;
    uint8_t ld[12];
} fileio_rf_struct;
#endif // defined(CFG_GAIA)

/**
 ****************************************************************************************
 * MACROS
 *****************************************************************************************
 */

/// BTIPT EM Write Macro for HW driven SPI chained structures
#define RF_BTIPT_EM_WR(addr, val) \
        EM_WR((((uint32_t) (addr)) + REG_EM_ET_BASE_ADDR), (val))

/**
 ****************************************************************************************
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 **/

// Max burst register
__STATIC uint8_t rf_btipt_reg_buf[BTIPT_MAX_BURST_SIZE + 7]; // max burst size + buffer controls
/**
 ****************************************************************************************
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Frequency Table Initialization In Exchange memory
 ***************************************************************************************
 */
__STATIC void rf_em_init(void)
{
    uint8_t idx = 0;
    uint8_t temp_freq_tbl[EM_RF_FREQ_TABLE_LEN];

#if BT_EMB_PRESENT
    // First half part of frequency table is for the even frequencies
    while (idx < EM_RF_FREQ_TABLE_LEN / 2)
    {
        temp_freq_tbl[idx] = 2 * idx + BTIPT_FREQTAB_OFFSET;
        idx++;
    }
    while (idx < EM_RF_FREQ_TABLE_LEN)
    {
        temp_freq_tbl[idx] = 2 * (idx - (EM_RF_FREQ_TABLE_LEN / 2)) + 1 + BTIPT_FREQTAB_OFFSET;
        idx++;
    }
#elif BLE_EMB_PRESENT
    while (idx < EM_RF_FREQ_TABLE_LEN)
    {
        temp_freq_tbl[idx] = 2 * idx + BTIPT_FREQTAB_OFFSET;
        idx++;
    }
#endif // BT_EMB_PRESENT/BLE_EMB_PRESENT
    em_wr(&temp_freq_tbl[0], EM_FT_OFFSET, EM_RF_FREQ_TABLE_LEN);
}

/**
 ****************************************************************************************
 * @brief SPI access
 ***************************************************************************************
 */
__STATIC void rf_btipt_spi_tf(void)
{
    //launch SPI transfer
    ip_radiocntl0_spicfg_setf(1);
    ip_radiocntl0_spigo_setf(1);

    //wait for transfer to be completed
    while (!ip_radiocntl0_spicomp_getf());
    ip_radiocntl0_spicfg_setf(0);
}

/**
 ****************************************************************************************
 * @brief BTIPT specific read access
 *
 * @param[in] addr    register address
 *
 * @return uint32_t value
 *****************************************************************************************
 */
__STATIC uint32_t rf_btipt_reg_rd(uint32_t addr)
{
    // Next Pointr to 0x0
    rf_btipt_reg_buf[0] = (uint8_t)(0);  // Next Pointer set to 0x0000 to stop the SPI Chained access
    rf_btipt_reg_buf[1] = (uint8_t)(0);

    // Header contains acces length in 4 LSBs
    rf_btipt_reg_buf[2] = 0x01;
    rf_btipt_reg_buf[3] = 0x00;

    // Address
    rf_btipt_reg_buf[4] = (uint8_t)(addr & 0x00FF);
    rf_btipt_reg_buf[5] = (uint8_t)((addr & 0xFF00) >> 8);
    rf_btipt_reg_buf[6] = (uint8_t)((addr & 0x7F0000) >> 16);

    // R/W bit
    rf_btipt_reg_buf[7] = (uint8_t)(BTIPT_SPIRD);

    // Data
    rf_btipt_reg_buf[8] = 0x00;
    rf_btipt_reg_buf[9] = 0x00;

    memcpy((void *)RF_EM_SPI_ADRESS, rf_btipt_reg_buf, 10);

    //do the transfer
    rf_btipt_spi_tf();

//    uint32_t dbg = (uint32_t)((*((uint8_t *)(RF_EM_SPI_ADRESS + 8))) | ((*((uint8_t *)(RF_EM_SPI_ADRESS + 9))) << 8));

//    lcd_printf(LCD_LINE_1,"val: %X",dbg);

    return (uint32_t)((*((uint8_t *)(RF_EM_SPI_ADRESS + 8))) | ((*((uint8_t *)(RF_EM_SPI_ADRESS + 9))) << 8));
}

/**
 ****************************************************************************************
 * @brief BTIPT specific write access
 *
 * @param[in] addr    register address
 * @param[in] value   value to write
 *
 * @return uint32_t value
 ****************************************************************************************
 */
__STATIC void rf_btipt_reg_wr(uint32_t addr, uint32_t value)
{
    // Next Pointr to 0x0
    rf_btipt_reg_buf[0] = (uint8_t)(0);  // Next Pointer set to 0x0000 to stop the SPI Chained access
    rf_btipt_reg_buf[1] = (uint8_t)(0);

    // Header contains acces length in 4 LSBs
    rf_btipt_reg_buf[2] = 0x01;
    rf_btipt_reg_buf[3] = 0x00;

    // Address
    rf_btipt_reg_buf[4] = (uint8_t)(addr & 0x00FF);
    rf_btipt_reg_buf[5] = (uint8_t)((addr & 0xFF00) >> 8);
    rf_btipt_reg_buf[6] = (uint8_t)((addr & 0x7F0000) >> 16);

    // R/W bit
    rf_btipt_reg_buf[7] = (uint8_t)(BTIPT_SPIWR);

    // Data
    rf_btipt_reg_buf[8] = (uint8_t)(value & 0x000000FF);
    rf_btipt_reg_buf[9] = (uint8_t)((value & 0x0000FF00) >> 8);

    memcpy((void *)RF_EM_SPI_ADRESS, rf_btipt_reg_buf, 10);

    //do the transfer
    rf_btipt_spi_tf();
}

__STATIC void rf_btipt_reg_wr_full(uint32_t addr, uint16_t header, uint16_t value)
{
    // Next Pointr to 0x0
    rf_btipt_reg_buf[0] = (uint8_t)(0);  // Next Pointer set to 0x0000 to stop the SPI Chained access
    rf_btipt_reg_buf[1] = (uint8_t)(0);

    // Header contains acces length in 4 LSBs
    rf_btipt_reg_buf[2] = 0x02;
    rf_btipt_reg_buf[3] = 0x00;

    // Address
    rf_btipt_reg_buf[4] = (uint8_t)(addr & 0x00FF);
    rf_btipt_reg_buf[5] = (uint8_t)((addr & 0xFF00) >> 8);
    rf_btipt_reg_buf[6] = (uint8_t)((addr & 0x7F0000) >> 16);

    // R/W bit
    rf_btipt_reg_buf[7] = (uint8_t)(BTIPT_SPIWR);

    // Data
    rf_btipt_reg_buf[8] = (uint8_t)(header & 0x000FF);
    rf_btipt_reg_buf[9] = (uint8_t)((header & 0xFF00) >> 8);
    rf_btipt_reg_buf[10] = (uint8_t)(value & 0x00FF);
    rf_btipt_reg_buf[11] = (uint8_t)((value & 0xFF00) >> 8);

    memcpy((void *)RF_EM_SPI_ADRESS, rf_btipt_reg_buf, 12);

    //do the transfer
    rf_btipt_spi_tf();
}

/**
 ****************************************************************************************
 * @brief BTIPT specific write access
 *
 * @param[in] addr    register address
 * @param[in] size    transfer size
 * @param[in] data    pointer to the data array
 *
 * @return uint32_t value
 ****************************************************************************************
 **/
//static void rf_btipt_reg_burst_rd (uint32_t addr, uint8_t size, uint8_t *data)
//{
//    // Next Pointr to 0x0
//    rf_btipt_reg_buf[0] = (uint8_t)(0);  // Next Pointer set to 0x0000 to stop the SPI Chained access
//    rf_btipt_reg_buf[1] = (uint8_t)(0);
//
//    // Header contains acces length in 4 LSBs
//    rf_btipt_reg_buf[2] = (uint8_t)(size>>1);
//    rf_btipt_reg_buf[3] = (uint8_t)(0x00);
//
//    // Address
//    rf_btipt_reg_buf[4] = (uint8_t)(addr & 0x00FF);
//    rf_btipt_reg_buf[5] = (uint8_t)((addr & 0xFF00) >> 8 );
//    rf_btipt_reg_buf[6] = (uint8_t)((addr & 0x7F0000)>>16);
//
//    // R/W bit
//    rf_btipt_reg_buf[7] = (uint8_t)(BTIPT_SPIRD);
//
//    for(int i =0;i < size+2;i++)
//    {
//        rf_btipt_reg_buf[i + 8] = 0x00;
//    }
//
//    memcpy((void *)RF_EM_SPI_ADRESS, rf_btipt_reg_buf,8 + size);
//
//    //do the transfer
//    rf_btipt_spi_tf();
//    memcpy(data,(void *)(RF_EM_SPI_ADRESS+8),size);
//    //    return (uint8_t *)(RF_EM_SPI_ADRESS + 8);
//
//
//}

/**
 ****************************************************************************************
 * @brief BTIPT specific read access
 *
 * @param[in] addr    register address
 * @param[in] size    transfer size
 * @param[in] data    pointer to the data array
 *
 * @return uint32_t value
 ****************************************************************************************
 **/
#if !defined(RP_HWSIM_BYPASS)
__STATIC void rf_btipt_reg_burst_wr(uint32_t addr, uint8_t size, uint8_t *data)
{
    // Next Pointr to 0x0
    rf_btipt_reg_buf[0] = (uint8_t)(0);  // Next Pointer set to 0x0000 to stop the SPI Chained access
    rf_btipt_reg_buf[1] = (uint8_t)(0);

    // Header contains acces length in 4 LSBs
    rf_btipt_reg_buf[2] = (uint8_t)(size >> 1);
    rf_btipt_reg_buf[3] = (uint8_t)(0x00);

    // Address
    rf_btipt_reg_buf[4] = (uint8_t)(addr & 0x00FF);
    rf_btipt_reg_buf[5] = (uint8_t)((addr & 0xFF00) >> 8);
    rf_btipt_reg_buf[6] = (uint8_t)((addr & 0x7F0000) >> 16);

    // R/W bit
    rf_btipt_reg_buf[7] = (uint8_t)(BTIPT_SPIWR);

    for (int i = 0; i < size; i++)
    {
        rf_btipt_reg_buf[i + 8] = *(data + i);
    }

    memcpy((void *)RF_EM_SPI_ADRESS, rf_btipt_reg_buf, 8 + size);

    //do the transfer
    rf_btipt_spi_tf();
}
#endif



/**
 *****************************************************************************************
 * @brief Init RF sequence after reset.
 *****************************************************************************************
 */
__STATIC void rf_reset(void)
{
}

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
    return (0x04);
}

/**
 *****************************************************************************************
 * @brief Sleep function for BTIPT RF.
 *****************************************************************************************
 */
__STATIC void rf_sleep(void)
{
    ip_deepslcntl_set(ip_deepslcntl_get() |
                      IP_DEEP_SLEEP_ON_BIT |    // RW BT Core sleep
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

    //Check if a value is availble else set the minimum to prevent channel assessment issue
    //If the rssi read is equal to BTIPT_RSSI_INVALID it means no valid data
    if (rssi_reg == BTIPT_RSSI_INVALID)
    {
        RssidBm = BTIPT_RSSI_70dB_THRHLD;
    }
    else
    {
        //2's complement
        RssidBm = rssi_reg;
    }

    return (RssidBm);
}

#if defined(CFG_BT_EMB)
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
    bool boMinpow = true;
    uint8_t tx_pwr = em_bt_pwrcntl_txpwr_getf(EM_BT_CS_ACL_INDEX(link_id)) & BTIPT_POWER_MSK;

    if (tx_pwr > BTIPT_POWER_MIN)
    {
        //Increase the TX power value
        em_bt_pwrcntl_txpwr_setf(EM_BT_CS_ACL_INDEX(link_id), tx_pwr - 3);
        boMinpow = false;
    }

    return (boMinpow);
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
    bool boMaxpow = true;
    uint8_t tx_pwr = em_bt_pwrcntl_txpwr_getf(EM_BT_CS_ACL_INDEX(link_id)) & BTIPT_POWER_MSK;

    if (tx_pwr < BTIPT_POWER_MAX)
    {
        //Increase the TX power value
        em_bt_pwrcntl_txpwr_setf(EM_BT_CS_ACL_INDEX(link_id), tx_pwr + 3);
        boMaxpow = false;
    }

    return (boMaxpow);
}

/**
 ****************************************************************************************
 * @brief Set the TX power to max
 *
 * @param[in] link_id     Link Identifier
 ****************************************************************************************
 */
__STATIC void txpwr_max_set(uint8_t link_id)
{
    //Increase the TX power value
    em_bt_pwrcntl_txpwr_setf(EM_BT_CS_ACL_INDEX(link_id), BTIPT_POWER_MAX);
}
#endif //CFG_BT_EMB

/**
 *****************************************************************************************
 * @brief Get the TX power as control structure TX power field from a value in dBm.
 *
 * @param[in] txpwr_dbm   TX power in dBm
 * @param[in] option      If TXPWR_CS_LOWER, return index equal to or lower than requested
 *                        If TXPWR_CS_HIGHER, return index equal to or higher than requested
 *                        If TXPWR_CS_NEAREST, return index nearest to the desired value
 *
 * @return The index of the TX power
 *
 *****************************************************************************************
 */
__STATIC uint8_t rf_txpwr_cs_get(int8_t txpwr_dbm, uint8_t option)
{
    return (BTIPT_POWER_MAX);
}


/**
 ****************************************************************************************
 * RADIO FUNCTION INTERFACE
 ****************************************************************************************
 **/
void rf_init(struct rwip_rf_api *api)
{

    //  uint32_t btipt_version = 0; // Default version is BTIPT V1

#if !defined(RP_HWSIM_BYPASS)
    uint32_t btipt_prog_mem_ptr = 0;
    uint32_t btipt_data_mem_ptr = 0;
#endif

    uint16_t btipt_cmd_buf_base_add = 0x1400;
    uint16_t btipt_rsp_buf_base_add = 0x140A;

#if !defined(RP_HWSIM_BYPASS)
#if !defined(CFG_GAIA)
    uint16_t btipt_prog_mem_length = 0;
    uint16_t btipt_data_mem_length = 0;

    uint8_t  btipt_flash_read_buf[32];
#endif // !defined(CFG_GAIA)
#endif

#if !defined(RP_HWSIM_BYPASS)
    uint16_t btipt_version[4];
    uint8_t  btipt_fw_minor;
#endif

#if  defined(CFG_BT_EMB)
    uint8_t bt_rxpath;
    uint8_t bt_txpath;
    uint8_t bt_rxpwrup;
    uint8_t bt_txpwrdn;
    uint8_t bt_txpwrup;
#endif //CFG_BT_EMB


#if defined(CFG_BLE_EMB)
    uint8_t ble_rxpath_1m;
    uint8_t ble_txpath_1m;
    uint8_t ble_rxpwrup_1m;
    uint8_t ble_txpwrdn_1m;
    uint8_t ble_txpwrup_1m;

    uint8_t ble_rxpath_2m;
    uint8_t ble_txpath_2m;
    uint8_t ble_rxpwrup_2m;
    uint8_t ble_txpwrdn_2m;
    uint8_t ble_txpwrup_2m;
#endif //CFG_BLE_EMB

    uint16_t btipt_xo_setting;
    uint16_t btipt_gain_setting;

#if !defined(RP_HWSIM_BYPASS)
    uint8_t btipt_version_chip;
#endif
    uint8_t btipt_subversion = 0;

#if !defined(RP_HWSIM_BYPASS)
#if !defined(CFG_GAIA)
    uint8_t btipt_data_step = 0;
    uint8_t btipt_prog_step = 0;
    uint8_t flash_status;
#endif
#endif // !defined(CFG_GAIA)

#if !defined(RP_HWSIM_BYPASS)
    uint8_t btipt_version_length = PARAM_LEN_RF_BTIPT_VERSION;
#endif
    uint8_t btipt_xo_setting_length = PARAM_LEN_RF_BTIPT_XO_SETTING;
    uint8_t btipt_gain_setting_length = PARAM_LEN_RF_BTIPT_GAIN_SETTING;

//#if  defined(CFG_BT_EMB)
    uint8_t rssi_length = PARAM_LEN_RSSI_THR;
//#endif //CFG_BT_EMB

    // Initialize the RF driver API structure
    api->reg_rd = rf_btipt_reg_rd;
    api->reg_wr = rf_btipt_reg_wr;
    api->txpwr_dbm_get = rf_txpwr_dbm_get;
    api->txpwr_min = BTIPT_POWER_MIN;
    api->txpwr_max = BTIPT_POWER_MAX;
    api->sleep = rf_sleep;
    api->reset = rf_reset;
    api->rssi_convert = rf_rssi_convert;
    api->txpwr_cs_get = rf_txpwr_cs_get;
#if defined(CFG_BLE_EMB)
    api->force_agc_enable = rf_force_agc_enable;
#endif //CFG_BLE_EMB


#if !defined(RP_HWSIM_BYPASS)
    if (rwip_param.get(PARAM_ID_RF_BTIPT_VERSION, &btipt_version_length, (uint8_t *)&btipt_version_chip) != PARAM_OK)
    {
        btipt_version_chip =  RF_BTIPT_BTDM001T28ES3;// RF_BTIPT_BTDM001G28ES1;
    }
//    btipt_version_chip = RF_BTIPT_BTDM001T28ES2;
    //Set Pointer
    if (btipt_version_chip == RF_BTIPT_BTDM001T28ES1)
    {
        btipt_prog_mem_ptr = RF_BTDM001T28ES1_PROG_MEM_PTR;
        btipt_data_mem_ptr = RF_BTDM001T28ES1_DATA_MEM_BUF;
#if !defined(CFG_GAIA)
        btipt_data_step = RF_BTDM001T28ES1_DATA_STEP;
        btipt_prog_step = RF_BTDM001T28ES1_PROG_STEP ;
#endif // !defined(CFG_GAIA)
        btipt_subversion = 0x0;
        btipt_cmd_buf_base_add = 0x1400;
        btipt_rsp_buf_base_add = 0x140A;
    }
    else if (btipt_version_chip == RF_BTIPT_BTDM001G28ES1)
    {
        btipt_prog_mem_ptr = RF_BTDM001G28ES1_PROG_MEM_PTR;
        btipt_data_mem_ptr = RF_BTDM001G28ES1_DATA_MEM_BUF;
#if !defined(CFG_GAIA)
        btipt_prog_step = RF_BTDM001G28ES1_PROG_STEP;
        btipt_data_step = RF_BTDM001G28ES1_DATA_STEP;
#endif // !defined(CFG_GAIA)
        btipt_subversion = 0x1;
        btipt_cmd_buf_base_add = 0x1800;
        btipt_rsp_buf_base_add = 0x180A;
    }
    else if (btipt_version_chip == RF_BTIPT_BTDM001T28ES2)
    {
        btipt_prog_mem_ptr = RF_BTDM001T28ES2_PROG_MEM_PTR;
        btipt_data_mem_ptr = RF_BTDM001T28ES2_DATA_MEM_BUF;
#if !defined(CFG_GAIA)
        btipt_prog_step = RF_BTDM001T28ES2_PROG_STEP;
        btipt_data_step = RF_BTDM001T28ES2_DATA_STEP;
#endif // !defined(CFG_GAIA)
        btipt_subversion = 0x2;
        btipt_cmd_buf_base_add = 0x1800;
        btipt_rsp_buf_base_add = 0x180A;
    }
    else if (btipt_version_chip == RF_BTIPT_BTDM001T28ES3)
    {
        btipt_prog_mem_ptr = RF_BTDM001T28ES3_PROG_MEM_PTR;
        btipt_data_mem_ptr = RF_BTDM001T28ES3_DATA_MEM_BUF;
#if !defined(CFG_GAIA)
        btipt_prog_step = RF_BTDM001T28ES3_PROG_STEP;
        btipt_data_step = RF_BTDM001T28ES3_DATA_STEP;
#endif // !defined(CFG_GAIA)
        btipt_subversion = 0x2;//TODO: Should be changed later
        btipt_cmd_buf_base_add = 0x3000;
        btipt_rsp_buf_base_add = 0x500A;
    }
    else
    {
        ASSERT_ERR(0);
    }

#else
    // Stick to Version T28ES1
    //btipt_version_chip = RF_BTIPT_BTDM001T28ES1;
    //btipt_prog_mem_ptr = RF_BTDM001T28ES1_PROG_MEM_PTR;
    //btipt_data_mem_ptr = RF_BTDM001T28ES1_DATA_MEM_BUF;
    //#if !defined(CFG_GAIA)
    //btipt_data_step = RF_BTDM001T28ES1_DATA_STEP;
    //btipt_prog_step = RF_BTDM001T28ES1_PROG_STEP ;
    //#endif //!defined(CFG_GAIA)
    btipt_subversion = 0x0;
    btipt_cmd_buf_base_add = 0x1400;
    btipt_rsp_buf_base_add = 0x140A;
#endif


#if  defined(CFG_BT_EMB)
    api->txpwr_dec = rf_txpwr_dec;
    api->txpwr_inc = rf_txpwr_inc;
    api->txpwr_max_set = txpwr_max_set;
#endif //CFG_BT_EMB

    // Initialize the RSSI thresholds (high, low, interference)
    // The upper threshold level is 20 dB above the lower threshold level to an accuracy of +-6 dB
    // These are 'real' signed values in dBm
    if (rwip_param.get(PARAM_ID_RSSI_HIGH_THR, &rssi_length, (uint8_t *)&api->rssi_high_thr)   != PARAM_OK)
    {
        api->rssi_high_thr   = (int8_t)BTIPT_RSSI_40dB_THRHLD;
    }
    if (rwip_param.get(PARAM_ID_RSSI_LOW_THR, &rssi_length, (uint8_t *)&api->rssi_low_thr)    != PARAM_OK)
    {
        api->rssi_low_thr    = (int8_t)BTIPT_RSSI_60dB_THRHLD;
    }
    if (rwip_param.get(PARAM_ID_RSSI_INTERF_THR, &rssi_length, (uint8_t *)&api->rssi_interf_thr) != PARAM_OK)
    {
        api->rssi_interf_thr = (int8_t)BTIPT_RSSI_70dB_THRHLD;
    }

    /* IP RADIOCNTL0 */
    ip_radiocntl0_pack(/*uint16_t spiptr*/ (EM_RF_SW_SPI_OFFSET >> 2),
                                           /*uint8_t  spicfg*/   1,  // Do not sent header throug SPI !
                                           /*uint8_t  spifreq*/  0,
                                           /*uint8_t  spigo*/    0);

    /* IP RADIOCNTL1 */
    ip_radiocntl1_pack(/*uint8_t  forceagcen*/      1,
            /*uint8_t  forceiq*/         0,
            /*uint8_t  rxdnsl*/          0,
            /*uint8_t  txdnsl*/          0,
            /*uint16_t forceagclength*/  0x000,
            /*uint8_t  syncpulsemode*/   0,
            /*uint8_t  syncpulsesrc*/    1,
            /*uint8_t  dpcorren*/        0,
            /*uint8_t  jefselect*/       0,
            /*uint8_t  xrfsel*/          0x05,
            /*uint8_t  subversion*/      btipt_subversion);
#if 1

#if !defined(RP_HWSIM_BYPASS)

#if !defined(CFG_GAIA)

    // Read BTIPT FW program memory depth
    flash_status = flash_read(FLASH_TYPE_NUMONYX_M25P128, RF_FLASH_BTIPT_PROG_MEM, 2, (uint8_t *)&btipt_prog_mem_length, NULL);
    ASSERT_ERR(flash_status == 0);
    ASSERT_ERR(btipt_prog_mem_length != 0);

    // Read BTIPT FW data memory depth
    flash_status = flash_read(FLASH_TYPE_NUMONYX_M25P128, RF_FLASH_BTIPT_DATA_MEM, 2, (uint8_t *)&btipt_data_mem_length, NULL);
    ASSERT_ERR(flash_status == 0);
    ASSERT_ERR(btipt_data_mem_length != 0);

#endif // defined(CFG_GAIA)

    if (btipt_version_chip == RF_BTIPT_BTDM001T28ES3)
    {
        rf_btipt_reg_wr(0x10A9, 0xF8);
        //Select program memory
        rf_btipt_reg_wr(btipt_prog_mem_ptr + 1, 0x00000001);
    }

    //Write start address of the memory image
    rf_btipt_reg_wr(btipt_prog_mem_ptr, 0x00000000);

    //Select program memory
    if (btipt_version_chip != RF_BTIPT_BTDM001T28ES3)
    {
        rf_btipt_reg_wr(btipt_prog_mem_ptr + 1, 0x0000001D);
    }

#if defined(CFG_GAIA)
    {
        const char *btipt_ver_str[] = {"T28ES1", "G28ES1", "T28ES2", "T28ES3"};
        fileio_rf_struct *volatile m_rf;
        uint32_t *volatile m_chk;
        uint32_t addr, sz, csum, csum_ref;

        // Load Program RAM
        m_rf = (fileio_rf_struct *)(BRAM_BASE_ADDRESS + BRAM_RF_PROG_OFFSET);

        {
            // Workaround initial BRAM read issue
            uint32_t bram_val;
            lcd_printf(LCD_LINE_0, "Waiting on BRAM");
            do
            {
                bram_val = *(uint32_t *)m_rf;
            }
            while (bram_val == 0);
        }

        m_chk = (uint32_t *)m_rf;
        csum = 0;

        lcd_printf(LCD_LINE_0, "LOADING RF PMEM ");
        for (sz = 0; m_rf->ld_len ; m_rf++)
        {
            addr = (uint32_t)m_rf->addr[0] + ((uint32_t)m_rf->addr[1] << 8) + ((uint32_t)m_rf->addr[2] << 16);
            rf_btipt_reg_burst_wr(addr, m_rf->ld_len, &m_rf->ld[0]);
            sz += m_rf->ld_len;
        }

        // calc checksum
        while ((void *)m_chk < (void *)m_rf)
            csum += *m_chk++;

        csum_ref = *(uint32_t *)(BRAM_BASE_ADDRESS + BRAM_RF_PROG_CSUM_OFFSET);
        ASSERT_INFO(csum == csum_ref, csum, csum_ref);

        lcd_printf(LCD_LINE_1, "%s sz %d", btipt_ver_str[btipt_version_chip], sz);

        //Write start address of the memory image
        rf_btipt_reg_wr(btipt_data_mem_ptr, 0x00000000);

        // Load Data RAM
        m_rf = (fileio_rf_struct *)((uint32_t)BRAM_BASE_ADDRESS + BRAM_RF_DATA_OFFSET);

        {
            // Workaround initial BRAM read issue
            uint32_t bram_val;
            lcd_printf(LCD_LINE_0, "Waiting on BRAM");
            do
            {
                bram_val = *(uint32_t *)m_rf;
            }
            while (bram_val == 0);
        }

        m_chk = (uint32_t *)m_rf;
        csum = 0;

        lcd_printf(LCD_LINE_0, "LOADING RF DMEM ");
        for (sz = 0; m_rf->ld_len ; m_rf++)
        {
            addr = (uint32_t)m_rf->addr[0] + ((uint32_t)m_rf->addr[1] << 8) + ((uint32_t)m_rf->addr[2] << 16);
            rf_btipt_reg_burst_wr(addr, m_rf->ld_len, &m_rf->ld[0]);
            sz += m_rf->ld_len;
        }

        // calc checksum
        while ((void *)m_chk < (void *)m_rf)
            csum += *m_chk++;

        csum_ref = *(uint32_t *)(BRAM_BASE_ADDRESS + BRAM_RF_DATA_CSUM_OFFSET);
        ASSERT_INFO(csum == csum_ref, csum, csum_ref);

        lcd_printf(LCD_LINE_1, "%s sz %d", btipt_ver_str[btipt_version_chip], sz);
    }
#else // !defined(CFG_GAIA)

    //Set Pointer
    if (btipt_version_chip == RF_BTIPT_BTDM001T28ES1)
    {
        // Load Program RAM
        lcd_printf(LCD_LINE_0, "LOADING RF PMEM ");
        lcd_printf(LCD_LINE_1, "T28ES1 sz %d", btipt_prog_mem_length, btipt_prog_step);
        for (int i = 0; i < btipt_prog_mem_length; i++)
        {
            flash_status = flash_read(FLASH_TYPE_NUMONYX_M25P128, RF_FLASH_BTIPT_PROG_MEM + 2 + i * (btipt_prog_step + RF_BTDM001T28ES1_ADDRESS), (btipt_prog_step + RF_BTDM001T28ES1_ADDRESS), (uint8_t *)&btipt_flash_read_buf[0], NULL);
            ASSERT_ERR(flash_status == 0);
            rf_btipt_reg_burst_wr(*(uint16_t *)&btipt_flash_read_buf[0], btipt_prog_step, &btipt_flash_read_buf[RF_BTDM001T28ES1_ADDRESS]);
        }
        //Write start address of the memory image
        rf_btipt_reg_wr(btipt_data_mem_ptr, 0x00000000);
        // Load Data RAM
        lcd_printf(LCD_LINE_0, "LOADING RF DMEM ");
        lcd_printf(LCD_LINE_1, "T28ES1 sz %d", btipt_data_mem_length, btipt_data_step);
        for (int i = 0; i < btipt_data_mem_length; i++)
        {
            flash_status = flash_read(FLASH_TYPE_NUMONYX_M25P128, RF_FLASH_BTIPT_DATA_MEM + 2 + i * (btipt_data_step + RF_BTDM001T28ES1_ADDRESS), (btipt_data_step + RF_BTDM001T28ES1_ADDRESS), (uint8_t *)&btipt_flash_read_buf[0], NULL);
            ASSERT_ERR(flash_status == 0);
            rf_btipt_reg_burst_wr(*(uint16_t *)&btipt_flash_read_buf[0], btipt_data_step, (uint8_t *)&btipt_flash_read_buf[RF_BTDM001T28ES1_ADDRESS]);

        }
    }
    else if (btipt_version_chip == RF_BTIPT_BTDM001G28ES1)
    {
        // Load Program RAM
        lcd_printf(LCD_LINE_0, "LOADING RF PMEM ");
        lcd_printf(LCD_LINE_1, "G28ES1 sz %d", btipt_prog_mem_length, btipt_prog_step);
        for (int i = 0; i < btipt_prog_mem_length; i++)
        {
            flash_status = flash_read(FLASH_TYPE_NUMONYX_M25P128, RF_FLASH_BTIPT_PROG_MEM + 2 + i * (btipt_prog_step + RF_BTDM001G28ES1_ADDRESS), (btipt_prog_step + RF_BTDM001G28ES1_ADDRESS), (uint8_t *)&btipt_flash_read_buf[0], NULL);
            ASSERT_ERR(flash_status == 0);
            rf_btipt_reg_burst_wr(*(uint32_t *)&btipt_flash_read_buf[0], (btipt_prog_step - 2), &btipt_flash_read_buf[RF_BTDM001G28ES1_ADDRESS]);
        }
        //Write start address of the memory image
        rf_btipt_reg_wr(btipt_data_mem_ptr, 0x00000000);
        // Load Data RAM
        lcd_printf(LCD_LINE_0, "LOADING RF DMEM ");
        lcd_printf(LCD_LINE_1, "G28ES1 sz %d", btipt_data_mem_length, btipt_data_step);
        for (int i = 0; i < btipt_data_mem_length; i++)
        {
            flash_status = flash_read(FLASH_TYPE_NUMONYX_M25P128, RF_FLASH_BTIPT_DATA_MEM + 2 + i * (btipt_data_step + RF_BTDM001G28ES1_ADDRESS), (btipt_data_step + RF_BTDM001G28ES1_ADDRESS), (uint8_t *)&btipt_flash_read_buf[0], NULL);
            ASSERT_ERR(flash_status == 0);
            rf_btipt_reg_burst_wr(*(uint32_t *)&btipt_flash_read_buf[0], btipt_data_step, (uint8_t *)&btipt_flash_read_buf[RF_BTDM001G28ES1_ADDRESS]);

        }
    }
    else if (btipt_version_chip == RF_BTIPT_BTDM001T28ES2)
    {
        // Load Program RAM
        lcd_printf(LCD_LINE_0, "LOADING RF PMEM ");
        lcd_printf(LCD_LINE_1, "T28ES2 sz %d", btipt_prog_mem_length, btipt_prog_step);
        for (int i = 0; i < btipt_prog_mem_length; i++)
        {
            flash_status = flash_read(FLASH_TYPE_NUMONYX_M25P128, RF_FLASH_BTIPT_PROG_MEM + 2 + i * (btipt_prog_step + RF_BTDM001G28ES1_ADDRESS), (btipt_prog_step + RF_BTDM001G28ES1_ADDRESS), (uint8_t *)&btipt_flash_read_buf[0], NULL);
            ASSERT_ERR(flash_status == 0);
            rf_btipt_reg_burst_wr(*(uint32_t *)&btipt_flash_read_buf[0], btipt_prog_step, &btipt_flash_read_buf[RF_BTDM001G28ES1_ADDRESS]);
        }
        //Write start address of the memory image
        rf_btipt_reg_wr(btipt_data_mem_ptr, 0x00000000);
        // Load Data RAM
        lcd_printf(LCD_LINE_0, "LOADING RF DMEM ");
        lcd_printf(LCD_LINE_1, "T28ES2 sz %d", btipt_data_mem_length, btipt_data_step);
        for (int i = 0; i < btipt_data_mem_length; i++)
        {
            flash_status = flash_read(FLASH_TYPE_NUMONYX_M25P128, RF_FLASH_BTIPT_DATA_MEM + 2 + i * (btipt_data_step + RF_BTDM001T28ES2_ADDRESS), (btipt_data_step + RF_BTDM001T28ES2_ADDRESS), (uint8_t *)&btipt_flash_read_buf[0], NULL);
            ASSERT_ERR(flash_status == 0);
            rf_btipt_reg_burst_wr(*(uint32_t *)&btipt_flash_read_buf[0], btipt_data_step, (uint8_t *)&btipt_flash_read_buf[RF_BTDM001T28ES2_ADDRESS]);

        }
    }
    else if (btipt_version_chip == RF_BTIPT_BTDM001T28ES3)
    {
        // Load Program RAM
        lcd_printf(LCD_LINE_0, "LOADING RF PMEM ");
        lcd_printf(LCD_LINE_1, "T28ES3 sz %d", btipt_prog_mem_length, btipt_prog_step);
        for (int i = 0; i < btipt_prog_mem_length; i++)
        {
            flash_status = flash_read(FLASH_TYPE_NUMONYX_M25P128, RF_FLASH_BTIPT_PROG_MEM + 2 + i * (btipt_prog_step + RF_BTDM001T28ES3_ADDRESS), (btipt_prog_step + RF_BTDM001T28ES3_ADDRESS), (uint8_t *)&btipt_flash_read_buf[0], NULL);
            ASSERT_ERR(flash_status == 0);
            rf_btipt_reg_burst_wr(*(uint32_t *)&btipt_flash_read_buf[0], btipt_prog_step, &btipt_flash_read_buf[RF_BTDM001T28ES3_ADDRESS]);
        }
        //Write start address of the memory image
        rf_btipt_reg_wr(btipt_data_mem_ptr, 0x00000000);
        // Load Data RAM
        lcd_printf(LCD_LINE_0, "LOADING RF DMEM ");
        lcd_printf(LCD_LINE_1, "T28ES3 sz %d", btipt_data_mem_length, btipt_data_step);
        for (int i = 0; i < btipt_data_mem_length; i++)
        {
            flash_status = flash_read(FLASH_TYPE_NUMONYX_M25P128, RF_FLASH_BTIPT_DATA_MEM + 2 + i * (btipt_data_step + RF_BTDM001T28ES3_ADDRESS), (btipt_data_step + RF_BTDM001T28ES3_ADDRESS), (uint8_t *)&btipt_flash_read_buf[0], NULL);
            ASSERT_ERR(flash_status == 0);
            rf_btipt_reg_burst_wr(*(uint32_t *)&btipt_flash_read_buf[0], btipt_data_step, (uint8_t *)&btipt_flash_read_buf[RF_BTDM001T28ES3_ADDRESS]);
        }
    }
    else
    {
        ASSERT_ERR(0);
    }

#endif // !defined(CFG_GAIA)

    /*
     * Once the complete firmware image and the Factory Settings have been transferred, the baseband
     * controller releases the processor from its frozen state by writing 0x00 to SPI address 0xF 0001,
     * followed by writing:
     */
    rf_btipt_reg_wr((btipt_prog_mem_ptr + 0x1), 0x0000);

    if (btipt_version_chip == RF_BTIPT_BTDM001T28ES2)
    {
        rf_btipt_reg_wr(0x10BE, 0x01);
    }
    else if (btipt_version_chip == RF_BTIPT_BTDM001T28ES3)
    {
        rf_btipt_reg_wr(0x10C4, 0x0001);
    }
    else
    {
        rf_btipt_reg_wr(0x10B7, 0x01);
    }
#endif // RP_HWSIM_BYPASS

    // enable sys_clk_out
    plf_rf_sysclk_cntl_rf_sysclk_en(1);

#if !defined(RP_HWSIM_BYPASS)
    // Wait for "some time" for the RF to be ready
    volatile int wait = 0;
    for (int i = 0; i < 800000; i++)
    {
        wait++;
    }
#endif

    /* *************************************************************************************** */
    /* Get the RF version                                                                      */
    /* *************************************************************************************** */
    // send GetVersion command req
    rf_btipt_reg_wr(btipt_cmd_buf_base_add, 0xc1a0);
#if !defined(RP_HWSIM_BYPASS)
    btipt_version[0] = (uint16_t)rf_btipt_reg_rd(btipt_rsp_buf_base_add);
    btipt_version[1] = (uint16_t)rf_btipt_reg_rd(btipt_rsp_buf_base_add + 0x1);
    btipt_version[2] = (uint16_t)rf_btipt_reg_rd(btipt_rsp_buf_base_add + 0x2);
    btipt_version[3] = (uint16_t)rf_btipt_reg_rd(btipt_rsp_buf_base_add + 0x3);
    btipt_fw_minor = (uint8_t)(btipt_version[2] & 0x00FF);
#endif

#if !defined(RP_HWSIM_BYPASS)
    if (btipt_version_chip == RF_BTIPT_BTDM001T28ES2)
    {
        ASSERT_INFO(btipt_version[0] == 0x01A3, btipt_version[0], 0x01A3);
        ASSERT_INFO(btipt_version[1] == 0x0300, btipt_version[1], 0x0300);
    }
    else if (btipt_version_chip == RF_BTIPT_BTDM001T28ES3)
    {
        ASSERT_INFO(btipt_version[0] == 0x01A3, btipt_version[0], 0x01A3);
        ASSERT_INFO(btipt_version[1] == 0x0500, btipt_version[1], 0x0500);
    }
    else
    {
        ASSERT_INFO(btipt_version[0] == 0x01A3, btipt_version[0], 0x01A3);
        ASSERT_INFO(btipt_version[1] == 0x0200, btipt_version[1], 0x0200);
    }
#endif


    /* *************************************************************************************** */
    /* Initialize the trimming load capacitor CWB_GEN_SetXOLoadCap  -> 1 structure of 1 word*/
    /* *************************************************************************************** */
    /* setXOloadcap Sequence -> Write Address @0x1400 / 1 word
                      -> Next Pointer= 0x0000
                      -> Header (RRQ=1/XRQ=1/CWB_TRX_SetPacketType=0x013/LEN=1) = 0xC131
                      -> DW1[15:5] -> Not used
                      -> DW1[4:0] -> Cload  */
    if ((rwip_param.get(PARAM_ID_RF_BTIPT_XO_SETTING, &btipt_xo_setting_length, (uint8_t *)&btipt_xo_setting) != PARAM_OK))
    {
        btipt_xo_setting = 0x007F;
    }
    rf_btipt_reg_wr_full(btipt_cmd_buf_base_add, 0xc131, btipt_xo_setting & 0xFF);

    /* *************************************************************************************** */
    /* Initialize the Gain Option */
    /* *************************************************************************************** */
    /*SetGainOption cmd_id: 0x24
                    command data: 1 word
                    If a value is present in NVDS this shall be written after download of the firmware to the BTIPT
                    0x1800 = 0xC241
                    0x1801 = Value */
    if ((rwip_param.get(PARAM_ID_RF_BTIPT_GAIN_SETTING, &btipt_gain_setting_length, (uint8_t *)&btipt_gain_setting) == PARAM_OK))
    {
        rf_btipt_reg_wr_full(btipt_cmd_buf_base_add, 0xC241, btipt_gain_setting);
    }



    /// Initialize Exchange Memory
    rf_em_init();

    /* BLE RADIOCNTL0 */
    ip_radiocntl0_pack(
        /*uint16_t spiptr*/ (EM_RF_SW_SPI_OFFSET >> 2),
        /*uint8_t  spicfg*/   0,
        /*uint8_t  spifreq*/  0,
        /*uint8_t  spigo*/    0);

#if defined(CFG_BLE_EMB)
    /* BLE RADIOCNTL2 */
    ble_radiocntl2_pack(
        /*uint8_t  lrsynccompmode*/ 0x0,
        /*uint8_t  rxcitermbypass*/ 0x1,
        /*uint8_t  lrvtbflush*/     0x0,
        /*uint8_t  phymsk*/         0x1, // mark that Coded phy are supported
        /*uint8_t  lrsyncerr*/      0,
        /*uint8_t  syncerr*/        0,
        /*uint16_t freqtableptr*/ (EM_FT_OFFSET >> 2));

    /* BLE RADIOCNTL3 */
    ble_radiocntl3_pack(
        /*uint8_t rxrate3cfg*/    0x0,
        /*uint8_t rxrate2cfg*/    0x0,
        /*uint8_t rxrate1cfg*/    0x1,
        /*uint8_t rxrate0cfg*/    0x0,
        /*uint8_t getrssidelay*/  0x0,
        /*uint8_t rxsyncrouting*/ 0x1,
        /*uint8_t rxvalidbeh*/    0x2,
        /*uint8_t txrate3cfg*/    0x2,
        /*uint8_t txrate2cfg*/    0x3,
        /*uint8_t txrate1cfg*/    0x1,
        /*uint8_t txrate0cfg*/    0x0,
#if !defined(CFG_GAIA)
        /*uint8_t txvalidbeh*/    0x3
#else
        /*uint8_t txvalidbeh*/    0x2
#endif
    );// FPGA V2


    // ------------ Tx/Rx Power up/down and Timings -------------------
#if !defined(RP_HWSIM_BYPASS)
    switch (btipt_fw_minor)
    {
    case (0x13):
    case (0x14):
        // 1Mbps - R19 Optimized path delay
        ble_txpwrup_1m = 39;
        ble_txpath_1m  = 16;
        ble_txpwrdn_1m = 16;

        ble_rxpwrup_1m = 55;
        ble_rxpath_1m  = 14;

        // 2Mbps - R19 Optimized path delay
        ble_txpwrup_2m = 39;
        ble_txpath_2m  = 16;
        ble_txpwrdn_2m = 16;

        ble_rxpwrup_2m = 51;
        ble_rxpath_2m  = 6;
        break;
    case (0x1A):
        // 1Mbps - R26 Optimized path delay
        ble_txpwrup_1m = 46 + 10 - 4;
        ble_txpath_1m  = 21;
        ble_txpwrdn_1m = 16;

        ble_rxpwrup_1m = 40 + 12;
        ble_rxpath_1m  = 6;

        // 2Mbps - R26 Optimized path delay
        ble_txpwrup_2m = 46 + 10 - 4;
        ble_txpath_2m  = 12;
        ble_txpwrdn_2m = 16;

        ble_rxpwrup_2m = 40 + 12;
        ble_rxpath_2m  = 3;
        break;
    case (0x00):
#if !defined(CFG_GAIA)
        // 1Mbps - R26 Optimized path delay
        ble_txpwrup_1m = 33 + 10 - 4;
        ble_txpath_1m  = 22; // 7; Expected Value (RF only)
        ble_txpwrdn_1m = 21;

        ble_rxpwrup_1m = 40 + 12; // 31+12; previous value
        ble_rxpath_1m  = 21; // 9; TODO Expected value

        // 2Mbps - R26 Optimized path delay
        ble_txpwrup_2m = 33 + 10 - 4;
        ble_txpath_2m  = 11; // TODO - To be verified
        ble_txpwrdn_2m = 21;

        ble_rxpwrup_2m = 40 + 12; // 31+12; previous value
        ble_rxpath_2m  = 22;
#else
        // Optimized for R1.0.024
        // 1Mbps
        ble_txpwrup_1m = 41 + 7 - 4;
        ble_txpath_1m  = 15;
        ble_txpwrdn_1m = 24;

        ble_rxpwrup_1m = 40 + 9;
        ble_rxpath_1m  = 18;

        // 2Mbps
        ble_txpwrup_2m = 46 + 7 - 4;
        ble_txpath_2m  = 8;
        ble_txpwrdn_2m = 14;

        ble_rxpwrup_2m = 40 + 9;
        ble_rxpath_2m  = 10;
#endif
        break;
    default:
        // 1Mbps - Non Optimized path delay
        ble_txpwrup_1m = 94;
        ble_txpath_1m  = 15;
        ble_txpwrdn_1m = 15;

        ble_rxpwrup_1m = 136;
        ble_rxpath_1m  = 14;
        // 2Mbps - Non Optimized path delay
        ble_txpwrup_2m = 94;
        ble_txpath_2m  = 15;
        ble_txpwrdn_2m = 15;

        ble_rxpwrup_2m = 136;
        ble_rxpath_2m  = 6;
        break;
    }

#else

    // 1Mbps - VAlue for FPGA simulation
    ble_txpwrup_1m = 34;
    ble_txpath_1m  = 16;
    ble_txpwrdn_1m = 30;

    ble_rxpwrup_1m = 55;
    ble_rxpath_1m  = 14;

    // 2Mbps - R19 Optimized path delay
    ble_txpwrup_2m = 34;
    ble_txpath_2m  = 16;
    ble_txpwrdn_2m = 30;

    ble_rxpwrup_2m = 51;
    ble_rxpath_2m  = 6;
#endif


    /* BLE RADIOPWRUPDN0 */
    ble_radiopwrupdn0_pack(
        /*uint8_t syncposition0*/ 0,
        /*uint8_t rxpwrup0*/      ble_rxpwrup_1m,// 136,
        /*uint8_t txpwrdn0*/      ble_txpwrdn_1m,//11,
        /*uint8_t txpwrup0*/      ble_txpwrup_1m);//96);

    /* BLE RADIOTXRXTIM0 */
    ble_radiotxrxtim0_pack(
        /*uint8_t rfrxtmda0*/     ble_rxpath_1m,//13,
        /*uint8_t rxpathdly0*/    ble_rxpath_1m,//13,
        /*uint8_t txpathdly0*/    ble_txpath_1m);//11);


//

    /* BLE RADIOPWRUPDN1 */
    ble_radiopwrupdn1_pack(
        /*uint8_t syncposition1*/ 0,
        /*uint8_t rxpwrup1*/      ble_rxpwrup_2m,//136,
        /*uint8_t txpwrdn1*/      ble_txpwrdn_2m,//7,
        /*uint8_t txpwrup1*/      ble_txpwrup_2m);//99);

    /* BLE RADIOTXRXTIM1 */
    ble_radiotxrxtim1_pack(
        /*uint8_t rfrxtmda1*/     ble_rxpath_2m,//6,
        /*uint8_t rxpathdly1*/    ble_rxpath_2m,//6,
        /*uint8_t txpathdly1*/    ble_txpath_2m);//7);

    // LR 125Kbps
    /* BLE RADIOPWRUPDN2 */
    ble_radiopwrupdn2_pack(
        /*uint8_t syncposition2*/ 0,
        /*uint8_t rxpwrup2*/      85,
        /*uint8_t txpwrdn2*/      15,
        /*uint8_t txpwrup2*/      82);

    /* BLE RADIOTXRXTIM2 */
    ble_radiotxrxtim2_pack(
        /*uint8_t rxflushpathdly2*/ 0x10,
        /*uint8_t rfrxtmda2*/       41,
        /*uint8_t rxpathdly2*/      40,
        /*uint8_t txpathdly2*/      9);

    //LR 500Kbps
    /* BLE RADIOPWRUPDN3 */
    ble_radiopwrupdn3_pack(
        /*uint8_t txpwrdn3*/      15,
        /*uint8_t txpwrup3*/      82);

    /* BLE RADIOTXRXTIM3 */
    ble_radiotxrxtim3_pack(
        /*uint8_t rxflushpathdly3*/ 0x10,
        /*uint8_t rfrxtmda3*/       41,
        /*uint8_t txpathdly3*/      9);

#endif // defined CFG_BLE_EMB

#if  defined(CFG_BT_EMB)

    /* EDRCNTL */
    bt_rwbtcntl_nwinsize_setf(0);

    bt_edrcntl_pack(/*uint8_t txrateswinstant*/  1,
            /*uint8_t edrbcast*/         0,
            /*uint8_t rxswap*/           0,
            /*uint8_t txswap*/           0,
#if !defined(CFG_GAIA)
            /*uint8_t guardbandtime*/    0x1,
#else
            /*uint8_t guardbandtime*/    0x5,
#endif
            /*uint8_t rxguarddsb*/       0,
            /*uint8_t gbtxqualgendsb*/   1,
            /*uint8_t rxgrdtimeout*/     0x12);

    /* BT RADIOCNTL2 */
    bt_radiocntl2_pack(/*uint8_t trailergatingval*/  4,
            /*uint8_t syncerr*/           0,
            /*uint16_t freqtableptr*/ (EM_FT_OFFSET >> 2));

    /* BT RADIOCNTL3 */
    bt_radiocntl3_pack(/*uint8_t rxrate2cfg*/    0x3,
            /*uint8_t rxrate1cfg*/    0x2,
            /*uint8_t rxrate0cfg*/    0x0,
            /*uint8_t getrssidelay*/  0x0,
            /*uint8_t rxserparif*/    0x1,
            /*uint8_t rxsyncrouting*/ 0x1,
            /*uint8_t rxvalidbeh*/    0x2,
            /*uint8_t txrate2cfg*/    0x3,
            /*uint8_t txrate1cfg*/    0x2,
            /*uint8_t txrate0cfg*/    0x0,
            /*uint8_t txserparif*/    0x1,
#if !defined(CFG_GAIA)
            /*uint8_t txvalidbeh*/    0x3
#else
            /*uint8_t txvalidbeh*/    0x2
#endif
                      );

#if !defined(RP_HWSIM_BYPASS)
    switch (btipt_fw_minor)
    {
    case (0x13):
    case (0x14):
        // optimized power up times
        bt_txpwrup = 38;
        bt_txpath  = 15;
        bt_txpwrdn = 16;

        bt_rxpwrup = 60;
        bt_rxpath  = 9;
        break;
    case (0x1A):
        // optimized power up times R26
        //pwr up + spi delay + fifo delay
        bt_txpwrup = 46 + 10 - 4;
        bt_txpath  = 12;
        bt_txpwrdn = 18;

        bt_rxpwrup = 40 + 16;
        bt_rxpath  = 9;
        break;
    case (0x00):
#if !defined(CFG_GAIA)
        // optimized power up times R26
        //pwr up + spi delay + fifo delay
        bt_txpwrup = 31 + 10 - 4;
        bt_txpath  = 22; // 7; // TODO - Expected value (RF only)
        bt_txpwrdn = 21;

        bt_rxpwrup = 42 + 16; // 33+16; Previous value
        bt_rxpath  = 8; // 9; Expected value
#else
        bt_txpwrup = 39 + 7 - 4;
        bt_txpath  = 20;
        bt_txpwrdn = 22;

        bt_rxpwrup = 42 + 12;
        bt_rxpath  = 8;
#endif
        break;
    default:
        // Not optimized power up times
        bt_txpwrup = 93;
        bt_txpath  = 15;
        bt_txpwrdn = 17;

        bt_rxpwrup = 140;
        bt_rxpath  = 9;
        break;
    }
#else
    bt_txpwrup = 38;
    bt_txpath  = 15;
    bt_txpwrdn = 30;

    bt_rxpwrup = 60;
    bt_rxpath  = 9;
#endif


    /* BT RADIOPWRUPDN */
    bt_radiopwrupdn_pack(
        /*uint8_t rxpwrupct*/    bt_rxpwrup,
        /*uint8_t txpwrdnct*/    bt_txpwrdn,
        /*uint8_t txpwrupct*/    bt_txpwrup);

    /* BT RADIOTXRXTIM */
    bt_radiotxrxtim_pack(
        /*uint8_t syncposition*/ 0x0,
        /*uint8_t rxpathdly*/    bt_rxpath,
        /*uint8_t txpathdly*/    bt_txpath);
#endif //CFG_BT_EMB

    /* *************************************************************************************** */
    /* Initialize HW SPI Chains Pointers  */
    /* *************************************************************************************** */
    uint16_t txonptr      = (EM_RF_HW_SPI_OFFSET);
    uint16_t txoffptr     = (EM_RF_HW_SPI_OFFSET + 0x10);
#if defined(CFG_BLE_EMB)
    uint16_t ble_rxonptr  = (EM_RF_HW_SPI_OFFSET + 0x10 + 0x10);
#endif // defined CFG_BLE_EMB
#if  defined(CFG_BT_EMB)
    uint16_t bt_rxonptr   = (EM_RF_HW_SPI_OFFSET + 0x10 + 0x10 + 0x10);
#endif // defined CFG_BT_EMB
    uint16_t rxoffptr     = (EM_RF_HW_SPI_OFFSET + 0x10 + 0x10 + 0x10 + 0x20);
    uint16_t rssiptr      = (EM_RF_HW_SPI_OFFSET + 0x10 + 0x10 + 0x10 + 0x20 + 0x10);
    uint16_t rxlengthptr  = (EM_RF_HW_SPI_OFFSET + 0x10 + 0x10 + 0x10 + 0x20 + 0x10 + 0x10);
    uint16_t rxpkttypptr  = (EM_RF_HW_SPI_OFFSET + 0x10 + 0x10 + 0x10 + 0x20 + 0x10 + 0x10 + 0x10);


#if defined(CFG_BLE_EMB)
    /* TxOn Sequence start pointer */
    ble_spiptrcntl0_txonptr_setf(txonptr >> 2);

    /* TxOff Sequence start pointer */
#if !defined(RP_HWSIM_BYPASS)
    //ble_spiptrcntl0_txoffptr_setf(0x0000);
    ble_spiptrcntl0_txoffptr_setf(txoffptr >> 2);  // TODO - Test of firm TxOff with ES3
#else
    ble_spiptrcntl0_txoffptr_setf(txoffptr >> 2);
#endif // RP_HWSIM_BYPASS)

    /* RxOn Sequence start pointer */
    ble_spiptrcntl1_rxonptr_setf(ble_rxonptr >> 2);

    /* RxOff Sequence start pointer */
    ble_spiptrcntl1_rxoffptr_setf(rxoffptr >> 2);

    /* RSSI Sequence start pointer */
    ble_spiptrcntl2_rssiptr_setf(rssiptr >> 2);

    /* Long Range packet length Sequence start pointer */
    ble_spiptrcntl2_rxlengthptr_setf(0x0000);

    /* Packet Type Sequence start pointer */
    ble_spiptrcntl3_rxpkttypptr_setf(rxpkttypptr >> 2);
#endif // defined CFG_BLE_EMB

#if  defined(CFG_BT_EMB)
    /* TxOn Sequence start pointer */
    bt_spiptrcntl0_txonptr_setf(txonptr >> 2);

    /* TxOff Sequence start pointer */
#if !defined(RP_HWSIM_BYPASS)
    //bt_spiptrcntl0_txoffptr_setf(0x0000);
    bt_spiptrcntl0_txoffptr_setf(txoffptr >> 2);  // TODO - Test of firm TxOff with ES3
#else
    bt_spiptrcntl0_txoffptr_setf(txoffptr >> 2);
#endif // RP_HWSIM_BYPASS)

    /* RxOn Sequence start pointer */
    bt_spiptrcntl1_rxonptr_setf(bt_rxonptr >> 2);

    /* RxOff Sequence start pointer */
    bt_spiptrcntl1_rxoffptr_setf(rxoffptr >> 2);

    /* RSSI Sequence start pointer */
    bt_spiptrcntl2_rssiptr_setf(rssiptr >> 2);

    /* Long Range packet length Sequence start pointer */
    bt_spiptrcntl2_rxlengthptr_setf(rxlengthptr >> 2);

    /* Packet Type Sequence start pointer */
    bt_spiptrcntl3_rxpkttypptr_setf(rxpkttypptr >> 2);
#endif // defined CFG_BT_EMB

    /* *************************************************************************************** */
    /* Initialize HW SPI Tx On Chained list  -> 1 structure of 3 words*/
    /* *************************************************************************************** */
    /*  TxON Sequence -> Write Address @0x1400 / 3 Words
                 -> DW1[15:13] -> LE Modulation type as per CS-TXRATE
                 -> DW1[12:6]  -> Channel as per DM table
                 -> DW1[5:0]   -> CS-TXPWR
                 -> DW2[15:11] -> 0x1D
                 -> DW2[10:0]  -> Length (0x0 at the moment as 0x1D discards it)
                 -> DW3[15:10] -> CTE field length: 0x0
                 -> DW3[9:0]   -> 0xA0 */
    RF_BTIPT_EM_WR(txonptr, 0x0000);
    RF_BTIPT_EM_WR(txonptr + 0x2, 0x0013);
    RF_BTIPT_EM_WR(txonptr + 0x4, btipt_cmd_buf_base_add);
    RF_BTIPT_EM_WR(txonptr + 0x6, 0x8000);
    RF_BTIPT_EM_WR(txonptr + 0x8, 0x0000);
    RF_BTIPT_EM_WR(txonptr + 0xA, 0x0000);
    RF_BTIPT_EM_WR(txonptr + 0xC, 0xB400);
    RF_BTIPT_EM_WR(txonptr + 0xE, 0x0000);

    /* *************************************************************************************** */
    /* Initialize HW SPI Tx Off Chained list */
    /* *************************************************************************************** */
    /* TxOFF Sequence -> Write Address @0x1400 / 0 bytes
                 -> Next Pointer = 0x0000
                 -> Header (RRQ=0/XRQ=0/CWB_TRX_SetRadioAbort=0x8/LEN=0) = 0x0080 */

    RF_BTIPT_EM_WR(txoffptr, 0x0000);
    RF_BTIPT_EM_WR(txoffptr + 0x2, 0x0080);
    RF_BTIPT_EM_WR(txoffptr + 0x4, btipt_cmd_buf_base_add);
    RF_BTIPT_EM_WR(txoffptr + 0x6, 0x8000);
    RF_BTIPT_EM_WR(txoffptr + 0x8, 0x0000);

    /* *************************************************************************************** */
    /* Initialize HW SPI Rx On Chained list -> 1 structure of 4 words */
    /* *************************************************************************************** */
    /* Initialize HW SPI Rx On Chained list  */
    /*  RxON Sequence -> Write Address @0x1400 / 10 bytes
                 -> Next Pointer = 0x0000
                 -> Header (RRQ=1/XRQ=0/CWB_TRX_SetPrepareBurst_RX=0x2/LEN=4) = 0x8024
                 -> DW1[15:13] -> LE Modulation type as per CS-RXRATE
                 -> DW1[12:6]  -> Channel as per DM table
                 -> DW1[5:0]   -> 0x0
                 -> DW2[15:10] -> 0x0
                 -> DW2[9:0]   -> 0xA0
                 -> For BLE only :
                 ->   DW3[15:0]  -> Access Address  /CS-SYNCWORD[31:16]
                 ->   DW4[15:0]  -> Access Address  /CS-SYNCWORD[15:0]
                 -> For BT only :
                 ->   DW3[15:0]  -> Access Address  /CS-SYNCWORD[63:48]
                 ->   DW4[15:0]  -> Access Address  /CS-SYNCWORD[47:32]
                 ->   DW5[15:0]  -> Access Address  /CS-SYNCWORD[31:16]
                 ->   DW6[15:0]  -> Access Address  /CS-SYNCWORD[15:0]  */

#if defined(CFG_BLE_EMB)
    RF_BTIPT_EM_WR(ble_rxonptr,      0x0000);
    RF_BTIPT_EM_WR(ble_rxonptr + 0x2,  0x8024);
    RF_BTIPT_EM_WR(ble_rxonptr + 0x4,  btipt_cmd_buf_base_add);
    RF_BTIPT_EM_WR(ble_rxonptr + 0x6,  0x8000);
    RF_BTIPT_EM_WR(ble_rxonptr + 0x8,  0x0000);
    RF_BTIPT_EM_WR(ble_rxonptr + 0xA,  0x0140);
    RF_BTIPT_EM_WR(ble_rxonptr + 0xC,  0x0000);
    RF_BTIPT_EM_WR(ble_rxonptr + 0xE,  0x0000);
    RF_BTIPT_EM_WR(ble_rxonptr + 0x10, 0x0000);
#endif // defined CFG_BLE_EMB

#if  defined(CFG_BT_EMB)
    RF_BTIPT_EM_WR(bt_rxonptr,      0x0000);
    RF_BTIPT_EM_WR(bt_rxonptr + 0x2,  0x8026);
    RF_BTIPT_EM_WR(bt_rxonptr + 0x4,  btipt_cmd_buf_base_add);
    RF_BTIPT_EM_WR(bt_rxonptr + 0x6,  0x8000);
    RF_BTIPT_EM_WR(bt_rxonptr + 0x8,  0x0000);
    RF_BTIPT_EM_WR(bt_rxonptr + 0xA,  0x0140);
    RF_BTIPT_EM_WR(bt_rxonptr + 0xC,  0x0000);
    RF_BTIPT_EM_WR(bt_rxonptr + 0xE,  0x0000);
    RF_BTIPT_EM_WR(bt_rxonptr + 0x10, 0x0000);
    RF_BTIPT_EM_WR(bt_rxonptr + 0x12, 0x0000);
    RF_BTIPT_EM_WR(bt_rxonptr + 0x14, 0x0000);
#endif // defined CFG_BT_EMB


    /* *************************************************************************************** */
    /* Initialize HW SPI Rx Off Chained list -> 1 structure of 0 words */
    /* *************************************************************************************** */
    /* RxOFF Sequence -> Write Address @0x1400 / 0 bytes
                 -> Next Pointer = 0x0000
                 -> Header (RRQ=0/XRQ=0/CWB_TRX_SetRadioAbort=0x8/LEN=0) = 0x0080 */

    RF_BTIPT_EM_WR(rxoffptr, 0x0000);
    RF_BTIPT_EM_WR(rxoffptr + 0x2, 0x0080);
    RF_BTIPT_EM_WR(rxoffptr + 0x4, btipt_cmd_buf_base_add);
    RF_BTIPT_EM_WR(rxoffptr + 0x6, 0x8000);
    RF_BTIPT_EM_WR(rxoffptr + 0x8, 0x0000);
    RF_BTIPT_EM_WR(rxoffptr + 0xA, 0x0000);

    /* *************************************************************************************** */
    /* Initialize HW SPI RSSI Chained list  -> 1 structure of 1 word*/
    /* *************************************************************************************** */
    /* RSSI Sequence -> Read Address @0x140A / 1 word
                 -> Header / 0x140B
                 -> Next Pointer= 0x0000u
                 -> Header (RRQ=0/XRQ=0/CWB_TRX_SetPrepareBurst_RX=0x2/LEN=1) = 0x0021
                 -> DW1[15:8] -> unused
                 -> DW1[7:9]  -> to RxDESC-RXRSSI */

    RF_BTIPT_EM_WR(rssiptr,     0x0000);
    RF_BTIPT_EM_WR(rssiptr + 0x2, 0x0022);
    RF_BTIPT_EM_WR(rssiptr + 0x4, btipt_rsp_buf_base_add);
    RF_BTIPT_EM_WR(rssiptr + 0x6, 0x0000);
    RF_BTIPT_EM_WR(rssiptr + 0x8, 0x0000);
    RF_BTIPT_EM_WR(rssiptr + 0xA, 0x0000);

    /* *************************************************************************************** */
    /* Initialize HW SPI Rx Length Chained list  -> 1 structure of 1 word NOT USED FOR NOW*/
    /* *************************************************************************************** */
    /* RxLength Sequence -> Write Address @0x140A / 1 word
                      -> Next Pointer= 0x0000
                     -> Header (RRQ=0/XRQ=0/CWB_TRX_SetPayloadLength=0x5/LEN=1) = 0x0051
                      -> DW1[15:11] -> 0x00
                      -> DW1[10:00] -> Payload as decoded by packet controler */

    RF_BTIPT_EM_WR(rxlengthptr,     0x0000);
    RF_BTIPT_EM_WR(rxlengthptr + 0x2, 0x0051);
    RF_BTIPT_EM_WR(rxlengthptr + 0x4, btipt_cmd_buf_base_add);
    RF_BTIPT_EM_WR(rxlengthptr + 0x6, 0x8000);
    RF_BTIPT_EM_WR(rxlengthptr + 0x8, 0x0000);
    RF_BTIPT_EM_WR(rxlengthptr + 0xA, 0x0000);

    /* *************************************************************************************** */
    /* Initialize HW SPI Rx Packet Type Chained list  -> 1 structure of 1 word*/
    /* *************************************************************************************** */
    /* RxPktType Sequence -> Write Address @0x140A / 1 word
                      -> Next Pointer= 0x0000
                     -> Header (RRQ=0/XRQ=0/CWB_TRX_SetPacketType=0x4/LEN=1) = 0x0041
                      -> DW1[15:5] -> 0x000
                      -> DW1[4:0] -> 0x1D if rxdnsl =1 */

    RF_BTIPT_EM_WR(rxpkttypptr,     0x0000);
    RF_BTIPT_EM_WR(rxpkttypptr + 0x2, 0x0041);
    RF_BTIPT_EM_WR(rxpkttypptr + 0x4, btipt_cmd_buf_base_add);
    RF_BTIPT_EM_WR(rxpkttypptr + 0x6, 0x8000);
    RF_BTIPT_EM_WR(rxpkttypptr + 0x8, 0x0000);
    RF_BTIPT_EM_WR(rxpkttypptr + 0xA, 0x0000);
#endif
};
///@} RF_BTIPTAS
