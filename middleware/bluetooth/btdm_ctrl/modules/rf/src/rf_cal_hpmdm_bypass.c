/*****************************************************************************************
*
* @file rf_cal_hpmdm_bypass.c
*
* @brief Calypso radio initialization and specific functions
*
* Copyright (C) RivieraWaves 2009-2021
*
*
*****************************************************************************************/

/*****************************************************************************************
* @addtogroup RF_CAL_HPMDM_BYPASS
* @ingroup RF
* @brief Calypso High Performance Modem Bypass Radio Driver
*
* This is the driver block for Calypso radio
* @{
*****************************************************************************************/

/*****************************************************************************************
 * INCLUDE FILES
 ****************************************************************************************/
#include <string.h>        // for memcpy
#include "co_utils.h"      // common utility definition
#include "co_math.h"       // common math functions
#include "co_endian.h"     // endian definitions
#include "rf.h"            // RF interface
#include "em_map.h"        // RF area mapping

#include "rwip.h"          // for RF API structure definition
#include "reg_ipcore.h"    // DM core registers

#include "plf.h"           // Platform register
#include "flash.h"         // Flash interface
#include "lcd.h"           // DBG interface

#if (BLE_EMB_PRESENT)
    #include "reg_blecore.h"   // ble core registers
    #include "reg_em_ble_cs.h" // control structure definitions
#endif //BLE_EMB_PRESENT

#if (BT_EMB_PRESENT)
    #include "reg_btcore.h"    // bt core registers
    #include "reg_em_bt_cs.h"  // control structure definitions
#endif //BT_EMB_PRESENT

// Calypso register definitions and access functions
#if !defined(RP_HWSIM_BYPASS)

    __STATIC void rf_clp_init_bias(uint32_t chip_id);
    __STATIC void rf_clp_init_icp(uint8_t channel, uint8_t icp);
    __STATIC void rf_clp_init_pll(void);
    __STATIC void rf_clp_init_lopath(void);
    __STATIC void rf_clp_init_rx(void);
    __STATIC void rf_clp_init_tx(void);
    __STATIC void rf_clp_bias_on(void);
    //__STATIC void rf_clp_bias_off(void);
    __STATIC void rf_clp_pll_vco_start_cal(uint8_t vcofc_mode, uint8_t icp_en);
    __STATIC void rf_clp_rc_cal_core(uint8_t trim_max, uint8_t trim_reg_select, uint16_t rc_val_target);
    __STATIC void rf_clp_pll_filter_cal_start(void);
    //__STATIC void rf_clp_load_pattern_gen_tab(void);
    //__STATIC void rf_clp_lo_on_man(void);
    //__STATIC void rf_clp_rx_on_man(uint8_t gain);
    //__STATIC void rf_pll_on_man(uint8_t channel);
    //__STATIC void rf_tx_off_man(void);
    //__STATIC void rf_lo_off_man(void);
    //__STATIC void rf_rx_off_man(void);
    //__STATIC void rf_pll_off_man(void);
    //__STATIC void rf_clp_start_rx_cal(void); TODO
    __STATIC void rf_clp_rx_filter_cal_start(void);
    __STATIC void rf_clp_tx_tanks_cal_start(void);
    __STATIC void rf_clp_tx_filter_cal_start(void);
    __STATIC void rf_clp_txdciqin(uint32_t chip_id);
    __STATIC void rf_clp_txgainphaseiqin(uint32_t chip_id);
    //__STATIC void rf_dump_lut(uint8_t lut_type);

    __STATIC void rf_clp_init_seq(void);
    // TODO: check if thoses function are still needed
    //__STATIC void rf_clp_calib(void);
    //__STATIC void rf_clp_set_txcntl1(void);
    //__STATIC void rf_clp_pw_up(void);
    //__STATIC void rf_clp_txgain_set(void);
    //__STATIC void rf_clp_txdc_cal_seq(void);

#endif //(RP_HWSIM_BYPASS)

// Calypso register definitions and access functions also needed for HW simulation
__STATIC void rf_clp_init_mdmhp(void);
__STATIC void rf_clp_init_mdmhp_bypass(void);
__STATIC void rf_clp_sequencers_init(void);


__STATIC uint32_t rf_clp_reg_rd(uint32_t addr);
__STATIC void rf_clp_reg_wr(uint32_t addr, uint32_t value);

#define REG_CLP_RD                rf_clp_reg_rd
#define REG_CLP_WR                rf_clp_reg_wr

#include "reg_calypso_tc1_bank0.h"          // CALYPSO Bank 0 register
#include "reg_calypso_tc1_bank1.h"          // CALYPSO Bank 1 register
#include "reg_calypso_tc1_bank2.h"          // CALYPSO Bank 2 register
#include "reg_calypso_tc1_bank3.h"          // CALYPSO Bank 3 register
#include "reg_calypso_tc1_bank4.h"          // CALYPSO Bank 4 register
#include "reg_modemhp.h"                    // ModemHP register
/*****************************************************************************************
 * DEFINES
 ****************************************************************************************/

#if !defined(RP_HWSIM_BYPASS)
#define CLP_CHIP_ID_TBL_SIZE        0xF

__STATIC const uint32_t RF_CLP_CHIP_ID[CLP_CHIP_ID_TBL_SIZE] =
{
    [0] = 0,
    [1] = 7200004,
    [2] = 28200002,
    [3] = 28200004,
    [4] = 28200006,
    [5] = 28200012,
    [6] = 52200021,
    [7] = 52200001,
    [8] = 52200028,
    [9] = 52200031,
    [10] = 52200038,
    [11] = 52200039,
    [12] = 52200040,
    [13] = 28200014,
    [14] = 7200001
};

__STATIC const uint8_t RF_CLP_BB_VREF[CLP_CHIP_ID_TBL_SIZE] =
{
    [0] = 10,
    [1] = 10, // 07200004
    [2] = 0,  // 28200002
    [3] = 1,  // 28200004
    [4] = 2,  // 28200006
    [5] = 0,  // 28200012
    [6] = 6,  // 52200021
    [7] = 0,  // 52200001
    [8] = 8,   // 52200028
    [9] = 4,   // 52200031
    [10] = 2,  // 52200038
    [11] = 10, // 52200039
    [12] = 0,  // 52200040
    [13] = 5,  // 28200014
    [14] = 3
}; // 7200001

__STATIC const uint8_t RF_CLP_BB_BIAS_CFG[CLP_CHIP_ID_TBL_SIZE] =
{
    [0] = 9,
    [1] = 9,  // 07200004
    [2] = 9,  // 28200002
    [3] = 11, // 28200004
    [4] = 9,  // 28200006
    [5] = 10, // 28200012
    [6] = 10,  // 52200021
    [7] = 7,   // 52200001
    [8] = 11,  // 52200028
    [9] = 9,   // 52200031
    [10] = 8,  // 52200038
    [11] = 8,  // 52200039
    [12] = 9,  // 52200040
    [13] = 12, // 28200014
    [14] = 9
}; // 7200001

__STATIC const uint8_t RF_CLP_DC_Q_IN[CLP_CHIP_ID_TBL_SIZE] =
{
    [0] = 0,
    [1] = 6,   // 07200004 // Tx DC Q = 6   / 6'b00_0110
    [2] = 16,  // 28200002 // Tx DC Q = 16  / 6'b01_0000
    [3] = 32,  // 28200004 // Tx DC Q = -32 / 6'b10_0000 / 6'b10_0000 / 32
    [4] = 30,  // 28200006 // Tx DC Q = 30  / 6'b01_1110
    [5] = 1,   // 28200012 // Tx DC Q = 1   / 6'b00_0001
    [6] = 15,  // 52200021
    [7] = 1,   // 52200001
    [8] = 37,  // 52200028 // -27
    [9] = 6,   // 52200031
    [10] = 61, // 52200038 // -3
    [11] = 43, // 52200039 // -21 / 01_0101 / 10_1011 / 43
    [12] = 6,  // 52200040
    [13] = 2,  // 28200014 // 2
    [14] = 1
}; // 7200001 // 1

__STATIC const uint8_t RF_CLP_DC_I_IN[CLP_CHIP_ID_TBL_SIZE] =
{
    [0] = 0,
    [1] = 52,   // 07200004 // Tx DC I = -12 / 6'b00_1100 / 6'b11_0100 / 52
    [2] = 8,    // 28200002 // Tx DC I = 8   / 6'b00_1000
    [3] = 30,   // 28200004 // Tx DC I = 30  / 6'b01_1110
    [4] = 32,   // 28200006 // Tx DC I = -32 / 6'b10_0000 / 6'b10_0000 / 32
    [5] = 34,   // 28200012 // Tx DC I = -30 / 6'b01_1110 / 6'b10_0010 / 34
    [6] = 14,   // 52200021
    [7] = 32,   // 52200001 // TX DC I = -32
    [8] = 25,   // 52200028
    [9] = 53,   // 52200031 // -11
    [10] = 5,   // 52200038
    [11] = 58,  // 52200039 // -6 / 00_0110 / 11_1010 / 58
    [12] = 14,  // 52200040
    [13] = 17,  // 28200014 // 17
    [14] = 63
}; // 7200001 // -1 / 11_1111 / 63

__STATIC const uint8_t RF_CLP_TXIQ_GAIN_IN[CLP_CHIP_ID_TBL_SIZE] =
{
    [0] = 0,
    [1] = 0,    // 07200004
    [2] = 0,    // 28200002
    [3] = 0,    // 28200004
    [4] = 233,  // 28200006 // Gain = -23 / 8'b00010111 / 8'b11101001 / 233
    [5] = 254,  // 28200012 // Gain = -2, 256-2 = 254
    [6] = 248,  // 52200021 // Gain = -8
    [7] = 8,    // 52200001
    [8] = 8,    // 52200028
    [9] = 0,    // 52200031
    [10] = 245, // 52200038 // -11
    [11] = 0,   // 52200039
    [12] = 0,   // 52200040
    [13] = 14,  // 28200014 // 14
    [14] = 9
};  //7200001 // 9

__STATIC const uint8_t RF_CLP_TXIQ_PHASE_IN[CLP_CHIP_ID_TBL_SIZE] =
{
    [0] = 0,
    [1] = 0,     // 07200004
    [2] = 0,     // 28200002
    [3] = 0,     // 28200004
    [4] = 207,   // 28200006 // Phase = -49, 256-49 = 207
    [5] = 240,   // 28200012 // Phase = -16, 256-16 = 240
    [6] = 226,   // 52200021 // Phase = -30, 256-30 = 226
    [7] = 220,   // 52200001 // Phase = -36, 256-36 = 220
    [8] = 226,   // 52200028 // -30
    [9] = 14,    // 52200031
    [10] = 239,  // 52200038 // -17
    [11] = 238,  // 52200039 // -18, 256-18 = 238
    [12] = 17,   // 52200040
    [13] = 238,  // 28200014 // -18, 256-18 = 238
    [14] = 227
}; // 7200001 // -29, 256-29 = 227

#endif

//#define CLP_PATTERN_GEN_SIZE 11

//__STATIC const uint8_t RF_PATTERN_GEN_I[CLP_PATTERN_GEN_SIZE] = {
//        [0]  = 0x3F, //-1  // 00 0001 -> 6bits 2's -> 11 1111
//        [1]  = 0x2F, //-17 // 01 0001 -> 6bits 2's -> 10 1111
//        [2]  = 0x24, //-28 // 01 1100 -> 6bits 2's -> 10 0100
//        [3]  = 0x23, //-29 // 01 1101 -> 6bits 2's -> 10 0011
//        [4]  = 0x2A, //-22 // 01 0110 -> 6bits 2's -> 10 1010
//        [5]  = 0x39, //-7  // 00 0111 -> 6bits 2's -> 11 1001
//        [6]  = 0x0A, //10
//        [7]  = 0x17, //23
//        [8]  = 0x1E, //30
//        [9]  = 0x1B, //27
//        [10] = 0x0F  //15
//};
//
//__STATIC const uint8_t RF_PATTERN_GEN_Q[CLP_PATTERN_GEN_SIZE] = {
//        [0]  = 0x22, //-30 // 01 1110 -> 6bits 2's -> 10 0010
//        [1]  = 0x27, //-25 // 01 1001 -> 6bits 2's -> 10 0111
//        [2]  = 0x35, //-11 // 00 1011 -> 6bits 2's -> 11 0101
//        [3]  = 0x06, //6
//        [4]  = 0x15, //21
//        [5]  = 0x1D, //29
//        [6]  = 0x1C, //28
//        [7]  = 0x13, //19
//        [8]  = 0x03, //3
//        [9]  = 0x32, //-14 // 00 1110 -> 6bits 2's -> 11 0010
//        [10] = 0x26  //-26 // 01 1010 -> 6bits 2's -> 10 0110
//};
//
//



//#define CLP_GAIN_TBL_SIZE           0x0F

// Gain table
//__STATIC const uint8_t RF_CLP_RX_GAIN_TBL[CLP_GAIN_TBL_SIZE] = {
//        [0] = 43,
//        [1] = 37,
//        [2] = 31,
//        [3] = 25,
//        [4] = 19,
//        [5] = 13,
//        [6] = 7,
//        [7] = 1};

// EM RF SPI address
#define RF_EM_SPI_ADRESS        (EM_BASE_ADDR + EM_RF_SW_SPI_OFFSET)

#define CLP_SPIRD                   0x00
#define CLP_SPIWR                   0x80
#define CLP_RFPLL_TBL_SIZE          0x50
#define CLP_PWR_TBL_SIZE            0x0F

/* The offset value given below is the offset to add to the frequency table index to
   get the value to be programmed in the radio for each channel                      */
#define CLP_FREQTAB_OFFSET          0   // Offset for Calypso radio

/// Radio skew compensation (round trip delay)
#define CLP_RADIO_SKEW              2

#define RFLOIF                      0x00

#define CLP_RSSI_20dB_THRHLD        -20
#define CLP_RSSI_40dB_THRHLD        -40
#define CLP_RSSI_45dB_THRHLD        -45
#define CLP_RSSI_48dB_THRHLD        -48
#define CLP_RSSI_55dB_THRHLD        -55
#define CLP_RSSI_60dB_THRHLD        -60
#define CLP_RSSI_70dB_THRHLD        -70

// EDR Control value
#define CLP_EDRCNTL                 18 // Default value is set to 18us

// TX max power
#define CLP_POWER_MAX               0x01 // 0x07 // 0x09 // reduce to 0x07 for better result right now
#define CLP_POWER_MIN               0x00
#define CLP_POWER_MSK               0x0F

// Generic RSSI Threshold
#define RF_CLP_RSSI_THR             0x29

#define CALYPSO_SPIRD               0x03
#define CALYPSO_SPIWR               0x83
#define CALYPSO_MAX_BURST_SIZE      4

#define CLP_RPLL_RC_TRIM            0
#define CLP_RXPGA_RC_TRIM           1
#define CLP_TXFILT_RC_TRIM          2

#define CLP_VCOFC_LUT               0
#define CLP_TXCNTL_LUT              1
#define CLP_RXCNTL_LUT              2

#define CLP_RC_VAL_TARGET_PLL       500
#define CLP_RC_VAL_TARGET_TX        197
#define CLP_RC_VAL_TARGET_RX        205

// RF Sequencer

// Reduce the Lock Detect time and threshold
// RPLLCNTL3 Settings
#define CLP_LOCK_THR    0x08
#define CLP_UNLOCK_THR  0x04
#define CLP_HYST_MAX    0x0A
#define CLP_FDIV_MAX    0x0C
#define CLP_REGRPLLCNTL3 (CLP_LOCK_THR<<24 | CLP_UNLOCK_THR<<16 | CLP_HYST_MAX <<8 | CLP_FDIV_MAX)

// TX Sequencer UP DELAY Settings

// TX UP DLY 0 : BB_BIAS_ON  = 0x00 : 0us
#define CLP_TX_UP_DLY0 0x00

// TX UP DLY 1 : RPLL_VCO_ON = 0x00 : 0.333us
#define CLP_TX_UP_DLY1 0x01

// TX UP DLY 2 : RPLL_CAL_EN = 0x3C : 19.98us (Estimated time for PLL to be stable and to start LD LOCK Detect)
#define CLP_TX_UP_DLY2 0x3C

// TX UP DLY 3 : RPLL_LD_ON  = 0x14 : 6.66us : rpll_locked raised 5us after the rpll_ld_on
#define CLP_TX_UP_DLY3 0x14

// TX UP DLY 4 : TXDAC_ON  = 0x04 : 1.332us
#define CLP_TX_UP_DLY4 0x04

// TX UP DLY 5 : TXPATH_ON = 0x01 : 0.333us
#define CLP_TX_UP_DLY5 0x01

// TX UP DLY 6 : MODEMTX_ON = 0x10 : 5.328us
#define CLP_TX_UP_DLY6 0x10

// TX UP DLY 7 : SIGMADELTA_EN = 0x01 : 0.333us
#define CLP_TX_UP_DLY7 0x01

// TX UP DLY 8 : TXPA ON = 0x01 : 0.333us
#define CLP_TX_UP_DLY8 0x01

// RX UP DLY 9 : RAMPGEN_EN = 0x01 : 0.333us
#define CLP_TX_UP_DLY9 0x01

// Registers RXUPDLY0/1/2
#define CLP_REGTXUPDLY0 (CLP_TX_UP_DLY0 | CLP_TX_UP_DLY1<<8 | CLP_TX_UP_DLY2<<16 | CLP_TX_UP_DLY3<<24)
#define CLP_REGTXUPDLY1 (CLP_TX_UP_DLY4 | CLP_TX_UP_DLY5<<8 | CLP_TX_UP_DLY6<<16 | CLP_TX_UP_DLY7<<24)
#define CLP_REGTXUPDLY2 (CLP_TX_UP_DLY8 | CLP_TX_UP_DLY9<<8 )

// RF Tx Power Up timing
// RF Sequencer      35us
// BB SPI @6MHz      6us
// Hidden            12us
#define CLP_TXPWRUP_1MBPS_US   53
#define CLP_TXPWRUP_2MBPS_US   53
#define CLP_TXPWRUP_LR_US      53
#define CLP_TXPWRUP_BT         53

// RX Sequencer UP DELAY Settings

// RX UP DLY 0 : BB_BIAS_ON  = 0x00 : 0us
#define CLP_RX_UP_DLY0 0x00

// RX UP DLY 1 : RPLL_VCO_ON = 0x00 : 0us
#define CLP_RX_UP_DLY1 0x00

// RX UP DLY 2 : RPLL_CAL_EN = 0x3C : 19.98us (Estimated time for PLL to be stable and to start LD LOCK Detect)
#define CLP_RX_UP_DLY2 0x3C

// RX UP DLY 3 : RPLL_LD_ON  = 0x14 : 6.66us : rpll_locked raised 5us after the rpll_ld_on
#define CLP_RX_UP_DLY3 0x14

// RX UP DLY 4 : RFRX_ON  = 0x04 : 1.332us
#define CLP_RX_UP_DLY4 0x04

// RX UP DLY 5 : RXPGA_ON = 0x01 : 0.333us
#define CLP_RX_UP_DLY5 0x01

// RX UP DLY 6 : RXADC_ON = 0x10 : 5.328us
#define CLP_RX_UP_DLY6 0x10

// RX UP DLY 7 : RXADC_EN = 0x01 : 0.333us
#define CLP_RX_UP_DLY7 0x01

// RX UP DLY 8 : Modem RX EN = 0x01 : 0.333us
#define CLP_RX_UP_DLY8 0x01

// RX UP DLY 9 : AGC_EN      = 0x01 : 0.333us
#define CLP_RX_UP_DLY9 0x01

// Registers RXUPDLY0/1/2
#define CLP_REGRXUPDLY0 (CLP_RX_UP_DLY0 | CLP_RX_UP_DLY1<<8 | CLP_RX_UP_DLY2<<16 | CLP_RX_UP_DLY3<<24)
#define CLP_REGRXUPDLY1 (CLP_RX_UP_DLY4 | CLP_RX_UP_DLY5<<8 | CLP_RX_UP_DLY6<<16 | CLP_RX_UP_DLY7<<24)
#define CLP_REGRXUPDLY2 (CLP_RX_UP_DLY8 | CLP_RX_UP_DLY9<<8 )

// RF Rx Power Up timing
// RF Sequencer     35us
// BB SPI @6MHz      6us
// Hidden           12us
#define CLP_RXPWRUP_1MBPS_US   54
#define CLP_RXPWRUP_2MBPS_US   54
#define CLP_RXPWRUP_LR_US      54
#define CLP_RXPWRUP_BT         54

/**
 ****************************************************************************************
 * MACROS
 *****************************************************************************************
 */

/// CALYPSO EM Write Macro for HW driven SPI chained structures
#define RF_CLP_EM_WR(addr, val) \
        EM_WR((((uint32_t) (addr)) + REG_EM_ET_BASE_ADDR), (val))

/*****************************************************************************************
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************/
// Power table
__STATIC const int8_t RF_CLP_TX_PW_CONV_TBL[CLP_PWR_TBL_SIZE] =
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

// Max burst register
__STATIC uint8_t rf_clp_reg_buf[CALYPSO_MAX_BURST_SIZE + 4]; // max burst size + buffer controls
/*****************************************************************************************
 * FUNCTION DEFINITIONS
 ****************************************************************************************/

/*****************************************************************************************
 * @brief SPI access
 ****************************************************************************************/
__STATIC void rf_clp_spi_tf(void)
{
    //launch SPI transfer
#if  defined(CFG_BT_EMB)
    ip_radiocntl0_spigo_setf(1);
#elif defined(CFG_BLE_EMB)
    ip_radiocntl0_spigo_setf(1);
#endif //CFG_BT_EMB

//#if !defined(RP_HWSIM_BYPASS)
    //wait for transfer to be completed
#if  defined(CFG_BT_EMB)
    while (!ip_radiocntl0_spicomp_getf());
#elif defined(CFG_BLE_EMB)
    while (!ip_radiocntl0_spicomp_getf());
#endif //CFG_BT_EMB
//#endif //(RP_HWSIM_BYPASS)
}

/*****************************************************************************************
 * @brief Calypso specific read access
 *
 * @param[in] addr    register address
 *
 * @return uint32_t value
 ****************************************************************************************/
__STATIC uint32_t rf_clp_reg_rd(uint32_t addr)
{
    uint32_t ret;
    // Next Pointr to 0x0
    rf_clp_reg_buf[0] = (uint8_t)(0);  // Next Pointer set to 0x0000 to stop the SPI Chained access
    rf_clp_reg_buf[1] = (uint8_t)(0);

    // Read acces length of 4 bytes
    rf_clp_reg_buf[2] = (uint8_t)(CALYPSO_SPIRD);
    // Address
    rf_clp_reg_buf[3] = (uint8_t)(addr & 0x00FF);

    // Data
    rf_clp_reg_buf[4] = 0x00;
    rf_clp_reg_buf[5] = 0x00;
    rf_clp_reg_buf[6] = 0x00;
    rf_clp_reg_buf[7] = 0x00;

    memcpy((void *)RF_EM_SPI_ADRESS, rf_clp_reg_buf, 8);

    //do the transfer
    rf_clp_spi_tf();

    //read back the buffer - 4 bytes register value MSB in buf[0]
    memcpy(rf_clp_reg_buf, (void *)(RF_EM_SPI_ADRESS + 4), 4);
    ret = co_read32p(&rf_clp_reg_buf[0]);

    return ret;
}

/*****************************************************************************************
 * @brief Calypso specific write access
 *
 * @param[in] addr    register address
 * @param[in] value   value to write
 *
 * @return uint32_t value
 ****************************************************************************************/
__STATIC void rf_clp_reg_wr(uint32_t addr, uint32_t value)
{
    // Next Pointr to 0x0
    rf_clp_reg_buf[0] = (uint8_t)(0);  // Next Pointer set to 0x0000 to stop the SPI Chained access
    rf_clp_reg_buf[1] = (uint8_t)(0);

    // Read acces length of 4 bytes
    rf_clp_reg_buf[2] = (uint8_t)(CALYPSO_SPIWR);
    // Address
    rf_clp_reg_buf[3] = (uint8_t)(addr & 0x00FF);

    //on old implementations (BT core 3.0, BLE core 1.0) swap the data
    co_write32p(&rf_clp_reg_buf[4], value);

    memcpy((void *)RF_EM_SPI_ADRESS, rf_clp_reg_buf, 8);

    //do the transfer
    rf_clp_spi_tf();

#if  defined(CFG_BT_EMB)
    ip_radiocntl0_spiptr_getf();
#elif defined(CFG_BLE_EMB)
    ip_radiocntl0_spiptr_getf();
#endif //CFG_BT_EMB
}

__STATIC void rf_em_init(void)
{
    uint8_t idx = 0;
    uint8_t temp_freq_tbl[EM_RF_FREQ_TABLE_LEN];

#if BT_EMB_PRESENT
    // First half part of frequency table is for the even frequencies
    while (idx < EM_RF_FREQ_TABLE_LEN / 2)
    {
        temp_freq_tbl[idx] = 2 * idx + CLP_FREQTAB_OFFSET;
        idx++;
    }
    while (idx < EM_RF_FREQ_TABLE_LEN)
    {
        temp_freq_tbl[idx] = 2 * (idx - (EM_RF_FREQ_TABLE_LEN / 2)) + 1 + CLP_FREQTAB_OFFSET;
        idx++;
    }
#elif BLE_EMB_PRESENT
    while (idx < EM_RF_FREQ_TABLE_LEN)
    {
        temp_freq_tbl[idx] = 2 * idx + CLP_FREQTAB_OFFSET;
        idx++;
    }
#endif // BT_EMB_PRESENT/BLE_EMB_PRESENT
    em_wr(&temp_freq_tbl[0], EM_FT_OFFSET, EM_RF_FREQ_TABLE_LEN);
}

#if (RF_CLASS1)
/*****************************************************************************************
 * @brief Static function - Calypso TX CNTL0 by radio / Used for External PA selection and Class 1 devices
 ****************************************************************************************/
__STATIC void rf_clp_set_txcntl0(void)
{
}
#endif // RF_CLASS1

#if !defined(RP_HWSIM_BYPASS)

/***************************************************************************************
 * @brief Static function - Initialization XO 1PIN
 ****************************************************************************************
 */

__STATIC void rf_clp_init_xo_1pin()
{
    uint8_t value;

    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

    clp_bank0_rpllcntl0_xo_1pin_c_trim_setf(0);
    ASSERT_ERR(clp_bank0_rpllcntl0_xo_1pin_c_trim_getf() == 0);

    clp_bank0_rpllcntl0_xo_bias_trim_setf(4);
    ASSERT_ERR(clp_bank0_rpllcntl0_xo_bias_trim_getf() == 4);

    // Wait 100ms
    for (int i = 0; i < 6667; i++)
    {
        value = clp_bank0_bankselcntl_banksel_getf();
        if (value == 1)
        {
            break;
        }
    }
}


/***************************************************************************************
 * @brief Static function - Initialization Bias
 ****************************************************************************************
 */

__STATIC void rf_clp_init_bias(uint32_t chip_id)
{
    uint8_t chip_id_found = 0;

    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

    for (int i = 0; i < CLP_CHIP_ID_TBL_SIZE; i++)
    {
        if (chip_id == RF_CLP_CHIP_ID[i])
        {
            clp_bank0_rftxrxconfig_bb_vref_setf(RF_CLP_BB_VREF[i]);
            ASSERT_ERR(clp_bank0_rftxrxconfig_bb_vref_getf() == RF_CLP_BB_VREF[i]);

            clp_bank0_rftxrxconfig_bb_bias_cfg_setf(RF_CLP_BB_BIAS_CFG[i]);
            ASSERT_ERR(clp_bank0_rftxrxconfig_bb_bias_cfg_getf() == RF_CLP_BB_BIAS_CFG[i]);

            chip_id_found = 1;
            break;
        }
    }

    ASSERT_INFO(chip_id_found == 1, 0, chip_id);
}

/**
 ****************************************************************************************
 * @brief Static function for init ICP value.
 *
 ****************************************************************************************
 */

__STATIC void rf_clp_init_icp(uint8_t channel, uint8_t icp)
{
    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

    clp_bank0_vcofc_icp_calib_lut_cntl_lut_vcofc_icp_wen_setf(0);
    ASSERT_ERR(clp_bank0_vcofc_icp_calib_lut_cntl_lut_vcofc_icp_wen_getf() == 0);

    clp_bank0_vcofc_icp_calib_lut_cntl_lut_icp_in_setf(icp);
    ASSERT_ERR(clp_bank0_vcofc_icp_calib_lut_cntl_lut_icp_in_getf() == icp);

    clp_bank0_vcofc_icp_calib_lut_cntl_lut_vcofc_icp_add_setf(channel);
    ASSERT_ERR(clp_bank0_vcofc_icp_calib_lut_cntl_lut_vcofc_icp_add_getf() == channel);

    clp_bank0_vcofc_icp_calib_lut_cntl_lut_vcofc_icp_wen_setf(1);
    ASSERT_ERR(clp_bank0_vcofc_icp_calib_lut_cntl_lut_vcofc_icp_wen_getf() == 1);

    clp_bank0_vcofc_icp_calib_lut_cntl_lut_vcofc_icp_wen_setf(0);
    ASSERT_ERR(clp_bank0_vcofc_icp_calib_lut_cntl_lut_vcofc_icp_wen_getf() == 0);

    clp_bank0_vcofc_icp_calib_lut_cntl_lut_vcofc_icp_add_setf(channel);
    ASSERT_ERR(clp_bank0_vcofc_icp_calib_lut_cntl_lut_vcofc_icp_add_getf() == channel);

    // Check that the icp value is really written
    ASSERT_ERR(clp_bank0_vcofc_icp_calib_lut_val_lut_icp_val_getf() == icp);
}

/***************************************************************************************
 * @brief Static function - Initialization PLL
 ****************************************************************************************
 */

__STATIC void rf_clp_init_pll(void)
{
    uint8_t new_default_icp = 7;

    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

    clp_bank0_rpllcntl1_rpll_ivco_buff_trim_setf(7);
    ASSERT_ERR(clp_bank0_rpllcntl1_rpll_ivco_buff_trim_getf() == 7);

    clp_bank0_rpllcntl1_rpll_ivco_setf(7);
    ASSERT_ERR(clp_bank0_rpllcntl1_rpll_ivco_getf() == 7);

    for (int i = 0; i < 80; i++)
    {
        rf_clp_init_icp(i, new_default_icp);
    }

    clp_bank0_rpllcntl1_rpll_icp_offset_setf(3);
    ASSERT_ERR(clp_bank0_rpllcntl1_rpll_icp_offset_getf() == 3);

}

/***************************************************************************************
 * @brief Static function - init LO path
 ****************************************************************************************
 */

__STATIC void rf_clp_init_lopath(void)
{
    // TODO when RF team is ready
}

/***************************************************************************************
 * @brief Static function - init RX
 ****************************************************************************************
 */

__STATIC void rf_clp_init_rx(void)
{
//    uint8_t value;

    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

    // Uses Rx RF path through Discrete Time Rx ADC
    clp_bank0_rfrxcntl_rxpath_sel_setf(1);
    ASSERT_ERR(clp_bank0_rfrxcntl_rxpath_sel_getf() == 1);

    // Uses Rx Digital path through Discrete Time Rx ADC
    clp_bank0_rfrxcntl_rxdigpath_sel_setf(1);
    ASSERT_ERR(clp_bank0_rfrxcntl_rxdigpath_sel_getf() == 1);

    clp_bank0_rftxrxcntl_txrx_matching_tank_setf(7);
    ASSERT_ERR(clp_bank0_rftxrxcntl_txrx_matching_tank_getf() == 7);

//    value = clp_bank0_rftxrxcntl_txrx_tank_force_getf();

    clp_bank0_lopathcntl_lopath_rx_iref_bias_trim_setf(2);
    ASSERT_ERR(clp_bank0_lopathcntl_lopath_rx_iref_bias_trim_getf() == 2);

//    value = clp_bank0_lopathcntl_lo_cntl_force_getf();

}

/***************************************************************************************
 * @brief Static function - init TX
 ****************************************************************************************
 */

__STATIC void rf_clp_init_tx(void)
{
    clp_bank0_bankselcntl_banksel_setf(1);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 1);

    clp_bank1_rftx_gfsk_gain_table_tx_fm1p_gfsk_mixer_gain_setf(10, 5);
    ASSERT_ERR(clp_bank1_rftx_gfsk_gain_table_tx_fm1p_gfsk_mixer_gain_getf(10) == 5);

    clp_bank1_rftx_gfsk_gain_table_tx_fm2p_gfsk_mixer_gain_setf(10, 5);
    ASSERT_ERR(clp_bank1_rftx_gfsk_gain_table_tx_fm2p_gfsk_mixer_gain_getf(10) == 5);
}

/***************************************************************************************
 * @brief Static function - bias on
 ****************************************************************************************
 */

__STATIC void rf_clp_bias_on(void)
{
    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

    clp_bank0_rftxrxcntl_bb_bias_on_setf(1);
    ASSERT_ERR(clp_bank0_rftxrxcntl_bb_bias_on_getf() == 1);

    clp_bank0_rftxrxcntl_txrx_bias_force_setf(1);
    ASSERT_ERR(clp_bank0_rftxrxcntl_txrx_bias_force_getf() == 1);
}

///***************************************************************************************
// * @brief Static function - bias off
// ****************************************************************************************
// */
//
//__STATIC void rf_clp_bias_off(void)
//{
//    clp_bank0_bankselcntl_banksel_setf(0);
//    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);
//
//    clp_bank0_rftxrxcntl_bb_bias_on_setf(0);
//    ASSERT_ERR(clp_bank0_rftxrxcntl_bb_bias_on_getf() == 0);
//
//    clp_bank0_rftxrxcntl_txrx_bias_force_setf(1);
//    ASSERT_ERR(clp_bank0_rftxrxcntl_txrx_bias_force_getf() == 1);
//}

/***************************************************************************************
 * @brief Static function - pll vco calibration
 ****************************************************************************************
 */

__STATIC void rf_clp_pll_vco_start_cal(uint8_t vcofc_mode, uint8_t icp_en)
{
    uint8_t value;
    int     i;

    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

    // icp calibration enable
    if (icp_en == 1)
    {
        clp_bank0_rftxrxcntl_icp_cal_dsb_setf(0);
        ASSERT_ERR(clp_bank0_rftxrxcntl_icp_cal_dsb_getf() == 0);
    }
    else
    {
        clp_bank0_rftxrxcntl_icp_cal_dsb_setf(1);
        ASSERT_ERR(clp_bank0_rftxrxcntl_icp_cal_dsb_getf() == 1);
    }

    // PLL calibration settings
    clp_bank0_rftxrxcntl_calib_mode_setf(1);
    ASSERT_ERR(clp_bank0_rftxrxcntl_calib_mode_getf() == 1);

    clp_bank0_rftxrxconfig_calib_timeout_delay_setf(555);
    ASSERT_ERR(clp_bank0_rftxrxconfig_calib_timeout_delay_getf() == 555);

    clp_bank0_vcofc_calib_config_vcofccalsign_setf(0);
    ASSERT_ERR(clp_bank0_vcofc_calib_config_vcofccalsign_getf() == 0);

    clp_bank0_icp_calib_config_rpll_icp_cal_num_setf(66);
    ASSERT_ERR(clp_bank0_icp_calib_config_rpll_icp_cal_num_getf() == 66);

    clp_bank0_icp_calib_config_rpll_icp_cal_den_setf(8);
    ASSERT_ERR(clp_bank0_icp_calib_config_rpll_icp_cal_den_getf() == 8);

    clp_bank0_icp_calib_config_icp_cal_ref_setf(16);
    ASSERT_ERR(clp_bank0_icp_calib_config_icp_cal_ref_getf() == 16);

    clp_bank0_mon_adc_cntl_monadc_out_sgn_setf(1);
    ASSERT_ERR(clp_bank0_mon_adc_cntl_monadc_out_sgn_getf() == 1);

    clp_bank0_icp_calib_config_rpll_icp_convn_setf(3);
    ASSERT_ERR(clp_bank0_icp_calib_config_rpll_icp_convn_getf() == 3);

    clp_bank0_vcofc_icp_timer_config_rpll_icp_dly2_setf(56);
    ASSERT_ERR(clp_bank0_vcofc_icp_timer_config_rpll_icp_dly2_getf() == 56);

    clp_bank0_icp_calib_config_rpll_icp_step_setf(4);
    ASSERT_ERR(clp_bank0_icp_calib_config_rpll_icp_step_getf() == 4);

    clp_bank0_rpllcntl2_rpll_on_dly_setf(20);
    ASSERT_ERR(clp_bank0_rpllcntl2_rpll_on_dly_getf() == 20);

    clp_bank0_vcofc_icp_timer_config_rpll_vcofc_dly_setf(24);
    ASSERT_ERR(clp_bank0_vcofc_icp_timer_config_rpll_vcofc_dly_getf() == 24);

    // Monitoring ADC clock
    clp_bank0_crm_cntl3_monadc_clk_sel_setf(2) ; //raw_root_clock clk/4 (12MHz)
    ASSERT_ERR(clp_bank0_crm_cntl3_monadc_clk_sel_getf() == 2);

    // Lock detect controlled by SW
    clp_bank0_rpllcntl0_force_ld_ctrl_setf(1);
    ASSERT_ERR(clp_bank0_rpllcntl0_force_ld_ctrl_getf() == 1);

    // Enable lock detect
    clp_bank0_rpllcntl0_rpll_ld_on_setf(1);
    ASSERT_ERR(clp_bank0_rpllcntl0_rpll_ld_on_getf() == 1);

    // Run calibration for coarse and fine modes
    clp_bank0_rpllcntl2_rpll_vcofc_ctrl_mode_setf(vcofc_mode);
    ASSERT_ERR(clp_bank0_rpllcntl2_rpll_vcofc_ctrl_mode_getf() == vcofc_mode);

    // Start calibration
    clp_bank0_rftxrxcntl_rfpll_cal_start_setf(1);
    ASSERT_ERR(clp_bank0_rftxrxcntl_rfpll_cal_start_getf() == 1);

    // Wait for 100ms
    for (i = 0; i < 6667; i++)
    {
        value = clp_bank0_rftxrxstat_vcofc_cal_done_getf();

        if (value == 1)
        {
            break;
        }
    }

    ASSERT_ERR(value == 1);

    if (icp_en == 1)
    {
        // Wait for 100ms
        for (i = 0; i < 6667; i++)
        {
            value = clp_bank0_rftxrxstat_icp_cal_done_getf();
            if (value == 1)
            {
                break;
            }
        }

        ASSERT_ERR(value == 1);
    }

    value = clp_bank0_rftxrxstat_vcofc_cold_cal_stat_getf();

    ASSERT_INFO((value == 0), value, 0);

    value = clp_bank0_rftxrxstat_cal_timeout_getf();

    ASSERT_INFO((value == 0), value, 0);

    clp_bank0_rftxrxcntl_rfpll_cal_start_setf(0);
    ASSERT_ERR(clp_bank0_rftxrxcntl_rfpll_cal_start_getf() == 0);

    // Clear force
    clp_bank0_rpllcntl0_force_ld_ctrl_setf(0);
    ASSERT_ERR(clp_bank0_rpllcntl0_force_ld_ctrl_getf() == 0);

}

/***************************************************************************************
 * @brief Static function - rc calibration core function
 ****************************************************************************************
 */
__STATIC void rf_clp_rc_cal_core(uint8_t trim_max, uint8_t trim_reg_select, uint16_t rc_val_target)
{
    int      triming        = (int)(trim_max);
    uint8_t  triming_val    = trim_max;
    uint8_t  value;
    uint16_t error          = 0;
    uint16_t previous_error = 0;
    int      i;
    uint8_t  polarity       = (trim_reg_select == CLP_RPLL_RC_TRIM) ? 0 : 1;
    uint16_t result         = (trim_reg_select == CLP_RPLL_RC_TRIM) ? 0 : 10000;

    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

    while ((((polarity == 1) && (result > rc_val_target)) || ((polarity == 0) && (result < rc_val_target)))
            &&
            (0 <= triming)
          )
    {
        triming_val = (uint8_t)(triming);

        switch (trim_reg_select)
        {
        case CLP_RPLL_RC_TRIM:
        {
            clp_bank0_rpllcntl1_rpll_rc_trim_setf(triming_val);
            ASSERT_ERR(clp_bank0_rpllcntl1_rpll_rc_trim_getf() == triming_val);
        }
        break;

        case CLP_RXPGA_RC_TRIM:
        {
            clp_bank0_rc_calib_cntl_rccal_rc_trim_setf(triming_val);
            ASSERT_ERR(clp_bank0_rc_calib_cntl_rccal_rc_trim_getf() == triming_val);
        }
        break;

        case CLP_TXFILT_RC_TRIM:
        {
            clp_bank0_rc_calib_cntl_rccal_rc_trim_setf(triming_val);
            ASSERT_ERR(clp_bank0_rc_calib_cntl_rccal_rc_trim_getf() == triming_val);
        }
        break;

        default:
        {
            ASSERT_ERR(0);
        }
        break;
        }

        // Start Calibration
        clp_bank0_rc_calib_cntl_rccal_on_setf(1);
        ASSERT_ERR(clp_bank0_rc_calib_cntl_rccal_on_getf() == 1);

        // Wait 100ms
        for (i = 0; i < 6667; i++)
        {
            value = clp_bank0_rc_calib_stat_rccal_done_getf();
            if (value == 1)
            {
                break;
            }
        }

        ASSERT_ERR(value == 1);

        result = clp_bank0_rc_calib_stat_rccal_stat_getf();

        previous_error = error;

        error = (result < rc_val_target) ? rc_val_target - result : result - rc_val_target;

        clp_bank0_rc_calib_cntl_rccal_on_setf(0);
        ASSERT_ERR(clp_bank0_rc_calib_cntl_rccal_on_getf() == 0);

        triming--;
    }

    ASSERT_INFO((triming >= 0), result, rc_val_target);

    if (previous_error < error)
    {
        triming_val = (triming_val < 6) ? triming_val + 2 : 7;
    }

    switch (trim_reg_select)
    {
    case CLP_RPLL_RC_TRIM:
    {
        clp_bank0_rpllcntl1_rpll_rc_trim_setf(triming_val);
        ASSERT_ERR(clp_bank0_rpllcntl1_rpll_rc_trim_getf() == triming_val);
    }
    break;

    case CLP_RXPGA_RC_TRIM:
    {
        clp_bank0_rfrxconfig0_rxpga_rc_trim_setf(triming_val);
        ASSERT_ERR(clp_bank0_rfrxconfig0_rxpga_rc_trim_getf() == triming_val);
    }
    break;

    case CLP_TXFILT_RC_TRIM:
    {
        clp_bank0_rftxconfig_txfilt_rc_trim_setf(triming_val);
        ASSERT_ERR(clp_bank0_rftxconfig_txfilt_rc_trim_getf() == triming_val);
    }
    break;

    default:
    {
        ASSERT_ERR(0);
    }
    break;
    }
}

/***************************************************************************************
 * @brief Static function - pll filter calibration
 ****************************************************************************************
 */

__STATIC void rf_clp_pll_filter_cal_start(void)
{
    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

    clp_bank0_rc_calib_cntl_rccal_sel_setf(1);
    ASSERT_ERR(clp_bank0_rc_calib_cntl_rccal_sel_getf() == 1);

    rf_clp_rc_cal_core(/*uint8_t trim_max*/ 7, /*uint8_t trim_reg_select*/ CLP_RPLL_RC_TRIM, /*uint16_t rc_val_target*/ CLP_RC_VAL_TARGET_PLL);
}

/***************************************************************************************
 * @brief Static function - rx filter calibration
 ****************************************************************************************
 */
__STATIC void rf_clp_rx_filter_cal_start(void)
{
    rf_clp_bias_on();

    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

    clp_bank0_rc_calib_cntl_rccal_sel_setf(0);
    ASSERT_ERR(clp_bank0_rc_calib_cntl_rccal_sel_getf() == 0);

    rf_clp_rc_cal_core(/*uint8_t trim_max*/ 15, /*uint8_t trim_reg_select*/ CLP_RXPGA_RC_TRIM, /*uint16_t rc_val_target*/ CLP_RC_VAL_TARGET_RX);
}

/***************************************************************************************
 * @brief Static function - tx filter calibration
 ****************************************************************************************
 */
__STATIC void rf_clp_tx_filter_cal_start(void)
{
    rf_clp_bias_on();

    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

    clp_bank0_rc_calib_cntl_rccal_sel_setf(0);
    ASSERT_ERR(clp_bank0_rc_calib_cntl_rccal_sel_getf() == 0);

    rf_clp_rc_cal_core(/*uint8_t trim_max*/ 15, /*uint8_t trim_reg_select*/ CLP_TXFILT_RC_TRIM, /*uint16_t rc_val_target*/ CLP_RC_VAL_TARGET_TX);
}

/***************************************************************************************
 * @brief Static function - tx tank calibration
 ****************************************************************************************
 */
__STATIC void rf_clp_tx_tanks_cal_start(void)
{
    uint8_t value;
    int     i;

    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

    // Monitoring ADC clock
    clp_bank0_crm_cntl3_monadc_clk_sel_setf(2);
    ASSERT_ERR(clp_bank0_crm_cntl3_monadc_clk_sel_getf() == 2);

    // Monadc set to envelope detector
    clp_bank0_mon_adc_cntl_monadc_sel_setf(1);
    ASSERT_ERR(clp_bank0_mon_adc_cntl_monadc_sel_getf() == 1);

    // Envelope detector set to Mixer + started
    clp_bank0_rftxconfig_txrf_detect_select_setf(0);
    ASSERT_ERR(clp_bank0_rftxconfig_txrf_detect_select_getf() == 0);

    clp_bank0_rftxcntl0_txrf_detect_on_setf(1);
    ASSERT_ERR(clp_bank0_rftxcntl0_txrf_detect_on_getf() == 1);

    clp_bank0_rftxcntl0_txrf_detect_dac_on_setf(1);
    ASSERT_ERR(clp_bank0_rftxcntl0_txrf_detect_dac_on_getf() == 1);

    // TODO: set txrf_detect_rfgain=1 if filter_gain below a threshold
    // TODO: set rfdetect_gain
    // TODO: set rfdetect_dc

    // Force gain using index 15
    clp_bank0_rftxcntl0_txgain_gfsk_sel_setf(15);
    ASSERT_ERR(clp_bank0_rftxcntl0_txgain_gfsk_sel_getf() == 15);

    clp_bank0_bankselcntl_banksel_setf(1);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 1);

    // PA gain = 0: tx_preamp_gain register = 2
    clp_bank1_rftx_gfsk_gain_table_tx_fm1p_gfsk_preamp_gain_setf(15, 2);
    ASSERT_ERR(clp_bank1_rftx_gfsk_gain_table_tx_fm1p_gfsk_preamp_gain_getf(15) == 2);

    clp_bank1_rftx_gfsk_gain_table_tx_fm1p_gfsk_filter_gain_setf(15, 2);
    ASSERT_ERR(clp_bank1_rftx_gfsk_gain_table_tx_fm1p_gfsk_filter_gain_getf(15) == 2);

    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

    clp_bank0_rftxcntl0_txgain_gfsk_force_setf(1);
    ASSERT_ERR(clp_bank0_rftxcntl0_txgain_gfsk_force_getf() == 1);

    // Tx mixer tank calibration select
    clp_bank0_tank_cal_cntl_tank_cal_sel_setf(1);
    ASSERT_ERR(clp_bank0_tank_cal_cntl_tank_cal_sel_getf() == 1);

    // Start calibration
    clp_bank0_tank_cal_cntl_tank_cal_start_setf(1);
    ASSERT_ERR(clp_bank0_tank_cal_cntl_tank_cal_start_getf() == 1);

    // Wait 100ms
    for (i = 0; i < 6667; i++)
    {
        value = clp_bank0_tank_cal_stat_tank_cal_done_getf();
        if (value == 1)
        {
            break;
        }
    }

    ASSERT_ERR(value == 1);

    // Stop calibration
    clp_bank0_tank_cal_cntl_tank_cal_start_setf(0);
    ASSERT_ERR(clp_bank0_tank_cal_cntl_tank_cal_start_getf() == 0);

    value = clp_bank0_tank_cal_stat_tank_cal_dead_getf();

    ASSERT_ERR(value == 0);

    // Tx matching tank cal:
    // PLL is still locked and txpath on
    clp_bank0_tank_cal_cntl_tank_cal_sel_setf(0);
    ASSERT_ERR(clp_bank0_tank_cal_cntl_tank_cal_sel_getf() == 0);

    // Envelope detector set to PA
    clp_bank0_rftxconfig_txrf_detect_select_setf(1);
    ASSERT_ERR(clp_bank0_rftxconfig_txrf_detect_select_getf() == 1);

    // Use mean of 4 ADC measures
    clp_bank0_txtank_config_tx_tank_convn_setf(1);
    ASSERT_ERR(clp_bank0_txtank_config_tx_tank_convn_getf() == 1);

    // Start calibration
    clp_bank0_tank_cal_cntl_tank_cal_start_setf(1);
    ASSERT_ERR(clp_bank0_tank_cal_cntl_tank_cal_start_getf() == 1);

    // Wait 100ms
    for (i = 0; i < 6667; i++)
    {
        value = clp_bank0_tank_cal_stat_tank_cal_done_getf();
        if (value == 1)
        {
            break;
        }
    }

    ASSERT_ERR(value == 1);

    // Stop calibration
    clp_bank0_tank_cal_cntl_tank_cal_start_setf(0);
    ASSERT_ERR(clp_bank0_tank_cal_cntl_tank_cal_start_getf() == 0);


    value = clp_bank0_tank_cal_stat_tank_cal_dead_getf();

    ASSERT_ERR(value == 0);

    // Monadc set to RF PLL voltmeter (default functional mode)
    clp_bank0_mon_adc_cntl_monadc_sel_setf(0);
    ASSERT_ERR(clp_bank0_mon_adc_cntl_monadc_sel_getf() == 0);

    // clear force
    clp_bank0_rftxcntl0_txgain_gfsk_force_setf(0);
    ASSERT_ERR(clp_bank0_rftxcntl0_txgain_gfsk_force_getf() == 0);
}



///***************************************************************************************
// * @brief Static function - load pattern
// *
// ****************************************************************************************
// */
//
//__STATIC void rf_clp_load_pattern_gen_tab(void)
//{
//    clp_bank0_bankselcntl_banksel_setf(0);
//    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);
//
//    for(int i=0; i<CLP_PATTERN_GEN_SIZE;i++)
//    {
//        clp_bank0_pattern_gen_lut_cntl_pack(/*uint8_t lutpatterngenwriteen*/ 1,
//                                            /*uint8_t lutpatterngenadd    */ (uint8_t)(i),
//                                            /*uint8_t lutpatterngenqin    */ RF_PATTERN_GEN_Q[i],
//                                            /*uint8_t lutpatterngeniin    */ RF_PATTERN_GEN_I[i]);
//    }
//
//    clp_bank0_pattern_gen_lut_cntl_lut_pattern_gen_write_en_setf(0);
//    ASSERT_ERR(clp_bank0_pattern_gen_lut_cntl_lut_pattern_gen_write_en_getf() == 0);
//
//    clp_bank0_pattern_gen_cntl_pattern_gen_mode_setf(2);
//    ASSERT_ERR(clp_bank0_pattern_gen_cntl_pattern_gen_mode_getf() == 2);
//
//    clp_bank0_pattern_gen_cntl_pattern_gen_length_setf(CLP_PATTERN_GEN_SIZE);
//    ASSERT_ERR(clp_bank0_pattern_gen_cntl_pattern_gen_length_getf() == CLP_PATTERN_GEN_SIZE);
//}

/***************************************************************************************
 * @brief Static function - lo on manual
 *
 ****************************************************************************************
 */

//__STATIC void rf_clp_lo_on_man(void)
//{
//    clp_bank0_bankselcntl_banksel_setf(0);
//    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);
//
//    clp_bank0_lopathcntl_lo_on_setf(1);
//    ASSERT_ERR(clp_bank0_lopathcntl_lo_on_getf() == 1);
//
//    clp_bank0_lopathcntl_lo_cntl_force_setf(1);
//    ASSERT_ERR(clp_bank0_lopathcntl_lo_cntl_force_getf() == 1);
//}

/***************************************************************************************
 * @brief Static function - rx on manual
 *
 ****************************************************************************************
 */

//__STATIC void rf_clp_rx_on_man(uint8_t gain)
//{
//    clp_bank0_bankselcntl_banksel_setf(0);
//    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);
//
//    clp_bank0_rfrxcntl_rxrf_on_setf(1);
//    ASSERT_ERR(clp_bank0_rfrxcntl_rxrf_on_getf() == 1);
//
//    clp_bank0_rfrxcntl_rxpga_i_on_setf(1);
//    ASSERT_ERR(clp_bank0_rfrxcntl_rxpga_i_on_getf() == 1);
//
//    clp_bank0_rfrxcntl_rxpga_q_on_setf(1);
//    ASSERT_ERR(clp_bank0_rfrxcntl_rxpga_q_on_getf() == 1);
//
//    clp_bank0_rfrxcntl_agc_en_setf(0);
//    ASSERT_ERR(clp_bank0_rfrxcntl_agc_en_getf() == 0);
//
//    clp_bank0_rfrxcntl_rxctrl_force_setf(1);
//    ASSERT_ERR(clp_bank0_rfrxcntl_rxctrl_force_getf() == 1);
//
//    clp_bank0_rfrxcntl_rxgain_force_setf(1);
//    ASSERT_ERR(clp_bank0_rfrxcntl_rxgain_force_getf() == 1);
//
//    clp_bank0_rfrxcntl_rxgain_sel_setf(gain); //  # force AGC index
//    ASSERT_ERR(clp_bank0_rfrxcntl_rxgain_sel_getf() == gain);
//}


//__STATIC void rf_pll_on_man(uint8_t channel)
//{
//    uint8_t   lutldval;
//    uint16_t  luticpkvcoval;
//    uint8_t   luticpval;
//    uint8_t   lutvcofcfineval;
//    uint8_t   lutvcofccoarseval;
//    int       i;
//    uint8_t   value;
//
//    clp_bank0_bankselcntl_banksel_setf(0);
//    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);
//
//    clp_bank0_rfdyncntl_channel_setf(channel);
//    ASSERT_ERR(clp_bank0_rfdyncntl_channel_getf() == channel);
//
//    // Read vcofc lut
//    clp_bank0_vcofc_icp_calib_lut_cntl_lut_vcofc_icp_add_setf(channel);
//    ASSERT_ERR(clp_bank0_vcofc_icp_calib_lut_cntl_lut_vcofc_icp_add_getf() == channel);
//
//    clp_bank0_vcofc_icp_calib_lut_val_unpack(&lutldval, &luticpkvcoval, &luticpval, &lutvcofcfineval, &lutvcofccoarseval);
//
//    clp_bank0_rpllcntl2_rpll_vcofc_coarse_setf(lutvcofccoarseval);
//    ASSERT_ERR(clp_bank0_rpllcntl2_rpll_vcofc_coarse_getf() == lutvcofccoarseval);
//
//    clp_bank0_rpllcntl2_rpll_vcofc_fine_setf(lutvcofcfineval);
//    ASSERT_ERR(clp_bank0_rpllcntl2_rpll_vcofc_fine_getf() == lutvcofcfineval);
//
//    clp_bank0_rpllcntl1_rpll_icp_setf(luticpval);
//    ASSERT_ERR(clp_bank0_rpllcntl1_rpll_icp_getf() == luticpval);
//
//
//    clp_bank0_rpllcntl0_rpll_vcoref_on_setf(1);
//    ASSERT_ERR(clp_bank0_rpllcntl0_rpll_vcoref_on_getf() == 1);
//
//    clp_bank0_rpllcntl0_rpll_cp_on_setf(1);
//    ASSERT_ERR(clp_bank0_rpllcntl0_rpll_cp_on_getf() == 1);
//
//    clp_bank0_rpllcntl0_rpll_vco_on_setf(1);
//    ASSERT_ERR(clp_bank0_rpllcntl0_rpll_vco_on_getf() == 1);
//
//    clp_bank0_rpllcntl0_rpll_div_on_setf(1);
//    ASSERT_ERR(clp_bank0_rpllcntl0_rpll_div_on_getf() == 1);
//
//    clp_bank0_rpllcntl0_force_rpll_ctrl_setf(1);
//    ASSERT_ERR(clp_bank0_rpllcntl0_force_rpll_ctrl_getf() == 1);
//
//    clp_bank0_rpllcntl0_rpll_ld_on_setf(1);
//    ASSERT_ERR(clp_bank0_rpllcntl0_rpll_ld_on_getf() == 1);
//
//    clp_bank0_rpllcntl0_force_ld_ctrl_setf(1);
//    ASSERT_ERR(clp_bank0_rpllcntl0_force_ld_ctrl_getf() == 1);
//
//    for(i=0; i<5; i++)
//    {
//        value = clp_bank0_rpllstat_rpll_locked_getf();
//
//        if(value == 1)
//            break;
//    }
//
//    ASSERT_INFO((value == 1), channel, value);
//}
//
//__STATIC void rf_tx_off_man(void)
//{
//
//    clp_bank0_bankselcntl_banksel_setf(0);
//    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);
//
//    clp_bank0_rftxcntl0_txpath_fm_on_setf(0);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txpath_fm_on_getf() == 0);
//
//    clp_bank0_rftxcntl0_fmtx2p_on_setf(0);
//    ASSERT_ERR(clp_bank0_rftxcntl0_fmtx2p_on_getf() == 0);
//
//    clp_bank0_rftxcntl0_txgain_gfsk_force_setf(0);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txgain_gfsk_force_getf() == 0);
//
//    clp_bank0_rftxcntl0_txgain_edr_force_setf(0);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txgain_edr_force_getf() == 0);
//
//    clp_bank0_rftxcntl0_sd_fifo_en_setf(0);
//    ASSERT_ERR(clp_bank0_rftxcntl0_sd_fifo_en_getf() == 0);
//
//    clp_bank0_rftxcntl0_txfilt_q_on_setf(0);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txfilt_q_on_getf() == 0);
//
//    clp_bank0_rftxcntl0_txfilt_i_on_setf(0);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txfilt_i_on_getf() == 0);
//
//    clp_bank0_rftxcntl0_txdac_q_on_setf(0);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txdac_q_on_getf() == 0);
//
//    clp_bank0_rftxcntl0_txdac_i_on_setf(0);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txdac_i_on_getf() == 0);
//
//    clp_bank0_rftxcntl0_txpath_iq_on_setf(0);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txpath_iq_on_getf() == 0);
//
//    clp_bank0_rftxcntl0_txpa_on_setf(0);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txpa_on_getf() == 0);
//
//    clp_bank0_rftxcntl0_txrf_detect_dac_on_setf(0);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txrf_detect_dac_on_getf() == 0);
//
//    clp_bank0_rftxcntl0_txrf_detect_on_setf(0);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txrf_detect_on_getf() == 0);
//
//    clp_bank0_rftxcntl0_txctrl_force_setf(1);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txctrl_force_getf() == 1);
//
//    clp_bank0_crm_cntl1_force_txdac_clk_setf(0);
//    ASSERT_ERR(clp_bank0_crm_cntl1_force_txdac_clk_getf() == 0);
//
//    clp_bank0_crm_cntl0_txdac_clk_gate_en_setf(0);
//    ASSERT_ERR(clp_bank0_crm_cntl0_txdac_clk_gate_en_getf() == 0);
//}
//
//__STATIC void rf_lo_off_man(void)
//{
//    clp_bank0_bankselcntl_banksel_setf(0);
//    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);
//
//    clp_bank0_lopathcntl_lo_on_setf(0);
//    ASSERT_ERR(clp_bank0_lopathcntl_lo_on_getf() == 0);
//
//    clp_bank0_lopathcntl_lo_cntl_force_setf(0);
//    ASSERT_ERR(clp_bank0_lopathcntl_lo_cntl_force_getf() == 0);
//}
//
//__STATIC void rf_rx_off_man(void)
//{
//    clp_bank0_bankselcntl_banksel_setf(0);
//    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);
//
//    clp_bank0_rfrxcntl_rxrf_on_setf(0);
//    ASSERT_ERR(clp_bank0_rfrxcntl_rxrf_on_getf() == 0);
//
//    clp_bank0_rfrxcntl_rxpga_i_on_setf(0);
//    ASSERT_ERR(clp_bank0_rfrxcntl_rxpga_i_on_getf() == 0);
//
//    clp_bank0_rfrxcntl_rxpga_q_on_setf(0);
//    ASSERT_ERR(clp_bank0_rfrxcntl_rxpga_q_on_getf() == 0);
//
//    clp_bank0_rfrxcntl_adc_on_setf(0);
//    ASSERT_ERR(clp_bank0_rfrxcntl_adc_on_getf() == 0);
//
//    clp_bank0_rfrxcntl_adc_en_setf(0);
//    ASSERT_ERR(clp_bank0_rfrxcntl_adc_en_getf() == 0);
//
//    clp_bank0_rfrxcntl_rssi_on_setf(0);
//    ASSERT_ERR(clp_bank0_rfrxcntl_rssi_on_getf() == 0);
//
//    clp_bank0_rfrxcntl_agc_en_setf(0);
//    ASSERT_ERR(clp_bank0_rfrxcntl_agc_en_getf() == 0);
//
//    clp_bank0_rfrxcntl_rxctrl_force_setf(1);
//    ASSERT_ERR(clp_bank0_rfrxcntl_rxctrl_force_getf() == 1);
//
//}
//
//__STATIC void rf_pll_off_man(void)
//{
//    clp_bank0_bankselcntl_banksel_setf(0);
//    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);
//
//    clp_bank0_rpllcntl0_rpll_vco_on_setf(0);
//    ASSERT_ERR(clp_bank0_rpllcntl0_rpll_vco_on_getf() == 0);
//
//    clp_bank0_rpllcntl0_rpll_vcoref_on_setf(0);
//    ASSERT_ERR(clp_bank0_rpllcntl0_rpll_vcoref_on_getf() == 0);
//
//    clp_bank0_rpllcntl0_rpll_cp_on_setf(0);
//    ASSERT_ERR(clp_bank0_rpllcntl0_rpll_cp_on_getf() == 0);
//
//    clp_bank0_rpllcntl0_rpll_div_on_setf(0);
//    ASSERT_ERR(clp_bank0_rpllcntl0_rpll_div_on_getf() == 0);
//
//    clp_bank0_rpllcntl0_rpll_locnt_on_setf(0);
//    ASSERT_ERR(clp_bank0_rpllcntl0_rpll_locnt_on_getf() == 0);
//
//    clp_bank0_rpllcntl0_rpll_ld_on_setf(0);
//    ASSERT_ERR(clp_bank0_rpllcntl0_rpll_ld_on_getf() == 0);
//
//    clp_bank0_rpllcntl0_force_rpll_ctrl_setf(0);
//    ASSERT_ERR(clp_bank0_rpllcntl0_force_rpll_ctrl_getf() == 0);
//
//    clp_bank0_rpllcntl0_force_ld_ctrl_setf(0);
//    ASSERT_ERR(clp_bank0_rpllcntl0_force_ld_ctrl_getf() == 0);
//}

///***************************************************************************************
// * @brief Static function - rx tank calibration
// *
// ****************************************************************************************
// */
//__STATIC void rf_clp_start_rx_cal(void)
//{
//
//    int i,j,k;
//    uint8_t  value;
//    uint16_t powest, rx_max_powest;
//    uint8_t  lna1_cap_trim_max=0;
//    uint8_t  lna2_cap_trim_max=0;
////    uint16_t DUMP_POWEST[19];
//
//    rf_clp_load_pattern_gen_tab();
//
//    clp_bank0_bankselcntl_banksel_setf(0);
//    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);
//
//    clp_bank0_crm_cntl0_txdac_clk_gate_en_setf(1);
//    ASSERT_ERR(clp_bank0_crm_cntl0_txdac_clk_gate_en_getf() == 1);
//
//    clp_bank0_crm_cntl1_force_txdac_clk_setf(1);
//    ASSERT_ERR(clp_bank0_crm_cntl1_force_txdac_clk_getf() == 1);
//
//    clp_bank0_crm_cntl0_rxadc_clk_gate_en_setf(1);
//    ASSERT_ERR(clp_bank0_crm_cntl0_rxadc_clk_gate_en_getf() == 1);
//
//    clp_bank0_crm_cntl1_force_rxadc_clk_setf(1);
//    ASSERT_ERR(clp_bank0_crm_cntl1_force_rxadc_clk_getf() == 1);
//
//    clp_bank0_crm_cntl0_mdm_rxfe48_clk_gate_en_setf(1);
//    ASSERT_ERR(clp_bank0_crm_cntl0_mdm_rxfe48_clk_gate_en_getf() == 1);
//
//    clp_bank0_crm_cntl1_force_mdm_rxfe48_clk_setf(1);
//    ASSERT_ERR(clp_bank0_crm_cntl1_force_mdm_rxfe48_clk_getf() == 1);
//
//    clp_bank0_crm_cntl0_mdm_rxfe24_clk_gate_en_setf(1);
//    ASSERT_ERR(clp_bank0_crm_cntl0_mdm_rxfe24_clk_gate_en_getf() == 1);
//
//    clp_bank0_crm_cntl1_force_mdm_rxfe24_clk_setf(1);
//    ASSERT_ERR(clp_bank0_crm_cntl1_force_mdm_rxfe24_clk_getf() == 1);
//
//    clp_bank0_crm_cntl0_mdm_rxfe12_clk_gate_en_setf(1);  // Not sure
//    ASSERT_ERR(clp_bank0_crm_cntl0_mdm_rxfe12_clk_gate_en_getf() == 1);
//
//    clp_bank0_crm_cntl1_force_mdm_rxfe12_clk_setf(1);  // Not sure
//    ASSERT_ERR(clp_bank0_crm_cntl1_force_mdm_rxfe12_clk_getf() == 1);
//
//    clp_bank0_rfrxcntl_adc_on_setf(1);
//    ASSERT_ERR(clp_bank0_rfrxcntl_adc_on_getf() == 1);
//
//    clp_bank0_rfrxcntl_adc_en_setf(1);
//    ASSERT_ERR(clp_bank0_rfrxcntl_adc_en_getf() == 1);
//
//    clp_bank0_rfrxcntl_rxrf_on_setf(1);
//    ASSERT_ERR(clp_bank0_rfrxcntl_rxrf_on_getf() == 1);
//
//    clp_bank0_rfrxcntl_rxpga_i_on_setf(1);
//    ASSERT_ERR(clp_bank0_rfrxcntl_rxpga_i_on_getf() == 1);
//
//    clp_bank0_rfrxcntl_rxpga_q_on_setf(1);
//    ASSERT_ERR(clp_bank0_rfrxcntl_rxpga_q_on_getf() == 1);
//
//    clp_bank0_rfrxcntl_rxctrl_force_setf(1);
//    ASSERT_ERR(clp_bank0_rfrxcntl_rxctrl_force_getf() == 1);
//
//    // Switch to bank 2
//    clp_bank0_bankselcntl_banksel_setf(2);
//    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 2);
//
//    clp_bank2_rxgain_table_rxpga_gain_setf(15, 5);
//    ASSERT_ERR(clp_bank2_rxgain_table_rxpga_gain_getf(15) == 5);
//
//    clp_bank2_rxgain_table_rxlna1_gain_setf(15, 0);
//    ASSERT_ERR(clp_bank2_rxgain_table_rxlna1_gain_getf(15) == 0);
//
//    clp_bank2_rxgain_table_rxlna2_gain_setf(15, 0);
//    ASSERT_ERR(clp_bank2_rxgain_table_rxlna2_gain_getf(15) == 0);
//
//    // Switch to bank 0
//    clp_bank0_bankselcntl_banksel_setf(0);
//    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);
//
//    clp_bank0_rfrxconfig0_dtadc_gain_cfg_setf(3);
//    ASSERT_ERR(clp_bank0_rfrxconfig0_dtadc_gain_cfg_getf() == 3);
//
//    clp_bank0_rfrxcntl_rxgain_sel_setf(15);
//    ASSERT_ERR(clp_bank0_rfrxcntl_rxgain_sel_getf() == 15);
//
//    clp_bank0_rfrxcntl_rxrf_on_setf(1);
//    ASSERT_ERR(clp_bank0_rfrxcntl_rxrf_on_getf() == 1);
//
//    clp_bank0_lopathcntl_lo_on_setf(1);
//    ASSERT_ERR(clp_bank0_lopathcntl_lo_on_getf() == 1);
//
//    clp_bank0_lopathcntl_lo_cntl_force_setf(1);
//    ASSERT_ERR(clp_bank0_lopathcntl_lo_cntl_force_getf() == 1);
//
//    clp_bank0_rftxcntl0_txgain_gfsk_sel_setf(15);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txgain_gfsk_sel_getf() == 15);
//
//    // Switch to bank 1
//    clp_bank0_bankselcntl_banksel_setf(1);
//    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 1);
//
//    clp_bank1_rftx_gfsk_gain_table_tx_iq_gfsk_filter_gain_setf(15, 6);
//    ASSERT_ERR(clp_bank1_rftx_gfsk_gain_table_tx_iq_gfsk_filter_gain_getf(15) == 6);
//
//
//    clp_bank1_rftx_gfsk_gain_table_tx_iq_gfsk_preamp_gain_setf(15, 0);
//    ASSERT_ERR(clp_bank1_rftx_gfsk_gain_table_tx_iq_gfsk_preamp_gain_getf(15) == 0);
//
//    clp_bank1_rftx_dac_gain_txdac_gfsk_gain_setf(0);
//    ASSERT_ERR(clp_bank1_rftx_dac_gain_txdac_gfsk_gain_getf() == 0);
//
//    // Switch to bank 0
//    clp_bank0_bankselcntl_banksel_setf(0);
//    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);
//
//    clp_bank0_rftxcntl0_txgain_gfsk_force_setf(0);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txgain_gfsk_force_getf() == 0);
//
//    clp_bank0_rftxcntl0_txgain_edr_force_setf(1);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txgain_edr_force_getf() == 1);
//
//    clp_bank0_rftxcntl0_txdac_i_on_setf(1);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txdac_i_on_getf() == 1);
//
//    clp_bank0_rftxcntl0_txdac_q_on_setf(1);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txdac_q_on_getf() == 1);
//
//    clp_bank0_rftxcntl0_txfilt_i_on_setf(1);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txfilt_i_on_getf() == 1);
//
//    clp_bank0_rftxcntl0_txfilt_q_on_setf(1);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txfilt_q_on_getf() == 1);
//
//    clp_bank0_rftxcntl0_txpath_iq_on_setf(1);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txpath_iq_on_getf() == 1);
//
//    clp_bank0_rftxcntl0_txpa_on_setf(0);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txpa_on_getf() == 0);
//
//    clp_bank0_rftxcntl0_txctrl_force_setf(1);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txctrl_force_getf() == 1);
//
//    clp_bank0_rftxcntl0_txpath_fm_on_setf(0);
//    ASSERT_ERR(clp_bank0_rftxcntl0_txpath_fm_on_getf() == 0);
//
//    clp_bank0_rftxcntl0_fmtx2p_on_setf(0);
//    ASSERT_ERR(clp_bank0_rftxcntl0_fmtx2p_on_getf() == 0);
//
//    clp_bank0_rftxconfig_txfilt_band_setf(1); //  # BW = 2MHz
//    ASSERT_ERR(clp_bank0_rftxconfig_txfilt_band_getf() == 1);
//
//    // # Program power estimator: measurement window of 100 us:
//    clp_bank0_powestcntl_powest_ns_fact_setf(10);
//    ASSERT_ERR(clp_bank0_powestcntl_powest_ns_fact_getf() == 10);
//
//    clp_bank0_rfrxcntl_rxlna_cap_trim_force_setf(1);
//    ASSERT_ERR(clp_bank0_rfrxcntl_rxlna_cap_trim_force_getf() == 1);
//
//    // # Start pattern generator
//    clp_bank0_pattern_gen_cntl_pattern_gen_mux_ctrl_setf(3);
//    ASSERT_ERR(clp_bank0_pattern_gen_cntl_pattern_gen_mux_ctrl_getf() == 3);
//
//    clp_bank0_pattern_gen_cntl_pattern_gen_en_setf(1);
//    ASSERT_ERR(clp_bank0_pattern_gen_cntl_pattern_gen_en_getf() == 1);
//
//    // Set LNA2 trimming to min value
//    clp_bank0_rfrxcntl_rxlna2_cap_trim_setf(RF_LNA_CAP_TRIM[0]); // -8
//    ASSERT_ERR(clp_bank0_rfrxcntl_rxlna2_cap_trim_getf() == RF_LNA_CAP_TRIM[0]);
//
//    // lna1
//    for(i=0; i<80; i++)
//    {
//        rf_pll_on_man(i);
//
//        // Switch to bank 0
//        clp_bank0_bankselcntl_banksel_setf(0);
//        ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);
//
//        lna1_cap_trim_max = RF_LNA_CAP_TRIM[0];
//        rx_max_powest     = 0;
//
//
//        for(j=0; j<16; j++)
//        {
//            clp_bank0_rfrxcntl_rxlna1_cap_trim_setf(RF_LNA_CAP_TRIM[j]);
//            ASSERT_ERR(clp_bank0_rfrxcntl_rxlna1_cap_trim_getf() == RF_LNA_CAP_TRIM[j]);
//
//            clp_bank0_powestcntl_powest_en_setf(1);
//            ASSERT_ERR(clp_bank0_powestcntl_powest_en_getf() == 1);
//
//            // Poll 100 ms
//            for(k=0; k<6667; k++)
//            {
//                value = clp_bank0_poweststat_powest_done_getf();
//
//                if(value == 1)
//                    break;
//            }
//
//            ASSERT_INFO((value == 1),i,RF_LNA_CAP_TRIM[j]);
//
//            powest = clp_bank0_poweststat_powest_lin_meas_getf();
////            DUMP_POWEST[j] = powest;
//
//            if (powest > rx_max_powest)
//            {
//                lna1_cap_trim_max = RF_LNA_CAP_TRIM[j];
//                rx_max_powest     = powest;
//            }
//
//            clp_bank0_powestcntl_powest_en_setf(0);
//            ASSERT_ERR(clp_bank0_powestcntl_powest_en_getf() == 0);
//        }
//
//        // write_lnatank_lut
//        clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_setf(0);
//        ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_getf() == 0);
//
//        clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_add_setf(i);
//        ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_add_getf() == i);
//
//        clp_bank0_rfrxcntl_lut_cntl_lut_rxlna1_cap_trim_in_setf(lna1_cap_trim_max);
//        ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rxlna1_cap_trim_in_getf() == lna1_cap_trim_max);
//
//        clp_bank0_rfrxcntl_lut_cntl_lut_rxlna2_cap_trim_in_setf(RF_LNA_CAP_TRIM[0]); // -8
//        ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rxlna2_cap_trim_in_getf() == RF_LNA_CAP_TRIM[0]);
//
//        clp_bank0_rfrxcntl_lut_cntl_lut_rxmatching_tank_in_setf(7);
//        ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rxmatching_tank_in_getf() == 7);
//
//        clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_setf(1);
//        ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_getf() == 1);
//
//        clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_setf(0);
//        ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_getf() == 0);
//
////        DUMP_POWEST[16] = (uint16_t)(i);
////        DUMP_POWEST[17] = (uint16_t)(lna1_cap_trim_max);
////        DUMP_POWEST[18] = 0;
////
////        DUMP_DATA(&DUMP_POWEST, sizeof(DUMP_POWEST));
////
////        for (int wait=0; wait<6667; wait++)
////        {
////            clp_bank0_btrfip_ver_get();
////        }
//    }
//
//    // lna2
//    for(i=0; i<80; i++)
//    {
//        rf_pll_on_man(i);
//
//        // Switch to bank 0
//        clp_bank0_bankselcntl_banksel_setf(0);
//        ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);
//
//        clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_setf(0);
//        ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_getf() == 0);
//
//        clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_add_setf(i);
//        ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_add_getf() == i);
//
//        lna1_cap_trim_max = clp_bank0_rfrxcntl_lut_val_lut_rxlna1_cap_trim_val_getf();
//
//        clp_bank0_rfrxcntl_rxlna1_cap_trim_setf(lna1_cap_trim_max);
//        ASSERT_ERR(clp_bank0_rfrxcntl_rxlna1_cap_trim_getf() == lna1_cap_trim_max);
//
//        lna2_cap_trim_max = RF_LNA_CAP_TRIM[0];
//        rx_max_powest     = 0;
//
//        for(j=0; j<16; j++)
//        {
//            clp_bank0_rfrxcntl_rxlna2_cap_trim_setf(RF_LNA_CAP_TRIM[j]);
//            ASSERT_ERR(clp_bank0_rfrxcntl_rxlna2_cap_trim_getf() == RF_LNA_CAP_TRIM[j]);
//
//            clp_bank0_powestcntl_powest_en_setf(1);
//            ASSERT_ERR(clp_bank0_powestcntl_powest_en_getf() == 1);
//
//            // Poll 100 ms
//            for(k=0; k<6667; k++)
//            {
//                value = clp_bank0_poweststat_powest_done_getf();
//
//                if(value == 1)
//                    break;
//            }
//
//            ASSERT_INFO((value == 1),i,RF_LNA_CAP_TRIM[j]);
//
//            powest = clp_bank0_poweststat_powest_lin_meas_getf();
////            DUMP_POWEST[j] = powest;
//
//            if (powest > rx_max_powest)
//            {
//                lna2_cap_trim_max = RF_LNA_CAP_TRIM[j];
//                rx_max_powest     = powest;
//            }
//
//            clp_bank0_powestcntl_powest_en_setf(0);
//            ASSERT_ERR(clp_bank0_powestcntl_powest_en_getf() == 0);
//        }
//
//        // write_lnatank_lut
//        clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_setf(0);
//        ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_getf() == 0);
//
//        clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_add_setf(i);
//        ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_add_getf() == i);
//
//        clp_bank0_rfrxcntl_lut_cntl_lut_rxlna1_cap_trim_in_setf(0); //lna1_cap_trim_max);
//        ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rxlna1_cap_trim_in_getf() == 0); //lna1_cap_trim_max);
//
//        clp_bank0_rfrxcntl_lut_cntl_lut_rxlna2_cap_trim_in_setf(0); //lna2_cap_trim_max);
//        ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rxlna2_cap_trim_in_getf() == 0);//lna2_cap_trim_max);
//
//        clp_bank0_rfrxcntl_lut_cntl_lut_rxmatching_tank_in_setf(7);
//        ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rxmatching_tank_in_getf() == 7);
//
//        clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_setf(1);
//        ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_getf() == 1);
//
//        clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_setf(0);
//        ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_getf() == 0);
//
////        DUMP_POWEST[16] = (uint16_t)(i);
////        DUMP_POWEST[17] = (uint16_t)(lna2_cap_trim_max);
////        DUMP_POWEST[18] = 0;
////
////        for (int wait=0; wait<6667; wait++)
////        {
////            clp_bank0_btrfip_ver_get();
////        }
//    }
//
//    clp_bank0_pattern_gen_cntl_pattern_gen_en_setf(0);
//    ASSERT_ERR(clp_bank0_pattern_gen_cntl_pattern_gen_en_getf() == 0);
//
//    rf_tx_off_man();
//
//    rf_lo_off_man();
//
//    rf_rx_off_man();
//
//    rf_pll_off_man();
//
//    // Switch to bank 0
//    clp_bank0_bankselcntl_banksel_setf(0);
//    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);
//
//    clp_bank0_crm_cntl0_txdac_clk_gate_en_setf(0);
//    ASSERT_ERR(clp_bank0_crm_cntl0_txdac_clk_gate_en_getf() == 0);
//
//    clp_bank0_crm_cntl1_force_txdac_clk_setf(0);
//    ASSERT_ERR(clp_bank0_crm_cntl1_force_txdac_clk_getf() == 0);
//
//    clp_bank0_crm_cntl0_rxadc_clk_gate_en_setf(0);
//    ASSERT_ERR(clp_bank0_crm_cntl0_rxadc_clk_gate_en_getf() == 0);
//
//    clp_bank0_crm_cntl1_force_rxadc_clk_setf(0);
//    ASSERT_ERR(clp_bank0_crm_cntl1_force_rxadc_clk_getf() == 0);
//
//    clp_bank0_crm_cntl0_mdm_rxfe48_clk_gate_en_setf(0);
//    ASSERT_ERR(clp_bank0_crm_cntl0_mdm_rxfe48_clk_gate_en_getf() == 0);
//
//    clp_bank0_crm_cntl1_force_mdm_rxfe48_clk_setf(0);
//    ASSERT_ERR(clp_bank0_crm_cntl1_force_mdm_rxfe48_clk_getf() == 0);
//
//    clp_bank0_crm_cntl0_mdm_rxfe24_clk_gate_en_setf(0);
//    ASSERT_ERR(clp_bank0_crm_cntl0_mdm_rxfe24_clk_gate_en_getf() == 0);
//
//    clp_bank0_crm_cntl1_force_mdm_rxfe24_clk_setf(0);
//    ASSERT_ERR(clp_bank0_crm_cntl1_force_mdm_rxfe24_clk_getf() == 0);
//
//    clp_bank0_crm_cntl0_mdm_rxfe12_clk_gate_en_setf(0); //  # Not sure
//    ASSERT_ERR(clp_bank0_crm_cntl0_mdm_rxfe12_clk_gate_en_getf() == 0);
//
//    clp_bank0_crm_cntl1_force_mdm_rxfe12_clk_setf(0); //  # Not sure
//    ASSERT_ERR(clp_bank0_crm_cntl1_force_mdm_rxfe12_clk_getf() == 0);
//}

///***************************************************************************************
// * @brief Static function - Dump RF Luts
// ****************************************************************************************
// */
//
//__STATIC void rf_dump_lut(uint8_t lut_type)
//{
//    uint32_t DUMP_LUT[81];
//
//    clp_bank0_bankselcntl_banksel_setf(0);
//    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);
//
//    switch(lut_type)
//    {
//        case CLP_VCOFC_LUT:
//        {
//            clp_bank0_vcofc_icp_calib_lut_cntl_lut_vcofc_icp_wen_setf(0);
//            ASSERT_ERR(clp_bank0_vcofc_icp_calib_lut_cntl_lut_vcofc_icp_wen_getf() == 0);
//        } break;
//
//        case CLP_TXCNTL_LUT:
//        {
//            clp_bank0_rftxcntl_lut_cntl_lut_rftxcntl_wen_setf(0);
//            ASSERT_ERR(clp_bank0_rftxcntl_lut_cntl_lut_rftxcntl_wen_getf() == 0);
//        } break;
//
//        case CLP_RXCNTL_LUT:
//        {
//            clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_setf(0);
//            ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_getf() == 0);
//        } break;
//
//        default:
//        {
//            ASSERT_ERR(0);
//        } break;
//    }
//
//    for(int channel=0; channel<80; channel++)
//    {
//        switch(lut_type)
//        {
//            case CLP_VCOFC_LUT:
//            {
//                clp_bank0_vcofc_icp_calib_lut_cntl_lut_vcofc_icp_add_setf(channel);
//                ASSERT_ERR(clp_bank0_vcofc_icp_calib_lut_cntl_lut_vcofc_icp_add_getf() == channel);
//
//                DUMP_LUT[channel] = clp_bank0_vcofc_icp_calib_lut_val_get();
//            } break;
//
//            case CLP_TXCNTL_LUT:
//            {
//                clp_bank0_rftxcntl_lut_cntl_lut_rftxcntl_add_setf(channel);
//                ASSERT_ERR(clp_bank0_rftxcntl_lut_cntl_lut_rftxcntl_add_getf() == channel);
//
//                DUMP_LUT[channel] = clp_bank0_rftxcntl_lut_val_get();
//            } break;
//
//            case CLP_RXCNTL_LUT:
//            {
//                clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_add_setf(channel);
//                ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_add_getf() == channel);
//
//                DUMP_LUT[channel] = clp_bank0_rfrxcntl_lut_val_get();
//            } break;
//
//            default:
//            {
//                ASSERT_ERR(0);
//            } break;
//        }
//    }
//
//    DUMP_LUT[80] = 0;
//    DUMP_DATA(&DUMP_LUT, sizeof(DUMP_LUT));
//
//    for (int i=0; i<6667; i++)
//    {
//        clp_bank0_btrfip_ver_get();
//    }
//}

/***************************************************************************************
 * @brief Static function - Set Tx DC I / Q In  for Calypso radio
 ****************************************************************************************
 */
__STATIC void rf_clp_txdciqin(uint32_t chip_id)
{

    uint8_t chip_id_found = 0;

    // Selection bank 0
    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

    if (chip_id != 0)
    {
        for (int i = 0; i < CLP_CHIP_ID_TBL_SIZE; i++)
        {
            if (chip_id == RF_CLP_CHIP_ID[i])
            {
                for (int j = 0; j < 16 ; j++)
                {
                    clp_bank0_txdc_calib_lut_cntl_pack(1,    // uint8_t luttxdcwen
                                                       0,   // uint8_t luttxdcsel - EPC mode not used
                                                       j,   // uint8_t luttxdcadd
                                                       RF_CLP_DC_Q_IN[i],   // uint8_t luttxdcqin
                                                       RF_CLP_DC_I_IN[i]);  // uint8_t luttxdciin
                }

                chip_id_found = 1;
                break;
            }
        }

        ASSERT_INFO(chip_id_found == 1, 0, chip_id);
    }
}

/***************************************************************************************
 * @brief Static function - Set Tx Gain / Phase In  for Calypso radio
 ****************************************************************************************
 */
__STATIC void rf_clp_txgainphaseiqin(uint32_t chip_id)
{

    uint8_t chip_id_found = 0;

    // Selection bank 0
    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

    if (chip_id != 0)
    {
        for (int i = 0; i < CLP_CHIP_ID_TBL_SIZE; i++)
        {
            if (chip_id == RF_CLP_CHIP_ID[i])
            {
                for (int j = 0; j < 16 ; j++)
                {
                    clp_bank0_txmdmfe_lut_cntl_pack(1,    // uint8_t luttxiqcompwriteen
                                                    0,   // uint8_t luttxiqcompsel - EPC mode not used
                                                    j,   // uint8_t luttxiqcompadd
                                                    RF_CLP_TXIQ_GAIN_IN[i],   // uint8_t luttxiqcompgainin
                                                    RF_CLP_TXIQ_PHASE_IN[i]);  // uint8_t luttxiqcompphasein
                }

                chip_id_found = 1;
                break;
            }
        }

        ASSERT_INFO(chip_id_found == 1, 0, chip_id);
    }
}

/***************************************************************************************
 * @brief Static function - Initialization sequence for Calypso radio
 ****************************************************************************************
 */
__STATIC void rf_clp_init_seq(void)
{
    // Try to get Calypso NVDS Chip RF ID
    uint8_t  rf_chip_id_length = PARAM_LEN_RF_ID;
    uint32_t rf_chip_id;
    uint8_t  chip_id_found = 0;

    if (rwip_param.get(PARAM_ID_RF_ID, &rf_chip_id_length, (uint8_t *)&rf_chip_id) != PARAM_OK)
    {
        rf_chip_id =  0;
    }

    // Check if chip_id value match with known chip in the table, excluding the index 0
    for (int i = 1; i < CLP_CHIP_ID_TBL_SIZE; i++)
    {
        if (rf_chip_id == RF_CLP_CHIP_ID[i])
        {
            chip_id_found = 1;
            break;
        }
    }

    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

    // Send a soft reset
    clp_bank0_rfdyncntl_soft_rst_setf(1);
    clp_bank0_rfdyncntl_soft_rst_setf(0);

    // Disable dig_test output because of too much current consumption;
//    clp_bank0_testcntl_mux_ctrl_en_setf(0);
//    ASSERT_ERR(clp_bank0_testcntl_mux_ctrl_en_getf() == 0);

//    clp_bank0_padcntl_set(0);

    /* RF DIAGCNTL*/
    clp_bank0_testcntl_mux_ctrl_setf(38);
    ASSERT_ERR(clp_bank0_testcntl_mux_ctrl_getf() == 38);

    /*init modem in core*/
    rf_clp_init_mdmhp();

    // Set Modem in bypass mode - this overrides some previous modem inits
    rf_clp_init_mdmhp_bypass();

    /*init sequencer */
    rf_clp_sequencers_init();

    // init  xo 1pin
    rf_clp_init_xo_1pin();

    // init bias for a chip ID
    rf_clp_init_bias(rf_chip_id);

    // Wait for an amount of time
    // A SPI read 32 at Baseband Clk 12Mhz is 8us
    // Processing time is 7 us (CPU 24Mhz)
    // RF python seems to wait for 100ms
    for (int i = 0; i < 6667; i++)
    {
        clp_bank0_btrfip_ver_get();
    }

    // init pll
    rf_clp_init_pll();

    // init LO path
    rf_clp_init_lopath();

    // init Tx
    rf_clp_init_tx();

    // init Rx
    rf_clp_init_rx();

    // bias on
    rf_clp_bias_on();

    // Wait for an amount of time
    // A SPI read 32 at Baseband Clk 12Mhz is 8us
    // Processing time is 7 us (CPU 24Mhz)
    // RF python seems to wait for 100ms
    for (int i = 0; i < 6667; i++)
    {
        clp_bank0_btrfip_ver_get();
    }

    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

    // pll vco calibratoin
    rf_clp_pll_vco_start_cal(/*uint8_t vcofc_mode*/ 1, /*uint8_t icp_en*/ 1);

    clp_bank0_rftxrxcntl_calib_mode_setf(0);
    ASSERT_ERR(clp_bank0_rftxrxcntl_calib_mode_getf() == 0);

    // pll filter calibration
    rf_clp_pll_filter_cal_start();

    // rx tank calibration
    // rf_clp_start_rx_cal();

    for (int i = 0; i < 80; i++)
    {
        // Switch to bank 0
        clp_bank0_bankselcntl_banksel_setf(0);
        ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

        // write_lnatank_lut
        clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_setf(0);
        ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_getf() == 0);

        clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_add_setf(i);
        ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_add_getf() == i);

        if (chip_id_found == 1) // Only known chip in the chip table
        {
            uint8_t trim_value;

            // Apply the value from ticket #12471 4bits width
            if (i <= 17)
            {
                trim_value = 0x9; // -7
            }
            else if (i <= 37)
            {
                trim_value = 0xA; // -6
            }
            else if (i <= 51)
            {
                trim_value = 0xB; // -5
            }
            else if (i <= 69)
            {
                trim_value = 0xC; // -4
            }
            else
            {
                trim_value = 0xD; // -3
            }

            clp_bank0_rfrxcntl_lut_cntl_lut_rxlna1_cap_trim_in_setf(trim_value);
            ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rxlna1_cap_trim_in_getf() == trim_value);
        }
        else // default settings
        {
            clp_bank0_rfrxcntl_lut_cntl_lut_rxlna1_cap_trim_in_setf(0); //lna1_cap_trim_max);
            ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rxlna1_cap_trim_in_getf() == 0); //lna1_cap_trim_max);
        }

        clp_bank0_rfrxcntl_lut_cntl_lut_rxmatching_tank_in_setf(7);
        ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rxmatching_tank_in_getf() == 7);

        clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_setf(1);
        ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_getf() == 1);

        clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_setf(0);
        ASSERT_ERR(clp_bank0_rfrxcntl_lut_cntl_lut_rfrxcntl_wen_getf() == 0);
    }

    // tx filter
    rf_clp_tx_filter_cal_start();

    // rx filter
    rf_clp_rx_filter_cal_start();

    // tx tanks calibration
    rf_clp_tx_tanks_cal_start();

    // Set Analog RSSI thresholds
    clp_bank0_rfrxconfig1_pack(3, 3, 3, 3, 0, 0);
    ASSERT_ERR(clp_bank0_rfrxconfig1_rssi_thres_low_cfg_getf() == 3);
    ASSERT_ERR(clp_bank0_rfrxconfig1_rssi_thres_high_cfg_getf() == 3);
    ASSERT_ERR(clp_bank0_rfrxconfig1_rssi_rectifier_low_cfg_getf() == 3);
    ASSERT_ERR(clp_bank0_rfrxconfig1_rssi_rectifier_high_cfg_getf() == 3);

    // Set digital RSSI thresholds
    clp_bank0_rxmdmfe_rssi_dig_est_rssi_thr_high_setf(7);
    ASSERT_ERR(clp_bank0_rxmdmfe_rssi_dig_est_rssi_thr_high_getf() == 7);
    clp_bank0_rxmdmfe_rssi_dig_est_rssi_thr_low_setf(6);
    ASSERT_ERR(clp_bank0_rxmdmfe_rssi_dig_est_rssi_thr_low_getf() == 6);

    // Selection bank 2
    clp_bank0_bankselcntl_banksel_setf(2);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 2);

    // AGC FE RST CNTL
    clp_bank2_agc_config_fe_rst_ctrl_setf(1);
    ASSERT_ERR(clp_bank2_agc_config_fe_rst_ctrl_getf() == 1);

    // Set agc fsm gain down step2 to 1
    // Fix the peak PER LE -60dbm
    clp_bank2_agc_fsm_config1_gain_dw_step2_setf(0x1);
    ASSERT_ERR(clp_bank2_agc_fsm_config1_gain_dw_step2_getf() == 0x1);

    // Set ADC reset duration to 250 ns so that if it happens during sync word, less than
    // one BLE2 symbol is corrupted
    clp_bank2_agc_config_adc_rst_cnt_dur_setf(11);
    ASSERT_ERR(clp_bank2_agc_config_adc_rst_cnt_dur_getf() == 11);

    // New AGC FSM
    clp_bank2_agc_fsm_state_condition_config_timer_stop_state_setf(2, 100);
    ASSERT_ERR(clp_bank2_agc_fsm_state_condition_config_timer_stop_state_getf(2) == 100);
    clp_bank2_agc_fsm_state_condition_config_pack(11, 0, 1, 0, 31, 0, 31);
    clp_bank2_agc_fsm_state_operation_config_next3_state_setf(0, 11);
    ASSERT_ERR(clp_bank2_agc_fsm_state_operation_config_next3_state_getf(0) == 11);
    clp_bank2_agc_fsm_state_operation_config_next1_state_setf(11, 2);
    ASSERT_ERR(clp_bank2_agc_fsm_state_operation_config_next1_state_getf(11) == 2);

    // Selection bank 0
    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

    // Switch back to normal calib function mode
    clp_bank0_rftxrxcntl_calib_mode_setf(0);
    ASSERT_ERR(clp_bank0_rftxrxcntl_calib_mode_getf() == 0);

    // Selection bank 1
    clp_bank0_bankselcntl_banksel_setf(1);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 1);

    // Set gain table for EDR. GFSK IQ gain is used as EPC mode=0
    for (int i = 0; i < 16 ; i++)
    {
        clp_bank1_rftx_gfsk_gain_table_tx_iq_gfsk_preamp_gain_setf(i, 0);
        clp_bank1_rftx_gfsk_gain_table_tx_iq_gfsk_filter_gain_setf(i, 2);
    }

    // TX DC I / Q IN
    rf_clp_txdciqin(rf_chip_id);

    if (rf_chip_id != 7200004) // See better result in EDR if the gain is 0 and the comp en = 0 (ie: we may have a bug if comp en  = 1 and gain is 0)
    {
        // TX IQ Gain / phase compensation
        rf_clp_txgainphaseiqin(rf_chip_id);
        clp_bank0_txmdmfecntl_txiq_comp_en_setf(1);
        ASSERT_ERR(clp_bank0_txmdmfecntl_txiq_comp_en_getf() == 1);
    }

//    clp_bank0_rfrxcntl_agc_en_setf(0);
//    clp_bank0_rfrxcntl_rxgain_force_setf(1);
//    clp_bank0_rfrxcntl_rxgain_sel_setf(0xB);
//    clp_bank0_rfrxconfig0_dtadc_gain_cfg_setf(2);
//    clp_bank0_rftxrxcntl_rxmdm_bypass_en_setf(1);

    // Dump to SDcard
//    rf_dump_lut(CLP_VCOFC_LUT);
//    rf_dump_lut(CLP_TXCNTL_LUT);
//    rf_dump_lut(CLP_RXCNTL_LUT);

}

///*****************************************************************************************
// * @brief Static function - Calypso TX CNTL1
// ****************************************************************************************/
//__STATIC void rf_clp_set_txcntl1(void)
//{
//}

/*****************************************************************************************
 * @brief Static function - Calypso RF Power up sequence (all on)
 ****************************************************************************************
 */
//__STATIC void rf_clp_pw_up(void)
//{
// /* TO DO */
//
//}

#endif //(RP_HWSIM_BYPASS)

//  The following functions are also needed for HW sim
/***************************************************************************************
 * @brief Static function - Init modem bypass mode
 ****************************************************************************************
 */

__STATIC void rf_clp_init_mdmhp_bypass(void)
{
    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

    // Set BB clock to 48 MHz*/
//    clp_bank0_crm_cntl3_bb_clk_div_setf(0) ;
//    ASSERT_ERR(clp_bank0_crm_cntl3_bb_clk_div_getf() == 0);

    // Set Modem in Bypass mode
    // Work-around for Calypso TC1 bug #12416: actually do not set rxmdm_bypass_en
//    clp_bank0_rftxrxcntl_rxmdm_bypass_en_setf(0x00000001);
//    ASSERT_ERR(clp_bank0_rftxrxcntl_rxmdm_bypass_en_getf() == 1);

    clp_bank0_rftxrxcntl_txmdm_bypass_en_setf(0x00000001);
    ASSERT_ERR(clp_bank0_rftxrxcntl_txmdm_bypass_en_getf() == 1);

    // RX FE clocks needed for RSSI are enabled by modem: in case of external modem, force clocks on
    clp_bank0_crm_cntl0_mdm_rxfe48_clk_gate_en_setf(1);
    ASSERT_ERR(clp_bank0_crm_cntl0_mdm_rxfe48_clk_gate_en_getf() == 1);

    clp_bank0_crm_cntl0_mdm_rxfe24_clk_gate_en_setf(1);
    ASSERT_ERR(clp_bank0_crm_cntl0_mdm_rxfe24_clk_gate_en_getf() == 1);

    clp_bank0_crm_cntl1_force_mdm_rxfe48_clk_setf(1);
    ASSERT_ERR(clp_bank0_crm_cntl1_force_mdm_rxfe48_clk_getf() == 1);

    clp_bank0_crm_cntl1_force_mdm_rxfe24_clk_setf(1);
    ASSERT_ERR(clp_bank0_crm_cntl1_force_mdm_rxfe24_clk_getf() == 1);

    // Invert Rx ADC clock
    clp_bank0_crm_cntl3_rxadc_clk_inv_en_setf(1);
    ASSERT_ERR(clp_bank0_crm_cntl3_rxadc_clk_inv_en_getf() == 1);
}

/***************************************************************************************
 * @brief Static function - Sequencer settings Initialization for Calypso radio
 ****************************************************************************************
*/
__STATIC void rf_clp_sequencers_init(void)
{

    // Selection bank 0
    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

    // Change the LD lock detect
    clp_bank0_rpllcntl3_set(CLP_REGRPLLCNTL3);
    ASSERT_ERR(clp_bank0_rpllcntl3_get() == CLP_REGRPLLCNTL3);

    // Selection bank 3
    clp_bank0_bankselcntl_banksel_setf(3);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 3);

    // Tx pwr up sequencer
    clp_bank3_rftxup_dly0_set(CLP_REGTXUPDLY0);
    ASSERT_ERR(clp_bank3_rftxup_dly0_get() == CLP_REGTXUPDLY0);

    clp_bank3_rftxup_dly1_set(CLP_REGTXUPDLY1);
    ASSERT_ERR(clp_bank3_rftxup_dly1_get() == CLP_REGTXUPDLY1);

    clp_bank3_rftxup_dly2_set(CLP_REGTXUPDLY2);
    ASSERT_ERR(clp_bank3_rftxup_dly2_get() == CLP_REGTXUPDLY2);

    clp_bank3_seqtxup_out0_set(0x00030001);
    ASSERT_ERR(clp_bank3_seqtxup_out0_get() == 0x00030001);

    clp_bank3_seqtxup_out1_set(0x000F0007);
    ASSERT_ERR(clp_bank3_seqtxup_out1_get() == 0x000F0007);

    clp_bank3_seqtxup_out2_set(0x003F001F);
    ASSERT_ERR(clp_bank3_seqtxup_out2_get() == 0x003F001F);

    clp_bank3_seqtxup_out3_set(0x01F30133);
    ASSERT_ERR(clp_bank3_seqtxup_out3_get() == 0x01F30133);

    clp_bank3_seqtxup_out4_set(0x03F301F3);
    ASSERT_ERR(clp_bank3_seqtxup_out4_get() == 0x03F301F3);

    // Tx pwr dwn sequencer

    // Rx pwr up sequencer
    clp_bank3_rfrxup_dly0_set(CLP_REGRXUPDLY0);
    ASSERT_ERR(clp_bank3_rfrxup_dly0_get() == CLP_REGRXUPDLY0);

    clp_bank3_rfrxup_dly1_set(CLP_REGRXUPDLY1);
    ASSERT_ERR(clp_bank3_rfrxup_dly1_get() == CLP_REGRXUPDLY1);

    clp_bank3_rfrxup_dl2_set(CLP_REGRXUPDLY2); // xls naming error
    ASSERT_ERR(clp_bank3_rfrxup_dl2_get() == CLP_REGRXUPDLY2);
}

/***************************************************************************************
 * @brief Static function - Tx Gain tables settings
 ****************************************************************************************
 */
//__STATIC void rf_clp_txgain_set(void)
//{
//}


/***************************************************************************************
* @ brief __STATIC function - Calibration sequence for TXDC offset
***************************************************************************************/
//__STATIC void rf_clp_txdc_cal_seq(void)
//{
//
//}


/*****************************************************************************************
 * @brief Static function - Init modem for Calypso.
 ****************************************************************************************
 */
__STATIC void rf_clp_init_mdmhp(void)
{
    clp_bank0_bankselcntl_banksel_setf(5);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 5);

    // DEBUG: Use LP mode
    //mdmhp_mdm_cntl_hplp_mode_setf(2);
    //ASSERT_ERR(mdmhp_mdm_cntl_hplp_mode_getf() == 2);

    // Set scaling so that Modem RSSI value is always > 0 (needed by BB RC)
    mdmhp_rxrssi_scaling_cntl_rssi_scaling_lr_setf(13);
    ASSERT_ERR(mdmhp_rxrssi_scaling_cntl_rssi_scaling_lr_getf() == 13);
    mdmhp_rxrssi_scaling_cntl_rssi_scaling_bt_setf(13);
    ASSERT_ERR(mdmhp_rxrssi_scaling_cntl_rssi_scaling_bt_getf() == 13);
    mdmhp_rxrssi_scaling_cntl_rssi_scaling_ble1_setf(13);
    ASSERT_ERR(mdmhp_rxrssi_scaling_cntl_rssi_scaling_ble1_getf() == 13);
    mdmhp_rxrssi_scaling_cntl_rssi_scaling_ble2_setf(13);
    ASSERT_ERR(mdmhp_rxrssi_scaling_cntl_rssi_scaling_ble2_getf() == 13);

    // Set threshold for LP/HP switch
    // BT: force switch at -88. thr = -87+46+117 = 76
    mdmhp_rxrssi_thr_cntl_rssi_selthr_bt_setf(76);
    ASSERT_ERR(mdmhp_rxrssi_thr_cntl_rssi_selthr_bt_getf() == 76);
    // BLE1: force switch at -89. thr = -89+46+117 = 74
    mdmhp_rxrssi_thr_cntl_rssi_selthr_ble1_setf(74);
    ASSERT_ERR(mdmhp_rxrssi_thr_cntl_rssi_selthr_ble1_getf() == 74);
    // BLE2: force switch at -84. thr = -84+46+117+2 = 81
    mdmhp_rxrssi_thr_cntl_rssi_selthr_ble2_setf(81);
    ASSERT_ERR(mdmhp_rxrssi_thr_cntl_rssi_selthr_ble2_getf() == 81);

    // TSI sync threshold for BLE2
    mdmhp_rxsync_cntl_ble_sync_tsi_thres_ble_setf(2048);
    ASSERT_ERR(mdmhp_rxsync_cntl_ble_sync_tsi_thres_ble_getf() == 2048);

    // Work-around for Calypso TC1 bug #12416
    // As modem enable is set only after second Rx On SPI access (reg_rx_modem_bypass),
    // Modem startup delay is 8 us shorter than RF startup delay (35 us)
    // Modem is started 4 us after AGC so that in case of interferer, first AGC setting is stable
    // and modem RSSI is not started during a power ramp-down.
    mdmhp_mdm_cntl_rx_startupdel_setf(27 + 4);
    ASSERT_ERR(mdmhp_mdm_cntl_rx_startupdel_getf() == 27 + 4);

    // No positive impact seen, but to be checked with PER over complete power range
    mdmhp_rxsop_cntl_lr_sop_highthr_en_lr_setf(0);
    ASSERT_ERR(mdmhp_rxsop_cntl_lr_sop_highthr_en_lr_getf() == 0);

    mdmhp_mdm_diagcntl_pack(15, 15, 15, 15);

    // Enable STO compensation for BLE
    //mdmhp_rxstocomp_cntl_btble_sto_comp_force_ble_setf(0);
    //ASSERT_ERR(mdmhp_rxstocomp_cntl_btble_sto_comp_force_ble_getf() == 0);
    //mdmhp_rxstoest_cntl0_stoest_en_ble_setf(1);
    //ASSERT_ERR(mdmhp_rxstoest_cntl0_stoest_en_ble_getf() == 1);


    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);
}



/**
 ****************************************************************************************
 * @brief Static function - Measure Calypso VCO Frequency
 *
 * @param[in] vco_fc_value  VCO
 * @param[in] vco_freq      Pointer to frequency value.
 ****************************************************************************************
 */
//__STATIC void rf_clp_measure_vco_freq(uint8_t vco_fc_value, int * vco_freq)
//{
//}

/**
 ****************************************************************************************
 * @brief Static function - for VCO Calibration
 *
 * @param[in] channel   channel
 * @param[in] vco_val   vco value
 ****************************************************************************************
 */
//__STATIC void rf_clp_calib_vco_fq(uint8_t channel, uint8_t *vco_val)
//{
//}


/**
 ****************************************************************************************
 * @brief Static function for status lock.
 *
 * @param[in] chnl  channel
 * @param[in] icp   icp
 * @param[in] vco   vco value
 * @param[in] lock  pointer to lock
 ****************************************************************************************
 */
//__STATIC void rf_clp_status_lock(uint8_t chnl, uint8_t icp, uint8_t vco, uint8_t *lock)
//{
//}


/***************************************************************************************
 * @brief Static function for radio PLL auto-calibration.
 ****************************************************************************************
 */
//__STATIC void rf_clp_pll_autocalib(void)
//{
// /* TO DO */
//}

/***************************************************************************************
 * @brief Static Calypso radio Calibration function.
 ***************************************************************************************
 */
//__STATIC void rf_clp_calib(void)
//{
//    rf_clp_pll_autocalib();
//}

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
    int8_t RssidBm;
//    uint8_t DUMP_RSSI[2];


    // This scaling includes the 3 dB of gain missing in Calypso over the complete input power range
    RssidBm = (int8_t)rssi_reg - 109;

    //DUMP_RSSI[0] = (int8_t)rssi_reg;
    //DUMP_RSSI[1] = RssidBm;
    //DUMP_DATA(&DUMP_RSSI, sizeof(DUMP_RSSI));

    return (RssidBm);
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
 * @return The index of the TX power
 *
 *****************************************************************************************
 */
__STATIC uint8_t rf_txpwr_cs_get(int8_t txpwr_dbm, uint8_t option)
{
    ASSERT_ERR(option <= TXPWR_CS_NEAREST);

    uint8_t i;

    for (i = CLP_POWER_MIN; i < CLP_POWER_MAX; i++)
    {
        // Loop until we find a power higher than or equal to the requested one
        if (RF_CLP_TX_PW_CONV_TBL[i] >= txpwr_dbm)
            break;
    }

    if ((RF_CLP_TX_PW_CONV_TBL[i] > txpwr_dbm) && (i > CLP_POWER_MIN))
    {
        if ((option == TXPWR_CS_LOWER)
                || ((option == TXPWR_CS_NEAREST) && (co_abs(txpwr_dbm - RF_CLP_TX_PW_CONV_TBL[i - 1]) < co_abs(txpwr_dbm - RF_CLP_TX_PW_CONV_TBL[i]))))
        {
            i--;
        }
    }

    return (i);
}

/**
 *****************************************************************************************
 * @brief Init RF sequence after reset.
 *****************************************************************************************
 */
__STATIC void rf_reset(void)
{
//    #if !defined(RP_HWSIM_BYPASS)
//    // Calibration procedure
//    rf_clp_calib();
//    #endif
}

#if  defined(CFG_BT_EMB)
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
    uint8_t tx_pwr = em_bt_pwrcntl_txpwr_getf(EM_BT_CS_ACL_INDEX(link_id)) & CLP_POWER_MSK;

    if (tx_pwr > CLP_POWER_MIN)
    {
        //Increase the TX power value
        em_bt_pwrcntl_txpwr_setf(EM_BT_CS_ACL_INDEX(link_id), tx_pwr - 1);
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
    uint8_t tx_pwr = em_bt_pwrcntl_txpwr_getf(EM_BT_CS_ACL_INDEX(link_id)) & CLP_POWER_MSK;

    if (tx_pwr < CLP_POWER_MAX)
    {
        //Increase the TX power value
        em_bt_pwrcntl_txpwr_setf(EM_BT_CS_ACL_INDEX(link_id), tx_pwr + 1);
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
    em_bt_pwrcntl_txpwr_setf(EM_BT_CS_ACL_INDEX(link_id), CLP_POWER_MAX);
}
#endif //CFG_BT_EMB

#if defined(CFG_BLE_EMB)

/**
 *****************************************************************************************
 * @brief Enable/disable force AGC mechanism
 *
 * @param[in]  True: Enable / False: disable
 *****************************************************************************************
 */
__STATIC void rf_force_agc_enable(bool en)
{
#if  defined(CFG_BT_EMB)
    ip_radiocntl1_forceagc_en_setf(en);
#else
    ip_radiocntl1_forceagc_en_setf(en);
#endif //CFG_BLE_EMB
}
#endif //CFG_BLE_EMB

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
    // power table is the same for BR and EDR
    return (RF_CLP_TX_PW_CONV_TBL[txpwr_idx]);
}

/**
 *****************************************************************************************
 * @brief Sleep function for the RF.
 *****************************************************************************************
 */
__STATIC void rf_sleep(void)
{
    ip_deepslcntl_set(ip_deepslcntl_get() |
                      IP_DEEP_SLEEP_ON_BIT |    // RW BT Core sleep
                      IP_RADIO_SLEEP_EN_BIT |   // Radio sleep
                      IP_OSC_SLEEP_EN_BIT);     // Oscillator sleep
}

/****************************************************************************************
 * RADIO FUNCTION INTERFACE
 ***************************************************************************************/
void rf_init(struct rwip_rf_api *api)
{
    uint8_t length = PARAM_LEN_RSSI_THR;
    uint16_t nextptr ;
    uint32_t dbg12416;

    // Initialize the RF driver API structure
    api->reg_rd = rf_clp_reg_rd;
    api->reg_wr = rf_clp_reg_wr;
    api->txpwr_dbm_get = rf_txpwr_dbm_get;
    api->txpwr_min = CLP_POWER_MIN;
    api->txpwr_max = CLP_POWER_MAX;
    api->sleep = rf_sleep;
    api->reset = rf_reset;

#if defined(CFG_BLE_EMB)
    api->force_agc_enable = rf_force_agc_enable;
#endif //CFG_BLE_EMB

    api->rssi_convert = rf_rssi_convert;
    api->txpwr_cs_get = rf_txpwr_cs_get;

#if  defined(CFG_BT_EMB)
    api->txpwr_dec = rf_txpwr_dec;
    api->txpwr_inc = rf_txpwr_inc;
    api->txpwr_max_set = txpwr_max_set;
#endif //CFG_BT_EMB

    // Initialize the RSSI thresholds (high, low, interference)
    // The upper threshold level is 20 dB above the lower threshold level to an accuracy of +-6 dB
    // These are 'real' signed values in dBm
    if (rwip_param.get(PARAM_ID_RSSI_HIGH_THR, &length, (uint8_t *)&api->rssi_high_thr) != PARAM_OK)
    {
        api->rssi_high_thr = (int8_t)CLP_RSSI_40dB_THRHLD;
    }
    if (rwip_param.get(PARAM_ID_RSSI_LOW_THR, &length, (uint8_t *)&api->rssi_low_thr) != PARAM_OK)
    {
        api->rssi_low_thr = (int8_t)CLP_RSSI_60dB_THRHLD;
    }
    if (rwip_param.get(PARAM_ID_RSSI_INTERF_THR, &length, (uint8_t *)&api->rssi_interf_thr) != PARAM_OK)
    {
        api->rssi_interf_thr = (int8_t)CLP_RSSI_70dB_THRHLD;
    }

    /// Initialize Exchange Memory
    rf_em_init();

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
            /*uint8_t  syncpulsemode*/   0,
            /*uint8_t  syncpulsesrc*/    1,
            /*uint8_t  dpcorren*/        0,
            /*uint8_t  jefselect*/       0,
            /*uint8_t  xrfsel*/          0x03,
            /*uint8_t  subversion*/      0x0);

#if defined(CFG_BLE_EMB)
    /* BLE RADIOCNTL2 */
    ble_radiocntl2_pack(/*uint8_t  lrsynccompmode*/ 0x0,
            /*uint8_t  rxcitermbypass*/ 0x0,
            /*uint8_t  lrvtbflush*/     16,
            /*uint8_t  phymsk*/         0x3, // mark that Coded phy and 2Mbps are supported
            /*uint8_t  lrsyncerr*/      0,
            /*uint8_t  syncerr*/        0,
            /*uint16_t freqtableptr*/ (EM_FT_OFFSET >> 2));

    /* BLE RADIOCNTL3 */
    ble_radiocntl3_pack(/*uint8_t rxrate3cfg*/    0x3,
            /*uint8_t rxrate2cfg*/    0x2,
            /*uint8_t rxrate1cfg*/    0x1,
            /*uint8_t rxrate0cfg*/    0x0,
            /*uint8_t getrssidelay */ BLE_GETRSSIDELAY_RST,
            /*uint8_t rxsyncrouting*/ 0x0,
            /*uint8_t rxvalidbeh*/    0x2,
            /*uint8_t txrate3cfg*/    0x0, // map on 1 Mbps
            /*uint8_t txrate2cfg*/    0x0, // map on 1 Mbps
            /*uint8_t txrate1cfg*/    0x1,
            /*uint8_t txrate0cfg*/    0x0,
            /*uint8_t txvalidbeh*/    0x2);

    /* BLE RADIOPWRUPDN0 */
    ble_radiopwrupdn0_pack(/*uint8_t syncposition0*/ 0,
            /*uint8_t rxpwrup0*/      CLP_RXPWRUP_1MBPS_US,
            /*uint8_t txpwrdn0*/      17,
            /*uint8_t txpwrup0*/      CLP_TXPWRUP_1MBPS_US);

    /* BLE RADIOPWRUPDN1 */
    ble_radiopwrupdn1_pack(/*uint8_t syncposition1*/ 0,
            /*uint8_t rxpwrup1*/      CLP_RXPWRUP_2MBPS_US,
            /*uint8_t txpwrdn1*/      17,
            /*uint8_t txpwrup1*/      CLP_TXPWRUP_2MBPS_US);

    /* BLE RADIOPWRUPDN2 */
    ble_radiopwrupdn2_pack(/*uint8_t syncposition2*/ 0,
            /*uint8_t rxpwrup2*/      CLP_RXPWRUP_LR_US,
            /*uint8_t txpwrdn2*/      17,
            /*uint8_t txpwrup2*/      CLP_TXPWRUP_LR_US);

    /* BLE RADIOPWRUPDN3 */
    ble_radiopwrupdn3_pack(/*uint8_t txpwrdn3*/      17,
            /*uint8_t txpwrup3*/      CLP_TXPWRUP_LR_US);

    /* BLE RADIOTXRXTIM0 */
    ble_radiotxrxtim0_pack(/*uint8_t rfrxtmda0*/   30,//12+22,
            /*uint8_t rxpathdly0*/  30,//9+24,
            /*uint8_t txpathdly0*/  3);

    /* BLE RADIOTXRXTIM1 */
    ble_radiotxrxtim1_pack(/*uint8_t rfrxtmda1*/   17,
            /*uint8_t rxpathdly1*/  17,
            /*uint8_t txpathdly1*/  3);

    /* BLE RADIOTXRXTIM2 */
    ble_radiotxrxtim2_pack(/*uint8_t rxflushpathdly2*/ 22,
            /*uint8_t rfrxtmda2*/       140,//139,
            /*uint8_t rxpathdly2*/      52,//48+7,
            /*uint8_t txpathdly2*/      4);

    /* BLE RADIOTXRXTIM3 */
    ble_radiotxrxtim3_pack(/*uint8_t rxflushpathdly3*/ 22,
            /*uint8_t rfrxtmda3*/       45,
            /*uint8_t txpathdly3*/      4);
#endif // defined CFG_BLE_EMB

#if  defined(CFG_BT_EMB)

    /* EDRCNTL */
    bt_rwbtcntl_nwinsize_setf(0);
    bt_edrcntl_rxgrd_timeout_setf(CLP_EDRCNTL);

    /* BT RADIOPWRUPDN */
    bt_radiopwrupdn_rxpwrupct_setf(CLP_RXPWRUP_BT);
    bt_radiopwrupdn_txpwrdnct_setf(17);
    bt_radiopwrupdn_txpwrupct_setf(CLP_TXPWRUP_BT);

    /* BT RADIOCNTL 2 */
    bt_radiocntl2_freqtable_ptr_setf((EM_FT_OFFSET >> 2));
    bt_radiocntl2_syncerr_setf(0x0);

    /* BT RADIOTXRXTIM */
    bt_radiotxrxtim_rxpathdly_setf(31);//9+27);
    bt_radiotxrxtim_txpathdly_setf(4);
    bt_radiotxrxtim_sync_position_setf(0x0); // Default is 0x10

    /* BT RADIOCNTL 3*/
    bt_radiocntl3_pack(
        /*uint8_t rxrate2cfg*/    3,
        /*uint8_t rxrate1cfg*/    2,
        /*uint8_t rxrate0cfg*/    0,
        /*uint8_t getrssidelay*/  0x0,
        /*uint8_t rxserparif*/    0,
        /*uint8_t rxsyncrouting*/ 0,
        /*uint8_t rxvalidbeh*/    2,
        /*uint8_t txrate2cfg*/    3,
        /*uint8_t txrate1cfg*/    2,
        /*uint8_t txrate0cfg*/    0,
        /*uint8_t txserparif*/    0,
        /*uint8_t txvalidbeh*/    2);

    // HP modem does not send trailer bits to Baseband
    bt_radiocntl2_trailer_gating_val_setf(0);
    ASSERT_ERR(bt_radiocntl2_trailer_gating_val_getf() == 0);
#endif //CFG_BT_EMB

#if !defined(RP_HWSIM_BYPASS)

    /*cold calib*/
    rf_clp_init_seq();

#else // Specific init sequence for RTL simulation

    clp_bank0_padcntl_set(0);

    // Emulate cold calibration results
    clp_bank0_bankselcntl_banksel_setf(0);

    clp_bank0_rftxrxcntl_calib_mode_setf(0);

    // channel 44 for inquiry_scan testcase
    clp_bank0_vcofc_icp_calib_lut_cntl_pack(1,  /* lutvcofcicpwen, */
                                            44, /* uint8_t lutvcofcicpadd, */
                                            1,  /* uint8_t lutldin, */
                                            0,  /* uint16_t luticpkvcoin, */
                                            6,  /* uint8_t luticpin, */
                                            0,  /* uint8_t lutvcofcfinein, */
                                            2); /*uint8_t lutvcofccoarsein)*/
    clp_bank0_vcofc_icp_calib_lut_cntl_lut_vcofc_icp_wen_setf(0);

    // channel 78-79 for slave_connect_disconnect_sleep
    clp_bank0_vcofc_icp_calib_lut_cntl_pack(1,  /* lutvcofcicpwen, */
                                            78, /* uint8_t lutvcofcicpadd, */
                                            1,  /* uint8_t lutldin, */
                                            0,  /* uint16_t luticpkvcoin, */
                                            6,  /* uint8_t luticpin, */
                                            0,  /* uint8_t lutvcofcfinein, */
                                            2); /*uint8_t lutvcofccoarsein)*/
    clp_bank0_vcofc_icp_calib_lut_cntl_lut_vcofc_icp_wen_setf(0);

    clp_bank0_vcofc_icp_calib_lut_cntl_pack(1,  /* lutvcofcicpwen, */
                                            79, /* uint8_t lutvcofcicpadd, */
                                            1,  /* uint8_t lutldin, */
                                            0,  /* uint16_t luticpkvcoin, */
                                            6,  /* uint8_t luticpin, */
                                            0,  /* uint8_t lutvcofcfinein, */
                                            2); /*uint8_t lutvcofccoarsein)*/
    clp_bank0_vcofc_icp_calib_lut_cntl_lut_vcofc_icp_wen_setf(0);

    /*init modem in core*/
    rf_clp_init_mdmhp();

    /*Set Modem in bypass mode */
    rf_clp_init_mdmhp_bypass();

    /*init sequencer */
    rf_clp_sequencers_init();

    // Selection bank 0
    clp_bank0_bankselcntl_banksel_setf(0);


#endif //RP_HWSIM_BYPASS

    // Settings for proper reception
#if defined(CFG_BLE_EMB)
//        ip_radiocntl1_forceiq_setf(1); TODO wait for CLP IQ calibration
    ip_radiocntl1_dpcorr_en_setf(0x0);
    ASSERT_ERR(ip_radiocntl1_dpcorr_en_getf() == 0x0);
#endif // CFG_BLE_EMB

#if  defined(CFG_BT_EMB)
    ip_radiocntl1_dpcorr_en_setf(0x0);
    ASSERT_ERR(ip_radiocntl1_dpcorr_en_getf() == 0x0);
#endif //CFG_BT_EMB

#if defined(CFG_BLE_EMB)
    // Force IQ mode for BLE only
//    ip_radiocntl1_forceiq_setf(1); TODO wait for CLP IQ calibration
#endif //CFG_BLE_EMB

    /* AoA/AoD DF control register settings */
#if defined(CFG_BLE_EMB)
    /* LE-1M timings */
    ble_dfcntl0_1us_rxsampstinst0_1us_setf(0x08);
    ble_dfcntl0_1us_rxswstinst0_1us_setf(0x18);
    ble_dfcntl0_1us_txswstinst0_1us_setf(0x19);

    ble_dfcntl0_2us_rxsampstinst0_2us_setf(0x08);
    ble_dfcntl0_2us_rxswstinst0_2us_setf(0x18);
    ble_dfcntl0_2us_txswstinst0_2us_setf(0x19);

    /* LE-2M timings */
    ble_dfcntl1_1us_rxsampstinst1_1us_setf(0x08);
    ble_dfcntl1_1us_rxswstinst1_1us_setf(0x18);
    ble_dfcntl1_1us_txswstinst1_1us_setf(0x19);

    ble_dfcntl1_2us_rxsampstinst1_2us_setf(0x08);
    ble_dfcntl1_2us_rxswstinst1_2us_setf(0x18);
    ble_dfcntl1_2us_txswstinst1_2us_setf(0x19);

    ble_dfantcntl_pack(/*uint8_t rxprimidcntlen*/ 1, /*uint8_t rxprimantid*/ 0, /*uint8_t txprimidcntlen*/ 1, /*uint8_t txprimantid*/ 0);
#endif //CFG_BLE_EMB


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
    uint16_t rssiptr      = (EM_RF_HW_SPI_OFFSET + 0x10 + 0x10 + 0x10 + 0x20 + 0x20);
    uint16_t rxpkttypptr  = (EM_RF_HW_SPI_OFFSET + 0x10 + 0x10 + 0x10 + 0x20 + 0x20 + 0x20);


#if defined(CFG_BLE_EMB)
    /* TxOn Sequence start pointer */
    ble_spiptrcntl0_txonptr_setf(txonptr >> 2);

    /* TxOff Sequence start pointer */
    ble_spiptrcntl0_txoffptr_setf(txoffptr >> 2);

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
    bt_spiptrcntl0_txoffptr_setf(txoffptr >> 2);

    /* RxOn Sequence start pointer */
    bt_spiptrcntl1_rxonptr_setf(bt_rxonptr >> 2);

    /* RxOff Sequence start pointer */
    bt_spiptrcntl1_rxoffptr_setf(rxoffptr >> 2);

    /* RSSI Sequence start pointer */
    bt_spiptrcntl2_rssiptr_setf(rssiptr >> 2);

    /* Packet Type Sequence start pointer */
    bt_spiptrcntl3_rxpkttypptr_setf(rxpkttypptr >> 2);
#endif // defined CFG_BT_EMB

    /* *************************************************************************************** */
    /* Initialize HW SPI Tx On Chained list  -> 1 structure of 3 words*/
    /* *************************************************************************************** */
    /*  TxON Sequence
                 -> Next PTR = 0x0000 <-> Last SPI frame
                 -> Write Access of 3 bytes = 8'h82
                 -> Address of RFDYNCNTL register = 8'h01 */
    RF_CLP_EM_WR(txonptr, 0x0000);
    RF_CLP_EM_WR(txonptr + 0x2, 0x0182);

    /* *************************************************************************************** */
    /* Initialize HW SPI Tx Off Chained list */
    /* *************************************************************************************** */
    /* TxOFF Sequence
                  -> Next PTR = 0x0000 <-> Last SPI frame
                  -> Write Access of 3 bytes = 8'h82
                  -> Address of RFDYNCNTL register = 8'h01 */

    RF_CLP_EM_WR(txoffptr, 0x0000);
    RF_CLP_EM_WR(txoffptr + 0x2, 0x0182);

    /* *************************************************************************************** */
    /* Initialize HW SPI Rx On Chained list -> 3 structures of 1 word */
    /* *************************************************************************************** */
    /* Initialize HW SPI Rx On Chained list  */
    /*  RxON Sequence*/

    // Work-around for Calypso TC1 bug #12416
    // Read final value of clp_bank0_rftxrxcntl
    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);
    dbg12416 = clp_bank0_rftxrxcntl_get();


#if defined(CFG_BLE_EMB)
    /*           -> Next PTR = (ble_rxonptr+0x8) >> 2
                 -> Write Access of 2 bytes = 8'h81
                 -> Address of RFDYNCNTL register = 8'h01 */
    nextptr = (ble_rxonptr + 0x8) >> 2;
    RF_CLP_EM_WR(ble_rxonptr,      nextptr);
    RF_CLP_EM_WR(ble_rxonptr + 0x2,  0x0181);
    // Writing correlation code not needed in Modem bypass mode
    // Work-around for Calypso TC1 bug #12416
    // Done after RFDYNCNTL access so that there is no impact on RX start delay
    /*           -> Next PTR = 0x0000 <-> Last SPI frame
                 -> Write Access of 4 bytes = 8'h83
                 -> Address of CLP_RFTXRXCNTL register = 8'h2E
                 -> Data = set rxmdm_bypass=1 */
    RF_CLP_EM_WR(ble_rxonptr + 0x8,  0x0000);
    RF_CLP_EM_WR(ble_rxonptr + 0xa,  0x2E83);
    RF_CLP_EM_WR(ble_rxonptr + 0xc, ((dbg12416 & 0x0000FFFF) | 0x0200));
    RF_CLP_EM_WR(ble_rxonptr + 0xe,  dbg12416 >> 16);
#endif // defined CFG_BLE_EMB

#if  defined(CFG_BT_EMB)
    /*           -> Next PTR = (bt_rxonptr+0x8) >> 2
                 -> Write Access of 2 bytes = 8'h81
                 -> Address of RFDYNCNTL register = 8'h01 */
    nextptr = (bt_rxonptr + 0x8) >> 2;
    RF_CLP_EM_WR(bt_rxonptr,      nextptr);
    RF_CLP_EM_WR(bt_rxonptr + 0x2,  0x0181);
    // Writing correlation code not needed in Modem bypass mode
    // Work-around for Calypso TC1 bug #12416
    // Done after RFDYNCNTL access so that there is no impact on RX start delay
    /*           -> Next PTR = 0x0000 <-> Last SPI frame
                 -> Write Access of 4 bytes = 8'h83
                 -> Address of CLP_RFTXRXCNTL register = 8'h2E
                 -> Data = set rxmdm_bypass=1 */
    RF_CLP_EM_WR(bt_rxonptr + 0x8,  0x0000);
    RF_CLP_EM_WR(bt_rxonptr + 0xa,  0x2E83);
    RF_CLP_EM_WR(bt_rxonptr + 0xc, ((dbg12416 & 0x0000FFFF) | 0x0200));
    RF_CLP_EM_WR(bt_rxonptr + 0xe,  dbg12416 >> 16);
#endif // defined CFG_BT_EMB


    /* *************************************************************************************** */
    /* Initialize HW SPI Rx Off Chained list -> 1 structure of 0 words */
    /* *************************************************************************************** */
    /* RxOFF Sequence
                 -> Next PTR = (rxoffptr+0x08) >> 2
                 -> Write Access of 2 bytes = 8'h81
                 -> Address of RFDYNCNTL register = 8'h01*/
    nextptr = (rxoffptr + 0x8) >> 2;
    RF_CLP_EM_WR(rxoffptr, nextptr);
    RF_CLP_EM_WR(rxoffptr + 0x2, 0x0181);
    // Work-around for Calypso TC1 bug #12416
    /*            -> Next PTR = 0x0000 <-> Last SPI frame
                  -> Write Access of 4 bytes = 8'h83
                  -> Address of CLP_RFTXRXCNTL register = 8'h2E
                 -> Data = set rxmdm_bypass=0 */
    RF_CLP_EM_WR(rxoffptr + 0x8,  0x0000);
    RF_CLP_EM_WR(rxoffptr + 0xa,  0x2E83);
    RF_CLP_EM_WR(rxoffptr + 0xc, (dbg12416 & 0x0000FDFF));
    RF_CLP_EM_WR(rxoffptr + 0xe,  dbg12416 >> 16);


    /* *************************************************************************************** */
    /* Initialize HW SPI RSSI Chained list  -> 2 structures of 1 word*/
    /* *************************************************************************************** */
    /* RSSI Sequence
                  -> Next PTR = (rssiptr+0x8) >> 2
                  -> Read Access of 1 byte = 8'h00
                  -> Address of AGC_STAT register = 8'h02*/

    nextptr = (rssiptr + 0x8) >> 2;
    RF_CLP_EM_WR(rssiptr, nextptr);
    RF_CLP_EM_WR(rssiptr + 0x2, 0x0200);

    /* Set RF bank to 0x5
                  -> Next PTR = (rssiptr+0x10) >> 2
                  -> Write Access of 2 bytes = 81'h
                  -> Address of BANK = 8'h7f*/
    nextptr = (rssiptr + 0x10) >> 2;
    RF_CLP_EM_WR(rssiptr + 0x8, nextptr);
    RF_CLP_EM_WR(rssiptr + 0xa, 0x7F81);
    RF_CLP_EM_WR(rssiptr + 0xC, 0x0005);

    /* Read AGC_STAT
                 -> Next PTR = (rssiptr+0x18) >> 2
                 -> Read Access of 1 byte = 8'h00
                 -> Address of STAT_RSSI_EST modem register = 8'h87*/
    nextptr = (rssiptr + 0x18) >> 2;
    RF_CLP_EM_WR(rssiptr + 0x10, nextptr);
    RF_CLP_EM_WR(rssiptr + 0x12, 0x8700);

    /* Set RF bank to 0x0
                 -> Next PTR = 0x0000 <-> Last SPI frame
                 -> Write Access of 1 byte = 80'h
                 -> Address of BANK = 8'h7f*/
    RF_CLP_EM_WR(rssiptr + 0x18, 0x0000);
    RF_CLP_EM_WR(rssiptr + 0x1a, 0x7F81);
    RF_CLP_EM_WR(rssiptr + 0x1c, 0x0000);

    /* *************************************************************************************** */
    /* Initialize HW SPI Rx Packet Type Chained list  -> 1 structure of 1 word*/
    /* *************************************************************************************** */
    /* RXPKTRATE Sequence
                  -> Next PTR = 0x1108
                  -> Write Access of 2 bytes = 8'h81
                  -> Address of RFDYNCNTL register = 8'h01*/

    RF_CLP_EM_WR(rxpkttypptr,     0x0000);
    RF_CLP_EM_WR(rxpkttypptr + 0x2, 0x0181);

    /// Force the calypso bank sel back to zero
    /// Avoiding HW accesses into others bank after the rf init function
    /// SW is not supposed to change the RF bank after rf init function
    clp_bank0_bankselcntl_banksel_setf(0);
    ASSERT_ERR(clp_bank0_bankselcntl_banksel_getf() == 0);

}
///@} RF_CALYPSO
