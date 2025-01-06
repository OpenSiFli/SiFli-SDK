/**
****************************************************************************************
*
* @file rf_icytrxdm.c
*
* @brief IcyTRxDM radio initialization and specific functions
*
* Copyright (C) RivieraWaves 2009-2015
*
* $Rev: $
*
****************************************************************************************
*/

/**
****************************************************************************************
* @addtogroup RF_ICYDM
* @ingroup RF
* @brief IcyTRxDM Radio Driver
*
* This is the driver block for IcyTRxDM radio
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

#include "plf.h"           // Platform register
#include "flash.h"         // Flash interface
#include "lcd.h"           // DBG interface
#include "reg_iqgen.h"     // Add DF Generator register h file here

#if (BLE_EMB_PRESENT)
    #include "reg_blecore.h"   // ble core registers
    #include "reg_em_ble_cs.h" // control structure definitions
#endif //BLE_EMB_PRESENT

#if (BT_EMB_PRESENT)
    #include "reg_btcore.h"    // bt core registers
    #include "reg_em_bt_cs.h"  // control structure definitions
#endif //BT_EMB_PRESENT

/**
****************************************************************************************
 * DEFINES
 ****************************************************************************************
 **/

#define RF_EM_SPI_ADRESS              (EM_BASE_ADDR + EM_RF_SW_SPI_OFFSET)

#define ICYDM_SPIRD                   0x00
#define ICYDM_SPIWR                   0x80

#define ICYDM_PWR_TBL_SIZE            0x10
#define ICYDM_PA_TBL_MSK              0x0F
#define ICYDM_PA_PWR_MSK              0x1F

#define ICYDM_MAX_BURST_SIZE          0x80
#define ICYDM_INIT_TBL_18_SIZE        0xD0
#define ICYDM_INIT_TBL_20_SIZE        0xFC
#define ICYDM_INIT_TBL_30_SIZE        0xF7

#define ICYDM_RSSI_20dB_THRHLD        -20
#define ICYDM_RSSI_40dB_THRHLD        -40
#define ICYDM_RSSI_45dB_THRHLD        -45
#define ICYDM_RSSI_48dB_THRHLD        -48
#define ICYDM_RSSI_55dB_THRHLD        -55
#define ICYDM_RSSI_60dB_THRHLD        -60
#define ICYDM_RSSI_70dB_THRHLD        -70


#define REG_ICYDM_RD                rf_icydm_reg_rd
#define REG_ICYDM_WR                rf_icydm_reg_wr

/* The offset value given below is the offset to add to the frequency table index to
   get the value to be programmed in the radio for each channel                      */
#define ICYDM_FREQTAB_OFFSET          0   // Offset for Calypso radio

// TX max power
#define ICYDM_POWER_MIN                 0
#define ICYDM_POWER_MAX                0xFF
#define BTIPT_POWER_MSK                0x3F

// Workaround RSSI value issues identified in EBQ testing
#define RF_EBQ_RSSI_WORKAROUND          1
/**
****************************************************************************************
* MACROS
*****************************************************************************************
*/
// Max burst register
__STATIC uint8_t rf_icydm_reg_buf[ICYDM_MAX_BURST_SIZE + 8]; // max burst size + buffer controls


// IcyTRxDM register definitions and access functions
__STATIC uint32_t rf_icydm_reg_rd(uint32_t addr);
__STATIC void rf_icydm_reg_wr(uint32_t addr, uint32_t value);


/// IcyTRx EM Write Macro for HW driven SPI chained structures
#define RF_ICTRXDM_EM_WR(addr, val) \
    EM_WR((((uint32_t) (addr)) + REG_EM_ET_BASE_ADDR), (val))


/**
 ****************************************************************************************
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 **/

/* ---------------------------Back to Back YUKON FPGA Init Tables ---------------------------------------------*/

__STATIC  const uint8_t RF_ICYDM_REG_TBL_YUKON_FPGA_INIT[578] =
{
    0x21, 0x00, 0x00, 0x00,    0x00, 0x08, 0x83, 0x01,   0x00, 0x00, 0x03, 0x00,    0x1e, 0x17, 0x1b,  0x00,    // 0x0*
    0x00, 0x1b, 0x1b, 0x1c,    0x1d, 0x1e, 0x00, 0xfc,   0x0f, 0x00, 0x00, 0x00,    0x00, 0x00, 0x00,  0x00,    // 0x1*
    0x00, 0x00, 0x00, 0x00,    0x2d, 0x03, 0x80, 0x00,   0x57, 0xd5, 0x75, 0x00,    0x00, 0x00, 0x00,  0x00,    // 0x2*
    0x00, 0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,    0x00, 0x00, 0x00,  0x00,    // 0x3*
    0x00, 0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00,   0x00, 0x3f, 0xf0, 0x08,    0x08, 0x3f, 0x3f,  0x3f,    // 0x4*
    0x24, 0x1c, 0x11, 0x16,    0x17, 0x1d, 0x13, 0x3f,   0x3f, 0x3f, 0x3f, 0x3f,    0x3f, 0x1a, 0x3f,  0x1f,    // 0x5*
    0x1e, 0x3f, 0x1d, 0x0a,    0x07, 0x3f, 0x1b, 0x14,   0x09, 0x25, 0x20, 0x1d,    0x21, 0x22, 0x18,  0x3f,    // 0x6*
    0x26, 0x06, 0x27, 0x00,    0x00, 0x00, 0xb3, 0x00,   0x00, 0x03, 0x00, 0x02,    0x00, 0x00, 0x00,  0x00,    // 0x7*

    0x00, 0xd2, 0xc6, 0x0f,    0x00, 0x09, 0x00, 0x00,   0x00, 0x00, 0x00, 0x01,    0x00, 0x00, 0x00,  0x00,    // 0x8*
    0x00, 0x00, 0xdf, 0x11,    0x20, 0x00, 0x11, 0x03,   0x11, 0x03, 0x00, 0x00,    0x1b, 0xd0, 0x10,  0x03,    // 0x9*
    0x07, 0xe0, 0x30, 0x00,    0x00, 0x10, 0x13, 0x04,   0x70, 0x00, 0x0f, 0x00,    0x00, 0x01, 0x00,  0xff,    // 0xa*
    0xff, 0x00, 0x03, 0x03,    0x00, 0xf9, 0xf6, 0xfb,   0x0b, 0x23, 0x38, 0x40,    0xf0, 0x00, 0x00,  0x00,    // 0xb*
    0x68, 0x00, 0xb0, 0x04,    0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x01, 0x03,    0x08, 0x12, 0x21,  0x33,    // 0xc*
    0x45, 0x54, 0x5d, 0x60,    0x02, 0x03, 0x01, 0x00,   0xa9, 0x01, 0x00, 0x00,    0x10, 0x00, 0xf0,  0x00,    // 0xd*
    0x71, 0x00, 0xff, 0x00,    0x08, 0x08, 0x00, 0xff,   0x08, 0x08, 0x00, 0x00,    0x15, 0x2c, 0x42,  0x58,    // 0xe*
    0x6f, 0x84, 0x99, 0xac,    0xbe, 0xce, 0xdc, 0xe7,   0xf0, 0xf7, 0xfc, 0x00,    0x00, 0x00, 0x1f,  0x00,    // 0xf*

    0x00, 0x00, 0x00, 0x00,    0x00, 0x25, 0x01, 0x00,   0x00, 0x00, 0x00, 0x00,    0x00, 0x00, 0x00,  0x00,    // 0x10*
    0x00, 0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,    0xd7, 0xfe, 0x9f,  0x8f,    // 0x11*
    0x2e, 0xdb, 0x7e, 0x7d,    0x07, 0xff, 0xa3, 0x97,   0x95, 0xff, 0xff, 0x32,    0x2d, 0xc7, 0xe3,  0xc3,    // 0x12*
    0xb4, 0xff, 0xff, 0xff,    0xff, 0xff, 0xff, 0x82,   0xe0, 0xe5, 0xe4, 0xcb,    0xc3, 0xbc, 0xbf,  0xff,    // 0x13*
    0xff, 0xc3, 0xc9, 0x84,    0x83, 0xde, 0xd4, 0xbd,   0xc7, 0xd2, 0xd3, 0xce,    0xff, 0xbb, 0xc0,  0xb0,    // 0x14*
    0xff, 0xb2, 0xa9, 0xca,    0xbe, 0xff, 0xff, 0x99,   0x13, 0x89, 0xc4, 0xe6,    0xbc, 0x9d, 0xa5,  0x44,    // 0x15*
    0x44, 0x9f, 0x8b, 0x55,    0x44, 0x48, 0x44, 0x4e,   0x48, 0xde, 0xc8, 0xff,    0xff, 0xa1, 0xc8,  0xd4,    // 0x16*
    0x5c, 0xff, 0xff, 0xff,    0xff, 0xff, 0xde, 0xd5,   0xc4, 0xac, 0x80, 0xab,    0xb6, 0xe5, 0xe1,  0xc8,    // 0x17*

    0x0c, 0x00, 0x00, 0x06,    0x00, 0x06, 0x0c, 0x06,   0x0c, 0x00, 0x0c, 0x00,    0x00, 0x00, 0xab,  0x0e,    // 0x18*
    0x00, 0x06, 0x0c, 0x00,    0x18, 0x00, 0x00, 0x02,   0x16, 0x04, 0x00, 0x00,    0x00, 0xf2, 0x0f,  0x21,    // 0x19*
    0x20, 0x10, 0x00, 0x00,    0x01, 0xf0, 0xee, 0xee,   0x1f, 0x74, 0xca, 0xfe,    0x21, 0x74, 0xca,  0xfe,    // 0x1a*
    0x07, 0x00, 0x00, 0x00,    0x28, 0x04, 0x00, 0x02,   0x84, 0x10, 0x00, 0x00,    0x14, 0x28, 0x3c,  0x12,    // 0x1b*
    0x3c, 0x01, 0x32, 0x01,    0xe7, 0x4a, 0x03, 0x00,   0x00, 0x20, 0x10, 0x00,    0x20, 0x00, 0x10,  0x02,    // 0x1c*
    0x14, 0x12, 0x00, 0x1f,    0x00, 0x00, 0x1f, 0x00,   0x8f, 0x00, 0x05, 0x28,    0x01, 0x00, 0x00,  0x00,    // 0x1d*
    0x40, 0x00, 0x02, 0x00,    0x44, 0xfa, 0x14, 0x30,   0x00, 0x33, 0x1f, 0x2f,    0x40, 0x20, 0x10,  0x08,    // 0x1e*
    0x01, 0x1c, 0x22, 0x34,    0x00, 0x00, 0x20, 0x40,   0x00, 0x00, 0x00, 0x00,    0x00, 0x00, 0x00,  0x00,    // 0x1f*

    0x00, 0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00,   0x2f, 0x49, 0xb0, 0x15,    0x00, 0x00, 0x00,  0x00,    // 0x20*
    0x25, 0x00, 0x08, 0x07,    0x00, 0x00, 0x03, 0x00,   0x00, 0x00, 0x00, 0x00,    0xbb, 0x08, 0x00,  0x45,    // 0x21*
    0x40, 0x28, 0x16, 0x02,    0x33, 0xa8, 0x33, 0xa8,   0x45, 0x45, 0x00, 0x40,    0x45, 0x45, 0x45,  0x45,    // 0x22*
    0x2b, 0x10, 0x7f, 0x01,    0xff, 0x03, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,    0x00, 0x00, 0x00,  0x00,    // 0x23*
    0x10, 0x00

};


__STATIC  const uint8_t RF_ICYDM_REG_TBL_YUKON_FPGA_GPIOS[10] =
{
    0x19,  0x16,  0x17,  0x18,
    0x1D,  0x1A,  0x1E,  0x1B,
    0x1C,  0x00
};

__STATIC  const uint8_t RF_ICYDM_REG_TBL_YUKON_FPGA_SEQUENCER[90] =
{
    0x00, 0x70, 0x85, 0x00,    0x31, 0x0a, 0x0a, 0x30,   0x0c, 0x18, 0x6e, 0x31,    0x01, 0x0b, 0x40, 0x8a,  // 0xad*
    0x52, 0x31, 0x03, 0xb4,    0x72, 0x42, 0x40, 0x00,   0x70, 0x85, 0x01, 0x0d,    0x62, 0x31, 0x03, 0x0b,  // 0xae*
    0x4c, 0x00, 0x31, 0x0a,    0x28, 0x30, 0x0c, 0x53,   0x5e, 0x31, 0x01, 0x0b,    0x40, 0x52, 0x31, 0x0c,  // 0xaf*
    0x72, 0x42, 0x42, 0x00,    0x0b, 0x4c, 0x62, 0x31,   0x0c, 0x6e, 0x09, 0x01,    0x62, 0x1a, 0x59, 0x00,  // 0xb0*
    0x5e, 0x48, 0x7f, 0x8a,    0x5e, 0x48, 0x80, 0x82,   0x5e, 0x30, 0x10, 0x0d,    0x6e, 0x30, 0x10, 0x6e,  // 0xb1*
    0x48, 0xff, 0x0d, 0x5e,    0x09, 0x01, 0x52, 0x1a,   0x59, 0x0d                                          // 0xb2*
};


__STATIC  const uint8_t RF_ICYDM_REG_TBL_YUKON_FPGA_TX_PKT_HANDLER[79] =
{
    0x40, 0xa5, 0x32, 0x07,    0x14, 0x48, 0x31, 0x07,   0x0d, 0x30, 0x08, 0x0d,    0x49, 0x37, 0x07, 0x48,  // 0xb5*
    0x02, 0xa0, 0x06, 0x0d,    0x41, 0x16, 0x7e, 0x37,   0x07, 0x48, 0x02, 0xa0,    0x31, 0x08, 0x17, 0x12,  // 0xb6*
    0x08, 0x17, 0x42, 0xa1,    0x43, 0xa0, 0x44, 0x09,   0x00, 0xa5, 0x45, 0x30,    0x08, 0x2f, 0x46, 0x37,  // 0xb7*
    0x07, 0x42, 0x02, 0x34,    0x07, 0x39, 0xa0, 0x06,   0x2f, 0x30, 0x07, 0x3f,    0xa1, 0x06, 0x2f, 0xa2,  // 0xb8*
    0x06, 0x2f, 0x09, 0x00,    0xa5, 0xab, 0x4b, 0xa0,   0xa0, 0x4e, 0x09, 0x00,    0xa0, 0x4f, 0xa0         // 0xb9*

};

__STATIC  const uint8_t RF_ICYDM_REG_TBL_YUKON_FPGA_RX_PKT_HANDLER[104] =
{
    0x71, 0x9c, 0x18, 0x32,    0x07, 0x26, 0x31, 0x08,   0x19, 0x49, 0x1b, 0x2c,    0x49, 0x1b, 0x37, 0xe1,  // 0xbd*
    0x1b, 0x44, 0xff, 0xa4,    0x1b, 0x5b, 0x01, 0x06,   0x22, 0x30, 0x08, 0x22,    0x1b, 0x31, 0x13, 0x1b,  // 0xbe*
    0x33, 0xe2, 0xa7, 0x01,    0x06, 0x22, 0x1a, 0x0f,   0x1b, 0x37, 0xe1, 0x1b,    0x16, 0x06, 0x71, 0xb1,  // 0xbf*
    0x1a, 0x00, 0xa3, 0xbf,    0xa5, 0x31, 0x08, 0x22,   0x17, 0x1c, 0x84, 0x1b,    0x62, 0x20, 0x1c, 0x10,  // 0xc0*
    0x1b, 0x2a, 0x52, 0x1c,    0x10, 0x1b, 0x44, 0x25,   0x1c, 0x10, 0x1b, 0x18,    0x0b, 0x30, 0x08, 0x54,  // 0xc1*
    0x71, 0xb1, 0x1a, 0x01,    0xaf, 0x1b, 0x48, 0x15,   0x34, 0x08, 0x22, 0x30,    0x07, 0x63, 0x1b, 0x5f,  // 0xc2*
    0xa1, 0x06, 0x22, 0x1b,    0x5f, 0xa2, 0x06, 0x22                                                        // 0xc3*

};


__STATIC  const uint8_t RF_ICYDM_REG_TBL_YUKON_FPGA_AGC[428] =
{
    0x99, 0x89, 0x98, 0xBB,    0xA9,  0x9A,  0x22,  0x49,    0x94,  0x55, 0x69, 0x96,    0x77, 0x29, 0xA2, 0x99, // 0xc5*
    0x2C, 0xBE, 0xD9, 0xAF,    0xFD,  0x11,  0xA,  0x0,    0x1,  0x4F, 0xC4, 0x55,    0x99, 0xD3, 0xE9, 0xBB,    // 0xc6*
    0xDF, 0x33, 0x29, 0xA2,    0x18,  0x8B,  0x40,  0x0,    0x60,  0x96, 0x77, 0x49,    0x40, 0x0, 0x50, 0xA5,   // 0xc7*
    0x7B, 0xB, 0x0, 0x19,    0x1D,  0xF0,  0xFB,  0x3D,    0xC3,  0x24, 0x5E, 0x21,    0x2, 0x10, 0x11, 0x53,    // 0xc8*
    0x5E, 0x8F, 0x33, 0x29,    0xA2,  0xE9,  0x5B,  0xE5,    0x88,  0xAC, 0xB7, 0x55,    0xBA, 0xB7, 0x0, 0x10,  // 0xc9*
    0xF0, 0x33, 0xBC, 0xD7,    0x46,  0x6E,  0x12,  0x53,    0x6E,  0x8F, 0x33, 0x19,    0x31, 0x55, 0xA, 0x0,   // 0xca*
    0x1, 0x3F, 0xC3, 0x3B,    0x7D,  0xE4,  0x37,  0x31,    0xE4,  0xF7, 0x18, 0x31,    0x99, 0xA, 0x0, 0xEB,    // 0xcb*
    0xBE, 0xF5, 0x55, 0xA,    0x0,  0x7B,  0x42,  0x0,    0x5B,  0x9F, 0xA9, 0x0,    0xB0, 0x23, 0x4, 0xB0,      // 0xcc*

    0xCB, 0xDD, 0xA, 0x0,    0x4B,  0x2E,  0xBE,  0x0,    0x50,  0xA, 0x1, 0x9F,    0x99, 0x88, 0xBC, 0x9B,      // 0xcd*
    0xAA, 0x2C, 0x61, 0xF9,    0xAF,  0xFF,  0x0,  0x40,    0x94,  0x55, 0x69, 0x96,    0x77, 0x9, 0x0, 0x0,     // 0xce*
    0x0, 0x0, 0x0, 0x0,    0x0,  0x0,  0x0,  0x0,    0x0,  0x0, 0x0, 0x0,    0x0, 0x0, 0x0, 0x0,                 // 0xcf*
    0x0, 0x0, 0x0, 0x0,    0x0,  0x0,  0x0,  0x0,    0x0,  0x0, 0x0, 0x0,    0x0, 0x0, 0x0, 0x0,                 // 0xd0*
    0x0, 0x0, 0x0, 0x0,    0x0,  0x0,  0x0,  0x0,    0x0,  0x0, 0x0, 0x0,    0x0, 0x0, 0x0, 0x0,                 // 0xd1*
    0x0, 0x0, 0x0, 0x0,    0x0,  0x0,  0x0,  0x0,    0x0,  0x0, 0x0, 0x0,    0x0, 0x0, 0x0, 0x0,                 // 0xd2*
    0x0, 0x0, 0x0, 0x0,    0x0,  0x0,  0x0,  0x0,    0x0,  0x0, 0x0, 0x0,    0x0, 0x0, 0x0, 0x0,                 // 0xd3*
    0x0, 0x0, 0x0, 0x0,    0x0,  0x0,  0x0,  0x0,    0x0,  0x0, 0x0, 0x0,    0x0, 0x0, 0x0, 0x0,                 // 0xd4*

    0x0, 0x0, 0x0, 0x0,    0x0,  0x0,  0x0,  0x0,    0x0,  0x0, 0x0, 0x0,    0x0, 0x0, 0x0, 0x0,                 // 0xd5*
    0x0, 0x0, 0x0, 0x0,    0x0,  0x0,  0x0,  0x0,    0x0,  0x0, 0x0, 0x0,    0x0, 0x0, 0x0, 0x0,                 // 0xd6*
    0x0, 0x0, 0x0, 0x0,    0x0,  0x0,  0x0,  0x0,    0x0,  0x0, 0x0, 0x0,    0x0, 0x0, 0x0, 0x0,                 // 0xd7*
    0x0, 0x0, 0x0, 0x0,    0x0,  0x0,  0x0,  0x0,    0x0,  0x0, 0x0, 0x0,    0x0, 0x0, 0x0, 0x0,                 // 0xd8*
    0x0, 0x0, 0x0, 0x0,    0x0,  0x0,  0x0,  0x0,    0x0,  0x0, 0x0, 0x0,    0x0, 0x0, 0x0, 0x0,                 // 0xd9*
    0x0, 0x0, 0x0, 0x0,    0x0,  0x0,  0x0,  0x0,    0x0,  0x0, 0x0, 0x0,    0x0, 0x0, 0x0, 0x0,                 // 0xda*
    0x0, 0x0, 0x0, 0x0,    0x0,  0x0,  0x0,  0x0,    0x0,  0x0, 0x0, 0x0,    0x0, 0x0, 0x0, 0x0,                 // 0xdb*
    0x0, 0x0, 0x0, 0x0,    0x0,  0x0,  0x0,  0x0,    0x0,  0x0, 0x0, 0x0,    0x0, 0x0, 0x0, 0x0,                 // 0xdc*

    0x0, 0x78, 0x5C, 0x78,    0x78,  0x78,  0x0,  0x65,    0x78,  0x0, 0x78, 0x78,    0x78, 0x10, 0x22, 0x23,    // 0xdd*
    0x23, 0x35, 0x35, 0x40,    0x40,  0x40,  0x40,  0x40,    0x40,  0x5B, 0x5B, 0x78,    0x4A, 0x78, 0x78, 0x78, // 0xde*
    0x0, 0x1, 0x0, 0x0,    0x23,  0x10,  0x10,  0x32,    0xD,  0x0, 0x0, 0x0                                     // 0xdf*


};

/* --------------------------- TestChip v1 Init Tables ---------------------------------------------*/

__STATIC  const uint8_t RF_ICYDM_REG_TBL_10_INIT[524] =
{
    0xb1, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,   // 0x00*
    0x00, 0x00, 0xfc, 0x0f,   0x00, 0x03, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x2d,   // 0x01*
    0x03, 0x80, 0x00, 0x55,   0x55, 0x55, 0x00, 0x08,   0x02, 0x03, 0x04, 0x24,   0x1c, 0x11, 0x16, 0x17,   // 0x02*
    0x1d, 0x13, 0x0b, 0x0c,   0x0d, 0x0e, 0x0f, 0x10,   0x1a, 0x12, 0x1f, 0x1e,   0x15, 0x1d, 0x0a, 0x07,   // 0x03*
    0x19, 0x1b, 0x14, 0x09,   0x25, 0x20, 0x1d, 0x21,   0x22, 0x18, 0x23, 0x26,   0x06, 0x27, 0x00, 0x28,   // 0x04*
    0x00, 0x00, 0xb3, 0x00,   0xf0, 0x08, 0x00, 0x03,   0x00, 0x02, 0x00, 0x00,   0x00, 0x00, 0xc6, 0x0f,   // 0x05*
    0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0xdf,   0x11, 0x20, 0x00, 0x11,   // 0x06*
    0x03, 0x11, 0x03, 0x00,   0x00, 0x1a, 0xd0, 0x10,   0x07, 0xe0, 0x30, 0x00,   0x10, 0x13, 0x00, 0x70,   // 0x07*

    0x00, 0x0f, 0x00, 0x01,   0x00, 0xff, 0xff, 0x00,   0x03, 0x03, 0x00, 0xf9,   0xf6, 0xfb, 0x0b, 0x23,   // 0x08*
    0x38, 0x40, 0xf0, 0x00,   0x00, 0x68, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,   0x00, 0x01, 0x03, 0x08,   // 0x09*
    0x12, 0x21, 0x33, 0x45,   0x54, 0x5d, 0x60, 0x02,   0x03, 0x01, 0xd2, 0x09,   0x01, 0xb0, 0x04, 0xa9,   // 0x0a*
    0x01, 0x00, 0x00, 0x10,   0x00, 0xf0, 0x3f, 0x40,   0x04, 0x0f, 0x01, 0xff,   0x08, 0x08, 0x00, 0xff,   // 0x0b*
    0x15, 0x2c, 0x42, 0x58,   0x6f, 0x84, 0x99, 0xac,   0xbe, 0xce, 0xdc, 0xe7,   0xf0, 0xf7, 0xfc, 0x0b,   // 0x0c*
    0x00, 0x00, 0x1f, 0x00,   0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,   0x25, 0x00, 0x00, 0x00,   // 0x0d*
    0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x40, 0x2e,   0x7f, 0x23, 0x17, 0x15,   0x7f, 0x7f, 0x7f, 0x7f,   // 0x0e*
    0x47, 0x60, 0x43, 0x34,   0x7f, 0x7f, 0x7f, 0x7f,   0x7f, 0x7f, 0x02, 0x5d,   0x62, 0x61, 0x4b, 0x43,   // 0x0f*

    0x3c, 0x3f, 0x7f, 0x7f,   0x43, 0x49, 0x04, 0x03,   0x5c, 0x52, 0x3d, 0x47,   0x50, 0x51, 0x4c, 0x7f,   // 0x10*
    0x3b, 0x40, 0x30, 0x7f,   0x32, 0x29, 0x4a, 0x3e,   0x7f, 0x7f, 0x19, 0x13,   0x09, 0x44, 0x63, 0x3c,   // 0x11*
    0x1d, 0x25, 0x7f, 0x7f,   0x1f, 0x0b, 0x53, 0x44,   0x48, 0x44, 0x4c, 0x7f,   0x5c, 0x48, 0x7f, 0x7f,   // 0x12*
    0x21, 0x48, 0x52, 0x5a,   0x7f, 0x7f, 0x7f, 0x7f,   0x7f, 0x5c, 0x53, 0x44,   0x2c, 0x00, 0x2b, 0x36,   // 0x13*
    0x62, 0x5e, 0x48, 0x01,   0x00, 0x0c, 0x00, 0x00,   0x06, 0x00, 0x06, 0x0c,   0x06, 0x0c, 0x00, 0x0c,   // 0x14*
    0x00, 0x00, 0x00, 0x00,   0x06, 0x0c, 0x00, 0x18,   0x00, 0x00, 0x00, 0x00,   0xab, 0x0e, 0x16, 0x04,   // 0x15*
    0x00, 0x00, 0x00, 0x02,   0x00, 0xf2, 0x0f, 0x21,   0x20, 0x10, 0x01, 0xf0,   0xee, 0xee, 0x1f, 0x74,   // 0x16*
    0xca, 0xfe, 0x00, 0x21,   0x74, 0xca, 0xfe, 0x00,   0x07, 0x00, 0x00, 0x28,   0x04, 0x00, 0x02, 0x84,   // 0x17*

    0x10, 0x00, 0x00, 0x00,   0x14, 0x28, 0x3c, 0x12,   0x24, 0x01, 0x32, 0x01,   0xe7, 0x4a, 0x03, 0x00,   // 0x18*
    0x00, 0x20, 0x10, 0x20,   0x00, 0x10, 0x14, 0x12,   0x00, 0x1f, 0x00, 0x00,   0x1f, 0x00, 0x00, 0x8f,   // 0x19*
    0x05, 0x28, 0x02, 0x01,   0x00, 0x00, 0x40, 0x00,   0x02, 0x00, 0x44, 0xfa,   0x14, 0x30, 0x00, 0x00,   // 0x1a*
    0x33, 0x40, 0x20, 0x10,   0x08, 0x00, 0x01, 0x15,   0x1b, 0x38, 0x00, 0x00,   0x2f, 0x20, 0x00, 0x00,   // 0x1b*
    0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,   0x2f, 0x49, 0x00, 0x00,   // 0x1c*
    0xf8, 0x02, 0x00, 0x77,   0x00, 0x27, 0x00, 0x08,   0x34, 0x00, 0x8c, 0x00,   0x1b, 0x07, 0x00, 0x77,   // 0x1d*
    0x77, 0x01, 0x40, 0x28,   0x16, 0x02, 0xa4, 0x33,   0xa8, 0x33, 0xa8, 0x33,   0xa7, 0x33, 0xa7, 0x0c,   // 0x1e*
    0x64, 0x33, 0xa8, 0x33,   0xa8, 0x33, 0xa8, 0x33,   0xa8, 0x33, 0xa8, 0x2b,   0xff, 0x01, 0xff, 0x03,   // 0x1f*

    0x10, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x10                              // 0x20*

};


__STATIC  const uint8_t RF_ICYDM_REG_TBL_10_GPIOS[10] =
{
    0x1c,  0x19,  0x1d,  0x2d,
    0x28,  0x29,  0x18,  0x15,
    0x2e,  0x2f
};

__STATIC  const uint8_t RF_ICYDM_REG_TBL_10_SEQUENCER[103] =
{
    0x00, 0x31, 0x0a, 0x07,     0x30, 0x0c, 0x14, 0x6e,     0x31, 0x01, 0x0b, 0x4d,     0x8a, 0x52, 0x00, 0x03,   // 0xad*
    0x72, 0x0d, 0x40, 0x00,     0x0d, 0x0b, 0x59, 0x62,     0x00, 0x03, 0x00, 0x31,     0x0a, 0x21, 0x30, 0x0c,   // 0xae*
    0x60, 0x5e, 0x31, 0x01,     0x51, 0xe1, 0x0f, 0x0b,     0x4d, 0x52, 0x00, 0x0c,     0x51, 0xda, 0x13, 0x8a,   // 0xaf*
    0x61, 0xe1, 0x08, 0x8a,     0x72, 0x0d, 0x42, 0x00,     0x72, 0x0d, 0x43, 0x0b,     0x59, 0x62, 0x00, 0x0c,   // 0xb0*
    0x61, 0xda, 0x13, 0x6e,     0x09, 0x01, 0x61, 0xe1,     0x0f, 0x61, 0xde, 0x59,     0x00, 0x5e, 0x4b, 0x7f,   // 0xb1*
    0x8a, 0x5e, 0x4b, 0x80,     0x82, 0x5e, 0x30, 0x10,     0x0d, 0x6e, 0x30, 0x10,     0x6e, 0x4b, 0xff, 0x0d,   // 0xb2*
    0x5e, 0x09, 0x01, 0x51,     0xde, 0x19, 0x0d                                                                  // 0xb3*
};


__STATIC  const uint8_t RF_ICYDM_REG_TBL_10_TX_PKT_HANDLER[104] =
{
    0x40, 0xa5, 0x32, 0x07,     0x14, 0x48, 0x31, 0x07,     0x0d, 0x30, 0x08, 0x0d,     0x49, 0x37, 0x07, 0x61,   // 0xb5*
    0x02, 0xa0, 0x06, 0x0d,     0x41, 0x16, 0x7e, 0x37,     0x07, 0x61, 0x02, 0xa0,     0x31, 0x08, 0x17, 0x12,   // 0xb6*
    0x08, 0x17, 0x30, 0x08,     0x40, 0x42, 0x02, 0x02,     0x02, 0xa0, 0x02, 0x02,     0x02, 0xa0, 0x43, 0x02,   // 0xb7*
    0x02, 0x02, 0xa0, 0x44,     0x09, 0x00, 0x02, 0x02,     0x02, 0xa5, 0x02, 0x02,     0x02, 0x46, 0x06, 0x54,   // 0xb8*
    0x42, 0x02, 0x02, 0xa0,     0x02, 0x02, 0xa0, 0x43,     0xa0, 0x02, 0x02, 0x44,     0x09, 0x00, 0xa5, 0x02,   // 0xb9*
    0x02, 0x02, 0x02, 0x45,     0x37, 0x07, 0x5b, 0x02,     0xa0, 0x06, 0x54, 0x09,     0x00, 0xa5, 0xab, 0x4b,   // 0xba*
    0xa0, 0x4e, 0x09, 0x00,     0xa2, 0x4f, 0xa0, 0x00                                                            // 0xbb*

};


__STATIC  const uint8_t RF_ICYDM_REG_TBL_10_RX_PKT_HANDLER[90] =
{
    0x71, 0x64, 0x1a, 0x32,     0x07, 0x26, 0x31, 0x08,     0x19, 0x49, 0x1d, 0x2c,     0x49, 0x1d, 0x37, 0xe1,   // 0xbd*
    0x1d, 0x44, 0xff, 0xa4,     0x1d, 0x59, 0x01, 0x06,     0x22, 0x30, 0x08, 0x22,     0x1d, 0x31, 0x13, 0x1d,   // 0xbe*
    0x33, 0xe2, 0xa7, 0x01,     0x06, 0x22, 0x1c, 0x0f,     0x1d, 0x37, 0xe1, 0x1d,     0x16, 0x06, 0x71, 0x81,   // 0xbf*
    0x1c, 0x00, 0xa3, 0xbf,     0xa5, 0x31, 0x08, 0x22,     0x19, 0x1e, 0x84, 0x1d,     0x5f, 0x20, 0x1e, 0x10,   // 0xc0*
    0x1d, 0x2a, 0x52, 0x1e,     0x10, 0x1d, 0x44, 0x25,     0x1e, 0x10, 0x1d, 0x18,     0x0b, 0x30, 0x08, 0x54,   // 0xc1*
    0x71, 0x81, 0x1c, 0x01,     0xaf, 0x1d, 0x48, 0x15,     0x06, 0x22
};


__STATIC  const uint8_t RF_ICYDM_REG_TBL_10_AGC[428] =
{
    0xA7, 0x9F, 0x99, 0x88,    0x89,  0xB1,  0xBB,  0xA9,    0x9A,  0x22, 0x49, 0x94,    0x55, 0x69, 0x96, 0x77, // 0xc5*
    0x29, 0xA2, 0x99, 0xBC,    0xCB,  0xE2,  0x9B,  0xFD,    0xDA,  0x1F, 0xA1, 0x00,    0x10, 0xF0, 0x9F, 0x4D, // 0xc6*
    0xC4, 0x55, 0x99, 0xD3,    0xE9,  0xBB,  0xDF,  0x33,    0x29,  0xA2, 0x18, 0x8B,    0x40, 0x00, 0x60, 0x96, // 0xc7*
    0x77, 0x49, 0x40, 0x00,    0x50,  0xA5,  0x7B,  0xFB,    0xB0,  0x00, 0x90, 0xD1,    0x1F, 0x1D, 0xF0, 0xFB, // 0xc8*
    0x3D, 0xC3, 0x24, 0x5E,    0x21,  0x02,  0x40,  0x11,    0x5E,  0x37, 0xE5, 0xFE,    0x38, 0x93, 0xE5, 0x27, // 0xc9*
    0xA2, 0xE9, 0x5B, 0xE5,    0x88,  0xAC,  0xB7,  0x55,    0xBA,  0xB7, 0x0F, 0x0B,    0x00, 0x2F, 0x1D, 0xF0, // 0xca*
    0x33, 0xFC, 0xD6, 0x7B,    0x6D,  0xE4,  0x2C,  0xE1,    0x76,  0x53, 0xEE, 0x8F,    0x33, 0x69, 0x7E, 0x11, // 0xcb*
    0x53, 0xA5, 0x0F, 0x0B,    0x00,  0x8F,  0xFD,  0xD4,    0x01,  0x3F, 0xC3, 0x3B,    0x7D, 0xE4, 0x41, 0x31, // 0xcc*

    0xE0, 0x05, 0xF0, 0xDE,    0x7E,  0xE7,  0x8F,  0xE7,    0xF7,  0xD1, 0x11, 0x93,    0xA9, 0x0F, 0x0B, 0x00, // 0xcd*
    0xEB, 0xBE, 0xF5, 0x55,    0x0A,  0x00,  0x7B,  0x42,    0x00,  0x5B, 0x9F, 0xA9,    0x00, 0xB0, 0x23, 0x04, // 0xce*
    0xB0, 0xCB, 0xDD, 0x0A,    0x00,  0x4B,  0x2E,  0xBE,    0x00,  0xF0, 0x08, 0x01,    0x9F, 0x99, 0x88, 0xBC, // 0xcf*
    0x9B, 0xAA, 0x2C, 0x61,    0xF9,  0xAF,  0xFF,  0x4C,    0xD7,  0x75, 0x6E, 0xF7,    0x77, 0x00, 0x40, 0xC4, // 0xd0*
    0x55, 0x69, 0x96, 0x77,    0x09,  0x00,  0x00,  0x00,    0x00,  0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00, // 0xd1*
    0x00, 0x00, 0x00, 0x00,    0x00,  0x00,  0x00,  0x00,    0x00,  0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00, // 0xd2*
    0x00, 0x00, 0x00, 0x00,    0x00,  0x00,  0x00,  0x00,    0x00,  0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00, // 0xd3*
    0x00, 0x00, 0x00, 0x00,    0x00,  0x00,  0x00,  0x00,    0x00,  0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00, // 0xd4*

    0x00, 0x00, 0x00, 0x00,    0x00,  0x00,  0x00,  0x00,    0x00,  0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00, // 0xd5*
    0x00, 0x00, 0x00, 0x00,    0x00,  0x00,  0x00,  0x00,    0x00,  0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00, // 0xd6*
    0x00, 0x00, 0x00, 0x00,    0x00,  0x00,  0x00,  0x00,    0x00,  0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00, // 0xd7*
    0x00, 0x00, 0x00, 0x00,    0x00,  0x00,  0x00,  0x00,    0x00,  0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00, // 0xd8*
    0x00, 0x00, 0x00, 0x00,    0x00,  0x00,  0x00,  0x00,    0x00,  0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00, // 0xd9*
    0x00, 0x00, 0x00, 0x00,    0x00,  0x00,  0x00,  0x00,    0x00,  0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00, // 0xda*
    0x00, 0x00, 0x00, 0x00,    0x00,  0x00,  0x00,  0x00,    0x00,  0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00, // 0xdb*
    0x00, 0x00, 0x00, 0x00,    0x00,  0x00,  0x00,  0x00,    0x00,  0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00, // 0xdc*

    0x00, 0x84, 0x72, 0x84,    0x84,  0x84,  0x00,  0x7F,    0x84,  0x00, 0x84, 0x84,    0x84, 0x13, 0x27, 0x28, // 0xdd*
    0x28, 0x3E, 0x3E, 0x4E,    0x4E,  0x4F,  0x4F,  0x4F,    0x4F,  0x71, 0x71, 0x84,    0x60, 0x84, 0x84, 0x84, // 0xde*
    0x00, 0x01, 0x00, 0x00,    0x23,  0x10,  0x10,  0x32,    0x0D,  0x00, 0x00, 0x00                             // 0xdf*


};

__STATIC  const uint8_t RF_ICYDM_REG_TBL_10_APLL[77] =
{
    0x55, 0x05, 0x19, 0x55,   0x15, 0xaa, 0x02, 0x00,   0x00, 0x00, 0x1c, 0xc9,   0x91, 0x1e, 0x9e, 0x73,   // 0xe0*
    0x12, 0x10, 0x59, 0x05,   0x10, 0x01, 0x00, 0x80,   0x01, 0x2e, 0x80, 0x6f,   0x5b, 0x12, 0x0a, 0x00,   // 0xe1*
    0x00, 0x01, 0x00, 0x00,   0x17, 0x4f, 0x8f, 0x64,   0xaf, 0x00, 0x00, 0x00,   0x00, 0x21, 0x13, 0x01,   // 0xe2*
    0x23, 0x00, 0x00, 0x00,   0x14, 0x12, 0x71, 0x08,   0x15, 0x08, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,   // 0xe3*
    0x00, 0x00, 0x00, 0x00,   0x33, 0xa8, 0x33, 0xa8,   0x33, 0xa8, 0x74, 0xcf,   0x03                      // 0xe4*
};

__STATIC  const uint8_t RF_ICYDM_REG_TBL_10_ENDINIT[4] =
{
    0xA1, 0x00, 0x00, 0x08
};

/* IcyTRx dynamic register */
enum
{
    ICYDM_RATE_TX        = 0x800,
    ICYDM_CHANNEL_RM_TX  = 0x801,
    ICYDM_PA_PWR_RM      = 0x802,
    ICYDM_FSM_MODE_RM_TX = 0x803,
    ICYDM_RATE_RX        = 0x810,
    ICYDM_CHANNEL_RM_RX  = 0x811,
    ICYDM_FSM_MODE_RM_RX = 0x812,
    ICYDM_SYNC_WORD_LOW  = 0x813,
    ICYDM_SYNC_WORD_HIGH = 0x817,
    ICYDM_RX_POWER_OFF   = 0x820,
    ICYDM_RSSI_READYOUT  = 0x824,
    ICYDM_PACKET_LEN     = 0x828
};

// Power table - ref. Table 10, icyTRXDigital_architecture_3v3.docx
__STATIC const int8_t RF_ICYDM_TX_PW_CONV_TBL[ICYDM_PWR_TBL_SIZE] =
{
    [0xD/*-3*/] = -40,
    [0xE/*-2*/] = -35,
    [0xF/*-1*/] = -30,
    [0] = -18,
    [1] = -16,
    [2] = -14,
    [3] = -13,
    [4] = -11,
    [5] = -10,
    [6] = -8,
    [7] = -7,
    [8] = -5,
    [9] = -4,
    [10] = -2,
    [11] = -1,
    [12] = 1,
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
__STATIC void rf_icydm_spi_tf(void)
{
    //launch SPI transfer
    ip_radiocntl0_spigo_setf(1);

    //wait for transfer to be completed
    while (!ip_radiocntl0_spicomp_getf());
}

/**
 ****************************************************************************************
 * @brief IcyTRxDM specific read access
 *
 * @param[in] addr    register address
 *
 * @return uint32_t value
 *****************************************************************************************
 */
__STATIC uint32_t rf_icydm_reg_rd(uint32_t addr)
{
    // Next Pointr to 0x0
    rf_icydm_reg_buf[0] = (uint8_t)(0);  // Next Pointer set to 0x0000 to stop the SPI Chained access
    rf_icydm_reg_buf[1] = (uint8_t)(0);

    //copy control and number of u32 to send
    rf_icydm_reg_buf[2] = (uint8_t)(ICYDM_SPIRD + 1);

    //copy address
    rf_icydm_reg_buf[3] = (uint8_t)(addr & 0x00FF);
    rf_icydm_reg_buf[4] = (uint8_t)((addr & 0xFF00) >> 8);

    // Padding
    rf_icydm_reg_buf[5] = (uint8_t)(0);
    rf_icydm_reg_buf[6] = (uint8_t)(0);
    rf_icydm_reg_buf[7] = (uint8_t)(0);

    memcpy((void *)RF_EM_SPI_ADRESS, rf_icydm_reg_buf, 8);

    //do the transfer
    rf_icydm_spi_tf();

    return (uint32_t)(*((uint8_t *)(RF_EM_SPI_ADRESS + 8)));
}

/**
 ****************************************************************************************
 * @brief IcyTRxDM specific write access
 *
 * @param[in] addr    register address
 * @param[in] value   value to write
 *
 * @return uint32_t value
 ****************************************************************************************
 */
__STATIC void rf_icydm_reg_wr(uint32_t addr, uint32_t value)
{
    rf_icydm_reg_buf[0] = (uint8_t)(0);  // Next Pointer set to 0x0000 to stop the SPI Chained access
    rf_icydm_reg_buf[1] = (uint8_t)(0);

    //inversion for EM reading by U8 on ATL SPI side
    //copy control and number of u32 to send
    rf_icydm_reg_buf[2] = (uint8_t)(ICYDM_SPIWR + 1);

    //copy address
    rf_icydm_reg_buf[3] = (uint8_t)(addr & 0x00FF);
    rf_icydm_reg_buf[4] = (uint8_t)((addr & 0xFF00) >> 8);

    // Padding
    rf_icydm_reg_buf[5] = (uint8_t)(0);
    rf_icydm_reg_buf[6] = (uint8_t)(0);
    rf_icydm_reg_buf[7] = (uint8_t)(0);

    // Byte to be written
    rf_icydm_reg_buf[8] = (uint8_t)value;

    memcpy((void *)RF_EM_SPI_ADRESS, rf_icydm_reg_buf, 9);

    //do the transfer
    rf_icydm_spi_tf();
}

/**
 ****************************************************************************************
 * @brief IcyTRxDM specific read access
 *
 * @param[in] addr    register address
 * @param[in] size    transfer size
 * @param[in] data    pointer to the data array
 *
 * @return uint32_t value
 ****************************************************************************************
 **/
__STATIC void rf_icydm_reg_burst_wr(uint16_t addr, uint8_t size, uint8_t *data)
{
    rf_icydm_reg_buf[0] = (uint8_t)(0);  // Next Pointer set to 0x0000 to stop the SPI Chained access
    rf_icydm_reg_buf[1] = (uint8_t)(0);

    //copy control and number of u8 to send
    rf_icydm_reg_buf[2] = (uint8_t)(ICYDM_SPIWR + size);

    //copy address
    rf_icydm_reg_buf[3] = (uint8_t)(addr & 0x00FF);
    rf_icydm_reg_buf[4] = (uint8_t)((addr & 0xFF00) >> 8);

    // Padding
    rf_icydm_reg_buf[5] = (uint8_t)(0);
    rf_icydm_reg_buf[6] = (uint8_t)(0);
    rf_icydm_reg_buf[7] = (uint8_t)(0);

    for (int i = 0; i < size + 2; i++)
    {
        rf_icydm_reg_buf[i + 8] = *(data + i);
    }

    memcpy((void *)RF_EM_SPI_ADRESS, rf_icydm_reg_buf, 8 + size);

    //do the transfer
    rf_icydm_spi_tf();
}

/**
 *****************************************************************************************
 * @brief Init Frequency Table.in Exchange Memory
 *****************************************************************************************
 */
static void rf_em_init(void)
{
    uint8_t idx = 0;
    uint8_t temp_freq_tbl[EM_RF_FREQ_TABLE_LEN];

#if BT_EMB_PRESENT
    // First half part of frequency table is for the even frequencies
    while (idx < EM_RF_FREQ_TABLE_LEN / 2)
    {
        temp_freq_tbl[idx] = 2 * idx + ICYDM_FREQTAB_OFFSET;
        idx++;
    }
    while (idx < EM_RF_FREQ_TABLE_LEN)
    {
        temp_freq_tbl[idx] = 2 * (idx - (EM_RF_FREQ_TABLE_LEN / 2)) + 1 + ICYDM_FREQTAB_OFFSET;
        idx++;
    }
#elif BLE_EMB_PRESENT
    while (idx < EM_RF_FREQ_TABLE_LEN)
    {
        temp_freq_tbl[idx] = 2 * idx + ICYDM_FREQTAB_OFFSET;
        idx++;
    }
#endif // BT_EMB_PRESENT/BLE_EMB_PRESENT
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
    // power table
    return (RF_ICYDM_TX_PW_CONV_TBL[txpwr_idx & ICYDM_PA_TBL_MSK]);
}

/**
 *****************************************************************************************
 * @brief Sleep function for IcyTRxDM RF.
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
#if RF_EBQ_RSSI_WORKAROUND
    RssidBm += 20;
#endif // RF_EBQ_RSSI_WORKAROUND
    return (RssidBm);
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
    uint8_t tx_pwr = em_bt_pwrcntl_txpwr_getf(EM_BT_CS_ACL_INDEX(link_id)) & BTIPT_POWER_MSK;

    if (tx_pwr > ICYDM_POWER_MIN)
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

    if (tx_pwr < ICYDM_POWER_MAX)
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
    em_bt_pwrcntl_txpwr_setf(EM_BT_CS_ACL_INDEX(link_id), ICYDM_POWER_MAX);
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
    ASSERT_ERR(option <= TXPWR_CS_NEAREST);

    uint8_t i;

    for (i = ICYDM_POWER_MIN; i < ICYDM_POWER_MAX; i++)
    {
        // Loop until we find a power higher than or equal to the requested one
        if (RF_ICYDM_TX_PW_CONV_TBL[i & ICYDM_PA_TBL_MSK] >= txpwr_dbm)
            break;
    }

    if ((RF_ICYDM_TX_PW_CONV_TBL[i & ICYDM_PA_TBL_MSK] > txpwr_dbm) && (i > ICYDM_POWER_MIN))
    {
        if ((option == TXPWR_CS_LOWER)
                || ((option == TXPWR_CS_NEAREST) && (co_abs(txpwr_dbm - RF_ICYDM_TX_PW_CONV_TBL[(i - 1) & ICYDM_PA_TBL_MSK]) < co_abs(txpwr_dbm - RF_ICYDM_TX_PW_CONV_TBL[i & ICYDM_PA_TBL_MSK]))))
        {
            i--;
        }
    }

    return (i & ICYDM_PA_PWR_MSK);
}

/**
 ****************************************************************************************
 * RADIO FUNCTION INTERFACE
 ****************************************************************************************
 **/
void rf_init(struct rwip_rf_api *api)
{
    uint32_t icydm_chip_id = 0;  // Default version is IcyTRXDM
    uint8_t length = PARAM_LEN_RSSI_THR;

    // Initialize the RF driver API structure
    api->reg_rd = rf_icydm_reg_rd;
    api->reg_wr = rf_icydm_reg_wr;
    api->txpwr_dbm_get = rf_txpwr_dbm_get;
    api->txpwr_min = ICYDM_POWER_MIN;
    api->txpwr_max = ICYDM_POWER_MAX;
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
        api->rssi_high_thr = (int8_t)ICYDM_RSSI_40dB_THRHLD;
    }
    if (rwip_param.get(PARAM_ID_RSSI_LOW_THR, &length, (uint8_t *)&api->rssi_low_thr) != PARAM_OK)
    {
        api->rssi_low_thr = (int8_t)ICYDM_RSSI_60dB_THRHLD;
    }
    if (rwip_param.get(PARAM_ID_RSSI_INTERF_THR, &length, (uint8_t *)&api->rssi_interf_thr) != PARAM_OK)
    {
        api->rssi_interf_thr = (int8_t)ICYDM_RSSI_70dB_THRHLD;
    }

    /// Initialize Exchange Memory
    rf_em_init();

#if !defined(RP_HWSIM_BYPASS)
    // Detect the RF version
    //icydm_chip_id = rf_icydm_reg_rd(0x3FF);
#else
    // Stick to v0x10 to prevent ASSERT
    icydm_chip_id = 0x10;
#endif
    icydm_chip_id = 0x10;

    // Force chip_id for YUKON_FPGA
    //icydm_chip_id = 0x01;

    // --------------- RADIOCNTL0 register ---------------
    // Set pointer SW SPI Drive access Pointer
    ip_radiocntl0_pack(/*uint16_t spiptr*/ (EM_RF_SW_SPI_OFFSET >> 2),
                                           /*uint8_t  spicfg*/         0,
                                           /*uint8_t  spifreq*/        1,
                                           /*uint8_t  spigo*/          0);


    // --------------- RADIOCNTL1 register ---------------
    /* RADIOCNTL1 */
    ip_radiocntl1_pack(/*uint8_t  forceagcen*/      0,
            /*uint8_t  forceiq*/         0,
            /*uint8_t  rxdnsl*/          0,
            /*uint8_t  txdnsl*/          0,
            /*uint16_t forceagclength*/  0x0,
            /*uint8_t  syncpulsemode*/   1,
            /*uint8_t  syncpulsesrc*/    1,
            /*uint8_t  dpcorren*/        0,
            /*uint8_t  jefselect*/       0,
            /*uint8_t  xrfsel*/          0x07,
            /*uint8_t  subversion*/      0x0);


#if defined(CFG_BLE_EMB)
    /* BLE RADIOCNTL2 */
    ble_radiocntl2_pack(/*uint8_t  lrsynccompmode*/  0x0,
            /*uint8_t  rxcitermbypass*/  0x0,
            /*uint8_t  lrvtbflush*/      0x10,
            /*uint8_t  phymsk*/          0x3, // mark that Coded phy and 2Mbps are supported
            /*uint8_t  lrsyncerr*/       0,
            /*uint8_t  syncerr*/         0,
            /*uint16_t freqtableptr*/ (EM_FT_OFFSET >> 2));

    /* BLE RADIOCNTL3 */
    ble_radiocntl3_pack(/*uint8_t rxrate3cfg*/       0x3,
            /*uint8_t rxrate2cfg*/       0x2,
            /*uint8_t rxrate1cfg*/       0x1,
            /*uint8_t rxrate0cfg*/       0x0,
            /*uint8_t getrssidelay */ BLE_GETRSSIDELAY_RST,
            /*uint8_t rxsyncrouting*/    0x0,
            /*uint8_t rxvalidbeh*/       0x2,
            /*uint8_t txrate3cfg*/       0x0, // map on 1 Mbps
            /*uint8_t txrate2cfg*/       0x0, // map on 1 Mbps
            /*uint8_t txrate1cfg*/       0x1,
            /*uint8_t txrate0cfg*/       0x0,
            /*uint8_t txvalidbeh*/       0x2);

    switch (icydm_chip_id)
    {
    // YUKON_FPGA
    case (0x01):
        /* BLE RADIOPWRUPDN0 */
        ble_radiopwrupdn0_pack(/*uint8_t syncposition0*/  0,
                /*uint8_t rxpwrup0*/       50,
                /*uint8_t txpwrdn0*/       10,
                /*uint8_t txpwrup0*/       88);

        /* BLE RADIOPWRUPDN1 */
        ble_radiopwrupdn1_pack(/*uint8_t syncposition1*/  0,
                /*uint8_t rxpwrup1*/       50,
                /*uint8_t txpwrdn1*/       10,
                /*uint8_t txpwrup1*/       100);

        /* BLE RADIOPWRUPDN2 */
        ble_radiopwrupdn2_pack(/*uint8_t syncposition2*/  0,
                /*uint8_t rxpwrup2*/       50,
                /*uint8_t txpwrdn2*/       15,
                /*uint8_t txpwrup2*/       100);

        /* BLE RADIOPWRUPDN3 */
        ble_radiopwrupdn3_pack(/*uint8_t txpwrdn3*/       15,
                /*uint8_t txpwrup3*/       100);

        /* BLE RADIOTXRXTIM0 */
        ble_radiotxrxtim0_pack(/*uint8_t rfrxtmda0*/       25,
                /*uint8_t rxpathdly0*/      25,
                /*uint8_t txpathdly0*/      1);

        /* BLE RADIOTXRXTIM1 */
        ble_radiotxrxtim1_pack(/*uint8_t rfrxtmda1*/       25,
                /*uint8_t rxpathdly1*/      25,
                /*uint8_t txpathdly1*/      1);

        /* BLE RADIOTXRXTIM2 */
        ble_radiotxrxtim2_pack(/*uint8_t rxflushpathdly2*/ 17,
                /*uint8_t rfrxtmda2*/       132,
                /*uint8_t rxpathdly2*/      39,
                /*uint8_t txpathdly2*/      1);

        /* BLE RADIOTXRXTIM3 */
        ble_radiotxrxtim3_pack(/*uint8_t rxflushpathdly3*/ 16,
                /*uint8_t rfrxtmda3*/       33,
                /*uint8_t txpathdly3*/      1);
    // TestChip v1
    case (0x10):
        /* BLE RADIOPWRUPDN0 */
        ble_radiopwrupdn0_pack(/*uint8_t syncposition0*/  0,
                /*uint8_t rxpwrup0*/       70,
                /*uint8_t txpwrdn0*/       5,
                /*uint8_t txpwrup0*/       93);

        /* BLE RADIOPWRUPDN1 */
        ble_radiopwrupdn1_pack(/*uint8_t syncposition1*/  0,
                /*uint8_t rxpwrup1*/       70,
                /*uint8_t txpwrdn1*/       5,
                /*uint8_t txpwrup1*/       97);

        /* BLE RADIOPWRUPDN2 */
        ble_radiopwrupdn2_pack(/*uint8_t syncposition2*/  0,
                /*uint8_t rxpwrup2*/       80,
                /*uint8_t txpwrdn2*/       5,
                /*uint8_t txpwrup2*/       93);

        /* BLE RADIOPWRUPDN3 */
        ble_radiopwrupdn3_pack(/*uint8_t txpwrdn3*/       5,
                /*uint8_t txpwrup3*/       93);

        /* BLE RADIOTXRXTIM0 */
        ble_radiotxrxtim0_pack(/*uint8_t rfrxtmda0*/       6,
                /*uint8_t rxpathdly0*/      1,
                /*uint8_t txpathdly0*/      15);

        /* BLE RADIOTXRXTIM1 */
        ble_radiotxrxtim1_pack(/*uint8_t rfrxtmda1*/       3,
                /*uint8_t rxpathdly1*/      1,
                /*uint8_t txpathdly1*/      10);

        /* BLE RADIOTXRXTIM2 */
        ble_radiotxrxtim2_pack(/*uint8_t rxflushpathdly2*/ 10,
                /*uint8_t rfrxtmda2*/       122,
                /*uint8_t rxpathdly2*/      43,
                /*uint8_t txpathdly2*/      15);

        /* BLE RADIOTXRXTIM3 */
        ble_radiotxrxtim3_pack(/*uint8_t rxflushpathdly3*/ 10,
                /*uint8_t rfrxtmda3*/       32,
                /*uint8_t txpathdly3*/      15);
        break;

    default:
        ASSERT_INFO(0, icydm_chip_id, 0);
        break;
    }
#endif // defined CFG_BLE_EMB

#if  defined(CFG_BT_EMB)

    switch (icydm_chip_id)
    {
    /* YUKON_FPGA */
    case (0x01):
        /* EDRCNTL */
        bt_rwbtcntl_nwinsize_setf(0);
        bt_edrcntl_rxgrd_timeout_setf(0x23); //0x16

        /* BT RADIOPWRUPDN */
        bt_radiopwrupdn_pack(/*uint8_t rxpwrupct*/       70,
                /*uint8_t txpwrdnct*/       15,
                /*uint8_t txpwrupct*/       95);

        /* BT RADIOCNTL 2 */
        bt_radiocntl2_pack(/*uint8_t trailer_gating_value*/ 0x4,
                /*uint8_t syncerr*/              0x0,
                /*uint8_t freqtable_ptr*/ ((EM_FT_OFFSET >> 2)));

        /* BT RADIOTXRXTIM */
        bt_radiotxrxtim_pack(/*uint8_t sync_position*/   0,
                /*uint8_t rxpathdly*/       17,
                /*uint8_t txpathdly*/       1);
        break;

    /* Test Chip */
    case (0x10):
        /* EDRCNTL */
        bt_rwbtcntl_nwinsize_setf(0);
        bt_edrcntl_rxgrd_timeout_setf(0x23);

        /* BT RADIOPWRUPDN */
        bt_radiopwrupdn_pack(/*uint8_t rxpwrupct*/       60,
                /*uint8_t txpwrdnct*/        5,
                /*uint8_t txpwrupct*/       96);

        /* BT RADIOCNTL 2 */
        bt_radiocntl2_pack(/*uint8_t trailer_gating_value*/ 0x4,
                /*uint8_t syncerr*/              0x0,
                /*uint8_t freqtable_ptr*/ ((EM_FT_OFFSET >> 2)));

        /* BT RADIOTXRXTIM */
        bt_radiotxrxtim_pack(/*uint8_t sync_position*/   0,
                /*uint8_t rxpathdly*/       5,
                /*uint8_t txpathdly*/       9);
        break;

    default:
        ASSERT_INFO(0, icydm_chip_id, 0);
        break;
    }

    /* BT RADIOCNTL 3*/
    bt_radiocntl3_pack(
        /*uint8_t rxrate2cfg*/      3,
        /*uint8_t rxrate1cfg*/      2,
        /*uint8_t rxrate0cfg*/      0,
        /*uint8_t getrssidelay*/    0x0,
        /*uint8_t rxserparif*/      1,  //1 IN RF TC, 0 in full SoC
        /*uint8_t rxsyncrouting*/   0,
        /*uint8_t rxvalidbeh*/      2,
        /*uint8_t txrate2cfg*/      3,
        /*uint8_t txrate1cfg*/      2,
        /*uint8_t txrate0cfg*/      0,
        /*uint8_t txserparif*/      1,  //1 in RF TC, but 0 in full SoC
        /*uint8_t txvalidbeh*/      2);

#endif //CFG_BT_EMB

#if (BLE_CON_CTE_REQ | BLE_CONLESS_CTE_RX)
    // Init the DF CNTL
    ble_dfcntl0_1us_pack(/*uint8_t rxsampstinst01us*/ 0x08, /*uint8_t rxswstinst01us*/ 0x18, /*uint8_t txswstinst01us*/ 0x19);
    ble_dfcntl0_2us_pack(/*uint8_t rxsampstinst02us*/ 0x08, /*uint8_t rxswstinst02us*/ 0x18, /*uint8_t txswstinst02us*/ 0x19);
    ble_dfcntl1_1us_pack(/*uint8_t rxsampstinst11us*/ 0x08, /*uint8_t rxswstinst11us*/ 0x18, /*uint8_t txswstinst11us*/ 0x19);
    ble_dfcntl1_2us_pack(/*uint8_t rxsampstinst12us*/ 0x08, /*uint8_t rxswstinst12us*/ 0x18, /*uint8_t txswstinst12us*/ 0x19);
    ble_dfantcntl_pack(/*uint8_t rxprimidcntlen*/ 1, /*uint8_t rxprimantid*/ 0, /*uint8_t txprimidcntlen*/ 1, /*uint8_t txprimantid*/ 0);
#endif // (BLE_CON_CTE_REQ | BLE_CONLESS_CTE_RX)

    ble_dfifcntl_pack(/*uint8_t antswitch_beh*/       0,
            /*uint8_t sampreq_beh*/         1, // Toggle mode
            /*uint8_t sampvalid_beh*/       2, // Toggle mode
            /*uint8_t if_width_setf*/       1, // 1 in RF TC (4bit), but 3 in full SoC (16bit)
            /*uint8_t msb_lsb_order*/       1, // LSB first
            /*uint8_t symbol_order*/        0);// I first


    // IcyTRx Static Register Initialization
    switch (icydm_chip_id)
    {
    // YUKON_FPGA
    case (0x01):
        // IcyTRxDM Static Register Initialization for Back to Back FPGA version
        // v1 - FPGA
        // TX/RX config setting
        rf_icydm_reg_burst_wr(0x00, 64, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_INIT[0]);
        rf_icydm_reg_burst_wr(0x40, 64, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_INIT[0x40]);
        rf_icydm_reg_burst_wr(0x80, 64, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_INIT[0x80]);
        rf_icydm_reg_burst_wr(0xC0, 64, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_INIT[0xC0]);
        rf_icydm_reg_burst_wr(0x100, 64, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_INIT[0x100]);
        rf_icydm_reg_burst_wr(0x140, 64, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_INIT[0x140]);
        rf_icydm_reg_burst_wr(0x180, 64, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_INIT[0x180]);
        rf_icydm_reg_burst_wr(0x1C0, 64, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_INIT[0x1C0]);
        rf_icydm_reg_burst_wr(0x200, 66, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_INIT[0x200]);

        // Sequencer
        rf_icydm_reg_burst_wr(0xAD0, 48, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_SEQUENCER[0]);
        rf_icydm_reg_burst_wr(0xB00, 42, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_SEQUENCER[0x30]);

        // TX packet handler
        rf_icydm_reg_burst_wr(0xB50, 79, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_TX_PKT_HANDLER[0]);

        // RX packet handler
        rf_icydm_reg_burst_wr(0xBD0, 48, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_RX_PKT_HANDLER[0]);
        rf_icydm_reg_burst_wr(0xC00, 56, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_RX_PKT_HANDLER[0x30]);

        // AGC packet handler
        rf_icydm_reg_burst_wr(0xC50, 64, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_AGC[0]);
        rf_icydm_reg_burst_wr(0xC90, 64, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_AGC[0x40]);
        rf_icydm_reg_burst_wr(0xCD0, 48, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_AGC[0x80]);
        rf_icydm_reg_burst_wr(0xD00, 80, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_AGC[0xB0]);
        rf_icydm_reg_burst_wr(0xD50, 64, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_AGC[0x100]);
        rf_icydm_reg_burst_wr(0xD90, 64, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_AGC[0x140]);
        rf_icydm_reg_burst_wr(0xDD0, 44, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_AGC[0x180]);

        // GPIOS settings
        rf_icydm_reg_burst_wr(0x00C, 10, (uint8_t *) &RF_ICYDM_REG_TBL_YUKON_FPGA_GPIOS[0]);

        break;

    // TestChip v1
    case (0x10):
        // IcyTRxDM Static Register Initialization for TestChip version
        // TX/RX config setting
        rf_icydm_reg_burst_wr(0x00, 64, (uint8_t *) &RF_ICYDM_REG_TBL_10_INIT[0]);
        rf_icydm_reg_burst_wr(0x40, 64, (uint8_t *) &RF_ICYDM_REG_TBL_10_INIT[0x40]);
        rf_icydm_reg_burst_wr(0x80, 64, (uint8_t *) &RF_ICYDM_REG_TBL_10_INIT[0x80]);
        rf_icydm_reg_burst_wr(0xC0, 64, (uint8_t *) &RF_ICYDM_REG_TBL_10_INIT[0xC0]);
        rf_icydm_reg_burst_wr(0x100, 64, (uint8_t *) &RF_ICYDM_REG_TBL_10_INIT[0x100]);
        rf_icydm_reg_burst_wr(0x140, 64, (uint8_t *) &RF_ICYDM_REG_TBL_10_INIT[0x140]);
        rf_icydm_reg_burst_wr(0x180, 64, (uint8_t *) &RF_ICYDM_REG_TBL_10_INIT[0x180]);
        rf_icydm_reg_burst_wr(0x1C0, 64, (uint8_t *) &RF_ICYDM_REG_TBL_10_INIT[0x1C0]);
        rf_icydm_reg_burst_wr(0x200, 12, (uint8_t *) &RF_ICYDM_REG_TBL_10_INIT[0x200]);

        // Sequencer
        rf_icydm_reg_burst_wr(0xAD0, 48, (uint8_t *) &RF_ICYDM_REG_TBL_10_SEQUENCER[0]);
        rf_icydm_reg_burst_wr(0xB00, 55, (uint8_t *) &RF_ICYDM_REG_TBL_10_SEQUENCER[0x30]);

        // TX packet handler
        rf_icydm_reg_burst_wr(0xB50, 104, (uint8_t *) &RF_ICYDM_REG_TBL_10_TX_PKT_HANDLER[0]);

        // RX packet handler
        rf_icydm_reg_burst_wr(0xBD0, 48, (uint8_t *) &RF_ICYDM_REG_TBL_10_RX_PKT_HANDLER[0]);
        rf_icydm_reg_burst_wr(0xC00, 42, (uint8_t *) &RF_ICYDM_REG_TBL_10_RX_PKT_HANDLER[0x30]);

        // AGC packet handler
        rf_icydm_reg_burst_wr(0xC50, 64, (uint8_t *) &RF_ICYDM_REG_TBL_10_AGC[0]);
        rf_icydm_reg_burst_wr(0xC90, 64, (uint8_t *) &RF_ICYDM_REG_TBL_10_AGC[0x40]);
        rf_icydm_reg_burst_wr(0xCD0, 48, (uint8_t *) &RF_ICYDM_REG_TBL_10_AGC[0x80]);
        rf_icydm_reg_burst_wr(0xD00, 80, (uint8_t *) &RF_ICYDM_REG_TBL_10_AGC[0xB0]);
        rf_icydm_reg_burst_wr(0xD50, 64, (uint8_t *) &RF_ICYDM_REG_TBL_10_AGC[0x100]);
        rf_icydm_reg_burst_wr(0xD90, 64, (uint8_t *) &RF_ICYDM_REG_TBL_10_AGC[0x140]);
        rf_icydm_reg_burst_wr(0xDD0, 44, (uint8_t *) &RF_ICYDM_REG_TBL_10_AGC[0x180]);

        // APLL
        rf_icydm_reg_burst_wr(0xE00, 77, (uint8_t *) &RF_ICYDM_REG_TBL_10_APLL[0]);

        // GPIOS settings
        rf_icydm_reg_burst_wr(0x007, 10, (uint8_t *) &RF_ICYDM_REG_TBL_10_GPIOS[0]);

        // End Init procedure
        rf_icydm_reg_burst_wr(0x000, 4, (uint8_t *) &RF_ICYDM_REG_TBL_10_ENDINIT[0]);

        break;

    default:
        ASSERT_ERR(0);
        break;
    }
    /* *************************************************************************************** */
    /* Initialize HW SPI Chains Pointers  */
    /* *************************************************************************************** */
    uint16_t txonptr      = (EM_RF_HW_SPI_OFFSET);
    uint16_t txoffptr     = (0);
#if defined(CFG_BLE_EMB)
    uint16_t ble_rxonptr  = (EM_RF_HW_SPI_OFFSET + 0x10);
#endif // defined CFG_BLE_EMB
#if  defined(CFG_BT_EMB)
    uint16_t bt_rxonptr   = (EM_RF_HW_SPI_OFFSET + 0x10 + 0x10);
#endif // defined CFG_BT_EMB
    uint16_t rssiptr      = (EM_RF_HW_SPI_OFFSET + 0x10 + 0x10 + 0x20);
    uint16_t rxlengthptr  = (EM_RF_HW_SPI_OFFSET + 0x10 + 0x10 + 0x20 + 0x10);
#if  defined(CFG_BT_EMB)
    uint16_t rxpkttypptr  = (EM_RF_HW_SPI_OFFSET + 0x10 + 0x10 + 0x20 + 0x10 + 0x10);
#endif
    uint16_t rxoffptr     = (EM_RF_HW_SPI_OFFSET + 0x10 + 0x10 + 0x20 + 0x10 + 0x10 + 0x10);

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
    ble_spiptrcntl2_rxlengthptr_setf(rxlengthptr >> 2);
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
    /* Initialize HW SPI Tx On Chained list  -> 1 structure of 12 bytes*/
    /* *************************************************************************************** */
    /*  TxON Sequence -> Write Address @0x800 / 4 bytes
                   -> Next Pointer = 0x0000
                   -> @800 - RATE_TX       - write BT_nBLE mode and CS-TXRATE
                   -> @801 - CHANNEL_TX    - write Bluetooth Channel
                   -> @802 - PA_PWR        - write CS-TXPWR
                   -> @803 - TX_STARTUP    - write 0x10 ( Tx On ) */
    RF_ICTRXDM_EM_WR(txonptr, 0x0000);
    RF_ICTRXDM_EM_WR(txonptr + 0x2, 0x0084);
    RF_ICTRXDM_EM_WR(txonptr + 0x4, 0x0008);
    RF_ICTRXDM_EM_WR(txonptr + 0x6, 0x0000);
    RF_ICTRXDM_EM_WR(txonptr + 0x8, 0x0000);
    RF_ICTRXDM_EM_WR(txonptr + 0xA, 0x1000);
    RF_ICTRXDM_EM_WR(txonptr + 0xC, 0x0000);

    /* *************************************************************************************** */
    /* Initialize HW SPI Tx Off Chained list -> Nothing here at the moment */
    /* *************************************************************************************** */

    /* *************************************************************************************** */
    /* Initialize HW SPI Rx On Chained list -> 1 structure of 16 bytes or 20 bytes*/
    /* *************************************************************************************** */
    /* Initialize HW SPI Rx On Chained list  */
#if defined(CFG_BLE_EMB)
    /*   RxON Sequence -> Write Address @0x810 / 7 bytes
                -> Next Pointer= 0x0000
                -> @810 - RATE_RX       - write BT_nBLE mode and CS-RXRATE
                -> @811 - CHANNEL_RX    - write Bluetooth Index
                -> @812 - RX_STARTUP    - write 0x12 ( Rx On )
                -> @813 - ACCESS_ADDR   - write CS-SYNCWORD[7:0]
                -> @814 - ACCESS_ADDR   - write CS-SYNCWORD[15:8]
                -> @815 - ACCESS_ADDR   - write CS-SYNCWORD[23:16]
                -> @816 - ACCESS_ADDR   - write CS-SYNCWORD[31:24]    */

    RF_ICTRXDM_EM_WR(ble_rxonptr,     0x0000);
    RF_ICTRXDM_EM_WR(ble_rxonptr + 0x2, 0x1087);
    RF_ICTRXDM_EM_WR(ble_rxonptr + 0x4, 0x0008);
    RF_ICTRXDM_EM_WR(ble_rxonptr + 0x6, 0x0000);
    RF_ICTRXDM_EM_WR(ble_rxonptr + 0x8, 0x0000);
    RF_ICTRXDM_EM_WR(ble_rxonptr + 0xA, 0x0012);
    RF_ICTRXDM_EM_WR(ble_rxonptr + 0xC, 0x0000);
    RF_ICTRXDM_EM_WR(ble_rxonptr + 0xE, 0x0000);
#endif // defined CFG_BLE_EMB

#if  defined(CFG_BT_EMB)
    /*   RxON Sequence -> Write Address @0x810 / 11 bytes
                -> Next Pointer= 0x0000
                -> @810 - RATE_RX       - write BT_nBLE mode and CS-RXRATE
                -> @811 - CHANNEL_RX    - write Bluetooth Index
                -> @812 - RX_STARTUP    - write 0x12 ( Rx On )
                -> @813 - SYNC_WORD_LOW - write CS-SYNCWORD[7:0]
                -> @814 - SYNC_WORD_LOW - write CS-SYNCWORD[15:8]
                -> @815 - SYNC_WORD_LOW - write CS-SYNCWORD[23:16]
                -> @816 - SYNC_WORD_LOW - write CS-SYNCWORD[31:24]
                -> @817 - SYNC_WORD_HIGH - write CS-SYNCWORD[39:32]
                -> @818 - SYNC_WORD_HIGH - write CS-SYNCWORD[47:40]
                -> @819 - SYNC_WORD_HIGH - write CS-SYNCWORD[55:48]
                -> @81A - SYNC_WORD_HIGH - write CS-SYNCWORD[63:56]    */

    RF_ICTRXDM_EM_WR(bt_rxonptr,     0x0000);
    RF_ICTRXDM_EM_WR(bt_rxonptr + 0x2, 0x108B);
    RF_ICTRXDM_EM_WR(bt_rxonptr + 0x4, 0x0008);
    RF_ICTRXDM_EM_WR(bt_rxonptr + 0x6, 0x0000);
    RF_ICTRXDM_EM_WR(bt_rxonptr + 0x8, 0x0000);
    RF_ICTRXDM_EM_WR(bt_rxonptr + 0xA, 0x0012);
    RF_ICTRXDM_EM_WR(bt_rxonptr + 0xC, 0x0000);
    RF_ICTRXDM_EM_WR(bt_rxonptr + 0xE, 0x0000);
    RF_ICTRXDM_EM_WR(bt_rxonptr + 0x10, 0x0000);
    RF_ICTRXDM_EM_WR(bt_rxonptr + 0x12, 0x0000);
#endif // defined CFG_BT_EMB

    /* *************************************************************************************** */
    /* Initialize HW SPI Rx Off Chained list -> 1 structure of 9 byte */
    /* *************************************************************************************** */
    /*    RxOFF Access 1  -> Write Address @0x820 / 1 byte
                  -> Next Pointer = 0x0000 -> end of Sequence
                  -> Write Address @0x820 - RX_POWEROFF
                  -> Write data = 0x13 (RX Off)*/

    RF_ICTRXDM_EM_WR(rxoffptr,     0x0000);
    RF_ICTRXDM_EM_WR(rxoffptr + 0x2, 0x2081);
    RF_ICTRXDM_EM_WR(rxoffptr + 0x4, 0x0008);
    RF_ICTRXDM_EM_WR(rxoffptr + 0x6, 0x0000);
    if (icydm_chip_id == 0x10)
        RF_ICTRXDM_EM_WR(rxoffptr + 0x8, 0x0013);
    else
        /* Due to a bug in sequencer microcode, value 0x43 has to be written instead of 0x13 until microcode is fixed */
        RF_ICTRXDM_EM_WR(rxoffptr + 0x8, 0x0043);


    /* *************************************************************************************** */
    /* Initialize HW SPI RSSI Chained list  -> 1 structure of 9 bytes*/
    /* *************************************************************************************** */
    /*    RSSI Access   -> Read Address @0x824 / 1 byte
                 -> Next Pointer = 0x0000 -> end of the sequence
                 -> Read Address @0x824 - RSSI_READYOUT
                 -> Write data = 0x000000 -> Read data to replace this / provided to Packet Controller */
    RF_ICTRXDM_EM_WR(rssiptr,     0x0000);
    RF_ICTRXDM_EM_WR(rssiptr + 0x2, 0x2401);
    RF_ICTRXDM_EM_WR(rssiptr + 0x4, 0x0008);
    RF_ICTRXDM_EM_WR(rssiptr + 0x6, 0x0000);
    RF_ICTRXDM_EM_WR(rssiptr + 0x8, 0x0000);

#if defined(CFG_BLE_EMB)
    /* *************************************************************************************** */
    /* Initialize HW SPI Rx Packet Length Chained list  -> 1 structure of 9 bytes*/
    /* *************************************************************************************** */
    /* PACKET_LENGTH Access -> Write Address @0x828 / 1 byte
                 -> Next Pointer = 0x0000 -> end of the sequence
                 -> Write Address @0x828 - PACKT_LENGTH
                 -> Write data = Long Range packet length */
    RF_ICTRXDM_EM_WR(rxlengthptr,     0x0000);
    RF_ICTRXDM_EM_WR(rxlengthptr + 0x2, 0x2881);
    RF_ICTRXDM_EM_WR(rxlengthptr + 0x4, 0x0008);
    RF_ICTRXDM_EM_WR(rxlengthptr + 0x6, 0x0000);
    RF_ICTRXDM_EM_WR(rxlengthptr + 0x8, 0x0000);
#endif // defined CFG_BLE_EMB

#if  defined(CFG_BT_EMB)
    /* *************************************************************************************** */
    /* Initialize HW SPI Rx Packet Rate Chained list  -> 1 structure of 9 bytes*/
    /* *************************************************************************************** */
    /* PACKET_RATE Access -> Write Address @0x810 / 1 byte
                 -> Next Pointer = 0x0000 -> end of the sequence
                 -> Write Address @0x810 - PACKET_RATE
                 -> Write data = EDR Packet Rate */
    RF_ICTRXDM_EM_WR(rxpkttypptr,     0x0000);
    RF_ICTRXDM_EM_WR(rxpkttypptr + 0x2, 0x1081);
    RF_ICTRXDM_EM_WR(rxpkttypptr + 0x4, 0x0008);
    RF_ICTRXDM_EM_WR(rxpkttypptr + 0x6, 0x0000);
    RF_ICTRXDM_EM_WR(rxpkttypptr + 0x8, 0x0000);
#endif // defined CFG_BT_EMB
}
///@} RF_ICYTRXDM
