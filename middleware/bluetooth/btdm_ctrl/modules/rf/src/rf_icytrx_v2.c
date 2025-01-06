/**
****************************************************************************************
*
* @file rf_icyv2as.c
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
* This is the driver block for Atlas radio
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
#include "reg_iqgen.h"     // Add DF Generator register h file here

// Atlas register definitions and access functions
__STATIC uint32_t rf_icyv2_reg_rd(uint32_t addr);
__STATIC void rf_icyv2_reg_wr(uint32_t addr, uint32_t value);

#define REG_ICYV2_RD                rf_icyv2_reg_rd
#define REG_ICYV2_WR                rf_icyv2_reg_wr

/**
 ****************************************************************************************
 * DEFINES
 ****************************************************************************************
 **/

#define RF_EM_SPI_ADRESS              (EM_BASE_ADDR + EM_RF_SW_SPI_OFFSET)

#define ICYV2_SPIRD                   0x00
#define ICYV2_SPIWR                   0x80

#define ICYV2_PWR_TBL_SIZE            0x10
#define ICYV2_PA_TBL_MSK              0x0F
#define ICYV2_PA_PWR_MSK              0x1F

#define ICYV2_MAX_BURST_SIZE          0x80
#define ICYV2_INIT_TBL_18_SIZE        0xD0
#define ICYV2_INIT_TBL_20_SIZE        0xFC
#define ICYV2_INIT_TBL_30_SIZE        0xF7

#define ICYV2_RSSI_20dB_THRHLD        -20
#define ICYV2_RSSI_40dB_THRHLD        -40
#define ICYV2_RSSI_45dB_THRHLD        -45
#define ICYV2_RSSI_48dB_THRHLD        -48
#define ICYV2_RSSI_55dB_THRHLD        -55
#define ICYV2_RSSI_60dB_THRHLD        -60
#define ICYV2_RSSI_70dB_THRHLD        -70

// TX max power
//#define ICYV2_POWER_MIN                 (-3) // from post-CS3 release
#define ICYV2_POWER_MIN                 0
#define ICYV2_POWER_MAX                12

// Workaround RSSI value issues identified in EBQ testing
#define RF_EBQ_RSSI_WORKAROUND          1
/**
****************************************************************************************
* MACROS
*****************************************************************************************
*/

/// IcyTRx EM Write Macro for HW driven SPI chained structures
#define RF_ICTRX_EM_BLE_WR(addr, val) \
    EM_BLE_WR((((uint32_t) (addr)) + REG_EM_BLE_CS_BASE_ADDR), (val))


// Max burst register
__STATIC uint8_t rf_icyv2_reg_buf[ICYV2_MAX_BURST_SIZE + 8]; // max burst size + buffer controls

/**
 ****************************************************************************************
 * GLOBAL VARIABLE DEFINITIONS
 *****************************************************************************************
 **/

/* --------------------------- IcyTRx v2 0x18 -------------------------------------------------*/
/* Icytrx v2 static settings - v0x18- 1Mbps table */
__STATIC  const uint8_t RF_ICYV2_REG_TBL_18_1MBPS[ICYV2_INIT_TBL_18_SIZE] =
{
    0x22, 0x08, 0x00, 0x00,   0x00, 0x22, 0x0b, 0x40,    0x30, 0x20, 0x00, 0x10,   0x03, 0x00, 0xff, 0xff, // 0x0*
    0x0f, 0x00, 0x00, 0x80,   0x23, 0x82, 0x04, 0x00,    0x1b, 0xc7, 0x15, 0x82,   0x01, 0x01, 0x72, 0x1c, // 0x1*
    0xc7, 0x00, 0x82, 0xff,   0x01, 0x55, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00,   0x29, 0x41, 0x76, 0x71, // 0x2*
    0x03, 0x08, 0xdf, 0x00,   0x2d, 0x03, 0x80, 0x00,    0x55, 0x55, 0x55, 0x00,   0x21, 0x00, 0x00, 0x00, // 0x3*
    0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x01, 0x02,    0x07, 0x10, 0x20, 0x37,   0x50, 0x66, 0x76, 0x7e, // 0x4*
    0x2b, 0x10, 0xc0, 0x3f,   0x21, 0x74, 0xca, 0xfe,    0x40, 0x00, 0x50, 0x1b,   0x50, 0x02, 0x30, 0x44, // 0x5*
    0x0b, 0x11, 0x55, 0x15,   0x00, 0x1b, 0x09, 0x3a,    0x0e, 0x17, 0x0c, 0x00,   0xfe, 0xa0, 0x50, 0x6f, // 0x6*
    0x00, 0x00, 0x04, 0x28,   0x40, 0x05, 0xca, 0x50,    0x95, 0xea, 0x94, 0xb7,   0xfc, 0xe5, 0xbf, 0xff, // 0x7*
    0x0e, 0x71, 0x00, 0x21,   0x94, 0xa2, 0x6d, 0x1b,    0x01, 0x35, 0xc1, 0x22,   0x51, 0x01, 0x00, 0x00, // 0x8*
    0x00, 0x73, 0x66, 0x72,   0x99, 0x73, 0xcc, 0x77,    0x37, 0x9f, 0x95, 0x06,   0x89, 0x67, 0x00, 0x00, // 0x9*
    0x08, 0x17, 0x00, 0x6f,   0xc2, 0xc2, 0x16, 0x41,    0x18, 0x00, 0x66, 0x63,   0x0b, 0x70, 0x00, 0x00, // 0xa*
    0x00, 0x35, 0x84, 0xc3,   0x05, 0x84, 0xf0, 0x03,    0xd0, 0xf1, 0xc3, 0x00,   0x11, 0x03, 0x00, 0xc0, // 0xb*
    0x00, 0x00, 0x8e, 0x33,   0x00, 0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x3c,   0x80, 0x06, 0x01, 0x38
};


/* Icytrx v2 static settings - v0x18 - 2Mbps table */
__STATIC  const uint8_t RF_ICYV2_REG_TBL_18_2MBPS[ICYV2_INIT_TBL_18_SIZE] =
{
    0x22, 0x08, 0x00, 0x00,   0x00, 0x22, 0x0b, 0x40,    0x30, 0x20, 0x00, 0x10,   0x03, 0x00, 0xff, 0xff, // 0x0*
    0x0f, 0x00, 0x00, 0x80,   0x23, 0x82, 0x05, 0x00,    0x1b, 0xc7, 0x15, 0x82,   0x00, 0x00, 0x72, 0x1c, // 0x1*
    0xc7, 0x00, 0x82, 0xff,   0x01, 0x55, 0x01, 0x00,    0x00, 0x00, 0x00, 0x00,   0x29, 0x41, 0x76, 0x71, // 0x2*
    0x03, 0x08, 0xdf, 0x00,   0x2d, 0x03, 0x80, 0x00,    0x55, 0x55, 0x55, 0x00,   0x21, 0x00, 0x00, 0x00, // 0x3*
    0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x01, 0x02,    0x07, 0x10, 0x20, 0x37,   0x50, 0x66, 0x76, 0x7e, // 0x4*
    0x3b, 0x00, 0xc0, 0x3f,   0x21, 0x74, 0xca, 0xfe,    0x80, 0x00, 0x70, 0x1b,   0x50, 0x02, 0x30, 0x44, // 0x5*
    0x0b, 0x11, 0xaa, 0x2a,   0x00, 0x15, 0x07, 0x3a,    0x0e, 0x17, 0x0c, 0x00,   0xfe, 0xa0, 0x50, 0x6f, // 0x6*
    0x00, 0x00, 0x04, 0x28,   0x40, 0x05, 0xca, 0x50,    0x95, 0xea, 0x94, 0xb7,   0xfc, 0xe5, 0xbf, 0xff, // 0x7*
    0x0e, 0x71, 0x00, 0x21,   0x94, 0xa2, 0x6d, 0x1b,    0x01, 0x35, 0xc1, 0x22,   0x51, 0x01, 0x00, 0x00, // 0x8*
    0x00, 0x73, 0x66, 0x72,   0x99, 0x73, 0xcc, 0x77,    0x37, 0x9f, 0x95, 0x06,   0x89, 0x67, 0x00, 0x00, // 0x9*
    0x08, 0x17, 0x00, 0x6f,   0xc2, 0xc2, 0x16, 0x41,    0x18, 0x00, 0x66, 0x63,   0x0b, 0x70, 0x00, 0x00, // 0xa*
    0x00, 0x35, 0x84, 0xc3,   0x05, 0x84, 0xf0, 0x03,    0xd0, 0xf1, 0xc3, 0x00,   0x11, 0x03, 0x00, 0xc0, // 0xb*
    0x00, 0x00, 0x8e, 0x33,   0x00, 0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x3c,   0x80, 0x06, 0x01, 0x38
};


/* --------------------------- IcyTRx v2 0x20 -------------------------------------------------*/
/* Icytrx v2 static settings - v0x20- 1Mbps table */
__STATIC  const uint8_t RF_ICYV2_REG_TBL_20_1MBPS[ICYV2_INIT_TBL_20_SIZE] =
{
    0x22, 0x08, 0x00, 0x00,   0x00, 0x23, 0x0b, 0x14,    0x10, 0x20, 0x00, 0x00,   0x10, 0x03, 0x00, 0xff, // 0x0*
    0xff, 0x0f, 0x00, 0x00,   0x80, 0x23, 0x82, 0x04,    0x1b, 0xc7, 0x15, 0x82,   0x00, 0x01, 0x01, 0x00, // 0x1*
    0x72, 0x1c, 0xc7, 0x82,   0xff, 0x01, 0x55, 0x00,    0x00, 0x00, 0x00, 0x00,   0x29, 0x41, 0x76, 0x71, // 0x2*
    0x00, 0x23, 0x08, 0xdf,   0x2d, 0x03, 0x80, 0x00,    0x55, 0x55, 0x55, 0x00,   0x21, 0x00, 0x00, 0x00, // 0x3*
    0x00, 0x29, 0x10, 0xc0,   0x00, 0x00, 0x00, 0x00,    0x00, 0x00, 0x01, 0x02,   0x07, 0x10, 0x20, 0x37, // 0x4*
    0x50, 0x66, 0x76, 0x7e,   0x21, 0x74, 0xca, 0xfe,    0x80, 0x50, 0x40, 0x00,   0x40, 0x02, 0x11, 0x28, // 0x5*
    0x3e, 0x0b, 0x55, 0x15,   0x11, 0x00, 0x19, 0x0c,    0x0c, 0x73, 0x0b, 0x16,   0x00, 0xfe, 0xb0, 0x30, // 0x6*
    0x59, 0x71, 0x20, 0x05,   0x00, 0x00, 0x04, 0xa0,    0x00, 0x49, 0x28, 0x42,   0x16, 0xf2, 0x90, 0x97, // 0x7*
    0xfc, 0xe4, 0x47, 0x7f,   0x0e, 0x21, 0x35, 0xc1,    0x4c, 0x94, 0xad, 0x26,   0x01, 0x22, 0x51, 0x01, // 0x8*
    0x00, 0x00, 0x00, 0x73,   0x66, 0x38, 0x77, 0x74,    0x8a, 0x77, 0x66, 0x9f,   0x95, 0x06, 0x8a, 0x67, // 0x9*
    0x17, 0x00, 0x00, 0xa5,   0x00, 0x00, 0x08, 0x46,    0x46, 0x1f, 0xd8, 0x02,   0x64, 0x00, 0x11, 0x24, // 0xa*
    0x1b, 0x70, 0x00, 0x00,   0x80, 0x15, 0x84, 0xc3,    0x05, 0x80, 0xf0, 0x03,   0xe0, 0xf1, 0xc3, 0x00, // 0xb*
    0x00, 0x00, 0xc0, 0x00,   0x00, 0x89, 0x33, 0x00,    0x00, 0x00, 0x2f, 0xae,   0x80, 0x04, 0x00, 0x3c, // 0xc*
    0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, // 0xd*
    0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00,   0x00, 0x01, 0x38, 0x00, // 0xe*
    0x00, 0x01, 0xa5, 0x14,   0x00, 0x00, 0x00, 0x25,    0x03, 0x00, 0x0e, 0x30
};

/* Icytrx v2 static settings - v0x20 - 2Mbps table */
__STATIC  const uint8_t RF_ICYV2_REG_TBL_20_2MBPS[ICYV2_INIT_TBL_20_SIZE] =
{
    0x22, 0x08, 0x00, 0x00,   0x00, 0x23, 0x0b, 0x14,    0x10, 0x20, 0x00, 0x00,   0x10, 0x03, 0x00, 0xff, // 0x0*
    0xff, 0x0f, 0x00, 0x00,   0x80, 0x23, 0x82, 0x05,    0x1b, 0xc7, 0x15, 0x82,   0x00, 0x00, 0x00, 0x00, // 0x1*
    0x72, 0x1c, 0xc7, 0x82,   0xff, 0x01, 0x55, 0x01,    0x00, 0x00, 0x00, 0x00,   0x29, 0x41, 0x76, 0x71, // 0x2*
    0x00, 0x23, 0x08, 0xdf,   0x2d, 0x03, 0x80, 0x00,    0x55, 0x55, 0x55, 0x00,   0x21, 0x00, 0x00, 0x00, // 0x3*
    0x00, 0x3a, 0x00, 0xc0,   0x00, 0x00, 0x00, 0x00,    0x00, 0x00, 0x01, 0x02,   0x07, 0x10, 0x20, 0x37, // 0x4*
    0x50, 0x66, 0x76, 0x7e,   0x21, 0x74, 0xca, 0xfe,    0x80, 0x70, 0x80, 0x00,   0x40, 0x02, 0x11, 0x28, // 0x5*
    0x3e, 0x0b, 0xaa, 0x2a,   0x11, 0x00, 0x15, 0x0c,    0x0c, 0x78, 0x10, 0x18,   0x00, 0xfe, 0xb0, 0x30, // 0x6*
    0x59, 0x71, 0x20, 0x05,   0x00, 0x00, 0x04, 0xa0,    0x00, 0x49, 0x28, 0x42,   0x16, 0xf2, 0x90, 0x97, // 0x7*
    0xfc, 0xe4, 0x47, 0x7f,   0x0e, 0x21, 0x35, 0xc1,    0x4c, 0x94, 0xad, 0x26,   0x01, 0x22, 0x51, 0x01, // 0x8*
    0x00, 0x00, 0x00, 0x73,   0x66, 0x38, 0x77, 0x74,    0x8a, 0x77, 0x66, 0x9f,   0x95, 0x06, 0x8a, 0x67, // 0x9*
    0x17, 0x00, 0x00, 0xa5,   0x00, 0x00, 0x08, 0x46,    0x46, 0x1f, 0xd8, 0x02,   0x64, 0x00, 0x11, 0x24, // 0xa*
    0x1b, 0x70, 0x00, 0x00,   0x80, 0x15, 0x84, 0xc3,    0x05, 0x80, 0xf0, 0x03,   0xe0, 0xf1, 0xc3, 0x00, // 0xb*
    0x00, 0x00, 0xc0, 0x00,   0x00, 0x89, 0x33, 0x00,    0x00, 0x00, 0x37, 0xb7,   0x80, 0x04, 0x00, 0x3c, // 0xc*
    0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00, // 0xd*
    0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00,   0x00, 0x01, 0x38, 0x00, // 0xe*
    0x00, 0x01, 0xa5, 0x14,   0x00, 0x00, 0x00, 0x25,    0x03, 0x00, 0x0e, 0x30                            // 0xf*
};


/* --------------------------- IcyTRx v2 0x30 -------------------------------------------------*/
// TODO [JPL] RF integration under work
/* Icytrx v2 static settings - v0x30- 1Mbps table */
__STATIC  const uint8_t RF_ICYV2_REG_TBL_30_1MBPS[ICYV2_INIT_TBL_30_SIZE] =
{
    0x22, 0x08, 0x00, 0x00,   0x00, 0x22, 0x0b, 0x14,    0x10, 0x20, 0x00, 0x00,   0x10, 0x03, 0x00, 0x80, // 0x0*
    0x0f, 0x0f, 0x0f, 0x0f,   0x0f, 0x00, 0x00, 0x00,    0x1b, 0xc7, 0x15, 0x82,   0x00, 0x00, 0x23, 0x82, // 0x1*
    0x04, 0x00, 0x01, 0x01,   0x72, 0x1c, 0xc7, 0x00,    0x82, 0xff, 0x01, 0x55,   0x29, 0x41, 0x76, 0x71, // 0x2*
    0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x23, 0x08,    0x2d, 0x03, 0x80, 0x00,   0x55, 0x55, 0x55, 0x00, // 0x3*
    0xdf, 0x00, 0x21, 0x00,   0x00, 0x00, 0x29, 0x80,    0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x01, 0x02, // 0x4*
    0x07, 0x10, 0x20, 0x37,   0x50, 0x66, 0x76, 0x7e,    0x01, 0x30, 0x80, 0x00,   0x21, 0x74, 0xca, 0xfe, // 0x5*
    0x50, 0x11, 0x40, 0x02,   0x28, 0x3e, 0x0b, 0x11,    0x55, 0x15, 0x00, 0x0c,   0x19, 0x00, 0x0c, 0x73, // 0x6*
    0x0b, 0x16, 0x00, 0xfe,   0xb0, 0x30, 0x59, 0x71,    0x00, 0x00, 0x04, 0xa0,   0x00, 0x49, 0x28, 0x42, // 0x7*
    0x16, 0xf2, 0x90, 0x97,   0xfc, 0xe4, 0x47, 0x7f,    0x0e, 0x21, 0x20, 0x05,   0x4c, 0x94, 0xad, 0x26, // 0x8*
    0x01, 0x35, 0xc1, 0x22,   0x51, 0x01, 0x00, 0x00,    0x00, 0x73, 0x66, 0x38,   0x77, 0x74, 0x8a, 0x77, // 0x9*
    0x66, 0x9f, 0x00, 0x95,   0x06, 0x8a, 0x67, 0x17,    0x00, 0x00, 0x08, 0x00,   0x46, 0x1f, 0xd8, 0x02, // 0xa*
    0x64, 0x00, 0x11, 0x24,   0x1b, 0x25, 0x46, 0x05,    0x00, 0x00, 0x70, 0x60,   0x80, 0x15, 0x84, 0xc3, // 0xb*
    0x80, 0xf0, 0xe0, 0xf1,   0xc3, 0x00, 0x00, 0x00,    0xc0, 0x00, 0x00, 0x89,   0x33, 0x00, 0x00, 0x00, // 0xc*
    0x2f, 0xae, 0x80, 0x04,   0x00, 0xfc, 0x01, 0x38,    0x1c, 0x00, 0x00, 0x00,   0x00, 0x25, 0x03, 0x00, // 0xd*
    0x0e, 0x30, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00,   0x01, 0x00, 0x00, 0x00, // 0xe*
    0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00
};

/* Icytrx v2 static settings - v0x30- 2Mbps table */
__STATIC  const uint8_t RF_ICYV2_REG_TBL_30_2MBPS[ICYV2_INIT_TBL_30_SIZE] =
{
    0x22, 0x08, 0x00, 0x00,   0x00, 0x22, 0x0b, 0x14,    0x10, 0x20, 0x00, 0x00,   0x10, 0x03, 0x00, 0x80, // 0x0*
    0x0f, 0x0f, 0x0f, 0x0f,   0x0f, 0x00, 0x00, 0x00,    0x1b, 0xc7, 0x15, 0x82,   0x00, 0x00, 0x23, 0x82, // 0x1*
    0x05, 0x00, 0x00, 0x00,   0x72, 0x1c, 0xc7, 0x00,    0x82, 0xff, 0x01, 0x55,   0x29, 0x41, 0x76, 0x71, // 0x2*
    0x01, 0x00, 0x00, 0x00,   0x00, 0x00, 0x23, 0x08,    0x2d, 0x03, 0x80, 0x00,   0x55, 0x55, 0x55, 0x00, // 0x3*
    0xdf, 0x00, 0x21, 0x00,   0x00, 0x00, 0x3a, 0x80,    0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x01, 0x02, // 0x4*
    0x07, 0x10, 0x20, 0x37,   0x50, 0x66, 0x76, 0x7e,    0x00, 0x30, 0x80, 0x00,   0x21, 0x74, 0xca, 0xfe, // 0x5*
    0x70, 0x11, 0x40, 0x02,   0x28, 0x3e, 0x0b, 0x11,    0xaa, 0x2a, 0x00, 0x0c,   0x15, 0x00, 0x0b, 0x78, // 0x6*
    0x10, 0x18, 0x00, 0xfe,   0xb0, 0x30, 0x59, 0x71,    0x00, 0x00, 0x04, 0xa0,   0x00, 0x49, 0x28, 0x42, // 0x7*
    0x16, 0xf2, 0x90, 0x97,   0xfc, 0xe4, 0x47, 0x7f,    0x0e, 0x21, 0x20, 0x05,   0x4c, 0x94, 0xad, 0x26, // 0x8*
    0x01, 0x35, 0xc1, 0x22,   0x51, 0x01, 0x00, 0x00,    0x00, 0x73, 0x66, 0x38,   0x77, 0x74, 0x8a, 0x77, // 0x9*
    0x66, 0x9f, 0x00, 0x95,   0x06, 0x8a, 0x67, 0x17,    0x00, 0x00, 0x08, 0x00,   0x46, 0x1f, 0xd8, 0x02, // 0xa*
    0x64, 0x00, 0x11, 0x24,   0x1b, 0x25, 0x46, 0x05,    0x00, 0x00, 0x70, 0x60,   0x80, 0x15, 0x84, 0xc3, // 0xb*
    0x80, 0xf0, 0xe0, 0xf1,   0xc3, 0x00, 0x00, 0x00,    0xc0, 0x00, 0x00, 0x89,   0x33, 0x00, 0x00, 0x00, // 0xc*
    0x37, 0xb7, 0x80, 0x04,   0x00, 0xfc, 0x01, 0x38,    0x1c, 0x00, 0x00, 0x00,   0x00, 0x25, 0x03, 0x00, // 0xd*
    0x0e, 0x30, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,    0x00, 0x00, 0x00, 0x00,   0x01, 0x00, 0x00, 0x00, // 0xe*
    0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00
};


/* IcyTRx dynamic register */
enum
{
    ICYV2_RATE_TX        = 0x1E0,
    ICYV2_CHANNEL_RM_TX  = 0x1E1,
    ICYV2_PA_PWR_RM      = 0x1E2,
    ICYV2_FSM_MODE_RM_TX = 0x1E3,
    ICYV2_RATE_RX        = 0x1E8,
    ICYV2_CHANNEL_RM_RX  = 0x1E9,
    ICYV2_FSM_MODE_RM_RX = 0x1EA,
    ICYV2_ACCESS_ADDRESS = 0x1EB,
    ICYV2_RSSI_AVG_RM    = 0x1F0,
    ICYV2_BLR_PACKET_LEN = 0x1F4,
};

// Power table - ref. Table 10, icyTRXDigital_architecture_3v3.docx
__STATIC const int8_t RF_ICYV2_TX_PW_CONV_TBL[ICYV2_PWR_TBL_SIZE] =
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
__STATIC void rf_icyv2_spi_tf(void)
{
    //launch SPI transfer
    ip_radiocntl0_spigo_setf(1);

    //wait for transfer to be completed
    while (!ip_radiocntl0_spicomp_getf());
}

/**
 ****************************************************************************************
 * @brief Atlas specific read access
 *
 * @param[in] addr    register address
 *
 * @return uint32_t value
 *****************************************************************************************
 */
__STATIC uint32_t rf_icyv2_reg_rd(uint32_t addr)
{
    // Next Pointr to 0x0
    rf_icyv2_reg_buf[0] = (uint8_t)(0);  // Next Pointer set to 0x0000 to stop the SPI Chained access
    rf_icyv2_reg_buf[1] = (uint8_t)(0);

    //copy control and number of u32 to send
    rf_icyv2_reg_buf[2] = (uint8_t)(ICYV2_SPIRD + 1);

    //copy address
    rf_icyv2_reg_buf[3] = (uint8_t)(addr & 0x00FF);
    rf_icyv2_reg_buf[4] = (uint8_t)((addr & 0xFF00) >> 8);

    // Padding
    rf_icyv2_reg_buf[5] = (uint8_t)(0);
    rf_icyv2_reg_buf[6] = (uint8_t)(0);
    rf_icyv2_reg_buf[7] = (uint8_t)(0);

    memcpy((void *)RF_EM_SPI_ADRESS, rf_icyv2_reg_buf, 8);

    //do the transfer
    rf_icyv2_spi_tf();

    return (uint32_t)(*((uint8_t *)(RF_EM_SPI_ADRESS + 8)));
}

/**
 ****************************************************************************************
 * @brief Atlas specific write access
 *
 * @param[in] addr    register address
 * @param[in] value   value to write
 *
 * @return uint32_t value
 ****************************************************************************************
 */
__STATIC void rf_icyv2_reg_wr(uint32_t addr, uint32_t value)
{
    rf_icyv2_reg_buf[0] = (uint8_t)(0);  // Next Pointer set to 0x0000 to stop the SPI Chained access
    rf_icyv2_reg_buf[1] = (uint8_t)(0);

    //inversion for EM reading by U8 on ATL SPI side
    //copy control and number of u32 to send
    rf_icyv2_reg_buf[2] = (uint8_t)(ICYV2_SPIWR + 1);

    //copy address
    rf_icyv2_reg_buf[3] = (uint8_t)(addr & 0x00FF);
    rf_icyv2_reg_buf[4] = (uint8_t)((addr & 0xFF00) >> 8);

    // Padding
    rf_icyv2_reg_buf[5] = (uint8_t)(0);
    rf_icyv2_reg_buf[6] = (uint8_t)(0);
    rf_icyv2_reg_buf[7] = (uint8_t)(0);

    // Byte to be written
    rf_icyv2_reg_buf[8] = (uint8_t)value;

    memcpy((void *)RF_EM_SPI_ADRESS, rf_icyv2_reg_buf, 9);

    //do the transfer
    rf_icyv2_spi_tf();
}

/**
 ****************************************************************************************
 * @brief Atlas specific read access
 *
 * @param[in] addr    register address
 * @param[in] size    transfer size
 * @param[in] data    pointer to the data array
 *
 * @return uint32_t value
 ****************************************************************************************
 **/
__STATIC void rf_icyv2_reg_burst_wr(uint16_t addr, uint8_t size, uint8_t *data)
{
    rf_icyv2_reg_buf[0] = (uint8_t)(0);  // Next Pointer set to 0x0000 to stop the SPI Chained access
    rf_icyv2_reg_buf[1] = (uint8_t)(0);

    //copy control and number of u8 to send
    rf_icyv2_reg_buf[2] = (uint8_t)(ICYV2_SPIWR + size);

    //copy address
    rf_icyv2_reg_buf[3] = (uint8_t)(addr & 0x00FF);
    rf_icyv2_reg_buf[4] = (uint8_t)((addr & 0xFF00) >> 8);

    // Padding
    rf_icyv2_reg_buf[5] = (uint8_t)(0);
    rf_icyv2_reg_buf[6] = (uint8_t)(0);
    rf_icyv2_reg_buf[7] = (uint8_t)(0);

    for (int i = 0; i < size + 2; i++)
    {
        rf_icyv2_reg_buf[i + 8] = *(data + i);
    }

    memcpy((void *)RF_EM_SPI_ADRESS, rf_icyv2_reg_buf, 8 + size);

    //do the transfer
    rf_icyv2_spi_tf();
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
    return (RF_ICYV2_TX_PW_CONV_TBL[txpwr_idx & ICYV2_PA_TBL_MSK]);
}

/**
 *****************************************************************************************
 * @brief Sleep function for Atlas RF.
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
    RssidBm += 10;
#endif // RF_EBQ_RSSI_WORKAROUND
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

    for (i = ICYV2_POWER_MIN; i < ICYV2_POWER_MAX; i++)
    {
        // Loop until we find a power higher than or equal to the requested one
        if (RF_ICYV2_TX_PW_CONV_TBL[i & ICYV2_PA_TBL_MSK] >= txpwr_dbm)
            break;
    }

    if ((RF_ICYV2_TX_PW_CONV_TBL[i & ICYV2_PA_TBL_MSK] > txpwr_dbm) && (i > ICYV2_POWER_MIN))
    {
        if ((option == TXPWR_CS_LOWER)
                || ((option == TXPWR_CS_NEAREST) && (co_abs(txpwr_dbm - RF_ICYV2_TX_PW_CONV_TBL[(i - 1) & ICYV2_PA_TBL_MSK]) < co_abs(txpwr_dbm - RF_ICYV2_TX_PW_CONV_TBL[i & ICYV2_PA_TBL_MSK]))))
        {
            i--;
        }
    }

    return (i & ICYV2_PA_PWR_MSK);
}

/**
 ****************************************************************************************
 * RADIO FUNCTION INTERFACE
 ****************************************************************************************
 **/
void rf_init(struct rwip_rf_api *api)
{
    uint32_t icy_v2_version = 0;  // Default version is IcyTRX V2
    uint32_t icy_v2_ext_dfif = 0; // Default is external DF IF not supported

    uint8_t length = PARAM_LEN_RSSI_THR;

    // Initialize the RF driver API structure
    api->reg_rd = rf_icyv2_reg_rd;
    api->reg_wr = rf_icyv2_reg_wr;
    api->txpwr_dbm_get = rf_txpwr_dbm_get;
    api->txpwr_min = ICYV2_POWER_MIN;
    api->txpwr_max = ICYV2_POWER_MAX;
    api->sleep = rf_sleep;
    api->reset = rf_reset;
    api->force_agc_enable = rf_force_agc_enable;
    api->rssi_convert = rf_rssi_convert;
    api->txpwr_cs_get = rf_txpwr_cs_get;

    // Initialize the RSSI thresholds (high, low, interference)
    // The upper threshold level is 20 dB above the lower threshold level to an accuracy of +-6 dB
    // These are 'real' signed values in dBm
    if (rwip_param.get(PARAM_ID_RSSI_HIGH_THR, &length, (uint8_t *)&api->rssi_high_thr) != PARAM_OK)
    {
        api->rssi_high_thr = (int8_t)ICYV2_RSSI_40dB_THRHLD;
    }
    if (rwip_param.get(PARAM_ID_RSSI_LOW_THR, &length, (uint8_t *)&api->rssi_low_thr) != PARAM_OK)
    {
        api->rssi_low_thr = (int8_t)ICYV2_RSSI_60dB_THRHLD;
    }
    if (rwip_param.get(PARAM_ID_RSSI_INTERF_THR, &length, (uint8_t *)&api->rssi_interf_thr) != PARAM_OK)
    {
        api->rssi_interf_thr = (int8_t)ICYV2_RSSI_70dB_THRHLD;
    }

    // --------------- RADIOCNTL0 register ---------------
    // Set pointer SW SPI Drive access Pointer
    ip_radiocntl0_spiptr_setf(EM_RF_SW_SPI_OFFSET >> 2);

    // --------------- RADIOCNTL1 register ---------------
    // RF Selection - IcyTRx V2 selected
    ip_radiocntl1_xrfsel_setf(0x04);

    // Enable Jitter elimination FIFO
    ip_radiocntl1_jef_select_setf(0x1);

    // select sync Pulse Mode
    ip_radiocntl1_sync_pulse_mode_setf(0x1);

    // Select sync pulse source
    ip_radiocntl1_sync_pulse_src_setf(0x1);

#if !defined(RP_HWSIM_BYPASS)
    // Detect the RF version
    icy_v2_version = rf_icyv2_reg_rd(0x1FF);

    // Detect External DF IF support
    icy_v2_ext_dfif = rf_icyv2_reg_rd(0x1F8) & 0x1; // Masking All bits except LSB

    // Select proper sub-version of IcyTRx Radio Controller / Need BLE Core xls update 1st
    switch (icy_v2_version)
    {
    case (0x18): // V2
    case (0x20): // CS555
    {
        ip_radiocntl1_subversion_setf(0x0);
        // Set Platform RF selection register
        plf_rf_interface_sel(RF_INTF_V3X_V4X);
    }
    break;
    case (0x30): // CS555 with AoA/AoD support proto
    {
        ip_radiocntl1_subversion_setf(0x0);
        // Set Platform RF selection register
        plf_rf_interface_sel(RF_INTF_V3X_V4X);
        if (icy_v2_ext_dfif == 0x1)
        {
            iqgen_df_source_setf(0x1);
        }
    }
    break;
    default:     // Defaut
    {
        ip_radiocntl1_subversion_setf(0x0);
        // Reset Platform RF selection register
        plf_rf_interface_sel(RF_INTF_V1X_V2X);
    }
    break;
    }
#else
    // Stick to v0x18 to prevent ASSERT
    icy_v2_version = 0x18;
    ip_radiocntl1_subversion_setf(0x0);
    plf_rf_interface_sel(RF_INTF_V1X_V2X);
#endif

    // --------------- RADIOCNTL2 register ---------------
    ble_radiocntl2_phymsk_setf(0x3);


    // Long Range Viterbi Flush
    ble_radiocntl2_lrvtbflush_setf(0x10);

    // --------------- RADIOCNTL3 register ---------------
    // Select Rx valid behavior
    ble_radiocntl3_rxvalid_beh_setf(0x2);

    // Select Tx valid behavior
    ble_radiocntl3_txvalid_beh_setf(0x2);

    // Set TX rate config
    ble_radiocntl3_txrate0cfg_setf(0x0);
    ble_radiocntl3_txrate1cfg_setf(0x1);
    ble_radiocntl3_txrate2cfg_setf(0x0);
    ble_radiocntl3_txrate3cfg_setf(0x0);

    // Set RX rate config
    ble_radiocntl3_rxrate0cfg_setf(0x0);
    ble_radiocntl3_rxrate1cfg_setf(0x1);
    ble_radiocntl3_rxrate2cfg_setf(0x2);
    ble_radiocntl3_rxrate3cfg_setf(0x2);

    // RSSI delay
    ble_radiocntl3_getrssidelay_setf(BLE_GETRSSIDELAY_RST);

    // ------------ Tx/Rx Power up/down and Timings -------------------
    switch (icy_v2_version)
    {
    case (0x18):
    case (0x20): // CS555
        // 1Mbps settings
        ble_radiopwrupdn0_txpwrup0_setf(36);        // Tx Power Up 0
        ble_radiopwrupdn0_txpwrdn0_setf(9);         // Tx Power Down 0
        ble_radiotxrxtim0_txpathdly0_setf(8);       // Tx Path Delay 0
        ble_radiopwrupdn0_rxpwrup0_setf(69);        // Rx Power Up 0
        ble_radiotxrxtim0_rxpathdly0_setf(8);       // Rx Path Delay 0
        ble_radiotxrxtim0_rfrxtmda0_setf(7);        // Rx Test Path Delay 0
        ble_radiopwrupdn0_sync_position0_setf(0);
        // 2Mbps settings
        ble_radiopwrupdn1_txpwrup1_setf(36);        // Tx Power Up 1
        ble_radiopwrupdn1_txpwrdn1_setf(6);         // Tx Power Down 1
        ble_radiotxrxtim1_txpathdly1_setf(6);       // Tx Path Delay 1
        ble_radiopwrupdn1_rxpwrup1_setf(65);        // Rx Power Up 1
        ble_radiotxrxtim1_rxpathdly1_setf(3);       // Rx Path Delay 1
        ble_radiotxrxtim1_rfrxtmda1_setf(3);        // Rx Test Path Delay 1
        ble_radiopwrupdn1_sync_position1_setf(0);
        // LR 125Kbps settings
        ble_radiopwrupdn2_txpwrup2_setf(34);        // Tx Power Up 2
        ble_radiotxrxtim2_txpathdly2_setf(11);      // Tx Path Delay 2
        ble_radiopwrupdn2_txpwrdn2_setf(14);        // Tx Power Down 2
        ble_radiopwrupdn2_rxpwrup2_setf(69);        // Rx Power Up 2
        ble_radiotxrxtim2_rxpathdly2_setf(39);      // Rx Path Delay 2
        ble_radiotxrxtim2_rfrxtmda2_setf(132);      // Rx Test Path Delay 2
        ble_radiotxrxtim2_rxflushpathdly2_setf(17); // Rx Flush Delay 2
        ble_radiopwrupdn2_sync_position2_setf(0);
        // LR 500Kbps settings
        ble_radiopwrupdn3_txpwrup3_setf(34);        // Tx Power Up 3
        ble_radiotxrxtim3_txpathdly3_setf(11);      // Tx Path Delay 3
        ble_radiopwrupdn3_txpwrdn3_setf(14);        // Tx Power Down 3
        ble_radiotxrxtim3_rfrxtmda3_setf(33);       // Rx Test Path Delay 3
        ble_radiotxrxtim3_rxflushpathdly3_setf(14); // Rx Flush Delay

#if (BLE_CON_CTE_REQ | BLE_CONLESS_CTE_RX)
        // Init the DF CNTL
        ble_dfcntl0_1us_pack(/*uint8_t rxsampstinst01us*/ 0x08, /*uint8_t rxswstinst01us*/ 0x18, /*uint8_t txswstinst01us*/ 0x19);
        ble_dfcntl0_2us_pack(/*uint8_t rxsampstinst02us*/ 0x08, /*uint8_t rxswstinst02us*/ 0x18, /*uint8_t txswstinst02us*/ 0x19);
        ble_dfcntl1_1us_pack(/*uint8_t rxsampstinst11us*/ 0x08, /*uint8_t rxswstinst11us*/ 0x18, /*uint8_t txswstinst11us*/ 0x19);
        ble_dfcntl1_2us_pack(/*uint8_t rxsampstinst12us*/ 0x08, /*uint8_t rxswstinst12us*/ 0x18, /*uint8_t txswstinst12us*/ 0x19);
        ble_dfantcntl_pack(/*uint8_t rxprimidcntlen*/ 1, /*uint8_t rxprimantid*/ 0, /*uint8_t txprimidcntlen*/ 1, /*uint8_t txprimantid*/ 0);
#endif // (BLE_CON_CTE_REQ | BLE_CONLESS_CTE_RX)

        break;

    case (0x30): // CS555 with aoA/AoD Support - VALUES FOR TEST SETUP !!
        // 1Mbps settings
        // TODO [JPL] Settings to be confirmed
        ble_radiopwrupdn0_txpwrup0_setf(36);        // Tx Power Up 0
        ble_radiopwrupdn0_txpwrdn0_setf(9 + 6);       // Tx Power Down 0
        ble_radiotxrxtim0_txpathdly0_setf(8 + 6);     // Tx Path Delay 0
        ble_radiopwrupdn0_rxpwrup0_setf(69 + 4);      // Rx Power Up 0
        ble_radiotxrxtim0_rxpathdly0_setf(8);       // Rx Path Delay 0
        ble_radiotxrxtim0_rfrxtmda0_setf(7);        // Rx Test Path Delay 0
        ble_radiopwrupdn0_sync_position0_setf(0);
        // 2Mbps settings
        // TODO [JPL] Settings to be confirmed
        ble_radiopwrupdn1_txpwrup1_setf(36 - 4);      // Tx Power Up 1
        ble_radiopwrupdn1_txpwrdn1_setf(6 + 6);       // Tx Power Down 1
        ble_radiotxrxtim1_txpathdly1_setf(6 + 6);     // Tx Path Delay 1
        ble_radiopwrupdn1_rxpwrup1_setf(65 + 4);      // Rx Power Up 1
        ble_radiotxrxtim1_rxpathdly1_setf(4);       // Rx Path Delay 1
        ble_radiotxrxtim1_rfrxtmda1_setf(3);        // Rx Test Path Delay 1
        ble_radiopwrupdn1_sync_position1_setf(0);
        // LR 125Kbps settings
        // TODO [JPL] Settings to be confirmed
        ble_radiopwrupdn2_txpwrup2_setf(34);        // Tx Power Up 2
        ble_radiotxrxtim2_txpathdly2_setf(11 + 14);    // Tx Path Delay 2
        ble_radiopwrupdn2_txpwrdn2_setf(14);        // Tx Power Down 2
        ble_radiopwrupdn2_rxpwrup2_setf(69);        // Rx Power Up 2
        ble_radiotxrxtim2_rxpathdly2_setf(39);      // Rx Path Delay 2
        ble_radiotxrxtim2_rfrxtmda2_setf(132);      // Rx Test Path Delay 2
        ble_radiotxrxtim2_rxflushpathdly2_setf(17); // Rx Flush Delay 2
        ble_radiopwrupdn2_sync_position2_setf(0);
        // LR 500Kbps settings
        // TODO [JPL] Settings to be confirmed
        ble_radiopwrupdn3_txpwrup3_setf(34);        // Tx Power Up 3
        ble_radiotxrxtim3_txpathdly3_setf(11 + 14);    // Tx Path Delay 3
        ble_radiopwrupdn3_txpwrdn3_setf(14);        // Tx Power Down 3
        ble_radiotxrxtim3_rfrxtmda3_setf(33);       // Rx Test Path Delay 3
        ble_radiotxrxtim3_rxflushpathdly3_setf(14 + 2); // Rx Flush Delay

#if (BLE_CON_CTE_REQ | BLE_CONLESS_CTE_RX)
        // Init the DF CNTL
        // TODO[JPL] Those timing will need confirmation from CSEM
        ble_dfcntl0_1us_pack(/*uint8_t rxsampstinst01us*/ 0x08, /*uint8_t rxswstinst01us*/ 0x18, /*uint8_t txswstinst01us*/ 0x19);
        ble_dfcntl0_2us_pack(/*uint8_t rxsampstinst02us*/ 0x08, /*uint8_t rxswstinst02us*/ 0x18, /*uint8_t txswstinst02us*/ 0x19);
        ble_dfcntl1_1us_pack(/*uint8_t rxsampstinst11us*/ 0x08, /*uint8_t rxswstinst11us*/ 0x18, /*uint8_t txswstinst11us*/ 0x19);
        ble_dfcntl1_2us_pack(/*uint8_t rxsampstinst12us*/ 0x08, /*uint8_t rxswstinst12us*/ 0x18, /*uint8_t txswstinst12us*/ 0x19);
        ble_dfantcntl_pack(/*uint8_t rxprimidcntlen*/ 1, /*uint8_t rxprimantid*/ 0, /*uint8_t txprimidcntlen*/ 1, /*uint8_t txprimantid*/ 0);
#endif // (BLE_CON_CTE_REQ | BLE_CONLESS_CTE_RX)

        // Program DF Interface
        if (icy_v2_ext_dfif == 0x1)
        {
            ble_dfifcntl_sampvalid_beh_setf(2);   // Toggle mode
            ble_dfifcntl_if_width_setf(1);        // 4 bits
            ble_dfifcntl_msb_lsb_order_setf(1);   // LSB first
            ble_dfifcntl_symbol_order_setf(0);    // I first
        }
        break;

    default:
        ASSERT_INFO(0, icy_v2_version, 0);
        break;
    }

    // IcyTRx Static Register Initialization
    switch (icy_v2_version)
    {
    // v2 - test chip
    case (0x18):
        // 1Mbps config setting
        rf_icyv2_reg_wr(0x016, 0x00);
        rf_icyv2_reg_burst_wr(0x00, 0x70, (uint8_t *) &RF_ICYV2_REG_TBL_18_1MBPS[0]);
        rf_icyv2_reg_burst_wr(0x70, 0x60, (uint8_t *) &RF_ICYV2_REG_TBL_18_1MBPS[0x70]);

        // 2Mbps config setting
        rf_icyv2_reg_wr(0x016, 0x01);
        rf_icyv2_reg_burst_wr(0x00, 0x70, (uint8_t *) &RF_ICYV2_REG_TBL_18_2MBPS[0]);
        rf_icyv2_reg_burst_wr(0x70, 0x60, (uint8_t *) &RF_ICYV2_REG_TBL_18_2MBPS[0x70]);

        break;
    // CS555
    case (0x20):
        // 1Mbps config setting
        rf_icyv2_reg_wr(0x017, 0x00);
        rf_icyv2_reg_burst_wr(0x00, 0x50, (uint8_t *) &RF_ICYV2_REG_TBL_20_1MBPS[0]);
        rf_icyv2_reg_burst_wr(0x50, 0x50, (uint8_t *) &RF_ICYV2_REG_TBL_20_1MBPS[0x50]);
        rf_icyv2_reg_burst_wr(0xA0, 0x5C, (uint8_t *) &RF_ICYV2_REG_TBL_20_1MBPS[0xA0]);

        // 2Mbps config setting
        rf_icyv2_reg_wr(0x017, 0x01);
        rf_icyv2_reg_burst_wr(0x00, 0x50, (uint8_t *) &RF_ICYV2_REG_TBL_20_2MBPS[0]);
        rf_icyv2_reg_burst_wr(0x50, 0x50, (uint8_t *) &RF_ICYV2_REG_TBL_20_2MBPS[0x50]);
        rf_icyv2_reg_burst_wr(0xA0, 0x5C, (uint8_t *) &RF_ICYV2_REG_TBL_20_2MBPS[0xA0]);

        break;
    // CS555 with AoA/AoD support
    case (0x30):
        // 1Mbps config setting
        rf_icyv2_reg_wr(0x017, 0x00);
        rf_icyv2_reg_burst_wr(0x00, 0x50, (uint8_t *) &RF_ICYV2_REG_TBL_30_1MBPS[0]);
        rf_icyv2_reg_burst_wr(0x50, 0x50, (uint8_t *) &RF_ICYV2_REG_TBL_30_1MBPS[0x50]);
        rf_icyv2_reg_burst_wr(0xA0, 0x57, (uint8_t *) &RF_ICYV2_REG_TBL_30_1MBPS[0xA0]);

        // 2Mbps config setting
        rf_icyv2_reg_wr(0x017, 0x01);
        rf_icyv2_reg_burst_wr(0x00, 0x50, (uint8_t *) &RF_ICYV2_REG_TBL_30_2MBPS[0]);
        rf_icyv2_reg_burst_wr(0x50, 0x50, (uint8_t *) &RF_ICYV2_REG_TBL_30_2MBPS[0x50]);
        rf_icyv2_reg_burst_wr(0xA0, 0x57, (uint8_t *) &RF_ICYV2_REG_TBL_30_2MBPS[0xA0]);

        // @DEBUG - IQ sample generated by icyTrx, zero to avoid fake
        // rf_icyv2_reg_wr(0xF5, 0x1);

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
    uint16_t rxonptr      = (EM_RF_HW_SPI_OFFSET + 0x10);
    uint16_t rxoffptr     = (EM_RF_HW_SPI_OFFSET + 0x10 + 0x10);
    uint16_t rssiptr      = (EM_RF_HW_SPI_OFFSET + 0x10 + 0x10 + 0x24);
    uint16_t rxlengthptr  = (EM_RF_HW_SPI_OFFSET + 0x10 + 0x10 + 0x24 + 0x10);

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

    /* Long Range packet length Sequence start pointer */
    ble_spiptrcntl2_rxlengthptr_setf(rxlengthptr >> 2);

    /* *************************************************************************************** */
    /* Initialize HW SPI Tx On Chained list  -> 1 structure of 4 bytes*/
    /* *************************************************************************************** */
    /*  TxON Sequence -> Write Address @0x1E0 / 4 bytes
                   -> Next Pointer = 0x0000
                   -> @1E0 - RATE_TX       - write CS-TXRATE
                   -> @1E1 - CHANNEL_RM_TX - write Data Channel Index
                   -> @1E2 - PA_PWR_TX     - write CS-TXPWR
                   -> @1E3 - FSM_MODE_TX   - write 0x07 ( Tx On ) */
    RF_ICTRX_EM_BLE_WR(txonptr, 0x0000);
    RF_ICTRX_EM_BLE_WR(txonptr + 0x2, 0xE084);
    RF_ICTRX_EM_BLE_WR(txonptr + 0x4, 0x0001);
    RF_ICTRX_EM_BLE_WR(txonptr + 0x6, 0x0000);
    RF_ICTRX_EM_BLE_WR(txonptr + 0x8, 0x0004);
    RF_ICTRX_EM_BLE_WR(txonptr + 0xA, 0x070C);
    RF_ICTRX_EM_BLE_WR(txonptr + 0xC, 0x0000);

    /* *************************************************************************************** */
    /* Initialize HW SPI Tx Off Chained list -> Nothing here at the moment */
    /* *************************************************************************************** */

    /* *************************************************************************************** */
    /* Initialize HW SPI Rx On Chained list -> 1 structure of 7 bytes */
    /* *************************************************************************************** */
    /* Initialize HW SPI Rx On Chained list  */
    /*   RxON Sequence -> Write Address @0x1E8 / 7 bytes
                -> Next Pointer= 0x0000
                -> @1E8 - RATE_RX       - write CS-RXRATE
                -> @1E9 - CHANNEL_RM_RX - write Data Channel Index
                -> @1EA - FSM_MODE_RX   - write 0x03 ( Rx On )
                -> @1EB - ACCESS_ADDR   - write CS-SYNCWORD[7:0]
                -> @1EC - ACCESS_ADDR   - write CS-SYNCWORD[15:8]
                -> @1ED - ACCESS_ADDR   - write CS-SYNCWORD[23:16]
                -> @1EE - ACCESS_ADDR   - write CS-SYNCWORD[31:24]    */

    RF_ICTRX_EM_BLE_WR(rxonptr,     0x0000);
    RF_ICTRX_EM_BLE_WR(rxonptr + 0x2, 0xE887);
    RF_ICTRX_EM_BLE_WR(rxonptr + 0x4, 0x0001);
    RF_ICTRX_EM_BLE_WR(rxonptr + 0x6, 0x0000);
    RF_ICTRX_EM_BLE_WR(rxonptr + 0x8, 0x0004);
    RF_ICTRX_EM_BLE_WR(rxonptr + 0xA, 0x0003);
    RF_ICTRX_EM_BLE_WR(rxonptr + 0xC, 0x0000);
    RF_ICTRX_EM_BLE_WR(rxonptr + 0xE, 0x0000);

    /* *************************************************************************************** */
    /* Initialize HW SPI Rx Off Chained list -> 1 structure of 1 byte */
    /* *************************************************************************************** */
    /*    RxOFF Access 1  -> Rx Disable
                  -> Next Pointer = 0x0000 -> end of Sequence
                  -> Write Address @0x1EA - FSM_MODE
                  -> Write data = 0x08 (RX Off)*/

    RF_ICTRX_EM_BLE_WR(rxoffptr, (uint16_t)((rxoffptr + 0xC) >> 2));
    RF_ICTRX_EM_BLE_WR(rxoffptr + 0x2, 0xEA81);
    RF_ICTRX_EM_BLE_WR(rxoffptr + 0x4, 0x0001);
    RF_ICTRX_EM_BLE_WR(rxoffptr + 0x6, 0x0000);
    RF_ICTRX_EM_BLE_WR(rxoffptr + 0x8, 0x0008);


    /*   Radio Soft Reset by doing two consecutive writes @0
         It's a patch to fix issue with Long Range Flush
               -> Write 0x00   Address@0
               -> Write 0x22   Address@0
    */
    RF_ICTRX_EM_BLE_WR(rxoffptr + 0xC, (uint16_t)((rxoffptr + 0x18) >> 2));
    RF_ICTRX_EM_BLE_WR(rxoffptr + 0xE, 0x0081);
    RF_ICTRX_EM_BLE_WR(rxoffptr + 0x10, 0x0000);
    RF_ICTRX_EM_BLE_WR(rxoffptr + 0x12, 0x0000);
    RF_ICTRX_EM_BLE_WR(rxoffptr + 0x14, 0x0000);

    RF_ICTRX_EM_BLE_WR(rxoffptr + 0x18, 0x0000);
    RF_ICTRX_EM_BLE_WR(rxoffptr + 0x1A, 0x0081);
    RF_ICTRX_EM_BLE_WR(rxoffptr + 0x1C, 0x0000);
    RF_ICTRX_EM_BLE_WR(rxoffptr + 0x1E, 0x0000);
    RF_ICTRX_EM_BLE_WR(rxoffptr + 0x20, 0x0022);




    /* *************************************************************************************** */
    /* Initialize HW SPI RSSI Chained list  -> 1 structure of 6 bytes*/
    /* *************************************************************************************** */
    /*    RSSI Access   -> RSSI read
                 -> Next Pointer = 0x0000 -> end of the sequence
                 -> Write Address @0x1F0 / 1 byte / Valid for all RF version at the moment
                 -> Write data = 0x000000 -> Read data to replace this / provided to Packet Controller */
    RF_ICTRX_EM_BLE_WR(rssiptr,     0x0000);
    RF_ICTRX_EM_BLE_WR(rssiptr + 0x2, 0xF001);
    RF_ICTRX_EM_BLE_WR(rssiptr + 0x4, 0x0001);
    RF_ICTRX_EM_BLE_WR(rssiptr + 0x6, 0x0000);
    RF_ICTRX_EM_BLE_WR(rssiptr + 0x8, 0x0000);

    /* *************************************************************************************** */
    /* Initialize HW SPI Rx Length Chained list  -> 1 structure of 6 bytes*/
    /* *************************************************************************************** */
    /*    RSSI Access   -> RSSI read
                 -> Next Pointer = 0x0000 -> end of the sequence
                 -> Write Address @0x1F4/ 1 byte / Valid for all RF version at the moment
                 -> Write data = 0x000000 -> Read data to replace this / provided to Packet Controller */
    RF_ICTRX_EM_BLE_WR(rxlengthptr,     0x0000);
    RF_ICTRX_EM_BLE_WR(rxlengthptr + 0x2, 0xF481);
    RF_ICTRX_EM_BLE_WR(rxlengthptr + 0x4, 0x0001);
    RF_ICTRX_EM_BLE_WR(rxlengthptr + 0x6, 0x0000);
    RF_ICTRX_EM_BLE_WR(rxlengthptr + 0x8, 0x0000);

};
///@} RF_ICYV2AS
