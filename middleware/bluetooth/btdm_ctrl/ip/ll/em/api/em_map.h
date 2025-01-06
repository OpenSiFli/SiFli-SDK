/**
 ****************************************************************************************
 *
 * @file em_map.h
 *
 * @brief Mapping of the exchange memory
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

#ifndef EM_MAP_H_
#define EM_MAP_H_

/**
 ****************************************************************************************
 * @addtogroup EM EM
 * @ingroup IP
 * @brief Mapping of the different common area in the exchange memory
 *
 * @{
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "co_math.h"                     // For use of CO_ALIGN4_HI

#include "_reg_em_et.h"

/// Retrieve Exchange memory address to set into HW interface
#define REG_EM_ADDR_GET(elem, idx) (REG_EM_##elem##_ADDR_GET(idx) >> 2)
/// Retrieve Exchange index from address load from  HW interface
#define REG_EM_IDX_GET(elem, addr) ((((addr)<<2) - (EM_##elem##_OFFSET))/(REG_EM_##elem##_SIZE))

/// Exchange memory base address
#define EM_BASE_ADDR         REG_EM_ET_BASE_ADDR

/// Null pointer in EM offset space
#define EM_PTR_NULL              (0x0000)


/*
 ****************************************************************************************
 **********************              Common EM part                **********************
 ****************************************************************************************
 */
/// Start of the common EM part
#define EM_COMMON_OFFSET     (0)

/*
 * EXCHANGE TABLE
 ****************************************************************************************
 */

/// Exchange table area definition
#define EM_EXCH_TABLE_LEN    16
#define EM_ET_OFFSET         (EM_COMMON_OFFSET)
#define EM_ET_END            (EM_ET_OFFSET + EM_EXCH_TABLE_LEN * REG_EM_ET_SIZE)

/**
 * MODEn[3:0]
 *
 *  0x0: No mode selected, nothing to be performed
 *  0x1: BR/EDR Mode
 *  0x2: BLE Mode
 *  0x3-0xF: Reserved for future use           -
 */
#define EM_ET_MODE_NONE    0x00
#define EM_ET_MODE_BREDR   0x01
#define EM_ET_MODE_BLE     0x02

/// exchange table entry status
enum em_et_status
{
    /// 000: Exchange Table entry associated event is ready for processing.
    EM_ET_STATUS_READY                 = 0x00,//!< EM_ET_STATUS_READY
    /// 001: Exchange Table entry is waiting for start (already read by Event Scheduler)
    EM_ET_STATUS_UNDER_PROCESS         = 0x01,//!< EM_ET_STATUS_UNDER_PROCESS
    /// 010: Exchange Table entry associated event is started
    EM_ET_STATUS_STARTED               = 0x02,//!< EM_ET_STATUS_STARTED
    /// 011: Exchange Table entry associated event is terminated (normal termination)
    EM_ET_STATUS_TERM_NORMAL           = 0x03,//!< EM_ET_STATUS_TERM_NORMAL
    /// 100: Exchange Table entry associated event is terminated (abort termination under prio bandwidth)
    EM_ET_STATUS_TERM_ABORT_IN_PRIO_BW = 0x04,//!< EM_ET_STATUS_TERM_ABORT_IN_PRIO_BW
    /// 101: Exchange Table entry associated event is terminated (abort termination after prio bandwidth)
    EM_ET_STATUS_TERM_ABORT            = 0x05,//!< EM_ET_STATUS_TERM_ABORT
    /// 110: Exchange Table entry associated event is skipped
    EM_ET_STATUS_SKIPPED               = 0x06,//!< EM_ET_STATUS_SKIPPED
    /// 111: Exchange Table entry associated event is skipped due to waiting time to start > 0x07FF_FFFF CLKN
    EM_ET_STATUS_SKIPPED_WAITING_TIME  = 0x07,//!< EM_ET_STATUS_SKIPPED_WAITING_TIME
    /// 111: Reserved for future use
};

/**
 * ISOBUFSELn
 * Used to select Isochronous channel Buffer Pointer
 * 0x0: Uses ISO<0/1/2><TX/RX>PTR0
 * 0x1: Uses ISO<0/1/2><TX/RX>PTR1
 */
#define EM_ET_ISO_PTR0          0x00
#define EM_ET_ISO_PTR1          0x01

/**
 * ISOn
 * Indicates a Isochronous connection event is programmed
 * 0x0: Not an Isochronous Channel event
 * 0x1: Isochronous Channel event
 */
#define EM_ET_ISO_NOTPROGRAMMED     0x00
#define EM_ET_ISO_PROGRAMMED        0x01

/**
 * ISOCHANn[1:0]
 * Meaningful if ISOn = 1
 * 00: Select Isochronous channel 0.
 * 01: Select Isochronous channel 1.
 * 10: Select Isochronous channel 2.
 * 11: Trash received isochronous packet / Sent null length packet if ET-ISO = 1 / No Tx if ET-ISO = 0
 */
#define EM_ET_ISO_CHANNEL_0     0x00
#define EM_ET_ISO_CHANNEL_1     0x01
#define EM_ET_ISO_CHANNEL_2     0x02
#define EM_ET_ISO_NOCHANNEL     0x03


/**
 * RSVDn
 * Indicates a reserved event
 * 0x0: Not a reserved event (ISO connection re-Tx)
 * 0x1: Reserved event (ISO connection primary event)
 */
#define EM_ET_ISO_NOT_RSVD      0x00
#define EM_ET_ISO_RSVD          0x01
/*
 * FREQUENCY TABLE
 ****************************************************************************************
 */

/// Frequency table area definition
#define EM_FT_OFFSET         (EM_ET_END)


#ifndef RF_FREQ_TABLE_ELT_SIZE
    #define RF_FREQ_TABLE_ELT_SIZE 1
#endif

#ifndef RF_VCO_TABLE_ELT_SIZE
    #define RF_VCO_TABLE_ELT_SIZE 0
#endif

#ifndef RF_SW_SPI_SIZE_MAX
    #define RF_SW_SPI_SIZE_MAX 4
#endif

#ifndef RF_HW_SPI_SIZE_MAX
    #define RF_HW_SPI_SIZE_MAX 4
#endif

/// number of frequencies / Depends on RF target
#if BT_EMB_PRESENT
    /// Requires 80 x words for Frequency table + potential VCO sub-band table
    #define EM_RF_FREQ_TABLE_LEN  (80 * RF_FREQ_TABLE_ELT_SIZE)
    #define EM_RF_VCO_TABLE_LEN (80 * RF_VCO_TABLE_ELT_SIZE)
#elif BLE_EMB_PRESENT
    /// Requires 40 x words for Frequency table + potential VCO sub-band table
    #define EM_RF_FREQ_TABLE_LEN  (40 * RF_FREQ_TABLE_ELT_SIZE)
    #define EM_RF_VCO_TABLE_LEN (40 * RF_VCO_TABLE_ELT_SIZE)
#endif // BT_EMB_PRESENT/BLE_EMB_PRESENT

#define EM_FT_END            (EM_FT_OFFSET + (EM_RF_VCO_TABLE_LEN + EM_RF_FREQ_TABLE_LEN) * sizeof(uint8_t))

/*
 * RF SW SPI
 ****************************************************************************************
 */
/// RF SW-Driven SPI transfers area definition
#define EM_RF_SW_SPI_OFFSET     (EM_FT_END)
#define EM_RF_SW_SPI_SIZE_MAX   (RF_SW_SPI_SIZE_MAX)
#define EM_RF_SW_SPI_END        (EM_RF_SW_SPI_OFFSET + EM_RF_SW_SPI_SIZE_MAX)

/*
 * RF HW SPI
 ****************************************************************************************
 */
/// RF HW-Driven SPI transfers area definition
#define EM_RF_HW_SPI_OFFSET     (EM_RF_SW_SPI_END)
#define EM_RF_HW_SPI_SIZE_MAX   (RF_HW_SPI_SIZE_MAX)
#define EM_RF_HW_SPI_END        (EM_RF_HW_SPI_OFFSET + EM_RF_HW_SPI_SIZE_MAX)

/// End of the common EM part
#define EM_COMMON_END        (EM_RF_HW_SPI_END)

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    /*
    * ENCRYPTION
    ****************************************************************************************
    */

    /// Encryption area definition
    #define EM_ENC_OFFSET          CO_ALIGN4_HI(EM_COMMON_END)
    #define EM_ENC_IN_OFFSET       (EM_ENC_OFFSET)
    #define EM_ENC_IN_SIZE         (16)
    #define EM_ENC_OUT_OFFSET      (EM_ENC_IN_OFFSET + EM_ENC_IN_SIZE)
    #define EM_ENC_OUT_SIZE        (16)
    #define EM_ENC_END             (EM_ENC_OFFSET + EM_ENC_IN_SIZE + EM_ENC_OUT_SIZE)

#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

/*
 ****************************************************************************************
 **********************                BLE EM part                 **********************
 ****************************************************************************************
 */

#define EM_BLE_OFFSET          CO_ALIGN4_HI(EM_ENC_END)
#if BLE_EMB_PRESENT
    #include "em_map_ble.h"
#else // BLE_EMB_PRESENT
    #define EM_BLE_END             (EM_BLE_OFFSET)
#endif // BLE_EMB_PRESENT

/*
 ****************************************************************************************
 **********************                BT EM part                  **********************
 ****************************************************************************************
 */

#define EM_BT_OFFSET           CO_ALIGN4_HI(EM_BLE_END)
#if BT_EMB_PRESENT
    #include "em_map_bt.h"
#else // BT_EMB_PRESENT
    #define EM_BT_END              (EM_BT_OFFSET)
#endif //BT_EMB_PRESENT


/*
 * RF SPI part
 ****************************************************************************************
 */


/// @} IPDEXMEM

#endif // EM_MAP_H_
