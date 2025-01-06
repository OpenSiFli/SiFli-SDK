/**
****************************************************************************************
*
* @file sch_prog.h
*
* @brief Scheduling Programmer API
*
* Copyright (C) RivieraWaves 2009-2017
*
****************************************************************************************
*/

#ifndef SCH_PROG_H_
#define SCH_PROG_H_

/**
 ****************************************************************************************
 * @defgroup SCH_PROG Scheduling Programmer
 * @ingroup SCH
 * @brief Responsible for programming BT/BLE frames.
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#include <stdint.h>         // integer
#include <stdbool.h>        // boolean

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Mode
enum SCH_FRAME_TYPE
{
    SCH_PROG_BLE,
    SCH_PROG_BT,
};

/// Frame type
enum SCH_FRAME_IRQ
{
    /// Normal End of Event
    SCH_FRAME_IRQ_EOF        = 0x00,
    /// End of event due to an abort under the primary priority duration
    SCH_FRAME_IRQ_EOF_ABORT_UNDER_PRIO  = 0x01,
    /// End of event due to an abort after the primary priority duration
    SCH_FRAME_IRQ_EOF_ABORT_AFTER_PRIO  = 0x07,
    /// SKIP Event IRQ
    SCH_FRAME_IRQ_SKIP       = 0x04,
    /// RX ACL IRQ
    SCH_FRAME_IRQ_RX         = 0x02,
    /// TX ACL IRQ
    SCH_FRAME_IRQ_TX         = 0x03,

#if (BLE_ISO_PRESENT)
    /// RX ISO IRQ
    SCH_FRAME_IRQ_RX_ISO     = 0x05,
    /// TX ISO IRQ
    SCH_FRAME_IRQ_TX_ISO     = 0x06,
#endif // (BLE_ISO_PRESENT)

    SCH_FRAME_IRQ_ERROR      = 0xFF,
};

#if BT_EMB_PRESENT
/// Frame type
enum SCH_BT_FRAME_TYPE
{
    SCH_BT_FRAME_TYPE_NORMAL    = 0x00,
    SCH_BT_FRAME_TYPE_SNIFF     = 0x01,
    SCH_BT_FRAME_TYPE_ESCO      = 0x02,
    SCH_BT_FRAME_TYPE_CSB       = 0x03,
    SCH_BT_FRAME_TYPE_ESCO_RETX = 0x04,
};
#endif //BT_EMB_PRESENT


/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Callback for interrupt related to the frame
 *
 * @param[in]   timestamp   Timestamp of the programmed frame (in BLE slots, based on local clock)
 * @param[in]   dummy       Dummy value (provided by the driver when programming the frame via sch_prog_push)
 * @param[in]   irq_type    IRQ type (enum SCH_FRAME_IRQ)
 ****************************************************************************************
 */
typedef void (*frm_cbk_t)(uint32_t timestamp, uint32_t dummy, uint8_t irq_type);

#if BT_EMB_PRESENT
    /**
    ****************************************************************************************
    * @brief Callback for interrupt related to the clock IRQ
    *
    * @param[in]   clock       Clock when the IRQ happened
    * @param[in]   id          Identification value
    ****************************************************************************************
    */
    typedef void (*clk_cbk_t)(uint32_t clock, uint8_t id);
#endif //BT_EMB_PRESENT

#if BLE_EMB_PRESENT
/// Additional parameters for BLE frame
struct sch_prog_params_ble
{
    /// Advertising channel type: 0x0: AE Start on Primary channel | 0x1: AE Start on Secondary channel
    uint8_t ae_nps;
    /// Reserved audio event (0: reTx, 1:primary)
    uint8_t rsvd;
    /// Isochronous event (0: normal event | 1: iso event)
    uint8_t iso;
    /// Start Instant Correction (SIC) bit value
    uint8_t sic;
};
#endif //BLE_EMB_PRESENT

#if BT_EMB_PRESENT
/// Additional parameters for BT frame
struct sch_prog_params_bt
{
    /// Frame type (0: normal | 1: sniff | 2: ESCO | 3: CSB)
    uint8_t frm_type;
    /// Voice channel
    uint8_t vxchan;
};
#endif //BT_EMB_PRESENT

/// Parameters for a programmed frame
struct sch_prog_params
{
    /// Callback for handling interrupts related to the frame
    frm_cbk_t frm_cbk;
    /// Timestamp (in half-slots, based on local clock) and event offset (in half-us) of the programmed frame
    rwip_time_t time;
    /// Bandwidth duration of the event using priority 1 (in half us)
    uint32_t bandwidth;
    /// Dummy value reported when an event happen during the frame or the frame is completed
    uint32_t dummy;
    /// Priority during duration of bandwidth
    uint8_t prio_1;
    /// Priority after bandwidth elapsed
    uint8_t prio_2;
    /// Priority after trigger conditions
    uint8_t prio_3;
    /// Priority when specific action occurs during the event
    uint8_t pti_prio;
    /// Control structure index
    uint8_t cs_idx;
    /// Mode (0: BLE, 1:BT)
    uint8_t mode;

    union
    {
#if BLE_EMB_PRESENT
        /// Additional parameters for BLE frame
        struct sch_prog_params_ble ble;
#endif //BLE_EMB_PRESENT

#if BT_EMB_PRESENT
        /// Additional parameters for BT frame
        struct sch_prog_params_bt bt;
#endif //BT_EMB_PRESENT
    } add;
};


/*
 * FUNCTIONS DECLARATION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialize the SCH_PROG module
 *
 * @param[in] is_reset  True if reset is requested, false for an initialization
 ****************************************************************************************
 */
void sch_prog_init(bool is_reset);

/**
 ****************************************************************************************
 * @brief Handle "RX" interrupt
 *
 * @param[in] et_idx Entry index that trigger the ISR (@note Valid only with 5.0 + ISO HW)
 ****************************************************************************************
 */
void sch_prog_rx_isr(uint8_t et_idx);

/**
 ****************************************************************************************
 * @brief Handle FIFO interrupt
 ****************************************************************************************
 */
void sch_prog_fifo_isr(void);

/**
****************************************************************************************
* @brief Push a new programmed frame
*
* @param[in]   params     Parameters structure
****************************************************************************************
*/
void sch_prog_push(struct sch_prog_params *params);

#if BT_EMB_PRESENT
    /**
    ****************************************************************************************
    * @brief Enable the programming of an activity
    *
    * @param[in]   clk_cbk     Callback for handling interrupts related to the clock
    * @param[in]   id          Identification value
    * @param[in]   sr_size     Size of the sub rating pattern (in number of bits), 0 if no sub rating
    * @param[in]   sr_val      Sub rating pattern value, representing the clock LSBs when interrupt is raised
    ****************************************************************************************
    */
    void sch_prog_enable(clk_cbk_t clk_cbk, uint8_t id, uint8_t sr_size, uint8_t sr_val);

    /**
    ****************************************************************************************
    * @brief Disable the programming of an activity
    *
    * @param[in]   id          Identification value
    ****************************************************************************************
    */
    void sch_prog_disable(uint8_t id);

    /**
    ****************************************************************************************
    * @brief Handle "clock" interrupt
    ****************************************************************************************
    */
    void sch_prog_clk_isr(void);

    /**
    ****************************************************************************************
    * @brief Handle "Skip" interrupt
    *
    * @param[in] et_idx Entry index that trigger the ISR (@note Valid only with 5.0 + ISO HW)
    ****************************************************************************************
    */
    void sch_prog_skip_isr(uint8_t et_idx);

    /**
    ****************************************************************************************
    * @brief Handle "End of Event" interrupt
    *
    * @param[in] et_idx Entry index that trigger the ISR (@note Valid only with 5.0 + ISO HW)
    ****************************************************************************************
    */
    void sch_prog_end_isr(uint8_t et_idx);

#endif //BT_EMB_PRESENT

/// @} SCH_PROG
#endif // SCH_PROG_H_
