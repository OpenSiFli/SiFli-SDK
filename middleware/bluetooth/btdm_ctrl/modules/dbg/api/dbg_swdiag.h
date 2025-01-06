/**
****************************************************************************************
*
* @file dbg_swdiag.h
*
* @brief SW profiling module
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/

#ifndef DBG_SWDIAG_H_
#define DBG_SWDIAG_H_

/**
 ****************************************************************************************
 * @addtogroup DBGSWDIAG Diag
 * @ingroup DBG
 * @brief Debug SW profiling module
 *
 * SW profiling is a debug feature that provides user a configurable way to analyze SW execution performance or
 * behavior, such as timings, state machines, bit field values and so on.
 * It manages the SW state representation over HW digital diagnostic signals (diagports), from the signals selection to
 * the toggling of real HW signals.
 * HW signals representing the SW execution could then be watched on a logic analyzer in parallel with HW internal
 * signals, that could be very useful for low level debugging.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"       // stack configuration

#if (!RW_SWDIAG)

/// Macro used to set a SW diag to specific value
#define DBG_SWDIAG(bank , field , value)

#else //RW_SWDIAG

#if 0
    #if (BT_DUAL_MODE)
        #include "reg_ipcore.h"          // btdm core registers
    #else // !(BT_DUAL_MODE)
        #if (BT_EMB_PRESENT) // TODO not fully agree with this
            #include "reg_btcore.h"          // bt core registers
        #elif (BLE_EMB_PRESENT)
            #include "reg_blecore.h"         // ble core registers
        #endif //BT_EMB_PRESENT / BLE_EMB_PRESENT
    #endif // (BT_DUAL_MODE)
#endif

#include "rom_config.h"
#include "reg_ipcore.h"
#include "reg_blecore.h"
#include "reg_btcore.h"


/*
 * DEFINES
 ****************************************************************************************
 */

/// Number of HW diagport banks
#define DBG_SWDIAG_NB_HW_BANKS        4

/// Undefined configuration
#define DBG_SWDIAG_UNDEFINED        0xFF


/*
 * MACROS
 ****************************************************************************************
 */
#if 0
    #if (BT_DUAL_MODE)
        #define SW_DIAG_SET     ip_swprofiling_set
        #define SW_DIAG_GET     ip_swprofiling_get
    #else // !(BT_DUAL_MODE)
        #if (BT_EMB_PRESENT)
            #define SW_DIAG_SET     bt_swprofiling_set
            #define SW_DIAG_GET     bt_swprofiling_get
        #elif (BLE_EMB_PRESENT)
            #define SW_DIAG_SET     ble_swprofiling_set
            #define SW_DIAG_GET     ble_swprofiling_get
        #endif //BT_EMB_PRESENT / BLE_EMB_PRESENT
    #endif // (BT_DUAL_MODE)
#endif


void btdm_swprofiling_set(uint32_t value);
uint32_t btdm_swprofiling_get(void);


#define SW_DIAG_SET     btdm_swprofiling_set
#define SW_DIAG_GET     btdm_swprofiling_get


/// Macro used to set a SW diag to specific value
#define DBG_SWDIAG(bank , field , value)                                              \
    if(sw_to_hw[DBG_SWDIAG_##bank] != DBG_SWDIAG_UNDEFINED)                           \
    {                                                                                 \
        SW_DIAG_SET( ((SW_DIAG_GET()) & (~(DBG_SWDIAG_##bank##_##field##_MASK << (8 * sw_to_hw[DBG_SWDIAG_##bank])))) | ((((value << DBG_SWDIAG_##bank##_##field##_OFFSET) & DBG_SWDIAG_##bank##_##field##_MASK) << (8*sw_to_hw[DBG_SWDIAG_##bank])))  );      \
    }


/*
 * SW DIAGS MAP
 ****************************************************************************************
 */

enum dbg_swdiag
{
    /**
    ******************************************************************************************
    * @brief BANK 0 : BT_ISR
    ******************************************************************************************
    */
    DBG_SWDIAG_BT_ISR                          = 0,

    DBG_SWDIAG_BT_ISR_CLK_OFFSET               = 0,
    DBG_SWDIAG_BT_ISR_CLK_MASK                 = 0x01,

    DBG_SWDIAG_BT_ISR_RX_OFFSET                = 1,
    DBG_SWDIAG_BT_ISR_RX_MASK                  = 0x02,

    DBG_SWDIAG_BT_ISR_SLP_OFFSET               = 2,
    DBG_SWDIAG_BT_ISR_SLP_MASK                 = 0x04,

    DBG_SWDIAG_BT_ISR_GROSSTGT_OFFSET          = 3,
    DBG_SWDIAG_BT_ISR_GROSSTGT_MASK            = 0x08,

    DBG_SWDIAG_BT_ISR_FRAME_OFFSET             = 4,
    DBG_SWDIAG_BT_ISR_FRAME_MASK               = 0x10,

    DBG_SWDIAG_BT_ISR_FINETGT_OFFSET           = 5,
    DBG_SWDIAG_BT_ISR_FINETGT_MASK             = 0x20,

    DBG_SWDIAG_BT_ISR_SKET_OFFSET              = 6,
    DBG_SWDIAG_BT_ISR_SKET_MASK                = 0x40,

    DBG_SWDIAG_BT_ISR_SW_OFFSET                = 7,
    DBG_SWDIAG_BT_ISR_SW_MASK                  = 0x80,

    /**
    ******************************************************************************************
    * @brief BANK 1 : SLEEP
    ******************************************************************************************
    */
    DBG_SWDIAG_SLEEP                           = 1,

    DBG_SWDIAG_SLEEP_SLEEP_OFFSET              = 0,
    DBG_SWDIAG_SLEEP_SLEEP_MASK                = 0x01,

    DBG_SWDIAG_SLEEP_FUNC_OFFSET               = 1,
    DBG_SWDIAG_SLEEP_FUNC_MASK                 = 0x02,

    DBG_SWDIAG_SLEEP_WAKEUP_END_OFFSET         = 2,
    DBG_SWDIAG_SLEEP_WAKEUP_END_MASK           = 0x04,

    DBG_SWDIAG_SLEEP_ALGO_OFFSET               = 4,
    DBG_SWDIAG_SLEEP_ALGO_MASK                 = 0xF0,

    /**
    ******************************************************************************************
    * @brief BANK 2 : ISR
    ******************************************************************************************
    */
    DBG_SWDIAG_ISR                             = 2,

    DBG_SWDIAG_ISR_UART_OFFSET                 = 0,
    DBG_SWDIAG_ISR_UART_MASK                   = 0x01,

    DBG_SWDIAG_ISR_BT_OFFSET                   = 1,
    DBG_SWDIAG_ISR_BT_MASK                     = 0x02,

    DBG_SWDIAG_ISR_PS2_OFFSET                  = 1,
    DBG_SWDIAG_ISR_PS2_MASK                    = 0x02,

    DBG_SWDIAG_ISR_BLE_OFFSET                  = 2,
    DBG_SWDIAG_ISR_BLE_MASK                    = 0x04,

    DBG_SWDIAG_ISR_RWIP_OFFSET                 = 3,
    DBG_SWDIAG_ISR_RWIP_MASK                   = 0x08,

    DBG_SWDIAG_ISR_GPIO_OFFSET                 = 4,
    DBG_SWDIAG_ISR_GPIO_MASK                   = 0x10,

    DBG_SWDIAG_ISR_RTC0_OFFSET                 = 5,
    DBG_SWDIAG_ISR_RTC0_MASK                   = 0x20,

    DBG_SWDIAG_ISR_BTDM_OFFSET                 = 5,
    DBG_SWDIAG_ISR_BTDM_MASK                   = 0x20,

    DBG_SWDIAG_ISR_SPI_OFFSET                  = 6,
    DBG_SWDIAG_ISR_SPI_MASK                    = 0x40,

    DBG_SWDIAG_ISR_WFI_OFFSET                  = 7,
    DBG_SWDIAG_ISR_WFI_MASK                    = 0x80,

    /**
    ******************************************************************************************
    * @brief BANK 3 : BLEISR
    ******************************************************************************************
    */
    DBG_SWDIAG_BLE_ISR                         = 3,

    DBG_SWDIAG_BLE_ISR_RXINT_OFFSET            = 0,
    DBG_SWDIAG_BLE_ISR_RXINT_MASK              = 0x01,

    DBG_SWDIAG_BLE_ISR_TXINT_OFFSET            = 1,
    DBG_SWDIAG_BLE_ISR_TXINT_MASK              = 0x02,

    DBG_SWDIAG_BLE_ISR_EVENTINT_OFFSET         = 2,
    DBG_SWDIAG_BLE_ISR_EVENTINT_MASK           = 0x04,

    DBG_SWDIAG_BLE_ISR_SKIPINT_OFFSET          = 3,
    DBG_SWDIAG_BLE_ISR_SKIPINT_MASK            = 0x08,

    DBG_SWDIAG_BLE_ISR_ERRORINT_OFFSET         = 4,
    DBG_SWDIAG_BLE_ISR_ERRORINT_MASK           = 0x10,

    DBG_SWDIAG_BLE_ISR_AUDIO0INT_OFFSET        = 5,
    DBG_SWDIAG_BLE_ISR_AUDIO0INT_MASK          = 0x20,

    DBG_SWDIAG_BLE_ISR_AUDIO1INT_OFFSET        = 6,
    DBG_SWDIAG_BLE_ISR_AUDIO1INT_MASK          = 0x40,

    DBG_SWDIAG_BLE_ISR_AUDIO2INT_OFFSET        = 7,
    DBG_SWDIAG_BLE_ISR_AUDIO2INT_MASK          = 0x80,

    DBG_SWDIAG_BLE_ISR_RXISOINT_OFFSET         = 5,
    DBG_SWDIAG_BLE_ISR_RXISOINT_MASK           = 0x20,

    DBG_SWDIAG_BLE_ISR_TXISOINT_OFFSET         = 6,
    DBG_SWDIAG_BLE_ISR_TXISOINT_MASK           = 0x40,

    /**
    ******************************************************************************************
    * @brief BANK 4 : FLASH
    ******************************************************************************************
    */
    DBG_SWDIAG_FLASH                           = 4,

    DBG_SWDIAG_FLASH_STATE_OFFSET              = 0,
    DBG_SWDIAG_FLASH_STATE_MASK                = 0x07,

    DBG_SWDIAG_FLASH_SUBSTATE_OFFSET           = 3,
    DBG_SWDIAG_FLASH_SUBSTATE_MASK             = 0x38,

    DBG_SWDIAG_FLASH_MANAGE_OFFSET             = 6,
    DBG_SWDIAG_FLASH_MANAGE_MASK               = 0x40,

    DBG_SWDIAG_FLASH_CALLBACK_OFFSET           = 7,
    DBG_SWDIAG_FLASH_CALLBACK_MASK             = 0x80,

    /**
    ******************************************************************************************
    * @brief BANK 5 : Reserved
    ******************************************************************************************
    */

    /**
    ******************************************************************************************
    * @brief BANK 6 : Kernel
    ******************************************************************************************
    */
    DBG_SWDIAG_KE                               = 6,

    DBG_SWDIAG_KE_TIMER_OFFSET                  = 0,
    DBG_SWDIAG_KE_TIMER_MASK                    = 0x01,

    DBG_SWDIAG_KE_MESSAGE_OFFSET                = 1,
    DBG_SWDIAG_KE_MESSAGE_MASK                  = 0x02,

    DBG_SWDIAG_KE_MALLOC_OFFSET                 = 2,
    DBG_SWDIAG_KE_MALLOC_MASK                   = 0x04,

    DBG_SWDIAG_KE_FREE_OFFSET                   = 3,
    DBG_SWDIAG_KE_FREE_MASK                     = 0x08,



    /**
    ******************************************************************************************
    * @brief BANK 8 : AFH
    ******************************************************************************************
    */
    DBG_SWDIAG_AFH                               = 8,

    DBG_SWDIAG_AFH_TO_OFFSET                     = 0,
    DBG_SWDIAG_AFH_TO_MASK                       = 0x01,

    DBG_SWDIAG_AFH_LMP_TX_OFFSET                 = 1,
    DBG_SWDIAG_AFH_LMP_TX_MASK                   = 0x02,

    DBG_SWDIAG_AFH_CLASS_RX_OFFSET               = 2,
    DBG_SWDIAG_AFH_CLASS_RX_MASK                 = 0x04,

    DBG_SWDIAG_AFH_HOST_OFFSET                   = 3,
    DBG_SWDIAG_AFH_HOST_MASK                     = 0x08,

    DBG_SWDIAG_AFH_REPORT_TO_OFFSET              = 4,
    DBG_SWDIAG_AFH_REPORT_TO_MASK                = 0x10,

    DBG_SWDIAG_AFH_HSSI_OFFSET                   = 5,
    DBG_SWDIAG_AFH_HSSI_MASK                     = 0x20,

    DBG_SWDIAG_AFH_CFM_OFFSET                    = 6,
    DBG_SWDIAG_AFH_CFM_MASK                      = 0x40,

    DBG_SWDIAG_AFH_LMP_RX_OFFSET                 = 7,
    DBG_SWDIAG_AFH_LMP_RX_MASK                   = 0x80,

    /**
    ******************************************************************************************
    * @brief BANK 9 : SWITCH
    ******************************************************************************************
    */
    DBG_SWDIAG_SWITCH                            = 9,

    DBG_SWDIAG_SWITCH_LOOP_OFFSET                = 0,
    DBG_SWDIAG_SWITCH_LOOP_MASK                  = 0x01,

    DBG_SWDIAG_SWITCH_SM_OFFSET                  = 1,
    DBG_SWDIAG_SWITCH_SM_MASK                    = 0x06,

    DBG_SWDIAG_SWITCH_RSW_FAIL_OFFSET            = 3,
    DBG_SWDIAG_SWITCH_RSW_FAIL_MASK              = 0x08,

    /**
    ******************************************************************************************
    * @brief BANK 11 : VALUE8
    ******************************************************************************************
    */
    DBG_SWDIAG_VALUE8                            = 11,

    DBG_SWDIAG_VALUE8_VALUE_OFFSET               = 0,
    DBG_SWDIAG_VALUE8_VALUE_MASK                 = 0xFF,

    /**
    ******************************************************************************************
    * @brief BANK 12 : VALUE16
    ******************************************************************************************
    */
    DBG_SWDIAG_VALUE16                           = 12,

    DBG_SWDIAG_VALUE16_VALUE_OFFSET              = 0,
    DBG_SWDIAG_VALUE16_VALUE_MASK                = 0xFF,

    /**
    ******************************************************************************************
    * @brief BANK 13 : SNIFF
    ******************************************************************************************
    */
    DBG_SWDIAG_SNIFF                             = 13,

    DBG_SWDIAG_SNIFF_EVT_START_OFFSET            = 0,
    DBG_SWDIAG_SNIFF_EVT_START_MASK              = 0x01,

    DBG_SWDIAG_SNIFF_EVT_CANCELED_OFFSET         = 1,
    DBG_SWDIAG_SNIFF_EVT_CANCELED_MASK           = 0x02,

    DBG_SWDIAG_SNIFF_FRM_ISR_OFFSET              = 2,
    DBG_SWDIAG_SNIFF_FRM_ISR_MASK                = 0x04,

    DBG_SWDIAG_SNIFF_RX_ISR_OFFSET               = 3,
    DBG_SWDIAG_SNIFF_RX_ISR_MASK                 = 0x08,

    DBG_SWDIAG_SNIFF_SCHED_OFFSET                = 4,
    DBG_SWDIAG_SNIFF_SCHED_MASK                  = 0x10,

    /**
    ******************************************************************************************
    * @brief BANK 14 : SCO
    ******************************************************************************************
    */
    DBG_SWDIAG_SCO                               = 14,

    DBG_SWDIAG_SCO_EVT_START_OFFSET              = 0,
    DBG_SWDIAG_SCO_EVT_START_MASK                = 0x01,

    DBG_SWDIAG_SCO_EVT_STOP_OFFSET               = 1,
    DBG_SWDIAG_SCO_EVT_STOP_MASK                 = 0x02,

    DBG_SWDIAG_SCO_EVT_CANCELED_OFFSET           = 2,
    DBG_SWDIAG_SCO_EVT_CANCELED_MASK             = 0x04,

    DBG_SWDIAG_SCO_FRM_ISR_OFFSET                = 3,
    DBG_SWDIAG_SCO_FRM_ISR_MASK                  = 0x08,

    DBG_SWDIAG_SCO_RESCHED_OFFSET                = 4,
    DBG_SWDIAG_SCO_RESCHED_MASK                  = 0x10,

    DBG_SWDIAG_SCO_UPDATE_OFFSET                 = 5,
    DBG_SWDIAG_SCO_UPDATE_MASK                   = 0x60,

    DBG_SWDIAG_SCO_AUDIO_ISR_OFFSET              = 7,
    DBG_SWDIAG_SCO_AUDIO_ISR_MASK                = 0x80,


    /**
    ******************************************************************************************
    * @brief BANK 17 : PAGE
    ******************************************************************************************
    */
    DBG_SWDIAG_PAGE                              = 17,

    DBG_SWDIAG_PAGE_PSCAN_EVT_START_OFFSET       = 0,
    DBG_SWDIAG_PAGE_PSCAN_EVT_START_MASK         = 0x01,

    DBG_SWDIAG_PAGE_PSCAN_EVT_CANCELED_OFFSET    = 1,
    DBG_SWDIAG_PAGE_PSCAN_EVT_CANCELED_MASK      = 0x02,

    DBG_SWDIAG_PAGE_PSCAN_FRM_ISR_OFFSET         = 2,
    DBG_SWDIAG_PAGE_PSCAN_FRM_ISR_MASK           = 0x04,

    DBG_SWDIAG_PAGE_PSCAN_KE_EVT_OFFSET          = 3,
    DBG_SWDIAG_PAGE_PSCAN_KE_EVT_MASK            = 0x08,

    DBG_SWDIAG_PAGE_PAGE_EVT_START_OFFSET        = 4,
    DBG_SWDIAG_PAGE_PAGE_EVT_START_MASK          = 0x10,

    DBG_SWDIAG_PAGE_PAGE_EVT_CANCELED_OFFSET     = 5,
    DBG_SWDIAG_PAGE_PAGE_EVT_CANCELED_MASK       = 0x20,

    DBG_SWDIAG_PAGE_PAGE_FRM_ISR_OFFSET          = 6,
    DBG_SWDIAG_PAGE_PAGE_FRM_ISR_MASK            = 0x40,

    DBG_SWDIAG_PAGE_PAGE_STEP_OFFSET             = 7,
    DBG_SWDIAG_PAGE_PAGE_STEP_MASK               = 0x80,

    /**
    ******************************************************************************************
    * @brief BANK 18 : INQ
    ******************************************************************************************
    */
    DBG_SWDIAG_INQ                               = 18,

    DBG_SWDIAG_INQ_ISCAN_EVT_START_OFFSET        = 0,
    DBG_SWDIAG_INQ_ISCAN_EVT_START_MASK          = 0x01,

    DBG_SWDIAG_INQ_ISCAN_EVT_CANCELED_OFFSET     = 1,
    DBG_SWDIAG_INQ_ISCAN_EVT_CANCELED_MASK       = 0x02,

    DBG_SWDIAG_INQ_ISCAN_FRM_ISR_OFFSET          = 2,
    DBG_SWDIAG_INQ_ISCAN_FRM_ISR_MASK            = 0x04,

    DBG_SWDIAG_INQ_INQ_EVT_START_OFFSET          = 3,
    DBG_SWDIAG_INQ_INQ_EVT_START_MASK            = 0x08,

    DBG_SWDIAG_INQ_INQ_EVT_CANCELED_OFFSET       = 4,
    DBG_SWDIAG_INQ_INQ_EVT_CANCELED_MASK         = 0x10,

    DBG_SWDIAG_INQ_INQ_FRM_ISR_OFFSET            = 5,
    DBG_SWDIAG_INQ_INQ_FRM_ISR_MASK              = 0x20,

    /**
    ******************************************************************************************
    * @brief BANK 19 : ACL
    ******************************************************************************************
    */
    DBG_SWDIAG_ACL                               = 19,

    DBG_SWDIAG_ACL_EVT_START_OFFSET              = 0,
    DBG_SWDIAG_ACL_EVT_START_MASK                = 0x01,

    DBG_SWDIAG_ACL_EVT_STOP_OFFSET               = 1,
    DBG_SWDIAG_ACL_EVT_STOP_MASK                 = 0x02,

    DBG_SWDIAG_ACL_EVT_CANCELED_OFFSET           = 2,
    DBG_SWDIAG_ACL_EVT_CANCELED_MASK             = 0x04,

    DBG_SWDIAG_ACL_FRM_ISR_OFFSET                = 3,
    DBG_SWDIAG_ACL_FRM_ISR_MASK                  = 0x08,

    DBG_SWDIAG_ACL_CLK_ISR_OFFSET                = 4,
    DBG_SWDIAG_ACL_CLK_ISR_MASK                  = 0x10,

    DBG_SWDIAG_ACL_PROG_EN_OFFSET                = 5,
    DBG_SWDIAG_ACL_PROG_EN_MASK                  = 0x20,

    DBG_SWDIAG_ACL_SKET_ISR_OFFSET               = 6,
    DBG_SWDIAG_ACL_SKET_ISR_MASK                 = 0x40,

    DBG_SWDIAG_ACL_PARITY_OFFSET                 = 7,
    DBG_SWDIAG_ACL_PARITY_MASK                   = 0x80,

    /**
    ******************************************************************************************
    * @brief BANK 20 : RSW
    ******************************************************************************************
    */
    DBG_SWDIAG_RSW                               = 20,

    DBG_SWDIAG_RSW_EVT_START_OFFSET              = 0,
    DBG_SWDIAG_RSW_EVT_START_MASK                = 0x01,

    DBG_SWDIAG_RSW_EVT_CANCELED_OFFSET           = 1,
    DBG_SWDIAG_RSW_EVT_CANCELED_MASK             = 0x02,

    DBG_SWDIAG_RSW_FRM_ISR_OFFSET                = 2,
    DBG_SWDIAG_RSW_FRM_ISR_MASK                  = 0x04,

    DBG_SWDIAG_RSW_STEP_OFFSET                   = 3,
    DBG_SWDIAG_RSW_STEP_MASK                     = 0x08,

    DBG_SWDIAG_RSW_TX_SLOTOFF_OFFSET             = 4,
    DBG_SWDIAG_RSW_TX_SLOTOFF_MASK               = 0x10,

    DBG_SWDIAG_RSW_TX_PARITY_OFFSET              = 5,
    DBG_SWDIAG_RSW_TX_PARITY_MASK                = 0x20,

    DBG_SWDIAG_RSW_TDD_PARITY_OFFSET             = 6,
    DBG_SWDIAG_RSW_TDD_PARITY_MASK               = 0x40,

    /**
    ******************************************************************************************
    * @brief BANK 21 : TX
    ******************************************************************************************
    */
    DBG_SWDIAG_TX                                = 21,

    DBG_SWDIAG_TX_LMP_TX_OFFSET                  = 0,
    DBG_SWDIAG_TX_LMP_TX_MASK                    = 0x01,

    DBG_SWDIAG_TX_ACL_TX_OFFSET                  = 1,
    DBG_SWDIAG_TX_ACL_TX_MASK                    = 0x02,

    DBG_SWDIAG_TX_SW_IDX_OFFSET                  = 2,
    DBG_SWDIAG_TX_SW_IDX_MASK                    = 0x04,

    DBG_SWDIAG_TX_HW_IDX_OFFSET                  = 3,
    DBG_SWDIAG_TX_HW_IDX_MASK                    = 0x08,

    DBG_SWDIAG_TX_TX_CNT_OFFSET                  = 4,
    DBG_SWDIAG_TX_TX_CNT_MASK                    = 0x30,

    /**
    ******************************************************************************************
    * @brief BANK 22 : DATA
    ******************************************************************************************
    */
    DBG_SWDIAG_DATA                              = 22,

    DBG_SWDIAG_DATA_LMP_TX_OFFSET                = 0,
    DBG_SWDIAG_DATA_LMP_TX_MASK                  = 0x01,

    DBG_SWDIAG_DATA_ACL_TX_OFFSET                = 1,
    DBG_SWDIAG_DATA_ACL_TX_MASK                  = 0x02,

    DBG_SWDIAG_DATA_LMP_RX_OFFSET                = 2,
    DBG_SWDIAG_DATA_LMP_RX_MASK                  = 0x04,

    DBG_SWDIAG_DATA_ACL_RX_OFFSET                = 3,
    DBG_SWDIAG_DATA_ACL_RX_MASK                  = 0x08,

    DBG_SWDIAG_DATA_LMP_ACK_OFFSET               = 4,
    DBG_SWDIAG_DATA_LMP_ACK_MASK                 = 0x10,

    DBG_SWDIAG_DATA_ACL_ACK_OFFSET               = 5,
    DBG_SWDIAG_DATA_ACL_ACK_MASK                 = 0x20,

    /**
    ******************************************************************************************
    * @brief BANK 23 : ECDH
    ******************************************************************************************
    */
    DBG_SWDIAG_ECDH                              = 23,

    DBG_SWDIAG_ECDH_BUSY_OFFSET                  = 0,
    DBG_SWDIAG_ECDH_BUSY_MASK                    = 0x01,

    DBG_SWDIAG_ECDH_COMPUTE_OFFSET               = 1,
    DBG_SWDIAG_ECDH_COMPUTE_MASK                 = 0x02,

    DBG_SWDIAG_ECDH_MULT_OFFSET                  = 2,
    DBG_SWDIAG_ECDH_MULT_MASK                    = 0x04,

    DBG_SWDIAG_ECDH_END_OFFSET                   = 3,
    DBG_SWDIAG_ECDH_END_MASK                     = 0x08,

    DBG_SWDIAG_ECDH_INV_OFFSET                   = 4,
    DBG_SWDIAG_ECDH_INV_MASK                     = 0x10,

    /**
     ******************************************************************************************
     * @brief BANK 24 : PCA
     ******************************************************************************************
     */
    DBG_SWDIAG_PCA                               = 24,

    DBG_SWDIAG_PCA_FRAME_SYNC_INT_OFFSET         = 0,
    DBG_SWDIAG_PCA_FRAME_SYNC_INT_MASK           = 0x01,

    DBG_SWDIAG_PCA_MTOFF_LT_INT_OFFSET           = 1,
    DBG_SWDIAG_PCA_MTOFF_LT_INT_MASK             = 0x02,

    DBG_SWDIAG_PCA_SLOT_INT_OFFSET               = 2,
    DBG_SWDIAG_PCA_SLOT_INT_MASK                 = 0x04,

    DBG_SWDIAG_PCA_COARSE_CLK_ADJ_OFFSET         = 3,
    DBG_SWDIAG_PCA_COARSE_CLK_ADJ_MASK           = 0x08,

    DBG_SWDIAG_PCA_EVT_START_OFFSET              = 4,
    DBG_SWDIAG_PCA_EVT_START_MASK                = 0x10,

    DBG_SWDIAG_PCA_EVT_CANCELED_OFFSET           = 5,
    DBG_SWDIAG_PCA_EVT_CANCELED_MASK             = 0x20,

    DBG_SWDIAG_PCA_ALARM_INSTANT_OFFSET          = 6,
    DBG_SWDIAG_PCA_ALARM_INSTANT_MASK            = 0x40,

    DBG_SWDIAG_PCA_CLK_DRAG_OFFSET               = 7,
    DBG_SWDIAG_PCA_CLK_DRAG_MASK                 = 0x80,

    /**
     ******************************************************************************************
     * @brief BANK 25 : BCST
     ******************************************************************************************
     */
    DBG_SWDIAG_BCST                              = 25,

    DBG_SWDIAG_BCST_FRM_ISR_OFFSET               = 0,
    DBG_SWDIAG_BCST_FRM_ISR_MASK                 = 0x01,

    DBG_SWDIAG_BCST_EVT_START_OFFSET             = 1,
    DBG_SWDIAG_BCST_EVT_START_MASK               = 0x02,

    DBG_SWDIAG_BCST_SKET_ISR_OFFSET              = 2,
    DBG_SWDIAG_BCST_SKET_ISR_MASK                = 0x04,

    DBG_SWDIAG_BCST_EVT_CANCELED_OFFSET          = 3,
    DBG_SWDIAG_BCST_EVT_CANCELED_MASK            = 0x08,

    DBG_SWDIAG_BCST_ACL_TX_OFFSET                = 4,
    DBG_SWDIAG_BCST_ACL_TX_MASK                  = 0x10,

    DBG_SWDIAG_BCST_LMP_TX_OFFSET                = 5,
    DBG_SWDIAG_BCST_LMP_TX_MASK                  = 0x20,

    DBG_SWDIAG_BCST_PDU_TX_CFM_OFFSET            = 6,
    DBG_SWDIAG_BCST_PDU_TX_CFM_MASK              = 0x40,

    DBG_SWDIAG_BCST_PDU_RX_OFFSET                = 7,
    DBG_SWDIAG_BCST_PDU_RX_MASK                  = 0x80,

    /**
    ******************************************************************************************
    * @brief IP_ISR - 26
    ******************************************************************************************
    */
    DBG_SWDIAG_IP_ISR                          = 26,

    DBG_SWDIAG_IP_ISR_CLKNINT_OFFSET           = 0,
    DBG_SWDIAG_IP_ISR_CLKNINT_MASK             = 0x01,

    DBG_SWDIAG_IP_ISR_CO_TIMER_OFFSET          = 1,
    DBG_SWDIAG_IP_ISR_CO_TIMER_MASK            = 0x02,

    DBG_SWDIAG_IP_ISR_ALARM_TIMER_OFFSET       = 2,
    DBG_SWDIAG_IP_ISR_ALARM_TIMER_MASK         = 0x04,

    DBG_SWDIAG_IP_ISR_SLPINT_OFFSET            = 3,
    DBG_SWDIAG_IP_ISR_SLPINT_MASK              = 0x08,

    DBG_SWDIAG_IP_ISR_CRYPTINT_OFFSET          = 4,
    DBG_SWDIAG_IP_ISR_CRYPTINT_MASK            = 0x10,

    DBG_SWDIAG_IP_ISR_SWINT_OFFSET             = 5,
    DBG_SWDIAG_IP_ISR_SWINT_MASK               = 0x20,

    DBG_SWDIAG_IP_ISR_ARB_TIMER_OFFSET         = 6,
    DBG_SWDIAG_IP_ISR_ARB_TIMER_MASK           = 0x40,

    DBG_SWDIAG_IP_ISR_FIFOINT_OFFSET           = 7,
    DBG_SWDIAG_IP_ISR_FIFOINT_MASK             = 0x80,

    /**
    ******************************************************************************************
    * @brief RSA Algorithm - 27
    ******************************************************************************************
    */
    DBG_SWDIAG_RSA                              = 27,

    DBG_SWDIAG_RSA_BUSY_OFFSET                  = 0,
    DBG_SWDIAG_RSA_BUSY_MASK                    = 0x01,

    DBG_SWDIAG_RSA_COMPUTE_OFFSET               = 1,
    DBG_SWDIAG_RSA_COMPUTE_MASK                 = 0x02,

    /**
     ******************************************************************************************
     * @brief SYNC SCAN - 28
     ******************************************************************************************
     */
    DBG_SWDIAG_SSCAN                              = 28,

    DBG_SWDIAG_SSCAN_FRM_ISR_OFFSET               = 0,
    DBG_SWDIAG_SSCAN_FRM_ISR_MASK                 = 0x01,

    DBG_SWDIAG_SSCAN_EVT_START_OFFSET             = 1,
    DBG_SWDIAG_SSCAN_EVT_START_MASK               = 0x02,

    DBG_SWDIAG_SSCAN_START_OFFSET                 = 2,
    DBG_SWDIAG_SSCAN_START_MASK                   = 0x04,

    DBG_SWDIAG_SSCAN_EVT_CANCELED_OFFSET          = 3,
    DBG_SWDIAG_SSCAN_EVT_CANCELED_MASK            = 0x08,

    DBG_SWDIAG_SSCAN_FRM_CBK_OFFSET               = 4,
    DBG_SWDIAG_SSCAN_FRM_CBK_MASK                 = 0x10,

    DBG_SWDIAG_SSCAN_STP_RX_OFFSET                = 5,
    DBG_SWDIAG_SSCAN_STP_RX_MASK                  = 0x20,

    DBG_SWDIAG_SSCAN_STP_RX_OK_OFFSET             = 6,
    DBG_SWDIAG_SSCAN_STP_RX_OK_MASK               = 0x40,

    DBG_SWDIAG_SSCAN_TIMEOUT_OFFSET               = 7,
    DBG_SWDIAG_SSCAN_TIMEOUT_MASK                 = 0x80,

    /**
     ******************************************************************************************
     * @brief BANK 29 : LE SCAN
     ******************************************************************************************
     */
    DBG_SWDIAG_LESCAN                              = 29,

    DBG_SWDIAG_LESCAN_START_OFFSET                 = 0,
    DBG_SWDIAG_LESCAN_START_MASK                   = 0x01,

    DBG_SWDIAG_LESCAN_STOP_OFFSET                  = 1,
    DBG_SWDIAG_LESCAN_STOP_MASK                    = 0x02,

    DBG_SWDIAG_LESCAN_EVT_START_OFFSET             = 2,
    DBG_SWDIAG_LESCAN_EVT_START_MASK               = 0x04,

    DBG_SWDIAG_LESCAN_EVT_CANCELED_OFFSET          = 3,
    DBG_SWDIAG_LESCAN_EVT_CANCELED_MASK            = 0x08,

    DBG_SWDIAG_LESCAN_SYNC_INFO_OFFSET            = 4,
    DBG_SWDIAG_LESCAN_SYNC_INFO_MASK              = 0x10,

    DBG_SWDIAG_LESCAN_FRM_EOF_ISR_OFFSET           = 5,
    DBG_SWDIAG_LESCAN_FRM_EOF_ISR_MASK             = 0x20,

    DBG_SWDIAG_LESCAN_EVT_IDX_OFFSET               = 6,
    DBG_SWDIAG_LESCAN_EVT_IDX_MASK                 = 0x40,

    DBG_SWDIAG_LESCAN_PKT_RX_OFFSET                = 7,
    DBG_SWDIAG_LESCAN_PKT_RX_MASK                  = 0x80,

    /**
     ******************************************************************************************
     * @brief BANK 30 : SCH_PROG
     ******************************************************************************************
     */
    DBG_SWDIAG_SCH_PROG                            = 30,

    DBG_SWDIAG_SCH_PROG_RXTX_ISR_OFFSET            = 0,
    DBG_SWDIAG_SCH_PROG_RXTX_ISR_MASK              = 0x01,

    DBG_SWDIAG_SCH_PROG_END_ISR_OFFSET             = 1,
    DBG_SWDIAG_SCH_PROG_END_ISR_MASK               = 0x02,

    DBG_SWDIAG_SCH_PROG_PROG_PUSH_OFFSET           = 2,
    DBG_SWDIAG_SCH_PROG_PROG_PUSH_MASK             = 0x04,

    DBG_SWDIAG_SCH_PROG_SKIP_OFFSET                = 3,
    DBG_SWDIAG_SCH_PROG_SKIP_MASK                  = 0x08,

    DBG_SWDIAG_SCH_PROG_NB_PROG_OFFSET             = 4,
    DBG_SWDIAG_SCH_PROG_NB_PROG_MASK               = 0xF0,

    /**
     ******************************************************************************************
     * @brief BANK 31 : LE INITIATING
     ******************************************************************************************
     */
    DBG_SWDIAG_LEINIT                             = 31,

    DBG_SWDIAG_LEINIT_START_OFFSET                = 0,
    DBG_SWDIAG_LEINIT_START_MASK                  = 0x01,

    DBG_SWDIAG_LEINIT_STOP_OFFSET                 = 1,
    DBG_SWDIAG_LEINIT_STOP_MASK                   = 0x02,

    DBG_SWDIAG_LEINIT_EVT_START_OFFSET            = 2,
    DBG_SWDIAG_LEINIT_EVT_START_MASK              = 0x04,

    DBG_SWDIAG_LEINIT_EVT_CANCELED_OFFSET         = 3,
    DBG_SWDIAG_LEINIT_EVT_CANCELED_MASK           = 0x08,

    DBG_SWDIAG_LEINIT_FRM_CBK_OFFSET               = 4,
    DBG_SWDIAG_LEINIT_FRM_CBK_MASK                 = 0x10,

    DBG_SWDIAG_LEINIT_FRM_EOF_ISR_OFFSET          = 5,
    DBG_SWDIAG_LEINIT_FRM_EOF_ISR_MASK            = 0x20,

    DBG_SWDIAG_LEINIT_SCHED_OFFSET                = 6,
    DBG_SWDIAG_LEINIT_SCHED_MASK                  = 0x40,

    DBG_SWDIAG_LEINIT_PKT_RX_OFFSET               = 7,
    DBG_SWDIAG_LEINIT_PKT_RX_MASK                 = 0x80,

    /**
     ******************************************************************************************
     * @brief BANK 32 : LE ADVERTISING
     ******************************************************************************************
     */
    DBG_SWDIAG_LEADV                              = 32,

    DBG_SWDIAG_LEADV_START_OFFSET                 = 0,
    DBG_SWDIAG_LEADV_START_MASK                   = 0x01,

    DBG_SWDIAG_LEADV_STOP_OFFSET                  = 1,
    DBG_SWDIAG_LEADV_STOP_MASK                    = 0x02,

    DBG_SWDIAG_LEADV_EVT_START_OFFSET             = 2,
    DBG_SWDIAG_LEADV_EVT_START_MASK               = 0x04,

    DBG_SWDIAG_LEADV_EVT_CANCELED_OFFSET          = 3,
    DBG_SWDIAG_LEADV_EVT_CANCELED_MASK            = 0x08,

    DBG_SWDIAG_LEADV_RX_ISR_OFFSET                = 4,
    DBG_SWDIAG_LEADV_RX_ISR_MASK                  = 0x10,

    DBG_SWDIAG_LEADV_FRM_ISR_OFFSET               = 5,
    DBG_SWDIAG_LEADV_FRM_ISR_MASK                 = 0x20,

    DBG_SWDIAG_LEADV_SCHED_OFFSET                 = 6,
    DBG_SWDIAG_LEADV_SCHED_MASK                   = 0x40,

    DBG_SWDIAG_LEADV_PKT_RX_OFFSET                = 7,
    DBG_SWDIAG_LEADV_PKT_RX_MASK                  = 0x80,

    /**
     ******************************************************************************************
     * @brief BANK 33 : LE CONNECTION
     ******************************************************************************************
     */
    DBG_SWDIAG_LECON                              = 33,

    DBG_SWDIAG_LECON_START_OFFSET                 = 0,
    DBG_SWDIAG_LECON_START_MASK                   = 0x01,

    DBG_SWDIAG_LECON_STOP_OFFSET                  = 1,
    DBG_SWDIAG_LECON_STOP_MASK                    = 0x02,

    DBG_SWDIAG_LECON_EVT_START_OFFSET             = 2,
    DBG_SWDIAG_LECON_EVT_START_MASK               = 0x04,

    DBG_SWDIAG_LECON_EVT_CANCELED_OFFSET          = 3,
    DBG_SWDIAG_LECON_EVT_CANCELED_MASK            = 0x08,

    DBG_SWDIAG_LECON_RX_ISR_OFFSET                = 4,
    DBG_SWDIAG_LECON_RX_ISR_MASK                  = 0x10,

    DBG_SWDIAG_LECON_FRM_ISR_OFFSET               = 5,
    DBG_SWDIAG_LECON_FRM_ISR_MASK                 = 0x20,

    DBG_SWDIAG_LECON_SCHED_OFFSET                 = 6,
    DBG_SWDIAG_LECON_SCHED_MASK                   = 0x40,

    DBG_SWDIAG_LECON_TX_ISR_OFFSET                = 7,
    DBG_SWDIAG_LECON_TX_ISR_MASK                  = 0x80,

    /**
    ******************************************************************************************
    * @brief BANK 34 : LE DATA
    ******************************************************************************************
    */
    DBG_SWDIAG_LEDATA                             = 34,

    DBG_SWDIAG_LEDATA_LLCP_TX_OFFSET              = 0,
    DBG_SWDIAG_LEDATA_LLCP_TX_MASK                = 0x01,

    DBG_SWDIAG_LEDATA_ACL_TX_OFFSET               = 1,
    DBG_SWDIAG_LEDATA_ACL_TX_MASK                 = 0x02,

    DBG_SWDIAG_LEDATA_LLCP_RX_OFFSET              = 2,
    DBG_SWDIAG_LEDATA_LLCP_RX_MASK                = 0x04,

    DBG_SWDIAG_LEDATA_ACL_RX_OFFSET               = 3,
    DBG_SWDIAG_LEDATA_ACL_RX_MASK                 = 0x08,

    DBG_SWDIAG_LEDATA_LLCP_ACK_OFFSET             = 4,
    DBG_SWDIAG_LEDATA_LLCP_ACK_MASK               = 0x10,

    DBG_SWDIAG_LEDATA_ACL_ACK_OFFSET              = 5,
    DBG_SWDIAG_LEDATA_ACL_ACK_MASK                = 0x20,

    /**
    ******************************************************************************************
    * @brief BANK 35 : LETX
    ******************************************************************************************
    */
    DBG_SWDIAG_LETX                               = 35,

    DBG_SWDIAG_LETX_LLCP_TX_OFFSET                = 0,
    DBG_SWDIAG_LETX_LLCP_TX_MASK                  = 0x01,

    DBG_SWDIAG_LETX_ACL_TX_OFFSET                 = 1,
    DBG_SWDIAG_LETX_ACL_TX_MASK                   = 0x02,

    DBG_SWDIAG_LETX_SW_IDX_OFFSET                 = 2,
    DBG_SWDIAG_LETX_SW_IDX_MASK                   = 0x04,

    DBG_SWDIAG_LETX_HW_IDX_OFFSET                 = 3,
    DBG_SWDIAG_LETX_HW_IDX_MASK                   = 0x08,

    DBG_SWDIAG_LETX_TX_CNT_OFFSET                 = 4,
    DBG_SWDIAG_LETX_TX_CNT_MASK                   = 0x30,

    /**
     ******************************************************************************************
     * @brief BANK 36 : LE PERIODIC ADVERTISING
     ******************************************************************************************
     */
    DBG_SWDIAG_LEPERADV                              = 36,

    DBG_SWDIAG_LEPERADV_START_OFFSET                 = 0,
    DBG_SWDIAG_LEPERADV_START_MASK                   = 0x01,

    DBG_SWDIAG_LEPERADV_STOP_OFFSET                  = 1,
    DBG_SWDIAG_LEPERADV_STOP_MASK                    = 0x02,

    DBG_SWDIAG_LEPERADV_EVT_START_OFFSET             = 2,
    DBG_SWDIAG_LEPERADV_EVT_START_MASK               = 0x04,

    DBG_SWDIAG_LEPERADV_EVT_CANCELED_OFFSET          = 3,
    DBG_SWDIAG_LEPERADV_EVT_CANCELED_MASK            = 0x08,

    DBG_SWDIAG_LEPERADV_RX_ISR_OFFSET                = 4,
    DBG_SWDIAG_LEPERADV_RX_ISR_MASK                  = 0x10,

    DBG_SWDIAG_LEPERADV_FRM_ISR_OFFSET               = 5,
    DBG_SWDIAG_LEPERADV_FRM_ISR_MASK                 = 0x20,

    DBG_SWDIAG_LEPERADV_SCHED_OFFSET                 = 6,
    DBG_SWDIAG_LEPERADV_SCHED_MASK                   = 0x40,

    /**
     ******************************************************************************************
     * @brief BANK 37 : LE SYNC
     ******************************************************************************************
     */
    DBG_SWDIAG_LESYNC                              = 37,

    DBG_SWDIAG_LESYNC_START_OFFSET                 = 0,
    DBG_SWDIAG_LESYNC_START_MASK                   = 0x01,

    DBG_SWDIAG_LESYNC_STOP_OFFSET                  = 1,
    DBG_SWDIAG_LESYNC_STOP_MASK                    = 0x02,

    DBG_SWDIAG_LESYNC_EVT_START_OFFSET             = 2,
    DBG_SWDIAG_LESYNC_EVT_START_MASK               = 0x04,

    DBG_SWDIAG_LESYNC_EVT_CANCELED_OFFSET          = 3,
    DBG_SWDIAG_LESYNC_EVT_CANCELED_MASK            = 0x08,

    DBG_SWDIAG_LESYNC_FRM_RX_ISR_OFFSET            = 4,
    DBG_SWDIAG_LESYNC_FRM_RX_ISR_MASK              = 0x10,

    DBG_SWDIAG_LESYNC_FRM_EOF_ISR_OFFSET           = 5,
    DBG_SWDIAG_LESYNC_FRM_EOF_ISR_MASK             = 0x20,

    DBG_SWDIAG_LESYNC_SCHED_OFFSET                 = 6,
    DBG_SWDIAG_LESYNC_SCHED_MASK                   = 0x40,

    DBG_SWDIAG_LESYNC_PKT_RX_OFFSET                = 7,
    DBG_SWDIAG_LESYNC_PKT_RX_MASK                  = 0x80,

    /**
     ******************************************************************************************
     * @brief BANK 38 : ADV REPORT FILTER
     ******************************************************************************************
     */
    DBG_SWDIAG_LEADVFILT                           = 38,

    DBG_SWDIAG_LEADVFILT_OK_OFFSET                 = 0,
    DBG_SWDIAG_LEADVFILT_OK_MASK                   = 0x01,

    DBG_SWDIAG_LEADVFILT_KO_OFFSET                 = 1,
    DBG_SWDIAG_LEADVFILT_KO_MASK                   = 0x02,

    DBG_SWDIAG_LEADVFILT_CHAIN_STAT_1M_OFFSET      = 2,
    DBG_SWDIAG_LEADVFILT_CHAIN_STAT_1M_MASK        = 0x0C,

    DBG_SWDIAG_LEADVFILT_CNT_OFFSET                = 4,
    DBG_SWDIAG_LEADVFILT_CNT_MASK                  = 0xF0,

    /**
     ******************************************************************************************
     * @brief BANK 39 : ACL_TX
     ******************************************************************************************
     */
    DBG_SWDIAG_ACL_TX                              = 39,

    DBG_SWDIAG_ACL_TX_TRAFFIC_OFFSET               = 0,
    DBG_SWDIAG_ACL_TX_TRAFFIC_MASK                 = 0xFF,

    /**
     ******************************************************************************************
     * @brief BANK 40 : ACL_RX
     ******************************************************************************************
     */
    DBG_SWDIAG_ACL_RX                              = 40,

    DBG_SWDIAG_ACL_RX_TRAFFIC_OFFSET               = 0,
    DBG_SWDIAG_ACL_RX_TRAFFIC_MASK                 = 0xFF,

    /**
     ******************************************************************************************
     * @brief BANK 41 : ISOOHCI (Audio over Application-Host-interface)
     ******************************************************************************************
     */
    DBG_SWDIAG_ISOOHCI                               = 41,

    DBG_SWDIAG_ISOOHCI_RX_ISO_OFFSET                 = 0,
    DBG_SWDIAG_ISOOHCI_RX_ISO_MASK                   = 0x01,

    DBG_SWDIAG_ISOOHCI_TX_ISO_OFFSET                 = 1,
    DBG_SWDIAG_ISOOHCI_TX_ISO_MASK                   = 0x02,

    DBG_SWDIAG_ISOOHCI_RX_TOG_OFFSET                 = 2,
    DBG_SWDIAG_ISOOHCI_RX_TOG_MASK                   = 0x04,

    DBG_SWDIAG_ISOOHCI_TX_TOG_OFFSET                 = 3,
    DBG_SWDIAG_ISOOHCI_TX_TOG_MASK                   = 0x08,

    /**
     ******************************************************************************************
     * @brief BANK 42 : BI Driver
     ******************************************************************************************
     */
    DBG_SWDIAG_BI                                    = 42,

    DBG_SWDIAG_BI_SCHED_OFFSET                       = 0,
    DBG_SWDIAG_BI_SCHED_MASK                         = 0x01,

    DBG_SWDIAG_BI_START_OFFSET                       = 1,
    DBG_SWDIAG_BI_START_MASK                         = 0x02,

    DBG_SWDIAG_BI_END_OFFSET                         = 2,
    DBG_SWDIAG_BI_END_MASK                           = 0x04,

    DBG_SWDIAG_BI_CANCEL_OFFSET                      = 3,
    DBG_SWDIAG_BI_CANCEL_MASK                        = 0x08,

    DBG_SWDIAG_BI_PROG_OFFSET                        = 4,
    DBG_SWDIAG_BI_PROG_MASK                          = 0x10,

    DBG_SWDIAG_BI_SKIP_OFFSET                        = 5,
    DBG_SWDIAG_BI_SKIP_MASK                          = 0x20,

    DBG_SWDIAG_BI_END_SCHED_OFFSET                   = 6,
    DBG_SWDIAG_BI_END_SCHED_MASK                     = 0x40,

    DBG_SWDIAG_BI_END_STREAM_OFFSET                  = 7,
    DBG_SWDIAG_BI_END_STREAM_MASK                    = 0x80,

    /**
     ******************************************************************************************
     * @brief BANK 43 : BI Data
     ******************************************************************************************
     */
    DBG_SWDIAG_BI_DATA                               = 43,

    DBG_SWDIAG_BI_DATA_PROG_OFFSET                   = 0,
    DBG_SWDIAG_BI_DATA_PROG_MASK                     = 0x01,

    DBG_SWDIAG_BI_DATA_LOAD_OFFSET                   = 1,
    DBG_SWDIAG_BI_DATA_LOAD_MASK                     = 0x02,

    /**
     ******************************************************************************************
     * @brief BANK 42 : CI Driver
     ******************************************************************************************
     */
    DBG_SWDIAG_CI                                    = 42,

    DBG_SWDIAG_CI_SCHED_OFFSET                       = 0,
    DBG_SWDIAG_CI_SCHED_MASK                         = 0x01,

    DBG_SWDIAG_CI_START_OFFSET                       = 1,
    DBG_SWDIAG_CI_START_MASK                         = 0x02,

    DBG_SWDIAG_CI_END_OFFSET                         = 2,
    DBG_SWDIAG_CI_END_MASK                           = 0x04,

    DBG_SWDIAG_CI_CANCEL_OFFSET                      = 3,
    DBG_SWDIAG_CI_CANCEL_MASK                        = 0x08,

    DBG_SWDIAG_CI_PROG_OFFSET                        = 4,
    DBG_SWDIAG_CI_PROG_MASK                          = 0x10,

    DBG_SWDIAG_CI_SKIP_OFFSET                        = 5,
    DBG_SWDIAG_CI_SKIP_MASK                          = 0x20,

    DBG_SWDIAG_CI_RX_ISO_OFFSET                      = 6,
    DBG_SWDIAG_CI_RX_ISO_MASK                        = 0x40,

    DBG_SWDIAG_CI_TX_ISO_OFFSET                      = 7,
    DBG_SWDIAG_CI_TX_ISO_MASK                        = 0x80,

    /**
     ******************************************************************************************
     * @brief BANK 43 : CI Data
     ******************************************************************************************
     */
    DBG_SWDIAG_CI_DATA                               = 43,

    DBG_SWDIAG_CI_DATA_PREPARE_TX_OFFSET             = 0,
    DBG_SWDIAG_CI_DATA_PREPARE_TX_MASK               = 0x01,

    DBG_SWDIAG_CI_DATA_PREPARE_RX_OFFSET             = 1,
    DBG_SWDIAG_CI_DATA_PREPARE_RX_MASK               = 0x02,

    DBG_SWDIAG_CI_DATA_PROG_TX_OFFSET                = 2,
    DBG_SWDIAG_CI_DATA_PROG_TX_MASK                  = 0x04,

    DBG_SWDIAG_CI_DATA_PROG_RX_OFFSET                = 3,
    DBG_SWDIAG_CI_DATA_PROG_RX_MASK                  = 0x08,

    DBG_SWDIAG_CI_DATA_COMP_SCHED_OFFSET             = 4,
    DBG_SWDIAG_CI_DATA_COMP_SCHED_MASK               = 0x10,

    DBG_SWDIAG_CI_DATA_COMP_SCHED_INT_OFFSET         = 5,
    DBG_SWDIAG_CI_DATA_COMP_SCHED_INT_MASK           = 0x20,

    DBG_SWDIAG_CI_DATA_COMP_SCHED_SEQ_OFFSET         = 6,
    DBG_SWDIAG_CI_DATA_COMP_SCHED_SEQ_MASK           = 0x40,

    /**
     ******************************************************************************************
     * @brief BANK 44 : ISO Generator Data Path
     ******************************************************************************************
     */
    DBG_SWDIAG_ISOGEN                                = 44,

    DBG_SWDIAG_ISOGEN_RX_OK_OFFSET                   = 0,
    DBG_SWDIAG_ISOGEN_RX_OK_MASK                     = 0x01,

    DBG_SWDIAG_ISOGEN_RX_NOT_GRANT_OFFSET            = 1,
    DBG_SWDIAG_ISOGEN_RX_NOT_GRANT_MASK              = 0x02,

    DBG_SWDIAG_ISOGEN_RX_CRC_ERR_OFFSET              = 2,
    DBG_SWDIAG_ISOGEN_RX_CRC_ERR_MASK                = 0x04,

    DBG_SWDIAG_ISOGEN_RX_SYNC_ERR_OFFSET             = 3,
    DBG_SWDIAG_ISOGEN_RX_SYNC_ERR_MASK               = 0x08,

    DBG_SWDIAG_ISOGEN_RX_DATA_ERR_OFFSET             = 4,
    DBG_SWDIAG_ISOGEN_RX_DATA_ERR_MASK               = 0x10,

    DBG_SWDIAG_ISOGEN_RX_EMPTY_OFFSET                = 5,
    DBG_SWDIAG_ISOGEN_RX_EMPTY_MASK                  = 0x20,

    DBG_SWDIAG_ISOGEN_TX_OK_OFFSET                   = 6,
    DBG_SWDIAG_ISOGEN_TX_OK_MASK                     = 0x40,

    DBG_SWDIAG_ISOGEN_TX_NOT_GRANT_OFFSET            = 7,
    DBG_SWDIAG_ISOGEN_TX_NOT_GRANT_MASK              = 0x80,

    /**
     ******************************************************************************************
     * @brief BANK 44 : ISO PCM - reuse same bank since no need to create a specific one
     ******************************************************************************************
     */
    DBG_SWDIAG_ISOPCM                                = 44,

    DBG_SWDIAG_ISOPCM_START_OFFSET                   = 2,
    DBG_SWDIAG_ISOPCM_START_MASK                     = 0x04,

    DBG_SWDIAG_ISOPCM_STOP_OFFSET                    = 3,
    DBG_SWDIAG_ISOPCM_STOP_MASK                      = 0x08,

    DBG_SWDIAG_ISOPCM_GET_OFFSET                     = 4,
    DBG_SWDIAG_ISOPCM_GET_MASK                       = 0x10,

    DBG_SWDIAG_ISOPCM_DONE_OFFSET                    = 5,
    DBG_SWDIAG_ISOPCM_DONE_MASK                      = 0x20,

    DBG_SWDIAG_ISOPCM_TX_OFFSET                      = 6,
    DBG_SWDIAG_ISOPCM_TX_MASK                        = 0x40,

    DBG_SWDIAG_ISOPCM_RX_OFFSET                      = 7,
    DBG_SWDIAG_ISOPCM_RX_MASK                        = 0x80,


    /**
     ******************************************************************************************
     * @brief BANK 45 : ISOAL
     ******************************************************************************************
     */
    DBG_SWDIAG_ISOAL                                 = 45,

    DBG_SWDIAG_ISOAL_SDU_TX_GET_OFFSET               = 0,
    DBG_SWDIAG_ISOAL_SDU_TX_GET_MASK                 = 0x01,

    DBG_SWDIAG_ISOAL_SDU_TX_DONE_OFFSET              = 1,
    DBG_SWDIAG_ISOAL_SDU_TX_DONE_MASK                = 0x02,

    DBG_SWDIAG_ISOAL_SDU_RX_GET_OFFSET               = 2,
    DBG_SWDIAG_ISOAL_SDU_RX_GET_MASK                 = 0x04,

    DBG_SWDIAG_ISOAL_SDU_RX_DONE_OFFSET              = 3,
    DBG_SWDIAG_ISOAL_SDU_RX_DONE_MASK                = 0x08,

    DBG_SWDIAG_ISOAL_RX_DONE_OFFSET                  = 4,
    DBG_SWDIAG_ISOAL_RX_DONE_MASK                    = 0x10,

    DBG_SWDIAG_ISOAL_TX_GET_OFFSET                   = 5,
    DBG_SWDIAG_ISOAL_TX_GET_MASK                     = 0x20,

    DBG_SWDIAG_ISOAL_DBG1_OFFSET                     = 6,
    DBG_SWDIAG_ISOAL_DBG1_MASK                       = 0x40,

    DBG_SWDIAG_ISOAL_DBG2_OFFSET                     = 7,
    DBG_SWDIAG_ISOAL_DBG2_MASK                       = 0x80,


    /**
     ******************************************************************************************
     * @brief BANK 47 : SCH ARB
     ******************************************************************************************
     */
    DBG_SWDIAG_SCH_ARB                                   = 47,

    DBG_SWDIAG_SCH_ARB_INSERT_OFFSET                     = 0,
    DBG_SWDIAG_SCH_ARB_INSERT_MASK                       = 0x01,

    DBG_SWDIAG_SCH_ARB_INSERT_ERR_OFFSET                 = 1,
    DBG_SWDIAG_SCH_ARB_INSERT_ERR_MASK                   = 0x02,

    DBG_SWDIAG_SCH_ARB_REMOVE_OFFSET                     = 2,
    DBG_SWDIAG_SCH_ARB_REMOVE_MASK                       = 0x04,

    DBG_SWDIAG_SCH_ARB_EVT_START_OFFSET                  = 3,
    DBG_SWDIAG_SCH_ARB_EVT_START_MASK                    = 0x08,

    DBG_SWDIAG_SCH_ARB_SW_ISR_OFFSET                     = 4,
    DBG_SWDIAG_SCH_ARB_SW_ISR_MASK                       = 0x10,

    DBG_SWDIAG_SCH_ARB_PROG_TIMER_OFFSET                 = 5,
    DBG_SWDIAG_SCH_ARB_PROG_TIMER_MASK                   = 0x20,

    DBG_SWDIAG_SCH_ARB_ELT_CANCEL_OFFSET                 = 6,
    DBG_SWDIAG_SCH_ARB_ELT_CANCEL_MASK                   = 0x40,

    DBG_SWDIAG_SCH_ARB_CONFLICT_OFFSET                   = 7,
    DBG_SWDIAG_SCH_ARB_CONFLICT_MASK                     = 0x80,


    /**
     ******************************************************************************************
     * @brief BANK 48 : SAM
     ******************************************************************************************
     */
    DBG_SWDIAG_SAM                                       = 48,

    DBG_SWDIAG_SAM_RMAP_OFFSET_CFG_OFFSET                = 0,
    DBG_SWDIAG_SAM_RMAP_OFFSET_CFG_MASK                  = 0x01,

    DBG_SWDIAG_SAM_CFG_OFFSET                            = 1,
    DBG_SWDIAG_SAM_CFG_MASK                              = 0x02,

    DBG_SWDIAG_SAM_LMAP_SET_OFFSET                       = 2,
    DBG_SWDIAG_SAM_LMAP_SET_MASK                         = 0x04,

    DBG_SWDIAG_SAM_RMAP_SET_OFFSET                       = 3,
    DBG_SWDIAG_SAM_RMAP_SET_MASK                         = 0x08,

    DBG_SWDIAG_SAM_RMAP_REFRESH_OFFSET                   = 4,
    DBG_SWDIAG_SAM_RMAP_REFRESH_MASK                     = 0x10,

    DBG_SWDIAG_SAM_LMAP_EN_OFFSET                        = 5,
    DBG_SWDIAG_SAM_LMAP_EN_MASK                          = 0x20,

    DBG_SWDIAG_SAM_RMAP_EN_OFFSET                        = 6,
    DBG_SWDIAG_SAM_RMAP_EN_MASK                          = 0x40,

    DBG_SWDIAG_SAM_LSUBMAP_CFG_OFFSET                    = 7,
    DBG_SWDIAG_SAM_LSUBMAP_CFG_MASK                      = 0x80,

    /**
     ******************************************************************************************
     * @brief BANK 49 : HL
     ******************************************************************************************
     */
    DBG_SWDIAG_HL                                        = 49,

    DBG_SWDIAG_HL_L2CAP_RX_OFFSET                        = 0,
    DBG_SWDIAG_HL_L2CAP_RX_MASK                          = 0x01,

    DBG_SWDIAG_HL_L2CAP_TX_OFFSET                        = 1,
    DBG_SWDIAG_HL_L2CAP_TX_MASK                          = 0x02,

    DBG_SWDIAG_HL_L2CAP_TX_DONE_OFFSET                   = 2,
    DBG_SWDIAG_HL_L2CAP_TX_DONE_MASK                     = 0x04,

    DBG_SWDIAG_HL_GATT_PROC_START_OFFSET                 = 3,
    DBG_SWDIAG_HL_GATT_PROC_START_MASK                   = 0x08,

    DBG_SWDIAG_HL_GATT_PROC_END_OFFSET                   = 4,
    DBG_SWDIAG_HL_GATT_PROC_END_MASK                     = 0x10,

    DBG_SWDIAG_HL_GATT_PROC_CONT_OFFSET                  = 5,
    DBG_SWDIAG_HL_GATT_PROC_CONT_MASK                    = 0x20,

    DBG_SWDIAG_HL_L2CAP_TX_CHK_OFFSET                    = 6,
    DBG_SWDIAG_HL_L2CAP_TX_CHK_MASK                      = 0x40,

    DBG_SWDIAG_HL_L2CAP_TX_HCI_OFFSET                    = 7,
    DBG_SWDIAG_HL_L2CAP_TX_HCI_MASK                      = 0x80,


    /**
     ******************************************************************************************
     * @brief BANK 50 : AM0
     ******************************************************************************************
     */
    DBG_SWDIAG_AM0                                       = 50,

    DBG_SWDIAG_AM0_START_OFFSET                          = 0,
    DBG_SWDIAG_AM0_START_MASK                            = 0x01,

    DBG_SWDIAG_AM0_STOP_OFFSET                           = 1,
    DBG_SWDIAG_AM0_STOP_MASK                             = 0x02,

    DBG_SWDIAG_AM0_CRYPT_OFFSET                          = 2,
    DBG_SWDIAG_AM0_CRYPT_MASK                            = 0x04,

    DBG_SWDIAG_AM0_REJECT_OFFSET                         = 3,
    DBG_SWDIAG_AM0_REJECT_MASK                           = 0x08,

    DBG_SWDIAG_AM0_EVT_START_OFFSET                      = 4,
    DBG_SWDIAG_AM0_EVT_START_MASK                        = 0x10,

    DBG_SWDIAG_AM0_EVT_CANCELED_OFFSET                   = 5,
    DBG_SWDIAG_AM0_EVT_CANCELED_MASK                     = 0x20,

    DBG_SWDIAG_AM0_TX_ISR_OFFSET                         = 6,
    DBG_SWDIAG_AM0_TX_ISR_MASK                           = 0x40,

    DBG_SWDIAG_AM0_RX_ISR_OFFSET                         = 7,
    DBG_SWDIAG_AM0_RX_ISR_MASK                           = 0x80,

    /**
     ******************************************************************************************
     * @brief BANK 51 : CSB_RX
     ******************************************************************************************
     */
    DBG_SWDIAG_CSB_RX                              = 51,

    DBG_SWDIAG_CSB_RX_FRM_ISR_OFFSET               = 0,
    DBG_SWDIAG_CSB_RX_FRM_ISR_MASK                 = 0x01,

    DBG_SWDIAG_CSB_RX_EVT_START_OFFSET             = 1,
    DBG_SWDIAG_CSB_RX_EVT_START_MASK               = 0x02,

    DBG_SWDIAG_CSB_RX_SYNC_OFFSET                  = 2,
    DBG_SWDIAG_CSB_RX_SYNC_MASK                    = 0x04,

    DBG_SWDIAG_CSB_RX_EVT_CANCELED_OFFSET          = 3,
    DBG_SWDIAG_CSB_RX_EVT_CANCELED_MASK            = 0x08,

    DBG_SWDIAG_CSB_RX_SCHED_OFFSET                 = 4,
    DBG_SWDIAG_CSB_RX_SCHED_MASK                   = 0x10,

    DBG_SWDIAG_CSB_RX_PKT_RX_OFFSET                = 5,
    DBG_SWDIAG_CSB_RX_PKT_RX_MASK                  = 0x20,

    DBG_SWDIAG_CSB_RX_START_OFFSET                 = 6,
    DBG_SWDIAG_CSB_RX_START_MASK                   = 0x40,

    DBG_SWDIAG_CSB_RX_END_OFFSET                   = 7,
    DBG_SWDIAG_CSB_RX_END_MASK                     = 0x80,

#if (EAVESDROPPING_SUPPORT)
    /**
    ******************************************************************************************
    * @brief BANK 52 : ED
    ******************************************************************************************
    */
    DBG_SWDIAG_ED                               = 52,

    DBG_SWDIAG_ED_EVT_START_OFFSET              = 0,
    DBG_SWDIAG_ED_EVT_START_MASK                = 0x01,

    DBG_SWDIAG_ED_EVT_STOP_OFFSET               = 1,
    DBG_SWDIAG_ED_EVT_STOP_MASK                 = 0x02,

    DBG_SWDIAG_ED_EVT_CANCELED_OFFSET           = 2,
    DBG_SWDIAG_ED_EVT_CANCELED_MASK             = 0x04,

    DBG_SWDIAG_ED_FRM_ISR_OFFSET                = 3,
    DBG_SWDIAG_ED_FRM_ISR_MASK                  = 0x08,

    DBG_SWDIAG_ED_CLK_ISR_OFFSET                = 4,
    DBG_SWDIAG_ED_CLK_ISR_MASK                  = 0x10,

    DBG_SWDIAG_ED_PROG_EN_OFFSET                = 5,
    DBG_SWDIAG_ED_PROG_EN_MASK                  = 0x20,

    DBG_SWDIAG_ED_SKET_ISR_OFFSET               = 6,
    DBG_SWDIAG_ED_SKET_ISR_MASK                 = 0x40,

    DBG_SWDIAG_ED_PARITY_OFFSET                 = 7,
    DBG_SWDIAG_ED_PARITY_MASK                   = 0x80,

    /**
    ******************************************************************************************
    * @brief BANK 53 : BT_INT_ED
    ******************************************************************************************
    */

    DBG_SWDIAG_BT_ISR_ED                        = 53,

    DBG_SWDIAG_BT_ISR_ED_BINACK_OFFSET          = 0,
    DBG_SWDIAG_BT_ISR_ED_BINACK_MASK            = 0x01,

    DBG_SWDIAG_BT_ISR_ED_CLKCAP_OFFSET          = 1,
    DBG_SWDIAG_BT_ISR_ED_CLKCAP_MASK            = 0x02,

    DBG_SWDIAG_BT_ISR_ED_CLKSET_OFFSET          = 2,
    DBG_SWDIAG_BT_ISR_ED_CLKSET_MASK            = 0x04,
#endif // EAVESDROPPING_SUPPORT

    /**
     ******************************************************************************************
     * @brief BANK 54 : CSB_TX
     ******************************************************************************************
     */
    DBG_SWDIAG_CSB_TX                              = 54,

    DBG_SWDIAG_CSB_TX_FRM_ISR_OFFSET               = 0,
    DBG_SWDIAG_CSB_TX_FRM_ISR_MASK                 = 0x01,

    DBG_SWDIAG_CSB_TX_EVT_START_OFFSET             = 1,
    DBG_SWDIAG_CSB_TX_EVT_START_MASK               = 0x02,

    DBG_SWDIAG_CSB_TX_AFH_UPDATE_OFFSET            = 2,
    DBG_SWDIAG_CSB_TX_AFH_UPDATE_MASK              = 0x04,

    DBG_SWDIAG_CSB_TX_EVT_CANCELED_OFFSET          = 3,
    DBG_SWDIAG_CSB_TX_EVT_CANCELED_MASK            = 0x08,

    DBG_SWDIAG_CSB_TX_SCHED_OFFSET                 = 4,
    DBG_SWDIAG_CSB_TX_SCHED_MASK                   = 0x10,

    DBG_SWDIAG_CSB_TX_SET_DATA_OFFSET              = 5,
    DBG_SWDIAG_CSB_TX_SET_DATA_MASK                = 0x20,

    DBG_SWDIAG_CSB_TX_CLR_DATA_OFFSET              = 6,
    DBG_SWDIAG_CSB_TX_CLR_DATA_MASK                = 0x40,

    DBG_SWDIAG_CSB_TX_END_OFFSET                   = 7,
    DBG_SWDIAG_CSB_TX_END_MASK                     = 0x80,

    /**
     ******************************************************************************************
     * @brief BANK 55 : SUBRATE
     ******************************************************************************************
     */
    DBG_SWDIAG_SUBRATE                              = 55,

    DBG_SWDIAG_SUBRATE_DATA_OFFSET                 = 0,
    DBG_SWDIAG_SUBRATE_DATA_MASK                   = 0x01,

    DBG_SWDIAG_SUBRATE_TX_IND_OFFSET               = 1,
    DBG_SWDIAG_SUBRATE_TX_IND_MASK                 = 0x02,

    DBG_SWDIAG_SUBRATE_RX_IND_OFFSET               = 2,
    DBG_SWDIAG_SUBRATE_RX_IND_MASK                 = 0x04,
    /*
        DBG_SWDIAG_SUBRATE_xxx_OFFSET                  = 3,
        DBG_SWDIAG_SUBRATE_xxx_MASK                    = 0x08,

        DBG_SWDIAG_SUBRATE_xxx_OFFSET                  = 4,
        DBG_SWDIAG_SUBRATE_xxx_MASK                    = 0x10,

        DBG_SWDIAG_SUBRATE_xxx_OFFSET                 = 5,
        DBG_SWDIAG_SUBRATE_xxx_MASK                     = 0x20,

        DBG_SWDIAG_SUBRATE_xxx_OFFSET                   = 6,
        DBG_SWDIAG_SUBRATE_xxx_MASK                     = 0x40,

        DBG_SWDIAG_SUBRATE_xxx_OFFSET                   = 7,
        DBG_SWDIAG_SUBRATE_xxx_MASK                     = 0x80,*/

    /**
    ******************************************************************************************
    * @brief BANK 56 : LECHSCAN
    ******************************************************************************************
    */
    DBG_SWDIAG_LECHSCAN                              = 56,

    DBG_SWDIAG_LECHSCAN_START_OFFSET                 = 0,
    DBG_SWDIAG_LECHSCAN_START_MASK                   = 0x01,

    DBG_SWDIAG_LECHSCAN_STOP_OFFSET                  = 1,
    DBG_SWDIAG_LECHSCAN_STOP_MASK                    = 0x02,

    DBG_SWDIAG_LECHSCAN_EVT_START_OFFSET             = 2,
    DBG_SWDIAG_LECHSCAN_EVT_START_MASK               = 0x04,

    DBG_SWDIAG_LECHSCAN_EVT_CANCELED_OFFSET          = 3,
    DBG_SWDIAG_LECHSCAN_EVT_CANCELED_MASK            = 0x08,

    DBG_SWDIAG_LECHSCAN_RX_ISR_OFFSET                = 4,
    DBG_SWDIAG_LECHSCAN_RX_ISR_MASK                  = 0x10,

    DBG_SWDIAG_LECHSCAN_EOF_ISR_OFFSET               = 5,
    DBG_SWDIAG_LECHSCAN_EOF_ISR_MASK                 = 0x20,

    /**
    ******************************************************************************************
    * @brief BANK 57 : I2C
    ******************************************************************************************
    */
    DBG_SWDIAG_I2C                                   = 57,

    DBG_SWDIAG_I2C_WRITE_START_OFFSET                = 0,
    DBG_SWDIAG_I2C_WRITE_START_MASK                  = 0x01,

    DBG_SWDIAG_I2C_READ_START_OFFSET                 = 1,
    DBG_SWDIAG_I2C_READ_START_MASK                   = 0x02,

    DBG_SWDIAG_I2C_WRITE_BYTES_OFFSET                = 2,
    DBG_SWDIAG_I2C_WRITE_BYTES_MASK                  = 0x04,

    DBG_SWDIAG_I2C_READ_BYTES_OFFSET                 = 3,
    DBG_SWDIAG_I2C_READ_BYTES_MASK                   = 0x08,

    DBG_SWDIAG_I2C_WRITE_ADDRESS_OFFSET              = 4,
    DBG_SWDIAG_I2C_WRITE_ADDRESS_MASK                = 0x10,

    DBG_SWDIAG_I2C_ISR_OFFSET                        = 5,
    DBG_SWDIAG_I2C_ISR_MASK                          = 0x20,

#if (EAVESDROPPING_SUPPORT)
    /**
    ******************************************************************************************
    * @brief BANK 58 : LE SEG SCAN
    ******************************************************************************************
    */
    DBG_SWDIAG_SEG_LESCAN                            = 58,

    DBG_SWDIAG_SEG_LESCAN_SCHED_OFFSET               = 0,
    DBG_SWDIAG_SEG_LESCAN_SCHED_MASK                 = 0x01,

    DBG_SWDIAG_SEG_LESCAN_SET_TIME_OFFSET            = 1,
    DBG_SWDIAG_SEG_LESCAN_SET_TIME_MASK              = 0x02,

    DBG_SWDIAG_SEG_LESCAN_START_OFFSET               = 2,
    DBG_SWDIAG_SEG_LESCAN_START_MASK                 = 0x04,

    DBG_SWDIAG_SEG_LESCAN_STOP_OFFSET                = 3,
    DBG_SWDIAG_SEG_LESCAN_STOP_MASK                  = 0x08,

    DBG_SWDIAG_SEG_LESCAN_ENABLING_OFFSET            = 4,
    DBG_SWDIAG_SEG_LESCAN_ENABLING_MASK              = 0x10,

    DBG_SWDIAG_SEG_LESCAN_RESCHED_OFFSET             = 5,
    DBG_SWDIAG_SEG_LESCAN_RESCHED_MASK               = 0x20,

    DBG_SWDIAG_SEG_LESCAN_PHY_1MBPS_OFFSET           = 6,
    DBG_SWDIAG_SEG_LESCAN_PHY_1MBPS_MASK             = 0x40,

    DBG_SWDIAG_SEG_LESCAN_PHY_CODED_OFFSET           = 7,
    DBG_SWDIAG_SEG_LESCAN_PHY_CODED_MASK             = 0x80,
#endif // (EAVESDROPPING_SUPPORT)

    /// Number of SW profiles - to increase when new bank are added
    DBG_SWDIAG_NB_PROFILES                           = 59,
};


/*
 * GLOBAL VARIABLE DECLARATION
 ****************************************************************************************
 */

/// Allocate HW banks to SW profiles
extern uint8_t sw_to_hw[DBG_SWDIAG_NB_PROFILES];


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
******************************************************************************************
* @brief Initialize SW profiling module
******************************************************************************************
*/
void dbg_swdiag_init(void);

/**
******************************************************************************************
* @brief Read the SW profiling configuration.
*
* @return SW profile configuration
******************************************************************************************
*/
uint32_t dbg_swdiag_read(void);

/**
******************************************************************************************
* @brief Write the SW profiling configuration.
*
* @param[in] profile  SW profiling configuration to apply
******************************************************************************************
*/
void dbg_swdiag_write(uint32_t profile);

#endif //RW_SWDIAG

/// @} DBGSWDIAG

#endif // DBG_SWDIAG_H_
