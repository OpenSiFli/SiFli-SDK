/**
****************************************************************************************
*
* @file rwip.h
*
* @brief RW IP SW main module
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/
#ifndef _RWIP_H_
#define _RWIP_H_

/**
 ****************************************************************************************
 * @addtogroup ROOT
 * @brief Entry points of the RW IP stacks/modules
 *
 * This module contains the primitives that allow an application accessing and running the
 * RW IP protocol stacks / modules.
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"          // stack configuration

#include <stdint.h>               // standard integer definitions
#include <stdbool.h>              // standard boolean definitions

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    #include "co_bt_defines.h"        // Bluetooth defines
#endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)


/*
 * DEFINES
 ****************************************************************************************
 */
/// Maximum value of a Bluetooth timestamp (in us)
#define RWIP_MAX_BTS_TIME                (0xFFFFFFFF)
/// Maximum value of a Bluetooth clock (in 312.5us half slots)
#define RWIP_MAX_CLOCK_TIME              ((1L<<28) - 1)
/// Maximum value of a 10ms Bluetooth clock
#define RWIP_MAX_10MS_TIME               ((1L<<23) - 1)
/// Invalid target time
#define RWIP_INVALID_TARGET_TIME         (0xFFFFFFFFL)
/// timer margin to ensure proper programming in microseconds
#define RWIP_BTS_TIMER_MARGIN            (50)
/// timer margin to ensure proper programming in half microseconds
#define RWIP_TIMER_MARGIN_HUS            (100)
/// Maximum additional random increment value (range 0-3)
#define RWIP_PRIO_MAX_RAND_INC           (4)

/**
* Inverse an intra-half-slot value (in half-us), from/to following formats:
*   - A: elapsed time from the previous half-slot (in half-us)
*   - B: remaining time to the next half-slot (in half-us)
* The function from A to B or from B to A.
*  ____________________________________________________________________________________________________
*     Half-slot N-1            |             Half-slot N              |             Half-slot N+1
*  ____________________________|______________________________________|________________________________
*                              |<---------- A ---------->|<---- B --->|
*  ____________________________|______________________________________|________________________________
*/
#define HALF_SLOT_INV(x)  (HALF_SLOT_SIZE - x - 1)


/// result of sleep state.
enum rwip_sleep_state
{
    /// Some activity pending, can not enter in sleep state
    RWIP_ACTIVE    = 0,
    /// CPU can be put in sleep state
    RWIP_CPU_SLEEP,
    /// IP could enter in deep sleep
    RWIP_DEEP_SLEEP,
};


/// Definition of the bits preventing the system from sleeping
enum prevent_sleep
{
    /// Flag indicating that the wake up process is ongoing
    RW_WAKE_UP_ONGOING                 = 0x000001,
    /// Flag indicating the IP is in sleep, to avoid running sleep algorithm while already entering sleep
    RW_DEEP_SLEEP                      = 0x000002,
    /// Flag indicating that an encryption is ongoing
    RW_CRYPT_ONGOING                   = 0x000004,
    /// Flag indicating that controller shall not sleep due to not CSB LPO_Allowed
    RW_CSB_NOT_LPO_ALLOWED             = 0x000008,
    /// Flag indicating the MWS/WLAN Event Generator is in operation
    RW_MWS_WLAN_EVENT_GENERATOR_ACTIVE = 0x000010,
    /// Flag to indicate that platform does not support deep sleep
    RW_PLF_DEEP_SLEEP_DISABLED         = 0x000020,
    /// Flag to indicate that a baseband frame is ongoing
    RW_BB_FRAME_ONGOING                = 0x000040,
    /// Flag to indicate that BLE Hopping computation on-going
    RW_HOP_CALC_ONGOING                = 0x000080,
    /// Flag to indicate that BT is in active mode (ACL, SCO)
    RW_BT_ACTIVE_MODE                  = 0x000100,
    /// Flag to indicate that BLE requires active mode
    RW_BLE_ACTIVE_MODE                 = 0x000200,
    /// Flag indicating that an TX transfer is ongoing on Transport Layer - First Interface
    RW_TL_1_TX_ONGOING                 = 0x000400,
    /// Flag indicating that an TX transfer is ongoing on Transport Layer - Second Interface
    RW_TL_2_TX_ONGOING                 = 0x000800,
    /// Flag indicating that an RX transfer is ongoing on Transport Layer - First Interface
    RW_TL_1_RX_ONGOING                 = 0x001000,
    /// Flag indicating that an RX transfer is ongoing on Transport Layer - Second Interface
    RW_TL_2_RX_ONGOING                 = 0x002000,
#if EAVESDROPPING_SUPPORT
    /// Flag indicating that a clock capture operation is ongoing
    RW_CLK_CAPTURE_ONGOING             = 0x004000,
    /// Flag indicating that a clock set operation is ongoing
    RW_CLK_SET_ONGOING                 = 0x008000,
    /// Flag indicating that eavesdropping is ongoing
    RW_EAVESDROPPING_ONGOING           = 0x010000,
    /// Flag indicating that the host is preventing deep sleep (e.g. for coexistence reasons)
    RW_HOST_PREVENTING                 = 0x020000,
#endif // EAVESDROPPING_SUPPORT
};

/// Parameters - Possible Returned Status
enum PARAM_STATUS
{
    /// PARAM status OK
    PARAM_OK,
    /// generic PARAM status KO
    PARAM_FAIL,
    /// PARAM ID unrecognized
    PARAM_ID_NOT_DEFINED,
    /// No space for PARAM
    PARAM_NO_SPACE_AVAILABLE,
    /// Length violation
    PARAM_LENGTH_OUT_OF_RANGE,
    /// PARAM parameter locked
    PARAM_PARAM_LOCKED,
    /// PARAM corrupted
    PARAM_CORRUPT
};

/**
 * External interface type types.
 */
enum rwip_eif_types
{
    /// Host Controller Interface - Controller part
    RWIP_EIF_HCIC,

    /// Host Controller Interface - Host part
    RWIP_EIF_HCIH,

    /// Application Host interface
    RWIP_EIF_AHI,
};


/// Enumeration of External Interface status codes
enum rwip_eif_status
{
    /// EIF status OK
    RWIP_EIF_STATUS_OK,
    /// EIF status KO
    RWIP_EIF_STATUS_ERROR,

#if (BLE_EMB_PRESENT == 0)
    /// External interface detached
    RWIP_EIF_STATUS_DETACHED,
    /// External interface attached
    RWIP_EIF_STATUS_ATTACHED,
#endif // (BLE_EMB_PRESENT == 0)
};

/// Enumeration of RF modulations
enum rwip_rf_mod
{
    MOD_GFSK  = 0x01,
    MOD_DQPSK = 0x02,
    MOD_8DPSK = 0x03,
};

/// Enumeration of txpwr_cs_get search options
enum rwip_txpwr_cs_get_opt
{
    TXPWR_CS_LOWER,
    TXPWR_CS_HIGHER,
    TXPWR_CS_NEAREST,
};

#if RW_DEBUG
/// Assert type
/*@TRACE*/
enum assert_type
{
    ASSERT_TYPE_WARNING = 0,
    ASSERT_TYPE_ERROR   = 1,
};
#endif //RW_DEBUG

/// Rx Status
/*@TRACE*/
enum rwip_rx_status
{
    RWIP_RX_OK,
    RWIP_RX_NO_SYNC,
    RWIP_RX_CRC_ERROR,
    RWIP_RX_OTHER_ERROR,
};

enum rwip_rf_modtype
{
    MOD_POLAR,
    MOD_IQ,
};

enum rwip_rf_type
{
    TYPE_BLE,
    TYPE_BT,
};


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Time information
/*@TRACE*/
typedef struct
{
    /// Integer part of the time (in half-slot)
    uint32_t hs;
    /// Fractional part of the time (in half-us) (range: 0-624)
    uint16_t hus;
    /// Bluetooth timestamp value (in us) 32 bits counter
    uint32_t bts;
} rwip_time_t;

/// API functions of the RF driver that are used by the BLE or BT software
struct rwip_rf_api
{
    /// Function called upon HCI reset command reception
    void (*reset)(void);
    /// Function called to enable/disable force AGC mechanism (true: en / false : dis)
    void (*force_agc_enable)(bool);
    /// Function called when TX power has to be decreased for a specific link id
    bool (*txpwr_dec)(uint8_t);
    /// Function called when TX power has to be increased for a specific link id
    bool (*txpwr_inc)(uint8_t);
    /// Function called when TX power has to be set to max for a specific link id
    void (*txpwr_max_set)(uint8_t);
    /// Function called to convert a TX power CS power field into the corresponding value in dBm
    int8_t (*txpwr_dbm_get)(uint8_t, uint8_t, uint8_t);//cs_val  GFSK/PSK   iq/polar
    /// Function called to convert a power in dBm into a control structure tx power field
    uint8_t (*txpwr_cs_get)(int8_t, uint8_t, uint8_t);//pwr higher/lower iq/polar
    /// Function called to get IQ/POLAR modulate type
    uint8_t (*txpwr_mdlt_get)(int8_t, uint8_t);//pwr  GFSK/PSK  pwr:max  initvalue
    //uint8_t (*txpwr_set)(int8_t, uint8_t, uint8_t, uint8_t);//bt/ble  cs_index  pwr  higher/lower
    uint8_t (*txpwr_cs_set)(int8_t, uint8_t, uint8_t, uint8_t);//bt/ble  cs_index  iq/polar  cs_value
    uint8_t (*txpwr_cs_update)(int8_t, uint8_t, uint8_t);//bt/ble  cs_index et_index
    /// Function called to convert the RSSI read from the control structure into a real RSSI
    int8_t (*rssi_convert)(uint8_t);
    /// Function used to read a RF register
    uint32_t (*reg_rd)(uint32_t);
    /// Function used to write a RF register
    void (*reg_wr)(uint32_t, uint32_t);
    /// Function called to put the RF in deep sleep mode
    void (*sleep)(void);
    /// Index of minimum TX power
    uint8_t txpwr_min;
    /// Index of maximum TX power
    uint8_t txpwr_max;
    /// rf mod of minimum TX power
    uint8_t txpwr_min_mod;
    /// rf mod of maximum TX power
    uint8_t txpwr_max_mod;
    /// RSSI high threshold ('real' signed value in dBm)
    int8_t rssi_high_thr;
    /// RSSI low threshold ('real' signed value in dBm)
    int8_t rssi_low_thr;
    /// RF wakeup delay (in slots)
    uint8_t wakeup_delay;
};

/// API functions of the parameters that are used by the BLE or BT software
struct rwip_param_api
{
    /**
     * Get a parameter value
     * @param[in]      param_id     Parameter identifier
     * @param[in/out]  lengthPtr    Pointer to the length of the parameter (input: contain max length, output contain the effective param length, in bytes)
     * @param[out]     buf          Pointer to the buffer be filled with the parameter value
     * @return  status              0: success | >0 : error
     */
    uint8_t (*get)(uint8_t param_id, uint8_t *lengthPtr, uint8_t *buf);

    /**
     * Set a parameter value
     * @param[in]      param_id     Parameter identifier
     * @param[in/out]  length       Length of the parameter (in bytes)
     * @param[out]     buf          Pointer to the buffer containing the parameter value
     * @return  status              0: success | >0 : error
     */
    uint8_t (*set)(uint8_t param_id, uint8_t length, uint8_t *buf);

    /**
     * Delete a parameter
     * @param[in]      param_id     Parameter identifier
     * @return  status              0: success | >0 : error
     */
    uint8_t (*del)(uint8_t param_id);
};

/// Internal API for priority
struct rwip_prio
{
    ///value
    uint8_t value;
    ///Increment
    uint8_t increment;
};
/**
 ****************************************************************************************
 * @brief Function called when packet transmission/reception is finished.

 * @param[in]  dummy  Dummy data pointer returned to callback when operation is over.
 * @param[in]  status Ok if action correctly performed, else reason status code.
 *****************************************************************************************
 */
typedef void (*rwip_eif_callback)(void *, uint8_t);

/**
 * Transport layer communication interface.
 */
struct bt_eif_api
{
    /**
     *************************************************************************************
     * @brief Starts a data reception.
     *
     * @param[out] bufptr      Pointer to the RX buffer
     * @param[in]  size        Size of the expected reception
     * @param[in]  callback    Pointer to the function called back when transfer finished
     * @param[in]  dummy       Dummy data pointer returned to callback when reception is finished
     *************************************************************************************
     */
    void (*read)(uint8_t *bufptr, uint32_t size, rwip_eif_callback callback, void *dummy);

    /**
     *************************************************************************************
     * @brief Starts a data transmission.
     *
     * @param[in]  bufptr      Pointer to the TX buffer
     * @param[in]  size        Size of the transmission
     * @param[in]  callback    Pointer to the function called back when transfer finished
     * @param[in]  dummy       Dummy data pointer returned to callback when transmission is finished
     *************************************************************************************
     */
    void (*write)(uint8_t *bufptr, uint32_t size, rwip_eif_callback callback, void *dummy);

    /**
     *************************************************************************************
     * @brief Enable Interface flow.
     *************************************************************************************
     */
    void (*flow_on)(void);

    /**
     *************************************************************************************
     * @brief Disable Interface flow.
     *
     * @return True if flow has been disabled, False else.
     *************************************************************************************
     */
    bool (*flow_off)(void);
};

#if (EMB_PRESENT)
/// BLE channel assessment data
struct rwip_ch_assess_data
{
    /// Timestamp of last received packet for each frequency
    uint32_t timestamp[AFH_NB_CHANNEL_MAX];

    /// Channel quality level for each frequency
    int8_t level[AFH_NB_CHANNEL_MAX];
};
#endif //BT_EMB_PRESENT

/*
 * VARIABLE DECLARATION
*****************************************************************************************
 */

/// API for RF driver
extern struct rwip_rf_api rwip_rf;
/// API for parameters
extern struct rwip_param_api rwip_param;
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    /// API for dual mode priority
    extern const struct rwip_prio rwip_priority[RWIP_PRIO_IDX_MAX];
    #if (RW_WLAN_COEX || RW_MWS_COEX)
        /// API for COEX configuration
        extern const uint8_t rwip_coex_cfg[RWIP_COEX_CFG_MAX];
    #endif //(RW_WLAN_COEX || RW_MWS_COEX)
    /// Programming delay, margin for programming the baseband in advance of each activity (in half-slots)
    extern uint8_t rwip_prog_delay;
#endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

typedef void (*env_init_handler)(void);
typedef void (*rwip_init_handler)(uint32_t error);
typedef void (*rwip_reset_handler)(void);
typedef uint8_t (*rwip_sleep_handler)(void);
typedef const struct bt_eif_api *(*rwip_eif_get_handler)(uint8_t idx);

extern env_init_handler _env_init_handler;
extern rwip_init_handler _rwip_init_handler;
extern rwip_reset_handler _rwip_reset_handler;
extern rwip_sleep_handler _rwip_sleep_handler;

#define env_init() _env_init_handler()
#define rwip_init(error) _rwip_init_handler(error)
#define rwip_reset() _rwip_reset_handler()
#define rwip_sleep() _rwip_sleep_handler()
#define rwip_eif_get(idx) (*(rwip_eif_get_handler*)rw_os_get_if)(idx)


/*
 * MACROS
 ****************************************************************************************
 */

/// Get Event status flag
#if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
#define RWIP_PRIO_INC(prio_idx) (rwip_priority[prio_idx].increment + CO_MOD(co_rand_byte(), RWIP_PRIO_MAX_RAND_INC))
/// Adds two RWIP priority values, limit at maximum
#define RWIP_PRIO_ADD_2(prio, inc)      co_min(RWIP_PRIO_MAX, (uint16_t)prio + inc)
#if (RW_WLAN_COEX || RW_MWS_COEX)
#define RWIP_COEX_GET(coex_cfg_idx, bit_field) \
                (uint8_t)(((rwip_coex_cfg[RWIP_COEX_ ## coex_cfg_idx ##_IDX]) >> RWIP_ ## bit_field ## _POS ) & RWIP_COEX_BIT_MASK)
#else //!(RW_WLAN_COEX || RW_MWS_COEX)
#define RWIP_COEX_GET(coex_cfg_idx, bit_field) 0
#endif //(RW_WLAN_COEX || RW_MWS_COEX)
#endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

/*
 * FUNCTION DECLARATION
*****************************************************************************************
 */

void _env_init(void);

/**
 ****************************************************************************************
 * @brief Initializes the RW BT SW.
 *
 ****************************************************************************************
 */
void _rwip_init(uint32_t error);

/**
 ****************************************************************************************
 * @brief Reset the RW BT SW.
 *
 ****************************************************************************************
 */
void _rwip_reset(void);



#if (BT_EMB_PRESENT)

    #if PCA_SUPPORT
        /**
        ****************************************************************************************
        * @brief Check if clock dragging limitation
        *
        * @return    true if clock dragging must be used
        ****************************************************************************************
        */
        bool rwip_pca_clock_dragging_only(void);
    #endif //PCA_SUPPORT
#endif // (BT_EMB_PRESENT)

#if (BT_EMB_PRESENT || BLE_EMB_PRESENT)
    #if (RW_MWS_COEX)
        /**
        ****************************************************************************************
        * @brief Enable/Disable the MWS coexistence interface.
        *
        * @param[in]   en     Coexistence enable setting
        *
        ****************************************************************************************
        */
        void rwip_mwscoex_set(bool en);
    #endif //RW_MWS_COEX

    #if (RW_WLAN_COEX)
        /**
        ****************************************************************************************
        * @brief Enable/Disable the Wireless LAN coexistence interface.
        *
        * @param[in]   en     Coexistence enable setting
        *
        ****************************************************************************************
        */
        void rwip_wlcoex_set(bool en);
    #endif //RW_WLAN_COEX
#endif //(BT_EMB_PRESENT || BLE_EMB_PRESENT)

/**
 ****************************************************************************************
 * @brief Function to implement in platform in order to retrieve each external interface API
 *
 * @param[in] idx External interface index
 *
 * @return External interface api structure
 ****************************************************************************************
 */
extern const struct bt_eif_api *_rwip_eif_get(uint8_t idx);

#if RW_DEBUG
    /**
    ****************************************************************************************
    * @brief Raises an assertion message to the control interface (if present)
    *
    * @param[in] file    File name
    * @param[in] line    Line number
    * @param[in] param0  Parameter 0 (custom value given by the assert instruction)
    * @param[in] param1  Parameter 1 (custom value given by the assert instruction)
    * @param[in] type    0: warning / 1: error
    ****************************************************************************************
    */
    void rwip_assert(const char *file, int line, int param0, int param1, uint8_t type);
#endif //RW_DEBUG

#if (BLE_ISO_PRESENT)
    /**
    ****************************************************************************************
    * @brief Request a transfer of ISO data between SDU and PDU buffers
    *
    * @param[in] p_dst_addr    Destination address
    * @param[in] p_src_addr    Source address
    * @param[in] size          Size of the transfer (in octets)
    ****************************************************************************************
    */
    void rwip_iso_data_transfer(void *p_dst_addr, const void *p_src_addr, uint16_t size);
#endif // (BLE_ISO_PRESENT)


/* **************************************************************************************
 * Driver functions
 * **************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Retrieved sampled time
 *
 * @note Shall be called within a critical section
 *
 * @return current time sampled (@see rwip_time_t)
 ****************************************************************************************
 */
rwip_time_t rwip_time_get(void);


#if (BT_EMB_PRESENT)
    /**
    ****************************************************************************************
    * @brief Set current time
    *
    * @param clock value in half-slots
    ****************************************************************************************
    */
    void rwip_time_set(uint32_t clock);

    /**
    ****************************************************************************************
    * @brief Adjust current time
    *
    * @param adjustment value in us
    ****************************************************************************************
    */
    void rwip_time_adj(int16_t clk_adj_us);
#endif // (BT_EMB_PRESENT)

/**
 ****************************************************************************************
 * @brief Set the common target timer
 *
 * @note if target is RWIP_INVALID_TARGET_TIME, no timer is programmed
 *
 * @param[in] target_bts       1us Timer target value (in microseconds)
 ****************************************************************************************
 */
void rwip_timer_co_set(uint32_t target_bts);

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    /**
    ****************************************************************************************
    * @brief Set the an alarm target timer
    *
    * @note if target is RWIP_INVALID_TARGET_TIME, no timer is programmed
    *
    * @param[in] target         Half Slot Timer target value
    * @param[in] half_us_delay  Half us timer delay in corresponding half slot (range [0:624])
    ****************************************************************************************
    */
    void rwip_timer_alarm_set(uint32_t target, uint32_t half_us_delay);

    /**
    ****************************************************************************************
    * @brief Set the an Arbiter target timer
    *
    * @note if target is RWIP_INVALID_TARGET_TIME, no timer is programmed
    *
    * @param[in] target         Half Slot Timer target value
    * @param[in] half_us_delay  Half us timer delay in corresponding half slot (range [0:624])
    ****************************************************************************************
    */
    void rwip_timer_arb_set(uint32_t target, uint32_t half_us_delay);
#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

/**
 ****************************************************************************************
 * @brief Start AES encryption
 *
 * The exchange memory must be filled before calling this function.
 * This function expect to be called from a BLE Module
 *
 * @param[in] key           AES Encryption key must be 16 bytes
 * @param[in] val           16 bytes value array to encrypt using AES
 ****************************************************************************************
 */
void rwip_aes(const uint8_t *key, const uint8_t *val, bool cipher);

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    /**
    ****************************************************************************************
    * @brief Request a Software interrupt to be triggered
    ****************************************************************************************
    */
    void rwip_sw_int_req(void);
#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

/**
 ****************************************************************************************
 * @brief Invoke the sleep function.
 *
 * @return  sleep status (@see enum rwip_sleep_state)
 ****************************************************************************************
 */
uint8_t _rwip_sleep(void);

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    #if !RW_DEBUG
        /**
        ****************************************************************************************
        * @brief Report HW errors to Host.
        *
        * @param[in] error_base        Base opcode for HW block (BLE/BT/BTDM)
        * @param[in] error_type_stat   Error Type status reported by HW
        ****************************************************************************************
        */
        void rwip_hw_error_report(uint8_t error_base, uint32_t error_type_stat);
    #endif // !RW_DEBUG
    /**
    ****************************************************************************************
    * @brief Handle the common core interrupts.
    ****************************************************************************************
    */
    void dm_isr(void);
#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

#if (BLE_EMB_PRESENT && BT_EMB_PRESENT)
    /**
    ****************************************************************************************
    * @brief Handle the btdm-specific core interrupts.
    ****************************************************************************************
    */
    void rwip_btdm_isr(void);
#endif // (BLE_EMB_PRESENT && BT_EMB_PRESENT)

/**
****************************************************************************************
* @brief Set a bit in the prevent sleep bit field, in order to prevent the system from
*        going to sleep
*
* @param[in] prv_slp_bit   Bit to be set in the prevent sleep bit field
****************************************************************************************
*/
void rwip_prevent_sleep_set(uint32_t prv_slp_bit);

/**
****************************************************************************************
* @brief Clears a bit in the prevent sleep bit field, in order to allow the system
*        going to sleep
*
* @param[in] prv_slp_bit   Bit to be cleared in the prevent sleep bit field
****************************************************************************************
*/
void rwip_prevent_sleep_clear(uint32_t prv_slp_bit);


bool rwip_wakeup_check(void);

uint16_t rwip_prevent_sleep_check(void);
#if (EAVESDROPPING_SUPPORT)
    /**
    ****************************************************************************************
    * @brief return how many times deep sleep was entered.
    *
    * @return  the number of times the Bluetooth system entered deep sleep
    ****************************************************************************************
    */
    uint32_t rwip_get_deep_sleep_counter(void);
#endif // EAVESDROPPING_SUPPORT

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    /**
    ****************************************************************************************
    * @brief Get the current clock drift of the system
    *
    * The drift depends on the system activity.
    * Note:
    *  - on dual-mode device the drift cannot exceed 250ppm
    *  - on BLE device the drift cannot exceed 500ppm
    *
    * @return  Current clock drift (in ppm)
    ****************************************************************************************
    */
    uint16_t rwip_current_drift_get(void);

    /**
    ****************************************************************************************
    * @brief Get the maximum clock drift of the system
    *
    * @return  Maximum clock drift (in ppm)
    ****************************************************************************************
    */
    uint16_t rwip_max_drift_get(void);

    /**
    ****************************************************************************************
    * @brief Get sleep clock accuracy of the system
    *
    * @return     sca     Sleep clock accuracy, physical low power clock accuracy (@see enum SCA)
    ****************************************************************************************
    */
    uint8_t rwip_sca_get(void);
#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

/**
 ****************************************************************************************
 * @brief Schedule all pending events.
 *
 ****************************************************************************************
 */
void rwip_schedule(void);

/**
 ****************************************************************************************
 * @brief Convert Bluetooth time provided in parameters to a Bluetooth timestamp value
 *
 * @param[in] hs   Bluetooth time in half slot
 * @param[in] hus  Bluetooth time in half microseconds
 *
 * @return Bluetooth timestamp in microseconds
 ****************************************************************************************
 */
uint32_t rwip_bt_time_to_bts(uint32_t hs, uint16_t hus);

/**
 ****************************************************************************************
 * @brief Convert Bluetooth timestamp value to Bluetooth time
 *
 * @param[in]  bts    Bluetooth timestamp
 * @param[out] p_hs   Bluetooth time in half slot
 * @param[out] p_hus  Bluetooth time in half microseconds
 ****************************************************************************************
 */
void rwip_bts_to_bt_time(uint32_t bts, uint32_t *p_hs, uint16_t *p_hus);

#if (BLE_EMB_PRESENT)
    /**
    ****************************************************************************************
    * @brief Assess a BLE RF channel
    *
    * @param[in]  channel       Channel index (range:[0..37])
    * @param[in]  rx_status     (see #lld_rx_status enumeration)
    * @param[in]  rssi_cs       RSSI read from the CS
    * @param[in]  timestamp     Timestamp of the assessment
    * @param[in]  connected     True: expecting to receive a packet (eg. RX connection) |
    *                           False: not expecting to receive a packet (eg. Channel Scan)
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    void rwip_channel_assess_ble(uint8_t channel, uint8_t rx_status, uint8_t rssi_cs, uint32_t timestamp, bool connected);
#endif //BLE_EMB_PRESENT

#if (EMB_PRESENT)
    /**
    ****************************************************************************************
    * @brief Get channel assessment
    *
    * @return  Pointer to channel assessment data
    ****************************************************************************************
    */
    struct rwip_ch_assess_data *rwip_ch_assess_data_get(void);

    /**
    ****************************************************************************************
    * @brief Insert status of a channel
    *
    * @param[in]  channel       Channel index (range:[0..79])
    * @param[in]  rx_status     (see #lld_rx_status enumeration)
    * @param[in]  rssi_cs       RSSI read from the CS
    * @param[in]  timestamp     Timestamp of the assessment (in half-slots)
    * @param[in]  connected     True: expecting to receive a packet (eg. RX connection) |
    *                           False: not expecting to receive a packet (eg. Channel Scan)
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    void rwip_channel_stat_insert(uint8_t channel, uint8_t rx_status, uint8_t rssi_cs, uint32_t timestamp, bool connected);
#endif //BLE_EMB_PRESENT

#if (BT_EMB_PRESENT)
    /**
    ****************************************************************************************
    * @brief Assess a BT RF channel
    *
    * @param[in]  channel       Channel index
    * @param[in]  rx_ok         True: reception is correct | False: reception is corrupted
    * @param[in]  rssi_cs       RSSI read from the CS
    * @param[in]  timestamp     Timestamp of the assessment (in half-slots)
    *
    * @return status            0: success | 1-255: error
    ****************************************************************************************
    */
    void rwip_channel_assess_bt(uint8_t channel, uint8_t rx_status, uint8_t rssi_cs, uint32_t timestamp);

    /**
    ****************************************************************************************
    * @brief Get the BT local channel classification based channel assessment
    *
    * @param[out]  ch_class      Pointer to BT channel classification
    ****************************************************************************************
    */
    void rwip_ch_class_bt_get(struct bt_ch_class *ch_class);

    /**
    ****************************************************************************************
    * @brief Get the BT local channel map based channel assessment
    *
    * @param[out]  ch_map        Pointer to BT channel map
    ****************************************************************************************
    */
    void rwip_ch_map_bt_get(struct bt_ch_map *ch_map);

    /**
    ****************************************************************************************
    * @brief Re-Introduce some channels in the BT map based on the Host classification
    *
    * @param[in]  nb_ch_to_add  Number of channels to introduce on the channel map
    * @param[in]  host_ch_class Pointer to Host channel classification
    * @param[out] ch_map        Pointer to BT channel map
    ****************************************************************************************
    */
    void rwip_activate_channels_ch_map_bt(uint8_t nb_ch_to_add, struct bt_ch_map *host_ch_class, struct bt_ch_map *ch_map);

#endif //BT_EMB_PRESENT

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    /**
    ****************************************************************************************
    * @brief  enable/disable channel assessment scheme
    *
    * @param[in] en              true if enabled, false otherwise
    *****************************************************************************************
    */
    void rwip_ch_ass_en_set(bool ch_ass_en);

    /**
    ****************************************************************************************
    * @brief  returns status of channel assessment scheme (enabled or disabled)
    *****************************************************************************************
    */
    bool rwip_ch_ass_en_get(void);

    /**
    ****************************************************************************************
    * @brief Get the LE local channel classification based channel assessment
    *
    * @param[out]  ch_class      Pointer to LE channel classification
    ****************************************************************************************
    */
    void rwip_ch_class_ble_get(struct le_ch_class *ch_class);

    /**
    ****************************************************************************************
    * @brief Get the LE local channel map based channel assessment
    *
    * @param[out]  ch_map        Pointer to LE channel map
    ****************************************************************************************
    */
    void rwip_ch_map_ble_get(struct le_ch_map *ch_map);

    /**
    ****************************************************************************************
    * @brief Re-Introduce some channels in the LE map based on the Host classification
    *
    * @param[in]  nb_ch_to_add  Number of channels to introduce on the channel map
    * @param[in]  host_ch_class Pointer to Host channel classification
    * @param[out] ch_map        Pointer to LE channel map
    ****************************************************************************************
    */
    void rwip_ch_map_ble_fill(uint8_t nb_ch_to_add, struct le_ch_map *ch_map, struct le_ch_map *host_ch_class);

#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

#if ((EMB_PRESENT) || (HOST_PRESENT))
    void common_task_init(bool is_reset);
#endif

///@} ROOT

#endif // _RWIP_H_
