/**
  ******************************************************************************
  * @file   bf0_ble_common.h
  * @author Sifli software development team
  * @brief Header file - Bluetooth common interface.
 *
  ******************************************************************************
*/
/*
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __BF0_BLE_COMMON_H
#define __BF0_BLE_COMMON_H

/*
 * INCLUDE FILES
 ****************************************************************************************
    */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#ifdef BSP_USING_PC_SIMULATOR
    #include <stdint.h>
#endif


/**
 ****************************************************************************************
 * @addtogroup ble_common BLE common interface
 * @ingroup sibles
 * @brief BLE Common interface
 * @{
 */

/*
 * DEFINES
 ****************************************************************************************
 */

/**
 *@brief Macro that define BD Address length(values in bytes).
 * |      3B        |  1B |    2B    |
 * |      LAP       | UAP |    NAP   |
 */
#define BD_ADDR_LEN         6

/**
 * @brief Macro that declare variable in a specified section.
 */

#ifndef BSP_USING_PC_SIMULATOR
#define SPACE1(x)                  __attribute__((section(x)))
#else
#define SPACE1(x)                  __pragma(section(x, read)) \
                                        __declspec(allocate(x))
#define __PACKED_STRUCT            __pragma(pack(push, 1)) struct __pragma(pack(pop))
#define __INLINE                   inline
#define __WEAK
#endif

#undef BT_ASSERT
#define BT_ASSERT(expr) RT_ASSERT(expr)

#define BT_OOM_ASSERT(expr) RT_ASSERT(expr)

/**
 * @brief Register function proto type of #BLE_EVENT_REGISTER.
 */
typedef int (*register_func_t)(uint16_t event_id, uint8_t *data, uint16_t len, uint32_t context);


typedef int (*register_bt_func_t)(uint16_t type, uint16_t event_id, uint8_t *data, uint32_t context);


/**
 * @brief .Structure of store register information.
 */
typedef struct
{
    register_func_t function;
    uint32_t        context_p;
} ble_register_callback_t;


typedef struct
{
    register_bt_func_t function;
    uint32_t        context_p;
} bt_register_callback_t;


/**
 * @brief Macro of sifli event register.
 * @param[in] func register function.
 * @param[in] context user data.
 * @param[in] n priority.
 */
#ifndef BSP_USING_PC_SIMULATOR
#define SIFLI_BLE_REGISTER(func, context, n) \
        RT_USED const ble_register_callback_t __ble_callback_##func SPACE1(".sifli_reg."n) = \
        { \
            .function = func, \
            .context_p = (uint32_t)context, \
        }
#else
#define SIFLI_BLE_REGISTER(func, context, n) \
        SPACE1(n) \
        RT_USED const ble_register_callback_t __ble_callback_##func = \
        { \
            .function = func, \
            .context_p = (uint32_t)context, \
        }

#endif


#ifndef BSP_USING_PC_SIMULATOR
#define SIFLI_BT_REGISTER(func, context, n) \
        RT_USED const bt_register_callback_t __bt_callback_##func SPACE1(".bt_sifli_reg."n) = \
        { \
            .function = func, \
            .context_p = (uint32_t)context, \
        }
#else
#define SIFLI_BT_REGISTER(func, context, n) \
        SPACE1(n) \
        RT_USED const bt_register_callback_t __bt_callback_##func = \
        { \
            .function = func, \
            .context_p = (uint32_t)context, \
        }

#endif


/**
 * @brief Macro of siflie ble event handler register.
 * @param[in] func register function.
 * @param[in] context user data.
 */
#ifndef BSP_USING_PC_SIMULATOR
    #define BLE_EVENT_REGISTER(func, context) SIFLI_BLE_REGISTER(func, context, "2")
    #define BLE_EVENT_REGISTER_HIGH(func, context) SIFLI_BLE_REGISTER(func, context, "1")
    #define BLE_EVENT_REGISTER_LOW(func, context) SIFLI_BLE_REGISTER(func, context, "3")
#else
    #define BLE_EVENT_REGISTER(func, context) SIFLI_BLE_REGISTER(func, context, ".ble_sifli_reg$2")
    #define BLE_EVENT_REGISTER_HIGH(func, context) SIFLI_BLE_REGISTER(func, context, ".ble_sifli_reg$1")
    #define BLE_EVENT_REGISTER_LOW(func, context) SIFLI_BLE_REGISTER(func, context, ".ble_sifli_reg$3")
#endif


#ifndef BSP_USING_PC_SIMULATOR
    #define BT_EVENT_REGISTER(func, context) SIFLI_BT_REGISTER(func, context, "2")                      /*!<Register BT event handler with normal priority*/
    #define BT_EVENT_REGISTER_HIGH(func, context) SIFLI_BT_REGISTER(func, context, "1")
    #define BT_EVENT_REGISTER_LOW(func, context) SIFLI_BT_REGISTER(func, context, "3")
#else
    #define BT_EVENT_REGISTER(func, context) SIFLI_BT_REGISTER(func, context, ".bt_sifli_reg$2")        /*!<Register BT event handler  with normal priority*/
    #define BT_EVENT_REGISTER_HIGH(func, context) SIFLI_BT_REGISTER(func, context, ".bt_sifli_reg$1")
    #define BT_EVENT_REGISTER_LOW(func, context) SIFLI_BT_REGISTER(func, context, ".bt_sifli_reg$3")
#endif


#define BT_RESET_MASK_BLE (1 << 0)
#define BT_RESET_MASK_BT (1 << 1)


enum ble_common_event_type_t
{
    BLE_COMMON_TYPE = 0x0,
    BLE_HCI_TYPE = 0x10,
    BLE_GAP_TYPE = 0x20,
    BLE_SIBLE_TYPE = 0x60,
    BLE_NVDS_TYPE = 0x80,
    BLE_ANCS_TYPE = 0x90,
    BLE_TIP_TYPE = 0xA0,
    BLE_CONNECTION_MANAGER_TYPE = 0xB0,
    BLE_AMS_TYPE = 0xD0,
    BLE_HRP_TYPE = 0xE0,
    BLE_BAS_TYPE = 0xF0,
    BLE_CPPC_TYPE = 0x100,
    BLE_CSCPC_TYPE = 0x110,
};


/**
 * @brief Common ble event that notify user.
 */
enum ble_common_event_t
{

    BLE_POWER_ON_IND = BLE_COMMON_TYPE, /**< BLE power on indication. */
    BLE_POWER_ON_CNF,                   /**< BLE power on confirm. */
    BLE_POWER_OFF_CNF,                  /**< BLE power off confirm. */
    BLE_DUT_TX_START_CNF,               /**< DUT TX start confirm. */
    BLE_DUT_RX_START_CNF,               /**< DUT RX start confirm. */
    BLE_DUT_END_IND,                    /**< DUT end indication. */
    BLE_TX_CONFIG_CNF,                   /**< DUT TX configure confirm. */
    BT_NS_DUT_RSP,                      /**< Non-signaling DUT reponse */
};

/**
 * @brief BLE power off type.
 */
enum ble_power_off_type
{
    BLE_OFF,                            /**< BLE stack power off. */
    BLE_SYSTEM_OFF,                     /**< BLE core power off. */
};


/**
 * @brief BT test operation for non-signaling mode.
 */
enum bt_ns_test_operation_t
{
    BT_TEST_OP_ENTER_TEST,            /**< Enter test mode. */
    BT_TEST_OP_EXIT_TEST,             /**< Exit test mode. */
    BT_TEST_OP_TX_TEST,               /**< TX test command. */
    BT_TEST_OP_RX_TEST,               /**< RX test command. */
    BT_TEST_OP_STOP_TEST,             /**< Stop TX/RX test command. */
};

/**
 * @brief BLE update type.
 */
typedef enum
{
    BLE_UPDATE_NO_UPDATE,     /**< No need to update. */
    BLE_UPDATE_ONCE,          /**< Only update if item doesn't exist. */
    BLE_UPDATE_ALWAYS        /**< Always update the item. */
} ble_common_update_type_t;


///BD Address structure
typedef struct
{
    ///6-byte array address value
    uint8_t  addr[BD_ADDR_LEN];
} bd_addr_t;

typedef struct
{
    uint8_t pwron_type;
} ble_power_on_t;

typedef struct
{
    /// GAPM requested operation:
    ///  - GAPM_LE_TEST_STOP: Unregister a LE Protocol/Service Multiplexer
    ///  - GAPM_LE_TEST_RX_START: Start RX Test Mode
    ///  - GAPM_LE_TEST_TX_START: Start TX Test Mode
    uint8_t operation;
    /// Tx or Rx Channel (Range 0x00 to 0x27)
    uint8_t channel;
    /// Length in bytes of payload data in each packet (only valid for TX mode, range 0x00-0xFF)
    uint8_t tx_data_length;
    /// Packet Payload type (only valid for TX mode @see enum gap_pkt_pld_type)
    uint8_t tx_pkt_payload;
    /// Test PHY rate (@see enum gap_test_phy)
    uint8_t phy;
    /// Modulation Index (only valid for RX mode @see enum gap_modulation_idx)
    uint8_t modulation_idx;
} ble_dut_mode_t;

/// Indicate end of test mode event
typedef struct
{
    /// Number of received packets
    uint16_t nb_packet_received;
} ble_dut_end_ind_t;


typedef struct
{
    uint8_t status;
} ble_status_t;

typedef ble_status_t ble_test_rx_start_cnf_t;                          /**< The struture of #BLE_DUT_RX_START_CNF. */

typedef ble_status_t ble_test_tx_start_cnf_t;                          /**< The struture of #BLE_DUT_TX_START_CNF. */




/**
 * @brief Structure of parameter for tx command in non-signaling test mode.
 */
typedef struct
{
    uint8_t channel;
    uint8_t pkt_payload;
    uint8_t pkt_type;
    uint8_t pwr_lvl;
    uint16_t pkt_len;
} bt_ns_test_mode_tx_para_t;

/**
 * @brief Structure of parameter for rx command in non-signaling test mode.
 */
typedef struct
{
    uint8_t channel;
    uint8_t pkt_type;
} bt_ns_test_mode_rx_para_t;

/**
 * @brief new Structure of parameter for rx command in non-signaling test mode.
 */
typedef struct
{
    uint8_t channel;
    uint8_t pkt_payload;
    uint8_t pkt_type;
    uint16_t pkt_len;
} bt_ns_test_new_rx_para_t;

/**
 * @brief new Structure of result for bt rx command in non-signaling test mode.
 */
typedef struct
{
    uint32_t err_bit_num;
    uint32_t total_bit_num;
    uint32_t err_pkt_num;
    uint32_t total_pkt_num;
    int8_t   rssi;
} bt_ns_test_new_rx_rslt_t;

typedef struct
{
    /// Tx or Rx Channel (Range 0x00 to 0x27)
    uint8_t channel;
    /// Test PHY rate (@see enum gap_test_phy)
    uint8_t phy;
    /// Modulation Index (only valid for RX mode @see enum gap_modulation_idx)
    uint8_t modulation_idx;
} ble_ns_test_rx_t;

/**
 * @brief new Structure of result for ble rx command in non-signaling test mode.
 */
typedef struct
{
    uint16_t total_pkt_num;
    int8_t   rssi;
} ble_ns_test_rx_rslt_t;

/**
 * @brief Structure of parameter for stop response in non-signaling test mode.
 */
typedef struct
{
    uint16_t cnt;
} bt_ns_test_mode_stop_test_para_t;


typedef union
{
    bt_ns_test_mode_tx_para_t tx_para;
    bt_ns_test_mode_rx_para_t rx_para;
} bt_ns_test_mode_cmd_para_t;

typedef union
{
    bt_ns_test_mode_stop_test_para_t stop_para;
} bt_ns_test_mode_rsp_para_t;


typedef struct
{
    enum bt_ns_test_operation_t op;
    bt_ns_test_mode_cmd_para_t para;
} bt_ns_test_mode_ctrl_cmd_t;

typedef struct
{
    enum bt_ns_test_operation_t op;
    uint8_t status;
    bt_ns_test_mode_rsp_para_t para;
} bt_ns_test_mode_ctrl_rsp_t;



/**
  * @brief  Power on BLE module
  * @retval 0 if success, otherwise failed.
  */
uint8_t ble_power_on(void);

/**
  * @brief  Power off BLE module
  * @param[in] pwroff_type Power off type, 0:turn off BLE MAC/BB only, 1: Turn off whole CPU.
  * @retval 0 if success, otherwise failed.
  */

uint8_t ble_power_off(uint8_t pwroff_type);


/**
  * @brief  Enter device firmware update mode.
  * @param[in] dut_ctrl Device firmware update configuration.
  * @retval 0 if success, otherwise failed.
  */
uint8_t ble_enter_dut_mode(ble_dut_mode_t *dut_ctrl);

//uint8_t ble_tx_power_configure(ble_tx_power_configure_t *config);


/**
  * @brief  Publish events to user that registered via #BLE_EVENT_REGISTER.
  * @param[in] event_id id to notify user.
  * @param[in] data Data asscoaited with event id.
  * @param[in] len Data len
 */
void ble_event_publish(uint16_t event_id, void *data, uint16_t len);

/**
  * @brief  Publish events to user that registered via #BT_EVENT_REGISTER.
  * @param[in] type type of event
  * @param[in] event_id id to notify user.
  * @param[in] data Data asscoaited with event id.
 */
void bt_event_publish(uint16_t type, uint16_t event_id, void *data);

/**
  * @brief  Get public address.
  * @param[out] addr Public address.
  * @retval 0 if public address could get.
 */
uint8_t ble_get_public_address(bd_addr_t *addr);


/**
  * @brief  User implmentation function. Stack will call this function to update public address.
  * @param[out] addr Public address
  * @retval #BLE_UPDATE_NO_UPDATE No need to update.
            #BLE_UPDATE_ONCE Only update if public address not existed.
            #BLE_UPDATE_ALWAYS Always use this public address.
  */
ble_common_update_type_t ble_request_public_address(bd_addr_t *addr);


/**
  * @brief  Get BT mac address from chip UID.
  * @param[out] addr generate mac address
  * @retval 0 Get mac address successful.
            -1 Parameter is null pointer.
            -2 read UID failed.
            -3 UID is 0.
  */
int bt_mac_addr_generate_via_uid(bd_addr_t *addr);

/**
  * @brief  Get BT mac address from chip UID v2.
  * @param[out] addr generate mac address
  * @retval 0 Get mac address successful.
            -1 Parameter is null pointer.
            -2 read UID failed.
            -3 UID is 0.
            -4 Generate UID is not support
            -5 V2 is not support
  */
int bt_mac_addr_generate_via_uid_v2(bd_addr_t *addr);

/**
  * @brief  Get BT randome mac address from chip UID.
  * @param[out] addr generate mac address
  * @retval 0 Get mac address successful.
            others failed.
  */
int bt_mac_addr_generate_rand_addr_via_uid(bd_addr_t *addr);


/**
  * @brief  Convert a string to BT mac addres.
  * @param[in] hexstr A string with format xxbxxbxxbxxbxxbxx or xxxxxxxxxxxx. x is a hex string(0-9,a-f,A-F)
  * @param[out] addr mac address
  * @retval addr length. Only 6 is correct. others failed.
  */
int bt_addr_convert_from_string_to_general(char *hexstr, bd_addr_t *addr);


#ifndef SOC_SF32LB55X
    uint8_t app_bt_get_non_signaling_test_status(void);
    uint8_t bt_enter_no_signal_dut_mode(bt_ns_test_mode_ctrl_cmd_t *dut_ctrl);
    uint8_t bt_system_reset(void);
    void bt_system_mask_clear(uint8_t mask);
#endif
char *bt_lib_get_ver(void);

#ifdef SOC_SF32LB52X
    void bt_sleep_control(uint8_t is_enable);
#endif


void bt_stack_nvds_update(void);




/**
* @}
*/

#endif // __BF0_BLE_COMMON_H
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
