/**
 ****************************************************************************************
 *
 * @file lc_clk.c
 *
 * @brief VS read piconnet clock.
 *
 * Copyright (C) RivieraWaves 2019
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LCCLK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"    // stack configuration

#if (BT_READ_PICONET_CLOCK)
#include "ke_mem.h"         // kernel memory
#include "ke_msg.h"         // kernel types
#include "ld.h"             // link driver
#include "hci.h"            // host controller interface
#include "lc_int.h"         // link controller internal definitions
#include "dbg.h"

#if (GPIO_SUPPORT)
    #include "gpio.h" //Include GPIO driver
#endif //(GPIO_SUPPORT)

/*
 * LOCAL FUNCTION DEFINITONS
 ****************************************************************************************
 */

/**
****************************************************************************************
* @brief get local timestamp with optional pulse on gpio.
*
* @param[in] trig_pulse          Enable or disable the pulse on gpio.
*
* @return timestamp              Local clock timestamp.
*
*****************************************************************************************
*/
__STATIC rwip_time_t lc_clk_local_clock_get(bool trig_pulse)
{
    rwip_time_t local_clock;

#if (GPIO_SUPPORT)
    if (trig_pulse)
    {
        // Get local timestamp and trigger pulse
        GLOBAL_INT_DISABLE();
        gpio_toggle_pin(GPIO_PIN_31);
        local_clock = rwip_time_get();
        gpio_toggle_pin(GPIO_PIN_31);
        GLOBAL_INT_RESTORE();
    }
#else
    //Get Local timestamp
    local_clock = rwip_time_get();
#endif //(GPIO_SUPPORT)

    return local_clock;
}

/*
 * EXPORTED FUNCTION DEFINITONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Vendor specific command to read the piconet clock
 *
 * @param[in] msgid Id of the message received (probably unused).
 * @param[in] param Pointer to the parameters of the message.
 * @param[in] dest_id ID of the receiving task instance (probably unused).
 * @param[in] src_id ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *****************************************************************************************
 */
int hci_vs_rd_piconet_clock_cmd_handler(struct hci_vs_rd_piconet_clock_cmd const *param, uint16_t opcode)
{
    // structure type for the complete command event
    struct hci_vs_rd_piconet_clock_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, HCI_VS_RD_PICONET_CLOCK_CMD_OPCODE, hci_vs_rd_piconet_clock_cmd_cmp_evt);

    //Local clock read
    rwip_time_t local_clock = lc_clk_local_clock_get(param->trig_pulse);
    event->loc_clk_hslt = local_clock.hs;
    event->loc_clk_hus = local_clock.hus;

    //Init other parameters to default
    event->conhdl = 0xFFFF;
    event->pic_clk_off_hslt = 0;
    event->pic_bit_off_hus = 0;

    if (LOCAL_CLOCK == param->clk_type)
    {
        event->status = CO_ERROR_NO_ERROR;
    }
    else //PICONET_CLOCK
    {
        ASSERT_INFO(PICONET_CLOCK == param->clk_type, param->clk_type, param->conhdl);

        uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;

        if ((param->conhdl >= BT_ACL_CONHDL_MIN) && (param->conhdl <= BT_ACL_CONHDL_MAX))
        {
            uint8_t link_id = param->conhdl - BT_ACL_CONHDL_MIN;

            if (lc_env[link_id] != NULL)
            {
                struct lc_env_tag *lc_env_ptr = lc_env[link_id];

                if (lc_env_ptr->link.ConnectionCompleteSent)
                {
                    // Get estimated clock offset (in half-slots)
                    rwip_time_t piconet_offset = ld_acl_clock_offset_get(link_id);

                    //return piconet clock
                    event->pic_clk_off_hslt = piconet_offset.hs;
                    event->pic_bit_off_hus = piconet_offset.hus;

                    event->conhdl = param->conhdl;
                    status = CO_ERROR_NO_ERROR;
                }
                else
                {
                    status = CO_ERROR_CONTROLLER_BUSY;
                }
            }
        }

        event->status = status;
    }

    // sends the message
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}
#endif // (BT_READ_PICONET_CLOCK)

///@} LCCLK
