/**
****************************************************************************************
*
* @file dbg.c
*
* @brief Debug function
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/

/**
****************************************************************************************
* @addtogroup DBG
* @{
****************************************************************************************
*/

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration
#if ((BLE_EMB_PRESENT) || (BT_EMB_PRESENT))
#include "rwip.h"           // IP definitions
#include "co_error.h"       // common error definition
#include "ke_task.h"        // kernel task definition
#include "dbg_trc_int.h"    // debug tracer definition
#include "dbg.h"            // debug definition

#if (HCI_PRESENT)
    #include "hci.h"
#endif //(HCI_PRESENT)

/*
 * DEFINES
 ****************************************************************************************
 */

extern const struct ke_task_desc TASK_DESC_DBG;


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */


/*
 * LOCAL FUNCTION DEFINITION
 ****************************************************************************************
 */



/*
 * EXPORTED FUNCTION DEFINITION
 ****************************************************************************************
 */
#if (RW_DEBUG)
void dbg_init(bool is_reset)
{
    if (!is_reset)
    {
        // Create DEBUG Task
        ke_task_create(TASK_DBG, &TASK_DESC_DBG);

#if (RW_SWDIAG)
        // Initialize SW profiling module
        dbg_swdiag_init();
#endif //RW_SWDIAG
    }
    else
    {
#if (RW_DEBUG_STACK_PROF)
        // Initialise stack memory area
        stack_init();
#endif //(RW_DEBUG_STACK_PROF)
    }

#if (TRACER_PRESENT)
    //Initialize tracer
    dbg_trc_init(is_reset);
#endif /*(TRACER_PRESENT)*/
}
#endif //RW_DEBUG

void dbg_platform_reset_complete(uint32_t error)
{
    // structure type for the complete command event
    struct hci_basic_cmd_cmp_evt *evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, HCI_DBG_PLF_RESET_CMD_OPCODE, hci_basic_cmd_cmp_evt);

    if (error == RESET_TO_ROM)
    {
        evt->status = CO_ERROR_HARDWARE_FAILURE;
    }
    else if (error == RESET_AND_LOAD_FW)
    {
        evt->status = CO_ERROR_NO_ERROR;
    }

    hci_send_2_host(evt);
}

#endif // ((BLE_EMB_PRESENT) || (BT_EMB_PRESENT))
///@} DBG
