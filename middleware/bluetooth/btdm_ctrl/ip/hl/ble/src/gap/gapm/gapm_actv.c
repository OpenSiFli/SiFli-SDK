/**
 ****************************************************************************************
 *
 * @file gapm_actv.c
 *
 * @brief Generic Access Profile Manager - Activity manager module.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup GAPM_ACTV Generic Access Profile Manager - Activity manager module.
 * @ingroup GAPM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include <string.h>

#include "rwip_config.h"
#include "gap.h"
#include "gapm_task.h"
#include "gapm_int.h"
#include "gapc.h"
#include "ke_mem.h"
#include "hci.h"
#include "ke_timer.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/// Bit indicating that resolvable private address has to be generated by the controller if
/// controller privacy has been enabled.
#define GAPM_ACTV_OWN_ADDR_TYPE_PRIV_BIT    (1 << 1)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/*
 * MACROS
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Check if a new activity can be created by GAPM and return the identifier to be
 * used.
 *
 * @return GAPM_ACTV_INVALID_IDX if no more activity can be created else the found activity
 * identifier.
 ****************************************************************************************
 */
__STATIC uint8_t gapm_actv_get_free_idx(void)
{
    // Identifier of an available activity, invalid by default
    uint8_t actv_idx = GAPM_ACTV_INVALID_IDX;

    // Loop over the activities
    for (int i = 0; i < GAPM_ACTV_NB; i++)
    {
        if (gapm_env.actvs[i] == NULL)
        {
            // Free identifier has been found, stop looping
            actv_idx = i;
            break;
        }
    }

    // Return the found identifier
    return (actv_idx);
}

/**
 ****************************************************************************************
 * @brief Free a previously allocated activity structure. Make the provided activity
 * identifier available.
 * The activity provided might not be allocated
 *
 * @param[in] actv_idx    Activity identifier
 ****************************************************************************************
 */
__STATIC void gapm_actv_free(uint8_t actv_idx)
{
    // Retrieve pointer to the activity structure
    struct gapm_actv_tag *p_actv = gapm_env.actvs[actv_idx];

    if (p_actv)
    {
#if (BLE_OBSERVER)
        if (p_actv->type == GAPM_ACTV_TYPE_SCAN)
        {
            gapm_scan_actv_clean(p_actv);
        }
        else if (p_actv->type == GAPM_ACTV_TYPE_PER_SYNC)
        {
            gapm_per_sync_clear_fragments((struct gapm_actv_per_sync_tag *)p_actv);
        }
#endif //(BLE_OBSERVER)

        // Free the allocated memory
        ke_free(p_actv);
    }

    // Free the activity identifier
    gapm_env.actvs[actv_idx] = NULL;
}

/**
 ****************************************************************************************
 * @brief Send GAPM_ACTIVITY_CREATED_IND message to the application. Indicate that an activity
 * structure has been successfully created and provided the allocated activity identifier
 * to be provided in any upcoming command implying the activity.
 *
 * @param[in] p_actv    Pointer to the structure describing the created activity
 ****************************************************************************************
 */
__STATIC void gapm_actv_send_created_ind(struct gapm_actv_tag *p_actv)
{
    // Allocate the message
    struct gapm_activity_created_ind *p_ind = KE_MSG_ALLOC(GAPM_ACTIVITY_CREATED_IND,
            p_actv->requester, TASK_GAPM,
            gapm_activity_created_ind);

    // Fill the message
    p_ind->actv_type = p_actv->type;
    p_ind->actv_idx = p_actv->idx;

    // Send the message
    ke_msg_send(p_ind);
}

/**
 ****************************************************************************************
 * @brief Send GAPM_ACTIVITY_STOPPED_IND message to the application.
 *
 * @param[in] p_actv    Pointer to the structure describing the stopped activity
 ****************************************************************************************
 */
__STATIC void gapm_actv_send_stopped_ind(struct gapm_actv_tag *p_actv, uint8_t reason,
        bool per_adv_stop)
{
    // Allocate the message
    struct gapm_activity_stopped_ind *p_ind = KE_MSG_ALLOC(GAPM_ACTIVITY_STOPPED_IND,
            p_actv->requester, TASK_GAPM,
            gapm_activity_stopped_ind);

    // Fill the message
    p_ind->actv_type = p_actv->type;
    p_ind->actv_idx = p_actv->idx;
    p_ind->reason = reason;
    p_ind->per_adv_stop = per_adv_stop;

    // Send the message
    ke_msg_send(p_ind);
}

/**
 ****************************************************************************************
 * @brief Get next activity whose state is the one provided as parameter and activity identifier is
 * at least provided identifier.
 *
 * @param[in] actv_idx  Activity identifier
 * @param[in] state     Activity state
 *
 * @return A point to the structure describing the next activity with given state.
 ****************************************************************************************
 */
__STATIC struct gapm_actv_tag *gapm_actv_get_next(uint8_t actv_idx, uint8_t state)
{
    // Activity structure
    struct gapm_actv_tag *p_actv = NULL;

    // Check if there is more activities
    for (int i = actv_idx; i < GAPM_ACTV_NB; i++)
    {
        // Loop activity structure
        struct gapm_actv_tag *p_loop_actv = gapm_env.actvs[i];

        if (p_loop_actv && (p_loop_actv->state == state))
        {
            // Required activity has been found, stop loop
            p_actv = p_loop_actv;
            break;
        }
    }

    // Return the found activity structure
    return (p_actv);
}

/**
 ****************************************************************************************
 * @brief Check creation parameters that are common for all kind of activities.
 *
 * @param[in] p_param      Pointer to the received GAPM_ACTIVITY_CREATE_CMD message structure
 *
 * @return GAP_ERR_NO_ERROR if all parameters are valid, else GAP_ERR_INVALID_PARAM
 ****************************************************************************************
 */
__STATIC uint8_t gapm_actv_check_create_param(struct gapm_activity_create_cmd *p_param)
{
    // Error code
    uint8_t error = GAP_ERR_INVALID_PARAM;

    do
    {
        if (p_param->own_addr_type > GAPM_GEN_NON_RSLV_ADDR)
        {
            break;
        }

        error = GAP_ERR_NO_ERROR;
    }
    while (0);

    // Return the error code
    return (error);
}

/*
 * EXTERNAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void gapm_actv_init(bool reset)
{
    for (int i = 0; i < GAPM_ACTV_NB; i++)
    {
        // Free the activity identifier
        if (reset)
        {
            gapm_actv_free(i);
        }
        else
        {
            gapm_env.actvs[i] = NULL;
        }
    }

    // Initialize currently used activity identifier
    gapm_env.actv_idx      = GAPM_ACTV_INVALID_IDX;
    // Initialize counters of activities
    gapm_env.created_actvs = 0;
    gapm_env.started_actvs = 0;
    // Initialize number of devices in the white list
    gapm_env.nb_dev_wl     = 0;
    gapm_env.nb_pause_act  = 0;
    gapm_env.pause_act_flg = 0;

#if (BLE_BROADCASTER)
    gapm_env.nb_adv_actv   = 0;
#endif //(BLE_BROADCASTER)

#if (BLE_OBSERVER)
    gapm_env.scan_actv_idx = GAPM_ACTV_INVALID_IDX;
#endif //(BLE_OBSERVER)

#if (BLE_CENTRAL)
    gapm_env.init_actv_idx = GAPM_ACTV_INVALID_IDX;
#endif //(BLE_CENTRAL)
}

struct gapm_actv_tag *gapm_actv_alloc(uint8_t actv_idx, uint8_t size)
{
    // Allocated activity structure
    struct gapm_actv_tag *p_actv = (struct gapm_actv_tag *)ke_malloc(size, KE_MEM_ENV);

    if (p_actv)
    {
        // Clear content of allocated structure
        memset(p_actv, 0, size);

        // Store the activity identifier
        p_actv->idx   = actv_idx;
        // Set creating state
        p_actv->state = GAPM_ACTV_CREATING;

        // Store the pointer to the activity structure
        gapm_env.actvs[actv_idx] = p_actv;
    }

    // Return the pointer to the indicated activity structure
    return (p_actv);
}

uint8_t gapm_actv_get_hci_own_addr_type(uint8_t app_addr_type)
{
    //  Own address type
    uint8_t hci_own_addr_type = ADDR_RAND;

    if (app_addr_type == GAPM_STATIC_ADDR)
    {
        if (!(gapm_get_address_type() & GAPM_PRIV_CFG_PRIV_ADDR_BIT))
        {
            hci_own_addr_type = ADDR_PUBLIC;
        }
    }

    // Check if controller privacy is enabled
    if (gapm_get_address_type() & GAPM_PRIV_CFG_PRIV_EN_BIT)
    {
        hci_own_addr_type |= GAPM_ACTV_OWN_ADDR_TYPE_PRIV_BIT;
    }

    return (hci_own_addr_type);
}

bool gapm_actv_retrieve_cmd_cmp_evt(struct gapm_actv_tag **pp_actv, uint16_t opcode)
{
    // Indicate if activity has been found
    bool found = false;
    // Activity
    struct gapm_actv_tag *p_actv;

    do
    {
        // Check if an air operation is pending
        if (gapm_get_operation(GAPM_OP_AIR) == GAPM_NO_OP)
        {
            // Drop the message
            break;
        }

        // Sanity check
        ASSERT_ERR(gapm_env.actv_idx < GAPM_ACTV_NB);

        // Retrieve the currently used activity
        p_actv = gapm_env.actvs[gapm_env.actv_idx];

        // Sanity check
        ASSERT_ERR(p_actv != NULL);

        *pp_actv = p_actv;

        // Check that received complete event message is the one that was expected
        if (opcode != p_actv->next_exp_opcode)
        {
            // Drop the message
            break;
        }

        // Reset expected operation code
        p_actv->next_exp_opcode = HCI_NO_OPERATION_CMD_OPCODE;

        // Activity has been found
        found = true;
    }
    while (0);

    return (found);
}

void gapm_actv_created(struct gapm_actv_tag *p_actv, uint8_t error)
{
    do
    {
        if (error == GAP_ERR_NO_ERROR)
        {
            // Send GAPM_ACTIVITY_CREATED_IND message to the application
            gapm_actv_send_created_ind(p_actv);

            // Check if a random address (static or not) has to be used,
            // if yes address must be generated (if not static) and provided to
            // the controller
            // No address is needed for periodic synchronization
            if ((p_actv->type == GAPM_ACTV_TYPE_PER_SYNC) ||
                    ((p_actv->own_addr_type == GAPM_STATIC_ADDR) &&
                     !(GAPM_F_GET(gapm_env.cfg_flags, ADDR_TYPE) & GAPM_PRIV_CFG_PRIV_ADDR_BIT)))
            {
                // Increase number of created activities
                gapm_env.created_actvs++;

                // Update activity state
                p_actv->state = GAPM_ACTV_CREATED;
            }
            else
            {
                // Activity will be considered has created when gapm_actv_rand_addr_set_ind function
                // will be called
                gapm_addr_set_rand_addr(p_actv);
                break;
            }
        }
        else
        {
            // Free the allocated activity structure
            gapm_actv_free(p_actv->idx);
        }

        // End operation and send status to the application
        gapm_send_complete_evt(GAPM_OP_AIR, error);
    }
    while (0);
}
extern void gapm_update_state(uint8_t operation, bool busy);
static void gapm_operation_cleanup_internal(uint8_t op_type)
{
    // check if operation is freed
    if (gapm_env.operation[op_type] != NULL)
    {
        gapm_env.operation[op_type] = NULL;
    }
    else
    {
        ASSERT_ERR(0);
    }

    // set operation state to Idle
    gapm_update_state(op_type, false);
}

#ifndef GAPM_SLIM
static struct gapm_activity_start_cmd gapm_start_param = {GAPM_START_ALL_ACTIVITIES, 0};
static void gapm_actv_resume()
{
    // Error code
    uint8_t error = GAP_ERR_COMMAND_DISALLOWED;
    // Get indicated activity
    struct gapm_actv_tag *p_actv;
    uint8_t idx = 0xFF;
    struct gapm_activity_start_cmd *p_param = &gapm_start_param;

    do
    {
        // Check provided activity index
        if ((gapm_env.nb_pause_act == 0) || (gapm_env.nb_pause_act > GAPM_ACTV_NB))
        {
            error = GAP_ERR_INVALID_PARAM;
            break;
        }

        idx = gapm_env.pause_act_idx[gapm_env.nb_pause_act - 1];

        if (idx >= GAPM_ACTV_NB)
        {
            break;
        }

        // Get indicated activity
        p_actv = gapm_env.actvs[idx];

        // Check if activity well exists
        if (!p_actv)
        {
            break;
        }

        if (p_actv->state != GAPM_ACTV_CREATED)
        {
            break;
        }
        else
        {
            p_actv->state = GAPM_ACTV_STARTING;
        }

        rt_kprintf("resume act num:%d, pause current idx:%d\n", gapm_env.nb_pause_act, idx);

        gapm_env.nb_pause_act--;
        p_param->actv_idx = idx;

        // Call the handler attached with the type of activity
        error = p_actv->cb_actv_start(p_actv, p_param);

        if (error != GAP_ERR_NO_ERROR)
        {
            p_actv->state = GAPM_ACTV_CREATED;
        }
    }
    while (0);

    // If an error has been raised, inform the application, else operation will be terminated
    // in gapm_actv_started function.
    if (error != GAP_ERR_NO_ERROR)
    {
        gapm_env.pause_act_flg = 0;
        // Send the GAPM_CMP_EVT message containing the status
        //gapm_send_complete_evt(GAPM_OP_AIR, error);
        gapm_operation_cleanup_internal(GAPM_OP_AIR);
        rt_kprintf("resume act fin, num:%d, idx:%d\n", gapm_env.nb_pause_act, idx);
    }
}
#endif // GAPM_SLIM
void gapm_actv_started(struct gapm_actv_tag *p_actv, uint8_t error)
{
    uint8_t op_code = gapm_get_operation(GAPM_OP_AIR);

    // End operation and send status to the application
    if (op_code != GAPM_START_ALL_ACTIVITIES)
    {
        gapm_send_complete_evt(GAPM_OP_AIR, error);
    }
    else
    {
        rt_kprintf("resume act %d, status %d\n", p_actv->idx, error);
    }

    if (error == GAP_ERR_NO_ERROR)
    {
        if (p_actv->state == GAPM_ACTV_STARTING)
        {
            // Increase number of started activities
            gapm_env.started_actvs++;

            // Update activity state
            p_actv->state = GAPM_ACTV_STARTED;

#ifndef GAPM_SLIM
            if (op_code == GAPM_START_ALL_ACTIVITIES)
            {
                gapm_actv_resume();
            }
#endif // !GAPM_SLIM
        }
    }
    else
    {
        p_actv->state = GAPM_ACTV_CREATED;
    }
}

void gapm_actv_stopped(struct gapm_actv_tag *p_actv, uint8_t status)
{
    // Special case: in case of periodic advertising, extended advertising can be stopped due to
    // timeout but periodic advertising is still running and activity must remain STARTED
    if ((status == GAP_ERR_TIMEOUT)
            && (p_actv->type == GAPM_ACTV_TYPE_ADV)
            && (p_actv->subtype == GAPM_ADV_TYPE_PERIODIC))
    {
        // Send GAPM_ACTIVITY_STOPPED_IND message to the application
        gapm_actv_send_stopped_ind(p_actv, status, false);
    }
    else
    {
        // Send GAPM_ACTIVITY_STOPPED_IND message to the application
        if (gapm_env.pause_act_flg == 0)
        {
            gapm_actv_send_stopped_ind(p_actv, status, true);
        }

        // Decrease number of started activities
        gapm_env.started_actvs--;

        if (p_actv->state == GAPM_ACTV_STOPPING)
        {
            // Get current air operation code
            uint8_t op_code = gapm_get_operation(GAPM_OP_AIR);

            if (op_code == GAPM_STOP_ACTIVITY)
            {
                // End operation and send status to the application
                gapm_send_complete_evt(GAPM_OP_AIR, GAP_ERR_NO_ERROR);
            }
            else if (op_code == GAPM_STOP_ALL_ACTIVITIES)
            {
                if (gapm_env.started_actvs)
                {
                    // Get next activity to be stopped
                    struct gapm_actv_tag *p_next_actv = gapm_actv_get_next(p_actv->idx + 1, GAPM_ACTV_STARTED);

                    // Sanity check
                    if (gapm_env.pause_act_flg == 0)
                    {
                        ASSERT_ERR(0);
                    }
                    else
                    {
                        gapm_env.pause_act_idx[gapm_env.nb_pause_act] = p_actv->idx;

                        gapm_env.nb_pause_act++;
                        rt_kprintf("pause act num:%d, pause current idx:%d\n", gapm_env.nb_pause_act, p_actv->idx);
                    }

                    // Stop the activity
                    p_next_actv->cb_actv_stop(p_next_actv);
                }
                else
                {
                    // All activities have been stopped, end the operation
                    if (gapm_env.pause_act_flg == 0)
                    {
                        gapm_send_complete_evt(GAPM_OP_AIR, GAP_ERR_NO_ERROR);
                    }
                    else
                    {
                        gapm_operation_cleanup_internal(GAPM_OP_AIR);
                        rt_kprintf("pause act finish, num:%d\n", gapm_env.nb_pause_act);
                    }
                }
            }
        }

        // Update activity state
        p_actv->state = GAPM_ACTV_CREATED;
    }
}

void gapm_actv_deleted(struct gapm_actv_tag *p_actv)
{
    // Get current air operation code
    uint8_t op_code = gapm_get_operation(GAPM_OP_AIR);

    // Decrease number of started activities
    gapm_env.created_actvs--;

    if (op_code == GAPM_DELETE_ACTIVITY)
    {
        // End operation and send status to the application
        gapm_send_complete_evt(GAPM_OP_AIR, GAP_ERR_NO_ERROR);
    }
    else if (op_code == GAPM_DELETE_ALL_ACTIVITIES)
    {
        if (gapm_env.created_actvs)
        {
            // Get next activity to delete
            struct gapm_actv_tag *p_next_actv = gapm_actv_get_next(p_actv->idx + 1, GAPM_ACTV_CREATED);

            // Sanity check
            ASSERT_ERR(p_next_actv);

            // Stop the activity
            p_next_actv->cb_actv_delete(p_next_actv);
        }
        else
        {
            // All activities have been deleted, end the operation
            gapm_send_complete_evt(GAPM_OP_AIR, GAP_ERR_NO_ERROR);
        }
    }

    // Free the activity structure
    gapm_actv_free(p_actv->idx);
}

void gapm_actv_rand_addr_set_ind(struct gapm_actv_tag *p_actv)
{
    // Increase number of created activities
    gapm_env.created_actvs++;

    // Update activity state
    p_actv->state = GAPM_ACTV_CREATED;

    // End operation and send status to the application
    gapm_send_complete_evt(GAPM_OP_AIR, CO_ERROR_NO_ERROR);
}

/*
 * MESSAGE HANDLERS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles request of creating a new activity (GAPM_ACTIVITY_CREATE_CMD message):
 *  - GAPM_CREATE_ADV_ACTIVITY: Create advertising activity
 *  - GAPM_CREATE_SCAN_ACTIVITY: Create scanning activity
 *  - GAPM_CREATE_INIT_ACTIVITY: Create initiating activity
 *  - GAPM_CREATE_PERIOD_SYNC_ACTIVITY: Create periodic synchronization activity
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapm_activity_create_cmd_handler(ke_msg_id_t const msgid,
                                     struct gapm_activity_create_cmd *p_param,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // List of handler supported operations
    enum gapm_operation supp_ops[] =
    {
#if (BLE_BROADCASTER)
        GAPM_CREATE_ADV_ACTIVITY,
#endif //(BLE_BROADCASTER)
#if (BLE_OBSERVER)
        GAPM_CREATE_SCAN_ACTIVITY,
        GAPM_CREATE_PERIOD_SYNC_ACTIVITY,
#endif //(BLE_OBSERVER)
#if (BLE_CENTRAL)
        GAPM_CREATE_INIT_ACTIVITY,
#endif //(BLE_CENTRAL)
        GAPM_NO_OP
    };
    // Check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_AIR, p_param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        // Error code
        uint8_t error;
        // Allocated activity index
        uint8_t actv_idx;

        do
        {
            // Check parameters common for all kind of activity
            error = gapm_actv_check_create_param(p_param);

            if (error != GAP_ERR_NO_ERROR)
            {
                break;
            }

            // Check if an available activity can been found
            actv_idx = gapm_actv_get_free_idx();

            if (actv_idx == GAPM_ACTV_INVALID_IDX)
            {
                error = GAP_ERR_INSUFF_RESOURCES;
                break;
            }

            // Call the handler attached with the requested operation
            switch (p_param->operation)
            {
#if (BLE_BROADCASTER)
            case GAPM_CREATE_ADV_ACTIVITY:
            {
                error = gapm_adv_create(actv_idx, (struct gapm_activity_create_adv_cmd *)p_param);
            }
            break;
#endif //(BLE_BROADCASTER)

#if (BLE_OBSERVER)
            case GAPM_CREATE_SCAN_ACTIVITY:
            {
                error = gapm_scan_create(actv_idx, p_param);
            }
            break;

            case GAPM_CREATE_PERIOD_SYNC_ACTIVITY:
            {
                error = gapm_per_sync_create(actv_idx, p_param);
            }
            break;
#endif //(BLE_OBSERVER)

#if (BLE_CENTRAL)
            case GAPM_CREATE_INIT_ACTIVITY:
            {
                error = gapm_init_create(actv_idx, p_param);
            }
            break;
#endif //(BLE_CENTRAL)

            default:
            {
                // Cannot happen as operation code is checked in gapm_process_op function
            }
            break;
            }

            if (error == GAP_ERR_NO_ERROR)
            {
                if (p_param->own_addr_type == GAPM_STATIC_ADDR)
                {
                    // Check if a static private random address has to be used
                    if (GAPM_F_GET(gapm_env.cfg_flags, ADDR_TYPE) & GAPM_PRIV_CFG_PRIV_ADDR_BIT)
                    {
                        // Activity structure has been allocated
                        struct gapm_actv_tag *p_actv = gapm_env.actvs[actv_idx];

                        // Copy the static random address if the activity structure
                        memcpy(&p_actv->addr.addr[0], &gapm_env.addr.addr[0], GAP_BD_ADDR_LEN);
                    }
                }
            }
        }
        while (0);

        // If an error has been raised send a status to the application, else operation will be completed
        // in gapm_actv_created function.
        if (error != GAP_ERR_NO_ERROR)
        {
            // Send the GAPM_CMP_EVT message containing the status
            gapm_send_complete_evt(GAPM_OP_AIR, error);
        }
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles request of starting an activity (GAPM_ACTIVITY_START_CMD message):
 *  - GAPM_START_ACTIVITY: Start an activity
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapm_activity_start_cmd_handler(ke_msg_id_t const msgid,
                                    struct gapm_activity_start_cmd *p_param,
                                    ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // List of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_START_ACTIVITY,
                                      GAPM_NO_OP
                                     };
    // Check if operation can be executed
    int msg_status = KE_MSG_SAVED;

    if (gapm_env.pause_act_flg == 0)
    {
        msg_status = gapm_process_op(GAPM_OP_AIR, p_param, supp_ops);
    }
    else
    {
        rt_kprintf("start activity postpone, idx:%d\n", p_param->actv_idx);
    }


    if (p_param->operation == GAPM_START_ALL_ACTIVITIES)
    {
        ASSERT_ERR(0);
    }

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        // Error code
        uint8_t error = GAP_ERR_COMMAND_DISALLOWED;
        // Get indicated activity
        struct gapm_actv_tag *p_actv;

        do
        {
            // Check provided activity index
            if (p_param->actv_idx >= GAPM_ACTV_NB)
            {
                error = GAP_ERR_INVALID_PARAM;
                break;
            }

            // Get indicated activity
            p_actv = gapm_env.actvs[p_param->actv_idx];

            // Check if activity well exists
            if (!p_actv)
            {
                break;
            }

            // Periodic advertising can be started even if state is STARTED, else state must be CREATED
            if (p_actv->state == GAPM_ACTV_STARTED)
            {
                if ((p_actv->type != GAPM_ACTV_TYPE_ADV)
                        || (p_actv->subtype != GAPM_ADV_TYPE_LEGACY))
                {
                    break;
                }
            }
            else if (p_actv->state != GAPM_ACTV_CREATED)
            {
                break;
            }
            else
            {
                p_actv->state = GAPM_ACTV_STARTING;
            }

            // Call the handler attached with the type of activity
            error = p_actv->cb_actv_start(p_actv, p_param);

            if (error != GAP_ERR_NO_ERROR)
            {
                p_actv->state = GAPM_ACTV_CREATED;
            }
        }
        while (0);

        // If an error has been raised, inform the application, else operation will be terminated
        // in gapm_actv_started function.
        if (error != GAP_ERR_NO_ERROR)
        {
            // Send the GAPM_CMP_EVT message containing the status
            gapm_send_complete_evt(GAPM_OP_AIR, error);
        }
    }

    return (msg_status);
}

#ifndef GAPM_SLIM
/**
 ****************************************************************************************
 * @brief Handles request of starting an activity (GAPM_ACTIVITY_START_CMD message):
 *  - GAPM_START_ACTIVITY: Start an activity
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapm_activity_resume_all_handler_internal()
{
    struct gapm_activity_start_cmd *p_param = &gapm_start_param;
    // List of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_START_ALL_ACTIVITIES,
                                      GAPM_NO_OP
                                     };
    // Check if operation can be executed
    int msg_status = 0;

    if (1 == gapm_env.pause_act_flg)
    {
        msg_status = gapm_process_op(GAPM_OP_AIR, p_param, supp_ops);
    }

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        gapm_actv_resume();
    }

    return (msg_status);
}

#endif // !GAPM_SLIM
/**
 ****************************************************************************************
 * @brief Handles request of stopping one or all activities (GAPM_ACTIVITY_STOP_CMD message):
 *  - GAPM_STOP_ACTIVITY: Stop an activity
 *  - GAPM_STOP_ALL_ACTIVITIES: Stop all activities
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapm_activity_stop_cmd_handler(ke_msg_id_t const msgid,
                                   struct gapm_activity_stop_cmd *p_param,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // List of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_STOP_ACTIVITY,
                                      GAPM_STOP_ALL_ACTIVITIES,
                                      GAPM_NO_OP
                                     };
    // Check if operation can be executed
    int msg_status;

    if (gapm_env.pause_act_flg == 0)
    {
        msg_status = gapm_process_op(GAPM_OP_AIR, p_param, supp_ops);
    }
    else
    {
        rt_kprintf("stop activity postpone, idx:%d ops:0x%x\n", p_param->actv_idx, p_param->operation);
    }

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        // Error code
        uint8_t error = GAP_ERR_NO_ERROR;
        // Indicate that operation is done
        bool op_done = true;
        // Get indicated activity
        struct gapm_actv_tag *p_actv;

        do
        {
            if (p_param->operation == GAPM_STOP_ACTIVITY)
            {
                // Check provided activity index
                if (p_param->actv_idx >= GAPM_ACTV_NB)
                {
                    error = GAP_ERR_INVALID_PARAM;
                    break;
                }

                // Get indicated activity
                p_actv = gapm_env.actvs[p_param->actv_idx];

                // Check if activity exists and if it is in GAPM_ACTV_STARTED state
                if (!p_actv || (p_actv->state != GAPM_ACTV_STARTED))
                {
                    error = GAP_ERR_COMMAND_DISALLOWED;
                    break;
                }
            }
            else
            {
                // Get next started activity
                p_actv = gapm_actv_get_next(0, GAPM_ACTV_STARTED);

                // Check if activity exists
                if (!p_actv)
                {
                    break;
                }
            }

            op_done = false;

            p_actv->state = GAPM_ACTV_STOPPING;

            // Stop the activity
            p_actv->cb_actv_stop(p_actv);
        }
        while (0);

        // If operation is already over, inform the application
        if (op_done)
        {
            // Send the GAPM_CMP_EVT message containing the status
            gapm_send_complete_evt(GAPM_OP_AIR, error);
        }
    }

    return (msg_status);
}

/**
 ****************************************************************************************
 * @brief Handles request of stopping all activities used by internal function
 *  - GAPM_STOP_ALL_ACTIVITIES: Stop all activities
 *
 *  - used for SET RAL
 ****************************************************************************************
 */
static struct gapm_activity_stop_cmd gapm_stop_param = {GAPM_STOP_ALL_ACTIVITIES, 0};
int gapm_activity_suspend_all_handler_internal()
{
    struct gapm_activity_stop_cmd *p_param = &gapm_stop_param;
    // List of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_STOP_ALL_ACTIVITIES,
                                      GAPM_NO_OP
                                     };
    // Check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_AIR, p_param, supp_ops);

    rt_kprintf("act num:%d\n", gapm_env.started_actvs);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        // Error code
        uint8_t error = GAP_ERR_NO_ERROR;
        // Indicate that operation is done
        bool op_done = true;
        // Get indicated activity
        struct gapm_actv_tag *p_actv;

        do
        {
            if (p_param->operation == GAPM_STOP_ALL_ACTIVITIES)
            {
                // Get next started activity
                p_actv = gapm_actv_get_next(0, GAPM_ACTV_STARTED);

                // Check if activity exists
                if (!p_actv)
                {
                    break;
                }
            }
            else
            {
                ASSERT_ERR(0);
            }

            gapm_env.pause_act_idx[gapm_env.nb_pause_act] = p_actv->idx;

            gapm_env.nb_pause_act++;

            gapm_env.pause_act_flg = 1;

            rt_kprintf("pause act num:%d, pause current idx:%d\n", gapm_env.nb_pause_act, p_actv->idx);

            op_done = false;

            p_actv->state = GAPM_ACTV_STOPPING;

            // Stop the activity
            p_actv->cb_actv_stop(p_actv);
        }
        while (0);
        // If operation is already over, inform the application
        if (op_done)
        {
            // Send the GAPM_CMP_EVT message containing the status
            //gapm_send_complete_evt(GAPM_OP_AIR, error);
            gapm_operation_cleanup_internal(GAPM_OP_AIR);
        }
    }

    return (gapm_env.pause_act_flg);

}

/**
 ****************************************************************************************
 * @brief Handles request of deleting one or all activities (GAPM_ACTIVITY_DELETE_CMD message)
 *  - GAPM_DELETE_ACTIVITY: Delete an activity
 *  - GAPM_DELETE_ALL_ACTIVITIES: Delete all activities
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapm_activity_delete_cmd_handler(ke_msg_id_t const msgid,
                                     struct gapm_activity_delete_cmd *p_param,
                                     ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // List of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_DELETE_ACTIVITY,
                                      GAPM_DELETE_ALL_ACTIVITIES,
                                      GAPM_NO_OP
                                     };
    // Check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_AIR, p_param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        // Error code
        uint8_t error = GAP_ERR_NO_ERROR;
        // Indicate that operation is done
        bool op_done = true;
        // Get indicated activity
        struct gapm_actv_tag *p_actv;

        do
        {
            if (p_param->operation == GAPM_DELETE_ACTIVITY)
            {
                // Check provided activity index
                if (p_param->actv_idx >= GAPM_ACTV_NB)
                {
                    error = GAP_ERR_INVALID_PARAM;
                    break;
                }

                // Get indicated activity
                p_actv = gapm_env.actvs[p_param->actv_idx];

                // If activity does not exist, consider it has been deleted and do not return an error
                if (!p_actv)
                {
                    break;
                }

                // Check that activity has well been stopped
                if (p_actv->state != GAPM_ACTV_CREATED)
                {
                    error = GAP_ERR_COMMAND_DISALLOWED;
                    break;
                }
            }
            else
            {
                // Check there is created activities
                if (!gapm_env.created_actvs)
                {
                    break;
                }

                // Check there is no more started activities
                if (gapm_env.started_actvs)
                {
                    error = GAP_ERR_COMMAND_DISALLOWED;
                    break;
                }

                // Get first created activity
                p_actv = gapm_actv_get_next(0, GAPM_ACTV_CREATED);

                // Sanity check
                ASSERT_ERR(p_actv);

                if (!p_actv)
                {
                    // Running here indicates gapm_env.created_actvs is not 0 but no activity actully.
                    // It must be some error. But if assert check disabled, should set gapm_env.created_actvs = 0
                    // and return completed.
                    gapm_env.created_actvs = 0;
                    break;
                }
            }

            // Operation will be cleared in gapm_actv_deleted function
            op_done = false;

            p_actv->state = GAPM_ACTV_DELETING;

            // Delete activity
            p_actv->cb_actv_delete(p_actv);
        }
        while (0);

        if (op_done)
        {
            // Send the GAPM_CMP_EVT message containing the status
            gapm_send_complete_evt(GAPM_OP_AIR, error);
        }
    }

    return (msg_status);
}

#if (BLE_CENTRAL || BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Handles enhanced connection complete event from the lower layer.
 * This handler is responsible for initiating the creation of L2CAP channel.
 *
 * @param[in] opcode      Operation code of received message
 * @param[in] p_event     Pointer to the parameters of the message.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_enh_con_cmp_evt_handler(uint16_t opcode, struct hci_le_enh_con_cmp_evt const *p_event)
{
    if ((ke_state_get(TASK_GAPM) != GAPM_DEVICE_SETUP) && (p_event->status == CO_ERROR_NO_ERROR))
    {
#if (BLE_CENTRAL)
        uint8_t conidx = GAP_INVALID_CONIDX;

        if (p_event->status == CO_ERROR_NO_ERROR)
        {
            // Connection creation succeeds, inform all layers
            conidx = gapm_con_create(opcode, p_event);
        }

        // If role is master, inform initiating activity
        if (p_event->role == GAPM_ROLE_MASTER)
        {
            gapm_init_connection_ind(conidx);
        }
        // else advertising activity will be stopped upon LE Advertising Set Terminated Event reception
#else //(BLE_CENTRAL)
        if (p_event->status == CO_ERROR_NO_ERROR)
        {
            // Connection creation succeeds, inform all layers
            gapm_con_create(opcode, p_event);
        }
#endif //(BLE_CENTRAL)
    }

    return (KE_MSG_CONSUMED);
}
#endif // (BLE_CENTRAL || BLE_PERIPHERAL)

/// @} GAPM_ACTV