/**
 ****************************************************************************************
 *
 * @file ke_task.c
 *
 * @brief This file contains the implementation of the kernel task management.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup TASK
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"       // stack configuration

#include <stddef.h>            // standard definition
#include <stdint.h>            // standard integer
#include <stdbool.h>           // standard boolean
#include <string.h>            // memcpy defintion

#include "ke_task.h"           // kernel task
#include "ke_int.h"            // kernel environment
#include "ke_queue.h"          // kernel queue
#include "ke_event.h"          // kernel event
#include "ke_mem.h"            // kernel memory

#include "dbg_swdiag.h"        // Software diag

#include "dbg.h"
/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * STRUCTURES DEFINTIONS
 ****************************************************************************************
 */

/// KE TASK element structure
struct ke_task_elem
{
    struct ke_task_desc const *p_desc;
};

/// KE TASK environment structure
struct ke_task_env_tag
{
    struct ke_task_elem task_list[TASK_MAX];
};

typedef ke_msg_func_t (*ke_task_handler_get_handler)(ke_msg_id_t const msg_id, ke_task_id_t const task_id);

static ke_msg_func_t _ke_task_handler_get_handler(ke_msg_id_t const msg_id, ke_task_id_t const task_id);


/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// KE TASK environment
__STATIC struct ke_task_env_tag ke_task_env;


ke_task_handler_get_handler ke_task_handler_pre_get = _ke_task_handler_get_handler;


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief This is for patch used
 *
 ****************************************************************************************
*/
static ke_msg_func_t _ke_task_handler_get_handler(ke_msg_id_t const msg_id, ke_task_id_t const task_id)
{
    return NULL;
}


/**
 ****************************************************************************************
 * @brief Compare destination task callback.
 *
 * @param[in] msg          kernel message
 * @param[in] dest_id      destination id
 *
 * @return bool
 ****************************************************************************************
 */
__STATIC bool cmp_dest_id(struct co_list_hdr const *msg, uint32_t dest_id)
{
    return ((struct ke_msg *)msg)->dest_id == dest_id;
}

/**
 ****************************************************************************************
 * @brief Reactivation of saved messages.
 *
 * This primitive looks for all the messages destined to the task ke_task_id that
 * have been saved and inserts them into the sent priority queue. These
 * messages will be scheduled at the next scheduler pass.
 *
 * @param[in] ke_task_id    Destination Identifier
 ****************************************************************************************
 */
__STATIC void ke_task_saved_update(ke_task_id_t const ke_task_id)
{
    for (;;)
    {
        // if the state has changed look in the Save queue if a message
        // need to be handled
        struct ke_msg *msg = (struct ke_msg *) ke_queue_extract(&ke_env.queue_saved,
                             &cmp_dest_id,
                             (uint32_t) ke_task_id);

        if (msg == NULL) break;

        // Insert it back in the sent queue
        GLOBAL_INT_DISABLE();

        if (ke_is_free(msg))
        {
            rt_kprintf("ke_task_saved_update 0x%x, %d, %d, %d\n", (uint8_t *)msg, msg->id, msg->dest_id, msg->src_id);
            ASSERT_INFO_FORCE(0, msg->id, msg->dest_id);
            break;
        }

        ke_queue_push(&ke_env.queue_sent, (struct co_list_hdr *)msg);
        GLOBAL_INT_RESTORE();

        // trigger the event
        ke_event_set(KE_EVENT_KE_MESSAGE);
    }

    return;
}


/**
 ****************************************************************************************
 * @brief Search message handler function matching the msg id
 *
 * @param[in] msg_id          Message identifier
 * @param[in] msg_handler_tab Pointer to the message handlers table
 * @param[in] msg_cnt         Number of message handlers in the table
 *
 * @return                  Pointer to the message handler (NULL if not found)
 *
 ****************************************************************************************
 */


#if 1 // Binary need message handle order as sequence and default handler in the last. For following optimize
__STATIC ke_msg_func_t ke_handler_search(ke_msg_id_t const msg_id, struct ke_msg_handler const *msg_handler_tab, uint16_t msg_cnt)
{
    // Get the message handler function by parsing the message table
    for (int i = (msg_cnt - 1); 0 <= i; i--)
    {
        if ((msg_handler_tab[i].id == msg_id)
                || (msg_handler_tab[i].id == KE_MSG_DEFAULT_HANDLER))
        {
            // If handler is NULL, message should not have been received in this state
            ASSERT_ERR_KE(msg_handler_tab[i].func);

            return msg_handler_tab[i].func;
        }
    }

    // If we execute this line of code, it means that we did not find the handler
    return NULL;
}
#else
__STATIC ke_msg_func_t ke_handler_search(ke_msg_id_t const msg_id, struct ke_msg_handler const *p_msg_handler_tab, uint16_t msg_cnt)
{
    // Message handler to search
    ke_msg_func_t handler = NULL;

    // Left boundary index
    int16_t left_bond = 0;
    // Right boundary index
    int16_t right_bond = (int16_t)(msg_cnt - 1);

    // Get the message handler function by binary searching the message table
    while (left_bond <= right_bond)
    {
        // Middle element index is the floor of (L + R)/2
        uint16_t mid = (left_bond + right_bond) / 2;

        // Index should not exceed table size
        ASSERT_ERR(mid <= msg_cnt - 1);

        // Update left boundary if middle element is smaller than target
        if (p_msg_handler_tab[mid].id < msg_id)
        {
            left_bond = mid + 1;
        }
        // Update right boundary if middle element is larger than target
        else if (p_msg_handler_tab[mid].id > msg_id)
        {
            right_bond = mid - 1;
        }
        else
        {
            // If handler is NULL, message should not have been received in this state
            ASSERT_ERR(p_msg_handler_tab[mid].func);

            handler = p_msg_handler_tab[mid].func;
            break;
        }
    }

    // Can't find the handler message id which is equal to the target
    if (handler == NULL)
    {
        // Return default handler if there is one
        if (p_msg_handler_tab[msg_cnt - 1].id == KE_MSG_DEFAULT_HANDLER)
        {
            ASSERT_ERR(p_msg_handler_tab[msg_cnt - 1].func);

            handler = p_msg_handler_tab[msg_cnt - 1].func;
        }
    }

    return (handler);
}
#endif
/**
 ****************************************************************************************
 * @brief Retrieve appropriate message handler function of a task
 *
 * @param[in]  msg_id   Message identifier
 * @param[in]  task_id  Task instance identifier
 *
 * @return              Pointer to the message handler (NULL if not found)
 *
 ****************************************************************************************
 */
__STATIC ke_msg_func_t ke_task_handler_get(ke_msg_id_t const msg_id, ke_task_id_t const task_id)
{
    DBG_FUNC_ENTER(ke_task_handler_get);
    ke_msg_func_t func = NULL;
    int idx = KE_IDX_GET(task_id);
    int type = KE_TYPE_GET(task_id);
    struct ke_task_desc const *p_task_desc = NULL;

    ASSERT_INFO_KE(type < TASK_MAX, msg_id, type);

    p_task_desc = ke_task_env.task_list[type].p_desc;

    // inactive task have no instance, drop message automatically for inactive tasks
    if (p_task_desc->idx_max != 0)
    {
        ASSERT_INFO_KE(p_task_desc != NULL, task_id, msg_id);
        ASSERT_INFO_KE((idx < p_task_desc->idx_max), task_id, msg_id);

        // If the idx found is out of range return NULL
        if (idx < p_task_desc->idx_max)
        {
            // Search the message handler in the table
            if (p_task_desc->msg_handler_tab)
            {
                func = ke_handler_search(msg_id, p_task_desc->msg_handler_tab, p_task_desc->msg_cnt);
            }
        }
    }

    DBG_FUNC_EXIT(ke_task_handler_get);
    return func;
}


/**
 ****************************************************************************************
 * @brief Scheduler entry point.
 *
 * This function is the scheduler of messages. It tries to get a message
 * from the sent queue, then try to get the appropriate message handler
 * function (from the current state, or the default one). This function
 * is called, then the message is saved or freed.
 ****************************************************************************************
 */
__STATIC void ke_task_schedule(void)
{
    DBG_FUNC_ENTER(ke_task_schedule);
    DBG_SWDIAG(KE, MESSAGE, 1);
    // Process one message at a time to ensure that events having higher priority are
    // handled in time
    do
    {
        ke_msg_id_t msg_id;
        ke_task_id_t src_id, dest_id;
        ke_msg_func_t func;
        int msg_status;
        struct ke_msg *msg;
        // Get a message from the queue
        GLOBAL_INT_DISABLE();
        msg = (struct ke_msg *) ke_queue_pop(&ke_env.queue_sent);
        GLOBAL_INT_RESTORE();
        if (msg == NULL) break;

        // Mark that message is no more in message queue
        msg->hdr.next = KE_MSG_NOT_IN_QUEUE;

        msg_id  = msg->id;
        src_id  = msg->src_id;
        dest_id = msg->dest_id;

        // check if message already free
        if (ke_is_free(msg))
        {
            rt_kprintf("ke_task_schedule 0x%x, %d, %d, %d\n", (uint8_t *)msg, msg->id, msg->dest_id, msg->src_id);
            ASSERT_INFO_FORCE(0, msg_id, dest_id);
            break;
        }

        // Retrieve a pointer to the task instance data
        func = ke_task_handler_pre_get(msg->id, msg->dest_id);
        if (func == NULL)
            func = ke_task_handler_get(msg_id, dest_id);

        // sanity check
        ASSERT_WARN(func != NULL, msg_id, dest_id);

        // Call the message handler
        if (func != NULL)
        {
            DBG_FUNC_EXIT(ke_task_schedule); // special case to not trace function execution
            msg_status = func(msg_id, ke_msg2param(msg), dest_id, src_id);
            DBG_FUNC_ENTER(ke_task_schedule); // special case to not trace function execution
        }
        else
        {
            msg_status = KE_MSG_CONSUMED;
        }

        //Trace handled kernel message
        TRC_REQ_KE_MSG_HANDLED(msg_id, dest_id, src_id, msg_status);

        switch (msg_status)
        {
        case KE_MSG_CONSUMED:
            // Free the message
            ke_msg_free(msg);
            break;

        case KE_MSG_NO_FREE:
            break;

        case KE_MSG_SAVED:
            // The message has been saved
            // Insert it at the end of the save queue

            if (ke_is_free(msg))
            {
                rt_kprintf("KE_MSG_SAVED 0x%x, %d, %d, %d\n", (uint8_t *)msg, msg->id, msg->dest_id, msg->src_id);
                ASSERT_INFO_FORCE(0, msg->id, msg->dest_id);
                break;
            }

            ke_queue_push(&ke_env.queue_saved, (struct co_list_hdr *) msg);
            break;

        default:
            ASSERT_ERR_FORCE(0);
            break;
        } // switch case
    }
    while (0);

    // Verify if we can clear the event bit
    GLOBAL_INT_DISABLE();
    if (co_list_is_empty(&ke_env.queue_sent))
        ke_event_clear(KE_EVENT_KE_MESSAGE);
    GLOBAL_INT_RESTORE();
    DBG_SWDIAG(KE, MESSAGE, 0);
    DBG_FUNC_EXIT(ke_task_schedule);
}


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */


void ke_task_init(void)
{
    memset(&ke_task_env, 0, sizeof(ke_task_env));

    // Register message event
    ke_event_callback_set(KE_EVENT_KE_MESSAGE, &ke_task_schedule);
}

uint8_t ke_task_create(uint8_t task_type, struct ke_task_desc const *p_task_desc)
{
    uint8_t status = KE_TASK_OK;

    GLOBAL_INT_DISABLE();

    do
    {
#if 0
        int16_t i;
        // Pointer to previous handler in the table
        const struct ke_msg_handler *p_prev_handler = &(p_task_desc->msg_handler_tab[0]);
        // Pointer to current handler in the table
        const struct ke_msg_handler *p_cur_handler = &(p_task_desc->msg_handler_tab[1]);

        for (i = p_task_desc->msg_cnt - 2; i >= 0; i--)
        {
            // ID of previous handler should small than ID of current handler as the table is sorted in ascending order
            ASSERT_ERR(p_prev_handler->id < p_cur_handler->id);

            p_prev_handler++;
            p_cur_handler++;
        }
#endif // (RW_DEBUG)

        ASSERT_INFO_KE(task_type < TASK_MAX, task_type, TASK_MAX);

        if (task_type >= TASK_MAX)
        {
            status = KE_TASK_CAPA_EXCEEDED;
            break;
        }

        if (ke_task_env.task_list[task_type].p_desc != NULL)
        {
            status = KE_TASK_ALREADY_EXISTS;
            break;
        }

        // Save task descriptor
        ke_task_env.task_list[task_type].p_desc = p_task_desc;
    }
    while (0);

    GLOBAL_INT_RESTORE();

    return (status);
}

uint8_t ke_task_delete(uint8_t task_type)
{
    uint8_t status = KE_TASK_OK;
    GLOBAL_INT_DISABLE();

    do
    {
        if (task_type >= TASK_MAX)
        {
            status = KE_TASK_UNKNOWN;
            break;
        }

        // Clear task descriptor
        ke_task_env.task_list[task_type].p_desc = NULL;
    }
    while (0);

    GLOBAL_INT_RESTORE();

    return (status);
}


void ke_state_set(ke_task_id_t const id, ke_state_t const state_id)
{
    int idx = KE_IDX_GET(id);
    int type = KE_TYPE_GET(id);
    struct ke_task_desc const *p_task_desc = NULL;
    ke_state_t *ke_stateid_ptr = NULL;

    // sanity checks
    ASSERT_ERR_KE(type < TASK_MAX);

    TRC_REQ_KE_STATE_SET(id, state_id);

    if (type < TASK_MAX)
    {
        p_task_desc = ke_task_env.task_list[type].p_desc;
    }

    if (p_task_desc == NULL)
    {
        ASSERT_INFO_KE(0, type, idx);
        return;
    }

    ASSERT_INFO_KE((idx < p_task_desc->idx_max), idx, p_task_desc->idx_max);
    // If the idx found is out of range return NULL
    if (idx < p_task_desc->idx_max)
    {
        // Get the state
        ke_stateid_ptr = &p_task_desc->state[idx];

        ASSERT_ERR_KE(ke_stateid_ptr);

        // set the state
        if (*ke_stateid_ptr != state_id)
        {
            *ke_stateid_ptr = state_id;

            // if the state has changed update the SAVE queue
            ke_task_saved_update(id);
        }
    }
}


ke_state_t ke_state_get(ke_task_id_t const id)
{
    int idx = KE_IDX_GET(id);
    int type = KE_TYPE_GET(id);
    struct ke_task_desc const *p_task_desc = NULL;
    ke_state_t state = 0xFF;

    ASSERT_ERR_KE(type < TASK_MAX);

    if (type < TASK_MAX)
    {
        p_task_desc = ke_task_env.task_list[type].p_desc;
    }

    if (p_task_desc == NULL)
    {
        ASSERT_INFO_KE(0, type, idx);
        return state;
    }

    ASSERT_INFO_KE((idx < p_task_desc->idx_max), idx, p_task_desc->idx_max);

    // If the idx found is out of range return NULL
    if (idx < p_task_desc->idx_max)
    {
        state = p_task_desc->state[idx];
    }

    // Get the state
    return state;
}

int ke_msg_discard(ke_msg_id_t const msgid, void const *param,
                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    return (KE_MSG_CONSUMED);
}

int ke_msg_save(ke_msg_id_t const msgid, void const *param,
                ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    return KE_MSG_SAVED;
}



void ke_task_msg_flush(ke_task_id_t task)
{
    // Free messages in sent queue
    struct ke_msg *msg = (struct ke_msg *) co_list_pick(&ke_env.queue_sent);
    struct ke_msg *msg_to_delete = NULL;
    bool s_queue_flushed = false;

    while (msg != NULL)
    {
        while (msg != NULL)
        {
            // check if current message have to be handled by task to flush
            if (KE_TYPE_GET(msg->dest_id) == task)
            {
                // update message pointer
                msg_to_delete = msg;
                msg = (struct ke_msg *)msg->hdr.next;

                // delete message
                co_list_extract(&ke_env.queue_sent, &(msg_to_delete->hdr));
                ke_msg_free(msg_to_delete);
            }
            else
            {
                // go to next message
                msg = (struct ke_msg *)msg->hdr.next;
            }
        }

        // Flush messages in save queue
        if (!s_queue_flushed)
        {
            msg = (struct ke_msg *) co_list_pick(&ke_env.queue_saved);
            s_queue_flushed = true;
        }
    }
}



ke_task_id_t ke_task_check(ke_task_id_t task)
{
    uint8_t idx  = KE_IDX_GET(task);
    uint8_t type = KE_TYPE_GET(task);

    // check if task type exist
    if ((type > TASK_MAX)
            || (ke_task_env.task_list[type].p_desc == NULL)
            // check that task instance exist
            || (idx > ke_task_env.task_list[type].p_desc->idx_max))
    {
        task = TASK_NONE;
    }

    return task;
}

/// @} TASK
