/**
 ****************************************************************************************
 *
 * @file ke_event.c
 *
 * @brief This file contains the event handling primitives.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup EVT
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"  // stack configuration

#include "arch.h"         // architecture
#include "co_math.h"      // maths definitions
#include <stdint.h>       // standard integer definition
#include <stddef.h>       // standard definition
#include <string.h>       // memcpy defintion

#include "ke_event.h"     // kernel event

#include "dbg.h"

/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Format of an event callback function
typedef void (*p_callback_t)(void);



/*
 * STRUCTURES DEFINTIONS
 ****************************************************************************************
 */

/// KE EVENT environment structure
struct ke_event_env_tag
{
    /// Event field
    uint32_t event_field;
    uint32_t event_mask[KE_EVNET_PRI_MAX];

    /// Callback table
    p_callback_t callback[KE_EVENT_MAX];
    void *task_handle[KE_EVNET_PRI_MAX];
    void *sema[KE_EVNET_PRI_MAX];
    uint8_t priority[KE_EVENT_MAX];
};

typedef struct
{
    uint8_t event;
    uint8_t event_pri;
} ke_event_evt_pri_mapping_t;

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */

/// KE EVENT environment
__STATIC struct ke_event_env_tag ke_event_env;

const static ke_event_evt_pri_mapping_t ke_event_pri_mapping[KE_EVENT_MAX] =
{
#if(APP_PRESENT)
    {APP_EVENT, KE_EVNET_PRI_HIGH},
#endif // (APP_PRESENT)

#if (TRACER_PRESENT)
    {KE_EVENT_TRC, KE_EVENT_PRI_LOW},               //!< KE_EVENT_TRC
#endif /*(TRACER_PRESENT)*/

    //{KE_EVENT_SHELL, KE_EVNET_PRI_HIGH},

#if DISPLAY_SUPPORT
    {KE_EVENT_DISPLAY, KE_EVNET_PRI_HIGH},           //!< KE_EVENT_DISPLAY
#endif //DISPLAY_SUPPORT

#if RTC_SUPPORT
    {KE_EVENT_RTC_1S_TICK, KE_EVNET_PRI_HIGH},
#endif //RTC_SUPPORT

#if BLE_RSA
    {KE_EVENT_RSA_SIGN, KE_EVNET_PRI_HIGH},
#endif //BLE_RSA


    // {KE_EVENT_ECC_MULTIPLICATION, KE_EVNET_PRI_HIGH},//!< KE_EVENT_ECC_MULTIPLICATION


#if BT_EMB_PRESENT
    // {KE_EVENT_P192_PUB_KEY_GEN, KE_EVNET_PRI_HIGH},//!< KE_EVENT_P192_PUB_KEY_GEN
#endif // BT_EMB_PRESENT

#if (BLE_MESH)
    // {KE_EVENT_BLE_MESH_DJOB, KE_EVNET_PRI_HIGH},
#endif // (BLE_MESH)

#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    {KE_EVENT_AES_END, KE_EVNET_PRI_HIGH},           //!< KE_EVENT_AES_END
#endif // (BLE_EMB_PRESENT || BT_EMB_PRESENT)

#if defined(CFG_SLIM) || !(EMB_PRESENT)
    {KE_EVENT_DJOB_LP, KE_EVENT_PRI_LOW},
#else
    {KE_EVENT_DJOB_LP, KE_EVENT_PRI_LOW},
#endif
    {KE_EVENT_KE_MESSAGE, KE_EVNET_PRI_HIGH},        //!< KE_EVENT_KE_MESSAGE
    {KE_EVENT_DJOB_HP, KE_EVNET_PRI_HIGH},

#if H4TL_SUPPORT
    //{KE_EVENT_H4TL_TX, KE_EVNET_PRI_HIGH},           //!< KE_EVENT_H4TL_TX
    //{KE_EVENT_H4TL_RX, KE_EVNET_PRI_HIGH},           //!< KE_EVENT_H4TL_RX
#if (BLE_EMB_PRESENT || BT_EMB_PRESENT)
    //{KE_EVENT_H4TL_CMD_HDR_RX, KE_EVNET_PRI_HIGH},   //!< KE_EVENT_H4TL_CMD_HDR_RX
    //{KE_EVENT_H4TL_CMD_PLD_RX, KE_EVNET_PRI_HIGH},  //!< KE_EVENT_H4TL_CMD_PLD_RX
#endif //(BLE_EMB_PRESENT || BT_EMB_PRESENT)
#if (((BLE_EMB_PRESENT || BLE_HOST_PRESENT) && (BLE_CENTRAL || BLE_PERIPHERAL)) || BT_EMB_PRESENT)
    //{KE_EVENT_H4TL_ACL_HDR_RX, KE_EVNET_PRI_HIGH},  //!< KE_EVENT_H4TL_ACL_HDR_RX
#endif //(((BLE_EMB_PRESENT || BLE_HOST_PRESENT) && (BLE_CENTRAL || BLE_PERIPHERAL)) || BT_EMB_PRESENT)
#endif //H4TL_SUPPORT

#if (BLE_HOST_PRESENT)
#if (BLE_L2CC)
    {KE_EVENT_L2CAP_TX, KE_EVNET_PRI_HIGH},
#endif //(BLE_L2CC)
#endif// (BLE_HOST_PRESENT)


    {KE_EVENT_TIMER, KE_EVNET_PRI_HIGH},
#if (BLE_ISOOHCI)
    {KE_EVENT_ISOOHCI_IN_DEFER, KE_EVNET_PRI_HIGH}, //!< KE_EVENT_ISOOHCI_IN_DEFER
    {KE_EVENT_ISOOHCI_OUT_DEFER, KE_EVNET_PRI_HIGH}, //!< KE_EVENT_ISOOHCI_OUT_DEFER
#endif //(BLE_ISOOHCI)
    {KE_EVENT_DJOB_ISR, KE_EVNET_PRI_HIGHEST},
};

static uint8_t _ke_evt_get_priority(uint8_t event_type);
static void _ke_evt_task_entry(void *param);

ke_evt_get_priority_handler _ke_evt_get_priority_handler = _ke_evt_get_priority;
ke_evt_task_entry_handler _ke_evt_task_entry_handler = _ke_evt_task_entry;
ke_event_init_handler _ke_event_init_handler = _ke_event_init;
ke_event_callback_set_handler _ke_event_callback_set_handler = _ke_event_callback_set;
ke_event_set_handler _ke_event_set_handler = _ke_event_set;
ke_event_clear_handler _ke_event_clear_handler = _ke_event_clear;
ke_event_get_handler _ke_event_get_handler = _ke_event_get;
ke_event_get_all_handler _ke_event_get_all_handler = _ke_event_get_all;
ke_event_flush_handler _ke_event_flush_handler = _ke_event_flush;
ke_event_schedule_handler _ke_event_schedule_handler = _ke_event_schedule;
ke_event_schedule_via_evt_mask_handler _ke_event_schedule_via_evt_mask_handler = _ke_event_schedule_via_evt_mask;

#define ke_evt_get_priority(event_type)  _ke_evt_get_priority_handler(event_type)
#define ke_evt_task_entry(param)   _ke_evt_task_entry_handler(param)

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

static uint8_t _ke_evt_get_priority(uint8_t event_type)
{
    uint8_t priority = KE_EVENT_PRI_LOW;
    uint8_t i;
    if (event_type < KE_EVENT_MAX)
    {
        // should be the same.
        //ASSERT_INFO(event_type == ke_event_pri_mapping[event_type].event, event_type, 0);
        //priority = ke_event_pri_mapping[event_type].event_pri;
        for (i = 0; i < KE_EVENT_MAX; i++)
        {
            if (event_type == ke_event_pri_mapping[i].event)
            {
                priority = ke_event_pri_mapping[i].event_pri;
            }
        }
    }
    return priority;
}

static void _ke_evt_task_entry(void *param)
{
    uint8_t priority = (uint8_t)(uint32_t)param;

    while (1)
    {
        rw_os_sem_get(ke_event_env.sema[priority]);
        ke_event_schedule_via_evt_mask(ke_event_env.event_mask[priority]);

        //ke_event_env.callback[event_type]();
    }

}



/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */


void _ke_event_init(void)
{
    memset(&ke_event_env, 0, sizeof(ke_event_env));
    DBG_MEM_PERM_SET(&ke_event_env.callback, sizeof(ke_event_env.callback), false, true, true);
}

uint8_t _ke_event_callback_set(uint8_t event_type, void (*p_callback)(void))
{
    uint8_t status = KE_EVENT_CAPA_EXCEEDED;
    uint8_t priority, evt_priority;

    ASSERT_INFO_KE((event_type < KE_EVENT_MAX) && (p_callback != NULL), event_type, p_callback);

    if (event_type < KE_EVENT_MAX)
    {
        DBG_MEM_GRANT_CTRL(&ke_event_env, true);
        // Store callback
        ke_event_env.callback[event_type] = p_callback;
        DBG_MEM_GRANT_CTRL(&ke_event_env, false);
        evt_priority = ke_event_env.priority[event_type] = ke_evt_get_priority(event_type);
        ke_event_env.event_mask[evt_priority] |= 1 << event_type;
        if (NULL == ke_event_env.task_handle[evt_priority])
        {
            //TODO: need to check thread priority
            //priority = KE_EVT_TASK_PRIOIRTY + KE_EVNET_PRI_MAX - evt_priority;
            if (NULL == ke_event_env.task_handle[evt_priority])
            {
                ke_event_env.sema[evt_priority] = rw_os_sem_create((int8_t *)"KE", 0);
                void *task_handle = rw_os_thread_create((int8_t *)"KE_EVT", (void *)_ke_evt_task_entry_handler, (void *)(uint32_t)evt_priority, evt_priority);
                ke_event_env.task_handle[evt_priority] = task_handle;
            }
        }
        // Status OK
        status = KE_EVENT_OK;
    }

    return (status);
}

void _ke_event_set(uint8_t event_type)
{
    ASSERT_INFO_KE((event_type < KE_EVENT_MAX), event_type, 0);
    uint8_t priority = ke_event_env.priority[event_type];
    GLOBAL_INT_DISABLE();

    if (event_type < KE_EVENT_MAX)
    {
        // Set the event in the bit field
        ke_event_env.event_field |= (1 << event_type);
    }

    GLOBAL_INT_RESTORE();

    rw_os_sem_set(ke_event_env.sema[priority]);

    //Trace the event
    TRC_REQ_KE_EVT_SET(event_type);
}

void _ke_event_clear(uint8_t event_type)
{
    ASSERT_INFO_KE((event_type < KE_EVENT_MAX), event_type, 0);

    GLOBAL_INT_DISABLE();

    if (event_type < KE_EVENT_MAX)
    {
        // Set the event in the bit field
        ke_event_env.event_field &= ~(1 << event_type);
    }

    GLOBAL_INT_RESTORE();
}

uint8_t _ke_event_get(uint8_t event_type)
{
    uint8_t state = 0;

    ASSERT_INFO_KE((event_type < KE_EVENT_MAX), event_type, 0);

    GLOBAL_INT_DISABLE();

    if (event_type < KE_EVENT_MAX)
    {
        // Get the event in the bit field
        state = (ke_event_env.event_field >> event_type) & (0x1);
    }

    GLOBAL_INT_RESTORE();

    return state;
}

uint32_t _ke_event_get_all(void)
{
    return ke_event_env.event_field;
}

void _ke_event_flush(void)
{
    ke_event_env.event_field = 0;
}

void _ke_event_schedule(void)
{
    DBG_FUNC_ENTER(ke_event_schedule);
    // Get the volatile value
    uint32_t field = ke_event_env.event_field;

    while (field) // Compiler is assumed to optimize with loop inversion
    {
        // Find highest priority event set
        uint8_t hdl = 32 - (uint8_t) co_clz(field) - 1;

        // Sanity check
        ASSERT_INFO_KE(hdl < KE_EVENT_MAX, hdl, field);

        //Trace the event
        TRC_REQ_KE_EVT_HANDLED(hdl);

        if (ke_event_env.callback[hdl] != NULL)
        {
            DBG_FUNC_EXIT(ke_event_schedule); // Here just for duration measurement
            // Execute corresponding handler
            (ke_event_env.callback[hdl])();
            DBG_FUNC_ENTER(ke_event_schedule); // Here just for duration measurement
        }
        else
        {
            ASSERT_INFO_FORCE(0, hdl, field);
        }

        // Update the volatile value
        field = ke_event_env.event_field;
    }
    DBG_FUNC_EXIT(ke_event_schedule);
}


void _ke_event_schedule_via_evt_mask(uint32_t evt_mask)
{
    uint8_t hdl;

    // Get the volatile value
    uint32_t field = ke_event_env.event_field & evt_mask;

    while (field) // Compiler is assumed to optimize with loop inversion
    {
        // Find highest priority event set
        hdl = 32 - (uint8_t) co_clz(field) - 1;

        // Sanity check
        ASSERT_INFO_KE(hdl < KE_EVENT_MAX, hdl, field);

        //Trace the event
        TRC_REQ_KE_EVT_HANDLED(hdl);

        if (ke_event_env.callback[hdl] != NULL)
        {
            // Execute corresponding handler
            (ke_event_env.callback[hdl])();
        }
        else
        {
            ASSERT_INFO_FORCE(0, hdl, field);
        }

        // Update the volatile value
        field = ke_event_env.event_field & evt_mask;
    }
}


///@} KE_EVT
