/**
 ****************************************************************************************
 *
 * @file gapm.c
 *
 * @brief Generic Access Profile Manager Implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPM Generic Access Profile Manager
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"  // Software configuration

#include "gap.h"          // Generic access profile
#include "gapm.h"         // Generic access profile Manager
#include "gapc.h"         // Generic access profile Controller
#include "../gapc/gapc_int.h" // Internal API required


#include "gapm_int.h"     // Generic access profile Manager Internal

#include "attm.h"         // Attribute Database management

#include "l2cm.h"         // L2CAP Manager API
#include "l2cc_pdu.h"     // L2CAP PDU defines

#include "ke_mem.h"       // Kernel memory management
#include "ke_timer.h"     // Kernel timers

#include "gattc.h"        // Generic Attribute
#include "gattm.h"        // Generic Attribute Manager

#include "co_math.h"      // Mathematic library

#if (BLE_PROFILES)
    #include "prf.h"
#endif // (BLE_PROFILES)

#if (BLE_ISO_MODE_0_PROTOCOL)
    #include "am0_api.h"
#endif // (BLE_ISO_MODE_0_PROTOCOL)

#include "ahi_int.h"


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */



/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * MACROS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct gapm_env_tag gapm_env;


/// GAP Manager task descriptor
extern const struct ke_task_desc TASK_DESC_GAPM;

#if (BLE_ATTS)
/// GAP Attribute database description
static const struct attm_desc gapm_att_db[GAP_IDX_NUMBER] =
{
    // GAP_IDX_PRIM_SVC - GAP service
    [GAP_IDX_PRIM_SVC]                 =   {ATT_DECL_PRIMARY_SERVICE, PERM(RD, ENABLE), 0, 0},
    // GAP_IDX_CHAR_DEVNAME - device name declaration
    [GAP_IDX_CHAR_DEVNAME]             =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // GAP_IDX_DEVNAME - device name definition
    [GAP_IDX_DEVNAME]                  =   {ATT_CHAR_DEVICE_NAME, PERM(RD, ENABLE), PERM(RI, ENABLE), GAP_MAX_NAME_SIZE},
    // GAP_IDX_CHAR_ICON - appearance declaration
    [GAP_IDX_CHAR_ICON]                =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // GAP_IDX_ICON -appearance
    [GAP_IDX_ICON]                     =   {ATT_CHAR_APPEARANCE, PERM(RD, ENABLE), PERM(RI, ENABLE), sizeof(uint16_t)},
    // GAP_IDX_CHAR_SLAVE_PREF_PARAM - Peripheral parameters declaration
    [GAP_IDX_CHAR_SLAVE_PREF_PARAM]    =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // GAP_IDX_SLAVE_PREF_PARAM - Peripheral parameters definition
    [GAP_IDX_SLAVE_PREF_PARAM]         =   {ATT_CHAR_PERIPH_PREF_CON_PARAM, PERM(RD, ENABLE), PERM(RI, ENABLE), sizeof(struct gap_slv_pref)},
    // GAP_IDX_CHAR_ADDR_RESOL - Central Address Resolution declaration
    [GAP_IDX_CHAR_CNT_ADDR_RESOL]      =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // GAP_IDX_ADDR_RESOL_SUPP - Central Address Resolution supported
    [GAP_IDX_CNT_ADDR_RESOL]           =   {ATT_CHAR_CTL_ADDR_RESOL_SUPP, PERM(RD, ENABLE), PERM(RI, ENABLE), sizeof(uint8_t)},
    // GAP_IDX_CHAR_RSLV_PRIV_ADDR_ONLY - Resolvable Private Address Only declaration
    [GAP_IDX_CHAR_RSLV_PRIV_ADDR_ONLY] =   {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    // GAP_IDX_ADDR_RESOL_SUPP - Central Address Resolution supported
    [GAP_IDX_RSLV_PRIV_ADDR_ONLY]      =   {ATT_CHAR_RSLV_PRIV_ADDR_ONLY, PERM(RD, ENABLE), 0, sizeof(uint8_t)},
};
#endif /* (BLE_ATTS) */

static ke_task_id_t g_gapm_default_requester = APP_MAIN_TASK;


/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Cleanup operation
 *
 * @param[in] op_type Operation type.
 ****************************************************************************************
 */
__STATIC void gapm_operation_cleanup(uint8_t op_type)
{
    // check if operation is freed
    if (gapm_env.operation[op_type] != NULL)
    {
        // check if operation is in kernel queue
        if (!ke_msg_in_queue(gapm_env.operation[op_type]))
        {
            // free operation
            ke_msg_free(ke_param2msg(gapm_env.operation[op_type]));
            gapm_env.operation[op_type] = NULL;
        }
    }

    // set operation state to Idle
    gapm_update_state(op_type, false);
}

/*
 * EXPORTED FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

void gapm_init(bool reset)
{
    int i;
    // boot configuration
    if (!reset)
    {
        // Create GAP Manager task
        ke_task_create(TASK_GAPM, &TASK_DESC_GAPM);

        // by default set operations pointer to null
        for (i = 0 ; i < GAPM_OP_MAX ; i++)
        {
            gapm_set_operation_ptr(i, NULL);
        }

#if (BLE_LECB)
        // Initialize list of LE PSM
        co_list_init(&(gapm_env.reg_le_psm));
#endif // (BLE_LECB)
    }

    // Reset activity module
    gapm_actv_init(reset);

    // Reset adress manager module
    gapm_addr_init();

#if (BLE_LECB)
    // remove all registered LE_PSM
    gapm_le_psm_cleanup();
#endif // (BLE_LECB)

    // Profile manager initialization
#if (BLE_PROFILES)
    prf_init(reset);
#endif // (BLE_PROFILES)

    for (i = 0 ; i < GAPM_OP_MAX ; i++)
    {
        gapm_operation_cleanup(i);
    }

#if (BLE_CENTRAL || BLE_PERIPHERAL)
    // Initialize GAP controllers
    gapc_init(reset);
#endif //(BLE_CENTRAL || BLE_PERIPHERAL)

    // clear current role
    gapm_env.role = GAP_ROLE_NONE;
    // all connections reset
    gapm_env.connections = 0;
#if (BLE_LECB)
    gapm_env.nb_lecb     = 0;
#endif // (BLE_LECB)
    /* Set the GAP state to GAP_DEV_SETUP
     * - First time GAP is used, a reset shall be performed to correctly initialize lower
     *   layers configuration. It can be performed automatically if receive a ready event
     *   from lower layers, but environment, since it's not mandatory to trigger this
     *   message, GAP must wait for a reset request before doing anything.
     */
    ke_state_set(TASK_GAPM, GAPM_DEVICE_SETUP);
}

#if (BLE_ATTS)
uint8_t gapm_init_attr(uint16_t start_hdl, uint32_t feat)
{
    /* initialize Start Handle */
    gapm_env.svc_start_hdl = start_hdl;

    /* Create the database */
    return attm_svc_create_db(&(gapm_env.svc_start_hdl), ATT_SVC_GENERIC_ACCESS, (uint8_t *) &feat,
                              GAP_IDX_NUMBER, NULL, TASK_GAPC, &gapm_att_db[0],
                              PERM(SVC_MI, ENABLE));
}

#endif // (BLE_ATTS)



int gapm_process_op(uint8_t op_type, void *op_msg, enum gapm_operation *supp_ops)
{
    ASSERT_ERR(op_type < GAPM_OP_MAX);
    // Returned message status
    int msg_status = KE_MSG_CONSUMED; // Reset State
    // Current process state
    uint8_t state = ke_state_get(TASK_GAPM);
    uint8_t operation = *((uint8_t *)op_msg);

    /* no operation on going or requested operation is current on going operation. */
    if (state != GAPM_DEVICE_SETUP)
    {
        if (gapm_get_operation_ptr(op_type) != op_msg)
        {
            uint8_t status = GAP_ERR_NO_ERROR;

            // check what to do with command if an operation is ongoing.
            if ((state & (1 << op_type)) != GAPM_IDLE)
            {
                // operation are queued by default
                // save it for later.
                msg_status = KE_MSG_SAVED;
            }
            else
            {
                // check if operation is suported
                while (*supp_ops != GAPM_NO_OP)
                {
                    // operation supported by command
                    if (operation == *supp_ops)
                    {
                        break;
                    }
                    // check next operation
                    else
                    {
                        supp_ops++;
                    }
                }

                // operation not supported
                if (*supp_ops == GAPM_NO_OP)
                {
                    status = GAP_ERR_INVALID_PARAM;
                }
                else
                {
                    // message memory will be managed by GAPM
                    msg_status = KE_MSG_NO_FREE;

                    // store operation
                    gapm_set_operation_ptr(op_type, op_msg);
                    // set state to busy
                    gapm_update_state(op_type, true);
                }
            }

            // if an error detected, send command completed with error status
            if (status != GAP_ERR_NO_ERROR)
            {
                gapm_send_error_evt(operation, ke_msg_src_id_get(op_msg), status);
            }
        }
        else
        {
            // message memory managed by GAPM
            msg_status = KE_MSG_NO_FREE;
        }
    }

    return msg_status;
}


uint8_t gapm_get_operation(uint8_t op_type)
{
    // by default no operation
    uint8_t ret = GAPM_NO_OP;

    ASSERT_ERR(op_type < GAPM_OP_MAX);

    // check if an operation is registered
    if (gapm_env.operation[op_type] != NULL)
    {
        // operation code if first by of an operation command
        ret = (*((uint8_t *) gapm_env.operation[op_type]));
    }

    return ret;
}


ke_task_id_t gapm_get_requester(uint8_t op_type)
{
    ke_task_id_t ret = 0;
    ASSERT_ERR(op_type < GAPM_OP_MAX);

    // check if an operation is registered
    if (gapm_env.operation[op_type] != NULL)
    {
        // retrieve operation requester.
        ret = ke_msg_src_id_get(gapm_env.operation[op_type]);
    }

    return ret;
}

ke_task_id_t gapm_get_default_requester(void)
{
    return g_gapm_default_requester;
}

int gapm_set_default_requester(ke_msg_id_t const msgid, void const *param,
                               ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    g_gapm_default_requester = KE_TYPE_GET(src_id);
    return KE_MSG_CONSUMED;
}



void gapm_send_complete_evt(uint8_t op_type, uint8_t status)
{
    uint8_t *op_cmd = (uint8_t *) gapm_get_operation_ptr(op_type);

    // check that operation is valid
    if (op_cmd != NULL)
    {
        // prepare command completed event
        struct gapm_cmp_evt *cmp_evt = KE_MSG_ALLOC(GAPM_CMP_EVT,
                                       gapm_get_requester(op_type), TASK_GAPM, gapm_cmp_evt);

        cmp_evt->operation = *op_cmd;
        cmp_evt->status = status;

        // send event
        ke_msg_send(cmp_evt);
    }

    if (op_type == GAPM_OP_AIR)
    {
        // Clear stored operation index
        gapm_env.actv_idx = GAPM_ACTV_INVALID_IDX;
    }

    // cleanup operation
    gapm_operation_cleanup(op_type);
}

void gapm_send_error_evt(uint8_t operation, const ke_task_id_t requester, uint8_t status)
{
    // check that requester value is valid
    if (requester != 0)
    {
        // prepare command completed event with error status
        struct gapm_cmp_evt *cmp_evt = KE_MSG_ALLOC(GAPM_CMP_EVT,
                                       requester, TASK_GAPM, gapm_cmp_evt);

        cmp_evt->operation = operation;
        cmp_evt->status = status;

        // send event
        ke_msg_send(cmp_evt);
    }
}

#if (BLE_CENTRAL || BLE_PERIPHERAL)
uint8_t gapm_con_create(ke_msg_id_t const msgid, struct hci_le_enh_con_cmp_evt const *con_params)
{
    // First create GAP controller task (that send indication message)
    uint8_t conidx;
    // Local Address
    bd_addr_t *p_loc_addr = &gapm_env.addr;
    // Local Address type
    uint8_t loc_addr_type = gapm_get_address_type();

    // Task at which GAPM_CONNECTION_REQ_IND message is intended to be sent
    ke_task_id_t requester = gapm_get_default_requester();
    //ke_task_id_t requester = TASK_AHI;

    loc_addr_type = ((loc_addr_type & GAPM_PRIV_CFG_PRIV_ADDR_BIT) ? ADDR_RAND : ADDR_PUBLIC);

#if (BLE_CENTRAL)
    if (con_params->role == GAPM_ROLE_MASTER)
    {
        // Look for started initiating activity
        struct gapm_actv_init_tag *p_actv_init
            = (struct gapm_actv_init_tag *)gapm_env.actvs[gapm_env.init_actv_idx];

        // Sanity check
        ASSERT_ERR(p_actv_init);

        // If name discovery has been required, connection is handled by GAPM
        requester = (p_actv_init->common.subtype == GAPM_INIT_TYPE_NAME_DISC)
                    ? TASK_GAPM : p_actv_init->common.requester;

        if (p_actv_init->common.own_addr_type != GAPM_STATIC_ADDR)
        {
            p_loc_addr = &p_actv_init->common.addr;
            loc_addr_type = ADDR_RAND;
        }
    }
#endif //(BLE_CENTRAL)

    conidx = gapc_con_create(msgid, con_params, requester, p_loc_addr, loc_addr_type);

    // check if an error occurs.
    if (conidx != GAP_INVALID_CONIDX)
    {
        // Increment number of connections.
        gapm_env.connections++;

        /* ******** Inform other tasks that connection has been established. ******** */

        // Inform L2CAP about new connection
        l2cm_create(conidx);
        // Inform GATT about new connection
        gattm_create(conidx);


#if (BLE_PROFILES)
        // Inform profiles about new connection
        prf_create(conidx);
#endif /* (BLE_PROFILES) */


#if (BLE_ISO_MODE_0_PROTOCOL)
        // Inform Audio Mode 0 task that new connection is created
        am0_task_create(conidx);
#endif // (BLE_ISO_MODE_0_PROTOCOL)
    }

    return conidx;
}


void gapm_con_enable(uint8_t conidx)
{
    // sanity check.
    if (conidx != GAP_INVALID_CONIDX)
    {
        // Inform ATT that connection information has be set
        gattc_con_enable(conidx);
    }
}

void gapm_con_cleanup(uint8_t conidx, uint16_t conhdl, uint8_t reason)
{
    // check if an error occurs.
    if (conidx != GAP_INVALID_CONIDX)
    {
        // Decrement number of connections.
        gapm_env.connections--;

        /* ******** Inform other tasks that connection has been disconnected. ******** */

#if (BLE_ISO_MODE_0_PROTOCOL)
        // Inform Audio Mode 0 task about terminated connection
        am0_task_cleanup(conidx);
#endif // (BLE_ISO_MODE_0_PROTOCOL)

        // Inform GAPC about terminated connection
        gapc_con_cleanup(conidx);

        // Inform L2CAP about terminated connection
        l2cm_cleanup(conidx);
        // Inform GATT about terminated connection
        gattm_cleanup(conidx);

#if (BLE_PROFILES)
        // Inform profiles about terminated connection
        prf_cleanup(conidx, reason);
#endif /* (BLE_PROFILES) */
    }
}
#endif /* (BLE_CENTRAL || BLE_PERIPHERAL) */

ke_task_id_t gapm_get_id_from_task(ke_msg_id_t task)
{
    ke_task_id_t id = TASK_ID_INVALID;
    uint8_t idx = KE_IDX_GET(task);
    task = KE_TYPE_GET(task);

    switch (task)
    {
    case TASK_GAPM:
        id = TASK_ID_GAPM;
        break;
    case TASK_GAPC:
        id = TASK_ID_GAPC;
        break;
    case TASK_GATTM:
        id = TASK_ID_GATTM;
        break;
    case TASK_GATTC:
        id = TASK_ID_GATTC;
        break;
    case TASK_L2CC:
        id = TASK_ID_L2CC;
        break;
#if (AHI_TL_SUPPORT)
    case TASK_AHI:
        id = TASK_ID_AHI;
        break;
#endif // (AHI_TL_SUPPORT)
#if (AHI_INT_SUPPORT)
    case TASK_AHI_INT:
        id = TASK_ID_AHI_INT;
        break;
#endif // (AHI_TL_SUPPORT)
#if (BLE_ISO_MODE_0_PROTOCOL)
    case TASK_AM0:
        id = TASK_ID_AM0;
        break;
#endif // (BLE_ISO_MODE_0_PROTOCOL)
    default:
    {
#if (BLE_PROFILES)
        // check if profile manager is able to retrieve the task id
        id = prf_get_id_from_task(task);
#endif // (BLE_PROFILES)
    }
    break;
    }

    return KE_BUILD_ID(id, idx);
}

ke_task_id_t gapm_get_task_from_id(ke_msg_id_t id)
{
    ke_task_id_t task = TASK_NONE;
    uint8_t idx = KE_IDX_GET(id);
    id = KE_TYPE_GET(id);

    switch (id)
    {
    case TASK_ID_GAPM:
        task = TASK_GAPM;
        break;
    case TASK_ID_GAPC:
        task = TASK_GAPC;
        break;
    case TASK_ID_GATTM:
        task = TASK_GATTM;
        break;
    case TASK_ID_GATTC:
        task = TASK_GATTC;
        break;
    case TASK_ID_L2CC:
        task = TASK_L2CC;
        break;
#if (AHI_TL_SUPPORT)
    case TASK_ID_AHI:
        task = TASK_AHI;
        break;
#endif // (AHI_TL_SUPPORT)
#if (AHI_INT_SUPPORT)
    case TASK_ID_AHI_INT:
        task = TASK_AHI_INT;
        break;
#endif // (AHI_TL_SUPPORT)
#if (BLE_APP_PRESENT)
    case TASK_ID_APP:
        task = TASK_APP;
        break;
#endif // (AHI_TL_SUPPORT)
#if (BLE_ISO_MODE_0_PROTOCOL)
    case TASK_ID_AM0:
        task = TASK_AM0;
        break;
#endif // (BLE_ISO_MODE_0_PROTOCOL)
    case TASK_ID_COMMON:
        task = TASK_COMMON;
        break;
    default:
    {
#if (BLE_PROFILES)
        // check if profile manager is able to retrieve the task number
        task = prf_get_task_from_id(id);
#endif // (BLE_PROFILES)
    }
    break;
    }

    return (ke_task_check(KE_BUILD_ID(task, idx)));
};


ke_task_id_t gapm_get_AHI_task_id(void)
{
    // If no task select, always send to GAPM to avoid assert.
    ke_task_id_t id = TASK_GAPM;
    do
    {
#if (AHI_INT_SUPPORT)
        if (ahi_int_is_ready())
        {
            id = TASK_AHI_INT;
            break;
        }
#endif

#if (AHI_TL_SUPPORT)
        id = TASK_AHI;
#endif
    }
    while (0);
    return id;
}

#if (BLE_CENTRAL || BLE_PERIPHERAL)


uint16_t gapm_get_max_mtu(void)
{
    return gapm_env.max_mtu;
}

uint16_t gapm_get_max_mps(void)
{
    return gapm_env.max_mps;
}

void gapm_set_max_mtu(uint16_t mtu)
{
    // The MTU value must be within the range [L2C_MIN_LE_MTUSIG:L2C_MIN_LE_MTUSIG]
    gapm_env.max_mtu = co_max(L2C_MIN_LE_MTUSIG, co_min(mtu, GAP_MAX_LE_MTU));
}

void gapm_set_max_mps(uint16_t mps)
{
    // The MPS value must be within the range [L2C_MIN_LE_MTUSIG: MTU]
    gapm_env.max_mps = co_max(L2C_MIN_LE_MTUSIG, co_min(mps, gapm_env.max_mtu));
}
#endif /* (BLE_CENTRAL || BLE_PERIPHERAL) */

uint8_t gapm_dle_val_check(uint16_t sugg_oct, uint16_t sugg_time)
{
    uint8_t status = GAP_ERR_INVALID_PARAM;

    // Check suggested Data Length parameters
    if ((sugg_oct >= LE_MIN_OCTETS) &&
            (sugg_oct <= LE_MAX_OCTETS) &&
            (sugg_time >= LE_MIN_TIME) &&
            (sugg_time <= LE_MAX_TIME))
    {
        status = GAP_ERR_NO_ERROR;
    }

    return status;
}

#if (SECURE_CONNECTIONS)
public_key_t *gapm_get_local_public_key(void)
{
    // Returns the local Public Key - X and Y coordinates.
    return &gapm_env.public_key;
}
#endif // (SECURE_CONNECTIONS)


void gapm_update_state(uint8_t operation, bool busy)
{
    ke_state_t old_state  = ke_state_get(TASK_GAPM);
    if (old_state != GAPM_DEVICE_SETUP)
    {
        if (busy)
        {
            // set state to busy
            ke_state_set(TASK_GAPM, old_state | (1 << operation));
        }
        else
        {
            // set state to idle
            ke_state_set(TASK_GAPM, old_state & ~(1 << operation));
        }
    }
}

bool gapm_is_legacy_pairing_supp(void)
{
    return ((gapm_env.pairing_mode & GAPM_PAIRING_LEGACY) != 0);
}


bool gapm_is_sec_con_pairing_supp(void)
{
    return ((gapm_env.pairing_mode & GAPM_PAIRING_SEC_CON) != 0);
}

#if (BLE_ISO_MODE_0_PROTOCOL)
bool gapm_is_audio_am0_sup(void)
{
    return GAPM_F_GET(gapm_env.audio_cfg, AUDIO_AM0_SUP) != 0;
}
#endif // (BLE_ISO_MODE_0_PROTOCOL)



#if (BLE_LECB)

struct gapm_le_psm_info *gapm_le_psm_find(uint16_t le_psm)
{
    // search if LE_PSM is present.
    struct gapm_le_psm_info *info = (struct gapm_le_psm_info *) co_list_pick(&(gapm_env.reg_le_psm));

    // browse register le_psm list
    while (info)
    {
        // check if LE_PSM already registered
        if (info->le_psm == le_psm)
        {
            break;
        }

        // go to next element
        info = (struct gapm_le_psm_info *) info->hdr.next;
    }

    return info;
}


uint8_t gapm_le_psm_get_info(uint16_t le_psm, uint8_t conidx, ke_task_id_t *app_task, uint8_t *sec_lvl)
{
    uint8_t status = GAP_ERR_NOT_FOUND;
    // search if LE_PSM is present.
    struct gapm_le_psm_info *info = gapm_le_psm_find(le_psm);

    if (info != NULL)
    {
        *sec_lvl  = (info->sec_lvl & GAPM_LE_PSM_SEC_LVL_MASK);
        *app_task = info->task_id;

        // check if target task is multi-instantiated
        if (info->sec_lvl & GAPM_LE_PSM_MI_TASK_MASK)
        {
            *app_task = KE_BUILD_ID(KE_TYPE_GET(info->task_id), conidx);
        }

        status = GAP_ERR_NO_ERROR;
    }

    return status;
}


uint8_t gapm_lecb_register(uint16_t le_psm, bool peer_con_init)
{
    uint8_t status = L2C_ERR_NO_RES_AVAIL;

    // check that resources are available
    if (gapm_env.nb_lecb < gapm_env.max_nb_lecb)
    {
        status = GAP_ERR_NO_ERROR;
        gapm_env.nb_lecb++;

        // increment number of connection for registerd LE_PSM
        if (peer_con_init)
        {
            struct gapm_le_psm_info *info = gapm_le_psm_find(le_psm);

            if (info != NULL)
            {
                info->nb_est_lk++;
            }
        }
    }

    return (status);
}


uint8_t gapm_lecb_unregister(uint16_t le_psm, bool peer_con_init)
{
    // decrement total number of connection
    gapm_env.nb_lecb--;

    // decrement number of connection for registerd LE_PSM
    if (peer_con_init)
    {
        struct gapm_le_psm_info *info = gapm_le_psm_find(le_psm);

        if (info != NULL)
        {
            info->nb_est_lk--;
        }
    }

    return (GAP_ERR_NO_ERROR);
}


// remove all registered LE_PSM
void gapm_le_psm_cleanup(void)
{
    while (!co_list_is_empty(&(gapm_env.reg_le_psm)))
    {
        struct co_list_hdr *hdr = co_list_pop_front(&(gapm_env.reg_le_psm));
        ke_free(hdr);
    }
}
#endif // (BLE_LECB)

uint8_t gapm_get_address_type(void)
{
    return (uint8_t) GAPM_F_GET(gapm_env.cfg_flags, ADDR_TYPE);
}

#if (BLE_ATTS)

bool gapm_is_pref_con_param_pres(void)
{
    return (bool) GAPM_F_GET(gapm_env.cfg_flags, PREF_CON_PAR_PRES);
}

uint16_t gapm_get_att_handle(uint8_t att_idx)
{
    uint16_t handle = ATT_INVALID_HANDLE;

    if (gapm_env.svc_start_hdl != 0)
    {
        handle = gapm_env.svc_start_hdl + att_idx;

        if ((att_idx > GAP_IDX_SLAVE_PREF_PARAM) && !GAPM_F_GET(gapm_env.cfg_flags, PREF_CON_PAR_PRES))
        {
            handle -= 2;
        }
    }

    return handle;
}
#endif // (BLE_ATTS)


bool gapm_svc_chg_en(void)
{
    return (GAPM_F_GET(gapm_env.cfg_flags, SVC_CHG_EN) != 0);
}

#if (RW_DEBUG)
bool gapm_dbg_mode_en(void)
{
    return (GAPM_F_GET(gapm_env.dbg_cfg, DBG_MODE_EN) != 0);
}

bool gapm_dbg_fwd_traffic(void)
{
    return (GAPM_F_GET(gapm_env.dbg_cfg, DBG_L2CAP_TRAFFIC_FWD_EN) != 0);
}

void gapm_set_svc_start_hdl(uint16_t start_hdl)
{
#if (BLE_ATTS)
    gapm_env.svc_start_hdl = start_hdl;
#endif //(BLE_ATTS)
}
#endif // (RW_DEBUG)


struct gap_sec_key *gapm_get_irk(void)
{
    return &(gapm_env.irk);
}


bd_addr_t *gapm_get_bdaddr(void)
{
    return &(gapm_env.addr);
}


/// @} GAPM
