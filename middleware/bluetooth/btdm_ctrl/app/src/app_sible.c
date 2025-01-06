/**
 ****************************************************************************************
 *
 * @file app_sible.c
 *
 * @brief SiFli BLE Application Module Entry point
 *
 * Copyright (C) SiFli 2019-2022
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (BLE_APP_SIBLE)

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "app.h"                     // Application Manager Definitions
#include "app_sible.h"               // SiFli BLE Service Application Definitions
#include "sibles_task.h"             // Sifli BLE Profile Functions
#include "prf_types.h"               // Profile Common Types Definitions
#include "gapm_task.h"               // GAP Manager Task API
#include <string.h>
#include "co_utils.h"
#include "attm.h"
#include "dbg_trc.h"

/// Full DIS Database Description - Used to add attributes into the database
struct attm_desc test_att_db[] =
{
    {ATT_DECL_PRIMARY_SERVICE, PERM(RD, ENABLE), 0, 0},
    {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    {ATT_CHAR_MANUF_NAME, PERM(RD, ENABLE), PERM(RI, ENABLE), 128},
    {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    {ATT_CHAR_MODEL_NB, PERM(RD, ENABLE), PERM(RI, ENABLE), 128},
    {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    {ATT_CHAR_SERIAL_NB, PERM(WRITE_REQ, ENABLE) | PERM(RD, ENABLE), PERM(RI, ENABLE), 128},
    {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    {ATT_CHAR_HW_REV, PERM(RD, ENABLE), PERM(RI, ENABLE), 128},
    {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    {ATT_CHAR_FW_REV, PERM(RD, ENABLE), PERM(RI, ENABLE), 128},
    {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    {ATT_CHAR_SW_REV, PERM(RD, ENABLE), PERM(RI, ENABLE), 128},
    {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    {ATT_CHAR_SYS_ID, PERM(RD, ENABLE), PERM(RI, ENABLE), 8},
    {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    {ATT_CHAR_IEEE_CERTIF, PERM(RD, ENABLE), PERM(RI, ENABLE), 8},
    {ATT_DECL_CHARACTERISTIC, PERM(RD, ENABLE), 0, 0},
    {ATT_CHAR_PNP_ID, PERM(RD, ENABLE), PERM(RI, ENABLE), 7},
    {ATT_DESC_CHAR_USER_DESCRIPTION, PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), PERM(RI, ENABLE), 8},
};

static uint8_t start_hdl;


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

static int sibles_value_req_ind_handler(ke_msg_id_t const msgid,
                                        struct sibles_value_req_ind const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    // Initialize length
    uint8_t len = 0;
    uint8_t idx = 0;

    idx = param->hdl - start_hdl;

    len = test_att_db[idx].max_size;
    // Allocate confirmation to send the value
    struct sibles_value *cfm_value = KE_MSG_ALLOC_DYN(SIBLES_VALUE_REQ_CFM,
                                     src_id, dest_id,
                                     sibles_value,
                                     len);

    // Set parameters
    cfm_value->hdl = param->hdl;
    if (len)
    {
        switch (idx)
        {
        case 2:
            // Copy data
            strcpy((char *)&cfm_value->data[0], "Sifli corporation");
            cfm_value->length = strlen((const char *)&cfm_value->data[0]);
            break;
        }
    }
    // Send message
    ke_msg_send(cfm_value);

    return (KE_MSG_CONSUMED);
}

static int sibles_svc_rsp_handler(ke_msg_id_t const msgid,
                                  struct sibles_svc_rsp const *param,
                                  ke_task_id_t const dest_id,
                                  ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_NO_ERROR)
        start_hdl = param->start_hdl;

#ifdef CFG_PRF_SIBLE_FORWARD
    {
#ifndef PLF_UART2_OVERMBOX
        struct sibles_svc_rsp *rsp = KE_MSG_ALLOC(SIBLES_SVC_RSP, src_id, dest_id, sibles_svc_rsp);
#else
        struct sibles_svc_rsp *rsp = KE_MSG_ALLOC(SIBLES_SVC_RSP, TASK_ID_AHI, dest_id, sibles_svc_rsp);
#endif
        memcpy(rsp, param, sizeof(struct sibles_svc_rsp));
        sifli_mbox_bcpu2hcpu((uint8_t *)rsp);

    }
#endif
    return (KE_MSG_CONSUMED);
}

static int sibles_set_value_rsp_handler(ke_msg_id_t const msgid,
                                        struct sibles_value_ack const *param,
                                        ke_task_id_t const dest_id,
                                        ke_task_id_t const src_id)
{
    if (param->status == ATT_ERR_NO_ERROR)
        TRC_PRINTF("Set %d success\n", param->hdl - start_hdl);
    else
        TRC_PRINTF("Set %d failed, status is 0x%x\n", param->hdl - start_hdl, param->status);
    return (KE_MSG_CONSUMED);
}

static int sibles_value_write_ind_handler(ke_msg_id_t const msgid,
        struct sibles_value_write_ind const *param,
        ke_task_id_t const dest_id,
        ke_task_id_t const src_id)
{
    TRC_PRINTF("write idx:%d, len=%d, offset=%d\n", param->hdl - start_hdl, param->length, param->offset);
    print_data((char *)param->data, 0, param->length);
    struct sibles_value_ack *cfm = KE_MSG_ALLOC(SIBLES_VALUE_WRITE_CFM, KE_BUILD_ID(TASK_PRF_MAX, 0), TASK_APP, sibles_value_ack);
    cfm->hdl = param->hdl;
    cfm->status = ATT_ERR_NO_ERROR;
    ke_msg_send(cfm);
    return (KE_MSG_CONSUMED);
}
/*
 * GLOBAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void app_sible_init(void)
{
    // Nothing to do
}

void app_sible_add_sibles(void)
{
    // Allocate the gapm_profile_task_add_cmd
    struct gapm_profile_task_add_cmd *req = KE_MSG_ALLOC(GAPM_PROFILE_TASK_ADD_CMD,
                                            TASK_GAPM, TASK_APP,
                                            gapm_profile_task_add_cmd);
    // Fill message
    req->operation = GAPM_PROFILE_TASK_ADD;
#if (BLE_APP_AM0)
    req->sec_lvl = PERM(SVC_AUTH, NO_AUTH);
#else
    req->sec_lvl = PERM(SVC_AUTH, NO_AUTH);
//    req->sec_lvl = PERM(SVC_AUTH, AUTH);
#endif //(BLE_APP_AM0)
    req->prf_task_id = TASK_ID_SIBLES;
    req->app_task = TASK_APP;
    req->start_hdl = 0;

    // Send the message
    ke_msg_send(req);
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

#if 0
/// Default State handlers definition
const struct ke_msg_handler app_sible_msg_handler_list[] =
{
    {SIBLES_VALUE_REQ_IND, (ke_msg_func_t)sibles_value_req_ind_handler},
    {SIBLES_SVC_RSP, (ke_msg_func_t)sibles_svc_rsp_handler},
    {SIBLES_SET_VALUE_RSP, (ke_msg_func_t)sibles_set_value_rsp_handler},
    {SIBLES_VALUE_WRITE_IND, (ke_msg_func_t)sibles_value_write_ind_handler},

};

const struct app_subtask_handlers app_sible_handlers = APP_HANDLERS(app_sible);


/**************************** Service commands***************************************************/
static void start_service()
{
    struct sibles_svc_reg_req *req = KE_MSG_ALLOC(SIBLES_SVC_REG_REQ, KE_BUILD_ID(TASK_PRF_MAX, 0), TASK_APP, sibles_svc_reg_req);
    req->svc_uuid = ATT_UUID_16(0x180A);
    req->att_db = test_att_db;
    req->attm_entries = sizeof(test_att_db) / sizeof(struct attm_desc);
    req->sec_lvl = PERM(SVC_AUTH, NO_AUTH);
    ke_msg_send(req);
}

static void set_value(int idx, int len, char *value)
{
    struct sibles_value *req = KE_MSG_ALLOC_DYN(SIBLES_SET_VALUE_REQ, KE_BUILD_ID(TASK_PRF_MAX, 0), TASK_APP, sibles_value, len);
    int val;
    req->hdl = idx + start_hdl;
    req->length = len;
    if (len <= 4)   // Conver to integer
    {
        val = atoi(value);
        value = (char *)&val;
    }
    memcpy(req->data, value, len);
    ke_msg_send(req);
}

static void write_value(int idx, int len, char *value)
{
    struct sibles_value *req = KE_MSG_ALLOC_DYN(SIBLES_VALUE_IND_REQ, KE_BUILD_ID(TASK_PRF_MAX, 0), TASK_APP, sibles_value, len);
    int val;
    req->hdl = idx + start_hdl;
    req->length = len;
    if (len <= 4)   // Conver to integer
    {
        val = atoi(value);
        value = (char *)&val;
    }
    memcpy(req->data, value, len);
    ke_msg_send(req);
}


/********************************************************************************************************/
int cmd_svc(int argc, char **argv)
{
    if (argc > 1)
    {
        if (strcmp(argv[1], "start") == 0)
        {
            start_service();
        }
        if (strcmp(argv[1], "set") == 0)
        {
            set_value(atoi(argv[2]), atoi(argv[3]), argv[4]);
        }
        if (strcmp(argv[1], "write") == 0)
        {
            write_value(atoi(argv[2]), atoi(argv[3]), argv[4]);
        }
    }

    return 0;
}
//FINSH_FUNCTION_EXPORT_ALIAS(cmd_svc, __cmd_svc, BLE service.);
#endif
#endif //BLE_APP_SIBLES

/// @} APP
