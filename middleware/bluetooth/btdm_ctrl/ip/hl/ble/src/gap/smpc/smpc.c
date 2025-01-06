/**
 ****************************************************************************************
 *
 * @file smpc.c
 *
 * @brief SMP Controller implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup SMPC
 * @ingroup SMP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#if (BLE_SMPC)

#include <string.h>

#include "ke_mem.h"
#include "ke_timer.h"           // Kernel Timer Functions

#include "gap.h"
#include "gapc.h"

#include "../gapc/gapc_int.h"  // Internal are required


#include "gapm.h"
#include "gapm_task.h"

#include "smpc_int.h"
#include "smpc_util.h"
#include "smpc_crypto.h"

#include "l2cc.h"

#include "hci.h"

#include "co_utils.h"
#include "co_error.h"

/*
 * PRIVATE VARIABLE DEFINITIONS
 ****************************************************************************************
 */



/// SMPC table for pairing method used for different IO capabilities, 1st idx = R, 2nd = I
static const uint8_t smpc_pair_method[GAP_IO_CAP_LAST][GAP_IO_CAP_LAST] =
{
    [GAP_IO_CAP_DISPLAY_ONLY][GAP_IO_CAP_DISPLAY_ONLY]             = SMPC_METH_JW,
    [GAP_IO_CAP_DISPLAY_ONLY][GAP_IO_CAP_DISPLAY_YES_NO]           = SMPC_METH_JW,
    [GAP_IO_CAP_DISPLAY_ONLY][GAP_IO_CAP_KB_ONLY]                  = SMPC_METH_PK,
    [GAP_IO_CAP_DISPLAY_ONLY][GAP_IO_CAP_NO_INPUT_NO_OUTPUT]       = SMPC_METH_JW,
    [GAP_IO_CAP_DISPLAY_ONLY][GAP_IO_CAP_KB_DISPLAY]               = SMPC_METH_PK,

    [GAP_IO_CAP_DISPLAY_YES_NO][GAP_IO_CAP_DISPLAY_ONLY]           = SMPC_METH_JW,
    [GAP_IO_CAP_DISPLAY_YES_NO][GAP_IO_CAP_DISPLAY_YES_NO]         = SMPC_METH_JW,
    [GAP_IO_CAP_DISPLAY_YES_NO][GAP_IO_CAP_KB_ONLY]                = SMPC_METH_PK,
    [GAP_IO_CAP_DISPLAY_YES_NO][GAP_IO_CAP_NO_INPUT_NO_OUTPUT]     = SMPC_METH_JW,
    [GAP_IO_CAP_DISPLAY_YES_NO][GAP_IO_CAP_KB_DISPLAY]             = SMPC_METH_PK,

    [GAP_IO_CAP_KB_ONLY][GAP_IO_CAP_DISPLAY_ONLY]                  = SMPC_METH_PK,
    [GAP_IO_CAP_KB_ONLY][GAP_IO_CAP_DISPLAY_YES_NO]                = SMPC_METH_PK,
    [GAP_IO_CAP_KB_ONLY][GAP_IO_CAP_KB_ONLY]                       = SMPC_METH_PK,
    [GAP_IO_CAP_KB_ONLY][GAP_IO_CAP_NO_INPUT_NO_OUTPUT]            = SMPC_METH_JW,
    [GAP_IO_CAP_KB_ONLY][GAP_IO_CAP_KB_DISPLAY]                    = SMPC_METH_PK,

    [GAP_IO_CAP_NO_INPUT_NO_OUTPUT][GAP_IO_CAP_DISPLAY_ONLY]       = SMPC_METH_JW,
    [GAP_IO_CAP_NO_INPUT_NO_OUTPUT][GAP_IO_CAP_DISPLAY_YES_NO]     = SMPC_METH_JW,
    [GAP_IO_CAP_NO_INPUT_NO_OUTPUT][GAP_IO_CAP_KB_ONLY]            = SMPC_METH_JW,
    [GAP_IO_CAP_NO_INPUT_NO_OUTPUT][GAP_IO_CAP_NO_INPUT_NO_OUTPUT] = SMPC_METH_JW,
    [GAP_IO_CAP_NO_INPUT_NO_OUTPUT][GAP_IO_CAP_KB_DISPLAY]         = SMPC_METH_JW,

    [GAP_IO_CAP_KB_DISPLAY][GAP_IO_CAP_DISPLAY_ONLY]               = SMPC_METH_PK,
    [GAP_IO_CAP_KB_DISPLAY][GAP_IO_CAP_DISPLAY_YES_NO]             = SMPC_METH_PK,
    [GAP_IO_CAP_KB_DISPLAY][GAP_IO_CAP_KB_ONLY]                    = SMPC_METH_PK,
    [GAP_IO_CAP_KB_DISPLAY][GAP_IO_CAP_NO_INPUT_NO_OUTPUT]         = SMPC_METH_JW,
    [GAP_IO_CAP_KB_DISPLAY][GAP_IO_CAP_KB_DISPLAY]                 = SMPC_METH_PK,
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */


void smpc_send_use_enc_block_cmd(uint8_t conidx, uint8_t *operand_1, uint8_t *operand_2)
{
    // Send an encryption request to the SMPM task
    struct gapm_use_enc_block_cmd *cmd = KE_MSG_ALLOC(GAPM_USE_ENC_BLOCK_CMD,
                                         TASK_GAPM, KE_BUILD_ID(TASK_GAPC, conidx),
                                         gapm_use_enc_block_cmd);

    cmd->operation = GAPM_USE_ENC_BLOCK;
    // Set operand_1 value
    memcpy(&cmd->operand_1[0], operand_1, GAP_KEY_LEN);
    // Set operand_2 value
    memcpy(&cmd->operand_2[0], operand_2, GAP_KEY_LEN);

    ke_msg_send(cmd);
}

#if (SECURE_CONNECTIONS)

void smpc_send_gen_dh_key_cmd(uint8_t conidx, uint8_t *operand_1, uint8_t *operand_2)
{
    struct gapm_gen_dh_key_cmd *cmd = KE_MSG_ALLOC(GAPM_GEN_DH_KEY_CMD,
                                      TASK_GAPM, KE_BUILD_ID(TASK_GAPC, conidx),
                                      gapm_gen_dh_key_cmd);
    cmd->operation = GAPM_GEN_DH_KEY;
    // Set operand_1 value
    memcpy(&cmd->operand_1[0], operand_1, GAP_P256_KEY_LEN);
    // Set operand_2 value
    memcpy(&cmd->operand_2[0], operand_2, GAP_P256_KEY_LEN);

    ke_msg_send(cmd);
}
#endif // (SECURE_CONNECTIONS)

void smpc_send_start_enc_cmd(uint8_t idx, uint8_t key_type, uint8_t *key, uint8_t *randnb, uint16_t ediv)
{
    struct hci_le_en_enc_cmd *cmd = KE_MSG_ALLOC(HCI_COMMAND, idx, HCI_LE_EN_ENC_CMD_OPCODE, hci_le_en_enc_cmd);

    cmd->conhdl = gapc_get_conhdl(idx);

    if (key_type == SMPC_USE_STK)
    {
        // Set EDIV value to 0
        cmd->enc_div = 0;
        // Set Random Number value to 0
        memset(&cmd->nb.nb[0], 0x00, GAP_RAND_NB_LEN);
    }
    else if (key_type == SMPC_USE_LTK) // SMPC_USE_LTK
    {
#if (SECURE_CONNECTIONS)
        if (smpc_secure_connections_enabled(idx) == true)
        {

            // Set EDIV value to 0
            cmd->enc_div = 0;
            // Set Random Number value to 0
            memset(&cmd->nb.nb[0], 0x00, GAP_RAND_NB_LEN);
        }
        else
#endif // (SECURE_CONNECTIONS)
        {
            // Set EDIV value
            cmd->enc_div = ediv;
            // Set Random Number
            memcpy(&cmd->nb.nb[0], randnb, GAP_RAND_NB_LEN);
            gapc_env[idx]->smpc.state = SMPC_START_ENC_LTK;
        }
    }

    // Copy the key
    memcpy(&cmd->ltk.ltk[0], key, GAP_KEY_LEN);

    hci_send_2_controller(cmd);


    // Set state to encrypting
    gapc_update_state(idx, GAPC_ENCRYPT_BUSY, true);
}

void smpc_send_ltk_req_rsp(uint8_t idx, bool found, uint8_t *key)
{
    if (found)
    {
        // Reply that the encryption key has been found
        struct hci_le_ltk_req_reply_cmd *cmd = KE_MSG_ALLOC(HCI_COMMAND, idx, HCI_LE_LTK_REQ_REPLY_CMD_OPCODE, hci_le_ltk_req_reply_cmd);

        cmd->conhdl = gapc_get_conhdl(idx);

        // Copy the found key
        memcpy(&cmd->ltk.ltk[0], key, GAP_KEY_LEN);

        hci_send_2_controller(cmd);

        // Set state to encrypting
        gapc_update_state(idx, GAPC_ENCRYPT_BUSY, true);
    }
    else
    {
        // Reply that the encryption key has not been found
        struct hci_basic_conhdl_cmd *neg_cmd = KE_MSG_ALLOC(HCI_COMMAND, idx, HCI_LE_LTK_REQ_NEG_REPLY_CMD_OPCODE, hci_basic_conhdl_cmd);

        neg_cmd->conhdl = gapc_get_conhdl(idx);

        hci_send_2_controller(neg_cmd);

        // Go back to IDLE state
        gapc_send_complete_evt(idx, GAPC_OP_SMP, LL_ERR_PIN_MISSING);

        // Reset the internal state
        gapc_env[idx]->smpc.state = SMPC_STATE_RESERVED;
    }
}

void smpc_send_pairing_req_ind(uint8_t conidx, uint8_t req_type)
{
    struct smpc_pair_info *pair = gapc_env[conidx]->smpc.info.pair;

    ASSERT_ERR(pair != NULL);

    struct gapc_bond_req_ind *req_ind = KE_MSG_ALLOC(GAPC_BOND_REQ_IND,
                                        gapc_get_dest_task(conidx),  KE_BUILD_ID(TASK_GAPC, conidx), gapc_bond_req_ind);

    req_ind->request = req_type;

    switch (req_type)
    {
    case (GAPC_PAIRING_REQ):
    {
        req_ind->data.auth_req = (pair->pair_req_feat.auth) & GAP_AUTH_REQ_MASK;

        // Update the internal state
        gapc_env[conidx]->smpc.state = SMPC_PAIRING_FEAT_WAIT;
    }
    break;

    case (GAPC_LTK_EXCH):
    {
        // Provide the key size
        req_ind->data.key_size = gapc_enc_keysize_get(conidx);
    }
    break;

    case (GAPC_CSRK_EXCH):
    {
        // Update the internal state
        gapc_env[conidx]->smpc.state = SMPC_PAIRING_APP_CSRK_WAIT;
    }
    break;

#if (SECURE_CONNECTIONS)
    case (GAPC_NC_EXCH) :
    {
        if (pair->pair_method == SMPC_METH_NC)
        {
            req_ind->data.nc_data.value[0] = (pair->passkey & 0xFF);
            req_ind->data.nc_data.value[1] = (pair->passkey & 0xFF00) >> 8;
            req_ind->data.nc_data.value[2] = (pair->passkey & 0xFF0000) >> 16;
            req_ind->data.nc_data.value[3] = 0;
        }
    }
    break;
#endif // (SECURE_CONNECTIONS)

    case (GAPC_TK_EXCH):
    {
        ASSERT_ERR(pair != NULL);

        // Both have OOB -> The length of the TK shall be 16 bytes
        if ((pair->pair_req_feat.oob == GAP_OOB_AUTH_DATA_PRESENT) &&
                (pair->pair_rsp_feat.oob == GAP_OOB_AUTH_DATA_PRESENT))
        {

            req_ind->data.tk_type = GAP_TK_OOB;
        }
        else
        {
            /*
             * If we are here, the method is Passkey Entry
             * Specification Vol 3, Part H, 2.3.5.1: Selecting STK Generation Method
             */
            if (gapc_get_role(conidx) == ROLE_MASTER)
            {
                if (((pair->pair_rsp_feat.iocap == GAP_IO_CAP_KB_ONLY) ||
                        (pair->pair_rsp_feat.iocap == GAP_IO_CAP_KB_DISPLAY)) &&
                        (pair->pair_req_feat.iocap != GAP_IO_CAP_KB_ONLY))
                {
                    // The application shall display the PIN Code
                    req_ind->data.tk_type = GAP_TK_DISPLAY;
                }
                else
                {
                    // The use shall enter the key using the keyboard
                    req_ind->data.tk_type = GAP_TK_KEY_ENTRY;
                }
            }
            else    // role == ROLE_SLAVE
            {
                if ((pair->pair_rsp_feat.iocap == GAP_IO_CAP_DISPLAY_ONLY)   ||
                        (pair->pair_rsp_feat.iocap == GAP_IO_CAP_DISPLAY_YES_NO) ||
                        ((pair->pair_rsp_feat.iocap == GAP_IO_CAP_KB_DISPLAY) &&
                         (pair->pair_req_feat.iocap == GAP_IO_CAP_KB_ONLY)))
                {
                    // The application shall display the PIN Code
                    req_ind->data.tk_type = GAP_TK_DISPLAY;
                }
                else
                {
                    // The use shall enter the key using the keyboard
                    req_ind->data.tk_type = GAP_TK_KEY_ENTRY;
                }
            }
        }

        gapc_env[conidx]->smpc.state = SMPC_PAIRING_TK_WAIT;
    }
    break;

    case (GAPC_IRK_EXCH):
    {
        // Do nothing
    } break;
#if (SECURE_CONNECTIONS)
    case (GAPC_OOB_EXCH):
    {
        memcpy(req_ind->data.oob_data.conf, &pair->conf_value[0], 16);
        memcpy(req_ind->data.oob_data.rand, &pair->local_r[0], 16);
    }
    break;
#endif // (SECURE_CONNECTIONS)
    default:
    {
        ASSERT_ERR(0);
    }
    break;
    }

    ke_msg_send(req_ind);
}

void smpc_send_pairing_ind(uint8_t conidx, uint8_t ind_type, void *value)
{


    struct gapc_bond_ind *ind = KE_MSG_ALLOC(GAPC_BOND_IND,
                                gapc_get_dest_task(conidx), KE_BUILD_ID(TASK_GAPC, conidx), gapc_bond_ind);

    ind->info = ind_type;

    switch (ind_type)
    {
    case (GAPC_PAIRING_FAILED):
    {
        ind->data.reason = *(uint8_t *)value;
#if (SECURE_CONNECTIONS)
        // clear secure connection enable bit
        gapc_env[conidx]->smpc.secure_connections_enabled = false;
#endif // (SECURE_CONNECTIONS)
    }
    break;

    case (GAPC_PAIRING_SUCCEED):
    {
        struct smpc_pair_info *pair = gapc_env[conidx]->smpc.info.pair;
        ASSERT_ERR(pair != NULL);
        ind->data.auth.info = pair->auth;
        ind->data.auth.ltk_present = pair->ltk_exchanged;

        // informs that link is now encrypted
        gapc_link_encrypted(conidx);

        // update link authorization level
        gapc_auth_set(conidx, ind->data.auth.info, pair->ltk_exchanged);
#if (SECURE_CONNECTIONS)
        // clear secure connection enable bit
        gapc_env[conidx]->smpc.secure_connections_enabled = false;
#endif // (SECURE_CONNECTIONS)
    }
    break;

    case (GAPC_LTK_EXCH):
    {
        memcpy(&ind->data.ltk, value, sizeof(struct gapc_ltk));
    }
    break;

    case (GAPC_IRK_EXCH):
    {
        memcpy(&ind->data.irk, value, sizeof(struct gapc_irk));
    }
    break;

    case (GAPC_CSRK_EXCH):
    {
        memcpy(&ind->data.csrk.key[0], value, GAP_KEY_LEN);
        // update connection environment variables
        memcpy(&(gapc_env[conidx]->smpc.csrk[SMPC_INFO_PEER]), value, GAP_KEY_LEN);
        // reset sign counter
        gapc_env[conidx]->smpc.sign_counter[SMPC_INFO_PEER] = 0;
    }
    break;

    default:
    {
        ASSERT_ERR(0);
    }
    break;
    }

    ke_msg_send(ind);
}

bool smpc_check_pairing_feat(struct gapc_pairing *pair_feat)
{
    // Returned status
    bool status = true;

    // Check IO Capabilities value
    if (pair_feat->iocap > GAP_IO_CAP_KB_DISPLAY)
    {
        status = false;
    }
    // Check Out Of Band status
    else if (((pair_feat->oob) != GAP_OOB_AUTH_DATA_NOT_PRESENT) &&
             ((pair_feat->oob) != GAP_OOB_AUTH_DATA_PRESENT))
    {
        status = false;
    }
    // Check Key Distribution
    else if ((pair_feat->ikey_dist > GAP_KDIST_LAST) ||
             (pair_feat->rkey_dist > GAP_KDIST_LAST))
    {
        status = false;
    }

    return (status);
}

uint8_t smpc_check_repeated_attempts(uint8_t conidx)
{
    // Returned status
    uint8_t status = SMPC_REP_ATTEMPTS_NO_ERROR;

    if (SMPC_IS_FLAG_SET(conidx, SMPC_TIMER_REP_ATT_FLAG))
    {
        // Check if an attack has already been detected
        if (gapc_env[conidx]->smpc.rep_att_timer_val != SMPC_REP_ATTEMPTS_TIMER_MAX_VAL)
        {
            // The timer value shall be increased exponentially if a repeated attempt occurs
            gapc_env[conidx]->smpc.rep_att_timer_val *= SMPC_REP_ATTEMPTS_TIMER_MULT;

            // Check if the timer value is upper than the max limit
            if (gapc_env[conidx]->smpc.rep_att_timer_val >= SMPC_REP_ATTEMPTS_TIMER_MAX_VAL)
            {
                gapc_env[conidx]->smpc.rep_att_timer_val = SMPC_REP_ATTEMPTS_TIMER_MAX_VAL;

                // Inform application about repeated attempt problem
                struct gapc_bond_ind *bond_ind = KE_MSG_ALLOC(GAPC_BOND_IND, gapc_get_dest_task(conidx),
                                                 KE_BUILD_ID(TASK_GAPC, conidx), gapc_bond_ind);
                bond_ind->info = GAPC_REPEATED_ATTEMPT;
                ke_msg_send(bond_ind);
            }

            status = SMPC_REP_ATTEMPT;
        }
        else
        {
            // New attack attempt, the pairing request PDU will be dropped
            status = SMPC_REP_ATTEMPTS_ATTACK;
        }

        // Restart the timer
        smpc_launch_rep_att_timer(conidx);
    }
    // else status is GAP_ERR_NO_ERROR

    return (status);
}

bool smpc_check_max_key_size(uint8_t conidx)
{
    // Returned status
    bool status = false;
    // Negociated key size (min value between both sent key sizes)
    uint8_t size;
    struct smpc_pair_info *pair = gapc_env[conidx]->smpc.info.pair;

    ASSERT_ERR(pair != NULL);

    // The lower size shall be kept as key size.
    if (pair->pair_req_feat.key_size
            < pair->pair_rsp_feat.key_size)
    {
        size = pair->pair_req_feat.key_size;
    }
    else
    {
        size = pair->pair_rsp_feat.key_size;
    }

    // If the key size is below the negociated size, reject it
    if (size >= SMPC_MIN_ENC_SIZE_LEN)
    {
        status = true;

        gapc_enc_keysize_set(conidx, size);
    }

    return (status);
}

bool smpc_check_key_distrib(uint8_t conidx, uint8_t sec_level)
{
    // Returned status
    bool status = true;

    struct smpc_pair_info *pair = gapc_env[conidx]->smpc.info.pair;

    // Keys distributed by the initiator
    uint8_t i_keys    = pair->pair_req_feat.ikey_dist & pair->pair_rsp_feat.ikey_dist;
    // Keys distributed by the responder
    uint8_t r_keys    = pair->pair_req_feat.rkey_dist & pair->pair_rsp_feat.rkey_dist;

    // If both device are bondable check that at least one key is distributed
    if (((pair->pair_req_feat.auth & GAP_AUTH_BOND) == GAP_AUTH_BOND) &&
            ((pair->pair_rsp_feat.auth & GAP_AUTH_BOND) == GAP_AUTH_BOND))
    {
#if (SECURE_CONNECTIONS)
        if ((smpc_secure_connections_enabled(conidx) == false) && ((i_keys == GAP_KDIST_NONE) && (r_keys == GAP_KDIST_NONE)))
        {
            status = false;
        }
#else // !(SECURE_CONNECTIONS)

        if ((i_keys == GAP_KDIST_NONE) && (r_keys == GAP_KDIST_NONE))
        {
            status = false;
        }
#endif // (SECURE_CONNECTIONS)
    }

#if (SECURE_CONNECTIONS)
    if (smpc_secure_connections_enabled(conidx) == true)
    {
        //GAP_SEC1_SEC_CON_PAIR_ENC
        pair->ltk_exchanged = true;
    }
    else
#endif // (SECURE_CONNECTIONS)
    {
        // check if an LTK has been echanged
        if (!(((i_keys & GAP_KDIST_ENCKEY) == GAP_KDIST_ENCKEY) ||
                ((r_keys & GAP_KDIST_ENCKEY) == GAP_KDIST_ENCKEY)))
        {
            pair->ltk_exchanged = false;
        }
        else
        {
            pair->ltk_exchanged = true;
        }
    }

    // If a security mode 1 is required, check if a LTK is distributed
    if ((sec_level == GAP_SEC1_NOAUTH_PAIR_ENC) || (sec_level == GAP_SEC1_AUTH_PAIR_ENC))
    {
        if (!pair->ltk_exchanged)
        {
            status = false;
        }
    }

    // If a security mode 2 is required, check if a CSRK is distributed
    if ((sec_level == GAP_SEC2_NOAUTH_DATA_SGN) || (sec_level == GAP_SEC2_AUTH_DATA_SGN))
    {
        if (!(((i_keys & GAP_KDIST_SIGNKEY) == GAP_KDIST_SIGNKEY) ||
                ((r_keys & GAP_KDIST_SIGNKEY) == GAP_KDIST_SIGNKEY)))
        {
            status = false;
        }
    }

    return (status);
}

void smpc_xor(uint8_t *result, uint8_t *operand_1, uint8_t *operand_2)
{
    // Counter
    uint8_t counter;

    // result = operand_1 XOR operand_2
    for (counter = 0; counter < GAP_KEY_LEN; counter++)
    {
        *(result + counter) = (*(operand_1 + counter)) ^ (*(operand_2 + counter));
    }
}

void smpc_generate_l(uint8_t conidx, uint8_t src)
{
    /**
     * L = AES_128(CSRK, 0[0:127])
     */

    struct gap_sec_key csrk;

    // Get the CSRK in the GAP
    memcpy(&csrk.key[0], gapc_get_csrk(conidx, src), GAP_KEY_LEN);

    // Set the current state of the procedure
    gapc_env[conidx]->smpc.state = SMPC_SIGN_L_GEN;

    // Send an encryption request to the SMPM task
    struct gapm_use_enc_block_cmd *cmd = KE_MSG_ALLOC(GAPM_USE_ENC_BLOCK_CMD,
                                         TASK_GAPM, KE_BUILD_ID(TASK_GAPC, conidx),
                                         gapm_use_enc_block_cmd);

    cmd->operation = GAPM_USE_ENC_BLOCK;
    // operand_1 is the CSRK
    memcpy(&cmd->operand_1[0], &csrk.key[0], GAP_KEY_LEN);
    // Set operand_2 value
    memset(&cmd->operand_2[0], 0x00, GAP_KEY_LEN);

    ke_msg_send(cmd);
}

void smpc_generate_ci(uint8_t conidx, uint8_t src, uint8_t *ci1, uint8_t *mi)
{
    /**
     * Ci = AES_128(CSRK, Ci-1 XOR Mi)
     */

    // Signature Information
    struct smpc_sign_info *sign_info = ((struct smpc_sign_info *)(gapc_env[conidx]->smpc.info.sign));

    // Set the current state of the procedure
    gapc_env[conidx]->smpc.state = SMPC_SIGN_Ci_GEN;

    struct gapm_use_enc_block_cmd *cmd = KE_MSG_ALLOC(GAPM_USE_ENC_BLOCK_CMD,
                                         TASK_GAPM, KE_BUILD_ID(TASK_GAPC, conidx),
                                         gapm_use_enc_block_cmd);

    cmd->operation = GAPM_USE_ENC_BLOCK;
    // operand_1 is the CSRK
    memcpy(&cmd->operand_1[0], gapc_get_csrk(conidx, src), GAP_KEY_LEN);
    // Set operand_2 value = Ci-1 XOR Mi
    smpc_xor(&cmd->operand_2[0], ci1, mi);

    ke_msg_send(cmd);

    // Update number of block to analyze
    sign_info->block_nb--;

    // Update message offset
    if (sign_info->msg_offset < GAP_KEY_LEN)
    {
        sign_info->msg_offset = 0;
    }
    else
    {
        sign_info->msg_offset -= GAP_KEY_LEN;
    }
}

void smpc_generate_rand(uint8_t conidx, uint8_t state)
{
    gapc_env[conidx]->smpc.state = state;

    struct gapm_gen_rand_nb_cmd *cmd = KE_MSG_ALLOC(GAPM_GEN_RAND_NB_CMD,
                                       TASK_GAPM, KE_BUILD_ID(TASK_GAPC, conidx),
                                       gapm_gen_rand_nb_cmd);

    cmd->operation = GAPM_GEN_RAND_NB;

    ke_msg_send(cmd);
}
#if (SECURE_CONNECTIONS)
uint8_t smpc_get_next_passkey_bit(uint8_t conidx)
{
    ASSERT_ERR(gapc_env[conidx]->smpc.info.pair != NULL);

    uint32_t passkey = gapc_env[conidx]->smpc.info.pair->passkey;
    uint8_t bit_num = gapc_env[conidx]->smpc.info.pair->passkey_bit_count;


    if (passkey > 0xF4240)
        passkey = passkey % 0xF4240;

    return ((passkey >> bit_num) & 0x01);
}
#endif // (SECURE_CONNECTIONS)
void smpc_generate_e1(uint8_t conidx, uint8_t role, bool local)
{
    struct smpc_pair_info *pair = gapc_env[conidx]->smpc.info.pair;
    /**
     * Calculation of e1 is the first step of the confirm value generation
     *  e1 = AES_128(RAND, p1)  with:
     *      p1 = PRES || PREQ || 0 || RAT || 0 || IAT
     *          PRES = Pairing Response Command
     *          PREQ = Pairing Request Command
     *          RAT  = Responding device address type
     *          IAT  = Initiating Device Address Type
     */

    // P1 value (LSB->MSB)
    uint8_t p1[GAP_KEY_LEN];
    // Offset
    uint8_t offset = 0;

    memset(&p1[0], 0x00, GAP_KEY_LEN);

    /*
     * Initiating Device Address Type
     * Responding Device Address Type
     */
    if (role == ROLE_MASTER)
    {
        p1[offset]     = gapc_get_bdaddr(conidx, SMPC_INFO_LOCAL)->addr_type;
        p1[offset + 1] = gapc_get_bdaddr(conidx, SMPC_INFO_PEER)->addr_type;
    }
    else    // role == ROLE_SLAVE
    {
        p1[offset]     = gapc_get_bdaddr(conidx, SMPC_INFO_PEER)->addr_type;
        p1[offset + 1] = gapc_get_bdaddr(conidx, SMPC_INFO_LOCAL)->addr_type;
    }

    offset += 2;

    /*
     * Pairing Request Command
     */
    p1[offset] = L2C_CODE_PAIRING_REQ;
    offset++;

    memcpy(&p1[offset], &pair->pair_req_feat, SMPC_CODE_PAIRING_REQ_RESP_LEN - 1);
    offset += (SMPC_CODE_PAIRING_REQ_RESP_LEN - 1);

    /*
     * Pairing Response Command
     */
    p1[offset] = L2C_CODE_PAIRING_RESP;
    offset++;
    memcpy(&p1[offset], &pair->pair_rsp_feat, SMPC_CODE_PAIRING_REQ_RESP_LEN - 1);

    struct gapm_use_enc_block_cmd *cmd = KE_MSG_ALLOC(GAPM_USE_ENC_BLOCK_CMD,
                                         TASK_GAPM, KE_BUILD_ID(TASK_GAPC, conidx),
                                         gapm_use_enc_block_cmd);

    cmd->operation = GAPM_USE_ENC_BLOCK;

    // Operand_1 is the TK
    memcpy(&cmd->operand_1[0], &pair->key.key[0], GAP_KEY_LEN);

    // The used random value depends on the confirm value type (Rand values stored LSB->MSB)
    if (local)
    {
        // Set operand_2 value = R XOR P1
        smpc_xor(&cmd->operand_2[0], &pair->rand[0], &p1[0]);
    }
    else    // Remote
    {
        // Set operand_2 value = R XOR P1
        smpc_xor(&cmd->operand_2[0], &pair->rem_rand[0], &p1[0]);
    }

    ke_msg_send(cmd);
}

void smpc_generate_cfm(uint8_t conidx, uint8_t role, uint8_t *e1)
{
    uint8_t operand_1[GAP_KEY_LEN];
    uint8_t operand_2[GAP_KEY_LEN];

    /**
     *  cfm = AES_128(e1, p2)  with:
     *      p2 = 0[0:4] || IA || RA
     *          RA  = Responding device address
     *          IA  = Initiating Device address
     */

    // P2
    uint8_t p2[CFM_LEN];

    memset(&p2[0], 0x00, GAP_KEY_LEN);

    /*
     * Responding Device Address
     * Initiating Device Address+
     */
    if (role == ROLE_MASTER)
    {
        memcpy(&p2[0], &(gapc_get_bdaddr(conidx, SMPC_INFO_PEER)->addr.addr), BD_ADDR_LEN);
        memcpy(&p2[BD_ADDR_LEN], &(gapc_get_bdaddr(conidx, SMPC_INFO_LOCAL)->addr.addr), BD_ADDR_LEN);
    }
    else    // role == ROLE_SLAVE
    {
        memcpy(&p2[0], &(gapc_get_bdaddr(conidx, SMPC_INFO_LOCAL)->addr.addr), BD_ADDR_LEN);
        memcpy(&p2[BD_ADDR_LEN], &(gapc_get_bdaddr(conidx, SMPC_INFO_PEER)->addr.addr), BD_ADDR_LEN);
    }

    // Operand_1 is the TK
    memcpy(&operand_1[0], &gapc_env[conidx]->smpc.info.pair->key.key[0], GAP_KEY_LEN);

    // Set operand_2 value = E1 XOR P2
    smpc_xor(&operand_2[0], e1, &p2[0]);

    smpc_send_use_enc_block_cmd(conidx, operand_1, operand_2);
}

void smpc_generate_stk(uint8_t conidx, uint8_t role)
{
    struct smpc_pair_info *pair = gapc_env[conidx]->smpc.info.pair;
    /**
     * ********************************************
     * CALCULATE STK
     *      STK = AES_128(TK, r) with:
     *            r = LSB64(Srand) || LSB64(Mrand)
     * ********************************************
     */

    uint8_t r[GAP_KEY_LEN];

    if (role == ROLE_MASTER)
    {
        memcpy(&(r[0]), &(pair->rand[0]), (RAND_VAL_LEN / 2));
        memcpy(&(r[RAND_VAL_LEN / 2]), &(pair->rem_rand[0]), (RAND_VAL_LEN / 2));
    }
    else    // role == ROLE_SLAVE
    {
        memcpy(&(r[0]), &(pair->rem_rand[0]), (RAND_VAL_LEN / 2));
        memcpy(&(r[RAND_VAL_LEN / 2]), &(pair->rand[0]), (RAND_VAL_LEN / 2));
    }

    // Set the state
    gapc_env[conidx]->smpc.state = SMPC_PAIRING_GEN_STK;

    // Call the AES 128 block
    smpc_send_use_enc_block_cmd(conidx, &pair->key.key[0], &r[0]);
}

void smpc_calc_subkeys(bool gen_k2, uint8_t *l_value, uint8_t *subkey)
{
    // NOTE: All arrays have format: [0:15] = LSB to MSB

    // Counter
    uint8_t counter;
    // MSB
    uint8_t msb;

    // Rb = 0*120 1000 0111
    uint8_t rb[16] = {0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                     };

    /**
     * -------------------------------------------------------------------
     * K1 CALCULATION
     * If MSB(L) = 0, K1 = (L << 1), else K1 = (L << 1) XOR Rb
     * -------------------------------------------------------------------
     */

    // L<<1
    for (counter = (GAP_KEY_LEN - 1); counter >= 1; counter--)
    {
        *(subkey + counter) = (*(l_value + counter) << 1) | (*(l_value + counter - 1) >> 7);
    }

    // Last one
    *(subkey) = (*(l_value) << 1);

    // Check MSBit(L) value
    if ((*(l_value +  GAP_KEY_LEN - 1) & 0x80) == 0x80)
    {
        smpc_xor(subkey, subkey, &rb[0]);
    }

    if (gen_k2)
    {
        /**
         * -------------------------------------------------------------------
         * K2 CALCULATION
         * If MSB(K2) = 0, K2 = (K1 << 1), else K2 = (K1 << 1) XOR Rb
         * -------------------------------------------------------------------
         */

        msb = *(subkey +  GAP_KEY_LEN - 1);

        // K1<<1
        for (counter = (GAP_KEY_LEN - 1); counter >= 1; counter--)
        {
            *(subkey + counter) = (*(subkey + counter) << 1) | (*(subkey + counter - 1) >> 7);
        }

        // Last one
        *(subkey) = (*(subkey) << 1);

        // Check MSB(K1) value
        if ((msb & 0x80) == 0x80)
        {
            smpc_xor(subkey, subkey, &rb[0]);
        }
    }
}

void smpc_tkdp_send_start(uint8_t conidx, uint8_t role)
{
    struct smpc_pair_info *pair = gapc_env[conidx]->smpc.info.pair;

    // Use generic variable for the master or slave key distribution
    ASSERT_ERR(pair != NULL);

    if (role == ROLE_MASTER)
    {
        pair->keys_dist = pair->pair_req_feat.ikey_dist & pair->pair_rsp_feat.ikey_dist;
    }
    else
    {
        pair->keys_dist = pair->pair_req_feat.rkey_dist & pair->pair_rsp_feat.rkey_dist;
    }

    // default key exchange state
    gapc_env[conidx]->smpc.state = SMPC_PAIRING_APP_WAIT;

    // execute sending state of key exchange
    smpc_tkdp_send_continue(conidx, role);
}

void smpc_tkdp_send_continue(uint8_t conidx, uint8_t role)
{
    struct smpc_pair_info *pair = gapc_env[conidx]->smpc.info.pair;

    // Use generic variable for the master or slave key distribution
    ASSERT_ERR(pair != NULL);

    switch (gapc_env[conidx]->smpc.state)
    {
    // answer from application received
    case SMPC_PAIRING_APP_LTK_WAIT:
    case SMPC_PAIRING_APP_IRK_WAIT:
    case SMPC_PAIRING_APP_CSRK_WAIT:
    {
        // return into default state to continue key exchange
        gapc_env[conidx]->smpc.state = SMPC_PAIRING_APP_WAIT;
    }
    break;

    // nothing more expected, so continue key exchange procedure
    case SMPC_PAIRING_APP_WAIT:
    {
        // nothing to do
    } break;

    default:
    {
        ASSERT_INFO(0, gapc_env[conidx]->smpc.state, pair->keys_dist);
    }
    break;
    }

    // default state check if key are expected
    while (gapc_env[conidx]->smpc.state == SMPC_PAIRING_APP_WAIT)
    {
        // LTK expected
        if ((pair->keys_dist & GAP_KDIST_ENCKEY) == GAP_KDIST_ENCKEY)
        {
            pair->keys_dist &= ~GAP_KDIST_ENCKEY;

#if (SECURE_CONNECTIONS)
            if (smpc_secure_connections_enabled(conidx) == false)
#endif // (SECURE_CONNECTIONS)
            {
                // LTK need to be retrieved from the HL
                smpc_send_pairing_req_ind(conidx, GAPC_LTK_EXCH);

                // Update the internal state
                gapc_env[conidx]->smpc.state = SMPC_PAIRING_APP_LTK_WAIT;
            }
        }
        // IRK Expected
        else if ((pair->keys_dist & GAP_KDIST_IDKEY) == GAP_KDIST_IDKEY)
        {
            pair->keys_dist &= ~GAP_KDIST_IDKEY;

            // Check if Controller Privacy is being used
            if (gapm_get_address_type() & GAPM_PRIV_CFG_PRIV_EN_BIT)
            {
                // Ask for the IRK
                smpc_send_pairing_req_ind(conidx, GAPC_IRK_EXCH);
                gapc_env[conidx]->smpc.state = SMPC_PAIRING_APP_IRK_WAIT;
            }
            else
            {
                // Send the IRK to the peer device
                smpc_pdu_send(conidx, L2C_CODE_IDENTITY_INF, (void *)gapm_get_irk());
                // Send the BD Address to the peer device
                smpc_pdu_send(conidx, L2C_CODE_ID_ADDR_INF, (void *)gapm_get_bdaddr());
            }
        }
        // CSRK Expected
        else if ((pair->keys_dist & GAP_KDIST_SIGNKEY) == GAP_KDIST_SIGNKEY)
        {
            pair->keys_dist &= ~GAP_KDIST_SIGNKEY;

            gapc_env[conidx]->smpc.state = SMPC_PAIRING_APP_CSRK_WAIT;
            // Ask for the CSRK
            smpc_send_pairing_req_ind(conidx, GAPC_CSRK_EXCH);
        }
        // All keys have been exchanged
        else
        {
            if (role == ROLE_MASTER)
            {
                // Pairing is over
                smpc_pairing_end(conidx, role, GAP_ERR_NO_ERROR, false);
            }
            else    // role == ROLE_SLAVE
            {
                // Slave will now receive keys (if expected)
                smpc_tkdp_rcp_start(conidx, role);
            }
            break;
        }
    }
}

void smpc_tkdp_rcp_start(uint8_t conidx, uint8_t role)
{
    struct smpc_pair_info *pair = gapc_env[conidx]->smpc.info.pair;

    // Use generic variable for the master or slave key distribution
    ASSERT_ERR(pair != NULL);

    if (role == ROLE_MASTER)
    {
        pair->keys_dist = pair->pair_req_feat.rkey_dist & pair->pair_rsp_feat.rkey_dist;
    }
    else
    {
        pair->keys_dist = pair->pair_req_feat.ikey_dist & pair->pair_rsp_feat.ikey_dist;
    }

    gapc_env[conidx]->smpc.state = SMPC_PAIRING_REM_WAIT;

    // start reception execution
    smpc_tkdp_rcp_continue(conidx, role);
}

void smpc_tkdp_rcp_continue(uint8_t conidx, uint8_t role)
{
    struct smpc_pair_info *pair = gapc_env[conidx]->smpc.info.pair;
    ASSERT_ERR(pair != NULL);

    switch (gapc_env[conidx]->smpc.state)
    {
    // LTK shall always be followed by the EDIV and the Rand values
    case SMPC_PAIRING_REM_LTK_WAIT:
    {
        // Wait for the Master ID PDU
        gapc_env[conidx]->smpc.state = SMPC_PAIRING_REM_MST_ID_WAIT;
    }
    break;

    // IRK shall always be followed by the BD Address
    case SMPC_PAIRING_REM_IRK_WAIT:
    {
        // Wait for the peer device BD Address
        gapc_env[conidx]->smpc.state = SMPC_PAIRING_REM_BD_ADDR_WAIT;
    }
    break;

    case SMPC_PAIRING_REM_MST_ID_WAIT:
    case SMPC_PAIRING_REM_BD_ADDR_WAIT:
    case SMPC_PAIRING_REM_CSRK_WAIT:
    {
        // return into default state to continue key exchange
        gapc_env[conidx]->smpc.state = SMPC_PAIRING_REM_WAIT;
    }
    break;

    // nothing more expected, so continue key exchange procedure
    case SMPC_PAIRING_REM_WAIT:
    {
        // nothing to do
    } break;

    default:
    {
        ASSERT_INFO(0, gapc_env[conidx]->smpc.state, pair->keys_dist);
    }
    break;
    }

    // default state check if key are expected
    if (gapc_env[conidx]->smpc.state == SMPC_PAIRING_REM_WAIT)
    {
#if (SECURE_CONNECTIONS)
        // LTK not expected for secure connection
        if (smpc_secure_connections_enabled(conidx) == true)
        {
            pair->keys_dist &= ~GAP_KDIST_ENCKEY;
        }
#endif // (SECURE_CONNECTIONS)

        // LTK expected
        if ((pair->keys_dist & GAP_KDIST_ENCKEY) == GAP_KDIST_ENCKEY)
        {
            gapc_env[conidx]->smpc.state = SMPC_PAIRING_REM_LTK_WAIT;
            pair->keys_dist &= ~GAP_KDIST_ENCKEY;
        }
        // IRK Expected
        else if ((pair->keys_dist & GAP_KDIST_IDKEY) == GAP_KDIST_IDKEY)
        {
            gapc_env[conidx]->smpc.state = SMPC_PAIRING_REM_IRK_WAIT;
            pair->keys_dist &= ~GAP_KDIST_IDKEY;
        }
        // CSRK Expected
        else if ((pair->keys_dist & GAP_KDIST_SIGNKEY) == GAP_KDIST_SIGNKEY)
        {
            gapc_env[conidx]->smpc.state = SMPC_PAIRING_REM_CSRK_WAIT;
            pair->keys_dist &= ~GAP_KDIST_SIGNKEY;
        }
        // All keys have been exchanged
        else
        {
            if (role == ROLE_MASTER)
            {
                // Slave device has sent all his keys, send his own keys
                smpc_tkdp_send_start(conidx, ROLE_MASTER);
            }
            else    // role = ROLE_SLAVE
            {
                // All keys have been exchanged, pairing is over
                smpc_pairing_end(conidx, role, GAP_ERR_NO_ERROR, false);
            }
        }
    }
}

void smpc_pairing_end(uint8_t conidx, uint8_t role, uint8_t status, bool start_ra_timer)
{
    ASSERT_ERR(gapc_env[conidx]->smpc.info.pair != NULL);

    if (status == GAP_ERR_NO_ERROR)
    {
        smpc_send_pairing_ind(conidx, GAPC_PAIRING_SUCCEED, (void *)NULL);
    }
    else
    {
        smpc_send_pairing_ind(conidx, GAPC_PAIRING_FAILED, (void *)&status);

        if (start_ra_timer)
        {
            smpc_launch_rep_att_timer(conidx);
        }
    }


    gapc_send_complete_evt(conidx, GAPC_OP_SMP, GAP_ERR_NO_ERROR);

    if (gapc_env[conidx]->smpc.info.pair != NULL)
    {
        // Release the memory allocated for the Pairing Information structure.
        ke_free(gapc_env[conidx]->smpc.info.pair);
        gapc_env[conidx]->smpc.info.pair = NULL;
    }

    // Stop the timeout timer if needed
    smpc_clear_timeout_timer(conidx);

    // Reset the internal state
    gapc_env[conidx]->smpc.state = SMPC_STATE_RESERVED;
}

void smpc_clear_timeout_timer(uint8_t conidx)
{
    // Test if the Timeout Timer already on, then clear it
    if (SMPC_IS_FLAG_SET(conidx, SMPC_TIMER_TIMEOUT_FLAG))
    {
        ke_timer_clear(GAPC_SMP_TIMEOUT_TIMER_IND, KE_BUILD_ID(TASK_GAPC, conidx));
        SMPC_TIMER_UNSET_FLAG(conidx, SMPC_TIMER_TIMEOUT_FLAG);
    }
}

void smpc_launch_rep_att_timer(uint8_t conidx)
{
    // Test if the timer is already running
    if (SMPC_IS_FLAG_SET(conidx, SMPC_TIMER_REP_ATT_FLAG))
    {
        // Stop the timer
        ke_timer_clear(GAPC_SMP_REP_ATTEMPTS_TIMER_IND, KE_BUILD_ID(TASK_GAPC, conidx));
    }

    // Start the timer
    ke_timer_set(GAPC_SMP_REP_ATTEMPTS_TIMER_IND,
                 KE_BUILD_ID(TASK_GAPC, conidx),
                 gapc_env[conidx]->smpc.rep_att_timer_val);

    // Set the status in the environment
    SMPC_TIMER_SET_FLAG(conidx, SMPC_TIMER_REP_ATT_FLAG);
}

void smpc_get_key_sec_prop(uint8_t conidx)
{
    ASSERT_ERR(gapc_env[conidx]->smpc.info.pair != NULL);

#if (SECURE_CONNECTIONS)
    if (!smpc_secure_connections_enabled(conidx))
#endif // (SECURE_CONNECTIONS)
    {
        // Check if the TK will be OOB data
        if ((gapc_env[conidx]->smpc.info.pair->pair_req_feat.oob == GAP_OOB_AUTH_DATA_PRESENT) &&
                (gapc_env[conidx]->smpc.info.pair->pair_rsp_feat.oob == GAP_OOB_AUTH_DATA_PRESENT))
        {
            // Will have to get the TK from host
            gapc_env[conidx]->smpc.info.pair->pair_method  = SMPC_METH_OOB;
        }
        // Both have no MITM set in authentication requirements
        else if (((gapc_env[conidx]->smpc.info.pair->pair_req_feat.auth & GAP_AUTH_MITM) == 0x00) &&
                 ((gapc_env[conidx]->smpc.info.pair->pair_rsp_feat.auth & GAP_AUTH_MITM) == 0x00))
        {
            // Will have to use TK = 0, no need to ask Host
            gapc_env[conidx]->smpc.info.pair->pair_method  = SMPC_METH_JW;
        }
        // In function of IOs, the PASSKEY ENTRY or JW methods will be used
        else
        {
            gapc_env[conidx]->smpc.info.pair->pair_method
                = smpc_pair_method[gapc_env[conidx]->smpc.info.pair->pair_rsp_feat.iocap]
                  [gapc_env[conidx]->smpc.info.pair->pair_req_feat.iocap];
        }

        // Security properties of the STK and all distributed keys
        switch (gapc_env[conidx]->smpc.info.pair->pair_method)
        {
        case (SMPC_METH_OOB):
        case (SMPC_METH_PK):
        {
            // All distributed keys will have these properties
            gapc_env[conidx]->smpc.info.pair->auth = GAP_AUTH_MITM;
        }
        break;

        case (SMPC_METH_JW):
        {
            // All distributed keys will have these properties
            gapc_env[conidx]->smpc.info.pair->auth = GAP_AUTH_NONE;
        }
        break;

        default:
        {
            ASSERT_ERR(0);
        }
        break;
        }

        // Check if both devices are bondable
        if (((gapc_env[conidx]->smpc.info.pair->pair_req_feat.auth & GAP_AUTH_BOND) == GAP_AUTH_BOND) &&
                ((gapc_env[conidx]->smpc.info.pair->pair_rsp_feat.auth & GAP_AUTH_BOND) == GAP_AUTH_BOND))
        {
            gapc_env[conidx]->smpc.info.pair->auth |= GAP_AUTH_BOND;
        }
    }
#if (SECURE_CONNECTIONS)
    else // Secure Connections
    {
        if ((gapc_env[conidx]->smpc.info.pair->pair_req_feat.oob == GAP_OOB_AUTH_DATA_PRESENT) ||
                (gapc_env[conidx]->smpc.info.pair->pair_rsp_feat.oob == GAP_OOB_AUTH_DATA_PRESENT))
        {
            // Will have to get the TK from host
            gapc_env[conidx]->smpc.info.pair->pair_method  = SMPC_METH_OOB;
        }
        // Both have no MITM set in authentication requirements
        else if (((gapc_env[conidx]->smpc.info.pair->pair_req_feat.auth & GAP_AUTH_MITM) == 0x00) &&
                 ((gapc_env[conidx]->smpc.info.pair->pair_rsp_feat.auth & GAP_AUTH_MITM) == 0x00))
        {
            // Will have to use TK = 0, no need to ask Host
            gapc_env[conidx]->smpc.info.pair->pair_method  = SMPC_METH_JW;
        }
        else
        {
            gapc_env[conidx]->smpc.info.pair->pair_method
                = smpc_pair_method[gapc_env[conidx]->smpc.info.pair->pair_rsp_feat.iocap]
                  [gapc_env[conidx]->smpc.info.pair->pair_req_feat.iocap];

            if (((gapc_env[conidx]->smpc.info.pair->pair_rsp_feat.iocap == GAP_IO_CAP_DISPLAY_YES_NO) ||
                    (gapc_env[conidx]->smpc.info.pair->pair_rsp_feat.iocap == GAP_IO_CAP_KB_DISPLAY)) &&
                    ((gapc_env[conidx]->smpc.info.pair->pair_req_feat.iocap == GAP_IO_CAP_DISPLAY_YES_NO) ||
                     (gapc_env[conidx]->smpc.info.pair->pair_req_feat.iocap == GAP_IO_CAP_KB_DISPLAY)))
            {
                gapc_env[conidx]->smpc.info.pair->pair_method = SMPC_METH_NC;
            }
        }
        // Security properties of the STK and all distributed keys
        switch (gapc_env[conidx]->smpc.info.pair->pair_method)
        {
        case (SMPC_METH_OOB):
        case (SMPC_METH_PK):
        case (SMPC_METH_NC):
        {
            // All distributed keys will have these properties
            gapc_env[conidx]->smpc.info.pair->auth = GAP_AUTH_SEC_CON | GAP_AUTH_MITM;
        }
        break;

        case (SMPC_METH_JW):
        {
            // All distributed keys will have these properties
            gapc_env[conidx]->smpc.info.pair->auth = GAP_AUTH_SEC_CON;
        }
        break;

        default:
        {
            ASSERT_ERR(0);
        }
        break;
        }

        // Check if both devices are bondable
        if (((gapc_env[conidx]->smpc.info.pair->pair_req_feat.auth & GAP_AUTH_BOND) == GAP_AUTH_BOND) &&
                ((gapc_env[conidx]->smpc.info.pair->pair_rsp_feat.auth & GAP_AUTH_BOND) == GAP_AUTH_BOND))
        {
            gapc_env[conidx]->smpc.info.pair->auth |= GAP_AUTH_BOND;
        }
    }
#endif // (SECURE_CONNECTIONS)
}

bool smpc_is_sec_mode_reached(uint8_t conidx, uint8_t role)
{
    // Returned status
    bool status = true;
    // Requested Security Mode
    uint8_t secmode;

    ASSERT_ERR(gapc_env[conidx]->smpc.info.pair != NULL);

    // Retrieve the requested security level
    if (role == ROLE_MASTER)
    {
        secmode = gapc_env[conidx]->smpc.info.pair->pair_req_feat.sec_req;
    }
    else // role == ROLE_SLAVE
    {
        secmode = gapc_env[conidx]->smpc.info.pair->pair_rsp_feat.sec_req;
    }

    // The mode is not reached
    if (gapc_env[conidx]->smpc.info.pair->pair_method == SMPC_METH_JW)
    {
        if ((secmode == GAP_SEC1_AUTH_PAIR_ENC) || (secmode == GAP_SEC2_AUTH_DATA_SGN))
        {
            status = false;
        }
    }

    return (status);
}

void smpc_handle_enc_change_evt(uint8_t conidx, uint8_t role, uint8_t status)
{
    uint8_t int_status = GAP_ERR_NO_ERROR;

    switch (gapc_env[conidx]->smpc.state)
    {
    case (SMPC_START_ENC_LTK):
#if (SECURE_CONNECTIONS)
    case (SMPC_PAIRING_SC_W4_ENCRYPTION_CHANGE) :
#endif // (SECURE_CONNECTIONS)
    {
#if (SECURE_CONNECTIONS)
        if (smpc_secure_connections_enabled(conidx) == false)
#endif // (SECURE_CONNECTIONS)
        {
            ASSERT_ERR(gapc_get_operation(conidx, GAPC_OP_SMP) == GAPC_ENCRYPT);
        }

        // Clear encrypt state bit
        gapc_update_state(conidx, GAPC_ENCRYPT_BUSY, false);

        if (status == CO_ERROR_PIN_MISSING)
        {
            // The peer device cannot find the keys to start encryption
            int_status = SMP_ERROR_ENC_KEY_MISSING;
        }
        else if (status == CO_ERROR_UNSUPPORTED)
        {
            // The peer device doesn't support encryption
            int_status = SMP_ERROR_ENC_NOT_SUPPORTED;
        }
        else if (status == CO_ERROR_LMP_RSP_TIMEOUT)
        {
            // The encryption didn't because a timeout has occurred
            int_status = SMP_ERROR_ENC_TIMEOUT;
        }
        else if (status == CO_ERROR_CON_TIMEOUT)
        {
            // The encryption didn't because a connection timeout has occurred
            int_status = RW_ERR_HCI_TO_HL(status);
        }
        else
        {
            ASSERT_INFO(status == CO_ERROR_NO_ERROR, status, conidx);
        }

        if (int_status == GAP_ERR_NO_ERROR)
        {
#if (SECURE_CONNECTIONS)
            if ((smpc_secure_connections_enabled(conidx) == true) &&
                    (gapc_env[conidx]->smpc.state == SMPC_PAIRING_SC_W4_ENCRYPTION_CHANGE))
            {
                // Reset the internal state
                gapc_env[conidx]->smpc.state = SMPC_STATE_RESERVED;

                // informs that link is now encrypted
                gapc_link_encrypted(conidx);

                // Check if pairing information should be bond or not, if yes, provide key to the application.
                if (((gapc_env[conidx]->smpc.info.pair->pair_req_feat.auth & GAP_AUTH_BOND) == GAP_AUTH_BOND) &&
                        ((gapc_env[conidx]->smpc.info.pair->pair_rsp_feat.auth & GAP_AUTH_BOND) == GAP_AUTH_BOND))
                {
                    struct gapc_bond_ind *ind = KE_MSG_ALLOC(GAPC_BOND_IND,
                                                gapc_get_dest_task(conidx), KE_BUILD_ID(TASK_GAPC, conidx), gapc_bond_ind);

                    ind->info = GAPC_LTK_EXCH;

                    memcpy(&ind->data.ltk.ltk.key[0], &gapc_env[conidx]->smpc.info.pair->key.key[0], sizeof(struct gapc_ltk));
                    memset(&ind->data.ltk.randnb.nb[0], 0x00, GAP_RAND_NB_LEN);
                    ind->data.ltk.ediv = 0x00;

                    ke_msg_send(ind);
                }

                if (role == ROLE_MASTER)
                {
                    /*
                     * Phase 2 of the pairing is now over, start Transport Keys Distribution.
                     * The master will received the slave's keys.
                     */
                    smpc_tkdp_rcp_start(conidx, ROLE_MASTER);
                }
                else
                {
                    /*
                     * Phase 2 of the pairing is now over, start Transport Keys Distribution.
                     * The slave begins to send its keys.
                     */
                    smpc_tkdp_send_start(conidx, ROLE_SLAVE);
                }
            }
            else
#endif // (SECURE_CONNECTIONS)
            {
                // Reset the internal state
                gapc_env[conidx]->smpc.state = SMPC_STATE_RESERVED;

                // send an indication to security application
                struct gapc_encrypt_ind *ind = KE_MSG_ALLOC(GAPC_ENCRYPT_IND,
                                               gapc_get_dest_task(conidx), KE_BUILD_ID(TASK_GAPC, conidx), gapc_encrypt_ind);

                // informs that link is now encrypted
                gapc_link_encrypted(conidx);

                // Retrieve authentification value
                ind->auth = gapc_auth_get(conidx);

                // send message.
                ke_msg_send(ind);

                gapc_send_complete_evt(conidx, GAPC_OP_SMP, int_status);
            }
        }
        else
        {
            // Reset the internal state
            gapc_env[conidx]->smpc.state = SMPC_STATE_RESERVED;

            // Inform the application about the error
            gapc_send_complete_evt(conidx, GAPC_OP_SMP, int_status);
        }
    }
    break;

    case (SMPC_START_ENC_STK):
    {
        // Clear encrypt state bit
        gapc_update_state(conidx, GAPC_ENCRYPT_BUSY, false);

        ASSERT_ERR(gapc_get_operation(conidx, GAPC_OP_SMP) == GAPC_BOND);
        ASSERT_ERR(gapc_env[conidx]->smpc.info.pair != NULL);

        if (int_status == GAP_ERR_NO_ERROR)
        {
            if (role == ROLE_MASTER)
            {
                /*
                 * Phase 2 of the pairing is now over, start Transport Keys Distribution.
                 * The master will received the slave's keys.
                 */
                smpc_tkdp_rcp_start(conidx, ROLE_MASTER);
            }
            else
            {
                /*
                 * Phase 2 of the pairing is now over, start Transport Keys Distribution.
                 * The slave begins to send its keys.
                 */
                smpc_tkdp_send_start(conidx, ROLE_SLAVE);
            }
        }
        else
        {
            // Inform the HL that the pairing failed
            smpc_pairing_end(conidx, role, int_status, false);
        }
    }
    break;

    default:
    {
        // Do nothing, this event can be triggered by LL during disconnection,
        // just ignore it.
    } break;
    }
}

void smpc_pdu_send(uint8_t conidx, uint8_t cmd_code, void *value)
{
    // Test if the Timeout Timer already on, then clear it
    if (SMPC_IS_FLAG_SET(conidx, SMPC_TIMER_TIMEOUT_FLAG))
    {
        ke_timer_clear(GAPC_SMP_TIMEOUT_TIMER_IND, KE_BUILD_ID(TASK_GAPC, conidx));
        SMPC_TIMER_UNSET_FLAG(conidx, SMPC_TIMER_TIMEOUT_FLAG);
    }

    // Allocate the message for L2CC task
    struct l2cc_pdu_data_t *pdu = L2CC_SMP_PDU_ALLOC(conidx, cmd_code, KE_BUILD_ID(TASK_GAPC, conidx), l2cc_pdu_data_t);
    struct l2cc_pdu_send_cmd *pdu_cmd =  L2CC_PDU_TO_CMD(pdu);

    smpc_construct_pdu[cmd_code](&(pdu_cmd->pdu), value);

    // Send message to L2CAP
    l2cc_pdu_send(pdu);

    // Start the Timeout Timer (SEC_REQ and PAIRING_FAILED don't need an answer)
    if ((cmd_code != L2C_CODE_SECURITY_REQ) &&
            (cmd_code != L2C_CODE_PAIRING_FAILED))
    {
        ke_timer_set(GAPC_SMP_TIMEOUT_TIMER_IND, KE_BUILD_ID(TASK_GAPC, conidx), SMPC_TIMEOUT_TIMER_DURATION);
        SMPC_TIMER_SET_FLAG(conidx, SMPC_TIMER_TIMEOUT_FLAG);
    }
}

void smpc_pdu_recv(uint8_t conidx, struct l2cc_pdu *pdu)
{
    // Check PDU parameters
    uint8_t status = smpc_check_param(pdu);

    if (status == GAP_ERR_NO_ERROR)
    {
        // Depending on PDU, do what is specified by protocol
        (smpc_recv_pdu[pdu->data.code])(conidx, pdu);
    }
    // The data packet is bad, send status to GAP.
    else
    {
        // If a packet is received with a reserved code it shall be ignored.
        if (status != SMP_ERROR_CMD_NOT_SUPPORTED)
        {
            // Send PDU fail to peer (status is SMP_ERROR_INVALID_PARAM)
            smpc_pdu_send(conidx, L2C_CODE_PAIRING_FAILED, (void *)&status);

            /*
             * Check if upper layers need to be informed about the error:
             *      => No error sent if code is PAIRING_REQUEST or SECURITY_REQUEST
             */
            if ((pdu->data.code != L2C_CODE_PAIRING_REQ) &&
                    (pdu->data.code != L2C_CODE_SECURITY_REQ))
            {
                // Inform HL about the pairing failed - PDU are received only during Pairing
                smpc_pairing_end(conidx, gapc_get_role(conidx),
                                 SMP_GEN_PAIR_FAIL_REASON(SMP_PAIR_FAIL_REASON_MASK, status), true);
            }
        }
    }
}


#if (SECURE_CONNECTIONS)
void smpc_initiate_dhkey_check(uint8_t conidx)
{
    uint8_t *p_MacKey;
    uint8_t  IOcap_local[3];
    uint8_t *Nlocal = &gapc_env[conidx]->smpc.info.pair->rand[0];
    uint8_t *Npeer = &gapc_env[conidx]->smpc.info.pair->rem_rand[0];
    uint8_t Addr_local[7];
    uint8_t Addr_peer[7];


    p_MacKey = &gapc_env[conidx]->smpc.info.pair->MacKey[0];

    // Construct the Address for local and peer.
    memcpy(&Addr_local[0], &gapc_env[conidx]->src[SMPC_INFO_LOCAL].addr, 6);
    memcpy(&Addr_peer[0], &gapc_env[conidx]->src[SMPC_INFO_PEER].addr, 6);

    Addr_local[6] = gapc_env[conidx]->src[SMPC_INFO_LOCAL].addr_type;
    Addr_peer[6] = gapc_env[conidx]->src[SMPC_INFO_PEER].addr_type;

    /* IOcap is  three octets with the most significant octet as the AuthReq parameter,
     * the middle octet as the OOB data flag and the least significant octet as the IO
     * capability parameter.
     */

    if (gapc_get_role(conidx) == ROLE_MASTER)
    {
        IOcap_local[2] = gapc_env[conidx]->smpc.info.pair->pair_req_feat.auth;
        IOcap_local[1] = gapc_env[conidx]->smpc.info.pair->pair_req_feat.oob;
        IOcap_local[0] = gapc_env[conidx]->smpc.info.pair->pair_req_feat.iocap;
    }
    else
    {
        IOcap_local[2] = gapc_env[conidx]->smpc.info.pair->pair_rsp_feat.auth;
        IOcap_local[1] = gapc_env[conidx]->smpc.info.pair->pair_rsp_feat.oob;
        IOcap_local[0] = gapc_env[conidx]->smpc.info.pair->pair_rsp_feat.iocap;
    }

    /* The DHkey Check is performed using f6.
     *  Kept as separate function as the inputs to F6 are dependent on the Association model used for the pairing.
     *
     *  --------------------------------------------
     *  |  Numeric Comparison/Just Works
     *  --------------------------------------------
     *  |    Ea = f6(MacKey, Na, Nb, 0,IOcapA, A, B)
     *  |    Eb = f6(MacKey, Nb, Na, 0,IOcapB, B, A)
     *  |
     *  |    E = f6(MaxKey,Nlocal,Npeer,IOcap_local,Addr_local,Addr_Peer)
     *  |===========================================
     *  |  Out Of Band
     *  --------------------------------------------
     *  |    Ea = f6(MacKey, Na, Nb, rb,IOcapA, A, B)
     *  |    Eb = f6(MacKey, Nb, Na, ra,IOcapB, B, A)
     *  |===========================================
     *  |  PassKey
     *  --------------------------------------------
     *  |    Ea = f6(MacKey, Na20, Nb20, rb,IOcapA, A, B)
     *  |    Eb = f6(MacKey, Nb20, Na20, ra,IOcapB, B, A)
     *  |===========================================
     ******************************************************/

    switch (gapc_env[conidx]->smpc.info.pair->pair_method)
    {
    case SMPC_METH_NC  :
    case SMPC_METH_JW  :
    {
        uint8_t r_zero[16];
        memset(r_zero, 0x00, 16);

        smpc_f6_init(conidx, p_MacKey, Nlocal, Npeer, r_zero, IOcap_local, Addr_local, Addr_peer);

    }
    break;

    case SMPC_METH_PK :
    {
        uint8_t *R = &gapc_env[conidx]->smpc.info.pair->local_r[0];
        smpc_f6_init(conidx, p_MacKey, Nlocal, Npeer, R, IOcap_local, Addr_local, Addr_peer);
    }
    break;

    case SMPC_METH_OOB :
    {
        uint8_t *R = &gapc_env[conidx]->smpc.info.pair->peer_r[0];
        smpc_f6_init(conidx, p_MacKey, Nlocal, Npeer, R, IOcap_local, Addr_local, Addr_peer);
    }
    break;
    default :
        break;

    }
}

void smpc_handle_dh_key_check_complete(uint8_t idx, const uint8_t *dh_key)
{
    // check that pairing is still on-going
    if (gapc_env[idx]->smpc.info.pair != NULL)
    {
        // mark local DHKey calculated
        gapc_env[idx]->smpc.info.pair->dh_key_calculation_complete = true;
        memcpy((void *)&gapc_env[idx]->smpc.info.pair->dh_key_local[0], dh_key, DH_KEY_LEN);

        if (gapc_env[idx]->smpc.state > SMPC_PAIRING_SC_AUTH_START_PHASE_2)
        {
            // calculate mac key
            smpc_init_mac_key_calculation(idx);
        }
    }
}

// Begin the next phase of Secure Connection Authentication
void smpc_init_mac_key_calculation(uint8_t idx)
{
    uint8_t role        = gapc_get_role(idx);

    if (gapc_env[idx]->smpc.info.pair->dh_key_calculation_complete == true)
    {
        // Begin f5
        // MacKey || LTK = f5(DHKey, N_master, N_slave, BD_ADDR_master, BD_ADDR_slave)

        uint8_t *dh_key = &gapc_env[idx]->smpc.info.pair->dh_key_local[0];

        uint8_t *N_master;
        uint8_t *N_slave;
        uint8_t BD_ADDR_master[7];
        uint8_t BD_ADDR_slave[7];

        if (role == ROLE_MASTER)
        {
            N_master = &gapc_env[idx]->smpc.info.pair->rand[0];
            N_slave = &gapc_env[idx]->smpc.info.pair->rem_rand[0];

            memcpy(&BD_ADDR_master[0], &gapc_env[idx]->src[SMPC_INFO_LOCAL].addr, 6);
            memcpy(&BD_ADDR_slave[0], &gapc_env[idx]->src[SMPC_INFO_PEER].addr, 6);

            BD_ADDR_master[6] = gapc_env[idx]->src[SMPC_INFO_LOCAL].addr_type;
            BD_ADDR_slave[6] = gapc_env[idx]->src[SMPC_INFO_PEER].addr_type;
        }
        else
        {
            N_master = &gapc_env[idx]->smpc.info.pair->rem_rand[0];
            N_slave = &gapc_env[idx]->smpc.info.pair->rand[0];

            memcpy(&BD_ADDR_master[0], &gapc_env[idx]->src[SMPC_INFO_PEER].addr, 6);
            memcpy(&BD_ADDR_slave[0], &gapc_env[idx]->src[SMPC_INFO_LOCAL].addr, 6);

            BD_ADDR_master[6] = gapc_env[idx]->src[SMPC_INFO_PEER].addr_type;
            BD_ADDR_slave[6] = gapc_env[idx]->src[SMPC_INFO_LOCAL].addr_type;
        }
        gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_W4_F5_P1;
        smpc_f5_init(idx, dh_key, N_master, N_slave, BD_ADDR_master, BD_ADDR_slave);
    }
    else
    {
        // Must Wait for the DH_Key Calculation to complete.
        gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_W4_DHKEY_KEY_COMPLETE;
    }
}


void smpc_initiate_dhkey_verification(uint8_t conidx)
{
    uint8_t *p_MacKey;
    uint8_t  IOcap_peer[3];

    uint8_t *Nlocal = &gapc_env[conidx]->smpc.info.pair->rand[0];
    uint8_t *Npeer = &gapc_env[conidx]->smpc.info.pair->rem_rand[0];
    uint8_t Addr_local[7];
    uint8_t Addr_peer[7];

    p_MacKey = &gapc_env[conidx]->smpc.info.pair->MacKey[0];

    // Construct the Address for local and peer.
    memcpy(&Addr_local[0], &gapc_env[conidx]->src[SMPC_INFO_LOCAL].addr, 6);
    memcpy(&Addr_peer[0], &gapc_env[conidx]->src[SMPC_INFO_PEER].addr, 6);

    Addr_local[6] = gapc_env[conidx]->src[SMPC_INFO_LOCAL].addr_type;
    Addr_peer[6] = gapc_env[conidx]->src[SMPC_INFO_PEER].addr_type;

    /* IOcap is  three octets with the most significant octet as the AuthReq parameter,
     * the middle octet as the OOB data flag and the least significant octet as the IO
     * capability parameter.
     */

    if (gapc_get_role(conidx) == ROLE_MASTER)
    {
        IOcap_peer[2] = gapc_env[conidx]->smpc.info.pair->pair_rsp_feat.auth;
        IOcap_peer[1] = gapc_env[conidx]->smpc.info.pair->pair_rsp_feat.oob;
        IOcap_peer[0] = gapc_env[conidx]->smpc.info.pair->pair_rsp_feat.iocap;
    }
    else
    {
        IOcap_peer[2] = gapc_env[conidx]->smpc.info.pair->pair_req_feat.auth;
        IOcap_peer[1] = gapc_env[conidx]->smpc.info.pair->pair_req_feat.oob;
        IOcap_peer[0] = gapc_env[conidx]->smpc.info.pair->pair_req_feat.iocap;
    }

    /* The DHkey Check is performed using f6.
     *  Kept as separate function as the inputs to F6 are dependent on the Association model used for the pairing.
     *
     *  --------------------------------------------
     *  |  Numeric Comparison/Just Works
     *  --------------------------------------------
     *  |    Ea = f6(MacKey, Na, Nb, 0,IOcapA, A, B)
     *  |    Eb = f6(MacKey, Nb, Na, 0,IOcapB, B, A)
     *  |
     *  |    E = f6(MacKey,Npeer,Nlocal,IOcap_peer,Addr_Peer,Addr_local)
     *  |===========================================
     *  |  Out Of Band
     *  --------------------------------------------
     *  |    Ea = f6(MacKey, Na, Nb, rb,IOcapA, A, B)
     *  |    Eb = f6(MacKey, Nb, Na, ra,IOcapB, B, A)
     *  |===========================================
     *  |  PassKey
     *  --------------------------------------------
     *  |    Ea = f6(MacKey, Na20, Nb20, rb,IOcapA, A, B)
     *  |    Eb = f6(MacKey, Nb20, Na20, ra,IOcapB, B, A)
     *  |===========================================
     ******************************************************/

    switch (gapc_env[conidx]->smpc.info.pair->pair_method)
    {
    case SMPC_METH_NC  :
    case SMPC_METH_JW  :
    {
        uint8_t R_zero[16];
        memset(R_zero, 0x00, 16);
        // Normal Code
        smpc_f6_init(conidx, p_MacKey, Npeer, Nlocal, R_zero, IOcap_peer, Addr_peer, Addr_local);

    }
    break;

    case SMPC_METH_PK :
    {
        // Ra = Rb = TK  :- the Passkey stored as a 16 byte array.
        // Passkey passed from the API is contained in the TK.

        uint8_t *R = &gapc_env[conidx]->smpc.info.pair->local_r[0];
        smpc_f6_init(conidx, p_MacKey, Npeer, Nlocal, R, IOcap_peer, Addr_peer, Addr_local);
    }
    break;

    case SMPC_METH_OOB :
    {
        // Check if Eb = f6(MacKey, Nb, Na, Rlocal, IOCapB, B, A)
        uint8_t *R = &gapc_env[conidx]->smpc.info.pair->local_r[0];
        smpc_f6_init(conidx, p_MacKey, Npeer, Nlocal, R, IOcap_peer, Addr_peer, Addr_local);
    }
    break;

    default :
        break;

    }
}

bool smpc_secure_connections_enabled(uint8_t idx)
{
    return gapc_env[idx]->smpc.secure_connections_enabled;
}
#endif // (SECURE_CONNECTIONS)


#endif //(BLE_SMPC)
/// @} SMPC
