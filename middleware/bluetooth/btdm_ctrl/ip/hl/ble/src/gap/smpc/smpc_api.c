/**
 ****************************************************************************************
 *
 * @file smpc_api.c
 *
 * @brief SMPC API implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup SMPC_API
 * @ingroup SMPC
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#if (BLE_SMPC)

#include "smpc_api.h"

#include "co_math.h"
#include "co_error.h"
#include "co_endian.h"
#include "co_utils.h"

#include <string.h>

#include "ke_timer.h"
#include "gap.h"
#include "gapc.h"
#include "gapm.h"
#include "../gapc/gapc_int.h"  // Internal are required

#include "l2cc_task.h"

#include "smpc_util.h"
#include "smpc_int.h"
#include "smpc_crypto.h"

#include "ke_mem.h"

#include "hci.h"

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (BLE_CENTRAL)
uint8_t smpc_pairing_start(uint8_t idx, struct gapc_pairing  *pairing)
{
    uint8_t status = GAP_ERR_NO_ERROR;

    ASSERT_ERR(gapc_env[idx]->smpc.info.pair == NULL);

    do
    {
        /* ------------------------------------------------
         * Check SMP Timeout timer state
         * ------------------------------------------------*/
        if (SMPC_IS_FLAG_SET(idx, SMPC_TIMER_TIMEOUT_BLOCKED_FLAG))
        {
            /*
             * Once a timeout has occurred, no security procedure can be initiated until a new
             * physical link has been established.
             */
            status = GAP_ERR_TIMEOUT;
            break;
        }

        /* ------------------------------------------------
         * Check Repeated Attempts Timer state
         * ------------------------------------------------*/
        if (SMPC_IS_FLAG_SET(idx, SMPC_TIMER_REP_ATT_FLAG))
        {
            status = SMP_GEN_PAIR_FAIL_REASON(SMP_PAIR_FAIL_REASON_MASK,
                                              SMP_ERROR_REPEATED_ATTEMPTS);
            break;
        }

        /* ------------------------------------------------
         * Check provided parameters
         * ------------------------------------------------*/
        if (!smpc_check_pairing_feat(pairing))
        {
            status = SMP_GEN_PAIR_FAIL_REASON(SMP_PAIR_FAIL_REASON_MASK,
                                              SMP_ERROR_INVALID_PARAM);
            break;
        }

        /* ------------------------------------------------
         * Check Encryption Key Size
         * ------------------------------------------------*/
        if ((pairing->key_size < SMPC_MIN_ENC_SIZE_LEN) ||
                (pairing->key_size > SMPC_MAX_ENC_SIZE_LEN))
        {
            status = SMP_GEN_PAIR_FAIL_REASON(SMP_PAIR_FAIL_REASON_MASK,
                                              SMP_ERROR_INVALID_PARAM);
            break;
        }

        /* ------------------------------------------------
         * Check pairing mode accepted
         * ------------------------------------------------*/
        if ((((pairing->auth & GAP_AUTH_SEC_CON) != 0) && !gapm_is_sec_con_pairing_supp())
                || (((pairing->auth & GAP_AUTH_SEC_CON) == 0) && !gapm_is_legacy_pairing_supp()))
        {
            status = SMP_GEN_PAIR_FAIL_REASON(SMP_PAIR_FAIL_REASON_MASK, SMP_ERROR_AUTH_REQ);
            break;
        }

        // Allocate memory for the pairing information structure
        gapc_env[idx]->smpc.info.pair = (struct smpc_pair_info *)ke_malloc(sizeof(struct smpc_pair_info), KE_MEM_KE_MSG);

        struct smpc_pair_info *pair_info = (struct smpc_pair_info *)gapc_env[idx]->smpc.info.pair;

        memset(pair_info, 0x00, sizeof(struct smpc_pair_info));

        // Copy the pairing features
        memcpy(&pair_info->pair_req_feat, pairing, sizeof(struct gapc_pairing));

        // If device is not bondable, useless to ask for keys
        if ((pair_info->pair_req_feat.auth & GAP_AUTH_BOND) != GAP_AUTH_BOND)
        {
            pair_info->pair_req_feat.ikey_dist = 0x00;
            pair_info->pair_req_feat.rkey_dist = 0x00;
        }

        // Send the pairing request PDU to the peer device
        smpc_pdu_send(idx, L2C_CODE_PAIRING_REQ, &pair_info->pair_req_feat);
        // Waiting for the pairing response
        gapc_env[idx]->smpc.state = SMPC_PAIRING_RSP_WAIT;

    }
    while (0);

    return (status);
}
#endif //(BLE_CENTRAL)

uint8_t smpc_pairing_tk_exch(uint8_t idx, bool accept,  struct gap_sec_key *tk)
{
    uint8_t status = GAP_ERR_NO_ERROR;

    if ((gapc_env[idx]->smpc.state == SMPC_PAIRING_TK_WAIT)
            || (gapc_env[idx]->smpc.state == SMPC_PAIRING_TK_WAIT_CONF_RCV))
    {
        bool gen_rand_nb = false;
        // Role
        uint8_t role        = gapc_get_role(idx);

        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);

        // If key is null, pairing fails
        if (accept)
        {
#if (SECURE_CONNECTIONS)
            if (smpc_secure_connections_enabled(idx) == true)
            {
                // In  Secure Connections - the TK exchange is used for Numeric Comparison and
                // Passkey Entry
                //
                if ((gapc_env[idx]->smpc.state == SMPC_PAIRING_TK_WAIT) ||
                        (gapc_env[idx]->smpc.state == SMPC_PAIRING_TK_WAIT_CONF_RCV))
                {
#ifndef SMP_SLIM
                    if (gapc_env[idx]->smpc.info.pair->pair_method == SMPC_METH_PK)
                    {

                        // Init the PassKey
                        // Store the PassKey - 20 bits significant.
                        memcpy(&gapc_env[idx]->smpc.info.pair->local_r[0], tk->key, GAP_KEY_LEN);
                        gapc_env[idx]->smpc.info.pair->passkey = co_read32p(&(tk->key[0]));
                        gapc_env[idx]->smpc.info.pair->passkey_bit_count = 0; // First Round
                        if (role == ROLE_MASTER)
                        {
                            gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_PASSKEY_W4_LOCAL_RAND_N_P1;
                            smpc_generate_rand(idx, SMPC_PAIRING_SC_PASSKEY_W4_LOCAL_RAND_N_P1);
                        }
                        else
                        {
                            // gf 23 July
                            if (gapc_env[idx]->smpc.info.pair->peer_confirm_received == true)
                            {
                                // Request Random Number from HCI
                                gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_PASSKEY_W4_LOCAL_RAND_N_P1;
                                smpc_generate_rand(idx, SMPC_PAIRING_SC_PASSKEY_W4_LOCAL_RAND_N_P1);
                            }
                            else
                            {
                                // We have not received the Confirm Value from the peer passkey.
                                // Wait for the Peer Commitment
                                gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_PASSKEY_W4_PEER_COMMITMENT;

                            }
                        }

                    }
#else // !SMP_SLIM
                    ASSERT_ERR(0);
#endif // !SMP_SLIM

                }
            }
            else
#endif // (SECURE_CONNECTIONS)
            {
                // Keep TK
                memcpy(&gapc_env[idx]->smpc.info.pair->key.key[0], tk, GAP_KEY_LEN);

                if (role == ROLE_MASTER)
                {
#if (BLE_CENTRAL)
                    // Verify the state
                    if (gapc_env[idx]->smpc.state == SMPC_PAIRING_TK_WAIT)
                    {
                        gen_rand_nb = true;
                    }
                    // Else drop the message, the message has been sent without any reason.
#endif //(BLE_CENTRAL)
                }
                else    // role == ROLE_SLAVE
                {
#if (BLE_PERIPHERAL)
                    if (gapc_env[idx]->smpc.state == SMPC_PAIRING_TK_WAIT)
                    {
                        // Wait for the master device confirm value
                        gapc_env[idx]->smpc.state = SMPC_PAIRING_WAIT_CONFIRM;
                    }
                    else if (gapc_env[idx]->smpc.state == SMPC_PAIRING_TK_WAIT_CONF_RCV)
                    {
                        gen_rand_nb = true;
                    }
                    // Else drop the message, the message has been sent without any reason.
#endif //(BLE_PERIPHERAL)
                }

                if (gen_rand_nb)
                {
                    // Generate Random Number
                    smpc_generate_rand(idx, SMPC_PAIRING_GEN_RAND_P1);
                }
            }
        }
        else
        {
            if (gapc_env[idx]->smpc.info.pair->pair_method == SMPC_METH_PK)
            {
                // TK could not be retrieved in the Host
                status = SMP_ERROR_PASSKEY_ENTRY_FAILED;
            }
        }
    }

    return (status);
}


uint8_t smpc_pairing_ltk_exch(uint8_t idx, struct gapc_ltk *ltk)
{
    uint8_t status = GAP_ERR_NO_ERROR;
    uint8_t role        = gapc_get_role(idx);

    // Verify the state
    if (gapc_env[idx]->smpc.state == SMPC_PAIRING_APP_LTK_WAIT)
    {
        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);

        // Counter
        uint8_t counter;
        // Master ID Information
        struct smpc_mst_id_info mst_id_info;

        ASSERT_ERR(gapc_enc_keysize_get(idx) <= GAP_KEY_LEN);

        // Mask the MSB to respect key size (Key is stored LSB->MSB)
        for (counter = gapc_enc_keysize_get(idx); counter < GAP_KEY_LEN; counter++)
        {
            ltk->ltk.key[counter] = 0x00;
        }

        // Send MST_ID PDU with EDIV and RandNb
        mst_id_info.ediv = ltk->ediv;
        memcpy(&(mst_id_info.randnb[0]), &(ltk->randnb.nb[0]), GAP_RAND_NB_LEN);

        // Send the LTK to the peer device
        smpc_pdu_send(idx, L2C_CODE_ENCRYPTION_INF, (void *) & (ltk->ltk.key[0]));
        // Send the BD Address to the peer device
        smpc_pdu_send(idx, L2C_CODE_MASTER_ID, (void *)&mst_id_info);

        // continue key exchange execution
        smpc_tkdp_send_continue(idx, role);
    }
    // Else drop the message

    return (status);
}

uint8_t smpc_pairing_irk_exch(uint8_t idx, struct gap_sec_key *irk, struct gap_bdaddr *identity)
{
    uint8_t status = GAP_ERR_NO_ERROR;
    uint8_t role   = gapc_get_role(idx);

    // Verify the state
    if (gapc_env[idx]->smpc.state == SMPC_PAIRING_APP_IRK_WAIT)
    {
        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);

        // Send the IRK to the peer device
        smpc_pdu_send(idx, L2C_CODE_IDENTITY_INF, irk);
        // Send the BD Address to the peer device
        smpc_pdu_send(idx, L2C_CODE_ID_ADDR_INF, identity);
        // continue key exchange
        smpc_tkdp_send_continue(idx, role);
    }
    // Else drop the message

    return (status);
}

#if (SECURE_CONNECTIONS)
#ifndef SMP_SLIM
uint8_t  smpc_pairing_oob_exch(uint8_t idx, bool accept, struct gapc_oob *oob)
{
    uint8_t status = GAP_ERR_NO_ERROR;

    // Device don't support OOB at all
    if (!accept)
    {
        status = SMP_ERROR_OOB_NOT_AVAILABLE;
    }

    // Check if Pairing Method is OOB
    else if ((gapc_env[idx]->smpc.info.pair->pair_method ==  SMPC_METH_OOB) &&
             (gapc_env[idx]->smpc.state == SMPC_PAIRING_SC_OOB_W4_OOB_DATA))
    {
        public_key_t *Pk = &(gapc_env[idx]->smpc.info.pair->peer_public_key);
        bool zero_rnd = true;
        int32_t i;

        // check if received random number is valid or not.
        for (i = (GAP_KEY_LEN - 1) ; i > 0 ; i--)
        {
            if (oob->rand[i] != 0)
            {
                zero_rnd = false;
                break;
            }

        }

        // Copy the Confirm and Rand values from the peer.

        memcpy(&(gapc_env[idx]->smpc.info.pair->conf_value[0]),
               &(oob->conf[0]), CFM_LEN);

        memcpy(&(gapc_env[idx]->smpc.info.pair->peer_r[0]), &(oob->rand[0]),
               RAND_VAL_LEN);

        if (!zero_rnd)
        {
            gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_OOB_W4_F4_COMMITMENT_CHECK;
            //  initiate the Verification of the peers confirm.
            // Ca = (PKa, PKa, ra, 0) if master
            // Cb = (PKb, PKb, rb, 0) if slave
            smpc_f4_Init(idx, Pk->x, Pk->x, &oob->rand[0], 0x00);
        }
        // do not run F4 if peer device is not able to send oob data
        else
        {
            uint8_t role = gapc_get_role(idx);

            // Get a local Random number
            if ((role == ROLE_MASTER) ||
                    ((role == ROLE_SLAVE) && (gapc_env[idx]->smpc.info.pair->peer_rand_received == true)))
            {
                gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_OOB_W4_LOCAL_RAND_N_P1;
                smpc_generate_rand(idx, SMPC_PAIRING_SC_OOB_W4_LOCAL_RAND_N_P1);
            }
            else
            {
                // A slave - waits for the peers rand if it has not already been recieved.
                gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_OOB_W4_PEER_RAND;
            }
        }
    }

    return status;
}
#endif // !SMP_SLIM

uint8_t smpc_pairing_nc_exch(uint8_t idx, uint8_t accept)
{
    uint8_t status = GAP_ERR_NO_ERROR;

    if ((gapc_env[idx]->smpc.info.pair->pair_method ==  SMPC_METH_NC) &&
            (gapc_env[idx]->smpc.state == SMPC_PAIRING_SC_W4_NC_ACCEPT))
    {
        if (accept)
        {
            // Begin the next phase of Secure Connection Authentication
            smpc_init_mac_key_calculation(idx);
        }
        else
        {
            // Pairing Failed -- Send Error to Peer
            uint8_t status = SMP_ERROR_NUMERIC_COMPARISON_FAILED;

            smpc_pdu_send(idx, L2C_CODE_PAIRING_FAILED, &status);

            smpc_pairing_end(idx, gapc_get_role(idx),
                             SMP_GEN_PAIR_FAIL_REASON(SMP_PAIR_FAIL_REASON_MASK, status), true);
        }
    }

    return status;
}

#endif // (SECURE_CONNECTIONS)

uint8_t smpc_pairing_csrk_exch(uint8_t idx, struct gap_sec_key *csrk)
{
    uint8_t status = GAP_ERR_NO_ERROR;
    uint8_t role        = gapc_get_role(idx);

    // Verify the state
    if (gapc_env[idx]->smpc.state == SMPC_PAIRING_APP_CSRK_WAIT)
    {
        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);

        // update connection environment variables
        memcpy(&(gapc_env[idx]->smpc.csrk[SMPC_INFO_LOCAL].key[0]), csrk, sizeof(struct gap_sec_key));
        // reset sign counter
        gapc_env[idx]->smpc.sign_counter[SMPC_INFO_LOCAL] = 0;

        // Send the PDU to the peer device
        smpc_pdu_send(idx, L2C_CODE_SIGNING_INF, (void *)csrk);

        // continue key exchange algorithm
        smpc_tkdp_send_continue(idx, role);
    }

    return (status);
}


uint8_t smpc_pairing_rsp(uint8_t idx, bool accept, struct gapc_pairing *feat)
{
    uint8_t status = GAP_ERR_NO_ERROR;
#if (BLE_PERIPHERAL)
    uint8_t role        = gapc_get_role(idx);

    // Only the slave device provides its pairing features in a SMPC_PAIRING_CFM message
    if (role == ROLE_SLAVE)
    {
        struct gapc_pairing *rsp_feat = &(gapc_env[idx]->smpc.info.pair->pair_rsp_feat);
        struct gapc_pairing *req_feat = &(gapc_env[idx]->smpc.info.pair->pair_req_feat);

        if (gapc_env[idx]->smpc.state == SMPC_PAIRING_FEAT_WAIT)
        {
            if (accept)
            {
                ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);

                // Store the pairing features of the slave
                memcpy(rsp_feat, feat, sizeof(struct gapc_pairing));

                // if secure connection not supported, ensure that Secure connection flag is not set
                if (!gapm_is_sec_con_pairing_supp())
                {
                    rsp_feat->auth &= ~GAP_AUTH_SEC_CON;
                }

#if (SECURE_CONNECTIONS)
                if ((rsp_feat->auth & GAP_AUTH_SEC_CON) &&
                        (req_feat->auth & GAP_AUTH_SEC_CON))

                {
                    gapc_env[idx]->smpc.secure_connections_enabled = true;
                }
#endif // (SECURE_CONNECTIONS)

                do
                {
                    // Check that secure connection mode is not required
#if (SECURE_CONNECTIONS)
                    if (!gapc_env[idx]->smpc.secure_connections_enabled && !gapm_is_legacy_pairing_supp())
                    {
                        status = SMP_ERROR_AUTH_REQ;
                        break;
                    }
#endif // (SECURE_CONNECTIONS)

                    /*
                     * On legacy pairing If the slave device has not found OOB data for the master while the master has
                     * OOB data, the pairing will failed with a OOB Not Available error status
                     */
                    if ((feat->oob == 0) && (req_feat->oob == 1)
#if (SECURE_CONNECTIONS)
                            && !gapc_env[idx]->smpc.secure_connections_enabled
#endif // (SECURE_CONNECTIONS)
                       )
                    {
                        // OOB status mismatch
                        status = SMP_ERROR_OOB_NOT_AVAILABLE;
                        break;
                    }

                    if (!smpc_check_max_key_size(idx))
                    {
                        // Resultant encryption key size is below the minimal accepted value
                        status = SMP_ERROR_ENC_KEY_SIZE;
                        break;
                    }
#if (SECURE_CONNECTIONS)
                    if (!smpc_check_key_distrib(idx, feat->sec_req) && (gapc_env[idx]->smpc.secure_connections_enabled == false))
                    {
                        // The resultant key distribution doesn't match with the provided parameters
                        status = SMP_ERROR_UNSPECIFIED_REASON;
                        break;
                    }
#else
                    if (!smpc_check_key_distrib(idx, feat->sec_req))
                    {
                        // The resultant key distribution doesn't match with the provided parameters
                        status = SMP_ERROR_UNSPECIFIED_REASON;
                        break;
                    }
#endif //  (SECURE_CONNECTIONS)

                    // Select security properties and STK generation type
                    smpc_get_key_sec_prop(idx);

                    // Check if the required security mode can be reached
                    if (!smpc_is_sec_mode_reached(idx, ROLE_SLAVE))
                    {
                        /*
                         * The pairing procedure cannot be performed as authentication requirements cannot
                         * be met due to IO capabilities of one or both devices.
                         */
                        status = SMP_ERROR_AUTH_REQ;
                        break;
                    }

                    // Adjust the pairing features according on master's
                    // If device is not bondable, useless to ask for keys

                    if (((rsp_feat->auth & 0x07) == GAP_AUTH_REQ_NO_MITM_NO_BOND) ||
                            ((rsp_feat->auth & 0x07) == GAP_AUTH_REQ_MITM_NO_BOND))
                    {
                        rsp_feat->ikey_dist    = 0x00;
                        rsp_feat->rkey_dist    = 0x00;
                    }
                    else
                    {
                        rsp_feat->ikey_dist &= req_feat->ikey_dist;
                        rsp_feat->rkey_dist &= req_feat->rkey_dist;
                    }

                    if (status == GAP_ERR_NO_ERROR)
                    {
                        // Can send the Pairing Response PDU
                        smpc_pdu_send(idx, L2C_CODE_PAIRING_RESP, rsp_feat);
#if (SECURE_CONNECTIONS)
                        if (smpc_secure_connections_enabled(idx) == true)
                        {
                            gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_W4_PEER_PUBLIC_KEY;
                        }
                        else //  (SECURE_CONNECTIONS)
#endif
                            if (gapc_env[idx]->smpc.info.pair->pair_method == SMPC_METH_JW)
                            {
                                gapc_env[idx]->smpc.state = SMPC_PAIRING_WAIT_CONFIRM;
                            }
                            else
                            {
                                // Send a TK request to the HL
                                smpc_send_pairing_req_ind(idx, GAPC_TK_EXCH);
                            }
                    }
                }
                while (0);
            }
            else
            {
                // Pairing will fail due to application error
                status = SMP_ERROR_UNSPECIFIED_REASON;
            }
        }
        // Else drop the message
    }
    // Else drop the message
#endif //(BLE_PERIPHERAL)

    return (status);
}

#if (BLE_PERIPHERAL)

void smpc_pairing_req_handler(uint8_t idx, struct gapc_pairing *feat)
{
    ASSERT_ERR(gapc_env[idx]->smpc.info.pair == NULL);

    // Allocate memory for the pairing information structure
    gapc_env[idx]->smpc.info.pair = (struct smpc_pair_info *)ke_malloc(sizeof(struct smpc_pair_info), KE_MEM_KE_MSG);
    ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);
    // Check if memory has been allocated
    struct smpc_pair_info *pair_info = (struct smpc_pair_info *)gapc_env[idx]->smpc.info.pair;

    memset(pair_info, 0x00, sizeof(struct smpc_pair_info));

    // Keep the packet for later use
    memcpy(&(gapc_env[idx]->smpc.info.pair->pair_req_feat), feat, SMPC_CODE_PAIRING_REQ_RESP_LEN - 1);

    // Inform the HL that a pairing request has been received
    smpc_send_pairing_req_ind(idx, GAPC_PAIRING_REQ);
}


uint8_t smpc_security_req_send(uint8_t idx, uint8_t auth)
{
    // Command Status
    uint8_t status      = GAP_ERR_NO_ERROR;

    do
    {
        /* ------------------------------------------------
         * Check SMP Timeout timer state
         * ------------------------------------------------*/
        if (SMPC_IS_FLAG_SET(idx, SMPC_TIMER_TIMEOUT_BLOCKED_FLAG))
        {
            /*
             * Once a timeout has occurred, no security procedure can be initiated until a new
             * physical link has been established.
             */
            status = GAP_ERR_TIMEOUT;
            break;
        }

        /* ------------------------------------------------
         * Check Repeated Attempts Timer state
         * ------------------------------------------------*/
        if (SMPC_IS_FLAG_SET(idx, SMPC_TIMER_REP_ATT_FLAG))
        {
            status = SMP_GEN_PAIR_FAIL_REASON(SMP_PAIR_FAIL_REASON_MASK,
                                              SMP_ERROR_REPEATED_ATTEMPTS);
            break;
        }

        /* ------------------------------------------------
         * Check pairing mode accepted
         * ------------------------------------------------*/
        if ((((auth & GAP_AUTH_SEC_CON) != 0) && !gapm_is_sec_con_pairing_supp())
                || (((auth & GAP_AUTH_SEC_CON) == 0) && !gapm_is_legacy_pairing_supp()))
        {
            status = SMP_GEN_PAIR_FAIL_REASON(SMP_PAIR_FAIL_REASON_MASK, SMP_ERROR_AUTH_REQ);
            break;
        }


        // State is either SMPC_IDLE or SMPC_BUSY
        // Send the pairing request PDU to the peer device
        smpc_pdu_send(idx, L2C_CODE_SECURITY_REQ, (void *)&auth);

    }
    while (0);

    return (status);
}
#endif //(BLE_PERIPHERAL)



#if (BLE_CENTRAL)
uint8_t smpc_encrypt_start(uint8_t idx, struct gapc_ltk *ltk)
{
    // Command Status
    uint8_t status      = GAP_ERR_NO_ERROR;

    // Send start encryption to LL
    smpc_send_start_enc_cmd(idx, SMPC_USE_LTK, &(ltk->ltk.key[0]), &(ltk->randnb.nb[0]), ltk->ediv);
    // store encryption key size
    gapc_enc_keysize_set(idx, ltk->key_size);
    return (status);
}
#endif //(BLE_CENTRAL)


#if (BLE_PERIPHERAL)
void smpc_encrypt_start_handler(uint8_t idx, struct gapc_ltk *ltk)
{
    // Indicate the host that a LTK is required
    struct gapc_encrypt_req_ind  *ind = KE_MSG_ALLOC(GAPC_ENCRYPT_REQ_IND,
                                        gapc_get_dest_task(idx), KE_BUILD_ID(TASK_GAPC, idx),
                                        gapc_encrypt_req_ind);

    // Set EDIV
    ind->ediv = ltk->ediv;
    // Set Random Value
    memcpy(&ind->rand_nb, &ltk->randnb, sizeof(rand_nb_t));

    ke_msg_send(ind);
}

void smpc_encrypt_cfm(uint8_t idx, bool accept, struct gap_sec_key *ltk, uint8_t key_size)
{
    // Check if a LTK has been found
    if (accept)
    {
        // Update the procedure state value
        gapc_env[idx]->smpc.state = SMPC_START_ENC_LTK;

        // Send the LTK to the controller so that it can start the encryption
        smpc_send_ltk_req_rsp(idx, true, (uint8_t *)ltk);

        // Update Link key size
        gapc_env[idx]->smpc.key_size = key_size;
    }
    else
    {
        smpc_send_ltk_req_rsp(idx, false, NULL);
    }
}
#endif //(BLE_PERIPHERAL)

#ifndef SMP_SLIM
uint8_t smpc_sign_command(uint8_t idx, struct gapc_sign_cmd *param)
{
    // Command Status
    uint8_t status      = GAP_ERR_NO_ERROR;
    //Source of the CSRK
    uint8_t csrk_src = SMPC_INFO_LOCAL;


    ASSERT_ERR(gapc_env[idx]->smpc.info.sign == NULL);

    // Allocate memory for the signature information structure
    gapc_env[idx]->smpc.info.sign = (struct smpc_sign_info *)ke_malloc(sizeof(struct smpc_sign_info), KE_MEM_KE_MSG);


    // Set C0 value = 0[0:127]
    memset(&(gapc_env[idx]->smpc.info.sign->cn1[0]), 0x00, GAP_KEY_LEN);

    if (param->operation == GAPC_SIGN_PACKET)
    {
        /*
         * **************************
         * * DATA PDU * SIGNCOUNTER *
         * **************************
         * LSB -----------------> MSB
         */

        uint32_t local_sign_counter = gapc_get_sign_counter(idx, SMPC_INFO_LOCAL);

        // Put the SignCounter in the message to sign
        co_write32p(&param->msg[param->byte_len - SMPC_SIGN_COUNTER_LEN], local_sign_counter);

        // Block counter value is the number of 128-bits block in the Data PDU
        gapc_env[idx]->smpc.info.sign->block_nb   = (uint16_t)((((param->byte_len << 1) - 1) / (GAP_KEY_LEN << 1)) + 1);
        // Set the message offset
        gapc_env[idx]->smpc.info.sign->msg_offset = param->byte_len;
    }
    else // if (param->operation == GAPC_SIGN_CHECK)
    {
        /*
         * ******************************************
         * * DATA PDU * SIGNCOUNTER * MAC (8 bytes) *
         * ******************************************
         * LSB ---------------------------------> MSB
         */

        ASSERT_ERR(param->byte_len > SMPC_SIGN_LEN);

        // Peer SignCounter
        uint32_t peer_sign_counter;
        // Received SignCounter
        uint32_t rcv_sign_counter;

        co_write32(&rcv_sign_counter, (uint32_t)param->msg[param->byte_len - SMPC_SIGN_LEN]);

        // Stored SignCounter
        peer_sign_counter = gapc_get_sign_counter(idx, SMPC_INFO_PEER);

        // Check signCounter
        if (rcv_sign_counter >= peer_sign_counter)
        {
            // Block counter value is the number of 128-bits block in the Data PDU - MAC not used
            gapc_env[idx]->smpc.info.sign->block_nb
                = (uint16_t)(((((param->byte_len - SMPC_SIGN_MAC_LEN) << 1) - 1) / (GAP_KEY_LEN << 1)) + 1);

            // Set the message offset
            gapc_env[idx]->smpc.info.sign->msg_offset = param->byte_len - SMPC_SIGN_MAC_LEN;

            csrk_src = SMPC_INFO_PEER;
        }
        else
        {
            status = SMP_ERROR_SIGN_VERIF_FAIL;
        }
    }

    if (status == GAP_ERR_NO_ERROR)
    {
        // Size of PDU to sign is lower than or equal to 128bits
        if (gapc_env[idx]->smpc.info.sign->block_nb == 1)
        {
            /*
             * ---------------------------------------------------
             * GENERATE L ----------------------------------------
             *----------------------------------------------------
             */
            smpc_generate_l(idx, csrk_src);
        }
        else    // Size of PDU to sign is upper than 128bits
        {
            ASSERT_ERR(gapc_env[idx]->smpc.info.sign->msg_offset > GAP_KEY_LEN);

            /*
             * ---------------------------------------------------
             * GENERATE C1 ---------------------------------------
             *----------------------------------------------------
             */
            smpc_generate_ci(idx, csrk_src,
                             &gapc_env[idx]->smpc.info.sign->cn1[0],
                             &param->msg[gapc_env[idx]->smpc.info.sign->msg_offset - GAP_KEY_LEN]);
        }
    }

    return (status);
}


void smpc_sign_cont(uint8_t idx, uint8_t *aes_res)
{
    // CSRK Source
    uint8_t csrk_src;

    // Retrieve the command request
    struct gapc_sign_cmd *sign_cmd   = gapc_get_operation_ptr(idx, GAPC_OP_SMP);

    // Get the source of the CSRK
    if (sign_cmd->operation == GAPC_SIGN_PACKET)
    {
        csrk_src = SMPC_INFO_LOCAL;
    }
    else        // SMPC_SIGN_VERIF
    {
        csrk_src = SMPC_INFO_PEER;
    }

    // L has been generated => Mn => Cn
    if (gapc_env[idx]->smpc.state == SMPC_SIGN_L_GEN)
    {
        // Subkey (K1 or K2)
        uint8_t subkey[GAP_KEY_LEN];
        // Mn value
        uint8_t mn[GAP_KEY_LEN];
        ASSERT_ERR(gapc_env[idx]->smpc.info.sign->msg_offset <= GAP_KEY_LEN);

        /*
         ******************************************************************
         * COMPUTE Mn
         * Computing method is different following remaining size.
         ******************************************************************
         */

        if (gapc_env[idx]->smpc.info.sign->msg_offset == GAP_KEY_LEN)     // Remaining block is complete
        {
            // Generate K1
            smpc_calc_subkeys(false, aes_res, &subkey[0]);

            // Compute Mn = K1 XOR Mn'
            smpc_xor(&mn[0], &subkey[0], &sign_cmd->msg[0]);
        }
        else if (gapc_env[idx]->smpc.info.sign->msg_offset < GAP_KEY_LEN)     // Remaining block is not complete
        {
            // Remaining PDU is shorter than 128 - needs padding
            memset(&mn[0], 0x00, GAP_KEY_LEN);
            memcpy(&mn[GAP_KEY_LEN - gapc_env[idx]->smpc.info.sign->msg_offset],
                   &sign_cmd->msg[0], gapc_env[idx]->smpc.info.sign->msg_offset);
            mn[GAP_KEY_LEN - gapc_env[idx]->smpc.info.sign->msg_offset - 1] = 0x80;

            // Generate K2
            smpc_calc_subkeys(true, aes_res, &subkey[0]);

            // Compute Mn = K2 XOR (Mn' || padding)
            smpc_xor(&mn[0], &subkey[0], &mn[0]);
        }

        /*
         ******************************************************************
         * COMPUTE Cn
         ******************************************************************
         */

        smpc_generate_ci(idx, csrk_src, &gapc_env[idx]->smpc.info.sign->cn1[0], &mn[0]);
    }

    else if (gapc_env[idx]->smpc.state == SMPC_SIGN_Ci_GEN)
    {
        uint16_t length = 0;
        switch (gapc_env[idx]->smpc.info.sign->block_nb)
        {
        case (0):   // Last Ci has been computed => MAC
        {
            uint8_t status = GAP_ERR_NO_ERROR;

            if (sign_cmd->operation == GAPC_SIGN_PACKET)
            {
                // Signed Message length
                length = sign_cmd->byte_len + SMPC_SIGN_MAC_LEN;

                // Set the sign counter values
                co_write32(&(gapc_env[idx]->smpc.sign_counter[SMPC_INFO_LOCAL]),
                           (uint32_t)sign_cmd->msg[sign_cmd->byte_len - SMPC_SIGN_COUNTER_LEN]);

                gapc_env[idx]->smpc.sign_counter[SMPC_INFO_LOCAL]++;
            }
            else // GAPC_SIGN_CHECK
            {
                // Compare generated MAC and received MAC (MAC = MSB64(Cn))
                if (!memcmp(&aes_res[GAP_KEY_LEN - SMPC_SIGN_MAC_LEN],
                            &sign_cmd->msg[sign_cmd->byte_len - SMPC_SIGN_MAC_LEN], SMPC_SIGN_MAC_LEN))
                {
                    length = sign_cmd->byte_len;

                    // Set the sign counter values
                    co_write32(&(gapc_env[idx]->smpc.sign_counter[SMPC_INFO_PEER]),
                               (uint32_t)sign_cmd->msg[sign_cmd->byte_len - SMPC_SIGN_LEN]);

                    gapc_env[idx]->smpc.sign_counter[SMPC_INFO_PEER]++;
                }
                else
                {
                    // The signature is not approved.
                    status = SMP_ERROR_SIGN_VERIF_FAIL;
                }
            }

            if (status == GAP_ERR_NO_ERROR)
            {
                // Inform the HL that one of the sign counter has been updated
                struct gapc_sign_counter_ind *sign_counter_ind = KE_MSG_ALLOC(GAPC_SIGN_COUNTER_IND,
                        gapc_get_dest_task(idx),
                        KE_BUILD_ID(TASK_GAPC, idx),
                        gapc_sign_counter_ind);

                // Send the signed data to the requester
                struct gapc_sign_ind *ind = KE_MSG_ALLOC_DYN(GAPC_SIGN_IND,
                                            gapc_get_requester(idx, GAPC_OP_SMP),
                                            KE_BUILD_ID(TASK_GAPC, idx),
                                            gapc_sign_ind,
                                            length);

                ind->operation = sign_cmd->operation;
                // Data PDU length (Bytes)
                ind->byte_len  = length;

                // Original Message
                memcpy(&ind->signed_msg[0], &sign_cmd->msg[0], sign_cmd->byte_len);

                if (sign_cmd->operation == GAPC_SIGN_PACKET)
                {
                    // Append MAC - MAC = MSB64(result)
                    memcpy(&(ind->signed_msg[sign_cmd->byte_len]), &(aes_res[GAP_KEY_LEN - SMPC_SIGN_MAC_LEN]), SMPC_SIGN_MAC_LEN);
                }

                ke_msg_send(ind);

                // Update the signCounter value in the GAP
                sign_counter_ind->local_sign_counter = gapc_get_sign_counter(idx, SMPC_INFO_LOCAL);
                sign_counter_ind->peer_sign_counter = gapc_get_sign_counter(idx, SMPC_INFO_PEER);

                ke_msg_send(sign_counter_ind);
            }

            // Send CMP_EVT to the requester
            gapc_send_complete_evt(idx, GAPC_OP_SMP, status);
        }
        break;

        case (1):   // The last Ci will be computed but before that we need subkeys
        {
            // Keep the last computed Ci value
            memcpy(&gapc_env[idx]->smpc.info.sign->cn1[0], aes_res, GAP_KEY_LEN);

            /*
             * ---------------------------------------------------
             * GENERATE L ----------------------------------------
             *----------------------------------------------------
             */

            smpc_generate_l(idx, csrk_src);
        }
        break;

        default:    // Next Ci to compute
        {
            ASSERT_ERR(gapc_env[idx]->smpc.info.sign->msg_offset > GAP_KEY_LEN);

            /*
             * ---------------------------------------------------
             * GENERATE Ci ---------------------------------------
             *----------------------------------------------------
             */
            smpc_generate_ci(idx, csrk_src, aes_res,
                             &sign_cmd->msg[gapc_env[idx]->smpc.info.sign->msg_offset - GAP_KEY_LEN]);
        }
        break;
        }
    }
}
#endif // !SMP_SLIM

void smpc_calc_confirm_cont(uint8_t idx, uint8_t *aes_res)
{
    // Role
    uint8_t role    = gapc_get_role(idx);

    switch (gapc_env[idx]->smpc.state)
    {

    // E1 value used in the Confirm Value generation
    case (SMPC_PAIRING_CFM_P1):
    {
        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);

        gapc_env[idx]->smpc.state = SMPC_PAIRING_CFM_P2;

        smpc_generate_cfm(idx, role, aes_res);
    }
    break;

    case (SMPC_PAIRING_REM_CFM_P1):
    {
        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);

        gapc_env[idx]->smpc.state = SMPC_PAIRING_REM_CFM_P2;

        smpc_generate_cfm(idx, role, aes_res);
    }
    break;

    case (SMPC_PAIRING_CFM_P2):
    {
        /*
         * Confirm value has been generated, next step is to send it to the peer device
         */

        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);

        // In any role, the confirm value is sent right after calculation
        smpc_pdu_send(idx, L2C_CODE_PAIRING_CFM, (void *)aes_res);

        /*
         * The slave device shall send its own confirm value upon reception of the master
         * confirm value.
         */
        if (role == ROLE_MASTER)
        {
#if (BLE_CENTRAL)
            gapc_env[idx]->smpc.state = SMPC_PAIRING_WAIT_CONFIRM;
#endif //(BLE_CENTRAL)
        }
        else
        {
#if (BLE_PERIPHERAL)
            gapc_env[idx]->smpc.state = SMPC_PAIRING_WAIT_RAND;
#endif //(BLE_PERIPHERAL)
        }
    }
    break;

    case (SMPC_PAIRING_REM_CFM_P2):
    {
        // Status
        uint8_t status = GAP_ERR_NO_ERROR;

        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);

        /*
         * ********************************************
         * COMPARE CONFIRM VALUES
         * ********************************************
         */

        if (!memcmp(&(gapc_env[idx]->smpc.info.pair->conf_value[0]), &(aes_res[0]), GAP_KEY_LEN))
        {
            if (role == ROLE_SLAVE)
            {
#if (BLE_PERIPHERAL)
                // If slave, this calculation is made prior to sending the Rand value
                smpc_pdu_send(idx, L2C_CODE_PAIRING_RANDOM, (void *)&gapc_env[idx]->smpc.info.pair->rand[0]);

                // Set the state
                gapc_env[idx]->smpc.state = SMPC_START_ENC_STK;
#endif //(BLE_PERIPHERAL)
            }
            else    // role = ROLE_MASTER
            {
#if (BLE_CENTRAL)
                smpc_generate_stk(idx, ROLE_MASTER);
#endif //(BLE_CENTRAL)
            }
        }
        else
        {
            status = SMP_ERROR_CONF_VAL_FAILED;

            smpc_pdu_send(idx, L2C_CODE_PAIRING_FAILED, &status);

            // Start the Repeated Attempts timer
            smpc_pairing_end(idx, role,
                             SMP_GEN_PAIR_FAIL_REASON(SMP_PAIR_FAIL_REASON_MASK, status), true);
        }
    }
    break;

    case (SMPC_PAIRING_GEN_STK):
    {
        // Counter
        uint8_t counter;

        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);

        // Mask the right bytes to respect key size
        for (counter = gapc_enc_keysize_get(idx); counter < GAP_KEY_LEN; counter++)
        {
            // The result is the LTK
            aes_res[counter] = 0x00;
        }

        gapc_env[idx]->smpc.state = SMPC_START_ENC_STK;

        // If Master, start encryption using STK
        if (role == ROLE_MASTER)
        {
#if (BLE_CENTRAL)
            // Send a Start Encryption Request using the STK
            smpc_send_start_enc_cmd(idx, SMPC_USE_STK, aes_res, NULL, 0);
#endif //(BLE_CENTRAL)
        }
        // If slave, STK was calculated for LTK_REQ_REPLY
        else
        {
#if (BLE_PERIPHERAL)
            smpc_send_ltk_req_rsp(idx, true, aes_res);
#endif //(BLE_PERIPHERAL)
        }
    }
    break;
#if (SECURE_CONNECTIONS)
    case SMPC_PAIRING_SC_W4_F4_COMMITMENT_DETERMINATION :
        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);
        if ((gapc_env[idx]->smpc.info.pair->pair_method == SMPC_METH_JW) ||
                (gapc_env[idx]->smpc.info.pair->pair_method == SMPC_METH_NC))
        {
            bool aes_cmac_complete = false;
            if (role == ROLE_SLAVE)
            {
                ASSERT_ERR(gapc_env[idx]->smpc.info.pair->aes_cmac != NULL);
                if (gapc_env[idx]->smpc.info.pair->aes_cmac)
                {
                    aes_cmac_complete = smpc_process_aes_cmac(idx, aes_res);

                    // In Secure Connections - Just Works or Numeric Comparison
                    // the Commitment is only sent by the slave.
                    // Also the commitment does not need to be stored as it is transmitted to the
                    // master and not used again in the slave.

                    if (aes_cmac_complete == true)
                    {
                        smpc_f4_complete(idx);

                        smpc_pdu_send(idx, L2C_CODE_PAIRING_CFM, (void *)aes_res);

                        // Now Wait for the Random Na from the master
                        gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_W4_PEER_RAND;
                    }
                }
            }

        }
        break;

    case SMPC_PAIRING_SC_PASSKEY_W4_F4_COMMITMENT_DETERMINATION :
#ifndef SMP_SLIM
        if (gapc_env[idx]->smpc.info.pair->pair_method == SMPC_METH_PK)
        {
            bool aes_cmac_complete = false;

            ASSERT_ERR(gapc_env[idx]->smpc.info.pair->aes_cmac != NULL);
            if (gapc_env[idx]->smpc.info.pair->aes_cmac)
            {
                aes_cmac_complete = smpc_process_aes_cmac(idx, aes_res);

                if (aes_cmac_complete == true)
                {
                    smpc_f4_complete(idx);
                    smpc_pdu_send(idx, L2C_CODE_PAIRING_CFM, (void *)aes_res);

                    if (role == ROLE_MASTER)
                    {
                        // Now Wait for the commitment from the slave
                        gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_PASSKEY_W4_PEER_COMMITMENT;
                    }
                    else
                    {
                        // Now Wait for the Random Na from the master
                        gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_PASSKEY_W4_PEER_RAND;
                    }
                }
            }
        }
#else
        ASSERT_ERR(0);
#endif // !SMP_SLIM
        break;

    case SMPC_PAIRING_SC_OOB_W4_F4_COMMITMENT_DETERMINATION :
#ifndef SMP_SLIM
        if (gapc_env[idx]->smpc.info.pair->pair_method == SMPC_METH_OOB)
        {
            bool aes_cmac_complete = false;

            ASSERT_ERR(gapc_env[idx]->smpc.info.pair->aes_cmac != NULL);
            if (gapc_env[idx]->smpc.info.pair->aes_cmac)
            {
                aes_cmac_complete = smpc_process_aes_cmac(idx, aes_res);

                // In Secure Connections - Passkey
                // the Commitment is only sent by the slave.
                // Also the commitment does not need to be stored as it is transmitted to the
                // master and not used again in the slave.

                if (aes_cmac_complete == true)
                {
                    smpc_f4_complete(idx);
                    memcpy(&gapc_env[idx]->smpc.info.pair->conf_value[0], aes_res, 16);
                    gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_OOB_W4_OOB_DATA;
                    smpc_send_pairing_req_ind(idx, GAPC_OOB_EXCH);
                }
            }
        }
#else // !SMP_SLIM
        ASSERT_ERR(0);
#endif
        break;
    case SMPC_PAIRING_SC_PASSKEY_W4_F4_COMMITMENT_CHECK :
#ifndef SMP_SLIM
        if (gapc_env[idx]->smpc.info.pair->pair_method == SMPC_METH_PK)
        {
            bool aes_cmac_complete = false;

            ASSERT_ERR(gapc_env[idx]->smpc.info.pair->aes_cmac != NULL);
            if (gapc_env[idx]->smpc.info.pair->aes_cmac)
            {
                aes_cmac_complete = smpc_process_aes_cmac(idx, aes_res);

                if (aes_cmac_complete == true)
                {
                    smpc_f4_complete(idx);
                    // Check the Peer commitment against the locally determined one
                    if (!memcmp(&(gapc_env[idx]->smpc.info.pair->conf_value[0]), &(aes_res[0]), GAP_KEY_LEN))
                    {
                        if (role == ROLE_SLAVE)
                        {
                            // Send the locally generated rand to the peer
                            smpc_pdu_send(idx, L2C_CODE_PAIRING_RANDOM, (void *)&gapc_env[idx]->smpc.info.pair->rand[0]);

                        }

                        // Check the PassKey bit counter to see if this is the last round
                        if (gapc_env[idx]->smpc.info.pair->passkey_bit_count != 19)
                        {
                            gapc_env[idx]->smpc.info.pair->passkey_bit_count++;
                            if (role == ROLE_MASTER)
                            {
                                gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_PASSKEY_W4_LOCAL_RAND_N_P1;
                                smpc_generate_rand(idx, SMPC_PAIRING_SC_PASSKEY_W4_LOCAL_RAND_N_P1);
                            }
                            else
                            {
                                gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_PASSKEY_W4_PEER_COMMITMENT;
                            }
                        }
                        else
                        {
                            // Advance to the DHkey Check phase which is common to all Auth-Methods
                            smpc_init_mac_key_calculation(idx);
                        }

                    }
                    else
                    {
                        // Pairing Failed -- Send Error to Peer
                        uint8_t status = SMP_ERROR_CONF_VAL_FAILED;

                        smpc_pdu_send(idx, L2C_CODE_PAIRING_FAILED, &status);

                        // Start the Repeated Attempts timer
                        smpc_pairing_end(idx, role,
                                         SMP_GEN_PAIR_FAIL_REASON(SMP_PAIR_FAIL_REASON_MASK, status), true);

                    }
                }
            }
        }
#else // SMP_SLIM
        ASSERT_ERR(0);
#endif
        break;

    case SMPC_PAIRING_SC_W4_F4_COMMITMENT_CHECK :
        if ((gapc_env[idx]->smpc.info.pair->pair_method == SMPC_METH_JW) ||
                (gapc_env[idx]->smpc.info.pair->pair_method == SMPC_METH_NC))
        {

            bool aes_cmac_complete = false;

            if (role == ROLE_MASTER)
            {

                ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);
                ASSERT_ERR(gapc_env[idx]->smpc.info.pair->aes_cmac != NULL);
                if (gapc_env[idx]->smpc.info.pair->aes_cmac)
                {
                    aes_cmac_complete = smpc_process_aes_cmac(idx, aes_res);

                    /*
                    *********************************************
                    * COMPARE CONFIRM VALUES
                    *********************************************
                    */

                    if (aes_cmac_complete == true)
                    {
                        smpc_f4_complete(idx);

                        if (!memcmp(&(gapc_env[idx]->smpc.info.pair->conf_value[0]), &(aes_res[0]), GAP_KEY_LEN))
                        {
                            if (gapc_env[idx]->smpc.info.pair->pair_method == SMPC_METH_NC)
                            {
                                // Determine Va = g2(Pka,Pkb,Na,Nb)

                                // Im a Master so !
                                // Pka is my device Public Key
                                // Pkb is peers Public Key
                                // Na is local Random Number
                                // Nb is peers Random Number
                                public_key_t *Pka;
                                public_key_t *Pkb = &gapc_env[idx]->smpc.info.pair->peer_public_key;
                                uint8_t *Na = &gapc_env[idx]->smpc.info.pair->rand[0];
                                uint8_t *Nb = &gapc_env[idx]->smpc.info.pair->rem_rand[0];
                                Pka = gapm_get_local_public_key();

                                gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_W4_G2_AES_CMAC;
                                smpc_g2_init(idx, Pka->x, Pkb->x, Na, Nb);
                            }
                            else if (gapc_env[idx]->smpc.info.pair->pair_method == SMPC_METH_JW)
                            {
                                smpc_init_mac_key_calculation(idx);
                            }

                        }
                        else
                        {
                            uint8_t status = SMP_ERROR_CONF_VAL_FAILED;

                            smpc_pdu_send(idx, L2C_CODE_PAIRING_FAILED, (void *)&status);

                            // Start the Repeated Attempts timer
                            smpc_pairing_end(idx, role,
                                             SMP_GEN_PAIR_FAIL_REASON(SMP_PAIR_FAIL_REASON_MASK, status), true);
                        }
                    }
                }
            }
        }
        break;

    case SMPC_PAIRING_SC_OOB_W4_F4_COMMITMENT_CHECK :
#ifndef SMP_SLIM
        if (gapc_env[idx]->smpc.info.pair->pair_method == SMPC_METH_OOB)
        {
            bool aes_cmac_complete = false;

            ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);
            ASSERT_ERR(gapc_env[idx]->smpc.info.pair->aes_cmac != NULL);

            if (gapc_env[idx]->smpc.info.pair->aes_cmac)
            {
                aes_cmac_complete = smpc_process_aes_cmac(idx, aes_res);
                /*
                 *********************************************
                 * COMPARE CONFIRM VALUES
                 *********************************************
                 */

                if (aes_cmac_complete == true)
                {
                    smpc_f4_complete(idx);
                    if (!memcmp(&(gapc_env[idx]->smpc.info.pair->conf_value[0]), &(aes_res[0]), GAP_KEY_LEN))
                    {
                        // Get a local Random number
                        if ((role == ROLE_MASTER) ||
                                ((role == ROLE_SLAVE) && (gapc_env[idx]->smpc.info.pair->peer_rand_received == true)))
                        {
                            gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_OOB_W4_LOCAL_RAND_N_P1;
                            smpc_generate_rand(idx, SMPC_PAIRING_SC_OOB_W4_LOCAL_RAND_N_P1);
                        }
                        else
                        {
                            // A slave - waits for the peers rand if it has not already been recieved.
                            gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_OOB_W4_PEER_RAND;
                        }
                    }
                    else
                    {
                        uint8_t status = SMP_ERROR_CONF_VAL_FAILED;

                        smpc_pdu_send(idx, L2C_CODE_PAIRING_FAILED, (void *)&status);

                        // Start the Repeated Attempts timer
                        smpc_pairing_end(idx, role,
                                         SMP_GEN_PAIR_FAIL_REASON(SMP_PAIR_FAIL_REASON_MASK, status), true);
                    }
                }
            }
        }
#else // !SMP_SLIM
        ASSERT_ERR(0);
#endif
        break;
    case SMPC_PAIRING_SC_W4_G2_AES_CMAC :
        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);
        if (gapc_env[idx]->smpc.info.pair->pair_method == SMPC_METH_NC)
        {
            bool aes_cmac_complete = false;
            ASSERT_ERR(gapc_env[idx]->smpc.info.pair->aes_cmac != NULL);
            if (gapc_env[idx]->smpc.info.pair->aes_cmac)
            {
                aes_cmac_complete = smpc_process_aes_cmac(idx, aes_res);
            }

            if (aes_cmac_complete)
            {
                smpc_g2_complete(idx);
                // If AES_CMAC complete then G2 is complete
                // we need to determine the number for comparison - Va.

                // Write the First 4 bytes of AES Result to TK.
                {

                    // Note ! we use the uint32_t for passkey to store the Numeric Value
                    gapc_env[idx]->smpc.info.pair->passkey = (co_read32p(&aes_res[0])) % 0xF4240;

                    // Change State to Wait for Response from Application.
                    // state changes to SMPC_PAIRING_TK_WAIT
                    gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_W4_NC_ACCEPT;
                    smpc_send_pairing_req_ind(idx, GAPC_NC_EXCH);
                }
            }
        }
        break;

    case SMPC_PAIRING_SC_W4_F5_P1 :
    case SMPC_PAIRING_SC_W4_F5_P2 :
    case SMPC_PAIRING_SC_W4_F5_P3 :
    {

        bool aes_cmac_complete = false;
        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);
        ASSERT_ERR(gapc_env[idx]->smpc.info.pair->aes_cmac != NULL);
        aes_cmac_complete = smpc_process_aes_cmac(idx, aes_res);

        if (aes_cmac_complete)
        {
            struct smp_f5 *p_f5 = gapc_env[idx]->smpc.info.pair->f5_info;

            // Move onto the next AES_CMAC in f5
            switch (gapc_env[idx]->smpc.state)
            {
            case SMPC_PAIRING_SC_W4_F5_P1 :
                memcpy(p_f5->T, aes_res, 16);
                gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_W4_F5_P2;
                // Move onto next AES-CMAC for the MACKEY - Free existing AES_CMAC Memory
                if (gapc_env[idx]->smpc.info.pair->aes_cmac != NULL)
                    ke_free(gapc_env[idx]->smpc.info.pair->aes_cmac);

                smpc_aes_cmac_init(idx, p_f5->T, p_f5->M, 53);
                break;

            case SMPC_PAIRING_SC_W4_F5_P2 :
                memcpy(gapc_env[idx]->smpc.info.pair->MacKey, aes_res, 16);
                // Move onto next AES-CMAC for the LTK
                p_f5->M[52] = 0x01; // Counter = 0x01 for LTK
                gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_W4_F5_P3;
                // Move onto next AES-CMAC for the MACKEY - Free existing AES_CMAC Memory
                if (gapc_env[idx]->smpc.info.pair->aes_cmac != NULL)
                    ke_free(gapc_env[idx]->smpc.info.pair->aes_cmac);
                smpc_aes_cmac_init(idx, p_f5->T, p_f5->M, 53);
                break;

            case SMPC_PAIRING_SC_W4_F5_P3 :
                // Mask the MSB to respect key size (Key is stored LSB->MSB)
                for (uint8_t counter = gapc_enc_keysize_get(idx); counter < GAP_KEY_LEN; counter++)
                {
                    aes_res[counter] = 0x00;
                }
                memcpy(gapc_env[idx]->smpc.info.pair->key.key, aes_res, GAP_KEY_LEN);
                smpc_f5_complete(idx);
                if (role == ROLE_MASTER)
                {
                    // Begin the DHKey Check.
                    gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_W4_F6_DHKEY_CHECK;
                    smpc_initiate_dhkey_check(idx);
                }
                else
                {
                    if (gapc_env[idx]->smpc.info.pair->dh_key_check_received_from_peer == false)
                    {
                        // Wait for DHKey_Check (Ea) from master.
                        gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_W4_PEER_DHKEY_CHECK;

                    }
                    else
                    {
                        // If we have already received the DH_Key we can peform DH_Key check immediately
                        gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_W4_F6_DHKEY_CHECK;
                        smpc_initiate_dhkey_check(idx);
                    }
                }
                break;

            default :
                break;
            }
        }
    }
    break;

    case SMPC_PAIRING_SC_W4_F6_DHKEY_CHECK :
    case SMPC_PAIRING_SC_W4_F6_DHKEY_VERIFICATION :
    {
        bool aes_cmac_complete = false;

        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);
        ASSERT_ERR(gapc_env[idx]->smpc.info.pair->aes_cmac != NULL);

        aes_cmac_complete = smpc_process_aes_cmac(idx, aes_res);

        if (aes_cmac_complete)
        {
            // If AES_CMAC complete then f6 is complete
            smpc_f6_complete(idx);
            if (gapc_env[idx]->smpc.state == SMPC_PAIRING_SC_W4_F6_DHKEY_CHECK)
            {
                // Store local DHKeyCheck - will need it later
                // Store the DHkey_Check -
                memcpy(gapc_env[idx]->smpc.info.pair->dh_key_check_local, aes_res, 16);

                if (role == MASTER_ROLE)
                {
                    // Send the DHKeyCheck to the peer device
                    smpc_pdu_send(idx, L2C_CODE_DHKEY_CHECK, (void *)aes_res);
                    // Change State to
                    gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_W4_PEER_DHKEY_CHECK;
                }
                else
                {
                    gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_W4_F6_DHKEY_VERIFICATION;
                    smpc_initiate_dhkey_verification(idx);
                }
            }
            else // SMPC_PAIRING_SC_W4_F6_DHKEY_VERIFICATION
            {
                if (role == ROLE_SLAVE)
                {
                    // Compare DHKey Check transmitted by peer against check for peer calculated locally
                    if (!memcmp(aes_res, gapc_env[idx]->smpc.info.pair->dh_key_check_peer, 16))
                    {
                        // Send the Local DHKeyCheck to the peer device
                        smpc_pdu_send(idx, L2C_CODE_DHKEY_CHECK, gapc_env[idx]->smpc.info.pair->dh_key_check_local);
                        // Wait for the Encryption to be initiated by the Master
                        gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_W4_ENCRYPTION_START;

                    }
                    else
                    {
                        uint8_t status = SMP_ERROR_DHKEY_CHECK_FAILED;

                        smpc_pdu_send(idx, L2C_CODE_PAIRING_FAILED, &status);

                        smpc_pairing_end(idx, role,
                                         SMP_GEN_PAIR_FAIL_REASON(SMP_PAIR_FAIL_REASON_MASK, status), true);
                    }
                }
                else
                {
#if (BLE_CENTRAL)
                    if (!memcmp(aes_res, gapc_env[idx]->smpc.info.pair->dh_key_check_peer, 16))
                    {
                        uint8_t random[16];

                        memset(random, 0x00, 16);
                        // Begin to encrypt the link..
                        smpc_send_start_enc_cmd(idx, SMPC_USE_LTK, &gapc_env[idx]->smpc.info.pair->key.key[0], &random[0], 0x00);

                        gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_W4_ENCRYPTION_CHANGE;

                    }
                    else
                    {
                        uint8_t status = SMP_ERROR_DHKEY_CHECK_FAILED;

                        smpc_pdu_send(idx, L2C_CODE_PAIRING_FAILED, &status);

                        smpc_pairing_end(idx, role,
                                         SMP_GEN_PAIR_FAIL_REASON(SMP_PAIR_FAIL_REASON_MASK, status), true);
                    }
#endif // BLE_CENTRAL
                }
            }
        }
    }
    break;

#endif // (SECURE_CONNECTIONS)
    default:
    {
        ASSERT_INFO(0, gapc_env[idx]->smpc.state, role);
    }
    break;
    }
}


void smpc_confirm_gen_rand(uint8_t idx, rand_nb_t *randnb)
{

#if (SECURE_CONNECTIONS)
    uint8_t role = gapc_get_role(idx);
#endif // (SECURE_CONNECTIONS)
    switch (gapc_env[idx]->smpc.state)
    {
    // Signature Procedure
    case (SMPC_PAIRING_GEN_RAND_P1):
    {
        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);

        memcpy(&gapc_env[idx]->smpc.info.pair->rand[0], randnb, GAP_RAND_NB_LEN);

        smpc_generate_rand(idx, SMPC_PAIRING_GEN_RAND_P2);
    }
    break;

    case (SMPC_PAIRING_GEN_RAND_P2):
    {
        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);

        memcpy(&gapc_env[idx]->smpc.info.pair->rand[GAP_RAND_NB_LEN], randnb, GAP_RAND_NB_LEN);

        gapc_env[idx]->smpc.state = SMPC_PAIRING_CFM_P1;

        // Random value has been generated => start confirm value generation
        smpc_generate_e1(idx, gapc_get_role(idx), true);
    }
    break;

#if (SECURE_CONNECTIONS)

    case (SMPC_PAIRING_SC_W4_LOCAL_RAND_N_P1) :

        // The first part of the local rand has been generated - next action is dependant on Role and Association Model.
        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);
        memcpy(&gapc_env[idx]->smpc.info.pair->rand[0], randnb, GAP_RAND_NB_LEN);
        smpc_generate_rand(idx, SMPC_PAIRING_SC_W4_LOCAL_RAND_N_P2);
        break;

    case (SMPC_PAIRING_SC_W4_LOCAL_RAND_N_P2) :
        // The secoond part of the local rand has been generated - next action is dependent on Role and Association Model.
        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);
        memcpy(&gapc_env[idx]->smpc.info.pair->rand[GAP_RAND_NB_LEN], randnb, GAP_RAND_NB_LEN);

        if (role == ROLE_SLAVE)
        {
            // Begin the Local Commitment determination
            public_key_t *Pka = &(gapc_env[idx]->smpc.info.pair->peer_public_key);
            public_key_t *Pkb;
            uint8_t *Nb = &gapc_env[idx]->smpc.info.pair->rand[0];

            Pkb = gapm_get_local_public_key();
            gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_W4_F4_COMMITMENT_DETERMINATION;
            // Cb = f4(Pkb,Pka,Nb,0)
            smpc_f4_Init(idx, Pkb->x, Pka->x, Nb, 0);
        }
        else
        {
            // Send local Rand to peer.
            smpc_pdu_send(idx, L2C_CODE_PAIRING_RANDOM, (void *)&gapc_env[idx]->smpc.info.pair->rand[0]);
            gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_W4_PEER_RAND;
        }

        break;

#ifndef SMP_SLIM
    case SMPC_PAIRING_SC_PASSKEY_W4_LOCAL_RAND_N_P1 :
        // The first part of the local rand has been generated - next action is dependant on Role and Association Model.
        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);
        memcpy(&gapc_env[idx]->smpc.info.pair->rand[0], randnb, GAP_RAND_NB_LEN);
        smpc_generate_rand(idx, SMPC_PAIRING_SC_PASSKEY_W4_LOCAL_RAND_N_P2);
        break;

    case SMPC_PAIRING_SC_PASSKEY_W4_LOCAL_RAND_N_P2 :
    {
        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);
        memcpy(&gapc_env[idx]->smpc.info.pair->rand[GAP_RAND_NB_LEN], randnb, GAP_RAND_NB_LEN);
        if (role == ROLE_MASTER)
        {
            // Begin the Local Commitment determination -
            public_key_t *Pka = gapm_get_local_public_key();
            public_key_t *Pkb = &(gapc_env[idx]->smpc.info.pair->peer_public_key);
            uint8_t Rai = smpc_get_next_passkey_bit(idx) | 0x80;
            uint8_t *Nai = &gapc_env[idx]->smpc.info.pair->rand[0];

            gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_PASSKEY_W4_F4_COMMITMENT_DETERMINATION;

            // Cb = f4(Pka,Pkb,Nai,rai)
            smpc_f4_Init(idx, Pka->x, Pkb->x, Nai, Rai);
        }
        else // Role == SLAVE
        {
            // Begin the Local Commitment determination - Cb = f4(Pka,Pkb,Nbi,Rbi)

            public_key_t *Pka = &(gapc_env[idx]->smpc.info.pair->peer_public_key);
            public_key_t *Pkb = gapm_get_local_public_key();
            uint8_t Rbi = smpc_get_next_passkey_bit(idx) | 0x80;
            uint8_t *Nbi = &gapc_env[idx]->smpc.info.pair->rand[0];

            gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_PASSKEY_W4_F4_COMMITMENT_DETERMINATION;
            // Cb = f4(Pkb,Pka,Nbi,rbi)
            smpc_f4_Init(idx, Pkb->x, Pka->x, Nbi, Rbi);
        }
    }
    break;

    case SMPC_PAIRING_SC_OOB_W4_LOCAL_RAND_R_P1 :
        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);
        memcpy(&gapc_env[idx]->smpc.info.pair->local_r[0], randnb, GAP_RAND_NB_LEN);
        smpc_generate_rand(idx, SMPC_PAIRING_SC_OOB_W4_LOCAL_RAND_R_P2);
        break;

    case SMPC_PAIRING_SC_OOB_W4_LOCAL_RAND_R_P2 :
    {
        bool oob_data_present = false;

        if (((role == ROLE_MASTER) &&
                (gapc_env[idx]->smpc.info.pair->pair_req_feat.oob == GAP_OOB_AUTH_DATA_PRESENT)) ||
                ((role == ROLE_SLAVE) &&
                 (gapc_env[idx]->smpc.info.pair->pair_rsp_feat.oob == GAP_OOB_AUTH_DATA_PRESENT)))
        {
            oob_data_present = true;
        }
        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);
        memcpy(&gapc_env[idx]->smpc.info.pair->local_r[GAP_RAND_NB_LEN], randnb, GAP_RAND_NB_LEN);

        if (oob_data_present)
        {
            // Begin the Local Commitment determination - Ca = f4(Pka,Pka,Na,0)
            public_key_t *Pk = gapm_get_local_public_key();
            uint8_t *r = &gapc_env[idx]->smpc.info.pair->local_r[0];

            gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_OOB_W4_F4_COMMITMENT_DETERMINATION;

            // Ca = (PKa, PKa, ra, 0) if slave
            // Cb = (PKb, PKb, rb, 0) if master
            smpc_f4_Init(idx, Pk->x, Pk->x, r, 0);
        }
        else
        {
            // Set the local commitment = 0
            memset(&gapc_env[idx]->smpc.info.pair->local_r[0], 0, 16);
            memset(&gapc_env[idx]->smpc.info.pair->conf_value, 0, 16);

            gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_OOB_W4_OOB_DATA;
            //smpc_send_oob_cfm_to_api(idx,aes_res,&gapc_env[idx]->smpc.info.pair->local_r[0]);
            smpc_send_pairing_req_ind(idx, GAPC_OOB_EXCH);
        }
    }
    break;

    case SMPC_PAIRING_SC_OOB_W4_LOCAL_RAND_N_P1 :
        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);
        memcpy(&gapc_env[idx]->smpc.info.pair->rand[0], randnb, GAP_RAND_NB_LEN);
        smpc_generate_rand(idx, SMPC_PAIRING_SC_OOB_W4_LOCAL_RAND_N_P2);
        break;

    case SMPC_PAIRING_SC_OOB_W4_LOCAL_RAND_N_P2 :
    {
        ASSERT_ERR(gapc_env[idx]->smpc.info.pair != NULL);
        memcpy(&gapc_env[idx]->smpc.info.pair->rand[GAP_RAND_NB_LEN], randnb, GAP_RAND_NB_LEN);

        if (role == ROLE_MASTER)
        {
            // Send Rand(Na) to Peer
            // wait for Rand from peer
            smpc_pdu_send(idx, L2C_CODE_PAIRING_RANDOM, (void *)&gapc_env[idx]->smpc.info.pair->rand[0]);
            gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_OOB_W4_PEER_RAND;
        }
        else
        {
            smpc_pdu_send(idx, L2C_CODE_PAIRING_RANDOM, (void *)&gapc_env[idx]->smpc.info.pair->rand[0]);
            smpc_init_mac_key_calculation(idx);
        }

    }
    break;
#endif //SMP_SLIM
#endif // (SECURE_CONNECTIONS)
    default:
    {
        ASSERT_INFO(0, gapc_env[idx]->smpc.state, gapc_get_role(idx));
    }
    break;
    }
}

void smpc_key_press_notification_ind(uint8_t idx, uint8_t notification_type)
{
    // Indicate the host that the peer has Press a Key
    struct gapc_key_press_notif_ind  *ind = KE_MSG_ALLOC(GAPC_KEY_PRESS_NOTIFICATION_IND,
                                            gapc_get_dest_task(idx), KE_BUILD_ID(TASK_GAPC, idx),
                                            gapc_key_press_notif_ind);

    // Set keyPress
    ind->notification_type = notification_type;
    ke_msg_send(ind);
}

#endif //(BLE_SMPC)

/// @} SMPC_API
