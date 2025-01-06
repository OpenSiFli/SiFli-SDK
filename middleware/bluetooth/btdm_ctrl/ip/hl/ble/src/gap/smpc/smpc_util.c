/**
 ****************************************************************************************
 *
 * @file smpc_util.c
 *
 * @brief SMPC Utility implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @addtogroup SMPC_UTIL
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

#include "smpc_util.h"
#include "co_utils.h"
#include "co_endian.h"

#include "smpc_api.h"
#include "smpc_crypto.h"

#include "gapm_task.h"
#include "gapm.h"
#include "gapc.h"

#include "ke_mem.h"
#include "ke_timer.h"

#include "../gapc/gapc_int.h" // Internal API required

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

uint8_t smpc_check_param(struct l2cc_pdu *pdu)
{
    // Status
    uint8_t status = GAP_ERR_NO_ERROR;

    switch (pdu->data.code)
    {
    case (L2C_CODE_PAIRING_REQ):
    case (L2C_CODE_PAIRING_RESP):
    {
        // Check IO Capabilities value
        if ((pdu->data.pairing_req.iocap) > GAP_IO_CAP_KB_DISPLAY)
        {
            status = SMP_ERROR_INVALID_PARAM;
        }
        // Check Authentication Requirements
        else if (((SMPC_MASK_AUTH_REQ(pdu->data.pairing_req.auth)) != GAP_AUTH_REQ_NO_MITM_NO_BOND) &&
                 ((SMPC_MASK_AUTH_REQ(pdu->data.pairing_req.auth)) != GAP_AUTH_REQ_NO_MITM_BOND)    &&
                 ((SMPC_MASK_AUTH_REQ(pdu->data.pairing_req.auth)) != GAP_AUTH_REQ_MITM_NO_BOND)    &&
                 ((SMPC_MASK_AUTH_REQ(pdu->data.pairing_req.auth)) != GAP_AUTH_REQ_MITM_BOND))
        {
            status = SMP_ERROR_INVALID_PARAM;
        }
        // Check Out Of Band status
        else if (((pdu->data.pairing_req.oob) != GAP_OOB_AUTH_DATA_NOT_PRESENT) &&
                 ((pdu->data.pairing_req.oob) != GAP_OOB_AUTH_DATA_PRESENT))
        {
            status = SMP_ERROR_INVALID_PARAM;
        }
        // Check Encryption Key Size
        else if ((pdu->data.pairing_req.key_size < SMPC_MIN_ENC_SIZE_LEN) ||
                 (pdu->data.pairing_req.key_size > SMPC_MAX_ENC_SIZE_LEN))
        {
            status = SMP_ERROR_ENC_KEY_SIZE;
        }
    }
    break;

    case (L2C_CODE_PAIRING_FAILED):
    {
        if ((pdu->data.pairing_failed.reason < SMP_ERROR_PASSKEY_ENTRY_FAILED) ||
                (pdu->data.pairing_failed.reason > SMP_ERROR_CROSS_TRANSPORT_KEY_GENERATION_NOT_ALLOWED))
        {
            // change the reason code because outside of known error codes.
            pdu->data.pairing_failed.reason = SMP_ERROR_UNSPECIFIED_REASON;
        }
    }
    break;

    case (L2C_CODE_SECURITY_REQ):
    {
        if (((SMPC_MASK_AUTH_REQ(pdu->data.security_req.auth)) != GAP_AUTH_REQ_NO_MITM_NO_BOND) &&
                ((SMPC_MASK_AUTH_REQ(pdu->data.security_req.auth)) != GAP_AUTH_REQ_NO_MITM_BOND)    &&
                ((SMPC_MASK_AUTH_REQ(pdu->data.security_req.auth)) != GAP_AUTH_REQ_MITM_NO_BOND)    &&
                ((SMPC_MASK_AUTH_REQ(pdu->data.security_req.auth)) != GAP_AUTH_REQ_MITM_BOND))
        {
            status = SMP_ERROR_INVALID_PARAM;
        }
    }
    break;

#if (SECURE_CONNECTIONS)
    case (L2C_CODE_KEYPRESS_NOTIFICATION):
        if (pdu->data.keypress_noticication.notification_type > SMP_PASSKEY_ENTRY_COMPLETED)
        {
            status = SMP_ERROR_INVALID_PARAM;
        }
        break;

    case (L2C_CODE_PUBLIC_KEY):
    case (L2C_CODE_DHKEY_CHECK):
#endif // (SECURE_CONNECTIONS)
    case (L2C_CODE_PAIRING_RANDOM):
    case (L2C_CODE_PAIRING_CFM):
    case (L2C_CODE_ENCRYPTION_INF):
    case (L2C_CODE_MASTER_ID):
    case (L2C_CODE_IDENTITY_INF):
    case (L2C_CODE_ID_ADDR_INF):
    case (L2C_CODE_SIGNING_INF):
    {
        // Not checkable
    } break;

    default:
    {
        // Code is not recognized, return an error
        status = SMP_ERROR_CMD_NOT_SUPPORTED;
    }
    break;
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Construct pairing request PDU.
 * @param buf  Pointer to buffer where the PDU is packed
 * @param idx  Index of task for which the PDU is being built
 * @return PDU length.
 *****************************************************************************************
 */
static void smpc_construct_pair_req_pdu(struct l2cc_pdu *pdu, void *pair_req_feat)
{
    // Pack the structure as it is - No pads
    memcpy(&(pdu->data.pairing_req.iocap), pair_req_feat, SMPC_CODE_PAIRING_REQ_RESP_LEN - 1);
}

/**
 ****************************************************************************************
 * @brief Construct pairing response PDU.
 * @param buf   Pointer to buffer where the PDU is packed
 * @param idx  Index of task for which the PDU is being built
 * @return PDU length
 *****************************************************************************************
 */
static void smpc_construct_pair_rsp_pdu(struct l2cc_pdu *pdu, void *pair_rsp_feat)
{
    // Pack the structure as it is - No pads
    memcpy(&(pdu->data.pairing_rsp.iocap), pair_rsp_feat, SMPC_CODE_PAIRING_REQ_RESP_LEN - 1);
}

/**
 ****************************************************************************************
 * @brief Construct pairing confirm PDU.
 * @param buf  Pointer to buffer where the PDU is packed
 * @param idx  Index of task for which the PDU is being built
 * @return PDU length
 *****************************************************************************************
 */
static void smpc_construct_pair_cfm_pdu(struct l2cc_pdu *pdu, void *conf_val)
{
    // Pack the confirm value - 16 bytes (Shall be given LSB->MSB)
    memcpy(&(pdu->data.pairing_cfm.cfm_val[0]), conf_val, CFM_LEN);
}

/**
 ****************************************************************************************
 * @brief Construct pairing random PDU.
 * @param buf  Pointer to buffer where the PDU is packed
 * @param idx  Index of task for which the PDU is being built
 * @return PDU length
 *****************************************************************************************
 */
static void smpc_construct_pair_rand_pdu(struct l2cc_pdu *pdu, void *rand_nb)
{
    // Pack the random number value - 16 bytes (Shall be given LSB->MSB)
    memcpy(&(pdu->data.pairing_random.random[0]), rand_nb, RAND_VAL_LEN);
}

/**
 ****************************************************************************************
 * @brief Construct pairing failed PDU.
 * @param buf  Pointer to buffer where the PDU is packed
 * @param idx  Index of task for which the PDU is being built
 * @return PDU length
 *****************************************************************************************
 */
static void smpc_construct_pair_fail_pdu(struct l2cc_pdu *pdu, void *status)
{
    // Pack failure reason - 1 byte
    pdu->data.pairing_failed.reason = *(uint8_t *)status;
}

/**
 ****************************************************************************************
 * @brief Construct encryption information PDU.
 * @param buf  Pointer to buffer where the PDU is packed
 * @param idx  Index of task for which the PDU is being built
 * @return PDU length
 *****************************************************************************************
 */
static void smpc_construct_enc_info_pdu(struct l2cc_pdu *pdu, void *ltk)
{
    // Pack LTK - 16 bytes (Shall be given LSB->MSB)
    memcpy(&(pdu->data.encryption_inf.ltk[0]), ltk, GAP_KEY_LEN);
}

/**
 ****************************************************************************************
 * @brief Construct master identification information PDU.
 * @param buf  Pointer to buffer where the PDU is packed
 * @param idx  Index of task for which the PDU is being built
 * @return PDU length
 *****************************************************************************************
 */
static void smpc_construct_mst_id_pdu(struct l2cc_pdu *pdu, void *mst_id_info)
{
    // Pack encryption diversifier
    pdu->data.master_id.ediv = co_htobs(((struct smpc_mst_id_info *)mst_id_info)->ediv);

    // Pack random number value - 8 bytes (Shall be given LSB->MSB)
    memcpy(&(pdu->data.master_id.nb), ((struct smpc_mst_id_info *)mst_id_info)->randnb, GAP_RAND_NB_LEN);
}

/**
 ****************************************************************************************
 * @brief Construct identification information PDU.
 * @param buf  Pointer to buffer where the PDU is packed
 * @param idx  Index of task for which the PDU is being built
 * @return PDU length
 *****************************************************************************************
 */
static void smpc_construct_id_info_pdu(struct l2cc_pdu *pdu, void *irk)
{
    // Pack IRK - 16 bytes (Shall be given LSB->MSB)
    memcpy(&(pdu->data.identity_inf.irk[0]), irk, GAP_KEY_LEN);
}

/**
 ****************************************************************************************
 * @brief Construct identification address information PDU.
 * @param buf  Pointer to buffer where the PDU is packed
 * @param idx  Index of task for which the PDU is being built
 * @return PDU length
 *****************************************************************************************
 */
static void smpc_construct_id_addr_info_pdu(struct l2cc_pdu *pdu, void *addr)
{
    // Pack address type - 1 byte
    pdu->data.id_addr_inf.addr_type
        = (gapm_get_address_type() & GAPM_PRIV_CFG_PRIV_ADDR_BIT) ? ADDR_RAND : ADDR_PUBLIC;

    // Pack BD address - 6 bytes (Shall be given LSB->MSB)
    memcpy(&pdu->data.id_addr_inf.addr.addr[0], &gapm_get_bdaddr()->addr[0], BD_ADDR_LEN);
}

/**
 ****************************************************************************************
 * @brief Construct signing information PDU.
 * @param buf  Pointer to buffer where the PDU is packed
 * @param idx  Index of task for which the PDU is being built
 * @return PDU length
 *****************************************************************************************
 */
static void smpc_construct_sign_info_pdu(struct l2cc_pdu *pdu, void *csrk)
{
    // Pack CSRK - 16 bytes (Shall be given LSB->MSB)
    memcpy(&(pdu->data.signing_inf.csrk[0]), csrk, GAP_KEY_LEN);
}

/**
 ****************************************************************************************
 * @brief Construct security request PDU.
 * @param buf  Pointer to buffer where the PDU is packed
 * @param idx  Index of task for which the PDU is being built
 * @return PDU length
 *****************************************************************************************
 */
static void smpc_construct_sec_req_pdu(struct l2cc_pdu *pdu, void *auth_req)
{
    // Pack authentication requirements from slave
    pdu->data.security_req.auth = *(uint8_t *)auth_req;
}

#if (SECURE_CONNECTIONS)
/**
 ****************************************************************************************
 * @brief Construct Pairing Public Key PDU.
 * @param buf  Pointer to buffer where the PDU is packed
 * @param idx  Index of task for which the PDU is being built
 * @return PDU length
 *****************************************************************************************
 */

static void smpc_construcpublic_key_t_pdu(struct l2cc_pdu *pdu, void *public_key)
{
    // Note - the Public Key is transmitted Least Significant Octet First
    uint8_t i;

    for (i = 0; i < 32; i++)
    {
        pdu->data.public_key.x[i] = ((public_key_t *)public_key)->x[i];
        pdu->data.public_key.y[i] = ((public_key_t *)public_key)->y[i];
    }
}

/**
 ****************************************************************************************
 * @brief Construct DH Key PDU.
 * @param buf  Pointer to buffer where the PDU is packed
 * @param idx  Index of task for which the PDU is being built
 * @return PDU length
 *****************************************************************************************
 */

static void smpc_construct_dhkey_check_pdu(struct l2cc_pdu *pdu, void *dh_key)
{
    // Pack DH Key - 16 bytes
    memcpy((void *)&pdu->data.dhkey_check.check[0], dh_key, DHKEY_CHECK_LEN);
}

/**
 ****************************************************************************************
 * @brief Construct KeyPress Notification PDU.
 * @param buf  Pointer to buffer where the PDU is packed
 * @param idx  Index of task for which the PDU is being built
 * @return PDU length
 *****************************************************************************************
 */

static void smpc_construct_keypress_notification_pdu(struct l2cc_pdu *pdu, void *keyNotication)
{
    pdu->data.keypress_noticication.notification_type = *(uint8_t *)keyNotication;
}
#endif // (SECURE_CONNECTIONS)
/**
 ****************************************************************************************
 * @brief Handle received SMP Pair Request Command PDU.
 *        Can only be received by a slave
 * @param idx   Index of task that received the PDU
 * @param data  Pointer to buffer where the PDU is, from L2CC packet.
 *****************************************************************************************
 */
static void smpc_recv_pair_req_pdu(uint8_t conidx, struct l2cc_pdu *pdu)
{
    if (gapc_get_role(conidx) == ROLE_SLAVE)
    {
#if (BLE_PERIPHERAL)
        // Check the repeated attempts status
        uint8_t status = smpc_check_repeated_attempts(conidx);

        if (status == SMPC_REP_ATTEMPTS_NO_ERROR)
        {

            // Check if a pairing procedure is currently in progress
            // The current operation might be a signature
            if (gapc_env[conidx]->smpc.info.pair == NULL)
            {
                // pairing not supported at all
                if (!gapm_is_legacy_pairing_supp() && !gapm_is_sec_con_pairing_supp())
                {
                    status = SMP_ERROR_PAIRING_NOT_SUPP;
                }
                // check that minimum pairing authentication requirements are present
                else if (((pdu->data.pairing_req.auth & GAP_AUTH_SEC_CON) == 0) && !gapm_is_legacy_pairing_supp())
                {
                    uint8_t local_status = SMP_GEN_PAIR_FAIL_REASON(SMP_PAIR_FAIL_REASON_MASK, SMP_ERROR_AUTH_REQ);
                    status = SMP_ERROR_AUTH_REQ;

                    smpc_send_pairing_ind(conidx, GAPC_PAIRING_FAILED, (void *)&local_status);
                }
                else
                {
                    // generate a local bond command to update correctly internal state machine
                    struct gapc_bond_cmd *cmd = KE_MSG_ALLOC(GAPC_BOND_CMD,
                                                KE_BUILD_ID(TASK_GAPC, conidx), KE_BUILD_ID(TASK_GAPC, conidx),
                                                gapc_bond_cmd);
                    cmd->operation = GAPC_BOND;

                    // Keep the packet for later use
                    memcpy(&(cmd->pairing), &(pdu->data.pairing_req.iocap), SMPC_CODE_PAIRING_REQ_RESP_LEN - 1);

                    ke_msg_send(cmd);
                }

                if (status != SMPC_REP_ATTEMPTS_NO_ERROR)
                {
                    // Send a Pairing Failed PDU to the slave device
                    smpc_pdu_send(conidx, L2C_CODE_PAIRING_FAILED, (void *)&status);
                }
            }
            // else drop the packet, a pairing procedure is in progress
        }
        else if (status == SMPC_REP_ATTEMPT)
        {
            // Send a Pairing Failed PDU to the slave device
            smpc_pdu_send(conidx, L2C_CODE_PAIRING_FAILED, (void *)&status);
        }
        // else ignore the request
#endif //(BLE_PERIPHERAL)
    }
    // else ignore the request
}

/**
 ****************************************************************************************
 * @brief Handle received SMP Pair Response Command PDU.
 *        Only a master can receive this packet.
 * @param idx   Index of task that received the PDU
 * @param data  Pointer to buffer where the PDU is, from L2CC packet.
 *****************************************************************************************
 */
static void smpc_recv_pair_rsp_pdu(uint8_t conidx, struct l2cc_pdu *pdu)
{
    uint8_t role = gapc_get_role(conidx);
    if (role == ROLE_MASTER)
    {
#if (BLE_CENTRAL)
        // Check if the device was waiting for the Pairing Response PDU
        if (gapc_env[conidx]->smpc.state == SMPC_PAIRING_RSP_WAIT)
        {
            ASSERT_ERR(gapc_env[conidx]->smpc.info.pair != NULL);

            uint8_t status = GAP_ERR_NO_ERROR;

            do
            {
                // check pairing feature
                if (!smpc_check_pairing_feat((struct gapc_pairing *)&pdu->data.pairing_rsp.iocap))
                {
                    status = SMP_ERROR_INVALID_PARAM;
                    break;
                }
                ASSERT_ERR(gapc_get_operation(conidx, GAPC_OP_SMP) == GAPC_BOND);

                // Keep the packet for confirm calculation use
                memcpy(&gapc_env[conidx]->smpc.info.pair->pair_rsp_feat,
                       &(pdu->data.pairing_rsp.iocap), SMPC_CODE_PAIRING_REQ_RESP_LEN - 1);

#if (SECURE_CONNECTIONS)
                if ((gapc_env[conidx]->smpc.info.pair->pair_rsp_feat.auth & GAP_AUTH_SEC_CON) &&
                        (gapc_env[conidx]->smpc.info.pair->pair_req_feat.auth & GAP_AUTH_SEC_CON))

                {
                    gapc_env[conidx]->smpc.secure_connections_enabled = true;
                }

                // Check that secure connection mode is not required
                if (!gapc_env[conidx]->smpc.secure_connections_enabled && !gapm_is_legacy_pairing_supp())
                {
                    status = SMP_ERROR_AUTH_REQ;
                    break;
                }
#endif // (SECURE_CONNECTIONS)

                // Check the encryption key size
                if (!smpc_check_max_key_size(conidx))
                {
                    status = SMP_ERROR_ENC_KEY_SIZE;
                    break;
                }
                // Check if the negociated key distribution allows to reach the required security level
                if (!smpc_check_key_distrib(conidx, gapc_env[conidx]->smpc.info.pair->pair_req_feat.sec_req))
                {
                    status = SMP_ERROR_UNSPECIFIED_REASON;
                    break;
                }

                // Select security properties and STK generation type
                smpc_get_key_sec_prop(conidx);

                // Check if the required security mode can be reached
                if (!smpc_is_sec_mode_reached(conidx, role))
                {
                    /*
                     * The pairing procedure cannot be performed as authentication requirements cannot
                     * be met due to IO capabilities of one or both devices.
                     */
                    status = SMP_ERROR_AUTH_REQ;
                    break;
                }

#if (SECURE_CONNECTIONS)
                if (smpc_secure_connections_enabled(conidx) == true)
                {
                    // Master response to Pairing_Rsp
                    // Send local Public Key to the Peer.

                    public_key_t *p_public_key;

                    p_public_key = gapm_get_local_public_key();

                    // Create a PDU to carry Public key to peer.
                    smpc_pdu_send(conidx, L2C_CODE_PUBLIC_KEY,
                                  (void *)p_public_key);

                    gapc_env[conidx]->smpc.state = SMPC_PAIRING_SC_W4_PEER_PUBLIC_KEY;
                }
                else
#endif // (SECURE_CONNECTIONS)
                {
                    // If Just Works, the TK is 0
                    if (gapc_env[conidx]->smpc.info.pair->pair_method != SMPC_METH_JW)
                    {
                        // Send a TK request to the HL
                        smpc_send_pairing_req_ind(conidx, GAPC_TK_EXCH);

                        gapc_env[conidx]->smpc.state = SMPC_PAIRING_TK_WAIT;
                    }
                    else
                    {
                        // Force to no MITM
                        gapc_env[conidx]->smpc.info.pair->auth &= ~GAP_AUTH_MITM;

                        // Start Rand Number Generation
                        smpc_generate_rand(conidx, SMPC_PAIRING_GEN_RAND_P1);
                    }
                }
            }
            while (0);

            if (status != GAP_ERR_NO_ERROR)
            {
                // Send a Pairing Failed PDU to the slave device
                smpc_pdu_send(conidx, L2C_CODE_PAIRING_FAILED, (void *)&status);

                status = SMP_GEN_PAIR_FAIL_REASON(SMP_PAIR_FAIL_REASON_MASK, status);

                // Send a complete event to the HL
                smpc_pairing_end(conidx, ROLE_SLAVE, status, true);
            }
        }
        // else ignore the message
#endif //(BLE_CENTRAL)
    }
    // else ignore the message
}

/**
 ****************************************************************************************
 * @brief Handle received SMP Pair Confirm Command PDU.
 * @param idx   Index of task that received the PDU
 * @param data  Pointer to buffer where the PDU is, from L2CC packet.
 *****************************************************************************************
 */
static void smpc_recv_pair_cfm_pdu(uint8_t conidx, struct l2cc_pdu *pdu)
{
    uint8_t role = gapc_get_role(conidx);

#if (SECURE_CONNECTIONS)
    if (smpc_secure_connections_enabled(conidx) == false)
#endif // (SECURE_CONNECTIONS)
    {
        if (role == ROLE_MASTER)
        {
#if (BLE_CENTRAL)
            if (gapc_env[conidx]->smpc.state == SMPC_PAIRING_WAIT_CONFIRM)
            {
                ASSERT_ERR(gapc_env[conidx]->smpc.info.pair != NULL);

                // Store the confirm value
                memcpy(&(gapc_env[conidx]->smpc.info.pair->conf_value[0]),
                       &(pdu->data.pairing_cfm.cfm_val[0]), CFM_LEN);

                // Send the generated random value
                smpc_pdu_send(conidx, L2C_CODE_PAIRING_RANDOM,
                              (void *) & (gapc_env[conidx]->smpc.info.pair->rand[0]));

                // Update the internal state
                gapc_env[conidx]->smpc.state = SMPC_PAIRING_WAIT_RAND;
            }
            // else ignore the message
#endif //(BLE_CENTRAL)
        }
        else    // role == ROLE_SLAVE
        {
#if (BLE_PERIPHERAL)
            if (gapc_env[conidx]->smpc.state == SMPC_PAIRING_WAIT_CONFIRM)
            {
                ASSERT_ERR(gapc_env[conidx]->smpc.info.pair != NULL);

                // Store the confirm value
                memcpy(&(gapc_env[conidx]->smpc.info.pair->conf_value[0]),
                       &(pdu->data.pairing_cfm.cfm_val[0]), CFM_LEN);

                // Generate random number
                smpc_generate_rand(conidx, SMPC_PAIRING_GEN_RAND_P1);
            }
            else if (gapc_env[conidx]->smpc.state == SMPC_PAIRING_TK_WAIT)
            {
                ASSERT_ERR(gapc_env[conidx]->smpc.info.pair != NULL);

                // Store the confirm value
                memcpy(&(gapc_env[conidx]->smpc.info.pair->conf_value[0]),
                       &(pdu->data.pairing_cfm.cfm_val[0]), CFM_LEN);

                // The peer device confirm value is received, but the TK has not been provided
                gapc_env[conidx]->smpc.state = SMPC_PAIRING_TK_WAIT_CONF_RCV;
            }
            // else ignore the message
#endif //(BLE_PERIPHERAL)
        }
    }
#if (SECURE_CONNECTIONS)
    else
    {
        memcpy(&(gapc_env[conidx]->smpc.info.pair->conf_value[0]),
               &(pdu->data.pairing_cfm.cfm_val[0]), CFM_LEN);

        // Handling for Secure Connections
        if ((gapc_env[conidx]->smpc.info.pair->pair_method == SMPC_METH_JW) ||
                (gapc_env[conidx]->smpc.info.pair->pair_method == SMPC_METH_NC))
        {
            if (gapc_env[conidx]->smpc.state == SMPC_PAIRING_SC_W4_PEER_COMMITMENT)
            {
                if (role == ROLE_MASTER)
                {
                    // Start the generation of a Random number Nb
                    smpc_generate_rand(conidx, SMPC_PAIRING_SC_W4_LOCAL_RAND_N_P1); //SMPC_PAIRING_GEN_RAND_P1);
                    gapc_env[conidx]->smpc.state = SMPC_PAIRING_SC_W4_LOCAL_RAND_N_P1;
                }
            }
        }
        else if (gapc_env[conidx]->smpc.info.pair->pair_method == SMPC_METH_PK)
        {
            if (role == ROLE_SLAVE)
            {
                if (gapc_env[conidx]->smpc.state == SMPC_PAIRING_SC_PASSKEY_W4_PEER_COMMITMENT)
                {
                    // Request Random Number from HCI
                    gapc_env[conidx]->smpc.state = SMPC_PAIRING_SC_PASSKEY_W4_LOCAL_RAND_N_P1;
                    smpc_generate_rand(conidx, SMPC_PAIRING_SC_PASSKEY_W4_LOCAL_RAND_N_P1);
                }
                else
                {
                    // GF 23 July 2015
                    // We have recieved the Confirm Value before the our App has entered the passkey.
                    // Set a flag to indicate peers confirm has been recieved.

                    gapc_env[conidx]->smpc.info.pair->peer_confirm_received = true;

                }
            }
            else // role == ROLE_MASTER
            {
                if (gapc_env[conidx]->smpc.state == SMPC_PAIRING_SC_PASSKEY_W4_PEER_COMMITMENT)
                {
                    // Send locally generated Random number to peer.
                    smpc_pdu_send(conidx, L2C_CODE_PAIRING_RANDOM, (void *)&gapc_env[conidx]->smpc.info.pair->rand[0]);

                    gapc_env[conidx]->smpc.state = SMPC_PAIRING_SC_PASSKEY_W4_PEER_RAND;
                }
                else
                {
                    // ERROR
                }

            }
        }
    }
#endif // (SECURE_CONNECTIONS)
}

/**
 ****************************************************************************************
 * @brief Handle received SMP Pair Rand Command PDU.
 * @param idx   Index of task that received the PDU
 * @param data  Pointer to buffer where the PDU is, from L2CC packet.
 *****************************************************************************************
 */
static void smpc_recv_pair_rand_pdu(uint8_t conidx, struct l2cc_pdu *pdu)
{
    if (gapc_env[conidx]->smpc.info.pair != NULL)
    {
#if (SECURE_CONNECTIONS)
        uint8_t auth_type = gapc_env[conidx]->smpc.info.pair->pair_method;
        uint8_t role = GAPC_GET_FIELD(conidx, ROLE);
#endif //  (SECURE_CONNECTIONS)

        // Check if the device was waiting for a master identifier
        if (gapc_env[conidx]->smpc.state == SMPC_PAIRING_WAIT_RAND)
        {
            // Store the random number
            memcpy(&(gapc_env[conidx]->smpc.info.pair->rem_rand[0]), &(pdu->data.pairing_random.random[0]), RAND_VAL_LEN);

            gapc_env[conidx]->smpc.state = SMPC_PAIRING_REM_CFM_P1;

            // Start the check confirm value procedure
            smpc_generate_e1(conidx, gapc_get_role(conidx), false);
        }
#if (SECURE_CONNECTIONS)
        else if ((role == ROLE_SLAVE) && (gapc_env[conidx]->smpc.state == SMPC_PAIRING_SC_W4_PEER_RAND))
        {
            if ((auth_type == SMPC_METH_JW) || (auth_type == SMPC_METH_NC))
            {
                //  I save the peers rand and transmit my local rand and progress to the generating Va.
                memcpy(&(gapc_env[conidx]->smpc.info.pair->rem_rand[0]), &(pdu->data.pairing_random.random[0]), RAND_VAL_LEN);

                // Send the locally generated random value
                smpc_pdu_send(conidx, L2C_CODE_PAIRING_RANDOM,
                              (void *) & (gapc_env[conidx]->smpc.info.pair->rand[0]));

                if (gapc_env[conidx]->smpc.info.pair->pair_method == SMPC_METH_NC)
                {
                    // Im a slave so !
                    // Pka is peers Public Key
                    // Pkb is my device Public Key
                    // Na is peers Random Number
                    // Nb is my local Random Number
                    public_key_t *Pka = &gapc_env[conidx]->smpc.info.pair->peer_public_key;
                    public_key_t *Pkb;
                    uint8_t *Na = &gapc_env[conidx]->smpc.info.pair->rem_rand[0];
                    uint8_t *Nb = &gapc_env[conidx]->smpc.info.pair->rand[0];
                    Pkb = gapm_get_local_public_key();

                    smpc_g2_init(conidx, Pka->x, Pkb->x, Na, Nb);

                    gapc_env[conidx]->smpc.state = SMPC_PAIRING_SC_W4_G2_AES_CMAC;
                }
                else if (gapc_env[conidx]->smpc.info.pair->pair_method == SMPC_METH_JW)
                {
                    // Begin the next phase of Secure Connection Authentication
                    smpc_init_mac_key_calculation(conidx);
                }

            }
        }
        else if ((role == ROLE_MASTER) && (gapc_env[conidx]->smpc.state == SMPC_PAIRING_SC_W4_PEER_RAND))
        {
            if ((auth_type == SMPC_METH_JW) || (auth_type == SMPC_METH_NC))
            {
                // I save the peers rand and check the commitment.
                memcpy(&(gapc_env[conidx]->smpc.info.pair->rem_rand[0]), &(pdu->data.pairing_random.random[0]), RAND_VAL_LEN);

                // Check the commitment
                {
                    // Begin the Local Commitment checl -
                    public_key_t *Pka;
                    public_key_t *Pkb = &(gapc_env[conidx]->smpc.info.pair->peer_public_key);
                    uint8_t *Nb = &gapc_env[conidx]->smpc.info.pair->rem_rand[0];

                    Pka = gapm_get_local_public_key();

                    gapc_env[conidx]->smpc.state = SMPC_PAIRING_SC_W4_F4_COMMITMENT_CHECK;
                    // Cb = f4(Pkb,Pka,Nb,0)
                    smpc_f4_Init(conidx, Pkb->x, Pka->x, Nb, 0);
                }
            }
        }
        else if ((role == ROLE_SLAVE) && (gapc_env[conidx]->smpc.state == SMPC_PAIRING_SC_PASSKEY_W4_PEER_RAND))
        {
            if (auth_type == SMPC_METH_PK)
            {
                //  I save the peers rand and transmit my local rand and progress to the generating Va.
                memcpy(&(gapc_env[conidx]->smpc.info.pair->rem_rand[0]), &(pdu->data.pairing_random.random[0]), RAND_VAL_LEN);

                {
                    // Im a slave so !
                    // Pka is peers Public Key
                    // Pkb is my device Public Key
                    // Nai is peers Random Number
                    // Rbi is bit of current Passkey
                    public_key_t *Pka = &gapc_env[conidx]->smpc.info.pair->peer_public_key;
                    public_key_t *Pkb;
                    uint8_t *Nai = &gapc_env[conidx]->smpc.info.pair->rem_rand[0];
                    uint8_t  Rbi = smpc_get_next_passkey_bit(conidx) | 0x80;
                    Pkb = gapm_get_local_public_key();

                    // Cai = f4(Pka,Pkb,Nai,rbi)
                    smpc_f4_Init(conidx, Pka->x, Pkb->x, Nai, Rbi);

                    gapc_env[conidx]->smpc.state = SMPC_PAIRING_SC_PASSKEY_W4_F4_COMMITMENT_CHECK;
                }
            }
        }
        else if ((role == ROLE_MASTER) && (gapc_env[conidx]->smpc.state == SMPC_PAIRING_SC_PASSKEY_W4_PEER_RAND))
        {
            if (auth_type == SMPC_METH_PK)
            {
                //  I save the peers rand and transmit my local rand and progress to the generating Va.
                memcpy(&(gapc_env[conidx]->smpc.info.pair->rem_rand[0]), &(pdu->data.pairing_random.random[0]), RAND_VAL_LEN);

                {
                    // Im a Master so !
                    // Pka is local Public Key
                    // Pkb is peers Public Key
                    // Nbi is peers Random Number
                    // Rai is bit of current Passkey
                    public_key_t *Pka = gapm_get_local_public_key();
                    public_key_t *Pkb = &gapc_env[conidx]->smpc.info.pair->peer_public_key;
                    uint8_t *Nbi = &gapc_env[conidx]->smpc.info.pair->rem_rand[0];
                    uint8_t  Rai = smpc_get_next_passkey_bit(conidx) | 0x80;

                    // Cbi = f4(Pkb,Pka,Nai,rai)
                    smpc_f4_Init(conidx, Pkb->x, Pka->x, Nbi, Rai);

                    gapc_env[conidx]->smpc.state = SMPC_PAIRING_SC_PASSKEY_W4_F4_COMMITMENT_CHECK;
                }
            }
        }
        else if (auth_type == SMPC_METH_OOB)  // - (GAPC_GET_FIELD(conidx, ROLE) == ROLE_MASTER)
        {
            if (role == ROLE_SLAVE)
            {
                // The slave can Rx the Rand(Na) from the master at any point in its OOB state machine
                // However if it is in the SMPC_PAIRING_SC_OOB_W4_PEER_RAND  state we initiate the
                // generation of local rand (Nb)
                gapc_env[conidx]->smpc.info.pair->peer_rand_received = true;
                memcpy(&(gapc_env[conidx]->smpc.info.pair->rem_rand[0]), &(pdu->data.pairing_random.random[0]), RAND_VAL_LEN);

                if (gapc_env[conidx]->smpc.state == SMPC_PAIRING_SC_OOB_W4_PEER_RAND)
                {
                    gapc_env[conidx]->smpc.state = SMPC_PAIRING_SC_OOB_W4_LOCAL_RAND_N_P1;
                    smpc_generate_rand(conidx, SMPC_PAIRING_SC_OOB_W4_LOCAL_RAND_N_P1);
                }
                else
                {
                    // Have to wait until the verification of peers commitment Ca is complete
                }

            }
            else if (gapc_env[conidx]->smpc.state == SMPC_PAIRING_SC_OOB_W4_PEER_RAND)
            {
                // Master will only validly Rx the Rand(Nb) in this state.
                gapc_env[conidx]->smpc.info.pair->peer_rand_received = true;
                memcpy(&(gapc_env[conidx]->smpc.info.pair->rem_rand[0]), &(pdu->data.pairing_random.random[0]), RAND_VAL_LEN);
                smpc_init_mac_key_calculation(conidx);

            }
        }
#endif // (SECURE_CONNECTIONS)
    }
}

/**
 ****************************************************************************************
 * @brief Handle received SMP Pair Fail Command PDU.
 * @param idx   Index of task that received the PDU
 * @param data  Pointer to buffer where the PDU is, from L2CC packet.
 *****************************************************************************************
 */
static void smpc_recv_pair_fail_pdu(uint8_t conidx, struct l2cc_pdu *pdu)
{
    uint8_t status = SMP_GEN_PAIR_FAIL_REASON(SMP_PAIR_FAIL_REASON_REM_MASK,
                     pdu->data.pairing_failed.reason);

    // Check if a pairing procedure is in progress
    if (gapc_env[conidx]->smpc.info.pair != NULL)
    {
        uint8_t role = gapc_get_role(conidx);

        // Inform the HL that an error has occurred.
        smpc_pairing_end(conidx, role, status, false);
    }
    else
    {
        smpc_send_pairing_ind(conidx, GAPC_PAIRING_FAILED, (void *)&status);
    }
    // else ignore the message
}

/**
 ****************************************************************************************
 * @brief Handle received SMP Encryption Information Command PDU.
 * @param idx   Index of task that received the PDU
 * @param data  Pointer to buffer where the PDU is, from L2CC packet.
 *****************************************************************************************
 */
static void smpc_recv_enc_info_pdu(uint8_t conidx, struct l2cc_pdu *pdu)
{
    // Check if the device was waiting for a master identifier
    if (gapc_env[conidx]->smpc.state == SMPC_PAIRING_REM_LTK_WAIT)
    {
        ASSERT_ERR(gapc_env[conidx]->smpc.info.pair != NULL);

        // Store the LTK and wait for the BD Address
        memcpy(&(gapc_env[conidx]->smpc.info.pair->key.key[0]), &(pdu->data.encryption_inf.ltk[0]), GAP_KEY_LEN);

        // Check the next step of the TKDP
        smpc_tkdp_rcp_continue(conidx, gapc_get_role(conidx));
    }
    // else ignore the message
}

/**
 ****************************************************************************************
 * @brief Handle received SMP Master ID Command PDU.
 * @param idx   Index of task that received the PDU
 * @param data  Pointer to buffer where the PDU is, from L2CC packet.
 *****************************************************************************************
 */
static void smpc_recv_mst_id_pdu(uint8_t conidx, struct l2cc_pdu *pdu)
{
    // Check if the device was waiting for a master identifier
    if (gapc_env[conidx]->smpc.state == SMPC_PAIRING_REM_MST_ID_WAIT)
    {
        ASSERT_ERR(gapc_env[conidx]->smpc.info.pair != NULL);

        struct gapc_ltk ltk;

        // Copy the EDIV
        ltk.ediv = co_read16p(&(pdu->data.master_id.ediv));
        // Copy the Rand Number
        memcpy(&(ltk.randnb.nb[0]), &(pdu->data.master_id.nb[0]), GAP_RAND_NB_LEN);
        // Copy the LTK
        memcpy(&(ltk.ltk.key[0]), &(gapc_env[conidx]->smpc.info.pair->key.key[0]), GAP_KEY_LEN);
        // Copy key size
        ltk.key_size = gapc_enc_keysize_get(conidx);
        // Send the LTK, the EDIV and the Rand Nb to the HL
        smpc_send_pairing_ind(conidx, GAPC_LTK_EXCH, (void *)&ltk);

        // Check the next step of the TKDP
        smpc_tkdp_rcp_continue(conidx, gapc_get_role(conidx));
    }
    // else ignore the message
}

/**
 ****************************************************************************************
 * @brief Handle received SMP Identity Information Command PDU.
 * @param idx   Index of task that received the PDU
 * @param data  Pointer to buffer where the PDU is, from L2CC packet.
 *****************************************************************************************
 */
static void smpc_recv_id_info_pdu(uint8_t conidx, struct l2cc_pdu *pdu)
{
    // Check if the device was waiting for a BD Address
    if (gapc_env[conidx]->smpc.state == SMPC_PAIRING_REM_IRK_WAIT)
    {
        ASSERT_ERR(gapc_env[conidx]->smpc.info.pair != NULL);

        // Store the IRK and wait for the BD Address
        memcpy(&(gapc_env[conidx]->smpc.info.pair->key.key[0]), &(pdu->data.identity_inf.irk[0]), GAP_KEY_LEN);

        // Check the next step of the TKDP
        smpc_tkdp_rcp_continue(conidx, gapc_get_role(conidx));
    }
    // else ignore the message
}

/**
 ****************************************************************************************
 * @brief Handle received SMP Identity Address Information Command PDU.
 * @param idx   Index of task that received the PDU
 * @param data  Pointer to buffer where the PDU is, from L2CC packet.
 *****************************************************************************************
 */
static void smpc_recv_id_addr_info_pdu(uint8_t conidx, struct l2cc_pdu *pdu)
{
    // Check if the device was waiting for a BD Address
    if (gapc_env[conidx]->smpc.state == SMPC_PAIRING_REM_BD_ADDR_WAIT)
    {
        ASSERT_ERR(gapc_env[conidx]->smpc.info.pair != NULL);

        struct gapc_irk irk;

        // Copy the BD Address
        memcpy(&(irk.addr.addr.addr[0]), &(pdu->data.id_addr_inf.addr.addr[0]), BD_ADDR_LEN);
        // Copy the BD Address Type
        irk.addr.addr_type = pdu->data.id_addr_inf.addr_type;
        // Copy the IRK
        memcpy(&(irk.irk.key[0]), &(gapc_env[conidx]->smpc.info.pair->key.key[0]), GAP_KEY_LEN);

        // Send the IRK and the BD Address to the HL
        smpc_send_pairing_ind(conidx, GAPC_IRK_EXCH, (void *)&irk);

        // Check the next step of the TKDP
        smpc_tkdp_rcp_continue(conidx, gapc_get_role(conidx));
    }
    // else ignore the message
}

/**
 ****************************************************************************************
 * @brief Handle received SMP Signature Information Command PDU.
 * @param idx   Index of task that received the PDU
 * @param data  Pointer to buffer where the PDU is, from L2CC packet.
 *****************************************************************************************
 */
static void smpc_recv_sign_info_pdu(uint8_t conidx, struct l2cc_pdu *pdu)
{
    // Check if the device was waiting for a CSRK
    if (gapc_env[conidx]->smpc.state == SMPC_PAIRING_REM_CSRK_WAIT)
    {
        ASSERT_ERR(gapc_env[conidx]->smpc.info.pair != NULL);

        // Send the CSRK to the HL
        smpc_send_pairing_ind(conidx, GAPC_CSRK_EXCH, (void *) & (pdu->data.signing_inf.csrk[0]));

        // Check the next step of the TKDP
        smpc_tkdp_rcp_continue(conidx, gapc_get_role(conidx));
    }
    // else ignore the message
}

/**
 ****************************************************************************************
 * @brief Handle received SMP Security Request Command PDU. Can only be Rx by Master
 * @param idx   Index of task that received the PDU
 * @param data  Pointer to buffer where the PDU is, from L2CC packet.
 *****************************************************************************************
 */
static void smpc_recv_sec_req_pdu(uint8_t conidx, struct l2cc_pdu *pdu)
{
    if (gapc_get_role(conidx) == ROLE_MASTER)
    {
#if (BLE_CENTRAL)
        // Check the repeated attempts status
        uint8_t status = smpc_check_repeated_attempts(conidx);

        if (status == SMPC_REP_ATTEMPTS_NO_ERROR)
        {
            if (((pdu->data.security_req.auth & GAP_AUTH_SEC_CON) == 0) && !gapm_is_legacy_pairing_supp())
            {
                uint8_t local_status = SMP_GEN_PAIR_FAIL_REASON(SMP_PAIR_FAIL_REASON_MASK, SMP_ERROR_AUTH_REQ);
                status = SMP_ERROR_AUTH_REQ;

                // Send a Pairing Failed PDU to the slave device
                smpc_pdu_send(conidx, L2C_CODE_PAIRING_FAILED, (void *)&status);
                smpc_send_pairing_ind(conidx, GAPC_PAIRING_FAILED, (void *)&local_status);
            }
            else
            {
                // Check if a pairing procedure is currently in progress
                // The current operation might be a signature
                if (gapc_env[conidx]->smpc.info.pair == NULL)
                {
                    // Inform the HL that the slave wants to start a pairing or an encryption procedure
                    struct gapc_security_ind *req_ind = KE_MSG_ALLOC(GAPC_SECURITY_IND,
                                                        gapc_get_dest_task(conidx), KE_BUILD_ID(TASK_GAPC, conidx),
                                                        gapc_security_ind);

                    req_ind->auth = (pdu->data.security_req.auth & GAP_AUTH_REQ_MASK);
                    ke_msg_send(req_ind);
                }
                else
                {
                    // Ignore the request
                    ASSERT_ERR(gapc_get_operation(conidx, GAPC_OP_SMP) == GAPC_BOND);
                }
            }
        }
        // else ignore the request
#endif //(BLE_CENTRAL)
    }
    // else ignore the request
}

#if (SECURE_CONNECTIONS)
static void smpc_recv_public_key_pdu(uint8_t conidx, struct l2cc_pdu *pdu)
{
    // Check if pairing is taking place.
    // Check State..
    // Malloc memory for the Public Keys.


    // If (role == MASTER)
    //   copy Peers Public Key
    //   initiate computation of DHkey
    // else
    //   copy Public Keys
    //   initiate computation of DHkey
    //   Transmit Local Public Keys

    // Next Action is dependent on the Association Model Selected.
    // A /Just Works || Numeric Comparison || PassKey)
    //    Begin Initiate Random Number
    // B/ Wait for OOB Data

    if (gapc_env[conidx]->smpc.state == SMPC_PAIRING_SC_W4_PEER_PUBLIC_KEY)
    {
        uint8_t role = gapc_get_role(conidx);
        ASSERT_ERR(gapc_env[conidx]->smpc.info.pair != NULL);

        // Stored The Public Key X and Y co-ordinates.
        memcpy(&(gapc_env[conidx]->smpc.info.pair->peer_public_key.x[0]), &(pdu->data.public_key.x[0]), SMPC_PUBLIC_KEY_256_COORD_LEN);
        memcpy(&(gapc_env[conidx]->smpc.info.pair->peer_public_key.y[0]), &(pdu->data.public_key.y[0]), SMPC_PUBLIC_KEY_256_COORD_LEN);

        smpc_send_gen_dh_key_cmd(conidx, &(gapc_env[conidx]->smpc.info.pair->peer_public_key.x[0]),
                                 &(gapc_env[conidx]->smpc.info.pair->peer_public_key.y[0]));

        if (role == ROLE_SLAVE)
        {
            // If slave I have to sent my own Public Key to the peer.

            public_key_t *p_public_key;

            p_public_key = gapm_get_local_public_key();


            // Create a PDU to carry Public key to peer.
            smpc_pdu_send(conidx, L2C_CODE_PUBLIC_KEY,
                          (void *)p_public_key);

        }

        // Next State is dependent on the Association Model.
        switch (gapc_env[conidx]->smpc.info.pair->pair_method)
        {
        case SMPC_METH_JW  :
        case SMPC_METH_NC  :
            if (role == ROLE_SLAVE)
            {
                // Start the generation of a Random number Nb
                smpc_generate_rand(conidx, SMPC_PAIRING_SC_W4_LOCAL_RAND_N_P1); //SMPC_PAIRING_GEN_RAND_P1);
                gapc_env[conidx]->smpc.state = SMPC_PAIRING_SC_W4_LOCAL_RAND_N_P1;
            }
            else
            {
                // In the master we wait until we Rx the slave confirm - Cb, before we generate the local rand.
                gapc_env[conidx]->smpc.state = SMPC_PAIRING_SC_W4_PEER_COMMITMENT;
            }
            break;

        case SMPC_METH_PK  :
            // Send a TK request to the HL - Assume Passkey is generated by Application if needed
        {
            smpc_send_pairing_req_ind(conidx, GAPC_TK_EXCH);
            gapc_env[conidx]->smpc.state = SMPC_PAIRING_TK_WAIT;
        }
        break;


        case SMPC_METH_OOB :
            // Start the generation of a Random number R
            smpc_generate_rand(conidx, SMPC_PAIRING_SC_OOB_W4_LOCAL_RAND_R_P1);
            gapc_env[conidx]->smpc.state = SMPC_PAIRING_SC_OOB_W4_LOCAL_RAND_R_P1;
            break;

        default :
            break;
        }
    }
    else
    {
        // Received UnExpected PUBLIC Key

    }

}

static void smpc_recv_dhkey_check_pdu(uint8_t idx, struct l2cc_pdu *pdu)
{
    //
    // DHKey Check received from the peer device.
    // In a slave device this can come asynchronously from the master - during Number verification OR
    // during the f5 algorithm..  Thus if we are not in the W4_PEER_DH_KEY_CHECK state, we simply record
    // the receipt of the DHKey_Check and process at the end of the f5 algorithm.
    uint8_t role        = gapc_get_role(idx);

    if (gapc_env[idx]->smpc.info.pair != NULL)
    {
        gapc_env[idx]->smpc.info.pair->dh_key_check_received_from_peer = true;
        // Save the Peers DHkey Check.
        memcpy(gapc_env[idx]->smpc.info.pair->dh_key_check_peer, &(pdu->data.dhkey_check.check[0]), DHKEY_CHECK_LEN);

        if (gapc_env[idx]->smpc.state == SMPC_PAIRING_SC_W4_PEER_DHKEY_CHECK)
        {
            if (role == MASTER_ROLE)
            {
                gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_W4_F6_DHKEY_VERIFICATION;
                smpc_initiate_dhkey_verification(idx);
            }
            else
            {
                // If we have already received the DH_Key we can peform DH_Key check immediately
                gapc_env[idx]->smpc.state = SMPC_PAIRING_SC_W4_F6_DHKEY_CHECK;
                smpc_initiate_dhkey_check(idx);
            }
        }
    }
}

static void smpc_recv_keypress_notification_pdu(uint8_t conidx, struct l2cc_pdu *pdu)
{
    // Check if we are waiting the TK (Pairing procedure on going)
    if ((gapc_env[conidx]->smpc.state > SMPC_STATE_RESERVED) && (gapc_env[conidx]->smpc.state < SMPC_SIGN_L_GEN))
    {
        if (ke_timer_active(GAPC_SMP_TIMEOUT_TIMER_IND, KE_BUILD_ID(TASK_GAPC, conidx)))
        {
            ke_timer_set(GAPC_SMP_TIMEOUT_TIMER_IND, KE_BUILD_ID(TASK_GAPC, conidx), SMPC_TIMEOUT_TIMER_DURATION);
        }

        // Send KeyPress Notification to the API
        smpc_key_press_notification_ind(conidx, pdu->data.keypress_noticication.notification_type);

    }
    else
    {
        ASSERT_ERR(0);
    }
}

#endif // (SECURE_CONNECTIONS)
/*
 * GLOBAL VARIABLES DEFINITIONS
 ****************************************************************************************
 */

/// SMPC Table with SMP commands PDU construction handlers
const smpc_construct_pdu_t smpc_construct_pdu[L2C_CODE_SECURITY_MAX] =
{
    [L2C_CODE_PAIRING_REQ]                  = (smpc_construct_pdu_t)smpc_construct_pair_req_pdu,
    [L2C_CODE_PAIRING_RESP]                 = (smpc_construct_pdu_t)smpc_construct_pair_rsp_pdu,
    [L2C_CODE_PAIRING_CFM]                  = (smpc_construct_pdu_t)smpc_construct_pair_cfm_pdu,
    [L2C_CODE_PAIRING_RANDOM]               = (smpc_construct_pdu_t)smpc_construct_pair_rand_pdu,
    [L2C_CODE_PAIRING_FAILED]               = (smpc_construct_pdu_t)smpc_construct_pair_fail_pdu,
    [L2C_CODE_ENCRYPTION_INF]               = (smpc_construct_pdu_t)smpc_construct_enc_info_pdu,
    [L2C_CODE_MASTER_ID]                    = (smpc_construct_pdu_t)smpc_construct_mst_id_pdu,
    [L2C_CODE_IDENTITY_INF]                 = (smpc_construct_pdu_t)smpc_construct_id_info_pdu,
    [L2C_CODE_ID_ADDR_INF]                  = (smpc_construct_pdu_t)smpc_construct_id_addr_info_pdu,
#ifndef SMP_SLIM
    [L2C_CODE_SIGNING_INF]                  = (smpc_construct_pdu_t)smpc_construct_sign_info_pdu,
#endif //!SMP_SLIM
    [L2C_CODE_SECURITY_REQ]                 = (smpc_construct_pdu_t)smpc_construct_sec_req_pdu,
#if (SECURE_CONNECTIONS)
    [L2C_CODE_PUBLIC_KEY]                   = (smpc_construct_pdu_t)smpc_construcpublic_key_t_pdu,
    [L2C_CODE_DHKEY_CHECK]                  = (smpc_construct_pdu_t)smpc_construct_dhkey_check_pdu,
#ifndef SMP_SLIM
    [L2C_CODE_KEYPRESS_NOTIFICATION]        = (smpc_construct_pdu_t)smpc_construct_keypress_notification_pdu,
#endif // !SMP_SLIM
#endif //  (SECURE_CONNECTIONS)
};

/// SMPC Table with SMP command PDU reception handlers
const smpc_recv_pdu_t smpc_recv_pdu[L2C_CODE_SECURITY_MAX] =
{
    [L2C_CODE_PAIRING_REQ]                  = (smpc_recv_pdu_t)smpc_recv_pair_req_pdu,
    [L2C_CODE_PAIRING_RESP]                 = (smpc_recv_pdu_t)smpc_recv_pair_rsp_pdu,
    [L2C_CODE_PAIRING_CFM]                  = (smpc_recv_pdu_t)smpc_recv_pair_cfm_pdu,
    [L2C_CODE_PAIRING_RANDOM]               = (smpc_recv_pdu_t)smpc_recv_pair_rand_pdu,
    [L2C_CODE_PAIRING_FAILED]               = (smpc_recv_pdu_t)smpc_recv_pair_fail_pdu,
    [L2C_CODE_ENCRYPTION_INF]               = (smpc_recv_pdu_t)smpc_recv_enc_info_pdu,
    [L2C_CODE_MASTER_ID]                    = (smpc_recv_pdu_t)smpc_recv_mst_id_pdu,
    [L2C_CODE_IDENTITY_INF]                 = (smpc_recv_pdu_t)smpc_recv_id_info_pdu,
    [L2C_CODE_ID_ADDR_INF]                  = (smpc_recv_pdu_t)smpc_recv_id_addr_info_pdu,
#ifndef SMP_SLIM
    [L2C_CODE_SIGNING_INF]                  = (smpc_recv_pdu_t)smpc_recv_sign_info_pdu,
#endif // !SMP_SLIM
    [L2C_CODE_SECURITY_REQ]                 = (smpc_recv_pdu_t)smpc_recv_sec_req_pdu,
#if (SECURE_CONNECTIONS)
    [L2C_CODE_PUBLIC_KEY]                   = (smpc_recv_pdu_t)smpc_recv_public_key_pdu,
    [L2C_CODE_DHKEY_CHECK]                  = (smpc_recv_pdu_t)smpc_recv_dhkey_check_pdu,
#ifndef SMP_SLIM
    [L2C_CODE_KEYPRESS_NOTIFICATION]        = (smpc_recv_pdu_t)smpc_recv_keypress_notification_pdu,
#endif //!SMP_SLIM
#endif // (SECURE_CONNECTIONS)
};

#endif //(BLE_SMPC)
/// @} SMPC_UTIL
