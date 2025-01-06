/**
 ****************************************************************************************
 *
 * @file gapm_smp.c
 *
 * @brief Generic Access Profile Manager Security manager handler module.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup GAPM_SMP Generic Access Profile Manager Security manager
 * @ingroup GAPM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#include "rwble_hl.h"
#include "rwip.h"            // Common API to retrieve device parameters

#include <string.h>

#include "co_error.h"
#include "co_bt.h"
#include "co_math.h"
#include "co_version.h"
#include "co_utils.h"        // core utility functions

#include "gap.h"
#include "gapm_task.h"
#include "gapm_int.h"

#include "ke_mem.h"

#include "hci.h"

#include "ke_timer.h"

#include "aes.h"

static ke_task_id_t  g_gapm_h6h7_requester;
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

/*
 * LOCAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

__STATIC void gapm_smp_resolv_op_cont(uint8_t status, const uint8_t *res);

/**
 ****************************************************************************************
 * @brief Call back definition of the function that can handle result of an AES based algorithm
 *
 * @param[in] status       Execution status
 * @param[in] aes_res      16 bytes block result
 * @param[in] src_info     Information provided by requester
 ****************************************************************************************
 */
__STATIC void gapm_smp_encrypt_rsp(uint8_t status, const uint8_t *aes_res, uint32_t src_info)
{
    // Check if a command is currently handled, drop the message if not the case.
    if (ke_state_get(TASK_GAPM) != GAPM_DEVICE_SETUP)
    {
        switch (gapm_get_operation(GAPM_OP_CFG))
        {
        // Generation Random Address
        case (GAPM_GEN_RAND_ADDR):
        {
            uint8_t status = GAP_ERR_NO_ERROR;

            // Check the returned status
            if (status == GAP_ERR_NO_ERROR)
            {
                // Get the command request
                struct gapm_gen_rand_addr_cmd *cmd
                    = (struct gapm_gen_rand_addr_cmd *)gapm_get_operation_ptr(GAPM_OP_CFG);
                // Indication sent to the requester
                struct gapm_dev_bdaddr_ind *ind = KE_MSG_ALLOC(GAPM_DEV_BDADDR_IND,
                                                  gapm_get_requester(GAPM_OP_CFG), TASK_GAPM,
                                                  gapm_dev_bdaddr_ind);

                // Set the prand part - Provided Address is LSB->MSB
                memcpy(&(ind->addr.addr.addr[GAP_ADDR_PRAND_LEN]), &(cmd->prand[0]), GAP_ADDR_PRAND_LEN);

                // Set the hash part
                memcpy(&(ind->addr.addr.addr[0]), aes_res, GAP_ADDR_PRAND_LEN);
                ind->addr.addr_type = ADDR_RAND;

                // Send the indication
                ke_msg_send(ind);
            }
            else
            {
                status = RW_ERR_HCI_TO_HL(status);
            }

            // Send the command complete event message
            gapm_send_complete_evt(GAPM_OP_CFG, status);
        }
        break;

        case (GAPM_RESOLV_ADDR):
        {
            // continue execution of the address resolving
            gapm_smp_resolv_op_cont(RW_ERR_HCI_TO_HL(status), aes_res);
        }
        break;

        case (GAPM_USE_ENC_BLOCK):
        {
            uint8_t status = GAP_ERR_NO_ERROR;

            // Check the returned status
            if (status == GAP_ERR_NO_ERROR)
            {
                // Indication sent to the requester
                struct gapm_use_enc_block_ind *ind = KE_MSG_ALLOC(GAPM_USE_ENC_BLOCK_IND,
                                                     gapm_get_requester(GAPM_OP_CFG), TASK_GAPM,
                                                     gapm_use_enc_block_ind);

                // Copy the result
                memcpy(&ind->result[0], aes_res, GAP_KEY_LEN);

                // Send the indication
                ke_msg_send(ind);
            }
            else
            {
                status = RW_ERR_HCI_TO_HL(status);
            }

            // Send the command complete event message
            gapm_send_complete_evt(GAPM_OP_CFG, status);
        }
        break;

        default:
        {
            // Drop the message
            ASSERT_ERR(0);
        }
        break;
        }
    }
}

/**
 ****************************************************************************************
 * @brief Send an encryption request.
 ****************************************************************************************
 */
__STATIC void gapm_smp_encrypt_req(const uint8_t *operand_1, const uint8_t *operand_2)
{
    // Ask for AES Execution
    aes_encrypt(operand_1, operand_2, true, gapm_smp_encrypt_rsp, 0);
}

/**
 ****************************************************************************************
 * @brief Call back definition of the function that can handle result of an AES based algorithm
 *
 * @param[in] status       Execution status
 * @param[in] aes_res      16 bytes block result
 * @param[in] src_info     Information provided by requester
 ****************************************************************************************
 */
__STATIC void gapm_smp_rand_rsp(uint8_t status, const uint8_t *aes_res, uint32_t src_info)
{
    // Check if a command is currently handled, drop the message if not the case.
    if (ke_state_get(TASK_GAPM) != GAPM_DEVICE_SETUP)
    {
        switch (gapm_get_operation(GAPM_OP_CFG))
        {
        // Generation Random Address
        case (GAPM_GEN_RAND_ADDR):
        {
            // Check the returned status
            if (status == GAP_ERR_NO_ERROR)
            {
                // Get the command request
                struct gapm_gen_rand_addr_cmd *cmd = (struct gapm_gen_rand_addr_cmd *)gapm_get_operation_ptr(GAPM_OP_CFG);
                // Prand' =  0[0:12] || prand (MSB->LSB in prand_bis)
                uint8_t prand_bis[GAP_KEY_LEN];

                // Generate prand value (MSB->LSB required for encryption function)
                memcpy(&cmd->prand[0], aes_res, GAP_ADDR_PRAND_LEN);

                // Set the address type flag, clear the 2 MSBits of prand
                cmd->prand[GAP_ADDR_PRAND_LEN - 1] &= 0x3F;
                // Set the address type value, MSB of prand is the addr_type
                cmd->prand[GAP_ADDR_PRAND_LEN - 1] |= cmd->rnd_type;

                // Clear prand_bis
                memset(&prand_bis[0], 0x00, GAP_KEY_LEN);
                // Copy prand value in prand_bis
                memcpy(&prand_bis[0], &cmd->prand[0], GAP_ADDR_PRAND_LEN);

                // Encrypt the prand value using the IRK value
                gapm_smp_encrypt_req(&(gapm_get_irk()->key[0]), &prand_bis[0]);
            }
            else
            {
                gapm_send_complete_evt(GAPM_OP_CFG, RW_ERR_HCI_TO_HL(status));
            }
        }
        break;

        case (GAPM_GEN_RAND_NB):
        {
            // Status
            uint8_t status = GAP_ERR_NO_ERROR;

            if (status == GAP_ERR_NO_ERROR)
            {
                // Send the generated random number to the requester task.
                struct gapm_gen_rand_nb_ind *ind = KE_MSG_ALLOC(GAPM_GEN_RAND_NB_IND,
                                                   gapm_get_requester(GAPM_OP_CFG), TASK_GAPM,
                                                   gapm_gen_rand_nb_ind);

                memcpy(&ind->randnb.nb[0], aes_res, GAP_RAND_NB_LEN);

                ke_msg_send(ind);
            }
            else
            {
                status = RW_ERR_HCI_TO_HL(status);
            }

            // Send a CMP_EVT message with the request status.
            gapm_send_complete_evt(GAPM_OP_CFG, status);
        }
        break;

        default:
        {
            // Drop the message
            ASSERT_ERR(0);
        }
        break;
        }
    }
}


/**
 ****************************************************************************************
 * @brief Send an rand request.
 ****************************************************************************************
 */
__STATIC void gapm_smp_rand_req(void)
{
    // ask for random number generation
    aes_rand(gapm_smp_rand_rsp, 0);
}

/**
 ****************************************************************************************
 * State machine use to resolve address using provided list of IRK
 *
 * @param status   Output status of encryption block or command request sanity check
 * @param res      Output result of encryption block
 ****************************************************************************************
 */
__STATIC void gapm_smp_resolv_op_cont(uint8_t status, const uint8_t *res)
{
    bool finished = false;
    // Get the command request
    struct gapm_resolv_addr_cmd *cmd = (struct gapm_resolv_addr_cmd *) gapm_get_operation_ptr(GAPM_OP_CFG);

    if (cmd != NULL)
    {

        if (status != GAP_ERR_NO_ERROR)
        {
            finished = true;
        }
        else if (res != NULL)
        {
            /*
             * Address provided from higher layers in LSB->MSB
             *      ----------------------------------------------------------------------------------------
             *      | hash[0:(GAP_ADDR_HASH_LEN-1)] | prand[(GAP_ADDR_HASH_LEN:(BD_ADDR_LEN-1)] |
             *      ----------------------------------------------------------------------------------------
             *
             * Encrypted Data received from LL is LSB->MSB
             *      ----------------------------------------------------------------------------------------------------
             *      | hash[0:(GAP_KEY_LEN-GAP_ADDR_HASH_LEN)] | data[((GAP_KEY_LEN-GAP_ADDR_HASH_LEN):(GAP_KEY_LEN-1)] |
             *      ----------------------------------------------------------------------------------------------------
             */

            // Compare the provided hash value and the generated hash value.
            if (!memcmp(&(cmd->addr.addr[0]), res, GAP_ADDR_HASH_LEN))
            {
                // Address resolution completed
                // Indicate which key has been used to resolve the random address.
                struct gapm_addr_solved_ind *solved_ind = KE_MSG_ALLOC(GAPM_ADDR_SOLVED_IND,
                        gapm_get_requester(GAPM_OP_CFG), TASK_GAPM,
                        gapm_addr_solved_ind);

                // Provide Address resolved
                memcpy(&(solved_ind->addr), &(cmd->addr.addr[0]), sizeof(bd_addr_t));

                // Provide IRK used for address resolution
                memcpy(&(solved_ind->irk), &(cmd->irk[cmd->nb_key]),
                       sizeof(struct gap_sec_key));

                // Send the message
                ke_msg_send(solved_ind);
                finished = true;
            }
        }

        if (!finished)
        {
            // Not able to solve random address.
            if (cmd->nb_key == 0)
            {
                status    = GAP_ERR_NOT_FOUND;
                finished = true;
            }
            else
            {
                // Prand' =  0[0:12] || prand (padding to have a 128-bit value)
                uint8_t prand_bis[GAP_KEY_LEN];

                // Check with latest key.
                cmd->nb_key--;

                // Clear prand_bis
                memset(&prand_bis[0], 0x00, GAP_KEY_LEN);

                /*
                 * Address provided from higher layers in LSB->MSB
                 *      -----------------------------------------------------------------------------------------
                 *      | hash[0:(GAP_ADDR_HASH_LEN-1)] | prand[(GAP_ADDR_HASH_LEN:(BD_ADDR_LEN-1)] |
                 *      -----------------------------------------------------------------------------------------
                 *
                 * prand_bis value sent to LL shall be LSB->MSB
                 *      --------------------------------------------------------------------------------------
                 *      | prand[0:(GAP_ADDR_PRAND_LEN-1)] | 0[(GAP_ADDR_PRAND_LEN):(GAP_KEY_LEN-1)]  |
                 *      --------------------------------------------------------------------------------------
                 */

                // Copy prand value in prand_bis
                memcpy(&(prand_bis[0]), &(cmd->addr.addr[GAP_ADDR_HASH_LEN]), GAP_ADDR_PRAND_LEN);

                // Ask for generation of a random number
                gapm_smp_encrypt_req((uint8_t *) & (cmd->irk[cmd->nb_key]), &prand_bis[0]);
            }
        }

        // if operation is finished, terminate it.
        if (finished)
        {
            // Send the command complete event message
            gapm_send_complete_evt(GAPM_OP_CFG, status);
        }
    }
}



/*
 * MESSAGES HANDLERS DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Handles request of solving a resolvable random address.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance (TASK_APP).
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
int gapm_resolv_addr_cmd_handler(ke_msg_id_t const msgid, struct gapm_resolv_addr_cmd *param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_RESOLV_ADDR, GAPM_NO_OP};
    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        // no need to check GAPM_RESOLV_ADDR operation, this has been done by gapm_process_op

        uint8_t status = GAP_ERR_NO_ERROR;

        // check if key are provided and if address is resolvable
        if ((param->nb_key == 0) || ((param->addr.addr[BD_ADDR_LEN - 1] & 0xC0) != GAP_RSLV_ADDR))
        {
            /* Invalid parameter */
            status = GAP_ERR_INVALID_PARAM;
        }

        // start the resolving operation using dedicated state machine
        gapm_smp_resolv_op_cont(status, NULL);

    }

    return msg_status;
}


/**
 ****************************************************************************************
 * @brief Handles request of generating a random address command
 * - GAPM_GEN_RAND_ADDR:  Generate a random address
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance.
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 *
 ****************************************************************************************
 */
int gapm_gen_rand_addr_cmd_handler(ke_msg_id_t const msgid, struct gapm_gen_rand_addr_cmd *param,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_GEN_RAND_ADDR, GAPM_NO_OP};

    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        // no need to check GAPM_GEN_RAND_ADDR operation, this has been done by gapm_process_op

        uint8_t status = GAP_ERR_NO_ERROR;

        // Check the operation code and the address type
        switch (param->rnd_type)
        {
        case (GAP_STATIC_ADDR):
        case (GAP_NON_RSLV_ADDR):
        case (GAP_RSLV_ADDR):
        {
            // Ask for generation of a random number
            gapm_smp_rand_req();
        }
        break;

        default:
        {
            status = GAP_ERR_INVALID_PARAM;
        }
        break;
        }

        if (status != GAP_ERR_NO_ERROR)
        {
            gapm_send_complete_evt(GAPM_OP_CFG, status);
        }
    }

    return msg_status;
}

/**
 ****************************************************************************************
 * @brief Command requested to SMP is completed.
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapm_use_enc_block_cmd_handler(ke_msg_id_t const msgid, struct gapm_use_enc_block_cmd *param,
                                   ke_task_id_t const dest_id, ke_task_id_t const src_id)
{

    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_USE_ENC_BLOCK, GAPM_GEN_RAND_NB, GAPM_NO_OP};
    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_CFG, param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        // no need to check GAPM_USE_ENC_BLOCK, GAPM_GEN_RAND_NB operation, this has been done by gapm_process_op

        switch (msgid)
        {
        case GAPM_USE_ENC_BLOCK_CMD:
        {
            // request aes usage
            gapm_smp_encrypt_req((uint8_t *) & (param->operand_1[0]), (uint8_t *) & (param->operand_2[0]));
        }
        break;
        case GAPM_GEN_RAND_NB_CMD:
        {
            // request aes usage
            gapm_smp_rand_req();
        }
        break;
        default: /* nothing to do */
            break;
        }
    }

    return msg_status;
}

#if (SECURE_CONNECTIONS)
/**
 ****************************************************************************************
 * @brief Handles request of generating a DH Key command
 * - GAPM_GEN_DH_KEY :  Generate a DH Key
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapm_gen_dh_key_cmd_handler(ke_msg_id_t const msgid, struct gapm_gen_dh_key_cmd *param,
                                ke_task_id_t const dest_id, ke_task_id_t const src_id)
{

    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_GEN_DH_KEY, GAPM_NO_OP};
    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_DHKEY, param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        struct hci_le_gen_dhkey_v1_cmd *cmd = KE_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_GEN_DHKEY_V1_CMD_OPCODE, hci_le_gen_dhkey_v1_cmd);

        memcpy((void *)&cmd->public_key[0], (void *)param->operand_1, 32);
        memcpy((void *)&cmd->public_key[32], (void *)param->operand_2, 32);

        hci_send_2_controller(cmd);

    }

    return msg_status;
}

/**
 ****************************************************************************************
 * @brief Handles request of generating a DH Key command
 * - GAPM_GET_PUB_KEY :  Generate a DH Key
 *
 * @param[in] msgid     Id of the message received.
 * @param[in] param     Pointer to the parameters of the message.
 * @param[in] dest_id   ID of the receiving task instance
 * @param[in] src_id    ID of the sending task instance.
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int gapm_get_pub_key_cmd_handler(ke_msg_id_t const msgid, struct gapm_get_pub_key_cmd *param,
                                 ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // list of handler supported operations
    enum gapm_operation supp_ops[] = {GAPM_GET_PUB_KEY, GAPM_NO_OP};
    // check if operation can be executed
    int msg_status = gapm_process_op(GAPM_OP_DHKEY, param, supp_ops);

    // Operation can be handled
    if (msg_status == KE_MSG_NO_FREE)
    {
        if (!param->renew)
        {
            // Public key value sent to the requester
            struct gapm_pub_key_ind *ind = KE_MSG_ALLOC(GAPM_PUB_KEY_IND, gapm_get_requester(GAPM_OP_DHKEY), dest_id,
                                           gapm_pub_key_ind);
            memcpy(&ind->pub_key_x, gapm_env.public_key.x,  32);
            memcpy(&ind->pub_key_y, gapm_env.public_key.y, 32);
            ke_msg_send(ind);

            gapm_send_complete_evt(GAPM_OP_DHKEY, GAP_ERR_NO_ERROR);
        }
        else
        {
            // Generate a new Public Key.
            hci_basic_cmd_send_2_controller(HCI_LE_RD_LOC_P256_PUB_KEY_CMD_OPCODE);
        }
    }

    return msg_status;
}
#endif // (SECURE_CONNECTIONS)


/*
 * HCI EVENT HANDLERS DEFINITIONS
 ****************************************************************************************
 */

#if !(BLE_EMB_PRESENT)


/**
 ****************************************************************************************
 * @brief Send encryption request over HCI
 ****************************************************************************************
 */
void gapm_smp_send_hci_encrypt(const uint8_t *key, const uint8_t *val)
{
    struct hci_le_enc_cmd *cmd = KE_MSG_ALLOC(HCI_COMMAND, 0, HCI_LE_ENC_CMD_OPCODE, hci_le_enc_cmd);

    memcpy(&cmd->key.ltk[0],    key, GAP_KEY_LEN);
    memcpy(&cmd->plain_data[0], val, GAP_KEY_LEN);

    hci_send_2_controller(cmd);
}

/**
 ****************************************************************************************
 * @brief HCI LE Encrypt command complete event handler.
 * The received encrypted data is to be sent to the saved SMPC source task ID (which is the
 * origin of the request).
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the message.
 * @return Received kernel message status (consumed or not).
 ****************************************************************************************
 */
int hci_le_enc_cmd_cmp_evt_handler(uint16_t opcode, struct hci_le_enc_cmd_cmp_evt *param)
{

    // inform AES result handler
    aes_result_handler(param->status, &param->encrypted_data[0]);

    return (KE_MSG_CONSUMED);
}
#endif // !(BLE_EMB_PRESENT)


#if (BLE_MESH || BLE_CENTRAL || BLE_PERIPHERAL)
#if (SECURE_CONNECTIONS)

/**
 ****************************************************************************************
 * @brief Handles the LE generate dh key complete event.
 *
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the event (32 byte DH Key)
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_generate_dhkey_cmp_evt_handler(uint16_t opcode, struct hci_le_gen_dhkey_cmp_evt const *param)
{
    // Check if a command is currently handled, drop the message if not the case.
    if (ke_state_get(TASK_GAPM) != GAPM_DEVICE_SETUP)
    {
        if (gapm_get_operation(GAPM_OP_DHKEY) == GAPM_GEN_DH_KEY)
        {
            uint8_t status = GAP_ERR_NO_ERROR;

            // Check the returned status
            if (param->status == GAP_ERR_NO_ERROR)
            {
                // Indication sent to the requester
                struct gapm_gen_dh_key_ind *ind = KE_MSG_ALLOC(GAPM_GEN_DH_KEY_IND,
                                                  gapm_get_requester(GAPM_OP_DHKEY), TASK_GAPM,
                                                  gapm_gen_dh_key_ind);


                memcpy(&ind->result[0], &param->dh_key[0], 32);

                // Send the indication
                ke_msg_send(ind);
            }
            else
            {
                status = RW_ERR_HCI_TO_HL(param->status);
            }

            // Send the command complete event message
            gapm_send_complete_evt(GAPM_OP_DHKEY, status);
        }
    }

    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the LE generate dh key command statyus event.
 *
 * @param[in] opcode    Operation code of received message
 * @param[in] param     Pointer to the parameters of the event
 *
 * @return If the message was consumed or not.
 ****************************************************************************************
 */
int hci_le_gen_dhkey_stat_evt_handler(uint16_t opcode, struct hci_basic_cmd_cmp_evt const *p_event)
{
    // Check if a command is currently handled, drop the message if not the case.
    if (ke_state_get(TASK_GAPM) != GAPM_DEVICE_SETUP)
    {
        if (gapm_get_operation(GAPM_OP_DHKEY) == GAPM_GEN_DH_KEY)
        {
            if (p_event->status != CO_ERROR_NO_ERROR)
            {
                // Send the command complete event message - DH Key generation fails
                gapm_send_complete_evt(GAPM_OP_DHKEY, p_event->status);
            }
        }
    }

    return (KE_MSG_CONSUMED);
}

#endif // (SECURE_CONNECTIONS)
#endif // (BLE_MESH || BLE_CENTRAL || BLE_PERIPHERAL)

void gapm_aes_h6_result_cb(uint8_t status, const uint8_t *aes_res, uint32_t metainfo)
{
    if (0 != status)
    {
        rt_kprintf(" aes_cmac h6 error %x\n", status);
        return;
    }
    rt_kprintf("gapm_aes_h6_result_cb\n");
    struct gapm_aes_h6_ind *ind = KE_MSG_ALLOC(GAPM_AES_H6_IND,
                                  g_gapm_h6h7_requester, TASK_GAPM, gapm_aes_h6_ind);
    memcpy(ind->aes_res, aes_res, GAP_KEY_LEN);
    ind->metainfo = metainfo;
    ke_msg_send(ind);
}
int gapm_aes_h6_cmd_handler(ke_msg_id_t const msgid,
                            struct gapm_aes_h6_cmd const *param,
                            ke_task_id_t const dest_id,
                            ke_task_id_t const src_id)
{
    rt_kprintf("gapm_aes_h6_cmd_handler\n");
    g_gapm_h6h7_requester = KE_TYPE_GET(src_id);
    aes_h6(param->w, param->key_id, gapm_aes_h6_result_cb, param->cb_request);
    return (KE_MSG_CONSUMED);
}
/// @} GAPM_SMP
