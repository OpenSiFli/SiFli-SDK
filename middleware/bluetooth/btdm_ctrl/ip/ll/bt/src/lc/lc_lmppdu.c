/**
 ****************************************************************************************
 *
 * @file lc_lmppdu.c
 *
 * @brief LM LMP PDU packing and unpacking utilities file.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LMLMPPDU
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "co_endian.h"         // endian definitions
#include "co_version.h"
#include "lc_int.h"
#include "lm.h"
#include "ld.h"
#include "reg_btcore.h"         // BT core registers
#include "dbg.h"

/*
 * FUNCTION DEFINITONS
 ****************************************************************************************
 */

void lc_send_lmp(uint8_t link_id, void *param)
{
    struct bt_em_lmp_buf_elt *buf_elt = bt_util_buf_lmp_tx_alloc();

    if (buf_elt != NULL)
    {
        const struct lmp_desc_tag *lmp_desc = NULL;
        uint8_t code = GETF(*((uint8_t *)param), LMP_OPCODE);
        uint16_t length = sizeof(union lmp_pdu);
        uint8_t status = CO_UTIL_PACK_ERROR;

        // Find LMP descriptor
        if (code == LMP_ESC4_OPCODE)
        {
            code = *(((uint8_t *)param) + 1);
            // Point to LMP descriptor
            lmp_desc = &lmp_ext_desc_tab[code];
        }
        else
        {
            // Point to LMP descriptor
            lmp_desc = &lmp_desc_tab[code];
        }

        if (lmp_desc->fmt != NULL)
        {
            // Pack the parameters
            status = co_util_pack(param, param, &length, length, lmp_desc->fmt);
        }

        // Check packing status
        if (status == CO_UTIL_PACK_OK)
        {
            em_wr(param, buf_elt->buf_ptr, length);
            buf_elt->length = length;
            ld_acl_lmp_tx(link_id, buf_elt);

            //Trace the LMP packet
            TRC_REQ_LMP_TX(BT_ACL_CONHDL_MIN + link_id, length, param);
        }
        else
        {
            ASSERT_INFO_FORCE(0, status, length);
        }
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}

void lc_send_pdu_acc(uint8_t idx, uint8_t opcode, uint8_t tr_id)
{
    struct lmp_accepted pdu;

    pdu.opcode = LMP_OPCODE(LMP_ACCEPTED_OPCODE, tr_id);
    pdu.orig_opcode  = opcode;

    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_acc_ext4(uint8_t idx, uint8_t opcode, uint8_t tr_id)
{
    struct lmp_accepted_ext pdu;

    pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, tr_id);
    pdu.ext_opcode = LMP_ACCEPTED_EXT_EXTOPCODE;
    pdu.orig_esc_opcode = LMP_ESC4_OPCODE;
    pdu.orig_ext_opcode = opcode;

    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_not_acc(uint8_t idx, uint8_t opcode, uint8_t reason, uint8_t tr_id)
{
    struct lmp_not_accepted pdu;

    pdu.opcode = LMP_OPCODE(LMP_NOT_ACCEPTED_OPCODE, tr_id);
    pdu.orig_opcode = opcode;
    pdu.reason  = reason;

    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_not_acc_ext4(uint8_t idx, uint8_t opcode, uint8_t reason, uint8_t tr_id)
{
    struct lmp_not_accepted_ext pdu;

    pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, tr_id);
    pdu.ext_opcode = LMP_NOT_ACCEPTED_EXT_EXTOPCODE;
    pdu.orig_esc_opcode = LMP_ESC4_OPCODE;
    pdu.orig_ext_opcode = opcode;
    pdu.reason     = reason;

    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_set_afh(uint8_t idx, uint32_t instant, uint8_t mode, uint8_t tr_id)
{
    struct lmp_set_afh pdu;

    pdu.opcode = LMP_OPCODE(LMP_SET_AFH_OPCODE, tr_id);
    pdu.instant = instant;
    pdu.mode    = mode;
    memcpy(&pdu.map.map[0], &lc_env[idx]->afh.ch_map.map[0], BT_CH_MAP_LEN);
    lc_send_lmp(idx, &pdu);

}

void lc_send_pdu_au_rand(uint8_t idx, struct ltk *random, uint8_t tr_id)
{
    struct lmp_aurand pdu;

    memcpy(&pdu.random.ltk[0], &random->ltk[0], KEY_LEN);
    pdu.opcode = LMP_OPCODE(LMP_AURAND_OPCODE, tr_id);
    lc_send_lmp(idx, &pdu);
}


void lc_send_pdu_in_rand(uint8_t idx, struct ltk *random, uint8_t tr_id)
{
    struct lmp_inrand pdu;

    memcpy(&pdu.random.ltk[0], &random->ltk[0], KEY_LEN);
    pdu.opcode = LMP_OPCODE(LMP_INRAND_OPCODE, tr_id);
    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_comb_key(uint8_t idx, struct ltk *random, uint8_t tr_id)
{
    struct lmp_combkey pdu;

    memcpy(&pdu.random.ltk[0], &random->ltk[0], KEY_LEN);
    pdu.opcode = LMP_OPCODE(LMP_COMBKEY_OPCODE, tr_id);
    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_max_slot(uint8_t idx, uint8_t max_slot, uint8_t tr_id)
{
    struct lmp_max_slot pdu;

    pdu.opcode = LMP_OPCODE(LMP_MAX_SLOT_OPCODE, tr_id);
    pdu.max_slots = max_slot;

    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_max_slot_req(uint8_t idx, uint8_t max_slot, uint8_t tr_id)
{
    struct lmp_max_slot_req pdu;

    pdu.opcode = LMP_OPCODE(LMP_MAX_SLOT_REQ_OPCODE, tr_id);
    pdu.max_slots = max_slot;
    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_encaps_payl(uint8_t idx, struct byte16 *data, uint8_t tr_id)
{
    struct lmp_encaps_payl pdu;

    memcpy(&pdu.data.A[0], &data->A[0], KEY_LEN);
    pdu.opcode = LMP_OPCODE(LMP_ENCAPS_PAYL_OPCODE, tr_id);

    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_encaps_head(uint8_t idx, uint8_t tr_id, uint8_t maj_type, uint8_t min_type, uint8_t payl_len)
{
    struct lmp_encaps_hdr pdu;

    pdu.maj_type = maj_type;
    pdu.min_type = min_type;
    pdu.payl_len = payl_len;
    pdu.opcode = LMP_OPCODE(LMP_ENCAPS_HDR_OPCODE, tr_id);

    lc_send_lmp(idx, &pdu);
}

#if PCA_SUPPORT
void lc_send_pdu_clk_adj_ack(uint8_t idx, uint8_t clk_adj_id, uint8_t tr_id)
{
    struct lmp_clk_adj_ack pdu;

    pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, tr_id);
    pdu.ext_opcode = LMP_CLK_ADJ_ACK_EXTOPCODE;

    pdu.clk_adj_id = clk_adj_id;

    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_clk_adj_req(uint8_t idx, uint16_t clk_adj_us, uint8_t clk_adj_slots, uint8_t clk_adj_period, uint8_t tr_id)
{
    struct lmp_clk_adj_req pdu;

    pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, tr_id);
    pdu.ext_opcode = LMP_CLK_ADJ_REQ_EXTOPCODE;

    pdu.clk_adj_us = clk_adj_us;
    pdu.clk_adj_slots = clk_adj_slots;
    pdu.clk_adj_period = clk_adj_period;

    lc_send_lmp(idx, &pdu);
}
#endif // PCA_SUPPORT

void lc_send_pdu_sam_define_map(uint8_t idx, uint8_t sam_index, uint8_t t_sam_sm, uint8_t n_sam_sm, const uint8_t *p_submaps, uint8_t tr_id)
{
    struct lmp_sam_define_map pdu;

    pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, tr_id);
    pdu.ext_opcode = LMP_SAM_DEFINE_MAP_EXTOPCODE;

    pdu.index = sam_index;
    pdu.t_sam_sm = t_sam_sm;
    pdu.n_sam_sm = n_sam_sm;
    memcpy(&pdu.submaps.map[0], p_submaps, SAM_SUBMAPS_LEN);

    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_sam_set_type0(uint8_t idx, uint8_t update_mode, const uint8_t *p_submap, uint8_t tr_id)
{
    struct lmp_sam_set_type0 pdu;

    pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, tr_id);
    pdu.ext_opcode = LMP_SAM_SET_TYPE0_EXTOPCODE;

    pdu.update_mode = update_mode;
    memcpy(&pdu.submap.map[0], p_submap, SAM_TYPE0_SUBMAP_LEN);

    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_sam_switch(uint8_t idx, uint8_t sam_index, uint8_t flags, uint8_t d_sam, uint32_t instant, uint8_t tr_id)
{
    struct lmp_sam_switch pdu;

    pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, tr_id);
    pdu.ext_opcode = LMP_SAM_SWITCH_EXTOPCODE;

    pdu.index = sam_index;
    pdu.flags = flags;
    pdu.d_sam = d_sam;
    pdu.instant = instant;

    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_ptt_req(uint8_t idx, uint8_t ptt, uint8_t tr_id)
{
    struct lmp_pkt_type_tbl_req pdu;

    pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, tr_id);
    pdu.ext_opcode = LMP_PKT_TYPE_TBL_REQ_EXTOPCODE;
    pdu.pkt_type_tbl = ptt;

    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_sp_nb(uint8_t idx, struct byte16 *data, uint8_t tr_id)
{
    struct lmp_sp_nb pdu;

    memcpy(&pdu.nonce.A[0], &data->A[0], 16);
    pdu.opcode = LMP_OPCODE(LMP_SP_NB_OPCODE, tr_id);
    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_sp_cfm(uint8_t idx, struct byte16 *data, uint8_t tr_id)
{
    struct lmp_sp_cfm pdu;

    memcpy(&pdu.commitment_val.A[0], &data->A[0], 16);
    pdu.opcode = LMP_OPCODE(LMP_SP_CFM_OPCODE, tr_id);
    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_sres(uint8_t idx, struct sres_nb *data, uint8_t tr_id)
{
    struct lmp_sres pdu;

    memcpy(&pdu.Sres.nb[0], &data->nb[0], 4);
    pdu.opcode = LMP_OPCODE(LMP_SRES_OPCODE, tr_id);

    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_sco_lk_rem_req(uint8_t idx, uint8_t handle, uint8_t reason, uint8_t tr_id)
{
    struct lmp_rmv_sco_link_req pdu;

    pdu.opcode = LMP_OPCODE(LMP_RMV_SCO_LINK_REQ_OPCODE, tr_id);
    pdu.reason = reason;
    pdu.sco_hdl = handle;
    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_esco_lk_rem_req(uint8_t idx, uint8_t handle, uint8_t reason, uint8_t tr_id)
{
    struct lmp_rmv_esco_link_req pdu;

    pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, tr_id);
    pdu.ext_opcode = LMP_RMV_ESCO_LINK_REQ_EXTOPCODE;
    pdu.reason   = reason;
    pdu.esco_hdl = handle;

    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_auto_rate(uint8_t idx, uint8_t tr_id)
{
    struct lmp_auto_rate pdu;

    pdu.opcode = LMP_OPCODE(LMP_AUTO_RATE_OPCODE, tr_id);

    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_lsto(uint8_t idx, uint16_t timeout, uint8_t role)
{
    struct lmp_supv_to pdu;

    pdu.opcode = LMP_OPCODE(LMP_SUPV_TO_OPCODE, role);
    pdu.supv_to  = timeout;
    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_enc_key_sz_req(uint8_t idx, uint8_t key_size, uint8_t tr_id)
{
    struct lmp_enc_key_size_req pdu;

    pdu.key_size = key_size;
    pdu.opcode = LMP_OPCODE(LMP_ENC_KEY_SIZE_REQ_OPCODE, tr_id);
    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_feats_res(uint8_t idx, uint8_t tr_id)
{
    struct lmp_feats_res pdu;

    lm_read_features(FEATURE_PAGE_0, NULL, &pdu.feats);
    pdu.opcode = LMP_OPCODE(LMP_FEATS_RES_OPCODE, tr_id);
    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_tim_acc(uint8_t idx, uint8_t tr_id)
{
    // send LMP_TimingAccuracyReq(idx, role)
    struct lmp_timing_accu_req pdu;

    pdu.opcode = LMP_OPCODE(LMP_TIMING_ACCU_REQ_OPCODE, tr_id);
    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_feats_ext_req(uint8_t idx, uint8_t page, uint8_t tr_id)
{
    struct lmp_feats_req_ext pdu;

    lm_read_features(page, &pdu.max_page, &pdu.ext_feats);
    pdu.page     = page;
    pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, tr_id);
    pdu.ext_opcode = LMP_FEATS_REQ_EXT_EXTOPCODE;
    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_io_cap_res(uint8_t idx)
{
    struct lmp_io_cap_res pdu;
    struct lc_env_tag *lc_env_ptr = lc_env[idx];

    pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, lc_env_ptr->sp.SpTId);
    pdu.ext_opcode = LMP_IO_CAP_RES_EXTOPCODE;
    pdu.auth_req = lc_env_ptr->sp.IOCap_loc.aut_req ;
    pdu.io_cap   = lc_env_ptr->sp.IOCap_loc.io_cap;
    pdu.oob_auth_data = lc_env_ptr->sp.IOCap_loc.oob_data_present;

    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_num_comp_fail(uint8_t idx, uint8_t tr_id)
{
    struct lmp_num_comparison_fail pdu;

    pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, tr_id);
    pdu.ext_opcode = LMP_NUM_COMPARISON_FAIL_EXTOPCODE;

    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_pause_enc_aes_req(uint8_t idx, uint8_t tr_id, struct byte16 *random)
{
    struct lmp_pause_enc_aes_req pdu;

    pdu.opcode = LMP_OPCODE(LMP_PAUSE_ENC_AES_REQ_OPCODE, tr_id);
    memcpy(&pdu.rand.ltk[0],  &random->A[0], KEY_LEN);

    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_paus_enc_req(uint8_t idx, uint8_t tr_id)
{
    struct lmp_pause_enc_req pdu;

    pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, tr_id);
    pdu.ext_opcode = LMP_PAUSE_ENC_REQ_EXTOPCODE;

    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_resu_enc_req(uint8_t idx, uint8_t tr_id)
{
    struct lmp_resume_enc_req pdu;

    pdu.opcode = LMP_OPCODE(LMP_ESC4_OPCODE, tr_id);
    pdu.ext_opcode = LMP_RESUME_ENC_REQ_EXTOPCODE;

    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_setup_cmp(uint8_t idx, uint8_t tr_id)
{
    struct lmp_setup_cmp pdu;
    pdu.opcode = LMP_OPCODE(LMP_SETUP_CMP_OPCODE, tr_id);
    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_qos_req(uint8_t idx, uint8_t nb_bcst, uint16_t poll_int, uint8_t tr_id)
{
    struct lmp_qos_req pdu;

    pdu.opcode = LMP_OPCODE(LMP_QOS_REQ_OPCODE, tr_id);
    pdu.poll_intv = poll_int;
    pdu.nbc       = nb_bcst;
    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_slot_off(uint8_t idx, uint16_t slot_off, uint8_t tr_id)
{
    struct lmp_slot_off pdu;
    struct lc_env_tag *lc_env_ptr = lc_env[idx];
    memcpy(&pdu.addr.addr[0], &lc_env_ptr->info.LocalBdAddr.addr[0], BD_ADDR_LEN);

    pdu.opcode = LMP_OPCODE(LMP_SLOT_OFF_OPCODE, tr_id);
    pdu.slot_off = slot_off;

    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_vers_req(uint8_t idx, uint8_t role)
{
    struct lmp_ver_req pdu;

    pdu.opcode = LMP_OPCODE(LMP_VER_REQ_OPCODE, role);
    pdu.subver  = co_htobs(CO_SUBVERSION_BUILD(RWBT_SW_VERSION_MINOR, RWBT_SW_VERSION_BUILD));
    pdu.ver     = RWBT_SW_VERSION_MAJOR;
    pdu.co_id   = co_htobs(RW_COMP_ID);

    lc_send_lmp(idx, &pdu);
}

void lc_send_pdu_dhkey_chk(uint8_t idx, struct ltk *dhkey, uint8_t tr_id)
{
    struct lmp_dhkey_chk pdu;

    pdu.opcode = LMP_OPCODE(LMP_DHKEY_CHK_OPCODE, tr_id);
    memcpy(&pdu.cfm_val.ltk[0], &dhkey->ltk[0], KEY_LEN);

    lc_send_lmp(idx, &pdu);
}

///@} LMLMPPDU
