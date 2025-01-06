/**
 ****************************************************************************************
 *
 * @file lc_lmppdu.h
 *
 * @brief LM LMP PDU packing and unpacking utilities header file.
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */
#ifndef LM_LMPPDU_H_
#define LM_LMPPDU_H_

/**
 ****************************************************************************************
 * @defgroup LMLMPPDU LMP PDUs
 * @ingroup LM
 * @brief LM module for LMP PDU packing and unpacking.
 * @{
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

/*
 * FUNCTION DECLARATIONS
 * ***************************************************************************************
 */
/**
******************************************************************************************
* @brief Function called to pack and send an LMP PDU.
*
* @param param  Pointer to the param part of the ke_msg used to create the PDU structure
*               and packet.
*
* @return Length of PDU.
******************************************************************************************
*/
void lc_send_lmp(uint8_t link_id, void *param);

//Functions dedicated to send a LMP
void lc_send_pdu_acc(uint8_t lid, uint8_t opcode, uint8_t tr_id);
void lc_send_pdu_acc_ext4(uint8_t lid, uint8_t opcode, uint8_t tr_id);
void lc_send_pdu_not_acc(uint8_t lid, uint8_t opcode, uint8_t reason, uint8_t tr_id);
void lc_send_pdu_not_acc_ext4(uint8_t lid, uint8_t opcode, uint8_t reason, uint8_t tr_id);
void lc_send_pdu_au_rand(uint8_t lid, struct ltk *random, uint8_t tr_id);
void lc_send_pdu_in_rand(uint8_t lid, struct ltk *random, uint8_t tr_id);
void lc_send_pdu_comb_key(uint8_t lid, struct ltk *random, uint8_t tr_id);
void lc_send_pdu_max_slot(uint8_t lid, uint8_t max_slot, uint8_t tr_id);
void lc_send_pdu_max_slot_req(uint8_t lid, uint8_t max_slot, uint8_t tr_id);
void lc_send_pdu_encaps_payl(uint8_t lid, struct byte16 *data, uint8_t tr_id);
void lc_send_pdu_encaps_head(uint8_t idx, uint8_t tr_id, uint8_t maj_type, uint8_t min_type, uint8_t payl_len);
void lc_send_pdu_clk_adj_ack(uint8_t idx, uint8_t clk_adj_id, uint8_t tr_id);
void lc_send_pdu_clk_adj_req(uint8_t idx, uint16_t clk_adj_us, uint8_t clk_adj_slots, uint8_t clk_adj_period, uint8_t tr_id);
void lc_send_pdu_sam_define_map(uint8_t idx, uint8_t sam_index, uint8_t t_sam_sm, uint8_t n_sam_sm, const uint8_t *p_submaps, uint8_t tr_id);
void lc_send_pdu_sam_set_type0(uint8_t idx, uint8_t update_mode, const uint8_t *p_submap, uint8_t tr_id);
void lc_send_pdu_sam_switch(uint8_t idx, uint8_t sam_index, uint8_t flags, uint8_t d_sam, uint32_t instant, uint8_t tr_id);
void lc_send_pdu_ptt_req(uint8_t lid, uint8_t ptt, uint8_t tr_id);
void lc_send_pdu_sp_nb(uint8_t lid, struct byte16 *data, uint8_t tr_id);
void lc_send_pdu_sp_cfm(uint8_t lid, struct byte16 *data, uint8_t tr_id);
void lc_send_pdu_sres(uint8_t lid, struct sres_nb *data, uint8_t tr_id);
void lc_send_pdu_sco_lk_req(uint8_t lid);
void lc_send_pdu_sco_lk_rem_req(uint8_t lid, uint8_t handle, uint8_t reason, uint8_t tr_id);
void lc_send_pdu_esco_lk_req(uint8_t lid);
void lc_send_pdu_esco_lk_rem_req(uint8_t lid, uint8_t handle, uint8_t reason, uint8_t tr_id);
void lc_send_pdu_auto_rate(uint8_t lid, uint8_t tr_id);
void lc_send_pdu_lsto(uint8_t lid, uint16_t timeout, uint8_t role);
void lc_send_pdu_enc_key_sz_req(uint8_t lid, uint8_t key_size, uint8_t tr_id);
void lc_send_pdu_feats_ext_req(uint8_t lid, uint8_t page, uint8_t tr_id);
void lc_send_pdu_io_cap_res(uint8_t lid);
void lc_send_pdu_num_comp_fail(uint8_t lid, uint8_t tr_id);
void lc_send_pdu_pause_enc_aes_req(uint8_t idx, uint8_t tr_id, struct byte16 *random);
void lc_send_pdu_paus_enc_req(uint8_t lid, uint8_t tr_id);
void lc_send_pdu_resu_enc_req(uint8_t lid, uint8_t tr_id);
void lc_send_pdu_setup_cmp(uint8_t lid, uint8_t tr_id);
void lc_send_pdu_qos_req(uint8_t lid, uint8_t nb_bcst, uint16_t poll_int, uint8_t tr_id);
void lc_send_pdu_slot_off(uint8_t lid, uint16_t slot_off, uint8_t tr_id);
void lc_send_pdu_vers_req(uint8_t lid, uint8_t role);
void lc_send_pdu_dhkey_chk(uint8_t lid, struct ltk *dhkey, uint8_t tr_id);
void lc_send_pdu_feats_res(uint8_t lid, uint8_t tr_id);
void lc_send_pdu_set_afh(uint8_t lid, uint32_t instant, uint8_t mode, uint8_t tr_id);
void lc_send_pdu_tim_acc(uint8_t idx, uint8_t tr_id);
/// @} LMLMPPDU
#endif //LM_LMPPDU_H_
