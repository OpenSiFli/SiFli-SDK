/**
  ******************************************************************************
  * @file   patch.h
  * @author Sifli software development team
  * @brief  patch entry definition
  *
  * @{
  ******************************************************************************
*/
/**
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdint.h>

#ifndef BTDM_PATCH_H_
#define BTDM_PATCH_H_

// name with file name
enum patch_type
{
    LLD_PATCH_TYPE,
    LLD_PATCH_2_TYPE,
    LLD_ADV_PATCH_TYPE,
    LLD_BI_1_PATCH_TYPE,
    LLD_BI_2_PATCH_TYPE,
    LLD_CI_1_PATCH_TYPE,
    LLD_CI_2_PATCH_TYPE,
    LLD_CON_1_PATCH_TYPE,
    LLD_CON_2_PATCH_TYPE,
    LLD_CON_3_PATCH_TYPE,
    LLD_INIT_PATCH_TYPE,
    LLD_ISO_PATCH_TYPE,
    LLD_ISOAL_PATCH_TYPE,
    LLD_PER_ADV_PATCH_TYPE,
    LLD_SCAN_PATCH_TYPE,
    LLD_SYNC_PATCH_TYPE,
    LLD_TEST_PATCH_TYPE,
    LD_PATCH_TYPE,
    LD_ACL_1_PATCH_TYPE,
    LD_ACL_2_PATCH_TYPE,
    LD_ACL_3_PATCH_TYPE,
    LD_ACL_4_PATCH_TYPE,
    LD_BCST_PATCH_TYPE,
    LD_INQ_PATCH_TYPE,
    LD_ISCAN_PATCH_TYPE,
    LD_PAGE_PATCH_TYPE,
    LD_PCA_PATCH_TYPE,
    LD_PSCAN_PATCH_TYPE,
    LD_SSCAN_PATCH_TYPE,
    LD_STRAIN_PATCH_TYPE,
    LD_TEST_PATCH_TYPE,
    LD_UTIL_PATCH_TYPE,
    SCH_ALARM_PATCH_TYPE,
    SCH_ARB_PATCH_TYPE,
    SCH_PLAN_PATCH_TYPE,
    SCH_PROG_PATCH_TYPE,
    SCH_SLICE_PATCH_TYPE,
    RWIP_PATCH_TYPE,
    RWIP_DRIVER_1_PATCH_TYPE,
    RWIP_DRIVER_2_PATCH_TYPE,
    KE_LP_PATCH_TYPE,
    LC_TASK_PATCH_TYPE,
    LLI_BI_PATCH_TYPE,
    LLI_CI_PATCH_TYPE,
    LLC_CIS_PATCH_TYPE,
    RWBLE_PATCH_TYPE,
    RWBT_PATCH_TYPE,
    LLD_CH_SCAN_PATCH_TYPE,
    LLI_PATCH_TYPE,
    RF_PATCH_TYPE,
    LM_1_PATCH_TYPE,
    LM_2_PATCH_TYPE,
    PROC_CONTINUE_PATCH_TYPE,
    DATA_PATH_PATCH_TYPE,
    PATCH_TYPE_MAX
};

enum ld_patch_fields
{
    LD_CORE_INIT_POS                = 0,
    LD_CORE_INIT_BIT                = (1 << 0),
    LD_CORE_RESET_POS               = 1,
    LD_CORE_RESET_BIT               = (1 << 1),
    LD_RXDESC_FREE_POS              = 2,
    LD_RXDESC_FREE_BIT              = (1 << 2),
    LD_RXDESC_CHECK_POS             = 3,
    LD_RXDESC_CHECK_BIT             = (1 << 3),
    LD_ACTIVE_MODE_SET_POS          = 4,
    LD_ACTIVE_MODE_SET_BIT          = (1 << 4),
    LD_ACTIVE_LINK_CHECK_POS        = 5,
    LD_ACTIVE_LINK_CHECK_BIT        = (1 << 5),
    LD_INIT_POS                     = 6,
    LD_INIT_BIT                     = (1 << 6),
    LD_RESET_POS                    = 7,
    LD_RESET_BIT                    = (1 << 7),
    LD_READ_CLOCK_POS               = 8,
    LD_READ_CLOCK_BIT               = (1 << 8),
    LD_BD_ADDR_GET_POS              = 9,
    LD_BD_ADDR_GET_BIT              = (1 << 9),
    LD_CLASS_OF_DEV_GET_POS         = 10,
    LD_CLASS_OF_DEV_GET_BIT         = (1 << 10),
    LD_CLASS_OF_DEV_SET_POS         = 11,
    LD_CLASS_OF_DEV_SET_BIT         = (1 << 11),
    LD_RXDESC_BUF_READY_POS         = 12,
    LD_RXDESC_BUF_READY_BIT         = (1 << 12),
};

enum ld_acl_1_patch_fields
{
    LD_ACL_ACTIVE_HOP_TYPES_GET_BIT               = (1 << 0),
    LD_ACL_AFH_APPLY_BIT                          = (1 << 1),
    LD_ACL_AFH_CONFIRM_BIT                        = (1 << 2),
    LD_ACL_AFH_PREPARE_BIT                        = (1 << 3),
    LD_ACL_AFH_SET_BIT                            = (1 << 4),
    LD_ACL_AFH_SWITCH_OFF_CBK_BIT                 = (1 << 5),
    LD_ACL_AFH_SWITCH_ON_CBK_BIT                  = (1 << 6),
    LD_ACL_ALLOWED_TX_PACKET_TYPES_SET_BIT        = (1 << 7),
    LD_ACL_AUTOMATIC_DATA_FLUSH_BIT               = (1 << 8),
    LD_ACL_BCST_RX_DEC_BIT                        = (1 << 9),
    LD_ACL_CLK_ADJ_INSTANT_CBK_BIT                = (1 << 10),
    LD_ACL_CLK_ADJ_SET_BIT                        = (1 << 11),
    LD_ACL_CLK_ISR_BIT                            = (1 << 12),
    LD_ACL_CLK_SET_BIT                            = (1 << 13),
    LD_ACL_CLOCK_OFFSET_GET_BIT                   = (1 << 14),
    LD_ACL_CURRENT_TX_POWER_GET_BIT               = (1 << 15),
    LD_ACL_DATA_FLUSH_BIT                         = (1 << 16),
    LD_ACL_DATA_TX_BIT                            = (1 << 17),
    LD_ACL_DAY_COUNT_UPDATE_BIT                   = (1 << 18),
    LD_ACL_DM1_PACKET_TYPE_DIS_BIT                = (1 << 19),
    LD_ACL_EDR_SET_BIT                            = (1 << 20),
    LD_ACL_ENC_KEY_LOAD_BIT                       = (1 << 21),
    LD_ACL_END_BIT                                = (1 << 22),
    LD_ACL_EVT_CANCELED_CBK_BIT                   = (1 << 23),
    LD_ACL_EVT_START_CBK_BIT                      = (1 << 24),
    LD_ACL_EVT_STOP_CBK_BIT                       = (1 << 25),
    LD_ACL_FLOW_OFF_BIT                           = (1 << 26),
    LD_ACL_FLOW_ON_BIT                            = (1 << 27),
    LD_ACL_FLUSH_TIMEOUT_GET_BIT                  = (1 << 28),
    LD_ACL_FLUSH_TIMEOUT_SET_BIT                  = (1 << 29),
    LD_ACL_FRM_CBK_BIT                            = (1 << 30),
    LD_ACL_FRM_ISR_BIT                            = (1 << 31),
};

enum ld_acl_2_patch_fields
{
    LD_ACL_INIT_2_FUNC_BIT                               = (1 << 0),
    LD_ACL_LMP_TX_2_FUNC_BIT                             = (1 << 1),
    LD_ACL_LOCAL_SAM_INDEX_SET_2_FUNC_BIT                = (1 << 2),
    LD_ACL_LSTO_GET_2_FUNC_BIT                           = (1 << 3),
    LD_ACL_LSTO_SET_2_FUNC_BIT                           = (1 << 4),
    LD_ACL_PHASE_ALIGN_2_FUNC_BIT                        = (1 << 5),
    LD_ACL_REMOTE_SAM_MAP_REFRESH_2_FUNC_BIT             = (1 << 6),
    LD_ACL_REMOTE_SAM_MAP_SET_2_FUNC_BIT                 = (1 << 7),
    LD_ACL_RESCHED_2_FUNC_BIT                            = (1 << 8),
    LD_ACL_RESET_2_FUNC_BIT                              = (1 << 9),
    LD_ACL_ROLE_GET_2_FUNC_BIT                           = (1 << 10),
    LD_ACL_RSSI_DELTA_GET_2_FUNC_BIT                     = (1 << 11),
    LD_ACL_RSW_END_2_FUNC_BIT                            = (1 << 12),
    LD_ACL_RSW_EVT_CANCELED_CBK_2_FUNC_BIT               = (1 << 13),
    LD_ACL_RSW_EVT_START_CBK_2_FUNC_BIT                  = (1 << 14),
    LD_ACL_RSW_FHS_RX_2_FUNC_BIT                         = (1 << 15),
    LD_ACL_RSW_FRM_CBK_2_FUNC_BIT                        = (1 << 16),
    LD_ACL_RSW_FRM_ISR_2_FUNC_BIT                        = (1 << 17),
    LD_ACL_RSW_REQ_2_FUNC_BIT                            = (1 << 18),
    LD_ACL_RSW_SLOT_OFFSET_GET_2_FUNC_BIT                = (1 << 19),
    LD_ACL_RSW_SLOT_OFFSET_SET_2_FUNC_BIT                = (1 << 20),
    LD_ACL_RSW_START_2_FUNC_BIT                          = (1 << 21),
    LD_ACL_RX_2_FUNC_BIT                                 = (1 << 22),
    LD_ACL_RX_ENC_2_FUNC_BIT                             = (1 << 23),
    LD_ACL_RX_ISR_2_FUNC_BIT                             = (1 << 24),
    LD_ACL_RX_MAX_SLOT_GET_2_FUNC_BIT                    = (1 << 25),
    LD_ACL_RX_MAX_SLOT_SET_2_FUNC_BIT                    = (1 << 26),
    LD_ACL_RX_NO_SYNC_2_FUNC_BIT                         = (1 << 27),
    LD_ACL_RX_SYNC_2_FUNC_BIT                            = (1 << 28),
    LD_ACL_SAM_CONFIG_2_FUNC_BIT                         = (1 << 29),
    LD_ACL_SAM_OFFSET_SET_2_FUNC_BIT                     = (1 << 30),
    LD_ACL_SAM_TS_GET_2_FUNC_BIT                         = (1 << 31),

};

enum ld_acl_3_patch_fields
{
    LD_ACL_SAM_UPDATE_PENDING_3_FUNC_BIT                 = (1 << 0),
    LD_ACL_SCHED_3_FUNC_BIT                              = (1 << 1),
    LD_ACL_SCO_CHECK_3_FUNC_BIT                          = (1 << 2),
    LD_ACL_SKET_ISR_3_FUNC_BIT                           = (1 << 3),
    LD_ACL_SNIFF_3_FUNC_BIT                              = (1 << 4),
    LD_ACL_SNIFF_ENTER_3_FUNC_BIT                        = (1 << 5),
    LD_ACL_SNIFF_EVT_CANCELED_CBK_3_FUNC_BIT             = (1 << 6),
    LD_ACL_SNIFF_EVT_START_CBK_3_FUNC_BIT                = (1 << 7),
    LD_ACL_SNIFF_EXIT_3_FUNC_BIT                         = (1 << 8),
    LD_ACL_SNIFF_FIRST_ANCHOR_COMPUTE_3_FUNC_BIT         = (1 << 9),
    LD_ACL_SNIFF_FRM_CBK_3_FUNC_BIT                      = (1 << 10),
    LD_ACL_SNIFF_FRM_ISR_3_FUNC_BIT                      = (1 << 11),
    LD_ACL_SNIFF_SCHED_3_FUNC_BIT                        = (1 << 12),
    LD_ACL_SNIFF_TRANS_3_FUNC_BIT                        = (1 << 13),
    LD_ACL_SNIFF_TRANS_SCHED_3_FUNC_BIT                  = (1 << 14),
    LD_ACL_SSR_SET_3_FUNC_BIT                            = (1 << 15),
    LD_ACL_START_3_FUNC_BIT                              = (1 << 16),
    LD_ACL_STOP_3_FUNC_BIT                               = (1 << 17),
    LD_ACL_TEST_MODE_SET_3_FUNC_BIT                      = (1 << 18),
    LD_ACL_TEST_MODE_UPDATE_3_FUNC_BIT                   = (1 << 19),
    LD_ACL_TIMING_ACCURACY_SET_3_FUNC_BIT                = (1 << 20),
    LD_ACL_TX_3_FUNC_BIT                                 = (1 << 21),
    LD_ACL_TX_ENC_3_FUNC_BIT                             = (1 << 22),
    LD_ACL_TX_PACKET_TYPE_SELECT_3_FUNC_BIT              = (1 << 23),
    LD_ACL_TX_PROG_3_FUNC_BIT                            = (1 << 24),
    LD_ACL_T_POLL_GET_3_FUNC_BIT                         = (1 << 25),
    LD_ACL_T_POLL_SET_3_FUNC_BIT                         = (1 << 26),
    LD_ACL_UNSNIFF_3_FUNC_BIT                            = (1 << 27),
    LD_ESCO_LOOPBACK_MODE_EN_3_FUNC_BIT                  = (1 << 28),
    LD_SCO_AUDIO_ISR_3_FUNC_BIT                          = (1 << 29),
    LD_SCO_DATA_TX_3_FUNC_BIT                            = (1 << 30),
    LD_SCO_END_3_FUNC_BIT                                = (1 << 31),

};

enum ld_acl_4_patch_fields
{
    LD_SCO_EVT_CANCELED_CBK_4_FUNC_BIT                   = (1 << 0),
    LD_SCO_EVT_START_CBK_4_FUNC_BIT                      = (1 << 1),
    LD_SCO_FRM_CBK_4_FUNC_BIT                            = (1 << 2),
    LD_SCO_FRM_ISR_4_FUNC_BIT                            = (1 << 3),
    LD_SCO_MODIFY_4_FUNC_BIT                             = (1 << 4),
    LD_SCO_PCM_INIT_4_FUNC_BIT                           = (1 << 5),
    LD_SCO_RESCHED_CBK_4_FUNC_BIT                        = (1 << 6),
    LD_SCO_SCHED_4_FUNC_BIT                              = (1 << 7),
    LD_SCO_SKET_ISR_4_FUNC_BIT                           = (1 << 8),
    LD_SCO_START_4_FUNC_BIT                              = (1 << 9),
    LD_SCO_STOP_4_FUNC_BIT                               = (1 << 10),
    LD_SCO_UPDATE_4_FUNC_BIT                             = (1 << 11),
    LD_ACL_LMP_TXDV_4_FUNC_BIT                           = (1 << 12),
};

enum ld_bcst_patch_fields
{
    LD_BCST_ACL_DATA_TX_BIT                    = (1 << 0),
    LD_BCST_ACL_INIT_BIT                       = (1 << 1),
    LD_BCST_ACL_RESET_BIT                      = (1 << 2),
    LD_BCST_AFH_UPDATE_BIT                     = (1 << 3),
    LD_BCST_EM_INIT_BIT                        = (1 << 4),
    LD_BCST_ENC_KEY_LOAD_BIT                   = (1 << 5),
    LD_BCST_EVT_CANCELED_CBK_BIT               = (1 << 6),
    LD_BCST_EVT_START_CBK_BIT                  = (1 << 7),
    LD_BCST_FRM_CBK_BIT                        = (1 << 8),
    LD_BCST_FRM_ISR_BIT                        = (1 << 9),
    LD_BCST_LMP_CANCEL_BIT                     = (1 << 10),
    LD_BCST_LMP_TX_BIT                         = (1 << 11),
    LD_BCST_PROCESS_PKT_TX_BIT                 = (1 << 12),
    LD_BCST_RESCHED_BIT                        = (1 << 13),
    LD_BCST_SCHED_BIT                          = (1 << 14),
    LD_BCST_SKET_ISR_BIT                       = (1 << 15),
    LD_BCST_START_BIT                          = (1 << 16),
    LD_BCST_TX_ENC_BIT                         = (1 << 17),
};

enum ld_inq_patch_fields
{
    LD_INQ_END_BIT                            = (1 << 0),
    LD_INQ_EVT_CANCELED_CBK_BIT               = (1 << 1),
    LD_INQ_EVT_START_CBK_BIT                  = (1 << 2),
    LD_INQ_FRM_CBK_BIT                        = (1 << 3),
    LD_INQ_FRM_ISR_BIT                        = (1 << 4),
    LD_INQ_INIT_BIT                           = (1 << 5),
    LD_INQ_PROG_BIT                           = (1 << 6),
    LD_INQ_RESET_BIT                          = (1 << 7),
    LD_INQ_RX_BIT                             = (1 << 8),
    LD_INQ_SCHED_BIT                          = (1 << 9),
    LD_INQ_START_BIT                          = (1 << 10),
    LD_INQ_STOP_BIT                           = (1 << 11),
};

enum ld_iscan_patch_fields
{
    LD_ISCAN_CLEANUP_BIT                                  = (1 << 0),
    LD_ISCAN_EIR_GET_BIT                                  = (1 << 1),
    LD_ISCAN_EIR_SET_BIT                                  = (1 << 2),
    LD_ISCAN_EVT_CANCELED_CBK_BIT                         = (1 << 3),
    LD_ISCAN_EVT_START_CBK_BIT                            = (1 << 4),
    LD_ISCAN_FRM_CBK_BIT                                  = (1 << 5),
    LD_ISCAN_FRM_ISR_BIT                                  = (1 << 6),
    LD_ISCAN_INIT_BIT                                     = (1 << 7),
    LD_ISCAN_MWSCOEX_XI_GET_BIT                           = (1 << 8),
    LD_ISCAN_MWSCOEX_XI_MASK_BUILD_BIT                    = (1 << 9),
    LD_ISCAN_PROG_BIT                                     = (1 << 10),
    LD_ISCAN_RESET_BIT                                    = (1 << 11),
    LD_ISCAN_RESTART_BIT                                  = (1 << 12),
    LD_ISCAN_START_BIT                                    = (1 << 13),
    LD_ISCAN_STOP_BIT                                     = (1 << 14),
};

enum ld_page_patch_fields
{
    LD_PAGE_1ST_PKT_RX_BIT                            = (1 << 0),
    LD_PAGE_EM_INIT_BIT                               = (1 << 1),
    LD_PAGE_END_BIT                                   = (1 << 2),
    LD_PAGE_EVT_CANCELED_CBK_BIT                      = (1 << 3),
    LD_PAGE_EVT_START_CBK_BIT                         = (1 << 4),
    LD_PAGE_FRM_CBK_BIT                               = (1 << 5),
    LD_PAGE_FRM_ISR_BIT                               = (1 << 6),
    LD_PAGE_INIT_BIT                                  = (1 << 7),
    LD_PAGE_PROG_BIT                                  = (1 << 8),
    LD_PAGE_RESET_BIT                                 = (1 << 9),
    LD_PAGE_START_BIT                                 = (1 << 10),
    LD_PAGE_STOP_BIT                                  = (1 << 11),
};

enum ld_pca_patch_fields
{
    LD_PCA_CLK_DRAG_STOP_BIT                          = (1 << 0),
    LD_PCA_CLK_RECOVER_ISR_BIT                        = (1 << 1),
    LD_PCA_COARSE_CLOCK_ADJUST_BIT                    = (1 << 2),
    LD_PCA_END_IND_BIT                                = (1 << 3),
    LD_PCA_EVT_CANCELED_CBK_BIT                       = (1 << 4),
    LD_PCA_EVT_START_CBK_BIT                          = (1 << 5),
    LD_PCA_EXT_FRAME_TS_GET_BIT                       = (1 << 6),
    LD_PCA_INIT_BIT                                   = (1 << 7),
    LD_PCA_INITIATE_CLOCK_DRAGGING_BIT                = (1 << 8),
    LD_PCA_INSTANT_CBK_BIT                            = (1 << 9),
    LD_PCA_LOCAL_CONFIG_BIT                           = (1 << 10),
    LD_PCA_MWS_FRAME_SYNC_BIT                         = (1 << 11),
    LD_PCA_MWS_MOMENT_OFFSET_GT_BIT                   = (1 << 12),
    LD_PCA_MWS_MOMENT_OFFSET_LT_BIT                   = (1 << 13),
    LD_PCA_REPORTING_ENABLE_BIT                       = (1 << 14),
    LD_PCA_RESET_BIT                                  = (1 << 15),
    LD_PCA_UPDATE_TARGET_OFFSET_BIT                   = (1 << 16),
};

enum ld_pscan_path_fields
{
    LD_PSCAN_1ST_PKT_RX_BIT                           = (1 << 0),
    LD_PSCAN_EM_INIT_BIT                              = (1 << 1),
    LD_PSCAN_END_BIT                                  = (1 << 2),
    LD_PSCAN_EVT_CANCELED_CBK_BIT                     = (1 << 3),
    LD_PSCAN_EVT_HANDLER_BIT                          = (1 << 4),
    LD_PSCAN_EVT_START_CBK_BIT                        = (1 << 5),
    LD_PSCAN_FHS_RX_BIT                               = (1 << 6),
    LD_PSCAN_FRM_CBK_BIT                              = (1 << 7),
    LD_PSCAN_FRM_ISR_BIT                              = (1 << 8),
    LD_PSCAN_INIT_BIT                                 = (1 << 9),
    LD_PSCAN_MWSCOEX_XI_GET_BIT                       = (1 << 10),
    LD_PSCAN_MWSCOEX_XI_MASK_BUILD_BIT                = (1 << 11),
    LD_PSCAN_PROG_BIT                                 = (1 << 12),
    LD_PSCAN_RESCHED_BIT                              = (1 << 13),
    LD_PSCAN_RESET_BIT                                = (1 << 14),
    LD_PSCAN_RESTART_BIT                              = (1 << 15),
    LD_PSCAN_START_BIT                                = (1 << 16),
    LD_PSCAN_STOP_BIT                                 = (1 << 17),
};

enum ld_sscan_path_fields
{
    LD_SSCAN_ACTIVATED_BIT                            = (1 << 0),
    LD_SSCAN_CLEANUP_BIT                              = (1 << 1),
    LD_SSCAN_EVT_CANCELED_CBK_BIT                     = (1 << 2),
    LD_SSCAN_EVT_START_CBK_BIT                        = (1 << 3),
    LD_SSCAN_FRM_CBK_BIT                              = (1 << 4),
    LD_SSCAN_FRM_ISR_BIT                              = (1 << 5),
    LD_SSCAN_INIT_BIT                                 = (1 << 6),
    LD_SSCAN_RESET_BIT                                = (1 << 7),
    LD_SSCAN_START_BIT                                = (1 << 8),
    LD_SSCAN_STP_RX_BIT                               = (1 << 9),
    LD_SSCAN_STOP_BIT                                 = (1 << 9),
};

enum ld_strain_path_fields
{
    LD_STRAIN_END_BIT                                 = (1 << 1),
    LD_STRAIN_EVT_CANCELED_CBK_BIT                    = (1 << 1),
    LD_STRAIN_EVT_START_CBK_BIT                       = (1 << 1),
    LD_STRAIN_FRM_CBK_BIT                             = (1 << 1),
    LD_STRAIN_FRM_ISR_BIT                             = (1 << 1),
    LD_STRAIN_INIT_BIT                                = (1 << 1),
    LD_STRAIN_RESET_BIT                               = (1 << 1),
    LD_STRAIN_START_BIT                               = (1 << 1),
    LD_STRAIN_STOP_BIT                                = (1 << 1),
};

enum ld_test_patch_fields
{
    LD_GEN_PATTERN_BIT                                = (1 << 1),
    LD_TEST_CLEANUP_BIT                               = (1 << 1),
    LD_TEST_EVT_CANCELED_CBK_BIT                      = (1 << 1),
    LD_TEST_EVT_START_CBK_BIT                         = (1 << 1),
    LD_TEST_FRM_CBK_BIT                               = (1 << 1),
    LD_TEST_FRM_ISR_BIT                               = (1 << 1),
    LD_TEST_INIT_BIT                                  = (1 << 1),
    LD_TEST_RESET_BIT                                 = (1 << 1),
    LD_TEST_RX_ISR_BIT                                = (1 << 1),
    LD_TEST_START_BIT                                 = (1 << 1),
    LD_TEST_STOP_BIT                                  = (1 << 1),
    LD_TEST_TRANSPORT_SELECT_BIT                      = (1 << 1),
};

enum ld_util_path_fields
{
    LD_UTIL_ACTIVE_MASTER_AFH_MAP_GET_BIT             = (1 << 0),
    LD_UTIL_ACTIVE_MASTER_AFH_MAP_SET_BIT             = (1 << 1),
    LD_UTIL_BCH_CREATE_BIT                            = (1 << 2),
    LD_UTIL_BCH_MODULO_BIT                            = (1 << 3),
    LD_UTIL_FHS_PK_BIT                                = (1 << 4),
    LD_UTIL_FHS_UNPK_BIT                              = (1 << 5),
    LD_UTIL_STP_PK_BIT                                = (1 << 6),
    LD_UTIL_STP_UNPK_BIT                              = (1 << 7),
};

enum lld_patch_fields
{
    LLD_AA_GEN_BIT                                    = (1 << 0),
    LLD_CALC_AUX_RX_BIT                               = (1 << 1),
    LLD_CH_IDX_GET_BIT                                = (1 << 2),
    LLD_SEC_ADV_CH_MAP_SET_FUNC_BIT                   = (1 << 3),
    LLD_CORE_INIT_BIT                                 = (1 << 4),
    LLD_INIT_BIT                                      = (1 << 5),
    LLD_PER_ADV_LIST_ADD_BIT                          = (1 << 6),
    LLD_PER_ADV_LIST_REM_BIT                          = (1 << 7),
    LLD_RAL_SEARCH_BIT                                = (1 << 8),
    LLD_READ_CLOCK_BIT                                = (1 << 9),
    LLD_REG_RD_BIT                                    = (1 << 10),
    LLD_REG_WR_BIT                                    = (1 << 11),
    LLD_RES_LIST_ADD_BIT                              = (1 << 12),
    LLD_RES_LIST_CLEAR_BIT                            = (1 << 13),
    LLD_RES_LIST_LOCAL_RPA_GET_BIT                    = (1 << 14),
    LLD_RES_LIST_PEER_RPA_GET_BIT                     = (1 << 15),
    LLD_RES_LIST_PRIV_MODE_UPDATE_BIT                 = (1 << 16),
    LLD_RES_LIST_REM_BIT                              = (1 << 17),
    LLD_RPA_RENEW_BIT                                 = (1 << 18),
    LLD_RPA_RENEW_EVT_CANCELED_CBK_BIT                = (1 << 19),
    LLD_RPA_RENEW_EVT_START_CBK_BIT                   = (1 << 20),
    LLD_RPA_RENEW_INSTANT_CBK_BIT                     = (1 << 21),
    LLD_RXDESC_BUF_READY_BIT                          = (1 << 22),
    LLD_RXDESC_CHECK_BIT                              = (1 << 23),
    LLD_RXDESC_FREE_BIT                               = (1 << 24),
    LLD_RX_TIMING_COMPUTE_BIT                         = (1 << 25),
    LLD_WHITE_LIST_ADD_BIT                            = (1 << 26),
    LLD_WHITE_LIST_REM_BIT                            = (1 << 27),
    LLD_RECOVERY_RXDESC_BIT                           = (1 << 28),
};

enum lld_2_patch_fields
{
    LLD_PATH_COMP_SET_2_FUN_BIT                       = (1 << 0),
    LLD_TX_PATH_COMP_GET_2_FUN_BIT                    = (1 << 1),
    LLD_RX_PATH_COMP_GET_2_FUN_BIT                    = (1 << 2),
    LLD_TX_POWER_DBM_GET_2_FUN_BIT                    = (1 << 3),
};

enum lld_bi_1_patch_fields
{
    LLD_BIG_ALLOC_BIT                                 = (1 << 0),
    LLD_BIG_CHMAP_UPDATE_BIT                          = (1 << 1),
    LLD_BIG_CNTL_TX_BIT                               = (1 << 2),
    LLD_BIG_CNTL_UPDATE_BIT                           = (1 << 3),
    LLD_BIG_COMPUTE_SCHED_BIT                         = (1 << 4),
    LLD_BIG_COMPUTE_SCHED_INT_BIT                     = (1 << 5),
    LLD_BIG_COMPUTE_SCHED_SEQ_BIT                     = (1 << 6),
    LLD_BIG_END_EVT_BIT                               = (1 << 7),
    LLD_BIG_EVENT_COUNTER_GET_BIT                     = (1 << 8),
    LLD_BIG_FREE_BIT                                  = (1 << 9),
    LLD_BIG_FREE_SE_BIT                               = (1 << 10),
    LLD_BIG_INIT_INFO_BIT                             = (1 << 11),
    LLD_BIG_START_BIT                                 = (1 << 12),
    LLD_BIG_STOP_FUNC_BIT                             = (1 << 13),
    LLD_BIG_UPDATE_INFO_BIT                           = (1 << 14),
    LLD_BIS_AA_GEN_BIT                                = (1 << 15),
    LLD_BIS_ALLOC_BIT                                 = (1 << 16),
    LLD_BIS_CONFIGURE_DESCS_BIT                       = (1 << 17),
    LLD_BIS_DATA_LOAD_BIT                             = (1 << 18),
    LLD_BIS_DATA_PROG_FUNC_BIT                        = (1 << 19),
    LLD_BIS_DATA_RELEASE_BIT                          = (1 << 20),
    LLD_BIS_END_EVT_BIT                               = (1 << 21),
    LLD_BIS_FREE_BIT                                  = (1 << 22),
    LLD_BIS_INSERT_BIT                                = (1 << 23),
    LLD_BIS_LOCAL_SYNC_DIS_BIT                        = (1 << 24),
    LLD_BIS_LOCAL_SYNC_EN_BIT                         = (1 << 25),
    LLD_BIS_PEER_SYNC_DIS_BIT                         = (1 << 26),
    LLD_BIS_PEER_SYNC_EN_BIT                          = (1 << 27),
    LLD_BIS_PLD_CNT_GET_BIT                           = (1 << 28),
    LLD_BIS_RX_BIT                                    = (1 << 29),
    LLD_BIS_SCAN_RX_BIT                               = (1 << 30),
    LLD_BIS_SKIP_SUBEVENT_BIT                         = (1 << 31),
};

enum lld_bi_2_patch_fields
{
    LLD_BIS_STATS_GET_2_FUNC_BIT                             = (1 << 0),
    LLD_BI_ALARM_CBK_2_FUNC_BIT                              = (1 << 1),
    LLD_BI_CNTL_FILL_CS_2_FUNC_BIT                           = (1 << 2),
    LLD_BI_COMPUTE_HOP_SCHEME_2_FUNC_BIT                     = (1 << 3),
    LLD_BI_END_SCHED_ACT_2_FUNC_BIT                          = (1 << 4),
    LLD_BI_END_SUBEVT_2_FUNC_BIT                             = (1 << 5),
    LLD_BI_EVT_CANCELED_CBK_2_FUNC_BIT                       = (1 << 6),
    LLD_BI_EVT_START_CBK_2_FUNC_BIT                          = (1 << 7),
    LLD_BI_EVT_STOP_CBK_2_FUNC_BIT                           = (1 << 8),
    LLD_BI_FILL_CS_2_FUNC_BIT                                = (1 << 9),
    LLD_BI_FRM_CBK_2_FUNC_BIT                                = (1 << 10),
    LLD_BI_FRM_ISR_2_FUNC_BIT                                = (1 << 11),
    LLD_BI_INIT_2_FUNC_BIT                                   = (1 << 12),
    LLD_BI_PREP_RSV_SCHED_2_FUNC_BIT                         = (1 << 13),
    LLD_BI_PROG_2_FUNC_BIT                                   = (1 << 14),
    LLD_BI_SCAN_FILL_CS_2_FUNC_BIT                           = (1 << 15),
    LLD_BI_SCAN_SET_CHMAP_2_FUNC_BIT                         = (1 << 16),
    LLD_BI_SCHED_2_FUNC_BIT                                  = (1 << 17),
    LLD_BI_SKIP_ISR_2_FUNC_BIT                               = (1 << 18),
    LLD_BI_SYNC_TIME_UPDATE_2_FUNC_BIT                       = (1 << 19),

};

enum lld_adv_path_fields
{
    LLD_ADV_ADI_DID_UPDATE_FUNC_BIT                   = (1 << 0),
    LLD_ADV_ADV_DATA_SET_FUNC_BIT                     = (1 << 1),
    LLD_ADV_ADV_DATA_UPDATE_FUNC_BIT                  = (1 << 2),
    LLD_ADV_AUX_CH_IDX_SET_FUNC_BIT                   = (1 << 3),
    LLD_ADV_AUX_EVT_CANCELED_CBK_FUNC_BIT             = (1 << 4),
    LLD_ADV_AUX_EVT_START_CBK_FUNC_BIT                = (1 << 5),
    LLD_ADV_DURATION_UPDATE_FUNC_BIT                  = (1 << 6),
    LLD_ADV_END_FUNC_BIT                              = (1 << 7),
    LLD_ADV_EVT_CANCELED_CBK_FUNC_BIT                 = (1 << 8),
    LLD_ADV_EVT_START_CBK_FUNC_BIT                    = (1 << 9),
    LLD_ADV_EXT_CHAIN_CONSTRUCT_FUNC_BIT              = (1 << 10),
    LLD_ADV_EXT_PKT_PREPARE_FUNC_BIT                  = (1 << 11),
    LLD_ADV_FRM_CBK_FUNC_BIT                          = (1 << 12),
    LLD_ADV_FRM_ISR_FUNC_BIT                          = (1 << 13),
    LLD_ADV_FRM_SKIP_ISR_FUNC_BIT                     = (1 << 14),
    LLD_ADV_INIT_FUNC_BIT                             = (1 << 15),
    LLD_ADV_PKT_RX_FUNC_BIT                           = (1 << 16),
    LLD_ADV_RAND_ADDR_UPDATE_FUNC_BIT                 = (1 << 17),
    LLD_ADV_RESTART_FUNC_BIT                          = (1 << 18),
    LLD_ADV_SCAN_RSP_DATA_SET_FUNC_BIT                = (1 << 19),
    LLD_ADV_SCAN_RSP_DATA_UPDATE_FUNC_BIT             = (1 << 20),
    LLD_ADV_START_FUNC_BIT                            = (1 << 21),
    LLD_ADV_STOP_FUNC_BIT                             = (1 << 22),
    LLD_ADV_SYNC_INFO_PREPARE_FUNC_BIT                = (1 << 23),
    LLD_ADV_SYNC_INFO_SET_FUNC_BIT                    = (1 << 24),
    LLD_ADV_SYNC_INFO_UPDATE_FUNC_BIT                 = (1 << 25),
};

enum lld_ci_1_patch_fields
{
    LLD_CIG_ALLOC_FUNC_BIT                            = (1 << 0),
    LLD_CIG_COMPUTE_SCHED_FUNC_BIT                    = (1 << 1),
    LLD_CIG_COMPUTE_SCHED_INT_FUNC_BIT                = (1 << 2),
    LLD_CIG_COMPUTE_SCHED_SEQ_FUNC_BIT                = (1 << 3),
    LLD_CIG_END_EVT_FUNC_BIT                          = (1 << 4),
    LLD_CIG_FREE_FUNC_BIT                             = (1 << 5),
    LLD_CIG_FREE_SCHED_FUNC_BIT                       = (1 << 6),
    LLD_CIS_AA_GEN_FUNC_BIT                           = (1 << 7),
    LLD_CIS_ALLOC_FUNC_BIT                            = (1 << 8),
    LLD_CIS_DATA_PREPARE_FUNC_BIT                     = (1 << 9),
    LLD_CIS_DATA_RX_PROG_FUNC_BIT                     = (1 << 10),
    LLD_CIS_DATA_SKIP_SUBEVT_FUNC_BIT                 = (1 << 11),
    LLD_CIS_DATA_TX_PROG_FUNC_BIT                     = (1 << 12),
    LLD_CIS_FREE_FUNC_BIT                             = (1 << 13),
    LLD_CIS_INSERT_FUNC_BIT                           = (1 << 14),
    LLD_CIS_LOCAL_SYNC_DIS_FUNC_BIT                   = (1 << 15),
    LLD_CIS_LOCAL_SYNC_EN_FUNC_BIT                    = (1 << 16),
    LLD_CIS_PEER_SYNC_DIS_FUNC_BIT                    = (1 << 17),
    LLD_CIS_PEER_SYNC_EN_FUNC_BIT                     = (1 << 18),
    LLD_CIS_PLD_CNT_GET_FUNC_BIT                      = (1 << 19),
    LLD_CIS_PROG_FUNC_BIT                             = (1 << 20),
    LLD_CIS_REMOVE_FUNC_BIT                           = (1 << 21),
    LLD_CIS_RSSI_GET_FUNC_BIT                         = (1 << 22),
    LLD_CIS_RX_FUNC_BIT                               = (1 << 23),
    LLD_CIS_START_FUNC_BIT                            = (1 << 24),
    LLD_CIS_START_SUP_TO_FUNC_BIT                     = (1 << 25),
    LLD_CIS_STATS_GET_FUNC_BIT                        = (1 << 26),
    LLD_CIS_STOP_FUNC_BIT                             = (1 << 27),
    LLD_CI_ALARM_CBK_FUNC_BIT                         = (1 << 28),
    LLD_CI_CH_MAP_UPDATE_FUNC_BIT                     = (1 << 29),
    LLD_CI_COMPUTE_HOP_SCHEME_FUNC_BIT                = (1 << 30),
    LLD_CI_END_SCHED_ACT_FUNC_BIT                     = (1 << 31),
};

enum lld_ci_2_patch_fields
{
    LLD_CI_END_SUBEVT_2_FUNC_BIT                        = (1 << 0),
    LLD_CI_EVT_CANCELED_CBK_2_FUNC_BIT                  = (1 << 1),
    LLD_CI_EVT_START_CBK_2_FUNC_BIT                     = (1 << 2),
    LLD_CI_EVT_STOP_CBK_2_FUNC_BIT                      = (1 << 3),
    LLD_CI_FRM_CBK_2_FUNC_BIT                           = (1 << 4),
    LLD_CI_FRM_ISR_2_FUNC_BIT                           = (1 << 5),
    LLD_CI_INIT_2_FUNC_BIT                              = (1 << 6),
    LLD_CI_RX_ISO_ISR_2_FUNC_BIT                        = (1 << 7),
    LLD_CI_SCHED_2_FUNC_BIT                             = (1 << 8),
    LLD_CI_SKIP_ISR_2_FUNC_BIT                          = (1 << 9),
    LLD_CI_SYNC_TIME_UPDATE_2_FUNC_BIT                  = (1 << 10),
    LLD_CI_TX_ISO_ISR_2_FUNC_BIT                        = (1 << 11),
};

enum lld_con_1_patch_fields
{
    LLD_CON_AM0_REJECT_1_FUNC_BIT                     = (1 << 0),
    LLD_CON_AM0_START_1_FUNC_BIT                      = (1 << 1),
    LLD_CON_AM0_STOP_1_FUNC_BIT                       = (1 << 2),
    LLD_CON_AM0_USE_MIC_LESS_1_FUNC_BIT               = (1 << 3),
    LLD_CON_APR_GET_1_FUNC_BIT                        = (1 << 4),
    LLD_CON_CH_MAP_UPDATE_1_FUNC_BIT                  = (1 << 5),
    LLD_CON_CLEANUP_1_FUNC_BIT                        = (1 << 6),
    LLD_CON_CTE_RX_DIS_1_FUNC_BIT                     = (1 << 7),
    LLD_CON_CTE_RX_EN_1_FUNC_BIT                      = (1 << 8),
    LLD_CON_CTE_TX_ANT_SWITCH_CONFIG_1_FUNC_BIT       = (1 << 9),
    LLD_CON_CTE_TX_PARAM_SET_1_FUNC_BIT               = (1 << 10),
    LLD_CON_CURRENT_TX_POWER_GET_1_FUNC_BIT           = (1 << 11),
    LLD_CON_DATA_FLOW_SET_1_FUNC_BIT                  = (1 << 12),
    LLD_CON_DATA_LEN_UPDATE_1_FUNC_BIT                = (1 << 13),
    LLD_CON_DATA_TX_1_FUNC_BIT                        = (1 << 14),
    LLD_CON_ENC_KEY_LOAD_1_FUNC_BIT                   = (1 << 15),
    LLD_CON_EVENT_COUNTER_GET_1_FUNC_BIT              = (1 << 16),
    LLD_CON_EVENT_TX_TIME_GET_1_FUNC_BIT              = (1 << 17),
    LLD_CON_EVT_CANCELED_CBK_1_FUNC_BIT               = (1 << 18),
    LLD_CON_EVT_START_CBK_1_FUNC_BIT                  = (1 << 19),
    LLD_CON_EVT_TIME_UPDATE_1_FUNC_BIT                = (1 << 20),
    LLD_CON_FRM_CBK_1_FUNC_BIT                        = (1 << 21),
    LLD_CON_FRM_ISR_1_FUNC_BIT                        = (1 << 22),
    LLD_CON_FRM_SKIP_ISR_1_FUNC_BIT                   = (1 << 23),
    LLD_CON_INFO_FOR_CIS_GET_1_FUNC_BIT               = (1 << 24),
    LLD_CON_INIT_1_FUNC_BIT                           = (1 << 25),
    LLD_CON_INSTANT_PROC_END_1_FUNC_BIT               = (1 << 26),
    LLD_CON_LLCP_TX_1_FUNC_BIT                        = (1 << 27),
    LLD_CON_MAX_LAT_CALC_1_FUNC_BIT                   = (1 << 28),
    LLD_CON_OFFSET_GET_1_FUNC_BIT                     = (1 << 29),
    LLD_CON_PARAM_UPDATE_1_FUNC_BIT                   = (1 << 30),
    LLD_CON_PATH_LOSS_MONITORING_1_FUNC_BIT           = (1 << 31),
};

enum lld_con_2_patch_fields
{
    LLD_CON_PATH_LOSS_MONITOR_CONFIG_2_FUNC_BIT       = (1 << 0),
    LLD_CON_PATH_LOSS_MONITOR_EN_2_FUNC_BIT           = (1 << 1),
    LLD_CON_PEER_SCA_SET_2_FUNC_BIT                   = (1 << 2),
    LLD_CON_PHYS_UPDATE_2_FUNC_BIT                    = (1 << 3),
    LLD_CON_PREF_SLAVE_EVT_DUR_SET_2_FUNC_BIT         = (1 << 4),
    LLD_CON_PREF_SLAVE_LATENCY_SET_2_FUNC_BIT         = (1 << 5),
    LLD_CON_PRIO_IDX_GET_2_FUNC_BIT                   = (1 << 6),
    LLD_CON_REMOTE_TX_PWR_GET_2_FUNC_BIT              = (1 << 7),
    LLD_CON_REMOTE_TX_PWR_SET_2_FUNC_BIT              = (1 << 8),
    LLD_CON_RSSI_GET_2_FUNC_BIT                       = (1 << 9),
    LLD_CON_RSSI_UPDATE_2_FUNC_BIT                    = (1 << 10),
    LLD_CON_RX_2_FUNC_BIT                             = (1 << 11),
    LLD_CON_RX_AM0_ISR_2_FUNC_BIT                     = (1 << 12),
    LLD_CON_RX_ENC_2_FUNC_BIT                         = (1 << 13),
    LLD_CON_RX_ISR_2_FUNC_BIT                         = (1 << 14),
    LLD_CON_RX_RATE_GET_2_FUNC_BIT                    = (1 << 15),
    LLD_CON_SCHED_2_FUNC_BIT                          = (1 << 16),
    LLD_CON_START_2_FUNC_BIT                          = (1 << 17),
    LLD_CON_STOP_2_FUNC_BIT                           = (1 << 18),
    LLD_CON_SUP_TO_GET_2_FUNC_BIT                     = (1 << 19),
    LLD_CON_SYNC_TIME_UPDATE_2_FUNC_BIT               = (1 << 20),
    LLD_CON_TIME_GET_2_FUNC_BIT                       = (1 << 21),
    LLD_CON_TX_2_FUNC_BIT                             = (1 << 22),
    LLD_CON_TX_AM0_ISR_2_FUNC_BIT                     = (1 << 23),
    LLD_CON_TX_ENC_2_FUNC_BIT                         = (1 << 24),
    LLD_CON_TX_ISR_2_FUNC_BIT                         = (1 << 25),
    LLD_CON_TX_LEN_UPDATE_2_FUNC_BIT                  = (1 << 26),
    LLD_CON_TX_LEN_UPDATE_FOR_INTV_2_FUNC_BIT         = (1 << 27),
    LLD_CON_TX_LEN_UPDATE_FOR_RATE_2_FUNC_BIT         = (1 << 28),
    LLD_CON_TX_POWER_ADJ_2_FUNC_BIT                   = (1 << 29),
    LLD_CON_TX_POWER_GET_2_FUNC_BIT                   = (1 << 30),
    LLD_CON_TX_PROG_2_FUNC_BIT                        = (1 << 31),
};

enum lld_con_3_patch_fields
{
    LLD_CON_TX_PWR_LVL_GET_3_FUNC_BIT                 = (1 << 0),
    LLD_CON_TX_RATE_GET_3_FUNC_BIT                    = (1 << 1),
    LLD_CON_PRIO_ELEVATION_SET_3_FUNC_BIT             = (1 << 2),
    LLD_CON_EVT_TRACE_3_FUNC_BIT                      = (1 << 3),
};

enum lld_init_patch_fields
{
    LLD_INIT_COMPUTE_WINOFFSET_FUNC_BIT             = (1 << 0),
    LLD_INIT_CONNECT_REQ_PACK_FUNC_BIT              = (1 << 1),
    LLD_INIT_END_FUNC_BIT                           = (1 << 2),
    LLD_INIT_EVT_CANCELED_CBK_FUNC_BIT              = (1 << 3),
    LLD_INIT_EVT_START_CBK_FUNC_BIT                 = (1 << 4),
    LLD_INIT_FRM_CBK_FUNC_BIT                       = (1 << 5),
    LLD_INIT_FRM_EOF_ISR_FUNC_BIT                   = (1 << 6),
    LLD_INIT_FRM_SKIP_ISR_FUNC_BIT                  = (1 << 7),
    LLD_INIT_INIT_FUNC_BIT                          = (1 << 8),
    LLD_INIT_PROCESS_PKT_RX_FUNC_BIT                = (1 << 9),
    LLD_INIT_PROCESS_PKT_TX_FUNC_BIT                = (1 << 10),
    LLD_INIT_RAND_ADDR_UPDATE_FUNC_BIT              = (1 << 11),
    LLD_INIT_SCHED_FUNC_BIT                         = (1 << 12),
    LLD_INIT_START_FUNC_BIT                         = (1 << 13),
    LLD_INIT_STOP_FUNC_BIT                          = (1 << 14),
};

enum lld_iso_patch_fields
{
    LLD_ISO_HOP_CANCEL_FUNC_BIT                          = (1 << 0),
    LLD_ISO_HOP_COMPUTE_FUNC_BIT                         = (1 << 1),
    LLD_ISO_HOP_ISR_FUNC_BIT                             = (1 << 2),
    LLD_ISO_INIT_FUNC_BIT                                = (1 << 3),
    LLD_ISO_START_HOP_SCHEME_COMPUTE_FUNC_BIT            = (1 << 4),
};

enum lld_isoal_patch_fields
{
    LLD_ISOAL_DATAPATH_REMOVE_FUNC_BIT                   = (1 << 0),
    LLD_ISOAL_DATAPATH_SET_FUNC_BIT                      = (1 << 1),
    LLD_ISOAL_INIT_FUNC_BIT                              = (1 << 2),
    LLD_ISOAL_RX_DONE_FUNC_BIT                           = (1 << 3),
    LLD_ISOAL_RX_DONE_FRAMED_FUNC_BIT                    = (1 << 4),
    LLD_ISOAL_RX_DONE_UNFRAMED_FUNC_BIT                  = (1 << 5),
    LLD_ISOAL_START_FUNC_BIT                             = (1 << 6),
    LLD_ISOAL_STOP_FUNC_BIT                              = (1 << 7),
    LLD_ISOAL_TX_GET_FUNC_BIT                            = (1 << 8),
    LLD_ISOAL_TX_GET_FRAMED_FUNC_BIT                     = (1 << 9),
    LLD_ISOAL_TX_GET_UNFRAMED_FUNC_BIT                   = (1 << 10),
};

enum lld_per_adv_patch_fields
{

    LLD_PER_ADV_ANT_SWITCH_CONFIG_FUNC_BIT               = (1 << 0),
    LLD_PER_ADV_BIG_UPDATE_FUNC_BIT                      = (1 << 1),
    LLD_PER_ADV_CHAIN_CONSTRUCT_FUNC_BIT                 = (1 << 2),
    LLD_PER_ADV_CH_MAP_UPDATE_FUNC_BIT                   = (1 << 3),
    LLD_PER_ADV_CLEANUP_FUNC_BIT                         = (1 << 4),
    LLD_PER_ADV_CTE_START_FUNC_BIT                       = (1 << 5),
    LLD_PER_ADV_CTE_STOP_FUNC_BIT                        = (1 << 6),
    LLD_PER_ADV_DATA_SET_FUNC_BIT                        = (1 << 7),
    LLD_PER_ADV_DATA_UPDATE_FUNC_BIT                     = (1 << 8),
    LLD_PER_ADV_EVT_CANCELED_CBK_FUNC_BIT                = (1 << 9),
    LLD_PER_ADV_EVT_START_CBK_FUNC_BIT                   = (1 << 10),
    LLD_PER_ADV_EXT_PKT_PREPARE_FUNC_BIT                 = (1 << 11),
    LLD_PER_ADV_FRM_CBK_FUNC_BIT                         = (1 << 12),
    LLD_PER_ADV_FRM_ISR_FUNC_BIT                         = (1 << 13),
    LLD_PER_ADV_FRM_SKIP_ISR_FUNC_BIT                    = (1 << 14),
    LLD_PER_ADV_INFO_GET_FUNC_BIT                        = (1 << 15),
    LLD_PER_ADV_INIT_FUNC_BIT                            = (1 << 16),
    LLD_PER_ADV_INIT_INFO_GET_FUNC_BIT                   = (1 << 17),
    LLD_PER_ADV_SCHED_FUNC_BIT                           = (1 << 18),
    LLD_PER_ADV_START_FUNC_BIT                           = (1 << 19),
    LLD_PER_ADV_STOP_FUNC_BIT                            = (1 << 20),
    LLD_PER_ADV_SYNC_INFO_GET_FUNC_BIT                   = (1 << 21),

};

enum lld_scan_patch_fields
{

    LLD_SCAN_AUX_EVT_CANCELED_CBK_FUNC_BIT               = (1 << 0),
    LLD_SCAN_AUX_EVT_START_CBK_FUNC_BIT                  = (1 << 1),
    LLD_SCAN_AUX_SCHED_FUNC_BIT                          = (1 << 2),
    LLD_SCAN_CREATE_SYNC_FUNC_BIT                        = (1 << 3),
    LLD_SCAN_CREATE_SYNC_CANCEL_FUNC_BIT                 = (1 << 4),
    LLD_SCAN_END_FUNC_BIT                                = (1 << 5),
    LLD_SCAN_EVT_CANCELED_CBK_FUNC_BIT                   = (1 << 6),
    LLD_SCAN_EVT_START_CBK_FUNC_BIT                      = (1 << 7),
    LLD_SCAN_FRM_CBK_FUNC_BIT                            = (1 << 8),
    LLD_SCAN_FRM_EOF_ISR_FUNC_BIT                        = (1 << 9),
    LLD_SCAN_FRM_RX_ISR_FUNC_BIT                         = (1 << 10),
    LLD_SCAN_FRM_SKIP_ISR_FUNC_BIT                       = (1 << 11),
    LLD_SCAN_INIT_FUNC_BIT                               = (1 << 12),
    LLD_SCAN_PARAMS_UPDATE_FUNC_BIT                      = (1 << 13),
    LLD_SCAN_PROCESS_PKT_RX_FUNC_BIT                     = (1 << 14),
    LLD_SCAN_RAND_ADDR_UPDATE_FUNC_BIT                   = (1 << 15),
    LLD_SCAN_RESTART_FUNC_BIT                            = (1 << 16),
    LLD_SCAN_SCHED_FUNC_BIT                              = (1 << 17),
    LLD_SCAN_START_FUNC_BIT                              = (1 << 18),
    LLD_SCAN_STOP_FUNC_BIT                               = (1 << 19),
    LLD_SCAN_SYNC_ACCEPT_FUNC_BIT                        = (1 << 20),
    LLD_SCAN_SYNC_INFO_UNPACK_FUNC_BIT                   = (1 << 21),
    LLD_SCAN_TRUNC_IND_FUNC_BIT                          = (1 << 22),
    LLD_SCAN_AUX_PTR_CHECK_FUNC_BIT                      = (1 << 23),
};

enum lld_sync_patch_fields
{
    LLD_SYNC_ANT_SWITCH_CONFIG_FUNC_BIT                  = (1 << 0),
    LLD_SYNC_CH_MAP_UPDATE_FUNC_BIT                      = (1 << 1),
    LLD_SYNC_CLEANUP_FUNC_BIT                            = (1 << 2),
    LLD_SYNC_CTE_START_FUNC_BIT                          = (1 << 3),
    LLD_SYNC_CTE_STOP_FUNC_BIT                           = (1 << 4),
    LLD_SYNC_EVT_CANCELED_CBK_FUNC_BIT                   = (1 << 5),
    LLD_SYNC_EVT_START_CBK_FUNC_BIT                      = (1 << 6),
    LLD_SYNC_FRM_CBK_FUNC_BIT                            = (1 << 7),
    LLD_SYNC_FRM_EOF_ISR_FUNC_BIT                        = (1 << 8),
    LLD_SYNC_FRM_RX_ISR_FUNC_BIT                         = (1 << 9),
    LLD_SYNC_FRM_SKIP_ISR_FUNC_BIT                       = (1 << 10),
    LLD_SYNC_INFO_FOR_BIG_GET_FUNC_BIT                   = (1 << 11),
    LLD_SYNC_INFO_GET_FUNC_BIT                           = (1 << 12),
    LLD_SYNC_INIT_FUNC_BIT                               = (1 << 13),
    LLD_SYNC_PROCESS_PKT_RX_FUNC_BIT                     = (1 << 14),
    LLD_SYNC_SCHED_FUNC_BIT                              = (1 << 15),
    LLD_SYNC_START_FUNC_BIT                              = (1 << 16),
    LLD_SYNC_STOP_FUNC_BIT                               = (1 << 17),
    LLD_SYNC_SYNC_TIME_UPDATE_FUNC_BIT                   = (1 << 18),
    LLD_SYNC_TRUNC_IND_FUNC_BIT                          = (1 << 19),
    LLD_SYNC_REP_FILT_EN_FUNC_BIT                        = (1 << 20),
};

enum lld_test_patch_fields
{

    LLD_GEN_PATTERN_FUNC_BIT                             = (1 << 0),
    LLD_TEST_CLEANUP_FUNC_BIT                            = (1 << 1),
    LLD_TEST_EVT_CANCELED_CBK_FUNC_BIT                   = (1 << 2),
    LLD_TEST_EVT_START_CBK_FUNC_BIT                      = (1 << 3),
    LLD_TEST_FREQ2CHNL_FUNC_BIT                          = (1 << 4),
    LLD_TEST_FRM_CBK_FUNC_BIT                            = (1 << 5),
    LLD_TEST_FRM_ISR_FUNC_BIT                            = (1 << 6),
    LLD_TEST_INIT_FUNC_BIT                               = (1 << 7),
    LLD_TEST_RX_ISR_FUNC_BIT                             = (1 << 8),
    LLD_TEST_START_FUNC_BIT                              = (1 << 9),
    LLD_TEST_STOP_FUNC_BIT                               = (1 << 10),
};

enum sch_alarm_patch_fields
{
    SCH_ALARM_CLEAR_FUNC_BIT                             = (1 << 0),
    SCH_ALARM_INIT_FUNC_BIT                              = (1 << 1),
    SCH_ALARM_PROG_FUNC_BIT                              = (1 << 2),
    SCH_ALARM_SET_FUNC_BIT                               = (1 << 3),
    SCH_ALARM_TIMER_ISR_FUNC_BIT                         = (1 << 4),
};

enum sch_arb_patch_fields
{

    SCH_ARB_CONFLICT_FUNC_BIT                            = (1 << 0),
    SCH_ARB_CONFLICT_CHECK_FUNC_BIT                      = (1 << 1),
    SCH_ARB_ELT_CANCEL_FUNC_BIT                          = (1 << 2),
    SCH_ARB_EVENT_START_ISR_FUNC_BIT                     = (1 << 3),
    SCH_ARB_INIT_FUNC_BIT                                = (1 << 4),
    SCH_ARB_INSERT_FUNC_BIT                              = (1 << 5),
    SCH_ARB_PHASE_ALIGN_HI_FUNC_BIT                      = (1 << 6),
    SCH_ARB_PROG_TIMER_FUNC_BIT                          = (1 << 7),
    SCH_ARB_REMOVE_FUNC_BIT                              = (1 << 8),
    SCH_ARB_SW_ISR_FUNC_BIT                              = (1 << 9),
};

enum sch_plan_patch_fields
{
    SCH_PLAN_CHK_FUNC_BIT                                = (1 << 0),
    SCH_PLAN_CLOCK_WRAP_OFFSET_UPDATE_FUNC_BIT           = (1 << 1),
    SCH_PLAN_INIT_FUNC_BIT                               = (1 << 2),
    SCH_PLAN_INTERVAL_REQ_FUNC_BIT                       = (1 << 3),
    SCH_PLAN_OFFSET_CHK_FUNC_BIT                         = (1 << 4),
    SCH_PLAN_OFFSET_RANGE_COMPUTE_FUNC_BIT               = (1 << 5),
    SCH_PLAN_REM_FUNC_BIT                                = (1 << 6),
    SCH_PLAN_REQ_FUNC_BIT                                = (1 << 7),
    SCH_PLAN_SET_FUNC_BIT                                = (1 << 8),
    SCH_PLAN_SHIFT_FUNC_BIT                              = (1 << 9),
};

enum sch_prog_patch_fields
{
    SCH_PROG_CLK_ISR_FUNC_BIT                            = (1 << 0),
    SCH_PROG_DISABLE_FUNC_BIT                            = (1 << 1),
    SCH_PROG_ENABLE_FUNC_BIT                             = (1 << 2),
    SCH_PROG_END_ISR_FUNC_BIT                            = (1 << 3),
    SCH_PROG_FIFO_ISR_FUNC_BIT                           = (1 << 4),
    SCH_PROG_INIT_FUNC_BIT                               = (1 << 5),
    SCH_PROG_PUSH_FUNC_BIT                               = (1 << 6),
    SCH_PROG_RX_ISO_ISR_FUNC_BIT                         = (1 << 7),
    SCH_PROG_RX_ISR_FUNC_BIT                             = (1 << 8),
    SCH_PROG_SKIP_ISR_FUNC_BIT                           = (1 << 9),
    SCH_PROG_TX_ISO_ISR_FUNC_BIT                         = (1 << 10),
    SCH_PROG_TX_ISR_FUNC_BIT                             = (1 << 11),
};

enum sch_slice_patch_fields
{

    SCH_SLICE_BG_ADD_FUNC_BIT                            = (1 << 0),
    SCH_SLICE_BG_REMOVE_FUNC_BIT                         = (1 << 1),
    SCH_SLICE_COMPUTE_FUNC_BIT                           = (1 << 2),
    SCH_SLICE_FG_ADD_FUNC_BIT                            = (1 << 3),
    SCH_SLICE_FG_REMOVE_FUNC_BIT                         = (1 << 4),
    SCH_SLICE_INIT_FUNC_BIT                              = (1 << 5),
    SCH_SLICE_PER_ADD_FUNC_BIT                           = (1 << 6),
    SCH_SLICE_PER_REMOVE_FUNC_BIT                        = (1 << 7),
};

enum rwip_path_fields
{
    BLE_BOOT_FUNC_BIT                                = (1 << 0),
    DISPLAY_ADD_CONFIG_FUNC_BIT                      = (1 << 1),
    _ENV_INIT_FUNC_BIT                               = (1 << 2),
    RWIP_ASSERT_FUNC_BIT                             = (1 << 3),
    _RWIP_INIT_FUNC_BIT                              = (1 << 4),
    RWIP_ISO_DATA_TRANSFER_FUNC_BIT                  = (1 << 5),
    RWIP_MWSCOEX_SET_FUNC_BIT                        = (1 << 6),
    RWIP_PARAM_DUMMY_DEL_FUNC_BIT                    = (1 << 7),
    RWIP_PARAM_DUMMY_GET_FUNC_BIT                    = (1 << 8),
    RWIP_PARAM_DUMMY_SET_FUNC_BIT                    = (1 << 9),
    RWIP_PCA_CLOCK_DRAGGING_ONLY_FUNC_BIT            = (1 << 10),
    RWIP_RAND_INIT_FUNC_BIT                          = (1 << 11),
    _RWIP_RESET_FUNC_BIT                             = (1 << 12),
    RWIP_SCHEDULE_FUNC_BIT                           = (1 << 13),
    RWIP_WLCOEX_SET_FUNC_BIT                         = (1 << 14),
};

enum rwip_driver_1_patch_fields
{
    DM_ISR_1_FUNC_BIT                                  = (1 << 0),
    RWIP_AES_ENCRYPT_1_FUNC_BIT                        = (1 << 1),
    RWIP_BTDM_ISR_1_FUNC_BIT                           = (1 << 2),
    RWIP_BTS_TO_BT_TIME_1_FUNC_BIT                     = (1 << 3),
    RWIP_BT_TIME_TO_BTS_1_FUNC_BIT                     = (1 << 4),
    RWIP_CHANNEL_ASSESS_BLE_1_FUNC_BIT                 = (1 << 5),
    RWIP_CHANNEL_ASSESS_BT_1_FUNC_BIT                  = (1 << 6),
    RWIP_CH_ASSESS_DATA_BLE_GET_1_FUNC_BIT             = (1 << 7),
    RWIP_CH_ASSESS_DATA_BT_GET_1_FUNC_BIT              = (1 << 8),
    RWIP_CH_ASS_EN_GET_1_FUNC_BIT                      = (1 << 9),
    RWIP_CH_ASS_EN_SET_1_FUNC_BIT                      = (1 << 10),
    RWIP_CRYPT_EVT_HANDLER_1_FUNC_BIT                  = (1 << 11),
    RWIP_CRYPT_ISR_HANDLER_1_FUNC_BIT                  = (1 << 12),
    RWIP_CURRENT_DRIFT_GET_1_FUNC_BIT                  = (1 << 13),
    RWIP_DRIVER_INIT_1_FUNC_BIT                        = (1 << 14),
    RWIP_GET_LP_REF_CYCLE_1_FUNC_BIT                   = (1 << 15),
    RWIP_LPCYCLES_2_HUS_1_FUNC_BIT                     = (1 << 16),
    RWIP_MAX_DRIFT_GET_1_FUNC_BIT                      = (1 << 17),
    RWIP_PREVENT_SLEEP_CHECK_1_FUNC_BIT                = (1 << 18),
    RWIP_PREVENT_SLEEP_CLEAR_1_FUNC_BIT                = (1 << 19),
    RWIP_PREVENT_SLEEP_SET_1_FUNC_BIT                  = (1 << 20),
    RWIP_RC_CAL_1_FUNC_BIT                             = (1 << 21),
    RWIP_RC_CYCLE_UPDATE_1_FUNC_BIT                    = (1 << 22),
    RWIP_SCA_GET_1_FUNC_BIT                            = (1 << 23),
    _RWIP_SLEEP_1_FUNC_BIT                             = (1 << 24),
    RWIP_SLEEP_ENTER_1_FUNC_BIT                        = (1 << 25),
    RWIP_SLEEP_WAKEUP_END_1_FUNC_BIT                   = (1 << 26),
    RWIP_SLOT_2_LPCYCLES_1_FUNC_BIT                    = (1 << 27),
    RWIP_SW_INT_HANDLER_1_FUNC_BIT                     = (1 << 28),
    RWIP_SW_INT_REQ_1_FUNC_BIT                         = (1 << 29),
    RWIP_TIMER_ALARM_HANDLER_1_FUNC_BIT                = (1 << 30),
    RWIP_TIMER_ALARM_SET_1_FUNC_BIT                    = (1 << 31),
};

enum rwip_driver_2_patch_fields
{
    RWIP_TIMER_ARB_HANDLER_2_FUNC_BIT                  = (1 << 0),
    RWIP_TIMER_ARB_SET_2_FUNC_BIT                      = (1 << 1),
    RWIP_TIMER_CO_HANDLER_2_FUNC_BIT                   = (1 << 2),
    RWIP_TIMER_CO_SET_2_FUNC_BIT                       = (1 << 3),
    RWIP_TIME_ADJ_2_FUNC_BIT                           = (1 << 4),
    RWIP_TIME_GET_2_FUNC_BIT                           = (1 << 5),
    RWIP_TIME_SET_2_FUNC_BIT                           = (1 << 6),
    RWIP_US_2_LPCYCLES_2_FUNC_BIT                      = (1 << 7),
    RWIP_WAKEUP_2_FUNC_BIT                             = (1 << 8),
    RWIP_WAKEUP_CHECK_2_FUNC_BIT                       = (1 << 9),
    RWIP_WAKEUP_END_2_FUNC_BIT                         = (1 << 10),
    RWIP_AES_2_FUNC_BIT                                = (1 << 11),
    RWIP_CH_CLASS_BLE_GET_2_FUNC_BIT                   = (1 << 12),
    RWIP_CH_MAP_BLE_GET_2_FUNC_BIT                     = (1 << 13),
    RWIP_CH_MAP_BLE_FILL_2_FUNC_BIT                    = (1 << 14),
    RWIP_CH_CLASS_BT_GET_2_FUNC_BIT                    = (1 << 15),
    RWIP_CH_MAP_BT_GET_2_FUNC_BIT                      = (1 << 16),
    RWIP_ACTIVATE_CHANNELS_CH_MAP_BT_2_FUNC_BIT        = (1 << 17),
    RWIP_CHANNEL_STAT_INSERT_2_FUNC_BIT                = (1 << 18),
    //RWIP_CHANNEL_ASSESS_BT_2_FUNC_BIT                  = (1 << 19),
};

enum ke_lp_patch_fields
{
    BLE_AON_IRQ_HANDLER_FUN_BIT                           = (1 << 0),
    BLE_DEEP_SLEEP_AFTER_HANDLER_FUN_BIT                  = (1 << 1),
    BLE_DEEP_SLEEP_PRE_HANDLER_FUN_BIT                    = (1 << 2),
    BLE_STANDBY_SLEEP_AFTER_HANDLER_FUN_BIT               = (1 << 3),
    BLE_STANDBY_SLEEP_PRE_HANDLER_FUN_BIT                 = (1 << 4),
    KE_BLE_MAC_REGISTER_FUN_BIT                           = (1 << 5),
    KE_IDLE_HOOK_FUNC_FUN_BIT                             = (1 << 6),
    KE_LP_AON_IRQ_HANDLER_FUN_BIT                         = (1 << 7),
    KE_MAC_SLEEP_CHECK_FUN_BIT                            = (1 << 8),
    KE_MAC_SLEEP_WAKEUP_FUN_BIT                           = (1 << 9),
    KE_SLEEP_BLE_MAC_SUSPEND_FUN_BIT                      = (1 << 10),
    KE_SLEEP_CONTEXT_SWITCH_HOOK_FUN_BIT                  = (1 << 11),
    KE_SLEEP_EXIT_FUN_BIT                                 = (1 << 12),
    KE_SLEEP_IDLE_HOOK_FUN_BIT                            = (1 << 13),
    _KE_SLEEP_OS_HOOK_FUN_BIT                             = (1 << 14),
    KE_SLEEP_REQUEST_FUN_BIT                              = (1 << 15),
    KE_SLEEP_TEST_TRIGGER_HANDLER_FUN_BIT                 = (1 << 16),
    RT_SYSTEM_TIMER_RECOVERY_COMPENSATION_FUN_BIT         = (1 << 17),
    SIFLI_STANDBY_REGISTER_RECOVERY_FUN_BIT               = (1 << 18),
    SIFLI_STANDBY_REGISTER_STORE_FUN_BIT                  = (1 << 19),
};

enum lc_task_patch_fields
{
    LC_UTIL_CHECK_TRANS_COLL_FUN_BIT                          = (1 << 0),
    LC_SAVE_SCO_PARA_FUN_BIT                                  = (1 << 1),
    LC_GET_SCO_CONHDL_FUN_BIT                                 = (1 << 2),
    LC_BT_SCO_DATA_RECV_FUN_BIT                               = (1 << 3),
    LC_BT_SCO_DATA_SEND_FUN_BIT                               = (1 << 4),
    LC_BT_SEND_DATA_DONE_FUN_BIT                              = (1 << 5),
    LC_SAVE_SCO_CHG_PARA_FUN_BIT                              = (1 << 6),
    LC_SCO_SEND_HOST_CMP_EVT_FUN_BIT                          = (1 << 7),
    LC_SCO_SEND_HOST_CHG_EVT_FUN_BIT                          = (1 << 8),
};

enum lli_bi_patch_fields
{
    HCI_LE_BIG_CREATE_SYNC_CMD_HANDLER_FUN_BIT                            = (1 << 0),
    HCI_LE_BIG_TERMINATE_SYNC_CMD_HANDLER_FUN_BIT                         = (1 << 1),
    HCI_LE_CREATE_BIG_CMD_HANDLER_FUN_BIT                                 = (1 << 2),
    HCI_LE_CREATE_BIG_TEST_CMD_HANDLER_FUN_BIT                            = (1 << 3),
    HCI_LE_TERMINATE_BIG_CMD_HANDLER_FUN_BIT                              = (1 << 4),
    LLD_BIG_RX_IND_HANDLER_FUN_BIT                                        = (1 << 5),
    LLD_BIG_STOP_IND_HANDLER_FUN_BIT                                      = (1 << 6),
    LLD_BIG_SYNC_ESTAB_IND_HANDLER_FUN_BIT                                = (1 << 7),
    LLD_BIG_SYNC_OFFSET_UPD_IND_HANDLER_FUN_BIT                           = (1 << 8),
    LLD_BIG_TX_IND_HANDLER_FUN_BIT                                        = (1 << 9),
    LLI_BIS_DATA_PATH_REMOVE_FUN_BIT                                      = (1 << 10),
    LLI_BIS_DATA_PATH_SET_FUN_BIT                                         = (1 << 11),
    LLI_BIS_STATS_GET_FUN_BIT                                             = (1 << 12),
    LLI_BI_CH_MAP_UPDATE_FUN_BIT                                          = (1 << 13),
    LLM_ACAD_DATA_IND_HANDLER_FUN_BIT                                     = (1 << 14),
    LLI_BI_INIT_FUN_BIT                                                   = (1 << 15),
};

enum lli_ci_patch_fields
{
    HCI_DISCONNECT_CIS_CMD_HANDLER_FUN_BIT                          = (1 << 0),
    HCI_LE_ACCEPT_CIS_REQ_CMD_HANDLER_FUN_BIT                       = (1 << 1),
    HCI_LE_CREATE_CIS_CMD_HANDLER_FUN_BIT                           = (1 << 2),
    HCI_LE_REJECT_CIS_REQ_CMD_HANDLER_FUN_BIT                       = (1 << 3),
    HCI_LE_REMOVE_CIG_CMD_HANDLER_FUN_BIT                           = (1 << 4),
    HCI_LE_SET_CIG_PARAMS_CMD_HANDLER_FUN_BIT                       = (1 << 5),
    HCI_LE_SET_CIG_PARAMS_TEST_CMD_HANDLER_FUN_BIT                  = (1 << 6),
    LLD_CIS_ESTAB_IND_HANDLER_FUN_BIT                               = (1 << 7),
    LLD_CIS_STOP_IND_HANDLER_FUN_BIT                                = (1 << 8),
    LLI_CIS_DATA_PATH_REMOVE_FUN_BIT                                = (1 << 9),
    LLI_CIS_DATA_PATH_SET_FUN_BIT                                   = (1 << 10),
    LLI_CIS_IS_PRESENT_FUN_BIT                                      = (1 << 11),
    LLI_CIS_LINK_STOP_IND_FUN_BIT                                   = (1 << 12),
    LLI_CIS_STATS_GET_FUN_BIT                                       = (1 << 13),
    LLI_CI_INIT_FUN_BIT                                             = (1 << 14),
    LLI_CIS_ACT_ID_GET_FUN_BIT                          = (1 << 15),
    LLI_CI_GET_RATE_FUN_BIT                             = (1 << 16),
    LLI_CI_SE_DUR_COMPUTE_FUN_BIT                       = (1 << 17),
    LLI_CIS_ESTABLISHED_EVT_SEND_FUN_BIT                = (1 << 18),
    LLI_CIS_DISC_CMP_EVT_SEND_FUN_BIT                   = (1 << 19),
    LLI_CIS_CREATE_FUN_BIT                              = (1 << 20),
    LLI_CIS_CLEANUP_FUN_BIT                             = (1 << 21),
    LLI_CIG_GET_FUN_BIT                                 = (1 << 22),
    LLI_CIS_CREATE_CONT_FUN_BIT                         = (1 << 23),
    LLI_CIG_COMPUTE_PARAMS_FUN_BIT                      = (1 << 24),
    LLI_CIS_START_FUN_BIT                               = (1 << 25),
    LLI_CI_PARAM_GET_FUN_BIT                            = (1 << 26),
    LLI_CIS_CREATE_REQ_FUN_BIT                          = (1 << 27),
    LLI_CIS_CREATE_END_FUN_BIT                          = (1 << 28),
    LLI_CIS_STOP_REQ_FUN_BIT                            = (1 << 29),
    LLI_CIS_STOP_END_FUN_BIT                            = (1 << 30),
    LLI_CIS_REQUEST_EVT_SEND_FUN_BIT                    = (1 << 31),
};

enum llc_cis_patch_fields
{
    LLC_CIS_ACCEPT_TO_HANDLER_FUN_BIT                    = (1 << 0),
    LLC_OP_CIS_CREATE_IND_HANDLER_FUN_BIT                = (1 << 1),
    LLC_OP_CIS_STOP_IND_HANDLER_FUN_BIT                  = (1 << 2),
    LL_CIS_IND_HANDLER_FUN_BIT                           = (1 << 3),
    LL_CIS_REQ_HANDLER_FUN_BIT                           = (1 << 4),
    LL_CIS_RSP_HANDLER_FUN_BIT                           = (1 << 5),
    LL_CIS_TERMINATE_IND_HANDLER_FUN_BIT                 = (1 << 6),
};

enum rwble_patch_fields
{
    RWBLE_INIT_FUN_BIT                                   = (1 << 0),
    RWBLE_ACTIVITY_ONGOING_CHECK_FUN_BIT                 = (1 << 1),
    BLE_ISR_FUN_BIT                                      = (1 << 2),
};

enum rwbt_patch_fields
{
    RWBT_INIT_FUN_BIT                                       = (1 << 0),
    RWBT_RESET_FUN_BIT                                      = (1 << 1),
    BT_ISR_FUN_BIT                                          = (1 << 2),
};

enum lld_ch_scan_patch_fields
{
    LLD_CH_SCAN_CLEANUP_FUN_BIT                          = (1 << 0),
    LLD_CH_SCAN_ENABLE_ALL_CH_MAP_FUN_BIT                = (1 << 1),
    LLD_CH_SCAN_EOF_ISR_FUN_BIT                          = (1 << 2),
    LLD_CH_SCAN_EVT_CANCELED_CBK_FUN_BIT                 = (1 << 3),
    LLD_CH_SCAN_EVT_START_CBK_FUN_BIT                    = (1 << 4),
    LLD_CH_SCAN_FRM_CBK_FUN_BIT                          = (1 << 5),
    LLD_CH_SCAN_INIT_FUN_BIT                             = (1 << 6),
    LLD_CH_SCAN_RX_ISR_FUN_BIT                           = (1 << 7),
    LLD_CH_SCAN_START_FUN_BIT                            = (1 << 8),
    LLD_CH_SCAN_STOP_FUN_BIT                             = (1 << 9),
    LLD_CH_SCAN_CH_MAP_UPDATE_FUN_BIT                    = (1 << 10),
};

enum lli_patch_fields
{
    HCI_DBG_ISO_SET_PARAM_CMD_HANDLER_FUN_BIT                                = (1 << 0),
    HCI_LE_RD_ISO_LINK_QUALITY_CMD_HANDLER_FUN_BIT                           = (1 << 1),
    LLI_GROUP_CLEANUP_FUN_BIT                                                = (1 << 2),
    LLI_GROUP_CREATE_FUN_BIT                                                 = (1 << 3),
    LLI_GROUP_ENV_GET_FUN_BIT                                                = (1 << 4),
    LLI_INIT_FUN_BIT                                                         = (1 << 5),
    LLI_LINK_STOP_IND_FUN_BIT                                                = (1 << 6),
};

enum rf_patch_fields
{
    RF_INIT_FUN_BIT                         = (1 << 0),
    RF_INIT_END_FUN_BIT                     = (1 << 1),
    BT_CFG_FUN_BIT                          = (1 << 2),
    BT_CFG_END_FUN_BIT                      = (1 << 3),
    RF_PARA_CONFIGURE_FUN_BIT               = (1 << 4),
    RF_PARA_CONFIGURE_END_FUN_BIT           = (1 << 5),
    RF_EM_INIT_FUN_BIT                      = (1 << 6),
};

enum lm_1_patch_fields
{
    LM_ACL_DISC_1_FUN_BIT                                   = (1 << 0),
    LM_AFH_ACTIVATE_TIMER_1_FUN_BIT                         = (1 << 1),
    LM_AFH_ED_CH_CLASS_GET_1_FUN_BIT                        = (1 << 2),
    LM_AFH_ED_CH_CLASS_SET_1_FUN_BIT                        = (1 << 3),
    LM_AFH_HOST_CH_CLASS_GET_1_FUN_BIT                      = (1 << 4),
    LM_BD_ADDR_GET_1_FUN_BIT                                = (1 << 5),
    LM_CH_MAP_COMPUTE_1_FUN_BIT                             = (1 << 6),
    LM_DEBUG_KEY_COMPARE_192_1_FUN_BIT                      = (1 << 7),
    LM_DEBUG_KEY_COMPARE_256_1_FUN_BIT                      = (1 << 8),
    LM_DUT_MODE_EN_GET_1_FUN_BIT                            = (1 << 9),
    LM_FILL_CH_MAP_1_FUN_BIT                                = (1 << 10),
    LM_FIND_LINK_ID_1_FUN_BIT                               = (1 << 11),
    LM_GENERATE_RAND_16BYTES_1_FUN_BIT                      = (1 << 12),
    LM_GEN_LOCAL_OOB_DATA_1_FUN_BIT                         = (1 << 13),
    LM_GEN_P192_PUB_KEY_COMPLETED_1_FUN_BIT                 = (1 << 14),
    LM_GEN_P256_PUB_KEY_COMPLETED_1_FUN_BIT                 = (1 << 15),
    LM_GETLOCALNAMESEG_1_FUN_BIT                            = (1 << 16),
    LM_GETPINTYPE_1_FUN_BIT                                 = (1 << 17),
    LM_GET_AUTH_EN_1_FUN_BIT                                = (1 << 18),
    LM_GET_HOST_MAX_SLOT_1_FUN_BIT                          = (1 << 19),
    LM_GET_LOOPBACK_MODE_1_FUN_BIT                          = (1 << 20),
    LM_GET_NB_ACL_1_FUN_BIT                                 = (1 << 21),
    LM_GET_SEC_CON_HOST_SUPP_1_FUN_BIT                      = (1 << 22),
    LM_GET_SP_EN_1_FUN_BIT                                  = (1 << 23),
    LM_INIT_1_FUN_BIT                                       = (1 << 24),
    LM_IS_ACL_CON_1_FUN_BIT                                 = (1 << 25),
    LM_IS_ACL_CON_ROLE_1_FUN_BIT                            = (1 << 26),
    LM_LOCAL_EXT_FR_CONFIGURED_1_FUN_BIT                    = (1 << 27),
    LM_LOOK_FOR_STORED_LINK_KEY_1_FUN_BIT                   = (1 << 28),
    LM_LT_ADDR_ALLOC_1_FUN_BIT                              = (1 << 29),
    LM_LT_ADDR_FREE_1_FUN_BIT                               = (1 << 30),
    LM_LT_ADDR_RESERVE_1_FUN_BIT                            = (1 << 31),
};

enum lm_2_patch_fields
{
    LM_PAGE_SCAN_REP_MODE_GET_2_FUN_BIT                     = (1 << 0),
    LM_READ_FEATURES_2_FUN_BIT                              = (1 << 1),
    LM_ROLE_SWITCH_FINISHED_2_FUN_BIT                       = (1 << 2),
    LM_ROLE_SWITCH_START_2_FUN_BIT                          = (1 << 3),
    LM_SAM_PATTERN_GET_2_FUN_BIT                            = (1 << 4),
    LM_SAM_SUBMAP0_GET_2_FUN_BIT                            = (1 << 5),
    LM_SAM_SUBMAP_RX_SLOTS_GET_2_FUN_BIT                    = (1 << 6),
    LM_SAM_SUBMAP_TX_SLOTS_GET_2_FUN_BIT                    = (1 << 7),
    LM_SCO_MOVE_EN_2_FUN_BIT                                = (1 << 8),
    LM_SP_DEBUG_MODE_GET_2_FUN_BIT                          = (1 << 9),
    LM_SP_GET_LOCAL_OOB_DATA_2_FUN_BIT                      = (1 << 10),
    LM_SP_RELEASE_LOCAL_OOB_DATA_2_FUN_BIT                  = (1 << 11),
    LM_SYNC_FLOW_CTRL_EN_GET_2_FUN_BIT                      = (1 << 12),
};

enum proc_continue_patch_fields
{
    LLC_DISCONNECT_PROC_CONTINUE_C_FUN_BIT                = (1 << 0),
    LLC_LOC_CTE_PROC_CONTINUE_C_FUN_BIT                   = (1 << 1),
    LLC_LOC_ENCRYPT_PROC_CONTINUE_C_FUN_BIT               = (1 << 2),
    LLC_REM_ENCRYPT_PROC_CONTINUE_C_FUN_BIT               = (1 << 3),
    LLC_LE_PING_PROC_CONTINUE_C_FUN_BIT                   = (1 << 4),
    LLC_PWR_CTRL_PROC_CONTINUE_C_FUN_BIT                  = (1 << 5),
    LLC_LOC_CH_MAP_PROC_CONTINUE_C_FUN_BIT                = (1 << 6),
    LLC_REM_CH_MAP_PROC_CONTINUE_C_FUN_BIT                = (1 << 7),
    LLC_CIS_LOC_CREATE_PROC_CONTINUE_C_FUN_BIT            = (1 << 8),
    LLC_CIS_LOC_STOP_PROC_CONTINUE_C_FUN_BIT              = (1 << 9),
    LLC_CIS_REM_CREATE_PROC_CONTINUE_C_FUN_BIT            = (1 << 10),
    LLC_LOC_CLK_ACC_PROC_CONTINUE_C_FUN_BIT               = (1 << 11),
    LLC_LOC_CON_UPD_PROC_CONTINUE_C_FUN_BIT               = (1 << 12),
    LLC_REM_CON_UPD_PROC_CONTINUE_C_FUN_BIT               = (1 << 13),
    LLC_LOC_DL_UPD_PROC_CONTINUE_C_FUN_BIT                = (1 << 14),
    LLC_LOC_FEATS_EXCH_PROC_CONTINUE_C_FUN_BIT            = (1 << 15),
    LLC_LOC_PHY_UPD_PROC_CONTINUE_C_FUN_BIT               = (1 << 16),
    LLC_REM_PHY_UPD_PROC_CONTINUE_C_FUN_BIT               = (1 << 17),
    LLC_VER_EXCH_LOC_PROC_CONTINUE_C_FUN_BIT              = (1 << 18),
    LLC_PROC_ERR_IND_C_FUN_BIT                            = (1 << 19),
    LLC_PROC_COLLISION_CHECK_C_FUN_BIT                    = (1 << 20),
};

enum data_path_patch_fields
{
    DATA_PATH_CONFIG_FUN_BIT                    = (1 << 0),
    DATA_PATH_INIT_FUN_BIT                      = (1 << 1),
    DATA_PATH_IS_DISABLED_FUN_BIT               = (1 << 2),
    DATA_PATH_ITF_GET_FUN_BIT                   = (1 << 3),
};

extern uint32_t g_patch_type[PATCH_TYPE_MAX];
#define  DIRECT_RETURN   0
#define  CONTINUE_RETURN   1

typedef void (*patch_no_parm_no_return_t)(uint32_t type, uint32_t bitmask, uint8_t *ret);
typedef uint32_t (*patch_no_parm_have_return_t)(uint32_t type, uint32_t bitmask, uint8_t *ret);
typedef void (*patch_1_parm_no_return_t)(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1);
typedef uint32_t (*patch_1_parm_have_return_t)(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1);
typedef void (*patch_2_parm_no_return_t)(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, uint32_t para2);
typedef uint32_t (*patch_2_parm_have_return_t)(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, uint32_t para2);
typedef void (*patch_3_parm_no_return_t)(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, uint32_t para2, uint32_t para3);
typedef uint32_t (*patch_3_parm_have_return_t)(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, uint32_t para2, uint32_t para3);
typedef void (*patch_4_parm_no_return_t)(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, uint32_t para2, uint32_t para3, uint32_t para4);
typedef uint32_t (*patch_4_parm_have_return_t)(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, uint32_t para2, uint32_t para3, uint32_t para4);
typedef void (*patch_vari_parm_no_return_t)(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, ...);
typedef uint32_t (*patch_vari_parm_have_return_t)(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, ...);



extern void     patch_no_parm_no_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret);
extern uint32_t patch_no_parm_have_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret);
extern void patch_1_parm_no_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1);
extern uint32_t patch_1_parm_have_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1);
extern void patch_2_parm_no_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, uint32_t para2);
extern uint32_t patch_2_parm_have_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, uint32_t para2);
extern void patch_3_parm_no_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, uint32_t para2, uint32_t para3);
extern uint32_t patch_3_parm_have_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, uint32_t para2, uint32_t para3);
extern void patch_4_parm_no_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, uint32_t para2, uint32_t para3, uint32_t para4);
extern uint32_t patch_4_parm_have_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, uint32_t para2, uint32_t para3, uint32_t para4);
extern void patch_vari_parm_no_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, ...);
extern uint32_t patch_vari_parm_have_return_handler(uint32_t type, uint32_t bitmask, uint8_t *ret, uint32_t para1, ...);


extern patch_no_parm_no_return_t     patch_no_parm_no_return   ;
extern patch_no_parm_have_return_t   patch_no_parm_have_return ;
extern patch_1_parm_no_return_t      patch_1_parm_no_return    ;
extern patch_1_parm_have_return_t    patch_1_parm_have_return  ;
extern patch_2_parm_no_return_t      patch_2_parm_no_return    ;
extern patch_2_parm_have_return_t    patch_2_parm_have_return  ;
extern patch_3_parm_no_return_t      patch_3_parm_no_return    ;
extern patch_3_parm_have_return_t    patch_3_parm_have_return  ;
extern patch_4_parm_no_return_t      patch_4_parm_no_return    ;
extern patch_4_parm_have_return_t    patch_4_parm_have_return  ;
extern patch_vari_parm_no_return_t      patch_vari_parm_no_return    ;
extern patch_vari_parm_have_return_t    patch_vari_parm_have_return  ;




#define FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(type, bitmask) \
     do{ \
         if(g_patch_type[type] & bitmask){ \
             uint8_t ret; \
             patch_no_parm_no_return(type, bitmask, &ret);\
             if(ret == DIRECT_RETURN){\
                 return;}\
         }\
     }while(0)

#define FUNC_PATCH_ENTRY_NO_PARAM_HAVE_RETURN(type, bitmask, ret_type) \
     do{ \
         if(g_patch_type[type] & bitmask){ \
             uint8_t ret; \
             uint32_t ret_param;\
             ret_param = patch_no_parm_have_return(type, bitmask, &ret);\
             if(ret == DIRECT_RETURN){\
                 return (ret_type)ret_param;}\
         }\
     }while(0)

#define FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(type, bitmask, param1) \
         do{ \
             if(g_patch_type[type] & bitmask){ \
                 uint8_t ret; \
                 patch_1_parm_no_return(type, bitmask, &ret, (uint32_t)param1);\
                 if(ret == DIRECT_RETURN){\
                     return;}\
             }\
         }while(0)

#define FUNC_PATCH_ENTRY_1_PARAM_HAVE_RETURN(type, bitmask, ret_type, param1) \
         do{ \
             if(g_patch_type[type] & bitmask){ \
                 uint8_t ret; \
                 uint32_t ret_param;\
                 ret_param = patch_1_parm_have_return(type, bitmask, &ret, (uint32_t)param1);\
                 if(ret == DIRECT_RETURN){\
                     return (ret_type)ret_param;}\
             }\
         }while(0)

#define FUNC_PATCH_ENTRY_2_PARAM_NO_RETURN(type, bitmask, param1, param2) \
        do{ \
          if(g_patch_type[type] & bitmask){ \
              uint8_t ret; \
              patch_2_parm_no_return(type, bitmask, &ret, (uint32_t)param1, (uint32_t)param2);\
              if(ret == DIRECT_RETURN){\
                  return;}\
          }\
        }while(0)

#define FUNC_PATCH_ENTRY_2_PARAM_HAVE_RETURN(type, bitmask, ret_type, param1, param2) \
          do{ \
              if(g_patch_type[type] & bitmask){ \
                  uint8_t ret; \
                  uint32_t ret_param;\
                  ret_param = patch_2_parm_have_return(type, bitmask, &ret, (uint32_t)param1, (uint32_t)param2);\
                  if(ret == DIRECT_RETURN){\
                      return (ret_type)ret_param;}\
              }\
          }while(0)

#define FUNC_PATCH_ENTRY_3_PARAM_NO_RETURN(type, bitmask, param1, param2, param3) \
          do{ \
            if(g_patch_type[type] & bitmask){ \
                uint8_t ret; \
                patch_3_parm_no_return(type, bitmask, &ret, (uint32_t)param1, (uint32_t)param2, (uint32_t)param3);\
                if(ret == DIRECT_RETURN){\
                    return;}\
            }\
          }while(0)

#define FUNC_PATCH_ENTRY_3_PARAM_HAVE_RETURN(type, bitmask, ret_type, param1, param2, param3) \
        do{ \
            if(g_patch_type[type] & bitmask){ \
                uint8_t ret; \
                uint32_t ret_param;\
                ret_param = patch_3_parm_have_return(type, bitmask, &ret, (uint32_t)param1, (uint32_t)param2, (uint32_t)param3);\
                if(ret == DIRECT_RETURN){\
                    return (ret_type)ret_param;}\
            }\
        }while(0)

#define FUNC_PATCH_ENTRY_4_PARAM_NO_RETURN(type, bitmask, param1, param2, param3, param4) \
          do{ \
            if(g_patch_type[type] & bitmask){ \
                uint8_t ret; \
                patch_4_parm_no_return(type, bitmask, &ret, (uint32_t)param1, (uint32_t)param2, (uint32_t)param3, (uint32_t)param4);\
                if(ret == DIRECT_RETURN){\
                    return;}\
            }\
          }while(0)

#define FUNC_PATCH_ENTRY_4_PARAM_HAVE_RETURN(type, bitmask, ret_type, param1, param2, param3, param4) \
        do{ \
            if(g_patch_type[type] & bitmask){ \
                uint8_t ret; \
                uint32_t ret_param;\
                ret_param = patch_4_parm_have_return(type, bitmask, &ret, (uint32_t)param1, (uint32_t)param2, (uint32_t)param3, (uint32_t)param4);\
                if(ret == DIRECT_RETURN){\
                    return (ret_type)ret_param;}\
            }\
        }while(0)

#define FUNC_PATCH_ENTRY_VARI_PARAM_NO_RETURN(type, bitmask, param1, ...) \
                         do{ \
                             if(g_patch_type[type] & bitmask){ \
                                 uint8_t ret; \
                                 patch_vari_parm_no_return(type, bitmask, &ret, (uint32_t)param1, __VA_ARGS__);\
                                 if(ret == DIRECT_RETURN){\
                                     return;}\
                             }\
                         }while(0)

#define FUNC_PATCH_ENTRY_VARI_PARAM_HAVE_RETURN(type, bitmask, ret_type, param1, ...) \
                         do{ \
                             if(g_patch_type[type] & bitmask){ \
                                 uint8_t ret; \
                                 uint32_t ret_param;\
                                 ret_param = patch_vari_parm_have_return(type, bitmask, &ret, (uint32_t)param1, __VA_ARGS__);\
                                 if(ret == DIRECT_RETURN){\
                                     return (ret_type)ret_param;}\
                             }\
                         }while(0)

#endif


