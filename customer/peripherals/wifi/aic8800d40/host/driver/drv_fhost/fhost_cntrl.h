/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ****************************************************************************************
 *
 * @file fhost_cntrl.h
 *
 * @brief Definition of control task for Fully Hosted firmware.
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */
#ifndef _FHOST_CNTRL_H_
#define _FHOST_CNTRL_H_

/**
 ****************************************************************************************
 * @defgroup FHOST_CNTRL FHOST_CNTRL
 * @ingroup FHOST
 * @brief Control task for fully hosted firmware
 *
 * @{
 ****************************************************************************************
 */

#include "cfgrwnx.h"
#include "mac.h"

enum lowpower_level {
    LP_LEVEL_NONE       = 0,
    LP_LEVEL_CLOCK_GATE = 1,
    LP_LEVEL_POWER_OFF  = 2,
};

struct cfgrwnx_msg_hdr;

/// Structure containing parameters to establish link with cntrl
struct fhost_cntrl_link
{
    /// CFGRWNX queue to get response from CNTRL task
    rtos_queue queue;
    /// Socket, for CNTRL task, to send events
    int sock_send;
    /// Socket to receive events from CNTRL task
    int sock_recv;
};

typedef struct {
    cb_fhost_tx cb_func;
    uint32_t frame_id;
    bool acknowledged;
    void *arg;
} tx_cfm_callback_t;

/// Max number of default keys for given VIF
#define MAC_DEFAULT_KEY_COUNT 4
/// Max number of MFP key for given VIF
#define MAC_DEFAULT_MFP_KEY_COUNT 6

/// Number of supported Default+Pairwise keys
#if 0 //NX_KEY_RAM_CONFIG
#define MM_SEC_MAX_KEY_NBR      (nxmac_sta_key_max_index_getf() + 1)
#else
#define MM_SEC_MAX_KEY_NBR      64
#endif

/// Number of VLAN Id
#if 0 //NX_KEY_RAM_CONFIG
#define MM_SEC_VLAN_COUNT (NX_VIRT_DEV_MAX + RW_MESH_LINK_NB)
#else
#define MM_SEC_VLAN_COUNT NX_VIRT_DEV_MAX
#endif

/// Number of default keys
#define MM_SEC_DEFAULT_KEY_COUNT (MAC_DEFAULT_KEY_COUNT * MM_SEC_VLAN_COUNT)

/// Macro converting a default key index to its HW index in key memory
#define MM_VIF_TO_KEY(key_idx, vif_idx)     ((key_idx) + ((vif_idx) * MAC_DEFAULT_KEY_COUNT))
/// Macro converting a STA index to the its HW pairwise key index in key memory
#define MM_STA_TO_KEY(sta_idx)              ((sta_idx) + MM_SEC_DEFAULT_KEY_COUNT)

#if NX_MFP
/// Number of MFP keys per VIF
#define MM_SEC_MFP_KEY_COUNT (MAC_DEFAULT_MFP_KEY_COUNT - MAC_DEFAULT_KEY_COUNT)
/// Macro converting mfp key (0/1) and vif index to HW key index
#define MM_VIF_TO_MFP_KEY(key_idx, vif_idx) ((((key_idx) - MAC_DEFAULT_KEY_COUNT) + ((vif_idx) * MM_SEC_MFP_KEY_COUNT)) + MM_SEC_MAX_KEY_NBR)
/// Macro converting a hw_key_idx, in range [@ref MM_SEC_MAX_KEY_NBR @ref MM_SEC_MAX_MFP_KEY_NBR], to the Vif Index it corresponds
#define MM_MFP_KEY_TO_VIF(hw_key_idx) (((hw_key_idx) - MM_SEC_MAX_KEY_NBR) / MM_SEC_MFP_KEY_COUNT)
#endif //(NX_MFP)

/**
 ****************************************************************************************
 * @brief Send a message to the Control TASK.
 *
 * The message pushed to the control task will only contains pointer to the actual
 * message, thus memory pointed by @p msg_data must not be modified until control task
 * process the message.
 *
 * For now:
 * - for @ref FHOST_MSG_KE_WIFI messages, control task will free (with ke_free)
 * them once processed.
 * - for @ref FHOST_MSG_CFGRWNX messages, it is assumed that supplicant will always
 * wait for a response from control task before re-using/freeing message memory.
 *
 * @param[in] id       ID of message (type(@ref fhost_msg_type) + index).
 * @param[in] msg_data Buffer containing the message.
 * @param[in] len      Length, in bytes, of the message.
 * @param[in] timeout  Maximum duration, in ms, to wait if queue is full.
 *                     0 means do not wait and -1 means wait indefinitely.
 * @param[in] isr      Indicates if function is called from ISR.
 *
 * @return 0 on success and != 0 if error occurred.
 ****************************************************************************************
 */
int fhost_cntrl_write_msg(int id, void *msg_data, int len, int timeout, bool isr);

/**
 ****************************************************************************************
 * @brief Send a CFGRWNX command to the Control TASK and get the response.
 *
 * @param[in]     cmd    Pointer to the command header (followed by the parameters)
 * @param[in,out] resp   Pointer to the response header (followed by the space for the
 *                       response parameters). This parameter can be set to NULL if no
 *                       response is expected.
 *
 * @return 0 on success and != 0 if error occurred.
 ****************************************************************************************
 */
int fhost_cntrl_cfgrwnx_cmd_send(struct cfgrwnx_msg_hdr *cmd,
                                 struct cfgrwnx_msg_hdr *resp);

/*
* Inform fhost cntrl task to process txcfm cb msg
*/
int fhost_cntrl_txcfm_cb_msg_send(tx_cfm_callback_t *cb);

/**
 ****************************************************************************************
 * @brief Send a CFGRWNX event to the specified socket
 *
 * Event will be entirely copied to the socket so memory pointed by msg can be re-used
 * after calling this function.
 *
 * @param[in] msg_hdr  Pointer on the header of the event to send.
 * @param[in] sock     Socket to send CFGRWNX events
 *
 * @return 0 on success and != 0 if error occurred.
 ****************************************************************************************
 */
int fhost_cntrl_cfgrwnx_event_send(struct cfgrwnx_msg_hdr *msg_hdr, int sock);

/**
 ****************************************************************************************
 * @brief Wait until new event is available.
 *
 * This function is blocking until a new event is sent by the CNTRL task on the specified
 * link. This function is used to get the event ID and length without consuming it and
 * then @ref fhost_cntrl_cfgrwnx_event_get can be called with a buffer big enough to
 * retrieve the complete event or @ref fhost_cntrl_cfgrwnx_event_discard to discard it.
 *
 * @param[in]  link     Link with CNTRL task to use
 * @param[out] msg_hdr  Event header
 *
 * @return 0 on success and != 0 if error occurred.
 ****************************************************************************************
 */
int fhost_cntrl_cfgrwnx_event_peek_header(struct fhost_cntrl_link *link,
                                          struct cfgrwnx_msg_hdr *msg_hdr);

/**
 ****************************************************************************************
 * @brief Read oldest event available
 *
 * This function doesn't wait and it is meant to be called after @ref
 * fhost_cntrl_cfgrwnx_event_peek_header once the event ID and length are known.
 *
 * @param[in]  link   Link with CNTRL task to use
 * @param[out] event  Event buffer
 * @param[in]  len    Length, in bytes, of the @p event buffer
 *
 * @return Number of bytes written on success and < 0 if error occurred.
 ****************************************************************************************
 */
int fhost_cntrl_cfgrwnx_event_get(struct fhost_cntrl_link *link, void *event, int len);

/**
 ****************************************************************************************
 * @brief Discard oldest event available.
 *
 * This function doesn't wait and it is meant to be called after @ref
 * fhost_cntrl_cfgrwnx_event_peek_header once the event ID is known.
 *
 * @param[in] link     Link with CNTRL task to use
 * @param[in] msg_hdr  Event header as returned by @ref fhost_cfgrwnx_event_peek_header
 *
 * @return 0 on success and != 0 if error occurred.
 ****************************************************************************************
 */
int fhost_cntrl_cfgrwnx_event_discard(struct fhost_cntrl_link *link,
                                      struct cfgrwnx_msg_hdr *msg_hdr);

/**
 ****************************************************************************************
 * @brief Change type of the MAC vif associated to a FHOST vif.
 *
 * Send CFGRWNX_SET_VIF_TYPE_CMD command to CNTRL task and wait for the response.
 * This will always delete the current MAC vif (if it exists) and create a new one
 * (unless type == VIF_UNKNOWN).
 *
 * @param[in] link           Link with CNTRL task to use
 * @param[in] fhost_vif_idx  Index of the FHOST vif
 * @param[in] type           Type to set on MAC vif
 * @param[in] p2p            P2P flag
 *
 * @return 0 on success and != 0 if error occurred.
 ****************************************************************************************
 */
int fhost_cntrl_set_mac_vif_type(struct fhost_cntrl_link *link, int fhost_vif_idx,
                                 enum mac_vif_type type, bool p2p);

#if NX_FHOST_MONITOR
/**
 ****************************************************************************************
 * @brief Configure a monitor interface
 *
 * Send message to control task to configure a monitor interface. The interface type
 * must have been changed to VIF_MONITOR (using @ref fhost_set_vif_type) before calling
 * this function.
 *
 * @param[in] link           Link with CNTRL task to use
 * @param[in] fhost_vif_idx  Index of the FHOST vif
 * @param[in] cfg            Configuration to apply
 *
 * @return 0 on success and != 0 if error occurred.
 ****************************************************************************************
 */
int fhost_cntrl_monitor_cfg(struct fhost_cntrl_link *link, int fhost_vif_idx,
                            struct fhost_vif_monitor_cfg *cfg);

#endif // NX_FHOST_MONITOR

/**
 ****************************************************************************************
 * @brief Create the Control task and the different queues associated to this task.
 *
 * @return 0 on success and != 0 if error occurred.
 ****************************************************************************************
 */
int fhost_cntrl_init(struct rwnx_hw *rwnx_hw);

void fhost_cntrl_vif_init(int fvif_idx, struct mac_addr *base_mac_addr);

void fhost_cntrl_cfgrwnx_set_tx_power(uint8_t fvif_idx, uint8_t tx_power);
void fhost_cntrl_cfgrwnx_set_fixed_rate(uint8_t sta_idx, uint8_t bw, uint8_t format_idx, uint16_t rate_idx, uint8_t pre_type);
int fhost_cntrl_mm_set_filter(uint32_t value);

int fhost_cntrl_me_set_lp_level(uint8_t lp_level);
void set_deepsleep_param(unsigned int listen_interval, unsigned int dont_wait_bcmc);

int fhost_cntrl_scan_cancel_req(void);
int fhost_cntrl_mm_set_monitor_channel(int fvif_idx, int chan_num, int band_width);

void fhost_cntrl_deinit(void);

/**
 * @}
 */
#endif /* _FHOST_CNTRL_H_ */