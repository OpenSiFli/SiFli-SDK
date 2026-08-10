/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 ****************************************************************************************
 *
 * @file fhost_tx.h
 *
 * @brief Definitions of the fully hosted TX task.
 *
 * Copyright (C) RivieraWaves 2017-2019
 *
 ****************************************************************************************
 */

#ifndef _FHOST_TX_H_
#define _FHOST_TX_H_

/**
 ****************************************************************************************
 * @defgroup FHOST_TX FHOST_TX
 * @ingroup FHOST
 * @brief Fully Hosted TX task implementation.
 * This module creates a task that interfaces between the networking stack and the WiFi
 * task. It is responsible for the assignment of the packets to the TX queues
 * and for the scheduling of these queues.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "co_list.h"
#include "tx_swdesc.h"
#include "fhost.h"
#include "netif_port.h"

/*
 * DEFINITIONS
 ****************************************************************************************
 */

#define TX_WAIT_FW_BUFFER_TIME_MS      20

/// Number of TX flow control credits allocated by default per RA/TID (UMAC only)
#define NX_DEFAULT_TX_CREDIT_CNT 4
/// Number of elements in the TX message queue
#define FHOST_TX_QUEUE_MSG_ELT_CNT     64

/// Frame transmission done
#define TX_STATUS_DONE                 CO_BIT(0)
/// Frame retry required
#define TX_STATUS_RETRY_REQUIRED       CO_BIT(1)
/// Frame sw retry required
#define TX_STATUS_SW_RETRY_REQUIRED    CO_BIT(2)
/// Frame acknoledged
#define TX_STATUS_ACKNOWLEDGED         CO_BIT(3)

#define TXU_CNTRL_RETRY        BIT(0)
#define TXU_CNTRL_MORE_DATA    BIT(2)
#define TXU_CNTRL_MGMT         BIT(3)
#define TXU_CNTRL_MGMT_NO_CCK  BIT(4)
#define TXU_CNTRL_AMSDU        BIT(6)
#define TXU_CNTRL_MGMT_ROBUST  BIT(7)
#define TXU_CNTRL_USE_4ADDR    BIT(8)
#define TXU_CNTRL_EOSP         BIT(9)
#define TXU_CNTRL_MESH_FWD     BIT(10)
#define TXU_CNTRL_TDLS         BIT(11)
/// This frame is postponed internally because of PS. (only for AP)
#define TXU_CNTRL_POSTPONE_PS   BIT(12)
/// Internal flag indicating that this packet should use the trial rate as first or second rate
#define TXU_CNTRL_RC_TRIAL      BIT(13)
/// Internal flag indicating that this is a UAPSD trigger frame
#define TXU_CNTRL_UASPD_TRIGGER BIT(14)
//add by aic
#define TXU_CNTRL_AMPDU_MPDU_RETRY  BIT(15)

/// Stored in hostdesc.status_desc_addr
#define TXU_CNTRL_NEED_CFM      BIT(31)

/// IDs of messages received by the FHOST TX task
enum fhost_tx_msg_id
{
    /// Indicate that a STA is connected
    FHOST_TX_STA_ADD = 0,
    /// Indicate that a STA is disconnected
    FHOST_TX_STA_DEL,
    /// Indicate that the credits for a TX queue are updated
    FHOST_TX_CREDITS_UPDATE,
    /// Enable TXQ associated to a VIF
    FHOST_TX_VIF_ENABLE,
    /// Disable TXQ associated to a VIF
    FHOST_TX_VIF_DISABLE,
    /// Indicate that a STA enabled PowerSave mode
    FHOST_TX_STA_ENTER_PS,
    /// Indicate that a STA disabled PowerSave mode
    FHOST_TX_STA_EXIT_PS,
    /// Request trafiic for STA in PS mode
    FHOST_TX_PS_TRAFFIC_REQ,
};

/// TXQ status flag
enum fhost_txq_flags {
    /// The queue is scheduled for transmission
    TXQ_IN_HWQ_LIST  = CO_BIT(0),
    /// No more credits for the queue
    TXQ_STOP_FULL    = CO_BIT(1),
    /// CSA is in progress
    TXQ_STOP_CSA     = CO_BIT(2),
    /// Destination sta is currently in power save mode
    TXQ_STOP_STA_PS  = CO_BIT(3),
    /// VIF owning this queue is currently in power save mode
    TXQ_STOP_VIF_PS  = CO_BIT(4),
    /// Channel of this queue is not the current active channel
    TXQ_STOP_CHAN    = CO_BIT(5),
    TXQ_STOP_TIMER   = CO_BIT(6),
    /// All possible reasons to have a txq stopped
    TXQ_STOP         = (TXQ_STOP_FULL | TXQ_STOP_CSA | TXQ_STOP_STA_PS | TXQ_STOP_VIF_PS |
                        TXQ_STOP_CHAN | TXQ_STOP_TIMER),
    /// TX queue is enabled, i.e. meaning that it can be used for transmission
    TXQ_ENABLED      = CO_BIT(7),
};

/// Type of PS traffic
enum fhost_tx_ps_type
{
    /// Legacy power save
    PS_LEGACY,
    /// UAPSD
    PS_UAPSD,
    /// Number of PS traffic
    PS_TYPE_CNT,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
/// Structure used for the inter-task communication
struct fhost_tx_msg_tag
{
    /// Message ID (from @ref fhost_tx_msg_id)
    enum fhost_tx_msg_id msg_id;
    /// Message structure
    union
    {
        /// Add station
        struct
        {
            /// Id of the station to add
            uint8_t sta_id;
        } sta_add;
        /// Delete station
        struct
        {
            /// Id of the station to delete
            uint8_t sta_id;
        } sta_del;
        /// Update credits
        struct
        {
            /// Id of the station for which credits are updated
            uint8_t sta_id;
            /// TID for which credits are updated
            uint8_t tid;
            /// Credits
            int8_t credits;
        } credits_upd;
    } u;
};


/// Structure mapped into the TX buffer for internal handling
struct fhost_tx_desc_tag
{
    /// Chained list element
    struct co_list_hdr hdr;
    /// TX descriptor passed to MAC
    struct txdesc txdesc;
    uint8_t pbd_cnt;
};

/// TX queue structure
struct fhost_tx_queue_tag
{
    /// Next TX queue element (used to chain the TX queue to the scheduler)
    struct fhost_tx_queue_tag *next;
    /// Next TX queue element (used to chain the TX queue to the scheduler)
    struct fhost_tx_queue_tag *prev;
    /// List of TX buffers ready for transmission (@ref fhost_tx_desc_tag.hdr)
    struct co_list ready;
    /// Status of the queue - see @ref fhost_txq_flags
    uint8_t status;
    /// Number of credits currently available in this queue
    int8_t credits;
    #if NX_BEACONING
    /// TXQ PS type (@ref fhost_tx_ps_type)
    int8_t ps_type;
    /// Number of packet queued in 'ready' list
    uint16_t nb_ready;
    /// Number of packet to push to complete PS service period
    uint16_t limit;
    #endif //NX_BEACONING
};

/// TX queue scheduler structure
struct fhost_tx_queue_sched_tag
{
    /// Current TX queue element being scheduled
    struct fhost_tx_queue_tag *current;
};
/// Traffic information for one station
struct fhost_tx_sta_traffic
{
    /// Number of packet ready for PS service period among all STA's TXQ
    /// (-1 if station is not in PS mode)
    int ps_ready[PS_TYPE_CNT];
    /// Number of packet to push to complete the PS service period
    int sp_cnt[PS_TYPE_CNT];
};

/**
 * Number of TXQ allocated
 *
 * Currently we have:
 * - One TXQ per TID and per STA:
 *   TXQ id is @verbatim  <sta_id> * TID_MAX + <tid> @endverbatim
 * - One TXQ for unknown STA for each VIF (Only enabled for AP and Monitor interface)
 *   TXQ id is @verbatim (NX_REMOTE_STA_MAX * TID_MAX) + <vif_idx> @endverbatim
 */
#if NX_P2P
#define FHOST_TXQ_CNT ((STA_MAX * TID_MAX) + NX_VIRT_DEV_MAX + 1)
#define FHOST_OFF_CHAN_TXQ_IDX ((STA_MAX * TID_MAX) + NX_VIRT_DEV_MAX)
#else
#define FHOST_TXQ_CNT ((STA_MAX * TID_MAX) + NX_VIRT_DEV_MAX)
#endif

/// FHOST RX environment structure
struct fhost_tx_env_tag
{
    /// Queues used for the transmission requests sent to the WiFi task
    //rtos_queue queue_tx[NX_TXQ_CNT];
    /// Queue used to allow other tasks interacting with the FHOST TX task (confirmations
    /// from the WiFi task, transmission requests from the networking stack, etc.)
    rtos_queue queue_msg;
    /// Internal per-STA/TID TX queues
    struct fhost_tx_queue_tag tx_q[FHOST_TXQ_CNT];
    /// TX queue scheduler structures (one per HW queue)
    struct fhost_tx_queue_sched_tag sched[NX_TXQ_CNT];
    /// Semaphore used for TX locking
    rtos_mutex tx_lock;
    rtos_mutex txq_lock;
    rtos_mutex tx_ac_lock;
    struct co_list tx_ac[AC_MAX];
    uint32_t tx_ac_count;
#if NX_BEACONING
    /// Traffic information for each station(only used by AP interface)
    struct fhost_tx_sta_traffic traffic[STA_MAX];
#endif
    uint8_t       to_times;
    uint8_t tx_task_terminate;

};

/// FHOST STA PS environment structure
struct fhost_tx_sta_ps_env_tag
{
    /// True when the sta is in PS mode, If false, other values should be ignored.
    uint8_t ps[NX_REMOTE_STA_MAX];
    /// Number of packets buffered for the sta in fhost's txq(1 counter for Legacy PS & 1 for U-APSD)
    uint16_t pkt_ready[2];
    /// Number of packets that remain to be pushed in the service period.
    /// 0 means that no service period is in progress(1 counter for Legacy PS & 1 for U-APSD)
    uint16_t sp_cnt[2];
};

#define LEGACY_PS_ID  (0)
#define UAPSD_ID      (1)

/// Macro used to lock the TX mutex
#define FHOST_TX_LOCK()    rtos_mutex_lock(fhost_tx_env.tx_lock, -1)
/// Macro used to unlock the TX mutex
#define FHOST_TX_UNLOCK()  rtos_mutex_unlock(fhost_tx_env.tx_lock)

/// Macro used to lock the TXQ
#define FHOST_TXQ_LOCK()    rtos_mutex_lock(fhost_tx_env.txq_lock, -1)
/// Macro used to unlock the TX mutex
#define FHOST_TXQ_UNLOCK()  rtos_mutex_unlock(fhost_tx_env.txq_lock)

/// Macro used to lock the TX AC
#define FHOST_TX_AC_LOCK()    rtos_mutex_lock(fhost_tx_env.tx_ac_lock, -1)
/// Macro used to unlock the TX mutex
#define FHOST_TX_AC_UNLOCK()  rtos_mutex_unlock(fhost_tx_env.tx_ac_lock)

/// Macro defining an invalid VIF index
#define INVALID_VIF_IDX 0xFF
#define INVALID_STA_IDX 0xFF

/*
 * GLOBAL VARIABLES
 ****************************************************************************************
 */
/// FHOST RX environment
extern struct fhost_tx_env_tag fhost_tx_env;
extern struct co_list tx_desc_free_list;
extern struct co_list tx_desc_cfm_list;
extern uint32_t tx_desc_free_cnt;
extern uint32_t tx_desc_cfm_cnt;

/*
 * FUNCTIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * @brief Initialization of the TX task.
 * This function initializes the different data structures used for the TX and creates the
 * RTOS task dedicated to the TX processing.
 *
 * @return 0 on success and != 0 if error occurred.
 ****************************************************************************************
 */
int fhost_tx_init(void);

/**
 ****************************************************************************************
 * @brief Push a buffer for transmission.
 *
 * The buffer is directly pushed with the TX mutex hold.
 *
 * @param[in] net_if      Pointer to the net interface for which the packet is pushed
 * @param[in] net_buf     Pointer to the net buffer to transmit.
 * @param[in] cfm_cb      Callback when transmission has been completed (may be NULL)
 * @param[in] cfm_cb_arg  Private argument for the callback.
 *
 * @return 0 on success and != 0 if error occurred.
 ****************************************************************************************
 */
int fhost_tx_start(net_if_t *net_if, net_buf_tx_t *net_buf,
                   cb_fhost_tx cfm_cb, void *cfm_cb_arg);

/**
 ****************************************************************************************
 * @brief Push a TX confirmation to the FHOST TX task
 *
 * @param[in] statinfo TX status information
 *
 ****************************************************************************************
 */
void fhost_tx_cfm_push(uint32_t *data);

/**
 ****************************************************************************************
 * @brief Update the credits of a TX queue, upon the creation or deletion of a BlockAck
 * agreement of the association STA/TID pair.
 *
 * @param[in] sta_id Index of the station
 * @param[in] tid Priority
 * @param[in] credits Number of credits to add/remove on the TX queue
 ****************************************************************************************
 */
void fhost_tx_credits_update(uint8_t sta_id, uint8_t tid, int8_t credits);

/**
 ****************************************************************************************
 * @brief Enable the TX queues for the given STA.
 *
 * Send message @ref FHOST_TX_STA_ADD to FHOST TX thread to enable the TX queues
 * associated to this STA.
 *
 * @param[in] sta_id Station index to add
 ****************************************************************************************
 */
void fhost_tx_sta_add(uint8_t sta_id);

/**
 ****************************************************************************************
 * @brief Disable the TX queues for the given STA.
 *
 * Send message @ref FHOST_TX_STA_DEL to FHOST TX thread to disable the TX queues
 * associated to this STA. Any pending packets in the queues will be freed.
 *
 * @param[in] sta_id Station index to delete
 ****************************************************************************************
 */
void fhost_tx_sta_del(uint8_t sta_id);
/// @}
void fhost_tx_enqueue_cfm_list(struct fhost_tx_desc_tag *desc);
struct fhost_tx_desc_tag *fhost_tx_dequeue_cfm_list(void);
uint32_t fhost_tx_cfm_list_cnt(bool lock);
void fhost_tx_enqueue_free_list(struct fhost_tx_desc_tag *desc);
struct fhost_tx_desc_tag *fhost_tx_dequeue_free_list(bool need);
uint32_t fhost_tx_free_list_cnt(bool lock);
void fhost_tx_desc_netbuf_free(struct fhost_tx_desc_tag *desc);
void fhost_tx_do_sta_add(uint8_t sta_id);
void fhost_tx_do_sta_del(uint8_t sta_id);
void fhost_tx_flush_txq(uint8_t sta_id);
void fhost_tx_ac_flush_sta(struct mac_addr *sta_addr);


int fhost_tx_sta_ps_enable(uint8_t sta_id, bool enable);
void fhost_tx_do_ps_traffic_req(uint8_t sta_id, int req_cnt, enum fhost_tx_ps_type ps_type);
void fhost_tx_do_vif_disable(uint8_t vif_idx);
void fhost_tx_vif_txq_enable(struct fhost_vif_tag *fhost_vif);
void fhost_txq_vif_stop(uint8_t sta_id, uint16_t reason);
void fhost_txq_vif_start(uint8_t sta_id, uint16_t reason);
#if NX_P2P
struct fhost_tx_queue_tag* fhost_txq_offchan_init(void);
void fhost_txq_offchan_deinit(struct fhost_tx_queue_tag* txq);
#endif

int fhost_tx_cntrl_by_timer(uint32_t timeout_ms, uint8_t num);
int fhost_stop_tx_cntrl_by_timer(void);
uint8_t fhost_tx_get_to_times(void);
int fhost_tx_task_init(void);
void fhost_tx_task_deinit(void);
int fhost_tx_deinit(void);

#endif // _FHOST_TX_H_