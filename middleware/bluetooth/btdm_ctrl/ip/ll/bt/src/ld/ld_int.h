/**
****************************************************************************************
*
* @file ld_int.h
*
* @brief LD Internal API
*
* Copyright (C) RivieraWaves 2009-2015
*
*
****************************************************************************************
*/

#ifndef LD_INT_H_
#define LD_INT_H_

/**
 ****************************************************************************************
 * @addtogroup LD
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#include <stdint.h>         // integer
#include <stdbool.h>        // boolean

#include "ld.h"

/*
 * CONSTANT DEFINITIONS
 ****************************************************************************************
 */

/// BCH size
#define LD_BCH_SIZE                          8

/// Set an ACL link as active
#define LD_ACT_ACL_SET(field, link_id)           (field |= (1<<(link_id)))
/// Clear an ACL link from active links
#define LD_ACT_ACL_CLEAR(field, link_id)         (field &= ~(1<<(link_id)))
/// Set a SCO link as active
#define LD_ACT_SCO_SET(field, sco_link_id, link_id)                                                       \
        field = (((field) & ~(0xFF << (8*(sco_link_id+1)))) | (0x80 + link_id) << (8*(sco_link_id+1)));
/// Clear a SCO link from active links
#define LD_ACT_SCO_CLEAR(field, sco_link_id)     (field &= ~(0xFF << (8*(sco_link_id+1))))

/// Allocate an event with an associated data structure
#define LD_ALLOC_EVT(data_struct)           (struct data_struct *) ke_malloc_system(sizeof(struct data_struct), KE_MEM_ENV);

/// Initialize memory of an event and an associated data structure
#define LD_INIT_EVT(evt, data_struct)       memset(evt , 0, sizeof(struct data_struct));


/// Index for DM1 packet type (used in both BR and EDR)
#define DM1_IDX         0

/// Basic Rate packet types indexes (classified by packet size)
#define DH1_IDX         1
#define DM3_IDX         2
#define DH3_IDX         3
#define DM5_IDX         4
#define DH5_IDX         5
#define BR_IDX_MAX      6

/// Enhanced Data Rate packet types indexes (classified by packet size)
#define DH1_2_IDX       1
#define DH1_3_IDX       2
#define DH3_2_IDX       3
#define DH3_3_IDX       4
#define DH5_2_IDX       5
#define DH5_3_IDX       6
#define EDR_IDX_MAX     7


/*
 * ENUMERATION DEFINITIONS
 ****************************************************************************************
 */

/// Activity type
enum LD_ACT_TYPE
{
    LD_ACT_TYPE_ACL,
    LD_ACT_TYPE_SCO,
};

/// Active mode
enum LD_ACT_MODE
{
    LD_ACT_MODE_OFF,
    LD_ACT_MODE_ON,
};


/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

#if (BT_RXDESC_NB <= 8)
    typedef uint8_t ld_rx_data_ptr_bf_t;
#elif (BT_RXDESC_NB <= 16)
    typedef uint16_t ld_rx_data_ptr_bf_t;
#else
    typedef uint32_t ld_rx_data_ptr_bf_t;
#endif // (BT_RXDESC_NB)

/// LD environment structure
struct ld_env_tag
{
    /// Local BD_ADDR
    struct bd_addr local_bd_addr;

    /**
     * Class of Device
     *
     * 0xXXXXXX Class of Device for the device.
     */
    struct devclass class_of_dev;

#if MAX_NB_SYNC
    /// Bit field for PCM setting (used when creating an audio link)
    uint8_t pcm_settings[8];
#endif //MAX_NB_SYNC

    /// Bit field indicating the active links
    ///
    ///   34         30 ... 24         23         22 ... 16         15          14 ... 8         7            6 ... 0
    /// +----+-----------------------+----+-----------------------+----+-----------------------+----+---------------------------+
    /// | 0/1|    link_id for SCO2   | 0/1|    link_id for SCO1   | 0/1|    link_id for SCO0   |    |      ACL[i] is active     |
    /// +----+-----------------------+----+-----------------------+----+-----------------------+----+---------------------------+
    ///
    uint32_t active_links;

    /**
     * Each bit of this field correspond to presence of data buffer onto a specific RX descriptor
     *  - if bit n = 1: RX descriptor index n has no buffer
     */
    ld_rx_data_ptr_bf_t rxdesc_data_ptr_bf;

    /// Local BCH
    uint8_t local_bch[LD_BCH_SIZE];

    /// Current RX descriptor index [0:N-1]
    uint8_t curr_rxdesc_index;

    /// Tx path delay
    uint8_t tx_path_delay;

    /// Rx path delay
    uint8_t rx_path_delay;

    /// Expected sync position in us, which incorporates the Rx path delay
    uint8_t exp_sync_pos;

#if (RW_BT_MWS_COEX)
    /// MWS external frame duration (us)
    uint16_t mws_ext_fr_duration;

    /// Scannable duration of MWS frame (us)
    uint16_t mws_ext_fr_scan_dur;
    /// MWS external frame sync assert offset (us)
    int16_t mws_ext_fr_offset;

    /// active MWS pattern index [0-2, 0xFF=disabled]
    uint8_t mws_index;
#endif //RW_BT_MWS_COEX
};

#if RW_BT_MWS_COEX
/// MWS pattern update information
typedef struct
{
    /// MWS pattern index (0-2, 0xFF=disabled)
    uint8_t pattern_index;
    /// slot offset (activation time) (slots)
    uint8_t slot_offset;
} ld_mwsifstat_t;
#endif //RW_BT_MWS_COEX


/*
 * VARIABLE DECLARATION
 *****************************************************************************************
 */

/// Basic Rate packet types table (classified by packet size)
extern const uint8_t ld_acl_br_types[BR_IDX_MAX];

/// Basic Rate packet sizes table (classified by packet size)
extern const uint16_t ld_acl_br_sizes[BR_IDX_MAX];

/// Enhanced Data Rate packet types table (classified by packet size)
extern const uint8_t ld_acl_edr_types[EDR_IDX_MAX];

/// Enhanced Data Rate packet sizes table (classified by packet size)
extern const uint16_t ld_acl_edr_sizes[EDR_IDX_MAX];

/// LD environment variable
extern struct ld_env_tag ld_env;

/*
 * FUNCTIONS DECLARATION
 ****************************************************************************************
 */

#if RW_BT_MWS_COEX
    /**
    ****************************************************************************************
    * @brief Get the active MWS index
    *
    * @return Pattern index (0-2, 0xFF=disabled) and activation offset (@see ld_mwsifstat_t)
    *
    *****************************************************************************************
    */
    ld_mwsifstat_t ld_mws_pattern_index_get(void);

    /**
    ****************************************************************************************
    * @brief Get the inactivity MWS duration
    *
    * @return uint8_t            Inactivity duration (0:none, 1-30: val*5ms, 31:infinite)
    *
    *****************************************************************************************
    */
    uint8_t ld_mws_inactivity_duration_get(void);

    /**
    ****************************************************************************************
    * @brief Build Xi scan mask for MWS COEX generically based on the specified uap_lap.
    *
    * @param[in]  mwscoex_xi_scan_mask         Xi mask to build.
    * @param[in]  uap_lap                      UAP/LAP for building Xi mask.
    *  ****************************************************************************************
    */
    void ld_mwscoex_xi_scan_mask_build(uint32_t *mwscoex_xi_scan_mask, uint32_t uap_lap);

#endif // RW_BT_MWS_COEX

/**
 ****************************************************************************************
 * @brief Free current RX descriptor
 ****************************************************************************************
 */
void ld_rxdesc_free(void);

/**
 ****************************************************************************************
 * @brief Check if the current RX descriptor is consumed by a given activity
 *
 * @param[in]   label   representing the activity (as indicated in the CS)
 *
 * @return True if current RX descriptor is consumed by the activity identified by label
 ****************************************************************************************
 */
bool ld_rxdesc_check(uint8_t label);

/**
 ****************************************************************************************
 * @brief Indicate active mode
 *
 * @param[in]   type        0:ACL | 1:SCO
 * @param[in]   link_id     ACL link ID
 * @param[in]   sco_link_id SCO link ID
 * @param[in]   mode        0: inactive | 1: active
 ****************************************************************************************
 */
void ld_active_mode_set(uint8_t type, uint8_t link_id, uint8_t sco_link_id, uint8_t mode);

/**
 ****************************************************************************************
 * @brief Check if an ACL link is active, or a SCO is present on this ACL
 *
 * @param[in]   link_id     ACL link ID
 *
 * @return True if ACL link is active, or a SCO is present on this ACL
 ****************************************************************************************
 */
bool ld_active_link_check(uint8_t link_id);


void ld_inq_init(void);
void ld_inq_reset(void);
void ld_iscan_init(void);
void ld_iscan_reset(void);
void ld_page_init(void);
void ld_page_reset(void);
void ld_pscan_init(void);
void ld_pscan_reset(void);
void ld_acl_init(void);
void ld_acl_reset(void);
void ld_bcst_acl_init(void);
void ld_bcst_acl_reset(void);
#if CSB_SUPPORT || PCA_SUPPORT
    void ld_sscan_init(void);
    void ld_sscan_reset(void);
    void ld_strain_init(void);
    void ld_strain_reset(void);
#endif // CSB_SUPPORT || PCA_SUPPORT
#if CSB_SUPPORT
    void ld_csb_tx_init(void);
    void ld_csb_tx_reset(void);
    void ld_csb_rx_init(void);
    void ld_csb_rx_reset(void);
#endif // CSB_SUPPORT
#if PCA_SUPPORT
    void ld_pca_init(void);
    void ld_pca_reset(void);
#endif // PCA_SUPPORT
#if (BT_HCI_TEST_MODE)
    void ld_test_init(void);
    void ld_test_reset(void);
#endif //(BT_HCI_TEST_MODE)

#if (EAVESDROPPING_SUPPORT)
    void ld_ed_init(void);
    void ld_ed_reset(void);
#endif // EAVESDROPPING_SUPPORT

/// @} LD
#endif // LD_INT_H_
