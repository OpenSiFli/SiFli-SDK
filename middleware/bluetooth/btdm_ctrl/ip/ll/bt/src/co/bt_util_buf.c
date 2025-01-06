/**
 ****************************************************************************************
 *
 * @file bt_util_buf.c
 *
 * @brief BT EM buffers
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup BT_UTIL_BUF
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"       // SW configuration

#include <string.h>
#include "arch.h"
#include "co_bt.h"
#include "co_utils.h"        // common utility definition
#include "co_math.h"
#include "bt_util_buf.h"
#include "em_map.h"
#include "ke_mem.h"
#include "ld.h"              // to inform that a buffer is ready to be used by a descriptor


/*
 * DEFINES
 ****************************************************************************************
 */


/*
 * STRUCTURE DEFINITION
 ****************************************************************************************
 */

#if VOICE_OVER_HCI
/// BT EM buffer management environment structure
struct bt_util_buf_sync_tag
{
    /// List of free synchronous TX buffers
    struct co_list tx_free;
    /// List of free synchronous RX buffers
    struct co_list rx_free;
    /// Pool of Sync TX buffers
    struct bt_em_sync_buf_elt *tx_pool;
    /// Pool of Sync RX buffers
    struct bt_em_sync_buf_elt *rx_pool;
    /// Number of Sync TX buffers
    uint8_t tx_buf_nb;
    /// Size of Sync TX buffers
    uint8_t tx_buf_size;
    /// Number of Sync RX buffers
    uint8_t rx_buf_nb;
    /// Size of Sync RX buffers
    uint8_t rx_buf_size;
};
#endif // VOICE_OVER_HCI

/// BT EM buffer management environment structure
struct bt_util_buf_env_tag
{
    /// List of free LMP RX buffers
    struct co_list lmp_tx_free;
    /// List of free ACL RX buffers
    struct co_list acl_rx_free;
    /// List of free ACL TX buffers
    struct co_list acl_tx_free;

    /// Pool of LMP TX buffers (one for all links), MAX IS EM_BT_LMPTXBUF_NB
    struct bt_em_lmp_buf_elt *lmp_tx_pool;

    /// Pool of ACL RX buffers (one for all links), max is EM_BT_ACLRXBUF_NB
    struct bt_em_acl_buf_elt *acl_rx_pool;

    /// Pool of ACL TX buffers (one for all links), max is EM_BT_ACLTXBUF_NB
    struct bt_em_acl_buf_elt *acl_tx_pool;

#if VOICE_OVER_HCI
    /// Environment for synchronous buffer allocation (one per synchronous link)
    struct bt_util_buf_sync_tag *sync_tag[MAX_NB_SYNC];
#endif // VOICE_OVER_HCI
};


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// BT EM buffer management environment
struct bt_util_buf_env_tag bt_util_buf_env;


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void bt_util_buf_init(bool reset)
{
#if defined EM_SIZE_LIMIT
    ASSERT_INFO((uint32_t)EM_BT_END <= EM_SIZE_LIMIT, EM_BT_END / 1024, EM_SIZE_LIMIT / 1024);
#endif // defined EM_SIZE_LIMIT

    if (reset)
    {
        bluetooth_act_configt_t *rom_cfg = rom_config_get_default_link_config();
        RW_ASSERT(rom_cfg);
#if VOICE_OVER_HCI
        for (int i = 0 ; i < MAX_NB_SYNC ; i++)
        {
            if (bt_util_buf_env.sync_tag[i] != NULL)
            {
                ke_free(bt_util_buf_env.sync_tag[i]);
                bt_util_buf_env.sync_tag[i] = NULL;
            }
        }
#endif //VOICE_OVER_HCI
    }
    if (!reset)
    {
        if (bt_util_buf_env.lmp_tx_pool)
            ke_free(bt_util_buf_env.lmp_tx_pool);

        if (bt_util_buf_env.acl_rx_pool)
            ke_free(bt_util_buf_env.acl_rx_pool);

        if (bt_util_buf_env.acl_tx_pool)
            ke_free(bt_util_buf_env.acl_tx_pool);

        // Clear environment memory
        memset(&bt_util_buf_env, 0, sizeof(bt_util_buf_env));

        bt_util_buf_env.lmp_tx_pool = (struct bt_em_lmp_buf_elt *)ke_malloc_system(sizeof(struct bt_em_lmp_buf_elt) * EM_NUM_BT_LMPTXBUF, KE_MEM_ENV);
        bt_util_buf_env.acl_rx_pool = (struct bt_em_acl_buf_elt *)ke_malloc_system(sizeof(struct bt_em_acl_buf_elt) * EM_NUM_BT_ACLRXBUF, KE_MEM_ENV);
        bt_util_buf_env.acl_tx_pool = (struct bt_em_acl_buf_elt *)ke_malloc_system(sizeof(struct bt_em_acl_buf_elt) * EM_NUM_BT_ACLTXBUF, KE_MEM_ENV);

        // Point each pool element to its associated buffer in EM

        //for (int i = 0 ; i < EM_BT_LMPTXBUF_NB ; i++)
        for (int i = 0 ; i < EM_NUM_BT_LMPTXBUF ; i++)
        {
            bt_util_buf_env.lmp_tx_pool[i].buf_ptr = EM_BT_LMPTXBUF_OFFSET + i * EM_BT_LMPTXBUF_SIZE;
        }

        // Point each pool element to its associated buffer in EM
        //for (int i = 0 ; i < EM_BT_ACLRXBUF_NB ; i++)
        for (int i = 0 ; i < EM_NUM_BT_ACLRXBUF ; i++)
        {
            bt_util_buf_env.acl_rx_pool[i].buf_ptr = EM_BT_ACLRXBUF_OFFSET + i * EM_BT_ACLRXBUF_SIZE + ACL_RX_BUF_HEADER_SPACE;
        }

        // Point each pool element to its associated buffer in EM
        //for (int i = 0 ; i < EM_BT_ACLTXBUF_NB ; i++)
        for (int i = 0 ; i < EM_NUM_BT_ACLTXBUF ; i++)
        {
            bt_util_buf_env.acl_tx_pool[i].buf_ptr = EM_BT_ACLTXBUF_OFFSET + i * EM_BT_ACLTXBUF_SIZE;
        }

    }


    // Initialize the list of free LMP TX buffers
    co_list_pool_init(&bt_util_buf_env.lmp_tx_free,
                      &bt_util_buf_env.lmp_tx_pool[0],
                      sizeof(struct bt_em_lmp_buf_elt),
                      EM_NUM_BT_LMPTXBUF);



    // Initialize the list of free ACL RX buffers
    co_list_pool_init(&bt_util_buf_env.acl_rx_free,
                      &bt_util_buf_env.acl_rx_pool[0],
                      sizeof(struct bt_em_acl_buf_elt),
                      EM_NUM_BT_ACLRXBUF);



    // Initialize the list of free ACL TX buffers
    co_list_pool_init(&bt_util_buf_env.acl_tx_free,
                      &bt_util_buf_env.acl_tx_pool[0],
                      sizeof(struct bt_em_acl_buf_elt),
                      EM_NUM_BT_ACLTXBUF);


}

struct bt_em_lmp_buf_elt *bt_util_buf_lmp_tx_alloc(void)
{
    // Get free element from free list
    struct bt_em_lmp_buf_elt *elt = (struct bt_em_lmp_buf_elt *) co_list_pop_front(&bt_util_buf_env.lmp_tx_free);

    return elt;
}

void bt_util_buf_lmp_tx_free(uint16_t buf)
{
    // Find associated pool element index
    uint8_t index = (buf - EM_BT_LMPTXBUF_OFFSET) / EM_BT_LMPTXBUF_SIZE;

    // Push to free list
    co_list_push_back(&bt_util_buf_env.lmp_tx_free, &bt_util_buf_env.lmp_tx_pool[index].hdr);
}

struct bt_em_acl_buf_elt *bt_util_buf_acl_rx_alloc(void)
{
    // Get free element from free list
    struct bt_em_acl_buf_elt *elt = (struct bt_em_acl_buf_elt *) co_list_pop_front(&bt_util_buf_env.acl_rx_free);

    return elt;
}

void bt_util_buf_acl_rx_free(uint16_t buf)
{
    // Find associated pool element index
    uint8_t index = (buf - ACL_RX_BUF_HEADER_SPACE - EM_BT_ACLRXBUF_OFFSET) / EM_BT_ACLRXBUF_SIZE;
    struct bt_em_acl_buf_elt *p_elt;
    // Check that the buffer address is in the buffer section
    ASSERT_INFO(index < EM_NUM_BT_ACLRXBUF, index, buf);

    p_elt = &(bt_util_buf_env.acl_rx_pool[index]);

#if(BLE_HOST_PRESENT)
    // ensure that buffer is properly initialized
    p_elt->buf_ptr = EM_BT_ACLRXBUF_OFFSET + (index * EM_BT_ACLRXBUF_SIZE) + ACL_RX_BUF_HEADER_SPACE;
#endif //(BLE_HOST_PRESENT)

    GLOBAL_INT_DISABLE();
    // Check if the buffer can be directly assigned to a RX descriptor
    if (!ld_rxdesc_buf_ready(p_elt->buf_ptr))
    {
        // Push to free list
        co_list_push_back(&bt_util_buf_env.acl_rx_free, &(p_elt->hdr));
    }
    GLOBAL_INT_RESTORE();
}

#if(HOST_PRESENT)
struct bt_em_acl_buf_elt *bt_util_buf_elt_rx_get(uint16_t buf)
{
    // Find associated pool element index
    uint8_t index = (buf - ACL_RX_BUF_HEADER_SPACE - EM_BT_ACLRXBUF_OFFSET) / EM_BT_ACLRXBUF_SIZE;
    struct bt_em_acl_buf_elt *p_elt = &(bt_util_buf_env.acl_rx_pool[index]);
    // Check that the buffer address is in the buffer section
    ASSERT_INFO(index < EM_NUM_BT_ACLRXBUF, index, buf);

    return (p_elt);
}
#endif //(HOST_PRESENT)

uint16_t bt_util_buf_acl_tx_alloc(void)
{
    uint16_t buf_ptr = 0;
    struct bt_em_acl_buf_elt *elt;
    GLOBAL_INT_DISABLE();
    // Get free element from free list
    elt = (struct bt_em_acl_buf_elt *) co_list_pop_front(&bt_util_buf_env.acl_tx_free);
    GLOBAL_INT_RESTORE();
    if (elt != NULL)
    {
        buf_ptr = elt->buf_ptr;
    }

    return buf_ptr;
}

struct bt_em_acl_buf_elt *bt_util_buf_acl_tx_elt_get(uint16_t buf)
{
    // Find associated pool element index
    uint8_t index = (buf - EM_BT_ACLTXBUF_OFFSET) / EM_BT_ACLTXBUF_SIZE;

    // Check that the buffer address is in the buffer section
    ASSERT_INFO(index < EM_BT_ACLTXBUF_NB, index, buf);

    return &(bt_util_buf_env.acl_tx_pool[index]);
}

void bt_util_buf_acl_tx_free(uint16_t buf)
{
    // Find associated pool element index
    uint8_t index = (buf - EM_BT_ACLTXBUF_OFFSET) / EM_BT_ACLTXBUF_SIZE;

    // Push to free list
    GLOBAL_INT_DISABLE();
    co_list_push_back(&bt_util_buf_env.acl_tx_free, &bt_util_buf_env.acl_tx_pool[index].hdr);
    GLOBAL_INT_RESTORE();
}

#if VOICE_OVER_HCI
void bt_util_buf_sync_init(uint8_t sync_id, uint8_t tx_buf_nb, uint8_t tx_buf_size, uint8_t rx_buf_nb, uint8_t rx_buf_size)
{
    // Allocate memory for the Sync buffers allocation system (including RX/TX elements pools)
    bt_util_buf_env.sync_tag[sync_id] = (struct bt_util_buf_sync_tag *) ke_malloc_system(CO_ALIGN4_HI(sizeof(struct bt_util_buf_sync_tag)) + CO_ALIGN4_HI(tx_buf_nb * sizeof(struct bt_em_sync_buf_elt)) + CO_ALIGN4_HI(rx_buf_nb * sizeof(struct bt_em_sync_buf_elt)), KE_MEM_ENV);

    if (bt_util_buf_env.sync_tag[sync_id] != NULL)
    {
        // Point to environment data
        struct bt_util_buf_sync_tag *sync_tag = bt_util_buf_env.sync_tag[sync_id];

        // Point Sync RX/TX buffers pool to the first element
        sync_tag->tx_pool = (struct bt_em_sync_buf_elt *)((uint8_t *) sync_tag + CO_ALIGN4_HI(sizeof(struct bt_util_buf_sync_tag)));
        sync_tag->rx_pool = (struct bt_em_sync_buf_elt *)((uint8_t *) sync_tag->tx_pool + CO_ALIGN4_HI(tx_buf_nb * sizeof(struct bt_em_sync_buf_elt)));

        // Initialize the list of free Sync TX buffers
        co_list_pool_init(&sync_tag->tx_free,
                          sync_tag->tx_pool,
                          sizeof(struct bt_em_sync_buf_elt),
                          tx_buf_nb);

        // Initialize the list of free Sync RX buffers
        co_list_pool_init(&sync_tag->rx_free,
                          sync_tag->rx_pool,
                          sizeof(struct bt_em_sync_buf_elt),
                          rx_buf_nb);

        // Point each pool element to its associated buffer in EM (voice buffers are aligned to 4-bytes EM addresses)
        for (int i = 0 ; i < tx_buf_nb ; i++)
        {
            sync_tag->tx_pool[i].buf_ptr = (EM_BT_AUDIOBUF_OFF(sync_id)) + i * CO_ALIGN4_HI(tx_buf_size);
        }
        for (int i = 0 ; i < rx_buf_nb ; i++)
        {
            sync_tag->rx_pool[i].buf_ptr = (EM_BT_AUDIOBUF_OFF(sync_id) + tx_buf_nb * CO_ALIGN4_HI(tx_buf_size)) + i * (CO_ALIGN4_HI(rx_buf_size) + SYNC_RX_BUF_HEADER_SPACE) + SYNC_RX_BUF_HEADER_SPACE;
        }

        // Retain Sync RX/TX buffers size and number
        sync_tag->tx_buf_nb = tx_buf_nb;
        sync_tag->tx_buf_size = tx_buf_size;
        sync_tag->rx_buf_nb = rx_buf_nb;
        sync_tag->rx_buf_size = rx_buf_size;
    }
    else
    {
        ASSERT_ERR_FORCE(0);
    }
}

void bt_util_buf_sync_clear(uint8_t sync_id)
{
    if (bt_util_buf_env.sync_tag[sync_id] != NULL)
    {
        ke_free(bt_util_buf_env.sync_tag[sync_id]);
        bt_util_buf_env.sync_tag[sync_id] = NULL;
    }
}

uint16_t bt_util_buf_sync_tx_alloc(uint8_t sync_id, uint8_t size)
{
    uint16_t buf_ptr = 0;

    if (bt_util_buf_env.sync_tag[sync_id] != NULL)
    {
        // Point to environment data
        struct bt_util_buf_sync_tag *sync_tag = bt_util_buf_env.sync_tag[sync_id];

        if (size <= sync_tag->tx_buf_size)
        {
            // Get free element from free list
            struct bt_em_sync_buf_elt *elt = (struct bt_em_sync_buf_elt *) co_list_pop_front(&sync_tag->tx_free);
            if (elt != NULL)
            {
                buf_ptr = elt->buf_ptr;
            }
        }
    }

    return buf_ptr;
}

struct bt_em_sync_buf_elt *bt_util_buf_sync_tx_elt_get(uint8_t sync_id, uint16_t buf)
{
    // Point to environment data
    struct bt_util_buf_sync_tag *sync_tag = bt_util_buf_env.sync_tag[sync_id];

    // Find associated pool element index
    uint8_t index = (uint8_t)((buf - EM_BT_AUDIOBUF_OFF(sync_id)) / CO_ALIGN4_HI(sync_tag->tx_buf_size));

    // Check that the buffer address is in the buffer section
    ASSERT_INFO(index < sync_tag->tx_buf_nb, index, buf);

    return &(sync_tag->tx_pool[index]);
}

void bt_util_buf_sync_tx_free(uint8_t sync_id, uint16_t buf)
{
    if (bt_util_buf_env.sync_tag[sync_id] != NULL)
    {
        // Point to environment data
        struct bt_util_buf_sync_tag *sync_tag = bt_util_buf_env.sync_tag[sync_id];

        // Find associated pool element index
        uint8_t index = (uint8_t)((buf - EM_BT_AUDIOBUF_OFF(sync_id)) / CO_ALIGN4_HI(sync_tag->tx_buf_size));

        // Push to free list
        co_list_push_back(&sync_tag->tx_free, &sync_tag->tx_pool[index].hdr);
    }
}

struct bt_em_sync_buf_elt *bt_util_buf_sync_rx_alloc(uint8_t sync_id, uint8_t size)
{
    struct bt_em_sync_buf_elt *elt = NULL;

    if (bt_util_buf_env.sync_tag[sync_id] != NULL)
    {
        // Point to environment data
        struct bt_util_buf_sync_tag *sync_tag = bt_util_buf_env.sync_tag[sync_id];

        if (size <= sync_tag->rx_buf_size)
        {
            // Get free element from free list
            elt = (struct bt_em_sync_buf_elt *) co_list_pop_front(&sync_tag->rx_free);
        }
    }

    return elt;
}

void bt_util_buf_sync_rx_free(uint8_t sync_id, uint16_t buf)
{
    if (bt_util_buf_env.sync_tag[sync_id] != NULL)
    {
        // Point to environment data
        struct bt_util_buf_sync_tag *sync_tag = bt_util_buf_env.sync_tag[sync_id];

        // Find associated pool element index
        uint8_t index = (uint8_t)((buf - SYNC_RX_BUF_HEADER_SPACE - (EM_BT_AUDIOBUF_OFF(sync_id) + sync_tag->tx_buf_nb * CO_ALIGN4_HI(sync_tag->tx_buf_size))) / CO_ALIGN4_HI(sync_tag->rx_buf_size + SYNC_RX_BUF_HEADER_SPACE));

        // Push to free list
        co_list_push_back(&sync_tag->rx_free, &sync_tag->rx_pool[index].hdr);
    }
}
#endif // VOICE_OVER_HCI


/// @} BT_UTIL_BUF
