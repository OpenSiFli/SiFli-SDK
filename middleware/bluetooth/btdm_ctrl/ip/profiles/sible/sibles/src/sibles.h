/**
 ****************************************************************************************
 *
 * @file sibles.h
 *
 * @brief Header file - Device Information Service Server.
 *
 * Copyright (C) Sifli 2019-2022
 *
 *
 ****************************************************************************************
 */

#ifndef SIBLES_H_
#define SIBLES_H_

/**
 ****************************************************************************************
 * @addtogroup SIBLES Device Information Service Server
 * @ingroup SIBLE
 * @brief Device Information Service Server
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#if (BLE_SIBLE_SERVER)
#include "prf_types.h"
#include "prf.h"

/*
 * DEFINES
 ****************************************************************************************
 */


#define SIBLES_IDX_MAX        (4)

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// Possible states of the SIBLES task
enum
{
    /// Idle state
    SIBLES_IDLE,
    /// Busy state
    SIBLES_BUSY,
    /// Number of defined states.
    SIBLES_STATE_MAX
};


/// Value element
struct sibles_val_elmt
{
    /// list element header
    struct co_list_hdr hdr;
    /// value handle
    uint8_t hdl;
    /// value length
    uint8_t length;
    /// value data
    uint8_t data[__ARRAY_EMPTY];
};

///Sifil BLE Server Environment Variable
struct sibles_env_tag
{
    prf_env_t prf_env;                  /// profile environment
    struct co_list values;              /// List of values set by application
    uint16_t start_hdl;                 /// Service Attribute Start Handle
    uint16_t task;                      /// Sible task id
    uint16_t  req_hdl;                  /// Last requested handle
    uint8_t  req_conidx;                /// Last connection index which request value
    uint8_t sec_lvl;                    /// Security level.
    struct sibles_value *adv_data;      /// Advertisement data
    struct sibles_value *scan_rsp;      /// Scan response data
    struct gattc_disc_svc_ind *svc;
    ke_state_t state[SIBLES_IDX_MAX];   /// SIBLES task state
};

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

typedef void (*sibles_task_init_handler)(struct ke_task_desc *task_desc);

extern sibles_task_init_handler _sibles_task_init_handler;

#define sibles_task_init(task_desc) _sibles_task_init_handler(task_desc)



/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Retrieve SIBLE service profile interface
 *
 * @return SIBLE service profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs *sibles_prf_itf_get(void);


/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 * Initialize task handler
 *
 * @param task_desc Task descriptor to fill
 ****************************************************************************************
 */
void _sibles_task_init(struct ke_task_desc *task_desc);

#endif //BLE_SIBLE_SERVER

/// @} SIBLES

#endif // SIBLES_H_
