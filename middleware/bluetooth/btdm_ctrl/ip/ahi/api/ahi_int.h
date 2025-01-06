/**
 ****************************************************************************************
 *
 * @file ahi_int.h
 *
 * @brief This file contains definitions related to the Application Host Interface in the same core
 *
 * Copyright (C) Sifli 2020-2020
 *
 *
 ****************************************************************************************
 */

#ifndef AHI_INT_H_
#define AHI_INT_H_

/**
 ****************************************************************************************
 * @addtogroup AHI_INT Application Host Interface
 * @ingroup AHI_INT
 * @brief Application Host Interface, based on AHI INT functionality.
 *
 *@{
 *
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"     // SW configuration

#if (AHI_INT_SUPPORT)
    #include "rwip.h"            // SW interface

    #include "stdbool.h"         // boolean definition
    #include "ke_msg.h"          // kernel message definition
    #include "ke_task.h"
    #include "co_list.h"         // list API



    /*
    * Defines
    ****************************************************************************************
    */

    /*
    * TYPE DEFINITIONS
    ****************************************************************************************
    */



    /*
    * GLOBAL VARIABLE DECLARATIONS
    ****************************************************************************************
    */


    /*
    * FUNCTION DECLARATIONS
    ****************************************************************************************
    */

    /**
    ****************************************************************************************
    * @brief AHI INT initialization function: initializes states and transport.
    *****************************************************************************************
    */
    void ahi_int_init(void);

    /**
    ****************************************************************************************
    * @brief AHI INT callback initialization function: initializes user callback to forward.
    *****************************************************************************************
    */
    void ahi_callback_init(ke_msg_func_t callback);

    /**
    ****************************************************************************************
    * @brief AHI INT ready check function: whether AHI INT is ready.
    *****************************************************************************************
    */
    uint32_t ahi_int_is_ready(void);

    /**
    ****************************************************************************************
    * @brief Callback to user over the Application to Host Interface in the  same core.
    *
    * @param[in] msgid    Message id
    * @param[in] param    Data buffer to callback
    * @param[in] dest_id  Destination ID corresponding to message module.
    * @param[in] src_id   Source ID corresponding to message module.
    *****************************************************************************************
    */
    void ahi_int_msg_callback(ke_msg_id_t const msgid, void *param, ke_task_id_t const dest_id, ke_task_id_t const src_id);

    /**
    ****************************************************************************************
    * @brief Send a data message over the Application to Host Interface in the same  core.
    * @param[in] data     Data buffer to send
    *****************************************************************************************
    */
    void ahi_int_msg_send(void const *param_ptr);

#endif //AHI_TL_SUPPORT

/// @} AHI
#endif // AHI_H_
