/**
 ****************************************************************************************
 *
 * @file rwhl_config.h
 *
 * @brief Configuration of the High level stack
 *
 *
 *
 ****************************************************************************************
 */

#ifndef RWHL_CONFIG_H_
#define RWHL_CONFIG_H_


#define HOST_HEAP_MSG_SIZE          BLEHL_HEAP_MSG_SIZE
#define HOST_HEAP_ENV_SIZE          BLEHL_HEAP_ENV_SIZE
#define HOST_HEAP_PROFILE_SIZE      BLEHL_HEAP_DB_SIZE

#define MAX_BT_ACL_LINK CFG_CON_ACL
#define MAX_BLE_ACL_LINK CFG_CON

#define GAP_LE_MTU_MAX              1024

#include "rwble_hl_config.h"

#endif //RWHL_CONFIG_H_
