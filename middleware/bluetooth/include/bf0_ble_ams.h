/**
  ******************************************************************************
  * @file   bf0_ble_ams.h
  * @author Sifli software development team
  * @brief Header file - Sible AMS service.
 *
  ******************************************************************************
*/
/*
 * @attention
 * Copyright (c) 2021 - 2021,  Sifli Technology
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

#ifndef __BF0_BLE_AMS_H
#define __BF0_BLE_AMS_H

#include "bf0_ble_common.h"

/** @defgroup profile BLE profiles
  * @ingroup sibles
  * @{
  */


/**
 * @defgroup AMS AMS
 * @{
 */

/* Player:3, Queue:4, Track:4*/
#define BLE_AMS_MAXIMUM_ATTR_COUNT 4



/**
 * @brief AMS events that notify user.
 */
enum ble_ams_event
{
    BLE_AMS_ENABLE_RSP = BLE_AMS_TYPE,    /**< Enable result response. */
    BLE_AMS_SUPPORTED_CMD_NOTIFY_IND,     /**< Media source supported command indication. */
    BLE_AMS_ENTITY_ATTRIBUTE_PAIR_IND,    /**< Entity attribute pair indication. */
    BLE_AMS_ENABLE_PENDING_IND,           /**< Pending enable result. */
};



/**
 * @brief AMS error type.
 */
enum ble_ams_err
{
    BLE_AMS_ERR_NO_ERR,                                /**< No error.*/
    BLE_AMS_ERR_NO_CONNECTION,                       /**< No connection yet. */
    BLE_AMS_ERR_SEARCH_REMOTE_SERVICE_FAILED,        /**< Remote device doesn't support AMS source. */
    BLE_AMS_ERR_REGISTER_REMOTE_DEVICE_FAILED,       /**< Register remote deivce failed. */
    BLE_AMS_ERR_REJECTED,                            /**< Current action is rejected due to, etc, the action is on-going. */
    BLE_AMS_ERR_GENERATE,                           /**< Generate error. */
    BLE_AMS_ERR_CCCD_PENDING,                       /**< Write busy, result will come from BLE_AMS_ENABLE_PENDING_IND later. */
    BLE_AMS_ERR_CCCD_PROCESSING,                    /**< Pending enable processing, ignore new write cccd. */
};


/**
 * @brief Remote commandID values.
 */
typedef enum
{
    BLE_AMS_CMD_PLAY,                     /**< Play command. */
    BLE_AMS_CMD_PAUSE,                    /**< Pause command. */
    BLE_AMS_CMD_TOGGLE_PLAY_PAUSE,        /**< Toggle command. */
    BLE_AMS_CMD_NEXT,                     /**< Next command. */
    BLE_AMS_CMD_PREV,                     /**< Previous command. */
    BLE_AMS_CMD_VOL_UP,                   /**< Voloume up command. */
    BLE_AMS_CMD_VOL_DOWN,                 /**< Volume down command. */
    BLE_AMS_CMD_REPEAT_MODE,              /**< Repeat mode command. */
    BLE_AMS_CMD_SHUFFLE_MODE,             /**< Shuffle mode command. */
    BLE_AMS_CMD_SKIP_FWD,                 /**< Skip forward command. */
    BLE_AMS_CMD_SKIP_BACKWD,              /**< Skip backward command. */
    BLE_AMS_CMD_LIKE_TRACK,               /**< Like track command. */
    BLE_AMS_CMD_DISLIKE_TRACK,            /**< Dislike track command. */
    BLE_AMS_CMD_BOOKMARK_TRACK,           /**< Bookmark track command. */
    BLE_AMS_CMD_TOTAL                     /**< Total commands number. */
} ble_ams_cmd_t;


/**
 * @brief Remote commandID mask. commandID @ref ble_ams_cmd_t.
 */
typedef enum
{
    BLE_AMS_CMD_PLAY_MASK                  = 1 << BLE_AMS_CMD_PLAY,                /**< Play command mask. */
    BLE_AMS_CMD_PAUSE_MASK                 = 1 << BLE_AMS_CMD_PAUSE,               /**< Pause command mask. */
    BLE_AMS_CMD_TOGGLE_PLAY_PAUSE_MASK     = 1 << BLE_AMS_CMD_TOGGLE_PLAY_PAUSE,   /**< Toggle command mask. */
    BLE_AMS_CMD_NEXT_MASK                  = 1 << BLE_AMS_CMD_NEXT,                /**< Next command mask. */
    BLE_AMS_CMD_PREV_MASK                  = 1 << BLE_AMS_CMD_PREV,                /**< Previous command mask. */
    BLE_AMS_CMD_VOL_UP_MASK                = 1 << BLE_AMS_CMD_VOL_UP,              /**< Voloume up command mask. */
    BLE_AMS_CMD_VOL_DOWN_MASK              = 1 << BLE_AMS_CMD_VOL_DOWN,            /**< Volume down command mask. */
    BLE_AMS_CMD_REPEAT_MODE_MASK           = 1 << BLE_AMS_CMD_REPEAT_MODE,         /**< Repeat mode command mask. */
    BLE_AMS_CMD_SHUFFLE_MODE_MASK          = 1 << BLE_AMS_CMD_SHUFFLE_MODE,        /**< Shuffle mode command mask. */
    BLE_AMS_CMD_SKIP_FWD_MASK              = 1 << BLE_AMS_CMD_SKIP_FWD,            /**< Skip forward command mask. */
    BLE_AMS_CMD_SKIP_BACKWD_MASK           = 1 << BLE_AMS_CMD_SKIP_BACKWD,         /**< Skip backward command mask. */
    BLE_AMS_CMD_LIKE_TRACK_MASK            = 1 << BLE_AMS_CMD_LIKE_TRACK,          /**< Like track command mask. */
    BLE_AMS_CMD_DISLIKE_TRACK_MASK         = 1 << BLE_AMS_CMD_DISLIKE_TRACK,       /**< Dislike track command mask. */
    BLE_AMS_CMD_BOOKMARK_TRACK_MASK        = 1 << BLE_AMS_CMD_BOOKMARK_TRACK,      /**< Bookmark track command mask. */
    BLE_AMS_CMD_MASK_ALL                   = 0xFFFF,                               /**< Mask all commands. */
} ble_ams_cmd_mask_t;


/**
 * @brief EntityID values.
 */
typedef enum
{
    BLE_AMS_ENTITY_ID_PLAYER,         /**< Player entity. The attribute is @ref ble_ams_entity_player_attribute_t. */
    BLE_AMS_ENTITY_ID_QUEUE,          /**< Queue entity. The attribute is @ref ble_ams_entity_queue_attribute_t. */
    BLE_AMS_ENTITY_ID_TRACK,          /**< Track entity. The attribute is @ref ble_ams_entity_queue_attribute_t. */
} ble_ams_entity_id_t;


/**
 * @brief Attribute of player entity.
 */
typedef enum
{
    BLE_AMS_PLAYER_ATTR_ID_NAME,            /**< Localized name of app.*/
    BLE_AMS_PLAYER_ATTR_ID_PB_INFO,         /**< Playback info.*/
    BLE_AMS_PLAYER_ATTR_ID_VOL,             /**< Volume.*/
} ble_ams_entity_player_attribute_t;

/**
 * @brief Mask of attribute of player entity.
 */
typedef enum
{
    BLE_AMS_PLAYER_ATTR_ID_NAME_MASK    = 1 << BLE_AMS_PLAYER_ATTR_ID_NAME,      /**< Mask of localized name of app.*/
    BLE_AMS_PLAYER_ATTR_ID_PB_INFO_MASK = 1 << BLE_AMS_PLAYER_ATTR_ID_PB_INFO,   /**< Mask of playback info.*/
    BLE_AMS_PLAYER_ATTR_ID_VOL_MASK     = 1 << BLE_AMS_PLAYER_ATTR_ID_VOL,       /**< Mask of volume.*/
    BLE_AMS_PLAYER_ATTR_ID_ALL_MASK        = BLE_AMS_PLAYER_ATTR_ID_NAME_MASK | BLE_AMS_PLAYER_ATTR_ID_PB_INFO_MASK | BLE_AMS_PLAYER_ATTR_ID_VOL_MASK, /**< Mask of all attributes of player entity.*/
} ble_ams_entity_player_attribute_mask_t;


/**
 * @brief Attribute of player queue.
 */
typedef enum
{
    BLE_AMS_QUEUE_ATTR_ID_INDEX,             /**< Queue index.*/
    BLE_AMS_QUEUE_ATTR_ID_COUNT,             /**< Total number of queue.*/
    BLE_AMS_QUEUE_ATTR_ID_SHUFFLE,           /**< Shuffle mode.*/
    BLE_AMS_QUEUE_ATTR_ID_REPEAT,            /**< Repeat mode. */
} ble_ams_entity_queue_attribute_t;



/**
 * @brief Mask of attribute of player queue.
 */
typedef enum
{
    BLE_AMS_QUEUE_ATTR_ID_INDEX_MASK   = 1 << BLE_AMS_QUEUE_ATTR_ID_INDEX,    /**< Mask of queue index.*/
    BLE_AMS_QUEUE_ATTR_ID_COUNT_MASK   = 1 << BLE_AMS_QUEUE_ATTR_ID_COUNT,    /**< Mask of total number of queue.*/
    BLE_AMS_QUEUE_ATTR_ID_SHUFFLE_MASK = 1 << BLE_AMS_QUEUE_ATTR_ID_SHUFFLE,  /**< Mask of shuffle mode.*/
    BLE_AMS_QUEUE_ATTR_ID_REPEAT_MASK  = 1 << BLE_AMS_QUEUE_ATTR_ID_REPEAT,   /**< Mask of repeat mode. */
    BLE_AMS_QUEUE_ATTR_ID_ALL_MASK = BLE_AMS_QUEUE_ATTR_ID_INDEX_MASK | BLE_AMS_QUEUE_ATTR_ID_COUNT_MASK | BLE_AMS_QUEUE_ATTR_ID_SHUFFLE_MASK | BLE_AMS_QUEUE_ATTR_ID_REPEAT_MASK, /**< Mask of all attributes of queue entity.*/
} ble_ams_entity_queue_attribute_mask_t;


/**
 * @brief Attribute of track queue.
 */
typedef enum
{
    BLE_AMS_TRACK_ATTR_ID_ARTIST,            /**< The name of artist.*/
    BLE_AMS_TRACK_ATTR_ID_ALBUM,             /**< The name of album.*/
    BLE_AMS_TRACK_ATTR_ID_TILTE,             /**< The title of track.*/
    BLE_AMS_TRACK_ATTR_ID_DURATION,          /**< Total duration of the track in seconds. */
} ble_ams_entity_track_attribute_t;


/**
 * @brief Mask of attribute of track queue.
 */
typedef enum
{
    BLE_AMS_TRACK_ATTR_ID_ARTIST_MASK   = 1 << BLE_AMS_TRACK_ATTR_ID_ARTIST,   /**< Mask of name of artist.*/
    BLE_AMS_TRACK_ATTR_ID_ALBUM_MASK    = 1 << BLE_AMS_TRACK_ATTR_ID_ALBUM,    /**< Mask of name of album.*/
    BLE_AMS_TRACK_ATTR_ID_TILTE_MASK    = 1 << BLE_AMS_TRACK_ATTR_ID_TILTE,    /**< Mask of title of track.*/
    BLE_AMS_TRACK_ATTR_ID_DURATION_MASK = 1 << BLE_AMS_TRACK_ATTR_ID_DURATION, /**< Mask of total duration of the track in seconds. */
    BLE_AMS_TRACK_ATTR_ID_ALL_MASK = BLE_AMS_TRACK_ATTR_ID_ARTIST_MASK | BLE_AMS_TRACK_ATTR_ID_ALBUM_MASK | BLE_AMS_TRACK_ATTR_ID_TILTE_MASK | BLE_AMS_TRACK_ATTR_ID_DURATION_MASK, /**< Mask of all attributes of track entity.*/
} ble_ams_entity_track_attribute_mask_t;



/**
 * @brief The structure of #BLE_AMS_ENABLE_RSP.
 */
typedef struct
{
    uint8_t result;        /**< @see enum ble_ams_err */
} ble_ams_enable_rsp_t;


/**
 * @brief The structure of #BLE_AMS_SUPPORTED_CMD_NOTIFY_IND.
 */
typedef struct
{
    uint16_t cmd_mask;     /**< @see enum ble_ams_cmd_mask_t */
} ble_ams_supported_cmd_notify_ind_t;

/**
 * @brief The structure of #BLE_AMS_ENTITY_ATTRIBUTE_PAIR_IND.
 */
typedef struct
{
    uint8_t entity_id;       /**< @see enum ble_ams_entity_id_t */
    uint8_t attr_id;         /**< Associated attribute ID for an dedicated entityID. */
    uint8_t entity_up_flag;  /**< Entity update flag. 1 is truncated. */
    uint16_t len;            /**< length of attribute value. */
    uint8_t value[0];        /**< Attribute value. */
} ble_ams_entity_attr_value_t;

typedef struct
{
    uint8_t result;          /**< @see enum ble_ams_err */
} ble_ams_enable_pending_ind_t;



/**
 * @brief Enable ams service.
   @param[in] conn_idx Connection index.
   @retval result.BLE_AMS_ERR_NO_ERR is successful.
 */
uint8_t ble_ams_enable(uint8_t conn_idx);


/**
 * @brief Send remote commandID to media source.
   @param[in] cmd commandID.
   @retval result.BLE_AMS_ERR_NO_ERR is successful.
 */
uint8_t ble_ams_send_command(ble_ams_cmd_t cmd);

/**
 * @brief Enable CCCD of media source.
   @param[in] is_enable whether enabled.
   @retval result.BLE_AMS_ERR_NO_ERR is successful.
 */
uint8_t ble_ams_cccd_enable(uint8_t is_enable);

/**
 * @brief Check whether cccd is enabled
   @retval result.TRUE if enabled
 */
bool ble_ams_is_cccd_enable();

/**
 * @brief Enable attribute of player entity.
   @param[in] attr_mask attribute mask.
 */
void ble_ams_player_attr_enable(ble_ams_entity_player_attribute_mask_t attr_mask);

/**
 * @brief Disable attribute of player entity.
   @param[in] attr_mask attribute mask.
 */
void ble_ams_player_attr_disable(ble_ams_entity_player_attribute_mask_t attr_mask);

/**
 * @brief Enable attribute of queue entity.
   @param[in] attr_mask attribute mask.
 */
void ble_ams_queue_attr_enable(ble_ams_entity_queue_attribute_mask_t attr_mask);

/**
 * @brief Disable attribute of queue entity.
   @param[in] attr_mask attribute mask.
 */
void ble_ams_queue_attr_disable(ble_ams_entity_queue_attribute_mask_t attr_mask);

/**
 * @brief Enable attribute of track entity.
   @param[in] attr_mask attribute mask.
 */
void ble_ams_track_attr_enable(ble_ams_entity_track_attribute_mask_t attr_mask);

/**
 * @brief Disable attribute of track entity.
   @param[in] attr_mask attribute mask.
 */
void ble_ams_track_attr_disable(ble_ams_entity_track_attribute_mask_t attr_mask);

/**
* @} AMS
*/

/**
* @} profile
*/


#endif // __BF0_BLE_AMS_H

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
