/*
 *      Copyright (C) 2020 Apple Inc. All Rights Reserved.
 *
 *      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
 *      which is contained in the License.txt file distributed with the Find My Network ADK,
 *      and only to those who accept that license.
 */

#ifndef fmna_state_machine_h
#define fmna_state_machine_h

#include "fmna_constants.h"
#include <stdbool.h>

typedef enum
{
    FMNA_SM_BOOT = 0,
    FMNA_SM_PAIR,
    FMNA_SM_SEPARATED,
    FMNA_SM_NEARBY,
    FMNA_SM_CONNECTING,
    FMNA_SM_FMNA_PAIR,
    FMNA_SM_FMNA_PAIR_COMPLETE,
    FMNA_SM_CONNECTED,
    FMNA_SM_DISCONNECTING,
    FMNA_SM_IDLE,
    FMNA_SM_NOCHANGE,
} FMNA_SM_State_t;

typedef enum
{
    FMNA_SM_EVENT_BOOT = 0,
    FMNA_SM_EVENT_NEARBY_SEPARATED_TIMEOUT,
    FMNA_SM_EVENT_KEY_ROTATE,
    FMNA_SM_EVENT_BONDED,
    FMNA_SM_EVENT_UNBONDED,
    FMNA_SM_EVENT_CONNECTED,
    FMNA_SM_EVENT_DISCONNECTED,
    FMNA_SM_EVENT_NEARBY,
    FMNA_SM_EVENT_SEPARATED,
    FMNA_SM_EVENT_PAIR,
    FMNA_SM_EVENT_SOUND_START,
    FMNA_SM_EVENT_SOUND_STOP,
    FMNA_SM_EVENT_SOUND_COMPLETE,
    FMNA_SM_EVENT_LOST_UT_SPEAKER_START,
    FMNA_SM_EVENT_FMNA_PAIRING_INITIATE,
    FMNA_SM_EVENT_FMNA_PAIRING_FINALIZE,
    FMNA_SM_EVENT_FMNA_PAIRING_COMPLETE,
    FMNA_SM_EVENT_FMNA_PAIRING_MFITOKEN,
    FMNA_SM_EVENT_MOTION_DETECTED,
    FMNA_SM_EVENT_DISABLE,
#ifdef DEBUG
    FMNA_SM_EVENT_DEBUG_RESET_INTO_SEPARATED,
#endif // DEBUG
} FMNA_SM_Event_t;

typedef struct
{
    uint32_t    next_secondary_key_rotation_index;
} __attribute__((aligned(SYS_POINTER_BSIZE), packed)) fmna_secondary_keys_info_t;

extern fmna_secondary_keys_info_t m_fmna_secondary_keys_info;

typedef struct
{
    uint32_t index;
    uint8_t  public_key[FMNA_PUBKEY_BLEN];
    uint8_t  ltk[GAP_SEC_KEY_LEN];
} fmna_primary_key_t;

typedef struct
{
    uint32_t index;
    uint8_t  public_key[FMNA_PUBKEY_BLEN];
} fmna_secondary_key_t;

extern fmna_primary_key_t m_fmna_current_primary_key;
extern fmna_primary_key_t m_fmna_current_separated_primary_key;
extern fmna_secondary_key_t m_fmna_current_secondary_key;

extern uint32_t m_fmna_key_rotation_timeout_ms;

/// Customized Find My State Machine error codes
#define FMNA_SM_ERROR_BASE_NUM          (0x02000000)

typedef enum
{
    FMNA_SM_STATUS_SUCCESS = FMNA_SM_ERROR_BASE_NUM,
    FMNA_SM_STATUS_FAIL,
    FMNA_SM_STATUS_INVALID_STATE,
    FMNA_SM_STATUS_NOT_BONDED,
    FMNA_SM_STATUS_CRYPTO_FAIL
} FMNA_Sm_Status_t;

/// Initializes the state machine.
/// @details     Initializes all state machine timers, and sends a BOOT event into the state machine.
///              Accessory drops into either the Pairing state or Separated state.
void fmna_state_machine_init(void);

/// Send event to the state machine.
///
/// @param[in]  fmna_evt     Event to send to the state machine, see FMNA_SM_Event_t.
/// @param[in]  p_context   Data associated with this state machine event.
void fmna_evt_handler(FMNA_SM_Event_t fmna_evt, void *p_context);

/// Dispatch state machine event to be handled in the main work loop.
///
/// @details    Intended for events that do not have additional associated info, e.g. trigger events.
///
/// @param[in]  fmna_evt     Event to send to the state machine, see FMNA_SM_Event_t.
void fmna_state_machine_dispatch_event(FMNA_SM_Event_t fmna_evt);

void fmna_state_machine_set_nearby_timeout_seconds(uint16_t nearby_timeout_seconds);
void fmna_state_machine_set_next_keyroll_ms(uint32_t next_keyroll_ms);
void fmna_state_machine_set_next_secondary_key_rotation_index(uint16_t conn_handle, uint32_t index);

/// Latch the current primary key as the primary separated key.
///
/// @details    Latches the key and sends LATCH_SEPARATED_KEY_RESPONSE.
///
/// @param[in]  conn_handle     Handle of connection issuiing the message.
void fmna_state_machine_latch_current_separated_key(uint16_t conn_handle);

void fmna_state_machine_stop_key_rotation_timers(void);

bool fmna_state_machine_has_been_maintenanced(void);

void fmna_state_machine_set_persistent_connection_disconnection(bool persistent_connection_disconnection);
bool fmna_state_machine_is_persistent_connection_disconnection(void);

bool fmna_state_machine_is_nearby(void);

uint32_t fmna_state_machine_get_non_owner_connection_timeout(void);

void fmna_state_machine_clear_keys(void);

void fmna_state_machine_disable(void);
void fmna_state_machine_enable(void);

#ifdef DEBUG
    void fmna_state_machine_set_key_rotation_timeout_ms(uint32_t key_rotation_timeout_ms);
    void fmna_state_machine_set_separated_ut_timeout_seconds(uint32_t separated_ut_timeout_seconds);
#endif

#endif /* fmna_state_machine_h */
