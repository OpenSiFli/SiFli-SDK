/*
 *      Copyright (C) 2020 Apple Inc. All Rights Reserved.
 *
 *      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
 *      which is contained in the License.txt file distributed with the Find My Network ADK,
 *      and only to those who accept that license.
 */

#include "fmna_util.h"
#include "fmna_platform_includes.h"
#include "fmna_sound_platform.h"
#include "fmna_state_machine.h"

#include "fmna_util_platform.h"
#include "fmna_constants_platform.h"

#define SOUND_LED                   BSP_BOARD_LED_1                         // LED to represent sound playing

APP_TIMER_DEF(m_fmna_sound_timeout_timer_id);    /** Sound timeout timer id*/

static void fmna_sound_timeout_handler(void *p_context);

void fmna_sound_platform_init(void)
{
    ret_code_t ret_code = app_timer_create(m_fmna_sound_timeout_timer_id,
                                           APP_TIMER_MODE_SINGLE_SHOT,
                                           fmna_sound_timeout_handler);
    APP_ERROR_CHECK(ret_code);

    //TODO: Replace with any speaker initializations.
    // bsp_board_init(BSP_INIT_LEDS);
}

void fmna_sound_platform_start(void)
{
    LOG_I("Sound starting...");
    // bsp_board_led_on(SOUND_LED);

    ret_code_t ret_code = app_timer_start(m_fmna_sound_timeout_timer_id,
                                          SEC_TO_MSEC(10),
                                          0);
    APP_ERROR_CHECK(ret_code);
}

void fmna_sound_platform_stop(void)
{
    ret_code_t ret_code = app_timer_stop(m_fmna_sound_timeout_timer_id);
    APP_ERROR_CHECK(ret_code);

    //bsp_board_led_off(SOUND_LED);
    LOG_I("Sound successfully stopped");
    fmna_state_machine_dispatch_event(FMNA_SM_EVENT_SOUND_COMPLETE);
}

static void fmna_sound_timeout_handler(void *p_context)
{
    LOG_I("fmna_sound_timeout_handler");
    // bsp_board_led_off(SOUND_LED);
    fmna_state_machine_dispatch_event(FMNA_SM_EVENT_SOUND_COMPLETE);
}
