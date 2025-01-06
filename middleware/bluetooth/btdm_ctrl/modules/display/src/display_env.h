/**
 ****************************************************************************************
 *
 * @file display_env.h
 *
 * @brief This file contains definitions related to the Display module
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef DISPLAY_ENV_H_
#define DISPLAY_ENV_H_

/**
 ****************************************************************************************
 * @addtogroup DISPLAY Display module
 * @ingroup DISPLAY
 * @brief Display module, based on DISPLAY functionality.
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

#if (DISPLAY_SUPPORT)

#include "co_list.h"
#include "ke_msg.h"



/*
 * DEFINES
 ****************************************************************************************
 */

/// Maximum number of screens
#if defined(CFG_APP)
    #define MAX_SCREEN_NUMBER      64
#else //CFG_APP
    #define MAX_SCREEN_NUMBER      14
#endif //CFG_APP

/// Periodic screen
#define DISPLAY_PERIODIC       0

/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */

/// Enumeration of display controller actions
enum display_mode
{
    DISPLAY_MODE_INTERACTIVE_OFF = 0,
    DISPLAY_MODE_INTERACTIVE_ON,
#if DISPLAY_PERIODIC
    DISPLAY_MODE_PERIODIC_OFF,
    DISPLAY_MODE_PERIODIC_ON,
#endif // DISPLAY_PERIODIC
};

/// Enumeration of display controller actions
enum display_action
{
    DISPLAY_NO_ACTION = 0,
    DISPLAY_DOWN,
    DISPLAY_UP,
    DISPLAY_RIGHT,
    DISPLAY_LEFT,
    DISPLAY_SELECT,
    DISPLAY_REFRESH,
};

/// Enumeration of display screen switch direction
enum display_direction
{
    DISPLAY_DIR_STAY = 0,
    DISPLAY_DIR_DOWN,
    DISPLAY_DIR_UP,
    DISPLAY_DIR_RIGHT,
    DISPLAY_DIR_LEFT,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */



/*
 * STRUCT DEFINITIONS
 ****************************************************************************************
 */

/// Element representing a screen in a double chained list
struct display_screen
{
    /// Pointer to down screen
    struct display_screen *down;

    /// Pointer to up screen
    struct display_screen *up;

    /// Pointer to right screen
    struct display_screen *right;

    /// Pointer to left screen
    struct display_screen *left;

    /// Pointer to event call back function
    display_callback_t p_handler;

    /// Screen line 0 content
    char line0[DISPLAY_LINE_SIZE + 1];
    /// Screen line 1 content
    char line1[DISPLAY_LINE_SIZE + 1];

    /// Selection type
    uint8_t select_type;
};

/// Display environment structure definition
struct display_env_tag
{
    /// Screen Pool
    struct display_screen screen[MAX_SCREEN_NUMBER];

    /// Active screen list
    struct display_screen *current;

    /// Current display manager action
    enum display_action disp_action;

    /// Current display manager mode
    enum display_mode disp_mode;
};

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */

///DISPLAY environment structure external global variable declaration
extern struct display_env_tag display_env;


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief   Switch the display screen following a direction.
 * @param[in]  direction     0:down, 1:up, 2:right, 3:left
 ****************************************************************************************
 */
void display_switch(enum display_direction direction);


#endif //DISPLAY_SUPPORT

/// @} DISPLAY
#endif // DISPLAY_ENV_H_
