/**
 ****************************************************************************************
 *
 * @file display.h
 *
 * @brief DISPLAY Manager
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

#ifndef _DISPLAY_H_
#define _DISPLAY_H_

/**
 ****************************************************************************************
 * @defgroup DISPLAY
 * @ingroup DRIVERS
 * @brief DISPLAY driver
 *
 * @{
 *
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#ifndef CFG_ROM
    #include "rwip_config.h"  // platform definitions
#else // CFG_ROM
    #include "arch.h"         // architectural platform definitions
    #define DISPLAY_SUPPORT PLF_DISPLAY
#endif // CFG_ROM

#if DISPLAY_SUPPORT

#include <stdint.h>            // standard integer functions



/*
 * DEFINES
 ****************************************************************************************
 */

/// Screen line size (in chars)
#define DISPLAY_LINE_SIZE            16

/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */
/// Enumeration of user input in active cells
enum display_input
{
    DISPLAY_INPUT_DOWN    = 0,
    DISPLAY_INPUT_UP,
    DISPLAY_INPUT_RIGHT,
    DISPLAY_INPUT_LEFT,
    DISPLAY_INPUT_SELECT,
    DISPLAY_INPUT_DESELECT,
};

/// Selection type for a display screen
enum display_select_type
{
    DISPLAY_SELECT_TYPE_NORMAL = 0,
    DISPLAY_SELECT_TYPE_PUSH,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Callback function pointer type for user action in active cells
typedef void (*display_callback_t)(uint8_t);

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initializes the DISPLAY.
 * Writes FW build version and date.
 *****************************************************************************************
 */
void display_init(void);

/**
 ****************************************************************************************
 * @brief Add a new screen.
 * @return allocated screen number
 *****************************************************************************************
 */
uint8_t display_screen_alloc(void);

/**
 ****************************************************************************************
 * @brief Insert a screen in a screens list.
 * @param[in] new_screen_id     Screen to insert
 * @param[in] list_screen_id    Screen of the list
 *****************************************************************************************
 */
void  display_screen_insert(uint8_t new_screen_id, uint8_t list_screen_id);

/**
 ****************************************************************************************
 * @brief Link a screens list to a screen.
 * @param[in] root_screen_id    Screen number of the root element
 * @param[in] list_screen_id    Screen of the list to link
 *****************************************************************************************
 */
void  display_screen_link(uint8_t root_screen_id, uint8_t list_screen_id);

/**
 ****************************************************************************************
 * @brief Remove screen.
 * @param[in] screen_id     Screen number
 *****************************************************************************************
 */
void display_screen_remove(uint8_t screen_id);

/**
 ****************************************************************************************
 * @brief Set screen.
 * @param[in] screen_id     Screen number
 * @param[in] p_callback    Callback to screen select operations
 * @param[in] screen_line0  Data of the screen line 0
 * @param[in] screen_line1  Data of the screen line 1
 *****************************************************************************************
 */
void display_screen_set(uint8_t screen_id, display_callback_t p_callback, const char *screen_line0, const char *screen_line1);

/**
 ****************************************************************************************
 * @brief Update screen.
 * @param[in] screen_id     Screen number
 * @param[in] line_nb       Line number
 * @param[in] line          Line data
 *****************************************************************************************
 */
void display_screen_update(uint8_t screen_id, uint8_t line_nb, const char *line);

/**
 ****************************************************************************************
 * @brief Display the content of a screen using its screen_id
 * @param[in] screen_id     Screen number
 *****************************************************************************************
 */
void display_goto_screen(uint8_t screen_id);

/**
 ****************************************************************************************
 * @brief Start the DISPLAY.
 * @param[in] screen_id     Screen number
 *****************************************************************************************
 */
void display_start(uint8_t screen_id);

/**
 ****************************************************************************************
 * @brief Refresh screen content on lcd
 *****************************************************************************************
 */
void display_refresh(void);

/**
 ****************************************************************************************
 * @brief Pause the DISPLAY.
 *****************************************************************************************
 */
void display_pause(void);

/**
 ****************************************************************************************
 * @brief Resume the DISPLAY.
 *****************************************************************************************
 */
void display_resume(void);

/**
 ****************************************************************************************
 * @brief   Switch the display to down.
 ****************************************************************************************
 */
void display_down(void);
/**
 ****************************************************************************************
 * @brief   Switch the display to up.
 ****************************************************************************************
 */
void display_up(void);

/**
 ****************************************************************************************
 * @brief   Switch the display to right.
 ****************************************************************************************
 */
void display_right(void);

/**
 ****************************************************************************************
 * @brief   Switch the display to left.
 ****************************************************************************************
 */
void display_left(void);

/**
 ****************************************************************************************
 * @brief   Select the display.
 ****************************************************************************************
 */
void display_select(void);

/**
 ****************************************************************************************
 * @brief   Set behaviour when display screen is selected
 ****************************************************************************************
 */
void display_select_type_set(uint8_t screen_id, uint8_t select_type);

#endif //DISPLAY_SUPPORT

/// @} DISPLAY
#endif /* _DISPLAY_H_ */
