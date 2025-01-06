/**
 ****************************************************************************************
 *
 * @file display.c
 *
 * @brief DISPLAY Manager
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup DISPLAY
 * @{
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"  // platform definitions

#if DISPLAY_SUPPORT

#include <stdio.h>        // standard io definitions
#include <string.h>       // string definitions
#include "display.h"      // display functions
#include "display_env.h"  // display environment
#include "display_task.h" // display task
#include "ke_task.h"      // kernel task
#include "ke_event.h"     // kernel event
#include "ke_timer.h"     // kernel timer defines

#include "lcd.h"          // LCD functions


/*
 * DEFINES
 ****************************************************************************************
 */

#if DISPLAY_PERIODIC
    /// Display SCREEN period (in 10ms unit)
    #define DISPLAY_SCREEN_1ST_TIMER_DURATION      200
#endif // DISPLAY_PERIODIC


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// Display environment
struct display_env_tag display_env;

/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief Manages the DISPLAY.
 * Update what is displayed on the LCD and toggles leds.
 *****************************************************************************************
 */
__STATIC void display_user_action(void);

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

__STATIC void display_user_action(void)
{
    // Clear DISPLAY kernel event
    ke_event_clear(KE_EVENT_DISPLAY);

    switch (display_env.disp_mode)
    {
    case DISPLAY_MODE_INTERACTIVE_OFF:
#if DISPLAY_PERIODIC
    case DISPLAY_MODE_PERIODIC_ON:
    case DISPLAY_MODE_PERIODIC_OFF:
#endif // DISPLAY_PERIODIC
    {
        // Perform action
        switch (display_env.disp_action)
        {
        case DISPLAY_DOWN:
        {
            display_switch(DISPLAY_DIR_DOWN);
        }
        break;

        case DISPLAY_UP:
        {
            display_switch(DISPLAY_DIR_UP);
        }
        break;

        case DISPLAY_RIGHT:
        {
            display_switch(DISPLAY_DIR_RIGHT);
        }
        break;

        case DISPLAY_LEFT:
        {
            display_switch(DISPLAY_DIR_LEFT);
        }
        break;

        case DISPLAY_REFRESH:
        {
            display_switch(DISPLAY_DIR_STAY);
        }
        break;

        case DISPLAY_SELECT:
        {
            if ((display_env.disp_mode == DISPLAY_MODE_INTERACTIVE_OFF)
                    && (display_env.current->p_handler != NULL))
            {
                //Call handler
                (display_env.current->p_handler)(DISPLAY_INPUT_SELECT);

                // Check selection type
                if (display_env.current->select_type == DISPLAY_SELECT_TYPE_NORMAL)
                {
                    // Update mode
                    display_env.disp_mode = DISPLAY_MODE_INTERACTIVE_ON;
                }

                // Refresh current screen
                display_switch(DISPLAY_DIR_STAY);
            }
#if DISPLAY_PERIODIC
            else if (display_env.disp_mode == DISPLAY_MODE_PERIODIC_OFF)
            {
                display_env.disp_mode = DISPLAY_MODE_PERIODIC_ON;

                // Program the next Screen Refreshment
                ke_timer_set(DISPLAY_SCREEN_TO, TASK_DISPLAY, 10 * DISPLAY_SCREEN_1ST_TIMER_DURATION);
            }
            else if (display_env.disp_mode == DISPLAY_MODE_PERIODIC_ON)
            {
                display_env.disp_mode = DISPLAY_MODE_PERIODIC_OFF;

                // Cancel the next Screen Refreshment
                ke_timer_clear(DISPLAY_SCREEN_TO, TASK_DISPLAY);
            }
#endif // DISPLAY_PERIODIC
        }
        break;

        default:
        {
            // Should not happen
        }
        }
    }
    break;

    case DISPLAY_MODE_INTERACTIVE_ON:
    {
        // Perform action
        switch (display_env.disp_action)
        {
        case DISPLAY_RIGHT:
        {
            // Call screen handler
            if (display_env.current->p_handler != NULL)
            {
                (display_env.current->p_handler)(DISPLAY_INPUT_RIGHT);
            }
        }
        break;

        case DISPLAY_LEFT:
        {
            // Call screen handler
            if (display_env.current->p_handler != NULL)
            {
                (display_env.current->p_handler)(DISPLAY_INPUT_LEFT);
            }
        }
        break;

        case DISPLAY_SELECT:
        {
            // Call screen handler
            if (display_env.current->p_handler != NULL)
            {
                (display_env.current->p_handler)(DISPLAY_INPUT_DESELECT);
            }

            // Update mode
            display_env.disp_mode = DISPLAY_MODE_INTERACTIVE_OFF;
        }
        break;

        default:
        {
            // Nothing to do
        }
        }

        // Refresh current screen
        display_switch(DISPLAY_DIR_STAY);

    }
    break;

    default:
    {
        // Should not happen
    }
    break;
    }

    // Clear action flag
    display_env.disp_action = DISPLAY_NO_ACTION;
}


/*
 * MODULE FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void display_switch(enum display_direction direction)
{
    if (display_env.current)
    {
        // Replace current screen by the following one
        switch (direction)
        {
        case DISPLAY_DIR_DOWN:
        {
            display_env.current = display_env.current->down;
        }
        break;

        case DISPLAY_DIR_UP:
        {
            display_env.current = display_env.current->up;
        }
        break;

        case DISPLAY_DIR_RIGHT:
        {
            display_env.current = display_env.current->right;
        }
        break;

        case DISPLAY_DIR_LEFT:
        {
            display_env.current = display_env.current->left;
        }
        break;

        default:
        {
            // Stay on the current screen
        }
        break;
        }

        // Display screen content on LCD
        if (display_env.current != NULL)
        {
            lcd_printf(LCD_LINE_0, &display_env.current->line0[0]);
            lcd_printf(LCD_LINE_1, &display_env.current->line1[0]);
        }
    }
}


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void display_init(void)
{
    // Create DISPLAY Task
    ke_task_create(TASK_DISPLAY, &TASK_DESC_DISPLAY);

    // Initialize DISPLAY task to idle state
    ke_state_set(TASK_DISPLAY, DISPLAY_IDLE);

    // Create DISPLAY kernel event
    ke_event_callback_set(KE_EVENT_DISPLAY, &display_user_action);

    // Initialize environment
    memset(&display_env, 0, sizeof(display_env));

    // Start in interactive mode by default
    display_env.disp_mode = DISPLAY_MODE_INTERACTIVE_OFF;
}

uint8_t display_screen_alloc(void)
{
    uint8_t screen_id = 0;
    struct display_screen *p_new;

    // Look for a free screen
    while ((screen_id < MAX_SCREEN_NUMBER) && (display_env.screen[screen_id].up != NULL))
    {
        screen_id++;
    }

    if (screen_id < MAX_SCREEN_NUMBER)
    {
        p_new  = &display_env.screen[screen_id];

        // Link element to itself, as for a new list entry point
        p_new->down   = p_new;
        p_new->up     = p_new;
        p_new->right  = p_new;
        p_new->left   = p_new;
    }

    return screen_id;
}

void display_screen_insert(uint8_t new_screen_id, uint8_t list_screen_id)
{
    struct display_screen *p_first;
    struct display_screen *p_new;

    // Check validity of both screen IDs
    if (new_screen_id < MAX_SCREEN_NUMBER && list_screen_id < MAX_SCREEN_NUMBER)
    {
        p_first = &display_env.screen[list_screen_id];
        p_new   = &display_env.screen[new_screen_id];

        // Insert new screen before current screen
        p_new->down        = p_first;
        p_new->up          = p_first->up;
        p_first->up->down  = p_new;
        p_first->up        = p_new;

        // Link on the left to the same screen for all the list elements
        if (p_first->left != p_first)
        {
            p_new->left = p_first->left;
        }
    }

}

void display_screen_link(uint8_t root_screen_id, uint8_t list_screen_id)
{
    struct display_screen *p_first;
    struct display_screen *p_root;
    struct display_screen *p_next;

    // Check validity of both screen IDs
    if (root_screen_id < MAX_SCREEN_NUMBER && list_screen_id < MAX_SCREEN_NUMBER)
    {
        p_first = &display_env.screen[list_screen_id];
        p_root  = &display_env.screen[root_screen_id];
        p_next  = p_first->down;

        // Link on the right to the first element of the list
        p_root->right = p_first;

        // Link on the left to the parent screen
        p_first->left = p_root;

        // Link on the left to the same screen for all the list elements
        while (p_next != p_first)
        {
            p_next->left = p_root;
            p_next = p_next->down;
        }
    }

}

void display_screen_set(uint8_t screen_id, display_callback_t p_callback, const char *screen_line0, const char *screen_line1)
{
    struct display_screen *p_screen;

    // Check if screen ID is valid
    if (screen_id < MAX_SCREEN_NUMBER)
    {
        p_screen = &display_env.screen[screen_id];

        if (p_callback)
        {
            // Store handler
            p_screen->p_handler = p_callback;

            // Activate interactive mode
            display_env.disp_mode = DISPLAY_MODE_INTERACTIVE_OFF;
        }
        // Copy line 0
        strncpy(p_screen->line0, screen_line0, DISPLAY_LINE_SIZE);
        p_screen->line0[DISPLAY_LINE_SIZE] = '\0';
        // Copy line 1
        strncpy(p_screen->line1, screen_line1, DISPLAY_LINE_SIZE);
        p_screen->line1[DISPLAY_LINE_SIZE] = '\0';
        // Initialize selection type (normal by default)
        p_screen->select_type = DISPLAY_SELECT_TYPE_NORMAL;
    }
}

void display_screen_update(uint8_t screen_id, uint8_t line_nb, const char *line)
{
    struct display_screen *p_screen;
    char *p_line;

    // Check if screen ID is valid
    if (screen_id < MAX_SCREEN_NUMBER)
    {
        p_screen = &display_env.screen[screen_id];

        if (line_nb <= LCD_LINE_1)
        {
            p_line = line_nb ? p_screen->line1 : p_screen->line0;

            // Copy line
            strncpy(p_line, line, DISPLAY_LINE_SIZE);
            p_line[DISPLAY_LINE_SIZE] = '\0';
        }
    }
}

void display_goto_screen(uint8_t screen_id)
{
    if (screen_id < MAX_SCREEN_NUMBER)
    {
        display_env.current = &display_env.screen[screen_id];

        display_refresh();
    }
}

void display_start(uint8_t screen_id)
{
    struct display_screen *p_screen;

    if (screen_id < MAX_SCREEN_NUMBER)
    {
        p_screen = &display_env.screen[screen_id];

        // Set current screen pointer to start screen
        display_env.current = p_screen;

        // Display screen content on LCD
        display_switch(DISPLAY_DIR_STAY);
    }
}

void display_refresh(void)
{
    // Set next action to switch to up screen
    display_env.disp_action = DISPLAY_REFRESH;

    // Set DISPLAY kernel event
    ke_event_set(KE_EVENT_DISPLAY);
}

void display_pause(void)
{
#if DISPLAY_PERIODIC
    // Stop periodic screen
    display_env.disp_mode = DISPLAY_MODE_PERIODIC_OFF;

    // Program the next Screen Refreshment
    ke_timer_clear(DISPLAY_SCREEN_TO, TASK_DISPLAY);
#endif // DISPLAY_PERIODIC
}

void display_resume(void)
{
#if DISPLAY_PERIODIC
    // Re-start periodic screen
    display_env.disp_mode = DISPLAY_MODE_PERIODIC_ON;

    // Program the next Screen Refreshment
    ke_timer_set(DISPLAY_SCREEN_TO, TASK_DISPLAY, 10 * DISPLAY_SCREEN_1ST_TIMER_DURATION);
#endif // DISPLAY_PERIODIC
}

void display_down(void)
{
    // Set next action to switch to down screen
    display_env.disp_action = DISPLAY_DOWN;

    // Set DISPLAY kernel event
    ke_event_set(KE_EVENT_DISPLAY);
}

void display_up(void)
{
    // Set next action to switch to up screen
    display_env.disp_action = DISPLAY_UP;

    // Set DISPLAY kernel event
    ke_event_set(KE_EVENT_DISPLAY);
}

void display_right(void)
{
    // Set next action to switch to right screen
    display_env.disp_action = DISPLAY_RIGHT;

    // Set DISPLAY kernel event
    ke_event_set(KE_EVENT_DISPLAY);
}

void display_left(void)
{
    // Set next action to switch to left screen
    display_env.disp_action = DISPLAY_LEFT;

    // Set DISPLAY kernel event
    ke_event_set(KE_EVENT_DISPLAY);
}

void display_select(void)
{
    // Set next action to select screen
    display_env.disp_action = DISPLAY_SELECT;

    // Set DISPLAY kernel event
    ke_event_set(KE_EVENT_DISPLAY);
}

void display_select_type_set(uint8_t screen_id, uint8_t select_type)
{
    // Check if screen ID is valid and if select type is valid
    if ((screen_id < MAX_SCREEN_NUMBER)
            && (select_type <= DISPLAY_SELECT_TYPE_PUSH))
    {
        struct display_screen *p_screen = &display_env.screen[screen_id];

        // Check that screen can be selected
        if (p_screen->p_handler)
        {
            p_screen->select_type = select_type;
        }
    }
}

#endif //DISPLAY_SUPPORT

/// @} DISPLAY
