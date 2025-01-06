/**
 ****************************************************************************************
 *
 * @file app_display.c
 *
 * @brief Application Display entry point
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if (DISPLAY_SUPPORT)

#include "arch.h"
#include <stdio.h>
#include <string.h>                  // String manipulation and functions
#include "display.h"

#include "app_display.h"
#include "app.h"
#if (BLE_APP_HT)
    #include "app_ht.h"                  // Health thermometer application definitions
#endif //BLE_APP_HT

#if (BLE_APP_SEC)
    #include "app_sec.h"                 // Application security module definitions
#endif //(BLE_APP_SEC)

#if (BLE_APP_AM0)
    #include "app_am0.h"
#endif //(BLE_APP_AM0)


#if RTC_SUPPORT
    #include "rtc.h"
    #include "ke_event.h"
#endif //RTC_SUPPORT

/*
 * STRUCT DEFINITIONS
 ****************************************************************************************
 */

struct app_display_env_tag
{
    uint8_t screen_id_adv;             // Advertising
    uint8_t screen_id_con;             // Connection State
#if (NVDS_SUPPORT)
    uint8_t screen_id_bond;            // Bond State
#endif //(NVDS_SUPPORT)
#if (KE_PROFILING)
//    uint8_t screen_id_env_size;        // Env Memory Used Size
//    uint8_t screen_id_msg_size;        // Msg Memory Used Size
//    uint8_t screen_id_db_size;         // DB Memory Used Size
#endif //(KE_PROFILING)
#if (BLE_APP_HT)
    uint8_t screen_id_pin;             // Pin Code
#endif //(BLE_APP_HT)

#if RTC_SUPPORT
    uint8_t screen_id_rtc;             // RTC
#endif //RTC_SUPPORT

#if (BLE_APP_HT)
    //Level 3 - Health Thermometer
    uint8_t screen_id_ht_value;        // Temperature Value
    uint8_t screen_id_ht_type;         // Temperature Type
    uint8_t screen_id_ht_unit;         // Unit
#endif //(BLE_APP_HT)

#if (BLE_APP_HID)
    //Level 3 - HID
    uint8_t screen_id_hid_value;        //HID Mouse
#endif //(BLE_APP_HID)

#if (BLE_APP_AM0)
    //Level 3 - Audio
    uint8_t screen_id_am0_value;        // Audio over LE
#endif //(BLE_APP_AM0)

    /// advertising state (true = is advertising, false = no advertising)
    bool advertising;
    /// connected (true = connected, false = idle)
    bool connected;
#if (NVDS_SUPPORT)
    /// bonded (true = bonded, false = idle)
    bool bonded;
#endif //(NVDS_SUPPORT)
};

/*
 * GLOBAL VARIABLE DECLARATION
 ****************************************************************************************
 */

/// Application display environment
struct app_display_env_tag app_display_env;

#if RTC_SUPPORT
static char *const Months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
static char *const Days[]   = {"Mon", "Tue", "Wen", "Thu", "Fri", "Sat", "Sun"};
#endif //RTC_SUPPORT

/*
 * LOCAL FUNCTION DECLARATIONS
 ****************************************************************************************
 */
static void app_display_hdl_adv(enum display_input input);
#if (BLE_APP_HT)
    static void app_display_hdl_ht_temp_type(enum display_input input);
    static void app_display_hdl_ht_temp_value(enum display_input input);
#endif //BLE_APP_HT

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

static void app_display_hdl_adv(enum display_input input)
{
    switch (input)
    {
    case DISPLAY_INPUT_SELECT:
    {
        display_screen_update(app_display_env.screen_id_adv, 0, "o ADVERTISING");
    }
    break;
    case DISPLAY_INPUT_DESELECT:
    {
        display_screen_update(app_display_env.screen_id_adv, 0, "< ADVERTISING");
    }
    break;
    case DISPLAY_INPUT_LEFT:
    case DISPLAY_INPUT_RIGHT:
    {
        appm_update_adv_state(!app_display_env.advertising);
    }
    break;
    default:
        break;
    }
}

#if (NVDS_SUPPORT)
/**
 ****************************************************************************************
 * @brief Allows to unpair from a bonded peer device
 ****************************************************************************************
 */
static void app_display_hdl_bond(enum display_input input)
{
    switch (input)
    {
    case DISPLAY_INPUT_SELECT:
    {
        // React only if the device if a bond relationship exists with a peer device
        if (app_display_env.bonded)
        {
            display_screen_update(app_display_env.screen_id_bond, 0, "o BONDED");
        }
    }
    break;

    case DISPLAY_INPUT_DESELECT:
    {
        display_screen_update(app_display_env.screen_id_bond, 0, "< BONDED");
    }
    break;

    case DISPLAY_INPUT_LEFT:
    case DISPLAY_INPUT_RIGHT:
    {
#if (BLE_APP_SEC)
        // React only if the device if a bond relationship exists with a peer device
        if (app_display_env.bonded)
        {
            app_sec_remove_bond();

            app_display_env.bonded = false;

            // Update the displayed information
            display_screen_update(app_display_env.screen_id_bond, 1, "  NO");
        }
#endif // (BLE_APP_SEC)
    }
    break;

    default:
    {
        ASSERT_ERR(0);
    }
    break;
    }
}
#endif //(NVDS_SUPPORT)

#if (BLE_APP_HT)
static void app_display_hdl_ht_temp_type(enum display_input input)
{
    switch (input)
    {
    case DISPLAY_INPUT_SELECT:
    {
        display_screen_update(app_display_env.screen_id_ht_type, 0, "o TEMP TYPE");
    }
    break;
    case DISPLAY_INPUT_DESELECT:
    {
        display_screen_update(app_display_env.screen_id_ht_type, 0, "< TEMP TYPE");
    }
    break;
    case DISPLAY_INPUT_LEFT:
    {
        app_ht_temp_type_dec();
    }
    break;
    case DISPLAY_INPUT_RIGHT:
    {
        app_ht_temp_type_inc();
    }
    break;
    default:
        break;
    }
}

static void app_display_hdl_ht_temp_value(enum display_input input)
{
    switch (input)
    {
    case DISPLAY_INPUT_SELECT:
    {
        display_screen_update(app_display_env.screen_id_ht_value, 0, "o TEMP VALUE");
    }
    break;
    case DISPLAY_INPUT_DESELECT:
    {
        display_screen_update(app_display_env.screen_id_ht_value, 0, "< TEMP VALUE");
    }
    break;
    case DISPLAY_INPUT_LEFT:
    {
        app_ht_temp_dec();
    }
    break;
    case DISPLAY_INPUT_RIGHT:
    {
        app_ht_temp_inc();
    }
    break;
    default:
        break;
    }
}
#endif //BLE_APP_HT


#if RTC_SUPPORT
/**
 ****************************************************************************************
 * @brief Handles the 1 second tick (event handler).
 *****************************************************************************************
 */
static void app_display_sec_tick_evt_hdl(void)
{
    char *ptr;
    char line0[DISPLAY_LINE_SIZE];
    char line1[DISPLAY_LINE_SIZE];
    struct rtc_time time;

    // Clear DISPLAY kernel event
    ke_event_clear(KE_EVENT_RTC_1S_TICK);

    // Read current time from RTC
    rtc_get_time(&time);

    // Format the date
    ptr = &line0[0];
    *ptr++ = '<';
    memcpy(ptr, Months[time.tm_mon - 1], strlen(Months[time.tm_mon - 1]));
    ptr += strlen(Months[time.tm_mon - 1]);
    *ptr++ = ' ';
    if (time.tm_mday > 9)
    {
        *ptr++ = 48 + time.tm_mday / 10;
        time.tm_mday = time.tm_mday - (10 * (time.tm_mday / 10));
    }
    *ptr++ = 48 + time.tm_mday;
    *ptr++ = ' ';
    memcpy(ptr, Days[time.tm_wday - 1], strlen(Days[time.tm_wday - 1]));
    ptr += strlen(Days[time.tm_wday - 1]);
    *ptr++ = ' ';
    *ptr++ = '2';
    *ptr++ = '0';
    time.tm_year -= 2000;
    *ptr++ = 48 + time.tm_year / 10;
    time.tm_year = time.tm_year - (10 * (time.tm_year / 10));
    *ptr++ = 48 + time.tm_year;
    *ptr++ = 0;
//    snprintf(line0, DISPLAY_LINE_SIZE -1, "<%s %d %s %d", Months[time.tm_mon-1], time.tm_mday, Days[time.tm_wday-1], time.tm_year);

    // Format the time
    ptr = &line1[0];
    *ptr++ = 48 + time.tm_hour / 10;
    time.tm_hour = time.tm_hour - (10 * (time.tm_hour / 10));
    *ptr++ = 48 + time.tm_hour;
    *ptr++ = ':';
    *ptr++ = 48 + time.tm_min / 10;
    time.tm_min = time.tm_min - (10 * (time.tm_min / 10));
    *ptr++ = 48 + time.tm_min;
    *ptr++ = ':';
    *ptr++ = 48 + time.tm_sec / 10;
    time.tm_sec = time.tm_sec - (10 * (time.tm_sec / 10));
    *ptr++ = 48 + time.tm_sec;
    *ptr++ = 0;
//    snprintf(line1, DISPLAY_LINE_SIZE -1, "%02d:%02d:%02d", time.tm_hour, time.tm_min, time.tm_sec);


    // Update screen content
    display_screen_update(app_display_env.screen_id_rtc, 0, line0);
    display_screen_update(app_display_env.screen_id_rtc, 1, line1);

    // Program a screen refreshment
    display_refresh();
//    display_goto_screen(app_display_env.screen_id_rtc);
}

/**
 ****************************************************************************************
 * @brief Handles the 1 second tick (call back).
 *****************************************************************************************
 */
static void app_display_sec_tick_cbck(void)
{
    ke_event_set(KE_EVENT_RTC_1S_TICK);
}
#endif // RTC_SUPPORT

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void app_display_init(void)
{
    /* ***********************************************************************/
    /*                              Screen Allocation                        */
    /* ***********************************************************************/

    //Level 1
    uint8_t s_prof     = display_screen_alloc();       // Profiles
    uint8_t s_app_mgmt = display_screen_alloc();       // Application Management
    uint8_t s_info     = display_screen_alloc();       // Platform Information

    //Level 2 - Features
    uint8_t s_feat_top;
#if (BLE_APP_HT)
    uint8_t s_ht  = display_screen_alloc();            // Health Thermometer
#endif //(BLE_APP_HT)
#if (BLE_APP_HID)
    uint8_t s_hid = display_screen_alloc();            // HID Mouse
#endif //(BLE_APP_HID)
#if (BLE_APP_AM0)
    uint8_t s_am0  = display_screen_alloc();           // Audio Mode 0
#endif //(BLE_APP_AM0)


    //Level 2 - Application Management
    uint8_t s_adv  = display_screen_alloc();           // Advertising
    uint8_t s_con  = display_screen_alloc();           // Connection State
#if (NVDS_SUPPORT)
    uint8_t s_bond = display_screen_alloc();           // Bond State
#endif //(NVDS_SUPPORT)
#if (KE_PROFILING)
//    uint8_t s_env_size = display_screen_alloc();       // Env Memory Used Size
//    uint8_t s_msg_size = display_screen_alloc();       // Msg Memory Used Size
//    uint8_t s_db_size  = display_screen_alloc();       // DB Memory Used Size
#endif //(KE_PROFILING)
#if (BLE_APP_HT)
    uint8_t s_pin  = display_screen_alloc();           // PIN Code
#endif //(BLE_APP_HT)
#if RTC_SUPPORT
    uint8_t s_rtc = display_screen_alloc();            // RTC
    struct rtc_time alarm = {0, 0, 0, 0, 0, 0, 0, 0};
#endif //RTC_SUPPORT

#if (BLE_APP_HT)
    //Level 3 - Health Thermometer
    uint8_t s_ht_value = display_screen_alloc();       // Temperature Value
    uint8_t s_ht_type  = display_screen_alloc();       // Temperature Type
    uint8_t s_ht_unit  = display_screen_alloc();       // Unit
#endif //BLE_APP_HT

#if (BLE_APP_HID)
    //Level 3 - HID Mouse
    uint8_t s_hid_value = display_screen_alloc();      // Mouse Information
#endif //(BLE_APP_HID)

    app_display_env.screen_id_adv  = s_adv;
    app_display_env.screen_id_con  = s_con;
#if (NVDS_SUPPORT)
    app_display_env.screen_id_bond = s_bond;
#endif //(NVDS_SUPPORT)
#if (KE_PROFILING)
//    app_display_env.screen_id_env_size  = s_env_size;
//    app_display_env.screen_id_msg_size  = s_msg_size;
//    app_display_env.screen_id_db_size   = s_db_size;
#endif //(KE_PROFILING)
#if (BLE_APP_HT)
    app_display_env.screen_id_pin  = s_pin;
#endif //(BLE_APP_HT)
#if RTC_SUPPORT
    app_display_env.screen_id_rtc = s_rtc;
#endif //RTC_SUPPORT

#if (BLE_APP_HT)
    app_display_env.screen_id_ht_value = s_ht_value;
    app_display_env.screen_id_ht_type  = s_ht_type;
    app_display_env.screen_id_ht_unit  = s_ht_unit;
#endif //(BLE_APP_HT)

#if (BLE_APP_HID)
    app_display_env.screen_id_hid_value = s_hid_value;
#endif //(BLE_APP_HID)

#if (BLE_APP_AM0)
    app_display_env.screen_id_am0_value = s_am0;
#endif //(BLE_APP_AM0)

    /* ***********************************************************************/
    /*                              Screen Insertion                         */
    /* ***********************************************************************/

    //Level 1
    display_screen_insert(s_app_mgmt, s_prof);
    display_screen_insert(s_info, s_prof);

    //Level 2 - Features
#if (BLE_APP_HT)
    s_feat_top = s_ht;
#elif (BLE_APP_HID)
    s_feat_top = s_hid;
#elif (BLE_APP_AM0)
    s_feat_top = s_am0;
#else
    //No profile
    s_feat_top = display_screen_alloc();
    display_screen_set(s_feat_top, NULL, "< NO PROFILE", "");
#endif

#if (BLE_APP_HT)
    display_screen_insert(s_ht, s_feat_top);
#endif //(BLE_APP_HT)
#if (BLE_APP_HID)
    display_screen_insert(s_hid, s_feat_top);
#endif //(BLE_APP_HID)
#if (BLE_APP_AM0)
    display_screen_insert(s_am0, s_feat_top);
#endif //(BLE_APP_AM0)


    //Level 2 - Application Management
    display_screen_insert(s_con, s_adv);
#if (NVDS_SUPPORT)
    display_screen_insert(s_bond, s_adv);
#endif //(NVDS_SUPPORT)
#if (KE_PROFILING)
//    display_screen_insert(s_env_size, s_adv);
//    display_screen_insert(s_msg_size, s_adv);
//    display_screen_insert(s_db_size,  s_adv);
#endif //(KE_PROFILING)
#if (BLE_APP_HT)
    display_screen_insert(s_pin, s_adv);
#endif //(BLE_APP_HT)
#if RTC_SUPPORT
    display_screen_insert(s_rtc, s_adv);
#endif //RTC_SUPPORT

    //Level 3 - Health Thermometer
#if (BLE_APP_HT)
    display_screen_insert(s_ht_type, s_ht_value);
    display_screen_insert(s_ht_unit, s_ht_value);
#endif //BLE_APP_HT

    /* ***********************************************************************/
    /*                              Screen List Link                         */
    /* ***********************************************************************/

    //Link Level 1 - Feature with Level 2
    display_screen_link(s_prof, s_feat_top);

    //Link Level 1 - Application Management with Level 2
    display_screen_link(s_app_mgmt, s_adv);

    //Link Level 1 - Platform Information with Level 2
    display_screen_link(s_info, 0);

#if (BLE_APP_HT)
    //Link Level 2 - Health Thermometer with Level 3
    display_screen_link(s_ht, s_ht_value);
#endif //(BLE_APP_HT)

#if (BLE_APP_HID)
    //Link Level 2 - HID Mouse with Level 3
    display_screen_link(s_hid, s_hid_value);
#endif //(BLE_APP_HID)




    /* ***********************************************************************/
    /*                              Screen Set Content                       */
    /* ***********************************************************************/

    //Level 1
    display_screen_set(s_prof, NULL, " PROFILES      >", "");
    display_screen_set(s_app_mgmt, NULL, " APPLICATION   >", " MANAGEMENT");
    display_screen_set(s_info, NULL, " PLATFORM      >", " INFORMATION");

    //Level 2 - Features
#if (BLE_APP_HT)
    display_screen_set(s_ht, NULL, "< HEALTH       >", "  THERMOMETER");
#endif //(BLE_APP_HT)
#if (BLE_APP_HID)
    display_screen_set(s_hid, NULL, "< HID MOUSE    >", "");
#endif //(BLE_APP_HID)
#if (BLE_APP_AM0)
    display_screen_set(s_am0, NULL, "< AUDIO         ", "");
#endif //(BLE_APP_AM0)


    //Level 2 - Application Management
    display_screen_set(s_adv, &app_display_hdl_adv, "< ADVERTISING", "s NO");
    display_screen_set(s_con, NULL, "< CONNECTED", "  NO");
#if (NVDS_SUPPORT)
    display_screen_set(s_bond, &app_display_hdl_bond, "< BONDED", "  NO");
#endif //(NVDS_SUPPORT)
#if (KE_PROFILING)
//    display_screen_set(s_env_size, &app_display_hdl_env_size, "< ENV HEAP SIZE", "  0x0000/0x0000");
//    display_screen_set(s_msg_size, &app_display_hdl_msg_size, "< MSG HEAP SIZE", "  0x0000/0x0000");
//    display_screen_set(s_db_size, &app_display_hdl_db_size, "< DB HEAP SIZE", "  0x0000/0x0000");
#endif //(KE_PROFILING)
#if (BLE_APP_HT)
    display_screen_set(s_pin, NULL, "< PIN CODE", "  ------");
#endif //(BLE_APP_HT)
#if RTC_SUPPORT
    display_screen_set(s_rtc, NULL, "< No time", "");
#endif //RTC_SUPPORT

#if (BLE_APP_HT)
    //Level 3 - Health Thermometer
    display_screen_set(s_ht_value, &app_display_hdl_ht_temp_value, "< TEMP VALUE", "s --.--");
    display_screen_set(s_ht_type, &app_display_hdl_ht_temp_type, "< TEMP TYPE", "s NONE");
    display_screen_set(s_ht_unit, NULL, "< TEMP UNIT", "  CELCIUS");
#endif //(BLE_APP_HT)

#if (BLE_APP_HID)
    //Level 3 - HID Mouse
    display_screen_set(s_hid_value, NULL, "< B:0x00 X:0x00", "  Y:0x00 W:0x00");
#endif //(BLE_APP_HID)

    // Start with Profile screen
    display_start(s_prof);

#if RTC_SUPPORT
    // Register event to handle 1sec tick
    ke_event_callback_set(KE_EVENT_RTC_1S_TICK, &app_display_sec_tick_evt_hdl);
    // Start the periodic RTC interrupt
    rtc_enable_alarm0(RTC_ALARM_SEC, &alarm, &app_display_sec_tick_cbck);
#endif //RTC_SUPPORT
}

void app_display_set_adv(bool enable)
{
    app_display_env.advertising = enable;

    if (enable)
    {
        display_screen_update(app_display_env.screen_id_adv, 1, "s YES");
    }
    else
    {
        display_screen_update(app_display_env.screen_id_adv, 1, "s NO");
    }

    display_refresh();
}

#if (BLE_APP_HT)
void app_display_pin_code(uint32_t pin_code)
{
    // PIN Code line text
    char string[DISPLAY_LINE_SIZE];
    char pin_code_str[6];

    snprintf(pin_code_str, 5m  "%d", (int)pin_code);
    //String creation
    strcpy(string, "  ");
    strcat(string, pin_code_str);

    display_screen_update(app_display_env.screen_id_pin, 1, string);

    display_goto_screen(app_display_env.screen_id_pin);
}
#endif //(BLE_APP_HT)

void app_display_set_con(bool state)
{
    app_display_env.connected = state;

    if (state)
    {
        display_screen_update(app_display_env.screen_id_con, 1, "  YES");
    }
    else
    {
        display_screen_update(app_display_env.screen_id_con, 1, "  NO");
    }

    display_refresh();
}

#if (KE_PROFILING)
    //void app_display_hdl_env_size(uint16_t heap_size, uint16_t used_size)
    //{
    //    char string[DISPLAY_LINE_SIZE];
    //
    //    snprintf(string, "  0x%04X/0x%04X", used_size, heap_size);
    //    display_screen_update(app_display_env.screen_id_env_size, 1, string);
    //
    //    display_refresh();
    //}
    //
    //void app_display_hdl_msg_size(uint16_t heap_size, uint16_t used_size)
    //{
    //    char string[DISPLAY_LINE_SIZE];
    //
    //    snprintf(string, "  0x%04X/0x%04X", used_size, heap_size);
    //    display_screen_update(app_display_env.screen_id_msg_size, 1, string);
    //
    //    display_refresh();
    //}
    //
    //void app_display_hdl_db_size(uint16_t heap_size, uint16_t used_size)
    //{
    //    char string[DISPLAY_LINE_SIZE];
    //
    //    snprintf(string, "  0x%04X/0x%04X", used_size, heap_size);
    //    display_screen_update(app_display_env.screen_id_db_size, 1, string);
    //
    //    display_refresh();
    //}
#endif //(KE_PROFILING)

#if (NVDS_SUPPORT)
void app_display_set_bond(bool bonded)
{
    app_display_env.bonded = bonded;

    if (bonded)
    {
        display_screen_update(app_display_env.screen_id_bond, 1, "s YES");
    }
    else
    {
        display_screen_update(app_display_env.screen_id_bond, 1, "  NO");
    }

    display_refresh();
}
#endif //(NVDS_SUPPORT)

#if (BLE_APP_HT)
void app_display_update_temp_val_screen(uint32_t value)
{
    char int_part[4];
    char dec_part[4];
    char string[DISPLAY_LINE_SIZE];
    int int_part_int = (int)(app_ht_env.temp_value) / 100;
    int dec_part_int = (int)(app_ht_env.temp_value) % 100;

    snprintf(int_part, 3, "%d", int_part_int);
    snprintf(dec_part, 3, "%d", dec_part_int);

    // String creation
    strcpy(string, "s ");
    strcat(string, int_part);
    strcat(string, ".");
    if (dec_part_int < 10)
    {
        strcat(string, "0");
    }
    strcat(string, dec_part);

    display_screen_update(app_display_env.screen_id_ht_value, 1, string);
    display_refresh();
}

void app_display_update_temp_type_screen(const char *type_string)
{
    char string[DISPLAY_LINE_SIZE];

    // String creation
    strcpy(string, "s ");
    strcat(string, type_string);

    display_screen_update(app_display_env.screen_id_ht_type, 1, string);
    display_refresh();
}
#endif //(BLE_APP_HT)

#if (BLE_APP_HID)
void app_display_update_hid_value_screen(uint8_t b, uint8_t x,
        uint8_t y, uint8_t w)
{
    char string[DISPLAY_LINE_SIZE];

    // First Line
    snprintf(string, DISPLAY_LINE_SIZE - 1, "< B:0x%02X X:0x%02X", b, x);
    display_screen_update(app_display_env.screen_id_hid_value, 0, string);

    // Second Line
    snprintf(string, DISPLAY_LINE_SIZE - 1, "  Y:0x%02X W:0x%02X", y, w);
    display_screen_update(app_display_env.screen_id_hid_value, 1, string);

    display_refresh();
}
#endif //(BLE_APP_HID)

#if (BLE_APP_AM0)
/**
 ****************************************************************************************
 * @brief Update Audio over LE info
 ****************************************************************************************
 */
void app_display_update_audio_info(uint8_t state, uint8_t volume)
{
    char string[DISPLAY_LINE_SIZE * 2];

    switch (state)
    {
    case APP_AM0_STATE_OFF:
        snprintf(string, DISPLAY_LINE_SIZE * 2 - 1, "< AUDIO OFF     ");
        break;
    case APP_AM0_STATE_INIT:
        snprintf(string, DISPLAY_LINE_SIZE * 2 - 1, "< AUDIO INIT    ");
        break;
    case APP_AM0_STATE_CONNECTED:
        snprintf(string, DISPLAY_LINE_SIZE * 2 - 1, "< AUDIO CONNECT ");
        break;
    case APP_AM0_STATE_AUTH:
        snprintf(string, DISPLAY_LINE_SIZE * 2 - 1, "< AUDIO AUTH    ");
        break;
    case APP_AM0_STATE_CODEC_SEL:
        snprintf(string, DISPLAY_LINE_SIZE * 2 - 1, "< AUDIO CODEC   ");
        break;
    case APP_AM0_STATE_STREAM_ON:
        snprintf(string, DISPLAY_LINE_SIZE * 2 - 1, "< AUDIO STR ON  ");
        break;
    case APP_AM0_STATE_STREAM_OFF:
        snprintf(string, DISPLAY_LINE_SIZE * 2 - 1, "< AUDIO STR OFF ");
        break;
    default:
        snprintf(string, DISPLAY_LINE_SIZE * 2 - 1, "< AUDIO ???     ");
        break;
    }

    // First Line
    display_screen_update(app_display_env.screen_id_am0_value, 0, string);

    // Second Line
    snprintf(string, DISPLAY_LINE_SIZE * 2 - 1, " VOL = %d", volume);
    display_screen_update(app_display_env.screen_id_am0_value, 1, string);

    display_refresh();
}
#endif //((BLE_APP_AM0))

#endif //DISPLAY_SUPPORT

/// @} APP
