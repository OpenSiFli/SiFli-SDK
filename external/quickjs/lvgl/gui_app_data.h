#ifndef GUI_APP_DATA_H
#define GUI_APP_DATA_H

typedef void (*script_data_callback)(void*,uint16_t,void*);

typedef enum
{
    GUI_DATA_NULL = 0,
    GUI_DATA_RT_BATTERY_INFO,
    GUI_DATA_RT_STEP_INFO,
    GUI_DATA_RT_HR_INFO,
    GUI_DATA_RT_SPO2_INFO,
    GUI_DATA_RT_BP_INFO,
    GUI_DATA_RT_TEMP_INFO, /*<!reserved*/
    GUI_DATA_RT_STRESS_INFO, /*<!reserved*/

    GUI_DATA_SETTING_SYS_USER,
    GUI_DATA_WEATHER_INFO,

    GUI_DATA_DEV_BLE_CONNECT,
    GUI_DATA_DEV_BLE_ADV,

    GUI_DATA_DEV_WIFI_CONNECT,
    GUI_DATA_DEV_WIFI_ENABLE,
    GUI_DATA_DEV_AIR_MODE_ENABLE,

    GUI_DATA_LANG_CHANGE,
	GUI_DATA_SETTING_SPORT_TARGET,

    GUI_DATA_SYS_ID_END = 0x50,
    GUI_DATA_CUSTOM_ID_BEGIN = GUI_DATA_SYS_ID_END + 1,
    /********for customized definition********/
}gui_data_idx_t;


/******GUI_DATA_RT_BATTERY_INFO*******/
typedef enum
{
    BATTERY_NORMAL_STATUS, /*!< battery is in normal status*/
    BATTERY_CHARGING_STATUS,  /*!< battery is in charging status*/
} battery_charger_status_t;

typedef struct
{
    uint16_t                    level;             /*!< batttery level: Percentage */
    battery_charger_status_t    charger_status;    /*!< batttery charging state */
} battery_info_t;
/*!< return battery_info_t */
/*****************END******************/


/******GUI_DATA_RT_STEP_INFO*******/
typedef struct
{
    uint32_t        step;
    uint32_t        calories;
    uint32_t        distance;
} step_info_t;
/*!< return step_info_t */
/*****************END******************/


/********GUI_DATA_RT_HR_INFO*********/
typedef struct
{
    uint8_t         hr;
    uint8_t         rhr;
    uint32_t        timestamp;
} hr_info_t;
/*!< return hr_info_t */
/*****************END******************/


/*********GUI_DATA_RT_SPO2_INFO*********/
typedef struct
{
    uint8_t         spo2;
} spo2_info_t;
/*!< return spo2_info_t */
/*****************END******************/


/*********GUI_DATA_RT_BP_INFO*********/
typedef struct
{
    uint8_t         bp_h;
    uint8_t         bp_l;
} bp_info_t;
/*!< return bp_info_t */
/*****************END******************/


/*********GUI_DATA_RT_TEMP_INFO*********/
/*!< not used*/
/*****************END******************/


/***********GUI_DATA_RT_STRESS_INFO**********/
/*!< not used*/
/*****************END******************/


/*********GUI_DATA_SETTING_SYS_USER*********/
typedef struct
{
    char           sys_language[20];
    uint8_t        sys_hour_mode;
    uint8_t        sys_display_time;
    uint8_t        sys_pair_req;
} setting_sys_user_t;
/*!< return setting_sys_user_t */
/*****************END******************/


/*********GUI_DATA_WEATHER_INFO*********/
typedef struct
{
    int8_t         l_temp;             /*!< lowest temp*/
    int8_t         h_temp;             /*!< highest temp*/
    uint8_t        state;              /*!< 0: rainy; 1: windy; 2: sunny; 3:cloudy; 4: snowy; 0xff: invalid*/
    int8_t         temperature;        /*!< value according to temperetureUnit*/
    uint8_t        aqi;				   /*!< Air quality index, 0: excellent; 1: good; 2:bad*/
    uint8_t        hour;			   /*!< hour of weather*/

} setting_weather_t;
/*!< return setting_weather_t */
/*****************END******************/


/********************GUI_DATA_DEV_BLE_CONNECT***************/
/*!< return uint8_t; ture for connect, false for disconnect*/
/***************************END*****************************/


/********************GUI_DATA_DEV_BLE_ADV***************/
/*!< return uint8_t; ture for ADV, false for NO ADV*/
/***************************END*****************************/


/********************GUI_DATA_DEV_WIFI_CONNECT***************/
/*!< return uint8_t; ture for connect, false for disconnect*/
/***************************END*****************************/


/********************GUI_DATA_DEV_WIFI_ENABLE***************/
/*!< return uint8_t; ture for enable, false for disable*/
/***************************END*****************************/


/******************GUI_DATA_DEV_AIR_MODE_ENABLE*************/
/*!< return uint8_t; ture for enable, false for disable*/
/***************************END*****************************/


/******************GUI_DATA_LANG_CHANGE*************/
/*!< return char* */
/************************END*************************/

/******************GUI_DATA_SETTING_SPORT_TARGET*************/
typedef struct
{
    uint32_t        step_target;
    uint8_t         sport_mode;
    uint8_t         reserved[3];
    uint32_t        calories_target;
    uint32_t        distance_target;
    uint32_t        time_target;
    uint32_t        stand_target;
} setting_sport_target_t;
/*!< return setting_sport_target_t */

#endif

/************************END*************************/
