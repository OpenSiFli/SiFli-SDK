#ifndef BL_CHIP_CUSTOM_H
#define BL_CHIP_CUSTOM_H
#include "bl6133.h"
#include "stdlib.h"
#include "string.h"



#define     TS_CHIP          BL6XX3
#define INT_UPDATE_MODE
//#define   BL6133_UPDATE_USE_SW_I2C
#define     CTP_USE_HW_I2C
//#define     CTP_USE_SW_I2C
#ifdef  CTP_USE_HW_I2C

    #ifdef  __CST_SUPPORT__
        #define     CTP_SLAVE_ADDR                (0x15)
    #else
        #define     CTP_SLAVE_ADDR                  (0x2C)
    #endif

#else
    #ifdef  __CST_SUPPORT__
        #define     CTP_SLAVE_ADDR                (0x15<<1)
    #else
        #define     CTP_SLAVE_ADDR                (0x2C<<1)
    #endif
#endif
#define     BTL_CHECK_CHIPID
#define     GPIO_EINT
#define     RESET_PIN_WAKEUP
//#define     INT_PIN_WAKEUP
//#define     NEED_CONFIG_EINT_RESUME
//#define     SWAP_XY
#define     BL_UPDATE_FIRMWARE_ENABLE
#define     TPD_RES_X        LCD_WIDTH
#define     TPD_RES_Y        LCD_HEIGHT
//#define     BL_DEBUG
//#define     BL_LOW_POWER
//#define     BL_DEBUG_SUPPORT

#define MDELAY(ms) rt_thread_mdelay(ms) //HAL_Delay(ms) //


#ifdef INT_UPDATE_MODE
    #define   SET_WAKEUP_HIGH    bl_exit_update_with_int()
    #define   SET_WAKEUP_LOW     bl_enter_update_with_int()
#endif

#endif

