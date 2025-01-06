
#pragma once

// c
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "pah_comm.h"

//============================================================================
// Functions
//============================================================================
#ifdef __cplusplus
extern  "C" {
#endif

//============================================================================
// Global variables
//============================================================================
#pragma pack(1)

bool pah_sensor_init(void);
void ppg_sensor_start(uint8_t work_mode);
void ppg_sensor_task_interrupt(void);
void ppg_sensor_task_polling(void);
void ppg_interrupt_handle(void);
void pah_sensor_stop(void);
uint8_t pah8007_alg_task(void);

uint8_t factory_test_led_golden_sample(uint8_t *touch_value_out);
uint8_t factory_test_led_target_sample(uint8_t *touch_value_out);
uint8_t factory_test_light_leak(uint8_t *touch_value_out);

//============================================================================
// 8007 Driver Config
//============================================================================
#define USE_TRUE_SYS_TICK       // enable to choose true system tick for PAH8007
#define V30_ALG                 // if use v30x009 version algorithm, enable this; if use v10x009 version algorithm, disable this
//#define STATIC_RAM_EN          // if use dymanic allocation to create space for v30x009 algorithm, disable this, or enable it

#ifdef V30_ALG
#ifdef STATIC_RAM_EN
#define V30_ALG_STATIC_RAM                  // create static array for algorithm
#else
#define V30_ALG_DYNAMIC_RAM                 // dynamic allocate RAM for algorithm
#endif
#endif


//============================================================================
// Default
//============================================================================
#define PAH_8007_PRODUCT_ID                (0x63)

#define PPG_INITIAL_EXPOSURE       (400)

#define ENABLE_PPG_CONVERGENCE     true

#define PPG_CONVERGENCE_T_WEIGHT_NORMAL_MODE   (10)
#define PPG_CONVERGENCE_T_WEIGHT_FAST_MODE     (20)
#define PPG_CONVERGENCE_T_MIN                  (20)
#define PPG_CONVERGENCE_T_MAX                  (2500)
#define PPG_CONVERGENCE_RAW_OUTER_HIGH_BOUND   (24576)
#define PPG_CONVERGENCE_RAW_INNER_HIGH_TARGET  (23552)
#define PPG_CONVERGENCE_RAW_INNER_LOW_TARGET   (15636)
#define PPG_CONVERGENCE_RAW_OUTER_LOW_BOUND    (12288)

#define ENABLE_PPG_RECONVERGENCE_IF_OUT_OF_BOUNDS  true

#define ENABLE_PPG_POWER_SAVING    false

#define TOUCH_FLAG_TRIGGER_COUNT   (3)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//============================================================================
// Register address
//============================================================================

#define ADDR_00_PRODUCT_ID                     (0x00)

#define ADDR_03                                (0x03)
#define ADDR_03_BIT_R_PS_EN                    (0)
#define ADDR_03_BIT_R_ALS_EN                   (1)
#define ADDR_03_BIT_SW_SUSPEND                 (5)

#define ADDR_04                                (0x04)
#define ADDR_04_BIT_ALS_OVERFLOW_ECO           (7)

#define ADDR_09                                (0x09)
#define ADDR_09_BIT_ALS_INTERRUPT              (2)
#define ADDR_09_BIT_PS_INTERRUPT               (5)
#define ADDR_09_BIT_PS_APPROACH                (6)

#define ADDR_10_PS_ONOFF_SYNC                  (0x10)

#define ADDR_11                                (0x11)
#define ADDR_11_BIT_R_LED_CONTROL_OUT_EN       (4)
#define ADDR_11_BIT_R_PS_INTERRUPT_EN          (7)

#define ADDR_0E_ALS_LUX_SYNC                   (0x0E)

#define ADDR_34                                (0x34)
#define ADDR_34_BIT_R_EXPOSURE_ALS_F0          (0)

#define ADDR_35                                (0x35)
#define ADDR_35_BIT_R_EXPOSURE_ALS_F0          (0)

#define ADDR_38                                (0x38)
#define ADDR_38_BIT_0_LOCK_EXPOTIME            (0)
#define ADDR_38_BIT_1_CLEAR_ALS_OVERFLOW_ECO   (1)

#define ADDR_EC                                (0xEC)
#define ADDR_EC_BIT_7_R_CLK_MANUAL             (7)


//============================================================================
// Definition
//============================================================================
typedef struct
{
    bool        has_data;
    uint16_t    data;
    // Avaiable only if has_data is true
    bool        has_overflow;
    // Avaiable only if has_data is true
    uint16_t    exposure_time;

} pah_8007_ppg_data;

typedef struct
{
    bool        has_touch_flag;
    int         touch_flag;
    int         approach;
    uint8_t     data;

} pah_8007_touch_data;

//============================================================================
// Private definition
//============================================================================
typedef struct
{
    bool        is_enabled;
    bool        is_converging;

} pah_8007_ppg_state;

typedef struct
{
    bool        is_enabled;
    int         approach;
    uint8_t     approach_continuous_count;
    bool        touch_flag_is_ready;
    int         touch_flag;

} pah_8007_touch_state;

typedef struct
{
    // Power state
    bool                is_powered;
    // PPG state
    pah_8007_ppg_state      ppg_state;
    pah_8007_ppg_data       ppg_result;
    // TOUCH state
    pah_8007_touch_state    touch_state;
    pah_8007_touch_data     touch_result;
    // Other
    int                 interrupt_config;

} pah_8007;

/////////////////////////////////////////////////////////////////

typedef struct raw_data_13bytes_t
{
    uint8_t loss;
    uint32_t rawData;
    uint8_t non_zero1;
    uint8_t non_zero2;
    uint8_t non_zero3;
    uint8_t frame_count;
    uint8_t time;
    uint8_t rsv;
    uint8_t touch;
    uint8_t rsv2;
} sRaw_Data_13Bytes;

typedef enum
{
    MSG_ALG_NOCHANGE = 0,
    MSG_ALG_ENABLE,
    MSG_ALG_DISABLE,
} alg_msg_code_t;

typedef enum
{
    PPG_ONLY_MODE,
    TOUCH_ONLY_MODE,
    AUTO_MODE,
} work_mode_config;

//============================================================================
// Definition
//============================================================================

typedef enum
{
    INTERRUPT_CONFIG_NONE,
    INTERRUPT_CONFIG_ALS,
    INTERRUPT_CONFIG_APPROACH,
} interrupt_config;

typedef enum
{

    PAH_RET_SUCCESS = 0,            // Success

    PAH_RET_PENDING,

    PAH_RET_FAILED,                 // Unknown error, generally there exists bug in this driver

    PAH_RET_PLAT_FAILED,            // Platform API failed
    PAH_RET_VERIFY_FAILED,          // Verify device failed

    PAH_RET_NOT_IMPLEMENTED,        // This API is not supported or is not implemented

    PAH_RET_INVALID_ARGUMENT,       // Invalid argument to API
    PAH_RET_INVALID_OPERATION,      // This operation is not available at this time

    PAH_RET_FIFO_CKS_FAILED,        // Failed to compare check sum with fifo data
    PAH_RET_FIFO_OVERFLOW,          // Fifo in device is overflow

    PAH_RET_ASYNC_PENDING,

} pah_ret;
#define PAH_SUCCEEDED(ret)              (ret == PAH_RET_SUCCESS)
#define PAH_FAILED(ret)                 (!PAH_SUCCEEDED(ret))


#define PAH_CHECK_RET(ret) \
    if (PAH_FAILED(ret)) { \
        return ret; \
    }

#define PAH_CHECK_NULL(p) \
    if (!p) { \
        return PAH_RET_INVALID_ARGUMENT; \
    }


#define PAH_ARRAY_SIZE(arr)         (sizeof(arr) / sizeof(arr[0]))
#define PAH_SET_BIT(var,pos)        do { ((var) |= (1 << (pos))); } while(0)
#define PAH_CLEAR_BIT(var,pos)      do { ((var) &= ~(1 << (pos))); } while(0)
#define PAH_CHECK_BIT(var,pos)      (((var) >> (pos)) & 1)
#define PAH_UINT16_HIGH(var)        ((uint8_t)(var >> 8))
#define PAH_UINT16_LOW(var)         ((uint8_t)(var & UINT8_MAX))

#pragma pack()


typedef struct
{

    volatile bool       has_interrupt;
    volatile uint64_t   interrupt_timestamp;
    // pah
    pah_8007            pah_8007;
    // ppg
    bool                ppg_is_enabled;
    bool                ppg_only_is_enabled;
    bool                ppg_has_prev_report_timestamp;
    uint64_t            ppg_prev_report_timestamp;
    uint8_t             ppg_frame_count;
    // touch
    bool                touch_is_enabled;
    bool                touch_only_is_enabled;
    int                 touch_last_is_touched;
    // hr
    bool                hr_is_enabled;
    // alg
    uint8_t             alg_status;
    // debug
    bool                debug_print_header;

} main_state;


#ifdef __cplusplus
}
#endif
