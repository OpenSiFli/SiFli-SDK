#ifndef BL_CHIP_COMMON_H
#define BL_CHIP_COMMON_H
/*****************************************************************
**头文件加载
*****************************************************************/
#include "bl_chip_custom.h"
//#include "SmartBand_Log.h"

/*****************************************************************
**Define
*****************************************************************/
#define BL8XXX_60   0x01//bl86XX,bl8878
#define BL8XXX_61   0x02//bl8858,bl8868
#define BL8XXX_63   0x03//bl8818,bl8128,bl8138
#define BL6XX0      0x04//bl6090.bl6130,bl6280,bl6360
#define BL6XX1      0x05//bl6391
#define BL6XX3      0x06//bl6131z,bl6133u,blm13
#define BL6XX6      0x07//bl7450
#define BL7XX0      0x08//bl7450
#define BL7XX1      0x09
#define BL7XX3      0x0a

#if defined(CTP_USE_SW_I2C) || defined(BL6133_UPDATE_USE_SW_I2C)
    //8bit addr
    #define BL_FLASH_I2C_ADDR       (0x2c<<1 )
#else
    #define BL_FLASH_I2C_ADDR       (0x2c )
#endif
#define I2C_WRITE               0x00
#define I2C_READ                0x01

#if defined(CTP_USE_SW_I2C) || defined(BL6133_UPDATE_USE_SW_I2C)
    #define FLASH_WSIZE             256 //0x128
    #define FLASH_RSIZE             64  //0x128
    #define PROJECT_INFO_LEN        20  //ProjectInfo
#elif defined(CTP_USE_HW_I2C)
    #define FLASH_WSIZE             256 //2
    #define FLASH_RSIZE             64 //8
    #define PROJECT_INFO_LEN        20 //8  // ProjectInfo
#endif

#define ARGU_MARK  "chp_cfg_mark"

#if(TS_CHIP == BL8XXX_60)
#define BL_ARGUMENT_BASE_OFFSET 0x200
#define MAX_POINT_NUM   2
#define BTL_FLASH_ID    0x05
#define MAX_FLASH_SIZE  0x8000
#define PJ_ID_OFFSET    0xcb

#define VERTIFY_START_OFFSET 0x3fc
#define VERTIFY_END_OFFSET   0x3fd

#define FW_CHECKSUM_DELAY_TIME  100


#define UPDATE_MODE             INT_UPDATE_MODE
enum BL8XXX_60_flash_cmd
{

    ERASE_SECTOR_MAIN_CMD   = 0x06,
    ERASE_ALL_MAIN_CMD  = 0x09,
    RW_REGISTER_CMD     = 0x0a,
    READ_MAIN_CMD       = 0x0D,
    WRITE_MAIN_CMD      = 0x0F,
    WRITE_RAM_CMD       = 0x11,
    READ_RAM_CMD        = 0x12,
};

#elif(TS_CHIP == BL8XXX_61)
#define BL_ARGUMENT_BASE_OFFSET 0x200
#define MAX_POINT_NUM   5
#define BTL_FLASH_ID    0x06
#define MAX_FLASH_SIZE  0x8000
#define PJ_ID_OFFSET    0xcb

#define VERTIFY_START_OFFSET 0x3fc
#define VERTIFY_END_OFFSET   0x3fd

#define FW_CHECKSUM_DELAY_TIME  100


enum BL8XXX_61_flash_cmd
{

    ERASE_SECTOR_MAIN_CMD   = 0x06,
    ERASE_ALL_MAIN_CMD  = 0x09,
    RW_REGISTER_CMD     = 0x0a,
    READ_MAIN_CMD       = 0x0D,
    WRITE_MAIN_CMD      = 0x0F,
    WRITE_RAM_CMD       = 0x11,
    READ_RAM_CMD        = 0x12,
};

#elif(TS_CHIP == BL8XXX_63)
#define BL_ARGUMENT_BASE_OFFSET 0x200
#define MAX_POINT_NUM   2
#define BTL_FLASH_ID    0x04
#define MAX_FLASH_SIZE  0xc000
#define PJ_ID_OFFSET    0xcb

#define VERTIFY_START_OFFSET 0x3fc
#define VERTIFY_END_OFFSET   0x3fd

#define FW_CHECKSUM_DELAY_TIME  250


enum BL8XXX_63_flash_cmd
{

    ERASE_SECTOR_MAIN_CMD   = 0x06,
    ERASE_ALL_MAIN_CMD  = 0x09,
    RW_REGISTER_CMD     = 0x0a,
    READ_MAIN_CMD       = 0x0D,
    WRITE_MAIN_CMD      = 0x0F,
    WRITE_RAM_CMD       = 0x11,
    READ_RAM_CMD        = 0x12,
};
#elif(TS_CHIP == BL6XX0)
#define BL_ARGUMENT_BASE_OFFSET 0x200
#define MAX_POINT_NUM   2
#define BTL_FLASH_ID    0x20
#define MAX_FLASH_SIZE  0x8000
#define PJ_ID_OFFSET    0xcb

#define VERTIFY_START_OFFSET 0x3fc
#define VERTIFY_END_OFFSET   0x3fd

#define FW_CHECKSUM_DELAY_TIME  250


enum BL6XX0_flash_cmd
{

    ERASE_SECTOR_MAIN_CMD   = 0x06,
    ERASE_ALL_MAIN_CMD  = 0x09,
    RW_REGISTER_CMD     = 0x0a,
    READ_MAIN_CMD       = 0x0D,
    WRITE_MAIN_CMD      = 0x0F,
    WRITE_RAM_CMD       = 0x11,
    READ_RAM_CMD        = 0x12,
};
#elif(TS_CHIP == BL6XX1)
#define BL_ARGUMENT_BASE_OFFSET 0x200
#define MAX_POINT_NUM   2
#define BTL_FLASH_ID    0x21
#define MAX_FLASH_SIZE  0x8000
#define PJ_ID_OFFSET    0xcb

#define VERTIFY_START_OFFSET 0x3fc
#define VERTIFY_END_OFFSET   0x3fd

#define FW_CHECKSUM_DELAY_TIME  250


enum BL6XX1_flash_cmd
{

    ERASE_SECTOR_MAIN_CMD   = 0x06,
    ERASE_ALL_MAIN_CMD  = 0x09,
    RW_REGISTER_CMD     = 0x0a,
    READ_MAIN_CMD       = 0x0D,
    WRITE_MAIN_CMD      = 0x0F,
    WRITE_RAM_CMD       = 0x11,
    READ_RAM_CMD        = 0x12,
};
#elif(TS_CHIP == BL6XX3)
#define BL_ARGUMENT_BASE_OFFSET 0x200
#define MAX_POINT_NUM   1
#define BTL_FLASH_ID    0x22
#define MAX_FLASH_SIZE  0x8000
#define PJ_ID_OFFSET    0xcb

#define VERTIFY_START_OFFSET 0x3fc
#define VERTIFY_END_OFFSET   0x3fd

#define FW_CHECKSUM_DELAY_TIME  250


enum BL6XX3_flash_cmd
{

    ERASE_SECTOR_MAIN_CMD   = 0x06,
    ERASE_ALL_MAIN_CMD  = 0x09,
    RW_REGISTER_CMD     = 0x0a,
    READ_MAIN_CMD       = 0x0D,
    WRITE_MAIN_CMD      = 0x0F,
    WRITE_RAM_CMD       = 0x11,
    READ_RAM_CMD        = 0x12,
};
#elif(TS_CHIP == BL6XX6)
#define BL_ARGUMENT_BASE_OFFSET 0xfc00
#define MAX_POINT_NUM   2
#define BTL_FLASH_ID    0x42
#define MAX_FLASH_SIZE  0x10000
#define PJ_ID_OFFSET    0xcb

#define VERTIFY_START_OFFSET 0x23
#define VERTIFY_END_OFFSET   0x24

#define FW_CHECKSUM_DELAY_TIME  250
enum BL6XX6_flash_cmd
{

    ERASE_SECTOR_MAIN_CMD   = 0x06,
    ERASE_ALL_MAIN_CMD  = 0x09,
    RW_REGISTER_CMD     = 0x0a,
    READ_MAIN_CMD       = 0x0D,
    WRITE_MAIN_CMD      = 0x0F,
    WRITE_RAM_CMD       = 0x11,
    READ_RAM_CMD        = 0x12,
};

#elif(TS_CHIP == BL7XX0)
#define BL_ARGUMENT_BASE_OFFSET 0xbc00
#define MAX_POINT_NUM   5
#define BTL_FLASH_ID    0x40
#define MAX_FLASH_SIZE  0xc000
#define PJ_ID_OFFSET    0xcb

#define VERTIFY_START_OFFSET 0x23
#define VERTIFY_END_OFFSET   0x24

#define FW_CHECKSUM_DELAY_TIME  250

enum BL7XX0_flash_cmd
{

    ERASE_SECTOR_MAIN_CMD   = 0x06,
    ERASE_ALL_MAIN_CMD  = 0x09,
    RW_REGISTER_CMD     = 0x0a,
    READ_MAIN_CMD       = 0x0D,
    WRITE_MAIN_CMD      = 0x0F,
    WRITE_RAM_CMD       = 0x11,
    READ_RAM_CMD        = 0x12,
};
#elif(TS_CHIP == BL7XX1)
#define BL_ARGUMENT_BASE_OFFSET 0xfc00
#define MAX_POINT_NUM   10
#define BTL_FLASH_ID    0x41
#define MAX_FLASH_SIZE  0x10000
#define PJ_ID_OFFSET    0xcb

#define VERTIFY_START_OFFSET 0x23
#define VERTIFY_END_OFFSET   0x24

#define FW_CHECKSUM_DELAY_TIME  250

enum BL7XX1_flash_cmd
{

    ERASE_SECTOR_MAIN_CMD   = 0x06,
    ERASE_ALL_MAIN_CMD  = 0x09,
    RW_REGISTER_CMD     = 0x0a,
    READ_MAIN_CMD       = 0x0D,
    WRITE_MAIN_CMD      = 0x0F,
    WRITE_RAM_CMD       = 0x11,
    READ_RAM_CMD        = 0x12,
};
#elif(TS_CHIP == BL7XX3)
#define BL_ARGUMENT_BASE_OFFSET 0xfc00
#define MAX_POINT_NUM   10
#define BTL_FLASH_ID    0x42
#define MAX_FLASH_SIZE  0x10000
#define PJ_ID_OFFSET    0xcb

#define VERTIFY_START_OFFSET 0x23
#define VERTIFY_END_OFFSET   0x24

#define FW_CHECKSUM_DELAY_TIME  250


enum BL7XX3_flash_cmd
{

    ERASE_SECTOR_MAIN_CMD   = 0x06,
    ERASE_ALL_MAIN_CMD  = 0x09,
    RW_REGISTER_CMD     = 0x0a,
    READ_MAIN_CMD       = 0x0D,
    WRITE_MAIN_CMD      = 0x0F,
    WRITE_RAM_CMD       = 0x11,
    READ_RAM_CMD        = 0x12,
};
#endif

enum fw_reg
{

    WORK_MODE_REG       = 0x00,
    TS_DATA_REG         = 0x01,
    CHECKSUM_REG        = 0x3f,
    CHECKSUM_CAL_REG    = 0x8a,
    AC_REG              = 0x8b,
    RESOLUTION_REG      = 0x98,
    LPM_REG             = 0xa5,
    PROXIMITY_REG       = 0xb0,
    PROXIMITY_FLAG_REG  = 0xB1,
    CALL_REG            = 0xb2,
    BL_CHIP_ID_REG      = 0xb8,
    BL_FWVER_PJ_ID_REG  = 0xb6,
    BL_PRJ_INFO_REG     = 0xb4,
    CHIP_ID_REG         = 0xe7,
    BL_PRJ_ID_REG       = 0xb5,
    COB_ID_REG          = 0x33,
    BL_PROTECT_REG      = 0xee,
};

enum checksum
{

    CHECKSUM_READY      = 0x01,
    CHECKSUM_CAL        = 0xaa,
    CHECKSUM_ARG        = 0xba,
};

enum update_type
{

    NONE_UPDATE     = 0x00,
    FW_ARG_UPDATE   = 0x01,
};

typedef enum
{
    CTP_UP   = 0,
    CTP_DOWN,
} ctp_pen_state_enum;

#define  TD_STAT_ADDR           0x1
#define  TD_STAT_NUMBER_TOUCH   0x07
#define  CTP_PATTERN                    0xAA

#define OUTPUT  1
#define INPUT   0
/*************Betterlife ic update***********/
#ifdef      BL_UPDATE_FIRMWARE_ENABLE
    #define     BL_FWVER_MAIN_OFFSET    (0x2a)
    #define     BL_FWVER_ARGU_OFFSET    (0x2b)
    #define     BL_PROJECT_ID_OFFSET    (0x2c)
    #define     BL_PROJECT_INFO_OFFSET  (0x0f)
    #define     BL_COB_ID_OFFSET        (0x34)
    #define     BL_COB_ID_LEN           (12)
    #define     BL_ARGUMENT_FLASH_SIZE  (1024)
    #define     PRJ_INFO_LEN            (0x08)
    #define     FLASH_PAGE_SIZE         (512)
    //Update firmware through driver probe procedure with h and c file
    #define     BL_AUTO_UPDATE_FARMWARE
    #ifdef      BL_AUTO_UPDATE_FARMWARE

    #endif
#endif

/*************Betterlife ic debug***********/
#if defined(BL_DEBUG)
    #define bl_log_trace(...)        LOG_I(__VA_ARGS__)
#else
    #define bl_log_trace(...)
#endif
#define BTL_DRIVER_VERSION    "betterlife_driver_version_v1.1.9"

#endif

