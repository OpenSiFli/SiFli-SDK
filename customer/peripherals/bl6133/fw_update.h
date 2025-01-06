#ifndef    _FW_UPDATE_H_
#define    _FW_UPDATE_H_
/*****************************************************************
**头文件加载
*****************************************************************/
#if 1//defined( TSC_USING_BL6133)
    #include "bl_chip_common.h"
    #include "fw_update.h"

    /*****************************************************************
    **Extern
    *****************************************************************/
    extern int bl_get_chip_id(unsigned char *buf);
    #if defined(BL_AUTO_UPDATE_FARMWARE)
        extern int bl_auto_update_fw(void);
        extern int bl_i2c_transfer(unsigned char i2c_addr, unsigned char *buf, int len, unsigned char rw);
    #endif
    #ifdef I2C_UPDATE_MODE
        void bl_enter_update_with_i2c(void);
        void bl_exit_update_with_i2c(void);
    #endif
    #ifdef INT_UPDATE_MODE
        void bl_enter_update_with_int(void);
        void bl_exit_update_with_int(void);
    #endif
#endif
#endif
