/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//#include <rtthread.h>
#include <register.h>
#include <string.h>
#include <stdlib.h>


#define PIN_MAX_NUM      12
#define FLASH_MAX_NUM    12
#define HEADER_EXT       0xabcddbca
//#define VERSION_EXT      0xffff0002
#define VERSION_EXT      0xffff0003

typedef struct
{
    uint8_t     pmic_disabele;
    uint8_t     scl_pin_type;
    uint8_t     scl_pin;
    uint8_t     sda_pin_type;
    uint8_t     sda_pin;
    uint8_t     reserved;
#if 1
    uint8_t     PMIC_channel_on[10];
#else
    uint8_t     PMIC_1V8_LVW100_1_on;
    uint8_t     PMIC_1V8_LVW100_2_on;
    uint8_t     PMIC_1V8_LVW100_3_on;
    uint8_t     PMIC_1V8_LVW100_4_on;
    uint8_t     PMIC_1V8_LVW100_5_on;
    uint8_t     PMIC_VBAT_HVSW150_1_on;
    uint8_t     PMIC_VBAT_HVSW150_2_on;
    uint8_t     PMIC_LDO33_VOUT_on;
    uint8_t     PMIC_LDO30_VOUT_on;
    uint8_t     PMIC_LDO28_VOUT_on;
#endif
} T_EXT_PMIC_CFG;

typedef struct
{
    uint8_t     type;
    uint8_t     pin;
    uint8_t     level;
    uint8_t     reserved;
} T_EXT_PIN_CFG;   //4 BYTES

typedef struct
{
    uint8_t    isnand;
    uint8_t    type;            //0-5
    uint8_t    reserved[2];
    uint8_t    manufacture_id;
    uint8_t    memory_type;
    uint8_t    memory_density;
    uint8_t    ext_flags;
    uint32_t   mem_size;
} T_EXT_FLASH_CFG;    //12 BYTES

typedef struct
{
    uint32_t    addr;
    uint8_t     pinmux_idx;
    uint8_t     init_idx;
    uint8_t     reserved[2];
} T_EXT_SD0_CFG;   //4 BYTES

typedef struct
{
    uint32_t h_flag;               //4 BYTES
    uint32_t version;              //4 BYTES
    uint16_t pin_mask;             //2 BYTE
    uint16_t flash_mask;           //2 BYTE
    uint8_t  pmic_mask;            //2 BYTE
    uint8_t  sd0_mask;             //2 BYTE
    uint8_t  reserved[2];          //2 BYTE
    //uint8_t flash_freq[4];         //4 BYTES
    T_EXT_PIN_CFG pin_cfg[PIN_MAX_NUM];       //4*12=48 BYTES
    T_EXT_FLASH_CFG flash_cfg[FLASH_MAX_NUM]; //12*12=144 BYTES
    T_EXT_PMIC_CFG pmic_cfg;       //16 BYTES
    T_EXT_SD0_CFG sd0_cfg;         //8 BYTES
    uint32_t t_flag;               //4 BYTES
} T_EXT_DRIVER_CFG;

__USED const T_EXT_DRIVER_CFG g_ext_driver_cfg2 =  
{
    HEADER_EXT,  //h_flag
    VERSION_EXT,  //version
    0x0000,   //pin_mask
    0x0000,   //flash_mask
    0x00,     //pmic_mask
    0x00,     //sd0_mask
    0x00,     //reserved
    0x00,     //reserved
    //0x00,   //flash_freq[0]
    //0x00,   //flash_freq[1]
    //0x00,   //flash_freq[2]
    //0x00,   //flash_freq[3]
    {   {0x00, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x00, 0x00},
        {0x00, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x00, 0x00},
        {0x00, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x00, 0x00}
    },
    {   {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00000000}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00000000},
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00000000}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00000000},
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00000000}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00000000},
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00000000}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00000000},
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00000000}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00000000},
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00000000}, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00000000}
    },
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {0x00000000, 0x00, 0x00, 0x00, 0x00},
    HEADER_EXT
};


volatile T_EXT_DRIVER_CFG g_ext_driver_cfg = {0};

void *get_user_flash_cfg(uint8_t isnand, uint8_t fid, uint8_t did, uint8_t type, uint8_t *flash_type)
{
    if(g_ext_driver_cfg.h_flag != HEADER_EXT )
    {
        memcpy((void *)&g_ext_driver_cfg, (void *)&g_ext_driver_cfg2, sizeof(T_EXT_DRIVER_CFG));
    }
    
    //rt_kprintf("get_user_flash_cfg: version %x mask %x %d %02x%02x%02x\n", g_ext_driver_cfg.version, g_ext_driver_cfg.flash_mask, isnand, fid, type, did);
    if(g_ext_driver_cfg.h_flag == HEADER_EXT && 
       g_ext_driver_cfg.t_flag == HEADER_EXT && 
       g_ext_driver_cfg.version == VERSION_EXT &&
       g_ext_driver_cfg.flash_mask > 0)
    {
        for(int m=0; m<FLASH_MAX_NUM; m++)
        {
            if(g_ext_driver_cfg.flash_mask & (0x1 << m)) 
            {
                if (isnand == g_ext_driver_cfg.flash_cfg[m].isnand && 
                    fid == g_ext_driver_cfg.flash_cfg[m].manufacture_id &&
                    type == g_ext_driver_cfg.flash_cfg[m].memory_type &&
                    did == g_ext_driver_cfg.flash_cfg[m].memory_density)
                {
                    if(flash_type)
                    {
                        *flash_type = g_ext_driver_cfg.flash_cfg[m].type;
                    }
                    //rt_kprintf("get_user_flash_cfg: isnand_%d type_%d id_%02x%02x%02x\n", isnand, g_ext_driver_cfg.flash_cfg[m].type, fid, type, did);
                    return (void *)&(g_ext_driver_cfg.flash_cfg[m].manufacture_id);
                }
            }
        }
    }
    
    return NULL;
}

bool get_user_sd0_cfg(uint32_t *pAddr, int8_t *pPinIdx, int8_t *pInitIdx)
{
    if (g_ext_driver_cfg.h_flag != HEADER_EXT)
    {
        memcpy((void *)&g_ext_driver_cfg, (void *)&g_ext_driver_cfg2, sizeof(T_EXT_DRIVER_CFG));
    }

    //rt_kprintf("get_user_sd0_cfg: version 0x%x mask 0x%x\n", g_ext_driver_cfg.version, g_ext_driver_cfg.sd0_mask);

    if (g_ext_driver_cfg.h_flag == HEADER_EXT &&
            g_ext_driver_cfg.t_flag == HEADER_EXT &&
            g_ext_driver_cfg.version == VERSION_EXT &&
            g_ext_driver_cfg.sd0_mask)
    {
        if(g_ext_driver_cfg.sd0_cfg.addr != 0 && pAddr)
        {
             *pAddr = g_ext_driver_cfg.sd0_cfg.addr;
        }
        
        if(pPinIdx)
        {
            *pPinIdx = g_ext_driver_cfg.sd0_cfg.pinmux_idx;
        }
        
        if(pInitIdx)
        {
            *pInitIdx = g_ext_driver_cfg.sd0_cfg.init_idx;
        }
        
        return true;
    }

    return false;
}

void user_pin_cfg()
{
    if(g_ext_driver_cfg.h_flag != HEADER_EXT )
    {
        memcpy((void *)&g_ext_driver_cfg, (void *)&g_ext_driver_cfg2, sizeof(T_EXT_DRIVER_CFG));
    }
    
    //rt_kprintf("user_pin_cfg: version 0x%x mask 0x%x\n", g_ext_driver_cfg.version, g_ext_driver_cfg.pin_mask);
    
    if(g_ext_driver_cfg.h_flag == HEADER_EXT && 
       g_ext_driver_cfg.t_flag == HEADER_EXT && 
       g_ext_driver_cfg.version == VERSION_EXT &&
       g_ext_driver_cfg.pin_mask > 0)
    {
        for(int m=0; m<PIN_MAX_NUM; m++)
        {
            if(g_ext_driver_cfg.pin_mask & (0x1 << m))
            {
                if(g_ext_driver_cfg.pin_cfg[m].type == 0)
                {
                    if(g_ext_driver_cfg.pin_cfg[m].level == 1)
                    {
                        HAL_PIN_Set(PAD_PA00+g_ext_driver_cfg.pin_cfg[m].pin, GPIO_A0+g_ext_driver_cfg.pin_cfg[m].pin, PIN_PULLUP, 1);
                    }
                    else
                    {
                        HAL_PIN_Set(PAD_PA00+g_ext_driver_cfg.pin_cfg[m].pin, GPIO_A0+g_ext_driver_cfg.pin_cfg[m].pin, PIN_PULLDOWN, 1);
                    }
                }
                else if(g_ext_driver_cfg.pin_cfg[m].type == 1)
                {
                    if(g_ext_driver_cfg.pin_cfg[m].level == 1)
                    {
                        HAL_PIN_Set(PAD_PB00+g_ext_driver_cfg.pin_cfg[m].pin, GPIO_B0+g_ext_driver_cfg.pin_cfg[m].pin, PIN_PULLUP, 0);
                    }
                    else
                    {
                        HAL_PIN_Set(PAD_PB00+g_ext_driver_cfg.pin_cfg[m].pin, GPIO_B0+g_ext_driver_cfg.pin_cfg[m].pin, PIN_PULLDOWN, 0);
                    }
                }
#if !defined(SF32LB55X)
                else
                {
                    HAL_PBR_ConfigMode(g_ext_driver_cfg.pin_cfg[m].pin, true);
                    HAL_PBR_WritePin(g_ext_driver_cfg.pin_cfg[m].pin, g_ext_driver_cfg.pin_cfg[m].level);
                }
#endif
            }
        }
    }
}


#if !defined(SF32LB52X)
#include "pmic_controller.h"

bool user_pmic_cfg()
{
    if(g_ext_driver_cfg.h_flag != HEADER_EXT )
    {
        memcpy((void *)&g_ext_driver_cfg, (void *)&g_ext_driver_cfg2, sizeof(T_EXT_DRIVER_CFG));
    }
    
    //rt_kprintf("user_pmic_cfg: version 0x%x mask 0x%x\n", g_ext_driver_cfg.version, g_ext_driver_cfg.pmic_mask);
    
    if(g_ext_driver_cfg.h_flag == HEADER_EXT && 
       g_ext_driver_cfg.t_flag == HEADER_EXT && 
       g_ext_driver_cfg.version == VERSION_EXT &&
       g_ext_driver_cfg.pmic_mask)
    {
        if(g_ext_driver_cfg.pmic_cfg.pmic_disabele)
        {
             return true;
        }
        int scl = g_ext_driver_cfg.pmic_cfg.scl_pin;
        int sda = g_ext_driver_cfg.pmic_cfg.sda_pin;
        
        if(g_ext_driver_cfg.pmic_cfg.scl_pin_type)
        {
            scl += 96;
        }
        if(g_ext_driver_cfg.pmic_cfg.sda_pin_type)
        {
            sda += 96;
        }
        BSP_PMIC_Init(scl, sda);
        
        if(g_ext_driver_cfg.pmic_cfg.PMIC_channel_on[0] == 1)
        {
            BSP_PMIC_Control(PMIC_OUT_1V8_LVSW100_1, 1, 1);
        }
        if(g_ext_driver_cfg.pmic_cfg.PMIC_channel_on[1] == 1)
        {
            BSP_PMIC_Control(PMIC_OUT_1V8_LVSW100_2, 1, 1);
        }
        if(g_ext_driver_cfg.pmic_cfg.PMIC_channel_on[2] == 1)
        {
            BSP_PMIC_Control(PMIC_OUT_1V8_LVSW100_3, 1, 1);
        }
        if(g_ext_driver_cfg.pmic_cfg.PMIC_channel_on[3] == 1)
        {
            BSP_PMIC_Control(PMIC_OUT_1V8_LVSW100_4, 1, 1);
        }
        if(g_ext_driver_cfg.pmic_cfg.PMIC_channel_on[4] == 1)
        {
            BSP_PMIC_Control(PMIC_OUT_1V8_LVSW100_5, 1, 1);
        }
        if(g_ext_driver_cfg.pmic_cfg.PMIC_channel_on[5] == 1)
        {
            BSP_PMIC_Control(PMIC_OUT_VBAT_HVSW150_1, 1, 1);
        }
        if(g_ext_driver_cfg.pmic_cfg.PMIC_channel_on[6] == 1)
        {
            BSP_PMIC_Control(PMIC_OUT_VBAT_HVSW150_2, 1, 1);
        }
        if(g_ext_driver_cfg.pmic_cfg.PMIC_channel_on[7] == 1)
        {
            pmic_device_control(PMIC_OUT_LDO33_VOUT, 1, 1);
        }
        if(g_ext_driver_cfg.pmic_cfg.PMIC_channel_on[8] == 1)
        {
            pmic_device_control(PMIC_OUT_LDO30_VOUT, 1, 1);
        }
        if(g_ext_driver_cfg.pmic_cfg.PMIC_channel_on[9] == 1)
        {
            pmic_device_control(PMIC_OUT_LDO28_VOUT, 1, 1);
        }
        
        
        return true;
    }
    
    return false;
}

#else
bool user_pmic_cfg()
{
    return true;
}
#endif


__weak void HAL_ADC_HwInit(bool cold_boot)
{

}

