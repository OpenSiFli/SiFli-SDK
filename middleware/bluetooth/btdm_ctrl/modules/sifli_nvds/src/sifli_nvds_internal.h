/**
 ****************************************************************************************
 *
 * @file sifli_nvds_internal.h
 *
 * @brief SiFli NVDS internal definition.
 *
 * Copyright (C) SiFli 2019-2022
 *
 *
 ****************************************************************************************
 */


#ifndef _SIFLI_NVDS_INTERNAL_H
#define _SIFLI_NVDS_INTERNAL_H

/*
 * INCLUDE FILES
 ****************************************************************************************
*/


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
*/


//#define SIFLI_NVDS_STACK_MAX_SIZE (NVDS_LEN_BD_ADDRESS + NVDS_LEN_DEVICE_NAME + NVDS_LEN_APP_BLE_ADV_DATA)
#define SIFLI_NVDS_STACK_MAX_SIZE 256
#define SIFLI_NVDS_APP_MAX_SIZE 256



typedef struct
{
    sifli_nvds_state_t state;
    uint16_t operation;
    uint8_t *stack_buff;
    uint16_t stack_total_len;
} sifli_nvds_env_t;





#define SIFLI_NVDS_GET_MSG(x)     (x & 0xFF)
#define SIFLI_NVDS_GET_TYPE(x)   ((((uint16_t)x) >> 8) & 0xFF)
#define SIFLI_NVDS_BUILD_OP(x, t) (uint16_t)(SIFLI_NVDS_GET_MSG(x) | ((uint16_t)t << 8))

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
*/


#endif // _SIFLI_NVDS_INTERNAL_H



