/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bf0_hal.h"

/** @addtogroup BF0_HAL_Driver
 * @{
 */

/** @defgroup SECU Security
 * @brief Security HAL module driver
 * @{
 */

#if defined(HAL_SECU_MODULE_ENABLED)||defined(_SIFLI_DOXYGEN_)

typedef struct
{
    uint8_t  memory_type;
    uint8_t  secu_flag;
    uint8_t  reg_cnt;
    uint8_t  reserved;
    uint32_t *reg_addr;
} MemRegInfo;

static const MemRegInfo MemRegList[] =
{
    {SECU_MEM_MPI1, SECU_FLAG_PRIV, 2, 0, (uint32_t *) &hwp_secu1->MPI1_PRIV_CFG0},
    {SECU_MEM_MPI1, SECU_FLAG_SECU, 2, 0, (uint32_t *) &hwp_secu1->MPI1_SEC_CFG0},
    {SECU_MEM_MPI2, SECU_FLAG_PRIV, 2, 0, (uint32_t *) &hwp_secu1->MPI2_PRIV_CFG0},
    {SECU_MEM_MPI2, SECU_FLAG_SECU, 2, 0, (uint32_t *) &hwp_secu1->MPI2_SEC_CFG0},
    {SECU_MEM_HPSYS_RAM0, SECU_FLAG_PRIV, 1, 0, (uint32_t *) &hwp_secu1->RAM0_PRIV_CFG0},
    {SECU_MEM_HPSYS_RAM0, SECU_FLAG_SECU, 1, 0, (uint32_t *) &hwp_secu1->RAM0_SEC_CFG0},
    {SECU_MEM_HPSYS_RAM1, SECU_FLAG_PRIV, 1, 0, (uint32_t *) &hwp_secu1->RAM1_PRIV_CFG0},
    {SECU_MEM_HPSYS_RAM1, SECU_FLAG_SECU, 1, 0, (uint32_t *) &hwp_secu1->RAM1_SEC_CFG0},
    {SECU_MEM_HPSYS_RAM2, SECU_FLAG_PRIV, 1, 0, (uint32_t *) &hwp_secu1->RAM2_PRIV_CFG0},
    {SECU_MEM_HPSYS_RAM2, SECU_FLAG_SECU, 1, 0, (uint32_t *) &hwp_secu1->RAM2_SEC_CFG0},

    {SECU_MEM_LPSYS_RAM0, SECU_FLAG_PRIV, 1, 0, (uint32_t *) &hwp_secu2->RAM0_PRIV_CFG0},
    {SECU_MEM_LPSYS_RAM0, SECU_FLAG_SECU, 1, 0, (uint32_t *) &hwp_secu2->RAM0_SEC_CFG0},
    {SECU_MEM_LPSYS_RAM1, SECU_FLAG_PRIV, 1, 0, (uint32_t *) &hwp_secu2->RAM1_PRIV_CFG0},
    {SECU_MEM_LPSYS_RAM1, SECU_FLAG_SECU, 1, 0, (uint32_t *) &hwp_secu2->RAM1_SEC_CFG0}
};

HAL_StatusTypeDef HAL_SECU_SetAttr(SECU_MODULE_TYPE module, uint32_t role, uint32_t flag)
{
    volatile uint32_t *p_reg;
    uint32_t bit_n, tmp;
    if (role & SECU_ROLE_MASTER)
    {
        switch (module)
        {
        /*SECU1*/
        case SECU_MOD_PTC1:
            p_reg = &(hwp_secu1->HPMST_ATTR_CFG);
            bit_n = SECU1_HPMST_ATTR_CFG_PTC1_SEC_Pos;
            break;
        case SECU_MOD_DMAC1:
            p_reg = &(hwp_secu1->HPMST_ATTR_CFG);
            bit_n = SECU1_HPMST_ATTR_CFG_DMAC1_SEC_Pos;
            break;
        case SECU_MOD_USBC:
            p_reg = &(hwp_secu1->HPMST_ATTR_CFG);
            bit_n = SECU1_HPMST_ATTR_CFG_USB_SEC_Pos;
            break;
        case SECU_MOD_AES:
            p_reg = &(hwp_secu1->HPMST_ATTR_CFG);
            bit_n = SECU1_HPMST_ATTR_CFG_AES_SEC_Pos;
            break;
        case SECU_MOD_LCDC1:
            p_reg = &(hwp_secu1->HPMST_ATTR_CFG);
            bit_n = SECU1_HPMST_ATTR_CFG_LCDC_SEC_Pos;
            break;
        case SECU_MOD_EPIC:
            p_reg = &(hwp_secu1->HPMST_ATTR_CFG);
            bit_n = SECU1_HPMST_ATTR_CFG_EPIC_SEC_Pos;
            break;
        case SECU_MOD_EXTDMA:
            p_reg = &(hwp_secu1->HPMST_ATTR_CFG);
            bit_n = SECU1_HPMST_ATTR_CFG_EXTDMA_SEC_Pos;
            break;
        case SECU_MOD_HCPU:
            p_reg = &(hwp_secu1->HPMST_ATTR_CFG);
            bit_n = SECU1_HPMST_ATTR_CFG_HCPU_SEC_Pos;
            break;



        /*SECU2*/
        case SECU_MOD_LCPU:
            p_reg = &(hwp_secu2->LPMST_ATTR_CFG);
            bit_n = SECU2_LPMST_ATTR_CFG_LCPU_SEC_Pos;
            break;
        case SECU_MOD_PTC2:
            p_reg = &(hwp_secu2->LPMST_ATTR_CFG);
            bit_n = SECU2_LPMST_ATTR_CFG_PTC2_SEC_Pos;
            break;
        case SECU_MOD_DMAC2:
            p_reg = &(hwp_secu2->LPMST_ATTR_CFG);
            bit_n = SECU2_LPMST_ATTR_CFG_DMAC2_SEC_Pos;
            break;

        default:
            HAL_ASSERT(0);
            return HAL_ERROR;
        }

        tmp = *p_reg;
        if ((module != SECU_MOD_HCPU) && (module != SECU_MOD_LCPU)) //No privilege bit for HCPU&LCPU
        {
            tmp |= 1 << (bit_n + 3); //set xxx_priv_use
            if (flag & SECU_FLAG_PRIV)
            {
                tmp |= 1 << (bit_n + 1); //set xxx_priv
            }
            else
            {
                tmp &= ~(1 << (bit_n + 1)); //clear xxx_priv
            }
        }

        tmp |= 1 << (bit_n + 2); //set xxx_sec_use
        if (flag & SECU_FLAG_SECU)
        {
            tmp |= 1 << (bit_n + 0); //set xxx_sec
        }
        else
        {
            tmp &= ~(1 << (bit_n + 0)); //clear xxx_sec
        }

        *p_reg = tmp;

        //Apply immediately
        if (p_reg == &(hwp_secu1->HPMST_ATTR_CFG))
            hwp_secu1->SECU_CTRL = SECU1_SECU_CTRL_HPMST_ATTR_UPDATE;
        else if (p_reg == &(hwp_secu2->LPMST_ATTR_CFG))
            hwp_secu2->SECU_CTRL = SECU2_SECU_CTRL_LPMST_ATTR_UPDATE;
        else
            HAL_ASSERT(0);
    }


    if (role & SECU_ROLE_SLAVE)
    {
        switch (module)
        {
        /*SECU1*/
        case SECU_MOD_PTC1:
            p_reg = &(hwp_secu1->HPSLV_ATTR_CFG);
            bit_n = SECU1_HPSLV_ATTR_CFG_PTC1_SEC_Pos;
            break;
        case SECU_MOD_DMAC1:
            p_reg = &(hwp_secu1->HPSLV_ATTR_CFG);
            bit_n = SECU1_HPSLV_ATTR_CFG_DMAC1_SEC_Pos;
            break;
        case SECU_MOD_AES:
            p_reg = &(hwp_secu1->HPSLV_ATTR_CFG);
            bit_n = SECU1_HPSLV_ATTR_CFG_AES_SEC_Pos;
            break;
        case SECU_MOD_TRNG:
            p_reg = &(hwp_secu1->HPSLV_ATTR_CFG);
            bit_n = SECU1_HPSLV_ATTR_CFG_TRNG_SEC_Pos;
            break;
        case SECU_MOD_EFUSE:
            p_reg = &(hwp_secu1->HPSLV_ATTR_CFG);
            bit_n = SECU1_HPSLV_ATTR_CFG_EFUSE_SEC_Pos;
            break;


        /*SECU2*/
        case SECU_MOD_PTC2:
            p_reg = &(hwp_secu2->LPSLV_ATTR_CFG);
            bit_n = SECU2_LPSLV_ATTR_CFG_PTC2_SEC_Pos;
            break;
        case SECU_MOD_DMAC2:
            p_reg = &(hwp_secu2->LPSLV_ATTR_CFG);
            bit_n = SECU2_LPSLV_ATTR_CFG_DMAC2_SEC_Pos;
            break;

        default:
            HAL_ASSERT(0);
            return HAL_ERROR;
        }

        tmp = *p_reg;

        if (flag & SECU_FLAG_PRIV)
        {
            tmp |= 1 << (bit_n + 1); //set xxx_priv
        }
        else
        {
            tmp &= ~(1 << (bit_n + 1)); //clear xxx_priv
        }

        if (flag & SECU_FLAG_SECU)
        {
            tmp |= 1 << (bit_n + 0); //set xxx_sec
        }
        else
        {
            tmp &= ~(1 << (bit_n + 0)); //clear xxx_sec
        }

        *p_reg = tmp;

    }

    return HAL_OK;
}

/*Treat S-bus C-bus access as same address*/
#define SECU_MEM_ADDR(addr) ((addr) & 0x0FFFFFFF)
#define SECU_HPMEM_ADDR(addr) ((addr) & 0x00FFFFFF)

HAL_StatusTypeDef HAL_SECU_SetMemoryAttr(SECU_MEM_TYPE memory_type, const SECU_MemAttr_Type *attrs, uint32_t attrs_cnt)
{
    uint32_t i, j, k;
    for (i = 0; i < attrs_cnt; i++)
    {
        if ((SECU_MEMRange_ALIGN(attrs[i].start) != attrs[i].start)
                || (SECU_MEMRange_ALIGN(attrs[i].end) != attrs[i].end))
        {
            return HAL_ERROR; //Address not aligned.
        }

        if (attrs[i].start >= attrs[i].end)
        {
            return HAL_ERROR;//end address must larger than start addr.
        }

        for (j = 0; j < sizeof(MemRegList) / sizeof(MemRegList[0]); j++)
        {
            if ((MemRegList[j].memory_type == memory_type) && (MemRegList[j].secu_flag == attrs[i].flag))
            {
                volatile uint32_t *start_addr, *end_addr;

                start_addr = MemRegList[j].reg_addr;
                end_addr   = start_addr + 1;
                for (k = 0; k < MemRegList[j].reg_cnt; k++)
                {

                    if (*start_addr > *end_addr)
                    {
                        if ((SECU_MEM_HPSYS_RAM0 == memory_type)
                                || (SECU_MEM_HPSYS_RAM1 == memory_type)
                                || (SECU_MEM_HPSYS_RAM2 == memory_type))
                        {
                            *start_addr = SECU_HPMEM_ADDR(attrs[i].start);
                            *end_addr   = SECU_HPMEM_ADDR(attrs[i].end - SECU_MEM_MIN_BLOCK);
                        }
                        else
                        {
                            *start_addr = SECU_MEM_ADDR(attrs[i].start) ;
                            *end_addr   = SECU_MEM_ADDR(attrs[i].end - SECU_MEM_MIN_BLOCK);
                        }

                        break; //Write attrs[i] success
                    }

                    //Move to next range registers
                    start_addr += 2;
                    end_addr   += 2;
                }

                if (MemRegList[j].reg_cnt == k)
                {
                    return HAL_ERROR; //None available memory register
                }

                break; //Write attrs[i] success
            }
        }


        if (sizeof(MemRegList) / sizeof(MemRegList[0]) == j)
        {
            return HAL_ERROR; //Can't find matched record in MemRegList
        }
    }


    return HAL_OK;
}


HAL_StatusTypeDef HAL_SECU_Lock(SECU_GROUP_TYPE group)
{
    switch (group)
    {
    case SECU_GROUP_HPMST:
        hwp_secu1->SECU_CTRL = SECU1_SECU_CTRL_HPMST_ATTR_UPDATE | SECU1_SECU_CTRL_HPMST_LOCK;
        break;
    case SECU_GROUP_HPSLV:
        hwp_secu1->SECU_CTRL = SECU1_SECU_CTRL_HPSLV_LOCK;
        break;
    case SECU_GROUP_HPMPI1:
        hwp_secu1->SECU_CTRL = SECU1_SECU_CTRL_MPI1_LOCK;
        break;
    case SECU_GROUP_HPMPI2:
        hwp_secu1->SECU_CTRL = SECU1_SECU_CTRL_MPI2_LOCK;
        break;
    case SECU_GROUP_HPRAM:
        hwp_secu1->SECU_CTRL = SECU1_SECU_CTRL_RAM_LOCK;
        break;

    case SECU_GROUP_LPMST:
        hwp_secu2->SECU_CTRL = SECU2_SECU_CTRL_LPMST_ATTR_UPDATE | SECU2_SECU_CTRL_LPMST_LOCK;
        break;
    case SECU_GROUP_LPSLV:
        hwp_secu2->SECU_CTRL = SECU2_SECU_CTRL_LPSLV_LOCK;
        break;
    case SECU_GROUP_LPRAM:
        hwp_secu2->SECU_CTRL = SECU2_SECU_CTRL_RAM_LOCK;
        break;
    default:
        return HAL_ERROR;
    }

    return HAL_OK;
}


#endif /* HAL_SECU_MODULE_ENABLED */
/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
