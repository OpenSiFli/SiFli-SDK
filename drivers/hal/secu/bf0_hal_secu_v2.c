/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
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

static __IO uint32_t *Get_Master_Reg(SECU_MODULE_TYPE module, uint32_t *bit_n)
{
    __IO uint32_t *p_reg = NULL;
    HAL_ASSERT(bit_n != NULL);

    switch (module)
    {
    /*SECU1*/
    case SECU_MOD_PTC1:
        p_reg = &(hwp_secu1->HPMST_ATTR_CFG);
        *bit_n = SECU1_HPMST_ATTR_CFG_PTC1_SEC_Pos;
        break;
    case SECU_MOD_DMAC1:
        p_reg = &(hwp_secu1->HPMST_ATTR_CFG);
        *bit_n = SECU1_HPMST_ATTR_CFG_DMAC1_SEC_Pos;
        break;
    case SECU_MOD_USBC:
        p_reg = &(hwp_secu1->HPMST_ATTR_CFG);
        *bit_n = SECU1_HPMST_ATTR_CFG_USB_SEC_Pos;
        break;
    case SECU_MOD_AES:
        p_reg = &(hwp_secu1->HPMST_ATTR_CFG);
        *bit_n = SECU1_HPMST_ATTR_CFG_AES_SEC_Pos;
        break;
    case SECU_MOD_LCDC1:
        p_reg = &(hwp_secu1->HPMST_ATTR_CFG);
        *bit_n = SECU1_HPMST_ATTR_CFG_LCDC_SEC_Pos;
        break;
    case SECU_MOD_EPIC:
        p_reg = &(hwp_secu1->HPMST_ATTR_CFG);
        *bit_n = SECU1_HPMST_ATTR_CFG_EPIC_SEC_Pos;
        break;
    case SECU_MOD_ACPU:
        p_reg = &(hwp_secu1->HPMST_ATTR_CFG);
        *bit_n = SECU1_HPMST_ATTR_CFG_ACPU_SEC_Pos;
        break;
    case SECU_MOD_HCPU:
        p_reg = &(hwp_secu1->HPMST_ATTR_CFG);
        *bit_n = SECU1_HPMST_ATTR_CFG_HCPU_SEC_Pos;
        break;

    /*SECU2*/
    case SECU_MOD_PTC2:
        p_reg = &(hwp_secu2->LPMST_ATTR_CFG);
        *bit_n = SECU2_LPMST_ATTR_CFG_PTC2_SEC_Pos;
        break;
    case SECU_MOD_DMAC2:
        p_reg = &(hwp_secu2->LPMST_ATTR_CFG);
        *bit_n = SECU2_LPMST_ATTR_CFG_DMAC2_SEC_Pos;
        break;
    case SECU_MOD_LCPU:
        p_reg = &(hwp_secu2->LPMST_ATTR_CFG);
        *bit_n = SECU2_LPMST_ATTR_CFG_LCPU_SEC_Pos;
        break;
    default:
        HAL_ASSERT(0);
    }

    return p_reg;
}

static __IO uint32_t *Get_Slave_Reg(SECU_MODULE_TYPE module, uint32_t *bit_n)
{
    __IO uint32_t *p_reg = NULL;
    HAL_ASSERT(bit_n != NULL);

    switch (module)
    {
    /*SECU1*/
    case SECU_MOD_HCPU_DBG:
        p_reg = &(hwp_secu1->HPSLV_ATTR_CFG);
        *bit_n = SECU1_HPSLV_ATTR_CFG_HCPU_DBG_SEC_Pos;
        break;
    case SECU_MOD_ACPU_DBG:
        p_reg = &(hwp_secu1->HPSLV_ATTR_CFG);
        *bit_n = SECU1_HPSLV_ATTR_CFG_ACPU_DBG_SEC_Pos;
        break;
    case SECU_MOD_SECU1:
        p_reg = &(hwp_secu1->HPSLV_ATTR_CFG);
        *bit_n = SECU1_HPSLV_ATTR_CFG_SECU1_SEC_Pos;
        break;
    case SECU_MOD_PTC1:
        p_reg = &(hwp_secu1->HPSLV_ATTR_CFG);
        *bit_n = SECU1_HPSLV_ATTR_CFG_PTC1_SEC_Pos;
        break;
    case SECU_MOD_TRNG:
        p_reg = &(hwp_secu1->HPSLV_ATTR_CFG);
        *bit_n = SECU1_HPSLV_ATTR_CFG_TRNG_SEC_Pos;
        break;
    case SECU_MOD_EFUSE:
        p_reg = &(hwp_secu1->HPSLV_ATTR_CFG);
        *bit_n = SECU1_HPSLV_ATTR_CFG_EFUSE_SEC_Pos;
        break;
    case SECU_MOD_AES:
        p_reg = &(hwp_secu1->HPSLV_ATTR_CFG);
        *bit_n = SECU1_HPSLV_ATTR_CFG_AES_SEC_Pos;
        break;
    case SECU_MOD_DMAC1:
        p_reg = &(hwp_secu1->HPSLV_ATTR_CFG);
        *bit_n = SECU1_HPSLV_ATTR_CFG_DMAC1_SEC_Pos;
        break;

    /*SECU2*/
    case SECU_MOD_DMAC2:
        p_reg = &(hwp_secu2->LPSLV_ATTR_CFG);
        *bit_n = SECU2_LPSLV_ATTR_CFG_DMAC2_SEC_Pos;
        break;
    case SECU_MOD_PTC2:
        p_reg = &(hwp_secu2->LPSLV_ATTR_CFG);
        *bit_n = SECU2_LPSLV_ATTR_CFG_PTC2_SEC_Pos;
        break;
    case SECU_MOD_SECU2:
        p_reg = &(hwp_secu2->LPSLV_ATTR_CFG);
        *bit_n = SECU2_LPSLV_ATTR_CFG_SECU2_SEC_Pos;
        break;
    default:
        HAL_ASSERT(0);
    }

    return p_reg;
}

HAL_StatusTypeDef HAL_SECU_SetAttr(SECU_MODULE_TYPE module, uint32_t role, uint32_t flag)
{
    __IO uint32_t *p_reg;
    uint32_t bit_n, tmp;

    if (role & SECU_ROLE_MASTER)
    {
        p_reg = Get_Master_Reg(module, &bit_n);
        tmp = *p_reg;

        if (SECU_MOD_HCPU == module)
        {
            tmp |= SECU1_HPMST_ATTR_CFG_HCPU_SEC_USE;
            if (flag & SECU_FLAG_SECU)
                tmp |= SECU1_HPMST_ATTR_CFG_HCPU_SEC;
            else
                tmp &= ~SECU1_HPMST_ATTR_CFG_HCPU_SEC;
        }
        else if (SECU_MOD_ACPU == module)
        {
            tmp |= SECU1_HPMST_ATTR_CFG_ACPU_SEC_USE;
            if (flag & SECU_FLAG_SECU)
                tmp |= SECU1_HPMST_ATTR_CFG_ACPU_SEC;
            else
                tmp &= ~SECU1_HPMST_ATTR_CFG_ACPU_SEC;
        }
        else if (SECU_MOD_LCPU == module)
        {
            tmp |= SECU2_LPMST_ATTR_CFG_LCPU_SEC_USE;
            if (flag & SECU_FLAG_SECU)
                tmp |= SECU2_LPMST_ATTR_CFG_LCPU_SEC;
            else
                tmp &= ~SECU2_LPMST_ATTR_CFG_LCPU_SEC;
        }
        else
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


            tmp |= 1 << (bit_n + 2); //set xxx_sec_use
            if (flag & SECU_FLAG_SECU)
            {
                tmp |= 1 << (bit_n + 0); //set xxx_sec
            }
            else
            {
                tmp &= ~(1 << (bit_n + 0)); //clear xxx_sec
            }
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
        p_reg = Get_Slave_Reg(module, &bit_n);
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
/** SECU_RW  Security setting flags */
#define SECU_PRIV_RD_ATTR  (0x01UL) /* priviledge read attribute */
#define SECU_PRIV_WR_ATTR  (0x02UL) /* priviledge write attribute */
#define SECU_SEC_RD_ATTR   (0x04UL) /* secure read attribute */
#define SECU_SEC_WR_ATTR   (0x08UL) /* secure write attribute */

/** Treat S-bus C-bus access as same address*/
#define SECU_MEM_ADDR(addr) ((addr) & 0x0FFFFFFF)
#define SECU_HPMEM_ADDR(addr) ((addr) & 0x00FFFFFF)

/** SECU_MemAttr_Reg */
typedef struct
{
    __IO uint32_t *st_reg; /* start address config register addr */
    __IO uint32_t *end_reg; /* end address config register addr*/
    __IO uint32_t *rw_reg; /* read write config register addr*/
    uint32_t rg_of; /* config region offset, r0 r1 r2*/
} SECU_MemAttr_Reg;


static HAL_StatusTypeDef MemReg_CFG(SECU_MEM_TYPE memory_type, SECU_MemAttr_Reg *reg, uint32_t start, uint32_t end, uint32_t flag)
{
    uint32_t pos, mask;

    //1. Set region start&end address
    if ((memory_type == SECU_MEM_MPI1)
            || (memory_type == SECU_MEM_MPI2)
            || (memory_type == SECU_MEM_MPI3)) //mpi memory
    {
        *reg->st_reg = SECU_MEM_ADDR(start);
        *reg->end_reg   = SECU_MEM_ADDR(end - SECU_MEM_MIN_BLOCK);
    }
    else //sram memory
    {
        if ((memory_type == SECU_MEM_LPSYS_RAM0)
                || (memory_type == SECU_MEM_LPSYS_RAM1))
        {
            *reg->st_reg = SECU_MEM_ADDR(start);
            *reg->end_reg   = SECU_MEM_ADDR(end - SECU_SRAM_MIN_BLOCK);
        }
        else
        {
            *reg->st_reg = SECU_HPMEM_ADDR(start);
            *reg->end_reg   = SECU_HPMEM_ADDR(end - SECU_SRAM_MIN_BLOCK);
        }
    }

    //2. Set region read/write attributes
    pos = reg->rg_of * 4;
    mask = 0xFUL << pos;
    MODIFY_REG(*reg->rw_reg, mask, flag << pos);
    return HAL_OK;
}

static HAL_StatusTypeDef MemAddr_Check(SECU_MEM_TYPE memory_type, uint32_t start, uint32_t end)
{
    if (start >= end)
    {
        return HAL_ERROR;//end address must larger than start addr.
    }

    if ((memory_type == SECU_MEM_MPI1)
            || (memory_type == SECU_MEM_MPI2)
            || (memory_type == SECU_MEM_MPI3)) //mpi
    {
        if ((SECU_MEMRange_ALIGN(start) != start)
                || (SECU_MEMRange_ALIGN(end) != end))
        {
            return HAL_ERROR; //Address not aligned.
        }
    }
    else //sram
    {
        if ((SECU_SRAMRange_ALIGN(start) != start)
                || (SECU_SRAMRange_ALIGN(end) != end))
        {
            return HAL_ERROR; //Address not aligned.
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef MemReg_GET(SECU_MEM_TYPE memory_type, SECU_MemAttr_Reg *MemReg, uint32_t *rg_num)
{
    switch (memory_type)
    {
    case SECU_MEM_MPI1:
        MemReg->st_reg = &(hwp_secu1->MPI1_R0_CFG0);
        MemReg->end_reg = &(hwp_secu1->MPI1_R0_CFG1);
        MemReg->rw_reg = &(hwp_secu1->MPI1_RW_CFG);
        *rg_num = 3;
        break;
    case SECU_MEM_MPI2:
        MemReg->st_reg = &(hwp_secu1->MPI2_R0_CFG0);
        MemReg->end_reg = &(hwp_secu1->MPI2_R0_CFG1);
        MemReg->rw_reg = &(hwp_secu1->MPI2_RW_CFG);
        *rg_num = 3;
        break;
    case SECU_MEM_MPI3:
        MemReg->st_reg = &(hwp_secu1->MPI3_R0_CFG0);
        MemReg->end_reg = &(hwp_secu1->MPI3_R0_CFG1);
        MemReg->rw_reg = &(hwp_secu1->MPI3_RW_CFG);
        *rg_num = 3;
        break;
    case SECU_MEM_HPSYS_RAM1:
        MemReg->st_reg = &(hwp_secu1->RAM1_R0_CFG0);
        MemReg->end_reg = &(hwp_secu1->RAM1_R0_CFG1);
        MemReg->rw_reg = &(hwp_secu1->RAM1_RW_ATTR);
        *rg_num = 2;
        break;
    case SECU_MEM_HPSYS_RAM2:
        MemReg->st_reg = &(hwp_secu1->RAM2_R0_CFG0);
        MemReg->end_reg = &(hwp_secu1->RAM2_R0_CFG1);
        MemReg->rw_reg = &(hwp_secu1->RAM2_RW_ATTR);
        *rg_num = 2;
        break;
    case SECU_MEM_LPSYS_RAM0:
        MemReg->st_reg = &(hwp_secu2->RAM0_R0_CFG0);
        MemReg->end_reg = &(hwp_secu2->RAM0_R0_CFG1);
        MemReg->rw_reg = &(hwp_secu2->RAM0_RW_CFG);
        *rg_num = 2;
        break;
    case SECU_MEM_LPSYS_RAM1:
        MemReg->st_reg = &(hwp_secu2->RAM1_R0_CFG0);
        MemReg->end_reg = &(hwp_secu2->RAM1_R0_CFG1);
        MemReg->rw_reg = &(hwp_secu2->RAM1_RW_CFG);
        *rg_num = 2;
        break;
    default:
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef HAL_SECU_SetMemoryAttr(SECU_MEM_TYPE memory_type, const SECU_MemAttr_Type *attrs, uint32_t attrs_cnt)
{
    uint32_t i, j, rg_num = 0, flag = 0;
    SECU_MemAttr_Reg MemReg = {0};

    if (MemReg_GET(memory_type, &MemReg, &rg_num) != HAL_OK)
    {
        return HAL_ERROR;
    }

    for (i = 0; i < rg_num; i++) //find free mem region
    {
        if (*MemReg.st_reg > *MemReg.end_reg)
            break;
        MemReg.st_reg += 2;
        MemReg.end_reg += 2;
        MemReg.rg_of++;
    }

    if ((i == rg_num) || (attrs_cnt > (rg_num - MemReg.rg_of))) //unfind free mem region
    {
        return HAL_ERROR;
    }

    for (j = 0; j < attrs_cnt; j++)
    {
        if (MemAddr_Check(memory_type, attrs[j].start, attrs[j].end) != HAL_OK)
        {
            return HAL_ERROR;
        }
        flag = SECU_FLAG_NONE;
        if (attrs[j].flag & SECU_FLAG_PRIV)
            flag |= (SECU_PRIV_RD_ATTR | SECU_PRIV_WR_ATTR);
        if (attrs[j].flag & SECU_FLAG_SECU)
            flag |= (SECU_SEC_RD_ATTR | SECU_SEC_WR_ATTR);
        if (attrs[j].flag & SECU_FLAG_SEC_WR)
            flag = SECU_SEC_WR_ATTR;

        if (MemReg_CFG(memory_type, &MemReg, attrs[j].start, attrs[j].end, flag) != HAL_OK)
        {
            return HAL_ERROR;
        }

        MemReg.st_reg += 2;
        MemReg.end_reg += 2;
        MemReg.rg_of++;
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
    case SECU_GROUP_HPMPI3:
        hwp_secu1->SECU_CTRL = SECU1_SECU_CTRL_MPI3_LOCK;
        break;
    case SECU_GROUP_HPRAM:
        hwp_secu1->SECU_CTRL = SECU1_SECU_CTRL_RAM_LOCK;
        break;
    case SECU_GROUP_ACPU:
        hwp_secu1->ACPU |= SECU1_ACPU_LOCKNSMPU;
        hwp_secu1->ACPU |= SECU1_ACPU_LOCKNSVTOR;
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
        break;
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
