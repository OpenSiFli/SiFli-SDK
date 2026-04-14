/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "register.h"
#include <string.h>

#include "basetype.h"
#include "dwl_defs.h"
#include "dwl.h"
#include "dwlthread.h"
#include "register.h"

#ifdef SOLUTION
#include "app_mem.h"
#endif

#define DBG_LEVEL         DBG_ERROR  // DBG_LOG //

#define LOG_TAG                log_tag
#include "log.h"

#define DWL_DEBUG  LOG_D
#define DWL_LOG_I  LOG_I
#define DWL_LOG_E  LOG_E


#define DWL_INST_LOG_I(instance,...) do{log_tag=GetInstanceName(((DWLInstance_t *)(instance))->type); LOG_I(__VA_ARGS__);}while(0)


//#define MEMORY_USAGE_TRACE
#define DWL_DISABLE_REG_PRINTS




/* wrapper information */
typedef struct DWLInstance
{
    u32 type; //Like DWL_CLIENT_TYPE_JPEG_DEC, DWL_CLIENT_TYPE_PP
    u32 *RegBase;

    /* Keep track of allocated memories */
    u32 linearTotal;
    i32 linearAllocCount;

    struct rt_semaphore sem;
    u32 ReserveCnt;
} DWLInstance_t;


static DWLInstance_t *p_JPGDec_DWLInst = NULL;
static const char *log_tag = "DWL";

#ifdef _DWL_DEBUG
static void PrintIrqType(u32 isPP, u32 coreID, u32 status)
{
    if (isPP)
    {
        DWL_LOG_I("PP[%d] IRQ %s\n", coreID,
                  status & PP_IRQ_RDY ? "READY" : "BUS ERROR");
    }
    else
    {
        if (status & DEC_IRQ_ABORT)
            DWL_LOG_I("DEC[%d] IRQ ABORT\n", coreID);
        else if (status & DEC_IRQ_RDY)
            DWL_LOG_I("DEC[%d] IRQ READY\n", coreID);
        else if (status & DEC_IRQ_BUS)
            DWL_LOG_I("DEC[%d] IRQ BUS ERROR\n", coreID);
        else if (status & DEC_IRQ_BUFFER)
            DWL_LOG_I("DEC[%d] IRQ BUFFER\n", coreID);
        else if (status & DEC_IRQ_ASO)
            DWL_LOG_I("DEC[%d] IRQ ASO\n", coreID);
        else if (status & DEC_IRQ_ERROR)
            DWL_LOG_I("DEC[%d] IRQ STREAM ERROR\n", coreID);
        else if (status & DEC_IRQ_SLICE)
            DWL_LOG_I("DEC[%d] IRQ SLICE\n", coreID);
        else if (status & DEC_IRQ_TIMEOUT)
            DWL_LOG_I("DEC[%d] IRQ TIMEOUT\n", coreID);
        else
            DWL_LOG_I("DEC[%d] IRQ UNKNOWN 0x%08x\n", coreID, status);
    }
}
#endif /* _DWL_DEBUG */

static const char *GetInstanceName(u32 type)
{
    switch (type)
    {
    case DWL_CLIENT_TYPE_JPEG_DEC:
        return "JPEG_DEC";
    case DWL_CLIENT_TYPE_PP:
        return "JPEG_PP";
    default:
        return "DWL";
    }
}
void JPEG_DEC_IRQHandler(void)
{
    uint32_t swreg1;
    uint8_t release_sem = 0;
    /* enter interrupt */
    rt_interrupt_enter();


    swreg1 = hwp_jpeg_dec->SWREG1;

    DWL_LOG_I("JPEG_DEC_IRQHandler,%x", swreg1);



    if (swreg1 & JPEGDEC_SWREG1_DEC_IRQ)
    {
        hwp_jpeg_dec->SWREG1 |= JPEGDEC_SWREG1_DEC_IRQ; //Clear irq
        HAL_NVIC_DisableIRQ(JPEG_DEC_IRQn);
    }

    if (swreg1 & JPEGDEC_SWREG1_DEC_TIMEOUT)
    {
        DWL_LOG_E("JPEG_DEC_IRQHandler timeout");
        release_sem = 1;
    }

    if (swreg1 & JPEGDEC_SWREG1_DEC_ERROR_INT)
    {
        DWL_LOG_E("JPEG_DEC_IRQHandler Error");
        release_sem = 1;
    }


    if (swreg1 & JPEGDEC_SWREG1_DEC_RDY_INT) release_sem = 1;

    if(release_sem)
    {
        if(p_JPGDec_DWLInst)
        {
            rt_err_t err;
            err = rt_sem_release(&p_JPGDec_DWLInst->sem);
            RT_ASSERT(err == RT_EOK);
        }
    }


    /* leave interrupt */
    rt_interrupt_leave();
}

/*------------------------------------------------------------------------------
    Function name   : DWLReadAsicCoreCount
    Description     : Return the number of hardware cores available
------------------------------------------------------------------------------*/
u32 DWLReadAsicCoreCount(void)
{
    return (u32)1;
}

/*------------------------------------------------------------------------------
    Function name   : DWLReadAsicID
    Description     : Read the HW ID. Does not need a DWL instance to run

    Return type     : u32 - the HW ID
------------------------------------------------------------------------------*/
u32 DWLReadAsicID()
{
    u32 id = ~0;

    DWL_DEBUG("\n");


    return id;
}

/*------------------------------------------------------------------------------
    Function name   : DWLReadAsicConfig
    Description     : Read HW configuration. Does not need a DWL instance to run

    Return type     : DWLHwConfig_t - structure with HW configuration
------------------------------------------------------------------------------*/
void DWLReadAsicConfig(DWLHwConfig_t *pHwCfg)
{
    RT_ASSERT(pHwCfg);
    memset((void *)pHwCfg, 0, sizeof(DWLHwConfig_t));

    pHwCfg->jpegSupport = JPEG_BASELINE;
    pHwCfg->ppSupport = PP_SUPPORTED;

#if 0

    tmp = 0;
    if (tbCfg->ppParams.ditheringSupport)  tmp |= PP_DITHERING;
    if (tbCfg->ppParams.tiledSupport)      tmp |= PP_TILED_4X4;
    if (tbCfg->ppParams.scalingSupport)
    {
        u32 scalingBits;
        scalingBits = tbCfg->ppParams.scalingSupport & 0x3;
        scalingBits <<= 26;
        tmp |= scalingBits; /* PP_SCALING */
    }
    if (tbCfg->ppParams.deinterlacingSupport)  tmp |= PP_DEINTERLACING;
    if (tbCfg->ppParams.alphaBlendingSupport)  tmp |= PP_ALPHA_BLENDING;
    if (tbCfg->ppParams.ppOutEndianSupport)    tmp |= PP_OUTP_ENDIAN;
    if (tbCfg->ppParams.pixAccOutSupport)      tmp |= PP_PIX_ACC_OUTPUT;
    if (tbCfg->ppParams.ablendCropSupport)     tmp |= PP_ABLEND_CROP;
    if (tbCfg->ppParams.tiledRefSupport)
    {
        u32 tiledBits;
        tiledBits = tbCfg->ppParams.tiledRefSupport & 0x3;
        tiledBits <<= 14;
        tmp |= tiledBits; /* PP_TILED_INPUT */
    }

#endif /* 0 */


    pHwCfg->ppConfig = PP_OUTP_ENDIAN;
    pHwCfg->maxPpOutPicWidth = 4096;
}


/*------------------------------------------------------------------------------
    Function name   : DWLMallocRefFrm
    Description     : Allocate a frame buffer (contiguous linear RAM memory)

    Return type     : i32 - 0 for success or a negative error code

    Argument        : const void * instance - DWL instance
    Argument        : u32 size - size in bytes of the requested memory
    Argument        : void *info - place where the allocated memory buffer
                        parameters are returned
------------------------------------------------------------------------------*/
i32 DWLMallocRefFrm(const void *instance, u32 size, DWLLinearMem_t *info)
{

#ifdef MEMORY_USAGE_TRACE
    DWL_INST_LOG_I(instance, "DWLMallocRefFrm\t%8d bytes\n", size);
#endif

    return DWLMallocLinear(instance, size, info);

}

/*------------------------------------------------------------------------------
    Function name   : DWLFreeRefFrm
    Description     : Release a frame buffer previously allocated with
                        DWLMallocRefFrm.

    Return type     : void

    Argument        : const void * instance - DWL instance
    Argument        : void *info - frame buffer memory information
------------------------------------------------------------------------------*/
void DWLFreeRefFrm(const void *instance, DWLLinearMem_t *info)
{
    DWLFreeLinear(instance, info);
}

/*------------------------------------------------------------------------------
    Function name   : DWLMallocLinear
    Description     : Allocate a contiguous, linear RAM  memory buffer

    Return type     : i32 - 0 for success or a negative error code

    Argument        : const void * instance - DWL instance
    Argument        : u32 size - size in bytes of the requested memory
    Argument        : void *info - place where the allocated memory buffer
                        parameters are returned
------------------------------------------------------------------------------*/
i32 DWLMallocLinear(const void *instance, u32 size, DWLLinearMem_t *info)
{
    DWLInstance_t *dwlInst = (DWLInstance_t *) instance;

#ifdef SOLUTION
    info->virtualAddress = app_sram_calloc(size, 1);
#else
    info->virtualAddress = rt_calloc(size, 1);
#endif
    DWL_INST_LOG_I(instance, "DWLMallocLinear: %p, size=%8d\n", info->virtualAddress, size);
    if (info->virtualAddress == NULL)
        return DWL_ERROR;
    info->busAddress = (g1_addr_t) info->virtualAddress;
    info->size = size;
    dwlInst->linearTotal += size;
    dwlInst->linearAllocCount++;
    DWL_INST_LOG_I(instance, "DWLMallocLinear: allocated total %8d bytes in %2d buffers\n",
                   dwlInst->linearTotal, dwlInst->linearAllocCount);


    return DWL_OK;
}

/*------------------------------------------------------------------------------
    Function name   : DWLFreeLinear
    Description     : Release a linera memory buffer, previously allocated with
                        DWLMallocLinear.

    Return type     : void

    Argument        : const void * instance - DWL instance
    Argument        : void *info - linear buffer memory information
------------------------------------------------------------------------------*/
void DWLFreeLinear(const void *instance, DWLLinearMem_t *info)
{
    DWLInstance_t *dwlInst = (DWLInstance_t *) instance;
    RT_ASSERT(dwlInst != NULL);

    DWL_INST_LOG_I(instance, "DWLFreeLinear: %8d\n", info->size);
    dwlInst->linearTotal -= info->size;
    dwlInst->linearAllocCount--;
    DWL_INST_LOG_I(instance, "DWLFreeLinear: not freed %8d bytes in %2d buffers\n",
                   dwlInst->linearTotal, dwlInst->linearAllocCount);

#ifdef SOLUTION
    app_sram_free(info->virtualAddress);
#else
    rt_free(info->virtualAddress);
#endif

    info->size = 0;
}

/*------------------------------------------------------------------------------
    Function name   : DWLWriteReg
    Description     : Write a value to a hardware IO register

    Return type     : void

    Argument        : const void * instance - DWL instance
    Argument        : u32 offset - byte offset of the register to be written
    Argument        : u32 value - value to be written out
------------------------------------------------------------------------------*/

void DWLWriteReg(const void *instance, i32 coreID, u32 offset, u32 value)
{
    DWLInstance_t *dwlInst = (DWLInstance_t *) instance;
    u32 reg_idx = offset >> 2;

#ifndef DWL_DISABLE_REG_PRINTS
    DWL_DEBUG("Write core[%d] swreg[%d] = %08X\n",
              coreID, reg_idx, value);
#endif

    dwlInst->RegBase[reg_idx] = value;
}

/*------------------------------------------------------------------------------
    Function name   : DWLReadReg
    Description     : Read the value of a hardware IO register

    Return type     : u32 - the value stored in the register

    Argument        : const void * instance - DWL instance
    Argument        : u32 offset - byte offset of the register to be read
------------------------------------------------------------------------------*/
u32 DWLReadReg(const void *instance, i32 coreID, u32 offset)
{
    DWLInstance_t *dwlInst = (DWLInstance_t *) instance;
    u32 reg_idx = offset >> 2;

    u32 val;

    RT_ASSERT(dwlInst != NULL);

    val = dwlInst->RegBase[reg_idx];


#ifndef DWL_DISABLE_REG_PRINTS
    DWL_DEBUG("Read core[%d] swreg[%d] ret %08X\n",
              coreID, reg_idx, val);
#endif




    return val;
}

/*------------------------------------------------------------------------------
    Function name   : DWLEnableHW
    Description     : Enable hw by writing to register
    Return type     : void
    Argument        : const void * instance - DWL instance
    Argument        : u32 offset - byte offset of the register to be written
    Argument        : u32 value - value to be written out
------------------------------------------------------------------------------*/
void DWLEnableHW(const void *instance, i32 coreID, u32 offset, u32 value)
{
    DWL_INST_LOG_I(instance, "DWLEnableHW\n");

    HAL_NVIC_SetPriority(JPEG_DEC_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(JPEG_DEC_IRQn);
    HAL_RCC_EnableModule(RCC_MOD_JDEC);

    DWLWriteReg(instance, coreID, offset, value);
}

/*------------------------------------------------------------------------------
    Function name   : DWLDisableHW
    Description     : Disable hw by writing to register
    Return type     : void
    Argument        : const void * instance - DWL instance
    Argument        : u32 offset - byte offset of the register to be written
    Argument        : u32 value - value to be written out
------------------------------------------------------------------------------*/
void DWLDisableHW(const void *instance, i32 coreID, u32 offset, u32 value)
{
    HAL_NVIC_DisableIRQ(JPEG_DEC_IRQn);
    HAL_RCC_ResetModule(RCC_MOD_JDEC);
    HAL_RCC_DisableModule(RCC_MOD_JDEC);

    DWLWriteReg(instance, coreID, offset, value);
    DWL_INST_LOG_I(instance, "DWLDisableHW\n");
}

/*------------------------------------------------------------------------------
    Function name   : DWLWaitHwReady
    Description     : Wait until hardware has stopped running.
                      Used for synchronizing software runs with the hardware.
                      The wait could succed, timeout, or fail with an error.

    Return type     : i32 - one of the values DWL_HW_WAIT_OK
                                              DWL_HW_WAIT_TIMEOUT
                                              DWL_HW_WAIT_ERROR

    Argument        : const void * instance - DWL instance
------------------------------------------------------------------------------*/
i32 DWLWaitHwReady(const void *instance, i32 coreID, u32 timeout)
{
    i32 ret = DWL_HW_WAIT_OK;
    DWLInstance_t *dwlInst = (DWLInstance_t *) instance;
    rt_err_t err;
    DWL_INST_LOG_I(instance, "DWLWaitHwReady");

    err = rt_sem_take(&dwlInst->sem, rt_tick_from_millisecond(1000));

    if (err == -RT_ERROR)         ret = DWL_HW_WAIT_ERROR;
    else if (err == -RT_ETIMEOUT) ret = DWL_HW_WAIT_TIMEOUT;
    else                        ret = DWL_HW_WAIT_OK;

    return ret;
}

/*------------------------------------------------------------------------------
    Function name   : DWLmalloc
    Description     : Allocate a memory block.

    Return type     : void pointer to the allocated space, or NULL if there
                      is insufficient memory available

    Argument        : u32 n - Bytes to allocate
------------------------------------------------------------------------------*/
void *DWLmalloc(u32 n)
{
    void *p;

    p = rt_malloc((size_t) n);

#ifdef MEMORY_USAGE_TRACE
    DWL_LOG_I("DWLmalloc\t%x,  %8d bytes\n", p, n);
#endif


    return p;
}

/*------------------------------------------------------------------------------
    Function name   : DWLfree
    Description     : Deallocates or frees a memory block. Same functionality as
                      the ANSI C rt_free()

    Return type     : void

    Argument        : void *p - Previously allocated memory block to be freed
------------------------------------------------------------------------------*/
void DWLfree(void *p)
{
    if (p != NULL)
        rt_free(p);
}

/*------------------------------------------------------------------------------
    Function name   : DWLcalloc
    Description     : Allocates an array in memory with elements initialized
                      to 0.

    Return type     : void pointer to the allocated space, or NULL if there
                      is insufficient memory available

}
    Argument        : u32 n - Number of elements
    Argument        : u32 s - Length in bytes of each element.
------------------------------------------------------------------------------*/
void *DWLcalloc(u32 n, u32 s)
{
#ifdef MEMORY_USAGE_TRACE
    DWL_LOG_I("DWLcalloc\t%8d bytes\n", n * s);
#endif

    void *p = NULL;



    p = rt_calloc(n, s);


    return p;
}

/*------------------------------------------------------------------------------
    Function name   : DWLmemcpy
    Description     : Copies characters between buffers. Same functionality as
                      the ANSI C memcpy()

    Return type     : The value of destination d

    Argument        : void *d - Destination buffer
    Argument        : const void *s - Buffer to copy from
    Argument        : u32 n - Number of bytes to copy
------------------------------------------------------------------------------*/
void *DWLmemcpy(void *d, const void *s, u32 n)
{
    return rt_memcpy(d, s, (size_t) n);
}

/*------------------------------------------------------------------------------
    Function name   : DWLmemset
    Description     : Sets buffers to a specified character. Same functionality
                      as the ANSI C memset()

    Return type     : The value of destination d

    Argument        : void *d - Pointer to destination
    Argument        : i32 c - Character to set
    Argument        : u32 n - Number of characters
------------------------------------------------------------------------------*/
void *DWLmemset(void *d, i32 c, u32 n)
{
    return rt_memset(d, (int) c, (size_t) n);
}


/*------------------------------------------------------------------------------
    Function name   : DWLInit
    Description     : Initialize a DWL instance

    Return type     : const void * - pointer to a DWL instance

    Argument        : void * param - not in use, application passes NULL
------------------------------------------------------------------------------*/
const void *DWLInit(DWLInitParam_t *param)
{
    rt_err_t err;
    DWLInstance_t *dec_dwl;

    RT_ASSERT(param);
    DWL_DEBUG("DWLInit");

    dec_dwl = (DWLInstance_t *) rt_calloc(1, sizeof(DWLInstance_t));

    if (dec_dwl == NULL)
    {
        DWL_DEBUG("failed to alloc DWLInstance_t struct");
        return NULL;
    }

    HAL_RCC_EnableModule(RCC_MOD_JDEC);
    HAL_RCC_ResetModule(RCC_MOD_JDEC);

    switch (param->clientType)
    {
    case DWL_CLIENT_TYPE_JPEG_DEC:
    {
        DWL_DEBUG("DWL initialized by a JPEG decoder instance...\n");

        dec_dwl->RegBase = (u32 *)JPEG_DEC_BASE;
        p_JPGDec_DWLInst = dec_dwl;
        err = rt_sem_init(&dec_dwl->sem, "JpgDec", 0, RT_IPC_FLAG_FIFO);
        RT_ASSERT(err == RT_EOK);

    }
    break;

    case DWL_CLIENT_TYPE_PP:
    {
        DWL_DEBUG("DWL initialized by a PP instance...\n");

        dec_dwl->RegBase = (u32 *)JPEG_DEC_BASE;
        err = rt_sem_init(&dec_dwl->sem, "PP", 0, RT_IPC_FLAG_FIFO);
        RT_ASSERT(err == RT_EOK);
    }
    break;

    default:
        DWL_DEBUG("ERROR: DWL client type has to be always specified!\n");
        rt_free(dec_dwl);
        return NULL;
    }

    dec_dwl->type    = param->clientType;

    dec_dwl->linearAllocCount = 0;
    dec_dwl->linearTotal = 0;

    dec_dwl->ReserveCnt = 0;

    DWL_DEBUG("====DWLInit %p====\r\n", dec_dwl);
    return dec_dwl;
}

/*------------------------------------------------------------------------------
    Function name   : DWLRelease
    Description     : Release a DWl instance

    Return type     : i32 - 0 for success or a negative error code

    Argument        : const void * instance - instance to be released
------------------------------------------------------------------------------*/
i32 DWLRelease(const void *instance)
{
    DWLInstance_t *dec_dwl = (DWLInstance_t *) instance;

    if(p_JPGDec_DWLInst == dec_dwl) p_JPGDec_DWLInst = NULL;
    rt_err_t err;
    err = rt_sem_detach(&dec_dwl->sem);
    RT_ASSERT(err == RT_EOK);
    
    DWL_DEBUG("DWLRelease ReserveCnt =%d\n", dec_dwl->ReserveCnt);

    rt_free((void *)instance);

    DWL_DEBUG("DWLRelease SUCCESS\n");

    return (DWL_OK);
}

/* HW locking */


/*------------------------------------------------------------------------------
    Function name   : DWLReserveHw
    Description     :
    Return type     : i32
    Argument        : const void *instance
    Argument        : i32 *coreID - ID of the reserved HW core
------------------------------------------------------------------------------*/
i32 DWLReserveHw(const void *instance, i32 *coreID)
{
    DWLInstance_t *dec_dwl = (DWLInstance_t *) instance;

    dec_dwl->ReserveCnt++;

    DWL_DEBUG("====DWLReserveHw %d %p(%d)====\r\n", *coreID, dec_dwl, dec_dwl->ReserveCnt);



    return DWL_OK;
}

/*------------------------------------------------------------------------------
    Function name   : DWLReleaseHw
    Description     :
    Return type     : void
    Argument        : const void *instance
------------------------------------------------------------------------------*/
void DWLReleaseHw(const void *instance, i32 coreID)
{
    DWLInstance_t *dec_dwl = (DWLInstance_t *) instance;

    dec_dwl->ReserveCnt--;

    DWL_DEBUG("====DWLReleaseHw %d %p(%d)====\r\n", coreID, dec_dwl, dec_dwl->ReserveCnt);

}

void DWLSetIRQCallback(const void *instance, i32 coreID,
                       DWLIRQCallbackFn *pCallbackFn, void *arg)
{
    /* not in use with single core only control code */
    UNUSED(instance);
    UNUSED(coreID);
    UNUSED(pCallbackFn);
    UNUSED(arg);

    RT_ASSERT(0);
}




void JpegDecTrace(const char *string)
{
    DWL_LOG_I(string);
    //rt_kprintf("\n");
}



void PPTrace(const char *string)
{
    DWL_LOG_I(string);
}


