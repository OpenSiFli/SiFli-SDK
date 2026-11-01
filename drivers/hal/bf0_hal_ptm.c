/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <string.h>

#include "bf0_hal.h"

#if defined(HAL_PTM_MODULE_ENABLED) || defined(_SIFLI_DOXYGEN_)

/** @addtogroup BF0_HAL_Driver
 * @{
 */

/** @defgroup PTM PTM (Parallel Task Machine)
 * @{
 */

/** @defgroup PTM_Private_Constants PTM private constants
 * @{
 */

/**
 * Distance between the register slice of two cores.
 *
 * PTM_CoreRegTypeDef (ptm.h) mirrors one core block of PTM_TypeDef, so its size
 * is that distance.  The checks below fail the build if the two definitions ever
 * diverge, field by field and not only in size.
 */
#define PTM_CORE_REG_STRIDE     ((uint32_t)sizeof(PTM_CoreRegTypeDef))

#define PTM_CORE_VIEW_MUST_MATCH(field) \
    typedef char PTM_view_##field##_must_match_the_map[ \
        (offsetof(PTM_CoreRegTypeDef, field) \
         == ((size_t)offsetof(PTM_TypeDef, field##0) - (size_t)offsetof(PTM_TypeDef, CCR0))) ? 1 : -1]

PTM_CORE_VIEW_MUST_MATCH(CCR);
PTM_CORE_VIEW_MUST_MATCH(ICR);
PTM_CORE_VIEW_MUST_MATCH(OCR);
PTM_CORE_VIEW_MUST_MATCH(X);
PTM_CORE_VIEW_MUST_MATCH(Y);
PTM_CORE_VIEW_MUST_MATCH(A);
PTM_CORE_VIEW_MUST_MATCH(B);
PTM_CORE_VIEW_MUST_MATCH(PC);
PTM_CORE_VIEW_MUST_MATCH(IFIFO);
PTM_CORE_VIEW_MUST_MATCH(OFIFO);

typedef char PTM_core_view_size_must_match_the_map[
     (sizeof(PTM_CoreRegTypeDef) == ((size_t)offsetof(PTM_TypeDef, X1) - (size_t)offsetof(PTM_TypeDef, X0))) ? 1 : -1];

#define PTM_CER_EN_BIT(core)    (PTM_CER_EN0_Msk   << (core))
#define PTM_CER_STOP_BIT(core)  (PTM_CER_STOP0_Msk << (core))
#define PTM_CER_RST_BIT(core)   (PTM_CER_RST0_Msk  << (core))
#define PTM_CER_STEP_BIT(core)  (PTM_CER_STEP0_Msk << (core))
#define PTM_ISR_EVT_BIT(core)   (PTM_ISR_EVT0_Msk  << (core))
#define PTM_ISR_ERR_BIT(core)   (PTM_ISR_ERR0_Msk  << (core))
#define PTM_ISR_TCM_BIT(core)   (PTM_ISR_TCM0_Msk  << (core))
#define PTM_IER_EVTIE_BIT(core) (PTM_IER_EVTIE0_Msk   << (core))
#define PTM_IER_ERRIE_BIT(core) (PTM_IER_ERRIE0_Msk   << (core))
#define PTM_IER_EVTAUTO_BIT(core) (PTM_IER_EVTAUTO0_Msk << (core))
#define PTM_IER_CORE_Msk(core)  (PTM_IER_EVTIE_BIT(core) | PTM_IER_EVTAUTO_BIT(core) | PTM_IER_ERRIE_BIT(core))

/** Bitmaps of the implemented cores, derived from the register map. */
#define PTM_CORE_Msk            ((PTM_CER_EN0_Msk << PTM_CORE_NUM) - PTM_CER_EN0_Msk)
#define PTM_ISR_EVT_Msk         ((PTM_ISR_EVT15_Msk << 1U) - PTM_ISR_EVT0_Msk)
#define PTM_ISR_TCM_Msk         ((PTM_ISR_TCM0_Msk << PTM_CORE_NUM) - PTM_ISR_TCM0_Msk)

/** Base of the shared TCM, from the instruction encoding header. */
#define PTM_TCM_BASE            PTM1_CORE0_TCM
/**
 * @}
 */

/** @defgroup PTM_Private_Variables PTM private variables
 * @{
 */

/** Core index to handle mapping, used by HAL_PTM_IRQHandler(). */
static PTM_CoreHandleTypeDef *s_ptm_core_handle[PTM_CORE_NUM];

/**
 * Number of TCM words loaded by each core, 0 when it loaded nothing.
 *
 * The TCM is shared by the cores (core N address 0 is TCM[16 * N]) and a program
 * longer than 16 instructions overruns into the following cores, so the HAL has
 * to know how far the program of every core reaches.  The length is enough: the
 * program of a core always starts at its own window.
 */
static uint8_t s_ptm_tcm_len[PTM_CORE_NUM];

/**
 * @}
 */

/** @defgroup PTM_Private_Functions PTM private functions
 * @{
 */

/**
 * @brief  Enter a short critical section.
 *
 * Same idiom as HAL_RCC_DisableInterrupt()/HAL_RCC_EnableInterrupt(), which are
 * __STATIC_INLINE as well: the previous interrupt state is returned so that a
 * caller running with interrupts already masked can restore it unchanged.
 * @retval Previous PRIMASK value
 */
__STATIC_INLINE uint32_t PTM_DisableInterrupt(void)
{
    uint32_t mask;

    mask = __get_PRIMASK();
    __set_PRIMASK(1U);

    return mask;
}

/**
 * @brief  Leave a critical section entered with PTM_DisableInterrupt().
 * @param  mask Value returned by PTM_DisableInterrupt()
 */
__STATIC_INLINE void PTM_EnableInterrupt(uint32_t mask)
{
    __set_PRIMASK(mask);
}

/**
 * @brief  Get the register slice of one core.
 * @param  hptm Module handle
 * @param  core Core index
 * @retval Registers of that core
 */
__STATIC_INLINE PTM_CoreRegTypeDef *PTM_GetCoreReg(PTM_HandleTypeDef *hptm, uint32_t core)
{
    return (PTM_CoreRegTypeDef *)((uint32_t) & (hptm->Instance->CCR0) + (core * PTM_CORE_REG_STRIDE));
}

/**
 * @brief  Pack an IO/FIFO configuration into the ICRx register value.
 * @param  cfg Configuration to pack
 * @retval ICRx value
 */
static uint32_t PTM_PackICR(const PTM_IoFifoInitTypeDef *cfg)
{
    uint32_t value = 0U;

    value |= MAKE_REG_VAL2(cfg->ififo_en, PTM_ICR0_IFIFO);
    value |= MAKE_REG_VAL2(cfg->iauto, PTM_ICR0_IAUTO);
    value |= MAKE_REG_VAL2(cfg->irsh, PTM_ICR0_IRSH);
    value |= MAKE_REG_VAL2(cfg->iblock, PTM_ICR0_IBLOCK);
    value |= MAKE_REG_VAL2(cfg->ibase, PTM_ICR0_IBASE);
    value |= MAKE_REG_VAL2(cfg->ishift, PTM_ICR0_ISHIFT);
    value |= MAKE_REG_VAL2(cfg->ilimit, PTM_ICR0_ILIMIT);

    return value;
}

/**
 * @brief  Pack an IO/FIFO configuration into the OCRx register value.
 * @param  cfg Configuration to pack
 * @retval OCRx value
 */
static uint32_t PTM_PackOCR(const PTM_IoFifoInitTypeDef *cfg)
{
    uint32_t value = 0U;

    value |= MAKE_REG_VAL2(cfg->ofifo_en, PTM_OCR0_OFIFO);
    value |= MAKE_REG_VAL2(cfg->oauto, PTM_OCR0_OAUTO);
    value |= MAKE_REG_VAL2(cfg->orsh, PTM_OCR0_ORSH);
    value |= MAKE_REG_VAL2(cfg->oblock, PTM_OCR0_OBLOCK);
    value |= MAKE_REG_VAL2(cfg->obase, PTM_OCR0_OBASE);
    value |= MAKE_REG_VAL2(cfg->oshift, PTM_OCR0_OSHIFT);
    value |= MAKE_REG_VAL2(cfg->olimit, PTM_OCR0_OLIMIT);

    return value;
}

/**
 * @brief  Tell whether a handle owns a core of this module.
 * @param  hcore Core handle
 * @retval 1 when the handle may be used with the HAL accessors
 */
__STATIC_INLINE int PTM_IsCoreHandleValid(PTM_CoreHandleTypeDef *hcore)
{
    return ((hcore != NULL) && (hcore->hptm != NULL) && (hcore->core < PTM_CORE_NUM));
}

/**
 * @brief  Tell whether a TCM slot belongs to another core.
 * @param  hptm Module handle, which holds the core allocation bitmap
 * @param  core Core asking for the slot
 * @param  idx  Global TCM slot index (0..PTM_TCM_WORDS-1)
 * @retval 1 when another core owns that slot
 */
static int PTM_TcmOwnedByOther(PTM_HandleTypeDef *hptm, uint32_t core, uint32_t idx)
{
    uint32_t other;
    uint32_t offset;
    uint32_t owned;

    for (other = 0U; other < PTM_CORE_NUM; other++)
    {
        /* Only a claimed core owns TCM words. */
        if ((other == core) || ((hptm->core_bitmap & (1U << other)) == 0U))
        {
            continue;
        }

        /* A claimed core owns the words of its program and, until it has one,
           its whole window: a program that would overrun the window of an
           initialized but still empty core must be refused here, because that
           core has no program yet to collide with (it would only notice the
           overrun when it loads its own code later). */
        owned = (uint32_t)s_ptm_tcm_len[other];
        if (owned < PTM_TCM_WORDS_PER_CORE)
        {
            owned = PTM_TCM_WORDS_PER_CORE;
        }

        /* Offset of idx inside the window of that core, with the wraparound of
           the TCM window taken into account. */
        offset = (idx + PTM_TCM_WORDS - (PTM_TCM_WORDS_PER_CORE * other)) % PTM_TCM_WORDS;
        if (offset < owned)
        {
            return 1;
        }
    }

    return 0;
}

/** Compute a tick deadline, HAL_MAX_DELAY meaning "no deadline". */
static uint32_t PTM_Deadline(uint32_t timeout_ms)
{
    return (timeout_ms == HAL_MAX_DELAY) ? HAL_MAX_DELAY : (HAL_GetTick() + timeout_ms);
}

/** Wrap safe timeout test of a deadline built by PTM_Deadline(). */
__STATIC_INLINE int PTM_IsTimeout(uint32_t deadline)
{
    if (deadline == HAL_MAX_DELAY)
    {
        return 0;
    }

    return (((int32_t)(HAL_GetTick() - deadline)) >= 0) ? 1 : 0;
}

/**
 * @brief  Consume the event flag of one core.
 *
 * Used when a core is started: a completion event left over by a previous run
 * must not make CoreWaitDone() return immediately, nor raise an interrupt as
 * soon as the NVIC line is enabled.
 * @param  hptm Module handle
 * @param  core Core index
 */
__STATIC_INLINE void PTM_ClearCoreEvent(PTM_HandleTypeDef *hptm, uint32_t core)
{
    hptm->Instance->ISR = PTM_ISR_EVT_BIT(core);        /* write 1 clear */
}

/**
 * @brief  Program the notification bits of one core and its NVIC line.
 * @param  hptm Module handle
 * @param  core Core index
 * @param  bits IER bits of that core to enable, 0 to report nothing
 */
static void PTM_WriteCoreNotify(PTM_HandleTypeDef *hptm, uint32_t core, uint32_t bits)
{
    uint32_t mask;

    mask = PTM_DisableInterrupt();

    /* IER is shared by all the cores: only the three bits of this core change. */
    hptm->Instance->IER = (hptm->Instance->IER & ~PTM_IER_CORE_Msk(core)) | (bits & PTM_IER_CORE_Msk(core));

    if ((bits & (PTM_IER_EVTIE_BIT(core) | PTM_IER_ERRIE_BIT(core))) != 0U)
    {
        HAL_NVIC_EnableIRQ(PTM_CORE_IRQn(core));
    }
    else
    {
        HAL_NVIC_DisableIRQ(PTM_CORE_IRQn(core));
    }

    PTM_EnableInterrupt(mask);
}

/**
 * @brief  Program the notification mode of one core.
 * @param  hptm           Module handle
 * @param  core           Core index
 * @param  interrupt_mode 0: polling (completion event only), 1: interrupt mode
 */
static void PTM_SetCoreNotify(PTM_HandleTypeDef *hptm, uint32_t core, uint8_t interrupt_mode)
{
    uint32_t bits = PTM_IER_EVTAUTO_BIT(core);

    if (interrupt_mode != 0U)
    {
        bits |= PTM_IER_EVTIE_BIT(core) | PTM_IER_ERRIE_BIT(core);
    }

    PTM_WriteCoreNotify(hptm, core, bits);
}

/**
 * @brief  Enable a set of cores, each one either in interrupt or in polling mode.
 *
 * Every core of the set is prepared first (its notification mode and a cleared
 * event flag), then all of them are enabled by a single register write, i.e. in
 * the same hclk cycle.  The mode is per core: a helper core that nobody waits
 * for can be started together with the master without adding interrupts.
 * @param  hptm      Module handle
 * @param  core_mask Cores to enable, bit n = core n
 * @param  it_mask   Cores of @c core_mask working in interrupt mode, the others
 *                   only get the completion event (polling mode)
 */
static void PTM_StartCores(PTM_HandleTypeDef *hptm, uint32_t core_mask, uint32_t it_mask)
{
    uint32_t core;

    core_mask &= PTM_CORE_Msk;
    it_mask &= core_mask;

    for (core = 0U; core < PTM_CORE_NUM; core++)
    {
        if ((core_mask & (1UL << core)) != 0U)
        {
            /* Forget the event left over by an earlier run *before* the NVIC
               line is enabled: the interrupt is level triggered, so a stale
               EVTx would be dispatched as a spurious callback as soon as the
               line is enabled, before this clear had a chance to run. */
            PTM_ClearCoreEvent(hptm, core);
            (void)PTM_SetCoreNotify(hptm, core, ((it_mask & (1UL << core)) != 0U) ? 1U : 0U);
        }
    }

    hptm->Instance->CER = core_mask;
}

/**
 * @}
 */

/** @defgroup PTM_Exported_Functions_Module Module level
 * @{
 */

PTM_HandleTypeDef *HAL_PTM_GetHandle(void)
{
    static PTM_HandleTypeDef s_ptm;

    if (s_ptm.Instance == NULL)
    {
        s_ptm.Instance = hwp_ptm1;
    }

    return &s_ptm;
}

HAL_StatusTypeDef HAL_PTM_Init(PTM_HandleTypeDef *hptm)
{
    uint32_t mask;

    if (hptm == NULL)
    {
        return HAL_ERROR;
    }

    /* The reference count and the provisioning are shared by every user of the
       module: without the critical section two threads could both see "first
       user" and reset the module while the other one already runs a core. */
    mask = PTM_DisableInterrupt();

    if (hptm->init_cnt == 0U)
    {
        /* First user: provision the hardware.  Later users only bump the
           reference count, so a core that is already running is never reset. */
        HAL_RCC_EnableModule(RCC_MOD_PTM1);
        HAL_RCC_ResetModule(RCC_MOD_PTM1);

        hptm->Instance->IER = 0U;
        hptm->Instance->ISR = PTM_ISR_EVT_Msk;  /* write 1 clear, clears EVT0~EVT15 */
        hptm->State = HAL_PTM_STATE_READY;
    }

    /* The counter saturates instead of wrapping; reaching 0xFF would take as
       many HAL_PTM_DeInit() calls, which only an unbalanced user hits. */
    if (hptm->init_cnt < 0xFFU)
    {
        hptm->init_cnt++;
    }

    PTM_EnableInterrupt(mask);

    return HAL_OK;
}

HAL_StatusTypeDef HAL_PTM_DeInit(PTM_HandleTypeDef *hptm)
{
    uint32_t core;
    uint32_t mask;

    if ((hptm == NULL) || (hptm->init_cnt == 0U))
    {
        return HAL_ERROR;
    }

    mask = PTM_DisableInterrupt();

    hptm->init_cnt--;

    if (hptm->init_cnt == 0U)
    {
        hptm->Instance->IER = 0U;
        for (core = 0U; core < PTM_CORE_NUM; core++)
        {
            hptm->Instance->CER = PTM_CER_RST_BIT(core);
            s_ptm_core_handle[core] = NULL;
            s_ptm_tcm_len[core] = 0U;
            PTM_WriteCoreNotify(hptm, core, 0U);
        }

        /* The module is reset, so nothing stays claimed either: a core that was
           claimed through a handle whose owner is gone must not block a later
           HAL_PTM_CoreInit() with HAL_BUSY. */
        hptm->core_bitmap = 0U;
        hptm->State = HAL_PTM_STATE_RESET;
        HAL_RCC_DisableModule(RCC_MOD_PTM1);
    }

    PTM_EnableInterrupt(mask);

    return HAL_OK;
}

HAL_StatusTypeDef HAL_PTM_ForceReset(PTM_HandleTypeDef *hptm)
{
    uint32_t core;
    uint32_t mask;

    if ((hptm == NULL) || (hptm->init_cnt == 0U))
    {
        return HAL_ERROR;
    }

    mask = PTM_DisableInterrupt();

    /* Stops every core and clears all of their state: only for teardown, or
       when the caller knows no other user of the module is active. */
    HAL_RCC_ResetModule(RCC_MOD_PTM1);

    for (core = 0U; core < PTM_CORE_NUM; core++)
    {
        s_ptm_core_handle[core] = NULL;
        s_ptm_tcm_len[core] = 0U;
    }

    /* Nothing stays claimed either: after a full module reset a stale claim
       would make every HAL_PTM_CoreInit() fail with HAL_BUSY.  The handles that
       were in use must be released with HAL_PTM_CoreDeInit(), or re-initialized
       afterwards, which keeps the reference count balanced. */
    hptm->core_bitmap = 0U;

    hptm->Instance->IER = 0U;
    hptm->Instance->ISR = PTM_ISR_EVT_Msk;

    PTM_EnableInterrupt(mask);

    return HAL_OK;
}

int32_t HAL_PTM_CoreAlloc(PTM_HandleTypeDef *hptm, uint32_t core)
{
    uint32_t mask;
    uint32_t i;
    int32_t result = -1;

    if (hptm == NULL)
    {
        return -1;
    }

    /* Test and set of the bitmap is a read-modify-write shared by all the
       users of the module, so it needs to be atomic. */
    mask = PTM_DisableInterrupt();

    if (core == PTM_CORE_ANY)
    {
        for (i = 0U; i < PTM_CORE_NUM; i++)
        {
            if ((hptm->core_bitmap & (1U << i)) == 0U)
            {
                hptm->core_bitmap |= (1U << i);
                result = (int32_t)i;
                break;
            }
        }
    }
    else if ((core < PTM_CORE_NUM) && ((hptm->core_bitmap & (1U << core)) == 0U))
    {
        hptm->core_bitmap |= (1U << core);
        result = (int32_t)core;
    }

    PTM_EnableInterrupt(mask);

    return result;
}

HAL_StatusTypeDef HAL_PTM_CoreFree(PTM_HandleTypeDef *hptm, uint32_t core)
{
    uint32_t mask;

    if ((hptm == NULL) || (core >= PTM_CORE_NUM))
    {
        return HAL_ERROR;
    }

    mask = PTM_DisableInterrupt();

    hptm->core_bitmap &= ~(1U << core);
    s_ptm_tcm_len[core] = 0U;       /* the TCM range of that core is free again */
    s_ptm_core_handle[core] = NULL; /* no dispatch to a handle that lost it */

    PTM_EnableInterrupt(mask);

    /* And stop reporting from a core that is not owned anymore. */
    PTM_WriteCoreNotify(hptm, core, 0U);

    return HAL_OK;
}

void HAL_PTM_IRQHandler(uint32_t core)
{
    PTM_HandleTypeDef *hptm = HAL_PTM_GetHandle();
    PTM_CoreHandleTypeDef *hcore;
    uint32_t isr, ier, err = 0U;
    uint16_t evt = 0U;

    if (core >= PTM_CORE_NUM)
    {
        return;
    }

    hcore = s_ptm_core_handle[core];
    isr = hptm->Instance->ISR;
    ier = hptm->Instance->IER;

    if ((isr & PTM_ISR_ERR_BIT(core)) != 0U)
    {
        err |= PTM_ERR_CORE;
    }
    if ((isr & PTM_ISR_TCM_BIT(core)) != 0U)
    {
        err |= PTM_ERR_TCM_CONFLICT;
    }

    /* EVT0~EVT15 are global flags shared by the cores: clear only the bit of
       this core, and only when its interrupt is enabled, otherwise another
       core may be waiting on (WAIT) or polling that flag. */
    if (((isr & PTM_ISR_EVT_BIT(core)) != 0U) && ((ier & PTM_IER_EVTIE_BIT(core)) != 0U))
    {
        hptm->Instance->ISR = PTM_ISR_EVT_BIT(core);    /* write 1 clear */
        evt = (uint16_t)PTM_ISR_EVT_BIT(core);
    }

    if (err != 0U)
    {
        /* ERRx/TCMx cannot be cleared by a write, only CER.RSTx clears them, so
           the error interrupt is level triggered: mask it before reporting,
           otherwise this handler is entered again as soon as it returns (and
           forever, when the error callback is the empty default one).
           HAL_PTM_CoreReset() clears the flags, CoreStart*() re-arms. */
        PTM_WriteCoreNotify(hptm, core, ier & PTM_IER_CORE_Msk(core) & ~PTM_IER_ERRIE_BIT(core));
    }

    if (hcore == NULL)
    {
        /* Interrupt after the core was released: flags are handled, no callback. */
        return;
    }

    if (evt != 0U)
    {
        if (hcore->EventCallback != NULL)
        {
            hcore->EventCallback(hcore, evt);
        }
        else
        {
            HAL_PTM_CoreEventCallback(hcore, evt);
        }
    }

    if (err != 0U)
    {
        if (hcore->ErrorCallback != NULL)
        {
            hcore->ErrorCallback(hcore, err);
        }
        else
        {
            HAL_PTM_ErrorCallback(hcore, err);
        }
    }
}

uint16_t HAL_PTM_GetEvent(PTM_HandleTypeDef *hptm)
{
    if (hptm == NULL)
    {
        return 0U;
    }

    return (uint16_t)(hptm->Instance->ISR & PTM_ISR_EVT_Msk);
}

HAL_StatusTypeDef HAL_PTM_ClearEvent(PTM_HandleTypeDef *hptm, uint16_t evt_mask)
{
    if (hptm == NULL)
    {
        return HAL_ERROR;
    }

    hptm->Instance->ISR = (uint32_t)evt_mask & PTM_ISR_EVT_Msk;    /* write 1 clear */

    return HAL_OK;
}

HAL_StatusTypeDef HAL_PTM_SetOutputEnable(PTM_HandleTypeDef *hptm, uint16_t ch_mask, uint16_t en_mask)
{
    if (hptm == NULL)
    {
        return HAL_ERROR;
    }

    hptm->Instance->DOESR = MAKE_REG_VAL2(ch_mask & en_mask, PTM_DOESR_DOES);
    hptm->Instance->DOECR = MAKE_REG_VAL2(ch_mask & (uint16_t)(~en_mask), PTM_DOECR_DOEC);

    return HAL_OK;
}

uint16_t HAL_PTM_GetOutputEnable(PTM_HandleTypeDef *hptm)
{
    if (hptm == NULL)
    {
        return 0U;
    }

    return (uint16_t)(hptm->Instance->DOER & PTM_DOER_DOE_Msk);
}

HAL_StatusTypeDef HAL_PTM_SetOutput(PTM_HandleTypeDef *hptm, uint16_t ch_mask, uint16_t high_mask)
{
    if (hptm == NULL)
    {
        return HAL_ERROR;
    }

    hptm->Instance->DOSR = MAKE_REG_VAL2(ch_mask & high_mask, PTM_DOSR_DOS);
    hptm->Instance->DOCR = MAKE_REG_VAL2(ch_mask & (uint16_t)(~high_mask), PTM_DOCR_DOC);

    return HAL_OK;
}

uint16_t HAL_PTM_GetOutput(PTM_HandleTypeDef *hptm)
{
    if (hptm == NULL)
    {
        return 0U;
    }

    return (uint16_t)(hptm->Instance->DOR & PTM_DOR_DO_Msk);
}

uint32_t HAL_PTM_GetTcmConflict(PTM_HandleTypeDef *hptm)
{
    if (hptm == NULL)
    {
        return 0U;
    }

    return (hptm->Instance->ISR & PTM_ISR_TCM_Msk) >> PTM_ISR_TCM0_Pos;
}

/**
 * @}
 */

/** @defgroup PTM_Exported_Functions_Core Core level
 * @{
 */

HAL_StatusTypeDef HAL_PTM_CoreInit(PTM_CoreHandleTypeDef *hcore)
{
    PTM_HandleTypeDef *hptm;
    int32_t core;

    if (hcore == NULL)
    {
        return HAL_ERROR;
    }

    if (hcore->hptm != NULL)
    {
        /* Already claimed: HAL_PTM_CoreDeInit() has to be used first, otherwise
           the core claimed by the previous call and its reference count leak. */
        return HAL_ERROR;
    }

    if ((hcore->core != PTM_CORE_ANY) && (hcore->core >= PTM_CORE_NUM))
    {
        return HAL_ERROR;
    }

    hptm = HAL_PTM_GetHandle();
    if (HAL_PTM_Init(hptm) != HAL_OK)
    {
        return HAL_ERROR;
    }

    core = HAL_PTM_CoreAlloc(hptm, hcore->core);
    if (core < 0)
    {
        (void)HAL_PTM_DeInit(hptm);
        return HAL_BUSY;
    }

    hcore->hptm = hptm;
    hcore->core = (uint32_t)core;
    hcore->State = HAL_PTM_STATE_BUSY;
    hcore->io_configured = 0U;
    memset(&hcore->Init, 0, sizeof(hcore->Init));
    memset(&hcore->IoInit, 0, sizeof(hcore->IoInit));

    s_ptm_core_handle[core] = hcore;

    return HAL_PTM_CoreReset(hcore);
}

HAL_StatusTypeDef HAL_PTM_CoreDeInit(PTM_CoreHandleTypeDef *hcore)
{
    PTM_HandleTypeDef *hptm;

    if (!PTM_IsCoreHandleValid(hcore))
    {
        return HAL_ERROR;
    }

    hptm = hcore->hptm;

    /* No interrupt from this core anymore, it is about to be released. */
    PTM_WriteCoreNotify(hptm, hcore->core, 0U);

    /* Reset clears the core status, its FIFO and its error flags, and stops it. */
    hptm->Instance->CER = PTM_CER_RST_BIT(hcore->core);

    s_ptm_core_handle[hcore->core] = NULL;
    (void)HAL_PTM_CoreFree(hptm, hcore->core);

    hcore->hptm = NULL;
    hcore->State = HAL_PTM_STATE_RESET;

    return HAL_PTM_DeInit(hptm);
}

HAL_StatusTypeDef HAL_PTM_CoreReset(PTM_CoreHandleTypeDef *hcore)
{
    PTM_CoreRegTypeDef *reg;

    if (!PTM_IsCoreHandleValid(hcore))
    {
        return HAL_ERROR;
    }

    /* RSTx clears the status and the registers of the core (not the TCM), so
       the configuration applied through this handle is written back after. */
    hcore->hptm->Instance->CER = PTM_CER_RST_BIT(hcore->core);

    reg = PTM_GetCoreReg(hcore->hptm, hcore->core);
    reg->CCR = MAKE_REG_VAL2(hcore->Init.psc, PTM_CCR0_PSC)
               | MAKE_REG_VAL2(hcore->Init.rep, PTM_CCR0_REP)
               | MAKE_REG_VAL2(hcore->Init.end0, PTM_CCR0_END0)
               | MAKE_REG_VAL2(hcore->Init.end1, PTM_CCR0_END1);

    if (hcore->io_configured != 0U)
    {
        reg->ICR = PTM_PackICR(&hcore->IoInit);
        reg->OCR = PTM_PackOCR(&hcore->IoInit);
    }

    hcore->State = HAL_PTM_STATE_READY;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_PTM_CoreStart(PTM_CoreHandleTypeDef *hcore)
{
    if (!PTM_IsCoreHandleValid(hcore))
    {
        return HAL_ERROR;
    }

    PTM_StartCores(hcore->hptm, PTM_CER_EN_BIT(hcore->core), 0U);
    hcore->State = HAL_PTM_STATE_BUSY;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_PTM_CoreStart_IT(PTM_CoreHandleTypeDef *hcore)
{
    if (!PTM_IsCoreHandleValid(hcore))
    {
        return HAL_ERROR;
    }

    PTM_StartCores(hcore->hptm, PTM_CER_EN_BIT(hcore->core), PTM_CER_EN_BIT(hcore->core));
    hcore->State = HAL_PTM_STATE_BUSY;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_PTM_CoreStop(PTM_CoreHandleTypeDef *hcore)
{
    if (!PTM_IsCoreHandleValid(hcore))
    {
        return HAL_ERROR;
    }

    hcore->hptm->Instance->CER = PTM_CER_STOP_BIT(hcore->core);

    return HAL_OK;
}

HAL_StatusTypeDef HAL_PTM_CoreStartMask(PTM_HandleTypeDef *hptm, uint32_t core_mask, uint32_t it_mask)
{
    if (hptm == NULL)
    {
        return HAL_ERROR;
    }

    if ((core_mask & PTM_CORE_Msk) & ~hptm->core_bitmap)
    {
        return HAL_ERROR;
    }

    PTM_StartCores(hptm, core_mask, it_mask);

    return HAL_OK;
}

HAL_StatusTypeDef HAL_PTM_CoreStopMask(PTM_HandleTypeDef *hptm, uint32_t core_mask)
{
    if (hptm == NULL)
    {
        return HAL_ERROR;
    }

    hptm->Instance->CER = (core_mask & PTM_CORE_Msk) << PTM_CER_STOP0_Pos;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_PTM_CoreStep(PTM_CoreHandleTypeDef *hcore)
{
    if (!PTM_IsCoreHandleValid(hcore))
    {
        return HAL_ERROR;
    }

    hcore->hptm->Instance->CER = PTM_CER_STEP_BIT(hcore->core);

    return HAL_OK;
}

uint32_t HAL_PTM_CoreIsRunning(PTM_CoreHandleTypeDef *hcore)
{
    if (!PTM_IsCoreHandleValid(hcore))
    {
        return 0U;
    }

    return ((hcore->hptm->Instance->CER & PTM_CER_EN_BIT(hcore->core)) != 0U) ? 1U : 0U;
}

HAL_StatusTypeDef HAL_PTM_CoreConfig(PTM_CoreHandleTypeDef *hcore, const PTM_CoreCfgTypeDef *cfg)
{
    PTM_CoreRegTypeDef *reg;

    if (!PTM_IsCoreHandleValid(hcore) || (cfg == NULL))
    {
        return HAL_ERROR;
    }

    if (HAL_PTM_CoreIsRunning(hcore) != 0U)
    {
        return HAL_BUSY;
    }

    reg = PTM_GetCoreReg(hcore->hptm, hcore->core);
    reg->CCR = MAKE_REG_VAL2(cfg->psc, PTM_CCR0_PSC)
               | MAKE_REG_VAL2(cfg->rep, PTM_CCR0_REP)
               | MAKE_REG_VAL2(cfg->end0, PTM_CCR0_END0)
               | MAKE_REG_VAL2(cfg->end1, PTM_CCR0_END1);

    hcore->Init = *cfg;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_PTM_CoreConfigIo(PTM_CoreHandleTypeDef *hcore, const PTM_IoFifoInitTypeDef *cfg)
{
    PTM_CoreRegTypeDef *reg;

    if (!PTM_IsCoreHandleValid(hcore) || (cfg == NULL))
    {
        return HAL_ERROR;
    }

    if (HAL_PTM_CoreIsRunning(hcore) != 0U)
    {
        return HAL_BUSY;
    }

    reg = PTM_GetCoreReg(hcore->hptm, hcore->core);
    reg->ICR = PTM_PackICR(cfg);
    reg->OCR = PTM_PackOCR(cfg);

    hcore->IoInit = *cfg;
    hcore->io_configured = 1U;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_PTM_CoreLoadCode(PTM_CoreHandleTypeDef *hcore, const uint32_t *code, uint32_t words)
{
    uint32_t *tcm;
    uint32_t base_idx;
    uint32_t i;
    uint32_t idx;
    uint32_t mask;

    if (!PTM_IsCoreHandleValid(hcore) || (code == NULL) || (words == 0U) || (words > PTM_TCM_WORDS))
    {
        return HAL_ERROR;
    }

    if (HAL_PTM_CoreIsRunning(hcore) != 0U)
    {
        /* Loading under a running core would mix two programs: load first, then
           start (or stop the core, reset it and load). */
        return HAL_BUSY;
    }

    /* Core N address 0 is TCM[16 * N] (UM5701 18.1.8), so a program longer than
       16 instructions overruns into the following cores.  A slot owned by
       another core is refused, which covers the three ways to collide: loading
       over the window of a core that already has a program, loading over the
       window of a core that is claimed but still empty, and loading into a core
       whose window was overrun by a longer program loaded earlier. */
    base_idx = PTM_TCM_WORDS_PER_CORE * hcore->core;
    mask = PTM_DisableInterrupt();

    for (i = 0U; i < words; i++)
    {
        idx = (base_idx + i) % PTM_TCM_WORDS;
        if (PTM_TcmOwnedByOther(hcore->hptm, hcore->core, idx) != 0)
        {
            PTM_EnableInterrupt(mask);
            return HAL_ERROR;
        }
    }

    /* Nothing else owns the range: write it and remember how far this core
       reaches (a reload replaces the previous length). */
    tcm = PTM_TCM_BASE;
    for (i = 0U; i < words; i++)
    {
        tcm[(base_idx + i) % PTM_TCM_WORDS] = code[i];
    }
    s_ptm_tcm_len[hcore->core] = (uint8_t)words;

    PTM_EnableInterrupt(mask);

    return HAL_OK;
}

uint32_t *HAL_PTM_CoreGetTcmPtr(PTM_CoreHandleTypeDef *hcore)
{
    if (!PTM_IsCoreHandleValid(hcore))
    {
        return NULL;
    }

    return PTM_TCM_BASE + (hcore->core * PTM_TCM_WORDS_PER_CORE);
}

HAL_StatusTypeDef HAL_PTM_CoreSetReg(PTM_CoreHandleTypeDef *hcore, PTM_RegTypeDef reg, uint32_t value)
{
    PTM_CoreRegTypeDef *hw;

    if (!PTM_IsCoreHandleValid(hcore))
    {
        return HAL_ERROR;
    }

    hw = PTM_GetCoreReg(hcore->hptm, hcore->core);

    switch (reg)
    {
    case PTM_REG_X:
        hw->X = value;
        break;

    case PTM_REG_Y:
        hw->Y = value;
        break;

    case PTM_REG_A:
        hw->A = MAKE_REG_VAL2(value, PTM_A0_VAL);   /* A and B are 16 bit registers */
        break;

    case PTM_REG_B:
        hw->B = MAKE_REG_VAL2(value, PTM_B0_VAL);
        break;

    default:
        return HAL_ERROR;               /* PC is read only */
    }

    return HAL_OK;
}

uint32_t HAL_PTM_CoreGetReg(PTM_CoreHandleTypeDef *hcore, PTM_RegTypeDef reg)
{
    PTM_CoreRegTypeDef *hw;

    if (!PTM_IsCoreHandleValid(hcore))
    {
        return 0U;
    }

    hw = PTM_GetCoreReg(hcore->hptm, hcore->core);

    switch (reg)
    {
    case PTM_REG_X:
        return hw->X;

    case PTM_REG_Y:
        return hw->Y;

    case PTM_REG_A:
        return hw->A & PTM_A0_VAL_Msk;

    case PTM_REG_B:
        return hw->B & PTM_B0_VAL_Msk;

    case PTM_REG_PC:
        return hw->PC;

    default:
        return 0U;
    }
}

HAL_StatusTypeDef HAL_PTM_CorePushOFifo(PTM_CoreHandleTypeDef *hcore, uint32_t data)
{
    if (!PTM_IsCoreHandleValid(hcore))
    {
        return HAL_ERROR;
    }

    PTM_GetCoreReg(hcore->hptm, hcore->core)->OFIFO = data;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_PTM_CorePopIFifo(PTM_CoreHandleTypeDef *hcore, uint32_t *data)
{
    if (!PTM_IsCoreHandleValid(hcore) || (data == NULL))
    {
        return HAL_ERROR;
    }

    *data = PTM_GetCoreReg(hcore->hptm, hcore->core)->IFIFO;

    return HAL_OK;
}

uint32_t HAL_PTM_CoreGetFifoStatus(PTM_CoreHandleTypeDef *hcore)
{
    PTM_CoreRegTypeDef *hw;
    uint32_t icr, ocr, flags = 0U;

    if (!PTM_IsCoreHandleValid(hcore))
    {
        return 0U;
    }

    hw = PTM_GetCoreReg(hcore->hptm, hcore->core);
    icr = hw->ICR;
    ocr = hw->OCR;

    if ((icr & PTM_ICR0_IFULL) != 0U)
    {
        flags |= PTM_FIFO_FLAG_IFIFO_FULL;
    }
    if ((icr & PTM_ICR0_IEMPTY) != 0U)
    {
        flags |= PTM_FIFO_FLAG_IFIFO_EMPTY;
    }
    if ((icr & PTM_ICR0_IOF) != 0U)
    {
        flags |= PTM_FIFO_FLAG_IFIFO_OVF;
    }
    if ((ocr & PTM_OCR0_OFULL) != 0U)
    {
        flags |= PTM_FIFO_FLAG_OFIFO_FULL;
    }
    if ((ocr & PTM_OCR0_OEMPTY) != 0U)
    {
        flags |= PTM_FIFO_FLAG_OFIFO_EMPTY;
    }
    if ((ocr & PTM_OCR0_OUF) != 0U)
    {
        flags |= PTM_FIFO_FLAG_OFIFO_UNF;
    }

    return flags;
}

HAL_StatusTypeDef HAL_PTM_CoreClearFifoFlag(PTM_CoreHandleTypeDef *hcore, uint32_t flag_mask)
{
    PTM_CoreRegTypeDef *hw;
    uint32_t icr, ocr;

    if (!PTM_IsCoreHandleValid(hcore) || (hcore->io_configured == 0U))
    {
        return HAL_ERROR;
    }

    /* The configuration bits of ICR/OCR are read/write, so a plain write of the
       overflow flag would clear them: rebuild the whole register from the
       configuration shadow kept by HAL_PTM_CoreConfigIo(). */
    icr = PTM_PackICR(&hcore->IoInit);
    ocr = PTM_PackOCR(&hcore->IoInit);

    if ((flag_mask & PTM_FIFO_FLAG_IFIFO_OVF) != 0U)
    {
        icr |= PTM_ICR0_IOF;            /* write 1 clear */
    }
    if ((flag_mask & PTM_FIFO_FLAG_OFIFO_UNF) != 0U)
    {
        ocr |= PTM_OCR0_OUF;
    }

    hw = PTM_GetCoreReg(hcore->hptm, hcore->core);
    hw->ICR = icr;
    hw->OCR = ocr;

    return HAL_OK;
}

uint32_t HAL_PTM_CoreGetFifoAddr(PTM_CoreHandleTypeDef *hcore, PTM_FifoDirTypeDef dir)
{
    PTM_CoreRegTypeDef *hw;

    if (!PTM_IsCoreHandleValid(hcore))
    {
        return 0U;
    }

    hw = PTM_GetCoreReg(hcore->hptm, hcore->core);

    return (dir == PTM_FIFO_IFIFO) ? (uint32_t) & (hw->IFIFO) : (uint32_t) & (hw->OFIFO);
}

HAL_StatusTypeDef HAL_PTM_CoreGetStatus(PTM_CoreHandleTypeDef *hcore, PTM_CoreStatusTypeDef *status)
{
    PTM_HandleTypeDef *hptm;

    if (!PTM_IsCoreHandleValid(hcore) || (status == NULL))
    {
        return HAL_ERROR;
    }

    hptm = hcore->hptm;

    status->pc = PTM_GetCoreReg(hptm, hcore->core)->PC;
    status->running = ((hptm->Instance->CER & PTM_CER_EN_BIT(hcore->core)) != 0U) ? 1U : 0U;
    status->error = ((hptm->Instance->ISR & PTM_ISR_ERR_BIT(hcore->core)) != 0U) ? 1U : 0U;
    status->tcm_conflict = ((hptm->Instance->ISR & PTM_ISR_TCM_BIT(hcore->core)) != 0U) ? 1U : 0U;
    status->reserved = 0U;

    return HAL_OK;
}

HAL_StatusTypeDef HAL_PTM_CoreRegisterCallback(PTM_CoreHandleTypeDef *hcore,
        PTM_CoreEventCallback_t evt_cb, PTM_ErrorCallback_t err_cb)
{
    if (hcore == NULL)
    {
        return HAL_ERROR;
    }

    hcore->EventCallback = evt_cb;
    hcore->ErrorCallback = err_cb;

    return HAL_OK;
}

/**
 * @}
 */

/** @defgroup PTM_Exported_Functions_Poll Polling mode
 * @{
 */
HAL_StatusTypeDef HAL_PTM_CoreWaitEvent(PTM_CoreHandleTypeDef *hcore, uint32_t timeout_ms)
{
    uint32_t deadline;
    uint32_t bit;

    if (!PTM_IsCoreHandleValid(hcore))
    {
        return HAL_ERROR;
    }

    bit = PTM_ISR_EVT_BIT(hcore->core);
    deadline = PTM_Deadline(timeout_ms);

    while ((hcore->hptm->Instance->ISR & bit) == 0U)
    {
        if (PTM_IsTimeout(deadline))
        {
            return HAL_TIMEOUT;
        }
        HAL_Delay(1U);
    }

    hcore->hptm->Instance->ISR = bit;       /* consume, write 1 clear */

    return HAL_OK;
}

HAL_StatusTypeDef HAL_PTM_CoreWaitDone(PTM_CoreHandleTypeDef *hcore, uint32_t timeout_ms)
{
    uint32_t deadline;
    uint32_t error_bits;
    uint32_t isr;

    if (!PTM_IsCoreHandleValid(hcore))
    {
        return HAL_ERROR;
    }

    /* The end of a run is reported by the hardware with the same flags the
       interrupt mode uses: EVTn is set on completion (IER.EVTAUTOx, enabled by
       CoreStart/CoreStart_IT) and ERRx/TCMx are set when the run is aborted.
       Waiting on those flags instead of CER.ENx tells a completed run from a
       core that was stopped on purpose or that was never started.  The event is
       left pending, so CoreWaitEvent()/GetEvent() still see it. */
    error_bits = PTM_ISR_ERR_BIT(hcore->core) | PTM_ISR_TCM_BIT(hcore->core);
    deadline = PTM_Deadline(timeout_ms);

    for (;;)
    {
        isr = hcore->hptm->Instance->ISR;

        if ((isr & error_bits) != 0U)
        {
            return HAL_ERROR;       /* aborted, the details are in CoreGetStatus() */
        }
        if ((isr & PTM_ISR_EVT_BIT(hcore->core)) != 0U)
        {
            return HAL_OK;          /* the task completed */
        }

        if (PTM_IsTimeout(deadline))
        {
            return HAL_TIMEOUT;
        }

        HAL_Delay(1U);
    }
}

/**
 * @}
 */

/** @defgroup PTM_Exported_Functions_Callback Default callbacks
 * @{
 */

__weak void HAL_PTM_CoreEventCallback(PTM_CoreHandleTypeDef *hcore, uint16_t evt_mask)
{
    /* NOTE: this function should not be modified, when the callback is needed
             the user can register its own with HAL_PTM_CoreRegisterCallback() */
    UNUSED(hcore);
    UNUSED(evt_mask);
}

__weak void HAL_PTM_ErrorCallback(PTM_CoreHandleTypeDef *hcore, uint32_t err_mask)
{
    /* NOTE: this function should not be modified, when the callback is needed
             the user can register its own with HAL_PTM_CoreRegisterCallback() */
    UNUSED(hcore);
    UNUSED(err_mask);
}

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#endif /* HAL_PTM_MODULE_ENABLED && PTM1_BASE */

/// @}
