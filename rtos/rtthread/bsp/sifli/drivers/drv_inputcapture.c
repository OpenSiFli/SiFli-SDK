/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <board.h>

/** @addtogroup bsp_driver Driver IO
  * @{
  */

/** @defgroup drv_inputcapture Input Capture
  * @brief Input Capture BSP driver for GPTIM
  * @{
  */

#ifdef BSP_USING_INPUT_CAPTURE

#include "drv_config.h"
#include "drv_inputcapture.h"
#include "bf0_hal_rcc.h"

//#define DRV_DEBUG
#define LOG_TAG             "drv.incap"
#include <drv_log.h>

#include <rtdevice.h>

#if defined(BSP_USING_INPUT_CAPTURE_GPTIM1) && (defined(BSP_USING_GPTIM1) || defined(BSP_USING_PWMT1))
    #error "BSP_USING_INPUT_CAPTURE_GPTIM1 conflicts with BSP_USING_GPTIM1/BSP_USING_PWMT1"
#endif
#if defined(BSP_USING_INPUT_CAPTURE_GPTIM2) && (defined(BSP_USING_GPTIM2) || defined(BSP_USING_PWMT2))
    #error "BSP_USING_INPUT_CAPTURE_GPTIM2 conflicts with BSP_USING_GPTIM2/BSP_USING_PWMT2"
#endif
#if defined(BSP_USING_INPUT_CAPTURE_GPTIM3) && (defined(BSP_USING_GPTIM3) || defined(BSP_USING_PWMT3))
    #error "BSP_USING_INPUT_CAPTURE_GPTIM3 conflicts with BSP_USING_GPTIM3/BSP_USING_PWMT3"
#endif
#if defined(BSP_USING_INPUT_CAPTURE_GPTIM4) && (defined(BSP_USING_GPTIM4) || defined(BSP_USING_PWMT4))
    #error "BSP_USING_INPUT_CAPTURE_GPTIM4 conflicts with BSP_USING_GPTIM4/BSP_USING_PWMT4"
#endif
#if defined(BSP_USING_INPUT_CAPTURE_GPTIM5) && (defined(BSP_USING_GPTIM5) || defined(BSP_USING_PWMT5))
    #error "BSP_USING_INPUT_CAPTURE_GPTIM5 conflicts with BSP_USING_GPTIM5/BSP_USING_PWMT5"
#endif

/* The HAL dispatches ALL GPTIM input-capture interrupts through a single
 * global HAL_GPT_IC_CaptureCallback / HAL_GPT_PeriodElapsedCallback hook, so
 * this driver must define those hooks (strong) to receive capture/overflow
 * events for its own GPTIMs. Any other driver defining the same hook would be
 * silently overridden (weak) or collide (strong) regardless of which GPTIM
 * each driver uses, so such coexistence is rejected at compile time:
 *  - drv_button.c  defines weak HAL_GPT_IC_CaptureCallback/HAL_GPT_IC_MspInit
 *  - drv_hwtimer.c defines HAL_GPT_PeriodElapsedCallback (strong, or weak on
 *    LB55X CHIP_ID<3 where it would be silently overridden) */

#if defined(BSP_USING_INPUT_CAPTURE) && defined(BSP_USING_BUTTON)
    #error "BSP_USING_INPUT_CAPTURE conflicts with BSP_USING_BUTTON: both drivers override the global HAL_GPT_IC_CaptureCallback hook"
#endif

#define INCAP_NVIC_PRIORITY  3

/* ======================== global variables ======================== */

/* GPTIM start index in bf0_inputcapture_obj[] (each GPTIM has 4 channels),
 * used to get the device directly in the IRQ handler without searching. */
enum
{
#ifdef BSP_USING_INPUT_CAPTURE_GPTIM1
    INPUT_CAPTURE1_INDEX,
#endif
#ifdef BSP_USING_INPUT_CAPTURE_GPTIM2
    INPUT_CAPTURE2_INDEX,
#endif
#ifdef BSP_USING_INPUT_CAPTURE_GPTIM3
    INPUT_CAPTURE3_INDEX,
#endif
#ifdef BSP_USING_INPUT_CAPTURE_GPTIM4
    INPUT_CAPTURE4_INDEX,
#endif
#ifdef BSP_USING_INPUT_CAPTURE_GPTIM5
    INPUT_CAPTURE5_INDEX,
#endif
    INPUT_CAPTURE_MAX,
};

static _incap_irq_ref_t _incap_irq_refs[INPUT_CAPTURE_MAX];

/* One device per GPTIM capture channel, e.g. GPTIM3 -> "incap3"(CH1),
 * "incap3c2"(CH2), "incap3c3"(CH3), "incap3c4"(CH4), so multiple channels
 * of the same GPTIM can capture simultaneously (hardware supports up to 4).
 * Device names must fit RT_NAME_MAX (8). */
#define INCAP_DEV(inst, irq, _core, devname, ch, _ref)               \
    {                                                                \
       .tim_handle.Instance     = inst,                              \
       .tim_irqn                = irq,                               \
       .name                    = devname,                           \
       .core                    = _core,                             \
       .channel                 = ch,                                \
       .ref                     = _ref,                              \
    }

#ifdef BSP_USING_INPUT_CAPTURE_GPTIM1
#define GPTIM1_INPUTCAPTURE_CONFIG                                  \
    INCAP_DEV(GPTIM1, GPTIM1_IRQn, GPTIM1_CORE, "incap1c1", GPT_CHANNEL_1, &_incap_irq_refs[INPUT_CAPTURE1_INDEX]), \
    INCAP_DEV(GPTIM1, GPTIM1_IRQn, GPTIM1_CORE, "incap1c2", GPT_CHANNEL_2, &_incap_irq_refs[INPUT_CAPTURE1_INDEX]), \
    INCAP_DEV(GPTIM1, GPTIM1_IRQn, GPTIM1_CORE, "incap1c3", GPT_CHANNEL_3, &_incap_irq_refs[INPUT_CAPTURE1_INDEX]), \
    INCAP_DEV(GPTIM1, GPTIM1_IRQn, GPTIM1_CORE, "incap1c4", GPT_CHANNEL_4, &_incap_irq_refs[INPUT_CAPTURE1_INDEX])
#endif

#ifdef BSP_USING_INPUT_CAPTURE_GPTIM2
#define GPTIM2_INPUTCAPTURE_CONFIG                                  \
    INCAP_DEV(GPTIM2, GPTIM2_IRQn, GPTIM2_CORE, "incap2c1", GPT_CHANNEL_1, &_incap_irq_refs[INPUT_CAPTURE2_INDEX]), \
    INCAP_DEV(GPTIM2, GPTIM2_IRQn, GPTIM2_CORE, "incap2c2", GPT_CHANNEL_2, &_incap_irq_refs[INPUT_CAPTURE2_INDEX]), \
    INCAP_DEV(GPTIM2, GPTIM2_IRQn, GPTIM2_CORE, "incap2c3", GPT_CHANNEL_3, &_incap_irq_refs[INPUT_CAPTURE2_INDEX]), \
    INCAP_DEV(GPTIM2, GPTIM2_IRQn, GPTIM2_CORE, "incap2c4", GPT_CHANNEL_4, &_incap_irq_refs[INPUT_CAPTURE2_INDEX])
#endif

#if defined(BSP_USING_INPUT_CAPTURE_GPTIM3)
#define GPTIM3_INPUTCAPTURE_CONFIG                                  \
    INCAP_DEV(GPTIM3, GPTIM3_IRQn, GPTIM3_CORE, "incap3c1", GPT_CHANNEL_1, &_incap_irq_refs[INPUT_CAPTURE3_INDEX]), \
    INCAP_DEV(GPTIM3, GPTIM3_IRQn, GPTIM3_CORE, "incap3c2", GPT_CHANNEL_2, &_incap_irq_refs[INPUT_CAPTURE3_INDEX]), \
    INCAP_DEV(GPTIM3, GPTIM3_IRQn, GPTIM3_CORE, "incap3c3", GPT_CHANNEL_3, &_incap_irq_refs[INPUT_CAPTURE3_INDEX]), \
    INCAP_DEV(GPTIM3, GPTIM3_IRQn, GPTIM3_CORE, "incap3c4", GPT_CHANNEL_4, &_incap_irq_refs[INPUT_CAPTURE3_INDEX])
#endif

#if defined(BSP_USING_INPUT_CAPTURE_GPTIM4)
#define GPTIM4_INPUTCAPTURE_CONFIG                                  \
    INCAP_DEV(GPTIM4, GPTIM4_IRQn, GPTIM4_CORE, "incap4c1", GPT_CHANNEL_1, &_incap_irq_refs[INPUT_CAPTURE4_INDEX]), \
    INCAP_DEV(GPTIM4, GPTIM4_IRQn, GPTIM4_CORE, "incap4c2", GPT_CHANNEL_2, &_incap_irq_refs[INPUT_CAPTURE4_INDEX]), \
    INCAP_DEV(GPTIM4, GPTIM4_IRQn, GPTIM4_CORE, "incap4c3", GPT_CHANNEL_3, &_incap_irq_refs[INPUT_CAPTURE4_INDEX]), \
    INCAP_DEV(GPTIM4, GPTIM4_IRQn, GPTIM4_CORE, "incap4c4", GPT_CHANNEL_4, &_incap_irq_refs[INPUT_CAPTURE4_INDEX])
#endif

#if defined(BSP_USING_INPUT_CAPTURE_GPTIM5)
#define GPTIM5_INPUTCAPTURE_CONFIG                                  \
    INCAP_DEV(GPTIM5, GPTIM5_IRQn, GPTIM5_CORE, "incap5c1", GPT_CHANNEL_1, &_incap_irq_refs[INPUT_CAPTURE5_INDEX]), \
    INCAP_DEV(GPTIM5, GPTIM5_IRQn, GPTIM5_CORE, "incap5c2", GPT_CHANNEL_2, &_incap_irq_refs[INPUT_CAPTURE5_INDEX]), \
    INCAP_DEV(GPTIM5, GPTIM5_IRQn, GPTIM5_CORE, "incap5c3", GPT_CHANNEL_3, &_incap_irq_refs[INPUT_CAPTURE5_INDEX]), \
    INCAP_DEV(GPTIM5, GPTIM5_IRQn, GPTIM5_CORE, "incap5c4", GPT_CHANNEL_4, &_incap_irq_refs[INPUT_CAPTURE5_INDEX])
#endif

static struct bf0_inputcapture bf0_inputcapture_obj[] =
{
#ifdef BSP_USING_INPUT_CAPTURE_GPTIM1
    GPTIM1_INPUTCAPTURE_CONFIG,
#endif
#ifdef BSP_USING_INPUT_CAPTURE_GPTIM2
    GPTIM2_INPUTCAPTURE_CONFIG,
#endif
#if defined(BSP_USING_INPUT_CAPTURE_GPTIM3)
    GPTIM3_INPUTCAPTURE_CONFIG,
#endif
#if defined(BSP_USING_INPUT_CAPTURE_GPTIM4)
    GPTIM4_INPUTCAPTURE_CONFIG,
#endif
#if defined(BSP_USING_INPUT_CAPTURE_GPTIM5)
    GPTIM5_INPUTCAPTURE_CONFIG,
#endif
};


/* All channel devices of one GPTIM share the same hardware (base registers,
 * DIER/CCER bit-fields and the NVIC refcount table above), but each channel is
 * an independent rt_device with its own per-device lock. This global mutex
 * serializes the shared-resource parts of init/open/close across channels.
 * open/close are low-frequency operations, so a single lock is sufficient. */
static struct rt_mutex _incap_lock;

static rt_err_t _incap_init(struct rt_inputcapture_device *inputcapture);
static rt_err_t _incap_open(struct rt_inputcapture_device *inputcapture);
static rt_err_t _incap_close(struct rt_inputcapture_device *inputcapture);
static rt_err_t _incap_get_pulsewidth(struct rt_inputcapture_device *inputcapture, rt_uint32_t *pulsewidth_us);

static const struct rt_inputcapture_ops _incap_ops =
{
    .init = _incap_init,
    .open = _incap_open,
    .close = _incap_close,
    .get_pulsewidth = _incap_get_pulsewidth,
};

#if defined(BSP_USING_INPUT_CAPTURE_GPTIM1)
void GPTIM1_IRQHandler(void)
{
    rt_interrupt_enter();
    // TODO: to find a better solution, multiple channels should share the same handle
    HAL_GPT_IRQHandler(&bf0_inputcapture_obj[INPUT_CAPTURE1_INDEX].tim_handle);
    rt_interrupt_leave();
}
#endif

#if defined(BSP_USING_INPUT_CAPTURE_GPTIM2)
void GPTIM2_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_GPT_IRQHandler(&bf0_inputcapture_obj[INPUT_CAPTURE2_INDEX].tim_handle);
    rt_interrupt_leave();
}
#endif

#if defined(BSP_USING_INPUT_CAPTURE_GPTIM3)
void GPTIM3_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_GPT_IRQHandler(&bf0_inputcapture_obj[INPUT_CAPTURE3_INDEX].tim_handle);
    rt_interrupt_leave();
}
#endif

#if defined(BSP_USING_INPUT_CAPTURE_GPTIM4)
void GPTIM4_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_GPT_IRQHandler(&bf0_inputcapture_obj[INPUT_CAPTURE4_INDEX].tim_handle);
    rt_interrupt_leave();
}
#endif

#if defined(BSP_USING_INPUT_CAPTURE_GPTIM5)
void GPTIM5_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_GPT_IRQHandler(&bf0_inputcapture_obj[INPUT_CAPTURE5_INDEX].tim_handle);
    rt_interrupt_leave();
}
#endif

/**
  * @brief  Initialize input capture device.
  * @param[in]  inputcapture: input capture device.
  * @retval RT_EOK if success.
  */
static rt_err_t _incap_init(struct rt_inputcapture_device *inputcapture)
{
    struct bf0_inputcapture *dev;
    GPT_HandleTypeDef *htim;
    GPT_IC_InitTypeDef ic_config = {0};
    uint32_t prescaler_value;
    rt_err_t result = RT_EOK;

    RT_ASSERT(inputcapture != RT_NULL);

    dev = (struct bf0_inputcapture *)inputcapture;
    htim = &dev->tim_handle;

    /* base registers, DIER/CCER bit-fields and the counter of one GPTIM are
     * shared by its 4 channel devices, so serialize with the global lock */
    rt_mutex_take(&_incap_lock, RT_WAITING_FOREVER);

    prescaler_value = HAL_RCC_GetPCLKFreq(dev->core, 1);
#if defined(FIXED_GPTBTIM_SRC_CLK) && defined(GPTIM2)
    if (htim->Instance == GPTIM2)
        prescaler_value = FIXED_GPTBTIM_SRC_CLK;
#endif

    /* configure timer for input capture: 1 MHz (1 us per tick) */
    htim->Init.Prescaler         = prescaler_value / 1000000 - 1;
    htim->Init.CounterMode       = GPT_COUNTERMODE_UP;
    htim->Init.Period            = 0xFFFF;
    htim->Init.RepetitionCounter = 0;

    /* The GPTIM base (prescaler/period/counter) is shared by all channels.
     * HAL_GPT_IC_Init() writes EGR=UG which resets the shared counter, so it
     * may run only for the first channel of each GPTIM: doing it again while
     * another channel is already capturing would corrupt its last_ccr. */
    if (0 == dev->ref->ref)
    {
        if (HAL_GPT_IC_Init(htim) != HAL_OK)
        {
            LOG_E("%s time base init failed", dev->name);
            result = -RT_ERROR;
            goto exit;
        }
    }

    /* configure input capture channel: both edges, no filter, no prescaler */
    ic_config.ICPolarity  = GPT_ICPOLARITY_BOTHEDGE;
    ic_config.ICSelection = GPT_ICSELECTION_DIRECTTI;
    ic_config.ICPrescaler = GPT_ICPSC_DIV1;
    ic_config.ICFilter    = 0;

    if (HAL_GPT_IC_ConfigChannel(htim, &ic_config, dev->channel) != HAL_OK)
    {
        LOG_E("%s channel config failed", dev->name);
        result = -RT_ERROR;
        goto exit;
    }

    dev->first_capture = RT_TRUE;
    dev->in_high       = RT_FALSE;
    dev->last_ts       = 0;
    dev->pulsewidth_us = 0;

    LOG_D("%s init success", dev->name);

exit:
    rt_mutex_release(&_incap_lock);
    return result;
}

/**
  * @brief  Open input capture device.
  * @param[in]  inputcapture: input capture device.
  * @retval RT_EOK if success.
  */
static rt_err_t _incap_open(struct rt_inputcapture_device *inputcapture)
{
    struct bf0_inputcapture *dev;
    int i;
    rt_err_t result = RT_EOK;

    RT_ASSERT(inputcapture != RT_NULL);

    dev = (struct bf0_inputcapture *)inputcapture;

    /* DIER/CCER/CR1 bit-fields and the NVIC refcount table are shared by all
     * channels of one GPTIM, so serialize with the global lock */
    rt_mutex_take(&_incap_lock, RT_WAITING_FOREVER);

    /* Reset the capture state BEFORE starting the hardware: an edge arriving
     * right after HAL_GPT_IC_Start_IT() would otherwise be processed with the
     * previous session's state (first_capture=FALSE and a stale last_ts) and
     * produce a bogus first pulse. On a reopen this window is a few us, which
     * a high-frequency input will almost certainly hit. */
    dev->first_capture = RT_TRUE;
    dev->in_high       = RT_FALSE;
    dev->last_ts       = 0;
    dev->pulsewidth_us = 0;

    /* Start input capture first; only on success increment the refcount and
     * enable the IRQ, otherwise a failure would leak a refcount and leave the
     * IRQ enabled without any channel using it. */
    if (HAL_GPT_IC_Start_IT(&dev->tim_handle, dev->channel) != HAL_OK)
    {
        LOG_E("%s start capture failed", dev->name);
        result = -RT_ERROR;
        goto exit;
    }

    /* set NVIC priority, enable interrupt only on first channel open */
    HAL_NVIC_SetPriority(dev->tim_irqn, INCAP_NVIC_PRIORITY, 0);

    dev->ref->ref++;
    if (1 == dev->ref->ref)
    {

        HAL_NVIC_EnableIRQ(dev->tim_irqn);
    }

    LOG_D("%s open success", dev->name);

exit:
    rt_mutex_release(&_incap_lock);
    return result;
}

/**
  * @brief  Close input capture device.
  * @param[in]  inputcapture: input capture device.
  * @retval RT_EOK if success.
  */
static rt_err_t _incap_close(struct rt_inputcapture_device *inputcapture)
{
    struct bf0_inputcapture *dev;
    int i;

    RT_ASSERT(inputcapture != RT_NULL);

    dev = (struct bf0_inputcapture *)inputcapture;

    rt_mutex_take(&_incap_lock, RT_WAITING_FOREVER);

    /* stop input capture interrupt */
    HAL_GPT_IC_Stop_IT(&dev->tim_handle, dev->channel);

    /* disable NVIC only after the last channel of this GPTIM is closed */
    RT_ASSERT(dev->ref->ref > 0);
    if (0 == --dev->ref->ref)
    {
        HAL_NVIC_DisableIRQ(dev->tim_irqn);
    }

    LOG_D("%s close success", dev->name);
    rt_mutex_release(&_incap_lock);
    return RT_EOK;
}

/**
  * @brief  Get captured pulse width in microseconds.
  *         Called from rt_hw_inputcapture_isr() in interrupt context, so
  *         no mutex is taken here; pulsewidth_us is a 32-bit read/write.
  * @param[in]  inputcapture: input capture device.
  * @param[out] pulsewidth_us: pulse width in microseconds.
  * @retval RT_EOK if success.
  */
static rt_err_t _incap_get_pulsewidth(struct rt_inputcapture_device *inputcapture, rt_uint32_t *pulsewidth_us)
{
    struct bf0_inputcapture *dev;

    RT_ASSERT(inputcapture != RT_NULL);
    RT_ASSERT(pulsewidth_us != RT_NULL);

    dev = (struct bf0_inputcapture *)inputcapture;
    *pulsewidth_us = dev->pulsewidth_us;

    return RT_EOK;
}

/**
  * @brief  Convert HAL active channel (htim->Channel) to channel index
  * @param[in]  active: HAL_GPT_ACTIVE_CHANNEL_x from the HAL callback handle.
  * @retval 0~3, or -1 if invalid.
  */
static int32_t _incap_active_to_channel(uint32_t active)
{
    switch (active)
    {
    case HAL_GPT_ACTIVE_CHANNEL_1:
        return 0;
    case HAL_GPT_ACTIVE_CHANNEL_2:
        return 1;
    case HAL_GPT_ACTIVE_CHANNEL_3:
        return 2;
    case HAL_GPT_ACTIVE_CHANNEL_4:
        return 3;
    default:
        return -1;
    }
}

/**
  * @brief  Input capture interrupt callback from HAL.
  * @param[in]  htim: GPT handle.
  */
void HAL_GPT_IC_CaptureCallback(GPT_HandleTypeDef *htim)
{
    struct bf0_inputcapture *dev;
    uint32_t ccr;
    uint32_t now;
    uint32_t diff;
    int32_t idx;

    idx = _incap_active_to_channel(htim->Channel);
    RT_ASSERT((idx >= 0) && (idx < 4));

    /* htim is the tim_handle of the first channel device of a GPTIM
     * (IRQHandler passes &bf0_inputcapture_obj[INPUT_CAPTUREx_INDEX].tim_handle),
     * so locate that device via container_of, then shift by the active channel
     * (the 4 channel devices of one GPTIM are contiguous in the array). */
    dev = rt_container_of(htim, struct bf0_inputcapture, tim_handle);
    dev = &dev[idx];

    /* read captured counter value */
    switch (htim->Channel)
    {
    case HAL_GPT_ACTIVE_CHANNEL_1:
        ccr = htim->Instance->CCR1;
        break;
    case HAL_GPT_ACTIVE_CHANNEL_2:
        ccr = htim->Instance->CCR2;
        break;
    case HAL_GPT_ACTIVE_CHANNEL_3:
        ccr = htim->Instance->CCR3;
        break;
    case HAL_GPT_ACTIVE_CHANNEL_4:
        ccr = htim->Instance->CCR4;
        break;
    default:
        return;
    }

    /* TODO: only support pulse width less than one period, i.e. 65535us */
    now = ccr;

    if (!dev->first_capture)
    {

        /* compute difference with overflow handling for 16-bit counter */
        if (now >= dev->last_ts)
        {
            diff = now - dev->last_ts;
        }
        else
        {
            diff = (htim->Init.Period + 1) - dev->last_ts + now;
        }

        /* convert ticks to microseconds (timer runs at 1 MHz) */
        dev->pulsewidth_us = diff;

        /*
         * in_high is TRUE  → last edge was rising, current is falling
         *                    → high pulse just ended → is_high = TRUE
         * in_high is FALSE → last edge was falling, current is rising
         *                    → low pulse just ended → is_high = FALSE
         */
        rt_hw_inputcapture_isr(&dev->inputcapture_device, dev->in_high);
    }

    dev->last_ts       = now;
    dev->first_capture = RT_FALSE;
    dev->in_high       = !dev->in_high;
    /* next edge will complete the opposite pulse type */
}

/**
  * @brief  MSP init: enable GPTIM module clock before using it.
  *         GPTIM3/4/5 exist only on some chips; judge each one by the
  *         base-address macro (GPTIM3_BASE) defined in the chip register.h.
  * @param[in]  htim: GPT handle.
  */
void HAL_GPT_IC_MspInit(GPT_HandleTypeDef *htim)
{
#if defined(GPTIM1_BASE)
    if (htim->Instance == GPTIM1)
        HAL_RCC_EnableModule(RCC_MOD_GPTIM1);
#endif
#if defined(GPTIM2_BASE)
    if (htim->Instance == GPTIM2)
        HAL_RCC_EnableModule(RCC_MOD_GPTIM2);
#endif
#if defined(GPTIM3_BASE)
    if (htim->Instance == GPTIM3)
        HAL_RCC_EnableModule(RCC_MOD_GPTIM3);
#endif
#if defined(GPTIM4_BASE)
    if (htim->Instance == GPTIM4)
        HAL_RCC_EnableModule(RCC_MOD_GPTIM4);
#endif
#if defined(GPTIM5_BASE)
    if (htim->Instance == GPTIM5)
        HAL_RCC_EnableModule(RCC_MOD_GPTIM5);
#endif
}

/**
  * @brief  Register all input capture devices.
  * @retval RT_EOK if success.
  */
int incap_register_all(void)
{
    int result = RT_EOK;
    int i;
    int count = sizeof(bf0_inputcapture_obj) / sizeof(bf0_inputcapture_obj[0]);

    rt_mutex_init(&_incap_lock, "incap", RT_IPC_FLAG_PRIO);

    for (i = 0; i < count; i++)
    {
        bf0_inputcapture_obj[i].inputcapture_device.ops = &_incap_ops;

        if (rt_device_inputcapture_register(
                    &bf0_inputcapture_obj[i].inputcapture_device,
                    bf0_inputcapture_obj[i].name,
                    RT_NULL) == RT_EOK)
        {
            LOG_D("%s register success", bf0_inputcapture_obj[i].name);
        }
        else
        {
            LOG_E("%s register failed", bf0_inputcapture_obj[i].name);
            result = -RT_ERROR;
        }
    }

    return result;
}
INIT_DEVICE_EXPORT(incap_register_all);
#endif /* BSP_USING_INPUT_CAPTURE */

/** @} drv_inputcapture */
/** @} bsp_driver */
