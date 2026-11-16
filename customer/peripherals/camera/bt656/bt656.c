/**
 * @file bt656.c
 * @brief Generic BT.656 capture layer: DCMI/DMA setup, multi-buffer lending,
 *        cache handling and recovery.
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * @details
 * This layer knows only the BT.656 protocol and the DCMI controller, never a
 * specific decoder chip. Chip differences are supplied by bt656_decoder_ops
 * (see bt656_decoder.h).
 *
 * Completion criteria:
 *   - progressive (interlace == 0): DCMI FRAME event;
 *   - interlaced (interlace != 0): DMA transfer fully drained (FRAME fires per
 *     line inside VBI, so it cannot mark a field boundary).
 * Both paths converge in bt656_frame_irq(), so buffer switching and dropping
 * behave identically.
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>

#include "system_bf0_ap.h"
#include "dma_config.h"

#include "bt656.h"
#include "bt656_decoder.h"

#define DBG_TAG "bt656"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

/** @brief Internal buffer index: none */
#define BT656_STREAM_NONE       (-1)

/** @brief BT.656 embedded sync codes and comparison masks (protocol constants, independent of the decoder chip) */
struct bt656_sync_codes
{
    rt_uint8_t frame_start_code;    /**< Frame start code */
    rt_uint8_t line_start_code;     /**< Line start code (SAV) */
    rt_uint8_t line_end_code;       /**< Line end code (EAV) */
    rt_uint8_t frame_end_code;      /**< Frame end code */
    rt_uint8_t frame_start_mask;    /**< Comparison mask for the matching code */
    rt_uint8_t line_start_mask;
    rt_uint8_t line_end_mask;
    rt_uint8_t frame_end_mask;
};

static const struct bt656_sync_codes bt656_sync_interlaced =
{
    .frame_start_code = 0xAB,
    .line_start_code  = 0x80,
    .line_end_code    = 0x9D,
    .frame_end_code   = 0xB6,
    .frame_start_mask = 0xFF,
    .line_start_mask  = 0xFF,
    .line_end_mask    = 0xD4,
    .frame_end_mask   = 0xFF,
};

static const struct bt656_sync_codes bt656_sync_progressive =
{
    .frame_start_code = 0xC7,
    .line_start_code  = 0xC7,
    .line_end_code    = 0xDA,
    .frame_end_code   = 0xF1,
    .frame_start_mask = 0x70,
    .line_start_mask  = 0x70,
    .line_end_mask    = 0x70,
    .frame_end_mask   = 0x70,
};

static DCMI_HandleTypeDef s_dcmi;
static DMA_HandleTypeDef  s_dma;

static const struct bt656_decoder_ops *s_ops;
static const struct bt656_format *s_fmt;

static struct rt_semaphore s_sem;
static rt_bool_t s_sem_inited;
static rt_bool_t s_inited;

/* Buffer pool state: modified only inside the interrupt-disabled critical
 * section or in interrupt context */
static volatile rt_bool_t   s_running;
static volatile rt_int32_t  s_write;        /**< Block the DMA is writing */
static volatile rt_int32_t  s_ready;        /**< Completed block waiting to be taken */
static volatile rt_uint32_t s_ready_size;
static volatile rt_uint32_t s_ready_seq;
static volatile rt_uint32_t s_in_use;       /**< Bit n = block n is held by the upper layer */
static volatile rt_uint32_t s_seq;
static volatile rt_uint32_t s_frames;
static volatile rt_uint32_t s_dropped;
static volatile rt_uint32_t s_errors;
static volatile rt_bool_t   s_first;        /**< The first field after start may be partial and is dropped */
/** Set by the DMA transfer-complete callback, consumed by bt656_frame_irq() */
static volatile rt_bool_t   s_dma_done;

/* Tick of the last "no signal" log, used to rate-limit it to about once a second */
static rt_tick_t s_last_nosignal_log;

static const struct dma_config s_dma_cfg =
{
    .Instance     = DCMI_DMA_INSTANCE,
    .request      = DCMI_DMA_REQUEST,
    .dma_irq      = DCMI_DMA_IRQ,
    .dma_irq_prio = DCMI_DMA_IRQ_PRIO,
};

static void bt656_dcmi_prepare(void);
static void bt656_dma_complete(DMA_HandleTypeDef *hdma);

static rt_uint32_t bt656_capture_mode(void)
{
    return (s_fmt->interlace != 0U) ? DCMI_MODE_CONTINUOUS : DCMI_MODE_SNAPSHOT;
}

static rt_uint8_t *bt656_buf_at(rt_int32_t idx)
{
    return (rt_uint8_t *)s_ops->buffer + (rt_uint32_t)idx * s_ops->buffer_stride;
}

/**
 * @brief Pick a block the DMA may write to.
 *
 * @param[in] excluded Block that must not be overwritten yet: the frame about to
 *                     be published, or the frame still sitting in the ready slot.
 *
 * @return Block index; BT656_STREAM_NONE when no block is free.
 */
static rt_int32_t bt656_pick(rt_int32_t excluded)
{
    rt_uint32_t i;
    rt_uint32_t count = s_ops->buffer_count;

    for (i = 1U; i <= count; i++)
    {
        rt_int32_t c = (rt_int32_t)(((rt_uint32_t)s_write + i) % count);

        if ((c == excluded) || ((s_in_use & (1UL << c)) != 0U))
        {
            continue;
        }
        return c;
    }

    return BT656_STREAM_NONE;
}

/**
 * @brief End of a field/frame: the current block is full, decide whether to
 *        publish it and switch to the next block.
 *
 * @note Callable from interrupt context. Only the DMA is stopped; CR.CAPTURE is
 *       left alone because toggling capture once per field wedges the DCMI
 *       state machine.
 *
 * @return RT_TRUE when a new frame was published. The caller in interrupt
 *         context uses it to release the semaphore outside any critical
 *         section; the thread path ignores it.
 */
static rt_bool_t bt656_frame_irq(void)
{
    rt_uint32_t remaining;
    rt_uint32_t size;
    rt_int32_t done = s_write;
    rt_int32_t ready_old = s_ready;
    rt_int32_t next;
    rt_bool_t publish;

    if (done < 0)
    {
        return RT_FALSE;
    }

    /*
     * The DMA transfer-complete callback records a full transfer. Only read
     * the hardware counter while the channel is still owned by this capture
     * (the progressive FRAME event can arrive before the transfer completes);
     * after DMA_FreeChannel() the counter may belong to another user.
     */
    if (s_dma_done)
    {
        s_dma_done = RT_FALSE;
        size = s_fmt->frame_bytes;
    }
    else
    {
        remaining = HAL_DCMI_GetRemainingCount(&s_dcmi);
        if (remaining <= (s_fmt->frame_bytes / 4U))
        {
            size = s_fmt->frame_bytes - remaining * 4U;
        }
        else
        {
            size = s_fmt->frame_bytes;
        }
    }

    if (s_first)
    {
        /* Capture may have started in the middle of a field/frame, so this
         * one does not begin on a boundary */
        s_first = RT_FALSE;
        publish = RT_FALSE;
    }
    else
    {
        /* Do not publish a partial line: the display path assumes whole lines
         * and would keep a full-height frame over incomplete data */
        publish = (size >= s_fmt->stride);
    }

    next = bt656_pick(publish ? done : ready_old);

    if ((next < 0) && publish)
    {
        /*
         * No writable block: the upper layer still holds every other block.
         * Discard the frame just completed and keep this block for the DMA so
         * an already published frame is never overwritten.
         */
        publish = RT_FALSE;
        s_dropped++;
        next = bt656_pick(ready_old);
    }
    if (next < 0)
    {
        next = done;
        publish = RT_FALSE;
        s_dropped++;
    }

    if (publish)
    {
        if (ready_old >= 0)
        {
            /* The previous frame has not been taken yet: drop the old one */
            s_dropped++;
        }
        s_ready_size = size;
        s_ready_seq = ++s_seq;
        s_ready = done;
        s_frames++;
    }

    s_write = next;
    __DSB();

    if (HAL_DCMI_Start_DMA(&s_dcmi, bt656_capture_mode(),
                           (uint32_t)bt656_buf_at(next),
                           s_fmt->frame_bytes / 4U) != HAL_OK)
    {
        s_errors++;
        s_write = BT656_STREAM_NONE;
    }

    return publish;
}

/**
 * @brief Take the frame from the ready slot (mark it in use, invalidate cache).
 *
 * @return RT_EOK when a frame was taken; -RT_ETIMEOUT when the slot is empty.
 */
static rt_err_t bt656_take(struct bt656_frame *frame)
{
    rt_int32_t idx;
    rt_uint32_t size = 0U;
    rt_uint32_t seq = 0U;
    rt_base_t level;

    level = rt_hw_interrupt_disable();
    idx = s_ready;
    if (idx >= 0)
    {
        s_ready = BT656_STREAM_NONE;
        s_in_use |= (1UL << idx);
        size = s_ready_size;
        seq = s_ready_seq;
    }
    rt_hw_interrupt_enable(level);

    if (idx < 0)
    {
        return -RT_ETIMEOUT;
    }

    /* The DMA writes straight into PSRAM, so the CPU must drop the matching
     * cache lines before reading */
    (void)mpu_dcache_invalidate(bt656_buf_at(idx), size);

    frame->data   = bt656_buf_at(idx);
    frame->width  = s_fmt->width;
    frame->height = s_fmt->height;
    frame->stride = s_fmt->stride;
    frame->field  = (s_fmt->interlace != 0U) ? 1U : 0U;
    frame->seq    = seq;
    frame->bytes  = size;
    frame->token  = (void *)(rt_uint32_t)(idx + 1U);   /* 0 means "none" */

    /* Give out only what was actually received: narrow height when the byte
     * count is below the theoretical line count */
    if ((frame->stride > 0U) && (size > 0U))
    {
        rt_uint32_t lines = size / frame->stride;

        if ((lines > 0U) && (lines < frame->height))
        {
            frame->height = (rt_uint16_t)lines;
        }
    }

    return RT_EOK;
}

/**
 * @brief Reset the buffer pool and clear the blocks that are not lent out
 *        (all-zero YUV is black).
 *
 * @param[in] reset_stats Also clear the statistics counters; the recovery path
 *                        must keep the accumulated values.
 * @param[in] keep_in_use Keep the blocks held by the upper layer: their
 *                        contents and their "in use" bits are left untouched,
 *                        because the upper layer may still be reading them.
 *
 * @return RT_TRUE when at least one block is free.
 */
static rt_bool_t bt656_pool_reset(rt_bool_t reset_stats, rt_bool_t keep_in_use)
{
    rt_uint32_t i;
    rt_base_t level;
    rt_uint32_t count = s_ops->buffer_count;
    rt_uint32_t in_use;
    rt_uint32_t all;

    level = rt_hw_interrupt_disable();
    s_write = BT656_STREAM_NONE;
    s_ready = BT656_STREAM_NONE;
    if (!keep_in_use)
    {
        s_in_use = 0U;
    }
    in_use = s_in_use;
    s_first = RT_TRUE;
    s_dma_done = RT_FALSE;
    if (reset_stats)
    {
        s_seq = 0U;
        s_frames = 0U;
        s_dropped = 0U;
        s_errors = 0U;
    }
    rt_hw_interrupt_enable(level);

    for (i = 0U; i < count; i++)
    {
        rt_uint8_t *buf;

        if ((in_use & (1UL << i)) != 0U)
        {
            continue;
        }

        buf = bt656_buf_at((rt_int32_t)i);
        memset(buf, 0, s_fmt->frame_bytes);
        (void)mpu_dcache_clean(buf, s_ops->buffer_stride);
        (void)mpu_dcache_invalidate(buf, s_ops->buffer_stride);
    }
    __DSB();

    all = (count >= 32U) ? 0xFFFFFFFFUL : ((1UL << count) - 1UL);
    return ((all & ~in_use) != 0U) ? RT_TRUE : RT_FALSE;
}

/** @brief Whether the last recovery attempt was deferred because every block
 *         is still lent out (used to log it only once) */
static rt_bool_t s_recover_deferred;

/**
 * @brief Last-resort restart when the DCMI state machine is stuck.
 *
 * @note Measurements show that rewriting the registers or re-arming the DMA
 *       alone does not bring it back; the module must be reset first.
 *
 * @note Blocks held by the upper layer keep their contents and their "in use"
 *       bit: a frame is valid from bt656_get() until bt656_release(). When the
 *       upper layer holds every block the restart is skipped and retried on the
 *       next timeout, after the consumer has returned a buffer.
 */
static void bt656_recover(void)
{
    rt_base_t level;
    rt_uint32_t count = s_ops->buffer_count;
    rt_uint32_t in_use;
    rt_uint32_t all;
    rt_uint32_t i;
    rt_int32_t idx;

    all = (count >= 32U) ? 0xFFFFFFFFUL : ((1UL << count) - 1UL);

    level = rt_hw_interrupt_disable();
    in_use = s_in_use;
    rt_hw_interrupt_enable(level);

    /*
     * Defensive check: bt656_pick() never arms an in-use block, so the
     * consumer can hold at most count - 1 blocks and this branch is not
     * expected on real hardware. Keep it so a future change to the pool
     * rules cannot make recovery overwrite a frame that is still lent out.
     */
    if ((in_use & all) == all)
    {
        if (!s_recover_deferred)
        {
            s_recover_deferred = RT_TRUE;
            LOG_W("unexpected: all %u buffers lent out, recovery deferred",
                  (unsigned int)count);
        }
        return;
    }
    s_recover_deferred = RT_FALSE;

    level = rt_hw_interrupt_disable();
    s_running = RT_FALSE;
    s_write = BT656_STREAM_NONE;
    s_ready = BT656_STREAM_NONE;
    rt_hw_interrupt_enable(level);

    (void)HAL_DCMI_Stop(&s_dcmi);
    bt656_dcmi_prepare();

    if (HAL_DCMI_Init(&s_dcmi) != HAL_OK)
    {
        s_errors++;
        return;
    }

    if (!bt656_pool_reset(RT_FALSE, RT_TRUE))
    {
        LOG_E("no free buffer after DCMI reset, capture stays stopped");
        return;
    }

    /* bt656_pool_reset() cleared s_ready, so no block can be taken while the
     * module was being reset; the first free block is stable here */
    level = rt_hw_interrupt_disable();
    in_use = s_in_use;
    for (i = 0U; i < count; i++)
    {
        if ((in_use & (1UL << i)) == 0U)
        {
            break;
        }
    }
    idx = (i < count) ? (rt_int32_t)i : BT656_STREAM_NONE;
    s_write = idx;
    s_running = RT_TRUE;
    rt_hw_interrupt_enable(level);

    if ((idx < 0) ||
            (HAL_DCMI_Start_DMA(&s_dcmi, bt656_capture_mode(),
                                (uint32_t)bt656_buf_at(idx),
                                s_fmt->frame_bytes / 4U) != HAL_OK))
    {
        s_errors++;
        s_running = RT_FALSE;
        s_write = BT656_STREAM_NONE;
        return;
    }

    LOG_W("capture restarted after DCMI module reset");
}

/**
 * @brief Register the single BT.656 decoder chip implementation.
 */
int bt656_register_decoder(const struct bt656_decoder_ops *ops)
{
    if ((ops == RT_NULL) || (ops->format == RT_NULL) ||
            (ops->buffer == RT_NULL) || (ops->buffer_count < 2U) ||
            (ops->buffer_count > 32U) || (ops->buffer_stride == 0U))
    {
        return -RT_EINVAL;
    }

    if (s_ops != RT_NULL)
    {
        return -RT_EBUSY;
    }

    s_ops = ops;
    return RT_EOK;
}

static void bt656_fill_dcmi_config(void)
{
    const struct bt656_sync_codes *sync;
    rt_uint32_t irq = DCMI_IT_OVR | DCMI_IT_ERR | DCMI_IT_PACK;

    sync = (s_fmt->interlace != 0U) ? &bt656_sync_interlaced : &bt656_sync_progressive;

    s_dcmi.Instance = hwp_dcmi;

    s_dcmi.Init.SynchroMode       = DCMI_SYNCHRO_EMBEDDED;
    /*
     * Keep the CR value that was validated on this board: PCKPOL set and both
     * HS/VS polarities low. With embedded sync the sampled BT.656 codes depend
     * on the pixel-clock edge, so flipping PCKPOL makes the FSM never lock and
     * the DMA counter stays at its initial value.
     */
    s_dcmi.Init.PCKPolarity       = DCMI_PCKPOLARITY_FALLING;
    s_dcmi.Init.VSPolarity        = DCMI_VSPOLARITY_LOW;
    s_dcmi.Init.HSPolarity        = DCMI_HSPOLARITY_LOW;
    s_dcmi.Init.ExtendedDataMode  = DCMI_EXTEND_DATA_8B;

    s_dcmi.Init.SyncroCode.FrameStartCode = sync->frame_start_code;
    s_dcmi.Init.SyncroCode.LineStartCode  = sync->line_start_code;
    s_dcmi.Init.SyncroCode.LineEndCode    = sync->line_end_code;
    s_dcmi.Init.SyncroCode.FrameEndCode   = sync->frame_end_code;

    s_dcmi.Init.SyncUnmask.FrameStartUnmask = sync->frame_start_mask;
    s_dcmi.Init.SyncUnmask.LineStartUnmask  = sync->line_start_mask;
    s_dcmi.Init.SyncUnmask.LineEndUnmask    = sync->line_end_mask;
    s_dcmi.Init.SyncUnmask.FrameEndUnmask   = sync->frame_end_mask;

    s_dcmi.Init.ExtendedSyncSwap  = 1U;         /* BT.656 data is byte-swapped inside each 32-bit word */
    s_dcmi.Init.DmaBurstNum       = 16U;
    s_dcmi.Init.CaptureMode       = bt656_capture_mode();
    s_dcmi.Init.Interface         = DCMI_INTERFACE_DVP;
    s_dcmi.Init.IrqMode           = DCMI_IRQ_MODE_APPLICATION;

    if (s_fmt->interlace == 0U)
    {
        irq |= DCMI_IT_FRAME;
    }
    s_dcmi.Init.IrqMask = irq;
}

/**
 * @brief Prepare the DCMI block for a new capture session.
 *
 * @details The validated CVBS sequence resets the block with the clock gated:
 *          a reset pulse taken while the clock is running makes the interlaced
 *          FSM receive zero bytes. HAL_DCMI_Init() pulses the reset after
 *          enabling the clock, so for interlaced formats the reset is done
 *          here and skipped inside the HAL (Init.SkipReset).
 *
 * @note Must be called immediately before HAL_DCMI_Init().
 */
static void bt656_dcmi_prepare(void)
{
    if ((s_fmt != RT_NULL) && (s_fmt->interlace != 0U))
    {
        HAL_RCC_DisableModule(RCC_MOD_DCMI);
        HAL_RCC_ResetModule(RCC_MOD_DCMI);
        rt_thread_mdelay(2);
        s_dcmi.Init.SkipReset = 1U;
    }
    else
    {
        s_dcmi.Init.SkipReset = 0U;
    }
}

static int bt656_dma_init(void)
{
    HAL_RCC_EnableModule(RCC_MOD_DMAC1);

    s_dma.Instance                 = s_dma_cfg.Instance;
    s_dma.Init.Request             = s_dma_cfg.request;
    s_dma.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    s_dma.Init.PeriphInc           = DMA_PINC_ENABLE;
    s_dma.Init.MemInc              = DMA_MINC_ENABLE;
    s_dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    s_dma.Init.MemDataAlignment    = DMA_MDATAALIGN_WORD;
    s_dma.Init.Mode                = DMA_NORMAL;
    s_dma.Init.Priority            = DMA_PRIORITY_HIGH;
    s_dma.Init.BurstSize           = 0x3;
#ifdef DMA_SUPPORT_DYN_CHANNEL_ALLOC
    s_dma.Init.IrqPrio             = s_dma_cfg.dma_irq_prio;
#endif

    if (HAL_DMA_Init(&s_dma) != HAL_OK)
    {
        return -RT_EIO;
    }

    /*
     * Completion runs from the DMA interrupt. HAL_DCMI_Start_DMA() in
     * application-managed mode deliberately leaves this callback alone, so it
     * stays installed across every buffer re-arm.
     */
    s_dma.XferCpltCallback = bt656_dma_complete;

    __HAL_LINKDMA(&s_dcmi, DMA_Handle, s_dma);

    /*
     * DCMI and DMA interrupts use the same priority: both arrive almost
     * simultaneously at the end of a frame, and if the DMA interrupt could
     * preempt the DCMI one, the HAL_DMA_Start_IT() inside bt656_frame_irq()
     * would hit the DMA handle lock and return HAL_BUSY.
     */
    HAL_NVIC_SetPriority(DCMI_IRQn, s_dma_cfg.dma_irq_prio, 0);
    HAL_NVIC_EnableIRQ(DCMI_IRQn);

#if !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
    /* Static channel: nothing else enables this interrupt. In the dynamic
     * allocation build DMA_AllocChannel() owns it. */
    HAL_NVIC_SetPriority(s_dma_cfg.dma_irq, s_dma_cfg.dma_irq_prio, 0);
    HAL_NVIC_EnableIRQ(s_dma_cfg.dma_irq);
#endif

    return RT_EOK;
}

int bt656_init(void)
{
    int ret;

    if (s_inited)
    {
        return RT_EOK;
    }
    if (s_ops == RT_NULL)
    {
        return -RT_EIO;
    }

    s_fmt = s_ops->format();
    if ((s_fmt == RT_NULL) || (s_fmt->frame_bytes == 0U) ||
            ((s_fmt->frame_bytes & 3U) != 0U))
    {
        LOG_E("invalid format");
        return -RT_EINVAL;
    }

    /* A block must be able to hold a whole frame: bt656_buf_at() indexes the
     * pool by this stride */
    if (s_ops->buffer_stride < s_fmt->frame_bytes)
    {
        LOG_E("buffer stride %u smaller than frame size %u",
              (unsigned int)s_ops->buffer_stride,
              (unsigned int)s_fmt->frame_bytes);
        return -RT_EINVAL;
    }

    if (s_ops->init != RT_NULL)
    {
        ret = s_ops->init();
        if (ret != RT_EOK)
        {
            return ret;
        }
    }

    bt656_fill_dcmi_config();

    ret = bt656_dma_init();
    if (ret != RT_EOK)
    {
        return ret;
    }

    if (!s_sem_inited)
    {
        ret = rt_sem_init(&s_sem, "bt656", 0, RT_IPC_FLAG_FIFO);
        if (ret != RT_EOK)
        {
            return ret;
        }
        s_sem_inited = RT_TRUE;
    }

    s_inited = RT_TRUE;
    LOG_I("ready: %s %ux%u, %u buffers", s_fmt->name, s_fmt->width,
          s_fmt->height, (unsigned int)s_ops->buffer_count);
    return RT_EOK;
}

int bt656_deinit(void)
{
    if (!s_inited)
    {
        return RT_EOK;
    }

    (void)bt656_stop();

    (void)HAL_DCMI_DeInit(&s_dcmi);
    (void)HAL_DMA_DeInit(&s_dma);
    NVIC_DisableIRQ(DCMI_IRQn);
#if !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
    NVIC_DisableIRQ(s_dma_cfg.dma_irq);
#endif

    if (s_ops->deinit != RT_NULL)
    {
        (void)s_ops->deinit();
    }

    s_inited = RT_FALSE;
    return RT_EOK;
}

int bt656_start(void)
{
    int ret;

    if ((!s_inited) || (s_ops == RT_NULL) || (s_fmt == RT_NULL))
    {
        return -RT_EIO;
    }
    if (s_running)
    {
        return RT_EOK;
    }

    if (s_ops->configure != RT_NULL)
    {
        ret = s_ops->configure(s_fmt);
        if (ret != RT_EOK)
        {
            LOG_E("decoder configure failed: %d", ret);
            return ret;
        }
    }

    if (s_ops->wait_ready != RT_NULL)
    {
        ret = s_ops->wait_ready(800);
        if (ret != RT_EOK)
        {
            LOG_W("decoder lock not stable, continue");
        }
    }

    /* New session: reset DCMI and re-apply the configuration */
    bt656_dcmi_prepare();
    if (HAL_DCMI_Init(&s_dcmi) != HAL_OK)
    {
        LOG_E("dcmi init failed");
        return -RT_EIO;
    }

    (void)bt656_pool_reset(RT_TRUE, RT_FALSE);

    {
        rt_base_t level = rt_hw_interrupt_disable();

        s_write = 0;
        s_running = RT_TRUE;
        rt_hw_interrupt_enable(level);
    }

    if (HAL_DCMI_Start_DMA(&s_dcmi, bt656_capture_mode(),
                           (uint32_t)bt656_buf_at(0),
                           s_fmt->frame_bytes / 4U) != HAL_OK)
    {
        LOG_E("dcmi start dma failed");
        s_running = RT_FALSE;
        s_write = BT656_STREAM_NONE;
        return -RT_EIO;
    }

    LOG_I("started: %s, %u buffers", s_fmt->name,
          (unsigned int)s_ops->buffer_count);
    return RT_EOK;
}

int bt656_stop(void)
{
    rt_base_t level;

    if (!s_running)
    {
        return RT_EOK;
    }

    level = rt_hw_interrupt_disable();
    s_running = RT_FALSE;
    rt_hw_interrupt_enable(level);

    (void)HAL_DCMI_Stop(&s_dcmi);

    level = rt_hw_interrupt_disable();
    s_write = BT656_STREAM_NONE;
    s_ready = BT656_STREAM_NONE;
    s_in_use = 0U;
    s_dma_done = RT_FALSE;
    rt_hw_interrupt_enable(level);

    LOG_I("stopped: %u frames, drop=%u err=%u", (unsigned int)s_frames,
          (unsigned int)s_dropped, (unsigned int)s_errors);
    return RT_EOK;
}

int bt656_get(struct bt656_frame *frame, rt_int32_t timeout_ms)
{
    rt_tick_t start;
    rt_tick_t tmo;

    if (frame == RT_NULL)
    {
        return -RT_EINVAL;
    }

    frame->data = RT_NULL;
    frame->token = RT_NULL;

    if (!s_running || (s_fmt == RT_NULL))
    {
        return -RT_EIO;
    }

    start = rt_tick_get();
    tmo = (timeout_ms <= 0) ? 0 : rt_tick_from_millisecond(timeout_ms);

    for (;;)
    {
        if (!s_running)
        {
            return -RT_EIO;
        }

        if (bt656_take(frame) == RT_EOK)
        {
            return RT_EOK;
        }

        if (tmo == 0)
        {
            return -RT_ETIMEOUT;
        }

        if ((rt_tick_get() - start) >= tmo)
        {
            if (s_running)
            {
                /*
                 * With no signal, resetting DCMI and wiping the whole pool on
                 * every timeout would never end and would flood the log, so
                 * only rate-limit the message and keep waiting for a source.
                 */
                if ((s_ops->signal_locked != RT_NULL) && !s_ops->signal_locked())
                {
                    rt_tick_t now = rt_tick_get();

                    if ((s_last_nosignal_log == 0U) ||
                            ((now - s_last_nosignal_log) >= rt_tick_from_millisecond(1000)))
                    {
                        s_last_nosignal_log = now;
                        LOG_W("no frame in %d ms, no signal, waiting for source",
                              (int)timeout_ms);
                    }
                }
                else
                {
                    LOG_W("no frame in %d ms, restart capture", (int)timeout_ms);
                    bt656_recover();
                }
            }
            return -RT_ETIMEOUT;
        }

        /* Must block and wait, otherwise the capture thread would starve the
         * display thread */
        (void)rt_sem_take(&s_sem, rt_tick_from_millisecond(1));
    }
}

void bt656_release(struct bt656_frame *frame)
{
    rt_uint32_t idx;
    rt_base_t level;

    if ((frame == RT_NULL) || (frame->token == RT_NULL))
    {
        return;
    }

    idx = (rt_uint32_t)frame->token - 1U;
    if ((s_ops != RT_NULL) && (idx < s_ops->buffer_count))
    {
        level = rt_hw_interrupt_disable();
        s_in_use &= ~(1UL << idx);
        rt_hw_interrupt_enable(level);
    }

    frame->token = RT_NULL;
    frame->data = RT_NULL;
}

void bt656_stats(rt_uint32_t *frames, rt_uint32_t *dropped,
                 rt_uint32_t *errors)
{
    if (frames != RT_NULL)
    {
        *frames = s_frames;
    }
    if (dropped != RT_NULL)
    {
        *dropped = s_dropped;
    }
    if (errors != RT_NULL)
    {
        *errors = s_errors;
    }
}

int bt656_set_source(rt_uint8_t source)
{
    if ((s_ops == RT_NULL) || (s_ops->set_source == RT_NULL))
    {
        return -RT_EIO;
    }

    return s_ops->set_source(source);
}

const struct bt656_format *bt656_get_format(void)
{
    return s_fmt;
}

rt_uint16_t bt656_detect(void)
{
    if ((s_ops == RT_NULL) || (s_ops->detect == RT_NULL))
    {
        return 0U;
    }

    return s_ops->detect();
}

rt_uint8_t bt656_signal_locked(void)
{
    if ((s_ops == RT_NULL) || (s_ops->signal_locked == RT_NULL))
    {
        return 0U;
    }

    return s_ops->signal_locked();
}

/**
 * @brief DMA transfer-complete callback (runs from the DMA interrupt).
 *
 * @details The HAL has already released the channel when this callback runs,
 *          so the hardware counter must not be used here. For interlaced
 *          formats this is the field boundary: switch the buffer and re-arm
 *          immediately. For progressive formats the DCMI FRAME event is still
 *          the boundary; only record that the whole transfer completed so
 *          bt656_frame_irq() takes the size from the format instead of a
 *          counter that may now belong to another DMA user.
 */
static void bt656_dma_complete(DMA_HandleTypeDef *hdma)
{
    (void)hdma;

    if (!s_running || (s_fmt == RT_NULL))
    {
        return;
    }

    s_dma_done = RT_TRUE;

    if (s_fmt->interlace != 0U)
    {
        if (bt656_frame_irq())
        {
            rt_sem_release(&s_sem);
        }
    }
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
    (void)hdcmi;

    if (s_running && (s_fmt != RT_NULL) && (s_fmt->interlace == 0U))
    {
        /*
         * Release the semaphore here rather than inside bt656_frame_irq():
         * the thread path calls that function with interrupts disabled, and
         * entering the scheduler with PRIMASK set is not allowed.
         */
        if (bt656_frame_irq())
        {
            rt_sem_release(&s_sem);
        }
    }
}

void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *hdcmi)
{
    (void)hdcmi;

    if (s_running)
    {
        s_errors++;
    }
}

void DCMI_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_DCMI_IRQHandler(&s_dcmi);
    rt_interrupt_leave();
}

#if !defined(DMA_SUPPORT_DYN_CHANNEL_ALLOC)
void DCMI_DMA_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_DMA_IRQHandler(&s_dma);
    rt_interrupt_leave();
}
#endif
