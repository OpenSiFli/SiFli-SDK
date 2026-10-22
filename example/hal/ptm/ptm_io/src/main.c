/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * PTM example: the IO channels and the FIFOs seen from the CPU side.
 *
 * A PTM task drives the 16 IO channels through the IO instruction, but the CPU
 * can also use them directly, and it always has to feed or drain the FIFOs:
 *   - HAL_PTM_SetOutputEnable() / HAL_PTM_SetOutput() drive an output channel
 *     without running any task, like a GPIO,
 *   - HAL_PTM_CoreConfigIo() describes how the FIFOs and the IO groups are
 *     wired for a core,
 *   - the FIFOs are raw hardware queues: the HAL reads and writes them, the
 *     caller asks the status first (HAL_PTM_CoreGetFifoStatus()), exactly like
 *     with a UART,
 *   - a FIFO belongs to the task engine of one core, so it only works while that
 *     core is enabled: a core that was initialized but never started is held by
 *     the reset written in HAL_PTM_CoreReset() and its FIFO drops the writes,
 *   - for real transfers the FIFO address is handed to the DMA
 *     (HAL_PTM_CoreGetFifoAddr()) and the core issues the request itself.
 *
 * PTM exists on SF32LB57x only, and this HAL is enabled for the HCPU side.  The
 * pins have to be muxed to the PTM function by the board to see anything on
 * them.
 */
#include "rtthread.h"
#include "bf0_hal.h"

static PTM_CoreHandleTypeDef ptm_core;

/* ------------------------------------------------------------------------- */
/* 0. Pin mux: a PTM channel only reaches a pad once the pad carries it       */
/* ------------------------------------------------------------------------- */

/* Like any other peripheral function, the PTM channels have to be routed to
   pads by the board:
       HAL_PIN_Set(PAD_PA04, PTM1_CH7, PIN_NOPULL, 1);
   Only the pads whose function list contains that PTM channel can be used (the
   DPI board uses PA00..PA10 and PA35/PA36 for CH0..CH10, see
   customer/boards/sf32lb57-dpi-hdk_base/bsp_pinmux.c), and a pad that another
   peripheral already drives must not be taken over: on the spi-hdk board, for
   example, PAD_PA04 (CH7 on the DPI board) is the LCD SPI clock.  Check your own
   board before enabling this. */
#define PTM_IO_PIN_SETUP    1                   /* 1: configure the pin mux below  */
#define PTM_IO_PAD          PAD_PA35            /* adapt: a FREE pad of the board  */
#define PTM_IO_CHANNEL      PTM1_CH0            /* adapt: the channel it carries   */

static void ptm_pin_mux(void)
{
    rt_kprintf("\n[0] pin mux\n");

#if PTM_IO_PIN_SETUP
    (void)HAL_PIN_Set(PTM_IO_PAD, PTM_IO_CHANNEL, PIN_NOPULL, 1);
    rt_kprintf("    pad muxed to the PTM channel\n");
#else
    rt_kprintf("    left to the board: set PTM_IO_PIN_SETUP and adapt the pad,\n");
    rt_kprintf("    or mux it in the bsp_pinmux.c (HAL_PIN_Set + PTM1_CH<n>)\n");
#endif
}

/* ------------------------------------------------------------------------- */
/* 1. Driving an output channel from the CPU                                  */
/* ------------------------------------------------------------------------- */

static void ptm_cpu_output(void)
{
    PTM_HandleTypeDef *hptm = HAL_PTM_GetHandle();
    /* CH0: the channel muxed in section 0.  More channels are driven the same
       way, by muxing their pads and adding them to this mask. */
    uint16_t channels = (uint16_t)(1U << 0);

    rt_kprintf("\n[1] an output channel driven by the CPU\n");

    /* An output is enabled before its value matters, otherwise the channel
       stays in its reset state (high impedance / no drive). */
    (void)HAL_PTM_SetOutputEnable(hptm, channels, channels);
    (void)HAL_PTM_SetOutput(hptm, channels, (uint16_t)(1U << 0));   /* CH0 = 1 */

    rt_kprintf("    enable = 0x%x, value = 0x%x\n",
               (unsigned)HAL_PTM_GetOutputEnable(hptm),
               (unsigned)HAL_PTM_GetOutput(hptm));

    rt_thread_mdelay(10);

    (void)HAL_PTM_SetOutput(hptm, channels, 0U);                    /* both low */
    rt_kprintf("    value after the second write = 0x%x\n",
               (unsigned)HAL_PTM_GetOutput(hptm));

    /* Hand the channels back when they are not needed anymore. */
    (void)HAL_PTM_SetOutputEnable(hptm, channels, 0U);
}

/* ------------------------------------------------------------------------- */
/* 2. The FIFOs: configure, write, and look at the status                     */
/* ------------------------------------------------------------------------- */

/* A core that keeps running without ever touching its FIFOs, so that what the
   CPU writes stays in the OFIFO and the status read below is stable.  A real
   program would drive the IO channels with the words it pulls, or fill the
   IFIFO with what it sampled.
   The task has to be enabled: a core whose handle exists but was never started
   is held by the reset of HAL_PTM_CoreReset() and its FIFOs are dead. */
static const uint32_t program_loop[2] =
{
    PTM_WAIT(0, 0, 100),                            /* 0: pure delay (WAIT NULL), no FIFO access */
    PTM_JMP(-1, PTM_ZERO, JMP_EQ, PTM_ZERO, 0),     /* 1: back to 0 (CCR.REP = 255)     */
};

static void ptm_fifo(void)
{
    PTM_IoFifoInitTypeDef io = {0};
    PTM_CoreCfgTypeDef cfg = {0};
    uint32_t status;

    rt_kprintf("\n[2] the output FIFO\n");

    ptm_core.core = 0U;
    if (HAL_PTM_CoreInit(&ptm_core) != HAL_OK)
    {
        rt_kprintf("    CoreInit failed: is that core already taken?\n");
        return;
    }

    /* The IO and FIFO wiring of a core is described once, by CoreConfigIo():
       here the output FIFO is enabled, the core does not pop it by itself
       (oauto = 0, the program reads a word when it needs one) and the IO group
       starts at IO channel 0 (obase).  The shift and limit fields describe how
       the bits are packed in a FIFO word, see the ICRx/OCR register description
       of UM5701 chapter 18. */
    io.ofifo_en = 1U;
    io.oauto    = 0U;
    io.obase    = 0U;
    (void)HAL_PTM_CoreConfigIo(&ptm_core, &io);

    cfg.psc  = 0U;
    cfg.rep  = 255U;                    /* 255: repeat forever                  */
    cfg.end0 = 1U;                      /* end of one pass, tested after it     */
    cfg.end1 = 0U;
    (void)HAL_PTM_CoreConfig(&ptm_core, &cfg);

    if (HAL_PTM_CoreLoadCode(&ptm_core, program_loop, 2U) != HAL_OK)
    {
        rt_kprintf("    CoreLoadCode failed\n");
        (void)HAL_PTM_CoreDeInit(&ptm_core);
        return;
    }

    /* The core has to run for its FIFO to accept anything, see program_loop. */
    (void)HAL_PTM_CoreStart(&ptm_core);

    /* A raw write, like a UART transmit register: ask the status first. */
    status = HAL_PTM_CoreGetFifoStatus(&ptm_core);
    if ((status & PTM_FIFO_FLAG_OFIFO_FULL) == 0U)
    {
        (void)HAL_PTM_CorePushOFifo(&ptm_core, 0xA5U);
        rt_kprintf("    pushed one word into OFIFO\n");
    }

    status = HAL_PTM_CoreGetFifoStatus(&ptm_core);
    rt_kprintf("    OFIFO is %s and %s\n",
               ((status & PTM_FIFO_FLAG_OFIFO_EMPTY) != 0U) ? "empty" : "not empty",
               ((status & PTM_FIFO_FLAG_OFIFO_FULL) != 0U) ? "full" : "not full");

    /* The underflow/overflow flags are cleared through the HAL, which rebuilds
       ICR/OCR from the configuration so that no configuration bit is lost. */
    (void)HAL_PTM_CoreClearFifoFlag(&ptm_core, PTM_FIFO_FLAG_OFIFO_UNF);

    /* For a real transfer, the DMA is given the address of the FIFO and the
       core requests a burst whenever it has room:
         hdma.Init.Request = PTM1_OFIFO0_DMA_REQUEST;   // see dma_config.h
         HAL_DMA_Start_IT(&hdma, (uint32_t)source,
                          HAL_PTM_CoreGetFifoAddr(&ptm_core, PTM_FIFO_OFIFO), length);
       Nothing in the HAL has to poll the FIFO in that case. */
    rt_kprintf("    OFIFO address for the DMA: 0x%x\n",
               (unsigned)HAL_PTM_CoreGetFifoAddr(&ptm_core, PTM_FIFO_OFIFO));

    /* CoreReset() stops the core, empties the FIFO and clears the registers
       (the TCM keeps its content), CoreDeInit() releases the core and the module
       reference. */
    (void)HAL_PTM_CoreReset(&ptm_core);
    (void)HAL_PTM_CoreDeInit(&ptm_core);
}

/* ------------------------------------------------------------------------- */
/* 3. The input FIFO: who fills it, and when to read it                       */
/* ------------------------------------------------------------------------- */

/* The task side fills the IFIFO.  This program samples 8 IO channels into Y, and
   when ICRx.IAUTO is set the core pushes that value into the IFIFO by itself, so
   the CPU only has to drain it.  (The sampled value depends on what the pads
   carry, see the pin mux note at the top of this file.) */
static const uint32_t program_sample[2] =
{
    PTM_IO(IO_PULL8, IO_NOP, IO_NOP, 0),    /* 0: pull 8 IO channels into Y     */
    PTM_WAIT(0, 0, 0),                      /* 1: end marker (CCR.END0 = 1)     */
};

static void ptm_ififo(void)
{
    PTM_IoFifoInitTypeDef io = {0};
    PTM_CoreCfgTypeDef cfg = {0};
    uint32_t status;
    uint32_t data;

    rt_kprintf("\n[3] the input FIFO\n");

    ptm_core.core = 0U;
    if (HAL_PTM_CoreInit(&ptm_core) != HAL_OK)
    {
        rt_kprintf("    CoreInit failed: is that core already taken?\n");
        return;
    }

    /* Input path: the FIFO is enabled and the core pushes the sampled value into
       it without any CPU intervention (iauto), starting at IO channel 0.  irsh
       right-shifts the sampled bits, exactly what the reference user of this
       path sets (the camera driver samples 8 channels with irsh = 1, see
       drv_dcmi.c).  ishift stays 0: the sample is pushed as it is.
       ilimit is the number of accumulated input bits that triggers the automatic
       push, and 0 is a special value meaning 32 (see the ICRx description of
       UM5701 chapter 18); a single 8 channel sample never reaches 32, so the
       threshold is set to the 8 bits sampled below. */
    io.ififo_en = 1U;
    io.iauto    = 1U;
    io.irsh     = 1U;
    io.ibase    = 0U;
    io.ilimit   = 8U;
    (void)HAL_PTM_CoreConfigIo(&ptm_core, &io);

    cfg.psc  = 0U;
    cfg.rep  = 0U;
    cfg.end0 = 1U;
    cfg.end1 = 0U;
    (void)HAL_PTM_CoreConfig(&ptm_core, &cfg);

    if (HAL_PTM_CoreLoadCode(&ptm_core, program_sample, 2U) != HAL_OK)
    {
        rt_kprintf("    CoreLoadCode failed\n");
        (void)HAL_PTM_CoreDeInit(&ptm_core);
        return;
    }

    (void)HAL_PTM_CoreStart(&ptm_core);
    if (HAL_PTM_CoreWaitDone(&ptm_core, 1000U) != HAL_OK)
    {
        rt_kprintf("    the sampling task did not complete\n");
    }

    /* The core has pushed one sampled word by now.  The CPU side reads it only
       after checking that there is data, because reading an empty FIFO is not a
       defined operation - the same convention as a UART. */
    status = HAL_PTM_CoreGetFifoStatus(&ptm_core);
    if ((status & PTM_FIFO_FLAG_IFIFO_EMPTY) == 0U)
    {
        (void)HAL_PTM_CorePopIFifo(&ptm_core, &data);
        rt_kprintf("    read 0x%x from IFIFO\n", (unsigned)data);
    }
    else
    {
        rt_kprintf("    IFIFO is empty: the task sampled nothing\n");
    }

    (void)HAL_PTM_CoreDeInit(&ptm_core);
}

int main(void)
{
    PTM_HandleTypeDef *hptm = HAL_PTM_GetHandle();

    rt_kprintf("PTM example: IO channels and FIFOs from the CPU side\n");

    /* HAL_PTM_Init() enables the module clock (and resets the module for its
       first user).  HAL_PTM_CoreInit() does it too, for the sections that claim
       a core, but section 1 only drives the IO channels: without the module
       clock every write to the output registers is dropped. */
    if (HAL_PTM_Init(hptm) != HAL_OK)
    {
        rt_kprintf("HAL_PTM_Init failed\n");
        return -1;
    }

    ptm_pin_mux();
    ptm_cpu_output();
    ptm_fifo();
    ptm_ififo();

    (void)HAL_PTM_DeInit(hptm);

    rt_kprintf("\ndone\n");

    return 0;
}
