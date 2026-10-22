/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * PTM example: several cores and interrupt reporting.
 *
 * This example shows the multi core side of the HAL:
 *   - a master core and a helper core started in the same hclk cycle with
 *     HAL_PTM_CoreStartMask(), where only the master reports to the CPU,
 *   - the completion event of the master delivered to a callback instead of
 *     being polled,
 *   - what an aborted task (bus error) looks like, and how to recover.
 *
 * Interrupt mode is selected per core at start time: a core listed in the
 * it_mask of HAL_PTM_CoreStartMask() (or started with HAL_PTM_CoreStart_IT())
 * gets the completion, event and error interrupts; the other cores of the mask
 * only get the completion event and stay silent, which is what a helper core
 * that nobody waits for wants.
 *
 * The vector entry itself belongs to the application, the HAL only provides the
 * dispatcher (HAL_PTM_IRQHandler(core)): see the handlers at the end of this
 * file.
 *
 * PTM exists on SF32LB57x only, and this HAL is enabled for the HCPU side.
 */
#include "rtthread.h"
#include "bf0_hal.h"

/* Static storage: the HAL and the interrupt handler keep pointers to the
   handles. */
static PTM_CoreHandleTypeDef ptm_master;
static PTM_CoreHandleTypeDef ptm_helper;

/* Written from the callbacks, read by the thread: keep them small and never
   call a blocking API from a callback. */
static volatile uint32_t s_master_events;
static volatile uint32_t s_errors;

/* ------------------------------------------------------------------------- */
/* The two programs                                                           */
/* ------------------------------------------------------------------------- */

/* Master, 3 instructions: set X, wait for the helper, then end.
   CCR.END0/END1 are tested after the instruction at that address has run, so the
   WAIT(0, 0, 0) at address 2 is a plain end marker: it executes and the task
   ends right after it. */
static const uint32_t program_master[3] =
{
    PTM_SET(PTM_X, PTM_ZERO, OP_ADD, 0x22U),        /* 0: X = 0x22                     */
    PTM_WAIT(WAIT_EVT1, 1, 0),                      /* 1: wait for the helper (EVT1)    */
    PTM_WAIT(0, 0, 0),                              /* 2: end marker                    */
};

/* Helper, endless: signal the master, touch X and Y, start over.
   CCR.REP = 255 means "repeat forever", so this core never completes and never
   needs to report anything to the CPU. */
static const uint32_t program_helper[3] =
{
    PTM_SET(PTM_EVTS, PTM_ZERO, OP_OR, (1U << 1)),  /* 0: set EVT1, the master waits   */
    PTM_SET(PTM_X, PTM_ZERO, OP_ADD, 1U),           /* 1: X = 1                        */
    PTM_SET(PTM_Y, PTM_Y, OP_ADD, 1U),              /* 2: Y++                          */
};

/* A bus error: LDM reads through an address that is not on the bus, so the core
   aborts on that instruction and the HAL reports it through the error callback.
   The abort also clears CER.EN0, i.e. the core stops by itself.
   A load or a store through a dead address (and an illegal instruction) is what
   the hardware raises ERRn for; a jump is not a fault, even to an address that
   does not exist, so PTM_JMP(JMP_X, ...) would never abort. */
static const uint32_t program_fault[2] =
{
    PTM_LDM(PTM_X, 0xDEADBEEFU),                    /* 0: load a dead address          */
    PTM_WAIT(0, 0, 0),                              /* 1: never reached                */
};

/* A plain program used to show the recovery: it does not depend on any event. */
static const uint32_t program_ok[2] =
{
    PTM_SET(PTM_X, PTM_ZERO, OP_ADD, 0x22U),        /* 0: X = 0x22                     */
    PTM_WAIT(0, 0, 0),                              /* 1: end marker (END0 = 1)        */
};

/* ------------------------------------------------------------------------- */
/* Callbacks                                                                  */
/* ------------------------------------------------------------------------- */

static void ptm_event_cb(PTM_CoreHandleTypeDef *hcore, uint16_t evt_mask)
{
    /* The event flag is already cleared by the HAL dispatcher. */
    (void)hcore;
    (void)evt_mask;
    s_master_events++;
}

static void ptm_error_cb(PTM_CoreHandleTypeDef *hcore, uint32_t err_mask)
{
    /* The HAL masks the error interrupt of that core before calling this
       callback, because the error flags are level and can only be cleared by
       CER.RSTx.  A caller that wants to continue must reset the core (which
       clears the flags) and start it again (which re-arms the interrupt). */
    (void)hcore;
    (void)err_mask;
    s_errors++;
}

/* ------------------------------------------------------------------------- */
/* 1. Master and helper started in the same cycle                             */
/* ------------------------------------------------------------------------- */

static uint32_t ptm_wait_flag(volatile uint32_t *flag, uint32_t timeout_ms)
{
    uint32_t waited = 0U;

    while ((*flag == 0U) && (waited < timeout_ms))
    {
        rt_thread_mdelay(1);
        waited++;
    }

    return *flag;
}

static void ptm_run_master_and_helper(void)
{
    PTM_CoreCfgTypeDef cfg = {0};
    PTM_HandleTypeDef *hptm = HAL_PTM_GetHandle();

    rt_kprintf("\n[1] master and helper started in the same cycle\n");

    ptm_master.core = 0U;
    if (HAL_PTM_CoreInit(&ptm_master) != HAL_OK)
    {
        rt_kprintf("    CoreInit(master) failed\n");
        return;
    }

    ptm_helper.core = 1U;
    if (HAL_PTM_CoreInit(&ptm_helper) != HAL_OK)
    {
        rt_kprintf("    CoreInit(helper) failed\n");
        (void)HAL_PTM_CoreDeInit(&ptm_master);
        return;
    }

    /* Callbacks may be registered before or after CoreInit(). */
    (void)HAL_PTM_CoreRegisterCallback(&ptm_master, ptm_event_cb, ptm_error_cb);
    (void)HAL_PTM_CoreRegisterCallback(&ptm_helper, NULL, NULL);

    cfg.psc  = 0U;                                  /* task clock = hclk          */
    cfg.rep  = 0U;                                  /* the master runs once       */
    cfg.end0 = 2U;
    cfg.end1 = 0U;
    (void)HAL_PTM_CoreConfig(&ptm_master, &cfg);

    cfg.rep  = 255U;                                /* 255: repeat forever        */
    cfg.end0 = 2U;                                  /* end of one pass            */
    (void)HAL_PTM_CoreConfig(&ptm_helper, &cfg);

    if ((HAL_PTM_CoreLoadCode(&ptm_master, program_master, 3U) != HAL_OK) ||
            (HAL_PTM_CoreLoadCode(&ptm_helper, program_helper, 3U) != HAL_OK))
    {
        rt_kprintf("    CoreLoadCode failed\n");
        (void)HAL_PTM_CoreDeInit(&ptm_helper);
        (void)HAL_PTM_CoreDeInit(&ptm_master);
        return;
    }

    s_master_events = 0U;
    s_errors = 0U;

    /* core_mask: the two cores are enabled by a single register write, so they
       start in the same hclk cycle.
       it_mask: only core 0 reports through interrupts, the helper stays silent
       (it would only add useless ISR entries). */
    if (HAL_PTM_CoreStartMask(hptm, (1U << 0) | (1U << 1), (1U << 0)) != HAL_OK)
    {
        rt_kprintf("    CoreStartMask failed\n");
        return;
    }

    if (ptm_wait_flag(&s_master_events, 1000U) != 0U)
    {
        rt_kprintf("    the master completed and called its event callback\n");
    }
    else
    {
        rt_kprintf("    no completion event within 1 s\n");
    }

    /* The helper runs forever, so it is stopped from the CPU side (its Y
       counter shows how many passes it did). */
    (void)HAL_PTM_CoreStop(&ptm_helper);
    rt_thread_mdelay(1);
    rt_kprintf("    helper stopped after Y = %u passes\n",
               (unsigned)HAL_PTM_CoreGetReg(&ptm_helper, PTM_REG_Y));

    (void)HAL_PTM_CoreDeInit(&ptm_helper);
    (void)HAL_PTM_CoreDeInit(&ptm_master);
}

/* ------------------------------------------------------------------------- */
/* 2. An aborted task, and how to recover                                     */
/* ------------------------------------------------------------------------- */

static void ptm_run_error_recovery(void)
{
    PTM_CoreStatusTypeDef status;
    PTM_CoreCfgTypeDef cfg = {0};

    rt_kprintf("\n[2] aborted task and recovery\n");

    ptm_master.core = 0U;
    if (HAL_PTM_CoreInit(&ptm_master) != HAL_OK)
    {
        rt_kprintf("    CoreInit failed\n");
        return;
    }

    (void)HAL_PTM_CoreRegisterCallback(&ptm_master, ptm_event_cb, ptm_error_cb);

    cfg.psc  = 0U;
    cfg.rep  = 0U;
    cfg.end0 = 1U;
    cfg.end1 = 0U;
    (void)HAL_PTM_CoreConfig(&ptm_master, &cfg);

    if (HAL_PTM_CoreLoadCode(&ptm_master, program_fault, 2U) != HAL_OK)
    {
        rt_kprintf("    CoreLoadCode failed\n");
        (void)HAL_PTM_CoreDeInit(&ptm_master);
        return;
    }

    s_errors = 0U;
    (void)HAL_PTM_CoreStart_IT(&ptm_master);        /* interrupt mode             */

    if (ptm_wait_flag(&s_errors, 1000U) != 0U)
    {
        (void)HAL_PTM_CoreGetStatus(&ptm_master, &status);
        rt_kprintf("    error callback called, PC = 0x%x\n", (unsigned)status.pc);
    }
    else
    {
        rt_kprintf("    no error reported within 1 s\n");
    }

    /* Recovery: the error flags are sticky, CoreReset() clears them and
       CoreStart_IT() arms the interrupts again.  Without the reset, the core
       would report the same error forever.  The program used here has no event
       dependency: the helper of section 1 is gone and its EVT1 was cleared when
       the module reference count dropped to zero.
       Loading a program does not clear the rest of the TCM, so END0 has to be
       set again for the new program: it must point inside it, otherwise the core
       would keep running into the instructions the previous program left
       behind. */
    (void)HAL_PTM_CoreReset(&ptm_master);
    cfg.end0 = 1U;                                  /* program_ok ends after its WAIT */
    (void)HAL_PTM_CoreConfig(&ptm_master, &cfg);
    (void)HAL_PTM_CoreLoadCode(&ptm_master, program_ok, 2U);

    s_master_events = 0U;
    (void)HAL_PTM_CoreStart_IT(&ptm_master);
    if (ptm_wait_flag(&s_master_events, 1000U) != 0U)
    {
        rt_kprintf("    recovered: the core runs and completes again\n");
    }
    else
    {
        rt_kprintf("    recovery failed: no completion event within 1 s\n");
    }

    (void)HAL_PTM_CoreDeInit(&ptm_master);
}

int main(void)
{
    rt_kprintf("PTM example: several cores, interrupt reporting\n");

    ptm_run_master_and_helper();
    ptm_run_error_recovery();

    rt_kprintf("\ndone\n");

    return 0;
}

/* ------------------------------------------------------------------------- */
/* Vector entries: the application owns them, the HAL provides the dispatcher */
/* ------------------------------------------------------------------------- */

void PTM1_CORE0_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_PTM_IRQHandler(0);
    rt_interrupt_leave();
}

void PTM1_CORE1_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_PTM_IRQHandler(1);
    rt_interrupt_leave();
}
