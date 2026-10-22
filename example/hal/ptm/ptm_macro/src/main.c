/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * PTM example: a task program written with the ptm_ins.h instruction macros.
 *
 * This example does not use the assembler (ptm_asm.h).  The program is a plain
 * array of instructions, which is the simplest way to use PTM when the code is
 * short and its jump offsets are easy to count by hand.  As soon as a program
 * has labels or forward references, the assembler of example/hal/ptm is much
 * easier to maintain.
 *
 * The flow of a task is always the same, only the mode changes:
 *   HAL_PTM_CoreInit()      claim a core, and take a reference on the module
 *   HAL_PTM_CoreConfig()    CCR: task clock divider, repetition, end addresses
 *   HAL_PTM_CoreLoadCode()  copy the program into the TCM window of that core
 *   HAL_PTM_CoreStart()     run it without interrupts (HAL_PTM_CoreStart_IT()
 *                           would report completion and errors through the
 *                           callbacks instead)
 *   HAL_PTM_CoreWaitDone()  wait for the completion of the task
 *   HAL_PTM_CoreGetReg()    read the result back
 *   HAL_PTM_CoreDeInit()    release the core and the module reference
 *
 * PTM exists on SF32LB57x only, and this HAL is enabled for the HCPU side.
 */
#include "rtthread.h"
#include "bf0_hal.h"

/* A core handle must have static storage (or live inside a device object):
   the HAL and the interrupt handler keep a pointer to it. */
static PTM_CoreHandleTypeDef ptm_core;

/* ------------------------------------------------------------------------- */
/* 1. Straight line program                                                   */
/* ------------------------------------------------------------------------- */

/* The two instructions are at TCM address 0 and 1, and CCR.END0 is set to 1: the
   instruction at address 1 runs first, and the task ends right after it.  A core
   executes its own TCM window, the address 0 of core N is TCM[16 * N] and the
   HAL does that translation. */
static const uint32_t program_straight[2] =
{
    PTM_SET(PTM_X, PTM_ZERO, OP_ADD, 0x1234U),      /* 0: X = 0x1234            */
    PTM_SET(PTM_Y, PTM_X,    OP_ADD, 1U),           /* 1: Y = X + 1, task ends  */
};

static void ptm_run_straight(void)
{
    PTM_CoreCfgTypeDef cfg = {0};

    rt_kprintf("\n[1] straight line program, polling mode\n");

    ptm_core.core = 0U;                             /* core index or PTM_CORE_ANY */
    if (HAL_PTM_CoreInit(&ptm_core) != HAL_OK)
    {
        rt_kprintf("    CoreInit failed: is that core already taken?\n");
        return;
    }

    cfg.psc  = 0U;                                  /* task clock = hclk          */
    cfg.rep  = 0U;                                  /* run the program once       */
    cfg.end0 = 1U;                                  /* task ends at address 1     */
    cfg.end1 = 0U;                                  /* address 0 never ends it    */
    (void)HAL_PTM_CoreConfig(&ptm_core, &cfg);

    if (HAL_PTM_CoreLoadCode(&ptm_core, program_straight, 2U) != HAL_OK)
    {
        rt_kprintf("    CoreLoadCode failed\n");
        (void)HAL_PTM_CoreDeInit(&ptm_core);
        return;
    }

    (void)HAL_PTM_CoreStart(&ptm_core);             /* polling mode               */

    /* WaitDone() reports how the task ended: HAL_OK when it completed, HAL_ERROR
       when the core aborted (bus error, illegal instruction, TCM collision) and
       HAL_TIMEOUT when nothing happened before the timeout. */
    if (HAL_PTM_CoreWaitDone(&ptm_core, 1000U) == HAL_OK)
    {
        rt_kprintf("    completed: X = 0x%x, Y = 0x%x\n",
                   (unsigned)HAL_PTM_CoreGetReg(&ptm_core, PTM_REG_X),
                   (unsigned)HAL_PTM_CoreGetReg(&ptm_core, PTM_REG_Y));
    }
    else
    {
        rt_kprintf("    the task did not complete\n");
    }

    /* The program can be started again as it is; to replace it, stop the core
       first: HAL_PTM_CoreReset() then HAL_PTM_CoreLoadCode(). */
    (void)HAL_PTM_CoreDeInit(&ptm_core);
}

/* ------------------------------------------------------------------------- */
/* 2. Loop with a hand written jump offset                                    */
/* ------------------------------------------------------------------------- */

/* The target of a PTM JMP is relative to the JMP instruction itself, so the
   offset is counted from its own address and not from the next instruction:
       instruction 2 jumps back to instruction 1 -> offset = 1 - 2 = -1
   B is used as the loop counter: JMP_NEQDEC jumps while B != 0 and decrements
   it, so the body runs six times (five jumps plus the fall through) and X ends
   at 6 * 3 = 18.
   END0/END1 are tested after the instruction at that address has run, so the
   WAIT at the end address is an end marker that executes (with a zero delay
   here) and the task ends right after it. */
static const uint32_t program_loop[4] =
{
    PTM_SET(PTM_B, PTM_ZERO, OP_ADD, 5U),           /* 0: B = 5                   */
    PTM_SET(PTM_X, PTM_X,    OP_ADD, 3U),           /* 1: X += 3                  */
    PTM_JMP(-1, PTM_B, JMP_NEQDEC, PTM_ZERO, 0),    /* 2: back to 1 while B-- != 0 */
    PTM_WAIT(0, 0, 0),                              /* 3: end marker              */
};

static void ptm_run_loop(void)
{
    PTM_CoreCfgTypeDef cfg = {0};

    rt_kprintf("\n[2] loop with a hand written jump offset\n");

    ptm_core.core = 0U;
    if (HAL_PTM_CoreInit(&ptm_core) != HAL_OK)
    {
        rt_kprintf("    CoreInit failed: is that core already taken?\n");
        return;
    }

    cfg.psc  = 0U;
    cfg.rep  = 0U;
    cfg.end0 = 3U;                                  /* the end marker             */
    cfg.end1 = 0U;
    (void)HAL_PTM_CoreConfig(&ptm_core, &cfg);

    if (HAL_PTM_CoreLoadCode(&ptm_core, program_loop, 4U) != HAL_OK)
    {
        rt_kprintf("    CoreLoadCode failed\n");
        (void)HAL_PTM_CoreDeInit(&ptm_core);
        return;
    }

    (void)HAL_PTM_CoreStart(&ptm_core);

    if (HAL_PTM_CoreWaitDone(&ptm_core, 1000U) == HAL_OK)
    {
        rt_kprintf("    completed: X = %u after 5 jumps\n",
                   (unsigned)HAL_PTM_CoreGetReg(&ptm_core, PTM_REG_X));
    }
    else
    {
        rt_kprintf("    the task did not complete\n");
    }

    (void)HAL_PTM_CoreDeInit(&ptm_core);
}

int main(void)
{
    rt_kprintf("PTM example: programs built with the instruction macros\n");
    rt_kprintf("(no assembler, ptm_asm.h is not used)\n");

    ptm_run_straight();
    ptm_run_loop();

    rt_kprintf("\ndone\n");

    return 0;
}
