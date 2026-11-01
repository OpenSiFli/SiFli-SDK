/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BF0_HAL_PTM_H
#define __BF0_HAL_PTM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "bf0_hal_def.h"

#if defined(HAL_PTM_MODULE_ENABLED) || defined(_SIFLI_DOXYGEN_)
#include "ptm_ins.h"

/** @addtogroup BF0_HAL_Driver
 * @{
 */

/** @defgroup PTM PTM (Parallel Task Machine)
 * @brief  HAL for the SF32LB57x PTM: a 4 core, 32 bit programmable timing
 *         machine (see UM5701 chapter 18).
 *
 * Handle model:
 *   - one module handle per chip, owned by the HAL and returned by
 *     HAL_PTM_GetHandle().  It carries the clock/reset reference count, the
 *     core allocation bitmap and the global EVT/CH helpers.
 *   - one PTM_CoreHandleTypeDef per user of a core, declared by that user
 *     (static storage or inside its own device object).  It owns the CCR/ICR/
 *     OCR configuration and the callbacks of that core.
 *
 * A driver normally only calls:
 * @code
 *   static PTM_CoreHandleTypeDef hcore;
 *   hcore.core = 0;                 // or PTM_CORE_ANY
 *   HAL_PTM_CoreInit(&hcore);       // module ref count + 1, claims a core, resets it
 *   HAL_PTM_CoreLoadCode(&hcore, code, words);
 *   HAL_PTM_CoreConfig(&hcore, &cfg);
 *   HAL_PTM_CoreStart(&hcore);
 *   ...
 *   HAL_PTM_CoreDeInit(&hcore);
 * @endcode
 * HAL_PTM_Init()/HAL_PTM_DeInit() are called internally by CoreInit/CoreDeInit
 * and only need to be used directly when no core is taken (CH/EVT only).
 *
 * Thread safety: the module state shared by all the users (clock/reset
 * reference count, core allocation bitmap and the IER bits) is updated inside
 * short critical sections, so the HAL may be called from several threads of
 * the same CPU.  Two precautions stay on the caller side:
 *   - a core handle belongs to a single user; do not drive one handle from
 *     two threads without serialising the calls;
 *   - the critical sections only mask this CPU.  If two CPUs can drive PTM
 *     (for example HCPU and ACPU), an external lock is required.
 * @{
 */

/** @defgroup PTM_Exported_Constants PTM exported constants
 * @{
 */

/** Number of cores.  SF32LB57x implements 4 (CER.EN0~EN3, ISR.ERR0~ERR3),
 * can be overridden by the build. */
#ifndef PTM_CORE_NUM
#define PTM_CORE_NUM                4U
#endif

/** Passed as PTM_CoreHandleTypeDef.core to let the HAL pick a free core. */
#define PTM_CORE_ANY                0xFFU

/** TCM slots owned by one core: core N address 0 maps to TCM[16 * N]. */
#define PTM_TCM_WORDS_PER_CORE      16U

/** Total number of TCM instruction slots: one window per core (UM5701 18.1.8). */
#define PTM_TCM_WORDS               (PTM_TCM_WORDS_PER_CORE * PTM_CORE_NUM)

/** Number of IO channels CH0~CH15. */
#define PTM_IO_CH_NUM               16U

/** Number of event flags EVT0~EVT15 (global, shared by all cores). */
#define PTM_EVT_NUM                 16U

/** NVIC line of a core: interrupt routing is fixed to core n <-> EVTn. */
#define PTM_CORE_IRQn(core)         ((IRQn_Type)((uint32_t)PTM1_CORE0_IRQn + (uint32_t)(core)))

/** Status bits returned by HAL_PTM_CoreGetFifoStatus(). */
#define PTM_FIFO_FLAG_IFIFO_FULL    (1UL << 0)      /*!< ICR.IFULL  */
#define PTM_FIFO_FLAG_IFIFO_EMPTY   (1UL << 1)      /*!< ICR.IEMPTY */
#define PTM_FIFO_FLAG_OFIFO_FULL    (1UL << 2)      /*!< OCR.OFULL  */
#define PTM_FIFO_FLAG_OFIFO_EMPTY   (1UL << 3)      /*!< OCR.OEMPTY */
#define PTM_FIFO_FLAG_IFIFO_OVF     (1UL << 4)      /*!< ICR.IOF, clear with HAL_PTM_CoreClearFifoFlag() */
#define PTM_FIFO_FLAG_OFIFO_UNF     (1UL << 5)      /*!< OCR.OUF, clear with HAL_PTM_CoreClearFifoFlag() */

/** Error bits passed to PTM_ErrorCallback_t. */
#define PTM_ERR_CORE                (1UL << 0)      /*!< ISR.ERRx: bus error or illegal instruction */
#define PTM_ERR_TCM_CONFLICT        (1UL << 1)      /*!< ISR.TCMx: TCM collision */

/**
 * @}
 */

/**
 * @}
 */

/** @defgroup PTM_Exported_Types PTM exported types
 * @{
 */

/** @brief PTM state, used by handles. */
typedef enum
{
    HAL_PTM_STATE_RESET = 0x00U,    /*!< Not initialized            */
    HAL_PTM_STATE_READY = 0x01U,    /*!< Initialized, core usable   */
    HAL_PTM_STATE_BUSY  = 0x02U     /*!< Core claimed/running       */
} PTM_StateTypeDef;

/** @brief General purpose register of a core, see HAL_PTM_CoreSetReg(). */
typedef enum
{
    PTM_REG_X = 0,                  /*!< 32 bit X                    */
    PTM_REG_Y,                      /*!< 32 bit Y                    */
    PTM_REG_A,                      /*!< 16 bit A (low half used)    */
    PTM_REG_B,                      /*!< 16 bit B (low half used)    */
    PTM_REG_PC                      /*!< Program counter, read only  */
} PTM_RegTypeDef;

/** @brief Direction selector for the FIFO helpers. */
typedef enum
{
    PTM_FIFO_IFIFO = 0,             /*!< Input  FIFO (IO -> memory)  */
    PTM_FIFO_OFIFO                  /*!< Output FIFO (memory -> IO)  */
} PTM_FifoDirTypeDef;

/**
 * @brief Core configuration, written to CCRx (UM5701 18.1.5, 18.1.26).
 */
typedef struct
{
    uint8_t psc;                    /*!< Core clock = hclk / (psc + 1), 0..255   */
    uint8_t rep;                    /*!< 0: run once, 1..254: repeat, 255: forever */
    uint8_t end0;                   /*!< TCM instruction index ending the sequence:
                                         the instruction at that index runs, and
                                         the sequence ends right after it.
                                         0 means "no end check at this index"     */
    uint8_t end1;                   /*!< Second end index, for the other branch    */
} PTM_CoreCfgTypeDef;

/**
 * @brief Per core IO/FIFO configuration, written to ICRx and OCRx
 *        (UM5701 18.1.19 ~ 18.1.22).
 */
typedef struct
{
    /* Input path, ICRx (UM5701 18.1.19): IO channels -> Y -> IFIFO -> CPU/DMA */
    uint8_t ififo_en;               /*!< Enable IFIFO, depth 4 or 8              */
    uint8_t iauto;                  /*!< Y -> IFIFO automatic push               */
    uint8_t ibase;                  /*!< IN[n]  = CH[(n + ibase) % 16]           */
    uint8_t irsh;                   /*!< Input shift direction, 0: shift left    */
    uint8_t iblock;                 /*!< Stall the pipeline when IFIFO is full   */
    uint8_t ishift;                 /*!< Extra shift before pushing to IFIFO     */
    uint8_t ilimit;                 /*!< Accumulated input bit count that triggers
                                         the automatic push (ICRx.ILIMIT).  Note
                                         that 0 means 32, not "every sample":
                                         Y_CNT >= ilimit pushes Y.                */

    /* Output path, OCRx (UM5701 18.1.21): CPU/DMA -> OFIFO -> X -> IO channels */
    uint8_t ofifo_en;               /*!< Enable OFIFO, depth 4 or 8              */
    uint8_t oauto;                  /*!< OFIFO -> X automatic pop                */
    uint8_t obase;                  /*!< CH[(n + obase) % 16] = OUT[n]           */
    uint8_t orsh;                   /*!< Output shift direction, 0: shift left   */
    uint8_t oblock;                 /*!< Stall the pipeline when OFIFO is empty  */
    uint8_t oshift;                 /*!< Extra shift after popping from OFIFO    */
    uint8_t olimit;                 /*!< Output bit count that triggers the auto
                                         pop (OCRx.OLIMIT); 0 means 32, the same
                                         convention as ilimit.                    */
} PTM_IoFifoInitTypeDef;

/**
 * @brief Snapshot of one core, see HAL_PTM_CoreGetStatus().
 */
typedef struct
{
    uint32_t pc;                    /*!< PCx                                      */
    uint8_t  running;               /*!< CER.ENx read back                        */
    uint8_t  error;                 /*!< ISR.ERRx: bus error or illegal instruction */
    uint8_t  tcm_conflict;          /*!< ISR.TCMx                                 */
    uint8_t  reserved;
} PTM_CoreStatusTypeDef;

struct PTM_CoreHandleTypeDef;

/**
 * @brief Event callback, called from HAL_PTM_IRQHandler().
 *
 * The EVT flag has already been cleared by the HAL when the callback runs.
 * @param hcore    Core that raised the interrupt
 * @param evt_mask EVT bits, always the core's own EVTn
 */
typedef void (*PTM_CoreEventCallback_t)(struct PTM_CoreHandleTypeDef *hcore, uint16_t evt_mask);

/**
 * @brief Error callback, called from HAL_PTM_IRQHandler().
 *
 * ISR.ERRx / ISR.TCMx cannot be cleared by writing, only CER.RSTx clears them,
 * so the callback receives a snapshot and may call HAL_PTM_CoreReset().
 * @param hcore    Core that raised the error
 * @param err_mask PTM_ERR_CORE / PTM_ERR_TCM_CONFLICT
 */
typedef void (*PTM_ErrorCallback_t)(struct PTM_CoreHandleTypeDef *hcore, uint32_t err_mask);

/**
 * @brief Module handle, single instance owned by the HAL.
 */
typedef struct PTM_HandleTypeDef
{
    PTM_TypeDef      *Instance;     /*!< PTM register base (= hwp_ptm1)          */
    PTM_StateTypeDef  State;        /*!< Module state                            */
    uint8_t           core_bitmap;  /*!< Claimed cores, bit n = core n           */
    uint8_t           init_cnt;     /*!< Reference count of HAL_PTM_Init()       */
} PTM_HandleTypeDef;

/**
 * @brief Core handle, one per user of a core.
 *
 * Must have static storage duration (or live inside a device object): the HAL
 * keeps a pointer to it for interrupt dispatch.  A function local variable is
 * not allowed.
 */
typedef struct PTM_CoreHandleTypeDef
{
    PTM_HandleTypeDef      *hptm;           /*!< Filled by HAL_PTM_CoreInit()    */
    uint32_t                core;           /*!< Core index, or PTM_CORE_ANY     */
    PTM_CoreCfgTypeDef      Init;           /*!< Last CCRx configuration         */
    PTM_IoFifoInitTypeDef   IoInit;         /*!< Last ICRx/OCR configuration     */
    PTM_CoreEventCallback_t EventCallback;  /*!< May be NULL                     */
    PTM_ErrorCallback_t     ErrorCallback;  /*!< May be NULL                     */
    PTM_StateTypeDef        State;          /*!< Core state                      */
    uint8_t                 io_configured;  /*!< IoInit has been written         */
} PTM_CoreHandleTypeDef;

/**
 * @}
 */

/** @defgroup PTM_Exported_Functions PTM exported functions
 * @{
 */

/** @defgroup PTM_Exported_Functions_Module Module level
 * @{
 */

/**
 * @brief  Get the single PTM module handle.
 * @retval Module handle, never NULL
 */
PTM_HandleTypeDef *HAL_PTM_GetHandle(void);

/**
 * @brief  Initialize the PTM module (reference counted).
 *
 * The first caller enables the clock, resets the module and clears IER/ISR.
 * Later callers only increase the reference count, so they never disturb a
 * core that is already running.  Normally called by HAL_PTM_CoreInit().
 * @param  hptm Module handle
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_Init(PTM_HandleTypeDef *hptm);

/**
 * @brief  Release the PTM module.
 *
 * When the reference count reaches zero, every core is reset and the clock is
 * disabled.  Normally called by HAL_PTM_CoreDeInit().
 * @param  hptm Module handle
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_DeInit(PTM_HandleTypeDef *hptm);

/**
 * @brief  Reset the whole PTM module.
 *
 * Stops every core, clears all their state and drops every claim and TCM record:
 * the handles that were in use have to be released with HAL_PTM_CoreDeInit(), or
 * re-initialized afterwards, which keeps the module reference count balanced.
 * It must only be used when no other user of the module is active.
 * @param  hptm Module handle
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_ForceReset(PTM_HandleTypeDef *hptm);

/**
 * @brief  Claim a core.
 * @param  hptm Module handle
 * @param  core Core index 0..PTM_CORE_NUM-1, or PTM_CORE_ANY to pick a free one
 * @retval Core index, or -1 if the requested core is busy or out of range
 */
int32_t HAL_PTM_CoreAlloc(PTM_HandleTypeDef *hptm, uint32_t core);

/**
 * @brief  Release a previously claimed core.
 *
 * Drops its claim, its TCM record and its reporting: the interrupts of that core
 * (IER bits and NVIC line) are switched off and it is removed from the dispatch
 * table, so no callback is called for it afterwards.
 * @param  hptm Module handle
 * @param  core Core index
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_CoreFree(PTM_HandleTypeDef *hptm, uint32_t core);

/**
 * @brief  Handle a core interrupt.
 *
 * To be called from the matching PTM1_COREn_IRQHandler().  It clears the EVT
 * flag of that core (write 1 clear) before invoking the core callbacks, and
 * never touches the other cores.
 * @param  core Core index
 */
void HAL_PTM_IRQHandler(uint32_t core);

/**
 * @brief  Read the global event flags.
 * @param  hptm Module handle
 * @retval Bitmap of EVT0~EVT15
 */
uint16_t HAL_PTM_GetEvent(PTM_HandleTypeDef *hptm);

/**
 * @brief  Clear global event flags.
 *
 * The CPU can only clear events, setting them is done by PTM instructions.
 * @param  hptm    Module handle
 * @param  evt_mask EVT bits to clear
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_ClearEvent(PTM_HandleTypeDef *hptm, uint16_t evt_mask);

/**
 * @brief  Drive the output enable of IO channels from the CPU (DOESR/DOECR).
 * @param  hptm     Module handle
 * @param  ch_mask  Channels to update
 * @param  en_mask  Channels of ch_mask to enable, the others are disabled
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_SetOutputEnable(PTM_HandleTypeDef *hptm, uint16_t ch_mask, uint16_t en_mask);

/**
 * @brief  Read back the IO channel output enable (DOER).
 * @param  hptm Module handle
 * @retval Bitmap of the 16 channels
 */
uint16_t HAL_PTM_GetOutputEnable(PTM_HandleTypeDef *hptm);

/**
 * @brief  Drive IO channel outputs from the CPU (DOSR/DOCR).
 * @param  hptm      Module handle
 * @param  ch_mask   Channels to update
 * @param  high_mask Channels of ch_mask driven high, the others low
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_SetOutput(PTM_HandleTypeDef *hptm, uint16_t ch_mask, uint16_t high_mask);

/**
 * @brief  Read back the IO channel outputs (DOR).
 * @param  hptm Module handle
 * @retval Bitmap of the 16 channels
 */
uint16_t HAL_PTM_GetOutput(PTM_HandleTypeDef *hptm);

/**
 * @brief  Read the TCM collision flags (ISR.TCMx).
 *
 * A core whose program exceeds 16 instructions overruns into the next core's
 * address 0, which raises this flag (UM5701 18.1.8).  Cleared by CER.RSTx.
 * @param  hptm Module handle
 * @retval Bitmap, bit n = core n collided
 */
uint32_t HAL_PTM_GetTcmConflict(PTM_HandleTypeDef *hptm);

/**
 * @}
 */

/** @defgroup PTM_Exported_Functions_Core Core level
 * @{
 */

/**
 * @brief  Claim and reset a core, and increase the module reference count.
 *
 * Must be called on a handle that is not claimed yet: an already claimed handle
 * returns HAL_ERROR (release it with HAL_PTM_CoreDeInit() first, otherwise the
 * core and the reference count of the previous call would leak).
 * @param  hcore Core handle with @c core preset (index or PTM_CORE_ANY)
 * @retval HAL_OK if successful, HAL_BUSY if the requested core is taken,
 *         HAL_ERROR if the handle is already claimed or @c core is out of range
 */
HAL_StatusTypeDef HAL_PTM_CoreInit(PTM_CoreHandleTypeDef *hcore);

/**
 * @brief  Stop and reset a core, release it and decrease the module reference
 *         count.  The handle memory must stay valid until this call returns.
 * @param  hcore Core handle
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_CoreDeInit(PTM_CoreHandleTypeDef *hcore);

/**
 * @brief  Reset a core: clears its registers, state and FIFO, not the TCM.
 *
 * The configuration previously applied through this handle (HAL_PTM_CoreConfig()
 * and HAL_PTM_CoreConfigIo()) is written back after the reset, so a stopped
 * core can simply be started again.  The general purpose registers, the FIFOs
 * and the error flags are not restored, they come back cleared.  The
 * polling/interrupt mode is re-applied by HAL_PTM_CoreStart()/HAL_PTM_CoreStart_IT().
 * @param  hcore Core handle
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_CoreReset(PTM_CoreHandleTypeDef *hcore);

/**
 * @brief  Start a core in polling mode.
 *
 * The completion event is enabled (IER.EVTAUTOx) so that
 * HAL_PTM_CoreWaitDone() and HAL_PTM_CoreWaitEvent() can observe the end of
 * the task, any interrupt of that core is switched off, and the event flag of
 * the core is cleared so that a completion left over by a previous run cannot
 * be mistaken for this one.  This is the counterpart of
 * HAL_PTM_CoreStart_IT().
 * @param  hcore Core handle
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_CoreStart(PTM_CoreHandleTypeDef *hcore);

/**
 * @brief  Start a core in interrupt mode.
 *
 * Enables the completion and error interrupts of the core (IER.EVTAUTOx |
 * IER.EVTIE<x> | IER.ERRIE<x>) together with its NVIC line, so the callbacks
 * registered with HAL_PTM_CoreRegisterCallback() are called when the task
 * completes, when the program raises an event, or when the core reports an
 * error.  The event flag of the core is cleared first, which also avoids an
 * interrupt from a completion left over by a previous run.  This is the
 * counterpart of HAL_PTM_CoreStart().
 * @param  hcore Core handle
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_CoreStart_IT(PTM_CoreHandleTypeDef *hcore);

/**
 * @brief  Stop a core.
 * @param  hcore Core handle
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_CoreStop(PTM_CoreHandleTypeDef *hcore);

/**
 * @brief  Enable several cores with a single CER write, each one in its own mode.
 *
 * All the cores of @c core_mask are enabled by the same register write, i.e. in
 * the same hclk cycle, which is the closest thing to a simultaneous start.  The
 * mode is chosen per core: the ones also set in @c it_mask behave like
 * HAL_PTM_CoreStart_IT() (completion, event and error interrupts), the others
 * like HAL_PTM_CoreStart() (completion event only, no interrupt).  A helper core
 * that nobody waits for can therefore be started together with the master
 * without generating interrupts.
 *
 * @code
 *   HAL_PTM_CoreStartMask(hptm, (1U << 0) | (1U << 1), (1U << 0));
 *   // core0 and core1 start in the same cycle, only core0 interrupts the CPU
 * @endcode
 *
 * @param  hptm      Module handle
 * @param  core_mask Bit n enables core n
 * @param  it_mask   Bit n: core n reports through interrupts.  Bits outside
 *                   core_mask are ignored
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_CoreStartMask(PTM_HandleTypeDef *hptm, uint32_t core_mask, uint32_t it_mask);

/**
 * @brief  Stop several cores with a single CER write.
 * @param  hptm      Module handle
 * @param  core_mask Bit n stops core n
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_CoreStopMask(PTM_HandleTypeDef *hptm, uint32_t core_mask);

/**
 * @brief  Single step a stopped core.
 *
 * If the stepped instruction stalls the pipeline (WAIT, blocked FIFO...), the
 * step does not complete until the condition disappears (UM5701 18.1.25).
 * @param  hcore Core handle
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_CoreStep(PTM_CoreHandleTypeDef *hcore);

/**
 * @brief  Check whether a core is running (CER.ENx read back).
 * @param  hcore Core handle
 * @retval 1 if running, 0 if stopped
 */
uint32_t HAL_PTM_CoreIsRunning(PTM_CoreHandleTypeDef *hcore);

/**
 * @brief  Configure CCRx: clock divider, repetition and end addresses.
 * @param  hcore Core handle
 * @param  cfg   Configuration to apply
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_CoreConfig(PTM_CoreHandleTypeDef *hcore, const PTM_CoreCfgTypeDef *cfg);

/**
 * @brief  Configure ICRx/OCR: FIFO, IO mapping and shift settings.
 * @param  hcore Core handle
 * @param  cfg   Configuration to apply
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_CoreConfigIo(PTM_CoreHandleTypeDef *hcore, const PTM_IoFifoInitTypeDef *cfg);

/**
 * @brief  Load a program into the TCM window of a core.
 *
 * The TCM mapping of UM5701 18.1.8 is applied: core N address 0 is TCM[16*N].
 * A program longer than PTM_TCM_WORDS_PER_CORE instructions overruns into the
 * address 0 of the following cores; the HAL records which core owns which TCM
 * slot and refuses (HAL_ERROR, nothing written) any load that would overwrite
 * the program of another core.  That covers both orders: loading over the
 * window of an already claimed core, and claiming a core whose window was
 * already overrun by a longer program loaded earlier.
 * @param  hcore Core handle
 * @param  code  Instruction words, usually built with ptm_ins.h or ptm_asm.h
 * @param  words Number of words, 1..PTM_TCM_WORDS
 * @retval HAL_OK if successful, HAL_ERROR if another core owns part of the
 *         range, HAL_BUSY if the core is running (load the program before
 *         starting the core)
 */
HAL_StatusTypeDef HAL_PTM_CoreLoadCode(PTM_CoreHandleTypeDef *hcore, const uint32_t *code, uint32_t words);

/**
 * @brief  Get the TCM window of a core, i.e. the address its PC=0 maps to.
 *
 * Advanced use only: writing through this pointer bypasses the ownership
 * tracking of HAL_PTM_CoreLoadCode(), so the overrun protection of the TCM is
 * lost for that program.  It also does not wrap: a program longer than
 * PTM_TCM_WORDS_PER_CORE instructions has to be loaded with
 * HAL_PTM_CoreLoadCode(), which applies the TCM mapping of UM5701 18.1.8.
 * @param  hcore Core handle
 * @retval Pointer into TCM, word aligned
 */
uint32_t *HAL_PTM_CoreGetTcmPtr(PTM_CoreHandleTypeDef *hcore);

/**
 * @brief  Read or write a general purpose register of a core.
 * @param  hcore Core handle
 * @param  reg   Register selector
 * @param  value Value to write (set only)
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_CoreSetReg(PTM_CoreHandleTypeDef *hcore, PTM_RegTypeDef reg, uint32_t value);

/**
 * @brief  Read a general purpose register of a core.
 * @param  hcore Core handle
 * @param  reg   Register selector, PTM_REG_PC reads PCx
 * @retval Register value, 0 if the handle is invalid
 */
uint32_t HAL_PTM_CoreGetReg(PTM_CoreHandleTypeDef *hcore, PTM_RegTypeDef reg);

/**
 * @brief  Push one word into the output FIFO (OFIFOx).
 *
 * Like a UART transmit FIFO, this function does not look at the FIFO status:
 * the caller checks that there is room first with HAL_PTM_CoreGetFifoStatus()
 * (PTM_FIFO_FLAG_OFIFO_FULL clear in the returned bitmap).  Writing to a full
 * FIFO has no behaviour defined by the manual and must be avoided.
 *
 * A FIFO belongs to the task engine of a core, so it only takes data once that
 * core is enabled by HAL_PTM_CoreStart() / HAL_PTM_CoreStart_IT(): a core that
 * was only initialized is still held by the CER.RSTx written in
 * HAL_PTM_CoreReset() and silently drops the writes.
 * @param  hcore Core handle
 * @param  data  Word to push
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_CorePushOFifo(PTM_CoreHandleTypeDef *hcore, uint32_t data);

/**
 * @brief  Pop one word from the input FIFO (IFIFOx).
 *
 * Same convention as HAL_PTM_CorePushOFifo(): the FIFO status is not checked
 * here, the caller confirms that data is available first with
 * HAL_PTM_CoreGetFifoStatus() (PTM_FIFO_FLAG_IFIFO_EMPTY clear).  Reading an
 * empty FIFO has no behaviour defined by the manual and must be avoided.
 * @param  hcore Core handle
 * @param  data  Destination of the popped word
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_CorePopIFifo(PTM_CoreHandleTypeDef *hcore, uint32_t *data);

/**
 * @brief  Read the FIFO status of a core.
 * @param  hcore Core handle
 * @retval Bitmap of PTM_FIFO_FLAG_xxx
 */
uint32_t HAL_PTM_CoreGetFifoStatus(PTM_CoreHandleTypeDef *hcore);

/**
 * @brief  Clear the FIFO overflow/underflow flags.
 *
 * Requires HAL_PTM_CoreConfigIo() to have been called: the configuration is
 * rewritten from the shadow copy, because ICR/OCR configuration bits are not
 * preserved by a write 1 clear of IOF/OUF.
 * @param  hcore     Core handle
 * @param  flag_mask PTM_FIFO_FLAG_IFIFO_OVF / PTM_FIFO_FLAG_OFIFO_UNF
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_CoreClearFifoFlag(PTM_CoreHandleTypeDef *hcore, uint32_t flag_mask);

/**
 * @brief  Get the FIFO register address, to be used as the peripheral address
 *         of a DMA transfer (the DMA request numbers are PTM1_IFIFOn_DMA_REQUEST
 *         and PTM1_OFIFOn_DMA_REQUEST, see the board dma_config.h).
 * @param  hcore Core handle
 * @param  dir   FIFO direction
 * @retval Register address, 0 if the handle is invalid
 */
uint32_t HAL_PTM_CoreGetFifoAddr(PTM_CoreHandleTypeDef *hcore, PTM_FifoDirTypeDef dir);

/**
 * @brief  Read the status of a core.
 * @param  hcore  Core handle
 * @param  status Destination structure
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_CoreGetStatus(PTM_CoreHandleTypeDef *hcore, PTM_CoreStatusTypeDef *status);

/**
 * @brief  Register the callbacks of a core.
 *
 * Only the handle itself is checked, so the callbacks may be registered before
 * HAL_PTM_CoreInit(): HAL_PTM_CoreInit() keeps them.
 *
 * The event callback runs after the HAL has cleared the event flag of the core.
 * The error callback runs after the HAL has masked the error interrupt of that
 * core (the ERRx/TCMx flags are level and can only be cleared by CER.RSTx), so
 * an error callback that wants to continue has to call
 * HAL_PTM_CoreReset() and start the core again, which re-arms the interrupt.
 * @param  hcore Core handle
 * @param  evt_cb Event callback, may be NULL
 * @param  err_cb Error callback, may be NULL
 * @retval HAL_OK if successful, otherwise error
 */
HAL_StatusTypeDef HAL_PTM_CoreRegisterCallback(PTM_CoreHandleTypeDef *hcore,
        PTM_CoreEventCallback_t evt_cb, PTM_ErrorCallback_t err_cb);

/**
 * @brief  Polling wait for the event of a core, without using interrupts.
 *
 * Reads the global EVT flags until the own EVTn of the core is set, consumes it
 * (same semantics as HAL_PTM_IRQHandler()) and returns.  Blocking call, to be
 * used from thread context; in interrupt mode use the callbacks instead.
 * @param  hcore      Core handle
 * @param  timeout_ms Timeout in ms, HAL_MAX_DELAY to wait indefinitely
 * @retval HAL_OK when the event was consumed, HAL_TIMEOUT on timeout
 */
HAL_StatusTypeDef HAL_PTM_CoreWaitEvent(PTM_CoreHandleTypeDef *hcore, uint32_t timeout_ms);

/**
 * @brief  Polling wait for the end of the task, without using interrupts.
 *
 * Waits on the flags the hardware uses to report the run, exactly like the
 * interrupt mode does: EVTn is set when the task completes (IER.EVTAUTOx is
 * enabled by HAL_PTM_CoreStart()/HAL_PTM_CoreStart_IT()) and ISR.ERRx/TCMx are
 * set when the run is aborted.  The event is left pending, so
 * HAL_PTM_CoreWaitEvent() or HAL_PTM_GetEvent() still observe it afterwards.
 * Blocking call, thread context only.
 *
 * @code
 *   switch (HAL_PTM_CoreWaitDone(&hcore, 1000))
 *   {
 *   case HAL_OK:      break;      // the task completed
 *   case HAL_ERROR:   break;      // aborted: bus/instruction error or TCM collision
 *   case HAL_TIMEOUT: break;      // not finished: still running, or stopped on purpose
 *   default:          break;
 *   }
 * @endcode
 *
 * @param  hcore      Core handle
 * @param  timeout_ms Timeout in ms, HAL_MAX_DELAY to wait indefinitely
 * @retval HAL_OK: the task completed.  HAL_ERROR: the run was aborted, the
 *         details are in HAL_PTM_CoreGetStatus().  HAL_TIMEOUT: neither a
 *         completion nor an error before the timeout, for example a program
 *         that never ends or a core stopped with HAL_PTM_CoreStop().
 */
HAL_StatusTypeDef HAL_PTM_CoreWaitDone(PTM_CoreHandleTypeDef *hcore, uint32_t timeout_ms);

/**
 * @}
 */

/** @defgroup PTM_Exported_Functions_Callback Default callbacks
 * @{
 */

/**
 * @brief  Default event callback, does nothing.
 * @param  hcore    Core handle
 * @param  evt_mask Event bits
 */
void HAL_PTM_CoreEventCallback(PTM_CoreHandleTypeDef *hcore, uint16_t evt_mask);

/**
 * @brief  Default error callback, does nothing.
 * @param  hcore    Core handle
 * @param  err_mask Error bits
 */
void HAL_PTM_ErrorCallback(PTM_CoreHandleTypeDef *hcore, uint32_t err_mask);

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

#endif /* PTM1_BASE */

#ifdef __cplusplus
}
#endif

#endif /* __BF0_HAL_PTM_H */

/// @}
