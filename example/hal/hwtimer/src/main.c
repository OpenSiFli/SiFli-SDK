#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "string.h"
/* user start */
#include "drv_flash.h"
/* user end */


/* Common functions for RT-Thread based platform -----------------------------------------------*/

/* User code start from here --------------------------------------------------------*/
/*******************************************************************************
 ** Macro
 *******************************************************************************/
/* TIM FREQ */
#define FREQENCY 10000

#define CURRENT_TIME() (rt_tick_get())
#define COST_TIME(TIME) (rt_tick_get()  - TIME)

/*******************************************************************************
 ** Variable
 *******************************************************************************/
static rt_sem_t g_hwtim_sem;
static GPT_HandleTypeDef TIM_Handle = {0};
static uint32_t ticks_start = 0;

/*******************************************************************************
 ** Function
 *******************************************************************************/
/**
 * @brief Common initialization.
 */
static rt_err_t example_comm_init(void)
{
    g_hwtim_sem = rt_sem_create("hwtim_sem", 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_hwtim_sem);

    return RT_EOK;
}

void BTIM1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_kprintf("BTIM1 timeout %d\n", COST_TIME(ticks_start));
    HAL_GPT_IRQHandler(&TIM_Handle);
    rt_sem_release(g_hwtim_sem);

    /* leave interrupt */
    rt_interrupt_leave();
}

void GPTIM1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_kprintf("GPTIM1 timeout %d\n", COST_TIME(ticks_start));
    HAL_GPT_IRQHandler(&TIM_Handle);
    rt_sem_release(g_hwtim_sem);

    /* leave interrupt */
    rt_interrupt_leave();
}

void ATIM1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    rt_kprintf("ATIM1 timeout %d\n", COST_TIME(ticks_start));
    HAL_GPT_IRQHandler(&TIM_Handle);
    rt_sem_release(g_hwtim_sem);

    /* leave interrupt */
    rt_interrupt_leave();
}



/**
  * @brief  HWTIM Initialization.
  * @param  tim                  HWTIM handle.
  * @param  instance             HWTIM instance.
  * @param  irq                  HWTIM irq.
  * @param  period               HWTIM timeout period.
  * @param  op_mode              GPT_OPMODE_SINGLE or GPT_OPMODE_REPETITIVE.
  * @param  repetition_counter   HWTIM repetion counter. Only atim1 and atim2 support.
  * @param  core_id              HWTIM core id.

  * @retval HAL_OK if success, otherwise failure number
  */
static HAL_StatusTypeDef example_hwtimer_init(GPT_HandleTypeDef *tim,
        GPT_TypeDef *instance,
        IRQn_Type irq,
        uint32_t period,
        uint32_t op_mode,
        uint32_t repetition_counter,
        int core_id)
{
    HAL_StatusTypeDef r = HAL_OK;

    tim->Instance = instance;
    tim->Init.Prescaler = HAL_RCC_GetPCLKFreq(core_id, 1) / FREQENCY; /*Prescaler is 16 bits, please select correct frequency*/
    tim->core = core_id;
    tim->Init.CounterMode = GPT_COUNTERMODE_UP;         /*GPTIM could support counter up/down, BTIM only support count up*/
    tim->Init.RepetitionCounter = repetition_counter;
    tim->Init.Period = period;
    rt_kprintf("Prescaler:%d PCLK:%d period:%d\n", tim->Init.Prescaler, HAL_RCC_GetPCLKFreq(core_id, 1), period);

    if (HAL_GPT_Base_Init(tim) == HAL_OK)
    {
        rt_kprintf("Timer init ok\n");
        HAL_NVIC_SetPriority(irq, 3, 0);                /* set the TIMx priority */
        HAL_NVIC_EnableIRQ(irq);                        /* enable the TIMx global Interrupt */
        __HAL_GPT_CLEAR_FLAG(tim, GPT_FLAG_UPDATE);     /* clear update flag */
        __HAL_GPT_URS_ENABLE(tim);                      /* enable update request source */
        __HAL_GPT_SET_MODE(tim, op_mode);               /* pretive or single */
    }
    else
    {
        rt_kprintf("Timer init error\n");
        r = HAL_ERROR;
    }

    return HAL_OK;
}

/**
  * @brief  HWTIM Start.
  * @param  tim      HWTIM handle.
  * @retval HAL_OK if success, otherwise failure number
  */
static HAL_StatusTypeDef example_hwtimer_start_IT(GPT_HandleTypeDef *tim)
{
    HAL_StatusTypeDef result = HAL_OK;

    if (HAL_GPT_Base_Start_IT(tim) != HAL_OK)           /* start timer */
    {
        rt_kprintf("Timer start error\n");
        result = HAL_ERROR;
    }
    return result;
}


/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    rt_err_t err = RT_EOK;
    uint32_t count = 0;

    rt_kprintf("\nHWTIM Example:\n");

    /* Common initialization. */
    example_comm_init();

#ifdef hwp_gptim1
    rt_kprintf("GPTIM1: SINGLE. Timeout is 3.5 seconds\n");
    example_hwtimer_init(&TIM_Handle, hwp_gptim1, GPTIM1_IRQn, 3500 * FREQENCY / 1000,  GPT_OPMODE_SINGLE, 0, CORE_ID_HCPU);
    example_hwtimer_start_IT(&TIM_Handle);
    /* Wait HWTIMER expired. */
    ticks_start = CURRENT_TIME();
    err = rt_sem_take(g_hwtim_sem, RT_WAITING_FOREVER);
    RT_ASSERT(RT_EOK == err);
    /* HWTIMER deinit.*/
    HAL_GPT_Base_DeInit(&TIM_Handle);
    rt_kprintf("GPTIM1: SINGLE. END.\n");

    rt_kprintf("\nGPTIM1: REPETITIVE. Timeout is 3.5 seconds * 10 times.\n");
    /* HWTIMER config and start. */
    example_hwtimer_init(&TIM_Handle, hwp_gptim1, GPTIM1_IRQn, 3500 * FREQENCY / 1000,  GPT_OPMODE_REPETITIVE, 0, CORE_ID_HCPU);
    example_hwtimer_start_IT(&TIM_Handle);
    /* 10 times. */
    while (count < 10)
    {
        ticks_start = CURRENT_TIME();
        err = rt_sem_take(g_hwtim_sem, RT_WAITING_FOREVER);
        RT_ASSERT(RT_EOK == err);
        count ++;
    }
    /* HWTIMER deinit.*/
    HAL_GPT_Base_DeInit(&TIM_Handle);
    rt_kprintf("GPTIM1: REPETITIVE. END.\n");
#endif

#ifdef hwp_atim1
    printf("\nATIM1: SINGLE. Timeout is 3.5 seconds\n");
    example_hwtimer_init(&TIM_Handle, (GPT_TypeDef *)hwp_atim1, ATIM1_IRQn, 3500 * FREQENCY / 1000,  GPT_OPMODE_SINGLE, 0, CORE_ID_HCPU);
    example_hwtimer_start_IT(&TIM_Handle);
    /* Wait HWTIMER expired. */
    ticks_start = CURRENT_TIME();
    err = rt_sem_take(g_hwtim_sem, RT_WAITING_FOREVER);
    RT_ASSERT(RT_EOK == err);
    /* HWTIMER deinit.*/
    HAL_GPT_Base_DeInit(&TIM_Handle);
    printf("ATIM1: SINGLE. END.\n");

    printf("\nATIM1: REPETITIVE. Timeout is (3.5 * 10) seconds.\n");
    /* HWTIMER config and start. */
    example_hwtimer_init(&TIM_Handle, (GPT_TypeDef *)hwp_atim1, ATIM1_IRQn, 3500 * FREQENCY / 1000, GPT_OPMODE_REPETITIVE, 10, CORE_ID_HCPU);
    example_hwtimer_start_IT(&TIM_Handle);
    /* Wait HWTIMER expired. */
    ticks_start = CURRENT_TIME();
    err = rt_sem_take(g_hwtim_sem, RT_WAITING_FOREVER);
    RT_ASSERT(RT_EOK == err);
    /* HWTIMER deinit.*/
    HAL_GPT_Base_DeInit(&TIM_Handle);
    printf("ATIM1: END.\n");
#endif

#ifdef hwp_btim1
    printf("\nBTIM1: SINGLE. Timeout is 3.5 seconds\n");
    /* HWTIMER config and start. */
    HAL_RCC_EnableModule(RCC_MOD_BTIM1); /* Enable btim1 rcc */
    example_hwtimer_init(&TIM_Handle, (GPT_TypeDef *)hwp_btim1, BTIM1_IRQn, 3500 * FREQENCY / 1000, GPT_OPMODE_SINGLE, 0, CORE_ID_HCPU);
    example_hwtimer_start_IT(&TIM_Handle);
    /* Wait HWTIMER expired. */
    ticks_start = CURRENT_TIME();
    err = rt_sem_take(g_hwtim_sem, RT_WAITING_FOREVER);
    RT_ASSERT(RT_EOK == err);
    /* HWTIMER deinit.*/
    HAL_GPT_Base_DeInit(&TIM_Handle);
    printf("BTIM1: SINGLE. END.\n");

    printf("\nBTIM1: REPETITIVE. Timeout is 3.5 seconds * 10 times.\n");
    /* HWTIMER config and start. */
    example_hwtimer_init(&TIM_Handle, (GPT_TypeDef *)hwp_btim1, BTIM1_IRQn, 3500 * FREQENCY / 1000,  GPT_OPMODE_REPETITIVE, 0, CORE_ID_HCPU);
    example_hwtimer_start_IT(&TIM_Handle);
    /* 10 times. */
    count = 0;
    while (count < 10)
    {
        ticks_start = CURRENT_TIME();
        err = rt_sem_take(g_hwtim_sem, RT_WAITING_FOREVER);
        RT_ASSERT(RT_EOK == err);
        count ++;
    }
    /* HWTIMER deinit.*/
    HAL_GPT_Base_DeInit(&TIM_Handle);
    printf("BTIM1: REPETITIVE. END.\n");
#endif

    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(10000);
    }
    return 0;
}

