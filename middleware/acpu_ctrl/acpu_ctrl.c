/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtconfig.h>
#include <board.h>
#ifndef SOC_BF0_ACPU
    #include "rtthread.h"
#endif /* SOC_BF0_ACPU */
#include "ipc_queue.h"
#include <string.h>
#include "acpu_ctrl.h"
#if defined(ANYKA_RUN_IN_ACPU) && defined(SOC_BF0_ACPU)
    #include "audio_3a_anyka.h"
#endif

//#ifndef SOC_BF0_ACPU
//#define LOG_TAG      "mw.acpu"
//#include "log.h"
//#endif

#define ACPU_TASK_DONE_NTF_QUEUE (8)

#define ACPU_CALLER_ENABLED 1
typedef struct
{
    volatile uint8_t task_name;  /**< task name, @ref #acpu_task_name_t */
    volatile uint8_t seq_no;     /**< sequence number */
    volatile uint8_t reserved[2];
    volatile uint8_t param[ACPU_TASK_INPUT_PARAM_SIZE];
} acpu_task_input_t;

/** ACPU Task input buf size, must be consistent with acpu_task_input_t */
#define ACPU_TASK_INPUT_BUF_SIZE  (ACPU_TASK_INPUT_PARAM_SIZE + 4)

typedef struct
{
    volatile uint8_t task_name;   /**< task name, @ref #acpu_task_name_t */
    volatile uint8_t seq_no;
    volatile uint8_t error_code;
    volatile uint8_t reserved;
    volatile uint32_t ret;
    volatile uint8_t val[ACPU_TASK_OUTPUT_VAL_SIZE];
} acpu_task_output_t;

/** ACPU Task output buf size, must be consistent with acpu_task_output_t */
#define ACPU_TASK_OUTPUT_BUF_SIZE  (ACPU_TASK_OUTPUT_VAL_SIZE + 8)

typedef struct
{
    volatile uint8_t task_name;   /**< task name, @ref #hcpu_task_name_t */
    volatile uint8_t reserved[3];
    volatile uint32_t input;
    volatile uint8_t val[HCPU_TASK_INPUT_PARAM_SIZE];
} hcpu_task_input_t;

#define HCPU_TASK_INPUT_BUF_SIZE  (HCPU_TASK_INPUT_PARAM_SIZE + 8)

typedef struct
{
    volatile uint8_t task_name;   /**< task name, @ref #hcpu_task_name_t */
    volatile uint8_t seq_no;
    volatile uint8_t error_code;
    volatile uint8_t reserved;
    volatile uint32_t ret;
    volatile uint8_t val[HCPU_TASK_OUTPUT_VAL_SIZE];
} hcpu_task_output_t;

#define HCPU_TASK_OUTPUT_BUF_SIZE  (HCPU_TASK_OUTPUT_VAL_SIZE + 8)

/*cmd buffer memory layout*/
#define ACPU_INPUT_START      (ACPU_CMD_BUF_START_ADDR)
#define ACPU_INPUT_END        (ACPU_INPUT_START + ACPU_TASK_INPUT_BUF_SIZE)
#define ACPU_OUTPUT_START     (ACPU_INPUT_END)
#define ACPU_OUTPUT_END       (ACPU_OUTPUT_START + ACPU_TASK_OUTPUT_BUF_SIZE)
#define HCPU_INPUT_START      (ACPU_OUTPUT_END)
#define HCPU_INPUT_END        (HCPU_INPUT_START + HCPU_TASK_INPUT_BUF_SIZE)
#define HCPU_OUTPUT_START     (HCPU_INPUT_END)
#define HCPU_OUTPUT_END       (HCPU_OUTPUT_START + HCPU_TASK_OUTPUT_BUF_SIZE)

#define CMD_LAST_END          HCPU_OUTPUT_END

#if ACPU_CMD_BUF_SIZE < (CMD_LAST_END - ACPU_CMD_BUF_START_ADDR)
    #error "ACPU_CMD_BUF_SIZE too small"
#endif

static acpu_task_input_t *acpu_task_input = (acpu_task_input_t *)ACPU_INPUT_START;
static acpu_task_output_t *acpu_task_output = (acpu_task_output_t *)ACPU_OUTPUT_START;
static hcpu_task_input_t *hcpu_task_input = (hcpu_task_input_t *)HCPU_INPUT_START;
static hcpu_task_output_t *hcpu_task_output = (hcpu_task_output_t *)HCPU_OUTPUT_START;

#ifndef SOC_BF0_ACPU
    static ipc_queue_handle_t acpu_task_ntf_queue = IPC_QUEUE_INVALID_HANDLE;
    static struct rt_semaphore acpu_task_done_sema;
    static struct rt_mutex     acpu_task_mutex;
#endif /* SOC_BF0_ACPU */

#ifdef SOC_BF0_ACPU
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <alloca.h>
#ifdef PKG_LIB_OPUS
    #include "opus.h"
#endif

void *acpu_call_hcpu_malloc(uint32_t size)
{
    hcpu_task_input->input = size;
    hcpu_task_input->task_name = HCPU_TASK_MALLOC;

    acpu_task_output->error_code = ACPU_ERR_CALL_HCPU;
    MAILBOX_HandleTypeDef handle;
    handle.Instance = A2H_MAILBOX;
    __HAL_MAILBOX_TRIGGER_CHANNEL_IT(&handle, (ACPU_TASK_DONE_NTF_QUEUE % IPC_HW_QUEUE_NUM));

    while (acpu_task_output->error_code);

    return (void *)hcpu_task_output->ret;
}
void acpu_call_hcpu_free(void *p)
{
    hcpu_task_input->input = (uint32_t)p;
    hcpu_task_input->task_name = HCPU_TASK_FREE;

    acpu_task_output->error_code = ACPU_ERR_CALL_HCPU;

    MAILBOX_HandleTypeDef handle;
    handle.Instance = A2H_MAILBOX;
    __HAL_MAILBOX_TRIGGER_CHANNEL_IT(&handle, (ACPU_TASK_DONE_NTF_QUEUE % IPC_HW_QUEUE_NUM));

    while (acpu_task_output->error_code);
}

void acpu_printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf((char *)&acpu_task_output->val[0], ACPU_TASK_OUTPUT_VAL_SIZE - 1, fmt, args);
    va_end(args);
    acpu_task_output->error_code = (uint8_t)ACPU_ERR_PRINTF;
    MAILBOX_HandleTypeDef handle;
    handle.Instance = A2H_MAILBOX;
    __HAL_MAILBOX_TRIGGER_CHANNEL_IT(&handle, (ACPU_TASK_DONE_NTF_QUEUE % IPC_HW_QUEUE_NUM));

    while (acpu_task_output->error_code);
}
void acpu_send_assert(const char *file, int line)
{
    MAILBOX_HandleTypeDef handle;

    acpu_task_output->error_code = (uint8_t)ACPU_ERR_ASSERT;

    acpu_task_output->seq_no = acpu_task_input->seq_no;
    snprintf((char *)&acpu_task_output->val[0], ACPU_TASK_OUTPUT_VAL_SIZE - 1, "%s %d\n", file, line);
    acpu_task_output->task_name = acpu_task_input->task_name;

    handle.Instance = A2H_MAILBOX;
    __HAL_MAILBOX_TRIGGER_CHANNEL_IT(&handle, (ACPU_TASK_DONE_NTF_QUEUE % IPC_HW_QUEUE_NUM));

    while (1);
}

#define ACPU_ASSERT(ex) if (!(ex)) acpu_send_assert(__FILE__, __LINE__)

//static OpusEncoder *encoder;
void acpu_send_result(void *val, uint32_t val_size)
{
    MAILBOX_HandleTypeDef handle;

    if (val_size > ACPU_TASK_OUTPUT_VAL_SIZE)
    {
        acpu_task_output->error_code = 1;
    }

    acpu_task_output->seq_no = acpu_task_input->seq_no;
    for (uint32_t i = 0; i < val_size; i++)
    {
        acpu_task_output->val[i] = *((uint8_t *)val + i);
    }

    acpu_task_output->task_name = acpu_task_input->task_name;
    acpu_task_input->task_name = ACPU_TASK_INVALID;

    handle.Instance = A2H_MAILBOX;
    __HAL_MAILBOX_TRIGGER_CHANNEL_IT(&handle, (ACPU_TASK_DONE_NTF_QUEUE % IPC_HW_QUEUE_NUM));
}

void acpu_send_result2(void *val, uint32_t val_size, uint8_t error_code)
{
    acpu_task_output->error_code = error_code;
    acpu_send_result(val, val_size);
}

__WEAK void acpu_main(uint8_t task_name, void *param)
{
    switch (task_name)
    {
    case ACPU_TASK_0:
    {
        acpu_send_result("task_0", strlen("task_0") + 1);
        break;
    }
    case ACPU_TASK_1:
    {
        acpu_send_result("task_1", strlen("task_1") + 1);
        break;
    }
#ifdef PKG_LIB_OPUS
    case ACPU_TASK_opus_encoder_init:
    {
        //acpu_printf("acpu: encode init\n");
        opus_encode_init_arg_t *arg  = (opus_encode_init_arg_t *)param;
        //ACPU_ASSERT(arg);
        int result = opus_encoder_init((OpusEncoder *)arg->st, arg->fs, arg->channels, arg->application);
        acpu_task_output->ret = result;
        acpu_send_result(NULL, 0);
        break;
    }
    case ACPU_TASK_opus_encoder_ctl:
    {
        opus_encode_ctl_arg_t *arg  = (opus_encode_ctl_arg_t *)param;
        //ACPU_ASSERT(arg);
        OpusEncoder *encoder = (OpusEncoder *)arg->st;
#if 1
        opus_encoder_ctl(encoder, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_20_MS));
#else
        opus_encoder_ctl(encoder, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_20_MS));
#endif
        opus_encoder_ctl(encoder, OPUS_SET_VBR(1));
        opus_encoder_ctl(encoder, OPUS_SET_VBR_CONSTRAINT(1));

        opus_encoder_ctl(encoder, OPUS_SET_BITRATE(16000));
        opus_encoder_ctl(encoder, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
        opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(0));
        opus_encoder_ctl(encoder, OPUS_SET_LSB_DEPTH(24));

        opus_encoder_ctl(encoder, OPUS_SET_DTX(0));
        opus_encoder_ctl(encoder, OPUS_SET_INBAND_FEC(0));
        opus_encoder_ctl(encoder, OPUS_SET_PACKET_LOSS_PERC(0));
        opus_encoder_ctl(encoder, OPUS_SET_PREDICTION_DISABLED(0));

        opus_encoder_ctl(encoder, OPUS_SET_MAX_BANDWIDTH(OPUS_BANDWIDTH_WIDEBAND));
        opus_encoder_ctl(encoder, OPUS_SET_BANDWIDTH(OPUS_AUTO));

        acpu_send_result(NULL, 0);
        break;
    }
    case ACPU_TASK_opus_encode:
    {
        //acpu_printf("acpu: encode\n");
        opus_encode_arg_t *arg  = (opus_encode_arg_t *)param;
        //ACPU_ASSERT(arg->data && arg->pcm && arg->st);
        opus_int32 len = opus_encode((OpusEncoder *)arg->st,
                                     arg->pcm,
                                     arg->analysis_frame_size,
                                     (uint8_t *)arg->data,
                                     arg->max_data_bytes);
        //acpu_printf("acpu: encode = %d\n", len);
        acpu_task_output->ret = len;
        acpu_send_result(NULL, 0);
        break;
    }
    case ACPU_TASK_opus_decode:
    {
        opus_decode_arg_t *arg  = (opus_decode_arg_t *)param;
        //ACPU_ASSERT(arg->st);
        opus_int32 res = opus_decode((OpusDecoder *)arg->st,
                                     arg->data,
                                     arg->len,
                                     arg->pcm,
                                     arg->frame_size,
                                     arg->decode_fec);
        acpu_task_output->ret = res;
        acpu_send_result(NULL, 0);
        break;
    }
    case ACPU_TASK_opus_decoder_init:
    {
        opus_decode_init_arg_t *arg  = (opus_decode_init_arg_t *)param;
        //ACPU_ASSERT(arg);
        int de_ret = opus_decoder_init((OpusDecoder *)arg->st, arg->fs, arg->channels);
        acpu_task_output->ret = de_ret;
        acpu_send_result(NULL, 0);
        break;
    }
    case ACPU_TASK_opus_decoder_ctl:
    {
#if 0 //identify different decoder by id
        opus_decode_ctl_arg_t *arg  = (opus_decode_ctl_arg_t *)param;
        //ACPU_ASSERT(arg);
        OpusDecoder *decoder = (OpusDecoder *)arg->st;
        switch (arg->id)
        {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        default:
            opus_decoder_ctl(decoder,);
            break;
        }
#else
        ACPU_ASSERT(0);
#endif
        acpu_send_result(NULL, 0);
        break;
    }
#endif
#if ANYKA_RUN_IN_ACPU
    case ACPU_TASK_audio_3a_open:
    {
        acpu_audio_3a_open_parameter_t *arg  = (acpu_audio_3a_open_parameter_t *)param;
        int ret = acpu_audio_3a_open(arg);
        acpu_send_result(0, ret);
        break;
    }
    case ACPU_TASK_audio_3a_uplink:
    {
        acpu_audio_3a_uplink_parameter_t *arg  = (acpu_audio_3a_uplink_parameter_t *)param;
        acpu_audio_3a_uplink(arg);
        acpu_send_result(0, 0);
        break;
    }
    case ACPU_TASK_audio_3a_downlink:
    {
        acpu_audio_3a_downlink_parameter_t *arg  = (acpu_audio_3a_downlink_parameter_t *)param;
        acpu_audio_3a_downlink(arg);
        acpu_send_result(0, 0);
        break;
    }
    case ACPU_TASK_audio_3a_close:
    {
        acpu_audio_3a_close();
        acpu_send_result(0, 0);
        break;
    }
#endif
    default:
    {
        //here will come sometimes
        //ACPU_ASSERT(0);
    }
    }
}

int main(void)
{
    while (1)
    {
        if (ACPU_TASK_INVALID != acpu_task_input->task_name)
        {
            acpu_task_output->error_code = 0;
            acpu_main(acpu_task_input->task_name, (void *)&acpu_task_input->param[0]);
        }
    }
}

#else
void acpu_power_on(void)
{
    HAL_RCC_ResetACPU();
    HAL_RCC_ReleaseACPU();
}

void acpu_power_off(void)
{
    HAL_RCC_ResetACPU();
}

__ROM_USED int acpu(int argc, char **argv)
{
    if (argc > 1)
    {

        if (strcmp("on", argv[1]) == 0)
        {
            acpu_power_on();
        }
        else if (strcmp("off", argv[1]) == 0)
        {
            acpu_power_off();
        }
        else
        {
            goto __ERR;
        }
        return 0;
    }
    else
    {
        goto __ERR;
    }

__ERR:
    rt_kprintf("wrong arg\n");

    return 0;
}
MSH_CMD_EXPORT(acpu, acpu control);

#ifdef ACPU_CALLER_ENABLED
static rt_event_t g_call_start_event;

/*
    notice: can not call acpu function in acpu_caller_entry()
*/
void acpu_caller_entry(void *parameter)
{
    rt_uint32_t recv;
    while (1)
    {
        if (rt_event_recv(g_call_start_event, 1,
                          RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                          RT_WAITING_FOREVER, &recv) == RT_EOK)
        {
            if (hcpu_task_input->task_name == HCPU_TASK_MALLOC)
            {
                void *p = malloc(hcpu_task_input->input);
                if (!p)
                {
                    rt_kprintf("malloc %d\n", hcpu_task_input->input);
                    RT_ASSERT(0);
                }
                hcpu_task_output->ret = (uint32_t)p;
            }
            else if (hcpu_task_input->task_name == HCPU_TASK_FREE)
            {
                free((void *)hcpu_task_input->input);
            }
            else
            {
                RT_ASSERT(0);
            }
            acpu_task_output->error_code = 0;
        }
    }
}
#endif /* ACPU_CALLER_ENABLED */

int32_t acpu_task_done_ind(ipc_queue_handle_t handle, size_t size)
{
    if (acpu_task_output->error_code == ACPU_ERR_OK || acpu_task_output->error_code == ACPU_ERR_COMMON)
    {
        rt_sem_release(&acpu_task_done_sema);
    }
    else if (acpu_task_output->error_code == ACPU_ERR_PRINTF)
    {
        rt_kprintf("%s\n", acpu_task_output->val);
        acpu_task_output->error_code = ACPU_ERR_OK;
    }
    else if (acpu_task_output->error_code == ACPU_ERR_ASSERT)
    {
        rt_kprintf("acpu assert:%s\n", acpu_task_output->val);
        RT_ASSERT(0);
    }
#ifdef ACPU_CALLER_ENABLED
    else if (acpu_task_output->error_code == ACPU_ERR_CALL_HCPU)
    {
        rt_event_send(g_call_start_event, 1);
    }
#endif /* ACPU_CALLER_ENABLED */
    return 0;
}

RT_WEAK int acpu_init(void)
{
    ipc_queue_cfg_t q_cfg;
    rt_err_t err;
    int32_t r;
    rt_mutex_init(&acpu_task_mutex, "acpu", RT_IPC_FLAG_FIFO);
    err = rt_sem_init(&acpu_task_done_sema, "acpu_done", 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(RT_EOK == err);

    q_cfg.qid = ACPU_TASK_DONE_NTF_QUEUE;
    q_cfg.tx_buf_size = 0;
    q_cfg.tx_buf_addr = (uint32_t)NULL;
    q_cfg.tx_buf_addr_alias = (uint32_t)NULL;
    q_cfg.rx_buf_addr = (uint32_t)NULL;
    q_cfg.rx_ind = acpu_task_done_ind;
    q_cfg.user_data = 0;

    acpu_task_ntf_queue = ipc_queue_init(&q_cfg);

    RT_ASSERT(IPC_QUEUE_INVALID_HANDLE != acpu_task_ntf_queue);

    r = ipc_queue_open(acpu_task_ntf_queue);
    RT_ASSERT(0 == r);

    memset(acpu_task_input, 0, sizeof(*acpu_task_input));
    memset(acpu_task_output, 0, sizeof(*acpu_task_output));

#ifdef ACPU_CALLER_ENABLED
    g_call_start_event = rt_event_create("acall_s", RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_call_start_event);
    rt_thread_t tid = rt_thread_create("acpu",
                                       acpu_caller_entry,
                                       NULL,
                                       1024,
                                       RT_THREAD_PRIORITY_HIGH,
                                       RT_THREAD_TICK_DEFAULT);
    rt_thread_startup(tid);
#endif /* ACPU_CALLER_ENABLED */

    acpu_power_on();

    return 0;
}
INIT_PRE_APP_EXPORT(acpu_init);

static inline void lock()
{
    rt_mutex_take(&acpu_task_mutex, RT_WAITING_FOREVER);
}
static inline void unlock()
{
    rt_mutex_release(&acpu_task_mutex);
}

RT_WEAK void *acpu_run_task(uint8_t task_name, void *param, uint32_t param_size, uint8_t *error_code)
{
    uint32_t ret;
    rt_err_t err;
    MAILBOX_HandleTypeDef handle;

    if (param_size > ACPU_TASK_INPUT_PARAM_SIZE)
    {
        return NULL;
    }

#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
#endif /* RT_USING_PM */

    lock();

    /* ACPU doesn't use ipc_queue module, so sender won't unmask the interrupt for receiver,
     *  HCPU has to unmask by itself
     */
    handle.Instance = A2H_MAILBOX;
    __HAL_MAILBOX_UNMASK_CHANNEL_IT(&handle, (ACPU_TASK_DONE_NTF_QUEUE % IPC_HW_QUEUE_NUM));

    acpu_task_output->task_name = ACPU_TASK_INVALID;
    if (param_size && param)
    {
        memcpy((void *)acpu_task_input->param, param, param_size);
    }
    acpu_task_input->seq_no++;
    acpu_task_input->task_name = task_name;

    HAL_RCC_EnableModule(RCC_MOD_ACPU);

    err = rt_sem_take(&acpu_task_done_sema, RT_WAITING_FOREVER);
    RT_ASSERT(RT_EOK == err);

    HAL_RCC_DisableModule(RCC_MOD_ACPU);

    RT_ASSERT(task_name == acpu_task_output->task_name);

    // rt_kprintf("acpu error_code=%d\n", acpu_task_output->error_code);
    if (acpu_task_output->error_code == (uint8_t)ACPU_ERR_ASSERT)
    {
        rt_kprintf("acpu assert: %s\n", acpu_task_output->val);
        RT_ASSERT(0);
    }
    if (error_code)
    {
        *error_code = acpu_task_output->error_code;
    }
    ret = acpu_task_output->ret;

    unlock();

#ifdef RT_USING_PM
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif /* RT_USING_PM */

    //can't use acpu_task_output->val for many task call acpu_run_task
    if (task_name == ACPU_TASK_0 || task_name == ACPU_TASK_1)
    {
        if (0 == acpu_task_output->error_code)
        {
            return (void *)acpu_task_output->val;
        }
        else
        {
            return NULL;
        }
    }
    return (void *)ret;
}

#endif /* SOC_BF0_ACPU */

