/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
   file brief: ACPU main function implementation which runs on ACPU
*/
#include <rtconfig.h>
#include "rtthread.h"
#include "ipc_queue.h"
#include <string.h>
#include "acpu_ctrl.h"
#include "acpu_ctrl_private.h"
#include "bf0_mbox_common.h"

#ifdef SOC_BF0_ACPU
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <alloca.h>
#ifdef PKG_LIB_OPUS
    #include "opus.h"
#endif
#if defined(ANYKA_RUN_IN_ACPU)
    #include "audio_3a_anyka.h"
#endif

static rt_mailbox_t g_call_mb;
static acpu_ctrl_ipc_msg_t *p_received_msg;
#ifdef DRV_EPIC_NEW_API
    static rt_mailbox_t g_epic_mb;
#endif /* DRV_EPIC_NEW_API */

static void *req_hcpu_run_task(uint8_t task_name, void *param, uint32_t param_size, uint8_t *error_code)
{
    rt_err_t err;

#ifdef RT_USING_PM
    rt_pm_request(PM_SLEEP_MODE_IDLE);
#endif /* RT_USING_PM */

    //Create blocking semaphore
    char task_name_str[RT_NAME_MAX];
    snprintf(task_name_str, sizeof(task_name_str), "tsk_%d", task_name);
    rt_sem_t sem = rt_sem_create(task_name_str, 0, RT_IPC_FLAG_FIFO);
    RT_ASSERT(sem);


    //Send message to ACPU via IPC queue
    size_t wr_size;
    size_t msg_size;
    acpu_ctrl_ipc_msg_t msg;
    acpu_ctrl_ipc_msg_t *p_msg = &msg;
    msg.is_rsp = 0;
    msg.task_id = task_name;
    msg.task_param_size = param_size;
    msg.task_param = param;
    msg.sema = sem;
    msg.ret_error_code = 0;
    msg.ret_value = 0;

    msg_size = sizeof(acpu_ctrl_ipc_msg_t *);
    wr_size = ipc_queue_write(sys_get_ah_ipc_queue(), &p_msg, msg_size, 1000);
    RT_ASSERT(wr_size == msg_size);



    //Wait for task completion, and delete semaphore
    err = rt_sem_take(sem, rt_tick_from_millisecond(1000));
    RT_ASSERT(RT_EOK == err);
    rt_sem_delete(sem);

    //Make sure the message was processed by ACPU
    RT_ASSERT(1 == msg.is_rsp);
    RT_ASSERT(task_name == msg.task_id);

    // rt_kprintf("acpu error_code=%d\n", msg.ret_error_code);
    if (msg.ret_error_code == (uint8_t)ACPU_ERR_ASSERT)
    {
        rt_kprintf("acpu assert: %s\n", msg.ret_value);
        RT_ASSERT(0);
    }
    if (error_code)
    {
        *error_code = msg.ret_error_code;
    }


#ifdef RT_USING_PM
    rt_pm_release(PM_SLEEP_MODE_IDLE);
#endif /* RT_USING_PM */

    return (void *)msg.ret_value;
}

void *acpu_call_hcpu_malloc(uint32_t size)
{
    return req_hcpu_run_task(HCPU_TASK_MALLOC, (void *)size, sizeof(size), NULL);
}
void acpu_call_hcpu_free(void *p)
{
    req_hcpu_run_task(HCPU_TASK_FREE, p, sizeof(p), NULL);
}

void acpu_printf(const char *fmt, ...)
{
    char print_buffer[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf((char *)&print_buffer[0], 128 - 1, fmt, args);
    va_end(args);
    print_buffer[128 - 1] = '\0';

    req_hcpu_run_task(HCPU_TASK_PRINTF, print_buffer, sizeof(print_buffer), NULL);
}

void acpu_send_assert(const char *file, int line)
{
    RT_ASSERT(p_received_msg);

    char assert_info[128];
    p_received_msg->is_rsp = 1;
    p_received_msg->ret_error_code = ACPU_ERR_ASSERT;
    p_received_msg->ret_value = (uint32_t) &assert_info[0];
    snprintf((char *)&assert_info[0], 128 - 1, "%s %d\n", file, line);
    assert_info[127] = '\0';

    size_t wr_size = ipc_queue_write(sys_get_ah_ipc_queue(), &p_received_msg, sizeof(acpu_ctrl_ipc_msg_t *), 1000);
    RT_ASSERT(wr_size == sizeof(acpu_ctrl_ipc_msg_t *));

    RT_ASSERT(0);
}

#define ACPU_ASSERT(ex) if (!(ex)) acpu_send_assert(__FILE__, __LINE__)


static void acpu_send_result2(acpu_ctrl_ipc_msg_t *p_msg, uint32_t err_code, uint32_t ret_value)
{
    RT_ASSERT(p_msg);

    p_msg->is_rsp = 1;
    p_msg->ret_error_code = err_code;
    p_msg->ret_value = ret_value;


    size_t wr_size = ipc_queue_write(sys_get_ah_ipc_queue(), &p_msg, sizeof(acpu_ctrl_ipc_msg_t *), 1000);
    RT_ASSERT(wr_size == sizeof(acpu_ctrl_ipc_msg_t *));


    // acpu_printf("acpu: send result done\n");
}

void acpu_send_result(uint32_t err_code, uint32_t ret_value)
{
    SCB_CleanInvalidateDCache();
    acpu_send_result2(p_received_msg, err_code, ret_value);
}



__WEAK void acpu_main(uint8_t task_name, void *param)
{
    switch (task_name)
    {
    case ACPU_TASK_0:
    {
        acpu_send_result(0, (uint32_t)"task_0");
        break;
    }
    case ACPU_TASK_1:
    {
        acpu_send_result(0, (uint32_t)"task_1");
        break;
    }
#ifdef PKG_LIB_OPUS
    case ACPU_TASK_opus_encoder_init:
    {
        //acpu_printf("acpu: encode init\n");
        opus_encode_init_arg_t *arg  = (opus_encode_init_arg_t *)param;
        //ACPU_ASSERT(arg);
        int result = opus_encoder_init((OpusEncoder *)arg->st, arg->fs, arg->channels, arg->application);
        acpu_send_result(0, result);
        break;
    }
    case ACPU_TASK_opus_encoder_ctl:
    {
        opus_encode_ctl_arg_t *arg  = (opus_encode_ctl_arg_t *)param;
        //ACPU_ASSERT(arg);
        OpusEncoder *encoder = (OpusEncoder *)arg->st;
#if 1
        opus_encoder_ctl(encoder, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_10_MS));
#else
        opus_encoder_ctl(encoder, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_10_MS));
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

        acpu_send_result(0, 0);
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
        acpu_send_result(0, len);
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
        acpu_send_result(0, res);
        break;
    }
    case ACPU_TASK_opus_decoder_init:
    {
        opus_decode_init_arg_t *arg  = (opus_decode_init_arg_t *)param;
        //ACPU_ASSERT(arg);
        int de_ret = opus_decoder_init((OpusDecoder *)arg->st, arg->fs, arg->channels);
        acpu_send_result(0, de_ret);
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
        acpu_send_result(0, 0);
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
        acpu_printf("acpu: unknown task id %d\n", task_name);
        acpu_send_result(0, (uint32_t)"unknown task");
    }
    }
}


static int32_t queue_rx_ind(ipc_queue_handle_t handle, size_t size)
{
    acpu_ctrl_ipc_msg_t *p_msg;
    size_t rd_size = ipc_queue_read(handle, &p_msg, sizeof(acpu_ctrl_ipc_msg_t *));
    RT_ASSERT(rd_size == sizeof(acpu_ctrl_ipc_msg_t *));
    RT_ASSERT(p_msg);

    if (p_msg->is_rsp)
    {
        if (p_msg->ret_error_code == ACPU_ERR_OK)
        {
            if (p_msg->sema) rt_sem_release(p_msg->sema);
        }
        else if (p_msg->ret_error_code == ACPU_ERR_ASSERT)
        {
            RT_ASSERT(0);
        }
    }
    else
    {
        rt_err_t err;
        if (0)
        {
        }
#ifdef DRV_EPIC_NEW_API
        else if (ACPU_TASK_epic_rl == p_msg->task_id)
        {
            err = rt_mb_send(g_epic_mb, (rt_uint32_t)p_msg);
        }
#endif /*DRV_EPIC_NEW_API*/
        else
        {
            err = rt_mb_send(g_call_mb, (rt_uint32_t)p_msg);
        }
        RT_ASSERT(err == RT_EOK);
    }

    return 0;
}
#ifdef DRV_EPIC_NEW_API
#include "drv_epic.h"
extern rt_err_t drv_epic_render_list(void *p_drv_epic, void *list);
extern rt_err_t drv_epic_render_list_scale(void *p_drv_epic, void *list, void *p_scaled_area);
static void epic_rl_task_entry(void *parameter)
{
    rt_kprintf("epic_rl_task_entry start\n");

    while (1)
    {
        acpu_ctrl_ipc_msg_t *p_msg;

        if (rt_mb_recv(g_epic_mb, (rt_uint32_t *)&p_msg, RT_WAITING_FOREVER) == RT_EOK)
        {
            if (p_msg)
            {
                epic_rl_arg_t *arg = (epic_rl_arg_t *)p_msg->task_param;
                rt_err_t ret;
                //acpu_printf("epic_rl_task_entry: 0x%x, 0x%x\n", arg->p_drv_epic, arg->p_render_list);

                if (arg->p_scaled_area == NULL)
                    ret = drv_epic_render_list(arg->p_drv_epic, arg->p_render_list);
                else
                    ret = drv_epic_render_list_scale(arg->p_drv_epic, arg->p_render_list, arg->p_scaled_area);

                acpu_send_result2(p_msg, 0, ret);
            }
        }
    }

}
#endif /*DRV_EPIC_NEW_API*/

int main(void)
{
    rt_err_t result;
    ipc_queue_handle_t ah_ipc_queue;

    g_call_mb = rt_mb_create("recv_req", 1, RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_call_mb);


#ifdef DRV_EPIC_NEW_API
    g_epic_mb = rt_mb_create("epic_rl", 1, RT_IPC_FLAG_FIFO);
    RT_ASSERT(g_epic_mb);

    rt_thread_t tid = rt_thread_create("epic_rl", epic_rl_task_entry,
                                       NULL, 4096,
                                       RT_THREAD_PRIORITY_HIGH,
                                       RT_THREAD_TICK_DEFAULT);
    rt_thread_startup(tid);
#endif /* DRV_EPIC_NEW_API */

    ah_ipc_queue = sys_init_ah_ipc_queue(queue_rx_ind);
    RT_ASSERT(IPC_QUEUE_INVALID_HANDLE != ah_ipc_queue);

    result = ipc_queue_open(ah_ipc_queue);
    RT_ASSERT(0 == result);

    while (1)
    {
        acpu_ctrl_ipc_msg_t *p_msg;

        if (rt_mb_recv(g_call_mb, (rt_uint32_t *)&p_msg, RT_WAITING_FOREVER) == RT_EOK)
        {
            if (p_msg)
            {
                p_received_msg = p_msg;
                acpu_main(p_received_msg->task_id, p_received_msg->task_param);
                p_received_msg = NULL;
            }
        }
    }

    return RT_EOK;
}
#endif /* SOC_BF0_ACPU */

