/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtthread.h"
#include "bf0_hal.h"
#include "usbh_core.h"
#include <stdio.h>
#include <string.h>
#include "usbh_uvc_stream.h"
#include "usbh_uac_stream.h"

/* buffer size should be adjust according to your application. for example,
 * mjpeg buffer could be smaller than yuyv, and higher resolution buffer should
 * be larger than lower resolution.*/
#define BUFFER_SIZE 32768

static USB_MEM_ALIGNX uint8_t frame_buffer1[BUFFER_SIZE];
static USB_MEM_ALIGNX uint8_t frame_buffer2[BUFFER_SIZE];
static struct usbh_videoframe frame_pool[2];

static USB_MEM_ALIGNX uint8_t frame_buffer[AUDIO_MIC_EP_MAX_MPS * AUDIO_MIC_ISO_PACKETS * 8];
static struct usbh_audioframe frame_pool2[8];

static void usbh_video_frame_thread(void *argument)
{
    int ret;
    struct usbh_videoframe *frame;

    while (1)
    {
        ret = usbh_video_stream_dequeue(&frame, 0xfffffff);
        if (ret < 0)
        {
            continue;
        }

        USB_LOG_RAW("frame buf:%p,frame len:%d\r\n", frame->frame_buf, frame->frame_size);

        usbh_video_stream_enqueue(frame);
    }
}

static void usbh_audio_mic_frame_thread(void *argument)
{
    int ret;
    struct usbh_audioframe *frame;

    while (1)
    {
        ret = usbh_audio_mic_stream_dequeue(&frame, 0xfffffff);
        if (ret < 0)
        {
            continue;
        }

        USB_LOG_RAW("frame buf:%p,frame len:%d\r\n", frame->frame_buf, frame->frame_size);

        usbh_audio_mic_stream_enqueue(frame);
    }
}
/**
 * @brief  Main program
 * @param  None
 * @retval 0 if success, otherwise failure number
 */
int main(void)
{
    rt_kprintf("cherryusb host demo!\n");

    for (uint8_t i = 0; i < 8; i++)
    {
        frame_pool2[i].frame_buf = frame_buffer + i * AUDIO_MIC_EP_MAX_MPS * AUDIO_MIC_ISO_PACKETS;
        frame_pool2[i].frame_bufsize = AUDIO_MIC_EP_MAX_MPS * AUDIO_MIC_ISO_PACKETS;
    }

    frame_pool[0].frame_buf = frame_buffer1;
    frame_pool[0].frame_bufsize = 640 * 480 * 2;
    frame_pool[1].frame_buf = frame_buffer2;
    frame_pool[1].frame_bufsize = 640 * 480 * 2;

    usbh_video_stream_create(frame_pool, 2);

    usb_osal_thread_create("uvc_frame", 3072, 5, usbh_video_frame_thread, NULL);
    extern void usbh_video_fps_init(void);
    usbh_video_fps_init();

    usbh_audio_mic_stream_create(frame_pool2, 8);
    usb_osal_thread_create("uac_mic", 3072, 5, usbh_audio_mic_frame_thread, NULL);

    usbh_initialize(0, (uintptr_t)USBC_BASE, RT_NULL);

    while (1)
    {
        rt_thread_mdelay(1000);
    }
    return 0;
}

int usbh_uvc_start(int argc, char **argv)
{
    uint8_t type;

    if (argc < 2)
    {
        USB_LOG_ERR("please input correct command: usbh_uvc_start yuyv|mjpeg\r\n");
        return -1;
    }

    if (strcmp(argv[1], "yuyv") == 0)
    {
        type = 0;
    }
    else if (strcmp(argv[1], "mjpeg") == 0)
    {
        type = 1;
    }
    else
    {
        USB_LOG_ERR("unsupported type: %s\r\n", argv[1]);
        USB_LOG_ERR("please use yuyv or mjpeg\r\n");
        return -1;
    }

    usbh_video_stream_start(320, 240, type);
    return 0;
}

MSH_CMD_EXPORT(usbh_uvc_start, usbh_uvc_start);

int usbh_uvc_stop(int argc, char **argv)
{
    usbh_video_stream_stop();
    return 0;
}

MSH_CMD_EXPORT(usbh_uvc_stop, usbh_uvc_stop);

int usbh_uac_start(int argc, char **argv)
{
    uint32_t freq;

    if (argc < 2)
    {
        USB_LOG_ERR("please input correct command: usbh_uac_start freq\r\n");
        return -1;
    }

    freq = atoi(argv[1]);
    usbh_audio_mic_stream_start(freq);
    return 0;
}

MSH_CMD_EXPORT(usbh_uac_start, usbh_uac_start);

int usbh_uac_stop(int argc, char **argv)
{
    usbh_audio_mic_stream_stop();
    return 0;
}

MSH_CMD_EXPORT(usbh_uac_stop, usbh_uac_stop);

int usbh_uac_volume(int argc, char **argv)
{
    struct usbh_audio *audio_class;

    if (argc < 3)
    {
        USB_LOG_ERR("please input correct command: usbh_uac_volume volume is_tx\r\n");
        return -1;
    }

    audio_class = (struct usbh_audio *)usbh_find_class_instance("/dev/audio0");

    usbh_audio_set_mute(audio_class, false, false);
    int ret = usbh_audio_set_volume(audio_class, atoi(argv[1]), atoi(argv[2]));
    if (ret < 0)
    {
        USB_LOG_ERR("set volume failed, ret: %d\r\n", ret);
    }
    return 0;
}

MSH_CMD_EXPORT(usbh_uac_volume, usbh_uac_volume);