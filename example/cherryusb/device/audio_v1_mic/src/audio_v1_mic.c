/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co, Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "audio_server.h"
#include "rtthread.h"
#include "usb_audio.h"
#include "usbd_audio.h"
#include "usbd_core.h"

#define USBD_VID 0x38f4
#define USBD_PID 0xFFFF
#define USBD_MAX_POWER 100
#define USBD_LANGID_STRING 1033

#define EP_INTERVAL_FS 0x01

#define AUDIO_VERSION 0x0100

#define AUDIO_IN_EP 0x85

#define AUDIO_IN_FU_ID 0x02

/* AUDIO Class Config */
#define AUDIO_FREQ 16000U

#define IN_CHANNEL_NUM 1

#if IN_CHANNEL_NUM == 1
    #define INPUT_CTRL 0x03, 0x03
    #define INPUT_CH_ENABLE 0x0000
#elif IN_CHANNEL_NUM == 2
    #define INPUT_CTRL 0x03, 0x03, 0x03
    #define INPUT_CH_ENABLE 0x0003
#elif IN_CHANNEL_NUM == 3
    #define INPUT_CTRL 0x03, 0x03, 0x03, 0x03
    #define INPUT_CH_ENABLE 0x0007
#elif IN_CHANNEL_NUM == 4
    #define INPUT_CTRL 0x03, 0x03, 0x03, 0x03, 0x03
    #define INPUT_CH_ENABLE 0x000f
#elif IN_CHANNEL_NUM == 5
    #define INPUT_CTRL 0x03, 0x03, 0x03, 0x03, 0x03, 0x03
    #define INPUT_CH_ENABLE 0x001f
#elif IN_CHANNEL_NUM == 6
    #define INPUT_CTRL 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03
    #define INPUT_CH_ENABLE 0x003F
#elif IN_CHANNEL_NUM == 7
    #define INPUT_CTRL 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03
    #define INPUT_CH_ENABLE 0x007f
#elif IN_CHANNEL_NUM == 8
    #define INPUT_CTRL 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03
    #define INPUT_CH_ENABLE 0x00ff
#endif

/* AudioFreq * DataSize (2 bytes) * NumChannels (Stereo: 1) */
/* 16bit(2 Bytes) One Channel(Mono:1) */
#define AUDIO_IN_PACKET ((uint32_t)((AUDIO_FREQ * 2 * IN_CHANNEL_NUM) / 1000))

#define USB_AUDIO_CONFIG_DESC_SIZ                                              \
  (unsigned long)(9 + AUDIO_AC_DESCRIPTOR_LEN(1) +                             \
                  AUDIO_SIZEOF_AC_INPUT_TERMINAL_DESC +                        \
                  AUDIO_SIZEOF_AC_FEATURE_UNIT_DESC(IN_CHANNEL_NUM, 1) +       \
                  AUDIO_SIZEOF_AC_OUTPUT_TERMINAL_DESC +                       \
                  AUDIO_AS_DESCRIPTOR_LEN(1))

#define AUDIO_AC_SIZ                                                           \
  (AUDIO_SIZEOF_AC_HEADER_DESC(1) + AUDIO_SIZEOF_AC_INPUT_TERMINAL_DESC +      \
   AUDIO_SIZEOF_AC_FEATURE_UNIT_DESC(IN_CHANNEL_NUM, 1) +                      \
   AUDIO_SIZEOF_AC_OUTPUT_TERMINAL_DESC)

static const uint8_t device_descriptor[] =
{
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID,
                               0x0001, 0x01),
};

static const uint8_t config_descriptor_fs[] =
{
    USB_CONFIG_DESCRIPTOR_INIT(USB_AUDIO_CONFIG_DESC_SIZ, 0x02, 0x01,
                               USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    AUDIO_AC_DESCRIPTOR_INIT(0x00, 0x02, AUDIO_AC_SIZ, 0x00, 0x01),
    AUDIO_AC_INPUT_TERMINAL_DESCRIPTOR_INIT(0x01, AUDIO_INTERM_MIC,
                                            IN_CHANNEL_NUM, INPUT_CH_ENABLE),
    AUDIO_AC_FEATURE_UNIT_DESCRIPTOR_INIT(AUDIO_IN_FU_ID, 0x01, 0x01,
                                          INPUT_CTRL),
    AUDIO_AC_OUTPUT_TERMINAL_DESCRIPTOR_INIT(0x03, AUDIO_TERMINAL_STREAMING,
            AUDIO_IN_FU_ID),
    AUDIO_AS_DESCRIPTOR_INIT(0x01, 0x03, IN_CHANNEL_NUM, 2, 16, AUDIO_IN_EP,
                             0x05, AUDIO_IN_PACKET, EP_INTERVAL_FS,
                             AUDIO_SAMPLE_FREQ_3B(AUDIO_FREQ)),
};

static const uint8_t device_quality_descriptor[] =
{
    USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, 0x01),
};

static const char *string_descriptors[] =
{
    (const char[]){0x09, 0x04}, /* Langid */
    "SiFli",                    /* Manufacturer */
    "SiFli UAC DEMO",           /* Product */
    "2025083018",               /* Serial Number */
};

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return config_descriptor_fs;
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return device_quality_descriptor;
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    (void)speed;

    if (index >= (sizeof(string_descriptors) / sizeof(char *)))
    {
        return NULL;
    }
    return string_descriptors[index];
}

const struct usb_descriptor audio_v1_descriptor =
{
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback,
};

/* RTOS IPC objects for synchronization and communication */
static struct rt_semaphore tx_sem;
static struct rt_mailbox uac_mb;
static rt_uint32_t uac_mb_pool[4];

// UAC commands for the mailbox
enum uac_cmd
{
    UAC_CMD_UNKNOWN = 0,
    UAC_CMD_OPEN,
    UAC_CMD_CLOSE,
};

static audio_client_t g_rec_client = 0;
static volatile bool tx_flag;

static uint32_t s_mic_sample_rate = AUDIO_FREQ;

static struct usbd_interface intf0;
static struct usbd_interface intf1;

static void usbd_audio_iso_callback(uint8_t busid, uint8_t ep, uint32_t nbytes);
static struct usbd_endpoint audio_in_ep = {.ep_cb = usbd_audio_iso_callback,
           .ep_addr = AUDIO_IN_EP
};

static struct audio_entity_info audio_entity_table[] =
{
    {
        .bEntityId = AUDIO_IN_FU_ID,
        .bDescriptorSubtype = AUDIO_CONTROL_FEATURE_UNIT,
        .ep = AUDIO_IN_EP
    },
};

/* extern function definition */
static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    (void)busid;

    rt_kprintf("usbd event %d\n", event);
    switch (event)
    {
    case USBD_EVENT_RESET:
        break;
    case USBD_EVENT_CONNECTED:
        break;
    case USBD_EVENT_DISCONNECTED:
        break;
    case USBD_EVENT_RESUME:
        break;
    case USBD_EVENT_SUSPEND:
        break;
    case USBD_EVENT_CONFIGURED:
        break;
    case USBD_EVENT_SET_REMOTE_WAKEUP:
        break;
    case USBD_EVENT_CLR_REMOTE_WAKEUP:
        break;

    default:
        break;
    }
}

void usbd_audio_set_volume(uint8_t busid, uint8_t ep, uint8_t ch, int volume)
{
    (void)busid;
    (void)ch;
    if (ep == AUDIO_IN_EP)
    {
        uint8_t server_volume = (uint8_t)((volume * AUDIO_MAX_VOLUME) / 100);
        audio_server_set_public_volume(server_volume);
    }
}

int usbd_audio_get_volume(uint8_t busid, uint8_t ep, uint8_t ch)
{
    (void)busid;
    (void)ch;
    if (ep == AUDIO_IN_EP)
    {
        uint8_t server_volume =
            audio_server_get_private_volume(AUDIO_TYPE_LOCAL_RECORD);
        return (server_volume * 100) / AUDIO_MAX_VOLUME;
    }
    return 0;
}

void usbd_audio_set_mute(uint8_t busid, uint8_t ep, uint8_t ch, bool mute)
{
    (void)busid;
    (void)ch;
    if (ep == AUDIO_IN_EP)
    {
        audio_server_set_public_mic_mute(mute);
    }
}

bool usbd_audio_get_mute(uint8_t busid, uint8_t ep, uint8_t ch)
{
    (void)busid;
    (void)ch;
    if (ep == AUDIO_IN_EP)
    {
        return audio_server_get_public_mic_mute();
    }
    return false;
}

void usbd_audio_set_sampling_freq(uint8_t busid, uint8_t ep,
                                  uint32_t sampling_freq)
{
    (void)busid;
    if (ep == AUDIO_IN_EP)
    {
        s_mic_sample_rate = sampling_freq;
    }
}

uint32_t usbd_audio_get_sampling_freq(uint8_t busid, uint8_t ep)
{
    (void)busid;
    if (ep == AUDIO_IN_EP)
    {
        return s_mic_sample_rate;
    }
    return 0;
}

/**
 * @brief Record audio callback function
 * This function sends audio data to USB in chunks. It uses a semaphore
 * to wait for each transfer to complete before sending the next one.
 */
static int audio_callback_record(audio_server_callback_cmt_t cmd,
                                 void *callback_userdata, uint32_t reserved)
{
    if (cmd == as_callback_cmd_data_coming)
    {
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;

        if (!tx_flag)
        {
            // rt_kprintf("[UAC Record] TX disabled, dropping data\n");
            return 0;
        }

        const uint32_t chunk_size = AUDIO_IN_PACKET;
        if (p->data_len % chunk_size == 0)
        {
            rt_kprintf("[UAC Record] data_len %d is a multiple of chunk_size %d\n",
                       p->data_len, chunk_size);
            // return -1;
        }

        const uint32_t total_chunks = p->data_len / chunk_size;
        uint8_t *data_ptr = (uint8_t *)p->data;

        for (uint32_t i = 0; i < total_chunks; i++)
        {
            /* Send the current data chunk */
            int ret = usbd_ep_start_write(0, AUDIO_IN_EP, data_ptr + (i * chunk_size),
                                          chunk_size);
            if (ret == 0)
            {
                rt_kprintf("[UAC Record] send chunk %d, ret=%d\n", i, ret);
                // return -1;
            }

            /* Wait for the transfer to complete. The task will block here without
             * consuming CPU. */
            /* A timeout is added to prevent permanent blocking. */
            rt_err_t result = rt_sem_take(&tx_sem, rt_tick_from_millisecond(10));
            if (result != RT_EOK)
            {
                rt_kprintf(
                    "[UAC Record] Timed out waiting for TX completion on chunk %d\n",
                    i);
                /* You might want to handle this error, e.g., by stopping the stream */
                return -1;
            }
        }
    }

    return 0;
}

void usbd_audio_open(uint8_t busid, uint8_t intf)
{
    (void)busid;
    (void)intf;

    if (intf == 1)   // Interface 1 is the audio streaming interface
    {
        // Send an OPEN command to the audio task via mailbox
        rt_mb_send(&uac_mb, (rt_uint32_t)UAC_CMD_OPEN);
        rt_kprintf("UAC OPEN command sent\n");
    }
}

void usbd_audio_close(uint8_t busid, uint8_t intf)
{
    (void)busid;
    (void)intf;

    if (intf == 1)
    {
        // Send a CLOSE command to the audio task via mailbox
        rt_mb_send(&uac_mb, (rt_uint32_t)UAC_CMD_CLOSE);
        rt_kprintf("UAC CLOSE command sent\n");
    }
}

void audio_v1_init(uint8_t busid, uint32_t reg_base)
{
    // Initialize the semaphore for TX synchronization
    // Initial value is 0, max value is 1 (binary semaphore)
    rt_sem_init(&tx_sem, "tx_sem", 0, RT_IPC_FLAG_FIFO);

    // Initialize the mailbox for UAC commands
    rt_mb_init(&uac_mb, "uac_mb", &uac_mb_pool[0],
               sizeof(uac_mb_pool) / sizeof(rt_uint32_t), RT_IPC_FLAG_FIFO);

    usbd_desc_register(busid, &audio_v1_descriptor);
    usbd_add_interface(busid, usbd_audio_init_intf(busid, &intf0, AUDIO_VERSION,
                       audio_entity_table, 1));
    usbd_add_interface(busid, usbd_audio_init_intf(busid, &intf1, AUDIO_VERSION,
                       audio_entity_table, 1));
    usbd_add_endpoint(busid, &audio_in_ep);

    usbd_initialize(busid, reg_base, usbd_event_handler);
}

void audio_v1_task(uint8_t busid)
{
    (void)busid;
    rt_uint32_t cmd;

    // This task now blocks waiting for commands, instead of polling
    while (1)
    {
        // Block indefinitely until a message is received in the mailbox
        if (rt_mb_recv(&uac_mb, &cmd, RT_WAITING_FOREVER) == RT_EOK)
        {
            switch (cmd)
            {
            case UAC_CMD_OPEN:
                if (g_rec_client)
                {
                    rt_kprintf("[RECORD] already running, ignoring OPEN cmd.\n");
                    continue;
                }
                rt_kprintf("[RECORD] Received OPEN command. Starting...\n");
                tx_flag = true;

                // It's good practice to clear the semaphore in case of stale signals
                rt_sem_control(&tx_sem, RT_IPC_CMD_RESET, RT_NULL);

                audio_parameter_t param = {0};
                param.write_bits_per_sample = 16;
                param.write_channnel_num = 1;
                param.write_samplerate = 16000;
                param.write_cache_size = 32;

                param.read_bits_per_sample = 16;
                param.read_channnel_num = 1;
                param.read_samplerate = 16000;
                // The cache size for read should match the data size from the audio
                // server
                param.read_cache_size = 320;

                g_rec_client = audio_open(AUDIO_TYPE_LOCAL_RECORD, AUDIO_RX, &param,
                                          audio_callback_record, NULL);
                if (!g_rec_client)
                {
                    rt_kprintf("[RECORD] Failed to open record client.\n");
                    tx_flag = false;
                }
                else
                {
                    rt_kprintf("[RECORD] Record client opened successfully.\n");
                }
                break;

            case UAC_CMD_CLOSE:
                rt_kprintf("[RECORD] Received CLOSE command. Stopping...\n");
                tx_flag = false;
                if (g_rec_client)
                {
                    audio_close(g_rec_client);
                    g_rec_client = 0;
                    rt_kprintf("[RECORD] Record client closed.\n");
                }
                break;

            default:
                rt_kprintf("[RECORD] Received unknown command: %d\n", cmd);
                break;
            }
        }
    }
}

/* static function definition */
static void usbd_audio_iso_callback(uint8_t busid, uint8_t ep,
                                    uint32_t nbytes)
{
    (void)busid;
    (void)ep;
    (void)nbytes;

    // This callback is in an interrupt context or a high-priority USB task.
    // Release the semaphore to signal that the transfer is complete.
    rt_sem_release(&tx_sem);
}
