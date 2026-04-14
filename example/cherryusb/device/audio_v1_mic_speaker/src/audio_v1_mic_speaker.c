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

#define AUDIO_IN_EP 0x85  // Endpoint 5 IN for microphone
#define AUDIO_OUT_EP 0x02 // Endpoint 2 OUT for speaker

#define AUDIO_IN_FU_ID 0x02
#define AUDIO_OUT_FU_ID 0x05

/* AUDIO Class Config */
#define AUDIO_MIC_FREQ 16000U
#define AUDIO_MIC_FRAME_SIZE_BYTE 2u
#define AUDIO_MIC_RESOLUTION_BIT 16u
#define AUDIO_SPEAKER_FREQ 16000U
#define AUDIO_SPEAKER_FRAME_SIZE_BYTE 2u
#define AUDIO_SPEAKER_RESOLUTION_BIT 16u

#define IN_CHANNEL_NUM 1
#define OUT_CHANNEL_NUM 1

#define AUDIO_SAMPLE_FREQ(frq)                                                 \
    (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))

/* AudioFreq * DataSize (2 bytes) * NumChannels (Stereo: 2) */
#define AUDIO_OUT_PACKET                                                       \
    ((uint32_t)((AUDIO_SPEAKER_FREQ * AUDIO_SPEAKER_FRAME_SIZE_BYTE * 1) /     \
                1000))
/* 16bit(2 Bytes) Two Channels(Mono:2) */
#define AUDIO_IN_PACKET                                                        \
    ((uint32_t)((AUDIO_MIC_FREQ * AUDIO_MIC_FRAME_SIZE_BYTE * 1) / 1000))

#define USB_AUDIO_CONFIG_DESC_SIZ                                              \
    (unsigned long)(9 + AUDIO_AC_DESCRIPTOR_LEN(2) +                           \
                    AUDIO_SIZEOF_AC_INPUT_TERMINAL_DESC +                      \
                    AUDIO_SIZEOF_AC_FEATURE_UNIT_DESC(1, 1) +                  \
                    AUDIO_SIZEOF_AC_OUTPUT_TERMINAL_DESC +                     \
                    AUDIO_SIZEOF_AC_INPUT_TERMINAL_DESC +                      \
                    AUDIO_SIZEOF_AC_FEATURE_UNIT_DESC(1, 1) +                  \
                    AUDIO_SIZEOF_AC_OUTPUT_TERMINAL_DESC +                     \
                    AUDIO_AS_DESCRIPTOR_LEN(1) +                               \
                    AUDIO_AS_DESCRIPTOR_LEN(1))

#define AUDIO_AC_SIZ                                                           \
    (AUDIO_SIZEOF_AC_HEADER_DESC(2) + AUDIO_SIZEOF_AC_INPUT_TERMINAL_DESC +    \
     AUDIO_SIZEOF_AC_FEATURE_UNIT_DESC(1, 1) +                                 \
     AUDIO_SIZEOF_AC_OUTPUT_TERMINAL_DESC +                                    \
     AUDIO_SIZEOF_AC_INPUT_TERMINAL_DESC +                                     \
     AUDIO_SIZEOF_AC_FEATURE_UNIT_DESC(1, 1) +                                 \
     AUDIO_SIZEOF_AC_OUTPUT_TERMINAL_DESC)

static const uint8_t device_descriptor[] =
{
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID,
                               0x0001, 0x01),
};

static const uint8_t config_descriptor_fs[] =
{
    USB_CONFIG_DESCRIPTOR_INIT(USB_AUDIO_CONFIG_DESC_SIZ, 0x03, 0x01,
                               USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    AUDIO_AC_DESCRIPTOR_INIT(0x00, 0x03, AUDIO_AC_SIZ, 0x00, 0x01, 0x02),
    AUDIO_AC_INPUT_TERMINAL_DESCRIPTOR_INIT(0x01, AUDIO_INTERM_MIC,
                                            IN_CHANNEL_NUM, 0x0000),
    AUDIO_AC_FEATURE_UNIT_DESCRIPTOR_INIT(AUDIO_IN_FU_ID, 0x01, 0x01, 0x03,
                                          0x00),
    AUDIO_AC_OUTPUT_TERMINAL_DESCRIPTOR_INIT(0x03, AUDIO_TERMINAL_STREAMING,
            AUDIO_IN_FU_ID),
    AUDIO_AC_INPUT_TERMINAL_DESCRIPTOR_INIT(0x04, AUDIO_TERMINAL_STREAMING,
                                            OUT_CHANNEL_NUM, 0x0000),
    AUDIO_AC_FEATURE_UNIT_DESCRIPTOR_INIT(AUDIO_OUT_FU_ID, 0x04, 0x01, 0x03,
                                          0x00),
    AUDIO_AC_OUTPUT_TERMINAL_DESCRIPTOR_INIT(0x06, AUDIO_OUTTERM_SPEAKER,
            AUDIO_OUT_FU_ID),
    AUDIO_AS_DESCRIPTOR_INIT(
        0x01, 0x04, OUT_CHANNEL_NUM, AUDIO_SPEAKER_FRAME_SIZE_BYTE,
        AUDIO_SPEAKER_RESOLUTION_BIT, AUDIO_OUT_EP, 0x09, AUDIO_OUT_PACKET,
        EP_INTERVAL_FS, AUDIO_SAMPLE_FREQ_3B(AUDIO_SPEAKER_FREQ)),
    AUDIO_AS_DESCRIPTOR_INIT(
        0x02, 0x03, IN_CHANNEL_NUM, AUDIO_MIC_FRAME_SIZE_BYTE,
        AUDIO_MIC_RESOLUTION_BIT, AUDIO_IN_EP, 0x05, AUDIO_IN_PACKET,
        EP_INTERVAL_FS, AUDIO_SAMPLE_FREQ_3B(AUDIO_MIC_FREQ)),
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
static struct rt_mailbox uac_mb;
static rt_uint32_t uac_mb_pool[5];
static struct rt_semaphore mic_tx_sem;
static struct rt_semaphore speaker_rx_sem;

// UAC commands for the mailbox
enum uac_cmd
{
    UAC_CMD_UNKNOWN = 0,
    UAC_CMD_MIC_OPEN,
    UAC_CMD_MIC_CLOSE,
    UAC_CMD_SPEAKER_OPEN,
    UAC_CMD_SPEAKER_CLOSE,
};

static volatile bool mic_tx_flag;
static volatile bool speaker_rx_flag;
static audio_client_t g_mic_client = 0;
static audio_client_t g_speaker_client = 0;

static uint32_t s_mic_sample_rate = AUDIO_MIC_FREQ;
static uint32_t s_speaker_sample_rate = AUDIO_SPEAKER_FREQ;

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t
g_speaker_buffer[AUDIO_OUT_PACKET];

static struct usbd_interface intf0;
static struct usbd_interface intf1;
static struct usbd_interface intf2;

static void usbd_audio_in_callback(uint8_t busid, uint8_t ep, uint32_t nbytes);
static void usbd_audio_out_callback(uint8_t busid, uint8_t ep, uint32_t nbytes);
static struct usbd_endpoint audio_in_ep = {.ep_cb = usbd_audio_in_callback,
           .ep_addr = AUDIO_IN_EP
};
static struct usbd_endpoint audio_out_ep = {.ep_cb = usbd_audio_out_callback,
           .ep_addr = AUDIO_OUT_EP
};

static struct audio_entity_info audio_entity_table[] =
{
    {
        .bEntityId = AUDIO_IN_FU_ID,
        .bDescriptorSubtype = AUDIO_CONTROL_FEATURE_UNIT,
        .ep = AUDIO_IN_EP
    },
    {
        .bEntityId = AUDIO_OUT_FU_ID,
        .bDescriptorSubtype = AUDIO_CONTROL_FEATURE_UNIT,
        .ep = AUDIO_OUT_EP
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

    if (volume < -100)
        volume = -100;
    if (volume > 0)
        volume = 0;

    uint8_t server_volume = (uint8_t)((volume + 100) * AUDIO_MAX_VOLUME / 100);

    printf("server_volume: %d\n", ((server_volume * 100) / AUDIO_MAX_VOLUME));

    if (ep == AUDIO_IN_EP)
    {

        audio_server_set_public_volume(server_volume);
    }
    else if (ep == AUDIO_OUT_EP)
    {
        audio_server_set_private_volume(AUDIO_TYPE_LOCAL_MUSIC, server_volume);
    }
}

int usbd_audio_get_volume(uint8_t busid, uint8_t ep, uint8_t ch)
{
    (void)busid;
    (void)ch;

    uint8_t server_volume = 0;

    if (ep == AUDIO_IN_EP)
    {
        server_volume =
            audio_server_get_private_volume(AUDIO_TYPE_LOCAL_RECORD);
    }
    else if (ep == AUDIO_OUT_EP)
    {
        server_volume = audio_server_get_private_volume(AUDIO_TYPE_LOCAL_MUSIC);
    }

    int host_volume = (server_volume * 100) / AUDIO_MAX_VOLUME - 100;

    return host_volume;
}

void usbd_audio_set_mute(uint8_t busid, uint8_t ep, uint8_t ch, bool mute)
{
    (void)busid;
    (void)ch;
    if (ep == AUDIO_IN_EP)
    {
        audio_server_set_public_mic_mute(mute);
    }
    else if (ep == AUDIO_OUT_EP)
    {
        audio_server_set_public_speaker_mute(mute);
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
    else if (ep == AUDIO_OUT_EP)
    {
        return audio_server_get_public_speaker_mute();
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
    else if (ep == AUDIO_OUT_EP)
    {
        s_speaker_sample_rate = sampling_freq;
    }
}

uint32_t usbd_audio_get_sampling_freq(uint8_t busid, uint8_t ep)
{
    (void)busid;
    if (ep == AUDIO_IN_EP)
    {
        return s_mic_sample_rate;
    }
    else if (ep == AUDIO_OUT_EP)
    {
        return s_speaker_sample_rate;
    }
    return 0;
}

/**
 * @brief Record audio callback - sends audio data to USB in chunks
 *
 * @param cmd Callback command (as_callback_cmd_data_coming for new data)
 * @param callback_userdata User data pointer (unused)
 * @param reserved Pointer to audio_server_coming_data_t with audio data
 * @return 0 on success, -1 on timeout
 */
static int audio_callback_mic(audio_server_callback_cmt_t cmd,
                              void *callback_userdata, uint32_t reserved)
{
    if (cmd == as_callback_cmd_data_coming)
    {
        audio_server_coming_data_t *p = (audio_server_coming_data_t *)reserved;

        if (!mic_tx_flag)
        {
            return 0;
        }

        const uint32_t chunk_size = AUDIO_IN_PACKET;
        if (p->data_len % chunk_size == 0)
        {
            rt_kprintf(
                "[UAC Record] data_len %d is a multiple of chunk_size %d\n",
                p->data_len, chunk_size);
        }

        const uint32_t total_chunks = p->data_len / chunk_size;
        uint8_t *data_ptr = (uint8_t *)p->data;

        for (uint32_t i = 0; i < total_chunks; i++)
        {
            /* Send the current data chunk */
            int ret = usbd_ep_start_write(
                          0, AUDIO_IN_EP, data_ptr + (i * chunk_size), chunk_size);
            if (ret == 0)
            {
                rt_kprintf("[UAC Record] send chunk %d, ret=%d\n", i, ret);
            }

            rt_err_t result =
                rt_sem_take(&mic_tx_sem, rt_tick_from_millisecond(10));
            if (result != RT_EOK)
            {
                rt_kprintf("[UAC Record] Timed out waiting for TX completion "
                           "on chunk %d\n",
                           i);
                return -1;
            }
        }
    }

    return 0;
}

static int audio_callback_speaker(audio_server_callback_cmt_t cmd,
                                  void *callback_userdata, uint32_t reserved)
{
    return 0;
}

void usbd_audio_open(uint8_t busid, uint8_t intf)
{
    (void)busid;
    (void)intf;

    if (intf == 2)
    {
        // Interface 2 is the audio mic streaming interface
        rt_mb_send(&uac_mb, (rt_uint32_t)UAC_CMD_MIC_OPEN);
        rt_kprintf("UAC MIC OPEN command sent\n");
    }
    else if (intf == 1)
    {
        // Interface 1 is the audio speaker streaming interface
        rt_mb_send(&uac_mb, (rt_uint32_t)UAC_CMD_SPEAKER_OPEN);
        rt_kprintf("UAC SPEAKER OPEN command sent\n");
    }
    else
    {
        ;
    }
}

void usbd_audio_close(uint8_t busid, uint8_t intf)
{
    (void)busid;
    (void)intf;

    if (intf == 2)
    {
        // Send a CLOSE command to the audio task via mailbox
        rt_mb_send(&uac_mb, (rt_uint32_t)UAC_CMD_MIC_CLOSE);
        rt_kprintf("UAC MIC CLOSE command sent\n");
    }
    else if (intf == 1)
    {
        rt_mb_send(&uac_mb, (rt_uint32_t)UAC_CMD_SPEAKER_CLOSE);
        rt_kprintf("UAC SPEAKER CLOSE command sent\n");
    }
    else
    {
        ;
    }
}

/**
 * @brief Initialize audio device parameters.
 *
 * This function abstracts the initialization of audio device parameters
 * for both microphone and speaker.
 *
 * @param param Pointer to the audio_parameter_t structure to be filled.
 * @param type The type of audio device (e.g., AUDIO_TYPE_LOCAL_RECORD,
 * AUDIO_TYPE_LOCAL_MUSIC).
 * @param bits_per_sample The number of bits per audio sample.
 * @param channel_num The number of audio channels.
 * @param samplerate The audio sampling rate.
 * @param write_cache_size The size of the write cache.
 * @param read_cache_size The size of the read cache.
 * @return 0 on success, -1 on failure.
 */
static int audio_device_init(audio_parameter_t *param, uint8_t type,
                             uint8_t bits_per_sample, uint8_t channel_num,
                             uint32_t samplerate, uint32_t write_cache_size,
                             uint32_t read_cache_size)
{
    if (param == NULL)
    {
        return -1;
    }

    rt_memset(param, 0, sizeof(audio_parameter_t));

    param->write_bits_per_sample = bits_per_sample;
    param->write_channnel_num = channel_num;
    param->write_samplerate = samplerate;
    param->write_cache_size = write_cache_size;

    param->read_bits_per_sample = bits_per_sample;
    param->read_channnel_num = channel_num;
    param->read_samplerate = samplerate;
    param->read_cache_size = read_cache_size;

    return 0;
}

void audio_v1_init(uint8_t busid, uint32_t reg_base)
{
    // Initialize the semaphore for TX synchronization
    // Initial value is 0, max value is 1 (binary semaphore)
    rt_sem_init(&mic_tx_sem, "mic_tx_sem", 0, RT_IPC_FLAG_FIFO);
    rt_sem_init(&speaker_rx_sem, "speaker_rx_sem", 0, RT_IPC_FLAG_FIFO);

    // Initialize the mailbox for UAC commands
    rt_mb_init(&uac_mb, "uac_mb", &uac_mb_pool[0],
               sizeof(uac_mb_pool) / sizeof(rt_uint32_t), RT_IPC_FLAG_FIFO);

    usbd_desc_register(busid, &audio_v1_descriptor);
    usbd_add_interface(busid, usbd_audio_init_intf(busid, &intf0, AUDIO_VERSION,
                       audio_entity_table, 2));
    usbd_add_interface(busid, usbd_audio_init_intf(busid, &intf1, AUDIO_VERSION,
                       audio_entity_table, 2));
    usbd_add_interface(busid, usbd_audio_init_intf(busid, &intf2, AUDIO_VERSION,
                       audio_entity_table, 2));
    usbd_add_endpoint(busid, &audio_in_ep);
    usbd_add_endpoint(busid, &audio_out_ep);

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
            case UAC_CMD_MIC_OPEN:
                if (g_mic_client)
                {
                    rt_kprintf(
                        "[RECORD] already running, ignoring OPEN cmd.\n");
                    continue;
                }
                rt_kprintf("[RECORD] Received OPEN command. Starting...\n");
                mic_tx_flag = true;

                rt_sem_control(&mic_tx_sem, RT_IPC_CMD_RESET, RT_NULL);

                audio_parameter_t mic_param;
                audio_device_init(&mic_param, AUDIO_TYPE_LOCAL_RECORD, 16, 1,
                                  16000, 32, 320);

                g_mic_client = audio_open(AUDIO_TYPE_LOCAL_RECORD, AUDIO_RX,
                                          &mic_param, audio_callback_mic, NULL);
                if (!g_mic_client)
                {
                    rt_kprintf("[RECORD] Failed to open record client.\n");
                    mic_tx_flag = false;
                }
                else
                {
                    rt_kprintf("[RECORD] Record client opened successfully.\n");
                }
                break;

            case UAC_CMD_MIC_CLOSE:
                rt_kprintf("[RECORD] Received CLOSE command. Stopping...\n");
                mic_tx_flag = false;
                if (g_mic_client)
                {
                    audio_close(g_mic_client);
                    g_mic_client = 0;
                    rt_kprintf("[RECORD] Record client closed.\n");
                }
                break;

            case UAC_CMD_SPEAKER_OPEN:
                if (g_speaker_client)
                {
                    rt_kprintf(
                        "[SPEAKER] already running, ignoring OPEN cmd.\n");
                    continue;
                }
                rt_kprintf("[SPEAKER] Received OPEN command. Starting...\n");
                speaker_rx_flag = true;

                rt_sem_control(&speaker_rx_sem, RT_IPC_CMD_RESET, RT_NULL);

                audio_parameter_t speaker_param;
                audio_device_init(&speaker_param, AUDIO_TYPE_LOCAL_MUSIC, 16, 1,
                                  16000, AUDIO_OUT_PACKET, 320);

                g_speaker_client =
                    audio_open(AUDIO_TYPE_LOCAL_MUSIC, AUDIO_TX, &speaker_param,
                               audio_callback_speaker, NULL);

                if (g_speaker_client)
                {
                    rt_kprintf(
                        "[SPEAKER] Speaker client opened successfully.\n");
                    usbd_ep_start_read(busid, AUDIO_OUT_EP, g_speaker_buffer,
                                       AUDIO_OUT_PACKET);
                }
                else
                {
                    rt_kprintf("[SPEAKER] Failed to open speaker client.\n");
                    speaker_rx_flag = false;
                }

                break;

            case UAC_CMD_SPEAKER_CLOSE:
                rt_kprintf("[SPEAKER] Received CLOSE command. Stopping...\n");
                speaker_rx_flag = false;
                if (g_speaker_client)
                {
                    audio_close(g_speaker_client);
                    g_speaker_client = 0;
                    rt_kprintf("[SPEAKER] Speaker client closed.\n");
                }
                break;

            default:
                rt_kprintf("[UAC] Received unknown command: %d\n", cmd);
                break;
            }
        }
    }
}

/* static function definition */
static void usbd_audio_in_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)ep;
    (void)nbytes;

    // This callback is in an interrupt context or a high-priority USB task.
    // Release the semaphore to signal that the transfer is complete.
    rt_sem_release(&mic_tx_sem);
}

static void usbd_audio_out_callback(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)ep;
    (void)nbytes;

    if (g_speaker_client)
    {
        audio_write(g_speaker_client, g_speaker_buffer, nbytes);
    }

    usbd_ep_start_read(busid, AUDIO_OUT_EP, g_speaker_buffer, AUDIO_OUT_PACKET);
}
