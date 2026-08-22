/*
 * All Intellectual Property rights in the software belongs to sakumisu.
 *
 *   Licensing information
 *   ---------------------
 *
 *   Licensor:                 sakumisu
 *   Licensed to:
 *   License software version: cherryusb musb host uvc&uac v2.3
 *   Licensed platform:
 */

#ifndef USBH_UAC_STREAM_H
#define USBH_UAC_STREAM_H

#include "usbh_core.h"
#include "usbh_audio.h"

struct usbh_audioframe {
    uint8_t *frame_buf;
    uint32_t frame_bufsize;
    uint32_t frame_size;
};

#define AUDIO_MIC_ISO_PACKETS  (1)
#define AUDIO_MIC_EP_MAX_MPS   1024

#define AUDIO_SPEAKER_ISO_PACKETS  (1)
#define AUDIO_SPEAKER_EP_MAX_MPS   1024

extern volatile uint32_t audio_mic_complete_count;

int usbh_audio_mic_stream_create(struct usbh_audioframe *frame, uint32_t count);
int usbh_audio_mic_stream_start(uint32_t freq);
void usbh_audio_mic_stream_stop(void);
int usbh_audio_mic_stream_enqueue(struct usbh_audioframe *frame);
int usbh_audio_mic_stream_dequeue(struct usbh_audioframe **frame, uint32_t timeout);

int usbh_audio_speaker_stream_create(struct usbh_audioframe *frame, uint32_t count);
int usbh_audio_speaker_stream_start(uint32_t freq);
void usbh_audio_speaker_stream_stop(void);
struct usbh_audioframe *usbh_uac_speaker_frame_alloc(void);
int usbh_uac_speaker_frame_send(struct usbh_audioframe *frame);

#endif
