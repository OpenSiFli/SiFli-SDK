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

#ifndef USBH_UVC_STREAM_H
#define USBH_UVC_STREAM_H

#include "usbh_core.h"
#include "usbh_video.h"

#define VIDEO_ISO_PACKETS  (10 * 1)
#define VIDEO_EP_MAX_MPS   1024

extern volatile uint32_t video_complete_count;

/* implemented by user */
void usbh_video_fps_record(void);
void usbh_video_transfer_abort_callback(void);

/* implemented by user */
void usbh_video_dma_init(void);
void usbh_video_dma_lli_fill(uint32_t desc_index, uint32_t src_addr, uint32_t dst_addr, uint32_t nbytes);
void usbh_video_dma_start(void);
void usbh_video_dma_stop(void);
bool usbh_video_dma_isbusy(void);

int usbh_video_stream_create(struct usbh_videoframe *frame, uint32_t count);
int usbh_video_stream_start(uint16_t width, uint16_t height, uint8_t format);
void usbh_video_stream_stop(void);
int usbh_video_stream_enqueue(struct usbh_videoframe *frame);
int usbh_video_stream_dequeue(struct usbh_videoframe **frame, uint32_t timeout);

#endif
