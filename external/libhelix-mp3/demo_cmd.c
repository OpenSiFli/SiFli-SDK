#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include <rtthread.h>

#include "mp3dec.h"


#define DEBUG_OUT_SIZE (1*1024*1024)

extern void *app_anim_mem_alloc(rt_size_t size, bool anim_data);
extern void app_anim_mem_free(void *p);

static uint8_t *g_out;
static uint8_t *g_out_start;

#include "test_mp3.data"

int mp3_decode_real()
{
	int bytesLeft,  err, offset, outOfData, eofReached;
	unsigned char *readPtr = (unsigned char *)&mp3_data[0];
	short outBuf[MAX_NCHAN * MAX_NGRAN * MAX_NSAMP];
	MP3FrameInfo mp3FrameInfo;
	HMP3Decoder hMP3Decoder;
	int nFrames;

    g_out = app_anim_mem_alloc(64*1024, 1);
    g_out_start = g_out;

	if ( (hMP3Decoder = MP3InitDecoder()) == 0 )
		return -2;


	bytesLeft = sizeof(mp3_data);
	outOfData = 0;

	nFrames = 0;
    rt_tick_t t0 = rt_tick_get_millisecond();
	do {
		/* somewhat arbitrary trigger to refill buffer - should always be enough for a full frame */
		if (bytesLeft < MAINBUF_SIZE) {
            eofReached = 1;
            break;
		}

		/* find start of next MP3 frame - assume EOF if no sync found */
		offset = MP3FindSyncWord(readPtr, bytesLeft);
		if (offset < 0) {
			outOfData = 1;
			break;
		}
		readPtr += offset;
		bytesLeft -= offset;


		/* decode one MP3 frame - if offset < 0 then bytesLeft was less than a full frame */

 		err = MP3Decode(hMP3Decoder, &readPtr, &bytesLeft, outBuf, 0);
 		nFrames++;
 		
		if (err) {
			/* error occurred */
			switch (err) {
			case ERR_MP3_INDATA_UNDERFLOW:
				outOfData = 1;
				break;
			case ERR_MP3_MAINDATA_UNDERFLOW:
				/* do nothing - next call to decode will provide more mainData */
				break;
			case ERR_MP3_FREE_BITRATE_SYNC:
			default:
				outOfData = 1;
				break;
			}
		} else {
			/* no error */
			MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);

			if (g_out && (g_out + mp3FrameInfo.outputSamps * mp3FrameInfo.bitsPerSample / 8 - g_out_start) <  DEBUG_OUT_SIZE)
            {
                memcpy(g_out, outBuf, mp3FrameInfo.bitsPerSample / 8 * mp3FrameInfo.outputSamps);
                g_out += mp3FrameInfo.bitsPerSample / 8 * mp3FrameInfo.outputSamps;
            }
            
		}

	} while (!outOfData);

    rt_tick_t t1 = rt_tick_get_millisecond();

	MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);

	rt_kprintf("\noutput samplerate=%dK\n,start=0x%x, end=0x%x\n", nFrames * mp3FrameInfo.outputSamps/ mp3FrameInfo.nChans / (t1 - t0), g_out_start, g_out);
	rt_kprintf("nFrames=%d, output samps=%d, sampRate=%d, nChans=%d bitsPerSample=%d\n", nFrames, mp3FrameInfo.outputSamps, mp3FrameInfo.samprate, mp3FrameInfo.nChans, mp3FrameInfo.bitsPerSample);

	MP3FreeDecoder(hMP3Decoder);
    if (g_out_start)
        app_anim_mem_free(g_out_start);
	return 0;
}

static void mp3_decode(void *p)
{
    mp3_decode_real();
}
int mp3test(int argc, char *argv[])
{
    rt_thread_t thread = rt_thread_create("mp3", mp3_decode, NULL, 64000, 14, 10);
    rt_kprintf("thread=%d\n", thread);
    rt_thread_mdelay(100);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    return 0;
}
MSH_CMD_EXPORT(mp3test, mp3 test using libhelix);