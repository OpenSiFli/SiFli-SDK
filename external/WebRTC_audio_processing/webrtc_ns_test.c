#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "rtconfig.h"


#define FRAME_LENGTH_MAX 160
#define INPUT_ADDRESS 0x60000000 
#define OUTPUT_ADDRESS 0x60400000
static uint32_t fs;
static uint32_t data_len;
static uint16_t frame_len;
static uint8_t nMode;// 0, 1, 2, 3

#ifdef WEBRTC_ANS_FLOAT
#include "webrtc/modules/audio_processing/ns/include/noise_suppression.h"

NsHandle *pNS_inst = NULL;

void nstest_thread(void *p)
{
    uint32_t i, j, framenum; 
    int16_t *datain;
    int16_t *dataout;

    float spframe[FRAME_LENGTH_MAX];
    float outframe[FRAME_LENGTH_MAX];
    //const float* const* spframe_p = (const float* const*)&spframe[0];
    float *spframe_p[1] = {&spframe[0]};
    float *outframe_p[1] = {&outframe[0]};
    uint32_t time_tick1, time_tick2, tick_sum=0;
    float tick_per_frame;
    
    datain = (int16_t*)INPUT_ADDRESS;
    dataout = (int16_t*)OUTPUT_ADDRESS;
    framenum = (data_len/frame_len) >> 1;// one sample have 2 byte , last data may not enought

    rt_kprintf("nstest_thread para,fs:%d, ns_mode:%d, datalen=0x%x, framenum=%d\n", fs, nMode, data_len, framenum);

    pNS_inst = WebRtcNs_Create();
    if (pNS_inst != NULL)
    {
        if (0 != WebRtcNs_Init(pNS_inst, fs))
        {
            rt_kprintf("WebRtcNs_Init error !\n");
        }
        rt_kprintf("WebRtcNs_Init finish !\n");
        
        if (0 != WebRtcNs_set_policy(pNS_inst, nMode))
        {
            rt_kprintf("WebRtcNs_set_policy error !\n");
        }
        rt_kprintf("WebRtcNs_set_policy finish !\n");

        rt_kprintf("WebRtcNs Analyze process.......\n");
        tick_sum = 0;
        for (i = 0; i < framenum; i ++) 
        {
            
            time_tick1 = rt_tick_get();


            for (j = 0; j < frame_len; j++)
            {
                spframe[j] = (float)(datain[i*frame_len + j]);
            }

            WebRtcNs_Analyze(pNS_inst, spframe);

            WebRtcNs_Process(pNS_inst, (const float* const*)spframe_p, 1, outframe_p);

            for (j = 0; j < frame_len; j++)
                dataout[i*frame_len + j] = (int16_t)outframe[j];
            
            time_tick2 = rt_tick_get();
            tick_sum = tick_sum + (time_tick2 - time_tick1);
                
        }

        tick_per_frame = (float)tick_sum/framenum;
        
        rt_kprintf("WebRtcNs_data finish tick:%f!\n", tick_per_frame);
        
        WebRtcNs_Free(pNS_inst);
        rt_kprintf("WebRtcNs_Free finish !\n");
    }
    else
    {
        rt_kprintf("no memory !\n");
    }


}
#else

#include "webrtc/modules/audio_processing/ns/include/noise_suppression_x.h"

NsxHandle *pNS_inst = NULL;

void nstest_thread(void *p)
{
    uint32_t i, j, framenum; 
    int16_t *datain;
    int16_t *dataout;

    int16_t spframe[FRAME_LENGTH_MAX];
    int16_t outframe[FRAME_LENGTH_MAX];
    int16_t *spframe_p[1] = {&spframe[0]};
    int16_t *outframe_p[1] = {&outframe[0]};
    uint32_t time_tick1, time_tick2, tick_sum=0;
    float tick_per_frame;
    
    datain = (int16_t*)INPUT_ADDRESS;
    dataout = (int16_t*)OUTPUT_ADDRESS;
    framenum = (data_len/frame_len) >> 1;// one sample have 2 byte , last data may not enought

    rt_kprintf("nstest_thread para,fs:%d, ns_mode:%d, datalen=0x%x, framenum=%d\n", fs, nMode, data_len, framenum);

    pNS_inst = WebRtcNsx_Create();
    if (pNS_inst != NULL)
    {
        if (0 != WebRtcNsx_Init(pNS_inst, fs))
        {
            rt_kprintf("fix WebRtcNs_Init error !\n");
        }
        rt_kprintf("fix WebRtcNs_Init finish !\n");
        
        if (0 != WebRtcNsx_set_policy(pNS_inst, nMode))
        {
            rt_kprintf("fix WebRtcNs_set_policy error !\n");
        }
        rt_kprintf("fix WebRtcNs_set_policy finish !\n");

        rt_kprintf("fix WebRtcNs Analyze process.......\n");
        tick_sum = 0;
        for (i = 0; i < framenum; i ++) 
        {
            
            time_tick1 = rt_tick_get();


            for (j = 0; j < frame_len; j++)
            {
                spframe[j] = (datain[i*frame_len + j]);
            }


            WebRtcNsx_Process(pNS_inst, (const int16_t* const*)spframe_p, 1, outframe_p);

            for (j = 0; j < frame_len; j++)
                dataout[i*frame_len + j] = (int16_t)outframe[j];
            
            time_tick2 = rt_tick_get();
            tick_sum = tick_sum + (time_tick2 - time_tick1);
                
        }

        tick_per_frame = (float)tick_sum/framenum;
        
        rt_kprintf("fix WebRtcNs_data finish tick:%f!\n", tick_per_frame);
        
        WebRtcNsx_Free(pNS_inst);
        rt_kprintf("fix WebRtcNs_Free finish !\n");
    }
    else
    {
        rt_kprintf("no memory !\n");
    }


}

#endif

int nstest(int argc, char *argv[])
{
    rt_thread_t thread;
    
    if (argc != 4)
    {
        rt_kprintf("nstest para,fs, ns_mode, datalen\n");
        rt_kprintf("arg para num error, argc:%d !\n", argc);
        return -1;
    }
    
    if ((strcmp(argv[1], "8k") == 0)||(strcmp(argv[1], "8000") == 0))
    {
        fs = 8000;
        frame_len = 80;
    }
    else if((strcmp(argv[1], "16k") == 0)||(strcmp(argv[1], "16000") == 0))
    {
        fs = 16000;
        frame_len = 160;
    }
    else
    {
        rt_kprintf("arg para 1 error, samplerate:%s !\n", argv[1]);
    }

    if((strcmp(argv[2], "0") == 0)||(strcmp(argv[2], "1") == 0)||(strcmp(argv[2], "2") == 0)||(strcmp(argv[2], "3") == 0))
    {
        nMode = strtol(argv[2],NULL, 16);
    }
    else
    {
        rt_kprintf("arg para 2 error, nsmode:%s !\n", argv[2]);
    }

    data_len = strtol(argv[3],NULL, 16);

    rt_kprintf("nstest para,fs:%d, ns_mode:%d, datalen=0x%x\n", fs, nMode, data_len);


    
    thread = rt_thread_create("nstest", nstest_thread, NULL, 16000, 14, 10);

    rt_kprintf("thread=%d\n", thread);
    rt_thread_mdelay(100);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    
    return 0;
}

MSH_CMD_EXPORT(nstest, webrtc ns test);
