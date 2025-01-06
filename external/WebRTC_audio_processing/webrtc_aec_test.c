#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "rtconfig.h"

#define FRAME_LENGTH_MAX 160
#define INPUT_ADDRESS  0x60000000 
#define INPUT2_ADDRESS 0x60200000 
#define OUTPUT_ADDRESS 0x60400000
static uint32_t fs;
static uint32_t data_len;
static uint16_t frame_len;
static uint8_t nMode;
static uint16_t delay;

#ifdef WEBRTC_AEC
#include "webrtc/modules/audio_processing/aec/include/echo_cancellation.h"
static void *aecInst = NULL;

void aectest_thread(void *p)
{
    uint32_t i, j, framenum; 
    int16_t *datain;
    int16_t *datain2;
    int16_t *dataout;

    uint32_t time_tick1, time_tick2, tick_sum=0;
    float tick_per_frame;
    
    datain = (int16_t*)INPUT_ADDRESS;
    datain2 = (int16_t*)INPUT2_ADDRESS;
    dataout = (int16_t*)OUTPUT_ADDRESS;
    framenum = (data_len/frame_len) >> 1;// one sample have 2 byte , last data may not enought

    rt_kprintf("aectest_thread para,fs:%d, mode:%d, datalen=0x%x, framenum=%d\n", fs, nMode, data_len, framenum); 
    
    aecInst = WebRtcAec_Create();
    
    if (aecInst != NULL)
    {
        AecConfig config;
        config.nlpMode = nMode;//kAecNlpAggressive  0 1 2(default)
        config.skewMode = kAecFalse;
        config.metricsMode = kAecFalse;
        config.delay_logging = kAecFalse;

        if (0 != WebRtcAec_Init(aecInst, fs, fs))
        {
           rt_kprintf("WebRtcAec_Init para error !\n"); 
           return;
        }
        rt_kprintf("WebRtcAec_Init finish !\n");
        
        if (0 != WebRtcAec_set_config(aecInst, config))
        {
            rt_kprintf("WebRtcAec_set_config para error !\n");
            return;
        }
        rt_kprintf("WebRtcAec_set_config finish !\n");

        rt_kprintf("WebRtcAec Analyze process.......\n");
        tick_sum = 0;
        

        float farFrame[FRAME_LENGTH_MAX];
        float nearFrame[FRAME_LENGTH_MAX];
        float outFrame[FRAME_LENGTH_MAX];
        float *pNearFrame[1] = {&nearFrame[0]};
        float *pOutFrame[1] = {&outFrame[0]};
    
        for (i = 0; i < framenum; i ++) 
        {
            
            time_tick1 = rt_tick_get();


            for (j = 0; j < frame_len; j++)
            {
                farFrame[j] = (float)(datain2[i*frame_len + j]);
                nearFrame[j] = (float)(datain[i*frame_len + j]);
            }
            
            if (0 != WebRtcAec_BufferFarend(aecInst, farFrame, frame_len))
            {
                rt_kprintf("WebRtcAec_BufferFarend  error !\n");
                return;
            }
            
            if (0 != WebRtcAec_Process(aecInst, (const float* const*)pNearFrame, 1, pOutFrame, frame_len, delay, 0))
            {
                rt_kprintf("WebRtcAec_Process  error !\n");
                return;
            }
            
            for (j = 0; j < frame_len; j++)
            {
                dataout[i*frame_len + j] = (int16_t)outFrame[j];
            }
            
            time_tick2 = rt_tick_get();
            tick_sum = tick_sum + (time_tick2 - time_tick1);
            
        }

        tick_per_frame = (float)tick_sum/framenum;
        
        rt_kprintf("WebRtcAec_data finish tick:%f!\n", tick_per_frame);
        
        WebRtcAec_Free(aecInst);
        rt_kprintf("WebRtcAec_Free finish !\n");
    }
    else
    {
        rt_kprintf("no memory !\n");
    }


}
#else
#include "webrtc/modules/audio_processing/aecm/include/echo_control_mobile.h"
static void *aecmInst = NULL;

void aectest_thread(void *p)
{
    uint32_t i, j, framenum; 
    int16_t *datain;
    int16_t *datain2;
    int16_t *dataout;

    uint32_t time_tick1, time_tick2, tick_sum=0;
    float tick_per_frame;
    
    datain = (int16_t*)INPUT_ADDRESS;
    datain2 = (int16_t*)INPUT2_ADDRESS;
    dataout = (int16_t*)OUTPUT_ADDRESS;
    framenum = (data_len/frame_len) >> 1;// one sample have 2 byte , last data may not enought

    rt_kprintf("aectest_thread para,fs:%d, mode:%d, datalen=0x%x, framenum=%d\n", fs, nMode, data_len, framenum); 
    
    aecmInst = WebRtcAecm_Create();
    
    if (aecmInst != NULL)
    {
        AecmConfig config;
        config.cngMode = AecmTrue;
        config.echoMode = nMode;//0 1 2 3 4

        if (0 != WebRtcAecm_Init(aecmInst, fs))
        {
            rt_kprintf("WebRtcAecm_Init para error !\n");
            return;
        }
        rt_kprintf("WebRtcAecm_Init finish !\n");
        
        if (0 != WebRtcAecm_set_config(aecmInst, config))
        {
           rt_kprintf("WebRtcAecm_set_config para error !\n"); 
        }
        rt_kprintf("WebRtcAecm_set_config finish !\n");

        rt_kprintf("WebRtcAecm Analyze process.......\n");
        tick_sum = 0;
        

        int16_t farFrame[FRAME_LENGTH_MAX];
        int16_t nearFrame[FRAME_LENGTH_MAX];
        int16_t outFrame[FRAME_LENGTH_MAX];
    
        for (i = 0; i < framenum; i ++) 
        {
            
            time_tick1 = rt_tick_get();


            for (j = 0; j < frame_len; j++)
            {
                farFrame[j] = (datain2[i*frame_len + j]);
                nearFrame[j] = (datain[i*frame_len + j]);
            }               
            
            if(0 != WebRtcAecm_BufferFarend(aecmInst, farFrame, frame_len))
            {
                rt_kprintf("WebRtcAecm_BufferFarend  error !\n");
                return;
            }
            
            if(0 != WebRtcAecm_Process(aecmInst, nearFrame, NULL, outFrame, frame_len, delay))
            {
                rt_kprintf("WebRtcAecm_Process  error !\n");
                return;
            }
            
            for (j = 0; j < frame_len; j++)
            {
                dataout[i*frame_len + j] = outFrame[j];
            }
            
            time_tick2 = rt_tick_get();
            tick_sum = tick_sum + (time_tick2 - time_tick1);
            
        }

        tick_per_frame = (float)tick_sum/framenum;
        
        rt_kprintf("WebRtcAecm_data finish tick:%f!\n", tick_per_frame);
        
        WebRtcAecm_Free(aecmInst);
        rt_kprintf("WebRtcAecm_Free finish !\n");
    }
    else
    {
        rt_kprintf("no memory !\n");
    }


}
#endif

int aectest(int argc, char *argv[])
{
    rt_thread_t thread;
    
    if (argc != 5)
    {
        rt_kprintf("aectest para,fs, mode, delay, datalen\n");
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

    if((strcmp(argv[2], "0") == 0)||(strcmp(argv[2], "1") == 0)||(strcmp(argv[2], "2") == 0)||(strcmp(argv[2], "3") == 0)||(strcmp(argv[2], "4") == 0))
    {
        nMode = strtol(argv[2],NULL, 16);
    }
    else
    {
        rt_kprintf("arg para 2 error, nsmode:%s !\n", argv[2]);
    }
    
    delay = strtol(argv[3],NULL, 16);
    data_len = strtol(argv[4],NULL, 16);

    rt_kprintf("aectest para,fs:%d, mode:%d, delay:%d, datalen=0x%x\n", fs, nMode, delay, data_len);


    
    thread = rt_thread_create("aectest", aectest_thread, NULL, 16000, 14, 10);

    rt_kprintf("thread=%d\n", thread);
    rt_thread_mdelay(100);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    
    return 0;
}

MSH_CMD_EXPORT(aectest, webrtc aec test);


