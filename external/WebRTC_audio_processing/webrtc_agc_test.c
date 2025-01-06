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



#include "webrtc/modules/audio_processing/agc/legacy/gain_control.h"

static uint8_t agcMode = kAgcModeFixedDigital;// 0, 1, 2, 3  kAgcModeAdaptiveDigital   kAgcModeFixedDigital
static void *agcInst = NULL;


void agctest_thread(void *p)
{
    int32_t micLevelIn = 0;
    int32_t micLevelOut = 0;
    uint8_t saturationWarning;
    int32_t minLevel = 0;
    int32_t maxLevel = 255;

    WebRtcAgcConfig agcConfig;
    agcConfig.compressionGaindB = 20;
    agcConfig.limiterEnable = 1;
    agcConfig.targetLevelDbfs = 3;
    
        
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

    rt_kprintf("agctest_thread para,fs:%d, agcMode:%d, datalen=0x%x, framenum=%d\n", fs, agcMode, data_len, framenum);

    agcInst = WebRtcAgc_Create();
    if (agcInst != NULL)
    {
        if (0 != WebRtcAgc_Init(agcInst, minLevel, maxLevel, agcMode, fs))
        {
            rt_kprintf("WebRtcAgc_Init para error !\n");
        }
        rt_kprintf("WebRtcAgc_Init finish !\n");
        
        if (0 != WebRtcAgc_set_config(agcInst, agcConfig))
        {
            rt_kprintf("WebRtcAgc_set_config para error !\n");
        }
        rt_kprintf("WebRtcAgc_set_config finish !\n");

        rt_kprintf("WebRtcAgc is processing.......\n");
        tick_sum = 0;
        for (i = 0; i < framenum; i ++) 
        {
            
            time_tick1 = rt_tick_get();


            for (j = 0; j < frame_len; j++) {
                spframe[j] = (datain[i*frame_len + j]);
            }

            if (0 != WebRtcAgc_Process(agcInst, (const int16_t* const*)spframe_p, 1, frame_len, (int16_t* const*)outframe_p, micLevelIn, &micLevelOut, 0, &saturationWarning))
            {
                rt_kprintf("WebRtcAgc_Process error !\n");
            }
            micLevelIn = micLevelOut;

            for (j = 0; j < frame_len; j++)
                dataout[i*frame_len + j] = (int16_t)outframe[j];
            
            time_tick2 = rt_tick_get();
            tick_sum = tick_sum + (time_tick2 - time_tick1);
                
        }

        tick_per_frame = (float)tick_sum/framenum;

        rt_kprintf("micLevelIn:%ld, micLevelOut=%d!\n", micLevelIn, micLevelOut);
        
        rt_kprintf("WebRtcAgc_data finish tick:%f!\n", tick_per_frame);
        
        WebRtcAgc_Free(agcInst);
        rt_kprintf("WebRtcAgc_Free finish !\n");
    }
    else
    {
        rt_kprintf("no memory !\n");
    }


}


int agctest(int argc, char *argv[])
{
    rt_thread_t thread;
    
    if (argc != 4)
    {
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
        agcMode = strtol(argv[2],NULL, 16);
    }
    else
    {
        rt_kprintf("arg para 2 error, nsmode:%s !\n", argv[2]);
    }

    data_len = strtol(argv[3],NULL, 16);

    rt_kprintf("nstest para,fs:%d, agc_mode:%d, datalen=0x%x\n", fs, agcMode, data_len);


    
    thread = rt_thread_create("agctest", agctest_thread, NULL, 16000, 14, 10);

    rt_kprintf("thread=%d\n", thread);
    rt_thread_mdelay(100);
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    
    return 0;
}

MSH_CMD_EXPORT(agctest, webrtc agc test);




