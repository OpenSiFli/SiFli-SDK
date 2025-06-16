#include <rtthread.h>

#ifdef SOC_BF0_HCPU

#include <string.h>
#include <stdlib.h>
#include <rtdevice.h>
#include <drv_config.h>
#include "board.h"
#include "audioproc.h"
#include "audio_server.h"
#include "audio_mem.h"
#include "drivers/audio.h"


#define WAIT_PA_STABLE_TIME_MS      200

/* some board stop PA has pop noise, need delay some time than close DAC*/
#define PA_CLOSE_DELAY_MS           10

#ifdef PA_USING_AW87390
#include "sifli_aw87390.h"
void audio_hardware_pa_init()
{
    sifli_aw87390_init();
}

void audio_hardware_pa_start(uint32_t samplerate, uint32_t reserved)
{
    (void)samplerate;
    (void)reserved;
    sifli_aw87390_start();
    rt_thread_mdelay(WAIT_PA_STABLE_TIME_MS);
}
void audio_hardware_pa_stop(void)
{
    sifli_aw87390_stop();
    if (PA_CLOSE_DELAY_MS > 0)
    {
        rt_thread_mdelay(PA_CLOSE_DELAY_MS);
    }
}

#elif defined(PA_USING_AW8155)

#include "sifli_aw8155.h"
void audio_hardware_pa_init(void)
{
}
void audio_hardware_pa_start(uint32_t samplerate, uint32_t reserved)
{
    (void)samplerate;
    (void)reserved;
    sifli_aw8155_start();
    rt_thread_mdelay(WAIT_PA_STABLE_TIME_MS);
}
void audio_hardware_pa_stop(void)
{
    sifli_aw8155_stop();
    if (PA_CLOSE_DELAY_MS > 0)
    {
        rt_thread_mdelay(PA_CLOSE_DELAY_MS);
    }

}

#elif defined(PA_USING_AW882XX)
#include "sifli_aw882xx.h"
void audio_hardware_pa_init(void)
{
    rt_aw882xx_init();
}
void audio_hardware_pa_start(uint32_t samplerate, uint32_t reserved)
{
    (void)reserved;
    sifli_aw882xx_start(samplerate, 3);
    rt_thread_mdelay(WAIT_PA_STABLE_TIME_MS);
}
void audio_hardware_pa_stop(void)
{
    sifli_aw882xx_stop();
    if (PA_CLOSE_DELAY_MS > 0)
    {
        rt_thread_mdelay(PA_CLOSE_DELAY_MS);
    }

}
#else

RT_WEAK void audio_hardware_pa_init()
{

}
RT_WEAK void audio_hardware_pa_start(uint32_t samplerate, uint32_t reserved)
{

}
RT_WEAK void audio_hardware_pa_stop(void)
{

}

#endif

#endif //SOC_BF0_HCPU
