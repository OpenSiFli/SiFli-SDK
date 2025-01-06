#ifndef _SOUNDPLUS_DM_GSC_CM4_H
#define _SOUNDPLUS_DM_GSC_CM4_H

#include <rtthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "audio_mem.h"

#undef assert
#undef malloc
#undef free

#define assert  RT_ASSERT
#define malloc  audio_mem_malloc
#define free    audio_mem_free

#ifdef __cplusplus
extern "C" {
#endif

int sndp_license_auth(char *license_key, int Key_len);
int sndp_license_status_get(void);
char *get_Sndp_alg_ver(void);
int Sndp_SpxEnh_MemSize(int type_result);
int Sndp_Rx_MemSize(int type_result);
void Sndp_SpxEnh_Init(void *membuf);
void Sndp_Rx_Init(void *membuf);
void Sndp_SpxEnh_Tx(float *outY, float *inX, float *ref, int numsamples_mic, int numsamples_ref);
void Sndp_SpxEnh_Rx(float *outY, float *inX, int numsamples_mic);

#ifdef __cplusplus
}
#endif

#endif
