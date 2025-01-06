#include "sndp_lib.h"
#include "Soundplus_adapter.h"
#include "SpEcCfgParaSt.h"

#define FRMLEN      (240)

static bool soundplus_inited = false;
static bool soundplus_rx_inited = false;
static bool soundplus_authed = false;
static void *pBuff_TX = NULL;
static float *pMic_In = NULL;
static float *pRef_In = NULL;
static float *pBuff_RX = NULL;
static float *pRef_RX = NULL;

int soundplus_auth(char *key_buf, int key_len)
{
    int ret = 0;

    if (soundplus_authed == false)
    {
        //ret = sndp_license_auth(key_buf, key_len);
        soundplus_authed = true;
    }

    return ret;
}

int soundplus_auth_status(void)
{
    //return sndp_license_status_get();
    return 0;
}

int soundplus_deal_Tx(short *buf, short *ref, int buf_len, int ref_len)
{
    int i;
    float tmpAbs;
    if (ref)
    {
        for (i = 0; i < buf_len; i++)
        {
            pMic_In[i] = buf[i] * INPUT_GAIN_TALK;
        }
        for (i = 0; i < ref_len; i++)
        {
            pRef_In[i] = ref[i];
        }

        Sndp_SpxEnh_Tx(pMic_In, pMic_In, pRef_In, buf_len, ref_len);
    }
    else
    {
        for (i = 0; i < buf_len; i++)
        {
            pMic_In[i] = buf[i] * INPUT_GAIN_TALK;
        }
        Sndp_SpxEnh_Tx(pMic_In, pMic_In, NULL, buf_len, ref_len);
    }

    for (i = 0; i < buf_len; i++)
    {
        tmpAbs = pMic_In[i];
        if (tmpAbs < -32768) tmpAbs = -32768;
        if (tmpAbs > 32767) tmpAbs = 32767;
        buf[i] = (short)tmpAbs;
    }

    return 0;
}

int soundplus_deal_Rx(short *out, short *ref, int ref_len)
{
    int i;
    float tmpAbs;

    for (i = 0; i < ref_len; i++)
    {
        pRef_RX[i] = ref[i];
    }

    Sndp_SpxEnh_Rx(pRef_RX, pRef_RX, ref_len);

    for (i = 0; i < ref_len; i++)
    {
        tmpAbs = pRef_RX[i];
        if (tmpAbs < -32768) tmpAbs = -32768;
        if (tmpAbs > 32767) tmpAbs = 32767;
        out[i] = (short)tmpAbs;
    }

    return 0;
}

int soundplus_init(int typeR)
{
    if (soundplus_inited)
    {
        return -1;
    }
    int memsize = Sndp_SpxEnh_MemSize(typeR);

    pBuff_TX = (void *)malloc(memsize);
    if (pBuff_TX == NULL)
    {
        return -1;
    }
    //printf("[%s] ,start:%p  memsize=%d version=%s", __func__, pBuff_TX, memsize, get_Sndp_alg_ver());
    pMic_In = (float *)malloc(FRMLEN * sizeof(float));
    if (pMic_In == NULL)
    {
        return -1;
    }

    if (typeR != 0)
    {
        pRef_In = (float *)malloc(FRMLEN * sizeof(float));
        if (pRef_In == NULL)
        {
            return -1;
        }
    }
    else
    {
        pRef_In = (float *)malloc(FRMLEN * sizeof(float) * 3);
        if (pRef_In == NULL)
        {
            return -1;
        }
    }

    Sndp_SpxEnh_Init(pBuff_TX);
    soundplus_inited = true;

    return 0;
}

void soundplus_deinit(void)
{
    if (pBuff_TX)
    {
        free((void *)pBuff_TX);
        pBuff_TX = NULL;
    }
    if (pMic_In)
    {
        free((float *)pMic_In);
        pMic_In = NULL;
    }
    if (pRef_In)
    {
        free((float *)pRef_In);
        pRef_In = NULL;
    }
    soundplus_inited = false;
}

int soundplus_rx_init(int typeR)
{
    if (soundplus_rx_inited)
    {
        return -1;
    }
    int rx_memsize = Sndp_Rx_MemSize(typeR);
    pBuff_RX = (void *)malloc(rx_memsize);
    if (pBuff_RX == NULL)
    {
        return -1;
    }
    pRef_RX = (float *)malloc(FRMLEN * sizeof(float));
    if (pRef_RX == NULL)
    {
        return -1;
    }

    Sndp_Rx_Init(pBuff_RX);
    soundplus_rx_inited = true;

    return 0;
}

void soundplus_rx_deinit(void)
{
    if (pBuff_RX)
    {
        free((void *)pBuff_RX);
        pBuff_RX = NULL;
    }
    if (pRef_RX)
    {
        free((float *)pRef_RX);
        pRef_RX = NULL;
    }

    soundplus_rx_inited = false;
}

