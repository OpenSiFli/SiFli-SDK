/*************************************************************
SBC Example PLC ANSI-C Source Code
File: sbcplc.c
*************************************************************/
#include <math.h>
//#include "sbc.h"
#include "audio_msbc_plc.h"
#include "assert.h"
#include "stdlib.h"

static void g711plc_scalespeech(LowcFE_c *, short *out);
static void g711plc_getfespeech(LowcFE_c *, short *out, int sz);
static void g711plc_savespeech(LowcFE_c *, short *s);
static int g711plc_findpitch(LowcFE_c *);
static void g711plc_overlapadd(Float *l, Float *r, Float *o, int cnt);
static void g711plc_overlapadds(short *l, short *r, short *o, int cnt);
static void g711plc_overlapaddatend(LowcFE_c *, short *s, short *f, int cnt);
static void g711plc_convertsf(short *f, Float *t, int cnt);
static void g711plc_convertfs(Float *f, short *t, int cnt);
static void g711plc_copyf(Float *f, Float *t, int cnt);
static void g711plc_copys(short *f, short *t, int cnt);
static void g711plc_zeros(short *s, int cnt);

void msbc_g711plc_construct(LowcFE_c *lc)
{
    lc->pitch_min = 40 * 2;      /* minimum allowed pitch, 200 Hz */
    lc->pitch_max = 120 * 2;     /* maximum allowed pitch, 66 Hz */
    lc->pitchdiff = (lc->pitch_max - lc->pitch_min);
    lc->poverlapmax = (lc->pitch_max >> 2);        /* maximum pitch ola window */
    lc->historylen = (lc->pitch_max * 3 + lc->poverlapmax);   /* history buffer length */
    lc->ndec = 2;       /* 2:1 decimation */
    lc->corrlen = 160 * 2;     /* 20 msec correlation length */
    lc->corrbuflen = (lc->corrlen + lc->pitch_max);   /* correlation buffer length */
    lc->corrminpower = ((float)250. * 2);   /* minimum power */

#if 1
    lc->eoverlapincr = 24 * 2;      /* end ola increment per frame, 3ms */
    lc->framesz = 60 * 2;      /* 7.5 msec at 8khz */
#else
    lc->eoverlapincr    32      /* end ola increment per frame, 4ms */
    lc->framesz     80      /* 10 msec at 8khz */
#endif

    lc->attenfac = ((float).2);     /* attenuation factor per 10ms frame */
    lc->attenincr = (lc->attenfac / lc->framesz);      /* attenuation per sample */
    assert(lc->historylen <= HISTORYLEN_MAX);
    assert(lc->poverlapmax <= POVERLAPMAX_);
    assert(lc->framesz <= FRAMESZ_MAX);

    lc->sbcrt = 36;     /* SBC reconvergence time*/

    lc->erasecnt = 0;
    lc->pitchbufend = &lc->pitchbuf[lc->historylen];
    g711plc_zeros(lc->history, lc->historylen);
}


void cvsd_g711plc_construct(LowcFE_c *lc)
{
    lc->pitch_min = 40;      /* minimum allowed pitch, 200 Hz */
    lc->pitch_max = 120;     /* maximum allowed pitch, 66 Hz */
    lc->pitchdiff = (lc->pitch_max - lc->pitch_min);
    lc->poverlapmax = (lc->pitch_max >> 2);        /* maximum pitch ola window */
    lc->historylen = (lc->pitch_max * 3 + lc->poverlapmax);   /* history buffer length */
    lc->ndec = 2;       /* 2:1 decimation */
    lc->corrlen = 160;     /* 20 msec correlation length */
    lc->corrbuflen = (lc->corrlen + lc->pitch_max);   /* correlation buffer length */
    lc->corrminpower = ((Float)250.);   /* minimum power */

#if 1
    lc->eoverlapincr = 24;      /* end ola increment per frame, 3ms */
    lc->framesz = 60;      /* 7.5 msec at 8khz */
#else
    lc->eoverlapincr    32      /* end ola increment per frame, 4ms */
    lc->framesz     80      /* 10 msec at 8khz */
#endif

    lc->attenfac = ((Float).2);     /* attenuation factor per 10ms frame */
    lc->attenincr = (lc->attenfac / lc->framesz);      /* attenuation per sample */
    assert(lc->historylen <= HISTORYLEN_MAX);
    assert(lc->poverlapmax <= POVERLAPMAX_);
    assert(lc->framesz <= FRAMESZ_MAX);

    lc->sbcrt = 0;      /* SBC reconvergence time*/

    lc->erasecnt = 0;
    lc->pitchbufend = &lc->pitchbuf[lc->historylen];
    g711plc_zeros(lc->history, lc->historylen);
}

/*
 * Get samples from the circular pitch buffer. Update poffset so
 * when subsequent frames are erased the signal continues.
 */
static void g711plc_getfespeech(LowcFE_c *lc, short *out, int sz)
{
    while (sz)
    {
        int cnt = lc->pitchblen - lc->poffset;
        if (cnt > sz)
            cnt = sz;
        g711plc_convertfs(&lc->pitchbufstart[lc->poffset], out, cnt);
        lc->poffset += cnt;
        if (lc->poffset == lc->pitchblen)
            lc->poffset = 0;
        out += cnt;
        sz -= cnt;
    }
}

static void g711plc_scalespeech(LowcFE_c *lc, short *out)
{
    int i;
    Float g = (Float) 1. - (lc->erasecnt - 1) * lc->attenfac;
    for (i = 0; i < lc->framesz; i++)
    {
        out[i] = (short)(out[i] * g);
        g -= lc->attenincr;
    }
}

/*
 * Generate the synthetic signal.
 * At the beginning of an erasure determine the pitch, and extract
 * one pitch period from the tail of the signal. Do an OLA for 1/4
 * of the pitch to smooth the signal. Then repeat the extracted signal
 * for the length of the erasure. If the erasure continues for more than
 * 10 msec, increase the number of periods in the pitchbuffer. At the end
 * of an erasure, do an OLA with the start of the first good frame.
 * The gain decays as the erasure gets longer.
 */
void g711plc_dofe(LowcFE_c *lc, short *out)
{
    if (lc->erasecnt == 0)
    {
        /* get history */
        g711plc_convertsf(lc->history, lc->pitchbuf, lc->historylen);
        lc->pitch = g711plc_findpitch(lc); /* find pitch */
        lc->poverlap = lc->pitch >> 2;      /* OLA 1/4 wavelength */
        /* save original last poverlap samples */
        g711plc_copyf(lc->pitchbufend - lc->poverlap, lc->lastq, lc->poverlap);
        lc->poffset = 0;            /* create pitch buffer with 1 period */
        lc->pitchblen = lc->pitch;
        lc->pitchbufstart = lc->pitchbufend - lc->pitchblen;
        g711plc_overlapadd(lc->lastq, lc->pitchbufstart - lc->poverlap, lc->pitchbufend - lc->poverlap, lc->poverlap);
        /* update last 1/4 wavelength in history buffer */
        g711plc_convertfs(lc->pitchbufend - lc->poverlap, &lc->history[lc->historylen - lc->poverlap], lc->poverlap);
        /* get synthesized speech */
        g711plc_getfespeech(lc, out, lc->framesz);
    }
    else if (lc->erasecnt == 1 || lc->erasecnt == 2)
    {
        /* tail of previous pitch estimate */
        short tmp[POVERLAPMAX_];
        int saveoffset = lc->poffset;       /* save offset for OLA */
        /* continue with old pitchbuf */
        g711plc_getfespeech(lc, tmp, lc->poverlap);
        /* add periods to the pitch buffer */
        lc->poffset = saveoffset;
        while (lc->poffset > lc->pitch)
            lc->poffset -= lc->pitch;
        lc->pitchblen += lc->pitch; /* add a period */
        lc->pitchbufstart = lc->pitchbufend - lc->pitchblen;
        g711plc_overlapadd(lc->lastq, lc->pitchbufstart - lc->poverlap, lc->pitchbufend - lc->poverlap, lc->poverlap);
        /* overlap add old pitchbuffer with new */
        g711plc_getfespeech(lc, out, lc->framesz);
        g711plc_overlapadds(tmp, out, out, lc->poverlap);
        g711plc_scalespeech(lc, out);
    }
    else if (lc->erasecnt > 5)
    {
        g711plc_zeros(out, lc->framesz);
    }
    else
    {
        g711plc_getfespeech(lc, out, lc->framesz);
        g711plc_scalespeech(lc, out);
    }
    lc->erasecnt++;
    g711plc_savespeech(lc, out);
}

/*
 * Save a frames worth of new speech in the history buffer.
 * Return the output speech delayed by POVERLAPMAX.
 */
static void g711plc_savespeech(LowcFE_c *lc, short *s)
{
    /* make room for new signal */
    g711plc_copys(&lc->history[lc->framesz], lc->history, lc->historylen - lc->framesz);
    /* copy in the new frame */
    g711plc_copys(s, &lc->history[lc->historylen - lc->framesz], lc->framesz);
    /* copy out the delayed frame */
    g711plc_copys(&lc->history[lc->historylen - lc->framesz - lc->poverlapmax], s, lc->framesz);
}

/*
 * A good frame was received and decoded.
 * If right after an erasure, do an overlap add with the synthetic signal.
 * Add the frame to history buffer.
 */
void g711plc_addtohistory(LowcFE_c *lc, short *s)
{
    if (lc->erasecnt)
    {
        short overlapbuf[FRAMESZ_MAX];
        /*
         * longer erasures require longer overlaps
         * to smooth the transition between the synthetic
         * and real signal.
         */
        int olen = lc->poverlap + (lc->erasecnt + 1 - 1) * lc->eoverlapincr + lc->sbcrt;
        if (olen > lc->framesz)
            olen = lc->framesz;
        g711plc_getfespeech(lc, overlapbuf, olen);
        g711plc_overlapaddatend(lc, s, overlapbuf, olen);
        lc->erasecnt = 0;
    }
    g711plc_savespeech(lc, s);
}

/*
 * Overlapp add the end of the erasure with the start of the first good frame
 * Scale the synthetic speech by the gain factor before the OLA.
 */
static void g711plc_overlapaddatend(LowcFE_c *lc, short *s, short *f, int cnt)
{
    int i;
    Float incrg;
    Float lw, rw;
    Float t;
    Float incr = (Float) 1. / (cnt - lc->sbcrt);
    Float gain = (Float) 1. - (lc->erasecnt - 1) * lc->attenfac;
    if (gain < 0.)
        gain = (Float) 0.;
    incrg = incr * gain;
    lw = ((Float) 1. - incr) * gain;
    rw = incr;

    for (i = 0; i < lc->sbcrt; i++)
    {
        t = gain * f[i];
        s[i] = (short)t;
    }

    for (i = lc->sbcrt; i < cnt; i++)
    {
        t = lw * f[i] + rw * s[i];
        if (t > 32767.)
            t = (Float) 32767.;
        else if (t < -32768.)
            t = (Float) - 32768.;
        s[i] = (short)t;
        lw -= incrg;
        rw += incr;
    }
}

/*
 * Overlapp add left and right sides
 */
static void g711plc_overlapadd(Float *l, Float *r, Float *o, int cnt)
{
    int i;
    Float incr, lw, rw, t;

    if (cnt == 0)
        return;
    incr = (Float) 1. / cnt;
    lw = (Float) 1. - incr;
    rw = incr;
    for (i = 0; i < cnt; i++)
    {
        t = lw * l[i] + rw * r[i];
        if (t > (Float) 32767.)
            t = (Float) 32767.;
        else if (t < (Float) - 32768.)
            t = (Float) - 32768.;
        o[i] = t;
        lw -= incr;
        rw += incr;
    }
}

/*
 * Overlapp add left and right sides
 */
static void g711plc_overlapadds(short *l, short *r, short *o, int cnt)
{
    int i;
    Float incr, lw, rw, t;

    if (cnt == 0)
        return;
    incr = (Float) 1. / cnt;
    lw = (Float) 1. - incr;
    rw = incr;
    for (i = 0; i < cnt; i++)
    {
        t = lw * l[i] + rw * r[i];
        if (t > (Float) 32767.)
            t = (Float) 32767.;
        else if (t < (Float) - 32768.)
            t = (Float) - 32768.;
        o[i] = (short)t;
        lw -= incr;
        rw += incr;
    }
}

/*
 * Estimate the pitch.
 * l - pointer to first sample in last 20 msec of speech.
 * r - points to the sample PITCH_MAX before l
 */
static int g711plc_findpitch(LowcFE_c *lc)
{
    int i, j, k;
    int bestmatch;
    Float bestcorr;
    Float corr;                   /* correlation */
    Float energy;                 /* running energy */
    Float scale;                  /* scale correlation by average power */
    Float *rp;                    /* segment to match */
    Float *l = lc->pitchbufend - lc->corrlen;
    Float *r = lc->pitchbufend - lc->corrbuflen;

    /* coarse search */
    rp = r;
    energy = (Float) 0.;
    corr = (Float) 0.;
    for (i = 0; i < lc->corrlen; i += lc->ndec)
    {
        energy += rp[i] * rp[i];
        corr += rp[i] * l[i];
    }
    scale = energy;
    if (scale < lc->corrminpower)
        scale = lc->corrminpower;
    corr = corr / (Float)sqrt(scale);
    bestcorr = corr;
    bestmatch = 0;
    for (j = lc->ndec; j <= lc->pitchdiff; j += lc->ndec)
    {
        energy -= rp[0] * rp[0];
        energy += rp[lc->corrlen] * rp[lc->corrlen];
        rp += lc->ndec;
        corr = 0.f;
        for (i = 0; i < lc->corrlen; i += lc->ndec)
            corr += rp[i] * l[i];
        scale = energy;
        if (scale < lc->corrminpower)
            scale = lc->corrminpower;
        corr /= (Float)sqrt(scale);
        if (corr >= bestcorr)
        {
            bestcorr = corr;
            bestmatch = j;
        }
    }
    /* fine search */
    j = bestmatch - (lc->ndec - 1);
    if (j < 0)
        j = 0;
    k = bestmatch + (lc->ndec - 1);
    if (k > lc->pitchdiff)
        k = lc->pitchdiff;
    rp = &r[j];
    energy = 0.f;
    corr = 0.f;
    for (i = 0; i < lc->corrlen; i++)
    {
        energy += rp[i] * rp[i];
        corr += rp[i] * l[i];
    }
    scale = energy;
    if (scale < lc->corrminpower)
        scale = lc->corrminpower;
    corr = corr / (Float)sqrt(scale);
    bestcorr = corr;
    bestmatch = j;
    for (j++; j <= k; j++)
    {
        energy -= rp[0] * rp[0];
        energy += rp[lc->corrlen] * rp[lc->corrlen];
        rp++;
        corr = 0.f;
        for (i = 0; i < lc->corrlen; i++)
            corr += rp[i] * l[i];
        scale = energy;
        if (scale < lc->corrminpower)
            scale = lc->corrminpower;
        corr = corr / (Float)sqrt(scale);
        if (corr > bestcorr)
        {
            bestcorr = corr;
            bestmatch = j;
        }
    }
    return lc->pitch_max - bestmatch;
}

static void g711plc_convertsf(short *f, Float *t, int cnt)
{
    int i;
    for (i = 0; i < cnt; i++)
        t[i] = (Float)f[i];
}

static void g711plc_convertfs(Float *f, short *t, int cnt)
{
    int i;
    for (i = 0; i < cnt; i++)
        t[i] = (short)f[i];
}

static void g711plc_copyf(Float *f, Float *t, int cnt)
{
    int i;
    for (i = 0; i < cnt; i++)
        t[i] = f[i];
}

static void g711plc_copys(short *f, short *t, int cnt)
{
    int i;
    for (i = 0; i < cnt; i++)
        t[i] = f[i];
}

static void g711plc_zeros(short *s, int cnt)
{
    int i;
    for (i = 0; i < cnt; i++)
        s[i] = 0;
}
