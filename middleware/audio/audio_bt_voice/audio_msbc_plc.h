/*
  ============================================================================
   File: lowcfe.h                                            V.1.0-24.MAY-2005
  ============================================================================
                     UGST/ITU-T G711 Appendix I PLC MODULE
                          GLOBAL FUNCTION PROTOTYPES
   History:
   24.May.05    v1.0    First version <AT&T>
                        Integration in STL2005 <Cyril Guillaume & Stephane Ragot - stephane.ragot@francetelecom.com>
  ============================================================================
*/
#ifndef __MSBC_LOWCFE_C_H__
#define __MSBC_LOWCFE_C_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifdef USEDOUBLES
typedef double Float;         /* likely to be bit-exact between machines */
#else
typedef float Float;
#endif

#define HISTORYLEN_MAX  780
#define POVERLAPMAX_    60
#define FRAMESZ_MAX     120
typedef struct _LowcFE_c
{
    int pitch_min;
    int pitch_max;
    int pitchdiff;
    int poverlapmax;
    int historylen;
    int ndec;
    int corrlen;
    int corrbuflen;
    Float corrminpower;
    int eoverlapincr;
    int framesz;
    Float attenfac;
    Float attenincr;
    int erasecnt;               /* consecutive erased frames */
    int poverlap;               /* overlap based on pitch */
    int poffset;                /* offset into pitch period */
    int pitch;                  /* pitch estimate */
    int pitchblen;              /* current pitch buffer length */
    int sbcrt;                  /* SBC reconvergence time*/
    Float *pitchbufend;         /* end of pitch buffer */
    Float *pitchbufstart;       /* start of pitch buffer */
    Float pitchbuf[HISTORYLEN_MAX]; /* buffer for cycles of speech */
    Float lastq[POVERLAPMAX_];   /* saved last quarter wavelengh */
    short history[HISTORYLEN_MAX];  /* history buffer */
} LowcFE_c;

/* public functions */
void msbc_g711plc_construct(LowcFE_c *);  /* constructor */
void cvsd_g711plc_construct(LowcFE_c *);  /* constructor */
void g711plc_dofe(LowcFE_c *, short *s);     /* synthesize speech for erasure */
void g711plc_addtohistory(LowcFE_c *, short *s);
/* add a good frame to history buffer */

#ifdef __cplusplus
}
#endif
#endif                          /* __MSBC_LOWCFE_C_H__ */