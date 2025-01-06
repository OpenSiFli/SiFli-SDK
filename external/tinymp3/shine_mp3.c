/* shine_layer3.c */
#include "shine_mp3.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#define GRANULE_SIZE  576
/* Include arch-specific instructions,
 * when defined. */
#if defined(__mips__) && (__mips == 32)
#include <stdint.h>

#define mul(a,b) \
({ \
    register int32_t res; \
    __asm__ __volatile__("mult %0, %1" : : "r" (a), "r" (b)); \
    __asm__ __volatile__("mfhi %0" : "=r" (res)); \
    res; \
})

#define mul0(hi,lo,a,b) \
    __asm__ __volatile__("mult %0, %1" : : "r" (a), "r" (b))

#define muladd(hi,lo,a,b) \
    __asm__ __volatile__("madd %0, %1" : : "r" (a), "r" (b))

#define mulsub(hi,lo,a,b) \
    __asm__ __volatile__("msub %0, %1" : : "r" (a), "r" (b))

#define mulz(hi,lo) \
do { \
    register int32_t t; \
    __asm__ __volatile__("mfhi %0" : "=r" (t)); \
    (hi) = t; \
} while (0)

#define cmuls(dre, dim, are, aim, bre, bim) \
do { \
    register int32_t t1, t2, tre; \
    __asm__ __volatile__("mult %0, %1" : : "r" (are), "r" (bre)); \
    __asm__ __volatile__("msub %0, %1" : : "r" (aim), "r" (bim)); \
    __asm__ __volatile__("mfhi %0; mflo %1" : "=r" (t1), "=r" (t2)); \
    tre = (t1 << 1) | ((uint32_t)t2 >> 31); \
    __asm__ __volatile__("mult %0, %1" : : "r" (are), "r" (bim)); \
    __asm__ __volatile__("madd %0, %1" : : "r" (bre), "r" (aim)); \
    dre = tre; \
    __asm__ __volatile__("mfhi %0; mflo %1" : "=r" (t1), "=r" (t2)); \
    dim = (t1 << 1) | ((uint32_t)t2 >> 31); \
} while (0)

#if __mips_isa_rev >= 2
static inline uint32_t SWAB32(uint32_t x)
{
    __asm__(
    "	wsbh	%0, %1			\n"
    "	rotr	%0, %0, 16		\n"
    : "=r" (x) : "r" (x));
    return x;
}
#define SWAB32 SWAB32
#endif

#elif defined(__arm__) && !defined(__thumb__)
#include <stdint.h>

/* Fractional multiply */
#if __ARM_ARCH >= 6
#define mul(x,y) \
({ \
    register int32_t result; \
    asm ("smmul %0, %2, %1" : "=r" (result) : "r" (x), "r" (y)); \
    result ;\
})
#else
#define mul(x,y) \
({ \
    register int32_t result; \
    asm ("smull r3, %0, %2, %1" : "=r" (result) : "r" (x), "r" (y) : "r3"); \
    result ; \
})
#endif

/* Fractional multiply with single bit left shift. */
#define muls(x,y) \
({ \
    register int32_t result; \
    asm ( \
        "smull r3, %0, %2, %1\n\t" \
        "movs r3, r3, lsl #1\n\t" \
        "adc %0, %0, %0" \
        : "=r" (result) : "r" (x), "r" (y) : "r3", "cc" \
    ); \
    result; \
})


#if __ARM_ARCH >= 6
#define mulr(x,y) \
({ \
    register int32_t result; \
    asm ( \
        "smmulr %0, %2, %1" : "=r" (result) : "r" (x), "r" (y) \
    ); \
    result; \
})
#else
#define mulr(x,y) \
({ \
    register int32_t result; \
    asm ( \
        "smull r3, %0, %2, %1\n\t" \
        "adds r3, r3, #0x80000000\n\t" \
        "adc %0, %0, #0" \
        : "=r" (result) : "r" (x), "r" (y) : "r3", "cc" \
    ); \
    result; \
})
#endif

#define mulsr(x,y) \
({ \
    register int32_t result; \
    asm ( \
        "smull r3, %0, %1, %2\n\t" \
        "movs r3, r3, lsl #1\n\t" \
        "adc %0, %0, %0\n\t" \
        "adds r3, r3, #0x80000000\n\t" \
        "adc %0, %0, #0" \
        : "=r" (result) : "r" (x), "r" (y) : "r3", "cc" \
    ); \
    result; \
})

#define mul0(hi,lo,a,b) \
    asm ("smull %0, %1, %2, %3" : "=r" (lo), "=r" (hi) : "r" (a), "r" (b))

#define muladd(hi,lo,a,b) \
    asm ("smlal %0, %1, %2, %3" : "+r" (lo), "+r" (hi) : "r" (a), "r" (b))

#define mulsub(hi,lo,a,b) \
    asm ("smlal %0, %1, %2, %3" : "+r" (lo), "+r" (hi) : "r" (a), "r" (-(b)))

#define mulz(hi,lo)

#define cmuls(dre, dim, are, aim, bre, bim) \
do { \
    register int32_t tre, tim; \
    asm ( \
        "smull r3, %0, %2, %4\n\t" \
        "smlal r3, %0, %3, %5\n\t" \
        "movs r3, r3, lsl #1\n\t" \
        "adc %0, %0, %0\n\t" \
        "smull r3, %1, %2, %6\n\t" \
        "smlal r3, %1, %4, %3\n\t" \
        "movs r3, r3, lsl #1\n\t" \
        "adc %1, %1, %1\n\t" \
        : "=&r" (tre), "=&r" (tim) \
        : "r" (are), "r" (aim), "r" (bre), "r" (-(bim)), "r" (bim) \
        : "r3", "cc" \
    ); \
    dre = tre; \
    dim = tim; \
} while (0)

#if __ARM_ARCH >= 6
static inline uint32_t SWAB32(uint32_t x)
{
    asm ("rev %0, %1" : "=r" (x) : "r" (x));
    return x;
}
#define SWAB32 SWAB32
#endif
#endif

/* Include and define generic instructions,
 * when not already defined above. */
#include <stdint.h>

#ifndef mul
#define mul(a, b)   (int32_t)  ( ( ((int64_t) a) * ((int64_t) b) ) >>32 )
#endif

#ifndef muls
#define muls(a, b)  (int32_t)  ( ( ((int64_t) a) * ((int64_t) b) ) >>31 )
#endif

#ifndef mulr
#define mulr(a, b)  (int32_t)  ( ( ( ((int64_t) a) * ((int64_t) b)) + 0x80000000LL ) >>32 )
#endif

#ifndef mulsr
#define mulsr(a, b) (int32_t)  ( ( ( ((int64_t) a) * ((int64_t) b)) + 0x40000000LL ) >>31 )
#endif

#ifndef mul0
#define mul0(hi, lo, a, b)     ((hi)  = mul((a), (b)))
#define muladd(hi, lo, a, b)   ((hi) += mul((a), (b)))
#define mulsub(hi, lo, a, b)   ((hi) -= mul((a), (b)))
#define mulz(hi, lo)
#endif

#ifndef cmuls
#define cmuls(dre, dim, are, aim, bre, bim) \
do { \
    int32_t tre; \
    (tre) = (int32_t) (((int64_t) (are) * (int64_t) (bre) - (int64_t) (aim) * (int64_t) (bim)) >> 31); \
    (dim) = (int32_t) (((int64_t) (are) * (int64_t) (bim) + (int64_t) (aim) * (int64_t) (bre)) >> 31); \
    (dre) = tre; \
} while (0)
#endif


#ifndef SWAB32
#if defined(__GNUC__) && (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 2))
#define SWAB32(x)    __builtin_bswap32(x)
#else
#define SWAB32(x)	(((unsigned int)(x) >> 24) | (((unsigned int)(x) >> 8) & 0xff00) | (((unsigned int)(x) & 0xff00) << 8) | ((unsigned int)(x) << 24))
#endif

/* #define DEBUG if you want the library to dump info to stdout */

#define PI          3.14159265358979
#define PI4         0.78539816339745
#define PI12        0.26179938779915
#define PI36        0.087266462599717
#define PI64        0.049087385212
#define SQRT2       1.41421356237
#define LN2         0.69314718
#define LN_TO_LOG10 0.2302585093
#define BLKSIZE     1024
#define HAN_SIZE    512 /* for loop unrolling, require that HAN_SIZE%8==0 */
#define SCALE_BLOCK 12
#define SCALE_RANGE 64
#define SCALE       32768
#define SBLIMIT     32

#ifndef MAX_CHANNELS
#define MAX_CHANNELS 2
#endif

#ifndef MAX_GRANULES
#define MAX_GRANULES 2
#endif

typedef struct {
    int channels;
    int samplerate;
} priv_shine_wave_t;

typedef struct {
    int version;
    int layer;
    int granules_per_frame;
    int mode;      /* + */ /* Stereo mode */
    int bitr;      /* + */ /* Must conform to known bitrate - see Main.c */
    int emph;      /* + */ /* De-emphasis */
    int padding;
    int bits_per_frame;
    int bits_per_slot;
    double frac_slots_per_frame;
    double slot_lag;
    int whole_slots_per_frame;
    int bitrate_index;     /* + */ /* See Main.c and Layer3.c */
    int samplerate_index;  /* + */ /* See Main.c and Layer3.c */
    int crc;
    int ext;
    int mode_ext;
    int copyright;  /* + */
    int original;   /* + */
} priv_shine_mpeg_t;

typedef struct {
    int32_t *xr;                    /* magnitudes of the spectral values */
    int32_t xrsq[GRANULE_SIZE];     /* xr squared */
    int32_t xrabs[GRANULE_SIZE];    /* xr absolute */
    int32_t xrmax;                  /* maximum of xrabs array */
    int32_t en_tot[MAX_GRANULES];   /* gr */
    int32_t en[MAX_GRANULES][21];
    int32_t xm[MAX_GRANULES][21];
    int32_t xrmaxl[MAX_GRANULES];
    double steptab[128]; /* 2**(-x/4)  for x = -127..0 */
    int32_t steptabi[128];  /* 2**(-x/4)  for x = -127..0 */
    int int2idx[10000]; /* x**(3/4)   for x = 0..9999 */
} l3loop_t;

typedef struct {
    int32_t cos_l[18][36];
} mdct_t;

typedef struct {
    int off[MAX_CHANNELS];
    int32_t fl[SBLIMIT][64];
    int32_t x[MAX_CHANNELS][HAN_SIZE];
} subband_t;

/* Side information */
typedef struct {
    unsigned part2_3_length;
    unsigned big_values;
    unsigned count1;
    unsigned global_gain;
    unsigned scalefac_compress;
    unsigned table_select[3];
    unsigned region0_count;
    unsigned region1_count;
    unsigned preflag;
    unsigned scalefac_scale;
    unsigned count1table_select;
    unsigned part2_length;
    unsigned sfb_lmax;
    unsigned address1;
    unsigned address2;
    unsigned address3;
    int quantizerStepSize;
    unsigned slen[4];
} gr_info;

typedef struct {
    unsigned private_bits;
    int resvDrain;
    unsigned scfsi[MAX_CHANNELS][4];
    struct {
        struct {
            gr_info tt;
        } ch[MAX_CHANNELS];
    } gr[MAX_GRANULES];
} shine_side_info_t;

typedef struct {
    double l[MAX_GRANULES][MAX_CHANNELS][21];
} shine_psy_ratio_t;

typedef struct {
    double l[MAX_GRANULES][MAX_CHANNELS][21];
} shine_psy_xmin_t;

typedef struct {
    int32_t l[MAX_GRANULES][MAX_CHANNELS][22];            /* [cb] */
    int32_t s[MAX_GRANULES][MAX_CHANNELS][13][3];         /* [window][cb] */
} shine_scalefac_t;

typedef struct bit_stream_struc {
    unsigned char *data;        /* Processed data */
    int data_size;      /* Total data size */
    int data_position;  /* Data position */
    unsigned int cache;            /* bit stream cache */
    int cache_bits;     /* free bits in cache */
} bitstream_t;


#define         MINIMUM         4    /* Minimum size of the buffer in bytes */
#define         MAX_LENGTH      32   /* Maximum length of word written or
                                        read from bit stream */

#define         BUFFER_SIZE     4096

#define         MIN(A, B)       ((A) < (B) ? (A) : (B))
#define         MAX(A, B)       ((A) > (B) ? (A) : (B))

void shine_open_bit_stream(bitstream_t *bs, const int size);

void shine_close_bit_stream(bitstream_t *bs);

void shine_putbits(bitstream_t *bs, unsigned int val, unsigned int N);

int shine_get_bits_count(bitstream_t *bs);


typedef struct shine_global_flags {
    priv_shine_wave_t wave;
    priv_shine_mpeg_t mpeg;
    bitstream_t bs;
    shine_side_info_t side_info;
    int sideinfo_len;
    int mean_bits;
    shine_psy_ratio_t ratio;
    shine_scalefac_t scalefactor;
    int16_t *buffer[MAX_CHANNELS];
    double pe[MAX_CHANNELS][MAX_GRANULES];
    int l3_enc[MAX_CHANNELS][MAX_GRANULES][GRANULE_SIZE];
    int32_t l3_sb_sample[MAX_CHANNELS][MAX_GRANULES + 1][18][SBLIMIT];
    int32_t mdct_freq[MAX_CHANNELS][MAX_GRANULES][GRANULE_SIZE];
    int ResvSize;
    int ResvMax;
    l3loop_t l3loop;
    mdct_t mdct;
    subband_t subband;
} shine_global_config;


void shine_format_bitstream(shine_global_config *config);


/*
 *
 * Here are MPEG1 Table B.8 and MPEG2 Table B.1 -- Layer III scalefactor bands.
 * Index into this using a method such as:
 *    idx  = fr_ps->header->sampling_frequency + (fr_ps->header->version * 3)
 */

const int shine_slen1_tab[16] = {0, 0, 0, 0, 3, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4};
const int shine_slen2_tab[16] = {0, 1, 2, 3, 0, 1, 2, 3, 1, 2, 3, 1, 2, 3, 2, 3};

/* Valid samplerates and bitrates. */
const int samplerates[9] = {
        44100, 48000, 32000, /* MPEG-I */
        22050, 24000, 16000, /* MPEG-II */
        11025, 12000, 8000   /* MPEG-2.5 */
};

const int bitrates[16][4] = {
        /* MPEG version:
         * 2.5, reserved, II, I */
        {-1, -1, -1,  -1},
        {8,  -1, 8,   32},
        {16, -1, 16,  40},
        {24, -1, 24,  48},
        {32, -1, 32,  56},
        {40, -1, 40,  64},
        {48, -1, 48,  80},
        {56, -1, 56,  96},
        {64, -1, 64,  112},
        {-1, -1, 80,  128},
        {-1, -1, 96,  160},
        {-1, -1, 112, 192},
        {-1, -1, 128, 224},
        {-1, -1, 144, 256},
        {-1, -1, 160, 320},
        {-1, -1, -1,  -1}
};

const int shine_scale_fact_band_index[9][23] =
        {
                /* MPEG-I */
                /* Table B.8.b: 44.1 kHz */
                {0, 4,  8,  12, 16, 20, 24, 30, 36,  44,  52,  62,  74,  90,  110, 134, 162, 196, 238, 288, 342, 418, 576},
                /* Table B.8.c: 48 kHz */
                {0, 4,  8,  12, 16, 20, 24, 30, 36,  42,  50,  60,  72,  88,  106, 128, 156, 190, 230, 276, 330, 384, 576},
                /* Table B.8.a: 32 kHz */
                {0, 4,  8,  12, 16, 20, 24, 30, 36,  44,  54,  66,  82,  102, 126, 156, 194, 240, 296, 364, 448, 550, 576},
                /* MPEG-II */
                /* Table B.2.b: 22.05 kHz */
                {0, 6,  12, 18, 24, 30, 36, 44, 54,  66,  80,  96,  116, 140, 168, 200, 238, 284, 336, 396, 464, 522, 576},
                /* Table B.2.c: 24 kHz */
                {0, 6,  12, 18, 24, 30, 36, 44, 54,  66,  80,  96,  114, 136, 162, 194, 232, 278, 330, 394, 464, 540, 576},
                /* Table B.2.a: 16 kHz */
                {0, 6,  12, 18, 24, 30, 36, 44, 45,  66,  80,  96,  116, 140, 168, 200, 238, 248, 336, 396, 464, 522, 576},

                /* MPEG-2.5 */
                /* 11.025 kHz */
                {0, 6,  12, 18, 24, 30, 36, 44, 54,  66,  80,  96,  116, 140, 168, 200, 238, 284, 336, 396, 464, 522, 576},
                /* 12 kHz */
                {0, 6,  12, 18, 24, 30, 36, 44, 54,  66,  80,  96,  116, 140, 168, 200, 238, 284, 336, 396, 464, 522, 576},
                /* MPEG-2.5 8 kHz */
                {0, 12, 24, 36, 48, 60, 72, 88, 108, 132, 160, 192, 232, 280, 336, 400, 476, 566, 568, 570, 572, 574, 576},
        };

/* note. 0.035781 is shine_enwindow maximum value */
/* scale and convert to fixed point before storing */
#define SHINE_EW(x)                    (int32_t)((double)(x) * 0x7fffffff)
#define SHINE_EW2(a, b)                    SHINE_EW(a), SHINE_EW(b)
#define SHINE_EW10(a, b, c, d, e, f, g, h, i, j)    SHINE_EW2(a,b), SHINE_EW2(c,d), SHINE_EW2(e,f), SHINE_EW2(g,h), SHINE_EW2(i,j)

const int32_t shine_enwindow[] = {
        SHINE_EW10(0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000000, -0.000001, -0.000001,
                   -0.000001),
        SHINE_EW10(-0.000001, -0.000001, -0.000001, -0.000002, -0.000002, -0.000002, -0.000002, -0.000003, -0.000003,
                   -0.000003),
        SHINE_EW10(-0.000004, -0.000004, -0.000005, -0.000005, -0.000006, -0.000007, -0.000008, -0.000008, -0.000009,
                   -0.000010),
        SHINE_EW10(-0.000011, -0.000012, -0.000014, -0.000015, -0.000017, -0.000018, -0.000020, -0.000021, -0.000023,
                   -0.000025),
        SHINE_EW10(-0.000028, -0.000030, -0.000032, -0.000035, -0.000038, -0.000041, -0.000043, -0.000046, -0.000050,
                   -0.000053),
        SHINE_EW10(-0.000056, -0.000060, -0.000063, -0.000066, -0.000070, -0.000073, -0.000077, -0.000081, -0.000084,
                   -0.000087),
        SHINE_EW10(-0.000091, -0.000093, -0.000096, -0.000099, 0.000102, 0.000104, 0.000106, 0.000107, 0.000108,
                   0.000109),
        SHINE_EW10(0.000109, 0.000108, 0.000107, 0.000105, 0.000103, 0.000099, 0.000095, 0.000090, 0.000084, 0.000078),
        SHINE_EW10(0.000070, 0.000061, 0.000051, 0.000040, 0.000027, 0.000014, -0.000001, -0.000017, -0.000034,
                   -0.000053),
        SHINE_EW10(-0.000073, -0.000094, -0.000116, -0.000140, -0.000165, -0.000191, -0.000219, -0.000247, -0.000277,
                   -0.000308),
        SHINE_EW10(-0.000339, -0.000371, -0.000404, -0.000438, -0.000473, -0.000507, -0.000542, -0.000577, -0.000612,
                   -0.000647),
        SHINE_EW10(-0.000681, -0.000714, -0.000747, -0.000779, -0.000810, -0.000839, -0.000866, -0.000892, -0.000915,
                   -0.000936),
        SHINE_EW10(-0.000954, -0.000969, -0.000981, -0.000989, -0.000994, -0.000995, -0.000992, -0.000984, 0.000971,
                   0.000954),
        SHINE_EW10(0.000931, 0.000903, 0.000869, 0.000829, 0.000784, 0.000732, 0.000674, 0.000610, 0.000539, 0.000463),
        SHINE_EW10(0.000379, 0.000288, 0.000192, 0.000088, -0.000021, -0.000137, -0.000260, -0.000388, -0.000522,
                   -0.000662),
        SHINE_EW10(-0.000807, -0.000957, -0.001111, -0.001270, -0.001432, -0.001598, -0.001767, -0.001937, -0.002110,
                   -0.002283),
        SHINE_EW10(-0.002457, -0.002631, -0.002803, -0.002974, -0.003142, -0.003307, -0.003467, -0.003623, -0.003772,
                   -0.003914),
        SHINE_EW10(-0.004049, -0.004175, -0.004291, -0.004396, -0.004490, -0.004570, -0.004638, -0.004691, -0.004728,
                   -0.004749),
        SHINE_EW10(-0.004752, -0.004737, -0.004703, -0.004649, -0.004574, -0.004477, -0.004358, -0.004215, -0.004049,
                   -0.003859),
        SHINE_EW10(-0.003643, -0.003402, 0.003135, 0.002841, 0.002522, 0.002175, 0.001801, 0.001400, 0.000971,
                   0.000516),
        SHINE_EW10(0.000033, -0.000476, -0.001012, -0.001574, -0.002162, -0.002774, -0.003411, -0.004072, -0.004756,
                   -0.005462),
        SHINE_EW10(-0.006189, -0.006937, -0.007703, -0.008487, -0.009288, -0.010104, -0.010933, -0.011775, -0.012628,
                   -0.013489),
        SHINE_EW10(-0.014359, -0.015234, -0.016113, -0.016994, -0.017876, -0.018757, -0.019634, -0.020507, -0.021372,
                   -0.022229),
        SHINE_EW10(-0.023074, -0.023907, -0.024725, -0.025527, -0.026311, -0.027074, -0.027815, -0.028533, -0.029225,
                   -0.029890),
        SHINE_EW10(-0.030527, -0.031133, -0.031707, -0.032248, -0.032755, -0.033226, -0.033660, -0.034056, -0.034413,
                   -0.034730),
        SHINE_EW10(-0.035007, -0.035242, -0.035435, -0.035586, -0.035694, -0.035759, 0.035781, 0.035759, 0.035694,
                   0.035586),
        SHINE_EW10(0.035435, 0.035242, 0.035007, 0.034730, 0.034413, 0.034056, 0.033660, 0.033226, 0.032755, 0.032248),
        SHINE_EW10(0.031707, 0.031133, 0.030527, 0.029890, 0.029225, 0.028533, 0.027815, 0.027074, 0.026311, 0.025527),
        SHINE_EW10(0.024725, 0.023907, 0.023074, 0.022229, 0.021372, 0.020507, 0.019634, 0.018757, 0.017876, 0.016994),
        SHINE_EW10(0.016113, 0.015234, 0.014359, 0.013489, 0.012628, 0.011775, 0.010933, 0.010104, 0.009288, 0.008487),
        SHINE_EW10(0.007703, 0.006937, 0.006189, 0.005462, 0.004756, 0.004072, 0.003411, 0.002774, 0.002162, 0.001574),
        SHINE_EW10(0.001012, 0.000476, -0.000033, -0.000516, -0.000971, -0.001400, -0.001801, -0.002175, -0.002522,
                   -0.002841),
        SHINE_EW10(0.003135, 0.003402, 0.003643, 0.003859, 0.004049, 0.004215, 0.004358, 0.004477, 0.004574, 0.004649),
        SHINE_EW10(0.004703, 0.004737, 0.004752, 0.004749, 0.004728, 0.004691, 0.004638, 0.004570, 0.004490, 0.004396),
        SHINE_EW10(0.004291, 0.004175, 0.004049, 0.003914, 0.003772, 0.003623, 0.003467, 0.003307, 0.003142, 0.002974),
        SHINE_EW10(0.002803, 0.002631, 0.002457, 0.002283, 0.002110, 0.001937, 0.001767, 0.001598, 0.001432, 0.001270),
        SHINE_EW10(0.001111, 0.000957, 0.000807, 0.000662, 0.000522, 0.000388, 0.000260, 0.000137, 0.000021, -0.000088),
        SHINE_EW10(-0.000192, -0.000288, -0.000379, -0.000463, -0.000539, -0.000610, -0.000674, -0.000732, -0.000784,
                   -0.000829),
        SHINE_EW10(-0.000869, -0.000903, -0.000931, -0.000954, 0.000971, 0.000984, 0.000992, 0.000995, 0.000994,
                   0.000989),
        SHINE_EW10(0.000981, 0.000969, 0.000954, 0.000936, 0.000915, 0.000892, 0.000866, 0.000839, 0.000810, 0.000779),
        SHINE_EW10(0.000747, 0.000714, 0.000681, 0.000647, 0.000612, 0.000577, 0.000542, 0.000507, 0.000473, 0.000438),
        SHINE_EW10(0.000404, 0.000371, 0.000339, 0.000308, 0.000277, 0.000247, 0.000219, 0.000191, 0.000165, 0.000140),
        SHINE_EW10(0.000116, 0.000094, 0.000073, 0.000053, 0.000034, 0.000017, 0.000001, -0.000014, -0.000027,
                   -0.000040),
        SHINE_EW10(-0.000051, -0.000061, -0.000070, -0.000078, -0.000084, -0.000090, -0.000095, -0.000099, -0.000103,
                   -0.000105),
        SHINE_EW10(-0.000107, -0.000108, -0.000109, -0.000109, -0.000108, -0.000107, -0.000106, -0.000104, 0.000102,
                   0.000099),
        SHINE_EW10(0.000096, 0.000093, 0.000091, 0.000087, 0.000084, 0.000081, 0.000077, 0.000073, 0.000070, 0.000066),
        SHINE_EW10(0.000063, 0.000060, 0.000056, 0.000053, 0.000050, 0.000046, 0.000043, 0.000041, 0.000038, 0.000035),
        SHINE_EW10(0.000032, 0.000030, 0.000028, 0.000025, 0.000023, 0.000021, 0.000020, 0.000018, 0.000017, 0.000015),
        SHINE_EW10(0.000014, 0.000012, 0.000011, 0.000010, 0.000009, 0.000008, 0.000008, 0.000007, 0.000006, 0.000005),
        SHINE_EW10(0.000005, 0.000004, 0.000004, 0.000003, 0.000003, 0.000003, 0.000002, 0.000002, 0.000002, 0.000002),
        SHINE_EW10(0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000000, 0.000000, 0.000000, 0.000000),
        SHINE_EW2 (0.000000, 0.000000)
};

#endif
#define HUFFBITS uint16_t
#define HTN     34
#define MXOFF   250

struct huffcodetab {
    unsigned int xlen;         /*max. x-index+                         */
    unsigned int ylen;         /*max. y-index+                         */
    unsigned int linbits;      /*number of linbits                     */
    unsigned int linmax;       /*max number to be stored in linbits    */
    const HUFFBITS *table;     /*pointer to array[xlen][ylen]          */
    const unsigned char *hlen; /*pointer to array[xlen][ylen]          */
};

extern const struct huffcodetab shine_huffman_table[HTN];/* global memory block                */
/* array of all huffcodtable headers    */
/* 0..31 Huffman code table 0..31       */
/* 32,33 count1-tables                  */

#ifndef RESERVOIR_H
#define RESERVOIR_H

void shine_ResvFrameBegin(int frameLength, shine_global_config *config);

int shine_max_reservoir_bits(double *pe, shine_global_config *config);

void shine_ResvAdjust(gr_info *gi, shine_global_config *config);

void shine_ResvFrameEnd(shine_global_config *config);

#endif


#ifndef L3SUBBAND_H
#define L3SUBBAND_H

#include <stdint.h>

void shine_subband_initialise(shine_global_config *config);

void shine_window_filter_subband(int16_t **buffer, int32_t s[SBLIMIT], int k, shine_global_config *config, int stride);

#endif

#ifndef shine_MDCT_H
#define shine_MDCT_H

void shine_mdct_initialise();

void shine_mdct_sub(shine_global_config *config, int stride);

#endif

#ifndef L3LOOP_H
#define L3LOOP_H

void shine_loop_initialise(shine_global_config *config);

void shine_iteration_loop(shine_global_config *config);

#endif


const HUFFBITS dmask = 1 << (((sizeof(HUFFBITS)) << 3) - 1);
const unsigned int hs = sizeof(HUFFBITS) << 3;

static const HUFFBITS t1HB[] = {1, 1, 1, 0};
static const HUFFBITS t2HB[] = {1, 2, 1, 3, 1, 1, 3, 2, 0};
static const HUFFBITS t3HB[] = {3, 2, 1, 1, 1, 1, 3, 2, 0};
static const HUFFBITS t5HB[] = {1, 2, 6, 5, 3, 1, 4, 4, 7, 5, 7, 1, 6, 1, 1, 0};
static const HUFFBITS t6HB[] = {7, 3, 5, 1, 6, 2, 3, 2, 5, 4, 4, 1, 3, 3, 2, 0};
static const HUFFBITS t7HB[] = {1, 2, 10, 19, 16, 10, 3, 3, 7, 10, 5, 3, 11, 4, 13, 17, 8, 4, 12, 11, 18, 15, 11, 2, 7,
                                6, 9, 14, 3, 1, 6, 4, 5, 3, 2, 0};
static const HUFFBITS t8HB[] = {3, 4, 6, 18, 12, 5, 5, 1, 2, 16, 9, 3, 7, 3, 5, 14, 7, 3, 19, 17, 15, 13, 10, 4, 13, 5,
                                8, 11, 5, 1, 12, 4, 4, 1, 1, 0};
static const HUFFBITS t9HB[] = {7, 5, 9, 14, 15, 7, 6, 4, 5, 5, 6, 7, 7, 6, 8, 8, 8, 5, 15, 6, 9, 10, 5, 1, 11, 7, 9, 6,
                                4, 1, 14, 4, 6, 2, 6, 0};
static const HUFFBITS t10HB[] = {1, 2, 10, 23, 35, 30, 12, 17, 3, 3, 8, 12, 18, 21, 12, 7, 11, 9, 15, 21, 32, 40, 19, 6,
                                 14, 13, 22, 34, 46, 23, 18, 7, 20, 19, 33, 47, 27, 22, 9, 3, 31, 22, 41, 26, 21, 20, 5,
                                 3, 14, 13, 10, 11, 16, 6, 5, 1, 9, 8, 7, 8, 4, 4, 2, 0};
static const HUFFBITS t11HB[] = {3, 4, 10, 24, 34, 33, 21, 15, 5, 3, 4, 10, 32, 17, 11, 10, 11, 7, 13, 18, 30, 31, 20,
                                 5, 25, 11, 19, 59, 27, 18, 12, 5, 35, 33, 31, 58, 30, 16, 7, 5, 28, 26, 32, 19, 17, 15,
                                 8, 14, 14, 12, 9, 13, 14, 9, 4, 1, 11, 4, 6, 6, 6, 3, 2, 0};
static const HUFFBITS t12HB[] = {9, 6, 16, 33, 41, 39, 38, 26, 7, 5, 6, 9, 23, 16, 26, 11, 17, 7, 11, 14, 21, 30, 10, 7,
                                 17, 10, 15, 12, 18, 28, 14, 5, 32, 13, 22, 19, 18, 16, 9, 5, 40, 17, 31, 29, 17, 13, 4,
                                 2, 27, 12, 11, 15, 10, 7, 4, 1, 27, 12, 8, 12, 6, 3, 1, 0};
static const HUFFBITS t13HB[] = {1, 5, 14, 21, 34, 51, 46, 71, 42, 52, 68, 52, 67, 44, 43, 19, 3, 4, 12, 19, 31, 26, 44,
                                 33, 31, 24, 32, 24, 31, 35, 22, 14, 15, 13, 23, 36, 59, 49, 77, 65, 29, 40, 30, 40, 27,
                                 33, 42, 16, 22,
                                 20, 37, 61, 56, 79, 73, 64, 43, 76, 56, 37, 26, 31, 25, 14, 35, 16, 60, 57, 97, 75,
                                 114, 91, 54, 73, 55, 41, 48, 53, 23, 24, 58, 27, 50, 96, 76, 70, 93, 84, 77, 58, 79,
                                 29, 74, 49, 41, 17, 47,
                                 45, 78, 74, 115, 94, 90, 79, 69, 83, 71, 50, 59, 38, 36, 15, 72, 34, 56, 95, 92, 85,
                                 91, 90, 86, 73, 77, 65, 51, 44, 43, 42, 43, 20, 30, 44, 55, 78, 72, 87, 78, 61, 46, 54,
                                 37, 30, 20, 16, 53,
                                 25, 41, 37, 44, 59, 54, 81, 66, 76, 57, 54, 37, 18, 39, 11, 35, 33, 31, 57, 42, 82, 72,
                                 80, 47, 58, 55, 21, 22, 26, 38, 22, 53, 25, 23, 38, 70, 60, 51, 36, 55, 26, 34, 23, 27,
                                 14, 9, 7, 34, 32,
                                 28, 39, 49, 75, 30, 52, 48, 40, 52, 28, 18, 17, 9, 5, 45, 21, 34, 64, 56, 50, 49, 45,
                                 31, 19, 12, 15, 10, 7, 6, 3, 48, 23, 20, 39, 36, 35, 53, 21, 16, 23, 13, 10, 6, 1, 4,
                                 2, 16, 15, 17, 27, 25,
                                 20, 29, 11, 17, 12, 16, 8, 1, 1, 0, 1};
static const HUFFBITS t15HB[] = {7, 12, 18, 53, 47, 76, 124, 108, 89, 123, 108, 119, 107, 81, 122, 63, 13, 5, 16, 27,
                                 46, 36, 61, 51, 42, 70, 52, 83, 65, 41, 59, 36, 19, 17, 15, 24, 41, 34, 59, 48, 40, 64,
                                 50, 78, 62, 80, 56,
                                 33, 29, 28, 25, 43, 39, 63, 55, 93, 76, 59, 93, 72, 54, 75, 50, 29, 52, 22, 42, 40, 67,
                                 57, 95, 79, 72, 57, 89, 69, 49, 66, 46, 27, 77, 37, 35, 66, 58, 52, 91, 74, 62, 48, 79,
                                 63, 90, 62, 40, 38,
                                 125, 32, 60, 56, 50, 92, 78, 65, 55, 87, 71, 51, 73, 51, 70, 30, 109, 53, 49, 94, 88,
                                 75, 66, 122, 91, 73, 56, 42, 64, 44, 21, 25, 90, 43, 41, 77, 73, 63, 56, 92, 77, 66,
                                 47, 67, 48, 53, 36, 20,
                                 71, 34, 67, 60, 58, 49, 88, 76, 67, 106, 71, 54, 38, 39, 23, 15, 109, 53, 51, 47, 90,
                                 82, 58, 57, 48, 72, 57, 41, 23, 27, 62, 9, 86, 42, 40, 37, 70, 64, 52, 43, 70, 55, 42,
                                 25, 29, 18, 11, 11,
                                 118, 68, 30, 55, 50, 46, 74, 65, 49, 39, 24, 16, 22, 13, 14, 7, 91, 44, 39, 38, 34, 63,
                                 52, 45, 31, 52, 28, 19, 14, 8, 9, 3, 123, 60, 58, 53, 47, 43, 32, 22, 37, 24, 17, 12,
                                 15, 10, 2, 1, 71,
                                 37, 34, 30, 28, 20, 17, 26, 21, 16, 10, 6, 8, 6, 2, 0};
static const HUFFBITS t16HB[] = {1, 5, 14, 44, 74, 63, 110, 93, 172, 149, 138, 242, 225, 195, 376, 17, 3, 4, 12, 20, 35,
                                 62, 53, 47, 83, 75, 68, 119, 201, 107, 207, 9, 15, 13, 23, 38, 67, 58, 103, 90, 161,
                                 72, 127, 117,
                                 110, 209, 206, 16, 45, 21, 39, 69, 64, 114, 99, 87, 158, 140, 252, 212, 199, 387, 365,
                                 26, 75, 36, 68, 65, 115, 101, 179, 164, 155, 264, 246, 226, 395, 382, 362, 9, 66, 30,
                                 59, 56, 102,
                                 185, 173, 265, 142, 253, 232, 400, 388, 378, 445, 16, 111, 54, 52, 100, 184, 178, 160,
                                 133, 257, 244, 228, 217, 385, 366, 715, 10, 98, 48, 91, 88, 165, 157, 148, 261, 248,
                                 407, 397, 372,
                                 380, 889, 884, 8, 85, 84, 81, 159, 156, 143, 260, 249, 427, 401, 392, 383, 727, 713,
                                 708, 7, 154, 76, 73, 141, 131, 256, 245, 426, 406, 394, 384, 735, 359, 710, 352, 11,
                                 139, 129, 67, 125,
                                 247, 233, 229, 219, 393, 743, 737, 720, 885, 882, 439, 4, 243, 120, 118, 115, 227, 223,
                                 396, 746, 742, 736, 721, 712, 706, 223, 436, 6, 202, 224, 222, 218, 216, 389, 386, 381,
                                 364, 888,
                                 443, 707, 440, 437, 1728, 4, 747, 211, 210, 208, 370, 379, 734, 723, 714, 1735, 883,
                                 877, 876, 3459, 865, 2, 377, 369, 102, 187, 726, 722, 358, 711, 709, 866, 1734, 871,
                                 3458, 870, 434,
                                 0, 12, 10, 7, 11, 10, 17, 11, 9, 13, 12, 10, 7, 5, 3, 1, 3};
static const HUFFBITS t24HB[] = {15, 13, 46, 80, 146, 262, 248, 434, 426, 669, 653, 649, 621, 517, 1032, 88, 14, 12, 21,
                                 38, 71, 130, 122, 216, 209, 198, 327, 345, 319, 297, 279, 42, 47, 22, 41, 74, 68, 128,
                                 120, 221,
                                 207, 194, 182, 340, 315, 295, 541, 18, 81, 39, 75, 70, 134, 125, 116, 220, 204, 190,
                                 178, 325, 311, 293, 271, 16, 147, 72, 69, 135, 127, 118, 112, 210, 200, 188, 352, 323,
                                 306, 285,
                                 540, 14, 263, 66, 129, 126, 119, 114, 214, 202, 192, 180, 341, 317, 301, 281, 262, 12,
                                 249, 123, 121, 117, 113, 215, 206, 195, 185, 347, 330, 308, 291, 272, 520, 10, 435,
                                 115, 111,
                                 109, 211, 203, 196, 187, 353, 332, 313, 298, 283, 531, 381, 17, 427, 212, 208, 205,
                                 201, 193, 186, 177, 169, 320, 303, 286, 268, 514, 377, 16, 335, 199, 197, 191, 189,
                                 181, 174, 333,
                                 321, 305, 289, 275, 521, 379, 371, 11, 668, 184, 183, 179, 175, 344, 331, 314, 304,
                                 290, 277, 530, 383, 373, 366, 10, 652, 346, 171, 168, 164, 318, 309, 299, 287, 276,
                                 263, 513, 375,
                                 368, 362, 6, 648, 322, 316, 312, 307, 302, 292, 284, 269, 261, 512, 376, 370, 364, 359,
                                 4, 620, 300, 296, 294, 288, 282, 273, 266, 515, 380, 374, 369, 365, 361, 357, 2, 1033,
                                 280, 278,
                                 274, 267, 264, 259, 382, 378, 372, 367, 363, 360, 358, 356, 0, 43, 20, 19, 17, 15, 13,
                                 11, 9, 7, 6, 4, 7, 5, 3, 1, 3};
static const HUFFBITS t32HB[] = {1, 5, 4, 5, 6, 5, 4, 4, 7, 3, 6, 0, 7, 2, 3, 1};
static const HUFFBITS t33HB[] = {15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0};

static const unsigned char t1l[] = {1, 3, 2, 3};
static const unsigned char t2l[] = {1, 3, 6, 3, 3, 5, 5, 5, 6};
static const unsigned char t3l[] = {2, 2, 6, 3, 2, 5, 5, 5, 6};
static const unsigned char t5l[] = {1, 3, 6, 7, 3, 3, 6, 7, 6, 6, 7, 8, 7, 6, 7, 8};
static const unsigned char t6l[] = {3, 3, 5, 7, 3, 2, 4, 5, 4, 4, 5, 6, 6, 5, 6, 7};
static const unsigned char t7l[] = {1, 3, 6, 8, 8, 9, 3, 4, 6, 7, 7, 8, 6, 5, 7, 8, 8, 9, 7, 7, 8, 9, 9, 9, 7, 7, 8, 9,
                                    9, 10, 8, 8, 9, 10, 10, 10};
static const unsigned char t8l[] = {2, 3, 6, 8, 8, 9, 3, 2, 4, 8, 8, 8, 6, 4, 6, 8, 8, 9, 8, 8, 8, 9, 9, 10, 8, 7, 8, 9,
                                    10, 10, 9, 8, 9, 9, 11, 11};
static const unsigned char t9l[] = {3, 3, 5, 6, 8, 9, 3, 3, 4, 5, 6, 8, 4, 4, 5, 6, 7, 8, 6, 5, 6, 7, 7, 8, 7, 6, 7, 7,
                                    8, 9, 8, 7, 8, 8, 9, 9};
static const unsigned char t10l[] = {1, 3, 6, 8, 9, 9, 9, 10, 3, 4, 6, 7, 8, 9, 8, 8, 6, 6, 7, 8, 9, 10, 9, 9, 7, 7, 8,
                                     9, 10, 10, 9, 10, 8, 8, 9, 10, 10, 10, 10, 10, 9, 9, 10, 10, 11, 11, 10, 11, 8, 8,
                                     9, 10, 10, 10, 11, 11, 9, 8, 9, 10, 10, 11, 11, 11};
static const unsigned char t11l[] = {2, 3, 5, 7, 8, 9, 8, 9, 3, 3, 4, 6, 8, 8, 7, 8, 5, 5, 6, 7, 8, 9, 8, 8, 7, 6, 7, 9,
                                     8, 10, 8, 9, 8, 8, 8, 9, 9, 10, 9, 10, 8, 8, 9, 10, 10, 11, 10, 11, 8, 7, 7, 8, 9,
                                     10, 10, 10, 8, 7, 8, 9, 10, 10, 10, 10};
static const unsigned char t12l[] = {4, 3, 5, 7, 8, 9, 9, 9, 3, 3, 4, 5, 7, 7, 8, 8, 5, 4, 5, 6, 7, 8, 7, 8, 6, 5, 6, 6,
                                     7, 8, 8, 8, 7, 6, 7, 7, 8, 8, 8, 9, 8, 7, 8, 8, 8, 9, 8, 9, 8, 7, 7, 8, 8, 9, 9,
                                     10, 9, 8, 8, 9, 9, 9, 9, 10};
static const unsigned char t13l[] = {1, 4, 6, 7, 8, 9, 9, 10, 9, 10, 11, 11, 12, 12, 13, 13, 3, 4, 6, 7, 8, 8, 9, 9, 9,
                                     9, 10, 10, 11, 12, 12, 12, 6, 6, 7, 8, 9, 9, 10, 10, 9, 10, 10, 11, 11, 12, 13, 13,
                                     7, 7, 8, 9, 9, 10, 10, 10, 10, 11, 11, 11, 11, 12, 13, 13,
                                     8, 7, 9, 9, 10, 10, 11, 11, 10, 11, 11, 12, 12, 13, 13, 14, 9, 8, 9, 10, 10, 10,
                                     11, 11, 11, 11, 12, 11, 13, 13, 14, 14, 9, 9, 10, 10, 11, 11, 11, 11, 11, 12, 12,
                                     12, 13, 13, 14, 14, 10, 9, 10, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 14, 16, 16,
                                     9, 8, 9, 10,
                                     10, 11, 11, 12, 12, 12, 12, 13, 13, 14, 15, 15, 10, 9, 10, 10, 11, 11, 11, 13, 12,
                                     13, 13, 14, 14, 14, 16, 15, 10, 10, 10, 11, 11, 12, 12, 13, 12, 13, 14, 13, 14, 15,
                                     16, 17, 11, 10, 10, 11, 12, 12, 12, 12, 13, 13, 13, 14, 15, 15, 15, 16, 11, 11, 11,
                                     12, 12,
                                     13, 12, 13, 14, 14, 15, 15, 15, 16, 16, 16, 12, 11, 12, 13, 13, 13, 14, 14, 14, 14,
                                     14, 15, 16, 15, 16, 16, 13, 12, 12, 13, 13, 13, 15, 14, 14, 17, 15, 15, 15, 17, 16,
                                     16, 12, 12, 13, 14, 14, 14, 15, 14, 15, 15, 16, 16, 19, 18, 19, 16};
static const unsigned char t15l[] = {3, 4, 5, 7, 7, 8, 9, 9, 9, 10, 10, 11, 11, 11, 12, 13, 4, 3, 5, 6, 7, 7, 8, 8, 8,
                                     9, 9, 10, 10, 10, 11, 11, 5, 5, 5, 6, 7, 7, 8, 8, 8, 9, 9, 10, 10, 11, 11, 11, 6,
                                     6, 6, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 7, 6, 7,
                                     7, 8, 8, 9, 9, 9, 9, 10, 10, 10, 11, 11, 11, 8, 7, 7, 8, 8, 8, 9, 9, 9, 9, 10, 10,
                                     11, 11, 11, 12, 9, 7, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 11, 11, 12, 12, 9, 8, 8, 9,
                                     9, 9, 9, 10, 10, 10, 10, 10, 11, 11, 11, 12, 9, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10,
                                     11, 11,
                                     12, 12, 12, 9, 8, 9, 9, 9, 9, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 10, 9, 9, 9,
                                     10, 10, 10, 10, 10, 11, 11, 11, 11, 12, 13, 12, 10, 9, 9, 9, 10, 10, 10, 10, 11,
                                     11, 11, 11, 12, 12, 12, 13, 11, 10, 9, 10, 10, 10, 11, 11, 11, 11, 11, 11, 12, 12,
                                     13, 13,
                                     11, 10, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12, 13, 13, 12, 11, 11, 11, 11,
                                     11, 11, 11, 12, 12, 12, 12, 13, 13, 12, 13, 12, 11, 11, 11, 11, 11, 11, 12, 12, 12,
                                     12, 12, 13, 13, 13, 13};
static const unsigned char t16l[] = {1, 4, 6, 8, 9, 9, 10, 10, 11, 11, 11, 12, 12, 12, 13, 9, 3, 4, 6, 7, 8, 9, 9, 9,
                                     10, 10, 10, 11, 12, 11, 12, 8, 6, 6, 7, 8, 9, 9, 10, 10, 11, 10, 11, 11, 11, 12,
                                     12, 9, 8, 7, 8, 9, 9, 10, 10, 10, 11, 11, 12, 12, 12, 13, 13,
                                     10, 9, 8, 9, 9, 10, 10, 11, 11, 11, 12, 12, 12, 13, 13, 13, 9, 9, 8, 9, 9, 10, 11,
                                     11, 12, 11, 12, 12, 13, 13, 13, 14, 10, 10, 9, 9, 10, 11, 11, 11, 11, 12, 12, 12,
                                     12, 13, 13, 14, 10, 10, 9, 10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 13, 15, 15, 10,
                                     10, 10,
                                     10, 11, 11, 11, 12, 12, 13, 13, 13, 13, 14, 14, 14, 10, 11, 10, 10, 11, 11, 12, 12,
                                     13, 13, 13, 13, 14, 13, 14, 13, 11, 11, 11, 10, 11, 12, 12, 12, 12, 13, 14, 14, 14,
                                     15, 15, 14, 10, 12, 11, 11, 11, 12, 12, 13, 14, 14, 14, 14, 14, 14, 13, 14, 11, 12,
                                     12,
                                     12, 12, 12, 13, 13, 13, 13, 15, 14, 14, 14, 14, 16, 11, 14, 12, 12, 12, 13, 13, 14,
                                     14, 14, 16, 15, 15, 15, 17, 15, 11, 13, 13, 11, 12, 14, 14, 13, 14, 14, 15, 16, 15,
                                     17, 15, 14, 11, 9, 8, 8, 9, 9, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 8};
static const unsigned char t24l[] = {4, 4, 6, 7, 8, 9, 9, 10, 10, 11, 11, 11, 11, 11, 12, 9, 4, 4, 5, 6, 7, 8, 8, 9, 9,
                                     9, 10, 10, 10, 10, 10, 8, 6, 5, 6, 7, 7, 8, 8, 9, 9, 9, 9, 10, 10, 10, 11, 7, 7, 6,
                                     7, 7, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 7, 8, 7, 7, 8,
                                     8, 8, 8, 9, 9, 9, 10, 10, 10, 10, 11, 7, 9, 7, 8, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10,
                                     10, 10, 7, 9, 8, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 10, 11, 7, 10, 8, 8, 8, 9, 9,
                                     9, 9, 10, 10, 10, 10, 10, 11, 11, 8, 10, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10,
                                     11, 11,
                                     8, 10, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 11, 11, 11, 8, 11, 9, 9, 9, 9, 10, 10,
                                     10, 10, 10, 10, 11, 11, 11, 11, 8, 11, 10, 9, 9, 9, 10, 10, 10, 10, 10, 10, 11, 11,
                                     11, 11, 8, 11, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 8, 11, 10,
                                     10,
                                     10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 8, 12, 10, 10, 10, 10, 10, 10, 11,
                                     11, 11, 11, 11, 11, 11, 11, 8, 8, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 4};
static const unsigned char t32l[] = {1, 4, 4, 5, 4, 6, 5, 6, 4, 5, 5, 6, 5, 6, 6, 6};
static const unsigned char t33l[] = {4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4};

#define NOREF -1
const struct huffcodetab shine_huffman_table[HTN] =
        {
                {0,  0,  0,  0, NULL, NULL},
                {2,  2,  0,  0,    t1HB,  t1l},
                {3,  3,  0,  0,    t2HB,  t2l},
                {3,  3,  0,  0,    t3HB,  t3l},
                {0,  0,  0,  0, NULL, NULL},/* Apparently not used*/
                {4,  4,  0,  0,    t5HB,  t5l},
                {4,  4,  0,  0,    t6HB,  t6l},
                {6,  6,  0,  0,    t7HB,  t7l},
                {6,  6,  0,  0,    t8HB,  t8l},
                {6,  6,  0,  0,    t9HB,  t9l},
                {8,  8,  0,  0,    t10HB, t10l},
                {8,  8,  0,  0,    t11HB, t11l},
                {8,  8,  0,  0,    t12HB, t12l},
                {16, 16, 0,  0,    t13HB, t13l},
                {0,  0,  0,  0, NULL, NULL},/* Apparently not used*/
                {16, 16, 0,  0,    t15HB, t15l},
                {16, 16, 1,  1,    t16HB, t16l},
                {16, 16, 2,  3,    t16HB, t16l},
                {16, 16, 3,  7,    t16HB, t16l},
                {16, 16, 4,  15,   t16HB, t16l},
                {16, 16, 6,  63,   t16HB, t16l},
                {16, 16, 8,  255,  t16HB, t16l},
                {16, 16, 10, 1023, t16HB, t16l},
                {16, 16, 13, 8191, t16HB, t16l},
                {16, 16, 4,  15,   t24HB, t24l},
                {16, 16, 5,  31,   t24HB, t24l},
                {16, 16, 6,  63,   t24HB, t24l},
                {16, 16, 7,  127,  t24HB, t24l},
                {16, 16, 8,  255,  t24HB, t24l},
                {16, 16, 9,  511,  t24HB, t24l},
                {16, 16, 11, 2047, t24HB, t24l},
                {16, 16, 13, 8191, t24HB, t24l},
                {1,  16, 0,  0,    t32HB, t32l},
                {1,  16, 0,  0,    t33HB, t33l},
        };


static int granules_per_frame[4] = {
        1,  /* MPEG 2.5 */
        -1,  /* Reserved */
        1,  /* MPEG II */
        2  /* MPEG I */
};

/* Set default values for important vars */
void shine_set_config_mpeg_defaults(shine_mpeg_t *mpeg) {
    mpeg->bitr = 64;
    mpeg->emph = NONE;
    mpeg->copyright = 0;
    mpeg->original = 1;
}

int shine_mpeg_version(int samplerate_index) {
    /* Pick mpeg version according to samplerate index. */
    if (samplerate_index < 3)
        /* First 3 samplerates are for MPEG-I */
        return MPEG_I;
    else if (samplerate_index < 6)
        /* Then it's MPEG-II */
        return MPEG_II;
    else
        /* Finally, MPEG-2.5 */
        return MPEG_25;
}

int shine_find_samplerate_index(int freq) {
    int i;

    for (i = 0; i < 9; i++)
        if (freq == samplerates[i]) return i;

    return -1; /* error - not a valid samplerate for encoder */
}

int shine_find_bitrate_index(int bitr, int mpeg_version) {
    int i;

    for (i = 0; i < 16; i++)
        if (bitr == bitrates[i][mpeg_version]) return i;

    return -1; /* error - not a valid samplerate for encoder */
}

int shine_check_config(int freq, int bitr) {
    int samplerate_index, bitrate_index, mpeg_version;

    samplerate_index = shine_find_samplerate_index(freq);
    if (samplerate_index < 0) return -1;

    mpeg_version = shine_mpeg_version(samplerate_index);

    bitrate_index = shine_find_bitrate_index(bitr, mpeg_version);
    if (bitrate_index < 0) return -1;

    return mpeg_version;
}

int shine_samples_per_pass(shine_t s) {
    return s->mpeg.granules_per_frame * GRANULE_SIZE;
}

/* Compute default encoding values. */
shine_global_config *shine_initialise(shine_config_t *pub_config) {
    double avg_slots_per_frame;
    shine_global_config *config;

    if (shine_check_config(pub_config->wave.samplerate, pub_config->mpeg.bitr) < 0)
        return NULL;

    config = rt_calloc(1, sizeof(shine_global_config));
    if (config == NULL)
        return config;

    shine_subband_initialise(config);
    shine_mdct_initialise(config);
    shine_loop_initialise(config);

    /* Copy public config. */
    config->wave.channels = pub_config->wave.channels;
    config->wave.samplerate = pub_config->wave.samplerate;
    config->mpeg.mode = pub_config->mpeg.mode;
    config->mpeg.bitr = pub_config->mpeg.bitr;
    config->mpeg.emph = pub_config->mpeg.emph;
    config->mpeg.copyright = pub_config->mpeg.copyright;
    config->mpeg.original = pub_config->mpeg.original;

    /* Set default values. */
    config->ResvMax = 0;
    config->ResvSize = 0;
    config->mpeg.layer = LAYER_III;
    config->mpeg.crc = 0;
    config->mpeg.ext = 0;
    config->mpeg.mode_ext = 0;
    config->mpeg.bits_per_slot = 8;

    config->mpeg.samplerate_index = shine_find_samplerate_index(config->wave.samplerate);
    config->mpeg.version = shine_mpeg_version(config->mpeg.samplerate_index);
    config->mpeg.bitrate_index = shine_find_bitrate_index(config->mpeg.bitr, config->mpeg.version);
    config->mpeg.granules_per_frame = granules_per_frame[config->mpeg.version];

    /* Figure average number of 'slots' per frame. */
    avg_slots_per_frame = ((double) config->mpeg.granules_per_frame * GRANULE_SIZE /
                           ((double) config->wave.samplerate)) *
                          (1000 * (double) config->mpeg.bitr /
                           (double) config->mpeg.bits_per_slot);

    config->mpeg.whole_slots_per_frame = (int) avg_slots_per_frame;

    config->mpeg.frac_slots_per_frame = avg_slots_per_frame - (double) config->mpeg.whole_slots_per_frame;
    config->mpeg.slot_lag = -config->mpeg.frac_slots_per_frame;

    if (config->mpeg.frac_slots_per_frame == 0)
        config->mpeg.padding = 0;

    shine_open_bit_stream(&config->bs, BUFFER_SIZE);

    memset((char *) &config->side_info, 0, sizeof(shine_side_info_t));

    /* determine the mean bitrate for main data */
    if (config->mpeg.granules_per_frame == 2) /* MPEG 1 */
        config->sideinfo_len = 8 * ((config->wave.channels == 1) ? 4 + 17 : 4 + 32);
    else                /* MPEG 2 */
        config->sideinfo_len = 8 * ((config->wave.channels == 1) ? 4 + 9 : 4 + 17);

    return config;
}

static unsigned char *shine_encode_buffer_internal(shine_global_config *config, int *written, int stride) {
    if (config->mpeg.frac_slots_per_frame) {
        config->mpeg.padding = (config->mpeg.slot_lag <= (config->mpeg.frac_slots_per_frame - 1.0));
        config->mpeg.slot_lag += (config->mpeg.padding - config->mpeg.frac_slots_per_frame);
    }

    config->mpeg.bits_per_frame = 8 * (config->mpeg.whole_slots_per_frame + config->mpeg.padding);
    config->mean_bits = (config->mpeg.bits_per_frame - config->sideinfo_len) / config->mpeg.granules_per_frame;

    /* apply mdct to the polyphase output */
    shine_mdct_sub(config, stride);

    /* bit and noise allocation */
    shine_iteration_loop(config);

    /* write the frame to the bitstream */
    shine_format_bitstream(config);

    /* Return data. */
    *written = config->bs.data_position;
    config->bs.data_position = 0;

    return config->bs.data;
}

unsigned char *shine_encode_buffer(shine_global_config *config, int16_t **data, int *written) {
    config->buffer[0] = data[0];
    if (config->wave.channels == 2)
        config->buffer[1] = data[1];

    return shine_encode_buffer_internal(config, written, 1);
}

unsigned char *shine_encode_buffer_interleaved(shine_global_config *config, int16_t *data, int *written) {
    config->buffer[0] = data;
    if (config->wave.channels == 2)
        config->buffer[1] = data + 1;

    return shine_encode_buffer_internal(config, written, config->wave.channels);
}

unsigned char *shine_flush(shine_global_config *config, int *written) {
    *written = config->bs.data_position;
    config->bs.data_position = 0;

    return config->bs.data;
}


void shine_close(shine_global_config *config) {
    shine_close_bit_stream(&config->bs);
    free(config);
}
/*
 *  bit_stream.c package
 *  Author:  Jean-Georges Fritsch, C-Cube Microsystems
 *
 * This package provides functions to write information to the bit stream.
 *
 * Removed unused functions. Feb 2001 P.Everett
 */



//#if !defined(__APPLE__)

//#include <malloc.h>

//#endif

/* open the device to write the bit stream into it */
void shine_open_bit_stream(bitstream_t *bs, int size) {
    bs->data = (unsigned char *) rt_malloc(size * sizeof(unsigned char));
    bs->data_size = size;
    bs->data_position = 0;
    bs->cache = 0;
    bs->cache_bits = 32;
}

/*close the device containing the bit stream */
void shine_close_bit_stream(bitstream_t *bs) {
    if (bs->data)
        rt_free(bs->data);
}

/*
 * shine_putbits:
 * --------
 * write N bits into the bit stream.
 * bs = bit stream structure
 * val = value to write into the buffer
 * N = number of bits of val
 */
void shine_putbits(bitstream_t *bs, unsigned int val, unsigned int N) {
#ifdef DEBUG
    if (N > 32)
        printf("Cannot write more than 32 bits at a time.\n");
    if (N < 32 && (val >> N) != 0)
        printf("Upper bits (higher than %d) are not all zeros.\n", N);
#endif

    if (bs->cache_bits > N) {
        bs->cache_bits -= N;
        bs->cache |= val << bs->cache_bits;
    } else {
        if (bs->data_position + sizeof(unsigned int) >= bs->data_size) {
            bs->data = (unsigned char *) realloc(bs->data, bs->data_size + (bs->data_size / 2));
            bs->data_size += (bs->data_size / 2);
        }

        N -= bs->cache_bits;
        bs->cache |= val >> N;
#ifdef SHINE_BIG_ENDIAN
        *(unsigned int*)(bs->data + bs->data_position) = bs->cache;
#else
        *(unsigned int *) (bs->data + bs->data_position) = SWAB32(bs->cache);
#endif
        bs->data_position += sizeof(unsigned int);
        bs->cache_bits = 32 - N;
        if (N != 0)
            bs->cache = val << bs->cache_bits;
        else
            bs->cache = 0;
    }
}

int shine_get_bits_count(bitstream_t *bs) {
    return bs->data_position * 8 + 32 - bs->cache_bits;
}


static void shine_HuffmanCode(bitstream_t *bs, int table_select, int x, int y);

static void shine_huffman_coder_count1(bitstream_t *bs, const struct huffcodetab *h, int v, int w, int x, int y);

static void encodeSideInfo(shine_global_config *config);

static void encodeMainData(shine_global_config *config);

static void Huffmancodebits(shine_global_config *config, int *ix, gr_info *gi);

/*
  shine_format_bitstream()

  This is called after a frame of audio has been quantized and coded.
  It will write the encoded audio to the bitstream. Note that
  from a layer3 encoder's perspective the bit stream is primarily
  a series of main_data() blocks, with header and side information
  inserted at the proper locations to maintain framing. (See Figure A.7
  in the IS).
*/

void
shine_format_bitstream(shine_global_config *config) {
    int gr, ch, i;

    for (ch = 0; ch < config->wave.channels; ch++)
        for (gr = 0; gr < config->mpeg.granules_per_frame; gr++) {
            int *pi = &config->l3_enc[ch][gr][0];
            int32_t *pr = &config->mdct_freq[ch][gr][0];
            for (i = 0; i < GRANULE_SIZE; i++) {
                if ((pr[i] < 0) && (pi[i] > 0))
                    pi[i] *= -1;
            }
        }

    encodeSideInfo(config);
    encodeMainData(config);
}

static void encodeMainData(shine_global_config *config) {
    int gr, ch, sfb;
    shine_side_info_t si = config->side_info;

    for (gr = 0; gr < config->mpeg.granules_per_frame; gr++) {
        for (ch = 0; ch < config->wave.channels; ch++) {
            gr_info *gi = &(si.gr[gr].ch[ch].tt);
            unsigned slen1 = shine_slen1_tab[gi->scalefac_compress];
            unsigned slen2 = shine_slen2_tab[gi->scalefac_compress];
            int *ix = &config->l3_enc[ch][gr][0];

            if (gr == 0 || si.scfsi[ch][0] == 0)
                for (sfb = 0; sfb < 6; sfb++)
                    shine_putbits(&config->bs, config->scalefactor.l[gr][ch][sfb], slen1);
            if (gr == 0 || si.scfsi[ch][1] == 0)
                for (sfb = 6; sfb < 11; sfb++)
                    shine_putbits(&config->bs, config->scalefactor.l[gr][ch][sfb], slen1);
            if (gr == 0 || si.scfsi[ch][2] == 0)
                for (sfb = 11; sfb < 16; sfb++)
                    shine_putbits(&config->bs, config->scalefactor.l[gr][ch][sfb], slen2);
            if (gr == 0 || si.scfsi[ch][3] == 0)
                for (sfb = 16; sfb < 21; sfb++)
                    shine_putbits(&config->bs, config->scalefactor.l[gr][ch][sfb], slen2);

            Huffmancodebits(config, ix, gi);
        }
    }
}

static void encodeSideInfo(shine_global_config *config) {
    int gr, ch, scfsi_band, region;
    shine_side_info_t si = config->side_info;

    shine_putbits(&config->bs, 0x7ff, 11);
    shine_putbits(&config->bs, config->mpeg.version, 2);
    shine_putbits(&config->bs, config->mpeg.layer, 2);
    shine_putbits(&config->bs, !config->mpeg.crc, 1);
    shine_putbits(&config->bs, config->mpeg.bitrate_index, 4);
    shine_putbits(&config->bs, config->mpeg.samplerate_index % 3, 2);
    shine_putbits(&config->bs, config->mpeg.padding, 1);
    shine_putbits(&config->bs, config->mpeg.ext, 1);
    shine_putbits(&config->bs, config->mpeg.mode, 2);
    shine_putbits(&config->bs, config->mpeg.mode_ext, 2);
    shine_putbits(&config->bs, config->mpeg.copyright, 1);
    shine_putbits(&config->bs, config->mpeg.original, 1);
    shine_putbits(&config->bs, config->mpeg.emph, 2);

    if (config->mpeg.version == MPEG_I) {
        shine_putbits(&config->bs, 0, 9);
        if (config->wave.channels == 2)
            shine_putbits(&config->bs, si.private_bits, 3);
        else
            shine_putbits(&config->bs, si.private_bits, 5);
    } else {
        shine_putbits(&config->bs, 0, 8);
        if (config->wave.channels == 2)
            shine_putbits(&config->bs, si.private_bits, 2);
        else
            shine_putbits(&config->bs, si.private_bits, 1);
    }

    if (config->mpeg.version == MPEG_I)
        for (ch = 0; ch < config->wave.channels; ch++) {
            for (scfsi_band = 0; scfsi_band < 4; scfsi_band++)
                shine_putbits(&config->bs, si.scfsi[ch][scfsi_band], 1);
        }

    for (gr = 0; gr < config->mpeg.granules_per_frame; gr++)
        for (ch = 0; ch < config->wave.channels; ch++) {
            gr_info *gi = &(si.gr[gr].ch[ch].tt);

            shine_putbits(&config->bs, gi->part2_3_length, 12);
            shine_putbits(&config->bs, gi->big_values, 9);
            shine_putbits(&config->bs, gi->global_gain, 8);
            if (config->mpeg.version == MPEG_I)
                shine_putbits(&config->bs, gi->scalefac_compress, 4);
            else
                shine_putbits(&config->bs, gi->scalefac_compress, 9);
            shine_putbits(&config->bs, 0, 1);

            for (region = 0; region < 3; region++)
                shine_putbits(&config->bs, gi->table_select[region], 5);

            shine_putbits(&config->bs, gi->region0_count, 4);
            shine_putbits(&config->bs, gi->region1_count, 3);

            if (config->mpeg.version == MPEG_I)
                shine_putbits(&config->bs, gi->preflag, 1);
            shine_putbits(&config->bs, gi->scalefac_scale, 1);
            shine_putbits(&config->bs, gi->count1table_select, 1);
        }
}

/* Note the discussion of huffmancodebits() on pages 28 and 29 of the IS, as
  well as the definitions of the side information on pages 26 and 27. */
static void Huffmancodebits(shine_global_config *config, int *ix, gr_info *gi) {
    const int *scalefac = &shine_scale_fact_band_index[config->mpeg.samplerate_index][0];
    unsigned scalefac_index;
    int region1Start, region2Start;
    int i, bigvalues, count1End;
    int v, w, x, y;
    const struct huffcodetab *h;
    int bits;

    bits = shine_get_bits_count(&config->bs);

    /* 1: Write the bigvalues */
    bigvalues = gi->big_values << 1;

    scalefac_index = gi->region0_count + 1;
    region1Start = scalefac[scalefac_index];
    scalefac_index += gi->region1_count + 1;
    region2Start = scalefac[scalefac_index];

    for (i = 0; i < bigvalues; i += 2) {
        /* get table pointer */
        int idx = (i >= region1Start) + (i >= region2Start);
        unsigned tableindex = gi->table_select[idx];
        /* get huffman code */
        if (tableindex) {
            x = ix[i];
            y = ix[i + 1];
            shine_HuffmanCode(&config->bs, tableindex, x, y);
        }
    }

    /* 2: Write count1 area */
    h = &shine_huffman_table[gi->count1table_select + 32];
    count1End = bigvalues + (gi->count1 << 2);
    for (i = bigvalues; i < count1End; i += 4) {
        v = ix[i];
        w = ix[i + 1];
        x = ix[i + 2];
        y = ix[i + 3];
        shine_huffman_coder_count1(&config->bs, h, v, w, x, y);
    }

    bits = shine_get_bits_count(&config->bs) - bits;
    bits = gi->part2_3_length - gi->part2_length - bits;
    if (bits) {
        int stuffingWords = bits / 32;
        int remainingBits = bits % 32;

        /* Due to the nature of the Huffman code tables, we will pad with ones */
        while (stuffingWords--)
            shine_putbits(&config->bs, ~0, 32);
        if (remainingBits)
            shine_putbits(&config->bs, (1UL << remainingBits) - 1, remainingBits);
    }
}

static inline int shine_abs_and_sign(int *x) {
    if (*x > 0) return 0;
    *x *= -1;
    return 1;
}

static void shine_huffman_coder_count1(bitstream_t *bs, const struct huffcodetab *h, int v, int w, int x, int y) {
    unsigned int signv, signw, signx, signy;
    unsigned int code = 0;
    int p, cbits = 0;

    signv = shine_abs_and_sign(&v);
    signw = shine_abs_and_sign(&w);
    signx = shine_abs_and_sign(&x);
    signy = shine_abs_and_sign(&y);

    p = v + (w << 1) + (x << 2) + (y << 3);
    shine_putbits(bs, h->table[p], h->hlen[p]);

    if (v) {
        code = signv;
        cbits = 1;
    }
    if (w) {
        code = (code << 1) | signw;
        cbits++;
    }
    if (x) {
        code = (code << 1) | signx;
        cbits++;
    }
    if (y) {
        code = (code << 1) | signy;
        cbits++;
    }
    shine_putbits(bs, code, cbits);
}

/* Implements the pseudocode of page 98 of the IS */
static void shine_HuffmanCode(bitstream_t *bs, int table_select, int x, int y) {
    int cbits = 0, xbits = 0;
    unsigned int code = 0, ext = 0;
    unsigned signx, signy, ylen, idx;
    const struct huffcodetab *h;

    signx = shine_abs_and_sign(&x);
    signy = shine_abs_and_sign(&y);

    h = &(shine_huffman_table[table_select]);
    ylen = h->ylen;

    if (table_select > 15) { /* ESC-table is used */
        unsigned linbitsx = 0, linbitsy = 0, linbits = h->linbits;

        if (x > 14) {
            linbitsx = x - 15;
            x = 15;
        }
        if (y > 14) {
            linbitsy = y - 15;
            y = 15;
        }

        idx = (x * ylen) + y;
        code = h->table[idx];
        cbits = h->hlen[idx];
        if (x > 14) {
            ext |= linbitsx;
            xbits += linbits;
        }
        if (x != 0) {
            ext <<= 1;
            ext |= signx;
            xbits += 1;
        }
        if (y > 14) {
            ext <<= linbits;
            ext |= linbitsy;
            xbits += linbits;
        }
        if (y != 0) {
            ext <<= 1;
            ext |= signy;
            xbits += 1;
        }

        shine_putbits(bs, code, cbits);
        shine_putbits(bs, ext, xbits);
    } else { /* No ESC-words */
        idx = (x * ylen) + y;
        code = h->table[idx];
        cbits = h->hlen[idx];
        if (x != 0) {
            code <<= 1;
            code |= signx;
            cbits += 1;
        }
        if (y != 0) {
            code <<= 1;
            code |= signy;
            cbits += 1;
        }

        shine_putbits(bs, code, cbits);
    }
}

#define e        2.71828182845
#define CBLIMIT  21
#define SFB_LMAX 22
#define en_tot_krit 10
#define en_dif_krit 100
#define en_scfsi_band_krit 10
#define xm_scfsi_band_krit 10

static void calc_scfsi(shine_psy_xmin_t *l3_xmin, int ch, int gr, shine_global_config *config);

static int part2_length(int gr, int ch, shine_global_config *config);

static int bin_search_StepSize(int desired_rate, int ix[GRANULE_SIZE], gr_info *cod_info, shine_global_config *config);

static int count_bit(int ix[GRANULE_SIZE], unsigned int start, unsigned int end, unsigned int table);

static int bigv_bitcount(int ix[GRANULE_SIZE], gr_info *gi);

static int new_choose_table(int ix[GRANULE_SIZE], unsigned int begin, unsigned int end);

static void bigv_tab_select(int ix[GRANULE_SIZE], gr_info *cod_info);

static void subdivide(gr_info *cod_info, shine_global_config *config);

static int count1_bitcount(int ix[GRANULE_SIZE], gr_info *cod_info);

static void calc_runlen(int ix[GRANULE_SIZE], gr_info *cod_info);

static void calc_xmin(shine_psy_ratio_t *ratio, gr_info *cod_info, shine_psy_xmin_t *l3_xmin, int gr, int ch);

static int quantize(int ix[GRANULE_SIZE], int stepsize, shine_global_config *config);

/*
 * shine_inner_loop:
 * ----------
 * The code selects the best quantizerStepSize for a particular set
 * of scalefacs.
 */
int shine_inner_loop(int ix[GRANULE_SIZE],
                     int max_bits, gr_info *cod_info, int gr, int ch,
                     shine_global_config *config) {
    int bits, c1bits, bvbits;

    if (max_bits < 0)
        cod_info->quantizerStepSize--;
    do {
        while (quantize(ix, ++cod_info->quantizerStepSize, config) > 8192); /* within table range? */

        calc_runlen(ix, cod_info);                        /* rzero,count1,big_values*/
        bits = c1bits = count1_bitcount(ix, cod_info);    /* count1_table selection*/
        subdivide(cod_info, config);                     /* bigvalues sfb division */
        bigv_tab_select(ix, cod_info);                    /* codebook selection*/
        bits += bvbits = bigv_bitcount(ix, cod_info);  /* bit count */
    } while (bits > max_bits);
    return bits;
}

/*
 * shine_outer_loop:
 * -----------
 *  Function: The outer iteration loop controls the masking conditions
 *  of all scalefactorbands. It computes the best scalefac and
 *  global gain. This module calls the inner iteration loop.
 */

int shine_outer_loop(int max_bits,
                     shine_psy_xmin_t *l3_xmin, /* the allowed distortion of the scalefactor */
                     int ix[GRANULE_SIZE], /* vector of quantized values ix(0..575) */
                     int gr, int ch, shine_global_config *config) {
    int bits, huff_bits;
    shine_side_info_t *side_info = &config->side_info;
    gr_info *cod_info = &side_info->gr[gr].ch[ch].tt;

    cod_info->quantizerStepSize = bin_search_StepSize(max_bits, ix, cod_info, config);

    cod_info->part2_length = part2_length(gr, ch, config);
    huff_bits = max_bits - cod_info->part2_length;

    bits = shine_inner_loop(ix, huff_bits, cod_info, gr, ch, config);
    cod_info->part2_3_length = cod_info->part2_length + bits;

    return cod_info->part2_3_length;
}

/*
 * shine_iteration_loop:
 * ------------------
 */
void shine_iteration_loop(shine_global_config *config) {
    shine_psy_xmin_t l3_xmin;
    gr_info *cod_info;
    int max_bits;
    int ch, gr, i;
    int *ix;

    for (ch = config->wave.channels; ch--;) {
        for (gr = 0; gr < config->mpeg.granules_per_frame; gr++) {
            /* setup pointers */
            ix = config->l3_enc[ch][gr];
            config->l3loop.xr = config->mdct_freq[ch][gr];

            /* Precalculate the square, abs,  and maximum,
             * for use later on.
             */
            for (i = GRANULE_SIZE, config->l3loop.xrmax = 0; i--;) {
                config->l3loop.xrsq[i] = mulsr(config->l3loop.xr[i], config->l3loop.xr[i]);
                config->l3loop.xrabs[i] = labs(config->l3loop.xr[i]);
                if (config->l3loop.xrabs[i] > config->l3loop.xrmax)
                    config->l3loop.xrmax = config->l3loop.xrabs[i];
            }

            cod_info = (gr_info *) &(config->side_info.gr[gr].ch[ch]);
            cod_info->sfb_lmax = SFB_LMAX - 1; /* gr_deco */

            calc_xmin(&config->ratio, cod_info, &l3_xmin, gr, ch);

            if (config->mpeg.version == MPEG_I)
                calc_scfsi(&l3_xmin, ch, gr, config);

            /* calculation of number of available bit( per granule ) */
            max_bits = shine_max_reservoir_bits(&config->pe[ch][gr], config);

            /* reset of iteration variables */
            memset(config->scalefactor.l[gr][ch], 0, sizeof(config->scalefactor.l[gr][ch]));
            memset(config->scalefactor.s[gr][ch], 0, sizeof(config->scalefactor.s[gr][ch]));

            for (i = 4; i--;)
                cod_info->slen[i] = 0;

            cod_info->part2_3_length = 0;
            cod_info->big_values = 0;
            cod_info->count1 = 0;
            cod_info->scalefac_compress = 0;
            cod_info->table_select[0] = 0;
            cod_info->table_select[1] = 0;
            cod_info->table_select[2] = 0;
            cod_info->region0_count = 0;
            cod_info->region1_count = 0;
            cod_info->part2_length = 0;
            cod_info->preflag = 0;
            cod_info->scalefac_scale = 0;
            cod_info->count1table_select = 0;

            /* all spectral values zero ? */
            if (config->l3loop.xrmax)
                cod_info->part2_3_length = shine_outer_loop(max_bits, &l3_xmin, ix,
                                                            gr, ch, config);

            shine_ResvAdjust(cod_info, config);
            cod_info->global_gain = cod_info->quantizerStepSize + 210;

        } /* for gr */
    } /* for ch */

    shine_ResvFrameEnd(config);
}

/*
 * calc_scfsi:
 * -----------
 * calculation of the scalefactor select information ( scfsi ).
 */
void calc_scfsi(shine_psy_xmin_t *l3_xmin, int ch, int gr,
                shine_global_config *config) {
    shine_side_info_t *l3_side = &config->side_info;
    /* This is the scfsi_band table from 2.4.2.7 of the IS */
    static const int scfsi_band_long[5] = {0, 6, 11, 16, 21};

    int scfsi_band;
    unsigned scfsi_set;

    int sfb, start, end, i;
    int condition = 0;
    int temp;

    const int *scalefac_band_long = &shine_scale_fact_band_index[config->mpeg.samplerate_index][0];

    /* note. it goes quite a bit faster if you uncomment the next bit and exit
       early from scfsi, but you then loose the advantage of common scale factors.

    for(scfsi_band=0;scfsi_band<4;scfsi_band++)
      l3_side->scfsi[ch][scfsi_band] = 0;
    return;

    */

    config->l3loop.xrmaxl[gr] = config->l3loop.xrmax;
    scfsi_set = 0;

    /* the total energy of the granule */
    for (temp = 0, i = GRANULE_SIZE; i--;)
        temp += config->l3loop.xrsq[i] >> 10; /* a bit of scaling to avoid overflow, (not very good) */
    if (temp)
        config->l3loop.en_tot[gr] = log((double) temp * 4.768371584e-7) / LN2; /* 1024 / 0x7fffffff */
    else
        config->l3loop.en_tot[gr] = 0;

    /* the energy of each scalefactor band, en */
    /* the allowed distortion of each scalefactor band, xm */

    for (sfb = 21; sfb--;) {
        start = scalefac_band_long[sfb];
        end = scalefac_band_long[sfb + 1];

        for (temp = 0, i = start; i < end; i++)
            temp += config->l3loop.xrsq[i] >> 10;
        if (temp)
            config->l3loop.en[gr][sfb] = log((double) temp * 4.768371584e-7) / LN2; /* 1024 / 0x7fffffff */
        else
            config->l3loop.en[gr][sfb] = 0;

        if (l3_xmin->l[gr][ch][sfb])
            config->l3loop.xm[gr][sfb] = log(l3_xmin->l[gr][ch][sfb]) / LN2;
        else
            config->l3loop.xm[gr][sfb] = 0;
    }

    if (gr == 1) {
        int gr2, tp;

        for (gr2 = 2; gr2--;) {
            /* The spectral values are not all zero */
            if (config->l3loop.xrmaxl[gr2])
                condition++;

            condition++;
        }
        if (abs(config->l3loop.en_tot[0] - config->l3loop.en_tot[1]) < en_tot_krit)
            condition++;
        for (tp = 0, sfb = 21; sfb--;)
            tp += abs(config->l3loop.en[0][sfb] - config->l3loop.en[1][sfb]);
        if (tp < en_dif_krit)
            condition++;

        if (condition == 6) {
            for (scfsi_band = 0; scfsi_band < 4; scfsi_band++) {
                int sum0 = 0, sum1 = 0;
                l3_side->scfsi[ch][scfsi_band] = 0;
                start = scfsi_band_long[scfsi_band];
                end = scfsi_band_long[scfsi_band + 1];
                for (sfb = start; sfb < end; sfb++) {
                    sum0 += abs(config->l3loop.en[0][sfb] - config->l3loop.en[1][sfb]);
                    sum1 += abs(config->l3loop.xm[0][sfb] - config->l3loop.xm[1][sfb]);
                }

                if (sum0 < en_scfsi_band_krit && sum1 < xm_scfsi_band_krit) {
                    l3_side->scfsi[ch][scfsi_band] = 1;
                    scfsi_set |= (1 << scfsi_band);
                } else
                    l3_side->scfsi[ch][scfsi_band] = 0;
            } /* for scfsi_band */
        } /* if condition == 6 */
        else
            for (scfsi_band = 0; scfsi_band < 4; scfsi_band++)
                l3_side->scfsi[ch][scfsi_band] = 0;
    } /* if gr == 1 */
}

/*
 * part2_length:
 * -------------
 * calculates the number of bits needed to encode the scalefacs in the
 * main data block.
 */
int part2_length(int gr, int ch, shine_global_config *config) {
    int slen1, slen2, bits;
    gr_info *gi = &config->side_info.gr[gr].ch[ch].tt;

    bits = 0;

    {
        slen1 = shine_slen1_tab[gi->scalefac_compress];
        slen2 = shine_slen2_tab[gi->scalefac_compress];

        if (!gr || !(config->side_info.scfsi[ch][0]))
            bits += (6 * slen1);

        if (!gr || !(config->side_info.scfsi[ch][1]))
            bits += (5 * slen1);

        if (!gr || !(config->side_info.scfsi[ch][2]))
            bits += (5 * slen2);

        if (!gr || !(config->side_info.scfsi[ch][3]))
            bits += (5 * slen2);
    }
    return bits;
}

/*
 * calc_xmin:
 * ----------
 * Calculate the allowed distortion for each scalefactor band,
 * as determined by the psychoacoustic model.
 * xmin(sb) = ratio(sb) * en(sb) / bw(sb)
 */
void calc_xmin(shine_psy_ratio_t *ratio,
               gr_info *cod_info,
               shine_psy_xmin_t *l3_xmin,
               int gr, int ch) {
    int sfb;

    for (sfb = cod_info->sfb_lmax; sfb--;) {
/*  note. xmin will always be zero with no psychoacoustic model

    start = scalefac_band_long[ sfb ];
    end   = scalefac_band_long[ sfb+1 ];
    bw = end - start;

    for ( en = 0, l = start; l < end; l++ )
      en += config->l3loop.xrsq[l];

    l3_xmin->l[gr][ch][sfb] = ratio->l[gr][ch][sfb] * en / bw;
*/
        l3_xmin->l[gr][ch][sfb] = 0;
    }
}

/*
 * shine_loop_initialise:
 * -------------------
 * Calculates the look up tables used by the iteration loop.
 */
void shine_loop_initialise(shine_global_config *config) {
    int i;

    /* quantize: stepsize conversion, fourth root of 2 table.
     * The table is inverted (negative power) from the equation given
     * in the spec because it is quicker to do x*y than x/y.
     * The 0.5 is for rounding.
     */
    for (i = 128; i--;) {
        config->l3loop.steptab[i] = pow(2.0, (double) (127 - i) / 4);
        if ((config->l3loop.steptab[i] * 2) > 0x7fffffff) /* MAXINT = 2**31 = 2**(124/4) */
            config->l3loop.steptabi[i] = 0x7fffffff;
        else
            /* The table is multiplied by 2 to give an extra bit of accuracy.
             * In quantize, the long multiply does not shift it's result left one
             * bit to compensate.
             */
            config->l3loop.steptabi[i] = (int32_t) ((config->l3loop.steptab[i] * 2) + 0.5);
    }

    /* quantize: vector conversion, three quarter power table.
     * The 0.5 is for rounding, the .0946 comes from the spec.
     */
    for (i = 10000; i--;)
        config->l3loop.int2idx[i] = (int) (sqrt(sqrt((double) i) * (double) i) - 0.0946 + 0.5);
}

/*
 * quantize:
 * ---------
 * Function: Quantization of the vector xr ( -> ix).
 * Returns maximum value of ix.
 */
int quantize(int ix[GRANULE_SIZE], int stepsize, shine_global_config *config) {
    int i, max, ln;
    int32_t scalei;
    double scale, dbl;

    scalei = config->l3loop.steptabi[stepsize + 127]; /* 2**(-stepsize/4) */

    /* a quick check to see if ixmax will be less than 8192 */
    /* this speeds up the early calls to bin_search_StepSize */
    if ((mulr(config->l3loop.xrmax, scalei)) > 165140) /* 8192**(4/3) */
        max = 16384; /* no point in continuing, stepsize not big enough */
    else
        for (i = 0, max = 0; i < GRANULE_SIZE; i++) {
            /* This calculation is very sensitive. The multiply must round it's
             * result or bad things happen to the quality.
             */
            ln = mulr(labs(config->l3loop.xr[i]), scalei);

            if (ln < 10000) /* ln < 10000 catches most values */
                ix[i] = config->l3loop.int2idx[ln]; /* quick look up method */
            else {
                /* outside table range so have to do it using floats */
                scale = config->l3loop.steptab[stepsize + 127]; /* 2**(-stepsize/4) */
                dbl = ((double) config->l3loop.xrabs[i]) * scale * 4.656612875e-10; /* 0x7fffffff */
                ix[i] = (int) sqrt(sqrt(dbl) * dbl); /* dbl**(3/4) */
            }

            /* calculate ixmax while we're here */
            /* note. ix cannot be negative */
            if (max < ix[i])
                max = ix[i];
        }

    return max;
}

/*
 * ix_max:
 * -------
 * Function: Calculate the maximum of ix from 0 to 575
 */
static inline int ix_max(int ix[GRANULE_SIZE], unsigned int begin, unsigned int end) {
    register int i;
    register int max = 0;

    for (i = begin; i < end; i++)
        if (max < ix[i])
            max = ix[i];
    return max;
}

/*
 * calc_runlen:
 * ------------
 * Function: Calculation of rzero, count1, big_values
 * (Partitions ix into big values, quadruples and zeros).
 */
void calc_runlen(int ix[GRANULE_SIZE], gr_info *cod_info) {
    int i;
    int rzero = 0;

    for (i = GRANULE_SIZE; i > 1; i -= 2)
        if (!ix[i - 1] && !ix[i - 2])
            rzero++;
        else
            break;

    cod_info->count1 = 0;
    for (; i > 3; i -= 4)
        if (ix[i - 1] <= 1
            && ix[i - 2] <= 1
            && ix[i - 3] <= 1
            && ix[i - 4] <= 1)
            cod_info->count1++;
        else
            break;

    cod_info->big_values = i >> 1;
}

/*
 * count1_bitcount:
 * ----------------
 * Determines the number of bits to encode the quadruples.
 */
int count1_bitcount(int ix[GRANULE_SIZE], gr_info *cod_info) {
    int p, i, k;
    int v, w, x, y, signbits;
    int sum0 = 0,
            sum1 = 0;

    for (i = cod_info->big_values << 1, k = 0; k < cod_info->count1; i += 4, k++) {
        v = ix[i];
        w = ix[i + 1];
        x = ix[i + 2];
        y = ix[i + 3];

        p = v + (w << 1) + (x << 2) + (y << 3);

        signbits = 0;
        if (v != 0) signbits++;
        if (w != 0) signbits++;
        if (x != 0) signbits++;
        if (y != 0) signbits++;

        sum0 += signbits;
        sum1 += signbits;

        sum0 += shine_huffman_table[32].hlen[p];
        sum1 += shine_huffman_table[33].hlen[p];
    }

    if (sum0 < sum1) {
        cod_info->count1table_select = 0;
        return sum0;
    } else {
        cod_info->count1table_select = 1;
        return sum1;
    }
}

/*
 * subdivide:
 * ----------
 * presumable subdivides the bigvalue region which will use separate Huffman tables.
 */
void subdivide(gr_info *cod_info, shine_global_config *config) {
    static const struct {
        unsigned region0_count;
        unsigned region1_count;
    } subdv_table[23] =
            {
                    {0, 0}, /* 0 bands */
                    {0, 0}, /* 1 bands */
                    {0, 0}, /* 2 bands */
                    {0, 0}, /* 3 bands */
                    {0, 0}, /* 4 bands */
                    {0, 1}, /* 5 bands */
                    {1, 1}, /* 6 bands */
                    {1, 1}, /* 7 bands */
                    {1, 2}, /* 8 bands */
                    {2, 2}, /* 9 bands */
                    {2, 3}, /* 10 bands */
                    {2, 3}, /* 11 bands */
                    {3, 4}, /* 12 bands */
                    {3, 4}, /* 13 bands */
                    {3, 4}, /* 14 bands */
                    {4, 5}, /* 15 bands */
                    {4, 5}, /* 16 bands */
                    {4, 6}, /* 17 bands */
                    {5, 6}, /* 18 bands */
                    {5, 6}, /* 19 bands */
                    {5, 7}, /* 20 bands */
                    {6, 7}, /* 21 bands */
                    {6, 7}, /* 22 bands */
            };

    if (!cod_info->big_values) { /* no big_values region */
        cod_info->region0_count = 0;
        cod_info->region1_count = 0;
    } else {
        const int *scalefac_band_long = &shine_scale_fact_band_index[config->mpeg.samplerate_index][0];
        int bigvalues_region, scfb_anz, thiscount;

        bigvalues_region = 2 * cod_info->big_values;

        /* Calculate scfb_anz */
        scfb_anz = 0;
        while (scalefac_band_long[scfb_anz] < bigvalues_region)
            scfb_anz++;

        for (thiscount = subdv_table[scfb_anz].region0_count; thiscount; thiscount--) {
            if (scalefac_band_long[thiscount + 1] <= bigvalues_region)
                break;
        }
        cod_info->region0_count = thiscount;
        cod_info->address1 = scalefac_band_long[thiscount + 1];

        scalefac_band_long += cod_info->region0_count + 1;

        for (thiscount = subdv_table[scfb_anz].region1_count; thiscount; thiscount--) {
            if (scalefac_band_long[thiscount + 1] <= bigvalues_region)
                break;
        }
        cod_info->region1_count = thiscount;
        cod_info->address2 = scalefac_band_long[thiscount + 1];

        cod_info->address3 = bigvalues_region;
    }
}

/*
 * bigv_tab_select:
 * ----------------
 * Function: Select huffman code tables for bigvalues regions
 */
void bigv_tab_select(int ix[GRANULE_SIZE], gr_info *cod_info) {
    cod_info->table_select[0] = 0;
    cod_info->table_select[1] = 0;
    cod_info->table_select[2] = 0;

    {
        if (cod_info->address1 > 0)
            cod_info->table_select[0] = new_choose_table(ix, 0, cod_info->address1);

        if (cod_info->address2 > cod_info->address1)
            cod_info->table_select[1] = new_choose_table(ix, cod_info->address1, cod_info->address2);

        if (cod_info->big_values << 1 > cod_info->address2)
            cod_info->table_select[2] = new_choose_table(ix, cod_info->address2, cod_info->big_values << 1);
    }
}

/*
 * new_choose_table:
 * -----------------
 * Choose the Huffman table that will encode ix[begin..end] with
 * the fewest bits.
 * Note: This code contains knowledge about the sizes and characteristics
 * of the Huffman tables as defined in the IS (Table B.7), and will not work
 * with any arbitrary tables.
 */
int new_choose_table(int ix[GRANULE_SIZE], unsigned int begin, unsigned int end) {
    int i, max;
    int choice[2];
    int sum[2];

    max = ix_max(ix, begin, end);
    if (!max)
        return 0;

    choice[0] = 0;
    choice[1] = 0;

    if (max < 15) {
        /* try tables with no linbits */
        for (i = 14; i--;)
            if (shine_huffman_table[i].xlen > max) {
                choice[0] = i;
                break;
            }

        sum[0] = count_bit(ix, begin, end, choice[0]);

        switch (choice[0]) {
            case 2:
                sum[1] = count_bit(ix, begin, end, 3);
                if (sum[1] <= sum[0])
                    choice[0] = 3;
                break;

            case 5:
                sum[1] = count_bit(ix, begin, end, 6);
                if (sum[1] <= sum[0])
                    choice[0] = 6;
                break;

            case 7:
                sum[1] = count_bit(ix, begin, end, 8);
                if (sum[1] <= sum[0]) {
                    choice[0] = 8;
                    sum[0] = sum[1];
                }
                sum[1] = count_bit(ix, begin, end, 9);
                if (sum[1] <= sum[0])
                    choice[0] = 9;
                break;

            case 10:
                sum[1] = count_bit(ix, begin, end, 11);
                if (sum[1] <= sum[0]) {
                    choice[0] = 11;
                    sum[0] = sum[1];
                }
                sum[1] = count_bit(ix, begin, end, 12);
                if (sum[1] <= sum[0])
                    choice[0] = 12;
                break;

            case 13:
                sum[1] = count_bit(ix, begin, end, 15);
                if (sum[1] <= sum[0])
                    choice[0] = 15;
                break;
        }
    } else {
        /* try tables with linbits */
        max -= 15;

        for (i = 15; i < 24; i++)
            if (shine_huffman_table[i].linmax >= max) {
                choice[0] = i;
                break;
            }

        for (i = 24; i < 32; i++)
            if (shine_huffman_table[i].linmax >= max) {
                choice[1] = i;
                break;
            }

        sum[0] = count_bit(ix, begin, end, choice[0]);
        sum[1] = count_bit(ix, begin, end, choice[1]);
        if (sum[1] < sum[0])
            choice[0] = choice[1];
    }
    return choice[0];
}

/*
 * bigv_bitcount:
 * --------------
 * Function: Count the number of bits necessary to code the bigvalues region.
 */
int bigv_bitcount(int ix[GRANULE_SIZE], gr_info *gi) {
    int bits = 0;
    unsigned int table;

    if ((table = gi->table_select[0]))  /* region0 */
        bits += count_bit(ix, 0, gi->address1, table);
    if ((table = gi->table_select[1]))  /* region1 */
        bits += count_bit(ix, gi->address1, gi->address2, table);
    if ((table = gi->table_select[2]))  /* region2 */
        bits += count_bit(ix, gi->address2, gi->address3, table);
    return bits;
}

/*
 * count_bit:
 * ----------
 * Function: Count the number of bits necessary to code the subregion.
 */
int count_bit(int ix[GRANULE_SIZE],
              unsigned int start,
              unsigned int end,
              unsigned int table) {
    unsigned linbits, ylen;
    register int i, sum;
    register int x, y;
    const struct huffcodetab *h;

    if (!table)
        return 0;

    h = &(shine_huffman_table[table]);
    sum = 0;

    ylen = h->ylen;
    linbits = h->linbits;

    if (table > 15) { /* ESC-table is used */
        for (i = start; i < end; i += 2) {
            x = ix[i];
            y = ix[i + 1];
            if (x > 14) {
                x = 15;
                sum += linbits;
            }
            if (y > 14) {
                y = 15;
                sum += linbits;
            }

            sum += h->hlen[(x * ylen) + y];
            if (x)
                sum++;
            if (y)
                sum++;
        }
    } else { /* No ESC-words */
        for (i = start; i < end; i += 2) {
            x = ix[i];
            y = ix[i + 1];

            sum += h->hlen[(x * ylen) + y];

            if (x != 0)
                sum++;
            if (y != 0)
                sum++;
        }
    }
    return sum;
}

/*
 * bin_search_StepSize:
 * --------------------
 * Succesive approximation approach to obtaining a initial quantizer
 * step size.
 * The following optional code written by Seymour Shlien
 * will speed up the shine_outer_loop code which is called
 * by iteration_loop. When BIN_SEARCH is defined, the
 * shine_outer_loop function precedes the call to the function shine_inner_loop
 * with a call to bin_search gain defined below, which
 * returns a good starting quantizerStepSize.
 */
int bin_search_StepSize(int desired_rate, int ix[GRANULE_SIZE],
                        gr_info *cod_info, shine_global_config *config) {
    int bit, next, count;

    next = -120;
    count = 120;

    do {
        int half = count / 2;

        if (quantize(ix, next + half, config) > 8192)
            bit = 100000;  /* fail */
        else {
            calc_runlen(ix, cod_info);           /* rzero,count1,big_values */
            bit = count1_bitcount(ix, cod_info); /* count1_table selection */
            subdivide(cod_info, config);         /* bigvalues sfb division */
            bigv_tab_select(ix, cod_info);       /* codebook selection */
            bit += bigv_bitcount(ix, cod_info);  /* bit count */
        }

        if (bit < desired_rate)
            count = half;
        else {
            next += half;
            count -= half;
        }
    } while (count > 1);

    return next;
}

/* This is table B.9: coefficients for aliasing reduction */
#define MDCT_CA(coef)    (int32_t)(coef / sqrt(1.0 + (coef * coef)) * 0x7fffffff)
#define MDCT_CS(coef)    (int32_t)(1.0  / sqrt(1.0 + (coef * coef)) * 0x7fffffff)

#define MDCT_CA0    MDCT_CA(-0.6)
#define MDCT_CA1    MDCT_CA(-0.535)
#define MDCT_CA2    MDCT_CA(-0.33)
#define MDCT_CA3    MDCT_CA(-0.185)
#define MDCT_CA4    MDCT_CA(-0.095)
#define MDCT_CA5    MDCT_CA(-0.041)
#define MDCT_CA6    MDCT_CA(-0.0142)
#define MDCT_CA7    MDCT_CA(-0.0037)

#define MDCT_CS0    MDCT_CS(-0.6)
#define MDCT_CS1    MDCT_CS(-0.535)
#define MDCT_CS2    MDCT_CS(-0.33)
#define MDCT_CS3    MDCT_CS(-0.185)
#define MDCT_CS4    MDCT_CS(-0.095)
#define MDCT_CS5    MDCT_CS(-0.041)
#define MDCT_CS6    MDCT_CS(-0.0142)
#define MDCT_CS7    MDCT_CS(-0.0037)

/*
 * shine_mdct_initialise:
 * -------------------
 */
void shine_mdct_initialise(shine_global_config *config) {
    int m, k;

    /* prepare the mdct coefficients */
    for (m = 18; m--;)
        for (k = 36; k--;)
            /* combine window and mdct coefficients into a single table */
            /* scale and convert to fixed point before storing */
            config->mdct.cos_l[m][k] = (int32_t) (sin(PI36 * (k + 0.5))
                                                  * cos((PI / 72) * (2 * k + 19) * (2 * m + 1)) * 0x7fffffff);
}

/*
 * shine_mdct_sub:
 * ------------
 */
void shine_mdct_sub(shine_global_config *config, int stride) {
    /* note. we wish to access the array 'config->mdct_freq[2][2][576]' as
     * [2][2][32][18]. (32*18=576),
     */
    int32_t (*mdct_enc)[18];

    int ch, gr, band, j, k;
    int32_t mdct_in[36];

    for (ch = config->wave.channels; ch--;) {
        for (gr = 0; gr < config->mpeg.granules_per_frame; gr++) {
            /* set up pointer to the part of config->mdct_freq we're using */
            mdct_enc = (int32_t (*)[18]) config->mdct_freq[ch][gr];

            /* polyphase filtering */
            for (k = 0; k < 18; k += 2) {
                shine_window_filter_subband(&config->buffer[ch], &config->l3_sb_sample[ch][gr + 1][k][0], ch, config,
                                            stride);
                shine_window_filter_subband(&config->buffer[ch], &config->l3_sb_sample[ch][gr + 1][k + 1][0], ch,
                                            config, stride);
                /* Compensate for inversion in the analysis filter
                 * (every odd index of band AND k)
                 */
                for (band = 1; band < 32; band += 2)
                    config->l3_sb_sample[ch][gr + 1][k + 1][band] *= -1;
            }

            /* Perform imdct of 18 previous subband samples + 18 current subband samples */
            for (band = 0; band < 32; band++) {
                for (k = 18; k--;) {
                    mdct_in[k] = config->l3_sb_sample[ch][gr][k][band];
                    mdct_in[k + 18] = config->l3_sb_sample[ch][gr + 1][k][band];
                }

                /* Calculation of the MDCT
                 * In the case of long blocks ( block_type 0,1,3 ) there are
                 * 36 coefficients in the time domain and 18 in the frequency
                 * domain.
                 */
                for (k = 18; k--;) {
                    int32_t vm;
                    uint32_t vm_lo __attribute__((unused));

                    mul0(vm, vm_lo, mdct_in[35], config->mdct.cos_l[k][35]);
                    for (j = 35; j; j -= 7) {
                        muladd(vm, vm_lo, mdct_in[j - 1], config->mdct.cos_l[k][j - 1]);
                        muladd(vm, vm_lo, mdct_in[j - 2], config->mdct.cos_l[k][j - 2]);
                        muladd(vm, vm_lo, mdct_in[j - 3], config->mdct.cos_l[k][j - 3]);
                        muladd(vm, vm_lo, mdct_in[j - 4], config->mdct.cos_l[k][j - 4]);
                        muladd(vm, vm_lo, mdct_in[j - 5], config->mdct.cos_l[k][j - 5]);
                        muladd(vm, vm_lo, mdct_in[j - 6], config->mdct.cos_l[k][j - 6]);
                        muladd(vm, vm_lo, mdct_in[j - 7], config->mdct.cos_l[k][j - 7]);
                    }
                    mulz(vm, vm_lo);
                    mdct_enc[band][k] = vm;
                }

                /* Perform aliasing reduction butterfly */
                if (band != 0) {
                    cmuls(mdct_enc[band][0], mdct_enc[band - 1][17 - 0], mdct_enc[band][0], mdct_enc[band - 1][17 - 0],
                          MDCT_CS0, MDCT_CA0);
                    cmuls(mdct_enc[band][1], mdct_enc[band - 1][17 - 1], mdct_enc[band][1], mdct_enc[band - 1][17 - 1],
                          MDCT_CS1, MDCT_CA1);
                    cmuls(mdct_enc[band][2], mdct_enc[band - 1][17 - 2], mdct_enc[band][2], mdct_enc[band - 1][17 - 2],
                          MDCT_CS2, MDCT_CA2);
                    cmuls(mdct_enc[band][3], mdct_enc[band - 1][17 - 3], mdct_enc[band][3], mdct_enc[band - 1][17 - 3],
                          MDCT_CS3, MDCT_CA3);
                    cmuls(mdct_enc[band][4], mdct_enc[band - 1][17 - 4], mdct_enc[band][4], mdct_enc[band - 1][17 - 4],
                          MDCT_CS4, MDCT_CA4);
                    cmuls(mdct_enc[band][5], mdct_enc[band - 1][17 - 5], mdct_enc[band][5], mdct_enc[band - 1][17 - 5],
                          MDCT_CS5, MDCT_CA5);
                    cmuls(mdct_enc[band][6], mdct_enc[band - 1][17 - 6], mdct_enc[band][6], mdct_enc[band - 1][17 - 6],
                          MDCT_CS6, MDCT_CA6);
                    cmuls(mdct_enc[band][7], mdct_enc[band - 1][17 - 7], mdct_enc[band][7], mdct_enc[band - 1][17 - 7],
                          MDCT_CS7, MDCT_CA7);
                }
            }
        }

        /* Save latest granule's subband samples to be used in the next mdct call */
        memcpy(config->l3_sb_sample[ch][0], config->l3_sb_sample[ch][config->mpeg.granules_per_frame],
               sizeof(config->l3_sb_sample[0][0]));
    }
}

/*
 * shine_subband_initialise:
 * ----------------------
 * Calculates the analysis filterbank coefficients and rounds to the
 * 9th decimal place accuracy of the filterbank tables in the ISO
 * document.  The coefficients are stored in #filter#
 */
void shine_subband_initialise(shine_global_config *config) {
    int i, j;
    double filter;

    for (i = MAX_CHANNELS; i--;) {
        config->subband.off[i] = 0;
        memset(config->subband.x[i], 0, sizeof(config->subband.x[i]));
    }

    for (i = SBLIMIT; i--;)
        for (j = 64; j--;) {
            if ((filter = 1e9 * cos((double) ((2 * i + 1) * (16 - j) * PI64))) >= 0)
                modf(filter + 0.5, &filter);
            else
                modf(filter - 0.5, &filter);
            /* scale and convert to fixed point before storing */
            config->subband.fl[i][j] = (int32_t) (filter * (0x7fffffff * 1e-9));
        }
}

/*
 * shine_window_filter_subband:
 * -------------------------
 * Overlapping window on PCM samples
 * 32 16-bit pcm samples are scaled to fractional 2's complement and
 * concatenated to the end of the window buffer #x#. The updated window
 * buffer #x# is then windowed by the analysis window #shine_enwindow# to produce
 * the windowed sample #z#
 * Calculates the analysis filter bank coefficients
 * The windowed samples #z# is filtered by the digital filter matrix #filter#
 * to produce the subband samples #s#. This done by first selectively
 * picking out values from the windowed samples, and then multiplying
 * them by the filter matrix, producing 32 subband samples.
 */
void
shine_window_filter_subband(int16_t **buffer, int32_t s[SBLIMIT], int ch, shine_global_config *config, int stride) {
    int32_t y[64];
    int i, j;
    int16_t *ptr = *buffer;

    /* replace 32 oldest samples with 32 new samples */
    for (i = 32; i--;) {
        config->subband.x[ch][i + config->subband.off[ch]] = ((int32_t) *ptr) << 16;
        ptr += stride;
    }
    *buffer = ptr;

    for (i = 64; i--;) {
        int32_t s_value;
        uint32_t s_value_lo __attribute__((unused));

        mul0  (s_value, s_value_lo, config->subband.x[ch][(config->subband.off[ch] + i + (0 << 6)) & (HAN_SIZE - 1)],
               shine_enwindow[i + (0 << 6)]);
        muladd(s_value, s_value_lo, config->subband.x[ch][(config->subband.off[ch] + i + (1 << 6)) & (HAN_SIZE - 1)],
               shine_enwindow[i + (1 << 6)]);
        muladd(s_value, s_value_lo, config->subband.x[ch][(config->subband.off[ch] + i + (2 << 6)) & (HAN_SIZE - 1)],
               shine_enwindow[i + (2 << 6)]);
        muladd(s_value, s_value_lo, config->subband.x[ch][(config->subband.off[ch] + i + (3 << 6)) & (HAN_SIZE - 1)],
               shine_enwindow[i + (3 << 6)]);
        muladd(s_value, s_value_lo, config->subband.x[ch][(config->subband.off[ch] + i + (4 << 6)) & (HAN_SIZE - 1)],
               shine_enwindow[i + (4 << 6)]);
        muladd(s_value, s_value_lo, config->subband.x[ch][(config->subband.off[ch] + i + (5 << 6)) & (HAN_SIZE - 1)],
               shine_enwindow[i + (5 << 6)]);
        muladd(s_value, s_value_lo, config->subband.x[ch][(config->subband.off[ch] + i + (6 << 6)) & (HAN_SIZE - 1)],
               shine_enwindow[i + (6 << 6)]);
        muladd(s_value, s_value_lo, config->subband.x[ch][(config->subband.off[ch] + i + (7 << 6)) & (HAN_SIZE - 1)],
               shine_enwindow[i + (7 << 6)]);
        mulz  (s_value, s_value_lo);
        y[i] = s_value;
    }

    config->subband.off[ch] = (config->subband.off[ch] + 480) & (HAN_SIZE - 1); /* offset is modulo (HAN_SIZE)*/

    for (i = SBLIMIT; i--;) {
        int32_t s_value;
        uint32_t s_value_lo __attribute__((unused));

        mul0(s_value, s_value_lo, config->subband.fl[i][63], y[63]);
        for (j = 63; j; j -= 7) {
            muladd(s_value, s_value_lo, config->subband.fl[i][j - 1], y[j - 1]);
            muladd(s_value, s_value_lo, config->subband.fl[i][j - 2], y[j - 2]);
            muladd(s_value, s_value_lo, config->subband.fl[i][j - 3], y[j - 3]);
            muladd(s_value, s_value_lo, config->subband.fl[i][j - 4], y[j - 4]);
            muladd(s_value, s_value_lo, config->subband.fl[i][j - 5], y[j - 5]);
            muladd(s_value, s_value_lo, config->subband.fl[i][j - 6], y[j - 6]);
            muladd(s_value, s_value_lo, config->subband.fl[i][j - 7], y[j - 7]);
        }
        mulz(s_value, s_value_lo);
        s[i] = s_value;
    }
}


/*
 * shine_max_reservoir_bits:
 * ------------
 * Called at the beginning of each granule to get the max bit
 * allowance for the current granule based on reservoir size
 * and perceptual entropy.
 */
int shine_max_reservoir_bits(double *pe, shine_global_config *config) {
    int more_bits, max_bits, add_bits, over_bits;
    int mean_bits = config->mean_bits;

    mean_bits /= config->wave.channels;
    max_bits = mean_bits;

    if (max_bits > 4095)
        max_bits = 4095;
    if (!config->ResvMax)
        return max_bits;

    more_bits = *pe * 3.1 - mean_bits;
    add_bits = 0;
    if (more_bits > 100) {
        int frac = (config->ResvSize * 6) / 10;

        if (frac < more_bits)
            add_bits = frac;
        else
            add_bits = more_bits;
    }
    over_bits = config->ResvSize - ((config->ResvMax << 3) / 10) - add_bits;
    if (over_bits > 0)
        add_bits += over_bits;

    max_bits += add_bits;
    if (max_bits > 4095)
        max_bits = 4095;
    return max_bits;
}

/*
 * shine_ResvAdjust:
 * -----------
 * Called after a granule's bit allocation. Readjusts the size of
 * the reservoir to reflect the granule's usage.
 */
void shine_ResvAdjust(gr_info *gi, shine_global_config *config) {
    config->ResvSize += (config->mean_bits / config->wave.channels) - gi->part2_3_length;
}

/*
 * shine_ResvFrameEnd:
 * -------------
 * Called after all granules in a frame have been allocated. Makes sure
 * that the reservoir size is within limits, possibly by adding stuffing
 * bits. Note that stuffing bits are added by increasing a granule's
 * part2_3_length. The bitstream formatter will detect this and write the
 * appropriate stuffing bits to the bitstream.
 */
void shine_ResvFrameEnd(shine_global_config *config) {
    gr_info *gi;
    int gr, ch, ancillary_pad, stuffingBits;
    int over_bits;
    shine_side_info_t *l3_side = &config->side_info;

    ancillary_pad = 0;

    /* just in case mean_bits is odd, this is necessary... */
    if ((config->wave.channels == 2) && (config->mean_bits & 1))
        config->ResvSize += 1;

    over_bits = config->ResvSize - config->ResvMax;
    if (over_bits < 0)
        over_bits = 0;

    config->ResvSize -= over_bits;
    stuffingBits = over_bits + ancillary_pad;

    /* we must be byte aligned */
    if ((over_bits = config->ResvSize % 8)) {
        stuffingBits += over_bits;
        config->ResvSize -= over_bits;
    }

    if (stuffingBits) {
        /*
         * plan a: put all into the first granule
         * This was preferred by someone designing a
         * real-time decoder...
         */
        gi = (gr_info *) &(l3_side->gr[0].ch[0]);

        if (gi->part2_3_length + stuffingBits < 4095)
            gi->part2_3_length += stuffingBits;
        else {
            /* plan b: distribute throughout the granules */
            for (gr = 0; gr < config->mpeg.granules_per_frame; gr++)
                for (ch = 0; ch < config->wave.channels; ch++) {
                    int extraBits, bitsThisGr;
                    gr_info *gi = (gr_info *) &(l3_side->gr[gr].ch[ch]);
                    if (!stuffingBits)
                        break;
                    extraBits = 4095 - gi->part2_3_length;
                    bitsThisGr = extraBits < stuffingBits ? extraBits : stuffingBits;
                    gi->part2_3_length += bitsThisGr;
                    stuffingBits -= bitsThisGr;
                }
            /*
             * If any stuffing bits remain, we elect to spill them
             * into ancillary data. The bitstream formatter will do this if
             * l3side->resvDrain is set
             */
            l3_side->resvDrain = stuffingBits;
        }
    }
}


