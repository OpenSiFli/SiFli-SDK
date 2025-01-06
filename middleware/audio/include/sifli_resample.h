#ifndef SIFLI_RESAMPLE
#define SIFLI_RESAMPLE 1

typedef struct
{
    float    ratio;
    int16_t *dst;
    uint32_t dst_size;
    uint32_t dst_bytes;
    uint32_t total_src_index;
    uint32_t total_dst_index;
    uint32_t src_samplerate;
    uint32_t dst_samplerate;
    int16_t  last_left;
    int16_t  last_right;
    uint8_t  channels;
} sifli_resample_t;

uint32_t sifli_resample_process(sifli_resample_t *p, int16_t *src, uint32_t src_bytes, uint8_t is_last_packet);
sifli_resample_t *sifli_resample_open(uint8_t channels, uint32_t src_samplerate, uint32_t dst_samplerate);
int16_t *sifli_resample_get_output(sifli_resample_t *p);
void sifli_resample_close(sifli_resample_t *p);

#endif
