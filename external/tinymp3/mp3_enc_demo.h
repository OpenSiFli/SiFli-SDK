#include "rtdef.h"

struct mp3enc_file {
    char *input_file;
    char *output_file;
    rt_uint8_t channels;
    rt_uint8_t samplefmt;
    rt_uint32_t samplerate;
    rt_uint32_t bitrate; //码率 比特率
};

