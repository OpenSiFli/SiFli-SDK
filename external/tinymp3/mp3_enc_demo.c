#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include <rtdevice.h>
#include <dfs_fs.h>
#include <dfs_file.h>
#include <dfs_posix.h>
#include "shine_mp3.h"
#include "mp3_enc_demo.h"
                        
static struct mp3enc_file mp3enc_file[] = {
    {"pcm1.bin", "mp3_1.mp3", PCM_STEREO, 16, 16000, 128},
    {"pcm2.bin", "mp3_2.mp3", PCM_STEREO, 16, 16000, 128},
    {"pcm3.bin", "mp3_3.mp3", PCM_STEREO, 16, 16000, 128},
};

static void set_defaults(shine_config_t *config) {
    shine_set_config_mpeg_defaults(&config->mpeg);
} 

static void check_config(shine_config_t *config)
{
    static char *version_names[4] = {"2.5", "reserved", "II", "I"};
    static char *mode_names[4] = {"stereo", "joint-stereo", "dual-channel", "mono"};
    static char *demp_names[4] = {"none", "50/15us", "", "CITT"};

    rt_kprintf("MPEG-%s layer III, %s  Psychoacoustic Model: Shine\n",
           version_names[shine_check_config(config->wave.samplerate, config->mpeg.bitr)],
           mode_names[config->mpeg.mode]);
    rt_kprintf("Bitrate: %d kbps  ", config->mpeg.bitr);
    rt_kprintf("De-emphasis: %s   %s %s\n",
           demp_names[config->mpeg.emph],
           ((config->mpeg.original) ? "Original" : ""),
           ((config->mpeg.copyright) ? "(C)" : ""));
}

int pcmS16_totalSample(int fd)
{
    int cur = 0, end = 0;

    cur = lseek(fd, 0, SEEK_CUR);
    if (cur < 0)
        return RT_ERROR;

    end = lseek(fd, 0, SEEK_END);
    if (end < 0)
        return RT_ERROR;

    lseek(fd, cur, SEEK_SET);

    return end / sizeof(rt_uint16_t);
}

static rt_tick_t total_time = 0;
static rt_tick_t t0 = 0;

static int mp3_enc(int argc, char **argv)
{
    rt_uint8_t file_index = 0;
    struct mp3enc_file *file = RT_NULL;
    shine_config_t config = {0};
    shine_t shine = RT_NULL;
    int samples_per_pass = 0;
    int total_sample = 0;
    rt_uint8_t *out_data = RT_NULL, *in_data = RT_NULL;
    int written = 0, ret = RT_EOK;

    if (argc < 2) {
       rt_kprintf("param num %d < 2\n", argc);
       rt_kprintf("mp3_enc 1 (mp3 encoder pcm1.bin to mp3_1.mp3)\n");
       rt_kprintf("mp3_enc 2 (mp3 encoder pcm2.bin to mp3_2.mp3)\n");
       rt_kprintf("mp3_enc 3 (mp3 encoder pcm3.bin to mp3_3.mp3)\n");
       return RT_ERROR;
    }

    if (argv[1][0] == '1') {
        file_index = 0;
    } else if (argv[1][0] == '2') {
        file_index = 1;
    } else if (argv[1][0] == '3') {
        file_index = 2;
    } else {
        rt_kprintf("error invalid param %d\n", argv[1][0]);
        return RT_ERROR;
    }
    rt_kprintf("now begin to encoding %s to %s!\n", mp3enc_file[file_index].input_file, mp3enc_file[file_index].output_file);
    file = &mp3enc_file[file_index];
    /*1.set config*/
    set_defaults(&config);
    config.wave.channels = file->channels;
    config.wave.samplerate = file->samplerate;
    config.mpeg.mode = STEREO;
    config.mpeg.bitr = file->bitrate;
    /*2.check config samplerate bitrate*/
    if (shine_check_config(config.wave.samplerate, config.mpeg.bitr) < 0) {
        rt_kprintf("error unsupported samplerate/bitrate configuration\n");
        return RT_ERROR;
    }
    check_config(&config);
    /*3.shine init*/
    shine = shine_initialise(&config);
    /*4.get sample point num in every frame which shine process one time*/
    samples_per_pass = shine_samples_per_pass(shine) * file->channels;
    rt_kprintf("samples_per_pass:%d %d\n", samples_per_pass, samples_per_pass * sizeof(rt_uint16_t));
    in_data = rt_calloc(samples_per_pass, sizeof(rt_uint16_t));
    if (in_data == RT_NULL) {
        rt_kprintf("malloc in_data buf error\n");
        ret = RT_ERROR;
        goto EXIT1;
    }
    /*5.get pcm file total sample point num*/
    int fd_in = open(file->input_file, O_RDONLY | O_BINARY);
    if (fd_in < 0) {
        rt_kprintf("open file %s fail\n", file->input_file);
        ret = RT_ERROR;
        goto EXIT2;
    }
    total_sample = pcmS16_totalSample(fd_in);
    if (total_sample == RT_ERROR) {
        rt_kprintf("pcmS16_totalSample error\n");
        ret = RT_ERROR;
        goto EXIT3;
    }
    rt_kprintf("file %s totalSample %d\n", file->input_file, total_sample);
    /*6.create and open output file *.mp3*/
    int fd_out = open(file->output_file, O_RDWR | O_CREAT | O_TRUNC | O_BINARY);
    if (fd_out < 0) {
        rt_kprintf("open file %s fail\n", file->output_file);
        ret = RT_ERROR;
        goto EXIT3;
    }
    /*7.pcm to mp3 encoder*/
    for (int i = 0; i < (total_sample / samples_per_pass); i++) {
        if (read(fd_in, in_data, samples_per_pass * sizeof(rt_uint16_t)) !=
            samples_per_pass * sizeof(rt_uint16_t)) {
            rt_kprintf("read file %s error %d\n", file->input_file, i);
            ret = RT_ERROR;
            goto EXIT4;
        }
        t0 = rt_tick_get_millisecond();
        out_data = shine_encode_buffer_interleaved(shine, (int16_t *)in_data, &written);
        total_time += (rt_tick_get_millisecond() - t0);
        if (write(fd_out, out_data, written) != written) {
            rt_kprintf("write file %s error\n", file->output_file);
            ret = RT_ERROR;
            goto EXIT4;
        }
    }

    if ((total_sample % samples_per_pass) != 0) {
        rt_memset(in_data, 0, samples_per_pass * sizeof(rt_uint16_t));
        if (read(fd_in, in_data, (total_sample % samples_per_pass) * sizeof(rt_uint16_t)) !=
            (total_sample % samples_per_pass) * sizeof(rt_uint16_t)) {
            rt_kprintf("read file %s end error\n", file->input_file);
            ret = RT_ERROR;
            goto EXIT4;
        }
        t0 = rt_tick_get_millisecond();
        out_data = shine_encode_buffer_interleaved(shine, (int16_t *)in_data, &written);
        total_time += (rt_tick_get_millisecond() - t0);
        if (write(fd_out, out_data, written) != written) {
            rt_kprintf("write file %s end error\n", file->output_file);
            ret = RT_ERROR;
            goto EXIT4;
        }
    }

    /*8.Flush and write remaining data*/
    out_data = shine_flush(shine, &written);
    if (write(fd_out, out_data, written) != written) {
        rt_kprintf("write file %s remaining data error\n", file->output_file);
        ret = RT_ERROR;
        goto EXIT4;
    }
    rt_kprintf(" encoding %s to %s success!\n", file->input_file, file->output_file);
    rt_kprintf("mp3_enc total time %d, pcmsize%d, bitrate%d, samplerate%d\n", total_time, total_sample * 2, file->bitrate, file->samplerate);
    total_time = 0;

    /*9. close encoder and file*/
EXIT4:
    close(fd_out);
EXIT3:
    close(fd_in);
EXIT2:
    rt_free(in_data);
EXIT1:
    shine_close(shine);
    out_data = RT_NULL;
    in_data = RT_NULL;

    return ret;
}

MSH_CMD_EXPORT(mp3_enc, mp3_enc 0 / 1 / 2 para...);