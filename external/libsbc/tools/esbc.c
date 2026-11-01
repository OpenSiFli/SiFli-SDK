/******************************************************************************
 *
 *  Copyright 2022 Google LLC
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>

#include <sbc.h>
#include "wave.h"


/**
 * Error handling
 */

static void error(int status, const char *format, ...)
{
    va_list args;

    fflush(stdout);

    va_start(args, format);
    vfprintf(stderr, format, args);
    va_end(args);

    fprintf(stderr, status ? ": %s\n" : "\n", strerror(status));
    exit(status);
}


/**
 * Parameters
 */

struct parameters {
    const char *fname_in;
    const char *fname_out;
    struct sbc_frame frame;
};

static struct parameters parse_args(int argc, char *argv[])
{
    static const char *usage =
        "Usage: %s [in_file] [wav_file]\n"
        "\n"
        "wav_file\t"  "Input wave file, stdin if omitted\n"
        "out_file\t"  "Output bitstream file, stdout if omitted\n"
        "\n"
        "Options:\n"
        "\t-h\t"     "Display help\n"
        "\t-d\t"     "Dual channel mode\n"
        "\t-j\t"     "Joint Stereo mode\n"
        "\t-b <n>\t" "Bitpool value (default is 35)\n"
        "\t-s <n>\t" "Number of subbands (default is 8)\n"
        "\t-B <n>\t" "Number of blocks (default is 16)\n"
        "\t-S\t"     "Use SNR bit allocation (default is Loudness)\n"
        "\n";

    struct parameters p = { };
    struct sbc_frame *frame = &p.frame;

    *frame = (struct sbc_frame){
        .mode = SBC_MODE_STEREO,
        .nsubbands = 8, .nblocks = 16,
        .bam = SBC_BAM_LOUDNESS,
        .bitpool = 35
    };

    for (int iarg = 1; iarg < argc; ) {
        const char *arg = argv[iarg++];

        if (arg[0] == '-') {
            if (arg[2] != '\0')
                error(EINVAL, "Option %s", arg);

            char opt = arg[1];
            const char *optarg;

            switch (opt) {
                case 'b': case 's': case 'B':
                    if (iarg >= argc)
                        error(EINVAL, "Argument %s", arg);
                    optarg = argv[iarg++];
            }

            switch (opt) {
                case 'h': fprintf(stderr, usage, argv[0]); exit(0);
                case 'j': frame->mode = SBC_MODE_JOINT_STEREO; break;
                case 'd': frame->mode = SBC_MODE_DUAL_CHANNEL; break;
                case 'b': frame->bitpool = atoi(optarg); break;
                case 's': frame->nsubbands = atoi(optarg); break;
                case 'B': frame->nblocks = atoi(optarg); break;
                case 'S': frame->bam = SBC_BAM_SNR; break;
                default:
                    error(EINVAL, "Option %s", arg);
            }

        } else {

            if (!p.fname_in)
                p.fname_in = arg;
            else if (!p.fname_out)
                p.fname_out = arg;
            else
                error(EINVAL, "Argument %s", arg);
        }
    }

    return p;
}


/**
 * Entry point
 */
int main(int argc, char *argv[])
{
    /* --- Read parameters --- */

    struct parameters p = parse_args(argc, argv);
    struct sbc_frame *frame = &p.frame;
    FILE *fp_in = stdin, *fp_out = stdout;

    if (p.fname_in && (fp_in = fopen(p.fname_in, "rb")) == NULL)
        error(errno, "%s", p.fname_in);

    if (p.fname_out && (fp_out = fopen(p.fname_out, "wb")) == NULL)
        error(errno, "%s", p.fname_out);

    /* --- Check parameters --- */

    int srate_hz, nch, nsamples;
    int pcm_sbits, pcm_sbytes;

    if (wave_read_header(fp_in,
            &pcm_sbits, &pcm_sbytes, &srate_hz, &nch, &nsamples) < 0)
        error(EINVAL, "Bad or unsupported WAVE input file");

    frame->freq = srate_hz == 16000 ? SBC_FREQ_16K  :
                  srate_hz == 32000 ? SBC_FREQ_32K  :
                  srate_hz == 44100 ? SBC_FREQ_44K1 :
                  srate_hz == 48000 ? SBC_FREQ_48K  : SBC_NUM_FREQ;

    if (nch == 1)
        frame->mode = SBC_MODE_MONO;

    if (frame->freq >= SBC_NUM_FREQ)
        error(EINVAL, "Samplerate %d Hz", srate_hz);

    if (pcm_sbits != 16 || pcm_sbytes != sizeof(int16_t))
        error(EINVAL, "Bitdepth %d", pcm_sbits);

    if (nch  < 1 || nch  > 2)
        error(EINVAL, "Number of channels %d", nch);

    /* --- Setup decoding --- */

    uint8_t data[2*SBC_MAX_SAMPLES*sizeof(int16_t)];
    int16_t pcm[2*SBC_MAX_SAMPLES];
    sbc_t sbc;

    int npcm = frame->nblocks * frame->nsubbands;

    sbc_reset(&sbc);

    /* --- Encoding loop --- */

    for (int i = 0; wave_read_pcm(fp_in,
            pcm_sbytes, nch, npcm, pcm) >= npcm; i++) {

        sbc_encode(&sbc,
            pcm + 0, nch, pcm + 1, 2, frame, data, sizeof(data));

        fwrite(data, sbc_get_frame_size(frame), 1, fp_out);
    }

    /* --- Cleanup --- */

    if (fp_in != stdin)
        fclose(fp_in);

    if (fp_out != stdout)
        fclose(fp_out);
}
