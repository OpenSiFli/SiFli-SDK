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
        "\n";

    struct parameters p = { };

    for (int iarg = 1; iarg < argc; ) {
        const char *arg = argv[iarg++];

        if (arg[0] == '-') {
            if (arg[2] != '\0')
                error(EINVAL, "Option %s", arg);

            char opt = arg[1];

            switch (opt) {
                case 'h': fprintf(stderr, usage, argv[0]); exit(0);
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
    FILE *fp_in = stdin, *fp_out = stdout;

    if (p.fname_in && (fp_in = fopen(p.fname_in, "rb")) == NULL)
        error(errno, "%s", p.fname_in);

    if (p.fname_out && (fp_out = fopen(p.fname_out, "wb")) == NULL)
        error(errno, "%s", p.fname_out);

    /* --- Setup decoding --- */

    static const char *sbc_mode_str[] = {
        [SBC_MODE_MONO        ] = "Mono",
        [SBC_MODE_DUAL_CHANNEL] = "Dual-Channel",
        [SBC_MODE_STEREO      ] = "Stereo",
        [SBC_MODE_JOINT_STEREO] = "Joint-Stereo"
    };

    uint8_t data[2*SBC_MAX_SAMPLES*sizeof(int16_t)];
    int16_t pcm[2*SBC_MAX_SAMPLES];
    struct sbc_frame frame;
    sbc_t sbc;

    if (fread(data, SBC_PROBE_SIZE, 1, fp_in) < 1
            || sbc_probe(data, &frame) < 0)
        error(EINVAL, "SBC input file format");

    int srate_hz = sbc_get_freq_hz(frame.freq);

    fprintf(stderr, "%s %d Hz -- %.1f kbps (bitpool %d)"
                            " -- %d blocks, %d subbands\n",
        sbc_mode_str[frame.mode], srate_hz,
        sbc_get_frame_bitrate(&frame) * 1e-3, frame.bitpool,
        frame.nblocks, frame.nsubbands);

    int nch = 1 + (frame.mode != SBC_MODE_MONO);

    wave_write_header(fp_out, 16, sizeof(*pcm),
        sbc_get_freq_hz(frame.freq), nch, -1);

    sbc_reset(&sbc);

    /* --- Decoding loop --- */

    for (int i = 0; i == 0 || (fread(data, SBC_PROBE_SIZE, 1, fp_in) >= 1
                               && sbc_probe(data, &frame) == 0); i++) {

        if (fread(data + SBC_PROBE_SIZE,
                sbc_get_frame_size(&frame) - SBC_PROBE_SIZE, 1, fp_in) < 1)
            break;

        sbc_decode(&sbc, data, sizeof(data),
            &frame, pcm + 0, nch, pcm + 1, 2);

        int npcm = frame.nblocks * frame.nsubbands;
        wave_write_pcm(fp_out, sizeof(*pcm), pcm, nch, 0, npcm);
    }

    /* --- Cleanup --- */

    if (fp_in != stdin)
        fclose(fp_in);

    if (fp_out != stdout)
        fclose(fp_out);
}
