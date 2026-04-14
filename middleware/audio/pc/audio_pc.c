/*
 * SPDX-FileCopyrightText: 2022-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <string.h>
#include <stdlib.h>
#include <rtdevice.h>
#include <stdint.h>
#include "board.h"
#include "audio_server.h"
#include "audio_pc.h"
#include <dfs.h>
#include <dfs_posix.h>

#ifdef WIN32
    #define open rt_open
    #define close rt_close
    #define read rt_read
    #define write rt_write
    #define lseek rt_lseek
    #define fstat rt_fstat
#endif

static int file_handle = -1;

#if 0
audio_client_t audio_open(audio_type_t audio_type, audio_rwflag_t rwflag, audio_parameter_t *paramter, audio_server_callback_func callback, void *callback_userdata)
{
    file_handle = open("audio.pcm", O_RDWR | O_CREAT | O_TRUNC | O_BINARY);
    return (audio_client_t)malloc(300);
}

int audio_write(audio_client_t handle, uint8_t *data, uint32_t data_len)
{
    if (file_handle >= 0)
        data_len = write(file_handle, data, data_len);
    return data_len;
}
#else

#include <windows.h>
#include <mmsystem.h>
#include <stdio.h>

#pragma comment(lib, "winmm.lib")

/*
* some good values for block size and count
*/
#define BLOCK_SIZE 8192
#define BLOCK_COUNT 20
/*
* function prototypes
*/
static void CALLBACK waveOutProc(HWAVEOUT, UINT, DWORD, DWORD, DWORD);
static WAVEHDR *allocateBlocks(int size, int count);
static void freeBlocks(WAVEHDR *blockArray);
static void writeAudio(HWAVEOUT hWaveOut, LPSTR data, int size);
/*
* module level variables
*/
static CRITICAL_SECTION waveCriticalSection;
static WAVEHDR *waveBlocks;
static volatile int waveFreeBlockCount;
static int waveCurrentBlock;
static HWAVEOUT hWaveOut; /* device handle */
extern void *ffmpeg_alloc(size_t nbytes);
extern void ffmpeg_free(void *p);
static pcm_data_fun_cb pcm_data_fun =  NULL;
void audio_set_pcm_callback(pcm_data_fun_cb fun)
{
    pcm_data_fun = fun;
}
int audio_server_set_public_volume(uint8_t volume)
{
    return 0;
}
int audio_server_set_public_speaker_mute(uint8_t is_mute)
{
    return 0;
}
int audio_server_set_private_volume(audio_type_t audio_type, uint8_t volume)
{
    return 0;
}

audio_client_t audio_open(audio_type_t audio_type, audio_rwflag_t rwflag, audio_parameter_t *paramter, audio_server_callback_func callback, void *callback_userdata)
{
    WAVEFORMATEX wfx; /* look this up in your documentation */

    /*
    * initialise the module variables
    */
    waveBlocks = allocateBlocks(BLOCK_SIZE, BLOCK_COUNT);
    waveFreeBlockCount = BLOCK_COUNT;
    waveCurrentBlock = 0;
    InitializeCriticalSection(&waveCriticalSection);

    /*
    * set up the WAVEFORMATEX structure.
    */
    wfx.nSamplesPerSec = paramter->write_samplerate; /* sample rate */
    wfx.wBitsPerSample = paramter->write_bits_per_sample; /* sample size */
    wfx.nChannels = paramter->write_channnel_num; /* channels*/
    wfx.cbSize = 0; /* size of _extra_ info */
    wfx.wFormatTag = WAVE_FORMAT_PCM;
    wfx.nBlockAlign = (wfx.wBitsPerSample * wfx.nChannels) >> 3;
    wfx.nAvgBytesPerSec = wfx.nBlockAlign * wfx.nSamplesPerSec;
    /*
    * try to open the default wave device. WAVE_MAPPER is
    * a constant defined in mmsystem.h, it always points to the
    * default wave device on the system (some people have 2 or
    * more sound cards).
    */
    if (waveOutOpen(
                &hWaveOut,
                WAVE_MAPPER,
                &wfx,
                (DWORD_PTR)waveOutProc,
                (DWORD_PTR)&waveFreeBlockCount,
                CALLBACK_FUNCTION
            ) != MMSYSERR_NOERROR)
    {
        rt_kprintf("unable to open wave mapper device\n");
        return NULL;
    }

    return (audio_client_t)malloc(300);
}

void writeAudio(HWAVEOUT hWaveOut, LPSTR data, int size)
{
    WAVEHDR *current;
    int remain;
    current = &waveBlocks[waveCurrentBlock];
    while (size > 0)
    {
        /*
        * first make sure the header we're going to use is unprepared
        */
        if (current->dwFlags & WHDR_PREPARED)
            waveOutUnprepareHeader(hWaveOut, current, sizeof(WAVEHDR));
        if (size < (int)(BLOCK_SIZE - current->dwUser))
        {
            memcpy(current->lpData + current->dwUser, data, size);
            current->dwUser += size;
            break;
        }
        remain = BLOCK_SIZE - current->dwUser;
        memcpy(current->lpData + current->dwUser, data, remain);
        size -= remain;
        data += remain;
        current->dwBufferLength = BLOCK_SIZE;
        waveOutPrepareHeader(hWaveOut, current, sizeof(WAVEHDR));
        waveOutWrite(hWaveOut, current, sizeof(WAVEHDR));
        EnterCriticalSection(&waveCriticalSection);
        waveFreeBlockCount--;
        LeaveCriticalSection(&waveCriticalSection);
        /*
        * wait for a block to become free
        */
        while (!waveFreeBlockCount)
            Sleep(10);
        /*
        * point to the next block
        */
        waveCurrentBlock++;
        waveCurrentBlock %= BLOCK_COUNT;
        current = &waveBlocks[waveCurrentBlock];
        current->dwUser = 0;
    }
}

WAVEHDR *allocateBlocks(int size, int count)
{
    unsigned char *buffer;
    int i;
    WAVEHDR *blocks;
    DWORD totalBufferSize = (size + sizeof(WAVEHDR)) * count;
    /*
    * allocate memory for the entire set in one go
    */
    buffer = ffmpeg_alloc(totalBufferSize);
    if (buffer == NULL)
    {
        rt_kprintf("Memory allocation error\n");
        return NULL;
    }
    memset(buffer, 0, totalBufferSize);
    /*
    * and set up the pointers to each bit
    */
    blocks = (WAVEHDR *)buffer;
    buffer += sizeof(WAVEHDR) * count;
    for (i = 0; i < count; i++)
    {
        blocks[i].dwBufferLength = size;
        blocks[i].lpData = (LPSTR)buffer;
        buffer += size;
    }
    return blocks;
}

void freeBlocks(WAVEHDR *blockArray)
{
    /*
    * and this is why allocateBlocks works the way it does
    */
    ffmpeg_free(blockArray);
}

static void CALLBACK waveOutProc(
    HWAVEOUT hWaveOut,
    UINT uMsg,
    DWORD dwInstance,
    DWORD dwParam1,
    DWORD dwParam2
)
{
    /*
    * pointer to free block counter
    */
    int *freeBlockCounter = (int *)dwInstance;
    /*
    * ignore calls that occur due to openining and closing the
    * device.
    */
    if (uMsg != WOM_DONE)
        return;
    EnterCriticalSection(&waveCriticalSection);
    (*freeBlockCounter)++;
    LeaveCriticalSection(&waveCriticalSection);
}

int audio_write(audio_client_t handle, uint8_t *data, uint32_t data_len)
{
    writeAudio(hWaveOut, data, data_len);
    return data_len;
}
#endif

int audio_read(audio_client_t handle, uint8_t *buf, uint32_t buf_size)
{
    return 0;
}

int audio_ioctl(audio_client_t handle, int cmd, void *parameter)
{
    return 0;
}
int audio_close(audio_client_t handle)
{

    if (handle)
    {
        free(handle);
    }

#if 0
    if (file_handle >= 0)
    {
        close(file_handle);
        file_handle = -1;
    }
#else
    /*
    * wait for all blocks to complete
    */
    while (waveFreeBlockCount < BLOCK_COUNT)
        Sleep(10);
    /*
    * unprepare any blocks that are still prepared
    */
    for (int i = 0; i < waveFreeBlockCount; i++)
        if (waveBlocks[i].dwFlags & WHDR_PREPARED)
            waveOutUnprepareHeader(hWaveOut, &waveBlocks[i], sizeof(WAVEHDR));
    DeleteCriticalSection(&waveCriticalSection);
    freeBlocks(waveBlocks);
    waveOutClose(hWaveOut);
#endif
    return 0;
}

int audio_device_is_a2dp_sink()
{
    return 0;
}
