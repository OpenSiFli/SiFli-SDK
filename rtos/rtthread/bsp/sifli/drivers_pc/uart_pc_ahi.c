/**
  ******************************************************************************
  * @file   uart_pc.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdio.h>

#include <rthw.h>
#include <rtdevice.h>
#include <rtthread.h>
#include <rtdevice.h>

#include  <windows.h>
#include  <mmsystem.h>
#include  <conio.h>

/* uart driver */
struct console_uart
{
    int rx_ready;

    struct rt_ringbuffer rb;
    rt_uint8_t rx_buffer[2048];
} _console_uart;
static struct rt_serial_device _serial;

#define SAVEKEY(key)  do { char ch = key; rt_ringbuffer_put_force(&(_console_uart.rb), &ch, 1); } while (0)

/*
 * Handler for OSKey Thread
 */
static HANDLE       OSKey_Thread, hComm;
static OVERLAPPED   osWrite;
static DWORD        OSKey_ThreadID;
static DWORD WINAPI pc_uart_rx(LPVOID lpParam);
char g_uart2_com[32];
#undef printf

static void h4tl_lowlevel_init(struct rt_serial_device *serial)
{
    char name[16];
    BOOL Status;
    strcpy(name, "\\\\.\\");
    strcat(name, g_uart2_com);
    //strcat(name, ":");
    hComm = CreateFile(name,                        // Name of the Port to be Opened
                       GENERIC_READ | GENERIC_WRITE,      // Read/Write Access
                       0,                                 // No Sharing, ports cant be shared
                       NULL,                              // No Security
                       OPEN_EXISTING,                     // Open existing port only
                       FILE_FLAG_OVERLAPPED,              // Non Overlapped I/O
                       NULL);                             // Null for Comm Devices

    if (hComm == INVALID_HANDLE_VALUE)
    {
        printf("\n   Error! - Port %s can't be opened", name);
        return;
    }
    else
        printf("\n   Port %s Opened\n ", PCRTT_UART2);

    DCB dcb;
    GetCommState(hComm, &dcb);

    dcb.DCBlength = sizeof(DCB);
    dcb.BaudRate = RT_SERIAL_DEFAULT_BAUDRATE;
    dcb.Parity = 0;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;

    SetCommState(hComm, &dcb);
    //SetCommState(hComm, &dcb);

    COMMTIMEOUTS timeout;
    GetCommTimeouts(hComm, &timeout);
    timeout.ReadIntervalTimeout = 100;
    timeout.ReadTotalTimeoutConstant = 100;
    timeout.ReadTotalTimeoutMultiplier = 400;
    SetCommTimeouts(hComm, &timeout);
    SetupComm(hComm, 1024, 1024);

    PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);

    DWORD dwErr;
    COMSTAT cs;
    ClearCommError(hComm, &dwErr, &cs);
    Status = SetCommMask(hComm, EV_RXCHAR); //Configure Windows to Monitor the serial device for Character Reception

    osWrite.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

    if (Status == FALSE)
        printf("\n\n    Error! in Setting CommMask");
    else
        printf("\n\n    Setting CommMask successfull");

    /*
    * create serial thread that receive key input from keyboard
    */

    OSKey_Thread = CreateThread(NULL,
                                0,
                                (LPTHREAD_START_ROUTINE)pc_uart_rx,
                                0,
                                CREATE_SUSPENDED,
                                &OSKey_ThreadID);
    if (OSKey_Thread == NULL)
    {
        //Display Error Message
        return;
    }

    SetThreadPriority(OSKey_Thread,
                      THREAD_PRIORITY_NORMAL);
    SetThreadPriorityBoost(OSKey_Thread, TRUE);
    SetThreadAffinityMask(OSKey_Thread, 0x01);
    /*
     * Start OS get key Thread
     */
    ResumeThread(OSKey_Thread);
}

static int process_key(unsigned char key)
{
    /*
    * left  key(��)�� 0xe04b
    * up    key(��)�� 0xe048
    * right key(��)�� 0xe04d
    * down  key(��)�� 0xe050
    */
#if 0
    static unsigned char last_key = 0;
    if (last_key == 0xE0)
    {
        if (key == 0x48) //up key , 0x1b 0x5b 0x41
        {
            SAVEKEY(0x1b);
            SAVEKEY(0x5b);
            SAVEKEY(0x41);
        }
        else if (key == 0x50)//0x1b 0x5b 0x42
        {
            SAVEKEY(0x1b);
            SAVEKEY(0x5b);
            SAVEKEY(0x42);
        }
        else if (key == 0x4b)//<- 0x1b 0x5b 0x44
        {
            SAVEKEY(0x1b);
            SAVEKEY(0x5b);
            SAVEKEY(0x44);
        }
        else if (key == 0x4d)//<- 0x1b 0x5b 0x43
        {
            SAVEKEY(0x1b);
            SAVEKEY(0x5b);
            SAVEKEY(0x43);
        }
        last_key = key;
        return 1;
    }
    last_key = key;
#endif
    SAVEKEY(key);
    return 0;
}
static DWORD WINAPI pc_uart_rx(LPVOID lpParam)
{
#define BUFF_SIZE 1024
    unsigned char key[BUFF_SIZE];
    int count = 0;
    BOOL Status = FALSE;
    DWORD NoBytesRead;
    BOOL received = FALSE;
    OVERLAPPED osStatus = { 0 };
    HANDLE event;

    (void)lpParam;              //prevent compiler warnings

    PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);

    DWORD dwErr;
    COMSTAT cs;
    ClearCommError(hComm, &dwErr, &cs);

    event = CreateEvent(NULL, TRUE, FALSE, NULL);
    for (;;)
    {
        memset(&osStatus, 0, sizeof(OVERLAPPED));
        osStatus.hEvent = event;
        Status = ReadFile(hComm, &key[0], sizeof(key), &NoBytesRead, &osStatus);
        if (!Status)
        {
            if (GetLastError() == ERROR_IO_PENDING)
            {
                if (WaitForSingleObject(osStatus.hEvent, INFINITE) == WAIT_OBJECT_0)
                {
                    DWORD dwOvRest;
                    GetOverlappedResult(hComm, &osStatus, &dwOvRest, FALSE);
                    if (dwOvRest)
                    {
                        NoBytesRead = dwOvRest;
                        received = TRUE;
                    }
                    ResetEvent(event);
                }
                SetLastError(0);
            }
        }
        else
        {
            received = TRUE;
        }
        if (received)
        {
            rt_ringbuffer_put_force(&(_console_uart.rb), key, NoBytesRead);
            rt_hw_serial_isr(&_serial, RT_SERIAL_EVENT_RX_IND);
            received = FALSE;
        }
    }
    printf("End of RX thread\n");
    return 0;
} /*** ThreadforKeyGet ***/

static rt_err_t pc_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    rt_err_t r = RT_ERROR;
    BOOL   Status;
    DCB dcbSerialParams = { 0 };                        // Initializing DCB structure
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

    Status = GetCommState(hComm, &dcbSerialParams);     //retreives  the current settings

    if (Status == FALSE)
        printf("\n   Error! in GetCommState()");

    dcbSerialParams.BaudRate = cfg->baud_rate;      // Setting BaudRate = 9600
    dcbSerialParams.ByteSize = cfg->data_bits;      // Setting ByteSize = 8
    dcbSerialParams.StopBits = cfg->stop_bits;    // Setting StopBits = 1
    dcbSerialParams.Parity   = cfg->parity;      // Setting Parity = None
    dcbSerialParams.fRtsControl = (cfg->hwfc & RT_SERIAL_HWFC_RTS) ? 1 : 0;
    dcbSerialParams.fDtrControl = 0;
    Status = SetCommState(hComm, &dcbSerialParams);  //Configuring the port according to settings in DCB
    if (Status == FALSE)
        printf("\n   Error! in Setting DCB Structure");
    else
    {
#if 0
        COMMTIMEOUTS timeouts = { 0 };
        printf("\n   Setting DCB Structure Successfull\n");
        printf("\n       Baudrate = %d", dcbSerialParams.BaudRate);
        printf("\n       ByteSize = %d", dcbSerialParams.ByteSize);
        printf("\n       StopBits = %d", dcbSerialParams.StopBits);
        printf("\n       Parity   = %d", dcbSerialParams.Parity);

        timeouts.ReadIntervalTimeout         = 50;
        timeouts.ReadTotalTimeoutConstant    = 50;
        timeouts.ReadTotalTimeoutMultiplier  = 10;
        timeouts.WriteTotalTimeoutConstant   = 50;
        timeouts.WriteTotalTimeoutMultiplier = 10;
        Status = SetCommTimeouts(hComm, &timeouts);
        if (Status = FALSE)
            printf("\n   Error! in Setting Time Outs");
        else
        {
            printf("\n\n   Setting Serial Port Timeouts Successfull");
            r = RT_EOK;
        }
#else
        r = RT_EOK;
#endif
    }
    return r;
}

static rt_err_t pc_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct console_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct console_uart *)serial->parent.user_data;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        uart->rx_ready = 0;
        break;
    case RT_DEVICE_CTRL_SET_INT:
        uart->rx_ready = 1;
        break;
    }

    return RT_EOK;
}

static rt_size_t pc_dma(struct rt_serial_device *serial, rt_uint8_t *buf, rt_size_t size, int direction)
{
    DWORD  dNoOfBytesWritten = 0;

    if (direction == RT_SERIAL_DMA_TX)
    {
        if (!WriteFile(hComm,               // Handle to the Serialport
                       buf,            // Data to be written to the port
                       size,   // No of bytes to write into the port
                       &dNoOfBytesWritten,  // No of bytes written to the port
                       &osWrite))
        {
            if (GetLastError() != ERROR_IO_PENDING)
            {
                printf("\n\n   Error %d in Writing to Serial Port", GetLastError());
            }
            else
            {
                DWORD dwRes = WaitForSingleObject(osWrite.hEvent, INFINITE);
                switch (dwRes)
                {
                case WAIT_OBJECT_0:
                    break;
                default:
                    break;
                }
            }
        }
        rt_hw_serial_isr(serial, RT_SERIAL_EVENT_TX_DMADONE);
    }
    return dNoOfBytesWritten;
}


static int pc_putc(struct rt_serial_device *serial, char c)
{
    struct console_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct console_uart *)serial->parent.user_data;

#if 0 /* Enable it if you want to save the console log */
    {
        static FILE *fp = NULL;

        if (fp == NULL)
            fp = fopen("log.txt", "wb+");

        if (fp != NULL)
            fwrite(buffer, size, 1, fp);
    }
#endif
    char   lpBuffer[2] = {c, 0};           // lpBuffer should be  char or byte array, otherwise write wil fail
    DWORD  dNoOFBytestoWrite;              // No of bytes to write into the port
    DWORD  dNoOfBytesWritten = 0;          // No of bytes written to the port

    dNoOFBytestoWrite = 1; // Calculating the no of bytes to write into the port

    if (!WriteFile(hComm,               // Handle to the Serialport
                   lpBuffer,            // Data to be written to the port
                   dNoOFBytestoWrite,   // No of bytes to write into the port
                   &dNoOfBytesWritten,  // No of bytes written to the port
                   &osWrite))
    {
        if (GetLastError() != ERROR_IO_PENDING)
        {
            printf("\n\n   Error %d in Writing to Serial Port", GetLastError());
        }
        else
        {
            DWORD dwRes = WaitForSingleObject(osWrite.hEvent, INFINITE);
            switch (dwRes)
            {
            case WAIT_OBJECT_0:
                break;
            default:
                break;
            }
        }
    }
    return 1;
}

static int pc_getc(struct rt_serial_device *serial)
{
    unsigned char ch;
    struct console_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = (struct console_uart *)serial->parent.user_data;

    if (rt_ringbuffer_getchar(&(uart->rb), &ch)) return ch;

    return -1;
}

static const struct rt_uart_ops console_uart_ops =
{
    pc_configure,
    pc_control,
    pc_putc,
    pc_getc,
    pc_dma,
};

int uart_pc_ahi_init()
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    struct console_uart *uart;
    struct rt_serial_device *serial;

    if (strlen(g_uart2_com) == 0)
        strcat(g_uart2_com, PCRTT_UART2);

    uart = &_console_uart;
    serial = &_serial;

    uart->rx_ready = 0;

    serial->ops    = &console_uart_ops;
    serial->config = config;
    serial->config.baud_rate = RT_SERIAL_DEFAULT_BAUDRATE;
    /* initialize ring buffer */
    rt_ringbuffer_init(&uart->rb, uart->rx_buffer, sizeof(uart->rx_buffer));

#ifdef UART_PORT1_PORT
    /* register UART device */
    rt_hw_serial_register(serial, UART_PORT1_PORT,
                          //RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_INT_TX,
                          RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX | RT_DEVICE_FLAG_DMA_TX,
                          uart);
#endif

    h4tl_lowlevel_init(serial);

    return 0;
}

/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
