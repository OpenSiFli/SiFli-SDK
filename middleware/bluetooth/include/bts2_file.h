/**
  ******************************************************************************
  * @file   bts2_file.h
  * @author Sifli software development team
  ******************************************************************************
*/
/*
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

#ifndef _BTS2_FILE_H_
#define _BTS2_FILE_H_

#ifdef __cplusplus
extern "C" {
#endif

#define BTS2_SEEK_SET 0
#define BTS2_SEEK_CUR 1
#define BTS2_SEEK_END 2

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      The bfopen function opens the file specified by filename.
 *
 * INPUT:
 *     const char *filename: Filename.
 *     const char *mode: Type of access permitted.
 *
 * OUTPUT:
 *     void.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
void *bfopen(const char *filename, const char *mode);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      The bfclose function closes stream.
 *
 * INPUT:
 *     void *stream: Pointer to FILE structure.
 *
 * OUTPUT:
 *     bfclose returns 0 if the stream is successfully closed.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
int bfclose(void *stream);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      The bfwrite function writes up to count items, of size length each, from
 *      buffer to the output stream. The file pointer associated with stream (if
 *      there is one) is incremented by the number of bytes actually written. If
 *      stream is opened in text mode, each carriage return is replaced with a
 *      carriage-return 每 linefeed pair. The replacement has no effect on the
 *      return value
 *
 * INPUT:
 *     const void *buff: Pointer to data to be written.
 *     U32 size: Item size in bytes.
 *     U32 count: Maximum number of items to be written.
 *     void *stream: Pointer to FILE structure.
 *
 * OUTPUT:
 *     bfwrite returns the number of full items actually written, which may be
 *     less than count if an error occurs. Also, if an error occurs, the
 *     file-position indicator cannot be determined.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
U32 bfwrite(const void *buff,
            U32 size,
            U32 count,
            void *stream);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      The bfread function reads up to count items of size bytes from the input
 *      stream and stores them in buffer. The file pointer associated with stream
 *      (if there is one) is increased by the number of bytes actually read. If the
 *      given stream is opened in text mode, carriage return每linefeed pairs are
 *      replaced with single linefeed characters. The replacement has no effect on
 *      the file pointer or the return value. The file-pointer position is indeterminate
 *      if an error occurs. The value of a partially read item cannot be determined.
 *
 * INPUT:
 *     void *buff: Storage location for data.
 *     U32 size: Item size in bytes.
 *     U32 count: Maximum number of items to be read.
 *     void *stream: Pointer to FILE structure.
 *
 * OUTPUT:
 *     bfread returns the number of full items actually read, which may be less
 *     than count if an error occurs or if the end of the file is encountered
 *     before reaching count. Use the feof or ferror function to distinguish a
 *     read error from an end-of-file condition. If size or count is 0, bfread
 *     returns 0 and the buffer contents are unchanged.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
U32 bfread(void *buff,
           U32 size,
           U32 count,
           void *stream);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      The bfseek function moves the file pointer (if any) associated with stream
 *      to a new location that is offset bytes from origin. The next operation on
 *      the stream takes place at the new location. On a stream open for update,
 *      the next operation can be either a read or a write. The argument origin
 *      must be one of the following constants, defined in Stdio.h.
 *
 * INPUT:
 *     void *stream: Pointer to FILE structure.
 *     long offset: Number of bytes from origin.
 *     int origin: Initial position.
 *
 * OUTPUT:
 *     If successful, bfseek returns 0. Otherwise, it returns a nonzero value.
 *     On devices incapable of seeking, the return value is undefined.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
int bfseek(void *stream, long offset, int origin);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      The bftell function gets the current position of the file pointer (if any)
 *      associated with stream. The position is expressed as an offset relative to
 *      the beginning of the stream.
 *      Note that when a file is opened for appending data, the current file position
 *      is determined by the last I/O operation, not by where the next write would
 *      occur. For example, if a file is opened for an append and the last operation
 *      was a read, the file position is the point where the next read operation would
 *      start, not where the next write would start. (When a file is opened for
 *      appending, the file position is moved to end of file before any write
 *      operation.) If no I/O operation has yet occurred on a file opened for appending,
 *      the file position is the beginning of the file.
 *      In text mode, CTRL+Z is interpreted as an end-of-file character on input.
 *      In files opened for reading/writing, fopen and all related routines check for a
 *      CTRL+Z at the end of the file and remove it if possible. This is done because
 *      using bftell and bfseek to move within a file that ends with a CTRL+Z may cause
 *      bftell to behave improperly near the end of the file.
 *
 * INPUT:
 *     void *stream: Target FILE structure.
 *
 * OUTPUT:
 *     bftell returns the current file position. The value returned by bftell may
 *     not reflect the physical byte offset for streams opened in text mode, because
 *     text mode causes carriage return每linefeed translation. Use bftell with bfseek
 *     to return to file locations correctly. On error, bftell returns 每1L. On devices
 *     incapable of seeking (such as terminals and printers), or when stream does not
 *     refer to an open file, the return value is undefined.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
long bftell(void *stream);

/*----------------------------------------------------------------------------*
 *
 * DESCRIPTION:
 *      The bremove function deletes the file specified by path.
 *      All handles to a file must be closed before it can be deleted.
 *
 * INPUT:
 *     const char *path: Path of file to be removed.
 *
 * OUTPUT:
 *      Each of these functions returns 0 if the file is successfully deleted.
 *      Otherwise, it returns 每1 and sets errno either to EACCES to indicate
 *      that the path specifies a read-only file, or to ENOENT to indicate that
 *      the filename or path was not found or that the path specifies a directory.
 *      This function fails and returns -1 if the file is open.
 *
 * NOTE:
 *     none.
 *----------------------------------------------------------------------------*/
int bremove(const char *path);

#ifdef __cplusplus
}
#endif

#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
