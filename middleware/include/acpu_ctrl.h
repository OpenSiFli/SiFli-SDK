/**
  ******************************************************************************
  * @file   acpu_ctrl.h
  * @author Sifli software development team
  * @brief ACPU Controller
  * @{
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

#ifndef __ACPU_CTRL_H__
#define __ACPU_CTRL_H__


/**
****************************************************************************************
* @addtogroup acpu_ctrl ACPU Controller
* @ingroup middleware
* @brief ACPU Controller
* @{
****************************************************************************************
*/


#define ACPU_TASK_INPUT_PARAM_SIZE    (128)
#define ACPU_TASK_OUTPUT_VAL_SIZE     (64)
#define HCPU_TASK_INPUT_PARAM_SIZE    (8)
#define HCPU_TASK_OUTPUT_VAL_SIZE     (8)

#define ACPU_ERR_OK         0
#define ACPU_ERR_COMMON     1
#define ACPU_ERR_PRINTF     2
#define ACPU_ERR_CALL_HCPU  4
#define ACPU_ERR_ASSERT     0xff


typedef struct
{
    void *st;
    int fs;
    int channels;
    int application;
} opus_encode_init_arg_t;

typedef struct
{
    void *st;
    int id;
} opus_encode_ctl_arg_t;

typedef struct
{
    void *st;
    int fs;
    int channels;
} opus_decode_init_arg_t;

typedef struct
{
    void *st;
    int id;
} opus_decode_ctl_arg_t;

typedef struct
{
    void *st;
    const int16_t *pcm;
    int analysis_frame_size;
    uint8_t *data;
    int32_t max_data_bytes;
} opus_encode_arg_t;

typedef struct
{
    void *st;
    const uint8_t *data;
    int32_t len;
    int16_t *pcm;
    int frame_size;
    int decode_fec;
} opus_decode_arg_t;

/** ACPU task name macro */
#define ACPU_TASK_INVALID                (0)
#define ACPU_TASK_0                      (1)
#define ACPU_TASK_1                      (2)
#define ACPU_TASK_opus_encoder_init      (3)
#define ACPU_TASK_opus_encoder_ctl       (4)
#define ACPU_TASK_opus_encode            (5)
#define ACPU_TASK_opus_decoder_init      (6)
#define ACPU_TASK_opus_decoder_ctl       (7)
#define ACPU_TASK_opus_decode            (8)
#define ACPU_TASK_COUNT                  (9)

typedef enum
{
    HCPU_TASK_INVALID,
    HCPU_TASK_MALLOC,
    HCPU_TASK_FREE,
    HCPU_TASK_COUNT,
} hcpu_task_name_t;

/** Power on ACPU, no need to be called by user
 *
 *
 */
void acpu_power_on(void);

/** Power off ACPU, no need to be called by user
 *
 *
 */
void acpu_power_off(void);


/** Call ACPU to run specified task
 *
 *
 * @param[in] task_name  task name, 0 is invalid task name,
 *                       any number greater than 0 is a valid task name, e.g. macro ACPU_TASK_0 defined above use 1 as the value
 * @param[in] param      parameter buffer pointer, the param content is copied to shared buffer
 *                       which would be read by ACPU, the param buffer format is defined by each task
 * @param[in] param_size parameter buffer size, max size is limited by ACPU_TASK_INPUT_PARAM_SIZE
 * @param[out] error_code error code pointer, it's used to save error code returned by ACPU,
 *                        0 means no error happens, other value means some error happens
 *
 * @return ACPU returned result buffer pointer, its length is limited by ACPU_TASK_OUTPUT_VAL_SIZE.
 *         buffer format is defined by each task.
 */
void *acpu_run_task(uint8_t task_name, void *param, uint32_t param_size, uint8_t *error_code);


/** ACPU entry function
 *
 * User needs to implement this function to provide customized ACPU functionality.
 *
 * @param[in] task_name  task name, 0 is invalid task name,
 *                       any number greater than 0 is a valid task name, e.g. macro ACPU_TASK_0 defined above use 1 as the value
 * @param[in] param      parameter given by HCPU
 *
 */
void acpu_main(uint8_t task_name, void *param);

/** ACPU send result to HCPU
 *
 * It can be called by ACPU to return task result to HCPU
 *
 * @param[in] val        data needs to be sent to HCPU. It's copied to shared buffer which would be read by HCPU
 * @param[in] val_size   data size, it's limited by ACPU_TASK_OUTPUT_VAL_SIZE
 *
 */
void acpu_send_result(void *val, uint32_t val_size);


/// @}  acpu_ctrl
/// @}  file

#endif /* __ACPU_CTRL_H__ */
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

