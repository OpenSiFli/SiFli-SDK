/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __ACPU_CTRL_H__
#define __ACPU_CTRL_H__

#include <stdint.h>

/**
****************************************************************************************
* @addtogroup acpu_ctrl ACPU Controller
* @ingroup middleware
* @brief ACPU Controller
* @{
****************************************************************************************
*/



#ifdef __cplusplus
extern "C" {
#endif

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
#define ACPU_TASK_audio_3a_open          (9)
#define ACPU_TASK_audio_3a_close         (10)
#define ACPU_TASK_audio_3a_downlink      (11)
#define ACPU_TASK_audio_3a_uplink        (12)
#define ACPU_TASK_COUNT                  (13)

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

/** ACPU send result with given error_code to HCPU
 *
 * It can be called by ACPU to return task result to HCPU
 *
 * @param[in] val        data needs to be sent to HCPU. It's copied to shared buffer which would be read by HCPU
 * @param[in] val_size   data size, it's limited by ACPU_TASK_OUTPUT_VAL_SIZE
 * @param[in] error_code   error code
 *
 */
void acpu_send_result2(void *val, uint32_t val_size, uint8_t error_code);

/// @}  acpu_ctrl
/// @}  file

#ifdef __cplusplus
}
#endif
#endif /* __ACPU_CTRL_H__ */

