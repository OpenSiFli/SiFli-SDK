#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "time.h"
#include <rtdevice.h>

/* Common functions for RT-Thread based platform -----------------------------------------------*/

/* User code start from here --------------------------------------------------------*/

/* Slit threshold for multiple calculation, it must be 64 bytes aligned */
#define SPLIT_THRESHOLD 64

/* Semaphore used to wait aes interrupt. */
static rt_sem_t g_hash_sem;

/* to store SHA1 result. */
ALIGN(4)
uint8_t g_sha1_result[20] = {0};

/* to store SHA224 result. */
ALIGN(4)
uint8_t g_sha224_result[28] = {0};

/* to store SHA256 result. */
ALIGN(4)
uint8_t g_sha256_result[32] = {0};

/* to store SM3 result. */
ALIGN(4)
uint8_t g_sm3_result[32] = {0};

/* Input data for test. */
ALIGN(4)
uint8_t g_raw_data[] = "This is a test data used for testing hash algorithm with the hash hardware block.This is a test data used for testing hash algorithm with the hash hardware block.This is a test data used for testing hash algorithm with the hash hardware block.";

/**
  * @brief  Print hexadecimal data.
  * @param  info Customized information.
  * @param  buf Data buffer.
  * @param  len Data length.
  *
  * @retval None
  */
static void dump_hex(const char *info, const uint8_t *buf, size_t len)
{
    size_t i, col;
    col = 0;
    rt_kprintf("%s\n", info);
    for (i = 0; i < len; i++)
    {
        rt_kprintf(" 0x%02x,", buf[i]);
        if (++col == 8)
        {
            rt_kprintf("\n");
            col = 0;
        }
    }
    if (col != 0)
        rt_kprintf("\n");
}

/**
 * @brief Common initialization.
 */
static rt_err_t comm_init(void)
{
    g_hash_sem = rt_sem_create("hash_sem", 0, RT_IPC_FLAG_FIFO);
    return RT_EOK;
}

/*
 *@brief AES_IRQ ISR.
 *
 * AES and HASH share one interrupt.
 */
void AES_IRQHandler(void)
{
    rt_interrupt_enter();

    if (hwp_aes_acc->IRQ)
    {
        /* Clear AES IRQ state. */
        HAL_AES_IRQHandler();
        HAL_HASH_IRQHandler();
        /* Notify that AES or HASH has done. */
        rt_sem_release(g_hash_sem);
    }

    rt_interrupt_leave();
}

/**
 * @brief Do hash , Single calculation, polling mode.
 * @param algo HASH Algorithm type.
 * @param raw_data Input data.
 * @param raw_data_len Input data len.
 * @param result Output data.
 * @param result_len Output data len.
 *
 * @retval none
 */
void hash_run(uint8_t algo, uint8_t *raw_data, uint32_t raw_data_len,
              uint8_t *result, uint32_t result_len)
{
    /* Rest hash block. */
    HAL_HASH_reset();
    /* Initialize AES Hash hardware block. */
    HAL_HASH_init(NULL, algo, 0);
    /* Do hash. HAL_HASH_run will block until hash finish. */
    HAL_HASH_run(raw_data, raw_data_len, 1);
    /* Get hash result. */
    HAL_HASH_result(result);
}

/**
 * @brief Do hash , Single calculation, interrupt mode.
 * @param algo HASH Algorithm type.
 * @param raw_data Input data.
 * @param raw_data_len Input data len.
 * @param result Output data.
 * @param result_len Output data len.
 *
 * @retval none
 */
void hash_run_IT(uint8_t algo, uint8_t *raw_data, uint32_t raw_data_len,
                 uint8_t *result, uint32_t result_len)
{
    /* Rest hash block. */
    HAL_HASH_reset();
    /* Initialize AES Hash hardware block. */
    HAL_HASH_init(NULL, algo, 0);
    /* Do hash. HAL_HASH_run_IT run Async, interrupt will generate when done. */
    HAL_HASH_run_IT(raw_data, raw_data_len, 1);
    /* Wait AES interrupt. */
    rt_err_t ret = rt_sem_take(g_hash_sem, RT_WAITING_FOREVER);
    HAL_ASSERT(RT_EOK == ret);
    /* Get hash result. */
    HAL_HASH_result(result);
}

/**
 * @brief Do hash , multiple calculation.
 * @param algo HASH Algorithm type.
 * @param raw_data Input data.
 * @param raw_data_len Input data len.
 * @param result Output data.
 * @param result_len Output data len.
 *
 * @retval none
 */
void hash_run_split(uint8_t algo, uint8_t *raw_data, uint32_t raw_data_len,
                    uint8_t *result, uint32_t result_len)
{
    int i, last = 0;

    /**
     * Calculate every SPLIT_THRESHOLD bytes.
     * SPLIT_THRESHOLD needs to be 64bytes aligned.
     */
    for (i = 0; i < raw_data_len; i += SPLIT_THRESHOLD)
    {
        /* Check whether current calculation is the last one. */
        last = (i + SPLIT_THRESHOLD >= raw_data_len) ? 1 : 0;
        /* Rest hash block. */
        HAL_HASH_reset();
        /* Initialize AES Hash hardware block. */
        HAL_HASH_init((i == 0) ? NULL : (uint32_t *)result,  /* last result as initialization vector. */
                      algo,                    /* Algorithm type. */
                      i);                      /* The length of data that has been calculated. */
        /* Do hash. HAL_HASH_run will block until hash finish. */
        HAL_HASH_run(&(raw_data[i]),                                  /* Input data. */
                     last ? (raw_data_len - i) : SPLIT_THRESHOLD,  /* Input data length. */
                     last);                                        /* Wether is the last packet.  */
        /* Get hash result. */
        HAL_HASH_result(result);
    }
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    /* Common initialization. */
    comm_init();

    dump_hex("Input raw data:", g_raw_data, sizeof(g_raw_data));

    /**
     * Single calculation.
     */
    /* Polling mode. */
    /* SHA1 algorithm. */
    rt_kprintf("SHA1:\n");
    hash_run(HASH_ALGO_SHA1, g_raw_data, strlen((const char *)g_raw_data), g_sha1_result, sizeof(g_sha1_result));
    dump_hex("HASH Result:", g_sha1_result, sizeof(g_sha1_result));

    /* SHA224 algorithm. */
    rt_kprintf("SHA224:\n");
    hash_run(HASH_ALGO_SHA224, g_raw_data, strlen((const char *)g_raw_data), g_sha224_result, sizeof(g_sha224_result));
    dump_hex("HASH Result:", g_sha224_result, sizeof(g_sha224_result));

    /* SHA256 algorithm. */
    rt_kprintf("SHA256:\n");
    hash_run(HASH_ALGO_SHA256, g_raw_data, strlen((const char *)g_raw_data), g_sha256_result, sizeof(g_sha256_result));
    dump_hex("HASH Result:", g_sha256_result, sizeof(g_sha256_result));

    /* SM3 algorithm. */
    rt_kprintf("SM3:\n");
    hash_run(HASH_ALGO_SM3, g_raw_data, strlen((const char *)g_raw_data), g_sm3_result, sizeof(g_sm3_result));
    dump_hex("HASH Result:", g_sm3_result, sizeof(g_sm3_result));

    /* Interrupt mode.*/
    rt_kprintf("SHA1(IT):\n");
    hash_run_IT(HASH_ALGO_SHA1, g_raw_data, strlen((const char *)g_raw_data), g_sha1_result, sizeof(g_sha1_result));
    dump_hex("HASH Result(IT):", g_sha1_result, sizeof(g_sha1_result));

    /**
     * Multiple calculation.
     * If the data is large, multiple calculations can be considered.
     */
    rt_kprintf("SHA1(multiple):\n");
    hash_run_split(HASH_ALGO_SHA1, g_raw_data, strlen((const char *)g_raw_data), g_sha1_result, sizeof(g_sha1_result));
    dump_hex("HASH Result(multiple):", g_sha1_result, sizeof(g_sha1_result));

    /* Infinite loop */
    while (1)
    {
        /* Do nothing. */
    }

    return 0;
}

