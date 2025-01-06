#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include "time.h"
#include <rtdevice.h>

/* Common functions for RT-Thread based platform -----------------------------------------------*/

/* User code start from here --------------------------------------------------------*/

/* Semaphore used to wait aes interrupt. */
static rt_sem_t g_aes_sem;

/*
 * Initialization Vector.
 * 1. IV is 128-bit(16bytes) for AES algorithm.
 * 2. ECB mode does not need Initialization Vector.
 */
ALIGN(4) static uint8_t g_iv[16] =
{
    0xf0, 0xd7, 0x77, 0x7f, 0x61, 0x6f, 0x7c, 0x89,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

/*
 * AES algorithm key defined by user, used for encryption and decryption.
 * Key length supports:
 * - 128 bits (AES-128)
 * - 196 bits (AES-192)
 * - 256 bits (AES-256)
 */
/* This key is AES-128. */
ALIGN(4)                        // Make sure g_key and g_nounce_counter 4bytes aligned
static uint8_t g_secret_Key_128[16] =
{
    0x32, 0x31, 0x35, 0x31, 0x43, 0x32, 0x34, 0x41,
    0x35, 0x32, 0x32, 0x32, 0x46, 0x45, 0x41, 0x31
};

/* This key is AES-192. */
ALIGN(4)                        // Make sure g_key and g_nounce_counter 4bytes aligned
static uint8_t g_secret_Key_192[24] =
{
    0x32, 0x31, 0x35, 0x31, 0x43, 0x32, 0x34, 0x41,
    0x35, 0x32, 0x32, 0x32, 0x46, 0x45, 0x41, 0x31,
    0x30, 0x31, 0x32, 0x33, 0x33, 0x32, 0x31, 0x30
};

/* This key is AES-256. */
ALIGN(4)                        // Make sure g_key and g_nounce_counter 4bytes aligned
static uint8_t g_secret_Key_256[32] =
{
    0x32, 0x31, 0x35, 0x31, 0x43, 0x32, 0x34, 0x41,
    0x35, 0x32, 0x32, 0x32, 0x46, 0x45, 0x41, 0x31,
    0x30, 0x31, 0x32, 0x33, 0x33, 0x32, 0x31, 0x30,
    0x35, 0x32, 0x32, 0x32, 0x46, 0x45, 0x41, 0x31
};

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
    g_aes_sem = rt_sem_create("aes_sem", 0, RT_IPC_FLAG_FIFO);
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
        rt_sem_release(g_aes_sem);
    }

    rt_interrupt_leave();
}

/**
 * @brief This function used to demonstrate AES encryption and decryption.
 *
 * @param secret_key Secret key.
 * @param key_size Key length. 16:AES-128 24:AES-192 32:AES-256
 * @param iv 128bit initialization vector.
 * @param mode AES mode, supports AES_MODE_CBC, AES_MODE_ECB and AES_MODE_CBC.
 *
 * @retval none
 */
void aes_encry_and_decry(uint8_t *secret_key, uint32_t key_size, uint8_t *iv, uint32_t mode)
{
    /*sync
     * WARNING:
     * If input memory and output memory of AES are in PSRAM, cache synchronization needs to be considered.
     * 1. Need to invalidate dcache before AES run, to make sure that input data is synchronized.
     * 2. Before reading output data, it may also be necessary to invalidate dcache.
     */
    /* Raw data that requires encryption. */
    uint8_t raw_data[16] =
    {
        0x43, 0x4A, 0x4B, 0x2f, 0x31, 0x38, 0x32, 0x31,
        0x39, 0x31, 0x31, 0x32, 0x31, 0x35, 0x36, 0x2E,
    };
    /* To store ciphertext. */
    uint8_t ciphertext[16] = {0};
    /* To store plaintext. */
    uint8_t plaintext[16] = {0};

    dump_hex("Raw data:", raw_data, sizeof(raw_data));

    /* Encryption */
    /* Initialize AES hardware block. */
    HAL_AES_init((uint32_t *)secret_key, key_size, (uint32_t *)iv, mode);
    /* Do encryption. HAL_AES_run will block until encryption finish. */
    HAL_AES_run(AES_ENC, raw_data, ciphertext, sizeof(raw_data));
    /* Print ciphertext. */
    dump_hex("Encry. ciphertext:", ciphertext, sizeof(ciphertext));

    /* Decryption */
    /* Initialize AES hardware block. */
    HAL_AES_init((uint32_t *)secret_key, key_size, (uint32_t *)iv, mode);
    /* Do decryption. HAL_AES_run will block until decryption finish. */
    HAL_AES_run(AES_DEC, ciphertext, plaintext, sizeof(ciphertext));
    /* Print decrypted data. */
    dump_hex("Decry. plaintext:", plaintext, sizeof(plaintext));
}

/**
 * @brief This function used to demonstrate AES encryption in interrupt mode.
 *
 * @param secret_key Secret key.
 * @param key_size Key length. 16:AES-128 24:AES-192 32:AES-256
 * @param iv 128bit initialization vector.
 * @param mode AES mode, supports AES_MODE_CBC, AES_MODE_ECB and AES_MODE_CBC.
 *
 * @retval none
 */
void aes_encry_IT(uint8_t *secret_key, uint32_t key_size, uint8_t *iv, uint32_t mode)
{
    /* Raw data that requires encryption. */
    uint8_t raw_data[16] =
    {
        0x43, 0x4A, 0x4B, 0x2f, 0x31, 0x38, 0x32, 0x31,
        0x39, 0x31, 0x31, 0x32, 0x31, 0x35, 0x36, 0x2E,
    };
    /* To store ciphertext. */
    uint8_t ciphertext[16] = {0};

    dump_hex("Raw data:", raw_data, sizeof(raw_data));

    /* Encryption */
    /* Initialize AES hardware block. */
    HAL_AES_init((uint32_t *)secret_key, key_size, (uint32_t *)iv, mode);
    /* Enable AES IRQ. */
    NVIC_EnableIRQ(AES_IRQn);
    /* Do encryption. HAL_AES_run_IT run Async, interrupt will generate when done. */
    HAL_AES_run_IT(AES_ENC, raw_data, ciphertext, sizeof(raw_data));
    /* Wait AES interrupt. */
    rt_err_t ret = rt_sem_take(g_aes_sem, RT_WAITING_FOREVER);
    HAL_ASSERT(RT_EOK == ret);
    /* Print ciphertext. */
    dump_hex("Encry. ciphertext:", ciphertext, sizeof(ciphertext));
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

    /* Using CBC mode(AES_MODE_CBC) for AES encryption and decryption. */
    rt_kprintf("AES CBC Mode:\n");
    aes_encry_and_decry(g_secret_Key_128, sizeof(g_secret_Key_128), g_iv, AES_MODE_CBC);

    /* Using CTR mode(AES_MODE_CTR) for AES encryption and decryption. */
    rt_kprintf("\nAES CTR Mode:\n");
    aes_encry_and_decry(g_secret_Key_192, sizeof(g_secret_Key_192), g_iv, AES_MODE_CTR);

    /* Using ECB mode(AES_MODE_ECB) for AES encryption and decryption. */
    rt_kprintf("\nAES ECB Mode:\n");
    aes_encry_and_decry(g_secret_Key_256, sizeof(g_secret_Key_256), NULL, AES_MODE_ECB);

    /* Interrupt mode for AES encryption. */
    rt_kprintf("\nAES encryption interrupt mode:\n");
    aes_encry_IT(g_secret_Key_128, sizeof(g_secret_Key_128), g_iv, AES_MODE_CBC);

    /* Infinite loop */
    while (1)
    {
        /* Do nothing. */
    }

    return 0;
}

