#include <rtconfig.h>
#include <string.h>
#include <stdlib.h>
#include "register.h"
#include "../dfu/dfu.h"
#ifdef PKG_SIFLI_MBEDTLS_BOOT
    #include "mbedtls/cipher.h"
    #include "mbedtls/pk.h"
#endif
#include "secboot.h"

/* out buf size must more than 32 byte */
int sifli_hash_calculate(uint8_t *in, uint32_t in_size, uint8_t *out, uint8_t algo)
{
    int last, i;

    if (!in || !in_size || !out || algo > 3)
        return -1;

    HAL_HASH_reset();
    HAL_HASH_init(NULL, algo, 0);

    if (in_size > SPLIT_THRESHOLD)
    {
        for (i = 0; i < in_size; i += SPLIT_THRESHOLD)
        {
            last = (i + SPLIT_THRESHOLD >= in_size) ? 1 : 0;
            if (i > 0)
            {
                HAL_HASH_reset();
                HAL_HASH_init((uint32_t *)out, algo, last ? i : 0);
            }
            HAL_HASH_run(&in[i], last ? in_size - i : SPLIT_THRESHOLD, last);
            HAL_HASH_result(out);
        }
        HAL_HASH_result(out);
    }
    else
    {
        HAL_HASH_run(in, in_size, 1);
        HAL_HASH_result(out);
    }

    return 0;
}

int sifli_hash_verify(uint8_t *data, uint32_t data_size, uint8_t *hash, uint32_t hash_size)
{
    uint8_t hash_out[32] = {0};

    if (!data || !hash)
        return -1;

    if (sifli_hash_calculate(data, data_size, hash_out, HASH_ALGO_SHA256))
        return -1;

    if (memcmp(hash_out, hash, hash_size))
        return -1;

    return 0;
}

int sifli_sigkey_pub_verify(uint8_t *sigkey, uint32_t key_size)
{
    uint32_t size = 0;

    uint8_t sigkey_hash[DFU_SIG_HASH_SIZE] = {0};
    size = sifli_hw_efuse_read(EFUSE_ID_SIG_HASH, sigkey_hash, DFU_SIG_HASH_SIZE);
    if (size == DFU_SIG_HASH_SIZE)
        return sifli_hash_verify(sigkey, key_size, sigkey_hash, DFU_SIG_HASH_SIZE);
    else
        return -1;
}

#ifdef PKG_SIFLI_MBEDTLS_BOOT
int sifli_img_sig_hash_verify(uint8_t *img_hash_sig, uint8_t *sig_pub_key, uint8_t *image, uint32_t img_size)
{
    uint8_t img_hash[32] = {0};
    mbedtls_pk_context pk;

    /*1.calculate image hash*/
    if (sifli_hash_calculate(image, img_size, img_hash, HASH_ALGO_SHA256))
        return -1;

    /*2.verify image hash digital signature*/
    mbedtls_pk_init(&pk);
    if (mbedtls_pk_parse_public_key(&pk, sig_pub_key, DFU_SIG_KEY_SIZE))
        return -1;
    mbedtls_rsa_set_padding((mbedtls_rsa_context *)pk.pk_ctx, MBEDTLS_RSA_PKCS_V15, MBEDTLS_MD_SHA256);
    if (mbedtls_pk_verify(&pk, MBEDTLS_MD_SHA256, img_hash, DFU_IMG_HASH_SIZE, img_hash_sig, DFU_SIG_SIZE))
        return -1;

    return 0;
}
#endif

void sifli_secboot_exception(uint8_t excpt)
{
    char *err = NULL;

    switch (excpt)
    {
    case SECBOOT_SIGKEY_PUB_ERR:
        err = "secboot sigkey pub err!";
        boot_uart_tx(hwp_usart1, (uint8_t *)err, strlen(err));
        break;
    case SECBOOT_IMG_HASH_SIG_ERR:
        err = "secboot img hash sig err!";
        boot_uart_tx(hwp_usart1, (uint8_t *)err, strlen(err));
        break;
    default:
        err = "secboot excpt null!";
        boot_uart_tx(hwp_usart1, (uint8_t *)err, strlen(err));
        break;
    }

    HAL_sw_breakpoint();
}
