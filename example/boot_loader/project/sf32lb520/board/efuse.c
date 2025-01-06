#include "rtconfig.h"
#include "stdint.h"
#include "string.h"
#include "../dfu/dfu.h"
#include "bf0_hal.h"
#include "board.h"

#define AES_BLOCK_SIZE 512

#define EFUSE_OFFSET_UID        0
#define EFUSE_OFFSET_SIG_HASH   128
#define EFUSE_OFFSET_SECURE     192
#define EFUSE_OFFSET_ROOT       768
#define EFUSE_BANK_SIZE         32
#define EFUSE_BANK_NUM          (4)

ALIGN(4)
uint8_t g_uid[DFU_UID_SIZE];
ALIGN(4)
static uint8_t g_aes_ctr_iv[DFU_IV_LEN];

int sifli_hw_efuse_read_bank(uint32_t i)
{
    static uint8_t data[HAL_EFUSE_BANK_SIZE];
    int32_t r = 0;
    int size;
    if (i >= HAL_EFUSE_BANK_NUM)
        return -1;

    size = HAL_EFUSE_Read(HAL_EFUSE_BANK_SIZE * i * 8, data, HAL_EFUSE_BANK_SIZE);;
    if (size == 0)
        r = -2;

    return r;
}




