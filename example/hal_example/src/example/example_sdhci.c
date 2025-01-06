
#ifdef SF32LB55X
#ifdef HAL_USING_HTOL

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>
#include "utest.h"
#include "bf0_hal.h"
#include "board.h"

#include "bf0_hal_sd_ex.h"

static SDHCI_HandleTypeDef sd_hal_instance;
static SD_HandleTypeDef sd_ex_instance;

static uint8_t lsd_rxbuf[4096];
static uint8_t lsd_txbuf[4096];

static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}

static int sample_hal_ex_init()
{
    memset(&sd_ex_instance, 0, sizeof(SD_HandleTypeDef));
    sd_hal_instance.Instance = (uint32_t)SDIO2;
    sd_hal_instance.ErrorCode = 0;
    sd_hal_instance.Lock = 0;
    sd_hal_instance.State = 0;
    sd_hal_instance.Init.blocks = 1;

    sd_ex_instance.Instance = &sd_hal_instance;
    sd_ex_instance.pRxBuffPtr = (uint32_t *)lsd_rxbuf;
    sd_ex_instance.pTxBuffPtr = (uint32_t *)lsd_txbuf;
    sd_ex_instance.Init.BusWide = 4;

    return 0;
}

static void testcase(int argc, char **argv)
{
    SD_HandleTypeDef *hsd = &sd_ex_instance;
    char dbuf[1024];
    HAL_StatusTypeDef res;
    int addr, blk, i;

    // 1. init
    sample_hal_ex_init();
    res = HAL_SD_Init(hsd);
    if (res != 0)
        rt_kprintf("HAL_SD_Init fail %d, 0x%08x\n", res, hsd->ErrorCode);
    else
        rt_kprintf("HAL_SD_Init done !!!\n\n");

    // 2. configure
    res = HAL_SD_ConfigWideBusOperation(hsd, 4);
    if (res != 0)
        rt_kprintf("HAL_SD_ConfigWideBusOperation fail %d, 0x%08x\n", res, hsd->ErrorCode);
    else
        rt_kprintf("HAL_SD_ConfigWideBusOperation done !!!\n\n");
    //res = HAL_SD_InitCard(hsd);
    //if(res != 0)
    //    rt_kprintf("HAL_SD_InitCard fail %d, 0x%x\n", res, hsd->ErrorCode);

    // 3. read block
    addr = 0;
    blk = 1;
    rt_kprintf("Read from %d with %d blk\n", addr, blk);
    hsd->State = HAL_SD_STATE_READY;
    hsd->ErrorCode = HAL_DMA_ERROR_NONE;
    res = HAL_SD_ReadBlocks(hsd, (uint8_t *)dbuf, addr, blk, 1000);
    if (res == 0)
        rt_kprintf("Read SD sucess \n");
    else
        rt_kprintf("Read SD fail %d, 0x%08x\n", res, hsd->ErrorCode);

    // 4. write block
    for (i = 0; i < 512; i++)
        dbuf[i] = (uint8_t)(i & 0xff);
    hsd->State = HAL_SD_STATE_READY;
    hsd->ErrorCode = HAL_DMA_ERROR_NONE;
    res = HAL_SD_WriteBlocks(hsd, (uint8_t *)dbuf, addr, 1, 1000);
    if (res != 0)
        rt_kprintf("Write SD fail %d: 0x%08x\n", res, hsd->ErrorCode);

    return;
}

UTEST_TC_EXPORT(testcase, "example_sdmmc", utest_tc_init, utest_tc_cleanup, 10);

#endif //  HAL_USING_HTOL

#endif  //SF32LB55X