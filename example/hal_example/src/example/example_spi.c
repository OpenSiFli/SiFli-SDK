/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>
#include "utest.h"
#include "bf0_hal.h"

#if defined(HAL_SPI_MODULE_ENABLED)

#ifdef SF32LB52X
    #define TEST_SPI  SPI1
#elif defined(SF32LB57X)
    #define TEST_SPI  SPI2
#else
    #define TEST_SPI  SPI3
#endif

/*
    This example demo:
        1. USE SPI1 CONTROL lsm6dsl sensor for example.
           The sensor bord should be connected to the EVB bord.
        2. For A0 and Z0 evb, it may connect to different SPI and pin mux set.
*/

#define LSM6DSL_ID            0x6AU
#define LSM6DSL_WHO_AM_I      0x0FU

static SPI_HandleTypeDef spi_Handle = {0};

static void spi2_test_init(int lineNum, int mode)  //mode 0:sync mode    mode 1: INT mode   mode 2: DMA mode
{

}

static rt_err_t utest_tc_init(void)
{
    return RT_EOK;
}

static rt_err_t utest_tc_cleanup(void)
{
    return RT_EOK;
}

static void testcase(int argc, char **argv)
{
    uint32_t baundRate = 6000000;
    uint16_t cmd = ((uint16_t)LSM6DSL_WHO_AM_I | 0x80) << 8;   //SPI_DATASIZE_16BIT
    uint16_t pid = 0;
    HAL_StatusTypeDef ret;

    //----------------------------------------------
    // 1. pin mux

#if defined(SOC_SF32LB52X)
#elif defined(SOC_SF32LB57X)
#else
    HAL_PIN_Set(PAD_PB25, GPIO_B25, PIN_PULLUP, 0);             // SPI3_EN
    HAL_PIN_Set(PAD_PB13, SPI3_CLK, PIN_NOPULL, 0);             // SPI3
    HAL_PIN_Set(PAD_PB16, SPI3_DO, PIN_NOPULL, 0);
    HAL_PIN_Set(PAD_PB19, SPI3_DI, PIN_NOPULL, 0);
    HAL_PIN_Set(PAD_PB23, SPI3_CS, PIN_NOPULL, 0);
#endif
    //gpio_set(30);

    //----------------------------------------------
    // 2. spi init

    spi_Handle.Instance = TEST_SPI;
    spi_Handle.Init.Direction = SPI_DIRECTION_2LINES;
    spi_Handle.Init.Mode = SPI_MODE_MASTER;
    spi_Handle.Init.DataSize = SPI_DATASIZE_16BIT;
    spi_Handle.Init.CLKPhase = SPI_PHASE_2EDGE;
    spi_Handle.Init.CLKPolarity = SPI_POLARITY_HIGH;
    spi_Handle.Init.BaudRatePrescaler = (HAL_RCC_GetPCLKFreq(CORE_ID_LCPU, 1) + baundRate / 2) / baundRate;
    spi_Handle.Init.FrameFormat = SPI_FRAME_FORMAT_SPI;
    spi_Handle.Init.SFRMPol = SPI_SFRMPOL_HIGH;
    spi_Handle.State = HAL_SPI_STATE_RESET;
    if (HAL_SPI_Init(&spi_Handle) != HAL_OK)
    {
        uassert_false(RT_TRUE);
        return;
    }

    //----------------------------------------------
    // 3.1. spi sync rtx
    ret = HAL_SPI_TransmitReceive(&spi_Handle, (uint8_t *)&cmd, (uint8_t *)&pid, 1, 1000);
    uassert_true(ret == HAL_OK);
    pid &= 0xff;

    LOG_I("get pid %d, expect %d", pid, LSM6DSL_ID);
    uassert_true(pid == LSM6DSL_ID);

}

/*PARAM DESCRIPTION:
    see enum ARG_IDX ahead
*/
UTEST_TC_EXPORT(testcase, "example_spi", utest_tc_init, utest_tc_cleanup, 10);

#endif /* HAL_SPI_MODULE_ENABLED */

