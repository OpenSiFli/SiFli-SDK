#include "rtthread.h"
#include "bf0_hal.h"
#include "drv_io.h"
#include "stdio.h"
#include "string.h"
#include <rtdevice.h>
#include "bf0_hal_ezip.h"


/* Common functions for RT-Thread based platform -----------------------------------------------*/

/* User code start from here --------------------------------------------------------*/

/* With LCD device. */
#define EXAMPLE_WITH_LCD 0

#if EXAMPLE_WITH_LCD
    static rt_device_t g_lcd_device = NULL;
#endif

/* Semaphore used to wait ezip interrupt. */
static rt_sem_t g_ezip_sem;
/* EZIP instance. */
static EZIP_HandleTypeDef g_ezip_handle = {0};

/* Test data. */
ALIGN(4)  /* Source and destination address must be 4bytes aligned. */
const static uint8_t png_data_argb565[] =
{
#include "..\assets\clock_mickey_shoe01_565A_s_ezip.dat"
};
ALIGN(4)
const static uint8_t exp_data_argb565[] =
{
#include "..\assets\clock_mickey_shoe01_565A.dat"
};
ALIGN(4)  /* Source and destination address must be 4bytes aligned. */
const static uint8_t gzip_data[] =
{
#include "..\assets\gzip_input.dat"
};
ALIGN(4)
const static uint8_t exp_data_gzip[] =
{
#include "..\assets\lz4_gzip_output.dat"
};
ALIGN(4)  /* Source and destination address must be 4bytes aligned. */
const static uint8_t lz4_data[] =
{
#include "..\assets\lz4_input.dat"
};
ALIGN(4)
const static uint8_t exp_data_lz4[] =
{
#include "..\assets\lz4_gzip_output.dat"
};
// Test data.


/**
 * @brief Common initialization.
 */
static rt_err_t comm_init(void)
{
    g_ezip_sem = rt_sem_create("ezip_sem", 0, RT_IPC_FLAG_FIFO);
    return RT_EOK;
}

#if EXAMPLE_WITH_LCD
/**
 * @brief Open LCD device .
 */
static void lcd_device_init(void)
{
    g_lcd_device = rt_device_find("lcd");

    HAL_ASSERT(g_lcd_device != NULL);

    rt_err_t ret = rt_device_open(g_lcd_device, RT_DEVICE_OFLAG_RDWR);
    if (RT_EOK != ret)
    {
        rt_kprintf("[EZIP]Failed to open lcd device.\n");
        HAL_ASSERT(0);
    }
}
#endif

#ifdef HAL_EZIP_MODULE_ENABLED
/**
 * @brief EZIP_IRQ ISR.
 */
void EZIP_IRQHandler(void)
{
    rt_interrupt_enter();

    /* g_ezip_handle.CpltCallback will be called when EZIP_INT_STA_END_INT_STA. */
    HAL_EZIP_IRQHandler(&g_ezip_handle);

    rt_interrupt_leave();
}
#endif

/* EZIP cpltcallback. Invoked by HAL_EZIP_IRQHandler when EZIP_INT_STA_END_INT_STA. */
static void ezip_cpltcallback(EZIP_HandleTypeDef *ezip)
{
    rt_kprintf("[EZIP]ezip_done.\n");
    rt_sem_release(g_ezip_sem);
}

/**
 * @brief Ezip initialization.
 */
static void example_ezip_init(void)
{
#ifdef HAL_EZIP_MODULE_ENABLED
    /* Enable IRQ */
    HAL_NVIC_SetPriority(EZIP_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EZIP_IRQn);

    /* Hal init */
    g_ezip_handle.Instance = EZIP;
    HAL_StatusTypeDef ret = HAL_EZIP_Init(&g_ezip_handle);
    HAL_ASSERT(HAL_OK == ret);
#else
#error "HAL_EZIP_MODULE_ENABLED should be defined."
#endif  /* HAL_EZIP_MODULE_ENABLED */
    rt_kprintf("[EZIP]EZIP initialization OK.\n");
}


/**
 * @brief Example : EZIP/AHB output mode. Polling mode.
 *
 * @param img       source data.
 * @param color_fmt color fomat.
 * @param fg_width  width
 * @param fg_height height
 * @param exp_data  expected output data.
 */
static void example_ezip_ahb(const uint8_t *img, uint16_t color_fmt, uint16_t fg_width, uint16_t fg_height, const uint8_t *exp_data)
{
    EZIP_DecodeConfigTypeDef config = {0};
    HAL_StatusTypeDef ret = HAL_OK;
    uint16_t cf = color_fmt;
    uint16_t tl_x = 50;
    uint16_t tl_y = 50;
    uint32_t output_size = 0;

    output_size = fg_width * fg_height * ((RTGRAPHIC_PIXEL_FORMAT_ARGB888 == color_fmt) ? 4 : 3);

    /* WARNING: EZIP's input data and output data must be 4 bytes aligned. */
    config.input = (uint8_t *)img;
    config.output = rt_malloc(output_size);
    RT_ASSERT(config.output);
    memset(config.output, 0, output_size);
    /* Invalidate cache. */
    SCB_InvalidateDCache_by_Addr(config.output, output_size);

    config.start_x = 0;
    config.start_y = 0;
    config.width = fg_width;
    config.height = fg_height;
    config.work_mode = HAL_EZIP_MODE_EZIP;       /* Work mode. See enum EZIP_WorkModeTypeDef. */
    config.output_mode = HAL_EZIP_OUTPUT_AHB;    /* Output mode. See enum EZIP_OutputModeTypeDef. */

    ret = HAL_EZIP_Decode(&g_ezip_handle, &config);
    HAL_ASSERT(HAL_OK == ret);

    /* Check whether output data is correct. */
    if (exp_data)
    {
        if (0 == memcmp(exp_data, config.output, output_size))
        {
            rt_kprintf("[EZIP]Output is correct.\n");
        }
        else
        {
            rt_kprintf("[EZIP]Output is incorrect.\n");
            HAL_ASSERT(0);
        }
    }

#if EXAMPLE_WITH_LCD
    /* Display img on LCD. */
    rt_thread_mdelay(500);
    rt_graphix_ops(g_lcd_device)->set_window(tl_x, tl_y, (tl_x + fg_width - 1) | 0x1, (tl_y + fg_height - 1) | 0x1);
    rt_device_control(g_lcd_device, RTGRAPHIC_CTRL_SET_BUF_FORMAT, &cf);
    rt_graphix_ops(g_lcd_device)->draw_rect((const char *)config.output, tl_x, tl_y, tl_x + fg_width - 1, tl_y + fg_height - 1);
    uint8_t brightness = 100;
    rt_device_control(g_lcd_device, RTGRAPHIC_CTRL_SET_BRIGHTNESS, &brightness);
    rt_thread_mdelay(10000);
#endif

    rt_free(config.output);
}

/**
 * @brief Example : EZIP/AHB output mode. Interrupt mode.
 *
 * @param img       source data.
 * @param color_fmt color fomat.
 * @param fg_width  width
 * @param fg_height height
 * @param exp_data  expected output data.
 */
static void example_ezip_ahb_IT(const uint8_t *img, uint16_t color_fmt, uint16_t fg_width, uint16_t fg_height, const uint8_t *exp_data)
{
    EZIP_DecodeConfigTypeDef config = {0};
    HAL_StatusTypeDef ret = HAL_OK;
    rt_err_t err = RT_EOK;
    uint16_t cf = color_fmt;
    uint16_t tl_x = 50;
    uint16_t tl_y = 250;
    uint32_t output_size = 0;

    output_size = fg_width * fg_height * ((RTGRAPHIC_PIXEL_FORMAT_ARGB888 == color_fmt) ? 4 : 3);

    /* WARNING: EZIP's input data and output data must be 4 bytes aligned. */
    config.input = (uint8_t *)img;
    config.output = rt_malloc(output_size);
    RT_ASSERT(config.output);
    memset(config.output, 0, output_size);
    /* Invalidate cache. */
    SCB_InvalidateDCache_by_Addr(config.output, output_size);

    config.start_x = 0;
    config.start_y = 0;
    config.width = fg_width;
    config.height = fg_height;
    config.work_mode = HAL_EZIP_MODE_EZIP;       /* Work mode. See enum EZIP_WorkModeTypeDef. */
    config.output_mode = HAL_EZIP_OUTPUT_AHB;    /* Output mode. See enum EZIP_OutputModeTypeDef. */

    g_ezip_handle.CpltCallback = ezip_cpltcallback;

    ret = HAL_EZIP_Decode_IT(&g_ezip_handle, &config);
    HAL_ASSERT(HAL_OK == ret);

    /* Wait EZIP interrupt. */
    err = rt_sem_take(g_ezip_sem, RT_WAITING_FOREVER);
    HAL_ASSERT(RT_EOK == err);

    /* Invalidate cache. */
    SCB_InvalidateDCache_by_Addr(config.output, output_size);

    /* Check whether output data is correct. */
    if (exp_data)
    {
        if (0 == memcmp(exp_data, config.output, output_size))
        {
            rt_kprintf("[EZIP]Output is correct.\n");
        }
        else
        {
            rt_kprintf("[EZIP]Output is incorrect.\n");
            HAL_ASSERT(0);
        }
    }

#if EXAMPLE_WITH_LCD
    /* Display img on LCD. */
    rt_thread_mdelay(500);
    rt_graphix_ops(g_lcd_device)->set_window(tl_x, tl_y, (tl_x + fg_width - 1) | 0x1, (tl_y + fg_height - 1) | 0x1);
    rt_device_control(g_lcd_device, RTGRAPHIC_CTRL_SET_BUF_FORMAT, &cf);
    rt_graphix_ops(g_lcd_device)->draw_rect((const char *)config.output, tl_x, tl_y, tl_x + fg_width - 1, tl_y + fg_height - 1);
    uint8_t brightness = 100;
    rt_device_control(g_lcd_device, RTGRAPHIC_CTRL_SET_BRIGHTNESS, &brightness);
    rt_thread_mdelay(10000);
#endif

    rt_free(config.output);
}

/**
 * @brief Example : LZ4/AHB output mode. Polling mode.
 *
 * @param input       source data.
 * @param output_size output size.
 * @param exp_data    expected output data.
 */
static void example_lz4_ahb(const uint8_t *input, uint32_t output_size, const uint8_t *exp_data)
{
    EZIP_DecodeConfigTypeDef config = {0};
    HAL_StatusTypeDef ret;

    /* WARNING: EZIP's input data and output data must be 4 bytes aligned. */
    config.input = (uint8_t *)input;
    config.output = rt_malloc(output_size);
    RT_ASSERT(config.output);
    memset(config.output, 0, output_size);
    /* Invalidate cache. */
    SCB_InvalidateDCache_by_Addr(config.output, output_size);

    config.start_x = 0;
    config.start_y = 0;
    config.width = 0;
    config.height = 0;
    config.work_mode = HAL_EZIP_MODE_LZ4;        /* Work mode. See enum EZIP_WorkModeTypeDef. */
    config.output_mode = HAL_EZIP_OUTPUT_AHB;    /* Output mode. See enum EZIP_OutputModeTypeDef. */

    /* EZIP decode. */
    ret = HAL_EZIP_Decode(&g_ezip_handle, &config);
    HAL_ASSERT(HAL_OK == ret);

    /* Check whether output data is correct. */
    if (exp_data)
    {
        if (0 == memcmp(exp_data, config.output, output_size))
        {
            rt_kprintf("[EZIP]Output is correct.\n");
        }
        else
        {
            rt_kprintf("[EZIP]Output is incorrect.\n");
            HAL_ASSERT(0);
        }
    }

    rt_free(config.output);
}

/**
 * @brief Example : GZIP/AHB output mode. Polling mode.
 *
 * @param input       source data.
 * @param output_size output size.
 * @param exp_data    expected output data.
 */
static void example_gzip_ahb(const uint8_t *input, uint32_t output_size, const uint8_t *exp_data)
{
    EZIP_DecodeConfigTypeDef config;
    HAL_StatusTypeDef ret;

    /* WARNING: EZIP's input data and output data must be 4 bytes aligned. */
    config.input = (uint8_t *)input;
    config.output = rt_malloc(output_size);
    RT_ASSERT(config.output);
    memset(config.output, 0, output_size);
    /* Invalidate cache. */
    SCB_InvalidateDCache_by_Addr(config.output, output_size);

    config.start_x = 0;
    config.start_y = 0;
    config.width = 0;
    config.height = 0;
    config.work_mode = HAL_EZIP_MODE_GZIP;
    config.output_mode = HAL_EZIP_OUTPUT_AHB;

    /* EZIP decode. */
    ret = HAL_EZIP_Decode(&g_ezip_handle, &config);
    HAL_ASSERT(HAL_OK == ret);

    /* Check whether output data is correct. */
    if (exp_data)
    {
        if (0 == memcmp(exp_data, config.output, output_size))
        {
            rt_kprintf("[EZIP]Output is correct.\n");
        }
        else
        {
            rt_kprintf("[EZIP]Output is incorrect.\n");
            HAL_ASSERT(0);
        }
    }

    rt_free(config.output);
}

/**
  * @brief  Main program
  * @param  None
  * @retval 0 if success, otherwise failure number
  */
int main(void)
{
    uint32_t output_size = 0;

    rt_kprintf("\n[EZIP]EZIP Example:\n");

    /* Common initialization. */
    comm_init();
#if EXAMPLE_WITH_LCD
    /* Open lcd. */
    lcd_device_init();
#endif

    /* Ezip initialization. */
    example_ezip_init();

    /* WORK MODE:EZIP OUTPUT:AHB . Polling mode. */
    rt_kprintf("[EZIP]EZIP AHB (polling mode).\n");
    example_ezip_ahb(png_data_argb565, RTGRAPHIC_PIXEL_FORMAT_ARGB565, 68, 37, exp_data_argb565);
    rt_kprintf("[EZIP]EZIP AHB (polling mode)  --- end.\n");

    /* WORK MODE:EZIP OUTPUT:AHB . IntrInterrupt mode. */
    rt_kprintf("[EZIP]EZIP AHB (intrInterrupt mode).\n");
    example_ezip_ahb_IT(png_data_argb565, RTGRAPHIC_PIXEL_FORMAT_ARGB565, 68, 37, exp_data_argb565);
    rt_kprintf("[EZIP]EZIP AHB (intrInterrupt mode)  --- end.\n");

    /* WORK MODE:LZ4 OUTPUT:AHB . Polling mode. */
    rt_kprintf("[EZIP]LZ4 AHB (polling mode).\n");
    output_size = *(uint32_t *)lz4_data;  /* First 4 bytes is data size. */
    example_lz4_ahb(lz4_data + 4, output_size, exp_data_lz4);
    rt_kprintf("[EZIP]LZ4 AHB (polling mode)  --- end.\n");

    /* WORK MODE:GZIP OUTPUT:AHB . Polling mode. */
    rt_kprintf("[EZIP]GZIP AHB (polling mode).\n");
    output_size = *(uint32_t *)gzip_data;  /* First 4 bytes is data size. */
    example_gzip_ahb(gzip_data + 4, output_size, exp_data_gzip);
    rt_kprintf("[EZIP]GZIP AHB (polling mode)  --- end.\n");

    /* Infinite loop */
    while (1)
    {
        rt_thread_mdelay(10000);
    }

    return 0;
}

