
#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>
#include "utest.h"
#include "bf0_hal.h"
#include "tc_utils.h"


#if defined(HAL_NNACC_MODULE_ENABLED)

/* Example Description:
 *
 *  Calculate depthwise_conv2d with parameters defined below with NNACC,
 *  then check the result with software.
 *
 *  Kernel&Input&Bias will fill with random data.
 *
 *
 */



/***********************
    Parameters used in depthwise_conv2d
************************/
//Kernel
#define  NNACC_KER_X            6
#define  NNACC_KER_Y            15
//Input
#define  NNACC_INPUT_X          62
#define  NNACC_INPUT_Y          3
#define  NNACC_INPUT_CHANNELS   124
//Output
#define  NNACC_OUTPUT_X         6
#define  NNACC_OUTPUT_Y         3
#define  NNACC_OUTPUT_CHANNELS  124
//Padding
#define NNACC_PADDING_X         5
#define NNACC_PADDING_Y         13
//Stride
#define NNACC_STRIDE_X          4
#define NNACC_STRIDE_Y          2
//Bias shift
#define NNACC_BIAS_SHIFT        12
//Out shift
#define NNACC_OUT_SHIFT         9







#define NNACC_ALIGN4(size)           (((size) + 3) & ~(3))
#define  NNACC_KER_ARRAY_SIZE     NNACC_ALIGN4(NNACC_KER_X * NNACC_KER_Y * NNACC_OUTPUT_CHANNELS)
#define  NNACC_INPUT_ARRAY_SIZE   NNACC_ALIGN4(NNACC_INPUT_X * NNACC_INPUT_Y * NNACC_INPUT_CHANNELS)
#define  NNACC_OUTPUT_ARRAY_SIZE  NNACC_ALIGN4(NNACC_OUTPUT_X * NNACC_OUTPUT_Y * NNACC_OUTPUT_CHANNELS)
#define  NNACC_BIAS_ARRAY_SIZE    NNACC_ALIGN4(NNACC_OUTPUT_CHANNELS)




NNACC_HandleTypeDef nn_acc;



NNACC_ConfigTypeDef config =
{
    .mode = HAL_NNACC_MODE_DEPTHWISE_CONV2D,

    .bias_shift = NNACC_BIAS_SHIFT,
    .out_shift = NNACC_OUT_SHIFT,
    .padding_x = NNACC_PADDING_X,
    .padding_y = NNACC_PADDING_Y,

    .stride_x = NNACC_STRIDE_X,
    .stride_y = NNACC_STRIDE_Y,


    .kernel_dim_x = NNACC_KER_X,
    .kernel_dim_y = NNACC_KER_Y,

    .in_dim_x = NNACC_INPUT_X,
    .in_dim_y = NNACC_INPUT_Y,
    .in_ch_num = NNACC_INPUT_CHANNELS,

    .out_dim_x = NNACC_OUTPUT_X,
    .out_dim_y = NNACC_OUTPUT_Y,
    .out_ch_num = NNACC_OUTPUT_CHANNELS,


    .wt   = (const int8_t *)  NULL,
    .bias = (const int8_t *)  NULL,
    .input = (const int8_t *) NULL,
    .output = (int8_t *)      NULL,

};


/*
    Software check NNACC result
*/
static int check_nnacc_depthwise_result(NNACC_ConfigTypeDef *cfg)
{
    int err_cnt = 0;
    int    max_log = 20;

#ifndef NN_ROUND
#define NN_ROUND( out_shift ) ((( uint32_t )1 << out_shift) >> 1)
#endif /* NN_ROUND */


    int       i_out_y, i_out_x, i_ch_out;
    int       i_ker_y, i_ker_x;
    uint16_t  dim_im_in_x  = cfg->in_dim_x;
    uint16_t  dim_im_in_y  = cfg->in_dim_y;
    uint16_t  ch_im_in     = cfg->in_ch_num;
    uint16_t  ch_im_out    = cfg->out_ch_num;
    uint16_t  dim_kernel_x = cfg->kernel_dim_x;
    uint16_t  dim_kernel_y = cfg->kernel_dim_y;
    uint16_t  padding_x    = cfg->padding_x;
    uint16_t  padding_y    = cfg->padding_y;
    uint16_t  stride_x     = cfg->stride_x;
    uint16_t  stride_y     = cfg->stride_y;
    uint16_t  bias_shift   = cfg->bias_shift;
    uint16_t  out_shift    = cfg->out_shift;
    uint16_t  dim_im_out_x = cfg->out_dim_x;
    uint16_t  dim_im_out_y = cfg->out_dim_y;

    uint32_t  ker_addr     = (uint32_t) cfg->wt;
    uint32_t  input_addr   = (uint32_t) cfg->input;
    uint32_t  bias_addr    = (uint32_t) cfg->bias;
    uint32_t  out_addr     = (uint32_t) cfg->output;



    int8_t data, wt, input, conv_out_sat;
    int8_t    bias;
    for (i_out_y = 0; i_out_y < dim_im_out_y; i_out_y++)
    {
        //printf( "ref row cal done\n" );
        for (i_out_x = 0; i_out_x < dim_im_out_x; i_out_x++)
        {
            for (i_ch_out = 0; i_ch_out < ch_im_out; i_ch_out++)
            {
                //printf( "ref cal begin\n" );
                // for each output
                bias = *(volatile int8_t *)(bias_addr + i_ch_out);
                int       conv_out = ((int32_t)(bias) << bias_shift) + NN_ROUND(out_shift);
                for (i_ker_y = 0; i_ker_y < dim_kernel_y; i_ker_y++)
                {
                    for (i_ker_x = 0; i_ker_x < dim_kernel_x; i_ker_x++)
                    {
                        int       in_row = stride_y * i_out_y + i_ker_y - padding_y;
                        int       in_col = stride_x * i_out_x + i_ker_x - padding_x;
                        if (in_row >= 0 && in_col >= 0 && in_row < dim_im_in_y && in_col < dim_im_in_x)
                        {
                            input   = *(volatile int8_t *)(input_addr + (in_row * dim_im_in_x + in_col) * ch_im_in + i_ch_out);
                            wt      = *(volatile int8_t *)(ker_addr   + (i_ker_y * dim_kernel_x + i_ker_x) * ch_im_out + i_ch_out);

                            conv_out += input *  wt;
                            //printf( "ker_addr= %x ker=%x input addr=%x input = %x conv_out=%x\n",(ker_addr   + (i_ker_y * dim_kernel_x + i_ker_x) * ch_im_out + i_ch_out),wt,(input_addr + (in_row * dim_im_in_x + in_col) * ch_im_in + i_ch_out),input,conv_out );
                        }
                    }
                }
                conv_out_sat = (int8_t) __SSAT((conv_out >> out_shift), 8);
                data         = *(volatile int8_t *)(out_addr + (i_out_y * dim_im_out_x + i_out_x) * ch_im_out + i_ch_out) ;
                if (conv_out_sat != data)
                {
                    if (max_log > 0)
                    {
                        LOG_E("out_x=%d out_y=%d out_ch=%d, rslt=%x, rslt_sat=%x nn_rslt=%x", i_out_x, i_out_y, i_ch_out, conv_out, conv_out_sat, data);
                        max_log--;
                    }

                    err_cnt ++ ;

                }
            }
        }
    }

    return err_cnt;


}



void NNACC_IRQHandler(void)
{
    if (nn_acc.instance->IRQ & NN_ACC_IRQ_ISR)
    {
        LOG_I("NNACC finished... \n check result...\n");
    }
    nn_acc.instance->IRQ |= NN_ACC_IRQ_ICR;
    nn_acc.instance->IRQ &= ~NN_ACC_IRQ_ICR;
}


static rt_err_t utest_tc_init(void)
{

    config.output = (int8_t *)      rt_malloc(NNACC_OUTPUT_ARRAY_SIZE);
    config.wt   = (const int8_t *)  rt_malloc(NNACC_KER_ARRAY_SIZE);
    config.input = (const int8_t *) rt_malloc(NNACC_INPUT_ARRAY_SIZE);
    config.bias = (const int8_t *)  rt_malloc(NNACC_BIAS_ARRAY_SIZE);

    if (config.wt && config.bias && config.input && config.output)
        return RT_EOK;
    else
    {
        rt_free((void *)config.wt);
        rt_free((void *)config.bias);
        rt_free((void *)config.input);
        rt_free((void *)config.output);

        return RT_ERROR;
    }
}

static rt_err_t utest_tc_cleanup(void)
{
    rt_free((void *)config.wt);
    rt_free((void *)config.bias);
    rt_free((void *)config.input);
    rt_free((void *)config.output);
    return RT_EOK;
}

static void testcase(int argc, char **argv)
{
    HAL_StatusTypeDef ret = HAL_ERROR;

    LOG_I("testcase start.");


    /* 1. Initialize NNACC */
    nn_acc.instance = hwp_nnacc;
#ifdef SF32LB55X
    HAL_RCC_EnableModule(RCC_MOD_NNACC);
#else
    if (hwp_nnacc1 == nn_acc.instance)
        HAL_RCC_EnableModule(RCC_MOD_NNACC1);
    else
        HAL_RCC_EnableModule(RCC_MOD_NNACC2);
#endif /* SF32LB55X */
    ret = HAL_NNACC_Init(&nn_acc);

    if (HAL_OK == ret)
    {
        LOG_I("fill_with_random_data. %x %x %x  %x, sp=%x", config.wt, config.bias, config.input, config.output, &ret);

        /*Fill random data for test*/
        fill_with_random_data((uint8_t *)config.wt,      NNACC_KER_ARRAY_SIZE);
        fill_with_random_data((uint8_t *)config.bias,    NNACC_BIAS_ARRAY_SIZE);
        fill_with_random_data((uint8_t *)config.input,   NNACC_INPUT_ARRAY_SIZE);
        fill_with_random_data((uint8_t *)config.output,  NNACC_OUTPUT_ARRAY_SIZE);

        LOG_I("HAL_NNACC_Start.");

        /* 2. run NNACC */
        ret = HAL_NNACC_Start(&nn_acc, &config);

        if (HAL_OK == ret)
        {
            /* 3. Wait NNACC finish IRQ   */
            rt_thread_mdelay(1000);

            LOG_I("Check NNACC result.");

            /*4. Check nnacc result with soft nn*/
            if (0 == check_nnacc_depthwise_result(&config))
            {
                LOG_I("nnacc result OK.");
            }
            else
            {
                LOG_I("nnacc result Error!");
            }
        }
        else
        {
            LOG_I("nnacc Start Error %d", ret);
        }
    }
    else
    {
        LOG_I("nnacc Init Error %d", ret);
    }
}

#ifndef HAL_USING_HTOL
    UTEST_TC_EXPORT(testcase, "example_nnacc", utest_tc_init, utest_tc_cleanup, 10);
#endif
#endif /* HAL_NNACC_MODULE_ENABLED */

