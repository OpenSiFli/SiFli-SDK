/**
  ******************************************************************************
  * @file   example_dsi.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
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

#include <string.h>
#include <stdlib.h>
#include "utest.h"
#include "bf0_hal.h"
#include "tc_utils.h"

#ifdef HAL_DSI_MODULE_ENABLED

/* Example Description:
 *
 *  Run RM69330 DSI LCD at 484Mbps, 1-datalane,  RGB888 color mode,
 *  and convert an 100x100 pixels RGB565 buffer which color is 0xE0E0 to RGB888 with LCDC,
 *  meanwhile draw it to DSI LCD .
 *
 * +-----------------------+
 * |                       |
 * | (100,100)             |
 * |    +-----+            |
 * |    |     |            |
 * |    |     |            |
 * |    +-----+            |
 * |        (199,199)      |
 * |                       |
 * +-----------------------+
 *
 *
 */


/**********************
 *      MACROS
 **********************/

#if SOC_BF0_HCPU
    #define LCDC_INSTANCE hwp_lcdc1
    #define LCDC_IRQ_NUM   LCDC1_IRQn
#else
    #define LCDC_INSTANCE hwp_lcdc2
    #define LCDC_IRQ_NUM   LCDC2_IRQn
#endif




/**********************
 *      Variables
 **********************/
static LCDC_HandleTypeDef hlcdc_rm69330;



/*DSI LCD configuration*/
static const LCDC_InitTypeDef lcdc_int_cfg =
{
    .lcd_itf = LCDC_INTF_DSI,                              //LCDC output interface
    .freq = DSI_FREQ_480Mbps,                               //DSI clk lane freqence(DDR freqence)
    .color_mode = LCDC_PIXEL_FORMAT_RGB888,

    .cfg = {

        .dsi = {

            .Init = {
                .AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_ENABLE,        //Auto gate clock to save power
                .NumberOfLanes = DSI_ONE_DATA_LANE,                                 //Numbers of data-lane
                .TXEscapeCkdiv = 0x4,                                               //Escape clk = lcdc_int_cfg_dsi.freq / 2(DDR) / 8 / TXEscapeCkdiv
            },

            .CmdCfg = {
                .VirtualChannelID      = 0,                             //Fixed 0,  DBI virtual channel id
                .CommandSize           = 0xFFFF,                        //Maximum write bytes of every partial write
                .TEAcknowledgeRequest  = DSI_TE_ACKNOWLEDGE_DISABLE,     //Open or Close TE
            },

            .PhyTimings = { /*DSI PHY config */
                .ClockLaneHS2LPTime = 35,  /* The maximum time that the D-PHY clock lane takes to go from high-speed
                                              to low-power transmission                                              */
                .ClockLaneLP2HSTime = 35,  /* The maximum time that the D-PHY clock lane takes to go from low-power
                                              to high-speed transmission                                             */
                .DataLaneHS2LPTime = 35,   /* The maximum time that the D-PHY data lanes takes to go from high-speed
                                              to low-power transmission                                              */
                .DataLaneLP2HSTime = 35,   /* The maximum time that the D-PHY data lanes takes to go from low-power
                                              to high-speed transmission                                             */

                .DataLaneMaxReadTime = 0,  /* The maximum time required to perform a read command */
                .StopWaitTime = 0,        /* The minimum wait period to request a High-Speed transmission after the
                                              Stop state                                                             */
            },

            .HostTimeouts = {
                .TimeoutCkdiv = 1,                          //Timeout clk = lcdc_int_cfg_dsi.freq / 2 / 8 / TimeoutCkdiv
                .HighSpeedTransmissionTimeout = 0,
                .LowPowerReceptionTimeout = 0,
                .HighSpeedReadTimeout = 0,
                .LowPowerReadTimeout = 0,
                .HighSpeedWriteTimeout = 0,
                //.HighSpeedWritePrespMode = DSI_HS_PM_DISABLE,
                .LowPowerWriteTimeout = 0,
                .BTATimeout = 0,
            },


            .LPCmd = {
                .LPGenShortWriteNoP    = DSI_LP_GSW0P_ENABLE,     //Generic short write 0 parameter use LP mode
                .LPGenShortWriteOneP   = DSI_LP_GSW1P_ENABLE,     //Generic short write 1 parameter use LP mode
                .LPGenShortWriteTwoP   = DSI_LP_GSW2P_ENABLE,     //Generic short write 2 parameter use LP mode
                .LPGenShortReadNoP     = DSI_LP_GSR0P_ENABLE,     //Generic short read 0 parameter use LP mode
                .LPGenShortReadOneP    = DSI_LP_GSR1P_ENABLE,     //Generic short read 0 parameter use LP mode
                .LPGenShortReadTwoP    = DSI_LP_GSR2P_ENABLE,     //Generic short read 0 parameter use LP mode
                .LPGenLongWrite        = DSI_LP_GLW_ENABLE,       //Generic long write use LP mode
                .LPDcsShortWriteNoP    = DSI_LP_DSW0P_ENABLE,     //Dcs short write 0 parameter use LP mode
                .LPDcsShortWriteOneP   = DSI_LP_DSW1P_ENABLE,     //Dcs short write 1 parameter use LP mode
                .LPDcsShortReadNoP     = DSI_LP_DSR0P_ENABLE,     //Dcs short read 0 parameter use LP mode
                .LPDcsLongWrite        = DSI_LP_DLW_DISABLE,      //Dcs long write use HS mode
                .LPMaxReadPacket       = DSI_LP_MRDP_ENABLE,      //Maximum Read Packet Size
                .AcknowledgeRequest    = DSI_ACKNOWLEDGE_DISABLE, //Request acknowledge
            },


            .vsyn_delay_us = 1000,  //The delay during receive TE and send framebuffer. If TE was disable ignore it.
        },
    },
};


/**********************
 *   FUNCTIONS
 **********************/

__ROM_USED void LCDC1_IRQHandler(void)
{
    /* enter interrupt */
    ENTER_INTERRUPT();

    HAL_LCDC_IRQHandler(&hlcdc_rm69330);

    /* leave interrupt */
    LEAVE_INTERRUPT();
}



void HAL_LCDC_SendLayerDataCpltCbk(LCDC_HandleTypeDef *lcdc)
{
    LOG_I("Send layer buffer done.");
}

/*
    RM69330 initialization sequence
 */
static void RM69330_Init_Seq(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t   parameter[14];

    parameter[0] = 0x07;
    HAL_LCDC_WriteU8Reg(hlcdc, 0xFE, parameter, 1); //page 6
    {
        parameter[0] = 0x04;
        HAL_LCDC_WriteU8Reg(hlcdc, 0x15, parameter, 1); //SRAM read adjust control
    }

    parameter[0] = 0x00;
    HAL_LCDC_WriteU8Reg(hlcdc, 0xFE, parameter, 1); //User cmd
    {
        //Enable TE output
        parameter[0] = 0x00;
        HAL_LCDC_WriteU8Reg(hlcdc, 0x35, parameter, 1);

        /*Set color mode*/
        if (LCDC_PIXEL_FORMAT_RGB888 == lcdc_int_cfg.color_mode)
            parameter[0] = 0x77; //24bit rgb
        else if (LCDC_PIXEL_FORMAT_RGB565 == lcdc_int_cfg.color_mode)
            parameter[0] = 0x75; //16bit rgb
        else
            ;
        HAL_LCDC_WriteU8Reg(hlcdc, 0x3A, parameter, 1);

        //Set brightness maximum
        parameter[0] = 0xFF;
        HAL_LCDC_WriteU8Reg(hlcdc, 0x51, parameter, 1);


        /* Wait for 110ms */
        rt_thread_mdelay(110);

        //Sleep out
        HAL_LCDC_WriteU8Reg(hlcdc, 0x11, (uint8_t *)NULL, 0);

        /* Wait for 150ms */
        rt_thread_mdelay(150);

        /* Display On */
        HAL_LCDC_WriteU8Reg(hlcdc, 0x29, (uint8_t *)NULL, 0);

        /* Wait for 150ms */
        rt_thread_mdelay(150);
    }
}


/*Set LCD receive area*/
void RM69330_SetRegion(LCDC_HandleTypeDef *hlcdc, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint8_t   parameter[4];

    parameter[0] = (x0) >> 8;
    parameter[1] = (x0) & 0xFF;
    parameter[2] = (x1) >> 8;
    parameter[3] = (x1) & 0xFF;
    HAL_LCDC_WriteU8Reg(hlcdc, 0x2a, parameter, 4);

    parameter[0] = (y0) >> 8;
    parameter[1] = (y0) & 0xFF;
    parameter[2] = (y1) >> 8;
    parameter[3] = (y1) & 0xFF;
    HAL_LCDC_WriteU8Reg(hlcdc, 0x2b, parameter, 4);
}


static void Poweron_LCD_Board(void)
{
    /*
        EVB A0 use GPIO_A79 (GPIO1 PIN79) as LCD board power control PIN,
        so power on it first.
    */

    HAL_PIN_Set(PAD_PA79, GPIO_A79, PIN_PULLUP, 1);
    //Set PIN 0 to high
    {
        GPIO_TypeDef *gpio = hwp_gpio1;
        GPIO_InitTypeDef GPIO_InitStruct;
        uint16_t pin = 79;

        /* set GPIO1 pin10 to output mode */
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
        GPIO_InitStruct.Pin = pin;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(gpio, &GPIO_InitStruct);

        /* set pin to high */
        HAL_GPIO_WritePin(gpio, pin, GPIO_PIN_SET);
    }

}

static void RM69330_LCD_Reset(uint8_t high1_low0)
{
    /*
        LB555 use GPIO_B17 (GPIO2 PIN17) as LCD reset PIN
    */

    HAL_PIN_Set(PAD_PB17, GPIO_B17, PIN_NOPULL, 0);
    {
        GPIO_TypeDef *gpio = hwp_gpio2;
        GPIO_InitTypeDef GPIO_InitStruct;
        uint16_t pin = 17;

        /* set GPIO1 pin10 to output mode */
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
        GPIO_InitStruct.Pin = pin;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(gpio, &GPIO_InitStruct);

        /* set pin to high */
        HAL_GPIO_WritePin(gpio, pin, (1 == high1_low0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

static void testcase(int argc, char **argv)
{
    uint8_t *p_framebuffer = (uint8_t *) rt_malloc(100 * 100 * 2);

    if (NULL == p_framebuffer) return;

    /*Turn on LCD board power supply*/
    Poweron_LCD_Board();


    memset(&hlcdc_rm69330, 0, sizeof(hlcdc_rm69330));

    hlcdc_rm69330.Instance = LCDC_INSTANCE;

    /*Set background layer color*/
    hlcdc_rm69330.bg.r = 0;
    hlcdc_rm69330.bg.g = 0;
    hlcdc_rm69330.bg.b = 255;

    memcpy(&hlcdc_rm69330.Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));

    /*Open DSI & LCDC */
    HAL_LCDC_Init(&hlcdc_rm69330);

    /*  Reset LCD by RESX pin */
    RM69330_LCD_Reset(0);
    HAL_Delay_us(20);
    RM69330_LCD_Reset(1);

    /* Wait for 200ms */
    rt_thread_mdelay(200);


    /*DSI LCD initial */
    RM69330_Init_Seq(&hlcdc_rm69330);

    {
        //Read LCD id

        uint32_t rd_data = 0;

        HAL_LCDC_ReadU8Reg(&hlcdc_rm69330, 0x04, (uint8_t *)&rd_data, 4);
        LOG_I("rm69330 id is %x", rd_data);
    }

    /*Set default layer configuration*/
    HAL_LCDC_LayerReset(&hlcdc_rm69330, HAL_LCDC_LAYER_DEFAULT);

    /*Disable layer compress*/
    HAL_LCDC_LayerSetCmpr(&hlcdc_rm69330, HAL_LCDC_LAYER_DEFAULT, 0);
    /*
        Set layer format RGB565, and LCDC will covert to RGB888 as lcdc_init_cfg.color_mode is RGB888
    */
    HAL_LCDC_LayerSetFormat(&hlcdc_rm69330, HAL_LCDC_LAYER_DEFAULT, LCDC_PIXEL_FORMAT_RGB565);

    /*Fill frambuffer with single color*/
    memset(p_framebuffer, 0xE0, 100 * 100 * 2);

    /*Assign framebuffer  and area to layer*/
    HAL_LCDC_LayerSetData(&hlcdc_rm69330, HAL_LCDC_LAYER_DEFAULT, (uint8_t *)p_framebuffer, 100, 100, 199, 199);


    /*Set clip area of all LCDC layer*/
    //HAL_LCDC_SetROIArea(&hlcdc_rm69330, 80, 80, 219, 319);
    HAL_LCDC_SetROIArea(&hlcdc_rm69330, 100, 100, 199, 199);

    /*Set LCD receive area */
    //RM69330_SetRegion(&hlcdc_rm69330, 80, 80, 219, 319);
    RM69330_SetRegion(&hlcdc_rm69330, 100, 100, 199, 199);

    /*Enable LCDC IRQ*/
    HAL_NVIC_SetPriority(LCDC_IRQ_NUM, 3, 0);
    HAL_NVIC_EnableIRQ(LCDC_IRQ_NUM);



    /*
        Send framebuffer after command 0x2c
    */
    HAL_LCDC_SendLayerData2Reg_IT(&hlcdc_rm69330, 0x2c, 1);

    /* Wait for 1 second */
    rt_thread_mdelay(1000);

    /*Writing framebuffer is done while LCDC_IRQHandler come*/

    /*Disable LCDC IRQ*/
    HAL_NVIC_DisableIRQ(LCDC_IRQ_NUM);


    rt_free(p_framebuffer);
}



UTEST_TC_EXPORT(testcase, "example_dsi", NULL, NULL, 10);



#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

