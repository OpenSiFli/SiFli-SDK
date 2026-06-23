/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include "bf0_hal.h"
#include "bf0_hal_lcdc_private.h"

#ifdef HAL_RAMLESS_LCD_ENABLED
#ifdef SF32LB57X
#define PTM_IRQ_NUM PTM1_CORE0_IRQn



void SPI_AUX_RST_HW_FSM(void)
{
    HAL_ASSERT(0); //To be implemented
}

void SPI_AUX_HW_FSM_START(LCDC_HandleTypeDef *lcdc)
{
    HAL_ASSERT(0); //To be implemented
}


void SPI_AUX_HW_FSM_STOP(LCDC_HandleTypeDef *lcdc)
{
    HAL_ASSERT(0); //To be implemented
}

HAL_StatusTypeDef RAMLESS_HW_FSM_READ_DATAS_START(LCDC_HandleTypeDef *lcdc, uint32_t freq, uint32_t addr, uint32_t addr_len, uint32_t data_len)
{
    HAL_ASSERT(0); //To be implemented
    return HAL_OK;
}

HAL_StatusTypeDef RAMLESS_HW_FSM_WRITE_DATAS_START(LCDC_HandleTypeDef *lcdc, uint32_t addr, uint32_t addr_len, uint8_t *p_data, uint32_t data_len)
{
    HAL_ASSERT(0); //To be implemented
    return HAL_OK;
}

HAL_StatusTypeDef RAMLESS_HW_FSM_READ_DATAS_END(LCDC_HandleTypeDef *lcdc, uint8_t *p_data, uint32_t data_len)
{
    HAL_ASSERT(0); //To be implemented
    return HAL_OK;
}

HAL_StatusTypeDef RAMLESS_HW_FSM_WRITE_DATAS_END(LCDC_HandleTypeDef *lcdc)
{
    HAL_ASSERT(0); //To be implemented
    return HAL_OK;
}

#ifdef LCDC_SUPPORT_DPI
__weak uint32_t BSP_GET_DPI_AUX_VSYNC_PIN(void)
{
    /* This function shouble be implemented in BSP layer */
    HAL_ASSERT(0);
    return 0;
}

void DPI_HW_FSM_START(LCDC_HandleTypeDef *lcdc)
{
    __IO uint32_t *p_VSYNC_PINMUX_REG = &(hwp_pinmux1->PAD_PA00) + BSP_GET_DPI_AUX_VSYNC_PIN();
    const uint32_t VSYNC_GPIO_ID  =  BSP_GET_DPI_AUX_VSYNC_PIN();

    uint32_t psram_data;
    uint32_t vsh0_hsw_cfg1;//Only Hsync cfg
    uint32_t vsh1_hsw_cfg1;//Vsync + Hsync cfg
    uint32_t vbp_vah_conf2, vbp_vah_conf3; //Only VBP & VAH
    uint32_t vah_conf2, vah_conf3; //Only VAH
    uint32_t vah_vfp_conf2, vah_vfp_conf3; //Only VAH & VFP
    uint32_t vsync_pinmux_cfg;
    uint32_t gpio_pinmux_cfg;
    uint32_t SRAM_BUF_LINES;

    uint32_t sram_buf_bytes, sram_buf_words;

    LCDC_LayerxTypeDef *pHwLayer0;
    LCDC_InitTypeDef *init;
    HAL_LCDC_ASSERT(lcdc);
    init = &lcdc->Init;


    uint32_t bytes_per_line;
    bytes_per_line = HAL_LCDC_GetPixelSize(lcdc->Layer[0].data_format) * init->cfg.dpi.HAW;
    HAL_ASSERT(0 != bytes_per_line);


    SRAM_BUF_LINES = lcdc->sram_buf_bytes / bytes_per_line;

    sram_buf_bytes = SRAM_BUF_LINES * bytes_per_line;
    sram_buf_words = (sram_buf_bytes / 4);

    //HW FSM repeat count(Part1+Part3 repeated once, Part2 repeated 'repeat_count-1' times)
    uint32_t repeat_count = init->cfg.dpi.VAH / (SRAM_BUF_LINES * 2);
    HAL_LCDC_ASSERT(0 == (init->cfg.dpi.VAH % (2 * SRAM_BUF_LINES)));

    {
        GPIO_TypeDef *gpio = hwp_gpio1;
        GPIO_InitTypeDef GPIO_InitStruct;

        // set vsync pin to output mode
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT;
        GPIO_InitStruct.Pin = VSYNC_GPIO_ID;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(gpio, &GPIO_InitStruct);

        // set vsync pin to idle
        HAL_GPIO_WritePin(gpio, VSYNC_GPIO_ID, (GPIO_PinState)init->cfg.dpi.VS_polarity);
    }

    //Bypass the line buffer
    lcdc->Instance->CANVAS_BG |= LCD_IF_CANVAS_BG_LB_BYPASS;
    pHwLayer0 = (LCDC_LayerxTypeDef *) & (lcdc->Instance->LAYER0_CONFIG);


    psram_data = pHwLayer0->SRC;
    pHwLayer0->SRC = (uint32_t) lcdc->sram_buf0;
    pHwLayer0->BR_POS = (pHwLayer0->BR_POS & LCD_IF_LAYER0_BR_POS_X1_Msk)
                        | ((pHwLayer0->TL_POS & LCD_IF_LAYER0_TL_POS_Y0_Msk) + ((SRAM_BUF_LINES - 1) << LCD_IF_LAYER0_BR_POS_Y1_Pos));

    lcdc->Instance->CANVAS_BR_POS = (lcdc->Instance->CANVAS_BR_POS & LCD_IF_CANVAS_BR_POS_X1_Msk)
                                    | ((lcdc->Instance->CANVAS_TL_POS & LCD_IF_CANVAS_TL_POS_Y0_Msk) + ((SRAM_BUF_LINES - 1) << LCD_IF_CANVAS_BR_POS_Y1_Pos));

    HAL_LCDC_ASSERT(pHwLayer0->BR_POS == lcdc->Instance->CANVAS_BR_POS);
    HAL_LCDC_ASSERT(pHwLayer0->TL_POS == lcdc->Instance->CANVAS_TL_POS);

    lcdc->Instance->DPI_IF_CONF4 = (SRAM_BUF_LINES << LCD_IF_DPI_IF_CONF4_VAH_Pos) |
                                   (init->cfg.dpi.HAW    << LCD_IF_DPI_IF_CONF4_HAW_Pos);


    vsh0_hsw_cfg1 = (0 << LCD_IF_DPI_IF_CONF1_VSH_Pos) |
                    (init->cfg.dpi.HS_width << LCD_IF_DPI_IF_CONF1_HSW_Pos);

    vsh1_hsw_cfg1 = (init->cfg.dpi.VS_width << LCD_IF_DPI_IF_CONF1_VSH_Pos) |
                    (init->cfg.dpi.HS_width << LCD_IF_DPI_IF_CONF1_HSW_Pos);



    vbp_vah_conf2 = (init->cfg.dpi.VBP << LCD_IF_DPI_IF_CONF2_VBP_Pos) | (init->cfg.dpi.HBP << LCD_IF_DPI_IF_CONF2_HBP_Pos);
    vah_conf2     = (0                 << LCD_IF_DPI_IF_CONF2_VBP_Pos) | (init->cfg.dpi.HBP << LCD_IF_DPI_IF_CONF2_HBP_Pos);
    vah_vfp_conf2 = (0                 << LCD_IF_DPI_IF_CONF2_VBP_Pos) | (init->cfg.dpi.HBP << LCD_IF_DPI_IF_CONF2_HBP_Pos);

    vbp_vah_conf3 = (0                 << LCD_IF_DPI_IF_CONF3_VFP_Pos) | (init->cfg.dpi.HFP << LCD_IF_DPI_IF_CONF3_HFP_Pos);
    vah_conf3     = (0                 << LCD_IF_DPI_IF_CONF3_VFP_Pos) | (init->cfg.dpi.HFP << LCD_IF_DPI_IF_CONF3_HFP_Pos);
    vah_vfp_conf3 = (init->cfg.dpi.VFP << LCD_IF_DPI_IF_CONF3_VFP_Pos) | (init->cfg.dpi.HFP << LCD_IF_DPI_IF_CONF3_HFP_Pos);


    vsync_pinmux_cfg = *p_VSYNC_PINMUX_REG;
    gpio_pinmux_cfg = (*p_VSYNC_PINMUX_REG) & ~HPSYS_PINMUX_PAD_PA00_FSEL_Msk;

    //PTC cycles per LCD Pixel Clock
    uint32_t Hclk_cycles_per_pclk = (lcdc->Instance->DPI_IF_CONF5 & LCD_IF_DPI_IF_CONF5_PCLK_DIV_Msk) >> LCD_IF_DPI_IF_CONF5_PCLK_DIV_Pos;


    uint32_t HFP_delay = 0 * (init->cfg.dpi.HS_width
                              + init->cfg.dpi.HBP
                              + init->cfg.dpi.HAW
                              + init->cfg.dpi.HFP) * Hclk_cycles_per_pclk;

    HAL_RCC_EnableModule(RCC_MOD_PTM1);

    HAL_RCC_ResetModule(RCC_MOD_PTM1);
    NVIC_EnableIRQ(PTM_IRQ_NUM);

    /*Dynamic allocation of DMA channels*/
    DMA_Channel_TypeDef *p_DMACH = NULL;
    uint32_t PTC_DMACH_TC;
    memset(&lcdc->hdma_handle, 0, sizeof(lcdc->hdma_handle));

    lcdc->hdma_handle.Instance = DMA1_Channel2;
    HAL_DMA_Init(&lcdc->hdma_handle);
    if (HAL_DMA_AllocChannel(&lcdc->hdma_handle) != HAL_OK)
    {
        HAL_LCDC_ASSERT(0); //DMA channel allocation failed
    }
    if (lcdc->hdma_handle.Instance != DMA1_Channel2 && lcdc->hdma_handle.Instance != DMA1_Channel1)
    {
        HAL_LCDC_ASSERT(0); //Only DMA1_Channel2 and DMA1_Channel1 have good performance on PSRAM read
    }


    p_DMACH = lcdc->hdma_handle.Instance;
    uint32_t channel_num = (lcdc->hdma_handle.ChannelIndex >> 2) + 1;
    PTC_DMACH_TC = PTC_HCPU_DMAC1_DONE1 + (channel_num - 1);
    /*DMA channel init end*/

    //Burst copy from CPAR to CM0AR
    p_DMACH->CCR = DMAC_CCR1_MEM2MEM
                   | (2 << DMAC_CCR1_PL_Pos)
                   | DMAC_CCR1_MINC | (0x2 << DMAC_CCR1_MSIZE_Pos)
                   | DMAC_CCR1_PINC | (0x2 << DMAC_CCR1_PSIZE_Pos)
                   | GPDMA_CCR1_DBURST | GPDMA_CCR1_SBURST
                   | DMAC_CCR1_EN;



    /*Force PCLK always on, to avoid LCD WDT timeout*/
    if (init->cfg.dpi.PCLK_force_on)
    {
        lcdc->Instance->DPI_IF_CONF5 |= LCD_IF_DPI_IF_CONF5_CLK_FORCE_ON;
        lcdc->Instance->SETTING  &= ~LCD_IF_SETTING_AUTO_GATE_EN_Msk;
    }


    int pclk1_div = HAL_RCC_GetHCLKFreq(CORE_ID_HCPU) / HAL_RCC_GetPCLKFreq(CORE_ID_HCPU, 1);

    // uint32_t VFP_delay_in_btim = ((init->cfg.dpi.VFP *
    //                                (init->cfg.dpi.HS_width
    //                                 + init->cfg.dpi.HBP
    //                                 + init->cfg.dpi.HAW
    //                                 + init->cfg.dpi.HFP)) + (pclk1_div >> 1)) / pclk1_div;


    // HAL_RCC_EnableModule(BTIM_RCC_MOD);
    // HAL_RCC_ResetModule(BTIM_RCC_MOD);
    // PTC_btim->CR1 |= BTIM_CR1_OPM | BTIM_CR1_URS;
    // PTC_btim->PSC = Hclk_cycles_per_pclk - 1;
    // PTC_btim->ARR = VFP_delay_in_btim;
    // PTC_btim->EGR |= BTIM_EGR_UG;


#define PTM_CODE_INIT() uint32_t ptm_pc = 0; HAL_LCDC_ASSERT(lcdc->ptc_code!=NULL);
#define PTM_CODE lcdc->ptc_code[ptm_pc++]
#define PTM_CODE_END() HAL_LCDC_ASSERT(ptm_pc <= RAMLESS_AUTO_REFR_CODE_SIZE_IN_WORD)

#define PTM_2_STM_MEM(dst, src)         PTM_LDM(PTM_X, (uint32_t) &(src)); PTM_CODE=PTM_STM(PTM_X, (uint32_t) &(dst))
#define PTM_2_SET_IMMIDIATE16(dst, val) PTM_SET(PTM_X,PTM_ZERO,OP_ADD,val); PTM_CODE=PTM_STM(PTM_X, (uint32_t) &(dst))
#define PTM_3_SET_IMMIDIATE32(dst, val) PTM_SET(PTM_XH,PTM_ZERO,OP_ADD,val>>16); PTM_CODE=PTM_SET(PTM_XL,PTM_ZERO,OP_ADD,val); PTM_CODE=PTM_STM(PTM_X, (uint32_t) &(dst))
#define PTM_2_CALL_DMA_WAIT()           PTM_LDM(PTM_X, (uint32_t) &(hwp_ptm1->PC0)); PTM_CODE = PTM_JMP(JMP_TCM3, PTM_ZERO, JMP_EQ, PTM_ZERO, 0)
#define PTM_2_CALL_LCDC_WAIT()          PTM_LDM(PTM_X, (uint32_t) &(hwp_ptm1->PC0)); PTM_CODE = PTM_JMP(JMP_TCM7, PTM_ZERO, JMP_EQ, PTM_ZERO, 0)
#define PTM_2_CALL_LCDC_WAIT2()         PTM_LDM(PTM_X, (uint32_t) &(hwp_ptm1->PC0)); PTM_CODE = PTM_JMP(JMP_TCM11, PTM_ZERO, JMP_EQ, PTM_ZERO, 0)

    PTM_CODE_INIT();

    ///////////////////////////////////
    // FRAME START   //
    ///////////////////////////////////
    ////Part1  VSYNC+VBP+VAH
    //////////////////////
    PTM_CODE = PTM_3_SET_IMMIDIATE32(*p_VSYNC_PINMUX_REG, vsync_pinmux_cfg); //Set the vsync pin to VSYNC mode
    PTM_CODE = PTM_3_SET_IMMIDIATE32(lcdc->Instance->DPI_IF_CONF1, vsh1_hsw_cfg1); //Enable VSYNC
    PTM_CODE = PTM_3_SET_IMMIDIATE32(lcdc->Instance->DPI_IF_CONF2, vbp_vah_conf2);
    PTM_CODE = PTM_3_SET_IMMIDIATE32(lcdc->Instance->DPI_IF_CONF3, vbp_vah_conf3);
    //DPI flush buffer 0
    PTM_CODE = PTM_2_STM_MEM(hwp_lcdc1->LAYER0_SRC, lcdc->sram_buf0); /* set lcdc src to buffer 0 */
    PTM_CODE = PTM_2_SET_IMMIDIATE16(hwp_lcdc1->DPI_CTRL, (1 << LCD_IF_DPI_CTRL_DPI_EN_Pos)); /* start lcdc */
    PTM_CODE = PTM_2_SET_IMMIDIATE16(hwp_lcdc1->DPI_CTRL, (0 << LCD_IF_DPI_CTRL_DPI_EN_Pos)); /* stop lcdc */

    /* Get the latest layer address and save it to Y register */
    PTM_CODE = PTM_LDM(PTM_Y, (uint32_t)&lcdc->Layer[HAL_LCDC_LAYER_DEFAULT].data);
    PTM_CODE = PTM_SET(PTM_EVTS, PTM_ZERO, OP_OR, 0X1); //Set EVT0
    //DMA copy Y to buffer 0, and Y+= sram_buf_bytes
    PTM_CODE = PTM_STM(PTM_Y, &(p_DMACH->CPAR)); /* set DMA src*/
    PTM_CODE = PTM_2_STM_MEM(p_DMACH->CM0AR, lcdc->sram_buf0); /* set DMA dst to buffer 1 */
    PTM_CODE = PTM_2_SET_IMMIDIATE16(p_DMACH->CNDTR, sram_buf_words); /* set DMA number and start dma */
    PTM_CODE = PTM_SET(PTM_Y, PTM_Y, OP_ADD, sram_buf_bytes); /* Y+= sram_buf_bytes */
    PTM_CODE = PTM_2_CALL_DMA_WAIT(); /* wait dma done */

    //DMA copy Y to buffer 1, and Y+= sram_buf_bytes
    PTM_CODE = PTM_STM(PTM_Y, &(p_DMACH->CPAR)); /* set DMA src*/
    PTM_CODE = PTM_2_STM_MEM(p_DMACH->CM0AR, lcdc->sram_buf1); /* set DMA dst to buffer 1 */
    PTM_CODE = PTM_2_SET_IMMIDIATE16(p_DMACH->CNDTR, sram_buf_words); /* set DMA number and start dma */
    PTM_CODE = PTM_SET(PTM_Y, PTM_Y, OP_ADD, sram_buf_bytes); /* Y+= sram_buf_bytes */
    PTM_CODE = PTM_2_CALL_DMA_WAIT(); /* wait dma done */
    PTM_CODE = PTM_2_CALL_LCDC_WAIT(); /* wait lcdc busy 0 */


    //////////////////////
    ///////Part2 VAH*N start {
    //////////////////////
    PTM_CODE = PTM_3_SET_IMMIDIATE32(*p_VSYNC_PINMUX_REG, gpio_pinmux_cfg); //Set the vsync pin to GPIO mode
    PTM_CODE = PTM_3_SET_IMMIDIATE32(lcdc->Instance->DPI_IF_CONF1, vsh0_hsw_cfg1); //Disable vsync
    PTM_CODE = PTM_3_SET_IMMIDIATE32(lcdc->Instance->DPI_IF_CONF2, vah_conf2);
    PTM_CODE = PTM_3_SET_IMMIDIATE32(lcdc->Instance->DPI_IF_CONF3, vah_conf3);

    /* for(B=repeat_count - 2; B!=0; B--) */
    PTM_CODE = PTM_SET(PTM_B, PTM_ZERO, OP_ADD, repeat_count - 2);
    {
        /** DMA copy Y to buffer 0  while DPI flush buffer 1 */
        //DPI flush buffer 1
        PTM_CODE = PTM_2_CALL_LCDC_WAIT(); /* wait lcdc busy 0 */
        PTM_CODE = PTM_2_STM_MEM(hwp_lcdc1->LAYER0_SRC, lcdc->sram_buf1); /* set lcdc src to buffer 1 */
        PTM_CODE = PTM_2_SET_IMMIDIATE16(hwp_lcdc1->DPI_CTRL, (1 << LCD_IF_DPI_CTRL_DPI_EN_Pos)); /* start lcdc */
        PTM_CODE = PTM_2_SET_IMMIDIATE16(hwp_lcdc1->DPI_CTRL, (0 << LCD_IF_DPI_CTRL_DPI_EN_Pos)); /* stop lcdc */

        //DMA copy y to buffer 0, and Y+= sram_buf_bytes
        PTM_CODE = PTM_STM(PTM_Y, &(p_DMACH->CPAR)); /* set DMA src*/
        PTM_CODE = PTM_2_STM_MEM(p_DMACH->CM0AR, lcdc->sram_buf0); /* set DMA dst to buffer 0 */
        PTM_CODE = PTM_2_SET_IMMIDIATE16(p_DMACH->CNDTR, sram_buf_words); /* set DMA number and start dma */
        PTM_CODE = PTM_SET(PTM_Y, PTM_Y, OP_ADD, sram_buf_bytes); /* Y+= sram_buf_bytes */
        PTM_CODE = PTM_2_CALL_DMA_WAIT(); /* wait dma done */



        /** DMA copy Yto buffer 1  while DPI flush buffer 0 */
        //DPI flush buffer 0
        PTM_CODE = PTM_2_CALL_LCDC_WAIT(); /* wait lcdc busy 0 */
        PTM_CODE = PTM_2_STM_MEM(hwp_lcdc1->LAYER0_SRC, lcdc->sram_buf0); /* set lcdc src to buffer 0 */
        PTM_CODE = PTM_2_SET_IMMIDIATE16(hwp_lcdc1->DPI_CTRL, (1 << LCD_IF_DPI_CTRL_DPI_EN_Pos)); /* start lcdc */
        PTM_CODE = PTM_2_SET_IMMIDIATE16(hwp_lcdc1->DPI_CTRL, (0 << LCD_IF_DPI_CTRL_DPI_EN_Pos)); /* stop lcdc */

        //DMA copy y to buffer 1, and Y+= sram_buf_bytes
        PTM_CODE = PTM_STM(PTM_Y, &(p_DMACH->CPAR)); /* set DMA src*/
        PTM_CODE = PTM_2_STM_MEM(p_DMACH->CM0AR, lcdc->sram_buf1); /* set DMA dst to buffer 1 */
        PTM_CODE = PTM_2_SET_IMMIDIATE16(p_DMACH->CNDTR, sram_buf_words); /* set DMA number and start dma */
        PTM_CODE = PTM_SET(PTM_Y, PTM_Y, OP_ADD, sram_buf_bytes); /* Y+= sram_buf_bytes */
        PTM_CODE = PTM_2_CALL_DMA_WAIT(); /* wait dma done */


        PTM_CODE = PTM_JMP(-32, PTM_B, JMP_NEQDEC, PTM_ZERO, 0); /* loop then B-=1  */
    }
    ////}VAH loop
    //////////////////////


    //////////////////////
    /////Part3 VAH+VFP
    //////////////////////
    PTM_CODE = PTM_2_CALL_LCDC_WAIT(); /* wait lcdc busy 0 */
    PTM_CODE = PTM_3_SET_IMMIDIATE32(lcdc->Instance->DPI_IF_CONF2, vah_vfp_conf2);
    PTM_CODE = PTM_3_SET_IMMIDIATE32(lcdc->Instance->DPI_IF_CONF3, vah_vfp_conf3);
    //DPI flush buffer 0
    PTM_CODE = PTM_2_STM_MEM(hwp_lcdc1->LAYER0_SRC, lcdc->sram_buf1); /* set lcdc src to buffer 1 */
    PTM_CODE = PTM_2_SET_IMMIDIATE16(hwp_lcdc1->DPI_CTRL, (1 << LCD_IF_DPI_CTRL_DPI_EN_Pos)); /* start lcdc */
    PTM_CODE = PTM_2_SET_IMMIDIATE16(hwp_lcdc1->DPI_CTRL, (0 << LCD_IF_DPI_CTRL_DPI_EN_Pos)); /* stop lcdc */
    PTM_CODE = PTM_2_CALL_LCDC_WAIT2(); /* wait lcdc busy 0 */
    PTM_CODE = PTM_JMP(JMP_TCM0, PTM_ZERO, JMP_EQ, PTM_ZERO, 0); /* Jump to TCM0 */
    PTM_CODE_END();




    uint32_t core0_tcm_code[16];
    core0_tcm_code[0] = PTM_SET(PTM_XL, PTM_ZERO, OP_ADD, (uint32_t)lcdc->ptc_code);
    core0_tcm_code[1] = PTM_SET(PTM_XH, PTM_ZERO, OP_ADD, (uint32_t)lcdc->ptc_code >> 16);
    core0_tcm_code[2] = PTM_JMP(JMP_X, PTM_ZERO, JMP_EQ, PTM_ZERO, 0);
    core0_tcm_code[3] = PTM_WAIT(PTC_DMACH_TC, 1, 0);
    core0_tcm_code[4] = PTM_SET(PTM_X, PTM_X, OP_ADD, 8);
    core0_tcm_code[5] = PTM_JMP(JMP_X, PTM_ZERO, JMP_EQ, PTM_ZERO, 0);
    core0_tcm_code[6] = 0;
    core0_tcm_code[7] = PTM_WAIT(PTC_HCPU_LCDC1_BUSY, 0, 0);
    core0_tcm_code[8] = PTM_SET(PTM_X, PTM_X, OP_ADD, 8);
    core0_tcm_code[9] = PTM_JMP(JMP_X, PTM_ZERO, JMP_EQ, PTM_ZERO, 0);
    core0_tcm_code[10] = 0;
    core0_tcm_code[11] = PTM_WAIT(PTC_HCPU_LCDC1_BUSY, 1, 0);
    core0_tcm_code[12] = PTM_WAIT(PTC_HCPU_LCDC1_BUSY, 0, 0);
    core0_tcm_code[13] = PTM_SET(PTM_X, PTM_X, OP_ADD, 8);
    core0_tcm_code[14] = PTM_JMP(JMP_X, PTM_ZERO, JMP_EQ, PTM_ZERO, 0);
    core0_tcm_code[15] = 0;


    memcpy(PTM1_CORE0_TCM, core0_tcm_code, sizeof(core0_tcm_code));
    hwp_ptm1->CER = PTM_CER_RST0;
    hwp_ptm1->CCR0 = (0 << PTM_CCR0_END0_Pos) |
                     (0 << PTM_CCR0_REP_Pos) ; //infinite repetition

    //Start!!!
    hwp_ptm1->CER = PTM_CER_EN0;

}

void DPI_HW_FSM_STOP(LCDC_HandleTypeDef *lcdc)
{

    //Stop it at end of one frame
    // hwp_ptc1->MEM3 = RAMLESS_STOP_MAGIC_FLAG;
    // uint32_t *p_uint32_t = PTC_CH_DATA_CODE_ADDR(8, 8);
    // *p_uint32_t = (uint32_t) PTC_PHASE_ADDR(9);//Start it!

    // HAL_LCDC_ASSERT(HAL_OK == WAIT_EXECUTE_CODE_DONE(lcdc));
    hwp_ptm1->CER = PTM_CER_RST0;

    NVIC_DisableIRQ(PTM_IRQ_NUM);

    //Free dma channel
    HAL_DMA_FreeChannel(&lcdc->hdma_handle);
}

void DPI_HW_FSM_UPDATE_LAYER_DATA(LCDC_HandleTypeDef *lcdc)
{
    //Enable PTC interrupt
    hwp_ptm1->IER |= PTM_IER_EVTIE0;
}

HAL_StatusTypeDef DPI_HW_FSM_UPDATE_LAYER_DATA_DONE(LCDC_HandleTypeDef *lcdc)
{
    uint32_t start, end;
    LCDC_LayerCfgTypeDef *cfg = &lcdc->Layer[HAL_LCDC_LAYER_DEFAULT];
    uint32_t bytes_per_pixel, data_w, data_h;

    data_w = cfg->data_area.x1 - cfg->data_area.x0 + 1;
    data_h = cfg->data_area.y1 - cfg->data_area.y0 + 1;
    bytes_per_pixel = HAL_LCDC_GetPixelSize(cfg->data_format);

    start = (uint32_t)lcdc->Layer[HAL_LCDC_LAYER_DEFAULT].data;
    end   = start + (data_w * data_h * bytes_per_pixel);

    //Y0 indicates the PSRAM address where the LCDC is reading
    if ((hwp_ptm1->Y0 >= start) && (hwp_ptm1->Y0 < end))
    {
        //Disable PTM interrupt
        hwp_ptm1->IER &= ~PTM_IER_EVTIE0;
        return HAL_OK;
    }
    return HAL_BUSY;
}
#endif /* LCDC_SUPPORT_DPI */
#endif /* SF32LB57X */
#endif /* HAL_RAMLESS_LCD_ENABLED */