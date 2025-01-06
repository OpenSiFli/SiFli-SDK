#ifndef _BF0_PIN_CONST_H
#define _BF0_PIN_CONST_H
/** @addtogroup PINMUX
 * @{
 */

/** pin function */
typedef enum
{
    PIN_FUNC_UNDEF,
    /** PSRAM_CLK */
    PSRAM_CLK,
    /** PSRAM_CLKB */
    PSRAM_CLKB,
    /** PSRAM_CS */
    PSRAM_CS,
    /** PSRAM_DM0 */
    PSRAM_DM0,
    /** PSRAM_DM1 */
    PSRAM_DM1,
    /** PSRAM_DQS0 */
    PSRAM_DQS0,
    /** PSRAM_DQS1 */
    PSRAM_DQS1,
    /** PSRAM_DQ0 */
    PSRAM_DQ0,
    /** PSRAM_DQ1 */
    PSRAM_DQ1,
    /** PSRAM_DQ2 */
    PSRAM_DQ2,
    /** PSRAM_DQ3 */
    PSRAM_DQ3,
    /** PSRAM_DQ4 */
    PSRAM_DQ4,
    /** PSRAM_DQ5 */
    PSRAM_DQ5,
    /** PSRAM_DQ6 */
    PSRAM_DQ6,
    /** PSRAM_DQ7 */
    PSRAM_DQ7,
    /** PSRAM_DQ8 */
    PSRAM_DQ8,
    /** PSRAM_DQ9 */
    PSRAM_DQ9,
    /** PSRAM_DQ10 */
    PSRAM_DQ10,
    /** PSRAM_DQ11 */
    PSRAM_DQ11,
    /** PSRAM_DQ12 */
    PSRAM_DQ12,
    /** PSRAM_DQ13 */
    PSRAM_DQ13,
    /** PSRAM_DQ14 */
    PSRAM_DQ14,
    /** PSRAM_DQ15 */
    PSRAM_DQ15,
    /** QSPI1_CLK */
    QSPI1_CLK,
    /** QSPI1_CS */
    QSPI1_CS,
    /** QSPI1_DIO0 */
    QSPI1_DIO0,
    /** QSPI1_DIO1 */
    QSPI1_DIO1,
    /** QSPI1_DIO2 */
    QSPI1_DIO2,
    /** QSPI1_DIO3 */
    QSPI1_DIO3,
    /** QSPI1_DIO4 */
    QSPI1_DIO4,
    /** QSPI1_DIO5 */
    QSPI1_DIO5,
    /** QSPI1_DIO6 */
    QSPI1_DIO6,
    /** QSPI1_DIO7 */
    QSPI1_DIO7,
    /** QSPI2_CLK */
    QSPI2_CLK,
    /** QSPI2_CS */
    QSPI2_CS,
    /** QSPI2_DIO0 */
    QSPI2_DIO0,
    /** QSPI2_DIO1 */
    QSPI2_DIO1,
    /** QSPI2_DIO2 */
    QSPI2_DIO2,
    /** QSPI2_DIO3 */
    QSPI2_DIO3,
    /** QSPI2_DIO4 */
    QSPI2_DIO4,
    /** QSPI2_DIO5 */
    QSPI2_DIO5,
    /** QSPI2_DIO6 */
    QSPI2_DIO6,
    /** QSPI2_DIO7 */
    QSPI2_DIO7,
    /** QSPI3_CLK */
    QSPI3_CLK,
    /** QSPI3_CS */
    QSPI3_CS,
    /** QSPI3_DIO0 */
    QSPI3_DIO0,
    /** QSPI3_DIO1 */
    QSPI3_DIO1,
    /** QSPI3_DIO2 */
    QSPI3_DIO2,
    /** QSPI3_DIO3 */
    QSPI3_DIO3,
    /** QSPI3_DIO4 */
    QSPI3_DIO4,
    /** QSPI3_DIO5 */
    QSPI3_DIO5,
    /** QSPI3_DIO6 */
    QSPI3_DIO6,
    /** QSPI3_DIO7 */
    QSPI3_DIO7,
    /** QSPI4_CLK */
    QSPI4_CLK,
    /** QSPI4_CS */
    QSPI4_CS,
    /** QSPI4_DIO0 */
    QSPI4_DIO0,
    /** QSPI4_DIO1 */
    QSPI4_DIO1,
    /** QSPI4_DIO2 */
    QSPI4_DIO2,
    /** QSPI4_DIO3 */
    QSPI4_DIO3,
    /** QSPI4_DIO4 */
    QSPI4_DIO4,
    /** QSPI4_DIO5 */
    QSPI4_DIO5,
    /** QSPI4_DIO6 */
    QSPI4_DIO6,
    /** QSPI4_DIO7 */
    QSPI4_DIO7,
    /** GPIO_A0 */
    GPIO_A0,
    /** GPIO_A1 */
    GPIO_A1,
    /** GPIO_A2 */
    GPIO_A2,
    /** GPIO_A3 */
    GPIO_A3,
    /** GPIO_A4 */
    GPIO_A4,
    /** GPIO_A5 */
    GPIO_A5,
    /** GPIO_A6 */
    GPIO_A6,
    /** GPIO_A7 */
    GPIO_A7,
    /** GPIO_A8 */
    GPIO_A8,
    /** GPIO_A9 */
    GPIO_A9,
    /** GPIO_A10 */
    GPIO_A10,
    /** GPIO_A11 */
    GPIO_A11,
    /** GPIO_A12 */
    GPIO_A12,
    /** GPIO_A13 */
    GPIO_A13,
    /** GPIO_A14 */
    GPIO_A14,
    /** GPIO_A15 */
    GPIO_A15,
    /** GPIO_A16 */
    GPIO_A16,
    /** GPIO_A17 */
    GPIO_A17,
    /** GPIO_A18 */
    GPIO_A18,
    /** GPIO_A19 */
    GPIO_A19,
    /** GPIO_A20 */
    GPIO_A20,
    /** GPIO_A21 */
    GPIO_A21,
    /** GPIO_A22 */
    GPIO_A22,
    /** GPIO_A23 */
    GPIO_A23,
    /** GPIO_A24 */
    GPIO_A24,
    /** GPIO_A25 */
    GPIO_A25,
    /** GPIO_A26 */
    GPIO_A26,
    /** GPIO_A27 */
    GPIO_A27,
    /** GPIO_A28 */
    GPIO_A28,
    /** GPIO_A29 */
    GPIO_A29,
    /** GPIO_A30 */
    GPIO_A30,
    /** GPIO_A31 */
    GPIO_A31,
    /** GPIO_A32 */
    GPIO_A32,
    /** GPIO_A33 */
    GPIO_A33,
    /** GPIO_A34 */
    GPIO_A34,
    /** GPIO_A35 */
    GPIO_A35,
    /** GPIO_A36 */
    GPIO_A36,
    /** GPIO_A37 */
    GPIO_A37,
    /** GPIO_A38 */
    GPIO_A38,
    /** GPIO_A39 */
    GPIO_A39,
    /** GPIO_A40 */
    GPIO_A40,
    /** GPIO_A41 */
    GPIO_A41,
    /** GPIO_A42 */
    GPIO_A42,
    /** GPIO_A43 */
    GPIO_A43,
    /** GPIO_A44 */
    GPIO_A44,
    /** GPIO_A45 */
    GPIO_A45,
    /** GPIO_A46 */
    GPIO_A46,
    /** GPIO_A47 */
    GPIO_A47,
    /** GPIO_A48 */
    GPIO_A48,
    /** GPIO_A49 */
    GPIO_A49,
    /** GPIO_A50 */
    GPIO_A50,
    /** GPIO_A51 */
    GPIO_A51,
    /** GPIO_A52 */
    GPIO_A52,
    /** GPIO_A53 */
    GPIO_A53,
    /** GPIO_A54 */
    GPIO_A54,
    /** GPIO_A55 */
    GPIO_A55,
    /** GPIO_A56 */
    GPIO_A56,
    /** GPIO_A57 */
    GPIO_A57,
    /** GPIO_A58 */
    GPIO_A58,
    /** GPIO_A59 */
    GPIO_A59,
    /** GPIO_A60 */
    GPIO_A60,
    /** GPIO_A61 */
    GPIO_A61,
    /** GPIO_A62 */
    GPIO_A62,
    /** GPIO_A63 */
    GPIO_A63,
    /** GPIO_A64 */
    GPIO_A64,
    /** GPIO_A65 */
    GPIO_A65,
    /** GPIO_A66 */
    GPIO_A66,
    /** GPIO_A67 */
    GPIO_A67,
    /** GPIO_A68 */
    GPIO_A68,
    /** GPIO_A69 */
    GPIO_A69,
    /** GPIO_A70 */
    GPIO_A70,
    /** GPIO_A71 */
    GPIO_A71,
    /** GPIO_A72 */
    GPIO_A72,
    /** GPIO_A73 */
    GPIO_A73,
    /** GPIO_A74 */
    GPIO_A74,
    /** GPIO_A75 */
    GPIO_A75,
    /** GPIO_A76 */
    GPIO_A76,
    /** GPIO_A77 */
    GPIO_A77,
    /** GPIO_A78 */
    GPIO_A78,
    /** GPIO_A79 */
    GPIO_A79,
    /** GPIO_A80 */
    GPIO_A80,
    /** GPIO_A81 */
    GPIO_A81,
    /** GPIO_A82 */
    GPIO_A82,
    /** GPIO_A83 */
    GPIO_A83,
    /** GPIO_A84 */
    GPIO_A84,
    /** GPIO_A85 */
    GPIO_A85,
    /** GPIO_A86 */
    GPIO_A86,
    /** GPIO_A87 */
    GPIO_A87,
    /** GPIO_B0 */
    GPIO_B0,
    /** GPIO_B1 */
    GPIO_B1,
    /** GPIO_B2 */
    GPIO_B2,
    /** GPIO_B3 */
    GPIO_B3,
    /** GPIO_B4 */
    GPIO_B4,
    /** GPIO_B5 */
    GPIO_B5,
    /** GPIO_B6 */
    GPIO_B6,
    /** GPIO_B7 */
    GPIO_B7,
    /** GPIO_B8 */
    GPIO_B8,
    /** GPIO_B9 */
    GPIO_B9,
    /** GPIO_B10 */
    GPIO_B10,
    /** GPIO_B11 */
    GPIO_B11,
    /** GPIO_B12 */
    GPIO_B12,
    /** GPIO_B13 */
    GPIO_B13,
    /** GPIO_B14 */
    GPIO_B14,
    /** GPIO_B15 */
    GPIO_B15,
    /** GPIO_B16 */
    GPIO_B16,
    /** GPIO_B17 */
    GPIO_B17,
    /** GPIO_B18 */
    GPIO_B18,
    /** GPIO_B19 */
    GPIO_B19,
    /** GPIO_B20 */
    GPIO_B20,
    /** GPIO_B21 */
    GPIO_B21,
    /** GPIO_B22 */
    GPIO_B22,
    /** GPIO_B23 */
    GPIO_B23,
    /** GPIO_B24 */
    GPIO_B24,
    /** GPIO_B25 */
    GPIO_B25,
    /** GPIO_B26 */
    GPIO_B26,
    /** GPIO_B27 */
    GPIO_B27,
    /** GPIO_B28 */
    GPIO_B28,
    /** GPIO_B29 */
    GPIO_B29,
    /** GPIO_B30 */
    GPIO_B30,
    /** GPIO_B31 */
    GPIO_B31,
    /** GPIO_B32 */
    GPIO_B32,
    /** GPIO_B33 */
    GPIO_B33,
    /** GPIO_B34 */
    GPIO_B34,
    /** GPIO_B35 */
    GPIO_B35,
    /** GPIO_B36 */
    GPIO_B36,
    /** GPIO_B37 */
    GPIO_B37,
    /** GPIO_B38 */
    GPIO_B38,
    /** GPIO_B39 */
    GPIO_B39,
    /** GPIO_B40 */
    GPIO_B40,
    /** GPIO_B41 */
    GPIO_B41,
    /** GPIO_B42 */
    GPIO_B42,
    /** GPIO_B43 */
    GPIO_B43,
    /** GPIO_B44 */
    GPIO_B44,
    /** GPIO_B45 */
    GPIO_B45,
    /** GPIO_B46 */
    GPIO_B46,
    /** GPIO_B47 */
    GPIO_B47,
    /** GPIO_B48 */
    GPIO_B48,
    /** GPIO_B49 */
    GPIO_B49,
    /** GPIO_B50 */
    GPIO_B50,
    /** GPIO_B51 */
    GPIO_B51,
    /** GPIO_B52 */
    GPIO_B52,
    /** GPIO_B53 */
    GPIO_B53,
    /** GPIO_B54 */
    GPIO_B54,
    /** GPIO_B55 */
    GPIO_B55,
    /** GPIO_B56 */
    GPIO_B56,
    /** GPIO_B57 */
    GPIO_B57,
    /** GPIO_B58 */
    GPIO_B58,
    /** GPIO_B59 */
    GPIO_B59,
    /** GPIO_B60 */
    GPIO_B60,
    /** GPIO_B61 */
    GPIO_B61,
    /** GPIO_B62 */
    GPIO_B62,
    /** GPIO_B63 */
    GPIO_B63,
    /** USART1_RXD */
    USART1_RXD,
    /** USART1_TXD */
    USART1_TXD,
    /** USART1_CTS */
    USART1_CTS,
    /** USART1_RTS */
    USART1_RTS,
    /** USART2_RXD */
    USART2_RXD,
    /** USART2_TXD */
    USART2_TXD,
    /** USART2_CTS */
    USART2_CTS,
    /** USART2_RTS */
    USART2_RTS,
    /** USART3_RXD */
    USART3_RXD,
    /** USART3_TXD */
    USART3_TXD,
    /** USART3_CTS */
    USART3_CTS,
    /** USART3_RTS */
    USART3_RTS,
    /** USART4_RXD */
    USART4_RXD,
    /** USART4_TXD */
    USART4_TXD,
    /** USART4_CTS */
    USART4_CTS,
    /** USART4_RTS */
    USART4_RTS,
    /** USART5_RXD */
    USART5_RXD,
    /** USART5_TXD */
    USART5_TXD,
    /** USART5_CTS */
    USART5_CTS,
    /** USART5_RTS */
    USART5_RTS,
    /** I2C1_SCL */
    I2C1_SCL,
    /** I2C1_SDA */
    I2C1_SDA,
    /** I2C2_SCL */
    I2C2_SCL,
    /** I2C2_SDA */
    I2C2_SDA,
    /** I2C3_SCL */
    I2C3_SCL,
    /** I2C3_SDA */
    I2C3_SDA,
    /** I2C4_SCL */
    I2C4_SCL,
    /** I2C4_SDA */
    I2C4_SDA,
    /** I2C5_SCL */
    I2C5_SCL,
    /** I2C5_SDA */
    I2C5_SDA,
    /** I2C6_SCL */
    I2C6_SCL,
    /** I2C6_SDA */
    I2C6_SDA,
    /** SPI1_CLK */
    SPI1_CLK,
    /** SPI1_CS */
    SPI1_CS,
    /** SPI1_DI */
    SPI1_DI,
    /** SPI1_DO */
    SPI1_DO,
    /** SPI1_DIO */
    SPI1_DIO,
    /** SPI2_CLK */
    SPI2_CLK,
    /** SPI2_CS */
    SPI2_CS,
    /** SPI2_DI */
    SPI2_DI,
    /** SPI2_DO */
    SPI2_DO,
    /** SPI2_DIO */
    SPI2_DIO,
    /** SPI3_CLK */
    SPI3_CLK,
    /** SPI3_CS */
    SPI3_CS,
    /** SPI3_DI */
    SPI3_DI,
    /** SPI3_DO */
    SPI3_DO,
    /** SPI3_DIO */
    SPI3_DIO,
    /** SPI4_CLK */
    SPI4_CLK,
    /** SPI4_CS */
    SPI4_CS,
    /** SPI4_DI */
    SPI4_DI,
    /** SPI4_DO */
    SPI4_DO,
    /** SPI4_DIO */
    SPI4_DIO,
    /** SPI5_CLK */
    SPI5_CLK,
    /** SPI5_CS */
    SPI5_CS,
    /** SPI5_DI */
    SPI5_DI,
    /** SPI5_DO */
    SPI5_DO,
    /** SPI5_DIO */
    SPI5_DIO,
    /** LCDC1_SPI_CS */
    LCDC1_SPI_CS,
    /** LCDC1_SPI_CLK */
    LCDC1_SPI_CLK,
    /** LCDC1_SPI_DIO0 */
    LCDC1_SPI_DIO0,
    /** LCDC1_SPI_DIO1 */
    LCDC1_SPI_DIO1,
    /** LCDC1_SPI_DIO2 */
    LCDC1_SPI_DIO2,
    /** LCDC1_SPI_DIO3 */
    LCDC1_SPI_DIO3,
    /** LCDC1_SPI_RSTB */
    LCDC1_SPI_RSTB,
    /** LCDC1_SPI_TE */
    LCDC1_SPI_TE,
    /** LCDC2_SPI_CS */
    LCDC2_SPI_CS,
    /** LCDC2_SPI_CLK */
    LCDC2_SPI_CLK,
    /** LCDC2_SPI_DIO0 */
    LCDC2_SPI_DIO0,
    /** LCDC2_SPI_DIO1 */
    LCDC2_SPI_DIO1,
    /** LCDC2_SPI_DIO2 */
    LCDC2_SPI_DIO2,
    /** LCDC2_SPI_DIO3 */
    LCDC2_SPI_DIO3,
    /** LCDC2_SPI_RSTB */
    LCDC2_SPI_RSTB,
    /** LCDC2_SPI_TE */
    LCDC2_SPI_TE,
    /** LCDC1_8080_WR */
    LCDC1_8080_WR,
    /** LCDC1_8080_CS */
    LCDC1_8080_CS,
    /** LCDC1_8080_DIO0 */
    LCDC1_8080_DIO0,
    /** LCDC1_8080_DIO1 */
    LCDC1_8080_DIO1,
    /** LCDC1_8080_DIO2 */
    LCDC1_8080_DIO2,
    /** LCDC1_8080_DIO3 */
    LCDC1_8080_DIO3,
    /** LCDC1_8080_DIO4 */
    LCDC1_8080_DIO4,
    /** LCDC1_8080_DIO5 */
    LCDC1_8080_DIO5,
    /** LCDC1_8080_DIO6 */
    LCDC1_8080_DIO6,
    /** LCDC1_8080_DIO7 */
    LCDC1_8080_DIO7,
    /** LCDC1_8080_RD */
    LCDC1_8080_RD,
    /** LCDC1_8080_DC */
    LCDC1_8080_DC,
    /** LCDC1_8080_RSTB */
    LCDC1_8080_RSTB,
    /** LCDC1_8080_TE */
    LCDC1_8080_TE,
    /** LCDC1_DPI_CLK */
    LCDC1_DPI_CLK,
    /** LCDC1_DPI_DE */
    LCDC1_DPI_DE,
    /** LCDC1_DPI_SD */
    LCDC1_DPI_SD,
    /** LCDC1_DPI_CM */
    LCDC1_DPI_CM,
    /** LCDC1_DPI_HSYNC */
    LCDC1_DPI_HSYNC,
    /** LCDC1_DPI_VSYNC */
    LCDC1_DPI_VSYNC,
    /** LCDC1_DPI_R7 */
    LCDC1_DPI_R7,
    /** LCDC1_DPI_R6 */
    LCDC1_DPI_R6,
    /** LCDC1_DPI_R5 */
    LCDC1_DPI_R5,
    /** LCDC1_DPI_R4 */
    LCDC1_DPI_R4,
    /** LCDC1_DPI_R3 */
    LCDC1_DPI_R3,
    /** LCDC1_DPI_R2 */
    LCDC1_DPI_R2,
    /** LCDC1_DPI_R1 */
    LCDC1_DPI_R1,
    /** LCDC1_DPI_R0 */
    LCDC1_DPI_R0,
    /** LCDC1_DPI_G7 */
    LCDC1_DPI_G7,
    /** LCDC1_DPI_G6 */
    LCDC1_DPI_G6,
    /** LCDC1_DPI_G5 */
    LCDC1_DPI_G5,
    /** LCDC1_DPI_G4 */
    LCDC1_DPI_G4,
    /** LCDC1_DPI_G3 */
    LCDC1_DPI_G3,
    /** LCDC1_DPI_G2 */
    LCDC1_DPI_G2,
    /** LCDC1_DPI_G1 */
    LCDC1_DPI_G1,
    /** LCDC1_DPI_G0 */
    LCDC1_DPI_G0,
    /** LCDC1_DPI_B7 */
    LCDC1_DPI_B7,
    /** LCDC1_DPI_B6 */
    LCDC1_DPI_B6,
    /** LCDC1_DPI_B5 */
    LCDC1_DPI_B5,
    /** LCDC1_DPI_B4 */
    LCDC1_DPI_B4,
    /** LCDC1_DPI_B3 */
    LCDC1_DPI_B3,
    /** LCDC1_DPI_B2 */
    LCDC1_DPI_B2,
    /** LCDC1_DPI_B1 */
    LCDC1_DPI_B1,
    /** LCDC1_DPI_B0 */
    LCDC1_DPI_B0,
    /** LCDC1_JDI_SCLK */
    LCDC1_JDI_SCLK,
    /** LCDC1_JDI_SCS */
    LCDC1_JDI_SCS,
    /** LCDC1_JDI_SO */
    LCDC1_JDI_SO,
    /** LCDC1_JDI_DISP */
    LCDC1_JDI_DISP,
    /** LCDC1_JDI_EXTCOMIN */
    LCDC1_JDI_EXTCOMIN,
    /** LCDC1_JDI_XRST */
    LCDC1_JDI_XRST,
    /** LCDC1_JDI_VCK */
    LCDC1_JDI_VCK,
    /** LCDC1_JDI_VST */
    LCDC1_JDI_VST,
    /** LCDC1_JDI_ENB */
    LCDC1_JDI_ENB,
    /** LCDC1_JDI_HCK */
    LCDC1_JDI_HCK,
    /** LCDC1_JDI_HST */
    LCDC1_JDI_HST,
    /** LCDC1_JDI_R1 */
    LCDC1_JDI_R1,
    /** LCDC1_JDI_R2 */
    LCDC1_JDI_R2,
    /** LCDC1_JDI_G1 */
    LCDC1_JDI_G1,
    /** LCDC1_JDI_G2 */
    LCDC1_JDI_G2,
    /** LCDC1_JDI_B1 */
    LCDC1_JDI_B1,
    /** LCDC1_JDI_B2 */
    LCDC1_JDI_B2,
    /** LCDC1_JDI_FRP */
    LCDC1_JDI_FRP,
    /** LCDC1_JDI_XFRP */
    LCDC1_JDI_XFRP,
    /** LCDC1_JDI_VCOM */
    LCDC1_JDI_VCOM,
    /** SD1_CLK */
    SD1_CLK,
    /** SD1_CMD */
    SD1_CMD,
    /** SD1_DIO0 */
    SD1_DIO0,
    /** SD1_DIO1 */
    SD1_DIO1,
    /** SD1_DIO2 */
    SD1_DIO2,
    /** SD1_DIO3 */
    SD1_DIO3,
    /** SD1_DIO4 */
    SD1_DIO4,
    /** SD1_DIO5 */
    SD1_DIO5,
    /** SD1_DIO6 */
    SD1_DIO6,
    /** SD1_DIO7 */
    SD1_DIO7,
    /** SD2_CLK */
    SD2_CLK,
    /** SD2_CMD */
    SD2_CMD,
    /** SD2_DIO0 */
    SD2_DIO0,
    /** SD2_DIO1 */
    SD2_DIO1,
    /** SD2_DIO2 */
    SD2_DIO2,
    /** SD2_DIO3 */
    SD2_DIO3,
    /** I2S1_BCK */
    I2S1_BCK,
    /** I2S1_LRCK */
    I2S1_LRCK,
    /** I2S1_SDI */
    I2S1_SDI,
    /** I2S2_BCK */
    I2S2_BCK,
    /** I2S2_LRCK */
    I2S2_LRCK,
    /** I2S2_SDI */
    I2S2_SDI,
    /** I2S2_SDO */
    I2S2_SDO,
    /** PDM1_CLK */
    PDM1_CLK,
    /** PDM1_DATA */
    PDM1_DATA,
    /** PDM2_CLK */
    PDM2_CLK,
    /** PDM2_DATA */
    PDM2_DATA,
    /** GPTIM1_CH1 */
    GPTIM1_CH1,
    /** GPTIM1_CH2 */
    GPTIM1_CH2,
    /** GPTIM1_CH3 */
    GPTIM1_CH3,
    /** GPTIM1_CH4 */
    GPTIM1_CH4,
    /** GPTIM1_ETR */
    GPTIM1_ETR,
    /** GPTIM2_CH1 */
    GPTIM2_CH1,
    /** GPTIM2_CH2 */
    GPTIM2_CH2,
    /** GPTIM2_CH3 */
    GPTIM2_CH3,
    /** GPTIM2_CH4 */
    GPTIM2_CH4,
    /** GPTIM2_ETR */
    GPTIM2_ETR,
    /** GPTIM3_CH1 */
    GPTIM3_CH1,
    /** GPTIM3_CH2 */
    GPTIM3_CH2,
    /** GPTIM3_CH3 */
    GPTIM3_CH3,
    /** GPTIM3_CH4 */
    GPTIM3_CH4,
    /** GPTIM3_ETR */
    GPTIM3_ETR,
    /** GPTIM4_CH1 */
    GPTIM4_CH1,
    /** GPTIM4_CH2 */
    GPTIM4_CH2,
    /** GPTIM4_CH3 */
    GPTIM4_CH3,
    /** GPTIM4_CH4 */
    GPTIM4_CH4,
    /** GPTIM4_ETR */
    GPTIM4_ETR,
    /** GPTIM5_CH1 */
    GPTIM5_CH1,
    /** GPTIM5_CH2 */
    GPTIM5_CH2,
    /** GPTIM5_CH3 */
    GPTIM5_CH3,
    /** GPTIM5_CH4 */
    GPTIM5_CH4,
    /** GPTIM5_ETR */
    GPTIM5_ETR,
    /** IR_OUT */
    IR_OUT,
    /** LPTIM1_IN */
    LPTIM1_IN,
    /** LPTIM1_OUT */
    LPTIM1_OUT,
    /** LPTIM1_ETR */
    LPTIM1_ETR,
    /** LPTIM2_IN */
    LPTIM2_IN,
    /** LPTIM2_OUT */
    LPTIM2_OUT,
    /** LPTIM2_ETR */
    LPTIM2_ETR,
    /** LPTIM3_IN */
    LPTIM3_IN,
    /** LPTIM3_OUT */
    LPTIM3_OUT,
    /** LPTIM3_ETR */
    LPTIM3_ETR,
    /** LPCOMP1_OUT */
    LPCOMP1_OUT,
    /** LPCOMP2_OUT */
    LPCOMP2_OUT,
    /** SWCLK */
    SWCLK,
    /** SWDIO */
    SWDIO,
    /** SCAN_CLK */
    SCAN_CLK,
    /** SCAN_RSTB */
    SCAN_RSTB,
    /** SCAN_EN */
    SCAN_EN,
    /** EDT_CLK */
    EDT_CLK,
    /** EDT_UPDATE */
    EDT_UPDATE,
    /** EDT_BYPASS */
    EDT_BYPASS,
    /** EDT_CHANNEL_IN0 */
    EDT_CHANNEL_IN0,
    /** EDT_CHANNEL_IN1 */
    EDT_CHANNEL_IN1,
    /** EDT_CHANNEL_IN2 */
    EDT_CHANNEL_IN2,
    /** EDT_CHANNEL_IN3 */
    EDT_CHANNEL_IN3,
    /** EDT_CHANNEL_IN4 */
    EDT_CHANNEL_IN4,
    /** EDT_CHANNEL_IN5 */
    EDT_CHANNEL_IN5,
    /** EDT_CHANNEL_IN6 */
    EDT_CHANNEL_IN6,
    /** EDT_CHANNEL_IN7 */
    EDT_CHANNEL_IN7,
    /** EDT_CHANNEL_OUT0 */
    EDT_CHANNEL_OUT0,
    /** EDT_CHANNEL_OUT1 */
    EDT_CHANNEL_OUT1,
    /** EDT_CHANNEL_OUT2 */
    EDT_CHANNEL_OUT2,
    /** EDT_CHANNEL_OUT3 */
    EDT_CHANNEL_OUT3,
    /** EDT_CHANNEL_OUT4 */
    EDT_CHANNEL_OUT4,
    /** EDT_CHANNEL_OUT5 */
    EDT_CHANNEL_OUT5,
    /** EDT_CHANNEL_OUT6 */
    EDT_CHANNEL_OUT6,
    /** EDT_CHANNEL_OUT7 */
    EDT_CHANNEL_OUT7,
    /** BIST_CLK */
    BIST_CLK,
    /** BIST_RST */
    BIST_RST,
    /** BIST_FAIL */
    BIST_FAIL,
    /** BIST_DONE */
    BIST_DONE,
    /** DBG_DO0 */
    DBG_DO0,
    /** DBG_DO1 */
    DBG_DO1,
    /** DBG_DO2 */
    DBG_DO2,
    /** DBG_DO3 */
    DBG_DO3,
    /** DBG_DO4 */
    DBG_DO4,
    /** DBG_DO5 */
    DBG_DO5,
    /** DBG_DO6 */
    DBG_DO6,
    /** DBG_DO7 */
    DBG_DO7,
    /** DBG_DO8 */
    DBG_DO8,
    /** DBG_DO9 */
    DBG_DO9,
    /** DBG_DO10 */
    DBG_DO10,
    /** DBG_DO11 */
    DBG_DO11,
    /** DBG_DO12 */
    DBG_DO12,
    /** DBG_DO13 */
    DBG_DO13,
    /** DBG_DO14 */
    DBG_DO14,
    /** DBG_DO15 */
    DBG_DO15,
    /** DBG_CLK */
    DBG_CLK,
    /** EXT_S_TX_EN */
    EXT_S_TX_EN,
    /** EXT_S_RX_EN */
    EXT_S_RX_EN,
    /** EXT_S_LE_2M */
    EXT_S_LE_2M,
    /** EXT_S_TX_BIT */
    EXT_S_TX_BIT,
    /** EXT_S_TX_FLAG */
    EXT_S_TX_FLAG,
    /** EXT_S_SYNC_CLK */
    EXT_S_SYNC_CLK,
    /** EXT_S_SYNC_DAT */
    EXT_S_SYNC_DAT,
    /** EXT_S_SCLK */
    EXT_S_SCLK,
    /** EXT_S_SDO */
    EXT_S_SDO,
    /** EXT_S_RX_BIT */
    EXT_S_RX_BIT,
    /** EXT_S_RX_FLAG */
    EXT_S_RX_FLAG,
    /** EXT_S_PHY_CLK */
    EXT_S_PHY_CLK,
    /** EXT_S_PHY_DAT0 */
    EXT_S_PHY_DAT0,
    /** EXT_S_PHY_DAT1 */
    EXT_S_PHY_DAT1,
    /** EXT_S_PHY_DAT2 */
    EXT_S_PHY_DAT2,
    /** EXT_S_PHY_DAT3 */
    EXT_S_PHY_DAT3,
    /** EXT_S_PHY_DAT4 */
    EXT_S_PHY_DAT4,
    /** EXT_S_PHY_DAT5 */
    EXT_S_PHY_DAT5,
    /** EXT_S_PHY_DAT6 */
    EXT_S_PHY_DAT6,
    /** EXT_S_PHY_DAT7 */
    EXT_S_PHY_DAT7,
    /** EXT_S_PHY_DAT8 */
    EXT_S_PHY_DAT8,
    /** EXT_S_PHY_DAT9 */
    EXT_S_PHY_DAT9,
    /** EXT_S_PHY_DAT10 */
    EXT_S_PHY_DAT10,
    /** LPCOMP1_P */
    LPCOMP1_P,
    /** LPCOMP1_N */
    LPCOMP1_N,
    /** LPCOMP2_P */
    LPCOMP2_P,
    /** LPCOMP2_N */
    LPCOMP2_N,
    /** GPADC_CH0 */
    GPADC_CH0,
    /** GPADC_CH1 */
    GPADC_CH1,
    /** GPADC_CH2 */
    GPADC_CH2,
    /** GPADC_CH3 */
    GPADC_CH3,
    /** GPADC_CH4 */
    GPADC_CH4,
    /** GPADC_CH5 */
    GPADC_CH5,
    /** GPADC_CH6 */
    GPADC_CH6,
    /** GPADC_CH7 */
    GPADC_CH7,
    /** SDADC_CH0 */
    SDADC_CH0,
    /** SDADC_CH1 */
    SDADC_CH1,
    /** SDADC_CH2 */
    SDADC_CH2,
    /** SDADC_CH3 */
    SDADC_CH3,
    PIN_FUNC_MAX,
} pin_function;

/** HCPU pin pad */
typedef enum
{
    PIN_PAD_UNDEF_H,
    /** PAD_SIP00 */
    PAD_SIP00,
    /** PAD_SIP01 */
    PAD_SIP01,
    /** PAD_SIP02 */
    PAD_SIP02,
    /** PAD_SIP03 */
    PAD_SIP03,
    /** PAD_SIP04 */
    PAD_SIP04,
    /** PAD_SIP05 */
    PAD_SIP05,
    /** PAD_SIP06 */
    PAD_SIP06,
    /** PAD_SIP07 */
    PAD_SIP07,
    /** PAD_SIP08 */
    PAD_SIP08,
    /** PAD_SIP09 */
    PAD_SIP09,
    /** PAD_SIP10 */
    PAD_SIP10,
    /** PAD_SIP11 */
    PAD_SIP11,
    /** PAD_SIP12 */
    PAD_SIP12,
    /** PAD_SIP13 */
    PAD_SIP13,
    /** PAD_SIP14 */
    PAD_SIP14,
    /** PAD_SIP15 */
    PAD_SIP15,
    /** PAD_SIP16 */
    PAD_SIP16,
    /** PAD_SIP17 */
    PAD_SIP17,
    /** PAD_SIP18 */
    PAD_SIP18,
    /** PAD_PA00 */
    PAD_PA00,
    /** PAD_PA01 */
    PAD_PA01,
    /** PAD_PA02 */
    PAD_PA02,
    /** PAD_PA03 */
    PAD_PA03,
    /** PAD_PA04 */
    PAD_PA04,
    /** PAD_PA05 */
    PAD_PA05,
    /** PAD_PA06 */
    PAD_PA06,
    /** PAD_PA07 */
    PAD_PA07,
    /** PAD_PA08 */
    PAD_PA08,
    /** PAD_PA09 */
    PAD_PA09,
    /** PAD_PA10 */
    PAD_PA10,
    /** PAD_PA11 */
    PAD_PA11,
    /** PAD_PA12 */
    PAD_PA12,
    /** PAD_PA13 */
    PAD_PA13,
    /** PAD_PA14 */
    PAD_PA14,
    /** PAD_PA15 */
    PAD_PA15,
    /** PAD_PA16 */
    PAD_PA16,
    /** PAD_PA17 */
    PAD_PA17,
    /** PAD_PA18 */
    PAD_PA18,
    /** PAD_PA19 */
    PAD_PA19,
    /** PAD_PA20 */
    PAD_PA20,
    /** PAD_PA21 */
    PAD_PA21,
    /** PAD_PA22 */
    PAD_PA22,
    /** PAD_PA23 */
    PAD_PA23,
    /** PAD_PA24 */
    PAD_PA24,
    /** PAD_PA25 */
    PAD_PA25,
    /** PAD_PA26 */
    PAD_PA26,
    /** PAD_PA27 */
    PAD_PA27,
    /** PAD_PA28 */
    PAD_PA28,
    /** PAD_PA29 */
    PAD_PA29,
    /** PAD_PA30 */
    PAD_PA30,
    /** PAD_PA31 */
    PAD_PA31,
    /** PAD_PA32 */
    PAD_PA32,
    /** PAD_PA33 */
    PAD_PA33,
    /** PAD_PA34 */
    PAD_PA34,
    /** PAD_PA35 */
    PAD_PA35,
    /** PAD_PA36 */
    PAD_PA36,
    /** PAD_PA37 */
    PAD_PA37,
    /** PAD_PA38 */
    PAD_PA38,
    /** PAD_PA39 */
    PAD_PA39,
    /** PAD_PA40 */
    PAD_PA40,
    /** PAD_PA41 */
    PAD_PA41,
    /** PAD_PA42 */
    PAD_PA42,
    /** PAD_PA43 */
    PAD_PA43,
    /** PAD_PA44 */
    PAD_PA44,
    /** PAD_PA45 */
    PAD_PA45,
    /** PAD_PA46 */
    PAD_PA46,
    /** PAD_PA47 */
    PAD_PA47,
    /** PAD_PA48 */
    PAD_PA48,
    /** PAD_PA49 */
    PAD_PA49,
    /** PAD_PA50 */
    PAD_PA50,
    /** PAD_PA51 */
    PAD_PA51,
    /** PAD_PA52 */
    PAD_PA52,
    /** PAD_PA53 */
    PAD_PA53,
    /** PAD_PA54 */
    PAD_PA54,
    /** PAD_PA55 */
    PAD_PA55,
    /** PAD_PA56 */
    PAD_PA56,
    /** PAD_PA57 */
    PAD_PA57,
    /** PAD_PA58 */
    PAD_PA58,
    /** PAD_PA59 */
    PAD_PA59,
    /** PAD_PA60 */
    PAD_PA60,
    /** PAD_PA61 */
    PAD_PA61,
    /** PAD_PA62 */
    PAD_PA62,
    /** PAD_PA63 */
    PAD_PA63,
    /** PAD_PA64 */
    PAD_PA64,
    /** PAD_PA65 */
    PAD_PA65,
    /** PAD_PA66 */
    PAD_PA66,
    /** PAD_PA67 */
    PAD_PA67,
    /** PAD_PA68 */
    PAD_PA68,
    /** PAD_PA69 */
    PAD_PA69,
    /** PAD_PA70 */
    PAD_PA70,
    /** PAD_PA71 */
    PAD_PA71,
    /** PAD_PA72 */
    PAD_PA72,
    /** PAD_PA73 */
    PAD_PA73,
    /** PAD_PA74 */
    PAD_PA74,
    /** PAD_PA75 */
    PAD_PA75,
    /** PAD_PA76 */
    PAD_PA76,
    /** PAD_PA77 */
    PAD_PA77,
    /** PAD_PA78 */
    PAD_PA78,
    /** PAD_PA79 */
    PAD_PA79,
    /** PAD_PA80 */
    PAD_PA80,
    PIN_PAD_MAX_H,
} pin_pad_hcpu;

/** LCPU pin pad */
typedef enum
{
    PIN_PAD_UNDEF_L,
    /** PAD_PB00 */
    PAD_PB00,
    /** PAD_PB01 */
    PAD_PB01,
    /** PAD_PB02 */
    PAD_PB02,
    /** PAD_PB03 */
    PAD_PB03,
    /** PAD_PB04 */
    PAD_PB04,
    /** PAD_PB05 */
    PAD_PB05,
    /** PAD_PB06 */
    PAD_PB06,
    /** PAD_PB07 */
    PAD_PB07,
    /** PAD_PB08 */
    PAD_PB08,
    /** PAD_PB09 */
    PAD_PB09,
    /** PAD_PB10 */
    PAD_PB10,
    /** PAD_PB11 */
    PAD_PB11,
    /** PAD_PB12 */
    PAD_PB12,
    /** PAD_PB13 */
    PAD_PB13,
    /** PAD_PB14 */
    PAD_PB14,
    /** PAD_PB15 */
    PAD_PB15,
    /** PAD_PB16 */
    PAD_PB16,
    /** PAD_PB17 */
    PAD_PB17,
    /** PAD_PB18 */
    PAD_PB18,
    /** PAD_PB19 */
    PAD_PB19,
    /** PAD_PB20 */
    PAD_PB20,
    /** PAD_PB21 */
    PAD_PB21,
    /** PAD_PB22 */
    PAD_PB22,
    /** PAD_PB23 */
    PAD_PB23,
    /** PAD_PB24 */
    PAD_PB24,
    /** PAD_PB25 */
    PAD_PB25,
    /** PAD_PB26 */
    PAD_PB26,
    /** PAD_PB27 */
    PAD_PB27,
    /** PAD_PB28 */
    PAD_PB28,
    /** PAD_PB29 */
    PAD_PB29,
    /** PAD_PB30 */
    PAD_PB30,
    /** PAD_PB31 */
    PAD_PB31,
    /** PAD_PB32 */
    PAD_PB32,
    /** PAD_PB33 */
    PAD_PB33,
    /** PAD_PB34 */
    PAD_PB34,
    /** PAD_PB35 */
    PAD_PB35,
    /** PAD_PB36 */
    PAD_PB36,
    /** PAD_PB37 */
    PAD_PB37,
    /** PAD_PB38 */
    PAD_PB38,
    /** PAD_PB39 */
    PAD_PB39,
    /** PAD_PB40 */
    PAD_PB40,
    /** PAD_PB41 */
    PAD_PB41,
    /** PAD_PB42 */
    PAD_PB42,
    /** PAD_PB43 */
    PAD_PB43,
    /** PAD_PB44 */
    PAD_PB44,
    /** PAD_PB45 */
    PAD_PB45,
    /** PAD_PB46 */
    PAD_PB46,
    /** PAD_PB47 */
    PAD_PB47,
    /** PAD_PB48 */
    PAD_PB48,
    PIN_PAD_MAX_L,
} pin_pad_lcpu;
#define PIN_FUNC_SEL_NUM  (16)

/** HCPU pad function definition table */
extern const unsigned short pin_pad_func_hcpu[][16];
/** LCPU pad function definition table */
extern const unsigned short pin_pad_func_lcpu[][16];
#ifdef PIN_DEBUG
    extern const char pin_function_str[][20];
    extern const char pin_pad_str_hcpu[][20];
    extern const char pin_pad_str_lcpu[][20];
#endif
/**
 * @}
 */

#endif
