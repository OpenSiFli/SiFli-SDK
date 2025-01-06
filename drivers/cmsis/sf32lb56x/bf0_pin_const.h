#ifndef _BF0_PIN_CONST_H
#define _BF0_PIN_CONST_H
/** @addtogroup PINMUX
 * @{
 */

/** pin function */
typedef enum
{
    PIN_FUNC_UNDEF,
    /** MPI1_CLK */
    MPI1_CLK,
    /** MPI1_CLKB */
    MPI1_CLKB,
    /** MPI1_CS */
    MPI1_CS,
    /** MPI1_DM */
    MPI1_DM,
    /** MPI1_DQS */
    MPI1_DQS,
    /** MPI1_DQSDM */
    MPI1_DQSDM,
    /** MPI1_DQSDM0 */
    MPI1_DQSDM0,
    /** MPI1_DQSDM1 */
    MPI1_DQSDM1,
    /** MPI1_DIO0 */
    MPI1_DIO0,
    /** MPI1_DIO1 */
    MPI1_DIO1,
    /** MPI1_DIO2 */
    MPI1_DIO2,
    /** MPI1_DIO3 */
    MPI1_DIO3,
    /** MPI1_DIO4 */
    MPI1_DIO4,
    /** MPI1_DIO5 */
    MPI1_DIO5,
    /** MPI1_DIO6 */
    MPI1_DIO6,
    /** MPI1_DIO7 */
    MPI1_DIO7,
    /** MPI1_DIO8 */
    MPI1_DIO8,
    /** MPI1_DIO9 */
    MPI1_DIO9,
    /** MPI1_DIO10 */
    MPI1_DIO10,
    /** MPI1_DIO11 */
    MPI1_DIO11,
    /** MPI1_DIO12 */
    MPI1_DIO12,
    /** MPI1_DIO13 */
    MPI1_DIO13,
    /** MPI1_DIO14 */
    MPI1_DIO14,
    /** MPI1_DIO15 */
    MPI1_DIO15,
    /** MPI2_CLK */
    MPI2_CLK,
    /** MPI2_CLKB */
    MPI2_CLKB,
    /** MPI2_CS */
    MPI2_CS,
    /** MPI2_DM */
    MPI2_DM,
    /** MPI2_DQS */
    MPI2_DQS,
    /** MPI2_DQSDM */
    MPI2_DQSDM,
    /** MPI2_DQSDM0 */
    MPI2_DQSDM0,
    /** MPI2_DQSDM1 */
    MPI2_DQSDM1,
    /** MPI2_DIO0 */
    MPI2_DIO0,
    /** MPI2_DIO1 */
    MPI2_DIO1,
    /** MPI2_DIO2 */
    MPI2_DIO2,
    /** MPI2_DIO3 */
    MPI2_DIO3,
    /** MPI2_DIO4 */
    MPI2_DIO4,
    /** MPI2_DIO5 */
    MPI2_DIO5,
    /** MPI2_DIO6 */
    MPI2_DIO6,
    /** MPI2_DIO7 */
    MPI2_DIO7,
    /** MPI2_DIO8 */
    MPI2_DIO8,
    /** MPI2_DIO9 */
    MPI2_DIO9,
    /** MPI2_DIO10 */
    MPI2_DIO10,
    /** MPI2_DIO11 */
    MPI2_DIO11,
    /** MPI2_DIO12 */
    MPI2_DIO12,
    /** MPI2_DIO13 */
    MPI2_DIO13,
    /** MPI2_DIO14 */
    MPI2_DIO14,
    /** MPI2_DIO15 */
    MPI2_DIO15,
    /** MPI3_CLK */
    MPI3_CLK,
    /** MPI3_CS */
    MPI3_CS,
    /** MPI3_DQS */
    MPI3_DQS,
    /** MPI3_DIO0 */
    MPI3_DIO0,
    /** MPI3_DIO1 */
    MPI3_DIO1,
    /** MPI3_DIO2 */
    MPI3_DIO2,
    /** MPI3_DIO3 */
    MPI3_DIO3,
    /** MPI3_DIO4 */
    MPI3_DIO4,
    /** MPI3_DIO5 */
    MPI3_DIO5,
    /** MPI3_DIO6 */
    MPI3_DIO6,
    /** MPI3_DIO7 */
    MPI3_DIO7,
    /** MPI4_CLK */
    MPI4_CLK,
    /** MPI4_CS */
    MPI4_CS,
    /** MPI4_DQS */
    MPI4_DQS,
    /** MPI4_DIO0 */
    MPI4_DIO0,
    /** MPI4_DIO1 */
    MPI4_DIO1,
    /** MPI4_DIO2 */
    MPI4_DIO2,
    /** MPI4_DIO3 */
    MPI4_DIO3,
    /** MPI4_DIO4 */
    MPI4_DIO4,
    /** MPI4_DIO5 */
    MPI4_DIO5,
    /** MPI4_DIO6 */
    MPI4_DIO6,
    /** MPI4_DIO7 */
    MPI4_DIO7,
    /** MPI5_CLK */
    MPI5_CLK,
    /** MPI5_CS */
    MPI5_CS,
    /** MPI5_DQS */
    MPI5_DQS,
    /** MPI5_DIO0 */
    MPI5_DIO0,
    /** MPI5_DIO1 */
    MPI5_DIO1,
    /** MPI5_DIO2 */
    MPI5_DIO2,
    /** MPI5_DIO3 */
    MPI5_DIO3,
    /** MPI5_DIO4 */
    MPI5_DIO4,
    /** MPI5_DIO5 */
    MPI5_DIO5,
    /** MPI5_DIO6 */
    MPI5_DIO6,
    /** MPI5_DIO7 */
    MPI5_DIO7,
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
    /** GPIO_A88 */
    GPIO_A88,
    /** GPIO_A89 */
    GPIO_A89,
    /** GPIO_A90 */
    GPIO_A90,
    /** GPIO_A91 */
    GPIO_A91,
    /** GPIO_A92 */
    GPIO_A92,
    /** GPIO_A93 */
    GPIO_A93,
    /** GPIO_A94 */
    GPIO_A94,
    /** GPIO_A95 */
    GPIO_A95,
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
    /** SAIO_D0 */
    SAIO_D0,
    /** SAIO_D1 */
    SAIO_D1,
    /** SAIO_D2 */
    SAIO_D2,
    /** SAIO_D3 */
    SAIO_D3,
    /** SAIO_D4 */
    SAIO_D4,
    /** SAIO_D5 */
    SAIO_D5,
    /** SAIO_D6 */
    SAIO_D6,
    /** SAIO_D7 */
    SAIO_D7,
    /** SAIO_D8 */
    SAIO_D8,
    /** SAIO_D9 */
    SAIO_D9,
    /** SAIO_D10 */
    SAIO_D10,
    /** SAIO_D11 */
    SAIO_D11,
    /** SAIO_D12 */
    SAIO_D12,
    /** SAIO_D13 */
    SAIO_D13,
    /** SAIO_D14 */
    SAIO_D14,
    /** SAIO_D15 */
    SAIO_D15,
    /** SAIO_D16 */
    SAIO_D16,
    /** SAIO_D17 */
    SAIO_D17,
    /** SAIO_D18 */
    SAIO_D18,
    /** SAIO_D19 */
    SAIO_D19,
    /** SBIO_D0 */
    SBIO_D0,
    /** SBIO_D1 */
    SBIO_D1,
    /** SBIO_D2 */
    SBIO_D2,
    /** SBIO_D3 */
    SBIO_D3,
    /** SBIO_D4 */
    SBIO_D4,
    /** SBIO_D5 */
    SBIO_D5,
    /** SBIO_D6 */
    SBIO_D6,
    /** SBIO_D7 */
    SBIO_D7,
    /** SBIO_D8 */
    SBIO_D8,
    /** SBIO_D9 */
    SBIO_D9,
    /** SBIO_D10 */
    SBIO_D10,
    /** SBIO_D11 */
    SBIO_D11,
    /** SBIO_D12 */
    SBIO_D12,
    /** SBIO_D13 */
    SBIO_D13,
    /** SBIO_D14 */
    SBIO_D14,
    /** SBIO_D15 */
    SBIO_D15,
    /** SBIO_D16 */
    SBIO_D16,
    /** SBIO_D17 */
    SBIO_D17,
    /** SBIO_D18 */
    SBIO_D18,
    /** SBIO_D19 */
    SBIO_D19,
    /** SCIO_D0 */
    SCIO_D0,
    /** SCIO_D1 */
    SCIO_D1,
    /** SCIO_D2 */
    SCIO_D2,
    /** SCIO_D3 */
    SCIO_D3,
    /** SCIO_D4 */
    SCIO_D4,
    /** SCIO_D5 */
    SCIO_D5,
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
    /** USART6_RXD */
    USART6_RXD,
    /** USART6_TXD */
    USART6_TXD,
    /** USART6_CTS */
    USART6_CTS,
    /** USART6_RTS */
    USART6_RTS,
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
    /** I2C7_SCL */
    I2C7_SCL,
    /** I2C7_SDA */
    I2C7_SDA,
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
    /** LCDC2_JDI_SCLK */
    LCDC2_JDI_SCLK,
    /** LCDC2_JDI_SCS */
    LCDC2_JDI_SCS,
    /** LCDC2_JDI_SO */
    LCDC2_JDI_SO,
    /** LCDC2_JDI_DISP */
    LCDC2_JDI_DISP,
    /** LCDC2_JDI_EXTCOMIN */
    LCDC2_JDI_EXTCOMIN,
    /** LCDC2_JDI_XRST */
    LCDC2_JDI_XRST,
    /** LCDC2_JDI_VCK */
    LCDC2_JDI_VCK,
    /** LCDC2_JDI_VST */
    LCDC2_JDI_VST,
    /** LCDC2_JDI_ENB */
    LCDC2_JDI_ENB,
    /** LCDC2_JDI_HCK */
    LCDC2_JDI_HCK,
    /** LCDC2_JDI_HST */
    LCDC2_JDI_HST,
    /** LCDC2_JDI_R1 */
    LCDC2_JDI_R1,
    /** LCDC2_JDI_R2 */
    LCDC2_JDI_R2,
    /** LCDC2_JDI_G1 */
    LCDC2_JDI_G1,
    /** LCDC2_JDI_G2 */
    LCDC2_JDI_G2,
    /** LCDC2_JDI_B1 */
    LCDC2_JDI_B1,
    /** LCDC2_JDI_B2 */
    LCDC2_JDI_B2,
    /** LCDC2_JDI_FRP */
    LCDC2_JDI_FRP,
    /** LCDC2_JDI_XFRP */
    LCDC2_JDI_XFRP,
    /** LCDC2_JDI_VCOM */
    LCDC2_JDI_VCOM,
    /** SD1_CLK */
    SD1_CLK,
    /** SD1_CLKIN */
    SD1_CLKIN,
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
    /** SD2_CLKIN */
    SD2_CLKIN,
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
    /** I2S1_MCLK */
    I2S1_MCLK,
    /** I2S1_SDI */
    I2S1_SDI,
    /** I2S1_SDO */
    I2S1_SDO,
    /** I2S2_BCK */
    I2S2_BCK,
    /** I2S2_LRCK */
    I2S2_LRCK,
    /** I2S2_MCLK */
    I2S2_MCLK,
    /** I2S2_SDI */
    I2S2_SDI,
    /** I2S2_SDO */
    I2S2_SDO,
    /** I2S3_BCK */
    I2S3_BCK,
    /** I2S3_LRCK */
    I2S3_LRCK,
    /** I2S3_MCLK */
    I2S3_MCLK,
    /** I2S3_SDI */
    I2S3_SDI,
    /** I2S3_SDO */
    I2S3_SDO,
    /** AUD_CLK_EXT */
    AUD_CLK_EXT,
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
    /** ATIM1_CH1 */
    ATIM1_CH1,
    /** ATIM1_CH1N */
    ATIM1_CH1N,
    /** ATIM1_CH2 */
    ATIM1_CH2,
    /** ATIM1_CH2N */
    ATIM1_CH2N,
    /** ATIM1_CH3 */
    ATIM1_CH3,
    /** ATIM1_CH3N */
    ATIM1_CH3N,
    /** ATIM1_CH4 */
    ATIM1_CH4,
    /** ATIM1_ETR */
    ATIM1_ETR,
    /** ATIM1_BKIN */
    ATIM1_BKIN,
    /** ATIM1_BKIN2 */
    ATIM1_BKIN2,
    /** ATIM2_CH1 */
    ATIM2_CH1,
    /** ATIM2_CH1N */
    ATIM2_CH1N,
    /** ATIM2_CH2 */
    ATIM2_CH2,
    /** ATIM2_CH2N */
    ATIM2_CH2N,
    /** ATIM2_CH3 */
    ATIM2_CH3,
    /** ATIM2_CH3N */
    ATIM2_CH3N,
    /** ATIM2_CH4 */
    ATIM2_CH4,
    /** ATIM2_ETR */
    ATIM2_ETR,
    /** ATIM2_BKIN */
    ATIM2_BKIN,
    /** ATIM2_BKIN2 */
    ATIM2_BKIN2,
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
    /** SCI_RST */
    SCI_RST,
    /** SCI_CLK */
    SCI_CLK,
    /** SCI_DIO */
    SCI_DIO,
    /** CAN1_RXD */
    CAN1_RXD,
    /** CAN1_TXD */
    CAN1_TXD,
    /** CAN1_STBY */
    CAN1_STBY,
    /** CAN2_RXD */
    CAN2_RXD,
    /** CAN2_TXD */
    CAN2_TXD,
    /** CAN2_STBY */
    CAN2_STBY,
    /** SWCLK */
    SWCLK,
    /** SWDIO */
    SWDIO,
    /** TWI_CLK */
    TWI_CLK,
    /** TWI_DIO */
    TWI_DIO,
    /** BT_ACTIVE */
    BT_ACTIVE,
    /** WLAN_ACTIVE */
    WLAN_ACTIVE,
    /** BT_PRIORITY */
    BT_PRIORITY,
    /** BT_COLLISION */
    BT_COLLISION,
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
    /** EXT_S_LE_MODE */
    EXT_S_LE_MODE,
    /** EXT_S_TX_BIT0 */
    EXT_S_TX_BIT0,
    /** EXT_S_TX_BIT1 */
    EXT_S_TX_BIT1,
    /** EXT_S_TX_BIT2 */
    EXT_S_TX_BIT2,
    /** EXT_S_TX_FLAG */
    EXT_S_TX_FLAG,
    /** EXT_S_EDR2_MODE */
    EXT_S_EDR2_MODE,
    /** EXT_S_EDR3_MODE */
    EXT_S_EDR3_MODE,
    /** EXT_S_BR_LAST_BIT */
    EXT_S_BR_LAST_BIT,
    /** EXT_S_EDR_LAST_BIT */
    EXT_S_EDR_LAST_BIT,
    /** EXT_S_EDR_BIT_FLAG */
    EXT_S_EDR_BIT_FLAG,
    /** EXT_S_RAMP_UP_IND */
    EXT_S_RAMP_UP_IND,
    /** EXT_S_EDR_EN */
    EXT_S_EDR_EN,
    /** EXT_S_SYNC_CLK */
    EXT_S_SYNC_CLK,
    /** EXT_S_SYNC_DAT */
    EXT_S_SYNC_DAT,
    /** EXT_S_SCLK */
    EXT_S_SCLK,
    /** EXT_S_SDO */
    EXT_S_SDO,
    /** EXT_S_RX_BIT0 */
    EXT_S_RX_BIT0,
    /** EXT_S_RX_BIT1 */
    EXT_S_RX_BIT1,
    /** EXT_S_RX_BIT2 */
    EXT_S_RX_BIT2,
    /** EXT_S_RX_FLAG */
    EXT_S_RX_FLAG,
    /** PA00_TIM */
    PA00_TIM,
    /** PA01_TIM */
    PA01_TIM,
    /** PA02_TIM */
    PA02_TIM,
    /** PA03_TIM */
    PA03_TIM,
    /** PA04_TIM */
    PA04_TIM,
    /** PA05_TIM */
    PA05_TIM,
    /** PA06_TIM */
    PA06_TIM,
    /** PA07_TIM */
    PA07_TIM,
    /** PA08_TIM */
    PA08_TIM,
    /** PA09_TIM */
    PA09_TIM,
    /** PA10_TIM */
    PA10_TIM,
    /** PA11_TIM */
    PA11_TIM,
    /** PA12_TIM */
    PA12_TIM,
    /** PA13_TIM */
    PA13_TIM,
    /** PA14_TIM */
    PA14_TIM,
    /** PA15_TIM */
    PA15_TIM,
    /** PA16_TIM */
    PA16_TIM,
    /** PA17_TIM */
    PA17_TIM,
    /** PA18_TIM */
    PA18_TIM,
    /** PA19_TIM */
    PA19_TIM,
    /** PA20_TIM */
    PA20_TIM,
    /** PA21_TIM */
    PA21_TIM,
    /** PA22_TIM */
    PA22_TIM,
    /** PA23_TIM */
    PA23_TIM,
    /** PA24_TIM */
    PA24_TIM,
    /** PA25_TIM */
    PA25_TIM,
    /** PA26_TIM */
    PA26_TIM,
    /** PA27_TIM */
    PA27_TIM,
    /** PA28_TIM */
    PA28_TIM,
    /** PA29_TIM */
    PA29_TIM,
    /** PA30_TIM */
    PA30_TIM,
    /** PA31_TIM */
    PA31_TIM,
    /** PA32_TIM */
    PA32_TIM,
    /** PA33_TIM */
    PA33_TIM,
    /** PA34_TIM */
    PA34_TIM,
    /** PA35_TIM */
    PA35_TIM,
    /** PA36_TIM */
    PA36_TIM,
    /** PA37_TIM */
    PA37_TIM,
    /** PA38_TIM */
    PA38_TIM,
    /** PA39_TIM */
    PA39_TIM,
    /** PA40_TIM */
    PA40_TIM,
    /** PA41_TIM */
    PA41_TIM,
    /** PA42_TIM */
    PA42_TIM,
    /** PA43_TIM */
    PA43_TIM,
    /** PA44_TIM */
    PA44_TIM,
    /** PA45_TIM */
    PA45_TIM,
    /** PA46_TIM */
    PA46_TIM,
    /** PA47_TIM */
    PA47_TIM,
    /** PA48_TIM */
    PA48_TIM,
    /** PA49_TIM */
    PA49_TIM,
    /** PA50_TIM */
    PA50_TIM,
    /** PA51_TIM */
    PA51_TIM,
    /** PA52_TIM */
    PA52_TIM,
    /** PA53_TIM */
    PA53_TIM,
    /** PA54_TIM */
    PA54_TIM,
    /** PA55_TIM */
    PA55_TIM,
    /** PA56_TIM */
    PA56_TIM,
    /** PA57_TIM */
    PA57_TIM,
    /** PA58_TIM */
    PA58_TIM,
    /** PA59_TIM */
    PA59_TIM,
    /** PA60_TIM */
    PA60_TIM,
    /** PA61_TIM */
    PA61_TIM,
    /** PA62_TIM */
    PA62_TIM,
    /** PA63_TIM */
    PA63_TIM,
    /** PA64_TIM */
    PA64_TIM,
    /** PA65_TIM */
    PA65_TIM,
    /** PA66_TIM */
    PA66_TIM,
    /** PA67_TIM */
    PA67_TIM,
    /** PA68_TIM */
    PA68_TIM,
    /** PA69_TIM */
    PA69_TIM,
    /** PA70_TIM */
    PA70_TIM,
    /** PA71_TIM */
    PA71_TIM,
    /** PA72_TIM */
    PA72_TIM,
    /** PA73_TIM */
    PA73_TIM,
    /** PA74_TIM */
    PA74_TIM,
    /** PA75_TIM */
    PA75_TIM,
    /** PA76_TIM */
    PA76_TIM,
    /** PA77_TIM */
    PA77_TIM,
    /** PA78_TIM */
    PA78_TIM,
    /** PA00_I2C_UART */
    PA00_I2C_UART,
    /** PA01_I2C_UART */
    PA01_I2C_UART,
    /** PA02_I2C_UART */
    PA02_I2C_UART,
    /** PA03_I2C_UART */
    PA03_I2C_UART,
    /** PA04_I2C_UART */
    PA04_I2C_UART,
    /** PA05_I2C_UART */
    PA05_I2C_UART,
    /** PA06_I2C_UART */
    PA06_I2C_UART,
    /** PA07_I2C_UART */
    PA07_I2C_UART,
    /** PA08_I2C_UART */
    PA08_I2C_UART,
    /** PA09_I2C_UART */
    PA09_I2C_UART,
    /** PA10_I2C_UART */
    PA10_I2C_UART,
    /** PA11_I2C_UART */
    PA11_I2C_UART,
    /** PA12_I2C_UART */
    PA12_I2C_UART,
    /** PA13_I2C_UART */
    PA13_I2C_UART,
    /** PA14_I2C_UART */
    PA14_I2C_UART,
    /** PA15_I2C_UART */
    PA15_I2C_UART,
    /** PA16_I2C_UART */
    PA16_I2C_UART,
    /** PA17_I2C_UART */
    PA17_I2C_UART,
    /** PA18_I2C_UART */
    PA18_I2C_UART,
    /** PA19_I2C_UART */
    PA19_I2C_UART,
    /** PA20_I2C_UART */
    PA20_I2C_UART,
    /** PA21_I2C_UART */
    PA21_I2C_UART,
    /** PA22_I2C_UART */
    PA22_I2C_UART,
    /** PA23_I2C_UART */
    PA23_I2C_UART,
    /** PA24_I2C_UART */
    PA24_I2C_UART,
    /** PA25_I2C_UART */
    PA25_I2C_UART,
    /** PA26_I2C_UART */
    PA26_I2C_UART,
    /** PA27_I2C_UART */
    PA27_I2C_UART,
    /** PA28_I2C_UART */
    PA28_I2C_UART,
    /** PA29_I2C_UART */
    PA29_I2C_UART,
    /** PA30_I2C_UART */
    PA30_I2C_UART,
    /** PA31_I2C_UART */
    PA31_I2C_UART,
    /** PA32_I2C_UART */
    PA32_I2C_UART,
    /** PA33_I2C_UART */
    PA33_I2C_UART,
    /** PA34_I2C_UART */
    PA34_I2C_UART,
    /** PA35_I2C_UART */
    PA35_I2C_UART,
    /** PA36_I2C_UART */
    PA36_I2C_UART,
    /** PA37_I2C_UART */
    PA37_I2C_UART,
    /** PA38_I2C_UART */
    PA38_I2C_UART,
    /** PA39_I2C_UART */
    PA39_I2C_UART,
    /** PA40_I2C_UART */
    PA40_I2C_UART,
    /** PA41_I2C_UART */
    PA41_I2C_UART,
    /** PA42_I2C_UART */
    PA42_I2C_UART,
    /** PA43_I2C_UART */
    PA43_I2C_UART,
    /** PA44_I2C_UART */
    PA44_I2C_UART,
    /** PA45_I2C_UART */
    PA45_I2C_UART,
    /** PA46_I2C_UART */
    PA46_I2C_UART,
    /** PA47_I2C_UART */
    PA47_I2C_UART,
    /** PA48_I2C_UART */
    PA48_I2C_UART,
    /** PA49_I2C_UART */
    PA49_I2C_UART,
    /** PA50_I2C_UART */
    PA50_I2C_UART,
    /** PA51_I2C_UART */
    PA51_I2C_UART,
    /** PA52_I2C_UART */
    PA52_I2C_UART,
    /** PA53_I2C_UART */
    PA53_I2C_UART,
    /** PA54_I2C_UART */
    PA54_I2C_UART,
    /** PA55_I2C_UART */
    PA55_I2C_UART,
    /** PA56_I2C_UART */
    PA56_I2C_UART,
    /** PA57_I2C_UART */
    PA57_I2C_UART,
    /** PA58_I2C_UART */
    PA58_I2C_UART,
    /** PA59_I2C_UART */
    PA59_I2C_UART,
    /** PA60_I2C_UART */
    PA60_I2C_UART,
    /** PA61_I2C_UART */
    PA61_I2C_UART,
    /** PA62_I2C_UART */
    PA62_I2C_UART,
    /** PA63_I2C_UART */
    PA63_I2C_UART,
    /** PA64_I2C_UART */
    PA64_I2C_UART,
    /** PA65_I2C_UART */
    PA65_I2C_UART,
    /** PA66_I2C_UART */
    PA66_I2C_UART,
    /** PA67_I2C_UART */
    PA67_I2C_UART,
    /** PA68_I2C_UART */
    PA68_I2C_UART,
    /** PA69_I2C_UART */
    PA69_I2C_UART,
    /** PA70_I2C_UART */
    PA70_I2C_UART,
    /** PA71_I2C_UART */
    PA71_I2C_UART,
    /** PA72_I2C_UART */
    PA72_I2C_UART,
    /** PA73_I2C_UART */
    PA73_I2C_UART,
    /** PA74_I2C_UART */
    PA74_I2C_UART,
    /** PA75_I2C_UART */
    PA75_I2C_UART,
    /** PA76_I2C_UART */
    PA76_I2C_UART,
    /** PA77_I2C_UART */
    PA77_I2C_UART,
    /** PA78_I2C_UART */
    PA78_I2C_UART,
    /** PB00_TIM */
    PB00_TIM,
    /** PB01_TIM */
    PB01_TIM,
    /** PB02_TIM */
    PB02_TIM,
    /** PB03_TIM */
    PB03_TIM,
    /** PB04_TIM */
    PB04_TIM,
    /** PB05_TIM */
    PB05_TIM,
    /** PB06_TIM */
    PB06_TIM,
    /** PB07_TIM */
    PB07_TIM,
    /** PB08_TIM */
    PB08_TIM,
    /** PB09_TIM */
    PB09_TIM,
    /** PB10_TIM */
    PB10_TIM,
    /** PB11_TIM */
    PB11_TIM,
    /** PB12_TIM */
    PB12_TIM,
    /** PB13_TIM */
    PB13_TIM,
    /** PB14_TIM */
    PB14_TIM,
    /** PB15_TIM */
    PB15_TIM,
    /** PB16_TIM */
    PB16_TIM,
    /** PB17_TIM */
    PB17_TIM,
    /** PB18_TIM */
    PB18_TIM,
    /** PB19_TIM */
    PB19_TIM,
    /** PB20_TIM */
    PB20_TIM,
    /** PB21_TIM */
    PB21_TIM,
    /** PB22_TIM */
    PB22_TIM,
    /** PB23_TIM */
    PB23_TIM,
    /** PB24_TIM */
    PB24_TIM,
    /** PB25_TIM */
    PB25_TIM,
    /** PB26_TIM */
    PB26_TIM,
    /** PB27_TIM */
    PB27_TIM,
    /** PB28_TIM */
    PB28_TIM,
    /** PB29_TIM */
    PB29_TIM,
    /** PB30_TIM */
    PB30_TIM,
    /** PB31_TIM */
    PB31_TIM,
    /** PB00_I2C_UART */
    PB00_I2C_UART,
    /** PB01_I2C_UART */
    PB01_I2C_UART,
    /** PB02_I2C_UART */
    PB02_I2C_UART,
    /** PB03_I2C_UART */
    PB03_I2C_UART,
    /** PB04_I2C_UART */
    PB04_I2C_UART,
    /** PB05_I2C_UART */
    PB05_I2C_UART,
    /** PB06_I2C_UART */
    PB06_I2C_UART,
    /** PB07_I2C_UART */
    PB07_I2C_UART,
    /** PB08_I2C_UART */
    PB08_I2C_UART,
    /** PB09_I2C_UART */
    PB09_I2C_UART,
    /** PB10_I2C_UART */
    PB10_I2C_UART,
    /** PB11_I2C_UART */
    PB11_I2C_UART,
    /** PB12_I2C_UART */
    PB12_I2C_UART,
    /** PB13_I2C_UART */
    PB13_I2C_UART,
    /** PB14_I2C_UART */
    PB14_I2C_UART,
    /** PB15_I2C_UART */
    PB15_I2C_UART,
    /** PB16_I2C_UART */
    PB16_I2C_UART,
    /** PB17_I2C_UART */
    PB17_I2C_UART,
    /** PB18_I2C_UART */
    PB18_I2C_UART,
    /** PB19_I2C_UART */
    PB19_I2C_UART,
    /** PB20_I2C_UART */
    PB20_I2C_UART,
    /** PB21_I2C_UART */
    PB21_I2C_UART,
    /** PB22_I2C_UART */
    PB22_I2C_UART,
    /** PB23_I2C_UART */
    PB23_I2C_UART,
    /** PB24_I2C_UART */
    PB24_I2C_UART,
    /** PB25_I2C_UART */
    PB25_I2C_UART,
    /** PB26_I2C_UART */
    PB26_I2C_UART,
    /** PB27_I2C_UART */
    PB27_I2C_UART,
    /** PB28_I2C_UART */
    PB28_I2C_UART,
    /** PB29_I2C_UART */
    PB29_I2C_UART,
    /** PB30_I2C_UART */
    PB30_I2C_UART,
    /** PB31_I2C_UART */
    PB31_I2C_UART,
    /** PBR_PWR_REQ */
    PBR_PWR_REQ,
    /** PBR_GPO */
    PBR_GPO,
    /** PBR_CLK_LP */
    PBR_CLK_LP,
    /** PBR_LPTIM3_OUT */
    PBR_LPTIM3_OUT,
    /** PBR_LPTIM3_OUT_BAR */
    PBR_LPTIM3_OUT_BAR,
    PIN_FUNC_MAX,
} pin_function;

/** HCPU pin pad */
typedef enum
{
    PIN_PAD_UNDEF_H,
    /** PAD_SA00 */
    PAD_SA00,
    /** PAD_SA01 */
    PAD_SA01,
    /** PAD_SA02 */
    PAD_SA02,
    /** PAD_SA03 */
    PAD_SA03,
    /** PAD_SA04 */
    PAD_SA04,
    /** PAD_SA05 */
    PAD_SA05,
    /** PAD_SA06 */
    PAD_SA06,
    /** PAD_SA07 */
    PAD_SA07,
    /** PAD_SA08 */
    PAD_SA08,
    /** PAD_SA09 */
    PAD_SA09,
    /** PAD_SA10 */
    PAD_SA10,
    /** PAD_SA11 */
    PAD_SA11,
    /** PAD_SA12 */
    PAD_SA12,
    /** PAD_SB00 */
    PAD_SB00,
    /** PAD_SB01 */
    PAD_SB01,
    /** PAD_SB02 */
    PAD_SB02,
    /** PAD_SB03 */
    PAD_SB03,
    /** PAD_SB04 */
    PAD_SB04,
    /** PAD_SB05 */
    PAD_SB05,
    /** PAD_SB06 */
    PAD_SB06,
    /** PAD_SB07 */
    PAD_SB07,
    /** PAD_SB08 */
    PAD_SB08,
    /** PAD_SB09 */
    PAD_SB09,
    /** PAD_SB10 */
    PAD_SB10,
    /** PAD_SB11 */
    PAD_SB11,
    /** PAD_SB12 */
    PAD_SB12,
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
    PIN_PAD_MAX_H,
} pin_pad_hcpu;

/** LCPU pin pad */
typedef enum
{
    PIN_PAD_UNDEF_L,
    /** PAD_SC00 */
    PAD_SC00,
    /** PAD_SC01 */
    PAD_SC01,
    /** PAD_SC02 */
    PAD_SC02,
    /** PAD_SC03 */
    PAD_SC03,
    /** PAD_SC04 */
    PAD_SC04,
    /** PAD_SC05 */
    PAD_SC05,
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
    /** PAD_PBR0 */
    PAD_PBR0,
    /** PAD_PBR1 */
    PAD_PBR1,
    /** PAD_PBR2 */
    PAD_PBR2,
    /** PAD_PBR3 */
    PAD_PBR3,
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
