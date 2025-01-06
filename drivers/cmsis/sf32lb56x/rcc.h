#ifndef __RCC_H
#define __RCC_H

typedef struct
{
    __IO uint32_t RSTR1;
    __IO uint32_t RSTR2;
    __IO uint32_t ENR1;
    __IO uint32_t ENR2;
    __IO uint32_t CSR;
    __IO uint32_t CFGR;
} RCC_TypeDef;


/******************* Bit definition for RCC_RSTR1 register ********************/
#define RCC_RSTR1_USART1_Pos            (1U)
#define RCC_RSTR1_USART1_Msk            (0x1UL << RCC_RSTR1_USART1_Pos)
#define RCC_RSTR1_USART1                RCC_RSTR1_USART1_Msk
#define RCC_RSTR1_USART2_Pos            (2U)
#define RCC_RSTR1_USART2_Msk            (0x1UL << RCC_RSTR1_USART2_Pos)
#define RCC_RSTR1_USART2                RCC_RSTR1_USART2_Msk
#define RCC_RSTR1_USART3_Pos            (3U)
#define RCC_RSTR1_USART3_Msk            (0x1UL << RCC_RSTR1_USART3_Pos)
#define RCC_RSTR1_USART3                RCC_RSTR1_USART3_Msk
#define RCC_RSTR1_USART4_Pos            (4U)
#define RCC_RSTR1_USART4_Msk            (0x1UL << RCC_RSTR1_USART4_Pos)
#define RCC_RSTR1_USART4                RCC_RSTR1_USART4_Msk
#define RCC_RSTR1_SPI1_Pos              (5U)
#define RCC_RSTR1_SPI1_Msk              (0x1UL << RCC_RSTR1_SPI1_Pos)
#define RCC_RSTR1_SPI1                  RCC_RSTR1_SPI1_Msk
#define RCC_RSTR1_SPI2_Pos              (6U)
#define RCC_RSTR1_SPI2_Msk              (0x1UL << RCC_RSTR1_SPI2_Pos)
#define RCC_RSTR1_SPI2                  RCC_RSTR1_SPI2_Msk
#define RCC_RSTR1_SPI3_Pos              (7U)
#define RCC_RSTR1_SPI3_Msk              (0x1UL << RCC_RSTR1_SPI3_Pos)
#define RCC_RSTR1_SPI3                  RCC_RSTR1_SPI3_Msk
#define RCC_RSTR1_I2C1_Pos              (8U)
#define RCC_RSTR1_I2C1_Msk              (0x1UL << RCC_RSTR1_I2C1_Pos)
#define RCC_RSTR1_I2C1                  RCC_RSTR1_I2C1_Msk
#define RCC_RSTR1_I2C2_Pos              (9U)
#define RCC_RSTR1_I2C2_Msk              (0x1UL << RCC_RSTR1_I2C2_Pos)
#define RCC_RSTR1_I2C2                  RCC_RSTR1_I2C2_Msk
#define RCC_RSTR1_I2C3_Pos              (10U)
#define RCC_RSTR1_I2C3_Msk              (0x1UL << RCC_RSTR1_I2C3_Pos)
#define RCC_RSTR1_I2C3                  RCC_RSTR1_I2C3_Msk
#define RCC_RSTR1_GPTIM1_Pos            (11U)
#define RCC_RSTR1_GPTIM1_Msk            (0x1UL << RCC_RSTR1_GPTIM1_Pos)
#define RCC_RSTR1_GPTIM1                RCC_RSTR1_GPTIM1_Msk
#define RCC_RSTR1_GPTIM2_Pos            (12U)
#define RCC_RSTR1_GPTIM2_Msk            (0x1UL << RCC_RSTR1_GPTIM2_Pos)
#define RCC_RSTR1_GPTIM2                RCC_RSTR1_GPTIM2_Msk
#define RCC_RSTR1_GPTIM3_Pos            (13U)
#define RCC_RSTR1_GPTIM3_Msk            (0x1UL << RCC_RSTR1_GPTIM3_Pos)
#define RCC_RSTR1_GPTIM3                RCC_RSTR1_GPTIM3_Msk
#define RCC_RSTR1_GPIO_Pos              (14U)
#define RCC_RSTR1_GPIO_Msk              (0x1UL << RCC_RSTR1_GPIO_Pos)
#define RCC_RSTR1_GPIO                  RCC_RSTR1_GPIO_Msk
#define RCC_RSTR1_DMAC1_Pos             (15U)
#define RCC_RSTR1_DMAC1_Msk             (0x1UL << RCC_RSTR1_DMAC1_Pos)
#define RCC_RSTR1_DMAC1                 RCC_RSTR1_DMAC1_Msk
#define RCC_RSTR1_GPADC1_Pos            (16U)
#define RCC_RSTR1_GPADC1_Msk            (0x1UL << RCC_RSTR1_GPADC1_Pos)
#define RCC_RSTR1_GPADC1                RCC_RSTR1_GPADC1_Msk
#define RCC_RSTR1_GPADC2_Pos            (17U)
#define RCC_RSTR1_GPADC2_Msk            (0x1UL << RCC_RSTR1_GPADC2_Pos)
#define RCC_RSTR1_GPADC2                RCC_RSTR1_GPADC2_Msk
#define RCC_RSTR1_BTIM1_Pos             (18U)
#define RCC_RSTR1_BTIM1_Msk             (0x1UL << RCC_RSTR1_BTIM1_Pos)
#define RCC_RSTR1_BTIM1                 RCC_RSTR1_BTIM1_Msk
#define RCC_RSTR1_BTIM2_Pos             (19U)
#define RCC_RSTR1_BTIM2_Msk             (0x1UL << RCC_RSTR1_BTIM2_Pos)
#define RCC_RSTR1_BTIM2                 RCC_RSTR1_BTIM2_Msk
#define RCC_RSTR1_LPTIM_Pos             (20U)
#define RCC_RSTR1_LPTIM_Msk             (0x1UL << RCC_RSTR1_LPTIM_Pos)
#define RCC_RSTR1_LPTIM                 RCC_RSTR1_LPTIM_Msk
#define RCC_RSTR1_LPUART_Pos            (21U)
#define RCC_RSTR1_LPUART_Msk            (0x1UL << RCC_RSTR1_LPUART_Pos)
#define RCC_RSTR1_LPUART                RCC_RSTR1_LPUART_Msk

/******************* Bit definition for RCC_RSTR2 register ********************/
#define RCC_RSTR2_LCDC_Pos              (0U)
#define RCC_RSTR2_LCDC_Msk              (0x1UL << RCC_RSTR2_LCDC_Pos)
#define RCC_RSTR2_LCDC                  RCC_RSTR2_LCDC_Msk
#define RCC_RSTR2_PSRAMC_Pos            (1U)
#define RCC_RSTR2_PSRAMC_Msk            (0x1UL << RCC_RSTR2_PSRAMC_Pos)
#define RCC_RSTR2_PSRAMC                RCC_RSTR2_PSRAMC_Msk
#define RCC_RSTR2_TRNG_Pos              (2U)
#define RCC_RSTR2_TRNG_Msk              (0x1UL << RCC_RSTR2_TRNG_Pos)
#define RCC_RSTR2_TRNG                  RCC_RSTR2_TRNG_Msk
#define RCC_RSTR2_AES_Pos               (3U)
#define RCC_RSTR2_AES_Msk               (0x1UL << RCC_RSTR2_AES_Pos)
#define RCC_RSTR2_AES                   RCC_RSTR2_AES_Msk
#define RCC_RSTR2_CRC_Pos               (4U)
#define RCC_RSTR2_CRC_Msk               (0x1UL << RCC_RSTR2_CRC_Pos)
#define RCC_RSTR2_CRC                   RCC_RSTR2_CRC_Msk
#define RCC_RSTR2_WDT1_Pos              (6U)
#define RCC_RSTR2_WDT1_Msk              (0x1UL << RCC_RSTR2_WDT1_Pos)
#define RCC_RSTR2_WDT1                  RCC_RSTR2_WDT1_Msk
#define RCC_RSTR2_I2S_Pos               (7U)
#define RCC_RSTR2_I2S_Msk               (0x1UL << RCC_RSTR2_I2S_Pos)
#define RCC_RSTR2_I2S                   RCC_RSTR2_I2S_Msk
#define RCC_RSTR2_EPIC_Pos              (8U)
#define RCC_RSTR2_EPIC_Msk              (0x1UL << RCC_RSTR2_EPIC_Pos)
#define RCC_RSTR2_EPIC                  RCC_RSTR2_EPIC_Msk
#define RCC_RSTR2_KEYPAD_Pos            (9U)
#define RCC_RSTR2_KEYPAD_Msk            (0x1UL << RCC_RSTR2_KEYPAD_Pos)
#define RCC_RSTR2_KEYPAD                RCC_RSTR2_KEYPAD_Msk
#define RCC_RSTR2_USBC_Pos              (10U)
#define RCC_RSTR2_USBC_Msk              (0x1UL << RCC_RSTR2_USBC_Pos)
#define RCC_RSTR2_USBC                  RCC_RSTR2_USBC_Msk
#define RCC_RSTR2_GPDAC_Pos             (11U)
#define RCC_RSTR2_GPDAC_Msk             (0x1UL << RCC_RSTR2_GPDAC_Pos)
#define RCC_RSTR2_GPDAC                 RCC_RSTR2_GPDAC_Msk
#define RCC_RSTR2_MAILBOX_Pos           (12U)
#define RCC_RSTR2_MAILBOX_Msk           (0x1UL << RCC_RSTR2_MAILBOX_Pos)
#define RCC_RSTR2_MAILBOX               RCC_RSTR2_MAILBOX_Msk
#define RCC_RSTR2_SYSCFG_Pos            (13U)
#define RCC_RSTR2_SYSCFG_Msk            (0x1UL << RCC_RSTR2_SYSCFG_Pos)
#define RCC_RSTR2_SYSCFG                RCC_RSTR2_SYSCFG_Msk
#define RCC_RSTR2_FLASHC_Pos            (14U)
#define RCC_RSTR2_FLASHC_Msk            (0x1UL << RCC_RSTR2_FLASHC_Pos)
#define RCC_RSTR2_FLASHC                RCC_RSTR2_FLASHC_Msk
#define RCC_RSTR2_DMAC2_Pos             (15U)
#define RCC_RSTR2_DMAC2_Msk             (0x1UL << RCC_RSTR2_DMAC2_Pos)
#define RCC_RSTR2_DMAC2                 RCC_RSTR2_DMAC2_Msk
#define RCC_RSTR2_ATIM1_Pos             (16U)
#define RCC_RSTR2_ATIM1_Msk             (0x1UL << RCC_RSTR2_ATIM1_Pos)
#define RCC_RSTR2_ATIM1                 RCC_RSTR2_ATIM1_Msk
#define RCC_RSTR2_ATIM2_Pos             (17U)
#define RCC_RSTR2_ATIM2_Msk             (0x1UL << RCC_RSTR2_ATIM2_Pos)
#define RCC_RSTR2_ATIM2                 RCC_RSTR2_ATIM2_Msk
#define RCC_RSTR2_BTIM3_Pos             (18U)
#define RCC_RSTR2_BTIM3_Msk             (0x1UL << RCC_RSTR2_BTIM3_Pos)
#define RCC_RSTR2_BTIM3                 RCC_RSTR2_BTIM3_Msk
#define RCC_RSTR2_BTIM4_Pos             (19U)
#define RCC_RSTR2_BTIM4_Msk             (0x1UL << RCC_RSTR2_BTIM4_Pos)
#define RCC_RSTR2_BTIM4                 RCC_RSTR2_BTIM4_Msk
#define RCC_RSTR2_GPTIM4_Pos            (20U)
#define RCC_RSTR2_GPTIM4_Msk            (0x1UL << RCC_RSTR2_GPTIM4_Pos)
#define RCC_RSTR2_GPTIM4                RCC_RSTR2_GPTIM4_Msk
#define RCC_RSTR2_GPTIM5_Pos            (21U)
#define RCC_RSTR2_GPTIM5_Msk            (0x1UL << RCC_RSTR2_GPTIM5_Pos)
#define RCC_RSTR2_GPTIM5                RCC_RSTR2_GPTIM5_Msk
#define RCC_RSTR2_EXTDMA_Pos            (22U)
#define RCC_RSTR2_EXTDMA_Msk            (0x1UL << RCC_RSTR2_EXTDMA_Pos)
#define RCC_RSTR2_EXTDMA                RCC_RSTR2_EXTDMA_Msk
#define RCC_RSTR2_NNACC_Pos             (28U)
#define RCC_RSTR2_NNACC_Msk             (0x1UL << RCC_RSTR2_NNACC_Pos)
#define RCC_RSTR2_NNACC                 RCC_RSTR2_NNACC_Msk
#define RCC_RSTR2_GAU_Pos               (29U)
#define RCC_RSTR2_GAU_Msk               (0x1UL << RCC_RSTR2_GAU_Pos)
#define RCC_RSTR2_GAU                   RCC_RSTR2_GAU_Msk
#define RCC_RSTR2_WDT2_Pos              (30U)
#define RCC_RSTR2_WDT2_Msk              (0x1UL << RCC_RSTR2_WDT2_Pos)
#define RCC_RSTR2_WDT2                  RCC_RSTR2_WDT2_Msk
#define RCC_RSTR2_ACPU_Pos              (31U)
#define RCC_RSTR2_ACPU_Msk              (0x1UL << RCC_RSTR2_ACPU_Pos)
#define RCC_RSTR2_ACPU                  RCC_RSTR2_ACPU_Msk

/******************** Bit definition for RCC_ENR1 register ********************/
#define RCC_ENR1_USART1_Pos             (1U)
#define RCC_ENR1_USART1_Msk             (0x1UL << RCC_ENR1_USART1_Pos)
#define RCC_ENR1_USART1                 RCC_ENR1_USART1_Msk
#define RCC_ENR1_USART2_Pos             (2U)
#define RCC_ENR1_USART2_Msk             (0x1UL << RCC_ENR1_USART2_Pos)
#define RCC_ENR1_USART2                 RCC_ENR1_USART2_Msk
#define RCC_ENR1_USART3_Pos             (3U)
#define RCC_ENR1_USART3_Msk             (0x1UL << RCC_ENR1_USART3_Pos)
#define RCC_ENR1_USART3                 RCC_ENR1_USART3_Msk
#define RCC_ENR1_USART4_Pos             (4U)
#define RCC_ENR1_USART4_Msk             (0x1UL << RCC_ENR1_USART4_Pos)
#define RCC_ENR1_USART4                 RCC_ENR1_USART4_Msk
#define RCC_ENR1_SPI1_Pos               (5U)
#define RCC_ENR1_SPI1_Msk               (0x1UL << RCC_ENR1_SPI1_Pos)
#define RCC_ENR1_SPI1                   RCC_ENR1_SPI1_Msk
#define RCC_ENR1_SPI2_Pos               (6U)
#define RCC_ENR1_SPI2_Msk               (0x1UL << RCC_ENR1_SPI2_Pos)
#define RCC_ENR1_SPI2                   RCC_ENR1_SPI2_Msk
#define RCC_ENR1_SPI3_Pos               (7U)
#define RCC_ENR1_SPI3_Msk               (0x1UL << RCC_ENR1_SPI3_Pos)
#define RCC_ENR1_SPI3                   RCC_ENR1_SPI3_Msk
#define RCC_ENR1_I2C1_Pos               (8U)
#define RCC_ENR1_I2C1_Msk               (0x1UL << RCC_ENR1_I2C1_Pos)
#define RCC_ENR1_I2C1                   RCC_ENR1_I2C1_Msk
#define RCC_ENR1_I2C2_Pos               (9U)
#define RCC_ENR1_I2C2_Msk               (0x1UL << RCC_ENR1_I2C2_Pos)
#define RCC_ENR1_I2C2                   RCC_ENR1_I2C2_Msk
#define RCC_ENR1_I2C3_Pos               (10U)
#define RCC_ENR1_I2C3_Msk               (0x1UL << RCC_ENR1_I2C3_Pos)
#define RCC_ENR1_I2C3                   RCC_ENR1_I2C3_Msk
#define RCC_ENR1_GPTIM1_Pos             (11U)
#define RCC_ENR1_GPTIM1_Msk             (0x1UL << RCC_ENR1_GPTIM1_Pos)
#define RCC_ENR1_GPTIM1                 RCC_ENR1_GPTIM1_Msk
#define RCC_ENR1_GPTIM2_Pos             (12U)
#define RCC_ENR1_GPTIM2_Msk             (0x1UL << RCC_ENR1_GPTIM2_Pos)
#define RCC_ENR1_GPTIM2                 RCC_ENR1_GPTIM2_Msk
#define RCC_ENR1_GPTIM3_Pos             (13U)
#define RCC_ENR1_GPTIM3_Msk             (0x1UL << RCC_ENR1_GPTIM3_Pos)
#define RCC_ENR1_GPTIM3                 RCC_ENR1_GPTIM3_Msk
#define RCC_ENR1_GPIO_Pos               (14U)
#define RCC_ENR1_GPIO_Msk               (0x1UL << RCC_ENR1_GPIO_Pos)
#define RCC_ENR1_GPIO                   RCC_ENR1_GPIO_Msk
#define RCC_ENR1_DMAC1_Pos              (15U)
#define RCC_ENR1_DMAC1_Msk              (0x1UL << RCC_ENR1_DMAC1_Pos)
#define RCC_ENR1_DMAC1                  RCC_ENR1_DMAC1_Msk
#define RCC_ENR1_GPADC1_Pos             (16U)
#define RCC_ENR1_GPADC1_Msk             (0x1UL << RCC_ENR1_GPADC1_Pos)
#define RCC_ENR1_GPADC1                 RCC_ENR1_GPADC1_Msk
#define RCC_ENR1_GPADC2_Pos             (17U)
#define RCC_ENR1_GPADC2_Msk             (0x1UL << RCC_ENR1_GPADC2_Pos)
#define RCC_ENR1_GPADC2                 RCC_ENR1_GPADC2_Msk
#define RCC_ENR1_BTIM1_Pos              (18U)
#define RCC_ENR1_BTIM1_Msk              (0x1UL << RCC_ENR1_BTIM1_Pos)
#define RCC_ENR1_BTIM1                  RCC_ENR1_BTIM1_Msk
#define RCC_ENR1_BTIM2_Pos              (19U)
#define RCC_ENR1_BTIM2_Msk              (0x1UL << RCC_ENR1_BTIM2_Pos)
#define RCC_ENR1_BTIM2                  RCC_ENR1_BTIM2_Msk
#define RCC_ENR1_LPTIM_Pos              (20U)
#define RCC_ENR1_LPTIM_Msk              (0x1UL << RCC_ENR1_LPTIM_Pos)
#define RCC_ENR1_LPTIM                  RCC_ENR1_LPTIM_Msk
#define RCC_ENR1_LPUART_Pos             (21U)
#define RCC_ENR1_LPUART_Msk             (0x1UL << RCC_ENR1_LPUART_Pos)
#define RCC_ENR1_LPUART                 RCC_ENR1_LPUART_Msk

/******************** Bit definition for RCC_ENR2 register ********************/
#define RCC_ENR2_LCDC_Pos               (0U)
#define RCC_ENR2_LCDC_Msk               (0x1UL << RCC_ENR2_LCDC_Pos)
#define RCC_ENR2_LCDC                   RCC_ENR2_LCDC_Msk
#define RCC_ENR2_PSRAMC_Pos             (1U)
#define RCC_ENR2_PSRAMC_Msk             (0x1UL << RCC_ENR2_PSRAMC_Pos)
#define RCC_ENR2_PSRAMC                 RCC_ENR2_PSRAMC_Msk
#define RCC_ENR2_TRNG_Pos               (2U)
#define RCC_ENR2_TRNG_Msk               (0x1UL << RCC_ENR2_TRNG_Pos)
#define RCC_ENR2_TRNG                   RCC_ENR2_TRNG_Msk
#define RCC_ENR2_AES_Pos                (3U)
#define RCC_ENR2_AES_Msk                (0x1UL << RCC_ENR2_AES_Pos)
#define RCC_ENR2_AES                    RCC_ENR2_AES_Msk
#define RCC_ENR2_CRC_Pos                (4U)
#define RCC_ENR2_CRC_Msk                (0x1UL << RCC_ENR2_CRC_Pos)
#define RCC_ENR2_CRC                    RCC_ENR2_CRC_Msk
#define RCC_ENR2_WDT1_Pos               (6U)
#define RCC_ENR2_WDT1_Msk               (0x1UL << RCC_ENR2_WDT1_Pos)
#define RCC_ENR2_WDT1                   RCC_ENR2_WDT1_Msk
#define RCC_ENR2_I2S_Pos                (7U)
#define RCC_ENR2_I2S_Msk                (0x1UL << RCC_ENR2_I2S_Pos)
#define RCC_ENR2_I2S                    RCC_ENR2_I2S_Msk
#define RCC_ENR2_EPIC_Pos               (8U)
#define RCC_ENR2_EPIC_Msk               (0x1UL << RCC_ENR2_EPIC_Pos)
#define RCC_ENR2_EPIC                   RCC_ENR2_EPIC_Msk
#define RCC_ENR2_KEYPAD_Pos             (9U)
#define RCC_ENR2_KEYPAD_Msk             (0x1UL << RCC_ENR2_KEYPAD_Pos)
#define RCC_ENR2_KEYPAD                 RCC_ENR2_KEYPAD_Msk
#define RCC_ENR2_USBC_Pos               (10U)
#define RCC_ENR2_USBC_Msk               (0x1UL << RCC_ENR2_USBC_Pos)
#define RCC_ENR2_USBC                   RCC_ENR2_USBC_Msk
#define RCC_ENR2_GPDAC_Pos              (11U)
#define RCC_ENR2_GPDAC_Msk              (0x1UL << RCC_ENR2_GPDAC_Pos)
#define RCC_ENR2_GPDAC                  RCC_ENR2_GPDAC_Msk
#define RCC_ENR2_MAILBOX_Pos            (12U)
#define RCC_ENR2_MAILBOX_Msk            (0x1UL << RCC_ENR2_MAILBOX_Pos)
#define RCC_ENR2_MAILBOX                RCC_ENR2_MAILBOX_Msk
#define RCC_ENR2_SYSCFG_Pos             (13U)
#define RCC_ENR2_SYSCFG_Msk             (0x1UL << RCC_ENR2_SYSCFG_Pos)
#define RCC_ENR2_SYSCFG                 RCC_ENR2_SYSCFG_Msk
#define RCC_ENR2_FLASHC_Pos             (14U)
#define RCC_ENR2_FLASHC_Msk             (0x1UL << RCC_ENR2_FLASHC_Pos)
#define RCC_ENR2_FLASHC                 RCC_ENR2_FLASHC_Msk
#define RCC_ENR2_DMAC2_Pos              (15U)
#define RCC_ENR2_DMAC2_Msk              (0x1UL << RCC_ENR2_DMAC2_Pos)
#define RCC_ENR2_DMAC2                  RCC_ENR2_DMAC2_Msk
#define RCC_ENR2_ATIM1_Pos              (16U)
#define RCC_ENR2_ATIM1_Msk              (0x1UL << RCC_ENR2_ATIM1_Pos)
#define RCC_ENR2_ATIM1                  RCC_ENR2_ATIM1_Msk
#define RCC_ENR2_ATIM2_Pos              (17U)
#define RCC_ENR2_ATIM2_Msk              (0x1UL << RCC_ENR2_ATIM2_Pos)
#define RCC_ENR2_ATIM2                  RCC_ENR2_ATIM2_Msk
#define RCC_ENR2_BTIM3_Pos              (18U)
#define RCC_ENR2_BTIM3_Msk              (0x1UL << RCC_ENR2_BTIM3_Pos)
#define RCC_ENR2_BTIM3                  RCC_ENR2_BTIM3_Msk
#define RCC_ENR2_BTIM4_Pos              (19U)
#define RCC_ENR2_BTIM4_Msk              (0x1UL << RCC_ENR2_BTIM4_Pos)
#define RCC_ENR2_BTIM4                  RCC_ENR2_BTIM4_Msk
#define RCC_ENR2_GPTIM4_Pos             (20U)
#define RCC_ENR2_GPTIM4_Msk             (0x1UL << RCC_ENR2_GPTIM4_Pos)
#define RCC_ENR2_GPTIM4                 RCC_ENR2_GPTIM4_Msk
#define RCC_ENR2_GPTIM5_Pos             (21U)
#define RCC_ENR2_GPTIM5_Msk             (0x1UL << RCC_ENR2_GPTIM5_Pos)
#define RCC_ENR2_GPTIM5                 RCC_ENR2_GPTIM5_Msk
#define RCC_ENR2_EXTDMA_Pos             (22U)
#define RCC_ENR2_EXTDMA_Msk             (0x1UL << RCC_ENR2_EXTDMA_Pos)
#define RCC_ENR2_EXTDMA                 RCC_ENR2_EXTDMA_Msk
#define RCC_ENR2_NNACC_Pos              (28U)
#define RCC_ENR2_NNACC_Msk              (0x1UL << RCC_ENR2_NNACC_Pos)
#define RCC_ENR2_NNACC                  RCC_ENR2_NNACC_Msk
#define RCC_ENR2_GAU_Pos                (29U)
#define RCC_ENR2_GAU_Msk                (0x1UL << RCC_ENR2_GAU_Pos)
#define RCC_ENR2_GAU                    RCC_ENR2_GAU_Msk
#define RCC_ENR2_WDT2_Pos               (30U)
#define RCC_ENR2_WDT2_Msk               (0x1UL << RCC_ENR2_WDT2_Pos)
#define RCC_ENR2_WDT2                   RCC_ENR2_WDT2_Msk
#define RCC_ENR2_ACPU_Pos               (31U)
#define RCC_ENR2_ACPU_Msk               (0x1UL << RCC_ENR2_ACPU_Pos)
#define RCC_ENR2_ACPU                   RCC_ENR2_ACPU_Msk

/******************** Bit definition for RCC_CSR register *********************/
#define RCC_CSR_SEL_SYS_Pos             (0U)
#define RCC_CSR_SEL_SYS_Msk             (0x3UL << RCC_CSR_SEL_SYS_Pos)
#define RCC_CSR_SEL_SYS                 RCC_CSR_SEL_SYS_Msk
#define RCC_CSR_SEL_PSRAMC_Pos          (2U)
#define RCC_CSR_SEL_PSRAMC_Msk          (0x3UL << RCC_CSR_SEL_PSRAMC_Pos)
#define RCC_CSR_SEL_PSRAMC              RCC_CSR_SEL_PSRAMC_Msk
#define RCC_CSR_SEL_FLASHC_Pos          (4U)
#define RCC_CSR_SEL_FLASHC_Msk          (0x3UL << RCC_CSR_SEL_FLASHC_Pos)
#define RCC_CSR_SEL_FLASHC              RCC_CSR_SEL_FLASHC_Msk
#define RCC_CSR_SEL_USART1_Pos          (8U)
#define RCC_CSR_SEL_USART1_Msk          (0x1UL << RCC_CSR_SEL_USART1_Pos)
#define RCC_CSR_SEL_USART1              RCC_CSR_SEL_USART1_Msk
#define RCC_CSR_SEL_USART2_Pos          (9U)
#define RCC_CSR_SEL_USART2_Msk          (0x1UL << RCC_CSR_SEL_USART2_Pos)
#define RCC_CSR_SEL_USART2              RCC_CSR_SEL_USART2_Msk
#define RCC_CSR_SEL_USART3_Pos          (10U)
#define RCC_CSR_SEL_USART3_Msk          (0x1UL << RCC_CSR_SEL_USART3_Pos)
#define RCC_CSR_SEL_USART3              RCC_CSR_SEL_USART3_Msk
#define RCC_CSR_SEL_USART4_Pos          (11U)
#define RCC_CSR_SEL_USART4_Msk          (0x1UL << RCC_CSR_SEL_USART4_Pos)
#define RCC_CSR_SEL_USART4              RCC_CSR_SEL_USART4_Msk
#define RCC_CSR_SEL_LPUART_Pos          (12U)
#define RCC_CSR_SEL_LPUART_Msk          (0x1UL << RCC_CSR_SEL_LPUART_Pos)
#define RCC_CSR_SEL_LPUART              RCC_CSR_SEL_LPUART_Msk
#define RCC_CSR_SEL_SPI1_Pos            (13U)
#define RCC_CSR_SEL_SPI1_Msk            (0x1UL << RCC_CSR_SEL_SPI1_Pos)
#define RCC_CSR_SEL_SPI1                RCC_CSR_SEL_SPI1_Msk
#define RCC_CSR_SEL_SPI2_Pos            (14U)
#define RCC_CSR_SEL_SPI2_Msk            (0x1UL << RCC_CSR_SEL_SPI2_Pos)
#define RCC_CSR_SEL_SPI2                RCC_CSR_SEL_SPI2_Msk
#define RCC_CSR_SEL_SPI3_Pos            (15U)
#define RCC_CSR_SEL_SPI3_Msk            (0x1UL << RCC_CSR_SEL_SPI3_Pos)
#define RCC_CSR_SEL_SPI3                RCC_CSR_SEL_SPI3_Msk
#define RCC_CSR_SEL_LPTIM_Pos           (16U)
#define RCC_CSR_SEL_LPTIM_Msk           (0x3UL << RCC_CSR_SEL_LPTIM_Pos)
#define RCC_CSR_SEL_LPTIM               RCC_CSR_SEL_LPTIM_Msk
#define RCC_CSR_SEL_I2S_Pos             (18U)
#define RCC_CSR_SEL_I2S_Msk             (0x1UL << RCC_CSR_SEL_I2S_Pos)
#define RCC_CSR_SEL_I2S                 RCC_CSR_SEL_I2S_Msk

/******************** Bit definition for RCC_CFGR register ********************/
#define RCC_CFGR_HDIV1_Pos              (0U)
#define RCC_CFGR_HDIV1_Msk              (0x3FFUL << RCC_CFGR_HDIV1_Pos)
#define RCC_CFGR_HDIV1                  RCC_CFGR_HDIV1_Msk
#define RCC_CFGR_HDIV2_Pos              (10U)
#define RCC_CFGR_HDIV2_Msk              (0xFUL << RCC_CFGR_HDIV2_Pos)
#define RCC_CFGR_HDIV2                  RCC_CFGR_HDIV2_Msk
#define RCC_CFGR_PDIV1_Pos              (16U)
#define RCC_CFGR_PDIV1_Msk              (0x7UL << RCC_CFGR_PDIV1_Pos)
#define RCC_CFGR_PDIV1                  RCC_CFGR_PDIV1_Msk
#define RCC_CFGR_PDIV2_Pos              (20U)
#define RCC_CFGR_PDIV2_Msk              (0x7UL << RCC_CFGR_PDIV2_Pos)
#define RCC_CFGR_PDIV2                  RCC_CFGR_PDIV2_Msk
#define RCC_CFGR_PDIV3_Pos              (24U)
#define RCC_CFGR_PDIV3_Msk              (0x7UL << RCC_CFGR_PDIV3_Pos)
#define RCC_CFGR_PDIV3                  RCC_CFGR_PDIV3_Msk

#endif
