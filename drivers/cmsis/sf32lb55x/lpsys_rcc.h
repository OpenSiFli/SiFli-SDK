#ifndef __LPSYS_RCC_H
#define __LPSYS_RCC_H

typedef struct
{
    __IO uint32_t RSTR1;
    __IO uint32_t RSTR2;
    __IO uint32_t ENR1;
    __IO uint32_t ENR2;
    __IO uint32_t CSR;
    __IO uint32_t CFGR;
    __IO uint32_t HRC1M_CAL1;
    __IO uint32_t HRC1M_CAL2;
} LPSYS_RCC_TypeDef;


/**************** Bit definition for LPSYS_RCC_RSTR1 register *****************/
#define LPSYS_RCC_RSTR1_LCPU_Pos        (0U)
#define LPSYS_RCC_RSTR1_LCPU_Msk        (0x1UL << LPSYS_RCC_RSTR1_LCPU_Pos)
#define LPSYS_RCC_RSTR1_LCPU            LPSYS_RCC_RSTR1_LCPU_Msk
#define LPSYS_RCC_RSTR1_DMAC2_Pos       (1U)
#define LPSYS_RCC_RSTR1_DMAC2_Msk       (0x1UL << LPSYS_RCC_RSTR1_DMAC2_Pos)
#define LPSYS_RCC_RSTR1_DMAC2           LPSYS_RCC_RSTR1_DMAC2_Msk
#define LPSYS_RCC_RSTR1_MAILBOX2_Pos    (2U)
#define LPSYS_RCC_RSTR1_MAILBOX2_Msk    (0x1UL << LPSYS_RCC_RSTR1_MAILBOX2_Pos)
#define LPSYS_RCC_RSTR1_MAILBOX2        LPSYS_RCC_RSTR1_MAILBOX2_Msk
#define LPSYS_RCC_RSTR1_PINMUX2_Pos     (3U)
#define LPSYS_RCC_RSTR1_PINMUX2_Msk     (0x1UL << LPSYS_RCC_RSTR1_PINMUX2_Pos)
#define LPSYS_RCC_RSTR1_PINMUX2         LPSYS_RCC_RSTR1_PINMUX2_Msk
#define LPSYS_RCC_RSTR1_PATCH_Pos       (4U)
#define LPSYS_RCC_RSTR1_PATCH_Msk       (0x1UL << LPSYS_RCC_RSTR1_PATCH_Pos)
#define LPSYS_RCC_RSTR1_PATCH           LPSYS_RCC_RSTR1_PATCH_Msk
#define LPSYS_RCC_RSTR1_USART3_Pos      (5U)
#define LPSYS_RCC_RSTR1_USART3_Msk      (0x1UL << LPSYS_RCC_RSTR1_USART3_Pos)
#define LPSYS_RCC_RSTR1_USART3          LPSYS_RCC_RSTR1_USART3_Msk
#define LPSYS_RCC_RSTR1_USART4_Pos      (6U)
#define LPSYS_RCC_RSTR1_USART4_Msk      (0x1UL << LPSYS_RCC_RSTR1_USART4_Pos)
#define LPSYS_RCC_RSTR1_USART4          LPSYS_RCC_RSTR1_USART4_Msk
#define LPSYS_RCC_RSTR1_USART5_Pos      (7U)
#define LPSYS_RCC_RSTR1_USART5_Msk      (0x1UL << LPSYS_RCC_RSTR1_USART5_Pos)
#define LPSYS_RCC_RSTR1_USART5          LPSYS_RCC_RSTR1_USART5_Msk
#define LPSYS_RCC_RSTR1_SPI3_Pos        (9U)
#define LPSYS_RCC_RSTR1_SPI3_Msk        (0x1UL << LPSYS_RCC_RSTR1_SPI3_Pos)
#define LPSYS_RCC_RSTR1_SPI3            LPSYS_RCC_RSTR1_SPI3_Msk
#define LPSYS_RCC_RSTR1_SPI4_Pos        (10U)
#define LPSYS_RCC_RSTR1_SPI4_Msk        (0x1UL << LPSYS_RCC_RSTR1_SPI4_Pos)
#define LPSYS_RCC_RSTR1_SPI4            LPSYS_RCC_RSTR1_SPI4_Msk
#define LPSYS_RCC_RSTR1_I2C4_Pos        (12U)
#define LPSYS_RCC_RSTR1_I2C4_Msk        (0x1UL << LPSYS_RCC_RSTR1_I2C4_Pos)
#define LPSYS_RCC_RSTR1_I2C4            LPSYS_RCC_RSTR1_I2C4_Msk
#define LPSYS_RCC_RSTR1_I2C5_Pos        (13U)
#define LPSYS_RCC_RSTR1_I2C5_Msk        (0x1UL << LPSYS_RCC_RSTR1_I2C5_Pos)
#define LPSYS_RCC_RSTR1_I2C5            LPSYS_RCC_RSTR1_I2C5_Msk
#define LPSYS_RCC_RSTR1_I2C6_Pos        (14U)
#define LPSYS_RCC_RSTR1_I2C6_Msk        (0x1UL << LPSYS_RCC_RSTR1_I2C6_Pos)
#define LPSYS_RCC_RSTR1_I2C6            LPSYS_RCC_RSTR1_I2C6_Msk
#define LPSYS_RCC_RSTR1_SYSCFG2_Pos     (15U)
#define LPSYS_RCC_RSTR1_SYSCFG2_Msk     (0x1UL << LPSYS_RCC_RSTR1_SYSCFG2_Pos)
#define LPSYS_RCC_RSTR1_SYSCFG2         LPSYS_RCC_RSTR1_SYSCFG2_Msk
#define LPSYS_RCC_RSTR1_GPTIM3_Pos      (16U)
#define LPSYS_RCC_RSTR1_GPTIM3_Msk      (0x1UL << LPSYS_RCC_RSTR1_GPTIM3_Pos)
#define LPSYS_RCC_RSTR1_GPTIM3          LPSYS_RCC_RSTR1_GPTIM3_Msk
#define LPSYS_RCC_RSTR1_GPTIM4_Pos      (17U)
#define LPSYS_RCC_RSTR1_GPTIM4_Msk      (0x1UL << LPSYS_RCC_RSTR1_GPTIM4_Pos)
#define LPSYS_RCC_RSTR1_GPTIM4          LPSYS_RCC_RSTR1_GPTIM4_Msk
#define LPSYS_RCC_RSTR1_GPTIM5_Pos      (18U)
#define LPSYS_RCC_RSTR1_GPTIM5_Msk      (0x1UL << LPSYS_RCC_RSTR1_GPTIM5_Pos)
#define LPSYS_RCC_RSTR1_GPTIM5          LPSYS_RCC_RSTR1_GPTIM5_Msk
#define LPSYS_RCC_RSTR1_BTIM3_Pos       (19U)
#define LPSYS_RCC_RSTR1_BTIM3_Msk       (0x1UL << LPSYS_RCC_RSTR1_BTIM3_Pos)
#define LPSYS_RCC_RSTR1_BTIM3           LPSYS_RCC_RSTR1_BTIM3_Msk
#define LPSYS_RCC_RSTR1_BTIM4_Pos       (20U)
#define LPSYS_RCC_RSTR1_BTIM4_Msk       (0x1UL << LPSYS_RCC_RSTR1_BTIM4_Pos)
#define LPSYS_RCC_RSTR1_BTIM4           LPSYS_RCC_RSTR1_BTIM4_Msk
#define LPSYS_RCC_RSTR1_WDT2_Pos        (21U)
#define LPSYS_RCC_RSTR1_WDT2_Msk        (0x1UL << LPSYS_RCC_RSTR1_WDT2_Pos)
#define LPSYS_RCC_RSTR1_WDT2            LPSYS_RCC_RSTR1_WDT2_Msk
#define LPSYS_RCC_RSTR1_GPADC_Pos       (22U)
#define LPSYS_RCC_RSTR1_GPADC_Msk       (0x1UL << LPSYS_RCC_RSTR1_GPADC_Pos)
#define LPSYS_RCC_RSTR1_GPADC           LPSYS_RCC_RSTR1_GPADC_Msk
#define LPSYS_RCC_RSTR1_SDADC_Pos       (23U)
#define LPSYS_RCC_RSTR1_SDADC_Msk       (0x1UL << LPSYS_RCC_RSTR1_SDADC_Pos)
#define LPSYS_RCC_RSTR1_SDADC           LPSYS_RCC_RSTR1_SDADC_Msk
#define LPSYS_RCC_RSTR1_LPCOMP_Pos      (25U)
#define LPSYS_RCC_RSTR1_LPCOMP_Msk      (0x1UL << LPSYS_RCC_RSTR1_LPCOMP_Pos)
#define LPSYS_RCC_RSTR1_LPCOMP          LPSYS_RCC_RSTR1_LPCOMP_Msk
#define LPSYS_RCC_RSTR1_TSEN_Pos        (26U)
#define LPSYS_RCC_RSTR1_TSEN_Msk        (0x1UL << LPSYS_RCC_RSTR1_TSEN_Pos)
#define LPSYS_RCC_RSTR1_TSEN            LPSYS_RCC_RSTR1_TSEN_Msk
#define LPSYS_RCC_RSTR1_PTC2_Pos        (27U)
#define LPSYS_RCC_RSTR1_PTC2_Msk        (0x1UL << LPSYS_RCC_RSTR1_PTC2_Pos)
#define LPSYS_RCC_RSTR1_PTC2            LPSYS_RCC_RSTR1_PTC2_Msk
#define LPSYS_RCC_RSTR1_LCDC2_Pos       (28U)
#define LPSYS_RCC_RSTR1_LCDC2_Msk       (0x1UL << LPSYS_RCC_RSTR1_LCDC2_Pos)
#define LPSYS_RCC_RSTR1_LCDC2           LPSYS_RCC_RSTR1_LCDC2_Msk
#define LPSYS_RCC_RSTR1_BUSMON2_Pos     (29U)
#define LPSYS_RCC_RSTR1_BUSMON2_Msk     (0x1UL << LPSYS_RCC_RSTR1_BUSMON2_Pos)
#define LPSYS_RCC_RSTR1_BUSMON2         LPSYS_RCC_RSTR1_BUSMON2_Msk

/**************** Bit definition for LPSYS_RCC_RSTR2 register *****************/
#define LPSYS_RCC_RSTR2_GPIO2_Pos       (0U)
#define LPSYS_RCC_RSTR2_GPIO2_Msk       (0x1UL << LPSYS_RCC_RSTR2_GPIO2_Pos)
#define LPSYS_RCC_RSTR2_GPIO2           LPSYS_RCC_RSTR2_GPIO2_Msk
#define LPSYS_RCC_RSTR2_QSPI4_Pos       (1U)
#define LPSYS_RCC_RSTR2_QSPI4_Msk       (0x1UL << LPSYS_RCC_RSTR2_QSPI4_Pos)
#define LPSYS_RCC_RSTR2_QSPI4           LPSYS_RCC_RSTR2_QSPI4_Msk
#define LPSYS_RCC_RSTR2_RFC_Pos         (2U)
#define LPSYS_RCC_RSTR2_RFC_Msk         (0x1UL << LPSYS_RCC_RSTR2_RFC_Pos)
#define LPSYS_RCC_RSTR2_RFC             LPSYS_RCC_RSTR2_RFC_Msk
#define LPSYS_RCC_RSTR2_PHY_Pos         (3U)
#define LPSYS_RCC_RSTR2_PHY_Msk         (0x1UL << LPSYS_RCC_RSTR2_PHY_Pos)
#define LPSYS_RCC_RSTR2_PHY             LPSYS_RCC_RSTR2_PHY_Msk
#define LPSYS_RCC_RSTR2_MAC_Pos         (4U)
#define LPSYS_RCC_RSTR2_MAC_Msk         (0x1UL << LPSYS_RCC_RSTR2_MAC_Pos)
#define LPSYS_RCC_RSTR2_MAC             LPSYS_RCC_RSTR2_MAC_Msk

/***************** Bit definition for LPSYS_RCC_ENR1 register *****************/
#define LPSYS_RCC_ENR1_DMAC2_Pos        (1U)
#define LPSYS_RCC_ENR1_DMAC2_Msk        (0x1UL << LPSYS_RCC_ENR1_DMAC2_Pos)
#define LPSYS_RCC_ENR1_DMAC2            LPSYS_RCC_ENR1_DMAC2_Msk
#define LPSYS_RCC_ENR1_MAILBOX2_Pos     (2U)
#define LPSYS_RCC_ENR1_MAILBOX2_Msk     (0x1UL << LPSYS_RCC_ENR1_MAILBOX2_Pos)
#define LPSYS_RCC_ENR1_MAILBOX2         LPSYS_RCC_ENR1_MAILBOX2_Msk
#define LPSYS_RCC_ENR1_PINMUX2_Pos      (3U)
#define LPSYS_RCC_ENR1_PINMUX2_Msk      (0x1UL << LPSYS_RCC_ENR1_PINMUX2_Pos)
#define LPSYS_RCC_ENR1_PINMUX2          LPSYS_RCC_ENR1_PINMUX2_Msk
#define LPSYS_RCC_ENR1_PATCH_Pos        (4U)
#define LPSYS_RCC_ENR1_PATCH_Msk        (0x1UL << LPSYS_RCC_ENR1_PATCH_Pos)
#define LPSYS_RCC_ENR1_PATCH            LPSYS_RCC_ENR1_PATCH_Msk
#define LPSYS_RCC_ENR1_USART3_Pos       (5U)
#define LPSYS_RCC_ENR1_USART3_Msk       (0x1UL << LPSYS_RCC_ENR1_USART3_Pos)
#define LPSYS_RCC_ENR1_USART3           LPSYS_RCC_ENR1_USART3_Msk
#define LPSYS_RCC_ENR1_USART4_Pos       (6U)
#define LPSYS_RCC_ENR1_USART4_Msk       (0x1UL << LPSYS_RCC_ENR1_USART4_Pos)
#define LPSYS_RCC_ENR1_USART4           LPSYS_RCC_ENR1_USART4_Msk
#define LPSYS_RCC_ENR1_USART5_Pos       (7U)
#define LPSYS_RCC_ENR1_USART5_Msk       (0x1UL << LPSYS_RCC_ENR1_USART5_Pos)
#define LPSYS_RCC_ENR1_USART5           LPSYS_RCC_ENR1_USART5_Msk
#define LPSYS_RCC_ENR1_SPI3_Pos         (9U)
#define LPSYS_RCC_ENR1_SPI3_Msk         (0x1UL << LPSYS_RCC_ENR1_SPI3_Pos)
#define LPSYS_RCC_ENR1_SPI3             LPSYS_RCC_ENR1_SPI3_Msk
#define LPSYS_RCC_ENR1_SPI4_Pos         (10U)
#define LPSYS_RCC_ENR1_SPI4_Msk         (0x1UL << LPSYS_RCC_ENR1_SPI4_Pos)
#define LPSYS_RCC_ENR1_SPI4             LPSYS_RCC_ENR1_SPI4_Msk
#define LPSYS_RCC_ENR1_I2C4_Pos         (12U)
#define LPSYS_RCC_ENR1_I2C4_Msk         (0x1UL << LPSYS_RCC_ENR1_I2C4_Pos)
#define LPSYS_RCC_ENR1_I2C4             LPSYS_RCC_ENR1_I2C4_Msk
#define LPSYS_RCC_ENR1_I2C5_Pos         (13U)
#define LPSYS_RCC_ENR1_I2C5_Msk         (0x1UL << LPSYS_RCC_ENR1_I2C5_Pos)
#define LPSYS_RCC_ENR1_I2C5             LPSYS_RCC_ENR1_I2C5_Msk
#define LPSYS_RCC_ENR1_I2C6_Pos         (14U)
#define LPSYS_RCC_ENR1_I2C6_Msk         (0x1UL << LPSYS_RCC_ENR1_I2C6_Pos)
#define LPSYS_RCC_ENR1_I2C6             LPSYS_RCC_ENR1_I2C6_Msk
#define LPSYS_RCC_ENR1_SYSCFG2_Pos      (15U)
#define LPSYS_RCC_ENR1_SYSCFG2_Msk      (0x1UL << LPSYS_RCC_ENR1_SYSCFG2_Pos)
#define LPSYS_RCC_ENR1_SYSCFG2          LPSYS_RCC_ENR1_SYSCFG2_Msk
#define LPSYS_RCC_ENR1_GPTIM3_Pos       (16U)
#define LPSYS_RCC_ENR1_GPTIM3_Msk       (0x1UL << LPSYS_RCC_ENR1_GPTIM3_Pos)
#define LPSYS_RCC_ENR1_GPTIM3           LPSYS_RCC_ENR1_GPTIM3_Msk
#define LPSYS_RCC_ENR1_GPTIM4_Pos       (17U)
#define LPSYS_RCC_ENR1_GPTIM4_Msk       (0x1UL << LPSYS_RCC_ENR1_GPTIM4_Pos)
#define LPSYS_RCC_ENR1_GPTIM4           LPSYS_RCC_ENR1_GPTIM4_Msk
#define LPSYS_RCC_ENR1_GPTIM5_Pos       (18U)
#define LPSYS_RCC_ENR1_GPTIM5_Msk       (0x1UL << LPSYS_RCC_ENR1_GPTIM5_Pos)
#define LPSYS_RCC_ENR1_GPTIM5           LPSYS_RCC_ENR1_GPTIM5_Msk
#define LPSYS_RCC_ENR1_BTIM3_Pos        (19U)
#define LPSYS_RCC_ENR1_BTIM3_Msk        (0x1UL << LPSYS_RCC_ENR1_BTIM3_Pos)
#define LPSYS_RCC_ENR1_BTIM3            LPSYS_RCC_ENR1_BTIM3_Msk
#define LPSYS_RCC_ENR1_BTIM4_Pos        (20U)
#define LPSYS_RCC_ENR1_BTIM4_Msk        (0x1UL << LPSYS_RCC_ENR1_BTIM4_Pos)
#define LPSYS_RCC_ENR1_BTIM4            LPSYS_RCC_ENR1_BTIM4_Msk
#define LPSYS_RCC_ENR1_WDT2_Pos         (21U)
#define LPSYS_RCC_ENR1_WDT2_Msk         (0x1UL << LPSYS_RCC_ENR1_WDT2_Pos)
#define LPSYS_RCC_ENR1_WDT2             LPSYS_RCC_ENR1_WDT2_Msk
#define LPSYS_RCC_ENR1_GPADC_Pos        (22U)
#define LPSYS_RCC_ENR1_GPADC_Msk        (0x1UL << LPSYS_RCC_ENR1_GPADC_Pos)
#define LPSYS_RCC_ENR1_GPADC            LPSYS_RCC_ENR1_GPADC_Msk
#define LPSYS_RCC_ENR1_SDADC_Pos        (23U)
#define LPSYS_RCC_ENR1_SDADC_Msk        (0x1UL << LPSYS_RCC_ENR1_SDADC_Pos)
#define LPSYS_RCC_ENR1_SDADC            LPSYS_RCC_ENR1_SDADC_Msk
#define LPSYS_RCC_ENR1_LPCOMP_Pos       (25U)
#define LPSYS_RCC_ENR1_LPCOMP_Msk       (0x1UL << LPSYS_RCC_ENR1_LPCOMP_Pos)
#define LPSYS_RCC_ENR1_LPCOMP           LPSYS_RCC_ENR1_LPCOMP_Msk
#define LPSYS_RCC_ENR1_TSEN_Pos         (26U)
#define LPSYS_RCC_ENR1_TSEN_Msk         (0x1UL << LPSYS_RCC_ENR1_TSEN_Pos)
#define LPSYS_RCC_ENR1_TSEN             LPSYS_RCC_ENR1_TSEN_Msk
#define LPSYS_RCC_ENR1_PTC2_Pos         (27U)
#define LPSYS_RCC_ENR1_PTC2_Msk         (0x1UL << LPSYS_RCC_ENR1_PTC2_Pos)
#define LPSYS_RCC_ENR1_PTC2             LPSYS_RCC_ENR1_PTC2_Msk
#define LPSYS_RCC_ENR1_LCDC2_Pos        (28U)
#define LPSYS_RCC_ENR1_LCDC2_Msk        (0x1UL << LPSYS_RCC_ENR1_LCDC2_Pos)
#define LPSYS_RCC_ENR1_LCDC2            LPSYS_RCC_ENR1_LCDC2_Msk
#define LPSYS_RCC_ENR1_BUSMON2_Pos      (29U)
#define LPSYS_RCC_ENR1_BUSMON2_Msk      (0x1UL << LPSYS_RCC_ENR1_BUSMON2_Pos)
#define LPSYS_RCC_ENR1_BUSMON2          LPSYS_RCC_ENR1_BUSMON2_Msk

/***************** Bit definition for LPSYS_RCC_ENR2 register *****************/
#define LPSYS_RCC_ENR2_GPIO2_Pos        (0U)
#define LPSYS_RCC_ENR2_GPIO2_Msk        (0x1UL << LPSYS_RCC_ENR2_GPIO2_Pos)
#define LPSYS_RCC_ENR2_GPIO2            LPSYS_RCC_ENR2_GPIO2_Msk
#define LPSYS_RCC_ENR2_QSPI4_Pos        (1U)
#define LPSYS_RCC_ENR2_QSPI4_Msk        (0x1UL << LPSYS_RCC_ENR2_QSPI4_Pos)
#define LPSYS_RCC_ENR2_QSPI4            LPSYS_RCC_ENR2_QSPI4_Msk
#define LPSYS_RCC_ENR2_RFC_Pos          (2U)
#define LPSYS_RCC_ENR2_RFC_Msk          (0x1UL << LPSYS_RCC_ENR2_RFC_Pos)
#define LPSYS_RCC_ENR2_RFC              LPSYS_RCC_ENR2_RFC_Msk
#define LPSYS_RCC_ENR2_PHY_Pos          (3U)
#define LPSYS_RCC_ENR2_PHY_Msk          (0x1UL << LPSYS_RCC_ENR2_PHY_Pos)
#define LPSYS_RCC_ENR2_PHY              LPSYS_RCC_ENR2_PHY_Msk
#define LPSYS_RCC_ENR2_MAC_Pos          (4U)
#define LPSYS_RCC_ENR2_MAC_Msk          (0x1UL << LPSYS_RCC_ENR2_MAC_Pos)
#define LPSYS_RCC_ENR2_MAC              LPSYS_RCC_ENR2_MAC_Msk

/***************** Bit definition for LPSYS_RCC_CSR register ******************/
#define LPSYS_RCC_CSR_SEL_SYS_Pos       (0U)
#define LPSYS_RCC_CSR_SEL_SYS_Msk       (0x3UL << LPSYS_RCC_CSR_SEL_SYS_Pos)
#define LPSYS_RCC_CSR_SEL_SYS           LPSYS_RCC_CSR_SEL_SYS_Msk
#define LPSYS_RCC_CSR_SEL_USART3_Pos    (4U)
#define LPSYS_RCC_CSR_SEL_USART3_Msk    (0x1UL << LPSYS_RCC_CSR_SEL_USART3_Pos)
#define LPSYS_RCC_CSR_SEL_USART3        LPSYS_RCC_CSR_SEL_USART3_Msk
#define LPSYS_RCC_CSR_SEL_USART4_Pos    (5U)
#define LPSYS_RCC_CSR_SEL_USART4_Msk    (0x1UL << LPSYS_RCC_CSR_SEL_USART4_Pos)
#define LPSYS_RCC_CSR_SEL_USART4        LPSYS_RCC_CSR_SEL_USART4_Msk
#define LPSYS_RCC_CSR_SEL_USART5_Pos    (6U)
#define LPSYS_RCC_CSR_SEL_USART5_Msk    (0x1UL << LPSYS_RCC_CSR_SEL_USART5_Pos)
#define LPSYS_RCC_CSR_SEL_USART5        LPSYS_RCC_CSR_SEL_USART5_Msk
#define LPSYS_RCC_CSR_FORCE_MAC_Pos     (30U)
#define LPSYS_RCC_CSR_FORCE_MAC_Msk     (0x1UL << LPSYS_RCC_CSR_FORCE_MAC_Pos)
#define LPSYS_RCC_CSR_FORCE_MAC         LPSYS_RCC_CSR_FORCE_MAC_Msk

/***************** Bit definition for LPSYS_RCC_CFGR register *****************/
#define LPSYS_RCC_CFGR_HDIV1_Pos        (0U)
#define LPSYS_RCC_CFGR_HDIV1_Msk        (0x3FUL << LPSYS_RCC_CFGR_HDIV1_Pos)
#define LPSYS_RCC_CFGR_HDIV1            LPSYS_RCC_CFGR_HDIV1_Msk
#define LPSYS_RCC_CFGR_HDIV2_Pos        (6U)
#define LPSYS_RCC_CFGR_HDIV2_Msk        (0x1UL << LPSYS_RCC_CFGR_HDIV2_Pos)
#define LPSYS_RCC_CFGR_HDIV2            LPSYS_RCC_CFGR_HDIV2_Msk
#define LPSYS_RCC_CFGR_PDIV1_Pos        (8U)
#define LPSYS_RCC_CFGR_PDIV1_Msk        (0x7UL << LPSYS_RCC_CFGR_PDIV1_Pos)
#define LPSYS_RCC_CFGR_PDIV1            LPSYS_RCC_CFGR_PDIV1_Msk
#define LPSYS_RCC_CFGR_PDIV2_Pos        (12U)
#define LPSYS_RCC_CFGR_PDIV2_Msk        (0x7UL << LPSYS_RCC_CFGR_PDIV2_Pos)
#define LPSYS_RCC_CFGR_PDIV2            LPSYS_RCC_CFGR_PDIV2_Msk
#define LPSYS_RCC_CFGR_MACDIV_Pos       (16U)
#define LPSYS_RCC_CFGR_MACDIV_Msk       (0x7UL << LPSYS_RCC_CFGR_MACDIV_Pos)
#define LPSYS_RCC_CFGR_MACDIV           LPSYS_RCC_CFGR_MACDIV_Msk
#define LPSYS_RCC_CFGR_MACFREQ_Pos      (19U)
#define LPSYS_RCC_CFGR_MACFREQ_Msk      (0x1FUL << LPSYS_RCC_CFGR_MACFREQ_Pos)
#define LPSYS_RCC_CFGR_MACFREQ          LPSYS_RCC_CFGR_MACFREQ_Msk

/************** Bit definition for LPSYS_RCC_HRC1M_CAL1 register **************/
#define LPSYS_RCC_HRC1M_CAL1_CAL_LENGTH_Pos  (0U)
#define LPSYS_RCC_HRC1M_CAL1_CAL_LENGTH_Msk  (0xFFFFUL << LPSYS_RCC_HRC1M_CAL1_CAL_LENGTH_Pos)
#define LPSYS_RCC_HRC1M_CAL1_CAL_LENGTH  LPSYS_RCC_HRC1M_CAL1_CAL_LENGTH_Msk
#define LPSYS_RCC_HRC1M_CAL1_CAL_EN_Pos  (30U)
#define LPSYS_RCC_HRC1M_CAL1_CAL_EN_Msk  (0x1UL << LPSYS_RCC_HRC1M_CAL1_CAL_EN_Pos)
#define LPSYS_RCC_HRC1M_CAL1_CAL_EN     LPSYS_RCC_HRC1M_CAL1_CAL_EN_Msk
#define LPSYS_RCC_HRC1M_CAL1_CAL_DONE_Pos  (31U)
#define LPSYS_RCC_HRC1M_CAL1_CAL_DONE_Msk  (0x1UL << LPSYS_RCC_HRC1M_CAL1_CAL_DONE_Pos)
#define LPSYS_RCC_HRC1M_CAL1_CAL_DONE   LPSYS_RCC_HRC1M_CAL1_CAL_DONE_Msk

/************** Bit definition for LPSYS_RCC_HRC1M_CAL2 register **************/
#define LPSYS_RCC_HRC1M_CAL2_HRC1M_CNT_Pos  (0U)
#define LPSYS_RCC_HRC1M_CAL2_HRC1M_CNT_Msk  (0xFFFFUL << LPSYS_RCC_HRC1M_CAL2_HRC1M_CNT_Pos)
#define LPSYS_RCC_HRC1M_CAL2_HRC1M_CNT  LPSYS_RCC_HRC1M_CAL2_HRC1M_CNT_Msk
#define LPSYS_RCC_HRC1M_CAL2_HXT48_CNT_Pos  (16U)
#define LPSYS_RCC_HRC1M_CAL2_HXT48_CNT_Msk  (0xFFFFUL << LPSYS_RCC_HRC1M_CAL2_HXT48_CNT_Pos)
#define LPSYS_RCC_HRC1M_CAL2_HXT48_CNT  LPSYS_RCC_HRC1M_CAL2_HXT48_CNT_Msk

#endif