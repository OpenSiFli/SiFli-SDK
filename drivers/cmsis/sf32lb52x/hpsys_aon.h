#ifndef __HPSYS_AON_H
#define __HPSYS_AON_H

typedef struct
{
    __IO uint32_t PMR;
    __IO uint32_t CR1;
    __IO uint32_t CR2;
    __IO uint32_t CR3;
    __IO uint32_t ACR;
    __IO uint32_t LSCR;
    __IO uint32_t DSCR;
    __IO uint32_t SBCR;
    __IO uint32_t WER;
    __IO uint32_t WSR;
    __IO uint32_t WCR;
    __IO uint32_t ISSR;
    __IO uint32_t ANACR;
    __IO uint32_t GTIMR;
    __IO uint32_t RESERVE0;
    __IO uint32_t RESERVE1;
} HPSYS_AON_TypeDef;


/***************** Bit definition for HPSYS_AON_PMR register ******************/
#define HPSYS_AON_PMR_MODE_Pos          (0U)
#define HPSYS_AON_PMR_MODE_Msk          (0x3UL << HPSYS_AON_PMR_MODE_Pos)
#define HPSYS_AON_PMR_MODE              HPSYS_AON_PMR_MODE_Msk
#define HPSYS_AON_PMR_FORCE_LCPU_Pos    (30U)
#define HPSYS_AON_PMR_FORCE_LCPU_Msk    (0x1UL << HPSYS_AON_PMR_FORCE_LCPU_Pos)
#define HPSYS_AON_PMR_FORCE_LCPU        HPSYS_AON_PMR_FORCE_LCPU_Msk
#define HPSYS_AON_PMR_FORCE_SLEEP_Pos   (31U)
#define HPSYS_AON_PMR_FORCE_SLEEP_Msk   (0x1UL << HPSYS_AON_PMR_FORCE_SLEEP_Pos)
#define HPSYS_AON_PMR_FORCE_SLEEP       HPSYS_AON_PMR_FORCE_SLEEP_Msk

/***************** Bit definition for HPSYS_AON_CR1 register ******************/
#define HPSYS_AON_CR1_PIN0_MODE_Pos     (0U)
#define HPSYS_AON_CR1_PIN0_MODE_Msk     (0x7UL << HPSYS_AON_CR1_PIN0_MODE_Pos)
#define HPSYS_AON_CR1_PIN0_MODE         HPSYS_AON_CR1_PIN0_MODE_Msk
#define HPSYS_AON_CR1_PIN1_MODE_Pos     (3U)
#define HPSYS_AON_CR1_PIN1_MODE_Msk     (0x7UL << HPSYS_AON_CR1_PIN1_MODE_Pos)
#define HPSYS_AON_CR1_PIN1_MODE         HPSYS_AON_CR1_PIN1_MODE_Msk
#define HPSYS_AON_CR1_PIN2_MODE_Pos     (6U)
#define HPSYS_AON_CR1_PIN2_MODE_Msk     (0x7UL << HPSYS_AON_CR1_PIN2_MODE_Pos)
#define HPSYS_AON_CR1_PIN2_MODE         HPSYS_AON_CR1_PIN2_MODE_Msk
#define HPSYS_AON_CR1_PIN3_MODE_Pos     (9U)
#define HPSYS_AON_CR1_PIN3_MODE_Msk     (0x7UL << HPSYS_AON_CR1_PIN3_MODE_Pos)
#define HPSYS_AON_CR1_PIN3_MODE         HPSYS_AON_CR1_PIN3_MODE_Msk
#define HPSYS_AON_CR1_PIN4_MODE_Pos     (12U)
#define HPSYS_AON_CR1_PIN4_MODE_Msk     (0x7UL << HPSYS_AON_CR1_PIN4_MODE_Pos)
#define HPSYS_AON_CR1_PIN4_MODE         HPSYS_AON_CR1_PIN4_MODE_Msk
#define HPSYS_AON_CR1_PIN5_MODE_Pos     (15U)
#define HPSYS_AON_CR1_PIN5_MODE_Msk     (0x7UL << HPSYS_AON_CR1_PIN5_MODE_Pos)
#define HPSYS_AON_CR1_PIN5_MODE         HPSYS_AON_CR1_PIN5_MODE_Msk
#define HPSYS_AON_CR1_PIN6_MODE_Pos     (18U)
#define HPSYS_AON_CR1_PIN6_MODE_Msk     (0x7UL << HPSYS_AON_CR1_PIN6_MODE_Pos)
#define HPSYS_AON_CR1_PIN6_MODE         HPSYS_AON_CR1_PIN6_MODE_Msk
#define HPSYS_AON_CR1_PIN7_MODE_Pos     (21U)
#define HPSYS_AON_CR1_PIN7_MODE_Msk     (0x7UL << HPSYS_AON_CR1_PIN7_MODE_Pos)
#define HPSYS_AON_CR1_PIN7_MODE         HPSYS_AON_CR1_PIN7_MODE_Msk
#define HPSYS_AON_CR1_PINOUT_SEL0_Pos   (25U)
#define HPSYS_AON_CR1_PINOUT_SEL0_Msk   (0x7UL << HPSYS_AON_CR1_PINOUT_SEL0_Pos)
#define HPSYS_AON_CR1_PINOUT_SEL0       HPSYS_AON_CR1_PINOUT_SEL0_Msk
#define HPSYS_AON_CR1_PINOUT_SEL1_Pos   (28U)
#define HPSYS_AON_CR1_PINOUT_SEL1_Msk   (0x7UL << HPSYS_AON_CR1_PINOUT_SEL1_Pos)
#define HPSYS_AON_CR1_PINOUT_SEL1       HPSYS_AON_CR1_PINOUT_SEL1_Msk
#define HPSYS_AON_CR1_GTIM_EN_Pos       (31U)
#define HPSYS_AON_CR1_GTIM_EN_Msk       (0x1UL << HPSYS_AON_CR1_GTIM_EN_Pos)
#define HPSYS_AON_CR1_GTIM_EN           HPSYS_AON_CR1_GTIM_EN_Msk

/***************** Bit definition for HPSYS_AON_CR2 register ******************/
#define HPSYS_AON_CR2_PIN8_MODE_Pos     (0U)
#define HPSYS_AON_CR2_PIN8_MODE_Msk     (0x7UL << HPSYS_AON_CR2_PIN8_MODE_Pos)
#define HPSYS_AON_CR2_PIN8_MODE         HPSYS_AON_CR2_PIN8_MODE_Msk
#define HPSYS_AON_CR2_PIN9_MODE_Pos     (3U)
#define HPSYS_AON_CR2_PIN9_MODE_Msk     (0x7UL << HPSYS_AON_CR2_PIN9_MODE_Pos)
#define HPSYS_AON_CR2_PIN9_MODE         HPSYS_AON_CR2_PIN9_MODE_Msk
#define HPSYS_AON_CR2_PIN10_MODE_Pos    (6U)
#define HPSYS_AON_CR2_PIN10_MODE_Msk    (0x7UL << HPSYS_AON_CR2_PIN10_MODE_Pos)
#define HPSYS_AON_CR2_PIN10_MODE        HPSYS_AON_CR2_PIN10_MODE_Msk
#define HPSYS_AON_CR2_PIN11_MODE_Pos    (9U)
#define HPSYS_AON_CR2_PIN11_MODE_Msk    (0x7UL << HPSYS_AON_CR2_PIN11_MODE_Pos)
#define HPSYS_AON_CR2_PIN11_MODE        HPSYS_AON_CR2_PIN11_MODE_Msk
#define HPSYS_AON_CR2_PIN12_MODE_Pos    (12U)
#define HPSYS_AON_CR2_PIN12_MODE_Msk    (0x7UL << HPSYS_AON_CR2_PIN12_MODE_Pos)
#define HPSYS_AON_CR2_PIN12_MODE        HPSYS_AON_CR2_PIN12_MODE_Msk
#define HPSYS_AON_CR2_PIN13_MODE_Pos    (15U)
#define HPSYS_AON_CR2_PIN13_MODE_Msk    (0x7UL << HPSYS_AON_CR2_PIN13_MODE_Pos)
#define HPSYS_AON_CR2_PIN13_MODE        HPSYS_AON_CR2_PIN13_MODE_Msk
#define HPSYS_AON_CR2_PIN14_MODE_Pos    (18U)
#define HPSYS_AON_CR2_PIN14_MODE_Msk    (0x7UL << HPSYS_AON_CR2_PIN14_MODE_Pos)
#define HPSYS_AON_CR2_PIN14_MODE        HPSYS_AON_CR2_PIN14_MODE_Msk
#define HPSYS_AON_CR2_PIN15_MODE_Pos    (21U)
#define HPSYS_AON_CR2_PIN15_MODE_Msk    (0x7UL << HPSYS_AON_CR2_PIN15_MODE_Pos)
#define HPSYS_AON_CR2_PIN15_MODE        HPSYS_AON_CR2_PIN15_MODE_Msk

/***************** Bit definition for HPSYS_AON_CR3 register ******************/
#define HPSYS_AON_CR3_PIN16_MODE_Pos    (0U)
#define HPSYS_AON_CR3_PIN16_MODE_Msk    (0x7UL << HPSYS_AON_CR3_PIN16_MODE_Pos)
#define HPSYS_AON_CR3_PIN16_MODE        HPSYS_AON_CR3_PIN16_MODE_Msk
#define HPSYS_AON_CR3_PIN17_MODE_Pos    (3U)
#define HPSYS_AON_CR3_PIN17_MODE_Msk    (0x7UL << HPSYS_AON_CR3_PIN17_MODE_Pos)
#define HPSYS_AON_CR3_PIN17_MODE        HPSYS_AON_CR3_PIN17_MODE_Msk
#define HPSYS_AON_CR3_PIN18_MODE_Pos    (6U)
#define HPSYS_AON_CR3_PIN18_MODE_Msk    (0x7UL << HPSYS_AON_CR3_PIN18_MODE_Pos)
#define HPSYS_AON_CR3_PIN18_MODE        HPSYS_AON_CR3_PIN18_MODE_Msk
#define HPSYS_AON_CR3_PIN19_MODE_Pos    (9U)
#define HPSYS_AON_CR3_PIN19_MODE_Msk    (0x7UL << HPSYS_AON_CR3_PIN19_MODE_Pos)
#define HPSYS_AON_CR3_PIN19_MODE        HPSYS_AON_CR3_PIN19_MODE_Msk
#define HPSYS_AON_CR3_PIN20_MODE_Pos    (12U)
#define HPSYS_AON_CR3_PIN20_MODE_Msk    (0x7UL << HPSYS_AON_CR3_PIN20_MODE_Pos)
#define HPSYS_AON_CR3_PIN20_MODE        HPSYS_AON_CR3_PIN20_MODE_Msk

/***************** Bit definition for HPSYS_AON_ACR register ******************/
#define HPSYS_AON_ACR_HRC48_REQ_Pos     (0U)
#define HPSYS_AON_ACR_HRC48_REQ_Msk     (0x1UL << HPSYS_AON_ACR_HRC48_REQ_Pos)
#define HPSYS_AON_ACR_HRC48_REQ         HPSYS_AON_ACR_HRC48_REQ_Msk
#define HPSYS_AON_ACR_HXT48_REQ_Pos     (1U)
#define HPSYS_AON_ACR_HXT48_REQ_Msk     (0x1UL << HPSYS_AON_ACR_HXT48_REQ_Pos)
#define HPSYS_AON_ACR_HXT48_REQ         HPSYS_AON_ACR_HXT48_REQ_Msk
#define HPSYS_AON_ACR_PWR_REQ_Pos       (2U)
#define HPSYS_AON_ACR_PWR_REQ_Msk       (0x1UL << HPSYS_AON_ACR_PWR_REQ_Pos)
#define HPSYS_AON_ACR_PWR_REQ           HPSYS_AON_ACR_PWR_REQ_Msk
#define HPSYS_AON_ACR_EXTPWR_REQ_Pos    (3U)
#define HPSYS_AON_ACR_EXTPWR_REQ_Msk    (0x1UL << HPSYS_AON_ACR_EXTPWR_REQ_Pos)
#define HPSYS_AON_ACR_EXTPWR_REQ        HPSYS_AON_ACR_EXTPWR_REQ_Msk
#define HPSYS_AON_ACR_HRC48_RDY_Pos     (30U)
#define HPSYS_AON_ACR_HRC48_RDY_Msk     (0x1UL << HPSYS_AON_ACR_HRC48_RDY_Pos)
#define HPSYS_AON_ACR_HRC48_RDY         HPSYS_AON_ACR_HRC48_RDY_Msk
#define HPSYS_AON_ACR_HXT48_RDY_Pos     (31U)
#define HPSYS_AON_ACR_HXT48_RDY_Msk     (0x1UL << HPSYS_AON_ACR_HXT48_RDY_Pos)
#define HPSYS_AON_ACR_HXT48_RDY         HPSYS_AON_ACR_HXT48_RDY_Msk

/***************** Bit definition for HPSYS_AON_LSCR register *****************/
#define HPSYS_AON_LSCR_HRC48_REQ_Pos    (0U)
#define HPSYS_AON_LSCR_HRC48_REQ_Msk    (0x1UL << HPSYS_AON_LSCR_HRC48_REQ_Pos)
#define HPSYS_AON_LSCR_HRC48_REQ        HPSYS_AON_LSCR_HRC48_REQ_Msk
#define HPSYS_AON_LSCR_HXT48_REQ_Pos    (1U)
#define HPSYS_AON_LSCR_HXT48_REQ_Msk    (0x1UL << HPSYS_AON_LSCR_HXT48_REQ_Pos)
#define HPSYS_AON_LSCR_HXT48_REQ        HPSYS_AON_LSCR_HXT48_REQ_Msk
#define HPSYS_AON_LSCR_PWR_REQ_Pos      (2U)
#define HPSYS_AON_LSCR_PWR_REQ_Msk      (0x1UL << HPSYS_AON_LSCR_PWR_REQ_Pos)
#define HPSYS_AON_LSCR_PWR_REQ          HPSYS_AON_LSCR_PWR_REQ_Msk
#define HPSYS_AON_LSCR_EXTPWR_REQ_Pos   (3U)
#define HPSYS_AON_LSCR_EXTPWR_REQ_Msk   (0x1UL << HPSYS_AON_LSCR_EXTPWR_REQ_Pos)
#define HPSYS_AON_LSCR_EXTPWR_REQ       HPSYS_AON_LSCR_EXTPWR_REQ_Msk

/***************** Bit definition for HPSYS_AON_DSCR register *****************/
#define HPSYS_AON_DSCR_HRC48_REQ_Pos    (0U)
#define HPSYS_AON_DSCR_HRC48_REQ_Msk    (0x1UL << HPSYS_AON_DSCR_HRC48_REQ_Pos)
#define HPSYS_AON_DSCR_HRC48_REQ        HPSYS_AON_DSCR_HRC48_REQ_Msk
#define HPSYS_AON_DSCR_HXT48_REQ_Pos    (1U)
#define HPSYS_AON_DSCR_HXT48_REQ_Msk    (0x1UL << HPSYS_AON_DSCR_HXT48_REQ_Pos)
#define HPSYS_AON_DSCR_HXT48_REQ        HPSYS_AON_DSCR_HXT48_REQ_Msk
#define HPSYS_AON_DSCR_PWR_REQ_Pos      (2U)
#define HPSYS_AON_DSCR_PWR_REQ_Msk      (0x1UL << HPSYS_AON_DSCR_PWR_REQ_Pos)
#define HPSYS_AON_DSCR_PWR_REQ          HPSYS_AON_DSCR_PWR_REQ_Msk
#define HPSYS_AON_DSCR_EXTPWR_REQ_Pos   (3U)
#define HPSYS_AON_DSCR_EXTPWR_REQ_Msk   (0x1UL << HPSYS_AON_DSCR_EXTPWR_REQ_Pos)
#define HPSYS_AON_DSCR_EXTPWR_REQ       HPSYS_AON_DSCR_EXTPWR_REQ_Msk

/***************** Bit definition for HPSYS_AON_SBCR register *****************/
#define HPSYS_AON_SBCR_HRC48_REQ_Pos    (0U)
#define HPSYS_AON_SBCR_HRC48_REQ_Msk    (0x1UL << HPSYS_AON_SBCR_HRC48_REQ_Pos)
#define HPSYS_AON_SBCR_HRC48_REQ        HPSYS_AON_SBCR_HRC48_REQ_Msk
#define HPSYS_AON_SBCR_HXT48_REQ_Pos    (1U)
#define HPSYS_AON_SBCR_HXT48_REQ_Msk    (0x1UL << HPSYS_AON_SBCR_HXT48_REQ_Pos)
#define HPSYS_AON_SBCR_HXT48_REQ        HPSYS_AON_SBCR_HXT48_REQ_Msk
#define HPSYS_AON_SBCR_PWR_REQ_Pos      (2U)
#define HPSYS_AON_SBCR_PWR_REQ_Msk      (0x1UL << HPSYS_AON_SBCR_PWR_REQ_Pos)
#define HPSYS_AON_SBCR_PWR_REQ          HPSYS_AON_SBCR_PWR_REQ_Msk
#define HPSYS_AON_SBCR_EXTPWR_REQ_Pos   (3U)
#define HPSYS_AON_SBCR_EXTPWR_REQ_Msk   (0x1UL << HPSYS_AON_SBCR_EXTPWR_REQ_Pos)
#define HPSYS_AON_SBCR_EXTPWR_REQ       HPSYS_AON_SBCR_EXTPWR_REQ_Msk
#define HPSYS_AON_SBCR_PD_RAM0_Pos      (6U)
#define HPSYS_AON_SBCR_PD_RAM0_Msk      (0x1UL << HPSYS_AON_SBCR_PD_RAM0_Pos)
#define HPSYS_AON_SBCR_PD_RAM0          HPSYS_AON_SBCR_PD_RAM0_Msk
#define HPSYS_AON_SBCR_PD_RAM1_Pos      (7U)
#define HPSYS_AON_SBCR_PD_RAM1_Msk      (0x1UL << HPSYS_AON_SBCR_PD_RAM1_Pos)
#define HPSYS_AON_SBCR_PD_RAM1          HPSYS_AON_SBCR_PD_RAM1_Msk
#define HPSYS_AON_SBCR_PD_RAM2_Pos      (8U)
#define HPSYS_AON_SBCR_PD_RAM2_Msk      (0x1UL << HPSYS_AON_SBCR_PD_RAM2_Pos)
#define HPSYS_AON_SBCR_PD_RAM2          HPSYS_AON_SBCR_PD_RAM2_Msk

/***************** Bit definition for HPSYS_AON_WER register ******************/
#define HPSYS_AON_WER_RTC_Pos           (0U)
#define HPSYS_AON_WER_RTC_Msk           (0x1UL << HPSYS_AON_WER_RTC_Pos)
#define HPSYS_AON_WER_RTC               HPSYS_AON_WER_RTC_Msk
#define HPSYS_AON_WER_GPIO1_Pos         (1U)
#define HPSYS_AON_WER_GPIO1_Msk         (0x1UL << HPSYS_AON_WER_GPIO1_Pos)
#define HPSYS_AON_WER_GPIO1             HPSYS_AON_WER_GPIO1_Msk
#define HPSYS_AON_WER_LPTIM1_Pos        (2U)
#define HPSYS_AON_WER_LPTIM1_Msk        (0x1UL << HPSYS_AON_WER_LPTIM1_Pos)
#define HPSYS_AON_WER_LPTIM1            HPSYS_AON_WER_LPTIM1_Msk
#define HPSYS_AON_WER_PMUC_Pos          (3U)
#define HPSYS_AON_WER_PMUC_Msk          (0x1UL << HPSYS_AON_WER_PMUC_Pos)
#define HPSYS_AON_WER_PMUC              HPSYS_AON_WER_PMUC_Msk
#define HPSYS_AON_WER_LP2HP_REQ_Pos     (6U)
#define HPSYS_AON_WER_LP2HP_REQ_Msk     (0x1UL << HPSYS_AON_WER_LP2HP_REQ_Pos)
#define HPSYS_AON_WER_LP2HP_REQ         HPSYS_AON_WER_LP2HP_REQ_Msk
#define HPSYS_AON_WER_LP2HP_IRQ_Pos     (7U)
#define HPSYS_AON_WER_LP2HP_IRQ_Msk     (0x1UL << HPSYS_AON_WER_LP2HP_IRQ_Pos)
#define HPSYS_AON_WER_LP2HP_IRQ         HPSYS_AON_WER_LP2HP_IRQ_Msk
#define HPSYS_AON_WER_PIN0_Pos          (8U)
#define HPSYS_AON_WER_PIN0_Msk          (0x1UL << HPSYS_AON_WER_PIN0_Pos)
#define HPSYS_AON_WER_PIN0              HPSYS_AON_WER_PIN0_Msk
#define HPSYS_AON_WER_PIN1_Pos          (9U)
#define HPSYS_AON_WER_PIN1_Msk          (0x1UL << HPSYS_AON_WER_PIN1_Pos)
#define HPSYS_AON_WER_PIN1              HPSYS_AON_WER_PIN1_Msk
#define HPSYS_AON_WER_PIN2_Pos          (10U)
#define HPSYS_AON_WER_PIN2_Msk          (0x1UL << HPSYS_AON_WER_PIN2_Pos)
#define HPSYS_AON_WER_PIN2              HPSYS_AON_WER_PIN2_Msk
#define HPSYS_AON_WER_PIN3_Pos          (11U)
#define HPSYS_AON_WER_PIN3_Msk          (0x1UL << HPSYS_AON_WER_PIN3_Pos)
#define HPSYS_AON_WER_PIN3              HPSYS_AON_WER_PIN3_Msk
#define HPSYS_AON_WER_PIN4_Pos          (12U)
#define HPSYS_AON_WER_PIN4_Msk          (0x1UL << HPSYS_AON_WER_PIN4_Pos)
#define HPSYS_AON_WER_PIN4              HPSYS_AON_WER_PIN4_Msk
#define HPSYS_AON_WER_PIN5_Pos          (13U)
#define HPSYS_AON_WER_PIN5_Msk          (0x1UL << HPSYS_AON_WER_PIN5_Pos)
#define HPSYS_AON_WER_PIN5              HPSYS_AON_WER_PIN5_Msk
#define HPSYS_AON_WER_PIN6_Pos          (14U)
#define HPSYS_AON_WER_PIN6_Msk          (0x1UL << HPSYS_AON_WER_PIN6_Pos)
#define HPSYS_AON_WER_PIN6              HPSYS_AON_WER_PIN6_Msk
#define HPSYS_AON_WER_PIN7_Pos          (15U)
#define HPSYS_AON_WER_PIN7_Msk          (0x1UL << HPSYS_AON_WER_PIN7_Pos)
#define HPSYS_AON_WER_PIN7              HPSYS_AON_WER_PIN7_Msk
#define HPSYS_AON_WER_PIN8_Pos          (16U)
#define HPSYS_AON_WER_PIN8_Msk          (0x1UL << HPSYS_AON_WER_PIN8_Pos)
#define HPSYS_AON_WER_PIN8              HPSYS_AON_WER_PIN8_Msk
#define HPSYS_AON_WER_PIN9_Pos          (17U)
#define HPSYS_AON_WER_PIN9_Msk          (0x1UL << HPSYS_AON_WER_PIN9_Pos)
#define HPSYS_AON_WER_PIN9              HPSYS_AON_WER_PIN9_Msk
#define HPSYS_AON_WER_PIN10_Pos         (18U)
#define HPSYS_AON_WER_PIN10_Msk         (0x1UL << HPSYS_AON_WER_PIN10_Pos)
#define HPSYS_AON_WER_PIN10             HPSYS_AON_WER_PIN10_Msk
#define HPSYS_AON_WER_PIN11_Pos         (19U)
#define HPSYS_AON_WER_PIN11_Msk         (0x1UL << HPSYS_AON_WER_PIN11_Pos)
#define HPSYS_AON_WER_PIN11             HPSYS_AON_WER_PIN11_Msk
#define HPSYS_AON_WER_PIN12_Pos         (20U)
#define HPSYS_AON_WER_PIN12_Msk         (0x1UL << HPSYS_AON_WER_PIN12_Pos)
#define HPSYS_AON_WER_PIN12             HPSYS_AON_WER_PIN12_Msk
#define HPSYS_AON_WER_PIN13_Pos         (21U)
#define HPSYS_AON_WER_PIN13_Msk         (0x1UL << HPSYS_AON_WER_PIN13_Pos)
#define HPSYS_AON_WER_PIN13             HPSYS_AON_WER_PIN13_Msk
#define HPSYS_AON_WER_PIN14_Pos         (22U)
#define HPSYS_AON_WER_PIN14_Msk         (0x1UL << HPSYS_AON_WER_PIN14_Pos)
#define HPSYS_AON_WER_PIN14             HPSYS_AON_WER_PIN14_Msk
#define HPSYS_AON_WER_PIN15_Pos         (23U)
#define HPSYS_AON_WER_PIN15_Msk         (0x1UL << HPSYS_AON_WER_PIN15_Pos)
#define HPSYS_AON_WER_PIN15             HPSYS_AON_WER_PIN15_Msk
#define HPSYS_AON_WER_PIN16_Pos         (24U)
#define HPSYS_AON_WER_PIN16_Msk         (0x1UL << HPSYS_AON_WER_PIN16_Pos)
#define HPSYS_AON_WER_PIN16             HPSYS_AON_WER_PIN16_Msk
#define HPSYS_AON_WER_PIN17_Pos         (25U)
#define HPSYS_AON_WER_PIN17_Msk         (0x1UL << HPSYS_AON_WER_PIN17_Pos)
#define HPSYS_AON_WER_PIN17             HPSYS_AON_WER_PIN17_Msk
#define HPSYS_AON_WER_PIN18_Pos         (26U)
#define HPSYS_AON_WER_PIN18_Msk         (0x1UL << HPSYS_AON_WER_PIN18_Pos)
#define HPSYS_AON_WER_PIN18             HPSYS_AON_WER_PIN18_Msk
#define HPSYS_AON_WER_PIN19_Pos         (27U)
#define HPSYS_AON_WER_PIN19_Msk         (0x1UL << HPSYS_AON_WER_PIN19_Pos)
#define HPSYS_AON_WER_PIN19             HPSYS_AON_WER_PIN19_Msk
#define HPSYS_AON_WER_PIN20_Pos         (28U)
#define HPSYS_AON_WER_PIN20_Msk         (0x1UL << HPSYS_AON_WER_PIN20_Pos)
#define HPSYS_AON_WER_PIN20             HPSYS_AON_WER_PIN20_Msk

/***************** Bit definition for HPSYS_AON_WSR register ******************/
#define HPSYS_AON_WSR_RTC_Pos           (0U)
#define HPSYS_AON_WSR_RTC_Msk           (0x1UL << HPSYS_AON_WSR_RTC_Pos)
#define HPSYS_AON_WSR_RTC               HPSYS_AON_WSR_RTC_Msk
#define HPSYS_AON_WSR_GPIO1_Pos         (1U)
#define HPSYS_AON_WSR_GPIO1_Msk         (0x1UL << HPSYS_AON_WSR_GPIO1_Pos)
#define HPSYS_AON_WSR_GPIO1             HPSYS_AON_WSR_GPIO1_Msk
#define HPSYS_AON_WSR_LPTIM1_Pos        (2U)
#define HPSYS_AON_WSR_LPTIM1_Msk        (0x1UL << HPSYS_AON_WSR_LPTIM1_Pos)
#define HPSYS_AON_WSR_LPTIM1            HPSYS_AON_WSR_LPTIM1_Msk
#define HPSYS_AON_WSR_PMUC_Pos          (3U)
#define HPSYS_AON_WSR_PMUC_Msk          (0x1UL << HPSYS_AON_WSR_PMUC_Pos)
#define HPSYS_AON_WSR_PMUC              HPSYS_AON_WSR_PMUC_Msk
#define HPSYS_AON_WSR_LP2HP_REQ_Pos     (6U)
#define HPSYS_AON_WSR_LP2HP_REQ_Msk     (0x1UL << HPSYS_AON_WSR_LP2HP_REQ_Pos)
#define HPSYS_AON_WSR_LP2HP_REQ         HPSYS_AON_WSR_LP2HP_REQ_Msk
#define HPSYS_AON_WSR_LP2HP_IRQ_Pos     (7U)
#define HPSYS_AON_WSR_LP2HP_IRQ_Msk     (0x1UL << HPSYS_AON_WSR_LP2HP_IRQ_Pos)
#define HPSYS_AON_WSR_LP2HP_IRQ         HPSYS_AON_WSR_LP2HP_IRQ_Msk
#define HPSYS_AON_WSR_PIN0_Pos          (8U)
#define HPSYS_AON_WSR_PIN0_Msk          (0x1UL << HPSYS_AON_WSR_PIN0_Pos)
#define HPSYS_AON_WSR_PIN0              HPSYS_AON_WSR_PIN0_Msk
#define HPSYS_AON_WSR_PIN1_Pos          (9U)
#define HPSYS_AON_WSR_PIN1_Msk          (0x1UL << HPSYS_AON_WSR_PIN1_Pos)
#define HPSYS_AON_WSR_PIN1              HPSYS_AON_WSR_PIN1_Msk
#define HPSYS_AON_WSR_PIN2_Pos          (10U)
#define HPSYS_AON_WSR_PIN2_Msk          (0x1UL << HPSYS_AON_WSR_PIN2_Pos)
#define HPSYS_AON_WSR_PIN2              HPSYS_AON_WSR_PIN2_Msk
#define HPSYS_AON_WSR_PIN3_Pos          (11U)
#define HPSYS_AON_WSR_PIN3_Msk          (0x1UL << HPSYS_AON_WSR_PIN3_Pos)
#define HPSYS_AON_WSR_PIN3              HPSYS_AON_WSR_PIN3_Msk
#define HPSYS_AON_WSR_PIN4_Pos          (12U)
#define HPSYS_AON_WSR_PIN4_Msk          (0x1UL << HPSYS_AON_WSR_PIN4_Pos)
#define HPSYS_AON_WSR_PIN4              HPSYS_AON_WSR_PIN4_Msk
#define HPSYS_AON_WSR_PIN5_Pos          (13U)
#define HPSYS_AON_WSR_PIN5_Msk          (0x1UL << HPSYS_AON_WSR_PIN5_Pos)
#define HPSYS_AON_WSR_PIN5              HPSYS_AON_WSR_PIN5_Msk
#define HPSYS_AON_WSR_PIN6_Pos          (14U)
#define HPSYS_AON_WSR_PIN6_Msk          (0x1UL << HPSYS_AON_WSR_PIN6_Pos)
#define HPSYS_AON_WSR_PIN6              HPSYS_AON_WSR_PIN6_Msk
#define HPSYS_AON_WSR_PIN7_Pos          (15U)
#define HPSYS_AON_WSR_PIN7_Msk          (0x1UL << HPSYS_AON_WSR_PIN7_Pos)
#define HPSYS_AON_WSR_PIN7              HPSYS_AON_WSR_PIN7_Msk
#define HPSYS_AON_WSR_PIN8_Pos          (16U)
#define HPSYS_AON_WSR_PIN8_Msk          (0x1UL << HPSYS_AON_WSR_PIN8_Pos)
#define HPSYS_AON_WSR_PIN8              HPSYS_AON_WSR_PIN8_Msk
#define HPSYS_AON_WSR_PIN9_Pos          (17U)
#define HPSYS_AON_WSR_PIN9_Msk          (0x1UL << HPSYS_AON_WSR_PIN9_Pos)
#define HPSYS_AON_WSR_PIN9              HPSYS_AON_WSR_PIN9_Msk
#define HPSYS_AON_WSR_PIN10_Pos         (18U)
#define HPSYS_AON_WSR_PIN10_Msk         (0x1UL << HPSYS_AON_WSR_PIN10_Pos)
#define HPSYS_AON_WSR_PIN10             HPSYS_AON_WSR_PIN10_Msk
#define HPSYS_AON_WSR_PIN11_Pos         (19U)
#define HPSYS_AON_WSR_PIN11_Msk         (0x1UL << HPSYS_AON_WSR_PIN11_Pos)
#define HPSYS_AON_WSR_PIN11             HPSYS_AON_WSR_PIN11_Msk
#define HPSYS_AON_WSR_PIN12_Pos         (20U)
#define HPSYS_AON_WSR_PIN12_Msk         (0x1UL << HPSYS_AON_WSR_PIN12_Pos)
#define HPSYS_AON_WSR_PIN12             HPSYS_AON_WSR_PIN12_Msk
#define HPSYS_AON_WSR_PIN13_Pos         (21U)
#define HPSYS_AON_WSR_PIN13_Msk         (0x1UL << HPSYS_AON_WSR_PIN13_Pos)
#define HPSYS_AON_WSR_PIN13             HPSYS_AON_WSR_PIN13_Msk
#define HPSYS_AON_WSR_PIN14_Pos         (22U)
#define HPSYS_AON_WSR_PIN14_Msk         (0x1UL << HPSYS_AON_WSR_PIN14_Pos)
#define HPSYS_AON_WSR_PIN14             HPSYS_AON_WSR_PIN14_Msk
#define HPSYS_AON_WSR_PIN15_Pos         (23U)
#define HPSYS_AON_WSR_PIN15_Msk         (0x1UL << HPSYS_AON_WSR_PIN15_Pos)
#define HPSYS_AON_WSR_PIN15             HPSYS_AON_WSR_PIN15_Msk
#define HPSYS_AON_WSR_PIN16_Pos         (24U)
#define HPSYS_AON_WSR_PIN16_Msk         (0x1UL << HPSYS_AON_WSR_PIN16_Pos)
#define HPSYS_AON_WSR_PIN16             HPSYS_AON_WSR_PIN16_Msk
#define HPSYS_AON_WSR_PIN17_Pos         (25U)
#define HPSYS_AON_WSR_PIN17_Msk         (0x1UL << HPSYS_AON_WSR_PIN17_Pos)
#define HPSYS_AON_WSR_PIN17             HPSYS_AON_WSR_PIN17_Msk
#define HPSYS_AON_WSR_PIN18_Pos         (26U)
#define HPSYS_AON_WSR_PIN18_Msk         (0x1UL << HPSYS_AON_WSR_PIN18_Pos)
#define HPSYS_AON_WSR_PIN18             HPSYS_AON_WSR_PIN18_Msk
#define HPSYS_AON_WSR_PIN19_Pos         (27U)
#define HPSYS_AON_WSR_PIN19_Msk         (0x1UL << HPSYS_AON_WSR_PIN19_Pos)
#define HPSYS_AON_WSR_PIN19             HPSYS_AON_WSR_PIN19_Msk
#define HPSYS_AON_WSR_PIN20_Pos         (28U)
#define HPSYS_AON_WSR_PIN20_Msk         (0x1UL << HPSYS_AON_WSR_PIN20_Pos)
#define HPSYS_AON_WSR_PIN20             HPSYS_AON_WSR_PIN20_Msk
#define HPSYS_AON_WSR_PIN_ALL           (HPSYS_AON_WSR_PIN0 | HPSYS_AON_WSR_PIN1 | HPSYS_AON_WSR_PIN2 | HPSYS_AON_WSR_PIN3      \
                                         | HPSYS_AON_WSR_PIN4 | HPSYS_AON_WSR_PIN5 | HPSYS_AON_WSR_PIN6 | HPSYS_AON_WSR_PIN7    \
                                         | HPSYS_AON_WSR_PIN8 | HPSYS_AON_WSR_PIN9 | HPSYS_AON_WSR_PIN10 | HPSYS_AON_WSR_PIN11  \
                                         | HPSYS_AON_WSR_PIN12 | HPSYS_AON_WSR_PIN13 | HPSYS_AON_WSR_PIN14 | HPSYS_AON_WSR_PIN15 \
                                         | HPSYS_AON_WSR_PIN16 | HPSYS_AON_WSR_PIN17 | HPSYS_AON_WSR_PIN18 | HPSYS_AON_WSR_PIN19 \
                                         | HPSYS_AON_WSR_PIN20)


#define HPSYS_AON_WSR_PIN_NUM           (21)




/***************** Bit definition for HPSYS_AON_WCR register ******************/
#define HPSYS_AON_WCR_PIN0_Pos          (8U)
#define HPSYS_AON_WCR_PIN0_Msk          (0x1UL << HPSYS_AON_WCR_PIN0_Pos)
#define HPSYS_AON_WCR_PIN0              HPSYS_AON_WCR_PIN0_Msk
#define HPSYS_AON_WCR_PIN1_Pos          (9U)
#define HPSYS_AON_WCR_PIN1_Msk          (0x1UL << HPSYS_AON_WCR_PIN1_Pos)
#define HPSYS_AON_WCR_PIN1              HPSYS_AON_WCR_PIN1_Msk
#define HPSYS_AON_WCR_PIN2_Pos          (10U)
#define HPSYS_AON_WCR_PIN2_Msk          (0x1UL << HPSYS_AON_WCR_PIN2_Pos)
#define HPSYS_AON_WCR_PIN2              HPSYS_AON_WCR_PIN2_Msk
#define HPSYS_AON_WCR_PIN3_Pos          (11U)
#define HPSYS_AON_WCR_PIN3_Msk          (0x1UL << HPSYS_AON_WCR_PIN3_Pos)
#define HPSYS_AON_WCR_PIN3              HPSYS_AON_WCR_PIN3_Msk
#define HPSYS_AON_WCR_PIN4_Pos          (12U)
#define HPSYS_AON_WCR_PIN4_Msk          (0x1UL << HPSYS_AON_WCR_PIN4_Pos)
#define HPSYS_AON_WCR_PIN4              HPSYS_AON_WCR_PIN4_Msk
#define HPSYS_AON_WCR_PIN5_Pos          (13U)
#define HPSYS_AON_WCR_PIN5_Msk          (0x1UL << HPSYS_AON_WCR_PIN5_Pos)
#define HPSYS_AON_WCR_PIN5              HPSYS_AON_WCR_PIN5_Msk
#define HPSYS_AON_WCR_PIN6_Pos          (14U)
#define HPSYS_AON_WCR_PIN6_Msk          (0x1UL << HPSYS_AON_WCR_PIN6_Pos)
#define HPSYS_AON_WCR_PIN6              HPSYS_AON_WCR_PIN6_Msk
#define HPSYS_AON_WCR_PIN7_Pos          (15U)
#define HPSYS_AON_WCR_PIN7_Msk          (0x1UL << HPSYS_AON_WCR_PIN7_Pos)
#define HPSYS_AON_WCR_PIN7              HPSYS_AON_WCR_PIN7_Msk
#define HPSYS_AON_WCR_PIN8_Pos          (16U)
#define HPSYS_AON_WCR_PIN8_Msk          (0x1UL << HPSYS_AON_WCR_PIN8_Pos)
#define HPSYS_AON_WCR_PIN8              HPSYS_AON_WCR_PIN8_Msk
#define HPSYS_AON_WCR_PIN9_Pos          (17U)
#define HPSYS_AON_WCR_PIN9_Msk          (0x1UL << HPSYS_AON_WCR_PIN9_Pos)
#define HPSYS_AON_WCR_PIN9              HPSYS_AON_WCR_PIN9_Msk
#define HPSYS_AON_WCR_PIN10_Pos         (18U)
#define HPSYS_AON_WCR_PIN10_Msk         (0x1UL << HPSYS_AON_WCR_PIN10_Pos)
#define HPSYS_AON_WCR_PIN10             HPSYS_AON_WCR_PIN10_Msk
#define HPSYS_AON_WCR_PIN11_Pos         (19U)
#define HPSYS_AON_WCR_PIN11_Msk         (0x1UL << HPSYS_AON_WCR_PIN11_Pos)
#define HPSYS_AON_WCR_PIN11             HPSYS_AON_WCR_PIN11_Msk
#define HPSYS_AON_WCR_PIN12_Pos         (20U)
#define HPSYS_AON_WCR_PIN12_Msk         (0x1UL << HPSYS_AON_WCR_PIN12_Pos)
#define HPSYS_AON_WCR_PIN12             HPSYS_AON_WCR_PIN12_Msk
#define HPSYS_AON_WCR_PIN13_Pos         (21U)
#define HPSYS_AON_WCR_PIN13_Msk         (0x1UL << HPSYS_AON_WCR_PIN13_Pos)
#define HPSYS_AON_WCR_PIN13             HPSYS_AON_WCR_PIN13_Msk
#define HPSYS_AON_WCR_PIN14_Pos         (22U)
#define HPSYS_AON_WCR_PIN14_Msk         (0x1UL << HPSYS_AON_WCR_PIN14_Pos)
#define HPSYS_AON_WCR_PIN14             HPSYS_AON_WCR_PIN14_Msk
#define HPSYS_AON_WCR_PIN15_Pos         (23U)
#define HPSYS_AON_WCR_PIN15_Msk         (0x1UL << HPSYS_AON_WCR_PIN15_Pos)
#define HPSYS_AON_WCR_PIN15             HPSYS_AON_WCR_PIN15_Msk
#define HPSYS_AON_WCR_PIN16_Pos         (24U)
#define HPSYS_AON_WCR_PIN16_Msk         (0x1UL << HPSYS_AON_WCR_PIN16_Pos)
#define HPSYS_AON_WCR_PIN16             HPSYS_AON_WCR_PIN16_Msk
#define HPSYS_AON_WCR_PIN17_Pos         (25U)
#define HPSYS_AON_WCR_PIN17_Msk         (0x1UL << HPSYS_AON_WCR_PIN17_Pos)
#define HPSYS_AON_WCR_PIN17             HPSYS_AON_WCR_PIN17_Msk
#define HPSYS_AON_WCR_PIN18_Pos         (26U)
#define HPSYS_AON_WCR_PIN18_Msk         (0x1UL << HPSYS_AON_WCR_PIN18_Pos)
#define HPSYS_AON_WCR_PIN18             HPSYS_AON_WCR_PIN18_Msk
#define HPSYS_AON_WCR_PIN19_Pos         (27U)
#define HPSYS_AON_WCR_PIN19_Msk         (0x1UL << HPSYS_AON_WCR_PIN19_Pos)
#define HPSYS_AON_WCR_PIN19             HPSYS_AON_WCR_PIN19_Msk
#define HPSYS_AON_WCR_PIN20_Pos         (28U)
#define HPSYS_AON_WCR_PIN20_Msk         (0x1UL << HPSYS_AON_WCR_PIN20_Pos)
#define HPSYS_AON_WCR_PIN20             HPSYS_AON_WCR_PIN20_Msk
#define HPSYS_AON_WCR_AON_Pos           (31U)
#define HPSYS_AON_WCR_AON_Msk           (0x1UL << HPSYS_AON_WCR_AON_Pos)
#define HPSYS_AON_WCR_AON               HPSYS_AON_WCR_AON_Msk

/***************** Bit definition for HPSYS_AON_ISSR register *****************/
#define HPSYS_AON_ISSR_HP2LP_REQ_Pos    (0U)
#define HPSYS_AON_ISSR_HP2LP_REQ_Msk    (0x1UL << HPSYS_AON_ISSR_HP2LP_REQ_Pos)
#define HPSYS_AON_ISSR_HP2LP_REQ        HPSYS_AON_ISSR_HP2LP_REQ_Msk
#define HPSYS_AON_ISSR_LP2HP_REQ_Pos    (1U)
#define HPSYS_AON_ISSR_LP2HP_REQ_Msk    (0x1UL << HPSYS_AON_ISSR_LP2HP_REQ_Pos)
#define HPSYS_AON_ISSR_LP2HP_REQ        HPSYS_AON_ISSR_LP2HP_REQ_Msk
#define HPSYS_AON_ISSR_HP_ACTIVE_Pos    (4U)
#define HPSYS_AON_ISSR_HP_ACTIVE_Msk    (0x1UL << HPSYS_AON_ISSR_HP_ACTIVE_Pos)
#define HPSYS_AON_ISSR_HP_ACTIVE        HPSYS_AON_ISSR_HP_ACTIVE_Msk
#define HPSYS_AON_ISSR_LP_ACTIVE_Pos    (5U)
#define HPSYS_AON_ISSR_LP_ACTIVE_Msk    (0x1UL << HPSYS_AON_ISSR_LP_ACTIVE_Pos)
#define HPSYS_AON_ISSR_LP_ACTIVE        HPSYS_AON_ISSR_LP_ACTIVE_Msk

/**************** Bit definition for HPSYS_AON_ANACR register *****************/
#define HPSYS_AON_ANACR_PA_ISO_Pos      (0U)
#define HPSYS_AON_ANACR_PA_ISO_Msk      (0x1UL << HPSYS_AON_ANACR_PA_ISO_Pos)
#define HPSYS_AON_ANACR_PA_ISO          HPSYS_AON_ANACR_PA_ISO_Msk
#define HPSYS_AON_ANACR_VHP_ISO_Pos     (1U)
#define HPSYS_AON_ANACR_VHP_ISO_Msk     (0x1UL << HPSYS_AON_ANACR_VHP_ISO_Pos)
#define HPSYS_AON_ANACR_VHP_ISO         HPSYS_AON_ANACR_VHP_ISO_Msk

/**************** Bit definition for HPSYS_AON_GTIMR register *****************/
#define HPSYS_AON_GTIMR_CNT_Pos         (0U)
#define HPSYS_AON_GTIMR_CNT_Msk         (0xFFFFFFFFUL << HPSYS_AON_GTIMR_CNT_Pos)
#define HPSYS_AON_GTIMR_CNT             HPSYS_AON_GTIMR_CNT_Msk

/*************** Bit definition for HPSYS_AON_RESERVE0 register ***************/
#define HPSYS_AON_RESERVE0_DATA_Pos     (0U)
#define HPSYS_AON_RESERVE0_DATA_Msk     (0xFFFFFFFFUL << HPSYS_AON_RESERVE0_DATA_Pos)
#define HPSYS_AON_RESERVE0_DATA         HPSYS_AON_RESERVE0_DATA_Msk

/*************** Bit definition for HPSYS_AON_RESERVE1 register ***************/
#define HPSYS_AON_RESERVE1_DATA_Pos     (0U)
#define HPSYS_AON_RESERVE1_DATA_Msk     (0xFFFFFFFFUL << HPSYS_AON_RESERVE1_DATA_Pos)
#define HPSYS_AON_RESERVE1_DATA         HPSYS_AON_RESERVE1_DATA_Msk

#endif
