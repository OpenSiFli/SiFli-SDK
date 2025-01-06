#ifndef __LPSYS_AON_H
#define __LPSYS_AON_H

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
    __IO uint32_t TARGET;
    __IO uint32_t ACTUAL;
    __IO uint32_t PRE_WKUP;
    __IO uint32_t SLP_CFG;
    __IO uint32_t SLP_CTRL;
    __IO uint32_t ANACR;
    __IO uint32_t GTIMR;
    __IO uint32_t RESERVE0;
    __IO uint32_t RESERVE1;
    __IO uint32_t RSVD1[43];
    __IO uint32_t SPR;
    __IO uint32_t PCR;
} LPSYS_AON_TypeDef;


/***************** Bit definition for LPSYS_AON_PMR register ******************/
#define LPSYS_AON_PMR_MODE_Pos          (0U)
#define LPSYS_AON_PMR_MODE_Msk          (0x3UL << LPSYS_AON_PMR_MODE_Pos)
#define LPSYS_AON_PMR_MODE              LPSYS_AON_PMR_MODE_Msk
#define LPSYS_AON_PMR_CPUWAIT_Pos       (2U)
#define LPSYS_AON_PMR_CPUWAIT_Msk       (0x1UL << LPSYS_AON_PMR_CPUWAIT_Pos)
#define LPSYS_AON_PMR_CPUWAIT           LPSYS_AON_PMR_CPUWAIT_Msk
#define LPSYS_AON_PMR_FORCE_SLEEP_Pos   (31U)
#define LPSYS_AON_PMR_FORCE_SLEEP_Msk   (0x1UL << LPSYS_AON_PMR_FORCE_SLEEP_Pos)
#define LPSYS_AON_PMR_FORCE_SLEEP       LPSYS_AON_PMR_FORCE_SLEEP_Msk

/***************** Bit definition for LPSYS_AON_CR1 register ******************/
#define LPSYS_AON_CR1_PIN0_MODE_Pos     (0U)
#define LPSYS_AON_CR1_PIN0_MODE_Msk     (0x7UL << LPSYS_AON_CR1_PIN0_MODE_Pos)
#define LPSYS_AON_CR1_PIN0_MODE         LPSYS_AON_CR1_PIN0_MODE_Msk
#define LPSYS_AON_CR1_PIN1_MODE_Pos     (3U)
#define LPSYS_AON_CR1_PIN1_MODE_Msk     (0x7UL << LPSYS_AON_CR1_PIN1_MODE_Pos)
#define LPSYS_AON_CR1_PIN1_MODE         LPSYS_AON_CR1_PIN1_MODE_Msk
#define LPSYS_AON_CR1_PIN2_MODE_Pos     (6U)
#define LPSYS_AON_CR1_PIN2_MODE_Msk     (0x7UL << LPSYS_AON_CR1_PIN2_MODE_Pos)
#define LPSYS_AON_CR1_PIN2_MODE         LPSYS_AON_CR1_PIN2_MODE_Msk
#define LPSYS_AON_CR1_PIN3_MODE_Pos     (9U)
#define LPSYS_AON_CR1_PIN3_MODE_Msk     (0x7UL << LPSYS_AON_CR1_PIN3_MODE_Pos)
#define LPSYS_AON_CR1_PIN3_MODE         LPSYS_AON_CR1_PIN3_MODE_Msk
#define LPSYS_AON_CR1_PIN4_MODE_Pos     (12U)
#define LPSYS_AON_CR1_PIN4_MODE_Msk     (0x7UL << LPSYS_AON_CR1_PIN4_MODE_Pos)
#define LPSYS_AON_CR1_PIN4_MODE         LPSYS_AON_CR1_PIN4_MODE_Msk
#define LPSYS_AON_CR1_PIN5_MODE_Pos     (15U)
#define LPSYS_AON_CR1_PIN5_MODE_Msk     (0x7UL << LPSYS_AON_CR1_PIN5_MODE_Pos)
#define LPSYS_AON_CR1_PIN5_MODE         LPSYS_AON_CR1_PIN5_MODE_Msk
#define LPSYS_AON_CR1_PIN6_MODE_Pos     (18U)
#define LPSYS_AON_CR1_PIN6_MODE_Msk     (0x7UL << LPSYS_AON_CR1_PIN6_MODE_Pos)
#define LPSYS_AON_CR1_PIN6_MODE         LPSYS_AON_CR1_PIN6_MODE_Msk
#define LPSYS_AON_CR1_PIN7_MODE_Pos     (21U)
#define LPSYS_AON_CR1_PIN7_MODE_Msk     (0x7UL << LPSYS_AON_CR1_PIN7_MODE_Pos)
#define LPSYS_AON_CR1_PIN7_MODE         LPSYS_AON_CR1_PIN7_MODE_Msk
#define LPSYS_AON_CR1_PINOUT_SEL0_Pos   (25U)
#define LPSYS_AON_CR1_PINOUT_SEL0_Msk   (0x7UL << LPSYS_AON_CR1_PINOUT_SEL0_Pos)
#define LPSYS_AON_CR1_PINOUT_SEL0       LPSYS_AON_CR1_PINOUT_SEL0_Msk
#define LPSYS_AON_CR1_PINOUT_SEL1_Pos   (28U)
#define LPSYS_AON_CR1_PINOUT_SEL1_Msk   (0x7UL << LPSYS_AON_CR1_PINOUT_SEL1_Pos)
#define LPSYS_AON_CR1_PINOUT_SEL1       LPSYS_AON_CR1_PINOUT_SEL1_Msk
#define LPSYS_AON_CR1_GTIM_EN_Pos       (31U)
#define LPSYS_AON_CR1_GTIM_EN_Msk       (0x1UL << LPSYS_AON_CR1_GTIM_EN_Pos)
#define LPSYS_AON_CR1_GTIM_EN           LPSYS_AON_CR1_GTIM_EN_Msk

/***************** Bit definition for LPSYS_AON_CR2 register ******************/
#define LPSYS_AON_CR2_PIN8_MODE_Pos     (0U)
#define LPSYS_AON_CR2_PIN8_MODE_Msk     (0x7UL << LPSYS_AON_CR2_PIN8_MODE_Pos)
#define LPSYS_AON_CR2_PIN8_MODE         LPSYS_AON_CR2_PIN8_MODE_Msk
#define LPSYS_AON_CR2_PIN9_MODE_Pos     (3U)
#define LPSYS_AON_CR2_PIN9_MODE_Msk     (0x7UL << LPSYS_AON_CR2_PIN9_MODE_Pos)
#define LPSYS_AON_CR2_PIN9_MODE         LPSYS_AON_CR2_PIN9_MODE_Msk
#define LPSYS_AON_CR2_PIN10_MODE_Pos    (6U)
#define LPSYS_AON_CR2_PIN10_MODE_Msk    (0x7UL << LPSYS_AON_CR2_PIN10_MODE_Pos)
#define LPSYS_AON_CR2_PIN10_MODE        LPSYS_AON_CR2_PIN10_MODE_Msk
#define LPSYS_AON_CR2_PIN11_MODE_Pos    (9U)
#define LPSYS_AON_CR2_PIN11_MODE_Msk    (0x7UL << LPSYS_AON_CR2_PIN11_MODE_Pos)
#define LPSYS_AON_CR2_PIN11_MODE        LPSYS_AON_CR2_PIN11_MODE_Msk
#define LPSYS_AON_CR2_PIN12_MODE_Pos    (12U)
#define LPSYS_AON_CR2_PIN12_MODE_Msk    (0x7UL << LPSYS_AON_CR2_PIN12_MODE_Pos)
#define LPSYS_AON_CR2_PIN12_MODE        LPSYS_AON_CR2_PIN12_MODE_Msk
#define LPSYS_AON_CR2_PIN13_MODE_Pos    (15U)
#define LPSYS_AON_CR2_PIN13_MODE_Msk    (0x7UL << LPSYS_AON_CR2_PIN13_MODE_Pos)
#define LPSYS_AON_CR2_PIN13_MODE        LPSYS_AON_CR2_PIN13_MODE_Msk
#define LPSYS_AON_CR2_PIN14_MODE_Pos    (18U)
#define LPSYS_AON_CR2_PIN14_MODE_Msk    (0x7UL << LPSYS_AON_CR2_PIN14_MODE_Pos)
#define LPSYS_AON_CR2_PIN14_MODE        LPSYS_AON_CR2_PIN14_MODE_Msk
#define LPSYS_AON_CR2_PIN15_MODE_Pos    (21U)
#define LPSYS_AON_CR2_PIN15_MODE_Msk    (0x7UL << LPSYS_AON_CR2_PIN15_MODE_Pos)
#define LPSYS_AON_CR2_PIN15_MODE        LPSYS_AON_CR2_PIN15_MODE_Msk

/***************** Bit definition for LPSYS_AON_CR3 register ******************/
#define LPSYS_AON_CR3_PIN16_MODE_Pos    (0U)
#define LPSYS_AON_CR3_PIN16_MODE_Msk    (0x7UL << LPSYS_AON_CR3_PIN16_MODE_Pos)
#define LPSYS_AON_CR3_PIN16_MODE        LPSYS_AON_CR3_PIN16_MODE_Msk
#define LPSYS_AON_CR3_PIN17_MODE_Pos    (3U)
#define LPSYS_AON_CR3_PIN17_MODE_Msk    (0x7UL << LPSYS_AON_CR3_PIN17_MODE_Pos)
#define LPSYS_AON_CR3_PIN17_MODE        LPSYS_AON_CR3_PIN17_MODE_Msk
#define LPSYS_AON_CR3_PIN18_MODE_Pos    (6U)
#define LPSYS_AON_CR3_PIN18_MODE_Msk    (0x7UL << LPSYS_AON_CR3_PIN18_MODE_Pos)
#define LPSYS_AON_CR3_PIN18_MODE        LPSYS_AON_CR3_PIN18_MODE_Msk
#define LPSYS_AON_CR3_PIN19_MODE_Pos    (9U)
#define LPSYS_AON_CR3_PIN19_MODE_Msk    (0x7UL << LPSYS_AON_CR3_PIN19_MODE_Pos)
#define LPSYS_AON_CR3_PIN19_MODE        LPSYS_AON_CR3_PIN19_MODE_Msk
#define LPSYS_AON_CR3_PIN20_MODE_Pos    (12U)
#define LPSYS_AON_CR3_PIN20_MODE_Msk    (0x7UL << LPSYS_AON_CR3_PIN20_MODE_Pos)
#define LPSYS_AON_CR3_PIN20_MODE        LPSYS_AON_CR3_PIN20_MODE_Msk

/***************** Bit definition for LPSYS_AON_ACR register ******************/
#define LPSYS_AON_ACR_HRC48_REQ_Pos     (0U)
#define LPSYS_AON_ACR_HRC48_REQ_Msk     (0x1UL << LPSYS_AON_ACR_HRC48_REQ_Pos)
#define LPSYS_AON_ACR_HRC48_REQ         LPSYS_AON_ACR_HRC48_REQ_Msk
#define LPSYS_AON_ACR_HXT48_REQ_Pos     (1U)
#define LPSYS_AON_ACR_HXT48_REQ_Msk     (0x1UL << LPSYS_AON_ACR_HXT48_REQ_Pos)
#define LPSYS_AON_ACR_HXT48_REQ         LPSYS_AON_ACR_HXT48_REQ_Msk
#define LPSYS_AON_ACR_PWR_REQ_Pos       (2U)
#define LPSYS_AON_ACR_PWR_REQ_Msk       (0x1UL << LPSYS_AON_ACR_PWR_REQ_Pos)
#define LPSYS_AON_ACR_PWR_REQ           LPSYS_AON_ACR_PWR_REQ_Msk
#define LPSYS_AON_ACR_EXTPWR_REQ_Pos    (3U)
#define LPSYS_AON_ACR_EXTPWR_REQ_Msk    (0x1UL << LPSYS_AON_ACR_EXTPWR_REQ_Pos)
#define LPSYS_AON_ACR_EXTPWR_REQ        LPSYS_AON_ACR_EXTPWR_REQ_Msk
#define LPSYS_AON_ACR_HRC48_RDY_Pos     (30U)
#define LPSYS_AON_ACR_HRC48_RDY_Msk     (0x1UL << LPSYS_AON_ACR_HRC48_RDY_Pos)
#define LPSYS_AON_ACR_HRC48_RDY         LPSYS_AON_ACR_HRC48_RDY_Msk
#define LPSYS_AON_ACR_HXT48_RDY_Pos     (31U)
#define LPSYS_AON_ACR_HXT48_RDY_Msk     (0x1UL << LPSYS_AON_ACR_HXT48_RDY_Pos)
#define LPSYS_AON_ACR_HXT48_RDY         LPSYS_AON_ACR_HXT48_RDY_Msk

/***************** Bit definition for LPSYS_AON_LSCR register *****************/
#define LPSYS_AON_LSCR_HRC48_REQ_Pos    (0U)
#define LPSYS_AON_LSCR_HRC48_REQ_Msk    (0x1UL << LPSYS_AON_LSCR_HRC48_REQ_Pos)
#define LPSYS_AON_LSCR_HRC48_REQ        LPSYS_AON_LSCR_HRC48_REQ_Msk
#define LPSYS_AON_LSCR_HXT48_REQ_Pos    (1U)
#define LPSYS_AON_LSCR_HXT48_REQ_Msk    (0x1UL << LPSYS_AON_LSCR_HXT48_REQ_Pos)
#define LPSYS_AON_LSCR_HXT48_REQ        LPSYS_AON_LSCR_HXT48_REQ_Msk
#define LPSYS_AON_LSCR_PWR_REQ_Pos      (2U)
#define LPSYS_AON_LSCR_PWR_REQ_Msk      (0x1UL << LPSYS_AON_LSCR_PWR_REQ_Pos)
#define LPSYS_AON_LSCR_PWR_REQ          LPSYS_AON_LSCR_PWR_REQ_Msk
#define LPSYS_AON_LSCR_EXTPWR_REQ_Pos   (3U)
#define LPSYS_AON_LSCR_EXTPWR_REQ_Msk   (0x1UL << LPSYS_AON_LSCR_EXTPWR_REQ_Pos)
#define LPSYS_AON_LSCR_EXTPWR_REQ       LPSYS_AON_LSCR_EXTPWR_REQ_Msk

/***************** Bit definition for LPSYS_AON_DSCR register *****************/
#define LPSYS_AON_DSCR_HRC48_REQ_Pos    (0U)
#define LPSYS_AON_DSCR_HRC48_REQ_Msk    (0x1UL << LPSYS_AON_DSCR_HRC48_REQ_Pos)
#define LPSYS_AON_DSCR_HRC48_REQ        LPSYS_AON_DSCR_HRC48_REQ_Msk
#define LPSYS_AON_DSCR_HXT48_REQ_Pos    (1U)
#define LPSYS_AON_DSCR_HXT48_REQ_Msk    (0x1UL << LPSYS_AON_DSCR_HXT48_REQ_Pos)
#define LPSYS_AON_DSCR_HXT48_REQ        LPSYS_AON_DSCR_HXT48_REQ_Msk
#define LPSYS_AON_DSCR_PWR_REQ_Pos      (2U)
#define LPSYS_AON_DSCR_PWR_REQ_Msk      (0x1UL << LPSYS_AON_DSCR_PWR_REQ_Pos)
#define LPSYS_AON_DSCR_PWR_REQ          LPSYS_AON_DSCR_PWR_REQ_Msk
#define LPSYS_AON_DSCR_EXTPWR_REQ_Pos   (3U)
#define LPSYS_AON_DSCR_EXTPWR_REQ_Msk   (0x1UL << LPSYS_AON_DSCR_EXTPWR_REQ_Pos)
#define LPSYS_AON_DSCR_EXTPWR_REQ       LPSYS_AON_DSCR_EXTPWR_REQ_Msk

/***************** Bit definition for LPSYS_AON_SBCR register *****************/
#define LPSYS_AON_SBCR_HRC48_REQ_Pos    (0U)
#define LPSYS_AON_SBCR_HRC48_REQ_Msk    (0x1UL << LPSYS_AON_SBCR_HRC48_REQ_Pos)
#define LPSYS_AON_SBCR_HRC48_REQ        LPSYS_AON_SBCR_HRC48_REQ_Msk
#define LPSYS_AON_SBCR_HXT48_REQ_Pos    (1U)
#define LPSYS_AON_SBCR_HXT48_REQ_Msk    (0x1UL << LPSYS_AON_SBCR_HXT48_REQ_Pos)
#define LPSYS_AON_SBCR_HXT48_REQ        LPSYS_AON_SBCR_HXT48_REQ_Msk
#define LPSYS_AON_SBCR_PWR_REQ_Pos      (2U)
#define LPSYS_AON_SBCR_PWR_REQ_Msk      (0x1UL << LPSYS_AON_SBCR_PWR_REQ_Pos)
#define LPSYS_AON_SBCR_PWR_REQ          LPSYS_AON_SBCR_PWR_REQ_Msk
#define LPSYS_AON_SBCR_EXTPWR_REQ_Pos   (3U)
#define LPSYS_AON_SBCR_EXTPWR_REQ_Msk   (0x1UL << LPSYS_AON_SBCR_EXTPWR_REQ_Pos)
#define LPSYS_AON_SBCR_EXTPWR_REQ       LPSYS_AON_SBCR_EXTPWR_REQ_Msk
#define LPSYS_AON_SBCR_PD_RAM0_Pos      (6U)
#define LPSYS_AON_SBCR_PD_RAM0_Msk      (0x1UL << LPSYS_AON_SBCR_PD_RAM0_Pos)
#define LPSYS_AON_SBCR_PD_RAM0          LPSYS_AON_SBCR_PD_RAM0_Msk
#define LPSYS_AON_SBCR_PD_RAM1_Pos      (7U)
#define LPSYS_AON_SBCR_PD_RAM1_Msk      (0x1UL << LPSYS_AON_SBCR_PD_RAM1_Pos)
#define LPSYS_AON_SBCR_PD_RAM1          LPSYS_AON_SBCR_PD_RAM1_Msk

/***************** Bit definition for LPSYS_AON_WER register ******************/
#define LPSYS_AON_WER_RTC_Pos           (0U)
#define LPSYS_AON_WER_RTC_Msk           (0x1UL << LPSYS_AON_WER_RTC_Pos)
#define LPSYS_AON_WER_RTC               LPSYS_AON_WER_RTC_Msk
#define LPSYS_AON_WER_GPIO2_Pos         (1U)
#define LPSYS_AON_WER_GPIO2_Msk         (0x1UL << LPSYS_AON_WER_GPIO2_Pos)
#define LPSYS_AON_WER_GPIO2             LPSYS_AON_WER_GPIO2_Msk
#define LPSYS_AON_WER_LPTIM3_Pos        (2U)
#define LPSYS_AON_WER_LPTIM3_Msk        (0x1UL << LPSYS_AON_WER_LPTIM3_Pos)
#define LPSYS_AON_WER_LPTIM3            LPSYS_AON_WER_LPTIM3_Msk
#define LPSYS_AON_WER_BT_Pos            (5U)
#define LPSYS_AON_WER_BT_Msk            (0x1UL << LPSYS_AON_WER_BT_Pos)
#define LPSYS_AON_WER_BT                LPSYS_AON_WER_BT_Msk
#define LPSYS_AON_WER_HP2LP_REQ_Pos     (6U)
#define LPSYS_AON_WER_HP2LP_REQ_Msk     (0x1UL << LPSYS_AON_WER_HP2LP_REQ_Pos)
#define LPSYS_AON_WER_HP2LP_REQ         LPSYS_AON_WER_HP2LP_REQ_Msk
#define LPSYS_AON_WER_HP2LP_IRQ_Pos     (7U)
#define LPSYS_AON_WER_HP2LP_IRQ_Msk     (0x1UL << LPSYS_AON_WER_HP2LP_IRQ_Pos)
#define LPSYS_AON_WER_HP2LP_IRQ         LPSYS_AON_WER_HP2LP_IRQ_Msk
#define LPSYS_AON_WER_PIN0_Pos          (8U)
#define LPSYS_AON_WER_PIN0_Msk          (0x1UL << LPSYS_AON_WER_PIN0_Pos)
#define LPSYS_AON_WER_PIN0              LPSYS_AON_WER_PIN0_Msk
#define LPSYS_AON_WER_PIN1_Pos          (9U)
#define LPSYS_AON_WER_PIN1_Msk          (0x1UL << LPSYS_AON_WER_PIN1_Pos)
#define LPSYS_AON_WER_PIN1              LPSYS_AON_WER_PIN1_Msk
#define LPSYS_AON_WER_PIN2_Pos          (10U)
#define LPSYS_AON_WER_PIN2_Msk          (0x1UL << LPSYS_AON_WER_PIN2_Pos)
#define LPSYS_AON_WER_PIN2              LPSYS_AON_WER_PIN2_Msk
#define LPSYS_AON_WER_PIN3_Pos          (11U)
#define LPSYS_AON_WER_PIN3_Msk          (0x1UL << LPSYS_AON_WER_PIN3_Pos)
#define LPSYS_AON_WER_PIN3              LPSYS_AON_WER_PIN3_Msk
#define LPSYS_AON_WER_PIN4_Pos          (12U)
#define LPSYS_AON_WER_PIN4_Msk          (0x1UL << LPSYS_AON_WER_PIN4_Pos)
#define LPSYS_AON_WER_PIN4              LPSYS_AON_WER_PIN4_Msk
#define LPSYS_AON_WER_PIN5_Pos          (13U)
#define LPSYS_AON_WER_PIN5_Msk          (0x1UL << LPSYS_AON_WER_PIN5_Pos)
#define LPSYS_AON_WER_PIN5              LPSYS_AON_WER_PIN5_Msk
#define LPSYS_AON_WER_PIN6_Pos          (14U)
#define LPSYS_AON_WER_PIN6_Msk          (0x1UL << LPSYS_AON_WER_PIN6_Pos)
#define LPSYS_AON_WER_PIN6              LPSYS_AON_WER_PIN6_Msk
#define LPSYS_AON_WER_PIN7_Pos          (15U)
#define LPSYS_AON_WER_PIN7_Msk          (0x1UL << LPSYS_AON_WER_PIN7_Pos)
#define LPSYS_AON_WER_PIN7              LPSYS_AON_WER_PIN7_Msk
#define LPSYS_AON_WER_PIN8_Pos          (16U)
#define LPSYS_AON_WER_PIN8_Msk          (0x1UL << LPSYS_AON_WER_PIN8_Pos)
#define LPSYS_AON_WER_PIN8              LPSYS_AON_WER_PIN8_Msk
#define LPSYS_AON_WER_PIN9_Pos          (17U)
#define LPSYS_AON_WER_PIN9_Msk          (0x1UL << LPSYS_AON_WER_PIN9_Pos)
#define LPSYS_AON_WER_PIN9              LPSYS_AON_WER_PIN9_Msk
#define LPSYS_AON_WER_PIN10_Pos         (18U)
#define LPSYS_AON_WER_PIN10_Msk         (0x1UL << LPSYS_AON_WER_PIN10_Pos)
#define LPSYS_AON_WER_PIN10             LPSYS_AON_WER_PIN10_Msk
#define LPSYS_AON_WER_PIN11_Pos         (19U)
#define LPSYS_AON_WER_PIN11_Msk         (0x1UL << LPSYS_AON_WER_PIN11_Pos)
#define LPSYS_AON_WER_PIN11             LPSYS_AON_WER_PIN11_Msk
#define LPSYS_AON_WER_PIN12_Pos         (20U)
#define LPSYS_AON_WER_PIN12_Msk         (0x1UL << LPSYS_AON_WER_PIN12_Pos)
#define LPSYS_AON_WER_PIN12             LPSYS_AON_WER_PIN12_Msk
#define LPSYS_AON_WER_PIN13_Pos         (21U)
#define LPSYS_AON_WER_PIN13_Msk         (0x1UL << LPSYS_AON_WER_PIN13_Pos)
#define LPSYS_AON_WER_PIN13             LPSYS_AON_WER_PIN13_Msk
#define LPSYS_AON_WER_PIN14_Pos         (22U)
#define LPSYS_AON_WER_PIN14_Msk         (0x1UL << LPSYS_AON_WER_PIN14_Pos)
#define LPSYS_AON_WER_PIN14             LPSYS_AON_WER_PIN14_Msk
#define LPSYS_AON_WER_PIN15_Pos         (23U)
#define LPSYS_AON_WER_PIN15_Msk         (0x1UL << LPSYS_AON_WER_PIN15_Pos)
#define LPSYS_AON_WER_PIN15             LPSYS_AON_WER_PIN15_Msk
#define LPSYS_AON_WER_PIN16_Pos         (24U)
#define LPSYS_AON_WER_PIN16_Msk         (0x1UL << LPSYS_AON_WER_PIN16_Pos)
#define LPSYS_AON_WER_PIN16             LPSYS_AON_WER_PIN16_Msk
#define LPSYS_AON_WER_PIN17_Pos         (25U)
#define LPSYS_AON_WER_PIN17_Msk         (0x1UL << LPSYS_AON_WER_PIN17_Pos)
#define LPSYS_AON_WER_PIN17             LPSYS_AON_WER_PIN17_Msk
#define LPSYS_AON_WER_PIN18_Pos         (26U)
#define LPSYS_AON_WER_PIN18_Msk         (0x1UL << LPSYS_AON_WER_PIN18_Pos)
#define LPSYS_AON_WER_PIN18             LPSYS_AON_WER_PIN18_Msk
#define LPSYS_AON_WER_PIN19_Pos         (27U)
#define LPSYS_AON_WER_PIN19_Msk         (0x1UL << LPSYS_AON_WER_PIN19_Pos)
#define LPSYS_AON_WER_PIN19             LPSYS_AON_WER_PIN19_Msk
#define LPSYS_AON_WER_PIN20_Pos         (28U)
#define LPSYS_AON_WER_PIN20_Msk         (0x1UL << LPSYS_AON_WER_PIN20_Pos)
#define LPSYS_AON_WER_PIN20             LPSYS_AON_WER_PIN20_Msk

/***************** Bit definition for LPSYS_AON_WSR register ******************/
#define LPSYS_AON_WSR_RTC_Pos           (0U)
#define LPSYS_AON_WSR_RTC_Msk           (0x1UL << LPSYS_AON_WSR_RTC_Pos)
#define LPSYS_AON_WSR_RTC               LPSYS_AON_WSR_RTC_Msk
#define LPSYS_AON_WSR_GPIO2_Pos         (1U)
#define LPSYS_AON_WSR_GPIO2_Msk         (0x1UL << LPSYS_AON_WSR_GPIO2_Pos)
#define LPSYS_AON_WSR_GPIO2             LPSYS_AON_WSR_GPIO2_Msk
#define LPSYS_AON_WSR_LPTIM3_Pos        (2U)
#define LPSYS_AON_WSR_LPTIM3_Msk        (0x1UL << LPSYS_AON_WSR_LPTIM3_Pos)
#define LPSYS_AON_WSR_LPTIM3            LPSYS_AON_WSR_LPTIM3_Msk
#define LPSYS_AON_WSR_BT_Pos            (5U)
#define LPSYS_AON_WSR_BT_Msk            (0x1UL << LPSYS_AON_WSR_BT_Pos)
#define LPSYS_AON_WSR_BT                LPSYS_AON_WSR_BT_Msk
#define LPSYS_AON_WSR_HP2LP_REQ_Pos     (6U)
#define LPSYS_AON_WSR_HP2LP_REQ_Msk     (0x1UL << LPSYS_AON_WSR_HP2LP_REQ_Pos)
#define LPSYS_AON_WSR_HP2LP_REQ         LPSYS_AON_WSR_HP2LP_REQ_Msk
#define LPSYS_AON_WSR_HP2LP_IRQ_Pos     (7U)
#define LPSYS_AON_WSR_HP2LP_IRQ_Msk     (0x1UL << LPSYS_AON_WSR_HP2LP_IRQ_Pos)
#define LPSYS_AON_WSR_HP2LP_IRQ         LPSYS_AON_WSR_HP2LP_IRQ_Msk
#define LPSYS_AON_WSR_PIN0_Pos          (8U)
#define LPSYS_AON_WSR_PIN0_Msk          (0x1UL << LPSYS_AON_WSR_PIN0_Pos)
#define LPSYS_AON_WSR_PIN0              LPSYS_AON_WSR_PIN0_Msk
#define LPSYS_AON_WSR_PIN1_Pos          (9U)
#define LPSYS_AON_WSR_PIN1_Msk          (0x1UL << LPSYS_AON_WSR_PIN1_Pos)
#define LPSYS_AON_WSR_PIN1              LPSYS_AON_WSR_PIN1_Msk
#define LPSYS_AON_WSR_PIN2_Pos          (10U)
#define LPSYS_AON_WSR_PIN2_Msk          (0x1UL << LPSYS_AON_WSR_PIN2_Pos)
#define LPSYS_AON_WSR_PIN2              LPSYS_AON_WSR_PIN2_Msk
#define LPSYS_AON_WSR_PIN3_Pos          (11U)
#define LPSYS_AON_WSR_PIN3_Msk          (0x1UL << LPSYS_AON_WSR_PIN3_Pos)
#define LPSYS_AON_WSR_PIN3              LPSYS_AON_WSR_PIN3_Msk
#define LPSYS_AON_WSR_PIN4_Pos          (12U)
#define LPSYS_AON_WSR_PIN4_Msk          (0x1UL << LPSYS_AON_WSR_PIN4_Pos)
#define LPSYS_AON_WSR_PIN4              LPSYS_AON_WSR_PIN4_Msk
#define LPSYS_AON_WSR_PIN5_Pos          (13U)
#define LPSYS_AON_WSR_PIN5_Msk          (0x1UL << LPSYS_AON_WSR_PIN5_Pos)
#define LPSYS_AON_WSR_PIN5              LPSYS_AON_WSR_PIN5_Msk
#define LPSYS_AON_WSR_PIN6_Pos          (14U)
#define LPSYS_AON_WSR_PIN6_Msk          (0x1UL << LPSYS_AON_WSR_PIN6_Pos)
#define LPSYS_AON_WSR_PIN6              LPSYS_AON_WSR_PIN6_Msk
#define LPSYS_AON_WSR_PIN7_Pos          (15U)
#define LPSYS_AON_WSR_PIN7_Msk          (0x1UL << LPSYS_AON_WSR_PIN7_Pos)
#define LPSYS_AON_WSR_PIN7              LPSYS_AON_WSR_PIN7_Msk
#define LPSYS_AON_WSR_PIN8_Pos          (16U)
#define LPSYS_AON_WSR_PIN8_Msk          (0x1UL << LPSYS_AON_WSR_PIN8_Pos)
#define LPSYS_AON_WSR_PIN8              LPSYS_AON_WSR_PIN8_Msk
#define LPSYS_AON_WSR_PIN9_Pos          (17U)
#define LPSYS_AON_WSR_PIN9_Msk          (0x1UL << LPSYS_AON_WSR_PIN9_Pos)
#define LPSYS_AON_WSR_PIN9              LPSYS_AON_WSR_PIN9_Msk
#define LPSYS_AON_WSR_PIN10_Pos         (18U)
#define LPSYS_AON_WSR_PIN10_Msk         (0x1UL << LPSYS_AON_WSR_PIN10_Pos)
#define LPSYS_AON_WSR_PIN10             LPSYS_AON_WSR_PIN10_Msk
#define LPSYS_AON_WSR_PIN11_Pos         (19U)
#define LPSYS_AON_WSR_PIN11_Msk         (0x1UL << LPSYS_AON_WSR_PIN11_Pos)
#define LPSYS_AON_WSR_PIN11             LPSYS_AON_WSR_PIN11_Msk
#define LPSYS_AON_WSR_PIN12_Pos         (20U)
#define LPSYS_AON_WSR_PIN12_Msk         (0x1UL << LPSYS_AON_WSR_PIN12_Pos)
#define LPSYS_AON_WSR_PIN12             LPSYS_AON_WSR_PIN12_Msk
#define LPSYS_AON_WSR_PIN13_Pos         (21U)
#define LPSYS_AON_WSR_PIN13_Msk         (0x1UL << LPSYS_AON_WSR_PIN13_Pos)
#define LPSYS_AON_WSR_PIN13             LPSYS_AON_WSR_PIN13_Msk
#define LPSYS_AON_WSR_PIN14_Pos         (22U)
#define LPSYS_AON_WSR_PIN14_Msk         (0x1UL << LPSYS_AON_WSR_PIN14_Pos)
#define LPSYS_AON_WSR_PIN14             LPSYS_AON_WSR_PIN14_Msk
#define LPSYS_AON_WSR_PIN15_Pos         (23U)
#define LPSYS_AON_WSR_PIN15_Msk         (0x1UL << LPSYS_AON_WSR_PIN15_Pos)
#define LPSYS_AON_WSR_PIN15             LPSYS_AON_WSR_PIN15_Msk
#define LPSYS_AON_WSR_PIN16_Pos         (24U)
#define LPSYS_AON_WSR_PIN16_Msk         (0x1UL << LPSYS_AON_WSR_PIN16_Pos)
#define LPSYS_AON_WSR_PIN16             LPSYS_AON_WSR_PIN16_Msk
#define LPSYS_AON_WSR_PIN17_Pos         (25U)
#define LPSYS_AON_WSR_PIN17_Msk         (0x1UL << LPSYS_AON_WSR_PIN17_Pos)
#define LPSYS_AON_WSR_PIN17             LPSYS_AON_WSR_PIN17_Msk
#define LPSYS_AON_WSR_PIN18_Pos         (26U)
#define LPSYS_AON_WSR_PIN18_Msk         (0x1UL << LPSYS_AON_WSR_PIN18_Pos)
#define LPSYS_AON_WSR_PIN18             LPSYS_AON_WSR_PIN18_Msk
#define LPSYS_AON_WSR_PIN19_Pos         (27U)
#define LPSYS_AON_WSR_PIN19_Msk         (0x1UL << LPSYS_AON_WSR_PIN19_Pos)
#define LPSYS_AON_WSR_PIN19             LPSYS_AON_WSR_PIN19_Msk
#define LPSYS_AON_WSR_PIN20_Pos         (28U)
#define LPSYS_AON_WSR_PIN20_Msk         (0x1UL << LPSYS_AON_WSR_PIN20_Pos)
#define LPSYS_AON_WSR_PIN20             LPSYS_AON_WSR_PIN20_Msk
#define LPSYS_AON_WSR_PIN_ALL           (LPSYS_AON_WSR_PIN0 | LPSYS_AON_WSR_PIN1 | LPSYS_AON_WSR_PIN2 | LPSYS_AON_WSR_PIN3      \
                                         | LPSYS_AON_WSR_PIN4 | LPSYS_AON_WSR_PIN5 | LPSYS_AON_WSR_PIN6 | LPSYS_AON_WSR_PIN7    \
                                         | LPSYS_AON_WSR_PIN8 | LPSYS_AON_WSR_PIN9 | LPSYS_AON_WSR_PIN10 | LPSYS_AON_WSR_PIN11  \
                                         | LPSYS_AON_WSR_PIN12 | LPSYS_AON_WSR_PIN13 | LPSYS_AON_WSR_PIN14 | LPSYS_AON_WSR_PIN15 \
                                         | LPSYS_AON_WSR_PIN16 | LPSYS_AON_WSR_PIN17 | LPSYS_AON_WSR_PIN18 | LPSYS_AON_WSR_PIN19 \
                                         | LPSYS_AON_WSR_PIN20)
#define LPSYS_AON_WSR_PIN_NUM           (21)


/***************** Bit definition for LPSYS_AON_WCR register ******************/
#define LPSYS_AON_WCR_PIN0_Pos          (8U)
#define LPSYS_AON_WCR_PIN0_Msk          (0x1UL << LPSYS_AON_WCR_PIN0_Pos)
#define LPSYS_AON_WCR_PIN0              LPSYS_AON_WCR_PIN0_Msk
#define LPSYS_AON_WCR_PIN1_Pos          (9U)
#define LPSYS_AON_WCR_PIN1_Msk          (0x1UL << LPSYS_AON_WCR_PIN1_Pos)
#define LPSYS_AON_WCR_PIN1              LPSYS_AON_WCR_PIN1_Msk
#define LPSYS_AON_WCR_PIN2_Pos          (10U)
#define LPSYS_AON_WCR_PIN2_Msk          (0x1UL << LPSYS_AON_WCR_PIN2_Pos)
#define LPSYS_AON_WCR_PIN2              LPSYS_AON_WCR_PIN2_Msk
#define LPSYS_AON_WCR_PIN3_Pos          (11U)
#define LPSYS_AON_WCR_PIN3_Msk          (0x1UL << LPSYS_AON_WCR_PIN3_Pos)
#define LPSYS_AON_WCR_PIN3              LPSYS_AON_WCR_PIN3_Msk
#define LPSYS_AON_WCR_PIN4_Pos          (12U)
#define LPSYS_AON_WCR_PIN4_Msk          (0x1UL << LPSYS_AON_WCR_PIN4_Pos)
#define LPSYS_AON_WCR_PIN4              LPSYS_AON_WCR_PIN4_Msk
#define LPSYS_AON_WCR_PIN5_Pos          (13U)
#define LPSYS_AON_WCR_PIN5_Msk          (0x1UL << LPSYS_AON_WCR_PIN5_Pos)
#define LPSYS_AON_WCR_PIN5              LPSYS_AON_WCR_PIN5_Msk
#define LPSYS_AON_WCR_PIN6_Pos          (14U)
#define LPSYS_AON_WCR_PIN6_Msk          (0x1UL << LPSYS_AON_WCR_PIN6_Pos)
#define LPSYS_AON_WCR_PIN6              LPSYS_AON_WCR_PIN6_Msk
#define LPSYS_AON_WCR_PIN7_Pos          (15U)
#define LPSYS_AON_WCR_PIN7_Msk          (0x1UL << LPSYS_AON_WCR_PIN7_Pos)
#define LPSYS_AON_WCR_PIN7              LPSYS_AON_WCR_PIN7_Msk
#define LPSYS_AON_WCR_PIN8_Pos          (16U)
#define LPSYS_AON_WCR_PIN8_Msk          (0x1UL << LPSYS_AON_WCR_PIN8_Pos)
#define LPSYS_AON_WCR_PIN8              LPSYS_AON_WCR_PIN8_Msk
#define LPSYS_AON_WCR_PIN9_Pos          (17U)
#define LPSYS_AON_WCR_PIN9_Msk          (0x1UL << LPSYS_AON_WCR_PIN9_Pos)
#define LPSYS_AON_WCR_PIN9              LPSYS_AON_WCR_PIN9_Msk
#define LPSYS_AON_WCR_PIN10_Pos         (18U)
#define LPSYS_AON_WCR_PIN10_Msk         (0x1UL << LPSYS_AON_WCR_PIN10_Pos)
#define LPSYS_AON_WCR_PIN10             LPSYS_AON_WCR_PIN10_Msk
#define LPSYS_AON_WCR_PIN11_Pos         (19U)
#define LPSYS_AON_WCR_PIN11_Msk         (0x1UL << LPSYS_AON_WCR_PIN11_Pos)
#define LPSYS_AON_WCR_PIN11             LPSYS_AON_WCR_PIN11_Msk
#define LPSYS_AON_WCR_PIN12_Pos         (20U)
#define LPSYS_AON_WCR_PIN12_Msk         (0x1UL << LPSYS_AON_WCR_PIN12_Pos)
#define LPSYS_AON_WCR_PIN12             LPSYS_AON_WCR_PIN12_Msk
#define LPSYS_AON_WCR_PIN13_Pos         (21U)
#define LPSYS_AON_WCR_PIN13_Msk         (0x1UL << LPSYS_AON_WCR_PIN13_Pos)
#define LPSYS_AON_WCR_PIN13             LPSYS_AON_WCR_PIN13_Msk
#define LPSYS_AON_WCR_PIN14_Pos         (22U)
#define LPSYS_AON_WCR_PIN14_Msk         (0x1UL << LPSYS_AON_WCR_PIN14_Pos)
#define LPSYS_AON_WCR_PIN14             LPSYS_AON_WCR_PIN14_Msk
#define LPSYS_AON_WCR_PIN15_Pos         (23U)
#define LPSYS_AON_WCR_PIN15_Msk         (0x1UL << LPSYS_AON_WCR_PIN15_Pos)
#define LPSYS_AON_WCR_PIN15             LPSYS_AON_WCR_PIN15_Msk
#define LPSYS_AON_WCR_PIN16_Pos         (24U)
#define LPSYS_AON_WCR_PIN16_Msk         (0x1UL << LPSYS_AON_WCR_PIN16_Pos)
#define LPSYS_AON_WCR_PIN16             LPSYS_AON_WCR_PIN16_Msk
#define LPSYS_AON_WCR_PIN17_Pos         (25U)
#define LPSYS_AON_WCR_PIN17_Msk         (0x1UL << LPSYS_AON_WCR_PIN17_Pos)
#define LPSYS_AON_WCR_PIN17             LPSYS_AON_WCR_PIN17_Msk
#define LPSYS_AON_WCR_PIN18_Pos         (26U)
#define LPSYS_AON_WCR_PIN18_Msk         (0x1UL << LPSYS_AON_WCR_PIN18_Pos)
#define LPSYS_AON_WCR_PIN18             LPSYS_AON_WCR_PIN18_Msk
#define LPSYS_AON_WCR_PIN19_Pos         (27U)
#define LPSYS_AON_WCR_PIN19_Msk         (0x1UL << LPSYS_AON_WCR_PIN19_Pos)
#define LPSYS_AON_WCR_PIN19             LPSYS_AON_WCR_PIN19_Msk
#define LPSYS_AON_WCR_PIN20_Pos         (28U)
#define LPSYS_AON_WCR_PIN20_Msk         (0x1UL << LPSYS_AON_WCR_PIN20_Pos)
#define LPSYS_AON_WCR_PIN20             LPSYS_AON_WCR_PIN20_Msk
#define LPSYS_AON_WCR_AON_Pos           (31U)
#define LPSYS_AON_WCR_AON_Msk           (0x1UL << LPSYS_AON_WCR_AON_Pos)
#define LPSYS_AON_WCR_AON               LPSYS_AON_WCR_AON_Msk

/***************** Bit definition for LPSYS_AON_ISSR register *****************/
#define LPSYS_AON_ISSR_LP2HP_REQ_Pos    (0U)
#define LPSYS_AON_ISSR_LP2HP_REQ_Msk    (0x1UL << LPSYS_AON_ISSR_LP2HP_REQ_Pos)
#define LPSYS_AON_ISSR_LP2HP_REQ        LPSYS_AON_ISSR_LP2HP_REQ_Msk
#define LPSYS_AON_ISSR_HP2LP_REQ_Pos    (1U)
#define LPSYS_AON_ISSR_HP2LP_REQ_Msk    (0x1UL << LPSYS_AON_ISSR_HP2LP_REQ_Pos)
#define LPSYS_AON_ISSR_HP2LP_REQ        LPSYS_AON_ISSR_HP2LP_REQ_Msk
#define LPSYS_AON_ISSR_LP_ACTIVE_Pos    (4U)
#define LPSYS_AON_ISSR_LP_ACTIVE_Msk    (0x1UL << LPSYS_AON_ISSR_LP_ACTIVE_Pos)
#define LPSYS_AON_ISSR_LP_ACTIVE        LPSYS_AON_ISSR_LP_ACTIVE_Msk
#define LPSYS_AON_ISSR_HP_ACTIVE_Pos    (5U)
#define LPSYS_AON_ISSR_HP_ACTIVE_Msk    (0x1UL << LPSYS_AON_ISSR_HP_ACTIVE_Pos)
#define LPSYS_AON_ISSR_HP_ACTIVE        LPSYS_AON_ISSR_HP_ACTIVE_Msk

/**************** Bit definition for LPSYS_AON_TARGET register ****************/
#define LPSYS_AON_TARGET_SLEEP_TARGET_Pos  (0U)
#define LPSYS_AON_TARGET_SLEEP_TARGET_Msk  (0xFFFFFFFUL << LPSYS_AON_TARGET_SLEEP_TARGET_Pos)
#define LPSYS_AON_TARGET_SLEEP_TARGET   LPSYS_AON_TARGET_SLEEP_TARGET_Msk

/**************** Bit definition for LPSYS_AON_ACTUAL register ****************/
#define LPSYS_AON_ACTUAL_SLEEP_CNT_Pos  (0U)
#define LPSYS_AON_ACTUAL_SLEEP_CNT_Msk  (0xFFFFFFFUL << LPSYS_AON_ACTUAL_SLEEP_CNT_Pos)
#define LPSYS_AON_ACTUAL_SLEEP_CNT      LPSYS_AON_ACTUAL_SLEEP_CNT_Msk

/*************** Bit definition for LPSYS_AON_PRE_WKUP register ***************/
#define LPSYS_AON_PRE_WKUP_XTAL_TIME_Pos  (0U)
#define LPSYS_AON_PRE_WKUP_XTAL_TIME_Msk  (0x3FFUL << LPSYS_AON_PRE_WKUP_XTAL_TIME_Pos)
#define LPSYS_AON_PRE_WKUP_XTAL_TIME    LPSYS_AON_PRE_WKUP_XTAL_TIME_Msk
#define LPSYS_AON_PRE_WKUP_WKUP_TIME_Pos  (16U)
#define LPSYS_AON_PRE_WKUP_WKUP_TIME_Msk  (0x3FFUL << LPSYS_AON_PRE_WKUP_WKUP_TIME_Pos)
#define LPSYS_AON_PRE_WKUP_WKUP_TIME    LPSYS_AON_PRE_WKUP_WKUP_TIME_Msk

/*************** Bit definition for LPSYS_AON_SLP_CFG register ****************/
#define LPSYS_AON_SLP_CFG_XTAL_ALWAYS_ON_Pos  (2U)
#define LPSYS_AON_SLP_CFG_XTAL_ALWAYS_ON_Msk  (0x1UL << LPSYS_AON_SLP_CFG_XTAL_ALWAYS_ON_Pos)
#define LPSYS_AON_SLP_CFG_XTAL_ALWAYS_ON  LPSYS_AON_SLP_CFG_XTAL_ALWAYS_ON_Msk
#define LPSYS_AON_SLP_CFG_XTAL_FORCE_OFF_Pos  (3U)
#define LPSYS_AON_SLP_CFG_XTAL_FORCE_OFF_Msk  (0x1UL << LPSYS_AON_SLP_CFG_XTAL_FORCE_OFF_Pos)
#define LPSYS_AON_SLP_CFG_XTAL_FORCE_OFF  LPSYS_AON_SLP_CFG_XTAL_FORCE_OFF_Msk

/*************** Bit definition for LPSYS_AON_SLP_CTRL register ***************/
#define LPSYS_AON_SLP_CTRL_SLEEP_REQ_Pos  (0U)
#define LPSYS_AON_SLP_CTRL_SLEEP_REQ_Msk  (0x1UL << LPSYS_AON_SLP_CTRL_SLEEP_REQ_Pos)
#define LPSYS_AON_SLP_CTRL_SLEEP_REQ    LPSYS_AON_SLP_CTRL_SLEEP_REQ_Msk
#define LPSYS_AON_SLP_CTRL_WKUP_REQ_Pos  (1U)
#define LPSYS_AON_SLP_CTRL_WKUP_REQ_Msk  (0x1UL << LPSYS_AON_SLP_CTRL_WKUP_REQ_Pos)
#define LPSYS_AON_SLP_CTRL_WKUP_REQ     LPSYS_AON_SLP_CTRL_WKUP_REQ_Msk
#define LPSYS_AON_SLP_CTRL_SLEEP_STATUS_Pos  (4U)
#define LPSYS_AON_SLP_CTRL_SLEEP_STATUS_Msk  (0x1UL << LPSYS_AON_SLP_CTRL_SLEEP_STATUS_Pos)
#define LPSYS_AON_SLP_CTRL_SLEEP_STATUS  LPSYS_AON_SLP_CTRL_SLEEP_STATUS_Msk
#define LPSYS_AON_SLP_CTRL_XTAL_REQ_Pos  (5U)
#define LPSYS_AON_SLP_CTRL_XTAL_REQ_Msk  (0x1UL << LPSYS_AON_SLP_CTRL_XTAL_REQ_Pos)
#define LPSYS_AON_SLP_CTRL_XTAL_REQ     LPSYS_AON_SLP_CTRL_XTAL_REQ_Msk
#define LPSYS_AON_SLP_CTRL_BT_WKUP_Pos  (6U)
#define LPSYS_AON_SLP_CTRL_BT_WKUP_Msk  (0x1UL << LPSYS_AON_SLP_CTRL_BT_WKUP_Pos)
#define LPSYS_AON_SLP_CTRL_BT_WKUP      LPSYS_AON_SLP_CTRL_BT_WKUP_Msk

/**************** Bit definition for LPSYS_AON_ANACR register *****************/
#define LPSYS_AON_ANACR_PB_ISO_Pos      (0U)
#define LPSYS_AON_ANACR_PB_ISO_Msk      (0x1UL << LPSYS_AON_ANACR_PB_ISO_Pos)
#define LPSYS_AON_ANACR_PB_ISO          LPSYS_AON_ANACR_PB_ISO_Msk
#define LPSYS_AON_ANACR_VLP_ISO_Pos     (1U)
#define LPSYS_AON_ANACR_VLP_ISO_Msk     (0x1UL << LPSYS_AON_ANACR_VLP_ISO_Pos)
#define LPSYS_AON_ANACR_VLP_ISO         LPSYS_AON_ANACR_VLP_ISO_Msk

/**************** Bit definition for LPSYS_AON_GTIMR register *****************/
#define LPSYS_AON_GTIMR_CNT_Pos         (0U)
#define LPSYS_AON_GTIMR_CNT_Msk         (0xFFFFFFFFUL << LPSYS_AON_GTIMR_CNT_Pos)
#define LPSYS_AON_GTIMR_CNT             LPSYS_AON_GTIMR_CNT_Msk

/*************** Bit definition for LPSYS_AON_RESERVE0 register ***************/
#define LPSYS_AON_RESERVE0_DATA_Pos     (0U)
#define LPSYS_AON_RESERVE0_DATA_Msk     (0xFFFFFFFFUL << LPSYS_AON_RESERVE0_DATA_Pos)
#define LPSYS_AON_RESERVE0_DATA         LPSYS_AON_RESERVE0_DATA_Msk

/*************** Bit definition for LPSYS_AON_RESERVE1 register ***************/
#define LPSYS_AON_RESERVE1_DATA_Pos     (0U)
#define LPSYS_AON_RESERVE1_DATA_Msk     (0xFFFFFFFFUL << LPSYS_AON_RESERVE1_DATA_Pos)
#define LPSYS_AON_RESERVE1_DATA         LPSYS_AON_RESERVE1_DATA_Msk

/***************** Bit definition for LPSYS_AON_SPR register ******************/
#define LPSYS_AON_SPR_SP_Pos            (0U)
#define LPSYS_AON_SPR_SP_Msk            (0xFFFFFFFFUL << LPSYS_AON_SPR_SP_Pos)
#define LPSYS_AON_SPR_SP                LPSYS_AON_SPR_SP_Msk

/***************** Bit definition for LPSYS_AON_PCR register ******************/
#define LPSYS_AON_PCR_PC_Pos            (0U)
#define LPSYS_AON_PCR_PC_Msk            (0xFFFFFFFFUL << LPSYS_AON_PCR_PC_Pos)
#define LPSYS_AON_PCR_PC                LPSYS_AON_PCR_PC_Msk

#endif
