#ifndef __WDT_H
#define __WDT_H

typedef struct
{
    __IO uint32_t WDT_CVR0;
    __IO uint32_t WDT_CVR1;
    __IO uint32_t WDT_CR;
    __IO uint32_t WDT_CCR;
    __IO uint32_t WDT_ICR;
    __IO uint32_t WDT_SR;
    __IO uint32_t WDT_WP;
    __IO uint32_t WDT_FG;
} WDT_TypeDef;


/****************** Bit definition for WDT_WDT_CVR0 register ******************/
#define WDT_WDT_CVR0_COUNT_VALUE_0_Pos  (0U)
#define WDT_WDT_CVR0_COUNT_VALUE_0_Msk  (0xFFFFFFUL << WDT_WDT_CVR0_COUNT_VALUE_0_Pos)
#define WDT_WDT_CVR0_COUNT_VALUE_0      WDT_WDT_CVR0_COUNT_VALUE_0_Msk

/****************** Bit definition for WDT_WDT_CVR1 register ******************/
#define WDT_WDT_CVR1_COUNT_VALUE_1_Pos  (0U)
#define WDT_WDT_CVR1_COUNT_VALUE_1_Msk  (0xFFFFFFUL << WDT_WDT_CVR1_COUNT_VALUE_1_Pos)
#define WDT_WDT_CVR1_COUNT_VALUE_1      WDT_WDT_CVR1_COUNT_VALUE_1_Msk

/******************* Bit definition for WDT_WDT_CR register *******************/
#define WDT_WDT_CR_RESET_LENGTH_Pos     (0U)
#define WDT_WDT_CR_RESET_LENGTH_Msk     (0x7UL << WDT_WDT_CR_RESET_LENGTH_Pos)
#define WDT_WDT_CR_RESET_LENGTH         WDT_WDT_CR_RESET_LENGTH_Msk
#define WDT_WDT_CR_RESPONSE_MODE_Pos    (4U)
#define WDT_WDT_CR_RESPONSE_MODE_Msk    (0x1UL << WDT_WDT_CR_RESPONSE_MODE_Pos)
#define WDT_WDT_CR_RESPONSE_MODE        WDT_WDT_CR_RESPONSE_MODE_Msk

/****************** Bit definition for WDT_WDT_CCR register *******************/
#define WDT_WDT_CCR_COUNTER_CONTROL_Pos  (0U)
#define WDT_WDT_CCR_COUNTER_CONTROL_Msk  (0xFFUL << WDT_WDT_CCR_COUNTER_CONTROL_Pos)
#define WDT_WDT_CCR_COUNTER_CONTROL     WDT_WDT_CCR_COUNTER_CONTROL_Msk

/****************** Bit definition for WDT_WDT_ICR register *******************/
#define WDT_WDT_ICR_INT_CLR_Pos         (0U)
#define WDT_WDT_ICR_INT_CLR_Msk         (0x1UL << WDT_WDT_ICR_INT_CLR_Pos)
#define WDT_WDT_ICR_INT_CLR             WDT_WDT_ICR_INT_CLR_Msk

/******************* Bit definition for WDT_WDT_SR register *******************/
#define WDT_WDT_SR_INT_ASSERT_Pos       (0U)
#define WDT_WDT_SR_INT_ASSERT_Msk       (0x1UL << WDT_WDT_SR_INT_ASSERT_Pos)
#define WDT_WDT_SR_INT_ASSERT           WDT_WDT_SR_INT_ASSERT_Msk
#define WDT_WDT_SR_WDT_ACTIVE_Pos       (1U)
#define WDT_WDT_SR_WDT_ACTIVE_Msk       (0x1UL << WDT_WDT_SR_WDT_ACTIVE_Pos)
#define WDT_WDT_SR_WDT_ACTIVE           WDT_WDT_SR_WDT_ACTIVE_Msk

/******************* Bit definition for WDT_WDT_WP register *******************/
#define WDT_WDT_WP_WRPT_Pos             (0U)
#define WDT_WDT_WP_WRPT_Msk             (0x7FFFFFFFUL << WDT_WDT_WP_WRPT_Pos)
#define WDT_WDT_WP_WRPT                 WDT_WDT_WP_WRPT_Msk
#define WDT_WDT_WP_WRPT_ST_Pos          (31U)
#define WDT_WDT_WP_WRPT_ST_Msk          (0x1UL << WDT_WDT_WP_WRPT_ST_Pos)
#define WDT_WDT_WP_WRPT_ST              WDT_WDT_WP_WRPT_ST_Msk

/******************* Bit definition for WDT_WDT_FG register *******************/
#define WDT_WDT_FG_RST_FG_CLR_Pos       (0U)
#define WDT_WDT_FG_RST_FG_CLR_Msk       (0x1UL << WDT_WDT_FG_RST_FG_CLR_Pos)
#define WDT_WDT_FG_RST_FG_CLR           WDT_WDT_FG_RST_FG_CLR_Msk
#define WDT_WDT_FG_RST_FG_Pos           (1U)
#define WDT_WDT_FG_RST_FG_Msk           (0x1UL << WDT_WDT_FG_RST_FG_Pos)
#define WDT_WDT_FG_RST_FG               WDT_WDT_FG_RST_FG_Msk
#define WDT_WDT_FG_SYNC_FG_CLR_Pos      (2U)
#define WDT_WDT_FG_SYNC_FG_CLR_Msk      (0x1UL << WDT_WDT_FG_SYNC_FG_CLR_Pos)
#define WDT_WDT_FG_SYNC_FG_CLR          WDT_WDT_FG_SYNC_FG_CLR_Msk
#define WDT_WDT_FG_SYNC_FG_Pos          (3U)
#define WDT_WDT_FG_SYNC_FG_Msk          (0x1UL << WDT_WDT_FG_SYNC_FG_Pos)
#define WDT_WDT_FG_SYNC_FG              WDT_WDT_FG_SYNC_FG_Msk

#endif
