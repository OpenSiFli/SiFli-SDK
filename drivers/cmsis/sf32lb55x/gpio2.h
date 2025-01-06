#ifndef __GPIO2_H
#define __GPIO2_H

typedef struct
{
    __IO uint32_t DIR0;
    __IO uint32_t DOR0;
    __IO uint32_t DOER0;
    __IO uint32_t OESR0;
    __IO uint32_t OECR0;
    __IO uint32_t IER0;
    __IO uint32_t IESR0;
    __IO uint32_t IECR0;
    __IO uint32_t ITR0;
    __IO uint32_t ITSR0;
    __IO uint32_t ITCR0;
    __IO uint32_t IPSR0;
    __IO uint32_t IPCR0;
    __IO uint32_t ISR0;
    __IO uint32_t DIR1;
    __IO uint32_t DOR1;
    __IO uint32_t DOER1;
    __IO uint32_t OESR1;
    __IO uint32_t OECR1;
    __IO uint32_t IER1;
    __IO uint32_t IESR1;
    __IO uint32_t IECR1;
    __IO uint32_t ITR1;
    __IO uint32_t ITSR1;
    __IO uint32_t ITCR1;
    __IO uint32_t IPSR1;
    __IO uint32_t IPCR1;
    __IO uint32_t ISR1;
} GPIO2_TypeDef;


/******************* Bit definition for GPIO2_DIR0 register *******************/
#define GPIO2_DIR0_IN_Pos               (0U)
#define GPIO2_DIR0_IN_Msk               (0xFFFFFFFFUL << GPIO2_DIR0_IN_Pos)
#define GPIO2_DIR0_IN                   GPIO2_DIR0_IN_Msk

/******************* Bit definition for GPIO2_DOR0 register *******************/
#define GPIO2_DOR0_OUT_Pos              (0U)
#define GPIO2_DOR0_OUT_Msk              (0xFFFFFFFFUL << GPIO2_DOR0_OUT_Pos)
#define GPIO2_DOR0_OUT                  GPIO2_DOR0_OUT_Msk

/****************** Bit definition for GPIO2_DOER0 register *******************/
#define GPIO2_DOER0_DOE_Pos             (0U)
#define GPIO2_DOER0_DOE_Msk             (0xFFFFFFFFUL << GPIO2_DOER0_DOE_Pos)
#define GPIO2_DOER0_DOE                 GPIO2_DOER0_DOE_Msk

/****************** Bit definition for GPIO2_OESR0 register *******************/
#define GPIO2_OESR0_OES_Pos             (0U)
#define GPIO2_OESR0_OES_Msk             (0xFFFFFFFFUL << GPIO2_OESR0_OES_Pos)
#define GPIO2_OESR0_OES                 GPIO2_OESR0_OES_Msk

/****************** Bit definition for GPIO2_OECR0 register *******************/
#define GPIO2_OECR0_OEC_Pos             (0U)
#define GPIO2_OECR0_OEC_Msk             (0xFFFFFFFFUL << GPIO2_OECR0_OEC_Pos)
#define GPIO2_OECR0_OEC                 GPIO2_OECR0_OEC_Msk

/******************* Bit definition for GPIO2_IER0 register *******************/
#define GPIO2_IER0_IER_Pos              (0U)
#define GPIO2_IER0_IER_Msk              (0xFFFFFFFFUL << GPIO2_IER0_IER_Pos)
#define GPIO2_IER0_IER                  GPIO2_IER0_IER_Msk

/****************** Bit definition for GPIO2_IESR0 register *******************/
#define GPIO2_IESR0_IES_Pos             (0U)
#define GPIO2_IESR0_IES_Msk             (0xFFFFFFFFUL << GPIO2_IESR0_IES_Pos)
#define GPIO2_IESR0_IES                 GPIO2_IESR0_IES_Msk

/****************** Bit definition for GPIO2_IECR0 register *******************/
#define GPIO2_IECR0_IEC_Pos             (0U)
#define GPIO2_IECR0_IEC_Msk             (0xFFFFFFFFUL << GPIO2_IECR0_IEC_Pos)
#define GPIO2_IECR0_IEC                 GPIO2_IECR0_IEC_Msk

/******************* Bit definition for GPIO2_ITR0 register *******************/
#define GPIO2_ITR0_ITR_Pos              (0U)
#define GPIO2_ITR0_ITR_Msk              (0xFFFFFFFFUL << GPIO2_ITR0_ITR_Pos)
#define GPIO2_ITR0_ITR                  GPIO2_ITR0_ITR_Msk

/****************** Bit definition for GPIO2_ITSR0 register *******************/
#define GPIO2_ITSR0_ITS_Pos             (0U)
#define GPIO2_ITSR0_ITS_Msk             (0xFFFFFFFFUL << GPIO2_ITSR0_ITS_Pos)
#define GPIO2_ITSR0_ITS                 GPIO2_ITSR0_ITS_Msk

/****************** Bit definition for GPIO2_ITCR0 register *******************/
#define GPIO2_ITCR0_ITC_Pos             (0U)
#define GPIO2_ITCR0_ITC_Msk             (0xFFFFFFFFUL << GPIO2_ITCR0_ITC_Pos)
#define GPIO2_ITCR0_ITC                 GPIO2_ITCR0_ITC_Msk

/****************** Bit definition for GPIO2_IPSR0 register *******************/
#define GPIO2_IPSR0_IPS_Pos             (0U)
#define GPIO2_IPSR0_IPS_Msk             (0xFFFFFFFFUL << GPIO2_IPSR0_IPS_Pos)
#define GPIO2_IPSR0_IPS                 GPIO2_IPSR0_IPS_Msk

/****************** Bit definition for GPIO2_IPCR0 register *******************/
#define GPIO2_IPCR0_IPC_Pos             (0U)
#define GPIO2_IPCR0_IPC_Msk             (0xFFFFFFFFUL << GPIO2_IPCR0_IPC_Pos)
#define GPIO2_IPCR0_IPC                 GPIO2_IPCR0_IPC_Msk

/******************* Bit definition for GPIO2_ISR0 register *******************/
#define GPIO2_ISR0_IS_Pos               (0U)
#define GPIO2_ISR0_IS_Msk               (0xFFFFFFFFUL << GPIO2_ISR0_IS_Pos)
#define GPIO2_ISR0_IS                   GPIO2_ISR0_IS_Msk

/******************* Bit definition for GPIO2_DIR1 register *******************/
#define GPIO2_DIR1_IN_Pos               (0U)
#define GPIO2_DIR1_IN_Msk               (0xFFFFFFFFUL << GPIO2_DIR1_IN_Pos)
#define GPIO2_DIR1_IN                   GPIO2_DIR1_IN_Msk

/******************* Bit definition for GPIO2_DOR1 register *******************/
#define GPIO2_DOR1_OUT_Pos              (0U)
#define GPIO2_DOR1_OUT_Msk              (0xFFFFFFFFUL << GPIO2_DOR1_OUT_Pos)
#define GPIO2_DOR1_OUT                  GPIO2_DOR1_OUT_Msk

/****************** Bit definition for GPIO2_DOER1 register *******************/
#define GPIO2_DOER1_DOE_Pos             (0U)
#define GPIO2_DOER1_DOE_Msk             (0xFFFFFFFFUL << GPIO2_DOER1_DOE_Pos)
#define GPIO2_DOER1_DOE                 GPIO2_DOER1_DOE_Msk

/****************** Bit definition for GPIO2_OESR1 register *******************/
#define GPIO2_OESR1_OES_Pos             (0U)
#define GPIO2_OESR1_OES_Msk             (0xFFFFFFFFUL << GPIO2_OESR1_OES_Pos)
#define GPIO2_OESR1_OES                 GPIO2_OESR1_OES_Msk

/****************** Bit definition for GPIO2_OECR1 register *******************/
#define GPIO2_OECR1_OEC_Pos             (0U)
#define GPIO2_OECR1_OEC_Msk             (0xFFFFFFFFUL << GPIO2_OECR1_OEC_Pos)
#define GPIO2_OECR1_OEC                 GPIO2_OECR1_OEC_Msk

/******************* Bit definition for GPIO2_IER1 register *******************/
#define GPIO2_IER1_IER_Pos              (0U)
#define GPIO2_IER1_IER_Msk              (0xFFFFFFFFUL << GPIO2_IER1_IER_Pos)
#define GPIO2_IER1_IER                  GPIO2_IER1_IER_Msk

/****************** Bit definition for GPIO2_IESR1 register *******************/
#define GPIO2_IESR1_IES_Pos             (0U)
#define GPIO2_IESR1_IES_Msk             (0xFFFFFFFFUL << GPIO2_IESR1_IES_Pos)
#define GPIO2_IESR1_IES                 GPIO2_IESR1_IES_Msk

/****************** Bit definition for GPIO2_IECR1 register *******************/
#define GPIO2_IECR1_IEC_Pos             (0U)
#define GPIO2_IECR1_IEC_Msk             (0xFFFFFFFFUL << GPIO2_IECR1_IEC_Pos)
#define GPIO2_IECR1_IEC                 GPIO2_IECR1_IEC_Msk

/******************* Bit definition for GPIO2_ITR1 register *******************/
#define GPIO2_ITR1_ITR_Pos              (0U)
#define GPIO2_ITR1_ITR_Msk              (0xFFFFFFFFUL << GPIO2_ITR1_ITR_Pos)
#define GPIO2_ITR1_ITR                  GPIO2_ITR1_ITR_Msk

/****************** Bit definition for GPIO2_ITSR1 register *******************/
#define GPIO2_ITSR1_ITS_Pos             (0U)
#define GPIO2_ITSR1_ITS_Msk             (0xFFFFFFFFUL << GPIO2_ITSR1_ITS_Pos)
#define GPIO2_ITSR1_ITS                 GPIO2_ITSR1_ITS_Msk

/****************** Bit definition for GPIO2_ITCR1 register *******************/
#define GPIO2_ITCR1_ITC_Pos             (0U)
#define GPIO2_ITCR1_ITC_Msk             (0xFFFFFFFFUL << GPIO2_ITCR1_ITC_Pos)
#define GPIO2_ITCR1_ITC                 GPIO2_ITCR1_ITC_Msk

/****************** Bit definition for GPIO2_IPSR1 register *******************/
#define GPIO2_IPSR1_IPS_Pos             (0U)
#define GPIO2_IPSR1_IPS_Msk             (0xFFFFFFFFUL << GPIO2_IPSR1_IPS_Pos)
#define GPIO2_IPSR1_IPS                 GPIO2_IPSR1_IPS_Msk

/****************** Bit definition for GPIO2_IPCR1 register *******************/
#define GPIO2_IPCR1_IPC_Pos             (0U)
#define GPIO2_IPCR1_IPC_Msk             (0xFFFFFFFFUL << GPIO2_IPCR1_IPC_Pos)
#define GPIO2_IPCR1_IPC                 GPIO2_IPCR1_IPC_Msk

/******************* Bit definition for GPIO2_ISR1 register *******************/
#define GPIO2_ISR1_IS_Pos               (0U)
#define GPIO2_ISR1_IS_Msk               (0xFFFFFFFFUL << GPIO2_ISR1_IS_Pos)
#define GPIO2_ISR1_IS                   GPIO2_ISR1_IS_Msk

#endif
