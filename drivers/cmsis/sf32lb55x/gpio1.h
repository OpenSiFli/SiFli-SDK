#ifndef __GPIO1_H
#define __GPIO1_H


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
    __IO uint32_t DIR2;
    __IO uint32_t DOR2;
    __IO uint32_t DOER2;
    __IO uint32_t OESR2;
    __IO uint32_t OECR2;
    __IO uint32_t IER2;
    __IO uint32_t IESR2;
    __IO uint32_t IECR2;
    __IO uint32_t ITR2;
    __IO uint32_t ITSR2;
    __IO uint32_t ITCR2;
    __IO uint32_t IPSR2;
    __IO uint32_t IPCR2;
    __IO uint32_t ISR2;
} GPIO1_TypeDef;


/******************* Bit definition for GPIO1_DIR0 register *******************/
#define GPIO1_DIR0_IN_Pos               (0U)
#define GPIO1_DIR0_IN_Msk               (0xFFFFFFFFUL << GPIO1_DIR0_IN_Pos)
#define GPIO1_DIR0_IN                   GPIO1_DIR0_IN_Msk

/******************* Bit definition for GPIO1_DOR0 register *******************/
#define GPIO1_DOR0_OUT_Pos              (0U)
#define GPIO1_DOR0_OUT_Msk              (0xFFFFFFFFUL << GPIO1_DOR0_OUT_Pos)
#define GPIO1_DOR0_OUT                  GPIO1_DOR0_OUT_Msk

/****************** Bit definition for GPIO1_DOER0 register *******************/
#define GPIO1_DOER0_DOE_Pos             (0U)
#define GPIO1_DOER0_DOE_Msk             (0xFFFFFFFFUL << GPIO1_DOER0_DOE_Pos)
#define GPIO1_DOER0_DOE                 GPIO1_DOER0_DOE_Msk

/****************** Bit definition for GPIO1_OESR0 register *******************/
#define GPIO1_OESR0_OES_Pos             (0U)
#define GPIO1_OESR0_OES_Msk             (0xFFFFFFFFUL << GPIO1_OESR0_OES_Pos)
#define GPIO1_OESR0_OES                 GPIO1_OESR0_OES_Msk

/****************** Bit definition for GPIO1_OECR0 register *******************/
#define GPIO1_OECR0_OEC_Pos             (0U)
#define GPIO1_OECR0_OEC_Msk             (0xFFFFFFFFUL << GPIO1_OECR0_OEC_Pos)
#define GPIO1_OECR0_OEC                 GPIO1_OECR0_OEC_Msk

/******************* Bit definition for GPIO1_IER0 register *******************/
#define GPIO1_IER0_IER_Pos              (0U)
#define GPIO1_IER0_IER_Msk              (0xFFFFFFFFUL << GPIO1_IER0_IER_Pos)
#define GPIO1_IER0_IER                  GPIO1_IER0_IER_Msk

/****************** Bit definition for GPIO1_IESR0 register *******************/
#define GPIO1_IESR0_IES_Pos             (0U)
#define GPIO1_IESR0_IES_Msk             (0xFFFFFFFFUL << GPIO1_IESR0_IES_Pos)
#define GPIO1_IESR0_IES                 GPIO1_IESR0_IES_Msk

/****************** Bit definition for GPIO1_IECR0 register *******************/
#define GPIO1_IECR0_IEC_Pos             (0U)
#define GPIO1_IECR0_IEC_Msk             (0xFFFFFFFFUL << GPIO1_IECR0_IEC_Pos)
#define GPIO1_IECR0_IEC                 GPIO1_IECR0_IEC_Msk

/******************* Bit definition for GPIO1_ITR0 register *******************/
#define GPIO1_ITR0_ITR_Pos              (0U)
#define GPIO1_ITR0_ITR_Msk              (0xFFFFFFFFUL << GPIO1_ITR0_ITR_Pos)
#define GPIO1_ITR0_ITR                  GPIO1_ITR0_ITR_Msk

/****************** Bit definition for GPIO1_ITSR0 register *******************/
#define GPIO1_ITSR0_ITS_Pos             (0U)
#define GPIO1_ITSR0_ITS_Msk             (0xFFFFFFFFUL << GPIO1_ITSR0_ITS_Pos)
#define GPIO1_ITSR0_ITS                 GPIO1_ITSR0_ITS_Msk

/****************** Bit definition for GPIO1_ITCR0 register *******************/
#define GPIO1_ITCR0_ITC_Pos             (0U)
#define GPIO1_ITCR0_ITC_Msk             (0xFFFFFFFFUL << GPIO1_ITCR0_ITC_Pos)
#define GPIO1_ITCR0_ITC                 GPIO1_ITCR0_ITC_Msk

/****************** Bit definition for GPIO1_IPSR0 register *******************/
#define GPIO1_IPSR0_IPS_Pos             (0U)
#define GPIO1_IPSR0_IPS_Msk             (0xFFFFFFFFUL << GPIO1_IPSR0_IPS_Pos)
#define GPIO1_IPSR0_IPS                 GPIO1_IPSR0_IPS_Msk

/****************** Bit definition for GPIO1_IPCR0 register *******************/
#define GPIO1_IPCR0_IPC_Pos             (0U)
#define GPIO1_IPCR0_IPC_Msk             (0xFFFFFFFFUL << GPIO1_IPCR0_IPC_Pos)
#define GPIO1_IPCR0_IPC                 GPIO1_IPCR0_IPC_Msk

/******************* Bit definition for GPIO1_ISR0 register *******************/
#define GPIO1_ISR0_IS_Pos               (0U)
#define GPIO1_ISR0_IS_Msk               (0xFFFFFFFFUL << GPIO1_ISR0_IS_Pos)
#define GPIO1_ISR0_IS                   GPIO1_ISR0_IS_Msk

/******************* Bit definition for GPIO1_DIR1 register *******************/
#define GPIO1_DIR1_IN_Pos               (0U)
#define GPIO1_DIR1_IN_Msk               (0xFFFFFFFFUL << GPIO1_DIR1_IN_Pos)
#define GPIO1_DIR1_IN                   GPIO1_DIR1_IN_Msk

/******************* Bit definition for GPIO1_DOR1 register *******************/
#define GPIO1_DOR1_OUT_Pos              (0U)
#define GPIO1_DOR1_OUT_Msk              (0xFFFFFFFFUL << GPIO1_DOR1_OUT_Pos)
#define GPIO1_DOR1_OUT                  GPIO1_DOR1_OUT_Msk

/****************** Bit definition for GPIO1_DOER1 register *******************/
#define GPIO1_DOER1_DOE_Pos             (0U)
#define GPIO1_DOER1_DOE_Msk             (0xFFFFFFFFUL << GPIO1_DOER1_DOE_Pos)
#define GPIO1_DOER1_DOE                 GPIO1_DOER1_DOE_Msk

/****************** Bit definition for GPIO1_OESR1 register *******************/
#define GPIO1_OESR1_OES_Pos             (0U)
#define GPIO1_OESR1_OES_Msk             (0xFFFFFFFFUL << GPIO1_OESR1_OES_Pos)
#define GPIO1_OESR1_OES                 GPIO1_OESR1_OES_Msk

/****************** Bit definition for GPIO1_OECR1 register *******************/
#define GPIO1_OECR1_OEC_Pos             (0U)
#define GPIO1_OECR1_OEC_Msk             (0xFFFFFFFFUL << GPIO1_OECR1_OEC_Pos)
#define GPIO1_OECR1_OEC                 GPIO1_OECR1_OEC_Msk

/******************* Bit definition for GPIO1_IER1 register *******************/
#define GPIO1_IER1_IER_Pos              (0U)
#define GPIO1_IER1_IER_Msk              (0xFFFFFFFFUL << GPIO1_IER1_IER_Pos)
#define GPIO1_IER1_IER                  GPIO1_IER1_IER_Msk

/****************** Bit definition for GPIO1_IESR1 register *******************/
#define GPIO1_IESR1_IES_Pos             (0U)
#define GPIO1_IESR1_IES_Msk             (0xFFFFFFFFUL << GPIO1_IESR1_IES_Pos)
#define GPIO1_IESR1_IES                 GPIO1_IESR1_IES_Msk

/****************** Bit definition for GPIO1_IECR1 register *******************/
#define GPIO1_IECR1_IEC_Pos             (0U)
#define GPIO1_IECR1_IEC_Msk             (0xFFFFFFFFUL << GPIO1_IECR1_IEC_Pos)
#define GPIO1_IECR1_IEC                 GPIO1_IECR1_IEC_Msk

/******************* Bit definition for GPIO1_ITR1 register *******************/
#define GPIO1_ITR1_ITR_Pos              (0U)
#define GPIO1_ITR1_ITR_Msk              (0xFFFFFFFFUL << GPIO1_ITR1_ITR_Pos)
#define GPIO1_ITR1_ITR                  GPIO1_ITR1_ITR_Msk

/****************** Bit definition for GPIO1_ITSR1 register *******************/
#define GPIO1_ITSR1_ITS_Pos             (0U)
#define GPIO1_ITSR1_ITS_Msk             (0xFFFFFFFFUL << GPIO1_ITSR1_ITS_Pos)
#define GPIO1_ITSR1_ITS                 GPIO1_ITSR1_ITS_Msk

/****************** Bit definition for GPIO1_ITCR1 register *******************/
#define GPIO1_ITCR1_ITC_Pos             (0U)
#define GPIO1_ITCR1_ITC_Msk             (0xFFFFFFFFUL << GPIO1_ITCR1_ITC_Pos)
#define GPIO1_ITCR1_ITC                 GPIO1_ITCR1_ITC_Msk

/****************** Bit definition for GPIO1_IPSR1 register *******************/
#define GPIO1_IPSR1_IPS_Pos             (0U)
#define GPIO1_IPSR1_IPS_Msk             (0xFFFFFFFFUL << GPIO1_IPSR1_IPS_Pos)
#define GPIO1_IPSR1_IPS                 GPIO1_IPSR1_IPS_Msk

/****************** Bit definition for GPIO1_IPCR1 register *******************/
#define GPIO1_IPCR1_IPC_Pos             (0U)
#define GPIO1_IPCR1_IPC_Msk             (0xFFFFFFFFUL << GPIO1_IPCR1_IPC_Pos)
#define GPIO1_IPCR1_IPC                 GPIO1_IPCR1_IPC_Msk

/******************* Bit definition for GPIO1_ISR1 register *******************/
#define GPIO1_ISR1_IS_Pos               (0U)
#define GPIO1_ISR1_IS_Msk               (0xFFFFFFFFUL << GPIO1_ISR1_IS_Pos)
#define GPIO1_ISR1_IS                   GPIO1_ISR1_IS_Msk

/******************* Bit definition for GPIO1_DIR2 register *******************/
#define GPIO1_DIR2_IN_Pos               (0U)
#define GPIO1_DIR2_IN_Msk               (0xFFFFFFFFUL << GPIO1_DIR2_IN_Pos)
#define GPIO1_DIR2_IN                   GPIO1_DIR2_IN_Msk

/******************* Bit definition for GPIO1_DOR2 register *******************/
#define GPIO1_DOR2_OUT_Pos              (0U)
#define GPIO1_DOR2_OUT_Msk              (0xFFFFFFFFUL << GPIO1_DOR2_OUT_Pos)
#define GPIO1_DOR2_OUT                  GPIO1_DOR2_OUT_Msk

/****************** Bit definition for GPIO1_DOER2 register *******************/
#define GPIO1_DOER2_DOE_Pos             (0U)
#define GPIO1_DOER2_DOE_Msk             (0xFFFFFFFFUL << GPIO1_DOER2_DOE_Pos)
#define GPIO1_DOER2_DOE                 GPIO1_DOER2_DOE_Msk

/****************** Bit definition for GPIO1_OESR2 register *******************/
#define GPIO1_OESR2_OES_Pos             (0U)
#define GPIO1_OESR2_OES_Msk             (0xFFFFFFFFUL << GPIO1_OESR2_OES_Pos)
#define GPIO1_OESR2_OES                 GPIO1_OESR2_OES_Msk

/****************** Bit definition for GPIO1_OECR2 register *******************/
#define GPIO1_OECR2_OEC_Pos             (0U)
#define GPIO1_OECR2_OEC_Msk             (0xFFFFFFFFUL << GPIO1_OECR2_OEC_Pos)
#define GPIO1_OECR2_OEC                 GPIO1_OECR2_OEC_Msk

/******************* Bit definition for GPIO1_IER2 register *******************/
#define GPIO1_IER2_IER_Pos              (0U)
#define GPIO1_IER2_IER_Msk              (0xFFFFFFFFUL << GPIO1_IER2_IER_Pos)
#define GPIO1_IER2_IER                  GPIO1_IER2_IER_Msk

/****************** Bit definition for GPIO1_IESR2 register *******************/
#define GPIO1_IESR2_IES_Pos             (0U)
#define GPIO1_IESR2_IES_Msk             (0xFFFFFFFFUL << GPIO1_IESR2_IES_Pos)
#define GPIO1_IESR2_IES                 GPIO1_IESR2_IES_Msk

/****************** Bit definition for GPIO1_IECR2 register *******************/
#define GPIO1_IECR2_IEC_Pos             (0U)
#define GPIO1_IECR2_IEC_Msk             (0xFFFFFFFFUL << GPIO1_IECR2_IEC_Pos)
#define GPIO1_IECR2_IEC                 GPIO1_IECR2_IEC_Msk

/******************* Bit definition for GPIO1_ITR2 register *******************/
#define GPIO1_ITR2_ITR_Pos              (0U)
#define GPIO1_ITR2_ITR_Msk              (0xFFFFFFFFUL << GPIO1_ITR2_ITR_Pos)
#define GPIO1_ITR2_ITR                  GPIO1_ITR2_ITR_Msk

/****************** Bit definition for GPIO1_ITSR2 register *******************/
#define GPIO1_ITSR2_ITS_Pos             (0U)
#define GPIO1_ITSR2_ITS_Msk             (0xFFFFFFFFUL << GPIO1_ITSR2_ITS_Pos)
#define GPIO1_ITSR2_ITS                 GPIO1_ITSR2_ITS_Msk

/****************** Bit definition for GPIO1_ITCR2 register *******************/
#define GPIO1_ITCR2_ITC_Pos             (0U)
#define GPIO1_ITCR2_ITC_Msk             (0xFFFFFFFFUL << GPIO1_ITCR2_ITC_Pos)
#define GPIO1_ITCR2_ITC                 GPIO1_ITCR2_ITC_Msk

/****************** Bit definition for GPIO1_IPSR2 register *******************/
#define GPIO1_IPSR2_IPS_Pos             (0U)
#define GPIO1_IPSR2_IPS_Msk             (0xFFFFFFFFUL << GPIO1_IPSR2_IPS_Pos)
#define GPIO1_IPSR2_IPS                 GPIO1_IPSR2_IPS_Msk

/****************** Bit definition for GPIO1_IPCR2 register *******************/
#define GPIO1_IPCR2_IPC_Pos             (0U)
#define GPIO1_IPCR2_IPC_Msk             (0xFFFFFFFFUL << GPIO1_IPCR2_IPC_Pos)
#define GPIO1_IPCR2_IPC                 GPIO1_IPCR2_IPC_Msk

/******************* Bit definition for GPIO1_ISR2 register *******************/
#define GPIO1_ISR2_IS_Pos               (0U)
#define GPIO1_ISR2_IS_Msk               (0xFFFFFFFFUL << GPIO1_ISR2_IS_Pos)
#define GPIO1_ISR2_IS                   GPIO1_ISR2_IS_Msk

#endif
