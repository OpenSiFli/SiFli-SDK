#ifndef __GPIO1_H
#define __GPIO1_H

typedef struct
{
    __IO uint32_t DIR0;
    __IO uint32_t DOR0;
    __IO uint32_t DOSR0;
    __IO uint32_t DOCR0;
    __IO uint32_t DOER0;
    __IO uint32_t DOESR0;
    __IO uint32_t DOECR0;
    __IO uint32_t IER0;
    __IO uint32_t IESR0;
    __IO uint32_t IECR0;
    __IO uint32_t ITR0;
    __IO uint32_t ITSR0;
    __IO uint32_t ITCR0;
    __IO uint32_t IPHR0;
    __IO uint32_t IPHSR0;
    __IO uint32_t IPHCR0;
    __IO uint32_t IPLR0;
    __IO uint32_t IPLSR0;
    __IO uint32_t IPLCR0;
    __IO uint32_t ISR0;
    __IO uint32_t IER0_EXT;
    __IO uint32_t IESR0_EXT;
    __IO uint32_t IECR0_EXT;
    __IO uint32_t ISR0_EXT;
    __IO uint32_t OEMR0;
    __IO uint32_t OEMSR0;
    __IO uint32_t OEMCR0;
    __IO uint32_t RSVD1[5];
    __IO uint32_t DIR1;
    __IO uint32_t DOR1;
    __IO uint32_t DOSR1;
    __IO uint32_t DOCR1;
    __IO uint32_t DOER1;
    __IO uint32_t DOESR1;
    __IO uint32_t DOECR1;
    __IO uint32_t IER1;
    __IO uint32_t IESR1;
    __IO uint32_t IECR1;
    __IO uint32_t ITR1;
    __IO uint32_t ITSR1;
    __IO uint32_t ITCR1;
    __IO uint32_t IPHR1;
    __IO uint32_t IPHSR1;
    __IO uint32_t IPHCR1;
    __IO uint32_t IPLR1;
    __IO uint32_t IPLSR1;
    __IO uint32_t IPLCR1;
    __IO uint32_t ISR1;
    __IO uint32_t IER1_EXT;
    __IO uint32_t IESR1_EXT;
    __IO uint32_t IECR1_EXT;
    __IO uint32_t ISR1_EXT;
    __IO uint32_t OEMR1;
    __IO uint32_t OEMSR1;
    __IO uint32_t OEMCR1;
} GPIO1_TypeDef;


/******************* Bit definition for GPIO1_DIR0 register *******************/
#define GPIO1_DIR0_IN_Pos               (0U)
#define GPIO1_DIR0_IN_Msk               (0xFFFFFFFFUL << GPIO1_DIR0_IN_Pos)
#define GPIO1_DIR0_IN                   GPIO1_DIR0_IN_Msk

/******************* Bit definition for GPIO1_DOR0 register *******************/
#define GPIO1_DOR0_OUT_Pos              (0U)
#define GPIO1_DOR0_OUT_Msk              (0xFFFFFFFFUL << GPIO1_DOR0_OUT_Pos)
#define GPIO1_DOR0_OUT                  GPIO1_DOR0_OUT_Msk

/****************** Bit definition for GPIO1_DOSR0 register *******************/
#define GPIO1_DOSR0_DOS_Pos             (0U)
#define GPIO1_DOSR0_DOS_Msk             (0xFFFFFFFFUL << GPIO1_DOSR0_DOS_Pos)
#define GPIO1_DOSR0_DOS                 GPIO1_DOSR0_DOS_Msk

/****************** Bit definition for GPIO1_DOCR0 register *******************/
#define GPIO1_DOCR0_DOC_Pos             (0U)
#define GPIO1_DOCR0_DOC_Msk             (0xFFFFFFFFUL << GPIO1_DOCR0_DOC_Pos)
#define GPIO1_DOCR0_DOC                 GPIO1_DOCR0_DOC_Msk

/****************** Bit definition for GPIO1_DOER0 register *******************/
#define GPIO1_DOER0_DOE_Pos             (0U)
#define GPIO1_DOER0_DOE_Msk             (0xFFFFFFFFUL << GPIO1_DOER0_DOE_Pos)
#define GPIO1_DOER0_DOE                 GPIO1_DOER0_DOE_Msk

/****************** Bit definition for GPIO1_DOESR0 register ******************/
#define GPIO1_DOESR0_DOES_Pos           (0U)
#define GPIO1_DOESR0_DOES_Msk           (0xFFFFFFFFUL << GPIO1_DOESR0_DOES_Pos)
#define GPIO1_DOESR0_DOES               GPIO1_DOESR0_DOES_Msk

/****************** Bit definition for GPIO1_DOECR0 register ******************/
#define GPIO1_DOECR0_DOEC_Pos           (0U)
#define GPIO1_DOECR0_DOEC_Msk           (0xFFFFFFFFUL << GPIO1_DOECR0_DOEC_Pos)
#define GPIO1_DOECR0_DOEC               GPIO1_DOECR0_DOEC_Msk

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

/****************** Bit definition for GPIO1_IPHR0 register *******************/
#define GPIO1_IPHR0_IPH_Pos             (0U)
#define GPIO1_IPHR0_IPH_Msk             (0xFFFFFFFFUL << GPIO1_IPHR0_IPH_Pos)
#define GPIO1_IPHR0_IPH                 GPIO1_IPHR0_IPH_Msk

/****************** Bit definition for GPIO1_IPHSR0 register ******************/
#define GPIO1_IPHSR0_IPHS_Pos           (0U)
#define GPIO1_IPHSR0_IPHS_Msk           (0xFFFFFFFFUL << GPIO1_IPHSR0_IPHS_Pos)
#define GPIO1_IPHSR0_IPHS               GPIO1_IPHSR0_IPHS_Msk

/****************** Bit definition for GPIO1_IPHCR0 register ******************/
#define GPIO1_IPHCR0_IPHC_Pos           (0U)
#define GPIO1_IPHCR0_IPHC_Msk           (0xFFFFFFFFUL << GPIO1_IPHCR0_IPHC_Pos)
#define GPIO1_IPHCR0_IPHC               GPIO1_IPHCR0_IPHC_Msk

/****************** Bit definition for GPIO1_IPLR0 register *******************/
#define GPIO1_IPLR0_IPL_Pos             (0U)
#define GPIO1_IPLR0_IPL_Msk             (0xFFFFFFFFUL << GPIO1_IPLR0_IPL_Pos)
#define GPIO1_IPLR0_IPL                 GPIO1_IPLR0_IPL_Msk

/****************** Bit definition for GPIO1_IPLSR0 register ******************/
#define GPIO1_IPLSR0_IPLS_Pos           (0U)
#define GPIO1_IPLSR0_IPLS_Msk           (0xFFFFFFFFUL << GPIO1_IPLSR0_IPLS_Pos)
#define GPIO1_IPLSR0_IPLS               GPIO1_IPLSR0_IPLS_Msk

/****************** Bit definition for GPIO1_IPLCR0 register ******************/
#define GPIO1_IPLCR0_IPLC_Pos           (0U)
#define GPIO1_IPLCR0_IPLC_Msk           (0xFFFFFFFFUL << GPIO1_IPLCR0_IPLC_Pos)
#define GPIO1_IPLCR0_IPLC               GPIO1_IPLCR0_IPLC_Msk

/******************* Bit definition for GPIO1_ISR0 register *******************/
#define GPIO1_ISR0_IS_Pos               (0U)
#define GPIO1_ISR0_IS_Msk               (0xFFFFFFFFUL << GPIO1_ISR0_IS_Pos)
#define GPIO1_ISR0_IS                   GPIO1_ISR0_IS_Msk

/***************** Bit definition for GPIO1_IER0_EXT register *****************/
#define GPIO1_IER0_EXT_IER_Pos          (0U)
#define GPIO1_IER0_EXT_IER_Msk          (0xFFFFFFFFUL << GPIO1_IER0_EXT_IER_Pos)
#define GPIO1_IER0_EXT_IER              GPIO1_IER0_EXT_IER_Msk

/**************** Bit definition for GPIO1_IESR0_EXT register *****************/
#define GPIO1_IESR0_EXT_IES_Pos         (0U)
#define GPIO1_IESR0_EXT_IES_Msk         (0xFFFFFFFFUL << GPIO1_IESR0_EXT_IES_Pos)
#define GPIO1_IESR0_EXT_IES             GPIO1_IESR0_EXT_IES_Msk

/**************** Bit definition for GPIO1_IECR0_EXT register *****************/
#define GPIO1_IECR0_EXT_IEC_Pos         (0U)
#define GPIO1_IECR0_EXT_IEC_Msk         (0xFFFFFFFFUL << GPIO1_IECR0_EXT_IEC_Pos)
#define GPIO1_IECR0_EXT_IEC             GPIO1_IECR0_EXT_IEC_Msk

/***************** Bit definition for GPIO1_ISR0_EXT register *****************/
#define GPIO1_ISR0_EXT_IS_Pos           (0U)
#define GPIO1_ISR0_EXT_IS_Msk           (0xFFFFFFFFUL << GPIO1_ISR0_EXT_IS_Pos)
#define GPIO1_ISR0_EXT_IS               GPIO1_ISR0_EXT_IS_Msk

/****************** Bit definition for GPIO1_OEMR0 register *******************/
#define GPIO1_OEMR0_OEM_Pos             (0U)
#define GPIO1_OEMR0_OEM_Msk             (0xFFFFFFFFUL << GPIO1_OEMR0_OEM_Pos)
#define GPIO1_OEMR0_OEM                 GPIO1_OEMR0_OEM_Msk

/****************** Bit definition for GPIO1_OEMSR0 register ******************/
#define GPIO1_OEMSR0_OEMS_Pos           (0U)
#define GPIO1_OEMSR0_OEMS_Msk           (0xFFFFFFFFUL << GPIO1_OEMSR0_OEMS_Pos)
#define GPIO1_OEMSR0_OEMS               GPIO1_OEMSR0_OEMS_Msk

/****************** Bit definition for GPIO1_OEMCR0 register ******************/
#define GPIO1_OEMCR0_OEMC_Pos           (0U)
#define GPIO1_OEMCR0_OEMC_Msk           (0xFFFFFFFFUL << GPIO1_OEMCR0_OEMC_Pos)
#define GPIO1_OEMCR0_OEMC               GPIO1_OEMCR0_OEMC_Msk

/******************* Bit definition for GPIO1_DIR1 register *******************/
#define GPIO1_DIR1_IN_Pos               (0U)
#define GPIO1_DIR1_IN_Msk               (0xFFFFFFFFUL << GPIO1_DIR1_IN_Pos)
#define GPIO1_DIR1_IN                   GPIO1_DIR1_IN_Msk

/******************* Bit definition for GPIO1_DOR1 register *******************/
#define GPIO1_DOR1_OUT_Pos              (0U)
#define GPIO1_DOR1_OUT_Msk              (0xFFFFFFFFUL << GPIO1_DOR1_OUT_Pos)
#define GPIO1_DOR1_OUT                  GPIO1_DOR1_OUT_Msk

/****************** Bit definition for GPIO1_DOSR1 register *******************/
#define GPIO1_DOSR1_DOS_Pos             (0U)
#define GPIO1_DOSR1_DOS_Msk             (0xFFFFFFFFUL << GPIO1_DOSR1_DOS_Pos)
#define GPIO1_DOSR1_DOS                 GPIO1_DOSR1_DOS_Msk

/****************** Bit definition for GPIO1_DOCR1 register *******************/
#define GPIO1_DOCR1_DOC_Pos             (0U)
#define GPIO1_DOCR1_DOC_Msk             (0xFFFFFFFFUL << GPIO1_DOCR1_DOC_Pos)
#define GPIO1_DOCR1_DOC                 GPIO1_DOCR1_DOC_Msk

/****************** Bit definition for GPIO1_DOER1 register *******************/
#define GPIO1_DOER1_DOE_Pos             (0U)
#define GPIO1_DOER1_DOE_Msk             (0xFFFFFFFFUL << GPIO1_DOER1_DOE_Pos)
#define GPIO1_DOER1_DOE                 GPIO1_DOER1_DOE_Msk

/****************** Bit definition for GPIO1_DOESR1 register ******************/
#define GPIO1_DOESR1_DOES_Pos           (0U)
#define GPIO1_DOESR1_DOES_Msk           (0xFFFFFFFFUL << GPIO1_DOESR1_DOES_Pos)
#define GPIO1_DOESR1_DOES               GPIO1_DOESR1_DOES_Msk

/****************** Bit definition for GPIO1_DOECR1 register ******************/
#define GPIO1_DOECR1_DOEC_Pos           (0U)
#define GPIO1_DOECR1_DOEC_Msk           (0xFFFFFFFFUL << GPIO1_DOECR1_DOEC_Pos)
#define GPIO1_DOECR1_DOEC               GPIO1_DOECR1_DOEC_Msk

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

/****************** Bit definition for GPIO1_IPHR1 register *******************/
#define GPIO1_IPHR1_IPH_Pos             (0U)
#define GPIO1_IPHR1_IPH_Msk             (0xFFFFFFFFUL << GPIO1_IPHR1_IPH_Pos)
#define GPIO1_IPHR1_IPH                 GPIO1_IPHR1_IPH_Msk

/****************** Bit definition for GPIO1_IPHSR1 register ******************/
#define GPIO1_IPHSR1_IPHS_Pos           (0U)
#define GPIO1_IPHSR1_IPHS_Msk           (0xFFFFFFFFUL << GPIO1_IPHSR1_IPHS_Pos)
#define GPIO1_IPHSR1_IPHS               GPIO1_IPHSR1_IPHS_Msk

/****************** Bit definition for GPIO1_IPHCR1 register ******************/
#define GPIO1_IPHCR1_IPHC_Pos           (0U)
#define GPIO1_IPHCR1_IPHC_Msk           (0xFFFFFFFFUL << GPIO1_IPHCR1_IPHC_Pos)
#define GPIO1_IPHCR1_IPHC               GPIO1_IPHCR1_IPHC_Msk

/****************** Bit definition for GPIO1_IPLR1 register *******************/
#define GPIO1_IPLR1_IPL_Pos             (0U)
#define GPIO1_IPLR1_IPL_Msk             (0xFFFFFFFFUL << GPIO1_IPLR1_IPL_Pos)
#define GPIO1_IPLR1_IPL                 GPIO1_IPLR1_IPL_Msk

/****************** Bit definition for GPIO1_IPLSR1 register ******************/
#define GPIO1_IPLSR1_IPLS_Pos           (0U)
#define GPIO1_IPLSR1_IPLS_Msk           (0xFFFFFFFFUL << GPIO1_IPLSR1_IPLS_Pos)
#define GPIO1_IPLSR1_IPLS               GPIO1_IPLSR1_IPLS_Msk

/****************** Bit definition for GPIO1_IPLCR1 register ******************/
#define GPIO1_IPLCR1_IPLC_Pos           (0U)
#define GPIO1_IPLCR1_IPLC_Msk           (0xFFFFFFFFUL << GPIO1_IPLCR1_IPLC_Pos)
#define GPIO1_IPLCR1_IPLC               GPIO1_IPLCR1_IPLC_Msk

/******************* Bit definition for GPIO1_ISR1 register *******************/
#define GPIO1_ISR1_IS_Pos               (0U)
#define GPIO1_ISR1_IS_Msk               (0xFFFFFFFFUL << GPIO1_ISR1_IS_Pos)
#define GPIO1_ISR1_IS                   GPIO1_ISR1_IS_Msk

/***************** Bit definition for GPIO1_IER1_EXT register *****************/
#define GPIO1_IER1_EXT_IER_Pos          (0U)
#define GPIO1_IER1_EXT_IER_Msk          (0xFFFFFFFFUL << GPIO1_IER1_EXT_IER_Pos)
#define GPIO1_IER1_EXT_IER              GPIO1_IER1_EXT_IER_Msk

/**************** Bit definition for GPIO1_IESR1_EXT register *****************/
#define GPIO1_IESR1_EXT_IES_Pos         (0U)
#define GPIO1_IESR1_EXT_IES_Msk         (0xFFFFFFFFUL << GPIO1_IESR1_EXT_IES_Pos)
#define GPIO1_IESR1_EXT_IES             GPIO1_IESR1_EXT_IES_Msk

/**************** Bit definition for GPIO1_IECR1_EXT register *****************/
#define GPIO1_IECR1_EXT_IEC_Pos         (0U)
#define GPIO1_IECR1_EXT_IEC_Msk         (0xFFFFFFFFUL << GPIO1_IECR1_EXT_IEC_Pos)
#define GPIO1_IECR1_EXT_IEC             GPIO1_IECR1_EXT_IEC_Msk

/***************** Bit definition for GPIO1_ISR1_EXT register *****************/
#define GPIO1_ISR1_EXT_IS_Pos           (0U)
#define GPIO1_ISR1_EXT_IS_Msk           (0xFFFFFFFFUL << GPIO1_ISR1_EXT_IS_Pos)
#define GPIO1_ISR1_EXT_IS               GPIO1_ISR1_EXT_IS_Msk

/****************** Bit definition for GPIO1_OEMR1 register *******************/
#define GPIO1_OEMR1_OEM_Pos             (0U)
#define GPIO1_OEMR1_OEM_Msk             (0xFFFFFFFFUL << GPIO1_OEMR1_OEM_Pos)
#define GPIO1_OEMR1_OEM                 GPIO1_OEMR1_OEM_Msk

/****************** Bit definition for GPIO1_OEMSR1 register ******************/
#define GPIO1_OEMSR1_OEMS_Pos           (0U)
#define GPIO1_OEMSR1_OEMS_Msk           (0xFFFFFFFFUL << GPIO1_OEMSR1_OEMS_Pos)
#define GPIO1_OEMSR1_OEMS               GPIO1_OEMSR1_OEMS_Msk

/****************** Bit definition for GPIO1_OEMCR1 register ******************/
#define GPIO1_OEMCR1_OEMC_Pos           (0U)
#define GPIO1_OEMCR1_OEMC_Msk           (0xFFFFFFFFUL << GPIO1_OEMCR1_OEMC_Pos)
#define GPIO1_OEMCR1_OEMC               GPIO1_OEMCR1_OEMC_Msk

#endif
