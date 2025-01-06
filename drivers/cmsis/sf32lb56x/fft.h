#ifndef __FFT_H
#define __FFT_H

typedef struct
{
    __IO uint32_t CR;
    __IO uint32_t SAR;
    __IO uint32_t DAR;
    __IO uint32_t WAR;
    __IO uint32_t IRQ;
} FFT_TypeDef;


/********************* Bit definition for FFT_CR register *********************/
#define FFT_CR_START_Pos                (0U)
#define FFT_CR_START_Msk                (0x1UL << FFT_CR_START_Pos)
#define FFT_CR_START                    FFT_CR_START_Msk
#define FFT_CR_FFT_DCT_MODE_Pos         (1U)
#define FFT_CR_FFT_DCT_MODE_Msk         (0x1UL << FFT_CR_FFT_DCT_MODE_Pos)
#define FFT_CR_FFT_DCT_MODE             FFT_CR_FFT_DCT_MODE_Msk
#define FFT_CR_IFFT_MODE_Pos            (2U)
#define FFT_CR_IFFT_MODE_Msk            (0x1UL << FFT_CR_IFFT_MODE_Pos)
#define FFT_CR_IFFT_MODE                FFT_CR_IFFT_MODE_Msk
#define FFT_CR_RFFT_MODE_Pos            (3U)
#define FFT_CR_RFFT_MODE_Msk            (0x1UL << FFT_CR_RFFT_MODE_Pos)
#define FFT_CR_RFFT_MODE                FFT_CR_RFFT_MODE_Msk
#define FFT_CR_DCT_MODE_Pos             (4U)
#define FFT_CR_DCT_MODE_Msk             (0x1UL << FFT_CR_DCT_MODE_Pos)
#define FFT_CR_DCT_MODE                 FFT_CR_DCT_MODE_Msk
#define FFT_CR_DCT_TYPE_Pos             (5U)
#define FFT_CR_DCT_TYPE_Msk             (0x3UL << FFT_CR_DCT_TYPE_Pos)
#define FFT_CR_DCT_TYPE                 FFT_CR_DCT_TYPE_Msk
#define FFT_CR_FFT_LENGTH_Pos           (7U)
#define FFT_CR_FFT_LENGTH_Msk           (0xFUL << FFT_CR_FFT_LENGTH_Pos)
#define FFT_CR_FFT_LENGTH               FFT_CR_FFT_LENGTH_Msk
#define FFT_CR_INPUT_BW_SEL_Pos         (11U)
#define FFT_CR_INPUT_BW_SEL_Msk         (0x3UL << FFT_CR_INPUT_BW_SEL_Pos)
#define FFT_CR_INPUT_BW_SEL             FFT_CR_INPUT_BW_SEL_Msk
#define FFT_CR_OUTPUT_BW_SEL_Pos        (13U)
#define FFT_CR_OUTPUT_BW_SEL_Msk        (0x3UL << FFT_CR_OUTPUT_BW_SEL_Pos)
#define FFT_CR_OUTPUT_BW_SEL            FFT_CR_OUTPUT_BW_SEL_Msk
#define FFT_CR_FFT_WIN_EN_Pos           (15U)
#define FFT_CR_FFT_WIN_EN_Msk           (0x1UL << FFT_CR_FFT_WIN_EN_Pos)
#define FFT_CR_FFT_WIN_EN               FFT_CR_FFT_WIN_EN_Msk
#define FFT_CR_FFT_WIN_REAL_Pos         (16U)
#define FFT_CR_FFT_WIN_REAL_Msk         (0x1UL << FFT_CR_FFT_WIN_REAL_Pos)
#define FFT_CR_FFT_WIN_REAL             FFT_CR_FFT_WIN_REAL_Msk
#define FFT_CR_FFT_WIN_BW_SEL_Pos       (17U)
#define FFT_CR_FFT_WIN_BW_SEL_Msk       (0x3UL << FFT_CR_FFT_WIN_BW_SEL_Pos)
#define FFT_CR_FFT_WIN_BW_SEL           FFT_CR_FFT_WIN_BW_SEL_Msk
#define FFT_CR_FORCE_RSTB_Pos           (19U)
#define FFT_CR_FORCE_RSTB_Msk           (0x1UL << FFT_CR_FORCE_RSTB_Pos)
#define FFT_CR_FORCE_RSTB               FFT_CR_FORCE_RSTB_Msk

/******************** Bit definition for FFT_SAR register *********************/
#define FFT_SAR_INPUT_ADDR_Pos          (0U)
#define FFT_SAR_INPUT_ADDR_Msk          (0xFFFFFFFFUL << FFT_SAR_INPUT_ADDR_Pos)
#define FFT_SAR_INPUT_ADDR              FFT_SAR_INPUT_ADDR_Msk

/******************** Bit definition for FFT_DAR register *********************/
#define FFT_DAR_OUTPUT_ADDR_Pos         (0U)
#define FFT_DAR_OUTPUT_ADDR_Msk         (0xFFFFFFFFUL << FFT_DAR_OUTPUT_ADDR_Pos)
#define FFT_DAR_OUTPUT_ADDR             FFT_DAR_OUTPUT_ADDR_Msk

/******************** Bit definition for FFT_WAR register *********************/
#define FFT_WAR_WINDOW_ADDR_Pos         (0U)
#define FFT_WAR_WINDOW_ADDR_Msk         (0xFFFFFFFFUL << FFT_WAR_WINDOW_ADDR_Pos)
#define FFT_WAR_WINDOW_ADDR             FFT_WAR_WINDOW_ADDR_Msk

/******************** Bit definition for FFT_IRQ register *********************/
#define FFT_IRQ_ISR_Pos                 (0U)
#define FFT_IRQ_ISR_Msk                 (0x1UL << FFT_IRQ_ISR_Pos)
#define FFT_IRQ_ISR                     FFT_IRQ_ISR_Msk
#define FFT_IRQ_IRSR_Pos                (1U)
#define FFT_IRQ_IRSR_Msk                (0x1UL << FFT_IRQ_IRSR_Pos)
#define FFT_IRQ_IRSR                    FFT_IRQ_IRSR_Msk
#define FFT_IRQ_ICR_Pos                 (2U)
#define FFT_IRQ_ICR_Msk                 (0x1UL << FFT_IRQ_ICR_Pos)
#define FFT_IRQ_ICR                     FFT_IRQ_ICR_Msk
#define FFT_IRQ_IMR_Pos                 (3U)
#define FFT_IRQ_IMR_Msk                 (0x1UL << FFT_IRQ_IMR_Pos)
#define FFT_IRQ_IMR                     FFT_IRQ_IMR_Msk
#define FFT_IRQ_BUSY_Pos                (4U)
#define FFT_IRQ_BUSY_Msk                (0x1UL << FFT_IRQ_BUSY_Pos)
#define FFT_IRQ_BUSY                    FFT_IRQ_BUSY_Msk

#endif
