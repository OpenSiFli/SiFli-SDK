#ifndef __BOOT_FLASH__
#define __BOOT_FLASH__

#include "rtconfig.h"
#include <register.h>

#ifdef __cplusplus
extern "C" {
#endif

//---------------------------- MPI1 ----------------------------
#define FLASH1_IRQHandler              DMAC1_CH1_IRQHandler
#define FLASH1_DMA_IRQ_PRIO            0
#define FLASH1_DMA_INSTANCE            DMA1_Channel1
#define FLASH1_DMA_REQUEST             DMA_REQUEST_0
#define FLASH1_DMA_IRQ                 DMAC1_CH1_IRQn

#define FLASH1_CONFIG                                  \
    {                                                  \
        .Instance = FLASH1,                            \
        .line = 2,                                     \
        .base = FLASH_BASE_ADDR,                       \
        .msize = 4,                                    \
        .SpiMode = 0,                                  \
    }

#define FLASH1_DMA_CONFIG                              \
    {                                                  \
        .dma_irq_prio = FLASH1_DMA_IRQ_PRIO,                     \
        .Instance = FLASH1_DMA_INSTANCE,               \
        .dma_irq = FLASH1_DMA_IRQ,                     \
        .request = FLASH1_DMA_REQUEST,                 \
    }

//---------------------------- MPI2 ----------------------------
#define FLASH2_IRQHandler              DMAC1_CH2_IRQHandler
#define FLASH2_DMA_IRQ_PRIO            0
#define FLASH2_DMA_INSTANCE            DMA1_Channel2
#define FLASH2_DMA_REQUEST             DMA_REQUEST_1
#define FLASH2_DMA_IRQ                 DMAC1_CH2_IRQn

#define FLASH2_CONFIG                                  \
    {                                                  \
        .Instance = FLASH2,                            \
        .line = 2,                                     \
        .base = FLASH2_BASE_ADDR,                      \
        .msize = 4,                                    \
        .SpiMode = 0,                                  \
    }


#define FLASH2_DMA_CONFIG                              \
    {                                                  \
        .dma_irq_prio = FLASH2_DMA_IRQ_PRIO,                     \
        .Instance = FLASH2_DMA_INSTANCE,               \
        .dma_irq = FLASH2_DMA_IRQ,                     \
        .request = FLASH2_DMA_REQUEST,                 \
    }

//---------------------------- SD-NAND ----------------------------
#define SDNAND_MEM_ADDR (MPI2_MEM_BASE+HPSYS_MPI_MEM_CBUS_2_SBUS_OFFSET)
#define SDNAND_START_OFFSET     (0X1000)


/**
 * @brief Get Flash divider
 */
uint16_t BSP_GetFlash1DIV(void);
uint16_t BSP_GetFlash2DIV(void);
uint16_t BSP_GetFlash3DIV(void);
uint16_t BSP_GetFlash4DIV(void);
uint16_t BSP_GetFlash5DIV(void);

/**
 * @brief Set Flash divider
 */
void BSP_SetFlash1DIV(uint16_t div);
void BSP_SetFlash2DIV(uint16_t div);
void BSP_SetFlash3DIV(uint16_t div);
void BSP_SetFlash4DIV(uint16_t div);
void BSP_SetFlash5DIV(uint16_t div);

uint8_t is_addr_in_flash(uint32_t addr);


extern FLASH_HandleTypeDef *boot_handle;
extern uint32_t g_config_addr;

extern uint32_t g_boot_opt;
#define MPI_POWER_PIN  (21)
#ifdef TARMAC
#define BOOT_MODE_DELAY 1000
#else
#define BOOT_MODE_DELAY 1000000
#endif
extern void BSP_GPIO_Set(int pin, int val, int is_porta);

#define BOOT_SRC_Pos                    (0U)
#define BOOT_SRC_Msk                    (0xFUL << BOOT_SRC_Pos)
#define BOOT_SRC                        BOOT_SRC_Msk
#define BOOT_PD_Delay_Pos               (04U)
#define BOOT_PD_Delay_Msk               (0xFFUL << BOOT_PD_Delay_Pos)
#define BOOT_PD_Delay                   BOOT_PD_Delay_Msk
#define BOOT_PU_Delay_Pos               (12U)
#define BOOT_PU_Delay_Msk               (0xFFFUL << BOOT_PU_Delay_Pos)
#define BOOT_PU_Delay                   BOOT_PU_Delay_Msk

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */
