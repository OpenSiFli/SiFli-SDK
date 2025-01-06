#ifndef __BOOT_FLASH__
#define __BOOT_FLASH__

#include "rtconfig.h"
#include <register.h>

#ifdef __cplusplus
extern "C" {
#endif

//---------------------------- MPI1 ----------------------------
#define FLASH3_IRQHandler              DMAC1_CH3_IRQHandler
#define FLASH3_DMA_IRQ_PRIO            (0)
#define FLASH3_DMA_INSTANCE            DMA1_Channel3
#define FLASH3_DMA_REQUEST             DMA_REQUEST_2
#define FLASH3_DMA_IRQ                 DMAC1_CH3_IRQn

#define FLASH3_CONFIG                                  \
    {                                                  \
        .Instance = FLASH3,                            \
        .line = 2,                                     \
        .base = FLASH3_BASE_ADDR,                       \
        .msize = 4,                                    \
        .SpiMode = 0,                                  \
    }

#define FLASH3_DMA_CONFIG                              \
    {                                                  \
        .dma_irq_prio = FLASH3_DMA_IRQ_PRIO,           \
        .Instance = FLASH3_DMA_INSTANCE,               \
        .dma_irq = FLASH3_DMA_IRQ,                     \
        .request = FLASH3_DMA_REQUEST,                 \
    }

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

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */
