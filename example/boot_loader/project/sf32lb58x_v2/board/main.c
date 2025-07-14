/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     zylx         first version
 */

#include <rtconfig.h>
#include <board.h>
#include "stdio.h"
#include <string.h>
#include "register.h"
#include "../dfu/dfu.h"
#include "drv_io.h"
#include "dma_config.h"
#include "flash_config.h"
#include "sifli_bbm.h"
#include "secboot.h"
#include "../dfu/dfu_protocol.h"

// Target is retention memory 0x20000, 0x20000000 is temp before retention memory ready
#define HCPU_RAM_RETENTION_ADDR 0x20000000
#define REG_LOCK_PASSWORD       "66776677"

#define BOOT_INVALID_ADDR 0xFFFFFFFF

#define MPI1_POWER_PIN  (33)
#define MPI2_POWER_PIN  (26)
#define MPI3_POWER_PIN  (43)

typedef enum
{
    BOOT_FLASH1,
    BOOT_FLASH3,
    BOOT_FLASH4
} boot_flash_t;

typedef void (*ram_hook_handler)(void);
typedef int (*flash_enable_func)();

extern int BSP_Flash_hw1_init();
extern int BSP_Flash_hw3_init();
extern int BSP_Flash_hw4_init();
extern void flash_set_dual_mode(uint8_t id, uint8_t dual);

static UART_HandleTypeDef uart_handle;
static QSPI_FLASH_CTX_T spi_nand_handle;
static DMA_HandleTypeDef spi_nand_dma_handle;
static uint32_t nand_pagesize = 2048;
static uint32_t nand_blksize = 0x20000;
static uint32_t nand_cache[(4096 + 128) / 4];
static uint32_t bbm_cache_buf[(4096 + 128) / 4];
struct sec_configuration *g_sec_config;

struct sec_configuration sec_config_cache;
flash_read_func dfu_flash_reader = NULL;
static flash_read_func mpi5_flash_reader;
static flash_write_func mpi5_flash_writer;
static flash_erase_func mpi5_flash_eraser;


__STATIC_INLINE void dwtIpInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

__STATIC_INLINE void dwtIpDeinit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
}

__STATIC_INLINE void dwtReset(void)
{
    DWT->CYCCNT = 0; /* Clear DWT cycle counter */
}

__STATIC_INLINE uint32_t dwtGetCycles(void)
{
    return DWT->CYCCNT;
}

static void boot_uart_init(void)
{
    memset(&uart_handle, 0, sizeof(UART_HandleTypeDef));

#ifdef BSP_USING_UART4
    HAL_PIN_Set(PAD_PB37, USART4_TXD, PIN_PULLUP, 0);
    HAL_PIN_Set(PAD_PB36, USART4_RXD, PIN_PULLUP, 0);
    uart_handle.Instance = USART4;
    HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_LP_PERI, RCC_CLK_PERI_HXT48);
#elif defined(BSP_USING_UART1)
    HAL_PIN_Set(PAD_PA31, USART1_TXD, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA32, USART1_RXD, PIN_PULLUP, 1);
    uart_handle.Instance = USART1;
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_PERI, RCC_CLK_PERI_HXT48);
#else
#error Select UART1/4 for bootloader
#endif

    uart_handle.Init.BaudRate   = 1000000;
    uart_handle.Init.WordLength = UART_WORDLENGTH_8B;
    uart_handle.Init.StopBits   = UART_STOPBITS_1;
    uart_handle.Init.Parity     = UART_PARITY_NONE;
    uart_handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    uart_handle.Init.Mode       = UART_MODE_TX_RX;
    uart_handle.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&uart_handle) != HAL_OK)
    {

    }
}

#ifdef __CC_ARM
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#elif __ICCARM__
#error "not supported"
#else
// #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
int _write(int fd, char *ptr, int len)
{

    if (fd > 2)
    {
        return -1;
    }
    HAL_UART_Transmit(&uart_handle, ptr, len, 0xFFFF);
    return len;
}
#endif

#ifdef PUTCHAR_PROTOTYPE
PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART1 and Loop until the end of transmission */
    HAL_UART_Transmit(&uart_handle, (uint8_t *)&ch, 1, 0xFFFF);

    return ch;
}
#endif /* PUTCHAR_PROTOTYPE */


static uint32_t boot_get_flash_start_addr(boot_flash_t flash)
{
    uint32_t addr = BOOT_INVALID_ADDR;
    switch (flash)
    {
    case BOOT_FLASH1:
    {
        addr = QSPI1_MEM_BASE;
        break;
    }
    case BOOT_FLASH3:
    {
        addr = QSPI3_MEM_BASE;
        break;
    }
    case BOOT_FLASH4:
    {
        addr = QSPI4_MEM_BASE;
        break;
    }
    default:
        break;
    }
    return addr;
}

void boot_mpu_img(uint8_t *dest)
{
    __asm("LDR SP, [%0]" :: "r"(dest));
    __asm("LDR PC, [%0, #4]" :: "r"(dest));
}


void boot_ram(void)
{

    if (__HAL_SYSCFG_GET_BOOT_MODE() < 1)
    {
        volatile ram_hook_handler hook = (volatile ram_hook_handler)hwp_hpsys_aon->RESERVE0;
        if (hook)
            hook();
    }
}


void boot_image(int coreid, int force)
{
    struct image_header_enc *hdr;
    hdr = g_sec_config->running_imgs[coreid];
    if (hdr != (struct image_header_enc *)FLASH_UNINIT_32)
    {
        int flashid = hdr - &(g_sec_config->imgs[0]) + DFU_FLASH_IMG_LCPU;
        dfu_boot_img_in_flash(flashid);
    }
}

static void boot_disable_aes(uint32_t addr)
{
    FLASH_HandleTypeDef *fhandle = BSP_Flash_get_handle(addr);

    if (fhandle == NULL)
        return;

    HAL_FLASH_ENABLE_AES(fhandle, 0);
}


static int boot_nand_init()
{
    qspi_configure_t flash_cfg; // = FLASH3_CONFIG;
    struct dma_config flash_dma;
    uint16_t div;
    HAL_StatusTypeDef res = HAL_ERROR;
    int clk_mode;
    memset(&spi_nand_handle, 0, sizeof(QSPI_FLASH_CTX_T));

#if defined(BSP_ENABLE_QSPI3) && (BSP_QSPI3_MODE == SPI_MODE_NAND)
    qspi_configure_t flash_cfg3 = FLASH3_CONFIG;
    struct dma_config flash_dma3 = FLASH3_DMA_CONFIG;

    // set clock divider, it related with source clock freq
    div = BSP_GetFlash3DIV();
    clk_mode = RCC_CLK_MOD_FLASH3;
    memcpy(&flash_cfg, &flash_cfg3, sizeof(qspi_configure_t));
    memcpy(&flash_dma, &flash_dma3, sizeof(struct dma_config));
#elif defined(BSP_ENABLE_QSPI4) && (BSP_QSPI4_MODE == SPI_MODE_NAND)
    qspi_configure_t flash_cfg4 = FLASH4_CONFIG;
    struct dma_config flash_dma4 = FLASH4_DMA_CONFIG;

    // set clock divider, it related with source clock freq
    div = BSP_GetFlash4DIV();
    clk_mode = RCC_CLK_MOD_FLASH4;
    memcpy(&flash_cfg, &flash_cfg4, sizeof(qspi_configure_t));
    memcpy(&flash_dma, &flash_dma4, sizeof(struct dma_config));
#elif defined(BSP_ENABLE_QSPI2) && (BSP_QSPI2_MODE == SPI_MODE_NAND)
    qspi_configure_t flash_cfg2 = FLASH2_CONFIG;
    struct dma_config flash_dma2 = FLASH2_DMA_CONFIG;

    // set clock divider, it related with source clock freq
    div = BSP_GetFlash2DIV();
    clk_mode = RCC_CLK_MOD_FLASH2;
    memcpy(&flash_cfg, &flash_cfg2, sizeof(qspi_configure_t));
    memcpy(&flash_dma, &flash_dma2, sizeof(struct dma_config));
#endif

    // for nand, use nand interface to read/write, do not use AHB, so do not care address
    if (flash_cfg.SpiMode == SPI_MODE_NAND)
    {
#ifdef SOC_SF32LB55X
        if (flash_cfg.msize > (QSPI3_MAX_SIZE >> 20))
            flash_cfg.msize = QSPI3_MAX_SIZE >> 20;
#else
        flash_cfg.base = HCPU_MPI_SBUS_ADDR(flash_cfg.base);
#endif
    }
    else
    {
        return 0;
    }

    spi_nand_handle.handle.freq = flash_get_freq(clk_mode, div, 1);
    // init hardware, set dma, clock
    res = HAL_FLASH_Init(&spi_nand_handle, &flash_cfg, &spi_nand_dma_handle, &flash_dma, div);
    if (res == HAL_OK)
    {
        nand_pagesize = HAL_NAND_PAGE_SIZE(&spi_nand_handle.handle);
        nand_blksize = HAL_NAND_BLOCK_SIZE(&spi_nand_handle.handle);

        spi_nand_handle.handle.data_buf = (uint8_t *)nand_cache;
        spi_nand_handle.handle.buf_mode = 1;    // default set to buffer mode for nand
        HAL_NAND_CONF_ECC(&spi_nand_handle.handle, 1); // default enable ECC if support !
        bbm_set_page_size(nand_pagesize);
        bbm_set_blk_size(nand_blksize);
        sif_bbm_init(spi_nand_handle.total_size, (uint8_t *)bbm_cache_buf);

        return 1;
    }

    return 0;
}


int port_read_page(int blk, int page, int offset, uint8_t *buff, uint32_t size, uint8_t *spare, uint32_t spare_len)
{
    int res;
    uint32_t addr = (blk * nand_blksize) + (page * nand_pagesize) + offset;

    FLASH_HandleTypeDef *hflash = (FLASH_HandleTypeDef *)&spi_nand_handle.handle;
    if ((offset + size) > nand_pagesize)
    {
        HAL_ASSERT(0);
    }
#if (NAND_BUF_CPY_MODE == 1)
    SCB_InvalidateDCache_by_Addr(buff, size);
#endif
    SCB_InvalidateDCache_by_Addr((void *)hflash->base, nand_pagesize + SPI_NAND_MAXOOB_SIZE);
    if (((blk & 1) != 0) && (hflash->wakeup != 0))
        SCB_InvalidateDCache_by_Addr((void *)(hflash->base + (1 << 12)), SPI_NAND_PAGE_SIZE + SPI_NAND_MAXOOB_SIZE);
    res = HAL_NAND_READ_WITHOOB(hflash, addr, buff, size, spare, spare_len);


    return res; //RET_NOERROR;
}

int bbm_get_bb(int blk)
{
    FLASH_HandleTypeDef *hflash = (FLASH_HandleTypeDef *)&spi_nand_handle.handle;

    SCB_InvalidateDCache_by_Addr((void *)hflash->base, nand_pagesize + SPI_NAND_MAXOOB_SIZE);
    if (((blk & 1) != 0) && (hflash->wakeup != 0))
        SCB_InvalidateDCache_by_Addr((void *)(hflash->base + (1 << 12)), SPI_NAND_PAGE_SIZE + SPI_NAND_MAXOOB_SIZE);
    int bad = HAL_NAND_GET_BADBLK(hflash, blk);

    return bad;
}

static int read_nand(uint32_t addr, const int8_t *buf, uint32_t size)
{
    int cnt;
    uint32_t fill, offset;

    cnt = 0;
    offset = addr >= spi_nand_handle.handle.base ? addr - spi_nand_handle.handle.base : addr;
    while (size > 0)
    {
        fill = size > nand_pagesize ? nand_pagesize : size;
        if (bbm_read_page(offset / nand_blksize, (offset / nand_pagesize) & (nand_blksize / nand_pagesize - 1), offset & (nand_pagesize - 1), (uint8_t *)(buf + cnt), fill, NULL, 0) != fill)
            break;
        cnt += fill;
        size -= fill;
        offset += fill;
    }
    return cnt;
}

static int read_nor(uint32_t addr, const int8_t *buf, uint32_t size)
{
    memcpy((void *)buf, (uint8_t *)addr, size);
    return size;
}

static int write_nor(uint32_t addr, const int8_t *buf, uint32_t size)
{
    return BSP_Nor_write(addr, buf, size);
}

static int erase_nor(uint32_t addr, uint32_t size)
{
    return BSP_Nor_erase(addr, size);
}

static void init_mpi5(void)
{
    BSP_SetFlash5DIV(2);
    int res = BSP_Flash_hw5_init();
    if (res != 1)
        //return;
        HAL_ASSERT(0);

    mpi5_flash_reader = read_nor;
    mpi5_flash_writer = write_nor;
    mpi5_flash_eraser = erase_nor;
}

// Return:
// BOOT_INVALID_ADDR means init failed
// 1 means init success,

static int boot_flash_init(boot_flash_t flash)
{
    int ret = -1;
    int flash_en;
    flash_read_func flash_reader = NULL;
    uint32_t flash_addr = boot_get_flash_start_addr(flash);
    // Only need check flash1 and flash2
    if (flash_addr == BOOT_INVALID_ADDR)
        return ret;
    uint8_t id = flash == BOOT_FLASH1 ? 0 : 1;
    flash_enable_func init_func;

    /* init mpi5 for OTA */
    init_mpi5();

    if (flash == BOOT_FLASH1)
    {
        init_func = BSP_Flash_hw1_init;
        flash_reader = read_nor;
    }
    else if (flash == BOOT_FLASH3)
    {
#if defined(BSP_ENABLE_MPI3)
#if defined(BSP_USING_NOR_FLASH3)
        init_func = BSP_Flash_hw3_init;
        flash_reader = read_nor;
#else
        init_func = boot_nand_init;
        flash_reader = read_nand;
#endif /* BSP_USING_NOR_FLASH3 */
#endif /* BSP_ENABLE_MPI3 */
    }
    else if (flash == BOOT_FLASH4)
    {
#if defined(BSP_ENABLE_MPI4)
#if defined(BSP_USING_NOR_FLASH4)
        init_func = BSP_Flash_hw4_init;
        flash_reader = read_nor;
#else
        init_func = boot_nand_init;
        flash_reader = read_nand;
#endif  /* BSP_USING_NOR_FLASH4 */
#endif /* BSP_ENABLE_MPI4 */
    }
    else
    {
        HAL_ASSERT(0);
    }

    // Enable flash with single mode
    //flash_set_dual_mode(id, 0);
    flash_en = init_func();

    // Enable failed
    if (!flash_en)
    {
        return ret;
    }
    else
    {
        ret = 1; // flash existed
    }

    boot_set_flash_read_func(flash_reader);
    dfu_flash_reader = flash_reader;

    return ret;
}

uint32_t boot_enable_flash(void)
{
    uint32_t addr = BOOT_INVALID_ADDR;
    int ret;
#if  (defined (BSP_USING_SPI_FLASH))
    do
    {
        int flash_en;
        /* init divider for bootloader */
        if (CHIP_IS_585())
            BSP_SetFlash1DIV(6);
        else
            BSP_SetFlash1DIV(1);
        BSP_SetFlash3DIV(6);
        BSP_SetFlash4DIV(6);

#if defined(BSP_USING_NOR_FLASH4) || defined(BSP_USING_NAND_FLASH4)
        HAL_PIN_Set(PAD_PA39, MPI4_CLK, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_PA30, MPI4_CS, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_PA40, MPI4_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA37, MPI4_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA36, MPI4_DIO2, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA38, MPI4_DIO3, PIN_PULLUP, 1);

        if (boot_flash_init(BOOT_FLASH4) != BOOT_INVALID_ADDR)
            addr = boot_get_flash_start_addr(BOOT_FLASH4);
#elif defined(BSP_USING_NOR_FLASH3) || defined(BSP_USING_NAND_FLASH3)
        HAL_PIN_Set(PAD_PA46, MPI3_CLK, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_PA44, MPI3_CS, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_PA50, MPI3_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA48, MPI3_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_PA47, MPI3_DIO2, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_PA45, MPI3_DIO3, PIN_PULLUP, 1);
        if (boot_flash_init(BOOT_FLASH3) != BOOT_INVALID_ADDR)
            addr = boot_get_flash_start_addr(BOOT_FLASH3);
#elif defined(BSP_USING_NOR_FLASH1)
        HAL_PIN_Set(PAD_SA11, MPI1_CLK, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA01, MPI1_CS, PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA10, MPI1_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA02, MPI1_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA03, MPI1_DIO2, PIN_PULLUP, 1);
        HAL_PIN_Set(PAD_SA12, MPI1_DIO3, PIN_PULLUP, 1);
        if (boot_flash_init(BOOT_FLASH1) != BOOT_INVALID_ADDR)
            addr = boot_get_flash_start_addr(BOOT_FLASH1);
#endif
    }
    while (0);
#endif /* (defined (BSP_USING_FLASH)||defined (BSP_USING_SPI_FLASH))  */
    return addr;
}

static void config_psram_pinmux(void)
{
    if (CHIP_IS_583())
    {
        // PSRAM1
        HAL_PIN_Set(PAD_SA04, MPI1_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA05, MPI1_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA06, MPI1_DIO2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA07, MPI1_DIO3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA09, MPI1_CS,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA10, MPI1_CLK,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA11, MPI1_DIO4, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA12, MPI1_DIO5, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA13, MPI1_DIO6, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA14, MPI1_DIO7, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA15, MPI1_DQS0,  PIN_PULLDOWN, 1);

        // PSRAM2
        HAL_PIN_Set(PAD_SB04, MPI2_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB05, MPI2_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB06, MPI2_DIO2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB07, MPI2_DIO3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB09, MPI2_CS,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB10, MPI2_CLK,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB11, MPI2_DIO4, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB12, MPI2_DIO5, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB13, MPI2_DIO6, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB14, MPI2_DIO7, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB15, MPI2_DQS0,  PIN_PULLDOWN, 1);
    }
    else if (CHIP_IS_587())
    {
        // PSRAM1
        HAL_PIN_Set(PAD_SA00, MPI1_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA01, MPI1_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA02, MPI1_DIO2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA03, MPI1_DIO3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA04, MPI1_DIO4, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA05, MPI1_DIO5, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA06, MPI1_DIO6, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA07, MPI1_DIO7, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA08, MPI1_DQS0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA09, MPI1_CLK,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA10, MPI1_CS,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SA11, MPI1_DIO8, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA12, MPI1_DIO9, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA13, MPI1_DIO10, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA14, MPI1_DIO11, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA15, MPI1_DIO12, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA16, MPI1_DIO13, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA17, MPI1_DIO14, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA18, MPI1_DIO15, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SA19, MPI1_DQS1, PIN_PULLDOWN, 1);

        // PSRAM2
        HAL_PIN_Set(PAD_SB00, MPI2_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB01, MPI2_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB02, MPI2_DIO2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB03, MPI2_DIO3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB04, MPI2_DIO4, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB05, MPI2_DIO5, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB06, MPI2_DIO6, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB07, MPI2_DIO7, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB08, MPI2_DQS0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB09, MPI2_CLK,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB10, MPI2_CS,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB11, MPI2_DIO8, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB12, MPI2_DIO9, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB13, MPI2_DIO10, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB14, MPI2_DIO11, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB15, MPI2_DIO12, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB16, MPI2_DIO13, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB17, MPI2_DIO14, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB18, MPI2_DIO15, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB19, MPI2_DQS1, PIN_PULLDOWN, 1);
    }
    else if (CHIP_IS_585())
    {
        // PSRAM2
        HAL_PIN_Set(PAD_SB00, MPI2_DIO0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB01, MPI2_DIO1, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB02, MPI2_DIO2, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB03, MPI2_DIO3, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB04, MPI2_DIO4, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB05, MPI2_DIO5, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB06, MPI2_DIO6, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB07, MPI2_DIO7, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB08, MPI2_DQS0, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB09, MPI2_CLK,   PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB10, MPI2_CS,  PIN_NOPULL, 1);
        HAL_PIN_Set(PAD_SB11, MPI2_DIO8, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB12, MPI2_DIO9, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB13, MPI2_DIO10, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB14, MPI2_DIO11, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB15, MPI2_DIO12, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB16, MPI2_DIO13, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB17, MPI2_DIO14, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB18, MPI2_DIO15, PIN_PULLDOWN, 1);
        HAL_PIN_Set(PAD_SB19, MPI2_DQS1, PIN_PULLDOWN, 1);
    }
}

void boot_images()
{
}

struct sec_configuration g_temp_sec_config;
struct sec_configuration *temp_sec_config;

static void update_sec_flash(struct sec_configuration *sec_config)
{
    uint32_t flash_addr = FLASH_BASE_ADDR;
    //uint32_t size = (sizeof(struct sec_configuration) + SPI_NAND_PAGE_SIZE * 2) & (~(SPI_NAND_PAGE_SIZE * 2 - 1));
    uint32_t size = sizeof(struct sec_configuration);
    if (size % 4096 != 0)
    {
        size = (size + 4096) / 4096 * 4096;
    }
    mpi5_flash_eraser(flash_addr, size);
    mpi5_flash_writer(flash_addr, (const int8_t *)sec_config, sizeof(struct sec_configuration));

    g_sec_config = (struct sec_configuration *)FLASH_BASE_ADDR;
}

static void select_boot()
{
    uint8_t hcpu_des = HAL_Get_backup(RTC_BACKUP_NAND_OTA_DES);
    uint32_t hcpu_len = HAL_Get_backup(RTC_BAKCUP_OTA_FORCE_MODE);
    temp_sec_config = &g_temp_sec_config;

    struct sec_configuration *flash_config = (struct sec_configuration *)FLASH_BASE_ADDR;
    //memcpy((const int8_t *)temp_sec_config, (uint8_t *)MPI5_MEM_BASE, sizeof(struct sec_configuration));
    read_nor(FLASH_BASE_ADDR, (const int8_t *)temp_sec_config, sizeof(struct sec_configuration));

    int hcpu1_img_idx = DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_HCPU);
    int hcpu2_img_idx = DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_HCPU2);
    // HAL_sw_breakpoint();

    if (sec_config_cache.running_imgs[CORE_HCPU] == &(flash_config->imgs[hcpu1_img_idx]))
    {
        // now we are running on hcpu1
        switch (hcpu_des)
        {
        case DFU_DES_RUNNING_ON_HCPU1:
        {
            // normal power on hcpu1
            break;
        }
        case DFU_DES_SWITCH_TO_HCPU2:
        {
            // switch to hcpu2

            sec_config_cache.running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu2_img_idx]);

            hcpu_des = DFU_DES_UPDATE_HCPU2;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);

            temp_sec_config->running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu2_img_idx]);
            if (hcpu_len != 0)
            {
                temp_sec_config->imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_HCPU2)].length = hcpu_len;
            }

            update_sec_flash(temp_sec_config);
            break;
        }
        case DFU_DES_UPDATE_HCPU2:
        {
            // switch to hcpu2 fail, back to hcpu1
            sec_config_cache.running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu1_img_idx]);
            hcpu_des = DFU_DES_RUNNING_ON_HCPU1;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);

            temp_sec_config->running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu1_img_idx]);
            update_sec_flash(temp_sec_config);
            break;
        }
        case DFU_DES_NONE:
        {
            // first power on, use current
            hcpu_des = DFU_DES_RUNNING_ON_HCPU1;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);
            break;
        }
        default:
            hcpu_des = DFU_DES_RUNNING_ON_HCPU1;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);
        }

    }
    else if (sec_config_cache.running_imgs[CORE_HCPU] == &(flash_config->imgs[hcpu2_img_idx]))
    {
        // now we are running on hcpu2
        switch (hcpu_des)
        {
        case DFU_DES_RUNNING_ON_HCPU2:
        {
            // normal power on hcpu2
            break;
        }
        case DFU_DES_SWITCH_TO_HCPU1:
        {
            // switch to hcpu1
            sec_config_cache.running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu1_img_idx]);
            hcpu_des = DFU_DES_UPDATE_HCPU1;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);

            temp_sec_config->running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu1_img_idx]);
            if (hcpu_len != 0)
            {
                temp_sec_config->imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_HCPU)].length = hcpu_len;
            }

            update_sec_flash(temp_sec_config);
            break;
        }
        case DFU_DES_UPDATE_HCPU1:
        {
            // switch to hcpu1 fail, back to hcpu2
            sec_config_cache.running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu2_img_idx]);
            hcpu_des = DFU_DES_RUNNING_ON_HCPU2;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);

            temp_sec_config->running_imgs[CORE_HCPU] = &(flash_config->imgs[hcpu2_img_idx]);
            update_sec_flash(temp_sec_config);
            break;
        }
        case DFU_DES_NONE:
        {
            // first power on, use current
            hcpu_des = DFU_DES_RUNNING_ON_HCPU2;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);
            break;
        }
        default:
            hcpu_des = DFU_DES_RUNNING_ON_HCPU2;
            HAL_Set_backup(RTC_BACKUP_NAND_OTA_DES, hcpu_des);
        }
    }
}

void boot_images_main()
{
    int i;
    int flash_en;
    uint32_t start_addr;

#ifdef CODE_IN_RAM
    boot_mpu_img((uint8_t *)(HPSYS_RAM0_BASE + 0x20000));
#endif

    /* set MPI1 clock divider for PSRAM */
    BSP_SetFlash1DIV(1);
    BSP_SetFlash2DIV(1);

    HAL_HPAON_EnableXT48();

    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
    // HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_PERI, RCC_CLK_PERI_HXT48);
    boot_uart_init();

    HAL_PMU_EnableDLL(1);

#if defined (BSP_USING_PSRAM)
    dwtIpInit();
    /* wait until 2ms elapse to ensure flash LDO is stable before flash access
     * default clock is 24MHz, maybe slower than 24MHz at startup stage
     *
     */
    while (dwtGetCycles() < (2 * 24000))
    {
    }

    HAL_RCC_HCPU_EnableDLL1(240000000);
    /* set hdiv again to make HCLK run on 240MHz actually
     * Although the default hdiv registe value is 1, hardware would use 2 as the default hdiv.
     * After write the hdiv register, hardware would use the updated value.
     */
    HAL_RCC_HCPU_SetDiv(1, 2, 5);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_DLL1);

    __HAL_SYSCFG_HPBG_EN();
    __HAL_SYSCFG_HPBG_VDDPSW_EN();

    dwtIpInit();
    /* wait until 2ms elapse to ensure flash LDO is stable before flash access
     * default clock is 24MHz, maybe slower than 24MHz at startup stage
     *
     */
    while (dwtGetCycles() < (20 * 2400))
    {
    }


    if (CHIP_IS_583() || CHIP_IS_585())
    {
        HAL_RCC_HCPU_EnableDLL2(240000000);
    }
    else if (CHIP_IS_587())
    {
        HAL_RCC_HCPU_EnableDLL2(336000000);
        HAL_RCC_HCPU_EnableDLL3(216000000);
        HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH4, RCC_CLK_FLASH_DLL3);
    }

    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH1, RCC_CLK_FLASH_DLL2);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH2, RCC_CLK_FLASH_DLL2);
    config_psram_pinmux();
    bsp_psramc_init();
#endif
    HAL_Delay_us(0);

    start_addr = boot_enable_flash();
    // Enable failed just back to bootloader loop.
    if (start_addr == BOOT_INVALID_ADDR)
        return;

    {
        extern void mbedtls_aes_lib_init(void);
        mbedtls_aes_lib_init();
    }

    boot_disable_aes(start_addr);
    /* init AES_ACC as normal mode */
    __HAL_SYSCFG_CLEAR_SECURITY();

    dfu_install_info info;
    dfu_install_info info_ext;
    if (dfu_flash_reader != NULL)
    {
        if (DFU_DOWNLOAD_REGION_START_ADDR != FLASH_UNINIT_32)
        {
            dfu_flash_reader(DFU_DOWNLOAD_REGION_START_ADDR, (const int8_t *)&info, sizeof(dfu_install_info));
        }
        if (DFU_INFO_REGION_START_ADDR != FLASH_UNINIT_32)
        {
            dfu_flash_reader(DFU_INFO_REGION_START_ADDR, (const int8_t *)&info_ext, sizeof(dfu_install_info));
        }
        if (info.magic == SEC_CONFIG_MAGIC && info_ext.magic == SEC_CONFIG_MAGIC)
        {
            info = info_ext;
        }
    }

    g_sec_config = (struct sec_configuration *)FLASH_BASE_ADDR;

    read_nor(FLASH_BASE_ADDR, (const int8_t *)&sec_config_cache, sizeof(struct sec_configuration));

    select_boot();

    if (DFU_DOWNLOAD_REGION_START_ADDR != FLASH_UNINIT_32)
    {
        if ((HAL_Get_backup(RTC_BAKCUP_OTA_FORCE_MODE) == DFU_FORCE_MODE_REBOOT_TO_PACKAGE_OTA_MANAGER) ||
                (info.magic == SEC_CONFIG_MAGIC) && (info.install_state == DFU_PACKAGE_INSTALL))
        {
            sec_config_cache.running_imgs[CORE_HCPU] = (struct image_header_enc *) & (((struct sec_configuration *)FLASH_TABLE_START_ADDR)->imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_IMG_LCPU)]);
        }
    }

    if (g_sec_config->magic == SEC_CONFIG_MAGIC)
    {
        if (g_sec_config->running_imgs[CORE_BOOT] != (struct image_header_enc *)FLASH_UNINIT_32)
        {
            int flash_id = g_sec_config->running_imgs[CORE_BOOT] - &(g_sec_config->imgs[0]) + DFU_FLASH_IMG_LCPU;
            dfu_boot_img_in_flash(flash_id);
        }

        /* load extended img if present, such as ACPU img */
        if (g_sec_config->imgs[DFU_FLASH_IMG_IDX(DFU_FLASH_HCPU_EXT2)].length != FLASH_UNINIT_32)
        {
            dfu_boot_img_in_flash(DFU_FLASH_HCPU_EXT2);
        }

        if (g_sec_config->running_imgs[CORE_HCPU] != (struct image_header_enc *)FLASH_UNINIT_32)
        {
            int flash_id = ((uint32_t)sec_config_cache.running_imgs[CORE_HCPU] - FLASH_BASE_ADDR - 0x1000) / sizeof(struct image_header_enc) + DFU_FLASH_IMG_LCPU;
            dfu_boot_img_in_flash(flash_id);
        }

        boot_image(CORE_HCPU, 0);
    }
}


static void boot_flash_power_on(void)
{
#define V58_PIN   (58U)
    uint8_t pin = V58_PIN;

#if 0
    //GPIO_InitTypeDef gpio_config;
    //GPIO_A39
    gpio_config.Pin = 39;
    gpio_config.Mode = GPIO_MODE_OUTPUT;
    gpio_config.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(hwp_gpio1, &gpio_config);
    HAL_GPIO_WritePin(hwp_gpio1, 39, GPIO_PIN_SET);
#endif


    /* enable V43 */
    pin = 43 - 32;
    (hwp_gpio1 + 1)->DOESR |= (1UL << pin);
    (hwp_gpio1 + 1)->DOR |= (1UL << pin);

    /* enable V33 */
    pin = 33 - 32;
    (hwp_gpio1 + 1)->DOESR |= (1UL << pin);
    (hwp_gpio1 + 1)->DOR |= (1UL << pin);

    /* enable V26 */
    pin = 26;
    (hwp_gpio1 + 0)->DOESR |= (1UL << pin);
    (hwp_gpio1 + 0)->DOR |= (1UL << pin);

#ifdef SOC_BF0_HCPU
    // Enable PADA
    HAL_HPAON_ENABLE_PAD();

    HAL_PBR0_FORCE1_ENABLE();
    HAL_PBR_ConfigMode(0, 1); //set PBR0 output
    HAL_PBR_WritePin(0, 1); //set PBR0 high

#endif /* SOC_BF0_HCPU */


    dwtIpInit();
    /* wait until 2ms elapse to ensure flash LDO is stable before flash access
     * default clock is 24MHz, maybe slower than 24MHz at startup stage
     *
     */
    while (dwtGetCycles() < (2 * 24000))
    {
    }

    dwtIpDeinit();

#if defined(SOC_SF32LB58X) && defined(FPGA)
    HAL_QSPI_SET_RXDELAY(0, 0, 2);
    HAL_QSPI_SET_RXDELAY(1, 0, 2);
#endif
}


static void boot_efuse_init(void)
{

    HAL_EFUSE_Init();
    sifli_hw_efuse_read_all();
    /* init hook to NULL as bss is initialized yet */
    g_dfu_efuse_read_hook = NULL;
}

void hw_preinit0(void)
{
    // Enable cache due to boot flow won't enable cache.
    SCB_EnableICache();
    SCB_EnableDCache();
    // 1. Read efuse first to take efuse effect.
    boot_efuse_init();

    // 2. If ram hook existed, just jump to ram.
    boot_ram();

    // 3. Power on flash and wait it's stable.
    boot_flash_power_on();
}



int regop_lock_check(char *passwd, uint32_t len)
{
    int ret = -1;
    if (strncmp((const char *)passwd, REG_LOCK_PASSWORD, sizeof(REG_LOCK_PASSWORD)) == 0)
        ret = 0;
    return ret;
}


int main(void)
{
    int count = 1;

    //g_sec_config = (struct sec_configuration *)boot_enable_flash();
#if 0
    rt_flash_set_alias(FLASH_START, FLASH_START, 0, 0);
    g_sec_config = (struct sec_configuration *)FLASH_START;
    if (g_sec_config->magic != SEC_CONFIG_MAGIC)
    {
        // TODO: patch not installed
        uint32_t magic = SEC_CONFIG_MAGIC;
        sec_flash_erase(DFU_FLASH_SEC_CONFIG, 0, sizeof(*g_sec_config));
        sec_flash_write(DFU_FLASH_SEC_CONFIG, 0, (uint8_t *)&magic, 4);
    }
#endif

    boot_images_main();

    while (count++)
    {
        printf("boot fail\n");
        HAL_Delay(2000);
    }

    return 0;
}

