/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*********************************************************************
*            (c) 1995 - 2018 SEGGER Microcontroller GmbH             *
*                        The Embedded Experts                        *
*                           www.segger.com                           *
**********************************************************************
----------------------------------------------------------------------
File    : FlashPrg.c
Purpose : Implementation of RAMCode template
--------  END-OF-HEADER  ---------------------------------------------
*/

#include <bf0_hal.h>
#include <string.h>
#include <stdio.h>
#include <drv_io.h>
#include "flash_table.h"
#include "flash_config.h"
#include "dma_config.h"
#include "flashos.h"
#include "flashdrv_version.h"

#define FLASH1_ENABLED      (1<<0)
#define FLASH2_ENABLED      (1<<1)
#define FLASH3_ENABLED      (1<<2)
#define FLASH4_ENABLED      (1<<3)
#define FLASH5_ENABLED      (1<<4)

#define ECC_OFFSET   (8)
#define ECC_SIZE     (8)

typedef enum
{
    JLINK_STAGE_COMPARE,
    JLINK_STAGE_ERASE,
    JLINK_STAGE_PROGRAM,
    JLINK_STAGE_VERIFY
} jlink_stage_t;


#define JLINK_FUNC_INVALID      (0)
#define JLINK_FUNC_ERASE        (1)
#define JLINK_FUNC_PROGRAM      (2)
#define JLINK_FUNC_VERIFY_READ  (3)

#define DEBUG_JLINK         0
#define DEFAULT_TRACE       1

static QSPI_FLASH_CTX_T spi_flash_handle[FLASH_MAX_INSTANCE];
static DMA_HandleTypeDef spi_flash_dma_handle[FLASH_MAX_INSTANCE];
static char nand_buf[SPI_NAND_PAGE_SIZE + SPI_NAND_MAXOOB_SIZE];

extern void debug_print(char *str);
extern struct FlashDevice const FlashDevice;
#ifdef _USE_PRODUCTLINE
uint32_t Crc32Table1[256] =
{
    0x0, 0x77073096, 0xee0e612c, 0x990951ba, 0x76dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
    0xedb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x9b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
    0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
    0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
    0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
    0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
    0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
    0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
    0x76dc4190, 0x1db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x6b6b51f, 0x9fbfe4a5, 0xe8b8d433,
    0x7807c9a2, 0xf00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x86d3d2d, 0x91646c97, 0xe6635c01,
    0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
    0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
    0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
    0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
    0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
    0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
    0xedb88320, 0x9abfb3b6, 0x3b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x4db2615, 0x73dc1683,
    0xe3630b12, 0x94643b84, 0xd6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0xa00ae27, 0x7d079eb1,
    0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
    0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
    0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
    0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
    0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
    0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
    0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x26d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x5005713,
    0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0xcb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0xbdbdf21,
    0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
    0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
    0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
    0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
    0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
    0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

uint32_t Crc32Table2[256] =
{
    0x0, 0xf26b8303, 0xe13b70f7, 0x1350f3f4, 0xc79a971f, 0x35f1141c, 0x26a1e7e8, 0xd4ca64eb,
    0x8ad958cf, 0x78b2dbcc, 0x6be22838, 0x9989ab3b, 0x4d43cfd0, 0xbf284cd3, 0xac78bf27, 0x5e133c24,
    0x105ec76f, 0xe235446c, 0xf165b798, 0x30e349b, 0xd7c45070, 0x25afd373, 0x36ff2087, 0xc494a384,
    0x9a879fa0, 0x68ec1ca3, 0x7bbcef57, 0x89d76c54, 0x5d1d08bf, 0xaf768bbc, 0xbc267848, 0x4e4dfb4b,
    0x20bd8ede, 0xd2d60ddd, 0xc186fe29, 0x33ed7d2a, 0xe72719c1, 0x154c9ac2, 0x61c6936, 0xf477ea35,
    0xaa64d611, 0x580f5512, 0x4b5fa6e6, 0xb93425e5, 0x6dfe410e, 0x9f95c20d, 0x8cc531f9, 0x7eaeb2fa,
    0x30e349b1, 0xc288cab2, 0xd1d83946, 0x23b3ba45, 0xf779deae, 0x5125dad, 0x1642ae59, 0xe4292d5a,
    0xba3a117e, 0x4851927d, 0x5b016189, 0xa96ae28a, 0x7da08661, 0x8fcb0562, 0x9c9bf696, 0x6ef07595,
    0x417b1dbc, 0xb3109ebf, 0xa0406d4b, 0x522bee48, 0x86e18aa3, 0x748a09a0, 0x67dafa54, 0x95b17957,
    0xcba24573, 0x39c9c670, 0x2a993584, 0xd8f2b687, 0xc38d26c, 0xfe53516f, 0xed03a29b, 0x1f682198,
    0x5125dad3, 0xa34e59d0, 0xb01eaa24, 0x42752927, 0x96bf4dcc, 0x64d4cecf, 0x77843d3b, 0x85efbe38,
    0xdbfc821c, 0x2997011f, 0x3ac7f2eb, 0xc8ac71e8, 0x1c661503, 0xee0d9600, 0xfd5d65f4, 0xf36e6f7,
    0x61c69362, 0x93ad1061, 0x80fde395, 0x72966096, 0xa65c047d, 0x5437877e, 0x4767748a, 0xb50cf789,
    0xeb1fcbad, 0x197448ae, 0xa24bb5a, 0xf84f3859, 0x2c855cb2, 0xdeeedfb1, 0xcdbe2c45, 0x3fd5af46,
    0x7198540d, 0x83f3d70e, 0x90a324fa, 0x62c8a7f9, 0xb602c312, 0x44694011, 0x5739b3e5, 0xa55230e6,
    0xfb410cc2, 0x92a8fc1, 0x1a7a7c35, 0xe811ff36, 0x3cdb9bdd, 0xceb018de, 0xdde0eb2a, 0x2f8b6829,
    0x82f63b78, 0x709db87b, 0x63cd4b8f, 0x91a6c88c, 0x456cac67, 0xb7072f64, 0xa457dc90, 0x563c5f93,
    0x82f63b7, 0xfa44e0b4, 0xe9141340, 0x1b7f9043, 0xcfb5f4a8, 0x3dde77ab, 0x2e8e845f, 0xdce5075c,
    0x92a8fc17, 0x60c37f14, 0x73938ce0, 0x81f80fe3, 0x55326b08, 0xa759e80b, 0xb4091bff, 0x466298fc,
    0x1871a4d8, 0xea1a27db, 0xf94ad42f, 0xb21572c, 0xdfeb33c7, 0x2d80b0c4, 0x3ed04330, 0xccbbc033,
    0xa24bb5a6, 0x502036a5, 0x4370c551, 0xb11b4652, 0x65d122b9, 0x97baa1ba, 0x84ea524e, 0x7681d14d,
    0x2892ed69, 0xdaf96e6a, 0xc9a99d9e, 0x3bc21e9d, 0xef087a76, 0x1d63f975, 0xe330a81, 0xfc588982,
    0xb21572c9, 0x407ef1ca, 0x532e023e, 0xa145813d, 0x758fe5d6, 0x87e466d5, 0x94b49521, 0x66df1622,
    0x38cc2a06, 0xcaa7a905, 0xd9f75af1, 0x2b9cd9f2, 0xff56bd19, 0xd3d3e1a, 0x1e6dcdee, 0xec064eed,
    0xc38d26c4, 0x31e6a5c7, 0x22b65633, 0xd0ddd530, 0x417b1db, 0xf67c32d8, 0xe52cc12c, 0x1747422f,
    0x49547e0b, 0xbb3ffd08, 0xa86f0efc, 0x5a048dff, 0x8ecee914, 0x7ca56a17, 0x6ff599e3, 0x9d9e1ae0,
    0xd3d3e1ab, 0x21b862a8, 0x32e8915c, 0xc083125f, 0x144976b4, 0xe622f5b7, 0xf5720643, 0x7198540,
    0x590ab964, 0xab613a67, 0xb831c993, 0x4a5a4a90, 0x9e902e7b, 0x6cfbad78, 0x7fab5e8c, 0x8dc0dd8f,
    0xe330a81a, 0x115b2b19, 0x20bd8ed, 0xf0605bee, 0x24aa3f05, 0xd6c1bc06, 0xc5914ff2, 0x37faccf1,
    0x69e9f0d5, 0x9b8273d6, 0x88d28022, 0x7ab90321, 0xae7367ca, 0x5c18e4c9, 0x4f48173d, 0xbd23943e,
    0xf36e6f75, 0x105ec76, 0x12551f82, 0xe03e9c81, 0x34f4f86a, 0xc69f7b69, 0xd5cf889d, 0x27a40b9e,
    0x79b737ba, 0x8bdcb4b9, 0x988c474d, 0x6ae7c44e, 0xbe2da0a5, 0x4c4623a6, 0x5f16d052, 0xad7d5351
};

uint32_t Crc32Table3[256] = {0};
#endif

uint8_t *htoa(uint8_t *p, uint32_t d)
{
    uint8_t *r = p;
    uint8_t ch;
    int i;

    p = r + 6;
    *(r + 8) = '\0';
    for (i = 0; i < 4; i++, p -= 3, d >>= 8)
    {
        ch = d & 0xff;
        (*p) = ch & 0xf0;
        (*p) >>= 4;
        if ((*p) < 10)
            *p += '0';
        else
            *p += ('A' - 10);
        p++;
        (*p) = ch & 0xf;
        if ((*p) < 10)
            *p += '0';
        else
            *p += ('A' - 10);
    }
    return r;
}


/*********************************************************************
*
*       Public code
*
**********************************************************************
*/

static FLASH_HandleTypeDef *Addr2Handle(uint32_t addr)
{
    int8_t id = 0;
    uint32_t i;

    for (i = 0; i < FLASH_MAX_INSTANCE; i++)
    {
        if ((addr >= spi_flash_handle[i].base_addr)
                && (addr < (spi_flash_handle[i].base_addr + spi_flash_handle[i].total_size)))
        {
            id = i;
            break;
        }
    }
    if (i >= FLASH_MAX_INSTANCE)
    {
        id = -1;
    }

    if (id < 0)
        return NULL;

    return &(spi_flash_handle[id].handle);
}

//#define DUAL_FLASH_AUTO_DETECT
extern uint16_t BSP_GetFlash1DIV(void);
extern uint16_t BSP_GetFlash2DIV(void);
extern void BSP_SetFlash1DIV(uint16_t div);
extern void BSP_SetFlash2DIV(uint16_t div);


int rt_hw_flash2_init()
{
    HAL_StatusTypeDef res = HAL_ERROR;

#if !defined(JLINK) || defined(JLINK_FLASH_2)
#ifdef BSP_ENABLE_QSPI2 // add qspi check to make sure config table exist
    qspi_configure_t flash_cfg2 = FLASH2_CONFIG;
    struct dma_config flash_dma2 = FLASH2_DMA_CONFIG;

    //HAL_PIN_SetFlash2();
    uint16_t div = BSP_GetFlash2DIV();
    flash_cfg2.base = HCPU_MPI_SBUS_ADDR(flash_cfg2.base);
    spi_flash_handle[1].handle.data_buf = (uint8_t *)&nand_buf[0];
    spi_flash_handle[1].handle.data_buf_w = (uint8_t *)&nand_buf[0];
    // todo, update freq when do not use FPGA any more
    //spi_flash_handle[1].handle.freq = flash_get_freq(RCC_CLK_MOD_FLASH2, div, 1);
    spi_flash_handle[1].handle.freq = 120000000 / div;
    // init hardware, set dma, clock
    res = HAL_FLASH_Init(&(spi_flash_handle[1]), &flash_cfg2, &spi_flash_dma_handle[1], &flash_dma2, div);
#if (DEBUG_JLINK+DEFAULT_TRACE)
    uint8_t hex[16];
    debug_print("\r\nFLASH2 INIT : ID 0x");
    debug_print((char *)htoa(hex, spi_flash_handle[1].dev_id));
    debug_print(" ,res ");
    debug_print((char *)htoa(hex, res));
    debug_print("\r\n");
#endif    
    if (res == HAL_OK)
        return 1;
#endif  // BSP_ENABLE_QSPI2
#endif //#if !defined(JLINK) || defined(JLINK_FLASH_2)

    return 0;
}

static int rt_hw_flash_init(void)
{
    int fen = 0;

    memset(spi_flash_handle, 0, sizeof(QSPI_FLASH_CTX_T)*FLASH_MAX_INSTANCE);

    if (rt_hw_flash2_init())
        fen |= FLASH2_ENABLED;

    return fen;
}


void init_clock()
{
    HAL_HPAON_EnableXT48();
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_HP_PERI, RCC_CLK_PERI_HXT48);

    if (1)// (PM_COLD_BOOT == SystemPowerOnModeGet())
    {
        HAL_PMU_EnableDLL(1);

        HAL_HPAON_StartGTimer();
        HAL_PMU_LpCLockSelect(PMU_LPCLK_XT32);

        HAL_RCC_HCPU_SetDiv(1, 1, 5);
        /* Configure in LCPU. */
        //HAL_LPAON_EnableXT48();

        //HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_HXT48);
        //HAL_RCC_LCPU_ClockSelect(RCC_CLK_MOD_LP_PERI, RCC_CLK_PERI_HXT48);
    }

    __HAL_SYSCFG_HPBG_EN();
    __HAL_SYSCFG_HPBG_VDDPSW_EN();

    HAL_RCC_HCPU_EnableDLL1(144000000);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_SYS, RCC_SYSCLK_DLL1);

    HAL_RCC_HCPU_EnableDLL2(120000000);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH1, RCC_CLK_FLASH_DLL2);
    HAL_RCC_HCPU_ClockSelect(RCC_CLK_MOD_FLASH2, RCC_CLK_FLASH_DLL2);
    HAL_RCC_HCPU_SetDiv(1, 1, 5);
    //HAL_RCC_LCPU_SetDiv(1, 1, 3);

    BSP_SetFlash1DIV(2);    // src 120, div 2, output 60m
    BSP_SetFlash2DIV(2);
}


/*********************************************************************
*
*       Init
*
*  Function description
*    Handles the initialization of the flash module.
*
*  Parameters
*    Addr: Flash base address
*    Freq: Clock frequency in Hz
*    Func: Specifies the action followed by Init() (e.g.: 1 - Erase, 2 - Program, 3 - Verify / Read)
*
*  Return value
*    0 O.K.
*    1 Error
*/

__attribute__((used)) int Init(uint32_t Addr, uint32_t Freq, uint32_t Func)
{
    uint32_t sysFreq;

    SCB_DisableICache();
    SCB_DisableDCache();

    HAL_MspInit();

#if (DEBUG_JLINK+DEFAULT_TRACE)

    uint32_t v1,v2,v3;

    debug_print("Build data: ");
    debug_print(__DATE__);
    debug_print("\n");
    uint8_t data[16];
    v1 = FLASH_JLINK_VERSION >> 24;
    v2 = (FLASH_JLINK_VERSION & 0xff0000)>>16;
    v3 = FLASH_JLINK_VERSION & 0xffff;
    
    debug_print("SDK Version: ");
    debug_print((char *)htoa(data, v1));
    debug_print(".");
    debug_print((char *)htoa(data, v2));
    debug_print(".");
    debug_print((char *)htoa(data, v3));
    debug_print("\n");


    debug_print("Init : Msp");    
#endif    
    sysFreq = HAL_RCC_GetSysCLKFreq(CORE_ID_HCPU);
    //if (sysFreq / 1000000 < 90)
    {
        init_clock();
    }

#if (DEBUG_JLINK+DEFAULT_TRACE)
    debug_print("Init : Clock");    
#endif        
    
    rt_hw_flash_init();


#if (DEBUG_JLINK+DEFAULT_TRACE)
    uint8_t hex[16];
    debug_print("Init : Addr-");
    debug_print((char *)htoa(hex, Addr));
    debug_print(" Func-");
    debug_print((char *)htoa(hex, Func));
    debug_print(" sysFreq-");
    debug_print((char *)htoa(hex, sysFreq));
    debug_print("\r\n");
#endif

    return 0;                                  // Finished without Errors
}


/*********************************************************************
*
*       UnInit
*
*  Function description
*    Handles the de-initialization of the flash module.
*
*  Parameters
*    Func: Caller type (e.g.: 1 - Erase, 2 - Program, 3 - Verify)
*
*  Return value
*    0 O.K.
*    1 Error
*/
__attribute__((used)) int UnInit(uint32_t Func)
{
    (void)Func;
    return 0;                                  // Finished without Errors
}


/*********************************************************************
*
*       EraseSector
*
*  Function description
*    Erases one flash sector.
*
*  Parameters
*    SectorAddr: Absolute address of the sector to be erased
*
*  Return value
*    0 O.K.
*    1 Error
*/
__attribute__((used)) int EraseSector(uint32_t SectorAddr)
{
#if DEBUG_JLINK
    uint8_t hex[16];
    debug_print("Erase : Addr-");
    debug_print((char *)htoa(hex, SectorAddr));
    debug_print(" Size-");
    debug_print((char *)htoa(hex, PAGE_SIZE));
    debug_print("\r\n");
#endif

    FLASH_HandleTypeDef *hflash = Addr2Handle(SectorAddr);
    uint32_t addr = SectorAddr >= hflash->base ? SectorAddr - hflash->base : SectorAddr;
    //int res = HAL_QSPIEX_FLASH_ERASE(Addr2Handle(SectorAddr), SectorAddr, PAGE_SIZE);
    int res = HAL_NAND_GET_BADBLK(Addr2Handle(SectorAddr), (addr >> 17));
    if (res) // block is bad, do not erase, or bad mark will be cover
        return -2;
    res = HAL_NAND_ERASE_BLK(hflash, SectorAddr);
    if (res) // erase fail
        return -1;


    return res;
}


/*********************************************************************
*
*       ProgramPage
*
*  Function description
*    Programs one flash page.
*
*  Parameters
*    DestAddr: Destination address
*    NumBytes: Number of bytes to be programmed (always a multiple of program page size, defined in FlashDev.c)
*    pSrcBuff: Point to the source buffer
*
*  Return value
*    0 O.K.
*    1 Error
*/
__attribute__((used)) int ProgramPage(uint32_t DestAddr, uint32_t NumBytes, uint8_t *pSrcBuff)
{
    int ret;

#if DEBUG_JLINK
    uint8_t hex[16];
    debug_print("Write :");
    debug_print((char *)htoa(hex, DestAddr));
    debug_print("\r\n");
#endif
    //ret = HAL_QSPIEX_FLASH_WRITE(Addr2Handle(DestAddr), DestAddr, pSrcBuff, NumBytes);
    ret = HAL_NAND_WRITE_WITHOOB(Addr2Handle(DestAddr), DestAddr, pSrcBuff, NumBytes, NULL, 0);


    return (ret < NumBytes) ? -1 : 0;
}


/*********************************************************************
*
*       Verify
*
*  Function description
*    Compares a specified number of bytes of a provided data
*    buffer with the content of the device
*
*  Parameters
*    Addr: Start address in memory which should be compared
*    NumBytes: Number of bytes to be compared
*    pBuff: Pointer to the data to be compared
*
*  Return value
*    == (Addr + NumBytes): O.K.
*    != (Addr + NumBytes): *not* O.K. (ideally the fail address is returned)
*
*/
#if 0
//#ifdef _USE_PRODUCTLINE
uint32_t Verify(uint32_t Addr, uint32_t NumBytes, uint8_t *pBuff)
{
    uint8_t *pFlash = (uint8_t *)Addr;
    uint32_t r = Addr + NumBytes;

#if DEBUG_JLINK
    uint8_t hex[16];
    debug_print("Verify : Addr-");
    debug_print((char *)htoa(hex, Addr));
    debug_print(" Size-");
    debug_print((char *)htoa(hex, NumBytes));
    debug_print(" Src-");
    debug_print((char *)htoa(hex, (uint32_t)pBuff));
    debug_print("\r\n");
#endif

    do
    {
        if (*pFlash != *pBuff)
        {
            r = (unsigned long)pFlash;
            break;
        }
        pFlash++;
        pBuff++;
    }
    while (--NumBytes);

    return r;
}
#endif


/*********************************************************************
*
*       BlankCheck
*
*  Function description
*    Checks if a memory region is blank
*
*  Parameters
*    Addr: Blank check start address
*    NumBytes: Number of bytes to be checked
*    BlankData: Pointer to the destination data
*
*  Return value
*    0: O.K., blank
*    1: O.K., *not* blank
*    < 0: Error
*
*/
#if 0
//#ifdef _USE_PRODUCTLINE
__attribute__((used)) int BlankCheck(uint32_t Addr, uint32_t NumBytes, uint8_t BlankData)
{
    uint8_t *pData;

#if DEBUG_JLINK
    uint8_t hex[16];
    debug_print("BlankCheck : Addr-");
    debug_print((char *)htoa(hex, Addr));
    debug_print(" Size-");
    debug_print((char *)htoa(hex, NumBytes));
    debug_print(" Data-");
    debug_print((char *)htoa(hex, BlankData));
    debug_print("\r\n");
#endif

    pData = (uint8_t *)Addr;
    do
    {
        if (*pData++ != BlankData)
        {
            return 1;
        }
    }
    while (--NumBytes);

    return 0;
}
#endif
/*********************************************************************
*
*       EraseChip
*
*  Function description
*    Erases the entire flash
*
*  Return value
*    0: O.K.
*    1: Error
*/
#if 0
//#ifdef _USE_PRODUCTLINE
__attribute__((used)) int EraseChip(void)
{
    int res = HAL_QSPIEX_FLASH_ERASE(Addr2Handle(FLASH_BASE), FLASH_BASE, FLASH_SIZE);

    //HAL_QSPIEX_CHIP_ERASE(Addr2Handle(FLASH_BASE), FLASH_BASE);
#if DEBUG_JLINK
    uint8_t hex[16];
    debug_print("EraseChip : Addr-");
    debug_print((char *)htoa(hex, FLASH_BASE));
    debug_print(" Size-");
    debug_print((char *)htoa(hex, FLASH_SIZE));
    debug_print("\r\n");
#endif

    return (0);                                  // Finished without Errors
}
#endif

/*********************************************************************
*
*       SEGGER_OPEN_Read
*
*  Function description
*    Reads a specified number of bytes into the provided buffer
*
*  Parameters
*    Addr: Start read address
*    NumBytes: Number of bytes to be read
*    pBuff: Pointer to the destination data
*
*  Return value
*    >= 0: O.K., NumBytes read
*    <  0: Error
*
*/
#ifdef _USE_PRODUCTLINE
__attribute__((used)) int SEGGER_OPEN_Read(uint32_t Addr, uint32_t NumBytes, uint8_t *pDestBuff)
{
    FLASH_HandleTypeDef *fhandle = Addr2Handle(Addr);
    int ret;

#if DEBUG_JLINK
    uint8_t hex[16];
    debug_print("SEGGER_OPEN_Read : Addr-");
    debug_print((char *)htoa(hex, Addr));
    debug_print(" Size-");
    debug_print((char *)htoa(hex, NumBytes));
    debug_print(" DstAddr-");
    debug_print((char *)htoa(hex, (uint32_t)pDestBuff));
    debug_print("\r\n");
#endif

    if (fhandle->isNand == 0)
        memcpy(pDestBuff, (uint8_t *)Addr, NumBytes);
    else
    {
        if ((Addr & 2047) != 0)
            return 0;
        uint32_t len = NumBytes;
        uint8_t *dst = pDestBuff;
        uint32_t src = Addr;

        while (len >= 2048)
        {
            //SCB_InvalidateDCache_by_Addr((void *)fhandle->base, 2048);
            ret = HAL_NAND_READ_PAGE(fhandle, src, dst, 2048);
            if(ret != 2048)
                return 0;
            HAL_Delay_us(50);
            //__DSB();
            //__ASM volatile ("dsb");
            src += 2048;
            dst += 2048;
            len -= 2048;
        }
        if (len > 0)
        {
            //SCB_InvalidateDCache_by_Addr((void *)fhandle->base, 2048);
            //ret = HAL_NAND_READ_PAGE(fhandle, src, dst, len);
            ret = HAL_NAND_READ_WITHOOB(fhandle, src, dst, len, NULL, 0);
            if(ret != len)
                return 0;
        }
    }

    return NumBytes;
}
#endif
/*********************************************************************
*
*       SEGGER_OPEN_Program
*
*  Function description
*    Programs a specified number of bytes into the target flash.
*    NumBytes is either FlashDevice.PageSize or a multiple of it.
*
*  Notes
*    (1) This function can rely on that at least FlashDevice.PageSize will be passed
*    (2) This function must be able to handle multiple of FlashDevice.PageSize
*
*  Parameters
*    Addr: Start read address
*    NumBytes: Number of bytes to be read
*    pBuff: Pointer to the destination data
*
*  Return value
*    0 O.K.
*    1 Error
*
*/
#ifdef _USE_PRODUCTLINE
__attribute__((used)) int SEGGER_OPEN_Program(uint32_t DestAddr, uint32_t NumBytes, uint8_t *pSrcBuff)
{
    int ret = -1;
    int verify = -1;
    FLASH_HandleTypeDef *fhandle = Addr2Handle(DestAddr);
#if (DEBUG_JLINK+DEFAULT_TRACE)
    uint8_t hex[16];
#endif
#if DEBUG_JLINK
    debug_print("Write2 : Dst-");
    debug_print((char *)htoa(hex, DestAddr));
    debug_print(" Src-");
    debug_print((char *)htoa(hex, (uint32_t)pSrcBuff));
    debug_print(" Len-");
    debug_print((char *)htoa(hex, (uint32_t)NumBytes));
#endif
    if (fhandle->isNand)
    {
        int i;
        uint32_t *chk = (uint32_t *)pSrcBuff;
        uint32_t len = NumBytes;
        uint8_t *src = pSrcBuff;
        uint32_t dst = DestAddr;
        ret = 0;
        while (len >= 2048)
        {
            uint32_t *chk = (uint32_t *)src;
            for(i=0; i<2048/4; i++)
            {
                if(chk[i] != 0xffffffff)
                    break;
            }
            if(i == 2048/4) // full page data is 0xffffffff, do not need write to flash
            {
#if (DEBUG_JLINK)
                debug_print("SKIP page write for addr: 0x");
                debug_print((char *)htoa(hex, dst));
                debug_print(" \r\n");
#endif
                len -= 2048;
                dst += 2048;
                src += 2048;
                ret += 2048;
                continue;
            }
            //ret += HAL_QSPIEX_FLASH_WRITE(Addr2Handle(dst), dst, src, 2048);
            ret += HAL_NAND_WRITE_WITHOOB(Addr2Handle(dst), dst, src, 2048, NULL, 0);
            len -= 2048;
            dst += 2048;
            src += 2048;
        }
        if (len > 0)
        {
#if (DEBUG_JLINK+DEFAULT_TRACE)
            debug_print("Do not support not page aligned write\r\n");
#endif
            return -1; // only full page write support!
        }
    }
    else
    {
        ret = -1;
        //ret = HAL_QSPIEX_FLASH_WRITE(Addr2Handle(DestAddr), DestAddr, pSrcBuff, NumBytes);
    }

    if (ret >= NumBytes)
    {
        /*
        for (int m = 0; m < NumBytes / 0x1000; m++)
        {

            verify = memcmp((uint8_t *)(pSrcBuff + 0x1000 * m), (uint8_t *)(DestAddr + 0x1000 * m), 0x1000);
            if (verify != 0)
            {
                break;
            }
        }*/
        verify = 0;
    }

#if DEBUG_JLINK
    debug_print(" ret-");
    debug_print((char *)htoa(hex, (uint32_t)ret));
    debug_print(" verify-");
    debug_print((char *)htoa(hex, (uint32_t)verify));
    debug_print("\r\n");
#endif

    return verify;
}
#endif

/*********************************************************************
*
*       SEGGER_OPEN_Erase
*
*  Function description
*    Erases one or more flash sectors
*
*  Notes
*    (1) This function can rely on that at least one sector will be passed
*    (2) This function must be able to handle multiple sectors at once
*    (3) This function can rely on that only multiple sectors of the same sector
*        size will be passed. (e.g. if the device has two sectors with different
*        sizes, the DLL will call this function two times with NumSectors = 1)
*
*  Parameters
*    SectorAddr: Address of the start sector to be erased
*    SectorIndex: Index of the start sector to be erased
*    NumSectors: Number of sectors to be erased. At least 1 sector is passed.
*
*  Return value
*    0 O.K.
*    1 Error
*
*/
#ifdef _USE_PRODUCTLINE
__attribute__((used)) int SEGGER_OPEN_Erase(uint32_t SectorAddr, uint32_t SectorIndex, uint32_t NumSectors)
{
    int res, i;

#if DEBUG_JLINK
    uint8_t hex[16];
    debug_print("SEGGER_OPEN_Erase : Addr-");
    debug_print((char *)htoa(hex, SectorAddr));
    debug_print(" Index-");
    debug_print((char *)htoa(hex, SectorIndex));
    debug_print(" Sectors-");
    debug_print((char *)htoa(hex, NumSectors));
    debug_print(" Size-");
    debug_print((char *)htoa(hex, PAGE_SIZE));
#endif

    //res = HAL_QSPIEX_FLASH_ERASE(Addr2Handle(SectorAddr), SectorAddr, NumSectors * SPI_NAND_BLK_SIZE);
    for (i = 0; i < NumSectors; i++)
    {
        //res = HAL_NAND_GET_BADBLK(Addr2Handle(SectorAddr), (SectorAddr >> 17) + i);
        //if (res) // block is bad, do not erase, or bad mark will be cover
        //    return -2;
        res = HAL_NAND_ERASE_BLK(Addr2Handle(SectorAddr), SectorAddr + i * SPI_NAND_BLK_SIZE);
        if (res) // erase fail
        {
            res = -1;
            break;
        }
    }


__EXIT:

#if DEBUG_JLINK
    debug_print(" res-");
    debug_print((char *)htoa(hex, res));
    debug_print("\r\n");
#endif

    return res;
}
#endif



/*********************************************************************
*
*       SEGGER_OPEN_CalcCRC
*
*  Function description
*    Calculates the CRC over a specified number of bytes
*    Even more optimized version of Verify() as this avoids downloading the compare data into the RAMCode for comparison.
*    Heavily reduces traffic between J-Link software and target and therefore speeds up verification process significantly.
*
*  Parameters
*    CRC       CRC start value
*    Addr      Address where to start calculating CRC from
*    NumBytes  Number of bytes to calculate CRC on
*    Polynom   Polynom to be used for CRC calculation
*
*  Return value
*    CRC
*
*  Notes
*    (1) This function is optional
*    (2) Use "noinline" attribute to make sure that function is never inlined and label not accidentally removed by linker from ELF file.
*/

#ifdef _USE_PRODUCTLINE
void init_CRC32_table(uint32_t Polynom)
{
    for (int i = 0; i < 256; i++)
    {
        uint32_t crcs = i;

        for (int j = 0; j < 8; j++)
        {
            if (crcs & 1)
            {
                crcs = (crcs >> 1) ^ Polynom;
            }
            else
            {
                crcs >>= 1;
            }
        }
        Crc32Table3[i] = crcs;
    }
}

static uint8_t nand_cache[PAGE_SIZE];

uint32_t SEGGER_OPEN_CalcCRC(uint32_t crcs, uint32_t Addr, uint32_t NumBytes, uint32_t Polynom)
{
    FLASH_HandleTypeDef *fhandle = Addr2Handle(Addr);
    uint8_t   *pCrcData;
    uint32_t  *pCrcTable = NULL;

#if (DEBUG_JLINK)
    uint8_t hex[16];
    debug_print("SEGGER_OPEN_CalcCRC : crcs-");
    debug_print((char *)htoa(hex, crcs));
    debug_print(" Addr-");
    debug_print((char *)htoa(hex, Addr));
    debug_print(" NumBytes-");
    debug_print((char *)htoa(hex, NumBytes));
    debug_print(" Polynom-");
    debug_print((char *)htoa(hex, Polynom));
    debug_print("\r\n");
#endif

    if (fhandle)
    {
        if (fhandle->isNand) // nand
        {
            pCrcData = (uint8_t *)(&nand_cache[0]);
            int ret;
            {
                ret = SEGGER_OPEN_Read(Addr, NumBytes, pCrcData);
            }
        }
        else // nor
            pCrcData = (uint8_t *)Addr;
    }
    else
        return 0;

    if (Polynom == 0xEDB88320)
    {
        pCrcTable = Crc32Table1;
    }
    else if (Polynom == 0x82F63B78)
    {
        pCrcTable = Crc32Table2;
    }
    else
    {
        init_CRC32_table(Polynom);
        pCrcTable = Crc32Table3;
    }

    crcs = 0;
    {
        /* Always reprogram all if flashing file system */
        while (NumBytes--)
        {
            crcs = (crcs >> 8) ^ pCrcTable[(crcs & 0xFF) ^ *pCrcData++];
        }
    }

#if (DEBUG_JLINK)
    debug_print(" crc-");
    debug_print((char *)htoa(hex, crcs));
    debug_print("\r\n");
#endif

    return crcs;
}
#endif


/**************************** End of file ***************************/
