/**
  ******************************************************************************
  * @file   flash_table.c
  * @author Sifli software development team
  * @brief Flash command table list
  ******************************************************************************
*/

#include <string.h>
#include "flash_table.h"

typedef enum
{
    NOR_TYPE0 = 0,  // normal type 0, DTR, NO CMD_WRSR2, Max 128Mb, as default command table
    NOR_TYPE1,      // type 1, WRSR2 to write status register 2(QE), Max 128Mb
    NOR_TYPE2,      // type 2, 256Mb, DTR, 4 bytes address command diff with 3 bytes, OTP support 4-B mode
    NOR_TYPE3,      // type 3, 256Mb , NO DTR , 4 bytes command same to 3 bytes, only timing changed, OTP 3-B only
    NOR_TYPE4,      // type 4, 256Mb, NO DTR, 4B ADDR command diff with 3B addr , OTP support 4-B mode
    NOR_TYPE5,      // type 5, 256Mb, NO DTR, MXIC flash have too many diff with others
    NOR_CMD_TABLE_CNT
} FLASH_CMD_TABLE_ID_T;

#define FLASH_DEFAULT_CMD_TABLE         (NOR_TYPE0)

/* For bootloader, need compress to reduce code size */
__weak FT_CONST SPI_FLASH_FACT_CFG_T flash_cmd_table_list[] =
{
    {
        // type 0
        0,    /* NOR flash */
        0X85, /* puya manuf id */
        0x60,
        0x15, /* memory density */
        0x0,
        0x0,
        0x3,
        0x0,
        0x0,
        0x0,
        0x0,
        0x1,
        4096,
        {
            {0x06, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WREN*/
            {0x04, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRDI*/
            {0x05, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDSR*/
            {0x01, 1, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRSR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PREAD*/
            {0x03, 0, 1, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_READ*/
            {0x0b, 0, 1, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_FREAD*/
            {0x3b, 0, 2, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_DREAD*/
            {0x6b, 0, 3, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_QREAD*/
            {0xbb, 0, 2, 0, 0, 2, 2, 2, 1}, /* SPI_FLASH_CMD_2READ*/
            {0xeb, 0, 3, 4, 0, 3, 2, 3, 1}, /* SPI_FLASH_CMD_4READ*/
            {0x9f, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDID*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PLD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPLD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PLDR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPLDR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PEXE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE*/
            {0x99, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RST*/
            {0x66, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RST_EN*/
            {0x35, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDSR2*/
            {0x50, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WVSR*/
            {0x02, 1, 1, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PP*/
            {0x32, 1, 3, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_QPP*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDEAR*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WREAR*/
            {0x81, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PE*/
            {0x20, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_SE*/
            {0x52, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_BE32*/
            {0xd8, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_BE64*/
            {0x60, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_CE*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSR3*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WRSR3*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_EN4BM*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_ET4BM*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RD4BA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_FR4BA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_FQR4BA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_4RD4BA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PP4BA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPP4BA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_SE4BA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE4BA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WRSR2*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_LEFPA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BBM*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RBLUT*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_CFREAD*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_C4READ*/
            {0x4b, 0, 1, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_RUID*/
            {0x48, 0, 1, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_RDSCUR*/
            {0x42, 1, 1, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PRSCUR*/
            {0x44, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_ERSCUR*/
            {0xb9, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_DPD*/
            {0xab, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDP*/
            {0xed, 0, 7, 7, 0, 7, 2, 7, 1}, /* SPI_FLASH_CMD_DTR4R*/
            {0x5a, 0, 1, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_RDSFDP*/
        }
    },
    {
        // type 1
        0,    /* NOR flash */
        0X85, /* puya manuf id */
        0x60,
        0x16, /* memory density */
        0x0,
        0x0,
        0x3,
        0x0,
        0x0,
        0x0,
        0x0,
        0x1,
        4096,
        {
            {0x06, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WREN*/
            {0x04, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRDI*/
            {0x05, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDSR*/
            {0x01, 1, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRSR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PREAD*/
            {0x03, 0, 1, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_READ*/
            {0x0b, 0, 1, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_FREAD*/
            {0x3b, 0, 2, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_DREAD*/
            {0x6b, 0, 3, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_QREAD*/
            {0xbb, 0, 2, 0, 0, 2, 2, 2, 1}, /* SPI_FLASH_CMD_2READ*/
            {0xeb, 0, 3, 4, 0, 3, 2, 3, 1}, /* SPI_FLASH_CMD_4READ*/
            {0x9f, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDID*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PLD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPLD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PLDR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPLDR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PEXE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE*/
            {0x99, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RST*/
            {0x66, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RST_EN*/
            {0x35, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDSR2*/
            {0x50, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WVSR*/
            {0x02, 1, 1, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PP*/
            {0x32, 1, 3, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_QPP*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDEAR*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WREAR*/
            {0x81, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PE*/
            {0x20, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_SE*/
            {0x52, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_BE32*/
            {0xd8, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_BE64*/
            {0x60, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_CE*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSR3*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WRSR3*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_EN4BM*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_ET4BM*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RD4BA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_FR4BA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_FQR4BA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_4RD4BA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PP4BA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPP4BA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_SE4BA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE4BA*/
            {0x31, 1, 1, 0, 0, 0, 0, 0,  1}, /* SPI_FLASH_CMD_WRSR2*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_LEFPA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BBM*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RBLUT*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_CFREAD*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_C4READ*/
            {0x4b, 0, 1, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_RUID*/
            {0x48, 0, 1, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_RDSCUR*/
            {0x42, 1, 1, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PRSCUR*/
            {0x44, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_ERSCUR*/
            {0xb9, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_DPD*/
            {0xab, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDP*/
            {0xed, 0, 7, 7, 0, 7, 2, 7, 1}, /* SPI_FLASH_CMD_DTR4R*/
            //{0xed, 0, 7, 8, 0, 7, 2, 7, 1}, /* SPI_FLASH_CMD_DTR4R*/
            {0x5a, 0, 1, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_RDSFDP*/
        }
    },
#ifndef SIMP_FLASHTAB
    {
        // type 2
        0,    /* NOR flash */
        0Xef, /* winbond manuf id */
        0x60,
        0x19, /* device id, for winbond, they have 16 bit device, just use 8 now  */
        0x0,
        0x0,
        0x3,
        0x0,
        0x0,
        0x0,
        0x0,
        0x1,
        4096,
        {
            {0x06, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WREN*/
            {0x04, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRDI*/
            {0x05, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDSR*/
            {0x01, 1, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRSR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PREAD*/
            {0x03, 0, 1, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_READ*/
            {0x0b, 0, 1, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_FREAD*/
            {0x3b, 0, 2, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_DREAD*/
            {0x6b, 0, 3, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_QREAD*/
            {0xbb, 0, 2, 0, 0, 2, 2, 2, 1}, /* SPI_FLASH_CMD_2READ*/
            {0xeb, 0, 3, 4, 0, 3, 2, 3, 1}, /* SPI_FLASH_CMD_4READ*/
            {0x9f, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDID*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PLD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPLD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PLDR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPLDR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PEXE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE*/
            {0x99, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RST*/
            {0x66, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RST_EN*/
            {0x35, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDSR2*/
            {0x50, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WVSR*/
            {0x02, 1, 1, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PP*/
            {0x32, 1, 3, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_QPP*/
            {0xc8, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDEAR*/
            {0xc5, 1, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WREAR*/
            {0x81, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PE*/
            {0x20, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_SE*/
            {0x52, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_BE32*/
            {0xd8, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_BE64*/
            {0x60, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_CE*/
            {0x15, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDSR3*/
            {0x11, 1, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRSR3*/
            {0xb7, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_EN4BM*/
            {0xe9, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_ET4BM*/
            {0x13, 0, 1, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_RD4BA*/
            {0x0c, 0, 1, 8, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_FR4BA*/
            {0x6c, 0, 3, 8, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_FQR4BA*/
            {0xec, 0, 3, 4, 0, 3, 3, 3, 1}, /* SPI_FLASH_CMD_4RD4BA*/
            {0x12, 1, 1, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_PP4BA*/
            {0x34, 1, 3, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_QPP4BA*/
            {0x21, 0, 0, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_SE4BA*/
            {0xDC, 0, 0, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_BE4BA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WRSR2*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_LEFPA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BBM*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RBLUT*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_CFREAD*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_C4READ*/
            {0x4b, 0, 1, 8, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_RUID*/
            {0x48, 0, 1, 8, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_RDSCUR*/
            {0x42, 1, 1, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_PRSCUR*/
            {0x44, 0, 0, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_ERSCUR*/
            {0xb9, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_DPD*/
            {0xab, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDP*/
            {0xed, 0, 7, 7, 0, 7, 3, 7, 1}, /* SPI_FLASH_CMD_DTR4R*/
            {0x5a, 0, 1, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_RDSFDP*/
        }
    },
    {
        //type 3
        0,    /* NOR flash */
        0Xc8, /* giga manuf id */
        0x60,
        0x19,
        0x0,
        0x0,
        0x3,
        0x0,
        0x0,
        0x0,
        0x0,
        0x1,
        4096,
        {
            {0x06, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WREN*/
            {0x04, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRDI*/
            {0x05, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDSR*/
            {0x01, 1, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRSR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PREAD*/
            {0x03, 0, 1, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_READ*/
            {0x0b, 0, 1, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_FREAD*/
            {0x3b, 0, 2, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_DREAD*/
            {0x6b, 0, 3, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_QREAD*/
            {0xbb, 0, 2, 0, 0, 2, 2, 2, 1}, /* SPI_FLASH_CMD_2READ*/
            {0xeb, 0, 3, 4, 0, 3, 2, 3, 1}, /* SPI_FLASH_CMD_4READ*/
            {0x9f, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDID*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PLD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPLD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PLDR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPLDR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PEXE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE*/
            {0x99, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RST*/
            {0x66, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RST_EN*/
            {0x35, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDSR2*/
            {0x50, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WVSR*/
            {0x02, 1, 1, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PP*/
            {0x32, 1, 3, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_QPP*/
            {0xc8, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDEAR*/
            {0xc5, 1, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WREAR*/
            {0x81, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PE*/
            {0x20, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_SE*/
            {0x52, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_BE32*/
            {0xd8, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_BE64*/
            {0x60, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_CE*/
            {0x15, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDSR3*/
            {0x11, 1, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRSR3*/
            {0xb7, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_EN4BM*/
            {0xe9, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_ET4BM*/
            {0x03, 0, 1, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_RD4BA*/
            {0x0b, 0, 1, 8, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_FR4BA*/
            {0x6b, 0, 3, 8, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_FQR4BA*/
            {0xeb, 0, 3, 4, 0, 3, 3, 3, 1}, /* SPI_FLASH_CMD_4RD4BA*/
            {0x02, 1, 1, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_PP4BA*/
            {0x32, 1, 3, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_QPP4BA*/
            {0x20, 0, 0, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_SE4BA*/
            {0xD8, 0, 0, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_BE4BA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WRSR2*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_LEFPA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BBM*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RBLUT*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_CFREAD*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_C4READ*/
            {0x4b, 0, 1, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_RUID*/
            {0x48, 0, 1, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_RDSCUR*/
            {0x42, 1, 1, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PRSCUR*/
            {0x44, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_ERSCUR*/
            {0xb9, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_DPD*/
            {0xab, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDP*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_DTR4R*/
            {0x5a, 0, 1, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_RDSFDP*/
        }
    },
    {
        //type 4
        0,    /* NOR flash */
        0X20, /* wuhan xinxin manuf id */
        0x41,
        0x19, /* mem cap */
        0x0,
        0x0,
        0x3,
        0x0,
        0x0,
        0x0,
        0x0,
        0x1,
        4096,
        {
            {0x06, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WREN*/
            {0x04, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRDI*/
            {0x05, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDSR*/
            {0x01, 1, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRSR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PREAD*/
            {0x03, 0, 1, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_READ*/
            {0x0b, 0, 1, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_FREAD*/
            {0x3b, 0, 2, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_DREAD*/
            {0x6b, 0, 3, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_QREAD*/
            {0xbb, 0, 2, 0, 0, 2, 2, 2, 1}, /* SPI_FLASH_CMD_2READ*/
            {0xeb, 0, 3, 4, 0, 3, 2, 3, 1}, /* SPI_FLASH_CMD_4READ*/
            {0x9f, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDID*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PLD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPLD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PLDR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPLDR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PEXE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE*/
            {0x99, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RST*/
            {0x66, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RST_EN*/
            {0x35, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDSR2*/
            {0x50, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WVSR*/
            {0x02, 1, 1, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PP*/
            {0x32, 1, 3, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_QPP*/
            {0xc8, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDEAR*/
            {0xc5, 1, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WREAR*/
            {0x81, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PE*/
            {0x20, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_SE*/
            {0x52, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_BE32*/
            {0xd8, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_BE64*/
            {0x60, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_CE*/
            {0x15, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDSR3*/
            {0x11, 1, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRSR3*/
            {0xb7, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_EN4BM*/
            {0xe9, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_ET4BM*/
            {0x13, 0, 1, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_RD4BA*/
            {0x0c, 0, 1, 8, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_FR4BA*/
            {0x6c, 0, 3, 8, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_FQR4BA*/
            {0xec, 0, 3, 4, 0, 3, 3, 3, 1}, /* SPI_FLASH_CMD_4RD4BA*/
            {0x12, 1, 1, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_PP4BA*/
            {0x34, 1, 3, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_QPP4BA*/
            {0x21, 0, 0, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_SE4BA*/
            {0xDC, 0, 0, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_BE4BA*/
            {0x31, 1, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRSR2*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_LEFPA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BBM*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RBLUT*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_CFREAD*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_C4READ*/
            {0x4b, 0, 1, 8, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_RUID*/
            {0x48, 0, 1, 8, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_RDSCUR*/
            {0x42, 1, 1, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_PRSCUR*/
            {0x44, 0, 0, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_ERSCUR*/
            {0xb9, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_DPD*/
            {0xab, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDP*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_DTR4R*/
            {0x5a, 0, 1, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_RDSFDP*/
        }
    },
    {
        // type 5
        0,    /* NOR flash */
        0Xc2, /* MXIC manuf id */
        0x25,
        0x39,
        0x0,
        0x0,
        0x0,
        0x3c,
        0x60,
        0x0,
        0x0,
        0x0,
        4096,
        {
            {0x06, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WREN*/
            {0x04, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRDI*/
            {0x05, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDSR*/
            {0x01, 1, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRSR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PREAD*/
            {0x03, 0, 1, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_READ*/
            {0x0b, 0, 1, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_FREAD*/
            {0x3b, 0, 2, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_DREAD*/
            {0x6b, 0, 3, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_QREAD*/
            {0xbb, 0, 2, 0, 0, 2, 2, 2, 1}, /* SPI_FLASH_CMD_2READ*/
            {0xeb, 0, 3, 4, 0, 3, 2, 3, 1}, /* SPI_FLASH_CMD_4READ*/
            {0x9f, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDID*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PLD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPLD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PLDR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPLDR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PEXE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE*/
            {0x99, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RST*/
            {0x66, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RST_EN*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSR2*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WVSR*/
            {0x02, 1, 1, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PP*/
            {0x32, 1, 3, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_QPP*/
            {0xc8, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDEAR*/
            {0xc5, 1, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WREAR*/
            {0x81, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PE*/
            {0x20, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_SE*/
            {0x52, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_BE32*/
            {0xd8, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_BE64*/
            {0x60, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_CE*/
            {0x15, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDSR3*/
            {0x11, 1, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRSR3*/
            {0xb7, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_EN4BM*/
            {0xe9, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_ET4BM*/
            {0x03, 0, 1, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_RD4BA*/
            {0x0b, 0, 1, 8, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_FR4BA*/
            {0x6b, 0, 3, 8, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_FQR4BA*/
            {0xeb, 0, 3, 4, 0, 3, 3, 3, 1}, /* SPI_FLASH_CMD_4RD4BA*/
            {0x02, 1, 1, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_PP4BA*/
            {0x32, 1, 3, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_QPP4BA*/
            {0x20, 0, 0, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_SE4BA*/
            {0xD8, 0, 0, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_BE4BA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WRSR2*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_LEFPA*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BBM*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RBLUT*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_CFREAD*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_C4READ*/
            {0x4b, 0, 1, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_RUID*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSCUR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PRSCUR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_ERSCUR*/
            {0xb9, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_DPD*/
            {0xab, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDP*/
            {0x00, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_DTR4R*/
            {0x5a, 0, 1, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_RDSFDP*/
        }
    },
#endif
};

FT_CONST FLASH_RDID_TYPE_T flash_cmd_id_pool_typ0[] =
{
    {0x85, 0x60, 0x15, 1, 0x200000},    //P25Q16LE_RDID
    {0x85, 0x60, 0x14, 1, 0x100000},    //P25Q80LE_RDID
    {0x85, 0x60, 0x13, 0, 0x80000},    //P25Q40SU_RDID
    {0xc8, 0x60, 0x14, 0, 0x100000},    //GD25LE80E_RDID
    {0xc8, 0x60, 0x15, 0, 0x200000},    //GD25LE16E_RDID
    {0xc8, 0x60, 0x16, 0, 0x400000},    //GD25LE32E_RDID
    {0xc8, 0x60, 0x17, 0, 0x800000},    //GD25LE64E_RDID
    {0xc8, 0x60, 0x18, 0, 0x1000000},   //GD25LQ128E_RDID
    {0x0B, 0x40, 0x17, 0, 0x800000},    //XT25F64BW_RDID
    {0xf8, 0x42, 0x18, 0, 0x1000000},   //FM25M4AA_RDID
    {0x5e, 0x70, 0x17, 0, 0x800000},    //ZB25LQ64A_RDID
    {0x5e, 0x40, 0x18, 0, 0x1000000},   //ZB25VQ128_SPI
    {0xef, 0x60, 0x16, 0, 0x400000},    //W25Q32JW_RDID
    {0x20, 0x40, 0X18, 1, 0x1000000},   //XM25QH128D ???
    {0x20, 0x41, 0X18, 0, 0x1000000},   //XM25QU128C
    {0x20, 0x50, 0x18, 0, 0x1000000},   //XM25QU128B
    {0xeb, 0x60, 0x13, 0, 0x100000},    //TH25Q-80UA_RDID   // reused command, but otp only 512 x 3
    {0xeb, 0x60, 0x14, 0, 0x100000},    //TH25Q-80U_RDID    // reused command, but otp only 512 x 3
    {0x1c, 0x40, 0x18, 0, 0x1000000},   //GM25Q128A
    {0x68, 0x40, 0x17, 0, 0x800000},   //BY25Q64ES_RDID
    {0x68, 0x40, 0x18, 0, 0x1000000},   //BY25Q128ES_RDID
    {FLASH_INVALID_ID, 0, 0, 0, 0},      //last one
};

FT_CONST FLASH_RDID_TYPE_T flash_cmd_id_pool_typ1[] =
{
    {0x85, 0x60, 0x16, 1, 0x400000},    //P25Q32L_RDID
    {0x85, 0x60, 0x17, 1, 0x800000},    //P25Q64H_RDID, P25Q64SH
    {0x85, 0x60, 0x18, 1, 0x1000000},   //P25Q128L_RDID
    {0x85, 0x65, 0x18, 1, 0x1000000},   //P25Q128LA_RDID
    {0x85, 0x63, 0x18, 1, 0x1000000},   //PY25F128LA
    {0x85, 0x20, 0x17, 1, 0x800000},    //PY25Q64HA
    {0x85, 0x20, 0x18, 1, 0x1000000},   //PY25Q128HA
    {0x25, 0x70, 0x16, 0, 0x400000},    //SK25LE032_RDID
    {0xef, 0x60, 0x18, 0, 0x1000000},   //W25Q128JW_RDID
    {0xef, 0x40, 0x18, 0, 0x1000000},   //W25Q128JV_RDID
    {0x0B, 0x60, 0X18, 0, 0x1000000},   //XT25Q128DW_RDID
    {0x0B, 0x40, 0X18, 1, 0x1000000},   //XT25F128F_RDID
    {0x5e, 0x50, 0x18, 0, 0x1000000},   //ZB25LQ128BW_RDID
    {0xa1, 0x28, 0x18, 0, 0x1000000},   //FM25W128_RDID
    {0x68, 0x40, 0x18, 0, 0x1000000},   //BY25Q128ES_RDID
    {0xc8, 0x43, 0x18, 1, 0x1000000},   //GD25Q128F_RDID
    {0xc8, 0x40, 0x18, 0, 0x1000000},   //MD25Q128_RDID
    {0xc8, 0x40, 0x17, 0, 0x800000},    //MD25Q64C_RDID
    {0x20, 0x40, 0X17, 1, 0x800000},    //XM25QH64D
    {0xcd, 0x60, 0X18, 0, 0x1000000},   //TH25Q128HA_RDID
    {FLASH_INVALID_ID, 0, 0, 0, 0},      //last one
};

FT_CONST FLASH_RDID_TYPE_T flash_cmd_id_pool_typ2[] =
{
    {0xef, 0x60, 0x19, 0, 0x2000000},   //W25Q256JW_RDID
    {0xef, 0x70, 0x19, 1, 0x2000000},   //W25Q256JV_RDID
    {0xE5, 0x42, 0x19, 0, 0x2000000},   //DS25M4BA_RDID
    {0xef, 0x40, 0x19, 0, 0x2000000},   //W25Q256JVM_RDID
    {0x68, 0x49, 0x19, 0, 0x2000000},   //BY25Q256FS
    {0x5e, 0x40, 0x19, 0, 0x2000000},   //ZQ25Q256AW1G
    {FLASH_INVALID_ID, 0, 0, 0, 0},      //last one
};
FT_CONST FLASH_RDID_TYPE_T flash_cmd_id_pool_typ3[] =
{
    {0xc8, 0x60, 0x19, 0, 0x2000000},   //GD25LQ256D_RDID
    {0x0B, 0x40, 0X19, 0, 0x2000000},   //XT25F256BW_RDID
    {0xc8, 0x40, 0x19, 0, 0x2000000},   //GD25Q256E
    {FLASH_INVALID_ID, 0, 0, 0, 0},      //last one
};
FT_CONST FLASH_RDID_TYPE_T flash_cmd_id_pool_typ4[] =
{
    {0x20, 0x41, 0x19, 0, 0x2000000},   //XM25QU256C_RDID
    {0x20, 0x40, 0x19, 0, 0x2000000},   //XM25QH256C_RDID
    {0x20, 0x43, 0x19, 0, 0x2000000},   //XM25RH256C_RDID
    {0x85, 0x20, 0x19, 1, 0x2000000},   //PY25Q256HB_RDID
    {0x85, 0x20, 0x1a, 1, 0x4000000},   //PY25Q512HB_RDID
    {FLASH_INVALID_ID, 0, 0, 0, 0},      //last one
};
FT_CONST FLASH_RDID_TYPE_T flash_cmd_id_pool_typ5[] =
{
    {0xc2, 0x25, 0x39, 0, 0x2000000},   //MX25U25643G_RDID
    {0x25, 0x70, 0x18, 0, 0x1000000},   //SK25LP128_RDID
    {FLASH_INVALID_ID, 0, 0, 0, 0},      //last one
};


#if defined(JLINK) || defined(KEIL)
FLASH_RDID_TYPE_T *flash_cmd_id_pool[NOR_CMD_TABLE_CNT];
#else
__weak FT_CONST FLASH_RDID_TYPE_T *FT_CONST flash_cmd_id_pool[] =
{
    &flash_cmd_id_pool_typ0[0], // type 0, < 32m
    &flash_cmd_id_pool_typ1[0], // type 1, < 32m
    &flash_cmd_id_pool_typ2[0], // type 2, 32m
    &flash_cmd_id_pool_typ3[0], // type 3, 32m
    &flash_cmd_id_pool_typ4[0], // type 4, 32m
    &flash_cmd_id_pool_typ5[0], // type 5, 32m
};
#endif /* JLINK || KEIL */

#if defined(JLINK) || defined(KEIL)
__weak void spi_nor_table_init(void)
{
    if (NOR_CMD_TABLE_CNT != 6)
    {
        while (1);
    }
    flash_cmd_id_pool[0] = &flash_cmd_id_pool_typ0[0];
    flash_cmd_id_pool[1] = &flash_cmd_id_pool_typ1[0];
    flash_cmd_id_pool[2] = &flash_cmd_id_pool_typ2[0];
    flash_cmd_id_pool[3] = &flash_cmd_id_pool_typ3[0];
    flash_cmd_id_pool[4] = &flash_cmd_id_pool_typ4[0];
    flash_cmd_id_pool[5] = &flash_cmd_id_pool_typ5[0];
}
#endif /* JLINK || KEIL */




#if defined(CFG_FACTORY_DEBUG)
__WEAK void *get_user_flash_cfg(uint8_t isnand, uint8_t fid, uint8_t did, uint8_t type, uint8_t *flash_type)
{
    return NULL;
}
#endif

FT_CONST FLASH_RDID_TYPE_T *spi_flash_get_rdid(uint8_t fid, uint8_t did, uint8_t type, uint8_t *flash_type)
{
    int i;
    FT_CONST FLASH_RDID_TYPE_T *res;

    // check flash id valid
    if ((fid == FLASH_INVALID_ID) || (fid == FLASH_UNKNOW_ID))
        return NULL;

    for (i = 0; i < NOR_CMD_TABLE_CNT; i++)
    {
        res = flash_cmd_id_pool[i];
        while (res->manufacture_id != FLASH_INVALID_ID)
        {
            if (fid == res->manufacture_id &&
                    type == res->memory_type &&
                    did == res->memory_density)
                break;
            res++;
        };
        if (res->manufacture_id != FLASH_INVALID_ID)      // Found flash ID
            break;
    }
    if (i == NOR_CMD_TABLE_CNT)
    {
#if defined(CFG_FACTORY_DEBUG)
        res = (FLASH_RDID_TYPE_T *)get_user_flash_cfg(0, fid, did, type, flash_type);
#else
        res = NULL;
#endif
        //
    }
    else if (flash_type)
    {
        *flash_type = i;
    }
    return res;
}

// get command table by index from register table
const SPI_FLASH_FACT_CFG_T *spi_flash_get_cmd_by_id(uint8_t fid, uint8_t did, uint8_t type)
{
    uint8_t i;
    const SPI_FLASH_FACT_CFG_T *res = NULL;

    FT_CONST FLASH_RDID_TYPE_T *rdid = spi_flash_get_rdid(fid, did, type, &i);

    if (rdid)
        res = (const SPI_FLASH_FACT_CFG_T *)&flash_cmd_table_list[i];
    else // set a default table for nor flash
        res = &flash_cmd_table_list[FLASH_DEFAULT_CMD_TABLE];
    return res;
}

int spi_flash_get_size_by_id(uint8_t fid, uint8_t did, uint8_t type)
{
    int res = 0x80000;

    FT_CONST FLASH_RDID_TYPE_T *rdid = spi_flash_get_rdid(fid, did, type, NULL);
    if (rdid)
        res = rdid->mem_size;

    return res;
}

int spi_flash_is_support_dtr(uint8_t fid, uint8_t did, uint8_t type)
{
    int res = 0;

    FT_CONST FLASH_RDID_TYPE_T *rdid = spi_flash_get_rdid(fid, did, type, NULL);
    if (rdid)
        res = (rdid->ext_flags & DTR_SUPPORT_FLAG);

    return res;
}


