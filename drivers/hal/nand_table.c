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
    NAND_TYPE0 = 0,  // normal type, base on winbond w25n01gw, with NON-BUF, NO QE, EB with 4 dummy
    NAND_TYPE1,      // based on XT26G01D, BUF, QE, EB, EB with 2 dummy
    NAND_TYPE2,      // based on ds35x1gaxxx, BUF , QE, NO EB
    NAND_TYPE3,      // based on tc58cyg0s3hraij, BUF, NO QE, NO EB
    NAND_TYPE4,      // based on FM25LS01, BUF, NO QE, EB   with 4 dummy
    NAND_TYPE5,      // based on GD5F1GM7RE, BUF, QE, EB, EB with 4 dummy
    NAND_CMD_TABLE_CNT
} NAND_CMD_TABLE_ID_T;


__weak FT_CONST SPI_FLASH_FACT_CFG_T nand_cmd_table_list[] =
{
    {
        //winb_w25n01gw_ops
        1,    /* NAND flash */
        0XEF, /* winbond manuf id */
        0xba,    /* Device ID*/
        0x21, /* device id, for winbond, they have 16 bit device, just use 8 now */
        0xC0, /* bit0 as busy, bit1 as WEL */
        0xA0, /* protect, WPE */
        0xB0, /* bit3 as buf mode, bit4 as ECC-C */
        0x30, /* bit 4 and 5 in status register C0*/
        0x00, /* not support QE configure */
        0x08, /* bit 3 in mode register B0 for continue mode*/
        0x10, /* bit 4 in mode register B0 for ecc enable */
        64,
        131072, /* 128KB */
        {
            {0x06, 0, 0, 0, 0, 0,  0, 0, 1}, /* SPI_FLASH_CMD_WREN*/
            {0x04, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRDI*/
            {0x0f, 0, 1, 0, 0, 0, 0, 1, 1}, /* SPI_FLASH_CMD_RDSR*/
            {0x1f, 1, 1, 0, 0, 0, 0, 1, 1}, /* SPI_FLASH_CMD_WRSR*/
            {0x13, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PREAD*/
            {0x03, 0, 1, 8, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_READ*/
            {0x0b, 0, 1, 8, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_FREAD*/
            {0x3b, 0, 2, 8, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_DREAD*/
            {0x6b, 0, 3, 8, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_QREAD*/
            {0xbb, 0, 2, 4, 0, 0, 1, 2, 1}, /* SPI_FLASH_CMD_2READ*/
            {0xeb, 0, 3, 4, 0, 0, 1, 3, 1}, /* SPI_FLASH_CMD_4READ*/
            {0x9f, 0, 1, 8, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDID*/
            {0x02, 1, 1, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_PLD*/
            {0x32, 1, 3, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_QPLD*/
            {0x84, 1, 1, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_PLDR*/
            {0x34, 1, 3, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_QPLDR*/
            {0x10, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PEXE*/
            {0xd8, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_BE*/
            {0xff, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RST*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RST_EN*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSR2*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WVSR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PP*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPP*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDEAR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WREAR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_SE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE32*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE64*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_CE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSR3*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WRSR3*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_EN4BM*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_ET4BM*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RD4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_FR4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_FQR4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_4RD4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PP4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPP4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_SE4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WRSR2*/
            {0xA9, 0, 1, 8, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_LEFPA*/
            {0xA1, 0, 0, 0, 0, 0, 3, 1, 1}, /* SPI_FLASH_CMD_BBM*/
            {0xA5, 0, 1, 8, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RBLUT*/
            {0x0B, 0, 3, 32, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_CFREAD*/
            {0xEB, 0, 3, 12, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_C4READ*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RUID*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSCUR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PRSCUR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_ERSCUR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_DPD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDP*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_DTR4R*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSFDP*/
        }
    },
    {
        //gd5f1gq4xc_ops
        1,    /* NAND flash */
        0XC8, /* giga device manuf id */
        0xA1,    /* Device ID*/
        0x48, /* device id, for winbond, they have 16 bit device, just use 8 now */
        0xC0, /* bit0 as busy, bit1 as WEL */
        0xA0, /* protect, WPE */
        0xB0, /* bit3 as buf mode, bit4 as ECC-C */
        0x30, /* bit 4 and 5 and 6in status register C0*/
        0x01,   /* bit 1 in mode register B0 for QE set*/
        0x00,   /* Not support continue mode, only buf mode */
        0x10,   /* bit 4 in mode register B0 for ecc enable*/
        128,
        131072, /* 128KB */
        {
            {0x06, 0, 0, 0, 0, 0,  0, 0, 1}, /* SPI_FLASH_CMD_WREN*/
            {0x04, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRDI*/
            {0x0F, 0, 1, 0, 0, 0, 0, 1, 1}, /* SPI_FLASH_CMD_RDSR*/
            {0x1F, 1, 1, 0, 0, 0, 0, 1, 1}, /* SPI_FLASH_CMD_WRSR*/
            {0x13, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PREAD*/
            {0x03, 0, 1, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_READ*/
            {0x0b, 0, 1, 8, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_FREAD*/
            {0x3b, 0, 2, 8, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_DREAD*/
            {0x6b, 0, 3, 8, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_QREAD*/
            {0xbb, 0, 2, 4, 0, 0, 1, 2, 1}, /* SPI_FLASH_CMD_2READ*/
            {0xeb, 0, 3, 2, 0, 0, 1, 3, 1}, /* SPI_FLASH_CMD_4READ*/
            {0x9f, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDID*/
            {0x02, 1, 1, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_PLD*/
            {0x32, 1, 3, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_QPLD*/
            {0x84, 1, 1, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_PLDR*/
            {0x34, 1, 3, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_QPLDR*/
            {0x10, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PEXE*/
            {0xd8, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_BE*/
            {0xff, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RST*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RST_EN*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSR2*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WVSR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PP*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPP*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDEAR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WREAR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_SE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE32*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE64*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_CE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSR3*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WRSR3*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_EN4BM*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_ET4BM*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RD4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_FR4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_FQR4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_4RD4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PP4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPP4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_SE4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WRSR2*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_LEFPA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BBM*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RBLUT*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_CFREAD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_C4READ*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RUID*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSCUR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PRSCUR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_ERSCUR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_DPD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDP*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_DTR4R*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSFDP*/
        }
    },
    {
        //ds35x1gaxxx_ops
        1,    /* NAND flash */
        0XE5, /* manuf id */
        0x21,    /* Device ID*/
        0XE5, /* device id, for winbond, they have 16 bit device, just use 8 now */
        0xC0, /* bit0 as busy, bit1 as WEL */
        0xA0, /* protect, WPE */
        0xB0, /* bit3 as buf mode, bit4 as ECC-C */
        0x30, /* bit 4 and 5 in status register C0*/
        0x01,   /* bit 1 in mode register B0  for QE set*/
        0x00,   /* Not support continue mode, only buf mode */
        0x10,   /* bit 4 in mode register B0 for ecc enable*/
        64,
        131072, /* 128KB */
        {
            {0x06, 0, 0, 0, 0, 0,  0, 0, 1}, /* SPI_FLASH_CMD_WREN*/
            {0x04, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRDI*/
            {0x0F, 0, 1, 0, 0, 0, 0, 1, 1}, /* SPI_FLASH_CMD_RDSR*/
            {0x1F, 1, 1, 0, 0, 0, 0, 1, 1}, /* SPI_FLASH_CMD_WRSR*/
            {0x13, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PREAD*/
            {0x03, 0, 1, 8, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_READ*/
            {0x0b, 0, 1, 8, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_FREAD*/
            {0x3b, 0, 2, 8, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_DREAD*/
            {0x6b, 0, 3, 8, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_QREAD*/
            {0xbb, 0, 2, 4, 0, 0, 1, 2, 1}, /* SPI_FLASH_CMD_2READ*/
            // {0xeb, 0, 3, 4, 0, 0, 1, 3}, /* SPI_FLASH_CMD_4READ*/
            {0x6b, 0, 3, 8, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_4READ*/
            {0x9f, 0, 1, 8, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDID*/
            {0x02, 1, 1, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_PLD*/
            {0x32, 1, 3, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_QPLD*/
            {0x84, 1, 1, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_PLDR*/
            {0x34, 1, 3, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_QPLDR*/
            {0x10, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PEXE*/
            {0xd8, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_BE*/
            {0xff, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RST*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RST_EN*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSR2*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WVSR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PP*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPP*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDEAR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WREAR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_SE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE32*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE64*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_CE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSR3*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WRSR3*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_EN4BM*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_ET4BM*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RD4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_FR4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_FQR4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_4RD4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PP4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPP4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_SE4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WRSR2*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_LEFPA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BBM*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RBLUT*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_CFREAD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_C4READ*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RUID*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSCUR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PRSCUR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_ERSCUR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_DPD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDP*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_DTR4R*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSFDP*/
        }
    },
    {
        //kioxia_tc58cyg0s3hraij_ops
        1,    /* NAND flash */
        0X98, /* manufacture ID : KIOXIA*/
        0xd2,   /* Device ID*/
        0X40, /* Organization ID */
        0xC0, /* status regiser ,bit0 as busy, bit1 as WEL */
        0xA0, /* protect register address, WPE */
        0xB0, /* for qe/continue/ecc-e  mode register */
        0x30, /* ecc status bits: bit 4 and 5 in status register C0*/
        0x00,   /* no need set qe setting bits*/
        0x00,   /* Not support continue mode, only buf mode */
        0x10,   /* ecc enable bits: 4 in mode register B0*/
        64,
        131072, /* 128KB */
        {
            {0x06, 0, 0, 0, 0, 0,  0, 0, 1}, /* SPI_FLASH_CMD_WREN*/
            {0x04, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRDI*/
            {0x0F, 0, 1, 0, 0, 0, 0, 1, 1}, /* SPI_FLASH_CMD_RDSR*/
            {0x1F, 1, 1, 0, 0, 0, 0, 1, 1}, /* SPI_FLASH_CMD_WRSR*/
            {0x13, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PREAD*/
            {0x03, 0, 1, 8, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_READ*/
            {0x0b, 0, 1, 8, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_FREAD*/
            {0x3b, 0, 2, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_DREAD*/
            {0x6b, 0, 3, 8, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_QREAD*/
            {0xbb, 0, 2, 4, 0, 0, 1, 2, 1}, /* SPI_FLASH_CMD_2READ*/
            // {0xeb, 0, 3, 4, 0, 0, 1, 3}, /* SPI_FLASH_CMD_4READ*/
            {0x6b, 0, 3, 8, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_4READ*/
            {0x9f, 0, 1, 8, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDID*/
            {0x02, 1, 1, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_PLD*/
            {0x32, 1, 3, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_QPLD*/
            {0x84, 1, 1, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_PLDR*/
            {0x34, 1, 3, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_QPLDR*/
            {0x10, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PEXE*/
            {0xd8, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_BE*/
            {0xff, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RST*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RST_EN*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSR2*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WVSR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PP*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPP*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDEAR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WREAR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_SE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE32*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE64*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_CE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSR3*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WRSR3*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_EN4BM*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_ET4BM*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RD4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_FR4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_FQR4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_4RD4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PP4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPP4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_SE4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WRSR2*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_LEFPA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BBM*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RBLUT*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_CFREAD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_C4READ*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RUID*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSCUR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PRSCUR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_ERSCUR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_DPD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDP*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_DTR4R*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSFDP*/
        }
    },
    {
        //FUDAN_FM25LS01_ops
        1,    /* NAND flash */
        0XA1, /* manufacture ID : FUDAN MICRO*/
        0xA5,   /* Device ID*/
        0X7F, /* Organization ID, NO USE FOR this type */
        0xC0, /* status regiser ,bit0 as busy, bit1 as WEL */
        0xA0, /* protect register address, WPE */
        0xB0, /* for qe/continue/ecc-e  mode register */
        0x30, /* ecc status bits: bit 4 and 5 in status register C0*/
        0x00,   /* no need set qe setting bits*/
        0x00,   /* Not support continue mode, only buf mode */
        0x10,   /* ecc enable bits: 4 in mode register B0*/
        64,
        131072, /* 128KB */
        {
            {0x06, 0, 0, 0, 0, 0,  0, 0, 1}, /* SPI_FLASH_CMD_WREN*/
            {0x04, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRDI*/
            {0x0F, 0, 1, 0, 0, 0, 0, 1, 1}, /* SPI_FLASH_CMD_RDSR*/
            {0x1F, 1, 1, 0, 0, 0, 0, 1, 1}, /* SPI_FLASH_CMD_WRSR*/
            {0x13, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PREAD*/
            {0x03, 0, 1, 8, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_READ*/
            {0x0b, 0, 1, 8, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_FREAD*/
            {0x3b, 0, 2, 8, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_DREAD*/
            {0x6b, 0, 3, 8, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_QREAD*/
            {0xbb, 0, 2, 4, 0, 0, 1, 2, 1}, /* SPI_FLASH_CMD_2READ*/
            {0xeb, 0, 3, 4, 0, 0, 1, 3, 1}, /* SPI_FLASH_CMD_4READ*/
            {0x9f, 0, 1, 8, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDID*/
            {0x02, 1, 1, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_PLD*/
            {0x32, 1, 3, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_QPLD*/
            {0x84, 1, 1, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_PLDR*/
            {0x34, 1, 3, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_QPLDR*/
            {0x10, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PEXE*/
            {0xd8, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_BE*/
            {0xff, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RST*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RST_EN*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSR2*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WVSR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PP*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPP*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDEAR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WREAR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_SE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE32*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE64*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_CE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSR3*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WRSR3*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_EN4BM*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_ET4BM*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RD4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_FR4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_FQR4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_4RD4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PP4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPP4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_SE4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WRSR2*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_LEFPA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BBM*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RBLUT*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_CFREAD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_C4READ*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RUID*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSCUR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PRSCUR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_ERSCUR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_DPD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDP*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_DTR4R*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSFDP*/
        }
    },
    {
        //gd5f1gm7re_ops
        1,    /* NAND flash */
        0XC8, /* winbond manuf id */
        0x81,    /* Device ID*/
        0xC8, /* device id, for winbond, they have 16 bit device, just use 8 now */
        0xC0, /* bit0 as busy, bit1 as WEL */
        0xA0, /* protect, WPE */
        0xB0, /* bit3 as buf mode, bit4 as ECC-C */
        0x30, /* bit 4 and 5 in status register C0*/
        0x01,   /* bit 1 in mode register B0 for QE set*/
        0x00,   /* Not support continue mode, only buf mode */
        0x10,   /* bit 4 in mode register B0 for ecc enable*/
        128,
        131072, /* 128KB */
        {
            {0x06, 0, 0, 0, 0, 0,  0, 0, 1}, /* SPI_FLASH_CMD_WREN*/
            {0x04, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_WRDI*/
            {0x0F, 0, 1, 0, 0, 0, 0, 1, 1}, /* SPI_FLASH_CMD_RDSR*/
            {0x1F, 1, 1, 0, 0, 0, 0, 1, 1}, /* SPI_FLASH_CMD_WRSR*/
            {0x13, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PREAD*/
            {0x03, 0, 1, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_READ*/
            {0x0b, 0, 1, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_FREAD*/
            {0x3b, 0, 2, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_DREAD*/
            {0x6b, 0, 3, 8, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_QREAD*/
            {0xbb, 0, 2, 4, 0, 0, 1, 2, 1}, /* SPI_FLASH_CMD_2READ*/
            {0xeb, 0, 3, 4, 0, 0, 1, 3, 1}, /* SPI_FLASH_CMD_4READ*/
            {0x9f, 0, 1, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RDID*/
            {0x02, 1, 1, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_PLD*/
            {0x32, 1, 3, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_QPLD*/
            {0x84, 1, 1, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_PLDR*/
            {0x34, 1, 3, 0, 0, 0, 1, 1, 1}, /* SPI_FLASH_CMD_QPLDR*/
            {0x10, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_PEXE*/
            {0xd8, 0, 0, 0, 0, 0, 2, 1, 1}, /* SPI_FLASH_CMD_BE*/
            {0xff, 0, 0, 0, 0, 0, 0, 0, 1}, /* SPI_FLASH_CMD_RST*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RST_EN*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSR2*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WVSR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PP*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPP*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDEAR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WREAR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_SE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE32*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE64*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_CE*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSR3*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WRSR3*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_EN4BM*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_ET4BM*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RD4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_FR4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_FQR4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_4RD4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PP4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_QPP4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_SE4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BE4BA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_WRSR2*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_LEFPA*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_BBM*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RBLUT*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_CFREAD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_C4READ*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RUID*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSCUR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_PRSCUR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_ERSCUR*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_DPD*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDP*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_DTR4R*/
            {0, 0, 0, 0, 0, 0, 0, 0, 0}, /* SPI_FLASH_CMD_RDSFDP*/
        }
    },
};

FT_CONST FLASH_RDID_TYPE_T nand_cmd_id_pool_type0[] =
{
    {0xef, 0xba, 0x21, 0, 0x8000000},    //W25N01GW_RDID
    {0xef, 0xaa, 0x21, 0, 0x8000000},    //W25N01GV_RDID
    {0xef, 0xba, 0x23, 0, 0x20000000},   //W25N04KW_RDID
    {FLASH_INVALID_ID, 0, 0, 0, 0},      //last one
};

FT_CONST FLASH_RDID_TYPE_T nand_cmd_id_pool_type1[] =
{
    {0xC9, 0x81, 0x00, 0x10, 0x8000000}, //HYF1GQ4IDACAE_RDID
    {0x81, 0xc9, 0x81, 0x10, 0x8000000}, //HYF1GQ4IDACAE_RDID
    {0x01, 0x81, 0xc9, 0x10, 0x8000000}, //HYF1GQ4IDACAE_RDID
    {0x01, 0xc9, 0x81, 0x10, 0x8000000}, //HYF1GQ4IDACAE_RDID
    {0x5e, 0x44, 0x5e, 0, 0x20000000},   //ZB35Q04A_RDID, used as 2KB page
    {0x0b, 0x31, 0x00, 0x40, 0x8000000}, //XT26G01DXXX_RDID
    {0x8c, 0x01, 0x8c, 0x10, 0x8000000}, //XCSP1AAPK-IT_RDID
    {0x8c, 0xb1, 0x8c, 0x18, 0x20000000}, //XCSP4AAPK-IT_RDID
    {0x0b, 0x33, 0X00, 0x44, 0x20000000},//XT26G04DXXX_RDID
    {0xc8, 0xd9, 0xc8, 0x10, 0x8000000}, //GD5F1GQ4UxxH_RDID
    {0xc8, 0xc9, 0xc8, 0x10, 0x8000000}, //GD5F1GQ4RxxH_RDID
    {FLASH_INVALID_ID, 0, 0, 0, 0},      //last one
};

FT_CONST FLASH_RDID_TYPE_T nand_cmd_id_pool_type2[] =
{
    {0xE5, 0x21, 0XE5, 0, 0x8000000},       //DS35X1GAXXX_RDID
    {0xE5, 0xA5, 0XE5, 0x20, 0x4000000},    //DS35M12BXXX_RDID
    {0xE5, 0xF5, 0XE5, 0x20, 0x4000000},    //DS35Q12BXXX_RDID
    {0xE5, 0xF1, 0XE5, 0x20, 0x8000000},    //DS35Q1GBXXX_RDID
    {0xE5, 0xA1, 0XE5, 0x20, 0x8000000},    //DS35M1GBXXX_RDID
    {0xE5, 0xA2, 0XE5, 0x22, 0x10000000},   //DS35M2GBXXX_RDID
    {0xE5, 0xF4, 0XE5, 0x22, 0x20000000},   //DS35X4GMXXX_RDID rev01
    {0xE5, 0x74, 0xE5, 0x22, 0x20000000},   //DS35X4GMXXX_RDID diff version?
    {0xE5, 0xB4, 0XE5, 0x20, 0x20000000},   //DS35Q4GBXXX_RDID
    {0xE5, 0x75, 0XE5, 0x20, 0x4000000},   //DS35Q12CXXX_RDID
    {0xCD, 0x70, 0X70, 0, 0x4000000},       //F35SQA512MX_RDID
    {0xCD, 0x60, 0X60, 0, 0x4000000},       //F35UQA512MXXX_RDID
    {0xCD, 0x61, 0x61, 0, 0x8000000},      //F35UQA001GXXX_RDID
    {0xCD, 0x71, 0X71, 0, 0x8000000},       //F35SQA0001GXXX_RDID
    {0xCD, 0x72, 0X72, 0, 0x10000000},      //F35SQA0002GXXX_RDID
    {0xCD, 0x53, 0X53, 0x34, 0x20000000},    //F35SQB004G_RDID
    {0x0B, 0x51, 0X00, 0x40, 0x8000000},    //XT26Q01DXXX_RDID
    {0x0B, 0x11, 0X00, 0x50, 0x8000000},    //XT26G01CXXX_RDID
    {0x3C, 0xD1, 0xD1, 0, 0x8000000},       //HSESYHDSW1G_RDID
    {0x3C, 0xD2, 0xD2, 0, 0x10000000},      //HSESYHDSW2G_RDID
    {0xa1, 0xd4, 0xa1, 0x20, 0x8000000},    //FM25S01BI3_RDID
    {0xc9, 0xd4, 0xc9, 0x14, 0x20000000},   //HYF4GQ4UAACBE_RDID
    {FLASH_INVALID_ID, 0, 0, 0, 0},         //last one
};

FT_CONST FLASH_RDID_TYPE_T nand_cmd_id_pool_type3[] =
{
    {0x98, 0xd2, 0X40, 0x10, 0x8000000},    //TC58CYG0S3HRAIJ_RDID
    {0x3C, 0xD1, 0x3C, 0, 0x8000000},       //HSESDDDSW1G_RDID
    {0x01, 0x15, 0X01, 0x60, 0x8000000},    //HYF1GQ4UTXCAE_RDID
    {0x01, 0x35, 0X01, 0x60, 0x20000000},   //HYF4GQ4UTACAE_RDID
    {FLASH_INVALID_ID, 0, 0, 0, 0},         //last one
};

FT_CONST FLASH_RDID_TYPE_T nand_cmd_id_pool_type4[] =
{
    {0xa1, 0xa5, 0X7f, 0, 0x8000000},       //FM25LS01_RDID
    {0xc9, 0x21, 0xc9, 0x10, 0x8000000},    //HYF1GQ4UDA_RDID
    {0x5e, 0xa1, 0xa1, 0, 0x8000000},       //ZB35Q01B_RDID
    {0x5e, 0xa2, 0xa1, 0, 0x10000000},      //ZB35Q02BYIG_RDID
    {FLASH_INVALID_ID, 0, 0, 0, 0},         //last one
};

FT_CONST FLASH_RDID_TYPE_T nand_cmd_id_pool_type5[] =
{
    {0xc8, 0x81, 0xc8, 0x10, 0x8000000},    //GD5F1GM7RE_RDID
    {0xc8, 0x91, 0xc8, 0x10, 0x8000000},    //GD5F1GM7UE_RDID
    {0xc8, 0x92, 0xc8, 0x10, 0x10000000},   //GD5F2GM7UE_RDID
    {0xc8, 0x82, 0xc8, 0x10, 0x10000000},   //GD5F2GM7RE_RDID
    {0xc8, 0x95, 0xc8, 0x10, 0x20000000},   //GD5F4GM8UE_RDID
    {0xc8, 0x85, 0xc8, 0x10, 0x20000000},   //GD5F4GM8RE_RDID
    {0x52, 0xCA, 0x13, 0x10, 0x8000000},    //GSS01GSAX1_RDID
    {FLASH_INVALID_ID, 0, 0, 0, 0},         //last one
};


#if defined(JLINK) || defined(KEIL)
/* For Jlink device, as it does not have scatter loading, need assign this table in Jlink init code.*/
FLASH_RDID_TYPE_T    *nand_cmd_id_pool[NAND_CMD_TABLE_CNT];
#else
/* For bootloader, need compress to reduce code size */
__weak FT_CONST FLASH_RDID_TYPE_T   *FT_CONST nand_cmd_id_pool[] =
{
    &nand_cmd_id_pool_type0[0],      // type 0, NON-BUF,NO QE, EB with 4 dummy
    &nand_cmd_id_pool_type1[0],      // type 1, BUF,    QE,    EB, EB with 2 dummy
    &nand_cmd_id_pool_type2[0],      // type 2, BUF,    QE,    NO EB
    &nand_cmd_id_pool_type3[0],      // type 3, BUF,    NO QE, NO EB
    &nand_cmd_id_pool_type4[0],      // type 4, BUF,    NO QE, EB   with 4 dummy
    &nand_cmd_id_pool_type5[0],      // type 5, BUF,    QE,    EB, EB with 4 dummy
};
#endif /* JLINK || KEIL */

#if defined(CFG_FACTORY_DEBUG)
__WEAK void *get_user_flash_cfg(uint8_t isnand, uint8_t fid, uint8_t did, uint8_t type, uint8_t *flash_type)
{
    return NULL;
}
#endif


#if defined(JLINK) || defined(KEIL)
__weak void spi_nand_table_init(void)
{
    if (NAND_CMD_TABLE_CNT != 6)
    {
        while (1);
    }

    nand_cmd_id_pool[0] = &nand_cmd_id_pool_type0[0];
    nand_cmd_id_pool[1] = &nand_cmd_id_pool_type1[0];
    nand_cmd_id_pool[2] = &nand_cmd_id_pool_type2[0];
    nand_cmd_id_pool[3] = &nand_cmd_id_pool_type3[0];
    nand_cmd_id_pool[4] = &nand_cmd_id_pool_type4[0];
    nand_cmd_id_pool[5] = &nand_cmd_id_pool_type5[0];
}
#endif /* JLINK || KEIL */


FT_CONST FLASH_RDID_TYPE_T *spi_nand_get_rdid(uint8_t fid, uint8_t did, uint8_t type, uint8_t *flash_type)
{
    int i;
    FT_CONST FLASH_RDID_TYPE_T *res;

    // check flash id valid
    if ((fid == FLASH_INVALID_ID) || (fid == FLASH_UNKNOW_ID))
        return NULL;

    for (i = 0; i < NAND_CMD_TABLE_CNT; i++)
    {
        res = nand_cmd_id_pool[i];
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

    if (i == NAND_CMD_TABLE_CNT)
    {
#if defined(CFG_FACTORY_DEBUG)
        res = (FLASH_RDID_TYPE_T *)get_user_flash_cfg(1, fid, did, type, flash_type);
#else
        res = NULL;
#endif
    }
    else if (flash_type)
    {
        *flash_type = i;
    }
    return res;
}

// get command table by index from register table
const SPI_FLASH_FACT_CFG_T *spi_nand_get_cmd_by_id(uint8_t fid, uint8_t did, uint8_t type)
{
    uint8_t i;
    const SPI_FLASH_FACT_CFG_T *res = NULL;

    FT_CONST FLASH_RDID_TYPE_T *rdid = spi_nand_get_rdid(fid, did, type, &i);
    if (rdid)
        res = (const SPI_FLASH_FACT_CFG_T *)&nand_cmd_table_list[i];

    return res;
}

__weak int HAL_GET_FLASH_DEFAUT_INX(void)
{
    return -1;
}

// get default command table if default index valid
const SPI_FLASH_FACT_CFG_T *spi_nand_get_default_ctable(void)
{
    int deft;
    const SPI_FLASH_FACT_CFG_T *res = NULL;

    deft = HAL_GET_FLASH_DEFAUT_INX();
    if (deft >= 0)
    {
        res = (const SPI_FLASH_FACT_CFG_T *)&nand_cmd_table_list[deft];
    }

    return res;
}

// get nand size by id from register table
int spi_nand_get_size_by_id(uint8_t fid, uint8_t did, uint8_t type)
{
    int res = 0x4000000;

    FT_CONST FLASH_RDID_TYPE_T *rdid = spi_nand_get_rdid(fid, did, type, NULL);
    if (rdid)
        res = rdid->mem_size;

    return res;
}

// get plane slect flag by id from register table
int spi_nand_get_plane_select_flag(uint8_t fid, uint8_t did, uint8_t type)
{
    int res = 0;

    FT_CONST FLASH_RDID_TYPE_T *rdid = spi_nand_get_rdid(fid, did, type, NULL);
    if (rdid && (rdid->ext_flags & PLANE_SELEC_FLAG) != 0)
        res = 1;

    return res;
}

int spi_nand_get_big_page_flag(uint8_t fid, uint8_t did, uint8_t type)
{
    int res = 0;

    FT_CONST FLASH_RDID_TYPE_T *rdid = spi_nand_get_rdid(fid, did, type, NULL);
    if (rdid)
        res = (rdid->ext_flags & (BIG_PAGE_FLAG | BIG_BLK_FLAG)) >> 2;
    return res;
}

int spi_nand_get_ecc_mode(uint8_t fid, uint8_t did, uint8_t type)
{
    int res = 0;

    FT_CONST FLASH_RDID_TYPE_T *rdid = spi_nand_get_rdid(fid, did, type, NULL);
    if (rdid)
        res = (rdid->ext_flags & NAND_ECC_FULL_RESERVED) >> NAND_ECC_START_POS;
    return res;
}
