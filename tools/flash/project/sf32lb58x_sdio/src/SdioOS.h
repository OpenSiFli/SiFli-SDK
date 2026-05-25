/*********************************************************************
*            (c) 1995 - 2018 SEGGER Microcontroller GmbH             *
*                        The Embedded Experts                        *
*                           www.segger.com                           *
**********************************************************************
*/
#include "stdint.h"
#define U8  uint8_t
#define U16 uint16_t
#define U32 uint32_t

#define I8  int8_t
#define I16 int16_t
#define I32 int32_t

#define ONCHIP     (1)             // On-chip Flash Memory

#define MAX_NUM_SECTORS (512)      // Max. number of sectors, must not be modified.
#define ALGO_VERSION    (0x0101)   // Algo version, must not be modified.

#ifdef JLINK_SDIO_1
#ifdef SD1_PINMUX_TYPE1
    #define SDIO_NAME  "SF32LB58X External SDIO1 TYPE1"
#else
    #define SDIO_NAME  "SF32LB58X External SDIO1"
#endif /* SD1_PINMUX_TYPE1 */
    #define SDIO_BASE  0x68000000
    #define SDIO_SIZE  0x38000000
#elif defined(JLINK_SDIO_2)
    #define SDIO_NAME  "SF32LB58X External SDIO2"
    #define SDIO_BASE  0xa0000000
    #define SDIO_SIZE  0x40000000
#endif

struct SECTOR_INFO
{
    U32 SectorSize;       // Sector Size in bytes
    U32 SectorStartAddr;  // Start address of the sector area (relative to the "BaseAddr" of the flash)
};

struct FlashDevice
{
    U16 AlgoVer;       // Algo version number
    U8  Name[128];     // Flash device name
    U16 Type;          // Flash device type
    U32 BaseAddr;      // Flash base address
    U32 TotalSize;     // Total flash device size in Bytes (256 KB)
    U32 PageSize;      // Page Size (number of bytes that will be passed to ProgramPage(). MinAlig is 8 byte
    U32 Reserved;      // Reserved, should be 0
    U8  ErasedVal;     // Flash erased value
    U32 TimeoutProg;   // Program page timeout in ms
    U32 TimeoutErase;  // Erase sector timeout in ms
    struct SECTOR_INFO SectorInfo[MAX_NUM_SECTORS]; // Flash sector layout definition
};

//
// Flash module functions
//
extern int Init(U32 Addr, U32 Freq, U32 Func);                  // Mandatory
extern int UnInit(U32 Func);                                    // Mandatory
extern int EraseSector(U32 Addr);                               // Mandatory
extern int ProgramPage(U32 Addr, U32 NumBytes, U8 *pSrcBuff);   // Mandatory
extern int BlankCheck(U32 Addr, U32 NumBytes, U8 BlankData);    // Optional
extern int EraseChip(void);                                     // Optional
extern U32 Verify(U32 Addr, U32 NumBytes, U8 *pSrcBuff);        // Optional
extern void HAL_MspInit(void);

//
// SEGGER defined functions
//
extern U32  SEGGER_OPEN_CalcCRC(U32 crcs, U32 Addr, U32 NumBytes, U32 Polynom);      // Optional
extern int  SEGGER_OPEN_Read(U32 Addr, U32 NumBytes, U8 *pDestBuff);                // Optional
extern int  SEGGER_OPEN_Program(U32 DestAddr, U32 NumBytes, U8 *pSrcBuff);          // Optional
extern int  SEGGER_OPEN_Erase(U32 SectorAddr, U32 SectorIndex, U32 NumSectors);     // Optional
//extern void SEGGER_OPEN_Start    (volatile struct SEGGER_OPEN_CMD_INFO* pInfo);     // Optional


#define PAGE_SIZE 0x4000








