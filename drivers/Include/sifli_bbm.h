/*
*/

#ifndef __SIF_NAND_BBM_H__
#define __SIF_NAND_BBM_H__

//#include "board.h"
#include "bf0_hal_def.h"

//#define BBM_TABLE_AUTO_TEST

#ifdef BBM_UT
    #include <stdint.h>
#endif

#define RET_ECC_WORK                (1)
#define RET_NOERROR                  (0)
#define RET_ERROR                       (-1)
#define RET_ECC_FAIL                    (-2)
#define RET_P_FAIL                        (-3)
#define RET_E_FAIL                        (-4)


#define BBM_NAND_BLOCK_SIZE             (0x20000)
#define BBM_NAND_PAGE_SIZE                  (0x800)

#define BBM_NAND_E_FAIL                     (1<<2)
#define BBM_NAND_P_FAIL                     (1<<3)


#define BBM_INVALID_BLK             (0XFFFF)

//#define BBM_RESERVED_START      (BBM_USER_BLK)

#define BBM_MAGIC_NUM               (0x5366424d)

typedef struct
{
    uint16_t logic_blk;     // logic block number
    uint16_t physical_blk;  // physical block number
} Sifli_MapTbl;

typedef struct
{
    uint32_t magic;// magic number to decalare bbm talbe, 0x5366424d
    uint32_t version: 31;
    uint32_t tbl_idx: 1; //0:TBL1, 1:TBL2
    uint16_t bbk_num; // bad block number
    uint16_t free_blk_num;// free block number
    uint16_t free_blk_start; // free block start addr, it will be used for next map
    uint16_t reserv_blk_start; // start block number for reverved spare
    uint32_t hdr_crc; // crc for this table before this member
    uint32_t tbl_crc; // map table crc
    Sifli_MapTbl stru_tbl[128 - 4]; // max support 128 block backup, to meet 4Gb nand request
} Sifli_NandBBM; //Nand Flash Bad block management

typedef void (*bbm_log_func)(const char *fmt, ...);


/**
 * @brief Bad block manager initial, will parse all nand blocks and initialize bbm table.
 * @param[in] total: total nand size in bytes, decide how many backup blocks can be used.
 * @param[out] cache: bbm buffer, at least on page + oob size.
 * @return 0 if succes.
 */
int sif_bbm_init(uint32_t total, uint8_t *cache);

int bbm_read_page(int blk, int page, int offset, uint8_t *buff, uint32_t size, uint8_t *spare, uint32_t spare_len);

int bbm_write_page(int blk, int page, uint8_t *data, uint8_t *spare, uint32_t spare_len);

int bbm_erase_block(int blk);

int port_read_page(int blk, int page, int offset, uint8_t *buff, uint32_t size, uint8_t *spare, uint32_t spare_len);

int port_write_page(int blk, int page, uint8_t *data, uint8_t *spare, uint32_t spare_len);

int port_erase_block(int blk);

int bbm_mark_bb(int blk);

int bbm_get_bb(int blk);

void bbm_set_page_size(uint32_t page_size);

void bbm_set_blk_size(uint32_t blk_size);

int bbm_get_total(void);

void *bbm_get_context(uint8_t idx);


void bbm_register_log(bbm_log_func log_func);

#ifdef BBM_TABLE_AUTO_TEST

typedef enum
{
    ERR_TYPE_NON = 0,
    ERR_P_FAIL,
    ERR_ECC_FAIL,
    ERR_E_FAIL,
    ERR_FACT_BAD,
    ERR_CNT
} BBM_Sim_Err_T;
typedef struct
{
    uint8_t err_type;   // BBM_Sim_Err_T
    uint8_t page;
    uint16_t block;
} Sifli_BBM_Test_Table_T;

#endif  //BBM_TABLE_AUTO_TEST

#endif //__SIF_NAND_BBM_H__

