/*
*/
#include "sifli_bbm.h"
#include "string.h"

typedef struct
{
    uint16_t tblk1[2];  // 2 block to save table 1
    uint16_t tblk2[2];  // 2 block to save table 2
    uint16_t idle_blk[4];   // found 4 idle block to save map table if table1/table2 not save on 2 block
    uint16_t cur_blk[2];    // which block with large version for 2 table
    uint32_t boot_flag; // 0 for first update after reboot, 1 for normal mode
} NAND_BBM_CTX_T;

static NAND_BBM_CTX_T bbm_ctx;
static Sifli_NandBBM bbm_local[2];
static uint32_t user_blk;
static uint32_t bkup_blk;
static uint32_t total_blk;
static uint32_t bbm_blk_size = 0x20000;
static uint32_t bbm_page_size = 2048;
static int gbbm_init_flag = 0;

uint8_t *bbm_page_cache;

static bbm_log_func g_bbm_dlog = NULL;


#define BBM_ASSERT(a)      \
if(!(a))   \
{ \
    HAL_ASSERT(a);               \
}
#define BBM_MAX(a, b)       ((a)>(b)? (a):(b))

#define BBM_DBG_LV_ERR           (1)
#define BBM_DBG_LV_WARN          (2)
#define BBM_DBG_LV_INFO          (3)
#define BBM_DBG_LV_DBG           (4)

#define BBM_DBG_LV      BBM_DBG_LV_INFO
#define BBM_ERR(fmt, ...)         \
if((g_bbm_dlog != NULL) && (BBM_DBG_LV >= BBM_DBG_LV_ERR))   \
{ \
    g_bbm_dlog(fmt, ##__VA_ARGS__);               \
}

#define BBM_WARN(fmt, ...)        \
if((g_bbm_dlog != NULL) && (BBM_DBG_LV >= BBM_DBG_LV_WARN))   \
{ \
    g_bbm_dlog(fmt, ##__VA_ARGS__);               \
}

#define BBM_INFO(fmt, ...)        \
if((g_bbm_dlog != NULL) && (BBM_DBG_LV >= BBM_DBG_LV_INFO))   \
{ \
    g_bbm_dlog(fmt, ##__VA_ARGS__);               \
}

#define BBM_DBG(fmt, ...)        \
if((g_bbm_dlog != NULL) && (BBM_DBG_LV >= BBM_DBG_LV_DBG))   \
{ \
    g_bbm_dlog(fmt, ##__VA_ARGS__);               \
}



#ifdef BBM_TABLE_AUTO_TEST
    #include <stdlib.h>

    // 1024 * 2%
    #define BBM_TEST_MAX_CNT        (20)
    Sifli_BBM_Test_Table_T g_bbm_test_table[BBM_TEST_MAX_CNT];
    static int bbm_init_test_table();
    static int bbm_get_test_format(uint16_t blk, uint8_t page);

#endif //BBM_TABLE_AUTO_TEST

static int bbm_write_talbe(int tid, uint16_t blk);
static uint16_t bbm_replace_blk(uint8_t idx);
static int bbm_get_page_num(uint32_t blk);


static const unsigned int crc32tab[] =
{
    0x00000000L, 0x77073096L, 0xee0e612cL, 0x990951baL,
    0x076dc419L, 0x706af48fL, 0xe963a535L, 0x9e6495a3L,
    0x0edb8832L, 0x79dcb8a4L, 0xe0d5e91eL, 0x97d2d988L,
    0x09b64c2bL, 0x7eb17cbdL, 0xe7b82d07L, 0x90bf1d91L,
    0x1db71064L, 0x6ab020f2L, 0xf3b97148L, 0x84be41deL,
    0x1adad47dL, 0x6ddde4ebL, 0xf4d4b551L, 0x83d385c7L,
    0x136c9856L, 0x646ba8c0L, 0xfd62f97aL, 0x8a65c9ecL,
    0x14015c4fL, 0x63066cd9L, 0xfa0f3d63L, 0x8d080df5L,
    0x3b6e20c8L, 0x4c69105eL, 0xd56041e4L, 0xa2677172L,
    0x3c03e4d1L, 0x4b04d447L, 0xd20d85fdL, 0xa50ab56bL,
    0x35b5a8faL, 0x42b2986cL, 0xdbbbc9d6L, 0xacbcf940L,
    0x32d86ce3L, 0x45df5c75L, 0xdcd60dcfL, 0xabd13d59L,
    0x26d930acL, 0x51de003aL, 0xc8d75180L, 0xbfd06116L,
    0x21b4f4b5L, 0x56b3c423L, 0xcfba9599L, 0xb8bda50fL,
    0x2802b89eL, 0x5f058808L, 0xc60cd9b2L, 0xb10be924L,
    0x2f6f7c87L, 0x58684c11L, 0xc1611dabL, 0xb6662d3dL,
    0x76dc4190L, 0x01db7106L, 0x98d220bcL, 0xefd5102aL,
    0x71b18589L, 0x06b6b51fL, 0x9fbfe4a5L, 0xe8b8d433L,
    0x7807c9a2L, 0x0f00f934L, 0x9609a88eL, 0xe10e9818L,
    0x7f6a0dbbL, 0x086d3d2dL, 0x91646c97L, 0xe6635c01L,
    0x6b6b51f4L, 0x1c6c6162L, 0x856530d8L, 0xf262004eL,
    0x6c0695edL, 0x1b01a57bL, 0x8208f4c1L, 0xf50fc457L,
    0x65b0d9c6L, 0x12b7e950L, 0x8bbeb8eaL, 0xfcb9887cL,
    0x62dd1ddfL, 0x15da2d49L, 0x8cd37cf3L, 0xfbd44c65L,
    0x4db26158L, 0x3ab551ceL, 0xa3bc0074L, 0xd4bb30e2L,
    0x4adfa541L, 0x3dd895d7L, 0xa4d1c46dL, 0xd3d6f4fbL,
    0x4369e96aL, 0x346ed9fcL, 0xad678846L, 0xda60b8d0L,
    0x44042d73L, 0x33031de5L, 0xaa0a4c5fL, 0xdd0d7cc9L,
    0x5005713cL, 0x270241aaL, 0xbe0b1010L, 0xc90c2086L,
    0x5768b525L, 0x206f85b3L, 0xb966d409L, 0xce61e49fL,
    0x5edef90eL, 0x29d9c998L, 0xb0d09822L, 0xc7d7a8b4L,
    0x59b33d17L, 0x2eb40d81L, 0xb7bd5c3bL, 0xc0ba6cadL,
    0xedb88320L, 0x9abfb3b6L, 0x03b6e20cL, 0x74b1d29aL,
    0xead54739L, 0x9dd277afL, 0x04db2615L, 0x73dc1683L,
    0xe3630b12L, 0x94643b84L, 0x0d6d6a3eL, 0x7a6a5aa8L,
    0xe40ecf0bL, 0x9309ff9dL, 0x0a00ae27L, 0x7d079eb1L,
    0xf00f9344L, 0x8708a3d2L, 0x1e01f268L, 0x6906c2feL,
    0xf762575dL, 0x806567cbL, 0x196c3671L, 0x6e6b06e7L,
    0xfed41b76L, 0x89d32be0L, 0x10da7a5aL, 0x67dd4accL,
    0xf9b9df6fL, 0x8ebeeff9L, 0x17b7be43L, 0x60b08ed5L,
    0xd6d6a3e8L, 0xa1d1937eL, 0x38d8c2c4L, 0x4fdff252L,
    0xd1bb67f1L, 0xa6bc5767L, 0x3fb506ddL, 0x48b2364bL,
    0xd80d2bdaL, 0xaf0a1b4cL, 0x36034af6L, 0x41047a60L,
    0xdf60efc3L, 0xa867df55L, 0x316e8eefL, 0x4669be79L,
    0xcb61b38cL, 0xbc66831aL, 0x256fd2a0L, 0x5268e236L,
    0xcc0c7795L, 0xbb0b4703L, 0x220216b9L, 0x5505262fL,
    0xc5ba3bbeL, 0xb2bd0b28L, 0x2bb45a92L, 0x5cb36a04L,
    0xc2d7ffa7L, 0xb5d0cf31L, 0x2cd99e8bL, 0x5bdeae1dL,
    0x9b64c2b0L, 0xec63f226L, 0x756aa39cL, 0x026d930aL,
    0x9c0906a9L, 0xeb0e363fL, 0x72076785L, 0x05005713L,
    0x95bf4a82L, 0xe2b87a14L, 0x7bb12baeL, 0x0cb61b38L,
    0x92d28e9bL, 0xe5d5be0dL, 0x7cdcefb7L, 0x0bdbdf21L,
    0x86d3d2d4L, 0xf1d4e242L, 0x68ddb3f8L, 0x1fda836eL,
    0x81be16cdL, 0xf6b9265bL, 0x6fb077e1L, 0x18b74777L,
    0x88085ae6L, 0xff0f6a70L, 0x66063bcaL, 0x11010b5cL,
    0x8f659effL, 0xf862ae69L, 0x616bffd3L, 0x166ccf45L,
    0xa00ae278L, 0xd70dd2eeL, 0x4e048354L, 0x3903b3c2L,
    0xa7672661L, 0xd06016f7L, 0x4969474dL, 0x3e6e77dbL,
    0xaed16a4aL, 0xd9d65adcL, 0x40df0b66L, 0x37d83bf0L,
    0xa9bcae53L, 0xdebb9ec5L, 0x47b2cf7fL, 0x30b5ffe9L,
    0xbdbdf21cL, 0xcabac28aL, 0x53b39330L, 0x24b4a3a6L,
    0xbad03605L, 0xcdd70693L, 0x54de5729L, 0x23d967bfL,
    0xb3667a2eL, 0xc4614ab8L, 0x5d681b02L, 0x2a6f2b94L,
    0xb40bbe37L, 0xc30c8ea1L, 0x5a05df1bL, 0x2d02ef8dL
};

uint32_t bbm_crc_check(const uint8_t *buf, uint32_t size)
{
    unsigned int i, crc;
    crc = 0xFFFFFFFF;

    for (i = 0; i < size; i++)
        crc = crc32tab[(crc ^ buf[i]) & 0xff] ^ (crc >> 8);

    return crc ^ 0xFFFFFFFF;
}

static int bbm_map_check(Sifli_NandBBM *bbm_table)
{
    int i, j;

    if (bbm_table && bbm_table->magic == BBM_MAGIC_NUM)
    {
        for (i = 0; i < (int)(bkup_blk - 4); i++)
        {
            if ((bbm_table->stru_tbl[i].logic_blk != 0) && (bbm_table->stru_tbl[i].physical_blk != 0))
            {
                if ((bbm_table->stru_tbl[i].logic_blk >= user_blk) || (bbm_table->stru_tbl[i].physical_blk < user_blk))
                {
                    // logic blk or phy block over range
                    BBM_ERR("Map error: logic %d, phy %d\n", bbm_table->stru_tbl[i].logic_blk, bbm_table->stru_tbl[i].physical_blk);
                    BBM_ASSERT(0);
                }
                for (j = i + 1; j < (int)(bkup_blk - 4); j++)
                {
                    if (bbm_table->stru_tbl[j].physical_blk != 0)
                    {
                        if (bbm_table->stru_tbl[i].physical_blk == bbm_table->stru_tbl[j].physical_blk)
                        {
                            // something wrong, 2 blk map to same physical block
                            BBM_ERR("ERR: 2 logic blocks map to same blk: logic0 %d, phy0 %d, logic1 %d, phy1 %d\n",
                                    bbm_table->stru_tbl[i].logic_blk, bbm_table->stru_tbl[i].physical_blk,
                                    bbm_table->stru_tbl[j].logic_blk, bbm_table->stru_tbl[j].physical_blk);
                            BBM_ASSERT(0);
                        }
                    }
                    else    // valid table end
                        break;
                }
            }
            else if ((bbm_table->stru_tbl[i].logic_blk == 0) && (bbm_table->stru_tbl[i].physical_blk == 0))
            {
                // empty table, to end
                break;
            }
            else
            {
                // one of blk is 0, not reasonable !
                BBM_ERR("Map error0: logic %d, phy %d\n", bbm_table->stru_tbl[i].logic_blk, bbm_table->stru_tbl[i].physical_blk);
                BBM_ASSERT(0);
            }
        }
    }
    else // table error?
        return 1;

    return 0;
}

static int bbm_map_new_blk(uint16_t bblk)
{
    int bad, cnt, i;
    int nblk1, nblk2;

    // found an idle node or the block using node
    cnt = 0;
    for (i = 0; i < (int)(bkup_blk - 4); i++)
    {
        if (bbm_local[0].stru_tbl[i].logic_blk == bblk)
        {
            // old map block is bad now mark it
            bbm_mark_bb(bbm_local[0].stru_tbl[i].physical_blk);
            BBM_ERR("Old map logic blk %d to phy %d fail, mark phy as bad\n", bblk, bbm_local[0].stru_tbl[i].physical_blk);
            cnt = i;
            break;
        }
        else if ((bbm_local[0].stru_tbl[i].logic_blk == 0) && (bbm_local[0].stru_tbl[i].physical_blk == 0)) // not used map table
        {
            cnt = i;
            break;
        }
    }
    if (i >= (int)(bkup_blk - 4)) // stru_tbl full ?
    {
        BBM_ERR("Map table full, can not map new bad block!\n");
        BBM_ASSERT(0);
        return -1;
    }

    if ((bbm_local[0].free_blk_start < user_blk + 4) || (bbm_local[0].free_blk_num == 0))
    {
        BBM_ERR("Map table full, free_blk_start %d, free_blk_num %d\n", bbm_local[0].free_blk_start, bbm_local[0].free_blk_num);
        BBM_ASSERT(0);
    }
    // find a good block to do map
    do
    {
        bad = bbm_get_bb(bbm_local[0].free_blk_start);
#ifdef  BBM_TABLE_AUTO_TEST
        int fmt = bbm_get_test_format(bbm_local[0].free_blk_start, 0);
        if ((fmt == ERR_FACT_BAD) || (fmt == ERR_E_FAIL))
        {
            bad = 1;
        }
#endif
        bbm_local[0].free_blk_start--;
        bbm_local[0].free_blk_num--;
    }
    while ((bad != 0) && (bbm_local[0].free_blk_num > 0));

    if (bad == 0)   // found idle block to do map
    {
        bbm_local[0].stru_tbl[cnt].logic_blk = bblk;
        bbm_local[0].stru_tbl[cnt].physical_blk = bbm_local[0].free_blk_start + 1;
        bbm_local[0].bbk_num++;
        bbm_local[0].version++;
    }
    else // not idle block, full chip can not used any more
    {
        BBM_ERR("No idle block any more!!\n");
        BBM_ASSERT(0);
    }

    bbm_local[0].hdr_crc = bbm_crc_check((const uint8_t *)(&bbm_local[0]), 16);
    bbm_local[0].tbl_crc = bbm_crc_check((const uint8_t *)(&(bbm_local[0].stru_tbl)), sizeof(Sifli_MapTbl) * (bkup_blk - 4));
    memcpy((void *)&bbm_local[1], (void *)&bbm_local[0], sizeof(Sifli_NandBBM));

    bbm_local[1].tbl_idx = 1;
    bbm_local[1].hdr_crc = bbm_crc_check((const uint8_t *)(&bbm_local[1]), 16);

    // write to flash
    if (bbm_ctx.boot_flag == 0)   // reboot
    {
        // check table 1
        nblk1 = bbm_replace_blk(0);
        if (nblk1 <= 0)
        {
            BBM_ERR("can not find 2nd block for tabl 1\n");
            BBM_ASSERT(0);
        }

        // check  table 2
        nblk2 = bbm_replace_blk(1);
        if (nblk2 <= 0)
        {
            BBM_ERR("can not find 2nd block for tabl 2\n");
            BBM_ASSERT(0);
        }

        bbm_ctx.boot_flag++;
        port_erase_block(nblk1);
        port_erase_block(nblk2);
    }
    else
    {
        int page = 0;
        nblk1 = bbm_ctx.cur_blk[0];
        page = bbm_get_page_num(nblk1);
        if (page < 0)   // current block full
            nblk1 = bbm_replace_blk(0);

        nblk2 = bbm_ctx.cur_blk[1];
        page = bbm_get_page_num(nblk2);
        if (page < 0)  // current block full
            nblk2 = bbm_replace_blk(1);

        if ((nblk1 <= 0) || (nblk2 <= 0))
        {
            BBM_ERR("Invalid bbm block %d, %d\n", nblk1, nblk2);
            BBM_ASSERT(0);
        }
    }

    bbm_write_talbe(0, nblk1);
    bbm_write_talbe(1, nblk2);
    bbm_ctx.cur_blk[0] = nblk1;
    bbm_ctx.cur_blk[1] = nblk2;

    return (int)(bbm_local[0].stru_tbl[cnt].physical_blk); // 0;
}

static int bbm_get_page_num(uint32_t blk)
{
    int res, status;
    uint32_t page;

    status = 0;
    page = 0;
    BBM_DBG("bbm_get_page_num %d\n", blk);
    while (page < bbm_blk_size / bbm_page_size)
    {
        res = port_read_page(blk, page, 0, bbm_page_cache, bbm_page_size, NULL, 0);
        if (res)
        {
            Sifli_NandBBM *bbmt = (Sifli_NandBBM *)bbm_page_cache;
            if (bbmt->magic == BBM_MAGIC_NUM)   // table block
            {
                page++;
                continue;
            }
            else if (bbmt->magic == 0xffffffff)
            {
                status = 1;
                break;
            }
        }
        page++;
    }
    BBM_DBG("Blk %d cur page %d, status %d\n", blk, page, status);

    if (status)
        return page;

    return -1;
}

static uint16_t bbm_replace_blk(uint8_t idx)
{
    uint16_t blk = bbm_ctx.cur_blk[idx];
    uint16_t nblk1 = 0;
    uint16_t *obptr;    // old block ptr
    uint16_t *nbptr;    // new block ptr
    uint16_t *tables;

    if (idx == 0)
        tables = (uint16_t *)bbm_ctx.tblk1;
    else
        tables = (uint16_t *)bbm_ctx.tblk2;

    if (blk == tables[0])
    {
        obptr = &tables[0];
        nbptr = &tables[1];
    }
    else if (blk == tables[1])
    {
        obptr = &tables[1];
        nbptr = &tables[0];
    }
    else
    {
        BBM_ERR("invalid block %d for table %d\n", blk, idx);
        BBM_ASSERT(0);
    }

    if (*nbptr != 0)  // idx table use 2 blocks
        nblk1 = *nbptr;
    else
    {
        if (bbm_ctx.idle_blk[0] != 0)
        {
            nblk1 = bbm_ctx.idle_blk[0];
            *nbptr = bbm_ctx.idle_blk[0];
            bbm_ctx.idle_blk[0] = 0;
        }
        else if (bbm_ctx.idle_blk[1] != 0)
        {
            nblk1 = bbm_ctx.idle_blk[1];
            *nbptr = bbm_ctx.idle_blk[1];
            bbm_ctx.idle_blk[1] = 0;
        }
        else
        {
            BBM_ERR("Idle block not enough for new tab %d\n", idx);
            BBM_ASSERT(0);
        }
    }

    return nblk1;
}
static int bbm_write_talbe(int tid, uint16_t blk)
{
    int page;

    if ((tid > 1) || (tid < 0))
    {
        BBM_ASSERT(0);
    }

    page = bbm_get_page_num(blk);
    if ((page < 0) || (page >= (int)(bbm_blk_size / bbm_page_size)))
        return -1;

    memset((void *)bbm_page_cache, 0xff, bbm_page_size);
    memcpy((void *)bbm_page_cache, (void *)&bbm_local[tid], sizeof(Sifli_NandBBM));

    // check map table
    if (bbm_map_check((Sifli_NandBBM *)bbm_page_cache) != 0)
        return -2;

    int res = port_write_page(blk, page, (uint8_t *) bbm_page_cache, NULL, 0);
    if (res <= 0)
        return -1;

    return 0;
}

// return 0 for 2 table all found, return 1 for no map block used, 2 for each map use at lease 1 block, others error
static int bbm_det_bbm_blk(void)
{
    int blk, page, res, tab1_cnt, tab2_cnt, tab_cnt, bad, status;

    blk = user_blk;
    page = 0;
    tab1_cnt = 0;
    tab2_cnt = 0;
    tab_cnt = 0;
    status = -1;
    while (blk < (int)total_blk)
    {
        bad = bbm_get_bb(blk);
        if (bad)
        {
            blk++;
            BBM_INFO("DET %d bad\n", blk);
            continue;
        }
#ifdef  BBM_TABLE_AUTO_TEST
        int fmt = bbm_get_test_format(blk, 0);
        if (fmt == ERR_FACT_BAD)
        {
            blk++;
            BBM_DBG("DET %d bad\n", blk);
            continue;
        }
#endif
        memset(bbm_page_cache, 0xff, bbm_page_size);
        res = port_read_page(blk, page, 0, bbm_page_cache, bbm_page_size, NULL, 0);
        if (res == (int)bbm_page_size)
        {
            Sifli_NandBBM *bbmt = (Sifli_NandBBM *)bbm_page_cache;
            if (bbmt->magic == BBM_MAGIC_NUM)   // table block
            {
                if (bbmt->tbl_idx == 0) // table 1
                {
                    //mark as table 1
                    bbm_ctx.tblk1[tab1_cnt] = blk;
                    tab1_cnt++;
                    BBM_DBG("tab1 %d\n", tab1_cnt);
                }
                else    // table 2
                {
                    //mark as table 2
                    bbm_ctx.tblk2[tab2_cnt] = blk;
                    tab2_cnt++;
                    BBM_DBG("tab2 %d\n", tab2_cnt);
                }
            }
            else // empty table
            {
                bbm_ctx.idle_blk[tab_cnt] = blk;
                tab_cnt++;
                BBM_DBG("empty table %d\n", tab_cnt);
            }
        }
        else // read fail?
        {
            //mark as bad
            //bbm_mark_bb(blk);
            blk++;
            BBM_INFO("BLK %d read fail, mark as bad\n", blk);
            continue;
        }

        if ((tab_cnt + tab1_cnt + tab2_cnt)  >= 4)   // found 4 blocks for saving bbm
        {
            if ((tab1_cnt == 0) || (tab2_cnt == 0)) // init
                status = 1;
            else if ((tab1_cnt >= 2) && (tab2_cnt >= 2)) // all valid table blocks found
                status = 0;
            else  // each table used at least 1 block
            {
                int idleblk = 0;
                status = 2;
                idleblk = bbm_ctx.idle_blk[0];
                if (bbm_ctx.tblk1[0] == 0)
                {
                    bbm_ctx.tblk1[0] = idleblk;
                    idleblk = 0;
                }
                else if (bbm_ctx.tblk1[1] == 0)
                {
                    bbm_ctx.tblk1[1] = idleblk;
                    idleblk = 0;
                }

                if (idleblk == 0)
                    idleblk = bbm_ctx.idle_blk[1];
                if (bbm_ctx.tblk2[0] == 0)
                    bbm_ctx.tblk2[0] = idleblk;
                else if (bbm_ctx.tblk2[1] == 0)
                    bbm_ctx.tblk2[1] = idleblk;
            }
            break;
        }
        blk++;
    }
    BBM_INFO("det bbm table with %d, %d, %d\n", tab1_cnt, tab2_cnt, tab_cnt);

    return status;
}

static int bbm_get_version_inblk(int blk, int *plast_page)
{
    int vers, res;
    uint32_t page;
    Sifli_NandBBM *tab;

    if (plast_page == NULL)
        return 0;

    vers = 0;
    for (page = 0; page < bbm_blk_size / bbm_page_size; page++)
    {
        res = port_read_page(blk, page, 0, bbm_page_cache, bbm_page_size, NULL, 0);
        if (res > 0)
        {
            tab = (Sifli_NandBBM *)bbm_page_cache;
            if (tab->magic == BBM_MAGIC_NUM)
            {
                if (tab->version > vers)
                {
                    uint32_t cval = bbm_crc_check((const uint8_t *) tab, 16);   //sizeof(Sifli_NandBBM) - sizeof(stru_tbl) - 4*2
                    if (cval == tab->hdr_crc)   // header crc check ok
                    {
                        vers = tab->version;
                        // do not save, there only one buffer //memcpy(&bbm_local[tid], tab, sizeof(Sifli_NandBBM));
                        *plast_page = page;
                    }
                }
                else
                {
                    BBM_ERR("BBM version not increased: prev %d, curr %d\n", vers, tab->version);
                }
            }
            else    // page not used
            {
                if (tab->magic == 0XFFFFFFFF)  // if not write for new page, previous read should be latest version
                    break;
                else
                {
                    BBM_ERR("DATA not reasonable in BBM blk %d page %d: 0x%x\n", blk, page, tab->magic);
                    return 0; // do not use this block data
                }
            }
        }
        else
        {
            BBM_ERR("Read bbm blk %d page %d fail\n", blk, page);
            //TODO, continue to read next page but not return 0;
        }
    }

    return vers;
}
int bbm_get_map_table(int tid)
{
    int b1, b2, blk, res, page, p1, p2;
    uint32_t v1, v2, vers;
    Sifli_NandBBM *tab;

    if (tid > 1)
    {
        BBM_ERR("Invalid BBM IDX %d\n", tid);
        return 0;
    }


    if (tid == 0)
    {
        b1 = bbm_ctx.tblk1[0];
        b2 = bbm_ctx.tblk1[1];
    }
    else
    {
        b1 = bbm_ctx.tblk2[0];
        b2 = bbm_ctx.tblk2[1];
    }
    BBM_DBG("block1 %d, block2 %d\n", b1, b2);

    vers = 0;
    v1 = 0;
    v2 = 0;
    if (b1 > 0)
        v1 = bbm_get_version_inblk(b1, &p1);

    if (b2 > 0)
        v2 = bbm_get_version_inblk(b2, &p2);

    BBM_INFO("V1 %d in block %d, V2 %d in block %d\n", v1, b1, v2, b2);
    // check which block with large version
    if (v1 == v2)   // something wrong or not initial
        return 0;

    if (v1 > v2) // block 1 with large version
    {
        blk = b1;
        bbm_ctx.cur_blk[tid] = b1;
        page = p1;
        vers = v1;
    }
    else
    {
        blk = b2;
        bbm_ctx.cur_blk[tid] = b2;
        page = p2;
        vers = v2;
    }

    // read map table and check crc
    res = port_read_page(blk, page, 0, bbm_page_cache, bbm_page_size, NULL, 0);
    if (res > 0)
    {
        tab = (Sifli_NandBBM *)bbm_page_cache;
        if (tab->magic == BBM_MAGIC_NUM)
        {
            uint32_t cval = bbm_crc_check((const uint8_t *) tab, 16);   //sizeof(Sifli_NandBBM) - sizeof(stru_tbl) - 4*2
            if (cval == tab->hdr_crc)   // header crc check ok
            {
                if (tab->version == vers)   // get version should same to prev checked max version
                    memcpy(&bbm_local[tid], tab, sizeof(Sifli_NandBBM));
                else
                {
                    BBM_ERR("Samething must be wrong, get new version %d do not same to prev check %d\n", tab->version, vers);
                    BBM_ASSERT(0);
                }
            }
            else
            {
                BBM_ERR("CRC check error \n", cval, tab->hdr_crc);
                BBM_ASSERT(0);
            }
        }
        else   // should not happened, it's has been read OK at previous check
        {
            BBM_ERR("Read bbm blk %d page %d data not write for 2nd time\n", blk, page);
            BBM_ASSERT(0);
        }
    }
    else
    {
        BBM_ERR("Read bbm blk %d page %d fail for 2nd time?\n", blk, page);
        // read fail, can check backup blk , this blk version as invalid
        return 0; //BBM_ASSERT(0);
    }

    BBM_INFO("Latest version %d\n", vers);

    return vers;
}

int bbm_init_table()
{
    int bad, cnt, i;
    int next_idle, res, tab_cnt;
    if ((bbm_local[0].magic != BBM_MAGIC_NUM) && (bbm_local[1].magic != BBM_MAGIC_NUM)) // no table exist
    {
        bbm_local[0].magic = BBM_MAGIC_NUM;
        bbm_local[0].version = 1;
        bbm_local[0].tbl_idx = 0;
        bbm_local[0].bbk_num = 0;
        //bbm_local[0].free_blk_num = bkup_blk - BBM_MAX(BBM_MAX(bbm_ctx.tblk1[0],bbm_ctx.tblk1[1]),BBM_MAX(bbm_ctx.tblk2[0]),bbm_ctx.tblk2[1]);
        bbm_local[0].free_blk_num = bkup_blk - 4;   // reserved at least 4 blocks for map table
        bbm_local[0].free_blk_start = total_blk - 1;
        bbm_local[0].reserv_blk_start = user_blk;
        //bbm_local[0].hdr_crc = ;
        // bbm_local[0].tbl_crc = ;
        //bbm_local[0].stru_tbl;
    }
    else
    {
        BBM_ERR("Initial table fail\n");
        BBM_ASSERT(0);
    }
#if 1
    // erase all reserved blocks except bad block
    for (i = (int)user_blk; i < (int)total_blk; i++)
    {
        bad = bbm_get_bb(i);
        if (bad == 0)
        {
#ifdef  BBM_TABLE_AUTO_TEST
            int fmt = bbm_get_test_format(i, 0);
            if ((fmt == ERR_FACT_BAD) || (fmt == ERR_E_FAIL))
            {
                BBM_INFO("Block %d test as bad block\n", i);
                bbm_mark_bb(i);
                bbm_local[0].free_blk_num--;
                continue;
            }
#endif
            int res = port_erase_block(i);
            if (res != 0)
            {
                BBM_WARN("Block %d erase fail, mark as bad\n", i);
                //bbm_mark_bb(i);
                bbm_local[0].free_blk_num--;
            }
        }
        else
        {
            BBM_INFO("Block %d check as bad block\n", i);
            bbm_local[0].free_blk_num--;
        }
    }
    BBM_DBG("All reserved block erased\n");
    // parse all blocks except reserved
    cnt = 0;
    for (i = 0; i < (int)user_blk; i++)
    {
        bad = bbm_get_bb(i);
#ifdef  BBM_TABLE_AUTO_TEST
        int fmt = bbm_get_test_format(i, 0);
#endif
        if (bad
#ifdef  BBM_TABLE_AUTO_TEST
                || (fmt == ERR_FACT_BAD)
#endif
           )
        {
            BBM_INFO("Block %d is bad in user block\n", i);
            // only factory mark bad at initial
            // try to find a idle block
            do
            {
                bad = bbm_get_bb(bbm_local[0].free_blk_start);
#ifdef  BBM_TABLE_AUTO_TEST
                fmt = bbm_get_test_format(bbm_local[0].free_blk_start, 0);
                if ((fmt == ERR_FACT_BAD) || (fmt == ERR_E_FAIL))
                {
                    bad = 1;
                }
#endif
                bbm_local[0].free_blk_start--;
                bbm_local[0].free_blk_num--;
            }
            while ((bad != 0) && (bbm_local[0].free_blk_num > 0));
            BBM_INFO("bad %d, replace %d\n", i, cnt);
            if (bad == 0)   // found idle block to do map
            {
                bbm_local[0].stru_tbl[cnt].logic_blk = i;
                bbm_local[0].stru_tbl[cnt].physical_blk = bbm_local[0].free_blk_start + 1;
            }
            cnt++;
            bbm_local[0].bbk_num++;
        }
    }
    if (bbm_local[0].free_blk_num <= 0)
    {
        BBM_ERR("No backup block any more\n");
        BBM_ASSERT(0);
    }
#endif
    BBM_DBG("Update table to flash\n");

    bbm_local[0].hdr_crc = bbm_crc_check((const uint8_t *)(&bbm_local[0]), 16);
    bbm_local[0].tbl_crc = bbm_crc_check((const uint8_t *)(&(bbm_local[0].stru_tbl)), sizeof(Sifli_MapTbl) * (bkup_blk - 4));
    memcpy((void *)&bbm_local[1], (void *)&bbm_local[0], sizeof(Sifli_NandBBM));

    bbm_local[1].tbl_idx = 1;
    bbm_local[1].hdr_crc = bbm_crc_check((const uint8_t *)(&bbm_local[1]), 16);

    // get 4 idle block for init
    next_idle = user_blk;
    tab_cnt = 0;
    while (next_idle < (int)total_blk)
    {
        bad = bbm_get_bb(next_idle);
        if (bad)
        {
            next_idle++;
            BBM_DBG("DET %d bad\n", next_idle);
            continue;
        }
        memset(bbm_page_cache, 0xff, bbm_page_size);
        res = port_read_page(next_idle, 0, 0, bbm_page_cache, bbm_page_size, NULL, 0);
        if (res == (int)bbm_page_size)
        {
            Sifli_NandBBM *bbmt = (Sifli_NandBBM *)bbm_page_cache;
            if (bbmt->magic == 0xffffffff)   // empty
            {
                bbm_ctx.idle_blk[tab_cnt] = next_idle;
                tab_cnt++;
                BBM_DBG("empty table %d\n", tab_cnt);
            }
        }
        if (tab_cnt >= 4)   // find 4 idle
            break;
        next_idle++;
    }

    if (tab_cnt < 4)
    {
        BBM_ERR("empty table %d not enough for initial\n", tab_cnt);
        BBM_ASSERT(0);
    }

    // write to flash for idx 0
    {
        BBM_DBG("Update table 1 to flash\n");
        bbm_write_talbe(0, bbm_ctx.idle_blk[0]);
        bbm_ctx.tblk1[0] = bbm_ctx.idle_blk[0];
        bbm_ctx.idle_blk[0] = 0;
        bbm_ctx.cur_blk[0] = bbm_ctx.tblk1[0];
    }

    // write to flash for idx 1
    {
        BBM_DBG("Update table 2 to flash\n");
        bbm_write_talbe(1, bbm_ctx.idle_blk[1]);
        bbm_ctx.tblk2[0] = bbm_ctx.idle_blk[1];
        bbm_ctx.idle_blk[1] = 0;
        bbm_ctx.cur_blk[1] = bbm_ctx.tblk2[0];
    }

    bbm_ctx.tblk1[1] = bbm_ctx.idle_blk[2];
    bbm_ctx.tblk2[1] = bbm_ctx.idle_blk[3];

    BBM_INFO("Update table to flash done\n");

    return 0;
}


void bbm_register_log(bbm_log_func log_func)
{
    g_bbm_dlog = log_func;
}

int sif_bbm_init(uint32_t total, uint8_t *cache)
{
    int sta, v1, v2;
    if (cache == NULL)
        return -1;

    if (gbbm_init_flag == 1)
    {
        BBM_INFO("BBM initialized before, do not init any more\n");
        return 0;
    }
    gbbm_init_flag = 1;

    BBM_ASSERT(bbm_blk_size);
    BBM_ASSERT(bbm_page_size);

    bbm_page_cache = cache;
    total_blk = total / bbm_blk_size;    // Bytes to Blocks
    bkup_blk = total_blk >> 5;  // 1/32 of total blocks , 1Gb use 32 blocks, 512Mb use 16 Blocks
    user_blk = total_blk - bkup_blk;    // remain blocks for user

    memset((void *)&bbm_ctx, 0, sizeof(NAND_BBM_CTX_T));
    memset((void *)&bbm_local[0], 0, sizeof(Sifli_NandBBM) * 2);
#ifdef BBM_TABLE_AUTO_TEST
    // generate bbm test table
    bbm_init_test_table();
#endif

    sta = bbm_det_bbm_blk();
    BBM_INFO("detect result %d\n", sta);
    if (sta == 0 || sta == 2)  // find 2 tab block success, get table and check 2 table status
    {
        v1 = bbm_get_map_table(0);
        v2 = bbm_get_map_table(1);
        BBM_INFO("v1 %d in blk %d, v2 %d in block %d\n", v1, bbm_ctx.cur_blk[0], v2, bbm_ctx.cur_blk[1]);
        if (v1 > v2)   // table 1 with latest version
        {
            // use v1 replace v2
            memcpy((void *)&bbm_local[1], (void *)&bbm_local[0], sizeof(Sifli_NandBBM));
            bbm_local[1].tbl_idx = 1;
            bbm_local[1].hdr_crc = bbm_crc_check((const uint8_t *)(&bbm_local[1]), 16);
            bbm_write_talbe(1, bbm_ctx.cur_blk[1]);
        }
        else if (v1 < v2)   // table 2 with latest version
        {
            // use v2 replace v1
            memcpy((void *)&bbm_local[0], (void *)&bbm_local[1], sizeof(Sifli_NandBBM));
            bbm_local[0].tbl_idx = 0;
            bbm_local[0].hdr_crc = bbm_crc_check((const uint8_t *)(&bbm_local[0]), 16);
            bbm_write_talbe(0, bbm_ctx.cur_blk[0]);
        }
        if (bbm_map_check(&bbm_local[0]) != 0)
        {
            BBM_ERR("Check bbm table fail\n");
            BBM_ASSERT(0);
        }
        //else //  do nothing
    }
    else if (sta == 1)  // initial status
    {
        bbm_init_table();
    }
    else
    {
        BBM_ERR("detect result %d not reasonable\n", sta);
        BBM_ASSERT(0);
    }
    BBM_INFO("BBM MEM: ctx %p, map1 %p, map2 %p \n", (void *)&bbm_ctx, (void *)&bbm_local[0], (void *)&bbm_local[1]);

    BBM_INFO("sif_bbm_init done\n");
    return 0;
}

int bbm_get_phy_blk(uint16_t log_blk)
{
    int i, res;

    if (log_blk >= user_blk)
        return -1;

    if (log_blk == 0)   // block 0 as golden block, can not bad!
        return 0;

    res = log_blk;
    for (i = 0; i < (int)(bkup_blk - 4); i++)
    {
        if (bbm_local[0].stru_tbl[i].logic_blk == log_blk)
        {
            res = bbm_local[0].stru_tbl[i].physical_blk;
            if (res < (int)user_blk) // map error!
            {
                BBM_ERR("Get map block error %d --> %d\n", log_blk, res);
                BBM_ASSERT(0);
            }
            break;
        }
        else if ((bbm_local[0].stru_tbl[i].logic_blk == 0) && (bbm_local[0].stru_tbl[i].physical_blk == 0)) // not find in valid table
        {
            break;
        }
    }

    return res;
}

int bbm_read_page(int blk, int page, int offset, uint8_t *buff, uint32_t size, uint8_t *spare, uint32_t spare_len)
{
    int nblk = bbm_get_phy_blk(blk);
    if (nblk < 0)
    {
        BBM_ERR("Get phy blk for %d fail when read\n", blk);
        return 0;
    }
    if (blk != nblk) /// use map block
    {
        BBM_DBG("READ: src blk %d, dst blk %d\n", blk, nblk);
    }
#ifdef  BBM_TABLE_AUTO_TEST
    int fmt = bbm_get_test_format(blk, page);
    if (fmt == ERR_ECC_FAIL)
    {
        BBM_INFO("Simu ecc fail for blk %d\n", blk);
        return 0;
    }
#endif

    return port_read_page(nblk, page, offset, buff, size, spare, spare_len);
}

int bbm_write_page(int blk, int page, uint8_t *data, uint8_t *spare, uint32_t spare_len)
{
    int rcnt;
    int nblk = bbm_get_phy_blk(blk);
    if (nblk < 0)
    {
        BBM_ERR("Get phy blk for %d fail when write\n", blk);
        return 0;
    }

    if (blk != nblk) /// use map block
    {
        BBM_DBG("WRITE: src blk %d, dst blk %d\n", blk, nblk);
    }
    rcnt = 0;   // max 3, if try 3 times not found a good block to write, assert
    int res = port_write_page(nblk, page, data, spare, spare_len);
#ifdef  BBM_TABLE_AUTO_TEST
    int fmt = bbm_get_test_format(blk, page);
    if (fmt == ERR_P_FAIL)
    {
        BBM_INFO("Simu prog fail for blk %d\n", blk);
        res = RET_P_FAIL;
    }
#endif

    if (res <= 0)
    {
        //if (res == RET_P_FAIL)  // block error, replace it
        {
            int blk2;
retry:
            do
            {
                blk2 = bbm_map_new_blk(blk);
                res = port_erase_block(blk2);
            }
            while (res != 0);
            BBM_INFO("Write to blk %d fail, map to blk %d\n", blk, blk2);
            // suppose write always be page increased? copy data before this page
            int i;
            for (i = 0; i < page; i++)
            {
                // ignor error, data haved lost before if read error
                res = port_read_page(nblk, i, 0, bbm_page_cache, bbm_page_size, NULL, 0);
                if (res != (int)bbm_page_size)
                {
                    // old block can not read ?
                    BBM_ERR("Old blck %d read fail\n", nblk);
                    HAL_ASSERT(0);
                }
                res = port_write_page(blk2, i, bbm_page_cache, NULL, 0);
                if (res <= 0)
                {
                    if (res == RET_P_FAIL)
                    {
                        rcnt++;
                        if (rcnt >= 3)
                        {
                            BBM_ERR("Map blk %d fail more than 3 times\n", blk);
                            HAL_ASSERT(0);
                        }
                        goto retry;
                    }
                    else
                    {
                        BBM_ERR("Map blk %d fail more than 3 times\n", blk);
                        HAL_ASSERT(0);
                        return 0;
                    }
                }
            }
            // write again, check result and do while?
            res = port_write_page(blk2, page, data, spare, spare_len);
            if (res <= 0)
            {
                if (res == RET_P_FAIL)
                {
                    rcnt++;
                    if (rcnt >= 3)
                    {
                        BBM_ERR("Map blk %d fail more than 3 times\n", blk);
                        HAL_ASSERT(0);
                    }
                    goto retry;
                }
                else
                {
                    HAL_ASSERT(0);
                    return 0;
                }
            }
#if 0   // ignore pages after this page, as FORESEE AE said, nand can not write back
            for (i = page + 1; i < bbm_blk_size / bbm_page_size; i++)
            {
                // ignor error, data haved lost before if read error
                res = port_read_page(nblk, i, 0, bbm_page_cache, bbm_page_size, NULL, 0);
                if (res != bbm_page_size)
                {
                    // old block can not read ?
                    LOG_E("Old blck %d read fail\n", nblk);
                    HAL_ASSERT(0);
                }
                res = port_write_page(blk2, i, bbm_page_cache, NULL, 0);
                if (res <= 0)
                {
                    if (res == RET_P_FAIL)
                    {
                        rcnt++;
                        if (rcnt >= 3)
                            HAL_ASSERT(0);
                        goto retry;
                    }
                    else
                    {
                        HAL_ASSERT(0);
                        return 0;
                    }
                }
            }
#endif
        }
    }

    return res;
}

int bbm_erase_block(int blk)
{
    int nblk = bbm_get_phy_blk(blk);
    if (nblk < 0)
    {
        BBM_ERR("Get phy blk for %d fail when erase\n", blk);
        return -1;
    }

    if (blk != nblk) /// use map block
    {
#if !defined(CFG_FACTORY_DEBUG)
        BBM_INFO("ERASE: src blk %d, dst blk %d\n", blk, nblk);
#endif
    }
    int res = port_erase_block(nblk);
#ifdef  BBM_TABLE_AUTO_TEST
    int fmt = bbm_get_test_format(blk, 0);
    if ((fmt == ERR_E_FAIL) || (fmt == ERR_FACT_BAD))
    {
        res = RET_E_FAIL;
    }
#endif
    if (res != 0)
    {
        //if (res == RET_E_FAIL) // block error, replace it
        if (1) // for some special cases, erase return fail not only E_FAIL
        {
            int blk2;
            do
            {
                blk2 = bbm_map_new_blk(blk);
                res = port_erase_block(blk2);
            }
            while (res != 0);
            BBM_ERR("Map blk %d to blk %d when erase\n", blk, blk2);
        }
        else
            return res;
    }

    return 0;
}

int bbm_get_total()
{
    return bkup_blk * bbm_blk_size;
}

void *bbm_get_context(uint8_t idx)
{
    if (idx == 0)
        return (void *)(&bbm_local[0]);
    else if (idx == 1)
        return (void *)(&bbm_local[1]);
    else
        HAL_ASSERT(0);

    return NULL;
}

void bbm_dump_info()
{
    int i;
    Sifli_NandBBM *btabl = &bbm_local[0];

    BBM_INFO("bbm version : %d\n", btabl->version);
    BBM_INFO("BLK MAP: \n");
    for (i = 0; i < 128; i++)
    {
        if (btabl->stru_tbl[i].logic_blk == 0)
            break;
        BBM_INFO("Logic %d, Phy %d\n", btabl->stru_tbl[i].logic_blk, btabl->stru_tbl[i].physical_blk);
    }
}

#ifndef BBM_UT

__weak int port_read_page(int blk, int page, int offset, uint8_t *buff, uint32_t size, uint8_t *spare, uint32_t spare_len)
{
    return size;
}

__weak int port_write_page(int blk, int page, uint8_t *data, uint8_t *spare, uint32_t spare_len)
{
    return bbm_page_size;
}

__weak int port_erase_block(int blk)
{
    return 0;
}

__weak int bbm_mark_bb(int blk)
{
    return 0;
}

__weak int bbm_get_bb(int blk)
{
    return 0;
}

void bbm_set_page_size(uint32_t page_size)
{
    bbm_page_size = page_size;
}

void bbm_set_blk_size(uint32_t blk_size)
{
    bbm_blk_size = blk_size;
}


#endif
#ifdef BBM_TABLE_AUTO_TEST
static int bbm_init_test_table()
{
    int i;
    for (i = 0; i < BBM_TEST_MAX_CNT; i++)
    {
        srand(HAL_GTIMER_READ());
        g_bbm_test_table[i].block = rand() % total_blk; // 0 ~ total_blk-1; how to discard selected block?
        g_bbm_test_table[i].page = rand() % 64;    // 0 ~ 63
        g_bbm_test_table[i].err_type = rand() % ERR_CNT;    // ERR_P_FAIL ~ ERR_FACT_BAD
    }
    return 0;
}

static int bbm_get_test_format(uint16_t blk, uint8_t page)
{
    int i;
    int res = 0;
    for (i = 0; i < BBM_TEST_MAX_CNT; i++)
    {
        if ((blk == g_bbm_test_table[i].block) && (g_bbm_test_table[i].err_type > ERR_TYPE_NON)
                && (g_bbm_test_table[i].err_type < ERR_CNT))
        {
            if ((g_bbm_test_table[i].err_type == ERR_FACT_BAD)
                    || (g_bbm_test_table[i].err_type == ERR_E_FAIL))
            {
                res = g_bbm_test_table[i].err_type;
                break;
            }
            if (page == g_bbm_test_table[i].page)
            {
                res = g_bbm_test_table[i].err_type;
                break;
            }
        }
    }

    return res;
}

#endif

#ifdef BBM_UT
    #include "test_bbm.c"
#endif

