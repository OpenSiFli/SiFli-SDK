#include "yaffs/direct/yaffs_flashif2.h"
#include "yaffs_trace.h"

#if 0
    #include "yaffs_packedtags2.h"

    #define YAFFS_SPARE_SIZE   (64)
    #define YAFFS_SPARE_USER_DATA_SIZE    (4 * 4)
#endif

static int write_chunk(struct yaffs_dev *dev, int nand_chunk,
                       const u8 *data, int data_len,
                       const u8 *oob, int oob_len)
{
    return rt_mtd_nand_write(RT_MTD_NAND_DEVICE(dev->driver_context), nand_chunk,  data, data_len, oob, oob_len) ? YAFFS_FAIL : YAFFS_OK;
}

static int read_chunk(struct yaffs_dev *dev, int nand_chunk,
                      u8 *data, int data_len,
                      u8 *oob, int oob_len,
                      enum yaffs_ecc_result *ecc_result)
{
    int ret = -1;

    if (data == NULL && oob == NULL)
    {
        goto exit_read_chunk;
    }

    ret = rt_mtd_nand_read(RT_MTD_NAND_DEVICE(dev->driver_context),
                           nand_chunk, data, data_len, oob, oob_len);

    if (ret == RT_MTD_EOK)
    {
        *ecc_result = YAFFS_ECC_RESULT_NO_ERROR;
    }
    else if (ret == -RT_MTD_EECC_CORRECT)
    {
        *ecc_result = YAFFS_ECC_RESULT_FIXED;
    }
    else if (ret == -RT_MTD_EECC)
    {
        *ecc_result = YAFFS_ECC_RESULT_UNFIXED;
    }

exit_read_chunk:

    return ret ? YAFFS_FAIL : YAFFS_OK;
}

static int erase(struct yaffs_dev *dev, int block_no)
{
    return rt_mtd_nand_erase_block(RT_MTD_NAND_DEVICE(dev->driver_context), block_no) ? YAFFS_FAIL : YAFFS_OK;
}

static int mark_bad(struct yaffs_dev *dev, int block_no)
{
    return rt_mtd_nand_mark_badblock(RT_MTD_NAND_DEVICE(dev->driver_context), block_no) ? YAFFS_FAIL : YAFFS_OK;
}

static int check_bad(struct yaffs_dev *dev, int block_no)
{
    return rt_mtd_nand_check_block(RT_MTD_NAND_DEVICE(dev->driver_context), block_no) ? YAFFS_FAIL : YAFFS_OK;
}

static int initialise(struct yaffs_dev *dev)
{
    return YAFFS_OK;
}

static int deinitialise(struct yaffs_dev *dev)
{
    return YAFFS_OK;
}

#if 0
static int write_chunk_tags(struct yaffs_dev *dev,
                            int nand_chunk, const u8 *data,
                            const struct yaffs_ext_tags *tags)
{
    struct yaffs_packed_tags2 pt;
    int retval;
    u8 spare_buffer[YAFFS_SPARE_SIZE];
    u8 *rd_ptr;
    u8 *wr_ptr;

    int packed_tags_size =
        dev->param.no_tags_ecc ? sizeof(pt.t) : sizeof(pt);
    void *packed_tags_ptr =
        dev->param.no_tags_ecc ? (void *)&pt.t : (void *)&pt;

    yaffs_trace(YAFFS_TRACE_MTD,
                "yaffs_tags_marshall_write chunk %d data %p tags %p",
                nand_chunk, data, tags);

    if (dev->param.inband_tags || !dev->param.no_tags_ecc)
        BUG();

    if (packed_tags_size > YAFFS_SPARE_USER_DATA_SIZE)
        return YAFFS_FAIL;



    /* For yaffs2 writing there must be both data and tags.
     * If we're using inband tags, then the tags are stuffed into
     * the end of the data buffer.
     */
    if (!data || !tags)
        BUG();
    else if (dev->param.inband_tags)
    {
        struct yaffs_packed_tags2_tags_only *pt2tp;
        pt2tp =
            (struct yaffs_packed_tags2_tags_only *)(data +
                    dev->data_bytes_per_chunk);
        yaffs_pack_tags2_tags_only(dev, pt2tp, tags);
    }
    else
    {
        //yaffs_pack_tags2(dev, &pt, tags, !dev->param.no_tags_ecc);

        yaffs_pack_tags2_tags_only(dev, &pt.t, tags);
        memset(spare_buffer, 0xFF, sizeof(spare_buffer));
        wr_ptr = &spare_buffer[4];
        rd_ptr = (u8 *)packed_tags_ptr;
        while (packed_tags_size >= 4)
        {
            memcpy(wr_ptr, rd_ptr, 4);
            wr_ptr += 16;
            rd_ptr += 4;
            packed_tags_size -=  4;
        }
        if (packed_tags_size > 0)
        {
            memcpy(wr_ptr, rd_ptr, packed_tags_size);
        }
    }

#if 0
    retval = dev->drv.drv_write_chunk_fn(dev, nand_chunk,
                                         data, dev->param.total_bytes_per_chunk,
                                         (dev->param.inband_tags) ? NULL : packed_tags_ptr,
                                         (dev->param.inband_tags) ? 0 : packed_tags_size);
#else
    retval = dev->drv.drv_write_chunk_fn(dev, nand_chunk,
                                         data, dev->param.total_bytes_per_chunk,
                                         (void *)spare_buffer, YAFFS_SPARE_SIZE);

#endif


    return retval;



}

static int read_chunk_tags(struct yaffs_dev *dev,
                           int nand_chunk, u8 *data,
                           struct yaffs_ext_tags *tags)
{
    int retval = 0;
    int local_data = 0;
    u8 spare_buffer[YAFFS_SPARE_SIZE];
    enum yaffs_ecc_result ecc_result;
    int i;
    u8 *rd_ptr;
    u8 *wr_ptr;

    struct yaffs_packed_tags2 pt;

    int packed_tags_size =
        dev->param.no_tags_ecc ? sizeof(pt.t) : sizeof(pt);
    void *packed_tags_ptr =
        dev->param.no_tags_ecc ? (void *)&pt.t : (void *)&pt;

    yaffs_trace(YAFFS_TRACE_MTD,
                "yaffs_tags_marshall_read chunk %d data %p tags %p",
                nand_chunk, data, tags);

    if (dev->param.inband_tags || !dev->param.no_tags_ecc)
        BUG();

    if (packed_tags_size > YAFFS_SPARE_USER_DATA_SIZE)
        return YAFFS_FAIL;

    if (dev->param.inband_tags)
    {
        if (!data)
        {
            local_data = 1;
            data = yaffs_get_temp_buffer(dev);
        }
    }

    if (dev->param.inband_tags || (data && !tags))
        retval = dev->drv.drv_read_chunk_fn(dev, nand_chunk,
                                            data, dev->param.total_bytes_per_chunk,
                                            NULL, 0,
                                            &ecc_result);
    else if (tags)
        retval = dev->drv.drv_read_chunk_fn(dev, nand_chunk,
                                            data, dev->param.total_bytes_per_chunk,
                                            spare_buffer, YAFFS_SPARE_SIZE,
                                            &ecc_result);
    else
        BUG();


    if (retval == YAFFS_FAIL)
        return YAFFS_FAIL;

    if (dev->param.inband_tags)
    {
        if (tags)
        {
            struct yaffs_packed_tags2_tags_only *pt2tp;
            pt2tp =
                (struct yaffs_packed_tags2_tags_only *)
                &data[dev->data_bytes_per_chunk];
            yaffs_unpack_tags2_tags_only(dev, tags, pt2tp);
        }
    }
    else if (tags)
    {
        rd_ptr = &spare_buffer[4];
        wr_ptr = (u8 *)packed_tags_ptr;
        while (packed_tags_size >= 4)
        {
            memcpy(wr_ptr, rd_ptr, 4);
            rd_ptr += 16;
            wr_ptr += 4;
            packed_tags_size -= 4;
        }
        if (packed_tags_size > 0)
        {
            memcpy(wr_ptr, rd_ptr, packed_tags_size);
        }
        yaffs_unpack_tags2(dev, tags, &pt, !dev->param.no_tags_ecc);
    }

    if (local_data)
        yaffs_release_temp_buffer(dev, data);

    if (tags && ecc_result == YAFFS_ECC_RESULT_UNFIXED)
    {
        tags->ecc_result = YAFFS_ECC_RESULT_UNFIXED;
        dev->n_ecc_unfixed++;
    }

    if (tags && ecc_result == YAFFS_ECC_RESULT_FIXED)
    {
        if (tags->ecc_result <= YAFFS_ECC_RESULT_NO_ERROR)
            tags->ecc_result = YAFFS_ECC_RESULT_FIXED;
        dev->n_ecc_fixed++;
    }

    if (ecc_result < YAFFS_ECC_RESULT_UNFIXED)
        return YAFFS_OK;
    else
        return YAFFS_FAIL;
}


static int query_block(struct yaffs_dev *dev, int block_no,
                       enum yaffs_block_state *state,
                       u32 *seq_number)
{
    int retval;

    yaffs_trace(YAFFS_TRACE_MTD, "yaffs_tags_marshall_query_block %d",
                block_no);

    retval = dev->drv.drv_check_bad_fn(dev, block_no);

    if (retval == YAFFS_FAIL)
    {
        yaffs_trace(YAFFS_TRACE_MTD, "block is bad");

        *state = YAFFS_BLOCK_STATE_DEAD;
        *seq_number = 0;
    }
    else
    {
        struct yaffs_ext_tags t;

        read_chunk_tags(dev,
                        block_no * dev->param.chunks_per_block,
                        NULL, &t);

        if (t.chunk_used)
        {
            *seq_number = t.seq_number;
            *state = YAFFS_BLOCK_STATE_NEEDS_SCAN;
        }
        else
        {
            *seq_number = 0;
            *state = YAFFS_BLOCK_STATE_EMPTY;
        }
    }

    yaffs_trace(YAFFS_TRACE_MTD,
                "block query returns  seq %d state %d",
                *seq_number, *state);

    if (retval == 0)
        return YAFFS_OK;
    else
        return YAFFS_FAIL;
}
#endif


void yaffs_mtd_drv_install(struct yaffs_dev *dev)
{
    dev->drv.drv_deinitialise_fn = deinitialise;
    dev->drv.drv_initialise_fn   = initialise;
    dev->drv.drv_check_bad_fn    = check_bad;
    dev->drv.drv_mark_bad_fn     = mark_bad;
    dev->drv.drv_erase_fn        = erase;
    dev->drv.drv_read_chunk_fn   = read_chunk;
    dev->drv.drv_write_chunk_fn   = write_chunk;

#if 0
    dev->tagger.write_chunk_tags_fn = write_chunk_tags;
    dev->tagger.read_chunk_tags_fn = read_chunk_tags;
    dev->tagger.query_block_fn = query_block;
#endif
}

RT_WEAK int yaffs_start_up(struct rt_mtd_nand_device *psMtdNandDev, const char *pcMountingPath)
{
    rt_device_t psRTDev;
    RT_ASSERT(psMtdNandDev);

    struct yaffs_dev *psYaffsDev = (struct yaffs_dev *)rt_malloc(sizeof(struct yaffs_dev));
    if (!psYaffsDev)
    {
        rt_kprintf("Fail to memory allocation.\n");
        goto exit_yaffs_start_up;
    }
    rt_memset(psYaffsDev, 0, sizeof(struct yaffs_dev));

    psYaffsDev->param.name = pcMountingPath;
    psYaffsDev->param.inband_tags = 1;
    psYaffsDev->param.n_caches = 10;
    psYaffsDev->param.start_block = psMtdNandDev->block_start;
    psYaffsDev->param.end_block   = psMtdNandDev->block_end;;
    psYaffsDev->param.total_bytes_per_chunk = psMtdNandDev->page_size;
    psYaffsDev->param.spare_bytes_per_chunk = psMtdNandDev->oob_size;
    psYaffsDev->param.use_nand_ecc = 1;
    psYaffsDev->param.is_yaffs2 = 1;
    psYaffsDev->param.refresh_period = 1000;
    psYaffsDev->param.no_tags_ecc = 1;
    psYaffsDev->param.empty_lost_n_found = 1;
    psYaffsDev->param.n_reserved_blocks = 5;
    psYaffsDev->param.enable_xattr = 1;
    psYaffsDev->param.hide_lost_n_found = 1;
    psYaffsDev->param.always_check_erased = 0;
    psYaffsDev->param.chunks_per_block = psMtdNandDev->pages_per_block;
    psYaffsDev->driver_context = psMtdNandDev;

    yaffs_mtd_drv_install(psYaffsDev);
    yaffs_add_device(psYaffsDev);

exit_yaffs_start_up:

    psMtdNandDev->priv = (void *)psYaffsDev;
    return 0;
}

