#if 0
#include "sv6621s_mem_map.h"
#include "skw_log_to_file.h"

struct memory_segment
{
    const char  *name;
    uint32_t    address;
    uint32_t    size;
};

struct memory_segment cp_mem_seg[] =
{
    { "CODE",   CODE_MEM_BASE_ADDR, CODE_MEM_SIZE},
    { "DATA",   DATA_MEM_BASE_ADDR, DATA_MEM_SIZE},
    { "AHBR",   AHB_REG_BASE_ADDR, AHB_REG_SIZE},
    { "PPBM",   UMEM_MEM_BASE_ADDR, UMEM_MEM_SIZE}
};
static uint32_t skw_checksum(void *data, int data_len)
{
    uint32_t *d32 = data;
    uint32_t checksum = 0;
    int i;

    data_len = data_len >> 2;
    for (i = 0; i < data_len; i++)
        checksum += d32[i];
    return checksum;
}
int skw_dump_memory_into_buffer(struct ucom_dev *ucom, char *buffer, int length)
{
    struct memory_segment *mem_sg = &cp_mem_seg[0];
    int offset = 0;
    uint16_t seq, packet_len;
    uint32_t sg_size;
    char *read_buf;
    uint8_t sg_count;
    int ret = 0;

    if (!ucom || !ucom->pdata ||
            !ucom->pdata->skw_dump_mem)
        return 0;
    sg_count = sizeof(cp_mem_seg) / sizeof(cp_mem_seg[0]);
    if (sg_count == 0)
        return 0;
    packet_len = 0x800;
    read_buf = kmalloc(packet_len, GFP_KERNEL);
    if (read_buf == NULL)
        return 0;
    buffer[offset] = sg_count; //save total segment count
    offset++;

    do
    {
        uint32_t source_addr;
        sg_size = mem_sg->size;

        memcpy(&buffer[offset], mem_sg->name, 5); //save segment name
        offset += 5;
        memcpy(&buffer[offset], &mem_sg->address, 4); //save segment base addrss
        offset += 4;
        memcpy(&buffer[offset], &mem_sg->size, 4); //save segment size
        offset += 4;
        memcpy(&buffer[offset], &packet_len, 2); //save segment size
        offset += 2;
        skwlog_log("%s %s:%d 0x%x 0x%x\n", __func__, mem_sg->name,
                   offset, mem_sg->address, mem_sg->size);
        seq = 0;
        source_addr = mem_sg->address;
        do
        {
            int read_len;
            uint32_t sum;

            memcpy(&buffer[offset], &seq, 2); //save segment size
            seq++;
            offset += 2;

            if (sg_size > packet_len)
                read_len = packet_len;
            else
                read_len = sg_size;
            ret = ucom->pdata->skw_dump_mem(source_addr, (void *)read_buf, read_len);
            if (ret != 0)
            {
                skwlog_err("%s dump memory fail :%d \n", __func__, ret);
                break;
            }
            source_addr += read_len;
            memcpy(buffer + offset, read_buf, read_len); //save packet payload
            offset += read_len;

            sum = skw_checksum(read_buf, read_len);
            memcpy(buffer + offset, &sum, 4); //save checksum
            offset += 4;

            sg_size -= read_len;
        }
        while (sg_size);
        mem_sg++;
        sg_count--;
    }
    while (sg_count && (!ret));
    kfree(read_buf);
    return offset;
}
#endif
