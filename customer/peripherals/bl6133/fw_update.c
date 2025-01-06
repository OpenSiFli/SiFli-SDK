#if 1//defined(TSC_USING_BL6133)
#define CLIB_H /* don't include clib.h */

//#include "co_printf.h"
//#include "TP_ts.h"

#include "DQ0815_X27_ZCX_BOE_ST77903_KP_RQ_BL6133_320X380.h"
#include "bl_chip_common.h"


//#include "bl_jjt_qcy_rq_fw.h"
//#include "bl_fw.h"
//#include "BL6XX3_B09S_J_X6L_CJ_128X128.h"

#if defined(CTP_USE_SW_I2C) || defined(BL6133_UPDATE_USE_SW_I2C)
int bl_i2c_transfer(unsigned char i2c_addr, unsigned char *buf, int len, unsigned char rw)
{
    int ret;

    switch (rw)
    {
    case I2C_WRITE:
        ret = CTP_FLASH_I2C_WRITE(i2c_addr, buf, len);
        break;

    case I2C_READ:
        ret = CTP_FLASH_I2C_READ(i2c_addr, buf, len);
        break;
    }

    if (ret)
    {
        bl_log_trace("bl_i2c_transfer:i2c transfer error___\n");
        return -1;
    }

    return 0;
}

static int bl_read_fw(unsigned char i2c_addr, unsigned char reg_addr, unsigned char *buf, int len)
{
    int ret;

    ret = CTP_FLASH_I2C_WRITE(i2c_addr, &reg_addr, 1);

    if (ret)
    {
        goto IIC_COMM_ERROR;
    }

    ret = CTP_FLASH_I2C_READ(i2c_addr, buf, len);

    if (ret)
    {
        goto IIC_COMM_ERROR;
    }

IIC_COMM_ERROR:
    if (ret)
    {
        bl_log_trace("bl_read_fw:i2c transfer error___\n");
        return -1;
    }

    return 0;
}
#elif defined(CTP_USE_HW_I2C)

extern int bl_i2c_transfer(unsigned char i2c_addr, unsigned char *buf, int len, unsigned char rw);
extern int bl_read_fw(unsigned char i2c_addr, unsigned char reg_addr, unsigned char *buf, int len);
#if 0

int bl_i2c_transfer(unsigned char i2c_addr, unsigned char *buf, int len, unsigned char rw)
{
    int ret;

    switch (rw)
    {
    case I2C_WRITE:
        ret = CTP_FLASH_I2C_WRITE(i2c_addr, buf, len);
        break;

    case I2C_READ:
        ret = CTP_FLASH_I2C_READ(i2c_addr, buf, len);
        break;
    }

    if (ret == 0)
    {
        bl_log_trace("bl_i2c_transfer:i2c transfer error___\n");
        return -1;
    }

    return 0;
}

static int bl_read_fw(unsigned char i2c_addr, unsigned char reg_addr, unsigned char *buf, int len)
{
    int ret;

    ret = i2c_1_read(i2c_addr, I2C_1_BYTE_ADDRESS, reg_addr, buf, len);

    if (!ret)
    {
        bl_log_trace("bl_read_fw:i2c transfer error___\n");
        return -1;
    }

    return 0;
}
#endif /* 0 */
#endif

#ifdef BL_DEBUG_SUPPORT
void bl_debug_for_touch(unsigned char *debugInfo)
{
    unsigned int i = 0;
    short freq = 0x00;
    short maxNoise = 0x00;
    short minNoise = 0x00;
    bl_log_trace("debugData start:\n");
    freq = (debugInfo[0] << 8) | debugInfo[1];
    maxNoise = (debugInfo[2] << 8) | debugInfo[3];
    minNoise = (debugInfo[4] << 8) | debugInfo[5];
    bl_log_trace("freq = %d maxNoise = %d minNoise = %d\n", freq, maxNoise, minNoise);
    bl_log_trace("debugData end\n");
}
#endif

int bl_soft_reset_switch_int_wakemode()
{
    unsigned char cmd[4];
    int ret = 0x00;

    cmd[0] = RW_REGISTER_CMD;
    cmd[1] = ~cmd[0];
    cmd[2] = CHIP_ID_REG;
    cmd[3] = 0xe8;

    ret = bl_i2c_transfer(CTP_SLAVE_ADDR, cmd, 4, I2C_WRITE);
    if (ret < 0)
    {
        bl_log_trace("bl_soft_reset_switch_int_wakemode failed:i2c write flash error___\n");
    }

    return ret;
}

#ifdef I2C_UPDATE_MODE
void bl_enter_update_with_i2c(void)
{
    unsigned char cmd[200] = {0x00};
    int i = 0;
    int ret = 0;

    for (i = 0; i < sizeof(cmd); i += 2)
    {
        cmd[i] = 0x5a;
        cmd[i + 1] = 0xa5;
    }

    ret = bl_i2c_transfer(CTP_SLAVE_ADDR, cmd, sizeof(cmd), I2C_WRITE);
    if (ret < 0)
    {
        bl_log_trace("bl_enter_update_with_i2c failed:send 5a a5 error___\n");
        goto error;
    }
    MDELAY(50);

error:
    return ;
}

void bl_exit_update_with_i2c(void)
{
    int ret = 0;
    unsigned char cmd[2] = {0x5a, 0xa5};

    ret = bl_i2c_transfer(CTP_SLAVE_ADDR, cmd, sizeof(cmd), I2C_WRITE);
    if (ret < 0)
    {
        bl_log_trace("bl_exit_update_with_i2c failed:send 5a a5 error___\n");
        goto error;
    }
    MDELAY(20);
error:

    return;
}
#endif

#ifdef INT_UPDATE_MODE
void bl_enter_update_with_int(void)
{
    bl_ts_set_intmode(0);
    bl_soft_reset_switch_int_wakemode();
    bl_ts_set_intup(0);
    MDELAY(50);
}

void bl_exit_update_with_int(void)
{
    bl_ts_set_intup(1);
    MDELAY(20);
    bl_ts_set_intmode(1);
}
#endif

int bl_get_chip_id(unsigned char *buf)
{

    unsigned char cmd[3];
    int ret = 0x00;

    bl_log_trace("bl_get_chip_id\n");

    cmd[0] = RW_REGISTER_CMD;
    cmd[1] = ~cmd[0];
    cmd[2] = CHIP_ID_REG;

    ret = bl_i2c_transfer(BL_FLASH_I2C_ADDR, cmd, 3, I2C_WRITE);
    if (ret < 0)
    {
        bl_log_trace("bl_get_chip_id:i2c write flash error___\n");
        goto GET_CHIP_ID_ERROR;
    }

    ret = bl_i2c_transfer(BL_FLASH_I2C_ADDR, buf, 1, I2C_READ);
    if (ret < 0)
    {
        bl_log_trace("bl_get_chip_id:i2c read flash error___\n");
        goto GET_CHIP_ID_ERROR;
    }

GET_CHIP_ID_ERROR:
    return ret;
}

int bl_get_fwArgPrj_id(unsigned char *buf)
{
    bl_log_trace("bl_get_fwArgPrj_id\n");
    return bl_read_fw(CTP_SLAVE_ADDR, BL_FWVER_PJ_ID_REG, buf, 3);
}

#ifdef BL_UPDATE_FIRMWARE_ENABLE
static int bl_get_protect_flag(void)
{
    unsigned char ret = 0;
    unsigned char protectFlag = 0x00;
    bl_log_trace("bl_get_protect_flag\n");
    ret = bl_read_fw(CTP_SLAVE_ADDR, BL_PROTECT_REG, &protectFlag, 1);
    if (ret < 0)
    {
        bl_log_trace("bl_get_protect_flag failed,ret = %x\n", ret);
        return 0;
    }
    if (protectFlag == 0x55)
    {
        bl_log_trace("bl_get_protect_flag:protectFlag = %x\n", protectFlag);
        return 1;
    }
    return 0;
}

static int bl_get_specific_argument(unsigned int *arguOffset, unsigned char *cobID, unsigned char *fw_data, unsigned int fw_size, unsigned char arguCount)
{
    unsigned char convertCobId[12] = {0x00};
    unsigned char i = 0;
    unsigned int cobArguAddr = fw_size - arguCount * BL_ARGUMENT_FLASH_SIZE;
    bl_log_trace("fw_size is %x\n", fw_size);
    bl_log_trace("arguCount is %d\n", arguCount);
    bl_log_trace("cobArguAddr is %x\n", cobArguAddr);

    for (i = 0; i < sizeof(convertCobId); i++)
    {
        if (i % 2)
        {
            convertCobId[i] = cobID[i / 2] & 0x0f;
        }
        else
        {
            convertCobId[i] = (cobID[i / 2] & 0xf0) >> 4;
        }
        bl_log_trace("before convert:convertCobId[%d] is %x\n", i, convertCobId[i]);
        if (convertCobId[i] < 10)
        {
            convertCobId[i] = '0' + convertCobId[i];
        }
        else
        {
            convertCobId[i] = 'a' + convertCobId[i] - 10;
        }
        bl_log_trace("after convert:convertCobId[%d] is %x\n", i, convertCobId[i]);
    }

    bl_log_trace("convertCobId is:\n");
    for (i = 0; i < 12; i++)
    {
        bl_log_trace("%x  ", convertCobId[i]);
    }
    bl_log_trace("\n");

    for (i = 0; i < arguCount; i++)
    {
        if (memcmp(convertCobId, fw_data + cobArguAddr + i * BL_ARGUMENT_FLASH_SIZE + BL_COB_ID_OFFSET, 12))
        {
            bl_log_trace("This argu is not the specific argu\n");
        }
        else
        {
            *arguOffset = cobArguAddr + i * BL_ARGUMENT_FLASH_SIZE;
            bl_log_trace("This argu is the specific argu, and arguOffset is %x\n", *arguOffset);
            break;
        }
    }

    if (i == arguCount)
    {
        *arguOffset = BL_ARGUMENT_BASE_OFFSET;
        return -1;
    }
    else
    {
        return 0;
    }
}

static unsigned char bl_get_argument_count(unsigned char *fw_data, unsigned int fw_size)
{
    unsigned char i = 0;
    unsigned addr = 0;
    addr = fw_size;
    bl_log_trace("addr is %x\n", addr);
    while (addr > (BL_ARGUMENT_BASE_OFFSET + BL_ARGUMENT_FLASH_SIZE))
    {
        addr = addr - BL_ARGUMENT_FLASH_SIZE;
        if (memcmp(fw_data + addr, ARGU_MARK, sizeof(ARGU_MARK) - 1))
        {
            bl_log_trace("arguMark found flow complete");
            break;
        }
        else
        {
            i++;
            bl_log_trace("arguMark founded\n");
        }
    }
    bl_log_trace("The argument count is %d\n", i);
    return i;
}

static unsigned int bl_get_cob_project_down_size_arguCnt(unsigned char *fw_data, unsigned int fw_size, unsigned char *arguCnt)
{
    unsigned int downSize = 0;

    *arguCnt = bl_get_argument_count(fw_data, fw_size);
    downSize = fw_size - (*arguCnt) * BL_ARGUMENT_FLASH_SIZE - FLASH_PAGE_SIZE;
    return downSize;
}

static unsigned char bl_is_cob_project(unsigned char *fw_data, int fw_size)
{
    unsigned char arguKey[4] = {0xaa, 0x55, 0x09, 0x09};
    unsigned char *pfw;

    pfw = fw_data + fw_size - 4;

    if (fw_size % FLASH_PAGE_SIZE)
    {
        return 0;
    }
    else
    {
        if (memcmp(arguKey, pfw, 4))
        {
            return 0;
        }
        else
        {
            return 1;
        }
    }
}

static int bl_get_cob_id(unsigned char *buf)
{
    bl_log_trace("bl_get_cob_id\n");
    return bl_read_fw(CTP_SLAVE_ADDR, COB_ID_REG, buf, 6);
}

int bl_get_fw_checksum(unsigned short *fw_checksum)
{
    unsigned char buf[3];
    unsigned char checksum_ready = 0;
    int retry = 5;
    int ret = 0x00;

    bl_log_trace("bl_get_fw_checksum\n");

    buf[0] = CHECKSUM_CAL_REG;
    buf[1] = CHECKSUM_CAL;
    ret = bl_i2c_transfer(CTP_SLAVE_ADDR, buf, 2, I2C_WRITE);
    if (ret < 0)
    {
        bl_log_trace("bl_get_fw_checksum:write checksum cmd error___\n");
        return -1;
    }
    MDELAY(FW_CHECKSUM_DELAY_TIME);

    ret = bl_read_fw(CTP_SLAVE_ADDR, CHECKSUM_REG, buf, 3);
    bl_log_trace("CHECKSUM_REG %x %x %x\n", buf[0], buf[1], buf[2]);
    if (ret < 0)
    {
        bl_log_trace("bl_get_fw_checksum:read checksum error___\n");
        return -1;
    }

    checksum_ready = buf[0];

    while ((retry--) && (checksum_ready != CHECKSUM_READY))
    {

        MDELAY(50);
        ret = bl_read_fw(CTP_SLAVE_ADDR, CHECKSUM_REG, buf, 3);
        if (ret < 0)
        {
            bl_log_trace("bl_get_fw_checksum:read checksum error___\n");
            return -1;
        }

        checksum_ready = buf[0];
    }

    if (checksum_ready != CHECKSUM_READY)
    {
        bl_log_trace("bl_get_fw_checksum:read checksum fail___\n");
        return -1;
    }
    *fw_checksum = (buf[1] << 8) + buf[2];

    return 0;
}

static void bl_get_fw_bin_checksum(unsigned char *fw_data, unsigned short *fw_bin_checksum, int fw_size, int specifyArgAddr)
{
    int i = 0;
    int temp_checksum = 0x0;

    for (i = 0; i < BL_ARGUMENT_BASE_OFFSET; i++)
    {
        temp_checksum += fw_data[i];
    }
    for (i = specifyArgAddr; i < specifyArgAddr + VERTIFY_START_OFFSET; i++)
    {
        temp_checksum += fw_data[i];
    }
    for (i = specifyArgAddr + VERTIFY_START_OFFSET; i < specifyArgAddr + VERTIFY_START_OFFSET + 4; i++)
    {
        temp_checksum += fw_data[i];
    }
    for (i = BL_ARGUMENT_BASE_OFFSET + VERTIFY_START_OFFSET + 4; i < fw_size; i++)
    {
        temp_checksum += fw_data[i];
    }

    for (i = fw_size; i < MAX_FLASH_SIZE; i++)
    {
        temp_checksum += 0xff;
    }

    *fw_bin_checksum = temp_checksum & 0xffff;
}

static int bl_erase_flash(void)
{
    unsigned char cmd[2];

    bl_log_trace("bl_erase_flash\n");

    cmd[0] = ERASE_ALL_MAIN_CMD;
    cmd[1] = ~cmd[0];

    return bl_i2c_transfer(BL_FLASH_I2C_ADDR, cmd, 0x02, I2C_WRITE);
}

static int bl_write_flash(unsigned char cmd, int flash_start_addr, unsigned char *buf, int len)
{
    unsigned char cmd_buf[6 + FLASH_WSIZE];
    unsigned short flash_end_addr;
    int ret;

    bl_log_trace("bl_write_flash\n");

    if (!len)
    {
        bl_log_trace("___write flash len is 0x00,return___\n");
        return -1;
    }

    flash_end_addr = flash_start_addr + len - 1;

    if (flash_end_addr >= MAX_FLASH_SIZE)
    {
        bl_log_trace("___write flash end addr is overflow,return___\n");
        return -1;
    }

    cmd_buf[0] = cmd;
    cmd_buf[1] = ~cmd;
    cmd_buf[2] = flash_start_addr >> 0x08;
    cmd_buf[3] = flash_start_addr & 0xff;
    cmd_buf[4] = flash_end_addr >> 0x08;
    cmd_buf[5] = flash_end_addr & 0xff;

    memcpy(&cmd_buf[6], buf, len);

    ret = bl_i2c_transfer(BL_FLASH_I2C_ADDR, cmd_buf, len + 6, I2C_WRITE);
    if (ret < 0)
    {
        bl_log_trace("i2c transfer error___\n");
        return -1;
    }

    return 0;
}

static int bl_read_flash(unsigned char cmd, int flash_start_addr, unsigned char *buf, int len)
{
    char ret = 0;
    unsigned char cmd_buf[6];
    unsigned short flash_end_addr;

    flash_end_addr = flash_start_addr + len - 1;
    cmd_buf[0] = cmd;
    cmd_buf[1] = ~cmd;
    cmd_buf[2] = flash_start_addr >> 0x08;
    cmd_buf[3] = flash_start_addr & 0xff;
    cmd_buf[4] = flash_end_addr >> 0x08;
    cmd_buf[5] = flash_end_addr & 0xff;
    ret = bl_i2c_transfer(BL_FLASH_I2C_ADDR, cmd_buf, 6, I2C_WRITE);
    if (ret < 0)
    {
        bl_log_trace("bl_read_flash:i2c transfer write error\n");
        return -1;
    }
    ret = bl_i2c_transfer(BL_FLASH_I2C_ADDR, buf, len, I2C_READ);
    if (ret < 0)
    {
        bl_log_trace("bl_read_flash:i2c transfer read error\n");
        return -1;
    }

    return 0;
}

static int bl_download_fw(unsigned char *pfwbin, int specificArgAddr, int fwsize)
{
    unsigned int i;
    unsigned short size, len;
    unsigned short addr;
    unsigned char verifyBuf[4] = {0xff, 0xff, 0x09, 0x09};
    bl_log_trace("bl_download_fw\n");
    if (bl_erase_flash())
    {
        bl_log_trace("___erase flash fail___\n");
        return -1;
    }

    MDELAY(50);

    //Write data before BL_ARGUMENT_BASE_OFFSET
    for (i = 0; i < BL_ARGUMENT_BASE_OFFSET;)
    {
        size = BL_ARGUMENT_BASE_OFFSET - i;
        if (size > FLASH_WSIZE)
        {
            len = FLASH_WSIZE;
        }
        else
        {
            len = size;
        }

        addr = i;

        if (bl_write_flash(WRITE_MAIN_CMD, addr, &pfwbin[i], len))
        {
            return -1;
        }
        i += len;
        MDELAY(5);
    }

    //Write the data from BL_ARGUMENT_BASE_OFFSET to VERTIFY_START_OFFSET
    for (i = BL_ARGUMENT_BASE_OFFSET; i < (VERTIFY_START_OFFSET + BL_ARGUMENT_BASE_OFFSET);)
    {
        size = VERTIFY_START_OFFSET + BL_ARGUMENT_BASE_OFFSET - i;
        if (size > FLASH_WSIZE)
        {
            len = FLASH_WSIZE;
        }
        else
        {
            len = size;
        }

        addr = i;

        if (bl_write_flash(WRITE_MAIN_CMD, addr, &pfwbin[i + specificArgAddr - BL_ARGUMENT_BASE_OFFSET], len))
        {
            return -1;
        }
        i += len;
        MDELAY(5);
    }

    //Write the four bytes verifyBuf from VERTIFY_START_OFFSET
    for (i = (VERTIFY_START_OFFSET + BL_ARGUMENT_BASE_OFFSET); i < (VERTIFY_START_OFFSET + BL_ARGUMENT_BASE_OFFSET + sizeof(verifyBuf));)
    {
        size = VERTIFY_START_OFFSET + BL_ARGUMENT_BASE_OFFSET + sizeof(verifyBuf) - i;
        if (size > FLASH_WSIZE)
        {
            len = FLASH_WSIZE;
        }
        else
        {
            len = size;
        }

        addr = i;

        if (bl_write_flash(WRITE_MAIN_CMD, addr, &verifyBuf[i - VERTIFY_START_OFFSET - BL_ARGUMENT_BASE_OFFSET], len))
        {
            return -1;
        }
        i += len;
        MDELAY(5);
    }

    //Write data after verityBuf from VERTIFY_START_OFFSET + 4
    for (i = (BL_ARGUMENT_BASE_OFFSET + VERTIFY_START_OFFSET + 4); i < fwsize;)
    {
        size = fwsize - i;
        if (size > FLASH_WSIZE)
        {
            len = FLASH_WSIZE;
        }
        else
        {
            len = size;
        }

        addr = i;

        if (bl_write_flash(WRITE_MAIN_CMD, addr, &pfwbin[i], len))
        {
            return -1;
        }
        i += len;
        MDELAY(5);
    }

    return 0;
}


static int bl_read_flash_vertify(void)
{
    unsigned char cnt = 0;
    int ret = 0;
    unsigned char vertify[2] = {0xAA, 0x55};
    unsigned char vertify1[2] = {0};
    SET_WAKEUP_LOW;
    while (cnt < 3)
    {
        cnt++;
        ret = bl_read_flash(READ_MAIN_CMD, BL_ARGUMENT_BASE_OFFSET + VERTIFY_START_OFFSET, vertify1, 2);
        if (ret < 0)
        {
            bl_log_trace("bl_write_flash_vertify: read fail\n");
            continue;
        }

        if (memcmp(vertify, vertify1, 2) == 0)
        {
            ret = 0;
            break;
        }
        else
        {
            ret = -1;
        }
    }
    SET_WAKEUP_HIGH;
    MDELAY(20);
    return ret;
}

static int bl_write_flash_vertify(void)
{
    unsigned char cnt = 0;
    int ret = 0;
    unsigned char vertify[2] = {0xAA, 0x55};
    unsigned char vertify1[2] = {0};
    SET_WAKEUP_LOW;
    while (cnt < 3)
    {
        cnt++;
        ret = bl_write_flash(WRITE_MAIN_CMD, BL_ARGUMENT_BASE_OFFSET + VERTIFY_START_OFFSET, vertify, 2);
        if (ret < 0)
        {
            bl_log_trace("bl_write_flash_vertify: write fail\n");
            continue;
        }

        MDELAY(10);

        ret = bl_read_flash(READ_MAIN_CMD, BL_ARGUMENT_BASE_OFFSET + VERTIFY_START_OFFSET, vertify1, 2);
        if (ret < 0)
        {
            bl_log_trace("bl_write_flash_vertify: read fail\n");
            continue;
        }

        if (memcmp(vertify, vertify1, 2) == 0)
        {
            ret = 0;
            break;
        }
        else
        {
            bl_log_trace("bl_write_flash_vertify fail %x %x != %x %x\n", vertify[0], vertify[1], vertify1[0], vertify1[1]);
            ret = -1;
        }
    }
    SET_WAKEUP_HIGH;
    MDELAY(20);
    return ret;
}

static int bl_update_flash(unsigned char update_type, unsigned char *pfwbin, int fwsize, int specificArgAddr)
{
    int retry = 0;
    int ret = 0;
    unsigned short fw_checksum = 0x0;
    unsigned short fw_bin_checksum = 0x0;
    unsigned short lk_checksum = 0x0;
    unsigned short lk_bin_checksum = 0x0;
    retry = 3;
    while (retry--)
    {
        SET_WAKEUP_LOW;

        ret = bl_download_fw(pfwbin, specificArgAddr, fwsize);

        if (ret < 0)
        {
            bl_log_trace("btl fw update start bl_download_fw error retry=%d\n", retry);
            continue;
        }

        MDELAY(50);

        SET_WAKEUP_HIGH;

        MDELAY(200);
#if defined(RESET_PIN_WAKEUP)
        bl_ts_reset_wakeup();
#endif

        bl_get_fw_bin_checksum(pfwbin, &fw_bin_checksum, fwsize, specificArgAddr);
        ret = bl_get_fw_checksum(&fw_checksum);
        fw_checksum -= 0xff;
        bl_log_trace("btl fw update end,fw checksum = 0x%x,fw_bin_checksum =0x%x\n", fw_checksum, fw_bin_checksum);


        if ((ret < 0) || ((update_type == FW_ARG_UPDATE) && (fw_checksum != fw_bin_checksum)))
        {
            bl_log_trace("btl fw update start bl_download_fw bl_get_fw_checksum error");
            continue;
        }

        if ((update_type == FW_ARG_UPDATE) && (fw_checksum == fw_bin_checksum))
        {
            ret = bl_write_flash_vertify();
            if (ret < 0)
                continue;
        }
        break;
    }

    if (retry < 0)
    {
        bl_log_trace("btl fw update error\n");
        return -1;
    }

    bl_log_trace("btl fw update success___\n");

    return 0;
}

static unsigned char choose_update_type_for_self_ctp(unsigned char isBlank, unsigned char *fw_data, unsigned char fwVer, unsigned char arguVer, unsigned short fwChecksum, unsigned short fwBinChecksum, int specifyArgAddr)
{
    unsigned char update_type = NONE_UPDATE;
    if (isBlank)
    {
        update_type = FW_ARG_UPDATE;
        bl_log_trace("Update case 0:FW_ARG_UPDATE\n");
    }
    else
    {
        if (((fwVer != fw_data[specifyArgAddr + BL_FWVER_MAIN_OFFSET])
                || (arguVer != fw_data[specifyArgAddr + BL_FWVER_ARGU_OFFSET])
                || (fwChecksum != fwBinChecksum)) && (!bl_get_protect_flag()))
        {
            update_type = FW_ARG_UPDATE;
            bl_log_trace("Update case 1:FW_ARG_UPDATE\n");
        }
        else
        {
            update_type = NONE_UPDATE;
            bl_log_trace("Update case 4:NONE_UPDATE\n");
        }
    }
    return update_type;
}

static int bl_update_fw_for_self_ctp(unsigned char *fw_data, int fw_size)
{
    unsigned char fwArgPrjID[3];    //firmware version/argument version/project identification
    unsigned char chip_id = 0x00;   //The IC identification
    int ret = 0x00;
    unsigned char isBlank = 0x0;    //Indicate the IC have any firmware
    unsigned short fw_checksum = 0x0;  //The checksum for firmware in IC
    unsigned short fw_bin_checksum = 0x0;  //The checksum for firmware in file
    unsigned char update_type = NONE_UPDATE;
    unsigned int downSize = 0x0;       //The available size of firmware data in file
    unsigned char cobID[7] = {0};           //The identification for COB project
    unsigned int specificArguAddr = BL_ARGUMENT_BASE_OFFSET;   //The specific argument base address in firmware date with cobID
    unsigned char arguCount = 0x0;      //The argument count for COB firmware
    unsigned char IsCobPrj = 0;         //Judge the project type depend firmware file
    bl_log_trace("bl_update_fw_for_self_ctp start\n");

//Check chipID
#ifdef BTL_CHECK_CHIPID
    SET_WAKEUP_LOW;
    MDELAY(50);
    ret = bl_get_chip_id(&chip_id);
    SET_WAKEUP_HIGH;
    if (ret < 0 || chip_id != BTL_FLASH_ID)
    {
        bl_log_trace("bl_update_fw_for_self_ctp:chip_id = %d", chip_id);
        return -1;
    }
    bl_log_trace("bl_update_fw_for_self_ctp:chip_id = %x", chip_id);
#endif

//Step 1:Obtain project type
    IsCobPrj = bl_is_cob_project(fw_data, fw_size);
    bl_log_trace("bl_update_fw_for_self_ctp:IsCobPrj = %x", IsCobPrj);

//Step 2:Obtain IC version number
    MDELAY(5);
    ret = bl_get_fwArgPrj_id(fwArgPrjID);
    if ((ret < 0)
            || ((ret == 0) && (fwArgPrjID[0] == 0))
            || ((ret == 0) && (fwArgPrjID[0] == 0xff))
            || ((ret == 0) && (fwArgPrjID[0] == BL_FWVER_PJ_ID_REG))
            || (bl_read_flash_vertify() < 0))
    {
        isBlank = 1;
        bl_log_trace("bl_update_fw_for_self_ctp:This is blank IC ret = %x fwArgPrjID[0]=%x\n", ret, fwArgPrjID[0]);
    }
    else
    {
        isBlank = 0;
        bl_log_trace("bl_update_fw_for_self_ctp:ret=%x fwID=%x argID=%x prjID=%\n", ret, fwArgPrjID[0], fwArgPrjID[1], fwArgPrjID[2]);
    }
    bl_log_trace("bl_update_fw_for_self_ctp:isBlank = %x\n", isBlank);
    //isBlank = 1;//Force update firmware
//Step 3:Specify download size
    if (IsCobPrj)
    {
        downSize = bl_get_cob_project_down_size_arguCnt(fw_data, fw_size, &arguCount);
        bl_log_trace("bl_update_fw_for_self_ctp:downSize = %x,arguCount = %x\n", downSize, arguCount);
    }
    else
    {
        downSize = fw_size;
        bl_log_trace("bl_update_fw_for_self_ctp:downSize = %x\n", downSize);
    }

UPDATE_SECOND_FOR_COB:
//Step 4:Update the fwArgPrjID
    if (!isBlank)
    {
        bl_get_fwArgPrj_id(fwArgPrjID);
    }

//Step 5:Specify the argument data for cob project
    if (IsCobPrj && !isBlank)
    {
        MDELAY(50);
        ret = bl_get_cob_id(cobID);
        if (ret < 0)
        {
            bl_log_trace("bl_update_fw_for_self_ctp:bl_get_cob_id error\n");
            ret = -1;
            goto UPDATE_ERROR;
        }
        else
        {
            bl_log_trace("bl_update_fw_for_self_ctp:cobID = %x %x %x %x %x %x %x\n", cobID[0], cobID[1], cobID[2], cobID[3], cobID[4], cobID[5], cobID[6]);
        }
        ret = bl_get_specific_argument(&specificArguAddr, cobID, fw_data, fw_size, arguCount);
        if (ret < 0)
        {
            bl_log_trace("Can't found argument for CTP module,use default argu:\n");
        }
        bl_log_trace("bl_update_fw_for_self_ctp:specificArguAddr = %x\n", specificArguAddr);
    }

    bl_log_trace("fw_data[] = %x  fw_data[] = %x", fw_data[BL_ARGUMENT_BASE_OFFSET + VERTIFY_START_OFFSET], fw_data[BL_ARGUMENT_BASE_OFFSET + VERTIFY_END_OFFSET]);

//Step 6:Obtain IC firmware checksum when the version number is same between IC and host firmware
    bl_log_trace("isBlank = %d  ver1 = %d ver2 = %d binVer1 = %d binVer2 = %d specificAddr = %x", isBlank, fwArgPrjID[0], fwArgPrjID[1], fw_data[specificArguAddr + BL_FWVER_MAIN_OFFSET], fw_data[specificArguAddr + BL_FWVER_ARGU_OFFSET], specificArguAddr);
    if (!isBlank)
    {
        bl_get_fw_bin_checksum(fw_data, &fw_bin_checksum, downSize, specificArguAddr);
        ret = bl_get_fw_checksum(&fw_checksum);
        if ((ret < 0) || (fw_checksum != fw_bin_checksum))
        {
            bl_log_trace("bl_update_fw_for_self_ctp:Read checksum fail fw_checksum = %x\n", fw_checksum);
            fw_checksum = 0x00;
        }
        bl_log_trace("bl_update_fw_for_self_ctp:fw_checksum = 0x%x,fw_bin_checksum = 0x%x___\n", fw_checksum, fw_bin_checksum);
    }

//Step 7:Select update fw+arg or only update arg
    update_type = choose_update_type_for_self_ctp(isBlank, fw_data, fwArgPrjID[0], fwArgPrjID[1], fw_checksum, fw_bin_checksum, specificArguAddr);

//Step 8:Start Update depend condition
    if (update_type != NONE_UPDATE)
    {
        ret = bl_update_flash(update_type, fw_data, downSize, specificArguAddr);
        if (ret < 0)
        {
            bl_log_trace("bl_update_fw_for_self_ctp:bl_update_flash failed\n");
            goto UPDATE_ERROR;
        }
    }

//Step 9:Execute second update flow when project firmware is cob and last update_type is FW_ARG_UPDATE
    if ((ret == 0) && (IsCobPrj) && (isBlank))
    {
        isBlank = 0;
        bl_log_trace("bl_update_fw_for_self_ctp:bl_update_flash for COB project need second update with blank IC:isBlank = %d\n", isBlank);
        goto UPDATE_SECOND_FOR_COB;
    }
    bl_log_trace("bl_update_fw_for_self_ctp exit\n");

UPDATE_ERROR:
    return ret;
}

static int bl_update_fw(unsigned char *pFwData, unsigned int fwLen)
{
    int ret = 0;
    ret = bl_update_fw_for_self_ctp(pFwData, fwLen);
    return ret;
}
#ifdef BL_AUTO_UPDATE_FARMWARE
int bl_auto_update_fw(void)
{
    int ret = 0;

    unsigned int fwLen = sizeof(fwbin);

    bl_log_trace("bl_auto_update_fw:fwLen = %x\n", fwLen);
    ret = bl_update_fw((unsigned char *)fwbin, fwLen);


    if (ret < 0)
    {
        bl_log_trace("bl_auto_update_fw: bl_update_fw fail\n");
    }
    else
    {
        bl_log_trace("bl_auto_update_fw: bl_update_fw success\n");
    }

    return ret;
}
#endif
#if defined(__TP_SDK_UPDATE_SUPPORT__)
#include "Os_mem.h"
#define TP_SDK_SIZE    27*1024
bool ota_update_tp_sdk_flag = false;
u8 cur_data_count = 0;
//uint8_t custom_fwbin[25*1024]={0};
uint8_t *custom_fwbin = NULL; //[25*1024]={0};
u16 ota_fwbin_len = 0;
void custom_update_tp_sdk_init(void)
{
    cur_data_count = 0;
    ota_fwbin_len = 0;
    ota_update_tp_sdk_flag = false;
    //memset(custom_fwbin,0,25*1024);
    if (custom_fwbin == NULL)
        custom_fwbin = (u8 *)os_malloc(TP_SDK_SIZE);            //根据协议指定的数据包大小创建存储存放数据

}
void ota_update_tp_sdk(void)
{
    int ret = 0;
    bl_ts_set_intmode(0);
    ret = bl_update_fw((unsigned char *)custom_fwbin, ota_fwbin_len);
    if (ret < 0)
    {
        ota_update_tp_sdk_flag = false;
        bl_log_trace("bl_auto_update_fw: bl_update_fw fail\n");
    }
    else
    {
        ota_update_tp_sdk_flag = true;
        bl_log_trace("bl_auto_update_fw: bl_update_fw success\n");
    }
    if (custom_fwbin != NULL)
        os_free(custom_fwbin);

    ctp_delay_ms(5);
    bl_ts_set_intmode(1);
}
void custom_update_tp_sdk(u8 *data, u16 len, u16 sdk_len)
{
    if (custom_fwbin == NULL)
        return;

    memcpy(custom_fwbin + cur_data_count * 128, data, 128);
    if ((cur_data_count * 128 + len) >= sdk_len)
    {
        char  ret = 0;
        unsigned char chip_id[4] = {0};
        ota_fwbin_len = sdk_len;
#ifdef  RESET_PIN_WAKEUP
        bl_ts_reset_wakeup();
#endif
#ifdef INT_PIN_WAKEUP
        bl_ts_int_wakeup();
#endif

        //I2C初始化
        ctp_i2c_init();

        MDELAY(20);

        bl_ts_set_intmode(0);

        SET_WAKEUP_LOW;
        MDELAY(50);
        ret = bl_get_chip_id(chip_id);
        SET_WAKEUP_HIGH;

        APP_LOG("ctp_bl_ts_init id=0x%x\r\n", chip_id[0]);

        //芯片固件更新,开机时先不更新固件
#ifdef  BL_AUTO_UPDATE_FARMWARE

        if (!detect_autotest_mode()) //TP_INT拉低会导致固件升级死机，所以在进入工厂模式时，不要让其更新固件。
        {
            ota_update_tp_sdk();
        }



        //拉复位
        CTP_SET_RESET_PIN_OUTPUT;
        CTP_SET_RESET_PIN_LOW;
        ctp_delay_ms(50);
        CTP_SET_RESET_PIN_HIGH;
        ctp_delay_ms(50);
#endif
    }
    else
        cur_data_count++;
}
#endif
#endif
#endif
