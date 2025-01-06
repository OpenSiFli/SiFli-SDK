#include <rtthread.h>
#include <string.h>
#include "HYN_CST1_1.h"

//#define  HYN_UPDATE_FIRMWARE_FORCE  /* 强制升级,测试用 */

#define TOUCH_CHIP_ID_CST918            (0x396242E)
#define TOUCH_SLAVE_ADDRESS             (0x1A)

uint32_t g_cst9xx_ic_version            = 0;
uint32_t g_cst9xx_ic_checksum           = 0;
uint32_t g_cst9xx_ic_checkcode          = 0;
uint32_t g_cst9xx_ic_project_id         = 0;
uint32_t g_cst9xx_ic_type               = 0;

#define cst9xx_BIN_SIZE     (24*1024 + 24)

static int cst9xx_firmware_info(void);
static int cst9xx_update_judge(unsigned char *pdata, int strict);
static int cst9xx_update_firmware(void);
static void cst9xx_reset_ic(unsigned int ms);
static int cst9xx_into_program_mode(void);
static int cst9xx_erase_program_area(void);
static int cst9xx_write_program_data(void);
static int cst9xx_check_checksum(void);
static int cst9xx_check_checksum(void);
static int cst9xx_exit_program_mode(void);

#if  0  /* 外部提供的读写接口 */
static uint32_t cst918_i2c_write(uint16_t reg, uint8_t *data, uint16_t len)
{
    if (len >= 30 || (len > 0 && data == NULL)) return USER_ERROR_INVALID_DATA;

    uint8_t I2C_WriteBuf[32] = {0};
    I2C_WriteBuf[0] = reg >> 8;
    I2C_WriteBuf[1] = reg;

    for (int i = 0; i < len; i++)
    {
        I2C_WriteBuf[i + 2] = data[i];
    }
    return dev_i2c_rtl876x_send(TOUCH_I2C_INDEX, TOUCH_SLAVE_ADDRESS, I2C_WriteBuf, len + 2);
}

static uint32_t cst918_i2c_read(uint16_t reg, uint8_t *p_data, uint8_t len)
{
    uint8_t buf[2] = {0};
    buf[0] = reg >> 8;
    buf[1] = reg;
    return dev_i2c_rtl876x_receive(TOUCH_I2C_INDEX, TOUCH_SLAVE_ADDRESS, buf, 2, p_data, len);
}
#else
extern rt_err_t cst918_i2c_write(uint16_t reg, uint8_t *data, uint16_t len);
extern rt_err_t cst918_i2c_read(const uint16_t reg, uint8_t *p_data, uint8_t len);
#endif

int cst9xx_boot_update_fw(void)
{
    int ret = 0;
    int retry = 0;
    int flag = 0;

    rt_kprintf("cst9xx_boot_update_fw ..");
    while (retry++ < 3)
    {
        ret = cst9xx_firmware_info();
        if (ret == 0)
        {
            flag = 1;
            break;
        }
    }

    if (flag == 1)
    {
        ret = cst9xx_update_judge((uint8_t *)CST918_FRMEWARE_DATA, 1);
        if (ret < 0)
        {
            rt_kprintf(" cst9xx[cst9xx] no need to update firmware.\r\n");
            return 0;
        }
    }

    ret = cst9xx_update_firmware();
    if (ret < 0)
    {
        rt_kprintf(" cst9xx [cst9xx] update firmware failed.\r\n");
        return -1;
    }
    rt_thread_mdelay(50);

    ret = cst9xx_firmware_info();
    if (ret < 0)
    {
        rt_kprintf(" cst9xx [cst9xx] after update read version and checksum fail.\r\n");
        return -1;
    }

    return 0;
}

static int cst9xx_firmware_info(void)
{
    int ret;
    unsigned char buf[20] = {0};
    //unsigned short ic_type, project_id;

    rt_kprintf("cst9xx_firmware_info . \r\r\r\n");

    //buf[0] = 0xD1;
    //buf[1] = 0x01;
    ret = cst918_i2c_write(0xD101, NULL, 0);
    rt_thread_mdelay(40);

    //buf[0] = 0xD1;
    //buf[1] = 0xFC;
    cst918_i2c_read(0xD1FC, buf, 4);

    //0xCACA0000
    g_cst9xx_ic_checkcode = buf[3];
    g_cst9xx_ic_checkcode <<= 8;
    g_cst9xx_ic_checkcode |= buf[2];
    g_cst9xx_ic_checkcode <<= 8;
    g_cst9xx_ic_checkcode |= buf[1];
    g_cst9xx_ic_checkcode <<= 8;
    g_cst9xx_ic_checkcode |= buf[0];

    rt_kprintf("linc cst9xx [cst9xx] the chip g_cst9xx_ic_checkcode:0x%x.\r\r\n", g_cst9xx_ic_checkcode);

    rt_thread_mdelay(2);

    //buf[0] = 0xD2;
    //buf[1] = 0x04;
    cst918_i2c_read(0xD204, buf, 4);
    g_cst9xx_ic_type = buf[3];
    g_cst9xx_ic_type <<= 8;
    g_cst9xx_ic_type |= buf[2];


    g_cst9xx_ic_project_id = buf[1];
    g_cst9xx_ic_project_id <<= 8;
    g_cst9xx_ic_project_id |= buf[0];

    rt_kprintf("linc cst9xx [cst9xx] the chip ic g_cst9xx_ic_type :0x%x, g_cst9xx_ic_project_id:0x%x\r\r\n",
               g_cst9xx_ic_type, g_cst9xx_ic_project_id);

    rt_thread_mdelay(2);

    //buf[0] = 0xD2;
    //buf[1] = 0x08;
    cst918_i2c_read(0xD208, buf, 8);

    g_cst9xx_ic_version = buf[3];
    g_cst9xx_ic_version <<= 8;
    g_cst9xx_ic_version |= buf[2];
    g_cst9xx_ic_version <<= 8;
    g_cst9xx_ic_version |= buf[1];
    g_cst9xx_ic_version <<= 8;
    g_cst9xx_ic_version |= buf[0];

    g_cst9xx_ic_checksum = buf[7];
    g_cst9xx_ic_checksum <<= 8;
    g_cst9xx_ic_checksum |= buf[6];
    g_cst9xx_ic_checksum <<= 8;
    g_cst9xx_ic_checksum |= buf[5];
    g_cst9xx_ic_checksum <<= 8;
    g_cst9xx_ic_checksum |= buf[4];

    rt_kprintf(" cst9xx [cst9xx] the chip ic version:0x%x, checksum:0x%x\r\r\n",
               g_cst9xx_ic_version, g_cst9xx_ic_checksum);

    if (g_cst9xx_ic_version == 0xA5A5A5A5)
    {
        rt_kprintf(" cst9xx [cst9xx] the chip ic don't have firmware. \r\n");
        return -1;
    }
    if ((g_cst9xx_ic_checkcode & 0xffff0000) != 0xCACA0000)
    {
        rt_kprintf("linc cst9xx [cst9xx] cst9xx_firmware_info read error .\r\r\n");
        return -1;
    }

    //buf[0] = 0xD1;
    //buf[1] = 0x09;
    cst918_i2c_write(0Xd109, NULL, 0);
    rt_thread_mdelay(5);

    return 0;
}

static int cst9xx_update_judge(unsigned char *pdata, int strict)
{
    unsigned short ic_type, project_id;
    unsigned int fw_checksum, fw_version;
    const unsigned int *p;
    //int i;
    unsigned char *pBuf;

#if 0
    fw_checksum = 0x55;
    p = (unsigned int *)get_mapping_address(pdata);
    for (i = 0; i < (cst9xx_BIN_SIZE - 4); i += 4)
    {
        fw_checksum += (*p);
        p++;
    }
    if (fw_checksum != (*p))
    {
        rt_kprintf(" cst9xx[cst9xx]calculated checksum error:0x%x not equal 0x%x.\r\n", fw_checksum, *p);
        return -1;  //bad fw, so do not update
    }
#endif

    pBuf = &pdata[cst9xx_BIN_SIZE - 16];

    project_id = pBuf[1];
    project_id <<= 8;
    project_id |= pBuf[0];

    ic_type = pBuf[3];
    ic_type <<= 8;
    ic_type |= pBuf[2];

    fw_version = pBuf[7];
    fw_version <<= 8;
    fw_version |= pBuf[6];
    fw_version <<= 8;
    fw_version |= pBuf[5];
    fw_version <<= 8;
    fw_version |= pBuf[4];

    fw_checksum = pBuf[11];
    fw_checksum <<= 8;
    fw_checksum |= pBuf[10];
    fw_checksum <<= 8;
    fw_checksum |= pBuf[9];
    fw_checksum <<= 8;
    fw_checksum |= pBuf[8];

    rt_kprintf(" cst9xx[cst9xx]the updating firmware:project_id:0x%04x,ic type:0x%04x,version:0x%x,checksum:0x%x\r\n",
               project_id, ic_type, fw_version, fw_checksum);

#ifdef HYN_UPDATE_FIRMWARE_FORCE
    rt_kprintf("[cst9xx]update firmware FORCE.\r\n");
    return 0;
#endif

    if (strict > 0)
    {

        if (g_cst9xx_ic_checksum != fw_checksum)
        {

            if (g_cst9xx_ic_version > fw_version)
            {
                rt_kprintf("[cst9xx]fw version(0x%x), ic version(0x%x).\r\n", fw_version, g_cst9xx_ic_version);
                return -1;
            }
        }
        else
        {
            rt_kprintf("[cst9xx]fw checksum(0x%x), ic checksum(0x%x).\r\n", fw_checksum, g_cst9xx_ic_checksum);
            return -1;
        }
    }

    return 0;
}

static int cst9xx_update_firmware(void)
{
    int ret;
    int retry = 0;

    rt_kprintf(" cst9xx----------upgrade cst9xx begain------------\r\r\n");
    rt_thread_mdelay(20);

START_FLOW:
    cst9xx_reset_ic(7 + retry);
    ret = cst9xx_into_program_mode();
    if (ret < 0)
    {
        rt_kprintf(" cst9xx[cst9xx]into program mode failed.\r\r\n");
        goto err_out;
    }

    ret = cst9xx_erase_program_area();
    if (ret)
    {
        rt_kprintf(" cst9xx[cst9xx]erase main area failed.\r\r\n");
        goto err_out;
    }

    ret = cst9xx_write_program_data();
    if (ret < 0)
    {
        rt_kprintf(" cst9xx[cst9xx]write program data into cstxxx failed.\r\r\n");
        goto err_out;
    }

    ret = cst9xx_check_checksum();
    if (ret < 0)
    {
        rt_kprintf(" cst9xx[cst9xx] after write program cst9xx_check_checksum failed.\r\r\n");
        goto err_out;
    }

    ret = cst9xx_exit_program_mode();
    if (ret < 0)
    {
        rt_kprintf(" cst9xx[cst9xx]exit program mode failed.\r\r\n");
        goto err_out;
    }

    cst9xx_reset_ic(20);

    rt_kprintf(" cst9xx hyn----------cst9xx_update_firmware  end------------\r\r\n");

    return 0;

err_out:
    if (retry < 30)
    {
        retry++;
        rt_thread_mdelay(20);
        goto START_FLOW;
    }
    else
    {
        return -1;
    }
}

static void cst9xx_reset_ic(unsigned int ms)
{
    cst918_i2c_write(0xD10E, NULL, 0);

    rt_thread_mdelay(ms);
}

static int cst9xx_into_program_mode(void)
{
    //int ret;
    //unsigned char buf[4];
    uint8_t data = 0;
    //buf[0] = 0xA0;
    //buf[1] = 0x01;
    //buf[2] = 0xAA;    //set cmd to enter program mode
    uint8_t tmp = 0xAA;
    cst918_i2c_write(0XA001, &tmp, 1);
    rt_thread_mdelay(2);

    //buf[0] = 0xA0;
    //buf[1] = 0x02;    //check whether into program mode
    cst918_i2c_read(0xA002, &data, 1);
    if (data != 0x55) return -1;

    return 0;
}

static int cst9xx_erase_program_area(void)
{
    //int ret;
    //unsigned char buf[3];
    uint8_t data;
    //buf[0] = 0xA0;
    //buf[1] = 0x02;
    //buf[2] = 0x00;        //set cmd to erase main area
    uint8_t tmp = 0X00;
    cst918_i2c_write(0xA002, &tmp, 1);

    rt_thread_mdelay(5);

    //buf[0] = 0xA0;
    //buf[1] = 0x03;
    cst918_i2c_read(0xA003, &data, 1);
    if (data != 0x55) return -1;

    return 0;
}

static int cst9xx_write_program_data(void)
{
    int i, ret;
    //unsigned char *i2c_buf;
    unsigned short eep_addr;
    int total_kbyte;

    unsigned char temp_buf[8];
    unsigned short iic_addr;
    int  j;
    uint8_t i2c_buf[1024 + 2] = {0};
    uint8_t data = 0;
    uint16_t addr = 0;
    uint8_t tmp = 0;
//      i2c_buf = kmalloc(sizeof(unsigned char)*(1024 + 2), GFP_KERNEL);
//      if (i2c_buf == NULL)
    //      return -1;

    //make sure fwbin len is N*1K
    //total_kbyte = len / 1024;
    total_kbyte = 24;
    for (i = 0; i < total_kbyte; i++)
    {
        rt_kprintf("tp update send data: %d", i);

        //i2c_buf[0] = 0xA0;
        //i2c_buf[1] = 0x14;
        uint8_t index[2] = {0};
        eep_addr = i << 10;     //i * 1024
        index[0] = eep_addr;
        index[1] = eep_addr >> 8;
        cst918_i2c_write(0xA014, index, sizeof(index));

        //i2c_buf[0] = 0xA0;
        //i2c_buf[0] = 0x18;
        memcpy(&i2c_buf, CST918_FRMEWARE_DATA + eep_addr, 1024);
        //dev_i2c_rtl876x_send(TOUCH_I2C_INDEX, TOUCH_SLAVE_ADDRESS, i2c_buf, sizeof(i2c_buf));

        /* wdg_feed(); */
        for (j = 0; j < 256; j++)
        {
            iic_addr = (j << 2);
            temp_buf[0] = (iic_addr + 0xA018) >> 8;
            temp_buf[1] = (iic_addr + 0xA018) & 0xFF;
            addr = (temp_buf[0] << 8) | temp_buf[1] ;
//                      data = i2c_buf[iic_addr+0];
//                      cst918_i2c_write( addr, &data, 1);
//                      data = i2c_buf[iic_addr+1];
//                      cst918_i2c_write( addr, &data, 1);
//                      data = i2c_buf[iic_addr+2];
//                      cst918_i2c_write( addr, &data, 1);
//                      data = i2c_buf[iic_addr+3];
//                      cst918_i2c_write( addr, &data, 1);

            cst918_i2c_write(addr, &i2c_buf[iic_addr], 4);
        }

        //i2c_buf[0] = 0xA0;
        //i2c_buf[1] = 0x04;
        //i2c_buf[2] = 0xEE;
        tmp = 0xEE;
        cst918_i2c_write(0xA004, &tmp, 1);
        rt_thread_mdelay(60);

        //i2c_buf[0] = 0xA0;
        //i2c_buf[1] = 0x05;
        cst918_i2c_read(0xA005, temp_buf, 1);
        if (temp_buf[0] != 0x55)
        {
            goto error_out;
        }
    }

    //i2c_buf[0] = 0xA0;
    //i2c_buf[1] = 0x03;
    //i2c_buf[2] = 0x00;
    tmp = 0x00;
    cst918_i2c_write(0xA003, &tmp, 1);

    rt_thread_mdelay(8);

    return 0;

error_out:
    return -1;
}

static int cst9xx_check_checksum(void)
{
    //int ret;
    int i;
    unsigned int  checksum = {0};
    unsigned int  bin_checksum = {0};
    unsigned char buf[4] = {0};
    const unsigned char *pData = NULL;

    for (i = 0; i < 5; i++)
    {
        //buf[0] = 0xA0;
        //buf[1] = 0x00;
        cst918_i2c_read(0xA000, buf, 1);

        if (buf[0] != 0)
            break;
        else
            rt_thread_mdelay(2);
    }
    rt_thread_mdelay(2);

    if (buf[0] == 0x01)
    {
        //buf[0] = 0xA0;
        //buf[1] = 0x08;
        cst918_i2c_read(0xA008, buf, 4);

        // read chip checksum
        checksum = buf[0] + (buf[1] << 8) + (buf[2] << 16) + (buf[3] << 24);

        pData = (unsigned char *)CST918_FRMEWARE_DATA + 24 * 1024 + 16; //7*1024 +512
        bin_checksum = pData[0] + (pData[1] << 8) + (pData[2] << 16) + (pData[3] << 24);

        rt_kprintf("  hyn the updated ic checksum is :0x%x. the updating firmware checksum is:0x%x------\r\r\n", checksum, bin_checksum);

        if (checksum != bin_checksum)
        {
            rt_kprintf(" cst9xx hyn check sum error.\r\r\n");
            return -1;
        }
    }
    else
    {
        rt_kprintf(" cst9xx hyn No checksum.\r\r\n");
        while (1)
        {
            /* wdg_feed(); */
            rt_thread_mdelay(200);
        }
        return -1;
    }
    return 0;
}

static int cst9xx_exit_program_mode(void)
{
    //int ret;
    //unsigned char buf[3];

    //buf[0] = 0xA0;
    //buf[1] = 0x06;
    //buf[2] = 0xEE;
    uint8_t tmp = 0xEE;
    cst918_i2c_write(0xA006, &tmp, 1);

    rt_thread_mdelay(10);   //wait for restart
    return 0;
}
