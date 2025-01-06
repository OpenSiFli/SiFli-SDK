
#ifndef _CHS5816_H_
#define _CHS5816_H_
//chsc5816 
#define MAX_IO_BUFFER_LEN 32

/*ctp work staus*/
#define CTP_POINTING_WORK            0x00000000
#define CTP_READY_UPGRADE            (1 << 1)
#define CTP_UPGRAD_RUNING            (1 << 2)
#define CTP_SUSPEND_GATE             (1 << 16)
#define CTP_GUESTURE_GATE            (1 << 17)
#define CTP_PROXIMITY_GATE           (1 << 18)
#define CTP_GLOVE_GATE               (1 << 19)
#define CTP_ORIENTATION_GATE         (1 << 20)

struct chsc_updfile_header {
    uint32_t sig;
    uint32_t resv;
    uint32_t n_cfg;
    uint32_t n_match;
    uint32_t len_cfg;
    uint32_t len_boot;
};

typedef struct _img_header_t
{
    uint16_t fw_ver;
    uint16_t resv;
    uint32_t sig;
    uint32_t vid_pid;
    uint16_t raw_offet;
    uint16_t dif_offet;
}img_header_t;

typedef struct sm_touch_dev
{
    uint32_t ctp_run_status;
    uint32_t vid_pid;    //0xVID_PID_CFGVER
    uint16_t fw_ver;
    uint8_t setup_ok;
}sm_touch_dev, *psm_touch_dev;

//cammand struct for mcap
struct m_ctp_cmd_std_t
{
    uint16_t chk; // 16 bit checksum
    uint16_t d0;  //data 0
    uint16_t d1;  //data 1
    uint16_t d2;  //data 2
    uint16_t d3;  //data 3
    uint16_t d4;  //data 4
    uint16_t d5;  //data 5

    uint8_t  id;   //offset 15
    uint8_t  tag;  //offset 16
};

//response struct for mcap
struct m_ctp_rsp_std_t
{
    uint16_t chk; // 16 bit checksum
    uint16_t d0;  //data 0
    uint16_t d1;  //data 1
    uint16_t d2;  //data 2
    uint16_t d3;  //data 3
    uint16_t d4;  //data 4
    uint16_t d5;  //data 5

    uint8_t  cc;  //offset 15
    uint8_t  id;  //offset 16
};

enum SEMI_DRV_ERR
{
    SEMI_DRV_ERR_OK = 0,
    SEMI_DRV_ERR_HAL_IO,
    SEMI_DRV_ERR_NO_INIT,
    SEMI_DRV_ERR_TIMEOUT,
    SEMI_DRV_ERR_CHECKSUM,
    SEMI_DRV_ERR_RESPONSE,
    SEMI_DRV_INVALID_CMD,
    SEMI_DRV_INVALID_PARAM,
    SEMI_DRV_ERR_NOT_MATCH,
};

enum CMD_TYPE_ID
{
    CMD_NA              = 0x0f,
    CMD_IDENTITY        = 0x01,
    CMD_CTP_SSCAN       = 0x02,
    CMD_CTP_IOCTL       = 0x03,


    CMD_MEM_WR          = 0x30,
    CMD_MEM_RD          = 0x31,
    CMD_FLASH_ERASE     = 0x32,
    CMD_FW_SUM          = 0x33,
    CMD_WRITE_REGISTER  = 0X35,
    CMD_READ_REGISTER   = 0X36,
    CMD_BSPR_WRITE      = 0x37,
    CMD_BSPR_READ       = 0x38,
};

#define NVM_W                              0x0
#define NVM_R                              0x3
#define CORE_R                              0x4
#define CORE_W                              0x5

#define TP_CMD_BUFF_ADDR             0x20000000
#define TP_RSP_BUFF_ADDR             0x20000000
#define TP_WR_BUFF_ADDR              0x20002000
#define TP_RD_BUFF_ADDR              0x20002400
#define TP_HOLD_MCU_ADDR             0x40007000
#define TP_AUTO_FEED_ADDR            0x40007010
#define TP_REMAP_MCU_ADDR            0x40007000
#define TP_RELEASE_MCU_ADDR          0x40007000
#define TP_HOLD_MCU_VAL              0x12044000
#define TP_AUTO_FEED_VAL             0x0000925a
#define TP_REMAP_MCU_VAL             0x12044002
#define TP_RELEASE_MCU_VAL           0x12044003
#define VID_PID_BACKUP_ADDR          (40 * 1024 + 0x10)

#define platform_delay_ms rt_thread_mdelay

//chsc5816 end

#define TOUCH_CHIP_ID_CST716            0x20
#define TOUCH_CHIP_ID_CST816T           0xB5
#define TOUCH_CHIP_ID_CST918            (0x3962426)
#define TOUCH_CHIP_ID_CST918A           (0x396241a)


#define TOUCH_SLAVE                  (0x2e)

#define  CHSC5816_I2C_ADDR           0x2E  /* I2C slave address */

int32_t semi_check_and_update(sm_touch_dev *st_dev);
void semi_touch_setup_check(void);
void CHSC5816TP_INIT(void);

#endif

