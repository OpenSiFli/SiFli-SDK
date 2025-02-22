#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <string.h>
#include "dfu_service.h"
#include "dfu_internal.h"



//#define LOG_TAG "dfu_install"
//#include "log.h"
#if defined(RT_USING_DFS_MNTTABLE)
#include "dfs_posix.h"

#include "flash_map.h"
const struct dfs_mount_tbl mount_table[] =
{
    FS_MOUNT_TABLE
};
#endif

void boot_images(void)
{
    uint32_t mode = HAL_HPAON_GET_POWER_MODE();
    if (mode == AON_PMR_STANDBY)
    {
        /* Speed up for standby mode. */
        run_img((uint8_t *)HCPU_FLASH_CODE_START_ADDR);
    }
}

#include "dfu_service.h"
static dfu_event_ack_t dfu_cb(uint16_t event, void *param)
{
    return DFU_EVENT_SUCCESSED;
}

int main(void)
{
    int count = 0;
    dfu_register(dfu_cb);

    dfu_ctrl_set_mode(DFU_CTRL_OTA_MODE);
    dfu_ctrl_reset_handler();

    while (1)
    {
        count++;
        rt_thread_mdelay(200000);
    }


    return RT_EOK;
}