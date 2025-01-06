#include <stdint.h>
#include <string.h>
#include "bf0_hal.h"
#include "mem_map.h"
#include "register.h"
#include "bf0_hal_patch.h"
#ifdef HAL_LCPU_PATCH_MODULE
static const unsigned int g_lcpu_patch_list[] =
{
    0x50544348, 0x00000000,
};

static const unsigned int g_lcpu_patch_bin[] =
{
    0xB804F000, 0x70102000, 0x00004770, 0x21044803,
    0x10C4F8C0, 0x49034802, 0x47706001, 0x204008A8,
    0x204001B8, 0x00405011,
};
void lcpu_patch_install_rev_b()
{
    uint32_t entry[3] = {0x48434150, 0x0, LCPU_PATCH_CODE_START_ADDR + 1};
    memcpy((void *)LCPU_PATCH_BUF_START_ADDR, (void *)&entry, 12);
#ifdef SOC_BF0_HCPU
    memset((void *)(LCPU_PATCH_CODE_START_ADDR_S), 0, LCPU_PATCH_CODE_SIZE);
    memcpy((void *)(LCPU_PATCH_CODE_START_ADDR_S), g_lcpu_patch_bin, sizeof(g_lcpu_patch_bin));
#else
    memset((void *)(LCPU_PATCH_CODE_START_ADDR), 0, LCPU_PATCH_CODE_SIZE);
    memcpy((void *)(LCPU_PATCH_CODE_START_ADDR), g_lcpu_patch_bin, sizeof(g_lcpu_patch_bin));
#endif
}
uint32_t *HAL_PATCH_GetEntryAddr(void)
{
    uint32_t *entry_addr = (uint32_t *)LCPU_PATCH_RECORD_ADDR;
    uint8_t rev_id = __HAL_SYSCFG_GET_REVID();
    if (rev_id >= HAL_CHIP_REV_ID_A4)
        entry_addr = (uint32_t *)g_lcpu_patch_list;
    return entry_addr;
}
#endif
