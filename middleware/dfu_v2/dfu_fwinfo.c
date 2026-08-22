/**
 * @file dfu_fwinfo.c
 * @brief DFU V2 — Firmware Info Management Implementation
 *
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include <string.h>
#include "dfu_fwinfo.h"
#include "engine/dfu_engine_flash.h"

#define LOG_TAG "dfu.fwinfo"
#include <log.h>

/*============================================================================
 * Helpers
 *============================================================================*/

#define INFO_TOTAL_SIZE  (sizeof(struct dfu_fw_info) * DFU_MAX_FW_FILES)

static int read_all(struct dfu_fw_info *all)
{
    dfu_engine_err_t err = dfu_ef_read(DFU_FWINFO_BASE_ADDR,
                                        (uint8_t *)all, INFO_TOTAL_SIZE);
    return (err == DFU_ENGINE_ERR_NONE) ? 0 : -1;
}

static int write_all(const struct dfu_fw_info *all)
{
    /* Erase */
    dfu_engine_err_t err = dfu_ef_erase(DFU_FWINFO_BASE_ADDR, INFO_TOTAL_SIZE);
    if (err != DFU_ENGINE_ERR_NONE)
        return -1;

    /* Write */
    err = dfu_ef_write(DFU_FWINFO_BASE_ADDR,
                       (const uint8_t *)all, INFO_TOTAL_SIZE);
    return (err == DFU_ENGINE_ERR_NONE) ? 0 : -1;
}

/*============================================================================
 * Public API
 *============================================================================*/

int dfu_fwinfo_set(int index, const struct dfu_fw_info *info)
{
    if (index < 0 || index >= DFU_MAX_FW_FILES || !info)
        return -1;

    struct dfu_fw_info *all = rt_malloc(sizeof(struct dfu_fw_info) * DFU_MAX_FW_FILES);
    if (!all)
    {
        LOG_E("fwinfo_set: malloc failed");
        return -1;
    }

    /* Init NAND cache (no-op for NOR) */
    dfu_ef_cache_init(DFU_FWINFO_BASE_ADDR);

    if (read_all(all) != 0)
    {
        LOG_E("read firmware info failed");
        dfu_ef_cache_deinit();
        rt_free(all);
        return -1;
    }

    memcpy(&all[index], info, sizeof(struct dfu_fw_info));

    if (write_all(all) != 0)
    {
        LOG_E("write firmware info failed");
        dfu_ef_cache_deinit();
        rt_free(all);
        return -1;
    }

    dfu_ef_cache_flush();
    dfu_ef_cache_deinit();
    rt_free(all);
    return 0;
}

int dfu_fwinfo_get(int index, struct dfu_fw_info *info)
{
    if (index < 0 || index >= DFU_MAX_FW_FILES || !info)
        return -1;

    uint32_t addr = DFU_FWINFO_BASE_ADDR +
                    (uint32_t)index * sizeof(struct dfu_fw_info);

    dfu_engine_err_t err = dfu_ef_read(addr, (uint8_t *)info,
                                        sizeof(struct dfu_fw_info));
    return (err == DFU_ENGINE_ERR_NONE) ? 0 : -1;
}

int dfu_fwinfo_set_update_flags(void)
{
    LOG_I("setting update flags...");

    struct dfu_fw_info all[DFU_MAX_FW_FILES];

    dfu_ef_cache_init(DFU_FWINFO_BASE_ADDR);

    if (read_all(all) != 0)
    {
        LOG_E("read failed");
        dfu_ef_cache_deinit();
        return -1;
    }

    int marked = 0;
    for (int i = 0; i < DFU_MAX_FW_FILES; i++)
    {
        if (all[i].name[0] == '\0')
            continue;

        all[i].magic = DFU_FW_MAGIC;
        all[i].needs_update = 1;
        marked++;
        LOG_I("marked [%d] %s for update", i, all[i].name);
    }

    /* If no existing entries, create a DFU trigger entry */
    if (marked == 0)
    {
        LOG_I("creating DFU trigger entry");
        memset(&all[0], 0, sizeof(struct dfu_fw_info));
        strncpy(all[0].name, "DFU_TRIGGER", sizeof(all[0].name) - 1);
        all[0].magic = DFU_FW_MAGIC;
        all[0].needs_update = 1;
    }

    if (write_all(all) != 0)
    {
        LOG_E("write failed");
        dfu_ef_cache_deinit();
        return -1;
    }

    dfu_ef_cache_flush();
    dfu_ef_cache_deinit();

    LOG_I("update flags set, %d entries marked", marked);
    return 0;
}

void dfu_fwinfo_clear(void)
{
    LOG_I("clearing firmware info...");

    struct dfu_fw_info all[DFU_MAX_FW_FILES];
    memset(all, 0, sizeof(all));

    dfu_ef_cache_init(DFU_FWINFO_BASE_ADDR);
    write_all(all);
    dfu_ef_cache_flush();
    dfu_ef_cache_deinit();

    LOG_I("firmware info cleared");
}

/*============================================================================
 * Backward Compatibility Aliases
 *
 * dfu_engine.c calls dfu_core_set_firmware_info / dfu_core_set_update_flags.
 * These wrappers eliminate the need for dfu_core.c.
 *============================================================================*/

int dfu_core_set_firmware_info(int index, const struct dfu_fw_info *fw_info)
{
    return dfu_fwinfo_set(index, fw_info);
}

int dfu_core_set_update_flags(void)
{
    return dfu_fwinfo_set_update_flags();
}
