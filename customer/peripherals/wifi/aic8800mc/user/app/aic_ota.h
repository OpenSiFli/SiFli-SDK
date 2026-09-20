/**
 * @attention
 * Copyright (c) 2018-2024 AICSemi Ltd. All rights reserved.
 * Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _AIC_OTA_H_
#define _AIC_OTA_H_

#define IMAGE_INFO_SIZE             0x1000

/* Must match UPGRADE_START_ADDR in the AIC8800MC firmware bootloader_lzma.h:
   the upgrade area 0x08142000..0x081E3000 (640KB) reserves the first 4KB as
   the image_header. */
#define UPGRADE_START_ADDR          0x08142000
#define UPGRADE_INFO_ADDR           (UPGRADE_START_ADDR)
#define UPGRADE_IMAGE_ADDR          (UPGRADE_START_ADDR + IMAGE_INFO_SIZE)

#define OTA_PKG_SIZE                1024

/* OTA mode, must match the AIC8800MC firmware LZMA=on/off (CFG_LZMA_APP):
   - NORMAL: send the raw image, write from UPGRADE_IMAGE_ADDR, firmware writes
             the image_header at the last step, version taken from aic_host_ota()
             new_version argument;
   - LZMA:   send the whole package (4KB image_header + lzma stream) produced by
             image_pack_ota.py, write from UPGRADE_START_ADDR (image carries its
             own header), firmware only recomputes the whole-package CRC and
             compares the header version; new_version is ignored. */
#define AIC_OTA_MODE_NORMAL         0
#define AIC_OTA_MODE_LZMA           1

int aic_host_ota(struct rwnx_hw *rwnx_hw, uint8_t *file_addr, uint32_t file_len,
                 const char *new_version, int mode);

#endif /* _AIC_OTA_H_ */
