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

#ifndef _WIFI_IF_H_
#define _WIFI_IF_H_

int wifi_if_sdio_init(void);
void wifi_if_sdio_exit(void);

int wifi_if_sdio_suspend(void);
int wifi_if_sdio_resume(void);

void wifi_open(void);
void wifi_close(void);

#endif /* _WIFI_IF_H_ */
