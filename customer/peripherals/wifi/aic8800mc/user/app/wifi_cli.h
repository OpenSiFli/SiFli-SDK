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

#ifndef _WIFI_CLI_H_
#define _WIFI_CLI_H_

/**
 * @brief CLI initialization
 * @return non-0 if error occurs
 */
int wifi_cli_init(void);

/**
 * @brief CLI command handler
 * @param argc - arg cnt
 * @param argv - arg val
 * @return non-0 if error occurs
 */
int wifi_cli_handler(int argc, char **argv);

int wifi_cli_handler_t(char *cmd);


#endif /* _WIFI_CLI_H_ */
