/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _CLI_CMD_H_
#define _CLI_CMD_H_

#include "rwnx_defs.h"

typedef void (*CliCb)(char *rsp);

static int parse_line (char *line, char *argv[]);
int handle_private_cmd(struct rwnx_hw *rwnx_hw, char *command);
unsigned int command_strtoul(const char *cp, char **endp, unsigned int base);

bool aic_cli_cmd_init(struct rwnx_hw *rwnx_hw);
bool aic_cli_cmd_deinit(struct rwnx_hw *rwnx_hw);
void CliSetCb(CliCb cb);
int Cli_RunCmd(char *CmdBuffer);

#endif