/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include "cli_al.h"
#include "cli_cmd.h"

void aic_cli_set_cb(aic_clicb cb)
{
    CliSetCb((CliCb)cb);
}

int aic_cli_run_cmd(char *CmdBuffer)
{
    return Cli_RunCmd(CmdBuffer);
}

