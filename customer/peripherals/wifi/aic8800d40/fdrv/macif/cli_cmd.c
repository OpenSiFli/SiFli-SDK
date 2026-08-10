/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//-------------------------------------------------------------------
// Driver Header Files
//-------------------------------------------------------------------
#include "lmac_msg.h"
#include "fhost.h"
#include "fhost_cntrl.h"
#include "rwnx_utils.h"
#include "rwnx_defs.h"
#include "reg_access.h"
#include "rwnx_main.h"
#include "rwnx_msg_tx.h"
#include "rwnx_rx.h"
#include "rwnx_platform.h"
#include "cli_cmd.h"
#include "wifi_if.h"
#include "wlan_if.h"
#include "sys_al.h"
#include "rtos_port.h"
#include "rtos_errno.h"
#include <ctype.h>
#include <stdio.h>
#include <string.h>

#define CMD_MAXARGS 30

extern struct aic_ap_info g_ap_info;
extern int ap_net_id;

enum
{
    SET_TX,
    SET_TXSTOP,
    SET_TXTONE,
    SET_RX,
    GET_RX_RESULT,
    SET_RXSTOP,
    SET_RX_METER,
    SET_POWER,
    SET_XTAL_CAP,
    SET_XTAL_CAP_FINE,
    GET_EFUSE_BLOCK,
    SET_FREQ_CAL,
    SET_FREQ_CAL_FINE,
    GET_FREQ_CAL,
    SET_MAC_ADDR,
    GET_MAC_ADDR,
    SET_BT_MAC_ADDR,
    GET_BT_MAC_ADDR,
    SET_VENDOR_INFO,
    GET_VENDOR_INFO,
    RDWR_PWRMM,
    RDWR_PWRIDX,
    RDWR_PWRLVL = RDWR_PWRIDX,
    RDWR_PWROFST,
    RDWR_DRVIBIT,
    RDWR_EFUSE_PWROFST,
    RDWR_EFUSE_DRVIBIT,
    SET_PAPR,
    SET_CAL_XTAL,
    GET_CAL_XTAL_RES,
    SET_COB_CAL,
    GET_COB_CAL_RES,
    RDWR_EFUSE_USRDATA,
    SET_NOTCH,
    RDWR_PWROFSTFINE,
    RDWR_EFUSE_PWROFSTFINE,
    RDWR_EFUSE_SDIOCFG,
    RDWR_EFUSE_USBVIDPID,
    SET_SRRC,
    SET_FSS,
    RDWR_EFUSE_HE_OFF,
    SET_USB_OFF,
    SET_PLL_TEST,
    SET_ANT_MODE,
    RDWR_BT_EFUSE_PWROFST,
#ifdef CONFIG_USB_BT
    BT_CMD_BASE = 0x100,
    BT_RESET,
    BT_TXDH,
    BT_RXDH,
    BT_STOP,
    GET_BT_RX_RESULT,
#endif
};

typedef struct
{
    u8_l chan;
    u8_l bw;
    u8_l mode;
    u8_l rate;
    u16_l length;
    u16_l tx_intv_us;
    s8_l max_pwr;
} cmd_rf_settx_t;

typedef struct
{
    u8_l val;
} cmd_rf_setfreq_t;

typedef struct
{
    u8_l chan;
    u8_l bw;
} cmd_rf_rx_t;

typedef struct
{
    u8_l block;
} cmd_rf_getefuse_t;

typedef struct
{
    u8_l dutid;
    u8_l chip_num;
    u8_l dis_xtal;
} cmd_rf_setcobcal_t;

typedef struct
{
    struct co_list_hdr hdr;
    char cmd_buf[0];
} cli_cmd_t;

static struct co_list cli_cmd_list;
static rtos_mutex cli_cmd_mutex = NULL;
static CliCb Cli_cb = NULL;

#define BUF_PRINT_SIZE_MAX  256

static void CliPrint(const char *fmt, ...)
{
    va_list ap;
    char buffer[BUF_PRINT_SIZE_MAX];

    memset(buffer, 0, BUF_PRINT_SIZE_MAX);

    va_start(ap, fmt);
    vsnprintf(buffer, BUF_PRINT_SIZE_MAX, fmt, ap);
    va_end(ap);

    AIC_LOG_PRINTF("[Cli]:%s", buffer);

    if (Cli_cb)
    {
        Cli_cb(buffer);
    }
}

void CliSetCb(CliCb cb)
{
    Cli_cb = cb;
}
#if 0
static int strcasecmp(const char *s1, char *s2)
{
    char a, b;
    a = ((*s1 >= 'a' && *s1 <= 'z') ? *s1 - 32 : *s1);
    b = ((*s2 >= 'a' && *s2 <= 'z') ? *s2 - 32 : *s2);
    while (a == b)
    {
        s1++;
        s2++;
        if ((*s1 == '\0') && (*s2 == '\0'))
            return 0;
        a = ((*s1 >= 'a' && *s1 <= 'z') ? *s1 - 32 : *s1);
        b = ((*s2 >= 'a' && *s2 <= 'z') ? *s2 - 32 : *s2);

    }


    return (a - b);
}
#endif
int Cli_RunCmd(char *CmdBuffer)
{
    AIC_LOG_PRINTF("cli cmd: %s\n", CmdBuffer);

    uint32_t cli_cmd_len = sizeof(cli_cmd_t) + strlen(CmdBuffer) + 1;
    cli_cmd_t *cli_cmd = rtos_malloc(cli_cmd_len);
    if (cli_cmd == NULL)
    {
        AIC_LOG_ERROR("cli_cmd malloc fail\n");
        return -1;
    }
    memset(cli_cmd, 0, cli_cmd_len);
    strcpy(cli_cmd->cmd_buf, CmdBuffer);
    rtos_mutex_lock(cli_cmd_mutex, -1);
    co_list_push_back(&cli_cmd_list, &cli_cmd->hdr);
    rtos_mutex_unlock(cli_cmd_mutex);
    AIC_LOG_TRACE("put cli cmd to list\n");

    rtos_semaphore_signal(g_rwnx_hw->cli_cmd_sema, false);

    return 0;
}

void cli_cmd_task(void *argv)
{
    struct rwnx_hw *rwnx_hw = (struct rwnx_hw *)argv;
    int ret = 0;

    AIC_LOG_PRINTF("cli_cmd_task\n");
    while (1)
    {
        int32_t ret = rtos_semaphore_wait(rwnx_hw->cli_cmd_sema, AIC_RTOS_WAIT_FOREVEVR);
        AIC_LOG_TRACE("cli cmd sema wait: ret=%d\n", ret);
        rtos_mutex_lock(cli_cmd_mutex, -1);
        cli_cmd_t *cli_cmd = (cli_cmd_t *)co_list_pop_front(&cli_cmd_list);
        rtos_mutex_unlock(cli_cmd_mutex);
        if (cli_cmd == NULL)
        {
            AIC_LOG_ERROR("cli cmd list empty\n");
            continue;
        }
        AIC_LOG_TRACE("cli cmd check again: %s\n", cli_cmd->cmd_buf);
        ret = handle_private_cmd(rwnx_hw, cli_cmd->cmd_buf);
        rtos_free(cli_cmd);
        if (ret < 0)
        {
            AIC_LOG_ERROR("handle_private_cmd fail: ret=%d\n", ret);
        }
    }
}

bool aic_cli_cmd_init(struct rwnx_hw *rwnx_hw)
{
    int ret = 0;

    AIC_LOG_PRINTF("aic_cli_cmd_init\n");

    rtos_semaphore_create(&rwnx_hw->cli_cmd_sema, "cli_cmd_sema", 1024, 0);
    if (rwnx_hw->cli_cmd_sema == NULL)
    {
        AIC_LOG_ERROR("aic cli cmd sema create fail\n");
        return FALSE;
    }

    rtos_mutex_create(&cli_cmd_mutex, "cli_cmd_mutex");
    if (cli_cmd_mutex == NULL)
    {
        AIC_LOG_ERROR("cli_cmd_mutex create fail\n");
        return FALSE;
    }

    ret = rtos_task_create(cli_cmd_task, "cli_cmd_task", CLI_CMD_TASK,
                           cli_cmd_stack_size, (void *)rwnx_hw, cli_cmd_priority,
                           &rwnx_hw->cli_cmd_task_hdl);
    if (ret || (rwnx_hw->cli_cmd_task_hdl == NULL))
    {
        AIC_LOG_ERROR("aic cli cmd task create fail\n");
        return FALSE;
    }

    co_list_init(&cli_cmd_list);

    return TRUE;
}

bool aic_cli_cmd_deinit(struct rwnx_hw *rwnx_hw)
{
    int ret = 0;

    AIC_LOG_PRINTF("aic_cli_cmd_deinit\n");
    //if rtos one key remove ,ignore it
#ifndef CONFIG_DRIVER_ORM
    rtos_task_delete(rwnx_hw->cli_cmd_task_hdl);
    rtos_semaphore_delete(rwnx_hw->cli_cmd_sema);
    rtos_mutex_delete(cli_cmd_mutex);
    while (co_list_cnt(&cli_cmd_list))
    {
        cli_cmd_t *cli_cmd = (cli_cmd_t *)co_list_pop_front(&cli_cmd_list);
        if (cli_cmd)
        {
            rtos_free(cli_cmd);
        }
    }
#endif
    return TRUE;
}

static int parse_line(char *line, char *argv[])
{
    int nargs = 0;

    while (nargs < CMD_MAXARGS)
    {
        /* skip any white space */
        while ((*line == ' ') || (*line == '\t'))
        {
            ++line;
        }

        if (*line == '\0')      /* end of line, no more args    */
        {
            argv[nargs] = 0;
            return (nargs);
        }

        /* Argument include space should be bracketed by quotation mark */
        if (*line == '\"')
        {
            /* Skip quotation mark */
            line++;

            /* Begin of argument string */
            argv[nargs++] = line;

            /* Until end of argument */
            while (*line && (*line != '\"'))
            {
                ++line;
            }
        }
        else
        {
            argv[nargs++] = line;    /* begin of argument string    */

            /* find end of string */
            while (*line && (*line != ' ') && (*line != '\t'))
            {
                ++line;
            }
        }

        if (*line == '\0')      /* end of line, no more args    */
        {
            argv[nargs] = 0;
            return (nargs);
        }

        *line++ = '\0';         /* terminate current arg     */
    }

    AIC_LOG_PRINTF("** Too many args (max. %d) **\n", CMD_MAXARGS);

    return (nargs);
}

unsigned int command_strtoul(const char *cp, char **endp, unsigned int base)
{
    unsigned int result = 0, value, is_neg = 0;

    if (*cp == '0')
    {
        cp++;
        if ((*cp == 'x') && isxdigit(cp[1]))
        {
            base = 16;
            cp++;
        }
        if (!base)
        {
            base = 8;
        }
    }
    if (!base)
    {
        base = 10;
    }
    if (*cp == '-')
    {
        is_neg = 1;
        cp++;
    }
    while (isxdigit(*cp) && (value = isdigit(*cp) ? *cp - '0' : (islower(*cp) ? toupper(*cp) : *cp) - 'A' + 10) < base)
    {
        result = result * base + value;
        cp++;
    }
    if (is_neg)
        result = (unsigned int)((int)result * (-1));

    if (endp)
        *endp = (char *)cp;
    return result;
}

#define P2P_GO_SSID_STRING  "DIRECT-AIC-RTT-P2P"
#define P2P_GO_PASS_STRING  "kkkkkkkk"

void cmd_set_lp_level(uint8_t lp_level)
{
    int ret;
    ret = fhost_cntrl_me_set_lp_level(lp_level);
    if (ret)
    {
        AIC_LOG_PRINTF("set lp_level fail, ret=%d\n", ret);
    }
}

int cmd_dbg_severity_config(int argc, char *const argv[])
{
    int ret = 0;
    unsigned int func = 0;
    if (argc > 1)
    {
        func = command_strtoul(argv[1], NULL, 0);
    }
    if (func == 0)   // show curr
    {
        CliPrint("dbg sev: %d\n", dbg_get_severity());
    }
    else if (func == 1)     // config sev
    {
        if (argc > 2)
        {
            unsigned int sev_idx = command_strtoul(argv[2], NULL, 0);
            if (sev_idx < DBG_SEV_IDX_MAX)
            {
                CliPrint("Cfg dbg sev: %d\n", sev_idx);
                dbg_set_severity(sev_idx);
            }
            else
            {
                CliPrint("invalid sev_idx:%d\n", sev_idx);
                ret = 1;
            }
        }
        else
        {
            CliPrint("invalid args\n");
            ret = 2;
        }
    }
    else
    {
        CliPrint("invalid func\n");
        ret = 3;
    }
    return ret;
}

int cmd_dbg_module_config(int argc, char *const argv[])
{
    int ret = 0;
    unsigned int func = 0;
    if (argc > 1)
    {
        func = command_strtoul(argv[1], NULL, 0);
    }
    if (func == 0)   // show curr
    {
        CliPrint("dbg mod: %08X\n", dbg_get_module());
    }
    else if (func == 1)     // config mod
    {
        if (argc > 2)
        {
            unsigned int mod_msk = command_strtoul(argv[2], NULL, 16);
            CliPrint("Cfg dbg mod: %08X\n", mod_msk);
            dbg_set_module(mod_msk);
        }
        else
        {
            CliPrint("invalid args\n");
            ret = 2;
        }
    }
    else
    {
        CliPrint("invalid func\n");
        ret = 3;
    }
    return ret;
}

int handle_private_cmd(struct rwnx_hw *rwnx_hw, char *command)
{
    int bytes_written = 0;
    char *argv[CMD_MAXARGS + 1];
    int argc;

    //RWNX_DBG(RWNX_FN_ENTRY_STR);

    if ((argc = parse_line(command, argv)) == 0)
    {
        return -1;
    }

    if (rwnx_hw->mode == WIFI_MODE_RFTEST)
    {
        struct dbg_rftest_cmd_cfm cfm = {{0,}};
        u8_l mac_addr[6];
        cmd_rf_settx_t settx_param;
        cmd_rf_rx_t setrx_param;
        int freq;
        cmd_rf_getefuse_t getefuse_param;
        cmd_rf_setfreq_t cmd_setfreq;
        cmd_rf_setcobcal_t setcob_cal;
        u8_l ana_pwr;
        u8_l dig_pwr;
        u8_l pwr;
        u8_l xtal_cap;
        u8_l xtal_cap_fine;
        u8_l vendor_info;
#ifdef CONFIG_USB_BT
        int bt_index;
        u8_l dh_cmd_reset[4];
        u8_l dh_cmd_txdh[18];
        u8_l dh_cmd_rxdh[17];
        u8_l dh_cmd_stop[5];
#endif
        u8_l buf[2];
        s8_l freq_ = 0;
        u8_l func = 0;
        int ret;
        do
        {
            //#ifdef AICWF_SDIO_SUPPORT
            //struct rwnx_hw *p_rwnx_hw = g_rwnx_plat->sdiodev->rwnx_hw;
            //#endif
            //#ifdef AICWF_USB_SUPPORT

            //struct rwnx_hw *p_rwnx_hw = cntrl_rwnx_hw;
            //#endif
            if (strcasecmp(argv[0], "GET_RX_RESULT") == 0)
            {
                CliPrint("get_rx_result\r\n");
                rwnx_send_rftest_req(rwnx_hw, GET_RX_RESULT, 0, NULL, &cfm);
                //memcpy(command, &cfm.rftest_result[0], 8);
                bytes_written = 8;
                CliPrint("done: getrx fcsok=%d, total=%d\r\n", (unsigned int)cfm.rftest_result[0], (unsigned int)cfm.rftest_result[1]);
            }
            else if (strcasecmp(argv[0], "SET_TX") == 0)
            {
                CliPrint("set_tx\r\n");
                AIC_LOG_PRINTF("lemon set_tx\r\n");
                if (argc < 6)
                {
                    CliPrint("wrong param\r\n");
                    break;
                }
                settx_param.chan = command_strtoul(argv[1], NULL, 10);
                settx_param.bw = command_strtoul(argv[2], NULL, 10);
                settx_param.mode = command_strtoul(argv[3], NULL, 10);
                settx_param.rate = command_strtoul(argv[4], NULL, 10);
                settx_param.length = command_strtoul(argv[5], NULL, 10);
                if (argc > 6)
                {
                    settx_param.tx_intv_us = command_strtoul(argv[6], NULL, 10);
                }
                else
                {
                    settx_param.tx_intv_us = 100000;
                }
                if (argc > 7)
                {
                    settx_param.max_pwr = command_strtoul(argv[7], NULL, 10);
                }
                else
                {
                    settx_param.max_pwr = 127;
                }
                CliPrint("txparam:%d,%d,%d,%d,%d\r\n", settx_param.chan, settx_param.bw,
                         settx_param.mode, settx_param.rate, settx_param.length);
                rwnx_send_rftest_req(rwnx_hw, SET_TX, sizeof(cmd_rf_settx_t), (u8_l *)&settx_param, NULL);
            }
            else if (strcasecmp(argv[0], "SET_TXSTOP") == 0)
            {
                CliPrint("settx_stop\r\n");
                rwnx_send_rftest_req(rwnx_hw, SET_TXSTOP, 0, NULL, NULL);
            }
            else if (strcasecmp(argv[0], "SET_TXTONE") == 0)
            {
                CliPrint("set_tx_tone,argc:%d\r\n", argc);
                if ((argc == 2) || (argc == 3))
                {
                    CliPrint("argv 1:%s\r\n", argv[1]);
                    //u8_l func = (u8_l)command_strtoul(argv[1], NULL, 16);
                    func = (u8_l)command_strtoul(argv[1], NULL, 16);
                    //s8_l freq;
                    if (argc == 3)
                    {
                        CliPrint("argv 2:%s\r\n", argv[2]);
                        freq_ = (u8_l)command_strtoul(argv[2], NULL, 10);
                    }
                    else
                    {
                        freq_ = 0;
                    };
                    //u8_l buf[2] = {func, (u8_l)freq};
                    buf[0] = func;
                    buf[1] = (u8_l)freq_;
                    rwnx_send_rftest_req(rwnx_hw, SET_TXTONE, argc - 1, buf, NULL);
                }
                else
                {
                    CliPrint("wrong args\r\n");
                }
            }
            else if (strcasecmp(argv[0], "SET_RX") == 0)
            {
                CliPrint("set_rx\r\n");
                if (argc < 3)
                {
                    CliPrint("wrong param\r\n");
                    break;
                }
                setrx_param.chan = command_strtoul(argv[1], NULL, 10);
                setrx_param.bw = command_strtoul(argv[2], NULL, 10);
                rwnx_send_rftest_req(rwnx_hw, SET_RX, sizeof(cmd_rf_rx_t), (u8_l *)&setrx_param, NULL);
            }
            else if (strcasecmp(argv[0], "SET_RXSTOP") == 0)
            {
                CliPrint("set_rxstop\r\n");
                rwnx_send_rftest_req(rwnx_hw, SET_RXSTOP, 0, NULL, NULL);
            }
            else if (strcasecmp(argv[0], "SET_RX_METER") == 0)
            {
                CliPrint("set_rx_meter\r\n");
                freq = (int)command_strtoul(argv[1], NULL, 10);
                rwnx_send_rftest_req(rwnx_hw, SET_RX_METER, sizeof(freq), (u8_l *)&freq, NULL);
            }
            else if (strcasecmp(argv[0], "SET_FREQ_CAL") == 0)
            {
                CliPrint("set_freq_cal\r\n");
                if (argc < 2)
                {
                    CliPrint("wrong param\r\n");
                    break;
                }
                cmd_setfreq.val = command_strtoul(argv[1], NULL, 16);
                CliPrint("param:%x\r\n", cmd_setfreq.val);
                rwnx_send_rftest_req(rwnx_hw, SET_FREQ_CAL, sizeof(cmd_rf_setfreq_t), (u8_l *)&cmd_setfreq, &cfm);
                //memcpy(command, &cfm.rftest_result[0], 4);
                bytes_written = 4;
                CliPrint("done: freq_cal: 0x%8x\r\n", (unsigned int)cfm.rftest_result[0]);
            }
            else if (strcasecmp(argv[0], "SET_FREQ_CAL_FINE") == 0)
            {
                CliPrint("set_freq_cal_fine\r\n");
                if (argc < 2)
                {
                    CliPrint("wrong param\r\n");
                    break;
                }
                cmd_setfreq.val = command_strtoul(argv[1], NULL, 16);
                CliPrint("param:%x\r\n", cmd_setfreq.val);
                rwnx_send_rftest_req(rwnx_hw, SET_FREQ_CAL_FINE, sizeof(cmd_rf_setfreq_t), (u8_l *)&cmd_setfreq, &cfm);
                //memcpy(command, &cfm.rftest_result[0], 4);
                bytes_written = 4;
                CliPrint("done: freq_cal_fine: 0x%8x\r\n", (unsigned int)cfm.rftest_result[0]);
            }
            else if (strcasecmp(argv[0], "GET_EFUSE_BLOCK") == 0)
            {
                CliPrint("get_efuse_block\r\n");
                if (argc < 2)
                {
                    CliPrint("wrong param\r\n");
                    break;
                }
                getefuse_param.block = command_strtoul(argv[1], NULL, 10);
                rwnx_send_rftest_req(rwnx_hw, GET_EFUSE_BLOCK, sizeof(cmd_rf_getefuse_t), (u8_l *)&getefuse_param, &cfm);
                //memcpy(command, &cfm.rftest_result[0], 4);
                bytes_written = 4;
                CliPrint("done:efuse: 0x%8x\r\n", (unsigned int)cfm.rftest_result[0]);
            }
            else if (strcasecmp(argv[0], "SET_POWER") == 0)
            {
                CliPrint("set_power\r\n");
                ana_pwr = command_strtoul(argv[1], NULL, 16);
                dig_pwr = command_strtoul(argv[2], NULL, 16);
                pwr = (ana_pwr << 4 | dig_pwr);
                if (ana_pwr > 0xf || dig_pwr > 0xf)
                {
                    CliPrint("invalid param\r\n");
                    break;
                }
                CliPrint("pwr =%x\r\n", pwr);
                rwnx_send_rftest_req(rwnx_hw, SET_POWER, sizeof(pwr), (u8_l *)&pwr, NULL);
            }
            else if (strcasecmp(argv[0], "SET_XTAL_CAP") == 0)
            {
                CliPrint("set_xtal_cap\r\n");
                if (argc < 2)
                {
                    CliPrint("wrong param\r\n");
                    break;
                }
                xtal_cap = command_strtoul(argv[1], NULL, 10);
                CliPrint("xtal_cap =%d\r\n", (int8_t)xtal_cap);
                rwnx_send_rftest_req(rwnx_hw, SET_XTAL_CAP, sizeof(xtal_cap), (u8_l *)&xtal_cap, &cfm);
                //memcpy(command, &cfm.rftest_result[0], 4);
                bytes_written = 4;
                CliPrint("done:xtal cap: 0x%x -> 0x%x\r\n", (unsigned int)cfm.rftest_result[0] - (int8_t)xtal_cap, (unsigned int)cfm.rftest_result[0]);
            }
            else if (strcasecmp(argv[0], "SET_XTAL_CAP_FINE") == 0)
            {
                CliPrint("set_xtal_cap_fine\r\n");
                if (argc < 2)
                {
                    CliPrint("wrong param\r\n");
                    break;
                }
                xtal_cap_fine = command_strtoul(argv[1], NULL, 10);
                CliPrint("xtal_cap_fine =%d\r\n", (int8_t)xtal_cap_fine);
                rwnx_send_rftest_req(rwnx_hw, SET_XTAL_CAP_FINE, sizeof(xtal_cap_fine), (u8_l *)&xtal_cap_fine, &cfm);
                //memcpy(command, &cfm.rftest_result[0], 4);
                bytes_written = 4;
                CliPrint("done:xtal cap_fine: 0x%x -> 0x%x\r\n", (unsigned int)cfm.rftest_result[0] - (int8_t)xtal_cap_fine, (unsigned int)cfm.rftest_result[0]);
            }
            else if (strcasecmp(argv[0], "SET_MAC_ADDR") == 0)
            {
                CliPrint("set_mac_addr\r\n");
                if (argc < 7)
                {
                    CliPrint("wrong param\r\n");
                    break;
                }
                mac_addr[5] = command_strtoul(argv[1], NULL, 16);
                mac_addr[4] = command_strtoul(argv[2], NULL, 16);
                mac_addr[3] = command_strtoul(argv[3], NULL, 16);
                mac_addr[2] = command_strtoul(argv[4], NULL, 16);
                mac_addr[1] = command_strtoul(argv[5], NULL, 16);
                mac_addr[0] = command_strtoul(argv[6], NULL, 16);
                CliPrint("set macaddr:%x,%x,%x,%x,%x,%x\r\n", mac_addr[5], mac_addr[4], mac_addr[3], mac_addr[2], mac_addr[1], mac_addr[0]);
                rwnx_send_rftest_req(rwnx_hw, SET_MAC_ADDR, sizeof(mac_addr), (u8_l *)&mac_addr, NULL);
            }
            else if (strcasecmp(argv[0], "GET_MAC_ADDR") == 0)
            {
                CliPrint("get mac addr\r\n");
                rwnx_send_rftest_req(rwnx_hw, GET_MAC_ADDR, 0, NULL, &cfm);
                //memcpy(command, &cfm.rftest_result[0], 8);
                bytes_written = 8;
                CliPrint("done: get macaddr: 0x%x,0x%x\r\n", cfm.rftest_result[0], cfm.rftest_result[1]);
            }
            else if (strcasecmp(argv[0], "SET_BT_MAC_ADDR") == 0)
            {
                CliPrint("set_bt_mac_addr\r\n");
                if (argc < 7)
                {
                    CliPrint("wrong param\r\n");
                    break;
                }
                mac_addr[5] = command_strtoul(argv[1], NULL, 16);
                mac_addr[4] = command_strtoul(argv[2], NULL, 16);
                mac_addr[3] = command_strtoul(argv[3], NULL, 16);
                mac_addr[2] = command_strtoul(argv[4], NULL, 16);
                mac_addr[1] = command_strtoul(argv[5], NULL, 16);
                mac_addr[0] = command_strtoul(argv[6], NULL, 16);
                CliPrint("set bt macaddr:%x,%x,%x,%x,%x,%x\r\n", mac_addr[5], mac_addr[4], mac_addr[3], mac_addr[2], mac_addr[1], mac_addr[0]);
                rwnx_send_rftest_req(rwnx_hw, SET_BT_MAC_ADDR, sizeof(mac_addr), (u8_l *)&mac_addr, NULL);
            }
            else if (strcasecmp(argv[0], "GET_BT_MAC_ADDR") == 0)
            {
                CliPrint("get bt mac addr\r\n");
                rwnx_send_rftest_req(rwnx_hw, GET_BT_MAC_ADDR, 0, NULL, &cfm);
                //memcpy(command, &cfm.rftest_result[0], 8);
                bytes_written = 8;
                CliPrint("done: get bt macaddr: 0x%x,0x%x\r\n", cfm.rftest_result[0], cfm.rftest_result[1]);
            }
            else if (strcasecmp(argv[0], "SET_VENDOR_INFO") == 0)
            {
                vendor_info = command_strtoul(argv[1], NULL, 16);
                CliPrint("set vendor info:%x\r\n", vendor_info);
                rwnx_send_rftest_req(rwnx_hw, SET_VENDOR_INFO, 1, &vendor_info, &cfm);
                //memcpy(command, &cfm.rftest_result[0], 1);
                bytes_written = 1;
                CliPrint("done: get_vendor_info = 0x%x\r\n", (unsigned int)cfm.rftest_result[0]);
            }
            else if (strcasecmp(argv[0], "GET_VENDOR_INFO") == 0)
            {
                CliPrint("get vendor info\r\n");
                rwnx_send_rftest_req(rwnx_hw, GET_VENDOR_INFO, 0, NULL, &cfm);
                //memcpy(command, &cfm.rftest_result[0], 1);
                bytes_written = 1;
                CliPrint("done: get_vendor_info = 0x%x\r\n", (unsigned int)cfm.rftest_result[0]);
            }
            else if (strcasecmp(argv[0], "GET_FREQ_CAL") == 0)
            {
                CliPrint("get freq cal\r\n");
                rwnx_send_rftest_req(rwnx_hw, GET_FREQ_CAL, 0, NULL, &cfm);
                //memcpy(command, &cfm.rftest_result[0], 4);
                bytes_written = 4;
                CliPrint("done: get_freq_cal: xtal_cap=0x%x (efuse remain:%d), xtal_cap_fine=0x%x (efuse remain:%d)\r\n",
                         cfm.rftest_result[0] & 0x000000ff, (cfm.rftest_result[0] >> 16) & 0x000000ff,
                         (cfm.rftest_result[0] >> 8) & 0x000000ff, (cfm.rftest_result[0] >> 24) & 0x000000ff);
            }
            else if (strcasecmp(argv[0], "RDWR_PWRMM") == 0)
            {
                CliPrint("read/write txpwr manul mode\r\n");
                if (argc <= 1)   // read cur
                {
                    rwnx_send_rftest_req(rwnx_hw, RDWR_PWRMM, 0, NULL, &cfm);
                }
                else     // write
                {
                    u8_l pwrmm = (u8_l)command_strtoul(argv[1], NULL, 16);
                    pwrmm = (pwrmm) ? 1 : 0;
                    CliPrint("set pwrmm = %x\r\n", pwrmm);
                    rwnx_send_rftest_req(rwnx_hw, RDWR_PWRMM, sizeof(pwrmm), (u8_l *)&pwrmm, &cfm);
                }
                //memcpy(command, &cfm.rftest_result[0], 4);
                bytes_written = 4;
                CliPrint("done: txpwr manual mode = %x\r\n", (unsigned int)cfm.rftest_result[0]);
            }
            else if (strcasecmp(argv[0], "RDWR_PWRIDX") == 0)
            {
                u8_l func = 0;
                CliPrint("read/write txpwr index\r\n");
                if (argc > 1)
                {
                    func = (u8_l)command_strtoul(argv[1], NULL, 16);
                }
                if (func == 0)   // read cur
                {
                    rwnx_send_rftest_req(rwnx_hw, RDWR_PWRIDX, 0, NULL, &cfm);
                }
                else if (func <= 2)     // write 2.4g/5g pwr idx
                {
                    if (argc > 3)
                    {
                        u8_l type = (u8_l)command_strtoul(argv[2], NULL, 16);
                        u8_l pwridx = (u8_l)command_strtoul(argv[3], NULL, 10);
                        u8_l buf[3] = {func, type, pwridx};
                        CliPrint("set pwridx:[%x][%x]=%x\r\n", func, type, pwridx);
                        rwnx_send_rftest_req(rwnx_hw, RDWR_PWRIDX, sizeof(buf), buf, &cfm);
                    }
                    else
                    {
                        CliPrint("wrong args\r\n");
                    }
                }
                else
                {
                    CliPrint("wrong func: %x\r\n", func);
                }
                //memcpy(command, &cfm.rftest_result[0], 9);
                bytes_written = 9;
                char *buff = (char *)&cfm.rftest_result[0];
                CliPrint("done:\r\n");
                CliPrint("txpwr idx 2.4g:\r\n"
                         " [0]=%d(ofdmlowrate)\r\n"
                         " [1]=%d(ofdm64qam)\r\n"
                         " [2]=%d(ofdm256qam)\r\n"
                         " [3]=%d(ofdm1024qam)\r\n"
                         " [4]=%d(dsss)\r\n", (int8_t)buff[0], (int8_t)buff[1], (int8_t)buff[2], (int8_t)buff[3], (int8_t)buff[4]);
                CliPrint("txpwr idx 5g:\r\n"
                         " [0]=%d(ofdmlowrate)\r\n"
                         " [1]=%d(ofdm64qam)\r\n"
                         " [2]=%d(ofdm256qam)\r\n"
                         " [3]=%d(ofdm1024qam)\r\n", (int8_t)buff[5], (int8_t)buff[6], (int8_t)buff[7], (int8_t)buff[8]);
            }
            else if (strcasecmp(argv[0], "RDWR_PWRLVL") == 0)
            {
                u8_l func = 0;
                CliPrint("read/write txpwr level\r\n");
                if (argc > 1)
                {
                    func = (u8_l)command_strtoul(argv[1], NULL, 16);
                }
                if (func == 0)   // read cur
                {
                    rwnx_send_rftest_req(rwnx_hw, RDWR_PWRLVL, 0, NULL, &cfm);
                }
                else if (func <= 2)     // write 2.4g/5g pwr lvl
                {
                    if (argc > 4)
                    {
                        u8_l grp = (u8_l)command_strtoul(argv[2], NULL, 16);
                        u8_l idx, size;
                        u8_l buf[14] = {func, grp,};
                        if (argc > 12)   // set all grp
                        {
                            CliPrint("set pwrlvl %s:\r\n"
                                     "  [%x] =", (func == 1) ? "2.4g" : "5g", grp);
                            if (grp == 1)   // TXPWR_LVL_GRP_11N_11AC
                            {
                                size = 10;
                            }
                            else
                            {
                                size = 12;
                            }
                            for (idx = 0; idx < size; idx++)
                            {
                                s8_l pwrlvl = (s8_l)command_strtoul(argv[3 + idx], NULL, 10);
                                buf[2 + idx] = (u8_l)pwrlvl;
                                if (idx && !(idx & 0x3))
                                {
                                    CliPrint(" ");
                                }
                                CliPrint(" %2d", pwrlvl);
                            }
                            CliPrint("\r\n");
                            size += 2;
                        }
                        else     // set grp[idx]
                        {
                            u8_l idx = (u8_l)command_strtoul(argv[3], NULL, 10);
                            s8_l pwrlvl = (s8_l)command_strtoul(argv[4], NULL, 10);
                            buf[2] = idx;
                            buf[3] = (u8_l)pwrlvl;
                            size = 4;
                            CliPrint("set pwrlvl %s:\r\n"
                                     "  [%x][%d] = %d\r\n", (func == 1) ? "2.4g" : "5g", grp, idx, pwrlvl);
                        }
                        rwnx_send_rftest_req(rwnx_hw, RDWR_PWRLVL, size, buf, &cfm);
                    }
                    else
                    {
                        CliPrint("wrong args\r\n");
                        bytes_written = -EINVAL;
                        break;
                    }
                }
                else
                {
                    CliPrint("wrong func: %x\r\n", func);
                    bytes_written = -EINVAL;
                    break;
                }
                //memcpy(command, &cfm.rftest_result[0], 3 * 12);
                bytes_written = 3 * 12;
                char *buff = (char *)&cfm.rftest_result[0];
                int grp, idx;
                CliPrint("done:\r\n"
                         "txpwr level 2.4g: [0]:11b+11a/g, [1]:11n/11ac, [2]:11ax\r\n");
#if 0
                for (grp = 0; grp < 3; grp++)
                {
                    int cnt = 12;
                    if (grp == 1)
                    {
                        cnt = 10;
                    }
                    CliPrint("  [%x] =", grp);
                    for (idx = 0; idx < cnt; idx++)
                    {
                        if (idx && !(idx & 0x3))
                        {
                            CliPrint(" ");
                        }
                        CliPrint(" %2d", buff[12 * grp + idx]);
                    }
                    CliPrint("\r\n");
                }
#endif
                for (grp = 0; grp < 3; grp++)
                {
                    if (grp == 1)   // 10
                    {
                        CliPrint("  [%x] = %2d %2d %2d %2d  %2d %2d %2d %2d  %2d %2d\r\n", grp,
                                 buff[12 * grp + 0], buff[12 * grp + 1], buff[12 * grp + 2], buff[12 * grp + 3],
                                 buff[12 * grp + 4], buff[12 * grp + 5], buff[12 * grp + 6], buff[12 * grp + 7],
                                 buff[12 * grp + 8], buff[12 * grp + 9]);
                    }
                    else            // 12
                    {
                        CliPrint("  [%x] = %2d %2d %2d %2d  %2d %2d %2d %2d  %2d %2d %2d %2d\r\n", grp,
                                 buff[12 * grp + 0], buff[12 * grp + 1], buff[12 * grp + 2], buff[12 * grp + 3],
                                 buff[12 * grp + 4], buff[12 * grp + 5], buff[12 * grp + 6], buff[12 * grp + 7],
                                 buff[12 * grp + 8], buff[12 * grp + 9], buff[12 * grp + 10], buff[12 * grp + 11]);
                    }
                }
                if ((rwnx_hw->chipid != PRODUCT_ID_AIC8800DC) &&
                        (rwnx_hw->chipid != PRODUCT_ID_AIC8800DW))
                {
                    CliPrint("txpwr level 5g: [0]:11a, [1]:11n/11ac, [2]:11ax\n");
                    for (grp = 0; grp < 3; grp++)
                    {
                        if (grp == 1)   // 10
                        {
                            CliPrint("  [%x] = %2d %2d %2d %2d  %2d %2d %2d %2d  %2d %2d\r\n", grp,
                                     buff[12 * grp + 0], buff[12 * grp + 1], buff[12 * grp + 2], buff[12 * grp + 3],
                                     buff[12 * grp + 4], buff[12 * grp + 5], buff[12 * grp + 6], buff[12 * grp + 7],
                                     buff[12 * grp + 8], buff[12 * grp + 9]);
                        }
                        else            // 12
                        {
                            CliPrint("  [%x] = %2d %2d %2d %2d  %2d %2d %2d %2d  %2d %2d %2d %2d\r\n", grp,
                                     buff[12 * grp + 0], buff[12 * grp + 1], buff[12 * grp + 2], buff[12 * grp + 3],
                                     buff[12 * grp + 4], buff[12 * grp + 5], buff[12 * grp + 6], buff[12 * grp + 7],
                                     buff[12 * grp + 8], buff[12 * grp + 9], buff[12 * grp + 10], buff[12 * grp + 11]);
                        }
                    }
                }
            }
            else if (strcasecmp(argv[0], "RDWR_PWROFST") == 0)
            {
                u8_l func = 0;
                CliPrint("read/write txpwr offset\r\n");
                if (argc > 1)
                {
                    func = (u8_l)command_strtoul(argv[1], NULL, 16);
                }
                if (func == 0)   // read cur
                {
                    rwnx_send_rftest_req(rwnx_hw, RDWR_PWROFST, 0, NULL, &cfm);
                }
                else if (func <= 2)     // write 2.4g/5g pwr ofst
                {
                    if ((argc > 4) &&
                            ((rwnx_hw->chipid == PRODUCT_ID_AIC8800D80) ||
                             (rwnx_hw->chipid == PRODUCT_ID_AIC8800D81)))
                    {
                        u8_l type = (u8_l)command_strtoul(argv[2], NULL, 16);
                        u8_l chgrp = (u8_l)command_strtoul(argv[3], NULL, 16);
                        s8_l pwrofst = (u8_l)command_strtoul(argv[4], NULL, 10);
                        u8_l buf[4] = {func, type, chgrp, (u8_l)pwrofst};
                        CliPrint("set pwrofst_%s:[%x][%x]=%d\r\n", (func == 1) ? "2.4g" : "5g", type, chgrp, pwrofst);
                        rwnx_send_rftest_req(rwnx_hw, RDWR_PWROFST, sizeof(buf), buf, &cfm);
                    }
                    else if ((argc > 3) &&
                             ((rwnx_hw->chipid == PRODUCT_ID_AIC8800DC) ||
                              (rwnx_hw->chipid == PRODUCT_ID_AIC8800DW) ||
                              (rwnx_hw->chipid == PRODUCT_ID_AIC8801)))
                    {
                        u8_l chgrp = (u8_l)command_strtoul(argv[2], NULL, 16);
                        s8_l pwrofst = (u8_l)command_strtoul(argv[3], NULL, 10);
                        u8_l buf[3] = {func, chgrp, (u8_l)pwrofst};
                        CliPrint("set pwrofst_%s:[%x]=%d\r\n", (func == 1) ? "2.4g" : "5g", chgrp, pwrofst);
                        rwnx_send_rftest_req(rwnx_hw, RDWR_PWROFST, sizeof(buf), buf, &cfm);
                    }
                    else
                    {
                        CliPrint("wrong args\r\n");
                    }
                }
                else
                {
                    CliPrint("wrong func: %x\r\n", func);
                }
                char *buff = (char *)&cfm.rftest_result[0];
                if (rwnx_hw->chipid == PRODUCT_ID_AIC8801)
                {
                    bytes_written = 7;
                    CliPrint("done:\r\n"
                             "txpwr offset 2.4g:\r\n"
                             "  [0]=%d(ch1~4)\r\n"
                             "  [1]=%d(ch5~9)\r\n"
                             "  [2]=%d(ch10~13)\r\n", (int8_t)buff[0], (int8_t)buff[1], (int8_t)buff[2]);
                    CliPrint("txpwr offset 5g:\r\n"
                             "  [0]=%d(ch36~64)\r\n"
                             "  [1]=%d(ch100~120)\r\n"
                             "  [2]=%d(ch122~140)\r\n"
                             "  [3]=%d(ch142~165)\r\n", (int8_t)buff[3], (int8_t)buff[4], (int8_t)buff[5], (int8_t)buff[6]);
                }
                else if ((rwnx_hw->chipid == PRODUCT_ID_AIC8800DC) ||
                         (rwnx_hw->chipid == PRODUCT_ID_AIC8800DW))
                {
                    bytes_written = 3;
                    CliPrint("done:\r\n"
                             "txpwr offset 2.4g:\r\n"
                             "  [0]=%d(ch1~4)\r\n"
                             "  [1]=%d(ch5~9)\r\n"
                             "  [2]=%d(ch10~13)\r\n", (int8_t)buff[0], (int8_t)buff[1], (int8_t)buff[2]);
                }
                else
                {
                    int type;
                    bytes_written = 3 * 3 + 3 * 6;
                    CliPrint("done:\r\n"
                             "txpwr offset 2.4g:\r\n");
                    CliPrint("  chan=" "\t1~4" "\t5~9" "\t10~13\r\n");
                    for (type = 0; type < 3; type++)
                    {
                        CliPrint("  [%d] = " "\t%d" "\t%d" "\t%d\r\n",
                                 type, (int8_t)buff[type * 3 + 0], (int8_t)buff[type * 3 + 1], (int8_t)buff[type * 3 + 2]);
                    }
                    CliPrint("txpwr offset 5g:\r\n");
                    CliPrint("  chan=" "\t36~50" "\t51-64" "\t98-114" "\t115-130" "\t131-146" "\t147-166\r\n");
                    buff = (char *)&cfm.rftest_result[3 * 3];
                    for (type = 0; type < 3; type++)
                    {
                        CliPrint("  [%d] = " "\t%d" "\t%d" "\t%d" "\t%d" "\t%d" "\t%d\r\n",
                                 type, (int8_t)buff[type * 6 + 0], (int8_t)buff[type * 6 + 1], (int8_t)buff[type * 6 + 2],
                                 (int8_t)buff[type * 6 + 3], (int8_t)buff[type * 6 + 4], (int8_t)buff[type * 6 + 5]);
                    }
                }
            }
            else if (strcasecmp(argv[0], "RDWR_DRVIBIT") == 0)
            {
                u8_l func = 0;
                CliPrint("read/write pa drv_ibit\r\n");
                if (argc > 1)
                {
                    func = (u8_l)command_strtoul(argv[1], NULL, 16);
                }
                if (func == 0)   // read cur
                {
                    rwnx_send_rftest_req(rwnx_hw, RDWR_DRVIBIT, 0, NULL, &cfm);
                }
                else if (func == 1)     // write 2.4g pa drv_ibit
                {
                    if (argc > 2)
                    {
                        u8_l ibit = (u8_l)command_strtoul(argv[2], NULL, 16);
                        u8_l buf[2] = {func, ibit};
                        CliPrint("set drvibit:[%x]=%x\r\n", func, ibit);
                        rwnx_send_rftest_req(rwnx_hw, RDWR_DRVIBIT, sizeof(buf), buf, &cfm);
                    }
                    else
                    {
                        CliPrint("wrong args\r\n");
                    }
                }
                else
                {
                    CliPrint("wrong func: %x\r\n", func);
                }
                //memcpy(command, &cfm.rftest_result[0], 16);
                bytes_written = 16;
                char *buff = (char *)&cfm.rftest_result[0];
                CliPrint("done: 2.4g txgain tbl pa drv_ibit:\r\n");
                int idx;
                for (idx = 0; idx < 16; idx++)
                {
                    printf(" %x", buff[idx]);
                    if (!((idx + 1) & 0x03))
                    {
                        printf(" [%x~%x]\r\n", idx - 3, idx);
                    }
                }
            }
            else if (strcasecmp(argv[0], "RDWR_EFUSE_PWROFST") == 0)
            {
                u8_l func = 0;
                CliPrint("read/write txpwr offset into efuse\r\n");
                if (argc > 1)
                {
                    func = (u8_l)command_strtoul(argv[1], NULL, 16);
                }
                if (func == 0)   // read cur
                {
                    rwnx_send_rftest_req(rwnx_hw, RDWR_EFUSE_PWROFST, 0, NULL, &cfm);
                }
                else if (func <= 2)     // write 2.4g/5g pwr ofst
                {
                    if ((argc > 4) && (rwnx_hw->chipid == PRODUCT_ID_AIC8800D80))
                    {
                        u8_l type = (u8_l)command_strtoul(argv[2], NULL, 16);
                        u8_l chgrp = (u8_l)command_strtoul(argv[3], NULL, 16);
                        s8_l pwrofst = (u8_l)command_strtoul(argv[4], NULL, 10);
                        u8_l buf[4] = {func, type, chgrp, (u8_l)pwrofst};
                        CliPrint("set efuse pwrofst_%s:[%x][%x]=%d\r\n", (func == 1) ? "2.4g" : "5g", type, chgrp, pwrofst);
                        rwnx_send_rftest_req(rwnx_hw, RDWR_EFUSE_PWROFST, sizeof(buf), buf, &cfm);
                    }
                    else if ((argc > 3) && (rwnx_hw->chipid != PRODUCT_ID_AIC8800D80))
                    {
                        u8_l chgrp = (u8_l)command_strtoul(argv[2], NULL, 16);
                        s8_l pwrofst = (u8_l)command_strtoul(argv[3], NULL, 10);
                        u8_l buf[3] = {func, chgrp, (u8_l)pwrofst};
                        CliPrint("set efuse pwrofst_%s:[%x]=%d\r\n", (func == 1) ? "2.4g" : "5g", chgrp, pwrofst);
                        rwnx_send_rftest_req(rwnx_hw, RDWR_EFUSE_PWROFST, sizeof(buf), buf, &cfm);
                    }
                    else
                    {
                        CliPrint("wrong args\r\n");
                        bytes_written = -EINVAL;
                        break;
                    }
                }
                else
                {
                    CliPrint("wrong func: %x\r\n", func);
                }
                //memcpy(command, &cfm.rftest_result[0], 7);
                bytes_written = 7;
                char *buff = (char *)&cfm.rftest_result[0];
                if (rwnx_hw->chipid == PRODUCT_ID_AIC8801)
                {
#if 1//defined(CONFIG_AIC8801)
                    CliPrint("done:\r\n"
                             "efuse txpwr offset 2.4g:\r\n"
                             "  [0]=%d(ch1~4)\r\n"
                             "  [1]=%d(ch5~9)\r\n"
                             "  [2]=%d(ch10~13)\r\n", (int8_t)buff[0], (int8_t)buff[1], (int8_t)buff[2]);
                    CliPrint("efuse txpwr offset 5g:\r\n"
                             "  [0]=%d(ch36~64)\r\n"
                             "  [1]=%d(ch100~120)\r\n"
                             "  [2]=%d(ch122~140)\r\n"
                             "  [3]=%d(ch142~165)\r\n", (int8_t)buff[3], (int8_t)buff[4], (int8_t)buff[5], (int8_t)buff[6]);
#endif /* CONFIG_AIC8801 */
                }
                else if (rwnx_hw->chipid == PRODUCT_ID_AIC8800D80)
                {
                    int type, ch_grp;
                    CliPrint("done:\n"
                             "pwrofst2x 2.4g: [0]:11b, [1]:ofdm_highrate, [2]:ofdm_lowrate\n"
                             "  chan=" "\t1-4" "\t5-9" "\t10-13");
                    for (type = 0; type < 3; type++)
                    {
                        CliPrint("\n  [%d] =", type);
                        for (ch_grp = 0; ch_grp < 3; ch_grp++)
                        {
                            CliPrint("\t%d", buff[3 * type + ch_grp]);
                        }
                    }
                    CliPrint("\npwrofst2x 5g: [0]:ofdm_lowrate, [1]:ofdm_highrate, [2]:ofdm_midrate\n"
                             "  chan=" "\t36-50" "\t51-64" "\t98-114" "\t115-130" "\t131-146" "\t147-166");
                    buff = (signed char *)&cfm.rftest_result[3 * 3];
                    for (type = 0; type < 3; type++)
                    {
                        CliPrint("\n  [%d] =", type);
                        for (ch_grp = 0; ch_grp < 6; ch_grp++)
                        {
                            CliPrint("\t%d", buff[6 * type + ch_grp]);
                        }
                    }
                    CliPrint("\n");
                }
                else if (rwnx_hw->chipid == PRODUCT_ID_AIC8800DC || rwnx_hw->chipid == PRODUCT_ID_AIC8800DW)
                {
                    CliPrint("done:\n"
                             "efuse txpwr offset 2.4g:\n"
                             "  [0]=%d(remain:%x, ch1~4)\n"
                             "  [1]=%d(remain:%x, ch5~9)\n"
                             "  [2]=%d(remain:%x, ch10~13)\n",
                             (int8_t)buff[0], (int8_t)buff[3],
                             (int8_t)buff[1], (int8_t)buff[4],
                             (int8_t)buff[2], (int8_t)buff[5]);
                }
            }
            else if (strcasecmp(argv[0], "RDWR_EFUSE_DRVIBIT") == 0)
            {
                u8_l func = 0;
                CliPrint("read/write pa drv_ibit into efuse\r\n");
                if (argc > 1)
                {
                    func = (u8_l)command_strtoul(argv[1], NULL, 16);
                }
                if (func == 0)   // read cur
                {
                    rwnx_send_rftest_req(rwnx_hw, RDWR_EFUSE_DRVIBIT, 0, NULL, &cfm);
                }
                else if (func == 1)     // write 2.4g pa drv_ibit
                {
                    if (argc > 2)
                    {
                        u8_l ibit = (u8_l)command_strtoul(argv[2], NULL, 16);
                        u8_l buf[2] = {func, ibit};
                        CliPrint("set efuse drvibit:[%x]=%x\r\n", func, ibit);
                        rwnx_send_rftest_req(rwnx_hw, RDWR_EFUSE_DRVIBIT, sizeof(buf), buf, &cfm);
                    }
                    else
                    {
                        CliPrint("wrong args\r\n");
                    }
                }
                else
                {
                    CliPrint("wrong func: %x\r\n", func);
                }
                //memcpy(command, &cfm.rftest_result[0], 4);
                bytes_written = 4;
                CliPrint("done: efsue 2.4g txgain tbl pa drv_ibit: %x\r\n", cfm.rftest_result[0]);
            }
            else if (strcasecmp(argv[0], "SET_PAPR") == 0)
            {
                CliPrint("set papr\r\n");
                if (argc > 1)
                {
                    u8_l func = (u8_l) command_strtoul(argv[1], NULL, 10);
                    CliPrint("papr %d\r\n", func);
#ifdef CONFIG_SDIO_SUPPORT
                    rwnx_send_rftest_req(rwnx_hw, SET_PAPR, sizeof(func), &func, NULL);
#endif
                }
                else
                {
                    CliPrint("wrong args\r\n");
                    bytes_written = -EINVAL;
                    break;
                }
            }
            else if (strcasecmp(argv[0], "SET_COB_CAL") == 0)
            {
                CliPrint("set_cob_cal\r\n");
                if (argc < 3)
                {
                    CliPrint("wrong param\r\n");
                    bytes_written = -EINVAL;
                    break;
                }
                setcob_cal.dutid = command_strtoul(argv[1], NULL, 10);
                setcob_cal.chip_num = command_strtoul(argv[2], NULL, 10);
                setcob_cal.dis_xtal = command_strtoul(argv[3], NULL, 10);
                rwnx_send_rftest_req(rwnx_hw, SET_COB_CAL, sizeof(cmd_rf_setcobcal_t), (u8_l *)&setcob_cal, NULL);
            }
            else if (strcasecmp(argv[0], "GET_COB_CAL_RES") == 0)
            {
                CliPrint("get cob cal res\r\n");
                rwnx_send_rftest_req(rwnx_hw, GET_COB_CAL_RES, 0, NULL, &cfm);
                memcpy(command, &cfm.rftest_result[0], 4);
                bytes_written = 4;
                CliPrint("cap=0x%x, cap_fine=0x%x\r\n", cfm.rftest_result[0] & 0x0000ffff, (cfm.rftest_result[0] >> 16) & 0x0000ffff);
            }
            else if (strcasecmp(argv[0], "SET_NOTCH") == 0)
            {
                if (argc > 1)
                {
                    u8_l func = (u8_l) command_strtoul(argv[1], NULL, 10);
                    CliPrint("set notch %d\r\n", func);
#ifdef CONFIG_SDIO_SUPPORT
                    rwnx_send_rftest_req(rwnx_hw, SET_NOTCH, sizeof(func), &func, NULL);
#endif
                }
                else
                {
                    CliPrint("wrong args\r\n");
                    bytes_written = -EINVAL;
                    break;
                }
            }
            else if (strcasecmp(argv[0], "RDWR_PWROFSTFINE") == 0)
            {
                u8_l func = 0;
                CliPrint("read/write txpwr offset fine\r\n");
                if (argc > 1)
                {
                    func = (u8_l)command_strtoul(argv[1], NULL, 16);
                }
                if (func == 0)   // read cur
                {
                    rwnx_send_rftest_req(rwnx_hw, RDWR_PWROFSTFINE, 0, NULL, &cfm);
                }
                else if (func <= 2)     // write 2.4g/5g pwr ofst
                {
                    if (argc > 3)
                    {
                        u8_l chgrp = (u8_l)command_strtoul(argv[2], NULL, 16);
                        s8_l pwrofst = (u8_l)command_strtoul(argv[3], NULL, 10);
                        u8_l buf[3] = {func, chgrp, (u8_l)pwrofst};
                        CliPrint("set pwrofstfine:[%x][%x]=%d\r\n", func, chgrp, pwrofst);
                        rwnx_send_rftest_req(rwnx_hw, RDWR_PWROFSTFINE, sizeof(buf), buf, &cfm);
                    }
                    else
                    {
                        CliPrint("wrong args\r\n");
                        bytes_written = -EINVAL;
                        break;
                    }
                }
                else
                {
                    CliPrint("wrong func: %x\r\n", func);
                    bytes_written = -EINVAL;
                    break;
                }
                //memcpy(command, &cfm.rftest_result[0], 7);
                bytes_written = 7;
                char *buff = (char *)&cfm.rftest_result[0];
                CliPrint("done:\r\n"
                         "txpwr offset 2.4g:\r\n"
                         "  [0]=%d(ch1~4)\r\n"
                         "  [1]=%d(ch5~9)\r\n"
                         "  [2]=%d(ch10~13)\r\n", (int8_t)buff[0], (int8_t)buff[1], (int8_t)buff[2]);
                CliPrint("txpwr offset 5g:\r\n"
                         "  [0]=%d(ch36~64)\r\n"
                         "  [1]=%d(ch100~120)\r\n"
                         "  [2]=%d(ch122~140)\r\n"
                         "  [3]=%d(ch142~165)\r\n", (int8_t)buff[3], (int8_t)buff[4], (int8_t)buff[5], (int8_t)buff[6]);
            }
            else if (strcasecmp(argv[0], "RDWR_EFUSE_PWROFSTFINE") == 0)
            {
                u8_l func = 0;
                CliPrint("read/write txpwr offset fine into efuse\r\n");
                if (argc > 1)
                {
                    func = (u8_l)command_strtoul(argv[1], NULL, 16);
                }
                if (func == 0)   // read cur
                {
                    rwnx_send_rftest_req(rwnx_hw, RDWR_EFUSE_PWROFSTFINE, 0, NULL, &cfm);
                }
                else if (func <= 2)     // write 2.4g/5g pwr ofst
                {
                    if (argc > 3)
                    {
                        u8_l chgrp = (u8_l)command_strtoul(argv[2], NULL, 16);
                        s8_l pwrofst = (u8_l)command_strtoul(argv[3], NULL, 10);
                        u8_l buf[3] = {func, chgrp, (u8_l)pwrofst};
                        CliPrint("set efuse pwrofstfine:[%x][%x]=%d\r\n", func, chgrp, pwrofst);
                        rwnx_send_rftest_req(rwnx_hw, RDWR_EFUSE_PWROFSTFINE, sizeof(buf), buf, &cfm);
                    }
                    else
                    {
                        CliPrint("wrong args\r\n");
                        bytes_written = -EINVAL;
                        break;
                    }
                }
                else
                {
                    CliPrint("wrong func: %x\r\n", func);
                    bytes_written = -EINVAL;
                    break;
                }
                if ((rwnx_hw->chipid == PRODUCT_ID_AIC8800DC) ||
                        (rwnx_hw->chipid == PRODUCT_ID_AIC8800DW))   // 6 = 3 (2.4g) * 2
                {
                    //memcpy(command, &cfm.rftest_result[0], 6);
                    bytes_written = 6;
                    char *buff = (char *)&cfm.rftest_result[0];
                    CliPrint("efuse txpwr offset fine 2.4g:\r\n"
                             "  [0]=%d(remain:%x, ch1~4)\r\n"
                             "  [1]=%d(remain:%x, ch5~9)\r\n"
                             "  [2]=%d(remain:%x, ch10~13)\r\n",
                             (int8_t)buff[0], (int8_t)buff[3],
                             (int8_t)buff[1], (int8_t)buff[4],
                             (int8_t)buff[2], (int8_t)buff[5]);
                }
                else     // 7 = 3(2.4g) + 4(5g)
                {
                    //memcpy(command, &cfm.rftest_result[0], 7);
                    //bytes_written = 7;
                    // 5G need to check!!!
                    bytes_written = 14;
                    char *buff = (char *)&cfm.rftest_result[0];
                    CliPrint("done:\r\n"
                             "efuse txpwr offset fine 2.4g:\r\n"
                             "  [0]=%d(remain:%x, ch1~4)\r\n"
                             "  [1]=%d(remain:%x, ch5~9)\r\n"
                             "  [2]=%d(remain:%x, ch10~13)\r\n",
                             (int8_t)buff[0], (int8_t)buff[3],
                             (int8_t)buff[1], (int8_t)buff[4],
                             (int8_t)buff[2], (int8_t)buff[5]);
                    CliPrint("efuse txpwr offset fine 5g:\r\n"
                             "  [0]=%d(remain:%x, ch36~64)\r\n"
                             "  [1]=%d(remain:%x, ch100~120)\r\n"
                             "  [2]=%d(remain:%x, ch122~140)\r\n"
                             "  [3]=%d(remain:%x, ch142~165)\r\n",
                             (int8_t)buff[6], (int8_t)buff[10],
                             (int8_t)buff[7], (int8_t)buff[11],
                             (int8_t)buff[8], (int8_t)buff[12],
                             (int8_t)buff[9], (int8_t)buff[13]);
                }
            }
            else if (strcasecmp(argv[0], "RDWR_EFUSE_SDIOCFG") == 0)
            {
                u8_l func = 0;
                CliPrint("read/write sdiocfg_bit into efuse\r\n");
                if (argc > 1)
                {
                    func = (u8_l)command_strtoul(argv[1], NULL, 16);
                }
                if (func == 0)   // read cur
                {
                    rwnx_send_rftest_req(rwnx_hw, RDWR_EFUSE_SDIOCFG, 0, NULL, &cfm);
                }
                else if (func == 1)     // write sdiocfg
                {
                    if (argc > 2)
                    {
                        u8_l ibit = (u8_l)command_strtoul(argv[2], NULL, 16);
                        u8_l buf[2] = {func, ibit};
                        CliPrint("set efuse sdiocfg:[%x]=%x\r\n", func, ibit);
                        rwnx_send_rftest_req(rwnx_hw, RDWR_EFUSE_SDIOCFG, sizeof(buf), buf, &cfm);
                    }
                    else
                    {
                        CliPrint("wrong args\r\n");
                        bytes_written = -EINVAL;
                        break;
                    }
                }
                else
                {
                    CliPrint("wrong func: %x\r\n", func);
                    bytes_written = -EINVAL;
                    break;
                }
                //memcpy(command, &cfm.rftest_result[0], 4);
                bytes_written = 4;
                char *buff = (char *)&cfm.rftest_result[0];
                unsigned int val = (unsigned int)buff[0] |
                                   (unsigned int)(buff[1] <<  8) |
                                   (unsigned int)(buff[2] << 16) |
                                   (unsigned int)(buff[3] << 24);
                CliPrint("done: efsue usb vid/pid: %x\\r\n", val);
            }
            else if (strcasecmp(argv[0], "SET_SRRC") == 0)
            {
                if (argc > 1)
                {
                    u8_l func = (u8_l) command_strtoul(argv[1], NULL, 10);
                    CliPrint("set srrc %d\r\n", func);
#ifdef CONFIG_SDIO_SUPPORT
                    rwnx_send_rftest_req(rwnx_hw, SET_SRRC, sizeof(func), &func, NULL);
#endif
                }
                else
                {
                    CliPrint("wrong args\r\n");
                    bytes_written = -EINVAL;
                    break;
                }
            }
            else if (strcasecmp(argv[0], "SET_FSS") == 0)
            {
                if (argc > 1)
                {
                    u8_l func = (u8_l) command_strtoul(argv[1], NULL, 10);
                    CliPrint("set fss: %d\r\n", func);
#ifdef CONFIG_SDIO_SUPPORT
                    rwnx_send_rftest_req(rwnx_hw, SET_FSS, sizeof(func), &func, NULL);
#endif
                }
                else
                {
                    CliPrint("wrong args\r\n");
                    bytes_written = -EINVAL;
                    break;
                }
            }
            else if (strcasecmp(argv[0], "RDWR_EFUSE_HE_OFF") == 0)
            {
                if (argc > 1)
                {
                    u8_l func = command_strtoul(argv[1], NULL, 10);
                    CliPrint("set he off: %d\r\n", func);
                    rwnx_send_rftest_req(rwnx_hw, RDWR_EFUSE_HE_OFF, sizeof(func), (u8_l *)&func, &cfm);
                    CliPrint("he_off cfm: %d\r\n", cfm.rftest_result[0]);
                    memcpy(command, &cfm.rftest_result[0], 4);
                    bytes_written = 4;
                }
                else
                {
                    CliPrint("wrong args\r\n");
                    bytes_written = -EINVAL;
                    break;
                }
            }

#ifdef CONFIG_USB_BT
            else if (strcasecmp(argv[0], "BT_RESET") == 0)
            {
                if (argc == 5)
                {
                    CliPrint("btrf reset\r\n");
                    for (bt_index = 0; bt_index < 4; bt_index++)
                    {
                        dh_cmd_reset[bt_index] = command_strtoul(argv[bt_index + 1], NULL, 16);
                        CliPrint("0x%x ", dh_cmd_reset[bt_index]);
                    }
                    CliPrint("\r\n");
                }
                else
                {
                    CliPrint("wrong param\r\n");
                    break;
                }
                rwnx_send_rftest_req(rwnx_hw, BT_RESET, sizeof(dh_cmd_reset), (u8_l *)&dh_cmd_reset, NULL);
            }
            else if (strcasecmp(argv[0], "BT_TXDH") == 0)
            {
                if (argc == 19)
                {
                    CliPrint("btrf txdh\r\n");
                    for (bt_index = 0; bt_index < 18; bt_index++)
                    {
                        dh_cmd_txdh[bt_index] = command_strtoul(argv[bt_index + 1], NULL, 16);
                        CliPrint("0x%x ", dh_cmd_txdh[bt_index]);
                    }
                    CliPrint("\r\n");
                }
                else
                {
                    CliPrint("wrong param\r\n");
                    break;
                }
                rwnx_send_rftest_req(rwnx_hw, BT_TXDH, sizeof(dh_cmd_txdh), (u8_l *)&dh_cmd_txdh, NULL);
            }
            else if (strcasecmp(argv[0], "BT_RXDH") == 0)
            {
                if (argc == 18)
                {
                    CliPrint("btrf rxdh\r\n");
                    for (bt_index = 0; bt_index < 17; bt_index++)
                    {
                        dh_cmd_rxdh[bt_index] = command_strtoul(argv[bt_index + 1], NULL, 16);
                        CliPrint("0x%x ", dh_cmd_rxdh[bt_index]);
                    }
                    CliPrint("\r\n");
                }
                else
                {
                    CliPrint("wrong param\r\n");
                    break;
                }
                rwnx_send_rftest_req(rwnx_hw, BT_RXDH, sizeof(dh_cmd_rxdh), (u8_l *)&dh_cmd_rxdh, NULL);
            }
            else if (strcasecmp(argv[0], "BT_STOP") == 0)
            {
                if (argc == 6)
                {
                    CliPrint("btrf stop\r\n");
                    for (bt_index = 0; bt_index < 5; bt_index++)
                    {
                        dh_cmd_stop[bt_index] = command_strtoul(argv[bt_index + 1], NULL, 16);
                        CliPrint("0x%x ", dh_cmd_stop[bt_index]);
                    }
                    CliPrint("\r\n");
                }
                else
                {
                    CliPrint("wrong param\r\n");
                    break;
                }
                rwnx_send_rftest_req(rwnx_hw, BT_STOP, sizeof(dh_cmd_stop), (u8_l *)&dh_cmd_stop, NULL);
            }
            else if (strcasecmp(argv[0], "GET_BT_RX_RESULT") == 0)
            {
                CliPrint("get_bt_rx_result\r\n");
                rwnx_send_rftest_req(rwnx_hw, GET_BT_RX_RESULT, 0, NULL, &cfm);
                //memcpy(command, &cfm.rftest_result[0], 12);
                bytes_written = 12;
                CliPrint("done: get bt rx total=%d, ok=%d, err=%d\r\n", (unsigned int)cfm.rftest_result[0], (unsigned int)cfm.rftest_result[1], (unsigned int)cfm.rftest_result[2]);
            }
#endif
            else
            {
                CliPrint("%s: wrong cmd(%s) in mode:%x\r\n", __func__, argv[0], rwnx_hw->mode);
            }
        }
        while (0);
    }
    else
    {
        u8_l sta_idx = FMAC_STA_MAX;
        u8_l bw;
        u8_l format_idx;
        u16_l rate_idx;
        u8_l pre_type;
        do
        {
            if (strcasecmp(argv[0], "TEST_CMD") == 0)
            {
                CliPrint("this is test cmd\r\n");
            }
            else if (strcasecmp(argv[0], "SCAN_OPEN") == 0)
            {
                CliPrint("scan open\r\n");

                wlan_if_scan_open();
            }
            else if (strcasecmp(argv[0], "SCAN") == 0)
            {
                CliPrint("scan\r\n");

                wlan_if_scan(NULL);
            }
            else if (strcasecmp(argv[0], "GET_SCAN") == 0)
            {
                CliPrint("get scan\r\n");
                wifi_ap_list_t ap_list;

                wlan_if_getscan(&ap_list);
            }
            else if (strcasecmp(argv[0], "SCAN_CLOSE") == 0)
            {
                CliPrint("scan close\r\n");

                wlan_if_scan_close();
            }
            else if (strcasecmp(argv[0], "STOP_STA") == 0)
            {
                CliPrint("stop sta\r\n");
                wlan_disconnect_sta(0);
            }
            else if (strcasecmp(argv[0], "START_STA") == 0)
            {
                CliPrint("start station\r\n");
                rwnx_hw->net_id = wlan_start_sta(NULL, NULL, -1);
#if 0
                if (argc == 2)
                {
                    CliPrint("connect unencrypted ap\n");
                    rwnx_hw->net_id = wlan_start_sta(argv[1], NULL, -1);
                }
                else if (argc == 3)
                {
                    CliPrint("connect encrypted ap\n");
                    rwnx_hw->net_id = wlan_start_sta(argv[1], argv[2], -1);
                }
                else
                {
                    CliPrint("wrong param\n");
                    break;
                }
#endif
            }
            else if (strcasecmp(argv[0], "CONN") == 0)
            {
                CliPrint("station connect\r\n");
                if (argc == 2)
                {
                    CliPrint("connect unencrypted ap\r\n");
                    wlan_sta_connect((uint8_t *)argv[1], NULL, -1);
                }
                else if (argc == 3)
                {
                    CliPrint("connect encrypted ap\r\n");
                    wlan_sta_connect((uint8_t *)argv[1], (uint8_t *)argv[2], -1);
                }
                else
                {
                    CliPrint("wrong param\r\n");
                    break;
                }
            }
            else if (strcasecmp(argv[0], "START_AP") == 0)
            {
                CliPrint("start ap\r\n");

#define AP_SSID_STRING  "AIC-AP"
#define AP_PASS_STRING  "00000000"
                struct aic_ap_cfg cfg;
                memset(&cfg, 0, sizeof(cfg));
                cfg.band = PHY_BAND_5G;
                cfg.channel = 149;
                cfg.type = PHY_CHNL_BW_40;
                cfg.max_inactivity = 60;
                cfg.enable_he = 1;
                cfg.bcn_interval = 100;
                cfg.sercurity_type = KEY_WPA2;
                cfg.sta_num = 10;
                memcpy(cfg.aic_ap_ssid.array, AP_SSID_STRING, strlen(AP_SSID_STRING));
                memcpy(cfg.aic_ap_passwd.array, AP_PASS_STRING, strlen(AP_PASS_STRING));
                cfg.aic_ap_ssid.length = strlen(AP_SSID_STRING);
                cfg.aic_ap_passwd.length = strlen(AP_PASS_STRING);
                aic_wifi_init(WIFI_MODE_AP, 0, &cfg);
            }
            else if (strcasecmp(argv[0], "STOP_AP") == 0)
            {
                CliPrint("stop ap\r\n");
                aic_wifi_deinit(WIFI_MODE_AP);
            }
            else if (strcasecmp(argv[0], "START_P2P") == 0)
            {
                int fvif_idx = 0;
                struct fhost_vif_p2p_cfg p2p_cfg = {{0,},};
                int ssid_len, pass_len;
                CliPrint("start p2p go\r\n");
                ssid_len = strlen(P2P_GO_SSID_STRING);
                memcpy(p2p_cfg.ssid.array, P2P_GO_SSID_STRING, ssid_len);
                p2p_cfg.ssid.length = ssid_len;
                p2p_cfg.ssid.array[ssid_len] = '\0';
                pass_len = strlen(P2P_GO_PASS_STRING);
                memcpy(p2p_cfg.key, P2P_GO_PASS_STRING, pass_len);
                p2p_cfg.key[pass_len] = '\0';
                if ((argc == 2) && (strncasecmp("5G", argv[1], strlen(argv[1])) == 0))
                {
                    p2p_cfg.chan.band = PHY_BAND_5G;
                    p2p_cfg.chan.type = PHY_CHNL_BW_40;
                    p2p_cfg.chan.prim20_freq = 5180;
                    p2p_cfg.chan.center1_freq = 5190;
                }
                else
                {
                    p2p_cfg.chan.band = PHY_BAND_2G4;
                    p2p_cfg.chan.type = PHY_CHNL_BW_20;
                    p2p_cfg.chan.prim20_freq = 2412;
                    p2p_cfg.chan.center1_freq = 2412;
                }
                p2p_cfg.chan.tx_power = 20;
                p2p_cfg.enable_he = 1;
                p2p_cfg.enable_acs = 0;
                wlan_start_p2p(fvif_idx, &p2p_cfg);
            }
            else if (strcasecmp(argv[0], "STOP_P2P") == 0)
            {
                int fvif_idx = 0;
                CliPrint("stop p2p go\r\n");
                wlan_stop_p2p(fvif_idx);
            }
            else if (strcasecmp(argv[0], "START_MON") == 0)
            {
                struct fhost_vif_tag *p_fvif = fhost_vif_get_first_free_itf();
                int fvif_idx;
                struct wifi_api_monitor_start_tag mon_start;
                struct fhost_vif_monitor_cfg *p_cfg = &mon_start.cfg;
                struct mac_chan_def *chan_def;
                struct mac_chan_op chan_op = {0,};
                if (p_fvif == NULL)
                {
                    DBG_APP_ERR("%s Fail to get free fvif\n", __func__);
                    break;
                }
                fvif_idx = fhost_vif_idx_from_vif_tag(p_fvif);
                if ((fvif_idx < 0) || (fvif_idx >= NX_VIRT_DEV_MAX))
                {
                    DBG_APP_ERR("invalid fvif=%p, idx=%d\n", p_fvif, fvif_idx);
                    break;
                }
                mon_start.hdr.len = sizeof(struct wifi_api_monitor_start_tag);
                mon_start.hdr.id  = WIFI_API_MONITOR_START;
                mon_start.fhost_vif_idx = fvif_idx;
                if ((argc == 2) && (strncasecmp("5G", argv[1], strlen(argv[1])) == 0))
                {
                    chan_def = fhost_chan_get(5180); // ch36
                }
                else
                {
                    chan_def = fhost_chan_get(2437); // ch6
                }
                chan_op.band = chan_def->band;
                chan_op.tx_power = chan_def->tx_power;
                chan_op.type = PHY_CHNL_BW_40;
                chan_op.prim20_freq  = chan_def->freq;
                chan_op.center1_freq = chan_def->freq + 10;
                p_cfg->chan = chan_op;
                p_cfg->uf = false;
                p_cfg->cb = rwnx_rx_monitor_cb;
                p_cfg->cb_arg = NULL;
                p_cfg->rx_filter = (
                                       acceptOtherDataFrames |
                                       acceptQCFWOData |
                                       acceptQData |
                                       acceptCFWOData |
                                       acceptData |
                                       acceptOtherCntrlFrames |
                                       acceptOtherMgmtFrames |
                                       acceptProbeResp |
                                       acceptProbeReq |
                                       acceptMyUnicast |
                                       //acceptUnicast |
                                       acceptOtherBSSID |
                                       acceptBroadcast |
                                       //acceptMulticast |
                                       0);
                wifi_api_msg_send(&mon_start);
            }
            else if (strcasecmp(argv[0], "STOP_MON") == 0)
            {
                struct fhost_vif_tag *p_fvif = &fhost_env.vif[0];
                int fvif_idx;
                struct wifi_api_monitor_stop_tag mon_stop;
                if (p_fvif == NULL)
                {
                    DBG_APP_ERR("%s null fvif\n", __func__);
                    break;
                }
                fvif_idx = fhost_vif_idx_from_vif_tag(p_fvif);
                if ((fvif_idx < 0) || (fvif_idx >= NX_VIRT_DEV_MAX))
                {
                    DBG_APP_ERR("invalid fvif=%p, idx=%d\n", p_fvif, fvif_idx);
                    break;
                }
                mon_stop.hdr.len = sizeof(struct wifi_api_monitor_stop_tag);
                mon_stop.hdr.id  = WIFI_API_MONITOR_STOP;
                mon_stop.fhost_vif_idx = fvif_idx;
                wifi_api_msg_send(&mon_stop);
            }
            else if (strcasecmp(argv[0], "ADD_BLACK") == 0)
            {
                CliPrint("Add Blacklist\r\n");
                struct mac_addr macaddr;
                uint8_t addr[] = {0xFE, 0x37, 0xB4, 0x23, 0x56, 0x4c};
                memcpy(&macaddr, addr, 6);
                wlan_ap_add_blacklist(&macaddr);
                wlan_ap_set_mac_acl_mode(1);
            }
            else if (strcasecmp(argv[0], "DEL_BLACK") == 0)
            {
                CliPrint("Del Blacklist\r\n");
                struct mac_addr macaddr;
                uint8_t addr[] = {0xFE, 0x37, 0xB4, 0x23, 0x56, 0x4c};
                memcpy(&macaddr, addr, 6);
                wlan_ap_delete_blacklist(&macaddr);
            }
            else if (strcasecmp(argv[0], "GET_ACL") == 0)
            {
                CliPrint("Get ACL list\r\n");
                uint8_t index, cnt = wlan_ap_get_mac_acl_list_cnt();
                void *list = wlan_ap_get_mac_acl_list();

                index = 0;
                struct co_list_hdr *list_hdr = co_list_pick(list);
                while (list_hdr != NULL)
                {
                    struct wifi_mac_node *marked_sta = (struct wifi_mac_node *)list_hdr;
                    CliPrint("ACL list[%d] = %02x:%02x:%02x:%02x:%02x:%02x\r\n", index, marked_sta->mac[0], marked_sta->mac[1], marked_sta->mac[2],
                             marked_sta->mac[3], marked_sta->mac[4], marked_sta->mac[5]);
                    list_hdr = co_list_next(list_hdr);
                    index ++;
                }
            }
            else if (strcasecmp(argv[0], "ADD_WHITE") == 0)
            {
                CliPrint("Add Whitelist\r\n");
                struct mac_addr macaddr;
                uint8_t addr[] = {0xFE, 0x37, 0xB4, 0x23, 0x56, 0x4c};
                memcpy(&macaddr, addr, 6);
                wlan_ap_add_whitelist(&macaddr);
                wlan_ap_set_mac_acl_mode(2);
            }
            else if (strcasecmp(argv[0], "DEL_WHITE") == 0)
            {
                CliPrint("Del Whitelist\r\n");
                struct mac_addr macaddr;
                uint8_t addr[] = {0xFE, 0x37, 0xB4, 0x23, 0x56, 0x4c};
                memcpy(&macaddr, addr, 6);
                wlan_ap_delete_whitelist(&macaddr);
            }
            else if (strcasecmp(argv[0], "ACL_DIS") == 0)
            {
                CliPrint("ACL disable\r\n");
                wlan_ap_set_mac_acl_mode(0);
            }
            else if (strcasecmp(argv[0], "GET_STAS") == 0)
            {
                CliPrint("Get stas\r\n");
                uint8_t index, cnt = wlan_ap_get_associated_sta_cnt();
                void *sta_list = wlan_ap_get_associated_sta_list();

                index = 0;
                struct co_list_hdr *list_hdr = co_list_pick(sta_list);
                while (list_hdr != NULL)
                {
                    struct sta_info_tag *sta = (struct sta_info_tag *)list_hdr;
                    CliPrint("STA[%d] = %x:%x:%x\r\n", index, sta->mac_addr.array[0], sta->mac_addr.array[1], sta->mac_addr.array[2]);
                    list_hdr = co_list_next(list_hdr);
                    index ++;
                }
            }
            else if (strcasecmp(argv[0], "GET_RSSI") == 0)
            {
                uint8_t addr[] = {0xFE, 0x37, 0xB4, 0x23, 0x56, 0x4c};
                CliPrint("RSSI[ %02x:%02x:%02x:%02x:%02x:%02x] = %d\r\n", addr[0], addr[1],
                         addr[2], addr[3], addr[4], addr[5], wlan_ap_get_associated_sta_rssi(addr));

            }
            else if (strcasecmp(argv[0], "SET_RATE") == 0)
            {
                if (argc == 5)
                {
                    CliPrint("set rate\r\n");
                    struct fhost_vif_tag *fhost_vif = &fhost_env.vif[0];
                    struct vif_info_tag *mac_vif = fhost_vif->mac_vif;
                    if (mac_vif->type == VIF_STA)
                    {
                        sta_idx = mac_vif->u.sta.ap_id;
                    }
                    else if (mac_vif->type == VIF_AP)     // for softAP, only support first sta
                    {
                        struct sta_info_tag *sta = (struct sta_info_tag *)co_list_pick(&mac_vif->sta_list);
                        if (sta->valid)
                        {
                            sta_idx = sta->staid;
                        }
                    }
                    if (sta_idx < FMAC_STA_MAX)
                    {
                        bw = command_strtoul(argv[1], NULL, 10);
                        format_idx = command_strtoul(argv[2], NULL, 10);
                        rate_idx = command_strtoul(argv[3], NULL, 10);
                        pre_type = command_strtoul(argv[4], NULL, 10);
                        CliPrint("sta_idx:%d, bw:%d, format_idx:%d, rate_idx:%d, pre_type:%d\r\n", sta_idx, bw, format_idx, rate_idx, pre_type);
                    }
                    else
                    {
                        CliPrint("no valid sta_idx\r\n");
                    }
                }
                else
                {
                    CliPrint("wrong param\r\n");
                    break;
                }
                fhost_cntrl_cfgrwnx_set_fixed_rate(sta_idx, bw, format_idx, rate_idx, pre_type);
            }
            else if (strcasecmp(argv[0], "SET_LP") == 0)
            {
                if (argc >= 2)
                {
                    uint8_t lp_level = (uint8_t)command_strtoul(argv[1], NULL, 10);
                    cmd_set_lp_level(lp_level);
                }
                else
                {
                    CliPrint("wrong param\r\n");
                }
            }
            else if (strcasecmp(argv[0], "ds") == 0)
            {
                cmd_dbg_severity_config(argc, argv);
            }
            else if (strcasecmp(argv[0], "dm") == 0)
            {
                cmd_dbg_module_config(argc, argv);
            }
            else
            {
                CliPrint("%s: wrong cmd(%s) in mode:%x\r\n", __func__, argv[0], rwnx_hw->mode);
            }
        }
        while (0);
    }
    return bytes_written;
}


static int p2p_go_start(int argc, char **argv)
{
    int fvif_idx = 0;
    struct fhost_vif_p2p_cfg p2p_cfg = {{0,},};
    int ssid_len, pass_len;
    CliPrint("start p2p go\r\n");
    ssid_len = strlen(P2P_GO_SSID_STRING);
    memcpy(p2p_cfg.ssid.array, P2P_GO_SSID_STRING, ssid_len);
    p2p_cfg.ssid.length = ssid_len;
    p2p_cfg.ssid.array[ssid_len] = '\0';
    pass_len = strlen(P2P_GO_PASS_STRING);
    memcpy(p2p_cfg.key, P2P_GO_PASS_STRING, pass_len);
    p2p_cfg.key[pass_len] = '\0';
    p2p_cfg.chan.band = PHY_BAND_2G4;
    p2p_cfg.chan.type = PHY_CHNL_BW_20;
    p2p_cfg.chan.prim20_freq = 2437;
    p2p_cfg.chan.center1_freq = 2437;
    p2p_cfg.chan.tx_power = 20;
    p2p_cfg.enable_he = 1;
    p2p_cfg.enable_acs = 0;
    wlan_start_p2p(fvif_idx, &p2p_cfg);
    return 0;
}
MSH_CMD_EXPORT(p2p_go_start, p2p_go_start);

static int p2p_go_stop(int argc, char **argv)
{
    int fvif_idx = 0;
    CliPrint("stop p2p go\r\n");
    wlan_stop_p2p(fvif_idx);
    return 0;
}
MSH_CMD_EXPORT(p2p_go_stop, p2p_go_stop);