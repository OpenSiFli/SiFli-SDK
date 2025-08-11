/** @file wlan_test_mode_tests.c
 *
 *  @brief  This file provides WLAN Test Mode APIs
 *
 *  Copyright 2008-2020, 2023 NXP
 *
 *  SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <wlan_wifi.h>
#include <cli.h>
#include <cli_utils.h>
#include <string.h>
#include <wm_net.h> /* for net_inet_aton */
#include <wifi.h>
#include <wlan_tests.h>

/*
 * NXP Test Framework (MTF) functions
 */

#if CONFIG_RF_TEST_MODE

static bool rf_test_mode = false;

static void dump_wlan_set_rf_test_mode_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-set-rf-test-mode \r\n");
    (void)PRINTF("\r\n");
}

static void dump_wlan_set_rf_test_mode(void)
{
    (void)PRINTF("RF Test Mode is not set\r\n");
    dump_wlan_set_rf_test_mode_usage();
}

static void wlan_rf_test_mode_set(int argc, char *argv[])
{
    int ret;

    if (argc != 1)
    {
        dump_wlan_set_rf_test_mode_usage();
        return;
    }

    ret = wlan_set_rf_test_mode();
    if (ret == WM_SUCCESS)
    {
        rf_test_mode = true;
        (void)PRINTF("RF Test Mode Set configuration successful\r\n");
    }
    else
    {
        (void)PRINTF("RF Test Mode Set configuration failed\r\n");
        dump_wlan_set_rf_test_mode_usage();
    }
}

static void dump_wlan_unset_rf_test_mode_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-unset-rf-test-mode \r\n");
    (void)PRINTF("\r\n");
}

static void wlan_rf_test_mode_unset(int argc, char *argv[])
{
    int ret;

    if (argc != 1)
    {
        dump_wlan_unset_rf_test_mode_usage();
        return;
    }

    ret = wlan_unset_rf_test_mode();
    if (ret == WM_SUCCESS)
    {
        rf_test_mode = false;
        (void)PRINTF("RF Test Mode Unset configuration successful\r\n");
    }
    else
    {
        (void)PRINTF("RF Test Mode Unset configuration failed\r\n");
        dump_wlan_unset_rf_test_mode_usage();
    }
}

static void dump_wlan_set_channel_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-set-rf-channel <channel> \r\n");
    (void)PRINTF("\r\n");
}

static void dump_wlan_set_radio_mode_usage()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-set-rf-radio-mode <radio_mode> \r\n");
    (void)PRINTF("0: set the radio in power down mode\r\n");
    (void)PRINTF("3: sets the radio in 5GHz band, 1X1 mode(path A)\r\n");
    (void)PRINTF("11: sets the radio in 2.4GHz band, 1X1 mode(path A)\r\n");
    (void)PRINTF("\r\n");
}

static void wlan_rf_channel_set(int argc, char *argv[])
{
    int ret;
    uint8_t channel;

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    if (argc != 2)
    {
        dump_wlan_set_channel_usage();
        return;
    }

    channel = strtol(argv[1], NULL, 10);

    ret = wlan_set_rf_channel(channel);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Channel configuration successful\r\n");
    }
    else
    {
        (void)PRINTF("Channel configuration failed\r\n");
        dump_wlan_set_channel_usage();
    }
}

static void dump_wlan_get_radio_mode_usage()
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-get-rf-radio-mode \r\n");
}

static void wlan_rf_radio_mode_get(int argc, char *argv[])
{
    int ret;
    uint8_t radio_mode;

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    if (argc != 1)
    {
        dump_wlan_get_radio_mode_usage();
        return;
    }

    ret = wlan_get_rf_radio_mode(&radio_mode);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Configured radio mode is: %d\r\n", radio_mode);
    }
    else
    {
        (void)PRINTF("Radio mode configuration read failed\r\n");
        dump_wlan_get_radio_mode_usage();
    }
}

static void dump_wlan_get_channel_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-get-rf-channel \r\n");
}

static void wlan_rf_channel_get(int argc, char *argv[])
{
    int ret;
    uint8_t channel;

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    if (argc != 1)
    {
        dump_wlan_get_channel_usage();
        return;
    }

    ret = wlan_get_rf_channel(&channel);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Configured channel is: %d\r\n", channel);
    }
    else
    {
        (void)PRINTF("Channel configuration read failed\r\n");
        dump_wlan_get_channel_usage();
    }
}

static void dump_wlan_set_rf_band_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-set-rf-band <band> \r\n");
#if CONFIG_5GHz_SUPPORT
    (void)PRINTF("band: 0=2.4G, 1=5G \r\n");
#else
    (void)PRINTF("band: 0=2.4G \r\n");
#endif
    (void)PRINTF("\r\n");
}

static void wlan_rf_band_set(int argc, char *argv[])
{
    int ret;
    uint8_t band;

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    if (argc != 2)
    {
        dump_wlan_set_rf_band_usage();
        return;
    }

    band = strtol(argv[1], NULL, 10);

    if (band != 0U
#if CONFIG_5GHz_SUPPORT
            && band != 1U
#endif
       )
    {
        dump_wlan_set_rf_band_usage();
        return;
    }

    ret = wlan_set_rf_band(band);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("RF Band configuration successful\r\n");
    }
    else
    {
        (void)PRINTF("RF Band configuration failed\r\n");
        dump_wlan_set_rf_band_usage();
    }
}

static void dump_wlan_get_rf_band_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-get-rf-band \r\n");
}

static void wlan_rf_band_get(int argc, char *argv[])
{
    int ret;
    uint8_t band;

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    if (argc != 1)
    {
        dump_wlan_get_rf_band_usage();
        return;
    }

    ret = wlan_get_rf_band(&band);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Configured RF Band is: %s\r\n", band ? "5G" : "2.4G");
    }
    else
    {
        (void)PRINTF("RF Band configuration read failed\r\n");
        dump_wlan_get_rf_band_usage();
    }
}

static void dump_wlan_set_bandwidth_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-set-bandwidth <bandwidth> \r\n");
    (void)PRINTF("\r\n");
    (void)PRINTF("\t<bandwidth>: \r\n");
    (void)PRINTF("\t        0: 20MHz\r\n");
#if CONFIG_5GHz_SUPPORT
    (void)PRINTF("\t        1: 40MHz\r\n");
#endif
#if CONFIG_11AC
    (void)PRINTF("\t        4: 80MHz\r\n");
#endif
    (void)PRINTF("\r\n");
}

static void wlan_rf_bandwidth_set(int argc, char *argv[])
{
    int ret;
    uint8_t bandwidth;

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    if (argc != 2)
    {
        dump_wlan_set_bandwidth_usage();
        return;
    }

    bandwidth = strtol(argv[1], NULL, 10);

    ret = wlan_set_rf_bandwidth(bandwidth);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Bandwidth configuration successful\r\n");
    }
    else
    {
        (void)PRINTF("Bandwidth configuration failed\r\n");
        dump_wlan_set_bandwidth_usage();
    }
}

static void dump_wlan_get_bandwidth_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-get-rf-bandwidth \r\n");
}

static void wlan_rf_bandwidth_get(int argc, char *argv[])
{
    int ret;
    uint8_t bandwidth;

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    if (argc != 1)
    {
        dump_wlan_get_bandwidth_usage();
        return;
    }

    ret = wlan_get_rf_bandwidth(&bandwidth);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Configured bandwidth is: %s\r\n", bandwidth == 0 ? "20MHz" : bandwidth == 1U ? "40MHz" : "80MHz");
    }
    else
    {
        (void)PRINTF("Bandwidth configuration read failed\r\n");
        dump_wlan_get_bandwidth_usage();
    }
}

static void dump_wlan_get_and_reset_per_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-get-rf-per \r\n");
}

static void wlan_rf_per_get(int argc, char *argv[])
{
    int ret;
    uint32_t rx_tot_pkt_count, rx_mcast_bcast_count, rx_pkt_fcs_error;

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    if (argc != 1)
    {
        dump_wlan_get_and_reset_per_usage();
        return;
    }

    ret = wlan_get_rf_per(&rx_tot_pkt_count, &rx_mcast_bcast_count, &rx_pkt_fcs_error);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("PER is as below: \r\n");
        (void)PRINTF("  Total Rx Packet Count                    : %d\r\n", rx_tot_pkt_count);
        (void)PRINTF("  Total Rx Multicast/Broadcast Packet Count: %d\r\n", rx_mcast_bcast_count);
        (void)PRINTF("  Total Rx Packets with FCS error          : %d\r\n", rx_pkt_fcs_error);
    }
    else
    {
        (void)PRINTF("PER configuration read failed\r\n");
        dump_wlan_get_and_reset_per_usage();
    }
}

static void dump_wlan_set_tx_cont_mode_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF(
        "wlan-set-rf-tx-cont-mode <enable_tx> <cw_mode> <payload_pattern> <cs_mode> <act_sub_ch> <tx_rate> \r\n");
    (void)PRINTF("Enable                (0:disable, 1:enable)\r\n");
    (void)PRINTF("Continuous Wave Mode  (0:disable, 1:enable)\r\n");
    (void)PRINTF("Payload Pattern       (0 to 0xFFFFFFFF) (Enter hexadecimal value)\r\n");
    (void)PRINTF("CS Mode               (Applicable only when continuous wave is disabled) (0:disable, 1:enable)\r\n");
    (void)PRINTF("Active SubChannel     (0:low, 1:upper, 3:both)\r\n");
    (void)PRINTF("Tx Data Rate          (Rate Index corresponding to legacy/HT/VHT rates)\r\n");
    (void)PRINTF("\r\n");
    (void)PRINTF("To Disable:\r\n");
#ifdef SD9177
    (void)PRINTF("Set all parameters with expected values\r\n");
#else
    (void)PRINTF("  In Continuous Wave Mode:\r\n");
    (void)PRINTF("    Step1: wlan-set-rf-tx-cont-mode 0 1 0 0 0 0 \r\n");
    (void)PRINTF("    Step2: wlan-set-rf-tx-cont-mode 0 \r\n");
    (void)PRINTF("  In none continuous Wave Mode:\r\n");
    (void)PRINTF("    Step1: wlan-set-rf-tx-cont-mode 0 \r\n");
#endif
    (void)PRINTF("\r\n");
}

static void wlan_rf_tx_cont_mode_set(int argc, char *argv[])
{
    int ret;
    uint32_t enable_tx, cw_mode, payload_pattern, cs_mode, act_sub_ch, tx_rate;

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    if (argc == 2 && strtol(argv[1], NULL, 10) == 0)
    {
        enable_tx       = 0;
        cw_mode         = 0;
        payload_pattern = 0;
        cs_mode         = 0;
        act_sub_ch      = 0;
        tx_rate         = 0;
        goto disable;
    }
    else if (argc != 7)
    {
        dump_wlan_set_tx_cont_mode_usage();
        return;
    }
    else
    {
        /*Do nothing*/
    }

    enable_tx       = strtol(argv[1], NULL, 10);
    cw_mode         = strtol(argv[2], NULL, 10);
    errno           = 0;
    payload_pattern = strtol(argv[3], NULL, 16);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtoul errno:%d", errno);
    }
    cs_mode    = strtol(argv[4], NULL, 10);
    act_sub_ch = strtol(argv[5], NULL, 10);
    tx_rate    = strtol(argv[6], NULL, 10);

disable:
    ret = wlan_set_rf_tx_cont_mode(enable_tx, cw_mode, payload_pattern, cs_mode, act_sub_ch, tx_rate);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Tx continuous configuration successful\r\n");
        (void)PRINTF("  Enable                : %s\r\n", enable_tx ? "enable" : "disable");
        (void)PRINTF("  Continuous Wave Mode  : %s\r\n", cw_mode ? "enable" : "disable");
        (void)PRINTF("  Payload Pattern       : 0x%08X\r\n", payload_pattern);
        (void)PRINTF("  CS Mode               : %s\r\n", cs_mode ? "enable" : "disable");
        (void)PRINTF("  Active SubChannel     : %s\r\n", act_sub_ch == 0U ? "low" :
                     act_sub_ch == 1U ? "upper" :
                     "both");
        (void)PRINTF("  Tx Data Rate          : %d\r\n", tx_rate);
    }
    else
    {
        (void)PRINTF("Tx continuous configuration failed\r\n");
        dump_wlan_set_tx_cont_mode_usage();
    }
}

static void dump_wlan_set_tx_antenna_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-set-rf-tx-antenna <antenna> \r\n");
    (void)PRINTF("antenna: 1=Main, 2=Aux \r\n");
    (void)PRINTF("\r\n");
}

static void wlan_rf_tx_antenna_set(int argc, char *argv[])
{
    int ret;
    uint8_t ant;

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    if (argc != 2)
    {
        dump_wlan_set_tx_antenna_usage();
        return;
    }

    ant = strtol(argv[1], NULL, 10);

    if (ant != 1U && ant != 2U)
    {
        dump_wlan_set_tx_antenna_usage();
        return;
    }

    ret = wlan_set_rf_tx_antenna(ant);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Tx Antenna configuration successful\r\n");
    }
    else
    {
        (void)PRINTF("Tx Antenna configuration failed\r\n");
        dump_wlan_set_tx_antenna_usage();
    }
}

static void dump_wlan_get_tx_antenna_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-get-rf-tx-antenna \r\n");
}

static void wlan_rf_tx_antenna_get(int argc, char *argv[])
{
    int ret;
    uint8_t ant;

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    if (argc != 1)
    {
        dump_wlan_get_tx_antenna_usage();
        return;
    }

    ret = wlan_get_rf_tx_antenna(&ant);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Configured Tx Antenna is: %s\r\n", ant == 1U ? "Main" : "Aux");
    }
    else
    {
        (void)PRINTF("Tx Antenna configuration read failed\r\n");
        dump_wlan_get_tx_antenna_usage();
    }
}

static void dump_wlan_set_rx_antenna_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-set-rf-rx-antenna <antenna> \r\n");
    (void)PRINTF("antenna: 1=Main, 2=Aux \r\n");
    (void)PRINTF("\r\n");
}

static void wlan_rf_rx_antenna_set(int argc, char *argv[])
{
    int ret;
    uint8_t ant;

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    if (argc != 2)
    {
        dump_wlan_set_rx_antenna_usage();
        return;
    }

    ant = strtol(argv[1], NULL, 10);

    if (ant != 1U && ant != 2U)
    {
        dump_wlan_set_rx_antenna_usage();
        return;
    }

    ret = wlan_set_rf_rx_antenna(ant);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Rx Antenna configuration successful\r\n");
    }
    else
    {
        (void)PRINTF("Rx Antenna configuration failed\r\n");
        dump_wlan_set_rx_antenna_usage();
    }
}

static void dump_wlan_get_rx_antenna_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-get-rf-rx-antenna \r\n");
}

static void wlan_rf_rx_antenna_get(int argc, char *argv[])
{
    int ret;
    uint8_t ant;

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    if (argc != 1)
    {
        dump_wlan_get_rx_antenna_usage();
        return;
    }

    ret = wlan_get_rf_rx_antenna(&ant);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Configured Rx Antenna is: %s\r\n", ant == 1U ? "Main" : "Aux");
    }
    else
    {
        (void)PRINTF("Rx Antenna configuration read failed\r\n");
        dump_wlan_get_rx_antenna_usage();
    }
}

static void dump_wlan_set_tx_power_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-set-rf-tx-power <tx_power> <modulation> <path_id> \r\n");
#ifdef RW610
    (void)PRINTF("Power       (0 to 20 dBm)\r\n");
#else
    (void)PRINTF("Power       (0 to 24 dBm)\r\n");
#endif
    (void)PRINTF("Modulation  (0: CCK, 1:OFDM, 2:MCS)\r\n");
    (void)PRINTF("Path ID     (0: PathA, 1:PathB, 2:PathA+B)\r\n");
    (void)PRINTF("\r\n");
}

#if !defined(SD8978) && !defined(SD8987) && !defined(SD9177) && !defined(SD8801)
/*
 *  @brief PowerLevelToDUT11Bits
 *
 *  @param Pwr      A user txpwr values of type int
 *  @param PowerLevel   A Pointer of uint32 type for converted txpwr vals
 *  @return     nothing just exit
 */
static void PowerLevelToDUT11Bits(int Pwr, uint32_t *PowerLevel)
{
    int Z = 0;

    if ((Pwr > 64) || (Pwr < -64))
        return;

    Z = (int)(Pwr * 16);
    if (Z < 0)
    {
        Z = Z + (1 << 11);
    }
    (*PowerLevel) = (uint32_t)Z;

    return;
}
#endif

static void wlan_rf_tx_power_set(int argc, char *argv[])
{
    int ret;
    uint32_t power;
    uint8_t mod;
    uint8_t path_id;
#if !defined(SD8978) && !defined(SD8987) && !defined(SD9177) && !defined(SD8801)
    uint32_t power_converted = 0xffffffff;
#endif

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    if (argc != 4)
    {
        dump_wlan_set_tx_power_usage();
        return;
    }

    power   = strtol(argv[1], NULL, 10);
    mod     = strtol(argv[2], NULL, 10);
    path_id = strtol(argv[3], NULL, 10);

#ifdef RW610
    if (power > 20U)
#else
    if (power > 24U)
#endif
    {
        dump_wlan_set_tx_power_usage();
        return;
    }

    if (mod != 0U && mod != 1U && mod != 2U)
    {
        dump_wlan_set_tx_power_usage();
        return;
    }

    if (path_id != 0U && path_id != 1U && path_id != 2U)
    {
        dump_wlan_set_tx_power_usage();
        return;
    }

#if !defined(SD8978) && !defined(SD8987) && !defined(SD9177) && !defined(SD8801)
    /* We need to convert user power vals including -ve vals as per labtool */
    PowerLevelToDUT11Bits((int)power, &power_converted);
    ret = wlan_set_rf_tx_power(power_converted, mod, path_id);
#else
    ret = wlan_set_rf_tx_power(power, mod, path_id);
#endif
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Tx Power configuration successful\r\n");
        (void)PRINTF("  Power         : %d dBm\r\n", power);
        (void)PRINTF("  Modulation    : %s\r\n", mod == 0 ? "CCK" : mod == 1 ? "OFDM" : "MCS");
        (void)PRINTF("  Path ID       : %s\r\n", path_id == 0 ? "PathA" : path_id == 1 ? "PathB" : "PathA+B");
    }
    else
    {
        (void)PRINTF("Tx Power configuration failed\r\n");
        dump_wlan_set_tx_power_usage();
    }
}

static void dump_wlan_set_tx_frame_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF(
        "wlan-set-rf-tx-frame <start> <data_rate> <frame_pattern> <frame_len> <adjust_burst_sifs> <burst_sifs_in_us> "
        "<short_preamble> <act_sub_ch> <short_gi> <adv_coding> <tx_bf> <gf_mode> <stbc> <bssid>\r\n");
    (void)PRINTF("Enable                 (0:disable, 1:enable)\r\n");
    (void)PRINTF(
        "Tx Data Rate           (Rate Index corresponding to legacy/HT/VHT rates)(Enter hexadecimal value)\r\n");
    (void)PRINTF("Payload Pattern        (0 to 0xFFFFFFFF) (Enter hexadecimal value)\r\n");
    (void)PRINTF("Payload Length         (1 to 0x400) (Enter hexadecimal value)\r\n");
    (void)PRINTF("Adjust Burst SIFS3 Gap (0:disable, 1:enable)\r\n");
    (void)PRINTF("Burst SIFS in us       (0 to 255us)\r\n");
    (void)PRINTF("Short Preamble         (0:disable, 1:enable)\r\n");
    (void)PRINTF("Active SubChannel      (0:low, 1:upper, 3:both)\r\n");
    (void)PRINTF("Short GI               (0:disable, 1:enable)\r\n");
    (void)PRINTF("Adv Coding             (0:disable, 1:enable)\r\n");
    (void)PRINTF("Beamforming            (0:disable, 1:enable)\r\n");
    (void)PRINTF("GreenField Mode        (0:disable, 1:enable)\r\n");
    (void)PRINTF("STBC                   (0:disable, 1:enable)\r\n");
    (void)PRINTF("BSSID                  (xx:xx:xx:xx:xx:xx)\r\n");
    (void)PRINTF("\r\n");
    (void)PRINTF("To Disable:\r\n");
    (void)PRINTF("wlan-set-rf-tx-frame 0\r\n");
    (void)PRINTF("\r\n");
}

static void wlan_rf_tx_frame_set(int argc, char *argv[])
{
    int ret;
    uint32_t enable;
    uint32_t data_rate;
    uint32_t frame_pattern;
    uint32_t frame_length;
    uint16_t adjust_burst_sifs;
    uint32_t burst_sifs_in_us;
    uint32_t short_preamble;
    uint32_t act_sub_ch;
    uint32_t short_gi;
    uint32_t adv_coding;
    uint32_t tx_bf;
    uint32_t gf_mode;
    uint32_t stbc;
    uint8_t bssid[MLAN_MAC_ADDR_LENGTH];

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    if (argc == 2 && strtol(argv[1], NULL, 10) == 0)
    {
        enable            = 0;
        data_rate         = 0;
        frame_pattern     = 0;
        frame_length      = 1;
        adjust_burst_sifs = 0;
        burst_sifs_in_us  = 0;
        short_preamble    = 0;
        act_sub_ch        = 0;
        short_gi          = 0;
        adv_coding        = 0;
        tx_bf             = 0;
        gf_mode           = 0;
        stbc              = 0;
        (void)memset(bssid, 0, MLAN_MAC_ADDR_LENGTH);
        goto disable;
    }
    else if (argc != 15)
    {
        dump_wlan_set_tx_frame_usage();
        return;
    }

    enable        = strtol(argv[1], NULL, 10);
    data_rate     = strtol(argv[2], NULL, 16);
    errno         = 0;
    frame_pattern = strtoul(argv[3], NULL, 16);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtoul errno:%d", errno);
    }
    errno        = 0;
    frame_length = strtol(argv[4], NULL, 16);
    if (errno != 0)
    {
        (void)PRINTF("Error during strtoul errno:%d", errno);
    }
    adjust_burst_sifs = strtol(argv[5], NULL, 10);
    burst_sifs_in_us  = strtol(argv[6], NULL, 10);
    short_preamble    = strtol(argv[7], NULL, 10);
    act_sub_ch        = strtol(argv[8], NULL, 10);
    short_gi          = strtol(argv[9], NULL, 10);
    adv_coding        = strtol(argv[10], NULL, 10);
    tx_bf             = strtol(argv[11], NULL, 10);
    gf_mode           = strtol(argv[12], NULL, 10);
    stbc              = strtol(argv[13], NULL, 10);
    ret               = get_mac((const char *)argv[14], (char *)bssid, ':');
    if (ret != 0U)
    {
        dump_wlan_set_tx_frame_usage();
        return;
    }

    if (enable > 1U || frame_length < 1 || frame_length > 0x400U || burst_sifs_in_us > 255U || short_preamble > 1U ||
            act_sub_ch == 2 || act_sub_ch > 3 || short_gi > 1U || adv_coding > 1U || tx_bf > 1U || gf_mode > 1U ||
            stbc > 1U)
    {
        dump_wlan_set_tx_frame_usage();
        return;
    }

disable:
    ret = wlan_set_rf_tx_frame(enable, data_rate, frame_pattern, frame_length, adjust_burst_sifs, burst_sifs_in_us,
                               short_preamble, act_sub_ch, short_gi, adv_coding, tx_bf, gf_mode, stbc, bssid);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Tx Frame configuration successful\r\n");
        (void)PRINTF("  Enable                    : %s\r\n", enable ? "enable" : "disable");
        (void)PRINTF("  Tx Data Rate              : %d\r\n", data_rate);
        (void)PRINTF("  Payload Pattern           : 0x%X\r\n", frame_pattern);
        (void)PRINTF("  Payload Length            : 0x%X\r\n", frame_length);
        (void)PRINTF("  Adjust Burst SIFS3 Gap    : %s\r\n", adjust_burst_sifs ? "enable" : "disable");
        (void)PRINTF("  Burst SIFS in us          : %d us\r\n", burst_sifs_in_us);
        (void)PRINTF("  Short Preamble            : %s\r\n", short_preamble ? "enable" : "disable");
        (void)PRINTF("  Active SubChannel         : %s\r\n", act_sub_ch == 0U ? "low" :
                     act_sub_ch == 1U ? "upper" :
                     "both");
        (void)PRINTF("  Short GI                  : %s\r\n", short_gi ? "enable" : "disable");
        (void)PRINTF("  Adv Coding                : %s\r\n", adv_coding ? "enable" : "disable");
        (void)PRINTF("  Beamforming               : %s\r\n", tx_bf ? "enable" : "disable");
        (void)PRINTF("  GreenField Mode           : %s\r\n", gf_mode ? "enable" : "disable");
        (void)PRINTF("  STBC                      : %s\r\n", stbc ? "enable" : "disable");
        (void)PRINTF("  BSSID                     : ");
        print_mac((const char *)bssid);
        (void)PRINTF("\r\n");
    }
    else
    {
        (void)PRINTF("Tx Frame configuration failed\r\n");
        dump_wlan_set_tx_frame_usage();
    }
}

static void dump_wlan_set_rf_trigger_frame_cfg_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF(
        "wlan-set-rf-trigger-frame-cfg <Enable_tx> <Standalone_hetb> <FRAME_CTRL_TYPE> <FRAME_CTRL_SUBTYPE> "
        "<FRAME_DURATION>"
        "<TriggerType> <UlLen> <MoreTF> <CSRequired> <UlBw> <LTFType> <LTFMode>"
        "<LTFSymbol> <UlSTBC> <LdpcESS> <ApTxPwr> <PreFecPadFct> <PeDisambig> <SpatialReuse>"
        "<Doppler> <HeSig2> <AID12> <RUAllocReg> <RUAlloc> <UlCodingType> <UlMCS> <UlDCM>"
        "<SSAlloc> <UlTargetRSSI> <MPDU_MU_SF> <TID_AL> <AC_PL> <Pref_AC>\r\n");
    (void)PRINTF("Enable_tx                   (Enable/Disable trigger frame transmission)\r\n");
    (void)PRINTF("Standalone_hetb             (Enable/Disable Standalone HE TB support.)\r\n");
    (void)PRINTF("FRAME_CTRL_TYPE             (Frame control type)\r\n");
    (void)PRINTF("FRAME_CTRL_SUBTYPE          (Frame control subtype)\r\n");
    (void)PRINTF("FRAME_DURATION              (Max Duration time)\r\n");
    (void)PRINTF("TriggerType                 (Identifies the Trigger frame variant and its encoding)\r\n");
    (void)PRINTF(
        "UlLen                       (Indicates the value of the L-SIG LENGTH field of the solicited HE TB PPDU)\r\n");
    (void)PRINTF(
        "MoreTF                      (Indicates whether a subsequent Trigger frame is scheduled for transmission)\r\n");
    (void)PRINTF(
        "CSRequired                  (Required to use ED to sense the medium and to consider the medium state and the "
        "NAV in determining whether to respond)\r\n");
    (void)PRINTF("UlBw                        (Indicates the bandwidth in the HE-SIG-A field of the HE TB PPDU)\r\n");
    (void)PRINTF("LTFType                     (Indicates the LTF type of the HE TB PPDU response)\r\n");
    (void)PRINTF("LTFMode                     (Indicates the LTF mode for an HE TB PPDU)\r\n");
    (void)PRINTF("LTFSymbol                   (Indicates the number of LTF symbols present in the HE TB PPDU)\r\n");
    (void)PRINTF(
        "UlSTBC                      (Indicates the status of STBC encoding for the solicited HE TB PPDUs)\r\n");
    (void)PRINTF("LdpcESS                     (Indicates the status of the LDPC extra symbol segment)\r\n");
    (void)PRINTF(
        "ApTxPwr                     (Indicates the AP’s combined transmit power at the transmit antenna connector of "
        "all the antennas used to transmit the triggering PPDU)\r\n");
    (void)PRINTF("PreFecPadFct                (Indicates the pre-FEC padding factor)\r\n");
    (void)PRINTF("PeDisambig                  (Indicates PE disambiguity)\r\n");
    (void)PRINTF(
        "SpatialReuse                (Carries the values to be included in the Spatial Reuse fields in the HE-SIG-A "
        "field of the solicited HE TB PPDUs)\r\n");
    (void)PRINTF("Doppler                     (Indicate that a midamble is present in the HE TB PPDU)\r\n");
    (void)PRINTF(
        "HeSig2                      (Carries the value to be included in the Reserved field in the HE-SIG-A2 subfield "
        "of the solicited HE TB PPDUs)\r\n");
    (void)PRINTF(
        "AID12                       (If set to 0 allocates one or more contiguous RA-RUs for associated STAs)\r\n");
    (void)PRINTF("RUAllocReg                  (RUAllocReg)\r\n");
    (void)PRINTF("RUAlloc                     (Identifies the size and the location of the RU)\r\n");
    (void)PRINTF("UlCodingType                (Indicates the code type of the solicited HE TB PPDU)\r\n");
    (void)PRINTF("UlMCS                       (Indicates the HE-MCS of the solicited HE TB PPDU)\r\n");
    (void)PRINTF("UlDCM                       (Indicates DCM of the solicited HE TB PPDU)\r\n");
    (void)PRINTF("SSAlloc                     (Indicates the spatial streams of the solicited HE TB PPDU)\r\n");
    (void)PRINTF("UlTargetRSSI                (Indicates the expected receive signal power)\r\n");
    (void)PRINTF(
        "MPDU_MU_SF                  (Used for calculating the value by which the minimum MPDU start spacing is "
        "multiplied)\r\n");
    (void)PRINTF(
        "TID_AL                      (Indicates the MPDUs allowed in an A-MPDU carried in the HE TB PPDU and the "
        "maximum number of TIDs that can be aggregated by the STA in the A-MPDU)\r\n");
    (void)PRINTF("AC_PL                       (Reserved)\r\n");
    (void)PRINTF(
        "Pref_AC                     (Indicates the lowest AC that is recommended for aggregation of MPDUs in the "
        "A-MPDU contained in the HE TB PPDU sent as a response to the Trigger frame)\r\n");
}

static void wlan_set_rf_trigger_frame_cfg(int argc, char *argv[])
{
    int ret;
    uint32_t Enable_tx;
    uint32_t Standalone_hetb;
    uint8_t FRAME_CTRL_TYPE;
    uint8_t FRAME_CTRL_SUBTYPE;
    uint16_t FRAME_DURATION;
    uint64_t TriggerType;
    uint64_t UlLen;
    uint64_t MoreTF;
    uint64_t CSRequired;
    uint64_t UlBw;
    uint64_t LTFType;
    uint64_t LTFMode;
    uint64_t LTFSymbol;
    uint64_t UlSTBC;
    uint64_t LdpcESS;
    uint64_t ApTxPwr;
    uint64_t PreFecPadFct;
    uint64_t PeDisambig;
    uint64_t SpatialReuse;
    uint64_t Doppler;
    uint64_t HeSig2;
    uint32_t AID12;
    uint32_t RUAllocReg;
    uint32_t RUAlloc;
    uint32_t UlCodingType;
    uint32_t UlMCS;
    uint32_t UlDCM;
    uint32_t SSAlloc;
    uint8_t UlTargetRSSI;
    uint8_t MPDU_MU_SF;
    uint8_t TID_AL;
    uint8_t AC_PL;
    uint8_t Pref_AC;

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    if (argc != 34)
    {
        dump_wlan_set_rf_trigger_frame_cfg_usage();
        return;
    }

    Enable_tx          = strtol(argv[1], NULL, 10);
    Standalone_hetb    = strtol(argv[2], NULL, 10);
    FRAME_CTRL_TYPE    = strtol(argv[3], NULL, 10);
    FRAME_CTRL_SUBTYPE = strtol(argv[4], NULL, 10);
    FRAME_DURATION     = strtol(argv[5], NULL, 10);
    TriggerType        = strtol(argv[6], NULL, 10);
    UlLen              = strtol(argv[7], NULL, 10);
    MoreTF             = strtol(argv[8], NULL, 10);
    CSRequired         = strtol(argv[9], NULL, 10);
    UlBw               = strtol(argv[10], NULL, 10);
    LTFType            = strtol(argv[11], NULL, 10);
    LTFMode            = strtol(argv[12], NULL, 10);
    LTFSymbol          = strtol(argv[13], NULL, 10);
    UlSTBC             = strtol(argv[14], NULL, 10);
    LdpcESS            = strtol(argv[15], NULL, 10);
    ApTxPwr            = strtol(argv[16], NULL, 10);
    PreFecPadFct       = strtol(argv[17], NULL, 10);
    PeDisambig         = strtol(argv[18], NULL, 10);
    SpatialReuse       = strtol(argv[19], NULL, 10);
    Doppler            = strtol(argv[20], NULL, 10);
    HeSig2             = strtol(argv[21], NULL, 10);
    AID12              = strtol(argv[22], NULL, 10);
    RUAllocReg         = strtol(argv[23], NULL, 10);
    RUAlloc            = strtol(argv[24], NULL, 10);
    UlCodingType       = strtol(argv[25], NULL, 10);
    UlMCS              = strtol(argv[26], NULL, 10);
    UlDCM              = strtol(argv[27], NULL, 10);
    SSAlloc            = strtol(argv[28], NULL, 10);
    UlTargetRSSI       = strtol(argv[29], NULL, 10);
    MPDU_MU_SF         = strtol(argv[30], NULL, 10);
    TID_AL             = strtol(argv[31], NULL, 10);
    AC_PL              = strtol(argv[32], NULL, 10);
    Pref_AC            = strtol(argv[33], NULL, 10);

    ret = wlan_rf_trigger_frame_cfg(Enable_tx, Standalone_hetb, FRAME_CTRL_TYPE, FRAME_CTRL_SUBTYPE, FRAME_DURATION,
                                    TriggerType, UlLen, MoreTF, CSRequired, UlBw, LTFType, LTFMode, LTFSymbol, UlSTBC,
                                    LdpcESS, ApTxPwr, PreFecPadFct, PeDisambig, SpatialReuse, Doppler, HeSig2, AID12,
                                    RUAllocReg, RUAlloc, UlCodingType, UlMCS, UlDCM, SSAlloc, UlTargetRSSI, MPDU_MU_SF,
                                    TID_AL, AC_PL, Pref_AC);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("RF Trigger Frame configuration successful\r\n");
        (void)PRINTF("Enable_tx                   : %d\r\n", Enable_tx);
        (void)PRINTF("Standalone_hetb             : %d\r\n", Standalone_hetb);
        (void)PRINTF("FRAME_CTRL_TYPE             : %d\r\n", FRAME_CTRL_TYPE);
        (void)PRINTF("FRAME_CTRL_SUBTYPE          : %d\r\n", FRAME_CTRL_SUBTYPE);
        (void)PRINTF("FRAME_DURATION              : %d\r\n", FRAME_DURATION);
        (void)PRINTF("TriggerType                 : %lld\r\n", TriggerType);
        (void)PRINTF("UlLen                       : %lld\r\n", UlLen);
        (void)PRINTF("MoreTF                      : %lld\r\n", MoreTF);
        (void)PRINTF("CSRequired                  : %lld\r\n", CSRequired);
        (void)PRINTF("UlBw                        : %lld\r\n", UlBw);
        (void)PRINTF("LTFType                     : %lld\r\n", LTFType);
        (void)PRINTF("LTFMode                     : %lld\r\n", LTFMode);
        (void)PRINTF("LTFSymbol                   : %lld\r\n", LTFSymbol);
        (void)PRINTF("UlSTBC                      : %lld\r\n", UlSTBC);
        (void)PRINTF("LdpcESS                     : %lld\r\n", LdpcESS);
        (void)PRINTF("ApTxPwr                     : %lld\r\n", ApTxPwr);
        (void)PRINTF("PreFecPadFct                : %lld\r\n", PreFecPadFct);
        (void)PRINTF("PeDisambig                  : %lld\r\n", PeDisambig);
        (void)PRINTF("SpatialReuse                : %lld\r\n", SpatialReuse);
        (void)PRINTF("Doppler                     : %lld\r\n", Doppler);
        (void)PRINTF("HeSig2                      : %lld\r\n", HeSig2);
        (void)PRINTF("AID12                       : %d\r\n", AID12);
        (void)PRINTF("RUAllocReg                  : %d\r\n", RUAllocReg);
        (void)PRINTF("RUAlloc                     : %d\r\n", RUAlloc);
        (void)PRINTF("UlCodingType                : %d\r\n", UlCodingType);
        (void)PRINTF("UlMCS                       : %d\r\n", UlMCS);
        (void)PRINTF("UlDCM                       : %d\r\n", UlDCM);
        (void)PRINTF("SSAlloc                     : %d\r\n", SSAlloc);
        (void)PRINTF("UlTargetRSSI                : %d\r\n", UlTargetRSSI);
        (void)PRINTF("MPDU_MU_SF                  : %d\r\n", MPDU_MU_SF);
        (void)PRINTF("TID_AL                      : %d\r\n", TID_AL);
        (void)PRINTF("AC_PL                       : %d\r\n", AC_PL);
        (void)PRINTF("Pref_AC                     : %d\r\n", Pref_AC);
    }
    else
    {
        (void)PRINTF("RF Trigger Frame configuration failed\r\n");
        dump_wlan_set_rf_trigger_frame_cfg_usage();
    }
}

static void dump_wlan_set_rf_he_tb_tx_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-set-rf-he-tb-tx <enable> <qnum> <uint16_t aid> <axq_mu_timer> <tx_power>\r\n");
    (void)PRINTF("Enable           (Enable/Disable trigger response mode)\r\n");
    (void)PRINTF("qnum             (AXQ to be used for the trigger response frame)\r\n");
    (void)PRINTF("aid              (AID of the peer to which response is to be generated)\r\n");
    (void)PRINTF("axq_mu_timer     (MU timer for the AXQ on which response is sent)\r\n");
    (void)PRINTF("tx_power         (TxPwr to be configured for the response)\r\n");
}

static void wlan_set_rf_he_tb_tx(int argc, char *argv[])
{
    int ret;

    uint16_t enable;
    uint16_t qnum;
    uint16_t aid;
    uint16_t axq_mu_timer;
    int16_t tx_power;

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    if (argc != 6)
    {
        dump_wlan_set_rf_he_tb_tx_usage();
        return;
    }

    enable       = strtol(argv[1], NULL, 10);
    qnum         = strtol(argv[2], NULL, 10);
    aid          = strtol(argv[3], NULL, 10);
    axq_mu_timer = strtol(argv[4], NULL, 10);
    tx_power     = strtol(argv[5], NULL, 10);

    ret = wlan_cfg_rf_he_tb_tx(enable, qnum, aid, axq_mu_timer, tx_power);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("HE TB Tx configuration successful\r\n");
        (void)PRINTF("Enable           : %d\r\n", enable);
        (void)PRINTF("qnum             : %d\r\n", qnum);
        (void)PRINTF("aid              : %d\r\n", aid);
        (void)PRINTF("axq_mu_timer     : %d\r\n", axq_mu_timer);
        (void)PRINTF("tx_power         : %d\r\n", tx_power);
    }
    else
    {
        (void)PRINTF("Wrong he tb tx configurations\r\n");
        dump_wlan_set_rf_he_tb_tx_usage();
    }
}

static void wlan_rf_radio_mode_set(int argc, char *argv[])
{
    int ret;
    uint8_t radio_mode;

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    if (argc != 2)
    {
        dump_wlan_set_radio_mode_usage();
        return;
    }

    radio_mode = atoi(argv[1]);
    ret        = wlan_set_rf_radio_mode(radio_mode);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("Set radio mode successful\r\n");
    }
    else
    {
        (void)PRINTF("Set radio mode failed!\r\n");
        dump_wlan_set_radio_mode_usage();
    }
}

static void dump_wlan_set_otp_mac_addr_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-set-rf-otp-mac-addr <mac_addr> \r\n");
}

static void wlan_rf_otp_mac_addr_set(int argc, char *argv[])
{
    int ret;
    uint8_t mac_addr[MLAN_MAC_ADDR_LENGTH];

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    if (argc != 2)
    {
        dump_wlan_set_otp_mac_addr_usage();
        return;
    }

    ret = get_mac(argv[1], (char *)mac_addr, ':');
    if (ret != 0)
    {
        (void)PRINTF("Error: invalid MAC argument\r\n");
        return;
    }

    ret = wlan_set_rf_otp_mac_addr(mac_addr);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("OTP MAC address configuration successful\r\n");
    }
    else
    {
        (void)PRINTF("OTP MAC address configuration failed\r\n");
        dump_wlan_set_otp_mac_addr_usage();
    }
}

static void dump_wlan_get_otp_mac_addr_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-get-rf-otp-mac-addr \r\n");
}

static void wlan_rf_otp_mac_addr_get(int argc, char *argv[])
{
    int ret;
    uint8_t mac_addr[MLAN_MAC_ADDR_LENGTH];

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    if (argc != 1)
    {
        dump_wlan_get_otp_mac_addr_usage();
        return;
    }

    ret = wlan_get_rf_otp_mac_addr(mac_addr);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("OTP MAC address: %02X:%02X:%02X:%02X:%02X:%02X\r\n", mac_addr[0], mac_addr[1], mac_addr[2],
                     mac_addr[3], mac_addr[4], mac_addr[5]);
    }
    else
    {
        (void)PRINTF("OTP MAC address read failed\r\n");
        dump_wlan_get_otp_mac_addr_usage();
    }
}

const uint8_t otp_cal_data[] =
{
    0x01, 0x00, 0x0F, 0x00, 0x88, 0x00, 0x00, 0x20, 0x44, 0x0F, 0x00, 0x00, 0x00, 0x20, 0xFF, 0xFF, 0x40, 0x00,
    0x77, 0x00, 0x29, 0x12, 0x00, 0x00, 0x00, 0x10, 0x00, 0x04, 0x6A, 0xB1, 0x02, 0x00, 0x00, 0x3F, 0x01, 0x00,
    0x00, 0x0D, 0x00, 0x18, 0x97, 0x53, 0x00, 0x00, 0x00, 0x38, 0x39, 0x22, 0x3C, 0x55, 0xBC, 0x68, 0x6A, 0x37,
    0xBE, 0x82, 0x22, 0xB4, 0x41, 0x64, 0x8D, 0xCE, 0x00, 0x1C, 0x9F, 0x37, 0x00, 0x00, 0x00, 0x54, 0x02, 0x04,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x08, 0x00, 0x2D, 0xC6, 0xC0, 0x43, 0x00, 0x00, 0x66, 0x00, 0x00, 0x00, 0x50,
    0x00, 0x1C, 0x49, 0x5F, 0x00, 0x00, 0x00, 0x70, 0x02, 0x05, 0x00, 0x01, 0x00, 0x00, 0x00, 0x08, 0x00, 0x2D,
    0xC6, 0xC0, 0x43, 0x00, 0x00, 0x77, 0x00, 0x00, 0x00, 0x50, 0x00, 0x18, 0xB2, 0x68, 0xFF, 0xFF, 0xFF, 0xFF,
    0xD3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static void dump_wlan_set_otp_cal_data_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-set-rf-otp-cal-data\r\n");
}

static void wlan_rf_otp_cal_data_set(int argc, char *argv[])
{
    int ret;

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    ret = wlan_set_rf_otp_cal_data(otp_cal_data, sizeof(otp_cal_data));
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("OTP cal data configuration successful\r\n");
    }
    else
    {
        (void)PRINTF("OTP cal data configuration failed\r\n");
        dump_wlan_set_otp_cal_data_usage();
    }
}

static void dump_wlan_get_otp_cal_data_usage(void)
{
    (void)PRINTF("Usage:\r\n");
    (void)PRINTF("wlan-get-rf-otp-cal-data \r\n");
}

static void wlan_rf_otp_cal_data_get(int argc, char *argv[])
{
    int ret;
    // int i = 0;
    uint8_t *cal_data = NULL;

    if (!rf_test_mode)
    {
        dump_wlan_set_rf_test_mode();
        return;
    }

    if (argc != 1)
    {
        dump_wlan_get_otp_cal_data_usage();
        return;
    }
#if !CONFIG_MEM_POOLS
    cal_data = (uint8_t *)OSA_MemoryAllocate(CAL_DATA_LEN);
#else
    cal_data = (uint8_t *)OSA_MemoryPoolAllocate(buf_3072_MemoryPool);
#endif
    if (!cal_data)
    {
        (void)PRINTF("Error: failed to alloc memory!\r\n");
        return;
    }

    ret = wlan_get_rf_otp_cal_data(cal_data);
    if (ret == WM_SUCCESS)
    {
        (void)PRINTF("OTP cal data read successfully: 1\r\n");
#if 0
        while (i < CAL_DATA_LEN)
        {
            (void)PRINTF("%02x ", cal_data[i++]);
            if (!(i % 16))
            {
                (void)PRINTF("\r\n");
            }
        }
#endif
    }
    else
    {
        (void)PRINTF("OTP cal data read failed: 0\r\n");
        dump_wlan_get_otp_cal_data_usage();
    }
#if !CONFIG_MEM_POOLS
    (void)OSA_MemoryFree(cal_data);
#else
    OSA_MemoryPoolFree(buf_3072_MemoryPool, cal_data);
#endif
}

static struct cli_command wlan_test_mode_commands[] =
{
    {"wlan-set-rf-test-mode", NULL, wlan_rf_test_mode_set},
    {"wlan-unset-rf-test-mode", NULL, wlan_rf_test_mode_unset},
    {"wlan-set-rf-tx-antenna", "<antenna>", wlan_rf_tx_antenna_set},
    {"wlan-get-rf-tx-antenna", NULL, wlan_rf_tx_antenna_get},
    {"wlan-set-rf-rx-antenna", "<antenna>", wlan_rf_rx_antenna_set},
    {"wlan-get-rf-rx-antenna", NULL, wlan_rf_rx_antenna_get},
    {"wlan-set-rf-band", "<band>", wlan_rf_band_set},
    {"wlan-get-rf-band", NULL, wlan_rf_band_get},
    {"wlan-set-rf-bandwidth", "<bandwidth>", wlan_rf_bandwidth_set},
    {"wlan-get-rf-bandwidth", NULL, wlan_rf_bandwidth_get},
    {"wlan-set-rf-channel", "<channel>", wlan_rf_channel_set},
    {"wlan-get-rf-channel", NULL, wlan_rf_channel_get},
    {"wlan-set-rf-radio-mode", "<radio_mode>", wlan_rf_radio_mode_set},
    {"wlan-get-rf-radio-mode", NULL, wlan_rf_radio_mode_get},
    {"wlan-set-rf-tx-power", "<tx_power> <modulation> <path_id>", wlan_rf_tx_power_set},
    {
        "wlan-set-rf-tx-cont-mode", "<enable_tx> <cw_mode> <payload_pattern> <cs_mode> <act_sub_ch> <tx_rate>",
        wlan_rf_tx_cont_mode_set
    },
    {
        "wlan-set-rf-tx-frame",
        "<start> <data_rate> <frame_pattern> <frame_len> <adjust_burst_sifs> <burst_sifs_in_us> <short_preamble> "
        "<act_sub_ch> <short_gi> <adv_coding> <tx_bf> <gf_mode> <stbc> <bssid>",
        wlan_rf_tx_frame_set
    },
    {
        "wlan-set-rf-trigger-frame-cfg",
        "<Enable_tx> <Standalone_hetb> <FRAME_CTRL_TYPE> <FRAME_CTRL_SUBTYPE> <FRAME_DURATION>"
        "<TriggerType> <UlLen> <MoreTF> <CSRequired> <UlBw> <LTFType> <LTFMode>"
        "<LTFSymbol> <UlSTBC> <LdpcESS> <ApTxPwr> <PreFecPadFct> <PeDisambig> <SpatialReuse>"
        "<Doppler> <HeSig2> <AID12> <RUAllocReg> <RUAlloc> <UlCodingType> <UlMCS> <UlDCM>"
        "<SSAlloc> <UlTargetRSSI> <MPDU_MU_SF> <TID_AL> <AC_PL> <Pref_AC> ",
        wlan_set_rf_trigger_frame_cfg
    },
    {"wlan-set-rf-he-tb-tx", "<enable> <qnum> <aid> <axq_mu_timer> <tx_power>", wlan_set_rf_he_tb_tx},
    {"wlan-get-and-reset-rf-per", NULL, wlan_rf_per_get},
    {"wlan-set-rf-otp-mac-addr", "<mac_addr>", wlan_rf_otp_mac_addr_set},
    {"wlan-get-rf-otp-mac-addr", NULL, wlan_rf_otp_mac_addr_get},
    {"wlan-set-rf-otp-cal-data", NULL, wlan_rf_otp_cal_data_set},
    {"wlan-get-rf-otp-cal-data", NULL, wlan_rf_otp_cal_data_get},
};

int wlan_test_mode_cli_init(void)
{
    if (cli_register_commands(wlan_test_mode_commands, sizeof(wlan_test_mode_commands) / sizeof(struct cli_command)) !=
            0U)
    {
        return -WM_FAIL;
    }

    return WM_SUCCESS;
}

int wlan_test_mode_cli_deinit(void)
{
    if (cli_unregister_commands(wlan_test_mode_commands,
                                sizeof(wlan_test_mode_commands) / sizeof(struct cli_command)) != 0U)
    {
        return -WM_FAIL;
    }

    return WM_SUCCESS;
}
#endif
