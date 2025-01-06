/*****************************************************************************************
*
* @file rf_ripple_df.c
*
* @brief Ripple radio initialization and specific functions
*
* Copyright (C) RivieraWaves 2009-2015
*
*
*****************************************************************************************/

/*****************************************************************************************
* @addtogroup RF_RIPPLE_DF
* @ingroup RF
* @brief Ripple DF Radio Driver
*
* This is the driver block for Ripple DF radio
* @{
*****************************************************************************************/

/*****************************************************************************************
 * INCLUDE FILES
 ****************************************************************************************/
#include "rwip_config.h"        // RW SW configuration

#include <string.h>             // for memcpy
#include "co_utils.h"           // common utility definition
#include "co_math.h"            // common math functions
#include "rf.h"                 // RF interface
#include "plf.h"                // Platform functions

#include "rwip.h"               // for RF API structure definition

#include "em_map.h"


#include "reg_ipcore.h"        // DM core registers

#if (BLE_EMB_PRESENT)
    #include "reg_blecore.h"        // ble core registers
    #include "reg_em_ble_cs.h"      // control structure definitions
#endif //BLE_EMB_PRESENT

#if (BT_EMB_PRESENT)
    #include "reg_btcore.h"         // bt core registers
    #include "reg_em_bt_cs.h"       // control structure definitions
#endif //BT_EMB_PRESENT

// Ripple register definitions and access functions
#if !defined(RP_HWSIM_BYPASS)
    __STATIC void rf_rpl_calib(void);
    __STATIC void rf_rpl_set_txcntl1(void);
    __STATIC void rf_rpl_init_seq(void);
    __STATIC void rf_rpl_pw_up(void);
#endif //(RP_HWSIM_BYPASS)
__STATIC void rf_rpl_mdm_init(void);
#if !defined(RP_HWSIM_BYPASS)
    __STATIC void rf_rpl_sequencers_init(void);
    __STATIC void rf_rpl_txgain_set(void);
    __STATIC void rf_rpl_txdc_cal_seq(void);
#endif //(RP_HWSIM_BYPASS)
__STATIC uint32_t rf_rpl_reg_rd(uint32_t addr);
__STATIC void rf_rpl_reg_wr(uint32_t addr, uint32_t value);

#define REG_RPL_RD                rf_rpl_reg_rd
#define REG_RPL_WR                rf_rpl_reg_wr

#include "reg_ripple.h"          // ripple register
#include "reg_modem.h"           // modem register

/*****************************************************************************************
 * DEFINES
 ****************************************************************************************/
#define RPL_GAIN_TBL_SIZE           0x0F

// Gain table
__STATIC const uint8_t RF_RPL_RX_GAIN_TBL[RPL_GAIN_TBL_SIZE] =
{
    [0] = 43,
    [1] = 37,
    [2] = 31,
    [3] = 25,
    [4] = 19,
    [5] = 13,
    [6] = 7,
    [7] = 1
};

// EM RF SPI address
#define RF_EM_SPI_ADRESS        (EM_BASE_ADDR + EM_RF_SW_SPI_OFFSET)

#define RPL_SPIRD                   0x00
#define RPL_SPIWR                   0x80
#define RPL_RFPLL_TBL_SIZE          0x50
#define RPL_PWR_TBL_SIZE            0x0F

/* The offset value given below is the offset to add to the frequency table index to
   get the value to be programmed in the radio for each channel                      */
#define RPL_FREQTAB_OFFSET          0   // Offset for Ripple radio

/// Radio skew compensation (round trip delay)
#define RPL_RADIO_SKEW              12L

#define RFLOIF                      0x00

#define RPL_RSSI_20dB_THRHLD        -20
#define RPL_RSSI_40dB_THRHLD        -40
#define RPL_RSSI_45dB_THRHLD        -45
#define RPL_RSSI_48dB_THRHLD        -48
#define RPL_RSSI_55dB_THRHLD        -55
#define RPL_RSSI_60dB_THRHLD        -60
#define RPL_RSSI_70dB_THRHLD        -70

// EDR Control value
#define RPL_EDRCNTL                 18 // Default value is set to 18us

// TX max power
#define RPL_POWER_MAX               0x07
#define RPL_POWER_MIN               0x01
#define RPL_POWER_MSK               0x07

// Generic RSSI Threshold
#define RF_RPL_RSSI_THR             0x29


/*****************************************************************************************
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************/
// PLL Lock Table
__STATIC uint8_t RFPLL_LOCK_TABLE[RPL_RFPLL_TBL_SIZE];

// PLL VCOFC table
__STATIC uint8_t RFPLL_VCOFC_TABLE[RPL_RFPLL_TBL_SIZE];

// PLL ICP table
__STATIC uint8_t RFPLL_ICP_TABLE[RPL_RFPLL_TBL_SIZE];

// Power table
__STATIC const int8_t RF_RPL_TX_PW_CONV_TBL[RPL_PWR_TBL_SIZE] =
{
    [0] = -23,
    [1] = -20,
    [2] = -17,
    [3] = -14,
    [4] = -11,
    [5] = -8,
    [6] = -5,
    [7] = -2
};

/*****************************************************************************************
 * FUNCTION DEFINITIONS
 ****************************************************************************************/

/*****************************************************************************************
 * @brief SPI access
 ****************************************************************************************/
__STATIC void rf_rpl_spi_tf(void)
{
    //launch SPI transfer
#if defined(CFG_BT_EMB)
    ip_radiocntl0_spigo_setf(1);
#elif defined(CFG_BLE_EMB)
    ip_radiocntl0_spigo_setf(1);
#endif //CFG_BT_EMB

//#if !defined(RP_HWSIM_BYPASS)
    //wait for transfer to be completed
#if defined(CFG_BT_EMB)
    while (!ip_radiocntl0_spicomp_getf());
#elif defined(CFG_BLE_EMB)
    while (!ip_radiocntl0_spicomp_getf());
#endif //CFG_BT_EMB
//#endif //(RP_HWSIM_BYPASS)
}

/*****************************************************************************************
 * @brief Ripple specific read access
 *
 * @param[in] addr    register address
 *
 * @return uint32_t value
 ****************************************************************************************/
__STATIC uint32_t rf_rpl_reg_rd(uint32_t addr)
{
    uint8_t buf[EM_RF_SW_SPI_SIZE_MAX];
    uint32_t ret;

    //copy control and number of u32 to send
    buf[0] = (uint8_t)(RPL_SPIRD + 1);

    //copy address
    buf[1] = (uint8_t)(addr);

    memcpy((void *)RF_EM_SPI_ADRESS, buf, 2);

    //do the transfer
    rf_rpl_spi_tf();

    //read back the buffer - 4 bytes register value MSB in buf[0]
    memcpy(buf, (void *)(RF_EM_SPI_ADRESS + 4), 4);
    ret = co_read32p(&buf[0]);

    return ret;
}

/*****************************************************************************************
 * @brief Ripple specific write access
 *
 * @param[in] addr    register address
 * @param[in] value   value to write
 *
 * @return uint32_t value
 ****************************************************************************************/
__STATIC void rf_rpl_reg_wr(uint32_t addr, uint32_t value)
{
    uint8_t buf[EM_RF_SW_SPI_SIZE_MAX];

    //inversion for EM reading by U8 on BJ SPI side
    //copy control and number of u32 to send
    buf[0] = (uint8_t)(RPL_SPIWR + 1);
    //copy address
    buf[1] = (uint8_t)(addr);

    //empty byte
    buf[2] = (uint8_t)(0);
    buf[3] = (uint8_t)(0);

    //on old implementations (BT core 3.0, BLE core 1.0) swap the data
    co_write32p(&buf[4], value);

    memcpy((void *)RF_EM_SPI_ADRESS, buf, 8);

    //do the transfer
    rf_rpl_spi_tf();

#if defined(CFG_BT_EMB)
    ip_radiocntl0_spiptr_getf();
#elif defined(CFG_BLE_EMB)
    ip_radiocntl0_spiptr_getf();
#endif //CFG_BT_EMB
}

__STATIC void rf_em_init(void)
{
    uint8_t idx = 0;
    uint8_t temp_freq_tbl[EM_RF_FREQ_TABLE_LEN];

#if BT_EMB_PRESENT
    // First half part of frequency table is for the even frequencies
    while (idx < EM_RF_FREQ_TABLE_LEN / 2)
    {
        temp_freq_tbl[idx] = 2 * idx + RPL_FREQTAB_OFFSET;
        idx++;
    }
    while (idx < EM_RF_FREQ_TABLE_LEN)
    {
        temp_freq_tbl[idx] = 2 * (idx - (EM_RF_FREQ_TABLE_LEN / 2)) + 1 + RPL_FREQTAB_OFFSET;
        idx++;
    }
    em_wr(&temp_freq_tbl[0], EM_FT_OFFSET, EM_RF_FREQ_TABLE_LEN);
#elif BLE_EMB_PRESENT
    while (idx < EM_RF_FREQ_TABLE_LEN)
    {
        temp_freq_tbl[idx] = 2 * idx + RPL_FREQTAB_OFFSET;
        idx++;
    }
    em_wr(&temp_freq_tbl[0], EM_FT_OFFSET, EM_RF_FREQ_TABLE_LEN);
#endif // BT_EMB_PRESENT/BLE_EMB_PRESENT
}

#if (RF_CLASS1)
/*****************************************************************************************
 * @brief Static function - Ripple TX CNTL0 by radio / Used for External PA selection and Class 1 devices
 ****************************************************************************************/
__STATIC void rf_rpl_set_txcntl0(void)
{
    // Program TXCNTL0 with dedicated value
    rpl_rftxcntl0_set(RPL_TXPA_SEL_BIT);
}
#endif // RF_CLASS1

#if !defined(RP_HWSIM_BYPASS)
/*****************************************************************************************
 * @brief Static function - Ripple TX CNTL1
 ****************************************************************************************/
__STATIC void rf_rpl_set_txcntl1(void)
{
    // Program RSSI threshold
    rpl_rssi_cfg_setf(RF_RPL_RSSI_THR);
    ASSERT_ERR(rpl_rssi_cfg_getf() == RF_RPL_RSSI_THR);
}

/***************************************************************************************
 * @brief Static function - Initialization sequence for Ripple radio
 ****************************************************************************************
 */
__STATIC void rf_rpl_init_seq(void)
{
    // Note: All settings are enforced in case reset to the original settings  is required
    // RF PLL Settings
    rpl_rpllcntl0_set(0x00000100);
    ASSERT_ERR(rpl_rpllcntl0_get() == 0x00000100);
    rpl_rpllcntl1_set(0x0010740C);
    ASSERT_ERR(rpl_rpllcntl1_get() == 0x0010740C);
    rpl_rpllcntl2_set(0x00080346);
    ASSERT_ERR(rpl_rpllcntl2_get() == 0x00080346);

    // Gain for FM
    rpl_fmtx_trim_setf(2);

    // RF Sequencer Settings
    rf_rpl_sequencers_init();

    /* **********************
     * RF Tx Chain Settings *
     * **********************/
    rpl_rampgencntl_set(0x10000202);
    ASSERT_ERR(rpl_rampgencntl_get() == 0x10000202);

    rpl_txctrl_force_setf(0x0);
    ASSERT_ERR(rpl_txctrl_force_getf() == 0x0);

//    // DC calibration disabled
//    rpl_txdc_cal_en_setf(0x0);
//    ASSERT_ERR(rpl_txdc_cal_en_getf() == 0x0);

    // RF TX Gain Table Settings
    rf_rpl_txgain_set();

    // Internal PA Selection
    rpl_txpa_sel_setf(0x0);
    ASSERT_ERR(rpl_txpa_sel_getf() == 0x0);
    rpl_txpa_cfg_setf(0x03);
    ASSERT_ERR(rpl_txpa_cfg_getf() == 0x3);

#if (RF_CLASS1)
    // External PA selection
    rf_rpl_set_txcntl0();
#endif // RF_CLASS1
    rf_rpl_set_txcntl1();

    // Make sure IF_EN is set to 0
    rpl_txfilt_if_en_setf(0x0);
    ASSERT_ERR(rpl_txfilt_if_en_getf() == 0x0);
    rpl_txtank_sel_setf(0x07);
    ASSERT_ERR(rpl_txtank_sel_getf() == 0x07);

    // Set DAC gains
    // Except GFSK, needed to pass relative power
//    rpl_rftx_daciq_gain_set(0x00777700);
//    ASSERT_ERR(rpl_rftx_daciq_gain_get() == 0x00777700);

    // [DT] : set back the value 0x77 to the lsb, else LE packet is not seen by PEER (PTS or SNIFFER) when Tx first
    rpl_rftx_daciq_gain_set(0x00777777);
    ASSERT_ERR(rpl_rftx_daciq_gain_get() == 0x00777777);

    /* **********************
     * RF Rx Chain Settings *
     * **********************/
    rpl_rxlna_cfg_setf(0x02);
    ASSERT_ERR(rpl_rxlna_cfg_getf() == 0x02);
    rpl_rxadc_cfg_setf(0x17);
    ASSERT_ERR(rpl_rxadc_cfg_getf() == 0x17);
    rpl_rxadc_clk_inv_en_setf(0x0);

    // Rx Chain optimal settings
    rpl_rxmix_trim_setf(0x3);
    ASSERT_ERR(rpl_rxmix_trim_getf() == 0x3);
    rpl_rxlna_trim_setf(0x3);
    ASSERT_ERR(rpl_rxlna_trim_getf() == 0x3);

    // AGC Settings
    rpl_rssi_preload_cnt_setf(0x1F);
    ASSERT_ERR(rpl_rssi_preload_cnt_getf() == 0x1F);
    rpl_rssi_on_dlyd_cnt_setf(0x8);
    ASSERT_ERR(rpl_rssi_on_dlyd_cnt_getf() == 0x8);
    rpl_agc_settle_time_setf(0x34);
    ASSERT_ERR(rpl_agc_settle_time_getf() == 0x34);

    // RF Tx/Rx Common Settings
    rpl_icp_cal_dsb_setf(0x1);
    ASSERT_ERR(rpl_icp_cal_dsb_getf() == 0x1);
    rpl_calib_mode_setf(0x00);
    ASSERT_ERR(rpl_calib_mode_getf() == 0x00);
    rpl_bb_bias_cfg_setf(0x02);
    ASSERT_ERR(rpl_bb_bias_cfg_getf() == 0x02);

    rpl_dap_clk_mode_setf(0x0);
    ASSERT_ERR(rpl_dap_clk_mode_getf() == 0x0);
    rpl_lodiv2_trim_setf(0x3);
    ASSERT_ERR(rpl_lodiv2_trim_getf() == 0x3);
    rpl_lobuf_trim_setf(0x3);
    ASSERT_ERR(rpl_lobuf_trim_getf() == 0x3);

    // Settings to pass TRMCA11
    rpl_filt_rc_val_setf(0x0);
    ASSERT_ERR(rpl_filt_rc_val_getf() == 0x0);

    // Others Static Settings
    rpl_pullup_cntl_setf(0x1);
    ASSERT_ERR(rpl_pullup_cntl_getf() == 0x1);
    rpl_rfswcntl_set(0x0);
    ASSERT_ERR(rpl_rfswcntl_get() == 0x0);
    rpl_rpllsdcntl2_set(0x80402C0F);
    ASSERT_ERR(rpl_rpllsdcntl2_get() == 0x80402C0F);

//    #if defined(CFG_MBP)
    rpl_rxmdm_bypass_en_setf(0x1);
    rpl_txmdm_bypass_en_setf(0x1);
    rpl_mux0_ctrl_setf(0xA2);
//    #endif //CFG_MBP

    // Calibration procedure
    rf_rpl_calib();
    rpl_mux1_ctrl_setf(0x93);
    ASSERT_ERR(rpl_mux1_ctrl_getf() == 0x93);
    rpl_mux0_ctrl_setf(0x94);
    ASSERT_ERR(rpl_mux0_ctrl_getf() == 0x94);
    rf_rpl_txdc_cal_seq();

    // Set FMTX
    // Note: This must be done after TX DC CAL, never before
    rpl_fmtx_en_setf(0x1);
    ASSERT_ERR(rpl_fmtx_en_getf() == 0x1);
}
/*****************************************************************************************
 * @brief Static function - Ripple RF Power up sequence (all on)
 ****************************************************************************************
 */
__STATIC void rf_rpl_pw_up(void)
{
    /* Set trimming values for LDO */
    rpl_ldo_rxtx_trim_setf(0x00);
    ASSERT_ERR(rpl_ldo_rxtx_trim_getf() == 0x00);
    rpl_ldo_vdig_trim_setf(0x00);
    ASSERT_ERR(rpl_ldo_vdig_trim_getf() == 0x00);
    rpl_ldo_adda_trim_setf(0x00);
    ASSERT_ERR(rpl_ldo_adda_trim_getf() == 0x00);
    rpl_ldo_vco_trim_setf(0x00);
    ASSERT_ERR(rpl_ldo_vco_trim_getf() == 0x00);
    rpl_ldo_xo_trim_setf(0x00);
    ASSERT_ERR(rpl_ldo_xo_trim_getf() == 0x00);
    rpl_ldo_pll_trim_setf(0x00);
    ASSERT_ERR(rpl_ldo_pll_trim_getf() == 0x00);
    rpl_ldo_bgap_trim_setf(0x04);
    ASSERT_ERR(rpl_ldo_bgap_trim_getf() == 0x04);

    /* Set behaviour vs. emable_rm low power mode */
    rpl_ldo_rxtx_lp_mode_setf(0x0);
    ASSERT_ERR(rpl_ldo_rxtx_lp_mode_getf() == 0x00);
    rpl_lda_adda_lp_mode_setf(0x0);
    ASSERT_ERR(rpl_lda_adda_lp_mode_getf() == 0x00);
    rpl_ldo_vco_lp_mode_setf(0x0);
    ASSERT_ERR(rpl_ldo_vco_lp_mode_getf() == 0x00);
    rpl_ldo_pll_lp_mode_setf(0x0);
    ASSERT_ERR(rpl_ldo_pll_lp_mode_getf() == 0x00);
    rpl_ldo_xo_lp_mode_setf(0x0);
    ASSERT_ERR(rpl_ldo_xo_lp_mode_getf() == 0x00);

    /* Power-up PLL */
    rpl_ldo_xo_on_setf(0x01);
    ASSERT_ERR(rpl_ldo_xo_on_getf() == 0x01);
    rpl_ldo_vco_on_setf(0x01);
    ASSERT_ERR(rpl_ldo_vco_on_getf() == 0x01);
    rpl_ldo_pll_on_setf(0x01);
    ASSERT_ERR(rpl_ldo_pll_on_getf() == 0x01);
    rpl_ldo_adda_on_setf(0x01);
    ASSERT_ERR(rpl_ldo_adda_on_getf() == 0x01);
    rpl_ldo_rxtx_on_setf(0x01);
    ASSERT_ERR(rpl_ldo_rxtx_on_getf() == 0x01);

    /* Ramp Gen Control is left as default */
    /* Other Power Settings */
    rpl_crm_cntl0_set(0x00000000);

    // Set clock to 52 MHz
//    if(plf_rf_switch())
//    {
    rpl_bb_clk_div_setf(0x00);
    ASSERT_ERR(rpl_bb_clk_div_getf() == 0x00);
//    }
}
#endif //(RP_HWSIM_BYPASS)

/*****************************************************************************************
 * @brief Static function - Init modem for Ripple.
 ****************************************************************************************
 */
__STATIC void rf_rpl_mdm_init(void)
{
//    mdm_rx_startupdel_set(0x00010003);
//    mdm_tx_startupdel_set(0x00010003);
    mdm_rx_startupdel_set(0x0001004D);
    mdm_tx_startupdel_set(0x0001004D);
    mdm_pe_powerthr_set(0x00000018);


    // New value for DFD KFACTOR
    mdm_dfd_kfactor_set(0x0000006E);

    mdm_gsg_dphi_den_set(0x00050000);
    mdm_gsg_dphi_nom_set(0x00300001);
//    mdm_gsg_dphi_den_set(0x00050009);
//    mdm_gsg_dphi_nom_set(0x00300003);
//    mdm_gsg_dphi_den_set(0x00000005);
//    mdm_gsg_dphi_nom_set(0x00010030);
    mdm_mdm_cntl_set(0x00000002);
    mdm_tx_gfskmode_set(0x00000000);

    // Set FMTX anyways
    mdm_fmtxen_setf(0x1);
    //ASSERT_ERR(mdm_fmtxen_getf() == 0x1);

    //Setting br/ble
//    mdm_dsg_nom_set(0x01);
//    mdm_dsg_den_set(0x00);
//    mdm_gsg_den_set(0x00);
//    mdm_gsg_nom_set(0x01);

}
/***************************************************************************************
 * @brief Static function - Sequencer settings Initialization for Ripple radio
 ****************************************************************************************
*/
#if !defined(RP_HWSIM_BYPASS)
__STATIC void rf_rpl_sequencers_init(void)
{
    rpl_seqcntl_set(0x02080208);
    ASSERT_ERR(rpl_seqcntl_get() == 0x02080208);
    // RF TX UP Sequencer Settings
    rpl_tx_up_dly0_setf(0x1D);
    ASSERT_ERR(rpl_tx_up_dly0_getf() == 0x1D);
    rpl_tx_up_dly1_setf(0xCA);
    ASSERT_ERR(rpl_tx_up_dly1_getf() == 0xCA);
    rpl_tx_up_dly2_setf(0x01);
    ASSERT_ERR(rpl_tx_up_dly2_getf() == 0x01);
    rpl_tx_up_dly3_setf(0xE7);
    ASSERT_ERR(rpl_tx_up_dly3_getf() == 0xE7);
    rpl_tx_up_dly4_setf(0x01);
    ASSERT_ERR(rpl_tx_up_dly4_getf() == 0x01);
    rpl_tx_up_dly5_setf(0x3A);
    ASSERT_ERR(rpl_tx_up_dly5_getf() == 0x3A);
    rpl_tx_up_dly6_setf(0x01);
    ASSERT_ERR(rpl_tx_up_dly6_getf() == 0x01);
    rpl_tx_up_dly7_setf(0x01);
    ASSERT_ERR(rpl_tx_up_dly7_getf() == 0x01);
    rpl_tx_up_dly8_setf(0x01);
    ASSERT_ERR(rpl_tx_up_dly8_getf() == 0x01);

    // RF TX Down Sequencer timings
    rpl_tx_dn_dly0_setf(0x01);
    ASSERT_ERR(rpl_tx_dn_dly0_getf() == 0x01);
    rpl_tx_dn_dly1_setf(0x01);
    ASSERT_ERR(rpl_tx_dn_dly1_getf() == 0x01);
    rpl_tx_dn_dly2_setf(0x01);
    ASSERT_ERR(rpl_tx_dn_dly2_getf() == 0x01);
    rpl_tx_dn_dly3_setf(0x01);
    ASSERT_ERR(rpl_tx_dn_dly3_getf() == 0x01);
    rpl_tx_dn_dly4_setf(0x01);
    ASSERT_ERR(rpl_tx_dn_dly4_getf() == 0x01);
    rpl_tx_dn_dly5_setf(0x01);
    ASSERT_ERR(rpl_tx_dn_dly5_getf() == 0x01);
    rpl_tx_dn_dly6_setf(0x01);
    ASSERT_ERR(rpl_tx_dn_dly6_getf() == 0x01);
    rpl_tx_dn_dly7_setf(0x01);
    ASSERT_ERR(rpl_tx_dn_dly7_getf() == 0x01);
    rpl_tx_dn_dly8_setf(0x01);
    ASSERT_ERR(rpl_tx_dn_dly8_getf() == 0x01);

    // TX UP Sequencer Outputs
    rpl_tx_up_out0_setf(0x00);
    ASSERT_ERR(rpl_tx_up_out0_getf() == 0x00);

#if (!RF_CLASS1)
    rpl_tx_up_out1_setf(0x01);
    ASSERT_ERR(rpl_tx_up_out1_getf() == 0x01);
    rpl_tx_up_out2_setf(0x03);
    ASSERT_ERR(rpl_tx_up_out2_getf() == 0x03);
    rpl_tx_up_out3_setf(0x07);
    ASSERT_ERR(rpl_tx_up_out3_getf() == 0x07);
    rpl_tx_up_out4_setf(0x0F);
    ASSERT_ERR(rpl_tx_up_out4_getf() == 0x0F);
    rpl_tx_up_out5_setf(0x1F);
    ASSERT_ERR(rpl_tx_up_out5_getf() == 0x1F);
    rpl_tx_up_out6_setf(0x1F);
    ASSERT_ERR(rpl_tx_up_out6_getf() == 0x1F);
    rpl_tx_up_out7_setf(0x1F);
    ASSERT_ERR(rpl_tx_up_out7_getf() == 0x1F);
    rpl_tx_up_out8_setf(0x1F);
    ASSERT_ERR(rpl_tx_up_out8_getf() == 0x1F);

    // TX DOWN Sequencer Outputs
    rpl_tx_dn_out0_setf(0x0F);
    ASSERT_ERR(rpl_tx_dn_out0_getf() == 0x0F);
    rpl_tx_dn_out1_setf(0x07);
    ASSERT_ERR(rpl_tx_dn_out1_getf() == 0x07);
    rpl_tx_dn_out2_setf(0x03);
    ASSERT_ERR(rpl_tx_dn_out2_getf() == 0x03);
    rpl_tx_dn_out3_setf(0x01);
    ASSERT_ERR(rpl_tx_dn_out3_getf() == 0x01);

#else
    rpl_tx_up_out1_setf(0x21);
    ASSERT_ERR(rpl_tx_up_out1_getf() == 0x21);
    rpl_tx_up_out2_setf(0x23);
    ASSERT_ERR(rpl_tx_up_out2_getf() == 0x23);
    rpl_tx_up_out3_setf(0x27);
    ASSERT_ERR(rpl_tx_up_out3_getf() == 0x27);
    rpl_tx_up_out4_setf(0x2F);
    ASSERT_ERR(rpl_tx_up_out4_getf() == 0x2F);
    rpl_tx_up_out5_setf(0x3F);
    ASSERT_ERR(rpl_tx_up_out5_getf() == 0x3F);
    rpl_tx_up_out6_setf(0x3F);
    ASSERT_ERR(rpl_tx_up_out6_getf() == 0x3F);
    rpl_tx_up_out7_setf(0x3F);
    ASSERT_ERR(rpl_tx_up_out7_getf() == 0x3F);
    rpl_tx_up_out8_setf(0x3F);
    ASSERT_ERR(rpl_tx_up_out8_getf() == 0x3F);

    // TX DOWN Sequencer Outputs
    rpl_tx_dn_out0_setf(0x2F);
    ASSERT_ERR(rpl_tx_dn_out0_getf() == 0x2F);
    rpl_tx_dn_out1_setf(0x27);
    ASSERT_ERR(rpl_tx_dn_out1_getf() == 0x27);
    rpl_tx_dn_out2_setf(0x23);
    ASSERT_ERR(rpl_tx_dn_out2_getf() == 0x23);
    rpl_tx_dn_out3_setf(0x21);
    ASSERT_ERR(rpl_tx_dn_out3_getf() == 0x21);
#endif // !RF_CLASS1

    rpl_tx_dn_out4_setf(0x00);
    ASSERT_ERR(rpl_tx_dn_out4_getf() == 0x00);
    rpl_tx_dn_out5_setf(0x00);
    ASSERT_ERR(rpl_tx_dn_out5_getf() == 0x00);
    rpl_tx_dn_out6_setf(0x00);
    ASSERT_ERR(rpl_tx_dn_out6_getf() == 0x00);
    rpl_tx_dn_out7_setf(0x00);
    ASSERT_ERR(rpl_tx_dn_out7_getf() == 0x00);
    rpl_tx_dn_out8_setf(0x00);
    ASSERT_ERR(rpl_tx_dn_out8_getf() == 0x00);

    // RX Sequencer
    rpl_rx_up_dly0_setf(0x01);
    rpl_rx_up_dly1_setf(0x103);
    rpl_rx_up_dly2_setf(0x19);
    rpl_rx_up_dly3_setf(0x19);
    rpl_rx_up_dly4_setf(0x23);
    rpl_rx_up_dly5_setf(0x01);
    rpl_rx_up_dly6_setf(0x01);
    rpl_rx_up_dly7_setf(0x01);
    rpl_rx_up_dly8_setf(0x01);

    rpl_rx_dn_dly0_setf(0x01);
    rpl_rx_dn_dly1_setf(0x01);
    rpl_rx_dn_dly2_setf(0x01);
    rpl_rx_dn_dly3_setf(0x01);
    rpl_rx_dn_dly4_setf(0x01);
    rpl_rx_dn_dly5_setf(0x01);
    rpl_rx_dn_dly6_setf(0x01);
    rpl_rx_dn_dly7_setf(0x01);
    rpl_rx_dn_dly8_setf(0x01);

    // Sequence with AGC
    rpl_rx_up_out0_setf(0x00);

#if (!RF_CLASS1)
    rpl_rx_up_out1_setf(0x01);
    rpl_rx_up_out2_setf(0x03);
    rpl_rx_up_out3_setf(0x07);
    rpl_rx_up_out4_setf(0x0F);
    rpl_rx_up_out5_setf(0x1F);
    rpl_rx_up_out6_setf(0x1F);
    rpl_rx_up_out7_setf(0x1F);
    rpl_rx_up_out8_setf(0x1F);

    rpl_rx_dn_out0_setf(0x0F);
    rpl_rx_dn_out1_setf(0x07);
    rpl_rx_dn_out2_setf(0x03);
    rpl_rx_dn_out3_setf(0x01);
#else
    rpl_rx_up_out1_setf(0x41);
    rpl_rx_up_out2_setf(0x43);
    rpl_rx_up_out3_setf(0x47);
    rpl_rx_up_out4_setf(0x4F);
    rpl_rx_up_out5_setf(0x5F);
    rpl_rx_up_out6_setf(0x5F);
    rpl_rx_up_out7_setf(0x5F);
    rpl_rx_up_out8_setf(0x5F);

    rpl_rx_dn_out0_setf(0x4F);
    rpl_rx_dn_out1_setf(0x47);
    rpl_rx_dn_out2_setf(0x43);
    rpl_rx_dn_out3_setf(0x41);
#endif //!RF_CLASS1

    rpl_rx_dn_out4_setf(0x00);
    rpl_rx_dn_out5_setf(0x00);
    rpl_rx_dn_out6_setf(0x00);
    rpl_rx_dn_out7_setf(0x00);
    rpl_rx_dn_out8_setf(0x00);
}

/***************************************************************************************
 * @brief Static function - Tx Gain tables settings
 ****************************************************************************************
 */
__STATIC void rf_rpl_txgain_set(void)
{
    rpl_rftx_gfsk_gain_table_set(0, 0x00000003);
    ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(0) == 0x00000003);
    rpl_rftx_gfsk_gain_table_set(1, 0x00000013);
    ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(1) == 0x00000013);
    rpl_rftx_gfsk_gain_table_set(2, 0x00000023);
    ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(2) == 0x00000023);
    rpl_rftx_gfsk_gain_table_set(3, 0x00000033);
    ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(3) == 0x00000033);
    rpl_rftx_gfsk_gain_table_set(4, 0x00000133);
    ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(4) == 0x00000133);
    rpl_rftx_gfsk_gain_table_set(5, 0x00000233);
    ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(5) == 0x00000233);
    rpl_rftx_gfsk_gain_table_set(6, 0x00000333);
    ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(6) == 0x00000333);
    rpl_rftx_gfsk_gain_table_set(7, 0x00000433);
    ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(7) == 0x00000433);
    rpl_rftx_gfsk_gain_table_set(8, 0x00000324);
    ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(8) == 0x00000324);
    rpl_rftx_gfsk_gain_table_set(9, 0x00000324);
    ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(9) == 0x00000324);
    rpl_rftx_gfsk_gain_table_set(10, 0x00000324);
    ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(10) == 0x00000324);
    rpl_rftx_gfsk_gain_table_set(11, 0x00000324);
    ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(11) == 0x00000324);
    rpl_rftx_gfsk_gain_table_set(12, 0x00000324);
    ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(12) == 0x00000324);
    rpl_rftx_gfsk_gain_table_set(13, 0x00000324);
    ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(13) == 0x00000324);
    rpl_rftx_gfsk_gain_table_set(14, 0x00000324);
    ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(14) == 0x00000324);
    rpl_rftx_gfsk_gain_table_set(15, 0x00000324);
    ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(15) == 0x00000324);

    // EDR GAIN
    rpl_rftx_edr_gain_table_set(8, 0x00324324);
    ASSERT_ERR(rpl_rftx_edr_gain_table_get(8) == 0x00324324);
    rpl_rftx_edr_gain_table_set(9, 0x00324324);
    ASSERT_ERR(rpl_rftx_edr_gain_table_get(9) == 0x00324324);
    rpl_rftx_edr_gain_table_set(10, 0x00324324);
    ASSERT_ERR(rpl_rftx_edr_gain_table_get(10) == 0x00324324);
    rpl_rftx_edr_gain_table_set(11, 0x00324324);
    ASSERT_ERR(rpl_rftx_edr_gain_table_get(11) == 0x00324324);
    rpl_rftx_edr_gain_table_set(12, 0x00324324);
    ASSERT_ERR(rpl_rftx_edr_gain_table_get(12) == 0x00324324);
    rpl_rftx_edr_gain_table_set(13, 0x00324324);
    ASSERT_ERR(rpl_rftx_edr_gain_table_get(13) == 0x00324324);
    rpl_rftx_edr_gain_table_set(14, 0x00324324);
    ASSERT_ERR(rpl_rftx_edr_gain_table_get(14) == 0x00324324);
    rpl_rftx_edr_gain_table_set(15, 0x00324324);
    ASSERT_ERR(rpl_rftx_edr_gain_table_get(15) == 0x00324324);
}

/***************************************************************************************
* @ brief static function - Calibration sequence for TXDC offset
***************************************************************************************/
__STATIC void rf_rpl_txdc_cal_seq(void)
{

//    /* Program fake TX gain table for DC calibration */
//
//     rpl_rftx_gfsk_gain_table_set(4,0x00000033);
//     ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(4) == 0x00000033);
//     rpl_rftx_gfsk_gain_table_set(5,0x00000033);
//     ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(5) == 0x00000033);
//     rpl_rftx_gfsk_gain_table_set(6,0x00000033);
//     ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(6) == 0x00000033);
//     rpl_rftx_gfsk_gain_table_set(7,0x00000033);
//     ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(7) == 0x00000033);
//     rpl_rftx_gfsk_gain_table_set(8,0x00000024);
//     ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(8) == 0x00000024);
//     rpl_rftx_gfsk_gain_table_set(9,0x00000024);
//     ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(9) == 0x00000024);
//     rpl_rftx_gfsk_gain_table_set(10,0x00000024);
//     ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(10) == 0x00000024);
//     rpl_rftx_gfsk_gain_table_set(11,0x00000024);
//     ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(11) == 0x00000024);
//     rpl_rftx_gfsk_gain_table_set(12,0x00000024);
//     ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(12) == 0x00000024);
//     rpl_rftx_gfsk_gain_table_set(13,0x00000024);
//     ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(13) == 0x00000024);
//     rpl_rftx_gfsk_gain_table_set(14,0x00000024);
//     ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(14) == 0x00000024);
//     rpl_rftx_gfsk_gain_table_set(15,0x00000024);
//     ASSERT_ERR(rpl_rftx_gfsk_gain_table_get(15) == 0x00000024);
//
//     /* EDR GAIN */
//     rpl_rftx_edr_gain_table_set(4,0x00033033);
//     ASSERT_ERR(rpl_rftx_edr_gain_table_get(4) == 0x00033033);
//     rpl_rftx_edr_gain_table_set(5,0x00033033);
//     ASSERT_ERR(rpl_rftx_edr_gain_table_get(5) == 0x00033033);
//     rpl_rftx_edr_gain_table_set(6,0x00033033);
//     ASSERT_ERR(rpl_rftx_edr_gain_table_get(6) == 0x00033033);
//     rpl_rftx_edr_gain_table_set(7,0x00033033);
//     ASSERT_ERR(rpl_rftx_edr_gain_table_get(7) == 0x00033033);
//     rpl_rftx_edr_gain_table_set(8,0x00033033);
//     ASSERT_ERR(rpl_rftx_edr_gain_table_get(8) == 0x00033033);
//     rpl_rftx_edr_gain_table_set(9,0x00024024);
//     ASSERT_ERR(rpl_rftx_edr_gain_table_get(9) == 0x00024024);
//     rpl_rftx_edr_gain_table_set(10,0x00024024);
//     ASSERT_ERR(rpl_rftx_edr_gain_table_get(10) == 0x00024024);
//     rpl_rftx_edr_gain_table_set(11,0x00024024);
//     ASSERT_ERR(rpl_rftx_edr_gain_table_get(11) == 0x00024024);
//     rpl_rftx_edr_gain_table_set(12,0x00024024);
//     ASSERT_ERR(rpl_rftx_edr_gain_table_get(12) == 0x00024024);
//     rpl_rftx_edr_gain_table_set(13,0x00024024);
//     ASSERT_ERR(rpl_rftx_edr_gain_table_get(13) == 0x00024024);
//     rpl_rftx_edr_gain_table_set(14,0x00024024);
//     ASSERT_ERR(rpl_rftx_edr_gain_table_get(14) == 0x00024024);
//     rpl_rftx_edr_gain_table_set(15,0x00024024);
//     ASSERT_ERR(rpl_rftx_edr_gain_table_get(15) == 0x00024024);


    // Remove TXDC FORCE bit
    rpl_txdc_force_setf(0x0);
    ASSERT_ERR(rpl_txdc_force_getf() == 0x0);

    // Turn on TX
//   rpl_tx_edr_setf(0x1);
//   ASSERT_ERR(rpl_tx_edr_getf() == 0x1);
    rpl_tx_en_setf(0x1);
    ASSERT_ERR(rpl_tx_en_getf() == 0x1);

    // DC calibration settings
    rpl_txdc_ref_value_setf(0x36e);
    ASSERT_ERR(rpl_txdc_ref_value_getf() == 0x36e);
    rpl_txdc_settle_time_setf(0x68);
    ASSERT_ERR(rpl_txdc_settle_time_getf() == 0x68);
    rpl_epc_mode_setf(0x0);
    ASSERT_ERR(rpl_epc_mode_getf() == 0x0);
    rpl_txdc_mode_setf(0x0);
    ASSERT_ERR(rpl_txdc_mode_getf() == 0x0);

//    // Disable timeout counter
//    rpl_txdc_cal_timeout_dsb_setf(0x1);
//    ASSERT_ERR(rpl_txdc_cal_timeout_dsb_getf() == 0x1);

    // Enable TXDC calibration
    rpl_txdc_cal_en_setf(0x1);
    ASSERT_ERR(rpl_txdc_cal_en_getf() == 0x1);

    // Start TXDC calibration
    rpl_txdc_cal_start_setf(0x1);
    ASSERT_ERR(rpl_txdc_cal_start_getf() == 0x1);

    while (!(rpl_txdc_cal_done_getf() || rpl_txdc_cal_dead_getf()))
    {
    }

    rpl_tx_edr_setf(0x0);
    ASSERT_ERR(rpl_tx_edr_getf() == 0x0);
    rpl_tx_en_setf(0x0);
    ASSERT_ERR(rpl_tx_en_getf() == 0x0);
    rpl_txdc_cal_start_setf(0x0);
    ASSERT_ERR(rpl_txdc_cal_start_getf() == 0x0);
    rpl_txdc_cal_en_setf(0x0);
    ASSERT_ERR(rpl_txdc_cal_en_getf() == 0x0);

//     // reprogram tx gain table
//     rpl_rftx_edr_gain_table_set(4,0x00033033);
//     ASSERT_ERR(rpl_rftx_edr_gain_table_get(4) == 0x00033033);
//     rpl_rftx_edr_gain_table_set(5,0x00033033);
//     ASSERT_ERR(rpl_rftx_edr_gain_table_get(5) == 0x00033033);
//     rpl_rftx_edr_gain_table_set(6,0x00033033);
//     ASSERT_ERR(rpl_rftx_edr_gain_table_get(6) == 0x00033033);
//     rpl_rftx_edr_gain_table_set(7,0x00033033);
//     ASSERT_ERR(rpl_rftx_edr_gain_table_get(7) == 0x00033033);
//     rf_rpl_txgain_set();
}
#endif //(RP_HWSIM_BYPASS)

/**
 ****************************************************************************************
 * @brief Static function - Measure Ripple VCO Frequency
 *
 * @param[in] vco_fc_value  VCO
 * @param[in] vco_freq      Pointer to frequency value.
 ****************************************************************************************
 */
__STATIC void rf_rpl_measure_vco_freq(uint8_t vco_fc_value, int *vco_freq)
{
    //Loop control
    uint8_t  exit_loop = 0x00;
    uint32_t prev_RPLLCNTL0 = 0x00000000;
    uint32_t prev_RPLLCNTL1 = 0x00000000;
    uint16_t timeout = 0x0000;

    //Register read value
    uint32_t reg_val = 0x00000000;
    exit_loop = 0x0;

    //Push registers values
    prev_RPLLCNTL0 = rpl_rpllcntl0_get();
    prev_RPLLCNTL1 = rpl_rpllcntl1_get();

    //VCO frequency meter timeout
    timeout = 20;

    //VCO frequency measurement
    rpl_force_rpll_ctrl_setf(0x1);
    rpl_xo_on_setf(0x1);
    rpl_rpll_ld_on_setf(0x1);
    rpl_rpll_cp_on_setf(0x1);
    rpl_rpll_div_on_setf(0x1);
    rpl_rpllcntl1_set(0x105401 | (vco_fc_value << RPL_RPLL_VCOFC_LSB));
    rpl_rpll_vco_on_setf(0x1);
    rpl_rpll_locnt_on_setf(0x1);

    while (exit_loop == 0)
    {
        reg_val = rpl_rpll_locnt_done_getf();
        timeout = timeout - 1;

        if (reg_val == 0x1)
        {
            exit_loop = 1;
        }
        else if (timeout == 0)
        {
            exit_loop = 1;
        }
    }

    //Compute VCO frequency
    *vco_freq = (uint16_t)(rpl_rpll_locnt_val_getf());

    //Pop register values
    rpl_rpllcntl0_set(prev_RPLLCNTL0);
    rpl_rpllcntl1_set(prev_RPLLCNTL1);
}

/**
 ****************************************************************************************
 * @brief Static function - for VCO Calibration
 *
 * @param[in] channel   channel
 * @param[in] vco_val   vco value
 ****************************************************************************************
 */
__STATIC void rf_rpl_calib_vco_fq(uint8_t channel, uint8_t *vco_val)
{
    //Target VCO frequency error
    int     target_freq_err, vco_freq, freq_err ;
    bool    exit_loop;
    uint8_t vco = 0x00;

    target_freq_err = 2402 - 7 + channel;
    exit_loop = 0;

    //Set channel
    rpl_rfdyncntl_set(channel << RPL_CHANNEL_LSB);

    //Starting VCO_FC Algorithm
    vco = 8;

    //Till not all the VCO_FC values not covered or error frequency is good enough
    while (exit_loop == 0)
    {
        //Measure VCO frequency
        rf_rpl_measure_vco_freq(vco, &vco_freq);

        //Compute frequency error
        freq_err = 2402 - 7 + channel - vco_freq;

        //Save the best settings
        if (co_abs(freq_err) < target_freq_err)
        {
            target_freq_err = co_abs(freq_err);

            *vco_val = vco;

            //Try next VCO_FC value
            if (freq_err < 0)
            {
                vco = vco + 1;
            }
            else
            {
                vco = vco - 1;
            }
        }
        else
        {
            exit_loop = 1;
        }
    }
}

/**
 ****************************************************************************************
 * @brief Static function for calibrating ICP value.
 *
 * @param[in] icp Pointer to value to calibrate.
 ****************************************************************************************
 */
__STATIC void rf_rpl_calib_icp(uint8_t channel, uint8_t *icp)
{
    *icp = 4;
}

/**
 ****************************************************************************************
 * @brief Static function for status lock.
 *
 * @param[in] chnl  channel
 * @param[in] icp   icp
 * @param[in] vco   vco value
 * @param[in] lock  pointer to lock
 ****************************************************************************************
 */
__STATIC void rf_rpl_status_lock(uint8_t chnl, uint8_t icp, uint8_t vco, uint8_t *lock)
{
    //Loop control
    bool exit_loop;

    //Push registers values
    uint32_t prev_RPLLCNTL0;
    uint32_t prev_RPLLCNTL1;
    int timeout;

    //Register read value
    uint32_t regval;

    exit_loop = 0;

    prev_RPLLCNTL0 = rpl_rpllcntl0_get();
    prev_RPLLCNTL1 = rpl_rpllcntl1_get();

    //PLL lock timeout
    timeout = 20;

    //Set chnl
    rpl_rfdyncntl_set(chnl << RPL_CHANNEL_LSB);

    //Try to lock PLL
    rpl_rpllcntl1_set(0x105000 | (icp << RPL_RPLL_ICP_LSB) | (vco << RPL_RPLL_VCOFC_LSB));

    // Turn on vcoref, ld, vco, cp div
    rpl_rpllcntl0_set(0x8000012F);

    //Get PLL lock status
    while (exit_loop == 0)
    {
        regval = rpl_rpll_locked_getf();

        timeout = timeout - 1;

        if (regval == 0x1)
        {
            *lock = 0;
            exit_loop = 1;
        }
        else if (timeout <= 0)
        {
            *lock = 1;
            exit_loop = 1;
        }
    }

    //Pop register values
    rpl_rpllcntl0_set(prev_RPLLCNTL0);
    rpl_rpllcntl1_set(prev_RPLLCNTL1);
}


/***************************************************************************************
 * @brief Static function for radio PLL auto-calibration.
 ****************************************************************************************
 */
__STATIC void rf_rpl_pll_autocalib(void)
{
    uint8_t  chnl;
    uint32_t wreg;

    for (chnl = 0; chnl < 80; chnl++)
    {
        //Calibrate VCOFC value
        rf_rpl_calib_vco_fq(chnl, &RFPLL_VCOFC_TABLE[chnl]);

        //Calibrate ICP value
        rf_rpl_calib_icp(chnl, &RFPLL_ICP_TABLE[chnl]);

        //Test if PLL Lock
        rf_rpl_status_lock(chnl, RFPLL_ICP_TABLE[chnl],
                           RFPLL_VCOFC_TABLE[chnl],
                           &RFPLL_LOCK_TABLE[chnl]);
    }

    for (chnl = 0; chnl < 80; chnl++)
    {
        //Write calibration table
        wreg = (uint32_t)((RFPLL_LOCK_TABLE[chnl] * 0x10000) |
                          (RFPLL_ICP_TABLE[chnl] * 0x1000) |
                          (RFPLL_VCOFC_TABLE[chnl] * 0x100) | 0x80 | chnl);

        rpl_vcofc_icp_calibcntl_set(wreg);
        rpl_vcofc_icp_calibcntl_set((uint32_t)chnl);

    }
}

/***************************************************************************************
 * @brief Static Ripple radio Calibration function.
 ***************************************************************************************
 */
__STATIC void rf_rpl_calib(void)
{
    rf_rpl_pll_autocalib();
}

/**
 *****************************************************************************************
 * @brief Convert RSSI to dBm
 *
 * @param[in] rssi_reg RSSI read from the HW registers
 *
 * @return The converted RSSI
 *
 *****************************************************************************************
 */
__STATIC int8_t rf_rssi_convert(uint8_t rssi_reg)
{
    uint8_t GRx;
    int8_t RssidBm = 0;
    uint16_t PowerModem;

    /* Get the RSSI value from the look up table and get its signed value
     * Get the 2-complements signed value on 8 bits */
    PowerModem = ((rssi_reg & 0xF8) >> 3) * 2;
    GRx = RF_RPL_RX_GAIN_TBL[rssi_reg & 0x07];
    RssidBm = PowerModem  - GRx - 64;

    return (RssidBm);
}

/**
 *****************************************************************************************
 * @brief Get the TX power as control structure TX power field from a value in dBm.
 *
 * @param[in] txpwr_dbm   TX power in dBm
 * @param[in] option      If TXPWR_CS_LOWER, return index equal to or lower than requested
 *                        If TXPWR_CS_HIGHER, return index equal to or higher than requested
 *                        If TXPWR_CS_NEAREST, return index nearest to the desired value
 *
 * @return The index of the TX power
 *
 *****************************************************************************************
 */
__STATIC uint8_t rf_txpwr_cs_get(int8_t txpwr_dbm, uint8_t option)
{
    ASSERT_ERR(option <= TXPWR_CS_NEAREST);

    uint8_t i;

    for (i = RPL_POWER_MIN; i < RPL_POWER_MAX; i++)
    {
        // Loop until we find a power higher than or equal to the requested one
        if (RF_RPL_TX_PW_CONV_TBL[i] >= txpwr_dbm)
            break;
    }

    if ((RF_RPL_TX_PW_CONV_TBL[i] > txpwr_dbm) && (i > RPL_POWER_MIN))
    {
        if ((option == TXPWR_CS_LOWER)
                || ((option == TXPWR_CS_NEAREST) && (co_abs(txpwr_dbm - RF_RPL_TX_PW_CONV_TBL[i - 1]) < co_abs(txpwr_dbm - RF_RPL_TX_PW_CONV_TBL[i]))))
        {
            i--;
        }
    }

    return (i);
}

/**
 *****************************************************************************************
 * @brief Init RF sequence after reset.
 *****************************************************************************************
 */
__STATIC void rf_reset(void)
{
#if !defined(RP_HWSIM_BYPASS)
    // Calibration procedure
    rf_rpl_calib();
#endif
}

#if defined(CFG_BT_EMB)
/**
 *****************************************************************************************
 * @brief Decrease the TX power by one step
 *
 * @param[in] link_id Link ID for which the TX power has to be decreased
 *
 * @return true when minimum power is reached, false otherwise
 *****************************************************************************************
 */
__STATIC bool rf_txpwr_dec(uint8_t link_id)
{
    bool boMinpow = true;
    uint8_t tx_pwr = em_bt_pwrcntl_txpwr_getf(EM_BT_CS_ACL_INDEX(link_id)) & RPL_POWER_MSK;

    if (tx_pwr > RPL_POWER_MIN)
    {
        //Increase the TX power value
        em_bt_pwrcntl_txpwr_setf(EM_BT_CS_ACL_INDEX(link_id), tx_pwr - 1);
        boMinpow = false;
    }

    return (boMinpow);
}

/**
 *****************************************************************************************
 * @brief Increase the TX power by one step
 *
 * @param[in] link_id Link ID for which the TX power has to be increased
 *
 * @return true when maximum power is reached, false otherwise
 *****************************************************************************************
 */
__STATIC bool rf_txpwr_inc(uint8_t link_id)
{
    bool boMaxpow = true;
    uint8_t tx_pwr = em_bt_pwrcntl_txpwr_getf(EM_BT_CS_ACL_INDEX(link_id)) & RPL_POWER_MSK;

    if (tx_pwr < RPL_POWER_MAX)
    {
        //Increase the TX power value
        em_bt_pwrcntl_txpwr_setf(EM_BT_CS_ACL_INDEX(link_id), tx_pwr + 1);
        boMaxpow = false;
    }

    return (boMaxpow);
}

/**
 ****************************************************************************************
 * @brief Set the TX power to max
 *
 * @param[in] link_id     Link Identifier
 ****************************************************************************************
 */
__STATIC void txpwr_max_set(uint8_t link_id)
{
    //Increase the TX power value
    em_bt_pwrcntl_txpwr_setf(EM_BT_CS_ACL_INDEX(link_id), RPL_POWER_MAX);
}
#endif // CFG_BT_EMB

#if defined(CFG_BLE_EMB)

/**
 *****************************************************************************************
 * @brief Enable/disable force AGC mechanism
 *
 * @param[in]  True: Enable / False: disable
 *****************************************************************************************
 */
__STATIC void rf_force_agc_enable(bool en)
{
#if defined(CFG_BT_EMB)
    ip_radiocntl1_forceagc_en_setf(en);
#else
    ip_radiocntl1_forceagc_en_setf(en);
#endif //CFG_BLE_EMB
}
#endif //CFG_BLE_EMB

/**
 *****************************************************************************************
 * @brief Get TX power in dBm from the index in the control structure
 *
 * @param[in] txpwr_idx  Index of the TX power in the control structure
 * @param[in] modulation Modulation: 1 or 2 or 3 MBPS
 *
 * @return The TX power in dBm
 *
 *****************************************************************************************
 */
__STATIC int8_t rf_txpwr_dbm_get(uint8_t txpwr_idx, uint8_t modulation)
{
    // power table is the same for BR and EDR
    return (RF_RPL_TX_PW_CONV_TBL[txpwr_idx]);
}

/**
 *****************************************************************************************
 * @brief Sleep function for the RF.
 *****************************************************************************************
 */
__STATIC void rf_sleep(void)
{
    ip_deepslcntl_set(ip_deepslcntl_get() |
                      IP_DEEP_SLEEP_ON_BIT |    // RW BT Core sleep
                      IP_RADIO_SLEEP_EN_BIT |   // Radio sleep
                      IP_OSC_SLEEP_EN_BIT);     // Oscillator sleep
}

/****************************************************************************************
 * RADIO FUNCTION INTERFACE
 ***************************************************************************************/
void rf_init(struct rwip_rf_api *api)
{
    uint8_t length = PARAM_LEN_RSSI_THR;

    // Initialize the RF driver API structure
    api->reg_rd = rf_rpl_reg_rd;
    api->reg_wr = rf_rpl_reg_wr;
    api->txpwr_dbm_get = rf_txpwr_dbm_get;
    api->txpwr_min = RPL_POWER_MIN;
    api->txpwr_max = RPL_POWER_MAX;
    api->sleep = rf_sleep;
    api->reset = rf_reset;

#if defined(CFG_BLE_EMB)
    api->force_agc_enable = rf_force_agc_enable;
#endif //CFG_BLE_EMB

    api->rssi_convert = rf_rssi_convert;
    api->txpwr_cs_get = rf_txpwr_cs_get;

#if defined(CFG_BT_EMB)
    api->txpwr_dec = rf_txpwr_dec;
    api->txpwr_inc = rf_txpwr_inc;
    api->txpwr_max_set = txpwr_max_set;
#endif //CFG_BT_EMB

    // Initialize the RSSI thresholds (high, low, interference)
    // The upper threshold level is 20 dB above the lower threshold level to an accuracy of +-6 dB
    // These are 'real' signed values in dBm
    if (rwip_param.get(PARAM_ID_RSSI_HIGH_THR, &length, (uint8_t *)&api->rssi_high_thr) != PARAM_OK)
    {
        api->rssi_high_thr = (int8_t)RPL_RSSI_40dB_THRHLD;
    }
    if (rwip_param.get(PARAM_ID_RSSI_LOW_THR, &length, (uint8_t *)&api->rssi_low_thr) != PARAM_OK)
    {
        api->rssi_low_thr = (int8_t)RPL_RSSI_60dB_THRHLD;
    }
    if (rwip_param.get(PARAM_ID_RSSI_INTERF_THR, &length, (uint8_t *)&api->rssi_interf_thr) != PARAM_OK)
    {
        api->rssi_interf_thr = (int8_t)RPL_RSSI_70dB_THRHLD;
    }

    /// Initialize Exchange Memory
    rf_em_init();

    ip_radiocntl0_pack(/*uint16_t spiptr*/ (EM_RF_SW_SPI_OFFSET >> 2),
                                           /*uint8_t  spicfg*/   0,
                                           /*uint8_t  spifreq*/  1,
                                           /*uint8_t  spigo*/    0);

    /* BLE RADIOCNTL1 */
    ip_radiocntl1_pack(/*uint8_t  forceagcen*/       1,
            /*uint8_t  forceiq*/         0,
            /*uint8_t  rxdnsl*/          0,
            /*uint8_t  txdnsl*/          0,
            /*uint16_t forceagclength*/  0x420,
            /*uint8_t  syncpulsemode*/   0,
            /*uint8_t  syncpulsesrc*/    0,
            /*uint8_t  dpcorren*/        1,
            /*uint8_t  jefselect*/       0,
            /*uint8_t  xrfsel*/          0x01,
            /*uint8_t  subversion*/      0x0);

#if defined(CFG_BLE_EMB)
    /* BLE RADIOCNTL2 */
    ble_radiocntl2_pack(/*uint8_t  lrsynccompmode*/ 0x0,
            /*uint8_t  rxcitermbypass*/ 0x0,
            /*uint8_t  lrvtbflush*/     0x8,
            /*uint8_t  phymsk*/         0x2, // mark that Coded phy are supported
            /*uint8_t  lrsyncerr*/      0,
            /*uint8_t  syncerr*/        0,
            /*uint16_t freqtableptr*/ (EM_FT_OFFSET >> 2));

    /* BLE RADIOCNTL3 */
    ble_radiocntl3_pack(/*uint8_t rxrate3cfg*/    0x1, // map on 1 Mbps
            /*uint8_t rxrate2cfg*/    0x1, // map on 1 Mbps
            /*uint8_t rxrate1cfg*/    0x0,
            /*uint8_t rxrate0cfg*/    0x1,
            /*uint8_t getrssidelay */ BLE_GETRSSIDELAY_RST,
            /*uint8_t rxsyncrouting*/ 0x0,
            /*uint8_t rxvalidbeh*/    0x0,
            /*uint8_t txrate3cfg*/    0x1, // map on 1 Mbps
            /*uint8_t txrate2cfg*/    0x1, // map on 1 Mbps
            /*uint8_t txrate1cfg*/    0x0,
            /*uint8_t txrate0cfg*/    0x1,
            /*uint8_t txvalidbeh*/    0x0);

    /* BLE RADIOPWRUPDN0 */
    ble_radiopwrupdn0_pack(/*uint8_t syncposition0*/ 0,
            /*uint8_t rxpwrup0*/      0x50,
            /*uint8_t txpwrdn0*/      0x07,
            /*uint8_t txpwrup0*/      0x50);

    /* BLE RADIOPWRUPDN2 */
    ble_radiopwrupdn2_pack(/*uint8_t syncposition2*/ 0,
            /*uint8_t rxpwrup2*/      0x50,
            /*uint8_t txpwrdn2*/      0x07,
            /*uint8_t txpwrup2*/      0x50);

    /* BLE RADIOPWRUPDN3 */
    ble_radiopwrupdn3_pack(/*uint8_t txpwrdn3*/      0x07,
            /*uint8_t txpwrup3*/      0x50);

    /* BLE RADIOTXRXTIM0 */
    ble_radiotxrxtim0_pack(/*uint8_t rfrxtmda0*/   0,
            /*uint8_t rxpathdly0*/  0x8,
            /*uint8_t txpathdly0*/  0x3);

    /* BLE RADIOTXRXTIM2 */
    ble_radiotxrxtim2_pack(/*uint8_t rxflushpathdly2*/ 0x10,
            /*uint8_t rfrxtmda2*/       0x00,
            /*uint8_t rxpathdly2*/      0x49,
            /*uint8_t txpathdly2*/      0x03);

    /* BLE RADIOTXRXTIM3 */
    ble_radiotxrxtim3_pack(/*uint8_t rxflushpathdly3*/ 0x10,
            /*uint8_t rfrxtmda3*/       0x00,
            /*uint8_t txpathdly3*/      0x03);
#endif // defined CFG_BLE_EMB

#if defined(CFG_BT_EMB)

    /* EDRCNTL */
    bt_rwbtcntl_nwinsize_setf(0);
    bt_edrcntl_rxgrd_timeout_setf(RPL_EDRCNTL);

    /* BT RADIOPWRUPDN */
    bt_radiopwrupdn_rxpwrupct_setf(0x42);
    bt_radiopwrupdn_txpwrdnct_setf(0x07);
    bt_radiopwrupdn_txpwrupct_setf(0x56);

    /* BT RADIOCNTL 2 */
    bt_radiocntl2_freqtable_ptr_setf((EM_FT_OFFSET >> 2));
    bt_radiocntl2_syncerr_setf(0x7);

    /* BT RADIOTXRXTIM */
#define PRL_TX_PATH_DLY 4
#define PRL_RX_PATH_DLY (RPL_RADIO_SKEW - PRL_TX_PATH_DLY)
    bt_radiotxrxtim_rxpathdly_setf(PRL_RX_PATH_DLY);
    bt_radiotxrxtim_txpathdly_setf(PRL_TX_PATH_DLY);
    bt_radiotxrxtim_sync_position_setf(0x38); // Default is 0x10

    /* BT RADIOCNTL 3*/
    bt_radiocntl3_pack(/*uint8_t rxrate2cfg*/    3,
            /*uint8_t rxrate1cfg*/    2,
            /*uint8_t rxrate0cfg*/    1,
            /*uint8_t rxserparif*/    0,
            /*uint8_t rxsyncrouting*/ 0,
            /*uint8_t rxvalidbeh*/    0,
            /*uint8_t txrate2cfg*/    3,
            /*uint8_t txrate1cfg*/    2,
            /*uint8_t txrate0cfg*/    1,
            /*uint8_t txserparif*/    0,
            /*uint8_t txvalidbeh*/    0);

#endif //CFG_BT_EMB

    /*init modem in core*/
    rf_rpl_mdm_init();

#if !defined(RP_HWSIM_BYPASS)
    //power up sequence for radio
    rf_rpl_pw_up();

    // Ripple Modem and RF TC1 initialization sequence
    rf_rpl_init_seq();
#endif //RP_HWSIM_BYPASS

    // Settings for proper reception
#if defined(CFG_BLE_EMB)
    ip_radiocntl1_forceiq_setf(1);
    ip_radiocntl1_dpcorr_en_setf(0x0);
    ASSERT_ERR(ip_radiocntl1_dpcorr_en_getf() == 0x0);
#endif // CFG_BLE_EMB

#if defined(CFG_BT_EMB)
    ip_radiocntl1_dpcorr_en_setf(0x1);
    ASSERT_ERR(ip_radiocntl1_dpcorr_en_getf() == 0x1);
#endif // CFG_BT_EMB

#if defined(CFG_BLE_EMB)
    // Force IQ mode for BLE only
    ip_radiocntl1_forceiq_setf(1);
#endif //CFG_BLE_EMB

    /* AoA/AoD DF control register settings */
#if defined(CFG_BLE_EMB)
    /* LE-1M timings */
    ble_dfcntl0_1us_rxsampstinst0_1us_setf(22); /* 22/2 -> 11us / 4us (guard period) + 7us de path delay */
    ble_dfcntl0_1us_rxswstinst0_1us_setf(24);   /* 24/2 -> 12us */
    ble_dfcntl0_1us_txswstinst0_1us_setf(24);   /* 24/2 -> 12us */

    ble_dfcntl0_2us_rxsampstinst0_2us_setf(22); /* 22/2 -> 1us / 4us (guard period) + 7us de path delay */
    ble_dfcntl0_2us_rxswstinst0_2us_setf(24);   /* 24/2 -> 12us */
    ble_dfcntl0_2us_txswstinst0_2us_setf(24);   /* 24/2 -> 12us */

    /* LE-2M timings -> forced to 0 as not applicable to ripple */
    ble_dfcntl1_1us_rxsampstinst1_1us_setf(0);
    ble_dfcntl1_1us_rxswstinst1_1us_setf(0);
    ble_dfcntl1_1us_txswstinst1_1us_setf(0);

    ble_dfcntl1_2us_rxsampstinst1_2us_setf(0);
    ble_dfcntl1_2us_rxswstinst1_2us_setf(0);
    ble_dfcntl1_2us_txswstinst1_2us_setf(0);

    ble_dfantcntl_pack(/*uint8_t rxprimidcntlen*/ 1, /*uint8_t rxprimantid*/ 0, /*uint8_t txprimidcntlen*/ 1, /*uint8_t txprimantid*/ 0);
#endif //CFG_BLE_EMB

}
///@} RF_RIPPLE_DF
