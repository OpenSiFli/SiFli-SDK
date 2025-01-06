/**
  ******************************************************************************
  * @file   rf_bf_dual_fpga.c
  * @author Sifli software development team
  * @brief SIFLI BLE stack external implementation.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2020 - 2020,  Sifli Technology
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Sifli integrated circuit
 *    in a product or a software update for such product, must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Sifli nor the names of its contributors may be used to endorse
 *    or promote products derived from this software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Sifli integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY SIFLI TECHNOLOGY "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL SIFLI TECHNOLOGY OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

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
    #ifdef SOC_BF_Z0
        #include "reg_sleep.h"
    #else
        // TODO: Implement for A0
        #include "register.h"
    #endif
#endif //BLE_EMB_PRESENT

#if (BT_EMB_PRESENT)
    #include "reg_btcore.h"         // bt core registers
    #include "reg_em_bt_cs.h"       // control structure definitions
#endif //BT
#include "reg_em_et.h"
#include "reg_em_bt_txdesc.h"

#include "register.h"
#include "ad9361.h"
#include "btdm_patch.h"

//#include "spi_tst_drv.h"

#define RPL_FREQTAB_OFFSET 0
#define TEST_FAIL       0x0
#define TEST_PASS       0x1
#define TEST_UNFINISHED 0x2

#define read_memory(addr)        (*(volatile unsigned int *)((addr)))
#define write_memory(addr,value) (*(volatile unsigned int *)((addr))) = (value)
#define read_byte(addr)          (*(volatile unsigned char *)((addr)))
#define write_byte(addr,value)   (*(volatile unsigned char *)((addr))) = (value)
#define read_hword(addr)         (*(volatile unsigned short *)((addr)))
#define write_hword(addr,value)  (*(volatile unsigned short *)((addr))) = (value)

extern uint32_t g_rf_bt_cs_pwr[EM_BT_CS_NB];
extern uint32_t g_rf_ble_cs_pwr[EM_BLE_CS_NB_MAX];





/*****************************************************************************************
 * @brief Ripple specific read access
 *
 * @param[in] addr    register address
 *
 * @return uint32_t value
 ****************************************************************************************/
static uint32_t rf_rpl_reg_rd(uint32_t addr)
{
    return 0;
}

/*****************************************************************************************
 * @brief Ripple specific write access
 *
 * @param[in] addr    register address
 * @param[in] value   value to write
 *
 * @return uint32_t value
 ****************************************************************************************/
static void rf_rpl_reg_wr(uint32_t addr, uint32_t value)
{
}


/**
 *****************************************************************************************
 * @brief Get TX power in dBm from the index in the control structure
 *
 * @param[in] txpwr_idx  Index of the TX power in the control structure
 * @param[in] modulation Modulation: 1 or 2 or 3 MBPS
 * @param[in] rf_mode: 0:polar 1:IQ
 * @return The TX power in dBm
 *
 *****************************************************************************************
 */
static int8_t rf_txpwr_dbm_get(uint8_t txpwr_idx, uint8_t modulation, uint8_t rf_mode)
{
    return 0xFF;
}


/**
 *****************************************************************************************
 * @brief Sleep function for the RF.
 *****************************************************************************************
 */
static void rf_sleep(void)
{
    hwp_lpsys_aon->SLP_CTRL |= LPSYS_AON_SLP_CTRL_SLEEP_REQ;
    while (((hwp_lpsys_aon->SLP_CTRL & ((uint32_t)LPSYS_AON_SLP_CTRL_SLEEP_STATUS_Msk)) >> LPSYS_AON_SLP_CTRL_SLEEP_STATUS_Pos) == 0);
}



/**
 *****************************************************************************************
 * @brief Init RF sequence after reset.
 *****************************************************************************************
 */
static void rf_reset(void)
{
}


#if defined(CFG_BLE)

/**
 *****************************************************************************************
 * @brief Enable/disable force AGC mechanism
 *
 * @param[in]  True: Enable / False: disable
 *****************************************************************************************
 */
static void rf_force_agc_enable(bool en)
{
}
#endif //CFG_BLE

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
static int8_t rf_rssi_convert(uint8_t rssi_reg)
{
    return -50;
}


/**
 *****************************************************************************************
 * @brief Get the TX power as control structure TX power field from a value in dBm.
 *
 * @param[in] txpwr_dbm   TX power in dBm
 * @param[in] high        If true, return index equal to or higher than requested
 *                        If false, return index equal to or lower than requested
 * @param[in] rf_mode     0:polar  1:IQ
 *
 * @return The index of the TX power
 *
 *****************************************************************************************
 */
static uint8_t rf_txpwr_cs_get(int8_t txpwr_dbm, uint8_t high, uint8_t rf_mode)
{
    return 7;
}


#if defined(CFG_BT)
/**
 *****************************************************************************************
 * @brief Decrease the TX power by one step
 *
 * @param[in] link_id Link ID for which the TX power has to be decreased
 *
 * @return true when minimum power is reached, false otherwise
 *****************************************************************************************
 */
static bool rf_txpwr_dec(uint8_t link_id)
{
    em_bt_pwrcntl_txpwr_setf(link_id, 7);
    return true;
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
static bool rf_txpwr_inc(uint8_t link_id)
{
    em_bt_pwrcntl_txpwr_setf(link_id, 7);
    return true;
}

/**
 ****************************************************************************************
 * @brief Set the TX power to max
 *
 * @param[in] link_id     Link Identifier
 ****************************************************************************************
 */
static void txpwr_max_set(uint8_t link_id)
{
    //Increase the TX power value
    em_bt_pwrcntl_txpwr_setf(link_id, 7);
}

static uint8_t txpwr_mdlt_get(int8_t txpwr_dbm, uint8_t mod)//pwr  GFSK/PSK 0:GFSK 1:other
{
    uint8_t rf_mod = MOD_IQ;
    if (0 == mod)
    {
        if (txpwr_dbm > 13)
        {
            rf_mod = MOD_IQ;
        }
        else
        {
#ifndef FPGA
            rf_mod = MOD_POLAR;
#endif
        }
    }
    return rf_mod;
}

//high:0xFF  pwr:max_cs_level
static uint8_t txpwr_set(int8_t bt_mode, uint8_t cs_index, uint8_t pwr, uint8_t high) //bt/ble 0:ble  cs_index  pwr  higher/lower
{
    if (bt_mode == TYPE_BT)
    {
        em_bt_linkcntl_acledr_setf(cs_index, 1);
    }
    return 0;
}

static uint8_t txpwr_cs_set(int8_t bt_mode, uint8_t cs_index, uint8_t rf_mode, uint8_t cs_val) //bt/ble  cs_index  iq/polar  cs_value
{
    if (TYPE_BT == bt_mode)
    {
        em_bt_frcntl_rf_modtype_setf(cs_index, rf_mode);
        if (MOD_IQ == rf_mode)
        {
            em_bt_linkcntl_iqval_setf(cs_index, cs_val);
            g_rf_bt_cs_pwr[cs_index] |= 0x8000;
            g_rf_bt_cs_pwr[cs_index] = (g_rf_bt_cs_pwr[cs_index] & (~0x70000)) | (cs_val << 16);
        }
        else
        {
            em_bt_pwrcntl_txpwr_setf(cs_index, cs_val);
            g_rf_bt_cs_pwr[cs_index] = g_rf_bt_cs_pwr[cs_index] & (~0x8000);
            g_rf_bt_cs_pwr[cs_index] = (g_rf_bt_cs_pwr[cs_index] & (~0xFF)) | cs_val;
        }
    }
    else
    {
        em_ble_cntl_rf_modtype_setf(cs_index, rf_mode);
        if (MOD_IQ == rf_mode)
        {
            em_ble_filtpol_ralcntl_iqval_setf(cs_index, cs_val);
            g_rf_ble_cs_pwr[cs_index] |= 0x8000;
            g_rf_ble_cs_pwr[cs_index] = (g_rf_ble_cs_pwr[cs_index] & (~0x70000)) | (cs_val << 16);
        }
        else
        {
            em_ble_txrxcntl_txpwr_setf(cs_index, cs_val);
            g_rf_ble_cs_pwr[cs_index] = g_rf_ble_cs_pwr[cs_index] & (~0x8000);
            g_rf_ble_cs_pwr[cs_index] = (g_rf_ble_cs_pwr[cs_index] & (~0xFF)) | cs_val;
        }
    }
    return 0;
}

static uint8_t txpwr_cs_update(int8_t bt_mode, uint8_t cs_index, uint8_t et_idx)//bt/ble  cs_index  et_index
{
    uint8_t txdesc_idx;
    uint16_t addr;

    if (TYPE_BT == bt_mode)
    {
#if 0
        em_bt_linkcntl_iqval_setf(cs_index, (g_rf_bt_cs_pwr[cs_index] >> 16) & 0x7);
        if ((0 == em_bt_extab_e_sco_getf(et_idx)) && (EM_BT_CS_IDX_TO_LID(cs_index) < MAX_NB_ACTIVE_ACL)) //acl
        {
            addr = em_bt_txdescptr_getf(cs_index);
            txdesc_idx = REG_EM_IDX_GET(BT_TXDESC, addr);
            if (em_bt_txptr_txdone_getf(txdesc_idx) || (em_bt_txpheader_txlength_getf(txdesc_idx) == 0))
            {
                em_bt_linkcntl_iqval_setf(cs_index, 0);
            }

        }
#else
        if ((EM_BT_CS_IDX_TO_LID(cs_index) < MAX_NB_ACTIVE_ACL) && (0 == em_bt_extab_e_sco_getf(et_idx)))
        {
            em_bt_linkcntl_iqval_setf(cs_index, 0);
            addr = em_bt_txdescptr_getf(cs_index);
            if (addr)
            {
                txdesc_idx = REG_EM_IDX_GET(BT_TXDESC, addr);
                if ((em_bt_txlmbufptr_getf(txdesc_idx) == NULL)
                        && em_bt_txaclbufptr_getf(txdesc_idx) && (em_bt_linkcntl_acledr_getf(cs_index) != 0)
                        && em_bt_txheader_txtype_getf(txdesc_idx) != DM1_TYPE)
                {
                    em_bt_linkcntl_iqval_setf(cs_index, 7);
                }
            }
        }

#endif
    }
    return 0;
}

#endif // CFG_BT


static void rf_em_init(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(RF_PATCH_TYPE, RF_EM_INIT_FUN_BIT);
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
        temp_freq_tbl[idx] = 1 * idx + RPL_FREQTAB_OFFSET;
        idx++;
    }
    em_wr(&temp_freq_tbl[0], EM_FT_OFFSET, EM_RF_FREQ_TABLE_LEN);
#endif // BT_EMB_PRESENT/BLE_EMB_PRESENT
}

static void rf_para_configure(void)
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(RF_PATCH_TYPE, RF_PARA_CONFIGURE_FUN_BIT);
    ip_radiocntl0_pack(/*uint8_t  phytxondelay*/    0x30,
            /*uint8_t  phyrxondelay*/    0x30,
            /*uint8_t  phyrftxoffdelay */0x0A,
            /*uint8_t  phyrfrxoffdelay*/ 0x00);

#if 0
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
#endif

#if defined(CFG_BLE_EMB)
    /* BLE RADIOCNTL2 */
    ble_radiocntl2_pack(/*uint8_t  lrsynccompmode*/ 0,
            /*uint8_t  rxcitermbypass*/ 0x0,
            /*uint8_t  lrvtbflush*/     0x8,
            /*uint8_t  phymsk*/         0x3, // mark that Coded phy are supported
            /*uint8_t  lrsyncerr*/      0,
            /*uint8_t  syncerr*/        0,
            /*uint16_t freqtableptr*/ (EM_FT_OFFSET >> 2));

    /* BLE RADIOCNTL3 */
#if 0
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
#endif
    /* BLE RADIOPWRUPDN0 */
    ble_radiopwrupdn0_pack(/*uint8_t syncposition0*/ 0,
            /*uint8_t rxpwrup0*/      0x50,
            /*uint8_t txpwrdn0*/      0x03,
            /*uint8_t txpwrup0*/      0x50);

    ble_radiopwrupdn1_pack(/*uint8_t syncposition0*/ 0,
            /*uint8_t rxpwrup0*/      0x50,
            /*uint8_t txpwrdn0*/      0x03,
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
    ble_radiotxrxtim0_pack(/*uint8_t rfrxtmda0*/   0x16,
            /*uint8_t rxpathdly0*/  0x17,
            /*uint8_t txpathdly0*/  0x3);

    /* BLE RADIOTXRXTIM1 */
    ble_radiotxrxtim1_pack(/*uint8_t rfrxtmda1*/       0x0F,
            /*uint8_t rxpathdly1*/      0x0E,
            /*uint8_t txpathdly1*/      0x04);

    /* BLE RADIOTXRXTIM2 */
    ble_radiotxrxtim2_pack(/*uint8_t rxflushpathdly2*/ 0x22,
            /*uint8_t rfrxtmda2*/       0xa8,
            /*uint8_t rxpathdly2*/      0x0f,
            /*uint8_t txpathdly2*/      0x09);//0x1e  0xa8 0x09 0x03

    /* BLE RADIOTXRXTIM3 */
    ble_radiotxrxtim3_pack(/*uint8_t rxflushpathdly3*/ 0x1b,
            /*uint8_t rfrxtmda3*/       0x17,
            /*uint8_t txpathdly3*/      0x03);

#endif // defined CFG_BLE_EMB

#if  defined(CFG_BT_EMB)

    /* EDRCNTL */
    bt_rwbtcntl_nwinsize_setf(NORMAL_WIN_SIZE / 2);
    bt_edrcntl_rxgrd_timeout_setf(0x37);
    bt_edrcntl_tx_swap_setf(1);
    bt_edrcntl_rx_swap_setf(1);

    /* BT RADIOPWRUPDN */
    bt_radiopwrupdn_rxpwrupct_setf(0x50);
    bt_radiopwrupdn_txpwrupct_setf(0x50);


#if 1
    /* BT RADIOCNTL 2 */
    bt_radiocntl2_freqtable_ptr_setf((EM_FT_OFFSET >> 2));


    /* BT RADIOTXRXTIM */
#define RPL_TX_PATH_DLY 4
#define RPL_RADIO_SKEW              22L
#define RPL_RX_PATH_DLY (RPL_RADIO_SKEW - RPL_TX_PATH_DLY)
    bt_radiotxrxtim_rxpathdly_setf(RPL_RX_PATH_DLY);
    bt_radiotxrxtim_txpathdly_setf(RPL_TX_PATH_DLY);

#endif

#endif //CFG_BT_EMB

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(RF_PATCH_TYPE, RF_PARA_CONFIGURE_END_FUN_BIT);

}


static void bt_cfg()
{
    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(RF_PATCH_TYPE, BT_CFG_FUN_BIT);
    hwp_bt_phy->MIXER_CFG1 = 0 ;//set rx mixer if freq 0
    hwp_bt_phy->TX_IF_MOD_CFG  &= ~BT_PHY_TX_IF_MOD_CFG_TX_IF_PHASE_BLE_Msk ;//set tx mixer if freq 0
    hwp_bt_phy->TX_IF_MOD_CFG  &= ~BT_PHY_TX_IF_MOD_CFG_TX_IF_PHASE_BR_Msk ;//set tx mixer if freq 0

    hwp_bt_phy->RX_CTRL1 |= BT_PHY_RX_CTRL1_ADC_Q_EN_1;
    hwp_bt_phy->AGC_CTRL &= ~BT_PHY_AGC_CTRL_AGC_ENABLE;
#if defined(SOC_SF32LB52X)
    hwp_bt_phy->TX_GAUSSFLT_CFG1 = 0x02861500;
#else
    hwp_bt_phy->TX_GAUSSFLT_CFG = 0x02861500;
#endif
    hwp_bt_phy->RX_CTRL1 |= BT_PHY_RX_CTRL1_FRC_ADC_24M;

    //hwp_bt_phy->TX_IF_MOD_CFG2 |= BT_PHY_TX_IF_MOD_CFG2_TX_MOD_GAIN_EDR_FRC_EN;
    //hwp_bt_phy->TX_IF_MOD_CFG2 |= BT_PHY_TX_IF_MOD_CFG2_TX_MOD_GAIN_BR_FRC_EN;
    //hwp_bt_phy->TX_IF_MOD_CFG2 |= BT_PHY_TX_IF_MOD_CFG2_TX_MOD_GAIN_BLE_FRC_EN;
    //hwp_bt_phy->TX_DPSK_CFG1   |= BT_PHY_TX_DPSK_CFG1_TX_DPSK_GAIN_FRC_EN;
    hwp_bt_phy->AGC_CFG5       &= (~BT_PHY_AGC_CFG5_DIG_GAIN_LOW);
    hwp_bt_phy->AGC_CFG5       |= (0x10UL << BT_PHY_AGC_CFG5_DIG_GAIN_LOW_Pos);
    hwp_bt_phy->EDRSYNC_CFG1   |= BT_PHY_EDRSYNC_CFG1_EDRSYNC_METHOD;
    hwp_bt_phy->EDRDEMOD_CFG1  &= (~BT_PHY_EDRDEMOD_CFG1_EDR2_MU_DC);
    hwp_bt_phy->EDRDEMOD_CFG1  |= (0x40UL << BT_PHY_EDRDEMOD_CFG1_EDR2_MU_DC_Pos);
    hwp_bt_phy->EDRDEMOD_CFG1  &= (~BT_PHY_EDRDEMOD_CFG1_EDR2_MU_ERR);
    hwp_bt_phy->EDRDEMOD_CFG1  |= (0x100UL << BT_PHY_EDRDEMOD_CFG1_EDR2_MU_ERR_Pos);
    hwp_bt_phy->EDRDEMOD_CFG2  &= (~BT_PHY_EDRDEMOD_CFG2_EDR3_MU_DC);
    hwp_bt_phy->EDRDEMOD_CFG2  |= (0x40UL << BT_PHY_EDRDEMOD_CFG2_EDR3_MU_DC_Pos);
    hwp_bt_phy->EDRDEMOD_CFG2  &= (~BT_PHY_EDRDEMOD_CFG2_EDR3_MU_ERR);
    hwp_bt_phy->EDRDEMOD_CFG2  |= (0x140UL << BT_PHY_EDRDEMOD_CFG2_EDR3_MU_ERR_Pos);

    hwp_bt_phy->TX_CTRL |= BT_PHY_TX_CTRL_MOD_METHOD_BLE;
    hwp_bt_phy->TX_CTRL |= BT_PHY_TX_CTRL_MOD_METHOD_BR;
    hwp_bt_phy->TX_CTRL |= BT_PHY_TX_CTRL_TX_LOOPBACK_MODE;

    //hwp_bt_phy->TX_CTRL &= (~BT_PHY_TX_CTRL_MAC_MOD_CTRL_EN);
    hwp_bt_phy->TX_CTRL |= BT_PHY_TX_CTRL_MAC_MOD_CTRL_EN;

    hwp_bt_phy->TED_CFG1  &= (~BT_PHY_TED_CFG1_TED_MU_F_BR);
    hwp_bt_phy->TED_CFG1  |= (0x8UL << BT_PHY_TED_CFG1_TED_MU_F_BR_Pos);
    hwp_bt_phy->TED_CFG1  &= (~BT_PHY_TED_CFG1_TED_MU_P_BR);
    hwp_bt_phy->TED_CFG1  |= (0x8UL << BT_PHY_TED_CFG1_TED_MU_P_BR_Pos);
    hwp_bt_phy->TED_CFG1  &= (~BT_PHY_TED_CFG1_TED_MU_P_U);
    hwp_bt_phy->TED_CFG1  |= (0x6UL << BT_PHY_TED_CFG1_TED_MU_P_U_Pos);
    hwp_bt_phy->TED_CFG1  &= (~BT_PHY_TED_CFG1_TED_MU_F_U);
    hwp_bt_phy->TED_CFG1  |= (0x6UL << BT_PHY_TED_CFG1_TED_MU_F_U_Pos);
    hwp_bt_phy->EDRTED_CFG1 = 0xA6;
    hwp_bt_phy->EDRTED_CFG1  &= (~BT_PHY_EDRTED_CFG1_TED_EDR3_MU_P);
    hwp_bt_phy->EDRTED_CFG1  |= (0x6UL << BT_PHY_EDRTED_CFG1_TED_EDR3_MU_P_Pos);
    hwp_bt_phy->EDRTED_CFG1  &= (~BT_PHY_EDRTED_CFG1_TED_EDR3_MU_F);
    hwp_bt_phy->EDRTED_CFG1  |= (0xAUL << BT_PHY_EDRTED_CFG1_TED_EDR3_MU_F_Pos);

    hwp_bt_phy->INTERP_CFG1 |= BT_PHY_INTERP_CFG1_INTERP_METHOD_U;

    hwp_bt_mac->AESCNTL |= BT_MAC_AESCNTL_FORCE_IQ_PWR;
    hwp_bt_mac->AESCNTL |= BT_MAC_AESCNTL_FORCE_IQ_PWR_VAL;

    FUNC_PATCH_ENTRY_NO_PARAM_NO_RETURN(RF_PATCH_TYPE, BT_CFG_END_FUN_BIT);

}

extern void ad9364_bt_cfg();
extern uint8_t ad9364_calibration();


void rf_init(struct rwip_rf_api *api)
{
    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(RF_PATCH_TYPE, RF_INIT_FUN_BIT, api);
    uint8_t rssi_length = PARAM_LEN_RSSI_THR;

    api->reg_rd = rf_rpl_reg_rd;
    api->reg_wr = rf_rpl_reg_wr;
    api->txpwr_dbm_get = rf_txpwr_dbm_get;
    api->txpwr_min = 1;
    api->txpwr_max = 7;
    api->txpwr_max_mod = MOD_IQ;
    api->txpwr_min_mod = MOD_IQ;
    api->sleep = rf_sleep;
    api->reset = rf_reset;

#if defined(CFG_BLE)
    api->force_agc_enable = rf_force_agc_enable;
#endif //CFG_BLE

    api->rssi_convert = rf_rssi_convert;
    api->txpwr_cs_get = rf_txpwr_cs_get;

#if defined(CFG_BT)
    api->txpwr_dec = rf_txpwr_dec;
    api->txpwr_inc = rf_txpwr_inc;
    api->txpwr_max_set = txpwr_max_set;
    api->txpwr_mdlt_get = txpwr_mdlt_get;
    //api->txpwr_set = txpwr_set;
    api->txpwr_cs_set = txpwr_cs_set;
    api->txpwr_cs_update = txpwr_cs_update;
    // Do nothing
#endif

    //ad9364_calibration();

    //ad9364_bt_cfg();

    bt_cfg();


    rf_em_init();
    rf_para_configure();

    // Settings for proper reception
#if defined(CFG_BLE_EMB)
    //ip_radiocntl1_forceiq_setf(1);
    ip_radiocntl1_dpcorr_en_setf(0x0);
    ASSERT_ERR(ip_radiocntl1_dpcorr_en_getf() == 0x0);
#endif // CFG_BLE_EMB

#if  defined(CFG_BT_EMB)
    ip_radiocntl1_dpcorr_en_setf(0x1);
    ASSERT_ERR(ip_radiocntl1_dpcorr_en_getf() == 0x1);
#endif //CFG_BT_EMB

#if defined(CFG_BLE_EMB)
    // Force IQ mode for BLE only
    //ip_radiocntl1_forceiq_setf(1);
#endif //CFG_BLE_EMB

    // Initialize the RSSI thresholds (high, low, interference)
    // The upper threshold level is 20 dB above the lower threshold level to an accuracy of +-6 dB
    // These are 'real' signed values in dBm
    if (rwip_param.get(PARAM_ID_RSSI_HIGH_THR, &rssi_length, (uint8_t *)&api->rssi_high_thr)   != PARAM_OK)
    {
        api->rssi_high_thr   = -40; // TODO: to be sure the value
    }
    if (rwip_param.get(PARAM_ID_RSSI_LOW_THR, &rssi_length, (uint8_t *)&api->rssi_low_thr)    != PARAM_OK)
    {
        api->rssi_low_thr    = -60; // TODO: to be sure the value
    }

    FUNC_PATCH_ENTRY_1_PARAM_NO_RETURN(RF_PATCH_TYPE, RF_INIT_END_FUN_BIT, api);

}


/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/

