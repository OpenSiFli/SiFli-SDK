/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */




#include "bf0_hal.h"
#include "ble_rf_cal.h"


enum BT_TEST_PKT_TYPE
{
    DTM_DM1,
    DTM_DH1,
    DTM_DM3,
    DTM_DH3,
    DTM_DM5,
    DTM_DH5,
    DTM_2DH1,
    DTM_3DH1,
    DTM_2DH3,
    DTM_3DH3,
    DTM_2DH5,
    DTM_3DH5,
    DTM_HV1,
    DTM_HV2,
    DTM_HV3,
    DTM_EV3,
    DTM_EV4,
    DTM_EV5,
    DTM_2EV3,
    DTM_3EV3,
    DTM_2EV5,
    DTM_3EV5,
};
#define ID_NUL_TYPE     0x0
#define POLL_TYPE       0x1
#define FHS_TYPE        0x2
#define DM1_TYPE        0x3
#define DH1_TYPE        0x4
#define DH1_2_TYPE      0x4
#define DH1_3_TYPE      0x8
#define HV1_TYPE        0x5
#define HV2_TYPE        0x6
#define EV3_2_TYPE      0x6
#define HV3_TYPE        0x7
#define EV3_TYPE        0x7
#define EV3_3_TYPE      0x7
#define DV_TYPE         0x8
#define AUX1_TYPE       0x9
#define DM3_TYPE        0xA
#define DH3_TYPE        0xB
#define DH3_2_TYPE      0xA
#define DH3_3_TYPE      0xB
#define EV4_TYPE        0xC
#define EV5_2_TYPE      0xC
#define EV5_TYPE        0xD
#define EV5_3_TYPE      0xD
#define DM5_TYPE        0xE
#define DH5_TYPE        0xF
#define DH5_2_TYPE      0xE
#define DH5_3_TYPE      0xF

uint8_t bt_pkt_mapping(uint8_t pkt_type, uint8_t *phy)
{

    uint8_t tx_type = 0xFF;


    switch (pkt_type)
    {
    case DTM_DM1:
    {
        tx_type = DM1_TYPE;
        *phy = 0x0;
    }
    break;
    case DTM_DH1:
    {
        tx_type = DH1_TYPE;
        *phy = 0x0;
    }
    break;
    case DTM_DM3:
    {
        tx_type = DM3_TYPE;
        *phy = 0x0;
    }
    break;
    case DTM_DH3:
    {
        tx_type = DH3_TYPE;
        *phy = 0x0;
    }
    break;
    case DTM_DM5:
    {
        tx_type = DM5_TYPE;
        *phy = 0x0;
    }
    break;
    case DTM_DH5:
    {
        tx_type = DH5_TYPE;
        *phy = 0x0;
    }
    break;
    case DTM_2DH1:
    {
        tx_type = DH1_2_TYPE;
        *phy = 0x1;
    }
    break;
    case DTM_2DH3:
    {
        tx_type = DH3_2_TYPE;
        *phy = 0x1;
    }
    break;
    case DTM_2DH5:
    {
        tx_type = DH5_2_TYPE;
        *phy = 0x1;
    }
    break;
    case DTM_3DH1:
    {
        tx_type = DH1_3_TYPE;
        *phy = 0x2;
    }
    break;
    case DTM_3DH3:
    {
        tx_type = DH3_3_TYPE;
        *phy = 0x2;
    }
    break;
    case DTM_3DH5:
    {
        tx_type = DH5_3_TYPE;
        *phy = 0x2;
    }
    break;
    default:
        break;
    }

    return tx_type;
}

static void bt_et_update();
__INLINE uint8_t bt_hop_infinite_get_stop_state(void);
void dm_isr(void)
{
    uint32_t irq_stat;
    irq_stat = hwp_bt_mac->ACTFIFOSTAT;
    //printf("DM IRQ\n");
    //irq_result = bt_irq_handler();
    //if(read_memory(0x50090024)&0x2)
    {
        hwp_bt_mac->DMINTACK0 = 0xffffffff;
    }
    hwp_bt_mac->DMINTACK1 = 0xffffffff;
    NVIC_ClearPendingIRQ(DM_MAC_IRQn);

    if ((irq_stat & BT_MAC_ACTFIFOSTAT_ENDACTINTSTAT) && (!bt_hop_infinite_get_stop_state()))
    {
        bt_et_update();
    }

};


static uint8_t is_stop;

void bt_hop_infinite_set_stop(uint8_t state)
{
    is_stop = state;
}

__INLINE uint8_t bt_hop_infinite_get_stop_state(void)
{
    return is_stop;
}

uint32_t tx_des0;
uint32_t tx_des1;
uint32_t addr = 0x204f0000;
static void bt_em_init(uint32_t data_len, uint32_t type, uint8_t is_edr)
{
    uint8_t i;

    for (i = 0; i < 0x10; i++)
    {
        write_memory(addr + i * 0x10, 0x00000000); //init extab
    }

    // rf table

    write_memory(addr, 0x00082801); //set extab 0, rawstp=8
    write_memory(addr + 0x004, 0x00000000);
    write_memory(addr + 0x008, 0x80020080); //csptr 0x200
    write_memory(addr + 0x00c, 0x04000402);

    write_memory(addr + 0x200, 0x00000002);
    if (is_edr)
    {
        write_memory(addr + 0x204, 0x29550000); //[27] acledr
    }
    else
    {
        write_memory(addr + 0x204, 0x21550000); //[27] acledr
    }
    write_memory(addr + 0x208, 0x004ca3bc);
    write_memory(addr + 0x20c, 0xb1ee0000);
    write_memory(addr + 0x210, 0xfff14080);
    write_memory(addr + 0x214, 0x88208007); //[7:0] txpower
    write_memory(addr + 0x218, 0x00a8001e); //[29:16] tx_desc=0x2a0
    write_memory(addr + 0x21c, 0xffffffff);
    write_memory(addr + 0x220, 0xffffffff);
    write_memory(addr + 0x224, 0x0303ffff);
    write_memory(addr + 0x228, 0x0000ffff);
    write_memory(addr + 0x22c, 0xad990000);
    write_memory(addr + 0x230, 0xa326521b);
    write_memory(addr + 0x234, 0x8e053e7e);
    write_memory(addr + 0x238, 0xc2278e3b);
    write_memory(addr + 0x23c, 0xab2466c6);
    write_memory(addr + 0x240, 0xbabebadc);
    write_memory(addr + 0x244, 0x0000deaf);
    write_memory(addr + 0x248, 0x00000000);
    write_memory(addr + 0x24c, 0x00000000);
    write_memory(addr + 0x250, 0x00000000);
    write_memory(addr + 0x254, 0x00000000);
    write_memory(addr + 0x258, 0x00000000);
    write_memory(addr + 0x25c, 0x00040000);

    tx_des0 = 0x000102a0 | type << 19;
    tx_des1 = 0x03000000 | data_len << 3;
    write_memory(addr + 0x2a0, tx_des0); //set tx descriptor, txtypye[22:19] DH5
    write_memory(addr + 0x2a4, tx_des1); //tx buffer 300 [12:3] txlength=339

    for (i = 0; i < 11; i++)
    {
        write_memory(addr + 0x300 + i * 4, 0xaaaaaaaa); //set tx buffer
    }

}

static void bt_et_update()
{
    uint32_t clkn = 0;

    hwp_bt_mac->SLOTCLK = 0x80000000;
    while (hwp_bt_mac->SLOTCLK & 0x80000000);
    clkn = (hwp_bt_mac->SLOTCLK & 0xfffffff) + 4;
    write_memory(addr + 0x000, (clkn << 16) | 0x2801); //set extab 0, rawstp=8
    write_memory(addr + 0x004, clkn >> 16);
    write_memory(addr + 0x008, 0x80020080); //csptr 0x200
    write_memory(addr + 0x00c, 0x04000402);

    write_memory(addr + 0x2a0, tx_des0); //set tx descriptor, txtypye[22:19] DH5
    write_memory(addr + 0x2a4, tx_des1); //tx buffer 300 [12:3] txlength=339

    hwp_bt_mac->ACTSCHCNTL = 0x80000000; //start_evt
}

uint8_t bt_123m_hop_infinite_tx(uint32_t data_len, uint32_t type, uint32_t is_edr)
{

    hwp_lpsys_rcc->CSR = 0x11; //select hxt48
    hwp_lpsys_rcc->CFGR = 0x00863241; //enable mem clock gating

    hwp_bt_phy->TX_IF_MOD_CFG2 = 0x00001c6c; //[5] mothed_edr [4] mothed br [3] mothed ble

    bt_em_init(data_len, type, (uint8_t)is_edr);

    hwp_bt_mac->BTRFTESTCNTL = 0x800; //RFTESTCNTL 0x8800 infinite tx/0x0800 burst tx
    hwp_bt_mac->RWBTCNTL = 0x00030100; //rwble_en whit_dsb crc_dsb
    hwp_bt_mac->ACTSCHCNTL = 0x80000000; //start_evt
    hwp_bt_mac->RWDMCNTL = 0x40000000; //reset clkn

    //configure interrupt
    NVIC_EnableIRQ(DM_MAC_IRQn);
    //dm_irq_enable(DM_IRQ_ERROR | DM_IRQ_FIFO);
    hwp_bt_mac->DMINTCNTL0 = 0x10000;
    hwp_bt_mac->DMINTCNTL1 = 0x8000;
    //bt_irq_disable(BT_IRQ_ALL);
    hwp_bt_mac->BTINTCNTL0 = 0x10002;
    //bt_irq_clear(BT_IRQ_ALL);
    hwp_bt_mac->BTINTACK0 = 0xffffffff;
    //bt_irq_enable(BT_IRQ_ERROR | BT_IRQ_END);

    return 0;
}


uint8_t bt_hop_infinite_tx(uint32_t phy, uint32_t data_len, uint32_t type, int8_t pwr)
{
    uint8_t ret = 0xFF;

    HAL_RCC_DisableModule(RCC_MOD_PTC2);
    HAL_RCC_Reset_and_Halt_LCPU(0);
    // Stop systick
    SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
#if 0
    extern void rf_power_set(int8_t tx_pwr, uint8_t is_edr);
    if (phy == 0x0)
        rf_power_set(10, 0);
    else
        // rf_power_set(10, 1);
#else
    void blebredr_rf_power_set(uint8_t type, int8_t txpwr);
    blebredr_rf_power_set(0, 10);
#endif
        hwp_bt_phy->TX_CTRL &= ~BT_PHY_TX_CTRL_MAC_PWR_CTRL_EN;

    if (phy == 0x0) // 1M
        ret = bt_123m_hop_infinite_tx(data_len, type, 0);
    else if ((phy == 0x1) || (phy == 0x2)) // 2M 3M
        ret = bt_123m_hop_infinite_tx(data_len, type, 1);

    while (!bt_hop_infinite_get_stop_state())
    {
        __WFI();
    }
    HAL_Delay(10);
    bt_hop_infinite_set_stop(0);
    hwp_bt_phy->TX_CTRL |= BT_PHY_TX_CTRL_MAC_PWR_CTRL_EN;

    NVIC_DisableIRQ(DM_MAC_IRQn);
    NVIC_ClearPendingIRQ(DM_MAC_IRQn);
    // Enable systick
    SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
    HAL_RCC_ResetModule(RCC_MOD_PTC2);
    HAL_RCC_ReleaseLCPU();
    HAL_Delay(10);
    //HAL_RCC_EnableModule(RCC_MOD_PTC2);
    return ret;
}
