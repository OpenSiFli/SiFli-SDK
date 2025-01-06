/**
  ******************************************************************************
  * @file   bt_rf_test.c
  * @author Sifli software development team
  * @brief Header file - cw config.
 *
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2023 - 2023,  Sifli Technology
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
#include "rtconfig.h"
#ifdef BSP_USING_RTTHREAD
#include <rtthread.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <board.h>
#include "bf0_hal.h"
#include "ble_rf_cal.h"

#define BT_EM_OFFSET_ADDR       0x0000
#define BT_EM_BASE_ADDR         (0x20408000+BT_EM_OFFSET_ADDR)
#define BLE_EM_OFFSET_ADDR       0x0000
#define BLE_EM_BASE_ADDR        BT_EM_BASE_ADDR

#define BT_EM_CS0               0x2E14
#define BT_EM_RXDESCRIPTOR0     0x30b4
#define BT_EM_RXDESCRIPTOR1     0x30c4
#define BT_EM_RXDATABUFFER0     0x3468 //+ 5
#define BT_EM_RXDATABUFFER1     0x386c
#define BT_EM_TXDESCRIPTOR0     0x30d4

#define BT_BER_DEBUG_EN    0


/// Test mode packet types
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
enum bt_irq
{
    BT_IRQ_ALL        = 0x1ff17,
    BT_IRQ_ERROR      = 0x10000,
    BT_IRQ_AUDIO2     = 0x08000,
    BT_IRQ_AUDIO1     = 0x04000,
    BT_IRQ_AUDIO0     = 0x02000,
    BT_IRQ_MWSWCIRX   = 0x01000,
    BT_IRQ_MWSWCITX   = 0x00800,
    BT_IRQ_MTOFF1     = 0x00400,
    BT_IRQ_MTOFF0     = 0x00200,
    BT_IRQ_FRSYNC     = 0x00100,
    BT_IRQ_RX         = 0x00010,
    BT_IRQ_SKIP       = 0x00004,
    BT_IRQ_END        = 0x00002,
    BT_IRQ_START      = 0x00001
};
typedef enum bt_irq bt_irq_t;

enum dm_irq
{
    DM_IRQ_ALL        = 0x181ff,
    DM_IRQ_ERROR      = 0x10000,
    DM_IRQ_FIFO       = 0x08000,
    DM_IRQ_RCCAL      = 0x00100,
    DM_IRQ_TIMESTAMP3 = 0x00080,
    DM_IRQ_TIMESTAMP2 = 0x00040,
    DM_IRQ_TIMESTAMP1 = 0x00020,
    DM_IRQ_FINETGTIM  = 0x00010,
    DM_IRQ_SW         = 0x00008,
    DM_IRQ_CRYPT      = 0x00004,
    DM_IRQ_SLP        = 0x00002,
    DM_IRQ_CLKN       = 0x00001
};
typedef enum dm_irq dm_irq_t;

typedef struct
{
    uint32_t wrong_bit;
    uint32_t total_bit;
    uint32_t wrong_pkt;
    uint32_t total_pkt;
    uint32_t irq_cnt;
    uint16_t pkt_len;
    uint8_t  pkt_payload;
    volatile uint8_t  stop_en;
    volatile uint8_t  stop_fin;
    volatile uint8_t  start_en;
} bt_nosignal_result_t;

bt_nosignal_result_t g_nosignal_result =
{
    .start_en = 0,
    .stop_en  = 0,
};


void dm_irq_clear(uint32_t dm_irq);

#if BT_BER_DEBUG_EN
    uint32_t g_test_debug[4];
#endif
uint8_t ble_irq_handler(void)
{
    uint32_t irq_stat, err, err2, stat2;
    uint8_t ret = 0;

    //while(1);
#if BT_BER_DEBUG_EN
    g_test_debug[0] = 0;
    g_test_debug[1] = 0;
    g_test_debug[2] = 0;
    g_test_debug[3] = 0;
#endif
    //debug = read_memory(0x40090054);
    err = hwp_bt_mac->DMINTSTAT0;
    err2 = hwp_bt_mac->DMERRORTYPESTAT;
    stat2 = hwp_bt_mac->DMINTSTAT1;
    if (err)
    {
        ret = 1;
        //while(1);
    }
    irq_stat = hwp_bt_mac->ACTFIFOSTAT;
    if (irq_stat & BT_MAC_ACTFIFOSTAT_ENDACTINTSTAT)
    {
        dm_irq_clear(0x81ff);
        NVIC_ClearPendingIRQ(DM_MAC_IRQn);
    }
    else if (irq_stat & BT_MAC_ACTFIFOSTAT_RXINTSTAT)
    {
        dm_irq_clear(0x81ff);
        NVIC_ClearPendingIRQ(DM_MAC_IRQn);
    }

    if ((g_nosignal_result.start_en) && (irq_stat & (BT_MAC_ACTFIFOSTAT_ENDACTINTSTAT | BT_MAC_ACTFIFOSTAT_RXINTSTAT)))
    {
#if BT_BER_DEBUG_EN
        g_test_debug[0] = err;
        g_test_debug[1] = err2;
        g_test_debug[2] = irq_stat;
        g_test_debug[3] = stat2;
#endif
        //rt_kprintf("dm err:0x%x, err2:0x%x irq:0x%x debug:0x%x\n", err, err2, irq_stat, debug);
        extern void bt_nosignal_evt_send(void);
        bt_nosignal_evt_send();
    }

    return ret;
}
__WEAK void bt_nosignal_evt_send(void)
{
}

void dm_isr(void)
{
    ble_irq_handler();
};

uint32_t bt_get_clkn(void)
{
    hwp_bt_mac->SLOTCLK |= BT_MAC_SLOTCLK_SAMP;
    while (hwp_bt_mac->SLOTCLK & BT_MAC_SLOTCLK_SAMP_Msk);
    return hwp_bt_mac->SLOTCLK & BT_MAC_SLOTCLK_SCLK_Msk;
}

void ble_em_extab_set(uint8_t index, uint8_t mode, uint8_t status, uint8_t iso,
                      uint8_t rsvd, uint8_t ae_nps, uint8_t sic, uint8_t spa,
                      uint8_t sch_prio1, uint32_t rawstp, uint16_t finestp, uint16_t csptr,
                      uint8_t prio1d, uint8_t prio1d_unit, uint8_t sch_prio2,
                      uint8_t sch_prio3, uint8_t isochan, uint8_t pti_prio)
{
    uint32_t addr;
    uint32_t write_value;
    addr = BLE_EM_BASE_ADDR + (index << 4);
    write_value = (uint32_t)mode         | ((uint32_t)status << 3)     |
                  ((uint32_t)iso << 6)      | ((uint32_t)rsvd << 7)       |
                  ((uint32_t)ae_nps << 8)   | ((uint32_t)sic << 9)        |
                  ((uint32_t)spa << 10)     | ((uint32_t)sch_prio1 << 11) |
                  ((uint32_t)rawstp << 16);
    write_memory(addr, write_value); //0x0
    write_value = (uint32_t)rawstp >> 16 | ((uint32_t)finestp << 16);
    write_memory(addr + 0x4, write_value); //0x4
    write_value = (uint32_t)(csptr + BLE_EM_OFFSET_ADDR) >> 2 | ((uint32_t)prio1d << 16) |
                  ((uint32_t)prio1d_unit << 31);
    write_memory(addr + 0x8, write_value); //0x8
    write_value = (uint32_t)sch_prio2    | ((uint32_t)sch_prio3 << 8)  |
                  ((uint32_t)pti_prio << 24);
    write_memory(addr + 0xc, write_value); //0xc
}
void bt_em_cs_init(uint32_t base_addr)
{
    uint32_t addr;
    addr = BT_EM_BASE_ADDR + base_addr;
    write_memory(addr + 0x00, 0x00000802);
    write_memory(addr + 0x04, 0x21f50000);
    write_memory(addr + 0x08, 0x659e8b33);
    write_memory(addr + 0x0c, 0x3ae26ac1);
    write_memory(addr + 0x10, 0xfff23316);
    write_memory(addr + 0x14, 0x88288007);
    write_memory(addr + 0x18, ((BT_EM_TXDESCRIPTOR0 + BT_EM_OFFSET_ADDR) << 14) | 0x1e); //0x08a8001e);//tx_descriptor @ 0x22a0
    write_memory(addr + 0x1c, 0xffffffff);
    write_memory(addr + 0x20, 0xffffffff);
    write_memory(addr + 0x24, 0x0303ffff);
    write_memory(addr + 0x28, 0x0000ffff);
    write_memory(addr + 0x2c, 0xad990000); //SK:66c6_c227_8e3b_8e05_3e7e_a326_521b_ad99
    write_memory(addr + 0x30, 0xa326521b);
    write_memory(addr + 0x34, 0x8e053e7e);
    write_memory(addr + 0x38, 0xc2278e3b);
    write_memory(addr + 0x3c, 0xab2466c6);
    write_memory(addr + 0x40, 0xbabebadc); //IV:deaf_babe_badc_ab24
    write_memory(addr + 0x44, 0x0000deaf);
    write_memory(addr + 0x48, 0x00000000);
    write_memory(addr + 0x4c, 0x00000000);
    write_memory(addr + 0x50, 0x00000000);
    write_memory(addr + 0x54, 0x00000000);
    write_memory(addr + 0x58, 0x00000000);
    write_memory(addr + 0x5c, 0x00040000);
}

void write_field(uint32_t addr, uint32_t value, uint32_t mask)
{
    uint32_t read_value;
    uint32_t write_value;
    uint8_t  i;
    uint32_t shift_value;

    read_value = read_memory(addr);
    i = 0;
    shift_value = 0x1;
    //search field position
    while (i < 32)
    {
        if (shift_value & mask)
        {
            write_value = read_value & (~mask) | ((value << i) & mask);
            break;
        }
        shift_value = shift_value << 1;
        i++;
    }
    if (i < 32)
    {
        write_memory(addr, write_value);
    }
}

void bt_em_cs_set_format(uint32_t base_addr, uint8_t format)
{
    uint32_t addr;
    uint32_t mask;
    addr = BT_EM_BASE_ADDR + base_addr + 0x0;
    mask = 0x0000001f;
    write_field(addr, (uint32_t)format, mask);
}

void bt_em_rx_descriptor_set(uint32_t base_addr, uint16_t nextptr, uint16_t rxaclbufptr, uint16_t rxlmbufptr)
{
    uint32_t addr;
    uint32_t mask;
    uint32_t write_value;
    addr = BT_EM_BASE_ADDR + base_addr;
    write_value = (uint32_t)((nextptr + BT_EM_OFFSET_ADDR) >> 2);
    write_memory(addr, write_value);
    write_value = (uint32_t)(rxaclbufptr + BT_EM_OFFSET_ADDR);
    mask = 0xffff0000;
    write_field(addr + 0x8, write_value, mask);
    write_value = (uint32_t)(rxlmbufptr + BT_EM_OFFSET_ADDR);
    mask = 0x0000ffff;
    write_field(addr + 0xc, write_value, mask);
}
void bt_set_currentrxdescptr(uint16_t value)
{
    hwp_bt_mac->BTCURRENTRXDESCPTR = (value + BT_EM_OFFSET_ADDR) >> 2;
}
void bt_start_act(uint8_t entry_idx)
{
    hwp_bt_mac->ACTSCHCNTL = BT_MAC_ACTSCHCNTL_START_ACT | entry_idx;
}

typedef struct
{
    uint32_t dmintcntl0;
    uint32_t dmintcntl1;
    uint32_t btintcntl0;
} btdm_irq_t;

btdm_irq_t  g_btdm_irt_msk;

void btdm_register_save(void)
{
    g_btdm_irt_msk.dmintcntl0 = hwp_bt_mac->DMINTCNTL0;
    g_btdm_irt_msk.dmintcntl1 = hwp_bt_mac->DMINTCNTL1;
    g_btdm_irt_msk.btintcntl0 = hwp_bt_mac->BTINTCNTL0;
}

void btdm_register_restore(void)
{
    hwp_bt_mac->DMINTCNTL0 = g_btdm_irt_msk.dmintcntl0;
    hwp_bt_mac->DMINTCNTL1 = g_btdm_irt_msk.dmintcntl1;
    hwp_bt_mac->BTINTCNTL0 = g_btdm_irt_msk.btintcntl0;
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_FORCE_CHANNEL;
}

void dm_irq_enable(uint32_t dm_irq)
{
    hwp_bt_mac->DMINTCNTL0 |= dm_irq;
    hwp_bt_mac->DMINTCNTL1 |= dm_irq;
}

void dm_irq_disable(uint32_t dm_irq)
{
    hwp_bt_mac->DMINTCNTL0 &= ~dm_irq;
    hwp_bt_mac->DMINTCNTL1 &= ~dm_irq;
}

void dm_irq_clear(uint32_t dm_irq)
{
    hwp_bt_mac->DMINTACK0 = dm_irq;
    hwp_bt_mac->DMINTACK1 = dm_irq;
}

void bt_irq_enable(uint32_t bt_irq)
{
    hwp_bt_mac->BTINTCNTL0 |= bt_irq;
}

void bt_irq_disable(uint32_t bt_irq)
{
    hwp_bt_mac->BTINTCNTL0 &= ~bt_irq;
}

void bt_irq_clear(uint32_t bt_irq)
{
    hwp_bt_mac->BTINTACK0 = bt_irq;
}

void wait(uint32_t cycle)
{
    for (uint32_t i = 0; i < cycle; i++)
    {
        __NOP();
    }
}


static uint8_t bt_bredr_judge(uint8_t pkt_type)
{
    uint8_t is_edr;

    switch (pkt_type)
    {
    case DTM_2DH1:
    case DTM_3DH1:
    case DTM_2DH3:
    case DTM_3DH3:
    case DTM_2DH5:
    case DTM_3DH5:
    case DTM_2EV3:
    case DTM_3EV3:
    case DTM_2EV5:
    case DTM_3EV5:
        is_edr = 1;
        break;
    default:
        is_edr = 0;
    }

    return is_edr;
}


enum bt_cs_format
{
    BT_CS_FORMAT_MASTER_CONNECT       = 0x2,
    BT_CS_FORMAT_SLAVE_CONNECT        = 0x3,
    BT_CS_FORMAT_PAGE                 = 0x4,
    BT_CS_FORMAT_PAGE_SCAN            = 0x5,
    BT_CS_FORMAT_MASTER_PAGE_RESPONSE = 0x6,
    BT_CS_FORMAT_SLAVE_PAGE_RESPONSE  = 0x7,
    BT_CS_FORMAT_INQUIRY              = 0x8,
    BT_CS_FORMAT_INQUIRY_RESPONSE     = 0x9,
    BT_CS_FORMAT_BROADCAST            = 0x18,
    BT_CS_FORMAT_BROADCAST_SCAN       = 0x19,
    BT_CS_FORMAT_TX_DIRECT_MODE       = 0x1E,
    BT_CS_FORMAT_RX_DIRECT_MODE       = 0x1F
};
typedef enum bt_cs_format bt_cs_format_t;


static void result_reset(void)
{
    g_nosignal_result.irq_cnt = 0;
    g_nosignal_result.total_bit = 0;
    g_nosignal_result.total_pkt = 0;
    g_nosignal_result.wrong_bit = 0;
    g_nosignal_result.wrong_pkt = 0;
    g_nosignal_result.stop_en = 0;
    g_nosignal_result.stop_fin = 0;
}

static void bt_mac_enable(void)
{
    hwp_bt_mac->RWBTCNTL |= BT_MAC_RWBTCNTL_RWBTEN;
}

/**
  * @brief  single-carrier test
  * @param  start or stop test
  * @param  power value, 0:0dbm 1:4dbm  2:10dbm
  * @param  set channel N, means 2402+*N MHz
  */
#define read_memory(addr)        (*(volatile unsigned int *)((addr)))
//#define write_memory(addr,value) (*(volatile unsigned int *)((addr))) = (value)
void mac_config(void)
{
#if 0
    ip_radiocntl0_pack(/*uint8_t  phytxondelay*/    0x20,
            /*uint8_t  phyrxondelay*/    0x3c,//0x20
            /*uint8_t  phyrftxoffdelay */0x10,
            /*uint8_t  phyrfrxoffdelay*/ 0x00);
    bt_radiopwrupdn_rxpwrupct_setf(0x50);
#endif
    uint32_t dat0 = read_memory(0x40090070);

    dat0 &= ~(0xFF << 16);
    dat0 |= ((uint32_t)0x52 << 16);//((uint32_t)0x4e << 16);//0x52//0x56
    write_memory(0x40090070, dat0);

    dat0 = read_memory(0x4009048c);

    dat0 &= ~(0xFF << 16);
    dat0 |= ((uint32_t)0x50 << 16);
    write_memory(0x4009048c, dat0);


}
void bt_nosignal_start(uint8_t channel, uint8_t pkt_type, uint8_t pkt_payload, uint16_t pkt_len)
{
    uint8_t acledr;
    uint32_t clkncnt;
    uint16_t finecnt;
    uint64_t bdaddr = 0x3003000cf00f;
    uint64_t bch = 0x20838976e;
    rt_base_t int_save;

    result_reset();
    g_nosignal_result.pkt_len = pkt_len;
    g_nosignal_result.pkt_payload = pkt_payload;

    btdm_register_save();

    //debug
    //write_memory(0x40090050, 0x98);
    HAL_RCC_Reset_and_Halt_LCPU(0);
    // Stop systick
    SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);

    mac_config();
    //write_memory(0xE000E184, 0xFFFFFFFF);
    //write_memory(0xE000E188, 0xFFFFFFFF);

    acledr = bt_bredr_judge(pkt_type);

    //int_save = rt_hw_interrupt_disable();

    clkncnt = bt_get_clkn();
    finecnt = (hwp_bt_mac->FINETIMECNT);

    //slot1->CS0
    ble_em_extab_set(0x1,        //index
                     0x1,        //mode
                     0x0,        //status
                     0x0,        //iso
                     0x0,        //rsvd
                     0x0,        //ae_nps
                     0x0,        //isobufsel
                     0x0,        //spa
                     0x5,        //sch_prio1
                     clkncnt + 20,      //rawstp  600000
                     finecnt,        //finestp
                     BT_EM_CS0, //csptr
                     0x2,        //prio1d
                     0x1,        //prio1d_unit
                     0x2,        //sch_prio2
                     0x4,        //sch_prio3
                     0x0,        //ioschan
                     0x4         //pti_prio
                    );

    //build CS0
    bt_em_cs_init(BT_EM_CS0);
    bt_em_cs_set_format(BT_EM_CS0, BT_CS_FORMAT_SLAVE_CONNECT);
    write_field(BT_EM_BASE_ADDR + BT_EM_CS0 + 0x14, 0x1, 0xe0000000);
    write_field(BT_EM_BASE_ADDR + BT_EM_CS0 + 0x18, 0x8032, 0xffff); // wide window  0x9fff
    hwp_bt_mac->RWBTCNTL |= BT_MAC_RWBTCNTL_WHITDSB;
    hwp_bt_mac->RWBTCNTL |= BT_MAC_RWBTCNTL_SEQNDSB;
    hwp_bt_mac->BTSWPROFILING = 0x0;
    write_memory(BT_EM_BASE_ADDR + BT_EM_CS0 + 0x8, bdaddr);
    write_field(BT_EM_BASE_ADDR + BT_EM_CS0 + 0xc, bdaddr >> 32, 0xffff);
    write_field(BT_EM_BASE_ADDR + BT_EM_CS0 + 0xc, bch & 0xffff, 0xffff0000);
    write_field(BT_EM_BASE_ADDR + BT_EM_CS0 + 0x10, bch >> 16, 0x3ffff);

    write_field(BT_EM_BASE_ADDR + BT_EM_CS0 + 0x4, acledr, 0x8000000);

    hwp_bt_mac->DMRADIOCNTL1 |= BT_MAC_DMRADIOCNTL1_FORCE_CHANNEL;
    hwp_bt_mac->DMRADIOCNTL1 &= ~BT_MAC_DMRADIOCNTL1_CHANNEL;
    hwp_bt_mac->DMRADIOCNTL1 |= (channel << BT_MAC_DMRADIOCNTL1_CHANNEL_Pos);
    //hwp_bt_mac->DMRADIOCNTL1 |= BT_MAC_DMRADIOCNTL1_FORCE_SYNCWORD;
    //hwp_bt_mac->DMRADIOCNTL2 &= ~BT_MAC_DMRADIOCNTL2_SYNCWORD1;
    //hwp_bt_mac->DMRADIOCNTL2 |= (0x4080b1ee << BT_MAC_DMRADIOCNTL2_SYNCWORD1_Pos);
    //hwp_bt_mac->DMRADIOCNTL3 &= ~BT_MAC_DMRADIOCNTL3_SYNCWORD2;
    //hwp_bt_mac->DMRADIOCNTL3 |= (0xb1328ef1 << BT_MAC_DMRADIOCNTL2_SYNCWORD1_Pos);

    //set_rx0
    bt_em_rx_descriptor_set(BT_EM_RXDESCRIPTOR0,  //base_addr
                            BT_EM_RXDESCRIPTOR1, //nextptr
                            BT_EM_RXDATABUFFER0,  //rxdataptr
                            BT_EM_RXDATABUFFER0
                           );
    //set_rx1
    bt_em_rx_descriptor_set(BT_EM_RXDESCRIPTOR1,  //base_addr
                            BT_EM_RXDESCRIPTOR0, //nextptr
                            BT_EM_RXDATABUFFER1,  //rxdataptr
                            BT_EM_RXDATABUFFER1
                           );
    //point to rx0
    bt_set_currentrxdescptr(BT_EM_RXDESCRIPTOR0);

    //turn on BLE core
    bt_mac_enable();
    //while(1);
    //__asm__("B .");
    bt_start_act(1);

    //configure interrupt
    //configure interrupt
    NVIC_EnableIRQ(DM_MAC_IRQn);
    dm_irq_enable(DM_IRQ_ERROR | DM_IRQ_FIFO);
    bt_irq_disable(BT_IRQ_ALL);
    bt_irq_clear(BT_IRQ_ALL);
    bt_irq_enable(BT_IRQ_ERROR | BT_IRQ_RX);
    g_nosignal_result.start_en = 1;

    //rt_hw_interrupt_enable(int_save);

}

static const uint32_t g_prbs9[] =
{
    0xe8fbc1ff, 0x8b72904c, 0x8951b3e7, 0x2323ab63, 0x72188402, 0x3b2f61aa, 0x37e5a851, 0xcac9fb49, 0x2c53180c, 0x9ae345fd, 0xb05df1e6, 0xbeb41bb6, 0xe9ea502a, 0x5e4b9c0e, 0xa1cc2457, 0x87b859b7,
    0x747de0ff, 0xc5b94826, 0xc4a8d9f3, 0x1191d5b1, 0x390c4201, 0x9d97b0d5, 0x9bf2d428, 0x6564fda4, 0x96298c06, 0x4d71a2fe, 0x582ef8f3, 0x5f5a0ddb, 0x74f52815, 0xaf25ce07, 0xd0e6122b, 0xc3dc2cdb,
    0x3a3ef07f, 0xe2dca413, 0xe2546cf9, 0x88c8ead8, 0x9c862100, 0x4ecbd86a, 0x4df96a14, 0x32b27ed2, 0x4b14c603, 0xa6b8d17f, 0xac177c79, 0xafad06ed, 0xba7a940a, 0xd792e703, 0xe8730915, 0xe1ee166d,
    0x9d1f783f, 0xf16e5209, 0x712a367c, 0x4464756c, 0x4e431080, 0x2765ec35, 0x26fcb50a, 0x99593f69, 0xa58a6301, 0xd35c68bf, 0xd60bbe3c, 0x57d68376, 0xdd3d4a05, 0xebc97381, 0xf439848a, 0xf0f70b36,
    0xce8fbc1f, 0x78b72904, 0x38951b3e, 0x22323ab6, 0xa7218840, 0x13b2f61a, 0x937e5a85, 0xccac9fb4, 0xd2c53180, 0x69ae345f, 0x6b05df1e, 0xabeb41bb, 0xee9ea502, 0x75e4b9c0, 0x7a1cc245, 0xf87b859b,
    0x6747de0f, 0x3c5b9482, 0x1c4a8d9f, 0x11191d5b, 0x5390c420, 0x89d97b0d, 0x49bf2d42, 0x66564fda, 0xe96298c0, 0x34d71a2f, 0xb582ef8f, 0x55f5a0dd, 0x774f5281, 0xbaf25ce0, 0xbd0e6122, 0xfc3dc2cd,
    0x33a3ef07, 0x9e2dca41, 0x8e2546cf, 0x088c8ead, 0xa9c86210, 0x44ecbd86, 0x24df96a1, 0x332b27ed, 0xf4b14c60, 0x9a6b8d17, 0xdac177c7, 0xaafad06e, 0x3ba7a940, 0x5d792e70, 0xde873091, 0xfe1ee166,
    0x99d1f783, 0xcf16e520, 0xc712a367, 0x04464756, 0x54e43108, 0xa2765ec3, 0x926fcb50, 0x199593f6, 0xfa58a630, 0xcd35c68b, 0x6d60bbe3, 0x557d6837, 0x1dd3d4a0, 0xaebc9738, 0x6f439848, 0xff0f70b3,
    0x4ce8fbc1, 0xe78b7290, 0x638951b3, 0x022323ab, 0xaa721884, 0x513b2f61, 0x4937e5a8, 0x0ccac9fb, 0xfd2c5318, 0xe69ae345, 0xb6b05df1, 0x2abeb41b, 0x0ee9ea50, 0x575e4b9c, 0xb7a1cc24, 0xff87b859,
    0x26747de0, 0xf3c5b948, 0xb1c4a8d9, 0x011191d5, 0xd5390c42, 0x289d97b0, 0xa49bf2d4, 0x066564fd, 0xfe96298c, 0xf34d71a2, 0xdb582ef8, 0x155f5a0d, 0x0774f528, 0x2baf25ce, 0xdbd0e612, 0x7fc3dc2c,
    0x133a3ef0, 0xf9e2dca4, 0xd8e2546c, 0x0088c8ea, 0x6a9c8621, 0x144ecbd8, 0xd24df96a, 0x0332b27e, 0x7f4b14c6, 0x79a6b8d1, 0xedac177c, 0x0aafad06, 0x03ba7a94, 0x15d792e7, 0x6de87309, 0x3fe1ee16,
    0x099d1f78, 0x7cf16e52, 0x6c712a36, 0x80446475, 0x354e4310, 0x0a2765ec, 0x6926fcb5, 0x0199593f, 0xbfa58a63, 0x3cd35c68, 0x76d60bbe, 0x0557d683, 0x81dd3d4a, 0x8aebc973, 0x36f43984, 0x1ff0f70b,
    0x04ce8fbc, 0x3e78b729, 0xb638951b, 0x4022323a, 0x1aa72188, 0x8513b2f6, 0xb4937e5a, 0x80ccac9f, 0x5fd2c531, 0x1e69ae34, 0xbb6b05df, 0x02abeb41, 0xc0ee9ea5, 0x4575e4b9, 0x9b7a1cc2, 0x0ff87b85,
    0x826747de, 0x9f3c5b94, 0x5b1c4a8d, 0x2011191d, 0x0d5390c4, 0x4289d97b, 0xda49bf2d, 0xc066564f, 0x2fe96298, 0x8f34d71a, 0xddb582ef, 0x8155f5a0, 0xe0774f52, 0x22baf25c, 0xcdbd0e61, 0x07fc3dc2,
    0x4133a3ef, 0xcf9e2dca, 0xad8e2546, 0x10088c8e, 0x86a9c862, 0xa144ecbd, 0xed24df96, 0x60332b27, 0x17f4b14c, 0xc79a6b8d, 0x6edac177, 0x40aafad0, 0x703ba7a9, 0x915d792e, 0x66de8730, 0x83fe1ee1,
    0x2099d1f7, 0x67cf16e5, 0x56c712a3, 0x08044647, 0xc354e431, 0x50a2765e, 0xf6926fcb, 0x30199593, 0x8bfa58a6, 0xe3cd35c6, 0x376d60bb, 0xa0557d68, 0x381dd3d4, 0x48aebc97, 0xb36f4398, 0xc1ff0f70,
};
static uint32_t count_diff_bit(uint32_t a, uint32_t b)
{
    uint32_t c = a ^ b;
    uint32_t count = 0;
    while (c)
    {
        c = c & (c - 1);
        count++;
    }
    return count;
}
static uint32_t count_diff_bit_part(uint32_t a, uint32_t b, uint8_t bnum)
{
    uint32_t c;
    uint32_t count = 0;
    uint32_t mask[4] = {0x0, 0xFF, 0xFFFF, 0xFFFFFF};

    c = (a ^ b) & mask[bnum];
    while (c)
    {
        c = c & (c - 1);
        count++;
    }
    return count;
}

static uint32_t mem_diff_cmp(uint32_t *rxdata, uint16_t len)
{
    uint16_t div = (len + 3) >> 2;
    uint16_t res = len & 0x3;
    uint16_t i;
    uint32_t data0, data1, diff = 0;
    uint32_t pkt_data[8] = {0x0, 0xF0F0F0F0, 0xAAAAAAAA, 0x0, 0xFFFFFFFF, 0x0, 0x0F0F0F0F, 0x55555555};

    for (i = 0; i < (div - 1); i++)
    {
        if (g_nosignal_result.pkt_payload == 0)
        {
            data0 = g_prbs9[i];
        }
        else
        {
            data0 = pkt_data[g_nosignal_result.pkt_payload];
        }
        data1 = *(rxdata + i);
        diff += count_diff_bit(data0, data1);
    }

    if (g_nosignal_result.pkt_payload == 0)
    {
        data0 = g_prbs9[i];
    }
    else
    {
        data0 = pkt_data[g_nosignal_result.pkt_payload];
    }

    data1 = *(rxdata + i);
    if (res)
    {
        diff += count_diff_bit_part(data0, data1, res);
    }
    else
    {
        diff += count_diff_bit(data0, data1);
    }

    return diff;
}

typedef struct
{
    uint32_t rxstat;
    uint32_t rxhdr;
    uint32_t rxchass;
    uint32_t dataptr;
    uint32_t err;
    uint32_t err2;
    uint32_t irq;
} bt_nosignal_debug_st;

#if BT_BER_DEBUG_EN
    bt_nosignal_debug_st  g_debug_btsig[1024];
    uint16_t g_debug_bt_index = 0;
#endif

static int8_t bt_nosignal_rx_proc(void)
{
    uint32_t rxstat, rxhdr, rxchass;
    uint16_t len;
    uint32_t dataptr;
    uint32_t *rxdata;
    int8_t rssi;

    if (g_nosignal_result.irq_cnt & 0x1)
    {
        rxstat = read_memory(BT_EM_BASE_ADDR + BT_EM_RXDESCRIPTOR1);
        rxhdr = read_memory(BT_EM_BASE_ADDR + BT_EM_RXDESCRIPTOR1 + 0x4);
        rxchass = read_memory(BT_EM_BASE_ADDR + BT_EM_RXDESCRIPTOR1 + 0x8);
        dataptr = BT_EM_BASE_ADDR + BT_EM_RXDATABUFFER1;

        //set_rx1
        bt_em_rx_descriptor_set(BT_EM_RXDESCRIPTOR1,  //base_addr
                                BT_EM_RXDESCRIPTOR0, //nextptr
                                BT_EM_RXDATABUFFER1,  //rxdataptr
                                BT_EM_RXDATABUFFER1
                               );
    }
    else
    {
        rxstat = read_memory(BT_EM_BASE_ADDR + BT_EM_RXDESCRIPTOR0);
        rxhdr = read_memory(BT_EM_BASE_ADDR + BT_EM_RXDESCRIPTOR0 + 0x4);
        rxchass = read_memory(BT_EM_BASE_ADDR + BT_EM_RXDESCRIPTOR1 + 0x8);
        dataptr = BT_EM_BASE_ADDR + BT_EM_RXDATABUFFER0;

        //set_rx0
        bt_em_rx_descriptor_set(BT_EM_RXDESCRIPTOR0,  //base_addr
                                BT_EM_RXDESCRIPTOR1, //nextptr
                                BT_EM_RXDATABUFFER0,  //rxdataptr
                                BT_EM_RXDATABUFFER0
                               );

    }
    g_nosignal_result.irq_cnt++;

#if BT_BER_DEBUG_EN
    //rt_kprintf("rx stat:0x%x, hdr:0x%x, chass:0x%x, dataptr:0x%x\n", rxstat, rxhdr, rxchass, dataptr);
    //if(g_debug_bt_index<1024)
    {
        g_debug_btsig[g_debug_bt_index].dataptr = dataptr;
        g_debug_btsig[g_debug_bt_index].rxchass = rxchass;
        g_debug_btsig[g_debug_bt_index].rxhdr = rxhdr;
        g_debug_btsig[g_debug_bt_index].rxstat = rxstat;
        g_debug_btsig[g_debug_bt_index].err = g_test_debug[0];
        g_debug_btsig[g_debug_bt_index].err2 = g_test_debug[1];
        g_debug_btsig[g_debug_bt_index].irq = g_test_debug[2];
        g_debug_bt_index++;
        if (g_debug_bt_index == 1024)
        {
            g_debug_bt_index = 0;
        }
    }
#endif

    if (0 == (rxstat & 0xfb0000))
    {
        g_nosignal_result.total_pkt++;
        len = ((rxhdr & 0x1ff80000) >> 19);
        if (len == g_nosignal_result.pkt_len)
        {
            g_nosignal_result.total_bit += (len << 3);
        }
        if (0 != (rxstat & 0x40000))
        {
            g_nosignal_result.wrong_pkt++;
            if (len == g_nosignal_result.pkt_len)
            {
                rxdata = (uint32_t *)rt_malloc(len + 3);
                memcpy((void *)rxdata, (void *)dataptr, len);
                g_nosignal_result.wrong_bit += mem_diff_cmp(rxdata, len);
                rt_free(rxdata);
            }
        }
    }

    rssi = (int8_t)(rxchass & 0xFF);

    return rssi;
}

int8_t bt_nosignal_irq(void)
{
    uint32_t clkncnt;
    uint16_t finecnt;
    rt_base_t int_save;
    int8_t rssi;

    //while(1);

    hwp_bt_mac->RWDMCNTL |= BT_MAC_RWDMCNTL_MASTER_SOFT_RST;
    wait(60);
    //rt_kprintf("th dm err:0x%x, err2:0x%x irq:0x%x debug:0x%x\n", g_test_debug[0], g_test_debug[1], g_test_debug[2], g_test_debug[3]);

    if (g_nosignal_result.stop_en)
    {
        g_nosignal_result.stop_fin = 1;
        return -128;
    }

    int_save = rt_hw_interrupt_disable();

    clkncnt = bt_get_clkn();
    finecnt = (hwp_bt_mac->FINETIMECNT);
    ble_em_extab_set(0x1,        //index
                     0x1,        //mode
                     0x0,        //status
                     0x0,        //iso
                     0x0,        //rsvd
                     0x0,        //ae_nps
                     0x0,        //isobufsel
                     0x0,        //spa
                     0x5,        //sch_prio1
                     clkncnt + 1,      //rawstp
                     finecnt,        //finestp
                     BT_EM_CS0, //csptr
                     0x2,        //prio1d
                     0x1,        //prio1d_unit
                     0x2,        //sch_prio2
                     0x4,        //sch_prio3
                     0x0,        //ioschan
                     0x4         //pti_prio
                    );
    bt_start_act(1);
    rt_hw_interrupt_enable(int_save);

    rssi = bt_nosignal_rx_proc();

    return rssi;
}

typedef struct
{
    uint32_t wrong_bit;
    uint32_t total_bit;
    uint32_t wrong_pkt;
    uint32_t total_pkt;
} bt_nosignal_ret_st;
void bt_nosignal_stop(uint8_t *data)
{
    //bt_nosignal_ret_st *ret_data;
    uint8_t *rslt = (uint8_t *)&g_nosignal_result.wrong_bit;

    g_nosignal_result.stop_en = 1;
    //while(g_nosignal_result.stop_fin==0)
    //{
    //  rt_thread_mdelay(50);
    //}
    NVIC_DisableIRQ(DM_MAC_IRQn);
    NVIC_ClearPendingIRQ(DM_MAC_IRQn);
    btdm_register_restore();
    g_nosignal_result.start_en = 0;
    HAL_RCC_Reset_and_Halt_LCPU(0);

    // Enable systick
    SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
    HAL_RCC_ReleaseLCPU();

#if 0
    ret_data = (bt_nosignal_ret_st *) data;
    ret_data->wrong_bit = g_nosignal_result.wrong_bit;
    ret_data->total_bit = g_nosignal_result.total_bit;
    ret_data->wrong_pkt = g_nosignal_result.wrong_pkt;
    ret_data->total_pkt = g_nosignal_result.total_pkt;
#endif
    for (int i = 0; i < 16; i++)
    {
        *(data + i) = *(rslt + i);
    }
}

uint8_t lcpu_reset_evt_handle(uint8_t *evt, uint16_t len)
{
    uint8_t evt_cmp_pattern[7] = {0x04, 0x0E, 0x04, 0x06, 0x11, 0xFC, 0x00};
    uint8_t ret = 0;

    if (g_nosignal_result.stop_en)
    {
        if (memcmp(evt_cmp_pattern, evt, 7) == 0)
        {
            g_nosignal_result.stop_en = 0;
            ret = 1;
        }
    }

    return ret;
}
#endif
/************************ (C) COPYRIGHT Sifli Technology *******END OF FILE****/
