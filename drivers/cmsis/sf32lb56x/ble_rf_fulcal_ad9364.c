
#include "register.h"
#include "ad9361.h"
#include "spi_tst_drv.h"

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

#ifndef SOC_SF32LB56X
    #define SPI_TEMP_LCPU_ADDRESS 0x204FA000
#else
    #define SPI_TEMP_LCPU_ADDRESS 0x2041F300
#endif

uint32_t spi_9364_table[80 * 4 * 2] =
{
    0x82750C, 0x8274CC, 0x8273CC, 0x827178,
    0x827513, 0x827433, 0x827333, 0x827178,
    0x827519, 0x827499, 0x827399, 0x827178,
    0x827520, 0x827400, 0x827300, 0x827178,
    0x827526, 0x827466, 0x827366, 0x827178,
    0x82752C, 0x8274CC, 0x8273CC, 0x827178,
    0x827533, 0x827433, 0x827333, 0x827178,
    0x827539, 0x827499, 0x827399, 0x827178,
    0x827540, 0x827400, 0x827300, 0x827178,
    0x827546, 0x827466, 0x827366, 0x827178,
    0x82754C, 0x8274CC, 0x8273CC, 0x827178,
    0x827553, 0x827433, 0x827333, 0x827178,
    0x827559, 0x827499, 0x827399, 0x827178,
    0x827560, 0x827400, 0x827300, 0x827178,
    0x827566, 0x827466, 0x827366, 0x827178,
    0x82756C, 0x8274CC, 0x8273CC, 0x827178,
    0x827573, 0x827433, 0x827333, 0x827178,
    0x827579, 0x827499, 0x827399, 0x827178,
    0x827500, 0x827400, 0x827300, 0x827179,
    0x827506, 0x827466, 0x827366, 0x827179,
    0x82750C, 0x8274CC, 0x8273CC, 0x827179,
    0x827513, 0x827433, 0x827333, 0x827179,
    0x827519, 0x827499, 0x827399, 0x827179,
    0x827520, 0x827400, 0x827300, 0x827179,
    0x827526, 0x827466, 0x827366, 0x827179,
    0x82752C, 0x8274CC, 0x8273CC, 0x827179,
    0x827533, 0x827433, 0x827333, 0x827179,
    0x827539, 0x827499, 0x827399, 0x827179,
    0x827540, 0x827400, 0x827300, 0x827179,
    0x827546, 0x827466, 0x827366, 0x827179,
    0x82754C, 0x8274CC, 0x8273CC, 0x827179,
    0x827553, 0x827433, 0x827333, 0x827179,
    0x827559, 0x827499, 0x827399, 0x827179,
    0x827560, 0x827400, 0x827300, 0x827179,
    0x827566, 0x827466, 0x827366, 0x827179,
    0x82756C, 0x8274CC, 0x8273CC, 0x827179,
    0x827573, 0x827433, 0x827333, 0x827179,
    0x827579, 0x827499, 0x827399, 0x827179,
    0x827500, 0x827400, 0x827300, 0x82717A,
    0x827506, 0x827466, 0x827366, 0x82717A,
    0x82750C, 0x8274CC, 0x8273CC, 0x82717A,
    0x827513, 0x827433, 0x827333, 0x82717A,
    0x827519, 0x827499, 0x827399, 0x82717A,
    0x827520, 0x827400, 0x827300, 0x82717A,
    0x827526, 0x827466, 0x827366, 0x82717A,
    0x82752C, 0x8274CC, 0x8273CC, 0x82717A,
    0x827533, 0x827433, 0x827333, 0x82717A,
    0x827539, 0x827499, 0x827399, 0x82717A,
    0x827540, 0x827400, 0x827300, 0x82717A,
    0x827546, 0x827466, 0x827366, 0x82717A,
    0x82754C, 0x8274CC, 0x8273CC, 0x82717A,
    0x827553, 0x827433, 0x827333, 0x82717A,
    0x827559, 0x827499, 0x827399, 0x82717A,
    0x827560, 0x827400, 0x827300, 0x82717A,
    0x827566, 0x827466, 0x827366, 0x82717A,
    0x82756C, 0x8274CC, 0x8273CC, 0x82717A,
    0x827573, 0x827433, 0x827333, 0x82717A,
    0x827579, 0x827499, 0x827399, 0x82717A,
    0x827500, 0x827400, 0x827300, 0x82717B,
    0x827506, 0x827466, 0x827366, 0x82717B,
    0x82750C, 0x8274CC, 0x8273CC, 0x82717B,
    0x827513, 0x827433, 0x827333, 0x82717B,
    0x827519, 0x827499, 0x827399, 0x82717B,
    0x827520, 0x827400, 0x827300, 0x82717B,
    0x827526, 0x827466, 0x827366, 0x82717B,
    0x82752C, 0x8274CC, 0x8273CC, 0x82717B,
    0x827533, 0x827433, 0x827333, 0x82717B,
    0x827539, 0x827499, 0x827399, 0x82717B,
    0x827540, 0x827400, 0x827300, 0x82717B,
    0x827546, 0x827466, 0x827366, 0x82717B,
    0x82754C, 0x8274CC, 0x8273CC, 0x82717B,
    0x827553, 0x827433, 0x827333, 0x82717B,
    0x827559, 0x827499, 0x827399, 0x82717B,
    0x827560, 0x827400, 0x827300, 0x82717B,
    0x827566, 0x827466, 0x827366, 0x82717B,
    0x82756C, 0x8274CC, 0x8273CC, 0x82717B,
    0x827573, 0x827433, 0x827333, 0x82717B,
    0x827579, 0x827499, 0x827399, 0x82717B,
    0x827500, 0x827400, 0x827300, 0x82717C,
    0x000000, 0x000000, 0x000000, 0x000000,
//rxpll table
    0x82350C, 0x8234CC, 0x8233CC, 0x823178,
    0x823513, 0x823433, 0x823333, 0x823178,
    0x823519, 0x823499, 0x823399, 0x823178,
    0x823520, 0x823400, 0x823300, 0x823178,
    0x823526, 0x823466, 0x823366, 0x823178,
    0x82352C, 0x8234CC, 0x8233CC, 0x823178,
    0x823533, 0x823433, 0x823333, 0x823178,
    0x823539, 0x823499, 0x823399, 0x823178,
    0x823540, 0x823400, 0x823300, 0x823178,
    0x823546, 0x823466, 0x823366, 0x823178,
    0x82354C, 0x8234CC, 0x8233CC, 0x823178,
    0x823553, 0x823433, 0x823333, 0x823178,
    0x823559, 0x823499, 0x823399, 0x823178,
    0x823560, 0x823400, 0x823300, 0x823178,
    0x823566, 0x823466, 0x823366, 0x823178,
    0x82356C, 0x8234CC, 0x8233CC, 0x823178,
    0x823573, 0x823433, 0x823333, 0x823178,
    0x823579, 0x823499, 0x823399, 0x823178,
    0x823500, 0x823400, 0x823300, 0x823179,
    0x823506, 0x823466, 0x823366, 0x823179,
    0x82350C, 0x8234CC, 0x8233CC, 0x823179,
    0x823513, 0x823433, 0x823333, 0x823179,
    0x823519, 0x823499, 0x823399, 0x823179,
    0x823520, 0x823400, 0x823300, 0x823179,
    0x823526, 0x823466, 0x823366, 0x823179,
    0x82352C, 0x8234CC, 0x8233CC, 0x823179,
    0x823533, 0x823433, 0x823333, 0x823179,
    0x823539, 0x823499, 0x823399, 0x823179,
    0x823540, 0x823400, 0x823300, 0x823179,
    0x823546, 0x823466, 0x823366, 0x823179,
    0x82354C, 0x8234CC, 0x8233CC, 0x823179,
    0x823553, 0x823433, 0x823333, 0x823179,
    0x823559, 0x823499, 0x823399, 0x823179,
    0x823560, 0x823400, 0x823300, 0x823179,
    0x823566, 0x823466, 0x823366, 0x823179,
    0x82356C, 0x8234CC, 0x8233CC, 0x823179,
    0x823573, 0x823433, 0x823333, 0x823179,
    0x823579, 0x823499, 0x823399, 0x823179,
    0x823500, 0x823400, 0x823300, 0x82317A,
    0x823506, 0x823466, 0x823366, 0x82317A,
    0x82350C, 0x8234CC, 0x8233CC, 0x82317A,
    0x823513, 0x823433, 0x823333, 0x82317A,
    0x823519, 0x823499, 0x823399, 0x82317A,
    0x823520, 0x823400, 0x823300, 0x82317A,
    0x823526, 0x823466, 0x823366, 0x82317A,
    0x82352C, 0x8234CC, 0x8233CC, 0x82317A,
    0x823533, 0x823433, 0x823333, 0x82317A,
    0x823539, 0x823499, 0x823399, 0x82317A,
    0x823540, 0x823400, 0x823300, 0x82317A,
    0x823546, 0x823466, 0x823366, 0x82317A,
    0x82354C, 0x8234CC, 0x8233CC, 0x82317A,
    0x823553, 0x823433, 0x823333, 0x82317A,
    0x823559, 0x823499, 0x823399, 0x82317A,
    0x823560, 0x823400, 0x823300, 0x82317A,
    0x823566, 0x823466, 0x823366, 0x82317A,
    0x82356C, 0x8234CC, 0x8233CC, 0x82317A,
    0x823573, 0x823433, 0x823333, 0x82317A,
    0x823579, 0x823499, 0x823399, 0x82317A,
    0x823500, 0x823400, 0x823300, 0x82317B,
    0x823506, 0x823466, 0x823366, 0x82317B,
    0x82350C, 0x8234CC, 0x8233CC, 0x82317B,
    0x823513, 0x823433, 0x823333, 0x82317B,
    0x823519, 0x823499, 0x823399, 0x82317B,
    0x823520, 0x823400, 0x823300, 0x82317B,
    0x823526, 0x823466, 0x823366, 0x82317B,
    0x82352C, 0x8234CC, 0x8233CC, 0x82317B,
    0x823533, 0x823433, 0x823333, 0x82317B,
    0x823539, 0x823499, 0x823399, 0x82317B,
    0x823540, 0x823400, 0x823300, 0x82317B,
    0x823546, 0x823466, 0x823366, 0x82317B,
    0x82354C, 0x8234CC, 0x8233CC, 0x82317B,
    0x823553, 0x823433, 0x823333, 0x82317B,
    0x823559, 0x823499, 0x823399, 0x82317B,
    0x823560, 0x823400, 0x823300, 0x82317B,
    0x823566, 0x823466, 0x823366, 0x82317B,
    0x82356C, 0x8234CC, 0x8233CC, 0x82317B,
    0x823573, 0x823433, 0x823333, 0x82317B,
    0x823579, 0x823499, 0x823399, 0x82317B,
    0x823500, 0x823400, 0x823300, 0x82317C,
    0x000000, 0x000000, 0x000000, 0x000000
};

void ad9364_bt_cfg()
{
    //initialization
    hwp_spi3->FIFO_CTRL |= SPI_FIFO_CTRL_TSRE;
    hwp_spi3->INTE |= SPI_INTE_TIE;

#ifndef SOC_SF32LB56X
    //memcpy((uint8_t *)SPI_TEMP_LCPU_ADDRESS, (uint8_t *)&spi_9364_table, sizeof(spi_9364_table));
    //write channel pointer table base address
#ifdef SOC_BF0_HCPU
    hwp_bt_mac->DMRADIOCNTL4 = ((uint32_t)(&spi_9364_table) + 0x0a000000);
#else
    hwp_bt_mac->DMRADIOCNTL4 = (uint32_t)&spi_9364_table;
#endif
    //hwp_bt_mac->DMRADIOCNTL4 = SPI_TEMP_LCPU_ADDRESS;
    //write_memory(BT_MAC_BASE + 0x80, (read_memory(BT_MAC_BASE + 0x80)));
    //dmac2 channel 5 for channel pointer

    hwp_dmac3->CM0AR5 = BT_MAC_BASE + 0x80;
    hwp_dmac3->CPAR5 = (uint32_t) & (hwp_dmac3->CM0AR4);
    hwp_dmac3->CNDTR5 = 0;
    hwp_dmac3->CCR5 = DMAC_CCR5_DIR | DMAC_CCR5_MINC | DMAC_CCR5_MEM2MEM;
    hwp_dmac3->CCR5 |= (0x2 << DMAC_CCR5_MSIZE_Pos);
    hwp_dmac3->CCR5 |= (0x2 << DMAC_CCR5_PSIZE_Pos);
    hwp_dmac3->CCR5 |= DMAC_CCR5_EN;
    //dmac2 channel 4 for spi3 tx
    hwp_dmac3->CM0AR4 = 0;
    hwp_dmac3->CPAR4  = SPI3_BASE + 0x10;
    hwp_dmac3->CNDTR4 = 0;
    hwp_dmac3->CSELR1 = 0;
    hwp_dmac3->CSELR1 &= ~DMAC_CSELR1_C4S_Pos;
    hwp_dmac3->CSELR1 |= (16 << DMAC_CSELR1_C4S_Pos);
    hwp_dmac3->CCR4 = DMAC_CCR4_DIR | DMAC_CCR4_MINC;
    hwp_dmac3->CCR4 |= (0x2 << DMAC_CCR4_MSIZE_Pos);
    hwp_dmac3->CCR4 |= (0x2 << DMAC_CCR4_PSIZE_Pos);
    hwp_dmac3->CCR4 |= DMAC_CCR4_EN;
#else
    //memcpy((uint8_t *)SPI_TEMP_LCPU_ADDRESS, (uint8_t *)&spi_9364_table, sizeof(spi_9364_table));
    //hwp_bt_mac->DMRADIOCNTL4 = ((uint32_t)(&spi_9364_table) + 0x0a000000);
    //hwp_bt_mac->DMRADIOCNTL4 = SPI_TEMP_LCPU_ADDRESS;
#ifdef SOC_BF0_HCPU
    hwp_bt_mac->DMRADIOCNTL4 = ((uint32_t)(&spi_9364_table) + 0x0a000000);
#else
    hwp_bt_mac->DMRADIOCNTL4 = (uint32_t)&spi_9364_table;
#endif
    hwp_dmac2->CM0AR5 = BT_MAC_BASE + 0x80;
    hwp_dmac2->CPAR5 = (uint32_t) & (hwp_dmac2->CM0AR4);
    hwp_dmac2->CNDTR5 = 0;
    hwp_dmac2->CCR5 = DMAC_CCR5_DIR | DMAC_CCR5_MINC | DMAC_CCR5_MEM2MEM;
    hwp_dmac2->CCR5 |= (0x2 << DMAC_CCR5_MSIZE_Pos);
    hwp_dmac2->CCR5 |= (0x2 << DMAC_CCR5_PSIZE_Pos);
    hwp_dmac2->CCR5 |= DMAC_CCR5_EN;
    //dmac2 hannel 4 for spi3 tx
    hwp_dmac2->CM0AR4 = 0;
    hwp_dmac2->CPAR4  = SPI3_BASE + 0x10;
    hwp_dmac2->CNDTR4 = 0;
    hwp_dmac2->CSELR1 = 0;
    hwp_dmac2->CSELR1 &= ~DMAC_CSELR1_C4S_Pos;
    hwp_dmac2->CSELR1 |= (16 << DMAC_CSELR1_C4S_Pos);
    hwp_dmac2->CCR4 = DMAC_CCR4_DIR | DMAC_CCR4_MINC;
    hwp_dmac2->CCR4 |= (0x2 << DMAC_CCR4_MSIZE_Pos);
    hwp_dmac2->CCR4 |= (0x2 << DMAC_CCR4_PSIZE_Pos);
    hwp_dmac2->CCR4 |= DMAC_CCR4_EN;
#endif
    //ptc controls gpio toggle
    hwp_gpio2->DOER = 0x43;
    hwp_ptc2->TCR1 = (PTC_OP_OR << PTC_TCR1_OP_Pos) | 99; //trigger ble_phytxstart
    hwp_ptc2->TAR1 = (uint32_t) & (hwp_gpio2->DOR);
    hwp_ptc2->TDR1 = 0x43; //set PB00,PB01,PB06
    hwp_ptc2->TCR2 = (PTC_OP_AND << PTC_TCR2_OP_Pos) | 100; //trigger ble_txdone
    hwp_ptc2->TAR2 = (uint32_t) & (hwp_gpio2->DOR);
    hwp_ptc2->TDR2 = 0xffffffbc; //clear PB00,PB01,PB06
    hwp_ptc2->TCR3 = (PTC_OP_OR << PTC_TCR3_OP_Pos) | 102; //trigger ble_phyrxstart
    hwp_ptc2->TAR3 = (uint32_t) & (hwp_gpio2->DOR);
    hwp_ptc2->TDR3 = 0x1; //set PB00
    hwp_ptc2->TCR4 = (PTC_OP_AND << PTC_TCR4_OP_Pos) | 103; //trigger ble_rxdone
    hwp_ptc2->TAR4 = (uint32_t) & (hwp_gpio2->DOR);
    hwp_ptc2->TDR4 = 0xffffffbc; //clear PB00,PB01,PB06

#ifndef SOC_SF32LB56X
    //ptc controls channel config
    hwp_ptc2->TCR5 = (PTC_OP_WRITE << PTC_TCR5_OP_Pos) | 98; //trigger ble_rftxstart
    hwp_ptc2->TAR5 = (uint32_t) & (hwp_dmac3->CNDTR5);
    hwp_ptc2->TDR5 = 0x1; //start dmac2 channel5
    hwp_ptc2->TCR6 = (PTC_OP_WRITE << PTC_TCR6_OP_Pos) | 28; //trigger dmac2_done5
    hwp_ptc2->TAR6 = (uint32_t) & (hwp_dmac3->CNDTR4);
    hwp_ptc2->TDR6 = 0x4; //start dmac2 channel4
    hwp_ptc2->TCR7 = (PTC_OP_WRITE << PTC_TCR7_OP_Pos) | 101; //trigger ble_rfrxstart
    hwp_ptc2->TAR7 = (uint32_t) & (hwp_dmac3->CNDTR5);
    hwp_ptc2->TDR7 = 0x1; //start dmac2 channel5
    hwp_ptc2->TCR8 = (PTC_OP_WRITE << PTC_TCR8_OP_Pos) | 28; //trigger dmac2_done5
    hwp_ptc2->TAR8 = (uint32_t) & (hwp_dmac3->IFCR);
    hwp_ptc2->TDR8 = DMAC_ISR_GIF5; //clear dmac2_done5
#else
    hwp_ptc2->TCR5 = (PTC_OP_WRITE << PTC_TCR5_OP_Pos) | 98; //trigger ble_rftxstart
    hwp_ptc2->TAR5 = (uint32_t) & (hwp_dmac2->CNDTR5);
    hwp_ptc2->TDR5 = 0x1; //start dmac2 channel5
    hwp_ptc2->TCR6 = (PTC_OP_WRITE << PTC_TCR6_OP_Pos) | 28; //trigger dmac2_done5
    hwp_ptc2->TAR6 = (uint32_t) & (hwp_dmac2->CNDTR4);
    hwp_ptc2->TDR6 = 0x4; //start dmac2 channel4
    hwp_ptc2->TCR7 = (PTC_OP_WRITE << PTC_TCR7_OP_Pos) | 101; //trigger ble_rfrxstart
    hwp_ptc2->TAR7 = (uint32_t) & (hwp_dmac2->CNDTR5);
    hwp_ptc2->TDR7 = 0x1; //start dmac2 channel5
    hwp_ptc2->TCR8 = (PTC_OP_WRITE << PTC_TCR8_OP_Pos) | 28; //trigger dmac2_done5
    hwp_ptc2->TAR8 = (uint32_t) & (hwp_dmac2->IFCR);
    hwp_ptc2->TDR8 = DMAC_ISR_GIF5; //clear dmac2_done5
#endif
}

void pinmux_spi3()
{
#ifndef SOC_SF32LB56X
    hwp_pinmux2->PAD_PB21 = 0x53; //clk
    hwp_pinmux2->PAD_PB40 = 0x53; //cs
    hwp_pinmux2->PAD_PB22 = 0x53; //di
    hwp_pinmux2->PAD_PB25 = 0x53; //do/dio
#else
    hwp_pinmux2->PAD_PB19 = 0x51; //clk
    hwp_pinmux2->PAD_PB18 = 0x51; //cs
    hwp_pinmux2->PAD_PB20 = 0x51; //di
    hwp_pinmux2->PAD_PB21 = 0x51; //do/dio

#endif
}



static void wait(uint32_t cycle)
{

    for (uint32_t i = 0; i < cycle; i++)
    {
        __NOP();
    }
}

void ad9364_spi_init()
{
#if 0
    // SPI init, should put into system.
    rt_err_t ret;
    g_spi_handle = rt_device_find("spi3");
    do
    {

        uint16_t oflag = RT_DEVICE_OFLAG_RDWR;
        RT_ASSERT(g_spi_handle);
        ret = rt_device_init(g_spi_handle);
        if (ret != RT_EOK)
            break;



        if (g_spi_handle->flag & RT_DEVICE_FLAG_DMA_RX)
        {
            oflag |= RT_DEVICE_FLAG_DMA_RX;
        }
        else
        {
            oflag |= RT_DEVICE_FLAG_INT_RX;
        }

        if (g_spi_handle->flag & RT_DEVICE_FLAG_DMA_TX)
        {
            oflag |= RT_DEVICE_FLAG_DMA_TX;
        }
        else
        {
            oflag |= RT_DEVICE_FLAG_INT_TX;
        }
        ret = rt_device_open(g_spi_handle, oflag);

        if (ret != RT_EOK)
            break;

        struct rt_spi_configuration cfg;
        cfg.data_width = 24;
        cfg.mode =  RT_SPI_CPHA;
        cfg.frameMode = RT_SPI_MOTO;
        cfg.max_hz =
    }
    while (0)
        RT_ASSERT(ret == RT_EOK);
#else

    pinmux_spi3();

    //24bit data width
    set_spi3_frm_width(24);

    //clk cfg
    //div 2 by default

    //sclk phase to 1, polarity stay 0
    //sclk is 0 when idle, and lanch data at posedge
    set_spi3_sph();

    //enable enable spi3
    enable_spi3_trx();

#endif
}



void ad9364_spi_write(uint16_t addr, uint8_t data)
{
#if 0
    uint32_t t_data = 0x800000 | ((uint32_t)addr << 8) | data;
    uint32_t r_data;
    rt_device_write(g_spi_handle, 0,  &t_data, 4);
    rt_device_write(g_spi_handle, 0,  &r_data, 4);
#else
    uint32_t tx_data;
    uint32_t rx_data;
    uint32_t spi3_rne;

    tx_data = 0x800000 | ((uint32_t)addr << 8) | data;
    set_spi3_tdata(tx_data);
    //clear rx fifo
    spi3_rne = hwp_spi3->STATUS & SPI_STATUS_RNE;
    while (!spi3_rne)
    {
        spi3_rne = hwp_spi3->STATUS & SPI_STATUS_RNE;
    }
    rx_data = get_spi3_rdata();
    wait(40);

#endif

}

uint8_t ad9364_spi_read(uint16_t addr)
{
#if 0
    uint32_t t_data = (uint32_t)addr << 8;
    uint32_t r_data;
    rt_device_write(g_spi_handle, 0,  &t_data, 4);
    rt_device_write(g_spi_handle, 0,  &r_data, 4);
    return (uint8_t)r_data;
#else
    uint32_t tx_data;
    uint32_t rx_data;
    uint32_t spi3_rne;

    tx_data = ((uint32_t)addr << 8);
    set_spi3_tdata(tx_data);
    //read rx fifo
    spi3_rne = hwp_spi3->STATUS & SPI_STATUS_RNE;
    while (!spi3_rne)
    {
        spi3_rne = hwp_spi3->STATUS & SPI_STATUS_RNE;
    }
    rx_data = get_spi3_rdata();
    return (uint8_t)rx_data;

#endif
}





// FPGA rf board init, should put to rf calibration

uint8_t ad9364_calibration()
{
    uint8_t test_result = TEST_UNFINISHED;
    //uint16_t i;
    //uint32_t rdata;

    ad9364_spi_init();
    ad9364_spi_write(REG_CTRL_OUTPUT_POINTER, 0xB);

    ad9364_spi_write(REG_CTRL, 0x1);
    ad9364_spi_write(REG_BANDGAP_CONFIG0, 0xE);
    ad9364_spi_write(REG_BANDGAP_CONFIG1, 0xE);
    ad9364_spi_write(REG_DCXO_COARSE_TUNE, 0x13);
    //ad9364_spi_write(REG_DCXO_FINE_TUNE_HIGH,0x80);
    //ad9364_spi_write(REG_DCXO_FINE_TUNE_LOW,0x0);
    ad9364_spi_write(REG_REF_DIVIDE_CONFIG_1, 0x7);
    ad9364_spi_write(REG_REF_DIVIDE_CONFIG_2, 0xFF);
    ad9364_spi_write(REG_CLOCK_ENABLE, 0x07);
    ad9364_spi_write(REG_BBPLL, 0x13);

    ad9364_spi_write(REG_ENSM_CONFIG_2, 0x04); //SET SYNTH DUAL MODE 0 AND TXNRX SPI CONTROL 1
    ad9364_spi_write(REG_ENSM_CONFIG_1, 0x1D);
    ad9364_spi_write(REG_ENSM_MODE, 0x0);
    //ad9364_spi_write(REG_INPUT_SELECT,0x40);
    //wait(20000);

    //-------------------------------------------------------
    ad9364_spi_write(0x045, 0x00); // Set BBPLL reflclk scale to REFCLK /1
    ad9364_spi_write(0x046, 0x02); // Set BBPLL Loop Filter Charge Pump current
    ad9364_spi_write(0x048, 0xE8); // Set BBPLL Loop Filter C1, R1
    ad9364_spi_write(0x049, 0x5B); // Set BBPLL Loop Filter R2, C2, C1
    ad9364_spi_write(0x04A, 0x35); // Set BBPLL Loop Filter C3,R2
    ad9364_spi_write(0x04B, 0xE0); // Allow calibration to occur and set cal count to 1024 for max accuracy
    ad9364_spi_write(0x04E, 0x10); // Set calibration clock to REFCLK/4 for more accuracy
    ad9364_spi_write(0x043, 0x00); // BBPLL Freq Word (Fractional[7:0])
    ad9364_spi_write(0x042, 0x60); // BBPLL Freq Word (Fractional[15:8])
    ad9364_spi_write(0x041, 0x06); // BBPLL Freq Word (Fractional[23:16])
    ad9364_spi_write(0x044, 0x13); // BBPLL Freq Word (Integer[7:0])
    ad9364_spi_write(0x03F, 0x05); // Start BBPLL Calibration
    ad9364_spi_write(0x03F, 0x01); // Clear BBPLL start calibration bit
    ad9364_spi_write(0x04C, 0x86); // Increase BBPLL KV and phase margin
    ad9364_spi_write(0x04D, 0x01); // Increase BBPLL KV and phase margin
    ad9364_spi_write(0x04D, 0x05); // Increase BBPLL KV and phase margin

    while (!(ad9364_spi_read(0x05E) & 0x80));

    ad9364_spi_write(0x002, 0x4e); // Setup Tx Digital Filters/ Channels
    ad9364_spi_write(0x003, 0x5d); // Setup Rx Digital Filters/ Channels
    ad9364_spi_write(0x004, 0x03); // Select Rx input pin(A,B,C)/ Tx out pin (A,B)
    ad9364_spi_write(0x00A, 0x12); // Set BBPLL post divide rate

    ad9364_spi_write(0x065, 0x08); // Tx Filter Configuration

    //************************************************************
    // Program Tx FIR: C:\Program Files (x86)\Analog Devices\AD9361R2
    // Evaluation Software 2.1.3\DigitalFilters\128tapFilter.ftr
    //************************************************************
    ad9364_spi_write(0x065, 0xFA); // Enable clock to Tx FIR Filter and set Filter gain Setting
    ad9364_spi_write(0x060, 0x00); // Write FIR coefficient address
    ad9364_spi_write(0x061, 0x01); // Write FIR coefficient data[7:0]
    ad9364_spi_write(0x062, 0x00); // Write FIR coefficient data[15:8]
    ad9364_spi_write(0x065, 0xFE); // Set Write EN to push data into FIR filter register map
    ad9364_spi_write(0x064, 0x00); // Write to Read only register to delay ~1us
    ad9364_spi_write(0x064, 0x00); // Write to Read only register to delay ~1us
    ad9364_spi_write(0x060, 0x01);
    ad9364_spi_write(0x061, 0xF1);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x02);
    ad9364_spi_write(0x061, 0xCF);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x03);
    ad9364_spi_write(0x061, 0xC0);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x04);
    ad9364_spi_write(0x061, 0xE8);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x05);
    ad9364_spi_write(0x061, 0x20);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x06);
    ad9364_spi_write(0x061, 0x1A);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x07);
    ad9364_spi_write(0x061, 0xE3);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x08);
    ad9364_spi_write(0x061, 0xE1);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x09);
    ad9364_spi_write(0x061, 0x1F);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x0A);
    ad9364_spi_write(0x061, 0x28);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x0B);
    ad9364_spi_write(0x061, 0xDF);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x0C);
    ad9364_spi_write(0x061, 0xCC);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x0D);
    ad9364_spi_write(0x061, 0x24);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x0E);
    ad9364_spi_write(0x061, 0x43);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x0F);
    ad9364_spi_write(0x061, 0xDB);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x10);
    ad9364_spi_write(0x061, 0xAC);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x11);
    ad9364_spi_write(0x061, 0x26);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x12);
    ad9364_spi_write(0x061, 0x68);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x13);
    ad9364_spi_write(0x061, 0xDB);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x14);
    ad9364_spi_write(0x061, 0x80);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x15);
    ad9364_spi_write(0x061, 0x22);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x16);
    ad9364_spi_write(0x061, 0x9A);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x17);
    ad9364_spi_write(0x061, 0xE2);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x18);
    ad9364_spi_write(0x061, 0x47);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x19);
    ad9364_spi_write(0x061, 0x17);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x1A);
    ad9364_spi_write(0x061, 0xDB);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x1B);
    ad9364_spi_write(0x061, 0xF3);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x1C);
    ad9364_spi_write(0x061, 0xFF);
    ad9364_spi_write(0x062, 0xFE);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x1D);
    ad9364_spi_write(0x061, 0xFF);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x1E);
    ad9364_spi_write(0x061, 0x2B);
    ad9364_spi_write(0x062, 0x01);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x1F);
    ad9364_spi_write(0x061, 0x13);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x20);
    ad9364_spi_write(0x061, 0xA5);
    ad9364_spi_write(0x062, 0xFE);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x21);
    ad9364_spi_write(0x061, 0xD7);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x22);
    ad9364_spi_write(0x061, 0x90);
    ad9364_spi_write(0x062, 0x01);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x23);
    ad9364_spi_write(0x061, 0x46);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x24);
    ad9364_spi_write(0x061, 0x35);
    ad9364_spi_write(0x062, 0xFE);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x25);
    ad9364_spi_write(0x061, 0x97);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x26);
    ad9364_spi_write(0x061, 0x0E);
    ad9364_spi_write(0x062, 0x02);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x27);
    ad9364_spi_write(0x061, 0x95);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x28);
    ad9364_spi_write(0x061, 0xA7);
    ad9364_spi_write(0x062, 0xFD);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x29);
    ad9364_spi_write(0x061, 0x36);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x2A);
    ad9364_spi_write(0x061, 0xAE);
    ad9364_spi_write(0x062, 0x02);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x2B);
    ad9364_spi_write(0x061, 0x0D);
    ad9364_spi_write(0x062, 0x01);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x2C);
    ad9364_spi_write(0x061, 0xF0);
    ad9364_spi_write(0x062, 0xFC);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x2D);
    ad9364_spi_write(0x061, 0xA1);
    ad9364_spi_write(0x062, 0xFE);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x2E);
    ad9364_spi_write(0x061, 0x83);
    ad9364_spi_write(0x062, 0x03);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x2F);
    ad9364_spi_write(0x061, 0xC6);
    ad9364_spi_write(0x062, 0x01);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x30);
    ad9364_spi_write(0x061, 0xF3);
    ad9364_spi_write(0x062, 0xFB);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x31);
    ad9364_spi_write(0x061, 0xB6);
    ad9364_spi_write(0x062, 0xFD);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x32);
    ad9364_spi_write(0x061, 0xB7);
    ad9364_spi_write(0x062, 0x04);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x33);
    ad9364_spi_write(0x061, 0xF8);
    ad9364_spi_write(0x062, 0x02);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x34);
    ad9364_spi_write(0x061, 0x6D);
    ad9364_spi_write(0x062, 0xFA);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x35);
    ad9364_spi_write(0x061, 0x1A);
    ad9364_spi_write(0x062, 0xFC);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x36);
    ad9364_spi_write(0x061, 0xBE);
    ad9364_spi_write(0x062, 0x06);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x37);
    ad9364_spi_write(0x061, 0x41);
    ad9364_spi_write(0x062, 0x05);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x38);
    ad9364_spi_write(0x061, 0x87);
    ad9364_spi_write(0x062, 0xF7);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x39);
    ad9364_spi_write(0x061, 0x98);
    ad9364_spi_write(0x062, 0xF8);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x3A);
    ad9364_spi_write(0x061, 0x60);
    ad9364_spi_write(0x062, 0x0B);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x3B);
    ad9364_spi_write(0x061, 0x6D);
    ad9364_spi_write(0x062, 0x0B);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x3C);
    ad9364_spi_write(0x061, 0x88);
    ad9364_spi_write(0x062, 0xEE);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x3D);
    ad9364_spi_write(0x061, 0x40);
    ad9364_spi_write(0x062, 0xEA);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x3E);
    ad9364_spi_write(0x061, 0x86);
    ad9364_spi_write(0x062, 0x27);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x3F);
    ad9364_spi_write(0x061, 0x09);
    ad9364_spi_write(0x062, 0x72);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x40);
    ad9364_spi_write(0x061, 0x09);
    ad9364_spi_write(0x062, 0x72);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x41);
    ad9364_spi_write(0x061, 0x86);
    ad9364_spi_write(0x062, 0x27);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x42);
    ad9364_spi_write(0x061, 0x40);
    ad9364_spi_write(0x062, 0xEA);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x43);
    ad9364_spi_write(0x061, 0x88);
    ad9364_spi_write(0x062, 0xEE);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x44);
    ad9364_spi_write(0x061, 0x6D);
    ad9364_spi_write(0x062, 0x0B);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x45);
    ad9364_spi_write(0x061, 0x60);
    ad9364_spi_write(0x062, 0x0B);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x46);
    ad9364_spi_write(0x061, 0x98);
    ad9364_spi_write(0x062, 0xF8);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x47);
    ad9364_spi_write(0x061, 0x87);
    ad9364_spi_write(0x062, 0xF7);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x48);
    ad9364_spi_write(0x061, 0x41);
    ad9364_spi_write(0x062, 0x05);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x49);
    ad9364_spi_write(0x061, 0xBE);
    ad9364_spi_write(0x062, 0x06);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x4A);
    ad9364_spi_write(0x061, 0x1A);
    ad9364_spi_write(0x062, 0xFC);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x4B);
    ad9364_spi_write(0x061, 0x6D);
    ad9364_spi_write(0x062, 0xFA);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x4C);
    ad9364_spi_write(0x061, 0xF8);
    ad9364_spi_write(0x062, 0x02);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x4D);
    ad9364_spi_write(0x061, 0xB7);
    ad9364_spi_write(0x062, 0x04);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x4E);
    ad9364_spi_write(0x061, 0xB6);
    ad9364_spi_write(0x062, 0xFD);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x4F);
    ad9364_spi_write(0x061, 0xF3);
    ad9364_spi_write(0x062, 0xFB);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x50);
    ad9364_spi_write(0x061, 0xC6);
    ad9364_spi_write(0x062, 0x01);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x51);
    ad9364_spi_write(0x061, 0x83);
    ad9364_spi_write(0x062, 0x03);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x52);
    ad9364_spi_write(0x061, 0xA1);
    ad9364_spi_write(0x062, 0xFE);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x53);
    ad9364_spi_write(0x061, 0xF0);
    ad9364_spi_write(0x062, 0xFC);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x54);
    ad9364_spi_write(0x061, 0x0D);
    ad9364_spi_write(0x062, 0x01);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x55);
    ad9364_spi_write(0x061, 0xAE);
    ad9364_spi_write(0x062, 0x02);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x56);
    ad9364_spi_write(0x061, 0x36);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x57);
    ad9364_spi_write(0x061, 0xA7);
    ad9364_spi_write(0x062, 0xFD);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x58);
    ad9364_spi_write(0x061, 0x95);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x59);
    ad9364_spi_write(0x061, 0x0E);
    ad9364_spi_write(0x062, 0x02);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x5A);
    ad9364_spi_write(0x061, 0x97);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x5B);
    ad9364_spi_write(0x061, 0x35);
    ad9364_spi_write(0x062, 0xFE);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x5C);
    ad9364_spi_write(0x061, 0x46);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x5D);
    ad9364_spi_write(0x061, 0x90);
    ad9364_spi_write(0x062, 0x01);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x5E);
    ad9364_spi_write(0x061, 0xD7);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x5F);
    ad9364_spi_write(0x061, 0xA5);
    ad9364_spi_write(0x062, 0xFE);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x60);
    ad9364_spi_write(0x061, 0x13);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x61);
    ad9364_spi_write(0x061, 0x2B);
    ad9364_spi_write(0x062, 0x01);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x62);
    ad9364_spi_write(0x061, 0xFF);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x63);
    ad9364_spi_write(0x061, 0xFF);
    ad9364_spi_write(0x062, 0xFE);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x64);
    ad9364_spi_write(0x061, 0xF3);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x65);
    ad9364_spi_write(0x061, 0xDB);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x66);
    ad9364_spi_write(0x061, 0x17);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x67);
    ad9364_spi_write(0x061, 0x47);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x68);
    ad9364_spi_write(0x061, 0xE2);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x69);
    ad9364_spi_write(0x061, 0x9A);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x6A);
    ad9364_spi_write(0x061, 0x22);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x6B);
    ad9364_spi_write(0x061, 0x80);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x6C);
    ad9364_spi_write(0x061, 0xDB);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x6D);
    ad9364_spi_write(0x061, 0x68);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x6E);
    ad9364_spi_write(0x061, 0x26);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x6F);
    ad9364_spi_write(0x061, 0xAC);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x70);
    ad9364_spi_write(0x061, 0xDB);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x71);
    ad9364_spi_write(0x061, 0x43);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x72);
    ad9364_spi_write(0x061, 0x24);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x73);
    ad9364_spi_write(0x061, 0xCC);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x74);
    ad9364_spi_write(0x061, 0xDF);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x75);
    ad9364_spi_write(0x061, 0x28);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x76);
    ad9364_spi_write(0x061, 0x1F);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x77);
    ad9364_spi_write(0x061, 0xE1);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x78);
    ad9364_spi_write(0x061, 0xE3);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x79);
    ad9364_spi_write(0x061, 0x1A);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x7A);
    ad9364_spi_write(0x061, 0x20);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x7B);
    ad9364_spi_write(0x061, 0xE8);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x7C);
    ad9364_spi_write(0x061, 0xC0);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x7D);
    ad9364_spi_write(0x061, 0xCF);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x7E);
    ad9364_spi_write(0x061, 0xF1);
    ad9364_spi_write(0x062, 0xFF);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x060, 0x7F);
    ad9364_spi_write(0x061, 0x01);
    ad9364_spi_write(0x062, 0x00);
    ad9364_spi_write(0x065, 0xFE);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x064, 0x00);
    ad9364_spi_write(0x065, 0xF8); // Disable clock to Tx Filter

    //************************************************************
    // Program Rx FIR: C:\Program Files (x86)\Analog Devices\AD9361R2
    // Evaluation Software 2.1.3\DigitalFilters\128tapFilter.ftr
    //************************************************************
    ad9364_spi_write(0x0F5, 0xFA); // Enable clock to Rx FIR Filter
    ad9364_spi_write(0x0F6, 0x02); // Write Filter Gain setting
    ad9364_spi_write(0x0F0, 0x00); // Write FIR coefficient address
    ad9364_spi_write(0x0F1, 0x01); // Write FIR coefficient data[7:0]
    ad9364_spi_write(0x0F2, 0x00); // Write FIR coefficient data[15:8]
    ad9364_spi_write(0x0F5, 0xFE); // Set Write EN to push data into FIR filter register map
    ad9364_spi_write(0x0F4, 0x00); // Dummy Write to Read only register to delay ~1us
    ad9364_spi_write(0x0F4, 0x00); // Dummy Write to Read only register to delay ~1us
    ad9364_spi_write(0x0F0, 0x01);
    ad9364_spi_write(0x0F1, 0xF1);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x02);
    ad9364_spi_write(0x0F1, 0xCF);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x03);
    ad9364_spi_write(0x0F1, 0xC0);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x04);
    ad9364_spi_write(0x0F1, 0xE8);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x05);
    ad9364_spi_write(0x0F1, 0x20);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x06);
    ad9364_spi_write(0x0F1, 0x1A);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x07);
    ad9364_spi_write(0x0F1, 0xE3);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x08);
    ad9364_spi_write(0x0F1, 0xE1);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x09);
    ad9364_spi_write(0x0F1, 0x1F);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x0A);
    ad9364_spi_write(0x0F1, 0x28);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x0B);
    ad9364_spi_write(0x0F1, 0xDF);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x0C);
    ad9364_spi_write(0x0F1, 0xCC);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x0D);
    ad9364_spi_write(0x0F1, 0x24);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x0E);
    ad9364_spi_write(0x0F1, 0x43);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x0F);
    ad9364_spi_write(0x0F1, 0xDB);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x10);
    ad9364_spi_write(0x0F1, 0xAC);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x11);
    ad9364_spi_write(0x0F1, 0x26);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x12);
    ad9364_spi_write(0x0F1, 0x68);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x13);
    ad9364_spi_write(0x0F1, 0xDB);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x14);
    ad9364_spi_write(0x0F1, 0x80);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x15);
    ad9364_spi_write(0x0F1, 0x22);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x16);
    ad9364_spi_write(0x0F1, 0x9A);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x17);
    ad9364_spi_write(0x0F1, 0xE2);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x18);
    ad9364_spi_write(0x0F1, 0x47);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x19);
    ad9364_spi_write(0x0F1, 0x17);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x1A);
    ad9364_spi_write(0x0F1, 0xDB);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x1B);
    ad9364_spi_write(0x0F1, 0xF3);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x1C);
    ad9364_spi_write(0x0F1, 0xFF);
    ad9364_spi_write(0x0F2, 0xFE);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x1D);
    ad9364_spi_write(0x0F1, 0xFF);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x1E);
    ad9364_spi_write(0x0F1, 0x2B);
    ad9364_spi_write(0x0F2, 0x01);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x1F);
    ad9364_spi_write(0x0F1, 0x13);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x20);
    ad9364_spi_write(0x0F1, 0xA5);
    ad9364_spi_write(0x0F2, 0xFE);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x21);
    ad9364_spi_write(0x0F1, 0xD7);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x22);
    ad9364_spi_write(0x0F1, 0x90);
    ad9364_spi_write(0x0F2, 0x01);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x23);
    ad9364_spi_write(0x0F1, 0x46);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x24);
    ad9364_spi_write(0x0F1, 0x35);
    ad9364_spi_write(0x0F2, 0xFE);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x25);
    ad9364_spi_write(0x0F1, 0x97);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x26);
    ad9364_spi_write(0x0F1, 0x0E);
    ad9364_spi_write(0x0F2, 0x02);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x27);
    ad9364_spi_write(0x0F1, 0x95);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x28);
    ad9364_spi_write(0x0F1, 0xA7);
    ad9364_spi_write(0x0F2, 0xFD);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x29);
    ad9364_spi_write(0x0F1, 0x36);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x2A);
    ad9364_spi_write(0x0F1, 0xAE);
    ad9364_spi_write(0x0F2, 0x02);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x2B);
    ad9364_spi_write(0x0F1, 0x0D);
    ad9364_spi_write(0x0F2, 0x01);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x2C);
    ad9364_spi_write(0x0F1, 0xF0);
    ad9364_spi_write(0x0F2, 0xFC);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x2D);
    ad9364_spi_write(0x0F1, 0xA1);
    ad9364_spi_write(0x0F2, 0xFE);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x2E);
    ad9364_spi_write(0x0F1, 0x83);
    ad9364_spi_write(0x0F2, 0x03);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x2F);
    ad9364_spi_write(0x0F1, 0xC6);
    ad9364_spi_write(0x0F2, 0x01);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x30);
    ad9364_spi_write(0x0F1, 0xF3);
    ad9364_spi_write(0x0F2, 0xFB);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x31);
    ad9364_spi_write(0x0F1, 0xB6);
    ad9364_spi_write(0x0F2, 0xFD);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x32);
    ad9364_spi_write(0x0F1, 0xB7);
    ad9364_spi_write(0x0F2, 0x04);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x33);
    ad9364_spi_write(0x0F1, 0xF8);
    ad9364_spi_write(0x0F2, 0x02);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x34);
    ad9364_spi_write(0x0F1, 0x6D);
    ad9364_spi_write(0x0F2, 0xFA);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x35);
    ad9364_spi_write(0x0F1, 0x1A);
    ad9364_spi_write(0x0F2, 0xFC);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x36);
    ad9364_spi_write(0x0F1, 0xBE);
    ad9364_spi_write(0x0F2, 0x06);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x37);
    ad9364_spi_write(0x0F1, 0x41);
    ad9364_spi_write(0x0F2, 0x05);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x38);
    ad9364_spi_write(0x0F1, 0x87);
    ad9364_spi_write(0x0F2, 0xF7);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x39);
    ad9364_spi_write(0x0F1, 0x98);
    ad9364_spi_write(0x0F2, 0xF8);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x3A);
    ad9364_spi_write(0x0F1, 0x60);
    ad9364_spi_write(0x0F2, 0x0B);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x3B);
    ad9364_spi_write(0x0F1, 0x6D);
    ad9364_spi_write(0x0F2, 0x0B);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x3C);
    ad9364_spi_write(0x0F1, 0x88);
    ad9364_spi_write(0x0F2, 0xEE);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x3D);
    ad9364_spi_write(0x0F1, 0x40);
    ad9364_spi_write(0x0F2, 0xEA);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x3E);
    ad9364_spi_write(0x0F1, 0x86);
    ad9364_spi_write(0x0F2, 0x27);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x3F);
    ad9364_spi_write(0x0F1, 0x09);
    ad9364_spi_write(0x0F2, 0x72);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x40);
    ad9364_spi_write(0x0F1, 0x09);
    ad9364_spi_write(0x0F2, 0x72);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x41);
    ad9364_spi_write(0x0F1, 0x86);
    ad9364_spi_write(0x0F2, 0x27);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x42);
    ad9364_spi_write(0x0F1, 0x40);
    ad9364_spi_write(0x0F2, 0xEA);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x43);
    ad9364_spi_write(0x0F1, 0x88);
    ad9364_spi_write(0x0F2, 0xEE);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x44);
    ad9364_spi_write(0x0F1, 0x6D);
    ad9364_spi_write(0x0F2, 0x0B);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x45);
    ad9364_spi_write(0x0F1, 0x60);
    ad9364_spi_write(0x0F2, 0x0B);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x46);
    ad9364_spi_write(0x0F1, 0x98);
    ad9364_spi_write(0x0F2, 0xF8);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x47);
    ad9364_spi_write(0x0F1, 0x87);
    ad9364_spi_write(0x0F2, 0xF7);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x48);
    ad9364_spi_write(0x0F1, 0x41);
    ad9364_spi_write(0x0F2, 0x05);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x49);
    ad9364_spi_write(0x0F1, 0xBE);
    ad9364_spi_write(0x0F2, 0x06);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x4A);
    ad9364_spi_write(0x0F1, 0x1A);
    ad9364_spi_write(0x0F2, 0xFC);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x4B);
    ad9364_spi_write(0x0F1, 0x6D);
    ad9364_spi_write(0x0F2, 0xFA);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x4C);
    ad9364_spi_write(0x0F1, 0xF8);
    ad9364_spi_write(0x0F2, 0x02);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x4D);
    ad9364_spi_write(0x0F1, 0xB7);
    ad9364_spi_write(0x0F2, 0x04);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x4E);
    ad9364_spi_write(0x0F1, 0xB6);
    ad9364_spi_write(0x0F2, 0xFD);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x4F);
    ad9364_spi_write(0x0F1, 0xF3);
    ad9364_spi_write(0x0F2, 0xFB);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x50);
    ad9364_spi_write(0x0F1, 0xC6);
    ad9364_spi_write(0x0F2, 0x01);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x51);
    ad9364_spi_write(0x0F1, 0x83);
    ad9364_spi_write(0x0F2, 0x03);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x52);
    ad9364_spi_write(0x0F1, 0xA1);
    ad9364_spi_write(0x0F2, 0xFE);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x53);
    ad9364_spi_write(0x0F1, 0xF0);
    ad9364_spi_write(0x0F2, 0xFC);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x54);
    ad9364_spi_write(0x0F1, 0x0D);
    ad9364_spi_write(0x0F2, 0x01);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x55);
    ad9364_spi_write(0x0F1, 0xAE);
    ad9364_spi_write(0x0F2, 0x02);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x56);
    ad9364_spi_write(0x0F1, 0x36);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x57);
    ad9364_spi_write(0x0F1, 0xA7);
    ad9364_spi_write(0x0F2, 0xFD);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x58);
    ad9364_spi_write(0x0F1, 0x95);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x59);
    ad9364_spi_write(0x0F1, 0x0E);
    ad9364_spi_write(0x0F2, 0x02);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x5A);
    ad9364_spi_write(0x0F1, 0x97);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x5B);
    ad9364_spi_write(0x0F1, 0x35);
    ad9364_spi_write(0x0F2, 0xFE);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x5C);
    ad9364_spi_write(0x0F1, 0x46);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x5D);
    ad9364_spi_write(0x0F1, 0x90);
    ad9364_spi_write(0x0F2, 0x01);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x5E);
    ad9364_spi_write(0x0F1, 0xD7);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x5F);
    ad9364_spi_write(0x0F1, 0xA5);
    ad9364_spi_write(0x0F2, 0xFE);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x60);
    ad9364_spi_write(0x0F1, 0x13);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x61);
    ad9364_spi_write(0x0F1, 0x2B);
    ad9364_spi_write(0x0F2, 0x01);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x62);
    ad9364_spi_write(0x0F1, 0xFF);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x63);
    ad9364_spi_write(0x0F1, 0xFF);
    ad9364_spi_write(0x0F2, 0xFE);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x64);
    ad9364_spi_write(0x0F1, 0xF3);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x65);
    ad9364_spi_write(0x0F1, 0xDB);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x66);
    ad9364_spi_write(0x0F1, 0x17);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x67);
    ad9364_spi_write(0x0F1, 0x47);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x68);
    ad9364_spi_write(0x0F1, 0xE2);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x69);
    ad9364_spi_write(0x0F1, 0x9A);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x6A);
    ad9364_spi_write(0x0F1, 0x22);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x6B);
    ad9364_spi_write(0x0F1, 0x80);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x6C);
    ad9364_spi_write(0x0F1, 0xDB);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x6D);
    ad9364_spi_write(0x0F1, 0x68);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x6E);
    ad9364_spi_write(0x0F1, 0x26);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x6F);
    ad9364_spi_write(0x0F1, 0xAC);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x70);
    ad9364_spi_write(0x0F1, 0xDB);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x71);
    ad9364_spi_write(0x0F1, 0x43);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x72);
    ad9364_spi_write(0x0F1, 0x24);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x73);
    ad9364_spi_write(0x0F1, 0xCC);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x74);
    ad9364_spi_write(0x0F1, 0xDF);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x75);
    ad9364_spi_write(0x0F1, 0x28);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x76);
    ad9364_spi_write(0x0F1, 0x1F);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x77);
    ad9364_spi_write(0x0F1, 0xE1);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x78);
    ad9364_spi_write(0x0F1, 0xE3);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x79);
    ad9364_spi_write(0x0F1, 0x1A);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x7A);
    ad9364_spi_write(0x0F1, 0x20);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x7B);
    ad9364_spi_write(0x0F1, 0xE8);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x7C);
    ad9364_spi_write(0x0F1, 0xC0);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x7D);
    ad9364_spi_write(0x0F1, 0xCF);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x7E);
    ad9364_spi_write(0x0F1, 0xF1);
    ad9364_spi_write(0x0F2, 0xFF);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F0, 0x7F);
    ad9364_spi_write(0x0F1, 0x01);
    ad9364_spi_write(0x0F2, 0x00);
    ad9364_spi_write(0x0F5, 0xFE);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F4, 0x00);
    ad9364_spi_write(0x0F5, 0xF8); // Disable clock to Rx Filter

    //************************************************************
    // Setup the Parallel Port (Digital Data Interface)
    //************************************************************
    ad9364_spi_write(0x010, 0xC0); // I/O Config.  Tx Swap IQ; Rx Swap IQ; Tx CH Swap, Rx CH Swap; Rx Frame Mode; 2R2T bit; Invert data bus; Invert DATA_CLK
    ad9364_spi_write(0x011, 0x00); // I/O Config.  Alt Word Order; -Rx1; -Rx2; -Tx1; -Tx2; Invert Rx Frame; Delay Rx Data
    ad9364_spi_write(0x012, 0x28); // I/O Config.  Rx=2*Tx; Swap Ports; SDR; LVDS; Half Duplex; Single Port; Full Port; Swap Bits
    ad9364_spi_write(0x006, 0x0F); // PPORT Rx Delay (adjusts Tco Dataclk->Data)
    ad9364_spi_write(0x007, 0x00); // PPORT TX Delay (adjusts setup/hold FBCLK->Data)
    //-------------------------------------------------------
    ad9364_spi_write(REG_RX_LO_GEN_POWER_MODE, 0x0);
    ad9364_spi_write(REG_TX_LO_GEN_POWER_MODE, 0x0);
    ad9364_spi_write(REG_RX_VCO_LDO, 0xB);
    ad9364_spi_write(REG_TX_VCO_LDO, 0xB);
    ad9364_spi_write(REG_RX_VCO_PD_OVERRIDES, 0x2);
    ad9364_spi_write(REG_TX_VCO_PD_OVERRIDES, 0x2);
    ad9364_spi_write(REG_RX_VCO_CAL, 0x82);
    ad9364_spi_write(REG_TX_VCO_CAL, 0x82);
    ad9364_spi_write(REG_RX_CP_CURRENT, 0x80);
    ad9364_spi_write(REG_TX_CP_CURRENT, 0x80);
    ad9364_spi_write(REG_RX_CP_CONFIG, 0x0);
    ad9364_spi_write(REG_TX_CP_CONFIG, 0x0);
    //ad9364_spi_write(0x243,0xd);
    //ad9364_spi_write(0x283,0xd);
    ad9364_spi_write(0x28B, 0x17);
    ad9364_spi_write(0x28D, 0x00);

    //ad9364_spi_write(REG_ENSM_CONFIG_2,0x10);//SET SYNTH DUAL MODE 0 AND TXNRX SPI CONTROL 1
    //ad9364_spi_write(REG_ENSM_CONFIG_1,0x5);
    //ad9364_spi_write(REG_ENSM_MODE,0x0);
    //ad9364_spi_write(0x012,0x20); // Enable FDD mode during calibrations
    //ad9364_spi_write(0x015,0x04);   // Set Dual Synth mode bit
    //ad9364_spi_write(0x014,0x0D);   // Set Force ALERT State bit
    //ad9364_spi_write(0x013,0x01);   // Set ENSM FDD mode*/

    ad9364_spi_write(REG_RX_CP_CONFIG, 0x4);
    wait(1000);
    while (!(ad9364_spi_read(REG_RX_CAL_STATUS) & 0x80));

    ad9364_spi_write(REG_TX_CP_CONFIG, 0x4);
    wait(1000);
    while (!(ad9364_spi_read(REG_TX_CAL_STATUS) & 0x80)) {};
    while (!(ad9364_spi_read(REG_TX_CAL_STATUS) & 0x20)) {};

    ad9364_spi_write(REG_RX_CP_CONFIG, 0x0);
    ad9364_spi_write(REG_TX_CP_CONFIG, 0x0);
    //-------------------------------------------------------
    //while(ad9364_spi_read(REG_TX_CP_OVERRANGE_VCO_LOCK) & 0x2);
    ad9364_spi_write(REG_RX_VCO_OUTPUT, 0x4A);
    ad9364_spi_write(REG_RX_ALC_VARACTOR, 0xC0);
    ad9364_spi_write(REG_RX_VCO_BIAS_1, 0xD);
    ad9364_spi_write(REG_RX_FORCE_VCO_TUNE_1, 0x0);
    ad9364_spi_write(REG_RX_VCO_CAL_REF, 0x0);
    ad9364_spi_write(REG_RX_VCO_VARACTOR_CTRL_1, 0x8);
    ad9364_spi_write(REG_RX_VCO_VARACTOR_CTRL_0, 0x70);
    ad9364_spi_write(REG_RX_CP_CURRENT, 0x94);
    ad9364_spi_write(REG_RX_LOOP_FILTER_1, 0xD4);
    ad9364_spi_write(REG_RX_LOOP_FILTER_2, 0xDF);
    ad9364_spi_write(REG_RX_LOOP_FILTER_3, 0x9);

    ad9364_spi_write(REG_TX_VCO_OUTPUT, 0x4A);
    ad9364_spi_write(REG_TX_ALCVARACT_OR, 0xC0);
    ad9364_spi_write(REG_TX_VCO_BIAS_1, 0xD);
    ad9364_spi_write(REG_TX_FORCE_VCO_TUNE_1, 0x0);
    ad9364_spi_write(REG_TX_VCO_CAL_REF, 0x0);
    ad9364_spi_write(REG_TX_VCO_VARACTOR_CTRL_1, 0x8);
    ad9364_spi_write(REG_TX_VCO_VARACTOR_CTRL_1, 0x70);
    ad9364_spi_write(REG_TX_CP_CURRENT, 0x88);
    ad9364_spi_write(REG_TX_LOOP_FILTER_1, 0xD4);
    ad9364_spi_write(REG_TX_LOOP_FILTER_2, 0xDF);
    ad9364_spi_write(REG_TX_LOOP_FILTER_3, 0x9);
    //--------------------------------------------------
    ad9364_spi_write(REG_RX_FRACT_BYTE_0, 0xd0);
    ad9364_spi_write(REG_RX_FRACT_BYTE_1, 0x6d);
    ad9364_spi_write(REG_RX_FRACT_BYTE_2, 0x4b);
    ad9364_spi_write(REG_RX_INTEGER_BYTE_0, 0x7b);
    ad9364_spi_write(REG_RX_INTEGER_BYTE_1, 0x0);
    ad9364_spi_write(REG_RFPLL_DIVIDERS, 0x11);

    ad9364_spi_write(REG_TX_FRACT_BYTE_0, 0xe0);
    ad9364_spi_write(REG_TX_FRACT_BYTE_1, 0x9f);
    ad9364_spi_write(REG_TX_FRACT_BYTE_2, 0x7e);
    ad9364_spi_write(REG_TX_INTEGER_BYTE_0, 0x7b);
    ad9364_spi_write(REG_TX_INTEGER_BYTE_1, 0x0);
    ad9364_spi_write(REG_RFPLL_DIVIDERS, 0x11);
    while (!((ad9364_spi_read(REG_RX_CP_OVERRANGE_VCO_LOCK) & 0x2) == 0x2));

    while (!((ad9364_spi_read(REG_TX_CP_OVERRANGE_VCO_LOCK) & 0x2) == 0x2));


    //************************************************************
    // Program Mixer GM Sub-table
    //************************************************************
    ad9364_spi_write(0x13F, 0x02); // Start Clock
    ad9364_spi_write(0x138, 0x0F); // Addr Table Index
    ad9364_spi_write(0x139, 0x78); // Gain
    ad9364_spi_write(0x13A, 0x00); // Bias
    ad9364_spi_write(0x13B, 0x00); // GM
    ad9364_spi_write(0x13F, 0x06); // Write Words
    ad9364_spi_write(0x13C, 0x00); // Delay for 3 ADCCLK/16 clock cycles (Dummy Write)
    ad9364_spi_write(0x13C, 0x00); // Delay ~1us (Dummy Write)
    ad9364_spi_write(0x138, 0x0E); // Addr Table Index
    ad9364_spi_write(0x139, 0x74); // Gain
    ad9364_spi_write(0x13A, 0x00); // Bias
    ad9364_spi_write(0x13B, 0x0D); // GM
    ad9364_spi_write(0x13F, 0x06); // Write Words
    ad9364_spi_write(0x13C, 0x00); // Delay for 3 ADCCLK/16 clock cycles (Dummy Write)
    ad9364_spi_write(0x13C, 0x00); // Delay ~1us (Dummy Write)
    ad9364_spi_write(0x138, 0x0D); // Addr Table Index
    ad9364_spi_write(0x139, 0x70); // Gain
    ad9364_spi_write(0x13A, 0x00); // Bias
    ad9364_spi_write(0x13B, 0x15); // GM
    ad9364_spi_write(0x13F, 0x06); // Write Words
    ad9364_spi_write(0x13C, 0x00); // Delay for 3 ADCCLK/16 clock cycles (Dummy Write)
    ad9364_spi_write(0x13C, 0x00); // Delay ~1us (Dummy Write)
    ad9364_spi_write(0x138, 0x0C); // Addr Table Index
    ad9364_spi_write(0x139, 0x6C); // Gain
    ad9364_spi_write(0x13A, 0x00); // Bias
    ad9364_spi_write(0x13B, 0x1B); // GM
    ad9364_spi_write(0x13F, 0x06); // Write Words
    ad9364_spi_write(0x13C, 0x00); // Delay for 3 ADCCLK/16 clock cycles (Dummy Write)
    ad9364_spi_write(0x13C, 0x00); // Delay ~1us (Dummy Write)
    ad9364_spi_write(0x138, 0x0B); // Addr Table Index
    ad9364_spi_write(0x139, 0x68); // Gain
    ad9364_spi_write(0x13A, 0x00); // Bias
    ad9364_spi_write(0x13B, 0x21); // GM
    ad9364_spi_write(0x13F, 0x06); // Write Words
    ad9364_spi_write(0x13C, 0x00); // Delay for 3 ADCCLK/16 clock cycles (Dummy Write)
    ad9364_spi_write(0x13C, 0x00); // Delay ~1us (Dummy Write)
    ad9364_spi_write(0x138, 0x0A); // Addr Table Index
    ad9364_spi_write(0x139, 0x64); // Gain
    ad9364_spi_write(0x13A, 0x00); // Bias
    ad9364_spi_write(0x13B, 0x25); // GM
    ad9364_spi_write(0x13F, 0x06); // Write Words
    ad9364_spi_write(0x13C, 0x00); // Delay for 3 ADCCLK/16 clock cycles (Dummy Write)
    ad9364_spi_write(0x13C, 0x00); // Delay ~1us (Dummy Write)
    ad9364_spi_write(0x138, 0x09); // Addr Table Index
    ad9364_spi_write(0x139, 0x60); // Gain
    ad9364_spi_write(0x13A, 0x00); // Bias
    ad9364_spi_write(0x13B, 0x29); // GM
    ad9364_spi_write(0x13F, 0x06); // Write Words
    ad9364_spi_write(0x13C, 0x00); // Delay for 3 ADCCLK/16 clock cycles (Dummy Write)
    ad9364_spi_write(0x13C, 0x00); // Delay ~1us (Dummy Write)
    ad9364_spi_write(0x138, 0x08); // Addr Table Index
    ad9364_spi_write(0x139, 0x5C); // Gain
    ad9364_spi_write(0x13A, 0x00); // Bias
    ad9364_spi_write(0x13B, 0x2C); // GM
    ad9364_spi_write(0x13F, 0x06); // Write Words
    ad9364_spi_write(0x13C, 0x00); // Delay for 3 ADCCLK/16 clock cycles (Dummy Write)
    ad9364_spi_write(0x13C, 0x00); // Delay ~1us (Dummy Write)
    ad9364_spi_write(0x138, 0x07); // Addr Table Index
    ad9364_spi_write(0x139, 0x58); // Gain
    ad9364_spi_write(0x13A, 0x00); // Bias
    ad9364_spi_write(0x13B, 0x2F); // GM
    ad9364_spi_write(0x13F, 0x06); // Write Words
    ad9364_spi_write(0x13C, 0x00); // Delay for 3 ADCCLK/16 clock cycles (Dummy Write)
    ad9364_spi_write(0x13C, 0x00); // Delay ~1us (Dummy Write)
    ad9364_spi_write(0x138, 0x06); // Addr Table Index
    ad9364_spi_write(0x139, 0x54); // Gain
    ad9364_spi_write(0x13A, 0x00); // Bias
    ad9364_spi_write(0x13B, 0x31); // GM
    ad9364_spi_write(0x13F, 0x06); // Write Words
    ad9364_spi_write(0x13C, 0x00); // Delay for 3 ADCCLK/16 clock cycles (Dummy Write)
    ad9364_spi_write(0x13C, 0x00); // Delay ~1us (Dummy Write)
    ad9364_spi_write(0x138, 0x05); // Addr Table Index
    ad9364_spi_write(0x139, 0x50); // Gain
    ad9364_spi_write(0x13A, 0x00); // Bias
    ad9364_spi_write(0x13B, 0x33); // GM
    ad9364_spi_write(0x13F, 0x06); // Write Words
    ad9364_spi_write(0x13C, 0x00); // Delay for 3 ADCCLK/16 clock cycles (Dummy Write)
    ad9364_spi_write(0x13C, 0x00); // Delay ~1us (Dummy Write)
    ad9364_spi_write(0x138, 0x04); // Addr Table Index
    ad9364_spi_write(0x139, 0x4C); // Gain
    ad9364_spi_write(0x13A, 0x00); // Bias
    ad9364_spi_write(0x13B, 0x34); // GM
    ad9364_spi_write(0x13F, 0x06); // Write Words
    ad9364_spi_write(0x13C, 0x00); // Delay for 3 ADCCLK/16 clock cycles (Dummy Write)
    ad9364_spi_write(0x13C, 0x00); // Delay ~1us (Dummy Write)
    ad9364_spi_write(0x138, 0x03); // Addr Table Index
    ad9364_spi_write(0x139, 0x48); // Gain
    ad9364_spi_write(0x13A, 0x00); // Bias
    ad9364_spi_write(0x13B, 0x35); // GM
    ad9364_spi_write(0x13F, 0x06); // Write Words
    ad9364_spi_write(0x13C, 0x00); // Delay for 3 ADCCLK/16 clock cycles (Dummy Write)
    ad9364_spi_write(0x13C, 0x00); // Delay ~1us (Dummy Write)
    ad9364_spi_write(0x138, 0x02); // Addr Table Index
    ad9364_spi_write(0x139, 0x30); // Gain
    ad9364_spi_write(0x13A, 0x00); // Bias
    ad9364_spi_write(0x13B, 0x3A); // GM
    ad9364_spi_write(0x13F, 0x06); // Write Words
    ad9364_spi_write(0x13C, 0x00); // Delay for 3 ADCCLK/16 clock cycles (Dummy Write)
    ad9364_spi_write(0x13C, 0x00); // Delay ~1us (Dummy Write)
    ad9364_spi_write(0x138, 0x01); // Addr Table Index
    ad9364_spi_write(0x139, 0x18); // Gain
    ad9364_spi_write(0x13A, 0x00); // Bias
    ad9364_spi_write(0x13B, 0x3D); // GM
    ad9364_spi_write(0x13F, 0x06); // Write Words
    ad9364_spi_write(0x13C, 0x00); // Delay for 3 ADCCLK/16 clock cycles (Dummy Write)
    ad9364_spi_write(0x13C, 0x00); // Delay ~1us (Dummy Write)
    ad9364_spi_write(0x138, 0x00); // Addr Table Index
    ad9364_spi_write(0x139, 0x00); // Gain
    ad9364_spi_write(0x13A, 0x00); // Bias
    ad9364_spi_write(0x13B, 0x3E); // GM
    ad9364_spi_write(0x13F, 0x06); // Write Words
    ad9364_spi_write(0x13C, 0x00); // Delay for 3 ADCCLK/16 clock cycles (Dummy Write)
    ad9364_spi_write(0x13C, 0x00); // Delay ~1us (Dummy Write)
    ad9364_spi_write(0x13F, 0x02); // Clear Write Bit
    ad9364_spi_write(0x13C, 0x00); // Delay for 3 ADCCLK/16 clock cycles (Dummy Write)
    ad9364_spi_write(0x13C, 0x00); // Delay ~1us (Dummy Write)
    ad9364_spi_write(0x13F, 0x00); // Stop Clock

    //************************************************************
    // Program Rx Gain Tables with GainTable2300MHz.csv
    //************************************************************
    ad9364_spi_write(0x137, 0x1A); // Start Gain Table Clock
    ad9364_spi_write(0x130, 0x00); // Gain Table Index
    ad9364_spi_write(0x131, 0x00); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x18); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x00); // Gain Table Index
    ad9364_spi_write(0x131, 0x00); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x18); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x00); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x02); // Gain Table Index
    ad9364_spi_write(0x131, 0x00); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x18); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x00); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x03); // Gain Table Index
    ad9364_spi_write(0x131, 0x00); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x18); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x00); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x04); // Gain Table Index
    ad9364_spi_write(0x131, 0x00); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x18); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x00); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x05); // Gain Table Index
    ad9364_spi_write(0x131, 0x00); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x18); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x00); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x06); // Gain Table Index
    ad9364_spi_write(0x131, 0x00); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x18); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x00); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x07); // Gain Table Index
    ad9364_spi_write(0x131, 0x00); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x18); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x08); // Gain Table Index
    ad9364_spi_write(0x131, 0x01); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x18); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x09); // Gain Table Index
    ad9364_spi_write(0x131, 0x02); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x18); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x0A); // Gain Table Index
    ad9364_spi_write(0x131, 0x04); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x18); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x0B); // Gain Table Index
    ad9364_spi_write(0x131, 0x04); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x0C); // Gain Table Index
    ad9364_spi_write(0x131, 0x05); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x0D); // Gain Table Index
    ad9364_spi_write(0x131, 0x06); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x0E); // Gain Table Index
    ad9364_spi_write(0x131, 0x07); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x0F); // Gain Table Index
    ad9364_spi_write(0x131, 0x08); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x10); // Gain Table Index
    ad9364_spi_write(0x131, 0x09); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x11); // Gain Table Index
    ad9364_spi_write(0x131, 0x0A); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x12); // Gain Table Index
    ad9364_spi_write(0x131, 0x0B); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x13); // Gain Table Index
    ad9364_spi_write(0x131, 0x0C); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x14); // Gain Table Index
    ad9364_spi_write(0x131, 0x0D); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x15); // Gain Table Index
    ad9364_spi_write(0x131, 0x0E); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x16); // Gain Table Index
    ad9364_spi_write(0x131, 0x0F); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x17); // Gain Table Index
    ad9364_spi_write(0x131, 0x25); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x18); // Gain Table Index
    ad9364_spi_write(0x131, 0x26); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x19); // Gain Table Index
    ad9364_spi_write(0x131, 0x44); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x1A); // Gain Table Index
    ad9364_spi_write(0x131, 0x45); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x1B); // Gain Table Index
    ad9364_spi_write(0x131, 0x46); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x1C); // Gain Table Index
    ad9364_spi_write(0x131, 0x47); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x1D); // Gain Table Index
    ad9364_spi_write(0x131, 0x47); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x1E); // Gain Table Index
    ad9364_spi_write(0x131, 0x65); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x1F); // Gain Table Index
    ad9364_spi_write(0x131, 0x47); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x20); // Gain Table Index
    ad9364_spi_write(0x131, 0x47); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x21); // Gain Table Index
    ad9364_spi_write(0x131, 0x48); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x22); // Gain Table Index
    ad9364_spi_write(0x131, 0x49); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x23); // Gain Table Index
    ad9364_spi_write(0x131, 0x4A); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x24); // Gain Table Index
    ad9364_spi_write(0x131, 0x4B); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x25); // Gain Table Index
    ad9364_spi_write(0x131, 0x4C); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x26); // Gain Table Index
    ad9364_spi_write(0x131, 0x4D); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x27); // Gain Table Index
    ad9364_spi_write(0x131, 0x4E); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x130, 0x28); // Gain Table Index
    ad9364_spi_write(0x131, 0x4F); // Ext LNA, Int LNA, & Mixer Gain Word
    ad9364_spi_write(0x132, 0x38); // TIA & LPF Word
    ad9364_spi_write(0x133, 0x20); // DC Cal bit & Dig Gain Word
    ad9364_spi_write(0x137, 0x1E); // Write Words
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay 3 ADCCLK/16 cycles
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x137, 0x1A); // Clear Write Bit
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x134, 0x00); // Dummy Write to delay ~1us
    ad9364_spi_write(0x137, 0x00); // Stop Gain Table Clock
    //************************************************************
    // Setup Rx AGC Fast AttackRegisters
    //************************************************************
    ad9364_spi_write(0x022, 0x3F); // AGC Fast Attack Gain Lock Delay
    ad9364_spi_write(0x0FA, 0xE5); // Gain Control Mode Select
    ad9364_spi_write(0x0FB, 0x00); // Gain Control Config
    ad9364_spi_write(0x0FC, 0x23); // ADC Overrange Sample Size
    ad9364_spi_write(0x0FD, 0x28); // Max Full/LMT Gain Table Index
    ad9364_spi_write(0x0FE, 0x4A); // Peak Overload Wait Time
    ad9364_spi_write(0x100, 0x6F); // Dig Gain: Step Size & Max
    ad9364_spi_write(0x101, 0x05); // AGC Lock Level
    ad9364_spi_write(0x103, 0x08); // Large LMT or Step 3 Size
    ad9364_spi_write(0x104, 0x65); // ADC Small Overload Threshold
    ad9364_spi_write(0x105, 0x70); // ADC Large Overload Threshold
    ad9364_spi_write(0x106, 0x22); // Overload Step Sizes
    ad9364_spi_write(0x107, 0x01); // Small LMT Overload Threshold
    ad9364_spi_write(0x108, 0x05); // Large LMT Overload Threshold
    ad9364_spi_write(0x109, 0x28); // State 5 Power Measurement MSB
    ad9364_spi_write(0x10A, 0x18); // State 5 Power Measurement LSBs
    ad9364_spi_write(0x10B, 0x00); // Rx1 Force Digital Gain
    ad9364_spi_write(0x10E, 0x00); // Rx2 Force Digital Gain
    ad9364_spi_write(0x110, 0x01); // AGC Fast Attack Config
    ad9364_spi_write(0x111, 0x0A); // Settling Delay & AGC Config
    ad9364_spi_write(0x112, 0x52); // Post Lock Step & Energy Lost Thresh
    ad9364_spi_write(0x113, 0x4C); // Post Lock Step & Strong Sig Thresh
    ad9364_spi_write(0x114, 0x30); // Low Power Threshold & ADC Ovr Ld
    ad9364_spi_write(0x115, 0x00); // Stronger Signal Unlock Control
    ad9364_spi_write(0x116, 0x65); // Final Overrange and Opt Gain Offset
    ad9364_spi_write(0x117, 0x08); // Gain Inc Step & Energy Detect Cnt
    ad9364_spi_write(0x118, 0x15); // Lock Level GAin Incr Upper Limit
    ad9364_spi_write(0x119, 0x08); // Gain Lock Exit Count
    ad9364_spi_write(0x11A, 0x07); // Initial LMT Gain Limit
    ad9364_spi_write(0x11B, 0x0A); // Increment Time

    //************************************************************
    // RX Baseband Filter Tuning (Real BW: 1.000000 MHz) 3dB Filter
    // Corner @ 1.400000 MHz)
    //************************************************************
    ad9364_spi_write(0x1FB, 0x01); // RX Freq Corner (MHz)
    ad9364_spi_write(0x1FC, 0x00); // RX Freq Corner (Khz)
    ad9364_spi_write(0x1F8, 0x3D); // Rx BBF Tune Divider[7:0]
    ad9364_spi_write(0x1F9, 0x1E); // RX BBF Tune Divider[8]

    ad9364_spi_write(0x1D5, 0x3F); // Set Rx Mix LO CM
    ad9364_spi_write(0x1C0, 0x03); // Set GM common mode
    ad9364_spi_write(0x1E2, 0x02); // Enable Rx1 Filter Tuner
    ad9364_spi_write(0x1E3, 0x02); // Enable Rx2 Filter Tuner
    ad9364_spi_write(0x016, 0x80); // Start RX Filter Tune
    while ((ad9364_spi_read(0x016) & 0x80));      // Wait for RX filter to tune, Max Cal Time: 48.451 us (Done when 0x016[7]==0)

    ad9364_spi_write(0x1E2, 0x03); // Disable Rx Filter Tuner (Rx1)
    ad9364_spi_write(0x1E3, 0x03); // Disable Rx Filter Tuner (Rx2)

    //************************************************************
    // TX Baseband Filter Tuning (Real BW: 1.000000 MHz) 3dB Filter
    // Corner @ 1.600000 MHz)
    //************************************************************
    ad9364_spi_write(0x0D6, 0x35); // TX BBF Tune Divider[7:0]
    ad9364_spi_write(0x0D7, 0x1E); // TX BBF Tune Divider[8]
    ad9364_spi_write(0x0CA, 0x22); // Enable Tx Filter Tuner
    ad9364_spi_write(0x016, 0x40); // Start Tx Filter Tune
    while ((ad9364_spi_read(0x016) & 0x40));  // Wait for TX filter to tune, Max Cal Time: 24.499 us (Done when 0x016[6]==0)

    ad9364_spi_write(0x0CA, 0x26); // Disable Tx Filter Tuner (Both Channels)

    //************************************************************
    // RX TIA Setup:  Setup values scale based on RxBBF calibration
    // results.  See information in Calibration Guide.
    //************************************************************
    //SPIRead 1EB // Read RXBBF C3(MSB)
    //SPIRead 1EC // Read RXBBF C3(LSB)
    //SPIRead 1E6 // Read RXBBF R2346
    ad9364_spi_write(0x1DB, 0xE0); // Set TIA selcc[2:0]
    ad9364_spi_write(0x1DD, 0x37); // Set RX TIA1 C MSB[6:0]
    ad9364_spi_write(0x1DF, 0x37); // Set RX TIA2 C MSB[6:0]
    ad9364_spi_write(0x1DC, 0x40); // Set RX TIA1 C LSB[5:0]
    ad9364_spi_write(0x1DE, 0x40); // Set RX TIA2 C LSB[5:0]

    //************************************************************
    // TX Secondary Filter Calibration Setup:  Real Bandwidth
    // 1.000000MHz, 3dB Corner @ 5.000000MHz
    //************************************************************
    ad9364_spi_write(0x0D2, 0x1C);  // TX Secondary Filter PDF Cap cal[5:0]
    ad9364_spi_write(0x0D1, 0x01); // TX Secondary Filter PDF Res cal[3:0]
    ad9364_spi_write(0x0D0, 0x59); // Pdampbias

    //************************************************************
    // ADC Setup:  Tune ADC Performance based on RX analog filter tune
    // corner.  Real Bandwidth: 0.992085 MHz, ADC Clock Frequency:
    // 192.000000 MHz.  The values in registers 0x200 - 0x227 need to be
    // calculated using the equations in the Calibration Guide.
    //************************************************************
    //SPIRead 1EB // Read RxBBF C3 MSB after calibration
    //SPIRead 1EC // Read RxBBF C3 LSB after calibration
    //SPIRead 1E6 // Read RxBBF R3 after calibration

    ad9364_spi_write(0x200, 0x00);
    ad9364_spi_write(0x201, 0x00);
    ad9364_spi_write(0x202, 0x00);
    ad9364_spi_write(0x203, 0x24);
    ad9364_spi_write(0x204, 0x24);
    ad9364_spi_write(0x205, 0x00);
    ad9364_spi_write(0x206, 0x00);
    ad9364_spi_write(0x207, 0x7C);
    ad9364_spi_write(0x208, 0x6A);
    ad9364_spi_write(0x209, 0x3C);
    ad9364_spi_write(0x20A, 0x4B);
    ad9364_spi_write(0x20B, 0x42);
    ad9364_spi_write(0x20C, 0x4E);
    ad9364_spi_write(0x20D, 0x41);
    ad9364_spi_write(0x20E, 0x00);
    ad9364_spi_write(0x20F, 0x7F);
    ad9364_spi_write(0x210, 0x7F);
    ad9364_spi_write(0x211, 0x7F);
    ad9364_spi_write(0x212, 0x49);
    ad9364_spi_write(0x213, 0x49);
    ad9364_spi_write(0x214, 0x49);
    ad9364_spi_write(0x215, 0x4C);
    ad9364_spi_write(0x216, 0x4C);
    ad9364_spi_write(0x217, 0x4C);
    ad9364_spi_write(0x218, 0x2E);
    ad9364_spi_write(0x219, 0x92);
    ad9364_spi_write(0x21A, 0x16);
    ad9364_spi_write(0x21B, 0x11);
    ad9364_spi_write(0x21C, 0x92);
    ad9364_spi_write(0x21D, 0x16);
    ad9364_spi_write(0x21E, 0x11);
    ad9364_spi_write(0x21F, 0x92);
    ad9364_spi_write(0x220, 0x16);
    ad9364_spi_write(0x221, 0x22);
    ad9364_spi_write(0x222, 0x23);
    ad9364_spi_write(0x223, 0x40);
    ad9364_spi_write(0x224, 0x40);
    ad9364_spi_write(0x225, 0x2C);
    ad9364_spi_write(0x226, 0x00);
    ad9364_spi_write(0x227, 0x00);
    //************************************************************
    // Setup and Run BB DC and RF DC Offset Calibrations
    //************************************************************
    ad9364_spi_write(0x193, 0x3F);
    ad9364_spi_write(0x190, 0x0F); // Set BBDC tracking shift M value, only applies when BB DC tracking enabled
    ad9364_spi_write(0x194, 0x01); // BBDC Cal setting
    ad9364_spi_write(0x016, 0x01); // Start BBDC offset cal
    while ((ad9364_spi_read(0x016) & 0x1));   // BBDC Max Cal Time: 16833.333 us. Cal done when 0x016[0]==0

    ad9364_spi_write(0x185, 0x20); // Set RF DC offset Wait Count
    ad9364_spi_write(0x186, 0x32); // Set RF DC Offset Count[7:0]
    ad9364_spi_write(0x187, 0x24); // Settings for RF DC cal
    ad9364_spi_write(0x18B, 0x83); // Settings for RF DC cal
    ad9364_spi_write(0x188, 0x05); // Settings for RF DC cal
    ad9364_spi_write(0x189, 0x30); // Settings for RF DC cal
    ad9364_spi_write(0x016, 0x02); // Start RFDC offset cal
    while ((ad9364_spi_read(0x016) & 0x1));   // RFDC Max Cal Time: 229022.500 us

    //************************************************************
    // Tx Quadrature Calibration Settings
    //************************************************************
    //SPIRead 0A3 // Masked Read:  Read lower 6 bits, overwrite [7:6] below
    ad9364_spi_write(0x0A0, 0x15); // Set TxQuadcal NCO frequency
    ad9364_spi_write(0x0A3, 0x00); // Set TxQuadcal NCO frequency (Only update bits [7:6])
    ad9364_spi_write(0x0A1, 0x7B); // Tx Quad Cal Configuration, Phase and Gain Cal Enable
    ad9364_spi_write(0x0A9, 0xFF); // Set Tx Quad Cal Count
    ad9364_spi_write(0x0A2, 0x7F); // Set Tx Quad Cal Kexp
    ad9364_spi_write(0x0A5, 0x01); // Set Tx Quad Cal Magnitude Threshhold
    ad9364_spi_write(0x0A6, 0x01); // Set Tx Quad Cal Magnitude Threshhold
    ad9364_spi_write(0x0AA, 0x25); // Set Tx Quad Cal Gain Table index
    ad9364_spi_write(0x0A4, 0xF0); // Set Tx Quad Cal Settle Count
    ad9364_spi_write(0x0AE, 0x00); // Set Tx Quad Cal LPF Gain index incase Split table mode used
    ad9364_spi_write(0x169, 0xC0); // Disable Rx Quadrature Calibration before Running Tx Quadrature Calibration
    ad9364_spi_write(0x016, 0x10); // Start Tx Quad cal
    while ((ad9364_spi_read(0x016) & 0x10)); //WAIT_CALDONE TXQUAD,2000 // Wait for cal to complete (Done when 0x016[4]==0)

    ad9364_spi_write(0x16A, 0x75); // Set Kexp Phase
    ad9364_spi_write(0x16B, 0x95); // Set Kexp Amplitude & Prevent Positive Gain Bit
    ad9364_spi_write(0x169, 0xCF); // Enable Rx Quadrature Calibration Tracking
    ad9364_spi_write(0x18B, 0xAD); // Enable BB and RF DC Tracking Calibrations
    //ad9364_spi_write(0x012,0x28);   // Cals done, Set PPORT Config
    //ad9364_spi_write(0x013,0x00);   // Set ENSM FDD/TDD bit
    //ad9364_spi_write(0x015,0x00);   // Set Dual Synth Mode, FDD External Control bits properly

    //************************************************************
    // Set Tx Attenuation: Tx1: 10.00 dB,  Tx2: 10.00 dB
    //************************************************************
    ad9364_spi_write(0x073, 0x00);
    ad9364_spi_write(0x074, 0x00);
    ad9364_spi_write(0x075, 0x00);
    ad9364_spi_write(0x076, 0x00);

    //************************************************************
    // Setup RSSI and Power Measurement Duration Registers
    //************************************************************
    ad9364_spi_write(0x150, 0x0E); // RSSI Measurement Duration 0, 1
    ad9364_spi_write(0x151, 0x00); // RSSI Measurement Duration 2, 3
    ad9364_spi_write(0x152, 0xFF); // RSSI Weighted Multiplier 0
    ad9364_spi_write(0x153, 0x00); // RSSI Weighted Multiplier 1
    ad9364_spi_write(0x154, 0x00); // RSSI Weighted Multiplier 2
    ad9364_spi_write(0x155, 0x00); // RSSI Weighted Multiplier 3
    ad9364_spi_write(0x156, 0x00); // RSSI Delay
    ad9364_spi_write(0x157, 0x00); // RSSI Wait
    ad9364_spi_write(0x158, 0x09); // RSSI Mode Select
    ad9364_spi_write(0x15C, 0x62); // Power Measurement Duration

    //ad9364_spi_write(REG_ENSM_CONFIG_2,0x10);//SET SYNTH DUAL MODE 0 AND TXNRX SPI CONTROL 1
    //ad9364_spi_write(REG_ENSM_CONFIG_1,0x5);
    //ad9364_spi_write(REG_ENSM_MODE,0x0);

    //hwp_bt_phy->TX_HFP_CFG = 0x2FD00100;
    //hwp_bt_phy->TX_LFP_CFG = 0x1BFE;

    //ad9364_spi_write(REG_ENSM_CONFIG_1,0x2B);

    ad9364_spi_write(REG_CTRL_OUTPUT_POINTER, 0x16);
    ad9364_spi_write(0x08e, 00);
    ad9364_spi_write(0x08f, 00);
    ad9364_spi_write(0x092, 00);
    ad9364_spi_write(0x093, 00);
    ad9364_spi_write(0x09f, 05);


    if (test_result != TEST_FAIL)
    {
        test_result = TEST_FAIL;
    }

    return test_result;
}

void bt_rf_cal(void)
{
    ad9364_calibration();
    ad9364_bt_cfg();

}

