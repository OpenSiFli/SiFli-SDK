/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "register.h"
#include "ad9361.h"
#include "spi_tst_drv.h"
#include "ble_rf_cal.h"


#define RPL_FREQTAB_OFFSET 0
#define TEST_FAIL       0x0
#define TEST_PASS       0x1
#define TEST_UNFINISHED 0x2

#define read_byte(addr)          (*(volatile unsigned char *)((addr)))
#define write_byte(addr,value)   (*(volatile unsigned char *)((addr))) = (value)
#define read_hword(addr)         (*(volatile unsigned short *)((addr)))
#define write_hword(addr,value)  (*(volatile unsigned short *)((addr))) = (value)

#ifndef SOC_SF32LB56X
    #define SPI_TEMP_LCPU_ADDRESS 0x204FA000
#else
    #define SPI_TEMP_LCPU_ADDRESS 0x2041F300
#endif

typedef struct
{
    uint32_t ctrl;
    uint32_t src_addr;
    uint32_t dst_addr;
    uint32_t ndt_sel;
} gpdma_list;

#if 0
uint32_t spi_9364_table[80 * 4 * 2] =
{
    0x82750B, 0x827477, 0x827330, 0x827178,
    0x827511, 0x8274de, 0x827308, 0x827178,
    0x827518, 0x827444, 0x827300, 0x827178,
    0x82751e, 0x8274aa, 0x827388, 0x827178,
    0x827525, 0x827410, 0x827330, 0x827178,
    0x82752b, 0x827477, 0x827318, 0x827178,
    0x827531, 0x8274DD, 0x827350, 0x827178,
    0x827538, 0x827443, 0x827390, 0x827178,
    0x82753E, 0x8274A9, 0x8273D0, 0x827178,
    0x827545, 0x827410, 0x827318, 0x827178,
    0x82754B, 0x827476, 0x827350, 0x827178,
    0x827551, 0x8274dc, 0x827398, 0x827178,
    0x827558, 0x827442, 0x8273D8, 0x827178,
    0x82755e, 0x8274a9, 0x827340, 0x827178,
    0x827565, 0x82740F, 0x827358, 0x827178,
    0x82756b, 0x827475, 0x8273a0, 0x827178,
    0x827571, 0x8274DB, 0x8273D8, 0x827178,
    0x827578, 0x827442, 0x827328, 0x827178,
    0x82757E, 0x8274A8, 0x827368, 0x827178,
    0x827505, 0x82740e, 0x8273a8, 0x827179,
    0x82750B, 0x827474, 0x8273E8, 0x827179,
    0x827511, 0x8274db, 0x827330, 0x827179,
    0x827518, 0x827441, 0x827368, 0x827179,
    0x82751e, 0x8274a7, 0x8273b0, 0x827179,
    0x827525, 0x82740D, 0x8273F0, 0x827179,
    0x82752b, 0x827474, 0x827338, 0x827179,
    0x827531, 0x8274DA, 0x827370, 0x827179,
    0x827538, 0x827440, 0x8273b8, 0x827179,
    0x82753E, 0x8274A6, 0x8273F0, 0x827179,
    0x827545, 0x82740b, 0x827340, 0x827179,
    0x82754B, 0x827473, 0x827380, 0x827179,
    0x827551, 0x8274d9, 0x8273c0, 0x827179,
    0x827558, 0x827440, 0x827300, 0x827179,
    0x82755e, 0x8274a6, 0x827348, 0x827179,
    0x827565, 0x82740C, 0x827380, 0x827179,
    0x82756b, 0x827472, 0x8273c8, 0x827179,
    0x827571, 0x8274D9, 0x827308, 0x827179,
    0x827578, 0x82743F, 0x827350, 0x827179,
    0x82757E, 0x8274A5, 0x827388, 0x827179,
    0x827505, 0x82740B, 0x8273B0, 0x82717A,
    0x82750B, 0x827472, 0x827310, 0x82717A,
    0x827511, 0x8274D8, 0x827358, 0x82717A,
    0x827518, 0x82743E, 0x827398, 0x82717A,
    0x82751E, 0x8274A4, 0x8273D8, 0x82717A,
    0x827525, 0x82740B, 0x827318, 0x82717A,
    0x82752B, 0x827471, 0x827360, 0x82717A,
    0x827531, 0x8274D7, 0x827398, 0x82717A,
    0x827538, 0x82743D, 0x8273E0, 0x82717A,
    0x82753E, 0x8274A4, 0x827320, 0x82717A,
    0x827545, 0x82740A, 0x827368, 0x82717A,
    0x82754B, 0x827470, 0x8273A0, 0x82717A,
    0x827551, 0x8274D6, 0x8273E8, 0x82717A,
    0x827558, 0x82743D, 0x827328, 0x82717A,
    0x82755E, 0x8274A3, 0x827370, 0x82717A,
    0x827565, 0x827409, 0x8273B0, 0x82717A,
    0x82756B, 0x82746F, 0x8273F0, 0x82717A,
    0x827571, 0x8274D6, 0x827330, 0x82717A,
    0x827578, 0x82743C, 0x827378, 0x82717A,
    0x82757E, 0x8274A2, 0x8273B8, 0x82717A,
    0x827505, 0x827408, 0x8273F8, 0x82717B,
    0x82750B, 0x82746F, 0x827340, 0x82717B,
    0x827511, 0x8274D5, 0x827380, 0x82717B,
    0x827518, 0x82743B, 0x8273C0, 0x82717B,
    0x82751E, 0x8274A2, 0x827300, 0x82717B,
    0x827525, 0x827408, 0x827348, 0x82717B,
    0x82752B, 0x82746E, 0x827388, 0x82717B,
    0x827531, 0x8274D4, 0x8273C8, 0x82717B,
    0x827538, 0x82743B, 0x827308, 0x82717B,
    0x82753E, 0x8274A1, 0x827350, 0x82717B,
    0x827545, 0x827407, 0x827390, 0x82717B,
    0x82754B, 0x82746D, 0x8273D0, 0x82717B,
    0x827551, 0x8274D4, 0x827310, 0x82717B,
    0x827558, 0x82743A, 0x827358, 0x82717B,
    0x82755E, 0x8274A0, 0x827398, 0x82717B,
    0x827565, 0x827406, 0x8273D8, 0x82717B,
    0x82756B, 0x82746D, 0x827320, 0x82717B,
    0x827571, 0x8274D3, 0x827360, 0x82717B,
    0x827578, 0x827439, 0x8273A0, 0x82717B,
    0x82757E, 0x82749F, 0x8273E0, 0x82717B,
    0x000000, 0x000000, 0x000000, 0x000000,
//rxpll table
    0x82350B, 0x823477, 0x823330, 0x823178,
    0x823511, 0x8234de, 0x823308, 0x823178,
    0x823518, 0x823444, 0x823300, 0x823178,
    0x82351e, 0x8234aa, 0x823388, 0x823178,
    0x823525, 0x823410, 0x823330, 0x823178,
    0x82352b, 0x823477, 0x823318, 0x823178,
    0x823531, 0x8234DD, 0x823350, 0x823178,
    0x823538, 0x823443, 0x823390, 0x823178,
    0x82353E, 0x8234A9, 0x8233D0, 0x823178,
    0x823545, 0x823410, 0x823318, 0x823178,
    0x82354B, 0x823476, 0x823350, 0x823178,
    0x823551, 0x8234dc, 0x823398, 0x823178,
    0x823558, 0x823442, 0x8233D8, 0x823178,
    0x82355e, 0x8234a9, 0x823340, 0x823178,
    0x823565, 0x82340F, 0x823358, 0x823178,
    0x82356b, 0x823475, 0x8233a0, 0x823178,
    0x823571, 0x8234DB, 0x8233D8, 0x823178,
    0x823578, 0x823442, 0x823328, 0x823178,
    0x82357E, 0x8234A8, 0x823368, 0x823178,
    0x823505, 0x82340e, 0x8233a8, 0x823179,
    0x82350B, 0x823474, 0x8233E8, 0x823179,
    0x823511, 0x8234db, 0x823330, 0x823179,
    0x823518, 0x823441, 0x823368, 0x823179,
    0x82351e, 0x8234a7, 0x8233b0, 0x823179,
    0x823525, 0x82340D, 0x8233F0, 0x823179,
    0x82352b, 0x823474, 0x823338, 0x823179,
    0x823531, 0x8234DA, 0x823370, 0x823179,
    0x823538, 0x823440, 0x8233b8, 0x823179,
    0x82353E, 0x8234A6, 0x8233F0, 0x823179,
    0x823545, 0x82340b, 0x823340, 0x823179,
    0x82354B, 0x823473, 0x823380, 0x823179,
    0x823551, 0x8234d9, 0x8233c0, 0x823179,
    0x823558, 0x823440, 0x823300, 0x823179,
    0x82355e, 0x8234a6, 0x823348, 0x823179,
    0x823565, 0x82340C, 0x823380, 0x823179,
    0x82356b, 0x823472, 0x8233c8, 0x823179,
    0x823571, 0x8234D9, 0x823308, 0x823179,
    0x823578, 0x82343F, 0x823350, 0x823179,
    0x82357E, 0x8234A5, 0x823388, 0x823179,
    0x823505, 0x82340B, 0x8233B0, 0x82317A,
    0x82350B, 0x823472, 0x823310, 0x82317A,
    0x823511, 0x8234D8, 0x823358, 0x82317A,
    0x823518, 0x82343E, 0x823398, 0x82317A,
    0x82351E, 0x8234A4, 0x8233D8, 0x82317A,
    0x823525, 0x82340B, 0x823318, 0x82317A,
    0x82352B, 0x823471, 0x823360, 0x82317A,
    0x823531, 0x8234D7, 0x823398, 0x82317A,
    0x823538, 0x82343D, 0x8233E0, 0x82317A,
    0x82353E, 0x8234A4, 0x823320, 0x82317A,
    0x823545, 0x82340A, 0x823368, 0x82317A,
    0x82354B, 0x823470, 0x8233A0, 0x82317A,
    0x823551, 0x8234D6, 0x8233E8, 0x82317A,
    0x823558, 0x82343D, 0x823328, 0x82317A,
    0x82355E, 0x8234A3, 0x823370, 0x82317A,
    0x823565, 0x823409, 0x8233B0, 0x82317A,
    0x82356B, 0x82346F, 0x8233F0, 0x82317A,
    0x823571, 0x8234D6, 0x823330, 0x82317A,
    0x823578, 0x82343C, 0x823378, 0x82317A,
    0x82357E, 0x8234A2, 0x8233B8, 0x82317A,
    0x823505, 0x823408, 0x8233F8, 0x82317B,
    0x82350B, 0x82346F, 0x823340, 0x82317B,
    0x823511, 0x8234D5, 0x823380, 0x82317B,
    0x823518, 0x82343B, 0x8233C0, 0x82317B,
    0x82351E, 0x8234A2, 0x823300, 0x82317B,
    0x823525, 0x823408, 0x823348, 0x82317B,
    0x82352B, 0x82346E, 0x823388, 0x82317B,
    0x823531, 0x8234D4, 0x8233C8, 0x82317B,
    0x823538, 0x82343B, 0x823308, 0x82317B,
    0x82353E, 0x8234A1, 0x823350, 0x82317B,
    0x823545, 0x823407, 0x823390, 0x82317B,
    0x82354B, 0x82346D, 0x8233D0, 0x82317B,
    0x823551, 0x8234D4, 0x823310, 0x82317B,
    0x823558, 0x82343A, 0x823358, 0x82317B,
    0x82355E, 0x8234A0, 0x823398, 0x82317B,
    0x823565, 0x823406, 0x8233D8, 0x82317B,
    0x82356B, 0x82346D, 0x823320, 0x82317B,
    0x823571, 0x8234D3, 0x823360, 0x82317B,
    0x823578, 0x823439, 0x8233A0, 0x82317B,
    0x82357E, 0x82349F, 0x8233E0, 0x82317B,
    0x000000, 0x000000, 0x000000, 0x000000
};
#endif

uint32_t spi_9364_table[88 * 4 * 2] =
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
    0x827506, 0x827466, 0x827366, 0x82717C,
    0x82750C, 0x8274CC, 0x8273CD, 0x82717C,
    0x827513, 0x827433, 0x827333, 0x82717C,
    0x827519, 0x827499, 0x82739A, 0x82717C,
    0x827520, 0x827400, 0x827300, 0x82717C,
    0x827526, 0x827466, 0x827366, 0x82717C,
    0x82752C, 0x8274CC, 0x8273CD, 0x82717C,
    0x827533, 0x827433, 0x827333, 0x82717C,
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
    0x823506, 0x823466, 0x823366, 0x82317C,
    0x82350C, 0x8234CC, 0x8233CD, 0x82317C,
    0x823513, 0x823433, 0x823333, 0x82317C,
    0x823519, 0x823499, 0x82339A, 0x82317C,
    0x823520, 0x823400, 0x823300, 0x82317C,
    0x823526, 0x823466, 0x823366, 0x82317C,
    0x82352C, 0x8234CC, 0x8233CD, 0x82317C,
    0x823533, 0x823433, 0x823333, 0x82317C,
    0x000000, 0x000000, 0x000000, 0x000000
};



static __attribute__((aligned(4))) gpdma_list g_list1;
static __attribute__((aligned(4))) gpdma_list g_list2;


void ad9364_bt_cfg()
{

    //&list1[0] = 0x20040000;
    //&list1[1] = 0x20040010;
    //initialization
    hwp_spi1->FIFO_CTRL |= SPI_FIFO_CTRL_TSRE;
    hwp_spi1->INTE |= SPI_INTE_TIE;

    hwp_bt_mac->DMRADIOCNTL4 = (uint32_t) & (spi_9364_table);
    //hwp_gpdma1->CSELR1 |= (28 << DMAC_CSELR1_C4S_Pos);

    g_list1.ctrl = GPDMA_CCR4_DIR | GPDMA_CCR4_MEM2MEM | GPDMA_CCR4_MINC | (0x2 << GPDMA_CCR4_MSIZE_Pos) | (0x2 << GPDMA_CCR4_PSIZE_Pos); //SRC
    g_list1.ctrl |= 0x80000000; //link to next
    g_list1.src_addr = (uint32_t) & (hwp_bt_mac->DMRADIOCNTL4);
    g_list1.dst_addr = (uint32_t) & (g_list2.src_addr);
    g_list1.ndt_sel  = 1;

    g_list2.ctrl = GPDMA_CCR4_DIR | GPDMA_CCR4_MINC | (0x2 << GPDMA_CCR4_MSIZE_Pos) | (0x2 << GPDMA_CCR4_PSIZE_Pos); //SRC
    g_list2.src_addr = 0;
    g_list2.dst_addr = SPI1_BASE + 0x14;
    g_list2.ndt_sel  = 28 << 16 | 4; //28 << 16 : hwp_gpdma1->CSELR1 |= (28 << DMAC_CSELR1_C4S_Pos);

    //ptc controls gpio toggle
    ((GPIO1_TypeDef *)hwp_gpio1)->DOER0 = 0x3;
    hwp_ptc2->TCR1 = (PTC_OP_OR << PTC_TCR1_OP_Pos) | 99; //trigger ble_phytxstart
    hwp_ptc2->TAR1 = (uint32_t) & (((GPIO1_TypeDef *)hwp_gpio1)->DOR0);
    hwp_ptc2->TDR1 = 0x3; //set PA00,PA01
    hwp_ptc2->TCR2 = (PTC_OP_AND << PTC_TCR2_OP_Pos) | 100; //trigger ble_txdone
    hwp_ptc2->TAR2 = (uint32_t) & (((GPIO1_TypeDef *)hwp_gpio1)->DOR0);
    hwp_ptc2->TDR2 = 0x0; //clear PA00,PA01
    hwp_ptc2->TCR3 = (PTC_OP_OR << PTC_TCR3_OP_Pos) | 102; //trigger ble_phyrxstart
    hwp_ptc2->TAR3 = (uint32_t) & (((GPIO1_TypeDef *)hwp_gpio1)->DOR0);
    hwp_ptc2->TDR3 = 0x1; //set PA00
    hwp_ptc2->TCR4 = (PTC_OP_AND << PTC_TCR4_OP_Pos) | 103; //trigger ble_rxdone
    hwp_ptc2->TAR4 = (uint32_t) & (((GPIO1_TypeDef *)hwp_gpio1)->DOR0);
    hwp_ptc2->TDR4 = 0x0; //clear PB00,PB01,PB06

    //ptc controls channel config
    hwp_ptc2->TCR5 = (PTC_OP_WRITE << PTC_TCR5_OP_Pos) | 98; //trigger ble_rftxstart
    hwp_ptc2->TAR5 = (uint32_t) & (hwp_gpdma1->LCR4);
    hwp_ptc2->TDR5 = (uint32_t)&g_list1 | 0x1; //start dmac1 channel5

    hwp_ptc2->TCR7 = (PTC_OP_WRITE << PTC_TCR7_OP_Pos) | 101; //trigger ble_rfrxstart
    hwp_ptc2->TAR7 = (uint32_t) & (hwp_gpdma1->LCR4);
    hwp_ptc2->TDR7 = (uint32_t)&g_list1 | 0x1; //start dmac1 channel5

}

void pinmux_spi()
{

    hwp_pinmux1->PAD_PA29 = 0x4C19; //clk
    hwp_pinmux1->PAD_PA24 = 0x4C1B; //cs
    hwp_pinmux1->PAD_PA25 = 0x4C1A; //di
    hwp_pinmux1->PAD_PA28 = 0x4C18; //do/dio
}



static void wait(uint32_t cycle)
{
    cycle *= 10;
    for (uint32_t i = 0; i < cycle; i++)
    {
        __NOP();
    }
}

void ad9364_spi_init()
{

    pinmux_spi();

    //24bit data width
    set_spi1_frm_width(24);

    //clk cfg
    //div 2 by default

    //sclk phase to 1, polarity stay 0
    //sclk is 0 when idle, and lanch data at posedge
    set_spi1_sph();

    //enable enable spi1
    enable_spi1_trx();

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
    uint32_t spi1_rne;

    tx_data = 0x800000 | ((uint32_t)addr << 8) | data;
    set_spi1_tdata(tx_data);
    //clear rx fifo
    spi1_rne = hwp_spi1->STATUS & SPI_STATUS_RNE;
    while (!spi1_rne)
    {
        spi1_rne = hwp_spi1->STATUS & SPI_STATUS_RNE;
    }
    rx_data = get_spi1_rdata();
    wait(3);
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
    uint32_t spi1_rne;

    tx_data = ((uint32_t)addr << 8);
    set_spi1_tdata(tx_data);
    //read rx fifo
    spi1_rne = hwp_spi1->STATUS & SPI_STATUS_RNE;
    while (!spi1_rne)
    {
        spi1_rne = hwp_spi1->STATUS & SPI_STATUS_RNE;
    }
    rx_data = get_spi1_rdata();
    return (uint8_t)rx_data;

#endif
}





// FPGA rf board init, should put to rf calibration

uint8_t ad9364_calibration()
{
    uint8_t test_result = TEST_UNFINISHED;
    //uint16_t i;
    //uint32_t rdata;

    hwp_hpsys_rcc->ENR1 |= HPSYS_RCC_ENR1_SPI1;
    ad9364_spi_init();
    ad9364_spi_write(REG_CTRL_OUTPUT_ENABLE, 0x0);
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
    while (!(ad9364_spi_read(REG_RX_CAL_STATUS) & 0x80)) {};

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
    ad9364_spi_write(0x073, 0x08);
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

    //ad9364_spi_write(0x0FA, 0xE4);  // Gain Control Mode Select

    ad9364_spi_write(0x08E, 0x00);
    ad9364_spi_write(0x08f, 0x00);
    ad9364_spi_write(0x092, 0x00);
    ad9364_spi_write(0x093, 0x00);
    ad9364_spi_write(0x09f, 0x05);


    if (test_result != TEST_FAIL)
    {
        test_result = TEST_FAIL;
    }

    return test_result;
}

void bt_rfc_init()
{
    uint8_t EDR_LO3G = 0 ;
    uint8_t EDR_LO5G = 1 ;

    uint32_t i;

    uint32_t rxon_addr;
    uint32_t rxoff_addr;
    uint32_t txon_addr;
    uint32_t txoff_addr;
    uint32_t bt_txon_addr;
    uint32_t bt_txoff_addr;

    uint32_t rxon_cmd_num;
    uint32_t rxoff_cmd_num;
    uint32_t txon_cmd_num;
    uint32_t txoff_cmd_num;
    uint32_t bt_txon_cmd_num;
    uint32_t bt_txoff_cmd_num;

    uint32_t rxon_cmd[90];
    uint32_t rxoff_cmd[70];
    uint32_t txon_cmd[90];
    uint32_t txoff_cmd[70];
    uint32_t bt_txon_cmd[90];
    uint32_t bt_txoff_cmd[70];

    uint32_t tmxbuf_cp_factor[8] = {0x466, 0x600, 0x79A, 0x933, 0xACD, 0xC66, 0xE00, 0xF9A};
    uint32_t cmd;
    uint32_t reg_data;

    if (EDR_LO3G)
        hwp_bt_phy->TX_CTRL |=  BT_PHY_TX_CTRL_MMDIV_SEL;
    else
        hwp_bt_phy->TX_CTRL &= ~BT_PHY_TX_CTRL_MMDIV_SEL;

    //enable q path
    hwp_bt_phy->RX_CTRL1 |= BT_PHY_RX_CTRL1_ADC_Q_EN_1;
    hwp_bt_phy->RX_CTRL2 |= BT_PHY_RX_CTRL2_ADC_Q_EN_2;
    hwp_bt_phy->RX_CTRL2 |= BT_PHY_RX_CTRL2_ADC_Q_EN_C;
    hwp_bt_phy->RX_CTRL2 |= BT_PHY_RX_CTRL2_ADC_Q_EN_BR;
    //zero if
    hwp_bt_phy->TX_IF_MOD_CFG  &= ~BT_PHY_TX_IF_MOD_CFG_TX_IF_PHASE_BLE_Msk ;
    hwp_bt_phy->MIXER_CFG1 = 0;

    //reset rccal
    hwp_bt_rfc->RBB_REG5 &= ~BT_RFC_RBB_REG5_BRF_RSTB_RCCAL_LV ;
    //release adc reset
    hwp_bt_rfc->ADC_REG  |= BT_RFC_ADC_REG_BRF_RSTB_ADC_LV ;
    //enable rf ref clk and adc clk
    hwp_bt_rfc->MISC_CTRL_REG |= BT_RFC_MISC_CTRL_REG_PKDET_EN_EARLY_OFF_EN;
    hwp_bt_rfc->MISC_CTRL_REG |= BT_RFC_MISC_CTRL_REG_XTAL_REF_EN;
    hwp_bt_rfc->MISC_CTRL_REG |= BT_RFC_MISC_CTRL_REG_ADC_CLK_EN;

    //set edr gfsk gain mul factor
    hwp_bt_phy->TX_IF_MOD_CFG2 = 0x180 << BT_PHY_TX_IF_MOD_CFG2_EDR_GGAIN_SAT_Pos |
                                 0x160 << BT_PHY_TX_IF_MOD_CFG2_GGAIN_SAT_Pos |
                                 0x600 << BT_PHY_TX_IF_MOD_CFG2_EDR_GGAIN_MUL_Pos;

    hwp_bt_phy->TX_IF_MOD_CFG2  |= 0x140 << BT_PHY_TX_IF_MOD_CFG_DGAIN_SAT_Pos;
    //change adc fifo wr clk phase
    //hwp_bt_rfc->MISC_CTRL_REG |= BT_RFC_MISC_CTRL_REG_ADC_FIFO_CLK_PHASE_SEL;
    //{{{---------fulcal and dccal data for debug----------------------------
    uint32_t reg_addr = 0x0 ;
    reg_data = 0;
    //store ble rx cal result
    hwp_bt_rfc->CAL_ADDR_REG1 = 0;
    hwp_bt_rfc->CAL_ADDR_REG1 = reg_addr;
    for (i = 0; i < 40; i++)
    {
        reg_data = ((((79 - i) << BT_RFC_VCO_REG3_BRF_VCO_IDAC_LV_Pos) + ((79 - i) << BT_RFC_VCO_REG3_BRF_VCO_PDX_LV_Pos)) << 16) + \
                   (i << BT_RFC_VCO_REG3_BRF_VCO_IDAC_LV_Pos) + (i << BT_RFC_VCO_REG3_BRF_VCO_PDX_LV_Pos);
        //printf("reg_data is %x i=%d\n",reg_data, i);
        write_memory(BT_RFC_MEM_BASE + reg_addr, reg_data);
        reg_addr += 4;
    }
    //store bt rx cal result
    hwp_bt_rfc->CAL_ADDR_REG1 += reg_addr << 16;
    for (i = 0; i < 40; i++)
    {
        reg_data = ((((i * 2) << BT_RFC_VCO_REG3_BRF_VCO_IDAC_LV_Pos)   + ((i * 2) << BT_RFC_VCO_REG3_BRF_VCO_PDX_LV_Pos)) << 16) + \
                   ((i * 2 + 1) << BT_RFC_VCO_REG3_BRF_VCO_IDAC_LV_Pos)  + ((i * 2 + 1) << BT_RFC_VCO_REG3_BRF_VCO_PDX_LV_Pos);
        //printf("reg_data is %x i=%d\n",reg_data, i);
        write_memory(BT_RFC_MEM_BASE + reg_addr, reg_data);
        reg_addr += 4;
    }
    //store ble tx cal result
    hwp_bt_rfc->CAL_ADDR_REG2 = reg_addr;
    for (i = 0; i < 79; i++)
    {
        reg_data = ((79 - i) << BT_RFC_VCO_REG3_TX_KCAL_Pos) + ((79 - i) << BT_RFC_VCO_REG3_BRF_VCO_IDAC_LV_Pos) + ((79 - i) << BT_RFC_VCO_REG3_BRF_VCO_PDX_LV_Pos) + 0x80000000;
        write_memory(BT_RFC_MEM_BASE + reg_addr, reg_data);
        reg_addr += 4;
    }
    //store bt tx cal result
    hwp_bt_rfc->CAL_ADDR_REG2 += reg_addr << 16;
    for (i = 0; i < 79; i++)
    {
        reg_data = (((i) << BT_RFC_EDR_CAL_REG1_BRF_OSLO_FC_LV_Pos) & BT_RFC_EDR_CAL_REG1_BRF_OSLO_FC_LV_Msk) + \
                   (((i) << BT_RFC_EDR_CAL_REG1_BRF_OSLO_BM_LV_Pos) & BT_RFC_EDR_CAL_REG1_BRF_OSLO_BM_LV_Msk) + \
                   (((i) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Pos) & BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_IDAC_LV_Msk) + \
                   (((i) << BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Pos) & BT_RFC_EDR_CAL_REG1_BRF_EDR_VCO_PDX_LV_Msk) + \
                   (((i) << BT_RFC_EDR_CAL_REG1_BRF_TRF_EDR_TMXCAP_SEL_LV_Pos) & BT_RFC_EDR_CAL_REG1_BRF_TRF_EDR_TMXCAP_SEL_LV_Msk) + ((i & 0x1F) << 27);

        //printf("reg_data is %x i=%d\n",reg_data, i);
        write_memory(BT_RFC_MEM_BASE + reg_addr, reg_data);
        reg_addr += 4;
    }
    //store txdc cal result
    hwp_bt_rfc->CAL_ADDR_REG3 = reg_addr;
    for (i = 0; i < 8; i++)
    {
        reg_data = (i + 1) + ((0x1000 + i + 1) << BT_RFC_IQ_PWR_REG1_0_TX_DC_CAL_COEF1_Pos) + ((i + 1) << BT_RFC_IQ_PWR_REG1_0_EDR_TMXBUF_GC_GFSK_Pos);
        write_memory(BT_RFC_REG_BASE + 0xAC + i * 0xC, reg_data);
        //reg_addr +=4;
        reg_data = (8 - i) + ((8 - i) << BT_RFC_IQ_PWR_REG2_0_TX_DC_CAL_OFFSET_I_Pos) + ((8 - i) << BT_RFC_IQ_PWR_REG2_0_BRF_TRF_EDR_PA_BM_LV_Pos) + ((8 - i) << BT_RFC_IQ_PWR_REG2_0_EDR_TMXBUF_GC_DPSK_Pos) + ((i % 2) << BT_RFC_IQ_PWR_REG2_0_EDR_LPF_BYPASS_Pos);
        write_memory(BT_RFC_REG_BASE + 0xB0 + i * 0xC, reg_data);
        //reg_addr +=4;
        //reg_data = 0x200 + i*0x40;
        write_memory(BT_RFC_REG_BASE + 0xB4 + i * 0xC, tmxbuf_cp_factor[i]);
    }

    write_memory(BT_RFC_MEM_BASE + 0x764, 0x73727170);
    write_memory(BT_RFC_MEM_BASE + 0x768, 0x77767574);
    write_memory(BT_RFC_MEM_BASE + 0x76C, 0x7b7a7978);
    write_memory(BT_RFC_MEM_BASE + 0x770, 0x7f7e7d7c);
    write_memory(BT_RFC_MEM_BASE + 0x774, 0x83828180);
    write_memory(BT_RFC_MEM_BASE + 0x778, 0x87868584);
    write_memory(BT_RFC_MEM_BASE + 0x77C, 0x8b8a8988);
    write_memory(BT_RFC_MEM_BASE + 0x780, 0x8f8e8d8c);

    write_memory(BT_RFC_MEM_BASE + 0x784, 0x011D0141);
    write_memory(BT_RFC_MEM_BASE + 0x788, 0x00E300FE);
    write_memory(BT_RFC_MEM_BASE + 0x78C, 0x00B400CA);
    write_memory(BT_RFC_MEM_BASE + 0x790, 0x008F00A0);
    write_memory(BT_RFC_MEM_BASE + 0x794, 0x0072007F);
    write_memory(BT_RFC_MEM_BASE + 0x798, 0x005A0065);
    write_memory(BT_RFC_MEM_BASE + 0x79C, 0x00480050);
    write_memory(BT_RFC_MEM_BASE + 0x7a0, 0x00390041);


    //}}}

    //temp bp dccal_coef
    //hwp_bt_phy->TX_DC_CAL_CFG3 |=BT_PHY_TX_DC_CAL_CFG3_TX_DC_CAL_COEF_FRC_EN ;

    //inccal time setting
    hwp_bt_rfc->INCCAL_REG1 |= (0x3f << BT_RFC_INCCAL_REG1_VCO3G_INCFCAL_WAIT_TIME_Pos) | \
                               (0x3f << BT_RFC_INCCAL_REG1_VCO3G_INCACAL_WAIT_TIME_Pos) ;
    hwp_bt_rfc->INCCAL_REG2 |= (0x3f << BT_RFC_INCCAL_REG2_VCO5G_INCFCAL_WAIT_TIME_Pos) | \
                               (0x3f << BT_RFC_INCCAL_REG2_VCO5G_INCACAL_WAIT_TIME_Pos) ;
    hwp_bt_rfc->INCCAL_REG1 &= ~BT_RFC_INCCAL_REG1_FRC_INCCAL_CLK_ON ;
    //printf("BLE rf inccal init start\n");
    //----------------RXON CMD----------------{{{
    i = 0;
    //VDDPSW/RFBG_EN/LO_IARY_EN
    //rxon_cmd[i++] = RD( 0x10 ) ;
    rxon_cmd[i++] = RD(0x10) ;
    rxon_cmd[i++] = OR(16) ;
    rxon_cmd[i++] = OR(17) ;
    rxon_cmd[i++] = OR(18) ;
    rxon_cmd[i++] = WR(0x10) ;

    //wait 2us
    rxon_cmd[i++] = WAIT(6) ;

    //VCO_EN
    rxon_cmd[i++] = RD(0x0) ;
    rxon_cmd[i++] = OR(12) ;
    rxon_cmd[i++] = WR(0x0) ;

    //FBDV_EN
    rxon_cmd[i++] = RD(0x14) ;
    rxon_cmd[i++] = OR(12) ;
    rxon_cmd[i++] = WR(0x14) ;

    //PFDCP_EN
    rxon_cmd[i++] = RD(0x1C) ;
    rxon_cmd[i++] = OR(19) ;
    rxon_cmd[i++] = WR(0x1C) ;

    //LDO11_EN & LNA_SHUNTSW
    rxon_cmd[i++] = RD(0x44) ;
    rxon_cmd[i++] = OR(22) ;
    rxon_cmd[i++] = AND(6) ;
    rxon_cmd[i++] = WR(0x44) ;

    //ADC & LDO_ADC & LDO_ADCREF
    rxon_cmd[i++] = RD(0x60) ;
    rxon_cmd[i++] = OR(4) ;
    rxon_cmd[i++] = OR(9) ;
    rxon_cmd[i++] = OR(21) ;
    rxon_cmd[i++] = OR(20) ;   //if disable adc-1, change to 22
    rxon_cmd[i++] = WR(0x60) ;

    //LDO_RBB
    rxon_cmd[i++] = RD(0x48) ;
    rxon_cmd[i++] = OR(13) ;
    rxon_cmd[i++] = WR(0x48) ;

    //PA_TX_RX
    rxon_cmd[i++] = RD(0x34) ;
    rxon_cmd[i++] = AND(9) ;
    rxon_cmd[i++] = WR(0x34) ;

    //EN_IARRAY & EN_OSDAC
    rxon_cmd[i++] = RD(0x58) ;
    rxon_cmd[i++] = OR(5) ;
    rxon_cmd[i++] = OR(6) ;
    rxon_cmd[i++] = OR(7) ;
    rxon_cmd[i++] = WR(0x58) ;

    //EN_CBPF & EN_RVGA
    rxon_cmd[i++] = RD(0x4C) ;
    rxon_cmd[i++] = OR(27) ;
    rxon_cmd[i++] = OR(6) ;
    rxon_cmd[i++] = OR(7) ;
    rxon_cmd[i++] = WR(0x4C) ;

    //EN_PKDET
    rxon_cmd[i++] = RD(0x50) ;
    rxon_cmd[i++] = OR(0) ;
    rxon_cmd[i++] = OR(1) ;
    rxon_cmd[i++] = OR(2) ;
    rxon_cmd[i++] = OR(3) ;
    rxon_cmd[i++] = WR(0x50) ;

    //wait 4us
    rxon_cmd[i++] = WAIT(12) ;

    //LODIST_RX_EN
    rxon_cmd[i++] = RD(0x10) ;
    rxon_cmd[i++] = OR(9) ;
    rxon_cmd[i++] = WR(0x10) ;

    //LNA_PU & MX_PU
    rxon_cmd[i++] = RD(0x44) ;
    rxon_cmd[i++] = OR(3) ;
    rxon_cmd[i++] = OR(17) ;
    rxon_cmd[i++] = WR(0x44) ;

    //REG_INIT
    if (i % 2)
    {
        rxon_cmd[i++] = RD(0) ;
    }
    rxon_cmd[i++] = REG_INIT(0x123);
    rxon_cmd[i++] = 0x4567 ;
    rxon_cmd[i++] = WR(0x6c) ;

    //FULCAL RSLT
    rxon_cmd[i++] = RD_FULCAL;
    rxon_cmd[i++] = WR(0x8) ;

    //wait 6us
    rxon_cmd[i++] = WAIT(18) ;

    //VCO_FLT_EN
    //rxon_cmd[i++] = RD( 0x0 ) ;
    //rxon_cmd[i++] = OR( 7 ) ;
    //rxon_cmd[i++] = WR( 0x0 ) ;

    //FBDV_RSTB
    rxon_cmd[i++] = RD(0x14) ;
    rxon_cmd[i++] = AND(0x7) ;
    rxon_cmd[i++] = WR(0x14) ;

    //wait 30us for lo lock
    rxon_cmd[i++] = WAIT(90) ;

    //START INCCAL
    rxon_cmd[i++] = RD(0x74) ;   //inccal start
    rxon_cmd[i++] = OR(29) ;
    rxon_cmd[i++] = WR(0x74) ;

    rxon_cmd[i++] = WAIT(30) ;
    //END
    rxon_cmd[i++] = END ;
    if (i % 2)
    {
        rxon_cmd[i++] = END ;
    }
    rxon_cmd_num = i ;
    //}}}

    //----------------RXOFF CMD----------------------{{{
    i = 0;
    //VDDPSW/RFBG/LODIST_RX_EN
    //rxoff_cmd[i++] = RD( 0x10 ) ; //to avoid rx on/off collision in normal rx
    //rxoff_cmd[i++] = END ; //to retain rx state when rx off in dc_est mode
    rxoff_cmd[i++] = RD(0x10) ;
    rxoff_cmd[i++] = AND(9) ;
    rxoff_cmd[i++] = AND(16) ;
    rxoff_cmd[i++] = AND(17) ;
    rxoff_cmd[i++] = AND(18) ;
    rxoff_cmd[i++] = WR(0x10) ;
    //VCO_EN
    rxoff_cmd[i++] = RD(0x0) ;
    rxoff_cmd[i++] = AND(12) ;
    rxoff_cmd[i++] = WR(0x0) ;
    //FBDV_EN
    //FBDV RSTB
    rxoff_cmd[i++] = RD(0x14) ;
    rxoff_cmd[i++] = AND(12) ;
    rxoff_cmd[i++] = OR(7) ;
    rxoff_cmd[i++] = WR(0x14) ;

    //PFDCP_EN
    rxoff_cmd[i++] = RD(0x1C) ;
    rxoff_cmd[i++] = AND(19) ;
    rxoff_cmd[i++] = WR(0x1C) ;

    //LNA_PU & MX_PU & LDO11_EN & LNA_SHUNTSW
    rxoff_cmd[i++] = RD(0x44) ;
    rxoff_cmd[i++] = AND(3) ;
    rxoff_cmd[i++] = OR(6) ;
    rxoff_cmd[i++] = AND(17) ;
    rxoff_cmd[i++] = AND(22) ;
    rxoff_cmd[i++] = WR(0x44) ;

    //ADC & LDO_ADC & LDO_ADCREF
    rxoff_cmd[i++] = RD(0x60) ;
    rxoff_cmd[i++] = AND(4) ;
    rxoff_cmd[i++] = AND(9) ;
    rxoff_cmd[i++] = AND(21) ;
    rxoff_cmd[i++] = AND(20) ;
    rxoff_cmd[i++] = WR(0x60) ;

    //LDO_RBB
    rxoff_cmd[i++] = RD(0x48) ;
    rxoff_cmd[i++] = AND(13) ;
    rxoff_cmd[i++] = WR(0x48) ;

    //PA_TX_RX
    rxoff_cmd[i++] = RD(0x34) ;
    rxoff_cmd[i++] = OR(9) ;
    rxoff_cmd[i++] = WR(0x34) ;

    //EN_IARRAY & EN_OSDAC
    rxoff_cmd[i++] = RD(0x58) ;
    rxoff_cmd[i++] = AND(5) ;
    rxoff_cmd[i++] = AND(6) ;
    rxoff_cmd[i++] = AND(7) ;
    rxoff_cmd[i++] = WR(0x58) ;

    //EN_CBPF & EN_RVGA
    rxoff_cmd[i++] = RD(0x4c) ;
    rxoff_cmd[i++] = AND(27) ;
    rxoff_cmd[i++] = AND(6) ;
    rxoff_cmd[i++] = AND(7) ;
    rxoff_cmd[i++] = WR(0x4c) ;

    //EN_PKDET
    rxoff_cmd[i++] = RD(0x50) ;
    rxoff_cmd[i++] = AND(0) ;
    rxoff_cmd[i++] = AND(1) ;
    rxoff_cmd[i++] = AND(2) ;
    rxoff_cmd[i++] = AND(3) ;
    rxoff_cmd[i++] = WR(0x50) ;
    //END
    rxoff_cmd[i++] = END ;
    if (i % 2)
    {
        rxoff_cmd[i++] = END ;
    }
    rxoff_cmd_num = i;

    //}}}

    //----------------Polar Mod TXON CMD----------------------{{{
    i = 0;
    //VDDPSW/RFBG_EN/LO_IARY_EN
    //txon_cmd[i++] = RD( 0x10 ) ;
    txon_cmd[i++] = RD(0x10) ;
    txon_cmd[i++] = OR(16) ;
    txon_cmd[i++] = OR(17) ;
    txon_cmd[i++] = OR(18) ;
    txon_cmd[i++] = WR(0x10) ;

    //wait 2us
    txon_cmd[i++] = WAIT(6) ;

    //VCO5G_EN
    txon_cmd[i++] = RD(0x0) ;
    txon_cmd[i++] = OR(12) ;
    txon_cmd[i++] = WR(0x0) ;

    //FBDV_EN
    txon_cmd[i++] = RD(0x14) ;
    txon_cmd[i++] = OR(12) ;
    txon_cmd[i++] = WR(0x14) ;

    //PFDCP_EN&PFDCP_CSD_EN
    txon_cmd[i++] = RD(0x1C) ;
    txon_cmd[i++] = OR(19) ;
    //txon_cmd[i++] = OR( 4 ) ;
    txon_cmd[i++] = WR(0x1C) ;

    //PA_BUF_PU for normal tx
    txon_cmd[i++] = RD(0x34) ;
    txon_cmd[i++] = OR(22) ;
    txon_cmd[i++] = WR(0x34) ;

    ////ATTEN EN for dc est
    //txon_cmd[i++] = RD( 0x28 ) ;
    //txon_cmd[i++] = OR( 4 ) ;
    //txon_cmd[i++] = WR( 0x28 ) ;

    //EDR_IARRAY_EN
    txon_cmd[i++] = RD(0x3c) ;
    txon_cmd[i++] = OR(20) ;
    txon_cmd[i++] = WR(0x3c) ;

    //TRF REG
    txon_cmd[i++] = RD_POLAR(0x764) ;
    txon_cmd[i++] = WR(0x38) ;

    //EDR_XFMR_SG
    txon_cmd[i++] = RD(0x40) ;
    txon_cmd[i++] = AND(11) ;
    txon_cmd[i++] = WR(0x40) ;

    //wait 4us
    txon_cmd[i++] = WAIT(12) ;

    //LODIST_BLETX_EN
    txon_cmd[i++] = RD(0x10) ;
    txon_cmd[i++] = OR(8) ;
    txon_cmd[i++] = WR(0x10) ;

    //PA_OUT_PU & TRF_SIG_EN
    txon_cmd[i++] = RD(0x34) ;
    txon_cmd[i++] = OR(16) ;
    txon_cmd[i++] = OR(21) ;   //pa_out_pu for normal tx
    //txon_cmd[i++] = OR( 16 ) ; //no pa_out_pu for dc est tx
    txon_cmd[i++] = WR(0x34) ;

    //RD FULCAL
    txon_cmd[i++] = RD_FULCAL ;
    txon_cmd[i++] = WR(0x8) ;

    //wait 6us
    txon_cmd[i++] = WAIT(18) ;

    //FBDV_RSTB
    txon_cmd[i++] = RD(0x14) ;
    txon_cmd[i++] = AND(0x7) ;
    txon_cmd[i++] = WR(0x14) ;

    //wait 30us for lo lock
    txon_cmd[i++] = WAIT(90) ;
    //START INCCAL
    txon_cmd[i++] = RD(0x74) ;   //inccal start
    txon_cmd[i++] = OR(29) ;
    txon_cmd[i++] = WR(0x74) ;
    txon_cmd[i++] = WAIT(30) ;

    //END
    txon_cmd[i++] = END ;
    if (i % 2)
    {
        txon_cmd[i++] = END ;
    }
    txon_cmd_num = i ;
    //}}}

    //----------------Polar Mod TXOFF CMD----------------------{{{
    i = 0;
    //VDDPSW/RFBG_EN/LODIST_BLETX_EN/LO_IARY_EN
    txoff_cmd[i++] = RD(0x10) ;
    //txoff_cmd[i++] = END ;
    txoff_cmd[i++] = AND(8) ;
    txoff_cmd[i++] = AND(16) ;
    txoff_cmd[i++] = AND(17) ;
    txoff_cmd[i++] = AND(18) ;
    txoff_cmd[i++] = WR(0x10) ;
    //VCO5G_EN
    txoff_cmd[i++] = RD(0x0) ;
    txoff_cmd[i++] = AND(12) ;
    txoff_cmd[i++] = WR(0x0) ;
    //FBDV_EN
    txoff_cmd[i++] = RD(0x14) ;
    txoff_cmd[i++] = AND(12) ;
    //FBDV_RSTB
    txoff_cmd[i++] = OR(7) ;
    txoff_cmd[i++] = WR(0x14) ;
    //PFDCP_EN
    txoff_cmd[i++] = RD(0x1C) ;
    txoff_cmd[i++] = AND(19) ;
    txoff_cmd[i++] = WR(0x1C) ;

    //PA_BUF_PU & PA_OUT_PU & TRF_SIG_EN
    txoff_cmd[i++] = RD(0x34) ;
    txoff_cmd[i++] = AND(22) ;
    txoff_cmd[i++] = AND(16) ;
    txoff_cmd[i++] = AND(21) ;
    txoff_cmd[i++] = WR(0x34) ;

    //TRF_EDR_IARRAY_EN
    txoff_cmd[i++] = RD(0x3c) ;
    txoff_cmd[i++] = AND(20) ;
    txoff_cmd[i++] = WR(0x3c) ;

    //EDR_XFMR_SG
    txoff_cmd[i++] = RD(0x40) ;
    txoff_cmd[i++] = OR(11) ;
    txoff_cmd[i++] = WR(0x40) ;

    //END
    txoff_cmd[i++] = END ;
    if (i % 2)
    {
        txoff_cmd[i++] = END ;
    }
    txoff_cmd_num = i ;

    //}}}

    //----------------IQ Mod TXON CMD----------------------{{{
    i = 0;
    //VDDPSW/RFBG_EN/LO_IARY_EN
    bt_txon_cmd[i++] = RD(0x10) ;
    bt_txon_cmd[i++] = OR(16) ;
    bt_txon_cmd[i++] = OR(17) ;
    bt_txon_cmd[i++] = OR(18) ;
    if (EDR_LO5G)bt_txon_cmd[i++] = OR(5) ;    //LODISTEDR_TX_SEL
    if (EDR_LO5G)bt_txon_cmd[i++] = OR(7) ;    //LODIST5G_EDRTX_EN
    bt_txon_cmd[i++] = OR(0) ;  //LODISTEDR_EN
    bt_txon_cmd[i++] = WR(0x10) ;

    //wait 2us
    bt_txon_cmd[i++] = WAIT(6) ;

    // VCO3G_EN /VCO5G_EN
    bt_txon_cmd[i++] = RD(0x0) ;
    if (EDR_LO5G)bt_txon_cmd[i++] = OR(12) ;
    if (EDR_LO3G)bt_txon_cmd[i++] = OR(13) ;
    bt_txon_cmd[i++] = WR(0x0) ;

    //FBDV_EN
    bt_txon_cmd[i++] = RD(0x14) ;
    bt_txon_cmd[i++] = OR(12) ;
    if (EDR_LO3G)bt_txon_cmd[i++] = AND(3) ;    //SDM_CLK_SEL
    if (EDR_LO3G)bt_txon_cmd[i++] = OR(4) ;    //MOD_STG[0]
    if (EDR_LO3G)bt_txon_cmd[i++] = AND(5) ;   //MOD_STG[1]
    bt_txon_cmd[i++] = WR(0x14) ;

    //PFDCP_EN&PFDCP_CSD_EN
    bt_txon_cmd[i++] = RD(0x1C) ;
    bt_txon_cmd[i++] = OR(19) ;
    //txon_cmd[i++] = OR( 4 ) ;
    bt_txon_cmd[i++] = WR(0x1C) ;

    //OSLO for 3G
    if (EDR_LO3G)bt_txon_cmd[i++] = RD(0x28) ;
    if (EDR_LO3G)bt_txon_cmd[i++] = OR(11) ;
    if (EDR_LO3G)bt_txon_cmd[i++] = WR(0x28) ;

    //EN_TBB_IARRY & EN_LDO_DAC_AVDD & EN_LDO_DAC_DVDD & EN_DAC
    bt_txon_cmd[i++] = RD(0x64) ;
    bt_txon_cmd[i++] = OR(8) ;
    bt_txon_cmd[i++] = OR(9) ;
    bt_txon_cmd[i++] = OR(10) ;
    bt_txon_cmd[i++] = OR(11) ;
    bt_txon_cmd[i++] = WR(0x64) ;

    //TRF_EDR_IARRAY_EN
    bt_txon_cmd[i++] = RD(0x3c) ;
    bt_txon_cmd[i++] = OR(20) ;
    bt_txon_cmd[i++] = WR(0x3c) ;

    //EDR_PACAP_EN/EDR_PA_XFMR_SG
    bt_txon_cmd[i++] = RD(0x40) ;
    bt_txon_cmd[i++] = OR(11) ;
    bt_txon_cmd[i++] = OR(17) ;
    bt_txon_cmd[i++] = WR(0x40) ;

    //RD FULCAL
    bt_txon_cmd[i++] = RD_FULCAL ;
    bt_txon_cmd[i++] = WR(0x24) ;
    bt_txon_cmd[i++] = RD_FULCAL + 0x800 ;
    bt_txon_cmd[i++] = WR(0x8) ;
    //RD DCCAL
    //bt_txon_cmd[i++] = RD_DCCAL1 ;
    //bt_txon_cmd[i++] = WR( 0xA8 ) ;
    //bt_txon_cmd[i++] = RD_DCCAL2 ;
    //bt_txon_cmd[i++] = WR( 0xAC ) ;

    bt_txon_cmd[i++] = RD_FACTOR(0x784);
    bt_txon_cmd[i++] = WR(0xA8) ;
    //wait 6us
    bt_txon_cmd[i++] = WAIT(18) ;

    //EDR_TMXBUF_PU EDR_TMX_PU
    bt_txon_cmd[i++] = RD(0x3c) ;
    bt_txon_cmd[i++] = OR(12) ;
    bt_txon_cmd[i++] = OR(19) ;
    bt_txon_cmd[i++] = WR(0x3c) ;

    //wait 5us
    bt_txon_cmd[i++] = WAIT(15) ;

    //DAC_START
    bt_txon_cmd[i++] = RD(0x64) ;
    bt_txon_cmd[i++] = OR(12) ;
    bt_txon_cmd[i++] = WR(0x64) ;

    //EDR_PA_PU
    bt_txon_cmd[i++] = RD(0x3c) ;
    bt_txon_cmd[i++] = OR(2) ;
    bt_txon_cmd[i++] = WR(0x3c) ;

    //VCO_FLT_EN
    //bt_txon_cmd[i++] = RD( 0x0 ) ;
    //bt_txon_cmd[i++] = OR( 7 ) ;
    //bt_txon_cmd[i++] = WR( 0x0 ) ;

    //FBDV_RSTB
    bt_txon_cmd[i++] = RD(0x14) ;
    bt_txon_cmd[i++] = AND(0x7) ;
    bt_txon_cmd[i++] = WR(0x14) ;

    //wait 30us for lo lock
    bt_txon_cmd[i++] = WAIT(90) ;
    //START INCCAL
    bt_txon_cmd[i++] = RD(0x74) ;   //inccal start
    bt_txon_cmd[i++] = OR(29) ;
    bt_txon_cmd[i++] = WR(0x74) ;
    bt_txon_cmd[i++] = WAIT(30) ;

    //END
    bt_txon_cmd[i++] = END ;
    if (i % 2)
    {
        bt_txon_cmd[i++] = END ;
    }
    bt_txon_cmd_num = i ;
    //}}}

    //----------------IQ Mod TXOFF CMD----------------------{{{
    i = 0;


    //VCO_EN
    bt_txoff_cmd[i++] = RD(0x0) ;
    if (EDR_LO5G)bt_txoff_cmd[i++] = AND(12) ;
    if (EDR_LO3G)bt_txoff_cmd[i++] = AND(13) ;
    bt_txoff_cmd[i++] = WR(0x0) ;

    //OSLO for 3G
    if (EDR_LO3G)bt_txoff_cmd[i++] = RD(0x28) ;
    if (EDR_LO3G)bt_txoff_cmd[i++] = AND(11) ;
    if (EDR_LO3G)bt_txoff_cmd[i++] = WR(0x28) ;

    //VDDPSW/RFBG_EN/LODIST_BLETX_EN/LO_IARY_EN
    bt_txoff_cmd[i++] = RD(0x10) ;
    bt_txoff_cmd[i++] = AND(16) ;
    bt_txoff_cmd[i++] = AND(17) ;
    bt_txoff_cmd[i++] = AND(18) ;
    if (EDR_LO5G)bt_txoff_cmd[i++] = AND(5) ;    //LODISTEDR_TX_SEL
    if (EDR_LO5G)bt_txoff_cmd[i++] = AND(7) ;    //LODIST5G_EDRTX_EN
    bt_txoff_cmd[i++] = AND(0) ;  //LODISTEDR_EN
    bt_txoff_cmd[i++] = WR(0x10) ;
    //FBDV_EN
    bt_txoff_cmd[i++] = RD(0x14) ;
    bt_txoff_cmd[i++] = AND(12) ;
    if (EDR_LO3G)bt_txoff_cmd[i++] = OR(3) ;    //SDM_CLK_SEL
    if (EDR_LO3G)bt_txoff_cmd[i++] = AND(4) ;    //MOD_STG[0]
    if (EDR_LO3G)bt_txoff_cmd[i++] = OR(5) ;   //MOD_STG[1]
    //FBDV_RSTB
    bt_txoff_cmd[i++] = OR(7) ;
    bt_txoff_cmd[i++] = WR(0x14) ;
    //PFDCP_EN
    bt_txoff_cmd[i++] = RD(0x1C) ;
    bt_txoff_cmd[i++] = AND(19) ;
    bt_txoff_cmd[i++] = WR(0x1C) ;

    //DAC_START
    //EN_TBB_IARRY & EN_LDO_DAC_AVDD & EN_LDO_DAC_DVDD & EN_DAC
    bt_txoff_cmd[i++] = RD(0x64) ;
    bt_txoff_cmd[i++] = AND(8) ;
    bt_txoff_cmd[i++] = AND(9) ;
    bt_txoff_cmd[i++] = AND(10) ;
    bt_txoff_cmd[i++] = AND(11) ;
    bt_txoff_cmd[i++] = AND(12) ;
    bt_txoff_cmd[i++] = WR(0x64) ;

    //EDR_PA_PU
    //TRF_EDR_IARRAY_EN
    //EDR_TMXBUF_PU EDR_TMX_PU
    bt_txoff_cmd[i++] = RD(0x3c) ;
    bt_txoff_cmd[i++] = AND(2) ;
    bt_txoff_cmd[i++] = AND(12) ;
    bt_txoff_cmd[i++] = AND(19) ;
    bt_txoff_cmd[i++] = AND(20) ;
    bt_txoff_cmd[i++] = WR(0x3c) ;

    //EDR_PACAP_EN / EDR_PA_XFMR_SG
    bt_txoff_cmd[i++] = RD(0x40) ;
    bt_txoff_cmd[i++] = AND(11) ;
    bt_txoff_cmd[i++] = AND(17) ;
    bt_txoff_cmd[i++] = WR(0x40) ;

    //END
    bt_txoff_cmd[i++] = END ;
    if (i % 2)
    {
        bt_txoff_cmd[i++] = END ;
    }
    bt_txoff_cmd_num = i ;

    //}}}

    //printf("BLE rf rxon inccal init start\n");
    rxon_addr = reg_addr + 4;
    hwp_bt_rfc->CU_ADDR_REG1 = 0;
    hwp_bt_rfc->CU_ADDR_REG1 = rxon_addr;
    for (i = 0; i < rxon_cmd_num / 2; i = i + 1)
    {
        //printf("cmd_addr = %x\n",rxon_addr );
        //printf("rxon_cmd[%d] = %x\n",i*2+1,rxon_cmd[i*2+1]);
        //printf("rxon_cmd[%d] = %x\n",i*2,rxon_cmd[i*2]);
        cmd = rxon_cmd[i * 2] + (rxon_cmd[i * 2 + 1] << 16) ;
        write_memory(BT_RFC_MEM_BASE + rxon_addr, cmd);
        rxon_addr += 4;
    }
    //printf("BLE rf rxoff inccal init start\n");
    //rxoff_addr = BT_RFC_BASE + 0x298;//0x41040198;
    rxoff_addr = rxon_addr + 4 ;
    hwp_bt_rfc->CU_ADDR_REG1 += (rxoff_addr << 16);
    for (i = 0; i < rxoff_cmd_num / 2; i = i + 1)
    {
        //printf("cmd_addr = %x\n",rxoff_addr );
        //printf("rxoff_cmd[%d] = %x\n",i*2+1,rxoff_cmd[i*2+1]);
        //printf("rxoff_cmd[%d] = %x\n",i*2,rxoff_cmd[i*2]);
        cmd = rxoff_cmd[i * 2] + (rxoff_cmd[i * 2 + 1] << 16) ;
        write_memory(BT_RFC_MEM_BASE + rxoff_addr, cmd);
        rxoff_addr += 4 ;
    }

    //txon_addr = BT_RFC_BASE + 0x304;//0x41040204;
    txon_addr = rxoff_addr + 0x4;
    hwp_bt_rfc->CU_ADDR_REG2 = 0;
    hwp_bt_rfc->CU_ADDR_REG2 = txon_addr ;
    for (i = 0; i < txon_cmd_num / 2; i = i + 1)
    {
        //printf("cmd_addr = %x\n",txon_addr );
        //printf("txon_cmd[%d] = %x\n",i*2+1,txon_cmd[i*2+1]);
        //printf("txon_cmd[%d] = %x\n",i*2,txon_cmd[i*2]);
        cmd = txon_cmd[i * 2] + (txon_cmd[i * 2 + 1] << 16) ;
        write_memory(BT_RFC_MEM_BASE + txon_addr, cmd);
        txon_addr += 4;
    }
    //printf("BLE rf rxoff inccal init start\n");
    txoff_addr = txon_addr + 0x4;//0x4104025C;
    hwp_bt_rfc->CU_ADDR_REG2 += (txoff_addr << 16);
    for (i = 0; i < txoff_cmd_num / 2; i = i + 1)
    {
        //printf("cmd_addr = %x\n",txoff_addr );
        //printf("txoff_cmd[%d] = %x\n",i*2+1,txoff_cmd[i*2+1]);
        //printf("txoff_cmd[%d] = %x\n",i*2,txoff_cmd[i*2]);
        cmd = txoff_cmd[i * 2] + (txoff_cmd[i * 2 + 1] << 16) ;
        write_memory(BT_RFC_MEM_BASE + txoff_addr, cmd);
        txoff_addr += 4 ;
    }

    bt_txon_addr = txoff_addr + 0x4;
    hwp_bt_rfc->CU_ADDR_REG3 = 0;
    hwp_bt_rfc->CU_ADDR_REG3 = bt_txon_addr ;
    for (i = 0; i < bt_txon_cmd_num / 2; i = i + 1)
    {
        //printf("cmd_addr = %x\n",txon_addr );
        //printf("txon_cmd[%d] = %x\n",i*2+1,txon_cmd[i*2+1]);
        //printf("txon_cmd[%d] = %x\n",i*2,txon_cmd[i*2]);
        cmd = bt_txon_cmd[i * 2] + (bt_txon_cmd[i * 2 + 1] << 16) ;
        write_memory(BT_RFC_MEM_BASE + bt_txon_addr, cmd);
        bt_txon_addr += 4;
    }
    //printf("BLE rf rxoff inccal init start\n");
    bt_txoff_addr = bt_txon_addr + 0x4;//0x4104025C;
    hwp_bt_rfc->CU_ADDR_REG3 += (bt_txoff_addr << 16);
    for (i = 0; i < bt_txoff_cmd_num / 2; i = i + 1)
    {
        //printf("cmd_addr = %x\n",txoff_addr );
        //printf("txoff_cmd[%d] = %x\n",i*2+1,txoff_cmd[i*2+1]);
        //printf("txoff_cmd[%d] = %x\n",i*2,txoff_cmd[i*2]);
        cmd = bt_txoff_cmd[i * 2] + (bt_txoff_cmd[i * 2 + 1] << 16) ;
        write_memory(BT_RFC_MEM_BASE + bt_txoff_addr, cmd);
        bt_txoff_addr += 4 ;
    }
    //printf("cmd_addr = %x\n",bt_txoff_addr );
}

rf_power_inf_t *rf_power_env;
uint32_t *rf_power_field_base;
rf_power_inf_t *rf_get_base_inf(void)
{
    return rf_power_env;
}

void rf_power_env_init(void)
{
    rf_power_env = (rf_power_inf_t *)LCPU_RF_CONFIG_START_ADDR;
    rf_power_env->magic_num = 0x57;
    rf_power_env->ver_id = 0;
    rf_power_env->start_offset = 4;
    rf_power_field_base = (uint32_t *)rf_power_env + rf_power_env->start_offset;
}

void rf_power_field_set(uint8_t index, uint32_t value)
{
    uint32_t *pt_power;

    pt_power = (rf_power_field_base + index);

    *pt_power = value;

}
#define FPGA_RFTEST_CASE2
#ifdef FPGA_RFTEST_CASE0 //case 0 //fix power
void rf_power_config(void)
{
    rf_power_env_init();

    rf_power_env->max_ble_pwr = 7;
    rf_power_env->max_br_pwr = 7;
    rf_power_env->max_edr_pwr = 7;
    rf_power_env->min_ble_pwr = 7;
    rf_power_env->min_br_pwr = 7;
    rf_power_env->min_edr_pwr = 7;
    rf_power_env->max_ble_index = 0;
    rf_power_env->max_br_index = 0;
    rf_power_env->max_edr_index = 0;
    rf_power_env->min_ble_index = 0;
    rf_power_env->min_br_index = 0;
    rf_power_env->min_edr_index = 0;

    uint32_t power0 = 0;
    rf_power_field_t *pt_pwr = (rf_power_field_t *)&power0;
    pt_pwr->mod_cntl = 1;//IQ  0-->MAX POWER

    rf_power_field_set(0, power0);
}
#endif
#ifdef FPGA_RFTEST_CASE1
void rf_power_config(void)
{
    rf_power_env_init();

    rf_power_env->min_br_pwr = 7;
    rf_power_env->max_br_pwr = 7;
    rf_power_env->min_edr_pwr = 7;
    rf_power_env->max_edr_pwr = 7;
    rf_power_env->min_ble_pwr = 7;
    rf_power_env->max_ble_pwr = 7;

    rf_power_env->min_br_index = 0;
    rf_power_env->max_br_index = 0;
    rf_power_env->min_edr_index = 1;
    rf_power_env->max_edr_index = 1;
    rf_power_env->min_ble_index = 2;
    rf_power_env->max_ble_index = 2;

    uint32_t power0 = 0;
    rf_power_field_t *pt_pwr = (rf_power_field_t *)&power0;
    pt_pwr->mod_cntl = 1;//IQ  0-->MAX POWER
    pt_pwr->br_iq_level = 1;
    pt_pwr->br_iq_pwr = 1;
    pt_pwr->polar_level = 3;
    pt_pwr->polar_pwr = 6;
    rf_power_field_set(0, power0);

    power0 = 0;
    pt_pwr->mod_cntl = 1;//IQ  0-->MAX POWER
    pt_pwr->edr_iq_level = 5;
    pt_pwr->edr_iq_pwr = 0;
    rf_power_field_set(1, power0);

    power0 = 0;
    pt_pwr->mod_cntl = 1;//IQ  0-->MAX POWER
    pt_pwr->br_iq_level = 2;
    pt_pwr->br_iq_pwr = 2;
    pt_pwr->polar_level = 4;
    pt_pwr->polar_pwr = 7;
    rf_power_field_set(2, power0);
}
#endif
#ifdef FPGA_RFTEST_CASE2
void rf_power_config(void)
{
    rf_power_env_init();

    rf_power_env->min_br_pwr = 3;
    rf_power_env->max_br_pwr = 9;
    rf_power_env->min_edr_pwr = 3;
    rf_power_env->max_edr_pwr = 9;
    rf_power_env->min_ble_pwr = 3;
    rf_power_env->max_ble_pwr = 10;

    rf_power_env->min_br_index = 0;
    rf_power_env->max_br_index = 2;
    rf_power_env->min_edr_index = 0;
    rf_power_env->max_edr_index = 2;
    rf_power_env->min_ble_index = 3;
    rf_power_env->max_ble_index = 5;

    uint32_t power0 = 0;
    rf_power_field_t *pt_pwr = (rf_power_field_t *)&power0;
    pt_pwr->mod_cntl = 1;//IQ  0-->MAX POWER
    pt_pwr->br_iq_level = 1;
    pt_pwr->br_iq_pwr = 3;
    pt_pwr->polar_level = 2;
    pt_pwr->polar_pwr = 2;
    pt_pwr->edr_iq_level = 3;
    pt_pwr->edr_iq_pwr = 2;
    pt_pwr->pwr_dbm = 3;
    rf_power_field_set(0, power0);

    power0 = 0;
    pt_pwr->mod_cntl = 1;//IQ  0-->MAX POWER
    pt_pwr->br_iq_level = 2;
    pt_pwr->br_iq_pwr = 2;
    pt_pwr->polar_level = 3;
    pt_pwr->polar_pwr = 1;
    pt_pwr->edr_iq_level = 4;
    pt_pwr->edr_iq_pwr = 1;
    pt_pwr->pwr_dbm = 6;
    rf_power_field_set(1, power0);

    power0 = 0;
    pt_pwr->mod_cntl = 1;//IQ  0-->MAX POWER
    pt_pwr->br_iq_level = 3;
    pt_pwr->br_iq_pwr = 1;
    pt_pwr->polar_level = 4;
    pt_pwr->polar_pwr = 0;
    pt_pwr->edr_iq_level = 5;
    pt_pwr->edr_iq_pwr = 0;
    pt_pwr->pwr_dbm = 9;
    rf_power_field_set(2, power0);

    power0 = 0;
    pt_pwr->mod_cntl = 1;//IQ  0-->MAX POWER
    pt_pwr->br_iq_level = 4;
    pt_pwr->br_iq_pwr = 2;
    pt_pwr->polar_level = 5;
    pt_pwr->polar_pwr = 4;
    pt_pwr->pwr_dbm = 3;
    rf_power_field_set(3, power0);

    power0 = 0;
    pt_pwr->mod_cntl = 1;//IQ  0-->MAX POWER
    pt_pwr->br_iq_level = 5;
    pt_pwr->br_iq_pwr = 1;
    pt_pwr->polar_level = 6;
    pt_pwr->polar_pwr = 5;
    pt_pwr->pwr_dbm = 6;
    rf_power_field_set(4, power0);

    power0 = 0;
    pt_pwr->mod_cntl = 1;//IQ  0-->MAX POWER
    pt_pwr->br_iq_level = 7;
    pt_pwr->br_iq_pwr = 0;
    pt_pwr->polar_level = 7;
    pt_pwr->polar_pwr = 15;
    pt_pwr->pwr_dbm = 10;
    rf_power_field_set(5, power0);
}
#endif

void bt_rf_cal_9364(void)
{
    bt_rfc_init();
    ad9364_calibration();
    ad9364_bt_cfg();

    rf_power_config();
}

