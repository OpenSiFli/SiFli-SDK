/*
 * SPDX-FileCopyrightText: 2019-2025 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __PTM_INS_H__
#define __PTM_INS_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WIN32
#include "register.h"
#endif /* !WIN32 */

#ifdef PTM1_BASE
//PTM register macros used in parameter rn/rd/rm of all instruction
#define PTM_X      0
#define PTM_Y      1
#define PTM_A      2
#define PTM_B      3
#define PTM_XL     4
#define PTM_XH     5
#define PTM_YL     6
#define PTM_YH     7
#define PTM_OS     8
#define PTM_OC     9
#define PTM_OEREV 10
#define PTM_IN    11
#define PTM_FIFO  12
#define PTM_EVTS  13
#define PTM_EVTC  14
#define PTM_ALL1  14
#define PTM_ZERO  15

//PTM IO input macros used in parameter rn of JMP instruction
#define PTM_IN0   0
#define PTM_IN1   1
#define PTM_IN2   2
#define PTM_IN3   3
#define PTM_IN4   4
#define PTM_IN5   5
#define PTM_IN6   6
#define PTM_IN7   7
#define PTM_IN8   8
#define PTM_IN9   9
#define PTM_IN10 10
#define PTM_IN11 11
#define PTM_IN12 12
#define PTM_IN13 13
#define PTM_IN14 14
#define PTM_IN15 15

//PTM event macros used in parameter rn of JMP instruction
#define PTM_EVT0   0
#define PTM_EVT1   1
#define PTM_EVT2   2
#define PTM_EVT3   3
#define PTM_EVT4   4
#define PTM_EVT5   5
#define PTM_EVT6   6
#define PTM_EVT7   7
#define PTM_EVT8   8
#define PTM_EVT9   9
#define PTM_EVT10 10
#define PTM_EVT11 11
#define PTM_EVT12 12
#define PTM_EVT13 13
#define PTM_EVT14 14
#define PTM_EVT15 15

//PTM operation macros used in parameter op of SET instruction
#define OP_NOP    0
#define OP_XOR    1
#define OP_AND    2
#define OP_OR     3
#define OP_LSL    4
#define OP_LSR    5
#define OP_ADD    6
#define OP_ACCUM  7
#define OP_REV32  8
#define OP_REV16  9
#define OP_REV8  10

//PTM target macros used in parameter tgt of JMP instruction
#define JMP_TCM0   (128+  0)
#define JMP_TCM1   (128+  1)
#define JMP_TCM2   (128+  2)
#define JMP_TCM3   (128+  3)
#define JMP_TCM4   (128+  4)
#define JMP_TCM5   (128+  5)
#define JMP_TCM6   (128+  6)
#define JMP_TCM7   (128+  7)
#define JMP_TCM8   (128+  8)
#define JMP_TCM9   (128+  9)
#define JMP_TCM10  (128+ 10)
#define JMP_TCM11  (128+ 11)
#define JMP_TCM12  (128+ 12)
#define JMP_TCM13  (128+ 13)
#define JMP_TCM14  (128+ 14)
#define JMP_TCM15  (128+ 15)
#define JMP_TCM16  (128+ 16)
#define JMP_TCM17  (128+ 17)
#define JMP_TCM18  (128+ 18)
#define JMP_TCM19  (128+ 19)
#define JMP_TCM20  (128+ 20)
#define JMP_TCM21  (128+ 21)
#define JMP_TCM22  (128+ 22)
#define JMP_TCM23  (128+ 23)
#define JMP_TCM24  (128+ 24)
#define JMP_TCM25  (128+ 25)
#define JMP_TCM26  (128+ 26)
#define JMP_TCM27  (128+ 27)
#define JMP_TCM28  (128+ 28)
#define JMP_TCM29  (128+ 29)
#define JMP_TCM30  (128+ 30)
#define JMP_TCM31  (128+ 31)
#define JMP_TCM32  (128+ 32)
#define JMP_TCM33  (128+ 33)
#define JMP_TCM34  (128+ 34)
#define JMP_TCM35  (128+ 35)
#define JMP_TCM36  (128+ 36)
#define JMP_TCM37  (128+ 37)
#define JMP_TCM38  (128+ 38)
#define JMP_TCM39  (128+ 39)
#define JMP_TCM40  (128+ 40)
#define JMP_TCM41  (128+ 41)
#define JMP_TCM42  (128+ 42)
#define JMP_TCM43  (128+ 43)
#define JMP_TCM44  (128+ 44)
#define JMP_TCM45  (128+ 45)
#define JMP_TCM46  (128+ 46)
#define JMP_TCM47  (128+ 47)
#define JMP_TCM48  (128+ 48)
#define JMP_TCM49  (128+ 49)
#define JMP_TCM50  (128+ 50)
#define JMP_TCM51  (128+ 51)
#define JMP_TCM52  (128+ 52)
#define JMP_TCM53  (128+ 53)
#define JMP_TCM54  (128+ 54)
#define JMP_TCM55  (128+ 55)
#define JMP_TCM56  (128+ 56)
#define JMP_TCM57  (128+ 57)
#define JMP_TCM58  (128+ 58)
#define JMP_TCM59  (128+ 59)
#define JMP_TCM60  (128+ 60)
#define JMP_TCM61  (128+ 61)
#define JMP_TCM62  (128+ 62)
#define JMP_TCM63  (128+ 63)
#define JMP_TCM64  (128+ 64)
#define JMP_TCM65  (128+ 65)
#define JMP_TCM66  (128+ 66)
#define JMP_TCM67  (128+ 67)
#define JMP_TCM68  (128+ 68)
#define JMP_TCM69  (128+ 69)
#define JMP_TCM70  (128+ 70)
#define JMP_TCM71  (128+ 71)
#define JMP_TCM72  (128+ 72)
#define JMP_TCM73  (128+ 73)
#define JMP_TCM74  (128+ 74)
#define JMP_TCM75  (128+ 75)
#define JMP_TCM76  (128+ 76)
#define JMP_TCM77  (128+ 77)
#define JMP_TCM78  (128+ 78)
#define JMP_TCM79  (128+ 79)
#define JMP_TCM80  (128+ 80)
#define JMP_TCM81  (128+ 81)
#define JMP_TCM82  (128+ 82)
#define JMP_TCM83  (128+ 83)
#define JMP_TCM84  (128+ 84)
#define JMP_TCM85  (128+ 85)
#define JMP_TCM86  (128+ 86)
#define JMP_TCM87  (128+ 87)
#define JMP_TCM88  (128+ 88)
#define JMP_TCM89  (128+ 89)
#define JMP_TCM90  (128+ 90)
#define JMP_TCM91  (128+ 91)
#define JMP_TCM92  (128+ 92)
#define JMP_TCM93  (128+ 93)
#define JMP_TCM94  (128+ 94)
#define JMP_TCM95  (128+ 95)
#define JMP_TCM96  (128+ 96)
#define JMP_TCM97  (128+ 97)
#define JMP_TCM98  (128+ 98)
#define JMP_TCM99  (128+ 99)
#define JMP_TCM100 (128+100)
#define JMP_TCM101 (128+101)
#define JMP_TCM102 (128+102)
#define JMP_TCM103 (128+103)
#define JMP_TCM104 (128+104)
#define JMP_TCM105 (128+105)
#define JMP_TCM106 (128+106)
#define JMP_TCM107 (128+107)
#define JMP_TCM108 (128+108)
#define JMP_TCM109 (128+109)
#define JMP_TCM110 (128+110)
#define JMP_TCM111 (128+111)
#define JMP_TCM112 (128+112)
#define JMP_TCM113 (128+113)
#define JMP_TCM114 (128+114)
#define JMP_TCM115 (128+115)
#define JMP_TCM116 (128+116)
#define JMP_TCM117 (128+117)
#define JMP_TCM118 (128+118)
#define JMP_TCM119 (128+119)
#define JMP_TCM120 (128+120)
#define JMP_TCM121 (128+121)
#define JMP_TCM122 (128+122)
#define JMP_TCM123 (128+123)
#define JMP_TCM124 (128+124)
#define JMP_TCM125 (128+125)
#define JMP_TCM126 (128+126)
#define JMP_TCM127 (128+127)
#define JMP_X       256
#define JMP_Y       257

//PTM condition macros used in parameter cond of JMP instruction
#define JMP_EQ      0
#define JMP_NEQ     1
#define JMP_EQCH    2
#define JMP_EQEVT   3
#define JMP_EQDEC   4
#define JMP_NEQDEC  5
#define JMP_GT      6
#define JMP_LT      7

//PTM wait macros used in parameter src of WAIT instruction
#define WAIT_PA00     (128+ 0)
#define WAIT_PA01     (128+ 1)
#define WAIT_PA02     (128+ 2)
#define WAIT_PA03     (128+ 3)
#define WAIT_PA04     (128+ 4)
#define WAIT_PA05     (128+ 5)
#define WAIT_PA06     (128+ 6)
#define WAIT_PA07     (128+ 7)
#define WAIT_PA08     (128+ 8)
#define WAIT_PA09     (128+ 9)
#define WAIT_PA10     (128+10)
#define WAIT_PA11     (128+11)
#define WAIT_PA12     (128+12)
#define WAIT_PA13     (128+13)
#define WAIT_PA14     (128+14)
#define WAIT_PA15     (128+15)
#define WAIT_PA16     (128+16)
#define WAIT_PA17     (128+17)
#define WAIT_PA18     (128+18)
#define WAIT_PA19     (128+19)
#define WAIT_PA20     (128+20)
#define WAIT_PA21     (128+21)
#define WAIT_PA22     (128+22)
#define WAIT_PA23     (128+23)
#define WAIT_PA24     (128+24)
#define WAIT_PA25     (128+25)
#define WAIT_PA26     (128+26)
#define WAIT_PA27     (128+27)
#define WAIT_PA28     (128+28)
#define WAIT_PA29     (128+29)
#define WAIT_PA30     (128+30)
#define WAIT_PA31     (128+31)
#define WAIT_PA32     (128+32)
#define WAIT_PA33     (128+33)
#define WAIT_PA34     (128+34)
#define WAIT_PA35     (128+35)
#define WAIT_PA36     (128+36)
#define WAIT_PA37     (128+37)
#define WAIT_PA38     (128+38)
#define WAIT_PA39     (128+39)
#define WAIT_PA40     (128+40)
#define WAIT_PA41     (128+41)
#define WAIT_PA42     (128+42)
#define WAIT_PA43     (128+43)
#define WAIT_PA44     (128+44)
#define WAIT_PA45     (128+45)
#define WAIT_PA46     (128+46)
#define WAIT_PA47     (128+47)
#define WAIT_PA48     (128+48)
#define WAIT_PA49     (128+49)
#define WAIT_PA50     (128+50)
#define WAIT_PA51     (128+51)
#define WAIT_PA52     (128+52)
#define WAIT_PA53     (128+53)
#define WAIT_PA54     (128+54)
#define WAIT_PA55     (128+55)
#define WAIT_PA56     (128+56)
#define WAIT_PA57     (128+57)
#define WAIT_PA58     (128+58)
#define WAIT_PA59     (128+59)
#define WAIT_PA60     (128+60)
#define WAIT_PA61     (128+61)
#define WAIT_PA62     (128+62)
#define WAIT_PA63     (128+63)
#define WAIT_PA64     (128+64)
#define WAIT_PA65     (128+65)
#define WAIT_PA66     (128+66)
#define WAIT_PA67     (128+67)
#define WAIT_PA68     (128+68)
#define WAIT_PA69     (128+69)
#define WAIT_PA70     (128+70)
#define WAIT_PA71     (128+71)
#define WAIT_PA72     (128+72)
#define WAIT_PA73     (128+73)
#define WAIT_PA74     (128+74)
#define WAIT_PA75     (128+75)
#define WAIT_PA76     (128+76)
#define WAIT_PA77     (128+77)
#define WAIT_PA78     (128+78)
#define WAIT_PA79     (128+79)
#define WAIT_OEMPTY   216
#define WAIT_OFULL    217
#define WAIT_IEMPTY   218
#define WAIT_IFULL    219
#define WAIT_IN0      224
#define WAIT_IN1      225
#define WAIT_IN2      226
#define WAIT_IN3      227
#define WAIT_IN4      228
#define WAIT_IN5      229
#define WAIT_IN6      230
#define WAIT_IN7      231
#define WAIT_IN8      232
#define WAIT_IN9      233
#define WAIT_IN10     234
#define WAIT_IN11     235
#define WAIT_IN12     236
#define WAIT_IN13     237
#define WAIT_IN14     238
#define WAIT_IN15     239
#define WAIT_EVT0     240
#define WAIT_EVT1     241
#define WAIT_EVT2     242
#define WAIT_EVT3     243
#define WAIT_EVT4     244
#define WAIT_EVT5     245
#define WAIT_EVT6     246
#define WAIT_EVT7     247
#define WAIT_EVT8     248
#define WAIT_EVT9     249
#define WAIT_EVT10    250
#define WAIT_EVT11    251
#define WAIT_EVT12    252
#define WAIT_EVT13    253
#define WAIT_EVT14    254
#define WAIT_EVT15    255

//PTM output macros used in parameter Iocnt of IO instruction
#define IO_PUSH0     0
#define IO_PUSH1     1
#define IO_PUSH2     2
#define IO_PUSH3     3
#define IO_PUSH4     4
#define IO_PUSH5     5
#define IO_PUSH6     6
#define IO_PUSH7     7
#define IO_PUSH8     8
#define IO_PUSH9     9
#define IO_PUSH10   10
#define IO_PUSH11   11
#define IO_PUSH12   12
#define IO_PUSH13   13
#define IO_PUSH14   14
#define IO_PUSH15   15
#define IO_PUSH16   16

//PTM input macros used in parameter IOcnt of IO instruction
#define IO_PULL0    32
#define IO_PULL1    33
#define IO_PULL2    34
#define IO_PULL3    35
#define IO_PULL4    36
#define IO_PULL5    37
#define IO_PULL6    38
#define IO_PULL7    39
#define IO_PULL8    40
#define IO_PULL9    41
#define IO_PULL10   42
#define IO_PULL11   43
#define IO_PULL12   44
#define IO_PULL13   45
#define IO_PULL14   46
#define IO_PULL15   47
#define IO_PULL16   48

//PTM single output macros used in parameter Om and On of IO instruction
#define IO_NOP      0
#define IO_OUT0_R   16
#define IO_OUT1_R   17
#define IO_OUT2_R   18
#define IO_OUT3_R   19
#define IO_OUT4_R   20
#define IO_OUT5_R   21
#define IO_OUT6_R   22
#define IO_OUT7_R   23
#define IO_OUT8_R   24
#define IO_OUT9_R   25
#define IO_OUT10_R  26
#define IO_OUT11_R  27
#define IO_OUT12_R  28
#define IO_OUT13_R  29
#define IO_OUT14_R  30
#define IO_OUT15_R  31
#define IO_OUT0_L   32
#define IO_OUT1_L   33
#define IO_OUT2_L   34
#define IO_OUT3_L   35
#define IO_OUT4_L   36
#define IO_OUT5_L   37
#define IO_OUT6_L   38
#define IO_OUT7_L   39
#define IO_OUT8_L   40
#define IO_OUT9_L   41
#define IO_OUT10_L  42
#define IO_OUT11_L  43
#define IO_OUT12_L  44
#define IO_OUT13_L  45
#define IO_OUT14_L  46
#define IO_OUT15_L  47
#define IO_OUT0_H   48
#define IO_OUT1_H   49
#define IO_OUT2_H   50
#define IO_OUT3_H   51
#define IO_OUT4_H   52
#define IO_OUT5_H   53
#define IO_OUT6_H   54
#define IO_OUT7_H   55
#define IO_OUT8_H   56
#define IO_OUT9_H   57
#define IO_OUT10_H  58
#define IO_OUT11_H  59
#define IO_OUT12_H  60
#define IO_OUT13_H  61
#define IO_OUT14_H  62
#define IO_OUT15_H  63


//Construct SET instruction
//Rd = Rn Op Imm
//Rd: Destination register selected from PTM register macros. Example values: PTM_X, PTM_FIFO, PTM_EVTS.
//Rn: Source register selected from PTM register macros. Example values: PTM_X, PTM_FIFO.
//Op: Operation selected from PTM operation macros. Example values: OP_ADD, OP_REV8.
//Imm: 16-bit immediate value for operation. Considered as a signed value for add operation. Example values: 0x200, -30.
#define PTM_SET(Rd,Rn,Op,Imm) \
  (((((uint32_t)( Op)&0x0000000f)<<24) | \
    (((uint32_t)( Rd)&0x0000000f)<<20) | \
    (((uint32_t)( Rn)&0x0000000f)<<16) | \
     ((uint32_t)(Imm)&0x0000ffff) \
   ) | 0x30000000  \
  )

//Construct JMP instruction
//if Rm and Rn match Cond
//  PC = Tgt, delay Dly
//else
//  PC = PC + 1
//Cond: Condition selected from PTM condition macros. Example values: JMP_EQ, JMP_GT.
//Rm: Register selected from PTM register macros. Example values: PTM_X, PTM_YL, PTM_ALL1.
//Rn: Register selected from PTM register macros, or certain IO input from PTM IO input macros, or certain event from PTM event macros. Example values: PTM_X, PTM_IN5, PTM_EVT15.
//Dly: 6-bit delay cycles.
//Tgt: Target selected from PTM target macros, or 8-bit signed offset from -127 to 127. Example values: JMP_TCM18, JMP_X, -5, 127.
#define PTM_JMP(Tgt,Rn,Cond,Rm,Dly) \
  (((((uint32_t)(Cond)&0x00000007)<<24) | \
    (((uint32_t)(  Rm)&0x0000000f)<<20) | \
    (((uint32_t)(  Rn)&0x0000000f)<<16) | \
    (((uint32_t)( Dly)&0x0000003f)<<10) | \
     ((uint32_t)( Tgt)&0x000003ff) \
   ) | 0x20000000  \
  )

//Construct WAIT instruction
//wait until Src == Pol, then delay Dly
//Src: Source selected from PTM wait macros. Example values: WAIT_TRIG66, WAIT_PA35, WAIT_EVT10.
//Pol: Polarity selected between 0 and 1.
//Dly: 16-bit delay cycles.
#define PTM_WAIT(Src,Pol,Dly) \
  (((((uint32_t)(Pol)&0x00000001)<<24) | \
    (((uint32_t)(Src)&0x000000ff)<<16) | \
     ((uint32_t)(Dly)&0x0000ffff) \
   ) | 0x00000000  \
  )

//Construct LDM instruction
//Rd = [Addr]
//Rd: Register selected between PTM_X and PTM_Y.
//Addr: 32-bit address
#define PTM_LDM(Rd,Addr) \
  (((((uint32_t)(Rd)&0x00000001)<<29) | \
     (((uint32_t)(Addr)>>2)&0x1fffffff) \
   ) | 0x80000000  \
  )

//Construct STM instruction
//[Addr] = Rn
//Rn: Register selected between PTM_X and PTM_Y.
//Addr: 32-bit address
#define PTM_STM(Rn,Addr) \
  (((((uint32_t)(Rn)&0x00000001)<<29) | \
     (((uint32_t)(Addr)>>2)&0x1fffffff) \
   ) | 0xc0000000  \
  )

//Construct LDR instruction
//Rd = [Rn + Off]
//Rd: Destination register selected from PTM register macros. Example values: PTM_X, PTM_YH.
//Rn: Source register selected between PTM_X and PTM_Y.
//Off: 16-bit signed address offset
#define PTM_LDR(Rd,Rn,Off) \
  (((((uint32_t)( Rd)&0x00000007)<<20) | \
    (((uint32_t)( Rn)&0x00000001)<<16) | \
     ((uint32_t)(Off)&0x0000ffff) \
   ) | 0x66000000  \
  )

//Construct STR instruction
//[Rd + Off] = Rn
//Rn: Source register selected from PTM register macros. Example values: PTM_X, PTM_YH.
//Rd: Destination register selected between PTM_X and PTM_Y.
//Off: 16-bit signed address offset
#define PTM_STR(Rn,Rd,Off) \
  (((((uint32_t)( Rn)&0x00000007)<<20) | \
    (((uint32_t)( Rd)&0x00000001)<<16) | \
     ((uint32_t)(Off)&0x0000ffff) \
   ) | 0x76000000  \
  )

//Construct IO instruction
//Iocnt: Output count selected from PTM output macros, or Input count selected from PTM input macros. Example values: IO_PUSH5, IO_PULL16.
//Om: First single output selected from PTM single output macros. Example values: IO_NOP, IO_OUT10_L, IO_OUT5_R.
//On: Second single output selected from PTM single output macros. Example values: IO_NOP, IO_OUT10_L, IO_OUT5_R.
//Dly: 6-bit delay cycles.
#define PTM_IO(Iocnt,Om,On,Dly) \
  (((((uint32_t)(   Om)&0x0000003f)<<22) | \
    (((uint32_t)(   On)&0x0000003f)<<16) | \
    (((uint32_t)(  Dly)&0x0000003f)<<10) | \
     ((uint32_t)(Iocnt)&0x0000003f) \
   ) | 0x40000000  \
  )


//TCM base address of PTM
#define PTM1_CORE0_TCM (uint32_t *)(PTM1_BASE + 0x200)
#define PTM1_CORE1_TCM (uint32_t *)(PTM1_BASE + 0x240)
#define PTM1_CORE2_TCM (uint32_t *)(PTM1_BASE + 0x280)
#define PTM1_CORE3_TCM (uint32_t *)(PTM1_BASE + 0x2c0)
#define PTM1_CORE4_TCM (uint32_t *)(PTM1_BASE + 0x300)
#define PTM1_CORE5_TCM (uint32_t *)(PTM1_BASE + 0x340)
#define PTM1_CORE6_TCM (uint32_t *)(PTM1_BASE + 0x380)
#define PTM1_CORE7_TCM (uint32_t *)(PTM1_BASE + 0x3c0)

#endif /* PTM1_BASE */



#ifdef __cplusplus
}
#endif


#endif /* __PTM_INS_H__ */