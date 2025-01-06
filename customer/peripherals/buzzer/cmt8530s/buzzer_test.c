/**
  ******************************************************************************
  * @file   buzzer_test.c
  * @author Sifli software development team
  ******************************************************************************
*/
/**
 * @attention
 * Copyright (c) 2019 - 2022,  Sifli Technology
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

#include "cmt8530s.h"
#include "buzzer_test.h"

// 定义低音音名（数值单位：Hz）
#define L1 262
#define L2 294
#define L3 330
#define L4 349
#define L5 392
#define L6 440
#define L7 494

// 定义中音音名
#define M1 523
#define M2 587
#define M3 659
#define M4 698
#define M5 784
#define M6 880
#define M7 988

// 定义高音音名
#define H1 1047
#define H2 1175
#define H3 1319
#define H4 1397
#define H5 1568
#define H6 1760
#define H7 1976

// 定义时值单位，决定演奏速度（数值单位：ms）
#define TT 2000


// 定义乐曲：刘德华《恭喜发财》
const tNote MyScore1[] =
{
    {L3, TT / 8}, {M6, TT / 4}, {M5, TT / 4}, {M6, TT / 4}, {M5, TT / 8}, {M3, TT / 8}, {M3, TT / 4}, {L3, TT / 8}, {M6, TT / 4}, {M5, TT / 4}, {M6, TT / 4}, {M5, TT / 8}, {M6, TT / 8}, {M6, TT / 2}, {M3, TT / 8}, {M2, TT / 8 + TT / 16}, {M3, TT / 16}, {M2, TT / 8},
    {M1, TT / 8}, {L6, TT / 4}, {M3, TT / 8}, {M2, TT / 8 + TT / 16}, {M3, TT / 16}, {M2, TT / 8}, {M1, TT / 8}, {M2, TT / 4}, {M2, TT / 8 + TT / 16}, {M1, TT / 8}, {M1, TT / 4}, {M2, TT / 4}, {M3, TT / 4}, {M5, TT / 4}, {M6, TT}, {M6, TT / 8 + TT / 16}, {M5, TT / 16},
    {M3, TT / 8}, {M5, TT / 8}, {M6, TT / 4}, //恭喜你发财。。。礼多人不怪

    {L3, TT / 8}, {L6, TT / 4}, {L6, TT / 8}, {L5, TT / 8}, {L6, TT / 8}, {L3, TT / 8}, {L3, TT / 8}, {L5, TT / 8}, {L6, TT / 8}, {M1, TT / 8}, {L6, TT / 8}, {L5, TT / 8}, {L6, TT / 4}, {L3, TT / 8}, {L5, TT / 8}, {M1, TT / 4}, {M1, TT / 8}, {M1, TT / 8}, {M2, TT / 8},
    {M2, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 2}, {L3, TT / 8}, {M2, TT / 4}, {M2, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {L6, TT / 8}, {L6, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {M1, TT / 8}, {L6, TT / 8},
    {M1, TT / 8}, {M3, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 8}, {M3, TT / 4}, {L5, TT / 4}, {L6, TT / 2}, //我祝满天下的女孩。。。智商充满你脑袋

    {L3, TT / 8}, {L6, TT / 4}, {L6, TT / 8}, {L5, TT / 8}, {L6, TT / 8}, {L3, TT / 8}, {L3, TT / 8}, {L5, TT / 8}, {L6, TT / 8}, {M1, TT / 8}, {L6, TT / 8}, {L5, TT / 8}, {L6, TT / 4}, {L3, TT / 8}, {L5, TT / 8}, {M1, TT / 4}, {M1, TT / 8}, {M1, TT / 8}, {M2, TT / 8},
    {M2, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 2}, {L3, TT / 8}, {M2, TT / 4}, {M2, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {L6, TT / 8}, {L6, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {M1, TT / 8}, {L6, TT / 8},
    {M1, TT / 8}, {M3, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 8}, {M3, TT / 4}, {L5, TT / 4}, {L6, TT / 2}, //我祝满天下的女孩。。。智商充满你脑袋

    {L5, TT / 4}, {M1, TT / 2 + TT / 4}, {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 2 + TT / 4}, {M3, TT / 8}, {M5, TT / 8}, {M5, TT / 4 + TT / 8}, {M3, TT / 8}, {M2, TT / 4}, {M1, TT / 4}, {M2, TT / 2}, {M2, TT / 4 + TT / 8}, {L6, TT / 8}, {M2, TT / 4}, {M3, TT / 4},
    {M4, TT / 8 + TT / 16}, {M5, TT / 16}, {M4, TT / 8}, {M3, TT / 8}, {M2, TT / 2}, {M5, TT / 8}, {M5, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 4}, {L5, TT / 8}, {L6, TT / 2}, //大摇大摆。。。要喊得够豪迈

    {L3, TT / 8}, {M6, TT / 4}, {M5, TT / 4}, {M6, TT / 4}, {M5, TT / 8}, {M3, TT / 8}, {M3, TT / 4}, {L3, TT / 8}, {M6, TT / 4}, {M5, TT / 4}, {M6, TT / 4}, {M5, TT / 8}, {M6, TT / 8}, {M6, TT / 2}, {M3, TT / 8}, {M2, TT / 8 + TT / 16}, {M3, TT / 16}, {M2, TT / 8},
    {M1, TT / 8}, {L6, TT / 4}, {M3, TT / 8}, {M2, TT / 8 + TT / 16}, {M3, TT / 16}, {M2, TT / 8}, {M1, TT / 8}, {M2, TT / 4}, {M2, TT / 8 + TT / 16}, {M1, TT / 8}, {M1, TT / 4}, {M2, TT / 4}, {M3, TT / 4}, {M5, TT / 4}, {M6, TT}, {M6, TT / 8 + TT / 16}, {M5, TT / 16},
    {M3, TT / 8}, {M5, TT / 8}, {M6, TT / 4}, //恭喜你发财。。。礼多人不怪

    {L3, TT / 8}, {L6, TT / 4}, {L6, TT / 8}, {L5, TT / 8}, {L6, TT / 8}, {L3, TT / 8}, {L3, TT / 8}, {L5, TT / 8}, {L6, TT / 8}, {M1, TT / 8}, {L6, TT / 8}, {L5, TT / 8}, {L6, TT / 4}, {L3, TT / 8}, {L5, TT / 8}, {M1, TT / 4}, {M1, TT / 8}, {M1, TT / 8}, {M2, TT / 8},
    {M2, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 2}, {L3, TT / 8}, {M2, TT / 4}, {M2, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {L6, TT / 8}, {L6, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {M1, TT / 8}, {L6, TT / 8},
    {M1, TT / 8}, {M3, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 8}, {M3, TT / 4}, {L5, TT / 4}, {L6, TT / 2}, //我祝满天下的女孩。。。智商充满你脑袋

    {L5, TT / 4}, {M1, TT / 2 + TT / 4}, {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 2 + TT / 4}, {M3, TT / 8}, {M5, TT / 8}, {M5, TT / 4 + TT / 8}, {M3, TT / 8}, {M2, TT / 4}, {M1, TT / 4}, {M2, TT / 2}, {M2, TT / 4 + TT / 8}, {L6, TT / 8}, {M2, TT / 4}, {M3, TT / 4},
    {M4, TT / 8 + TT / 16}, {M5, TT / 16}, {M4, TT / 8}, {M3, TT / 8}, {M2, TT / 2}, {M5, TT / 8}, {M5, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 4}, {L5, TT / 8}, {L6, TT / 2}, //大摇大摆。。。要喊得够豪迈

    {L3, TT / 8}, {M6, TT / 4}, {M5, TT / 4}, {M6, TT / 4}, {M5, TT / 8}, {M3, TT / 8}, {M3, TT / 4}, {L3, TT / 8}, {M6, TT / 4}, {M5, TT / 4}, {M6, TT / 4}, {M5, TT / 8}, {M6, TT / 8}, {M6, TT / 2}, {M3, TT / 8}, {M2, TT / 8 + TT / 16}, {M3, TT / 16}, {M2, TT / 8},
    {M1, TT / 8}, {L6, TT / 4}, {M3, TT / 8}, {M2, TT / 8 + TT / 16}, {M3, TT / 16}, {M2, TT / 8}, {M1, TT / 8}, {M2, TT / 4}, {M2, TT / 8 + TT / 16}, {M1, TT / 8}, {M1, TT / 4}, {M2, TT / 4}, {M3, TT / 4}, {M5, TT / 4}, {M6, TT}, {M6, TT / 8 + TT / 16}, {M5, TT / 16},
    {M3, TT / 8}, {M5, TT / 8}, {M6, TT / 4}, //恭喜你发财。。。礼多人不怪

    {L3, TT / 8}, {M6, TT / 4}, {M5, TT / 4}, {M6, TT / 4}, {M5, TT / 8}, {M3, TT / 8}, {M3, TT / 4}, {L3, TT / 8}, {M6, TT / 4}, {M5, TT / 4}, {M6, TT / 4}, {M5, TT / 8}, {M6, TT / 8}, {M6, TT / 2}, {M3, TT / 8}, {M2, TT / 8 + TT / 16}, {M3, TT / 16}, {M2, TT / 8},
    {M1, TT / 8}, {L6, TT / 4}, {M3, TT / 8}, {M2, TT / 8 + TT / 16}, {M3, TT / 16}, {M2, TT / 8}, {M1, TT / 8}, {M2, TT / 4}, {M2, TT / 8 + TT / 16}, {M1, TT / 8}, {M1, TT / 4}, {M2, TT / 4}, {M3, TT / 4}, {M5, TT / 4}, {M6, TT}, {M6, TT / 8 + TT / 16}, {M5, TT / 16},
    {M3, TT / 8}, {M5, TT / 8}, {M6, TT / 4}, //恭喜你发财。。。礼多人不怪
    {0, 0},
};




// 定义乐曲：《荷塘月色》
const tNote MyScore2[] =
{
    {M1, TT / 8}, {M1, TT / 4}, {L6, TT / 8}, {L5, TT / 4}, {L6, TT / 4}, {M1, TT / 4}, {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 2}, {M2, TT / 8}, {M2, TT / 4}, {M1, TT / 8}, {M2, TT / 4}, {M2, TT / 8}, {M5, TT / 8}, {M5, TT / 8}, {M3, TT / 8},
    {M3, TT / 8}, {M2, TT / 8}, {M3, TT / 2}, {M1, TT / 8}, {M1, TT / 4}, {L6, TT / 8}, {L5, TT / 4}, {M5, TT / 4}, {M3, TT / 8}, {M2, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 2}, {M2, TT / 8}, {M2, TT / 4}, //苍茫的天涯是我的爱。。。最呀最摇
    {M1, TT / 8}, {M2, TT / 8}, {M2, TT / 4}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 8}, {L6, TT / 8}, {M2, TT / 8}, {M1, TT / 2}, //剪一段时光。。。美丽的琴音就落在我身旁
    {M1, TT / 8}, {M1, TT / 4}, {L6, TT / 8}, {L5, TT / 4}, {L6, TT / 4}, {M1, TT / 8}, {M1, TT / 4}, {M2, TT / 8}, {M3, TT / 2}, {M2, TT / 8}, {M2, TT / 4}, {M1, TT / 8}, {M2, TT / 4}, {M2, TT / 8}, {M5, TT / 8}, {M5, TT / 8}, {M3, TT / 8},
    {M3, TT / 8}, {M2, TT / 8}, {M3, TT / 2}, {M1, TT / 8}, {M1, TT / 8}, {M1, TT / 8}, {L6, TT / 8}, {L5, TT / 4}, {M5, TT / 4}, {M3, TT / 8}, {M2, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 2}, {M2, TT / 8}, {M2, TT / 4}, //苍茫的天涯是我的爱。。。最呀最摇
    {M1, TT / 8}, {M2, TT / 8}, {M2, TT / 4}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 8}, {L6, TT / 8}, {M2, TT / 8}, {M1, TT / 2}, //萤火虫。。。谁采下那一朵昨夜的忧伤
    {M3, TT / 8}, {M5, TT / 4}, {M5, TT / 8}, {M5, TT / 4}, {M5, TT / 4}, {M6, TT / 8}, {M5, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 2}, {M6, TT / 8}, {H1, TT / 8}, {M6, TT / 8}, {M5, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 8},
    {L6, TT / 8}, {M2, TT / 4}, {M2, TT / 8}, {M3, TT / 8}, {M3, TT / 8}, {M2, TT / 4 + TT / 8}, {M3, TT / 8}, {M5, TT / 4}, {M5, TT / 8}, {M5, TT / 4}, {M5, TT / 4}, {M6, TT / 8}, {M5, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, //苍茫的天涯是我的爱。。。最呀最摇
    {M1, TT / 2}, {L6, TT / 8}, {M1, TT / 8}, {L6, TT / 8}, {L5, TT / 8}, {M2, TT / 4}, {M3, TT / 4}, {M1, TT / 2 + TT / 4}, //我像只鱼儿。。。等你宛在水中央
    {M1, TT / 4 + TT / 8}, {M5, TT / 8}, {M1, TT / 8}, {M5, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {M3, TT}, {M1, TT / 4 + TT / 8}, {M5, TT / 8}, {M1, TT / 8}, {M5, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {M2, TT}, {M1, TT / 4 + TT / 8}, {M5, TT / 8}, {M1, TT / 8}, {M5, TT / 8},
    {M2, TT / 8}, {M1, TT / 8}, {L6, TT / 2}, {L6, TT / 8}, {L5, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {M1, TT / 4 + TT / 8}, {M5, TT / 8}, {M1, TT / 8}, {M5, TT / 8}, {M1, TT / 8}, {L6, TT / 8}, {M1, TT},
    {M1, TT / 8}, {M1, TT / 4}, {L6, TT / 8}, {L5, TT / 4}, {L6, TT / 4}, {M1, TT / 8}, {M1, TT / 4}, {M2, TT / 8}, {M3, TT / 2}, {M2, TT / 8}, {M2, TT / 4}, {M1, TT / 8}, {M2, TT / 4}, {M2, TT / 8}, {M5, TT / 8}, {M5, TT / 8}, {M3, TT / 8},
    {M3, TT / 8}, {M2, TT / 8}, {M3, TT / 2}, {M1, TT / 8}, {M1, TT / 8}, {M1, TT / 8}, {L6, TT / 8}, {L5, TT / 4}, {M5, TT / 4}, {M3, TT / 8}, {M2, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 2}, {M2, TT / 8}, {M2, TT / 4}, //苍茫的天涯是我的爱。。。最呀最摇
    {M1, TT / 8}, {M2, TT / 8}, {M2, TT / 4}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 8}, {L6, TT / 8}, {M2, TT / 8}, {M1, TT / 2}, //萤火虫。。。谁采下那一朵昨夜的忧伤
    {M3, TT / 8}, {M5, TT / 4}, {M5, TT / 8}, {M5, TT / 4}, {M5, TT / 4}, {M6, TT / 8}, {M5, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 2}, {M6, TT / 8}, {H1, TT / 8}, {M6, TT / 8}, {M5, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 8},
    {L6, TT / 8}, {M2, TT / 4}, {M2, TT / 8}, {M3, TT / 8}, {M3, TT / 8}, {M2, TT / 4 + TT / 8}, {M3, TT / 8}, {M5, TT / 4}, {M5, TT / 8}, {M5, TT / 4}, {M5, TT / 4}, {M6, TT / 8}, {M5, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, //苍茫的天涯是我的爱。。。最呀最摇
    {M1, TT / 2}, {L6, TT / 8}, {M1, TT / 8}, {L6, TT / 8}, {L5, TT / 8}, {M2, TT / 4}, {M3, TT / 4}, {M1, TT / 2 + TT / 4}, //我像只鱼儿。。。等你宛在水中央
    {M1, TT / 4 + TT / 8}, {M5, TT / 8}, {M1, TT / 8}, {M5, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {M3, TT}, {M1, TT / 4 + TT / 8}, {M5, TT / 8}, {M1, TT / 8}, {M5, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {M2, TT}, {M1, TT / 4 + TT / 8}, {M5, TT / 8}, {M1, TT / 8}, {M5, TT / 8},
    {M2, TT / 8}, {M1, TT / 8}, {L6, TT / 2}, {L6, TT / 8}, {L5, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {M1, TT / 4 + TT / 8}, {M5, TT / 8}, {M1, TT / 8}, {M5, TT / 8}, {M1, TT / 8}, {L6, TT / 8}, {M1, TT},

    {0, 0},
};



// 定义乐曲：《最炫民族风》
const tNote MyScore3[] =
{
    {L6, TT / 4}, {L3, TT / 8}, {L5, TT / 8}, {L6, TT / 4}, {L6, TT / 8}, {M1, TT / 8}, {M1, TT / 4}, {M2, TT / 8}, {M1, TT / 8}, {L6, TT / 2}, {M1, TT / 4}, {M1, TT / 8}, {L5, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 8}, {M5, TT / 8},
    {M5, TT / 8}, {M3, TT / 8}, {M2, TT / 4}, {M3, TT / 2}, {M6, TT / 8}, {M6, TT / 8}, {M6, TT / 8}, {M5, TT / 8}, {M3, TT / 8}, {M3, TT / 4}, {M1, TT / 8}, {L6, TT / 8}, {L6, TT / 8}, {L6, TT / 8}, {M3, TT / 8}, //苍茫的天涯是我的爱。。。最呀最摇
    {M2, TT / 2}, {M3, TT / 8}, {M3, TT / 8}, {M5, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 8}, {L6, TT / 4}, {L5, TT / 4}, {L6, TT / 2}, //摆，什么样的歌声才是最开怀
    {L6, TT / 4}, {L3, TT / 8}, {L5, TT / 8}, {L6, TT / 4}, {L6, TT / 8}, {M1, TT / 8}, {M1, TT / 4}, {M2, TT / 8}, {M1, TT / 8}, {L6, TT / 2}, {M1, TT / 4}, {M1, TT / 8}, {L5, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 8}, {M5, TT / 8},
    {M5, TT / 8}, {M3, TT / 8}, {M2, TT / 4}, {M3, TT / 2}, {M6, TT / 8}, {M6, TT / 8}, {M6, TT / 8}, {M5, TT / 8}, {M3, TT / 8}, {M3, TT / 4}, {M1, TT / 8}, {L6, TT / 8}, {L6, TT / 8}, {L6, TT / 8}, {M3, TT / 8}, //苍茫的天涯是我的爱。。。最呀最摇
    {M2, TT / 2}, {M3, TT / 8}, {M3, TT / 8}, {M5, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 8}, {L6, TT / 4}, {L5, TT / 4}, {L6, TT / 2}, //摆，什么样的歌声才是最开怀
    {M3, TT / 8}, {M3, TT / 8}, {M5, TT / 8}, {M3, TT / 8}, {M3, TT / 8}, {M5, TT / 8}, {M5, TT / 8}, {M6, TT / 8}, {H1, TT / 8}, {M6, TT / 8}, {M5, TT / 4}, {M6, TT / 2}, {L6, TT / 4}, {L6, TT / 8},
    {L5, TT / 8}, {L6, TT / 4}, {M1, TT / 4}, {M2, TT / 8}, {M3, TT / 16}, {M2, TT / 16}, {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 2}, {L6, TT / 8}, {M6, TT / 8}, {M6, TT / 8}, {M5, TT / 8}, {M2, TT / 8}, {M3, TT / 16}, {M2, TT / 16},
    {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 2}, //留下来
    {M1, TT / 8}, {L6, TT / 8}, {L6, TT / 8}, {M1, TT / 8}, {M2, TT / 4}, {L5, TT / 8}, {L5, TT / 8}, {M3, TT / 8}, {M5, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 2}, {L6, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 8},
    {L5, TT / 8}, {L3, TT / 8}, {L6, TT / 2}, {L6, TT / 4}, {L6, TT / 8}, {L5, TT / 8}, {L6, TT / 4}, {M1, TT / 4}, {M2, TT / 8}, {M3, TT / 16}, {M2, TT / 16}, {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 2}, {L6, TT / 8}, {M6, TT / 8},
    {M6, TT / 8}, {M5, TT / 8}, {M2, TT / 8}, {M3, TT / 16}, {M2, TT / 16},
    {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 2}, //留下来
    {M1, TT / 8}, {L6, TT / 8}, {L6, TT / 8}, {M1, TT / 8}, {M2, TT / 4}, {L5, TT / 8}, {L5, TT / 8}, {M3, TT / 8}, {M5, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 4 + TT / 8}, {M1, TT / 8}, {L6, TT / 8},
    {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 8}, {M5, TT / 8}, {M3, TT / 8}, {M3, TT / 8}, {M5, TT / 8}, {M6, TT / 2}, {M6, TT / 2}, {L6, TT / 4}, {L6, TT / 8}, {L5, TT / 8}, {L6, TT / 4},
    {L6, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 16}, {M2, TT / 16}, {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 2},
    {M6, TT / 8}, {M5, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M5, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 8}, {M1, TT / 2}, //登上天外云霄的舞台
    {L6, TT / 4}, {L6, TT / 8}, {L5, TT / 8}, {L6, TT / 4}, {M1, TT / 4}, {M2, TT / 8}, {M3, TT / 16}, {M2, TT / 16}, {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 2}, {L6, TT / 8}, {M6, TT / 8}, {M6, TT / 8}, {M5, TT / 8}, {M2, TT / 8}, {M3, TT / 16}, {M2, TT / 16},
    {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 2}, //留下来
    {M1, TT / 8}, {L6, TT / 8}, {L6, TT / 8}, {M1, TT / 8}, {M2, TT / 4}, {L5, TT / 8}, {L5, TT / 8}, {M3, TT / 8}, {M5, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 2}, {L6, TT / 8}, {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 8},
    {L5, TT / 8}, {L3, TT / 8}, {L6, TT / 2}, {L6, TT / 4}, {L6, TT / 8}, {L5, TT / 8}, {L6, TT / 4}, {M1, TT / 4}, {M2, TT / 8}, {M3, TT / 16}, {M2, TT / 16}, {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 2}, {L6, TT / 8}, {M6, TT / 8},
    {M6, TT / 8}, {M5, TT / 8}, {M2, TT / 8}, {M3, TT / 16}, {M2, TT / 16},
    {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 2}, //留下来
    {M1, TT / 8}, {L6, TT / 8}, {L6, TT / 8}, {M1, TT / 8}, {M2, TT / 4}, {L5, TT / 8}, {L5, TT / 8}, {M3, TT / 8}, {M5, TT / 8}, {M3, TT / 8}, {M2, TT / 8}, {M1, TT / 4 + TT / 8}, {M1, TT / 8}, {L6, TT / 8},
    {M1, TT / 8}, {M2, TT / 8}, {M3, TT / 8}, {M5, TT / 8}, {M3, TT / 8}, {M3, TT / 8}, {M5, TT / 8}, {M6, TT / 2}, {M6, TT / 2},
    {0, 0},
};


// 演奏乐曲
void musicPlay(tNote *MyScore)
{
    int i = 0;
    while (1)
    {
        if (MyScore[i].mTime == 0) break;
        cmt8530s_set_freq(MyScore[i].mName);
        cmt8530s_open();

        rt_thread_delay(MyScore[i].mTime);
        i++;
        cmt8530s_close(); // 蜂鸣器静音
        rt_thread_delay(10);// 10 ms
    }
}



