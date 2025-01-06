/**
  ******************************************************************************
  * @file   metrics_id_middleware.h
  * @author Sifli software development team
  * @brief Metrics Collector source.
  *
  ******************************************************************************
*/
/*
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

#ifndef _METRICS_ID_MIDDLEWARE_H_
#define _METRICS_ID_MIDDLEWARE_H_

#include "metrics_id.h"


#ifdef __cplusplus
extern "C" {
#endif

#define METRICS_MW_CPU_USAGE          (METRICS_MIDDLEWARE_ID_START)
#define METRICS_MW_PM_STAT            (METRICS_MIDDLEWARE_ID_START + 1)
#define METRICS_MW_PM_DEBUG           (METRICS_MIDDLEWARE_ID_START + 2)
#define METRICS_MW_BT_CONN            (METRICS_MIDDLEWARE_ID_START + 3)
#define METRICS_MW_BT_SCAN            (METRICS_MIDDLEWARE_ID_START + 4)
#define METRICS_MW_BT_ADV             (METRICS_MIDDLEWARE_ID_START + 5)
#define METRICS_MW_BT_RSSI            (METRICS_MIDDLEWARE_ID_START + 6)
#define METRICS_MW_GUI_PM_STAT        (METRICS_MIDDLEWARE_ID_START + 7)
#define METRICS_MW_MOTOR_STAT         (METRICS_MIDDLEWARE_ID_START + 8)
#define METRICS_MW_BAT_VOL_TEMP_STAT         (METRICS_MIDDLEWARE_ID_START + 9)
#define METRICS_MW_GS_STAT         (METRICS_MIDDLEWARE_ID_START + 10)
#define METRICS_MW_HR_STAT         (METRICS_MIDDLEWARE_ID_START + 11)
#define METRICS_MW_POWER_ON_STAT         (METRICS_MIDDLEWARE_ID_START + 12)
#define METRICS_MW_SHUTDOWN_STAT         (METRICS_MIDDLEWARE_ID_START + 13)




#ifdef __cplusplus
}
#endif


#endif /* _METRICS_ID_MIDDLEWARE_H_ */
