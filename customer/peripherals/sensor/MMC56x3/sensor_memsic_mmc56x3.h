/*
 * SPDX-FileCopyrightText: 2019-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "board.h"
#include "sensor.h"

#include "mmc56x3.h"

int rt_hw_mmc56x3_init(const char *name, struct rt_sensor_config *cfg);