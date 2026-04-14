/*
 * SPDX-FileCopyrightText: 2025-2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __DRV_SWITCH_MPI2_SDIO_H_
#define __DRV_SWITCH_MPI2_SDIO_H_

/**
 * @brief  Lock SDIO/MPI2 switch and switch to SDIO1 mode if required.
 *
 * This function takes the switch mutex and, when the current pinmux
 * is not in SDIO1 mode, reconfigures GPIO and pinmux to SDIO1 and
 * enables the SDIO interrupt.
 */
void rt_switch_sdio_lock(void);

/**
 * @brief  Unlock SDIO/MPI2 switch after SDIO1 operations.
 *
 * This function releases the switch mutex acquired by
 * rt_switch_sdio_lock. It does not change the current pinmux state.
 */
void rt_switch_sdio_unlock(void);

/**
 * @brief  Lock SDIO/MPI2 switch and switch to MPI2 mode if required.
 *
 * This function takes the switch mutex and, when the current pinmux
 * is not in MPI2 mode, reconfigures GPIO and pinmux to MPI2 and
 * disables the SDIO interrupt.
 */
void rt_switch_mpi2_lock(void);

/**
 * @brief  Unlock SDIO/MPI2 switch after MPI2 operations.
 *
 * This function releases the switch mutex acquired by
 * rt_switch_mpi2_lock. It does not change the current pinmux state.
 */
void rt_switch_mpi2_unlock(void);

#endif