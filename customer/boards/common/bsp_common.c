/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rtconfig.h"

#ifndef BSP_USING_PC_SIMULATOR
#include "register.h"
#include "drv_io.h"

HAL_RAM_RET_CODE_SECT(HAL_MspInit, __weak void HAL_MspInit(void))
{
    // HAL_sw_breakpoint();        /*For debugging purpose*/
    BSP_IO_Init();
}

__weak void BSP_SD2_PowerUp(void)
{
    // Default implementation - can be overridden by board-specific implementation
}

__weak void BSP_SD_PowerUp(void)
{
    // Default implementation - can be overridden by board-specific implementation
}


__weak void BSP_Flash4_PowerUp(void)
{
#if defined(SF32LB58X) && defined(BSP_ENABLE_MPI4)
    HAL_PIN_Set(PAD_PA39, MPI4_CLK, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA30, MPI4_CS, PIN_NOPULL, 1);
    HAL_PIN_Set(PAD_PA40, MPI4_DIO0, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA37, MPI4_DIO1, PIN_PULLDOWN, 1);
    HAL_PIN_Set(PAD_PA36, MPI4_DIO2, PIN_PULLUP, 1);
    HAL_PIN_Set(PAD_PA38, MPI4_DIO3, PIN_PULLUP, 1);
#endif /* SF32LB58X && BSP_ENABLE_MPI4 */
}

__weak void BSP_Flash4_PowerDown(void)
{

}

#if !defined(__CLANG_ARM) && defined (__GNUC__) && defined(BSP_USING_NO_OS)

#include <sys/stat.h>
#include <errno.h>

__weak int _close(int fd)
{
    (void)fd;
    return -1;
}

__weak int _lseek(int fd, int offset, int whence)
{
    (void)fd;
    (void)offset;
    (void)whence;
    return -1;
}

__weak int _read(int fd, char *buf, int len)
{
    (void)fd;
    (void)buf;
    (void)len;
    return 0;
}

__weak int _write(int fd, char *buf, int len)
{
    (void)fd;
    (void)buf;
    (void)len;
    return len;
}

__weak int _fstat(int fd, struct stat *st)
{
    (void)fd;
    (void)st;
    return 0;
}

__weak int _isatty(int fd)
{
    (void)fd;
    return 1;
}

__weak int _open(const char *path, int flags, int mode)
{
    (void)path;
    (void)flags;
    (void)mode;
    return -1;
}

// void *_sbrk(int incr)
// {
//     (void)incr;
//     return (void *)-1;
// }

#endif /* !__CLANG_ARM && __GNUC__ && !RT_USING_LIBC */

#endif /* BSP_USING_PC_SIMULATOR */
