/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-05-07     aozima       the first version
 */

#ifndef __DRV_PWM_H_INCLUDE__
#define __DRV_PWM_H_INCLUDE__

#include <rtthread.h>
#include <rtdevice.h>

#define PWM_CMD_ENABLE                  (128 + 0)
#define PWM_CMD_DISABLE                 (128 + 1)
#define PWM_CMD_SET                     (128 + 2)
#define PWM_CMD_GET                     (128 + 3)
#define PWM_CMD_SET_PERIOD              (128 + 4)
#define PWM_CMD_BREAK_DEAD              (128 + 5)
#define PWM_CMD_SET_CLK_SOURECE         (128 + 6)
#define PWM_CMD_SET_COLOR               (128 + 7)

/*
This bit-field defines the frequency used to sample BRK input and the length of the digital filter applied to BRK.
The digital filter is made of an event counter in which N consecutive events are needed to validate a transition on the output:

0000: No filter, BRK acts asynchronously
0001: fSAMPLING=fpclk, N=2
0010: fSAMPLING=fpclk, N=4
0011: fSAMPLING=fpclk, N=8
0100: fSAMPLING=fpclk/2, N=6
0101: fSAMPLING=fpclk/2, N=8
0110: fSAMPLING=fpclk/4, N=6
0111: fSAMPLING=fpclk/4, N=8
1000: fSAMPLING=fpclk/8, N=6
1001: fSAMPLING=fpclk/8, N=8
1010: fSAMPLING=fpclk/16, N=5
1011: fSAMPLING=fpclk/16, N=6
1100: fSAMPLING=fpclk/16, N=8
1101: fSAMPLING=fpclk/32, N=5
1110: fSAMPLING=fpclk/32, N=6
1111: fSAMPLING=fpclk/32, N=8
*/

struct rt_pwm_break_dead
{
    rt_uint32_t dtg: 10;    /* Dead-time generator setup*/
    rt_uint32_t rsvd: 1;
    rt_uint32_t dptsc: 1;   /* Dead-time prescaler, 0: dead-time is Tpclk*(DTG+1) if DTG is not zero, 1: dead-time is Tpclk*(DTG+1)*16 if DTG is not zero */
    rt_uint32_t bke: 1;     /* Break enable*/
    rt_uint32_t bkp: 1;     /* Break polarity, 0: Active low, 1: Active high */
    rt_uint32_t aoe: 1;     /* Automatic output enable*/
    rt_uint32_t rsvd2: 1;
    rt_uint32_t bkf: 4;     /* Break filter*/
    rt_uint32_t bk2f: 4;    /* Break 2 filter*/
    rt_uint32_t bk2e: 1;    /* Break 2 enable*/
    rt_uint32_t bk2p: 1;    /* Break 2 polarity, 0: Active low, 1: Active high */
    rt_uint32_t rsvd3: 1;   /* Break Disarm，0: Break input BRK is armed， 1: Break input BRK is disarmed*/
    rt_uint32_t rsvd4: 1;   /* Break 2 Disarm，0: Break input BRK is armed， 1: Break input BRK is disarmed*/
    rt_uint32_t rsvd5: 1;   /* Break Bidirectional*/
    rt_uint32_t rsvd6: 1;  /* Break 2 Bidirectional*/
    rt_uint32_t ossi: 1;    /* Off-state selection for Idle mode, 0: when inactive,output disable, 1:enable */
    rt_uint32_t ossr: 1;    /* Off-state selection for Run mode , 0: when inactive,output disable, 1:enable*/
};

struct rt_pwm_configuration
{
    rt_uint16_t channel; /* 0-n */
    rt_uint8_t  is_comp; /* Is complementary*/
    rt_uint8_t  reserved;
    rt_uint32_t period;  /* unit:ns 1ns~4.29s:1Ghz~0.23hz */
    rt_uint32_t pulse;   /* unit:ns (pulse<=period) */
    rt_uint32_t break_dead;
    rt_uint32_t dead_time; /*uint:ns if pclk=120MH, dead_time:0~136000ns*/
};

struct rt_device_pwm;
struct rt_rgbled_configuration
{
    rt_uint16_t channel; /* 0-n */
    rt_uint8_t  is_comp; /* Is complementary*/
    rt_uint8_t  reserved;
    rt_uint32_t color_rgb;  /*rgb color*/
};

struct rt_pwm_ops
{
    rt_err_t (*control)(struct rt_device_pwm *device, int cmd, void *arg);
};

struct rt_device_pwm
{
    struct rt_device parent;
    const struct rt_pwm_ops *ops;
};

rt_err_t rt_device_pwm_register(struct rt_device_pwm *device, const char *name, const struct rt_pwm_ops *ops, const void *user_data);

rt_err_t rt_pwm_enable2(struct rt_device_pwm *device, int channel, uint8_t is_comp);
#define  rt_pwm_enable(device,channel) rt_pwm_enable2(device,channel,0)

rt_err_t rt_pwm_disable(struct rt_device_pwm *device, int channel);
rt_err_t rt_pwm_set(struct rt_device_pwm *device, int channel, rt_uint32_t period, rt_uint32_t pulse);
rt_err_t rt_pwm_set_brk_dead(struct rt_device_pwm *device, rt_uint32_t *bkd, rt_uint32_t dt_ns);

#endif /* __DRV_PWM_H_INCLUDE__ */
