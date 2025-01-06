/****************************************************************************
 *Name: aw_audio_common.c
 ****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include "aw_audio_common.h"
#include "aw882xx.h"

#define aw_magic_splice(interface)  aw882xx##_##interface
#if (aw_magic_splice(auto_match) == aw882xx_auto_match)
#endif


struct aw_debug_attr aw_debug_attr =
{
#ifdef AW_DEBUG
    .reg_show = aw_magic_splice(reg_show),
    .reg_store = aw_magic_splice(reg_store),
    .hw_reset = aw_magic_splice(hw_reset_by_index),
    .soft_reset = aw_magic_splice(soft_reset),
#endif
};

struct aw_iface_cfg aw_iface_cfg =
{
    .init = aw_magic_splice(smartpa_init),
    .deinit = aw_magic_splice(smartpa_deinit),
    .set_profile_byname = aw_magic_splice(set_profile_byname),
    .ctrl_state = aw_magic_splice(ctrl_state),
#ifdef AW_FADE
    .set_fade_time = aw_magic_splice(set_fade_time),
    .get_fade_time = aw_magic_splice(get_fade_time),
    .set_fade_step = aw_magic_splice(set_fade_step),
    .get_fade_step = aw_magic_splice(get_fade_step),
#endif
#ifdef AW_VOLUME
    .set_volume = aw_magic_splice(set_volume),
    .get_volume = aw_magic_splice(get_volume),
#endif
#ifdef AW_IRQ
    .get_hw_irq_status = aw_magic_splice(get_hw_irq_status),
    .irq_handler = aw_magic_splice(irq_handler),
    .irq_trigger = aw_magic_splice(irq_trigger),
#endif
#ifdef AW_MONITOR
    .monitor_work = aw_magic_splice(monitor_work),
    .monitor_set_status = aw_magic_splice(monitor_set_status),
#endif
    .set_fs = aw_magic_splice(set_fs),
};

struct aw_iface_cfg *aw88xx_hal_iface_fops = &aw_iface_cfg;
struct aw_debug_attr *aw88xx_hal_debug_attr = &aw_debug_attr;
