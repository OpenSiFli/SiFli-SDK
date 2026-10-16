/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APP_CLOCK_EMOJI_H
#define APP_CLOCK_EMOJI_H

#include "lvgl.h"

/**
 * @brief Create one emoji fallback font per font size used by notify_msgs and
 *        attach it as fallback of the corresponding freetype font. Emojis use a
 *        single fixed-size image scaled to the font line height on demand.
 *        Idempotent, safe to call repeatedly.
 */
void app_clock_emoji_font_init(void);

#endif /* APP_CLOCK_EMOJI_H */
