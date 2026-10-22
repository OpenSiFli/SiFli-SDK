/*
 * SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _DEMO_MULTLIST_H
#define _DEMO_MULTLIST_H

#define DEMO_MULTLIST_MAIN_ID       "Main"
#define DEMO_MULTLIST_ID            "multlist"
#define DEMO_MULTLIST_PAGE_ID       "multlist_page"
#define DEMO_MULTLIST_LIST_ID       "multlist_list"
#define DEMO_MULTLIST_ANIM_ID       "multlist_anim"
#define DEMO_MULTLIST_DIALOGUE_ID   "multlist_dialog"

#define MULTLIST_DIR_HOR             0
#define MULTLIST_DIR_VER             1
#define MULTLIST_DIR_MASK            0x1
#define MULTLIST_DIR_OFFSET          0
#define MULTLIST_IS_VER(flag)   (MULTLIST_DIR_VER == ((flag >> MULTLIST_DIR_OFFSET) & MULTLIST_DIR_MASK))



#define DEMO_GAP  20
#endif
