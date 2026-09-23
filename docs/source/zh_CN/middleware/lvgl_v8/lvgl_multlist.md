# LVGL v8 multlist 列表控件

`lvsf_multlist` 是 SiFli 为 LVGL v8 封装的高性能可滑动列表容器，用于把多个同构元素按行（垂直）或按列（水平）组织起来，并统一管理滑动、惯性、对齐、回弹、缩放变形、节点增删改查等行为。它解决的是应用层反复手写“可滑动 + 自动对齐 + 边界回弹 + 动态加载 item”的问题，常见于主菜单、卡片流、封面流、聊天消息流、平铺式（TLV）页面等场景。

- 主要头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_multlist.h_
- 实现目录：_middleware/lvgl/lvsf/gui_widgets/_
- 示例工程：_example/multimedia/lvgl/lvgl_v8_multlist_

```{note}
multlist 本身是“列表容器”，具体每个 item 的视觉内容由应用通过 create / remove 回调动态创建和销毁。它与 `lvsf_multswipe`、`lvsf_multedge`、`lvsf_scrollbar` 组成插件化体系，三者通过 multlist 发出的事件联动，详见各控件文档。
```

## 功能简介

- 垂直 / 水平两个方向的可滑动列表，支持惯性滑动（throw）、循环（loop）和按页翻转。
- 滑动结束后自动对齐：支持无对齐、头对齐、居中对齐、尾对齐，并可配置距离 / 速度阈值。
- 边界回弹：可分别配置头尾的最大拖拽距离与回弹后的停靠位置。
- 动态 item：通过链表管理 item 节点，屏幕外 item 可动态创建 / 销毁，节省内存；支持带删除动画的节点移除与重排。
- 视觉变形：可使能贝塞尔（bezier）缩放算法和椭圆（ellipse）弧形偏移算法，实现中心 item 大、两侧 item 缩小或弧形排列的效果。
- 编码器（滚轮）支持：可通过旋转编码器驱动 item 移动。
- 插件联动：通过 flag 与事件与滑动删除（multswipe）、侧边边缘（multedge）、滚动条（scrollbar）等控件配合。

## 使用场景

- 应用主菜单 / 启动器图标列表、卡片流、封面流横向浏览。
- 整屏图片或页面的水平 / 垂直轮播、翻页。
- 聊天 / 对讲风格的消息流（节点信息动态增删）。
- 需要 item 随滑动距离做缩放、透明度、3D 翻转等过渡动画的聚焦式浏览界面。
- 需要外接滚轮编码器切换选项的列表界面。

## 支持的开发板

参考例程 `example/multimedia/lvgl/lvgl_v8_multlist` 在以下开发板验证：

- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

工程可通过 `scons --board=<board>` 适配不同板型，支持 SF32LB52x / SF32LB56x / SF32LB58x 等系列板子。

## 配置与初始化

在 menuconfig 中启用对应组件（默认已开启）：

```none
CONFIG_LVSF_USE_MULTLIST=y
```

控件内部用一组 flag 控制功能开关，通过 `lv_multlist_add_flag()` / `lv_multlist_clear_flag()` 使能或关闭。常用 flag 见下表：

| flag | 含义 |
| --- | --- |
| `LV_MULTLIST_FLAG_THROW` | 使能惯性滑动，默认使能 |
| `LV_MULTLIST_FLAG_LOOP` | 使能 item 循环模式 |
| `LV_MULTLIST_FLAG_RESIDENCY` | 居中对齐时按 item 间隔（而非 item 中心）对齐屏幕中心 |
| `LV_MULTLIST_FLAG_BEZIER_ALG` | 使能贝塞尔缩放算法，设置贝塞尔参数后需使能 |
| `LV_MULTLIST_FLAG_ELLIPSE_ALG` | 使能椭圆弧形偏移 |
| `LV_MULTLIST_FLAG_EDGE` | 滑动聚焦动画结束时发送 edge 事件，用于与 multedge 交互 |
| `LV_MULTLIST_FLAG_SCROLLBAR` | 滑动时发送 scrollbar 事件，用于与 scrollbar 交互 |
| `LV_MULTLIST_FLAG_BOUNDARY` | 滑动动画结束时将两个 item 间的分隔线对齐到中心点 |
| `LV_MULTLIST_FLAG_LOCK_SCRL` | 锁定滚动，禁止滑动 |
| `LV_MULTLIST_FLAG_SNAPSHOT` | 动态拍照模式，屏幕外 item 会被删除 |
| `LV_MULTLIST_FLAG_SNAPSHOT_ALL` | 对所有 item 拍照并保留，不删除 |
| `LV_MULTLIST_FLAG_TOW_PAGE` / `LV_MULTLIST_FLAG_THREE_PAGE` | 两页 / 三页刷新模式 |
| `LV_MULTLIST_FLAG_INFINTE` | 节点无限模式 |
| `LV_MULTLIST_FLAG_SHOW_ALL` | 元素总长度小于 multlist 长度时的对齐方式（无限模式用） |
| `LV_MULTLIST_FLAG_ALIGN_HEAD` | 元素不足一屏时默认头对齐 |

### 初始化流程

典型的创建、配置、回调注册、添加节点、初始对齐流程如下（摘自例程 `demo_multlist_list.c`）：

```c
#include "lvsf_multlist.h"

lv_obj_t *list = lv_multlist_create(lv_scr_act());
lv_obj_remove_style_all(list);
lv_obj_set_size(list, LV_HOR_RES_MAX, LV_VER_RES_MAX);
lv_obj_set_style_bg_color(list, lv_color_make(82, 93, 118), 0);
lv_obj_set_style_bg_opa(list, LV_OPA_COVER, 0);
lv_obj_center(list);

/* 贝塞尔变形 + 间距 + 拖拽边界 + 方向 */
float para[] = { 0, 0, 0, 0.1f, 0.3f };
lv_multlist_set_bezier_para(list, LV_VER_RES_MAX, para, para);
lv_multlist_set_gap(list, 20);
lv_multlist_set_scrl_pad(list, LV_VER_RES_MAX >> 1, LV_VER_RES_MAX >> 1);
lv_multlist_set_dir(list, LV_MULTLIST_DIR_VER);

/* 注册 item 生命周期回调，再添加节点信息 */
lv_multlist_set_item_cb(list, demo_create_item, NULL, NULL);
for (uint32_t i = 0; i < 100; i++)
{
    lv_multlist_add_info(list, LV_HOR_RES, 110, NULL, NULL);
}

/* 回弹位置与初始对齐 */
lv_multlist_set_springback(list, 0, 0);
lv_multlist_align_to(list, LV_MULTLIST_ALIGN_HEAD, 0, 0, 0);
```

页面恢复 / 暂停时通常配合编码器与生命周期调用：

```c
/* resume */
lv_multlist_on_resume(list);
lv_multlist_enable_encoder(list, 5, 400, false);

/* pause */
lv_multlist_on_pause(list);
lv_multlist_disable_encoder(list);
```

```{warning}
`lv_multlist_add_info()` 必须传入预估准确的 item 宽高。宽高偏差会直接导致元素对齐位置偏移、滚动条进度不准。清空所有节点后再对齐时，建议先把位置重置到有效 pos，避免历史 pos 超出范围导致 item 显示不全。
```

## API 说明

以下函数签名均逐字来自 _lvsf_multlist.h_。

### 基础对象与 flag

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `lv_obj_t *lv_multlist_create(lv_obj_t *parent)` | 创建 multlist 列表对象 | `parent`：父对象；返回新建列表对象指针 |
| `lv_dir_t lv_multlist_get_gesture(lv_obj_t *multlist)` | 获取触摸手势方向 | 返回当前手势方向枚举 |
| `int lv_multlist_has_flag(lv_obj_t *multlist, uint32_t flag)` | 查询是否已设置指定 flag | 已设置返回非 0，否则返回 0 |
| `void lv_multlist_add_flag(lv_obj_t *multlist, uint32_t flag)` | 添加指定功能 flag（使能对应功能） | `flag`：待使能的标志位 |
| `void lv_multlist_clear_flag(lv_obj_t *multlist, uint32_t flag)` | 清除指定 flag（关闭对应功能） | `flag`：待清除的标志位 |
| `void lv_multlist_refresh(lv_obj_t *multlist)` | 根据当前滚动位置刷新所有 item | 滚动位置 / 属性变化后调用 |
| `int lv_multlist_is_item_full(lv_obj_t *multlist)` | 检查所有 item 总长度是否超过 multlist 高度 | 超出返回 true（1），否则返回 false（0） |

### 滚动与位置

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `void lv_multlist_set_pos(lv_obj_t *multlist, int32_t position)` | 直接设置滚动位置（无动画跳转） | `position`：目标滚动位置（像素） |
| `int32_t lv_multlist_get_pos(lv_obj_t *multlist)` | 获取当前滚动位置 | 返回当前滚动位置（像素） |
| `void lv_multlist_set_snapshot(lv_obj_t *multlist, lv_event_cb_t cb, lv_img_cf_t cf)` | 配置 item 动态拍照（snapshot）回调与图像格式 | `cb`：快照事件回调；`cf`：图像颜色格式 |
| `void lv_multlist_set_scrl_pad(lv_obj_t *multlist, lv_coord_t head, lv_coord_t tail)` | 设置头尾可拖拽的最大偏移距离 | `head` / `tail`：头 / 尾最大拖拽距离 |
| `void lv_multlist_set_show_pad(lv_obj_t *multlist, uint16_t head, uint16_t tail)` | 设置额外展示区域，超出后 item 被移除 | 默认 0，控制 item 销毁阈值 |
| `void lv_multlist_set_gap(lv_obj_t *multlist, uint16_t gap)` | 设置相邻 item 之间的间距 | `gap`：item 间距（像素） |
| `void lv_multlist_set_dir(lv_obj_t *multlist, lv_multlist_dir_t dir)` | 设置滚动方向 | `dir`：`LV_MULTLIST_DIR_VER` / `LV_MULTLIST_DIR_HOR` |
| `lv_multlist_dir_t lv_multlist_get_dir(lv_obj_t *multlist)` | 获取滚动方向 | 返回当前方向枚举 |
| `void lv_multlist_set_springback(lv_obj_t *multlist, lv_coord_t edge_head, int16_t edge_tail)` | 设置越界滑动结束后的回弹停靠位置 | `edge_head` / `edge_tail`：头 / 尾回弹后相对边的间距 |
| `int32_t lv_multlist_get_springback_head(lv_obj_t *multlist)` | 计算头部回弹区域位置 | 返回头部回弹位置（像素） |
| `int32_t lv_multlist_get_springback_tail(lv_obj_t *multlist)` | 计算尾部回弹区域位置 | 返回尾部回弹位置（像素） |

### 对齐控制

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `void lv_multlist_set_align(lv_obj_t *multlist, lv_multlist_align_t align, lv_coord_t offset)` | 设置滑动结束后的默认对齐方式 | `align`：`LV_MULTLIST_ALIGN_NONE/CENTER/HEAD/TAIL`；`offset`：对齐点偏移 |
| `void lv_multlist_set_first_align(lv_obj_t *multlist, lv_multlist_align_t align, lv_coord_t offset, int16_t index)` | 设置首次（恢复时）的对齐方式 | `index`：初始对齐的 item 索引 |
| `int32_t lv_multlist_get_focus_pos(lv_obj_t *multlist, lv_multlist_align_t align, lv_multlist_item_t *item)` | 计算指定 item 对齐到目标位置所需的滚动值 | 返回所需滚动位置（像素） |
| `void lv_multlist_focus_near(lv_obj_t *multlist, int32_t offset, uint8_t dir_en, uint32_t time)` | 相对偏移滚动并对齐最近 item | `offset`：相对偏移；`dir_en`：true 按偏移方向对齐，false 对齐最近；`time`：动画时长，0 关闭动画 |
| `void lv_multlist_align_head_to(lv_obj_t *multlist, int16_t index, lv_coord_t edge_offset)` | 将指定索引 item 对齐到头部 | `edge_offset`：头部边缘偏移 |
| `void lv_multlist_align_tail_to(lv_obj_t *multlist, int16_t index, lv_coord_t edge_offset)` | 将指定索引 item 对齐到尾部 | `edge_offset`：尾部边缘偏移 |
| `void lv_multlist_align_center_to(lv_obj_t *multlist, int16_t index, lv_coord_t edge_offset)` | 将指定索引 item 对齐到中心 | `edge_offset`：中心偏移 |
| `void lv_multlist_align_to(lv_obj_t *multlist, lv_multlist_align_t align, int16_t index, lv_coord_t offset, uint32_t time)` | 以动画将指定 item 对齐到目标位置 | `time`：动画时长 |
| `void lv_multlist_set_focus_threshold(lv_obj_t *multlist, uint16_t dis, uint16_t vect)` | 设置翻页对齐的距离 / 速度阈值 | 滑动距离或速度超过阈值即触发下一页对齐 |

### item 节点管理

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `lv_multlist_item_t *lv_multlist_add_info(lv_obj_t *multlist, lv_coord_t w, lv_coord_t h, void *info, void *user_data)` | 向链表添加节点信息，滑动时动态加载 item | `w` / `h`：item 宽高；`info` / `user_data`：用户数据；返回 item 句柄 |
| `void lv_multlist_updata_element(lv_obj_t *multlist, lv_multlist_item_t *item, bool delete)` | item 属性变化时强制刷新元素 | `delete`：是否删除旧元素后重建 |
| `uint8_t lv_multlist_insert_info(lv_obj_t *multlist, lv_multlist_item_t *item, lv_multlist_item_t *ref)` | 在参考项前插入 item（无动画） | 成功返回 1，失败返回 0 |
| `uint32_t lv_multlist_item_remove(lv_obj_t *multlist, lv_multlist_item_t *delete_item, lv_multlist_anim_type_t type, uint8_t free)` | 以删除动画移除 item | `type`：动画类型 `LV_MULTLIST_ANIM_NONE/DEL/FLY/SALCE/ZOOM`；`free`：动画后是否释放内存；成功返回 1 |
| `uint32_t lv_multlist_item_move_before(lv_obj_t *multlist, lv_multlist_item_t *insert_item, lv_multlist_item_t *item_ref, uint8_t en_anim)` | 把 item 移动到参考项之前 | `en_anim`：1 使能动画；成功返回 1 |
| `void lv_multlist_remove_info_all(lv_obj_t *multlist)` | 移除所有由 add_info 添加的节点信息 | 常用于 pause 后重建节点 |
| `void lv_multlist_load_all_item(lv_obj_t *multlist)` | 依据已添加节点信息加载所有 item | — |
| `uint32_t lv_multlist_get_info_cnt(lv_obj_t *multlist)` | 获取节点信息数量 | 返回节点数 |
| `lv_multlist_item_t *lv_multlist_get_center_item(lv_obj_t *multlist)` | 获取当前居中的 item | 返回居中 item 句柄 |
| `lv_multlist_item_t *lv_multlist_get_focus_item(lv_obj_t *multlist, lv_coord_t offset)` | 获取当前对齐点处的 item | `offset`：对齐点偏移；返回对齐 item 句柄 |
| `lv_multlist_item_t *lv_multlist_get_item_by_index(lv_obj_t *multlist, int16_t index)` | 按索引获取 item 句柄 | 返回对应 item 句柄 |

### 变形与特效

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `void lv_multlist_set_bezier_para(lv_obj_t *multlist, uint32_t range, float *pos_arr, float *neg_arr)` | 设置 item 缩放的贝塞尔参数 | `range`：对齐点最远有效偏移；`pos_arr` / `neg_arr`：正 / 负方向 5 个递增缩放系数 |
| `void lv_multlist_set_ellipse_para(lv_obj_t *multlist, lv_coord_t x_axis, lv_coord_t y_axis)` | 设置 item 弧形（椭圆）偏移 | `x_axis` / `y_axis`：椭圆 X / Y 轴距 |
| `void lv_multlist_set_angles(lv_obj_t *multlist, int16_t offset_angle, int16_t start_angle, int16_t end_angle)` | 环形列表模式下设置角度 | `offset_angle`：整体旋转角；`start_angle` / `end_angle`：展示起止角 |
| `void lv_multlist_set_radius(lv_obj_t *multlist, uint16_t r, uint16_t virt_r)` | 设置环形列表半径 | `r`：布局半径；`virt_r`：拖拽距离换算的虚拟半径 |
| `void lv_multlist_set_overlap_cnt(lv_obj_t *multlist, uint16_t overlap_cnt)` | 设置堆叠（重叠）item 数量 | 堆叠菜单专用 |
| `void lv_multlist_set_pivot_offset(lv_obj_t *multlist, lv_coord_t offset_x, lv_coord_t offset_y)` | 设置 item 变换中心相对中心的偏移 | 用于缩放 / 旋转的支点偏移 |

### 动画与回调

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `void lv_multlist_set_anim_para(lv_obj_t *multlist, lv_coord_t rate, lv_coord_t min_time, lv_coord_t max_time)` | 设置动画基础参数 | `rate`：每像素耗时；`min_time` / `max_time`：最短 / 最长动画时长 |
| `void lv_multlist_fly_anim(lv_obj_t *multlist, lv_multlist_fly_type_t type, uint16_t delay, uint32_t time, uint8_t fly_in, lv_anim_ready_cb_t ready_cb)` | 设置 item 飞入 / 飞出动画 | `type`：方向（左上 / 右上 / 左中 / 右中 / 左下 / 右下）；`delay`：延迟；`time`：时长；`fly_in`：1 飞入 / 0 飞出 |
| `lv_anim_t *lv_multlist_anim(void *multlist, int32_t start, int32_t end, uint32_t pred, lv_anim_exec_xcb_t exe_cb, lv_anim_ready_cb_t ready_cb, lv_anim_path_cb_t path_cb)` | 创建自定义列表动画 | 返回动画句柄 |
| `void lv_multlist_set_refresh_cb(lv_obj_t *multlist, lv_multlist_refresh_cb refresh_cb)` | 设置自定义刷新回调 | 替代默认刷新逻辑时使用 |
| `void lv_multlist_set_page_anim_cb(lv_obj_t *multlist, lv_multlist_page_cb anim_cb)` | 设置分页模式动画回调 | 多页刷新时触发 |
| `void lv_multlist_set_tranform_cb(lv_obj_t *multlist, lv_multlist_tranform_cb tranform_cb)` | 设置 item 变换回调 | 自定义缩放 / 透明度 / 层级等变形 |
| `void lv_multlist_set_item_cb(lv_obj_t *multlist, lv_multlist_create_item_cb create_cb, lv_multlist_remove_item_cb remove_cb, lv_multlist_delete_info_cb delete_cb)` | 注册 item 生命周期回调 | `create_cb`：创建 element；`remove_cb`：删除 element 前（禁止在其中删 element，用于资源清理）；`delete_cb`：删除 info 数据（此时 element 已释放，禁止访问 element） |

### 编码器与生命周期

| 接口函数 | 功能说明 | 参数 / 返回值说明 |
| --- | --- | --- |
| `void lv_multlist_enable_encoder(lv_obj_t *multlist, uint32_t ratio, uint32_t vect_max, uint8_t reverse)` | 使能编码器（滚轮）控制 item 移动 | `ratio`：每刻度速度增量；`vect_max`：最大速度；`reverse`：是否反转方向 |
| `void lv_multlist_disable_encoder(lv_obj_t *multlist)` | 禁用编码器 | — |
| `void lv_multlist_on_pause(lv_obj_t *multlist)` | 暂停列表：移除所有显示 item 并停止刷新 | 页面 pause 时调用 |
| `void lv_multlist_on_resume(lv_obj_t *multlist)` | 恢复列表：重新加载 item 并恢复刷新 | 页面 resume 时调用 |

### 事件

multlist 在特定状态下向对象发送自定义事件，应用通过 `lv_obj_add_event_cb()` 监听：

| 事件 | 含义 |
| --- | --- |
| `LV_EVENT_LIST_SCROLL_STRAT` / `LV_EVENT_LIST_SCROLLING` / `LV_EVENT_LIST_SCROLL_END` | 开始滑动 / 滑动中 / 滑动结束 |
| `LV_EVENT_LIST_FOCUS` | item 对齐到目标位置时发送 |
| `LV_EVENT_LIST_FOCUS_LOSS` | item 离开对齐位置时发送 |
| `LV_EVENT_LIST_SCROLLBAR` | 通知 scrollbar 更新进度 |
| `LV_EVENT_EDGE_DRAGE_*` | 与 multedge 联动的边缘拖拽请求 / 开始 / 拖拽中 / 结束等事件 |
| `LV_EVENT_SWIPE_DELETE` | 有 item 被滑动删除时发送 |

## 典型用法

完整可运行例程见 `example/multimedia/lvgl/lvgl_v8_multlist`，包含普通列表、按页翻转、自定义动画、聊天消息流等子页面。最小调用序列如下：

```c
/* 1. 创建并配置 */
lv_obj_t *list = lv_multlist_create(lv_scr_act());
lv_multlist_set_dir(list, LV_MULTLIST_DIR_VER);
lv_multlist_set_gap(list, 20);
lv_multlist_set_scrl_pad(list, LV_VER_RES_MAX >> 1, LV_VER_RES_MAX >> 1);

/* 2. 注册 item 创建回调（在回调里真正创建每个 item 的控件树） */
lv_multlist_set_item_cb(list, my_create_item, NULL, NULL);

/* 3. 添加节点信息（item 真正的控件在滑入屏幕时才由回调创建） */
lv_multlist_add_info(list, LV_HOR_RES, 110, user_info, NULL);

/* 4. 设置对齐与回弹 */
lv_multlist_set_align(list, LV_MULTLIST_ALIGN_CENTER, 0);
lv_multlist_set_springback(list, 0, 0);
lv_multlist_align_center_to(list, 0, 0);

/* 5. resume 时使能编码器，pause 时停止 */
lv_multlist_on_resume(list);
lv_multlist_enable_encoder(list, 5, 400, false);
```

item 创建回调示例（摘自例程）：

```c
static lv_obj_t *demo_create_item(lv_obj_t *parent, lv_multlist_item_t *item)
{
    lv_obj_t *item_bg = lv_obj_create(parent);
    lv_obj_remove_style_all(item_bg);
    lv_obj_set_size(item_bg, item->org_w, item->org_h);
    lv_obj_set_style_bg_color(item_bg, ITEM_BG_COLOR, LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(item_bg, LV_OPA_100, LV_STATE_DEFAULT);
    lv_obj_add_flag(item_bg, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_EVENT_BUBBLE);
    lv_obj_clear_flag(item_bg, LV_OBJ_FLAG_SCROLLABLE);
    /* ... 在 item_bg 上继续放置图片、文本等内容 ... */
    return item_bg;
}
```

## 效果展示

常规垂直列表滑动与居中对齐效果：

```{image} ../../../assets/lvgl_v8/multlist_show.gif
:alt: multlist 效果
```

单独使能贝塞尔缩放变形（左右两图分别为仅贝塞尔、贝塞尔 + 椭圆偏移）：

```{image} ../../../assets/lvgl_v8/multlist_bezier.png
:alt: multlist 贝塞尔变形
:width: 800px
:align: center
```

头部回弹区域（head）与尾部回弹区域（tail）示意：

```{image} ../../../assets/lvgl_v8/multlist_head.png
:alt: multlist 头部回弹
:width: 800px
:align: center
```

```{image} ../../../assets/lvgl_v8/multlist_tail.png
:alt: multlist 尾部回弹
:width: 800px
:align: center
```

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- 头文件：_middleware/lvgl/lvsf/gui_widgets/lvsf_multlist.h_
- 示例工程：_example/multimedia/lvgl/lvgl_v8_multlist_
