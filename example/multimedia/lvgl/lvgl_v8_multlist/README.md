# LVGL v8 Multlist 示例

源码路径: `SiFli-SDK/example/multimedia/lvgl/lvgl_v8_multlist`

## 概述

本例程用于演示 `lvsf_multlist` 控件在实际应用中的组织方式与页面切换逻辑，重点展示以下几类典型能力：

- 基于 `gui_app_fwk` 的应用入口、页面注册与子页面跳转
- `lvsf_multlist` 在纵向/横向列表场景下的布局与聚焦控制
- `lvsf_multlist` 在整页滚动场景下的循环翻页效果
- 通过 `transform callback` 自定义卡片动画表现
- 结合 `lv_txtimg` 构建动态文本对话列表

整个例程不是单一控件的最小示例，而是一个带有主入口、菜单页、功能子页、资源缓存和返回导航的完整 Demo 框架，适合作为 `lvsf_multlist` 类界面的参考工程。
## 支持的开发板

<!-- 支持哪些板子和芯片平台 -->
- sf32lb52-lcd_n16r8
- sf32lb52-lchspi-ulp

## 硬件需求

运行该例程前，需要准备：
+ 一块本例程支持的开发板（[支持的平台](quick_start)）。
+ 屏幕。

工程编译及下载：
支持的板子
- 55x之后的板子，比如58x，56x, 52x的

板子工程在project目录下可以通过指定board来编译适应相对board的工程，
- 比如想编译可以在HDK 563上运行的工程，执行scons --board=eh-lb563即可生成工程
- 下载可以通过build目录下的download.bat进行，比如同样想烧录上一步生成的563工程，可以执行.\build_eh-lb563\download.bat来通过jlink下载
- 特别说明下，对于SF32LB52x/SF32LB56x系列会生成额外的uart_download.bat。可以执行该脚本并输入下载UART的端口号执行下载

## 模拟器配置

模拟器工程在simulator目录下，
- 使用 scons 进行编译，SiFli-SDK/msvc_setup.bat文件需要相应修改，和本机MSVC配置对应
- 也可以使用 scons --target=vs2017 生成 MSVC工程 project.vcxproj, 使用Visual Studio 进行编译。
    注：如果不是使用VS2017, 例如 VS2022, 加载工程的时候，会提示升级MSVC SDK, 升级后就可以使用了。


## 运行说明

### 启动流程

启动流程如下：
- 调用 `gui_app_init()` 初始化应用框架
- 调用 `gui_app_run(DEMO_MULTLIST_MAIN_ID)` 进入 Multlist 演示主入口

因此，本例程的运行入口不是单个 `lv_example_xxx()` 函数，而是基于 `gui_app_fwk` 管理的一组页面。

### 页面结构

例程整体可以分为两层：

- 主入口页
  说明：
  `demo_multlist_main.c` 创建一个启动卡片，点击后进入 Multlist 主菜单
- Multlist 主菜单页
  说明：
  `demo_multlist.c` 创建一个纵向 `lvsf_multlist` 菜单，并通过 `gui_app_run_subpage()` 跳转到不同功能页

主菜单中当前包含以下子功能：

- `horizontal list`
- `vertical list`
- `horizontal page`
- `vertical page`
- `horizontal animation`
- `vertical animation`
- `Intercom list`

### 设计逻辑框架

本例程的设计逻辑可以理解为“应用框架层 + 页面层 + 资源层”三层结构：

- 应用框架层
  说明：
  通过 `APPLICATION_REGISTER_*` 和 `APP_PAGE_REGISTER` 将页面注册进 `gui_app_fwk`，统一处理启动、恢复、暂停、停止和返回
- 页面实现层
  说明：
  每个子页面只关心自己的 `on_start / on_resume / on_pause / on_stop` 生命周期，以及对应的 `lvsf_multlist` 配置
- 资源与缓存层
  说明：
  `demo_multlist_cards.c` 负责图片源、卡片尺寸计算和快照缓存，为列表页和动画页复用卡片资源


### 各功能页说明

- 列表页 `demo_multlist_list.c`
  说明：
  演示普通列表样式。页面会先通过 `demo_multlist_card_cache_init()` 生成缩略卡片，再创建 `lvsf_multlist`，最后根据横向/纵向模式填充 100 个列表项
- 翻页页 `demo_multlist_page.c`
  说明：
  以整屏图片为单位演示横向或纵向分页效果，并开启 `LV_MULTLIST_FLAG_LOOP` 实现循环翻页
- 动画页 `demo_multlist_anim.c`
  说明：
  通过 `lv_multlist_set_tranform_cb()` 自定义元素的缩放、透明度和层级变化，展示更强的视觉动态效果
- 对话页 `demo_multlist_dialog.c`
  说明：
  结合 `lv_txtimg` 和 `lvsf_multlist` 实现聊天/对讲风格的消息流，支持新增消息和对最后一条消息继续追加文本

### 资源组织方式

与直接使用原始图片不同，本例程将卡片资源抽象成统一的快照缓存：

- 原始图片资源由 `demo_list_0 ~ demo_list_3` 提供
- `demo_multlist_cards.c` 根据屏幕大小动态计算列表卡片和动画卡片尺寸
- 页面实际使用的是缓存后的 `lv_img_dsc_t` 快照，而不是每次临时重新创建

### 导航与交互

- 每个页面顶部都创建统一的返回按钮，点击后调用 `gui_app_goback()`
- 页面恢复时会调用 `lv_multlist_on_resume()` 与 `lv_multlist_enable_encoder()`
- 页面暂停时会调用 `lv_multlist_on_pause()` 与 `lv_multlist_disable_encoder()`
- 在横向页面中会临时关闭手势，避免与页面滑动方向冲突

### 适合作为参考的场景

本例程比较适合作为以下类型界面的参考模板：

- 启动器或功能入口列表
- 卡片流、封面流、横向/纵向菜单
- 整页轮播或分页浏览
- 带有过渡动画的聚焦式浏览界面
- 聊天记录、对讲消息流等动态文本列表

## 异常诊断

如有任何技术疑问，请在GitHub上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)

## 参考文档
- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
