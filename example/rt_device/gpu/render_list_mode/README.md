# GPU渲染列表模式示例程序
源码路径：example/rt_device/gpu/render_list_mode
```{warning}
未验证
```
## 概述
EPIC（ePicasso），是 SiFli 自研的图形引擎，它是专为 2D/2.5D 图像处理设计的硬件加速模块，主要用于分担 CPU 在图像运算中的负载，提升图像处理效率。

图形渲染驱动层提供了两种不同的实现接口（不能同时使用）：
1. 单步渲染模式 —— 每准备一个操作之后，就执行对应处理的api
2. 渲染列表模式 —— 准备一帧所有操作，之后异步批量处理


本示例程序演示的是**渲染列表模式**：将一系列渲染操作（填充、图片、矩形、多边形、边框、圆弧、线、点阵字等）搜集到一个渲染列表中，然后一起渲染。渲染时利用 2 个 1/16 屏幕大小的 framebuffer（`render_buffer`）分块渲染，并通过回调将渲染结果送到屏幕显示；一次渲染完成后，继续创建新的渲染操作并送屏，实现循环演示。

## 功能介绍
该例程是一个基于 RT-Thread 操作系统和 EPIC 渲染列表模式的图形演示示例。程序内置 6 个场景，每 10 秒自动切换一次，循环播放：

| 场景 | 演示内容 |
|------|---------|
| 场景 0 | 全屏灰色填充 → 嵌套渲染列表生成随机色块测试图（`generate_image`）→ ARGB8565 图片居中显示 → 4 个同心圆弧（20°~160°，圆头圆尾） |
| 场景 1 | 纯色填充 → EZIP 压缩格式的时钟背景图（454x454，带缩放）→ 圆弧动画（6 个同心圆弧，前景红色顺时针扫过） |
| 场景 2 | 纯色填充 → 圆角矩形（交替纯色 / 四角渐变）→ 圆角边框（红色，线宽 20）→ 点阵字（27x30 字模，A8 半透明，铺满屏幕） |
| 场景 3 | 纯色填充 → 直线（水平虚线、垂直虚线、斜实线，线宽 20）→ 六边形多边形 → 圆弧动画 → 点阵字 |
| 场景 4 | 纯色填充 → 图片绕 Y 轴 / X 轴 3D 翻转动画（两幅不同尺寸图片，带透视变换） |
| 场景 5 | 纯色填充 → JPEG 硬件解码图片（454x454，需 `EPIC_SUPPORT_JPEGD`，否则回退为 EZIP 图）→ 渐变按钮 + 60x60 EZIP 图标 |

## 支持的开发板
例程可以运行在以下开发板（`project/proj.conf` 为默认单核配置；sf32lb57x 使用独立的 `project/sf32lb57x/proj.conf` 双核 HCPU 配置）：
* sf32lb52-lchspi-ulp
* sf32lb52-lcd_n16r8
* sf32lb56-lcd_a128r12n1
* sf32lb57x（双核 HCPU 模式，渲染列表任务在 ACPU 上执行）

### 硬件需求
* 支持该示例的开发板：需选用带 PSRAM 的封装（适用于 52x/56x；sf32lb57x 需配合双核 HCPU 配置使用）
* 配套 LCD 屏：开发板需带有 LCD 显示接口与屏幕（如 sf32lb52-lcd_n16r8、sf32lb56-lcd_a128r12n1）
* 一根具备数据传输能力的 USB 数据线（用于串口烧录与日志输出）

## 例程的使用
下面的小节仅提供本示例必需的信息。有关配置 SiFli-SDK 及使用其构建和运行项目的完整步骤，请参阅 [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)。

### menuconfig 配置
* 本示例所需的关键配置已默认写入 `project/proj.conf`，直接编译即可。如需修改，可在 SDK 根目录执行：
```
sdk.py menuconfig --board=<board_name>
```
以下按 `project/proj.conf` 中的配置项逐项说明其 menuconfig 路径与作用：

#### 1. CONFIG_DRV_EPIC_NEW_API=y — 渲染列表模式（核心开关）
- 路径：Board Config → On-chip Peripherals → EPIC
    - 开启： Using epic render list API
    - 作用：启用渲染列表模式（新 API），`main.c` 完全基于该模式编写；关闭则编译为单步渲染模式。
    - 注意：渲染列表模式不能用于 SF32LB55x 系列；渲染是异步的，渲染完成前必须保持渲染列表引用的 buffer 数据有效。
- 子选项 EPIC Render List API Mode（单选，默认 SINGLE_CORE）：
    - `CONFIG_DRV_EPIC_NEW_API_SINGLE_CORE`：单核模式（`project/proj.conf` 默认配置，渲染列表在本地 CPU 执行）
    - `CONFIG_DRV_EPIC_NEW_API_DUAL_CORE_HCPU`：双核 HCPU 模式（自动选中 `USING_ACPU_CTRL_FWK`；sf32lb57x 工程 `project/sf32lb57x/proj.conf` 使用此模式，渲染列表任务 `ACPU_TASK_epic_rl` 下发到 ACPU 上执行）
    - `CONFIG_DRV_EPIC_NEW_API_DUAL_CORE_ACPU`：双核 ACPU 模式（渲染列表任务在 ACPU 上执行）

#### 2. CONFIG_BSP_USING_FULL_ASSERT=y — HAL 断言类型
- 路径：SiFli Built-in Components → HAL Assert Type
    - 选择： Print and loop
    - 作用：HAL 断言失败时打印错误信息并进入死循环（三选一：Print and loop / Endless Loop / Empty，本项目选 Print and loop，`RT_ASSERT` 同时生效，便于调试定位）

#### 3. CONFIG_RT_MAIN_THREAD_STACK_SIZE=4096 — 主线程栈大小
- 路径：RT-Thread Components → The main() function as user entry function
    - 设置： Set main thread stack size
    - 作用：main 线程栈大小（字节）。本示例在 main 中执行大量渲染操作（局部结构体、嵌套渲染列表较多），默认值 2048 偏小，故设为 4096

#### 4. CONFIG_RT_MAIN_THREAD_PRIORITY=15 — 主线程优先级
- 路径：RT-Thread Components → The main() function as user entry function
    - 设置： Set main thread priority
    - 作用：main 线程优先级，数值越小优先级越高（15 为中等优先级；EPIC 渲染列表线程 `epic_task` 优先级为 `RT_THREAD_PRIORITY_HIGH-3`，数值上小于 15，即优先级更高）

#### 5. CONFIG_RT_USING_ULOG=y — ulog 日志
- 路径：RT-Thread Components → Utilities
    - 开启： Enable ulog
    - 作用：启用 RT-Thread ulog 日志框架；开启后 `rt_kprintf` 输出经 ulog 处理（如附加线程名、颜色控制）

#### 6. # CONFIG_ULOG_USING_COLOR is not set — 关闭彩色日志
- 路径：RT-Thread Components → Utilities → ulog（Enable ulog 开启后）→ log format
    - 关闭： Enable color log（默认开启，本项目关闭）
    - 作用：日志不输出 ANSI 颜色控制字符，串口工具 / 日志文件阅读更干净

#### 7. CONFIG_ULOG_OUTPUT_THREAD_NAME=y — 日志输出线程名
- 路径：RT-Thread Components → Utilities → ulog（Enable ulog 开启后）→ log format
    - 开启： Enable thread information（默认关闭，本项目开启）
    - 作用：每条日志附带线程名，便于多线程场景排查

### 编译和烧录
切换到例程 `project` 目录，运行 scons 命令执行编译（以 `sf32lb52-lchspi-ulp` 为例）：
```
scons --board=sf32lb52-lchspi-ulp -j8
```
编译产物生成在 `build_sf32lb52-lchspi-ulp_hcpu/` 目录下，执行烧写命令（Windows）：
```
build_sf32lb52-lchspi-ulp_hcpu\uart_download.bat
```
macOS/Linux 下执行：
```
./build_sf32lb52-lchspi-ulp_hcpu/uart_download.sh
```
按提示选择串口端口即可进行下载：
```none
please input the serial port num:6
```

## 例程输出结果展示
在 LCD 屏幕上循环播放 6 个场景，每 10 秒切换一个，演示内容见"功能介绍"表格。如果看到以下串口输出，则示例应已正确运行（以下为节选；日志中的场景编号比表格编号大 1，`Showing scene 1~6` 对应表格中的场景 0~5，循环播放）：

```none
__main start
generate_image start
generate_image done
Showing scene 1
Showing scene 2
Showing scene 3
Showing scene 4
Showing scene 5
Showing scene 6
```

---

## 例程的讲解
### 程序主要流程包括：
```
初始化 GPU 和 LCD 设备
  └─> drv_gpu_open() 打开 GPU 驱动
  └─> open_lcd() 打开 LCD 设备并获取屏幕信息、像素对齐要求
  └─> drv_epic_setup_render_buffer() 设置 2 个分块渲染 buffer
      └─> 生成测试图片（嵌套渲染列表，EPIC_MSG_RENDER_TO_BUF 渲染到 buffer）
          └─> while(1) 循环：
              ├─> drv_epic_alloc_render_list() 申请新的渲染列表
              ├─> 按当前场景创建一组渲染操作（drv_epic_alloc_op + drv_epic_commit_op）
              ├─> 提交 EPIC_MSG_RENDER_DRAW 消息开始渲染
              ├─> partial_done_cb 回调中分块送屏显示（lcd_flush）
              ├─> 等待渲染完成信号 render_done_sema
              └─> 等待 LCD 异步刷屏完成，进入下一帧
```

### 渲染列表模式的两种执行场景
1. **渲染 + 送屏（`EPIC_MSG_RENDER_DRAW`）**
   根据配置的渲染 buffer 大小分块渲染，每完成一小块就执行 `partial_done_cb` 回调，在回调中刷屏幕。本例程主循环使用的就是这种方式：`virtual_render_buf` 只是描述渲染输出的颜色格式和区域，`EPIC_MSG_RENDER_DRAW` 并不会直接画到这个 buffer，因此 `data` 可以不给真实地址（示例中为 `0xCCCCCCCC`）。
2. **渲染到指定 buffer（`EPIC_MSG_RENDER_TO_BUF`）**
   直接将渲染结果写到指定 buffer，整个完成后再调用 `done_cb`。本例程的 `generate_image()` 用它生成随机色块测试图：渲染到 `test_image` buffer，完成后由 `fill_done_cb` 释放信号量。

### 使用到的关键接口
#### 1. GPU / EPIC 渲染列表接口：
| 接口名                        | 功能描述                         |
|-----------------------------|------------------------------|
| drv_gpu_open()               | 初始化 GPU 资源，包括中断等         |
| drv_epic_setup_render_buffer() | 设置分块渲染 buffer（需要 2 个一样大小的 buffer） |
| drv_epic_alloc_render_list() | 申请一个渲染列表                   |
| drv_epic_alloc_op()          | 在渲染列表中创建一个渲染操作         |
| drv_epic_op_alloc_letter()   | 在点阵字（LETTERS）操作中分配一个字模 |
| drv_epic_commit_op()         | 将渲染操作提交到渲染列表            |
| drv_epic_render_msg_commit() | 提交渲染消息，启动渲染              |
| drv_epic_render_trav()       | 遍历渲染列表（用于渲染完成后释放变换矩阵内存等） |
| HAL_EPIC_LayerConfigInit()   | 初始化图层 / mask 配置结构体        |

#### 2. 渲染消息类型（EPIC_MsgIdDef）：
| 消息类型                   | 功能描述                             |
|---------------------------|----------------------------------|
| EPIC_MSG_RENDER_TO_BUF    | 渲染到指定 buffer，完成后调用 done_cb |
| EPIC_MSG_RENDER_DRAW      | 渲染 + 分块送屏，每块完成调用 partial_done_cb |
| EPIC_MSG_RENDER_DRAW2     | 先渲染到 buffer，再将结果绘制到 LCD |

#### 3. 渲染操作类型（drv_epic_op_type_t）：
| 操作类型                   | 功能描述                             |
|---------------------------|----------------------------------|
| DRV_EPIC_DRAW_FILL        | 填充（纯色，含透明度）                 |
| DRV_EPIC_DRAW_RECT        | 矩形（圆角、四角渐变色）               |
| DRV_EPIC_DRAW_IMAGE       | 图片（ARGB8565 / EZIP / JPEG，支持旋转缩放、3D 变换） |
| DRV_EPIC_DRAW_BORDER      | 边框（圆角，可分别控制上/下/左/右边）    |
| DRV_EPIC_DRAW_ARC         | 圆弧（起始/结束角度、粗细、圆头圆尾）    |
| DRV_EPIC_DRAW_LINE        | 直线（线宽、虚线、圆头圆尾、无限长）     |
| DRV_EPIC_DRAW_LETTERS     | 点阵字（A8 字模，颜色 + 透明度）        |
| DRV_EPIC_DRAW_POLYGON     | 多边形（最多 16 个顶点）               |

#### 4. LCD 显示接口：
| 接口名                                       | 功能描述                             |
|-------------------------------------------|----------------------------------|
| rt_device_find("lcd") | 查找 LCD 设备                     |
| rt_device_open()      | 打开 LCD 设备                     |
| rt_device_control(..., RTGRAPHIC_CTRL_GET_INFO)   | 获取 LCD 屏幕信息（分辨率、像素对齐要求） |
| rt_device_control(..., RTGRAPHIC_CTRL_SET_BUF_FORMAT) | 设置显存颜色格式（RGB888）      |
| rt_device_control(..., SF_GRAPHIC_CTRL_LCDC_FLUSH) | 将渲染完成的 buffer 刷到屏幕（lcd_flush） |
| rt_device_control(..., RTGRAPHIC_CTRL_SET_NEXT_TE) | 设置下一帧 TE（撕裂效应）同步 |

## 异常诊断

* 编译告警 "EPIC driver requrie RT_USING_MESSAGEQUEUE!"：EPIC 驱动通过消息队列提交渲染消息，请确认已开启 `RT_USING_MESSAGEQUEUE`（RT-Thread Kernel → IPC → Enable message queue）。
* 渲染列表模式在 SF32LB55x 系列上不可用：确认所用芯片为 SF32LB52x / SF32LB56x / SF32LB57x。
* 场景 5 未显示 JPEG 图片：确认已开启 `EPIC_SUPPORT_JPEGD`（未开启时该场景回退为 EZIP 图片）。
* 烧录失败或无法下载：确认串口端口选择正确（参见"编译和烧录"中的 `please input the serial port num` 提示）、开发板已连接且驱动正常。
* 画面花屏 / 撕裂：确认渲染 buffer、图片数据等地址满足 4 字节对齐要求；渲染为异步执行，渲染完成前需保持渲染列表引用的数据有效。

如有任何技术疑问，请在 GitHub 上提出 [issue](https://github.com/OpenSiFli/SiFli-SDK/issues)。

## 参考文档

- [SiFli-SDK 快速入门](https://docs.sifli.com/projects/sdk/latest/sf32lb52x/quickstart/index.html)
- EPIC 图形引擎驱动说明：`docs/source/zh_CN/drivers/epic.md`（SDK 内）
