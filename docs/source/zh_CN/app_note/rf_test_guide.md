# SiFli SDK 射频测试指南

涵盖每种测试模式对应的 finsh 命令、SDK API、参数取值与事件处理。

---

## 0. 测试模式总览

根据测试目的、控制方式和应用场景，蓝牙射频测试分为三大类：**蓝牙单项测试**、**蓝牙非信令测试**、**蓝牙信令测试**。SDK 把它们映射到不同的入口：

| 类型 | 用途 | 命令入口 | 触发源 |
|---|---|---|---|
| **BT 信令测试** | 蓝牙 SIG 资格认证(QDID)、全面性能与协议一致性评估 | `bt_cm dut` / `gap_enb_dut_mode_req()` | 综测仪空口 LMP_test_control |
| **BT 非信令测试** | 研发 RF 调试、产线快速 RF 校验 | `bt_rftest` / `bt_enter_no_signal_dut_mode()` | 本地 vendor HCI |
| **BLE 非信令测试** (SIG DTM) | 产线快速测 BLE RF、SIG 认证 DTM 项 | `bt_rftest bletx/blerx` / `ble_enter_dut_mode()` / 标准 HCI(经 `bt_cm uart_dut` 透传) | 本地 HCI 命令 / 综测仪 + PC 通过串口 HCI |
| **单项测试** | 板级深度 RF 调试、FCC/CE 法规预测试 | `bt_cm uart_dut` + SiFli_RfTool | PC 工具手动逐项 |

**信令 vs 非信令 vs 单项**：

- **信令**：仪器与 DUT 建立真实蓝牙链路，按标准协议双向通信。结果权威，认证强制。
- **非信令**：不建立蓝牙连接，通过特定指令直接控制射频芯片到指定状态(固定信道/功率/持续发收)，做快速测量。研发与产线主用。
- **单项**：工程师通过 PC 工具([SiFli_RfTool](../tools/SiFli_RfTool/SiFli_RfTool_UM.md))向 DUT 发底层控制指令，精确控制射频行为(单载波、特定调制信号)，配频谱仪/功率计逐项分析，以及法规预测试。

**与 BQB 认证的关系**：BR/EDR 的 BQB RF 测试走 BT 信令路径，BLE 的 BQB RF 测试走 SIG DTM(即 BLE 非信令);BT 非信令是思澈私有 vendor 协议，不参与 BQB，只用于研发与产线;单项测试不参与 BQB，常用于 FCC/CE 法规预认证与板级深度调优。

思澈平台提供两种测试固件：

- **rf_test.bin**：思澈内部独立的 RF 测试专用工程编译产物，**未对外开放**，特殊情况下需找 FAE 获取。
- **User.bin**：**用户基于本 SDK 自行编译出的任意固件**——包括 SDK 自带的例程、客户自己创建的产品工程等，均属于此类。SDK 已把 RF 测试功能集成进协议栈，这类固件默认开机跑用户业务，通过 finsh 命令(`bt_cm dut` / `bt_cm uart_dut` / `bt_rftest`)即可切到测试态，**无需向 FAE 申请**。

本文档以 **User.bin** 为主。

---

## 1. 编译配置

### 1.1 推荐基础工程

推荐以下开箱即用的基础工程：

| 测试场景 | 推荐基础工程 | 改动 |
|---|---|---|
| BT 信令测试(BR/EDR DUT 模式) | 如 [`example/bt/spp/`](../../../../example/bt/spp/) 等未关闭 `BT_RF_TEST` 的 BT 例程 | 直接编译，无需改 `proj.conf` |
| BT 非信令 / 单项透传(`bt_cm uart_dut` + SiFli_RfTool) | 同上 | 无需改动，直接使用 |
| BLE DTM(`bt_rftest bletx/blerx`) | 同上(BT 例程默认含 BLE controller) | 无需改动，直接使用 |
| 综测仪/PC 脚本直驱标准 HCI(开机即透传，不依赖 finsh 运行时切换) | [`example/bt/HCI_over_uart/`](../../../../example/bt/HCI_over_uart/) | 无需修改 `proj.conf`;但该例程不带 `bt_cm` 命令，也没有 BLE/BT 业务，只走标准 HCI |


### 1.2 Kconfig 开关一览

| 命令 / 功能 | 必需开启的 Kconfig | 代码位置 |
|---|---|---|
| `bt_cm` 命令 | `CONFIG_RT_USING_FINSH=y` + `CONFIG_BSP_BT_CONNECTION_MANAGER=y` | `bt_connection_manager.c:1937` |
| `bt_rftest` 命令 | `CONFIG_RT_USING_FINSH=y` + `CONFIG_BT_RF_TEST=y`(且非 SF32LB55X) | `bf0_bt_common.c:1209` |
| `bt_cm uart_dut` 内的 **UART 切换部分** | `CONFIG_BT_RF_TEST=y`(命令本身不被此开关控制，只是缺了它就不会调 `uart_ipc_path_change`) | `bt_connection_manager.c:1776` |
| 第 12.1 节 `<< enb DUT mode` 等 BT app trace | `CONFIG_BT_FINSH=y` | `bt_finsh/bts2_app_generic.c` |

> `BT_FINSH` 是 BT 协议栈调试 trace 总开关(`bts2_app_demo` / `bts2_app_menu` / `bts2_app_generic` 这套)，不要把它当成"启用 bt_cm / bt_rftest"的开关。

### 1.3 UART 路径切换的目标设备

只对 `bt_cm uart_dut` 及自定义调用 `uart_ipc_path_change()` 的入口有效：

```
# 默认就是 IPC_USE_CONSOLE_DEVICE=y, console UART 跑 finsh,切的也是它
# 想切到其他 UART(比如自定义产品里把测试口指向 uart2),改成:
CONFIG_IPC_USE_OWN_DEVICE=y
CONFIG_IPC_OWN_DEVICE_NAME="uart2"
```

---

## 2. BT Classic 信令测试

### 2.1 拓扑

DUT、综测仪、PC 控制端三者关系：

```{figure} assert/bt_signaling_test.png
:align: center

BT 信令测试拓扑
```

综测仪(MT88 系列、CMW 系列等)从空口扮演主设备，通过 LMP 命令驱动 DUT 进入信令测试态;PC 通过串口/GPIB 控制综测仪。DUT 自身只需要在协议栈层置好 DUT 允许位即可。

### 2.2 如何使用

| 选项 | 调用 |
|---|---|
| FinSH | `bt_cm dut`(只置允许位)/ `bt_cm uart_dut`(置允许位 + 切 UART) |
| API | `gap_enb_dut_mode_req(U16 tid)`，头文件 `gap_api.h` |

详细测试方法参见：[思澈SF32LB5xx芯片蓝牙信令测试指南](思澈SF32LB5xx芯片蓝牙信令测试指南.md)。

### 2.3 流程

```
1. 协议栈正常启动,BLE_POWER_ON_IND 已收到
2. host 显式开 scan(必要):
     bt_interface_set_scan_mode(1, 1);
     // 或 gap_wr_scan_enb_req(tid, 1, 1);
   gap_enb_dut_mode_req 不会自动开 inquiry/page scan,host 不开仪器搜不到 DUT。
3. 调 gap_enb_dut_mode_req(bts2_task_get_app_task_id()):
   - 下发 HCI_Enable_Device_Under_Test_Mode
   - controller 置"允许 DUT"标志,后续接受空口 LMP_test_control
4. console 看到 "<< enb DUT mode : <step_num>",且无 "<< failed to enb DUT mode!" 表示成功
5. 仪器(CMW500 信令模式)从空口连过来:
   inquiry → page → ACL connection → LMP_test_control → 进入测试态
6. 仪器跑测试套件
```

### 2.4 完整代码示例

```c
#include "bts2_app_inc.h"  /* 含 gap_api.h */

void enter_bt_signaling_dut(void)
{
    gap_enb_dut_mode_req(bts2_task_get_app_task_id());
    /* 等 BTS2MU_GAP_ENB_DUT_MODE_CFM 事件,可选 */
}
```

事件回调里(在你注册的 BT event handler 里)：

```c
case BTS2MU_GAP_ENB_DUT_MODE_CFM:
{
    BTS2S_GAP_ENB_DUT_MODE_CFM *cfm = (BTS2S_GAP_ENB_DUT_MODE_CFM *)data;
    if (cfm->res == 0)
        LOG_I("DUT mode enabled");
    else
        LOG_E("DUT mode enable failed: %d", cfm->res);
    break;
}
```

### 2.5 注意

- **只有"允许"作用**，不会让设备主动发包。真正发包要等仪器从空口发 LMP_test_control 触发。
- **HCI 命令通道完全正常**，调完之后还能继续发 HCI Reset、Read BD_Addr、LE DTM 命令。
- **BLE 完全不受影响**，BLE 控制器照常工作。
- **退出只能复位**，无逆操作。
- 调用前必须满足：`sifli_ble_enable()` 完成、`BLE_POWER_ON_IND` 已上来、且当前工作模式包含 BT Classic(双模或 BT-only，即 BT 协议栈已加载)。

---

## 3. BT Classic 非信令测试

### 3.1 拓扑

非信令测试不与 DUT 建立蓝牙链路，PC 通过串口发 HCI 命令控制 DUT，同时通过串口或 GPIB 控制综测仪：

```{figure} assert/non_signaling_test.png
:align: center

非信令测试拓扑
```

### 3.2 如何使用

| 选项 | 调用 |
|---|---|
| FinSH | `bt_rftest enter` → `bt_rftest bttx/btrx` → `bt_rftest btstop` → `bt_rftest exit` |
| API | `bt_enter_no_signal_dut_mode(bt_ns_test_mode_ctrl_cmd_t *)` |

头文件： `bf0_ble_common.h`

### 3.3 操作码和参数

```c
enum bt_test_operation_t {
    BT_TEST_OP_ENTER_TEST,    // 进非信令测试态
    BT_TEST_OP_EXIT_TEST,     // 退出
    BT_TEST_OP_TX_TEST,       // 开始发包(参数走 tx_para)
    BT_TEST_OP_RX_TEST,       // 开始收包(参数走 rx_para)
    BT_TEST_OP_STOP_TEST,     // 停止当前 TX/RX
};

typedef struct {
    uint8_t  channel;     // 0..78,BT 信道
    uint8_t  pkt_payload; // 0=PRBS9, 1=11110000, 2=10101010, 3=PRBS15, ...
    uint8_t  pkt_type;    // 0=ID, 1=NULL, 4=DM1, 5=DH5(常用), 14=2-DH5, 15=3-DH5
    uint8_t  pwr_lvl;     // 直接的 dBm 整数(典型 0..13);实际会量化到芯片 PA 支持的硬件档位,见 §7.1
    uint16_t pkt_len;     // payload 长度,DH5 最大 339
} bt_ns_test_mode_tx_para_t;

typedef struct {
    uint8_t channel;      // 0..78
    uint8_t pkt_type;     // 同上
} bt_ns_test_mode_rx_para_t;
```

### 3.4 FinSH 流程示例

```
msh > bt_rftest enter           # 进非信令测试态
msh > bt_rftest bttx            # ch8, DH5, 339B, PRBS9, pwr=10 (固定参数)
... 仪器测 TX power、频偏 (10~30 秒) ...
msh > bt_rftest btstop          # 停止 TX
msh > bt_rftest btrx            # ch0, DH1 收包
... 仪器从对端发包,DUT 计算 PER ...
msh > bt_rftest btstop
msh > bt_rftest exit            # 退出测试态
```

`bt_rftest` 子命令的测试参数(信道、包类型、长度、功率)在 SDK 内部为固定值，如需调整请参考 `SiFli-SDK/middleware/bluetooth/service/common/bf0_bt_common.c` 中 `bt_rftest()` 实现，自定义一组带参子命令。

### 3.5 完整 API 用法示例

> **`bt_enter_no_signal_dut_mode` 是异步的。** 函数内部 `sifli_msg_alloc` + `memcpy` + `sifli_msg_send` 后立即返回 `HL_ERR_NO_ERROR`，真正生效要等 `BT_NS_DUT_RSP` 事件 ack(实现见 `bf0_bt_common.c:793`)。下面 demo 在状态切换之间用事件等待保证顺序;若产线代码必须同步推进，可用一个 `rt_event` / `rt_sem` 在 `BT_NS_DUT_RSP` 回调里释放，或退而求其次用经验延时(典型 100~200ms 即够，但不保证)。

```c
#include "bf0_ble_common.h"

/* 在 BT event handler 里收 BT_NS_DUT_RSP,释放等待者(伪代码)*/
static rt_event_t s_bt_ns_evt;       /* 由模块初始化时 rt_event_create */
#define EVT_BT_NS_DONE  (1u << 0)

static void on_bt_ns_dut_rsp(bt_ns_test_mode_ctrl_rsp_t *rsp)
{
    /* 可按 rsp->op / rsp->status 进一步分发 */
    rt_event_send(s_bt_ns_evt, EVT_BT_NS_DONE);
}

static int wait_bt_ns_done(uint32_t timeout_ms)
{
    rt_uint32_t recved;
    return rt_event_recv(s_bt_ns_evt, EVT_BT_NS_DONE,
                         RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                         rt_tick_from_millisecond(timeout_ms), &recved);
}

void bt_nonsig_tx_test(uint8_t channel, uint16_t pkt_len, uint8_t pwr_lvl)
{
    bt_ns_test_mode_ctrl_cmd_t cmd;

    /* 1. 进测试态,等 ack */
    cmd.op = BT_TEST_OP_ENTER_TEST;
    bt_enter_no_signal_dut_mode(&cmd);
    if (wait_bt_ns_done(500) != RT_EOK) {
        LOG_E("ENTER timeout"); return;
    }

    /* 2. 配 TX 参数并启动 */
    cmd.op = BT_TEST_OP_TX_TEST;
    cmd.para.tx_para.channel     = channel;
    cmd.para.tx_para.pkt_type    = 5;       /* DH5 */
    cmd.para.tx_para.pkt_len     = pkt_len;
    cmd.para.tx_para.pkt_payload = 0;       /* PRBS9 */
    cmd.para.tx_para.pwr_lvl     = pwr_lvl;
    bt_enter_no_signal_dut_mode(&cmd);
    /* TX/RX 启动的 ack 同样经 BT_NS_DUT_RSP 上报,产线流程建议同样等 */
}

void bt_nonsig_stop_and_exit(void)
{
    bt_ns_test_mode_ctrl_cmd_t cmd;

    cmd.op = BT_TEST_OP_STOP_TEST;
    bt_enter_no_signal_dut_mode(&cmd);
    wait_bt_ns_done(500);

    cmd.op = BT_TEST_OP_EXIT_TEST;
    bt_enter_no_signal_dut_mode(&cmd);
    wait_bt_ns_done(500);
}
```

### 3.6 收包结果(只 RX 用)

事件 `BT_NS_DUT_RSP` 上来时，对应结构体 `bt_ns_test_mode_ctrl_rsp_t`：

```c
typedef struct {
    enum bt_test_operation_t  op;        // STOP_TEST 时返回 cnt
    uint8_t                    status;
    bt_ns_test_mode_rsp_para_t para;     // .stop_para.cnt = 总收包数
} bt_ns_test_mode_ctrl_rsp_t;
```

或者用更新的同步接口直接拿 PER + RSSI：

```c
extern int8_t bt_ns_rx_start(bt_ns_test_new_rx_para_t *rxpara,
                              bt_ns_test_new_rx_rslt_t *rst,
                              uint32_t delay_ms);

bt_ns_test_new_rx_para_t para = {
    .channel = 8, .pkt_type = 5, .pkt_len = 339, .pkt_payload = 0,
};
bt_ns_test_new_rx_rslt_t rst;
bt_ns_rx_start(&para, &rst, 2000);  /* 收 2s 后停 */
LOG_I("err_bit=%d total_bit=%d err_pkt=%d total_pkt=%d rssi=%d",
      rst.err_bit_num, rst.total_bit_num,
      rst.err_pkt_num, rst.total_pkt_num, rst.rssi);
```

### 3.7 注意

- 要用 **[SiFli_RfTool](../tools/SiFli_RfTool/SiFli_RfTool_UM.md)**。
- **必须先 ENTER 再 TX/RX，先 STOP 再切 TX/RX，最后 EXIT**。状态机不正确会出错。

---

## 4. BLE 非信令测试 (SIG DTM)

### 4.1 拓扑

所有 BLE RF 测试通过 DUT 与综测仪之间的串口传 3 条 HCI 命令(`LE_Transmitter_Test` / `LE_Receiver_Test` / `LE_Test_End`)完成。固件侧只有这一条路径，即 UART 切 HCI 透传：

```{figure} assert/ble_signaling_test.png
:align: center

BLE DTM 测试拓扑
```

DUT 与综测仪通过串口连接，综测仪发 HCI 命令控制 DUT 进入 TX/RX；PC 上的 RF 测试工具同时控制 DUT 与综测仪的收发。

### 4.2 如何使用

| 选项 | 调用 |
|---|---|
| FinSH | `bt_rftest bletx` / `bt_rftest blerx` / `bt_rftest btstop`(共用 stop) / `bt_rftest exit` |
| API | `ble_enter_dut_mode(ble_dut_mode_t *)` |
| 标准 HCI(经 UART 透传) | `HCI_LE_Transmitter_Test`(0x201E)/ `HCI_LE_Receiver_Test v2`(0x2033)/ `HCI_LE_Test_End`(0x201F) |

头文件： `bf0_ble_common.h`

### 4.3 数据结构

```c
typedef struct {
    uint8_t operation;       // GAPM_LE_TEST_TX_START / RX_START / STOP
    uint8_t channel;         // 0..39 (0=2402MHz, 19=2440MHz, 39=2480MHz)
    uint8_t tx_data_length;  // 仅 TX, 0..255
    uint8_t tx_pkt_payload;  // 仅 TX, gap_pkt_pld_type
    uint8_t phy;             // 1=1M, 2=2M, 3=Coded
    uint8_t modulation_idx;  // 仅 RX, 0=standard, 1=stable
} ble_dut_mode_t;

enum gap_pkt_pld_type {
    GAP_PKT_PLD_PRBS9                = 0,  // 默认,产线常用
    GAP_PKT_PLD_REPEATED_11110000    = 1,  // 测调制 Δf1/Δf2
    GAP_PKT_PLD_REPEATED_10101010    = 2,  // 测载波频偏
    GAP_PKT_PLD_PRBS15               = 3,
    /* 还有全 0、全 1、F0/0F、AA/55 等 */
};
```

### 4.4 通过 API 直发

> **`bt_enter_no_signal_dut_mode` 与 `ble_enter_dut_mode` 都是异步的**(实现见 `bf0_bt_common.c:769` / `:793`)。两条命令分别投递到 COMMON / GAPM 两个不同任务队列，**不能假设按调用顺序串行完成**。完成事件：
> - ENTER 完成 → `BT_NS_DUT_RSP`(`op == BT_TEST_OP_ENTER_TEST`)
> - TX 启动完成 → `BLE_DUT_TX_START_CNF`
> - RX 启动完成 → `BLE_DUT_RX_START_CNF`
> - 测试结束(STOP)→ `BLE_DUT_END_IND`(含 `nb_packet_received`)
>
> 产线/产品代码必须等对应事件再进入下一步。下面 demo 用 `rt_event` 串起整个时序。

```c
#include "bf0_ble_common.h"

static rt_event_t s_dut_evt;
#define EVT_NS_ENTER_DONE   (1u << 0)
#define EVT_BLE_TX_START    (1u << 1)
#define EVT_BLE_END         (1u << 2)

/* 在 BT/BLE event handler 里 set 对应位 */
static void on_bt_ns_dut_rsp(bt_ns_test_mode_ctrl_rsp_t *rsp) {
    if (rsp->op == BT_TEST_OP_ENTER_TEST)
        rt_event_send(s_dut_evt, EVT_NS_ENTER_DONE);
}
static void on_ble_dut_tx_cnf(ble_test_tx_start_cnf_t *cnf) {
    rt_event_send(s_dut_evt, EVT_BLE_TX_START);
}
static void on_ble_dut_end(ble_dut_end_ind_t *ind) {
    LOG_I("DTM end, nb_packet_received=%u", ind->nb_packet_received);
    rt_event_send(s_dut_evt, EVT_BLE_END);
}

static int wait_evt(uint32_t bits, uint32_t timeout_ms)
{
    rt_uint32_t recved;
    return rt_event_recv(s_dut_evt, bits,
                         RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                         rt_tick_from_millisecond(timeout_ms), &recved);
}

/* 1. 进测试态(BT_TEST_OP_ENTER_TEST 是 BLE/BT 共用入口)*/
bt_ns_test_mode_ctrl_cmd_t enter = { .op = BT_TEST_OP_ENTER_TEST };
bt_enter_no_signal_dut_mode(&enter);
if (wait_evt(EVT_NS_ENTER_DONE, 500) != RT_EOK) {
    LOG_E("ENTER timeout"); return;
}

/* 2. 启动 TX:ch19 (2440MHz), 37 字节, PRBS9, 1M PHY */
ble_dut_mode_t tx = {
    .operation       = GAPM_LE_TEST_TX_START,
    .channel         = 19,
    .tx_data_length  = 37,
    .tx_pkt_payload  = GAP_PKT_PLD_PRBS9,
    .phy             = 1,
    .modulation_idx  = 0,
};
ble_enter_dut_mode(&tx);
if (wait_evt(EVT_BLE_TX_START, 500) != RT_EOK) {
    LOG_E("TX start timeout"); return;
}

/* 3. 让 controller 持续发包 2s,然后停止 */
rt_thread_mdelay(2000);
ble_dut_mode_t stop = { .operation = GAPM_LE_TEST_STOP };
ble_enter_dut_mode(&stop);
wait_evt(EVT_BLE_END, 500);   /* 收到 BLE_DUT_END_IND 表示已停 */
```

### 4.5 通过 FinSH 跑(参数固定 ch0)

```
msh > bt_rftest enter
msh > bt_rftest bletx           # ch0 默认 37B PRBS9 1M PHY
... 仪器测 ...
msh > bt_rftest btstop
msh > bt_rftest exit
```

`bt_rftest blerx` 同理。RX 完成后通过 `BLE_DUT_END_IND` 事件返回 `nb_packet_received`。

### 4.6 通过 HCI 透传(综测仪直驱)

如果用 CMW500 这种标准仪器，不调 API，直接发 HCI：

```
1. msh > bt_cm uart_dut       # 切 UART 为 HCI 透传
2. 此后仪器/脚本通过该 UART 发标准 HCI 命令:

   01 1E 20 03 13 25 00       LE_Transmitter_Test    ch19, 37B, PRBS9
   01 33 20 03 13 01 00       LE_Receiver_Test v2    ch19, 1M PHY, modulation=standard
   01 1F 20 00                LE_Test_End → 返回 packet_count

3. 退出:复位芯片,或调用 `uart_ipc_path_revert()` 软切回(见 §11.5)
```

### 4.7 事件回调

```c
case BLE_DUT_TX_START_CNF: {
    ble_test_tx_start_cnf_t *cnf = (ble_test_tx_start_cnf_t *)data;
    LOG_I("TX start, status=%d", cnf->status);
    break;
}
case BLE_DUT_RX_START_CNF: {
    ble_test_rx_start_cnf_t *cnf = (ble_test_rx_start_cnf_t *)data;
    LOG_I("RX start, status=%d", cnf->status);
    break;
}
case BLE_DUT_END_IND: {
    ble_dut_end_ind_t *ind = (ble_dut_end_ind_t *)data;
    LOG_I("Test end, packet_count=%d", ind->nb_packet_received);
    /* TX 模式下 packet_count 永远 0,这是 spec 行为 */
    break;
}
```

### 4.8 注意

- **不需要 `gap_enb_dut_mode_req`**：BLE DTM 是非信令的，不需要置 BT Classic 的"允许"标志。
- **TX 测试 packet_count 总是 0**(spec 规定)，只有 RX 才有意义。
- BLE 和 BT Classic 共射频，**不能并行**。

---

## 5. 单项测试 (SiFli_RfTool)

研发阶段 RF 板级调优、法规预测试用。工程师通过 PC 端 **[SiFli_RfTool](../tools/SiFli_RfTool/SiFli_RfTool_UM.md)** 向 DUT 串口发送底层控制指令，精确控制射频行为(单载波、特定调制信号、固定信道/功率)，配合频谱仪、功率计等仪器逐项测量。

**工具位置**：`tools/SiFli_RfTool/SiFli_RfTool.exe`，可在Windows直接运行。

详细的单项测试步骤详见[思澈SF32LB5xx芯片蓝牙单项测试指南](思澈SF32LB5xx芯片蓝牙单项测试指南.md)。
### 5.1 入口

| 选项 | 调用 |
|---|---|
| FinSH | `bt_cm uart_dut`(置 DUT 允许位 + 切 UART 为 HCI 透传) |
| API | 与第 2 章相同(`gap_enb_dut_mode_req` + `uart_ipc_path_change`) |

固件侧切完后，SiFli_RfTool 通过串口发底层 HCI / vendor 命令逐项控制。固件本身不需要写额外的逻辑，只需要把 UART 路径让出来。

### 5.2 用法

DUT 发 `bt_cm uart_dut` → PC 上打开 SiFli_RfTool → 选串口和波特率 → 在工具界面里逐项发命令测试。串口命令也可以用 **SiFli_Trace** 或其他通用串口工具触发(信令测试模式下常用)。

---

## 6. RF 调试串口与波特率

各芯片默认 RF 调试串口与引脚映射(默认波特率 **1000000**)：

| 芯片型号 | 接口类型 | 引脚 | UART 通道(TX / RX) | 备注 |
|---|---|---|---|---|
| SF32LB551 | RF 调试串口 | PA49、PA51 | UART1: TX(PA49) / RX(PA51) | |
| SF32LB551 | 单载波测试 | PB45、PB46 | UART3: TX(PB45) / RX(PB46) | |
| SF32LB555 | RF 调试串口 | PA17、PA19 | UART1: TX(PA17) / RX(PA19) | |
| SF32LB555 | 单载波信号口 | PB45、PB46 | UART3: TX(PB45) / RX(PB46) | |
| SF32LB561 / SF32LB563 | RF 调试串口 | PA17、PA18 | UART1: TX(PA17) / RX(PA18) | |
| SF32LB567 | RF 调试串口 | PA34、PA30 | UART1: TX(PA34) / RX(PA30) | |
| SF32LB52x | RF 调试串口 | PA19、PA18 | UART1: TX(PA19) / RX(PA18) | 与软件烧录口共用引脚 |
| SF32LB58x | RF 调试串口 | PA31、PA32 | UART1: TX(PA31) / RX(PA32) | |

> SF32LB55X 系列由于历史原因，**单载波测试**需切到 UART3。其他芯片所有 RF 测试统一在 UART1。

综测仪(MT88 系列、CMW 系列)在 HCI over UART 配置里要匹配此波特率(默认 1Mbps)。如果链路不稳可以降到 921600 / 460800 / 115200，综测仪侧同步降。

---

## 7. 射频功率档位与配置

### 7.1 支持的功率档位

芯片 PA 工艺上稳定输出的几个档位(其他 dBm 值会被内部量化到最近档位)：

| 档位 | 备注 |
|---|---|
| 0 dBm | |
| 3 dBm | |
| 6 dBm | |
| 9 dBm | |
| 13 dBm | |
| 19 dBm | **仅BR&BLE支持19dbm** |

> 软件 API(`pwr_lvl`、Kconfig `BT_TX_POWER_VAL_*`)接受任意整数 dBm，不是档位索引;运行时由 controller 量化到最接近的硬件档。

### 7.2 menuconfig 配置

```
(Top) → SiFli SDK configuration → Board Config → Select BT RF TX power
```

Kconfig 字段都是 **`int` 类型，直接填 dBm 整数**(参考 [customer/boards/Kconfig:103+](D:/OpenSiFli/SiFli-SDK/customer/boards/Kconfig#L103))。新平台(52X/56X/58X)取值范围 0~13 dBm;老平台 BLE 取值范围 -10~10 dBm。

| 选项 | 含义 |
|---|---|
| Select BLE MAX TX power | BLE 最大发送功率(dBm 整数) |
| Select MINIMUM TX power | BR/EDR/BLE 最小发送功率(dBm 整数) |
| Select classic BT MAX TX power | BR/EDR 最大发送功率(dBm 整数);若该值小于 BLE 最大功率，则 BR/EDR 最大功率与 BLE 一致，否则取此值 |

非信令测试 TX 命令的 `pwr_lvl` 字段语义与上面这些 Kconfig 字段一致，**直接的 dBm 数值，不是档位索引**。

---

## 8. HCI 命令速查

下面是常用的 BT/BLE RF 测试相关 HCI 命令一览：

```{figure} assert/hci_cmd.png
:align: center

HCI 命令集
```

部分关键 opcode：

| 命令 | OGF/OCF | 完整 opcode (LE) |
|---|---|---|
| HCI Reset | 0x03 / 0x0003 | `01 03 0C 00` |
| HCI Read BD_Addr | 0x04 / 0x0009 | `01 09 10 00` |
| HCI Enable Device Under Test Mode | 0x06 / 0x0003 | `01 03 18 00` |
| LE Receiver Test v2 | 0x08 / 0x0033 | `01 33 20 03 ch phy mod` |
| LE Transmitter Test | 0x08 / 0x001E | `01 1E 20 03 ch len pld` |
| LE Test End | 0x08 / 0x001F | `01 1F 20 00` |

更详细的命令参数和响应字段参考 [Bluetooth Core Specification](https://www.bluetooth.com/specifications/specs/) Volume 4 Part E (HCI)。SiFli 私有 vendor 命令(BT 非信令测试用)不属于 SIG 标准，封装在第 3 章的 API 内，无需直接拼字节。

---

## 9. 三种测试态不能并行

LCPU 只有一套射频前端，任何时刻只能在一种测试态里：

```
正常运行态  ──[gap_enb_dut_mode_req]──→  BT 信令 DUT 准备态
            ──[bt_enter_no_signal_dut_mode(ENTER+TX)]──→  BT 非信令测试态
            ──[ble_enter_dut_mode(TX_START)]──→  BLE DTM 测试态
```

**切换原则**：

- 进入任意测试态前，如果之前是其他测试态，必须先 EXIT 退干净
- BT 信令 DUT 和 BT 非信令互斥，要切先 EXIT
- BLE DTM 和 BT 非信令互斥(共用 ENTER 状态)，要切先 EXIT 再 ENTER

---

## 10. 在产品里自定义测试入口

如果你想在自己的产品里增加"通过私有协议命令触发 RF 测试"的能力(比如产线工位发一条命令模组就进 DUT)，可以参考下面两个模板。

**模板 A：置 BT Classic DUT 允许位，UART 不动。退出靠复位。**

```c
void product_enter_bt_signaling_dut(void)
{
    gap_enb_dut_mode_req(bts2_task_get_app_task_id());
    /* 业务 UART 继续工作,仪器从空口连接触发测试 */
}
```

**模板 B：把业务 UART 切成 HCI H4 透传给 CMW500 用。**

调用方运行的线程之后**不能再回主循环访问该 UART**，否则下一次 `rt_device_read(NULL)` 会断言。必须释放 UART + 永久挂起当前线程：

```c
extern void uart_ipc_path_change(void);

void product_enter_hci_passthrough(void)
{
    /* 1. 给上位机回个 ACK 之类的提示,然后等 TX 冲完 */
    rt_thread_mdelay(50);

    /* 2. 释放业务 UART:停接收线程、解绑 rx_indicate、close device */
    my_app_release_uart();

    /* 3. SDK 接管该 UART,启 fwd2mb / fwd2uart 转发线程 */
    uart_ipc_path_change();

    /* 4. 永久挂起当前线程,避免它返回后又访问已 close 的 UART */
    rt_thread_suspend(rt_thread_self());
    rt_schedule();
    /* never returns */
}
```

`my_app_release_uart()` 的实现要做的事：

- 让你的接收线程感知到"该退出了"(标志位或 sem)
- `rt_device_set_rx_indicate(dev, RT_NULL)`
- `rt_device_close(dev)`

发送类 API(产品自己的 `xxx_uart_send`)在被释放后必须有 NULL 检查，否则后续事件上报会触发空指针。

### 10.1 命令 handler 挂在哪

挂载点取决于你产品的命令系统：

| 命令系统 | 挂载方式 |
|---|---|
| 标准 finsh / MSH | `MSH_CMD_EXPORT(my_dut_cmd, ...)` 直接导出 |
| 自定义文本协议(AT、私有 ASCII 等) | 加到协议解析器的命令分发表里;handler 跑在协议解析线程 |
| 二进制私有协议 | 加到 opcode 分发 switch;handler 跑在协议解析线程 |
| BLE/SPP 远程触发 | 在 GATT write callback 或 SPP data 回调里调 |

**注意 handler 的运行线程上下文**：模板 B 的 `rt_thread_suspend(rt_thread_self())` 挂的是**调用 `uart_ipc_path_change()` 这条指令所在的线程**。如果你的 handler 跑在协议解析线程上(典型场景)，挂的就是这条线程，之后该线程上的所有后续处理逻辑都不再运行，这是预期行为。

如果不希望整个协议处理线程被冻住，可以**派生一个临时一次性线程**专门跑模板 B 的逻辑，在那个临时线程里挂起;但这种做法收益不大，因为切完 UART 后协议解析线程也没活可干了(UART 不归它了)，挂哪条都一样。

### 10.2 调用前置条件

模板 A/B 都需要 **BT 协议栈已就绪**：

- `sifli_ble_enable()` 已调用，**且** `BLE_POWER_ON_IND` 事件已上来
- 当前工作模式包含 BT Classic(双模或 BT-only)时，`bt_interface_set_scan_mode(1, 1)` 已生效

如果产品上电流程里这些步骤异步完成，**handler 内必须挡一道**：

```c
extern app_env_t *app_get_env(void);   /* 你产品里维护栈状态的方式 */

static void product_enter_dut(void)
{
    if (!app_get_env()->is_power_on)
    {
        /* 协议栈还没起来,直接调会石沉大海 */
        respond_error("stack not ready");
        return;
    }
    gap_enb_dut_mode_req(bts2_task_get_app_task_id());
    respond_ok();
}
```

---

## 11. HCI 透传后的数据通路转换

模板 B 调完 `uart_ipc_path_change()` 之后，该 UART 上发生的变化要心里有数，否则容易踩坑：

### 11.1 发生了什么

| 切换前 | 切换后 |
|---|---|
| RX 数据由你的协议解析线程消费 | RX 数据由 SDK 的 `fwd2mb` 线程读取，直接送 mailbox 到 LCPU |
| TX 由你的协议代码 `rt_device_write` | TX 由 SDK 的 `fwd2uart` 线程从 mailbox 取数据写入 |
| `rx_indicate` 是你的回调 | `rx_indicate` 被 SDK 替换成 `hci_trans_uart_rx_ind` |
| log 输出经 ulog → console UART(可能就是这条) | 如果切的是 console UART，`log_pause(true)` 已被调，log 静默 |
| UART 上是文本协议 / 私有协议 | UART 上是 **HCI H4 二进制流** |

### 11.2 切换后必须避免的事

- **不要再调任何之前的 `xxx_uart_send` / `xxx_uart_send_str`**：即使加了 NULL 检查不崩溃，这些字节如果误写入 UART 也会被 SDK 误当成 HCI command 转发到 LCPU。LCPU 按 H4 帧格式解析后会得到非法 opcode 或非法长度，轻则返回 `Unknown HCI Command (0x01)` / `Invalid HCI Parameters (0x12)`，重则导致后续合法 HCI 命令的字节流被前一个错误帧的 `param_len` 截断，出现命令丢失或响应错位
- **不要再触发任何 log 打印到这条 UART**：`log_pause(true)` 暂停了 ulog 框架，但如果你绕过 ulog 直接 `rt_kprintf`，字节会泄漏到 HCI 流里污染上位机
- **不要往这条 UART 再上报事件**：断开/连接事件、心跳等周期性输出必须停掉

### 11.3 切换后能做的事

- **从这条 UART 收 HCI command**(由仪器/PC 脚本发，opcode 编码与第 8 节表一致，`OGF/OCF` 拼为 16-bit 值)：
  - 标准 BLE DTM：`HCI_LE_Transmitter_Test`(`0x08/0x001E`)、`HCI_LE_Receiver_Test v2`(`0x08/0x0033`)、`HCI_LE_Test_End`(`0x08/0x001F`)
  - 通用 HCI：`HCI_Reset`(`0x03/0x0003`)、`HCI_Read_BD_Addr`(`0x04/0x0009`)等
  - SiFli 私有 BT 非信令 vendor：见第 3 章
- **从这条 UART 看 HCI event**：每条 command 应当返回 Command Complete event(type `0x04`，event code `0x0E`)

### 11.4 H4 帧格式速查

发送(host → controller)：

```
| type | opcode_lo | opcode_hi | param_len | params... |
| 0x01 |  XX       |    XX     |    XX     |   ....    |
```

接收(controller → host)：

```
| type | event_code | param_len | params... |
| 0x04 |    0x0E    |    XX     |   ....    |  ← Command Complete
```

opcode 是小端 16 位，例如 `LE_Transmitter_Test` 是 `0x201E`，串行格式 `1E 20`。

### 11.5 切回业务态

SDK 提供软逆操作 `uart_ipc_path_revert()`(实现见 `bluetooth_config.c:1634`)，可在不复位的情况下还原：

```c
extern void uart_ipc_path_revert(void);

uart_ipc_path_revert();
/* 此后:
   - rx_indicate / open_flag 还原为切换前的值
   - log_pause(false) 恢复 ulog 输出
   - trans_en=0 通知 fwd2mb / fwd2uart 转发线程停止搬运
   - LCPU 低功耗活动锁释放
*/
```

**注意软切回的局限**：

- `uart_ipc_path_revert` 不会销毁 `f2mb_th` / `f2uart_th` 转发线程，这两条线程持续存在，只是因 `trans_en=0` 不再搬数据
- 不会调用 `rt_device_close`，UART 设备 ref_cnt 没还原
- 后续业务代码若要重新接管 UART(`rt_device_open` / 设新 `rx_indicate`)，需要确保自身重入逻辑没有依赖"设备此前未被打开"

如果产线流程要求"测完 RF 后彻底回到业务态、不留 SDK 后台线程"，仍建议走复位路径;`uart_ipc_path_revert` 适合调试/开发时反复切换，不必每次重启硬件。

---

## 12. 验证你的实现是否正确

按下面顺序验证

### 12.1 命令本身能调通

发命令 → 看 console 是否有 `BTS2MU_GAP_ENB_DUT_MODE_CFM` handler 打的这一行(以 BT 信令 DUT 为例)：

```
D/btapp_ge:  << enb DUT mode : <step_num>
```

实现位置：[bts2_app_generic.c:1738-1752](D:/OpenSiFli/SiFli-SDK/middleware/bluetooth/service/bt/bt_finsh/bts2_app_generic.c#L1738-L1752)。`<step_num>` 是步骤计数，具体数字依实现而定，不应当作固定值校验。**判定标准是：出现这条日志，且没有伴随出现 `<< failed to enb DUT mode!`**(后者由 `res != BTS2_SUCC` 触发)。

**没看到任一条**就是命令没走到 LCPU，往回查 `BLE_POWER_ON_IND` 时序、协议栈是否启动、handler 是否真的执行、`CONFIG_BT_FINSH=y` 是否打开(否则不会有这条 trace)。



### 12.2 scan mode 状态确认

`gap_enb_dut_mode_req` 不会自动开 scan，DUT 是否被仪器搜到取决于 host 此前是否调过 `bt_interface_set_scan_mode(1, 1)` 或 `gap_wr_scan_enb_req(tid, 1, 1)`。读出当前缓存值确认：

```c
uint8_t scan = bt_interface_get_current_scan_mode();
LOG_I("scan mode = %d", scan);
/* 0 = 都没开;1 = 仅 inquiry;2 = 仅 page;3 = inquiry+page 都开 */
```

如果发 DUT 命令前 host 已设过 scan_mode=3，这里期望读到 3;否则按实际 scan 状态返回。读到非 3 就说明你的初始化流程没把 scan 开起来，先补 `bt_interface_set_scan_mode(1, 1)` 再发 DUT 命令。

### 12.3 HCI 透传链路验证(模板 B)

切完 UART 后，用 PC 串口工具(切到 hex 模式)发 HCI Reset：

```
TX: 01 03 0C 00
RX: 04 0E 04 XX 03 0C 00
```

收到回应说明 HCI 透传链路通了。`XX` 是 num_hci_cmd_packets 字段，任意值;关键是开头 `04 0E` (Command Complete event)和末尾 `03 0C 00`(关联到 HCI_Reset opcode + status 0)。

进一步发 LE Transmitter Test：

```
TX: 01 1E 20 03 13 25 00      LE_Transmitter_Test ch19, 37B, PRBS9
RX: 04 0E 04 XX 1E 20 00       Command Complete, status 0
```

控制器开始在 ch19 持续发包，此时拿频谱仪或 CMW500 能看到 2440MHz 上有信号。

### 12.4 完整 RF 指标验证

接 CMW500(或 R&S CBT、Anritsu MT8852B 等同类)，按 SIG DTM 标准跑测试套件，看 TX power、频偏、PER 是否在 spec 范围内。这一步在仪器上完成，不需要你写代码。

---

## 13. 完整集成流程示例

假设你已有一个 RT-Thread + SiFli SDK 的工程，自己跑一套命令系统(本节通用，不限于 finsh / 自定义协议)。下面给一个完整集成"DUT 测试命令"的步骤，以**模板 A(信令 DUT，UART 不切)** 为例。

### Step 1：proj.conf 加配置

```
CONFIG_BT_FINSH=y
CONFIG_RT_USING_FINSH=y
```

### Step 2：在你的命令处理代码里加 handler

```c
#include "bts2_app_inc.h"     /* 含 gap_api.h */

static int s_dut_armed = 0;

static void cmd_dut_handler(...)
{
    if (!app_stack_ready())
    {
        respond_error("stack not ready");
        return;
    }

    if (s_dut_armed)
    {
        respond_error("already in DUT");
        return;
    }

    gap_enb_dut_mode_req(bts2_task_get_app_task_id());
    s_dut_armed = 1;
    respond_ok();
}
```

`s_dut_armed` 标志位用于阻止重复调用造成 LCPU 状态混乱。

### Step 3：挂载到命令分发

按你工程的命令系统：
- finsh：`MSH_CMD_EXPORT(cmd_dut_handler, enter BT signaling DUT mode)`
- 命令表分发：加一行 `{ "DUT", cmd_dut_handler }`
- BLE/SPP 远程触发：在收到特定 opcode 时调用

### Step 4：重编重烧 + 上电

确认上电流程跑完，业务命令(任意一条已有命令)能正常响应，以此证明协议栈和命令系统已就绪。

### Step 5：发 DUT 命令

收到 `OK` 响应后立刻看 console 日志，期望看到 12.1 节描述的 `<< enb DUT mode : <step_num>`，且没有伴随 `<< failed to enb DUT mode!`。

### Step 6：接仪器跑测试

CMW500 配 Bluetooth Signaling 测试模块，DUT Connection 选 BD Address(用 `Read_BD_Addr` 读出来填进去)，启动测试套件。仪器自动走 inquiry → page → ACL → LMP_test_control，跑完整 BQB 测试矩阵。

### Step 7：退出测试

复位芯片即可。

---

如果做的是模板 B(HCI 透传)，Step 1 还要加 IPC UART 配置(见第 1 章);Step 2 的 handler 内容换成模板 B 代码 + 实现 `my_app_release_uart()`;Step 5 之后改用 12.3 节的 PC 脚本验证;Step 6 直接接仪器到那条切走的 UART(不需要走 BT Address 流程，仪器直接发 HCI 命令)。

---

## 14. 故障排查

| 现象 | 原因 | 解法 |
|---|---|---|
| 调 `gap_enb_dut_mode_req` 没反应 | 协议栈没起来 | 等 `BLE_POWER_ON_IND` 后再调 |
| `<< enb DUT mode` 日志没出现 | LCPU 没收到命令 | 检查 mailbox / sifli_msg_send 是否正常 |
| `bt_rftest enter` 后 `bttx` 报 "please enter test mode first" | 状态机被打乱 | 重新 reset 走流程 |
| `bt_cm uart_dut` 后没任何 HCI 响应 | `CONFIG_BT_RF_TEST=y` 没开，该命令分支不会调用 `uart_ipc_path_change` | 查 `.config`，确认 `BT_RF_TEST=y`;或绕过 finsh，直接调 `uart_ipc_path_change()` API |
| `uart_ipc_path_change` 后 controller 收不到命令 | UART 还被原协议栈线程占着 | 切之前先 close device + 解绑 rx_indicate |
| 切完 UART 后 RX 线程断言 `dev != NULL` | 调用线程 return 后回到主循环又访问 device | 调用方必须 `rt_thread_suspend(rt_thread_self())` 永久挂起 |
| HCI 命令偶尔丢字节 | 1Mbps 在某些 USB-UART 上不稳 | 降到 921600 或 460800 |
| BLE TX 测试后读 packet_count 是 0 | spec 就是这样 | 只 RX 测试才有 packet_count |
| 测 BT 非信令时 TX 功率不符合预期 | `pwr_lvl` 是 dBm 整数(典型 0..13)，不是档位索引 | 直接填目标 dBm 值，运行时由 controller 量化到 PA 支持的硬件档位(见 §7.1);默认填 10 |

---

## 15. 关键源码位置

| 模块 | 文件 |
|---|---|
| BT 信令 DUT API | `gap_api.h`(声明)，实现在 BT 协议栈 lib(.a)里 |
| BT 非信令 / BLE DTM API | `bf0_ble_common.h` / `bf0_bt_common.c` |
| `bt_cm dut` / `bt_cm uart_dut` 实现 | `service/bt/bt_cm/bt_connection_manager.c` 1750–1778 行 |
| `bt_rftest` 实现 | `service/common/bf0_bt_common.c` 979–1210 行 |
| `uart_ipc_path_change` 实现 | `service/internal/bluetooth_config.c` 1539+ 行 |
| HCI 透传线程 / IPC mailbox | 同上文件 `hci_forward_to_mb_entry` / `hci_forward_to_uart_entry` |

---

## 16. 参考例程与文档

- BT 信令 / 非信令 / 单项 / DTM 测试基础例程：[`example/bt/`](../../../../example/bt/) 下任意 BT 例程，如 [`example/bt/spp/`](../../../../example/bt/spp/)
- 开机即 HCI 透传例程(综测仪/PC 脚本直驱)：[`example/bt/HCI_over_uart/`](../../../../example/bt/HCI_over_uart/)
- SiFli_RfTool 工具本体：`tools/SiFli_RfTool/SiFli_RfTool.exe`(SDK 自带，免安装)
- SiFli_RfTool 用户手册： [SiFli_RfTool_UM](../tools/SiFli_RfTool/SiFli_RfTool_UM.md)
- 单项测试与综测仪操作指南： [思澈 SF32LB5xx 芯片蓝牙单项测试指南](思澈SF32LB5xx芯片蓝牙单项测试指南.md)
