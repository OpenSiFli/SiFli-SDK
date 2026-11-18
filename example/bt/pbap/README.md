# BT PBAP client 示例

源码路径：example/bt/pbap

{#Platform_bt_pbap}
## 支持的平台
+ eh-lb52x
+ eh-lb56x
+ eh-lb58x
+ sf32lb52-lcd系列
+ sf32lb56-lcd系列
+ sf32lb58-lcd系列

## 概述

本例程演示 PBAP client 连接手机并访问手机电话簿，包含：
+ PBAP client 连接和断开
+ 设置电话簿仓库和电话簿对象
+ 拉取电话簿
+ 根据电话号码查询联系人
+ 根据 vCard handle 拉取联系人信息
+ 通过异步通知处理 PBAP 操作结果

## 例程的使用

例程启动后默认使能蓝牙，并通过 FINSH 命令控制 PBAP client。运行前请将手机与开发板配对，并在手机请求时允许共享联系人。

1. 连接手机

通过手机的蓝牙地址发起 PBAP 连接，`auth` 为可选认证参数，取值为 `0` 或 `1`：

```text
pbap_cmd connect 11:22:33:44:55:66 [0|1]
```

连接状态通过 PBAP 异步通知打印。也可以输入以下命令查看命令帮助：

```text
pbap_cmd help
```

2. 断开手机

```text
pbap_cmd disconnect 11:22:33:44:55:66
```

3. 设置电话簿路径

`set` 命令支持 `local`、`sim1`、`current` 三种仓库，以及 `pb`、`ich`、`och`、`mch`、`cch` 五种电话簿对象：

```text
pbap_cmd set local pb
pbap_cmd set sim1 ich
```

| 参数 | 含义 |
| --- | --- |
| `pb` | 主要电话簿 |
| `ich` | 已接来电记录 |
| `och` | 已拨电话记录 |
| `mch` | 未接来电记录 |
| `cch` | 综合通话记录 |

4. 拉取电话簿

```text
pbap_cmd pull_pb
pbap_cmd pull_pb 100
```

可选参数为最大联系人数量，默认值为 `65535`。联系人信息、联系人总数和操作完成结果通过异步通知输出。

5. 根据号码查询联系人

```text
pbap_cmd list 10086
```

`list` 使用 PBAP 客户端的号码查询接口，不是完整拉取电话簿的命令。

6. 拉取 vCard

```text
pbap_cmd vcard 123
```

命令返回值表示请求是否已被接受，具体结果需要查看串口输出的 PBAP 异步完成通知或结果事件。

### 硬件需求

运行该例程前，需要准备：
+ 一块本例程支持的开发板（[支持的平台](#Platform_bt_pbap)）。
+ 一部支持 PBAP 的蓝牙手机。
+ 一条连接开发板的串口，用于输入 FINSH 命令和查看日志。

### menuconfig配置

1. 使能蓝牙（`BLUETOOTH`）：
    - 路径：Sifli middleware → Bluetooth
    - 开启：Enable bluetooth
        - 宏开关：`CONFIG_BLUETOOTH`
        - 作用：使能蓝牙功能
2. 使能 PBAP client：
    - 路径：Sifli middleware → Bluetooth → Bluetooth service → Classic BT service
    - 开启：Enable BT finsh（可选）
        - 宏开关：`CONFIG_BT_FINSH`
        - 作用：使能 FINSH 命令行，用于控制蓝牙
    - 开启：Manually select profiles
        - 宏开关：`CONFIG_BT_PROFILE_CUSTOMIZE`
        - 作用：手动选择使能的配置文件
    - 开启：Enable PBAP client
        - 宏开关：`CONFIG_CFG_PBAP_CLT`
        - 作用：使能 PBAP client 功能

### 编译和烧录

在 SDK 根目录执行 `set_env.bat` 设置编译环境，然后运行：

```bat
cd example\bt\pbap\project
scons --board=<board> -j32
```

切换到 `project/build_<board>` 目录，运行生成的下载脚本进行烧录。详细步骤请参考[快速入门](/quickstart/get-started.md)。

## 例程的预期结果

例程启动后，串口可以输入 `pbap_cmd` 命令。开发板连接支持 PBAP 的手机后，能够设置电话簿路径、拉取电话簿、查询联系人并通过异步日志看到操作结果。

## 异常诊断

- 连接失败时，请确认手机已经配对，并检查手机是否允许共享联系人。
- 电话簿操作失败时，请确认 PBAP 连接已经建立，并已通过 `set` 设置有效路径。
- `list` 的电话号码不能为空，`vcard` 的 handle 长度不能超过 25 个字符。
- PBAP 操作是异步的，FinSH 命令返回成功只表示请求已提交，请继续查看 PBAP 结果日志。

## 参考文档

## 更新记录
|版本 |日期   |发布说明 |
|:---|:---|:---|
|0.0.1 |11/2026 |初始版本 |
| | | |