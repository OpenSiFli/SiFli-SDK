# HCI log 抓取方法

本文介绍蓝牙 HCI log 的抓取方法，包括：在开发板上打开 HCI log、用串口工具(SifliTrace)跟踪 HCI log、用 Frontline 解析 HCI log，以及在手机上抓取 HCI log。

---

## 1. 开发板上如何打开 HCI log

先确保工程配置下开启以下两个宏开关：

```
CONFIG_HCI_ON_LOG=y
CONFIG_LOG_ON_CONSOLE=y
```

HCI log 的开关保存在 NVDS 中，在串口终端(finsh/msh)中输入：

```
nvds update hci_log 1
```

- `1` 表示打开 HCI log，`0` 表示关闭。
- 该命令最终调用 `sifli_hci_log_enable()`，把 trace 配置写入 NVDS(`NVDS_STACK_TAG_TRACER_CONFIG`)，因此重启后依然有效。

## 2. 串口工具(SifliTrace)如何跟踪开发板 HCI log

选择对应 HCI log 输出端口号并将 log 类型选择为 HCI

```{figure} assets/hci_en.png
:align: center

选择 log 类型
```

## 3. 如何解析开发板 HCI log

解析开发板 HCI log 需要用到 Frontline 工具，使用步骤如下。

### 3.1 安装 Frontline

Frontline 是 Teledyne LeCroy 的商业软件(无线协议分析仪配套软件)。

- 官方下载页：[Software Downloads - Protocol Solutions](https://www.teledynelecroy.com/support/softwaredownload/psgdocuments.aspx)

```{warning}
Frontline 需默认路径安装。
```

### 3.2 打开 Frontline

```{figure} assets/frontline_open.png
:align: center

打开 Frontline
```

### 3.3 选择 CPAS Side

```{figure} assets/frontline_select_cpas.png
:align: center

选择 CPAS Side
```

### 3.4 配置串口协议

打开设置，配置串口协议。

```{figure} assets/frontline_uart_protocol_1.png
:align: center

配置串口协议(1)
```

```{figure} assets/frontline_uart_protocol_2.png
:align: center

配置串口协议(2)
```

### 3.5 SifliTrace 勾选 Frontline

```{figure} assets/siflitrace_enable_frontline.png
:align: center

在 SifliTrace 中勾选 Frontline
```

### 3.6 开始抓取并查看解析结果

Frontline 点击开始抓取 log。

```{figure} assets/frontline_start_capture.png
:align: center

Frontline 开始抓取 log
```

```{figure} assets/frontline_parsed_hci_log.png
:align: center

已解析的 HCI log
```

---

## 4. 手机如何抓取 HCI log

不同品牌的手机抓取 HCI log 的方法不同，以下罗列主流品牌手机抓取 HCI log 的方法。

### 4.1 华为/荣耀

1. 开启开发者模式
    - `设置` → `关于手机` → `软件信息`，点击“版本号” 7 次。
2. 启用 HCI 日志
    - 进入 `系统和更新` → `开发者选项`，打开“开启蓝牙HCI信息收集日志”。
3. 复现问题
    - 手机复现问题。
4. 提取日志
    - PC 通过 adb 工具导出 log：

      ```
      adb pull /data/log/bt <PC本地路径>
      ```

    - 日志路径：`bt/btsnoop_hci_xxxxxxxx.log`。

### 4.2 OPPO/Realme

1. 开启开发者模式
    - `设置` → `关于手机` → `软件信息`，点击“版本号” 7 次。
2. 启动日志抓取
    - 拨号界面输入 `*#800#` → 选择 蓝牙异常 → 切换为开发者模式 → 点击 开始抓取(不重启)。
3. 复现问题
    - 手机复现问题。
4. 结束抓取
    - 完成问题复现后点击 结束抓取。
5. 日志路径
    - OPPO：

      ```
      /内部共享存储空间/oppo_log/
      /内部共享存储空间/Android/data/com.coloros.logkit/files/Log/
      /storage/emulated/0/oppo_log
      ```

    - Realme：

      ```
      /storage/emulated/0/Android/data/com.coloros.logkit/files/Log/
      ```

    - 文件后缀为 `.cfa`。

### 4.3 小米/红米

1. 启用日志抓取
    - 拨号输入 `*#*#5959#*#*` → 任务栏提示抓取开始。
2. 复现问题
    - 手机复现问题。
3. 结束抓取
    - 拨号输入 `*#*#5959#*#*`。
4. 日志路径
    - 本地路径：`MIUI/debug_log/`
    - 或通过 `adb bugreport` 获取压缩包 `bugreport-xxxx.zip`。

### 4.4 三星

1. 开启开发者模式
    - `设置` → `关于手机` → `软件信息`，点击“版本号” 7 次。
2. 启用 HCI 日志
    - `开发者选项` → `启用蓝牙HCI监听日志`。
3. 复现问题
    - 手机复现问题。
4. 导出日志
    - 拨号输入 `*#9900#` → 点击 RUN DUMPSTATE/LOGCAT → 完成后选择 COPY TO SDCARD。
5. 日志路径
    - `log/bluetooth/btsnoop_hci_xxxxxxxx.cfa`

### 4.5 Google 手机

1. 开启 HCI 日志
    - `设置` → `关于手机` → `软件信息`，点击“版本号” 7 次。
2. 提取日志

    ```
    adb bugreport
    ```

    - 日志路径：`debuglogger/connsyslog/bthci/CsLog_xxxxxxxx`

### 4.6 vivo/IQOO

1. 启用日志抓取
    - 拨号输入 `*#*#112#*#*` 进入 log 抓取工具，log 类型选择蓝牙，点击开始抓取。
2. 复现问题
    - 手机复现问题。
3. 结束抓取
    - 在 log 抓取工具中点击停止抓取。
4. 导出日志
    - 待 log 自动压缩完成后点击分享，可通过分享给 QQ 或微信等导出日志。

### 4.7 iPhone(iOS)

#### 4.7.1 手机端安装 Profile

- 在手机 `Safari 浏览器`中打开 `https://developer.apple.com/bug-reporting/profiles-and-logs/?name=bluetooth`

**输入 Apple ID**

```{figure} assets/ios_login_apple_id.png
:align: center

输入 Apple ID
```

**选择 Bluetooth for iOS - Instructions**

```{figure} assets/ios_select_instructions.png
:align: center

选择 Instructions
```

**点击 profile**

```{figure} assets/ios_click_profile.png
:align: center

选择 profile
```

**profile 下载完成**

```{figure} assets/ios_profile_downloaded.png
:align: center

profile 下载完成
```

**安装配置文件**

```{figure} assets/ios_install_profile.png
:align: center

安装配置文件
```

**配置文件安装完成**

```{figure} assets/ios_profile_installed.png
:align: center

配置文件安装完成
```

**验证配置文件**

```{figure} assets/ios_verify_profile.png
:align: center

验证配置文件
```

#### 4.7.2 获取日志

- 手机连接 PC，PC 端打开 Additional Tools - Hardware - PacketLogger 后开始抓取 log。手机开始复现问题即可。

### 4.8 注意事项

1. Root 权限
    - 若无法找到日志路径，可能需要 Root 手机。
2. 配置文件修改
    - 修改 `bt_stack.conf` 需 Android 源码编译权限。
3. 日志格式
    - `.log`、`.cfa`、`.pklg` 均可用 Wireshark 或 Frontline 分析。
