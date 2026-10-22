# HCI Log Capture Guide

This document describes how to capture Bluetooth HCI logs, covering four scenarios: enabling the HCI log on the development board, tracing the board HCI log with a serial port tool (SifliTrace), parsing the HCI log with Frontline, and capturing the HCI log on a phone.

---

## 1. Enabling the HCI Log on the Development Board

First, make sure the following two macros are enabled in the project configuration:

```
CONFIG_HCI_ON_LOG=y
CONFIG_LOG_ON_CONSOLE=y
```

The HCI log switch is stored in NVDS. Enter the following command in the serial terminal (finsh/msh):

```
nvds update hci_log 1
```

- `1` enables the HCI log, and `0` disables it.
- The command eventually calls `sifli_hci_log_enable()`, which writes the trace configuration to NVDS (`NVDS_STACK_TAG_TRACER_CONFIG`); the setting therefore survives a reboot.

## 2. Tracing the Board HCI Log with a Serial Port Tool (SifliTrace)

Select the corresponding HCI log output port and set the log type to HCI.

```{figure} assets/hci_en.png
:align: center

Select the log type
```

## 3. Parsing the Board HCI Log

Parsing the board HCI log requires the Frontline tool. The steps are as follows.

### 3.1 Install Frontline

Frontline is a commercial software product from Teledyne LeCroy (the companion software for wireless protocol analyzers).

- Official download page: [Software Downloads - Protocol Solutions](https://www.teledynelecroy.com/support/softwaredownload/psgdocuments.aspx)

```{warning}
Frontline must be installed in the default path.
```

### 3.2 Open Frontline

```{figure} assets/frontline_open.png
:align: center

Open Frontline
```

### 3.3 Select the CPAS Side

```{figure} assets/frontline_select_cpas.png
:align: center

Select the CPAS side
```

### 3.4 Configure the Serial Protocol

Open the settings and configure the serial protocol.

```{figure} assets/frontline_uart_protocol_1.png
:align: center

Configure the serial protocol (1)
```

```{figure} assets/frontline_uart_protocol_2.png
:align: center

Configure the serial protocol (2)
```

### 3.5 Select Frontline in SifliTrace

```{figure} assets/siflitrace_enable_frontline.png
:align: center

Select Frontline in SifliTrace
```

### 3.6 Start Capturing and View the Parsed Result

Click start capture in Frontline.

```{figure} assets/frontline_start_capture.png
:align: center

Start capturing the log in Frontline
```

```{figure} assets/frontline_parsed_hci_log.png
:align: center

Parsed HCI log
```

---

## 4. Capturing the HCI Log on a Phone

The method for capturing an HCI log differs between phone brands. The following sections list the methods for mainstream phone brands.

### 4.1 Huawei/Honor

1. Enable developer mode
    - `Settings` → `About phone` → `Software information`, then tap "Build number" 7 times.
2. Enable the HCI log
    - Go to `System & updates` → `Developer options`, and enable "Enable Bluetooth HCI snoop log".
3. Reproduce the problem
    - Reproduce the problem on the phone.
4. Extract the log
    - Export the log from a PC using the adb tool:

      ```
      adb pull /data/log/bt <local path on PC>
      ```

    - Log path: `bt/btsnoop_hci_xxxxxxxx.log`.

### 4.2 OPPO/Realme

1. Enable developer mode
    - `Settings` → `About phone` → `Software information`, then tap "Build number" 7 times.
2. Start log capture
    - Dial `*#800#` → select Bluetooth exception → switch to developer mode → tap Start capture (without reboot).
3. Reproduce the problem
    - Reproduce the problem on the phone.
4. Stop capture
    - After reproducing the problem, tap Stop capture.
5. Log path
    - OPPO:

      ```
      /Internal shared storage/oppo_log/
      /Internal shared storage/Android/data/com.coloros.logkit/files/Log/
      /storage/emulated/0/oppo_log
      ```

    - Realme:

      ```
      /storage/emulated/0/Android/data/com.coloros.logkit/files/Log/
      ```

    - The file suffix is `.cfa`.

### 4.3 Xiaomi/Redmi

1. Enable log capture
    - Dial `*#*#5959#*#*` → the status bar shows a notification indicating that capture has started.
2. Reproduce the problem
    - Reproduce the problem on the phone.
3. Stop capture
    - Dial `*#*#5959#*#*`.
4. Log path
    - Local path: `MIUI/debug_log/`
    - Or obtain the `bugreport-xxxx.zip` archive via `adb bugreport`.

### 4.4 Samsung

1. Enable developer mode
    - `Settings` → `About phone` → `Software information`, then tap "Build number" 7 times.
2. Enable the HCI log
    - `Developer options` → "Enable Bluetooth HCI snoop log".
3. Reproduce the problem
    - Reproduce the problem on the phone.
4. Export the log
    - Dial `*#9900#` → tap RUN DUMPSTATE/LOGCAT → when finished, select COPY TO SDCARD.
5. Log path
    - `log/bluetooth/btsnoop_hci_xxxxxxxx.cfa`

### 4.5 Google Phones

1. Enable the HCI log
    - `Settings` → `About phone` → `Software information`, then tap "Build number" 7 times.
2. Extract the log

    ```
    adb bugreport
    ```

    - Log path: `debuglogger/connsyslog/bthci/CsLog_xxxxxxxx`

### 4.6 vivo/IQOO

1. Enable log capture
    - Dial `*#*#112#*#*` to open the log capture tool, select Bluetooth as the log type, and tap Start capture.
2. Reproduce the problem
    - Reproduce the problem on the phone.
3. Stop capture
    - Tap Stop capture in the log capture tool.
4. Export the log
    - After the log is compressed automatically, tap Share to export it, for example via QQ or WeChat.

### 4.7 iPhone (iOS)

#### 4.7.1 Install the Profile on the Phone

- On the phone, open `https://developer.apple.com/bug-reporting/profiles-and-logs/?name=bluetooth` in Safari.

**Enter your Apple ID**

```{figure} assets/ios_login_apple_id.png
:align: center

Enter your Apple ID
```

**Select Bluetooth for iOS - Instructions**

```{figure} assets/ios_select_instructions.png
:align: center

Select Instructions
```

**Tap the profile**

```{figure} assets/ios_click_profile.png
:align: center

Select the profile
```

**The profile is downloaded**

```{figure} assets/ios_profile_downloaded.png
:align: center

Profile downloaded
```

**Install the configuration profile**

```{figure} assets/ios_install_profile.png
:align: center

Install the configuration profile
```

**The configuration profile is installed**

```{figure} assets/ios_profile_installed.png
:align: center

Configuration profile installed
```

**Verify the configuration profile**

```{figure} assets/ios_verify_profile.png
:align: center

Verify the configuration profile
```

#### 4.7.2 Obtain the Log

- Connect the phone to a PC, open Additional Tools - Hardware - PacketLogger on the PC, and start capturing the log. Then reproduce the problem on the phone.

### 4.8 Notes

1. Root permission
    - If you cannot find the log path, you may need to root the phone.
2. Configuration file modification
    - Modifying `bt_stack.conf` requires the permission to build the Android source code.
3. Log format
    - `.log`, `.cfa`, and `.pklg` can all be analyzed with Wireshark or Frontline.
