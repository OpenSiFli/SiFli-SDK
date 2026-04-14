
# SiFli_Trace

## 1. Overview

SiFli_Trace is an in-house tool developed by SiFli Technology. Its primary function is to capture log output from a target board.\
Tool path: `tools/SiFli_Trace`

## 2. Environment Setup

SiFli_Trace requires no installation and runs directly on Windows (XP / 7 / 10 / 11 …).

When using JLink as the Trace interface, JLink hardware and its companion software are required. The recommended steps are:

- Purchase an official JLink device and install the SEGGER JLink Windows driver software. The version used during development of this tool is V680a.
- In the tool's configuration file `SifliTrace.ini`, modify the `JLINKPATH` entry under the `[WND]` section. e.g.: `JLINKDLL=C:\Program Files (x86)\SEGGER\JLink`

## 3. Features

<img src="png/SiFli_Trace_001.png"/>

The main interface is shown above and consists of five areas.

### 3.1 Function Control Area

- **New**
  Adds a new receive panel. Up to 8 panels can be added.
- **Horizontal**
  Arranges receive panels in a horizontal layout.
- **Vertical**
  Arranges receive panels in a vertical layout.
- **Connect**
  Opens the device connection on all receive panels to start receiving data.
- **Disconnect**
  Disconnects all receive panels from their devices, stopping data reception.
- **Save**
  Saves the current log file on all receive panels. Subsequent data is saved to a new file.
- **Clear**
  Clears the display content of all receive panels and the search panel.
- **Refresh**
  Enables real-time display refresh on all panels.
- **Pause**
  Pauses real-time display refresh on all panels. Data is still received and saved to file; only the live display is paused.
- **Search**
  Opens the search panel.
- **Device**
  Opens Device Manager.
- **Calculator**
  Opens the system calculator.
- **Path**
  Opens the log save directory.
- **DUMP**
  Opens the AssertDump tool (for reading crash memory data).
- **SERVER**
  Opens the UsartServer tool (required for serial-port-based debugging on SF32LB52X and SF32LB56X platforms).
- **File**
  Opens the FsrwTool (for reading and writing files on the target board).

### 3.2 Quick-Access Button Bar

The quick-access button bar shows the function control buttons as small icons, allowing the function control area to be hidden so the receive panels have more display space. Click the down-arrow next to the bar to add or remove buttons from the bar, or to show/hide the function control area.

### 3.3 Receive Panels

Up to 8 receive panels can be added. Each panel has the same layout but can be configured independently. Controls in each panel:

- **Port**
  The drop-down list shows detected COM port numbers, JLink SN numbers, UART1_SOCKET, and UART2_SOCKET. The last two are used together with the UsartServer tool — when UsartServer connects to a serial port for debugging, it distributes firmware log data over sockets, and Trace connects to those sockets for display.
- **Baud Rate**
  Sets the serial port baud rate or JLink speed. The serial baud rate must match the rate configured in the firmware.
- **Mode**
  Four processing modes:
  - **Char**
    Received data is displayed as-is in character format, with no processing.
  - **HCI**
    HCI data is recorded with a reception timestamp; non-HCI data is displayed as characters.
  - **HEX**
    Received data is displayed as raw hexadecimal bytes, with no other processing.
  - **Audio**
    Audio data is saved in a special format; non-audio data is displayed as characters.
- **Connection Indicator**
  Green when connected, red when not connected, grey when a previously connected device is lost. The icon flashes when data is being received.
- **Connect**
  Connects to or disconnects from the device.
- **Clear**
  Clears the received data displayed in this panel.
- **Refresh**
  When checked, the panel updates in real time as data arrives. When unchecked, data is still received and saved but the display is not refreshed. Disabling refresh improves reception throughput when high-volume data is being output.
- **Display**
  When checked, received data is shown in the panel. When unchecked, data is still received and saved but not displayed. Disabling display improves reception throughput when high-volume data is being output.
- **HCI Data Display**
  When in HCI mode, the tool parses received HCI data. Use this toggle to control whether the full HCI data is shown or only a summary notification. Displaying full HCI data can impact throughput when data volume is high.
- **52X_BOOT**
  When checked, the tool sends a command to force SF32LB52X boards into BOOT mode on startup.
- **Front.**
  Connects to the Frontline HCI analysis tool. HCI data is forwarded to Frontline for parsing. See the later section for usage details.
- **RTS Reset**
  Some development boards use the serial port's RTS pin to control the power supply. This button provides a convenient way to reset and reboot the target board.
- **Log Playback**
  Plays back log files previously captured by this tool. Click to select one or more log files for playback.
- **Command Input**
  Sends finsh shell commands to the target board. A `\r\n` terminator is automatically appended. Press Enter after typing to send.
- **Log Name**
  A fixed string appended to the log filename for this panel. Currently unused.
- **Log Size**
  The maximum size per log file. When this size is exceeded, the current file is saved and a new file is created for subsequent data.

### 3.4 Send Panel

- **Multi-Window**
  Commands typed in the send panel are sent to all checked receive panels simultaneously. The target panels must be connected.
- **Panel Index**
  Checkboxes corresponding to each receive panel. All are selected by default.
- **Send Status Indicator**
  Green during loop sending; red otherwise.
- **Loop Send**
  Sends the checked commands in the command list in a loop.
- **Stop on Crash**
  In loop send mode, stops sending if a crash event is detected on the target board.
- **Single Byte Send**
  Sends commands one byte at a time to avoid issues on target boards that cannot process rapid continuous data.
- **Command List**
  Stores commands for convenient reuse. Commands support four formats: Char, HEX, CMD, FILE. The default is Char; double-click a cell to switch format.
  - **Char**
    For sending finsh shell commands. The command is sent as a character string with `\r\n` appended.
  - **HEX**
    For sending HCI commands or serial debug commands. The command string is converted to raw hexadecimal bytes. e.g.: `06 01 00 03`
  - **CMD**
    For invoking third-party tools such as batch scripts. e.g.: `d:\debug.bat`
  - **FILE**
    For sending data read from a file. Rarely needed; mainly used for large-data stress tests. e.g.: `d:\debug.bin`

### 3.5 Search Panel

- **Search / Monitor Strings**
  Up to three search/monitor strings can be configured. Only checked entries are active.
- **Case Insensitive**
  When checked, the search ignores letter case.
- **Search All Windows**
  Searches and monitors across all open receive panels.
- **Search Current Window**
  Searches and monitors only the currently active receive panel, shown in the title bar (e.g., "Sifli_Trace v2.3.3 - Window 1").
- **Monitor Search Strings**
  When checked, the tool monitors the configured strings in real time and displays matches in the results list.
- **Monitor Error Messages**
  When checked, the tool monitors for error events from the target board, such as asserts.
- **Search**
  Performs a search based on the current configuration and appends results to the results list.
- **Clear**
  Clears all entries from the results list.
- **Message Color Settings**
  Configures the display color for different message types in the receive panels.
- **Results List**
  Displays found or monitored messages. The first column shows the receive panel index. Clicking a row selects the corresponding item in the associated panel. When a panel's display is cleared, its entries are also removed from the results list.

## 4. Usage

The tool is straightforward to use. Double-click to open it. Common workflows are described below.

### 4.1 Capturing Logs

- Select the port, speed, and Trace mode.
- Click **Connect** to open the device connection.
- The tool begins receiving data, displaying it in the panel and saving it to file. Saved files by mode:
  - **Char / HEX mode**
    `log\` folder — `Window_x_Port_(YYYY-MM-DD-HH-MM-SS)_ui.log`: UI display content with timestamps.
    `log\base\` folder — `Window_x_Port_(YYYY-MM-DD-HH-MM-SS).log`: raw data without timestamps.
  - **HCI mode**
    `log\` folder — `Window_x_Port_(YYYY-MM-DD-HH-MM-SS)_ui.log`: UI display content with timestamps.
    `log\` folder — `Window_x_Port_(YYYY-MM-DD-HH-MM-SS).bin`: raw HCI binary data.
    `log\` folder — `Window_x_Port_(YYYY-MM-DD-HH-MM-SS).bintime`: reception timestamp for each HCI packet.
    `log\` folder — `Window_x_Port_(YYYY-MM-DD-HH-MM-SS).pcap`: HCI data in pcap format for Wireshark.
  - **Audio mode**
    `log\` folder — `Window_x_Port_(YYYY-MM-DD-HH-MM-SS)_ui.log`: UI display content with timestamps.
    `log\base\` folder — `Window_x_Port_(YYYY-MM-DD-HH-MM-SS).log`: raw data without timestamps.
    `log\Window_x_Port_(YYYY-MM-DD-HH-MM-SS)_audio\` folder: captured audio data and error information.

:::{note}
A log file is automatically saved and a new file created when: the saved data exceeds the panel's configured size limit; the device is disconnected; or the **Save** button is clicked.
:::

### 4.2 Log Playback

When a panel is not connected to a device, click **Log Playback** and select one or more log files to replay. Raw data files from the `log\base\` folder can be replayed, as can raw HCI binary files (`.bin` extension). Multiple files can be selected.

:::{note}
The file browser defaults to `.bin` type. To open files with other extensions, change the file type filter in the open dialog.
:::

### 4.3 Using Frontline to Analyze HCI Data

- Install the Frontline software.
- In the tool's configuration file `SiFli.ini`, update the `FRONTLINEPATH` entry under the `[WND]` section. e.g.: `FRONTLINEPATH=C:\Program Files (x86)\Frontline Test System II\Frontline 15.14`
- Open Frontline, then go to **Options → Protocol Stack → Bluetooth HCI UART (H4) with autotraverse**.
- In Frontline, click **Live → Start Capture** to begin data reception.
- In SiFli_Trace, click **Front.** in a receive panel to connect to the device. Once connected, HCI data is forwarded to Frontline for analysis.

:::{note}
Frontline integration works for both live capture and log playback. Only one receive panel can be connected to Frontline at a time.
:::
