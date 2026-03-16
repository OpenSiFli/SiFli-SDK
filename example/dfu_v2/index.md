# DFU V2 例程

DFU V2 是 SiFli SDK 的固件升级（DFU / OTA）中间件，支持三种传输通道：USB-CDC、BLE、Bluetooth PAN（网络）。本目录按通道收纳了配套的例程。

## 目录结构

```
dfu_v2/
├── cdc/      USB-CDC 通道
│   ├── app/      用户应用 + PC 升级工具（dfu_v2_cdc_tool.py）
│   └── loader/   CDC DFU loader 子程序
├── ble/      BLE 通道
│   ├── app/      BLE 外设应用（支持运行中实时更新，HCPU 自身升级转 loader 完成）
│   └── loader/   BLE DFU loader 子程序
└── bt_pan/   Bluetooth PAN（网络 / HTTP）通道
    ├── app/      联网从服务器拉取固件升级的应用
    └── loader/   PAN DFU loader 子程序
```

## 怎么选通道

- **USB-CDC**：PC 经 USB 虚拟串口升级，适合产线、有线调试场景。
- **BLE**：手机 App 经蓝牙升级，设备作为外设。
- **PAN**：设备经蓝牙 PAN 联网，自行从服务器拉取固件升级。

## app 与 loader 的关系

app 与同通道的 loader 配套使用。编译 app 时，SDK 会通过 `building.py` 的 `AddDFU_*` 自动把同通道的 loader 作为子工程一起编进来，无需单独构建 loader。

## 编译

进入某个例程的 `project/` 目录，执行：

```
scons --board=<板名> -j8
```

各例程支持的板子见其各自的 README。

## 例程索引

| 通道 | 用户应用 | loader 子程序 |
|---|---|---|
| USB-CDC | [cdc/app](cdc/app/README.md) | [cdc/loader](cdc/loader/README.md) |
| BLE | [ble/app](ble/app/README.md) | [ble/loader](ble/loader/README.md) |
| Bluetooth PAN | [bt_pan/app](bt_pan/app/README.md) | [bt_pan/loader](bt_pan/loader/README.md) |

DFU V2 中间件本身的配置项见 `middleware/dfu_v2`。

```{toctree}
:hidden:

cdc/app/README
cdc/loader/README
ble/app/README
ble/loader/README
bt_pan/app/README
bt_pan/loader/README
```
