# Hello QEMU 示例（RT-Thread）

源码路径: example/qemu/hello_qemu

## 概述
在 QEMU（`qemu_sf32lb52x` 虚拟板，MPS2-AN505 机器）上运行的 RT-Thread 例程，通过 semihosting 在宿主终端打印 "Hello SiFli on QEMU!"。

此例程是纯 RT-Thread 应用，不使用 SiFli HAL，所以五块带芯片身份的载体板（`qemu_sf32lb52x`…`qemu_sf32lb58x`）都能跑；这里以 `qemu_sf32lb52x` 为例。

## 用法

### 编译
切换到例程 project 目录，运行 scons 命令执行编译：
```
scons --board=qemu_sf32lb52x -j8
```
产物：`build_qemu_sf32lb52x_hcpu/main.elf`

### 运行
需要安装 QEMU（`qemu-system-arm`）。运行：
```
qemu-system-arm -machine mps2-an505 -cpu cortex-m33 -nographic \
    -semihosting -semihosting-config enable=on,target=native \
    -kernel build_qemu_sf32lb52x_hcpu/main.elf
```
或使用脚本：
```
python tools/qemu/qemu_smoke_test.py
```
期望输出：RT-Thread banner + `Hello SiFli on QEMU!` + `msh />` 提示符。

### 调试
```
tools/qemu/qemu_debug.bat   # 启动 QEMU + GDB server（tcp:1234）+ gdb
```
