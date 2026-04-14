# Flash Loader Developer Notes

本目录用于维护 SiFli 的 flash 下载算法，包括：

- J-Link Open Flashloader `.elf`
- Keil CMSIS Flashloader `.FLM`
- 静态 DSK 包 `tools/flash/jlink_drv`

## 1. 依赖

使用 CMake 构建新的 flash 下载算法前，需要保证以下工具在 `PATH` 中可用：

- `cmake` 3.20 或以上
- `arm-none-eabi-gcc`
- `arm-none-eabi-g++`
- `arm-none-eabi-ar`
- `arm-none-eabi-ranlib`
- `arm-none-eabi-objcopy`
- `arm-none-eabi-readelf`
- `arm-none-eabi-size`

交叉工具链文件位于：

- [cmake/toolchain-arm-none-eabi-gcc.cmake](cmake/toolchain-arm-none-eabi-gcc.cmake)

默认情况下，执行完毕`export`命令之后`PATH`中就包含了上述工具，无需再做额外配置。

## 2. 配置构建目录

在 SDK 根目录执行：

```bash
cmake -S tools/flash -B tools/flash/build -G Ninja
```

请注意，必须使用`Ninja`生成器，否则在 Windows 系统上会默认使用 `Visual Studio` 生成器，导致构建失败。

如果需要重新配置，直接重复执行上面的命令即可。  
如果切换了工具链或需要彻底重建，可以先删除 `tools/flash/build` 后重新配置。

默认使用 Release 模式构建，如果需要 Debug 模式，可以添加 `-DCMAKE_BUILD_TYPE=Debug`

## 3. 构建目标

`tools/flash` 预定义了 3 个聚合目标：

- `flash_jlink_all`
  - 构建全部 J-Link Open Flashloader `.elf`
- `flash_cmsis_all`
  - 构建全部 Keil CMSIS Flashloader `.FLM`
- `flash_all`
  - 同时构建上述两类产物

常用命令：

```bash
cmake --build tools/flash/build --target flash_jlink_all -j8
cmake --build tools/flash/build --target flash_cmsis_all -j8
cmake --build tools/flash/build --target flash_all -j8
```

如果只想编译某一个具体算法，也可以直接指定单独 target。  
这些 target 定义集中在：

- [cmake/flash_targets.cmake](cmake/flash_targets.cmake)

命名风格大致如下：

- `flash__sf32lb52x__flash1`
- `flash__sf32lb56x_nand__nand3`
- `flash__sf32lb58x_sdio__sdio1`

例如只构建 `SF32LB56X_EXT_FLASH3`：

```bash
cmake --build tools/flash/build --target flash__sf32lb56x__flash3 -j8
```

## 4. 构建产物

构建完成后，主要产物位于：

- `tools/flash/build/stage/jlink_drv`
  - 新生成的 J-Link Open Flashloader `.elf`
- `tools/flash/build/stage/keil_drv`
  - 新生成的 Keil `.FLM`
- `tools/flash/build/generated/flashdrv_version.h`
  - 由 SDK 根目录 `.version` 生成的版本头文件
- `tools/flash/build/maps`
  - 各 target 的链接 map 文件
- `tools/flash/build/raw`
  - CMake target 的原始输出目录

其中：

- `tools/flash/build/stage/jlink_drv` 是 CMake 构建出来的新算法目录
- `tools/flash/jlink_drv` 是仓库中维护的静态 DSK 包目录

这两个目录的职责不同，不要混淆。

## 5. 更新静态 DSK 包

如果只是验证新算法，直接使用 `tools/flash/build/stage/jlink_drv` 下的 `.elf` 即可。  

如果要把新算法正式并入仓库交付的静态 DSK 包，需要做两步：

1. 将 `tools/flash/build/stage/jlink_drv` 中对应的 `.elf` 回灌到 `tools/flash/jlink_drv` 的相同子目录
2. 按需要更新 [jlink_drv/JLinkDevices.xml](jlink_drv/JLinkDevices.xml)

如果只是替换已有算法，通常只需要覆盖对应 `.elf`。  
如果新增了新的 bank、device 或 loader 组合，则必须同步修改 `JLinkDevices.xml`。

## 6. 调试建议

如果 J-Link 在加载算法时报类似错误：

```text
ERROR: Algo error: FlashDevice.SectorInfo[] NumEntries = 4096, max. allowed = 512
```

优先检查算法里的 `FlashDevice` 描述符，特别是：

- `TotalSize`
- `SectorInfo` / `SectorSize`

相关定义可见：

- [src/FlashDev.c](src/FlashDev.c)
- 各 SoC/介质专用 `FlashDev.c`
- `FlashOS.h` 中的 `MAX_NUM_SECTORS`

这个错误属于算法描述不合法，不是 DSK XML 路径问题。
