# disk 目录说明

本目录的内容由构建系统打包成 FAT 文件系统镜像 `fs_root.bin`，烧录后挂载为设备的根目录 `/`。

## 目录结构

```
lvgl_v8_freetype/
├── disk/                  ← 文件系统根目录
│   └── font/              ← 字体目录，对应设备上的 /font/
├── project/
│   └── SConstruct         ← 通过 FileSystemBuild("../disk") 完成打包
└── src/
    └── font_switch_demo.c ← 从 /font/ 扫描并加载字体
```

## 打包与烧录

编译时会自动扫描本目录、用 `mkfatimg` 生成 `fs_root.bin`，并输出到 `build_<board>_hcpu/` 目录；设备启动时由 `src/main.c` 的 `mnt_init()` 将其挂载到 `/`。

烧录时必须使用**完整镜像**（`download.bat` / `uart_download.bat` 已包含 `fs_root.bin`），只烧 `main.bin` 不会更新设备上的字体文件。

对应的必需配置：

```ini
CONFIG_RT_USING_DFS=y              # 设备文件系统
CONFIG_RT_USING_DFS_ELMFAT=y       # ELM FAT 文件系统
```

## 注意事项

1. 本目录的总大小受分区表中 `FS_REGION` 分区容量限制（默认 4MB），超出时镜像打包会失败；
2. 运行时字体文件为只读；
3. 字体文件较大时会增加编译（打包）时间。

## 故障排除

* **串口提示找不到字体文件**：确认 `disk/font/` 下有 `.ttf` 文件、编译日志中有 `Generating ... fs_root.bin`，且串口出现 `mount fs on flash to root success`；
* **打包失败（mkfatimg 返回非零）**：多为本目录总大小超出 `FS_REGION` 分区容量，删除不需要的字体或改用子集化字体。

## 参考

- [mkfatimg 工具说明](../../../../../tools/mkfatimg/readme.txt)
- [FileSystemBuild 实现](../../../../../tools/build/building.py)
