# 字体目录

本目录的字体会被打包进文件系统镜像，设备上通过 `/font/<文件名>` 访问；例程在打开字体列表时扫描本目录，`.ttf` / `.otf` 文件无需改代码即可出现在列表中。

## 当前字体文件

| 文件名 | 来源 | 大小 | 说明 |
|--------|------|------|------|
| DroidSansFallback_Simplified.ttf | Droid Sans Fallback 子集 | 约 1.0MB | **默认字体**，含 GB2312 全部汉字、数字、中英文标点 |
| A-SourceHanSansCN-M_Simplified.ttf | 思源黑体 Medium 子集 | 约 2.0MB | CFF 轮廓已转换为 TrueType 轮廓 |
| FontSwitchDemoCN.ttf | HarmonyOS Sans SC 子集 | 约 25KB | 精简测试字体，字形很少，用于观察缺字回退 |
| FontSwitchDemoCN1~4.ttf | 同上 | 约 25KB×4 | 副本，用于验证多字体列表与反复切换 |

## 添加自己的字体

把 `.ttf` 放入本目录、重新编译并烧录完整镜像即可。准备字体时请注意本 SDK 的两条限制：

1. **只支持 TrueType 轮廓**。固件的 FreeType 仅注册了 TrueType 驱动，CFF 轮廓的 OTF 打不开（串口提示 `unsupported outline format`），需要先转换为 TrueType 轮廓。
2. **全量中文字体必须子集化**。字体打开后其字形索引表常驻内存，五万字形的中文字体仅索引表就要数百 KB，小内存板卡无法承受。可用 [fontTools](https://github.com/fonttools/fonttools) 子集化：

```
pip install fonttools
pyftsubset YourFont.ttf --output-file=YourFont_Simplified.ttf \
    --unicodes=U+0020-007E,U+00A0-00FF,U+2000-206F,U+3000-303F,U+FF00-FFEF \
    --text-file=gb2312.txt --drop-tables+=GSUB,GPOS,GDEF
```

子集化时请确认收入了例程预览文本所需的中英文字符与全部数字，否则缺失的字符会回退到内置字体（显示偏小）。

## 注意事项

- 本目录连同 `disk/` 的总大小受 `FS_REGION` 分区容量限制（默认 4MB）；
- 更换默认字体需同步修改 `src/font_switch_demo.c` 中的 `FONT_DEMO_DEFAULT_FONT` 宏。

## 字体来源

- Droid Sans Fallback: https://android.googlesource.com/platform/frameworks/base/+/master/data/fonts
- 思源黑体 (Source Han Sans): https://github.com/adobe-fonts/source-han-sans
