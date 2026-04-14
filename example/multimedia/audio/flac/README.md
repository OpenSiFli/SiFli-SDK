# FLAC 编解码示例

本文档对应示例代码：`sifli-sdk/example/multimedia/audio/flac/src/main.c`。该示例展示了 **录音 → PCM → FLAC编码 → FLAC解码 → 扬声器播放** 的完整链路，并提供 3 个 MSH 命令入口。

---

## 1. 示例提供的命令

代码支持以下命令（在 `main()` 中也会打印提示）：

- **`flac_test [seconds]`**
  - 功能：录音指定秒数到 `/mic_record.pcm`，再编码为 `/test.flac`，最后解码播放
  - 默认：`seconds` 不传时为 10 秒
- **`flac_enc [in] [out]`**
  - 功能：把 PCM 文件编码为 FLAC
  - 默认：`/mic_record.pcm` → `/test.flac`
- **`flac_play [file]`**
  - 功能：解码 FLAC 文件并播放
  - 默认：播放 `/test.flac`

---



注意：

在external/kconfig目录下加：

```c
source "$SIFLI_SDK/external/flac-1.5.0/Kconfig"
```

