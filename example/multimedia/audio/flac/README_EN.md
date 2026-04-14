# FLAC Codec Example

This document corresponds to the example code: `sifli-sdk/example/multimedia/audio/flac/src/main.c`. This example demonstrates the complete chain of **Recording → PCM → FLAC Encoding → FLAC Decoding → Speaker Playback** and provides 3 MSH command interfaces.

---

## 1. Commands Provided by the Example

The code supports the following commands (prompts are also printed in `main()`):

- **`flac_test [seconds]`**
- Function: Records audio for the specified number of seconds to `/mic_record.pcm`, then encodes it to `/test.flac`, and finally decodes it for playback.
- Default: 10 seconds if `seconds` is not provided
- **`flac_enc [in] [out]`**
- Function: Encodes a PCM file to FLAC
- Default: `/mic_record.pcm` → `/test.flac`
- **`flac_play [file]`**
- Function: Decodes a FLAC file and plays it
- Default: plays `/test.flac`

---