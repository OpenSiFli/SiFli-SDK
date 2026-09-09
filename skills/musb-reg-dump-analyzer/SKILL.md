---
name: musb-reg-dump-analyzer
description: Decode and interpret a SiFli MUSB (USBC) register snapshot (.bin / usbc.bin) in host or device mode — which endpoints are armed/busy/stalled, direction, target device endpoint, and CSR-level detail. Auto-detects role from DEVCTL, or force with --role device. Use when the user hands a MUSB register dump, says "usbc.bin", "分析 MUSB 寄存器", "哪个端点在跑/在卡", "端点寄存器", "device 模式", or asks to tell which endpoint is working from registers.
---

# MUSB Register Dump Analyzer

Decode a raw little-endian memory snapshot of the SiFli **USBC (Mentor MUSB)** peripheral and explain what the registers say about endpoint activity.

## Input assumption

The `.bin` is a raw memory image where **file byte offset == USBC register offset**
(byte 0 = FADDR, byte 1 = POWER, …). This is the layout of the packed structs in
`drivers/cmsis/sf32lb58x/usbc_x.h` (same Mentor core on SF32LB52/55/56/57/58;
only the vendor tail above 0x340 differs). Register width is 8/16/32, little-endian.

## Run the decoder

```bash
python ".claude/skills/musb-reg-dump-analyzer/scripts/decode_musb_dump.py" <path/to/usbc.bin>
python ".claude/skills/musb-reg-dump-analyzer/scripts/decode_musb_dump.py" --bits <path/to/usbc.bin>
python ".claude/skills/musb-reg-dump-analyzer/scripts/decode_musb_dump.py" <path/to/usbc.bin> --role device
```

- `--role` 取值 `auto`(默认,按 `DEVCTL.HM/BDEVICE` 自动识别)/ `host` / `device`。
- Host 与 Device 模式共用同一套寄存器但**位含义不同**,解码器按角色切换:
  - CSR0/TXCSR/RXCSR 位名与 `[busy/armed]` 解释(host 用 NAKTIMEOUT/REQPKT 等;device 用 UNDERRUN/OVERRUN/SENTSTALL/SETUPEND 等);
  - 端点方向标注:host 标 `OUT>dev epn` / `IN<dev epn`;device 标 `send-to-host(IN)` / `recv-from-host(OUT)`。

`--bits` 追加一段 `[bit fields]`:把 POWER/DEVCTL/IS/IE、EP0 csr0 和各已配置端点的
TXCSR/RXCSR **逐位展开**(MSB→LSB,位名随当前角色切换),用于和 `usbc_x.h` 逐位核对原始值。

If `python` is the WindowsApps stub, use a real interpreter, e.g.
`/c/Python311/python`. The decoder prints:

- `[common]`: FADDR / POWER / DEVCTL / IS / TXIS / RXIS / TXIE / RXIE / EPIDX, EPINFO.
- `[endpoints]`: per physical endpoint bank (0x100 + 0x10*n) — maxpacket, CSR bits,
  TXTYPE/RXTYPE (protocol + which **device** endpoint it talks to).
- `[dma]`: DMA channels (0x200+) if any are active.
- `[busy/armed interpretation]`: host/device mode and which endpoint holds an
  unfinished packet / error flag, plus a full/low-speed vs >64B maxpacket check.

## How to read the result (don't mislead the user)

- **TXIS/RXIS are pending-interrupt flags** — they clear in an instant, so a zero
  value does *not* mean nothing is running.
- **TXIE/RXIE are armed flags.** In the SiFli CherryUSB host port they accumulate
  over the session and are rarely cleared, so they are *history + current*, not a
  precise "busy now" picture.
- **"Busy now" lives in the per-endpoint CSR low register:**
  - TX `TXCSR.TXPKTRDY`(0x1) + `FIFONOTEMPTY`(0x2) → a packet is loaded but not
    yet sent (data stuck, often the hang point).
  - RX `RXCSR.RXPKTRDY`(0x1) → data arrived, not read yet; `REQPKT`(0x20) → an IN
    request is outstanding.
  - Error/hang: `NAKTIMEOUT`(0x80), `RXSTALL`/`TXSTALL`(0x20/0x40), `ERROR`(0x4).
- Physical EP number (bank/chidx) is **not** the device endpoint number; read the
  TXTYPE/RXTYPE low nibble for the target device endpoint.
- A device pipe is often only used in one direction, but the CSR/TYPE of the other
  half may hold leftovers from an earlier URB on the same chidx — don't treat both
  halves as simultaneously live.

## Cross-checks

- Confirm register offsets/bit fields against `drivers/cmsis/<soc>/usbc_x.h`
  (e.g. `sf32lb58x`) when a value looks odd.
- `external/cherryusb/port/musb/usb_musb_reg.h` holds the *alternate* Stellaris
  bit numbering — do **not** use it for POWER/DEVCTL/CSR masks of this core; the
  glue code (`usb_glue_sifli.c`) explicitly `#undef`s those macros.
- For host-driver behavior context (pipe/URB allocation, why enables accumulate)
  see `external/cherryusb/port/musb/usb_hc_musb.c`.
- The user's own study notes live at `docs/usb_musb_notes.md`.

## Typical conclusion shape

A short answer of the form: *host/device mode; endpoint(s) X on the Y side (to/from
device ep Z, <proto>, <MPS>B) are armed/active; at capture time only EPn TX holds an
unfinished packet (csr=0x…); DMA idle; plus any anomaly (e.g. FS bus + 512B bulk = HS-only).*
