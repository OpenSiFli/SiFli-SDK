#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""
decode_musb_dump.py -- decode a SiFli MUSB (Mentor USB core) register snapshot,
host-mode OR device-mode aware.

Input
-----
A raw little-endian memory image captured from the USBC peripheral (e.g. the
file an engineer saves as usbc.bin). File byte offset == USBC register offset,
i.e. byte 0 is FADDR, byte 0x01 is POWER, ..., just like the packed structs in
drivers/cmsis/sf32lb58x/usbc_x.h.

The Mentor core exposes two views of the per-endpoint registers and they are
aliases:
  * Indexed window at 0x10..0x1D (selected by the EPIDX reg at 0x0E)
  * Flat bank array   at 0x100 + 0x10*ep   (what this tool mainly reads)

Role
----
Same registers carry different bit meanings in host vs device mode (see
docs/usb_musb_notes.md section 4). The role is auto-detected from DEVCTL
(HM bit = host, BDEVICE bit = device) unless overridden:

    python decode_musb_dump.py <file.bin>                # auto (default)
    python decode_musb_dump.py <file.bin> --role device  # force device
    python decode_musb_dump.py <file.bin> --bits         # + per-bit fields

Reading the output
------------------
  * TXIS/RXIS hold *pending* interrupt flags (instantaneous, cleared fast).
    A zero here does NOT mean nothing is running.
  * TXIE/RXIE hold *armed* endpoints. On the SiFli ports these bits accumulate
    over the session and are rarely cleared, so they show history + current use.
    TXIE/RXIE are interrupt enables only -- they do NOT tell whether an EP is
    open/closed or was killed; look at the CSR bits instead.
  * Busy "right now" lives in the CSRs:
        host:   TXCSR.TXPKTRDY|FIFONOTEMPTY -> loaded but not sent
                RXCSR.RXPKTRDY (data) / REQPKT (IN request outstanding)
        device: TXCSR.TXPKTRDY|FIFONOTEMPTY -> packet queued to send to host
                RXCSR.RXPKTRDY|FIFOFULL     -> host data arrived, not read
  * Error/hang bits are role-dependent:
        host:   NAKTIMEOUT / RXSTALL / ERROR
        device: TX UNDERRUN / SENTSTALL / INCOMPTX ; RX OVERRUN / SENTSTALL
"""

import struct
import sys
from pathlib import Path

# --------------------------------------------------------------------------
# Register offsets (Mentor MUSB common block, identical across SiFli chips)
# --------------------------------------------------------------------------
OFF_FADDR, OFF_POWER = 0x00, 0x01
OFF_TXIS, OFF_RXIS = 0x02, 0x04          # u16
OFF_TXIE, OFF_RXIE = 0x06, 0x08          # u16
OFF_IS, OFF_IE = 0x0A, 0x0B              # u8 (INTRUSB/INTRUSBE)
OFF_FRAME, OFF_EPIDX = 0x0C, 0x0E        # u16, u8
OFF_TXMAXP, OFF_CSR0_TXCSR = 0x10, 0x12  # indexed window (u16)
OFF_RXMAXP, OFF_RXCSR = 0x14, 0x16
OFF_RXCOUNT, = (0x18,)
OFF_TXTYPE, OFF_TXINT = 0x1A, 0x1B       # indexed window
OFF_RXTYPE, OFF_RXINT = 0x1C, 0x1D
OFF_DEVCTL = 0x60
OFF_TXFIFOSZ, OFF_RXFIFOSZ = 0x62, 0x63
OFF_TXFIFOADD, OFF_RXFIFOADD = 0x64, 0x66  # u16
OFF_VCONTROL, OFF_HWVERS = 0x68, 0x6C    # u32, u16
OFF_EPINFO, OFF_RAMINFO = 0x78, 0x79
OFF_EP_BANK = 0x100                      # ep[n] at 0x100 + 0x10*n, each 16 B
OFF_DMAINTR = 0x200                      # u32
OFF_DMA_CH = 0x204                       # stride 0x10: cntl,addr,count,rsvd

# --------------------------------------------------------------------------
# Bit fields (name -> mask), shared between the concise print and --bits
# --------------------------------------------------------------------------
POWER = {"ISOUPDATE": 0x80, "SOFTCONN": 0x40, "HSENAB": 0x20,
         "HSMODE": 0x10, "RESET": 0x08, "RESUME": 0x04,
         "SUSPENDM": 0x02, "ENSUSPEND": 0x01}
DEVCTL = {"BDEVICE": 0x80, "FSDEV": 0x40, "LSDEV": 0x20, "VBUS": 0x18,
          "HM": 0x04, "HR": 0x02, "SESSION": 0x01}
INTRUSB = {"VBUSERROR": 0x80, "SESSREQ": 0x40, "DISCON": 0x20,
           "CONNECT": 0x10, "SOF": 0x08, "BABBLE": 0x04,
           "RESUME": 0x02, "SUSPEND": 0x01}

# TXCSR -- host mode
TX_H = {"TXPKTRDY": 0x1, "FIFONOTEMPTY": 0x2, "ERROR": 0x4,
        "FLUSHFIFO": 0x8, "RXSTALL": 0x20, "NAKTIMEOUT": 0x80,
        "DATATOGGLE": 0x100, "WR_DATATOGGLE": 0x200, "FRCDATATOG": 0x800,
        "DMAENAB": 0x1000, "MODE": 0x2000, "AUTOSET": 0x8000}
# RXCSR -- host mode
RX_H = {"RXPKTRDY": 0x1, "FIFOFULL": 0x2, "ERROR": 0x4, "DATAERROR": 0x8,
        "FLUSHFIFO": 0x10, "REQPKT": 0x20, "RXSTALL": 0x40,
        "CLRDATATOG": 0x80, "INCOMPRX": 0x100, "DATATOGGLE": 0x200,
        "WR_DATATOGGLE": 0x400, "DMAMODE": 0x800, "PID_ERR": 0x1000,
        "DMAENAB": 0x2000, "AUTOCLEAR": 0x8000, "AUTOREQ": 0x4000}
# CSR0 -- EP0, host mode (16-bit; high byte holds the old CSRH0 bits)
CSR0 = {"RXPKTRDY": 0x1, "TXPKTRDY": 0x2, "RXSTALL": 0x4, "SETUPPKT": 0x8,
        "ERROR": 0x10, "REQPKT": 0x20, "STATUSPKT": 0x40,
        "NAKTIMEOUT": 0x80, "FLUSHFIFO": 0x100, "DATATOGGLE": 0x200,
        "WR_DATATOGGLE": 0x400, "DIS_PING": 0x800}

# TXCSR -- device (peripheral) mode
TX_P = {"TXPKTRDY": 0x1, "FIFONOTEMPTY": 0x2, "UNDERRUN": 0x4,
        "FLUSHFIFO": 0x8, "SENDSTALL": 0x10, "SENTSTALL": 0x20,
        "CLRDATATOG": 0x40, "INCOMPTX": 0x80, "DMAMODE": 0x400,
        "FRCDATATOG": 0x800, "DMAENAB": 0x1000, "MODE": 0x2000,
        "ISO": 0x4000, "AUTOSET": 0x8000}
# RXCSR -- device (peripheral) mode
RX_P = {"RXPKTRDY": 0x1, "FIFOFULL": 0x2, "OVERRUN": 0x4, "DATAERROR": 0x8,
        "FLUSHFIFO": 0x10, "SENDSTALL": 0x20, "SENTSTALL": 0x40,
        "CLRDATATOG": 0x80, "INCOMPRX": 0x100, "DMAMODE": 0x800,
        "PID_ERR": 0x1000, "DMAENAB": 0x2000, "ISO": 0x4000,
        "AUTOCLEAR": 0x8000}
# CSR0 -- EP0, device (peripheral) mode
CSR0_P = {"RXPKTRDY": 0x1, "TXPKTRDY": 0x2, "SENTSTALL": 0x4, "DATAEND": 0x8,
          "SETUPEND": 0x10, "SENDSTALL": 0x20, "SERVICEDRXPKTRDY": 0x40,
          "SERVICEDSETUPEND": 0x80, "FLUSHFIFO": 0x100}

PROTO = {0x0: "CTRL", 0x1: "ISO", 0x2: "BULK", 0x3: "INT"}
SPEED = {0x0: "?", 0x1: "HS", 0x2: "FS", 0x3: "LS"}  # TYPE[7:6] encoding


# --------------------------------------------------------------------------
# Per-bit field names for the --bits verbose dump. Index == bit number.
# --------------------------------------------------------------------------
POWER_B = {0: "ENSUSPEND", 1: "SUSPENDM", 2: "RESUME", 3: "RESET",
           4: "HSMODE", 5: "HSENAB", 6: "SOFTCONN", 7: "ISOUPDATE"}
DEVCTL_B = {0: "SESSION", 1: "HR", 2: "HM", 3: "VBUS[0]", 4: "VBUS[1]",
            5: "LSDEV", 6: "FSDEV", 7: "BDEVICE"}
INTR_B = {0: "SUSPEND", 1: "RESUME", 2: "BABBLE/RESET", 3: "SOF",
          4: "CONNECT", 5: "DISCONNECT", 6: "SESSREQ", 7: "VBUSERROR"}
CSR0_B = {0: "RXPKTRDY", 1: "TXPKTRDY", 2: "RXSTALL", 3: "SETUPPKT",
          4: "ERROR", 5: "REQPKT", 6: "STATUSPKT", 7: "NAKTIMEOUT",
          8: "FLUSHFIFO", 9: "DATATOGGLE", 10: "WR_DATATOGGLE", 11: "DIS_PING"}
TXCSR_B = {0: "TXPKTRDY", 1: "FIFONOTEMPTY", 2: "ERROR", 3: "FLUSHFIFO",
           4: "SENDSTALL", 5: "RXSTALL", 6: "CLRDATATOG", 7: "NAKTIMEOUT",
           8: "DATATOGGLE", 9: "WR_DATATOGGLE", 10: "DMAMODE", 11: "FRCDATATOG",
           12: "DMAENAB", 13: "MODE", 14: "ISO", 15: "AUTOSET"}
RXCSR_B = {0: "RXPKTRDY", 1: "FIFOFULL", 2: "ERROR", 3: "DATAERROR",
           4: "FLUSHFIFO", 5: "REQPKT", 6: "RXSTALL", 7: "CLRDATATOG",
           8: "INCOMPRX", 9: "DATATOGGLE", 10: "WR_DATATOGGLE", 11: "DMAMODE",
           12: "PID_ERR", 13: "DMAENAB", 14: "AUTOREQ", 15: "AUTOCLEAR"}
CSR0_PB = {0: "RXPKTRDY", 1: "TXPKTRDY", 2: "SENTSTALL", 3: "DATAEND",
           4: "SETUPEND", 5: "SENDSTALL", 6: "SERVICEDRXPKTRDY",
           7: "SERVICEDSETUPEND", 8: "FLUSHFIFO"}
TXCSR_PB = {0: "TXPKTRDY", 1: "FIFONOTEMPTY", 2: "UNDERRUN", 3: "FLUSHFIFO",
            4: "SENDSTALL", 5: "SENTSTALL", 6: "CLRDATATOG", 7: "INCOMPTX",
            10: "DMAMODE", 11: "FRCDATATOG", 12: "DMAENAB", 13: "MODE",
            14: "ISO", 15: "AUTOSET"}
RXCSR_PB = {0: "RXPKTRDY", 1: "FIFOFULL", 2: "OVERRUN", 3: "DATAERROR",
            4: "FLUSHFIFO", 5: "SENDSTALL", 6: "SENTSTALL", 7: "CLRDATATOG",
            8: "INCOMPRX", 11: "DMAMODE", 12: "PID_ERR/DISNYET",
            13: "DMAENAB", 14: "ISO", 15: "AUTOCLEAR"}


class Dump:
    def __init__(self, data):
        self.d = data

    def u8(self, o):
        return None if o + 1 > len(self.d) else self.d[o]

    def u16(self, o):
        if o + 2 > len(self.d):
            return None
        return struct.unpack_from("<H", self.d, o)[0]

    def u32(self, o):
        if o + 4 > len(self.d):
            return None
        return struct.unpack_from("<I", self.d, o)[0]

    def have(self, o):
        return o < len(self.d)


def bits(value, table):
    return ",".join(name for name, mask in table.items() if value & mask) if value else "-"


def dec_type(v, side):
    """Decode a host TXTYPE/RXTYPE byte: proto + target endpoint + speed."""
    if v is None:
        return "-"
    proto = PROTO.get((v >> 4) & 3, "?")
    return "%s %s>ep%d%s" % (proto, side, v & 0x0F,
                             "" if (v & 0xC0) == 0 else "@" + SPEED.get((v >> 6) & 3, "?"))


def resolve_role(dev, forced):
    """Return 'host', 'device' or None. forced overrides auto detection."""
    if forced in ("host", "device"):
        return forced
    if dev is None:
        return None
    if dev & 0x04:            # DEVCTL.HM
        return "host"
    if dev & 0x80:            # DEVCTL.BDEVICE
        return "device"
    return None


def masks_for(role):
    """Pick name->mask tables (for concise CSR print) by role."""
    dev = (role == "device")
    return (CSR0_P if dev else CSR0,
            TX_P if dev else TX_H,
            RX_P if dev else RX_H)


def grids_for(role):
    """Pick per-bit tables (for --bits) by role."""
    dev = (role == "device")
    return (CSR0_PB if dev else CSR0_B,
            TXCSR_PB if dev else TXCSR_B,
            RXCSR_PB if dev else RXCSR_B)


def grid(value, width, names):
    """Expand a register into per-bit lines (MSB..LSB). names maps bit->name."""
    lines = []
    for b in range(width - 1, -1, -1):
        name = names.get(b) or "-"
        lines.append("      b%-2d %-16s = %d" % (b, name, (value >> b) & 1))
    return lines


def dump_bitfields(d, n_eps, role):
    csr0_b, tx_b, rx_b = grids_for(role)
    print("\n[bit fields] role=%s bit names" % (role or "?"))
    for name, off, w, tbl in (("POWER", OFF_POWER, 8, POWER_B),
                              ("DEVCTL", OFF_DEVCTL, 8, DEVCTL_B),
                              ("IS", OFF_IS, 8, INTR_B),
                              ("IE", OFF_IE, 8, INTR_B)):
        v = (d.u8(off) if w == 8 else d.u16(off)) or 0
        print("%s = 0x%0*X" % (name, w // 4, v))
        for ln in grid(v, w, tbl):
            print(ln)
    if not n_eps:
        return
    b = OFF_EP_BANK
    csr0 = d.u16(b + 2) or 0
    print("EP0 csr0 = 0x%04X" % csr0)
    for ln in grid(csr0, 16, csr0_b):
        print(ln)
    for i in range(1, n_eps):
        b = OFF_EP_BANK + 0x10 * i
        txcsr, rxcsr = d.u16(b + 2) or 0, d.u16(b + 6) or 0
        if not (txcsr or rxcsr or (d.u16(b) or 0) or (d.u16(b + 4) or 0)):
            continue
        print("EP%d TXCSR = 0x%04X" % (i, txcsr))
        for ln in grid(txcsr, 16, tx_b):
            print(ln)
        print("EP%d RXCSR = 0x%04X" % (i, rxcsr))
        for ln in grid(rxcsr, 16, rx_b):
            print(ln)


def main():
    verbose = ("--bits" in sys.argv) or ("--fields" in sys.argv)
    argv = sys.argv[1:]
    forced = None
    args = []
    for i, a in enumerate(argv):
        if a.startswith("--role="):
            forced = a.split("=", 1)[1]
        elif a == "--role":
            forced = argv[i + 1] if i + 1 < len(argv) else None
        elif a.startswith("--"):
            pass  # other flags (--bits/--fields) already consumed
        else:
            args.append(a)
    if forced not in (None, "host", "device"):
        print("unknown --role '%s' (use host/device/auto)" % forced)
        sys.exit(1)
    if not args:
        print(__doc__)
        sys.exit(1)

    path = Path(args[0])
    raw = path.read_bytes()
    d = Dump(raw)
    print("== %s (%d bytes, assume offset 0 == USBC base, LE) ==" %
          (path.name, len(raw)))

    dev = d.u8(OFF_DEVCTL) or 0
    role = resolve_role(dev, forced)
    decode_role = role if role else "host"   # fall back to host tables if unknown
    hs = (d.u8(OFF_POWER) or 0) & 0x10       # POWER.HSMODE read-only
    bus = "HS" if hs else ("LS" if dev & 0x20 else ("FS" if dev & 0x40 else "?"))

    # ---- common regs ------------------------------------------------------
    print("\n[common]")
    print("FADDR   = 0x%02x" % (d.u8(OFF_FADDR) or 0))
    print("POWER   = 0x%02x  (%s)" % (d.u8(OFF_POWER) or 0, bits(d.u8(OFF_POWER), POWER)))
    print("DEVCTL  = 0x%02x  (%s)" % (dev, bits(dev, DEVCTL)))
    print("mode    = %s%s (bus speed ~%s)" % (
        role or "?", "" if forced else "", bus))
    print("IS      = 0x%02x  (%s)" % (d.u8(OFF_IS) or 0, bits(d.u8(OFF_IS), INTRUSB)))
    print("IE      = 0x%02x" % (d.u8(OFF_IE) or 0))
    txis, rxis = d.u16(OFF_TXIS) or 0, d.u16(OFF_RXIS) or 0
    txie, rxie = d.u16(OFF_TXIE) or 0, d.u16(OFF_RXIE) or 0
    print("TXIS    = 0x%04x pending:%s" % (txis, [i for i in range(16) if txis >> i & 1]))
    print("RXIS    = 0x%04x pending:%s" % (rxis, [i for i in range(16) if rxis >> i & 1]))
    print("TXIE    = 0x%04x armed  :%s" % (txie, [i for i in range(16) if txie >> i & 1]))
    print("RXIE    = 0x%04x armed  :%s" % (rxie, [i for i in range(16) if rxie >> i & 1]))
    print("EPIDX   = 0x%02x  (selected for the 0x10 indexed window)" % (d.u8(OFF_EPIDX) or 0))
    print("frame   = 0x%04x" % (d.u16(OFF_FRAME) or 0))

    if d.have(OFF_EPINFO):
        print("EPINFO  = 0x%02x (TXEP=%d RXEP=%d)" %
              (d.u8(OFF_EPINFO) or 0, (d.u8(OFF_EPINFO) or 0) & 0xF, (d.u8(OFF_EPINFO) or 0) >> 4))
        print("RAMINFO = 0x%02x" % (d.u8(OFF_RAMINFO) or 0))
        print("hwvers  = 0x%04x" % (d.u16(OFF_HWVERS) or 0))

    # ---- per-endpoint banks (flat, alias of the indexed window) -----------
    csr0_m, tx_m, rx_m = masks_for(decode_role)
    print("\n[endpoints]")
    n_eps = min(8, (len(raw) - OFF_EP_BANK) // 16)
    for i in range(max(0, n_eps)):
        b = OFF_EP_BANK + 0x10 * i
        if i == 0:
            csr0 = d.u16(b + 2) or 0
            print("EP0  csr0    =0x%04x (%s)" % (csr0, bits(csr0, csr0_m)))
            continue
        txmaxp, txcsr = d.u16(b + 0) or 0, d.u16(b + 2) or 0
        rxmaxp, rxcsr = d.u16(b + 4) or 0, d.u16(b + 6) or 0
        rxcount = d.u16(b + 8) or 0
        txt, txi = d.u8(b + 0xA), d.u8(b + 0xB)
        rxt, rxi = d.u8(b + 0xC), d.u8(b + 0xD)
        if not (txmaxp or txcsr or rxmaxp or rxcsr or (txt or 0) or (rxt or 0)):
            continue  # bank never programmed, skip
        if decode_role == "host":
            tx_extra = dec_type(txt, "OUT")
            rx_extra = dec_type(rxt, "IN")
        else:
            tx_extra = "send-to-host(IN)" if (txmaxp or txcsr) else "-"
            rx_extra = "recv-from-host(OUT)" if (rxmaxp or rxcsr) else "-"
        print("EP%-2d txmaxp=%5d  TX  csr=0x%04x (%s)  %s" %
              (i, txmaxp, txcsr, bits(txcsr, tx_m), tx_extra))
        print("     rxmaxp=%5d  RX  csr=0x%04x (%s)  %s  rxcount=%d" %
              (rxmaxp, rxcsr, bits(rxcsr, rx_m), rx_extra, rxcount))
        if (txi or 0) or (rxi or 0):
            print("     txintvl=%d rxintvl=%d" % ((txi or 0), (rxi or 0)))

    # ---- DMA --------------------------------------------------------------
    if d.have(OFF_DMAINTR):
        dmaintr = d.u32(OFF_DMAINTR) or 0
        print("\n[dma]  dmaintr=0x%08x" % dmaintr)
        for c in range(8):
            base = OFF_DMA_CH + 0x10 * c
            if not d.have(base):
                break
            cntl = d.u32(base) or 0
            if cntl:
                print("  ch%d cntl=0x%08x addr=0x%08x count=%d" %
                      (c, cntl, d.u32(base + 4) or 0, d.u32(base + 8) or 0))

    # ---- interpretation ---------------------------------------------------
    print("\n[busy/armed interpretation] (role=%s)" % (role or "?"))
    busy = []
    for i in range(1, max(1, n_eps)):
        b = OFF_EP_BANK + 0x10 * i
        txcsr = d.u16(b + 2) or 0
        rxcsr = d.u16(b + 6) or 0
        if decode_role == "host":
            txt, rxt = d.u8(b + 0xA), d.u8(b + 0xC)
            if txcsr & 0x3:  # TXPKTRDY | FIFONOTEMPTY
                busy.append("EP%d TX(out to dev ep%d): packet loaded, not sent (csr=0x%04x)" %
                            (i, txt & 0x0F if txt else -1, txcsr))
            if rxcsr & 0x21:  # RXPKTRDY | REQPKT
                busy.append("EP%d RX(in from dev ep%d): request/data pending (csr=0x%04x)" %
                            (i, rxt & 0x0F if rxt else -1, rxcsr))
            for nm, m in (("NAKTIMEOUT", 0x80), ("RXSTALL", 0x20), ("ERROR", 0x4)):
                if txcsr & m:
                    busy.append("EP%d TX: %s" % (i, nm))
                if rxcsr & (m if nm != "RXSTALL" else 0x40):
                    busy.append("EP%d RX: %s" % (i, nm))
        else:
            if txcsr & 0x3:  # TXPKTRDY | FIFONOTEMPTY
                busy.append("EP%d TX (dev IN->host): packet queued, not sent/acked (csr=0x%04x)" %
                            (i, txcsr))
            if rxcsr & 0x3:  # RXPKTRDY | FIFOFULL
                busy.append("EP%d RX (dev OUT<-host): data arrived, not read (csr=0x%04x)" %
                            (i, rxcsr))
            for nm, m in (("UNDERRUN", 0x4), ("SENTSTALL", 0x20), ("INCOMPTX", 0x80)):
                if txcsr & m:
                    busy.append("EP%d TX: %s" % (i, nm))
            for nm, m in (("OVERRUN", 0x4), ("SENTSTALL", 0x40)):
                if rxcsr & m:
                    busy.append("EP%d RX: %s" % (i, nm))
    # EP0 role-specific hints
    csr0 = d.u16(OFF_EP_BANK + 2) or 0
    if decode_role == "device" and (csr0 & 0x1A):  # SETUPEND(0x10)/DATAEND(0x8)/TXPKTRDY(0x2)
        busy.append("EP0: control state active on csr0=0x%04x (device)" % csr0)
    elif decode_role == "host" and (csr0 & 0x3F):
        busy.append("EP0: control state active on csr0=0x%04x (host)" % csr0)
    if not busy:
        busy.append("no endpoint holds an unfinished packet/request at this instant")
    for line in busy:
        print("  * " + line)

    # maxpacket vs bus-speed sanity (valid for both roles)
    if bus != "HS":
        for i in range(1, max(1, n_eps)):
            b = OFF_EP_BANK + 0x10 * i
            for side, mp in (("TX", d.u16(b) or 0), ("RX", d.u16(b + 4) or 0)):
                if mp > 64:
                    print("  ! EP%d %s maxpacket=%d at ~%s bus (HS-only) -- check" %
                          (i, side, mp, bus))

    if verbose:
        dump_bitfields(d, n_eps, decode_role)


if __name__ == "__main__":
    main()
