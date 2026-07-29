# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

#!/usr/bin/env python3
"""
USB-CDC DFU V2 Tool for SF32LB52 Platform
Implements the DFU V2 protocol (2-byte MSG_ID framing).

Protocol frame format:
  | 0xAA 0x55 | MSG_ID(2B LE) | LENGTH(2B LE) | PAYLOAD(N) | CRC16(2B LE) |

Usage:
    python dfu_v2_cdc_tool.py --port COM3 --firmware hcpu.bin --addr 0x12100000
    python dfu_v2_cdc_tool.py --port COM3 --firmware hcpu.bin lcpu.bin --addr 0x12100000 0x10100000
    python dfu_v2_cdc_tool.py --port COM3 --ping

Migrated from V1.5 dfu_cdc_tool.py:
  - Frame header 0xAA 0x55 preserved
  - CMD(1B) → MSG_ID(2B LE)
  - Command codes: V1.5 custom → V2 dfu_cmd_id_t enum
  - CRC16 scope: over MSG_ID(2B) + LENGTH(2B) + PAYLOAD(N)
"""

import argparse
import struct
import time
import sys
from pathlib import Path

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

# ============================================================================
# V2 Protocol Constants (from dfu_protocol.h)
# ============================================================================

FRAME_HEADER = bytes([0xAA, 0x55])

# Frame field sizes
HEADER_SIZE  = 2  # 0xAA 0x55
MSGID_SIZE   = 2  # uint16_t LE
LENGTH_SIZE  = 2  # uint16_t LE
CRC_SIZE     = 2  # uint16_t LE
FRAME_OVERHEAD = HEADER_SIZE + MSGID_SIZE + LENGTH_SIZE + CRC_SIZE  # 8

# Command IDs (dfu_cmd_id_t)
CMD_INIT_REQ            = 0
CMD_INIT_RSP            = 1
CMD_INIT_COMPLETE_IND   = 2
CMD_RESUME_REQ          = 3
CMD_RESUME_RSP          = 4
CMD_RESUME_COMPLETE_IND = 5
CMD_IMG_SEND_START      = 6
CMD_IMG_SEND_START_RSP  = 7
CMD_IMG_SEND_END        = 8
CMD_IMG_SEND_END_RSP    = 9
CMD_IMG_SEND_PKT        = 10
CMD_IMG_SEND_PKT_RSP    = 11
CMD_TRANS_END           = 12
CMD_END_IND             = 13
CMD_FORCE_INIT_REQ      = 14
CMD_CONN_PRIORITY       = 15
CMD_RETRANS_REQ         = 16
CMD_RETRANS_RSP         = 17
CMD_READ_VER_REQ        = 18
CMD_READ_VER_RSP        = 19
CMD_POWER_OFF           = 20
CMD_ABORT               = 37
CMD_LINK_LOSE_CHECK_REQ = 35
CMD_LINK_LOSE_CHECK_RSP = 36

# Protocol error codes (dfu_proto_err_t)
ERR_NONE                = 0
ERR_GENERAL             = 1
ERR_PARAM_INVALID       = 2
ERR_SPACE_NOT_ENOUGH    = 3
ERR_NOT_READY           = 4
ERR_HW_VER              = 5
ERR_SDK_VER             = 6
ERR_FW_VER              = 7
ERR_FW_INVALID          = 8
ERR_CTRL_PKT_INVALID    = 9
ERR_OTA_ONGOING         = 10
ERR_USER_REJECT         = 11
ERR_UNEXPECT_STATE      = 12
ERR_INDEX_ERROR         = 13
ERR_UPDATING            = 14
ERR_MISS_PACKET         = 15
ERR_FLASH               = 16

ERROR_NAMES = {
    ERR_NONE: "OK",
    ERR_GENERAL: "General error",
    ERR_PARAM_INVALID: "Invalid parameter",
    ERR_SPACE_NOT_ENOUGH: "Space not enough",
    ERR_NOT_READY: "Not ready",
    ERR_HW_VER: "HW version mismatch",
    ERR_SDK_VER: "SDK version mismatch",
    ERR_FW_VER: "FW version mismatch",
    ERR_FW_INVALID: "FW invalid",
    ERR_CTRL_PKT_INVALID: "Control packet invalid",
    ERR_OTA_ONGOING: "OTA ongoing",
    ERR_USER_REJECT: "User rejected",
    ERR_UNEXPECT_STATE: "Unexpected state",
    ERR_INDEX_ERROR: "Index error",
    ERR_UPDATING: "Updating",
    ERR_MISS_PACKET: "Missing packet",
    ERR_FLASH: "Flash error",
}

# Configuration
DEFAULT_BLOCK_SIZE = 2048   # bytes per IMG_SEND_PKT payload (must fit in device ringbuf)
DEFAULT_TIMEOUT    = 5.0    # seconds for normal commands
FLASH_TIMEOUT      = 120.0  # seconds for erase operations
MAX_RETRIES        = 3

# ============================================================================
# CRC Algorithms
# ============================================================================

def crc16_ccitt(data: bytes) -> int:
    """CRC16-CCITT. Polynomial: 0x1021, Init: 0xFFFF."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def crc32(data: bytes) -> int:
    """CRC32. Polynomial: 0xEDB88320 (reflected), Init/XorOut: 0xFFFFFFFF."""
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (c >> 1) ^ 0xEDB88320 if c & 1 else c >> 1
        table.append(c)
    crc = 0xFFFFFFFF
    for byte in data:
        crc = table[(crc ^ byte) & 0xFF] ^ (crc >> 8)
    return crc ^ 0xFFFFFFFF


# ============================================================================
# Protocol Implementation
# ============================================================================

class DfuV2Error(Exception):
    """DFU V2 protocol error."""
    pass


class DfuV2CdcTool:
    """USB-CDC DFU V2 Tool for SF32LB52 platform."""

    def __init__(self, port: str, baudrate: int = 115200, verbose: bool = False):
        self.port = port
        self.baudrate = baudrate
        self.verbose = verbose
        self.serial = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *args):
        self.close()
        return False

    def open(self):
        self.serial = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=FLASH_TIMEOUT,        # set once, don't change per-call
            write_timeout=DEFAULT_TIMEOUT
        )
        self.serial.dtr = True
        time.sleep(0.1)
        self._log(f"Opened {self.port}")

    def close(self):
        if self.serial and self.serial.is_open:
            self.serial.dtr = False
            self.serial.close()
            self._log(f"Closed {self.port}")

    def _log(self, msg: str):
        if self.verbose:
            print(f"[DFU] {msg}")

    # ---- Frame layer ----

    def _build_frame(self, msg_id: int, payload: bytes = b'') -> bytes:
        """Build V2 frame: AA 55 | MSG_ID(2B) | LEN(2B) | PAYLOAD | CRC16(2B)"""
        length = len(payload)
        inner = struct.pack('<HH', msg_id, length) + payload
        crc = crc16_ccitt(inner)
        return FRAME_HEADER + inner + struct.pack('<H', crc)

    def _recv_frame(self, timeout: float = DEFAULT_TIMEOUT) -> tuple:
        """Receive and parse one V2 frame.

        Returns: (msg_id, payload_bytes)
        Raises: DfuV2Error on timeout/CRC/format error
        """
        # Timeout is set once at open() — don't change it here

        # Read header (2 bytes)
        hdr = self.serial.read(2)
        if len(hdr) < 2:
            raise DfuV2Error("Timeout waiting for frame header")
        if hdr != FRAME_HEADER:
            raise DfuV2Error(f"Invalid header: {hdr.hex()}")

        # Read MSG_ID(2) + LENGTH(2) = 4 bytes
        meta = self.serial.read(4)
        if len(meta) < 4:
            raise DfuV2Error("Timeout reading MSG_ID/LENGTH")

        msg_id = struct.unpack('<H', meta[0:2])[0]
        length = struct.unpack('<H', meta[2:4])[0]

        # Read PAYLOAD(length) + CRC(2)
        remaining = length + CRC_SIZE
        rest = self.serial.read(remaining)
        if len(rest) < remaining:
            raise DfuV2Error(f"Incomplete frame: expected {remaining}, got {len(rest)}")

        payload = rest[:length]
        recv_crc = struct.unpack('<H', rest[length:length + 2])[0]

        # Verify CRC over MSG_ID(2) + LENGTH(2) + PAYLOAD(N)
        calc_crc = crc16_ccitt(meta + payload)
        if recv_crc != calc_crc:
            raise DfuV2Error(f"CRC mismatch: recv=0x{recv_crc:04X} calc=0x{calc_crc:04X}")

        self._log(f"RX: msg_id={msg_id} len={length} payload={payload.hex()[:40]}")
        return msg_id, payload

    def _send_recv(self, msg_id: int, payload: bytes = b'',
                   timeout: float = DEFAULT_TIMEOUT) -> tuple:
        """Send a command frame, receive one response frame."""
        frame = self._build_frame(msg_id, payload)
        # Don't call reset_input_buffer or change timeout per-call
        # — Windows CDC virtual serial driver crashes with OSError 121
        self.serial.write(frame)
        self._log(f"TX: msg_id={msg_id} len={len(payload)} frame={frame.hex()[:40]}")
        return self._recv_frame(timeout)

    # ---- Parse response status ----

    @staticmethod
    def _check_status(rsp_payload: bytes, context: str = ""):
        """Parse 1-byte status from response payload, raise on error."""
        if len(rsp_payload) < 1:
            raise DfuV2Error(f"{context}: empty response")
        status = rsp_payload[0]
        if status != ERR_NONE:
            name = ERROR_NAMES.get(status, f"0x{status:02X}")
            raise DfuV2Error(f"{context}: device returned error: {name}")

    # ---- High-level commands ----

    def read_version(self) -> dict:
        """Send READ_VER_REQ, return version dict."""
        rsp_id, payload = self._send_recv(CMD_READ_VER_REQ)
        if rsp_id != CMD_READ_VER_RSP:
            raise DfuV2Error(f"Expected READ_VER_RSP({CMD_READ_VER_RSP}), got {rsp_id}")
        if len(payload) < 12:
            return {"raw": payload.hex()}
        hw  = struct.unpack('<I', payload[0:4])[0]
        sdk = struct.unpack('<I', payload[4:8])[0]
        fw  = struct.unpack('<I', payload[8:12])[0]
        return {"hw": hw, "sdk": sdk, "fw": fw}

    def ping(self) -> bool:
        """Test connection via READ_VER_REQ (no dedicated PING in V2)."""
        try:
            ver = self.read_version()
            self._log(f"PING OK, version: {ver}")
            return True
        except Exception as e:
            self._log(f"PING failed: {e}")
            return False

    def init_session(self, images: list, block_size: int = DEFAULT_BLOCK_SIZE,
                     force: bool = False) -> dict:
        """Send INIT_REQ with file info.

        Args:
            images: list of dict with keys: img_id, size, crc32, flash_addr, flash_size, flag
            block_size: block size for transfer

        Returns:
            INIT_RSP parsed dict
        """
        total_length = sum(img['size'] for img in images)
        total_packets = sum((img['size'] + block_size - 1) // block_size for img in images)
        file_crc = 0  # per-file CRC, device computes its own

        # Build INIT_REQ payload:
        #   dfu_id(1) | total_length(4) | total_packets(4) | file_crc(4) |
        #   block_size(2) | img_count(1) | [img_desc(15B) * N]
        payload = struct.pack('<BIIIH B',
            0,              # dfu_id (0 = default)
            total_length,
            total_packets,
            file_crc,
            block_size,
            len(images),
        )

        # Image descriptors: img_id(1) | flag(2) | img_length(4) | flash_addr(4) | flash_size(4)
        for img in images:
            payload += struct.pack('<BHI II',
                img['img_id'],
                img.get('flag', 0),
                img['size'],
                img['flash_addr'],
                img['flash_size'],
            )

        self._log(f"INIT_REQ: {len(images)} image(s), total={total_length}, "
                  f"packets={total_packets}, block_size={block_size}")

        cmd = CMD_FORCE_INIT_REQ if force else CMD_INIT_REQ
        rsp_id, rsp_payload = self._send_recv(cmd, payload, timeout=FLASH_TIMEOUT)
        if rsp_id != CMD_INIT_RSP:
            raise DfuV2Error(f"Expected INIT_RSP({CMD_INIT_RSP}), got {rsp_id}")
        self._check_status(rsp_payload, "INIT_RSP")

        # Parse resume info from INIT_RSP payload (status(1) + resume fields)
        result = {"status": "ok"}
        if len(rsp_payload) >= 9:
            result["resume_offset"] = struct.unpack('<I', rsp_payload[1:5])[0]
            result["resume_packets"] = struct.unpack('<I', rsp_payload[5:9])[0]
        return result

    def init_complete(self):
        """Send INIT_COMPLETE_IND — tells device to proceed.
        This is an indication (one-way), no response expected."""
        frame = self._build_frame(CMD_INIT_COMPLETE_IND)
        self.serial.reset_input_buffer()
        self.serial.write(frame)
        self._log(f"TX: msg_id={CMD_INIT_COMPLETE_IND} len=0 (IND, no response)")
        time.sleep(0.1)  # Give device a moment to process

    def img_send_start(self, img_index: int = 0):
        """Send IMG_SEND_START for the given image index."""
        payload = struct.pack('<B', img_index)
        rsp_id, rsp_payload = self._send_recv(CMD_IMG_SEND_START, payload, timeout=FLASH_TIMEOUT)
        if rsp_id != CMD_IMG_SEND_START_RSP:
            raise DfuV2Error(f"Expected IMG_SEND_START_RSP({CMD_IMG_SEND_START_RSP}), got {rsp_id}")
        self._check_status(rsp_payload, "IMG_SEND_START")
        self._log(f"IMG_SEND_START OK for image[{img_index}]")

    def img_send_pkt(self, pkt_index: int, data: bytes, retries: int = MAX_RETRIES):
        """Send one data packet (IMG_SEND_PKT)."""
        # Payload: pkt_index(4B LE) + data(NB)
        payload = struct.pack('<I', pkt_index) + data

        for attempt in range(retries + 1):
            try:
                rsp_id, rsp_payload = self._send_recv(CMD_IMG_SEND_PKT, payload)
                if rsp_id != CMD_IMG_SEND_PKT_RSP:
                    raise DfuV2Error(f"Expected PKT_RSP({CMD_IMG_SEND_PKT_RSP}), got {rsp_id}")
                self._check_status(rsp_payload, "IMG_SEND_PKT")
                return
            except DfuV2Error as e:
                if attempt < retries:
                    self._log(f"PKT retry {attempt+1}/{retries}: {e}")
                    continue
                raise

    def img_send_end(self, fw_crc: int):
        """Send IMG_SEND_END with CRC32 — triggers device-side CRC verify."""
        payload = struct.pack('<I', fw_crc)
        rsp_id, rsp_payload = self._send_recv(CMD_IMG_SEND_END, payload, timeout=FLASH_TIMEOUT)
        if rsp_id != CMD_IMG_SEND_END_RSP:
            raise DfuV2Error(f"Expected IMG_SEND_END_RSP({CMD_IMG_SEND_END_RSP}), got {rsp_id}")
        self._check_status(rsp_payload, "IMG_SEND_END (verify)")
        self._log("IMG_SEND_END: verify passed")

    def trans_end(self):
        """Send TRANS_END — commit all images and finish."""
        rsp_id, rsp_payload = self._send_recv(CMD_TRANS_END, timeout=FLASH_TIMEOUT)
        if rsp_id != CMD_END_IND:
            # Some firmware sends TRANS_END response with same ID
            self._log(f"TRANS_END: rsp_id={rsp_id}")
        self._log("TRANS_END: session complete")

    def reboot(self):
        """Send POWER_OFF to trigger device reboot."""
        try:
            self._send_recv(CMD_POWER_OFF, timeout=1.0)
        except:
            pass  # Device may reset before responding
        self._log("Reboot command sent")

    def abort(self):
        """Abort current session."""
        try:
            self._send_recv(CMD_ABORT, timeout=2.0)
        except:
            pass
        self._log("Session aborted")

    # ---- DFU Trigger (for user app → DFU subprogram transition) ----

    def trigger_dfu(self, wait_reboot: float = 5.0) -> bool:
        """Send DFU trigger to user application.

        The user app listens for a simple 4-byte magic:
          TX: AA 55 DF 00  (enter DFU)
          RX: AA 55 DF 01  (ACK, device will reboot)

        After ACK, wait for device to reboot into DFU subprogram.

        Args:
            wait_reboot: seconds to wait for device to reboot and re-enumerate

        Returns:
            True if trigger was acknowledged
        """
        trigger_cmd = bytes([0xAA, 0x55, 0xDF, 0x00])
        self.serial.reset_input_buffer()
        self.serial.timeout = 3.0
        self.serial.write(trigger_cmd)
        self._log(f"TX trigger: {trigger_cmd.hex()}")

        # Read 4-byte ACK
        ack = self.serial.read(4)
        if len(ack) >= 4 and ack[0] == 0xAA and ack[1] == 0x55 and ack[2] == 0xDF and ack[3] == 0x01:
            print("Device acknowledged DFU trigger, rebooting into DFU mode...")
            self.close()
            print(f"Waiting {wait_reboot}s for device to reboot...")
            time.sleep(wait_reboot)
            return True
        else:
            self._log(f"Unexpected trigger response: {ack.hex() if ack else 'timeout'}")
            return False

    # ---- Upload workflow ----

    def upload_firmware(self, firmware_path: str, flash_addr: int,
                        flash_size: int = None, img_id: int = 0,
                        img_index: int = 0, total_images: int = 1,
                        block_size: int = DEFAULT_BLOCK_SIZE,
                        progress_callback=None, force: bool = False) -> None:
        """Upload one firmware image using V2 protocol flow.

        V2 flow:
          INIT_REQ → INIT_RSP
          INIT_COMPLETE_IND (→ device erases flash)
          IMG_SEND_START → IMG_SEND_START_RSP
          IMG_SEND_PKT × N → IMG_SEND_PKT_RSP × N
          IMG_SEND_END → IMG_SEND_END_RSP (CRC verify)
        """
        firmware = Path(firmware_path).read_bytes()
        fw_size = len(firmware)
        fw_crc = crc32(firmware)

        if flash_size is None:
            flash_size = ((fw_size + 0xFFF) // 0x1000) * 0x1000

        print(f"  File:   {firmware_path}")
        print(f"  Size:   {fw_size} bytes ({fw_size/1024:.1f} KB)")
        print(f"  CRC32:  0x{fw_crc:08X}")
        print(f"  Target: 0x{flash_addr:08X}")
        print(f"  Region: 0x{flash_size:X}")

        # --- Only first image sends INIT_REQ ---
        resume_offset = 0
        resume_packets = 0
        if img_index == 0:
            # For simplicity, single-image sessions send one image descriptor.
            # Multi-image: caller should batch all images in one init_session call.
            images = [{
                'img_id': img_id,
                'size': fw_size,
                'crc32': fw_crc,
                'flash_addr': flash_addr,
                'flash_size': flash_size,
                'flag': 0,
            }]
            init_result = self.init_session(images, block_size, force=force)
            self._log(f"INIT_RSP: {init_result}")

            resume_offset = init_result.get("resume_offset", 0)
            resume_packets = init_result.get("resume_packets", 0)

            print("  Sending INIT_COMPLETE (device erasing flash)...")
            self.init_complete()

        # --- Check if transfer already complete (resume case) ---
        total_packets = (fw_size + block_size - 1) // block_size
        if resume_packets >= total_packets and resume_offset >= fw_size:
            print(f"  Resume: all {total_packets} packets already sent, skipping to verify...")
            self.img_send_start(img_index)
            print("  Verifying CRC32...")
            self.img_send_end(fw_crc)
            print("  CRC OK!")
            return

        # --- Start image transfer ---
        self.img_send_start(img_index)

        # --- Send data packets (skip resumed ones) ---
        offset = resume_offset
        pkt_index = resume_packets
        if resume_packets > 0:
            print(f"  Resume: skipping {resume_packets} packets ({resume_offset} bytes)")
        start_time = time.time()

        while offset < fw_size:
            chunk = firmware[offset:offset + block_size]
            self.img_send_pkt(pkt_index, chunk)

            offset += len(chunk)
            pkt_index += 1
            percent = offset * 100 // fw_size
            elapsed = time.time() - start_time
            speed = (offset / 1024) / elapsed if elapsed > 0 else 0

            if progress_callback:
                progress_callback(percent, offset, fw_size)
            else:
                print(f"\r  Progress: {percent}% ({offset}/{fw_size}) - {speed:.1f} KB/s",
                      end='', flush=True)

        print()  # newline

        # --- Verify ---
        print("  Verifying CRC32...")
        self.img_send_end(fw_crc)
        print("  CRC OK!")


def main():
    parser = argparse.ArgumentParser(
        description='USB-CDC DFU V2 Tool for SF32LB52',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
  Single file:
    %(prog)s -p COM3 -f firmware.bin -a 0x12100000

  Multi-file:
    %(prog)s -p COM3 -f hcpu.bin lcpu.bin -a 0x12100000 0x10100000

  Test connection:
    %(prog)s -p COM3 --ping
"""
    )
    parser.add_argument('-p', '--port', required=True, help='Serial port')
    parser.add_argument('-f', '--firmware', nargs='+', help='Firmware file(s)')
    parser.add_argument('-a', '--addr', nargs='+', type=lambda x: int(x, 0),
                        help='Target flash address(es)')
    parser.add_argument('-r', '--region', nargs='+', type=lambda x: int(x, 0),
                        help='Flash region size(s) (optional)')
    parser.add_argument('-c', '--chunk', type=int, default=DEFAULT_BLOCK_SIZE,
                        help=f'Block size (default: {DEFAULT_BLOCK_SIZE})')
    parser.add_argument('-b', '--baudrate', type=int, default=115200)
    parser.add_argument('--ping', action='store_true', help='Test connection only')
    parser.add_argument('--trigger', action='store_true',
                        help='Send DFU trigger to user app before uploading')
    parser.add_argument('--trigger-only', action='store_true',
                        help='Only trigger DFU mode (no upload)')
    parser.add_argument('--wait-reboot', type=float, default=5.0,
                        help='Seconds to wait after trigger for device reboot (default: 5)')
    parser.add_argument('--no-reboot', action='store_true', help='Skip reboot')
    parser.add_argument('-v', '--verbose', action='store_true', help='Verbose output')
    parser.add_argument('--force', action='store_true',
                        help='Force fresh transfer (use FORCE_INIT_REQ to clear resume state)')
    args = parser.parse_args()

    if not args.ping and not args.trigger_only and not args.firmware:
        parser.error("Either --firmware, --ping, or --trigger-only is required")
    if args.firmware and not args.addr:
        parser.error("--addr is required with --firmware")
    if args.firmware and args.addr and len(args.firmware) != len(args.addr):
        parser.error("Number of firmware files must match addresses")
    if args.region and len(args.region) != len(args.firmware):
        parser.error("Number of region sizes must match firmware files")

    try:
        with DfuV2CdcTool(args.port, args.baudrate, args.verbose) as dfu:
            if args.ping:
                if dfu.ping():
                    print("Connection OK")
                    return 0
                else:
                    print("Connection FAILED")
                    return 1

            # Trigger-only mode: just send trigger and exit
            if args.trigger_only:
                if dfu.trigger_dfu(args.wait_reboot):
                    print("Device should now be in DFU mode.")
                    return 0
                else:
                    print("Trigger FAILED")
                    return 1

            # Auto-trigger: if --trigger, send trigger first then reconnect
            if args.trigger:
                print(f"Triggering DFU mode on {args.port}...")
                if not dfu.trigger_dfu(args.wait_reboot):
                    print("Trigger failed, aborting")
                    return 1
                # Reconnect to the DFU subprogram
                print(f"Reconnecting to {args.port}...")
                dfu.open()

            print(f"Starting DFU V2 session on {args.port}...")

            if not dfu.ping():
                print("WARNING: PING failed, continuing anyway...")

            total_files = len(args.firmware)
            start_time = time.time()

            for i, (fw_path, addr) in enumerate(zip(args.firmware, args.addr)):
                region = args.region[i] if args.region else None
                print(f"\n=== Uploading image {i+1}/{total_files} ===")
                dfu.upload_firmware(fw_path, addr, region,
                                   img_id=i, img_index=i,
                                   total_images=total_files,
                                   block_size=args.chunk,
                                   force=args.force)

            # TRANS_END to commit
            print("\nCommitting...")
            dfu.trans_end()

            total_time = time.time() - start_time
            print(f"\n=== Update complete in {total_time:.1f}s! ===")

            if not args.no_reboot:
                print("Rebooting device...")
                dfu.reboot()
            else:
                print("Skipping reboot (--no-reboot)")

            return 0

    except DfuV2Error as e:
        print(f"\nDFU ERROR: {e}")
        return 1
    except serial.SerialException as e:
        print(f"\nSerial ERROR: {e}")
        return 1
    except KeyboardInterrupt:
        print("\n\nAborted by user")
        return 1
    except Exception as e:
        print(f"\nERROR: {e}")
        if args.verbose:
            import traceback
            traceback.print_exc()
        return 1


if __name__ == '__main__':
    sys.exit(main())