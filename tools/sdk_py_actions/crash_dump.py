# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import configparser
import hashlib
import json
import os
import re
import shutil
import struct
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from sdk_py_actions.errors import FatalError
from sdk_py_actions.errors import UsageError


HEADER_SIZE = 24
MINIDUMP_TAIL_MAGIC = 0xF1EFEFEF
RAMLOG_MINIMUM_MAGIC = 0xEFEFEFEF
RAMLOG_REMAINING_MAGIC = 0xDFDFDFDF
_TAIL_MAGICS = {MINIDUMP_TAIL_MAGIC}
_RAMLOG_MAGICS = {
    RAMLOG_MINIMUM_MAGIC: "minimum",
    RAMLOG_REMAINING_MAGIC: "remaining",
}
_CHIP_RE = re.compile(r"^(52X|55X|56X|57X|58X)_(.*)$")
_SAVEBIN_RE = re.compile(
    r"^\s*savebin\s+(\S+)\s+(0x[0-9a-fA-F]+|\d+)\s+(0x[0-9a-fA-F]+|\d+)\s*$",
    re.IGNORECASE,
)
_UART_READ_CHUNK_SIZE = 32 * 1024
_PROGRESS_WIDTH = 20
SDK_MANIFEST_NAME = "sdk_manifest.json"
CORE_ELF_NAME = "coredump.elf"


@dataclass(frozen=True)
class Region:
    filename: str
    address: int
    size: int


@dataclass(frozen=True)
class ElfSymbol:
    name: str
    address: int
    size: int
    kind: str


class MemoryImage:
    def __init__(self, blocks: Iterable[Dict[str, Any]]) -> None:
        self._blocks = sorted(
            ((int(block["address"]), bytes(block["data"])) for block in blocks),
            key=lambda item: item[0],
        )

    def read(self, address: int, size: int) -> Optional[bytes]:
        for start, data in self._blocks:
            offset = address - start
            if offset >= 0 and offset + size <= len(data):
                return data[offset : offset + size]
        return None

    def read_available(self, address: int, limit: int) -> bytes:
        for start, data in self._blocks:
            offset = address - start
            if 0 <= offset < len(data):
                return data[offset : offset + limit]
        return b""


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _show_progress(name: str, done: int, total: int) -> None:
    percent = 100 if total <= 0 else min(100, done * 100 // total)
    filled = _PROGRESS_WIDTH * percent // 100
    bar = "#" * filled + "-" * (_PROGRESS_WIDTH - filled)
    print(
        f"export {name} [{bar}] {percent:3d}% {done}/{total} bytes",
        file=sys.stderr,
        flush=True,
    )


def _sdk_root(sdk_ctx: Any) -> Path:
    env = getattr(sdk_ctx, "env", None)
    if isinstance(env, dict):
        value = env.get("SIFLI_SDK_PATH")
        if value:
            return Path(value)
    value = os.environ.get("SIFLI_SDK_PATH")
    return Path(value) if value else Path.cwd()


def _parse_header(raw: bytes) -> Tuple[str, str]:
    if len(raw) < HEADER_SIZE:
        raise ValueError(f"coredump is shorter than the {HEADER_SIZE}-byte header")
    text = raw[:HEADER_SIZE].split(b"\0", 1)[0].decode("ascii", errors="strict")
    match = _CHIP_RE.match(text)
    if not match:
        raise ValueError(f"invalid coredump header: {text!r}")
    return match.group(1), match.group(2)


def parse_coredump(raw: bytes) -> Dict[str, Any]:
    chip, timestamp = _parse_header(raw)
    blocks: List[Dict[str, Any]] = []
    ramlogs: List[Dict[str, Any]] = []
    warnings: List[str] = []
    complete = True
    offset = HEADER_SIZE

    while offset + 8 <= len(raw):
        address, length = struct.unpack_from("<II", raw, offset)
        offset += 8

        if address == 0xFFFFFFFF and length == 0xFFFFFFFF:
            break
        if address in _TAIL_MAGICS:
            break
        if length == 0:
            warnings.append(f"zero-length block at offset 0x{offset - 8:x}")
            complete = False
            break
        if length > len(raw) - offset:
            warnings.append(
                f"truncated block at 0x{address:08x}: expected {length} bytes, "
                f"found {len(raw) - offset}"
            )
            complete = False
            break

        data = raw[offset : offset + length]
        if address in _RAMLOG_MAGICS:
            ramlogs.append({"kind": _RAMLOG_MAGICS[address], "data": data})
        else:
            blocks.append({"address": address, "data": data})
        offset += length

    trailing = raw[offset:]
    if trailing and len(trailing) < 8 and any(byte != 0xFF for byte in trailing):
        warnings.append(f"trailing {len(trailing)} byte(s) after final block")
        complete = False

    return {
        "chip": chip,
        "timestamp": timestamp,
        "blocks": blocks,
        "ramlogs": ramlogs,
        "warnings": warnings,
        "complete": complete,
    }


def _attach_file(output_dir: Path, source_path: Path) -> Dict[str, Any]:
    source_path = Path(source_path).resolve()
    attachments_dir = output_dir / "attachments"
    attachments_dir.mkdir(parents=True, exist_ok=True)
    destination = attachments_dir / source_path.name
    if source_path != destination.resolve():
        shutil.copy2(source_path, destination)
    data = destination.read_bytes()
    return {"file": f"attachments/{destination.name}", "size": len(data), "sha256": _sha256(data)}


def _manifest_file_path(
    package_dir: Path, item: Dict[str, Any], label: str
) -> Path:
    relative = item.get("file")
    if not isinstance(relative, str) or not relative:
        raise FatalError(f"{label} has no valid file path")
    package_dir = Path(package_dir).resolve()
    path = (package_dir / relative).resolve()
    try:
        path.relative_to(package_dir)
    except ValueError as exc:
        raise FatalError(f"{label} file escapes package directory: {relative}") from exc
    if not path.is_file():
        raise FatalError(f"{label} file is missing: {relative}")
    data = path.read_bytes()
    if "size" in item and len(data) != item["size"]:
        raise FatalError(
            f"{label} size mismatch: expected {item['size']}, found {len(data)}"
        )
    if "sha256" in item and _sha256(data) != item["sha256"]:
        raise FatalError(f"{label} sha256 mismatch: {relative}")
    return path


def validate_package(package_dir: Path) -> Dict[str, Any]:
    package_dir = Path(package_dir).resolve()
    manifest_path = (
        package_dir / SDK_MANIFEST_NAME
        if (package_dir / SDK_MANIFEST_NAME).is_file()
        else package_dir / "manifest.json"
    )
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise FatalError(f"Invalid crash package manifest: {exc}") from exc
    if manifest.get("schema_version") != 1:
        raise FatalError(
            f"Unsupported crash package schema: {manifest.get('schema_version')}"
        )
    if not isinstance(manifest.get("segments"), list):
        raise FatalError("Crash package manifest has no segment list")

    entries = []
    if isinstance(manifest.get("raw"), dict):
        entries.append(("raw", manifest["raw"]))
    entries.extend(
        (f"segment[{index}]", item)
        for index, item in enumerate(manifest["segments"])
    )
    entries.extend(
        (f"ramlog[{index}]", item)
        for index, item in enumerate(manifest.get("ramlogs", []))
    )
    for name in ("elf", "log", "debug_log"):
        if isinstance(manifest.get(name), dict):
            entries.append((name, manifest[name]))
    for label, item in entries:
        if not isinstance(item, dict):
            raise FatalError(f"{label} metadata is invalid")
        _manifest_file_path(package_dir, item, label)
    return manifest


def _attached_elf_path(
    package_dir: Path, manifest: Dict[str, Any], elf_path: Optional[Path]
) -> Path:
    attached_elf = (
        _manifest_file_path(package_dir, manifest["elf"], "elf")
        if isinstance(manifest.get("elf"), dict)
        else None
    )
    if elf_path is None:
        if attached_elf is None:
            raise UsageError("--elf is required when the package has no attached ELF")
        return attached_elf
    resolved = Path(elf_path).resolve()
    if (
        attached_elf is not None
        and _sha256(resolved.read_bytes()) != manifest["elf"]["sha256"]
    ):
        raise FatalError("ELF sha256 does not match the crash package")
    return resolved


def _assertdump_manifest(
    package_dir: Path,
    manifest: Dict[str, Any],
    regions: Sequence[Region],
    files: Sequence[str],
    missing: Sequence[str],
) -> Dict[str, Any]:
    file_set = set(files)
    missing_set = set(missing)
    entries = []
    items = []
    for index, region in enumerate(regions):
        item_id = f"region-{index}"
        selected = region.filename in file_set and region.filename not in missing_set
        kind = (
            "memoryRegion"
            if region.filename.endswith("_ram.bin") or "psram" in region.filename
            else "registerBlock"
        )
        item: Dict[str, Any] = {
            "id": item_id,
            "kind": kind,
            "name": region.filename,
            "address": region.address,
            "size": region.size,
            "fileName": region.filename,
            "selected": selected,
            "source": "sdk.py",
        }
        if item["kind"] == "memoryRegion":
            item["memoryKind"] = Path(region.filename).stem
        items.append(item)
        path = package_dir / region.filename
        if selected and path.is_file():
            entries.append(
                {
                    "itemId": item_id,
                    "fileName": region.filename,
                    "path": region.filename,
                    "status": "ok",
                    "size": path.stat().st_size,
                }
            )
    return {
        "schemaVersion": "1.0.0",
        "createdAt": manifest.get(
            "captured_at", datetime.now(timezone.utc).isoformat()
        ),
        "session": {"source": manifest.get("source", "sdk.py")},
        "chip": {"modelId": manifest.get("chip")},
        "items": items,
        "files": entries,
        "peripherals": {},
        "warnings": list(manifest.get("warnings", [])),
    }


def write_package(
    raw: bytes,
    output_dir: Path,
    source: str,
    elf_path: Optional[Path] = None,
    log_path: Optional[Path] = None,
) -> Dict[str, Any]:
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    parsed = parse_coredump(raw)

    raw_path = output_dir / "coredump.bin"
    if not raw_path.exists() or raw_path.read_bytes() != raw:
        raw_path.write_bytes(raw)

    segment_dir = output_dir / "segments"
    segment_dir.mkdir(exist_ok=True)
    segments = []
    for index, block in enumerate(parsed["blocks"]):
        address = int(block["address"])
        data = bytes(block["data"])
        relative_path = Path("segments") / f"{index:03d}-0x{address:08x}.bin"
        (output_dir / relative_path).write_bytes(data)
        segments.append(
            {
                "address": f"0x{address:08x}",
                "size": len(data),
                "file": relative_path.as_posix(),
                "sha256": _sha256(data),
            }
        )

    ramlog_dir = output_dir / "ramlogs"
    ramlog_dir.mkdir(exist_ok=True)
    ramlogs = []
    for item in parsed["ramlogs"]:
        data = bytes(item["data"])
        relative_path = Path("ramlogs") / f"{item['kind']}.bin"
        (output_dir / relative_path).write_bytes(data)
        ramlogs.append(
            {
                "kind": item["kind"],
                "size": len(data),
                "file": relative_path.as_posix(),
                "sha256": _sha256(data),
            }
        )

    manifest = {
        "schema_version": 1,
        "source": source,
        "chip": normalize_chip(parsed["chip"]),
        "captured_at": parsed["timestamp"],
        "complete": parsed["complete"],
        "warnings": parsed["warnings"],
        "raw": {
            "file": raw_path.name,
            "size": len(raw),
            "sha256": _sha256(raw),
        },
        "segments": segments,
        "ramlogs": ramlogs,
    }
    if elf_path:
        manifest["elf"] = _attach_file(output_dir, elf_path)
    if log_path:
        manifest["log"] = _attach_file(output_dir, log_path)
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return manifest


def write_legacy_layout(
    package_dir: Path,
    manifest: Dict[str, Any],
    regions: Sequence[Region],
    elf_path: Optional[Path] = None,
    debug_log_path: Optional[Path] = None,
) -> Dict[str, Any]:
    package_dir = Path(package_dir).resolve()
    image = MemoryImage(
        {
            "address": int(segment["address"], 0),
            "data": (package_dir / segment["file"]).read_bytes(),
        }
        for segment in manifest["segments"]
    )
    files = []
    missing = []
    for region in regions:
        destination = (package_dir / region.filename).resolve()
        try:
            destination.relative_to(package_dir)
        except ValueError as exc:
            raise FatalError(
                f"legacy output file escapes package directory: {region.filename}"
            ) from exc
        data = (
            destination.read_bytes()
            if destination.is_file() and destination.stat().st_size == region.size
            else image.read(region.address, region.size)
        )
        if data is None:
            destination.unlink(missing_ok=True)
            missing.append(region.filename)
            continue
        destination.write_bytes(data)
        files.append(region.filename)

    if elf_path:
        destination = package_dir / "hcpu.axf"
        source = Path(elf_path).resolve()
        if source != destination:
            shutil.copy2(source, destination)
        files.append(destination.name)
    if debug_log_path:
        destination = package_dir / "log.txt"
        source = Path(debug_log_path).resolve()
        if source != destination:
            shutil.copy2(source, destination)
        files.append(destination.name)

    legacy = {"files": files, "missing": missing}
    manifest["legacy"] = legacy
    (package_dir / SDK_MANIFEST_NAME).write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    (package_dir / "manifest.json").write_text(
        json.dumps(
            _assertdump_manifest(package_dir, manifest, regions, files, missing),
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return legacy


def decode_saved_stack_frame(data: bytes) -> Dict[str, int]:
    if len(data) < 32:
        raise ValueError("saved_stack_frame is shorter than its exception frame")
    if 32 < len(data) < 68:
        raise ValueError("saved_stack_frame must be exactly 32 or at least 68 bytes")
    if len(data) >= 68:
        # RT-Thread struct stack_frame: psplim + r4..r11 + exception_stack_frame
        except_words = struct.unpack("<8I", data[36:68])
        except_names = ("r0", "r1", "r2", "r3", "r12", "lr", "pc", "xpsr")
        registers = dict(zip(except_names, except_words))
        for i in range(8):
            val = struct.unpack("<I", data[4 + i * 4 : 4 + (i + 1) * 4])[0]
            registers[f"r{i+4}"] = val
        return registers
    values = struct.unpack("<8I", data[-32:])
    names = ("r0", "r1", "r2", "r3", "r12", "lr", "pc", "xpsr")
    return dict(zip(names, values))


def decode_scb_registers(data: bytes) -> Dict[str, int]:
    if len(data) < 20:
        raise ValueError("saved_scb_reg is shorter than 20 bytes")
    names = ("cfsr", "hfsr", "mmfar", "bfar", "shcsr")
    return dict(zip(names, struct.unpack("<5I", data[:20])))


def parse_jlink_registers(text: str) -> Dict[str, int]:
    registers = {}
    aliases = {"r13": "sp", "r14": "lr", "r15": "pc"}
    pattern = re.compile(
        r"\b(R(?:1[0-5]|[0-9])|SP|LR|PC|XPSR)\b"
        r"(?:\s*\([^)]*\))?\s*=\s*(?:0x)?([0-9a-fA-F]{1,8})",
        re.IGNORECASE,
    )
    for match in pattern.finditer(text):
        name = match.group(1).lower()
        registers[aliases.get(name, name)] = int(match.group(2), 16)
    return registers


def _function_symbol(address: int, symbols: Sequence[ElfSymbol]) -> Optional[ElfSymbol]:
    for symbol in symbols:
        if (
            symbol.kind == "function"
            and symbol.address <= address < symbol.address + max(symbol.size, 1)
        ):
            return symbol
    return None


def symbolize(address: int, symbols: Sequence[ElfSymbol]) -> Optional[str]:
    symbol = _function_symbol(address, symbols)
    if symbol is None and address & 1:
        symbol = _function_symbol(address - 1, symbols)
    if symbol is None:
        return None
    return f"{symbol.name}+0x{address - symbol.address:x}"


def find_stack_candidates(
    data: bytes, symbols: Sequence[ElfSymbol]
) -> List[Dict[str, Any]]:
    candidates = []
    seen = set()
    for offset in range(0, len(data) - 3, 4):
        address = struct.unpack_from("<I", data, offset)[0]
        name = symbolize(address, symbols)
        if name is not None and address not in seen:
            candidates.append(
                {"offset": offset, "address": f"0x{address:08x}", "symbol": name}
            )
            seen.add(address)
    return candidates


def load_elf_symbols(path: Path) -> List[ElfSymbol]:
    try:
        from elftools.elf.elffile import ELFFile
    except ImportError as exc:
        raise FatalError(
            "pyelftools is required for crash-dump analysis. "
            "Run sdk.py from an exported SiFli SDK environment."
        ) from exc

    symbols: Dict[Tuple[str, int, int, str], ElfSymbol] = {}
    with Path(path).open("rb") as stream:
        elf = ELFFile(stream)
        for section in elf.iter_sections():
            if section.header.sh_type not in ("SHT_SYMTAB", "SHT_DYNSYM"):
                continue
            for item in section.iter_symbols():
                name = item.name
                address = int(item.entry.st_value)
                size = int(item.entry.st_size)
                symbol_type = item.entry.st_info.type
                if not name or not address:
                    continue
                if symbol_type == "STT_FUNC":
                    kind = "function"
                elif symbol_type == "STT_OBJECT":
                    kind = "object"
                else:
                    continue
                symbol = ElfSymbol(name, address, size, kind)
                symbols[(name, address, size, kind)] = symbol
    return sorted(symbols.values(), key=lambda symbol: (symbol.address, symbol.name))


def _find_named_symbol(symbols: Sequence[ElfSymbol], name: str) -> Optional[ElfSymbol]:
    return next(
        (
            symbol
            for symbol in symbols
            if symbol.name == name and symbol.kind == "object"
        ),
        None,
    )


def _read_named_object(
    image: MemoryImage,
    symbols: Sequence[ElfSymbol],
    name: str,
    minimum_size: int,
) -> Optional[bytes]:
    symbol = _find_named_symbol(symbols, name)
    if symbol is None:
        return None
    size = max(symbol.size, minimum_size)
    return image.read(symbol.address, size)


def _manifest_blocks(package_dir: Path, manifest: Dict[str, Any]) -> List[Dict[str, Any]]:
    return [
        {
            "address": int(segment["address"], 0),
            "data": (package_dir / segment["file"]).read_bytes(),
        }
        for segment in manifest["segments"]
    ]


def _manifest_registers(manifest: Dict[str, Any], key: str = "registers") -> Dict[str, int]:
    registers = manifest.get(key)
    if not isinstance(registers, dict):
        return {}
    result = {}
    for name, value in registers.items():
        if isinstance(value, str):
            result[name.lower()] = int(value, 0)
        elif isinstance(value, int):
            result[name.lower()] = value
    return result


def _log_registers(package_dir: Path, manifest: Dict[str, Any]) -> Dict[str, int]:
    for log_name in ("debug_log", "log"):
        if not isinstance(manifest.get(log_name), dict):
            continue
        log_file = _manifest_file_path(package_dir, manifest[log_name], log_name)
        registers = parse_jlink_registers(
            log_file.read_text(encoding="utf-8", errors="replace")
        )
        if registers.get("pc"):
            return registers
    return {}


def _align(data: bytes, size: int = 4) -> bytes:
    return data + b"\0" * ((size - len(data) % size) % size)


def _elf_note(name: bytes, note_type: int, desc: bytes) -> bytes:
    return (
        struct.pack("<III", len(name), len(desc), note_type)
        + _align(name)
        + _align(desc)
    )


def _error_reason_to_signal(reason: int) -> int:
    """Map RT-Thread error_reason code to UNIX signal number.

    RT_ERROR_NO_ERROR=0     -> 0
    RT_ERROR_ASSERT=1       -> SIGABRT (6)
    RT_ERROR_HARD_FAULT=2   -> SIGSEGV (11)
    RT_ERROR_BUS_FAULT=3    -> SIGBUS (7)
    RT_ERROR_MEMFAULT=4     -> SIGSEGV (11)
    RT_ERROR_HW_EXCEPTION=5 -> SIGSEGV (11)
    RT_ERROR_UNKNOWN_CAUSE=6 -> SIGABRT (6)
    """
    _SIGNAL_MAP: Dict[int, int] = {
        0: 0,
        1: 6,
        2: 11,
        3: 7,
        4: 11,
        5: 11,
        6: 6,
    }
    return _SIGNAL_MAP.get(reason, 6)


def _core_prstatus(registers: Dict[str, int], error_reason: int = 0) -> bytes:
    signo = _error_reason_to_signal(error_reason)
    sp = registers.get("sp", registers.get("r13", 0))
    lr = registers.get("lr", registers.get("r14", 0))
    pc = registers.get("pc", registers.get("r15", 0))
    xpsr = registers.get("xpsr", 0)
    words = [
        signo,   # si_signo — from firmware error_reason
        0,       # si_code
        0,       # si_errno
        signo,   # pr_cursig
        0,       # pr_sigpend
        0,       # pr_sighold
        1,       # pr_pid — sentinel value for crash dump
        0,       # pr_ppid
        1,       # pr_pgrp
        1,       # pr_sid
        0, 0,    # pr_utime
        0, 0,    # pr_stime
        0, 0,    # pr_cutime
        0, 0,    # pr_cstime
    ]
    words.extend(registers.get(f"r{index}", 0) for index in range(13))
    words.extend(
        [
            sp,
            lr,
            pc,
            xpsr,
            0,
            0,
        ]
    )
    return struct.pack(f"<{len(words)}I", *words)


def _core_prpsinfo(name: str) -> bytes:
    return struct.pack(
        "<BBBBIHHIIII16s80s",
        0,
        ord("R"),
        0,
        0,
        0,
        0,
        0,
        1,
        0,
        1,
        1,
        b"sifli.elf".ljust(16, b"\0"),
        name.encode(errors="replace")[:80].ljust(80, b"\0"),
    )


def write_core_elf(
    package_dir: Path,
    elf_path: Optional[Path] = None,
    output_path: Optional[Path] = None,
) -> Path:
    package_dir = Path(package_dir).resolve()
    manifest = validate_package(package_dir)
    elf = _attached_elf_path(package_dir, manifest, elf_path)
    output = Path(output_path).resolve() if output_path else package_dir / CORE_ELF_NAME
    blocks = _manifest_blocks(package_dir, manifest)
    registers = _manifest_registers(manifest) or _log_registers(package_dir, manifest)
    error_reason = 0
    if not registers:
        image = MemoryImage(blocks)
        symbols = None
        try:
            symbols = load_elf_symbols(elf)
        except FatalError as exc:
            manifest.setdefault("warnings", []).append(
                f"pyelftools is required to extract CPU registers from ELF symbols: {exc}"
            )
        frame = _read_named_object(image, symbols, "saved_stack_frame", 32) if symbols else None
        if frame is not None:
            registers = decode_saved_stack_frame(frame)
            sp_data = _read_named_object(image, symbols, "saved_stack_pointer", 4)
            if sp_data is not None and len(sp_data) >= 4:
                registers["sp"] = struct.unpack("<I", sp_data[:4])[0]
            reason_data = _read_named_object(image, symbols, "error_reason", 4)
            if reason_data is not None and len(reason_data) >= 4:
                error_reason = struct.unpack("<I", reason_data[:4])[0]
        else:
            registers = {}

    notes = (
        _elf_note(b"CORE\0", 1, _core_prstatus(registers, error_reason))
        + _elf_note(b"CORE\0", 3, _core_prpsinfo(manifest.get("chip", "SiFli")))
    )
    pieces = [notes] + [bytes(block["data"]) for block in blocks]
    phnum = len(pieces)
    ehdr_size = 52
    phdr_size = 32
    offset = ehdr_size + phnum * phdr_size
    phdrs = []
    for index, data in enumerate(pieces):
        kind = 4 if index == 0 else 1
        address = 0 if index == 0 else int(blocks[index - 1]["address"])
        phdrs.append(
            struct.pack(
                "<8I",
                kind,
                offset,
                address,
                0,
                len(data),
                0 if kind == 4 else len(data),
                7,
                4,
            )
        )
        offset += len(data)

    ident = b"\x7fELF" + bytes([1, 1, 1, 0]) + b"\0" * 8
    header = ident + struct.pack(
        "<HHIIIIIHHHHHH",
        4,
        40,
        1,
        0,
        ehdr_size,
        0,
        0x5000200,
        ehdr_size,
        phdr_size,
        phnum,
        40,
        0,
        0,
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(header + b"".join(phdrs) + b"".join(pieces))
    manifest["core_elf"] = {
        "file": output.name if output.parent == package_dir else str(output),
        "size": output.stat().st_size,
        "sha256": _sha256(output.read_bytes()),
    }
    (package_dir / SDK_MANIFEST_NAME).write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return output


def _find_gdb() -> Optional[str]:
    for name in ("arm-none-eabi-gdb-py", "arm-none-eabi-gdb-py3", "arm-none-eabi-gdb"):
        path = shutil.which(name)
        if path:
            return path
    return None


def analyze_core_elf(
    core_path: Path,
    elf_path: Path,
    output_path: Optional[Path] = None,
    package_dir: Optional[Path] = None,
) -> Optional[Dict[str, Any]]:
    gdb = _find_gdb()
    if not gdb:
        return None
    core = Path(core_path).resolve()
    elf = Path(elf_path).resolve()
    script = (Path(package_dir).resolve() if package_dir else core.parent) / "analyze.gdb"
    quoted_elf = str(elf).replace("\\", "\\\\").replace('"', '\\"')
    quoted_core = str(core).replace("\\", "\\\\").replace('"', '\\"')
    script.write_text(
        "\n".join(
            [
                f'file "{quoted_elf}"',
                f'core-file "{quoted_core}"',
                "set backtrace limit 50",
                "set print elements 256",
                "echo \\n=== THREADS ===\\n",
                "info threads",
                "thread apply all bt",
                "echo \\n=== REGISTERS ===\\n",
                "info registers",
                "echo \\n=== STACK ===\\n",
                "x/32xw $sp",
                "quit",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    completed = subprocess.run(
        [gdb, "--batch", "--quiet", "-x", str(script)],
        capture_output=True,
        text=True,
        check=False,
        timeout=60,
    )
    analysis = {
        "schema_version": 1,
        "analysis_engine": "gdb",
        "core_elf": str(core),
        "elf": {"file": str(elf), "sha256": _sha256(elf.read_bytes())},
        "gdb_returncode": completed.returncode,
        "gdb_report": completed.stdout,
        "warnings": [],
    }
    if completed.stderr:
        analysis["warnings"].append(completed.stderr)
    destination = Path(output_path) if output_path else core.parent / "analysis.json"
    destination.write_text(
        json.dumps(analysis, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return analysis


def analyze_package(
    package_dir: Path,
    elf_path: Optional[Path] = None,
    output_path: Optional[Path] = None,
    engine: str = "auto",
) -> Dict[str, Any]:
    package_dir = Path(package_dir).resolve()
    manifest = validate_package(package_dir)
    elf_path = _attached_elf_path(package_dir, manifest, elf_path)
    core_path = package_dir / CORE_ELF_NAME
    gdb_analysis: Optional[Dict[str, Any]] = None
    if engine == "auto" and core_path.is_file():
        gdb_analysis = analyze_core_elf(
            core_path, elf_path, output_path, package_dir=package_dir
        )
        if gdb_analysis is not None and gdb_analysis.get("gdb_returncode", 0) == 0:
            return gdb_analysis
        # GDB had errors (e.g. corrupted SP from heap overflow) —
        # fall through to Python engine for supplemental analysis.
    elif engine == "gdb":
        if not core_path.is_file():
            write_core_elf(package_dir, elf_path, core_path)
        result = analyze_core_elf(
            core_path, elf_path, output_path, package_dir=package_dir
        )
        if result is None:
            raise FatalError("arm-none-eabi-gdb was not found")
        return result
    blocks = _manifest_blocks(package_dir, manifest)
    image = MemoryImage(blocks)
    symbols = load_elf_symbols(elf_path)
    analysis: Dict[str, Any] = {
        "schema_version": 1,
        "analysis_engine": "python",
        "chip": manifest["chip"],
        "elf": {
            "file": str(Path(elf_path).resolve()),
            "sha256": _sha256(Path(elf_path).read_bytes()),
        },
        "warnings": [],
    }

    current_registers = _manifest_registers(manifest, "current_registers")
    legacy_registers = _manifest_registers(manifest)
    saved_registers = {}
    frame_data = _read_named_object(image, symbols, "saved_stack_frame", 32)
    if frame_data is not None:
        saved_registers = decode_saved_stack_frame(frame_data)
        if saved_registers:
            analysis["saved_registers"] = {
                name: f"0x{value:08x}" for name, value in saved_registers.items()
            }
            analysis["saved_register_source"] = "saved_stack_frame"
    elif frame_data is None:
        analysis["warnings"].append("saved_stack_frame is absent from the dump")

    registers = current_registers or legacy_registers or saved_registers
    if current_registers:
        register_source = manifest.get("current_register_source")
    elif legacy_registers:
        register_source = manifest.get("register_source")
    elif saved_registers:
        register_source = "saved_stack_frame"
    else:
        register_source = None
    if current_registers:
        analysis["current_registers"] = {
            name: f"0x{value:08x}" for name, value in current_registers.items()
        }
        analysis["current_register_source"] = register_source
    if not registers.get("pc"):
        for log_name in ("debug_log", "log"):
            if not isinstance(manifest.get(log_name), dict):
                continue
            log_file = _manifest_file_path(package_dir, manifest[log_name], log_name)
            log_registers = parse_jlink_registers(
                log_file.read_text(encoding="utf-8", errors="replace")
            )
            if log_registers.get("pc"):
                registers = log_registers
                register_source = log_name
                analysis["warnings"].append(
                    f"registers restored from {log_name}"
                )
                break
    if registers:
        analysis["registers"] = {
            name: f"0x{value:08x}" for name, value in registers.items()
        }
        analysis["register_source"] = register_source
        if "pc" in registers:
            analysis["pc"] = {
                "address": f"0x{registers['pc']:08x}",
                "symbol": symbolize(registers["pc"], symbols),
            }
        if "lr" in registers:
            analysis["lr"] = {
                "address": f"0x{registers['lr']:08x}",
                "symbol": symbolize(registers["lr"], symbols),
            }
    scb_data = _read_named_object(image, symbols, "saved_scb_reg", 20)
    if scb_data is not None:
        analysis["scb"] = {
            name: f"0x{value:08x}"
            for name, value in decode_scb_registers(scb_data).items()
        }

    reason_data = _read_named_object(image, symbols, "error_reason", 4)
    if reason_data is not None:
        analysis["error_reason"] = struct.unpack("<I", reason_data[:4])[0]

    sp_data = _read_named_object(image, symbols, "saved_stack_pointer", 4)
    stack_pointer = struct.unpack("<I", sp_data[:4])[0] if sp_data is not None else 0
    if not stack_pointer:
        stack_pointer = registers.get("sp", 0)
    if stack_pointer:
        analysis["stack_pointer"] = f"0x{stack_pointer:08x}"
        stack = image.read_available(stack_pointer, 256)
        analysis["stack_candidates"] = find_stack_candidates(stack, symbols)

    # Merge partial GDB results when GDB errored (e.g. corrupted SP) but
    # still produced useful register / backtrace data.
    if gdb_analysis is not None:
        analysis["analysis_engine"] = "auto-gdb-partial"
        analysis["gdb_partial"] = {
            "gdb_returncode": gdb_analysis.get("gdb_returncode"),
            "gdb_report": gdb_analysis.get("gdb_report"),
        }
        analysis["elf"] = gdb_analysis.get("elf", analysis["elf"])
        for w in gdb_analysis.get("warnings", []):
            if w not in analysis["warnings"]:
                analysis["warnings"].append(w)
    destination = Path(output_path) if output_path else package_dir / "analysis.json"
    destination.write_text(
        json.dumps(analysis, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return analysis


def parse_jlink_regions(text: str) -> List[Region]:
    regions = []
    for line in text.splitlines():
        if line.lstrip().startswith("//"):
            continue
        match = _SAVEBIN_RE.match(line)
        if match:
            regions.append(
                Region(match.group(1), int(match.group(2), 0), int(match.group(3), 0))
            )
    return regions


def _region_core(filename: str) -> str:
    name = filename.lower()
    if name.startswith(("lcpu_", "lpsys_")):
        return "lcpu"
    return "hcpu"


def filter_jlink_script_core(text: str, core: Optional[str]) -> str:
    if core is None:
        return text
    selected = core.lower()
    lines = []
    for line in text.splitlines(keepends=True):
        if "Switch to " in line:
            continue
        match = _SAVEBIN_RE.match(line)
        if match and _region_core(match.group(1)) != selected:
            continue
        lines.append(line)
    return "".join(lines)


def render_jlink_usb_script(text: str, jlink_ip: Optional[str] = None) -> str:
    lines = []
    for line in text.splitlines(keepends=True):
        if re.match(r"^\s*ip\s+\S+", line, re.IGNORECASE):
            newline = "\n" if line.endswith("\n") else ""
            lines.append((f"ip {jlink_ip}" if jlink_ip else "usb") + newline)
        else:
            lines.append(line)
    return "".join(lines)


def normalize_chip(chip: str) -> str:
    match = re.search(r"(52|55|56|57|58)", chip.upper())
    if not match:
        raise UsageError(f"Unsupported chip: {chip}")
    return f"SF32LB{match.group(1)}"


def build_sftool_read_command(
    executable: str,
    chip: str,
    memory: str,
    port: str,
    baud: int,
    address: int,
    size: int,
    output: str,
) -> List[str]:
    return [
        executable,
        "-p",
        port,
        "-b",
        str(baud),
        "-c",
        normalize_chip(chip),
        "-m",
        memory,
        "--after",
        "no_reset",
        "read_flash",
        f"{output}@0x{address:x}:0x{size:x}",
    ]


def find_legacy_ptab_region(ptab: Any, tag: str) -> Tuple[int, int]:
    if not isinstance(ptab, list):
        raise ValueError("legacy PTAB must be a JSON list")
    for memory in ptab:
        base = (
            int(memory.get("base", 0), 0)
            if isinstance(memory.get("base", 0), str)
            else int(memory.get("base", 0))
        )
        for region in memory.get("regions", []):
            if tag in region.get("tags", []):
                if "offset" not in region:
                    raise ValueError(f"PTAB region tagged {tag} has no offset")
                offset = (
                    int(region["offset"], 0)
                    if isinstance(region["offset"], str)
                    else int(region["offset"])
                )
                size_value = region.get("max_size", region.get("size"))
                size = (
                    int(size_value, 0)
                    if isinstance(size_value, str)
                    else int(size_value)
                )
                return base + offset, size
    raise ValueError(f"PTAB tag not found: {tag}")


_JLINK_SCRIPTS = {
    "SF32LB52": "sf32lb52x_uart.jlink",
    "SF32LB55": "sf32lb55x.jlink",
    "SF32LB56": "sf32lb56x.jlink",
    "SF32LB57": "sf32lb57x.jlink",
    "SF32LB58": "sf32lb58x.jlink",
}
_ASSERTDUMP_PREFIXES = {
    "SF32LB52": "sf32lb52x",
    "SF32LB55": "sf32lb55x",
    "SF32LB56": "sf32lb56x",
    "SF32LB57": "sf32lb57x",
    "SF32LB58": "sf32lb58x",
}

_UART_CORE_SWITCH_ADDRESSES = {
    "SF32LB52": 0x5000B008,
    "SF32LB56": 0x5000F000,
}


def _normalize_psram_size(psram_size: Optional[str]) -> Optional[str]:
    if psram_size is None:
        return None
    value = str(psram_size).strip().upper()
    if value.endswith("MB"):
        value = value[:-2]
    if not value.isdigit():
        raise UsageError(f"Invalid PSRAM size: {psram_size}")
    return f"{int(value)}MB"


def _assertdump_script_for_model(
    sdk_root: Path,
    chip_model: str,
    chip: str,
    include_psram: bool = False,
) -> Path:
    config_path = sdk_root / "tools" / "AssertDump" / "AssertDump.ini"
    parser = configparser.ConfigParser()
    parser.read(config_path, encoding="utf-8")
    model = chip_model.strip().upper()
    for section in parser.sections():
        if parser.has_option(section, model):
            script_name = (
                parser.get(section, model)
                if include_psram
                else f"{_ASSERTDUMP_PREFIXES[normalize_chip(chip)]}_psramless.jlink"
            )
            script = sdk_root / "tools" / "AssertDump" / script_name
            if not script.is_file():
                raise UsageError(f"AssertDump script not found: {script}")
            return script
    raise UsageError(f"AssertDump chip model not found: {chip_model}")


def default_jlink_script(
    sdk_root: Path,
    chip: str,
    legacy_layout: bool = False,
    psram_size: Optional[str] = None,
    chip_model: Optional[str] = None,
    include_psram: bool = False,
) -> Path:
    sdk_root = Path(sdk_root)
    normalized_chip = normalize_chip(chip)
    normalized_psram = _normalize_psram_size(psram_size)
    if chip_model:
        return _assertdump_script_for_model(sdk_root, chip_model, chip, include_psram)
    if legacy_layout or normalized_psram:
        prefix = _ASSERTDUMP_PREFIXES.get(normalized_chip)
        if not prefix:
            raise UsageError(f"AssertDump legacy script is unavailable for {normalized_chip}")
        suffix = f"psram_{normalized_psram}" if normalized_psram else "psramless"
        script = sdk_root / "tools" / "AssertDump" / f"{prefix}_{suffix}.jlink"
        if not script.is_file():
            raise UsageError(f"AssertDump script not found: {script}")
        return script
    return sdk_root / "tools" / "crash_dump_analyser" / "script" / _JLINK_SCRIPTS[normalized_chip]


def _write_live_manifest(
    output_dir: Path,
    chip: str,
    transport: str,
    regions: Sequence[Region],
    elf_path: Optional[Path] = None,
    log_path: Optional[Path] = None,
    debug_log_path: Optional[Path] = None,
    registers: Optional[Dict[str, int]] = None,
    register_source: Optional[str] = None,
    extra_warnings: Optional[Sequence[str]] = None,
) -> Dict[str, Any]:
    segments = []
    warnings = list(extra_warnings or [])
    for region in regions:
        path = output_dir / region.filename
        if not path.exists():
            warnings.append(f"missing output file: {region.filename}")
            continue
        data = path.read_bytes()
        segments.append(
            {
                "address": f"0x{region.address:08x}",
                "size": len(data),
                "expected_size": region.size,
                "file": region.filename,
                "sha256": _sha256(data),
            }
        )
    manifest = {
        "schema_version": 1,
        "source": f"live-{transport}",
        "chip": normalize_chip(chip),
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "complete": not warnings
        and all(item["size"] == item["expected_size"] for item in segments),
        "warnings": warnings,
        "segments": segments,
    }
    if elf_path:
        manifest["elf"] = _attach_file(output_dir, elf_path)
    if log_path:
        manifest["log"] = _attach_file(output_dir, log_path)
    if debug_log_path:
        manifest["debug_log"] = _attach_file(output_dir, debug_log_path)
    if registers and registers.get("pc"):
        formatted = {
            name: f"0x{value:08x}" for name, value in registers.items()
        }
        manifest["current_register_source"] = register_source or "capture"
        manifest["current_registers"] = formatted
        manifest["register_source"] = manifest["current_register_source"]
        manifest["registers"] = formatted
    (output_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return manifest


def capture_live(
    sdk_ctx: Any,
    transport: str,
    chip: str,
    output_dir: Path,
    probe: Optional[str] = None,
    probe_rs: str = "probe-rs",
    jlink: str = "JLinkExe",
    script_path: Optional[Path] = None,
    elf_path: Optional[Path] = None,
    log_path: Optional[Path] = None,
    legacy_layout: bool = True,
    psram_size: Optional[str] = None,
    chip_model: Optional[str] = None,
    include_psram: bool = False,
    jlink_ip: Optional[str] = None,
    core: Optional[str] = None,
) -> Dict[str, Any]:
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    sdk_root = _sdk_root(sdk_ctx)
    source_script = (
        Path(script_path)
        if script_path
        else default_jlink_script(
            sdk_root,
            chip,
            legacy_layout=legacy_layout,
            psram_size=psram_size,
            chip_model=chip_model,
            include_psram=include_psram,
        )
    )
    text = filter_jlink_script_core(source_script.read_text(encoding="utf-8"), core)
    regions = parse_jlink_regions(text)
    if transport == "uart" and script_path is None and not legacy_layout and not psram_size:
        if core and core.lower() == "lcpu":
            raise UsageError("UART live capture does not support --core lcpu")
        skipped_regions = [
            region
            for region in regions
            if region.filename.lower().startswith(("lcpu_", "lpsys_"))
        ]
        if skipped_regions:
            print(
                "warning: UART live capture is skipping LCPU/LPSYS regions",
                file=sys.stderr,
            )
        regions = [
            region
            for region in regions
            if not region.filename.lower().startswith(("lcpu_", "lpsys_"))
        ]
    if not regions:
        raise FatalError(f"No savebin regions found in {source_script}")

    debug_log_path = None
    live_registers = None
    register_source = None
    live_warnings = []
    if transport == "uart":
        normalized_chip = normalize_chip(chip)
        if normalized_chip not in _UART_CORE_SWITCH_ADDRESSES:
            raise UsageError("UART live capture only supports SF32LB52X and SF32LB56X")
        if not probe:
            raise UsageError("--probe is required for --transport uart")
        from sdk_py_actions.sifli_uart_dap import (
            SifliUartDap,
            extract_probe_port,
            normalize_chip_for_dap,
        )
        port = extract_probe_port(probe)
        dap_chip = normalize_chip_for_dap(chip)
        with SifliUartDap(port, chip=dap_chip) as dap:
            # Core-switch initialization – required for the UART DEBUG IP
            # to access all bus domains (SRAM, HPSYS peripherals, MAC).
            core_switch = _UART_CORE_SWITCH_ADDRESSES[normalized_chip]
            dap.write32(core_switch, 1)   # switch to LCPU
            dap.write32(core_switch, 0)   # switch back to HCPU
            dap.halt()                    # halt HCPU via DHCSR – prevents
            # bus contention when the CPU is in a fault loop (e.g. after
            # heap overflow where the fault handler itself may be unstable).
            try:
                live_registers = dap.read_core_registers()
                register_source = "uart_dap"
            except Exception as exc:
                live_warnings.append(f"UART-DAP core register read failed: {exc}")
            for region in regions:
                _show_progress(region.filename, 0, region.size)
                data = dap.read(region.address, region.size)
                (output_dir / region.filename).write_bytes(data)
                _show_progress(region.filename, region.size, region.size)
    elif transport == "jlink":
        generated = output_dir / "capture.jlink"
        generated.write_text(
            render_jlink_usb_script(text, jlink_ip=jlink_ip),
            encoding="utf-8",
        )
        command = [jlink, "-Device", "CORTEX-M33"]
        if jlink_ip:
            command.extend(["-ip", jlink_ip])
        command.extend(["-CommanderScript", str(generated)])
        result = sdk_ctx.runner.run(
            command,
            cwd=str(output_dir),
            capture_output=True,
        )
        log = (result.stdout or "") + (result.stderr or "")
        debug_log_path = output_dir / "capture.log"
        debug_log_path.write_text(log, encoding="utf-8")
        live_registers = parse_jlink_registers(log)
        register_source = "debug_log"
        for region in regions:
            path = output_dir / region.filename
            _show_progress(
                region.filename,
                path.stat().st_size if path.exists() else 0,
                region.size,
            )
    else:
        raise UsageError(f"Unsupported transport: {transport}")

    manifest = _write_live_manifest(
        output_dir,
        chip,
        transport,
        regions,
        elf_path=elf_path,
        log_path=log_path,
        debug_log_path=debug_log_path,
        registers=live_registers,
        register_source=register_source,
        extra_warnings=live_warnings,
    )
    if legacy_layout:
        write_legacy_layout(
            output_dir,
            manifest,
            regions,
            elf_path=elf_path,
            debug_log_path=debug_log_path,
        )
    return manifest


def export_coredump(
    sdk_ctx: Any,
    output_dir: Path,
    input_path: Optional[Path] = None,
    transport: str = "uart",
    chip: Optional[str] = None,
    memory: str = "nor",
    port: Optional[str] = None,
    baud: int = 1_000_000,
    address: Optional[int] = None,
    size: Optional[int] = None,
    ptab_path: Optional[Path] = None,
    kind: str = "full",
    sftool: str = "sftool",
    jlink: str = "JLinkExe",
    elf_path: Optional[Path] = None,
    log_path: Optional[Path] = None,
    script_path: Optional[Path] = None,
    legacy_layout: bool = True,
    psram_size: Optional[str] = None,
    chip_model: Optional[str] = None,
    include_psram: bool = False,
) -> Dict[str, Any]:
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    raw_path = output_dir / "coredump.bin"

    if input_path:
        source = Path(input_path).read_bytes()
        raw_path.write_bytes(source)
        manifest = write_package(
            source,
            output_dir,
            source="file",
            elf_path=elf_path,
            log_path=log_path,
        )
        if legacy_layout:
            sdk_root = _sdk_root(sdk_ctx)
            source_script = (
                Path(script_path)
                if script_path
                else default_jlink_script(
                    sdk_root,
                    manifest["chip"],
                    legacy_layout=legacy_layout,
                    psram_size=psram_size,
                    chip_model=chip_model,
                    include_psram=include_psram,
                )
            )
            write_legacy_layout(
                output_dir,
                manifest,
                parse_jlink_regions(source_script.read_text(encoding="utf-8")),
                elf_path=elf_path,
                debug_log_path=log_path,
            )
        return manifest

    if ptab_path:
        ptab = json.loads(Path(ptab_path).read_text(encoding="utf-8"))
        address, size = find_legacy_ptab_region(
            ptab, "COREDUMP_MIN_REGION" if kind == "minimum" else "COREDUMP_REGION"
        )
    if chip is None or address is None or size is None:
        raise UsageError(
            "--chip and either --ptab or both --address/--size are required"
        )

    if transport == "uart":
        if not port:
            raise UsageError("--port is required for UART coredump export")
        _show_progress(raw_path.name, 0, size)
        sdk_ctx.runner.run(
            build_sftool_read_command(
                sftool, chip, memory, port, baud, address, size, str(raw_path)
            ),
            cwd=str(output_dir),
        )
        _show_progress(
            raw_path.name,
            raw_path.stat().st_size if raw_path.exists() else 0,
            size,
        )
    elif transport == "jlink":
        script = output_dir / "read-coredump.jlink"
        script.write_text(
            f"si SWD\nusb\nH\nsavebin {raw_path.name} 0x{address:x} 0x{size:x}\nexit\n",
            encoding="utf-8",
        )
        _show_progress(raw_path.name, 0, size)
        sdk_ctx.runner.run(
            [jlink, "-Device", "CORTEX-M33", "-CommanderScript", str(script)],
            cwd=str(output_dir),
        )
        _show_progress(raw_path.name, raw_path.stat().st_size if raw_path.exists() else 0, size)
    else:
        raise UsageError(f"Unsupported transport: {transport}")

    if not raw_path.exists():
        raise FatalError(f"Coredump backend did not create {raw_path}")
    manifest = write_package(
        raw_path.read_bytes(),
        output_dir,
        source=f"partition-{transport}",
        elf_path=elf_path,
        log_path=log_path,
    )
    if legacy_layout:
        sdk_root = _sdk_root(sdk_ctx)
        source_script = (
            Path(script_path)
            if script_path
            else default_jlink_script(
                sdk_root,
                manifest["chip"],
                legacy_layout=legacy_layout,
                psram_size=psram_size,
                chip_model=chip_model,
                include_psram=include_psram,
            )
        )
        write_legacy_layout(
            output_dir,
            manifest,
            parse_jlink_regions(source_script.read_text(encoding="utf-8")),
            elf_path=elf_path,
            debug_log_path=log_path,
        )
    return manifest
