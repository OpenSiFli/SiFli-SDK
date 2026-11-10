# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Turn a name in the built firmware into an address a probe can read.

A suite that wants to look at what the firmware rendered needs the address the
*current* build put it at; a number copied into a test is right for exactly one
build and silently wrong for the next. So the address is read from the ELF the
build just produced: the symbol table for a value, and DWARF for a member of a
struct (``drv_lcd.debug_cnt1``), whose offset is a property of the compiled
layout rather than something a test can restate.

``pyelftools`` is imported lazily: the framework's own tests are stdlib-only,
and a suite that never asks for an address never pays for the dependency.
"""

from __future__ import annotations

import glob
import os
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple

from .errors import SymbolError

_ElfKey = Tuple[str, float, int]

#: Parsed symbol tables and struct members, keyed by
#: ``(path, mtime, size)`` so a rebuild invalidates them.
_SYMBOL_CACHE: Dict[_ElfKey, Dict[str, int]] = {}
_AMBIGUOUS_CACHE: Dict[_ElfKey, set] = {}
_ADDRESS_CACHE: Dict[Tuple[_ElfKey, str], int] = {}


def find_elf(build_dir: str) -> str:
    """The ELF a build directory produced.

    ``main`` is the SDK's target name, so ``main.elf`` is the answer almost
    always; another name is accepted when it is the only ELF there. Several
    candidates is an error rather than a guess -- reading the wrong image's
    symbols produces addresses that look plausible and are not.
    """
    if not build_dir:
        raise SymbolError("a build directory is needed to resolve a symbol")
    preferred = os.path.join(build_dir, "main.elf")
    if os.path.isfile(preferred):
        return preferred

    candidates = sorted(glob.glob(os.path.join(build_dir, "*.elf")))
    if not candidates:
        raise SymbolError(
            f'no ELF file in build directory "{build_dir}"; '
            "was the project built before the suite ran?"
        )
    if len(candidates) > 1:
        names = ", ".join(os.path.basename(path) for path in candidates)
        raise SymbolError(
            f'several ELF files in build directory "{build_dir}" ({names}); '
            "cannot tell which one the board is running"
        )
    return candidates[0]


def address_of(elf_path: str, name: str) -> int:
    """Address of ``name``, or of a member written ``variable.member``.

    Nested members (``a.b.c``) are followed to any depth, which costs nothing
    and saves a second spelling of the same question.
    """
    name = (name or "").strip()
    if not name:
        raise SymbolError("symbol name must not be empty")

    # Walking DWARF is not cheap, and a suite asks for the same handful of
    # names on every poll; cache the answer for the life of this build.
    cache_key = (_elf_key(elf_path), name)
    cached = _ADDRESS_CACHE.get(cache_key)
    if cached is not None:
        return cached

    variable, _, members = name.partition(".")
    base = resolve_symbol(elf_path, variable)
    address = base if not members else base + _member_path_offset(
        elf_path, variable, members.split(".")
    )
    _ADDRESS_CACHE[cache_key] = address
    return address


def resolve_symbol(elf_path: str, name: str) -> int:
    """Address of one symbol, from the ELF's symbol table."""
    address = _symbols(elf_path).get(name)
    if address is not None:
        return address
    if name in _ambiguous(elf_path):
        raise SymbolError(
            f'symbol "{name}" is defined more than once with different '
            f'addresses in {elf_path}; the name alone cannot choose one'
        )
    raise SymbolError(
        f'symbol "{name}" is not in {elf_path}; '
        "is the suite looking at the image the board is running?"
    )


def member_offset(elf_path: str, variable: str, member: str) -> int:
    """Byte offset of ``member`` inside the struct type of ``variable``."""
    return _member_path_offset(elf_path, variable, [member])


# -- implementation ----------------------------------------------------------


def _elf_key(elf_path: str) -> _ElfKey:
    try:
        stat = os.stat(elf_path)
    except OSError as exc:
        raise SymbolError(f'cannot read ELF "{elf_path}": {exc}') from exc
    return os.path.abspath(elf_path), stat.st_mtime, stat.st_size


def _open_elf(elf_path: str):
    """``(ELFFile, handle)``; the caller closes the handle."""
    try:
        from elftools.elf.elffile import ELFFile
    except ImportError as exc:  # pragma: no cover - environment failure
        raise SymbolError(
            "pyelftools is required to resolve symbols and struct members; "
            "it is part of the SDK's default profile (run inside the "
            "environment export.ps1/export.sh set up)"
        ) from exc
    try:
        handle = open(elf_path, "rb")
    except OSError as exc:
        raise SymbolError(f'cannot read ELF "{elf_path}": {exc}') from exc
    try:
        return ELFFile(handle), handle
    except Exception as exc:  # noqa: BLE001 - a malformed ELF is the caller's
        handle.close()
        raise SymbolError(f'"{elf_path}" is not a readable ELF: {exc}') from exc


def _symbols(elf_path: str) -> Dict[str, int]:
    key = _elf_key(elf_path)
    cached = _SYMBOL_CACHE.get(key)
    if cached is not None:
        return cached

    elf, handle = _open_elf(elf_path)
    try:
        table = elf.get_section_by_name(".symtab")
        if table is None:
            raise SymbolError(
                f'"{elf_path}" has no symbol table; it was stripped, so no '
                "address can be resolved from it"
            )
        symbols: Dict[str, int] = {}
        ambiguous: set = set()
        for symbol in table.iter_symbols():
            name = symbol.name
            if not name or symbol["st_shndx"] == "SHN_UNDEF":
                continue
            value = int(symbol["st_value"])
            previous = symbols.get(name)
            if previous is not None and previous != value:
                # Two static objects of the same name in different files: the
                # name alone cannot answer which one was meant.
                ambiguous.add(name)
                continue
            symbols[name] = value
        for name in ambiguous:
            symbols.pop(name, None)
    finally:
        handle.close()

    _SYMBOL_CACHE[key] = symbols
    _AMBIGUOUS_CACHE[key] = ambiguous
    return symbols


def _ambiguous(elf_path: str) -> set:
    key = _elf_key(elf_path)
    if key not in _AMBIGUOUS_CACHE:
        _symbols(elf_path)
    return _AMBIGUOUS_CACHE.get(key, set())


def _member_path_offset(elf_path: str, variable: str, path: List[str]) -> int:
    """Offset of a nested member path, resolved segment by segment."""
    address = resolve_symbol(elf_path, variable)
    elf, handle = _open_elf(elf_path)
    try:
        dwarf = elf.get_dwarf_info()
        if not dwarf.has_debug_info:
            raise SymbolError(
                f'"{elf_path}" has no DWARF debug information; struct members '
                "cannot be resolved from it (was the build made without -g?)"
            )
        wanted = variable.encode("utf-8")
        candidates = []
        for cu in dwarf.iter_CUs():
            for die in cu.get_top_DIE().iter_children():
                if die.tag != "DW_TAG_variable":
                    continue
                attribute = die.attributes.get("DW_AT_name")
                if attribute is None or attribute.value != wanted:
                    continue
                candidates.append((_variable_address(die), die))

        if not candidates:
            raise SymbolError(f'no debug information for "{variable}" in {elf_path}')

        # Prefer the DIE that names the address the symbol table resolved. Two
        # translation units can both declare a static of the same name; the
        # linker may have dropped one while its DWARF record survives, and the
        # layouts need not agree.
        for location, die in candidates:
            if location == address:
                offset = _walk_members(die, path, variable)
                if offset is not None:
                    return offset
        unnamed = [die for location, die in candidates if location is None]
        if len(candidates) == 1 and len(unnamed) == 1:
            # Some producers omit DW_AT_location for a file-scope variable;
            # with only one candidate there is nothing to confuse it with.
            offset = _walk_members(unnamed[0], path, variable)
            if offset is not None:
                return offset

        raise SymbolError(
            f'cannot match the debug information for "{variable}" to the symbol '
            f'at 0x{address:08x} in {elf_path}'
        )
    finally:
        handle.close()


def _variable_address(die) -> Optional[int]:
    """The absolute address in a static variable's ``DW_AT_location``, if simple."""
    attribute = die.attributes.get("DW_AT_location")
    if attribute is None:
        return None
    value = attribute.value
    if isinstance(value, int) or not isinstance(
        value, (bytes, bytearray, list, tuple)
    ):
        return None
    data = list(value)
    # DW_OP_addr (0x03) followed by the target's address; this is a 32-bit
    # target, so the address is four bytes.
    if len(data) >= 5 and data[0] == 0x03:
        return int.from_bytes(bytes(data[1:5]), "little")
    return None


def _walk_members(variable_die, path: List[str], variable: str) -> Optional[int]:
    """Sum the offsets along ``path``, or ``None`` if it does not exist."""
    try:
        type_die = _dereference(
            variable_die.get_DIE_from_attribute("DW_AT_type")
        )
    except KeyError:
        return None

    total = 0
    for index, segment in enumerate(path):
        if type_die is None or type_die.tag != "DW_TAG_structure_type":
            return None
        member_die = _find_member(type_die, segment)
        if member_die is None:
            return None
        if _is_bitfield(member_die):
            full = ".".join(path[: index + 1])
            raise SymbolError(
                f'"{variable}.{full}" is a bitfield; a plain address cannot '
                "read it"
            )
        offset = _member_offset(member_die)
        if offset is None:
            return None
        total += offset
        try:
            type_die = _dereference(
                member_die.get_DIE_from_attribute("DW_AT_type")
            )
        except KeyError:
            type_die = None
    return total


def _find_member(type_die, name: str):
    wanted = name.encode("utf-8")
    for child in type_die.iter_children():
        if child.tag != "DW_TAG_member":
            continue
        attribute = child.attributes.get("DW_AT_name")
        if attribute is not None and attribute.value == wanted:
            return child
    return None


def _member_offset(member_die) -> Optional[int]:
    """Byte offset of a plain member; ``None`` for a bitfield or unknown form."""
    if _is_bitfield(member_die):
        return None
    location = member_die.attributes.get("DW_AT_data_member_location")
    if location is None:
        return None
    return _constant_member_offset(location.value)


def _is_bitfield(member_die) -> bool:
    return any(
        attribute in member_die.attributes
        for attribute in (
            "DW_AT_bit_size",
            "DW_AT_bit_offset",
            "DW_AT_data_bit_offset",
        )
    )


def _constant_member_offset(value) -> Optional[int]:
    """A constant offset from an integer or a DWARF location expression.

    DWARF 2 may encode ``DW_AT_data_member_location`` as ``DW_OP_plus_uconst``
    (some producers use ``DW_OP_constu``) instead of a plain integer.
    """
    if isinstance(value, int) and not isinstance(value, bool):
        return value
    if not isinstance(value, (bytes, bytearray, list, tuple)):
        return None
    data = list(value)
    if not data or data[0] not in (0x10, 0x23):  # DW_OP_constu, DW_OP_plus_uconst
        return None
    result, shift = 0, 0
    for index in range(1, len(data)):
        byte = data[index]
        result |= (byte & 0x7F) << shift
        if not byte & 0x80:
            # The expression must end here; a trailing operation is not
            # something this resolver understands.
            return result if index == len(data) - 1 else None
        shift += 7
    return None


def _dereference(type_die):
    """Follow typedefs/qualifiers to the type that carries the members."""
    seen = 0
    while type_die is not None and type_die.tag in (
        "DW_TAG_typedef",
        "DW_TAG_const_type",
        "DW_TAG_volatile_type",
        "DW_TAG_restrict_type",
    ):
        try:
            type_die = type_die.get_DIE_from_attribute("DW_AT_type")
        except KeyError:
            return None
        seen += 1
        if seen > 16:  # a cycle in malformed debug information
            return None
    return type_die
