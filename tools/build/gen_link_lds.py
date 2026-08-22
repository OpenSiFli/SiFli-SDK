#!/usr/bin/env python3
# Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

import ast
import os
import re
from typing import Any, Dict, List, Optional, Sequence, Tuple

from jinja2 import Environment, FileSystemLoader, StrictUndefined

import ptab as ptab_module


class LinkLdsError(RuntimeError):
    pass


_LEGACY_BOOTLOADER_RAM_DEFAULTS: Dict[str, Tuple[int, int]] = {
    'sf32lb56': (0x20020000, 0x00010000),
    'sf32lb58': (0x20000000, 0x00010000),
}


def hex32(value: int) -> str:
    return '0x{:08X}'.format(value & 0xFFFFFFFF)


def _safe_eval_int(expr: str) -> int:
    """Evaluate a C-like integer expression safely.

    Supported:
    - hex/dec literals
    - +, -, *, /, //, %, <<, >>, |, &, ^
    - unary +/-
    - parentheses

    Note: Identifiers are not supported in this helper.
    """
    expr = (expr or '').strip()
    if not expr:
        raise LinkLdsError('Empty expression')

    # Strip outer parentheses to reduce noise like "((0x12000000))"
    while expr.startswith('(') and expr.endswith(')'):
        inner = expr[1:-1].strip()
        if not inner:
            break
        expr = inner

    tree = ast.parse(expr, mode='eval')

    def _eval(node: ast.AST) -> int:
        if isinstance(node, ast.Expression):
            return _eval(node.body)
        if isinstance(node, ast.Constant):
            if isinstance(node.value, bool):
                return int(node.value)
            if isinstance(node.value, int):
                return node.value
            raise LinkLdsError('Unsupported constant: {}'.format(type(node.value)))
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
            v = _eval(node.operand)
            return +v if isinstance(node.op, ast.UAdd) else -v
        if isinstance(node, ast.BinOp):
            left = _eval(node.left)
            right = _eval(node.right)
            op = node.op
            if isinstance(op, ast.Add):
                return left + right
            if isinstance(op, ast.Sub):
                return left - right
            if isinstance(op, ast.Mult):
                return left * right
            if isinstance(op, ast.Div):
                return int(left / right)
            if isinstance(op, ast.FloorDiv):
                return left // right
            if isinstance(op, ast.Mod):
                return left % right
            if isinstance(op, ast.LShift):
                return left << right
            if isinstance(op, ast.RShift):
                return left >> right
            if isinstance(op, ast.BitOr):
                return left | right
            if isinstance(op, ast.BitAnd):
                return left & right
            if isinstance(op, ast.BitXor):
                return left ^ right
            raise LinkLdsError('Unsupported binary operator: {}'.format(type(op)))
        raise LinkLdsError('Unsupported expression node: {}'.format(type(node)))

    return int(_eval(tree))


def _parse_define_map(header_path: str) -> Dict[str, str]:
    """Parse simple `#define NAME VALUE` lines into a raw string map.

    Only the last definition wins.
    """
    define_re = re.compile(r'^\s*#\s*define\s+([A-Za-z_]\w*)\s*(.*)$')
    out: Dict[str, str] = {}
    try:
        with open(header_path, 'r', encoding='utf-8', errors='ignore') as f:
            logical_lines: List[str] = []
            pending = ''
            for raw in f:
                line = raw.rstrip('\n')
                if pending:
                    line = pending + line.lstrip()
                if line.rstrip().endswith('\\'):
                    pending = line.rstrip()[:-1] + ' '
                    continue
                pending = ''
                logical_lines.append(line)
            if pending:
                logical_lines.append(pending)

            for line in logical_lines:
                m = define_re.match(line)
                if not m:
                    continue
                name = m.group(1)
                value = (m.group(2) or '').strip()
                # Drop trailing comments
                value = value.split('//', 1)[0].strip()
                value = re.sub(r'/\*.*?\*/', '', value).strip()
                out[name] = value
    except FileNotFoundError:
        return out
    return out


def _parse_rtconfig_defines(rtconfig_h_path: str) -> Dict[str, Any]:
    """Parse rtconfig.h into a dict usable by Jinja2.

    - `#define FOO 1` -> 1
    - `#define BAR` -> 1
    - non-numeric values kept as strings
    """
    raw = _parse_define_map(rtconfig_h_path)
    out: Dict[str, Any] = {}
    for k, v in raw.items():
        if v == '':
            out[k] = 1
            continue
        # Typical Kconfig output uses 1/0 or strings.
        try:
            out[k] = int(v, 0)
            continue
        except Exception:
            pass
        out[k] = v
    return out


def _infer_cmsis_dir_from_chip(chip: str) -> str:
    chip = (chip or '').upper()
    if chip.startswith('SF32LB52'):
        return 'sf32lb52x'
    if chip.startswith('SF32LB56'):
        return 'sf32lb56x'
    if chip.startswith('SF32LB58'):
        return 'sf32lb58x'
    raise LinkLdsError('Unsupported chip for Jinja2 link generation: {}'.format(chip))


def _load_mem_map_ints(sifli_sdk_root: str, cmsis_dir: str) -> Dict[str, int]:
    mem_map_path = os.path.join(sifli_sdk_root, 'drivers', 'cmsis', cmsis_dir, 'mem_map.h')
    wanted = {
        # HCPU RAM layout / bank split
        'HPSYS_RAM_SIZE',
        'HPSYS_RAM0_BASE',
        'HPSYS_RAM0_SIZE',
        'HPSYS_RAM1_BASE',
        'HPSYS_RAM1_SIZE',
        'HCPU_RAM_DATA_START_ADDR',
        'HCPU_RAM_DATA_SIZE',
        'HCPU_RO_DATA_START_ADDR',
        'HCPU_RO_DATA_SIZE',
        # Flash2 layout (used by HCPU link script)
        'QSPI2_MEM_BASE',
        'HCPU_FLASH2_IMG_START_ADDR',
        'HCPU_FLASH2_IMG_SIZE',
        'HCPU_FLASH2_FONT_START_ADDR',
        'HCPU_FLASH2_FONT_SIZE',
        # Bootloader runtime size
        'FLASH_BOOT_LOADER_SIZE',
        # HCPU RO data (ROM_EX) size (placed at end of HPSYS RAM)
        # 58x HCPU ret/itcm
        'HPSYS_RETM_BASE',
        'HPSYS_RETM_SIZE',
        'HPSYS_ITCM_BASE',
        'HPSYS_ITCM_SIZE',
        # Mailbox sizes
        'HPSYS_MBOX_BUF_SIZE',
        'LPSYS_MBOX_BUF_SIZE',
        # LCPU RAM view
        'LPSYS_RAM_BASE',
        'LPSYS_RAM_CBUS_BASE',
        'LPSYS_RAM_SIZE',
        'LPSYS_EM_BASE',
        'LPSYS_EM_SIZE',
        'LCPU_MBOX_SIZE',
        'LCPU_RAM_CODE_SIZE',
        'LCPU_RAM_CODE_START_ADDR',
        'LCPU_FLASH_CODE_START_ADDR',
        # LCPU DTCM
        'LPSYS_DTCM_BASE',
        'LPSYS_DTCM_SIZE',
        # LCPU flash code (used by 56x LCPU linker scripts)
        'LCPU_FLASH_CODE_SIZE',
        # Optional PSRAM layout helpers used by legacy example linker scripts
        'PSRAM_DATA_START_ADDR',
        'PSRAM_DATA_SIZE',
        'PSRAM_DATA2_START_ADDR',
        'PSRAM_DATA2_SIZE',
        'PSRAM_CODE_START_ADDR',
        'PSRAM_CODE_SIZE',
    }

    raw = _parse_define_map(mem_map_path)

    cache: Dict[str, int] = {}
    visiting: set = set()

    def _safe_eval_int_with_names(expr: str) -> int:
        expr = (expr or '').strip()
        if not expr:
            raise LinkLdsError('Empty expression')

        # Strip outer parentheses to reduce noise like "((0x12000000))"
        while expr.startswith('(') and expr.endswith(')'):
            inner = expr[1:-1].strip()
            if not inner:
                break
            expr = inner

        tree = ast.parse(expr, mode='eval')

        def _eval(node: ast.AST) -> int:
            if isinstance(node, ast.Expression):
                return _eval(node.body)
            if isinstance(node, ast.Constant):
                if isinstance(node.value, bool):
                    return int(node.value)
                if isinstance(node.value, int):
                    return node.value
                raise LinkLdsError('Unsupported constant: {}'.format(type(node.value)))
            if isinstance(node, ast.Name):
                return _eval_define(node.id)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
                fname = node.func.id
                if fname == 'END_ADDR' and len(node.args) == 2:
                    start = _eval(node.args[0])
                    size = _eval(node.args[1])
                    return start + size - 1
                raise LinkLdsError('Unsupported macro call: {}'.format(fname))
            if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
                v = _eval(node.operand)
                return +v if isinstance(node.op, ast.UAdd) else -v
            if isinstance(node, ast.BinOp):
                left = _eval(node.left)
                right = _eval(node.right)
                op = node.op
                if isinstance(op, ast.Add):
                    return left + right
                if isinstance(op, ast.Sub):
                    return left - right
                if isinstance(op, ast.Mult):
                    return left * right
                if isinstance(op, ast.Div):
                    return int(left / right)
                if isinstance(op, ast.FloorDiv):
                    return left // right
                if isinstance(op, ast.Mod):
                    return left % right
                if isinstance(op, ast.LShift):
                    return left << right
                if isinstance(op, ast.RShift):
                    return left >> right
                if isinstance(op, ast.BitOr):
                    return left | right
                if isinstance(op, ast.BitAnd):
                    return left & right
                if isinstance(op, ast.BitXor):
                    return left ^ right
                raise LinkLdsError('Unsupported binary operator: {}'.format(type(op)))
            raise LinkLdsError('Unsupported expression node: {}'.format(type(node)))

        return int(_eval(tree))

    def _eval_define(name: str) -> int:
        name = (name or '').strip()
        if not name:
            raise LinkLdsError('Empty identifier')
        if name in cache:
            return cache[name]
        if name in visiting:
            raise LinkLdsError('Circular define reference: {}'.format(name))
        if name not in raw:
            raise LinkLdsError('Unknown identifier in mem_map.h: {}'.format(name))

        visiting.add(name)
        try:
            v = _safe_eval_int_with_names(raw[name])
        finally:
            visiting.remove(name)
        cache[name] = int(v)
        return int(v)

    out: Dict[str, int] = {}
    for name in wanted:
        if name not in raw:
            continue
        try:
            out[name] = _eval_define(name)
        except Exception as e:
            raise LinkLdsError('Failed to eval {} from {}: {} ({})'.format(name, mem_map_path, raw[name], e))
    return out


def _mpi_name_from_region(region: str) -> Optional[str]:
    region = (region or '').strip().lower()
    if not region:
        return None
    if region.startswith('mpi'):
        return region
    psram_match = re.match(r'^psram(\d+)?$', region)
    if psram_match:
        mpi_num = psram_match.group(1) or '1'
        return 'mpi{}'.format(mpi_num)
    return None


def _get_region_mem_type(region: str, chip_config: Dict[str, Any]) -> str:
    region = (region or '').strip().lower()
    if region == 'hpsys_ram' or region.startswith('hpsys') or region == 'lpsys_ram' or region.startswith('lpsys'):
        return 'ram'
    if region == 'psram' or region.startswith('psram'):
        return 'psram'
    mpi_name = _mpi_name_from_region(region)
    if mpi_name:
        info = chip_config.get('memory_info', {}).get(mpi_name, {})
        mtype = (info.get('type') or '').lower()
        return mtype or 'nor'
    return ''


def _select_start_addr(mem_type: str, sbus_addr: int, cbus_addr: int) -> int:
    return cbus_addr if mem_type == 'nor' else sbus_addr


def _select_exec_addr(mem_type: str, sbus_addr: int, cbus_addr: int, core=None) -> int:
    # Execution address selection:
    # - RAM/NAND: base (SBUS)
    # - NOR/PSRAM: XIP （CBUS）
    #
    # NOTE: ACPU deliberately uses the same SBUS-based rule instead of its
    # local CBUS view (e.g. SF32LB58 hpsys_ram is 0-based for ACPU while the
    # HCPU/DFU/SBUS view is 0x20200000). Picking CBUS for ACPU made the exec
    # address unusable for HCPU when ACPU passes that address over.
    return sbus_addr if mem_type in ('ram', 'nand') else cbus_addr


def _get_hpsys_ram_hcpu(chip_config: Dict[str, Any]) -> Tuple[int, int]:
    ram = ((chip_config.get('ram') or {}).get('hpsys') or {}).get('ram') or {}
    base: Optional[int] = None
    total = 0
    for _, bank in ram.items():
        hcpu = bank.get('hcpu') or {}
        off = hcpu.get('offset')
        size = hcpu.get('size')
        if off is None or size is None:
            continue
        if base is None:
            base = int(off)
        total += int(size)
    if base is None:
        base = 0x20000000
    return base, total


def _get_lpsys_ram(chip_config: Dict[str, Any]) -> Tuple[int, int, int]:
    ram = ((chip_config.get('ram') or {}).get('lpsys') or {}).get('ram') or {}
    base_hcpu: Optional[int] = None
    base_lcpu: Optional[int] = None
    total: Optional[int] = None
    for _, bank in ram.items():
        hcpu = bank.get('hcpu') or {}
        lcpu = bank.get('lcpu') or {}
        if base_hcpu is None and hcpu.get('offset') is not None:
            base_hcpu = int(hcpu.get('offset'))
        if base_lcpu is None and lcpu.get('offset') is not None:
            base_lcpu = int(lcpu.get('offset'))
        if total is None and hcpu.get('size') is not None:
            total = int(hcpu.get('size'))
    if base_hcpu is None:
        base_hcpu = 0x20400000
    if base_lcpu is None:
        base_lcpu = 0x00400000
    if total is None:
        total = 0x6000
    return base_hcpu, base_lcpu, total


def _find_partition(partitions, *, ptype: Optional[str] = None, subtype: Optional[str] = None,
                    name: Optional[str] = None, core: Optional[str] = None) -> Optional[Dict[str, Any]]:
    for p in partitions:
        if not isinstance(p, dict):
            continue
        if ptype and p.get('type') != ptype:
            continue
        if subtype and p.get('subtype') != subtype:
            continue
        if name and (p.get('name') or '').lower() != name.lower():
            continue
        if core:
            pcore = (p.get('core') or 'HCPU').upper()
            if pcore != core.upper():
                continue
        return p
    return None


def _match_partition_by_alias(partitions, alias_base: str) -> Optional[Dict[str, Any]]:
    alias_base = (alias_base or '').strip().upper()
    if not alias_base:
        return None
    for p in partitions:
        aliases = p.get('aliases') or []
        for a in aliases:
            if str(a).strip().upper() == alias_base:
                return p
    return None


def _selector_to_gnu(selector: Dict[str, str]) -> str:
    section = str(selector.get('section') or '').strip()
    obj = str(selector.get('object') or '').strip()
    if not section:
        return ''
    if obj:
        return '*{}({})'.format(obj, section)
    return '*({})'.format(section)


def _selector_to_keil(selector: Dict[str, str]) -> str:
    section = str(selector.get('section') or '').strip()
    obj = str(selector.get('object') or '').strip()
    if not section:
        return ''
    if obj:
        if not obj.endswith('.o'):
            obj = '{}.o'.format(obj)
        return '{} ({})'.format(obj, section)
    return '*.o ({})'.format(section)


def _find_named_code_partition(partitions, build_name: str, build_core: str) -> Optional[Dict[str, Any]]:
    build_name_l = (build_name or '').strip().lower()
    build_core_u = (build_core or 'HCPU').strip().upper()
    if not build_name_l:
        return None

    candidates = []
    for p in partitions or []:
        if not isinstance(p, dict):
            continue
        if (p.get('name') or '').strip().lower() != build_name_l:
            continue
        candidates.append(p)

    if not candidates:
        return None

    def _score(p: Dict[str, Any]) -> Tuple[int, int, int, int]:
        pcore = (p.get('core') or 'HCPU').strip().upper()
        ptype = (p.get('type') or '').strip().lower()
        psubtype = (p.get('subtype') or '').strip().lower()
        has_exec = 1 if isinstance(p.get('exec'), dict) else 0
        return (
            1 if pcore == build_core_u else 0,
            1 if ptype == 'app' else 0,
            1 if psubtype == 'factory' else 0,
            has_exec,
        )

    return max(candidates, key=_score)


def compute_link_defines(ptab_obj, build_name: str, build_core: str, rtconfig_defines: Dict[str, Any]) -> Dict[str, Any]:
    if not ptab_obj or not hasattr(ptab_obj, 'is_v3') or not ptab_obj.is_v3():
        raise LinkLdsError('compute_link_defines only supports ptab v3')

    sifli_sdk_root = os.getenv('SIFLI_SDK') or os.getcwd()
    cmsis_dir = _infer_cmsis_dir_from_chip(getattr(ptab_obj, 'chip', ''))

    chip_config = ptab_obj.get_chip_config()
    partitions = ptab_obj.partitions

    mem_map_ints = _load_mem_map_ints(sifli_sdk_root, cmsis_dir)

    # Default RAM windows from SiliconSchema
    hpsys_base, hpsys_total = _get_hpsys_ram_hcpu(chip_config)
    lpsys_base_hcpu, lpsys_base_lcpu, lpsys_total = _get_lpsys_ram(chip_config)

    hpsys_mbox_size = int(mem_map_ints.get('HPSYS_MBOX_BUF_SIZE', 0x400))
    lpsys_mbox_size = int(mem_map_ints.get('LPSYS_MBOX_BUF_SIZE', 0x400))

    # Prefer mem_map.h runtime macros when available; fall back to SiliconSchema.
    hcpu_ram_base = int(mem_map_ints.get('HCPU_RAM_DATA_START_ADDR', hpsys_base))
    hcpu_ro_data_size = int(mem_map_ints.get('HCPU_RO_DATA_SIZE', 0))
    hcpu_ram_size_total = max(0, int(hpsys_total) - hpsys_mbox_size)
    hcpu_ram_size_fallback = max(0, int(hcpu_ram_size_total) - int(hcpu_ro_data_size))
    hcpu_ram_size = int(mem_map_ints.get('HCPU_RAM_DATA_SIZE', 0))
    if hcpu_ram_size <= 0:
        hcpu_ram_size = hcpu_ram_size_fallback

    hcpu_rom_ex_size = hcpu_ro_data_size
    hcpu_rom_ex_base = hcpu_ram_base + hcpu_ram_size

    lcpu_ram_base = lpsys_base_hcpu
    lcpu_ram_size = max(0, int(lpsys_total) - lpsys_mbox_size)
    lcpu_rom_base = lpsys_base_lcpu
    lcpu_rom_size = int(mem_map_ints.get('LCPU_RAM_CODE_SIZE', 0)) * 2

    # PSRAM windows from ptab (optional, usually injected from SiliconSchema)
    psram_part = (
        _match_partition_by_alias(partitions, 'PSRAM_DATA')
        or _find_partition(partitions, name='psram_data')
    )
    lpsys_ram_part = _find_partition(partitions, name='lpsys_ram')
    hcpu_ram_part = (
        _match_partition_by_alias(partitions, 'HCPU_RAM_DATA')
        or _find_partition(partitions, name='hcpu_ram_data')
    )
    hcpu_ro_part = (
        _match_partition_by_alias(partitions, 'HCPU_RO_DATA')
        or _find_partition(partitions, name='hcpu_ro_data')
    )
    bootloader_ram_part = (
        _match_partition_by_alias(partitions, 'BOOTLOADER_RAM_DATA')
        or _find_partition(partitions, name='bootloader_ram_data')
    )
    psram_code_part = (
        _match_partition_by_alias(partitions, 'PSRAM_CODE')
        or _find_partition(partitions, name='psram_code')
    )
    psram2_part = (
        _match_partition_by_alias(partitions, 'PSRAM_DATA2')
        or _find_partition(partitions, name='psram_data2')
    )
    flash2_img_part = (
        _match_partition_by_alias(partitions, 'HCPU_FLASH2_IMG')
        or _find_partition(partitions, name='hcpu_flash2_img')
    )
    flash2_font_part = (
        _match_partition_by_alias(partitions, 'HCPU_FLASH2_FONT')
        or _find_partition(partitions, name='hcpu_flash2_font')
    )

    def _partition_to_base_size(p: Optional[Dict[str, Any]]) -> Tuple[int, int, str]:
        if not p:
            return 0, 0, ''
        region = p.get('region', '')
        offset = ptab_module.parse_size(p.get('offset', 0))
        size = ptab_module.parse_size(p.get('size', 0))
        sbus, cbus = ptab_module.resolve_region_address(region, offset, chip_config, core=p.get('core'))
        mem_type = _get_region_mem_type(region, chip_config)
        base = _select_start_addr(mem_type, sbus, cbus)
        return int(base), int(size), str(region)

    lpsys_ram_base_ptab, lpsys_ram_size_ptab, _ = _partition_to_base_size(lpsys_ram_part)
    hcpu_ram_base_ptab, hcpu_ram_size_ptab, _ = _partition_to_base_size(hcpu_ram_part)
    hcpu_ro_base_ptab, hcpu_ro_size_ptab, _ = _partition_to_base_size(hcpu_ro_part)
    bootloader_ram_base, bootloader_ram_size, _ = _partition_to_base_size(bootloader_ram_part)
    legacy_bootloader_ram = _LEGACY_BOOTLOADER_RAM_DEFAULTS.get(str(getattr(ptab_obj, 'chip_series', '')).strip().lower())
    if bootloader_ram_part and legacy_bootloader_ram:
        try:
            if ptab_module.partition_matches_default_v3(
                bootloader_ram_part,
                partitions,
                chip_config,
                getattr(ptab_obj, 'chip_series', None),
            ):
                bootloader_ram_base, bootloader_ram_size = legacy_bootloader_ram
        except Exception:
            pass
    psram_base0, psram_size0, psram_region0 = _partition_to_base_size(psram_part)
    psram_code_base, psram_code_size, _ = _partition_to_base_size(psram_code_part)
    psram2_base0, psram2_size0, psram2_region0 = _partition_to_base_size(psram2_part)
    flash2_img_base, flash2_img_size, _ = _partition_to_base_size(flash2_img_part)
    flash2_font_base, flash2_font_size, _ = _partition_to_base_size(flash2_font_part)

    # Enforce PSRAM naming rule:
    # - 1 PSRAM  -> always `PSRAM` (no matter which mpi)
    # - 2 PSRAMs -> mpi1 is `PSRAM`, mpi2 is `PSRAM2`
    psram_base = psram_base0
    psram_size = psram_size0
    psram2_base = psram2_base0
    psram2_size = psram2_size0

    # If only psram_data2 exists, treat it as the primary PSRAM.
    if psram_size <= 0 and psram2_size > 0:
        psram_base, psram_size, psram_region0 = psram2_base0, psram2_size0, psram2_region0
        psram2_base, psram2_size = 0, 0

    # If both exist, ensure mpi1 -> PSRAM, mpi2 -> PSRAM2 when possible.
    if psram_size > 0 and psram2_size > 0:
        mpi1 = 'mpi1'
        mpi2 = 'mpi2'
        if psram_region0.strip().lower() == mpi2 and psram2_region0.strip().lower() == mpi1:
            psram_base, psram2_base = psram2_base0, psram_base0
            psram_size, psram2_size = psram2_size0, psram_size0

    # Pick ROM region for current build
    build_name = (build_name or '').strip().lower()
    build_core = (build_core or 'HCPU').strip().upper()

    rom_base = 0
    rom_size = 0
    code_p: Optional[Dict[str, Any]] = None

    if build_name == 'bootloader':
        boot_p = _find_partition(partitions, ptype='bootloader', core=build_core)
        if not boot_p:
            boot_p = _find_partition(partitions, ptype='bootloader')
        if not boot_p:
            raise LinkLdsError('bootloader partition not found in ptab v3')
        exec_def = boot_p.get('exec')
        if not isinstance(exec_def, dict):
            raise LinkLdsError('bootloader partition requires exec for link script')
        exec_region = str(exec_def.get('region', '')).strip()
        exec_offset = ptab_module.parse_size(exec_def.get('offset', 0))
        exec_sbus, exec_cbus = ptab_module.resolve_region_address(exec_region, exec_offset, chip_config, core=boot_p.get('core'))
        exec_mem_type = _get_region_mem_type(exec_region, chip_config)
        rom_base = _select_exec_addr(exec_mem_type, exec_sbus, exec_cbus)
        rom_size = int(mem_map_ints.get('FLASH_BOOT_LOADER_SIZE', 0))
        if rom_size <= 0:
            rom_size = ptab_module.parse_size(boot_p.get('size', 0))
    elif build_core == 'LCPU':
        if cmsis_dir in ('sf32lb56x', 'sf32lb58x'):
            # For 56/58, LCPU linker script layout comes from mem_map.h.
            rom_base = int(mem_map_ints.get('LCPU_RAM_CODE_START_ADDR', mem_map_ints.get('LPSYS_RAM_CBUS_BASE', 0)))
            rom_size = int(mem_map_ints.get('LCPU_RAM_CODE_SIZE', 0))
        else:
            rom_base = lcpu_rom_base
            rom_size = lcpu_rom_size
    else:
        # main/dfu or named images (HCPU)
        if build_name == 'main':
            code_p = _find_partition(partitions, ptype='app', subtype='factory', core=build_core)
            if not code_p:
                code_p = _find_partition(partitions, ptype='app', subtype='factory')
        elif build_name == 'dfu':
            code_p = _find_partition(partitions, ptype='app', subtype='dfu', core=build_core)
            if not code_p:
                code_p = _find_partition(partitions, ptype='app', subtype='dfu')
        if not code_p and build_name:
            code_p = _find_named_code_partition(partitions, build_name, build_core)
        if not code_p:
            raise LinkLdsError('code partition not found for build: {}'.format(build_name))

        size = ptab_module.parse_size(code_p.get('size', 0))
        exec_def = code_p.get('exec')
        core = code_p.get('core')
        if isinstance(exec_def, dict):
            exec_region = str(exec_def.get('region', '')).strip()
            exec_offset = ptab_module.parse_size(exec_def.get('offset', 0))
            exec_sbus, exec_cbus = ptab_module.resolve_region_address(exec_region, exec_offset, chip_config, core=core)
            exec_mem_type = _get_region_mem_type(exec_region, chip_config)
            rom_base = _select_exec_addr(exec_mem_type, exec_sbus, exec_cbus, core)
        else:
            region = code_p.get('region', '')
            offset = ptab_module.parse_size(code_p.get('offset', 0))
            exec_sbus, exec_cbus = ptab_module.resolve_region_address(region, offset, chip_config, core=core)
            mem_type = _get_region_mem_type(region, chip_config)
            rom_base = _select_exec_addr(mem_type, exec_sbus, exec_cbus, core)
        rom_size = size

    # Pick RAM region for current build
    if build_name == 'bootloader':
        ram_part = (
            _match_partition_by_alias(partitions, 'BOOTLOADER_RAM_DATA')
            or _find_partition(partitions, name='bootloader_ram_data')
            or _find_partition(partitions, name='bootloader_ram')
        )
        if ram_part:
            region = ram_part.get('region', '')
            offset = ptab_module.parse_size(ram_part.get('offset', 0))
            size = ptab_module.parse_size(ram_part.get('size', 0))
            sbus, cbus = ptab_module.resolve_region_address(region, offset, chip_config, core=ram_part.get('core'))
            mem_type = _get_region_mem_type(region, chip_config)
            ram_base = _select_start_addr(mem_type, sbus, cbus)
            ram_size = size
        else:
            ram_base, ram_size = hcpu_ram_base, hcpu_ram_size
    elif build_core == 'LCPU':
        if cmsis_dir in ('sf32lb56x', 'sf32lb58x'):
            ram_base = int(mem_map_ints.get('LPSYS_RAM_BASE', 0))
            mbox_size = int(mem_map_ints.get('LCPU_MBOX_SIZE', mem_map_ints.get('LPSYS_MBOX_BUF_SIZE', 0)))
            ram_size = max(0, int(mem_map_ints.get('LPSYS_RAM_SIZE', 0)) - mbox_size)
        else:
            ram_base, ram_size = lcpu_ram_base, lcpu_ram_size
    elif build_core == 'ACPU' and code_p:
        # ACPU code executes from RAM mapped by `exec`.
        exec_def = code_p.get('exec')
        if isinstance(exec_def, dict):
            exec_region = str(exec_def.get('region', '')).strip()
            exec_offset = ptab_module.parse_size(exec_def.get('offset', 0))
            exec_sbus, exec_cbus = ptab_module.resolve_region_address(exec_region, exec_offset, chip_config, core=code_p.get('core'))
            exec_mem_type = _get_region_mem_type(exec_region, chip_config)
            ram_base = _select_start_addr(exec_mem_type, exec_sbus, exec_cbus)
            ram_size = ptab_module.parse_size(code_p.get('size', 0))
        else:
            ram_base, ram_size = hcpu_ram_base, hcpu_ram_size
    else:
        ram_base, ram_size = hcpu_ram_base, hcpu_ram_size

    # Prefer project-level ptab RAM windows when present so custom example
    # linker scripts can keep using legacy raw macros without drifting to the
    # board/chip default mem_map.h layout.
    if build_core == 'HCPU' and hcpu_ram_base_ptab and hcpu_ram_size_ptab:
        hcpu_ram_base = int(hcpu_ram_base_ptab)
        hcpu_ram_size = int(hcpu_ram_size_ptab)
        if not hcpu_ro_base_ptab and hcpu_ro_size_ptab == 0:
            hcpu_ro_base_ptab = int(hcpu_ram_base + hcpu_ram_size)
            hcpu_ro_size_ptab = hcpu_ro_data_size
        hcpu_rom_ex_base = int(hcpu_ro_base_ptab)
        hcpu_rom_ex_size = int(hcpu_ro_size_ptab)
        if build_name != 'bootloader':
            ram_base, ram_size = hcpu_ram_base, hcpu_ram_size

    out: Dict[str, Any] = dict(rtconfig_defines or {})
    # Expose selected mem_map.h constants so templates can consume chip-level
    # addresses without hardcoding them.
    out.update(mem_map_ints)
    if lpsys_ram_base_ptab and lpsys_ram_size_ptab:
        out['LPSYS_RAM_BASE'] = int(lpsys_ram_base_ptab)
        out['LPSYS_RAM_SIZE'] = int(lpsys_ram_size_ptab)
    out['__ROM_BASE'] = int(rom_base)
    out['__ROM_SIZE'] = int(rom_size)
    out['__RAM_BASE'] = int(ram_base)
    out['__RAM_SIZE'] = int(ram_size)
    out['__ROM_EX_BASE'] = int(hcpu_rom_ex_base if build_core == 'HCPU' else 0)
    out['__ROM_EX_SIZE'] = int(hcpu_rom_ex_size if build_core == 'HCPU' else 0)
    out['__PSRAM_BASE'] = int(psram_base)
    out['__PSRAM_SIZE'] = int(psram_size)
    out['__PSRAM2_BASE'] = int(psram2_base)
    out['__PSRAM2_SIZE'] = int(psram2_size)
    out['PSRAM_CODE_START_ADDR'] = int(psram_code_base)
    out['PSRAM_CODE_SIZE'] = int(psram_code_size)
    out['PSRAM_DATA_START_ADDR'] = int(psram_base)
    out['PSRAM_DATA_SIZE'] = int(psram_size)
    out['PSRAM_DATA2_START_ADDR'] = int(psram2_base)
    out['PSRAM_DATA2_SIZE'] = int(psram2_size)
    out['HCPU_RAM_DATA_START_ADDR'] = int(hcpu_ram_base)
    out['HCPU_RAM_DATA_SIZE'] = int(hcpu_ram_size)
    out['HCPU_RO_DATA_START_ADDR'] = int(hcpu_ro_base_ptab if hcpu_ro_base_ptab or hcpu_ro_size_ptab else hcpu_ram_base + hcpu_ram_size)
    out['HCPU_RO_DATA_SIZE'] = int(hcpu_ro_size_ptab)
    out['BOOTLOADER_RAM_DATA_START_ADDR'] = int(bootloader_ram_base)
    out['BOOTLOADER_RAM_DATA_SIZE'] = int(bootloader_ram_size)
    out['__RET_RAM_BASE'] = int(mem_map_ints.get('HPSYS_RETM_BASE', 0))
    out['__RET_RAM_SIZE'] = int(mem_map_ints.get('HPSYS_RETM_SIZE', 0))
    out['__ITCM_BASE'] = int(mem_map_ints.get('HPSYS_ITCM_BASE', 0))
    out['__ITCM_SIZE'] = int(mem_map_ints.get('HPSYS_ITCM_SIZE', 0))

    # Optional flash2 layout (used by some HCPU linker scripts)
    rom2_base = int(mem_map_ints.get('QSPI2_MEM_BASE', 0))
    rom2_size = int(mem_map_ints.get('HCPU_FLASH2_IMG_SIZE', 0))
    rom3_size = int(mem_map_ints.get('HCPU_FLASH2_FONT_SIZE', 0))
    if flash2_img_base and flash2_img_size:
        rom2_base = int(flash2_img_base)
        rom2_size = int(flash2_img_size)
    if flash2_font_base and flash2_font_size:
        rom3_size = int(flash2_font_size)
    out['__ROM2_BASE'] = rom2_base
    out['__ROM2_SIZE'] = rom2_size
    if flash2_font_base and flash2_font_size:
        out['__ROM3_BASE'] = int(flash2_font_base)
    else:
        out['__ROM3_BASE'] = int(rom2_base + rom2_size) if rom2_base and rom2_size else 0
    out['__ROM3_SIZE'] = rom3_size
    out['HCPU_FLASH2_IMG_START_ADDR'] = int(out['__ROM2_BASE'])
    out['HCPU_FLASH2_IMG_SIZE'] = int(out['__ROM2_SIZE'])
    out['HCPU_FLASH2_FONT_START_ADDR'] = int(out['__ROM3_BASE'])
    out['HCPU_FLASH2_FONT_SIZE'] = int(out['__ROM3_SIZE'])

    # 56x/58x LCPU extra regions
    if build_core == 'LCPU' and cmsis_dir in ('sf32lb56x', 'sf32lb58x'):
        out['__DTCM_BASE'] = int(mem_map_ints.get('LPSYS_DTCM_BASE', 0))
        out['__DTCM_SIZE'] = int(mem_map_ints.get('LPSYS_DTCM_SIZE', 0))
        # ROM2 (flash) starts right after bootloader in internal flash.
        boot_p = _find_partition(partitions, ptype='bootloader', core='HCPU') or _find_partition(partitions, ptype='bootloader')
        if boot_p and isinstance(boot_p.get('exec'), dict):
            b_exec = boot_p['exec']
            b_region = str(b_exec.get('region', '')).strip()
            b_offset = ptab_module.parse_size(b_exec.get('offset', 0))
            b_sbus, b_cbus = ptab_module.resolve_region_address(b_region, b_offset, chip_config, core=boot_p.get('core'))
            b_mem_type = _get_region_mem_type(b_region, chip_config)
            b_base = _select_exec_addr(b_mem_type, b_sbus, b_cbus)
            bl_size = int(mem_map_ints.get('FLASH_BOOT_LOADER_SIZE', 0))
            if bl_size <= 0:
                bl_size = ptab_module.parse_size(boot_p.get('size', 0))
            else:
                bl_storage = ptab_module.parse_size(boot_p.get('size', 0))
                if bl_storage > bl_size:
                    bl_size = bl_storage
            out['__ROM2_BASE'] = int(b_base + bl_size)
        out['__ROM2_SIZE'] = int(mem_map_ints.get('LCPU_FLASH_CODE_SIZE', 0))

    # app/ex resource partitions: dedicated MEMORY regions and output sections
    reserved_memory_names = {'ROM', 'RAM', 'ROM_EX', 'PSRAM', 'PSRAM2', 'ROM2', 'ROM3', 'DTCM'}
    int_res_parts: List[Dict[str, Any]] = []
    for p in ptab_module.iter_int_res_partitions_v3(ptab_obj, core=build_core):
        if ptab_module.get_legacy_int_res_kind_v3(p) is not None:
            continue
        name = (p.get('name') or '').strip()
        if not name:
            continue
        name_upper = name.upper()
        if name_upper in reserved_memory_names:
            raise LinkLdsError('int_res partition name conflicts with reserved MEMORY name: {}'.format(name_upper))

        region = (p.get('region') or '').strip()
        offset = ptab_module.parse_size(p.get('offset', 0))
        size = ptab_module.parse_size(p.get('size', 0))
        if size <= 0:
            continue
        base = ptab_module.get_download_addr_v3(region, offset, chip_config, core=p.get('core'))
        selectors = ptab_module.get_partition_selectors_v3(p)
        int_res_parts.append(
            {
                'name': name,
                'name_upper': name_upper,
                'base': int(base),
                'size': int(size),
                'selectors': selectors,
                'selectors_gnu': [s for s in (_selector_to_gnu(sel) for sel in selectors) if s],
                'selectors_keil': [s for s in (_selector_to_keil(sel) for sel in selectors) if s],
            }
        )
    out['INT_RES_PARTS'] = int_res_parts
    return out


def render_link_lds(template_path: str, output_path: str, defines: Dict[str, Any], search_paths: Optional[Sequence[str]] = None) -> None:
    template_path = os.path.abspath(str(template_path))
    output_path = os.path.abspath(str(output_path))

    template_dir = os.path.dirname(template_path)
    sifli_sdk_root = os.getenv('SIFLI_SDK') or os.getcwd()

    loader_paths = list(search_paths or [])
    loader_paths = [os.path.abspath(p) for p in loader_paths if p]
    # Always search template dir first, then repo root.
    for p in (template_dir, sifli_sdk_root):
        ap = os.path.abspath(p)
        if ap not in loader_paths:
            loader_paths.append(ap)

    env = Environment(
        loader=FileSystemLoader(loader_paths),
        undefined=StrictUndefined,
        keep_trailing_newline=True,
        autoescape=False,
    )
    env.filters['hex32'] = hex32
    env.globals['hex32'] = hex32

    # Load by basename; includes are resolved via loader paths.
    template = env.get_template(os.path.basename(template_path))
    rendered = template.render(defines=defines)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(rendered)
