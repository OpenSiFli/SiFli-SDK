# Copyright (c) 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import copy
import json
import logging
import os
import re
from collections import OrderedDict
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple, Union

import yaml

_PTAB_CACHE = {}
_OVERLAY_SUMMARY_CACHE: Set[Tuple[str, str, str]] = set()
_OVERLAY_IGNORE_WARNING_CACHE: Set[Tuple[str, str, str]] = set()

_PTAB_OVERLAY_FILENAME = 'ptab.overlay.yaml'
_PTAB_OVERLAY_TOP_KEYS = {'partitions'}
_PTAB_OVERLAY_PARTITION_KEYS = {
    'name',
    'op',
    'type',
    'subtype',
    'region',
    'offset',
    'size',
    'core',
    'attrs',
    'aliases',
    'exec',
    'sections',
}

NAND_BLOCK_SIZE_DEFAULT = 128 * 1024
NAND_BLOCK_SIZE_CHOICES = (128 * 1024, 256 * 1024)
SF32LB52_NAND_AUTO_FLASH_MAC_ADDRESS = 0x62000000

# SiliconSchema paths
#
# Preferred layout is a git submodule at `tools/SiliconSchema`, but some
# developer environments keep SiliconSchema as a sibling repo next to the SDK.
# Support both so ptab v3 can always resolve chip-internal regions.
_SILICON_SCHEMA_ROOT: Optional[Path] = None


def _looks_like_silicon_schema_root(root: Path) -> bool:
    try:
        return (root / 'common' / 'mpi').is_dir() and (root / 'common' / 'ram').is_dir()
    except Exception:
        return False


def _get_silicon_schema_root() -> Path:
    global _SILICON_SCHEMA_ROOT
    if _SILICON_SCHEMA_ROOT is not None:
        return _SILICON_SCHEMA_ROOT

    candidates: List[Path] = []
    env_path = os.environ.get('SIFLI_SILICON_SCHEMA') or os.environ.get('SILICON_SCHEMA_PATH')
    if env_path:
        candidates.append(Path(env_path).expanduser())

    tools_dir = Path(__file__).resolve().parents[1]  # .../tools
    repo_root = tools_dir.parent

    # 1) Submodule in SDK repo
    candidates.append(tools_dir / 'SiliconSchema')
    # 2) Sibling checkout: ../SiliconSchema
    candidates.append(repo_root.parent / 'SiliconSchema')

    tried: List[str] = []
    for c in candidates:
        tried.append(str(c))
        try:
            if _looks_like_silicon_schema_root(c):
                _SILICON_SCHEMA_ROOT = c
                return c
        except Exception:
            continue

    raise FileNotFoundError(
        "SiliconSchema root not found. Please init git submodule 'tools/SiliconSchema' "
        "or set env var SIFLI_SILICON_SCHEMA/SILICON_SCHEMA_PATH. Tried: {}".format(', '.join(tried))
    )

# Chip config cache
_CHIP_CONFIG_CACHE = {}


def _normalize_selector_object_name(obj: Any) -> str:
    """Normalize selector object globs to a basename-oriented pattern."""
    obj_s = str(obj or '').strip()
    if not obj_s:
        return ''
    while obj_s.startswith('*'):
        obj_s = obj_s[1:]
    if obj_s.endswith('.o'):
        obj_s = obj_s[:-2]
    return obj_s.strip()


def normalize_partition_sections(sections: Any) -> List[Dict[str, str]]:
    """Normalize structured partition section selectors.

    Canonical form:
    - [{'section': '.FOO*'}]
    - [{'object': 'bar_*', 'section': '.rodata*'}]
    """
    if sections is None:
        return []

    if isinstance(sections, dict):
        items = [sections]
    elif isinstance(sections, list):
        items = sections
    else:
        return []

    normalized: List[Dict[str, str]] = []
    seen: Set[Tuple[str, str]] = set()
    for item in items:
        if isinstance(item, str):
            section = item.strip()
            obj = ''
        elif isinstance(item, dict):
            section = str(item.get('section') or '').strip()
            obj = _normalize_selector_object_name(item.get('object'))
        else:
            continue

        if not section:
            continue

        selector: Dict[str, str] = {'section': section}
        if obj:
            selector['object'] = obj

        key = (selector.get('object', ''), selector['section'])
        if key in seen:
            continue
        seen.add(key)
        normalized.append(selector)

    return normalized


def is_app_ex_partition_v3(partition: Any) -> bool:
    if not isinstance(partition, dict):
        return False
    if str(partition.get('type') or '').strip().lower() != 'app':
        return False
    return str(partition.get('subtype') or '').strip().lower() == 'ex'


def get_partition_selectors_v3(partition: Any) -> List[Dict[str, str]]:
    """Return canonical selector list for a v3 resource partition."""
    if not isinstance(partition, dict):
        return []

    name = str(partition.get('name') or '').strip()
    if not name:
        return normalize_partition_sections(partition.get('sections'))

    selectors: List[Dict[str, str]] = [{'section': '.{}*'.format(name.upper())}]
    selectors.extend(normalize_partition_sections(partition.get('sections')))

    deduped: List[Dict[str, str]] = []
    seen: Set[Tuple[str, str]] = set()
    for selector in selectors:
        key = (selector.get('object', ''), selector.get('section', ''))
        if not key[1] or key in seen:
            continue
        seen.add(key)
        deduped.append(selector)
    return deduped


class Ptab:
    def __init__(self, path, mems):
        self.path = os.path.abspath(path)
        self.mems = mems
        header = mems[0] if mems else {}
        self._has_header = isinstance(header, dict) and ('version' in header)
        self._version = str(header.get('version')) if self._has_header else None

    @property
    def has_header(self):
        return self._has_header

    @property
    def header(self):
        return self.mems[0] if self._has_header else {}

    @property
    def version(self):
        return self._version if self._has_header else "1"

    def is_v2(self):
        return self.version == "2"

    def is_v3(self):
        return False

    def content_mems(self, clone=True):
        mems = self.mems[1:] if self._has_header else self.mems
        return copy.deepcopy(mems) if clone else mems

    def ensure_default_regions(self):
        mems = self.content_mems()
        if self.is_v2():
            add_default_regions(mems)
        return mems


class PtabV3:
    """ptab v3 格式解析器，使用 YAML 格式

    Notes:
    - v3 uses YAML dict with top-level keys: version/chip/memory/partitions
    - Execution address is described by `exec: {region, offset}`.
      `acc` / `exec_region` / `exec_offset` are NOT supported in v3.
    - `aliases` is used to generate legacy macro base names and to provide
      compatibility with v1/v2 projects.
    """

    def __init__(self, path, data):
        self.path = os.path.abspath(path)
        self._data = data
        self._version = str(data.get('version', '3'))
        self._chip = data.get('chip', '')
        self._partitions = self._normalize_partitions(data.get('partitions', []))
        self._memory = data.get('memory', [])  # 外部存储配置
        self._chip_config = None

        # Inject internal (SiP) partitions derived from SiliconSchema.
        # NOTE: These partitions are chip-internal and should not be described
        #       in board-level ptab.yaml.
        self._inject_internal_partitions()
        self._inject_default_partitions()

    @property
    def version(self):
        return self._version

    @property
    def chip(self):
        return self._chip

    @property
    def memory(self):
        return copy.deepcopy(self._memory)

    @property
    def chip_series(self):
        """获取芯片系列名（小写），如 'sf32lb52'"""
        if self._chip:
            # SF32LB52 -> sf32lb52, SF32LB525UC6 -> sf32lb52
            match = re.match(r'(SF32LB\d{2})', self._chip, re.IGNORECASE)
            if match:
                return match.group(1).lower()
        return None

    @property
    def partitions(self):
        # Ensure internal partitions (e.g. PSRAM windows) are injected even if
        # build options weren't ready during __init__.
        self._inject_internal_partitions()
        self._inject_default_partitions()
        return copy.deepcopy(self._partitions)

    def _normalize_partitions(self, partitions):
        """Normalize partitions to the canonical v3 shape.

        Canonical fields:
        - name/type/subtype/region/offset/size/core/attrs/aliases/exec/sections
        """
        if not isinstance(partitions, list):
            return []

        normalized = []
        for p in partitions:
            if not isinstance(p, dict):
                continue

            name = (p.get('name') or '').strip()
            ptype = (p.get('type') or '').strip()
            subtype = (p.get('subtype') or '').strip() or None

            region = (p.get('region') or '').strip()

            offset = p.get('offset', 0)
            size = p.get('size', 0)

            core = (p.get('core') or '').strip() or None

            # attrs: keep as-is but only allow dict
            attrs = p.get('attrs')
            if not isinstance(attrs, dict):
                attrs = {}

            # aliases: list[str]
            aliases = p.get('aliases')
            if aliases is None:
                aliases_list = []
            elif isinstance(aliases, list):
                aliases_list = [str(x).strip() for x in aliases if str(x).strip()]
            else:
                aliases_list = [str(aliases).strip()] if str(aliases).strip() else []

            # exec: canonical execution address
            exec_def = None
            if isinstance(p.get('exec'), dict):
                exec_region = (p['exec'].get('region') or '').strip()
                exec_offset = p['exec'].get('offset', 0)
                if exec_region:
                    exec_def = {'region': exec_region, 'offset': exec_offset}

            out = {
                'name': name,
                'type': ptype,
                'region': region,
                'offset': offset,
                'size': size,
            }
            if subtype:
                out['subtype'] = subtype
            if core:
                out['core'] = core
            if attrs:
                out['attrs'] = attrs
            if aliases_list:
                out['aliases'] = aliases_list
            if exec_def:
                out['exec'] = exec_def
            sections = normalize_partition_sections(p.get('sections'))
            if sections:
                out['sections'] = sections

            normalized.append(out)

        return normalized

    def is_v2(self):
        return False

    def is_v3(self):
        return True

    def get_chip_config(self):
        """获取芯片配置（MPI + RAM + SDMMC + Memory）
        
        合并以下来源：
        1. 基础 MPI/RAM 配置（从 SiliconSchema 加载）
        2. 芯片型号的 SiP memory（从 chip.yaml 获取）
        3. ptab.yaml 中的 memory 字段（外部存储）
        """
        if self._chip_config is None:
            series = self.chip_series
            if series:
                self._chip_config = load_chip_config(series)
            else:
                self._chip_config = {'mpi': {}, 'ram': {}, 'sdmmc': {}}
            
            # 合并芯片型号的 SiP memory 和 ptab.yaml 的 memory 配置
            self._chip_config = self._merge_memory_config(self._chip_config)
        else:
            # Build options (Kconfig) may become available after the ptab object
            # is instantiated. Refresh inferred SiP memory info to avoid
            # caching an incomplete memory_info (which would break PSRAM
            # injection and exec address selection).
            try:
                self._infer_sip_memory_from_kconfig(self._chip_config)
            except Exception:
                pass
        return self._chip_config

    def _merge_memory_config(self, config):
        """合并 memory 配置到 chip_config
        
        将 ptab.yaml 的 memory 字段和 chip.yaml 的 variant memory 合并到 mpi 配置中。
        memory 字段用于描述每个 mpi 上挂载的存储器类型和大小。
        """
        import copy
        config = copy.deepcopy(config)
        
        # 初始化 memory_info 存储每个外部存储 region 的存储器信息
        if 'memory_info' not in config:
            config['memory_info'] = {}
        
        # 加载芯片型号的 SiP memory 配置
        chip_memory = self._load_chip_variant_memory()
        for mem in chip_memory:
            mpi = mem.get('mpi')
            if mpi:
                config['memory_info'][mpi] = {
                    'type': mem.get('type', 'unknown'),
                    'size': mem.get('size', 0),
                    'sip': True,
                }
        
        # 合并 ptab.yaml 中的 memory 字段（外部存储，覆盖 SiP）
        for mem in self._memory:
            # Canonical keys in docs/examples: `mpi` / `sdmmc`
            # Backward-compat: allow `name: mpiN` or `name: sdmmcN`.
            mem_region = mem.get('mpi')
            if not mem_region:
                mem_region = mem.get('sdmmc')
            if not mem_region:
                name = mem.get('name')
                if isinstance(name, str):
                    name = name.strip()
                    if name.startswith('mpi') or name.startswith('sdmmc'):
                        mem_region = name
            if mem_region:
                memory_entry = {
                    'type': mem.get('type', 'unknown'),
                    'size': parse_size(mem.get('size', 0)),
                    'sip': False,
                }
                if 'block_size' in mem:
                    try:
                        memory_entry['block_size'] = parse_size(mem.get('block_size', 0))
                    except (ValueError, TypeError):
                        memory_entry['block_size'] = mem.get('block_size')
                    memory_entry['block_size_explicit'] = True
                config['memory_info'][str(mem_region).strip()] = memory_entry

        # Fallback: infer chip-internal (SiP) memories from Kconfig when the
        # SiliconSchema `chips/*/chip.yaml` database is unavailable.
        #
        # This keeps ptab.yaml free of internal PSRAM declarations while still
        # providing correct region types/sizes for link scripts and ptab.h.
        try:
            self._infer_sip_memory_from_kconfig(config)
        except Exception:
            pass
        
        return config

    def _infer_sip_memory_from_kconfig(self, config: Dict[str, Any]) -> None:
        """Infer SiP memory type/size from Kconfig build options.

        Only fills missing `memory_info` entries for `sip: true` MPIs.
        """
        memory_info = config.get('memory_info')
        if not isinstance(memory_info, dict):
            return

        mpi_cfg = config.get('mpi') or {}
        if not isinstance(mpi_cfg, dict):
            return

        def _to_int(v: Any) -> Optional[int]:
            if v is None:
                return None
            if v is False:
                return None
            if v is True:
                return 1
            try:
                if isinstance(v, str):
                    return int(v, 0)
                return int(v)
            except Exception:
                return None

        def _mode_to_type(mode: Optional[int]) -> Optional[str]:
            if mode is None:
                return None
            if mode == 0:
                return 'nor'
            if mode == 1:
                return 'nand'
            if mode in (2, 3, 4, 5, 6):
                return 'psram'
            return None

        for mpi_name, mpi_def in mpi_cfg.items():
            if not isinstance(mpi_def, dict):
                continue
            if not mpi_def.get('sip'):
                continue
            mpi_key = str(mpi_name).strip().lower()
            if not mpi_key.startswith('mpi'):
                continue
            if mpi_key in memory_info:
                continue

            m = re.match(r'^mpi(\d+)$', mpi_key)
            if not m:
                continue
            idx = m.group(1)

            mode = _to_int(_get_depend(f'BSP_QSPI{idx}_MODE'))
            mtype = _mode_to_type(mode)
            if not mtype:
                continue

            size_mb = _to_int(_get_depend(f'BSP_QSPI{idx}_MEM_SIZE'))
            if not size_mb or size_mb <= 0:
                continue

            memory_info[mpi_key] = {
                'type': mtype,
                'size': int(size_mb) * 1024 * 1024,
                'sip': True,
                'inferred': True,
            }

    def _load_chip_variant_memory(self):
        """从 chip.yaml 加载精确芯片型号的 memory 配置"""
        if not self._chip:
            return []
        
        # 查找芯片定义文件
        series = self.chip_series
        if not series:
            return []

        chips_root = _get_silicon_schema_root() / 'chips'

        # Known naming patterns in this repo:
        # - SF32LB52x/chip.yaml
        # - SF32LB52_X/chip.yaml
        preferred_dirs = [
            chips_root / f'{series.upper()}x',
            chips_root / f'{series.upper()}_X',
        ]

        # Also scan any matching directories for robustness
        scan_dirs = []
        try:
            for d in chips_root.iterdir():
                if not d.is_dir():
                    continue
                if d.name.upper().startswith(series.upper()):
                    scan_dirs.append(d)
        except Exception:
            scan_dirs = []

        checked = []
        for d in preferred_dirs + scan_dirs:
            chip_file = d / 'chip.yaml'
            if not chip_file.exists():
                continue
            if chip_file in checked:
                continue
            checked.append(chip_file)

            try:
                with open(chip_file, encoding='utf-8-sig') as f:
                    chip_data = yaml.safe_load(f)
            except Exception:
                continue

            variants = chip_data.get('variants', [])
            for variant in variants:
                if variant.get('part_number', '').upper() == self._chip.upper():
                    return variant.get('memory', []) or []

        return []

    def _inject_internal_partitions(self) -> None:
        """Inject chip-internal partitions derived from SiliconSchema.

        Currently injected:
        - PSRAM data windows (`psram_data`, `psram_data2`) derived from chip
          variant SiP memory, excluding any execution reservations described by
          `exec` in ptab.yaml (e.g. NAND load-to-PSRAM).
        """
        try:
            chip_config = self.get_chip_config()
        except Exception:
            # If SiliconSchema is unavailable, keep the original partitions.
            # Downstream builders may fail with a clearer error.
            return

        memory_info = chip_config.get('memory_info', {}) or {}

        def _has_partition(name: str) -> bool:
            name_l = (name or '').strip().lower()
            if not name_l:
                return False
            for p in self._partitions:
                if not isinstance(p, dict):
                    continue
                if (p.get('name') or '').strip().lower() == name_l:
                    return True
                for a in (p.get('aliases') or []):
                    if str(a).strip().lower() == name_l:
                        return True
            return False

        def _region_to_mpi(region: str) -> Optional[str]:
            region = (region or '').strip().lower()
            if not region:
                return None
            if region.startswith('mpi'):
                return region
            m = re.match(r'^psram(\d+)?$', region)
            if m:
                n = m.group(1) or '1'
                return f'mpi{n}'
            return None

        # Collect SiP PSRAM mpis and sizes
        psram_mpis: List[str] = []
        psram_sizes: Dict[str, int] = {}
        for mpi, info in memory_info.items():
            if not isinstance(info, dict):
                continue
            if not info.get('sip'):
                continue
            mtype = (info.get('type') or '').strip().lower()
            if mtype != 'psram':
                continue
            size = int(info.get('size') or 0)
            if size <= 0:
                continue
            mpi_l = str(mpi).strip().lower()
            if not mpi_l.startswith('mpi'):
                continue
            psram_mpis.append(mpi_l)
            psram_sizes[mpi_l] = size

        if not psram_mpis:
            return

        # Compute reserved execution windows on each PSRAM mpi, based on `exec`.
        reserved_end: Dict[str, int] = {}
        for p in self._partitions:
            if not isinstance(p, dict):
                continue
            exec_def = p.get('exec')
            if not isinstance(exec_def, dict):
                continue
            exec_region = str(exec_def.get('region', '')).strip()
            exec_offset = parse_size(exec_def.get('offset', 0))
            size = parse_size(p.get('size', 0))
            if size <= 0:
                continue
            mpi = _region_to_mpi(exec_region)
            if not mpi:
                continue
            mpi_l = mpi.lower()
            if mpi_l not in psram_sizes:
                continue
            end = exec_offset + size
            reserved_end[mpi_l] = max(int(reserved_end.get(mpi_l, 0)), int(end))

        # Partition naming rule for injected PSRAM data windows:
        # - 1 PSRAM  -> inject `psram_data` on that mpi
        # - 2 PSRAMs -> mpi1 inject `psram_data`, mpi2 inject `psram_data2`
        psram_mpis_sorted = sorted(set(psram_mpis), key=lambda x: (len(x), x))
        if len(psram_mpis_sorted) == 1:
            name_map = {psram_mpis_sorted[0]: 'psram_data'}
        else:
            name_map: Dict[str, str] = {}
            if 'mpi1' in psram_mpis_sorted:
                name_map['mpi1'] = 'psram_data'
            if 'mpi2' in psram_mpis_sorted:
                name_map['mpi2'] = 'psram_data2'
            # Fallback for unusual layouts: assign remaining mpis deterministically.
            remaining = [m for m in psram_mpis_sorted if m not in name_map]
            if remaining and 'psram_data' not in name_map.values():
                name_map[remaining.pop(0)] = 'psram_data'
            if remaining and 'psram_data2' not in name_map.values():
                name_map[remaining.pop(0)] = 'psram_data2'

        injected: List[Dict[str, Any]] = []
        for mpi, pname in name_map.items():
            if not pname:
                continue
            if _has_partition(pname):
                continue
            total = int(psram_sizes.get(mpi, 0))
            if total <= 0:
                continue
            off = int(reserved_end.get(mpi, 0))
            if off < 0:
                off = 0
            if off > total:
                raise ValueError(
                    f"PSRAM exec reservation overflow: {mpi} reserved {off} bytes > total {total} bytes"
                )
            size = total - off
            if size <= 0:
                continue
            injected.append(
                {
                    'name': pname,
                    'type': 'data',
                    'subtype': 'ram',
                    'region': mpi,
                    'offset': off,
                    'size': size,
                }
            )

        if injected:
            self._partitions.extend(self._normalize_partitions(injected))

    def _inject_default_partitions(self) -> None:
        """Inject default partitions omitted from compact v3 YAML files.

        These defaults preserve legacy SDK behaviour for:
        - flash table / bootloader storage layout
        - common RAM windows consumed by ptab.h / linker generation
        """
        try:
            chip_config = self.get_chip_config()
        except Exception:
            return

        existing = set()
        for p in self._partitions:
            if not isinstance(p, dict):
                continue
            name = str(p.get('name') or '').strip().lower()
            if name:
                existing.add(name)

        injected: List[Dict[str, Any]] = []
        for name, part in _infer_default_partitions_v3(self._partitions, chip_config, self.chip_series).items():
            if name in existing:
                continue
            injected.append(part)

        if injected:
            self._partitions.extend(self._normalize_partitions(injected))

    def content_mems(self, clone=True):
        """转换为 v1/v2 兼容的 mems 格式，用于向后兼容"""
        return self._convert_to_mems(clone)

    def _convert_to_mems(self, clone=True):
        """将 v3 partitions 转换为 v2 mems 格式"""
        # Some builders still consume v1/v2-style mems, so ensure internal
        # partitions are present before conversion.
        self._inject_internal_partitions()
        self._inject_default_partitions()
        chip_config = self.get_chip_config()
        mems_dict = OrderedDict()

        for partition in self._partitions:
            region = partition.get('region', '')
            if not region:
                continue

            # 获取 region 基地址
            core = partition.get('core')
            sbus_base, cbus_base = resolve_region_address(region, 0, chip_config, core=core)

            # Select a "download/storage" base address to mimic v1/v2 behaviour
            mem_type = _get_region_memory_type(region, chip_config).lower()
            if mem_type == 'nor':
                mem_base = cbus_base
            else:
                mem_base = sbus_base

            mem_name = region

            if mem_name not in mems_dict:
                mems_dict[mem_name] = {
                    'mem': mem_name,
                    'base': '0x{:08X}'.format(mem_base),
                    'regions': []
                }

            # 转换分区
            offset = parse_size(partition.get('offset', 0))
            size = parse_size(partition.get('size', 0))
            name = partition.get('name', '')
            ptype = partition.get('type', '')
            subtype = partition.get('subtype', '')

            # Collect tags for v1/v2 compatibility.
            tags = []
            for alias in partition.get('aliases', []) or []:
                alias = str(alias).strip()
                if alias:
                    tags.append(alias)

            # Always include NAME as a tag for convenience.
            if name:
                tags.append(name.upper())

            # Auto-add FS_REGION tag for filesystem partitions (used by some builders)
            if ptype == 'data' and subtype in ('littlefs', 'fat', 'fatfs', 'flashdb', 'filesystem'):
                tags.append('FS_REGION')

            region_entry = {
                'offset': '0x{:08X}'.format(offset),
                'max_size': '0x{:08X}'.format(size),
                'tags': tags,
                'name': name,
            }

            # 构建 type 列表
            type_list = []
            if ptype == 'bootloader' or (ptype == 'app' and subtype != 'ex'):
                type_list.append('app_img')

            exec_def = partition.get('exec')
            if ptype == 'app' and subtype != 'ex' and not exec_def:
                # XIP case: execution region is the storage region itself
                type_list.append('app_exec')
            if type_list:
                region_entry['type'] = type_list

            if partition.get('core'):
                region_entry['core'] = partition['core']

            mems_dict[mem_name]['regions'].append(region_entry)

            # If `exec` is present and differs from storage, add an exec-only region.
            if exec_def and isinstance(exec_def, dict):
                exec_region = (exec_def.get('region') or '').strip()
                if exec_region == 'psram':
                    exec_region = 'psram1'
                if exec_region and exec_region != region:
                    exec_offset = parse_size(exec_def.get('offset', 0))
                    exec_sbus_base, exec_cbus_base = resolve_region_address(exec_region, 0, chip_config, core=core)

                    exec_mem_type = _get_region_memory_type(exec_region, chip_config).lower()
                    if exec_mem_type == 'nor':
                        exec_mem_base = exec_cbus_base
                    else:
                        exec_mem_base = exec_sbus_base

                    if exec_region not in mems_dict:
                        mems_dict[exec_region] = {
                            'mem': exec_region,
                            'base': '0x{:08X}'.format(exec_mem_base),
                            'regions': []
                        }

                    exec_tags = []
                    if ptype == 'bootloader':
                        # For v1/v2 compatibility, bootloader exec region is tagged.
                        exec_tags.append('FLASH_BOOT_LOADER')

                    exec_entry = {
                        'offset': '0x{:08X}'.format(exec_offset),
                        'max_size': '0x{:08X}'.format(size),
                        'tags': exec_tags,
                        'name': name,
                        'type': ['app_exec']
                    }
                    if partition.get('core'):
                        exec_entry['core'] = partition['core']
                    mems_dict[exec_region]['regions'].append(exec_entry)

        mems = list(mems_dict.values())
        return copy.deepcopy(mems) if clone else mems

    def ensure_default_regions(self):
        return self.content_mems()


# ============================================================================
# ptab v3 default partition helpers
# ============================================================================

_PTAB_V3_SERIES_DEFAULTS: Dict[str, Dict[str, int]] = {
    'sf32lb52': {
        'hcpu_ram_size': 0x0007FC00,
        'bootloader_ram_offset': 0x00040000,
        'bootloader_ram_size': 0x00010000,
        'lpsys_ram_size': 0x00006000,
        'bootloader_exec_offset': 0x00020000,
        'flash_table_nor_size': 0x00008000,
        'flash_table_nand_size': 0x00020000,
        'flash_table_sd_offset': 0x00001000,
        'flash_table_sd_size': 0x00008000,
        'mbr_sd_offset': 0x00000000,
        'mbr_sd_size': 0x00001000,
        'factory_data_sd_offset': 0x00041000,
        'factory_data_sd_size': 0x00020000,
        'bootloader_nor_offset': 0x00010000,
        'bootloader_nand_offset': 0x00080000,
        'bootloader_sd_offset': 0x00061000,
        'bootloader_size': 0x00010000,
    },
    'sf32lb56': {
        'flash_table_size': 0x00008000,
        'bootloader_offset': 0x00020000,
        'bootloader_size': 0x00020000,
        'bootloader_ram_offset': 0x00020000,
        'bootloader_ram_size': 0x00010000,
    },
    'sf32lb58': {
        'flash_table_size': 0x00005000,
        'bootloader_offset': 0x00020000,
        'bootloader_size': 0x00020000,
        'bootloader_ram_offset': 0x00000000,
        'bootloader_ram_size': 0x00010000,
    },
}


def _normalize_partition_core(core: Optional[str]) -> Optional[str]:
    core_s = str(core or '').strip().upper()
    return core_s or None


def _partition_exec_signature(partition: Dict[str, Any]) -> Tuple[Optional[str], Optional[int]]:
    exec_def = partition.get('exec')
    if not isinstance(exec_def, dict):
        return None, None
    region = str(exec_def.get('region') or '').strip()
    if not region:
        return None, None
    return region, parse_size(exec_def.get('offset', 0))


def _partition_compare_key(partition: Dict[str, Any]) -> Tuple[Any, ...]:
    exec_region, exec_offset = _partition_exec_signature(partition)
    return (
        str(partition.get('name') or '').strip().lower(),
        str(partition.get('type') or '').strip().lower(),
        str(partition.get('subtype') or '').strip().lower(),
        str(partition.get('region') or '').strip().lower(),
        parse_size(partition.get('offset', 0)),
        parse_size(partition.get('size', 0)),
        _normalize_partition_core(partition.get('core')),
        exec_region.lower() if exec_region else None,
        exec_offset,
    )


def _find_partition_v3(
    partitions: List[Dict[str, Any]],
    *,
    name: Optional[str] = None,
    ptype: Optional[str] = None,
    subtype: Optional[str] = None,
    core: Optional[str] = None,
) -> Optional[Dict[str, Any]]:
    for partition in partitions:
        if not isinstance(partition, dict):
            continue
        if name and str(partition.get('name') or '').strip().lower() != str(name).strip().lower():
            continue
        if ptype and str(partition.get('type') or '').strip().lower() != str(ptype).strip().lower():
            continue
        if subtype and str(partition.get('subtype') or '').strip().lower() != str(subtype).strip().lower():
            continue
        if core and _normalize_partition_core(partition.get('core')) != _normalize_partition_core(core):
            continue
        return partition
    return None


def _infer_default_boot_region_v3(
    partitions: List[Dict[str, Any]],
    chip_config: Dict[str, Any],
) -> Tuple[Optional[str], Optional[str]]:
    candidates = [
        _find_partition_v3(partitions, name='bootloader'),
        _find_partition_v3(partitions, name='flash_table'),
        _find_partition_v3(partitions, ptype='app', subtype='factory', core='HCPU'),
        _find_partition_v3(partitions, ptype='app', core='HCPU'),
        _find_partition_v3(partitions, ptype='app', subtype='factory'),
        _find_partition_v3(partitions, ptype='app'),
    ]

    for partition in candidates:
        if not partition:
            continue
        region = str(partition.get('region') or '').strip()
        if not region:
            continue
        mem_type = _get_region_memory_type(region, chip_config).strip().lower()
        if mem_type in ('nor', 'nand', 'sd'):
            return region, mem_type

    return None, None


def _get_region_nand_block_size(region: str, chip_config: Dict[str, Any]) -> int:
    memory_info = chip_config.get('memory_info', {})
    info = memory_info.get(region, {}) if isinstance(memory_info, dict) else {}
    if isinstance(info, dict):
        try:
            block_size = parse_size(info.get('block_size', NAND_BLOCK_SIZE_DEFAULT))
        except (ValueError, TypeError):
            block_size = NAND_BLOCK_SIZE_DEFAULT
        if block_size in NAND_BLOCK_SIZE_CHOICES:
            return block_size
    return NAND_BLOCK_SIZE_DEFAULT


def _infer_default_partitions_v3(
    partitions: List[Dict[str, Any]],
    chip_config: Dict[str, Any],
    chip_series: Optional[str],
) -> "OrderedDict[str, Dict[str, Any]]":
    defaults: "OrderedDict[str, Dict[str, Any]]" = OrderedDict()
    series = str(chip_series or '').strip().lower()
    cfg = _PTAB_V3_SERIES_DEFAULTS.get(series)
    if not cfg:
        return defaults

    if series == 'sf32lb52':
        boot_region, boot_mem_type = _infer_default_boot_region_v3(partitions, chip_config)
        if boot_region and boot_mem_type in ('nor', 'nand', 'sd'):
            flash_table_attrs = {}
            if boot_mem_type == 'nand':
                block_size = _get_region_nand_block_size(boot_region, chip_config)
                flash_table_offset = 0
                flash_table_size = block_size
                bootloader_offset = 4 * block_size
                flash_table_attrs['AUTO_FLASH_MAC_ADDRESS'] = SF32LB52_NAND_AUTO_FLASH_MAC_ADDRESS
            elif boot_mem_type == 'sd':
                defaults['mbr'] = {
                    'name': 'mbr',
                    'type': 'data',
                    'subtype': 'raw',
                    'region': boot_region,
                    'offset': cfg['mbr_sd_offset'],
                    'size': cfg['mbr_sd_size'],
                    'aliases': ['MBR'],
                }
                flash_table_offset = cfg['flash_table_sd_offset']
                flash_table_size = cfg['flash_table_sd_size']
                bootloader_offset = cfg['bootloader_sd_offset']
            else:
                flash_table_offset = 0
                flash_table_size = cfg['flash_table_nor_size']
                bootloader_offset = cfg['bootloader_nor_offset']

            flash_table = {
                'name': 'flash_table',
                'type': 'ftab',
                'region': boot_region,
                'offset': flash_table_offset,
                'size': flash_table_size,
            }
            if flash_table_attrs:
                flash_table['attrs'] = flash_table_attrs
            defaults['flash_table'] = flash_table
            if boot_mem_type == 'nand':
                defaults['factory_data'] = {
                    'name': 'factory_data',
                    'type': 'data',
                    'subtype': 'raw',
                    'region': boot_region,
                    'offset': 2 * block_size,
                    'size': block_size,
                    'aliases': ['FACTORY_DATA'],
                }
            defaults['bootloader'] = {
                'name': 'bootloader',
                'type': 'bootloader',
                'region': boot_region,
                'offset': bootloader_offset,
                'size': cfg['bootloader_size'],
                'core': 'HCPU',
                'exec': {
                    'region': 'hpsys_ram',
                    'offset': cfg['bootloader_exec_offset'],
                },
            }
            if boot_mem_type == 'sd':
                defaults['factory_data'] = {
                    'name': 'factory_data',
                    'type': 'data',
                    'subtype': 'raw',
                    'region': boot_region,
                    'offset': cfg['factory_data_sd_offset'],
                    'size': cfg['factory_data_sd_size'],
                    'aliases': ['FACTORY_DATA'],
                }

        defaults['hcpu_ram_data'] = {
            'name': 'hcpu_ram_data',
            'type': 'data',
            'subtype': 'ram',
            'region': 'hpsys_ram',
            'offset': 0,
            'size': cfg['hcpu_ram_size'],
        }
        defaults['bootloader_ram_data'] = {
            'name': 'bootloader_ram_data',
            'type': 'data',
            'subtype': 'ram',
            'region': 'hpsys_ram',
            'offset': cfg['bootloader_ram_offset'],
            'size': cfg['bootloader_ram_size'],
            'core': 'HCPU',
        }
        defaults['lpsys_ram'] = {
            'name': 'lpsys_ram',
            'type': 'data',
            'subtype': 'ram',
            'region': 'lpsys_ram',
            'offset': 0,
            'size': cfg['lpsys_ram_size'],
        }
        return defaults

    defaults['flash_table'] = {
        'name': 'flash_table',
        'type': 'ftab',
        'region': 'mpi5',
        'offset': 0,
        'size': cfg['flash_table_size'],
    }
    defaults['bootloader'] = {
        'name': 'bootloader',
        'type': 'bootloader',
        'region': 'mpi5',
        'offset': cfg['bootloader_offset'],
        'size': cfg['bootloader_size'],
        'core': 'HCPU',
        'exec': {
            'region': 'mpi5',
            'offset': cfg['bootloader_offset'],
        },
    }
    defaults['bootloader_ram_data'] = {
        'name': 'bootloader_ram_data',
        'type': 'data',
        'subtype': 'ram',
        'region': 'hpsys_ram',
        'offset': cfg['bootloader_ram_offset'],
        'size': cfg['bootloader_ram_size'],
        'core': 'HCPU',
    }
    return defaults


def partition_matches_default_v3(
    partition: Dict[str, Any],
    partitions: List[Dict[str, Any]],
    chip_config: Dict[str, Any],
    chip_series: Optional[str],
) -> bool:
    name = str(partition.get('name') or '').strip().lower()
    if not name:
        return False

    defaults = _infer_default_partitions_v3(partitions, chip_config, chip_series)
    default_partition = defaults.get(name)
    if not default_partition:
        return False

    return _partition_compare_key(partition) == _partition_compare_key(default_partition)


def prune_default_partitions_v3_data(data: Dict[str, Any]) -> Dict[str, Any]:
    """Remove explicit partitions that match ptab v3 defaults.

    This is used by migration/cleanup helpers to keep YAML concise while
    preserving behaviour via implicit default inference.
    """
    if not isinstance(data, dict):
        return data

    chip = str(data.get('chip') or '').strip()
    if not chip:
        return copy.deepcopy(data)

    raw_partitions = data.get('partitions')
    if not isinstance(raw_partitions, list):
        return copy.deepcopy(data)

    ptab_obj = PtabV3('<in-memory>', {
        'version': data.get('version', 3),
        'chip': chip,
        'memory': copy.deepcopy(data.get('memory') or []),
        'partitions': copy.deepcopy(raw_partitions),
    })
    chip_config = ptab_obj.get_chip_config()
    normalized_raw = ptab_obj._normalize_partitions(copy.deepcopy(raw_partitions))

    kept: List[Dict[str, Any]] = []
    for raw_part, norm_part in zip(raw_partitions, normalized_raw):
        if partition_matches_default_v3(norm_part, normalized_raw, chip_config, ptab_obj.chip_series):
            continue
        kept.append(copy.deepcopy(raw_part))

    out = copy.deepcopy(data)
    out['partitions'] = kept
    return out


# ============================================================================
# Size parsing utilities
# ============================================================================

_SIZE_UNITS = {
    'kb': 1024,
    'k': 1024,
    'mb': 1024 * 1024,
    'm': 1024 * 1024,
    'gb': 1024 * 1024 * 1024,
    'g': 1024 * 1024 * 1024,
}


def parse_size(value: Union[int, str]) -> int:
    """解析大小值，支持多种格式

    支持格式：
    - 整数: 4096
    - 十六进制字符串: "0x1000"
    - 带单位字符串: "4KB", "4kb", "4K", "2MB"

    Returns:
        int: 字节数
    """
    if isinstance(value, int):
        return value
    if not isinstance(value, str):
        return int(value)

    value = value.strip()
    if not value:
        return 0

    # 十六进制
    if value.lower().startswith('0x'):
        return int(value, 16)

    # 带单位
    match = re.match(r'^(\d+)\s*([a-zA-Z]+)$', value)
    if match:
        num = int(match.group(1))
        unit = match.group(2).lower()
        if unit in _SIZE_UNITS:
            return num * _SIZE_UNITS[unit]
        raise ValueError(f"Unknown size unit: {match.group(2)}")

    # 纯数字
    return int(value)


# ============================================================================
# Chip configuration loading
# ============================================================================

def load_chip_config(chip_series: str) -> Dict[str, Any]:
    """加载芯片 MPI + RAM + SDMMC 配置

    Args:
        chip_series: 芯片系列名，如 'sf32lb52', 'sf32lb56', 'sf32lb58'

    Returns:
        dict: {'mpi': {...}, 'ram': {...}, 'sdmmc': {...}}
    """
    if chip_series in _CHIP_CONFIG_CACHE:
        return _CHIP_CONFIG_CACHE[chip_series]

    config = {'series': chip_series, 'mpi': {}, 'ram': {}, 'sdmmc': {}}

    # 加载 MPI 配置
    schema_root = _get_silicon_schema_root()
    mpi_file = schema_root / 'common' / 'mpi' / chip_series / 'mpi.yaml'
    if not mpi_file.exists():
        raise FileNotFoundError(f"SiliconSchema MPI config not found: {mpi_file}")
    with open(mpi_file, encoding='utf-8-sig') as f:
        mpi_data = yaml.safe_load(f)
        config['mpi'] = mpi_data.get('mpis', {})

    # 加载 RAM 配置
    ram_file = schema_root / 'common' / 'ram' / chip_series / 'ram.yaml'
    if not ram_file.exists():
        raise FileNotFoundError(f"SiliconSchema RAM config not found: {ram_file}")
    with open(ram_file, encoding='utf-8-sig') as f:
        config['ram'] = yaml.safe_load(f)

    # 加载 SDMMC 配置
    sdmmc_file = schema_root / 'common' / 'sdmmc' / chip_series / 'sdmmc.yaml'
    if sdmmc_file.exists():
        with open(sdmmc_file, encoding='utf-8-sig') as f:
            sdmmc_data = yaml.safe_load(f)
            config['sdmmc'] = sdmmc_data.get('sdmmcs', {})

    _CHIP_CONFIG_CACHE[chip_series] = config
    return config


def resolve_region_address(
    region: str,
    offset: int,
    chip_config: Dict[str, Any],
    core: Optional[str] = None,
) -> Tuple[int, int]:
    """解析 region 到物理地址

    Args:
        region: region 名称，如 'mpi1', 'mpi2', 'hpsys_ram'
        offset: 偏移量
        chip_config: load_chip_config() 返回的配置

    Returns:
        tuple: (sbus_addr, cbus_addr)
    """
    mpi_config = chip_config.get('mpi', {})
    ram_config = chip_config.get('ram', {})
    sdmmc_config = chip_config.get('sdmmc', {})
    series = str(chip_config.get('series') or '').strip().lower()
    memory_info = chip_config.get('memory_info', {}) if isinstance(chip_config.get('memory_info', {}), dict) else {}

    core_key = None
    if core:
        core_lower = str(core).lower()
        if core_lower in ('hcpu', 'lcpu', 'acpu'):
            core_key = core_lower

    # MPI region
    if region in mpi_config:
        mpi = mpi_config[region]
        base_offset = mpi.get('base', {}).get('offset', 0)
        xip_offset = mpi.get('xip', {}).get('offset') if isinstance(mpi.get('xip'), dict) else None

        # SF32LB56 special case: mpi2 address window differs between FLASH2 and PSRAM2.
        # SiliconSchema provides the FLASH2 window by default. When mpi2 is used as PSRAM,
        # align with SDK mem_map.h legacy mapping (PSRAM2: 0x6080_0000 / 0x1080_0000).
        if series == 'sf32lb56' and region.lower() == 'mpi2':
            mi = memory_info.get('mpi2', {})
            mtype = (mi.get('type') or '').strip().lower() if isinstance(mi, dict) else ''
            if mtype == 'psram':
                base_offset = 0x60800000
                xip_offset = 0x10800000

        sbus_addr = int(base_offset) + int(offset)
        cbus_addr = int(xip_offset) + int(offset) if xip_offset is not None else sbus_addr
        return sbus_addr, cbus_addr

    # SDMMC region
    if region in sdmmc_config:
        sdmmc = sdmmc_config[region]
        base_offset = sdmmc.get('base', 0)
        sbus_addr = int(base_offset) + int(offset)
        return sbus_addr, sbus_addr

    def _resolve_ram_region_base(ram_banks: Dict[str, Any], default_base: int) -> int:
        # First pass: if core is specified, scan all banks for that core.
        if core_key:
            for _, bank in ram_banks.items():
                if not isinstance(bank, dict):
                    continue
                c = bank.get(core_key)
                if isinstance(c, dict):
                    base_offset = c.get('offset')
                    if base_offset is not None:
                        return int(base_offset)

        # Second pass: fallback to the first hcpu bank (legacy default behaviour).
        for _, bank in ram_banks.items():
            if not isinstance(bank, dict):
                continue
            hcpu = bank.get('hcpu')
            if isinstance(hcpu, dict):
                base_offset = hcpu.get('offset')
                if base_offset is not None:
                    return int(base_offset)

        return int(default_base)

    # RAM region (hpsys_ram -> hpsys.ram)
    if region == 'hpsys_ram' or region.startswith('hpsys'):
        hpsys = ram_config.get('hpsys', {})
        ram = hpsys.get('ram', {})
        base = _resolve_ram_region_base(ram, 0x20000000)
        return base + offset, base + offset

    if region == 'lpsys_ram' or region.startswith('lpsys'):
        lpsys = ram_config.get('lpsys', {})
        ram = lpsys.get('ram', {})
        base = _resolve_ram_region_base(ram, 0x20400000)
        return base + offset, base + offset

    # psram 别名处理：psram{x} 直接映射到 mpi{x}
    # 用于 exec 的地址使用 xip 地址
    psram_match = re.match(r'^psram(\d+)?$', region)
    if psram_match:
        # psram1 -> mpi1, psram2 -> mpi2, psram -> mpi1
        mpi_num = psram_match.group(1) or '1'
        mpi_name = f'mpi{mpi_num}'
        if mpi_name in mpi_config:
            mpi = mpi_config[mpi_name]
            base_offset = mpi.get('base', {}).get('offset', 0)
            xip_offset = mpi.get('xip', {}).get('offset') if isinstance(mpi.get('xip'), dict) else None

            if series == 'sf32lb56' and mpi_name == 'mpi2':
                mi = memory_info.get('mpi2', {})
                mtype = (mi.get('type') or '').strip().lower() if isinstance(mi, dict) else ''
                if mtype == 'psram':
                    base_offset = 0x60800000
                    xip_offset = 0x10800000

            sbus_addr = int(base_offset) + int(offset)
            cbus_addr = int(xip_offset) + int(offset) if xip_offset is not None else sbus_addr
            return sbus_addr, cbus_addr

    logging.warning(f"Unknown region: {region}, using offset as address")
    return offset, offset


def _get_region_memory_type(region: str, chip_config: Dict[str, Any]) -> str:
    """Get memory type for a region.

    For mpi/psram regions, consult chip_config['memory_info'] by canonical mpi name.
    For RAM regions, returns 'ram'.
    """
    if region == 'hpsys_ram' or region.startswith('hpsys') or region == 'lpsys_ram' or region.startswith('lpsys'):
        return 'ram'

    if region.startswith('sdmmc'):
        memory_info = chip_config.get('memory_info', {})
        info = memory_info.get(region, {})
        mtype = info.get('type')
        return str(mtype) if mtype is not None else 'sd'

    # Normalize psram aliases to mpiN
    psram_match = re.match(r'^psram(\d+)?$', region)
    if psram_match:
        mpi_num = psram_match.group(1) or '1'
        region = f'mpi{mpi_num}'

    memory_info = chip_config.get('memory_info', {})
    info = memory_info.get(region, {})
    mtype = info.get('type')
    if mtype is not None:
        return str(mtype)

    return ''


def get_download_addr_v3(
    region: str,
    offset: int,
    chip_config: Dict[str, Any],
    core: Optional[str] = None,
) -> int:
    """Get download/storage address for a v3 partition.

    Address selection rule:
    - NOR: use CBUS (XIP) view
    - NAND/PSRAM/RAM/SDMMC: use SBUS/base view

    Notes:
    - This helper is intended for ptab v3 integrations.
    - `offset` should be in bytes.
    """
    sbus_addr, cbus_addr = resolve_region_address(region, offset, chip_config, core=core)
    mem_type = _get_region_memory_type(region, chip_config).lower()
    return cbus_addr if mem_type == 'nor' else sbus_addr


def get_legacy_int_res_kind_v3(partition: Any) -> Optional[str]:
    """Return legacy int_res kind for watch-style PTAB v3 partitions."""
    if not isinstance(partition, dict):
        return None
    if partition.get('type') != 'data':
        return None

    name = str(partition.get('name') or '').strip().lower()
    if name == 'hcpu_flash2_img':
        return 'flash2_img'
    if name == 'hcpu_flash2_font':
        return 'flash2_font'

    for alias in partition.get('aliases') or []:
        alias_u = str(alias or '').strip().upper()
        if alias_u == 'HCPU_FLASH2_IMG':
            return 'flash2_img'
        if alias_u == 'HCPU_FLASH2_FONT':
            return 'flash2_font'
    return None


def iter_int_res_partitions_v3(ptab_obj: Any, core: Optional[str] = None) -> List[Dict[str, Any]]:
    """Iterate resource partitions for ptab v3.

    Filters:
    - type == 'app'
    - subtype == 'ex'
    - core matches (default: 'HCPU')
    """
    if not ptab_obj or not hasattr(ptab_obj, 'is_v3') or not ptab_obj.is_v3():
        return []

    core_u = (core or 'HCPU').strip().upper()
    out: List[Dict[str, Any]] = []
    for p in getattr(ptab_obj, 'partitions', []) or []:
        if not isinstance(p, dict):
            continue
        if not is_app_ex_partition_v3(p):
            continue
        pcore = (p.get('core') or 'HCPU').strip().upper()
        if pcore != core_u:
            continue
        out.append(p)
    return out


def _parse_ptab_file(path):
    """解析 ptab 文件（JSON 格式）"""
    with open(path, encoding='utf-8-sig') as f:
        return json.load(f, object_pairs_hook=OrderedDict)


def _parse_ptab_yaml_file(path):
    """解析 ptab 文件（YAML 格式）"""
    with open(path, encoding='utf-8-sig') as f:
        return yaml.safe_load(f)


def _is_yaml_file(path):
    """检查是否为 YAML 文件"""
    ext = os.path.splitext(path)[1].lower()
    return ext in ('.yaml', '.yml')


def _is_v3_yaml(data):
    """检查是否为 v3 格式的 YAML"""
    if isinstance(data, dict):
        version = data.get('version')
        if version is not None and str(version) == '3':
            return True
        # 如果有 partitions 字段，也认为是 v3
        if 'partitions' in data:
            return True
    return False


def normalize_board_name(board: Optional[str]) -> Optional[str]:
    """Normalize board name to include the core suffix."""
    if board is None:
        return None
    board = str(board).strip()
    if not board:
        return None
    if board.endswith(('_hcpu', '_lcpu', '_acpu')):
        return board
    return board + '_hcpu'


def get_board_paths(
    board: Optional[str],
    board_search_path: Optional[str] = None,
    sdk_root: Optional[str] = None,
) -> Tuple[Optional[str], Optional[str]]:
    """Return `(board_root, core_subdir)` for a board name."""
    board = normalize_board_name(board)
    if board is None:
        return None, None

    if board.endswith('_hcpu'):
        board_path = board[:-len('_hcpu')]
        subfolder = 'hcpu'
    elif board.endswith('_lcpu'):
        board_path = board[:-len('_lcpu')]
        subfolder = 'lcpu'
    else:
        board_path = board[:-len('_acpu')]
        subfolder = 'acpu'

    sdk_root = os.path.abspath(sdk_root or os.environ.get('SIFLI_SDK') or os.getcwd())
    default_root = os.path.join(sdk_root, 'customer', 'boards')
    board_root = default_root

    if board_search_path:
        board_search_path = os.path.abspath(board_search_path)
        if os.path.exists(os.path.join(board_search_path, board_path)):
            board_root = board_search_path

    board_path1 = os.path.join(board_root, board_path).replace('\\', '/')
    board_path2 = os.path.join(board_path1, subfolder).replace('\\', '/')
    return board_path1, board_path2


def detect_chip_dir_from_board(
    board: Optional[str],
    board_search_path: Optional[str] = None,
    sdk_root: Optional[str] = None,
) -> Optional[str]:
    """Best-effort read the board rtconfig.py and return CHIP in lowercase."""
    _, board_core_path = get_board_paths(board, board_search_path=board_search_path, sdk_root=sdk_root)
    if not board_core_path:
        return None

    rtconfig_path = os.path.join(board_core_path, 'rtconfig.py')
    if not os.path.exists(rtconfig_path):
        return None

    try:
        with open(rtconfig_path, encoding='utf-8') as f:
            content = f.read()
    except Exception:
        return None

    match = re.search(r'^\s*CHIP\s*=\s*[\'"]([^\'"]+)[\'"]', content, flags=re.MULTILINE)
    if not match:
        return None
    chip = (match.group(1) or '').strip()
    return chip.lower() if chip else None


def _find_first_ptab_file(search_dirs: List[Optional[str]]) -> Optional[str]:
    for search_dir in search_dirs:
        if not search_dir:
            continue
        yaml_path = os.path.join(search_dir, 'ptab.yaml')
        json_path = os.path.join(search_dir, 'ptab.json')
        if os.path.exists(yaml_path):
            return os.path.abspath(yaml_path)
        if os.path.exists(json_path):
            return os.path.abspath(json_path)
    return None


def _find_first_ptab_yaml_file(search_dirs: List[Optional[str]]) -> Optional[str]:
    for search_dir in search_dirs:
        if not search_dir:
            continue
        yaml_path = os.path.join(search_dir, 'ptab.yaml')
        if os.path.exists(yaml_path):
            return os.path.abspath(yaml_path)
    return None


def resolve_project_ptab_paths(
    project_root: str,
    board: Optional[str],
    chip: Optional[str],
    board_path: Optional[str] = None,
) -> Dict[str, Any]:
    """Resolve the project-local PTAB replacement/overlay paths."""
    project_root = os.path.abspath(project_root)
    board = normalize_board_name(board)
    chip = (chip or '').strip().lower() or None
    board_path = os.path.abspath(board_path) if board_path else None

    project_search_dirs: List[str] = []
    if board:
        project_search_dirs.append(os.path.join(project_root, board))
    if chip:
        project_search_dirs.append(os.path.join(project_root, chip))
    project_search_dirs.append(project_root)

    project_full_ptab = _find_first_ptab_file(project_search_dirs)
    project_yaml_ptab = _find_first_ptab_yaml_file(project_search_dirs)
    board_base_ptab = _find_first_ptab_file([board_path] if board_path else [])

    overlay_paths = {
        'chip': None,
        'board': None,
    }
    if chip:
        chip_overlay = os.path.join(project_root, chip, _PTAB_OVERLAY_FILENAME)
        if os.path.exists(chip_overlay):
            overlay_paths['chip'] = os.path.abspath(chip_overlay)
    if board:
        board_overlay = os.path.join(project_root, board, _PTAB_OVERLAY_FILENAME)
        if os.path.exists(board_overlay):
            overlay_paths['board'] = os.path.abspath(board_overlay)

    fallback_path = os.path.join(board_path, 'ptab.json') if board_path else None

    return {
        'project_root': project_root,
        'board': board,
        'chip': chip,
        'board_path': board_path,
        'project_full_ptab': project_full_ptab,
        'project_yaml_ptab': project_yaml_ptab,
        'board_base_ptab': board_base_ptab,
        'board_fallback_ptab': fallback_path,
        'overlay_paths': overlay_paths,
    }


def _load_overlay_partitions(path: str) -> List[Dict[str, Any]]:
    data = _parse_ptab_yaml_file(path)
    if not isinstance(data, dict):
        raise ValueError(f"{path}: overlay file must be a YAML mapping")

    extra_keys = sorted(set(data.keys()) - _PTAB_OVERLAY_TOP_KEYS)
    if extra_keys:
        raise ValueError(
            f"{path}: unsupported top-level key(s): {', '.join(extra_keys)}; "
            f"overlay file only supports 'partitions'"
        )

    if 'partitions' not in data:
        raise ValueError(f"{path}: overlay file must define top-level 'partitions'")

    partitions = data.get('partitions')
    if not isinstance(partitions, list):
        raise ValueError(f"{path}: 'partitions' must be a list")
    return partitions


def _validate_overlay_partition_item(path: str, index: int, item: Any) -> Dict[str, Any]:
    if not isinstance(item, dict):
        raise ValueError(f"{path}: partitions[{index}] must be a mapping")

    extra_keys = sorted(set(item.keys()) - _PTAB_OVERLAY_PARTITION_KEYS)
    if extra_keys:
        raise ValueError(
            f"{path}: partitions[{index}] contains unsupported key(s): {', '.join(extra_keys)}"
        )

    name = str(item.get('name') or '').strip()
    if not name:
        raise ValueError(f"{path}: partitions[{index}] missing required field 'name'")

    if 'op' in item and item.get('op') is not None:
        op = str(item.get('op')).strip().lower()
        if op not in ('override', 'add'):
            raise ValueError(
                f"{path}: partitions[{index}] has invalid op '{item.get('op')}', expected 'override' or 'add'"
            )

    return item


def _infer_overlay_operation(item: Dict[str, Any], exists: bool) -> Tuple[str, str]:
    if 'op' in item and item.get('op') is not None:
        return str(item.get('op')).strip().lower(), 'explicit'
    return ('override' if exists else 'add'), 'inferred'


def _merge_overlay_partition(base: Dict[str, Any], update: Dict[str, Any]) -> Dict[str, Any]:
    merged = copy.deepcopy(base)
    for key, value in update.items():
        if key in ('name', 'op'):
            continue
        if key == 'attrs':
            base_attrs = copy.deepcopy(merged.get('attrs') or {})
            if not isinstance(base_attrs, dict):
                base_attrs = {}
            if value is None:
                merged.pop('attrs', None)
                continue
            if not isinstance(value, dict):
                raise ValueError("overlay attrs must be a mapping")
            base_attrs.update(copy.deepcopy(value))
            if base_attrs:
                merged['attrs'] = base_attrs
            else:
                merged.pop('attrs', None)
            continue
        if key == 'aliases':
            if value is None:
                merged.pop('aliases', None)
            else:
                merged['aliases'] = copy.deepcopy(value)
            continue
        if key == 'exec':
            if value is None:
                merged.pop('exec', None)
            else:
                merged['exec'] = copy.deepcopy(value)
            continue
        if key == 'sections':
            if value is None:
                merged.pop('sections', None)
            else:
                merged['sections'] = copy.deepcopy(value)
            continue
        if value is None and key in ('subtype', 'core'):
            merged.pop(key, None)
            continue
        merged[key] = copy.deepcopy(value)
    return merged


def _validate_effective_ptab_v3(
    ptab_obj: Any,
    strict: bool = False,
    context: Optional[str] = None,
) -> List[Any]:
    from validate_ptab_v3 import validate_ptab_v3

    issues = validate_ptab_v3(ptab_obj)
    blocking = [
        issue for issue in issues
        if issue.severity == 'error' or (strict and issue.severity == 'warning')
    ]
    if blocking:
        prefix = f"{context}: " if context else ''
        messages = '\n'.join(str(issue) for issue in blocking)
        raise ValueError(f"{prefix}invalid ptab v3 configuration:\n{messages}")
    return issues


def build_effective_ptab_v3(
    base_ptab_path: str,
    chip_overlay_path: Optional[str] = None,
    board_overlay_path: Optional[str] = None,
    strict_validation: bool = False,
) -> Tuple[Dict[str, Any], Dict[str, Any]]:
    """Apply project overlay files to a board-level ptab v3."""
    base_ptab_path = os.path.abspath(base_ptab_path)
    base_data = _parse_ptab_yaml_file(base_ptab_path)
    if not _is_v3_yaml(base_data):
        raise ValueError(
            f"{base_ptab_path}: project overlay only supports board-level ptab.yaml (v3), "
            "ptab.json cannot be overlaid"
        )

    merged_data = copy.deepcopy(base_data)
    partitions = merged_data.get('partitions')
    if not isinstance(partitions, list):
        raise ValueError(f"{base_ptab_path}: v3 ptab must define 'partitions' as a list")
    merged_data['partitions'] = partitions

    # Materialize implicit defaults up front so project overlays can override
    # omitted default partitions such as hcpu_ram_data / lpsys_ram.
    merged_data['partitions'] = PtabV3(base_ptab_path, merged_data).partitions
    partitions = merged_data['partitions']

    report = {
        'base_path': base_ptab_path,
        'overlay_paths': {
            'chip': os.path.abspath(chip_overlay_path) if chip_overlay_path else None,
            'board': os.path.abspath(board_overlay_path) if board_overlay_path else None,
        },
        'effective_path': None,
        'operations': [],
        'validation': [],
    }

    for layer, overlay_path in (('chip', chip_overlay_path), ('board', board_overlay_path)):
        if not overlay_path:
            continue
        overlay_path = os.path.abspath(overlay_path)
        overlay_partitions = _load_overlay_partitions(overlay_path)

        for index, raw_item in enumerate(overlay_partitions):
            item = _validate_overlay_partition_item(overlay_path, index, raw_item)
            name = str(item.get('name') or '').strip()
            name_lower = name.lower()

            current_index = None
            for part_idx, partition in enumerate(partitions):
                if not isinstance(partition, dict):
                    continue
                if str(partition.get('name') or '').strip().lower() == name_lower:
                    current_index = part_idx
                    break

            op, op_mode = _infer_overlay_operation(item, current_index is not None)
            changed_fields = [key for key in item.keys() if key not in ('name', 'op')]

            if op == 'override':
                if current_index is None:
                    raise ValueError(
                        f"{overlay_path}: partitions[{index}] override target '{name}' not found in base ptab"
                    )
                partitions[current_index] = _merge_overlay_partition(partitions[current_index], item)
            else:
                if current_index is not None:
                    raise ValueError(
                        f"{overlay_path}: partitions[{index}] add target '{name}' already exists"
                    )
                missing = [key for key in ('type', 'region', 'offset', 'size') if key not in item]
                if missing:
                    raise ValueError(
                        f"{overlay_path}: partitions[{index}] add entry '{name}' missing required field(s): "
                        f"{', '.join(missing)}"
                    )
                new_item = copy.deepcopy(item)
                new_item.pop('op', None)
                partitions.append(new_item)

            report['operations'].append({
                'layer': layer,
                'kind': op,
                'mode': op_mode,
                'name': name,
                'fields': changed_fields,
                'source': overlay_path,
            })

    effective_obj = PtabV3(base_ptab_path, merged_data)
    report['validation'] = _validate_effective_ptab_v3(
        effective_obj,
        strict=strict_validation,
        context=base_ptab_path,
    )

    return merged_data, report


def dump_ptab_yaml(data: Dict[str, Any]) -> str:
    return yaml.safe_dump(
        data,
        sort_keys=False,
        allow_unicode=False,
        default_flow_style=False,
    )


def write_effective_ptab_yaml(path: str, data: Dict[str, Any]) -> str:
    path = os.path.abspath(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(dump_ptab_yaml(data))
    return path


def print_overlay_summary(report: Dict[str, Any], dedupe: bool = True) -> None:
    ops = report.get('operations') or []
    if not ops:
        return

    cache_key = (
        str(report.get('base_path') or ''),
        str((report.get('overlay_paths') or {}).get('chip') or ''),
        str((report.get('overlay_paths') or {}).get('board') or ''),
    )
    if dedupe and cache_key in _OVERLAY_SUMMARY_CACHE:
        return
    _OVERLAY_SUMMARY_CACHE.add(cache_key)

    try:
        from rich.console import Console
        from rich.panel import Panel
        from rich.table import Table
    except ImportError:
        logging.info("PTAB overlay base: %s", report.get('base_path'))
        for op in ops:
            logging.info(
                "PTAB overlay %s %s (%s): %s [%s]",
                op.get('layer'),
                op.get('kind'),
                op.get('mode'),
                op.get('name'),
                ', '.join(op.get('fields') or ['-']),
            )
        return

    console = Console(stderr=True, width=180)

    summary_lines = [
        f"[bold]Base PTAB[/bold]: {report.get('base_path')}",
    ]
    overlay_paths = report.get('overlay_paths') or {}
    if overlay_paths.get('chip'):
        summary_lines.append(f"[bold]Chip Overlay[/bold]: {overlay_paths.get('chip')}")
    if overlay_paths.get('board'):
        summary_lines.append(f"[bold]Board Overlay[/bold]: {overlay_paths.get('board')}")
    if report.get('effective_path'):
        summary_lines.append(f"[bold]Effective PTAB[/bold]: {report.get('effective_path')}")
    console.print(Panel.fit('\n'.join(summary_lines), title='PTAB Overlay'))

    override_table = Table(title='Partition Overrides', show_header=True, header_style='bold magenta')
    override_table.add_column('Layer', style='cyan')
    override_table.add_column('Partition', style='green')
    override_table.add_column('Mode', style='yellow')
    override_table.add_column('Fields', style='white')
    override_table.add_column('Source', style='dim')

    addition_table = Table(title='Partition Additions', show_header=True, header_style='bold magenta')
    addition_table.add_column('Layer', style='cyan')
    addition_table.add_column('Partition', style='green')
    addition_table.add_column('Mode', style='yellow')
    addition_table.add_column('Fields', style='white')
    addition_table.add_column('Source', style='dim')

    has_override = False
    has_addition = False
    for op in ops:
        fields = ', '.join(op.get('fields') or ['-'])
        row = [
            str(op.get('layer') or ''),
            str(op.get('name') or ''),
            str(op.get('mode') or ''),
            fields,
            str(op.get('source') or ''),
        ]
        if op.get('kind') == 'add':
            addition_table.add_row(*row)
            has_addition = True
        else:
            override_table.add_row(*row)
            has_override = True

    if has_override:
        console.print(override_table)
    if has_addition:
        console.print(addition_table)


def _warn_overlay_ignored_for_v2_base(
    base_ptab_path: Optional[str],
    overlay_paths: Dict[str, Optional[str]],
    dedupe: bool = True,
) -> str:
    """Warn once when v3 overlay is ignored for a v1/v2 PTAB base."""
    overlay_paths = overlay_paths or {}
    cache_key = (
        str(os.path.abspath(base_ptab_path) if base_ptab_path else ''),
        str(overlay_paths.get('chip') or ''),
        str(overlay_paths.get('board') or ''),
    )
    message = (
        "Ignoring project PTAB overlay because board base PTAB is v1/v2 and no project "
        "ptab.yaml was found; continuing without overlay."
    )
    details = []
    if base_ptab_path:
        details.append(f"base={os.path.abspath(base_ptab_path)}")
    if overlay_paths.get('chip'):
        details.append(f"chip_overlay={overlay_paths.get('chip')}")
    if overlay_paths.get('board'):
        details.append(f"board_overlay={overlay_paths.get('board')}")
    if details:
        message = f"{message} ({', '.join(details)})"

    if dedupe and cache_key in _OVERLAY_IGNORE_WARNING_CACHE:
        return message

    _OVERLAY_IGNORE_WARNING_CACHE.add(cache_key)
    logging.warning(message)
    return message


def prepare_project_ptab(
    project_root: str,
    board: Optional[str],
    chip: Optional[str],
    board_path: Optional[str] = None,
    output_dir: Optional[str] = None,
    emit_summary: bool = False,
    validate: bool = False,
    strict_validation: bool = False,
    summary_dedupe: bool = True,
) -> Dict[str, Any]:
    """Resolve the PTAB file used by a project and apply overlays if needed."""
    resolved = resolve_project_ptab_paths(project_root, board, chip, board_path=board_path)
    project_full_ptab = resolved.get('project_full_ptab')
    project_yaml_ptab = resolved.get('project_yaml_ptab')
    board_base_ptab = resolved.get('board_base_ptab')
    overlay_paths = resolved.get('overlay_paths') or {}
    uses_overlay = bool(overlay_paths.get('chip') or overlay_paths.get('board'))
    ignored_overlay = False
    ignored_overlay_reason = None
    board_base_ptab_obj = None

    if uses_overlay:
        if not board_base_ptab:
            raise ValueError("Board-level ptab.yaml not found, cannot apply overlay")
        board_base_ptab_obj = load_ptab(board_base_ptab, fatal=validate)
        if board_base_ptab_obj and not board_base_ptab_obj.is_v3() and not project_yaml_ptab:
            ignored_overlay = True
            ignored_overlay_reason = _warn_overlay_ignored_for_v2_base(
                board_base_ptab,
                overlay_paths,
                dedupe=summary_dedupe,
            )
            uses_overlay = False

    if project_full_ptab and uses_overlay:
        raise ValueError(
            "Project-level ptab.yaml/ptab.json and ptab.overlay.yaml cannot be used together"
        )

    if project_full_ptab:
        ptab_obj = load_ptab(project_full_ptab, fatal=validate)
        validation = []
        if validate and ptab_obj and ptab_obj.is_v3():
            validation = _validate_effective_ptab_v3(
                ptab_obj,
                strict=strict_validation,
                context=project_full_ptab,
            )
        return {
            'path': project_full_ptab,
            'effective_path': project_full_ptab,
            'base_path': project_full_ptab,
            'project_root': resolved.get('project_root'),
            'board': resolved.get('board'),
            'chip': resolved.get('chip'),
            'uses_overlay': False,
            'ignored_overlay': ignored_overlay,
            'ignored_overlay_reason': ignored_overlay_reason,
            'overlay_paths': overlay_paths,
            'report': {
                'base_path': project_full_ptab,
                'overlay_paths': overlay_paths,
                'effective_path': project_full_ptab,
                'operations': [],
                'validation': validation,
                'ignored_overlay': ignored_overlay,
                'ignored_overlay_reason': ignored_overlay_reason,
            },
            'merged_data': None,
            'ptab_obj': ptab_obj,
        }

    if uses_overlay:
        if not board_base_ptab:
            raise ValueError("Board-level ptab.yaml not found, cannot apply overlay")
        merged_data, report = build_effective_ptab_v3(
            board_base_ptab,
            chip_overlay_path=overlay_paths.get('chip'),
            board_overlay_path=overlay_paths.get('board'),
            strict_validation=strict_validation if validate else False,
        )
        effective_path = board_base_ptab
        if output_dir:
            effective_path = write_effective_ptab_yaml(
                os.path.join(os.path.abspath(output_dir), 'ptab.effective.yaml'),
                merged_data,
            )
        report['effective_path'] = effective_path
        if emit_summary:
            print_overlay_summary(report, dedupe=summary_dedupe)
        return {
            'path': effective_path,
            'effective_path': effective_path,
            'base_path': board_base_ptab,
            'project_root': resolved.get('project_root'),
            'board': resolved.get('board'),
            'chip': resolved.get('chip'),
            'uses_overlay': True,
            'ignored_overlay': False,
            'ignored_overlay_reason': None,
            'overlay_paths': overlay_paths,
            'report': report,
            'merged_data': merged_data,
            'ptab_obj': PtabV3(effective_path, merged_data),
        }

    path = board_base_ptab or resolved.get('board_fallback_ptab')
    if path and board_base_ptab_obj and path == board_base_ptab:
        ptab_obj = board_base_ptab_obj
    else:
        ptab_obj = load_ptab(path, fatal=validate) if path else None
    validation = []
    if validate and ptab_obj and ptab_obj.is_v3():
        validation = _validate_effective_ptab_v3(
            ptab_obj,
            strict=strict_validation,
            context=path,
        )
    return {
        'path': path,
        'effective_path': path,
        'base_path': path,
        'project_root': resolved.get('project_root'),
        'board': resolved.get('board'),
        'chip': resolved.get('chip'),
        'uses_overlay': False,
        'ignored_overlay': ignored_overlay,
        'ignored_overlay_reason': ignored_overlay_reason,
        'overlay_paths': overlay_paths,
        'report': {
            'base_path': path,
            'overlay_paths': overlay_paths,
            'effective_path': path,
            'operations': [],
            'validation': validation,
            'ignored_overlay': ignored_overlay,
            'ignored_overlay_reason': ignored_overlay_reason,
        },
        'merged_data': None,
        'ptab_obj': ptab_obj,
    }


def load_ptab(path, fatal=False):
    """加载 ptab 文件

    支持格式：
    - v1/v2: JSON 格式的 ptab.json
    - v3: YAML 格式的 ptab.yaml

    Args:
        path: ptab 文件路径
        fatal: 解析错误时是否终止程序

    Returns:
        Ptab 或 PtabV3 对象
    """
    abspath = os.path.abspath(str(path))

    try:
        stat = os.stat(abspath)
        key = (abspath, stat.st_mtime_ns, stat.st_size)
        if key in _PTAB_CACHE:
            return _PTAB_CACHE[key]

        if _is_yaml_file(abspath):
            # YAML 格式
            data = _parse_ptab_yaml_file(abspath)
            if _is_v3_yaml(data):
                ptab_obj = PtabV3(abspath, data)
            else:
                # YAML 但不是 v3，转换为 v1/v2 兼容格式
                if isinstance(data, list):
                    ptab_obj = Ptab(abspath, data)
                else:
                    raise ValueError("Invalid YAML ptab format: expected dict with 'partitions' or list")
        else:
            # JSON 格式 (v1/v2)
            mems = _parse_ptab_file(abspath)
            ptab_obj = Ptab(abspath, mems)

    except (ValueError, yaml.YAMLError) as e:
        if fatal:
            if _is_yaml_file(abspath):
                print("ptab.yaml syntax error")
            else:
                print("ptab.json syntax error, might be caused by trailing comma of last item")
            print("Error message: {}".format(e))
            print("Please check file {}".format(abspath))
            raise SystemExit(1)
        raise

    _PTAB_CACHE[key] = ptab_obj
    return ptab_obj


def _get_depend(name):
    import building
    return building.GetDepend(name)


def _get_config_value(name):
    import building
    return building.GetConfigValue(name)

def convert_to_cbus_addr(addr, offset, core=None):
    if _get_depend("SOC_SF32LB55X"):
        return addr, offset
    if _get_depend("SOC_SF32LB56X"):
        if (addr >= 0x60000000) and (addr <= 0x6FFFFFFF):
            return addr - 0x50000000, offset
        return addr, offset
    if _get_depend("SOC_SF32LB58X"):
        cbus_addr = addr
        cbus_offset = offset
        if (addr >= 0x60000000) and (addr <= 0x6FFFFFFF):
            cbus_addr -= 0x50000000
        elif (addr >= 0x20000000) and (addr <= 0x2FFFFFFF) and core and core.lower() == "acpu":
            cbus_addr -= 0x20200000
            assert cbus_addr >= 0, "0x{:8X} is not a valid address for ACPU"
            cbus_offset -= 0x00200000
        return cbus_addr, cbus_offset
    if _get_depend("SOC_SF32LB52X"):
        if (addr >= 0x60000000) and (addr <= 0x6FFFFFFF):
            return addr - 0x50000000, offset
        return addr, offset
    if _get_depend("SOC_SF32LB57X"):
        if (addr >= 0x60000000) and (addr <= 0x6FFFFFFF):
            if (core is None) or (core.lower() != "acpu"):
                return addr - 0x50000000, offset
        elif (addr >= 0x10000000) and (addr <= 0x1FFFFFFF) and core and (core.lower() == "acpu"):
            # ACPU use sbus address
            return addr + 0x50000000, offset
        return addr, offset
    raise Exception("unknown chip")


def convert_to_sbus_addr(addr, offset, core=None):
    if _get_depend("SOC_SF32LB55X"):
        return addr, offset
    if _get_depend("SOC_SF32LB56X") or _get_depend("SOC_SF32LB58X"):
        if (addr >= 0x10000000) and (addr < 0x1C000000):
            return addr + 0x50000000, offset
        return addr, offset
    if _get_depend("SOC_SF32LB52X"):
        if (addr >= 0x10000000) and (addr < 0x14000000):
            return addr + 0x50000000, offset
        return addr, offset
    if _get_depend("SOC_SF32LB57X"):
        if (addr >= 0x10000000) and (addr < 0x1FFFFFFF):
            return addr + 0x50000000, offset
        return addr, offset
    raise Exception("unknown chip")


def add_default_regions(mems):
    if _get_depend("SOC_SF32LB55X"):
        _add_default_regions_55x(mems)
    elif _get_depend("SOC_SF32LB56X"):
        _add_default_regions_56x(mems)
    elif _get_depend("SOC_SF32LB58X"):
        _add_default_regions_58x(mems)
    elif _get_depend("SOC_SF32LB52X"):
        _add_default_regions_52x(mems)
    elif _get_depend("SOC_SF32LB57X"):
        _add_default_regions_57x(mems)
    else:
        raise Exception("unknown chip")


def _add_default_regions_55x(mems):
    ftab_found = False
    flash1_mem = None
    for mem in mems:
        if 'flash1' == mem['mem']:
            flash1_mem = mem
        if "regions" not in mem:
            continue
        for region in mem['regions']:
            if "name" in region and 'ftab' == region['name']:
                ftab_found = True
        if ftab_found:
            break

    if not ftab_found:
        if not flash1_mem:
            flash1_mem = {
                "mem": "flash1",
                "base": "0x10000000",
                "regions": []
            }
            mems.insert(0, flash1_mem)

        if 'regions' not in flash1_mem:
            flash1_mem['regions'] = []

        ftab_region = {
            "offset": "0x00000000",
            "max_size": "0x00005000",
            "tags": [
                "FLASH_TABLE"
            ],
            "name": "ftab",
            "type": ["app_img", "app_exec"]
        }
        flash1_mem['regions'].insert(0, ftab_region)


def _add_default_regions_56x(mems):
    ftab_found = False
    bootloader_found = False
    flash5_mem = None
    for mem in mems:
        if 'flash5' == mem['mem']:
            flash5_mem = mem
        if "regions" not in mem:
            continue
        for region in mem['regions']:
            if "name" in region and 'ftab' == region['name']:
                ftab_found = True
            if "name" in region and 'bootloader' == region['name']:
                bootloader_found = True

        if ftab_found and bootloader_found:
            break

    if (not ftab_found) or (not bootloader_found):
        if not flash5_mem:
            flash5_mem = {
                "mem": "flash5",
                "base": "0x1C000000",
                "regions": []
            }
            mems.insert(0, flash5_mem)

        if 'regions' not in flash5_mem:
            flash5_mem['regions'] = []

        if not ftab_found:
            ftab_region = {
                "offset": "0x00000000",
                "max_size": "0x00004000",
                "tags": [
                    "FLASH_TABLE"
                ],
                "name": "ftab",
                "type": ["app_img", "app_exec"]
            }
            flash5_mem['regions'].insert(0, ftab_region)

        if not bootloader_found:
            bootloader_region = {
                "offset": "0x00020000",
                "max_size": "0x0000C000",
                "tags": [
                    "FLASH_BOOT_LOADER"
                ],
                "name": "bootloader",
                "type": ["app_img", "app_exec"]
            }
            flash5_mem['regions'].insert(0, bootloader_region)


def _add_default_regions_58x(mems):
    ftab_found = False
    bootloader_found = False
    flash5_mem = None
    for mem in mems:
        if 'flash5' == mem['mem']:
            flash5_mem = mem
        if "regions" not in mem:
            continue
        for region in mem['regions']:
            if "name" in region and 'ftab' == region['name']:
                ftab_found = True
            if "name" in region and 'bootloader' == region['name']:
                bootloader_found = True

        if ftab_found and bootloader_found:
            break

    if (not ftab_found) or (not bootloader_found):
        if not flash5_mem:
            flash5_mem = {
                "mem": "flash5",
                "base": "0x1C000000",
                "regions": []
            }
            mems.insert(0, flash5_mem)

        if 'regions' not in flash5_mem:
            flash5_mem['regions'] = []

        if not ftab_found:
            ftab_region = {
                "offset": "0x00000000",
                "max_size": "0x00005000",
                "tags": [
                    "FLASH_TABLE"
                ],
                "name": "ftab",
                "type": ["app_img", "app_exec"]
            }
            flash5_mem['regions'].insert(0, ftab_region)

        if not bootloader_found:
            bootloader_region = {
                "offset": "0x00020000",
                "max_size": "0x00020000",
                "tags": [
                    "FLASH_BOOT_LOADER"
                ],
                "name": "bootloader",
                "type": ["app_img", "app_exec"]
            }
            flash5_mem['regions'].insert(0, bootloader_region)


def _add_default_regions_52x(mems):
    ftab_found = False
    bootloader_exec_found = False
    bootloader_img_found = False
    bootloader_data_found = False
    boot_mem = None
    hpsys_ram_mem = None
    boot_dev_type = None

    for mem in mems:
        # guess boot_dev_type and boot_mem by memory name and address
        if "flash1" == mem['mem']:
            boot_mem = mem
            boot_dev_type = "nor"
        elif "flash2" == mem['mem']:
            boot_mem = mem
            if "0x12000000" == mem['base']:
                boot_dev_type = "nor"
            else:
                boot_dev_type = "nand"
        elif "sd1" == mem['mem']:
            boot_mem = mem
            boot_dev_type = 'sd'

        if "hpsys_ram" == mem['mem']:
            hpsys_ram_mem = mem
            continue

        for region in mem['regions']:
            if "name" in region and 'ftab' == region['name']:
                ftab_found = True
            if "name" in region and 'bootloader' == region['name']:
                bootloader_img_found = True

    if hpsys_ram_mem and isinstance(hpsys_ram_mem.get('regions'), list):
        for region in hpsys_ram_mem['regions']:
            if not isinstance(region, dict):
                continue

            region_name = region.get('name')
            region_type = region.get('type', []) or []
            region_tags = region.get('tags', []) or []

            if region_name == 'bootloader' and 'app_exec' in region_type:
                bootloader_exec_found = True
            elif 'FLASH_BOOT_LOADER' in region_tags and 'app_exec' in region_type:
                bootloader_exec_found = True

            if region_name == 'bootloader' and 'app_exec' not in region_type:
                bootloader_data_found = True
            elif 'BOOTLOADER_RAM_DATA' in region_tags:
                bootloader_data_found = True

    if not bootloader_exec_found:
        bootloader_region = {
            "offset": "0x00020000",
            "max_size": "0x00010000",
            "name": "bootloader",
            "type": ["app_exec"],
            "tags": ["FLASH_BOOT_LOADER"]
        }
        hpsys_ram_mem["regions"].insert(0, bootloader_region)

    if not bootloader_data_found:
        bootloader_region = {
            "offset": "0x00040000",
            "max_size": "0x00010000",
            "tags": ["BOOTLOADER_RAM_DATA"]
        }
        hpsys_ram_mem['regions'].insert(0, bootloader_region)

    if (not ftab_found) or (not bootloader_img_found):
        if not ftab_found:
            if "sd" == boot_dev_type:
                # MBR uses first 4096 bytes
                ftab_region = {
                    "offset": "0x00001000",
                    "max_size": "0x00008000",
                    "tags": ["FLASH_TABLE"],
                    "name": "ftab",
                    "type": ["app_img", "app_exec"]
                }
            else:
                ftab_region = {
                    "offset": "0x00000000",
                    "max_size": "0x00008000",
                    "tags": ["FLASH_TABLE"],
                    "name": "ftab",
                    "type": ["app_img", "app_exec"]
                }

            boot_mem['regions'].insert(0, ftab_region)

        if not bootloader_img_found:
            if "sd" == boot_dev_type:
                bootloader_region = {
                    "offset": "0x00011000",
                    "max_size": "0x00010000",
                    "tags": [],
                    "name": "bootloader",
                    "type": ["app_img"]
                }
            elif "nor" == boot_dev_type:
                bootloader_region = {
                    "offset": "0x00010000",
                    "max_size": "0x00010000",
                    "tags": [],
                    "name": "bootloader",
                    "type": ["app_img"]
                }
            elif "nand" == boot_dev_type:
                if _get_config_value("FLASH_CONFIG_PAGE_SIZE") != '' and _get_config_value("FLASH_CONFIG_BLOCK_SIZE") != '':
                    block_size = int(_get_config_value("FLASH_CONFIG_BLOCK_SIZE"))
                else:
                    block_size = 128*1024
                bootloader_region = {
                    "offset": "0x{:08X}".format(4 * block_size), # block0: ftab, block1: ftab_bak, block2: calibration data, block3: reserved
                    "max_size": "0x00010000",
                    "tags": [],
                    "name": "bootloader",
                    "type": ["app_img"]
                }
            else:
                raise Exception(f"unknown type {boot_dev_type}")
            boot_mem['regions'].insert(0, bootloader_region)

def _add_default_regions_57x(mems):
    ftab_found = False
    bootloader_exec_found = False
    bootloader_img_found = False
    bootloader_data_found = False
    boot_mem = None
    hpsys_ram_mem = None
    boot_dev_type = None

    for mem in mems:
        # guess boot_dev_type and boot_mem by memory name and address
        if "flash1" == mem['mem']:
            boot_mem = mem
            boot_dev_type = "nor"
        elif "flash2" == mem['mem']:
            boot_mem = mem
            boot_dev_type = "nor"
        elif "flash3" == mem['mem']:
            boot_mem = mem
            if "0x14000000" == mem['base']:
                boot_dev_type = "nor"
            else:
                boot_dev_type = "nand"
        elif "sd1" == mem['mem']:
            boot_mem = mem
            boot_dev_type = 'sd'

        if "hpsys_ram" == mem['mem']:
            hpsys_ram_mem = mem
            continue

        for region in mem['regions']:
            if "name" in region and 'ftab' == region['name']:
                ftab_found = True
            if "name" in region and 'bootloader' == region['name']:
                bootloader_img_found = True

    if hpsys_ram_mem and isinstance(hpsys_ram_mem.get('regions'), list):
        for region in hpsys_ram_mem['regions']:
            if not isinstance(region, dict):
                continue

            region_name = region.get('name')
            region_type = region.get('type', []) or []
            region_tags = region.get('tags', []) or []

            if region_name == 'bootloader' and 'app_exec' in region_type:
                bootloader_exec_found = True
            elif 'FLASH_BOOT_LOADER' in region_tags and 'app_exec' in region_type:
                bootloader_exec_found = True

            if region_name == 'bootloader' and 'app_exec' not in region_type:
                bootloader_data_found = True
            elif 'BOOTLOADER_RAM_DATA' in region_tags:
                bootloader_data_found = True

    if not bootloader_exec_found:
        bootloader_region = {
            "offset": "0x00020000",
            "max_size": "0x00010000",
            "name": "bootloader",
            "type": ["app_exec"],
            "tags": ["FLASH_BOOT_LOADER"]
        }
        hpsys_ram_mem["regions"].insert(0, bootloader_region)

    if not bootloader_data_found:
        bootloader_region = {
            "offset": "0x00040000",
            "max_size": "0x00010000",
            "tags": ["BOOTLOADER_RAM_DATA"]
        }
        hpsys_ram_mem['regions'].insert(0, bootloader_region)

    if (not ftab_found) or (not bootloader_img_found):
        if not ftab_found:
            if "sd" == boot_dev_type:
                # MBR uses first 4096 bytes
                ftab_region = {
                    "offset": "0x00001000",
                    "max_size": "0x00008000",
                    "tags": ["FLASH_TABLE"],
                    "name": "ftab",
                    "type": ["app_img", "app_exec"]
                }
            else:
                ftab_region = {
                    "offset": "0x00000000",
                    "max_size": "0x00008000",
                    "tags": ["FLASH_TABLE"],
                    "name": "ftab",
                    "type": ["app_img", "app_exec"]
                }

            boot_mem['regions'].insert(0, ftab_region)

        if not bootloader_img_found:
            if "sd" == boot_dev_type:
                bootloader_region = {
                    "offset": "0x00011000",
                    "max_size": "0x00010000",
                    "tags": [],
                    "name": "bootloader",
                    "type": ["app_img"]
                }
            elif "nor" == boot_dev_type:
                bootloader_region = {
                    "offset": "0x00010000",
                    "max_size": "0x00010000",
                    "tags": [],
                    "name": "bootloader",
                    "type": ["app_img"]
                }
            elif "nand" == boot_dev_type:
                if _get_config_value("FLASH_CONFIG_PAGE_SIZE") != '' and _get_config_value("FLASH_CONFIG_BLOCK_SIZE") != '':
                    block_size = int(_get_config_value("FLASH_CONFIG_BLOCK_SIZE"))
                else:
                    block_size = 128*1024
                bootloader_region = {
                    "offset": "0x{:08X}".format(4 * block_size), # block0: ftab, block1: ftab_bak, block2: calibration data, block3: reserved
                    "max_size": "0x00010000",
                    "tags": [],
                    "name": "bootloader",
                    "type": ["app_img"]
                }
            else:
                raise Exception(f"unknown type {boot_dev_type}")
            boot_mem['regions'].insert(0, bootloader_region)


def _get_ptab_path(env):
    return env.get('PARTITION_TABLE')


def get_ptab(env):
    path = _get_ptab_path(env)
    if not path:
        return None
    return load_ptab(path, fatal=True)


def _ptab_header_build(target, source, env):
    import sdk_resource
    src_file = str(source[0])
    target_file = str(target[0])
    output_dir = os.path.dirname(target_file)
    output_name = os.path.splitext(os.path.basename(target_file))[0]
    sdk_resource.GenPartitionTableHeaderFile(src_file, output_dir, output_name, env=env)


def generate(env):
    from SCons.Script import Builder
    from SCons.Action import Action

    env.AddMethod(get_ptab, "GetPtab")
    env.AddMethod(_get_ptab_path, "GetPtabPath")

    action = Action(_ptab_header_build, 'Generating $TARGET ...')
    bld = Builder(action=action)
    env.Append(BUILDERS={"PtabHeader": bld})


def exists(env):
    return True
