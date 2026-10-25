#!/usr/bin/env python3
# Copyright (c) 2025 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

"""
Validate ptab v3 format and constraints.

Usage:
    python validate_ptab_v3.py ptab.yaml
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

# Add tools/build to path
_TOOLS_BUILD_PATH = Path(__file__).parent
if str(_TOOLS_BUILD_PATH) not in sys.path:
    sys.path.insert(0, str(_TOOLS_BUILD_PATH))

import ptab as ptab_module

# Extra ACPU code images use the '.rom<N>' / 'ER_IROM<N>' family shared with the v1/v2
# multi-image flow, which probes '.rom2'..'.rom6' (building.MAX_EX_IMG_NUM = 5).
_ACPU_CODE_ROM_INDEX_MAX = 6


class ValidationError:
    def __init__(self, message: str, severity: str = "error"):
        self.message = message
        self.severity = severity  # "error" or "warning"

    def __str__(self):
        return f"[{self.severity.upper()}] {self.message}"


def _is_auto_fal_partition(partition: Dict[str, Any]) -> bool:
    ptype = str(partition.get('type') or '').strip()
    subtype = str(partition.get('subtype') or '').strip()
    if ptype == 'data':
        return subtype in ('flashdb_kv', 'filesystem')
    if ptype == 'app':
        return subtype != 'ex'
    return False


def _auto_fal_region_supported(region: str) -> bool:
    region = str(region or '').strip().lower()
    if re.match(r'^mpi[1-5]$', region):
        return True
    return region in ('sdmmc1', 'sdmmc2')


def _region_memory_type(region: str, chip_config: dict) -> str:
    try:
        return str(ptab_module._get_region_memory_type(region, chip_config) or '').strip().lower()
    except Exception:
        return ''


def _memory_region(memory: Dict[str, Any]) -> str:
    region = memory.get('mpi') or memory.get('sdmmc')
    if not region:
        name = memory.get('name')
        if isinstance(name, str):
            name = name.strip()
            if name.startswith('mpi') or name.startswith('sdmmc'):
                region = name
    return str(region or '').strip()


def _nand_block_info(region: str, chip_config: dict) -> Tuple[int, bool]:
    memory_info = chip_config.get('memory_info', {})
    info = memory_info.get(region, {}) if isinstance(memory_info, dict) else {}
    if isinstance(info, dict):
        try:
            block_size = ptab_module.parse_size(info.get('block_size', ptab_module.NAND_BLOCK_SIZE_DEFAULT))
        except (ValueError, TypeError):
            block_size = ptab_module.NAND_BLOCK_SIZE_DEFAULT
        explicit = bool(info.get('block_size_explicit'))
        return block_size, explicit
    return ptab_module.NAND_BLOCK_SIZE_DEFAULT, False


def _partition_tags(partition: Dict[str, Any]) -> set[str]:
    tags = set()
    name = str(partition.get('name') or '').strip()
    if name:
        tags.add(name.upper())
    for alias in partition.get('aliases') or []:
        alias = str(alias or '').strip()
        if alias:
            tags.add(alias.upper())
    return tags


def _format_layouts(layouts: List[Tuple[int, int]]) -> str:
    return ', '.join(f'offset=0x{offset:X}, size=0x{size:X}' for offset, size in layouts)


def _validate_reserved_partition(
    partitions: List[dict],
    region: str,
    tag: str,
    layouts: List[Tuple[int, int]],
) -> List[ValidationError]:
    errors: List[ValidationError] = []
    candidates = [
        p for p in partitions
        if str(p.get('region') or '').strip() == region and tag in _partition_tags(p)
    ]

    if not candidates:
        errors.append(ValidationError(
            f"SF32LB52 region '{region}' must define reserved partition '{tag}' as type='data', subtype='raw' "
            f"with {_format_layouts(layouts)}"
        ))
        return errors

    raw_candidates = [
        p for p in candidates
        if p.get('type') == 'data' and p.get('subtype') == 'raw'
    ]
    if not raw_candidates:
        names = ', '.join(str(p.get('name') or '?') for p in candidates)
        errors.append(ValidationError(
            f"SF32LB52 reserved partition '{tag}' in region '{region}' must use type='data', subtype='raw' "
            f"(found: {names})"
        ))
        return errors

    for partition in raw_candidates:
        try:
            offset = ptab_module.parse_size(partition.get('offset', 0))
            size = ptab_module.parse_size(partition.get('size', 0))
        except (ValueError, TypeError):
            continue
        if (offset, size) in layouts:
            return errors

    actual = []
    for partition in raw_candidates:
        try:
            offset = ptab_module.parse_size(partition.get('offset', 0))
            size = ptab_module.parse_size(partition.get('size', 0))
            actual.append(f"{partition.get('name', '?')}: offset=0x{offset:X}, size=0x{size:X}")
        except (ValueError, TypeError):
            actual.append(f"{partition.get('name', '?')}: invalid offset/size")
    errors.append(ValidationError(
        f"SF32LB52 reserved partition '{tag}' in region '{region}' has invalid placement; "
        f"expected {_format_layouts(layouts)}; found {', '.join(actual)}"
    ))
    return errors


def validate_nand_memory_block_sizes(ptab_obj) -> List[ValidationError]:
    errors: List[ValidationError] = []
    for idx, memory in enumerate(getattr(ptab_obj, 'memory', []) or []):
        if not isinstance(memory, dict) or 'block_size' not in memory:
            continue

        region = _memory_region(memory) or f'memory[{idx}]'
        mem_type = str(memory.get('type') or '').strip().lower()
        if mem_type != 'nand':
            errors.append(ValidationError(
                f"Memory '{region}': block_size is only supported for NAND memory"
            ))
            continue

        try:
            block_size = ptab_module.parse_size(memory.get('block_size', 0))
        except (ValueError, TypeError) as e:
            errors.append(ValidationError(
                f"Memory '{region}': invalid block_size value '{memory.get('block_size')}': {e}"
            ))
            continue

        if block_size not in ptab_module.NAND_BLOCK_SIZE_CHOICES:
            errors.append(ValidationError(
                f"Memory '{region}': NAND block_size must be 0x20000 or 0x40000"
            ))

    return errors


def validate_nand_partition_alignment(partitions: List[dict], chip_config: dict) -> List[ValidationError]:
    errors: List[ValidationError] = []
    for partition in partitions:
        region = str(partition.get('region') or '').strip()
        if not region or _region_memory_type(region, chip_config) != 'nand':
            continue

        block_size, explicit = _nand_block_info(region, chip_config)
        if not explicit:
            continue
        if block_size not in ptab_module.NAND_BLOCK_SIZE_CHOICES:
            continue

        try:
            offset = ptab_module.parse_size(partition.get('offset', 0))
        except (ValueError, TypeError):
            continue

        if offset % block_size != 0:
            errors.append(ValidationError(
                f"Partition '{partition.get('name', '?')}' in NAND region '{region}' has offset 0x{offset:X}, "
                f"which is not aligned to block_size 0x{block_size:X}"
            ))

    return errors


def _find_partition_in_region(
    partitions: List[dict],
    region: str,
    *,
    name: Optional[str] = None,
    ptype: Optional[str] = None,
) -> Optional[dict]:
    for partition in partitions:
        if str(partition.get('region') or '').strip() != region:
            continue
        if name is not None and str(partition.get('name') or '').strip() != name:
            continue
        if ptype is not None and str(partition.get('type') or '').strip() != ptype:
            continue
        return partition
    return None


def validate_name(name: str, partition_idx: int) -> List[ValidationError]:
    """Validate partition name format"""
    errors = []

    if not name:
        errors.append(ValidationError(
            f"Partition {partition_idx}: 'name' is required"
        ))
        return errors

    # Must be lowercase letters, underscores, digits
    if not re.match(r'^[a-z][a-z0-9_]*$', name):
        if name[0].isdigit():
            errors.append(ValidationError(
                f"Partition '{name}': name cannot start with a digit"
            ))
        elif not name.islower() or not re.match(r'^[a-z0-9_]+$', name):
            errors.append(ValidationError(
                f"Partition '{name}': name must contain only lowercase letters, digits, and underscores"
            ))

    return errors


def validate_type(ptype: str, partition_name: str) -> List[ValidationError]:
    """Validate partition type"""
    errors = []
    valid_types = {'ftab', 'bootloader', 'app', 'data'}

    if not ptype:
        errors.append(ValidationError(
            f"Partition '{partition_name}': 'type' is required"
        ))
    elif ptype not in valid_types:
        errors.append(ValidationError(
            f"Partition '{partition_name}': unknown type '{ptype}', "
            f"valid types are: {', '.join(sorted(valid_types))}"
        ))

    return errors


def validate_region(region: str, partition_name: str, chip_config: dict) -> List[ValidationError]:
    """Validate region exists in chip config"""
    errors = []

    if not region:
        errors.append(ValidationError(
            f"Partition '{partition_name}': 'region' is required"
        ))
        return errors

    # Check if region exists
    mpi_config = chip_config.get('mpi', {})
    sdmmc_config = chip_config.get('sdmmc', {})
    ram_config = chip_config.get('ram', {})

    known_regions = set(mpi_config.keys())
    known_regions.update(sdmmc_config.keys())
    known_regions.update(['hpsys_ram', 'lpsys_ram', 'psram', 'psram1', 'psram2'])

    if region not in known_regions and not region.startswith('mpi') and not region.startswith('psram') and not region.startswith('sdmmc'):
        errors.append(ValidationError(
            f"Partition '{partition_name}': unknown region '{region}'",
            severity="warning"
        ))

    return errors


def validate_size(value, field_name: str, partition_name: str) -> List[ValidationError]:
    """Validate size/offset value"""
    errors = []

    try:
        parsed = ptab_module.parse_size(value)
        if parsed < 0:
            errors.append(ValidationError(
                f"Partition '{partition_name}': {field_name} cannot be negative"
            ))
    except (ValueError, TypeError) as e:
        errors.append(ValidationError(
            f"Partition '{partition_name}': invalid {field_name} value '{value}': {e}"
        ))

    return errors


def validate_exec(exec_def, partition_name: str, chip_config: dict, core: Optional[str]) -> List[ValidationError]:
    """Validate exec structure and address parsing."""
    errors = []

    if exec_def is None:
        return errors

    if not isinstance(exec_def, dict):
        errors.append(ValidationError(
            f"Partition '{partition_name}': 'exec' must be a dict with keys {{region, offset}}"
        ))
        return errors

    exec_region = exec_def.get('region', '')
    exec_offset = exec_def.get('offset', 0)

    if not exec_region:
        errors.append(ValidationError(
            f"Partition '{partition_name}': exec.region is required when exec is present"
        ))
        return errors

    errors.extend(validate_region(str(exec_region), f"{partition_name}.exec", chip_config))
    errors.extend(validate_size(exec_offset, 'exec.offset', partition_name))

    # Try resolving address (this may still succeed for unknown region but is useful for early errors)
    try:
        ptab_module.resolve_region_address(str(exec_region), ptab_module.parse_size(exec_offset), chip_config, core=core)
    except Exception as e:
        errors.append(ValidationError(
            f"Partition '{partition_name}': failed to resolve exec address: {e}"
        ))

    return errors


def validate_sections(sections: Any, partition: dict) -> List[ValidationError]:
    """Validate structured selectors used by resource and ACPU code partitions."""
    errors = []
    pname = partition.get('name', '?')
    ptype = str(partition.get('type') or '').strip().lower()
    psubtype = str(partition.get('subtype') or '').strip().lower()
    # 'sections' describes the ELF content of a partition: a resource partition
    # (app/ex) gathers input sections into an output section named after the
    # partition, while an extra ACPU code image (app/factory) names its own output
    # section '.rom<N>' so that it maps to ER_IROM<N>.bin.
    is_app_ex = ptype == 'app' and psubtype == 'ex'
    is_acpu_code = ptab_module.is_acpu_code_partition_v3(partition)

    if sections is None:
        return errors

    if not (is_app_ex or (is_acpu_code and ptab_module.get_acpu_code_rom_indexes(partition))):
        errors.append(ValidationError(
            f"Partition '{pname}': 'sections' is only supported for type='app' subtype='ex' "
            f"or for an ACPU code image naming its '.rom<N>' output section"
        ))
        return errors

    if not isinstance(sections, list):
        errors.append(ValidationError(
            f"Partition '{pname}': 'sections' must be a list"
        ))
        return errors

    seen = set()
    for idx, item in enumerate(sections):
        if not isinstance(item, dict):
            errors.append(ValidationError(
                f"Partition '{pname}': sections[{idx}] must be a mapping with keys {{section, object?}}"
            ))
            continue

        extra_keys = sorted(set(item.keys()) - {'section', 'object'})
        if extra_keys:
            errors.append(ValidationError(
                f"Partition '{pname}': sections[{idx}] contains unsupported key(s): {', '.join(extra_keys)}"
            ))

        section = str(item.get('section') or '').strip()
        if not section:
            errors.append(ValidationError(
                f"Partition '{pname}': sections[{idx}].section is required"
            ))
            continue
        if not section.startswith('.'):
            errors.append(ValidationError(
                f"Partition '{pname}': sections[{idx}].section must start with '.'"
            ))

        obj = item.get('object')
        obj_norm = ''
        if obj is not None:
            obj_norm = ptab_module._normalize_selector_object_name(obj)
            if not obj_norm:
                errors.append(ValidationError(
                    f"Partition '{pname}': sections[{idx}].object cannot be empty"
                ))
            elif any(ch.isspace() for ch in obj_norm):
                errors.append(ValidationError(
                    f"Partition '{pname}': sections[{idx}].object cannot contain whitespace"
                ))

        key = (obj_norm, section)
        if key in seen:
            errors.append(ValidationError(
                f"Partition '{pname}': duplicate selector in sections[{idx}]"
            ))
        else:
            seen.add(key)

    return errors


def validate_bootloader_unique(partitions: List[dict]) -> List[ValidationError]:
    """Validate: exactly one bootloader partition"""
    errors = []

    bootloaders = [p for p in partitions if p.get('type') == 'bootloader']

    if len(bootloaders) == 0:
        errors.append(ValidationError(
            "No partition with type='bootloader' found"
        ))
    elif len(bootloaders) > 1:
        names = [p.get('name', '?') for p in bootloaders]
        errors.append(ValidationError(
            f"Multiple bootloader partitions found: {', '.join(names)}"
        ))

    return errors


def validate_ftab_unique(partitions: List[dict]) -> List[ValidationError]:
    """Validate: exactly one ftab partition"""
    errors = []
    ftabs = [p for p in partitions if p.get('type') == 'ftab']

    if len(ftabs) == 0:
        errors.append(ValidationError("No partition with type='ftab' found"))
    elif len(ftabs) > 1:
        names = [p.get('name', '?') for p in ftabs]
        errors.append(ValidationError(f"Multiple ftab partitions found: {', '.join(names)}"))

    return errors


def validate_factory_unique_by_core(partitions: List[dict]) -> List[ValidationError]:
    """Validate: at most one factory app per core (ACPU may carry several images)."""
    errors = []
    seen: Dict[str, List[str]] = {}

    for p in partitions:
        if p.get('type') != 'app' or p.get('subtype') != 'factory':
            continue
        core = (p.get('core') or 'HCPU').upper()
        # One ACPU project may build several code images out of a single ELF, so the
        # ACPU core is allowed to own more than one factory partition; those extra
        # images are validated by validate_acpu_code_partitions().
        if core == 'ACPU':
            continue
        seen.setdefault(core, []).append(p.get('name', '?'))

    for core, names in seen.items():
        if len(names) > 1:
            errors.append(ValidationError(
                f"Multiple factory app partitions found for core {core}: {', '.join(names)}"
            ))

    return errors


def validate_acpu_code_partitions(partitions: List[dict]) -> List[ValidationError]:
    """Validate the ACPU code partitions, including extra multi-image partitions.

    One ACPU project may build several code images from a single ELF: the first
    image keeps its code in the project code artifact, while every further image is
    declared as its own app/factory/core=ACPU partition naming the ELF output
    section that holds its code ('.rom<N>', N >= 2) - which also gives the artifact
    suffix ER_IROM<N>.bin.
    """
    errors = []
    acpu_parts = [p for p in partitions if ptab_module.is_acpu_code_partition_v3(p)]
    if not acpu_parts:
        return errors

    indexes: Dict[int, str] = {}
    primary: List[str] = []
    for p in acpu_parts:
        pname = p.get('name', '?')
        if not isinstance(p.get('exec'), dict):
            errors.append(ValidationError(
                f"Partition '{pname}': ACPU code partition must provide 'exec: {{region, offset}}'"
            ))

        rom_indexes = ptab_module.get_acpu_code_rom_indexes(p)
        if len(rom_indexes) > 1:
            errors.append(ValidationError(
                "Partition '{}': at most one '.rom<N>' section is allowed (got {})".format(
                    pname, ', '.join('.rom{}'.format(n) for n in rom_indexes))
            ))

        # The first image owns region 1 and keeps its code in the code artifact.
        expected_region = 1
        if not rom_indexes:
            primary.append(pname)
        else:
            rom_index = rom_indexes[0]
            expected_region = rom_index
            if rom_index < 2 or rom_index > _ACPU_CODE_ROM_INDEX_MAX:
                errors.append(ValidationError(
                    f"Partition '{pname}': '.rom{rom_index}' is out of range "
                    f"(.rom2..rom{_ACPU_CODE_ROM_INDEX_MAX})"
                ))
            if rom_index in indexes:
                errors.append(ValidationError(
                    f"Partition '{pname}': '.rom{rom_index}' is already used by '{indexes[rom_index]}'"
                ))
            else:
                indexes[rom_index] = pname

        # The legacy macro alias and the ELF section must describe the same image.
        for alias in p.get('aliases') or []:
            match = re.match(r'^ACPU_CODE_LOAD_REGION(\d+)$', str(alias or '').strip().upper())
            if match and int(match.group(1)) != expected_region:
                errors.append(ValidationError(
                    f"Partition '{pname}': alias '{alias}' does not match image "
                    f"{expected_region}{' (.rom{})'.format(expected_region) if rom_indexes else ' (first image)'}"
                ))

    if len(primary) != 1:
        errors.append(ValidationError(
            "Exactly one ACPU code partition must stay without a '.rom<N>' section (the image "
            "whose code stays in the code artifact); found {}: {}".format(
                len(primary), ', '.join(primary) if primary else 'none')
        ))

    return errors


def validate_name_unique(partitions: List[dict]) -> List[ValidationError]:
    errors = []
    seen: Dict[str, int] = {}
    for idx, p in enumerate(partitions):
        name = p.get('name', '')
        if not name:
            continue
        if name in seen:
            errors.append(ValidationError(
                f"Duplicate partition name '{name}' found at indices {seen[name]} and {idx}"
            ))
        else:
            seen[name] = idx
    return errors


def validate_aliases_conflict(partitions: List[dict]) -> List[ValidationError]:
    errors = []
    owner: Dict[str, str] = {}

    for p in partitions:
        pname = p.get('name', '?')
        aliases = p.get('aliases', []) or []
        if not isinstance(aliases, list):
            continue
        for a in aliases:
            a_norm = str(a).strip().upper()
            if not a_norm:
                continue
            if a_norm in owner and owner[a_norm] != pname:
                errors.append(ValidationError(
                    f"Alias macro base '{a_norm}' is used by multiple partitions: {owner[a_norm]} and {pname}"
                ))
            else:
                owner[a_norm] = pname

    return errors


def validate_no_overlap(partitions: List[dict], chip_config: dict) -> List[ValidationError]:
    """Validate partitions don't overlap within same region"""
    errors = []

    # Group by region
    region_partitions = {}
    for p in partitions:
        region = p.get('region', '')
        # NOTE: internal RAM regions are frequently re-used across boot stages
        # (bootloader vs main) and may overlap by design. Keep overlap checks
        # strict for storage regions, but skip internal RAM.
        if region == 'hpsys_ram' or str(region).startswith('hpsys') or region == 'lpsys_ram' or str(region).startswith('lpsys'):
            continue
        if region not in region_partitions:
            region_partitions[region] = []

        try:
            offset = ptab_module.parse_size(p.get('offset', 0))
            size = ptab_module.parse_size(p.get('size', 0))
            region_partitions[region].append((
                p.get('name', '?'),
                offset,
                offset + size
            ))
        except (ValueError, TypeError):
            pass  # Already reported in validate_size

    # Check overlaps within each region
    for region, parts in region_partitions.items():
        parts.sort(key=lambda x: x[1])  # Sort by start offset

        for i in range(len(parts) - 1):
            name1, start1, end1 = parts[i]
            name2, start2, end2 = parts[i + 1]

            if end1 > start2:
                errors.append(ValidationError(
                    f"Partitions '{name1}' and '{name2}' overlap in region '{region}' "
                    f"(0x{start1:X}-0x{end1:X} vs 0x{start2:X}-0x{end2:X})"
                ))

    return errors


def validate_rom_index_deprecation(partitions: List[dict]) -> List[ValidationError]:
    """Warn about deprecated rom_index usage"""
    errors = []

    for p in partitions:
        if p.get('rom_index') is not None:
            errors.append(ValidationError(
                f"Partition '{p.get('name', '?')}': 'rom_index' is deprecated, "
                "please migrate to name-based section naming",
                severity="warning"
            ))

    return errors


def validate_sf32lb52_storage_reservations(ptab_obj, partitions: List[dict], chip_config: dict) -> List[ValidationError]:
    errors: List[ValidationError] = []
    if str(getattr(ptab_obj, 'chip_series', '') or '').strip().lower() != 'sf32lb52':
        return errors

    storage_regions: Dict[str, str] = {}
    for partition in partitions:
        region = str(partition.get('region') or '').strip()
        if not region:
            continue
        ptype = str(partition.get('type') or '').strip()
        subtype = str(partition.get('subtype') or '').strip()
        if ptype not in ('ftab', 'bootloader', 'app') and not (ptype == 'data' and subtype in ('filesystem', 'flashdb_kv')):
            continue

        mem_type = _region_memory_type(region, chip_config)
        if mem_type in ('nand', 'sd'):
            storage_regions.setdefault(region, mem_type)

    for region, mem_type in sorted(storage_regions.items()):
        if mem_type == 'nand':
            block_size, _explicit = _nand_block_info(region, chip_config)
            if block_size not in ptab_module.NAND_BLOCK_SIZE_CHOICES:
                continue
            errors.extend(_validate_reserved_partition(
                partitions,
                region,
                'FACTORY_DATA',
                [(2 * block_size, block_size)],
            ))
        elif mem_type == 'sd':
            errors.extend(_validate_reserved_partition(
                partitions,
                region,
                'MBR',
                [(0x00000000, 0x00001000)],
            ))
            errors.extend(_validate_reserved_partition(
                partitions,
                region,
                'FACTORY_DATA',
                [(0x00041000, 0x00020000)],
            ))

    return errors


def validate_sf32lb52_nand_boot_layout(ptab_obj, partitions: List[dict], chip_config: dict) -> List[ValidationError]:
    errors: List[ValidationError] = []
    if str(getattr(ptab_obj, 'chip_series', '') or '').strip().lower() != 'sf32lb52':
        return errors

    nand_regions = sorted({
        str(partition.get('region') or '').strip()
        for partition in partitions
        if str(partition.get('region') or '').strip()
        and _region_memory_type(str(partition.get('region') or '').strip(), chip_config) == 'nand'
    })

    for region in nand_regions:
        block_size, _explicit = _nand_block_info(region, chip_config)
        if block_size not in ptab_module.NAND_BLOCK_SIZE_CHOICES:
            continue

        flash_table = _find_partition_in_region(partitions, region, name='flash_table', ptype='ftab')
        if flash_table is not None:
            try:
                offset = ptab_module.parse_size(flash_table.get('offset', 0))
                size = ptab_module.parse_size(flash_table.get('size', 0))
            except (ValueError, TypeError):
                offset = None
                size = None
            # The flash table keeps the v1/v2 size (0x8000) regardless of the
            # NAND erase block size; the rest of the block stays reserved.
            expected_size = 0x00008000
            if offset != 0 or size != expected_size:
                errors.append(ValidationError(
                    f"SF32LB52 NAND region '{region}' flash_table must use offset=0x0, "
                    f"size=0x{expected_size:X}"
                ))

        bootloader = _find_partition_in_region(partitions, region, ptype='bootloader')
        if bootloader is not None:
            try:
                offset = ptab_module.parse_size(bootloader.get('offset', 0))
            except (ValueError, TypeError):
                offset = None
            expected_offset = 4 * block_size
            if offset != expected_offset:
                errors.append(ValidationError(
                    f"SF32LB52 NAND region '{region}' bootloader must use offset=0x{expected_offset:X} "
                    f"for block_size 0x{block_size:X}"
                ))

    return errors


def validate_sf32lb57_acpu_ram_data(ptab_obj, partitions: List[dict]) -> List[ValidationError]:
    """SF32LB57 ACPU images need an explicit ACPU RAM window.

    The 57 series runs ACPU code from PSRAM but keeps .data/.bss/stack in
    hpsys_ram, and its board headers derive HEAP_END from ACPU_RAM_DATA_*.
    Unlike the 58 series, the code region is not the RAM window, so the
    window cannot be inferred from the ACPU app partition.
    """
    errors: List[ValidationError] = []
    if str(getattr(ptab_obj, 'chip_series', '') or '').strip().lower() != 'sf32lb57':
        return errors

    if not any(
        str(partition.get('type') or '').strip() == 'app'
        and str(partition.get('core') or '').strip().upper() == 'ACPU'
        for partition in partitions
    ):
        return errors

    has_acpu_ram = any(
        str(partition.get('name') or '').strip() == 'acpu_ram_data'
        or 'ACPU_RAM_DATA' in {
            str(alias).strip().upper()
            for alias in (partition.get('aliases') or partition.get('tags') or [])
        }
        for partition in partitions
    )
    if not has_acpu_ram:
        errors.append(ValidationError(
            "SF32LB57 ACPU images require an 'acpu_ram_data' partition "
            "(or an 'ACPU_RAM_DATA' alias) in hpsys_ram: the ACPU HEAP_END "
            "and linker script resolve ACPU_RAM_DATA_* from it"
        ))
    return errors


def validate_ptab_v3(ptab_obj) -> List[ValidationError]:
    """Run all validations on a ptab v3 object"""
    errors = []

    if not ptab_obj.is_v3():
        errors.append(ValidationError("Not a v3 format ptab file"))
        return errors

    # Header checks
    if not str(getattr(ptab_obj, 'version', '') or '') == '3':
        errors.append(ValidationError("ptab.yaml: version must be 3"))
    chip = getattr(ptab_obj, 'chip', '') or ''
    if not str(chip).strip():
        errors.append(ValidationError("ptab.yaml: 'chip' is required"))

    chip_config = ptab_obj.get_chip_config()
    partitions = ptab_obj.partitions

    errors.extend(validate_name_unique(partitions))
    errors.extend(validate_aliases_conflict(partitions))

    # Per-partition validations
    for idx, p in enumerate(partitions):
        pname = p.get('name', f'partition_{idx}')
        errors.extend(validate_name(p.get('name', ''), idx))
        errors.extend(validate_type(p.get('type', ''), pname))
        errors.extend(validate_region(p.get('region', ''), pname, chip_config))
        errors.extend(validate_size(p.get('offset', 0), 'offset', pname))
        errors.extend(validate_size(p.get('size', 0), 'size', pname))
        errors.extend(validate_exec(p.get('exec'), pname, chip_config, p.get('core')))
        errors.extend(validate_sections(p.get('sections'), p))

        # Auto-exported FAL partitions must live on a region that maps to a
        # known FAL device macro.
        if _is_auto_fal_partition(p):
            region = str(p.get('region', '') or '').strip()
            if not _auto_fal_region_supported(region):
                errors.append(ValidationError(
                    f"Partition '{pname}': partitions that auto-generate FAL_PART_TABLE entries must use region 'mpi1'..'mpi5' or 'sdmmc1'/'sdmmc2' (got '{region}')"
                ))

        if p.get('type') == 'data' and p.get('subtype') == 'flashdb_kv':
            # Partition `name` is used as the FlashDB KV DB name and the FAL
            # partition name string. Legacy names like `kvdb_dfu_region` are
            # not supported and would generate wrong macros / FAL entries.
            m = re.match(r'^kvdb_(.+)_region$', str(pname).strip(), flags=re.IGNORECASE)
            if m:
                db_hint = (m.group(1) or '').strip().lower()
                if db_hint:
                    errors.append(ValidationError(
                        f"Partition '{pname}': flashdb_kv name must be the DB/FAL partition name (e.g. '{db_hint}'), not legacy '{pname}'"
                    ))
                else:
                    errors.append(ValidationError(
                        f"Partition '{pname}': flashdb_kv name must be the DB/FAL partition name (e.g. 'dfu', 'ble'), not legacy '{pname}'"
                    ))

    # Global validations
    errors.extend(validate_bootloader_unique(partitions))
    errors.extend(validate_ftab_unique(partitions))
    errors.extend(validate_factory_unique_by_core(partitions))
    errors.extend(validate_acpu_code_partitions(partitions))
    errors.extend(validate_nand_memory_block_sizes(ptab_obj))
    errors.extend(validate_nand_partition_alignment(partitions, chip_config))
    errors.extend(validate_sf32lb52_storage_reservations(ptab_obj, partitions, chip_config))
    errors.extend(validate_sf32lb52_nand_boot_layout(ptab_obj, partitions, chip_config))
    errors.extend(validate_sf32lb57_acpu_ram_data(ptab_obj, partitions))

    # Bootloader must have exec (execution address)
    bootloaders = [p for p in partitions if p.get('type') == 'bootloader']
    if len(bootloaders) == 1:
        if not isinstance(bootloaders[0].get('exec'), dict):
            errors.append(ValidationError("bootloader partition must provide 'exec: {region, offset}'"))

    errors.extend(validate_no_overlap(partitions, chip_config))
    errors.extend(validate_rom_index_deprecation(partitions))

    return errors


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate ptab v3 format",
        allow_abbrev=False
    )
    parser.add_argument(
        "ptab_file",
        help="Path to ptab.yaml file"
    )
    parser.add_argument(
        "--strict", "-s",
        action="store_true",
        help="Treat warnings as errors"
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    ptab_path = Path(args.ptab_file)
    if not ptab_path.exists():
        print(f"Error: File not found: {ptab_path}")
        sys.exit(1)

    # Load ptab
    try:
        ptab_obj = ptab_module.load_ptab(str(ptab_path), fatal=False)
    except Exception as e:
        print(f"Error loading ptab: {e}")
        sys.exit(1)

    # Validate
    errors = validate_ptab_v3(ptab_obj)

    # Report results
    error_count = sum(1 for e in errors if e.severity == "error")
    warning_count = sum(1 for e in errors if e.severity == "warning")

    for err in errors:
        print(err)

    print()
    if error_count == 0 and warning_count == 0:
        print(f"✓ {ptab_path}: Valid ptab v3 file")
        sys.exit(0)
    else:
        print(f"Found {error_count} error(s), {warning_count} warning(s)")
        if error_count > 0 or (args.strict and warning_count > 0):
            sys.exit(1)
        sys.exit(0)


if __name__ == "__main__":
    main()
