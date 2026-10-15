#!/usr/bin/env python3
# Copyright (c) 2025 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

"""
Generate ftab.bin for SiFli platforms from ptab v3 files.

This script replaces the ftab subproject and directly generates the binary
flash table used by the bootloader.
"""

from __future__ import annotations

import argparse
import enum
import os
import struct
import sys
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

# Add tools/build to path for ptab module
_TOOLS_BUILD_PATH = Path(__file__).parent
if str(_TOOLS_BUILD_PATH) not in sys.path:
    sys.path.insert(0, str(_TOOLS_BUILD_PATH))

import ptab as ptab_module


class PartitionIndex(enum.IntEnum):
    """Flash table partition indices"""
    FLASH_PARTITION_TABLE = 0
    CALIBRATION_TABLE = 1
    LCPU_IMAGE_PING = 2
    BOOTLOADER = 3
    HCPU_IMAGE = 4
    BOOT_PATCH = 5
    LCPU_IMAGE_PONG = 6
    BOOTLOADER_IMAGE_PONG = 7  # BCPU is bootloader
    HCPU_IMAGE_PONG = 8
    RAM_BOOT_PATCH = 9
    HCPU_IMAGE_RESERVE1 = 10
    HCPU_IMAGE_RESERVE2 = 11
    LCPU_IMAGE_RESERVE1 = 12
    LCPU_IMAGE_RESERVE2 = 13
    RESERVED1 = 14
    RESERVED2 = 15


class ImageFlag(enum.IntEnum):
    """DFU image flags"""
    DFU_FLAG_ENC = 1
    DFU_FLAG_AUTO = 2
    DFU_FLAG_SINGLE = 4
    DFU_IMGHDR_KEY_OFFSET = 8
    DFU_FLAG_COMPRESS = 16


# Constants for ftab binary format
PARTITION_ENTRY_COUNT = 16
IMAGE_DESCRIPTION_COUNT = 14
IMAGE_DESCRIPTION_SIZE = 512
PUBKEY_SIZE = 294
RESERVED_SIZE = 3542
IMAGE_INDEX_COUNT = 4
PARTITION_INFO_STRUCT = struct.Struct("<IIII")  # base, size, xip_base, flags
FLASH_TABLE_SIZE = 0x8000
SEC_CONFIG_MAGIC = 0x53454346  # 'FCES'
EX_IMAGE_SLOT_FLASH_IDS = [11, 12, 13]  # DFU_FLASH_HCPU_EXT2 / LCPU_EXT1 / LCPU_EXT2
EX_IMAGE_SLOT_NAMES = ['DFU_FLASH_HCPU_EXT2', 'DFU_FLASH_LCPU_EXT1', 'DFU_FLASH_LCPU_EXT2']
EX_IMAGE_SLOT_PARTITION_INDICES = [
    PartitionIndex.HCPU_IMAGE_RESERVE2,
    PartitionIndex.LCPU_IMAGE_RESERVE1,
    PartitionIndex.LCPU_IMAGE_RESERVE2,
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate ftab.bin from ptab file",
        allow_abbrev=False
    )
    parser.add_argument(
        "--ptab", "-p",
        required=True,
        help="Path to ptab.json or ptab.yaml file"
    )
    parser.add_argument(
        "--output", "-o",
        required=True,
        help="Path to write the generated ftab.bin"
    )
    parser.add_argument(
        "--chip", "-c",
        help="Chip series (e.g., sf32lb52), auto-detected from ptab v3"
    )
    parser.add_argument(
        "--bootloader-size",
        type=lambda x: int(x, 0),
        default=0x10000,
        help="Bootloader binary size (default: 0x10000)"
    )
    parser.add_argument(
        "--main-size",
        type=lambda x: int(x, 0),
        default=0x200000,
        help="Main application binary size (default: 0x200000)"
    )
    return parser.parse_args()


def _get_partition_core(partition: dict) -> str:
    return str(partition.get('core') or 'HCPU').strip().upper()


def _collect_partition_roles(partitions: List[dict]) -> Dict[str, object]:
    ftab_partition = None
    calibration_partition = None
    bootloader_partition = None
    factory_hcpu_partition = None
    dfu_hcpu_partition = None

    for p in partitions:
        ptype = p.get('type', '')
        subtype = p.get('subtype', '')
        core = _get_partition_core(p)
        if ptype == 'ftab' and ftab_partition is None:
            ftab_partition = p
        elif ptype == 'data' and subtype == 'calibration' and calibration_partition is None:
            calibration_partition = p
        elif ptype == 'bootloader' and bootloader_partition is None:
            bootloader_partition = p
        elif ptype == 'app' and subtype == 'factory' and core == 'HCPU' and factory_hcpu_partition is None:
            factory_hcpu_partition = p
        elif ptype == 'app' and subtype == 'dfu' and core == 'HCPU' and dfu_hcpu_partition is None:
            dfu_hcpu_partition = p

    if factory_hcpu_partition is None:
        for p in partitions:
            if p.get('type') == 'app' and p.get('subtype') == 'factory':
                # Never adopt another core's image as the HCPU main image: an ACPU code
                # partition would take ftab[4] (HCPU_IMAGE) instead of an extended slot.
                core = _get_partition_core(p)
                if core != 'HCPU':
                    raise ValueError(
                        "ptab v3 has no HCPU factory app partition; refusing to use "
                        "'{}' (core {}) as the main image".format(p.get('name', '?'), core))
                factory_hcpu_partition = p
                break
    if dfu_hcpu_partition is None:
        for p in partitions:
            if p.get('type') == 'app' and p.get('subtype') == 'dfu':
                dfu_hcpu_partition = p
                break

    extra_app_partitions: List[dict] = []
    for p in partitions:
        if p.get('type') != 'app':
            continue
        if str(p.get('subtype') or '').strip().lower() == 'ex':
            continue
        if p is factory_hcpu_partition or p is dfu_hcpu_partition:
            continue
        extra_app_partitions.append(p)

    return {
        'ftab': ftab_partition,
        'calibration': calibration_partition,
        'bootloader': bootloader_partition,
        'factory_hcpu': factory_hcpu_partition,
        'dfu_hcpu': dfu_hcpu_partition,
        'extra_apps': extra_app_partitions,
    }


def build_partition_table_entries(
    partitions: List[dict],
    chip_config: dict,
    chip: Optional[str] = None,
) -> Tuple[List[Tuple[int, int, int, int]], Dict[str, int]]:
    """Build partition table entries from ptab partitions.

    v3 address rules:
    - entry.base uses the "download/storage" view:
      - NOR: XIP (cbus)
      - NAND/RAM/PSRAM/SDMMC: base (sbus)
    - entry.xip_base uses the execution view:
      - Partitions with explicit `exec`: always use exec SBUS address
      - Partitions without `exec`: keep legacy XIP selection
        - NOR/PSRAM: XIP (cbus)
        - NAND/RAM: base (sbus)
    - For app/bootloader partitions with `exec`, xip_base comes from
      exec.{region,offset} and is always encoded as the target SBUS address.

    Returns:
        - List of (base, size, xip_base, flags) tuples
        - Mapping of extra image partition name -> flash_id (11/12/13)
    """
    entries = [(0, 0, 0, 0)] * PARTITION_ENTRY_COUNT
    extra_image_flash_ids: Dict[str, int] = {}
    chip_series = str(chip_config.get('series') or '').strip().lower()
    if not chip_series:
        chip_u = str(chip or '').strip().upper()
        if chip_u.startswith('SF32LB52'):
            chip_series = 'sf32lb52'
        elif chip_u.startswith('SF32LB56'):
            chip_series = 'sf32lb56'
        elif chip_u.startswith('SF32LB58'):
            chip_series = 'sf32lb58'
        elif chip_u.startswith('SF32LB55'):
            chip_series = 'sf32lb55'

    def _mpi_name_from_region(region: str) -> Optional[str]:
        if not region:
            return None
        if region.startswith('mpi'):
            return region
        if region.startswith('psram'):
            if region == 'psram':
                return 'mpi1'
            suffix = region.replace('psram', '')
            if suffix.isdigit():
                return f'mpi{suffix}'
        return None

    def _get_region_mem_type(region: str) -> str:
        if region == 'hpsys_ram' or region.startswith('hpsys') or region == 'lpsys_ram' or region.startswith('lpsys'):
            return 'ram'
        if region.startswith('sdmmc'):
            info = chip_config.get('memory_info', {}).get(region, {})
            mtype = info.get('type')
            return str(mtype).lower() if mtype is not None else 'sd'
        mpi_name = _mpi_name_from_region(region)
        if mpi_name:
            info = chip_config.get('memory_info', {}).get(mpi_name, {})
            mtype = (info.get('type') or '').lower()
            return mtype or 'nor'
        return ''

    def _select_download_addr(region: str, sbus_addr: int, cbus_addr: int) -> int:
        return cbus_addr if _get_region_mem_type(region) == 'nor' else sbus_addr

    def _select_exec_addr(region: str, sbus_addr: int, cbus_addr: int) -> int:
        mem_type = _get_region_mem_type(region)
        return sbus_addr if mem_type in ('ram', 'nand') else cbus_addr

    def _select_ftab_exec_addr(exec_sbus_addr: int) -> int:
        # In ftab, xip_base is the boot/DFU destination address for any
        # partition with explicit exec metadata. Always emit the SBUS view so
        # storage type does not affect the copy target encoding.
        return int(exec_sbus_addr)

    def _get_flash_boot_loader_size_default() -> Optional[int]:
        try:
            sifli_sdk = os.getenv('SIFLI_SDK')
            if not sifli_sdk:
                # Fallback for offline tools
                sifli_sdk = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
            if not sifli_sdk:
                return None
            chip_u = (chip or '').upper()
            if chip_u.startswith('SF32LB52'):
                cmsis_dir = 'sf32lb52x'
            elif chip_u.startswith('SF32LB56'):
                cmsis_dir = 'sf32lb56x'
            elif chip_u.startswith('SF32LB58'):
                cmsis_dir = 'sf32lb58x'
            else:
                return None
            import gen_link_lds
            mem_ints = gen_link_lds._load_mem_map_ints(sifli_sdk, cmsis_dir)
            v = mem_ints.get('FLASH_BOOT_LOADER_SIZE')
            if v is None:
                return None
            v = int(v)
            return v if v > 0 else None
        except Exception:
            return None

    _mem_map_cache: Dict[str, int] = {}
    _mem_map_loaded = False

    def _load_mem_map_ints() -> Dict[str, int]:
        nonlocal _mem_map_loaded, _mem_map_cache
        if _mem_map_loaded:
            return _mem_map_cache
        _mem_map_loaded = True
        try:
            sifli_sdk = os.getenv('SIFLI_SDK')
            if not sifli_sdk:
                sifli_sdk = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
            if not sifli_sdk:
                return _mem_map_cache
            chip_u = (chip or '').upper()
            if chip_u.startswith('SF32LB52'):
                cmsis_dir = 'sf32lb52x'
            elif chip_u.startswith('SF32LB56'):
                cmsis_dir = 'sf32lb56x'
            elif chip_u.startswith('SF32LB58'):
                cmsis_dir = 'sf32lb58x'
            else:
                return _mem_map_cache
            import gen_link_lds
            _mem_map_cache = dict(gen_link_lds._load_mem_map_ints(sifli_sdk, cmsis_dir))
        except Exception:
            _mem_map_cache = {}
        return _mem_map_cache

    def _acpu_exec_host_addr(exec_region: str, exec_offset: int) -> Optional[int]:
        """Resolve ACPU exec RAM address from HCPU/bootloader view.

        In legacy ftab.c, ACPU image xip_base uses HCPU-visible RAM address
        (e.g. HPSYS_RAM_BASE + offset -> 0x2020_0000), not ACPU local view.
        Returns None for regions this mapping does not cover (PSRAM/NOR); the
        caller then falls back to the SBUS/cbus rule.
        """
        r = (exec_region or '').strip().lower()
        mem_map_ints = _load_mem_map_ints()
        if r == 'hpsys_ram' or r.startswith('hpsys'):
            base = mem_map_ints.get('HPSYS_RAM_BASE', 0x20000000)
            return int(base) + int(exec_offset)
        if r == 'lpsys_ram' or r.startswith('lpsys'):
            base = mem_map_ints.get('LPSYS_RAM_BASE')
            if base is None:
                # Legacy default on older chips.
                base = 0x20400000
            return int(base) + int(exec_offset)
        return None

    def _partition_entry_addr(partition: dict) -> Tuple[int, int, int]:
        region = partition.get('region', '')
        offset = ptab_module.parse_size(partition.get('offset', 0))
        size = ptab_module.parse_size(partition.get('size', 0))
        core = partition.get('core')
        sbus_addr, cbus_addr = ptab_module.resolve_region_address(region, offset, chip_config, core=core)
        base_addr = _select_download_addr(region, sbus_addr, cbus_addr)
        exec_def = partition.get('exec')
        if isinstance(exec_def, dict):
            exec_region = str(exec_def.get('region', '')).strip()
            exec_offset = ptab_module.parse_size(exec_def.get('offset', 0))
            core_u = str(core or '').strip().upper()
            if core_u == 'ACPU':
                # RAM execution regions use the HCPU/bootloader view of the same memory
                # (see _acpu_exec_host_addr); regions it does not cover (PSRAM/NOR) fall
                # back to the SBUS/cbus rule below, i.e. the address the core that
                # copies the image uses.
                host_addr = _acpu_exec_host_addr(exec_region, exec_offset)
                if host_addr is not None:
                    xip_addr = int(host_addr)
                else:
                    exec_sbus_addr, _ = ptab_module.resolve_region_address(exec_region, exec_offset, chip_config, core=core)
                    xip_addr = _select_ftab_exec_addr(exec_sbus_addr)
                if not xip_addr:
                    raise ValueError(
                        "partition '{}': cannot resolve the ACPU execution address of "
                        "region '{}'".format(partition.get('name', '?'), exec_region))
            else:
                exec_sbus_addr, _ = ptab_module.resolve_region_address(exec_region, exec_offset, chip_config, core=core)
                xip_addr = _select_ftab_exec_addr(exec_sbus_addr)
        else:
            mem_type = _get_region_mem_type(region)
            if mem_type == 'nand':
                raise ValueError(
                    "app partition '{}' on NAND must provide exec for ftab generation".format(
                        partition.get('name', '?')
                    )
                )
            xip_addr = int(_select_exec_addr(region, sbus_addr, cbus_addr))
        return int(base_addr), int(size), int(xip_addr)

    roles = _collect_partition_roles(partitions)
    ftab_partition = roles['ftab']
    calibration_partition = roles['calibration']
    bootloader_partition = roles['bootloader']
    main_partition = roles['factory_hcpu']
    dfu_partition = roles['dfu_hcpu']
    extra_app_partitions = roles['extra_apps']

    chip_u = str(chip or '').strip().upper()
    bootloader_required = not chip_u.startswith('SF32LB55')
    if ftab_partition is None:
        raise ValueError("ftab partition missing; default inference failed")
    if bootloader_required and bootloader_partition is None:
        raise ValueError("bootloader partition missing; default inference failed")

    # Fill ftab entry
    size = ptab_module.parse_size(ftab_partition.get('size', 0))
    region = ftab_partition.get('region', '')
    offset = ptab_module.parse_size(ftab_partition.get('offset', 0))
    sbus_addr, cbus_addr = ptab_module.resolve_region_address(region, offset, chip_config, core=ftab_partition.get('core'))
    base_addr = _select_download_addr(region, sbus_addr, cbus_addr)

    entries[PartitionIndex.FLASH_PARTITION_TABLE] = (base_addr, size, 0, 0)

    # Fill calibration entry
    if calibration_partition:
        size = ptab_module.parse_size(calibration_partition.get('size', 0))
        region = calibration_partition.get('region', '')
        offset = ptab_module.parse_size(calibration_partition.get('offset', 0))
        sbus_addr, cbus_addr = ptab_module.resolve_region_address(region, offset, chip_config, core=calibration_partition.get('core'))
        base_addr = _select_download_addr(region, sbus_addr, cbus_addr)
        
        entries[PartitionIndex.CALIBRATION_TABLE] = (base_addr, size, 0, 0)

    # Fill bootloader entry
    if bootloader_partition:
        storage_size = ptab_module.parse_size(bootloader_partition.get('size', 0))
        size = _get_flash_boot_loader_size_default()
        if size is None:
            size = storage_size
        elif storage_size > size:
            size = storage_size
        region = bootloader_partition.get('region', '')
        offset = ptab_module.parse_size(bootloader_partition.get('offset', 0))
        sbus_addr, cbus_addr = ptab_module.resolve_region_address(region, offset, chip_config, core=bootloader_partition.get('core'))
        base_addr = _select_download_addr(region, sbus_addr, cbus_addr)

        exec_def = bootloader_partition.get('exec')
        if not isinstance(exec_def, dict):
            raise ValueError("bootloader partition must provide exec for ftab generation")

        exec_region = str(exec_def.get('region', '')).strip()
        exec_offset = ptab_module.parse_size(exec_def.get('offset', 0))
        exec_sbus_addr, _ = ptab_module.resolve_region_address(exec_region, exec_offset, chip_config, core=bootloader_partition.get('core'))
        xip_addr = _select_ftab_exec_addr(exec_sbus_addr)

        entries[PartitionIndex.BOOTLOADER] = (base_addr, size, xip_addr, 0)
        entries[PartitionIndex.BOOTLOADER_IMAGE_PONG] = (base_addr, size, xip_addr, 0)

    # Fill main entry
    if main_partition:
        base_addr, size, xip_addr = _partition_entry_addr(main_partition)
        entries[PartitionIndex.HCPU_IMAGE] = (base_addr, size, xip_addr, 0)
        entries[PartitionIndex.HCPU_IMAGE_PONG] = (base_addr, size, xip_addr, 0)

    # Fill dfu entry (optional)
    if dfu_partition:
        base_addr, size, xip_addr = _partition_entry_addr(dfu_partition)
        entries[PartitionIndex.LCPU_IMAGE_PING] = (base_addr, size, xip_addr, 0)

    # Only named partitions can be registered in the flash table.
    extra_app_partitions = [
        p for p in extra_app_partitions if str(p.get('name') or '').strip()
    ]

    # All extra images share the three extended ftab slots, so order them
    # deterministically: extra ACPU code images keep their image order (region 1
    # before region 2), everything else follows by name. An overlay file or the
    # injected default partitions can then never move an image to another slot.
    def _extra_slot_order(part: dict):
        if ptab_module.is_acpu_code_partition_v3(part):
            return (0, ptab_module.get_acpu_code_rom_index(part) or 0, str(part.get('name') or ''))
        return (1, 0, str(part.get('name') or ''))

    extra_app_partitions = sorted(extra_app_partitions, key=_extra_slot_order)

    if len(extra_app_partitions) > len(EX_IMAGE_SLOT_PARTITION_INDICES):
        names = [str(p.get('name') or '?') for p in extra_app_partitions]
        raise ValueError(
            "Too many extra images for the ftab extended slots ({} = flash ids {}); the pool "
            "is shared by every 'type: app' partition that is not the HCPU factory/DFU "
            "image: {}".format(
                '/'.join(EX_IMAGE_SLOT_NAMES), '/'.join(str(i) for i in EX_IMAGE_SLOT_FLASH_IDS),
                ", ".join(names))
        )

    for idx, part in enumerate(extra_app_partitions):
        part_name = str(part.get('name') or '').strip()
        base_addr, size, xip_addr = _partition_entry_addr(part)
        part_idx = EX_IMAGE_SLOT_PARTITION_INDICES[idx]
        flash_id = EX_IMAGE_SLOT_FLASH_IDS[idx]
        entries[part_idx] = (base_addr, size, xip_addr, 0)
        extra_image_flash_ids[part_name] = int(flash_id)

    return entries, extra_image_flash_ids


def pack_partition_table(entries: List[Tuple[int, int, int, int]]) -> bytes:
    """Pack partition table entries to binary"""
    out = bytearray()
    for base, size, xip_base, flags in entries:
        out.extend(PARTITION_INFO_STRUCT.pack(base, size, xip_base, flags))
    return bytes(out)


def pack_image_description(length: int, used: bool) -> bytes:
    """Pack a single image description entry"""
    block_size = 512 if used else 0
    flags = ImageFlag.DFU_FLAG_AUTO if used else 0
    header = struct.pack("<IHH", length, block_size, flags)
    payload_len = IMAGE_DESCRIPTION_SIZE - len(header)
    return header + bytes(payload_len)


def build_image_descriptions(
    bootloader_size: int,
    main_size: int,
    dfu_size: int = 0,
    hcpu2_size: Optional[int] = None,
    ext_image_sizes: Optional[Dict[int, int]] = None,
) -> bytes:
    """Build image description table

    Image description array indices correspond to:
    imgs[idx] = Flash ID (idx + 2)
    - imgs[0] = Flash ID 2 = DFU_FLASH_IMG_LCPU
    - imgs[1] = Flash ID 3 = DFU_FLASH_IMG_BL
    - imgs[2] = Flash ID 4 = DFU_FLASH_IMG_HCPU
    - imgs[3] = Flash ID 5 = DFU_FLASH_IMG_BOOT
    - imgs[4] = Flash ID 6 = DFU_FLASH_IMG_LCPU2
    - imgs[5] = Flash ID 7 = DFU_FLASH_IMG_BCPU2
    - imgs[6] = Flash ID 8 = DFU_FLASH_IMG_HCPU2
    ...
    """
    desc = bytearray()

    if hcpu2_size is None:
        # Keep legacy behaviour: HCPU2 uses HCPU size when backup slot exists.
        hcpu2_size = main_size

    # Sizes array: index = Flash ID - 2
    sizes = [0xFFFFFFFF] * IMAGE_DESCRIPTION_COUNT
    sizes[0] = dfu_size if dfu_size > 0 else 0xFFFFFFFF
    sizes[1] = bootloader_size
    sizes[2] = main_size
    sizes[6] = hcpu2_size if hcpu2_size and hcpu2_size > 0 else 0xFFFFFFFF

    for flash_id, length in (ext_image_sizes or {}).items():
        idx = int(flash_id) - 2
        if idx < 0 or idx >= IMAGE_DESCRIPTION_COUNT:
            continue
        sizes[idx] = int(length) if int(length) > 0 else 0xFFFFFFFF

    for idx in range(IMAGE_DESCRIPTION_COUNT):
        length = sizes[idx]
        used = length != 0xFFFFFFFF and length != 0
        desc.extend(pack_image_description(length, used))

    return bytes(desc)


def build_image_index_table(flash_base: int, dfu_present: bool = False) -> bytes:
    """Build image index table

    Image Index entries point to image descriptions by CORE_* index:
    - entries[0] = CORE_LCPU → imgs[0] (Flash ID 2)
    - entries[1] = CORE_BL → imgs[1] (Flash ID 3)
    - entries[2] = CORE_HCPU → imgs[2] (Flash ID 4)
    - entries[3] = CORE_BOOT → imgs[3] (Flash ID 5)
    """
    base_offset = (
        flash_base
        + 4  # magic
        + PARTITION_ENTRY_COUNT * PARTITION_INFO_STRUCT.size
        + PUBKEY_SIZE
        + RESERVED_SIZE
    )

    # Index corresponds to CORE_* enum
    entries = [
        base_offset + 0 * IMAGE_DESCRIPTION_SIZE if dfu_present else 0xFFFFFFFF,  # [0] LCPU/DFU
        base_offset + 1 * IMAGE_DESCRIPTION_SIZE,    # [1] Bootloader → imgs[1]
        base_offset + 2 * IMAGE_DESCRIPTION_SIZE,    # [2] HCPU → imgs[2]
        0xFFFFFFFF,                                   # [3] Boot - not used
    ]
    return struct.pack("<" + "I" * IMAGE_INDEX_COUNT, *entries)


def generate_ftab_binary(
    ptab_obj,
    chip_config: dict,
    bootloader_size: int,
    main_size: int,
    image_sizes: Optional[Dict[str, int]] = None,
    enabled_images: Optional[Set[str]] = None,
) -> bytes:
    """Generate complete ftab.bin content"""

    # Get partitions from v3 ptab
    partitions = ptab_obj.partitions
    image_sizes = dict(image_sizes or {})
    enabled_image_names = {
        str(name or '').strip().lower()
        for name in image_sizes.keys()
        if str(name or '').strip()
    }
    enabled_image_names.update(
        str(name or '').strip().lower()
        for name in (enabled_images or set())
        if str(name or '').strip()
    )
    roles = _collect_partition_roles(partitions)

    # Get flash base for ftab (download/storage view)
    flash_base = 0
    ftab_partition = roles.get('ftab')
    if isinstance(ftab_partition, dict):
        region = ftab_partition.get('region', '')
        offset = ptab_module.parse_size(ftab_partition.get('offset', 0))
        flash_base = ptab_module.get_download_addr_v3(region, offset, chip_config, core=ftab_partition.get('core'))

    # Build components
    entries, ext_flash_ids = build_partition_table_entries(partitions, chip_config, getattr(ptab_obj, 'chip', None))
    partition_table = pack_partition_table(entries)

    def _partition_size(part: object, default: int = 0) -> int:
        if isinstance(part, dict):
            return int(ptab_module.parse_size(part.get('size', default)))
        return int(default)

    partition_size_by_name: Dict[str, int] = {}
    partition_by_name: Dict[str, dict] = {}
    for p in partitions:
        pname = str(p.get('name') or '').strip()
        if not pname:
            continue
        partition_size_by_name[pname] = int(ptab_module.parse_size(p.get('size', 0)))
        partition_by_name[pname] = p

    bootloader_part = roles.get('bootloader')
    factory_hcpu = roles.get('factory_hcpu')
    dfu_hcpu = roles.get('dfu_hcpu')

    # Use partition max size as image length policy.
    bootloader_len = _partition_size(bootloader_part, bootloader_size)
    if bootloader_len <= 0:
        bootloader_len = int(bootloader_size)

    main_len = _partition_size(factory_hcpu, main_size)
    if main_len <= 0:
        main_len = int(main_size)

    # Match v2 default behaviour:
    # - only enable LCPU/DFU image when the DFU subproject/image is present
    # - otherwise keep CORE_LCPU image pointer invalid and clear ftab[2]
    # ftab[6] is the LCPU pong slot and is not used for DFU.
    dfu_enabled = False
    dfu_size = 0
    if isinstance(dfu_hcpu, dict):
        dfu_name = str(dfu_hcpu.get('name') or '').strip().lower()
        if 'dfu' in enabled_image_names or (dfu_name and dfu_name in enabled_image_names):
            dfu_enabled = True
            dfu_size = _partition_size(dfu_hcpu, 0)
    elif 'dfu' in enabled_image_names:
        raise ValueError("DFU image enabled but ptab v3 has no app partition with subtype 'dfu'")

    entries[PartitionIndex.LCPU_IMAGE_PONG] = (0, 0, 0, 0)
    if not dfu_enabled:
        entries[PartitionIndex.LCPU_IMAGE_PING] = (0, 0, 0, 0)
    partition_table = pack_partition_table(entries)

    # An ACPU image is only advertised when the build really produced one, which is
    # visible as an ACPU partition named after its build env. Extra images of the same
    # project are declared without such an env, so they follow the same gate instead of
    # always being registered from their partition size.
    acpu_image_built = any(
        ptab_module.is_acpu_code_partition_v3(p) and str(p.get('name') or '').strip() in image_sizes
        for p in partitions
    )

    ext_image_sizes: Dict[int, int] = {}
    for part_name, flash_id in ext_flash_ids.items():
        part = partition_by_name.get(part_name)
        if part_name in image_sizes:
            ext_size = int(partition_size_by_name.get(part_name, 0))
            if ext_size <= 0:
                ext_size = int(image_sizes[part_name])
            ext_image_sizes[int(flash_id)] = int(ext_size)
            continue
        # Extra ACPU code images: the partition size is the image size.
        if (acpu_image_built and part is not None
                and ptab_module.get_acpu_code_rom_index(part) is not None):
            ext_size = int(partition_size_by_name.get(part_name, 0))
            if ext_size <= 0:
                raise ValueError(
                    "ACPU code partition '{}' must declare a non-zero size".format(part_name))
            ext_image_sizes[int(flash_id)] = int(ext_size)

    image_descriptions = build_image_descriptions(
        bootloader_size=bootloader_len,
        main_size=main_len,
        dfu_size=dfu_size,
        hcpu2_size=main_len,
        ext_image_sizes=ext_image_sizes,
    )
    image_index_table = build_image_index_table(flash_base, dfu_present=dfu_enabled)

    # Assemble ftab binary
    blob = bytearray()
    blob.extend(struct.pack("<I", SEC_CONFIG_MAGIC))
    blob.extend(partition_table)
    blob.extend(bytes(PUBKEY_SIZE))
    blob.extend(bytes(RESERVED_SIZE))
    blob.extend(image_descriptions)
    blob.extend(image_index_table)

    return bytes(blob)


def main() -> None:
    args = parse_args()

    # Load ptab
    ptab_path = Path(args.ptab)
    if not ptab_path.exists():
        print(f"Error: ptab file not found: {ptab_path}")
        sys.exit(1)

    ptab_obj = ptab_module.load_ptab(str(ptab_path), fatal=True)

    # Only v3 format is supported
    if not ptab_obj.is_v3():
        print(f"Error: Only ptab v3 format is supported. Use ftab subproject for v1/v2.")
        sys.exit(1)

    # Get chip config
    chip_config = ptab_obj.get_chip_config()

    # Generate ftab binary
    ftab_binary = generate_ftab_binary(
        ptab_obj,
        chip_config,
        args.bootloader_size,
        args.main_size
    )

    # Write output
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(ftab_binary)

    print(f"Generated ftab.bin: {output_path} ({len(ftab_binary)} bytes)")


if __name__ == "__main__":
    main()
