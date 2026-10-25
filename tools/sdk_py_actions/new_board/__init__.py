# SPDX-FileCopyrightText: 2025 SiFli Technologies Co., Ltd
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

import json
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import yaml

from sdk_py_actions.errors import FatalError

KB = 1024
MB = 1024 * KB

KV_PART_SIZE = 0x00004000
MIN_FACTORY_APP_PART_SIZE = 500 * KB
NAND_BLOCK_SIZE_KB_DEFAULT = 128
NAND_BLOCK_SIZE_KB_CHOICES = (128, 256)
SF32LB52_NAND_BOOTLOADER_BLOCK_INDEX = 4
SF32LB52_NAND_BOOTLOADER_SIZE = 0x00010000
SF32LB57_NAND_BOOTLOADER_BLOCK_INDEX = 4
SF32LB57_NAND_BOOTLOADER_SIZE = 0x00010000
ACPU_PARTITION_SIZE = 0x00040000
# ACPU private RAM window inside hpsys_ram, right after the HCPU RAM data
# region (0x0006FC00) as used by the existing 57-series v2 boards. It is
# consumed by bsp_board.h for the ACPU heap end.
ACPU_RAM_DATA_OFFSET = 0x0006FC00
ACPU_RAM_DATA_SIZE = 0x0000FC00
# On series 57 SD cards the ACPU image does not follow the HCPU app region
# directly; it keeps the offset used by the existing boards (app start +
# 7 MB), which is also where the v2 layout placed it before the +0x50000 shift.
SF32LB57_SD_ACPU_OFFSET_FROM_APP = 0x00700000
STORAGE_OFFSET_ALIGNMENT = {
    'sdmmc': 512,
}

FIXED_PARTITION_LAYOUTS = {
    ('52', 'none'): {
        'hcpu_flash_code': (0x00020000, 0x00030000),
        'fs_region': (0x00050000, 0x000A8000),
        'dfu': (0x000F8000, 0x00004000),
        'ble': (0x000FC000, 0x00004000),
    },
    ('52', 'nor'): {
        'hcpu_flash_code': (0x00020000, 0x00700000),
        'fs_region': (0x008A0000, 0x00400000),
        'dfu': (0x00CA0000, 0x00004000),
        'ble': (0x00CA4000, 0x00004000),
    },
    ('52', 'nand'): {
        'hcpu_flash_code': (0x00090000, 0x00400000),
        'fs_region': (0x008A0000, 0x00400000),
        'dfu': (0x00CA0000, 0x00004000),
        'ble': (0x00CA4000, 0x00004000),
    },
    ('52', 'sdmmc'): {
        'hcpu_flash_code': (0x00071000, 0x00700000),
        'fs_region': (0x008F1000, 0x00400000),
        'dfu': (0x00CF1000, 0x00004000),
        'ble': (0x00CF5000, 0x00004000),
    },
    ('56', 'none'): {
        'hcpu_flash_code': (0x00040000, 0x00010000),
        'fs_region': (0x00050000, 0x00028000),
        'dfu': (0x00078000, 0x00004000),
        'ble': (0x0007C000, 0x00004000),
    },
    ('56', 'nor'): {
        'hcpu_flash_code': (0x00000000, 0x00200000),
        'fs_region': (0x003A0000, 0x00260000),
        'dfu': (0x00600000, 0x00004000),
        'ble': (0x00604000, 0x00004000),
    },
    ('56', 'nand'): {
        'hcpu_flash_code': (0x00000000, 0x00200000),
        'fs_region': (0x003A0000, 0x00260000),
        'dfu': (0x00600000, 0x00004000),
        'ble': (0x00604000, 0x00004000),
    },
    ('56', 'sdmmc'): {
        'hcpu_flash_code': (0x00000000, 0x00200000),
        'fs_region': (0x003A0000, 0x00260000),
        'dfu': (0x00600000, 0x00004000),
        'ble': (0x00604000, 0x00004000),
    },
    ('58', 'none'): {
        'hcpu_flash_code': (0x00040000, 0x00020000),
        'fs_region': (0x00060000, 0x00098000),
        'dfu': (0x000F8000, 0x00004000),
        'ble': (0x000FC000, 0x00004000),
    },
    ('58', 'nor'): {
        'hcpu_flash_code': (0x00000000, 0x00200000),
        'fs_region': (0x00A00000, 0x00100000),
        'dfu': (0x00400000, 0x00004000),
        'ble': (0x00404000, 0x00004000),
    },
    ('58', 'nand'): {
        'hcpu_flash_code': (0x00000000, 0x00200000),
        'fs_region': (0x00A00000, 0x00100000),
        'dfu': (0x00400000, 0x00004000),
        'ble': (0x00404000, 0x00004000),
    },
    ('58', 'sdmmc'): {
        'hcpu_flash_code': (0x00000000, 0x00200000),
        'fs_region': (0x00A00000, 0x00100000),
        'dfu': (0x00400000, 0x00004000),
        'ble': (0x00404000, 0x00004000),
    },
    ('57', 'none'): {
        'hcpu_flash_code': (0x00020000, 0x00010000),
        'fs_region': (0x00050000, 0x00028000),
        'dfu': (0x00078000, 0x00004000),
        'ble': (0x0007C000, 0x00004000),
    },
    ('57', 'nor'): {
        'hcpu_flash_code': (0x00020000, 0x00700000),
        'fs_region': (0x008E0000, 0x00600000),
        'dfu': (0x00EE0000, 0x00004000),
        'ble': (0x00EE4000, 0x00004000),
    },
    ('57', 'nand'): {
        'hcpu_flash_code': (0x000A0000, 0x00300000),
        'fs_region': (0x00540000, 0x00300000),
        'dfu': (0x00840000, 0x00004000),
        'ble': (0x00844000, 0x00004000),
    },
    ('57', 'sdmmc'): {
        'hcpu_flash_code': (0x00071000, 0x00300000),
        'fs_region': (0x00931000, 0x00600000),
        'dfu': (0x00F31000, 0x00004000),
        'ble': (0x00F35000, 0x00004000),
    },
}

SERIES_DIRS = {
    '52': ['SF32LB52_X', 'SF32LB52x'],
    '56': ['SF32LB56x'],
    '57': ['SF32LB57x'],
    '58': ['SF32LB58x'],
}

SERIES_LABEL = {
    '52': 'SF32LB52',
    '56': 'SF32LB56',
    '57': 'SF32LB57',
    '58': 'SF32LB58',
}

FAMILY_CHIP = {
    '52': 'SF32LB52X',
    '56': 'SF32LB56X',
    '57': 'SF32LB57X',
    '58': 'SF32LB58X',
}

SOC_SYMBOL = {
    '52': 'SOC_SF32LB52X',
    '56': 'SOC_SF32LB56X',
    '57': 'SOC_SF32LB57X',
    '58': 'SOC_SF32LB58X',
}

SAMPLE_BASE = {
    '52': 'customer/boards/sf32lb52-core_base',
    '56': 'customer/boards/sf32lb56-lcd_base',
    '57': 'customer/boards/sf32lb57-dpi-hdk_base',
    '58': 'customer/boards/sf32lb58-core_base',
}

# External NOR/NAND storage ports allowed per series, and their defaults.
# Series not present here use the generic path (52 -> mpi2, 56 -> mpi3).
# SF32LB57X NAND is only wired to mpi3, while NOR may use mpi2 or mpi3.
ALLOWED_STORAGE_PORTS = {
    '57': {
        'nor': ('mpi2', 'mpi3'),
        'nand': ('mpi3',),
    },
    '58': {
        'nor': ('mpi3', 'mpi4'),
        'nand': ('mpi3', 'mpi4'),
    },
}

DEFAULT_NOR_NAND_PORT = {
    '57': 'mpi3',
    '58': 'mpi3',
}

STORAGE_PINMUX_DETAILS = {
    'mpi4': {
        'type0': (
            'MPI4_CLK=PAD_PA09',
            'MPI4_CS=PAD_PA10',
            'MPI4_DIO0=PAD_PA05',
            'MPI4_DIO1=PAD_PA04',
            'MPI4_DIO2=PAD_PA01',
            'MPI4_DIO3=PAD_PA06',
        ),
        'type1': (
            'MPI4_CLK=PAD_PA39',
            'MPI4_CS=PAD_PA30',
            'MPI4_DIO0=PAD_PA40',
            'MPI4_DIO1=PAD_PA37',
            'MPI4_DIO2=PAD_PA36',
            'MPI4_DIO3=PAD_PA38',
        ),
    },
    'sdmmc1': {
        'type0': (
            'SD1_CLK=PAD_PA09',
            'SD1_CMD=PAD_PA10',
            'SD1_DIO0=PAD_PA05',
            'SD1_DIO1=PAD_PA04',
            'SD1_DIO2=PAD_PA01',
            'SD1_DIO3=PAD_PA06',
            'SD1_DIO4=PAD_PA07',
            'SD1_DIO5=PAD_PA03',
            'SD1_DIO6=PAD_PA08',
            'SD1_DIO7=PAD_PA00',
            'SD1_EN=PAD_PA02',
            'GPIO_A12=PAD_PA12',
            'GPIO_A13=PAD_PA13',
            'SD1_RESET=PAD_PA11',
        ),
        'type1': (
            'SD1_CLK=PAD_PA39',
            'SD1_CMD=PAD_PA34',
            'SD1_DIO0=PAD_PA41',
            'SD1_DIO1=PAD_PA30',
            'SD1_DIO2=PAD_PA36',
            'SD1_DIO3=PAD_PA40',
            'SD1_DIO4=PAD_PA38',
            'SD1_DIO5=PAD_PA37',
            'SD1_DIO6=PAD_PA35',
            'SD1_DIO7=PAD_PA33',
            'SD1_RESET=PAD_PA49',
            'SD1_EN=PAD_PA80',
        ),
    },
}

BOARD_NAME_RE = re.compile(r'^[a-z0-9][a-z0-9_-]*$')
PACKAGE_RE = re.compile(r'^(SF32LB\d{3})')


@dataclass(frozen=True)
class MemoryEntry:
    mpi: str
    mem_type: str
    size_bytes: int

    @property
    def size_mb(self) -> int:
        return max(1, (self.size_bytes + MB - 1) // MB)


@dataclass(frozen=True)
class ChipVariant:
    series: str
    chip_dir: str
    model_id: str
    part_number: str
    memory: Tuple[MemoryEntry, ...]

    def first_psram(self) -> Optional[MemoryEntry]:
        for entry in sorted(self.memory, key=_memory_sort_key):
            if entry.mem_type == 'psram':
                return entry
        return None

    def first_non_psram(self) -> Optional[MemoryEntry]:
        for entry in sorted(self.memory, key=_memory_sort_key):
            if entry.mem_type != 'psram':
                return entry
        return None

    def none_allowed(self) -> bool:
        return self.first_non_psram() is not None


@dataclass(frozen=True)
class Spec:
    series: str
    chip_model: str
    storage_type: str
    storage_size_mb: Optional[int]
    storage_port: Optional[str]
    board_name: str
    base_name: str
    generate_base: bool
    storage_pinmux_type: Optional[str] = None
    nand_block_size_kb: Optional[int] = None
    with_acpu: bool = False


@dataclass(frozen=True)
class Partition:
    name: str
    part_type: str
    region: str
    offset: int
    size: int
    subtype: Optional[str] = None
    core: Optional[str] = None
    aliases: Tuple[str, ...] = ()
    exec_region: Optional[str] = None
    exec_offset: Optional[int] = None
    attrs: Optional[Dict[str, Any]] = None


class HexInt(int):
    pass


class PtabYamlDumper(yaml.SafeDumper):
    def increase_indent(self, flow: bool = False, indentless: bool = False) -> None:
        return super().increase_indent(flow, False)


def _represent_hex_int(dumper: yaml.Dumper, data: HexInt) -> yaml.Node:
    return dumper.represent_scalar('tag:yaml.org,2002:int', f'0x{int(data):08X}')


PtabYamlDumper.add_representer(HexInt, _represent_hex_int)


def run_new_board(config_path: Optional[str], output_root: str) -> None:
    import click

    sdk_root = Path(os.environ['SIFLI_SDK_PATH']).resolve()
    output_root_path = Path(output_root).expanduser().resolve()
    output_root_path.mkdir(parents=True, exist_ok=True)
    assets_root = Path(__file__).resolve().parent

    variants = load_variants(sdk_root)
    schema_path = assets_root / 'schema' / 'new-board.schema.json'

    if config_path:
        with open(config_path, encoding='utf-8') as f:
            raw_config = json.load(f)
    else:
        raw_config = prompt_for_config(variants)

    spec, variant = normalize_spec(raw_config, schema_path, variants)

    rendered = render_board_files(
        spec=spec,
        variant=variant,
        sdk_root=sdk_root,
        output_root=output_root_path,
        assets_root=assets_root,
    )

    write_rendered_files(rendered)

    click.echo(f'Created board "{spec.board_name}" in {output_root_path}', err=True)
    if spec.generate_base:
        click.echo(f'Created base "{spec.base_name}" in {output_root_path}', err=True)


def load_variants(sdk_root: Path) -> Dict[str, List[ChipVariant]]:
    chips_root = sdk_root / 'tools' / 'SiliconSchema' / 'chips'
    result: Dict[str, List[ChipVariant]] = {series: [] for series in SERIES_DIRS}

    for series, chip_dirs in SERIES_DIRS.items():
        for chip_dir in chip_dirs:
            chip_yaml = chips_root / chip_dir / 'chip.yaml'
            if not chip_yaml.exists():
                continue
            with open(chip_yaml, encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}

            for variant in data.get('variants', []) or []:
                memory = tuple(
                    MemoryEntry(
                        mpi=str(mem.get('mpi') or '').strip().lower(),
                        mem_type=str(mem.get('type') or '').strip().lower(),
                        size_bytes=int(mem.get('size') or 0),
                    )
                    for mem in (variant.get('memory') or [])
                    if isinstance(mem, dict) and mem.get('mpi') and mem.get('type')
                )
                result[series].append(ChipVariant(
                    series=series,
                    chip_dir=chip_dir,
                    model_id=str(data.get('model_id') or '').strip(),
                    part_number=str(variant.get('part_number') or '').strip(),
                    memory=memory,
                ))

        result[series].sort(key=lambda item: item.part_number)

    return result


def prompt_for_config(variants: Dict[str, List[ChipVariant]]) -> Dict[str, Any]:
    try:
        import questionary
    except ImportError as e:
        raise FatalError('You need to install questionary package to use interactive new-board') from e

    def ask(result: Any, message: str) -> Any:
        value = result.ask()
        if value is None:
            raise FatalError(message)
        return value

    series = ask(
        questionary.select(
            'Select chip series',
            choices=[
                questionary.Choice(SERIES_LABEL['52'], value='52'),
                questionary.Choice(SERIES_LABEL['56'], value='56'),
                questionary.Choice(SERIES_LABEL['57'], value='57'),
                questionary.Choice(SERIES_LABEL['58'], value='58'),
            ],
        ),
        'Board creation cancelled.',
    )

    series_variants = variants.get(series, [])
    if not series_variants:
        raise FatalError(f'No chip variants found for series {series}')

    chip_model = ask(
        questionary.select(
            'Select chip model',
            choices=[
                questionary.Choice(
                    f'{variant.part_number}{_memory_summary_suffix(variant)}',
                    value=variant.part_number,
                )
                for variant in series_variants
            ],
        ),
        'Board creation cancelled.',
    )

    selected_variant = next(item for item in series_variants if item.part_number == chip_model)
    none_allowed = selected_variant.none_allowed()

    storage_choices = []
    if none_allowed:
        storage_choices.append(questionary.Choice('none', value='none'))
    storage_choices.extend([
        questionary.Choice('nor', value='nor'),
        questionary.Choice('nand', value='nand'),
        questionary.Choice('sdmmc', value='sdmmc'),
    ])

    storage_type = ask(
        questionary.select('Select external storage type', choices=storage_choices),
        'Board creation cancelled.',
    )

    storage_port: Optional[str] = None
    series_ports = ALLOWED_STORAGE_PORTS.get(series, {}).get(storage_type)
    if series_ports:
        storage_port = ask(
            questionary.select(
                'Select external storage port',
                choices=[
                    questionary.Choice(port, value=port)
                    for port in series_ports
                ],
            ),
            'Board creation cancelled.',
        )
    elif series in ('56', '57', '58') and storage_type == 'sdmmc':
        default_sdmmc = default_sdmmc_storage_port(series)
        other_sdmmc = 'sdmmc1' if default_sdmmc == 'sdmmc2' else 'sdmmc2'
        storage_port = ask(
            questionary.select(
                'Select SDMMC storage port',
                choices=[
                    questionary.Choice(default_sdmmc, value=default_sdmmc),
                    questionary.Choice(other_sdmmc, value=other_sdmmc),
                ],
            ),
            'Board creation cancelled.',
        )

    storage_pinmux_type: Optional[str] = None
    if series == '58' and storage_port in ('mpi4', 'sdmmc1'):
        storage_pinmux_type = ask(
            questionary.select(
                'Select storage pinmux type',
                choices=[
                    questionary.Choice(
                        'type0',
                        value='type0',
                        description=storage_pinmux_choice_description(storage_port, 'type0'),
                    ),
                    questionary.Choice(
                        'type1',
                        value='type1',
                        description=storage_pinmux_choice_description(storage_port, 'type1'),
                    ),
                ],
                style=questionary.Style([
                    ('text', 'fg:#8a8a8a'),
                ]),
            ),
            'Board creation cancelled.',
        )

    nand_block_size_kb: Optional[int] = None
    if storage_type == 'nand':
        nand_block_size_kb = ask(
            questionary.select(
                'Select NAND block size',
                choices=[
                    questionary.Choice('128KB (default)', value=128),
                    questionary.Choice('256KB', value=256),
                ],
                default=NAND_BLOCK_SIZE_KB_DEFAULT,
            ),
            'Board creation cancelled.',
        )

    storage_size_mb: Optional[int] = None
    if storage_type != 'none':
        raw_size = ask(
            questionary.text(
                'Input storage capacity (MB)',
                validate=lambda text: True if text.isdigit() and int(text) > 0 else 'Enter a positive integer.',
            ),
            'Board creation cancelled.',
        )
        storage_size_mb = int(raw_size)

    with_acpu = False
    if series == '57' and storage_type != 'none':
        with_acpu = ask(
            questionary.confirm('Generate an ACPU partition?', default=False),
            'Board creation cancelled.',
        )

    board_name = ask(
        questionary.text(
            'Input board name',
            validate=lambda text: True if BOARD_NAME_RE.fullmatch(text or '') else 'Use lowercase letters, digits, "_" or "-".',
        ),
        'Board creation cancelled.',
    )

    generate_base = ask(
        questionary.confirm('Generate a new base board?', default=True),
        'Board creation cancelled.',
    )

    if generate_base:
        base_name = f'{board_name}_base'
    else:
        base_name = ask(
            questionary.text(
                'Input existing base name',
                validate=lambda text: True if (text or '').strip() else 'Enter existing base name or path.',
            ),
            'Board creation cancelled.',
        )

    summary = build_confirmation_summary(
        series=series,
        chip_model=chip_model,
        storage_type=storage_type,
        storage_port=storage_port,
        storage_size_mb=storage_size_mb,
        board_name=board_name,
        base_name=base_name,
        generate_base=generate_base,
        storage_pinmux_type=storage_pinmux_type,
        nand_block_size_kb=nand_block_size_kb,
        with_acpu=with_acpu,
    )

    confirmed = ask(
        questionary.confirm('Review new board configuration:\n\n' + '\n'.join(summary) + '\n\nCreate board?', default=True),
        'Board creation cancelled.',
    )
    if not confirmed:
        raise FatalError('Board creation cancelled.')

    config: Dict[str, Any] = {
        'chip_series': series,
        'chip_model': chip_model,
        'storage': {
            'type': storage_type,
        },
        'board_name': board_name,
        'generate_base': generate_base,
        'base_name': base_name,
    }
    if storage_size_mb is not None:
        config['storage']['size_mb'] = storage_size_mb
    if storage_pinmux_type is not None:
        config['storage']['pinmux_type'] = storage_pinmux_type
    if nand_block_size_kb is not None:
        config['storage']['block_size_kb'] = nand_block_size_kb
    if storage_port is not None:
        config['storage_port'] = storage_port
    if with_acpu:
        config['with_acpu'] = True
    return config


def build_confirmation_summary(
    series: str,
    chip_model: str,
    storage_type: str,
    storage_port: Optional[str],
    storage_size_mb: Optional[int],
    board_name: str,
    base_name: str,
    generate_base: bool,
    storage_pinmux_type: Optional[str] = None,
    nand_block_size_kb: Optional[int] = None,
    with_acpu: bool = False,
) -> List[str]:
    lines = [
        '[Chip]',
        f'  series: {SERIES_LABEL.get(series, series)}',
        f'  chip_model: {chip_model}',
    ]
    if with_acpu:
        lines.append('  with_acpu: true')
    lines.extend([
        '',
        '[Storage]',
        f'  type: {storage_type}',
    ])
    if storage_port:
        lines.append(f'  port: {storage_port}')
    if storage_pinmux_type:
        lines.append(f'  pinmux_type: {storage_pinmux_type}')
        lines.extend(storage_pinmux_detail_lines(storage_port, selected_type=storage_pinmux_type, indent='  '))
    if nand_block_size_kb is not None:
        lines.append(f'  block_size_kb: {nand_block_size_kb}')
    if storage_size_mb is not None:
        lines.append(f'  capacity_mb: {storage_size_mb}')

    lines.extend([
        '',
        '[Board]',
        f'  board_name: {board_name}',
        '',
        '[Base board]',
        f'  mode: {"create new base board" if generate_base else "reuse existing base board"}',
        f'  base_name_or_path: {base_name}',
    ])

    return lines


def storage_pinmux_detail_lines(
    storage_port: Optional[str],
    selected_type: Optional[str] = None,
    indent: str = '',
) -> List[str]:
    if storage_port is None:
        return []
    details = STORAGE_PINMUX_DETAILS.get(storage_port)
    if details is None:
        return []

    if selected_type is not None:
        pins = details.get(selected_type)
        if pins is None:
            return []
        return [
            f'{indent}pinmux pins:',
            f'{indent}  {selected_type}: {", ".join(_format_pinmux_pin_name(pin) for pin in pins)}',
        ]

    lines = [f'{indent}pinmux pins:']
    for pinmux_type, pins in details.items():
        lines.append(f'{indent}  {pinmux_type}: {", ".join(_format_pinmux_pin_name(pin) for pin in pins)}')
    return lines


def storage_pinmux_choice_description(storage_port: Optional[str], pinmux_type: str) -> str:
    if storage_port is None:
        return ''
    pins = STORAGE_PINMUX_DETAILS.get(storage_port, {}).get(pinmux_type, ())
    return ', '.join(_short_pinmux_pin_name(storage_port, pin) for pin in pins)


def _format_pinmux_pin_name(pin: str) -> str:
    signal, pad = pin.split('=', 1)
    return f'{signal}={pad.removeprefix("PAD_")}'


def _short_pinmux_pin_name(storage_port: str, pin: str) -> str:
    signal, pad = pin.split('=', 1)
    if storage_port == 'mpi4':
        signal = signal.removeprefix('MPI4_')
    elif storage_port == 'sdmmc1':
        signal = signal.removeprefix('SD1_')
    return f'{signal}={pad.removeprefix("PAD_")}'


def normalize_spec(
    raw_config: Dict[str, Any],
    schema_path: Path,
    variants: Dict[str, List[ChipVariant]],
) -> Tuple[Spec, ChipVariant]:
    validate_config_schema(raw_config, schema_path)

    series = str(raw_config.get('chip_series') or '').strip()
    chip_model = str(raw_config.get('chip_model') or '').strip()
    storage = raw_config.get('storage') or {}
    storage_type = str(storage.get('type') or '').strip().lower()
    storage_size_mb = storage.get('size_mb')
    storage_pinmux_type = storage.get('pinmux_type')
    if storage_pinmux_type:
        storage_pinmux_type = str(storage_pinmux_type).strip().lower()
    raw_nand_block_size_kb = storage.get('block_size_kb')
    storage_port = raw_config.get('storage_port')
    if storage_port:
        storage_port = str(storage_port).strip().lower()
    board_name = str(raw_config.get('board_name') or '').strip()
    generate_base = bool(raw_config.get('generate_base'))
    base_name = str(raw_config.get('base_name') or '').strip()
    with_acpu = bool(raw_config.get('with_acpu', False))

    if not BOARD_NAME_RE.fullmatch(board_name):
        raise FatalError(f'Invalid board_name: {board_name}')
    if not generate_base and not base_name:
        raise FatalError('base_name is required when generate_base is false.')

    matching = [item for item in variants.get(series, []) if item.part_number.upper() == chip_model.upper()]
    if not matching:
        raise FatalError(f'Chip model "{chip_model}" does not belong to series {series}')
    variant = matching[0]

    if storage_type == 'none':
        if not variant.none_allowed():
            raise FatalError(f'Chip model "{variant.part_number}" has no non-PSRAM internal storage, so storage.type cannot be "none".')
        storage_size_mb = None
    else:
        if storage_size_mb is None or int(storage_size_mb) <= 0:
            raise FatalError('storage.size_mb must be a positive integer when storage.type is not "none".')
        storage_size_mb = int(storage_size_mb)

    nand_block_size_kb: Optional[int] = None
    if storage_type == 'nand':
        nand_block_size_kb = (
            NAND_BLOCK_SIZE_KB_DEFAULT
            if raw_nand_block_size_kb is None
            else int(raw_nand_block_size_kb)
        )
        if nand_block_size_kb not in NAND_BLOCK_SIZE_KB_CHOICES:
            allowed = '|'.join(str(value) for value in NAND_BLOCK_SIZE_KB_CHOICES)
            raise FatalError(f'storage.block_size_kb must be {allowed} when storage.type is "nand".')
    elif raw_nand_block_size_kb is not None:
        raise FatalError('storage.block_size_kb is only supported when storage.type is "nand".')

    if storage_type == 'none':
        if storage_port is not None:
            raise FatalError('storage_port is not supported when storage.type is "none".')
        if storage_pinmux_type is not None:
            raise FatalError('storage.pinmux_type is not supported when storage.type is "none".')
    elif series == '58':
        if storage_type in ('nor', 'nand') and storage_port not in ('mpi3', 'mpi4'):
            raise FatalError('Series 58 requires storage_port=mpi3|mpi4 for nor/nand external storage.')
        if storage_type == 'sdmmc':
            if storage_port is None:
                storage_port = default_sdmmc_storage_port(series)
            elif storage_port not in ('sdmmc1', 'sdmmc2'):
                raise FatalError('storage_port must be sdmmc1|sdmmc2 when storage.type is "sdmmc".')
        elif storage_type not in ('nor', 'nand'):
            storage_port = None

        if storage_port in ('mpi4', 'sdmmc1'):
            if storage_pinmux_type not in ('type0', 'type1'):
                raise FatalError(f'Series 58 storage_port={storage_port} requires storage.pinmux_type=type0|type1.')
        elif storage_pinmux_type is not None:
            raise FatalError('storage.pinmux_type is only supported for series 58 storage_port=mpi4|sdmmc1.')
    elif series == '57':
        if storage_type in ('nor', 'nand'):
            allowed_ports = ALLOWED_STORAGE_PORTS[series][storage_type]
            if storage_port is None:
                storage_port = DEFAULT_NOR_NAND_PORT[series]
            elif storage_port not in allowed_ports:
                allowed = '|'.join(allowed_ports)
                raise FatalError(f'Series 57 requires storage_port={allowed} for nor/nand external storage.')
        elif storage_type == 'sdmmc':
            if storage_port is None:
                storage_port = default_sdmmc_storage_port(series)
            elif storage_port not in allowed_sdmmc_storage_ports(series):
                allowed = '|'.join(allowed_sdmmc_storage_ports(series))
                raise FatalError(f'Series 57 requires storage_port={allowed} when storage.type is "sdmmc".')
        else:
            storage_port = None

        if storage_pinmux_type is not None:
            raise FatalError('storage.pinmux_type is only supported for series 58 storage_port=mpi4|sdmmc1.')
    elif storage_type == 'sdmmc':
        if storage_port is None:
            storage_port = default_sdmmc_storage_port(series)
        elif storage_port not in allowed_sdmmc_storage_ports(series):
            allowed = '|'.join(allowed_sdmmc_storage_ports(series))
            raise FatalError(f'Series {series} requires storage_port={allowed} when storage.type is "sdmmc".')
        if storage_pinmux_type is not None:
            raise FatalError('storage.pinmux_type is only supported for series 58 storage_port=mpi4|sdmmc1.')
    else:
        storage_port = None
        if storage_pinmux_type is not None:
            raise FatalError('storage.pinmux_type is only supported for series 58 storage_port=mpi4|sdmmc1.')

    psram = variant.first_psram()
    if storage_type in ('nand', 'sdmmc') and psram is None:
        raise FatalError(f'Chip model "{variant.part_number}" needs SiP PSRAM to use {storage_type}.')
    if series == '58' and psram is None:
        raise FatalError(f'Chip model "{variant.part_number}" is not compatible with the fixed 58-series template because it has no SiP PSRAM.')

    if with_acpu:
        if series != '57':
            raise FatalError('with_acpu is only supported for series 57.')
        if storage_type == 'none':
            raise FatalError(
                'with_acpu is not supported when storage.type is "none": '
                'the 57-series none layout has no free 0x40000-byte region between main and fs_region.'
            )
        if psram is None:
            raise FatalError(f'Chip model "{variant.part_number}" needs SiP PSRAM to use with_acpu.')

    if generate_base:
        base_name = f'{board_name}_base'

    spec = Spec(
        series=series,
        chip_model=variant.part_number,
        storage_type=storage_type,
        storage_size_mb=storage_size_mb,
        storage_port=storage_port,
        board_name=board_name,
        base_name=base_name,
        generate_base=generate_base,
        storage_pinmux_type=storage_pinmux_type,
        nand_block_size_kb=nand_block_size_kb,
        with_acpu=with_acpu,
    )
    return spec, variant


def validate_config_schema(raw_config: Dict[str, Any], schema_path: Path) -> None:
    try:
        import jsonschema
    except ImportError as e:
        raise FatalError('You need to install jsonschema package to use new-board JSON validation') from e

    with open(schema_path, encoding='utf-8') as f:
        schema = json.load(f)

    validator = jsonschema.Draft202012Validator(schema)
    errors = sorted(validator.iter_errors(raw_config), key=lambda err: list(err.absolute_path))
    if not errors:
        return

    first = errors[0]
    location = '.'.join(str(part) for part in first.absolute_path)
    if location:
        raise FatalError(f'Invalid board config at "{location}": {first.message}')
    raise FatalError(f'Invalid board config: {first.message}')


def render_board_files(
    spec: Spec,
    variant: ChipVariant,
    sdk_root: Path,
    output_root: Path,
    assets_root: Path,
) -> Dict[Path, str]:
    output_root = output_root.resolve()
    board_dir = (output_root / spec.board_name).resolve()
    base_dir = resolve_base_dir(output_root, spec)

    if board_dir.exists():
        raise FatalError(f'Board directory already exists: {board_dir}')
    if spec.generate_base and base_dir.exists():
        raise FatalError(f'Base directory already exists: {base_dir}')
    if not spec.generate_base and not base_dir.exists():
        raise FatalError(f'Base directory not found: {base_dir}')

    if spec.generate_base:
        sample_base_dir = sdk_root / SAMPLE_BASE[spec.series]
        base_kind = 'base'
    else:
        sample_base_dir = None
        base_kind = detect_existing_base_kind(base_dir)

    try:
        from jinja2 import Environment, FileSystemLoader, StrictUndefined
    except ImportError as e:
        raise FatalError('You need to install Jinja2 package to use new-board') from e

    env = Environment(
        loader=FileSystemLoader(str(assets_root / 'templates')),
        trim_blocks=False,
        lstrip_blocks=False,
        keep_trailing_newline=True,
        undefined=StrictUndefined,
    )

    board_symbol = board_config_symbol(spec.board_name)
    chip_family = FAMILY_CHIP[spec.series]
    package_name = chip_package_prefix(spec.chip_model) if spec.series == '58' else None
    part_symbol = f'SOC_{spec.chip_model.upper()}' if spec.series == '57' else None
    template_prefix = f'{spec.series}/'

    memory_entries = build_external_memory_entries(spec)
    partitions = build_partitions(spec, variant)
    base_reference_files = {
        'kconfig': base_dir / 'Kconfig.board',
        'sconscript': (base_dir / 'script' / 'SConscript') if base_kind == 'script' else (base_dir / 'SConscript.base'),
    }
    use_absolute_base_reference = not spec.generate_base and Path(spec.base_name).expanduser().is_absolute()
    if use_absolute_base_reference:
        base_reference_paths = {name: path.as_posix() for name, path in base_reference_files.items()}
    else:
        base_reference_paths = {
            name: Path(os.path.relpath(path, start=board_dir)).as_posix()
            for name, path in base_reference_files.items()
        }

    rendered: Dict[Path, str] = {}

    rendered[board_dir / 'Kconfig.board'] = render_template(env, template_prefix + 'Kconfig.board.jinja2', {
        'base_kconfig_directive': 'source' if use_absolute_base_reference else 'rsource',
        'base_kconfig_path': base_reference_paths['kconfig'],
    })
    rendered[board_dir / 'SConscript'] = render_template(env, template_prefix + 'SConscript.jinja2', {
        'board_symbol': board_symbol,
        'base_sconscript_path': base_reference_paths['sconscript'],
    })
    rendered[board_dir / 'ptab.yaml'] = render_ptab_yaml(
        board_name=spec.board_name,
        chip_model=spec.chip_model,
        memory_entries=memory_entries,
        partitions=partitions,
    )

    for core_name, core_select, extra_select in core_definitions(spec.series):
        core_dir = board_dir / core_name.lower()
        core_template_prefix = template_prefix + f'{core_name.lower()}/'
        rendered[core_dir / 'Kconfig.board'] = render_template(env, core_template_prefix + 'Kconfig.board.jinja2', {
            'board_symbol': board_symbol,
            'soc_symbol': SOC_SYMBOL[spec.series],
            'part_symbol': part_symbol,
            'core_select': core_select,
            'extra_select': extra_select,
        })

        if core_name == 'HCPU':
            rendered[core_dir / 'board.conf'] = render_template(
                env,
                core_template_prefix + 'board.conf.jinja2',
                build_hcpu_board_conf_context(spec, variant),
            )
            rendered[core_dir / 'rtconfig.py'] = render_template(env, core_template_prefix + 'rtconfig.py.jinja2', {
                'jlink_device': choose_hcpu_jlink_device(spec),
                'chip_family': chip_family,
                'package_name': package_name,
            })
        else:
            rendered[core_dir / 'board.conf'] = render_template(
                env,
                core_template_prefix + 'board.conf.jinja2',
                {},
            )
            rendered[core_dir / 'rtconfig.py'] = render_template(env, core_template_prefix + 'rtconfig.py.jinja2', {
                'target_name': 'lcpu' if core_name == 'LCPU' else 'acpu',
                'core_name': core_name,
                'chip_family': chip_family,
            })

    if spec.generate_base:
        if not sample_base_dir.exists():
            raise FatalError(f'Base template directory not found: {sample_base_dir}')

        for name in ['Kconfig.board', 'board.h', 'bsp_board.h', 'bsp_init.c', 'bsp_lcd_tp.c', 'bsp_pinmux.c', 'bsp_power.c']:
            source_path = sample_base_dir / name
            if not source_path.exists():
                raise FatalError(f'Base template file not found: {source_path}')
            if use_new_board_base_template(spec, name):
                rendered[base_dir / name] = render_template(env, template_prefix + f'base/{name}.jinja2', {
                    'base_content': source_path.read_text(encoding='utf-8'),
                    'storage_pinmux_type': spec.storage_pinmux_type,
                })
            else:
                rendered[base_dir / name] = env.from_string(source_path.read_text(encoding='utf-8')).render()

        rendered[base_dir / 'SConscript.base'] = render_template(env, template_prefix + 'base/SConscript.base.jinja2', {})

    check_render_targets(rendered, board_dir, base_dir if spec.generate_base else None)
    return rendered


def resolve_base_dir(output_root: Path, spec: Spec) -> Path:
    if spec.generate_base:
        return (output_root / spec.base_name).resolve()

    base_path = Path(spec.base_name).expanduser()
    if base_path.is_absolute():
        return base_path.resolve()
    return (output_root / base_path).resolve()


def write_rendered_files(rendered: Dict[Path, str]) -> None:
    for path, content in rendered.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding='utf-8')


def detect_existing_base_kind(base_dir: Path) -> str:
    if (base_dir / 'script' / 'SConscript').exists():
        return 'script'
    if (base_dir / 'SConscript.base').exists():
        return 'base'
    raise FatalError(f'Base board "{base_dir.name}" has neither script/SConscript nor SConscript.base.')


def check_render_targets(rendered: Dict[Path, str], board_dir: Path, base_dir: Optional[Path]) -> None:
    for path in rendered:
        if path.exists():
            raise FatalError(f'Target file already exists: {path}')
        if board_dir not in path.parents and path != board_dir and (base_dir is None or (base_dir not in path.parents and path != base_dir)):
            raise FatalError(f'Refusing to write outside board directories: {path}')


def render_template(env: Any, template_name: str, context: Dict[str, Any]) -> str:
    return env.get_template(template_name).render(**context)


def use_new_board_base_template(spec: Spec, filename: str) -> bool:
    if spec.series != '58' or filename not in ('bsp_pinmux.c', 'bsp_power.c'):
        return False
    if spec.storage_pinmux_type not in ('type0', 'type1'):
        return False
    storage_region = storage_region_for_spec(spec)
    if filename == 'bsp_pinmux.c':
        return storage_region in ('mpi4', 'sdmmc1')
    return storage_region in ('mpi4', 'sdmmc1') and spec.storage_pinmux_type == 'type0'


def render_ptab_yaml(
    board_name: str,
    chip_model: str,
    memory_entries: List[Dict[str, Any]],
    partitions: List[Partition],
) -> str:
    data: Dict[str, Any] = {
        'version': 3,
        'chip': chip_model,
    }
    if memory_entries:
        data['memory'] = memory_entries

    partition_items: List[Dict[str, Any]] = []
    for part in partitions:
        item: Dict[str, Any] = {
            'name': part.name,
            'type': part.part_type,
        }
        if part.subtype:
            item['subtype'] = part.subtype
        item['region'] = part.region
        item['offset'] = HexInt(part.offset)
        item['size'] = HexInt(part.size)
        if part.core:
            item['core'] = part.core
        if part.aliases:
            item['aliases'] = list(part.aliases)
        if part.exec_region:
            item['exec'] = {
                'region': part.exec_region,
                'offset': HexInt(part.exec_offset or 0),
            }
        if part.attrs:
            item['attrs'] = part.attrs
        partition_items.append(item)

    data['partitions'] = partition_items
    header = f'# ptab v3 - Partition Table\n# Board: {board_name}\n# Chip: {chip_model}\n\n'
    return header + yaml.dump(data, Dumper=PtabYamlDumper, sort_keys=False, allow_unicode=True)


def build_external_memory_entries(spec: Spec) -> List[Dict[str, Any]]:
    if spec.storage_type == 'none':
        return []
    storage_region = storage_region_for_spec(spec)
    memory_type = 'sd' if spec.storage_type == 'sdmmc' else spec.storage_type
    memory_entry: Dict[str, Any] = {
        'type': memory_type,
        'size': spec.storage_size_mb * MB,
    }
    if spec.storage_type == 'nand':
        memory_entry['block_size'] = HexInt(nand_block_size_bytes(spec))
    if spec.storage_type == 'sdmmc':
        memory_entry = {'sdmmc': storage_region, **memory_entry}
    else:
        memory_entry = {'mpi': storage_region, **memory_entry}
    return [memory_entry]


def build_partitions(spec: Spec, variant: ChipVariant) -> List[Partition]:
    if spec.storage_type == 'none':
        return build_none_partitions(spec, variant)

    storage_region = storage_region_for_spec(spec)
    psram = variant.first_psram()
    psram_region = psram_alias(psram)
    total_size = spec.storage_size_mb * MB if spec.storage_size_mb is not None else 0
    app_exec_region = None
    app_attrs = None
    if spec.storage_type in ('nand', 'sdmmc'):
        app_exec_region = psram_region
        if spec.series == '56':
            app_attrs = boot_psram_attrs(variant)
    elif spec.series == '58':
        app_exec_region = psram_region

    return build_fixed_partitions(
        spec=spec,
        variant=variant,
        region=storage_region,
        total_size=total_size,
        exec_region=app_exec_region,
        attrs=app_attrs,
    )


def build_none_partitions(spec: Spec, variant: ChipVariant) -> List[Partition]:
    internal_storage = variant.first_non_psram()
    if internal_storage is None or internal_storage.mem_type != 'nor':
        raise FatalError(f'Chip model "{variant.part_number}" does not expose supported internal NOR storage for storage.type=none.')

    psram = variant.first_psram()
    app_exec_region = psram_alias(psram) if psram is not None and spec.series in ('56', '58') else None
    return build_none_storage_partitions(
        spec=spec,
        region=internal_storage.mpi,
        total_size=internal_storage.size_bytes,
        exec_region=app_exec_region,
        attrs=None,
    )


def build_fixed_partitions(
    spec: Spec,
    variant: ChipVariant,
    region: str,
    total_size: int,
    exec_region: Optional[str],
    attrs: Optional[Dict[str, Any]],
) -> List[Partition]:
    layout = FIXED_PARTITION_LAYOUTS.get((spec.series, spec.storage_type))
    if layout is None:
        raise FatalError(f'No fixed partition layout for series={spec.series} storage={spec.storage_type}')

    alignment = storage_offset_alignment(spec)
    partitions = build_reserved_storage_partitions(spec, region)
    min_offset = max((part.offset + part.size for part in partitions), default=0)
    min_offset = max(min_offset, minimum_fixed_partition_offset(spec))
    aligned_layout = align_fixed_partition_layout(layout, alignment, min_offset=min_offset)

    acpu_exec: Optional[Tuple[str, int]] = None
    if spec.with_acpu:
        app_offset, app_size = aligned_layout['hcpu_flash_code']
        layout_with_acpu = dict(layout)
        layout_with_acpu['acpu'] = (acpu_storage_offset(spec, app_offset, app_size), ACPU_PARTITION_SIZE)
        aligned_layout = align_fixed_partition_layout(layout_with_acpu, alignment, min_offset=min_offset)
        acpu_exec = acpu_exec_location(spec, variant, app_size)

    max_end = max(offset + size for offset, size in aligned_layout.values())
    if total_size < max_end:
        raise FatalError(
            f'Target storage is too small for the fixed PTAB layout: need at least {max_end} bytes, got {total_size} bytes.'
        )

    app_offset, app_size = aligned_layout['hcpu_flash_code']
    fs_offset, fs_size = aligned_layout['fs_region']
    dfu_offset, _ = aligned_layout['dfu']
    ble_offset, _ = aligned_layout['ble']

    partitions.append(
        Partition(
            'hcpu_flash_code',
            'app',
            region,
            app_offset,
            app_size,
            subtype='factory',
            core='HCPU',
            exec_region=exec_region,
            exec_offset=0 if exec_region else None,
            attrs=attrs,
        )
    )
    if spec.with_acpu:
        acpu_offset, acpu_size = aligned_layout['acpu']
        assert acpu_exec is not None
        acpu_exec_region, acpu_exec_offset = acpu_exec
        partitions.append(
            Partition(
                'acpu',
                'app',
                region,
                acpu_offset,
                acpu_size,
                subtype='factory',
                core='ACPU',
                aliases=('ACPU_CODE_LOAD_REGION1', 'APP_ACPU_CODE'),
                exec_region=acpu_exec_region,
                exec_offset=acpu_exec_offset,
            )
        )
    partitions.extend([
        Partition('fs_region', 'data', region, fs_offset, fs_size, subtype='filesystem'),
        Partition('dfu', 'data', region, dfu_offset, KV_PART_SIZE, subtype='flashdb_kv'),
        Partition('ble', 'data', region, ble_offset, KV_PART_SIZE, subtype='flashdb_kv'),
    ])
    if spec.with_acpu:
        # RAM window for the ACPU core. It is not part of the storage layout,
        # but ptab.h must expose ACPU_RAM_DATA_START_ADDR/_SIZE for the ACPU
        # board header (HEAP_END). Must never be emitted without with_acpu.
        partitions.append(
            Partition(
                'acpu_ram_data',
                'data',
                'hpsys_ram',
                ACPU_RAM_DATA_OFFSET,
                ACPU_RAM_DATA_SIZE,
                subtype='ram',
                core='ACPU',
                aliases=('ACPU_RAM_DATA',),
            )
        )
    return partitions


def build_reserved_storage_partitions(spec: Spec, region: str) -> List[Partition]:
    if spec.series != '52':
        return []

    if spec.storage_type == 'nand':
        block_size = nand_block_size_bytes(spec)
        return [
            Partition(
                'factory_data',
                'data',
                region,
                2 * block_size,
                block_size,
                subtype='raw',
                aliases=('FACTORY_DATA',),
            ),
        ]

    if spec.storage_type == 'sdmmc':
        return [
            Partition(
                'mbr',
                'data',
                region,
                0x00000000,
                0x00001000,
                subtype='raw',
                aliases=('MBR',),
            ),
            Partition(
                'factory_data',
                'data',
                region,
                0x00041000,
                0x00020000,
                subtype='raw',
                aliases=('FACTORY_DATA',),
            ),
        ]

    return []


def acpu_storage_offset(spec: Spec, app_offset: int, app_size: int) -> int:
    if spec.storage_type == 'sdmmc':
        # Existing 57 SD boards keep the ACPU image 7 MB past the app start
        # instead of directly after the (3 MB) HCPU app region.
        return app_offset + SF32LB57_SD_ACPU_OFFSET_FROM_APP
    return app_offset + app_size


def acpu_exec_location(spec: Spec, variant: ChipVariant, app_size: int) -> Tuple[str, int]:
    psram = variant.first_psram()
    if psram is None:
        raise FatalError(f'Chip model "{variant.part_number}" needs SiP PSRAM to place the ACPU exec region.')

    psram2 = next(
        (
            entry
            for entry in sorted(variant.memory, key=_memory_sort_key)
            if entry.mem_type == 'psram' and entry.mpi == 'mpi2'
        ),
        None,
    )
    if psram2 is not None:
        # Dual-PSRAM parts (e.g. SF32LB57EYBBN6) put the ACPU image in psram2.
        return psram_alias(psram2), 0
    if spec.storage_type == 'nor':
        return psram_alias(psram), 0
    # NAND/SD execute the HCPU app from the first PSRAM, so the ACPU image
    # follows it in the same region.
    return psram_alias(psram), app_size


def build_none_storage_partitions(
    spec: Spec,
    region: str,
    total_size: int,
    exec_region: Optional[str],
    attrs: Optional[Dict[str, Any]],
) -> List[Partition]:
    if spec.with_acpu:
        raise FatalError(
            'with_acpu is not supported when storage.type is "none": '
            'the 57-series none layout has no free 0x40000-byte region between main and fs_region.'
        )

    layout = FIXED_PARTITION_LAYOUTS.get((spec.series, 'none'))
    if layout is None:
        raise FatalError(f'No fixed partition layout for series={spec.series} storage=none')

    app_offset, default_app_size = layout['hcpu_flash_code']
    app_size = max(default_app_size, MIN_FACTORY_APP_PART_SIZE)
    ble_offset = total_size - KV_PART_SIZE
    dfu_offset = ble_offset - KV_PART_SIZE
    fs_offset = app_offset + app_size
    fs_size = dfu_offset - fs_offset

    if fs_size < 0:
        min_size = app_offset + app_size + (2 * KV_PART_SIZE)
        raise FatalError(
            f'Target internal NOR is too small for the default PTAB layout: '
            f'need at least {min_size} bytes to reserve {MIN_FACTORY_APP_PART_SIZE} bytes for app/factory, got {total_size} bytes.'
        )

    return [
        Partition(
            'hcpu_flash_code',
            'app',
            region,
            app_offset,
            app_size,
            subtype='factory',
            core='HCPU',
            exec_region=exec_region,
            exec_offset=0 if exec_region else None,
            attrs=attrs,
        ),
        Partition('fs_region', 'data', region, fs_offset, fs_size, subtype='filesystem'),
        Partition('dfu', 'data', region, dfu_offset, KV_PART_SIZE, subtype='flashdb_kv'),
        Partition('ble', 'data', region, ble_offset, KV_PART_SIZE, subtype='flashdb_kv'),
    ]


def build_hcpu_board_conf_context(spec: Spec, variant: ChipVariant) -> Dict[str, Any]:
    storage_region = storage_region_for_spec(spec) if spec.storage_type == 'sdmmc' else None
    use_sdmmc1 = spec.storage_type == 'sdmmc' and storage_region == 'sdmmc1'
    use_sdmmc2 = spec.storage_type == 'sdmmc' and storage_region == 'sdmmc2'
    return {
        'enable_vddsip_ldo18': needs_52_vddsip_ldo18(variant),
        'memory': build_memory_config_context(spec, variant),
        'use_sdmmc': spec.storage_type == 'sdmmc',
        'use_sdhci1': use_sdmmc1,
        'use_sdhci2': use_sdmmc2,
        'sd_max_freq': spec.series in ('56', '58') and storage_region == 'sdmmc1',
        'sd2_max_freq': spec.series == '58' and storage_region == 'sdmmc2',
        'use_sdmmc1': use_sdmmc1,
        'use_sdmmc2': use_sdmmc2,
        'rt_using_mtd_nand': spec.storage_type == 'nand',
        'has_psram_cache_wb': variant.first_psram() is not None,
    }


def build_memory_config_context(spec: Spec, variant: ChipVariant) -> Dict[str, Dict[str, Any]]:
    context: Dict[str, Dict[str, Any]] = {
        'mpi1': {'enabled': False, 'mode': None, 'size_mb': None},
        'mpi2': {'enabled': False, 'mode': None, 'size_mb': None},
        'mpi3': {'enabled': False, 'mode': None, 'size_mb': None},
        'mpi4': {'enabled': False, 'mode': None, 'size_mb': None},
        'mpi5': {'enabled': False, 'mode': None, 'size_mb': None},
    }

    def enable(entry: MemoryEntry) -> None:
        mpi = entry.mpi
        if mpi not in context:
            return
        context[mpi] = {
            'enabled': True,
            'mode': mpi_mode_for_memory(spec.series, variant, entry.mpi, entry.mem_type),
            'size_mb': entry.size_mb,
        }

    included_mpis: set[str] = set()
    for entry in sorted(variant.memory, key=_memory_sort_key):
        if spec.series == '57':
            # SiP memory is fully described by the part number in
            # soc/sf32lb57x/Kconfig.<part>, which sets BSP_ENABLE_MPI<n>,
            # BSP_MPI<n>_MODE_<m> and BSP_QSPI<n>_MEM_SIZE. Do not duplicate
            # them in board.conf.
            included_mpis.add(entry.mpi)
            continue
        if should_enable_internal_memory(spec, entry):
            enable(entry)
            included_mpis.add(entry.mpi)

    external_entry = external_storage_entry(spec)
    if external_entry and external_entry.mpi not in included_mpis:
        enable(external_entry)

    return context


def should_enable_internal_memory(spec: Spec, entry: MemoryEntry) -> bool:
    if entry.mem_type == 'psram':
        return True
    if spec.storage_type == 'none':
        internal = storage_region_for_spec(spec, none_region=entry.mpi)
        return entry.mpi == internal
    if spec.series == '52' and entry.mpi == 'mpi1' and entry.mem_type == 'nor':
        return True
    return False


def external_storage_entry(spec: Spec) -> Optional[MemoryEntry]:
    if spec.storage_type == 'none' or spec.storage_size_mb is None:
        return None
    if spec.storage_type == 'sdmmc':
        return None
    return MemoryEntry(storage_region_for_spec(spec), spec.storage_type, spec.storage_size_mb * MB)


def enable_mpi_lines(series: str, mpi: str, mem_type: str, size_mb: int, variant: ChipVariant) -> List[str]:
    lines = [f'CONFIG_BSP_ENABLE_{mpi.upper()}=y']

    mode = mpi_mode_for_memory(series, variant, mpi, mem_type)
    if mode is not None:
        lines.append(f'CONFIG_BSP_{mpi.upper()}_MODE_{mode}=y')
    lines.append(f'CONFIG_BSP_QSPI{mpi_number(mpi)}_MEM_SIZE={size_mb}')
    return lines


def mpi_mode_for_memory(series: str, variant: ChipVariant, mpi: str, mem_type: str) -> Optional[int]:
    if mem_type == 'nor':
        return 0
    if mem_type == 'nand':
        return 1
    if mem_type != 'psram':
        return None

    if series == '52':
        if variant.chip_dir == 'SF32LB52x':
            return 3
        if variant.part_number.startswith('SF32LB52D'):
            return 5
        return 6
    if series == '56':
        if mpi == 'mpi1':
            return 3
        if mpi == 'mpi2':
            return 5
    if series == '57':
        # SiP PSRAM mode comes from soc/sf32lb57x/Kconfig.<part>; return None
        # so no board.conf override is emitted.
        return None
    if series == '58':
        return 4
    return None


def core_definitions(series: str) -> List[Tuple[str, str, Optional[str]]]:
    defs = [('HCPU', 'BF0_HCPU', None), ('LCPU', 'BF0_LCPU', None)]
    if series in ('57', '58'):
        defs.append(('ACPU', 'BF0_ACPU', 'BF0_HCPU'))
    return defs


def choose_hcpu_jlink_device(spec: Spec) -> str:
    if spec.series == '52':
        if spec.storage_type == 'nand':
            return 'SF32LB52X_NAND'
        if spec.storage_type == 'sdmmc':
            return 'SF32LB52X_SD'
        return 'SF32LB52X_NOR'

    if spec.series == '56':
        if spec.storage_type == 'nand':
            return 'SF32LB56X_NAND'
        if spec.storage_type == 'sdmmc':
            return 'SF32LB56X_SD'
        return 'SF32LB56X'

    if spec.series == '57':
        if spec.storage_type == 'nand':
            return 'SF32LB57X_NAND'
        if spec.storage_type == 'sdmmc':
            return 'SF32LB57X_SD'
        return 'SF32LB57X_NOR'

    if spec.storage_type == 'nand':
        device = 'SF32LB58X_NAND'
    elif spec.storage_type == 'sdmmc':
        device = 'SF32LB58X_SD'
    else:
        device = 'SF32LB58X'
    if spec.storage_type == 'sdmmc' and spec.storage_pinmux_type == 'type1':
        device += '_TYPE1'
    return device


def board_config_symbol(board_name: str) -> str:
    return 'BSP_USING_BOARD_' + re.sub(r'[^A-Z0-9]', '_', board_name.upper())


def chip_package_prefix(chip_model: str) -> Optional[str]:
    match = PACKAGE_RE.match(chip_model)
    return match.group(1) if match else None


def psram_alias(entry: Optional[MemoryEntry]) -> Optional[str]:
    if entry is None:
        return None
    return f'psram{mpi_number(entry.mpi)}'


def boot_psram_attrs(variant: ChipVariant) -> Optional[Dict[str, Any]]:
    psram = variant.first_psram()
    if psram is None:
        return None
    return {
        'PSRAM_BL_MODE': mpi_mode_for_memory(variant.series, variant, psram.mpi, psram.mem_type),
        'PSRAM_BL_SIZE': psram.size_mb,
        'PSRAM_BL_MPI': mpi_number(psram.mpi),
    }


def storage_region_for_spec(spec: Spec, none_region: Optional[str] = None) -> str:
    if spec.storage_type == 'none':
        if none_region:
            return none_region
        raise FatalError('none storage region requires an explicit internal storage region.')
    if spec.storage_type == 'sdmmc':
        if spec.storage_port:
            if spec.storage_port not in allowed_sdmmc_storage_ports(spec.series):
                allowed = '|'.join(allowed_sdmmc_storage_ports(spec.series))
                raise FatalError(f'Series {spec.series} requires storage_port={allowed} when storage.type is "sdmmc".')
            return spec.storage_port
        return default_sdmmc_storage_port(spec.series)
    if spec.series == '52':
        return 'mpi2'
    if spec.series == '56':
        return 'mpi3'
    return str(spec.storage_port)


def default_sdmmc_storage_port(series: str) -> str:
    return 'sdmmc2' if series == '58' else 'sdmmc1'


def allowed_sdmmc_storage_ports(series: str) -> Tuple[str, ...]:
    if series == '52':
        return ('sdmmc1',)
    return ('sdmmc1', 'sdmmc2')


def nand_block_size_bytes(spec: Spec) -> int:
    block_size_kb = spec.nand_block_size_kb or NAND_BLOCK_SIZE_KB_DEFAULT
    return int(block_size_kb) * KB


def storage_offset_alignment(spec: Spec) -> int:
    if spec.storage_type == 'nand':
        return nand_block_size_bytes(spec)
    return STORAGE_OFFSET_ALIGNMENT.get(spec.storage_type, 1)


def minimum_fixed_partition_offset(spec: Spec) -> int:
    if spec.series == '52' and spec.storage_type == 'nand':
        bootloader_end = (
            SF32LB52_NAND_BOOTLOADER_BLOCK_INDEX * nand_block_size_bytes(spec)
            + SF32LB52_NAND_BOOTLOADER_SIZE
        )
        return align_up(bootloader_end, storage_offset_alignment(spec))
    if spec.series == '57' and spec.storage_type == 'nand':
        bootloader_end = (
            SF32LB57_NAND_BOOTLOADER_BLOCK_INDEX * nand_block_size_bytes(spec)
            + SF32LB57_NAND_BOOTLOADER_SIZE
        )
        return align_up(bootloader_end, storage_offset_alignment(spec))
    return 0


def align_fixed_partition_layout(
    layout: Dict[str, Tuple[int, int]],
    alignment: int,
    min_offset: int = 0,
) -> Dict[str, Tuple[int, int]]:
    if alignment <= 1:
        return dict(layout)

    aligned: Dict[str, Tuple[int, int]] = {}
    next_offset = min_offset
    for name, (offset, size) in sorted(layout.items(), key=lambda item: item[1][0]):
        aligned_offset = align_up(max(offset, next_offset), alignment)
        aligned[name] = (aligned_offset, size)
        next_offset = aligned_offset + size
    return aligned


def rt_using_mtd_line(storage_type: str) -> str:
    if storage_type == 'nand':
        return 'CONFIG_RT_USING_MTD_NAND=y'
    return 'CONFIG_RT_USING_MTD_NOR=y'


def needs_52_vddsip_ldo18(variant: ChipVariant) -> bool:
    return variant.chip_dir == 'SF32LB52_X' and variant.first_psram() is not None


def _memory_summary_suffix(variant: ChipVariant) -> str:
    if not variant.memory:
        return ''
    parts = [
        f'{entry.mpi}:{entry.mem_type}:{_format_memory_size(entry.size_bytes)}'
        for entry in sorted(variant.memory, key=_memory_sort_key)
    ]
    return f' [{", ".join(parts)}]'


def _format_memory_size(size_bytes: int) -> str:
    if size_bytes >= MB and size_bytes % MB == 0:
        return f'{size_bytes // MB}MB'
    if size_bytes >= KB and size_bytes % KB == 0:
        return f'{size_bytes // KB}KB'
    return f'{size_bytes}B'


def _memory_sort_key(entry: MemoryEntry) -> Tuple[int, str]:
    return (mpi_number(entry.mpi), entry.mem_type)


def mpi_number(mpi_name: str) -> int:
    match = re.match(r'^mpi(\d+)$', mpi_name.strip().lower())
    return int(match.group(1)) if match else 0


def hex_u(value: int) -> str:
    return f'0x{value:08X}'


def parse_int(value: Any) -> int:
    if isinstance(value, int):
        return value
    return int(str(value), 0)


def align_down(value: int, alignment: int) -> int:
    return value - (value % alignment)


def align_up(value: int, alignment: int) -> int:
    remainder = value % alignment
    return value if remainder == 0 else value + (alignment - remainder)
