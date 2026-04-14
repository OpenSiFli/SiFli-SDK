#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

"""
sdk_size.py - SiFli SDK Memory Usage Analysis Script

Analyzes GCC linker .map files from a build directory and reports
section sizes and physical memory region usage statistics.

Features:
  - Recursive .map file discovery in build directories
  - Per-region memory usage breakdown with color-coded tables
  - Section-level detail mode (--detail)
  - Rich terminal output with colored tables
  - Plain-text file output (--output)

Usage:
    python sdk_size.py <path> [--detail] [--output <file>] [--no-color]

    <path> can be a build directory (recursively searches for .map files)
    or a single .map file.

Examples:
    python sdk_size.py project/build_sf32lb52-lcd_n16r8_hcpu
    python sdk_size.py project/build_sf32lb52-lcd_n16r8_hcpu --detail
    python sdk_size.py project/build_sf32lb52-lcd_n16r8_hcpu/main.map
    python sdk_size.py project/build_sf32lb58-lcd_a128r32n1_a1_dsi_hcpu -o report.txt
"""

import os
import re
import sys
import logging
import argparse
from dataclasses import dataclass, field
from typing import Optional

logger = logging.getLogger(__name__)

from rich.console import Console
from rich.table import Table
from rich.panel import Panel


# ============================================================================
# Data Structures
# ============================================================================

@dataclass
class MemoryRegion:
    """A memory region from the 'Memory Configuration' table in a .map file."""
    name: str
    origin: int
    length: int
    attributes: str

    @property
    def end(self) -> int:
        return self.origin + self.length

    def contains(self, address: int) -> bool:
        return self.origin <= address < self.end


@dataclass
class Section:
    """A linker output section."""
    name: str
    vma: int            # Virtual Memory Address (runtime address)
    size: int           # Section size in bytes
    lma: Optional[int] = None  # Load Memory Address (may differ from VMA)

    @property
    def has_load_address(self) -> bool:
        return self.lma is not None and self.lma != self.vma


@dataclass
class MapFileInfo:
    """Parsed results from a single .map file."""
    filepath: str
    name: str                                    # Artifact name (main/bootloader/ftab/lcpu...)
    regions: list[MemoryRegion] = field(default_factory=list)
    sections: list[Section] = field(default_factory=list)
    symbols: dict[str, int] = field(default_factory=dict)  # Symbol name -> value


@dataclass
class RegionUsage:
    """Usage statistics for a single memory region."""
    region: MemoryRegion
    used_size: int = 0
    sections: list[Section] = field(default_factory=list)

    @property
    def free_size(self) -> int:
        return max(0, self.region.length - self.used_size)

    @property
    def usage_percent(self) -> float:
        if self.region.length == 0:
            return 0.0
        return (self.used_size / self.region.length) * 100.0


# ============================================================================
# .map File Parser
# ============================================================================

class MapFileParser:
    """GCC linker .map file parser."""

    # Regular expressions
    RE_MEM_REGION = re.compile(
        r'^\s*(\S+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s+(\w+)\s*$'
    )
    RE_SECTION_HEADER = re.compile(
        r'^(\.\S+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)'
        r'(?:\s+load\s+address\s+(0x[0-9a-fA-F]+))?'
    )
    RE_SYMBOL_DEF = re.compile(
        r'^\s+(0x[0-9a-fA-F]+)\s+(\S+)\s*=\s*(.+)'
    )
    RE_SUBSECTION = re.compile(
        r'^\s+(\.\S+)\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s+\S'
    )
    RE_SUBSECTION_NAME_ONLY = re.compile(
        r'^\s+(\.\S+)\s*$'
    )
    RE_SUBSECTION_ADDR = re.compile(
        r'^\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s+\S'
    )
    RE_FILL = re.compile(
        r'^\s+\*fill\*\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)'
    )
    RE_OUTPUT = re.compile(r'^OUTPUT\(')
    RE_CROSS_REF = re.compile(r'^Cross Reference Table\s*$')

    # Section name prefixes to ignore (debug / ARM attribute sections)
    IGNORED_SECTION_PREFIXES = (
        '.debug_', '.comment', '.ARM.attributes', '.ARM.exidx',
        '.ARM.extab', '.stab', '.note',
    )

    # Top-level sections allowed for sub-section splitting.
    # Only these sections will be split when they contain mixed content
    # (e.g., .text containing .rodata sub-sections).
    SPLITTABLE_SECTIONS = {'.text'}

    def __init__(self, filepath: str):
        self.filepath = filepath
        self.name = self._derive_name(filepath)

    @staticmethod
    def _derive_name(filepath: str) -> str:
        """Derive artifact name from file path."""
        basename = os.path.splitext(os.path.basename(filepath))[0]
        return basename

    def parse(self) -> MapFileInfo:
        """Parse the .map file and return a MapFileInfo."""
        info = MapFileInfo(filepath=self.filepath, name=self.name)

        with open(self.filepath, 'r', encoding='utf-8', errors='replace') as f:
            lines = f.readlines()

        self._parse_memory_config(lines, info)
        self._parse_linker_map(lines, info)

        return info

    def _parse_memory_config(self, lines: list[str], info: MapFileInfo):
        """Parse the 'Memory Configuration' table."""
        in_mem_config = False
        header_seen = False

        for line in lines:
            stripped = line.strip()

            if stripped == 'Memory Configuration':
                in_mem_config = True
                continue

            if in_mem_config and not header_seen:
                if stripped.startswith('Name'):
                    header_seen = True
                continue

            if in_mem_config and header_seen:
                if stripped == '' or stripped.startswith('Linker script'):
                    break
                if stripped.startswith('*default*'):
                    break

                m = self.RE_MEM_REGION.match(line)
                if m:
                    region = MemoryRegion(
                        name=m.group(1),
                        origin=int(m.group(2), 16),
                        length=int(m.group(3), 16),
                        attributes=m.group(4),
                    )
                    # Skip zero-length regions
                    if region.length > 0:
                        info.regions.append(region)

    def _parse_linker_map(self, lines: list[str], info: MapFileInfo):
        """Parse the 'Linker script and memory map' section."""
        in_linker_map = False
        i = 0

        while i < len(lines):
            line = lines[i]
            stripped = line.strip()

            # Locate the start of the linker map section
            if stripped == 'Linker script and memory map':
                in_linker_map = True
                i += 1
                continue

            # Stop at end markers
            if in_linker_map and (self.RE_OUTPUT.match(stripped) or
                                   self.RE_CROSS_REF.match(stripped)):
                break

            if not in_linker_map:
                i += 1
                continue

            # Parse symbol definitions
            m = self.RE_SYMBOL_DEF.match(line)
            if m:
                value = int(m.group(1), 16)
                sym_name = m.group(2)
                info.symbols[sym_name] = value
                i += 1
                continue

            # Parse section headers
            m = self.RE_SECTION_HEADER.match(line)
            if m:
                sec_name = m.group(1)

                # Skip ignored sections
                if any(sec_name.startswith(p) for p in self.IGNORED_SECTION_PREFIXES):
                    i += 1
                    continue

                vma = int(m.group(2), 16)
                size = int(m.group(3), 16)
                lma = int(m.group(4), 16) if m.group(4) else None

                # Skip zero-size sections (unused regions)
                if size > 0:
                    # Only scan sub-sections for known splittable top-level
                    # sections (e.g., .text may contain .rodata sub-sections)
                    should_split = sec_name in self.SPLITTABLE_SECTIONS
                    if should_split:
                        i += 1
                        sub_sizes = self._scan_subsections(lines, i, sec_name)
                    else:
                        sub_sizes = {}

                    if sub_sizes:
                        for sub_name, sub_size in sub_sizes.items():
                            if sub_size > 0:
                                info.sections.append(Section(
                                    name=sub_name, vma=vma, size=sub_size, lma=lma,
                                ))
                    else:
                        info.sections.append(Section(
                            name=sec_name, vma=vma, size=size, lma=lma,
                        ))
                        i += 1
                    continue

                i += 1
                continue

            i += 1

        return info

    def _scan_subsections(self, lines: list[str], start: int,
                          parent_name: str) -> dict[str, int]:
        """Scan sub-sections within a top-level section and classify by prefix.

        Returns the total size per category. If all sub-sections belong to the
        same prefix (or there are none), returns an empty dict indicating no
        split is needed. When multiple categories exist (e.g., .text.* and
        .rodata.*), returns a {category_name: total_size} mapping.
        """
        categories: dict[str, int] = {}
        i = start
        pending_sub_name: Optional[str] = None

        while i < len(lines):
            line = lines[i]

            # A new top-level section header (no leading whitespace) ends the scan
            if line and not line[0].isspace() and line[0] == '.':
                break
            # OUTPUT / Cross Reference markers also end the scan
            stripped = line.strip()
            if self.RE_OUTPUT.match(stripped) or self.RE_CROSS_REF.match(stripped):
                break

            # Single-line sub-section: " .subsection  0xADDR  0xSIZE  source.o"
            m = self.RE_SUBSECTION.match(line)
            if m:
                pending_sub_name = None
                sub_name = m.group(1)
                sub_size = int(m.group(3), 16)
                cat = self._subsection_category(sub_name)
                categories[cat] = categories.get(cat, 0) + sub_size
                i += 1
                continue

            # Wrapped sub-section name (first line, name only): " .subsection"
            m = self.RE_SUBSECTION_NAME_ONLY.match(line)
            if m:
                pending_sub_name = m.group(1)
                i += 1
                continue

            # Wrapped sub-section second line (address + size):
            # "          0xADDR  0xSIZE  source.o"
            if pending_sub_name is not None:
                m = self.RE_SUBSECTION_ADDR.match(line)
                if m:
                    sub_size = int(m.group(2), 16)
                    cat = self._subsection_category(pending_sub_name)
                    categories[cat] = categories.get(cat, 0) + sub_size
                    pending_sub_name = None
                    i += 1
                    continue
                pending_sub_name = None

            # *fill* lines are attributed to the most recent category (alignment padding)
            m = self.RE_FILL.match(line)
            if m:
                fill_size = int(m.group(2), 16)
                if categories:
                    last_cat = list(categories.keys())[-1]
                    categories[last_cat] = categories.get(last_cat, 0) + fill_size
                i += 1
                continue

            i += 1

        # Only split when there are at least two distinct well-known categories.
        # Typical case: .text section contains both .text.* and .rodata.* sub-sections.
        # No-split cases: .retm_data with .l1_ret_text_* (all belong to parent),
        #                 .RW_IRAM0 with .bss.* (unrelated prefix).
        well_known = {'.text', '.rodata', '.data', '.bss'}
        meaningful_cats = {c for c in categories if c in well_known}

        # Need at least two different well-known categories to justify splitting
        if len(meaningful_cats) < 2:
            return {}

        # Merge unrecognized categories into the parent section name
        merged: dict[str, int] = {}
        for cat, sz in categories.items():
            key = cat if cat in well_known else parent_name
            merged[key] = merged.get(key, 0) + sz

        return merged

    @staticmethod
    def _subsection_category(sub_name: str) -> str:
        """Extract category prefix from a sub-section name.

        Examples:
            .rodata.LCD_ReadID.str1.1 -> .rodata
            .text.HAL_ADC_Init       -> .text
            .data.rti_fn             -> .data
            .bss.completed.0         -> .bss
        """
        if sub_name.startswith('.'):
            dot2 = sub_name.find('.', 1)
            if dot2 > 0:
                return sub_name[:dot2]
        return sub_name


# ============================================================================
# Memory Usage Analyzer
# ============================================================================

class MemoryAnalyzer:
    """Analyzes parsed .map file data and produces memory usage reports."""

    def __init__(self, map_infos: list[MapFileInfo]):
        self.map_infos = map_infos

    def analyze_single(self, info: MapFileInfo) -> list[RegionUsage]:
        """Analyze memory usage for a single .map file."""
        usages: dict[str, RegionUsage] = {}
        for region in info.regions:
            usages[region.name] = RegionUsage(region=region)

        for section in info.sections:
            # Determine the memory region by VMA address
            vma_region = self._find_region(info.regions, section.vma)
            if vma_region:
                usage = usages[vma_region.name]
                usage.sections.append(section)
                usage.used_size += section.size

            # If LMA differs from VMA, the section also occupies space
            # in the LMA region (flash storage footprint)
            if section.has_load_address:
                lma_region = self._find_region(info.regions, section.lma)
                if lma_region and (vma_region is None or lma_region.name != vma_region.name):
                    usage = usages[lma_region.name]
                    usage.used_size += section.size

        # Sort by origin address, filter out unused regions
        result = sorted(usages.values(), key=lambda u: u.region.origin)
        return [u for u in result if u.used_size > 0 or len(u.sections) > 0]

    def compute_program_size(self, info: MapFileInfo) -> int:
        """Compute the flash/ROM footprint for a single artifact.

        Finds all sections occupying ROM-type (executable/read-only) regions,
        then returns (max_LMA + size) - min_LMA, i.e., the contiguous image size.
        """
        rom_regions = [r for r in info.regions if 'x' in r.attributes]
        if not rom_regions:
            return 0

        # Collect all byte occupancy in ROM regions
        min_addr = None
        max_addr = 0

        for section in info.sections:
            # Determine the flash address for this section
            flash_addr = section.lma if section.lma is not None else section.vma

            # Check if this address falls within a ROM region
            region = self._find_region(rom_regions, flash_addr)
            if region and section.size > 0:
                end_addr = flash_addr + section.size
                if min_addr is None or flash_addr < min_addr:
                    min_addr = flash_addr
                if end_addr > max_addr:
                    max_addr = end_addr

        if min_addr is None:
            return 0

        return max_addr - min_addr

    def compute_total_program_size(self) -> int:
        """Compute the total flash footprint across all artifacts."""
        total = 0
        for info in self.map_infos:
            total += self.compute_program_size(info)
        return total

    def build_global_region_map(self) -> dict[str, RegionUsage]:
        """Build a global memory region usage map by merging all artifacts.

        Since different sub-programs may have different memory configurations
        (names and sizes), the main program's region definitions are used as
        the reference baseline.
        """
        # Use main's region config as the baseline
        main_info = None
        for info in self.map_infos:
            if info.name == 'main':
                main_info = info
                break

        if main_info is None and self.map_infos:
            main_info = self.map_infos[0]

        if main_info is None:
            return {}

        global_usages: dict[str, RegionUsage] = {}
        for region in main_info.regions:
            global_usages[region.name] = RegionUsage(region=region)

        # Merge section usage from all sub-programs
        for info in self.map_infos:
            for section in info.sections:
                # VMA occupancy
                vma_region = self._find_region(main_info.regions, section.vma)
                if vma_region:
                    usage = global_usages[vma_region.name]
                    usage.sections.append(section)
                    usage.used_size += section.size

                # LMA occupancy (only when LMA region differs from VMA region)
                if section.has_load_address:
                    lma_region = self._find_region(main_info.regions, section.lma)
                    if lma_region and (vma_region is None or lma_region.name != vma_region.name):
                        global_usages[lma_region.name].used_size += section.size

        result = {k: v for k, v in global_usages.items()
                  if v.used_size > 0 or len(v.sections) > 0}
        return result

    @staticmethod
    def _find_region(regions: list[MemoryRegion], address: int) -> Optional[MemoryRegion]:
        """Find the memory region containing the given address."""
        for region in regions:
            if region.contains(address):
                return region
        return None


# ============================================================================
# Report Generator (Rich-based)
# ============================================================================

class ReportGenerator:
    """Generates memory usage reports using Rich tables with color and progress bars."""

    # Column widths
    COL_SECTION = 24
    COL_SIZE = 12

    def __init__(self, map_infos: list[MapFileInfo], analyzer: MemoryAnalyzer,
                 console: Console):
        self.map_infos = map_infos
        self.analyzer = analyzer
        self.console = console

    def generate(self, detail: bool = False):
        """Generate and print the report.

        Args:
            detail: Whether to output detailed per-file analysis.
        """
        if detail:
            for info in self.map_infos:
                self._report_single_map(info)

        # Summary report: single file shows its own summary,
        # multiple files show a merged global summary.
        if len(self.map_infos) == 1:
            self._report_single_summary(self.map_infos[0])
        else:
            self._report_global_summary()

    def _report_single_map(self, info: MapFileInfo):
        """Generate a detailed report for a single artifact (--detail mode)."""
        prog_size = self.analyzer.compute_program_size(info)
        usages = self.analyzer.analyze_single(info)

        self.console.print()
        self.console.print(
            f"[bold cyan]\\[{info.name}] Flash Size: {self._fmt_size(prog_size)}[/]"
        )
        self.console.print()
        self._print_rich_table(usages, show_individual_sections=True)
        self.console.print()

    def _report_single_summary(self, info: MapFileInfo):
        """Generate a summary report for single-file mode."""
        prog_size = self.analyzer.compute_program_size(info)
        usages = self.analyzer.analyze_single(info)

        self.console.print()
        self.console.print(
            Panel.fit(
                f"[bold]Total Flash Size: [cyan]{self._fmt_size(prog_size)}[/cyan][/bold]",
                title=f"[bold cyan]Memory Usage Summary ({info.name})[/]",
                border_style="cyan",
            )
        )
        self.console.print()
        self._print_rich_table(usages, show_individual_sections=True)
        self._print_note()

    def _report_global_summary(self):
        """Generate a global summary report (multiple artifacts)."""
        total_size = self.analyzer.compute_total_program_size()
        global_usages = self.analyzer.build_global_region_map()

        usages_sorted = [v for _, v in sorted(global_usages.items(),
                                              key=lambda x: x[1].region.origin)]

        self.console.print()
        self.console.print(
            Panel.fit(
                f"[bold]Total Flash Size: [cyan]{self._fmt_size(total_size)}[/cyan][/bold]",
                title="[bold cyan]Memory Usage Summary[/]",
                border_style="cyan",
            )
        )
        self.console.print()
        self._print_rich_table(usages_sorted, show_individual_sections=False)
        self._print_note()

    def _print_rich_table(self, usages: list[RegionUsage], *,
                          show_individual_sections: bool):
        """Print region/section usage table using Rich."""
        table = Table(
            show_header=True,
            header_style="bold magenta",
            border_style="dim",
            pad_edge=True,
        )
        table.add_column("Region / Section", min_width=self.COL_SECTION, no_wrap=True)
        table.add_column("Total", justify="right", min_width=self.COL_SIZE)
        table.add_column("Used", justify="right", min_width=self.COL_SIZE)
        table.add_column("Free", justify="right", min_width=self.COL_SIZE)
        table.add_column("Usage", justify="right", min_width=8)

        for usage in usages:
            region = usage.region
            pct = usage.usage_percent
            color = "green" if pct < 70 else ("yellow" if pct < 90 else "red")

            table.add_row(
                f"[bold {color}]{region.name}[/]",
                self._fmt_size(region.length),
                f"[{color}]{self._fmt_size(usage.used_size)}[/]",
                self._fmt_size(usage.free_size),
                f"[{color}]{pct:.1f}%[/]",
            )

            if show_individual_sections:
                # List each individual section
                for section in sorted(usage.sections, key=lambda s: s.vma):
                    lma_mark = " [dim]*[/dim]" if section.has_load_address else ""
                    table.add_row(
                        f"  [dim]{section.name}{lma_mark}[/dim]",
                        f"[dim]{self._fmt_size(section.size)}[/dim]",
                        "", "", "",
                    )
            else:
                # Show aggregated sections
                section_agg = self._aggregate_sections(usage.sections)
                for sec_name, sec_size in section_agg:
                    table.add_row(
                        f"  [dim]{sec_name}[/dim]",
                        f"[dim]{self._fmt_size(sec_size)}[/dim]",
                        "", "", "",
                    )

        self.console.print(table)

    def _print_note(self):
        """Print the footnote about LMA-loaded sections."""
        self.console.print(
            "  [yellow]Note: '*' marks sections loaded from Flash to RAM[/yellow]"
        )
        self.console.print()

    # ------------------------------------------------------------------
    # Helper Methods
    # ------------------------------------------------------------------

    @staticmethod
    def _fmt_size(size: int) -> str:
        """Format byte size into a human-readable string."""
        if size >= 1024 * 1024:
            return f"{size / (1024 * 1024):.1f} MB"
        elif size >= 1024:
            return f"{size / 1024:.1f} KB"
        else:
            return f"{size} B"

    @staticmethod
    def _aggregate_sections(sections: list[Section]) -> list[tuple[str, int]]:
        """Aggregate sections with the same name and return sorted by size descending."""
        agg: dict[str, int] = {}
        for sec in sections:
            if sec.name in agg:
                agg[sec.name] += sec.size
            else:
                agg[sec.name] = sec.size
        return sorted(agg.items(), key=lambda x: x[1], reverse=True)


# ============================================================================
# File Discovery
# ============================================================================

def find_map_files(build_dir: str) -> list[str]:
    """Recursively search for all .map files under the build directory."""
    map_files = []
    for root, _dirs, files in os.walk(build_dir):
        for f in files:
            if f.endswith('.map'):
                map_files.append(os.path.join(root, f))
    return sorted(map_files)


def prioritize_map_files(map_files: list[str]) -> list[str]:
    """Sort .map files so that 'main' comes first, rest sorted alphabetically.

    The 'main' artifact must be first because the global region map builder
    (MemoryAnalyzer.build_global_region_map) uses the first entry as the
    baseline when no 'main' is found by name. All other files are sorted
    alphabetically by their base filename.
    """
    def sort_key(path: str):
        basename = os.path.splitext(os.path.basename(path))[0].lower()
        # 'main' gets priority 0; all others get priority 1 and sort by name
        if basename == 'main':
            return (0, basename)
        else:
            return (1, basename)
    return sorted(map_files, key=sort_key)


# ============================================================================
# Main Entry Point
# ============================================================================

def main():
    logging.basicConfig(
        level=logging.INFO,
        format='%(levelname)s: %(message)s',
    )

    parser = argparse.ArgumentParser(
        description='SiFli SDK Memory Usage Analyzer - Analyzes GCC .map files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python sdk_size.py project/build_sf32lb52-lcd_n16r8_hcpu
  python sdk_size.py project/build_sf32lb52-lcd_n16r8_hcpu --detail
  python sdk_size.py project/build_sf32lb52-lcd_n16r8_hcpu/main.map
  python sdk_size.py project/build_sf32lb58-lcd_a128r32n1_a1_dsi_hcpu -o report.txt
        """,
    )
    parser.add_argument(
        'path',
        help='Build directory path or single .map file path',
    )
    parser.add_argument(
        '-d', '--detail',
        action='store_true',
        help='Output detailed analysis for each .map file',
    )
    parser.add_argument(
        '-o', '--output',
        help='Save report to a file (default: print to console)',
        default=None,
    )
    parser.add_argument(
        '--no-color',
        action='store_true',
        help='Disable colored output',
    )

    args = parser.parse_args()

    # Set up Rich console; when outputting to a file, use a file-backed console
    # with no color to produce clean plain-text output.
    output_file = None
    if args.output:
        output_file = open(args.output, 'w', encoding='utf-8')
        console = Console(file=output_file, no_color=True, width=120)
    elif args.no_color:
        console = Console(no_color=True)
    else:
        console = Console()

    input_path = os.path.abspath(args.path)

    # Determine whether the input is a file or directory
    if os.path.isfile(input_path):
        if not input_path.endswith('.map'):
            logger.error("Not a .map file: %s", input_path)
            sys.exit(1)
        map_files = [input_path]
    elif os.path.isdir(input_path):
        map_files = find_map_files(input_path)
        if not map_files:
            logger.error("No .map files found under %s", input_path)
            sys.exit(1)
        map_files = prioritize_map_files(map_files)
    else:
        logger.error("Path does not exist: %s", input_path)
        sys.exit(1)

    # Parse all .map files
    map_infos: list[MapFileInfo] = []
    for f in map_files:
        try:
            parser_obj = MapFileParser(f)
            info = parser_obj.parse()
            map_infos.append(info)
        except Exception as e:
            logger.warning("Failed to parse %s: %s", f, e)

    if not map_infos:
        logger.error("No .map files were successfully parsed")
        sys.exit(1)

    # Analyze and generate report
    analyzer = MemoryAnalyzer(map_infos)
    report_gen = ReportGenerator(map_infos, analyzer, console)
    report_gen.generate(detail=args.detail)

    # Clean up file handle if writing to file
    if output_file:
        output_file.close()
        logger.info("Report saved to: %s", args.output)


if __name__ == '__main__':
    main()
