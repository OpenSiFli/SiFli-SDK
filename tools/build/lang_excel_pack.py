# -*- coding:utf-8 -*-
# SPDX-FileCopyrightText: 2026 SiFli
# SPDX-License-Identifier: Apache-2.0

#!/usr/bin/env python3
"""
Convert a Solution-style multi_language_table.xlsx into SDK language sources
and external language-pack bin files.
"""

import argparse
import importlib.util
import json
import posixpath
import re
import sys
import zipfile
from collections import OrderedDict
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple
from xml.etree import ElementTree as ET


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_VERSION = "v1.0.4"
WORKBOOK_NS = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
DOC_REL_NS = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
PKG_REL_NS = "http://schemas.openxmlformats.org/package/2006/relationships"
NS = {"main": WORKBOOK_NS}

CELL_REF_RE = re.compile(r"^([A-Z]+)([0-9]+)$")
C_IDENT_RE = re.compile(r"^[A-Za-z_]\w*$")
LANG_HEADER_RE = re.compile(r"^\s*([^()]+?)\s*\(([^()]*)\)\s*$")


class LangExcelPackError(RuntimeError):
    """Raised when xlsx packing fails."""


def load_lang_bin_pack_module():
    module_path = SCRIPT_DIR / "lang_bin_pack.py"
    spec = importlib.util.spec_from_file_location("sifli_lang_bin_pack", module_path)
    if spec is None or spec.loader is None:
        raise LangExcelPackError(f"failed to load module: {module_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Convert multi_language_table.xlsx into lang_pack.h/.c and external "
            "language-pack .bin files."
        )
    )
    parser.add_argument(
        "--xlsx",
        required=True,
        help="Path to multi_language_table.xlsx.",
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        help="Directory for generated .c/.h/.bin outputs.",
    )
    parser.add_argument(
        "--sheet",
        help="Worksheet name. Default: first sheet.",
    )
    parser.add_argument(
        "--default-lang",
        help="Language stem that should appear first in lang_pack.c. Default: first language column.",
    )
    parser.add_argument(
        "--c-langs",
        help="Comma-separated language stems to generate builtin .c files for. Default: all languages.",
    )
    parser.add_argument(
        "--bin-langs",
        help="Comma-separated language stems to generate external .bin files for. Default: all languages.",
    )
    parser.add_argument(
        "--no-c",
        action="store_true",
        help="Skip generating builtin .c/.h files.",
    )
    parser.add_argument(
        "--no-bin",
        action="store_true",
        help="Skip generating external .bin files.",
    )
    parser.add_argument(
        "--version",
        default=DEFAULT_VERSION,
        help=f"Version string written into generated .bin files. Default: {DEFAULT_VERSION}",
    )
    parser.add_argument(
        "--dump-json-dir",
        help="Optional directory to dump intermediate json files.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print extra details while generating.",
    )
    return parser.parse_args()


def column_letters_to_index(column_letters: str) -> int:
    value = 0
    for ch in column_letters:
        value = value * 26 + (ord(ch) - ord("A") + 1)
    return value


def parse_cell_ref(cell_ref: str) -> Tuple[int, int]:
    match = CELL_REF_RE.match(cell_ref)
    if not match:
        raise LangExcelPackError(f"invalid cell reference: {cell_ref}")
    column_idx = column_letters_to_index(match.group(1))
    row_idx = int(match.group(2))
    return row_idx, column_idx


def parse_shared_strings(zf: zipfile.ZipFile) -> List[str]:
    path = "xl/sharedStrings.xml"
    if path not in zf.namelist():
        return []

    root = ET.fromstring(zf.read(path))
    values: List[str] = []
    for item in root.findall("main:si", NS):
        texts = [node.text or "" for node in item.findall(".//main:t", NS)]
        values.append("".join(texts))
    return values


def get_cell_text(cell: ET.Element, shared_strings: Sequence[str]) -> str:
    cell_type = cell.get("t")
    if cell_type == "inlineStr":
        texts = [node.text or "" for node in cell.findall(".//main:t", NS)]
        return "".join(texts)

    value_node = cell.find("main:v", NS)
    value = value_node.text if value_node is not None and value_node.text is not None else ""

    if cell_type == "s":
        if not value:
            return ""
        index = int(value)
        if index < 0 or index >= len(shared_strings):
            raise LangExcelPackError(f"shared string index out of range: {index}")
        return shared_strings[index]

    if cell_type in ("str", "e", "b"):
        return value

    inline_node = cell.find("main:is", NS)
    if inline_node is not None:
        texts = [node.text or "" for node in inline_node.findall(".//main:t", NS)]
        return "".join(texts)

    return value


def resolve_sheet_path(zf: zipfile.ZipFile, sheet_name: Optional[str]) -> Tuple[str, str]:
    workbook_root = ET.fromstring(zf.read("xl/workbook.xml"))
    rel_root = ET.fromstring(zf.read("xl/_rels/workbook.xml.rels"))

    sheets = workbook_root.findall("main:sheets/main:sheet", NS)
    if not sheets:
        raise LangExcelPackError("no worksheet found in workbook")

    sheet_elem = None
    if sheet_name:
        for candidate in sheets:
            if candidate.get("name") == sheet_name:
                sheet_elem = candidate
                break
        if sheet_elem is None:
            raise LangExcelPackError(f"worksheet not found: {sheet_name}")
    else:
        sheet_elem = sheets[0]

    sheet_display_name = sheet_elem.get("name") or "sheet"
    rel_id = sheet_elem.get(f"{{{DOC_REL_NS}}}id")
    if not rel_id:
        raise LangExcelPackError(f"worksheet missing relationship id: {sheet_display_name}")

    target = None
    for rel in rel_root.findall(f"{{{PKG_REL_NS}}}Relationship"):
        if rel.get("Id") == rel_id:
            target = rel.get("Target")
            break

    if not target:
        raise LangExcelPackError(f"worksheet relationship target missing: {sheet_display_name}")

    if target.startswith("/"):
        sheet_path = target.lstrip("/")
    else:
        sheet_path = posixpath.normpath(posixpath.join("xl", target))

    return sheet_display_name, sheet_path


def read_sheet_rows(xlsx_path: Path, sheet_name: Optional[str]) -> Tuple[str, OrderedDict]:
    rows: OrderedDict[int, Dict[int, str]] = OrderedDict()

    with zipfile.ZipFile(xlsx_path) as zf:
        shared_strings = parse_shared_strings(zf)
        resolved_sheet_name, sheet_path = resolve_sheet_path(zf, sheet_name)
        if sheet_path not in zf.namelist():
            raise LangExcelPackError(f"worksheet xml missing: {sheet_path}")

        sheet_root = ET.fromstring(zf.read(sheet_path))
        for row in sheet_root.findall(".//main:sheetData/main:row", NS):
            row_idx_attr = row.get("r")
            row_idx = int(row_idx_attr) if row_idx_attr else len(rows) + 1
            row_data: Dict[int, str] = {}
            next_col_idx = 1
            for cell in row.findall("main:c", NS):
                cell_ref = cell.get("r")
                if cell_ref:
                    _, col_idx = parse_cell_ref(cell_ref)
                else:
                    col_idx = next_col_idx
                row_data[col_idx] = get_cell_text(cell, shared_strings)
                next_col_idx = col_idx + 1
            rows[row_idx] = row_data

    return resolved_sheet_name, rows


def parse_lang_header(value: str) -> Tuple[str, str]:
    text = value.strip()
    if not text:
        raise LangExcelPackError("language header is empty")

    match = LANG_HEADER_RE.match(text)
    if match:
        stem = match.group(1).strip()
        locale = match.group(2).strip()
    else:
        stem = text
        locale = text

    if not C_IDENT_RE.match(stem):
        raise LangExcelPackError(f"invalid language stem '{stem}', expected C identifier style")
    if not locale:
        raise LangExcelPackError(f"invalid locale display name in header '{value}'")

    return stem, locale


def split_lang_list(value: Optional[str]) -> Optional[List[str]]:
    if value is None:
        return None
    items = [item.strip() for item in value.split(",") if item.strip()]
    return items


def normalize_selected_langs(selected: Optional[Sequence[str]]) -> Optional[List[str]]:
    if selected is None:
        return None

    if isinstance(selected, str):
        return split_lang_list(selected)

    items: List[str] = []
    for item in selected:
        text = str(item).strip()
        if text:
            items.append(text)
    return items


def validate_selected_langs(selected: Optional[Sequence[str]], all_langs: Sequence[str], label: str) -> List[str]:
    if selected is None:
        return list(all_langs)

    result: List[str] = []
    known = set(all_langs)
    for lang in selected:
        if lang not in known:
            raise LangExcelPackError(f"{label} contains unknown language stem: {lang}")
        if lang not in result:
            result.append(lang)
    return result


def load_xlsx_table(
    xlsx_path: Path,
    sheet_name: Optional[str],
) -> Tuple[str, List[Tuple[int, str, str]], List[str], Dict[str, OrderedDict], Dict[str, str]]:
    resolved_sheet_name, rows = read_sheet_rows(xlsx_path, sheet_name)
    header = rows.get(1)
    if not header:
        raise LangExcelPackError("worksheet does not contain header row")

    languages: List[Tuple[int, str, str]] = []
    seen_stems = set()
    for col_idx in sorted(idx for idx in header.keys() if idx >= 3):
        value = (header.get(col_idx) or "").strip()
        if not value:
            continue
        stem, locale = parse_lang_header(value)
        if stem in seen_stems:
            raise LangExcelPackError(f"duplicate language stem in header: {stem}")
        languages.append((col_idx, stem, locale))
        seen_stems.add(stem)

    if not languages:
        raise LangExcelPackError("no language columns found from column C onward")

    keys: List[str] = []
    translations: Dict[str, OrderedDict] = {stem: OrderedDict() for _, stem, _ in languages}
    locales: Dict[str, str] = {stem: locale for _, stem, locale in languages}
    seen_keys = set()

    for row_idx in sorted(idx for idx in rows.keys() if idx >= 2):
        row = rows[row_idx]
        key = (row.get(1) or "").strip()
        desc = row.get(2) or ""
        lang_values = [(row.get(col_idx) or "") for col_idx, _, _ in languages]

        if not key and not desc and not any(value != "" for value in lang_values):
            continue

        if not key:
            raise LangExcelPackError(f"row {row_idx}: key column A is empty")
        if not C_IDENT_RE.match(key):
            raise LangExcelPackError(f"row {row_idx}: invalid key '{key}'")
        if key in seen_keys:
            raise LangExcelPackError(f"row {row_idx}: duplicate key '{key}'")

        seen_keys.add(key)
        keys.append(key)

        for col_idx, stem, _ in languages:
            value = row.get(col_idx) or ""
            if value == "":
                raise LangExcelPackError(
                    f"row {row_idx}: translation for key '{key}' in language '{stem}' is empty"
                )
            translations[stem][key] = value

    if not keys:
        raise LangExcelPackError("worksheet does not contain translation rows")

    return resolved_sheet_name, languages, keys, translations, locales


def c_string_literal(value: str) -> str:
    return json.dumps(value, ensure_ascii=False)


def write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as f:
        f.write(content)


def generate_lang_pack_h(output_dir: Path, keys: Sequence[str]) -> Path:
    lines = [
        "#ifndef __lang_pack__",
        "#define __lang_pack__",
        "",
        "typedef struct",
        "{",
    ]
    for key in keys:
        lines.append(f"    lv_i18n_phrase_t {key};")
    lines.extend(
        [
            "} lang_translation_t;",
            "",
            "extern const lv_i18n_lang_t * const lv_i18n_lang_pack[];",
            "",
            "#endif /* __lang_pack__ */",
            "",
        ]
    )

    output_path = output_dir / "lang_pack.h"
    write_text(output_path, "\n".join(lines))
    return output_path


def generate_lang_c(
    output_dir: Path,
    stem: str,
    locale: str,
    keys: Sequence[str],
    translations: Dict[str, str],
) -> Path:
    lines = [
        '#include "lv_ext_resource_manager.h"',
        "",
        f"const lang_translation_t {stem}_lang_translation = ",
        "{",
    ]

    for key in keys:
        lines.append(
            f"    .{key} = {{{c_string_literal(key)}, {c_string_literal(translations[key])}, {{0}}}},"
        )

    lines.extend(
        [
            "};",
            "",
            f"const lv_i18n_lang_t {stem}_lang = ",
            "{",
            f"    .locale = {c_string_literal(locale)},",
            f"    .translation = &{stem}_lang_translation,",
            "};",
            "",
        ]
    )

    output_path = output_dir / f"{stem}.c"
    write_text(output_path, "\n".join(lines))
    return output_path


def generate_lang_pack_c(output_dir: Path, langs: Sequence[str], default_lang: str) -> Path:
    ordered_langs = [lang for lang in langs if lang == default_lang] + [
        lang for lang in langs if lang != default_lang
    ]

    lines = ['#include "lv_ext_resource_manager.h"', "", ""]
    for lang in ordered_langs:
        lines.append(f"extern const lv_i18n_lang_t {lang}_lang;")

    lines.extend(["", "const lv_i18n_lang_t * const lv_i18n_lang_pack[] = ", "{"])
    for lang in ordered_langs:
        lines.append(f"    &{lang}_lang,")
    lines.extend(["    NULL //end mark", "};", ""])

    output_path = output_dir / "lang_pack.c"
    write_text(output_path, "\n".join(lines))
    return output_path


def dump_intermediate_json(
    output_dir: Path,
    langs: Sequence[str],
    translations: Dict[str, OrderedDict],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    for lang in langs:
        json_path = output_dir / f"{lang}.json"
        write_text(json_path, json.dumps(translations[lang], ensure_ascii=False, indent=2) + "\n")


def main() -> int:
    args = parse_args()

    if args.no_c and args.no_bin:
        raise LangExcelPackError("nothing to do: both --no-c and --no-bin were specified")

    xlsx_path = Path(args.xlsx).resolve()
    if not xlsx_path.is_file():
        raise LangExcelPackError(f"xlsx file not found: {xlsx_path}")

    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    sheet_name, languages, keys, translations, locales = load_xlsx_table(xlsx_path, args.sheet)
    lang_stems = [stem for _, stem, _ in languages]

    default_lang = args.default_lang or lang_stems[0]
    if default_lang not in locales:
        raise LangExcelPackError(f"default language not found in workbook: {default_lang}")

    c_langs = validate_selected_langs(normalize_selected_langs(args.c_langs), lang_stems, "--c-langs")
    bin_langs = validate_selected_langs(normalize_selected_langs(args.bin_langs), lang_stems, "--bin-langs")

    generated_c: List[Path] = []
    generated_bin: List[Path] = []

    if not args.no_c:
        generate_lang_pack_h(output_dir, keys)
        for lang in c_langs:
            generated_c.append(generate_lang_c(output_dir, lang, locales[lang], keys, translations[lang]))
        generated_c.append(generate_lang_pack_c(output_dir, c_langs, default_lang))

    if not args.no_bin:
        lang_bin_pack = load_lang_bin_pack_module()
        for lang in bin_langs:
            blob = lang_bin_pack.build_lang_bin_blob(
                keys,
                translations[lang],
                args.version,
                lang,
                locales[lang],
                f"{xlsx_path.name}:{lang}",
            )
            bin_path = output_dir / f"{lang}.bin"
            bin_path.write_bytes(blob)
            generated_bin.append(bin_path)

    if args.dump_json_dir:
        dump_intermediate_json(
            Path(args.dump_json_dir).resolve(),
            lang_stems,
            translations,
        )

    print(f"Workbook: {xlsx_path}")
    print(f"Worksheet: {sheet_name}")
    print(f"Keys: {len(keys)}")
    print(f"Languages: {', '.join(lang_stems)}")
    if not args.no_c:
        print(f"Generated builtin sources: {len(generated_c)} file(s) in {output_dir}")
    if not args.no_bin:
        print(f"Generated external bins: {len(generated_bin)} file(s) in {output_dir}")
        print(f"Pack version: {args.version}")
    if args.verbose:
        for _, stem, locale in languages:
            print(f"  {stem} -> locale={locale}")

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except LangExcelPackError as exc:
        print(f"[error] {exc}", file=sys.stderr)
        sys.exit(1)
