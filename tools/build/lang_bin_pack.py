# -*- coding:utf-8 -*-
# SPDX-FileCopyrightText: 2026 SiFli
# SPDX-License-Identifier: Apache-2.0

#!/usr/bin/env python3
"""
Generate external language-pack .bin files that are compatible with
middleware/lvgl/lv_ext_resouce/app_lang.c.

This script is intended to replace the Butterfli-side "external language pack"
packing step for SDK projects that already maintain translations as
`strings/*.json`.

Recommended workflow:
1. Build or generate `lang_pack.h` from the same `strings/*.json`.
2. Run this script with `--header` pointing to that exact `lang_pack.h`.
3. Copy the generated `*.bin` files into the target installer directory.

If `--header` is omitted, the script will generate a temporary `lang_pack.h`
using the current SDK `sdk_resource.py` logic and use that order for packing.
Each external pack is generated directly from its json filename stem, so no
extra metadata or translation-list manifest is required.
"""

import argparse
import importlib.util
import json
import re
import struct
import sys
import tempfile
import shutil
from pathlib import Path
from typing import Dict, Iterable, List, Tuple


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_VERSION = "v1.0.4"
LANG_FIELD_RE = re.compile(r"^\s*lv_i18n_phrase_t\s+([A-Za-z_]\w*)\s*;")


class LangBinPackError(RuntimeError):
    """Raised when language-pack generation fails."""


def load_sdk_resource_module():
    resource_path = SCRIPT_DIR / "sdk_resource.py"
    spec = importlib.util.spec_from_file_location("sifli_resource", resource_path)
    if spec is None or spec.loader is None:
        raise LangBinPackError(f"failed to load resource module: {resource_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate app_lang.c-compatible external language-pack .bin files."
    )
    parser.add_argument(
        "--strings-dir",
        required=True,
        help="Directory containing language JSON files such as en_us.json/zh_cn.json.",
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        help="Directory for generated .bin files.",
    )
    parser.add_argument(
        "--header",
        help=(
            "Path to generated lang_pack.h/resource.h. "
            "If omitted, a temporary lang_pack.h will be generated from the JSON files."
        ),
    )
    parser.add_argument(
        "--default-lang",
        default="en_us",
        help="Default language used only when a temporary lang_pack.h must be generated.",
    )
    parser.add_argument(
        "--version",
        default=DEFAULT_VERSION,
        help=f"Default pack version string written into each .bin header. Default: {DEFAULT_VERSION}",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print extra details while generating.",
    )
    return parser.parse_args()


def generate_temp_header(
    strings_dir: Path,
    default_lang: str,
) -> Tuple[tempfile.TemporaryDirectory, Path]:
    resource = load_sdk_resource_module()
    resource.ns = True
    resource.default_language = default_lang
    resource.resource_db = {"images": set(), "strings": []}

    temp_dir = tempfile.TemporaryDirectory(prefix="sifli_lang_bin_")
    out_dir = Path(temp_dir.name)
    temp_strings_dir = out_dir / "strings"
    temp_strings_dir.mkdir(parents=True, exist_ok=True)

    for json_path in strings_dir.glob("*.json"):
        shutil.copyfile(json_path, temp_strings_dir / json_path.name)

    resource.GenerateStrRes(str(temp_strings_dir), str(out_dir))

    header_path = out_dir / "lang_pack.h"
    if not header_path.is_file():
        raise LangBinPackError(f"temporary lang_pack.h was not generated: {header_path}")

    return temp_dir, header_path


def parse_key_order(header_path: Path) -> List[str]:
    keys: List[str] = []
    with header_path.open("r", encoding="utf-8") as f:
        for line in f:
            match = LANG_FIELD_RE.match(line)
            if match:
                keys.append(match.group(1))

    if not keys:
        raise LangBinPackError(f"no translation keys found in header: {header_path}")

    return keys


def list_language_jsons(strings_dir: Path) -> List[Path]:
    files = []
    for path in sorted(strings_dir.glob("*.json")):
        files.append(path)

    if not files:
        raise LangBinPackError(f"no language json files found in {strings_dir}")

    return files


def load_language_translations(json_path: Path) -> Dict[str, str]:
    with json_path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    if not isinstance(data, dict):
        raise LangBinPackError(f"language json root must be an object: {json_path}")

    translations: Dict[str, str] = {}
    for key, value in data.items():
        if not isinstance(key, str):
            raise LangBinPackError(f"language key must be string in {json_path}")
        if not isinstance(value, str):
            raise LangBinPackError(
                f"translation '{key}' must be string in {json_path}, got {type(value).__name__}"
            )
        translations[key] = value

    return translations


def encode_c_string(value: str) -> bytes:
    return value.encode("utf-8") + b"\x00"


def build_lang_bin_blob(
    key_order: Iterable[str],
    translations: Dict[str, str],
    version: str,
    pgm_name: str,
    locale: str,
    source_name: str,
) -> bytes:
    ordered_keys = list(key_order)
    missing = [key for key in ordered_keys if key not in translations]
    if missing:
        raise LangBinPackError(
            f"{source_name}: missing translations for keys: {', '.join(missing)}"
        )

    blob = bytearray()
    blob.extend(struct.pack("<I", len(ordered_keys)))
    blob.extend(encode_c_string(version))
    blob.extend(encode_c_string(pgm_name))
    blob.extend(encode_c_string(locale))
    for key in ordered_keys:
        blob.extend(encode_c_string(translations[key]))
    return bytes(blob)


def main() -> int:
    args = parse_args()

    strings_dir = Path(args.strings_dir).resolve()
    if not strings_dir.is_dir():
        raise LangBinPackError(f"strings directory not found: {strings_dir}")

    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    tmp_ctx = None
    try:
        if args.header:
            header_path = Path(args.header).resolve()
            if not header_path.is_file():
                raise LangBinPackError(f"header file not found: {header_path}")
        else:
            tmp_ctx, header_path = generate_temp_header(strings_dir, args.default_lang)

        key_order = parse_key_order(header_path)
        json_files = list_language_jsons(strings_dir)

        generated = []
        for json_path in json_files:
            stem = json_path.stem
            translations = load_language_translations(json_path)
            pgm_name = stem
            locale = stem
            version = args.version

            extra_keys = sorted(set(translations.keys()) - set(key_order))
            if extra_keys and args.verbose:
                print(
                    f"[warn] {json_path.name}: unused keys not present in header: "
                    f"{', '.join(extra_keys)}"
                )

            blob = build_lang_bin_blob(
                key_order, translations, version, pgm_name, locale, json_path.name
            )
            bin_path = output_dir / f"{pgm_name}.bin"
            with bin_path.open("wb") as f:
                f.write(blob)

            generated.append((json_path.name, bin_path.name, locale, version, len(key_order)))

        print(f"Generated {len(generated)} language-pack bin file(s) in {output_dir}")
        for source_name, bin_name, locale, version, key_count in generated:
            print(
                f"  {source_name} -> {bin_name} "
                f"(locale={locale}, version={version}, keys={key_count})"
            )
        print(f"Using key schema: {header_path}")
        return 0
    finally:
        if tmp_ctx is not None:
            tmp_ctx.cleanup()


if __name__ == "__main__":
    try:
        sys.exit(main())
    except LangBinPackError as exc:
        print(f"[error] {exc}", file=sys.stderr)
        sys.exit(1)
