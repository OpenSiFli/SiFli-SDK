# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""Value parsers shared by the instrument modules.

Kept out of ``bt_peer.py`` and ``classic.py`` so both halves parse a reply the
same way; a difference between them would be a difference in the wire contract.
"""

from __future__ import annotations

from typing import Dict
from typing import Optional

from .errors import InstrumentProtocolError


def int_value(values: Dict[str, str], key: str, default: Optional[int] = None) -> int:
    raw = values.get(key)
    if raw is None:
        if default is None:
            raise InstrumentProtocolError(
                f"the instrument omitted {key!r}: {values}"
            )
        return default
    try:
        return int(raw)
    except ValueError as exc:
        raise InstrumentProtocolError(f"{key}={raw!r} is not an integer") from exc


def payload(values: Dict[str, str]) -> bytes:
    """Decode ``len=N data=<hex>`` and reject a length that disagrees."""
    length = int_value(values, "len")
    raw = values.get("data", "")
    try:
        data = bytes.fromhex(raw) if raw else b""
    except ValueError as exc:
        raise InstrumentProtocolError(f"data={raw!r} is not hexadecimal") from exc
    if len(data) != length:
        raise InstrumentProtocolError(
            f"the instrument said len={length} but carried {len(data)} bytes"
        )
    return data
