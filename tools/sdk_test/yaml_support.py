# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0
"""A ``SafeLoader`` that refuses duplicate mapping keys.

``yaml.safe_load`` keeps the last of two identical keys, so a typo silently
overrides an earlier entry. Both hand-written documents this framework reads --
the fixture type contract and the runner-side params file -- treat a duplicate
as a mistake to fix rather than a value to prefer, and each raises its own
exception type for it.
"""

from __future__ import annotations

from typing import Any
from typing import Callable

#: The tag of a YAML merge key (``<<``). The base loader flattens merge keys;
#: the key node itself has no scalar constructor and is not a literal key, so
#: scanning it here would fail on a document ``yaml.safe_load`` accepts.
MERGE_TAG = "tag:yaml.org,2002:merge"


def strict_safe_loader(
    yaml_module: Any, source: str, error: Callable[[str], Exception]
) -> Any:
    """Return a ``SafeLoader`` subclass that rejects duplicate mapping keys.

    The check runs for nested mappings too, because every mapping is constructed
    through ``construct_mapping``. Merge keys are skipped rather than resolved
    first: flattening would make an explicit key that overrides a merged one
    look like a duplicate, which is valid YAML.

    :param yaml_module: the imported ``yaml`` module; callers import it lazily
        so the dependency stays visible where it is used.
    :param source: what to name in the message (usually the file path).
    :param error: exception type to raise, so the caller keeps its own error
        taxonomy for a malformed document.
    """

    class _StrictSafeLoader(yaml_module.SafeLoader):
        def construct_mapping(self, node: Any, deep: bool = False) -> Any:
            if isinstance(node, yaml_module.MappingNode):
                seen = set()
                for key_node, _value_node in node.value:
                    if key_node.tag == MERGE_TAG:
                        continue
                    key = self.construct_object(key_node, deep=True)
                    try:
                        duplicated = key in seen
                    except TypeError:  # an unhashable key; the base loader reports it
                        continue
                    if duplicated:
                        raise error(
                            f"{source}: duplicate key {key!r} at line "
                            f"{key_node.start_mark.line + 1}"
                        )
                    seen.add(key)
            return super().construct_mapping(node, deep=deep)

    return _StrictSafeLoader


__all__ = ["MERGE_TAG", "strict_safe_loader"]
